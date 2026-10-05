#pragma once
// ---------------------------------------------------------------------------
// 双视线三角化（纯数学，无 Qt 依赖，可单测）。
//
// 只处理"恰好2台相机看同一个点"这个最简单也最常见的场景：两条视线在
// 3D 空间里理想情况下应该相交，噪声存在时用"两条直线的最近点"这个闭式解
// 代替严格求交——不需要 SVD，几行线性代数就能解出来。
//
// 等以后要支持 N(>2) 台相机联合三角化多个标记点（比如手掌背 4 点刚体），
// 才需要升级成 solve_calibration.py 里那种基于 DLT+SVD 的最小二乘解法；
// 现在这一步先把"两台相机、一个球"这个最基础的环节验证死，不提前引入
// 用不上的复杂度（对应 README §12.3 的依赖隔离思路：能不引入重依赖
// 就不引入，等真正需要了再升级）。
//
// 坐标系约定与 Calibration.hpp / CalibResultView 保持一致：
//   世界->相机：Xc = R*Xw + t（R 行主序 3x3）
//   相机中心（世界系）：C = -R^T t
// ---------------------------------------------------------------------------
#include "calib/Calibration.hpp"
#include <array>

namespace mocap {

struct TriangulateResult {
    std::array<double, 3> point{0, 0, 0};  // 三角化出的世界坐标（mm）
    double residual = -1.0;                // 两条视线最近点间距（mm）
    bool   valid = false;                  // 是否得到可信结果
};

// 对畸变的像素坐标做去畸变+归一化。输入 (px,py) 是原始像素坐标；
// 输出 (nx,ny) 是去畸变后的归一化坐标——相机坐标系下 z=1 平面上的 x,y，
// 也就是说 (nx,ny,1) 就是这条视线在相机坐标系下的方向向量（未归一化长度）。
// 内部用定点迭代求逆畸变模型（与 OpenCV::undistortPoints 在不传 R/P 时的
// 内部算法一致），不依赖 OpenCV/Eigen。
void undistortNormalize(const CameraIntrinsics& intr, double px, double py,
                         double& nx, double& ny);

// 相机中心（世界系）：C = -R^T t。
std::array<double, 3> cameraCenterWorld(const CameraExtrinsics& extr);

// 把相机坐标系下的方向 (nx,ny,1) 转换成世界坐标系下的单位方向向量。
std::array<double, 3> rayDirectionWorld(const CameraExtrinsics& extr, double nx, double ny);

// 两条视线（各自的起点 + 单位方向）的最近点，取两条线各自最近点的中点
// 作为三角化结果，residual 记录这两个最近点之间的距离——这个值本质上是
// "两台相机看到的是否一致"的置信度指标：正常应该是几毫米量级，如果飙到
// 几十/上百毫米，大概率是标定不准，或者两台相机其实看错了目标（比如把
// 反光、误检测当成了同一颗球）。两条视线（近似）平行时问题在几何上是
// 退化的（基线夹角太小，前后方向的误差会被急剧放大），这里直接返回
// valid=false，不硬凑一个不可信的数值。
TriangulateResult closestPointBetweenRays(
    const std::array<double, 3>& c0, const std::array<double, 3>& d0,
    const std::array<double, 3>& c1, const std::array<double, 3>& d1);

// 便捷封装：直接从两台相机的标定参数 + 像素观测，三角化出世界坐标。
// 若任一相机未标定（intrinsics/extrinsics 的 valid==false），直接返回
// valid=false，不产出虚假结果。
TriangulateResult triangulateTwoViews(
    const CameraIntrinsics& intr0, const CameraExtrinsics& extr0, double px0, double py0,
    const CameraIntrinsics& intr1, const CameraExtrinsics& extr1, double px1, double py1);

// ---- N 视图三角化（N>=2 自适应）----
// 一台相机对该点的一次观测：这台相机的标定 + 像素坐标。
struct ViewObservation {
    const CameraIntrinsics* intr;
    const CameraExtrinsics* extr;
    double px, py;
};

// 任意 N(>=2) 台相机联合三角化。用 DLT（每台相机贡献2行约束）+ SVD 求
// 齐次最小二乘解——这是 solve_calibration.py 世界对齐那段手写 DLT 的
// C++ 实时版。相比两视线最近点法，DLT 天然支持任意台数：多一台就多两行
// 约束，看到的相机越多，超定程度越高、抗噪越好，不需要为不同台数写不同
// 分支。
//
// 只会用 valid==true（已标定）的观测；有效观测不足2台直接返回 valid=false。
// residual 这里定义为"解出的3D点重投影回各相机的平均像素误差换算到 mm 的
// 近似量"——为了与两视图版本的 residual（视线最近点间距）语义大致可比，
// 用一个基于视线偏离的度量，具体见 .cpp 注释。
TriangulateResult triangulateMultiView(const ViewObservation* obs, int count);

// N 视图三角化 + 鲁棒剔除坏视角。先用全部视角解一次，算出每个视角到
// 解出点的垂距；把明显偏大的视角（垂距 > median + k*MAD，稳健离群判据）
// 剔掉，用剩下一致的视角重新解一次。实时追踪时，某一台相机偶发的检测
// 噪点/时间戳偏差会污染那一帧的结果、且随机出现在不同帧不同相机上——
// 表现出来就是"有时候抖有时候不抖"。这个剔除让每一帧都只用互相一致的
// 视角求解，把这类随机污染压下去。
//   - 至少保留 minKeep 个视角（默认2，三角化的下限）；如果剔完不足
//     minKeep，退回用全部视角的结果（不是失败，只是这帧无法进一步净化）。
//   - droppedMask（可选，长度 count）：回填每个视角是否被剔除，供上层
//     统计"哪台相机经常被剔"（可能是这台的标定/检测有系统性问题）。
TriangulateResult triangulateMultiViewRobust(const ViewObservation* obs, int count,
                                              int minKeep = 2, double madScale = 3.0,
                                              bool* droppedMask = nullptr);

// ---- 软加权鲁棒三角化(IRLS，迭代重加权最小二乘) ----
// triangulateMultiViewRobust() 的"留/弃"是离散决策：某台相机的垂距一旦
// 跨过阈值，从"参与解算"瞬间变成"完全不参与"，解出来的3D点会有一个
// 台阶——这在多相机覆盖范围有重叠边界的场景里几乎必然发生：物体移动
// 经过某个位置时，恰好是某台相机的重投影质量跨过阈值的地方，坐标就在
// 那个位置固定地跳一下(悬崖/小抖动)，跟检测噪声完全是两回事，硬阈值
// 越"聪明"(剔得越干净)，这个台阶反而越明显。
//
// 这个函数不做"留/弃"二选一，给每台相机一个连续权重(Tukey biweight)：
// 垂距很小时权重接近1(几乎全信)，垂距增大权重平滑降到0(不是骤降到0)，
// 直接影响解出的3D点会跟着连续变化，不会有台阶——这是稳健统计里处理
// "离散剔除导致输出不连续"这类问题的标准解法，不是这个项目自己发明的
// 技巧。
//   - 每轮：用当前权重解一次加权DLT，用新的垂距重新算权重，迭代到收敛
//     (固定轮数，实测4轮足够，垂距量级下几乎不再变化)。
//   - c(权重函数的"截止点")取跟 triangulateMultiViewRobust 同一套判据
//     (统计阈值 与 绝对阈值 取更紧的那个)，两个函数用的是同一套"什么算
//     离群"的标准，只是硬剔除 vs 软降权的区别，不是另起一套新逻辑。
//   - weightsOut(可选，长度count)：回填每个视角最终收敛到的权重，供UI
//     展示"这台相机现在的可信度"，不再是非黑即白的"用/不用"。
TriangulateResult triangulateMultiViewRobustSoft(const ViewObservation* obs, int count,
                                                  double madScale = 3.0,
                                                  double* weightsOut = nullptr);

} // namespace mocap
