#pragma once
// ---------------------------------------------------------------------------
// 极线几何（多点对应性的数学地基）—— 纯数学，零 Qt/第三方依赖，可单测。
//
// 多点动捕的核心难题是"对应性"：cam0 看到 m 个反光点、cam1 看到 n 个，
// 谁对谁？极线约束把这个 m×n 的搜索空间砍掉一维——cam0 里一个点对应 cam1
// 里的点，必然落在一条极线附近。本文件提供两件工具：
//
//   1. essentialFromRelativePose：由两台相机的相对位姿算本质矩阵 E。
//      E 只跟相机间相对位姿有关、跟场景无关，所以每对相机算一次、缓存复用
//      （相机不动 E 就是常量），这是性能上的关键——不每帧重算。
//
//   2. sampsonDistance：衡量"cam1 的点 b 偏离 cam0 的点 a 的极线多远"。
//      不用原始代数值 b^T E a（那个量纲不干净、不同位置尺度不一致），用
//      Sampson 距离（一阶几何误差近似），它才是真正跟"像素偏差"线性相关、
//      可以设统一阈值的量。这是"能凑合"和"准"的分水岭。
//
// 坐标约定（与 Triangulation.hpp / Calibration.hpp 严格一致）
//   世界->相机：Xc = R * Xw + t（R 行主序 3x3）
//   本文件全程在"去畸变归一化坐标"里工作：一个 2D 观测 (px,py) 先经
//   undistortNormalize 得到 (nx,ny)，齐次形式 (nx,ny,1) 就是这条视线在
//   相机坐标系下的方向。极线约束 b^T E a = 0 里的 a,b 就是这种归一化齐次
//   坐标，不是原始像素——这点必须统一，否则 E 的定义对不上。
// ---------------------------------------------------------------------------
#include <array>
#include <cmath>

namespace mocap {

// 3x3 行主序矩阵（本文件内部用，名字加 Epi 前缀避免污染 mocap 命名空间）。
using EpiMat3 = std::array<double, 9>;
using EpiVec3 = std::array<double, 3>;

namespace epi_detail {

inline EpiMat3 matmul3(const EpiMat3& A, const EpiMat3& B) {
    EpiMat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += A[i * 3 + k] * B[k * 3 + j];
            C[i * 3 + j] = s;
        }
    return C;
}

inline EpiMat3 transpose3(const EpiMat3& A) {
    return { A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8] };
}

// 反对称矩阵 [v]_x：使得 [v]_x * u = v × u。
inline EpiMat3 skew(const EpiVec3& v) {
    return {    0.0, -v[2],  v[1],
             v[2],    0.0, -v[0],
            -v[1],  v[0],    0.0 };
}

inline EpiVec3 matvec3(const EpiMat3& A, const EpiVec3& v) {
    return { A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
             A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
             A[6] * v[0] + A[7] * v[1] + A[8] * v[2] };
}

} // namespace epi_detail

// 由两台相机各自的世界->相机位姿 (R0,t0),(R1,t1) 算本质矩阵 E，使得
// 对同一 3D 点的两条归一化视线 a(相机0系)、b(相机1系) 满足 b^T E a = 0。
//
// 推导：设相对位姿把相机0系变换到相机1系：X1 = Rrel * X0 + trel，
//   Rrel = R1 * R0^T,  trel = t1 - Rrel * t0。
// 本质矩阵 E = [trel]_x * Rrel。
//
// R0/R1 传行主序 9 元素数组，t0/t1 传 3 元素。相机固定时对每对相机算一次即可。
inline EpiMat3 essentialFromRelativePose(
        const EpiMat3& R0, const EpiVec3& t0,
        const EpiMat3& R1, const EpiVec3& t1) {
    using namespace epi_detail;
    const EpiMat3 R0t = transpose3(R0);
    const EpiMat3 Rrel = matmul3(R1, R0t);             // R1 * R0^T
    const EpiVec3 Rt0 = matvec3(Rrel, t0);
    const EpiVec3 trel = { t1[0] - Rt0[0], t1[1] - Rt0[1], t1[2] - Rt0[2] };
    return matmul3(skew(trel), Rrel);                  // [trel]_x * Rrel
}

// Sampson 距离（的平方）：衡量归一化点 a(相机0)、b(相机1) 满足极线约束
// b^T E a = 0 的几何偏差。a,b 传去畸变归一化坐标 (nx,ny)，函数内部补齐次 1。
//
//   d^2 = (b^T E a)^2 / ( (Ea)_x^2 + (Ea)_y^2 + (E^T b)_x^2 + (E^T b)_y^2 )
//
// 返回值量纲是"归一化坐标下的距离平方"——乘以焦距可换算成像素。对应性剪枝
// 时设一个阈值（对应几个像素），超了就判这两个点不可能是同一物理点。
inline double sampsonDistanceSq(const EpiMat3& E,
                                double ax, double ay,
                                double bx, double by) {
    using namespace epi_detail;
    const EpiVec3 a = { ax, ay, 1.0 };
    const EpiVec3 b = { bx, by, 1.0 };

    const EpiVec3 Ea = matvec3(E, a);                    // E a
    const EpiMat3 Et = transpose3(E);
    const EpiVec3 Etb = matvec3(Et, b);                  // E^T b

    // 代数残差 b^T E a
    const double r = b[0] * Ea[0] + b[1] * Ea[1] + b[2] * Ea[2];

    const double denom = Ea[0] * Ea[0] + Ea[1] * Ea[1]
                       + Etb[0] * Etb[0] + Etb[1] * Etb[1];
    if (denom < 1e-20) return 1e18;   // 退化（两视线几乎平行/重合），判为不匹配
    return (r * r) / denom;
}

// 便捷版：直接返回 Sampson 距离（非平方），单位同归一化坐标。
inline double sampsonDistance(const EpiMat3& E,
                              double ax, double ay, double bx, double by) {
    return std::sqrt(sampsonDistanceSq(E, ax, ay, bx, by));
}

} // namespace mocap
