#pragma once
// ---------------------------------------------------------------------------
// 轴系对齐——纯数学，零 Qt 依赖。
//
// 解决的问题：HandRigidAutoCalib.hpp 重建出的手背局部模板，只保证内部
// 距离矩阵和手性(chirality)正确，绝对朝向是任意的一个合法旋转——MDS
// 数学上就是这样，见该文件顶部注释。但 hand/HandModel.hpp 的全部公式
// （屈曲绕+Y轴、外展绕+Z轴、+X指手指方向、+Y指拇指侧）都假设了一个具体
// 的坐标约定，FingerKinematicCalib.hpp 拟合出的锚点/连杆/角度也是在"喂给
// 它的观测所在的那个坐标系"下有意义的。如果不对齐，标定出来的东西在数值
// 上依然自洽(能正确重建marker位置)，但"正的屈曲角"可能对应现实里的伸展、
// 关节限位表(jointLimits())会对不上号、骨架图调试UI画出来的手会不对劲——
// 所有假设了轴方向物理意义的地方都会出问题。
//
// 做法：不猜，用手本身的几何结构算出两个物理方向；
//   +X(远端/指尖方向)：手背刚体质心 -> 5根手指锚点均值 的方向，因为手指
//                      总是从手背朝外伸展。
//   +Y(桡侧/拇指方向)：先要认出哪根链是拇指——用"其余4指(食/中/无名/小)
//                      的锚点在垂直于+X的平面上大致排成一条线，拇指的
//                      锚点明显偏离这条线"这个几何事实，做留一法(leave-
//                      one-out)：分别假设每根链是拇指、排除它之后用剩下
//                      4个锚点拟合一条直线，看被排除的那个锚点离这条线
//                      多远——距离最大的那个就是拇指(因为拇指打破共线
//                      模式最明显，其余4指才是真正排成一排的那些)。
//                      认出拇指后，+Y := 质心指向拇指锚点的方向，正交化
//                      去掉+X分量。
//   +Z：叉乘 X×Y，自动补全成右手系——不引入新的手性判断，只是在已经由
//       HandRigidAutoCalib.hpp 的Kabsch-镜像测试固定过手性的那个3D嵌入
//       里，挑一组新的正交基，这本身是一次旋转（不是镜像），不会破坏
//       已经定好的手性。
//
// 诚实失败：5根手指锚点数量不对、拇指判别没有明显区分度（比如5个点几乎
// 共面共线，选不出"明显偏离"的那一个）都在 message 里报出来，不悍自选。
// ---------------------------------------------------------------------------
#include "estimate/PointIEKF.hpp"   // Vec3
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <algorithm>

namespace mocap {

using Mat3Flat = std::array<double, 9>;   // 行主序

struct AxisAlignmentResult {
    bool valid = false;
    Mat3Flat rotation{1,0,0, 0,1,0, 0,0,1};   // new = rotation * (old - origin)
    Vec3 origin{0,0,0};
    int thumbAnchorIndex = -1;   // 输入fingerAnchors里，哪个下标被判定为拇指
    double thumbOutlierScore = 0.0;   // 诊断量：拇指锚点离"其余4指共线拟合"的距离，越大越可信
    std::string message;
};

namespace axis_align_detail {

inline Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
inline Vec3 add(const Vec3& a, const Vec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
inline Vec3 scale(const Vec3& a, double s) { return {a[0]*s, a[1]*s, a[2]*s}; }
inline double dot(const Vec3& a, const Vec3& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return { a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0] };
}
inline double norm(const Vec3& a) { return std::sqrt(dot(a,a)); }
inline Vec3 normalize(const Vec3& a) { const double n = norm(a); return n>1e-9 ? scale(a, 1.0/n) : Vec3{1,0,0}; }

// 给定一批点(假设已经大致共面/共线)，用主成分方向(最大方差方向)拟合一条
// 过质心的直线，返回(质心, 方向单位向量)。3点以内直接退化处理。
inline void fitLine(const std::vector<Vec3>& pts, Vec3& centroid, Vec3& dir) {
    centroid = {0,0,0};
    for (auto& p : pts) centroid = add(centroid, scale(p, 1.0/double(pts.size())));

    // 3x3协方差矩阵的最大特征向量——点数很少(3~4个)，用幂迭代就够，
    // 不需要引入通用特征分解。
    double C[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (auto& p : pts) {
        const Vec3 d = sub(p, centroid);
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) C[i][j] += d[size_t(i)]*d[size_t(j)];
    }
    Vec3 v{1,0.5,0.3};
    for (int iter=0; iter<50; ++iter) {
        Vec3 nv{ C[0][0]*v[0]+C[0][1]*v[1]+C[0][2]*v[2],
                 C[1][0]*v[0]+C[1][1]*v[1]+C[1][2]*v[2],
                 C[2][0]*v[0]+C[2][1]*v[1]+C[2][2]*v[2] };
        v = normalize(nv);
    }
    dir = v;
}

inline double pointToLineDist(const Vec3& p, const Vec3& lineCentroid, const Vec3& lineDir) {
    const Vec3 d = sub(p, lineCentroid);
    const double proj = dot(d, lineDir);
    const Vec3 closest = add(lineCentroid, scale(lineDir, proj));
    return norm(sub(p, closest));
}

} // namespace axis_align_detail

// wristRigidTemplate：手背刚体局部模板点(通常5个，来自
// HandRigidAutoCalib.hpp 的 RigidClusterCandidate::localTemplate)。
// fingerAnchors：5根手指链的锚点(同一局部系下，来自
// FingerKinematicCalib.hpp 拟合结果的 params.anchor，顺序无所谓)。
inline AxisAlignmentResult computeAxisAlignment(const std::vector<Vec3>& wristRigidTemplate,
                                                const std::vector<Vec3>& fingerAnchors) {
    using namespace axis_align_detail;
    AxisAlignmentResult out;

    if (wristRigidTemplate.size() < 3) {
        out.message = "手背刚体模板点数不足3个，无法定义质心/坐标系。";
        return out;
    }
    if (fingerAnchors.size() != 5) {
        out.message = "手指链锚点数量应为5个(实际" + std::to_string(fingerAnchors.size()) + "个)，无法判别拇指。";
        return out;
    }

    Vec3 wristCentroid{0,0,0};
    for (auto& p : wristRigidTemplate) wristCentroid = add(wristCentroid, scale(p, 1.0/double(wristRigidTemplate.size())));

    // +X：质心指向5指锚点均值的方向。
    Vec3 anchorMean{0,0,0};
    for (auto& a : fingerAnchors) anchorMean = add(anchorMean, scale(a, 0.2));
    const Vec3 xAxis = normalize(sub(anchorMean, wristCentroid));

    // 把5个锚点投影到垂直于+X的平面(相对质心)。
    std::vector<Vec3> perp(5);
    for (size_t i=0;i<5;++i) {
        const Vec3 d = sub(fingerAnchors[i], wristCentroid);
        const double alongX = dot(d, xAxis);
        perp[i] = sub(d, scale(xAxis, alongX));
    }

    // 留一法找拇指：对每个候选i，排除它、用剩下4个点拟合一条直线，量化
    // "这4个点本身共线拟合得有多好"(4点各自到这条线的距离平方和)——
    // 排除掉真正的拇指后，剩下4指应该拟合得很好(残差很小)；排除掉任何
    // 一根正常手指，剩下的4点(3指+拇指)里还混着离群点，拟合优度明显更
    // 差。用"剩4点拟合优度"而不是"被排除点离线多远"来判别，是因为后者
    // 在剩下4点本身就不共线(混着拇指)时，拟合出来的线被拇指带偏，判据
    // 不可靠——这是本文件调试阶段实测踩到的一个坑，不是假设。
    int bestIdx = -1;
    double bestResidual = 1e18;
    std::vector<double> residuals(5, 0.0);
    for (int i=0;i<5;++i) {
        std::vector<Vec3> rest;
        for (int j=0;j<5;++j) if (j!=i) rest.push_back(perp[size_t(j)]);
        Vec3 lc, ld;
        fitLine(rest, lc, ld);
        double sumSq = 0.0;
        for (auto& p : rest) { const double d = pointToLineDist(p, lc, ld); sumSq += d*d; }
        residuals[size_t(i)] = sumSq;
        if (sumSq < bestResidual) { bestResidual = sumSq; bestIdx = i; }
    }

    // 区分度检查：排除拇指后的拟合优度应该明显好于排除任何其它手指——
    // 用"次佳排除方案的残差 / 最佳排除方案的残差"这个比值衡量，比值不够
    // 大说明5个点本身就已经接近共线共面，选不出真正的拇指。
    double secondBestResidual = 1e18;
    for (int i=0;i<5;++i) if (i!=bestIdx && residuals[size_t(i)]<secondBestResidual) secondBestResidual = residuals[size_t(i)];
    const double bestScore = std::sqrt(std::max(0.0, secondBestResidual - bestResidual));   // 用残差差值的平方根量纲对齐mm，供诊断展示

    if (secondBestResidual < bestResidual * 2.5 || bestResidual > 200.0) {
        out.message = "5根手指锚点的共线模式区分度不够(排除最佳候选后剩余残差" + std::to_string(bestResidual) +
                     "mm^2，排除次佳候选后残差" + std::to_string(secondBestResidual) +
                     "mm^2)，无法可靠判别哪根是拇指——检查分组/标定结果是否本身有误。";
        return out;
    }

    out.thumbAnchorIndex = bestIdx;
    out.thumbOutlierScore = bestScore;

    // +Y：质心指向拇指锚点的方向，正交化去掉+X分量。
    Vec3 yAxis = perp[size_t(bestIdx)];   // 已经正交于X(perp是投影到垂直X平面的结果)
    if (norm(yAxis) < 1e-6) {
        out.message = "拇指锚点投影后跟其余4指几乎重合在+X轴上，无法定义+Y方向。";
        return out;
    }
    yAxis = normalize(yAxis);

    const Vec3 zAxis = normalize(cross(xAxis, yAxis));
    // 重新正交化Y(叉乘两次保证严格正交，抵消数值误差)。
    const Vec3 yAxisOrtho = normalize(cross(zAxis, xAxis));

    out.rotation = { xAxis[0], xAxis[1], xAxis[2],
                    yAxisOrtho[0], yAxisOrtho[1], yAxisOrtho[2],
                    zAxis[0], zAxis[1], zAxis[2] };
    out.origin = wristCentroid;
    out.valid = true;
    out.message = "轴系对齐成功：判定手指链下标 " + std::to_string(bestIdx) + " 为拇指(区分度得分 " +
                 std::to_string(bestScore) + "mm)。";
    return out;
}

// 把对齐结果应用到任意一个点上：new = rotation * (old - origin)。
inline Vec3 applyAxisAlignment(const AxisAlignmentResult& align, const Vec3& p) {
    const Vec3 d = { p[0]-align.origin[0], p[1]-align.origin[1], p[2]-align.origin[2] };
    const auto& R = align.rotation;
    return { R[0]*d[0]+R[1]*d[1]+R[2]*d[2],
             R[3]*d[0]+R[4]*d[1]+R[5]*d[2],
             R[6]*d[0]+R[7]*d[1]+R[8]*d[2] };
}

} // namespace mocap
