#pragma once
// ---------------------------------------------------------------------------
// 半径无关的深度粗算 + 半径反推 —— 纯数学，零 Qt 依赖，可单测。
//
// 解决的问题：HandTrackingWorker 原来的"已知半径"假设来自一个固定的标称
// 工作距离常量(nominalWorkingDistanceMm)，但实际拍摄距离跨度很大(比如
// 15~50cm)时，这个固定假设在两头都严重偏离真实值，导致3a~3d约束圆拟合把
// 圆心系统性拟合偏，进而两视图对极几何验证失败，三角化拿不到候选点——
// 症状是"预览能看到点，检测本身没问题，但手部追踪/标定拿不到候选点"。
//
// 这个问题的根本解法：半径这个先验其实不是必须"预先知道"的——它可以从
// 数据本身反推出来。流程分两遍：
//   第一遍(本文件)：不需要知道半径，直接用每台相机检测到的原始质心
//   (Blob::cx,cy，跟约束圆拟合完全无关，纯粹是flood fill的加权质心)做
//   两视图/多视图匹配+三角化，得到每个候选点的粗略3D位置——质心位置不
//   依赖半径假设，这一步的匹配阈值需要放宽(质心噪声通常比精修后的圆心
//   噪声大)，但完全不需要猜"这个marker多大/多远"。
//   第二遍(调用方 HandTrackingWorker 负责)：粗略3D点反投影回每台相机，
//   算出该点在该相机的真实深度 Zc，用 physicalRadiusMm * fx / Zc 反推出
//   这台相机、这一帧、这个marker该用的真实像素半径——不再是一个全局固定
//   常量，而是每个点各自算一份，精度不受"标称距离假设"制约。拿这个精确
//   半径重新喂给 3a~3d 做精修拟合，圆心不再系统性偏移，最终三角化匹配率
//   应该显著回升。
//
// 本文件只提供"给定粗略3D点集 + 某台相机的位姿 + 某个2D质心观测，找出
// 这个质心对应哪个粗略3D点、算出对应深度、反推期望半径"这一步的纯数学，
// 不涉及 clusterMultiView 本身怎么跑(那部分调用方直接复用现成的
// reconstruct/MultiViewCluster.hpp，不需要在这里重新实现)。
// ---------------------------------------------------------------------------
#include <array>
#include <vector>
#include <cmath>
#include <optional>

namespace mocap {

using RfVec3 = std::array<double, 3>;
using RfMat3 = std::array<double, 9>;   // 行主序，世界->相机

struct RadiusRefineResult {
    bool matched = false;
    double expectedRadiusPx = -1.0;
    double matchedDepthMm = -1.0;      // 该候选点在这台相机下的深度 Zc(供诊断)
    double reprojResidualNorm = -1.0;  // 归一化坐标下的重投影残差(供诊断/门控)
};

// 把世界系点投影到某台相机的归一化坐标；点在相机后方(Zc<=eps)返回 false。
inline bool reprojectNormRf(const RfMat3& R, const RfVec3& t, const RfVec3& Xw,
                            double& nx, double& ny, double& Zc) {
    const double Xc = R[0]*Xw[0]+R[1]*Xw[1]+R[2]*Xw[2]+t[0];
    const double Yc = R[3]*Xw[0]+R[4]*Xw[1]+R[5]*Xw[2]+t[1];
    Zc = R[6]*Xw[0]+R[7]*Xw[1]+R[8]*Xw[2]+t[2];
    if (Zc <= 1.0) return false;   // 1mm 内视为退化，不是"刚好贴着相机"这种正常场景
    nx = Xc/Zc; ny = Yc/Zc;
    return true;
}

// 给定这台相机的一个2D质心观测(已去畸变归一化坐标)，在一批粗略3D候选点里
// 找"投影到这台相机后离这个观测最近"的那个，用它的深度反推期望像素半径。
// maxMatchNorm：投影残差门控(归一化坐标单位)——粗略三角化的点可能包含
// "只有另外两台相机三角化出来、这台相机本来就没看到"的候选点，不能無條件
// 认领离得最近的那个，必须离得足够近才采信，否则宁可不匹配、让调用方退回
// 旧的固定假设。
inline RadiusRefineResult refineExpectedRadius(
        double obsNx, double obsNy,
        const RfMat3& camR, const RfVec3& camT, double camFx,
        const std::vector<RfVec3>& roughPoints3D,
        double physicalRadiusMm,
        double maxMatchNorm = 0.03) {
    RadiusRefineResult out;

    double bestDist = 1e18;
    double bestZc = -1.0;
    for (const auto& p : roughPoints3D) {
        double px, py, Zc;
        if (!reprojectNormRf(camR, camT, p, px, py, Zc)) continue;
        const double dx = px - obsNx, dy = py - obsNy;
        const double d = std::sqrt(dx*dx + dy*dy);
        if (d < bestDist) { bestDist = d; bestZc = Zc; }
    }

    if (bestZc <= 0.0 || bestDist > maxMatchNorm) {
        out.matched = false;
        out.reprojResidualNorm = bestDist;
        return out;
    }

    out.matched = true;
    out.matchedDepthMm = bestZc;
    out.reprojResidualNorm = bestDist;
    out.expectedRadiusPx = physicalRadiusMm * camFx / bestZc;
    return out;
}

} // namespace mocap
