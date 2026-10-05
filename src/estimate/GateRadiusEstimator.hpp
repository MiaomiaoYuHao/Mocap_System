#pragma once
// ---------------------------------------------------------------------------
// 关联门控半径自动推算 —— 纯数学，零 Qt 依赖，可单测。
//
// 解决的问题：MarkerAssociator 的 gateRadiusNorm 之前是拍脑袋给的固定值
// (0.05)，跟你实际手的大小、贴球疏密、工作距离完全没关系——贴得紧凑的
// 手在这个半径下容易关联错位(手指乱跳)，贴得松散的手用这个半径又偏保守。
//
// 这个值其实不需要猜：手背模板+5指结构参数标定完之后，"20颗marker在各种
// 关节姿态下两两最近能凑多近"是纯几何计算，可以在几种代表性姿态(伸直、
// 半屈、攥拳)下遍历所有marker对的距离，取全局最小值——这就是"这个手在
// 门控半径给多大之前，两颗最容易被混淆的marker之间的真实间距"。乘一个
// 安全系数(留出位置估计误差的余量)、除以典型工作距离(把世界系距离换算成
// 归一化相机坐标下的角度)，就是一个真正跟这只手、这套贴球方案匹配的
// 门控半径，不是一个跟谁都不挂钩的常量。
//
// 【局限，不回避】这个推算假设了"关节角只在jointLimits范围内变化"这个
// 前提本身是对的(标定/占位表准)，也只采样了几个代表性姿态，不是穷举
// 所有可能姿态的严格下界——真实使用中如果某个特殊姿态让两颗marker贴得
// 比这里算出来的还近，门控半径依然可能不够用。这是一个"比拍脑袋常量
// 好得多的起点"，不是数学上绝对保证不出错的上界。
// ---------------------------------------------------------------------------
#include "estimate/HandStateIEKF.hpp"   // ForwardKinematicsFn, Vec3
#include "hand/HandModel.hpp"           // jointLimits(), kHandNumJoints概念(这里用16硬编码，跟项目约定一致)
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

namespace mocap {

struct GateRadiusEstimate {
    bool valid = false;
    double recommendedGateRadiusNorm = 0.05;   // 建议值，失败时保留一个安全的旧默认值
    double minMarkerDistMm = -1.0;             // 采样到的全局最小marker间距(诊断用)
    int worstMarkerI = -1, worstMarkerJ = -1;  // 哪两颗marker最容易混淆(诊断用，供UI提示"这两颗离得最近")
    std::string message;
};

namespace gate_estimate_detail {

inline std::vector<std::vector<double>> representativePoses() {
    // 伸直(0)、半屈(0.5*上限)、攥拳(上限)——用 jointLimits() 的上边界模拟
    // "手指尽量蜷起来"这个最容易让相邻marker靠近的姿态，下边界(通常只是
    // 外展的反方向，量级小)不太可能让marker更靠近，不用额外采样。
    const auto& lim = jointLimits();
    std::vector<double> extended(16, 0.0);
    std::vector<double> halfFlexed(16, 0.0);
    std::vector<double> fullFlexed(16, 0.0);
    for (int i = 0; i < 16; ++i) {
        fullFlexed[size_t(i)] = lim[size_t(i)].hi;
        halfFlexed[size_t(i)] = 0.5 * lim[size_t(i)].hi;
    }
    return { extended, halfFlexed, fullFlexed };
}

} // namespace gate_estimate_detail

// fk: 通常是 makeHandForwardKinematicsFromTemplate(...) 产出的、绑定了
// 实际标定模板的那个闭包——用真实模板算，不是占位值，算出来的间距才有
// 意义。workingDistanceMm：典型工作距离(比如冷启动/标定时手到相机的
// 大致距离)。marginFraction：安全折扣(默认0.4，即门控半径只取"最容易
// 混淆的两颗marker间距"的40%，留60%的余量给位置估计误差/多帧抖动)。
inline GateRadiusEstimate estimateGateRadius(const ForwardKinematicsFn& fk,
                                             double workingDistanceMm,
                                             double marginFraction = 0.4) {
    using namespace gate_estimate_detail;
    GateRadiusEstimate out;

    if (workingDistanceMm <= 1.0) {
        out.message = "工作距离参数不合理(<=1mm)，无法换算成归一化坐标下的门控半径。";
        return out;
    }
    if (marginFraction <= 0.0 || marginFraction >= 1.0) {
        out.message = "安全折扣marginFraction必须在(0,1)区间内。";
        return out;
    }

    double globalMinDist = 1e18;
    int worstI = -1, worstJ = -1;

    for (const auto& pose : representativePoses()) {
        const auto markers = fk(pose);
        const int n = int(markers.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i+1; j < n; ++j) {
                const auto& a = markers[size_t(i)]; const auto& b = markers[size_t(j)];
                const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
                const double d = std::sqrt(dx*dx+dy*dy+dz*dz);
                if (d < globalMinDist) { globalMinDist = d; worstI = i; worstJ = j; }
            }
        }
    }

    if (worstI < 0) {
        out.message = "FK没有产出任何marker(手部模板可能没有正确加载)，无法推算。";
        return out;
    }

    out.minMarkerDistMm = globalMinDist;
    out.worstMarkerI = worstI;
    out.worstMarkerJ = worstJ;
    out.recommendedGateRadiusNorm = (globalMinDist * marginFraction) / workingDistanceMm;
    out.valid = true;

    char buf[300];
    std::snprintf(buf, sizeof(buf),
        "推算完成：采样的代表性姿态里，marker #%d 和 #%d 最容易靠近(最小间距 %.1fmm)，"
        "按工作距离 %.0fmm、安全折扣 %.0f%% 算出建议门控半径 %.4f(归一化坐标)。",
        worstI, worstJ, globalMinDist, workingDistanceMm, marginFraction*100.0, out.recommendedGateRadiusNorm);
    out.message = buf;
    return out;
}

} // namespace mocap
