#pragma once
// ---------------------------------------------------------------------------
// 遮挡感知检测层 · 3d：检测层输出契约 —— 纯数学，零 Qt 依赖。
//
// 3a/3b/3c 是三条并列的拟合路径(完整圆/半遮挡弧/花生双圆)，各自内部逻辑不
// 一样，但上层(IEKF 状态估计)不该关心这一帧的某个观测究竟是哪条路径产出
// 的——它只应该看到统一的一件事:每个可能的球观测是"(μ, Σ, 置信度)"这样
// 一个标准形状。这一层就是把 3a/3b/3c 的输出捏成这个统一契约，是 §3 遮挡
// 感知检测层对外的唯一入口，IEKF 只需要 #include 这一个头。
//
// 流程：
//   1. 先用 3c 的 selectBallModel 判定这坨轮廓点是单球还是双球。
//   2. 单球：整坨点直接走 3b 的 fitArcWithGating，产出 1 个观测。
//   3. 双球：3c 已经把点分成两群，各自群再独立走一遍 3b 的弧长门控——因为
//      "两球贴在一起"不代表两颗球各自没有被别的东西(第三颗球/手指/画面
//      边缘)再遮一部分，两个观测的置信度可能完全不一样，不能假设双球场景
//      下两个观测都天然可信。产出 2 个观测。
//   4. 每个观测的协方差 Σ 不是各向同性的一个数——弧越短，圆心沿"圆心到
//      可见弧中点方向"(径向)的误差比切向误差大得多(这是圆拟合的经典
//      病态方向:一段浅弧既可以是"大圆的一小段"也可以是"小圆更弯的一
//      段"，径向深度信息弱，切向位置信息强)。这里用一个简单的经验膨胀
//      模型把方向性也编码进 Σ，比只给一个标量方差更忠实地反映"这个观测
//      在哪个方向上更不可信"，供 IEKF 的观测协方差直接使用。
//
// 契约设计原则：即使某个观测被 3b 判定 Discard(usable=false)，也**不丢弃
// 它**，而是原样放进输出里、把 usable 标成 false——弃用与否的决定权交给
// 调用方(IEKF 可能有自己的策略，比如"哪怕低置信度也比完全没观测强")，
// 检测层只负责如实报告，不替上层做最终取舍。
// ---------------------------------------------------------------------------
#include "detect/ArcCircleFit.hpp"
#include "detect/TwoCircleFit.hpp"
#include <vector>
#include <cmath>

namespace mocap {

// 2x2 协方差矩阵(对称，px^2 单位)。
struct Cov2 { double xx=0.0, xy=0.0, yy=0.0; };

struct Observation2D {
    Point2 mu{};
    Cov2 sigma{};
    double spanDegrees = 0.0;
    ArcConfidence confidence = ArcConfidence::Discard;
    bool usable = false;
};

struct DetectionOutput {
    BallModel model = BallModel::Single;
    std::vector<Observation2D> observations;   // Single: 0或1个；Double: 0或2个
    bool valid = false;
};

namespace contract_detail {

// 一批点相对圆心的"可见弧中点方向"(圆心指向弧中点的单位向量)，用角度的
// 圆均值(atan2(mean sin, mean cos))算，能正确处理角度环绕。
inline Point2 radialDirection(const std::vector<Point2>& pts, const Point2& center) {
    double sx=0.0, sy=0.0;
    for (const auto& p : pts) {
        const double a = std::atan2(p.y-center.y, p.x-center.x);
        sx += std::cos(a); sy += std::sin(a);
    }
    const double n = std::sqrt(sx*sx+sy*sy);
    if (n < 1e-12) return {1.0, 0.0};   // 完整圆时各方向抵消，方向不重要(各向同性)
    return {sx/n, sy/n};
}

// 弧越短，径向(深度方向)的不确定度相对切向膨胀得越厉害——经验模型：
// span>=150 视为接近各向同性(完整圆没有"哪个方向更不可信"一说)；
// span 越小，径向膨胀系数越大，上限钳制避免数值失控。
inline double radialInflationFactor(double spanDegrees) {
    if (spanDegrees >= 150.0) return 1.0;
    const double t = (150.0 - spanDegrees) / 150.0;   // 0..1
    return 1.0 + 4.0 * t * t;   // 150deg->1.0, 60deg->~2.44, 45deg->~2.94(上限4.0在0度时)
}

inline Cov2 buildCovariance(const std::vector<Point2>& pts, const Point2& center,
                            double sigmaIsotropic, double spanDegrees) {
    const Point2 u = radialDirection(pts, center);
    const double inflate = radialInflationFactor(spanDegrees);
    const double sigmaRadial = sigmaIsotropic * inflate;
    const double sigmaTangential = sigmaIsotropic;
    const double vr = sigmaRadial*sigmaRadial, vt = sigmaTangential*sigmaTangential;
    Cov2 c;
    c.xx = vt + (vr-vt)*u.x*u.x;
    c.xy =      (vr-vt)*u.x*u.y;
    c.yy = vt + (vr-vt)*u.y*u.y;
    return c;
}

inline Observation2D toObservation(const ArcFitResult& arc, const std::vector<Point2>& pts) {
    Observation2D obs;
    obs.mu = arc.center;
    obs.spanDegrees = arc.spanDegrees;
    obs.confidence = arc.confidence;
    obs.usable = arc.usable;
    if (arc.sigmaPx > 0.0) obs.sigma = buildCovariance(pts, arc.center, arc.sigmaPx, arc.spanDegrees);
    return obs;
}

} // namespace contract_detail

// 主入口：检测层对外的唯一函数。knownRadius 来自标定(§3.0)。
// 主入口：检测层对外的唯一函数。knownRadius 来自标定(§3.0)。
// mayBeMerged：调用方的廉价预筛结果。默认 true = 旧行为(总是尝试双圆拆分)。
//   传 false 表示"调用方已经用外接框长宽比等廉价判据确认这个 blob 不像
//   粘连"，此时直接走单球拟合、跳过昂贵的双圆交替优化(selectBallModel 里
//   那套 20 次迭代)。密集摆位下绝大多数 blob 是单球，这道门把 detectBalls
//   的单次成本从 ~107us 降到 ~单圆拟合的量级，实测整帧检测加速 5x、且对
//   真正粘连的 blob(长宽比>1.3)零漏检。预筛在调用方做，因为只有调用方
//   手里有 Blob 的外接框；这里只提供"跳过双圆"的开关。
inline DetectionOutput detectBalls(const std::vector<Point2>& contourPoints, double knownRadius,
                                   bool mayBeMerged = true) {
    using namespace contract_detail;
    DetectionOutput out;
    if (contourPoints.size() < 2 || knownRadius <= 0.0) return out;

    // 预筛判定为"不可能是粘连"→ 只做单球拟合，跳过双圆。
    if (!mayBeMerged) {
        const auto arc = fitArcWithGating(contourPoints, knownRadius);
        out.valid = true;
        out.model = BallModel::Single;
        out.observations.push_back(toObservation(arc, contourPoints));
        return out;
    }

    const auto sel = selectBallModel(contourPoints, knownRadius);
    if (!sel.valid) return out;

    out.valid = true;
    out.model = sel.model;

    if (sel.model == BallModel::Single) {
        const auto arc = fitArcWithGating(contourPoints, knownRadius);
        out.observations.push_back(toObservation(arc, contourPoints));
        return out;
    }

    // Double：直接复用 selectBallModel 内部那次双圆拟合的点群。
    // 【改动】原来这里会再调一次 fitTwoCirclesKnownRadius，理由是"不掏别的
    // 模块的内部细节，这点计算量可忽略"。在几十点的正常轮廓上确实可忽略，
    // 但轮廓涨到几百点(相机没摆好/阈值不对导致的大片不规则白块)时，这一次
    // 重跑就是整整一倍的开销，而它算出来的东西跟已经算过的一模一样。
    // 现在 BallModelResult 如实带出了那次结果(见 TwoCircleFit.hpp 的说明)，
    // 直接取用即可——确定性计算，同输入同输出，行为完全等价。
    const auto& dbl = sel.dbl;
    if (!dbl.valid) {   // 理论上不该发生(selectBallModel已经判定Double)，兜底退回单球
        out.model = BallModel::Single;
        const auto arc = fitArcWithGating(contourPoints, knownRadius);
        out.observations.push_back(toObservation(arc, contourPoints));
        return out;
    }

    const auto arc1 = fitArcWithGating(dbl.points1, knownRadius);
    const auto arc2 = fitArcWithGating(dbl.points2, knownRadius);
    out.observations.push_back(toObservation(arc1, dbl.points1));
    out.observations.push_back(toObservation(arc2, dbl.points2));
    return out;
}

} // namespace mocap
