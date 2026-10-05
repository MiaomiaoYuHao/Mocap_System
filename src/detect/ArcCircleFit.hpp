#pragma once
// ---------------------------------------------------------------------------
// 遮挡感知检测层 · 3b：单球半遮挡的圆弧拟合 + 弧长门控 —— 纯数学，零 Qt 依赖。
//
// 建立在 ConstrainedCircleFit.hpp (3a) 之上。3a 只回答"给一堆边缘点，圆心在
// 哪"；3b 要多回答两个问题：
//   1. 这堆点到底跨了多少角度的弧？（球被遮了多少）
//   2. 跨的角度够不够格信这次拟合？不够格的话，标成什么样的不确定度、
//      或者干脆弃用，交给上层的多相机/时序兜底（README §3.5）。
//
// 对应 README §3.1 的门槛表与判据：
//   ≥90°  直接信，精度好（几分之一像素级）
//   60~90° 能用，但要撑大协方差，标"低置信度"
//   <60°  判定为不可信观测，弃用
// 这个"信多少"的量化用 §3.1 实测表（不同弧长下约束拟合的圆心误差均值）做
// 分段线性插值，得到一个连续的 σ 估计，而不是只给三档粗糙标签——上层滤波
// 需要的是协方差数字，不是"高/中/低"这种定性描述。
//
// 范围声明（避免职责膨胀）：本模块**不做**"这坨轮廓是完整球还是被遮挡的弧"
// 这一步判断（README 提到的"轮廓是否闭合 + 遮挡边界处是否断崖式梯度"），那
// 是实际图像连通域/边缘提取阶段的工作，属于 CentroidDetector 那一层在真实
// 图像上要做的事。本模块假设调用方已经从轮廓上提取出一批边缘点（不要求有
// 序，来自 flood fill 的点集顺序本来就是任意的），只负责"这批点当一段弧来
// 拟合，值不值得信"这一步纯几何计算。
// ---------------------------------------------------------------------------
#include "detect/ConstrainedCircleFit.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace mocap {

enum class ArcConfidence {
    High,     // >=90°：直接信，协方差按表给
    Medium,   // 60~90°：能用，协方差撑大
    Discard   // <60°：不可信，弃用，交给多相机/时序兜底
};

struct ArcFitResult {
    Point2 center{};
    double radius = 0.0;
    double spanDegrees = 0.0;     // 估计出的可见弧跨度
    double sigmaPx = -1.0;        // 圆心误差的估计标准差（像素），供上层做协方差
    ArcConfidence confidence = ArcConfidence::Discard;
    bool usable = false;          // Discard 时恒为 false，即使拟合数值本身没崩
};

namespace arcfit_detail {

// 估计一批点相对某个圆心的可见弧跨度（角度）。做法：把每个点转成相对圆心的
// 方位角，排序后找"最大的角度空隙"——那段空隙就是弧没覆盖到的部分（球被遮
// 挡的那一侧），跨度 = 360° - 最大空隙。这个方法不要求点集有序（flood fill
// 提取的边缘点本来就没有固有顺序），对任意排列都稳健。
inline double estimateArcSpanDegrees(const std::vector<Point2>& pts, const Point2& center) {
    if (pts.size() < 2) return 0.0;
    std::vector<double> angles;
    angles.reserve(pts.size());
    for (const auto& p : pts) {
        double a = std::atan2(p.y - center.y, p.x - center.x) * 180.0 / M_PI;
        if (a < 0.0) a += 360.0;
        angles.push_back(a);
    }
    std::sort(angles.begin(), angles.end());
    double maxGap = 0.0;
    for (size_t i=0; i+1<angles.size(); ++i) maxGap = std::max(maxGap, angles[i+1]-angles[i]);
    // 首尾环绕的那一段空隙（从最后一个角度绕回第一个角度）
    maxGap = std::max(maxGap, 360.0 - angles.back() + angles.front());
    return std::min(360.0, 360.0 - maxGap);
}

// README §3.1 实测表的分段线性插值：给定弧跨度，估计约束拟合的圆心误差
// 标准差（像素）。表外（>180 视为满弧、<30 视为几乎报废）做保守的边界钳制。
inline double lookupSigmaFromSpan(double spanDegrees) {
    // (span, sigma) 断点，按 span 降序排列，跟 README 表格一致。
    static const double spanPts[]  = {180.0, 120.0, 90.0, 60.0, 45.0, 30.0};
    static const double sigmaPts[] = {0.18,  0.19,  0.30, 1.03, 2.69, 5.00};
    constexpr int N = 6;

    if (spanDegrees >= spanPts[0]) return sigmaPts[0];
    if (spanDegrees <= spanPts[N-1]) return sigmaPts[N-1] * (spanDegrees<=0.0?2.0:1.0 + (spanPts[N-1]-spanDegrees)/spanPts[N-1]);
    for (int i=0; i<N-1; ++i) {
        if (spanDegrees <= spanPts[i] && spanDegrees >= spanPts[i+1]) {
            const double t = (spanPts[i]-spanDegrees) / (spanPts[i]-spanPts[i+1]);
            return sigmaPts[i] + t*(sigmaPts[i+1]-sigmaPts[i]);
        }
    }
    return sigmaPts[N-1];
}

inline ArcConfidence classify(double spanDegrees) {
    if (spanDegrees >= 90.0) return ArcConfidence::High;
    if (spanDegrees >= 60.0) return ArcConfidence::Medium;
    return ArcConfidence::Discard;
}

} // namespace arcfit_detail

// 主入口：拟合 + 估弧长 + 门控，一步做完。knownRadius 来自标定（§3.0）。
// Medium 档会把 sigma 再撑大一截（README:"用但撑大不确定度"，不是照抄同一
// 个数字），避免"能用"和"优秀"两档在协方差上没有区分度。
inline ArcFitResult fitArcWithGating(const std::vector<Point2>& pts, double knownRadius) {
    using namespace arcfit_detail;
    ArcFitResult out;
    if (pts.size() < 2 || knownRadius <= 0.0) return out;

    const auto fit = fitCircleKnownRadiusAuto(pts, knownRadius);
    if (!fit.valid) return out;

    const double span = estimateArcSpanDegrees(pts, fit.center);
    const ArcConfidence conf = classify(span);
    double sigma = lookupSigmaFromSpan(span);
    if (conf == ArcConfidence::Medium) sigma *= 1.5;   // 撑大不确定度，不是虚报小协方差

    out.center = fit.center;
    out.radius = fit.radius;
    out.spanDegrees = span;
    out.sigmaPx = sigma;
    out.confidence = conf;
    out.usable = (conf != ArcConfidence::Discard);
    return out;
}

} // namespace mocap
