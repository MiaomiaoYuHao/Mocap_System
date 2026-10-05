#pragma once
// ---------------------------------------------------------------------------
// 遮挡感知检测层 · 3c：侧向花生双圆联合拟合 + 1球/2球模型选择 —— 纯数学，
// 零 Qt 依赖，建立在 3a (ConstrainedCircleFit.hpp) 之上。
//
// 场景：两颗反光球在图像上靠得很近甚至互相重叠，flood fill 出来的是一个
// 连通的"花生形"轮廓，而不是两个分开的圆。这层要回答两个问题：
//   1. 假设这坨点其实是两个圆(半径都已知、且相同——同一套 marker)，两个
//      圆心分别在哪？(双圆联合拟合)
//   2. 这坨点到底该当一个圆解释、还是两个圆解释？(模型选择——错判成两个
//      会凭空生出一个不存在的点；错判成一个会把两个真点的观测混成一个,
//      两种错误都要避免)
//
// 双圆联合拟合用交替优化(类似双圆心版的 k-means/Lloyd 迭代)：
//   a. 把每个点分配给"残差(|dist-r|)更小"的那个圆
//   b. 用分配到的点各自重新做一次 3a 的已知半径拟合，更新圆心
//   c. 重复到分配不再变化或到迭代上限
// 初始两个圆心的种子来自点云的主方向(2x2 协方差特征分解)：花生形的长轴
// 大概率就是两球连心线方向，沿这个方向在质心两侧各偏移半径长度做种子，
// 比随便取两个点当种子更稳。
//
// 模型选择不是单纯比较残差(双圆参数更多，残差几乎总是更小，无脑比较会
// 系统性偏向双圆)，用三个联合门槛：
//   1. 残差改善要足够大 (double 残差 <= single 残差 * 阈值，不是随便小一点)
//   2. 两个圆心的间距要足够开 (间距太近说明双圆退化成了同一个圆的两份重复解)
//   3. 两个圆分到的点数要足够均衡 (避免"主圆 + 几个噪声点凑出的伪第二圆")
// 三条都满足才判定为双球，否则一律按单球处理——宁可漏判花生、交给上层用
// 更大的不确定度兜底，也不要凭空报出一个不存在的第二个点。
// ---------------------------------------------------------------------------
#include "detect/ConstrainedCircleFit.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace mocap {

struct TwoCircleFitResult {
    Point2 center1{};
    Point2 center2{};
    double radius = 0.0;
    double residual = -1.0;   // 两个圆合起来的 RMS 几何残差
    int count1 = 0, count2 = 0;   // 各自分到的点数
    std::vector<Point2> points1, points2;   // 各自分到的点(供上层比如3d单独再拟合弧长/门控用)
    bool valid = false;
};

enum class BallModel { Single, Double };

struct BallModelResult {
    BallModel model = BallModel::Single;
    Point2 center1{};
    Point2 center2{};          // 仅当 model==Double 时有意义
    double radius = 0.0;
    double residualSingle = -1.0;
    double residualDouble = -1.0;
    bool valid = false;

    // 【新增】selectBallModel 内部那次双圆拟合的完整结果（含 points1/points2
    // 两个点群）。
    //
    // 为什么要带出来：原先 DetectionOutput.hpp::detectBalls 在判定为 Double
    // 之后会【再调一次】fitTwoCirclesKnownRadius 去拿点群，理由是"模块之间
    // 不互相掏内部细节，多的这一次计算量可以忽略"。那个"可忽略"是按几十点
    // 的正常轮廓估的；轮廓涨到几百点时，这一次重跑就是实打实的一倍开销
    // （双圆拟合本身约 20 轮外层迭代 × 每轮两次 30 迭代的 LM）。
    // 现在如实把结果带出来，调用方直接用，Double 路径省掉一半计算，且结果
    // 与重跑严格一致（同一份确定性计算，同样的输入必然同样的输出）。
    // dbl.valid==false 表示双圆拟合没收敛/退化，此时 model 必然是 Single。
    TwoCircleFitResult dbl{};
};

namespace twocircle_detail {

// 2x2 对称矩阵的较大特征值对应的（单位化）特征向量——花生形长轴方向。
inline Point2 principalDirection(double a, double b, double c) {
    // 特征值 = (a+c)/2 ± sqrt(((a-c)/2)^2+b^2)，取较大的那个对应的特征向量。
    const double mid = (a+c)*0.5;
    const double diff = (a-c)*0.5;
    const double rad = std::sqrt(diff*diff + b*b);
    const double lambda1 = mid + rad;
    Point2 v;
    if (std::abs(b) > 1e-12) { v.x = lambda1 - c; v.y = b; }
    else { v = (a >= c) ? Point2{1,0} : Point2{0,1}; }
    const double n = std::sqrt(v.x*v.x + v.y*v.y);
    if (n > 1e-12) { v.x/=n; v.y/=n; }
    return v;
}

inline void seedTwoCenters(const std::vector<Point2>& pts, double knownRadius,
                           Point2& seed1, Point2& seed2) {
    Point2 mean{0,0};
    for (const auto& p : pts) { mean.x += p.x; mean.y += p.y; }
    mean.x /= double(pts.size()); mean.y /= double(pts.size());

    double sxx=0, sxy=0, syy=0;
    for (const auto& p : pts) {
        const double dx = p.x-mean.x, dy = p.y-mean.y;
        sxx += dx*dx; sxy += dx*dy; syy += dy*dy;
    }
    sxx /= double(pts.size()); sxy /= double(pts.size()); syy /= double(pts.size());

    const Point2 dir = principalDirection(sxx, sxy, syy);
    // 沿长轴方向、质心两侧各偏移一个半径——两球紧贴/重叠时圆心距通常在
    // [~1r, ~2r] 之间，半径是个合理的起始猜测,交替优化会把它收敛到位。
    seed1 = { mean.x - dir.x*knownRadius, mean.y - dir.y*knownRadius };
    seed2 = { mean.x + dir.x*knownRadius, mean.y + dir.y*knownRadius };
}

} // namespace twocircle_detail

// 双圆联合拟合。种子留空(nullptr)则自动用主方向估计；调用方也可以传入自己
// 的种子(比如用上一帧的两个圆心，帧间连续性通常比重新估计主方向更准)。
inline TwoCircleFitResult fitTwoCirclesKnownRadius(const std::vector<Point2>& pts, double knownRadius,
                                                   const Point2* seed1 = nullptr, const Point2* seed2 = nullptr,
                                                   int maxIters = 20) {
    TwoCircleFitResult out;
    if (pts.size() < 4 || knownRadius <= 0.0) return out;   // 两个圆至少各要2点

    Point2 c1, c2;
    if (seed1 && seed2) { c1=*seed1; c2=*seed2; }
    else twocircle_detail::seedTwoCenters(pts, knownRadius, c1, c2);

    std::vector<int> assign(pts.size(), 0);
    for (int iter=0; iter<maxIters; ++iter) {
        bool changed = false;
        std::vector<int> newAssign(pts.size());
        for (size_t i=0;i<pts.size();++i) {
            const double d1 = std::abs(std::sqrt((pts[i].x-c1.x)*(pts[i].x-c1.x)+(pts[i].y-c1.y)*(pts[i].y-c1.y)) - knownRadius);
            const double d2 = std::abs(std::sqrt((pts[i].x-c2.x)*(pts[i].x-c2.x)+(pts[i].y-c2.y)*(pts[i].y-c2.y)) - knownRadius);
            newAssign[i] = (d1 <= d2) ? 0 : 1;
            if (newAssign[i] != assign[i]) changed = true;
        }
        assign = newAssign;

        std::vector<Point2> g1, g2;
        for (size_t i=0;i<pts.size();++i) (assign[i]==0 ? g1 : g2).push_back(pts[i]);
        if (g1.size() < 2 || g2.size() < 2) break;   // 退化成单群，交给模型选择去判定

        const auto f1 = fitCircleKnownRadius(g1, knownRadius, c1);
        const auto f2 = fitCircleKnownRadius(g2, knownRadius, c2);
        if (!f1.valid || !f2.valid) break;
        c1 = f1.center; c2 = f2.center;

        if (!changed && iter>0) break;   // 分配稳定，收敛
    }

    std::vector<Point2> g1, g2;
    for (size_t i=0;i<pts.size();++i) (assign[i]==0 ? g1 : g2).push_back(pts[i]);
    if (g1.size() < 2 || g2.size() < 2) return out;

    double sumSq = 0.0;
    for (const auto& p : g1) { const double d = std::sqrt((p.x-c1.x)*(p.x-c1.x)+(p.y-c1.y)*(p.y-c1.y)) - knownRadius; sumSq += d*d; }
    for (const auto& p : g2) { const double d = std::sqrt((p.x-c2.x)*(p.x-c2.x)+(p.y-c2.y)*(p.y-c2.y)) - knownRadius; sumSq += d*d; }

    out.center1 = c1; out.center2 = c2;
    out.radius = knownRadius;
    out.residual = std::sqrt(sumSq / double(pts.size()));
    out.count1 = int(g1.size()); out.count2 = int(g2.size());
    out.points1 = g1; out.points2 = g2;
    out.valid = true;
    return out;
}

// 1球/2球模型选择。三个门槛全部通过才判定 Double：
//   residualRatioThreshold：双圆残差 <= 单圆残差 * 这个比例(默认 0.6，即
//     至少要改善 40% 才算数，防止"参数多了残差自然更小"的虚假改善)。
//   minSeparationRatio：两圆心距离 / 半径 至少要达到这个比例(默认 0.5)，
//     太近说明两个圆解在退化成同一个圆。
//   minBalanceFraction：较小那个群的点数占比至少要达到这个比例(默认
//     0.15)，防止"主圆 + 几个离群点"被误判成第二个球。
inline BallModelResult selectBallModel(const std::vector<Point2>& pts, double knownRadius,
                                       double residualRatioThreshold = 0.6,
                                       double minSeparationRatio = 0.5,
                                       double minBalanceFraction = 0.15) {
    BallModelResult out;
    if (pts.size() < 4 || knownRadius <= 0.0) return out;

    const auto single = fitCircleKnownRadiusAuto(pts, knownRadius);
    if (!single.valid) return out;
    out.residualSingle = single.residual;

    const auto dbl = fitTwoCirclesKnownRadius(pts, knownRadius);

    out.model = BallModel::Single;
    out.center1 = single.center;
    out.radius = knownRadius;
    out.valid = true;

    out.dbl = dbl;                // 无论选中与否都带出去，调用方不必重算
    if (!dbl.valid) return out;   // 双圆拟合本身没收敛/退化，直接用单圆结果
    out.residualDouble = dbl.residual;

    const double sep = std::sqrt((dbl.center1.x-dbl.center2.x)*(dbl.center1.x-dbl.center2.x) +
                                 (dbl.center1.y-dbl.center2.y)*(dbl.center1.y-dbl.center2.y));
    const int minCount = std::min(dbl.count1, dbl.count2);
    const int totalCount = dbl.count1 + dbl.count2;
    const double balanceFraction = totalCount>0 ? double(minCount)/double(totalCount) : 0.0;

    const bool residualOk = dbl.residual <= single.residual * residualRatioThreshold;
    const bool separationOk = sep >= knownRadius * minSeparationRatio;
    const bool balanceOk = balanceFraction >= minBalanceFraction;

    if (residualOk && separationOk && balanceOk) {
        out.model = BallModel::Double;
        out.center1 = dbl.center1;
        out.center2 = dbl.center2;
    }
    return out;
}

} // namespace mocap
