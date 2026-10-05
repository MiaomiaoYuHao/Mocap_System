#pragma once
// ---------------------------------------------------------------------------
// 遮挡感知检测层 · 3a：已知半径约束的圆拟合 —— 纯数学，零 Qt/第三方依赖，可单测。
//
// 对应 README_手部动捕方案.md §3.0：标定完成后，给定一颗反光球的物理直径和它到
// 某台相机的深度，它在图像上的成像半径是可以预先算出来的。这个"已知半径"先验
// 把圆拟合从"三参数自由拟合"(圆心x,y + 半径)降到"两参数约束拟合"(只有圆心x,y)，
// 是后面 3b（半遮挡圆弧）、3c（花生双圆分离）共同的地基——那两层都要在这里的
// 两个函数上叠加"用什么点""怎么分组"，拟合数学本身在这里一次做对。
//
// 提供两个拟合器，供后续做 A/B 对比（README 里的 2~5 倍精度提升就是这两者的差）：
//   1. fitCircleFree      —— 自由三参数代数拟合（Kåsa 法），不知道半径时的基线。
//   2. fitCircleKnownRadius —— 已知半径的两参数几何拟合（高斯-牛顿迭代，最小化
//      "点到候选圆心的距离 - 已知半径"这个真正的几何残差，不是代数残差）。
//
// 为什么约束拟合要用几何法而不是代数法：如果沿用 Kåsa 那种线性代数技巧硬塞入
// 半径约束，本质上仍然是在解一个跟"半径"没有强绑定的线性系统（未知数 cx,cy,k，
// k 和已知 r 只在事后才对得上，约束是软的、事后验证性质，不是拟合过程中真正
// 生效的先验）。要让"已知半径"在拟合的每一步都起作用，必须用非线性最小二乘直接
// 最小化真正的物理量——每个点到圆心的欧氏距离与 r 的差——这样只剩 2 个自由度
// (cx,cy)，边缘点的信息全部只用来定位圆心，不浪费在同时估计一个本来就知道的半径
// 上，这正是精度提升的来源。
// ---------------------------------------------------------------------------
#include <vector>
#include <cmath>
#include <algorithm>

namespace mocap {

struct Point2 { double x = 0.0, y = 0.0; };

struct CircleFitResult {
    Point2 center{};
    double radius = 0.0;     // 拟合出的半径（约束拟合时恒等于传入的已知半径）
    double residual = -1.0;  // 均方根几何残差（像素）：点到圆心距离与半径之差的 RMS
    bool   valid = false;
};

namespace circlefit_detail {

// 计算一组点相对候选圆心的几何残差 RMS：sqrt(mean((dist_i - r)^2))。
inline double geometricResidualRMS(const std::vector<Point2>& pts, const Point2& c, double r) {
    if (pts.empty()) return -1.0;
    double sumSq = 0.0;
    for (const auto& p : pts) {
        const double dx = p.x - c.x, dy = p.y - c.y;
        const double d = std::sqrt(dx*dx + dy*dy);
        const double e = d - r;
        sumSq += e*e;
    }
    return std::sqrt(sumSq / double(pts.size()));
}

} // namespace circlefit_detail

// ---- 1. 自由三参数拟合（Kåsa 代数法）：不假设已知半径时的基线 ----
// 原理：圆方程 (x-cx)^2+(y-cy)^2=r^2 展开后对 (cx,cy,k) 线性，其中
// k = cx^2+cy^2-r^2。解线性最小二乘拿到 (cx,cy,k)，再反推 r。
// 代数残差（而非几何残差）在弧短、噪声大时会有系统性偏差——这正是 README
// §3.1 强调的"普通拟合在半遮挡时是有偏误差"的数学根源之一。
inline CircleFitResult fitCircleFree(const std::vector<Point2>& pts) {
    CircleFitResult out;
    const size_t n = pts.size();
    if (n < 3) return out;

    // 正规方程 A^T A x = A^T b，x=(cx,cy,k)^T，行：[-2xi, -2yi, 1]，b: -(xi^2+yi^2)
    double ATA[3][3] = {{0}};
    double ATb[3] = {0,0,0};
    for (const auto& p : pts) {
        const double row[3] = { -2.0*p.x, -2.0*p.y, 1.0 };
        const double b = -(p.x*p.x + p.y*p.y);
        for (int i=0;i<3;++i) {
            ATb[i] += row[i]*b;
            for (int j=0;j<3;++j) ATA[i][j] += row[i]*row[j];
        }
    }
    // 3x3 高斯消元解 ATA * x = ATb
    double M[3][4];
    for (int i=0;i<3;++i) { for (int j=0;j<3;++j) M[i][j]=ATA[i][j]; M[i][3]=ATb[i]; }
    for (int col=0; col<3; ++col) {
        int piv = col;
        for (int r=col+1; r<3; ++r) if (std::abs(M[r][col]) > std::abs(M[piv][col])) piv = r;
        if (std::abs(M[piv][col]) < 1e-12) return out;   // 退化（三点共线等）
        if (piv != col) for (int j=0;j<4;++j) std::swap(M[col][j], M[piv][j]);
        for (int r=0;r<3;++r) {
            if (r==col) continue;
            const double f = M[r][col]/M[col][col];
            for (int j=0;j<4;++j) M[r][j] -= f*M[col][j];
        }
    }
    const double cx = M[0][3]/M[0][0];
    const double cy = M[1][3]/M[1][1];
    const double k  = M[2][3]/M[2][2];
    const double r2 = cx*cx + cy*cy - k;
    if (r2 <= 0.0) return out;

    out.center = {cx, cy};
    out.radius = std::sqrt(r2);
    out.residual = circlefit_detail::geometricResidualRMS(pts, out.center, out.radius);
    out.valid = true;
    return out;
}

// ---- 2. 已知半径的两参数几何拟合（高斯-牛顿 + 阻尼，即 Levenberg-Marquardt 简化版）----
// 残差 f_i(cx,cy) = dist(p_i, (cx,cy)) - r，直接对真实几何量做最小二乘，
// 半径全程固定为已知值，不参与估计——这是"两参数"名副其实的地方。
// initGuess 建议传入 fitCircleFree 的中心作为起点（离真值通常已经很近，
// 收敛快；退化场景下调用方也可以传点集质心）。
inline CircleFitResult fitCircleKnownRadius(const std::vector<Point2>& pts, double knownRadius,
                                            Point2 initGuess, int maxIters = 30) {
    CircleFitResult out;
    const size_t n = pts.size();
    if (n < 2 || knownRadius <= 0.0) return out;

    double cx = initGuess.x, cy = initGuess.y;
    double lambda = 1e-3;   // LM 阻尼因子

    auto computeCostAndJTJ = [&](double ccx, double ccy, double JTJ[2][2], double JTf[2]) -> double {
        JTJ[0][0]=JTJ[0][1]=JTJ[1][0]=JTJ[1][1]=0.0;
        JTf[0]=JTf[1]=0.0;
        double cost = 0.0;
        for (const auto& p : pts) {
            const double dx = p.x - ccx, dy = p.y - ccy;
            double d = std::sqrt(dx*dx + dy*dy);
            if (d < 1e-9) d = 1e-9;   // 避免圆心恰好落在某点上时梯度爆炸
            const double f = d - knownRadius;
            // df/dcx = -(dx)/d, df/dcy = -(dy)/d
            const double j0 = -dx/d, j1 = -dy/d;
            JTJ[0][0] += j0*j0; JTJ[0][1] += j0*j1;
            JTJ[1][0] += j1*j0; JTJ[1][1] += j1*j1;
            JTf[0] += j0*f; JTf[1] += j1*f;
            cost += f*f;
        }
        return cost;
    };

    double JTJ[2][2], JTf[2];
    double cost = computeCostAndJTJ(cx, cy, JTJ, JTf);

    for (int iter=0; iter<maxIters; ++iter) {
        double A[2][2] = { {JTJ[0][0]*(1.0+lambda), JTJ[0][1]},
                            {JTJ[1][0], JTJ[1][1]*(1.0+lambda)} };
        const double det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
        if (std::abs(det) < 1e-15) break;
        // 解 A * delta = -JTf
        const double dcx = (-JTf[0]*A[1][1] + JTf[1]*A[0][1]) / det;
        const double dcy = (-A[0][0]*JTf[1] + A[1][0]*JTf[0]) / det;

        const double newCx = cx + dcx, newCy = cy + dcy;
        double newJTJ[2][2], newJTf[2];
        const double newCost = computeCostAndJTJ(newCx, newCy, newJTJ, newJTf);

        if (newCost < cost) {
            cx = newCx; cy = newCy; cost = newCost;
            JTJ[0][0]=newJTJ[0][0]; JTJ[0][1]=newJTJ[0][1];
            JTJ[1][0]=newJTJ[1][0]; JTJ[1][1]=newJTJ[1][1];
            JTf[0]=newJTf[0]; JTf[1]=newJTf[1];
            lambda = std::max(lambda*0.5, 1e-12);
            if (std::sqrt(dcx*dcx+dcy*dcy) < 1e-10) break;   // 收敛
        } else {
            lambda *= 2.0;
            if (lambda > 1e12) break;   // 阻尼已经大到没有意义，放弃继续迭代
        }
    }

    out.center = {cx, cy};
    out.radius = knownRadius;
    out.residual = circlefit_detail::geometricResidualRMS(pts, out.center, out.radius);
    out.valid = true;
    return out;
}

// 便捷封装：不知道半径时用自由拟合的中心当起点，一步做完约束拟合。
inline CircleFitResult fitCircleKnownRadiusAuto(const std::vector<Point2>& pts, double knownRadius) {
    Point2 seed{0,0};
    if (!pts.empty()) {
        for (const auto& p : pts) { seed.x += p.x; seed.y += p.y; }
        seed.x /= double(pts.size()); seed.y /= double(pts.size());
        // 若点分布在一段弧上，质心会偏向圆心的"弧那一侧"，用自由拟合的结果通常
        // 更接近真值；能拟合就优先用它当起点，拟合失败（比如弧太短退化）再退到质心。
        const auto freeFit = fitCircleFree(pts);
        if (freeFit.valid) seed = freeFit.center;
    }
    return fitCircleKnownRadius(pts, knownRadius, seed);
}

} // namespace mocap
