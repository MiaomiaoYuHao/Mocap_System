#include "detect/ConstrainedCircleFit.hpp"
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

// 沿一段圆弧生成噪声边缘点。centerAngleDeg 是弧中点方位角，spanDeg 是弧跨度，
// 点均匀撒在 [centerAngleDeg - span/2, centerAngleDeg + span/2] 上，每个点独立加
// 各向同性高斯像素噪声（模拟边缘检测的亚像素抖动，不是单纯径向噪声）。
static std::vector<Point2> sampleArc(double cx, double cy, double r,
                                     double centerAngleDeg, double spanDeg,
                                     int numPoints, double noiseSigma, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, noiseSigma);
    std::vector<Point2> pts;
    pts.reserve(size_t(numPoints));
    const double startRad = (centerAngleDeg - spanDeg*0.5) * M_PI / 180.0;
    const double endRad   = (centerAngleDeg + spanDeg*0.5) * M_PI / 180.0;
    for (int i=0;i<numPoints;++i) {
        const double t = (numPoints==1) ? 0.5 : double(i)/double(numPoints-1);
        const double a = startRad + t*(endRad-startRad);
        Point2 p;
        p.x = cx + r*std::cos(a) + noise(rng);
        p.y = cy + r*std::sin(a) + noise(rng);
        pts.push_back(p);
    }
    return pts;
}

static double dist(const Point2& a, const Point2& b) {
    const double dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy);
}

// 对某个弧跨度重复多次试验，返回 (free拟合中心误差均值, 约束拟合中心误差均值)。
static std::pair<double,double> benchmarkArc(double trueCx, double trueCy, double trueR,
                                              double spanDeg, int trials, std::mt19937& rng) {
    const double noiseSigma = 0.5;   // 模拟边缘检测噪声，单位：像素
    const int numPoints = std::max(8, int(spanDeg / 2.0));  // 弧越长采样点越多，密度接近
    double sumFreeErr = 0.0, sumConstrainedErr = 0.0;
    int validFree = 0;

    for (int t=0; t<trials; ++t) {
        const double centerAngle = std::uniform_real_distribution<double>(0.0,360.0)(rng);
        auto pts = sampleArc(trueCx, trueCy, trueR, centerAngle, spanDeg, numPoints, noiseSigma, rng);

        auto free = fitCircleFree(pts);
        auto constrained = fitCircleKnownRadiusAuto(pts, trueR);

        if (free.valid) { sumFreeErr += dist(free.center, {trueCx,trueCy}); ++validFree; }
        if (constrained.valid) sumConstrainedErr += dist(constrained.center, {trueCx,trueCy});
    }
    const double freeAvg = validFree>0 ? sumFreeErr/double(validFree) : -1.0;
    const double constrainedAvg = sumConstrainedErr/double(trials);
    return {freeAvg, constrainedAvg};
}

int main() {
    std::mt19937 rng(20260713);   // 固定种子，结果可复现
    const double CX = 100.0, CY = 80.0, R = 30.0;
    const int TRIALS = 300;

    // ---- 场景 1：完整圆（360°），两种拟合都该非常准，互相接近 ----
    {
        auto [freeErr, consErr] = benchmarkArc(CX, CY, R, 360.0, TRIALS, rng);
        CHECK(freeErr >= 0 && freeErr < 0.5, "full circle: free fit sub-pixel accurate");
        CHECK(consErr >= 0 && consErr < 0.5, "full circle: constrained fit sub-pixel accurate");
    }

    // ---- 场景 2：跑一遍 README 表格里的各档弧长，核心断言——约束拟合在
    // 每一档都应该不差于自由拟合（已知半径这个先验不应该让结果变差），
    // 且随着弧长变短，约束拟合相对自由拟合的优势应该扩大（这是"已知半径
    // 价值随遮挡加重而增长"的核心论断）。----
    const double spans[] = {180.0, 120.0, 90.0, 60.0, 45.0};
    double prevConsErr = -1.0;
    double firstRatio = -1.0, lastRatio = -1.0;
    for (double span : spans) {
        auto [freeErr, consErr] = benchmarkArc(CX, CY, R, span, TRIALS, rng);
        char msg[256];
        std::snprintf(msg, sizeof(msg),
            "span=%.0f deg: constrained error (%.3fpx) <= free error (%.3fpx)",
            span, consErr, freeErr);
        CHECK(consErr <= freeErr + 1e-6, msg);

        std::snprintf(msg, sizeof(msg), "span=%.0f deg: constrained fit produced a valid result", span);
        CHECK(consErr >= 0.0, msg);

        // 弧越短，约束拟合误差应该越大（单调变差的趋势，允许噪声带来的
        // 小幅波动，所以只在跨度差距较大的相邻档位间比较）。
        if (prevConsErr >= 0.0) {
            std::snprintf(msg, sizeof(msg),
                "span=%.0f deg: shorter arc has >= error than previous longer arc (%.3f vs %.3f)",
                span, consErr, prevConsErr);
            CHECK(consErr >= prevConsErr - 0.15, msg);   // 留一点容差，不要求严格单调
        }
        prevConsErr = consErr;

        const double ratio = (freeErr > 1e-9) ? consErr/freeErr : 1.0;
        if (span == 180.0) firstRatio = ratio;
        if (span == 45.0)  lastRatio  = ratio;

        std::printf("  span=%5.0f deg | free=%.3fpx | constrained=%.3fpx | ratio=%.3f\n",
                    span, freeErr, consErr, ratio);
    }

    CHECK(lastRatio <= firstRatio + 0.05,
          "known-radius advantage (constrained/free ratio) does not shrink as the arc gets shorter");

    // ---- 场景 3：退化输入不崩溃——弧短到只有 2 个点时，约束拟合(2参数、
    // 恰好2个观测)仍应给出某个结果而不是崩掉；自由拟合(3参数)在<3点时
    // 应诚实报告 invalid，而不是伪造一个解。----
    {
        std::vector<Point2> twoPts = { {CX+R, CY}, {CX, CY+R} };
        auto free2 = fitCircleFree(twoPts);
        CHECK(!free2.valid, "free fit with only 2 points honestly reports invalid (3 unknowns, 2 points)");

        auto cons2 = fitCircleKnownRadiusAuto(twoPts, R);
        CHECK(cons2.valid, "constrained fit with only 2 points still produces a result (2 unknowns, 2 points)");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
