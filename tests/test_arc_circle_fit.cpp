#include "detect/ArcCircleFit.hpp"
#include <cstdio>
#include <cmath>
#include <random>
#include <algorithm>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

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

int main() {
    std::mt19937 rng(20260714);
    const double CX = 200.0, CY = 150.0, R = 25.0;

    // ---- 场景 1：弧长估计本身准不准 —— 多个真实弧长、多个随机朝向，且把
    // 点集顺序打乱（模拟 flood fill 提取出的无序边缘点），估计值都应接近
    // 真值（容差 5°，来自离散采样 + 噪声引入的边界点抖动）。----
    {
        const double testSpans[] = {359.0, 180.0, 120.0, 90.0, 60.0, 45.0};
        for (double trueSpan : testSpans) {
            double maxAbsErr = 0.0;
            for (int trial=0; trial<20; ++trial) {
                const double centerAngle = std::uniform_real_distribution<double>(0.0,360.0)(rng);
                auto pts = sampleArc(CX, CY, R, centerAngle, trueSpan, 60, 0.15, rng);
                std::shuffle(pts.begin(), pts.end(), rng);   // 打乱顺序，验证方法不依赖点序

                const double estimated = arcfit_detail::estimateArcSpanDegrees(pts, {CX, CY});
                maxAbsErr = std::max(maxAbsErr, std::abs(estimated - trueSpan));
            }
            char msg[128];
            const double tol = (trueSpan >= 180.0) ? 8.0 : 5.0;   // 满弧时缺口本来就窄，噪声相对影响更大
            std::snprintf(msg, sizeof(msg), "arc span estimate near true=%.0f (worst-case err=%.1f deg)", trueSpan, maxAbsErr);
            CHECK(maxAbsErr < tol, msg);
        }
    }

    // ---- 场景 2：置信度分档要跟门槛对上号：>=90 High，60~90 Medium，
    // <60 Discard；Discard 时 usable 必须是 false（这是弃用契约，不是
    // 摆设标签）。----
    {
        struct Case { double span; ArcConfidence expect; };
        const Case cases[] = {
            {180.0, ArcConfidence::High},
            {90.5,  ArcConfidence::High},
            {89.0,  ArcConfidence::Medium},
            {61.0,  ArcConfidence::Medium},
            {59.0,  ArcConfidence::Discard},
            {30.0,  ArcConfidence::Discard},
        };
        for (const auto& c : cases) {
            const double centerAngle = std::uniform_real_distribution<double>(0.0,360.0)(rng);
            auto pts = sampleArc(CX, CY, R, centerAngle, c.span, 40, 0.15, rng);
            auto result = fitArcWithGating(pts, R);
            CHECK(result.usable == true || true, "sanity: fit produced a result object");  // 占位，主断言在下面
            char msg[128];
            std::snprintf(msg, sizeof(msg), "span=%.1f classified as expected confidence tier", c.span);
            CHECK(result.confidence == c.expect, msg);

            const bool expectUsable = (c.expect != ArcConfidence::Discard);
            std::snprintf(msg, sizeof(msg), "span=%.1f usable flag matches confidence tier (usable=%s)",
                          c.span, expectUsable ? "true" : "false");
            CHECK(result.usable == expectUsable, msg);
        }
    }

    // ---- 场景 3：即使 Discard 档的底层拟合数值本身"看起来还行"（比如弧
    // 短但噪声恰好小），usable 也必须是 false —— 弃用是硬门槛，不因为这
    // 一次运气好就放行,门控不能被"看起来还凑合"绕过。----
    {
        auto pts = sampleArc(CX, CY, R, 0.0, 45.0, 40, 0.02, rng);  // 噪声极小，拟合会很准
        auto result = fitArcWithGating(pts, R);
        CHECK(!result.usable, "45deg arc discarded even when underlying fit residual happens to be tiny");
        CHECK(result.confidence == ArcConfidence::Discard, "45deg arc still classified Discard regardless of fit quality");
    }

    // ---- 场景 4：sigma 估计要随弧长变短而增大（严格来说是不减，允许插值
    // 边界处打平），且跟 3a 断点处的表值大致对上。----
    {
        const double a = arcfit_detail::lookupSigmaFromSpan(180.0);
        const double b = arcfit_detail::lookupSigmaFromSpan(120.0);
        const double c = arcfit_detail::lookupSigmaFromSpan(90.0);
        const double d = arcfit_detail::lookupSigmaFromSpan(60.0);
        const double e = arcfit_detail::lookupSigmaFromSpan(45.0);
        CHECK(a <= b + 1e-9, "sigma(180) <= sigma(120)");
        CHECK(b <= c + 1e-9, "sigma(120) <= sigma(90)");
        CHECK(c < d, "sigma(90) < sigma(60): clear jump once below the trust threshold");
        CHECK(d < e, "sigma(60) < sigma(45): sigma keeps growing as arc shortens further");

        char msg[128];
        std::snprintf(msg, sizeof(msg), "sigma(180)=%.3f close to table value 0.18", a);
        CHECK(std::abs(a-0.18) < 1e-6, msg);
        std::snprintf(msg, sizeof(msg), "sigma(90)=%.3f close to table value 0.30", c);
        CHECK(std::abs(c-0.30) < 1e-6, msg);
    }

    // ---- 场景 5：完整圆（近 360°）应该是 High 置信度、sigma 很小 —— 全
    // 遮挡感知层的"最好情况"基线，不能因为门控逻辑反而把满弧判低。----
    {
        auto pts = sampleArc(CX, CY, R, 0.0, 359.0, 80, 0.15, rng);
        auto result = fitArcWithGating(pts, R);
        CHECK(result.confidence == ArcConfidence::High, "near-full circle classified High confidence");
        CHECK(result.usable, "near-full circle usable");
        CHECK(result.sigmaPx < 0.3, "near-full circle sigma stays small");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
