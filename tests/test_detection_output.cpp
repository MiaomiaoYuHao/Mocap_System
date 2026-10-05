#include "detect/DetectionOutput.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static double dist(const Point2& a, const Point2& b) {
    const double dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy);
}

static std::vector<Point2> sampleArc(double cx, double cy, double r,
                                     double centerAngleDeg, double spanDeg,
                                     int numPoints, double noiseSigma, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, noiseSigma);
    std::vector<Point2> pts;
    const double startRad = (centerAngleDeg - spanDeg*0.5) * M_PI / 180.0;
    const double endRad   = (centerAngleDeg + spanDeg*0.5) * M_PI / 180.0;
    for (int i=0;i<numPoints;++i) {
        const double t = (numPoints==1) ? 0.5 : double(i)/double(numPoints-1);
        const double a = startRad + t*(endRad-startRad);
        pts.push_back({ cx + r*std::cos(a) + noise(rng), cy + r*std::sin(a) + noise(rng) });
    }
    return pts;
}

static std::vector<Point2> samplePeanut(const Point2& cA, const Point2& cB, double r,
                                        int samplesPerCircle, double noiseSigma, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, noiseSigma);
    std::vector<Point2> pts;
    for (int which=0; which<2; ++which) {
        const Point2& self = (which==0) ? cA : cB;
        const Point2& other = (which==0) ? cB : cA;
        for (int i=0;i<samplesPerCircle;++i) {
            const double a = 2.0*M_PI*double(i)/double(samplesPerCircle);
            const Point2 p{ self.x + r*std::cos(a), self.y + r*std::sin(a) };
            if (dist(p, other) < r) continue;
            pts.push_back({ p.x + noise(rng), p.y + noise(rng) });
        }
    }
    return pts;
}

int main() {
    std::mt19937 rng(20260716);
    const double R = 25.0;

    // ---- 场景 1：完整圆 -> Single 模型，1 个高置信度观测，协方差接近
    // 各向同性(完整圆没有哪个方向更不可信)。----
    {
        auto pts = sampleArc(200,150,R, 0.0, 359.0, 80, 0.15, rng);
        auto out = detectBalls(pts, R);
        CHECK(out.valid && out.model==BallModel::Single, "full circle: Single model, valid output");
        CHECK(out.observations.size()==1, "full circle: exactly one observation");
        if (out.observations.size()==1) {
            const auto& o = out.observations[0];
            CHECK(o.usable && o.confidence==ArcConfidence::High, "full circle: usable, High confidence");
            CHECK(dist(o.mu,{200,150}) < 0.5, "full circle: center close to truth");
            const double anisotropy = std::abs(o.sigma.xx - o.sigma.yy);
            CHECK(anisotropy < 0.05, "full circle: covariance nearly isotropic (xx~=yy)");
            CHECK(std::abs(o.sigma.xy) < 0.05, "full circle: negligible off-diagonal covariance");
        }
    }

    // ---- 场景 2：90° 弧 -> Single 模型，High 置信度，但协方差应该明显
    // 各向异性——径向(圆心到弧中点方向)方差应该大于切向方向。----
    {
        auto pts = sampleArc(300,200,R, 45.0, 90.0, 50, 0.15, rng);
        auto out = detectBalls(pts, R);
        CHECK(out.valid && out.observations.size()==1, "90deg arc: one observation produced");
        const auto& o = out.observations[0];
        CHECK(o.usable, "90deg arc: usable (>=90 threshold)");

        // 径向方向(圆心指向45度方位)近似 (cos45,sin45)；该方向上的方差应该
        // 大于垂直方向上的方差。用二次型直接算这两个方向各自的方差来比较。
        const double ux = std::cos(45.0*M_PI/180.0), uy = std::sin(45.0*M_PI/180.0);
        const double varRadial = o.sigma.xx*ux*ux + 2*o.sigma.xy*ux*uy + o.sigma.yy*uy*uy;
        const double tx = -uy, ty = ux;   // 垂直方向
        const double varTangential = o.sigma.xx*tx*tx + 2*o.sigma.xy*tx*ty + o.sigma.yy*ty*ty;
        CHECK(varRadial > varTangential, "90deg arc: radial variance exceeds tangential variance");
    }

    // ---- 场景 3：45° 弧 -> Single 模型，但 Discard、usable=false —— 契约
    // 要求即使不可信也要原样报告出来，不能悄悄丢掉这个观测。----
    {
        auto pts = sampleArc(150,150,R, 200.0, 45.0, 40, 0.15, rng);
        auto out = detectBalls(pts, R);
        CHECK(out.valid, "45deg arc: output still valid (detection ran, just low confidence)");
        CHECK(out.observations.size()==1, "45deg arc: observation still present (not silently dropped)");
        if (!out.observations.empty()) {
            CHECK(!out.observations[0].usable, "45deg arc: usable=false");
            CHECK(out.observations[0].confidence==ArcConfidence::Discard, "45deg arc: confidence=Discard");
        }
    }

    // ---- 场景 4：花生双球 -> Double 模型，2 个观测，各自圆心接近对应真值，
    // 且各自都有自己独立的置信度(不假设双球场景两个观测天然都可信)。----
    {
        const Point2 cA{300, 250}, cB{300 + 1.6*R, 250};
        auto pts = samplePeanut(cA, cB, R, 90, 0.2, rng);
        auto out = detectBalls(pts, R);
        CHECK(out.valid && out.model==BallModel::Double, "peanut: Double model");
        CHECK(out.observations.size()==2, "peanut: exactly two observations");
        if (out.observations.size()==2) {
            const double errSame = dist(out.observations[0].mu,cA)+dist(out.observations[1].mu,cB);
            const double errSwap = dist(out.observations[0].mu,cB)+dist(out.observations[1].mu,cA);
            CHECK(std::min(errSame,errSwap)/2.0 < 1.5, "peanut: both centers close to true ball centers");
            CHECK(out.observations[0].usable, "peanut: first ball observation usable");
            CHECK(out.observations[1].usable, "peanut: second ball observation usable");
        }
    }

    // ---- 场景 5：退化输入 -> 诚实报 invalid，不崩溃、不编造观测。----
    {
        std::vector<Point2> tiny = { {0,0} };
        auto out = detectBalls(tiny, R);
        CHECK(!out.valid, "single point input: honestly reports invalid");
        CHECK(out.observations.empty(), "single point input: no fabricated observations");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
