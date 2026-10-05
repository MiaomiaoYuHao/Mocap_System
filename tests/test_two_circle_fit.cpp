#include "detect/TwoCircleFit.hpp"
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

static double dist(const Point2& a, const Point2& b) {
    const double dx=a.x-b.x, dy=a.y-b.y;
    return std::sqrt(dx*dx+dy*dy);
}

// 生成两球重叠/相切轮廓的花生形边缘点：沿每个圆采样，剔除落在另一个圆内部
// 的部分(那段被另一颗球挡住/合并掉了，不是可见轮廓)，再加噪声。
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
            if (dist(p, other) < r) continue;   // 落在对方圆内部，不是可见轮廓
            pts.push_back({ p.x + noise(rng), p.y + noise(rng) });
        }
    }
    return pts;
}

static std::vector<Point2> sampleSingleCircle(const Point2& c, double r, int samples,
                                              double noiseSigma, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, noiseSigma);
    std::vector<Point2> pts;
    for (int i=0;i<samples;++i) {
        const double a = 2.0*M_PI*double(i)/double(samples);
        pts.push_back({ c.x + r*std::cos(a) + noise(rng), c.y + r*std::sin(a) + noise(rng) });
    }
    return pts;
}

int main() {
    std::mt19937 rng(20260715);
    const double R = 25.0;
    const Point2 BASE{200.0, 150.0};

    // ---- 场景 1：双圆联合拟合精度——几档不同的圆心间距(相切到明显重叠)，
    // 拟合出的两个圆心都应该接近真值(允许两个圆心的编号顺序互换)。----
    {
        const double seps[] = {1.3, 1.5, 1.7, 1.9};   // 圆心距 / 半径
        for (double sepRatio : seps) {
            const double d = sepRatio * R;
            const Point2 cA{ BASE.x - d*0.5, BASE.y };
            const Point2 cB{ BASE.x + d*0.5, BASE.y };
            auto pts = samplePeanut(cA, cB, R, 90, 0.2, rng);

            auto fit = fitTwoCirclesKnownRadius(pts, R);
            char msg[160];
            std::snprintf(msg, sizeof(msg), "sep=%.1fR: two-circle fit converged (%zu pts)", sepRatio, pts.size());
            CHECK(fit.valid, msg);
            if (!fit.valid) continue;

            // 圆心编号顺序不保证跟真值一致，取误差较小的配对方式。
            const double errSame = dist(fit.center1,cA)+dist(fit.center2,cB);
            const double errSwap = dist(fit.center1,cB)+dist(fit.center2,cA);
            const double bestErr = std::min(errSame, errSwap) / 2.0;   // 平均每个圆心的误差

            std::snprintf(msg, sizeof(msg), "sep=%.1fR: recovered centers close to truth (avg err=%.3fpx)", sepRatio, bestErr);
            CHECK(bestErr < 1.0, msg);
        }
    }

    // ---- 场景 2：模型选择在花生形场景下应该判定 Double，且恢复出的圆心
    // 接近真值。----
    {
        const double seps[] = {1.3, 1.5, 1.7, 1.9};
        for (double sepRatio : seps) {
            const double d = sepRatio * R;
            const Point2 cA{ BASE.x - d*0.5, BASE.y + 30.0 };  // 换个位置避免跟场景1完全重复
            const Point2 cB{ BASE.x + d*0.5, BASE.y + 30.0 };
            auto pts = samplePeanut(cA, cB, R, 90, 0.2, rng);

            auto sel = selectBallModel(pts, R);
            char msg[160];
            std::snprintf(msg, sizeof(msg), "sep=%.1fR: model selection picks Double", sepRatio);
            CHECK(sel.valid && sel.model==BallModel::Double, msg);
        }
    }

    // ---- 场景 3：真正的单球(带正常噪声的完整圆)不该被误判成双球——这是
    // 这一层最重要的假阳性防线。----
    {
        for (int trial=0; trial<10; ++trial) {
            const Point2 c{ BASE.x + double(trial)*3.0, BASE.y - double(trial)*2.0 };
            auto pts = sampleSingleCircle(c, R, 80, 0.3, rng);
            auto sel = selectBallModel(pts, R);
            char msg[128];
            std::snprintf(msg, sizeof(msg), "single ball trial %d: correctly classified Single", trial);
            CHECK(sel.valid && sel.model==BallModel::Single, msg);
        }
    }

    // ---- 场景 4：单球 + 少量离群点(比如反光噪声/另一个不相关的小亮斑边缘
    // 混进了同一个连通域)不该被误判成第二个球——平衡度门槛要能拦住"主圆 +
    // 几个杂点凑出的伪双圆"。----
    {
        auto pts = sampleSingleCircle(BASE, R, 80, 0.2, rng);
        // 加 5 个离群点，聚在远处一角，数量远小于主圆的 80 个点。
        std::normal_distribution<double> noise(0.0, 1.0);
        for (int i=0;i<5;++i) pts.push_back({ BASE.x + R*2.5 + noise(rng), BASE.y + R*2.5 + noise(rng) });

        auto sel = selectBallModel(pts, R);
        CHECK(sel.valid && sel.model==BallModel::Single,
              "single ball + small outlier cluster still classified Single (balance gate holds)");
    }

    // ---- 场景 5：退化输入不崩溃——点太少时应该诚实报告 invalid，而不是
    // 编造一个结果。----
    {
        std::vector<Point2> tiny = { {0,0}, {1,1}, {2,2} };
        auto fit = fitTwoCirclesKnownRadius(tiny, R);
        CHECK(!fit.valid, "too few points: two-circle fit honestly reports invalid");
        auto sel = selectBallModel(tiny, R);
        CHECK(!sel.valid, "too few points: model selection honestly reports invalid");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
