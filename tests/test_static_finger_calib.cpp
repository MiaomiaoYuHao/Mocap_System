// 静态姿势标定(calibrateFingerFromStaticPose)的验证：
//  1. 理想平放姿势：闭式解应基本还原真值
//  2. 现实情况：手放桌上仍有轻微屈曲(关节角不是精确的0)+观测噪声——
//     误差应保持在可用范围，共线性偏差字段应如实反映弯曲程度
//  3. 静态结果做初值 + 少量重启的动态精修：精度不输旧默认(6次重启)，
//     速度显著更快——这是标定流程改造(静态打底+可选精修)的数字依据
#include "hand/HandSelfCalibration.hpp"
#include <cstdio>
#include <cmath>
#include <chrono>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while (0)

static double vecErr(const std::array<double,3>& a, const std::array<double,3>& b) {
    const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

int main() {
    FingerParam truth;
    truth.anchor = {85.0, 20.0, 0.0};
    truth.lengths = {41.0, 26.5, 19.0};   // 故意跟默认模板(40,25,20)不同——检验能不能测出个体差异
    const FingerParam& prior = fingerParam(1);   // index的默认模板做比例先验

    // ---- 场景1：理想平放(角度全0，无噪声) ----
    {
        std::array<double,3> p[3];
        fingerFK(truth, 0.0, 0.0, 0.0, p);
        const auto r = calibrateFingerFromStaticPose(p[0], p[1], p[2], prior);
        CHECK(r.valid, "ideal flat: result valid");
        CHECK(r.rmsMm < 1e-9, "ideal flat: colinearity deviation ~0");
        CHECK(vecErr(r.param.anchor, truth.anchor) < 1.5,
              "ideal flat: anchor recovered within 1.5mm");
        double lenErr = 0;
        for (int i=0;i<3;++i) lenErr += std::abs(r.param.lengths[size_t(i)] - truth.lengths[size_t(i)]);
        std::printf("[理想平放] anchor误差=%.2fmm 长度误差和=%.2fmm 共线偏差=%.3fmm\n",
                    vecErr(r.param.anchor, truth.anchor), lenErr, r.rmsMm);
        CHECK(lenErr < 4.0, "ideal flat: total length error under 4mm (ratio prior residual)");
    }

    // ---- 场景2：现实平放(轻微屈曲5°/8°+0.3mm噪声，多帧平均模拟UI做法) ----
    double staticAnchorErr = 0, staticLenErr = 0;
    FingerParam staticResult{};
    {
        std::mt19937 rng(99);
        std::normal_distribution<double> noise(0.0, 0.3);
        std::array<double,3> avg[3] = {};
        const int N = 45;
        for (int f = 0; f < N; ++f) {
            std::array<double,3> p[3];
            fingerFK(truth, 5.0*M_PI/180.0, 0.0, 8.0*M_PI/180.0, p);   // 桌面上手指也不会绝对伸直
            for (int k=0;k<3;++k) for (int d2=0;d2<3;++d2) avg[k][size_t(d2)] += (p[k][size_t(d2)] + noise(rng)) / N;
        }
        const auto r = calibrateFingerFromStaticPose(avg[0], avg[1], avg[2], prior);
        CHECK(r.valid, "realistic flat: still valid with slight rest flexion");
        staticAnchorErr = vecErr(r.param.anchor, truth.anchor);
        for (int i=0;i<3;++i) staticLenErr += std::abs(r.param.lengths[size_t(i)] - truth.lengths[size_t(i)]);
        staticResult = r.param;
        std::printf("[现实平放5°/8°] anchor误差=%.2fmm 长度误差和=%.2fmm 共线偏差=%.2fmm\n",
                    staticAnchorErr, staticLenErr, r.rmsMm);
        CHECK(staticAnchorErr < 6.0, "realistic flat: anchor error acceptable for an instant result");
        CHECK(r.rmsMm > 0.1, "realistic flat: colinearity deviation honestly reports the bend");
    }

    // ---- 场景2b：手明显弯着(30°)标——必须被共线性门控拒绝 ----
    {
        std::array<double,3> p[3];
        fingerFK(truth, 30.0*M_PI/180.0, 0.0, 40.0*M_PI/180.0, p);
        const auto r = calibrateFingerFromStaticPose(p[0], p[1], p[2], prior);
        CHECK(!r.valid, "bent hand: rejected by colinearity gate (user told to flatten hand)");
    }

    // ---- 场景3：静态初值+2次重启 vs 默认模板初值+6次重启(旧默认) ----
    {
        std::mt19937 rng(1234);
        std::normal_distribution<double> noise(0.0, 0.3);
        std::vector<FingerFrameObservation> frames;
        const int numFrames = 150;
        for (int f = 0; f < numFrames; ++f) {
            const double t = double(f)/numFrames;
            const double a0 = 0.1 + 0.9*std::sin(2*M_PI*1.7*t);
            const double a1 = 0.05*std::sin(2*M_PI*0.9*t);
            const double a2 = 0.1 + 0.6*std::sin(2*M_PI*2.3*t);
            std::array<double,3> p[3];
            fingerFK(truth, a0, a1, a2, p);
            FingerFrameObservation obs;
            for (int k=0;k<3;++k) {
                obs.hasMarker[size_t(k)] = true;
                for (int d2=0;d2<3;++d2) obs.markerPos[size_t(k)][size_t(d2)] = p[k][size_t(d2)] + noise(rng);
            }
            frames.push_back(obs);
        }

        auto t0 = std::chrono::steady_clock::now();
        const auto rOld = calibrateFingerStructure(frames, prior, 2, 12, 6);
        auto t1 = std::chrono::steady_clock::now();
        const auto rNew = calibrateFingerStructure(frames, staticResult, 2, 12, 2);
        auto t2 = std::chrono::steady_clock::now();

        const double secOld = std::chrono::duration<double>(t1-t0).count();
        const double secNew = std::chrono::duration<double>(t2-t1).count();
        double lenOld=0, lenNew=0;
        for (int i=0;i<3;++i) {
            lenOld += std::abs(rOld.param.lengths[size_t(i)] - truth.lengths[size_t(i)]);
            lenNew += std::abs(rNew.param.lengths[size_t(i)] - truth.lengths[size_t(i)]);
        }
        std::printf("[旧: 模板初值+6重启] 耗时=%.1fs anchor误差=%.2f 长度误差和=%.2f rms=%.2f\n",
                    secOld, vecErr(rOld.param.anchor, truth.anchor), lenOld, rOld.rmsMm);
        std::printf("[新: 静态初值+2重启] 耗时=%.1fs anchor误差=%.2f 长度误差和=%.2f rms=%.2f\n",
                    secNew, vecErr(rNew.param.anchor, truth.anchor), lenNew, rNew.rmsMm);
        CHECK(rNew.valid, "static-init refine: valid");
        CHECK(secNew < secOld * 0.55, "static-init refine: at least ~2x faster than old default");
        CHECK(vecErr(rNew.param.anchor, truth.anchor) <= vecErr(rOld.param.anchor, truth.anchor) + 0.5,
              "static-init refine: accuracy not worse than old default");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
