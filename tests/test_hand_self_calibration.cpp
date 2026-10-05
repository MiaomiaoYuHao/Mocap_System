#include "hand/HandSelfCalibration.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

int main() {
    std::mt19937 rng(20260721);
    std::uniform_real_distribution<double> flexDist(0.0, 1.4);
    std::uniform_real_distribution<double> abductDist(-0.25, 0.25);
    std::normal_distribution<double> noise(0.0, 0.15);   // 三角化噪声，mm量级
    std::bernoulli_distribution missOne(0.35);   // 35%概率某颗marker这一帧缺失(模拟遮挡/相机组合不够)
    std::bernoulli_distribution missAnother(0.20);

    // ---- 场景 1：真实结构参数明显不同于占位值，自然摆动(随机关节角组合)
    // + 部分帧部分marker缺失，验证依然能恢复出接近真值的结构参数。这是
    // 直接回答"是否需要所有相机同时看清"的检验:故意让不同帧缺不同的
    // marker，模拟"不同帧由不同相机组合三角化、覆盖不全"的真实情况。----
    {
        FingerParam trueParam;
        trueParam.anchor = {90, 5, -2};       // 明显不同于占位值 {85,20,0}
        trueParam.lengths = {38, 24, 19};     // 明显不同于占位值 {40,25,20}

        const int numFrames = 60;
        std::vector<FingerFrameObservation> frames;
        for (int f=0; f<numFrames; ++f) {
            const double mcpFlex = flexDist(rng);
            const double mcpAbduct = abductDist(rng);
            const double pipFlex = flexDist(rng);

            std::array<double,3> truePos[3];
            fingerFK(trueParam, mcpFlex, mcpAbduct, pipFlex, truePos);

            FingerFrameObservation obs;
            for (int k=0;k<3;++k) {
                obs.hasMarker[size_t(k)] = true;
                for (int d=0;d<3;++d) obs.markerPos[size_t(k)][size_t(d)] = truePos[k][size_t(d)] + noise(rng);
            }
            if (missOne(rng)) obs.hasMarker[size_t(rng()%3)] = false;
            if (missAnother(rng)) {
                int k1 = int(rng()%3), k2 = int(rng()%3);
                obs.hasMarker[size_t(k1)] = false; obs.hasMarker[size_t(k2)] = false;
            }
            frames.push_back(obs);
        }

        int fullyVisibleCount=0, partialCount=0, droppedCount=0;
        for (auto& fr : frames) {
            const int vc = fr.validCount();
            if (vc==3) ++fullyVisibleCount; else if (vc>=2) ++partialCount; else ++droppedCount;
        }
        std::printf("帧统计: 全可见=%d 部分可见(仍可用)=%d 被丢弃(<2颗)=%d / 共%d帧\n",
                   fullyVisibleCount, partialCount, droppedCount, numFrames);
        CHECK(droppedCount > 0, "sanity: this test genuinely includes frames with insufficient visibility (not a trivial all-visible case)");
        CHECK(partialCount > 0, "sanity: this test genuinely includes partially-visible frames (only 2 of 3 markers)");

        FingerParam initialGuess = fingerParam(1);   // 用现成的占位值当起点(index finger)
        auto result = calibrateFingerStructure(frames, initialGuess);

        CHECK(result.valid, "self-calibration converges to a valid result despite missing markers across frames");
        if (result.valid) {
            char msg[160];
            const double dax = result.param.anchor[0]-trueParam.anchor[0];
            const double day = result.param.anchor[1]-trueParam.anchor[1];
            const double daz = result.param.anchor[2]-trueParam.anchor[2];
            const double anchorErr = std::sqrt(dax*dax+day*day+daz*daz);
            std::snprintf(msg, sizeof(msg), "recovered anchor close to truth (err=%.3fmm)", anchorErr);
            CHECK(anchorErr < 1.0, msg);

            for (int i=0;i<3;++i) {
                std::snprintf(msg, sizeof(msg), "recovered length[%d] close to truth (got=%.2f true=%.2f)",
                             i, result.param.lengths[size_t(i)], trueParam.lengths[size_t(i)]);
                CHECK(std::abs(result.param.lengths[size_t(i)]-trueParam.lengths[size_t(i)]) < 1.0, msg);
            }
            std::snprintf(msg, sizeof(msg), "fit residual RMS is small (%.3fmm)", result.rmsMm);
            CHECK(result.rmsMm < 1.0, msg);
        }
    }

    // ---- 场景 2：拇指(4自由度)结构自标定，同样有缺失帧。----
    {
        FingerParam trueParam;
        trueParam.anchor = {32, 38, 6};
        trueParam.lengths = {42, 28, 24};

        const int numFrames = 80;
        std::vector<ThumbFrameObservation> frames;
        std::uniform_real_distribution<double> cmcFlexD(-0.4,0.9), cmcAbductD(-0.4,0.7),
                                                mcpD(0.0,1.3), ipD(0.0,1.3);
        for (int f=0; f<numFrames; ++f) {
            const double a0=cmcFlexD(rng), a1=cmcAbductD(rng), a2=mcpD(rng), a3=ipD(rng);
            std::array<double,3> truePos[3];
            thumbFK(trueParam, a0, a1, a2, a3, truePos);
            ThumbFrameObservation obs;
            for (int k=0;k<3;++k) { obs.hasMarker[size_t(k)]=true;
                for (int d=0;d<3;++d) obs.markerPos[size_t(k)][size_t(d)] = truePos[k][size_t(d)] + noise(rng); }
            if (missOne(rng)) obs.hasMarker[size_t(rng()%3)] = false;
            frames.push_back(obs);
        }

        FingerParam initialGuess = fingerParam(0);
        auto result = calibrateThumbStructure(frames, initialGuess);
        CHECK(result.valid, "thumb self-calibration converges despite missing markers");
        if (result.valid) {
            char msg[160];
            const double dax = result.param.anchor[0]-trueParam.anchor[0];
            const double day = result.param.anchor[1]-trueParam.anchor[1];
            const double daz = result.param.anchor[2]-trueParam.anchor[2];
            const double anchorErr = std::sqrt(dax*dax+day*day+daz*daz);
            std::snprintf(msg, sizeof(msg), "thumb: recovered anchor close to truth (err=%.3fmm)", anchorErr);
            CHECK(anchorErr < 1.5, msg);
            for (int i=0;i<3;++i) {
                std::snprintf(msg, sizeof(msg), "thumb: recovered length[%d] close to truth (got=%.2f true=%.2f)",
                             i, result.param.lengths[size_t(i)], trueParam.lengths[size_t(i)]);
                CHECK(std::abs(result.param.lengths[size_t(i)]-trueParam.lengths[size_t(i)]) < 2.0, msg);
            }
        }
    }

    // ---- 场景 3：帧数太少或者关节角度覆盖太窄(比如手指几乎没怎么动)——
    // 应该诚实报告不可信，而不是硬凑一个不准的结果冒充标定成功。----
    {
        FingerParam trueParam; trueParam.anchor={90,5,-2}; trueParam.lengths={38,24,19};
        std::vector<FingerFrameObservation> frames;
        for (int f=0; f<2; ++f) {
            std::array<double,3> truePos[3];
            fingerFK(trueParam, 0.5, 0.0, 0.5, truePos);
            FingerFrameObservation obs;
            for (int k=0;k<3;++k) { obs.hasMarker[size_t(k)]=true;
                for (int d=0;d<3;++d) obs.markerPos[size_t(k)][size_t(d)] = truePos[k][size_t(d)]; }
            frames.push_back(obs);
        }
        auto result = calibrateFingerStructure(frames, fingerParam(1));
        CHECK(!result.valid, "too few frames / no motion diversity: honestly reports invalid, not a fabricated result");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
