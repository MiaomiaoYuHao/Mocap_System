// WandPrecisionStats.hpp 单元测试。
//
// 覆盖范围：
//   1. 基本正确性：已知偏置+已知噪声的合成样本，统计出的bias/rms/std
//      应该跟解析预期吻合。
//   2. 零误差回归：真值本身，distance恰好等于trueLength，bias/rms都应
//      该是0。
//   3. 系统性偏置(bias)能被正确识别，且RMS会同时反映偏置和随机噪声两部分
//      (RMS >= |bias|，这是RMS的数学性质，用来验证没有算错公式)。
//   4. 异常值过滤：距离小于1mm的样本被诚实丢弃，不污染统计。
//   5. addFrame(3D点版本)和addDistanceSample(距离版本)应该产出一致的结果。
//   6. reset()清空样本。
//   7. 空数据集：report()诚实返回invalid，不崩溃、不除0。
#include "calib/WandPrecisionStats.hpp"
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
    // ---- 场景 1：已知偏置+已知噪声标准差的合成数据，统计结果应该跟
    // 解析预期吻合(样本量大时，大数定律下应该很接近)。----
    {
        const double trueLength = 200.0;
        const double injectedBias = 1.5;      // 系统性多测1.5mm
        const double injectedNoiseStd = 0.3;  // 随机噪声标准差0.3mm

        WandPrecisionAccumulator acc(trueLength);
        std::mt19937 rng(42);
        std::normal_distribution<double> noise(0.0, injectedNoiseStd);

        const int N = 5000;
        for (int i=0;i<N;++i) acc.addDistanceSample(trueLength + injectedBias + noise(rng), int64_t(i)*8000000);

        const auto rep = acc.report();
        CHECK(rep.valid, "basic scene: report is valid with enough samples");
        CHECK(rep.sampleCount == N, "basic scene: sample count matches what was fed in");

        char msg[200];
        std::snprintf(msg, sizeof(msg), "basic scene: recovered bias close to injected value (expected~%.2f got=%.4f)", injectedBias, rep.biasMm);
        CHECK(std::abs(rep.biasMm - injectedBias) < 0.05, msg);   // 5000样本，大数定律下应该很接近

        std::snprintf(msg, sizeof(msg), "basic scene: recovered stddev close to injected noise level (expected~%.2f got=%.4f)", injectedNoiseStd, rep.stdDevMm);
        CHECK(std::abs(rep.stdDevMm - injectedNoiseStd) < 0.05, msg);

        // RMS理论上应该约等于 sqrt(bias^2 + noiseStd^2)(偏置和随机噪声的
        // 均方根合成，标准的偏置-方差分解)。
        const double expectedRms = std::sqrt(injectedBias*injectedBias + injectedNoiseStd*injectedNoiseStd);
        std::snprintf(msg, sizeof(msg), "basic scene: RMS error matches bias-variance decomposition (expected~%.3f got=%.4f)", expectedRms, rep.rmsErrorMm);
        CHECK(std::abs(rep.rmsErrorMm - expectedRms) < 0.05, msg);
    }

    // ---- 场景 2：零误差回归——测量值恰好等于真值，bias/rms都该是0。----
    {
        WandPrecisionAccumulator acc(150.0);
        for (int i=0;i<10;++i) acc.addDistanceSample(150.0, int64_t(i));
        const auto rep = acc.report();
        CHECK(rep.valid, "zero-error regression: valid report");
        CHECK(std::abs(rep.biasMm) < 1e-9, "zero-error regression: bias is exactly zero");
        CHECK(std::abs(rep.rmsErrorMm) < 1e-9, "zero-error regression: RMS error is exactly zero");
        CHECK(std::abs(rep.stdDevMm) < 1e-9, "zero-error regression: stddev is exactly zero (no variation)");
    }

    // ---- 场景 3：RMS必须 >= |bias|(数学性质，验证公式没写错——RMS包含
    // 偏置和随机噪声两部分的贡献，不可能比纯偏置本身还小)。----
    {
        WandPrecisionAccumulator acc(100.0);
        std::mt19937 rng(7);
        std::normal_distribution<double> noise(0.0, 2.0);
        for (int i=0;i<500;++i) acc.addDistanceSample(100.0 + 5.0 + noise(rng), int64_t(i));
        const auto rep = acc.report();
        char msg[150];
        std::snprintf(msg, sizeof(msg), "RMS >= |bias| property holds (rms=%.3f bias=%.3f)", rep.rmsErrorMm, rep.biasMm);
        CHECK(rep.rmsErrorMm >= std::abs(rep.biasMm) - 1e-9, msg);
    }

    // ---- 场景 4：异常值(距离<1mm)被诚实丢弃，不污染统计。----
    {
        WandPrecisionAccumulator acc(200.0);
        acc.addDistanceSample(200.0, 0);
        acc.addDistanceSample(200.1, 1);
        acc.addDistanceSample(0.5, 2);    // 异常值，应该被丢弃(可能是三角化把两点判成同一个点)
        acc.addDistanceSample(199.9, 3);
        CHECK(acc.sampleCount() == 3, "outlier rejection: the <1mm sample is dropped, not counted");
    }

    // ---- 场景 5：addFrame(3D点)和addDistanceSample(距离)应该产出一致
    // 结果——3D点(0,0,0)和(200,0,0)距离恰好200mm，等价于直接喂200.0。----
    {
        WandPrecisionAccumulator accPoints(200.0);
        WandPrecisionAccumulator accDist(200.0);
        accPoints.addFrame({0,0,0}, {203,0,0}, 0);   // 距离203mm
        accDist.addDistanceSample(203.0, 0);
        const auto rp = accPoints.report();
        const auto rd = accDist.report();
        char msg[150];
        std::snprintf(msg, sizeof(msg), "3D-point and direct-distance APIs agree (points bias=%.6f direct bias=%.6f)", rp.biasMm, rd.biasMm);
        CHECK(std::abs(rp.biasMm - rd.biasMm) < 1e-9, msg);
    }

    // ---- 场景 6：reset()清空样本。----
    {
        WandPrecisionAccumulator acc(100.0);
        acc.addDistanceSample(100.0, 0);
        acc.addDistanceSample(101.0, 1);
        CHECK(acc.sampleCount() == 2, "reset test: 2 samples before reset");
        acc.reset();
        CHECK(acc.sampleCount() == 0, "reset test: 0 samples immediately after reset()");
    }

    // ---- 场景 7：空数据集，report()诚实报invalid，不崩溃、不除0。----
    {
        WandPrecisionAccumulator acc(100.0);
        const auto rep = acc.report();
        CHECK(!rep.valid, "empty dataset: report is honestly invalid");
        CHECK(rep.sampleCount == 0, "empty dataset: sample count is zero");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
