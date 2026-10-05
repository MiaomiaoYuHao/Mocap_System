// FingerKinematicCalib.hpp 单元测试——合成数据验证"给一根手指若干帧观测，
// 标定出锚点/连杆长度(+拇指根部角)+每帧关节角"这套交替最小二乘拟合。
//
// 覆盖范围：
//   1. 普通四指：合成真实anchor/lengths + 覆盖足够ROM的随机关节角序列，
//      加噪声，标定应恢复出接近真值的anchor/lengths，残差应很小。
//   2. 拇指：同上，额外验证baseRotZ(根部方向角)也被正确标定出来，而不是
//      停留在45度初始值不动。
//   3. 帧数不足：诚实返回invalid。
//   4. 屈伸范围过窄(所有帧关节角几乎相同)：诚实返回invalid，即使残差很小
//      也不能采信(连杆长度和屈曲角在这种退化情况下有耦合歧义)。
//   5. 残差过大(观测点被人为加了较大的系统性偏移，模拟"贴球错位/分组
//      错误")：诚实返回invalid。
#include "estimate/FingerKinematicCalib.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static double dist3(const Vec3& a, const Vec3& b) {
    const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

int main() {
    std::mt19937 rng(20260723);
    std::normal_distribution<double> noise(0.0, 0.3);   // mm，模拟三角化+关联噪声

    // ---- 场景 1：普通手指(以中指的真实参数为真值)——覆盖良好ROM，标定
    // 应恢复出接近真值的anchor/lengths。----
    {
        FingerChainParams truth;
        truth.anchor = {88, 3, 0};
        truth.lengths = {45, 27, 22};

        std::uniform_real_distribution<double> flexRange(0.05, 1.3);
        std::uniform_real_distribution<double> abductRange(-0.15, 0.15);
        std::uniform_real_distribution<double> pipRange(0.05, 1.6);

        const int F = 200;
        const size_t Fsz = size_t(F);
        std::vector<std::array<Vec3,3>> obs(Fsz);
        for (int f=0; f<F; ++f) {
            const double mcpFlex = flexRange(rng), mcpAbduct = abductRange(rng), pipFlex = pipRange(rng);
            auto pred = fingerChainFK(truth, mcpFlex, mcpAbduct, pipFlex);
            for (int k=0;k<3;++k) {
                pred[size_t(k)][0] += noise(rng); pred[size_t(k)][1] += noise(rng); pred[size_t(k)][2] += noise(rng);
            }
            obs[size_t(f)] = pred;
        }

        const auto result = calibrateFingerChain(obs);
        CHECK(result.valid, "regular finger: calibration succeeds with good ROM coverage");
        if (result.valid) {
            const double anchorErr = dist3(result.params.anchor, truth.anchor);
            char msg[160];
            std::snprintf(msg, sizeof(msg), "regular finger: recovered anchor close to truth (err=%.3fmm)", anchorErr);
            CHECK(anchorErr < 2.0, msg);

            double maxLenErr = 0.0;
            for (int i=0;i<3;++i) maxLenErr = std::max(maxLenErr, std::abs(result.params.lengths[size_t(i)]-truth.lengths[size_t(i)]));
            char msg2[160];
            std::snprintf(msg2, sizeof(msg2), "regular finger: recovered segment lengths close to truth (maxErr=%.3fmm)", maxLenErr);
            CHECK(maxLenErr < 2.0, msg2);

            char msg3[160];
            std::snprintf(msg3, sizeof(msg3), "regular finger: fit residual is small (%.3fmm)", result.rmsMm);
            CHECK(result.rmsMm < 1.0, msg3);
        }
    }

    // ---- 场景 2：拇指——同上，额外验证baseRotZ(根部方向角)被正确标定，
    // 用一个明显偏离45度初始值的真值，确认不是停留在初值不动。----
    {
        ThumbChainParams truth;
        truth.anchor = {30, 40, 5};
        truth.baseRotZ = 0.65;   // 明显不是pi/4(=0.785)，验证真的在拟合它而不是抄初值
        truth.lengths = {40, 30, 25};

        std::uniform_real_distribution<double> cmcFlexR(0.05, 0.9), cmcAbductR(-0.3, 0.3);
        std::uniform_real_distribution<double> mcpFlexR(0.05, 1.2), ipFlexR(0.05, 1.2);

        const int F = 220;
        const size_t Fsz = size_t(F);
        std::vector<std::array<Vec3,3>> obs(Fsz);
        for (int f=0; f<F; ++f) {
            auto pred = thumbChainFK(truth, cmcFlexR(rng), cmcAbductR(rng), mcpFlexR(rng), ipFlexR(rng));
            for (int k=0;k<3;++k) { pred[size_t(k)][0]+=noise(rng); pred[size_t(k)][1]+=noise(rng); pred[size_t(k)][2]+=noise(rng); }
            obs[size_t(f)] = pred;
        }

        const auto result = calibrateThumbChain(obs);
        CHECK(result.valid, "thumb: calibration succeeds with good ROM coverage");
        if (result.valid) {
            const double anchorErr = dist3(result.params.anchor, truth.anchor);
            char msg[160];
            std::snprintf(msg, sizeof(msg), "thumb: recovered anchor close to truth (err=%.3fmm)", anchorErr);
            CHECK(anchorErr < 3.0, msg);

            const double rotErrDeg = std::abs(result.params.baseRotZ - truth.baseRotZ) * 180.0 / M_PI;
            char msg2[200];
            std::snprintf(msg2, sizeof(msg2),
                "thumb: baseRotZ recovered close to truth, not stuck at 45-degree initial guess (truth=%.1fdeg fitted=%.1fdeg)",
                truth.baseRotZ*180.0/M_PI, result.params.baseRotZ*180.0/M_PI);
            CHECK(rotErrDeg < 3.0, msg2);

            double maxLenErr = 0.0;
            for (int i=0;i<3;++i) maxLenErr = std::max(maxLenErr, std::abs(result.params.lengths[size_t(i)]-truth.lengths[size_t(i)]));
            char msg3[160];
            std::snprintf(msg3, sizeof(msg3), "thumb: recovered segment lengths close to truth (maxErr=%.3fmm)", maxLenErr);
            CHECK(maxLenErr < 4.5, msg3);
        }
    }

    // ---- 场景 3：帧数不足——诚实返回invalid。----
    {
        FingerChainParams truth; truth.anchor={85,20,0}; truth.lengths={40,25,20};
        std::vector<std::array<Vec3,3>> obs(5);
        for (int f=0;f<5;++f) obs[size_t(f)] = fingerChainFK(truth, 0.2, 0.0, 0.3);
        const auto result = calibrateFingerChain(obs);
        CHECK(!result.valid, "insufficient frames: honestly reports invalid");
    }

    // ---- 场景 4：屈伸范围过窄(所有帧关节角几乎相同，模拟采集时没怎么
    // 动手指)——即使残差很小也不该采信，连杆长度在这种退化情况下有
    // 耦合歧义(近似静止时，"锚点稍微挪一点+连杆稍微变一点"可以同样
    // 拟合出很小的残差，但解不唯一)。----
    {
        FingerChainParams truth; truth.anchor={85,20,0}; truth.lengths={40,25,20};
        const int F = 100;
        const size_t Fsz = size_t(F);
        std::vector<std::array<Vec3,3>> obs(Fsz);
        std::normal_distribution<double> tinyJitter(0.0, 0.01);   // 关节角几乎不变，只有极小噪声抖动
        for (int f=0; f<F; ++f) {
            auto pred = fingerChainFK(truth, 0.5+tinyJitter(rng), 0.0, 0.5+tinyJitter(rng));
            for (int k=0;k<3;++k) { pred[size_t(k)][0]+=noise(rng); pred[size_t(k)][1]+=noise(rng); pred[size_t(k)][2]+=noise(rng); }
            obs[size_t(f)] = pred;
        }
        const auto result = calibrateFingerChain(obs);
        CHECK(!result.valid, "insufficient range of motion: honestly reports invalid even if residual looks small");
    }

    // ---- 场景 5：残差过大(模拟贴球错位/分组错误——观测点被加了较大的
    // 系统性偏移，不是这根手指真实的运动学模型能解释的)。----
    {
        FingerChainParams truth; truth.anchor={85,20,0}; truth.lengths={40,25,20};
        std::uniform_real_distribution<double> flexRange(0.05, 1.3);
        std::uniform_real_distribution<double> pipRange(0.05, 1.6);
        const int F = 150;
        const size_t Fsz = size_t(F);
        std::vector<std::array<Vec3,3>> obs(Fsz);
        for (int f=0; f<F; ++f) {
            auto pred = fingerChainFK(truth, flexRange(rng), 0.0, pipRange(rng));
            // 只给远节点加一个明显的系统性偏移(比如那颗球其实贴错了位置，
            // 不满足"在这一节指骨中点"这个假设)。
            pred[2][0] += 15.0; pred[2][2] += 10.0;
            for (int k=0;k<3;++k) { pred[size_t(k)][0]+=noise(rng); pred[size_t(k)][1]+=noise(rng); pred[size_t(k)][2]+=noise(rng); }
            obs[size_t(f)] = pred;
        }
        const auto result = calibrateFingerChain(obs);
        CHECK(!result.valid, "systematic marker misplacement: residual exceeds threshold, honestly reports invalid");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
