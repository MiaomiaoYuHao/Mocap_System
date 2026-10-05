// GateRadiusEstimator.hpp 单元测试——合成数据验证"从手部模板几何推算门控
// 半径"这套数学。
//
// 覆盖范围：
//   1. 基本正确性：构造一个已知最小间距的玩具FK，验证推算出的最小间距
//      跟真值吻合，且建议门控半径 = 最小间距*折扣/工作距离，算术对得上。
//   2. 手指越靠近关节限位上界(攥拳姿态)，marker越容易靠近——验证"半屈/
//      攥拳"姿态确实比"伸直"姿态发现更小的最小间距(如果只采样伸直姿态，
//      会得出一个虚假偏大、不安全的门控半径建议)。
//   3. 参数校验：工作距离/安全折扣给不合理的值时诚实报错。
//   4. FK产出空列表时诚实报错，不崩溃。
#include "estimate/GateRadiusEstimator.hpp"
#include <cstdio>
#include <cmath>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

int main() {
    // ---- 场景 1：玩具FK——两个marker，距离随第一个关节角变化(角度为0时
    // 距离最大=100mm，角度到jointLimits上限时距离缩小到已知的20mm)，
    // 验证推算能找到这个最小值、算术正确。----
    {
        ForwardKinematicsFn fk = [](const std::vector<double>& q) -> std::vector<Vec3> {
            // 用q[0]从0到jointLimits[0].hi(=1.0，见HandModel.hpp的拇指CMC屈
            // 限位)线性插值距离：0时100mm，1.0(上限)时20mm。
            const double t = q[0] / 1.0;   // 假设q[0]的上限就是1.0
            const double dist = 100.0 - t*80.0;
            return { Vec3{0,0,0}, Vec3{dist,0,0} };
        };
        const auto result = estimateGateRadius(fk, /*workingDistanceMm=*/500.0, /*marginFraction=*/0.4);
        CHECK(result.valid, "basic scene: estimation succeeds");
        char msg[200];
        std::snprintf(msg, sizeof(msg), "basic scene: found minimum distance close to the known 20mm minimum (got %.2fmm)", result.minMarkerDistMm);
        CHECK(std::abs(result.minMarkerDistMm - 20.0) < 1e-6, msg);

        const double expectedGate = (20.0 * 0.4) / 500.0;
        std::snprintf(msg, sizeof(msg), "basic scene: recommended gate radius matches the formula (expected=%.5f got=%.5f)", expectedGate, result.recommendedGateRadiusNorm);
        CHECK(std::abs(result.recommendedGateRadiusNorm - expectedGate) < 1e-9, msg);
    }

    // ---- 场景 2：只采样"伸直"姿态会漏掉攥拳时更小的间距——验证代表性
    // 姿态采样(半屈/攥拳)确实起作用，不是只测了q=0这一个点。----
    {
        ForwardKinematicsFn fk = [](const std::vector<double>& q) -> std::vector<Vec3> {
            // 距离在q[0]=0(伸直)时是100mm(偏大、看似安全)，但在攥拳姿态
            // (q[0]=jointLimits上限)时骤降到5mm——如果推算只测了伸直姿态，
            // 会给出一个虚假偏大、实际攥拳时会关联错位的门控半径建议。
            const double t = q[0] / 1.0;
            const double dist = 100.0 - t*95.0;
            return { Vec3{0,0,0}, Vec3{dist,0,0} };
        };
        const auto result = estimateGateRadius(fk, 500.0, 0.4);
        CHECK(result.valid, "fist-posture scene: estimation succeeds");
        char msg[220];
        std::snprintf(msg, sizeof(msg),
            "fist-posture scene: correctly finds the small fist-posture distance (5mm), not the misleadingly-large extended-posture distance (100mm) (got %.2fmm)",
            result.minMarkerDistMm);
        CHECK(result.minMarkerDistMm < 10.0, msg);
    }

    // ---- 场景 3：参数校验——工作距离/安全折扣给不合理的值时诚实报错。----
    {
        ForwardKinematicsFn fk = [](const std::vector<double>&) -> std::vector<Vec3> { return { Vec3{0,0,0}, Vec3{50,0,0} }; };
        auto r1 = estimateGateRadius(fk, /*workingDistanceMm=*/0.0, 0.4);
        CHECK(!r1.valid, "invalid working distance (0mm): honestly reports invalid");
        auto r2 = estimateGateRadius(fk, 500.0, /*marginFraction=*/1.5);
        CHECK(!r2.valid, "invalid margin fraction (>1): honestly reports invalid");
        auto r3 = estimateGateRadius(fk, 500.0, /*marginFraction=*/0.0);
        CHECK(!r3.valid, "invalid margin fraction (0): honestly reports invalid");
    }

    // ---- 场景 4：FK产出空列表——诚实报错，不崩溃。----
    {
        ForwardKinematicsFn fk = [](const std::vector<double>&) -> std::vector<Vec3> { return {}; };
        auto result = estimateGateRadius(fk, 500.0, 0.4);
        CHECK(!result.valid, "empty FK output: honestly reports invalid, no crash");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
