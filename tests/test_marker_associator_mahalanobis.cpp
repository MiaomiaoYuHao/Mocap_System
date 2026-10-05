// MarkerAssociator.hpp 自适应马氏距离门控测试。
//
// 覆盖范围：
//   1. 回归：低协方差、marker分得开的常规场景，关联结果应该跟旧的固定
//      半径行为一致(每个marker都关联对)。
//   2. 核心新行为——同一个欧氏距离的候选检测，协方差大时应该被接受
//      (原本euclid距离对固定半径来说太远，会被旧逻辑拒绝)，协方差小时
//      应该被拒绝——证明门控真的在跟着协方差自适应，不是摆设。
//   3. 硬性欧氏距离上限兜底——不管协方差算出来多大，候选对的实际像素
//      距离超过gateRadiusNorm就必须被拒绝，防止协方差异常时门控失控。
//   4. cov维度对不上(比如传空)时诚实退化成纯欧氏距离门控，不崩溃。
//   5. 检测自身的观测噪声(det.sigma)也应该影响门控——同样的状态协方差，
//      检测噪声大时更容易通过马氏检验。
#include "estimate/MarkerAssociator.hpp"
#include <cstdio>
#include <cmath>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

// 两个marker，局部系位置固定，不带任何关节角自由度(numJoints=0，简化
// 测试焦点在关联本身，不掺进FK雅可比的复杂度)。
static ForwardKinematicsFn twoMarkerFk() {
    return [](const std::vector<double>&) -> std::vector<Vec3> {
        return { Vec3{0,0,0}, Vec3{50,0,0} };
    };
}

static CamPose identityCamAt(double zOffset) {
    CamPose p;
    p.R = {1,0,0, 0,1,0, 0,0,1};
    p.t = {0,0,zOffset};
    return p;
}

static std::vector<std::vector<double>> makeCov(int n, double posVar, double rotVar, double jointVar) {
    std::vector<std::vector<double>> cov(size_t(n), std::vector<double>(size_t(n), 0.0));
    for (int i=0;i<3 && i<n;++i) cov[size_t(i)][size_t(i)] = posVar;
    for (int i=3;i<6 && i<n;++i) cov[size_t(i)][size_t(i)] = rotVar;
    for (int i=6;i<n;++i) cov[size_t(i)][size_t(i)] = jointVar;
    return cov;
}

int main() {
    // ---- 场景 1：回归——低协方差、marker分得开，应该正确关联每个marker。----
    {
        HandPoseState state;
        state.wristPos = {0,0,500};
        state.jointAngles = {};   // numJoints=0

        auto fk = twoMarkerFk();
        MarkerAssociator assoc(fk, /*gateRadiusNorm=*/0.05, /*chiSquareGate=*/9.21);

        const auto cov = makeCov(6, 1.0, 1e-6, 0.0);   // 很小的协方差(6=位置3+旋转3，numJoints=0)

        CameraFrame frame;
        frame.camIndex = 0;
        CamPose cam = identityCamAt(0);
        frame.cam = &cam;
        // marker0在(0,0,500)，marker1在(50,0,500)，投影到z=500平面附近，
        // 归一化坐标差值明显，检测跟真值完全重合(零噪声理想情况)。
        RawDetection d0; d0.nx = 0.0/500.0; d0.ny = 0.0/500.0; d0.sigma = {1e-6,0,1e-6};
        RawDetection d1; d1.nx = 50.0/500.0; d1.ny = 0.0/500.0; d1.sigma = {1e-6,0,1e-6};
        frame.detections = {d0, d1};

        const auto result = assoc.associate(state, cov, {frame});
        CHECK(result.measurements.size() == 2, "regression: both markers correctly associated");
        bool m0ok=false, m1ok=false;
        for (auto& m : result.measurements) {
            if (m.markerIndex==0 && std::abs(m.nx-d0.nx)<1e-9) m0ok=true;
            if (m.markerIndex==1 && std::abs(m.nx-d1.nx)<1e-9) m1ok=true;
        }
        CHECK(m0ok && m1ok, "regression: each marker matched to its correct detection, not swapped");
    }

    // ---- 场景 2(核心)：同样的候选检测偏移量，协方差大时接受、协方差小时拒绝。----
    {
        HandPoseState state;
        state.wristPos = {0,0,500};
        state.jointAngles = {};
        auto fk = twoMarkerFk();
        // gateRadiusNorm给得足够宽松(0.05)，不让硬性欧氏上限先拦下来，
        // 专门测马氏距离这一层本身的行为。
        MarkerAssociator assoc(fk, /*gateRadiusNorm=*/0.05, /*chiSquareGate=*/9.21);

        CameraFrame frame;
        frame.camIndex = 0;
        CamPose cam = identityCamAt(0);
        frame.cam = &cam;
        // 检测偏离marker0的真实期望位置一小段距离(在归一化坐标里，
        // 大致对应马氏距离在"协方差小时超出阈值、协方差大时在阈值内"
        // 的量级)。
        const double offsetNorm = 0.02;
        RawDetection d0; d0.nx = 0.0 + offsetNorm; d0.ny = 0.0; d0.sigma = {1e-8,0,1e-8};   // 检测自身噪声给到几乎为0，让状态协方差主导
        frame.detections = {d0};   // 只放1个检测，避免marker1抢走干扰判断

        // 小协方差：这个偏移量应该被拒绝。
        const auto covSmall = makeCov(6, 1e-8, 1e-10, 0.0);
        const auto resultSmall = assoc.associate(state, covSmall, {frame});
        CHECK(resultSmall.measurements.empty(), "small covariance: same offset rejected (too far relative to tight confidence)");

        // 大协方差：同一个偏移量应该被接受。
        const auto covLarge = makeCov(6, 50.0, 1e-10, 0.0);
        const auto resultLarge = assoc.associate(state, covLarge, {frame});
        CHECK(resultLarge.measurements.size() == 1 && resultLarge.measurements[0].markerIndex == 0,
              "large covariance: same offset accepted (wide uncertainty tolerates it) — proves the gate truly adapts, not a fixed radius");
    }

    // ---- 场景 3：硬性欧氏距离上限兜底——协方差给到离谱大，候选对的实际
    // 像素距离超过gateRadiusNorm还是必须被拒绝。----
    {
        HandPoseState state;
        state.wristPos = {0,0,500};
        state.jointAngles = {};
        auto fk = twoMarkerFk();
        MarkerAssociator assoc(fk, /*gateRadiusNorm=*/0.01, /*chiSquareGate=*/9.21);   // 硬上限给得很紧

        CameraFrame frame;
        frame.camIndex = 0;
        CamPose cam = identityCamAt(0);
        frame.cam = &cam;
        RawDetection d0; d0.nx = 0.0 + 0.05; d0.ny = 0.0; d0.sigma = {1e-8,0,1e-8};   // 偏移量(0.05)远超硬上限(0.01)
        frame.detections = {d0};

        // 协方差给到极大，马氏距离本身会认为这个偏移量"完全在容忍范围内"，
        // 但硬性欧氏上限应该照样拦住它。
        const auto covHuge = makeCov(6, 1e6, 1e-10, 0.0);
        const auto result = assoc.associate(state, covHuge, {frame});
        CHECK(result.measurements.empty(),
              "hard euclidean cap: rejected even with astronomically large covariance that would otherwise accept it via Mahalanobis alone");
    }

    // ---- 场景 4：cov维度对不上，诚实退化成纯欧氏距离门控，不崩溃。----
    {
        HandPoseState state;
        state.wristPos = {0,0,500};
        state.jointAngles = {};
        auto fk = twoMarkerFk();
        MarkerAssociator assoc(fk, /*gateRadiusNorm=*/0.05, /*chiSquareGate=*/9.21);

        CameraFrame frame;
        frame.camIndex = 0;
        CamPose cam = identityCamAt(0);
        frame.cam = &cam;
        RawDetection d0; d0.nx = 0.0; d0.ny = 0.0; d0.sigma = {1e-6,0,1e-6};
        frame.detections = {d0};

        const std::vector<std::vector<double>> emptyCov;   // 维度对不上(空)
        const auto result = assoc.associate(state, emptyCov, {frame});
        CHECK(result.measurements.size() == 1, "mismatched cov dimension: gracefully degrades to pure euclidean gating, does not crash");
    }

    // ---- 场景 5：检测自身的观测噪声也应该影响门控——状态协方差固定，
    // 检测噪声从很小调到很大，同样的偏移量应该从拒绝变成接受。----
    {
        HandPoseState state;
        state.wristPos = {0,0,500};
        state.jointAngles = {};
        auto fk = twoMarkerFk();
        MarkerAssociator assoc(fk, /*gateRadiusNorm=*/0.05, /*chiSquareGate=*/9.21);

        CameraFrame frame;
        frame.camIndex = 0;
        CamPose cam = identityCamAt(0);
        frame.cam = &cam;
        const double offsetNorm = 0.02;
        const auto cov = makeCov(6, 1e-8, 1e-10, 0.0);   // 状态协方差固定给到很小

        RawDetection dTight; dTight.nx = offsetNorm; dTight.ny = 0.0; dTight.sigma = {1e-8,0,1e-8};
        CameraFrame frameTight = frame; frameTight.detections = {dTight};
        const auto resultTight = assoc.associate(state, cov, {frameTight});
        CHECK(resultTight.measurements.empty(), "small detection noise: offset rejected when both state and detection are confident");

        RawDetection dLoose; dLoose.nx = offsetNorm; dLoose.ny = 0.0; dLoose.sigma = {50.0,0,50.0};
        CameraFrame frameLoose = frame; frameLoose.detections = {dLoose};
        const auto resultLoose = assoc.associate(state, cov, {frameLoose});
        CHECK(resultLoose.measurements.size() == 1,
              "large detection noise: same offset accepted once the detection's own uncertainty is factored in");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
