// RadiusRefine.hpp 单元测试——合成数据验证"给定粗略3D点+相机位姿+2D质心
// 观测，反推期望像素半径"这一步数学。
//
// 覆盖范围：
//   1. 基本正确性：已知世界系点+相机位姿+物理半径，正向算出"真实"像素
//      半径，验证反推结果跟正向计算一致。
//   2. 距离跨度大时的核心价值：同一个物理半径，近/远两种深度下反推出的
//      像素半径应该明显不同(近的大、远的小)，且都应该分别接近该深度下
//      真正该有的值——这是本模块要解决的核心问题(固定标称距离在两头都错，
//      按各自深度反推就都对)。
//   3. 门控生效：候选点集里没有任何点投影到这台相机附近(比如都是别的
//      marker的点)，应该诚实返回unmatched，不能瞎认领离得最近但其实很远
//      的候选点。
//   4. 空候选点集：诚实返回unmatched，不崩溃。
//   5. 候选点在相机后方(无效)：应该被跳过，不影响其它候选点的匹配。
#include "estimate/RadiusRefine.hpp"
#include <cstdio>
#include <cmath>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

// 简单的"看向原点"相机位姿构造，跟项目里其它测试同一个套路。
static RfMat3 lookAtR(const RfVec3& camPos, const RfVec3& target, RfVec3& outT) {
    RfVec3 fwd = { target[0]-camPos[0], target[1]-camPos[1], target[2]-camPos[2] };
    double n = std::sqrt(fwd[0]*fwd[0]+fwd[1]*fwd[1]+fwd[2]*fwd[2]);
    fwd = {fwd[0]/n, fwd[1]/n, fwd[2]/n};
    RfVec3 up = {0,0,1};
    if (std::abs(fwd[0]*up[0]+fwd[1]*up[1]+fwd[2]*up[2]) > 0.99) up = {0,1,0};
    RfVec3 right = { fwd[1]*up[2]-fwd[2]*up[1], fwd[2]*up[0]-fwd[0]*up[2], fwd[0]*up[1]-fwd[1]*up[0] };
    double rn = std::sqrt(right[0]*right[0]+right[1]*right[1]+right[2]*right[2]);
    right = {right[0]/rn, right[1]/rn, right[2]/rn};
    RfVec3 camUp = { right[1]*fwd[2]-right[2]*fwd[1], right[2]*fwd[0]-right[0]*fwd[2], right[0]*fwd[1]-right[1]*fwd[0] };

    RfMat3 R = { right[0],right[1],right[2], -camUp[0],-camUp[1],-camUp[2], fwd[0],fwd[1],fwd[2] };
    for (int r=0;r<3;++r) { double s=0; for (int k=0;k<3;++k) s += R[size_t(r*3+k)]*camPos[size_t(k)]; outT[size_t(r)] = -s; }
    return R;
}

int main() {
    RfVec3 camT;
    const RfMat3 camR = lookAtR({0,0,0}, {0,0,500}, camT);
    const double fx = 990.0;
    const double physicalRadiusMm = 4.5;   // 9mm直径的球

    // ---- 场景 1：基本正确性——正向算出的"真实"半径，反推应该原样对上。----
    {
        const RfVec3 pointNear{20, -10, 300};   // 30cm深度
        double px, py, Zc;
        reprojectNormRf(camR, camT, pointNear, px, py, Zc);
        const double trueRadius = physicalRadiusMm * fx / Zc;

        const auto result = refineExpectedRadius(px, py, camR, camT, fx, {pointNear}, physicalRadiusMm);
        CHECK(result.matched, "basic correctness: match succeeds");
        char msg[160];
        std::snprintf(msg, sizeof(msg), "basic correctness: refined radius matches ground truth (expected=%.3fpx got=%.3fpx)", trueRadius, result.expectedRadiusPx);
        CHECK(std::abs(result.expectedRadiusPx - trueRadius) < 1e-6, msg);
    }

    // ---- 场景 2：核心价值——近/远两种深度，反推半径应该明显不同，且
    // 各自接近"用固定250mm假设算出来的半径"完全不该用的那个正确值。----
    {
        const RfVec3 pointClose{5, 5, 150};   // 15cm，测试范围近端
        const RfVec3 pointFar{5, 5, 500};     // 50cm，测试范围远端

        double px1,py1,Zc1, px2,py2,Zc2;
        reprojectNormRf(camR, camT, pointClose, px1, py1, Zc1);
        reprojectNormRf(camR, camT, pointFar, px2, py2, Zc2);

        const auto rClose = refineExpectedRadius(px1, py1, camR, camT, fx, {pointClose, pointFar}, physicalRadiusMm);
        const auto rFar   = refineExpectedRadius(px2, py2, camR, camT, fx, {pointClose, pointFar}, physicalRadiusMm);

        CHECK(rClose.matched && rFar.matched, "near/far scene: both match successfully");

        const double fixedAssumptionRadius = physicalRadiusMm * fx / 250.0;   // 旧的固定250mm假设
        char msg[220];
        std::snprintf(msg, sizeof(msg),
            "near/far scene: refined radii clearly differ (close=%.2fpx far=%.2fpx), correctly diverging from the single fixed-250mm assumption (%.2fpx)",
            rClose.expectedRadiusPx, rFar.expectedRadiusPx, fixedAssumptionRadius);
        // 近端应该明显大于固定假设，远端应该明显小于固定假设——正是"固定
        // 假设两头都错，按各自深度反推才都对"这个核心问题的直接验证。
        CHECK(rClose.expectedRadiusPx > fixedAssumptionRadius * 1.3 &&
              rFar.expectedRadiusPx < fixedAssumptionRadius * 0.7, msg);

        char msg2[160];
        std::snprintf(msg2, sizeof(msg2), "near/far scene: each refined radius accurately matches its own true depth (close err=%.4f far err=%.4f)",
                     std::abs(rClose.expectedRadiusPx - physicalRadiusMm*fx/Zc1),
                     std::abs(rFar.expectedRadiusPx - physicalRadiusMm*fx/Zc2));
        CHECK(std::abs(rClose.expectedRadiusPx - physicalRadiusMm*fx/Zc1) < 1e-6 &&
              std::abs(rFar.expectedRadiusPx - physicalRadiusMm*fx/Zc2) < 1e-6, msg2);
    }

    // ---- 场景 3：门控生效——候选点集里全是投影到别处的点(比如另一个
    // marker远在天边)，不该被误认领。----
    {
        const RfVec3 realPoint{20, -10, 300};
        const RfVec3 decoyFarAway{400, 400, 300};   // 投影方向差得很远

        double px, py, Zc;
        reprojectNormRf(camR, camT, realPoint, px, py, Zc);

        // 候选点集里只有远离的干扰点，没有真正对应这个观测的点。
        const auto result = refineExpectedRadius(px, py, camR, camT, fx, {decoyFarAway}, physicalRadiusMm);
        CHECK(!result.matched, "gating: does not falsely claim a distant unrelated candidate point");
    }

    // ---- 场景 4：空候选点集——诚实返回unmatched，不崩溃。----
    {
        const auto result = refineExpectedRadius(0.01, 0.01, camR, camT, fx, {}, physicalRadiusMm);
        CHECK(!result.matched, "empty candidate set: honestly reports unmatched, no crash");
    }

    // ---- 场景 5：候选点集混入相机后方的无效点，不该干扰正常点的匹配。----
    {
        const RfVec3 realPoint{20, -10, 300};
        const RfVec3 behindCamera{0, 0, -500};   // 在相机后方，reprojectNormRf应返回false被跳过

        double px, py, Zc;
        reprojectNormRf(camR, camT, realPoint, px, py, Zc);

        const auto result = refineExpectedRadius(px, py, camR, camT, fx, {behindCamera, realPoint}, physicalRadiusMm);
        CHECK(result.matched, "invalid behind-camera candidate mixed in: valid point still matches correctly");
        if (result.matched) {
            char msg[160];
            std::snprintf(msg, sizeof(msg), "invalid behind-camera candidate mixed in: matched depth is the valid point's depth (%.2f vs Zc=%.2f)", result.matchedDepthMm, Zc);
            CHECK(std::abs(result.matchedDepthMm - Zc) < 1e-6, msg);
        }
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
