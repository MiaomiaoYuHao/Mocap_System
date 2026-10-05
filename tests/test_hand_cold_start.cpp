#include "estimate/HandColdStart.hpp"
#include <cstdio>
#include <cmath>
#include <random>
#include <string>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

int main() {
    std::mt19937 rng(20260722);
    std::normal_distribution<double> noise(0.0, 0.5);

    // 用一套"随便贴的"模板(不是占位值)，验证匹配函数确实在用传入的模板，
    // 不是偷偷调用 handBackMarkers()。
    const std::array<HandVec3,5> customTemplate = {{
        {5, 40, 0}, {70, -10, 15}, {20, -35, -8}, {60, 25, 5}, {0, 0, 0}
    }};

    HandMat3 R = {1,0,0, 0,1,0, 0,0,1};
    HandVec3 t = {100, 50, 800};

    // ---- 场景 1：候选点里包含这套自定义模板的5个真实点(加噪声) + 若干
    // 干扰点，应该成功匹配、诊断信息里 failReason=None。----
    {
        std::vector<Vec3> candidates;
        for (const auto& p : customTemplate) {
            const auto wp = transformPoint(p, R, t);
            candidates.push_back({wp[0]+noise(rng), wp[1]+noise(rng), wp[2]+noise(rng)});
        }
        for (int i=0;i<8;++i) candidates.push_back({double(i)*37.0, double(i)*13.0, 750.0});   // 干扰点(手指球)

        const auto result = matchHandBackTemplate(candidates, customTemplate);
        CHECK(result.found(), "custom template matches when its 5 points are present among candidates");
        CHECK(result.diag.failReason == ColdStartFailReason::None, "diagnostics report success (None)");
        CHECK(result.diag.numCandidates3D == int(candidates.size()), "diagnostics report correct candidate count");
        CHECK(result.diag.numDistanceMatchesFound >= 1, "diagnostics report at least one distance-matched combination");
        CHECK(result.diag.bestRms >= 0 && result.diag.bestRms < 2.0, "diagnostics report a small best RMS");
    }

    // ---- 场景 2：候选点低于最低要求——诊断应明确指出"候选点不足"。----
    // 【注意】这个用例在退化匹配(5->4->3)上线后改过：以前 minMatchedPoints
    // 隐含是5，所以3个候选点会报 NotEnoughCandidates；现在 minMatchedPoints
    // 默认是3(Kabsch的数学下限)，3个候选点是**合法**输入，会往下走距离匹配、
    // 最后报 NoDistanceMatch。要触发"点不够"必须真的低于3点。
    {
        std::vector<Vec3> candidates = {{0,0,0},{1,1,1}};
        const auto result = matchHandBackTemplate(candidates, customTemplate);
        CHECK(!result.found(), "too few candidates: not found");
        CHECK(result.diag.failReason == ColdStartFailReason::NotEnoughCandidates, "diagnostics correctly identify NotEnoughCandidates");
    }

    // ---- 场景 2b：恰好3个候选点但形状完全对不上模板——这是退化匹配上线
    // 后的新语义，应该走到距离匹配并报 NoDistanceMatch，而不是"点不够"。----
    {
        std::vector<Vec3> candidates = {{0,0,0},{1,1,1},{2,2,2}};
        const auto result = matchHandBackTemplate(candidates, customTemplate);
        CHECK(!result.found(), "3 collinear mismatched candidates: not found");
        CHECK(result.diag.failReason == ColdStartFailReason::NoDistanceMatch, "3 candidates now reach distance matching, not NotEnoughCandidates");
    }

    // ---- 场景 3：候选点够多，但没有一组距离对得上模板(比如拿默认占位
    // handBackMarkers() 的形状去凑候选点，却传一个完全不同的自定义模板去
    // 匹配)——诊断应报 NoDistanceMatch。这也顺带验证了"确实在用传入的
    // 模板做匹配，不是偷偷用了别的模板"，否则这个场景会意外匹配成功。----
    {
        const auto& placeholderShape = handBackMarkers();   // 占位模板的形状，跟customTemplate形状不同
        std::vector<Vec3> candidates;
        for (const auto& p : placeholderShape) {
            const auto wp = transformPoint(p, R, t);
            candidates.push_back({wp[0], wp[1], wp[2]});
        }
        for (int i=0;i<5;++i) candidates.push_back({double(i)*50.0, 200.0, 900.0});

        const auto result = matchHandBackTemplate(candidates, customTemplate, ColdStartConfig{4.0, 5.0, 200});
        CHECK(!result.found(), "wrong template shape: correctly fails to match (proves it's actually using the passed-in template, not a hardcoded one)");
        CHECK(result.diag.failReason == ColdStartFailReason::NoDistanceMatch || result.diag.failReason == ColdStartFailReason::RmsTooHigh,
              "diagnostics report a sensible failure reason for mismatched template shape");
    }

    // ---- 场景 4：coldStartHandPose 便捷封装——成功时返回带诊断的state；
    // 失败时 state 为空但诊断仍然完整。----
    {
        std::vector<Vec3> candidates;
        for (const auto& p : customTemplate) {
            const auto wp = transformPoint(p, R, t);
            candidates.push_back({wp[0], wp[1], wp[2]});
        }
        const auto result = coldStartHandPose(candidates, customTemplate);
        CHECK(result.state.has_value(), "coldStartHandPose: succeeds with matching template");
        CHECK(result.diag.failReason == ColdStartFailReason::None, "coldStartHandPose: diagnostics report success");
        if (result.state) {
            CHECK(int(result.state->jointAngles.size()) == kHandNumJoints, "coldStartHandPose: joint angles sized correctly");
        }

        std::vector<Vec3> tooFew = {{0,0,0}};
        const auto failResult = coldStartHandPose(tooFew, customTemplate);
        CHECK(!failResult.state.has_value(), "coldStartHandPose: fails gracefully with too few candidates");
        CHECK(failResult.diag.failReason == ColdStartFailReason::NotEnoughCandidates, "coldStartHandPose: diagnostics carried through on failure");
    }

    // ---- 场景 5：summary() 对每种失败原因都返回非空的人话提示。----
    {
        ColdStartDiagnostics d;
        d.failReason = ColdStartFailReason::NotEnoughCandidates;
        CHECK(std::string(d.summary()).find("候选点") != std::string::npos, "summary text mentions candidate count for NotEnoughCandidates");
        d.failReason = ColdStartFailReason::NoDistanceMatch;
        CHECK(std::string(d.summary()).find("模板") != std::string::npos, "summary text mentions template for NoDistanceMatch");
        d.failReason = ColdStartFailReason::RmsTooHigh;
        CHECK(std::string(d.summary()).find("残差") != std::string::npos, "summary text mentions residual for RmsTooHigh");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
