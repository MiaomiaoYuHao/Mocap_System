// HandColdStart.hpp 单元测试(退化搜索版)——验证"手背marker少一颗也能定位"
// 这个核心修复。
//
// 覆盖范围：
//   1. 全部5点可见：应该正常成功，numMatchedPoints==5(回归测试，确认
//      退化逻辑没有破坏原有的满点场景)。
//   2. 恰好4点可见(1颗被遮挡)：应该仍然成功，numMatchedPoints==4——这是
//      本轮修复的核心场景。
//   3. 恰好3点可见(2颗被遮挡，退化下限)：应该仍然成功，numMatchedPoints==3。
//   4. 只有2点可见：低于下限，诚实报告候选点不足，不强行凑数。
//   5. 优先级验证：5点和4点都能凑出合格匹配时，应该优先用5点的结果
//      (约束更多，不会退化到4点去)。
//   6. 决斗点干扰：混入若干跟手背模板无关的候选点，退化搜索不应该被
//      这些干扰点污染，依然正确识别出真正的手背子集。
#include "estimate/HandColdStart.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static HandMat3 randRot(std::mt19937& g) {
    std::normal_distribution<double> nd(0,1);
    std::uniform_real_distribution<double> ud(0,2*M_PI);
    double v[3]={nd(g),nd(g),nd(g)};
    double n=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    v[0]/=n;v[1]/=n;v[2]/=n;
    double a=ud(g), c=std::cos(a), s=std::sin(a), C=1-c;
    return {
        c+v[0]*v[0]*C,      v[0]*v[1]*C-v[2]*s, v[0]*v[2]*C+v[1]*s,
        v[1]*v[0]*C+v[2]*s, c+v[1]*v[1]*C,      v[1]*v[2]*C-v[0]*s,
        v[2]*v[0]*C-v[1]*s, v[2]*v[1]*C+v[0]*s, c+v[2]*v[2]*C
    };
}
static HandVec3 applyRT(const HandMat3& R, const HandVec3& t, const HandVec3& p) {
    return { R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0],
             R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1],
             R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2] };
}

int main() {
    std::mt19937 rng(20260810);
    std::normal_distribution<double> noise(0.0, 0.3);

    const std::array<HandVec3,5> templ = { HandVec3{15,0,2}, {55,22,5}, {50,-25,8}, {35,30,6}, {60,3,3} };

    // ---- 场景 1：全部5点可见，回归测试。----
    {
        const HandMat3 R = randRot(rng);
        const HandVec3 t = {80,-20,500};
        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            auto w = applyRT(R,t,templ[size_t(i)]);
            candidates.push_back({w[0]+noise(rng), w[1]+noise(rng), w[2]+noise(rng)});
        }
        const auto result = matchHandBackTemplate(candidates, templ);
        CHECK(result.found(), "5-point scene: match succeeds");
        CHECK(result.diag.numMatchedPoints == 5, "5-point scene: uses all 5 points, no unnecessary degradation");
    }

    // ---- 场景 2(核心修复)：恰好4点可见(第2颗被遮挡)，应该仍能成功。----
    {
        const HandMat3 R = randRot(rng);
        const HandVec3 t = {30,40,480};
        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            if (i == 1) continue;   // 模拟第2颗手背marker被遮挡，这一帧没有三角化出来
            auto w = applyRT(R,t,templ[size_t(i)]);
            candidates.push_back({w[0]+noise(rng), w[1]+noise(rng), w[2]+noise(rng)});
        }
        const auto result = matchHandBackTemplate(candidates, templ);
        CHECK(result.found(), "4-point scene (1 occluded): match still succeeds, not reported as totally lost");
        CHECK(result.diag.numMatchedPoints == 4, "4-point scene: diagnostics correctly report 4 matched points, not 5");
    }

    // ---- 场景 3：恰好3点可见(退化下限)，应该仍能成功。----
    {
        const HandMat3 R = randRot(rng);
        const HandVec3 t = {-10,15,520};
        std::vector<Vec3> candidates;
        for (int i : {0,2,4}) {   // 只留3颗(1、3号被遮挡)
            auto w = applyRT(R,t,templ[size_t(i)]);
            candidates.push_back({w[0]+noise(rng), w[1]+noise(rng), w[2]+noise(rng)});
        }
        const auto result = matchHandBackTemplate(candidates, templ);
        CHECK(result.found(), "3-point scene (degradation floor): match still succeeds");
        CHECK(result.diag.numMatchedPoints == 3, "3-point scene: diagnostics correctly report 3 matched points");
    }

    // ---- 场景 4：只有2点可见，低于下限，诚实报告候选点不足。----
    {
        const HandMat3 R = randRot(rng);
        const HandVec3 t = {0,0,500};
        std::vector<Vec3> candidates;
        for (int i : {0,2}) {
            auto w = applyRT(R,t,templ[size_t(i)]);
            candidates.push_back({w[0]+noise(rng), w[1]+noise(rng), w[2]+noise(rng)});
        }
        const auto result = matchHandBackTemplate(candidates, templ);
        CHECK(!result.found(), "2-point scene (below floor): honestly fails, does not fabricate a pose from insufficient points");
        CHECK(result.diag.failReason == ColdStartFailReason::NotEnoughCandidates,
              "2-point scene: correctly diagnosed as insufficient candidates, not some other failure reason");
    }

    // ---- 场景 5：优先级——5点和4点都能凑出合格匹配时，应该用5点的结果，
    // 不退化到4点。用全部5点可见的场景验证diag里报的是5，不是随便一个
    // 4点子集的结果。----
    {
        const HandMat3 R = randRot(rng);
        const HandVec3 t = {40,-10,510};
        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            auto w = applyRT(R,t,templ[size_t(i)]);
            candidates.push_back({w[0]+noise(rng), w[1]+noise(rng), w[2]+noise(rng)});
        }
        const auto result = matchHandBackTemplate(candidates, templ);
        CHECK(result.found() && result.diag.numMatchedPoints == 5,
              "priority test: with all 5 points available and matchable, prefers the full 5-point solution over any 4-point subset");
    }

    // ---- 场景 6：干扰点混入，退化搜索不应该被污染——4点可见+3个跟手背
    // 模板毫不相关的干扰点，应该还是正确用那4个真实点成功匹配，不会被
    // 干扰点带偏。----
    {
        const HandMat3 R = randRot(rng);
        const HandVec3 t = {15,25,495};
        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            if (i == 3) continue;   // 第4颗被遮挡
            auto w = applyRT(R,t,templ[size_t(i)]);
            candidates.push_back({w[0]+noise(rng), w[1]+noise(rng), w[2]+noise(rng)});
        }
        // 3个干扰点，随便撒在别处，两两距离跟手背模板的任何子集都对不上。
        candidates.push_back({300,300,500});
        candidates.push_back({-300,200,480});
        candidates.push_back({100,-300,510});

        const auto result = matchHandBackTemplate(candidates, templ);
        CHECK(result.found(), "decoys mixed in with 4 real points: still finds the correct match");
        CHECK(result.diag.numMatchedPoints == 4, "decoys mixed in: correctly uses the 4 real points, not fooled into a spurious combination");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
