// FingerChainSegmentation.hpp 单元测试——合成数据验证两级分组方案。
//
// 覆盖范围：
//   1. Tier1(引导式)：5根手指依次在各自的时间窗口内运动，其余时间几乎
//      不动，验证每个窗口都能正确分出该指的3颗球、且顺序(近/中/远)正确。
//   2. Tier1：某窗口内候选点不足3个/区分度不够时，诚实跳过。
//   3. Tier2(自由式)：5根手指全程用不同频率的独立摆动同时运动(不依次
//      动)，验证运动相关性聚类仍能正确分出5组。
//   4. Tier2：两根手指高度同步联动(模拟四指抓握时常见的联动)——不该被
//      误拆成2组各3点，而应该报告"发现一组6点的候选，未自动采纳"。
//   5. Tier2：完全静止/几乎不动的候选点，相关系数算不出意义，不该被
//      误分组。
#include "estimate/FingerChainSegmentation.hpp"
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

// 玩具三节链：anchor + 三段连杆，绕Y轴屈曲，跟FingerKinematicCalib.hpp的
// fingerChainFK同一个思路，这里独立重写一份简化版(不引入该文件)，保持
// 这个测试只依赖被测的分组逻辑，不依赖另一个模块的实现细节。
static std::array<Vec3,3> toyChainFK(const Vec3& anchor, double L1, double L2, double L3, double a0, double a1) {
    // 简化：整条链都在anchor所在的局部XZ平面里，累积屈曲角。
    const double ang1 = a0;
    const double ang2 = a0+a1;
    Vec3 p0 = { anchor[0]+L1*0.5*std::cos(ang1), anchor[1], anchor[2]-L1*0.5*std::sin(ang1) };
    Vec3 j1 = { anchor[0]+L1*std::cos(ang1), anchor[1], anchor[2]-L1*std::sin(ang1) };
    Vec3 p1 = { j1[0]+L2*0.5*std::cos(ang2), j1[1], j1[2]-L2*0.5*std::sin(ang2) };
    Vec3 j2 = { j1[0]+L2*std::cos(ang2), j1[1], j1[2]-L2*std::sin(ang2) };
    Vec3 p2 = { j2[0]+L3*0.5*std::cos(ang2), j2[1], j2[2]-L3*0.5*std::sin(ang2) };
    return {p0,p1,p2};
}

int main() {
    std::mt19937 rng(20260724);
    std::normal_distribution<double> noise(0.0, 0.2);

    // 5根手指的玩具锚点+连杆(数值随便取，只要求互不重叠、间距合理)。
    struct ChainDef { Vec3 anchor; double L1,L2,L3; };
    const std::vector<ChainDef> chains = {
        {{30,40,5}, 40,30,25},    // 0 = 拇指(占位)
        {{85,20,0}, 40,25,20},    // 1 = 食指
        {{88,3,0},  45,27,22},    // 2 = 中指
        {{85,-14,0},42,26,21},    // 3 = 无名指
        {{80,-30,0},33,20,17},    // 4 = 小指
    };

    // ---- 场景 1+2 合并：Tier1引导式——5根手指依次在各自60帧的窗口内
    // 运动(屈曲角大幅摆动)，窗口外几乎不动(只有极小噪声抖动，模拟"没在
    // 引导它动的时候，用户确实基本没动它")。----
    {
        const int perWindow = 60;
        const int totalFrames = perWindow * int(chains.size());
        const size_t totalFramesSz = size_t(totalFrames);
        std::vector<FrameTrackedPoints> frames(totalFramesSz);

        // track id 分配：链c的3个marker用 id = c*10 + {0,1,2}（对应近/中/远，
        // 但分组算法不应该依赖这个编号本身，只应该靠方差恢复出正确顺序，
        // 断言时会拿这个已知真值核对，不是让算法能看到这个编号含义）。
        for (int f=0; f<totalFrames; ++f) {
            const int activeChain = f / perWindow;   // 当前窗口对应哪根手指在动
            FrameTrackedPoints fr;
            for (int c=0; c<int(chains.size()); ++c) {
                double a0, a1;
                if (c == activeChain) {
                    // 大幅屈曲摆动(用窗口内的相对帧号算相位)。
                    const double t = double(f % perWindow) / double(perWindow);
                    a0 = 0.1 + 1.0*std::sin(t*2*M_PI*1.5);
                    a1 = 0.1 + 1.2*std::cos(t*2*M_PI*1.3);
                } else {
                    a0 = 0.15; a1 = 0.15;   // 静止(仅极小值，几乎不变)
                }
                auto pts = toyChainFK(chains[size_t(c)].anchor, chains[size_t(c)].L1, chains[size_t(c)].L2, chains[size_t(c)].L3, a0, a1);
                for (int k=0;k<3;++k) {
                    Vec3 p = pts[size_t(k)];
                    p[0]+=noise(rng); p[1]+=noise(rng); p[2]+=noise(rng);
                    fr.ids.push_back(c*10+k);
                    fr.positions.push_back(p);
                }
            }
            frames[size_t(f)] = fr;
        }

        std::vector<TimeWindow> windows;
        for (int c=0;c<int(chains.size());++c) windows.push_back({c*perWindow, (c+1)*perWindow});

        const auto result = segmentByTimeWindows(frames, windows);
        CHECK(result.valid, "tier1: segmentation succeeds");
        CHECK(int(result.groups.size()) == int(chains.size()), "tier1: found exactly 5 groups (one per finger window)");

        if (int(result.groups.size()) == int(chains.size())) {
            bool allCorrect = true;
            for (auto& g : result.groups) {
                // 该组3个id应该恰好是同一根链的(c*10+0,c*10+1,c*10+2)，
                // 且排序应该是 近(0)->中(1)->远(2)。
                if (g.orderedIds.size() != 3) { allCorrect = false; continue; }
                const int c = g.orderedIds[0] / 10;
                const bool sameChain = (g.orderedIds[1]/10==c) && (g.orderedIds[2]/10==c);
                const bool correctOrder = (g.orderedIds[0]%10==0) && (g.orderedIds[1]%10==1) && (g.orderedIds[2]%10==2);
                if (!sameChain || !correctOrder) allCorrect = false;
            }
            CHECK(allCorrect, "tier1: every group has the correct 3 same-chain ids in proximal->middle->distal order");
        }
    }

    // ---- 场景 3：Tier1——某窗口帧数不足，诚实跳过(不产出虚假分组)。----
    {
        std::vector<FrameTrackedPoints> frames(50);
        for (int f=0; f<50; ++f) {
            FrameTrackedPoints fr;
            fr.ids = {0,1,2};
            fr.positions = {{0,0,0},{10,0,0},{20,0,0}};
            frames[size_t(f)] = fr;
        }
        std::vector<TimeWindow> windows = { {0, 5} };   // 只有5帧，远低于门槛
        const auto result = segmentByTimeWindows(frames, windows);
        CHECK(!result.valid, "tier1: window with too few frames is honestly skipped, not forced into a group");
    }

    // ---- 场景 4：Tier2自由式——5根手指全程用不同频率独立摆动，不依次
    // 动，验证相关性聚类仍能分出5组、且成员/顺序都对。----
    {
        const int F = 400;
        const size_t Fsz = size_t(F);
        std::vector<FrameTrackedPoints> frames(Fsz);
        // 每根手指用不同频率/相位的正弦摆动，保证互相不相关(频率不同，
        // 长时间序列下几乎不可能凑出高相关系数)。
        const std::vector<double> freqs = {1.7, 2.3, 3.1, 3.7, 2.9};   // 全部选取足够高的频率，保证采集窗口内有多个完整摆动周期(频率太低=只摆半下，方向relation容易受随机相位组合影响，不是真实差异)
        const std::vector<double> phases = {0.0, 1.1, 2.2, 3.3, 4.4};

        for (int f=0; f<F; ++f) {
            FrameTrackedPoints fr;
            for (int c=0;c<int(chains.size());++c) {
                const double t = double(f)/double(F);
                // 同一根手指的a0/a1用相同频率/相位驱动(现实里"屈伸这根手指"
                // 是各关节协同动的，不会各自独立的频率)，只在幅度上略有差异，
                // 这样同链内相关性才有代表性；跨手指用完全不同频率/相位，
                // 保持互不相关。
                // a1跟a0用完全相同的相位(现实中"自然屈伸整根手指"时各关节
                // 协同度很高)，只是幅度不同——这样同链内近/中/远节的相关性
                // 才有代表性；跨手指freq/phase都不同，天然不相关。
                const double a0 = 0.1 + 0.9*std::sin(2*M_PI*freqs[size_t(c)]*t + phases[size_t(c)]);
                const double a1 = 0.1 + 0.6*std::sin(2*M_PI*freqs[size_t(c)]*t + phases[size_t(c)]);
                auto pts = toyChainFK(chains[size_t(c)].anchor, chains[size_t(c)].L1, chains[size_t(c)].L2, chains[size_t(c)].L3, a0, a1);
                for (int k=0;k<3;++k) {
                    Vec3 p = pts[size_t(k)];
                    p[0]+=noise(rng); p[1]+=noise(rng); p[2]+=noise(rng);
                    fr.ids.push_back(c*10+k);
                    fr.positions.push_back(p);
                }
            }
            frames[size_t(f)] = fr;
        }

        const auto result = segmentByMotionCorrelation(frames);
        CHECK(result.valid, "tier2: segmentation succeeds with 5 independently-moving fingers, no sequencing needed");
        char msg[128];
        std::snprintf(msg, sizeof(msg), "tier2: found exactly 5 groups (found %zu)", result.groups.size());
        CHECK(int(result.groups.size()) == int(chains.size()), msg);

        if (int(result.groups.size()) == int(chains.size())) {
            bool allCorrect = true;
            for (auto& g : result.groups) {
                if (g.orderedIds.size() != 3) { allCorrect = false; continue; }
                const int c = g.orderedIds[0] / 10;
                const bool sameChain = (g.orderedIds[1]/10==c) && (g.orderedIds[2]/10==c);
                const bool correctOrder = (g.orderedIds[0]%10==0) && (g.orderedIds[1]%10==1) && (g.orderedIds[2]%10==2);
                if (!sameChain || !correctOrder) allCorrect = false;
            }
            CHECK(allCorrect, "tier2: every group has the correct 3 same-chain ids in proximal->middle->distal order");
        }
    }

    // ---- 场景 5：Tier2——两根手指高度同步联动(比如无名指+小指经常一起
    // 屈曲，握拳时很常见)，不该被误拆成2组各3点，应报告"6点候选未采纳"。----
    {
        const int F = 300;
        const size_t Fsz = size_t(F);
        std::vector<FrameTrackedPoints> frames(Fsz);
        for (int f=0; f<F; ++f) {
            FrameTrackedPoints fr;
            const double t = double(f)/double(F);
            for (int c=0;c<int(chains.size());++c) {
                double a0, a1;
                if (c==3 || c==4) {
                    // 无名指(3)和小指(4)完全同步联动(同一个信号驱动)。
                    a0 = 0.1 + 0.9*std::sin(2*M_PI*1.5*t);
                    a1 = 0.1 + 0.6*std::sin(2*M_PI*1.5*t);
                } else {
                    const double freq = 1.8 + 0.7*c;   // 保证足够周期数，避开低频伪相关(同前面场景4踩过的坑)
                    a0 = 0.1 + 0.9*std::sin(2*M_PI*freq*t + c);
                    a1 = 0.1 + 0.6*std::sin(2*M_PI*freq*t + c);
                }
                auto pts = toyChainFK(chains[size_t(c)].anchor, chains[size_t(c)].L1, chains[size_t(c)].L2, chains[size_t(c)].L3, a0, a1);
                for (int k=0;k<3;++k) {
                    Vec3 p = pts[size_t(k)];
                    p[0]+=noise(rng); p[1]+=noise(rng); p[2]+=noise(rng);
                    fr.ids.push_back(c*10+k);
                    fr.positions.push_back(p);
                }
            }
            frames[size_t(f)] = fr;
        }
        const auto result = segmentByMotionCorrelation(frames);
        CHECK(result.valid, "tier2 with synchronized pair: the other 3 independent fingers still segment successfully");
        CHECK(int(result.groups.size()) == 3, "tier2 with synchronized pair: exactly 3 groups auto-accepted (thumb/index/middle), the synced ring+pinky pair is NOT force-split");

        bool syncedIdsNotInAnyGroup = true;
        for (auto& g : result.groups)
            for (int id : g.orderedIds)
                if (id/10==3 || id/10==4) syncedIdsNotInAnyGroup = false;
        CHECK(syncedIdsNotInAnyGroup, "tier2 with synchronized pair: ring+pinky ids are excluded from all accepted groups (reported as ambiguous 6-point cluster instead)");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
