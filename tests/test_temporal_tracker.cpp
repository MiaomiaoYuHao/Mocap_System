#include "reconstruct/TemporalTracker.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static Track3D mkTrack(double x, double y, double z) {
    Track3D t; t.point = {x,y,z}; t.verified = true; t.residual = 0.01; return t;
}

int main() {
    // ---- 场景 1：一个点匀速直线运动，中途被遮挡 3 帧后重新出现，
    // ID 必须保持不变（不能被当成新点）。----
    {
        TemporalTracker tr(/*maxAssocDist=*/15.0, /*maxMissedFrames=*/5);

        // 帧 0：起点
        auto r0 = tr.update({ mkTrack(0,0,0) });
        CHECK(r0.size()==1, "frame0: exactly one track");
        const int id0 = r0[0].id;

        // 帧 1..3：匀速沿 x 前进，每帧 +2.0（建立稳定的速度估计）
        tr.update({ mkTrack(2,0,0) });
        tr.update({ mkTrack(4,0,0) });
        auto r3 = tr.update({ mkTrack(6,0,0) });
        CHECK(r3.size()==1 && r3[0].id==id0, "frame3: same id maintained during continuous tracking");

        // 帧 4..6：遮挡，cluster 这三帧完全没产出（比如被手挡住）
        tr.update({});
        tr.update({});
        auto rOcc = tr.update({});
        CHECK(rOcc.size()==1, "occlusion: track kept alive within maxMissedFrames window");
        CHECK(rOcc[0].missedFrames==3, "occlusion: missedFrames counted correctly");

        // 帧 7：重新出现，按匀速预测应该在 x=12（6 + 2*3 步遮挡）附近
        auto rBack = tr.update({ mkTrack(12,0,0) });
        CHECK(rBack.size()==1, "reacquire: exactly one track after reappearing");
        CHECK(rBack[0].id==id0, "reacquire: same id recovered after occlusion, not a new one");
        CHECK(rBack[0].missedFrames==0, "reacquire: missedFrames reset to 0 after match");
    }

    // ---- 场景 2：遮挡时间超过 maxMissedFrames，ID 必须被销毁；
    // 之后重新出现的点应该拿到全新 ID，而不是"复活"旧 ID。----
    {
        TemporalTracker tr(/*maxAssocDist=*/15.0, /*maxMissedFrames=*/2);
        auto r0 = tr.update({ mkTrack(0,0,0) });
        const int oldId = r0[0].id;

        tr.update({});                 // missed 1
        auto r2 = tr.update({});        // missed 2 (still within window, <=2)
        CHECK(r2.size()==1, "still-alive window: track survives exactly maxMissedFrames misses");

        auto r3 = tr.update({});        // missed 3 (> window) -> destroyed
        CHECK(r3.empty(), "expired window: track destroyed after exceeding maxMissedFrames");

        auto r4 = tr.update({ mkTrack(0,0,0) });
        CHECK(r4.size()==1, "revival: reappearing point produces exactly one track");
        CHECK(r4[0].id != oldId, "revival: new id assigned, old id not resurrected");
    }

    // ---- 场景 3：同一帧出现一个全新的点（不关联任何已有轨迹），
    // 应立刻拿到新 ID 且 justAcquired 为 true；同时已有轨迹不受影响。----
    {
        TemporalTracker tr(/*maxAssocDist=*/5.0, /*maxMissedFrames=*/5);
        auto r0 = tr.update({ mkTrack(0,0,0) });
        const int id0 = r0[0].id;

        // 第二个点距离第一个很远，超出关联半径 -> 必须是新 ID，不能被错配。
        auto r1 = tr.update({ mkTrack(0,0,0.5), mkTrack(500,500,500) });
        CHECK(r1.size()==2, "new point: total active tracks becomes 2");
        bool foundOld=false, foundNewJustAcquired=false;
        for (auto& tp : r1) {
            if (tp.id==id0) { foundOld=true; CHECK(!tp.justAcquired, "existing track not marked justAcquired"); }
            else { CHECK(tp.justAcquired, "brand-new far point marked justAcquired"); foundNewJustAcquired=true; }
        }
        CHECK(foundOld, "original track id preserved when a second point appears");
        CHECK(foundNewJustAcquired, "second point got its own new id");
    }

    // ---- 场景 4：快速移动 + 遮挡叠加——建立起较大的匀速估计后，即使中途
    // 丢了几帧，捡回来时也该用"丢失帧数 x 速度"算出正确的预测位置，而不是
    // 拿丢失前的原始位置去比对（那样距离会远超关联半径，误判成新点）。----
    {
        TemporalTracker tr(/*maxAssocDist=*/25.0, /*maxMissedFrames=*/4);
        tr.update({ mkTrack(0,0,0) });
        auto r1 = tr.update({ mkTrack(20,0,0) });   // 建立速度 ~20/帧（首帧关联半径须覆盖冷启动跳变）
        const int id0 = r1[0].id;

        tr.update({});   // 遮挡，missed=1
        auto rOcc = tr.update({});   // missed=2
        CHECK(rOcc.size()==1 && rOcc[0].id==id0, "fast+occluded: track kept alive through occlusion");

        // 捡回来时位置应在 20 + 20*2(丢了2帧) = 60 附近；若只拿丢失前的
        // 原始位置(20)去比对，距离会是 40，超过半径 25，会被误判成新点。
        auto rBack = tr.update({ mkTrack(60,0,0) });
        CHECK(rBack.size()==1, "fast+occluded reacquire: still exactly one track");
        CHECK(rBack[0].id==id0, "fast+occluded reacquire: prediction (not stale raw position) recovers correct id");
    }

    // ---- 场景：转正确认机制(minHitsToConfirm=3)——幽灵点拿不到公开ID ----
    {
        TemporalTracker tr(25.0, 10, /*minHitsToConfirm=*/3);

        // 单帧幽灵：出现一帧就消失，永远不应该对外可见
        auto g0 = tr.update({ mkTrack(100,100,100) });
        CHECK(g0.empty(), "confirm: 1-frame ghost not published on its first frame");
        auto g1 = tr.update({});
        CHECK(g1.empty(), "confirm: 1-frame ghost died silently, never published");

        // 真点：连续出现，第3帧起才对外可见，且转正瞬间不换编号
        auto t0 = tr.update({ mkTrack(0,0,0) });
        CHECK(t0.empty(), "confirm: real point hidden on hit 1");
        auto t1 = tr.update({ mkTrack(0.5,0,0) });
        CHECK(t1.empty(), "confirm: real point hidden on hit 2");
        auto t2 = tr.update({ mkTrack(1.0,0,0) });
        CHECK(t2.size()==1, "confirm: real point published on hit 3");
        CHECK(t2[0].justAcquired, "confirm: first published frame marked justAcquired");
        const int realId = t2[0].id;
        auto t3 = tr.update({ mkTrack(1.5,0,0) });
        CHECK(t3.size()==1 && t3[0].id==realId, "confirm: id stable after confirmation");
        CHECK(!t3[0].justAcquired, "confirm: justAcquired only on the first published frame");

        // 转正之后才享受遮挡记忆窗口
        tr.update({}); tr.update({});
        auto t5 = tr.update({ mkTrack(2.5,0,0) });
        CHECK(t5.size()==1 && t5[0].id==realId, "confirm: confirmed track survives occlusion with same id");

        // tentative期间丢一帧立刻死：出现2帧(不够3)后消失，重新出现拿新ID且要重新攒
        TemporalTracker tr2(25.0, 10, 3);
        tr2.update({ mkTrack(0,0,0) });
        tr2.update({ mkTrack(0,0,0) });
        auto d0 = tr2.update({});
        CHECK(d0.empty(), "confirm: tentative dies immediately on first miss");
        tr2.update({ mkTrack(0,0,0) });
        tr2.update({ mkTrack(0,0,0) });
        auto d1 = tr2.update({ mkTrack(0,0,0) });
        CHECK(d1.size()==1, "confirm: reappeared point re-earns confirmation from scratch");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
