// IekfPointTracker.hpp 的单元测试——验证卡尔曼追踪后端在典型场景下的
// 基本正确性：ID稳定性、速度收敛、遮挡记忆、幽灵点门槛，跟
// test_temporal_tracker.cpp 覆盖的场景是同一套(方便横向对比两个后端)，
// 但因为IekfPointTracker吃的是"每相机2D观测"而不是"已经三角化好的3D点"，
// 这里额外需要构造简单的相机位姿+归一化观测。
#include "reconstruct/IekfPointTracker.hpp"
#include <cstdio>
#include <cmath>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

namespace {

// 两台简单相机：都在世界原点、无旋转，看向+Z——几何本身不是这个测试的
// 重点(IekfPointTracker不做三角化，只用相机位姿把3D点投影回2D算残差)，
// 用最简单的配置让每一步的期望值容易手算核对。
std::vector<EpiMat3> twoCamRs() { EpiMat3 I = {1,0,0, 0,1,0, 0,0,1}; return {I, I}; }
std::vector<EpiVec3> twoCamTs() { EpiVec3 z = {0,0,0}; return {z, z}; }

// 给定一个世界坐标点，算出两台相机(twoCamRs/twoCamTs同款配置)的归一化
// 观测——跟相机模型 nx=Xc/Zc, ny=Yc/Zc 保持一致(R=I,t=0时 Xc=Xw)。
std::vector<std::vector<std::array<double,2>>> obsForPoint(double x, double y, double z) {
    const double nx = x / z, ny = y / z;
    return { {{nx, ny}}, {{nx, ny}} };
}

Track3D mkTrackAtOrigin2Cam(double x, double y, double z) {
    Track3D t;
    t.point = {x, y, z};
    t.support = {{0, 0}, {1, 0}};   // 两台相机各自的第0号观测
    t.verified = true;
    return t;
}

} // namespace

int main() {
    const auto Rs = twoCamRs();
    const auto ts = twoCamTs();

    // ---- 场景1：匀速直线运动，ID全程稳定，速度估计收敛到真实值 ----
    {
        IekfPointTracker tr(50.0, 10, 1);
        int lastId = -1;
        bool idStable = true;
        double lastVx = 0.0;
        for (int frame = 0; frame < 12; ++frame) {
            const double x = frame * 5.0, depth = 1000.0;
            const auto obs = obsForPoint(x, 0, depth);
            auto out = tr.update({mkTrackAtOrigin2Cam(x, 0, depth)}, Rs, ts, obs);
            CHECK(!out.empty(), "constvel: track produced every frame");
            if (out.empty()) continue;
            if (frame == 0) lastId = out[0].id;
            else if (out[0].id != lastId) idStable = false;
            lastVx = out[0].velocity[0];
        }
        CHECK(idStable, "constvel: id stable across all frames");
        CHECK(std::abs(lastVx - 5.0) < 0.5, "constvel: velocity estimate converges near true value (5.0/frame)");
    }

    // ---- 场景2：短暂遮挡(几帧完全没有观测)后重新出现，ID应该保持 ----
    {
        IekfPointTracker tr(60.0, 10, 1);
        double x = 0, depth = 1000.0;
        int idBefore = -1;
        for (int frame = 0; frame < 5; ++frame) {
            x += 5.0;
            auto out = tr.update({mkTrackAtOrigin2Cam(x, 0, depth)}, Rs, ts, obsForPoint(x, 0, depth));
            idBefore = out.empty() ? idBefore : out[0].id;
        }
        // 3帧遮挡：喂空的frameTracks(没有任何候选点)。
        for (int i = 0; i < 3; ++i) {
            x += 5.0;   // 物理上球还在移动，只是没被看到
            tr.update({}, Rs, ts, {{}, {}});
        }
        auto back = tr.update({mkTrackAtOrigin2Cam(x, 0, depth)}, Rs, ts, obsForPoint(x, 0, depth));
        CHECK(back.size() == 1, "occlusion: exactly one track after reacquire");
        CHECK(!back.empty() && back[0].id == idBefore, "occlusion: same id recovered, not a new one");
        CHECK(!back.empty() && back[0].missedFrames == 0, "occlusion: missedFrames reset after reacquire");
    }

    // ---- 场景3：遮挡超过maxMissedFrames，ID被销毁，重新出现拿新ID ----
    {
        IekfPointTracker tr(60.0, 2, 1);
        auto r0 = tr.update({mkTrackAtOrigin2Cam(0, 0, 1000)}, Rs, ts, obsForPoint(0, 0, 1000));
        const int oldId = r0[0].id;
        tr.update({}, Rs, ts, {{}, {}});
        auto r2 = tr.update({}, Rs, ts, {{}, {}});
        CHECK(r2.size() == 1, "expiry: still alive at exactly maxMissedFrames misses");
        auto r3 = tr.update({}, Rs, ts, {{}, {}});
        CHECK(r3.empty(), "expiry: destroyed after exceeding maxMissedFrames");
        auto r4 = tr.update({mkTrackAtOrigin2Cam(0, 0, 1000)}, Rs, ts, obsForPoint(0, 0, 1000));
        CHECK(r4.size() == 1, "expiry: reappearing point produces exactly one track");
        CHECK(!r4.empty() && r4[0].id != oldId, "expiry: new id assigned, old id not resurrected");
    }

    // ---- 场景4：确认门槛(minHitsToConfirm)——幽灵点(单帧出现即消失)
    // 拿不到公开ID，真点连续命中够帧数才转正 ----
    {
        IekfPointTracker tr(30.0, 10, /*minHitsToConfirm=*/3);
        auto g0 = tr.update({mkTrackAtOrigin2Cam(500, 500, 1000)}, Rs, ts, obsForPoint(500, 500, 1000));
        CHECK(g0.empty(), "confirm: 1-frame ghost not published on first frame");
        auto g1 = tr.update({}, Rs, ts, {{}, {}});
        CHECK(g1.empty(), "confirm: 1-frame ghost died silently");

        auto t0 = tr.update({mkTrackAtOrigin2Cam(0, 0, 1000)}, Rs, ts, obsForPoint(0, 0, 1000));
        CHECK(t0.empty(), "confirm: real point hidden on hit 1");
        auto t1 = tr.update({mkTrackAtOrigin2Cam(0.5, 0, 1000)}, Rs, ts, obsForPoint(0.5, 0, 1000));
        CHECK(t1.empty(), "confirm: real point hidden on hit 2");
        auto t2 = tr.update({mkTrackAtOrigin2Cam(1.0, 0, 1000)}, Rs, ts, obsForPoint(1.0, 0, 1000));
        CHECK(t2.size() == 1, "confirm: real point published on hit 3");
        CHECK(!t2.empty() && t2[0].justAcquired, "confirm: first published frame marked justAcquired");
    }

    // ---- 场景5：两个点距离很远，各自应该拿到独立的id，不互相串号 ----
    {
        IekfPointTracker tr(20.0, 10, 1);
        auto r0 = tr.update({mkTrackAtOrigin2Cam(0, 0, 1000)}, Rs, ts, obsForPoint(0, 0, 1000));
        const int id0 = r0[0].id;
        std::vector<Track3D> two = {mkTrackAtOrigin2Cam(0.5, 0, 1000), mkTrackAtOrigin2Cam(800, 800, 1000)};
        // 两个候选各自的support都指向相机0号观测——现实中会是不同观测
        // 下标，这里为了测试简单只关心追踪器的关联/生命周期逻辑，不是
        // 真实的多观测场景，obs随意给一组跟两个点都对得上的即可。
        auto obsBoth = obsForPoint(0.5, 0, 1000);   // 只用于占位，IEKF内部会各自算残差
        auto r1 = tr.update(two, Rs, ts, obsBoth);
        CHECK(r1.size() == 2, "far apart points: two independent tracks");
        bool foundOld = false;
        for (auto& p : r1) if (p.id == id0) foundOld = true;
        CHECK(foundOld, "far apart points: original id preserved");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail > 0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
