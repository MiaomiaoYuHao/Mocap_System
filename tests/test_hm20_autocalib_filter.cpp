// 自标定 + 滤波 + IK 门控的单测。不依赖 onnxruntime：用一个可控的假后端。
#include "estimate/HandSkeletonAssociator.hpp"
#include "estimate/Hm20AutoCalib.hpp"
#include "estimate/Hm20PoseFilter.hpp"
#include <cstdio>
#include <cmath>

using namespace mocap::hm20;
static int g_fail = 0;
#define CHECK(c) do{ if(!(c)){ std::printf("FAIL %s:%d  %s\n",__FILE__,__LINE__,#c); ++g_fail; } }while(0)

static SkeletonFrameResult makeFrame(double ang, bool allDorsum = true) {
    SkeletonFrameResult r{};
    r.valid = true; r.pentagonOk = true; r.wristPoseValid = true;
    r.dorsumRmseMm = 0.4;
    const double c = std::cos(ang), s = std::sin(ang);
    r.wristR = Mat3{c,-s,0, s,c,0, 0,0,1};
    r.wristQuat = matToQuat(r.wristR);
    // 手背 5 点（非对称五边形），手指 15 点在 +X 侧
    const double back[5][3] = {{-20,25,8},{5,32,8},{28,4,8},{12,-26,8},{-18,-20,8}};
    for (int i = 0; i < 5; ++i) {
        Vec3 p{back[i][0], back[i][1], back[i][2]};
        r.markers[size_t(i)].posWorld = detail::matVec(r.wristR, p);
        r.markers[size_t(i)].observed = allDorsum || i < 4;
        r.markers[size_t(i)].confidence = 0.6;
    }
    for (int f = 0; f < 5; ++f)
        for (int j = 0; j < 3; ++j) {
            const int m = 5 + 3*f + j;
            Vec3 p{40.0 + 22.0*j + 4.0*std::sin(ang*3 + f), 26.0 - 13.0*f, 6.0 - 3.0*j};
            r.markers[size_t(m)].posWorld = detail::matVec(r.wristR, p);
            r.markers[size_t(m)].observed = true;
            r.markers[size_t(m)].confidence = 0.9;
        }
    return r;
}

int main() {
    // ---- 1. 不是一喂就标：必须先过 Warmup ----
    {
        Hm20AutoCalib ac;
        CHECK(ac.stage() == AutoCalibStage::Idle);
        auto r = makeFrame(0.0);
        ac.feed(r);
        CHECK(ac.stage() == AutoCalibStage::Warmup);
        CHECK(!ac.backReady());
    }
    // ---- 2. 手背点不全 -> 拒绝，并报得出原因 ----
    {
        Hm20AutoCalib ac;
        auto r = makeFrame(0.0, /*allDorsum=*/false);
        ac.feed(r);
        CHECK(ac.lastReject() == AutoCalibReject::NotEnoughPoints);
        CHECK(ac.stage() == AutoCalibStage::Idle);
    }
    // ---- 3. 动太快 -> 拒绝 ----
    {
        Hm20AutoCalib ac;
        ac.feed(makeFrame(0.0));
        auto r = makeFrame(0.0);
        for (int m = 0; m < 20; ++m) r.markers[size_t(m)].posWorld[0] += 200.0;
        ac.feed(r);
        CHECK(ac.lastReject() == AutoCalibReject::MovingTooFast);
    }
    // ---- 4. 缓慢转手腕 -> 走完 Warmup/Collecting，进入 Validating ----
    {
        Hm20AutoCalib ac;
        for (int k = 0; k < 200; ++k) ac.feed(makeFrame(k * 0.004));
        CHECK(ac.stage() == AutoCalibStage::Validating || ac.stage() == AutoCalibStage::Locked);
        CHECK(ac.backReady());
        // IK 在参数收敛前【必须】不放行
        CHECK(!ac.ikUsable());
    }
    // ---- 5. reset 之后回到干净状态 ----
    {
        Hm20AutoCalib ac;
        for (int k = 0; k < 200; ++k) ac.feed(makeFrame(k * 0.004));
        ac.reset();
        CHECK(ac.stage() == AutoCalibStage::Idle);
        CHECK(!ac.backReady());
        CHECK(!ac.ikUsable());
    }
    // ---- 6. 滤波：静止时输出真正不动（死区生效）----
    {
        Hm20PoseFilter f;
        auto r = makeFrame(0.3);
        for (int s = 0; s < kNumSegments; ++s) r.segSource[size_t(s)] = SegSource::Geometry;
        f.apply(r, 1.0/60.0);
        const Vec3 p0 = r.markers[7].posWorld;
        for (int k = 0; k < 40; ++k) { auto q = makeFrame(0.3); f.apply(q, 1.0/60.0); r = q; }
        CHECK(detail::norm(detail::sub(r.markers[7].posWorld, p0)) < 0.05);
    }
    // ---- 7. 滤波：marker 重捕时硬复位，不拖尾 ----
    {
        Hm20PoseFilter f;
        for (int k = 0; k < 30; ++k) { auto q = makeFrame(0.0); f.apply(q, 1.0/60.0); }
        auto r = makeFrame(0.0);
        r.markers[7].observed = false;
        f.apply(r, 1.0/60.0);
        auto r2 = makeFrame(0.0);
        r2.markers[7].posWorld[0] += 120.0;      // 重捕在很远的地方
        r2.markers[7].observed = true;
        f.apply(r2, 1.0/60.0);
        CHECK(std::fabs(r2.markers[7].posWorld[0] - (makeFrame(0.0).markers[7].posWorld[0] + 120.0)) < 1e-6);
    }
    // ---- 8. 滤波：四元数半球对齐，不产生 180° 假运动 ----
    {
        Hm20PoseFilter f;
        auto r = makeFrame(0.0);
        for (int s = 0; s < kNumSegments; ++s) r.segSource[size_t(s)] = SegSource::Geometry;
        f.apply(r, 1.0/60.0);
        auto r2 = makeFrame(0.0);
        for (int s = 0; s < kNumSegments; ++s) r2.segSource[size_t(s)] = SegSource::Geometry;
        for (auto& c : r2.segQuat[3]) c = -c;    // 同一个旋转，符号翻转
        f.apply(r2, 1.0/60.0);
        double d = 0;
        for (int i = 0; i < 4; ++i) d += r2.segQuat[3][size_t(i)] * r.segQuat[3][size_t(i)];
        CHECK(std::fabs(std::fabs(d) - 1.0) < 1e-3);   // 输出仍是同一个旋转
    }
    // ---- 9. 滤波关掉时必须逐位不动 ----
    {
        PoseFilterConfig c; c.enabled = false;
        Hm20PoseFilter f(c);
        auto a = makeFrame(0.7); auto b = a;
        f.apply(b, 1.0/60.0);
        for (int m = 0; m < 20; ++m)
            CHECK(detail::norm(detail::sub(a.markers[size_t(m)].posWorld,
                                           b.markers[size_t(m)].posWorld)) == 0.0);
    }
    // ---- 10. 预设表 ----
    {
        CHECK(!makePoseFilterPreset(PoseFilterPreset::Off).enabled);
        CHECK(makePoseFilterPreset(PoseFilterPreset::Strong).posMinCutoffHz <
              makePoseFilterPreset(PoseFilterPreset::Light).posMinCutoffHz);
    }

    if (g_fail == 0) std::printf("test_hm20_autocalib_filter: ALL PASS\n");
    else             std::printf("test_hm20_autocalib_filter: %d FAILED\n", g_fail);
    return g_fail ? 1 : 0;
}
