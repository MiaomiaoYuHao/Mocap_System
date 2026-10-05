// 这个测试需要真的 calib/Calibration.hpp + reconstruct/Triangulation.hpp，
// 两者都拉 Qt(QString/QJsonObject)，所以这个测试目标要链 Qt6::Core 才能编译
// （跟 test_triangulation 需要 Qt6::Core 是同一个道理：不调用 Qt 运行时功能，
// 但编译期要能找到这两个头文件）。
//
// 这份测试的核心逻辑(像素->归一化协方差传播的雅可比、已知半径拟合触发
// 弧长门控/花生分离)已经用等价的 stub 头文件在零 Qt 环境下独立验证过，
// 数值跟这里的断言完全对得上；这里用真头文件重跑一遍确认接口对得上。
#include "detect/BlobObservationAdapter.hpp"
#include <cstdio>
#include <cmath>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

int main() {
    CameraIntrinsics intr;
    intr.fx = intr.fy = 800.0; intr.cx = 320.0; intr.cy = 240.0;
    intr.valid = true;
    BlobObservationConfig cfg; cfg.knownRadiusPx = 6.0;

    // ---- 场景 1：完整圆，圆心在主点——应该产出1条观测，nx/ny 接近0，
    // sigma 应该是个很小的正数(手算期望：0.18px/800 ≈ 2.25e-4，平方后
    // 约 5.06e-8，两个测试环境应该一致)。----
    {
        Blob b; b.area = 999;
        for (int i=0;i<60;++i) {
            const double a = 2*M_PI*i/60.0;
            b.contour.push_back({float(320+6*std::cos(a)), float(240+6*std::sin(a))});
        }
        auto dets = blobToObservations(b, intr, cfg);
        CHECK(dets.size()==1, "full circle at principal point: exactly one detection");
        if (!dets.empty()) {
            CHECK(std::abs(dets[0].nx) < 1e-4, "full circle: nx near zero");
            CHECK(std::abs(dets[0].ny) < 1e-4, "full circle: ny near zero");
            CHECK(dets[0].sigma.xx > 0 && dets[0].sigma.xx < 1e-6, "full circle: sigma magnitude in expected range (~5e-8)");
            char msg[128];
            std::snprintf(msg, sizeof(msg), "full circle: sigma.xx close to hand-computed 5.06e-8 (got %.3e)", dets[0].sigma.xx);
            CHECK(std::abs(dets[0].sigma.xx - 5.06e-8) < 2e-8, msg);
        }
    }

    // ---- 场景 2：45度短弧——应该被3b的门控弃用，0条观测。----
    {
        Blob b; b.area=100;
        for (int i=0;i<20;++i) {
            const double a = (200.0 + 45.0*i/19.0) * M_PI/180.0;
            b.contour.push_back({float(320+6*std::cos(a)), float(240+6*std::sin(a))});
        }
        auto dets = blobToObservations(b, intr, cfg);
        CHECK(dets.empty(), "45deg arc: discarded by gating, zero detections");
    }

    // ---- 场景 3：花生双球——应该产出2条独立观测，位置分别对应两个圆心。----
    {
        Blob b; b.area=200;
        const double d = 1.6*6.0;
        const double cAx=320-d/2, cAy=240, cBx=320+d/2, cBy=240;
        for (int which=0; which<2; ++which) {
            const double ccx = which==0?cAx:cBx, ccy = which==0?cAy:cBy;
            const double ox  = which==0?cBx:cAx, oy  = which==0?cBy:cAy;
            for (int i=0;i<80;++i) {
                const double a = 2*M_PI*i/80.0;
                const double px = ccx+6*std::cos(a), py = ccy+6*std::sin(a);
                const double dx=px-ox, dy=py-oy;
                if (std::sqrt(dx*dx+dy*dy) < 6.0) continue;
                b.contour.push_back({float(px), float(py)});
            }
        }
        auto dets = blobToObservations(b, intr, cfg);
        CHECK(dets.size()==2, "peanut shape: exactly two independent detections from one blob");
        if (dets.size()==2) {
            const double expectedNx = (d/2)/800.0;
            const bool orderA = std::abs(dets[0].nx - (-expectedNx)) < 1e-3;
            char msg[128];
            std::snprintf(msg, sizeof(msg), "peanut: detection positions match the two circle centers (nx=%.4f,%.4f expected +-%.4f)",
                         dets[0].nx, dets[1].nx, expectedNx);
            CHECK(orderA || std::abs(dets[0].nx-expectedNx)<1e-3, msg);
        }
    }

    // ---- 场景 4：退化输入(轮廓点太少)——不崩溃，0条观测。----
    {
        Blob b; b.contour = {{1,1},{2,2}};
        auto dets = blobToObservations(b, intr, cfg);
        CHECK(dets.empty(), "degenerate blob (too few contour points): no crash, zero detections");
    }

    // ---- 场景 5：没开 collectContours 的 Blob(contour 为空)——同样优雅
    // 返回空，不崩溃(调用方忘记在 ICamera 上打开轮廓收集时的兜底行为)。----
    {
        Blob b; b.area = 50;   // contour 留空，模拟没开 collectContours 的情况
        auto dets = blobToObservations(b, intr, cfg);
        CHECK(dets.empty(), "blob without contour data: gracefully returns empty, no crash");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
