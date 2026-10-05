// 三角化离线验证：不依赖 Qt/相机/UI，纯数学正确性测试。
// 用已知的合成相机参数 + 已知世界坐标点，正向投影出"造出来的"像素观测，
// 再喂给 triangulateTwoViews，检验能否原样还原出世界坐标——这是排除
// "三角化数学翻译错了"这一类问题最直接的办法（跟 test_centroid.cpp
// 拿合成双高斯亮斑测检测算法是同一个思路：先在没有相机、没有标定误差
// 干扰的干净环境里把算法本身钉死，再上真实硬件）。
#include "reconstruct/Triangulation.hpp"
#include <cmath>
#include <cstdio>
#include <cassert>

using namespace mocap;

namespace {

struct SynthCam { CameraIntrinsics intr; CameraExtrinsics extr; };

// 正向投影（造测试数据用）：世界点 -> 该相机的像素坐标（含畸变）。
// 故意用另一套独立公式实现，不能直接复用 Triangulation.cpp 里的代码——
// 否则测试等于自己证明自己，没有意义。
void projectPoint(const SynthCam& cam, const std::array<double, 3>& Xw,
                   double& px, double& py) {
    const auto& R = cam.extr.R;
    const auto& t = cam.extr.t;
    const double xc = R[0] * Xw[0] + R[1] * Xw[1] + R[2] * Xw[2] + t[0];
    const double yc = R[3] * Xw[0] + R[4] * Xw[1] + R[5] * Xw[2] + t[1];
    const double zc = R[6] * Xw[0] + R[7] * Xw[1] + R[8] * Xw[2] + t[2];
    assert(zc > 0 && "测试点必须在相机前方，检查合成参数");
    const double x = xc / zc, y = yc / zc;

    const auto& I = cam.intr;
    const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    const double xd = x * (1 + I.k1 * r2 + I.k2 * r4 + I.k3 * r6) + 2 * I.p1 * x * y + I.p2 * (r2 + 2 * x * x);
    const double yd = y * (1 + I.k1 * r2 + I.k2 * r4 + I.k3 * r6) + I.p1 * (r2 + 2 * y * y) + 2 * I.p2 * x * y;
    px = I.fx * xd + I.cx;
    py = I.fy * yd + I.cy;
}

SynthCam makeCam(double fx, double fy, double cx, double cy,
                  double k1, double k2, double p1, double p2,
                  const std::array<double, 9>& R, const std::array<double, 3>& t) {
    SynthCam c;
    c.intr.fx = fx; c.intr.fy = fy; c.intr.cx = cx; c.intr.cy = cy;
    c.intr.k1 = k1; c.intr.k2 = k2; c.intr.p1 = p1; c.intr.p2 = p2; c.intr.k3 = 0;
    c.intr.width = 640; c.intr.height = 480; c.intr.valid = true;
    c.extr.R = R; c.extr.t = t; c.extr.valid = true;
    return c;
}

double dist3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

int main() {
    // 相机0：世界原点，无旋转，直接看向+Z。
    SynthCam cam0 = makeCam(900, 900, 320, 240, 0, 0, 0, 0,
                            {1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0});

    // 相机1：绕Y轴转15度，做出一定基线夹角，平移(150,0,50)。
    const double th = 15.0 * 3.14159265358979323846 / 180.0;
    const double cs = std::cos(th), sn = std::sin(th);
    SynthCam cam1 = makeCam(900, 900, 320, 240, 0, 0, 0, 0,
                            {cs, 0, sn, 0, 1, 0, -sn, 0, cs}, {150, 0, 50});

    const std::array<double, 3> Xw = {80, -50, 1200};   // 已知真值（mm）

    // ---- 用例1：无畸变，纯净 round-trip ----
    {
        double px0, py0, px1, py1;
        projectPoint(cam0, Xw, px0, py0);
        projectPoint(cam1, Xw, px1, py1);
        auto r = triangulateTwoViews(cam0.intr, cam0.extr, px0, py0,
                                     cam1.intr, cam1.extr, px1, py1);
        printf("[无畸变] 还原坐标 (%.4f, %.4f, %.4f)  残差 %.6f mm\n",
               r.point[0], r.point[1], r.point[2], r.residual);
        assert(r.valid);
        assert(dist3(r.point, Xw) < 1e-3);
        assert(r.residual < 1e-6);
    }

    // ---- 用例2：带畸变，验证 undistortNormalize 迭代求解也是对的 ----
    {
        SynthCam d0 = cam0, d1 = cam1;
        d0.intr.k1 = d1.intr.k1 = -0.15;
        d0.intr.k2 = d1.intr.k2 = 0.05;
        d0.intr.p1 = d1.intr.p1 = 0.001;
        d0.intr.p2 = d1.intr.p2 = -0.0008;

        double px0, py0, px1, py1;
        projectPoint(d0, Xw, px0, py0);
        projectPoint(d1, Xw, px1, py1);
        auto r = triangulateTwoViews(d0.intr, d0.extr, px0, py0,
                                     d1.intr, d1.extr, px1, py1);
        printf("[带畸变] 还原坐标 (%.4f, %.4f, %.4f)  残差 %.6f mm\n",
               r.point[0], r.point[1], r.point[2], r.residual);
        assert(r.valid);
        assert(dist3(r.point, Xw) < 0.05);   // 迭代去畸变收敛误差，容差稍放宽
        assert(r.residual < 1e-3);
    }

    // ---- 用例3：两条视线平行——几何退化，必须报告失败而不是硬凑结果 ----
    {
        std::array<double, 3> c0 = {0, 0, 0}, c1 = {100, 0, 0};
        std::array<double, 3> d0 = {0, 0, 1}, d1 = {0, 0, 1};   // 完全平行
        auto r = closestPointBetweenRays(c0, d0, c1, d1);
        printf("[平行退化] valid=%d（预期0）\n", int(r.valid));
        assert(!r.valid);
    }

    // ---- 用例4：未标定相机直接拒绝，不产出虚假结果 ----
    {
        CameraIntrinsics badIntr;   // 默认 valid=false
        auto r = triangulateTwoViews(badIntr, cam0.extr, 320, 240,
                                     cam1.intr, cam1.extr, 320, 240);
        printf("[未标定] valid=%d（预期0）\n", int(r.valid));
        assert(!r.valid);
    }

    // 再造一台相机2（绕Y轴-20度、平移到另一侧），用于 N 视图测试。
    const double th2 = -20.0 * 3.14159265358979323846 / 180.0;
    const double cs2 = std::cos(th2), sn2 = std::sin(th2);
    SynthCam cam2 = makeCam(950, 950, 320, 240, 0, 0, 0, 0,
                            {cs2, 0, sn2, 0, 1, 0, -sn2, 0, cs2}, {-160, 0, 60});
    // 相机3：绕X轴转10度、抬高一点，制造非共面的第四视角。
    const double th3 = 10.0 * 3.14159265358979323846 / 180.0;
    const double cs3 = std::cos(th3), sn3 = std::sin(th3);
    SynthCam cam3 = makeCam(880, 880, 320, 240, 0, 0, 0, 0,
                            {1, 0, 0, 0, cs3, -sn3, 0, sn3, cs3}, {40, -120, 30});

    auto projObs = [&](const SynthCam& c) {
        double px, py; projectPoint(c, Xw, px, py);
        return ViewObservation{ &c.intr, &c.extr, px, py };
    };

    // ---- 用例5：3 台相机 N 视图三角化 ----
    {
        ViewObservation obs[3] = { projObs(cam0), projObs(cam1), projObs(cam2) };
        auto r = triangulateMultiView(obs, 3);
        printf("[3视图] 还原 (%.4f, %.4f, %.4f)  残差 %.6f mm\n",
               r.point[0], r.point[1], r.point[2], r.residual);
        assert(r.valid);
        assert(dist3(r.point, Xw) < 1e-2);
    }

    // ---- 用例6：4 台相机 N 视图三角化，精度不应劣于少视图 ----
    {
        ViewObservation obs[4] = { projObs(cam0), projObs(cam1), projObs(cam2), projObs(cam3) };
        auto r = triangulateMultiView(obs, 4);
        printf("[4视图] 还原 (%.4f, %.4f, %.4f)  残差 %.6f mm\n",
               r.point[0], r.point[1], r.point[2], r.residual);
        assert(r.valid);
        assert(dist3(r.point, Xw) < 1e-2);
    }

    // ---- 用例7：4 台里有一台未标定，应自动跳过、用其余3台照常解算 ----
    {
        SynthCam bad = cam3; bad.intr.valid = false;   // 标记这一台未标定
        ViewObservation obs[4] = { projObs(cam0), projObs(cam1), projObs(cam2), projObs(bad) };
        auto r = triangulateMultiView(obs, 4);
        printf("[跳过未标定] valid=%d 还原 (%.4f, %.4f, %.4f)\n",
               int(r.valid), r.point[0], r.point[1], r.point[2]);
        assert(r.valid);                       // 剩3台有效，仍能解
        assert(dist3(r.point, Xw) < 1e-2);
    }

    // ---- 用例8：只有一台有效观测，无法三角化，必须报失败 ----
    {
        ViewObservation obs[1] = { projObs(cam0) };
        auto r = triangulateMultiView(obs, 1);
        printf("[单视图] valid=%d（预期0）\n", int(r.valid));
        assert(!r.valid);
    }

    // ---- 用例9：鲁棒剔除——4台里1台是坏视角（像素观测被故意打偏），
    //      robust 版本应把它剔掉、还原精度接近无污染，且 droppedMask
    //      正确标出坏视角。对照普通 multiView（不剔除）应明显更差。----
    {
        // cam1 的观测故意加 +25px 偏移，模拟这一帧的检测噪点/时间戳错位。
        ViewObservation good1 = projObs(cam1);
        ViewObservation bad1 = good1; bad1.px += 25.0; bad1.py -= 25.0;

        ViewObservation obsBad[4] = { projObs(cam0), bad1, projObs(cam2), projObs(cam3) };

        // 普通版：坏视角一起参与，结果被污染。
        auto rPlain = triangulateMultiView(obsBad, 4);
        // 鲁棒版：应剔掉坏视角。
        bool dropped[4] = { false, false, false, false };
        auto rRobust = triangulateMultiViewRobust(obsBad, 4, /*minKeep=*/2,
                                                   /*madScale=*/3.0, dropped);
        printf("[鲁棒剔除] 普通版误差=%.4f mm  鲁棒版误差=%.4f mm  "
               "dropped=[%d%d%d%d]\n",
               dist3(rPlain.point, Xw), dist3(rRobust.point, Xw),
               int(dropped[0]), int(dropped[1]), int(dropped[2]), int(dropped[3]));
        assert(rRobust.valid);
        assert(dropped[1]);                         // 坏视角(下标1)被正确剔除
        assert(!dropped[0] && !dropped[2] && !dropped[3]);   // 好视角不被误剔
        assert(dist3(rRobust.point, Xw) < 1e-2);    // 剔除后还原到干净精度
        // 剔除确实带来改善：鲁棒版应显著优于把坏视角算进去的普通版。
        assert(dist3(rRobust.point, Xw) < dist3(rPlain.point, Xw) * 0.5);
    }

    // ---- 用例10：全部视角一致（无坏点）时，鲁棒版不应误剔任何视角，
    //      结果与普通版一致。----
    {
        ViewObservation obsClean[4] = { projObs(cam0), projObs(cam1), projObs(cam2), projObs(cam3) };
        bool dropped[4] = { false, false, false, false };
        auto r = triangulateMultiViewRobust(obsClean, 4, 2, 3.0, dropped);
        printf("[鲁棒-无坏点] valid=%d 误差=%.4f mm dropped=[%d%d%d%d]\n",
               int(r.valid), dist3(r.point, Xw),
               int(dropped[0]), int(dropped[1]), int(dropped[2]), int(dropped[3]));
        assert(r.valid);
        assert(!dropped[0] && !dropped[1] && !dropped[2] && !dropped[3]);
        assert(dist3(r.point, Xw) < 1e-2);
    }

    // ---- 用例11：只有2台有效视角时，剔除无意义（剔掉任一台就没法三角化），
    //      鲁棒版应退回用两台的结果，不误伤。----
    {
        ViewObservation obs2[2] = { projObs(cam0), projObs(cam1) };
        bool dropped[2] = { false, false };
        auto r = triangulateMultiViewRobust(obs2, 2, 2, 3.0, dropped);
        printf("[鲁棒-仅2台] valid=%d 误差=%.4f mm dropped=[%d%d]\n",
               int(r.valid), dist3(r.point, Xw), int(dropped[0]), int(dropped[1]));
        assert(r.valid);
        assert(!dropped[0] && !dropped[1]);
        assert(dist3(r.point, Xw) < 1e-2);
    }

    printf("triangulation: ALL PASS\n");
    return 0;
}
