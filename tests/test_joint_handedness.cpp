// ===========================================================================
// test_joint_handedness.cpp
// 直接调用【生产代码】hm20::solveJointAngles()，验证左手在"右手档"下的
// 关节角符号，以及 mirrorOut 纠正之后能否还原成真值。
//
// 【跟上一版 test_joint_mirror_sign.cpp 的区别，很重要】
// 那个测试是我把提取公式重写了一遍再验——验的是我的理解，不是代码。
// 这个测试构造 SkeletonFrameResult 喂给真正的 solveJointAngles()，
// 验的是实际会跑起来的那条路径。
//
// 【构造原理】
// 腕部系按 +X 指向、+Y 拇指侧、Z=X×Y 造。对右手 Z 指手背外侧；同一套构造
// 描述左手时 Z 指掌侧。所以同一姿势的左手在这个系里的坐标 = 等效右手的坐标
// 把 z 取反。测试就按这个关系造左手数据，再看能不能把角度还原回去。
// ===========================================================================
#include "estimate/Hm20JointAngles.hpp"

#include <cstdio>
#include <cmath>

using namespace mocap::hm20;
using mocap::hm20::detail::matToQuat;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* msg) {
    if (ok) { ++g_pass; }
    else { ++g_fail; std::printf("  FAIL: %s\n", msg); }
}
static void near(double got, double want, double tol, const char* msg) {
    const bool ok = std::fabs(got - want) <= tol;
    if (!ok) std::printf("  FAIL: %s  期望%.4f 实得%.4f\n", msg, want, got);
    ok ? ++g_pass : ++g_fail;
}

// 由指定的骨轴(第0列)造一个旋转矩阵，其余两列 Gram-Schmidt 补齐。
// 关节角提取只用第0列，其余两列取什么不影响结果。
static Mat3 axisToMat(Vec3 x) {
    const double n = std::sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
    for (auto& v : x) v /= n;
    Vec3 ref{0, 0, 1};
    if (std::fabs(x[2]) > 0.9) ref = Vec3{0, 1, 0};
    Vec3 y{ref[1]*x[2] - ref[2]*x[1], ref[2]*x[0] - ref[0]*x[2], ref[0]*x[1] - ref[1]*x[0]};
    const double ny = std::sqrt(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
    for (auto& v : y) v /= ny;
    Vec3 z{x[1]*y[2] - x[2]*y[1], x[2]*y[0] - x[0]*y[2], x[0]*y[1] - x[1]*y[0]};
    return Mat3{x[0], y[0], z[0],      // 行主序，第0列 = x
                x[1], y[1], z[1],
                x[2], y[2], z[2]};
}

// 给定 (屈曲, 外展, PIP屈曲) 造一根手指的三段骨轴
static void fingerAxes(double flex, double abd, double pip,
                       Vec3& a0, Vec3& a1, Vec3& a2) {
    auto mk = [&](double fl) {
        return Vec3{std::cos(fl) * std::cos(abd), std::cos(fl) * std::sin(abd), -std::sin(fl)};
    };
    a0 = mk(flex);
    a1 = mk(flex + pip);
    a2 = mk(flex + pip + 0.5);
}

// mirrorZ=true 时把骨轴的 z 取反 —— 这就是"物理左手 + 系统按右手建系"
static SkeletonFrameResult makeFrame(double flex, double abd, double pip, bool mirrorZ) {
    SkeletonFrameResult r{};
    r.valid = true;              // 【别漏】solveJointAngles 第一行就查它
    r.wristPoseValid = true;
    r.wristR = Mat3{1,0,0, 0,1,0, 0,0,1};      // 腕部系 = 世界系，隔离掉无关变量
    r.wristT = Vec3{0, 0, 0};
    for (int s = 0; s < 16; ++s) r.segSource[size_t(s)] = SegSource::None;

    for (int f = 0; f < 5; ++f) {
        Vec3 a0, a1, a2;
        fingerAxes(flex, abd, pip, a0, a1, a2);
        if (mirrorZ) { a0[2] = -a0[2]; a1[2] = -a1[2]; a2[2] = -a2[2]; }
        const int sProx = 1 + f * 3;
        r.segQuat[size_t(sProx + 0)] = matToQuat(axisToMat(a0));
        r.segQuat[size_t(sProx + 1)] = matToQuat(axisToMat(a1));
        r.segQuat[size_t(sProx + 2)] = matToQuat(axisToMat(a2));
        for (int j = 0; j < 3; ++j)
            r.segSource[size_t(sProx + j)] = SegSource::Geometry;   // measured() 要它
    }
    return r;
}

int main() {
    const double flex = 30.0 * M_PI / 180.0;
    const double abd  = 12.0 * M_PI / 180.0;
    const double pip  = 40.0 * M_PI / 180.0;

    JointAngleResult prev{};
    WristHold hold{};

    // ---- ① 右手基准 ----
    std::printf("== ① 右手基准（解算手性 = 物理手性）==\n");
    hold = WristHold{};
    const auto R = solveJointAngles(makeFrame(flex, abd, pip, false), prev, hold, 24, false);
    check(R.nValidFingers == 5, "五指都解出来了");
    near(R.q[4], flex, 1e-6, "食MCP屈 = +30°");
    near(R.q[5], abd,  1e-6, "食MCP展 = +12°");
    // PIP 屈曲现在是【正】的，与 MCP 屈曲、与 jointLimits(){0.0,1.8} 一致。
    // 修复前这里是 -40°，会被 clamp 到 0，PIP 永远输不出信号。
    near(R.q[6], pip, 1e-6, "食PIP  = +40°（屈曲为正，与限位表一致）");
    std::printf("   食指: 屈=%.1f° 展=%.1f° PIP=%.1f°\n",
                R.q[4] * 57.2958, R.q[5] * 57.2958, R.q[6] * 57.2958);

    // ---- ② 左手 + 右手档 + 不纠正 = 现状（你现在拿到的数）----
    std::printf("== ② 物理左手 + 系统右手档 + 不纠正（修复前的现状）==\n");
    hold = WristHold{};
    const auto L0 = solveJointAngles(makeFrame(flex, abd, pip, true), prev, hold, 24, false);
    std::printf("   食指: 屈=%.1f° 展=%.1f° PIP=%.1f°\n",
                L0.q[4] * 57.2958, L0.q[5] * 57.2958, L0.q[6] * 57.2958);
    near(L0.q[4], -flex, 1e-6, "食MCP屈【反号】");
    near(L0.q[5],  abd,  1e-6, "食MCP展【不变】← 我之前说反的就是这一维");
    near(L0.q[6], -pip,  1e-6, "食PIP 【反号】");

    // ---- ③ 左手 + 右手档 + mirrorOut 纠正 = 本次改动 ----
    std::printf("== ③ 同上，但打开 mirrorOut（本次改动）==\n");
    hold = WristHold{};
    const auto L1 = solveJointAngles(makeFrame(flex, abd, pip, true), prev, hold, 24, true);
    std::printf("   食指: 屈=%.1f° 展=%.1f° PIP=%.1f°\n",
                L1.q[4] * 57.2958, L1.q[5] * 57.2958, L1.q[6] * 57.2958);
    for (int f = 1; f < 5; ++f) {
        const int b = 4 + (f - 1) * 3;
        near(L1.q[size_t(b + 0)], R.q[size_t(b + 0)], 1e-6, "纠正后屈曲 == 右手基准");
        near(L1.q[size_t(b + 1)], R.q[size_t(b + 1)], 1e-6, "纠正后外展 == 右手基准");
        near(L1.q[size_t(b + 2)], R.q[size_t(b + 2)], 1e-6, "纠正后PIP  == 右手基准");
    }
    // 拇指四维单独查（它走 q[0..3] 那条特殊分支，含 IP 耦合）
    for (int i = 0; i < 4; ++i)
        near(L1.q[size_t(i)], R.q[size_t(i)], 1e-6, "纠正后拇指四维 == 右手基准");

    // ---- ④ mirrorOut 对右手数据必须是恒等（别把好的弄坏）----
    std::printf("== ④ 右手数据打开 mirrorOut：应当等于左手不纠正的结果（对称性）==\n");
    hold = WristHold{};
    const auto R1 = solveJointAngles(makeFrame(flex, abd, pip, false), prev, hold, 24, true);
    for (int i = 0; i < 16; ++i)
        near(R1.q[size_t(i)], L0.q[size_t(i)], 1e-6, "镜像是对合的");

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
