// ===========================================================================
// test_joint_mirror_sign.cpp —— 独立数值算例：左手在"右手档"下哪几维反号
// ===========================================================================
// 【它和 test_joint_handedness.cpp 是什么关系】
//
// 这个文件【不 include 工程里的任何头文件】，把关节角提取式按 Hm20JointAngles
// 里的写法【独立重写一遍】再验算。所以它单独存在时【不能证明生产代码的行为】——
// 它证明的是"按这个公式推，符号该这么变"。
//
// 真正验生产代码的是 tests/test_joint_handedness.cpp，那个直接调
// hm20::solveJointAngles()。
//
// 【那为什么还留着它】因为两个是【独立的两条路】：
//   · 这个：从公式独立推导 -> 屈曲反号、PIP反号、外展不变
//   · 那个：跑真实函数    -> 屈=30→-30、PIP=-40→+40、展=12→12
// 两条互不依赖的路给出同一个结论，比任何一条单独成立都更硬。
// 而且这个文件零依赖、一条 g++ 命令就能跑，你不想编整个工程时可以只跑它。
//
// 编译运行：
//   g++ -std=c++20 tests/test_joint_mirror_sign.cpp -o t && ./t
//
// 【推导前提】腕部系按 +X 指向、+Y 拇指侧、Z=X×Y 构造。对右手 Z 指手背外侧；
// 同一套构造描述左手时 Z 指掌侧。所以同姿势的左手在这个系里的坐标
// = 等效右手的坐标把 z 取反。下面就按这个关系造数据。
// ===========================================================================
#include <array>
#include <cmath>
#include <cstdio>

using V = std::array<double, 3>;
static double dot(const V& a, const V& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
static V cross(const V& a, const V& b) {
    return {a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]};
}
static V unit(V v) { const double n = std::sqrt(dot(v, v)); return {v[0]/n, v[1]/n, v[2]/n}; }

struct Ang { double flex, abd, pip; };

// 与 Hm20JointAngles.hpp 里的提取式逐行对应（改那边时这里要同步改）
static Ang extract(V a0, V a1) {
    Ang r{};
    r.flex = std::atan2(-a0[2], std::sqrt(a0[0]*a0[0] + a0[1]*a0[1]));
    r.abd  = std::atan2(a0[1], a0[0]);
    r.pip  = std::acos(std::fmax(-1.0, std::fmin(1.0, dot(a0, a1))));
    V h = unit(cross(V{0, 0, 1}, a0));   // 与 Hm20JointAngles 同步：屈曲为正
    if (dot(cross(a0, a1), h) < 0) r.pip = -r.pip;
    return r;
}

int main() {
    // 一根屈曲 30°、外展 12°、PIP 再屈 40° 的手指
    const double fl = 30 * M_PI / 180, ab = 12 * M_PI / 180, pp = 40 * M_PI / 180;
    auto axis = [&](double f) {
        return unit({std::cos(f) * std::cos(ab), std::cos(f) * std::sin(ab), -std::sin(f)});
    };
    const V a0 = axis(fl), a1 = axis(fl + pp);

    const Ang R = extract(a0, a1);                                   // 右手
    const Ang L = extract({a0[0], a0[1], -a0[2]},                    // 左手：z 取反
                          {a1[0], a1[1], -a1[2]});

    std::printf("                 屈曲       外展       PIP\n");
    std::printf("右手(基准)   %8.3f  %8.3f  %8.3f  (rad)\n", R.flex, R.abd, R.pip);
    std::printf("左手(z取反)  %8.3f  %8.3f  %8.3f  (rad)\n", L.flex, L.abd, L.pip);
    std::printf("             %8.1f  %8.1f  %8.1f  (右手,度)\n",
                R.flex * 57.29578, R.abd * 57.29578, R.pip * 57.29578);
    std::printf("             %8.1f  %8.1f  %8.1f  (左手,度)\n\n",
                L.flex * 57.29578, L.abd * 57.29578, L.pip * 57.29578);

    int bad = 0;
    auto want = [&](bool ok, const char* msg) {
        std::printf("  %-34s %s\n", msg, ok ? "OK" : "FAIL");
        if (!ok) ++bad;
    };
    want(std::fabs(L.flex + R.flex) < 1e-12, "屈曲：反号");
    want(std::fabs(L.abd  - R.abd ) < 1e-12, "外展：不变  ← 我曾经说反的就是这维");
    want(std::fabs(L.pip  + R.pip ) < 1e-12, "PIP ：反号");

    std::printf("\n%s\n", bad ? "FAILED"
        : "结论：反号的是【屈曲类】(屈曲/PIP/IP)，外展不变。\n"
          "      与 test_joint_handedness.cpp 跑生产代码得到的结论一致。");
    return bad;
}
