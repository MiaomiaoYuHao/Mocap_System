// =============================================================================
// test_rom_calibration.cpp —— ROM 标定引擎 v2 的单测
// =============================================================================
// 【这个单测的目的不是"覆盖分支"，是复现真机上报的三个症状，并证明新引擎
//   能把它们各自修掉】。合成数据按真机观察到的失效模式构造：
//
//   场景 A  理想数据                 -> 覆盖度应该接近满
//   场景 B  握拳时腕部系失效         -> v1 的 MCP 会采到大量保持值
//   场景 C  四指 PIP 在遮挡帧恒为 0   -> v1 会把方向判反（握拳少、张开满）
//   场景 D  某一维整段是负向的       -> v1 无解，v2 应该自动纠正
//   场景 E  拇MCP 结构性恒零         -> 应该被判成 AxisDead 而不是"标定成功"
//
// 编译：g++ -std=c++17 -I src tests/test_rom_calibration.cpp -o /tmp/t && /tmp/t
// =============================================================================
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>

#include "estimate/RomCalibration.hpp"

using namespace mocap::hm20::rom;

static int g_fail = 0;
#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { std::printf("  [FAIL] " msg "\n", ##__VA_ARGS__); ++g_fail; } \
    else         { std::printf("  [ ok ] " msg "\n", ##__VA_ARGS__); } } while (0)

// 一次「张开->握拳」往复的 curl 轨迹（0=张开 1=握拳），带端点停留
static double curlAt(int frame, int total) {
    const double phase = 2.0 * M_PI * 3.0 * double(frame) / double(total); // 3 个来回
    const double raw = 0.5 - 0.5 * std::cos(phase);
    // 端点停留：把正弦往两端压，模拟"张开到底停一下、握到底停一下"
    return std::clamp(1.5 * (raw - 0.5) + 0.5, 0.0, 1.0);
}

struct Scenario {
    bool dropWristOnFist   = false;  // 握拳时腕部位姿失效 -> MCP 保持上一帧
    bool pipZeroOnOcclusion= false;  // 遮挡时 PIP 恒为 0（骨轴退化）
    bool markDegenerate    = true;   // 是否把退化如实标成 Degenerate
    bool negativeIndexPip  = false;  // 食PIP 整段方向相反
    bool thumbMcpDead      = false;  // 拇MCP 结构性恒零
    bool weakOwnStrongCommon = false; // 本指参考失效，只能靠四指共同参考定方向
    bool weakIndexPipNoSign = false;  // 食PIP 完全无方向证据，测试运行时同组耦合
};

// 生成一段标定素材。真值：屈曲维在 curl=0 时取 lo0、curl=1 时取 hi0。
static std::vector<FrameObs> makeSession(const Scenario& sc, int nFrames = 900) {
    std::mt19937 rng(1234);
    std::normal_distribution<double> noise(0.0, 0.012);   // ~0.7° 抖动
    const auto& lim = mocap::jointLimits();

    std::vector<FrameObs> out;
    out.reserve(size_t(nFrames));
    std::array<double, kDof> held{};       // 上一帧值，模拟"保持"
    for (int f = 0; f < nFrames; ++f) {
        FrameObs o;
        o.frameTsNs = int64_t(f) * 8333333LL;              // 120fps
        const double c = curlAt(f, nFrames);
        for (int k = 0; k < kFingers; ++k) { o.curl[k] = c; o.curlValid[k] = 1; }

        // 握拳深处手背最容易被挡
        const bool wristLost = sc.dropWristOnFist && (c > 0.62);
        for (int i = 0; i < kDof; ++i) {
            const size_t u = size_t(i);
            const bool abd = dofIsAbduction(i);
            const double lo0 = lim[u].lo * 0.6, hi0 = lim[u].hi * 0.92;
            double v;
            if (abd) v = 0.15 * lim[u].hi * std::sin(0.05 * f) + noise(rng);
            else     v = lo0 + (hi0 - lo0) * c + noise(rng);

            uint8_t st = uint8_t(DofState::Measured);

            const bool isPip = (i == 6 || i == 9 || i == 12 || i == 15);
            const bool isMcp = (i == 4 || i == 5 || i == 7 || i == 8 ||
                                i == 10 || i == 11 || i == 13 || i == 14);

            if (sc.negativeIndexPip && i == 6) v = -v;      // 方向整段反过来
            if (sc.weakIndexPipNoSign && i == 6) v = 0.5 * noise(rng); // 无方向证据的噪声
            if (sc.thumbMcpDead && i == 2) {
                v = 0.0;
                st = uint8_t(sc.markDegenerate ? DofState::Degenerate : DofState::Measured);
            }
            if (wristLost && isMcp) {                        // MCP 保持上一帧
                v = held[u];
                st = uint8_t(DofState::Held);
            }
            if (wristLost && isPip && sc.pipZeroOnOcclusion) {   // 骨轴退化
                v = 0.0 + 0.3 * noise(rng);
                st = uint8_t(sc.markDegenerate ? DofState::Degenerate : DofState::Measured);
            }
            // 多候选方向参考：候选0=本指 curl，候选1=四指共同 curl。
            // 旧单测只关心 q/state，这里补上后新判据才能在合成数据上被覆盖。
            if (!abd) {
                o.signRef[0][u] = c;
                o.signRefValid[0][u] = 1;
                o.signRef[1][u] = c;
                o.signRefValid[1][u] = 1;
                if (sc.weakOwnStrongCommon && (i == 0 || i == 6)) {
                    o.signRef[0][u] = 0.5;      // 本指参考无跨度 -> 候选0失效
                    o.signRefValid[0][u] = 1;
                    v = -v;                     // 真实方向与本指参考相反，只能靠共同参考纠正
                }
            }
            o.q[u] = v;
            held[u] = v;
            o.state[u] = st;
        }
        out.push_back(o);
    }
    return out;
}

// v1 的行为：全收 + 全局 p2/p98 + 无方向。用来做对照。
struct V1 {
    std::array<std::vector<double>, kDof> s;
    void observe(const FrameObs& o) {
        for (int i = 0; i < kDof; ++i) s[size_t(i)].push_back(o.q[size_t(i)]);
    }
    double coverage() {
        const auto& lim = mocap::jointLimits();
        double sum = 0; int n = 0;
        for (int i : flexDofs()) {
            auto v = s[size_t(i)];
            if (v.size() < 30) { ++n; continue; }
            std::sort(v.begin(), v.end());
            const double lo = v[size_t(0.02 * double(v.size() - 1) + 0.5)];
            const double hi = v[size_t(0.98 * double(v.size() - 1) + 0.5)];
            const double full = lim[size_t(i)].hi - lim[size_t(i)].lo;
            sum += std::min(1.0, (hi - lo) / full); ++n;
        }
        return n ? sum / n : 0;
    }
    // v1 映射后，握拳端输出 vs 张开端输出 —— 用来量"是不是反的"
    double mappedAt(int i, double q) {
        auto v = s[size_t(i)];
        std::sort(v.begin(), v.end());
        const double lo = v[size_t(0.02 * double(v.size() - 1) + 0.5)];
        const double hi = v[size_t(0.98 * double(v.size() - 1) + 0.5)];
        if (hi - lo < 1e-3) return 0;
        return std::clamp((q - lo) / (hi - lo), 0.0, 1.0);
    }
};

static CalibResult run(const std::vector<FrameObs>& sess, Calibrator& cal) {
    cal.beginPass(false);
    for (const auto& o : sess) cal.observe(o);
    return cal.finish();
}

int main() {
    // ---------------- 场景 A：理想数据 ----------------
    std::printf("\n场景 A —— 理想数据（无遮挡、方向正常）\n");
    {
        Calibrator cal; Mapper map;
        auto sess = makeSession({});
        auto r = run(sess, cal);
        std::printf("  覆盖度 %.1f%%  达标 %d/11  方向纠正 %d\n",
                    r.coverageFlex * 100, r.nOkFlex, r.nSignFlipped);
        CHECK(r.ready, "标定成功");
        CHECK(r.coverageFlex > 0.80, "覆盖度 > 80%%（实得 %.1f%%）", r.coverageFlex * 100);
        CHECK(r.nSignFlipped == 0, "没有误判方向");
    }

    // ---------------- 场景 B+C：真机失效模式 ----------------
    std::printf("\n场景 B+C —— 握拳时腕部系失效 + PIP 骨轴退化（真机实况）\n");
    {
        Scenario sc; sc.dropWristOnFist = true; sc.pipZeroOnOcclusion = true;
        auto sess = makeSession(sc);

        // v1 对照：状态位被忽略，全部当有效样本收
        V1 v1;
        for (const auto& o : sess) v1.observe(o);
        const double covV1 = v1.coverage();

        Calibrator cal;
        auto r = run(sess, cal);
        std::printf("  v1 覆盖度 %.1f%%   ->   v2 覆盖度 %.1f%%（达标 %d/11）\n",
                    covV1 * 100, r.coverageFlex * 100, r.nOkFlex);
        CHECK(covV1 < 0.70, "v1 复现了「覆盖度上不去」（实得 %.1f%%）", covV1 * 100);
        CHECK(r.coverageFlex > covV1, "v2 覆盖度高于 v1");
        CHECK(r.ready, "v2 标定成功");

        // 反向检查：v1 在食PIP 上握拳端的输出是不是比张开端小
        const double qOpen = -0.0, qFist = 1.7;
        std::printf("  v1 食PIP 映射：张开端 %.2f  握拳端 %.2f\n",
                    v1.mappedAt(6, qOpen), v1.mappedAt(6, qFist));

        // v2：PIP 的退化样本必须被拒
        const auto& d = r.dof[6];
        std::printf("  v2 食PIP：接受 %d / 见到 %d，其中骨轴退化拒收 %d\n",
                    d.nSamples, d.nSeen, d.nReject[size_t(RejectCode::AxisDegen)]);
        CHECK(d.nReject[size_t(RejectCode::AxisDegen)] > 0, "退化样本被识别并拒收");
        CHECK(DofStatus(d.status) != DofStatus::AxisDead, "食PIP 没被误判成死维");
    }

    // ---------------- 场景 D：方向相反 ----------------
    std::printf("\n场景 D —— 食PIP 整段方向相反（v1 无解）\n");
    {
        Scenario sc; sc.negativeIndexPip = true;
        auto sess = makeSession(sc);

        V1 v1;
        for (const auto& o : sess) v1.observe(o);
        // 真值：curl=1（握拳）时 q ≈ -hi0，curl=0 时 q ≈ -lo0
        const auto& lim = mocap::jointLimits();
        const double lo0 = lim[6].lo * 0.6, hi0 = lim[6].hi * 0.92;
        const double qOpenTrue = -lo0, qFistTrue = -hi0;
        const double v1open = v1.mappedAt(6, qOpenTrue);
        const double v1fist = v1.mappedAt(6, qFistTrue);
        std::printf("  v1 食PIP：张开端输出 %.2f，握拳端输出 %.2f  %s\n",
                    v1open, v1fist, (v1fist < v1open) ? "<- 反了" : "");
        CHECK(v1fist < v1open, "v1 复现了「握拳少、张开满」");

        Calibrator cal; Mapper map;
        auto r = run(sess, cal);
        std::printf("  v2 判定食PIP 方向 = %+d（两端中位差 %.1f°）\n",
                    r.dof[6].sign, r.dof[6].signDeltaRad * 57.2957795);
        CHECK(r.dof[6].sign == -1, "v2 判出方向是反的");
        CHECK(r.dof[6].signResolved, "方向是判出来的，不是默认值");

        std::array<double, kDof> qo{}, qf{};
        qo[6] = qOpenTrue; qf[6] = qFistTrue;
        std::array<uint8_t, kDof> stAll; stAll.fill(uint8_t(DofState::Measured));
        const double outOpen = map.apply(qo, cal, &stAll)[6];
        map.reset();
        const double outFist = map.apply(qf, cal, &stAll)[6];
        std::printf("  v2 食PIP：张开端输出 %.2f rad，握拳端输出 %.2f rad\n",
                    outOpen, outFist);
        CHECK(outFist > outOpen, "v2 映射后握拳端更大（方向修正生效）");
    }

    // ---------------- 场景 E：拇MCP 结构性恒零 ----------------
    std::printf("\n场景 E —— 拇MCP 结构性恒零（a0 与 a1 是同一根骨头）\n");
    {
        Scenario sc; sc.thumbMcpDead = true;
        auto sess = makeSession(sc);
        Calibrator cal;
        auto r = run(sess, cal);
        const auto& d = r.dof[2];
        std::printf("  拇MCP status=%d 样本 %d/%d 退化拒收 %d\n",
                    int(d.status), d.nSamples, d.nSeen,
                    d.nReject[size_t(RejectCode::AxisDegen)]);
        CHECK(DofStatus(d.status) == DofStatus::AxisDead,
              "拇MCP 被判成「骨轴退化」而不是悄悄当成标定好了");

        // 死维必须冻结在中立位，不能钳到行程端点
        Mapper map;
        std::array<double, kDof> q{};
        std::array<uint8_t, kDof> st; st.fill(uint8_t(DofState::Measured));
        const double o = map.apply(q, cal, &st)[2];
        const auto& lim = mocap::jointLimits();
        std::printf("  拇MCP 输出 %.3f rad（行程 %.2f..%.2f）\n", o, lim[2].lo, lim[2].hi);
        CHECK(std::fabs(o - 0.0) < 1e-9, "死维冻结在解剖中立位");
    }

    // ---------------- 场景 F：退化帧保持输出 ----------------
    std::printf("\n场景 F —— 运行时对退化帧保持上一帧输出\n");
    {
        Calibrator cal; Mapper map;
        auto sess = makeSession({});
        run(sess, cal);
        std::array<double, kDof> q{}; q[6] = 1.2;
        std::array<uint8_t, kDof> st; st.fill(uint8_t(DofState::Measured));
        const double good = map.apply(q, cal, &st)[6];
        std::array<double, kDof> bad{}; bad[6] = 0.0;      // 退化帧的假值
        st[6] = uint8_t(DofState::Degenerate);
        const double heldOut = map.apply(bad, cal, &st)[6];
        std::printf("  正常帧输出 %.3f，退化帧输出 %.3f\n", good, heldOut);
        CHECK(std::fabs(heldOut - good) < 1e-9, "退化帧保持了上一帧输出");
    }

    // ---------------- 场景 G：补标合并 ----------------
    std::printf("\n场景 G —— 补标（第二轮只动了小指）\n");
    {
        Calibrator cal;
        auto s1 = makeSession({});
        cal.beginPass(false);
        for (const auto& o : s1) cal.observe(o);
        const double cov1 = cal.finish().coverageFlex;

        // 第二轮：只有小指在动，其余维全部 Held
        auto s2 = makeSession({}, 400);
        for (auto& o : s2)
            for (int i = 0; i < kDof; ++i)
                if (dofFinger(i) != 4) o.state[size_t(i)] = uint8_t(DofState::Held);
        cal.beginPass(true);
        for (const auto& o : s2) cal.observe(o);
        const auto r2 = cal.finish();
        std::printf("  第一轮覆盖 %.1f%% -> 补标后 %.1f%%（达标 %d/11）\n",
                    cov1 * 100, r2.coverageFlex * 100, r2.nOkFlex);
        CHECK(r2.coverageFlex >= cov1 - 0.02, "补标没有把已标好的维冲掉");
        CHECK(r2.ready, "补标后仍然是已标定状态");
    }

    // ---------------- 场景 H：多候选参考量（本指失效，共同参考兜底）----------------
    std::printf("\n场景 H —— 本指方向参考失效，四指共同参考兜底\n");
    {
        Scenario sc; sc.weakOwnStrongCommon = true;
        auto sess = makeSession(sc);
        Calibrator cal;
        auto r = run(sess, cal);
        std::printf("  拇CMC屈 sign=%+d resolved=%d；食PIP sign=%+d resolved=%d\n",
                    r.dof[0].sign, int(r.dof[0].signResolved),
                    r.dof[6].sign, int(r.dof[6].signResolved));
        CHECK(r.dof[0].sign == -1 && r.dof[0].signResolved,
              "拇CMC屈服曲方向由共同参考纠正为 -1");
        CHECK(r.dof[6].sign == -1 && r.dof[6].signResolved,
              "食PIP 本指参考失效时由共同参考纠正为 -1");
    }

    // ---------------- 场景 I：弱PIP运行时同组耦合 ----------------
    std::printf("\n场景 I —— 食PIP方向不明时，运行时跟随同组可靠PIP\n");
    {
        Scenario sc; sc.weakIndexPipNoSign = true;
        auto sess = makeSession(sc);
        Calibrator cal; Mapper map;
        auto r = run(sess, cal);
        std::printf("  食PIP status=%d resolved=%d corr=%.3f\n",
                    int(r.dof[6].status), int(r.dof[6].signResolved), r.dof[6].signCorr);
        CHECK(!r.dof[6].signResolved,
              "食PIP 没有单维方向证据（status 可能是 SignUnknown 或 PriorFilled）");

        std::array<double, kDof> qOpen{}, qFist{};
        for (int i : {9, 12, 15}) {
            qOpen[size_t(i)] = r.dof[size_t(i)].rawLo;
            qFist[size_t(i)] = r.dof[size_t(i)].rawHi;
        }
        qOpen[6] = qFist[6] = r.dof[6].rawLo;
        std::array<uint8_t, kDof> st; st.fill(uint8_t(DofState::Measured));
        const double oOpen = map.apply(qOpen, cal, &st)[6];
        map.reset();
        const double oFist = map.apply(qFist, cal, &st)[6];
        std::printf("  食PIP 耦合同组：张开端 %.3f  握拳端 %.3f\n", oOpen, oFist);
        CHECK(oFist > oOpen + 0.1, "方向不明维跟随了同组可靠PIP的弯曲趋势");
    }

    std::printf("\n==== %s ====\n", g_fail ? "有用例失败" : "全部通过");
    return g_fail ? 1 : 0;
}
