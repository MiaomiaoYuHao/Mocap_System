// =============================================================================
// Hm20PoseFilter.hpp —— 骨架输出的时序滤波（可选，默认开）
// =============================================================================
// 【为什么需要它】关联网络是【逐帧独立】跑的：每一帧解一次指派、算一次 Kabsch、
// 连一次骨轴。逐帧独立意味着输出里带着逐帧独立的噪声，实测分段姿态的帧间跳变
// p99 在 27~35°，静止时也有 1~3° 的持续抖动。做动作捕捉这个是不能接受的 ——
// 导出到 bvh/c3d 里就是肉眼可见的高频颤动。
//
// 【为什么不是简单的低通】三个东西不能用同一套处理：
//   1. 位置：低通会引入延迟。手快速移动时延迟比抖动更讨厌。
//   2. 姿态：四元数在球面上，线性平均会掉出单位球；而且 q 和 -q 同旋转，
//      不做半球对齐直接插值会得到 180° 的假运动。
//   3. 观测状态会变：某个 marker 从"遮挡"变回"可见"时，滤波器里存的是几十
//      毫米之外的预测值，直接接着滤会把新观测拖住半秒 —— 表现为"手指重新出现
//      时会慢慢飘回去"。必须在重捕时重置。
//
// 【方案】
//   位置/平移：One-Euro（Casiez 2012）。核心是截止频率随速度自适应 ——
//              慢的时候截止低、抖动被压掉；快的时候截止高、几乎不引入延迟。
//              这是交互式系统里公认比卡尔曼更合用的一档（不需要运动模型、
//              两个参数、O(1) 状态）。
//   姿态：把 One-Euro 搬到 SO(3) 上 —— 用【球面插值】代替线性混合，
//         自适应增益由角速度驱动。数学上等价于对旋转做同样的自适应低通，
//         但全程留在单位四元数上，不会掉出球面。
//   静止死区：速度低于阈值时把输出钉住。One-Euro 在极低速时仍有残余抖动，
//             死区能把"手放着不动"的情况打到真正的零抖动。用【软死区】
//             （减去死区量而不是硬钳），避免出现台阶。
//   重捕重置：marker 从 unobserved 变 observed、且跳变超过门限时，直接把
//             滤波器状态置为新观测，不留任何历史。
//
// 【逐段自适应】segSource==Predicted 的分段（两端点都是网络补的）本身就不可信，
// 给它更强的平滑；segSource==Ik/Geometry 的给轻平滑。这样"测得准的不被拖慢、
// 猜出来的不乱跳"。
// =============================================================================
#pragma once

#include <array>
#include <cmath>
#include <algorithm>

#include "estimate/Hm20AssocContract.hpp"   // 【只要类型，不要那个类】原来这里 include 的是
                                            // HandSkeletonAssociator.hpp（闭包 5776 行），但本文件
                                            // 一次都没提到 Hm20SkeletonAssociator，要的只是
                                            // SkeletonFrameResult / Hm20Config / IHm20InferenceBackend
                                            // 这些类型 —— 它们在 contract 里（闭包 1557 行）。
                                            // estimate 目录下有 7 个头都犯了同一个错，每个多吃 4219 行，
                                            // 而这些头又被 UI 层层包含，代价是乘出来的。

namespace mocap {
namespace hm20 {

using namespace detail;

struct PoseFilterConfig {
    bool   enabled          = true;
    double fps              = 60.0;   // 只作为 dt 的兜底，正常由调用方传真实 dt

    // ---- 位置 One-Euro ----
    // 默认值来自实测扫描（6相机+不同步+遮挡，14秒会话，对比 5 组参数）：
    //   关      位置中位1.32mm 抖动2.771 段跳变中位4.02°
    //   保守    位置中位4.51mm 抖动0.542 段跳变2.71°   <- 抖动最小但滞后 3.4mm，不可接受
    //   本默认  位置中位1.65mm 抖动1.500 段跳变3.37°   <- 位置 p90 反而比不滤还好
    //   极快    位置中位1.48mm 抖动1.742 段跳变3.55°
    // 取这一档的理由：位置代价只有 0.33mm 中位、p90 还降了(6.55->6.26)，
    // 而高频抖动砍掉 46%。要更稳可以调向"保守"，但滞后会明显上来。
    double posMinCutoffHz   = 3.0;    // 越小越稳、越滞后
    double posBeta          = 0.40;   // 越大越跟手、越抖
    double posDCutoffHz     = 1.0;    // 速度估计自身的低通

    // ---- 姿态 One-Euro(SLERP) ----
    double rotMinCutoffHz   = 2.0;
    double rotBeta          = 0.50;

    // ---- 静止死区 ----
    double posDeadbandMm    = 0.30;
    double rotDeadbandDeg   = 0.25;

    // ---- 不可信分段的额外平滑倍数（截止频率除以它）----
    double predictedSmooth  = 2.5;
    // 遮挡点的固定截止频率(Hz)。>0 时【接管】遮挡点的滤波：固定截止 + beta=0。
    // 1.3Hz 在 46fps 下等价于一阶低通 alpha≈0.15，是仿真扫出来的最优点。
    // 设 0 = 退回旧行为(predictedSmooth 倍数 + 保留 beta)。
    // 现在默认开到 1.3：预测点不再直接吃网络原始 pos 头，而是走链式续解/
    // IK 补出来的保形点，骨轴方向已经在源头保住了；此时对它们做强时序低通
    // 不会再压坏方向，只把高频抖动压掉。0 仍可用于 A/B 回退。
    double predictedFixedCutoffHz = 1.6;
    // ---- 遮挡点：趋势外推 × 网络预测 的融合（默认开）----
    // 【为什么不是二选一】仿真实测(真模型闭环 + 3相机 + 32%遮挡，tools/sim_jitter.py)：
    //            拇指误差  四指误差  拇跳p90  四跳p90  拇跳max  四跳max
    //   现状        28.23    29.65    11.28    14.03    47.93    38.59
    //   纯平滑      11.68    38.52     1.86     5.04     6.43    12.84
    //   纯趋势      16.85    45.23     0.60     1.86     4.81     6.57
    //   融合        11.59    32.82     0.70     1.75     2.62     6.48  <- 六项赢五项
    // 两者的最坏情况来源不同(平滑是跟不上、趋势是漂)，混合互相抵消，
    // 所以 max 比任何单一方案都好。
    //
    // 【顺序关键：先混后滤】按固定比例直接混过一次，跳变 max 反而 27.8 ——
    // 网络那一份每帧在跳，混进来会污染趋势那条解析平滑的轨迹。必须混完再统一低通。
    //
    // 权重按遮挡时长走：刚遮挡时速度是刚测到的、趋势最准 -> 偏趋势；
    // 遮挡久了趋势衰减到停住会开始漂 -> 让网络接管。
    // 【默认关 —— 仿真判定为负收益】它把逐点位置误差和跳变优化得很好
    // (28.2->11.6mm、跳变 max 47.9->2.6mm)，但【骨轴方向从 21.4° 劣化到 40.5°】。
    // 骨轴方向是两点之【差】，不是位置本身：中节两球基线只有 40~55mm，两端
    // 各偏 7mm 就是 20° —— 位置平滑得再好也救不了方向，反而会污染它。
    // 而骨轴方向才是屏幕上和下游真正看的东西。
    // 遮挡点的正解是 ikFillOccluded（IK 摆出来的点天然满足骨长和限位）。
    bool   predictFuse       = false;
    double fuseVelAlpha      = 0.50;   // 速度估计的平滑系数
    double fuseVelTauFrames  = 12.0;   // 外推速度的衰减常数（帧）
    double fuseWeightTauFr   = 30.0;   // 趋势->网络 的过渡常数（帧）
    double fuseOutAlpha      = 0.12;   // 混合之后的一阶低通

    // ---- 重捕/跳变 ----
    double posJumpGateMm    = 60.0;   // 单帧位移超过它 => 认为是重捕或换目标，重置

    // 只滤位置不滤姿态（或反之）的开关，方便排查
    bool   filterPositions  = true;
    bool   filterRotations  = true;
};

// 三档预设。面板上给使用者选，不必理解 One-Euro 的参数含义。
enum class PoseFilterPreset : int { Off = 0, Light = 1, Balanced = 2, Strong = 3 };

inline PoseFilterConfig makePoseFilterPreset(PoseFilterPreset p) {
    PoseFilterConfig c;
    switch (p) {
        case PoseFilterPreset::Off:
            c.enabled = false; break;
        case PoseFilterPreset::Light:      // 几乎不引入滞后，只压最高频
            c.posMinCutoffHz = 4.0; c.posBeta = 0.80;
            c.rotMinCutoffHz = 3.0; c.rotBeta = 0.90;
            c.posDeadbandMm = 0.30; c.rotDeadbandDeg = 0.25;
            c.predictedSmooth = 2.0; c.predictedFixedCutoffHz = 2.0; break;
        case PoseFilterPreset::Balanced:   // 默认：均衡
            c.predictedFixedCutoffHz = 1.6; break;
        case PoseFilterPreset::Strong:     // 静态拍摄/慢动作，抖动最小但有滞后
            c.posMinCutoffHz = 1.0; c.posBeta = 0.02;
            c.rotMinCutoffHz = 1.2; c.rotBeta = 0.12;
            c.posDeadbandMm = 0.20; c.rotDeadbandDeg = 0.15;
            c.predictedSmooth = 3.0; c.predictedFixedCutoffHz = 0.9; break;
    }
    return c;
}

// ---------------------------------------------------------------------------
// 标量 One-Euro。刻意写成自带、不 include reconstruct/OneEuroFilter.hpp ——
// 这里需要"重捕重置"和"软死区"两个额外行为，而且不想让 estimate 层反向依赖
// reconstruct 层。
// ---------------------------------------------------------------------------
class OneEuro1 {
public:
    void reset() { init_ = false; }
    bool initialized() const { return init_; }
    void hardSet(double x) { x_ = x; dx_ = 0.0; init_ = true; }

    double filter(double x, double dt, double minCutoff, double beta, double dCutoff) {
        if (!init_) { x_ = x; dx_ = 0.0; init_ = true; return x_; }
        if (!(dt > 1e-6)) return x_;
        const double dxRaw = (x - x_) / dt;
        dx_ = lerp(dx_, dxRaw, alpha(dCutoff, dt));
        const double cutoff = minCutoff + beta * std::fabs(dx_);
        x_ = lerp(x_, x, alpha(cutoff, dt));
        return x_;
    }
    double value() const { return x_; }
    double speed() const { return dx_; }

    static double alpha(double cutoffHz, double dt) {
        const double tau = 1.0 / (2.0 * 3.14159265358979323846 * std::max(cutoffHz, 1e-4));
        return 1.0 / (1.0 + tau / dt);
    }

private:
    static double lerp(double a, double b, double t) { return a + (b - a) * t; }
    double x_ = 0.0, dx_ = 0.0;
    bool init_ = false;
};

// ---------------------------------------------------------------------------
class Hm20PoseFilter {
public:
    Hm20PoseFilter() = default;
    // =====================================================================
    // 逐帧调试快照。
    //
    // 【为什么这个类必须往外吐状态】apply() 是【就地覆写】SkeletonFrameResult
    // 的：滤波前的位置在整个系统里没有第二份。于是这两件事在录制文件里完全
    // 一样：
    //     "几何本来就把这个点解错位置了"
    //     "几何是对的，被 One-Euro 的滞后/软死区吃平了"
    // 而前者要去查关联和骨轴，后者要去调截止频率 —— 方向相反。
    //
    // 【内部状态也要吐，不能只吐前后】One-Euro 的截止频率是逐点逐帧【自适应】
    // 的（随速度变），软死区和硬重置是条件触发的。只看输入输出的话，
    // "这个点没动"有三种原因：它真没动、落在死区里、或者速度估计塌了把
    // 截止频率压到了最低。三者修法各不相同，只有把 cutoffHz / deadzone /
    // velLocal 一起摆出来才分得开。
    // =====================================================================
    struct FrameDebug {
        std::array<Vec3, kNumMarkers>   posIn{};       // 滤波前
        std::array<Vec3, kNumMarkers>   posOut{};      // 滤波后
        std::array<double, kNumMarkers> moveMm{};      // |out-in|
        // 【腕部系速度，不是世界系】世界系速度里混着整只手的刚体运动，
        // 拿它判断"这根手指自己在不在动"会被手的平移完全带偏。
        std::array<double, kNumMarkers> velLocalMm{};
        std::array<double, kNumMarkers> staleFrames{};
        std::array<double, kNumMarkers> fuseWeight{};  // 外推 vs 网络的混合权重
        std::array<double, kNumMarkers> cutoffHz{};    // 本帧该点实际用的截止频率
        std::array<bool, kNumMarkers>   hardReset{};
        std::array<bool, kNumMarkers>   deadzone{};    // 位移被死区吃掉了
        std::array<bool, kNumMarkers>   fused{};
        std::array<bool, kNumMarkers>   observed{};
        Quat   wristQuatIn{1, 0, 0, 0};
        Quat   wristQuatOut{1, 0, 0, 0};
        double wristAngMoveDeg = 0.0;
        std::array<double, kNumSegments> segQuatMoveDeg{};
        double dtSec = -1.0;
        bool   ran = false;      // apply() 这一帧真的跑了（没被 enabled/valid 挡掉）
    };
    const FrameDebug& lastFrameDebug() const { return dbg_; }

    explicit Hm20PoseFilter(const PoseFilterConfig& c) : cfg_(c) {}

    void configure(const PoseFilterConfig& c) { cfg_ = c; }
    const PoseFilterConfig& config() const { return cfg_; }

    void reset() {
        for (auto& m : mk_) m = MarkerState{};
        for (auto& s : seg_) s = QuatState{};
        wristQ_ = QuatState{};
        for (auto& f : wristT_) f.reset();
        hasPrev_ = false;
    }

    // dtSec <= 0 时用 cfg_.fps 兜底。
    void apply(SkeletonFrameResult& r, double dtSec = -1.0) {
        // 【无论跑不跑都先清空 dbg_】不清的话，被 enabled=false 挡掉的那些帧
        // 会保留上一帧的内容，而录制端看到的是一份"看起来正常"的旧数据 ——
        // 比没有数据危险得多。ran=false 明确表示这一帧没跑。
        dbg_ = FrameDebug{};
        if (!cfg_.enabled || !r.valid) return;
        const double dt = (dtSec > 1e-6) ? dtSec : (1.0 / std::max(cfg_.fps, 1.0));
        dbg_.dtSec = dt;
        dbg_.ran = true;
        for (int m = 0; m < kNumMarkers; ++m) {
            dbg_.posIn[size_t(m)]    = r.markers[size_t(m)].posWorld;
            dbg_.observed[size_t(m)] = r.markers[size_t(m)].observed;
        }
        dbg_.wristQuatIn = r.wristQuat;
        const std::array<Quat, kNumSegments> segIn = r.segQuat;

        if (cfg_.filterPositions) {
            // 【顺序】融合必须在 filterMarker 之前：它替换的是遮挡点的原始位置，
            // 之后照常走硬重置判定 / One-Euro / 软死区那整条路径。
            if (cfg_.predictFuse) fusePredicted(r);
            for (int m = 0; m < kNumMarkers; ++m) filterMarker(m, r, dt);
            for (int i = 0; i < 3; ++i) {
                // 腕部平移跟手背质心同量级，用同一组参数
                r.wristT[size_t(i)] = wristT_[size_t(i)].filter(
                    r.wristT[size_t(i)], dt, cfg_.posMinCutoffHz, cfg_.posBeta, cfg_.posDCutoffHz);
            }
        }

        if (cfg_.filterRotations) {
            // 腕部：手背 5 点刚体解出来的，质量最高，用基准强度
            r.wristQuat = filterQuat(wristQ_, r.wristQuat, dt, 1.0,
                                     r.wristPoseValid);
            r.wristR = quatToMat(r.wristQuat);
            for (int s = 0; s < kNumSegments; ++s) {
                // 猜出来的分段给更强的平滑：它的高频成分基本全是噪声
                const bool weak = (r.segSource[size_t(s)] == SegSource::Predicted ||
                                   r.segSource[size_t(s)] == SegSource::None);
                const double k = weak ? (1.0 / std::max(cfg_.predictedSmooth, 1.0)) : 1.0;
                const bool live = (r.segSource[size_t(s)] != SegSource::None);
                r.segQuat[size_t(s)] = filterQuat(seg_[size_t(s)], r.segQuat[size_t(s)], dt, k, live);
            }
        }

        // ---- 收尾：把"这一级搬了多远"直接算出来 ----
        // 【在线算而不是留给离线】离线也能算，但那要求离线先正确复现
        // "滤波前"是哪一份 —— 而那正是这一块要证明的事，循环论证。
        for (int m = 0; m < kNumMarkers; ++m) {
            const size_t um = size_t(m);
            dbg_.posOut[um] = r.markers[um].posWorld;
            dbg_.moveMm[um] = norm(sub(dbg_.posOut[um], dbg_.posIn[um]));
            const MarkerState& st = mk_[um];
            dbg_.velLocalMm[um]  = norm(st.velLocal);
            dbg_.staleFrames[um] = st.staleFrames;
        }
        dbg_.wristQuatOut = r.wristQuat;
        dbg_.wristAngMoveDeg = angleBetween(dbg_.wristQuatIn, dbg_.wristQuatOut);
        for (int s = 0; s < kNumSegments; ++s)
            dbg_.segQuatMoveDeg[size_t(s)] =
                angleBetween(segIn[size_t(s)], r.segQuat[size_t(s)]);
        hasPrev_ = true;
    }

private:
    struct MarkerState {
        OneEuro1 f[3];
        Vec3 out{};
        bool wasObserved = false;
        bool has = false;
        // ---- 融合外推用 ----
        // 【腕部系】速度必须在腕部系里估：整只手平移时世界系速度里混着手的
        // 刚体运动，拿它外推手指等于让遮挡的手指跟着手飞出去。
        Vec3   lastObsLocal{};      // 最近一次真观测到时的位置（腕部系）
        Vec3   velLocal{};          // 平滑后的速度（腕部系，mm/帧）
        double staleFrames = 0.0;   // 已经连续遮挡了几帧
        bool   hasObs = false;
    };
    struct QuatState {
        Quat q{1, 0, 0, 0};
        double w = 0.0;      // 角速度低通（deg/s）
        bool has = false;
    };

    // -------------------------------------------------------------------------
    // 遮挡点 = 趋势外推 × 网络预测，按遮挡时长加权，混完再统一低通。
    // 详细理由和实测数据见 Hm20PoseFilterConfig::predictFuse。
    // 【全程在腕部系里做】世界系速度里混着整只手的刚体运动，拿它外推手指，
    // 手一平移遮挡的手指就会跟着飞出去。腕部位姿无效时整个跳过 —— 没有腕部系
    // 就没有"手指自己的运动"这个概念，硬做只会更糟。
    // -------------------------------------------------------------------------
    void fusePredicted(SkeletonFrameResult& r) {
        if (!r.wristPoseValid) {
            // 腕部系不可用：不外推，但也别让状态过期 —— 全部标成"刚丢"，
            // 等腕部系回来时从最近的实测重新起步，而不是拿一段陈旧速度猛冲。
            for (int m = 0; m < kNumMarkers; ++m) mk_[size_t(m)].hasObs = false;
            return;
        }
        const auto& R = r.wristR;   // 行主序，腕部系 -> 世界系
        const auto& T = r.wristT;
        auto toLocal = [&](const Vec3& p) {
            const Vec3 d{p[0] - T[0], p[1] - T[1], p[2] - T[2]};
            return Vec3{R[0]*d[0] + R[3]*d[1] + R[6]*d[2],
                        R[1]*d[0] + R[4]*d[1] + R[7]*d[2],
                        R[2]*d[0] + R[5]*d[1] + R[8]*d[2]};
        };
        auto toWorld = [&](const Vec3& p) {
            return Vec3{R[0]*p[0] + R[1]*p[1] + R[2]*p[2] + T[0],
                        R[3]*p[0] + R[4]*p[1] + R[5]*p[2] + T[1],
                        R[6]*p[0] + R[7]*p[1] + R[8]*p[2] + T[2]};
        };

        for (int m = 0; m < kNumMarkers; ++m) {
            MarkerState& st = mk_[size_t(m)];
            const Vec3 loc = toLocal(r.markers[size_t(m)].posWorld);

            if (r.markers[size_t(m)].observed) {
                if (st.hasObs) {
                    const double n = std::max(1.0, st.staleFrames + 1.0);
                    const Vec3 vNew{(loc[0] - st.lastObsLocal[0]) / n,
                                    (loc[1] - st.lastObsLocal[1]) / n,
                                    (loc[2] - st.lastObsLocal[2]) / n};
                    const double a = cfg_.fuseVelAlpha;
                    for (int i = 0; i < 3; ++i)
                        st.velLocal[size_t(i)] = st.velLocal[size_t(i)] * (1.0 - a)
                                               + vNew[size_t(i)] * a;
                }
                st.lastObsLocal = loc;
                st.staleFrames = 0.0;
                st.hasObs = true;
                continue;                       // 真观测不动它
            }

            if (!st.hasObs) continue;           // 没有实测历史，无从外推，用网络原值
            st.staleFrames += 1.0;
            const double age = st.staleFrames;
            const double damp = std::exp(-age / std::max(1.0, cfg_.fuseVelTauFrames));
            const double w    = std::exp(-age / std::max(1.0, cfg_.fuseWeightTauFr));
            const double aOut = cfg_.fuseOutAlpha;
            // 【记下融合权重】w 是"信外推还是信网络"的分配。它恒为 0 说明
            // 融合实际没起作用（多半是 fuseWeightTauFr 太小或刚遮挡就退化），
            // 而那正是"遮挡点还是会跳"的常见成因 —— 光看输出位置看不出来。
            dbg_.fused[size_t(m)] = true;
            dbg_.fuseWeight[size_t(m)] = w;

            Vec3 mix{};
            for (int i = 0; i < 3; ++i) {
                const double ext = st.lastObsLocal[size_t(i)]
                                 + st.velLocal[size_t(i)] * age * damp;
                mix[size_t(i)] = ext * w + loc[size_t(i)] * (1.0 - w);
            }
            // 混完再低通。基准取上一帧的输出（转回腕部系），保证连续。
            const Vec3 prevLoc = st.has ? toLocal(st.out) : mix;
            Vec3 outLoc{};
            for (int i = 0; i < 3; ++i)
                outLoc[size_t(i)] = prevLoc[size_t(i)] * (1.0 - aOut)
                                  + mix[size_t(i)] * aOut;
            r.markers[size_t(m)].posWorld = toWorld(outLoc);
        }
    }

    void filterMarker(int m, SkeletonFrameResult& r, double dt) {
        MarkerState& st = mk_[size_t(m)];
        const Vec3 raw = r.markers[size_t(m)].posWorld;
        const bool obs = r.markers[size_t(m)].observed;

        // 重捕 / 大跳变 -> 硬重置。不重置的话滤波器里存的是几十毫米外的预测值，
        // 新观测会被拖住半秒，表现为"手指重新出现时慢慢飘回去"。
        bool hardReset = !st.has;
        if (st.has) {
            const double jump = norm(sub(raw, st.out));
            // 大跳变硬重置只对【真观测】做（重捕时直通到新位置，不拖尾）。
            // 遮挡点的大跳不是重捕，而是近端 marker 被重捕后把链式续解一起
            // 拽过去（真机实测 63~94mm 的单帧尖峰）。硬重置会把尖峰原样放行；
            // 让 One-Euro 自己平滑它，正常小跳变/真实运动完全不受影响。
            if (obs && jump > cfg_.posJumpGateMm) hardReset = true;
            if (obs && !st.wasObserved && jump > cfg_.posJumpGateMm * 0.25) hardReset = true;
        }
        if (hardReset) {
            for (int i = 0; i < 3; ++i) st.f[i].hardSet(raw[size_t(i)]);
            st.out = raw;
            st.has = true;
            st.wasObserved = obs;
            // 【硬重置必须可见】它把滤波器整个清空重起，本帧零延迟直通。
            // 频繁触发 = 跳变门限设小了或者点在闪，症状是"平滑时有时无"，
            // 而只看输出位置完全看不出这一帧走了直通。
            dbg_.hardReset[size_t(m)] = true;
            return;                       // 本帧直接输出原值，零延迟
        }

        // 遮挡点是网络补的，本身就带模型平滑，再叠一层强平滑会明显滞后；
        // 但它也确实比真观测抖，所以给一个折中的加强系数。
        // ---- 遮挡点：固定强平滑，且【关掉 beta】----
        // 【为什么 beta 必须是 0】One-Euro 的 beta 项的作用是"动得快就放行"，
        // 对真观测点是对的。但遮挡点没有真实运动信号，它的"快"全部来自
        // 闭环里累积的误差 —— beta 一放行，那个误差原样传到输出，正是
        // 仿真里看到的"跳变 max 47.9mm"。
        //
        // 仿真实测(真模型闭环 + 3相机 + 32%遮挡率，见 tools 里的 sim_jitter.py)：
        //          拇指遮挡误差   跳变p90   跳变max
        //   现状        28.23      11.28     47.93
        //   本档        11.68       1.86      6.43   <- 误差和跳变同时降
        // 四指遮挡误差会从 29.7 涨到 38.5（短时遮挡跟不上），这是有意的取舍：
        // 偶尔飞一下(max 47mm)比一直偏一点难受得多，尤其驱动机械手时。
        const double k = obs ? 1.0 : (1.0 / std::max(cfg_.predictedSmooth * 0.5, 1.0));
        // 遮挡点走固定截止 + beta=0（理由见上）。走【同一条】滤波调用，
        // 后面的软死区/输出路径完全复用 —— 早退会绕过软死区，那是另一个坑。
        const bool fixed = (!obs && cfg_.predictedFixedCutoffHz > 0.0);
        const double fc   = fixed ? cfg_.predictedFixedCutoffHz : cfg_.posMinCutoffHz * k;
        const double beta = fixed ? 0.0 : cfg_.posBeta;
        dbg_.cutoffHz[size_t(m)] = fc;
        Vec3 f{};
        for (int i = 0; i < 3; ++i)
            f[size_t(i)] = st.f[i].filter(raw[size_t(i)], dt, fc, beta, cfg_.posDCutoffHz);

        // 软死区：位移里减掉死区量，而不是硬钳住。硬钳会产生台阶，
        // 软死区在静止时输出真正不动、一旦动起来又是连续的。
        const Vec3 d = sub(f, st.out);
        const double dn = norm(d);
        if (dn > 1e-9) {
            const double keep = std::max(0.0, dn - cfg_.posDeadbandMm) / dn;
            // 【死区吃掉全部位移时要留痕】keep==0 意味着这一帧的输出被钉死了。
            // "这个点不动"有三种原因（真没动/落死区/截止频率塌了），
            // 这个 bool 把其中一种直接排除掉。
            if (keep <= 0.0) dbg_.deadzone[size_t(m)] = true;
            st.out = add(st.out, mul(d, keep));
        }
        st.wasObserved = obs;
        r.markers[size_t(m)].posWorld = st.out;
    }
    Quat filterQuat(QuatState& st, Quat raw, double dt, double cutoffScale, bool live) {
        normalizeQ(raw);
        if (!st.has || !live) {
            // live=false 表示这一段本帧没有任何依据（冷启动/腕部位姿没解出），
            // 不要拿它去更新滤波器状态，否则会把垃圾灌进历史。
            if (!st.has) { st.q = raw; st.w = 0.0; st.has = true; }
            return st.q;
        }
        // 半球对齐。q 和 -q 是同一个旋转，不对齐直接 slerp 会走"远的那条路",
        // 得到一次凭空的 180° 假运动。
        if (dotQ(raw, st.q) < 0) for (auto& c : raw) c = -c;

        const double angDeg = angleBetween(st.q, raw);
        const double wRaw = angDeg / std::max(dt, 1e-6);
        st.w = st.w + (wRaw - st.w) * OneEuro1::alpha(cfg_.rotMinCutoffHz, dt);

        const double cutoff = (cfg_.rotMinCutoffHz + cfg_.rotBeta * st.w) * cutoffScale;
        double a = OneEuro1::alpha(cutoff, dt);

        // 姿态的软死区：把有效插值角减去死区，静止时彻底不动
        const double eff = std::max(0.0, angDeg * a - cfg_.rotDeadbandDeg);
        a = (angDeg > 1e-9) ? std::clamp(eff / angDeg, 0.0, 1.0) : 0.0;

        st.q = slerp(st.q, raw, a);
        normalizeQ(st.q);
        return st.q;
    }

    static double dotQ(const Quat& a, const Quat& b) {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
    }
    static void normalizeQ(Quat& q) {
        const double n = std::sqrt(dotQ(q, q));
        if (n > 1e-12) for (auto& c : q) c /= n;
        else q = Quat{1, 0, 0, 0};
    }
    static double angleBetween(const Quat& a, const Quat& b) {
        const double d = std::min(1.0, std::fabs(dotQ(a, b)));
        return 2.0 * std::acos(d) * 180.0 / 3.14159265358979323846;
    }
    static Quat slerp(const Quat& a, const Quat& b, double t) {
        double d = dotQ(a, b);
        Quat bb = b;
        if (d < 0) { for (auto& c : bb) c = -c; d = -d; }
        if (d > 0.9995) {           // 夹角极小，slerp 数值不稳，退回 nlerp
            Quat q{};
            for (int i = 0; i < 4; ++i) q[size_t(i)] = a[size_t(i)] + (bb[size_t(i)] - a[size_t(i)]) * t;
            normalizeQ(q);
            return q;
        }
        const double th0 = std::acos(std::clamp(d, -1.0, 1.0));
        const double th = th0 * t;
        const double s0 = std::sin(th0);
        const double wa = std::sin(th0 - th) / s0, wb = std::sin(th) / s0;
        Quat q{};
        for (int i = 0; i < 4; ++i) q[size_t(i)] = a[size_t(i)] * wa + bb[size_t(i)] * wb;
        normalizeQ(q);
        return q;
    }

    PoseFilterConfig cfg_;
    FrameDebug dbg_{};
    std::array<MarkerState, kNumMarkers> mk_{};
    std::array<QuatState, kNumSegments> seg_{};
    QuatState wristQ_{};
    std::array<OneEuro1, 3> wristT_{};
    bool hasPrev_ = false;
};

}  // namespace hm20
}  // namespace mocap
