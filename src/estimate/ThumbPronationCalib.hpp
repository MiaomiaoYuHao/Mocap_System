#pragma once
// ===========================================================================
// ThumbPronationCalib.hpp —— 把 thumbPronation0 从"猜"变成"量"
//
// 【它解决什么】
// Hm20MarkerModel::thumbPronation0 是第一掌骨的常数解剖旋前，幅值约 1.4~1.57 rad。
// 幅值可以从解剖学推、也能从模型里量出来(probe_thumb_axis.py 给的缺口是 73°)，
// 但【符号】只能从数据里定 —— 它取决于 rotX/rotY/rotZ 的约定、行主序、以及
// refine() 内部镜像的实现细节，任何一处读错都会得到相反的结论。
//
// 而符号错了不是"稍微差一点"：73° 的缺口会变成 163°，比不补还差。
//
// 【怎么量】用【拇指三颗球全部可见】的帧。那些帧的观测是完整的，
// IK 拟合残差 fingerIkRmseMm[0] 就是一个直接可读的判据：
//     · 旋前补对了 -> markerFK 能摆出一个对掌的拇指 -> 残差掉到 3~5mm
//     · 补反了     -> 残差比不补还高
// 于是把 thumbPronation0 在 [-1.8, +1.8] 上扫一遍，取残差中位数最小的那个。
//
// 【为什么这个判据靠得住，而 thumbRollOffsetRad 那个靠不住】
// roll(绕骨轴自转)在每节只有 1 颗球时【不可观测】，所以调 thumbRollOffsetRad
// 永远没有数字能告诉你调对没有，只能对着画面猜。而 thumbPronation0 作用在
// marker 的【位置】上 —— 位置是测得到的，所以残差就是判据。
//
// 【用法】
//   ThumbPronationCalib cal;
//   // 每帧（在 process() 之后）：
//   cal.feed(result);                       // 只收拇指 3/3 可见且 wristPoseValid 的帧
//   if (cal.ready()) {                      // 攒够帧数 + 姿势有多样性
//       const auto r = cal.sweep(ik.get()); // 扫一遍，几十毫秒，别放在每帧路径上
//       if (r.confident) ik->setThumbPronation0(r.bestRad);
//   }
//
// sweep() 会把整条残差曲线返回来 —— 【一定要看曲线，不要只看最小值】。
// 真正的解会有一个明显的碗底(最小值比两端低 2 倍以上)；如果曲线是平的，
// 说明这批帧里拇指没怎么动，量出来的是噪声，此时 confident=false。
//
// 零 Qt / 零 onnxruntime，只依赖 Hm20IkRefiner.hpp 和 HandSkeletonAssociator.hpp。
// ===========================================================================

#include "estimate/Hm20AssocContract.hpp"   // 【只要类型，不要那个类】原来这里 include 的是
                                            // HandSkeletonAssociator.hpp（闭包 5776 行），但本文件
                                            // 一次都没提到 Hm20SkeletonAssociator，要的只是
                                            // SkeletonFrameResult / Hm20Config / IHm20InferenceBackend
                                            // 这些类型 —— 它们在 contract 里（闭包 1557 行）。
                                            // estimate 目录下有 7 个头都犯了同一个错，每个多吃 4219 行，
                                            // 而这些头又被 UI 层层包含，代价是乘出来的。
#include "estimate/Hm20IkRefiner.hpp"

#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

namespace mocap {
namespace hm20 {

struct ThumbPronationSweep {
    bool   confident   = false;
    double bestRad     = 0.0;
    double bestRmseMm  = -1.0;
    double baselineMm  = -1.0;   // thumbPronation0 = 0 时的残差（= 现状）
    double worstRmseMm = -1.0;
    double contrast    = 0.0;    // worst / best，> 2.0 才算碗底明显
    int    nFrames     = 0;
    std::vector<std::pair<double, double>> curve;   // (rad, 残差中位数 mm)
    bool ikNotReady = false;     // IK 的 anchor/骨长还没标定好，refine() 全部返回 false
    const char* verdict() const {
        if (ikNotReady)     return "IK 参数未就绪：先跑完自标定(anchor + 骨长)，再标拇指旋前";
        if (nFrames < 30)   return "帧数不足：把拇指三颗球都露出来，慢慢做几个对掌动作";
        if (contrast < 2.0) return "曲线是平的：这批帧里拇指活动范围太小，量出来的是噪声";
        return "OK";
    }
};

class ThumbPronationCalib {
public:
    struct Config {
        int    maxFrames    = 240;   // 环形缓冲上限；再多也不会更准，只会更慢
        int    minFrames    = 30;
        double gridLo       = -1.80;
        double gridHi       =  1.80;
        double gridStep     =  0.10;
        // 姿势多样性门：这批帧里拇指远节球在腕部系里的活动范围(mm)。
        // 拇指基本不动时任何旋前值的残差都差不多，扫出来的是噪声。
        double minSpreadMm  = 25.0;
        // 相邻帧去重：位移小于它的帧不收，避免缓冲被"手停着"的帧塞满。
        double minStepMm    = 3.0;
        double minContrast  = 2.0;
    };

    void setConfig(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }
    void reset() { frames_.clear(); hasLast_ = false; }
    int  frameCount() const { return int(frames_.size()); }

    // 只收【拇指 3/3 可见 + 腕部位姿本帧解出来】的帧。其余一律丢弃 ——
    // 用不完整的观测去标定一个常数，等于把遮挡的偏差焊进参数里。
    bool feed(const SkeletonFrameResult& r) {
        if (!r.valid || !r.wristPoseValid) return false;
        for (int m = 5; m <= 7; ++m) if (!r.markers[size_t(m)].observed) return false;

        Frame f;
        for (int m = 0; m < kNumMarkers; ++m) {
            f.mk[size_t(m)] = r.markers[size_t(m)].posWorld;
            f.ob[size_t(m)] = r.markers[size_t(m)].observed;
        }
        f.R = r.wristR;
        f.t = r.wristT;

        // 拇指远节球在【腕部系】里的位置，用于去重和多样性判据
        const Vec3 dpL = toLocal(f.R, f.t, f.mk[7]);
        if (hasLast_ && dist(dpL, lastDpL_) < cfg_.minStepMm) return false;
        lastDpL_ = dpL; hasLast_ = true;
        f.dpLocal = dpL;

        if (int(frames_.size()) >= cfg_.maxFrames) frames_.erase(frames_.begin());
        frames_.push_back(f);
        return true;
    }

    // 这批帧里拇指远节球在腕部系里的活动范围（包围盒对角线）
    double spreadMm() const {
        if (frames_.size() < 2) return 0.0;
        Vec3 lo = frames_.front().dpLocal, hi = lo;
        for (const Frame& f : frames_)
            for (int d = 0; d < 3; ++d) {
                lo[size_t(d)] = std::min(lo[size_t(d)], f.dpLocal[size_t(d)]);
                hi[size_t(d)] = std::max(hi[size_t(d)], f.dpLocal[size_t(d)]);
            }
        return dist(lo, hi);
    }

    bool ready() const {
        return int(frames_.size()) >= cfg_.minFrames && spreadMm() >= cfg_.minSpreadMm;
    }

    // 扫一遍。ik 必须是已经设好 anchor/骨长/手性/backTemplate 的那一个。
    //
    // 【会临时改动 ik 的状态】thumbPronation0 和时序先验都会被动，
    // 函数返回前恢复 thumbPronation0，并 resetPrior()。别在 worker 线程正在
    // 跑推理的时候调它 —— 放到用户点"标定拇指"按钮的那条路径上。
    ThumbPronationSweep sweep(Hm20IkRefiner* ik) const {
        ThumbPronationSweep out;
        out.nFrames = int(frames_.size());
        if (!ik || frames_.empty()) return out;

        const double save = ik->thumbPronation0();
        std::vector<double> rms;
        rms.reserve(frames_.size());

        for (double a = cfg_.gridLo; a <= cfg_.gridHi + 1e-9; a += cfg_.gridStep) {
            ik->setThumbPronation0(a);
            ik->resetPrior();        // 每档从同一个起点开始，否则档与档之间不可比
            rms.clear();
            for (const Frame& f : frames_) {
                std::array<Vec3, kNumMarkers> mk = f.mk;
                std::array<Quat, kNumSegments> q{};
                if (!ik->refine(mk, f.ob, f.R, f.t, q)) continue;
                const double e = ik->lastFrameInfo().rmseMm[0];
                if (e == e && e >= 0.0) rms.push_back(e);   // e==e 滤 NaN
            }
            if (rms.empty()) continue;
            std::nth_element(rms.begin(), rms.begin() + rms.size() / 2, rms.end());
            out.curve.emplace_back(a, rms[rms.size() / 2]);
        }
        ik->setThumbPronation0(save);
        ik->resetPrior();

        if (out.curve.empty()) { out.ikNotReady = true; return out; }
        double best = 1e18, worst = -1.0, base = -1.0, bestA = 0.0;
        for (const auto& kv : out.curve) {
            if (kv.second < best) { best = kv.second; bestA = kv.first; }
            if (kv.second > worst) worst = kv.second;
            if (std::fabs(kv.first) < cfg_.gridStep * 0.5) base = kv.second;
        }
        out.bestRad     = bestA;
        out.bestRmseMm  = best;
        out.worstRmseMm = worst;
        out.baselineMm  = base;
        out.contrast    = (best > 1e-6) ? (worst / best) : 0.0;
        out.confident   = (out.nFrames >= cfg_.minFrames)
                        && (spreadMm() >= cfg_.minSpreadMm)
                        && (out.contrast >= cfg_.minContrast);
        return out;
    }

private:
    struct Frame {
        std::array<Vec3, kNumMarkers> mk{};
        std::array<bool, kNumMarkers> ob{};
        Mat3 R{};
        Vec3 t{};
        Vec3 dpLocal{};
    };
    static double dist(const Vec3& a, const Vec3& b) {
        const double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    static Vec3 toLocal(const Mat3& R, const Vec3& t, const Vec3& p) {
        const double x = p[0]-t[0], y = p[1]-t[1], z = p[2]-t[2];
        return Vec3{{R[0]*x + R[3]*y + R[6]*z,
                     R[1]*x + R[4]*y + R[7]*z,
                     R[2]*x + R[5]*y + R[8]*z}};   // Rᵀ·(p−t)
    }

    Config cfg_{};
    std::vector<Frame> frames_;
    Vec3 lastDpL_{};
    bool hasLast_ = false;
};

}  // namespace hm20
}  // namespace mocap
