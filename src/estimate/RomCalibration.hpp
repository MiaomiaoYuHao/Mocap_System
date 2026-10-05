// =============================================================================
// RomCalibration.hpp —— ROM 标定引擎 v2（替换 Hm20JointAngles.hpp 里的 v1）
// =============================================================================
// 【为什么整个换掉，而不是在 v1 上打补丁】
// v1 的 RomCalibrator 只做一件事：把每一维的样本存起来，取 p2/p98 当区间。
// 这个做法隐含了三条假设，而真机上三条【全部不成立】：
//
//   假设一「进来的每个样本都是本帧新算出来的」
//     实际：solveJointAngles 在腕部系失效时【只更新 PIP】，MCP 屈曲/外展
//     保持上一帧的值，然后 fingerValid 仍然置 true。observe() 照单全收，
//     于是 MCP 那两维被灌进大量【重复的旧值】。握拳时手背最容易被挡，
//     也就是说：越是握到底，MCP 越是在采「张开时的那个值」。
//     直接后果就是 MCP 屈曲这一维的行程被削掉一大截。
//
//   假设二「角度随屈曲单调增大」
//     实际不一定。ROM 归一化是 u=(q-lo)/(hi-lo)，它对【方向】完全无知：
//     一个随握拳而【减小】的信号，映射完仍然是握拳→0、张开→满。
//     这就是「有几个是反的」。v1 无论怎么调分位数都救不了这一类，
//     因为方向信息根本不在 min/max 里。
//
//   假设三「四指 PIP 的两条骨轴总是不同的两根骨头」
//     实际：a0 取自 segQuat[近节]，而 computeSegmentQuats 里
//         dirProxUse = (anchor -> pp球)          需要 wristPoseValid
//         dirProxUse = (pp球   -> mp球)  ← 退化   wristPoseValid 为假时
//     而 solveJointAngles 里的 a1【恒等于】(pp球 -> mp球)。
//     也就是说腕部位姿一失效，a0 和 a1 变成【同一个向量】，
//     PIP = acos(1) = 0。不是不准，是恒等于零。
//     而手背恰恰是握拳时最容易被四指挡住的地方。
//     拇指更彻底：f==0 时 dirProxUse 永远走退化分支，所以【拇MCP 恒为 0】。
//
// 三条叠起来正好解释了报上来的三个症状：
//   · 覆盖度卡在 50% 上下  —— PIP 的样本是「真实屈曲值」和「恒零」的混合，
//     p2 落在 0、p98 落在某个中间值，(hi-lo)/1.92 差不多就是一半
//   · 握拳少、张开满      —— 混合里握拳端反而更接近 0，方向被整个反过来
//   · 四元数对得上但角度不对 —— 四元数确实是对的，坏在【从四元数取角】这一步
//     的轴选取上，所以「验证四元数」这条路查不到它
//
// 【这一版怎么做】
//   1) 逐维、逐帧判「这个样本能不能用」，不能用的记下原因（不是静默丢弃）
//   2) 用多个【与关节角无关】的参考信号判方向：
//      · 本指 curl（原始折角）
//      · 四指共同 curl（四指一起握拳/张开，压住单指噪声）
//      · 拇CMC屈专用参考（拇指尖到食指MCP的距离）
//      逐维选“样本足够、跨度够大、|秩相关|最高”的那条；相关不足时才退回
//      两端中位差。这样“中位差小但整体单调”的维不会漏判，单指噪声也不会
//      把整指趋势带反。
//   3) 区间在【带符号空间】v = sign*q 里定，于是映射恒为「屈曲→变大」
//   4) 两端分相位取分位数，再和全局 p2/p98 取并集 —— 只会让行程更满，
//      而分位数本身仍然挡得住野点
//   5) 标不满的维【按先验兜底】，而不是留 lo=hi=0。
//      v1 留 0 的后果是 RomMapper 走 clamp 透传分支，把带常量偏置的原始角
//      直接钳进目标行程 —— 那是 100% 触限，比不映射还糟
//   6) 运行时对【退化帧保持上一帧输出】，而不是把 PIP=0 这种假值发下去
//
// 【依赖】只依赖 hand/HandModel.hpp 的 jointLimits()。不含 Qt、不含关联器，
// 可以单独编译进测试程序跑（tests/test_rom_calibration.cpp）。
// =============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "hand/HandModel.hpp"

namespace mocap {
namespace hm20 {
namespace rom {

inline constexpr int kDof = 16;
inline constexpr int kFingers = 5;

// 每一维属于哪根手指。布局跟 HandModel 一致：
// 拇指占 0..3，食/中/无/小各占 3 个。
inline int dofFinger(int i) { return (i < 4) ? 0 : (1 + (i - 4) / 3); }

// 外展维：拇CMC展=1，四指 MCP展 = 5/8/11/14。
// 「张开→握拳」这个引导动作【不产生外展】，所以这几维既不参与方向判定，
// 也不参与达标计数 —— 把它们算进去等于凭空加严，v1 已经踩过一次。
inline bool dofIsAbduction(int i) {
    return i == 1 || i == 5 || i == 8 || i == 11 || i == 14;
}

// 屈曲维下标（11 个）
inline const std::array<int, 11>& flexDofs() {
    static const std::array<int, 11> f = {0, 2, 3, 4, 6, 7, 9, 10, 12, 13, 15};
    return f;
}

inline const char* dofName(int i) {
    static const char* n[kDof] = {
        "拇CMC屈", "拇CMC展", "拇MCP", "拇IP",
        "食MCP屈", "食MCP展", "食PIP",
        "中MCP屈", "中MCP展", "中PIP",
        "无MCP屈", "无MCP展", "无PIP",
        "小MCP屈", "小MCP展", "小PIP"};
    return (i >= 0 && i < kDof) ? n[i] : "?";
}

// ---------------------------------------------------------------------------
// 逐维的本帧状态。【这是 v1 缺的第一块信息】
// v1 只有 fingerValid（逐指一个 bool），而同一根手指里 MCP 和 PIP 的
// 新鲜程度可以完全不同 —— 腕部系失效时 PIP 是新算的、MCP 是保持的，
// 而 fingerValid 对这两种情况给的是同一个 true。
// ---------------------------------------------------------------------------
enum class DofState : uint8_t {
    Missing   = 0,   // 本帧压根没算这一维
    Held      = 1,   // 保持上一帧的值（腕部系失效 / 手指没解出来）
    Degenerate= 2,   // 算了，但两条骨轴退化成同一条 —— 值无意义（PIP≡0 那种）
    Predicted = 3,   // 算了，但用的是预测补出来的球
    Measured  = 4,   // 算了，几何/IK 实测
};

// 样本被拒的原因。【必须逐维分类计数】——「行程太短」只是结论，
// 而「因为八成的帧这一维在保持上一帧」和「因为骨轴退化」的修法完全不同。
enum class RejectCode : uint8_t {
    Accepted     = 0,
    NotFresh     = 1,   // Held / Missing
    AxisDegen    = 2,   // Degenerate
    NoClosure    = 3,   // 该指的 curl 参考这一帧算不出来
    Outlier      = 4,   // 解剖上不可能的值（|q| 过大）
    LowQuality   = 5,   // Predicted 且配置要求只收实测
    Count        = 6,
};

// ---------------------------------------------------------------------------
// 一帧的观测。由 solveJointAngles 的调试出参直接填，不需要额外算几何。
// ---------------------------------------------------------------------------
inline constexpr int kSignRefs = 3;

struct FrameObs {
    std::array<double, kDof>  q{};            // 未经平滑的原始角（弧度）
    std::array<uint8_t, kDof> state{};        // DofState
    // 逐指卷曲度参考：0=完全张开 1=握到底。保留给外推和旧调用方。
    std::array<double, kFingers>  curl{};
    std::array<uint8_t, kFingers> curlValid{};
    // 逐维、多候选方向参考。每个候选都统一约定：数值越大 = 越接近握拳。
    //   候选0：本指 curl；候选1：四指共同 curl；候选2：拇CMC屈专用参考。
    // 判方向时选“包含样本足够且 |秩相关| 最大”的那个候选，避免某一根指头
    // 的 curl 噪声把整维符号带反。旧调用方不填时自动回退到本指 curl。
    std::array<std::array<double, kDof>, kSignRefs>  signRef{};
    std::array<std::array<uint8_t, kDof>, kSignRefs> signRefValid{};
    int64_t frameTsNs = -1;
};

// ---------------------------------------------------------------------------
// 逐维标定结果。全部落进录制文件（块 31），离线能一眼看出每一维为什么不行。
// ---------------------------------------------------------------------------
enum class DofStatus : uint8_t {
    Ok           = 0,  // 样本够、行程够、方向确定
    PriorFilled  = 1,  // 行程不够，按先验兜底（能用，但行程利用率打折）
    SignUnknown  = 2,  // 行程够但所有候选参考都判不出方向——按 +1 处理并告警
    NoData       = 3,  // 样本太少，这一维【冻结在中立位】
    AxisDead     = 4,  // 绝大多数样本因骨轴退化被拒 —— 这是上游的结构问题
    Extrapolated = 5,  // 行程的一部分是按 curl 外推补的（见 Config::extrapolateByCurl）
};

struct DofResult {
    int      nSamples   = 0;                 // 被接受的样本数
    std::array<int, int(RejectCode::Count)> nReject{};   // 逐原因的拒收数
    int      nSeen      = 0;                 // 见到的总帧数（接受+拒收）

    int      sign       = 1;                 // +1 = 屈曲使 q 变大；-1 = 变小
    double   signDeltaRad = 0.0;             // 握拳端中位 - 张开端中位（原始 q）
    double   signCorr   = 0.0;               // 与最终选中参考量的秩相关，-1..1

    // 区间定义在带符号空间 v = sign*q 上，于是恒有 lo<hi 且屈曲→变大。
    double   lo = 0.0, hi = 0.0;
    double   rawLo = 0.0, rawHi = 0.0;       // 换算回原始 q 的区间，给人看
    double   coverage = 0.0;                 // (hi-lo)/解剖行程，封顶 1

    int      nOpen = 0, nClose = 0;          // 两端相位各自的样本数
    double   openMed = 0.0, closeMed = 0.0;  // 原始 q 的两端中位
    double   curlSpread = 0.0;               // 最终参考量的 p90-p10，<0.15 = 没真做动作

    // 方向是「判出来的」还是「默认给的 +1」。
    // 【必须分开】默认 +1 和判定为 +1 是两回事：前者意味着这一维的方向
    // 其实是未知的，输出可能仍然是反的，而 sign 字段本身看不出区别。
    bool     signResolved = false;

    // ---- 外推 ----
    double   measLo = 0.0, measHi = 0.0;   // 【纯实测】的区间，不含外推
    double   fitSlope = 0.0;               // v 对 curl 的斜率（rad / 单位 curl）
    double   fitR2 = 0.0;
    double   curlSeenLo = 0.0, curlSeenHi = 0.0;  // 该指全程 curl 的 p2/p98
    double   curlUsedLo = 0.0, curlUsedHi = 0.0;  // 被接受样本覆盖到的 curl 区间
    double   extrapLoRad = 0.0, extrapHiRad = 0.0; // 两端各外推了多少
    uint8_t  status = uint8_t(DofStatus::NoData);
};

struct CalibResult {
    std::array<DofResult, kDof> dof{};
    double coverageFlex = 0.0;      // 11 个屈曲维的平均覆盖度
    double coverageAbd  = 0.0;
    int    nOkFlex      = 0;        // 达标的屈曲维个数
    int    nPriorFlex   = 0;
    int    nDeadFlex    = 0;
    int    nSignFlipped = 0;        // 判出「方向是反的」的维数
    int    nFrames      = 0;        // observe() 被调用的帧数
    int    nFramesUsed  = 0;        // 至少接受了一维的帧数
    bool   ready        = false;
    double durationSec  = 0.0;
};

// ---------------------------------------------------------------------------
// 配置。默认值都写了「为什么是这个数」，改之前先看注释。
// ---------------------------------------------------------------------------
struct Config {
    // 达标判据。0.35rad≈20°，跟 v1 一致 —— 不动它，免得跟历史结论对不上。
    double minRangeRad      = 0.35;
    int    minSamplesPerDof = 30;
    int    minOkFlexDofs    = 8;      // 11 个屈曲维里至少 8 个

    // 方向判定
    bool   learnSign        = true;
    // curl 的 p90-p10 至少要拉开这么多，才认为「操作者真的做了张开→握拳」。
    // 0.15 是把 curl 定义成 0..1 之后取的：正常一次完整开合能拉开 0.35 以上，
    // 0.15 已经很松了，主要是挡住「手根本没动，量的全是噪声」这一类。
    double minCurlSpread    = 0.15;
    // 两端中位差至少 0.12rad(≈7°) 才敢定方向。低于它说明这一维在开合动作里
    // 本来就不怎么动（外展维就是这样），硬判方向只会判出噪声的符号。
    double minSignDeltaRad  = 0.12;
    // 多参考量选择后的秩相关下限。达到它就直接按相关符号定方向；
    // 达不到时才退回“两端中位差”。这样“中位差小但整体单调”的维也能纠正。
    double minSignCorr      = 0.35;

    // 相位切分：curl 最低的 openFrac 当「张开端」，最高的 closeFrac 当「握拳端」
    double openFrac         = 0.30;
    double closeFrac        = 0.30;
    // 端内分位。取 0.10/0.90 而不是 min/max：端内仍有野点。
    double endQuantile      = 0.10;
    // 全局分位，跟 v1 一致。最终区间取「相位区间 ∪ 全局区间」——
    // 并集只会让行程更满，而两者各自都是分位数，野点仍然进不来。
    double globalQuantileLo = 0.02;
    double globalQuantileHi = 0.98;

    // 采样门
    bool   acceptPredicted  = true;   // 收预测段（握拳时中远节球必然被挡）
    bool   acceptDegenerate = false;  // 【永远别打开】骨轴退化的值是恒零，不是噪声
    double outlierAbsRad    = 3.4;    // |q|>3.4rad(195°) 一定是解算炸了

    // ---- 按 curl 外推补齐行程 ----
    // 【为什么需要外推，光靠"多采样本"补不上】
    // 握到底那一段手背必然被四指挡住，腕部位姿失效，近节骨轴退化 —— 这不是
    // 采样不够，是那几帧的角度【物理上就没有】。合成实验里把退化帧如实拒掉
    // 之后覆盖度只有 63%：接受的样本只覆盖 curl∈[0,0.62]，剩下 38% 的行程
    // 在文件里根本不存在，再标十次也还是 63%。
    //
    // 但 curl（卷曲度）在那些帧里【是有的】—— 它只用球心间距算，不经过腕部系。
    // 于是可以拿"角度 vs curl"的线性关系，把可见段的斜率外推到 curl 的真实
    // 端点上去。这不是编数据：斜率是从几百个真实样本上拟合出来的，外推的只是
    // 最后那一小段，而且下面三道闸限着：
    //   · 拟合优度 R² 不够就不外推（关系本来就不线性时不硬来）
    //   · 可见段的 curl 跨度不够就不外推（外推距离比已知距离还长时不可信）
    //   · 外推量封顶（默认不超过实测行程的 60%）
    // 打上 Extrapolated 状态，录制文件里看得见哪几维是外推来的。
    bool   extrapolateByCurl = true;
    double extrapMinR2       = 0.55;
    double extrapMinCurlSpan = 0.25;   // 可见段 curl 至少要拉开这么多
    double extrapMaxFrac     = 0.60;   // 外推量 <= 实测行程 × 这个数

    // 兜底
    bool   fillFromPrior    = true;
    // 兜底行程 = 解剖行程 × 这个系数。0.6 是折中：给足行程让机械手动得开，
    // 又不至于把噪声放大成满行程。
    double priorSpanFrac    = 0.60;

    // 运行时
    // 退化帧保持上一帧输出。【默认开】把 PIP≡0 当真值发下去，
    // 机械手会在握拳时突然张开 —— 那是遥操作里最危险的一种输出。
    bool   holdOnDegenerate = true;

    size_t maxSamplesPerDof = 6000;   // 120fps 下 50 秒
};

// ---------------------------------------------------------------------------
namespace detail {

inline double quantileSorted(const std::vector<double>& v, double p) {
    if (v.empty()) return 0.0;
    if (v.size() == 1) return v[0];
    const double x = p * double(v.size() - 1);
    const size_t i = size_t(x);
    const size_t j = std::min(i + 1, v.size() - 1);
    const double t = x - double(i);
    return v[i] * (1.0 - t) + v[j] * t;
}

inline double medianOf(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return quantileSorted(v, 0.5);
}

// 秩相关（Spearman）。只作参考显示 —— 方向判定用两端中位差，
// 因为相关系数对「中间段占绝大多数样本」这种分布不敏感，而我们的动作
// 恰恰是两端停留短、中间过渡长。
inline double spearman(const std::vector<double>& a, const std::vector<double>& b) {
    const size_t n = a.size();
    if (n < 8 || b.size() != n) return 0.0;
    auto rank = [n](const std::vector<double>& x) {
        std::vector<size_t> idx(n);
        for (size_t i = 0; i < n; ++i) idx[i] = i;
        std::sort(idx.begin(), idx.end(),
                  [&x](size_t p, size_t q) { return x[p] < x[q]; });
        std::vector<double> r(n);
        size_t i = 0;
        while (i < n) {
            size_t j = i;
            while (j + 1 < n && x[idx[j + 1]] == x[idx[i]]) ++j;
            const double avg = 0.5 * double(i + j) + 1.0;
            for (size_t k = i; k <= j; ++k) r[idx[k]] = avg;
            i = j + 1;
        }
        return r;
    };
    const std::vector<double> ra = rank(a), rb = rank(b);
    double ma = 0, mb = 0;
    for (size_t i = 0; i < n; ++i) { ma += ra[i]; mb += rb[i]; }
    ma /= double(n); mb /= double(n);
    double sab = 0, saa = 0, sbb = 0;
    for (size_t i = 0; i < n; ++i) {
        const double da = ra[i] - ma, db = rb[i] - mb;
        sab += da * db; saa += da * da; sbb += db * db;
    }
    if (saa < 1e-12 || sbb < 1e-12) return 0.0;
    return sab / std::sqrt(saa * sbb);
}

}  // namespace detail

// ---------------------------------------------------------------------------
// ROM 标定器 v2
// ---------------------------------------------------------------------------
class Calibrator {
public:
    void configure(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }

    // keepPrevious=true 表示【补标】：保留上一轮的样本继续攒。
    // 【为什么要有补标】握拳时四指互相遮挡是物理事实，一次做不满很正常。
    // v1 只能推倒重来，于是用户反复重标反复不满。补标可以专门针对
    // 没标满的那几维再做一次动作，已经标好的维不受影响。
    void beginPass(bool keepPrevious = false) {
        if (!keepPrevious) {
            for (auto& v : sampQ_) v.clear();
            for (auto& v : sampC_) v.clear();
            for (auto& v : curlAll_) v.clear();
            for (auto& refs : sampSignRef_) for (auto& v : refs) v.clear();
            for (auto& flags : sampSignRefValid_) for (auto& v : flags) v.clear();
            for (auto& d : res_.dof) { d = DofResult{}; }
            res_ = CalibResult{};
            nFrames_ = 0; nFramesUsed_ = 0;
            prevReady_ = false;
        } else {
            prevReady_ = res_.ready;
            prevRes_   = res_;
        }
        learning_ = true;
        tStartNs_ = -1; tLastNs_ = -1;
    }
    void reset() { beginPass(false); learning_ = false; }

    bool learning() const { return learning_; }
    bool ready()    const { return res_.ready; }
    int  samples()  const { return nFrames_; }
    int  sampleCount(int i) const { return int(sampQ_[size_t(i)].size()); }

    // ------------------------------------------------------------------
    // 逐帧采样。【返回本帧逐维的拒收原因】—— 直接落进录制文件（块 32），
    // 于是「为什么这一维只攒到 40 个样本」在离线时是查得到的，
    // 而不是只能看到一个 40。
    // ------------------------------------------------------------------
    std::array<uint8_t, kDof> observe(const FrameObs& o) {
        std::array<uint8_t, kDof> why{};
        ++nFrames_;
        if (tStartNs_ < 0) tStartNs_ = o.frameTsNs;
        tLastNs_ = o.frameTsNs;
        bool used = false;
        // 【curl 要在这里全量攒下来，不受角度有效性影响】
        // 外推能成立就靠这一点：握到底那一段角度没有、curl 有。
        // 如果跟着角度一起被拒，外推就退化成"用可见段推可见段"，等于没做。
        for (int f = 0; f < kFingers; ++f)
            if (o.curlValid[size_t(f)] && curlAll_[size_t(f)].size() < cfg_.maxSamplesPerDof)
                curlAll_[size_t(f)].push_back(o.curl[size_t(f)]);
        for (int i = 0; i < kDof; ++i) {
            const size_t u = size_t(i);
            const int f = dofFinger(i);
            const auto st = DofState(o.state[u]);
            ++res_.dof[u].nSeen;

            // 至少有一个方向参考可用就允许采样。旧调用方不填 signRef，
            // 这里会回退到本指 curl，行为与之前一致。
            bool refValid = (o.curlValid[size_t(f)] != 0);
            for (int c = 0; c < kSignRefs; ++c)
                refValid = refValid || (o.signRefValid[size_t(c)][u] != 0);

            RejectCode rc = RejectCode::Accepted;
            if (st == DofState::Missing || st == DofState::Held)      rc = RejectCode::NotFresh;
            else if (st == DofState::Degenerate && !cfg_.acceptDegenerate) rc = RejectCode::AxisDegen;
            else if (st == DofState::Predicted && !cfg_.acceptPredicted)   rc = RejectCode::LowQuality;
            else if (!refValid)                                        rc = RejectCode::NoClosure;
            else if (!(std::fabs(o.q[u]) <= cfg_.outlierAbsRad))       rc = RejectCode::Outlier;

            why[u] = uint8_t(rc);
            ++res_.dof[u].nReject[size_t(rc)];
            if (rc != RejectCode::Accepted) continue;
            if (sampQ_[u].size() >= cfg_.maxSamplesPerDof) continue;
            sampQ_[u].push_back(o.q[u]);
            sampC_[u].push_back(o.curl[size_t(f)]);

            for (int c = 0; c < kSignRefs; ++c) {
                double v = o.signRef[size_t(c)][u];
                uint8_t ok = o.signRefValid[size_t(c)][u];
                // 候选0的兼容回退：调用方只填了 curl 时，直接拿它当候选0。
                if (c == 0 && !ok && o.curlValid[size_t(f)]) {
                    v = o.curl[size_t(f)];
                    ok = 1;
                }
                sampSignRef_[size_t(c)][u].push_back(v);
                sampSignRefValid_[size_t(c)][u].push_back(ok);
            }
            used = true;
        }
        if (used) ++nFramesUsed_;
        return why;
    }

    // ------------------------------------------------------------------
    // 收尾：定方向、定区间、算覆盖度、定状态。
    // ------------------------------------------------------------------
    const CalibResult& finish() {
        learning_ = false;
        const auto& lim = jointLimits();
        res_.nFrames     = nFrames_;
        res_.nFramesUsed = nFramesUsed_;
        res_.durationSec = (tStartNs_ > 0 && tLastNs_ > tStartNs_)
                         ? double(tLastNs_ - tStartNs_) / 1e9 : 0.0;

        int okFlex = 0, priorFlex = 0, deadFlex = 0, flipped = 0;
        double covFlexSum = 0.0; int covFlexN = 0;
        double covAbdSum  = 0.0; int covAbdN  = 0;

        // 多候选参考量统一判方向。放在逐维区间之前，后面只消费 sign。
        resolveSigns();

        for (int i = 0; i < kDof; ++i) {
            const size_t u = size_t(i);
            DofResult& d = res_.dof[u];
            d.nSamples = int(sampQ_[u].size());
            const double full = lim[u].hi - lim[u].lo;

            // ---- 样本太少 ----
            if (d.nSamples < cfg_.minSamplesPerDof) {
                // 【区分「没数据」和「轴死了」】前者是操作者没做到位，
                // 重做一次就行；后者是上游几何问题，重做一万次也没用。
                // 拒收原因的分布是唯一能分开这两种的东西。
                const int degen = d.nReject[size_t(RejectCode::AxisDegen)];
                d.status = uint8_t((degen > d.nSeen / 2 && d.nSeen > 0)
                                   ? DofStatus::AxisDead : DofStatus::NoData);
                d.lo = d.hi = d.rawLo = d.rawHi = 0.0;
                d.coverage = 0.0;
                if (!dofIsAbduction(i)) {
                    ++deadFlex;
                    // 【覆盖度分母要算上它】否则一维彻底死掉之后反而从平均里
                    // 消失，覆盖度看起来还涨了 —— 那是最容易骗过人的一种指标。
                    ++covFlexN;
                } else ++covAbdN;
                continue;
            }

            // ---- 方向 ----
            // 已在 resolveSigns() 中统一完成：逐维多参考量选择。
            if (d.sign < 0) ++flipped;

            // ---- 区间（带符号空间）----
            std::vector<double> v(sampQ_[u].size());
            for (size_t k = 0; k < v.size(); ++k) v[k] = double(d.sign) * sampQ_[u][k];
            std::vector<double> vs = v;
            std::sort(vs.begin(), vs.end());
            double lo = detail::quantileSorted(vs, cfg_.globalQuantileLo);
            double hi = detail::quantileSorted(vs, cfg_.globalQuantileHi);

            // 相位区间：只对屈曲维做（外展维在这个动作里没有相位结构）
            if (!dofIsAbduction(i) && d.nOpen >= 8 && d.nClose >= 8) {
                std::vector<double> vOpen, vClose;
                vOpen.reserve(size_t(d.nOpen)); vClose.reserve(size_t(d.nClose));
                splitByPhase(i, d.sign, vOpen, vClose);
                if (vOpen.size() >= 8 && vClose.size() >= 8) {
                    std::sort(vOpen.begin(), vOpen.end());
                    std::sort(vClose.begin(), vClose.end());
                    const double pLo = detail::quantileSorted(vOpen,  cfg_.endQuantile);
                    const double pHi = detail::quantileSorted(vClose, 1.0 - cfg_.endQuantile);
                    // 【取并集】相位分位数抓的是两端真正停住的地方，
                    // 全局分位数抓的是整段的包络。谁大听谁的，行程只会更满。
                    if (pHi > pLo) { lo = std::min(lo, pLo); hi = std::max(hi, pHi); }
                }
            }
            if (hi < lo) std::swap(lo, hi);
            d.measLo = lo; d.measHi = hi;

            // ---- 按 curl 外推，把被遮挡吃掉的那一段行程补回来 ----
            bool extrapolated = false;
            if (cfg_.extrapolateByCurl && !dofIsAbduction(i))
                extrapolated = extrapolate(i, d, lo, hi);
            d.lo = lo; d.hi = hi;

            // ---- 兜底 ----
            const double span = hi - lo;
            const bool okSpan = (span >= cfg_.minRangeRad);
            if (okSpan) {
                d.status = uint8_t(!d.signResolved ? DofStatus::SignUnknown
                                 : extrapolated   ? DofStatus::Extrapolated
                                                  : DofStatus::Ok);
            } else if (cfg_.fillFromPrior && full > 1e-6) {
                // 【为什么必须兜底而不是留 0】留 0 会让 RomMapper 走透传+钳位，
                // 而原始角带一个未知的常量零位偏置 —— 透传的结果是 100% 触限，
                // 输出被压成一条贴着限位的直线。兜底至少保住单调性和方向。
                const double mid = 0.5 * (lo + hi);
                const double halfSpan = 0.5 * std::max(span, cfg_.priorSpanFrac * full);
                d.lo = mid - halfSpan;
                d.hi = mid + halfSpan;
                d.status = uint8_t(DofStatus::PriorFilled);
            } else {
                d.status = uint8_t(DofStatus::NoData);
            }

            d.rawLo = (d.sign > 0) ? d.lo : -d.hi;
            d.rawHi = (d.sign > 0) ? d.hi : -d.lo;
            d.coverage = (full > 1e-6) ? std::min(1.0, (d.hi - d.lo) / full) : 0.0;

            if (dofIsAbduction(i)) { covAbdSum += d.coverage; ++covAbdN; }
            else {
                covFlexSum += d.coverage; ++covFlexN;
                // 【外推维算达标】它的斜率是几百个真实样本拟合出来的，
                // 只有末端那一小段是推出来的，而且有 R²/跨度/封顶三道闸。
                // 不算它的话，"握拳时手背被挡"这个物理事实会永远让标定失败。
                if (d.status == uint8_t(DofStatus::Ok) ||
                    d.status == uint8_t(DofStatus::Extrapolated)) ++okFlex;
                else if (d.status == uint8_t(DofStatus::PriorFilled)) ++priorFlex;
            }
        }

        res_.coverageFlex = covFlexN ? covFlexSum / double(covFlexN) : 0.0;
        res_.coverageAbd  = covAbdN  ? covAbdSum  / double(covAbdN)  : 0.0;
        res_.nOkFlex      = okFlex;
        res_.nPriorFlex   = priorFlex;
        res_.nDeadFlex    = deadFlex;
        res_.nSignFlipped = flipped;
        res_.ready        = (okFlex >= cfg_.minOkFlexDofs);

        // 【补标的合并】上一轮已经标好的维，这一轮如果反而更差就保留旧的。
        // 用户的补标动作通常只针对某几根手指，别让「这次没顾上的那几维」
        // 把上一次的成果冲掉。
        if (prevReady_) mergeWithPrevious();
        return res_;
    }

    const CalibResult& result() const { return res_; }

    // ---- 映射用的取值口。i 越界或未标定时给安全值 ----
    double lo(int i) const { return res_.dof[size_t(i)].lo; }
    double hi(int i) const { return res_.dof[size_t(i)].hi; }
    int    sign(int i) const { return res_.dof[size_t(i)].sign; }
    DofStatus status(int i) const { return DofStatus(res_.dof[size_t(i)].status); }

    // 兼容 v1 的两个口子，UI 和 stateJson 还在用
    double coverage() const { return res_.coverageFlex; }
    double abductionCoverage() const { return res_.coverageAbd; }

    // ------------------------------------------------------------------
    // 人话报告。UI 的完成弹窗直接显示这个 —— v1 只说「行程太小，标定未生效」，
    // 而用户看到这句话之后【不知道该改哪一根手指的动作】，只能整体重做。
    // ------------------------------------------------------------------
    std::string humanReport() const {
        std::string s;
        char buf[256];
        int nSignUnknown = 0;
        for (int i : flexDofs())
            if (DofStatus(res_.dof[size_t(i)].status) == DofStatus::SignUnknown)
                ++nSignUnknown;
        std::snprintf(buf, sizeof(buf),
            "屈曲行程覆盖 %d%%（达标 %d/11，兜底 %d，失效 %d，方向不明 %d），外展覆盖 %d%%\n"
            "采样 %d 帧，其中 %d 帧有效，历时 %.1f 秒\n",
            int(res_.coverageFlex * 100 + 0.5), res_.nOkFlex, res_.nPriorFlex,
            res_.nDeadFlex, nSignUnknown, int(res_.coverageAbd * 100 + 0.5),
            res_.nFrames, res_.nFramesUsed, res_.durationSec);
        s += buf;
        if (res_.nSignFlipped > 0) {
            std::snprintf(buf, sizeof(buf),
                "已自动纠正 %d 个方向相反的维（握拳时读数变小的那些）\n",
                res_.nSignFlipped);
            s += buf;
        }
        s += "\n逐维明细（问题维优先）：\n";
        // 先列有问题的
        for (int pass = 0; pass < 2; ++pass) {
            for (int i = 0; i < kDof; ++i) {
                const DofResult& d = res_.dof[size_t(i)];
                // 【外推不算"有问题"】它的斜率是几百个真实样本拟合出来的，
                // 只有末端一小段是推的，属于正常可用。排到前面会让用户以为
                // 出了 11 个问题，反而看不见真正该管的那两三个。
                const bool bad = (d.status != uint8_t(DofStatus::Ok) &&
                                  d.status != uint8_t(DofStatus::Extrapolated));
                if ((pass == 0) != bad) continue;
                const char* st = "正常";
                switch (DofStatus(d.status)) {
                    case DofStatus::Ok:          st = "正常"; break;
                    case DofStatus::PriorFilled: st = "行程不足-已兜底"; break;
                    case DofStatus::SignUnknown: st = "方向不明"; break;
                    case DofStatus::NoData:      st = "样本不足"; break;
                    case DofStatus::AxisDead:    st = "骨轴退化(上游问题)"; break;
                    case DofStatus::Extrapolated:st = "含外推"; break;
                }
                std::snprintf(buf, sizeof(buf),
                    "  %-8s %3d%%  样本%5d/%5d  方向%+d  区间[%6.1f,%6.1f]°  %s\n",
                    dofName(i), int(d.coverage * 100 + 0.5), d.nSamples, d.nSeen,
                    d.sign, d.rawLo * 57.2957795, d.rawHi * 57.2957795, st);
                s += buf;
            }
        }
        s += mainRejectHint();
        return s;
    }

    // 把最主要的拒收原因翻译成「下一步该做什么」。
    std::string mainRejectHint() const {
        int tot[int(RejectCode::Count)] = {0};
        for (int k : flexDofs())
            for (int c = 0; c < int(RejectCode::Count); ++c)
                tot[c] += res_.dof[size_t(k)].nReject[size_t(c)];
        int worst = 0, worstN = 0;
        for (int c = 1; c < int(RejectCode::Count); ++c)
            if (tot[c] > worstN) { worstN = tot[c]; worst = c; }
        int seen = 0;
        for (int c = 0; c < int(RejectCode::Count); ++c) seen += tot[c];
        if (seen == 0 || worstN * 4 < seen) return {};
        char buf[320];
        const char* txt = "";
        switch (RejectCode(worst)) {
            case RejectCode::NotFresh:
                txt = "多数帧这几维在【保持上一帧】——手背被四指挡住导致腕部位姿失效。\n"
                      "  对策：握拳时把手背朝向相机多一点，或降低手背点的丢失率。"; break;
            case RejectCode::AxisDegen:
                txt = "多数帧【近节骨轴退化】——腕部位姿失效时近节轴退回 pp->mp，\n"
                      "  与中节轴重合，PIP 恒等于 0。这是上游几何问题，不是操作问题。\n"
                      "  对策：见 RomCalibration.hpp 顶部「假设三」，需打开 anchor 保持开关。"; break;
            case RejectCode::NoClosure:
                txt = "多数帧【卷曲度参考算不出来】——该指三个球有缺失。\n"
                      "  对策：调整相机布局或降低遮挡。"; break;
            case RejectCode::Outlier:
                txt = "多数帧的角度是解剖上不可能的值——解算本身炸了，先查标签和手性。"; break;
            case RejectCode::LowQuality:
                txt = "多数帧只有预测值而配置要求实测。对策：打开 acceptPredicted。"; break;
            default: return {};
        }
        std::snprintf(buf, sizeof(buf), "\n主要拒收原因（%d%%）：%s\n",
                      int(100.0 * worstN / std::max(1, seen)), txt);
        return buf;
    }

    // 全量 JSON，落进录制头和 stateJson。离线脚本直接读这个。
    // 【为什么除了二进制块还要一份 JSON】二进制块是逐帧的、给脚本用的；
    // 这一份是人直接能看的，而且 ParamDelta 那条路已经通了，
    // 不用为它单独加解析代码 —— 排查时少一个环节就少一个出错的地方。
    std::string toJson() const {
        auto num = [](double v) {
            char b[40];
            if (!std::isfinite(v)) return std::string("null");
            std::snprintf(b, sizeof(b), "%.6g", v);
            return std::string(b);
        };
        std::string j = "{\"ready\":";
        j += res_.ready ? "true" : "false";
        j += ",\"coverageFlex\":" + num(res_.coverageFlex);
        j += ",\"coverageAbd\":"  + num(res_.coverageAbd);
        j += ",\"nOkFlex\":"      + std::to_string(res_.nOkFlex);
        j += ",\"nPriorFlex\":"   + std::to_string(res_.nPriorFlex);
        j += ",\"nDeadFlex\":"    + std::to_string(res_.nDeadFlex);
        j += ",\"nSignFlipped\":" + std::to_string(res_.nSignFlipped);
        j += ",\"nFrames\":"      + std::to_string(res_.nFrames);
        j += ",\"nFramesUsed\":"  + std::to_string(res_.nFramesUsed);
        j += ",\"durationSec\":"  + num(res_.durationSec);
        j += ",\"dof\":[";
        for (int i = 0; i < kDof; ++i) {
            const DofResult& d = res_.dof[size_t(i)];
            if (i) j += ",";
            j += "{\"name\":\"" + std::string(dofName(i)) + "\"";
            j += ",\"status\":"   + std::to_string(int(d.status));
            j += ",\"sign\":"     + std::to_string(d.sign);
            j += ",\"signOk\":"   + std::string(d.signResolved ? "true" : "false");
            j += ",\"lo\":"       + num(d.lo);
            j += ",\"hi\":"       + num(d.hi);
            j += ",\"rawLoDeg\":" + num(d.rawLo * 57.2957795130823);
            j += ",\"rawHiDeg\":" + num(d.rawHi * 57.2957795130823);
            j += ",\"coverage\":" + num(d.coverage);
            j += ",\"n\":"        + std::to_string(d.nSamples);
            j += ",\"nSeen\":"    + std::to_string(d.nSeen);
            j += ",\"nOpen\":"    + std::to_string(d.nOpen);
            j += ",\"nClose\":"   + std::to_string(d.nClose);
            j += ",\"openMedDeg\":"  + num(d.openMed  * 57.2957795130823);
            j += ",\"closeMedDeg\":" + num(d.closeMed * 57.2957795130823);
            j += ",\"curlSpread\":"  + num(d.curlSpread);
            j += ",\"reject\":[";
            for (int c = 0; c < int(RejectCode::Count); ++c) {
                if (c) j += ",";
                j += std::to_string(d.nReject[size_t(c)]);
            }
            j += "]}";
        }
        j += "]}";
        return j;
    }

private:
    struct SignRefEvidence {
        bool valid = false;
        int candidate = -1;
        double corr = 0.0;
        double delta = 0.0;
        double spread = 0.0;
        double score = 0.0;
        int nOpen = 0, nClose = 0;
        double openMed = 0.0, closeMed = 0.0;
    };

    bool evalSignRef(int i, int candidate, SignRefEvidence& out) const {
        const size_t u = size_t(i), c = size_t(candidate);
        const auto& qAll = sampQ_[u];
        const auto& rAll = sampSignRef_[c][u];
        const auto& okAll = sampSignRefValid_[c][u];
        if (qAll.size() < size_t(cfg_.minSamplesPerDof) ||
            rAll.size() != qAll.size() || okAll.size() != qAll.size())
            return false;
        std::vector<double> q, r;
        q.reserve(qAll.size()); r.reserve(qAll.size());
        for (size_t k = 0; k < qAll.size(); ++k) {
            if (!okAll[k] || !std::isfinite(rAll[k])) continue;
            q.push_back(qAll[k]); r.push_back(rAll[k]);
        }
        if (q.size() < size_t(cfg_.minSamplesPerDof)) return false;
        std::vector<double> rs = r;
        std::sort(rs.begin(), rs.end());
        const double r10 = detail::quantileSorted(rs, 0.10);
        const double r90 = detail::quantileSorted(rs, 0.90);
        const double spread = r90 - r10;
        if (spread < cfg_.minCurlSpread) return false;
        const double rOpenTh  = detail::quantileSorted(rs, cfg_.openFrac);
        const double rCloseTh = detail::quantileSorted(rs, 1.0 - cfg_.closeFrac);
        std::vector<double> qOpen, qClose;
        for (size_t k = 0; k < q.size(); ++k) {
            if (r[k] <= rOpenTh)  qOpen.push_back(q[k]);
            if (r[k] >= rCloseTh) qClose.push_back(q[k]);
        }
        if (qOpen.size() < 8 || qClose.size() < 8) return false;
        const double openMed  = detail::medianOf(qOpen);
        const double closeMed = detail::medianOf(qClose);
        const double corr = detail::spearman(q, r);
        const double nW = std::min(1.0, double(q.size()) / 90.0);
        const double sW = std::min(1.0, spread / 0.35);
        out.valid = true;
        out.candidate = candidate;
        out.corr = corr;
        out.delta = closeMed - openMed;
        out.spread = spread;
        out.score = std::abs(corr) * std::sqrt(std::max(0.0, nW * sW));
        out.nOpen = int(qOpen.size());
        out.nClose = int(qClose.size());
        out.openMed = openMed;
        out.closeMed = closeMed;
        return true;
    }

    void resolveSigns() {
        for (auto& d : res_.dof) {
            d.sign = 1;
            d.signResolved = false;
            d.signDeltaRad = 0.0;
            d.signCorr = 0.0;
            d.nOpen = d.nClose = 0;
            d.openMed = d.closeMed = 0.0;
            d.curlSpread = 0.0;
        }
        if (!cfg_.learnSign) {
            for (auto& d : res_.dof) d.signResolved = true;
            return;
        }
        for (int i = 0; i < kDof; ++i) {
            if (dofIsAbduction(i)) {
                res_.dof[size_t(i)].signResolved = true;
                continue;
            }
            SignRefEvidence best;
            for (int c = 0; c < kSignRefs; ++c) {
                SignRefEvidence e;
                if (!evalSignRef(i, c, e)) continue;
                if (!best.valid || e.score > best.score) best = e;
            }
            DofResult& d = res_.dof[size_t(i)];
            if (!best.valid) continue;
            d.signCorr = best.corr;
            d.signDeltaRad = best.delta;
            d.curlSpread = best.spread;
            d.nOpen = best.nOpen;
            d.nClose = best.nClose;
            d.openMed = best.openMed;
            d.closeMed = best.closeMed;
            if (std::abs(best.corr) >= cfg_.minSignCorr) {
                d.sign = (best.corr >= 0.0) ? 1 : -1;
                d.signResolved = true;
            } else if (std::abs(best.delta) >= cfg_.minSignDeltaRad) {
                d.sign = (best.delta >= 0.0) ? 1 : -1;
                d.signResolved = true;
            }
        }
    }

    // ------------------------------------------------------------------
    // 按 curl 外推。返回 true 表示真的动了区间。
    //
    // 做法：在【被接受的样本】上做 v = a + b·curl 的最小二乘，然后把
    // 这条线延到【全程 curl 的真实端点】上。全程 curl 来自所有帧（包括角度
    // 被拒的那些）—— 这正是外推能成立的原因：curl 只用球心间距算，
    // 腕部位姿失效影响不到它，所以握到底那一段 curl 是有的、角度没有。
    // ------------------------------------------------------------------
    bool extrapolate(int i, DofResult& d, double& lo, double& hi) {
        const size_t u = size_t(i);
        const int f = dofFinger(i);
        const auto& cAll = curlAll_[size_t(f)];
        if (cAll.size() < 30) return false;

        std::vector<double> cs = cAll;
        std::sort(cs.begin(), cs.end());
        d.curlSeenLo = detail::quantileSorted(cs, 0.02);
        d.curlSeenHi = detail::quantileSorted(cs, 0.98);

        std::vector<double> cu = sampC_[u];
        if (cu.size() < 30) return false;
        std::sort(cu.begin(), cu.end());
        d.curlUsedLo = detail::quantileSorted(cu, 0.02);
        d.curlUsedHi = detail::quantileSorted(cu, 0.98);
        const double usedSpan = d.curlUsedHi - d.curlUsedLo;
        if (usedSpan < cfg_.extrapMinCurlSpan) return false;

        // 最小二乘 v = a + b·c
        const size_t n = sampQ_[u].size();
        double sc = 0, sv = 0;
        for (size_t k = 0; k < n; ++k) {
            sc += sampC_[u][k];
            sv += double(d.sign) * sampQ_[u][k];
        }
        const double mc = sc / double(n), mv = sv / double(n);
        double scc = 0, scv = 0, svv = 0;
        for (size_t k = 0; k < n; ++k) {
            const double dc = sampC_[u][k] - mc;
            const double dv = double(d.sign) * sampQ_[u][k] - mv;
            scc += dc * dc; scv += dc * dv; svv += dv * dv;
        }
        if (scc < 1e-9 || svv < 1e-12) return false;
        const double b = scv / scc;
        const double a = mv - b * mc;
        d.fitSlope = b;
        d.fitR2 = (scv * scv) / (scc * svv);
        if (d.fitR2 < cfg_.extrapMinR2) return false;
        // 斜率方向必须跟已经判出来的 sign 一致（v 随 curl 增大）。
        // 不一致说明拟合和相位判定打架，这时候外推是危险的。
        if (b <= 0.0) return false;

        const double measSpan = hi - lo;
        if (measSpan < 1e-3) return false;
        const double cap = cfg_.extrapMaxFrac * measSpan;

        double addHi = 0.0, addLo = 0.0;
        if (d.curlSeenHi > d.curlUsedHi) {
            addHi = std::min(cap, b * (d.curlSeenHi - d.curlUsedHi));
            // 别推到解剖上不可能的地方
            const auto& lim = jointLimits();
            const double limHi = (d.sign > 0) ? lim[u].hi : -lim[u].lo;
            if (hi + addHi > limHi) addHi = std::max(0.0, limHi - hi);
        }
        if (d.curlUsedLo > d.curlSeenLo) {
            addLo = std::min(cap, b * (d.curlUsedLo - d.curlSeenLo));
            const auto& lim = jointLimits();
            const double limLo = (d.sign > 0) ? lim[u].lo : -lim[u].hi;
            if (lo - addLo < limLo) addLo = std::max(0.0, lo - limLo);
        }
        (void)a;
        if (addHi <= 1e-4 && addLo <= 1e-4) return false;
        hi += addHi; lo -= addLo;
        d.extrapHiRad = addHi; d.extrapLoRad = addLo;
        return true;
    }

    void splitByPhase(int i, int sgn, std::vector<double>& vOpen,
                      std::vector<double>& vClose) const {
        const size_t u = size_t(i);
        std::vector<double> cs = sampC_[u];
        if (cs.size() < 16) return;
        std::sort(cs.begin(), cs.end());
        const double cOpenTh  = detail::quantileSorted(cs, cfg_.openFrac);
        const double cCloseTh = detail::quantileSorted(cs, 1.0 - cfg_.closeFrac);
        for (size_t k = 0; k < sampQ_[u].size(); ++k) {
            const double v = double(sgn) * sampQ_[u][k];
            if (sampC_[u][k] <= cOpenTh)  vOpen.push_back(v);
            if (sampC_[u][k] >= cCloseTh) vClose.push_back(v);
        }
    }

    // 补标合并：逐维取「更好的那一份」。
    void mergeWithPrevious() {
        for (int i = 0; i < kDof; ++i) {
            const size_t u = size_t(i);
            const DofResult& oldD = prevRes_.dof[u];
            DofResult& newD = res_.dof[u];
            const bool oldOk = (oldD.status == uint8_t(DofStatus::Ok));
            const bool newOk = (newD.status == uint8_t(DofStatus::Ok));
            if (oldOk && !newOk) { newD = oldD; continue; }
            if (oldOk && newOk && oldD.sign == newD.sign) {
                // 两轮都好：取并集，行程只会更满
                newD.lo = std::min(newD.lo, oldD.lo);
                newD.hi = std::max(newD.hi, oldD.hi);
                newD.rawLo = (newD.sign > 0) ? newD.lo : -newD.hi;
                newD.rawHi = (newD.sign > 0) ? newD.hi : -newD.lo;
                const auto& lim = jointLimits();
                const double full = lim[u].hi - lim[u].lo;
                newD.coverage = (full > 1e-6)
                              ? std::min(1.0, (newD.hi - newD.lo) / full) : 0.0;
            }
        }
        // 合并后重算汇总
        const auto& lim = jointLimits();
        double cf = 0; int nf = 0; int ok = 0;
        for (int i : flexDofs()) {
            const DofResult& d = res_.dof[size_t(i)];
            const double full = lim[size_t(i)].hi - lim[size_t(i)].lo;
            (void)full;
            cf += d.coverage; ++nf;
            if (d.status == uint8_t(DofStatus::Ok)) ++ok;
        }
        res_.coverageFlex = nf ? cf / double(nf) : 0.0;
        res_.nOkFlex = ok;
        res_.ready = (ok >= cfg_.minOkFlexDofs);
    }

    Config cfg_{};
    std::array<std::vector<double>, kDof> sampQ_{};   // 原始角
    std::array<std::vector<double>, kDof> sampC_{};   // 对应帧的该指 curl
    // 逐维、多候选方向参考（与 sampQ_ 同步；每个候选单独记录 valid）。
    std::array<std::array<std::vector<double>, kDof>, kSignRefs> sampSignRef_{};
    std::array<std::array<std::vector<uint8_t>, kDof>, kSignRefs> sampSignRefValid_{};
    // 逐指 curl 的【全量】记录（含角度被拒的帧）。外推的依据。
    std::array<std::vector<double>, kFingers> curlAll_{};
    CalibResult res_{};
    CalibResult prevRes_{};
    bool   prevReady_ = false;
    bool   learning_ = false;
    int    nFrames_ = 0, nFramesUsed_ = 0;
    int64_t tStartNs_ = -1, tLastNs_ = -1;
};

// ---------------------------------------------------------------------------
// 映射器 v2
//
// 相对 v1 的三处改动，全部在这一层，不碰解算：
//   ① 带符号：u = (sign*q - lo)/(hi - lo)，于是「屈曲 -> 输出变大」恒成立
//   ② 退化帧保持：这一维本帧是 Held/Degenerate 时输出上一帧的值，
//      而不是把 PIP≡0 这种假值映射完发下去
//   ③ 死维冻结在中立位，而不是钳到行程端点
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// 映射器逐维逐帧的内部量。【为什么必须由 apply() 自己填，不能离线重算】
// 重算需要 sign / lo / hi / tLo / tHi 五个量，【以及下面那几个 clamp 的确切
// 顺序和边界条件】。离线复刻一遍，复刻得对不对本身又要验证 —— 而排查
// "输出为什么不动"时最不该再引入的就是这种不确定性。
//
// 这个结构直接对应录制块 33（RomMapDbgRec），字段一一对得上。
// dbg 传 nullptr 时一条多余指令都不执行，正常运行零开销。
// ---------------------------------------------------------------------------
struct MapDebug {
    // 本帧这一维走了哪条分支。【"输出是一个不动的数"有四种成因，
    // 它们的输出完全一样，而修法毫无共同点】
    enum Branch : uint8_t {
        PassClamp = 0,   // 未标定：out = clamp(q, tLo, tHi)
        DeadDof   = 1,   // NoData/AxisDead -> 永久钉在中立位
        Held      = 2,   // dofState 不新鲜 -> 永久保持上一帧
        SpanDead  = 3,   // hi-lo≈0 -> 中立位
        Mapped    = 4,   // 正常映射（但 t 仍可能恒被钳到 0 或 1）
        GroupCoupled = 5, // 本维方向证据弱，跟随同组可靠指的归一化弯曲位置
    };
    std::array<uint8_t, kDof> branch{};
    std::array<double,  kDof> vSigned{};   // sign*q
    std::array<double,  kDof> uNorm{};     // (v-lo)/span，【钳位之前】
    std::array<double,  kDof> tgtLo{}, tgtHi{};
    std::array<uint8_t, kDof> custom{};
    std::array<uint8_t, kDof> clampLo{}, clampHi{};
    std::array<uint8_t, kDof> status{};
    std::array<int8_t,  kDof> sign{};
    // 连续走 Held 分支的帧数 / 输出连续没变的帧数。
    // 【两个分开记】输出不变不一定是 Held 造成的：DeadDof/SpanDead 的
    // neutral 是常数，Mapped 但 t 恒被钳到同一端也是常数。
    // 两个数一起看才能定位到具体是哪一条。
    std::array<int32_t, kDof> heldRun{}, frozenRun{};
    int nHeld = 0, nFrozen = 0, nClamped = 0, nNeutral = 0;
    int maxFrozenRun = 0, maxFrozenIdx = -1;
    bool hasLast = false;
    bool justReset = false;
};

class Mapper {
public:
    void setTargetRange(int joint, double lo, double hi) {
        tLo_[size_t(joint)] = lo; tHi_[size_t(joint)] = hi; custom_[size_t(joint)] = true;
    }
    // 【reset 要留痕】区间换了之后保持器里的旧值必须清掉，而清掉的那一帧
    // 输出会突变一下 —— 事后看曲线会以为是解算跳了。记一个标志，
    // 落进块 33 的 mapperJustReset，一眼能排除掉这一种。
    void reset() {
        has_ = false; justReset_ = true; heldRun_ = {}; frozenRun_ = {};
        hasLastMcpT_ = false; lastMcpT_ = 0.5;
        hasBendRef_ = false; lastBendRef_ = 0.0; hasMonoOut_ = {};
    }

    // state 可以传 nullptr（不做保持），行为退回「逐帧无状态映射」。
    // dbg 传 nullptr 时行为跟以前【逐位一致】，不额外算任何东西。
    std::array<double, kDof> apply(const std::array<double, kDof>& q,
                                   const Calibrator& rom,
                                   const std::array<uint8_t, kDof>* state = nullptr,
                                   MapDebug* dbg = nullptr,
                                   double bendRef = -1.0) {
        const auto& lim = jointLimits();
        std::array<double, kDof> out{};
        std::array<double, kDof> tRawArr{}, tLoArr{}, tHiArr{};
        std::array<uint8_t, kDof> mapped{};
        if (dbg) {
            *dbg = MapDebug{};
            dbg->hasLast = has_;
            dbg->justReset = justReset_;
        }
        for (int i = 0; i < kDof; ++i) {
            const size_t u = size_t(i);
            const double tlo = custom_[u] ? tLo_[u] : lim[u].lo;
            const double thi = custom_[u] ? tHi_[u] : lim[u].hi;
            tLoArr[u] = tlo; tHiArr[u] = thi;
            if (dbg) {
                dbg->tgtLo[u] = tlo; dbg->tgtHi[u] = thi;
                dbg->custom[u] = uint8_t(custom_[u]);
                dbg->status[u] = uint8_t(rom.status(i));
                dbg->sign[u]   = int8_t(rom.sign(i));
            }

            if (!rom.ready()) {
                // 【没标定的行为跟 v1 一字不差】不要在这里"顺手改进"：
                // 未标定时的输出是很多历史结论的基线，动了就对不上了。
                out[u] = std::clamp(q[u], tlo, thi);
                if (dbg) {
                    dbg->branch[u]  = MapDebug::PassClamp;
                    // 【"标定之前就一直触限"的全部解释就在这两位上】
                    // 未标定时 q 带一个未知的常量零位偏置（hm20 腕部系的 +X
                    // 不是解剖中立位方向），偏置一大就整段贴在限位上 ——
                    // 而光看输出，贴限位和"手真的做到了极限"长得一模一样。
                    dbg->clampLo[u] = uint8_t(q[u] <= tlo);
                    dbg->clampHi[u] = uint8_t(q[u] >= thi);
                }
                continue;
            }

            const DofStatus st = rom.status(i);
            // 死维：冻结在中立位（0 弧度在目标行程里的位置），不是钳到端点。
            // 钳到端点对屈曲维就是「手指一直伸直」或「一直握死」，后者是危险动作。
            if (st == DofStatus::NoData || st == DofStatus::AxisDead) {
                out[u] = neutral(tlo, thi);
                if (dbg) dbg->branch[u] = MapDebug::DeadDof;
                continue;
            }

            // 本帧这一维不可信 -> 保持上一帧输出
            if (state && has_ && cfgHold_) {
                const DofState ds = DofState((*state)[u]);
                if (ds == DofState::Missing || ds == DofState::Held ||
                    ds == DofState::Degenerate) {
                    out[u] = last_[u];
                    if (dbg) dbg->branch[u] = MapDebug::Held;
                    continue;
                }
            }

            const double lo = rom.lo(i), hi = rom.hi(i);
            const double span = hi - lo;
            if (span < 1e-3) {
                out[u] = neutral(tlo, thi);
                if (dbg) dbg->branch[u] = MapDebug::SpanDead;
                continue;
            }
            const double v = double(rom.sign(i)) * q[u];
            const double tRaw = (v - lo) / span;
            tRawArr[u] = tRaw;
            mapped[u] = 1;
            const double t = std::clamp(tRaw, 0.0, 1.0);
            out[u] = std::clamp(tlo + t * (thi - tlo), tlo, thi);
            if (dbg) {
                dbg->branch[u]  = MapDebug::Mapped;
                dbg->vSigned[u] = v;
                // 【记钳位之前的 t】钳完之后它恒等于 0 或 1，而那个 0/1
                // 在输出里看不出是"到端点了"还是"信号本来就这么大"。
                dbg->uNorm[u]   = tRaw;
                dbg->clampLo[u] = uint8_t(tRaw <= 0.0);
                dbg->clampHi[u] = uint8_t(tRaw >= 1.0);
            }
        }

        // ---- 四指同组耦合：方向证据弱的指，跟随同组可靠指的弯曲位置 ----
        // 四指一起握拳/张开，同组可靠指的归一化位置比单指自身的噪声角稳。
        // 只在“本维自己的方向证据弱”时启用；证据强的指保持独立输出。
        if (rom.ready()) {
            auto reliable = [&](int j) {
                const size_t v = size_t(j);
                if (!mapped[v]) return false;
                const auto& dj = rom.result().dof[v];
                const auto sj = DofStatus(dj.status);
                if (sj != DofStatus::Ok && sj != DofStatus::Extrapolated) return false;
                if (!dj.signResolved || std::abs(dj.signCorr) < 0.35) return false;
                if (state) {
                    const auto ds = DofState((*state)[v]);
                    if (ds == DofState::Missing || ds == DofState::Held ||
                        ds == DofState::Degenerate) return false;
                }
                return true;
            };
            auto weak = [&](int j) {
                const auto& dj = rom.result().dof[size_t(j)];
                return !dj.signResolved || std::abs(dj.signCorr) < 0.35 ||
                       DofStatus(dj.status) == DofStatus::SignUnknown;
            };
            auto couple = [&](int i, const auto& group) {
                if (!weak(i)) return;
                std::vector<double> ref;
                for (int j : group) {
                    if (j == i || !reliable(j)) continue;
                    ref.push_back(std::clamp(tRawArr[size_t(j)], 0.0, 1.0));
                }
                if (ref.size() < 2) return;
                std::sort(ref.begin(), ref.end());
                const double t = detail::quantileSorted(ref, 0.5);
                const size_t u = size_t(i);
                out[u] = std::clamp(tLoArr[u] + t * (tHiArr[u] - tLoArr[u]),
                                    tLoArr[u], tHiArr[u]);
                if (dbg) {
                    dbg->branch[u] = MapDebug::GroupCoupled;
                    dbg->uNorm[u] = t;
                    dbg->vSigned[u] = t;
                    dbg->clampLo[u] = uint8_t(t <= 0.0);
                    dbg->clampHi[u] = uint8_t(t >= 1.0);
                }
            };
            static const std::array<int, 4> mcpGroup = {4, 7, 10, 13};
            static const std::array<int, 4> pipGroup = {6, 9, 12, 15};
            couple(0, mcpGroup);          // 拇CMC屈方向弱时先跟四指共同趋势
            for (int i : mcpGroup) couple(i, mcpGroup);
            for (int i : pipGroup) couple(i, pipGroup);

            // 拇CMC专项兜底：四指MCP正在整体开合时，拇CMC跟随其共同弯曲位置。
            // 四指静止时不接管，保留拇指独立输出。真机上拇CMC原始角在
            // 不同握拳循环里仍可能反，单靠ROM的固定sign救不了“本轮翻号”。
            std::vector<double> mcpVals;
            for (int j : mcpGroup)
                if (reliable(j)) mcpVals.push_back(std::clamp(tRawArr[size_t(j)], 0.0, 1.0));
            if (mcpVals.size() >= 2) {
                std::sort(mcpVals.begin(), mcpVals.end());
                const double mcpT = detail::quantileSorted(mcpVals, 0.5);
                if (hasLastMcpT_ && std::abs(mcpT - lastMcpT_) > 0.02) {
                    const size_t u = 0;
                    out[u] = std::clamp(tLoArr[u] + mcpT * (tHiArr[u] - tLoArr[u]),
                                        tLoArr[u], tHiArr[u]);
                    if (dbg) {
                        dbg->branch[u] = MapDebug::GroupCoupled;
                        dbg->uNorm[u] = mcpT;
                        dbg->vSigned[u] = mcpT;
                        dbg->clampLo[u] = uint8_t(mcpT <= 0.0);
                        dbg->clampHi[u] = uint8_t(mcpT >= 1.0);
                    }
                }
                lastMcpT_ = mcpT;
                hasLastMcpT_ = true;
            }
        }

        // ---- 握拳/张开单调趋势守卫 ----
        // 真机上有些手指的原始角会在握拳中后段“回缩”，固定 sign 和同组
        // 耦合都挡不住。这里直接按共同弯曲参考的方向做单向棘轮：
        //   bendRef 增加 -> 屈曲维只能增不能减
        //   bendRef 减少 -> 屈曲维只能减不能增
        // 平坦段（|d|<eps）不做限制，避免手静止时把微小动作也冻住。
        if (rom.ready() && bendRef >= 0.0 && std::isfinite(bendRef)) {
            constexpr double kDirEps = 0.002;
            int dir = 0;
            if (hasBendRef_) {
                const double dB = bendRef - lastBendRef_;
                if (dB > kDirEps) dir = 1;
                else if (dB < -kDirEps) dir = -1;
            }
            if (dir != 0 || !hasBendRef_) {
                for (int i : flexDofs()) {
                    const size_t u = size_t(i);
                    if (!hasMonoOut_[u]) {
                        lastMonoOut_[u] = out[u];
                        hasMonoOut_[u] = 1;
                        continue;
                    }
                    if (dir > 0)      out[u] = std::max(out[u], lastMonoOut_[u]);
                    else if (dir < 0) out[u] = std::min(out[u], lastMonoOut_[u]);
                    lastMonoOut_[u] = out[u];
                }
            } else {
                for (int i : flexDofs())
                    lastMonoOut_[size_t(i)] = out[size_t(i)];
            }
            lastBendRef_ = bendRef;
            hasBendRef_ = true;
        }

        if (dbg) fillRuns(*dbg, out);
        last_ = out; has_ = true; justReset_ = false;
        return out;
    }

    void setHoldOnDegenerate(bool on) { cfgHold_ = on; }
    bool holdOnDegenerate() const { return cfgHold_; }

private:
    // 连续保持 / 连续冻结的帧数。【计数器放在 Mapper 里而不是调用方】
    // 它要跟 last_ 同生共死：reset() 清掉 last_ 的同一刻这两个计数必须归零，
    // 否则"reset 之后又冻了多久"会跟 reset 之前的连起来算，直接看错。
    void fillRuns(MapDebug& d, const std::array<double, kDof>& out) {
        for (int i = 0; i < kDof; ++i) {
            const size_t u = size_t(i);
            if (d.branch[u] == MapDebug::Held) { ++heldRun_[u]; ++d.nHeld; }
            else heldRun_[u] = 0;
            if (has_ && std::fabs(out[u] - last_[u]) < 1e-6) ++frozenRun_[u];
            else frozenRun_[u] = 0;
            if (frozenRun_[u] > 0) ++d.nFrozen;
            if (d.clampLo[u] || d.clampHi[u]) ++d.nClamped;
            if (d.branch[u] == MapDebug::DeadDof || d.branch[u] == MapDebug::SpanDead)
                ++d.nNeutral;
            d.heldRun[u] = heldRun_[u];
            d.frozenRun[u] = frozenRun_[u];
            if (frozenRun_[u] > d.maxFrozenRun) {
                d.maxFrozenRun = frozenRun_[u]; d.maxFrozenIdx = i;
            }
        }
    }

    static double neutral(double tlo, double thi) {
        // 0 弧度落在目标行程里的位置。屈曲维 tlo<0<thi -> 略微伸直；
        // 外展维对称 -> 正中。两者都是解剖中立位，也是遥操作里最安全的姿态。
        if (thi - tlo < 1e-9) return tlo;
        return std::clamp(0.0, tlo, thi);
    }
    std::array<double, kDof> tLo_{}, tHi_{}, last_{};
    std::array<bool, kDof> custom_{};
    std::array<int32_t, kDof> heldRun_{}, frozenRun_{};
    bool has_ = false;
    bool justReset_ = false;
    bool cfgHold_ = true;
    bool hasLastMcpT_ = false;
    double lastMcpT_ = 0.5;
    // 单调趋势守卫状态
    bool hasBendRef_ = false;
    double lastBendRef_ = 0.0;
    std::array<double, kDof> lastMonoOut_{};
    std::array<uint8_t, kDof> hasMonoOut_{};
};

}  // namespace rom
}  // namespace hm20
}  // namespace mocap
