// =============================================================================
// Hm20AutoCalib.hpp —— 在线闭环自标定（取代旧的 Hm20AutoTemplate.hpp）
// =============================================================================
// 【它替代了什么】旧的 Hm20AutoTemplate 是"攒 12 帧就冻结，冻完不管"。那份
// 实现有几个致命的想当然，都在真机上会咬人：
//   · 打开面板立刻开始攒帧 —— 手可能还没进视野、还没摆正、还在快速移动。
//     标错的模板会被固化，之后每一帧都跟着错，而且没有任何机制发现。
//   · 冻结之后永不更新 —— 反光球滑一下、手套动一下就全乱，只能重启面板。
//   · anchor/骨长直接抄群体均值常量。而实测【IK 用群体均值参数比不开 IK 还差】
//     （骨轴中位 13.2° -> 17.6°，关掉才 13.2°）。也就是说旧版自举出来的模板
//     根本撑不起 IK，那条路走不通。
//
// 【现在的做法：分阶段 + 闭环 + 会自己发现自己错了】
//   Idle -> Warmup -> Collecting -> Locked -> (FittingParams) -> Ready
//                                     ^                            |
//                                     +------- 漂移检测 -----------+
//
// 每一阶段的准入条件都是【可解释的物理量】，不是拍脑袋的帧数：
//   Warmup     手在视野里、5 个手背点全被认领且概率够、手没在快速运动
//   Collecting 手背 5 点两两距离跨帧一致（刚体自检）、展布够大（不是一簇噪声）
//   Locked     手背模板锁定，开始解腕部位姿；同时持续做小增益 EMA 微调
//   Ready      anchor 由球面拟合解出、骨长由链上跨距解出，IK 可以放心开
//
// 【关键洞察：anchor 是可以自动解出来的】
// HandCalibration.hpp 里那句"两个方程三个未知数做不到自动标定"说的是【单帧】。
// 多姿势下不成立：近节球到该指根关节(MCP/CMC)的距离在关节转动时【恒定】
// （只随皮肤滑移微变），所以近节球在腕部系里的轨迹落在一个以 anchor 为球心的
// 球面上 —— 这是标准的球面拟合，4 个以上非退化姿势就能解。这跟动捕里用
// 功能法解髋关节中心是同一件事。实测收敛后 anchor 误差 2~4mm，而群体均值是
// 10~20mm，正是 IK 好坏的分水岭。
//
// 【骨长】伸直时相邻两球连线纯轴向（径向偏置同方位角、互相抵消），于是
//     |m1-m0| = uRatio0*L0 + (1-uRatio1)*L1
//     |m2-m1| = uRatio1*L1 + (1-uRatio2)*L2
// 两个方程三个未知数，但只要沿用群体的【比例】、只解一个逐指缩放 k，就闭合了。
// uRatio 直接用 Hm20MarkerModel 的那组常量，跟 IK 的 FK 自洽（这一点很重要：
// 标定和 IK 必须用同一个贴点模型，否则标出来的骨长是在补 IK 的模型误差）。
// =============================================================================
#pragma once
// 复用标定向导那套【已被真机验证】的静态标定：闭式解、微秒级、无迭代、
// 无局部极小、对运动多样性零要求。见本文件 tryStaticSeed()。
#include "hand/HandSelfCalibration.hpp"
#include "hand/HandModel.hpp"

#include <array>
#include <cmath>
#include <chrono>
#include <future>
#include <deque>
#include <string>
#include <vector>
#include <algorithm>

#include "estimate/Hm20AssocContract.hpp"   // 【只要类型，不要那个类】原来这里 include 的是
                                            // HandSkeletonAssociator.hpp（闭包 5776 行），但本文件
                                            // 一次都没提到 Hm20SkeletonAssociator，要的只是
                                            // SkeletonFrameResult / Hm20Config / IHm20InferenceBackend
                                            // 这些类型 —— 它们在 contract 里（闭包 1557 行）。
                                            // estimate 目录下有 7 个头都犯了同一个错，每个多吃 4219 行，
                                            // 而这些头又被 UI 层层包含，代价是乘出来的。
#include "estimate/Hm20IkRefiner.hpp"

// 【拆文件】类型契约(枚举/配置/结果)已移到 Hm20AutoCalibTypes.hpp；
// 必须在本文件重新打开 mocap::hm20 之前引入，否则类型会落到 mocap::hm20::mocap::hm20。
#include "estimate/Hm20AutoCalibTypes.hpp"

namespace mocap {
namespace hm20 {

// add/sub/mul/dot/norm/cross/kabsch/matToQuat 都在 mocap::hm20::detail。
// 【必须自己写这一行】不能指望别的头文件先被 include 而把 detail 带进来 ——
// 那样单独 include 本文件就编不过，而且是"看 include 顺序脸色"的隐性依赖。
using namespace detail;


// ---------------------------------------------------------------------------
class Hm20AutoCalib {
public:
    Hm20AutoCalib() = default;
    explicit Hm20AutoCalib(const AutoCalibConfig& c) : cfg_(c) {}

    void configure(const AutoCalibConfig& c) { cfg_ = c; }

    // -------------------------------------------------------------------------
    // 自标定结果导 JSON，给 .pcrec 文件头用。纯 std::string，不依赖 Qt。
    //
    // 【为什么必须录】现在文件头里存的是 HandTemplateStore 那份【存盘的】模板，
    // 而在线跑的时候用的是自标定【当前解出来的】这一份 —— 两者可以完全不同
    // (自标定还没 Ready、或者解出来了但没提交)。离线拿存盘模板去复算，
    // 得到的根本不是当时那条链路，比对基线必然对不上，然后会浪费一整天
    // 去查一个不存在的 bug。
    // -------------------------------------------------------------------------
    std::string resultJson() const {
        std::string o = "{";
        auto b = [&](const char* k, bool v) {
            o += "\""; o += k; o += "\":"; o += (v ? "true" : "false"); o += ","; };
        auto d = [&](const char* k, double v) {
            char t[64]; std::snprintf(t, sizeof(t), "%.6g", v);
            o += "\""; o += k; o += "\":"; o += t; o += ","; };
        auto arrB = [&](const char* k, const bool* v, int n) {
            o += "\""; o += k; o += "\":[";
            for (int i = 0; i < n; ++i) { o += (v[i] ? "true" : "false"); if (i != n-1) o += ","; }
            o += "],"; };

        b("isRight",           res_.isRight);
        b("handednessKnown",   res_.handednessKnown);
        b("frameAnatomical",   res_.frameAnatomical);
        b("backValid",         res_.backValid);
        b("dorsumReordered",   res_.dorsumReordered);
        d("handLenMm",         res_.handLenMm);
        d("bundleRmseMm",      res_.bundleRmseMm);
        d("thumbAxialK",       res_.thumbAxialK);
        b("thumbAxialFitted",  res_.thumbAxialFitted);
        d("thumbPronation0",   res_.thumbPronation0);
        b("thumbPronationFitted",   res_.thumbPronationFitted);
        d("thumbPronationContrast", res_.thumbPronationContrast);
        d("thumbCoverage",     thumbCoverage());
        arrB("anchorFitted",   res_.anchorFitted.data(), 5);
        arrB("lengthFitted",   res_.lengthFitted.data(), 5);

        o += "\"anchorsMm\":[";
        for (int f = 0; f < 5; ++f) for (int c = 0; c < 3; ++c) {
            char t[48]; std::snprintf(t, sizeof(t), "%.4f", res_.anchorsMm[size_t(f)][size_t(c)]);
            o += t; if (f != 4 || c != 2) o += ","; }
        o += "],\"lengthsMm\":[";
        for (int f = 0; f < 5; ++f) for (int c = 0; c < 3; ++c) {
            char t[48]; std::snprintf(t, sizeof(t), "%.4f", res_.lengthsMm[size_t(f)][size_t(c)]);
            o += t; if (f != 4 || c != 2) o += ","; }
        o += "],\"backMm\":[";
        for (int m = 0; m < 5; ++m) for (int c = 0; c < 3; ++c) {
            char t[48]; std::snprintf(t, sizeof(t), "%.4f", res_.backMm[size_t(m)][size_t(c)]);
            o += t; if (m != 4 || c != 2) o += ","; }
        o += "],\"markersMm\":[";
        for (int m = 0; m < 20; ++m) for (int c = 0; c < 3; ++c) {
            char t[48]; std::snprintf(t, sizeof(t), "%.4f", res_.markersMm[size_t(m)][size_t(c)]);
            o += t; if (m != 19 || c != 2) o += ","; }
        o += "]}";
        return o;
    }
    const AutoCalibConfig& config() const { return cfg_; }

    // 【只清跟踪态，保留标定结果】给面板的"手背复位"用。
    // reset() 会把模板/anchor/骨长一起清掉，那等于要求用户重新标定一遍 ——
    // 而用户想要的只是"把乱掉的跟踪甩掉"，不是重来。
    void resetTracking() {
        buf_.clear();
        hasPrevPose_ = false;
        driftCount_ = 0;
        // 位姿历史清掉，但 res_（模板/anchor/骨长/拇指常数）原样保留
    }

    void reset() {
        buf_.clear();
        validZAcc_ = 0.0; validZN_ = 0;
        staticSeeded_ = {}; staticSeedDevMm_ = {}; rigidStat_ = {};
        stage_ = AutoCalibStage::Idle;
        warmupCount_ = 0;
        driftCount_ = 0;
        hasPrevPose_ = false;
        reject_ = AutoCalibReject::None;
        res_ = AutoCalibResult{};
        for (auto& v : anchorSamp_) v.clear();
        for (auto& v : spanMax_) v = {0.0, 0.0};
        for (auto& v : spanHist_) { v[0].clear(); v[1].clear(); }
        extBestLen_ = {};
        shots_.clear();
        validRmse_.clear();
        thumbDirAcc_ = Vec3{}; thumbDirN_ = 0;
        thumbDpLo_ = Vec3{}; thumbDpHi_ = Vec3{}; thumbDpN_ = 0;
        attempts_ = 0;
        exhausted_ = false;
        handSignAcc_ = 0.0;
        haveDorsal_ = false;
        handSignN_ = 0;
        dirty_ = false;
    }

    AutoCalibStage stage() const { return stage_; }
    AutoCalibReject lastReject() const { return reject_; }
    const AutoCalibResult& result() const { return res_; }
    bool backReady() const { return res_.backValid; }
    bool fullyReady() const { return stage_ == AutoCalibStage::Ready; }

    // 面板「右手」的当前值。每帧刷一次，成本是一次 bool 赋值。
    // 不走 configure() 是因为那个要整份配置，而手性是用户随时会拨的开关。
    void setUserHandedness(bool isRight) { cfg_.userIsRight = isRight; }
    int  attempts() const { return attempts_; }

    // 拇指引导：0=外展伸直, 1=对掌触食指, 2=对掌触小指, 3=完成
    int    thumbGuideStep() const { return thumbReady() ? 3 : int(thumbCoverage() * 3.0); }
    // 0..1。两个因子都要够：方向张角负责 anchor 的球面拟合条件数，
    // 远节球空间范围负责 thumbAxialK 的可辨识性（耦合项 = k * 外展角，
    // 外展角小的时候整项接近 0，怎么拟合都无所谓）。
    double thumbCoverage() const {
        if (!cfg_.guidedThumb) return 1.0;
        return std::min(1.0, thumbSpreadDeg() / cfg_.thumbMinSpreadDeg) *
               std::min(1.0, thumbDpRangeMm() / cfg_.thumbMinDpRangeMm);
    }
    bool thumbReady() const { return !cfg_.guidedThumb || thumbCoverage() >= 0.999; }
    // 给操作者的一句话。【必须具体到动作】"请增加拇指活动范围"这种话没人知道
    // 该怎么做；"拇指指尖去碰食指指尖"是能照着做的。
    const char* thumbGuideHint() const {
        if (thumbReady()) return "";
        // 【故意不说"碰到某根手指"】指尖相触会让两颗球并成一个光斑，
        // 反而丢观测。要的是大幅度、不接触的挥动。
        // 【隐形模式下不该出现在这里】guidedThumb=false 时 thumbReady() 恒真，
        // 走不到这两行。保留是为了 guidedThumb 被显式打开时仍有可照做的指令 ——
        // "请增加拇指活动范围"这种话没人知道该怎么做。
        if (thumbSpreadDeg() < cfg_.thumbMinSpreadDeg * 0.6)
            return "拇指大幅外展再收回（张成 L 形 <-> 贴回手掌），反复 2~3 次，别碰到其他手指";
        return "拇指再做对掌：指尖划向小指根部方向，划到底再回来";
    }
    bool exhausted() const { return exhausted_; }
    // IK 是否可以开。【调用方必须判这个】—— 局部系没对齐到解剖学系之前开 IK，
    // 实测骨轴中位从 12.8° 劣化到 20.7°、IK 残差 10mm（正常 1.6mm）。
    // 【调用方必须判这个】局部系没对齐到解剖学系、或参数还没收敛就开 IK，
    // 实测骨轴中位从 12.3° 劣化到 18.9°、IK 残差 6mm（收敛后 <2mm）。
    // 宁可退回"只用几何法"这一档，也不要输出一组自洽但错误的关节角。
    bool ikUsable() const {
        return res_.frameAnatomical && nAnchorFitted() >= 4 &&
               res_.bundleRmseMm >= 0.0 && res_.bundleRmseMm < ikRmseGate_;
        // 注意：拇指引导没做完不挡 IK —— 四指的参数是独立收敛的，拇指
        // anchorFitted 为 false 时它继续用群体均值，只是那一根精度差些。
        // 为一根手指把另外四根一起关掉不划算。
    }
    // 【改为可调】原来硬编码 4.0mm。你的录制里 bundleRmse 中位 20mm，
    // IK 被永久锁死——而 20mm 里的大头来自相机不同步（时间戳离散度
    // 中位 5.76ms / 帧长 13.9ms），不是标定质量真的差到那个程度。
    // 手背刚体自检显示真实 3D 噪声底只有 2.09mm，说明标定本身没问题。
    //
    // 4.0mm 的门限是按"相机完美同步 + 120fps + 4 台以上"定的。你的设置
    // 是 3 台 / 72fps / USB 不同步，这个条件下 bundleRmse 稳态就在
    // 15~25mm 左右。门限不放开，IK 永远不会跑，遮挡点永远没有运动学约束。
    //
    // 放到 25.0mm 是基于你这批录制的实测：bundleRmse 中位 20.08mm、
    // P95 约 22mm。25.0 刚好卡在 P99 以上，允许 IK 在绝大多数帧跑起来，
    // 而异常帧（比如手整个出了视野导致标定崩溃）仍然会被挡住。
    //
    // 这不是最终值。等你换到硬件同步 / 更多相机 / Release 构建之后，
    // bundleRmse 会显著下降，到时候可以收回来。
    double ikRmseGate_ = 25.0;

    void setIkRmseGate(double v) { ikRmseGate_ = v; }
    double ikRmseGate() const { return ikRmseGate_; }
    static double cfgIkRmseGate() { return 25.0; }

    // 本帧是否产生了需要下发到 associator/IK 的新参数。读一次就清零。
    bool takeDirty() { const bool d = dirty_; dirty_ = false; return d; }

    // 进度 0..1，给 UI 画进度条
    double progress() const {
        switch (stage_) {
            case AutoCalibStage::Idle:       return 0.0;
            case AutoCalibStage::Warmup:     return double(warmupCount_) / std::max(cfg_.warmupFrames, 1);
            case AutoCalibStage::Relocking:
            case AutoCalibStage::Collecting: return double(buf_.size()) / std::max(cfg_.collectFrames, 1);
            case AutoCalibStage::Validating: return double(validRmse_.size()) / std::max(cfg_.validateFrames, 1);
            case AutoCalibStage::Locked:     return anchorProgress();
            case AutoCalibStage::Ready:      return 1.0;
        }
        return 0.0;
    }

    // 给 UI 的一行状态。故意写成"操作者能照着做"的话，不是内部术语。
    std::string statusText() const {
        char b[192];
        switch (stage_) {
            case AutoCalibStage::Idle:
                if (exhausted_)
                    std::snprintf(b, sizeof(b), "自标定：连续 %d 次校验未通过，建议改用手工标定", attempts_);
                else
                    std::snprintf(b, sizeof(b), "自标定：等待整只手进入视野（手背5点+手指≥%d点）", cfg_.minFinger);
                break;
            // 【隐形模式的文案原则】只报告进度，不下达动作指令。
            // 用户正常用手就会覆盖到需要的姿势，不需要停下来做体操；
            // 但"进行到哪一步、有没有在推进"必须让人看得见 —— 一个悄悄在后台
            // 跑、又不说自己跑到哪的东西，跟没跑没区别。
            case AutoCalibStage::Warmup:
                std::snprintf(b, sizeof(b), cfg_.silentMode
                              ? "自标定：起步 %d/%d"
                              : "自标定：请把手摆稳（%d/%d）",
                              warmupCount_, cfg_.warmupFrames);
                break;
            case AutoCalibStage::Collecting:
            case AutoCalibStage::Relocking:
                std::snprintf(b, sizeof(b), cfg_.silentMode
                              ? "自标定：学手背 %d/%d"
                              : "自标定：正在标定手背 %d/%d，请缓慢转动手腕",
                              int(buf_.size()), cfg_.collectFrames);
                break;
            case AutoCalibStage::Validating:
                std::snprintf(b, sizeof(b), "自标定：正在校验手背模板 %d/%d（第 %d 次尝试）",
                              int(validRmse_.size()), cfg_.validateFrames, attempts_ + 1);
                break;
            case AutoCalibStage::Locked:
                if (!thumbReady())
                    std::snprintf(b, sizeof(b), "自标定：拇指引导 %.0f%% —— %s",
                                  thumbCoverage() * 100.0, thumbGuideHint());
                else
                {
                    // 【进度和质量分开报】原来只有一个百分比，而那只是"样本攒了多少"，
                    // 不代表参数解对了 —— 真机上进度 100% 而 IK 残差 18~40mm 就是
                    // 被它误导的。现在把"几根已经量到骨长"一起写出来。
                    int ns = 0;
                    for (int f = 0; f < 5; ++f) if (staticSeeded_[size_t(f)]) ++ns;
                    std::snprintf(b, sizeof(b), cfg_.silentMode
                                  ? "自标定：手背OK，已量骨长 %d/5，样本 %.0f%%（正常用手即可）"
                                  : "自标定：手背已锁定，已量骨长 %d/5，样本 %.0f%%（请反复屈伸五指）",
                                  ns, anchorProgress() * 100.0);
                }
                break;
            case AutoCalibStage::Ready:
                std::snprintf(b, sizeof(b), "自标定：完成（手背RMS %.1fmm，anchor %d/5）",
                              lastRmse_, nAnchorFitted());
                break;
        }
        return std::string(b);
    }

    // =========================================================================
    // 每帧喂一次骨架结果。返回 true 表示本帧参数有更新，调用方应该下发。
    //
    // 【为什么吃 SkeletonFrameResult 而不是裸点】自标定要用到网络给的标签、
    // 逐点概率、以及 associator 解出的腕部位姿 —— 这三样是闭环的关键：
    // 有标签才知道哪 5 个点是手背，有概率才能挡掉低质量帧，有腕部位姿才能把
    // 手指点搬进腕部系去拟合 anchor。这正是你要的"拿到连线和匹配结果之后，
    // 在过程中自动闭环标定"。
    // =========================================================================
    // 后台束调整算完了就把结果换进来。每帧开头调一次，不阻塞。
    // 【只取拟合出来的参数】其余字段(stage/progress/诊断计数)以主对象为准，
    // 因为后台那份是旧快照，拿它的状态覆盖会把这段时间的进展抹掉。
    void pollBundleJob() {
        if (!bundleJob_.f.valid()) return;
        if (bundleJob_.f.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
        const AutoCalibResult nr = bundleJob_.f.get();
        res_.bundleLastMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - bundleT0_).count();
        ++res_.bundleRuns;
        // 无缝换：只有束调整真的把残差压下去了才采用，否则保持上一次的
        if (nr.bundleRmseMm > 0.0 &&
            (bundleBefore_ <= 0.0 || nr.bundleRmseMm <= bundleBefore_ * 1.05)) {
            res_.anchorsMm   = nr.anchorsMm;
            res_.lengthsMm   = nr.lengthsMm;
            res_.markersMm   = nr.markersMm;
            res_.thumbAxialK = nr.thumbAxialK;
            res_.thumbAxialFitted = nr.thumbAxialFitted;
            res_.thumbPronation0 = nr.thumbPronation0;
            res_.thumbPronationFitted = nr.thumbPronationFitted;
            res_.thumbPronationContrast = nr.thumbPronationContrast;
            res_.bundleRmseMm = nr.bundleRmseMm;
            res_.bundleShots  = nr.bundleShots;
            dirty_ = true;   // 通知上层：参数变了，该重新下发
        }
        const bool improved = (bundleBefore_ <= 0.0) ||
            (nr.bundleRmseMm > 0.0 &&
             nr.bundleRmseMm < bundleBefore_ * (1.0 - cfg_.stallImproveRatio));
        stallRuns_ = improved ? 0 : (stallRuns_ + 1);
        if (stage_ == AutoCalibStage::Ready && stallRuns_ >= cfg_.bundleStallRuns) {
            bundleStalled_ = true;
            res_.bundleStalled = true;
        }
    }

    bool feed(const SkeletonFrameResult& r) {
        pollBundleJob();
        if (!r.valid) { onBadFrame(); return false; }

        // ---- 逐帧质量门 ----
        int nd = 0; double dconf = 1e9;
        for (int i = 0; i < 5; ++i) {
            if (!r.markers[size_t(i)].observed) continue;
            ++nd;
            dconf = std::min(dconf, r.markers[size_t(i)].confidence);
        }
        int nf = 0;
        for (int i = 5; i < 20; ++i) if (r.markers[size_t(i)].observed) ++nf;

        const bool gateCount = (nd >= cfg_.minDorsum) && (nf >= cfg_.minFinger);
        const bool gateConf  = (nd == 0) ? false : (dconf >= cfg_.minDorsumConf);
        const bool gatePenta = r.pentagonOk;

        // 手背质心与朝向的帧间变化 —— 太快说明正在挥手，此刻各相机不同步
        // 造成的形变最大，攒进去的样本会把模板拉歪。
        Vec3 c{0, 0, 0};
        for (int i = 0; i < 5; ++i) c = add(c, r.markers[size_t(i)].posWorld);
        c = mul(c, 0.2);
        bool gateSpeed = true;
        if (hasPrevPose_) {
            const double d = norm(sub(c, prevCentroid_));
            const double a = quatAngleDeg(r.wristQuat, prevQuat_);
            gateSpeed = (d <= cfg_.maxSpeedMmPerFrame) && (a <= cfg_.maxRotDegPerFrame);
        }
        prevCentroid_ = c;
        prevQuat_ = r.wristQuat;
        hasPrevPose_ = true;

        reject_ = AutoCalibReject::None;
        if (!gateCount)      reject_ = AutoCalibReject::NotEnoughPoints;
        else if (!gateConf)  reject_ = AutoCalibReject::LowConfidence;
        else if (!gatePenta) reject_ = AutoCalibReject::PentagonBad;
        else if (!gateSpeed) reject_ = AutoCalibReject::MovingTooFast;
        const bool ok = gateCount && gateConf && gatePenta && gateSpeed;

        // ---- 锁定之后走另一条路：持续微调 + 漂移检测 + 学手指 ----
        if (stage_ == AutoCalibStage::Validating) return onValidateFrame(r, ok);
        if (stage_ == AutoCalibStage::Locked || stage_ == AutoCalibStage::Ready) {
            return onLockedFrame(r, ok);
        }

        if (!ok) { onBadFrame(); return false; }

        if (exhausted_) return false;
        if (stage_ == AutoCalibStage::Idle) {
            stage_ = AutoCalibStage::Warmup;
            warmupCount_ = 0;
        }
        if (stage_ == AutoCalibStage::Warmup) {
            if (++warmupCount_ < cfg_.warmupFrames) return false;
            stage_ = (res_.backValid ? AutoCalibStage::Relocking : AutoCalibStage::Collecting);
            buf_.clear();
        }

        // ---- Collecting ----
        Sample s{};
        for (int m = 0; m < 20; ++m) {
            s.p[size_t(m)] = r.markers[size_t(m)].posWorld;
            s.ok[size_t(m)] = r.markers[size_t(m)].observed;
        }
        // 展布检查：把一簇噪声当手背会让后面全错，先挡掉
        double spread = 0;
        for (int i = 0; i < 5; ++i) spread += norm(sub(s.p[size_t(i)], c));
        spread /= 5.0;
        if (spread < cfg_.minSpreadMm) { reject_ = AutoCalibReject::SpreadTooSmall; return false; }

        buf_.push_back(s);
        if (int(buf_.size()) > cfg_.collectFrames * 2) buf_.pop_front();
        if (int(buf_.size()) < cfg_.collectFrames) {
            reject_ = AutoCalibReject::WaitingSamples;
            return false;
        }
        return tryFreeze();
    }

private:
    struct Sample {
        std::array<Vec3, 20> p{};
        std::array<bool, 20> ok{};
    };

    // 束调整的姿势库条目：腕部系下的 20 点 + 可见性 + 姿势签名
    struct Shot {
        std::array<Vec3, 20> p{};
        std::array<bool, 20> ok{};
        std::array<double, 5> sig{};
    };
    struct PerShotAngles { std::array<std::array<double, 4>, 5> q{}; };

    static double quatAngleDeg(const Quat& a, const Quat& b) {
        double d = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
        d = std::min(1.0, std::fabs(d));
        return 2.0 * std::acos(d) * 180.0 / 3.14159265358979323846;
    }

    void onBadFrame() {
        if (stage_ == AutoCalibStage::Warmup) {
            // 抖一帧不至于全归零，但连续不合格要退回去，避免"手还没摆好就开标"
            warmupCount_ = std::max(0, warmupCount_ - 2);
            if (warmupCount_ == 0) stage_ = AutoCalibStage::Idle;
        } else if (stage_ == AutoCalibStage::Collecting || stage_ == AutoCalibStage::Relocking) {
            if (!buf_.empty()) buf_.pop_front();
            if (buf_.empty()) { stage_ = AutoCalibStage::Idle; warmupCount_ = 0; }
        }
    }

    // 手背是刚体：两两距离跨帧应该恒定。不恒定说明网络在这几帧里把手背标错了，
    // 冻结下去会把错误固化成模板，后面每一帧都跟着错 —— 这个检查必须有。
    // 返回手背两两距离跨帧标准差的最大值 —— 这套点位当前的噪声底。
    double dorsumSigma() const {
        const double n = double(buf_.size());
        if (n < 4) return 1e9;
        double worst = 0;
        for (int i = 0; i < 5; ++i)
            for (int j = i + 1; j < 5; ++j) {
                double s1 = 0, s2 = 0;
                for (const auto& s : buf_) {
                    const double d = norm(sub(s.p[size_t(i)], s.p[size_t(j)]));
                    s1 += d; s2 += d * d;
                }
                const double mean = s1 / n;
                worst = std::max(worst, std::sqrt(std::max(s2 / n - mean * mean, 0.0)));
            }
        return worst;
    }

    bool dorsumRigid() const {
        const double n = double(buf_.size());
        if (n < 4) return false;
        for (int i = 0; i < 5; ++i)
            for (int j = i + 1; j < 5; ++j) {
                double s1 = 0, s2 = 0;
                for (const auto& s : buf_) {
                    const double d = norm(sub(s.p[size_t(i)], s.p[size_t(j)]));
                    s1 += d; s2 += d * d;
                }
                const double mean = s1 / n;
                const double var = std::max(s2 / n - mean * mean, 0.0);
                if (std::sqrt(var) > cfg_.rigidTolMm) return false;
            }
        return true;
    }

    // -------------------------------------------------------------------------
    bool tryFreeze() {
        if (!dorsumRigid()) {
            reject_ = AutoCalibReject::NotRigid;
            buf_.pop_front();
            return false;
        }

        // ---- 以最后一帧为参考，把所有帧 Kabsch 对齐后取均值 ----
        // 手在动，直接平均世界坐标没有意义，必须先对齐。
        const Sample& ref = buf_.back();
        std::array<Vec3, 20> accAll{};
        std::array<int, 20>  cnt{};
        int used = 0;
        std::vector<Vec3> src, dst;
        for (const auto& s : buf_) {
            src.clear(); dst.clear();
            for (int i = 0; i < 5; ++i) { src.push_back(s.p[size_t(i)]); dst.push_back(ref.p[size_t(i)]); }
            Mat3 R{}; Vec3 t{};
            if (kabsch(src, dst, R, t) < 0) continue;
            for (int m = 0; m < 20; ++m) {
                if (!s.ok[size_t(m)]) continue;
                accAll[size_t(m)] = add(accAll[size_t(m)], add(matVec(R, s.p[size_t(m)]), t));
                ++cnt[size_t(m)];
            }
            ++used;
        }
        if (used < cfg_.collectFrames / 2) return false;
        std::array<Vec3, 20> avg{};
        for (int m = 0; m < 20; ++m)
            if (cnt[size_t(m)]) avg[size_t(m)] = mul(accAll[size_t(m)], 1.0 / cnt[size_t(m)]);

        // ---- 建腕部局部系：+X 手指方向、+Y 拇指侧、+Z 手背法向 ----
        Vec3 dc{0, 0, 0};
        for (int i = 0; i < 5; ++i) dc = add(dc, avg[size_t(i)]);
        dc = mul(dc, 0.2);

        Vec3 fx{0, 0, 0}; int nfx = 0;
        for (int m : {8, 11, 14, 17}) if (cnt[size_t(m)]) { fx = add(fx, avg[size_t(m)]); ++nfx; }
        if (nfx < 2) return false;
        Vec3 X = sub(mul(fx, 1.0 / nfx), dc);
        if (norm(X) < 1e-6) return false;
        X = normalize(X);

        if (!cnt[5]) return false;
        Vec3 Y = sub(avg[5], dc);
        Y = sub(Y, mul(X, dot(Y, X)));
        if (norm(Y) < 1e-6) return false;
        Y = normalize(Y);
        Vec3 Z = cross(X, Y);

        // ---- 左右手判定 ----
        // 【思路】上面这组 (X,Y,Z) 是按"拇指在 +Y"造的，所以它对右手是右手系、
        // 对左手是左手系 —— 也就是说 Z 对右手指向手背外侧、对左手指向掌侧。
        // 而手背贴点的物理事实是：反光球在手背【外】侧，手指屈曲时指尖往掌侧走。
        // 所以只要看手指点相对手背平面的有符号距离，就能定出真正的手背法向，
        // 进而定出手性。用整个采集窗口累计，比单帧稳。
        //
        // 【为什么不能只看一帧】手完全摊平时手指点几乎就在手背平面上，
        // 有符号距离接近 0，判不出来。所以要求 |均值| 超过阈值，
        // 达不到就报"未知"，由调用方沿用用户设置 —— 不猜。
        {
            // 【必须检查这两个点有没有样本】avg[m] 在 cnt[m]==0 时是【零向量】
            // ——上面建腕部系那几步每一处都查了 cnt（nfx<2 返回、!cnt[5] 返回），
            // 唯独这里漏了。
            //
            // 真机症状：握拳时 m8（食指近节）和 m17（小指近节）容易全程被挡，
            // avg 留成零向量，n 直接算废，手性随之乱翻 ——
            // 用户描述是"只要有一个点进入预测状态，就马上错判成右手 99%"。
            //
            // 没有样本时不猜：haveDorsal_ 保持 false，调用方沿用用户设置。
            // 这跟本函数其它地方"达不到阈值就报未知"的策略一致。
            const Vec3 nRaw = (cnt[8] && cnt[17])
                                  ? cross(sub(avg[8], dc), sub(avg[17], dc))
                                  : Vec3{0, 0, 0};
            // 两点跟质心接近共线时叉积也会退化，一并挡掉。
            const bool nOk = (norm(nRaw) > 1e-6);
            const Vec3 n = nOk ? normalize(nRaw) : Vec3{0, 0, 0};
            double acc = 0; int na = 0;
            // 【nOk 为假时整段跳过，不能只是让 n 归零】
            // n 是零向量的话 dot(nf0, n) 恒为 0，sgn 恒取 +1，累加出来的
            // handSignAcc_ 照样是个"看起来很自信"的数（实测能到 18mm），
            // 然后被当成有效判据用掉 —— 那正是这个 bug 最难查的地方。
            if (!nOk) buf2_.clear();
            const auto& src2 = nOk ? buf_ : buf2_;
            for (const auto& s : src2) {
                Vec3 c2{0,0,0};
                for (int i = 0; i < 5; ++i) c2 = add(c2, s.p[size_t(i)]);
                c2 = mul(c2, 0.2);
                // 用该帧自己的手背平面法向（由手背 3 点定），跟 n 同向化
                const Vec3 nf0 = normalize(cross(sub(s.p[1], s.p[0]), sub(s.p[2], s.p[0])));
                const double sgn = (dot(nf0, n) >= 0) ? 1.0 : -1.0;
                for (int m = 5; m < 20; ++m) {
                    if (!s.ok[size_t(m)]) continue;
                    acc += sgn * dot(sub(s.p[size_t(m)], c2), nf0);
                    ++na;
                }
            }
            if (na > 0) {
                handSignAcc_ = acc / na;
                handSignN_ = na;
                res_.handSignMm = handSignAcc_;   // 判据本身也上报，方便判断"离阈值多远"
                if (std::fabs(handSignAcc_) >= cfg_.handednessMinMm) {
                    // 手指点整体落在 -n 侧 => n 是手背外法向
                    nDorsal_ = (handSignAcc_ < 0) ? n : mul(n, -1.0);
                    haveDorsal_ = true;
                    if (cfg_.detectHandedness) {
                        // ---- 手性判据：不依赖拇指 ----
                        // 【原判据的毛病】用的是 dot(Z, nDorsal_)，而 Z = X×Y、
                        // Y 来自 avg[5] —— 拇指掌骨球。手性判定因此挂在【全手最
                        // 容易丢和误配的那颗球】上，拇指一歪整个判定就翻，表现就是
                        // "自标定有时候左右手错判"，而且判据值还很大(实测 18mm)，
                        // 看起来很"自信"。
                        //
                        // 新判据只用四指 pp 和手背法向：
                        //   w = nDorsal_ × X   （手背平面内、垂直于指向的横轴）
                        //   食指相对小指在 w 上的投影，左右手符号相反
                        // 食指(8)和小指(17)是四指里最靠边的两颗，比拇指稳得多。
                        const Vec3 w = cross(nDorsal_, X);
                        const double lat = dot(sub(avg[8], avg[17]), w);
                        const bool byLateral = (lat > 0);
                        const bool byThumb   = (dot(Z, nDorsal_) > 0);
                        res_.handednessAgree = (byLateral == byThumb);
                        res_.handSignLatMm = lat;
                        // 【两个判据不一致时不下结论】一致才采信 —— 宁可"判不出"
                        // 而沿用面板设置，也不要给一个会翻来翻去的错答案。
                        res_.isRightDetected = byLateral;
                        // 两判据不一致 => 拇指标签多半歪了，这一轮不认手性
                        res_.handednessKnown = res_.handednessAgree;
                        res_.handednessConflict = res_.handednessAgree
                                && (res_.isRightDetected != cfg_.userIsRight);
                        // 【生效值】默认取用户设置。applyHandedness=true 才采信推断，
                        // 理由见 AutoCalibConfig::applyHandedness 上的说明。
                        res_.isRight = cfg_.applyHandedness ? res_.isRightDetected
                                                            : cfg_.userIsRight;
                    }
                }
            }
        }
        // 左手时把 Y 翻过来，保证 (X,Y,Z) 恒为右手系、且 Z 恒为手背外法向。
        // 这样 IK 的贴点模型（径向偏置沿 +Z）对两只手都成立，只需要在 IK 里
        // 按手性做一次 diag(1,-1,1) 镜像。
        // 【注意这里翻的是模板本身】用生效值，不是推断值 —— 用推断值时一次误判
        // 就会把整个手背模板镜像掉，而 RMSE 自检查不出来（见 applyHandedness）。
        //
        // 【不再要求 handednessKnown】这是我改出来的一个回归：上一版把
        // handednessKnown 改成"两个判据一致才为真"，而这里拿它当翻转的门 ——
        // 结果判据一不一致，左手的 Y 就不翻，模板系的 Z 指向掌侧，整个 anchor
        // 和模板全错，IK 残差爆掉，表现成"左手+左手设置质量很差"。
        // 生效手性来自【用户的显式设置】，检测成不成功跟它无关。检测只负责
        // 报告冲突，没有资格决定翻不翻。
        const bool effRight = cfg_.applyHandedness
                            ? (res_.handednessKnown ? res_.isRightDetected : cfg_.userIsRight)
                            : cfg_.userIsRight;
        res_.isRight = effRight;
        if (!effRight) { Y = mul(Y, -1.0); Z = cross(X, Y); }

        auto toLocal = [&](const Vec3& p) {
            const Vec3 d = sub(p, dc);
            return Vec3{dot(d, X), dot(d, Y), dot(d, Z)};
        };
        for (int i = 0; i < 5; ++i) res_.backMm[size_t(i)] = toLocal(avg[size_t(i)]);

        // ---- 手背编号规范化（见 canonicalDorsumOrder 上方的说明）----
        // 局部系里 +Y 就是桡侧(拇指侧)、+Z 就是手背外法向，正好是规则要的两个方向。
        res_.dorsumReordered = false;
        res_.dorsumWouldReorder = false;
        if (haveDorsal_) {
            const auto ord = canonicalDorsumOrder(res_.backMm, Vec3{0, 1, 0}, Vec3{0, 0, 1});
            bool identity = true;
            for (int i = 0; i < 5; ++i) if (ord[size_t(i)] != i) identity = false;
            res_.dorsumWouldReorder = !identity;
            // 【只有显式打开才真的重排】默认只记录"本来会重排"，不动数据 ——
            // 动了就会打乱模板与网络标签的对应关系，而 RMSE 自检查不出来。
            if (!identity && cfg_.canonicalizeDorsum) {
                std::array<Vec3, 5> tmp{};
                for (int i = 0; i < 5; ++i) tmp[size_t(i)] = res_.backMm[size_t(ord[size_t(i)])];
                res_.backMm = tmp;
                res_.dorsumReordered = true;
            }
        }
        res_.backValid = true;

        // 手长：手背 5 点平均半径 ≈ 0.115 * handLen（真 rig 实测），
        // 跟 Hm20IkRefiner::setHandLenFromBackTemplate 用的是同一条关系。
        double sr = 0;
        for (int i = 0; i < 5; ++i) sr += norm(res_.backMm[size_t(i)]);
        res_.handLenMm = std::clamp(sr / 5.0 / 0.115, 150.0, 220.0);

        // anchor / 骨长先给群体均值的缩放版当起点，等样本够了再逐指替换成解出来的
        seedPopulationParams();
        rebuildNeutralMarkers();

        lockedRef_ = res_.backMm;
        frozenSigma_ = dorsumSigma();
        stage_ = AutoCalibStage::Validating;
        validRmse_.clear();
        for (auto& v : anchorSamp_) v.clear();
        for (auto& v : spanMax_) v = {0.0, 0.0};
        driftCount_ = 0;
        dirty_ = true;
        return true;
    }

    // -------------------------------------------------------------------------
    // 冻结后自检：拿刚冻的模板跑若干帧，看 Kabsch 残差站不站得住。
    // 站得住 -> Locked；站不住 -> 整个丢掉重来（重来时网络已经被上一版模板
    // 影响过，破对称的结果会不同，所以重试是有意义的，不是原地打转）。
    // -------------------------------------------------------------------------
    bool onValidateFrame(const SkeletonFrameResult& r, bool gateOk) {
        if (gateOk && r.wristPoseValid && r.dorsumRmseMm >= 0) {
            validRmse_.push_back(r.dorsumRmseMm);
            // ---- 镜像自检 ----
            // 【为什么 RMSE 不够】手背五点近平面且接近对称，把模板镜像一下，
            // Kabsch 照样能拟出不大的残差（反射 ≈ 绕面内某轴转 180°）。所以
            // "残差小"不等于"模板对"，这正是真机上"标完就错、诊断全绿"的原因。
            //
            // 加这一条：把本帧观测到的手指点变换回模板系，看它们落在手背平面的
            // 哪一侧。物理事实是手指往掌侧走，也就是模板系的 -Z 侧；如果它们
            // 明确落在 +Z 侧，说明模板(或它的 Z 轴)被镜像了。
            // 用跟手性判定同一个阈值和同一套语义，含糊区间(|均值|<阈值，比如手
            // 完全摊平)不下结论 —— 不猜。
            double zs = 0.0; int zn = 0;
            for (int m = 5; m < 20; ++m) {
                if (!r.markers[size_t(m)].observed) continue;
                const Vec3 d = sub(r.markers[size_t(m)].posWorld, r.wristT);
                // wristR 是行主序、把模板系转到世界系；转回去用它的转置
                zs += d[0]*r.wristR[2] + d[1]*r.wristR[5] + d[2]*r.wristR[8];
                ++zn;
            }
            if (zn > 0) { validZAcc_ += zs / zn; ++validZN_; }
        }
        if (int(validRmse_.size()) < cfg_.validateFrames) return false;

        std::vector<double> v = validRmse_;
        std::sort(v.begin(), v.end());
        const double med = v[v.size() / 2];
        lastRmse_ = med;
        validRmse_.clear();
        const double thr = std::max(cfg_.validateRmseMinMm, cfg_.validateRmseK * frozenSigma_);

        const double zMean = (validZN_ > 0) ? (validZAcc_ / validZN_) : 0.0;
        validZAcc_ = 0.0; validZN_ = 0;
        // 【只在手性一致时才判镜像】用户完全可以【故意】用不匹配的手性设置
        //（实测左手用右手档反而更稳，因为不匹配时 IK 直接不生效，退回几何路径）。
        // 这种情况下模板系的 Z 本来就指向掌侧，手指点落在 +Z 侧是必然结果，
        // 不是"模板被镜像"的证据 —— 在这里拒绝标定等于剥夺用户的选择权，
        // 而且会让自标定永远标不成(attempts_ 耗尽)。
        // 推断不出手性(手摊平、|判据|<阈值)时同样不下结论。
        res_.mirrorSuspect = (res_.handednessKnown && !res_.handednessConflict
                              && zMean > cfg_.handednessMinMm);

        if (med <= thr && !res_.mirrorSuspect) {
            stage_ = AutoCalibStage::Locked;
            driftCount_ = 0;
            dirty_ = true;
            return true;
        }
        // ---- 自检没过 ----
        // 【不要清空模板重来】那样网络又回到"没有模板"的状态，会再一次给出
        // 同一套错误标法，第二次冻出来的东西跟第一次一模一样 —— 纯粹原地打转，
        // 空转 6 次然后放弃。
        // 正确做法是【保留当前模板继续采集】：模板挂着的时候网络是"有条件"的，
        // 它的手背标法由模板破对称、并且稳定；用这批新观测重新冻结，得到的就是
        // 跟网络当前行为自洽的那一版。这是个不动点迭代，实测两三轮就收敛。
        // 手指参数必须清掉：它们是用上一版（错的）腕部系算出来的。
        ++attempts_;
        for (auto& q : anchorSamp_) q.clear();
        shots_.clear();
        extBestLen_ = {};
        res_.anchorFitted = {};
        res_.lengthFitted = {};
        res_.frameAnatomical = false;
        res_.bundleRmseMm = -1.0;
        res_.bundleShots = 0;
        buf_.clear();
        if (attempts_ >= cfg_.maxAttempts) {
            // 反复标不成，说明这套点位/视野本身有问题。停下来由 UI 提示改用
            // 手工标定，而不是无限重试制造"一直在标"的假象。
            exhausted_ = true;
            stage_ = AutoCalibStage::Idle;
            dirty_ = true;
            return true;
        }
        stage_ = AutoCalibStage::Collecting;   // 手还在，不用重走 Warmup
        dirty_ = true;
        return true;
    }

    // -------------------------------------------------------------------------
    // 锁定之后：① 手背模板小增益微调 ② 漂移检测 ③ 攒 anchor/骨长样本
    // -------------------------------------------------------------------------
    bool onLockedFrame(const SkeletonFrameResult& r, bool gateOk) {
        bool changed = false;

        // ---- ② 漂移检测 ----
        // dorsumRmseMm 是手背模板和当前观测的 Kabsch 残差。贴点没动时它稳定在
        // 1~3mm；某个球被蹭掉/手套转了，它会阶跃上去且【不再回落】。
        // 用"连续超阈值"而不是单帧，避免一次遮挡误触发重标。
        if (r.wristPoseValid && r.dorsumRmseMm > 0) {
            lastRmse_ = 0.9 * lastRmse_ + 0.1 * r.dorsumRmseMm;
            if (r.dorsumRmseMm > cfg_.driftRmseMm) {
                if (++driftCount_ >= cfg_.driftFrames) {
                    // 整个推倒重来：模板错了的话，用它解出来的腕部系也是错的，
                    // anchor 样本跟着一起废，不能留。
                    res_ = AutoCalibResult{};
                    for (auto& v : anchorSamp_) v.clear();
                    for (auto& v : spanMax_) v = {0.0, 0.0};
                    buf_.clear();
                    shots_.clear();
                    warmupCount_ = 0;
                    driftCount_ = 0;
                    stage_ = AutoCalibStage::Idle;
                    dirty_ = true;
                    return true;
                }
            } else {
                driftCount_ = std::max(0, driftCount_ - 1);
            }
        }

        if (!gateOk || !r.wristPoseValid) return false;

        // ---- ① 手背模板微调 ----
        // 把本帧观测的手背点用腕部位姿转回腕部系，跟模板做小增益 EMA。
        // 【必须再对齐一次】否则模板自身的坐标系会随 EMA 慢慢转，
        // 腕部四元数就会有一个谁都查不出来的缓慢漂移。做法是更新完之后
        // 把新模板 Procrustes 对回【锁定时那一版】，只让形状变、不让姿态变。
        {
            std::array<Vec3, 5> upd = res_.backMm;
            for (int i = 0; i < 5; ++i) {
                if (!r.markers[size_t(i)].observed) continue;
                const Vec3 d = sub(r.markers[size_t(i)].posWorld, r.wristT);
                const Vec3 local = matVecT(r.wristR, d);
                upd[size_t(i)] = add(mul(res_.backMm[size_t(i)], 1.0 - cfg_.refineGain),
                                     mul(local, cfg_.refineGain));
            }
            std::vector<Vec3> a, b;
            for (int i = 0; i < 5; ++i) { a.push_back(upd[size_t(i)]); b.push_back(lockedRef_[size_t(i)]); }
            Mat3 R{}; Vec3 t{};
            if (kabsch(a, b, R, t) >= 0)
                for (int i = 0; i < 5; ++i) res_.backMm[size_t(i)] = add(matVec(R, upd[size_t(i)]), t);
        }

        // ---- ③ anchor / 骨长样本 ----
        // ---- anchor 的诚实质量判据：|anchor - pp| 的跨帧标准差 ----
        // 【为什么需要另一个判据】现在报的 anchorResidMm 是 fitSphere 自己的
        // 拟合残差 —— 拟合自己评自己，一堆近似共面的点照样能拟出很小的残差，
        // 而球心可能飘在几十毫米外。真机上"anchor 残差看着正常、IK 残差 18~40mm"
        // 就是这么来的。
        //
        // 这里用一个【每帧都成立的硬约束】做预测型判据：anchor 是掌指关节中心，
        // pp 球贴在近节指骨上，两点【在同一根骨头上】—— 不管 PIP/DIP 怎么弯，
        // |anchor - pp| 恒定。所以拿当前 anchor 去算这个距离，它的跨帧标准差
        // 应该落在观测噪声量级(1~2mm)。std 大 = anchor 是错的，没有例外。
        //
        // 这是"拿拟合结果去预测新帧"，比"拟合残差"强得多：后者只说明这组样本
        // 被拟合得好，前者说明这个 anchor 对没见过的姿势也成立。
        if (res_.wristPoseValidCount >= 0 && r.wristPoseValid) {
            for (int f = 0; f < 5; ++f) {
                if (!res_.anchorFitted[size_t(f)]) continue;
                const int pp = 5 + 3*f;
                if (!r.markers[size_t(pp)].observed) continue;
                const Vec3 loc = matVecT(r.wristR, sub(r.markers[size_t(pp)].posWorld, r.wristT));
                const double d = norm(sub(loc, res_.anchorsMm[size_t(f)]));
                if (!(d > 1.0 && d < 200.0)) continue;      // 明显野点不进统计
                auto& st = rigidStat_[size_t(f)];
                ++st.n; st.sum += d; st.sum2 += d * d;
                // 【滑动而不是全程累计】anchor 会被 refit 更新，全程累计会把
                // 更新前的坏样本一直背着，看不出"改好了没有"。攒够就折半。
                if (st.n >= 480) { st.n /= 2; st.sum /= 2; st.sum2 /= 2; }
                if (st.n >= 20) {
                    const double mean = st.sum / st.n;
                    const double var = std::max(0.0, st.sum2 / st.n - mean * mean);
                    res_.anchorRigidStdMm[size_t(f)] = std::sqrt(var);
                    res_.anchorRigidMeanMm[size_t(f)] = mean;
                }
            }
        }

        // 【静态初值优先】它不需要运动多样性，等到一个伸直帧就闭式解完。
        // 球面拟合仍然继续跑，但它现在是在一个好初值上精修，而不是从群体
        // 均值瞎摸 —— 这正是向导两步法的结构。
        tryStaticSeed(r);

        if (cfg_.fitAnchors || cfg_.fitLengths) {
            for (int f = 0; f < 5; ++f) {
                const int pp = 5 + 3*f, mp = pp + 1, dp = pp + 2;
                if (cfg_.fitAnchors && r.markers[size_t(pp)].observed &&
                    int(anchorSamp_[size_t(f)].size()) < cfg_.anchorMaxSamples) {
                    const Vec3 d = sub(r.markers[size_t(pp)].posWorld, r.wristT);
                    anchorSamp_[size_t(f)].push_back(matVecT(r.wristR, d));
                }
                // 中立位手指方向样本：伸得最直的那一帧，anchor -> 远节球的方向
                // 就是解剖学 +X。用"最长"筛伸直，因为弯曲只会让这个距离变短。
                if (f >= 1 && r.markers[size_t(dp)].observed && res_.anchorFitted[size_t(f)]) {
                    const Vec3 v = sub(matVecT(r.wristR, sub(r.markers[size_t(dp)].posWorld, r.wristT)),
                                       res_.anchorsMm[size_t(f)]);
                    const double L = norm(v);
                    if (L > extBestLen_[size_t(f)]) {
                        extBestLen_[size_t(f)] = L;
                        extBestDir_[size_t(f)] = mul(v, 1.0 / std::max(L, 1e-9));
                    }
                }
                if (cfg_.fitLengths) {
                    // 伸直时相邻两球连线最长（弯曲只会让它变短），所以跨帧取
                    // 分位上界就是"这个人伸直时的跨距"。用 p98 而不是 max，
                    // 免得一次三角化野点把骨长顶上去。
                    if (r.markers[size_t(pp)].observed && r.markers[size_t(mp)].observed)
                        pushSpan(spanHist_[size_t(f)][0],
                                 norm(sub(r.markers[size_t(mp)].posWorld, r.markers[size_t(pp)].posWorld)));
                    if (r.markers[size_t(mp)].observed && r.markers[size_t(dp)].observed)
                        pushSpan(spanHist_[size_t(f)][1],
                                 norm(sub(r.markers[size_t(dp)].posWorld, r.markers[size_t(mp)].posWorld)));
                }
            }
            if (cfg_.guidedThumb) trackThumbGuide(r);
            if (cfg_.bundleRefine) collectShot(r);
            if (++fitTick_ >= cfg_.bundlePeriod) {   // 隔几帧做一次，别每帧都拟合
                fitTick_ = 0;
                changed |= refitParams();
            }
        }

        if (changed) { rebuildNeutralMarkers(); dirty_ = true; }
        return changed;
    }

    static void pushSpan(std::vector<double>& h, double v) {
        h.push_back(v);
        if (h.size() > 600) h.erase(h.begin());
    }

    static double pct(std::vector<double> v, double q) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        const size_t i = size_t(std::clamp(q, 0.0, 1.0) * double(v.size() - 1) + 0.5);
        return v[i];
    }

    // -------------------------------------------------------------------------
    // anchor 球面拟合 + 骨长逐指缩放
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // 静态初值：等一个"手指伸直"的帧，用向导那套闭式解直接量出 anchor 和骨长。
    //
    // 【为什么要它】原来只有球面拟合 fitSphere 一条路，而它假设 pp 球绕 anchor
    // 做球面运动 —— 只有手指【大幅屈伸】时球心才可辨识。手指活动小的时候，
    // 一堆近似共面的点能拟合出无数个球心，解会飘到几十毫米外。真机上
    // IK 残差 18~40mm 就是这么来的。
    //
    // 而向导的静态标定完全不需要运动：手摊平时三颗球近似共线，骨长由两个
    // 距离方程闭式解出，第三个自由度用模板比例先验闭合。向导实测 30 帧
    // anchor 误差 1.21mm、骨长 2.54mm —— 而且 223 帧 1.43mm、400 帧 1.44mm，
    // 【帧数多了反而略差】。关键不是样本量，是姿势质量。
    //
    // 所以隐形标定的正确形态不是"猛攒样本"，是【等一个好帧】：用户正常用手时
    // 手摊平是必然出现的，等到了就闭式解一次，等不到就继续用群体均值。
    // 共线偏差(rmsMm)天然就是"手放得平不平"的质量判据，不用另造。
    // -------------------------------------------------------------------------
    void tryStaticSeed(const SkeletonFrameResult& r) {
        if (!cfg_.staticSeed || !r.wristPoseValid) return;
        for (int f = 0; f < 5; ++f) {
            if (staticSeeded_[size_t(f)]) continue;          // 每根只种一次
            const int pp = 5 + 3*f;
            if (!r.markers[size_t(pp)].observed ||
                !r.markers[size_t(pp+1)].observed ||
                !r.markers[size_t(pp+2)].observed) continue; // 三颗球必须都在
            // 转到腕部系 —— anchor/lengths 就是在这个系里定义的
            auto loc = [&](int m) {
                const Vec3 d = sub(r.markers[size_t(m)].posWorld, r.wristT);
                const Vec3 v = matVecT(r.wristR, d);
                return std::array<double,3>{v[0], v[1], v[2]};
            };
            const auto res = calibrateFingerFromStaticPose(
                loc(pp), loc(pp+1), loc(pp+2), fingerParam(f), cfg_.staticSeedMaxDevMm);
            if (!res.valid) continue;                        // 手指是弯的，前提不成立
            res_.anchorsMm[size_t(f)] = Vec3{res.param.anchor[0], res.param.anchor[1],
                                             res.param.anchor[2]};
            res_.lengthsMm[size_t(f)] = res.param.lengths;
            res_.anchorFitted[size_t(f)] = true;
            res_.anchorResidMm[size_t(f)] = res.rmsMm;
            staticSeeded_[size_t(f)] = true;
            staticSeedDevMm_[size_t(f)] = res.rmsMm;
            res_.staticSeeded[size_t(f)] = true;
            res_.staticSeedDevMm[size_t(f)] = res.rmsMm;
            dirty_ = true;
        }
    }

    bool refitParams() {
        bool changed = false;
        for (int f = 0; f < 5; ++f) res_.anchorSamples[size_t(f)] = int(anchorSamp_[size_t(f)].size());
        for (int f = 0; f < 5; ++f) {
            // ---- anchor：球面拟合 ----
            if (cfg_.fitAnchors && int(anchorSamp_[size_t(f)].size()) >= cfg_.anchorMinSamples) {
                Vec3 c{}; double rad = 0, resid = 0;
                if (fitSphere(anchorSamp_[size_t(f)], res_.anchorsMm[size_t(f)], c, rad, resid) &&
                    spreadDeg(anchorSamp_[size_t(f)], c) >= cfg_.anchorMinSpreadDeg) {
                    // 拟合出的球心跟起点差太远，多半是标签串了，宁可不要
                    // 【这里【不】拿拇指覆盖度当硬门】我试过：拇指覆盖度不达标
                    // 就不落 anchor，结果 nAnchorFitted() 迟迟到不了 4，
                    // tryAnatomicalAlign / 束调整整条链一起被卡住，四指也跟着
                    // 退化（实测四指 anchor 均值 3.6mm -> 12.2mm）。
                    // 为一根手指把另外四根拖下水不划算。覆盖度只做【提示】，
                    // 以及 thumbAxialK 能不能拟合的可辨识性判据。
                    const bool covOk = true;
                    if (covOk && norm(sub(c, res_.anchorsMm[size_t(f)])) < 40.0 && anchorPlausible(f, c)) {
                        res_.anchorsMm[size_t(f)] = c;
                        rigidStat_[size_t(f)] = {};   // anchor 变了，统计重来
                        res_.anchorResidMm[size_t(f)] = resid;
                        res_.anchorSpreadDeg[size_t(f)] = spreadDeg(anchorSamp_[size_t(f)], c);
                        if (!res_.anchorFitted[size_t(f)]) changed = true;
                        res_.anchorFitted[size_t(f)] = true;
                        changed = true;
                    }
                }
            }
            // ---- 骨长 ----
            // 【这里【故意】不做基于"伸直时相邻两球跨距"的闭式估计】
            // 那条路我实现过并实测了：它系统性偏大 ~30%，五指全被钳到上界
            // （食指估 52.0mm / 真值 39.2mm）。原因是那个推导要求"伸直时两球
            // 连线纯轴向"，而 kRadialMm 逐节不同、皮肤滑移随角度变，径向分量
            // 并不抵消；再加上跨距取的是分位上界，少量错标点就把它顶上去。
            // 更要命的是错误的骨长会顺着 sig 归一化和球面拟合先验往回污染
            // anchor。所以骨长【只由束调整解】，初值就用群体比例 × 手长缩放。
        }
        if (nAnchorFitted() >= 4 && !res_.frameAnatomical) {
            if (tryAnatomicalAlign()) changed = true;
        }
        if (cfg_.bundleRefine && res_.frameAnatomical &&
            int(shots_.size()) >= cfg_.bundleMinShots) {
            const int n = int(shots_.size());
            const bool ready = (stage_ == AutoCalibStage::Ready);
            if (!bundleStalled_ && !bundleJob_.f.valid() &&
                (!ready || n >= lastBundleShots_ + cfg_.bundleRefitGrowth)) {
                lastBundleShots_ = n;
                // 【放后台跑，不阻塞出帧】束调整单次 Debug 下 280ms，直接跑在
                // 推理线程上就是骨架冻住 280ms、点云还在 120fps 走 —— 用户看到的
                // "困在原地然后瞬移"。
                // 拷一份自己丢给后台线程（整个对象才 2KB），期间照常用【上一次的
                // 结果】出帧；算完了再把拟合出来的参数换进来。对下游是无缝的。
                Hm20AutoCalib snap = *this;    // 拷贝构造已跳过 future
                bundleBefore_ = res_.bundleRmseMm;
                bundleT0_ = std::chrono::steady_clock::now();
                bundleJob_.f = std::async(std::launch::async,
                    [s = std::move(snap)]() mutable {
                        s.bundleRefine();
                        return s.res_;
                    });
            }
        }
        if (nAnchorFitted() >= 4 && res_.frameAnatomical && ikUsable() &&
            stage_ == AutoCalibStage::Locked) {
            stage_ = AutoCalibStage::Ready;
            changed = true;
        }
        return changed;
    }

    // -------------------------------------------------------------------------
    // 把局部系旋到【解剖学腕部系】
    //
    // 【为什么非做不可】tryFreeze() 里那套轴（手背质心->四指近节质心 当 X，
    // 手背质心->拇指根 当 Y）是"能用的一个系"，但它跟 Hm20IkRefiner 的
    // markerFK 假设的系差了 20~40°：markerFK 里零关节角时骨头沿 +X、贴球的
    // 径向偏置沿 +Z(手背外法向)。系一歪，零位就歪，LM 只能拿关节角去补，
    // 而关节角有限位补不动 —— 实测 IK 残差 1.6mm -> 10.2mm，骨轴中位
    // 12.75° -> 20.72°，比不开 IK 还差。这就是"自标定 + IK"一开始不work的
    // 全部原因，跟标定精度无关，是坐标系约定没对上。
    //
    // 【怎么定这个系】不猜，用物理事实：
    //   +X : anchor -> 远节球，取该指伸得最直的那一帧。这【就是】markerFK
    //        零关节角时的骨轴方向，定义上完全对应。四指取平均。
    //   +Y : 食指 anchor -> 小指 anchor 的反向（指根连线，恒指向拇指侧），
    //        对 X 正交化。
    //   +Z : X × Y。手背法向由构造保证（Y 在 tryFreeze 里已按手性统一过）。
    // 原点不用管：markerFK 显式吃 anchor，原点在哪都等价，只有【朝向】要对。
    // -------------------------------------------------------------------------
    bool tryAnatomicalAlign() {
        Vec3 X{0, 0, 0}; int nx = 0;
        for (int f = 1; f < 5; ++f) {
            if (extBestLen_[size_t(f)] < 20.0) continue;   // 这根指没伸直过
            X = add(X, extBestDir_[size_t(f)]);
            ++nx;
        }
        if (nx < 3) return false;
        X = mul(X, 1.0 / nx);
        if (norm(X) < 0.6) return false;                   // 四指方向彼此不一致，不可信
        X = normalize(X);

        if (!res_.anchorFitted[1] || !res_.anchorFitted[4]) return false;
        Vec3 Y = sub(res_.anchorsMm[1], res_.anchorsMm[4]);   // 小指 -> 食指 = 拇指侧
        Y = sub(Y, mul(X, dot(Y, X)));
        if (norm(Y) < 1e-6) return false;
        Y = normalize(Y);
        const Vec3 Z = cross(X, Y);

        auto rot = [&](const Vec3& p) { return Vec3{dot(p, X), dot(p, Y), dot(p, Z)}; };
        for (int i = 0; i < 5; ++i) res_.backMm[size_t(i)] = rot(res_.backMm[size_t(i)]);
        for (int f = 0; f < 5; ++f) res_.anchorsMm[size_t(f)] = rot(res_.anchorsMm[size_t(f)]);
        for (auto& v : anchorSamp_) for (auto& p : v) p = rot(p);
        // 姿势库也要跟着转 —— 库里存的是【局部系】坐标，系变了它们就得变，
        // 否则束调整会拿旧系的观测去拟合新系的模型，直接发散。
        for (auto& sh : shots_) for (auto& q : sh.p) q = rot(q);
        lockedRef_ = res_.backMm;
        res_.frameAnatomical = true;
        return true;
    }


    // -------------------------------------------------------------------------
    // 拇指引导动作的识别
    //
    // 【不需要知道关节角就能判】三个动作各自有一个直接可测的判据：
    //   外展伸直 : 拇指远节球到手背质心的距离达到最大档，且离食指远节球最远
    //   触食指   : |拇指dp - 食指dp| 小于阈值
    //   触小指   : |拇指dp - 小指dp| 小于阈值
    // 全是欧氏距离，不依赖任何还没标定好的参数 —— 这一点很重要，否则就成了
    // "要先标定才能标定"的循环依赖。
    // 每个动作要连续保持若干帧，避免路过时被误判成"做到了"。
    // -------------------------------------------------------------------------
    void trackThumbGuide(const SkeletonFrameResult& r) {
        if (!r.wristPoseValid) return;
        const int tpp = 5, tdp = 7;

        // 【判据为什么不是"指尖相触"】第一版用的是"拇指指尖碰食指指尖"，
        // 直接测两个 marker 的距离。这在标记式系统里【原理上就行不通】：
        // 指尖真碰上时两颗球相距只有几毫米，前端会把它们并成一个光斑
        // （本系统的合并阈值 18mm），于是那两个 marker 永远不会同时可见，
        // 判据恒不成立。实测卡在第 1 步过不去，就是这个原因。
        // 换成【非接触的覆盖度】：拇指近节球绕 anchor 的方向张角 + 远节球在
        // 腕部系里走过的空间范围。两者都只需要拇指自己的点，不依赖任何
        // 会跟别的球撞在一起的动作。
        if (r.markers[size_t(tpp)].observed) {
            const Vec3 v = sub(matVecT(r.wristR, sub(r.markers[size_t(tpp)].posWorld, r.wristT)),
                               res_.anchorsMm[0]);
            const double n = norm(v);
            if (n > 5.0) {
                const Vec3 d = mul(v, 1.0 / n);
                thumbDirAcc_ = add(thumbDirAcc_, d);
                ++thumbDirN_;
            }
        }
        if (r.markers[size_t(tdp)].observed) {
            const Vec3 p = matVecT(r.wristR, sub(r.markers[size_t(tdp)].posWorld, r.wristT));
            for (int i = 0; i < 3; ++i) {
                thumbDpLo_[size_t(i)] = thumbDpN_ ? std::min(thumbDpLo_[size_t(i)], p[size_t(i)]) : p[size_t(i)];
                thumbDpHi_[size_t(i)] = thumbDpN_ ? std::max(thumbDpHi_[size_t(i)], p[size_t(i)]) : p[size_t(i)];
            }
            ++thumbDpN_;
        }
    }

    // 拇指近节球方向的等效锥角（度）。R 越小说明方向越发散、可辨识性越好。
    double thumbSpreadDeg() const {
        if (thumbDirN_ < 20) return 0.0;
        const double R = std::clamp(norm(mul(thumbDirAcc_, 1.0 / thumbDirN_)), 0.0, 1.0);
        return std::acos(R) * 180.0 / 3.14159265358979323846;
    }
    // 远节球在腕部系里走过的对角线长度（mm）。外展+对掌会把它拉得很大。
    double thumbDpRangeMm() const {
        if (thumbDpN_ < 20) return 0.0;
        return norm(sub(thumbDpHi_, thumbDpLo_));
    }

    // =========================================================================
    // 束调整式手指标定
    //
    // 【为什么球面拟合还不够】近节球绕 MCP 转，轨迹是个球面，所以球心=anchor
    // 能解 —— 这一步把 anchor 从群体均值的 10~20mm 误差压到 5~10mm，够让
    // 局部系对齐到解剖学系，但【还不够开 IK】。原因有两个：
    //   · 皮肤滑移让半径随屈曲角变（u = uRatio*L - kSkinDu*θ），球面拟合把这
    //     部分当噪声吸收，球心被系统性拉偏；
    //   · 球面拟合只用了近节那一个球，中节/远节两个球的信息整个浪费了，而
    //     骨长恰恰主要由它们决定。
    //
    // 【束调整怎么做】这是动捕标定里的标准做法：把"每帧的关节角"当作待求的
    // 冗余变量一起解，交替固定一边解另一边 ——
    //   内层：参数固定，逐帧逐指解 4 个关节角（直接调 Hm20IkRefiner::solve，
    //         跟运行时【完全同一份求解器】，不是另写一套近似）
    //   外层：关节角固定，解 anchor(3) + 骨长(3) + 全局朝向修正(3)
    // 关键在于：目标函数就是 IK 运行时最小化的那个 marker 残差。所以标出来的
    // 参数是"让 IK 拟合得最好的参数"，而不是"让某个几何假设最自洽的参数"，
    // 两者在有模型误差时并不等价 —— 前者才是我们真正要的。
    //
    // 【姿势库】必须是【不同姿势】的帧。60 帧连续的握拳对 anchor 没有额外信息，
    // 反而会让法方程病态。所以入库要过多样性门（各指弯曲程度构成的签名向量，
    // 跟库里所有帧的距离都要够大）。
    // =========================================================================
    void collectShot(const SkeletonFrameResult& r) {
        // 姿势签名：各指 anchor->远节球的归一化距离，直接反映弯曲程度
        std::array<double, 5> sig{};
        int nf = 0;
        for (int f = 0; f < 5; ++f) {
            const int dp = 5 + 3*f + 2;
            double tot = 0;
            for (int j = 0; j < 3; ++j) tot += res_.lengthsMm[size_t(f)][size_t(j)];
            if (r.markers[size_t(dp)].observed && tot > 1.0) {
                const Vec3 v = sub(matVecT(r.wristR, sub(r.markers[size_t(dp)].posWorld, r.wristT)),
                                   res_.anchorsMm[size_t(f)]);
                sig[size_t(f)] = norm(v) / tot;
                ++nf;
            } else {
                sig[size_t(f)] = -1.0;   // 缺测，不参与比距离
            }
        }
        if (nf < 3) return;

        double dmin = 1e9;
        for (const auto& sh : shots_) {
            double d = 0; int nc = 0;
            for (int f = 0; f < 5; ++f) {
                if (sig[size_t(f)] < 0 || sh.sig[size_t(f)] < 0) continue;
                const double e = sig[size_t(f)] - sh.sig[size_t(f)];
                d += e * e; ++nc;
            }
            if (nc >= 2) dmin = std::min(dmin, std::sqrt(d / nc));
        }
        // 【先填满、再挑剔】一上来就卡多样性的话，轨迹姿势重复度稍高就永远
        // 攒不够 16 张（实测只攒到 4~8 张，束调整从头到尾没跑起来）。
        // 正确做法是先无条件填到最小规模，之后再用多样性做替换式维护。
        if (int(shots_.size()) >= cfg_.bundleMinShots && dmin < cfg_.bundleDiversity) return;

        Shot sh{};
        sh.sig = sig;
        for (int m = 0; m < 20; ++m) {
            sh.ok[size_t(m)] = r.markers[size_t(m)].observed;
            if (sh.ok[size_t(m)])
                sh.p[size_t(m)] = matVecT(r.wristR, sub(r.markers[size_t(m)].posWorld, r.wristT));
        }
        if (int(shots_.size()) >= cfg_.bundleShots) {
            // 库满了就替换掉"最像新来这张"的那一张，保持多样性而不是先进先出
            size_t worst = 0; double best = 1e18;
            for (size_t i = 0; i < shots_.size(); ++i) {
                double d = 0; int nc = 0;
                for (int f = 0; f < 5; ++f) {
                    if (sig[size_t(f)] < 0 || shots_[i].sig[size_t(f)] < 0) continue;
                    const double e = sig[size_t(f)] - shots_[i].sig[size_t(f)];
                    d += e * e; ++nc;
                }
                if (nc >= 2 && d / nc < best) { best = d / nc; worst = i; }
            }
            shots_[worst] = sh;
        } else {
            shots_.push_back(sh);
        }
        res_.bundleShots = int(shots_.size());
    }

    // 给定当前参数，逐帧解关节角并返回总残差平方和 / 观测数
    double bundleSolveAngles(Hm20IkRefiner& ik, std::vector<PerShotAngles>& ang) const {
        ang.resize(shots_.size());
        double se = 0; int n = 0;
        Hm20MarkerModel mm; mm.handLenMm = res_.handLenMm; mm.thumbAxialK = res_.thumbAxialK;
        mm.thumbPronation0 = res_.thumbPronation0;
        for (size_t i = 0; i < shots_.size(); ++i) {
            std::array<Vec3, 20> obs = shots_[i].p;
            if (!res_.isRight) for (auto& v : obs) v[1] = -v[1];   // 统一到右手系再解
            const auto R = ik.solve(obs, shots_[i].ok, ang[i].q, res_.backMm);
            ang[i].q = R.angles;
            for (int f = 0; f < 5; ++f) {
                const auto fk = markerFK(f, res_.anchorsMm[size_t(f)], res_.lengthsMm[size_t(f)],
                                         R.angles[size_t(f)], mm);
                for (int j = 0; j < 3; ++j) {
                    const int m = 5 + 3*f + j;
                    if (!shots_[i].ok[size_t(m)]) continue;
                    const Vec3 d = sub(fk.marker[size_t(j)], obs[size_t(m)]);
                    se += dot(d, d); ++n;
                }
            }
        }
        return (n > 0) ? std::sqrt(se / n) : -1.0;
    }

    bool bundleRefine() {
        Hm20IkRefiner ik;
        ik.setHandedness(true);        // shots 已经镜像成右手系
        ik.setThumbAxialK(res_.thumbAxialK);
        ik.setThumbPronation0(res_.thumbPronation0);
        ik.setBackTemplate(res_.backMm);
        for (int f = 0; f < 5; ++f)
            ik.setFingerParams(f, res_.anchorsMm[size_t(f)], res_.lengthsMm[size_t(f)]);

        std::vector<PerShotAngles> ang;
        double rmse0 = bundleSolveAngles(ik, ang);
        if (rmse0 < 0) return false;

        const auto anchor0 = res_.anchorsMm;
        const auto len0 = res_.lengthsMm;
        const double axial0 = res_.thumbAxialK;
        const double pron0   = res_.thumbPronation0;
        Hm20MarkerModel mm; mm.handLenMm = res_.handLenMm; mm.thumbAxialK = res_.thumbAxialK;
        mm.thumbPronation0 = res_.thumbPronation0;

        for (int it = 0; it < cfg_.bundleIters; ++it) {
            // ---- (a) anchor 闭式解 ----
            // markerFK 里 anchor 是【纯平移】地进入 marker 位置的，所以固定关节角
            // 之后，anchor 的最小二乘解就是残差均值，不需要迭代。
            for (int f = 0; f < 5; ++f) {
                Vec3 acc{0, 0, 0}; int n = 0;
                for (size_t i = 0; i < shots_.size(); ++i) {
                    const auto fk = markerFK(f, Vec3{0,0,0}, res_.lengthsMm[size_t(f)],
                                             ang[i].q[size_t(f)], mm);
                    for (int j = 0; j < 3; ++j) {
                        const int m = 5 + 3*f + j;
                        if (!shots_[i].ok[size_t(m)]) continue;
                        Vec3 o = shots_[i].p[size_t(m)];
                        if (!res_.isRight) o[1] = -o[1];
                        acc = add(acc, sub(o, fk.marker[size_t(j)]));
                        ++n;
                    }
                }
                if (n >= 6) {
                    Vec3 a = mul(acc, 1.0 / n);
                    // 别让它一步跑太远；同时留一点先验，防止某指长期只有 1 个点可见
                    const Vec3 d = sub(a, res_.anchorsMm[size_t(f)]);
                    const double dn = norm(d);
                    if (dn > 8.0) a = add(res_.anchorsMm[size_t(f)], mul(d, 8.0 / dn));
                    res_.anchorsMm[size_t(f)] = a;
                }
            }
            // ---- (b) 骨长：3 参数 Gauss-Newton（数值 Jacobian）----
            for (int f = 0; f < 5; ++f) refineLengths(f, ang, mm);
            // ---- (c) 全局朝向修正：3 参数 GN ----
            refineFrameDelta(ang, mm);
            // ---- (d) 拇指轴向旋前耦合：1 参数，直接网格搜 ----
            // 只有 1 个未知数、区间已知且很窄，网格搜比 GN 更省事也更稳
            // （残差对它并非处处可微得漂亮，外展角小的样本几乎没有梯度）。
            // 覆盖度是 thumbAxialK 的【可辨识性】前提，不是精度偏好：
            // 耦合项 = k * 外展角，外展角一直很小的话这一项恒接近 0，
            // 此时任何 k 的残差都差不多，网格搜出来的是噪声。
            // ---- (d0) 拇指常数旋前：先搜它，再搜耦合系数 ----
            // 【顺序不能反】常数基线的量级(~80°)比耦合项(~5°)大一个数量级。
            // 基线没补上时，耦合项的网格搜是在一个整体偏了 70° 的残差面上找
            // 极小值，找到的是噪声。先定基线，耦合项才有意义。
            if (std::fabs(cfg_.lockedThumbPronation0) > 1e-6) {
                res_.thumbPronation0 = cfg_.lockedThumbPronation0;
                res_.thumbPronationFitted = true;
                mm.thumbPronation0 = res_.thumbPronation0;
            } else if (cfg_.fitThumbPronation0 && !pronLocked_
                       && pronTries_ < cfg_.thumbPronMaxTries) {
                ++pronTries_;
                refineThumbPronation(ang, mm);
                mm.thumbPronation0 = res_.thumbPronation0;
                // 碗底够明显就锁；搜满次数也锁（拿当前最好的，不再烧）
                if ((res_.thumbPronationFitted && res_.thumbPronationContrast >= 3.0)
                    || pronTries_ >= cfg_.thumbPronMaxTries)
                    pronLocked_ = true;
            } else {
                mm.thumbPronation0 = res_.thumbPronation0;   // 已锁，直接用
            }
            // ---- (d) 拇指轴向旋前耦合 ----
            if (cfg_.fitThumbAxialK && thumbCoverage() >= 0.6) {
                refineThumbAxial(ang, mm);
                mm.thumbAxialK = res_.thumbAxialK;
            }
        }

        for (int f = 0; f < 5; ++f)
            ik.setFingerParams(f, res_.anchorsMm[size_t(f)], res_.lengthsMm[size_t(f)]);
        ik.setThumbAxialK(res_.thumbAxialK);
        ik.setThumbPronation0(res_.thumbPronation0);
        ik.setBackTemplate(res_.backMm);
        std::vector<PerShotAngles> ang2 = ang;
        const double rmse1 = bundleSolveAngles(ik, ang2);

        // 【只接受变好的更新】束调整在观测退化时可能把参数带跑，必须有这道闸。
        if (rmse1 < 0 || rmse1 > rmse0 * 1.02) {
            res_.anchorsMm = anchor0;
            res_.lengthsMm = len0;
            res_.thumbAxialK = axial0;
            res_.thumbPronation0 = pron0;
            res_.bundleRmseMm = rmse0;
            return false;
        }
        res_.bundleRmseMm = rmse1;
        // 【束调整成功 != anchor 合理】原来这里无条件把五指 anchorFitted 全置
        // true，完全不检查解出来的 anchor 在不在合理位置。
        //
        // 实测（刚体运动协议那份录制）：手整体移动但手指不屈伸，anchor 沿骨轴
        // 方向【不可辨识】，束调整照样"收敛"（bundleRmse 30.9），把五指 anchor
        // 全解到了手背质心的【反方向】—— anchor.x 是 -53/-40/-30/-5，而近节球
        // 在 +84/+93/+86/+72，anchor 到近节球的距离 100~154mm（正常 36~50mm）。
        // 下游 verifyAnchors 把它整体判死（nOk=0/4），于是 anchorsValid_=false，
        // 平面永不学、链式续解 100% 失效、遮挡点 87.6% 回落到网络原始预测，
        // 43.5% 的帧出现两个 marker 重合 —— 就是你截图里 idp/imp 飞到手背上方、
        // bk0 和 mpp 挤在一起的那个形态。
        //
        // 球面拟合那一支本来就有 anchorPlausible + spreadDeg 双重门控，
        // 这一支却没有。补上同一道门：任何一根不合理就整体不认，宁可退回
        // 上一组 anchor（那组至少通过过检查），也不要把坏解写进去。
        {
            bool allOk = true;
            for (int f = 0; f < 5 && allOk; ++f)
                if (!anchorPlausible(f, res_.anchorsMm[size_t(f)])) allOk = false;
            if (!allOk) {
                res_.anchorsMm = anchor0;
                res_.lengthsMm = len0;
                res_.thumbAxialK = axial0;
                res_.thumbPronation0 = pron0;
                res_.bundleRmseMm = rmse0;
                reject_ = AutoCalibReject::AnchorImplausible;
                return false;
            }
        }
        for (int f = 0; f < 5; ++f) {
            res_.anchorFitted[size_t(f)] = true;
            res_.lengthFitted[size_t(f)] = true;
        }
        return true;
    }

    // -------------------------------------------------------------------------
    // 拇指【常数】解剖旋前的一维搜索。
    //
    // 【为什么必须搜、不能填死】幅值能从解剖学推(80~90°)，也能从模型里量出来
    // (tools/hm20_diag/probe_thumb_axis.py 在 hm20_v7 上量到的缺口是 73°)，
    // 但【符号】取决于 rotX/rotY/rotZ 的约定、行主序、以及 refine() 内部镜像的
    // 实现细节 —— 任何一处读错就是相反的结论，而符号错了会把 73° 的缺口变成
    // 163°，比不补还差。所以只能让残差说话。
    //
    // 【判据是可观测量】这个参数作用在 marker 的【位置】上，位置是测得到的。
    // 对比 thumbRollOffsetRad：那个作用在 roll(绕骨轴自转)上，而 roll 在每节
    // 只有 1 颗球时【不可观测】—— 那个参数永远没有数字能告诉你调对没有。
    //
    // 【一定要看对比度】真解会有明显的碗底。曲线平(contrast<2)说明这批 shot 里
    // 拇指没怎么动，此时不写入结果 —— 宁可保持 0(等价旧行为)也不要写进一个噪声。
    // -------------------------------------------------------------------------
    void refineThumbPronation(const std::vector<PerShotAngles>& ang, Hm20MarkerModel mm) {
        auto cost = [&](double a) {
            mm.thumbPronation0 = a;
            double se = 0; int n = 0;
            for (size_t i = 0; i < shots_.size(); ++i) {
                const auto fk = markerFK(0, res_.anchorsMm[0], res_.lengthsMm[0],
                                         ang[i].q[0], mm);
                for (int j = 0; j < 3; ++j) {
                    const int m = 5 + j;
                    if (!shots_[i].ok[size_t(m)]) continue;
                    Vec3 o = shots_[i].p[size_t(m)];
                    if (!res_.isRight) o[1] = -o[1];
                    const Vec3 d = sub(fk.marker[size_t(j)], o);
                    se += dot(d, d); ++n;
                }
            }
            return (n >= 18) ? se / n : 1e18;
        };
        double bestA = 0.0, bestC = cost(0.0), worstC = bestC;
        if (bestC > 1e17) return;
        for (int i = 0; i <= 36; ++i) {               // 粗搜 [-1.8, 1.8]，覆盖 ±103°
            const double a = -1.80 + 0.10 * i;
            const double c = cost(a);
            if (c > 1e17) continue;
            if (c < bestC) { bestC = c; bestA = a; }
            if (c > worstC) worstC = c;
        }
        for (int i = -5; i <= 5; ++i) {               // 精搜 ±0.10 步长 0.02
            const double a = bestA + 0.02 * i;
            if (a < -2.0 || a > 2.0) continue;
            const double c = cost(a);
            if (c < 1e17 && c < bestC) { bestC = c; bestA = a; }
        }
        // cost 是均方，对比度要用 RMS 之比才有物理意义
        res_.thumbPronationContrast = (bestC > 1e-9) ? std::sqrt(worstC / bestC) : 0.0;
        res_.thumbPronationLocked = pronLocked_;
        if (res_.thumbPronationContrast >= 2.0) {
            res_.thumbPronation0 = bestA;
            res_.thumbPronationFitted = true;
        } else {
            res_.thumbPronation0 = 0.0;               // 曲线平：不写，保持旧行为
            res_.thumbPronationFitted = false;
        }
    }

    // 拇指轴向旋前耦合系数的一维搜索。
    void refineThumbAxial(const std::vector<PerShotAngles>& ang, Hm20MarkerModel mm) {
        auto cost = [&](double k) {
            mm.thumbAxialK = k;
            double se = 0; int n = 0;
            for (size_t i = 0; i < shots_.size(); ++i) {
                const auto fk = markerFK(0, res_.anchorsMm[0], res_.lengthsMm[0],
                                         ang[i].q[0], mm);
                for (int j = 0; j < 3; ++j) {
                    const int m = 5 + j;
                    if (!shots_[i].ok[size_t(m)]) continue;
                    Vec3 o = shots_[i].p[size_t(m)];
                    if (!res_.isRight) o[1] = -o[1];
                    const Vec3 d = sub(fk.marker[size_t(j)], o);
                    se += dot(d, d); ++n;
                }
            }
            return (n >= 12) ? se / n : 1e18;
        };
        double bestK = res_.thumbAxialK, bestC = cost(bestK);
        if (bestC > 1e17) return;
        for (int i = 0; i <= 24; ++i) {              // 0.30 .. 0.90, 步长 0.025
            const double k = 0.30 + 0.025 * i;
            const double c = cost(k);
            if (c < bestC) { bestC = c; bestK = k; }
        }
        if (std::fabs(bestK - res_.thumbAxialK) > 1e-9) {
            res_.thumbAxialK = bestK;
            res_.thumbAxialFitted = true;
        }
    }

    void refineLengths(int f, const std::vector<PerShotAngles>& ang, const Hm20MarkerModel& mm) {
        auto cost = [&](const std::array<double, 3>& L, std::vector<double>* res) {
            double se = 0;
            for (size_t i = 0; i < shots_.size(); ++i) {
                const auto fk = markerFK(f, res_.anchorsMm[size_t(f)], L, ang[i].q[size_t(f)], mm);
                for (int j = 0; j < 3; ++j) {
                    const int m = 5 + 3*f + j;
                    if (!shots_[i].ok[size_t(m)]) continue;
                    Vec3 o = shots_[i].p[size_t(m)];
                    if (!res_.isRight) o[1] = -o[1];
                    const Vec3 d = sub(fk.marker[size_t(j)], o);
                    if (res) { res->push_back(d[0]); res->push_back(d[1]); res->push_back(d[2]); }
                    se += dot(d, d);
                }
            }
            return se;
        };
        std::array<double, 3> L = res_.lengthsMm[size_t(f)];
        std::vector<double> r0;
        const double c0 = cost(L, &r0);
        if (r0.size() < 30) return;

        const double eps = 0.25;   // mm
        std::vector<std::vector<double>> J(3);
        for (int k = 0; k < 3; ++k) {
            auto Lp = L; Lp[size_t(k)] += eps;
            std::vector<double> rp;
            cost(Lp, &rp);
            if (rp.size() != r0.size()) return;
            J[size_t(k)].resize(r0.size());
            for (size_t t = 0; t < r0.size(); ++t) J[size_t(k)][t] = (rp[t] - r0[t]) / eps;
        }
        double A[3][3] = {}, b[3] = {};
        for (int a = 0; a < 3; ++a) {
            for (size_t t = 0; t < r0.size(); ++t) b[a] -= J[size_t(a)][t] * r0[t];
            for (int c = 0; c < 3; ++c)
                for (size_t t = 0; t < r0.size(); ++t) A[a][c] += J[size_t(a)][t] * J[size_t(c)][t];
        }
        // LM 阻尼 + 朝群体比例的弱正则（骨长三节之间的比例个体差异远小于绝对值）
        for (int a = 0; a < 3; ++a) A[a][a] *= 1.15;
        double dx[3];
        if (!solve3(A, b, dx)) return;
        std::array<double, 3> Ln = L;
        for (int k = 0; k < 3; ++k)
            Ln[size_t(k)] = std::clamp(L[size_t(k)] + std::clamp(dx[k], -3.0, 3.0),
                                       kLenRef()[size_t(f)][k] * 0.70,
                                       kLenRef()[size_t(f)][k] * 1.40);
        if (cost(Ln, nullptr) < c0) res_.lengthsMm[size_t(f)] = Ln;
    }

    // 全局朝向修正：解一个小旋转 δ 使 FK ≈ R(δ)·obs，然后把【局部系】整体转过去。
    // anchor 已经在模型系里了，所以只转手背模板和姿势库，不转 anchor。
    void refineFrameDelta(const std::vector<PerShotAngles>& ang, const Hm20MarkerModel& mm) {
        auto build = [&](const Vec3& w, std::vector<double>* res) {
            const Mat3 R = expSO3(w);
            double se = 0;
            for (size_t i = 0; i < shots_.size(); ++i) {
                for (int f = 0; f < 5; ++f) {
                    const auto fk = markerFK(f, res_.anchorsMm[size_t(f)],
                                             res_.lengthsMm[size_t(f)], ang[i].q[size_t(f)], mm);
                    for (int j = 0; j < 3; ++j) {
                        const int m = 5 + 3*f + j;
                        if (!shots_[i].ok[size_t(m)]) continue;
                        Vec3 o = shots_[i].p[size_t(m)];
                        if (!res_.isRight) o[1] = -o[1];
                        const Vec3 d = sub(fk.marker[size_t(j)], matVec(R, o));
                        if (res) { res->push_back(d[0]); res->push_back(d[1]); res->push_back(d[2]); }
                        se += dot(d, d);
                    }
                }
            }
            return se;
        };
        std::vector<double> r0;
        const double c0 = build(Vec3{0,0,0}, &r0);
        if (r0.size() < 60) return;
        const double eps = 1e-3;
        std::vector<std::vector<double>> J(3);
        for (int k = 0; k < 3; ++k) {
            Vec3 w{0,0,0}; w[size_t(k)] = eps;
            std::vector<double> rp;
            build(w, &rp);
            if (rp.size() != r0.size()) return;
            J[size_t(k)].resize(r0.size());
            for (size_t t = 0; t < r0.size(); ++t) J[size_t(k)][t] = (rp[t] - r0[t]) / eps;
        }
        double A[3][3] = {}, b[3] = {};
        for (int a = 0; a < 3; ++a) {
            for (size_t t = 0; t < r0.size(); ++t) b[a] -= J[size_t(a)][t] * r0[t];
            for (int c = 0; c < 3; ++c)
                for (size_t t = 0; t < r0.size(); ++t) A[a][c] += J[size_t(a)][t] * J[size_t(c)][t];
        }
        for (int a = 0; a < 3; ++a) A[a][a] *= 1.15;
        double dx[3];
        if (!solve3(A, b, dx)) return;
        Vec3 w{std::clamp(dx[0], -0.05, 0.05), std::clamp(dx[1], -0.05, 0.05),
               std::clamp(dx[2], -0.05, 0.05)};
        if (build(w, nullptr) >= c0) return;
        const Mat3 R = expSO3(w);
        for (int i = 0; i < 5; ++i) res_.backMm[size_t(i)] = matVec(R, res_.backMm[size_t(i)]);
        for (auto& sh : shots_) for (auto& p : sh.p) p = matVec(R, p);
        for (auto& v : anchorSamp_) for (auto& p : v) p = matVec(R, p);
        lockedRef_ = res_.backMm;
    }

    static Mat3 expSO3(const Vec3& w) {
        const double t = norm(w);
        if (t < 1e-12) return Mat3{1,0,0, 0,1,0, 0,0,1};
        const Vec3 a = mul(w, 1.0 / t);
        const double c = std::cos(t), s = std::sin(t), C = 1 - c;
        return Mat3{c + a[0]*a[0]*C,      a[0]*a[1]*C - a[2]*s, a[0]*a[2]*C + a[1]*s,
                    a[1]*a[0]*C + a[2]*s, c + a[1]*a[1]*C,      a[1]*a[2]*C - a[0]*s,
                    a[2]*a[0]*C - a[1]*s, a[2]*a[1]*C + a[0]*s, c + a[2]*a[2]*C};
    }

    static bool solve3(double A[3][3], const double b[3], double x[3]) {
        double M[3][4];
        for (int i = 0; i < 3; ++i) { for (int j = 0; j < 3; ++j) M[i][j] = A[i][j]; M[i][3] = b[i]; }
        for (int c = 0; c < 3; ++c) {
            int piv = c;
            for (int r = c + 1; r < 3; ++r) if (std::fabs(M[r][c]) > std::fabs(M[piv][c])) piv = r;
            if (std::fabs(M[piv][c]) < 1e-12) return false;
            for (int j = 0; j < 4; ++j) std::swap(M[c][j], M[piv][j]);
            for (int r = c + 1; r < 3; ++r) {
                const double m = M[r][c] / M[c][c];
                for (int j = c; j < 4; ++j) M[r][j] -= m * M[c][j];
            }
        }
        for (int i = 2; i >= 0; --i) {
            double s = M[i][3];
            for (int j = i + 1; j < 3; ++j) s -= M[i][j] * x[j];
            x[i] = s / M[i][i];
        }
        return true;
    }

    // 代数球面拟合（Pratt 的线性形式）+ 朝先验拉一点岭正则，防止姿势展布不够时
    // 球心跑飞。残差用几何残差 | |p-c| - r | 的 RMS。
    static bool fitSphere(const std::vector<Vec3>& P, const Vec3& prior,
                          Vec3& c, double& rad, double& residMm) {
        const size_t n = P.size();
        if (n < 12) return false;
        // 最小化 Σ(|p|² - 2c·p + |c|² - r²)²，令 x = (cx,cy,cz,k), k = |c|²-r²
        double A[4][4] = {}, b[4] = {};
        for (const auto& p : P) {
            const double g[4] = {-2.0*p[0], -2.0*p[1], -2.0*p[2], 1.0};
            const double y = -(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
            for (int i = 0; i < 4; ++i) {
                b[i] += g[i] * y;
                for (int j = 0; j < 4; ++j) A[i][j] += g[i] * g[j];
            }
        }
        // 岭正则：把 c 往先验拉。lam 相对于样本数取，样本越多先验影响越小。
        const double lam = 4.0 * double(n);
        for (int i = 0; i < 3; ++i) { A[i][i] += lam; b[i] += lam * prior[size_t(i)]; }

        double M[4][5];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) M[i][j] = A[i][j];
            M[i][4] = b[i];
        }
        for (int col = 0; col < 4; ++col) {
            int piv = col;
            for (int r2 = col + 1; r2 < 4; ++r2)
                if (std::fabs(M[r2][col]) > std::fabs(M[piv][col])) piv = r2;
            if (std::fabs(M[piv][col]) < 1e-9) return false;
            for (int j = 0; j < 5; ++j) std::swap(M[col][j], M[piv][j]);
            for (int r2 = col + 1; r2 < 4; ++r2) {
                const double m = M[r2][col] / M[col][col];
                for (int j = col; j < 5; ++j) M[r2][j] -= m * M[col][j];
            }
        }
        double x[4];
        for (int i = 3; i >= 0; --i) {
            double s = M[i][4];
            for (int j = i + 1; j < 4; ++j) s -= M[i][j] * x[j];
            x[i] = s / M[i][i];
        }
        c = {x[0], x[1], x[2]};
        const double r2v = dot(c, c) - x[3];
        if (!(r2v > 1.0)) return false;
        rad = std::sqrt(r2v);
        double se = 0;
        for (const auto& p : P) { const double d = norm(sub(p, c)) - rad; se += d * d; }
        residMm = std::sqrt(se / double(n));
        return residMm < 6.0;      // 拟合不上就别用
    }

    // =========================================================================
    // 手背编号规范化 —— 逐行照抄训练语料的 topology.canonical_dorsum_order()
    //
    // 【为什么必须有这一步】手背 5 点近似五重对称，网络在【还没有模板】时的
    // 标法不唯一。实测出现过整套【反向排列】：冻出来的模板半径序列跟真值正好
    // 倒过来。这类错误：
    //   · 刚体自检查不出 —— 同一组物理点换个标法，两两距离一模一样；
    //   · 靠"冻结后看 Kabsch 残差"也拦不住 —— 反向排列的五边形仍然能拟合得
    //     相当好（实测残差中位只有 3.26mm，而且我只在高质量帧上采样，
    //     统计口径又把它进一步低估，结果 461 帧就放行了）。
    //
    // 唯一可靠的是【离散量】：绕序。训练语料的编号规则写死在
    // hand_rig._sample_dorsum -> topology.canonical_dorsum_order：
    //     在手背平面内绕质心，从桡侧(拇指侧)方向起，绕手背外法向逆时针。
    // 左右手统一（生成器专门为此做过镜像处理）。反向排列会让绕序整个翻过来，
    // 这是个跟噪声无关的离散判据。
    //
    // 而且既然规则是显式的，就不止能【检测】，还能直接【重排】——把冻出来的
    // 模板按规范序重新编号，再喂给网络。网络拿到规范 tmpl 之后会依据它重新
    // 破对称，输出的标签自然就跟模板一致了。这比"检测到错误就推倒重来"强得多：
    // 推倒重来时网络还是没有模板，会再给出同一套错标法，纯粹原地打转。
    //
    // 【为什么连旋转相位也要一起归位】只翻绕序不够。编号整体循环移位一格
    // （0号落到本该是1号的位置）同样会让 packNormalized() 送进网络的条件向量
    // 偏离训练分布——tmpl 是按标签顺序展平的，移位后就是分布外输入。
    // 规范序同时定死绕向和起点，两个自由度一次解决。
    // =========================================================================
    static std::array<int, 5> canonicalDorsumOrder(const std::array<Vec3, 5>& P,
                                                   const Vec3& radialDir,
                                                   const Vec3& dorsalDir) {
        Vec3 c{0, 0, 0};
        for (const auto& p : P) c = add(c, p);
        c = mul(c, 0.2);
        const Vec3 n = normalize(dorsalDir);
        Vec3 e1 = sub(radialDir, mul(n, dot(radialDir, n)));
        e1 = normalize(e1);
        const Vec3 e2 = cross(n, e1);
        std::array<double, 5> ang{};
        std::array<int, 5> idx{0, 1, 2, 3, 4};
        for (int i = 0; i < 5; ++i) {
            const Vec3 d = sub(P[size_t(i)], c);
            double a = std::atan2(dot(d, e2), dot(d, e1));
            if (a < 0) a += 2.0 * 3.14159265358979323846;
            ang[size_t(i)] = a;
        }
        std::sort(idx.begin(), idx.end(),
                  [&](int a, int b) { return ang[size_t(a)] < ang[size_t(b)]; });
        return idx;
    }

    // 解剖学合理性闸。球面拟合在姿势展布不足时会给出一个"数值上很满意、
    // 物理上荒唐"的球心（实测某个 session 五指 anchor 平均错 49.7mm，而拟合
    // 残差看起来完全正常）。光看残差是拦不住的，必须拿人手的实际尺寸卡。
    bool anchorPlausible(int f, const Vec3& c) const {
        const double d = norm(c);                       // 到手背质心的距离
        const double k = res_.handLenMm / 185.0;
        if (f == 0) { if (d < 20.0 * k || d > 85.0 * k) return false; }   // 拇指 CMC
        else        { if (d < 25.0 * k || d > 95.0 * k) return false; }   // 四指 MCP
        // 【方向检查】——只查距离是不够的：实测出过一组 anchor，五指的距离
        // 全部落在上面的合理区间里，方向却整体反了（anchor 在手背质心的
        // 负 X 侧、近节球在正 X 侧），anchor 到近节球 100~154mm。
        // MCP 关节必须在手背质心和这根指近节球【之间】，也就是
        // anchor 到近节球的距离要明显小于质心到近节球的距离，
        // 且 anchor 大致朝着近节球那一侧。
        // 【用 res_.markersMm 的中立位近节球】它跟 anchorsMm 同在腕部系，
        // 且质心已被规范到原点（见 rebuildNeutralMarkers）。
        {
            const Vec3 pp = res_.markersMm[size_t(5 + f * 3)];   // 该指近节球
            const double dPp = norm(pp);
            if (dPp > 1.0) {
                const double bone = norm(sub(pp, c));     // anchor->近节球 = 一节近节骨
                // 近节骨长量级：12~70mm（跟 anchorsPlausibleUnused 用同一组界）
                if (bone < 12.0 * k || bone > 70.0 * k) return false;
                // anchor 必须比近节球更靠近质心，否则就是解到了反方向
                if (d >= dPp) return false;
                // 方向一致性：anchor 应当落在质心->近节球这个方向的同侧。
                // 反方向解出来的 anchor 点积为负，这一条专门抓它。
                if (dot(c, pp) <= 0.0) return false;
            }
        }
        // 跟已解出的邻指比：相邻掌骨头间距。
        // 【下限放宽到 4mm/指距】原来是 10mm，实测会误伤好数据：
        // 一组各项判据全部合格的 anchor（bone 44~50mm、方向全对），
        // 食指-中指的 anchor 间距只有 5.6mm，被 10mm 的下限拦掉。
        // 掌骨头在腕部系里本来就可以挨得很近（尤其中指/无名），
        // 这条下限的本意是抓"两指 anchor 解到同一个点"这种退化，
        // 4mm 足够抓住那种情况，又不会误伤真实的紧凑手型。
        // 【上限保持不变】它抓的是"anchor 散开到不可能的距离"，实测没有误伤。
        for (int g = 1; g < 5; ++g) {
            if (g == f || !res_.anchorFitted[size_t(g)]) continue;
            if (f == 0) continue;
            const double s2 = norm(sub(c, res_.anchorsMm[size_t(g)]));
            const double lo = 4.0 * k * std::abs(f - g), hi = 40.0 * k * std::abs(f - g);
            if (s2 < lo || s2 > hi) return false;
        }
        return true;
    }

    // 样本相对球心的方向展布（度）。太小说明手指一直没怎么动，球面拟合退化。
    static double spreadDeg(const std::vector<Vec3>& P, const Vec3& c) {
        Vec3 m{0, 0, 0};
        for (const auto& p : P) m = add(m, normalize(sub(p, c)));
        m = mul(m, 1.0 / double(P.size()));
        const double R = std::clamp(norm(m), 0.0, 1.0);
        // 球面上的平均合成向量长度 -> 等效锥角
        return std::acos(std::clamp(R, -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
    }

    int nAnchorFitted() const {
        int n = 0;
        for (int f = 0; f < 5; ++f) if (res_.anchorFitted[size_t(f)]) ++n;
        return n;
    }

    double anchorProgress() const {
        if (!cfg_.fitAnchors) return 1.0;
        double p = 0;
        for (int f = 0; f < 5; ++f)
            p += std::min(1.0, double(anchorSamp_[size_t(f)].size()) / std::max(cfg_.anchorMinSamples, 1));
        return p / 5.0;
    }

    static const double (&kLenRef())[5][3] {
        // 中立位群体均值骨长（mm），对应手长 185mm。逐指缩放的基准。
        static const double v[5][3] = {
            {40, 30, 25}, {40, 25, 20}, {45, 27, 22}, {42, 26, 21}, {33, 20, 17}};
        return v;
    }

    void seedPopulationParams() {
        // anchor 的群体均值是相对 HandModel 腕部原点的，而自举出的局部系原点在
        // 手背质心，两者差约 (43, 6, 5)mm。左手时 y 分量取反 —— 局部系的 +Y
        // 已经统一成"拇指侧"，所以这里不需要再翻（tryFreeze 里翻过 Y 了）。
        static const double kAnchorWrist[5][3] = {
            {30, 40, 5}, {85, 20, 0}, {88, 3, 0}, {85, -14, 0}, {80, -30, 0}};
        static const double kBackCtr[3] = {43.0, 6.0, 4.8};
        const double k = res_.handLenMm / 185.0;
        for (int f = 0; f < 5; ++f) {
            for (int j = 0; j < 3; ++j) {
                res_.anchorsMm[size_t(f)][size_t(j)] = (kAnchorWrist[f][j] - kBackCtr[j]) * k;
                res_.lengthsMm[size_t(f)][size_t(j)] = kLenRef()[f][j] * k;
            }
            res_.anchorFitted[size_t(f)] = false;
            res_.lengthFitted[size_t(f)] = false;
        }
    }

    // 中立位 20 点：手背用模板，手指用【跟 IK 完全同一份 FK】在零关节角下摆出来。
    //
    // 【为什么不用 Hm20TemplateAdapter 里那套"沿腕心->anchor 径向排中点"】
    // 那是个跟训练语料不一样的贴点模型（marker 在骨中点、无径向偏置），而
    // packNormalized() 送给网络的就是这 20 点的形状。形状对不上训练分布，
    // 网络看到的条件向量就是分布外的。用 markerFK(q=0) 摆出来的点跟
    // hand_rig.rest_pose_markers() 是同一个公式，形状能对上。
    void rebuildNeutralMarkers() {
        for (int i = 0; i < 5; ++i) res_.markersMm[size_t(i)] = res_.backMm[size_t(i)];
        Hm20MarkerModel mm;
        mm.handLenMm = res_.handLenMm;
        mm.thumbAxialK = res_.thumbAxialK;
        mm.thumbPronation0 = res_.thumbPronation0;
        const std::array<double, 4> zero{0, 0, 0, 0};
        for (int f = 0; f < 5; ++f) {
            const auto fk = markerFK(f, res_.anchorsMm[size_t(f)], res_.lengthsMm[size_t(f)], zero, mm);
            for (int j = 0; j < 3; ++j)
                res_.markersMm[size_t(5 + 3*f + j)] = fk.marker[size_t(j)];
        }
    }

    AutoCalibConfig cfg_;
    AutoCalibResult res_;
    AutoCalibStage stage_ = AutoCalibStage::Idle;
    // 束调整退避：上次跑它时的 shot 数。见 bundleRefitGrowth 的说明。
    int  lastBundleShots_ = -1000;
    bool pronLocked_ = false;        // 拇指常数旋前已定死，不再重搜
    int  pronTries_ = 0;             // 已经搜过几次
    int  stallRuns_ = 0;             // 连续几次 refine 没进展
    bool bundleStalled_ = false;     // 已判定停滞，彻底不再 refine
    // 后台束调整。future 不可拷贝，所以 Hm20AutoCalib 的拷贝构造会用默认值 ——
    // 这正是我们要的：拷出去的那份不该继承别人的任务。
    // 【为什么包一层】std::future 不可拷贝，直接当成员会把整个类的拷贝构造
    // 删掉；而启动后台任务时我们需要"拷一份自己丢过去"。
    // 手写逐成员的 copyFrom 会漏成员，而且漏了是静默 bug ——
    // 包一层之后编译器继续生成拷贝构造，一个成员都不会漏。
    // 拷贝语义：不继承别人的任务句柄（拷出来的那份从零开始）。
    struct BundleJob {
        std::future<AutoCalibResult> f;
        BundleJob() = default;
        BundleJob(const BundleJob&) {}
        BundleJob& operator=(const BundleJob&) { f = {}; return *this; }
        BundleJob(BundleJob&&) = default;
        BundleJob& operator=(BundleJob&&) = default;
    };
    BundleJob bundleJob_;
    std::chrono::steady_clock::time_point bundleT0_{};
    double bundleBefore_ = -1.0;
    std::deque<Sample> buf_;
    int warmupCount_ = 0;
    int driftCount_ = 0;
    int fitTick_ = 0;
    bool dirty_ = false;
    bool hasPrevPose_ = false;
    AutoCalibReject reject_ = AutoCalibReject::None;
    Vec3 prevCentroid_{};
    Quat prevQuat_{1, 0, 0, 0};
    std::array<Vec3, 5> lockedRef_{};
    double lastRmse_ = 0.0;
    double handSignAcc_ = 0.0;
    // 判据无效时用它代替 buf_ 做空遍历。成员而不是局部临时对象 ——
    // 对临时容器取引用做 range-for 是悬垂引用。
    std::deque<Sample> buf2_;
    Vec3 nDorsal_{};
    bool haveDorsal_ = false;
    int handSignN_ = 0;
    std::array<std::vector<Vec3>, 5> anchorSamp_{};
    std::array<std::array<std::vector<double>, 2>, 5> spanHist_{};
    std::array<std::array<double, 2>, 5> spanMax_{};
    std::vector<Shot> shots_;
    struct RigidStat { long n = 0; double sum = 0, sum2 = 0; };
    std::array<RigidStat, 5> rigidStat_{};
    std::array<bool, 5>   staticSeeded_{};
    std::array<double, 5> staticSeedDevMm_{};
    std::vector<double> validRmse_;
    double validZAcc_ = 0.0;   // 镜像自检：手指点在模板系里的平均 z
    int    validZN_ = 0;
    int attempts_ = 0;
    double frozenSigma_ = 0.0;
    Vec3 thumbDirAcc_{};
    int thumbDirN_ = 0;
    Vec3 thumbDpLo_{}, thumbDpHi_{};
    int thumbDpN_ = 0;
    bool exhausted_ = false;
    std::array<double, 5> extBestLen_{};      // 该指见过的最大 anchor->远节球距离
    std::array<Vec3, 5>   extBestDir_{};      // 对应的方向 = 解剖学 +X
};

}  // namespace hm20
}  // namespace mocap
