#pragma once
// ---------------------------------------------------------------------------
// IEKF 点追踪器 —— 纯数学，零 Qt 依赖，TemporalTracker 的"卡尔曼版"替代品。
//
// 【这一版重写的核心】把关联门控从"恒速预测点周围一个固定欧氏圆"升级为
// "用滤波器自己算出来的协方差做马氏/卡方门控"。旧版最大的浪费是：每条
// 轨迹每帧都算了一个 6x6 协方差(位置块 3x3 就是"我现在有多确定这个点在
// 哪"的完整椭球)，可是关联的时候把它整个扔掉、只用一个跟场景/运动无关的
// 固定 maxAssocDist。后果就是同一个固定值在两种场景各害一次：
//   · 单点快速晃动：反向甩手瞬间恒速预测朝反方向过冲，真实点落到固定圆
//     外→关联失败→重新起号。(门控太紧)
//   · 全手多点(幽灵多)：为了不丢单点而放宽的固定门控，又让乱跳的幽灵点
//     反复被关联上、活到转正，冒出一堆多余点。(门控太松)
// 单一固定门控不可能同时伺候好这两端。协方差门控天然两头都顾：滤波器
// 不确定时(刚起步/正在coast/大过程噪声下追剧烈运动)椭球大、门控自动放宽；
// 确定时椭球小、门控自动收紧——"该宽的时候宽、该紧的时候紧"由数据本身
// 决定，不再靠一个拍脑袋的固定值。
//
// 在协方差门控之上，再加两个针对性机制(都可单独开关，默认开)：
//   ① 双锚点关联(useDualAnchorGate_)：恒速模型在急动/反向时会朝错误方向
//      外推，预测点反而比"上一帧位置本身"离真实点更远。所以关联时同时拿
//      【恒速预测点】和【上一次真正测到的位置】两个锚点各算一遍马氏距离，
//      取小的那个——反向甩手时点回到上次位置附近，靠第二个锚点照样关联上，
//      不必等滤波器把速度纠正过来。代价几乎为零(多算一次3x3)，对急动鲁棒。
//   ② 机动自适应过程噪声(useManeuverAdaptiveQ_)：一条轨迹如果连续几帧
//      "预测点跟真实观测差得远"(归一化残差大)，说明它在做恒速模型解释不了
//      的机动，就临时把它的过程噪声放大(每条轨迹独立的 boost 倍率，事后
//      指数衰减回1)。过程噪声一大，下一帧协方差椭球就撑大、门控自动变宽，
//      平滑地把机动中的点重新兜住——本质是最简版的"机动检测→自适应Q"
//      (IMM 的穷人版)，不用真的跑多模型。
//
// 【为什么幽灵不会因为门控变宽而跟着混进来】马氏门控之外永远 AND 一道
// 硬欧氏上限：tentative(还没转正)的轨迹这道硬上限就是 maxAssocDist_ 本身
// (不享受任何放宽)，只有 confirmed 的真点在 coast 时才按丢帧数放宽硬上限。
// 于是"协方差撑大→门控放宽"这个好处只给已经证明过自己的真点；乱跳的幽灵
// 起的都是 tentative 轨迹，被死死摁在 maxAssocDist_ 以内、又因为每帧跳到
// 别处攒不够 minHits，转正前就死掉，不占号。①解决"该关联上的关联不上"，
// 这道硬上限解决"不该关联上的别放进来"，两件事分开治，互不牵连。
//
// 把 useMahalanobisGate_/useDualAnchorGate_/useManeuverAdaptiveQ_ 三个都
// 关掉，行为退回到跟旧版逐字节一致的"恒速预测点+固定欧氏门控"，方便对拍。
// ---------------------------------------------------------------------------
#include "estimate/PointIEKF.hpp"
#include "reconstruct/TrackFrameDebug.hpp"
#include "reconstruct/MultiViewCluster.hpp"
#include "reconstruct/TemporalTracker.hpp"   // 复用 TrackedPoint 输出结构，两个追踪器对下游暴露同一个类型
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>

namespace mocap {

namespace iekf_track_detail {

// 3x3 对称正定矩阵求逆(伴随矩阵法)。det 接近 0(退化/未标定)时返回 false，
// 调用方应回退到纯欧氏门控。只用于位置块，维度固定为3，直接展开最清楚。
inline bool invertSym3(const double M[3][3], double inv[3][3]) {
    const double a=M[0][0],b=M[0][1],c=M[0][2],
                 d=M[1][0],e=M[1][1],f=M[1][2],
                 g=M[2][0],h=M[2][1],i=M[2][2];
    const double A =  (e*i - f*h);
    const double B = -(d*i - f*g);
    const double C =  (d*h - e*g);
    const double det = a*A + b*B + c*C;
    if (std::abs(det) < 1e-18) return false;
    const double id = 1.0/det;
    inv[0][0] =  A*id;
    inv[0][1] = -(b*i - c*h)*id;
    inv[0][2] =  (b*f - c*e)*id;
    inv[1][0] =  B*id;
    inv[1][1] =  (a*i - c*g)*id;
    inv[1][2] = -(a*f - c*d)*id;
    inv[2][0] =  C*id;
    inv[2][1] = -(a*h - b*g)*id;
    inv[2][2] =  (a*e - b*d)*id;
    return true;
}

// δ^T Σ^-1 δ，3维马氏距离平方。
inline double mahaSq3(const double d[3], const double SigmaInv[3][3]) {
    double t0 = SigmaInv[0][0]*d[0] + SigmaInv[0][1]*d[1] + SigmaInv[0][2]*d[2];
    double t1 = SigmaInv[1][0]*d[0] + SigmaInv[1][1]*d[1] + SigmaInv[1][2]*d[2];
    double t2 = SigmaInv[2][0]*d[0] + SigmaInv[2][1]*d[1] + SigmaInv[2][2]*d[2];
    return d[0]*t0 + d[1]*t1 + d[2]*t2;
}

} // namespace iekf_track_detail

class IekfPointTracker {
public:
    // 上一次 update() 的逐点内部状态。见 TrackFrameDebug 的说明。
    const TrackFrameDebug& lastFrameDebug() const { return dbg_; }

    // 前三个参数跟 TemporalTracker 同名同义，直接复用同一份UI控件/QSettings。
    explicit IekfPointTracker(double maxAssocDist = 30.0, int maxMissedFrames = 10,
                              int minHitsToConfirm = 1)
        : maxAssocDist_(maxAssocDist), maxMissedFrames_(maxMissedFrames),
          minHitsToConfirm_(minHitsToConfirm < 1 ? 1 : minHitsToConfirm) {}

    void setMaxAssocDist(double v) { maxAssocDist_ = v; }

    // 【新增】每台相机的观测时刻偏移（单位：帧，跟 predict 的 dt 一致）。
    // 传空 vector 或不调用 = 全 0 = 旧行为。
    //   相机不同步：offset[c] = (相机c实际曝光时刻 - 参考时刻) / 标称帧长
    //     可以直接用 SyncMonitor 量到的时间戳偏差换算。
    //   运动模糊：所有相机再统一加上 +曝光时长/(2*标称帧长)
    //     因为模糊光斑的质心落在曝光窗口中点，不是窗口起点。
    // 实测(2000mm/s)：模糊增益1.0 时 RMSE 从 1.02mm 恶化到 9.55mm、ID跳变
    // 从 0 涨到 51，全部来自这个没被建模的时刻偏差。
    void setCameraTimeOffsets(const std::vector<double>& offsetsInFrames) {
        camTimeOffsets_ = offsetsInFrames;
    }
    // 所有相机共同的曝光中点补偿(帧)。跟 setCameraTimeOffsets 相加。
    void setExposureMidOffset(double frames) { exposureMidOffset_ = frames; }
    // 是否让带 dt 的观测参与速度估计。默认 false（见 PointIEKF.hpp 说明）。
    void setObsVelJacobian(bool on) { obsVelJacobian_ = on; }
    void setMaxMissedFrames(int v) { maxMissedFrames_ = v; }
    void setMinHitsToConfirm(int v) { minHitsToConfirm_ = v < 1 ? 1 : v; }

    // 过程噪声——每帧预测阶段累加给位置/速度不确定度的方差。越大＝越"信新
    // 观测"(响应快但抖)，越小＝越"信运动模型"(平滑但转向慢)。
    void setPosProcessVar(double v) { posProcessVar_ = std::max(1e-6, v); }
    void setVelProcessVar(double v) { velProcessVar_ = std::max(1e-6, v); }
    // 观测方差兜底——某台相机的观测没提供真实协方差时用的各向同性方差
    // (归一化坐标系单位)。
    void setDefaultObsVar(double v) { defaultObsVar_ = std::max(1e-9, v); }
    // 观测方差地板（归一化坐标单位，= (σ_px/fx)²）。见 update() 里应用处的
    // 完整说明。默认 0 = 关闭，保持旧行为。
    // 取值参考：想让地板等价于 σ=0.5px、fx=900，则填 (0.5/900)² ≈ 3.1e-7。
    void setObsVarFloor(double v) { obsVarFloor_ = std::max(0.0, v); }

    // ── 新增：协方差门控相关 ──────────────────────────────────────────
    // 马氏/卡方门控总开关。关掉＝退回旧版纯欧氏门控(对拍用)。
    void setUseMahalanobisGate(bool v) { useMahalanobisGate_ = v; }
    // 卡方门控阈值(3自由度)。经验取值：7.81=95%，11.34=99%，14.16=99.7%。
    // 默认16(略高于99.7%给点余量，宁可门控稍宽让真点别掉，靠下面硬上限
    // 兜住幽灵)。
    void setChiSquareGate(double v) { chiSquareGate_ = std::max(1e-3, v); }
    // 关联位置方差地板(mm²)。为什么需要：一个长期静止、很确定的 confirmed
    // 点协方差椭球会收缩到极小,此时纯马氏门控会窄到连几mm的检测抖动都容不下
    // →自己把自己的点判成外点→丢。给协方差加一个各向同性地板,门控就不会比
    // "地板对应的半径"更窄。-1=自动=(0.35*maxAssocDist)²。
    void setAssocPosFloorVar(double v) { assocPosFloorVar_ = v; }

    // ── 新增：双锚点关联 ──────────────────────────────────────────────
    void setUseDualAnchorGate(bool v) { useDualAnchorGate_ = v; }

    // ── 新增：coast 幽灵抑制(从 TemporalTracker 移植) ─────────────────
    // 正在 coast 的确认轨迹附近,不 spawn 竞争新轨迹,留给它下一帧自己捡回。
    void setUseCoastGhostSuppression(bool v) { useCoastGhostSuppression_ = v; }

    // ── 新增：基数上限(利用"手套上共 N 个球"的先验) ───────────────────
    // 可见真点数物理上 <= N(无遮挡时正好 N,有遮挡时更少),所以确认轨迹里
    // 排在最弱那几个、把总数顶超过 N 的,几乎必然是幽灵。maxConfirmed<=0 表示
    // 不启用(默认)。启用后:每帧发布阶段,若确认轨迹数 > maxConfirmed,只发布
    // 【存活最久(hits 最多)】的前 maxConfirmed 条,其余暂不发布(不销毁,内部
    // 保留,下一帧若重新挤进前 N 名还能再出现)。按 hits 排序而不是残差,是因为
    // "长期稳定存在"是比"这一帧残差小"强得多的真点信号——幽灵活不久、hits 攒
    // 不高,天然排在后面被挤掉。这是安全网,不是根治:它管住"屏幕上冒出第21个
    // 点"这个直接症状,但幽灵顶替真点(总数仍<=N、点却是错的)那种情况它看不见,
    // 根治仍要靠源头(检测质量、最小视线夹角、三视角仲裁)。
    void setMaxConfirmedTracks(int v) { maxConfirmed_ = v; }

    // ── 新增：机动自适应过程噪声 ──────────────────────────────────────
    void setUseManeuverAdaptiveQ(bool v) { useManeuverAdaptiveQ_ = v; }
    // 归一化残差超过这个阈值算"一次机动信号"。跟检测层残差量级同数量级,
    // 默认0.02(≈典型2px在f≈几百像素下的量级的几倍),按自己相机标定。
    void setManeuverResidThresh(double v) { maneuverResidThresh_ = std::max(1e-6, v); }
    // 检测到机动时过程噪声乘的倍率上限,以及每帧向1衰减的比例。
    void setManeuverBoostMax(double v) { maneuverBoostMax_ = std::max(1.0, v); }
    void setManeuverBoostDecay(double v) { maneuverBoostDecay_ = std::min(0.999, std::max(0.0, v)); }

    // ts_ns：见 TemporalTracker::update() 同名参数的说明——③输出端One Euro
    // 滤波用它换算采样率，-1时按setNominalFrameNs()配置的名义间隔自己合成。
    std::vector<TrackedPoint> update(const std::vector<Track3D>& frameTracks,
                                     const std::vector<EpiMat3>& Rs, const std::vector<EpiVec3>& ts,
                                     const std::vector<std::vector<std::array<double,2>>>& obsPerCam,
                                     const std::vector<std::vector<Cov2>>* obsCovPerCam = nullptr,
                                     int64_t ts_ns = -1) {
        using namespace iekf_track_detail;
        const int nc = int(Rs.size());
        const size_t ncz = size_t(nc);
        std::vector<CamPose> camPoses(ncz);
        for (int c = 0; c < nc; ++c) { camPoses[size_t(c)].R = Rs[size_t(c)]; camPoses[size_t(c)].t = ts[size_t(c)]; }

        const int n = int(frameTracks.size());
        std::vector<char> claimed(size_t(n), 0);

        const double floorVar = (assocPosFloorVar_ >= 0.0)
            ? assocPosFloorVar_
            : (0.35*maxAssocDist_)*(0.35*maxAssocDist_);

        // ------------------------------------------------------------------
        // 【真实 dt】旧版这里恒定 predict(1.0, ...)，也就是"假设每次 update
        // 之间正好过了一帧"。丢帧时这个假设直接崩掉：
        //   - 位置外推不够远 -> 预测点落在目标后方，快速运动时直接掉出关联门；
        //   - 协方差涨得不够 -> 门控该放宽的时候没放宽；
        //   - 下面那套 maxAssocDist_ + velCapGain_*speed 的速度自适应上限，
        //     speed 的单位是 mm/帧，也是按"过了一帧"算的，同样偏小。
        // 三个效应叠在一起，正好都发生在最不该出错的时候(高速+丢帧)。
        // 压测实测：3500mm/s 下丢帧率 30% 时召回率 0.799，而同样速度不丢帧
        // 是 0.993——差距几乎全部来自这里，跟卡尔曼调参无关。
        //
        // 修法保持"帧"为速度单位(不改成 mm/s)，只把经过的帧数算准：这样
        // posProcessVar_/velProcessVar_/velCapGain_ 这些已经调好的参数含义
        // 完全不变，属于纯修正、不需要重新整定。
        // 上下限保护：调用方第一次调用、时间戳回绕、或长时间暂停后恢复时，
        // dt 可能是 0 或者极大值，都夹住，避免一次预测把状态推飞。
        double dtFrames = 1.0;
        if (ts_ns >= 0 && lastUpdateNs_ >= 0 && ts_ns > lastUpdateNs_) {
            dtFrames = double(ts_ns - lastUpdateNs_) / double(nominalFrameNs_);
        }
        dtFrames = std::min(maxPredictDtFrames_, std::max(0.1, dtFrames));
        if (ts_ns >= 0) lastUpdateNs_ = ts_ns;
        lastDtFrames_ = dtFrames;

        // 预测所有已有轨迹一步。机动自适应：每条轨迹用自己的 boost 放大过程
        // 噪声(boost 事后衰减),没触发机动的轨迹 boost≈1、行为跟旧版一致。
        for (auto& t : tracks_) {
            // 只放大【位置】过程噪声,不动速度过程噪声——实测放大速度过程噪声
            // 会让速度估计跟着噪声乱跑、恒速预测直接飞出去,churn反而暴涨;只
            // 放大位置噪声则单纯把位置协方差椭球撑大(门控变宽,利于机动中重新
            // 兜住),不污染速度状态。
            const double pq = posProcessVar_ * (useManeuverAdaptiveQ_ ? t.qBoost : 1.0);
            // Q 随 dt 线性放大：过程噪声是"每帧注入多少不确定度"，跨了几帧
            // 就该注入几帧的量，否则丢帧后协方差偏小、门控偏紧。
            t.filter.predict(dtFrames, pq * dtFrames, velProcessVar_ * dtFrames);
            if (useManeuverAdaptiveQ_) {
                // 向1衰减(乘性),不会瞬间掉回去,给机动一个持续几帧的宽门控。
                t.qBoost = 1.0 + (t.qBoost - 1.0) * maneuverBoostDecay_;
            }
        }

        // confirmed 优先、丢帧的排后——跟旧版同一套贪心顺序。
        std::vector<int> order(tracks_.size());
        for (size_t i=0;i<order.size();++i) order[i]=int(i);
        std::sort(order.begin(), order.end(), [this](int a,int b){
            const bool ca = tracks_[size_t(a)].hits >= minHitsToConfirm_;
            const bool cb = tracks_[size_t(b)].hits >= minHitsToConfirm_;
            if (ca != cb) return ca;
            return tracks_[size_t(a)].missedFrames < tracks_[size_t(b)].missedFrames;
        });

        std::vector<char> alive(tracks_.size(), 0);

        // ── 关联：两阶段【全局最优指派】(confirmed 优先，再 tentative) ──────
        // 【为什么从逐轨迹贪心换成全局指派】旧版按 order 一条条挑，先挑的轨迹
        // 可能因为预测误差抢走本该属于相邻轨迹的观测，相邻轨迹只好去抢前者的
        // ——两个编号当场对调。20点云、点间距30mm、单帧位移>10mm 时这种互换极
        // 频繁(压测实测换号率 2.9 次/帧)。全局指派把所有"过门配对"按分数从小
        // 到大排、全局地先成全最匹配的那些，相邻点各归各的，天然防互换。
        // TemporalTracker 早就是这套做法(见那边 runAssignmentPass)，IEKF 这边
        // 一直没跟上，这里补齐。两阶段的语义严格保留："已证明过自己的真点"
        // 先挑，tentative(疑似幽灵)只能捡剩下的，不让幽灵抢真点的观测。
        // 【每帧先清空】不清的话，被提前 return 的那些帧会留着上一帧的内容，
        // 而录制端看到的是一份"看起来正常"的旧数据 —— 比没有数据危险得多。
        dbg_ = TrackFrameDebug{};
        dbg_.ran = true;
        dbg_.nObs = n;
        dbg_.chiSquareGate = useMahalanobisGate_ ? chiSquareGate_ : -1.0;
        dbg_.maxAssocDistMm = maxAssocDist_;

        struct GateCtx {
            Vec3   predCV{0,0,0};
            Vec3   lastPos{0,0,0};
            bool   confirmed = false;
            double effHardCap2 = 0.0;
            bool   bypassMaha = false;
            bool   haveInv = false;
            double SigInv[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
        };
        std::vector<GateCtx> ctx(tracks_.size());
        for (size_t ti=0; ti<tracks_.size(); ++ti) {
            InternalTrack& tr = tracks_[ti];
            GateCtx& g = ctx[ti];
            g.predCV = tr.filter.position();
            g.lastPos = tr.lastMeasuredPos;
            g.confirmed = tr.hits >= minHitsToConfirm_;
            const Vec3 vel = tr.filter.velocity();
            const double speed = std::sqrt(vel[0]*vel[0]+vel[1]*vel[1]+vel[2]*vel[2]);

            // 硬欧氏上限：tentative 死死摁在 maxAssocDist_(幽灵别乱抓)；
            // confirmed 按自身速度放宽、coast 期再放宽，最后夹到 confirmedCapMax 倍。
            double hardCap;
            if (g.confirmed) {
                // speed 单位是 mm/帧，跨了 dtFrames 帧就该按 dtFrames 折算
                double cap = maxAssocDist_ + velCapGain_ * speed * dtFrames;
                if (tr.missedFrames > 0)
                    cap *= std::min(3.0, 1.0 + tr.missedFrames * 0.4);
                hardCap = std::min(cap, maxAssocDist_ * confirmedCapMax_);
            } else {
                // 【tentative 速度爬坡】旧版把所有未转正轨迹死死钉在
                // maxAssocDist_。这确实摁住了幽灵，但也造成一个结构性的速度
                // 天花板：每条轨迹都从 tentative 起步、必须连续命中
                // minHitsToConfirm 次才转正，期间每帧位移都不许超过
                // maxAssocDist_——于是"能不能追上一个快点"跟卡尔曼、协方差、
                // 密度门控统统无关，只由 maxAssocDist_/帧 这一个数决定。压测
                // 里 20.9mm/帧、assoc=15mm 时覆盖率只有74%，把 assoc 放到 25mm
                // 立刻回到84%，就是这个天花板在起作用。
                // 解法不是无脑放宽（那等于把幽灵也放进来），而是【按它自己
                // 证明了多少】按比例解锁：第1次命中的轨迹(hits==1，刚出生，
                // 完全没有历史、速度估计还是初值)保持钉死；从第2次命中起，
                // 说明它已经连续两帧在合理位置出现、速度估计开始有意义，按
                // 命中数线性解锁速度放宽，到 tentativeRampHits_ 次时拿到跟
                // confirmed 一样的速度放宽。一闪就死的幽灵永远停在 hits==1，
                // 拿不到任何放宽；真的快点两三帧就能挣到自己需要的门控。
                double ramp = 0.0;
                if (useTentativeVelRamp_ && tr.hits >= 2) {
                    ramp = double(tr.hits - 1) / std::max(1.0, tentativeRampHits_);
                    ramp = std::min(1.0, ramp);
                }
                double cap = maxAssocDist_ + velCapGain_ * speed * ramp * dtFrames;
                hardCap = std::min(cap, maxAssocDist_ * confirmedCapMax_);
            }
            g.effHardCap2 = hardCap * hardCap;

            // 密度自适应门控：候选足够孤立(次近远大于最近)时，把有效硬上限放开
            // 到刚好容纳最近那个，并跳过马氏上界——没有可混淆项，认它就是安全的。
            // 注意这里在【任何观测被认领之前】统一评估，全局指派下视角一致。
            if (useDensityGate_) {
                double d1 = std::numeric_limits<double>::max();
                double d2 = std::numeric_limits<double>::max();
                for (int k=0;k<n;++k) {
                    const auto& p = frameTracks[size_t(k)].point;
                    const double ax=p[0]-g.predCV[0], ay=p[1]-g.predCV[1], az=p[2]-g.predCV[2];
                    double dd = ax*ax+ay*ay+az*az;
                    if (useDualAnchorGate_) {
                        const double bx=p[0]-g.lastPos[0], by=p[1]-g.lastPos[1], bz=p[2]-g.lastPos[2];
                        dd = std::min(dd, bx*bx+by*by+bz*bz);
                    }
                    const double d = std::sqrt(dd);
                    if (d < d1) { d2 = d1; d1 = d; }
                    else if (d < d2) { d2 = d; }
                }
                const double reachCap = maxAssocDist_ * densityReachMult_;
                if (d1 <= reachCap && d2 >= densityAmbiguitySep_ * std::max(d1, maxAssocDist_)) {
                    const double adm = d1 * 1.01;
                    g.effHardCap2 = std::max(g.effHardCap2, adm*adm);
                    g.bypassMaha = true;
                }
            }

            if (useMahalanobisGate_) {
                const Mat6& P = tr.filter.covariance();
                double Ppos[3][3];
                for (int r=0;r<3;++r) for (int c=0;c<3;++c) Ppos[r][c] = P[size_t(r)][size_t(c)];
                for (int r=0;r<3;++r) Ppos[r][r] += floorVar;
                g.haveInv = invertSym3(Ppos, g.SigInv);
            }
        }

        // 枚举所有"过门"的 (轨迹, 观测) 配对及其分数。
        struct AssocPair { int track; int obs; double score; };
        std::vector<AssocPair> pairs;
        pairs.reserve(tracks_.size() * size_t(n>0?n:1));
        for (size_t ti=0; ti<tracks_.size(); ++ti) {
            const GateCtx& g = ctx[ti];
            const bool mahaHere = useMahalanobisGate_ && g.haveInv && !g.bypassMaha;
            for (int k=0;k<n;++k) {
                const auto& p = frameTracks[size_t(k)].point;
                auto anchorCost = [&](const Vec3& anchor, double& euclid2)->double {
                    const double dx=p[0]-anchor[0], dy=p[1]-anchor[1], dz=p[2]-anchor[2];
                    euclid2 = dx*dx+dy*dy+dz*dz;
                    if (useMahalanobisGate_ && g.haveInv) {
                        const double d[3]={dx,dy,dz};
                        return mahaSq3(d, g.SigInv);
                    }
                    return euclid2;
                };
                double e2a=0.0, e2b=0.0;
                const double costA = anchorCost(g.predCV, e2a);
                const double costB = useDualAnchorGate_ ? anchorCost(g.lastPos, e2b)
                                                        : std::numeric_limits<double>::max();
                const bool passA = (mahaHere ? costA <= chiSquareGate_ : e2a <= g.effHardCap2)
                                   && e2a <= g.effHardCap2;
                const bool passB = useDualAnchorGate_ &&
                                   ((mahaHere ? costB <= chiSquareGate_ : e2b <= g.effHardCap2)
                                   && e2b <= g.effHardCap2);
                if (!passA && !passB) continue;
                double score;
                if (mahaHere) score = (passA && passB) ? std::min(costA,costB) : (passA ? costA : costB);
                else          score = (passA && passB) ? std::min(e2a,e2b)     : (passA ? e2a   : e2b);
                pairs.push_back({int(ti), k, score});
            }
        }

        std::vector<int> assignedObs(tracks_.size(), -1);
        if (useGlobalAssignment_) {
            // 全局：本阶段所有过门配对按分数升序，先成全最匹配的。
            auto runPass = [&](bool confirmedPass){
                std::vector<AssocPair> sub;
                sub.reserve(pairs.size());
                for (const auto& pr : pairs)
                    if (ctx[size_t(pr.track)].confirmed == confirmedPass) sub.push_back(pr);
                std::sort(sub.begin(), sub.end(),
                          [](const AssocPair&a, const AssocPair&b){ return a.score < b.score; });
                for (const auto& pr : sub) {
                    if (assignedObs[size_t(pr.track)] >= 0) continue;
                    if (claimed[size_t(pr.obs)]) continue;
                    assignedObs[size_t(pr.track)] = pr.obs;
                    claimed[size_t(pr.obs)] = 1;
                }
            };
            runPass(true);
            runPass(false);
        } else {
            // 退回旧版逐轨迹贪心(对拍用)：按 order 顺序，每条轨迹挑自己分数最低
            // 的未认领观测。
            for (int idx : order) {
                int best=-1; double bestScore=std::numeric_limits<double>::max();
                for (const auto& pr : pairs) {
                    if (pr.track != idx) continue;
                    if (claimed[size_t(pr.obs)]) continue;
                    if (pr.score < bestScore) { bestScore = pr.score; best = pr.obs; }
                }
                if (best>=0) { assignedObs[size_t(idx)] = best; claimed[size_t(best)] = 1; }
            }
        }

        // 应用指派结果：命中的做贝叶斯更新，没命中的按 coast/销毁规则处理。
        for (int idx : order) {
            InternalTrack& tr = tracks_[size_t(idx)];
            const bool wasConfirmed = ctx[size_t(idx)].confirmed;
            const Vec3& predCV = ctx[size_t(idx)].predCV;
            const int best = assignedObs[size_t(idx)];

            // ---- 逐点调试：关联结果 ----
            // 【assocDist 和 assocMahaSq 都要】欧氏距离说明"差多远"，
            // 马氏距离说明"按这个点自己的不确定度算，差得离不离谱"。
            // 协方差塌了的时候，欧氏只差 2mm 的观测也能被马氏门挡在外面 ——
            // 只看欧氏的话，那次拒绝会显得毫无道理。
            {
                TrackFrameDebugItem it;
                it.id = tr.id;
                const Vec3 pp = tr.filter.position();
                const Vec3 vv = tr.filter.velocity();
                for (int k = 0; k < 3; ++k) { it.pos[k] = pp[size_t(k)]; it.vel[k] = vv[size_t(k)]; }
                const Mat6& P = tr.filter.covariance();
                it.posVarTrace = P[0][0] + P[1][1] + P[2][2];
                it.velVarTrace = P[3][3] + P[4][4] + P[5][5];
                it.qBoost = tr.qBoost;
                it.missedFrames = tr.missedFrames;
                it.hits = tr.hits;
                it.justAcquired = tr.justAcquired;
                it.confirmed = wasConfirmed;
                it.coasting = (best < 0);
                it.obsIndex = best;
                it.gateBypassed = ctx[size_t(idx)].bypassMaha;
                if (best >= 0) {
                    const auto& o3 = frameTracks[size_t(best)].point;
                    const double dx = o3[0] - predCV[0], dy = o3[1] - predCV[1], dz = o3[2] - predCV[2];
                    it.assocDistMm = std::sqrt(dx*dx + dy*dy + dz*dz);
                    if (ctx[size_t(idx)].haveInv) {
                        const double d3[3] = {dx, dy, dz};
                        it.assocMahaSq = mahaSq3(d3, ctx[size_t(idx)].SigInv);
                    }
                    it.residualMm = frameTracks[size_t(best)].residual;
                }
                dbg_.items.push_back(it);
            }

            if (best >= 0) {
                std::vector<CameraMeasurement> meas;
                meas.reserve(frameTracks[size_t(best)].support.size());
                for (const auto& s : frameTracks[size_t(best)].support) {
                    const int ci = s.first, oi = s.second;
                    if (ci < 0 || ci >= nc) continue;
                    if (oi < 0 || oi >= int(obsPerCam[size_t(ci)].size())) continue;
                    CameraMeasurement m;
                    m.cam = &camPoses[size_t(ci)];
                    m.nx = obsPerCam[size_t(ci)][size_t(oi)][0];
                    m.ny = obsPerCam[size_t(ci)][size_t(oi)][1];
                    // 【观测时刻校正】这台相机的观测其实不是在参考时刻测的：
                    //   ① 相机间时间戳不同步(硬件没有外触发时普遍存在)；
                    //   ② 运动模糊：拖尾光斑质心落在曝光窗口中点，等价于
                    //      观测时刻比帧时间戳晚了半个曝光时长。
                    // 两者都是"这次观测对应的是另一个时刻的位置"，合成一个
                    // 偏移量交给 IEKF 的观测模型去处理(见 PointIEKF.hpp 的
                    // projectAndJacobian)。默认全 0 = 旧行为。
                    m.dt = camTimeOffset(ci);
                    m.velJacobian = obsVelJacobian_;
                    if (obsCovPerCam && ci < int(obsCovPerCam->size()) &&
                        oi < int((*obsCovPerCam)[size_t(ci)].size())) {
                        m.sigma = (*obsCovPerCam)[size_t(ci)][size_t(oi)];
                    } else {
                        m.sigma = {defaultObsVar_, 0.0, defaultObsVar_};
                    }
                    // 【观测方差地板】检测层报的协方差只描述"质心抖动"——零均值、
                    // 逐帧独立。它【不含】标定偏差、镜头畸变残留、球体形变/部分
                    // 遮挡导致的质心系统性偏移这些系统性误差；后者不会因为多看几帧
                    // 就变小。检测器越好，报的协方差越小，滤波器就越"绝对相信"这次
                    // 测量——于是任何一次误关联都会变成一次确信无疑的巨大状态跳变，
                    // 轨迹被拽飞、关联不上、拿新编号。实测(压测台20点手部场景)：
                    // 检测协方差被钉到 σ=1e-4px 时，单点最大误差 72.8mm、9 次编号
                    // 跳变；而同样数据喂启发式追踪(只吃聚类3D点)是零误差零跳变——
                    // 差别正来自这里"无条件相信观测协方差"。
                    // 地板值是"再准的检测器也不可能比这更准"的下限，跟检测质量无关，
                    // 只跟系统的标定/建模精度有关。默认 0 = 关闭(保持旧行为)。
                    if (obsVarFloor_ > 0.0) {
                        if (m.sigma.xx < obsVarFloor_) m.sigma.xx = obsVarFloor_;
                        if (m.sigma.yy < obsVarFloor_) m.sigma.yy = obsVarFloor_;
                        // 相关项按新的对角线做柯西-施瓦茨截断，避免抬高对角线之后
                        // 相关系数 |ρ|>1、协方差矩阵变成非正定。
                        const double maxOff = std::sqrt(m.sigma.xx * m.sigma.yy) * 0.999;
                        if (m.sigma.xy >  maxOff) m.sigma.xy =  maxOff;
                        if (m.sigma.xy < -maxOff) m.sigma.xy = -maxOff;
                    }
                    meas.push_back(m);
                }

                if (useManeuverAdaptiveQ_) {
                    const auto& obs3 = frameTracks[size_t(best)].point;
                    const double dx=obs3[0]-predCV[0], dy=obs3[1]-predCV[1], dz=obs3[2]-predCV[2];
                    const double resid3d = std::sqrt(dx*dx+dy*dy+dz*dz);
                    const double maneuver = resid3d / std::max(1e-6, maxAssocDist_);
                    if (maneuver > maneuverResidThresh_) {
                        const double target = std::min(maneuverBoostMax_, 1.0 + maneuver * 4.0);
                        tr.qBoost = std::max(tr.qBoost, target);
                    }
                }

                tr.filter.update(meas);
                tr.lastMeasuredPos = tr.filter.position();
                tr.missedFrames = 0;
                tr.hits += 1;
                tr.justAcquired = (!wasConfirmed && tr.hits >= minHitsToConfirm_);
                alive[size_t(idx)] = 1;
            } else {
                tr.missedFrames += 1;
                tr.justAcquired = false;
                if (!wasConfirmed) alive[size_t(idx)] = 0;
                else alive[size_t(idx)] = (tr.missedFrames <= maxMissedFrames_) ? 1 : 0;
            }
        }


        // 【coast 抑制】收集这一帧"没关联上、但还活着"的确认轨迹的当前位置
        // 和它们各自的可捡回半径(跟关联硬上限同口径,按速度放宽)。下面 spawn
        // 新轨迹时,落在任何一个这种范围里的观测【不生成新轨迹】——那几乎必然
        // 是这条确认轨迹这一帧因预测漂移没关联上的真观测,让它下一帧(missedFrames>0、
        // 门控已放宽)自己重新捡回即可,而不是凭空造一个竞争幽灵。没有这道保护时,
        // 高确认门槛下真点反向那帧 coast、观测 spawn 竞争新轨迹、新轨迹没攒够又被
        // 杀→真点永远转不了正→屏幕上直接消失。这是从 TemporalTracker 移植回来的
        // 关键保护(见 TemporalTracker.hpp 的 coastingConfirmed 段)。
        struct CoastZone { Vec3 pos; double r2; };
        std::vector<CoastZone> coastZones;
        if (useCoastGhostSuppression_) {
            for (size_t i=0;i<tracks_.size();++i) {
                if (!alive[i]) continue;
                const InternalTrack& t = tracks_[i];
                if (t.hits >= minHitsToConfirm_ && t.missedFrames > 0) {
                    const Vec3 v = t.filter.velocity();
                    const double sp = std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
                    // 口径必须跟上面的关联硬上限【逐字一致】：先按速度算基础
                    // cap、再按丢帧数放宽、【最后】才夹到 confirmedCapMax 倍。
                    // 之前是先夹后乘，抑制半径最大能到关联上限的3倍(24×
                    // maxAssocDist)，assoc=15mm 时抑制区半径可达360mm——整只手
                    // 都落在里面，手套场景下新出现的真球会被误当成"coast点的
                    // 观测"而拒绝生成轨迹，表现为真点迟迟不出现/覆盖率下降。
                    double r = maxAssocDist_ + velCapGain_ * sp;
                    r *= std::min(3.0, 1.0 + t.missedFrames * 0.4);
                    r = std::min(r, maxAssocDist_ * confirmedCapMax_);
                    coastZones.push_back({t.filter.position(), r*r});
                }
            }
        }

        // 没被认领的观测：新开 tentative 轨迹。
        for (int k=0;k<n;++k) {
            if (claimed[size_t(k)]) continue;
            if (!coastZones.empty()) {
                const auto& p = frameTracks[size_t(k)].point;
                bool onCoast = false;
                for (const auto& z : coastZones) {
                    const double dx=p[0]-z.pos[0], dy=p[1]-z.pos[1], dz=p[2]-z.pos[2];
                    if (dx*dx+dy*dy+dz*dz <= z.r2) { onCoast = true; break; }
                }
                if (onCoast) continue;   // 别在正在 coast 的确认点身上凭空造幽灵
            }
            InternalTrack tr;
            tr.id = nextId_++;
            tr.filter = PointIEKF6D(frameTracks[size_t(k)].point, initPosVar_, initVelVar_);
            tr.lastMeasuredPos = frameTracks[size_t(k)].point;
            tr.qBoost = 1.0;
            tr.hits = 1;
            tr.missedFrames = 0;
            tr.justAcquired = (minHitsToConfirm_ <= 1);
            tracks_.push_back(std::move(tr));
            alive.push_back(1);
        }

        std::vector<InternalTrack> kept;
        kept.reserve(tracks_.size());
        for (size_t i=0;i<tracks_.size();++i)
            if (alive[i]) kept.push_back(std::move(tracks_[i]));
        tracks_.swap(kept);

        // 发布阶段：先收集所有已确认轨迹的下标。
        std::vector<size_t> confirmedIdx;
        for (size_t i=0;i<tracks_.size();++i)
            if (tracks_[i].hits >= minHitsToConfirm_) confirmedIdx.push_back(i);

        // 【基数上限】若启用且确认数超预算,只保留 hits 最多(存活最久=最可能
        // 是真点)的前 maxConfirmed_ 条发布,其余暂不发布(内部保留,不销毁)。
        // 不改动 tracks_/编号/activeCount 的语义——纯粹是"这一帧对外发布哪些"
        // 的过滤,被压下去的轨迹下一帧若重新挤进前 N 名还能再出现。
        if (maxConfirmed_ > 0 && int(confirmedIdx.size()) > maxConfirmed_) {
            std::sort(confirmedIdx.begin(), confirmedIdx.end(),
                      [this](size_t a, size_t b){
                          if (tracks_[a].hits != tracks_[b].hits) return tracks_[a].hits > tracks_[b].hits;
                          if (tracks_[a].missedFrames != tracks_[b].missedFrames) return tracks_[a].missedFrames < tracks_[b].missedFrames;
                          return tracks_[a].id < tracks_[b].id;   // 稳定：编号小(更早出现)的优先
                      });
            confirmedIdx.resize(size_t(maxConfirmed_));
        }

        // 【新增·③输出端One Euro滤波】同TemporalTracker的合成时间戳策略。
        if (useOutputFilter_) {
            nextSynthNs_ = (ts_ns >= 0) ? ts_ns : (nextSynthNs_ + nominalFrameNs_);
        }

        dbg_.nTracks = int(tracks_.size());
        dbg_.nConfirmed = int(confirmedIdx.size());
        for (const auto& t : tracks_) if (t.justAcquired) ++dbg_.nNewborn;

        std::vector<TrackedPoint> out;
        out.reserve(confirmedIdx.size());
        for (size_t i : confirmedIdx) {
            InternalTrack& t = tracks_[i];
            TrackedPoint p;
            p.id = t.id;
            p.position = t.filter.position();
            p.velocity = t.filter.velocity();
            p.missedFrames = t.missedFrames;
            p.justAcquired = t.justAcquired;
            if (useOutputFilter_) {
                // 新转正的轨迹不能带着(默认构造的)滤波器状态平滑——首次输出
                // 前它还没接收过任何样本，OneEuroFilter3的initialized_本来就是
                // false，第一次filter()调用会原样返回不做平滑，天然安全，不需要
                // 像TemporalTracker那样显式reset()一次(那边是因为InternalTrack
                // 可能在旧实现里被复用；IEKF这里新轨迹总是全新default构造，
                // 双重保险起见仍然显式reset，逻辑更直白、不依赖这个隐式细节)。
                if (p.justAcquired) t.outFilter.reset();
                p.position = t.outFilter.filter(p.position, nextSynthNs_);
            }
            out.push_back(p);
        }
        return out;
    }

    int activeCount() const {
        int c = 0;
        for (const auto& t : tracks_) if (t.hits >= minHitsToConfirm_) ++c;
        return c;
    }
    int totalIdsAssigned() const { return nextId_; }

private:
    // 相机 ci 的观测时刻偏移(帧)：不同步偏移 + 曝光中点补偿。
    double camTimeOffset(int ci) const {
        double o = exposureMidOffset_;
        if (ci >= 0 && ci < int(camTimeOffsets_.size())) o += camTimeOffsets_[size_t(ci)];
        return o;
    }

    struct InternalTrack {
        int id = -1;
        PointIEKF6D filter{Vec3{0,0,0}};
        Vec3 lastMeasuredPos{0,0,0};   // 上一次真正测到的后验位置(双锚点的锚点B)
        double qBoost = 1.0;           // 机动自适应过程噪声倍率(>=1,事后衰减回1)
        int missedFrames = 0;
        int hits = 0;
        bool justAcquired = false;
        OneEuroFilter3 outFilter;      // ③输出端滤波器状态——只平滑对外发布值，不反馈进滤波器内部状态
    };

    std::vector<InternalTrack> tracks_;
    TrackFrameDebug dbg_{};
    std::vector<double> camTimeOffsets_;    // 每相机观测时刻偏移(帧)
    bool    obsVelJacobian_ = false;        // 见 setObsVelJacobian
    double  exposureMidOffset_ = 0.0;       // 曝光中点补偿(帧)，所有相机共用
    int64_t lastUpdateNs_ = -1;             // 上一次 update 的时间戳，用于算真实 dt
    double  lastDtFrames_  = 1.0;           // 最近一次预测实际跨了几帧(诊断用)
    double  maxPredictDtFrames_ = 6.0;      // 单次预测允许跨的最大帧数(防止暂停后推飞)
    double maxAssocDist_;
    int maxMissedFrames_;
    int minHitsToConfirm_;
    int nextId_ = 0;
    double posProcessVar_ = 4.0;
    double velProcessVar_ = 25.0;
    double initPosVar_ = 100.0;
    double initVelVar_ = 1e4;
    double defaultObsVar_ = 1e-5;
    double obsVarFloor_   = 0.0;   // 观测方差地板(归一化坐标)，0=关闭

    // 新增开关的默认值——【依据合成实验的实测结论,不是拍脑袋】：
    //  · 马氏门控 + 双锚点：默认开。实测两处净收益、零回退：
    //      单点快速晃动(80mm/8Hz)编号churn 63→49；
    //      静止点+4mm检测噪声 churn 12→1(恒速估计从噪声里估出假速度、预测
    //      偏出固定门控,双锚点回退到上次位置照样兜住)。
    //  · 机动自适应过程噪声：默认【关】。这是个实测失败的想法,如实记录：
    //      放大过程噪声(不论只放大位置还是位置+速度)都会让状态估计跟着噪声
    //      跑、恒速预测飞出去,同一批实验里 churn 反而暴涨(80mm/8Hz 49→400+,
    //      甚至把本来完美的 120mm/6Hz 从5打到400+)。留着 setter 供人做实验,
    //      但默认必须关,别开。
    bool   useMahalanobisGate_   = true;
    double chiSquareGate_        = 16.0;
    double assocPosFloorVar_     = -1.0;   // <0=自动=(0.35*maxAssocDist)²
    bool   useDualAnchorGate_    = true;
    bool   useCoastGhostSuppression_ = true;  // 从 TemporalTracker 移植的关键保护,默认开
    bool   useManeuverAdaptiveQ_ = false;  // 实测有害,默认关(见上)
    double maneuverResidThresh_  = 0.15;
    double maneuverBoostMax_     = 8.0;
    double maneuverBoostDecay_   = 0.6;
    // confirmed 真点的速度自适应硬上限：cap = maxAssocDist + velCapGain*speed，
    // 夹在 maxAssocDist*confirmedCapMax 以内。velCapGain≈1.5 表示"允许它落在
    // 恒速预测点周围 ~1.5 个单帧位移的范围内"——足够容纳转向点的二阶预测误差。
    double velCapGain_     = 1.5;
    double confirmedCapMax_ = 8.0;
    int    maxConfirmed_    = 0;   // <=0 关闭(默认)；>0 = 手套球数等基数上限
    // 密度自适应门控(把固定 maxAssocDist 升级成随局部歧义度自适应)。
    bool   useTentativeVelRamp_  = true;   // tentative 按命中数逐步解锁速度放宽(破速度天花板)
    double tentativeRampHits_    = 3.0;    // 连续命中这么多次后拿到完整速度放宽
    bool   useGlobalAssignment_  = true;   // 两阶段全局最优指派(防编号互换)；false=旧版逐轨迹贪心
    bool   useDensityGate_       = true;   // 实测：稀疏快速点速度上限大幅提高、稠密场景零副作用
    double densityReachMult_     = 12.0;   // 孤立候选最大可够到 maxAssocDist 的这个倍数
    double densityAmbiguitySep_  = 2.5;    // 次近须 >= 本值×最近 才算"孤立"(越大越保守)

    // ── 【新增·③输出端One Euro滤波】默认关，参数取自三角化调试窗口已验证
    // 的一组(minCutoff=30, beta=0.5)，理由同TemporalTracker.hpp同名字段。
    bool    useOutputFilter_ = false;
    double  filterMinCutoff_ = 30.0;
    double  filterBeta_      = 0.5;
    double  filterDCutoff_   = 1.0;
    int64_t nominalFrameNs_  = 8'333'333;   // ≈120fps，调用方不传真实ts_ns时的合成帧间隔
    int64_t nextSynthNs_     = 0;

public:
    void setVelCapGain(double v) { velCapGain_ = std::max(0.0, v); }
    void setConfirmedCapMax(double v) { confirmedCapMax_ = std::max(1.0, v); }
    void setUseTentativeVelRamp(bool v) { useTentativeVelRamp_ = v; }
    void setTentativeRampHits(double v) { tentativeRampHits_ = std::max(1.0, v); }
    void setUseGlobalAssignment(bool v) { useGlobalAssignment_ = v; }
    void setUseDensityGate(bool v) { useDensityGate_ = v; }
    void setDensityReachMult(double v) { densityReachMult_ = std::max(1.0, v); }
    void setDensityAmbiguitySep(double v) { densityAmbiguitySep_ = std::max(1.0, v); }
    void setUseOutputFilter(bool v) { useOutputFilter_ = v; }
    void setOutputFilterParams(double minCutoff, double beta, double dCutoff = 1.0) {
        filterMinCutoff_ = minCutoff; filterBeta_ = beta; filterDCutoff_ = dCutoff;
        for (auto& t : tracks_) t.outFilter.setParams(minCutoff, beta, dCutoff);
    }
    void setNominalFrameNs(int64_t ns) { nominalFrameNs_ = ns > 0 ? ns : nominalFrameNs_; }
};

} // namespace mocap
