#pragma once
// ---------------------------------------------------------------------------
// 时间连续性 —— 纯数学，零 Qt/第三方依赖，可单测。
//
// MultiViewCluster.hpp 每一帧独立产出一批 Track3D（无状态、不跨帧）。这一层
// 加上"记忆":
//   1. 跨帧维持同一个点的稳定 ID（不是每帧重新编号）。
//   2. 用恒速模型预测下一帧位置，遮挡（某帧该点没被 cluster 出来）后在
//      "预测位置 + 允许的丢失帧数窗口"内维持这个 ID 等它重新出现，而不是
//      立刻销毁重开一个新 ID。
//   3. 抗快速移动：关联时用"预测位置"而不是"上一帧原始位置"去比对新观测，
//      运动越快、预测补偿越多，命中半径可以设得更紧、减少误关联。
//
// 关联策略是贪心最近邻，按"最近刚看到、最有把握"的已存在轨迹优先抢最近的
// 新观测（同 MultiViewCluster 里"残差从小到大"的贪心思路一致）。这不是最优
// 的全局指派（比如两个点距离很近、轨迹交叉的极端情况可能错配），但对
// mocap 场景（点稀疏、帧率够高、点间距通常远大于单帧位移）是恰当的取舍——
// 全局最优指派（匈牙利算法）在需要时可以后续加，不影响这一层的接口。
//
// 跟前两层的关系：Epipolar+TwoViewMatcher 解决"同一帧内谁是谁"，
// MultiViewCluster 解决"这一帧里这个候选点是不是真的"，这一层解决
// "这一帧的点跟上一帧的点是不是同一个点"——三层叠起来就是完整的
// "多点对应性求解"。
// ---------------------------------------------------------------------------
#include "reconstruct/MultiViewCluster.hpp"
#include "reconstruct/TrackFrameDebug.hpp"
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>
#include "reconstruct/OneEuroFilter.hpp"
#include <limits>

namespace mocap {

struct TrackedPoint {
    int id = -1;
    std::array<double,3> position{0,0,0};   // 最近一次真正被看到时的位置（不是预测值）
    std::array<double,3> velocity{0,0,0};   // 恒速模型估计的速度（单位：位置/帧）
    int missedFrames = 0;                   // 连续多少帧没被看到（0 = 这一帧刚更新过）
    bool justAcquired = false;              // 这一帧是不是新分配的 ID（供调用方做提示用）
};

class TemporalTracker {
public:
    // 上一次 update() 的逐点内部状态。【跟 IekfPointTracker 共用同一个结构】——
    // 两个后端是可以互换的两套实现，分析脚本不该因为用户勾了哪个后端就走两条
    // 不同的路径。这个后端没有协方差和马氏距离，那两项留 -1（不是 0）：
    // 0 是合法值会被误当成真实测量，-1 一眼看得出是"这个后端给不出"。
    const TrackFrameDebug& lastFrameDebug() const { return dbg_; }

    // maxAssocDist: 预测位置与新观测的最大关联距离（世界单位，如 mm）。
    // maxMissedFrames: 允许"记忆"一个点最多丢失多少帧，超过就彻底销毁该 ID
    //   （下次再出现会拿到一个全新 ID，而不是复活旧的——避免遮挡太久之后
    //   把一个新点误认成旧点）。注意这个值是"帧数"，帧率翻倍时同样的遮挡
    //   时长对应的帧数也翻倍，调用方按自己的帧率换算(比如120fps下想容忍
    //   300ms遮挡就传36)。
    // minHitsToConfirm: 新轨迹要连续被看到多少帧才"转正"并对外可见。
    //   专业动捕的标准做法(tentative -> confirmed)：多视角聚类偶发的幽灵点
    //   (反光、极线歧义)通常只存活1~2帧，要求连续命中N帧后才发布，幽灵点
    //   根本拿不到公开ID——否则每个幽灵都领一个新编号，表现出来就是UI上
    //   "编号疯狂增加、点闪烁"。tentative期间一旦丢帧立刻销毁(不享受
    //   maxMissedFrames的记忆窗口——还没证明自己是真点，不值得记住)。
    //   默认1 = 完全保持旧行为(第一帧就发布)，调用方按需开启。
    //   代价：真点也要晚 minHitsToConfirm-1 帧才出现，120fps下设3只延迟
    //   17ms，肉眼无感。
    explicit TemporalTracker(double maxAssocDist = 30.0, int maxMissedFrames = 10,
                             int minHitsToConfirm = 1)
        : maxAssocDist_(maxAssocDist), maxMissedFrames_(maxMissedFrames),
          minHitsToConfirm_(minHitsToConfirm < 1 ? 1 : minHitsToConfirm) {}

    // 【新增】运行时改参数，不清空已有轨迹/编号——之前唯一的改法是整个
    // 重新构造一个 TemporalTracker，调用方(比如调参面板)图省事直接
    // `tracker_ = TemporalTracker(...)`，代价是每碰一下滑块所有已确认
    // 的编号都被清空重来，表现出来就是"点云明明很稳，编号却在频繁增长"
    // ——不是追踪真的不稳，是调参这个动作本身在不断重置状态。这几个
    // setter让"调参数"和"清空追踪状态"变成两件独立的事，只有真正需要
    // 从零开始时(比如换了相机组合)才应该调用完整重新构造。
    void setMaxAssocDist(double v) { maxAssocDist_ = v; }
    void setMaxMissedFrames(int v) { maxMissedFrames_ = v; }
    void setMinHitsToConfirm(int v) { minHitsToConfirm_ = v < 1 ? 1 : v; }

    // ── 运行时行为开关（供调参面板/设置加载调用）──────────────────────
    // 这些开关对应几种可独立开关的追踪策略。默认值是我实测下最稳的一组
    // (全局最优指派+速度平滑 开；自适应上限+恒加速度 关；丢帧放宽 开)。
    // 调用方设了哪个就以调用方为准，没设的用这里的默认。
    void setUseOptimalAssignment(bool v)      { useOptimalAssignment_ = v; }        // 全局最优指派(防编号互换)；false=退回逐轨迹贪心
    void setUseAdaptiveAssocCap(bool v)       { useAdaptiveAssocCap_ = v; }         // 逐点自适应关联上限(点密集时按邻距收窄)
    void setAdaptiveCapMultiplier(double v)   { adaptiveCapMultiplier_ = v; }       // 上限=该乘子×到最近另一轨迹的距离
    void setUseMissedFrameRelax(bool v)       { useMissedFrameRelax_ = v; }         // 丢帧期间放宽关联半径(利于遮挡后重新捡回)
    void setRelaxGrowthPerMissedFrame(double v){ relaxGrowthPerMissedFrame_ = v; }  // 每丢一帧半径按基准的这个比例增长
    void setRelaxCapMultiplier(double v)      { relaxCapMultiplier_ = v; }          // 放宽上限：最多到基准半径的这个倍数
    void setUseVelocitySmoothing(bool v)      { useVelocitySmoothing_ = v; }        // 速度EWMA平滑(抗静止噪声过冲导致的churn)
    void setVelocitySmoothingAlpha(double v)  { velSmoothingAlpha_ = v; }           // EWMA系数，越小越稳、对真实加速响应越慢
    void setUseConstantAcceleration(bool v)   { useConstantAcceleration_ = v; }     // 恒加速度预测(急动时更跟手)；关=恒速

    // 累计至今分配出去的编号总数(= 至今真正领过号的轨迹数)。调参/稳定性面板
    // 用它量化churn：记一个基准值(resetStabilityStats)，过一段时间再读，两者
    // 之差就是这段时间新生了多少编号——差为0说明完全没churn，差越大churn越
    // 严重。因为编号是【延迟到转正才领】且严格自增，这个值就等于 nextId_；
    // 一闪就死、从没转正的幽灵不消耗编号，所以这个指标不会被幽灵虚高，量出
    // 来的就是真实的"稳定点被迫换号"的次数。
    int totalIdsAssigned() const { return nextId_; }

    // 喂入这一帧 MultiViewCluster 产出的 verified tracks（只用 point 字段，
    // support/residual 这一层不关心）。返回带稳定 ID 的当前活跃轨迹列表——
    // 只含已转正(confirmed)的轨迹，tentative 的内部继续攒命中数但不外发。
    // ts_ns：这一帧的真实时间戳(纳秒)，用于③输出端One Euro滤波换算采样率。
    // 传-1(默认，兼容旧调用)时内部按setNominalFrameNs()配置的名义帧间隔
    // 自己合成一个单调递增的时间戳——One Euro只关心"帧间隔多长"来算截止
    // 频率，合成时间戳在没有真实时钟的调用场景(比如单元测试)下依然让
    // 滤波行为合理，只是假设了一个固定帧率。
    std::vector<TrackedPoint> update(const std::vector<Track3D>& frameTracks, int64_t ts_ns = -1) {
        const int n = int(frameTracks.size());
        std::vector<char> claimed(size_t(n), 0);

        // 【关联门控·"编号还在增长"的关键修复】
        // 旧做法(已删)：取整帧里最近的任意两个点的间距的一半，当作【所有点】
        // 的关联半径上限。它是为"逐轨迹贪心防互换"服务的补丁，副作用却极大：
        // 只要场景里有任意一对点靠得近，整帧每一个点的关联半径都被一起压窄；
        // 一个点(哪怕孤立)只要帧间抖动超过这个被压窄的半径，就关联不上→coast→
        // 被销毁→回来拿新编号，编号计数一路往上涨。这不是"两点互换编号"，是
        // "点反复死了又新生"，是无硬件同步相机上churn的主因之一。
        //
        // 现在关联是全局最优指派(见下)，防互换由指派本身保证——它天然把每个
        // 观测分给离它最近的轨迹，近邻点各归各的，根本不需要靠压窄门控来防
        // 互换。所以把收窄逻辑整个删掉，门控直接用完整的配置值 maxAssocDist_：
        // 该关联上的点不再被误杀，churn 的这个来源消失；互换由全局指派兜底。

        // 【ID稳定性关键改动】跨帧关联从"逐轨迹贪心"升级为"全局最近距离指派"。
        // 逐轨迹贪心的病根：按轨迹顺序一条条挑，先挑的那条可能因为帧间抖动或
        // 预测误差，抢走了本该属于相邻轨迹的那个观测；相邻轨迹只能去抢前者的
        // 观测——两个编号当场对调，表现出来就是"共视范围内编号莫名跳变"。
        // 全局指派：把所有(轨迹,观测)在门控内的配对按距离从小到大排，全局地
        // 先成全最近的那些配对，相邻两点各自落到离自己最近的观测上，不再互换。
        //
        // 分两趟，严格保留原先"confirmed 优先于 tentative 抢观测"的语义：第一趟
        // 只在 confirmed 轨迹与观测之间做全局最近指派；第二趟才让 tentative 轨迹
        // 认领第一趟剩下的观测。既修掉了真点之间的编号互换(最该修的)，又不让
        // 幽灵(tentative)抢到本该属于真点的观测。
        std::vector<std::array<double,3>> preds(tracks_.size());
        for (size_t i=0;i<tracks_.size();++i) preds[i] = predict(tracks_[i]);

        // 逐点关联门控 = 基准 maxAssocDist_，可选叠加两种独立调整：
        //  ① 自适应上限(useAdaptiveAssocCap_)：点密集时按到最近另一轨迹的距离
        //     收窄，防止抢到邻居观测。注意是【逐点】的——只有真正挨着别的点的
        //     点才收窄，孤立点不受影响(旧的全局收窄会连孤立点一起误杀，是churn源)。
        //  ② 丢帧放宽(useMissedFrameRelax_)：一条轨迹正在coast(missedFrames>0)时
        //     把它的关联半径按丢帧数放大，利于遮挡后把它重新捡回来，减少"丢几帧
        //     就彻底销毁、回来拿新号"的churn。
        // 默认①关②开，跟全局最优指派配合最稳。
        std::vector<double> perTrackGate(tracks_.size(), maxAssocDist_);
        for (size_t i=0;i<tracks_.size();++i) {
            double gate = maxAssocDist_;
            if (useAdaptiveAssocCap_ && tracks_.size() >= 2) {
                double nearestOther = std::numeric_limits<double>::max();
                for (size_t j=0;j<tracks_.size();++j) {
                    if (i==j) continue;
                    const double dx=preds[i][0]-preds[j][0];
                    const double dy=preds[i][1]-preds[j][1];
                    const double dz=preds[i][2]-preds[j][2];
                    nearestOther = std::min(nearestOther, std::sqrt(dx*dx+dy*dy+dz*dz));
                }
                if (nearestOther < std::numeric_limits<double>::max())
                    gate = std::min(gate, std::max(kMinEffectiveAssocDist, adaptiveCapMultiplier_*nearestOther));
            }
            if (useMissedFrameRelax_ && tracks_[i].missedFrames > 0) {
                double grow = 1.0 + relaxGrowthPerMissedFrame_ * double(tracks_[i].missedFrames);
                grow = std::min(grow, relaxCapMultiplier_);
                gate *= grow;
            }
            perTrackGate[i] = gate;
        }

        std::vector<int> assignedObs(tracks_.size(), -1);   // 每条轨迹这一帧关联到的观测下标(-1=没关联上)

        auto runAssignmentPass = [&](bool confirmedPass) {
            struct AssocPair { int track; int obs; double dist; };
            std::vector<AssocPair> pairs;
            for (size_t ti=0; ti<tracks_.size(); ++ti) {
                if (assignedObs[ti] >= 0) continue;
                const bool isConfirmed = tracks_[ti].hits >= minHitsToConfirm_;
                if (isConfirmed != confirmedPass) continue;   // 这一趟只处理对应类别(confirmed/tentative)的轨迹
                for (int k=0;k<n;++k) {
                    if (claimed[size_t(k)]) continue;
                    const double dx=frameTracks[size_t(k)].point[0]-preds[ti][0];
                    const double dy=frameTracks[size_t(k)].point[1]-preds[ti][1];
                    const double dz=frameTracks[size_t(k)].point[2]-preds[ti][2];
                    const double d=std::sqrt(dx*dx+dy*dy+dz*dz);
                    if (d<=perTrackGate[ti]) pairs.push_back({int(ti),k,d});
                }
            }
            // 指派顺序：最优=全局按距离升序(近的配对先成全，防互换)；关掉最优则
            // 退回逐轨迹贪心=先按轨迹优先级(丢帧少的先)、同轨迹内按距离，walk时
            // 每条轨迹先抢到自己最近的可用观测再轮到下一条。
            if (useOptimalAssignment_) {
                std::sort(pairs.begin(), pairs.end(),
                          [](const AssocPair&a,const AssocPair&b){ return a.dist<b.dist; });
            } else {
                std::sort(pairs.begin(), pairs.end(), [this](const AssocPair&a,const AssocPair&b){
                    if (a.track != b.track) {
                        const int ma = tracks_[size_t(a.track)].missedFrames;
                        const int mb = tracks_[size_t(b.track)].missedFrames;
                        if (ma != mb) return ma < mb;
                        return a.track < b.track;
                    }
                    return a.dist < b.dist;
                });
            }
            for (const auto& p : pairs) {
                if (assignedObs[size_t(p.track)] >= 0) continue;   // 这条轨迹已被更近的配对成全
                if (claimed[size_t(p.obs)]) continue;              // 这个观测已被更近的配对拿走
                assignedObs[size_t(p.track)] = p.obs;
                claimed[size_t(p.obs)] = 1;
            }
        };
        runAssignmentPass(/*confirmedPass=*/true);
        runAssignmentPass(/*confirmedPass=*/false);

        std::vector<char> trackAlive(tracks_.size(), 0);

        for (size_t ti=0; ti<tracks_.size(); ++ti) {
            InternalTrack& tp = tracks_[ti];
            const int best = assignedObs[ti];

            const bool wasConfirmed = tp.hits >= minHitsToConfirm_;
            if (best>=0) {
                const std::array<double,3>& newPos = frameTracks[size_t(best)].point;
                // 恒速模型更新：只有真正连续两次都被看到时才更新速度估计
                // （刚从遮挡里捡回来的那一帧，用"捡回位置 - 遮挡前位置"除以
                // 经过的帧数，得到跨遮挡区间的平均速度，而不是拿预测值污染它）。
                const int dtFrames = tp.missedFrames + 1;
                // 【churn修复·第二处】速度做EWMA平滑，不再直接用"这一帧位置-上一帧
                // 位置"当速度。原因：静止/近静止的点因检测噪声会来回小抖，直接
                // 用瞬时位移当速度，会估出一个忽正忽负的虚假速度，恒速预测就把
                // 下一帧预测到偏离真实位置更远的地方(来回抖时甚至朝反方向过冲)，
                // 反而超出关联门控→漏配→点被销毁→回来拿新编号，编号计数一路涨。
                // EWMA后，反复正负的噪声位移互相抵消，静止噪声点的估计速度趋近0、
                // 预测≈当前位置，稳；而真正持续的运动，EWMA会收敛到真实速度，不丢
                // 跟踪能力。跨遮挡那一帧(dtFrames>1)用区间平均位移，语义不变。
                const std::array<double,3> instVel = {
                    (newPos[0]-tp.position[0])/double(dtFrames),
                    (newPos[1]-tp.position[1])/double(dtFrames),
                    (newPos[2]-tp.position[2])/double(dtFrames)
                };
                const std::array<double,3> prevVel = tp.velocity;
                for (int c=0;c<3;++c)
                    tp.velocity[size_t(c)] = useVelocitySmoothing_
                        ? velSmoothingAlpha_*instVel[size_t(c)] + (1.0-velSmoothingAlpha_)*tp.velocity[size_t(c)]
                        : instVel[size_t(c)];
                // 恒加速度：仅在连续两帧都看到(dtFrames==1)时，用速度增量的EWMA估
                // 计加速度；跨遮挡那一帧速度是区间平均、增量无意义，不更新并清零，
                // 免得拿陈旧加速度乱外推。开关关时 accel 恒为0、predict里也不加。
                if (useConstantAcceleration_) {
                    if (dtFrames == 1) {
                        for (int c=0;c<3;++c) {
                            const double instAcc = tp.velocity[size_t(c)] - prevVel[size_t(c)];
                            tp.accel[size_t(c)] = velSmoothingAlpha_*instAcc
                                                + (1.0-velSmoothingAlpha_)*tp.accel[size_t(c)];
                        }
                    } else {
                        tp.accel = {0,0,0};
                    }
                }
                tp.position = newPos;
                tp.missedFrames = 0;
                tp.hits += 1;
                // 延迟领号：这条 tentative 恰好在这一帧攒够命中数转正，此刻才领
                // 一个编号(见上面生成处的完整说明)。只在从未领过号(id<0)时领。
                if (tp.id < 0 && tp.hits >= minHitsToConfirm_) tp.id = nextId_++;
                // justAcquired 的语义对外是"这一帧我第一次出现"——对开了确认
                // 机制的调用方来说，"第一次出现"指的是转正后首次对外可见那一帧。
                tp.justAcquired = (!wasConfirmed && tp.hits >= minHitsToConfirm_);
                trackAlive[ti] = 1;
            } else {
                tp.missedFrames += 1;
                tp.justAcquired = false;
                // tentative 一丢就死：还没攒够命中数就消失的，大概率是幽灵，
                // 不给记忆窗口。confirmed 才享受 maxMissedFrames 的遮挡记忆。
                if (!wasConfirmed) trackAlive[ti] = 0;
                else trackAlive[ti] = (tp.missedFrames <= maxMissedFrames_) ? 1 : 0;
            }
        }

        // 剩下没被任何已有轨迹认领的新观测：新开 tentative 轨迹。
        // 注意 ID 在这里就分配(而不是转正时)——转正瞬间不换号，调用方看到的
        // 第一帧编号就是最终编号。
        // 收集这一帧"没配上、但还活着"的已确认轨迹的当前位置。下面生成新
        // 轨迹时，落在它们附近的观测不再生成幽灵——因为那几乎必然是这条
        // 轨迹这帧因预测漂移(恒速外推偏了)没关联上的真观测，让它下一帧
        // 自己重新捡回来即可，而不是凭空造一个会短暂占号的新点。这是把
        // 近邻噪声点残余churn继续往下压的一刀。
        std::vector<std::array<double,3>> coastingConfirmed;
        for (size_t i=0;i<tracks_.size();++i) {
            if (!trackAlive[i]) continue;
            const InternalTrack& t = tracks_[i];
            if (t.hits >= minHitsToConfirm_ && t.missedFrames > 0)
                coastingConfirmed.push_back(t.position);
        }

        for (int k=0;k<n;++k) {
            if (claimed[size_t(k)]) continue;
            const std::array<double,3>& obs = frameTracks[size_t(k)].point;
            bool onCoasting = false;
            for (const auto& cp : coastingConfirmed) {
                const double dx=obs[0]-cp[0], dy=obs[1]-cp[1], dz=obs[2]-cp[2];
                if (std::sqrt(dx*dx+dy*dy+dz*dz) <= maxAssocDist_) { onCoasting = true; break; }
            }
            if (onCoasting) continue;   // 别在正在coast的确认点身上凭空造幽灵
            InternalTrack tp;
            // 【"编号一直增"的根治】编号不再在这里(生成即分配)领取。原来每
            // 生成一条 tentative 轨迹就 nextId_++，哪怕它下一帧就死、从没对外
            // 公开过，编号计数器也已经涨了一格——近邻点噪声下幽灵不停闪现，
            // 计数器就一路涨，屏幕上看到的就是"编号一直在增"。现在改成延迟
            // 分配：tentative 先不领号(id=-1)，等它真正攒够命中数【转正】的
            // 那一刻才领 nextId_。一闪就死的幽灵永远不消耗编号，计数器只为
            // 真正持久存在的点增长。(不开确认机制时 minHits<=1，生成即转正，
            // 保持旧语义立即领号。)
            tp.id = (minHitsToConfirm_ <= 1) ? nextId_++ : -1;
            tp.position = frameTracks[size_t(k)].point;
            tp.velocity = {0,0,0};
            tp.missedFrames = 0;
            tp.hits = 1;
            tp.justAcquired = (minHitsToConfirm_ <= 1);   // 不开确认机制时保持旧语义
            tracks_.push_back(tp);
            trackAlive.push_back(1);
        }

        // 清掉彻底销毁的轨迹（丢失太久 / tentative一丢就死）。
        std::vector<InternalTrack> kept;
        kept.reserve(tracks_.size());
        for (size_t i=0;i<tracks_.size();++i)
            if (trackAlive[i]) kept.push_back(tracks_[i]);
        tracks_.swap(kept);

        // 【新增·③输出端One Euro滤波】合成/采用时间戳——只在真正要用时才
        // 合成，避免没开这个功能的调用方白白维护一个计数器。
        if (useOutputFilter_) {
            nextSynthNs_ = (ts_ns >= 0) ? ts_ns : (nextSynthNs_ + nominalFrameNs_);
        }

        // 只对外发布已转正的轨迹。
        // ---- 逐点调试 ----
        // 【所有轨迹都记，不只是转正的】未转正的轨迹恰恰是"编号乱跳"的现场：
        // 一个真点如果反复在 tentative 阶段夭折，对外表现就是它不断拿新编号，
        // 而只记转正轨迹的话，夭折的那些在文件里根本不存在。
        dbg_ = TrackFrameDebug{};
        dbg_.ran = true;
        dbg_.nObs = int(frameTracks.size());
        dbg_.nTracks = int(tracks_.size());
        dbg_.maxAssocDistMm = maxAssocDist_;
        dbg_.chiSquareGate = -1.0;          // 这个后端没有马氏门
        for (const InternalTrack& t : tracks_) {
            TrackFrameDebugItem it;
            it.id = t.id;
            for (int k = 0; k < 3; ++k) {
                it.pos[k] = t.position[size_t(k)];
                it.vel[k] = t.velocity[size_t(k)];
            }
            it.missedFrames = t.missedFrames;
            it.hits         = t.hits;
            it.justAcquired = t.justAcquired;
            it.coasting     = (t.missedFrames > 0);
            it.confirmed    = (t.hits >= minHitsToConfirm_);
            if (it.confirmed) ++dbg_.nConfirmed;
            if (t.justAcquired) ++dbg_.nNewborn;
            dbg_.items.push_back(it);
        }

        std::vector<TrackedPoint> out;
        out.reserve(tracks_.size());
        for (InternalTrack& t : tracks_) {
            if (t.hits < minHitsToConfirm_) continue;
            TrackedPoint p;
            p.id = t.id;
            p.position = t.position;
            p.velocity = t.velocity;
            p.missedFrames = t.missedFrames;
            p.justAcquired = t.justAcquired;
            if (useOutputFilter_) {
                // 新转正的轨迹(justAcquired)是一个新身份，滤波器不能带着上一个
                // 身份的历史状态平滑到新点上——那样第一个输出点会被"拖"向
                // 滤波器还记得的旧值，等于人为制造一次虚假的位移轨迹。
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

private:
    struct InternalTrack {
        int id = -1;
        std::array<double,3> position{0,0,0};
        std::array<double,3> velocity{0,0,0};
        std::array<double,3> accel{0,0,0};   // 恒加速度模型用；useConstantAcceleration_ 关时恒为0、不参与预测
        int missedFrames = 0;
        int hits = 0;               // 累计被真正看到的帧数，>=minHitsToConfirm_ 即转正
        bool justAcquired = false;
        OneEuroFilter3 outFilter;   // ③输出端滤波器状态——只平滑对外发布值，不反馈进position/velocity本身
    };

    // 预测这一帧该点会出现在哪。恒速模型：pos + v*steps。开了恒加速度则再加
    // 0.5*a*steps² 项(急动/加速时更贴)。做成非static成员是因为要读开关。
    std::array<double,3> predict(const InternalTrack& tp) const {
        const double steps = double(tp.missedFrames + 1);
        std::array<double,3> p = {
            tp.position[0] + tp.velocity[0]*steps,
            tp.position[1] + tp.velocity[1]*steps,
            tp.position[2] + tp.velocity[2]*steps
        };
        if (useConstantAcceleration_) {
            const double h = 0.5*steps*steps;
            for (int c=0;c<3;++c) p[size_t(c)] += tp.accel[size_t(c)]*h;
        }
        return p;
    }

    std::vector<InternalTrack> tracks_;
    TrackFrameDebug dbg_{};
    double maxAssocDist_;
    int maxMissedFrames_;
    static constexpr double kMinEffectiveAssocDist = 3.0;   // 自适应上限的地板：比这更窄的两点多半是同点重复检测，不靠关联距离区分

    // ── 运行时行为开关的后备变量（默认=实测最稳的一组）──────────────
    bool   useOptimalAssignment_       = true;    // 全局最优指派(防互换)
    bool   useVelocitySmoothing_       = true;    // 速度EWMA平滑(抗噪churn)
    double velSmoothingAlpha_          = 0.4;
    bool   useAdaptiveAssocCap_        = false;   // 默认关：全局最优指派已防互换，收窄门控只会误杀抖动点
    double adaptiveCapMultiplier_      = 0.5;
    bool   useMissedFrameRelax_        = true;    // 丢帧期间放宽半径，利于遮挡后重新捡回、减少churn
    double relaxGrowthPerMissedFrame_  = 0.15;    // 每丢一帧半径按基准+15%
    double relaxCapMultiplier_         = 2.0;     // 最多放宽到基准的2倍
    bool   useConstantAcceleration_    = false;   // 默认关：恒速更稳，恒加速度在噪声上更容易过冲

    // ── 【新增·③输出端One Euro滤波】默认关(改变最终输出的原始信号，点云
    // 测试本身是拿来看"原始精度"的工具，不该默默替用户平滑掉；需要的场景
    // 手动开)。参数默认值取自三角化调试窗口里已验证的一组(minCutoff=30，
    // beta=0.5)，不是凭空新定。 ──────────────────────────────────────
    bool   useOutputFilter_ = false;
    double filterMinCutoff_ = 30.0;
    double filterBeta_      = 0.5;
    double filterDCutoff_   = 1.0;
    int64_t nominalFrameNs_ = 8'333'333;   // 调用方不传真实ts_ns时的合成帧间隔，默认≈120fps
    int64_t nextSynthNs_    = 0;

    int minHitsToConfirm_;
    int nextId_ = 0;

public:
    void setUseOutputFilter(bool v) { useOutputFilter_ = v; }
    void setOutputFilterParams(double minCutoff, double beta, double dCutoff = 1.0) {
        filterMinCutoff_ = minCutoff; filterBeta_ = beta; filterDCutoff_ = dCutoff;
        for (auto& t : tracks_) t.outFilter.setParams(minCutoff, beta, dCutoff);
    }
    // 调用方不提供真实ts_ns时的合成帧间隔(纳秒)，默认≈120fps。只在
    // useOutputFilter_开启且调用update()时不传ts_ns才用得上。
    void setNominalFrameNs(int64_t ns) { nominalFrameNs_ = ns > 0 ? ns : nominalFrameNs_; }
private:
};

} // namespace mocap
