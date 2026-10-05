#pragma once
// ---------------------------------------------------------------------------
// 手指链分组（从"手背刚体已标定、剩下的候选点是谁的手指"这个问题出发）——
// 纯数学，零 Qt 依赖。
//
// 前提：调用方已经用 HandRigidAutoCalib.hpp 找出手背刚体、并把每帧候选点
// 转换到腕部局部系；本文件只处理"刚体5点之外剩下的候选点，怎么分成5组
// 各3个，喂给 FingerKinematicCalib.hpp"这一步，不关心刚体、也不关心具体
// 拟合数学。
//
// 手指之间没有"距离不变"这个铁律可用（同一根手指相邻两颗球之间的距离本身
// 就随关节屈伸变化），没法照搬手背那套。这里靠的是运动的时间模式，提供
// 两级方案，可靠性依次降低、灵活性依次提高：
//
//   Tier 1（segmentByTimeWindows，引导式）：采集协议里明确告诉使用者"现在
//   只动第N根手指"，每根手指对应一个已知的时间窗口。窗口内活跃度
//   (帧间位移幅度总和)最高的3个候选点即为该指的球——因为按引导协议，
//   同一时刻只有一根手指在动。可靠性最高，因为分组判据不依赖手指之间
//   运动是否碰巧不相关，而是由采集流程本身保证。
//
//   Tier 2（segmentByMotionCorrelation，自由式）：不要求依次动，允许同时
//   动多根，只要求不同手指的运动模式在时间上不完全同步（比如各自独立
//   摆动、不同节奏）。判据是"活跃度时间序列的相关系数"——同一根手指的
//   3颗球机械相连，动的时候几乎同时变活跃/变不活跃，相关系数高；不同
//   手指相互独立运动，相关系数低。找相关系数达标的点两两连边，在这个图
//   上找极大团(复用 HandRigidAutoCalib.hpp 里现成的 Bron-Kerbosch)——
//   团大小恰好等于3才自动采纳，大于3(比如两根手指联动，常见于抓握动作)
//   只报告不自动拆分，避免把两根手指的球混进同一组。
//
// 两级方案共用同一个"近/中/远节排序"判据：手指是一个串联链，越靠近手背
// 的关节，运动幅度(位置方差)越小；沿链条往指尖走，运动幅度逐级放大——这
// 是运动学结构本身决定的，不依赖具体连杆长度，可以直接拿方差排序，不用
// 猜哪个是第几节。
//
// 诚实失败：候选点不足3个、活跃度/相关性区分度不够、团尺寸超出预期，都在
// message 里给出具体原因，不悍自拼凑分组。
// ---------------------------------------------------------------------------
#include "estimate/HandRigidAutoCalib.hpp"   // 复用 FrameTrackedPoints + Bron-Kerbosch
#include "estimate/PointIEKF.hpp"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <string>
#include <cstdio>

namespace mocap {

struct FingerGroupCandidate {
    std::vector<int> orderedIds;   // 恰好3个，已按近->中->远排好序
    double confidence = 0.0;       // 诊断量：Tier1是第3活跃点的活跃度，Tier2固定给1.0(已过团校验)
};

struct FingerSegmentationResult {
    bool valid = false;
    std::vector<FingerGroupCandidate> groups;
    std::vector<int> unassignedIds;   // 没能分进任何组的候选点(诊断用)
    std::string message;
};

namespace finger_seg_detail {

// 按"离自身轨迹均值的位置方差"从小到大排序 = 近节->中节->远节。纯粹是
// 运动学结构决定的(离手背越近的关节运动幅度越小)，不依赖连杆长度数值。
inline std::vector<int> orderByVariance(const std::vector<int>& ids,
                                        const std::unordered_map<int, std::vector<Vec3>>& trajectories) {
    std::vector<std::pair<double,int>> varId;
    for (int id : ids) {
        const auto& traj = trajectories.at(id);
        Vec3 mean{0,0,0};
        for (auto& p : traj) for (int k=0;k<3;++k) mean[size_t(k)] += p[size_t(k)] / double(traj.size());
        double var = 0.0;
        for (auto& p : traj) {
            const double dx=p[0]-mean[0], dy=p[1]-mean[1], dz=p[2]-mean[2];
            var += (dx*dx+dy*dy+dz*dz) / double(traj.size());
        }
        varId.push_back({var, id});
    }
    std::sort(varId.begin(), varId.end());
    std::vector<int> out;
    out.reserve(varId.size());
    for (auto& vi : varId) out.push_back(vi.second);
    return out;
}

inline std::unordered_map<int, std::unordered_map<int, Vec3>> buildByIdFrame(
        const std::vector<FrameTrackedPoints>& frames) {
    std::unordered_map<int, std::unordered_map<int, Vec3>> byIdFrame;
    for (int f = 0; f < int(frames.size()); ++f) {
        const auto& fr = frames[size_t(f)];
        const size_t m = std::min(fr.ids.size(), fr.positions.size());
        for (size_t k = 0; k < m; ++k) byIdFrame[fr.ids[k]][f] = fr.positions[k];
    }
    return byIdFrame;
}

} // namespace finger_seg_detail

// ===========================================================================
// Tier 1：引导式，按已知时间窗口分组。
// ===========================================================================

struct TimeWindow { int startFrame = 0; int endFrame = 0; };   // 半开区间 [start, end)

struct TimeWindowSegConfig {
    double minActivityRatio = 3.0;   // 第3活跃点/第4活跃点的活跃度比值门槛(区分度)
    int minFramesPerWindow = 20;
};

inline FingerSegmentationResult segmentByTimeWindows(
        const std::vector<FrameTrackedPoints>& frames,
        const std::vector<TimeWindow>& windows,
        const TimeWindowSegConfig& cfg = TimeWindowSegConfig{}) {
    using namespace finger_seg_detail;
    FingerSegmentationResult out;

    const auto byIdFrame = buildByIdFrame(frames);
    std::vector<int> usedIds;

    for (size_t w = 0; w < windows.size(); ++w) {
        const auto& win = windows[w];
        if (win.endFrame - win.startFrame < cfg.minFramesPerWindow) {
            out.message += "窗口" + std::to_string(w) + "帧数不足(" +
                          std::to_string(win.endFrame - win.startFrame) + "帧)，跳过；";
            continue;
        }

        struct Activity { int id; double energy; int framesInWindow; };
        std::vector<Activity> acts;
        for (auto& kv : byIdFrame) {
            const int id = kv.first;
            if (std::find(usedIds.begin(), usedIds.end(), id) != usedIds.end()) continue;

            std::vector<std::pair<int, Vec3>> pts;
            for (auto& fp : kv.second)
                if (fp.first >= win.startFrame && fp.first < win.endFrame) pts.push_back(fp);
            if (int(pts.size()) < cfg.minFramesPerWindow) continue;
            std::sort(pts.begin(), pts.end(), [](auto&a,auto&b){ return a.first<b.first; });

            double energy = 0.0;
            for (size_t i = 1; i < pts.size(); ++i) {
                const auto& a = pts[i-1].second; const auto& b = pts[i].second;
                const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
                energy += std::sqrt(dx*dx+dy*dy+dz*dz);
            }
            acts.push_back({id, energy, int(pts.size())});
        }

        if (int(acts.size()) < 3) {
            out.message += "窗口" + std::to_string(w) + "候选点不足3个(实际" +
                          std::to_string(acts.size()) + "个)，跳过；";
            continue;
        }
        std::sort(acts.begin(), acts.end(), [](auto& a, auto& b) { return a.energy > b.energy; });

        if (acts.size() > 3) {
            const double ratio = acts[2].energy / std::max(1e-6, acts[3].energy);
            if (ratio < cfg.minActivityRatio) {
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                    "窗口%zu第3/第4活跃点区分度不足(比值%.2f，门槛%.2f)，可能有其它手指同时动了或该窗口内运动噪声偏大，跳过；",
                    w, ratio, cfg.minActivityRatio);
                out.message += buf;
                continue;
            }
        }

        std::vector<int> top3 = { acts[0].id, acts[1].id, acts[2].id };
        std::unordered_map<int, std::vector<Vec3>> traj;
        for (int id : top3) {
            std::vector<Vec3> pts;
            for (auto& fp : byIdFrame.at(id)) pts.push_back(fp.second);
            traj[id] = pts;
        }
        const auto ordered = orderByVariance(top3, traj);

        FingerGroupCandidate g;
        g.orderedIds = ordered;
        g.confidence = acts[2].energy;
        out.groups.push_back(g);
        for (int id : ordered) usedIds.push_back(id);
    }

    out.valid = !out.groups.empty();
    if (!out.valid && out.message.empty()) out.message = "没有任何窗口成功分组。";
    return out;
}

// ===========================================================================
// Tier 2：自由式，按运动相关性聚类分组（不要求按顺序依次动，只要求不同
// 手指的运动模式在时间上不完全同步）。
// ===========================================================================

struct MotionCorrSegConfig {
    double minCorrelation = 0.75;
    int minCoOccurFrames = 30;
    int targetGroupSize = 3;
};

inline FingerSegmentationResult segmentByMotionCorrelation(
        const std::vector<FrameTrackedPoints>& frames,
        const MotionCorrSegConfig& cfg = MotionCorrSegConfig{}) {
    using namespace finger_seg_detail;
    FingerSegmentationResult out;

    const auto byIdFrame = buildByIdFrame(frames);
    std::vector<int> ids;
    for (auto& kv : byIdFrame) ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end());

    // 每个点的"帧间位移向量"时间序列(只用连续帧对，跳变的帧对不计入，
    // 避免偶发丢帧被误当成一次大位移)。注意：这里存的是带符号的位移
    // 向量，不是取模长的标量——如果只用模长(幅度)算相关性，不同频率的
    // 独立运动会因为"都非负、都有周期性起伏"产生虚高的相关系数，跟是否
    // 真的同步运动无关(这是本文件调试阶段实测踩到的一个坑，不是假设)。
    // 用位移向量本身的点积(类似余弦相似度)，方向不同的运动会互相抵消，
    // 只有真正同步(方向也一致)的运动才会有高相关。
    std::unordered_map<int, std::unordered_map<int, Vec3>> activity;
    for (int id : ids) {
        const auto& fp = byIdFrame.at(id);
        std::vector<int> fs;
        for (auto& kv : fp) fs.push_back(kv.first);
        std::sort(fs.begin(), fs.end());
        for (size_t i = 1; i < fs.size(); ++i) {
            if (fs[i] != fs[i-1] + 1) continue;
            const auto& a = fp.at(fs[i-1]); const auto& b = fp.at(fs[i]);
            activity[id][fs[i-1]] = { b[0]-a[0], b[1]-a[1], b[2]-a[2] };
        }
    }

    const int n = int(ids.size());
    std::vector<std::vector<bool>> adj(size_t(n), std::vector<bool>(size_t(n), false));
    int bestCoFrames = 0;
    double bestCorr = -2.0;

    for (int a = 0; a < n; ++a) {
        const auto& actA = activity[ids[size_t(a)]];
        for (int b = a + 1; b < n; ++b) {
            const auto& actB = activity[ids[size_t(b)]];
            double dot = 0.0, normA = 0.0, normB = 0.0;
            int coFrames = 0;
            for (auto& kv : actA) {
                auto it = actB.find(kv.first);
                if (it == actB.end()) continue;
                const Vec3& va = kv.second; const Vec3& vb = it->second;
                dot += va[0]*vb[0] + va[1]*vb[1] + va[2]*vb[2];
                normA += va[0]*va[0] + va[1]*va[1] + va[2]*va[2];
                normB += vb[0]*vb[0] + vb[1]*vb[1] + vb[2]*vb[2];
                ++coFrames;
            }
            if (coFrames < cfg.minCoOccurFrames) continue;
            const double denom = std::sqrt(normA * normB);
            const double corr = denom > 1e-9 ? dot / denom : 0.0;

            if (coFrames > bestCoFrames || (coFrames==bestCoFrames && corr>bestCorr)) {
                bestCoFrames = coFrames; bestCorr = corr;
            }
            if (corr >= cfg.minCorrelation) adj[size_t(a)][size_t(b)] = adj[size_t(b)][size_t(a)] = true;
        }
    }

    const size_t un = size_t(n);
    std::vector<int> P(un);
    for (int i = 0; i < n; ++i) P[size_t(i)] = i;
    std::vector<std::vector<int>> cliques;
    { std::vector<int> R, X; rigid_calib_detail::bronKerbosch(R, P, X, adj, cliques); }

    std::vector<int> assigned;
    for (auto& c : cliques) {
        if (int(c.size()) < cfg.targetGroupSize) continue;
        std::vector<int> memberIds;
        for (int idx : c) memberIds.push_back(ids[size_t(idx)]);

        if (int(c.size()) == cfg.targetGroupSize) {
            std::unordered_map<int, std::vector<Vec3>> traj;
            for (int id : memberIds) {
                std::vector<Vec3> pts;
                for (auto& fp : byIdFrame.at(id)) pts.push_back(fp.second);
                traj[id] = pts;
            }
            const auto ordered = orderByVariance(memberIds, traj);
            FingerGroupCandidate g;
            g.orderedIds = ordered;
            g.confidence = 1.0;
            out.groups.push_back(g);
            for (int id : ordered) assigned.push_back(id);
        } else {
            out.message += "发现一组 " + std::to_string(c.size()) +
                          " 点高度相关运动的候选(超过预期的3点/指，可能多根手指联动了，比如握拳时常见)，未自动采纳；";
        }
    }

    for (int id : ids)
        if (std::find(assigned.begin(), assigned.end(), id) == assigned.end()) out.unassignedIds.push_back(id);

    out.valid = !out.groups.empty();
    if (out.groups.empty()) {
        if (bestCoFrames == 0) {
            out.message = "没有任何一对候选点同时出现超过 " + std::to_string(cfg.minCoOccurFrames) +
                         " 帧，采集时长可能不够，或点的跨帧关联持续丢失。";
        } else {
            char buf[320];
            std::snprintf(buf, sizeof(buf),
                "没有找到运动相关性达标的3点分组(最高相关系数 %.2f，门槛 %.2f)。"
                "可能各手指运动幅度太小、太同步(没有独立摆动)，或采集时长不够。",
                bestCorr, cfg.minCorrelation);
            out.message = buf;
        }
    } else {
        out.message += "共分出 " + std::to_string(out.groups.size()) + " 组三点手指候选，" +
                      std::to_string(out.unassignedIds.size()) + " 个点未能分组。";
    }
    return out;
}

} // namespace mocap
