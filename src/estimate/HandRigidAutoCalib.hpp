#pragma once
// ---------------------------------------------------------------------------
// 手部刚体自动标定（手背5点，或任意数量的刚体反光球）—— 纯数学，零 Qt 依赖。
//
// 解决的问题：不预先知道贴了几颗球、贴在哪，只知道"晃动手一段时间，采集到的
// 若干候选3D点轨迹里，有一撮点两两之间的距离全程不变"——这撮点就是同一个
// 刚体（手背）。跟运动捕捉系统"新建刚体(Create Rigid Body)"标定同一个思路。
//
// 前提假设（调用方负责满足，不在本文件职责范围内）：
//   1. 每帧的候选3D点已经带了跨帧一致的track id（比如用 reconstruct/
//      TemporalTracker.hpp 的输出）——本文件不解决"点的身份对应"问题，只
//      解决"哪些身份一致的点构成刚体、局部形状是什么"这个问题。
//   2. 采集过程里手有足够的姿态变化（转动/平移），不能全程几乎不动——
//      静止不动时任何一撮点的两两距离当然都"稳定"，会把手指上偶然聚在一起
//      的点也误判成刚体，姿态变化越充分越能把真刚体和"暂时凑巧不动的一群
//      点"区分开。
//
// 算法四步：
//   1. 对每一对同时出现过的候选点，统计它们在共同帧里的距离均值/标准差；
//      标准差 <= 容差的算一条"稳定距离边"。
//   2. 在"稳定距离边"构成的图上找所有极大团(maximal clique)——要求团内
//      任意两点都稳定，而不只是连通，避免"A-B稳、B-C稳，但A-C不稳"这种
//      传递性陷阱被误判成同一个刚体。
//   3. 对每个够大的团，用它的距离矩阵做经典多维标度(MDS)重建出局部坐标
//      （只到"任意刚体变换+可能的镜像"的程度，绝对朝向未定，这对后续
//      Kabsch 配准无所谓——见 HandColdStart.hpp 同款逻辑）。
//   4. 镜像消歧：用重建出的模板(以及它的镜像版本)对采集期间的每一帧做
//      Kabsch 配准，哪个版本残差更小就是正确的那个——这跟 README §1.1
//      手背翻转歧义的解决思路是同一件事，只是这里反过来用它验证重建结果。
//
// 诚实失败：任何一步数据不够撑起判断，都不该硬凑一个结果，而是在
// RigidAutoCalibResult::message 里给出具体原因和可执行的建议（哪一步不
// 够、大概需要什么），不是简单一句"失败了"。
// ---------------------------------------------------------------------------
#include "estimate/PointIEKF.hpp"   // Vec3
#include "hand/HandPose.hpp"        // HandVec3/HandMat3/kabsch/matvec3（复用同一套SVD/Kabsch，不重新发明）
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <string>
#include <cstdio>

namespace mocap {

// 一帧里所有候选点的位置，配跨帧一致的 track id。
struct FrameTrackedPoints {
    std::vector<int> ids;
    std::vector<Vec3> positions;   // 与 ids 一一对应，长度必须相等
};

struct RigidClusterCandidate {
    std::vector<int> ids;             // 参与该刚体的 track id（原始顺序，无特定含义）
    std::vector<Vec3> localTemplate;  // 重建出的局部模板坐标，与 ids 一一对应（已消歧完成的正确朝向）
    double maxPairStdMm = 0.0;        // 簇内所有点对里，帧间距离标准差的最大值——越小说明"确实是刚体"这件事越可信
    double avgKabschRmsMm = 0.0;      // 用重建模板对采集帧逐帧做Kabsch配准的平均残差——同样是可信度诊断量
    int framesUsed = 0;               // 参与验证的总帧数（不是共视帧数，是采集总帧数，供调用方判断样本量够不够）
};

struct RigidAutoCalibResult {
    bool valid = false;
    std::vector<RigidClusterCandidate> clusters;   // 按点数降序；valid=true 时至少有1个
    std::string message;   // valid=false: 失败原因+建议；valid=true: 找到几组候选、是否需要人工确认
};

struct RigidAutoCalibConfig {
    double pairStdToleranceMm = 1.5;   // 两点间距离的帧间标准差容忍上限
    int minCoOccurFrames = 20;         // 一对点至少要共同出现这么多帧才纳入统计（太少的话标准差本身不可信）
    int minClusterPoints = 3;          // 至少3点才能解算6DOF刚体位姿（见 README §1.1 冗余度预算）
};

namespace rigid_calib_detail {

// 对称矩阵通用 Jacobi 特征值分解（经典 cyclic Jacobi，Golub & Van Loan 版本的
// 旋转公式，比 atan2 版本数值上更稳）。A 输入时是待分解的对称矩阵(行主序,
// 会被就地破坏)，n 是阶数；输出特征值降序、V 每一列是对应特征向量。
// 团的规模在标定场景里很小(几到十几个点)，cyclic sweep 足够快，不需要更
//复杂的QR算法。
inline void jacobiEigenSymN(std::vector<double> A, int n,
                            std::vector<double>& eigvals, std::vector<double>& V) {
    auto idx = [n](int r, int c) { return size_t(r) * size_t(n) + size_t(c); };

    V.assign(size_t(n) * size_t(n), 0.0);
    for (int i = 0; i < n; ++i) V[idx(i, i)] = 1.0;

    const int maxSweeps = 100;
    for (int sweep = 0; sweep < maxSweeps; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < n; ++p)
            for (int q = p + 1; q < n; ++q) off += A[idx(p, q)] * A[idx(p, q)];
        if (off < 1e-20) break;

        for (int p = 0; p < n; ++p) {
            for (int q = p + 1; q < n; ++q) {
                const double apq = A[idx(p, q)];
                if (std::abs(apq) < 1e-300) continue;
                const double app = A[idx(p, p)], aqq = A[idx(q, q)];
                const double theta = (aqq - app) / (2.0 * apq);
                const double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(1.0 + theta * theta));
                const double c = 1.0 / std::sqrt(1.0 + t * t);
                const double s = t * c;

                for (int k = 0; k < n; ++k) {
                    if (k == p || k == q) continue;
                    const double akp = A[idx(k, p)], akq = A[idx(k, q)];
                    A[idx(k, p)] = A[idx(p, k)] = c * akp - s * akq;
                    A[idx(k, q)] = A[idx(q, k)] = s * akp + c * akq;
                }
                A[idx(p, p)] = app - t * apq;
                A[idx(q, q)] = aqq + t * apq;
                A[idx(p, q)] = A[idx(q, p)] = 0.0;

                for (int k = 0; k < n; ++k) {
                    const double vkp = V[idx(k, p)], vkq = V[idx(k, q)];
                    V[idx(k, p)] = c * vkp - s * vkq;
                    V[idx(k, q)] = s * vkp + c * vkq;
                }
            }
        }
    }

    eigvals.assign(size_t(n), 0.0);
    for (int i = 0; i < n; ++i) eigvals[size_t(i)] = A[idx(i, i)];

    const size_t un = size_t(n);
    std::vector<int> order(un);
    for (int i = 0; i < n; ++i) order[size_t(i)] = i;
    std::sort(order.begin(), order.end(),
             [&](int a, int b) { return eigvals[size_t(a)] > eigvals[size_t(b)]; });

    std::vector<double> eigvalsSorted(un);
    std::vector<double> Vsorted(un * un);
    for (int newc = 0; newc < n; ++newc) {
        const int oldc = order[size_t(newc)];
        eigvalsSorted[size_t(newc)] = eigvals[size_t(oldc)];
        for (int r = 0; r < n; ++r) Vsorted[idx(r, newc)] = V[idx(r, oldc)];
    }
    eigvals = eigvalsSorted;
    V = Vsorted;
}

// Bron-Kerbosch（带简单pivot选取）：枚举无向图里全部极大团。图很小
// （标定场景候选点数量通常几到几十个），不需要更精细的分支限界。
inline void bronKerbosch(std::vector<int>& R, std::vector<int> P, std::vector<int> X,
                         const std::vector<std::vector<bool>>& adj,
                         std::vector<std::vector<int>>& cliques) {
    if (P.empty() && X.empty()) {
        if (!R.empty()) cliques.push_back(R);
        return;
    }
    std::vector<int> PX = P; PX.insert(PX.end(), X.begin(), X.end());
    int pivot = -1; size_t bestDeg = 0;
    for (int u : PX) {
        size_t deg = 0;
        for (int v : P) if (adj[size_t(u)][size_t(v)]) ++deg;
        if (pivot < 0 || deg > bestDeg) { pivot = u; bestDeg = deg; }
    }
    std::vector<int> candidates;
    for (int v : P) if (pivot < 0 || !adj[size_t(pivot)][size_t(v)]) candidates.push_back(v);

    for (int v : candidates) {
        std::vector<int> newP, newX;
        for (int u : P) if (adj[size_t(v)][size_t(u)]) newP.push_back(u);
        for (int u : X) if (adj[size_t(v)][size_t(u)]) newX.push_back(u);
        R.push_back(v);
        bronKerbosch(R, newP, newX, adj, cliques);
        R.pop_back();
        P.erase(std::remove(P.begin(), P.end(), v), P.end());
        X.push_back(v);
    }
}

} // namespace rigid_calib_detail

inline RigidAutoCalibResult calibrateRigidClusters(const std::vector<FrameTrackedPoints>& frames,
                                                   const RigidAutoCalibConfig& cfg = RigidAutoCalibConfig{}) {
    using namespace rigid_calib_detail;
    RigidAutoCalibResult out;

    // 1. 汇总每个 track id 在哪些帧出现、位置是什么。
    std::unordered_map<int, std::vector<std::pair<int, Vec3>>> track;
    for (int f = 0; f < int(frames.size()); ++f) {
        const auto& fr = frames[size_t(f)];
        const size_t m = std::min(fr.ids.size(), fr.positions.size());
        for (size_t k = 0; k < m; ++k) track[fr.ids[k]].push_back({f, fr.positions[k]});
    }

    if (int(track.size()) < cfg.minClusterPoints) {
        out.valid = false;
        out.message = "候选点(track)总数不足 " + std::to_string(cfg.minClusterPoints) +
                     " 个（实际 " + std::to_string(track.size()) +
                     " 个），无法构成刚体。检查检测层是否正常输出、跨帧关联(track)有没有大量丢失。";
        return out;
    }

    std::vector<int> ids;
    ids.reserve(track.size());
    for (auto& kv : track) ids.push_back(kv.first);
    std::sort(ids.begin(), ids.end());
    const int n = int(ids.size());

    // 2. 两两算距离均值/标准差，建立"稳定距离"邻接图。
    std::vector<std::vector<bool>> adj(size_t(n), std::vector<bool>(size_t(n), false));
    std::vector<std::vector<double>> meanDist(size_t(n), std::vector<double>(size_t(n), 0.0));
    std::vector<std::vector<double>> stdDist(size_t(n), std::vector<double>(size_t(n), 0.0));

    int bestPairFrames = 0;
    double bestPairStd = -1.0;

    for (int a = 0; a < n; ++a) {
        const auto& ta = track[ids[size_t(a)]];
        std::unordered_map<int, Vec3> byFrameA;
        for (auto& p : ta) byFrameA[p.first] = p.second;

        for (int b = a + 1; b < n; ++b) {
            const auto& tb = track[ids[size_t(b)]];
            std::vector<double> dists;
            for (auto& p : tb) {
                auto it = byFrameA.find(p.first);
                if (it == byFrameA.end()) continue;
                const Vec3& pa = it->second;
                const Vec3& pb = p.second;
                const double dx = pa[0]-pb[0], dy = pa[1]-pb[1], dz = pa[2]-pb[2];
                dists.push_back(std::sqrt(dx*dx + dy*dy + dz*dz));
            }
            if (int(dists.size()) < cfg.minCoOccurFrames) continue;

            double mean = 0.0;
            for (double d : dists) mean += d;
            mean /= double(dists.size());
            double var = 0.0;
            for (double d : dists) var += (d - mean) * (d - mean);
            var /= double(dists.size());
            const double sd = std::sqrt(var);

            meanDist[size_t(a)][size_t(b)] = meanDist[size_t(b)][size_t(a)] = mean;
            stdDist[size_t(a)][size_t(b)] = stdDist[size_t(b)][size_t(a)] = sd;

            if (int(dists.size()) > bestPairFrames ||
               (int(dists.size()) == bestPairFrames && (bestPairStd < 0 || sd < bestPairStd))) {
                bestPairFrames = int(dists.size());
                bestPairStd = sd;
            }
            if (sd <= cfg.pairStdToleranceMm) adj[size_t(a)][size_t(b)] = adj[size_t(b)][size_t(a)] = true;
        }
    }

    // 3. 极大团 = 刚体候选。
    const size_t unP = size_t(n);
    std::vector<int> P(unP);
    for (int i = 0; i < n; ++i) P[size_t(i)] = i;
    std::vector<std::vector<int>> cliques;
    { std::vector<int> R, X; bronKerbosch(R, P, X, adj, cliques); }

    std::vector<std::vector<int>> bigCliques;
    for (auto& c : cliques) if (int(c.size()) >= cfg.minClusterPoints) bigCliques.push_back(c);

    if (bigCliques.empty()) {
        out.valid = false;
        if (bestPairFrames == 0) {
            out.message = "没有任何一对候选点同时出现超过 " + std::to_string(cfg.minCoOccurFrames) +
                         " 帧。采集时长可能不够，或检测/关联层持续丢点导致点的身份对应不上，建议延长采集"
                         "时间、确认采集期间待标定的点持续在相机视野内。";
        } else {
            char buf[512];
            std::snprintf(buf, sizeof(buf),
                "没有找到 >= %d 个点两两距离都稳定的刚体候选（最接近的一对是 %d 帧共视、距离标准差 %.2fmm）。"
                "可能原因：① 采集期间手的朝向变化不够充分，偶然稳定的点组合没被有效排除，建议加大转动/"
                "平移幅度；② 实际贴的刚体点数不足 %d 颗；③ 检测噪声偏大，可以放宽 pairStdToleranceMm 再试。",
                cfg.minClusterPoints, bestPairFrames, bestPairStd, cfg.minClusterPoints);
            out.message = buf;
        }
        return out;
    }

    // 4. 逐个候选团：MDS重建局部模板 + 镜像消歧。
    for (auto& clique : bigCliques) {
        RigidClusterCandidate rc;
        const int m = int(clique.size());
        rc.ids.reserve(size_t(m));
        for (int c : clique) rc.ids.push_back(ids[size_t(c)]);

        std::vector<double> D2(size_t(m) * size_t(m), 0.0);
        double worstStd = 0.0;
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < m; ++j) {
                if (i == j) continue;
                const double d = meanDist[size_t(clique[size_t(i)])][size_t(clique[size_t(j)])];
                D2[size_t(i) * size_t(m) + size_t(j)] = d * d;
                worstStd = std::max(worstStd, stdDist[size_t(clique[size_t(i)])][size_t(clique[size_t(j)])]);
            }
        }
        rc.maxPairStdMm = worstStd;

        // 经典 MDS：双中心化。
        std::vector<double> rowMean(size_t(m), 0.0);
        double grandMean = 0.0;
        for (int i = 0; i < m; ++i) {
            double s = 0.0;
            for (int j = 0; j < m; ++j) s += D2[size_t(i) * size_t(m) + size_t(j)];
            rowMean[size_t(i)] = s / double(m);
            grandMean += s;
        }
        grandMean /= double(m) * double(m);

        std::vector<double> B(size_t(m) * size_t(m), 0.0);
        for (int i = 0; i < m; ++i)
            for (int j = 0; j < m; ++j)
                B[size_t(i) * size_t(m) + size_t(j)] =
                    -0.5 * (D2[size_t(i) * size_t(m) + size_t(j)] - rowMean[size_t(i)] - rowMean[size_t(j)] + grandMean);

        std::vector<double> eigvals, V;
        jacobiEigenSymN(B, m, eigvals, V);

        rc.localTemplate.assign(size_t(m), Vec3{0, 0, 0});
        for (int axis = 0; axis < 3 && axis < m; ++axis) {
            const double lam = std::max(0.0, eigvals[size_t(axis)]);
            const double s = std::sqrt(lam);
            for (int i = 0; i < m; ++i)
                rc.localTemplate[size_t(i)][size_t(axis)] = V[size_t(i) * size_t(m) + size_t(axis)] * s;
        }

        // 镜像消歧：模板本身和它的Z轴镜像版本，哪个对采集帧做Kabsch配准的
        // 平均残差更小就是对的——跟 README §1.1 手背翻转歧义的解决思路
        // 是同一件事，只是这里反过来用它验证/挑选重建结果。
        auto tryHandedness = [&](bool flipZ) {
            const size_t um = size_t(m);
            std::vector<HandVec3> templ(um);
            for (int i = 0; i < m; ++i) {
                templ[size_t(i)] = { rc.localTemplate[size_t(i)][0], rc.localTemplate[size_t(i)][1],
                                     flipZ ? -rc.localTemplate[size_t(i)][2] : rc.localTemplate[size_t(i)][2] };
            }
            double sumRms = 0.0;
            int framesUsed = 0;
            for (const auto& fr : frames) {
                std::vector<HandVec3> Pp, Qq;
                for (int i = 0; i < m; ++i) {
                    const int gid = rc.ids[size_t(i)];
                    for (size_t k = 0; k < fr.ids.size(); ++k) {
                        if (fr.ids[k] == gid) {
                            Pp.push_back(templ[size_t(i)]);
                            Qq.push_back({fr.positions[k][0], fr.positions[k][1], fr.positions[k][2]});
                            break;
                        }
                    }
                }
                if (int(Pp.size()) < 3) continue;
                HandMat3 R; HandVec3 t;
                mocap::kabsch(Pp, Qq, R, t);
                double sq = 0.0;
                for (size_t i = 0; i < Pp.size(); ++i) {
                    const HandVec3 rp = detail::matvec3(R, Pp[i]);
                    const double ex = rp[0]+t[0]-Qq[i][0], ey = rp[1]+t[1]-Qq[i][1], ez = rp[2]+t[2]-Qq[i][2];
                    sq += ex*ex + ey*ey + ez*ez;
                }
                sumRms += std::sqrt(sq / double(Pp.size()));
                ++framesUsed;
            }
            return framesUsed > 0 ? sumRms / double(framesUsed) : 1e18;
        };

        const double rmsNormal = tryHandedness(false);
        const double rmsFlipped = tryHandedness(true);
        if (rmsFlipped < rmsNormal) {
            for (auto& p : rc.localTemplate) p[2] = -p[2];
            rc.avgKabschRmsMm = rmsFlipped;
        } else {
            rc.avgKabschRmsMm = rmsNormal;
        }
        rc.framesUsed = int(frames.size());
        out.clusters.push_back(rc);
    }

    std::sort(out.clusters.begin(), out.clusters.end(),
             [](const RigidClusterCandidate& a, const RigidClusterCandidate& b) {
                 return a.ids.size() > b.ids.size();
             });

    out.valid = true;
    if (out.clusters.size() > 1) {
        std::string sizesStr;
        for (auto& c : out.clusters) sizesStr += std::to_string(c.ids.size()) + ",";
        out.message = "找到 " + std::to_string(out.clusters.size()) + " 组刚体候选（点数分别为 " + sizesStr +
                     "）。如果你只贴了一组待标定的刚体，建议采集时把其它反光物移出视野再重来，"
                     "或直接采用点数最多的一组（clusters[0]）。";
    } else {
        out.message = "找到 1 组刚体候选，点数 " + std::to_string(out.clusters[0].ids.size()) +
                     "，平均Kabsch残差 " + std::to_string(out.clusters[0].avgKabschRmsMm) + "mm。";
    }
    return out;
}

} // namespace mocap
