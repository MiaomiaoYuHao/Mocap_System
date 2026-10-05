#pragma once
// ---------------------------------------------------------------------------
// 两视图对应性匹配（多点动捕对应性求解的第一个完整功能）—— 纯数学，零 Qt。
//
// 输入：cam0 的 m 个归一化观测、cam1 的 n 个归一化观测，以及这对相机的本质
// 矩阵 E（由 Epipolar.hpp 的 essentialFromRelativePose 预计算、缓存）。
// 输出：一组配对 (i,j)，表示 cam0 的第 i 个点和 cam1 的第 j 个点是同一物理点。
//
// 算法（对应主方案的"极线剪枝 + 全局最优指派"）：
//   1. 极线剪枝：对每一对 (i,j) 算 Sampson 距离，超过阈值的直接判为不可能，
//      代价设为无穷（不进指派）。这把 m×n 的搜索空间砍成稀疏。
//   2. 全局最优指派：在剩下的可行配对里，用匈牙利算法求"总 Sampson 代价
//      最小、且每个点最多配一次"的全局最优匹配——不是贪心"各自找最近"，
//      贪心会在两个点争抢同一对应时抢错。两视图指派是标准二部图匹配，
//      匈牙利算法多项式时间精确可解（NP-hard 只在三视图及以上出现）。
//
// 这里的匈牙利算法用经典的 O(k^3) 稠密实现（k=max(m,n)），对动捕这种每帧
// 几十个点的规模绰绰有余；真正的性能瓶颈在检测和三角化，不在这里。
//
// 注意：两视图匹配天然有"极线共线歧义"——如果 cam1 的两个点恰好落在同一条
// 极线上，光靠极线分不清谁是谁（Sampson 距离都很小）。这种歧义要靠第三台
// 相机的交叉验证来消解（多视图仲裁，下一层的活）。本层如实反映这一点：
// 歧义配对会被指派，但可以通过返回的 cost 看出它们"太接近、不够肯定"。
// ---------------------------------------------------------------------------
#include "reconstruct/Epipolar.hpp"
#include <array>
#include <vector>
#include <limits>
#include <algorithm>

namespace mocap {

struct MatchPair {
    int i = -1;        // cam0 的点下标
    int j = -1;        // cam1 的点下标
    double cost = 0;   // 这对的 Sampson 距离（越小越肯定）
};

struct TwoViewMatchResult {
    std::vector<MatchPair> pairs;   // 配上的对
    std::vector<int> unmatched0;    // cam0 里没配上的点下标
    std::vector<int> unmatched1;    // cam1 里没配上的点下标
};

namespace corr_detail {

// 匈牙利算法（Kuhn-Munkres），最小化总代价的完美/最大匹配。
// 输入 cost 是 n×n 方阵（行主序），返回 assign[r]=c 表示行 r 配到列 c。
// 用经典的 O(n^3) 势函数实现（Jonker-Volgenant 风格的标准版本）。
// 不可行配对用一个很大的值 BIG 填充，指派后调用方自行剔除 >=BIG 的配对。
inline std::vector<int> hungarian(const std::vector<double>& cost, int n) {
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> u(n + 1, 0), v(n + 1, 0);
    std::vector<int> p(n + 1, 0), way(n + 1, 0);
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(n + 1, INF);
        std::vector<char> used(n + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = -1;
            double delta = INF;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double cur = cost[size_t(i0 - 1) * n + (j - 1)] - u[i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else minv[j] -= delta;
            }
            j0 = j1;
        } while (p[j0] != 0);
        do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
    }
    std::vector<int> assign(n, -1);
    for (int j = 1; j <= n; ++j) if (p[j] > 0) assign[p[j] - 1] = j - 1;
    return assign;
}

} // namespace corr_detail

// 两视图多点匹配。obs0/obs1 是各相机的归一化观测 (nx,ny) 列表。
// maxSampson 是极线剪枝阈值（归一化坐标下，超过就判不可能配对）——典型取
// 值对应几个像素除以焦距，比如 3px / 1000px焦距 ≈ 0.003。
inline TwoViewMatchResult matchTwoViews(
        const EpiMat3& E,
        const std::vector<std::array<double,2>>& obs0,
        const std::vector<std::array<double,2>>& obs1,
        double maxSampson) {
    using namespace corr_detail;
    TwoViewMatchResult res;
    const int m = int(obs0.size()), n = int(obs1.size());
    if (m == 0 || n == 0) {
        for (int i = 0; i < m; ++i) res.unmatched0.push_back(i);
        for (int j = 0; j < n; ++j) res.unmatched1.push_back(j);
        return res;
    }

    // 方阵化：匈牙利要方阵，取 k=max(m,n)，多出来的行/列是"虚拟点"，
    // 代价填 BIG，最后被当成未匹配剔除。
    const int k = std::max(m, n);
    const double BIG = 1e12;
    const double maxSq = maxSampson * maxSampson;
    std::vector<double> cost(size_t(k) * k, BIG);

    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j) {
            const double dSq = sampsonDistanceSq(E, obs0[i][0], obs0[i][1],
                                                    obs1[j][0], obs1[j][1]);
            // 极线剪枝：超阈值的配对不可行，保持 BIG。
            cost[size_t(i) * k + j] = (dSq <= maxSq) ? dSq : BIG;
        }

    const std::vector<int> assign = hungarian(cost, k);

    std::vector<char> matched0(m, false), matched1(n, false);
    for (int i = 0; i < k; ++i) {
        const int j = assign[i];
        if (i >= m || j < 0 || j >= n) continue;
        if (cost[size_t(i) * k + j] >= BIG) continue;   // 虚拟/被剪枝的配对，不算
        MatchPair mp;
        mp.i = i; mp.j = j;
        mp.cost = std::sqrt(cost[size_t(i) * k + j]);   // 存 Sampson 距离本身（非平方）
        res.pairs.push_back(mp);
        matched0[i] = true; matched1[j] = true;
    }
    for (int i = 0; i < m; ++i) if (!matched0[i]) res.unmatched0.push_back(i);
    for (int j = 0; j < n; ++j) if (!matched1[j]) res.unmatched1.push_back(j);
    return res;
}

} // namespace mocap
