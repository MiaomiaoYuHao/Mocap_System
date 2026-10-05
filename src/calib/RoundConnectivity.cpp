#include "calib/RoundConnectivity.hpp"
#include <QHash>

namespace mocap {

QVector<int> unionFindComponents(int n, const QVector<QPair<int, int>>& edges) {
    QVector<int> parent(n);
    for (int i = 0; i < n; ++i) parent[i] = i;

    // 路径减半的迭代版 find（不用递归，n 很小时无所谓，但避免引入 <functional>
    // 依赖，跟原来 CalibWizard.cpp 里那份用 std::function 的写法等价）。
    auto find = [&](int x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };

    for (const auto& e : edges) {
        const int ra = find(e.first), rb = find(e.second);
        if (ra != rb) parent[ra] = rb;
    }

    QVector<int> comp(n);
    for (int i = 0; i < n; ++i) comp[i] = find(i);
    return comp;
}

bool allConnected(const QVector<int>& comp) {
    if (comp.size() < 2) return false;   // 少于两台相机，谈不上"已连通"
    for (int i = 1; i < comp.size(); ++i)
        if (comp[i] != comp[0]) return false;
    return true;
}

MainComponentResult findMainComponent(const QVector<int>& comp) {
    MainComponentResult r;
    if (comp.isEmpty()) return r;

    QHash<int, int> sizeOf;
    for (int c : comp) sizeOf[c] = sizeOf.value(c, 0) + 1;

    int mainComp = comp[0], best = 0;
    for (auto it = sizeOf.constBegin(); it != sizeOf.constEnd(); ++it)
        if (it.value() > best) { best = it.value(); mainComp = it.key(); }

    r.mainComponent = mainComp;
    for (int i = 0; i < comp.size(); ++i)
        if (comp[i] != mainComp) r.isolatedIndices.push_back(i);
    return r;
}

bool cameraSeenBoard(int cornerCount) {
    return cornerCount > 0;
}

QVector<QPair<int, int>> buildCovisibilityEdges(const QVector<QVector<int>>& roundCamIndices) {
    QVector<QPair<int, int>> edges;
    for (const auto& idxs : roundCamIndices)
        for (int a = 0; a < idxs.size(); ++a)
            for (int b = a + 1; b < idxs.size(); ++b)
                edges.push_back({ idxs[a], idxs[b] });
    return edges;
}

} // namespace mocap
