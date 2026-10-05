#pragma once
// ---------------------------------------------------------------------------
// 标定板接力（第③步）用到的纯逻辑：谁跟谁连通、主分量是谁、这一帧算不算
// 看到板子。原来这些逻辑分散在 CalibWizard.cpp 的匿名命名空间和
// refreshRoundsUI()/roundsStepReady() 内联代码里——只依赖 Qt::Core 容器
// （QVector/QPair），不摸 Qt GUI、不摸相机、不摸文件系统，所以能完全脱离
// GUI 环境单独编译测试，取舍跟 Triangulation.cpp 一样（那边注释里解释过：
// 数学核心本身不调用 Qt 运行时功能，但引用到的类型需要 Qt::Core 的 include
// 路径）。CalibWizard.cpp 直接调用这里的函数，行为跟抽出来之前完全一致。
// ---------------------------------------------------------------------------
#include <QVector>
#include <QPair>

namespace mocap {

// 一轮标定板接力至少要几台相机看到才算数——单台相机没法跟别的相机产生
// 共视关系。抽到这里作为唯一定义，避免以后改门槛只改了 CalibWizard.cpp
// 里的一处、测试里用的是另一个硬编码数字，两边不同步。
constexpr int kMinCamsPerRound = 2;

// 并查集：给定 n 个节点（相机下标 0..n-1）和一批"这两台相机在某一轮里
// 共视过"的边，返回每个节点所属分量的代表元——同一分量的节点 comp[i] 数值
// 相等，具体等于哪个数字没有意义，只用来判断"是不是同一组"。
QVector<int> unionFindComponents(int n, const QVector<QPair<int, int>>& edges);

// 是否全部节点都在同一分量里。n<2 视为"无法判断已连通"，返回 false——
// 跟 roundsStepReady() 原有语义一致：少于两台相机谈不上外参标定。
bool allConnected(const QVector<int>& comp);

// 找样本最多（节点数最多）的分量当"主分量"（不是随便拿 comp[0]，避免
// "主分量"恰好只是某个孤立相机自己这种误判），返回主分量代表元 + 所有
// 不在主分量里的节点下标（= 尚未连通、需要提示用户"往哪挪"的那些相机）。
struct MainComponentResult {
    int mainComponent = -1;
    QVector<int> isolatedIndices;
};
MainComponentResult findMainComponent(const QVector<int>& comp);

// 这一帧算不算"看到标定板"：检出的 ChArUco 角点数 > 0。单独抽出来纯粹是
// 为了让判据在生产代码和测试里保持唯一定义，以后改门槛（比如要求角点数
// 不少于某个更严格的数字）只用改这一处。
bool cameraSeenBoard(int cornerCount);

// 给一批"每一轮参与的相机下标列表"，生成对应的共视边表：每一轮内，参与的
// 相机两两连一条边（下标基于调用方自己的 0..n-1 编号，不关心 camId）。
// 原来 CalibWizard.cpp 的 refreshRoundsUI() 和 roundsStepReady() 各自独立
// 写了一遍一模一样的双重循环来推导边表——抽出来避免"改一处的边表生成逻辑，
// 忘了同步改另一处"，这类重复正是这几轮排查出的其它 bug 的同一种成因。
QVector<QPair<int, int>> buildCovisibilityEdges(const QVector<QVector<int>>& roundCamIndices);

} // namespace mocap
