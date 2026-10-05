// ---------------------------------------------------------------------------
// RoundConnectivity 纯逻辑单元测试——不依赖 Qt GUI/相机/文件系统，脱离硬件
// 单独编译运行（跟 test_centroid.cpp/test_triangulation.cpp 同一路子）。
//
// 覆盖范围特意包含"正常操作不会主动凑到"的边界情况：0台/1台相机、只靠
// 接力连起来的链式连通（而不是要求两两都拍过）、多个互不连通的孤岛、
// 重复/反向/自环边、主分量出现并列的极端情况。这些是本轮对话里明确讨论过
// 的"手动测试测不出来、得靠脚本专门戳"的那一类问题。
//
// 用法：ctest 会把本文件编译成独立可执行文件运行；退出码非0 = 有断言失败。
// ---------------------------------------------------------------------------
#include "calib/RoundConnectivity.hpp"
#include <cstdio>
#include <cstdlib>

using namespace mocap;

namespace {

int g_failures = 0;
int g_checks = 0;

void reportFail(const char* file, int line, const char* expr) {
    ++g_failures;
    std::fprintf(stderr, "[FAIL] %s:%d  CHECK(%s)\n", file, line, expr);
}

} // namespace

// 不用宏藏起 __FILE__/__LINE__ 的话失败时定位不到具体是哪一行断言炸的，
// 所以还是用宏，但保持这一个测试文件里最简单的一种形式，没有额外框架依赖。
#define CHECK(expr) do { \
        ++g_checks; \
        if (!(expr)) reportFail(__FILE__, __LINE__, #expr); \
    } while (0)

// =====================================================================
// unionFindComponents
// =====================================================================

void test_unionFind_empty() {
    const auto comp = unionFindComponents(0, {});
    CHECK(comp.isEmpty());
}

void test_unionFind_singleNode_noEdges() {
    const auto comp = unionFindComponents(1, {});
    CHECK(comp.size() == 1);
}

void test_unionFind_twoNodes_noEdges_areSeparate() {
    const auto comp = unionFindComponents(2, {});
    CHECK(comp.size() == 2);
    CHECK(comp[0] != comp[1]);   // 没有任何边，两台各自孤立
}

void test_unionFind_twoNodes_oneEdge_areConnected() {
    const auto comp = unionFindComponents(2, { {0, 1} });
    CHECK(comp[0] == comp[1]);
}

// 这是标定板接力真正依赖的场景：不要求所有相机两两都拍过（那是全连接），
// 只要求"接力链"——0-1、1-2、2-3 挨个连上，0 和 3 之间即便没有直接拍过，
// 也应该判定为已连通。这正是接力模式相对挥球模式的核心卖点，必须测到。
void test_unionFind_relayChain_allConnectedThroughIntermediate() {
    const int n = 4;
    const QVector<QPair<int, int>> edges = { {0, 1}, {1, 2}, {2, 3} };
    const auto comp = unionFindComponents(n, edges);
    CHECK(comp[0] == comp[1]);
    CHECK(comp[1] == comp[2]);
    CHECK(comp[2] == comp[3]);
    CHECK(comp[0] == comp[3]);   // 0 和 3 从没直接共视过，但接力应判定连通
    CHECK(allConnected(comp));
}

// 这正是本轮对话最初发现的那个 bug 的数学层面版本："拍了无论什么都显示
// 连通"修好之后，还得保证"真的有相机完全没连上"时，算法正确报告"没连通"。
void test_unionFind_islandCamera_notConnected() {
    const int n = 4;
    // 0-1-2 连力成一串，3 号相机从来没有出现在任何一轮里——现实中对应
    // "某台相机被遗漏/一直被挡住，一次都没跟别人拍到共视"。
    const QVector<QPair<int, int>> edges = { {0, 1}, {1, 2} };
    const auto comp = unionFindComponents(n, edges);
    CHECK(comp[0] == comp[1]);
    CHECK(comp[1] == comp[2]);
    CHECK(comp[3] != comp[0]);       // 3 号确实是孤立的
    CHECK(!allConnected(comp));

    const auto mc = findMainComponent(comp);
    CHECK(mc.isolatedIndices.size() == 1);
    CHECK(mc.isolatedIndices[0] == 3);
}

void test_unionFind_twoSeparateIslands() {
    const int n = 4;
    const QVector<QPair<int, int>> edges = { {0, 1}, {2, 3} };   // 两对，互不相干
    const auto comp = unionFindComponents(n, edges);
    CHECK(comp[0] == comp[1]);
    CHECK(comp[2] == comp[3]);
    CHECK(comp[0] != comp[2]);
    CHECK(!allConnected(comp));
}

// 重复边、反向边、自环边都是"用户操作产生的边表里天然可能出现的噪音"
// （比如同一对相机拍了好几轮、或者边表构造时不小心把 (a,b)/(b,a) 都塞了
// 进去），并查集必须对这些不敏感，不能因为重复/自环就出错或者结果不同。
void test_unionFind_duplicateAndReversedEdges_areHarmless() {
    const auto comp1 = unionFindComponents(3, { {0, 1}, {0, 1}, {0, 1} });
    CHECK(comp1[0] == comp1[1]);
    CHECK(comp1[2] != comp1[0]);   // 2 号没参与任何边，仍然孤立

    const auto comp2 = unionFindComponents(2, { {0, 1} });
    const auto comp3 = unionFindComponents(2, { {1, 0} });   // 反向给
    CHECK((comp2[0] == comp2[1]) == (comp3[0] == comp3[1]));   // 结果等价
}

void test_unionFind_selfLoopEdge_doesNotCrashOrMergeWrongly() {
    const auto comp = unionFindComponents(2, { {0, 0} });   // 自环边（不该出现，但要防御）
    CHECK(comp.size() == 2);
    CHECK(comp[0] != comp[1]);   // 自环不应该把 0 和 1 错误地连起来
}

// =====================================================================
// allConnected
// =====================================================================

void test_allConnected_lessThanTwoNodes_isFalse() {
    CHECK(allConnected({}) == false);        // 0 台
    CHECK(allConnected({ 0 }) == false);     // 1 台——"谈不上已连通"，跟 roundsStepReady 原有语义一致
}

void test_allConnected_allSame_isTrue() {
    CHECK(allConnected({ 5, 5, 5, 5, 5 }) == true);   // 数值本身没意义，只要求全相等
}

void test_allConnected_oneOddOneOut_isFalse() {
    CHECK(allConnected({ 5, 5, 5, 9 }) == false);
}

// =====================================================================
// findMainComponent
// =====================================================================

void test_findMainComponent_empty() {
    const auto mc = findMainComponent({});
    CHECK(mc.mainComponent == -1);
    CHECK(mc.isolatedIndices.isEmpty());
}

void test_findMainComponent_singleNode() {
    const auto mc = findMainComponent({ 7 });
    CHECK(mc.mainComponent == 7);
    CHECK(mc.isolatedIndices.isEmpty());
}

void test_findMainComponent_allSame_noIsolated() {
    const auto mc = findMainComponent({ 3, 3, 3 });
    CHECK(mc.mainComponent == 3);
    CHECK(mc.isolatedIndices.isEmpty());
}

void test_findMainComponent_clearMajority() {
    // comp = [0,0,0,1] —— 0 号分量有 3 个节点，明显是主分量，3 号节点孤立。
    const auto mc = findMainComponent({ 0, 0, 0, 1 });
    CHECK(mc.mainComponent == 0);
    CHECK(mc.isolatedIndices.size() == 1);
    CHECK(mc.isolatedIndices[0] == 3);
}

// 并列的极端情况：两个分量大小相等（各2个节点）。具体选谁当"主分量"由
// QHash 的遍历顺序决定，Qt 公开 API 没有承诺这个顺序，所以这里不断言
// "选的是哪一个"，只断言结果内部自洽——mainComponent 必须是两个分量之一，
// isolatedIndices 必须精确等于"不属于 mainComponent 的下标"，两者合起来
// 覆盖全部节点、不重不漏。断言内部一致性比断言具体胜出者更稳，也更贴近
// "调用方到底会不会因为这个不确定性而出 bug"这个真正关心的问题——
// 只要提示文案能自洽地说出"谁跟谁一组、该往哪挪"，胜负是哪个不重要。
void test_findMainComponent_tie_isInternallyConsistent() {
    const QVector<int> comp = { 0, 0, 1, 1 };
    const auto mc = findMainComponent(comp);
    CHECK(mc.mainComponent == 0 || mc.mainComponent == 1);
    for (int i = 0; i < comp.size(); ++i) {
        const bool shouldBeIsolated = (comp[i] != mc.mainComponent);
        const bool isListedIsolated = mc.isolatedIndices.contains(i);
        CHECK(shouldBeIsolated == isListedIsolated);
    }
    CHECK(mc.isolatedIndices.size() == 2);   // 不管选谁当主分量，另一组两个都算孤立
}

// =====================================================================
// cameraSeenBoard
// =====================================================================

void test_cameraSeenBoard_zeroIsNotSeen() {
    CHECK(cameraSeenBoard(0) == false);
}

void test_cameraSeenBoard_negativeIsNotSeen() {
    // 正常情况下角点数不会是负数，但防御性地测一下不会被 >0 的判据坑到。
    CHECK(cameraSeenBoard(-1) == false);
    CHECK(cameraSeenBoard(-100) == false);
}

void test_cameraSeenBoard_positiveIsSeen() {
    // 这里只测">0"这一层 C++ 判据本身；"角点数至少要 6 个才算有效检测"
    // 是 Python 侧 detect_board_corners/matchImagePoints 的职责（角点不够
    // 会返回 None，对应这里传进来的就是 0），两层职责分开测，见
    // test_detect_only.py。
    CHECK(cameraSeenBoard(1) == true);
    CHECK(cameraSeenBoard(6) == true);
    CHECK(cameraSeenBoard(200) == true);
}

// =====================================================================
// buildCovisibilityEdges
// =====================================================================

void test_buildEdges_empty() {
    const auto edges = buildCovisibilityEdges({});
    CHECK(edges.isEmpty());
}

void test_buildEdges_singleCameraRound_noEdges() {
    // 一轮只有一台相机拍到：单台没法跟自己产生共视关系，不该生成任何边
    // （这也是 onCaptureRound 里"至少两台才算一轮"门槛在数学层面的对应）。
    const auto edges = buildCovisibilityEdges({ { 0 } });
    CHECK(edges.isEmpty());
}

void test_buildEdges_emptyRound_noCrash() {
    // 防御性场景：某一轮的相机下标列表是空的（理论上不该发生，但边表构造
    // 不应该因此崩溃或产生野边）。
    const auto edges = buildCovisibilityEdges({ {}, { 0, 1 } });
    CHECK(edges.size() == 1);
    CHECK(edges[0].first == 0 && edges[0].second == 1);
}

void test_buildEdges_twoCameraRound_oneEdge() {
    const auto edges = buildCovisibilityEdges({ { 0, 1 } });
    CHECK(edges.size() == 1);
    CHECK(edges[0].first == 0 && edges[0].second == 1);
}

void test_buildEdges_threeCameraRound_allPairs() {
    // 三台同时看到板子的一轮，应该两两都连边：(0,1) (0,2) (1,2)。
    const auto edges = buildCovisibilityEdges({ { 0, 1, 2 } });
    CHECK(edges.size() == 3);
    CHECK(edges.contains({0, 1}));
    CHECK(edges.contains({0, 2}));
    CHECK(edges.contains({1, 2}));
}

void test_buildEdges_multipleRounds_accumulate() {
    const auto edges = buildCovisibilityEdges({ { 0, 1 }, { 1, 2 } });
    CHECK(edges.size() == 2);
    CHECK(edges.contains({0, 1}));
    CHECK(edges.contains({1, 2}));
}

// =====================================================================
// 端到端小集成：从"每轮参与的相机下标"一路走到"是否已连通/谁孤立"，
// 完整复刻 refreshRoundsUI()/roundsStepReady() 里的真实调用链，而不是
// 只测单个函数——这样能测到函数之间"接得对不对"，不只是各自对不对。
// =====================================================================

void test_endToEnd_relayThenSolveExtrinsics_readyToProceed() {
    const int n = 3;   // 3 台相机
    // round0: 相机0、1 共视；round1: 相机1、2 共视——典型接力场景。
    const QVector<QVector<int>> roundIdxs = { { 0, 1 }, { 1, 2 } };
    const auto edges = buildCovisibilityEdges(roundIdxs);
    const auto comp = unionFindComponents(n, edges);
    CHECK(allConnected(comp));
    const auto mc = findMainComponent(comp);
    CHECK(mc.isolatedIndices.isEmpty());
}

void test_endToEnd_missingRelayLink_notReady() {
    const int n = 3;
    // 只拍了 round0（相机0、1），相机2 从来没跟任何人共视过。
    const QVector<QVector<int>> roundIdxs = { { 0, 1 } };
    const auto edges = buildCovisibilityEdges(roundIdxs);
    const auto comp = unionFindComponents(n, edges);
    CHECK(!allConnected(comp));
    const auto mc = findMainComponent(comp);
    CHECK(mc.isolatedIndices.size() == 1);
    CHECK(mc.isolatedIndices[0] == 2);   // 明确指出该往哪挪——2号需要接上
}

int main() {
    test_unionFind_empty();
    test_unionFind_singleNode_noEdges();
    test_unionFind_twoNodes_noEdges_areSeparate();
    test_unionFind_twoNodes_oneEdge_areConnected();
    test_unionFind_relayChain_allConnectedThroughIntermediate();
    test_unionFind_islandCamera_notConnected();
    test_unionFind_twoSeparateIslands();
    test_unionFind_duplicateAndReversedEdges_areHarmless();
    test_unionFind_selfLoopEdge_doesNotCrashOrMergeWrongly();

    test_allConnected_lessThanTwoNodes_isFalse();
    test_allConnected_allSame_isTrue();
    test_allConnected_oneOddOneOut_isFalse();

    test_findMainComponent_empty();
    test_findMainComponent_singleNode();
    test_findMainComponent_allSame_noIsolated();
    test_findMainComponent_clearMajority();
    test_findMainComponent_tie_isInternallyConsistent();

    test_cameraSeenBoard_zeroIsNotSeen();
    test_cameraSeenBoard_negativeIsNotSeen();
    test_cameraSeenBoard_positiveIsSeen();

    test_buildEdges_empty();
    test_buildEdges_singleCameraRound_noEdges();
    test_buildEdges_emptyRound_noCrash();
    test_buildEdges_twoCameraRound_oneEdge();
    test_buildEdges_threeCameraRound_allPairs();
    test_buildEdges_multipleRounds_accumulate();

    test_endToEnd_relayThenSolveExtrinsics_readyToProceed();
    test_endToEnd_missingRelayLink_notReady();

    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
