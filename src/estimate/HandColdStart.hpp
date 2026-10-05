#pragma once
// ---------------------------------------------------------------------------
// 手部冷启动 · 手背模板匹配 —— 纯数学，零 Qt 依赖，可单测。
// 对应 MarkerAssociator.hpp 里 ColdStartFn 明确留白的那部分实现。
//
// 输入是"这一帧三角化出来的一堆无标签 3D 候选点"(比如 reconstruct/
// MultiViewCluster.hpp 的 clusterMultiView 产出的 Track3D 点，混着手背5点、
// 20颗手指球、可能的噪声点，谁是谁完全不知道)。要在这堆点里找出哪几个是
// 手背 marker，并且确定"哪个候选点对应模板里的第几号"，才能喂给
// HandPose.hpp 的 solveHandBackPose 算出 (R,t)。
//
// 做法:手背模板5点两两之间的欧氏距离是已知常量(不随手怎么摆而变，因为
// 手背对手腕是刚体)，候选点里如果真有一组对应手背 marker，它们的两两
// 距离模式必须匹配模板(允许 marker 检测/三角化噪声的容差)。这是经典的
// "已知点间距离、点对应关系未知"的刚体点集匹配问题，用距离约束回溯搜索
// (backtracking)剪枝:模板点按顺序一个个往候选点里指派，每指派一个新的就
// 检查它跟已指派点的距离是否跟模板对应距离匹配，不匹配立刻剪掉这条分支。
//
// 【这一版修的真实bug】之前要求"必须5个点全部匹配上"才算数——但
// solveHandBackPose 本身早就支持最少3点求解(遮挡1~2颗手背marker时用
// 剩下的点照样能解位姿，Kabsch数学上就够)，上层这个"全有或全无"的匹配
// 逻辑完全没利用这个能力：只要一帧里有任意一颗手背marker暂时没被三角化
// 出来(比如被手指挡住一角、某台相机瞬间没检测到)，整帧直接报"未找到"，
// 明明用剩下4个甚至3个点就能正确解出位姿。现在改成优先级退化搜索：先试
// 全部5点，找不到合格匹配就退到"任选4个点的组合"，再不行退到"任选3个点
// 的组合"(3点是 solveHandBackPose 自身要求的下限，不能再退)——只要能用
// 的点里凑得出一组通过距离检验+Kabsch残差复核的子集，就用它，不再因为
// 少一个点就整帧报废。
//
// 可能找到多组通过距离检验的候选指派(比如手指球恰好也有几个凑巧距离模式
// 接近，理论上不是没可能，虽然概率低，点数越少越容易撞)——这种情况下
// 不是随便挑第一个,而是全部试一遍 Kabsch，用配准残差 RMS 最小的那组，
// 且必须通过 solveHandBackPose 自身的 confident 判定(残差阈值)才算数，
// 避免那种恰好距离对上、但配准残差明显更大的伪匹配赢过真正正确的匹配。
// 点数退化的优先级是"能用更多点就不用更少点"——5点能凑齐就不去搜4点的
// 组合(约束更多、更不容易蒙对，本来就该优先信)。
//
// 【重要】模板不再写死调用 HandModel.hpp::handBackMarkers()——那是占位值，
// 之前的版本一直在拿占位值匹配，就算跑了 HandCalibration.hpp 标定出真实
// 模板也没有任何地方会用上它，标定等于空转。现在模板作为参数传入，调用方
// (HandTrackingWorker)从 HandTemplateStore 读取实际标定好的模板传进来。
//
// 【重要】诊断信息不再是"要么给答案、要么给nullopt"——识别不了的时候，
// 到底是候选点不够、还是距离对不上、还是配准残差超标，这些原因现在都会
// 如实报出来(ColdStartDiagnostics)，供 UI(HandPoseDebugDialog / 标定向导)
// 直接展示给使用者，而不是让人对着一个"未找到"干瞪眼猜原因；成功时还会
// 报出这次实际用了几个点(numMatchedPoints)，让人知道是不是在退化状态下
// 工作(比如长期只有4点、说明有一颗marker贴的位置经常被挡，值得关注)。
// ---------------------------------------------------------------------------
#include "hand/HandPose.hpp"
#include "hand/HandModel.hpp"
#include "estimate/HandStateIEKF.hpp"
#include "estimate/HandModelAdapter.hpp"
#include <vector>
#include <array>
#include <optional>
#include <cmath>
#include <algorithm>

namespace mocap {

struct ColdStartConfig {
    double distToleranceMm = 8.0;     // 候选点对距离 vs 模板距离的容差
    double rmsAcceptMm = 5.0;         // Kabsch 配准残差 RMS 接受阈值(跟 solveHandBackPose 的 confident 判据一致)
    int    maxAcceptedMatches = 200;  // 每个"点数子集组合"找到这么多组通过距离检验的候选指派就停止继续搜索该组合(防止候选点很多时组合爆炸)
    int    minMatchedPoints = 3;      // 退化的下限——不能低于 solveHandBackPose 自身要求的3点(3点以下旋转在数学上欠定，见该函数注释)
};

// 冷启动失败原因的分类——UI 应该按这个直接给用户可操作的提示，而不是只说
// "未找到"。顺序大致对应"越靠前越基础、越该先排查"。
enum class ColdStartFailReason {
    None,                  // 没失败(找到了)
    NotEnoughCandidates,    // 候选3D点连最低要求(minMatchedPoints)都不够(多相机三角化本身没凑够点，检查相机是否都能看到手/标定是否正常)
    NoDistanceMatch,        // 候选点够，但从5点一路退化到minMatchedPoints，没有任何一个子集的两两距离能匹配模板(可能：模板本身没标定/标定错了)
    RmsTooHigh,             // 找到了距离匹配的候选组，但Kabsch配准残差都超过阈值(候选点里混进了噪声，或者距离容差设太松导致误匹配)
};

struct ColdStartDiagnostics {
    int numCandidates3D = 0;         // 这一帧三角化出的无标签候选点总数
    int numDistanceMatchesFound = 0; // 通过距离检验、进入Kabsch复核的候选组合数(累加所有尝试过的点数子集)
    int numMatchedPoints = 0;        // 成功时实际用了手背模板里的几个点(5=全部可见，<5=退化状态下工作，供UI提示"长期退化说明有颗marker经常被挡")
    double bestRms = -1.0;           // 所有候选组合里，Kabsch配准残差最小的那个(-1表示一组都没进入这一步)
    ColdStartFailReason failReason = ColdStartFailReason::None;

    // 生成一句人话摘要，直接丢给 UI 显示，不用在 UI 层重新翻译枚举值。
    const char* summary() const {
        switch (failReason) {
            case ColdStartFailReason::None: return "冷启动成功";
            case ColdStartFailReason::NotEnoughCandidates: return "候选点数量低于最低要求——检查是否所有相机都能看到手背marker，或标定是否正常";
            case ColdStartFailReason::NoDistanceMatch: return "候选点够，但从5点退化到最少点数都没有一组距离对得上模板——检查手背模板是否已标定/标定是否正确";
            case ColdStartFailReason::RmsTooHigh: return "有距离匹配的候选组，但配准残差偏大——可能是候选点噪声较大，或距离容差设置不合理导致误匹配";
        }
        return "";
    }
};

namespace coldstart_detail {

inline double dist3(const HandVec3& a, const HandVec3& b) {
    const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

// 在这个搜索状态里，subsetIdx 是"这次尝试用模板里的哪几个下标(0..4)"，
// 长度等于当前退化到的点数(5/4/3)——回溯只在这个子集内部做距离一致性
// 检查，其余模板点这次完全不参与。
struct SearchState {
    const std::array<HandVec3,5>* templ;
    const std::vector<int>* subsetIdx;
    const std::vector<HandVec3>* candidates;
    double tol;
    std::vector<int> assignment;   // 长度=subsetIdx->size()，值是候选点下标
    std::vector<char> used;
    std::vector<std::vector<int>>* results;
    int maxResults;
};

inline void backtrack(SearchState& st, int k) {
    const int subsetSize = int(st.subsetIdx->size());
    if (int(st.results->size()) >= st.maxResults) return;   // 已经够多了，别再搜了
    if (k == subsetSize) { st.results->push_back(st.assignment); return; }

    const auto& templ = *st.templ;
    const auto& subsetIdx = *st.subsetIdx;
    const auto& cands = *st.candidates;
    const int templK = subsetIdx[size_t(k)];

    for (int c=0; c<int(cands.size()); ++c) {
        if (st.used[size_t(c)]) continue;

        bool ok = true;
        for (int j=0; j<k; ++j) {
            const int templJ = subsetIdx[size_t(j)];
            const double dTempl = dist3(templ[size_t(templK)], templ[size_t(templJ)]);
            const double dCand  = dist3(cands[size_t(c)], cands[size_t(st.assignment[size_t(j)])]);
            if (std::abs(dTempl - dCand) > st.tol) { ok = false; break; }
        }
        if (!ok) continue;

        st.assignment[size_t(k)] = c;
        st.used[size_t(c)] = 1;
        backtrack(st, k+1);
        st.used[size_t(c)] = 0;
        if (int(st.results->size()) >= st.maxResults) return;
    }
}

// 从{0,1,2,3,4}里选size个下标的全部组合——手背模板固定5个点，组合数最多
// C(5,3)=10，穷举没有任何性能问题，不需要通用的组合生成算法。
inline std::vector<std::vector<int>> combinationsOf5Choose(int size) {
    std::vector<std::vector<int>> out;
    for (int mask = 0; mask < 32; ++mask) {
        int cnt = 0;
        for (int b=0;b<5;++b) if (mask & (1<<b)) ++cnt;
        if (cnt != size) continue;
        std::vector<int> combo;
        for (int b=0;b<5;++b) if (mask & (1<<b)) combo.push_back(b);
        out.push_back(combo);
    }
    return out;
}

} // namespace coldstart_detail

struct HandBackMatchResult {
    std::optional<HandBackPose> pose;
    ColdStartDiagnostics diag;
    bool found() const { return pose.has_value(); }
};

// 在一堆无标签 3D 候选点里找手背 marker 对应关系并求出 (R,t)。templ 是
// 实际标定出来的手背模板(HandCalibration.hpp::calibrateHandBackTemplate 的
// 输出，经 HandTemplateStore 持久化/读取)——不再写死用 handBackMarkers()。
//
// 退化搜索：优先尝试全部5点，找不到合格匹配就退到"任选4个点的组合"，
// 再退到"任选3个点的组合"——只要某个点数级别找到了合格匹配(通过距离
// 检验+Kabsch残差复核)就立刻返回，不会因为"更多点的组合更靠谱"这个直觉
// 而白白多算；但只有当前点数级别彻底没有任何组合成功时，才会往下退化，
// 保证"能用更多点就不用更少点"这个优先级。
inline HandBackMatchResult matchHandBackTemplate(
        const std::vector<Vec3>& candidates3D,
        const std::array<HandVec3,5>& templ,
        const ColdStartConfig& cfg = ColdStartConfig{}) {
    using namespace coldstart_detail;
    HandBackMatchResult out;
    out.diag.numCandidates3D = int(candidates3D.size());

    const int minPts = std::max(3, cfg.minMatchedPoints);   // 不能低于Kabsch自身要求的3点下限

    if (int(candidates3D.size()) < minPts) {
        out.diag.failReason = ColdStartFailReason::NotEnoughCandidates;
        return out;
    }

    std::vector<HandVec3> cands(candidates3D.size());
    for (size_t i=0;i<candidates3D.size();++i)
        cands[i] = { candidates3D[i][0], candidates3D[i][1], candidates3D[i][2] };

    for (int subsetSize = 5; subsetSize >= minPts; --subsetSize) {
        std::optional<HandBackPose> bestAtThisSize;

        for (const auto& combo : combinationsOf5Choose(subsetSize)) {
            SearchState st;
            st.templ = &templ;
            st.subsetIdx = &combo;
            st.candidates = &cands;
            st.tol = cfg.distToleranceMm;
            st.assignment.assign(size_t(subsetSize), 0);
            st.used.assign(cands.size(), 0);
            std::vector<std::vector<int>> results;
            st.results = &results;
            st.maxResults = cfg.maxAcceptedMatches;
            backtrack(st, 0);

            out.diag.numDistanceMatchesFound += int(results.size());

            for (const auto& assign : results) {
                std::array<HandVec3,5> observed{};
                std::array<bool,5> mask{false,false,false,false,false};
                for (int i=0;i<subsetSize;++i) {
                    const int templIdx = combo[size_t(i)];
                    observed[size_t(templIdx)] = cands[size_t(assign[size_t(i)])];
                    mask[size_t(templIdx)] = true;
                }
                const auto pose = solveHandBackPose(templ, observed, mask, minPts);
                if (!pose.confident) continue;
                if (!bestAtThisSize || pose.rms < bestAtThisSize->rms) bestAtThisSize = pose;
            }
        }

        if (bestAtThisSize) {
            if (out.diag.bestRms < 0.0 || bestAtThisSize->rms < out.diag.bestRms) out.diag.bestRms = bestAtThisSize->rms;
            if (bestAtThisSize->rms <= cfg.rmsAcceptMm) {
                out.pose = bestAtThisSize;
                out.diag.numMatchedPoints = subsetSize;
                out.diag.failReason = ColdStartFailReason::None;
                return out;   // 这个点数级别已经成功，不再退化到更少点
            }
        }
        // 这个点数级别没有合格匹配(要么距离检验就没过、要么Kabsch残差超标)，
        // 退到更少一个点的组合继续试。
    }

    if (out.diag.numDistanceMatchesFound == 0) {
        out.diag.failReason = ColdStartFailReason::NoDistanceMatch;
    } else {
        out.diag.failReason = ColdStartFailReason::RmsTooHigh;
    }
    return out;
}

struct ColdStartResult {
    std::optional<HandPoseState> state;
    ColdStartDiagnostics diag;
};

// 便捷封装：直接产出 HandStateIEKF 要的 HandPoseState(关节角给中性初值，
// 冷启动只能确定手腕刚体位姿，手指姿态留给后续帧的观测去收敛)，同时带上
// 诊断信息供 UI 展示失败原因。
inline ColdStartResult coldStartHandPose(
        const std::vector<Vec3>& candidates3D,
        const std::array<HandVec3,5>& templ,
        int numJoints = kHandNumJoints,
        const ColdStartConfig& cfg = ColdStartConfig{}) {
    ColdStartResult out;
    const auto matchResult = matchHandBackTemplate(candidates3D, templ, cfg);
    out.diag = matchResult.diag;
    if (!matchResult.pose) return out;

    HandPoseState state;
    state.wristPos = { matchResult.pose->t[0], matchResult.pose->t[1], matchResult.pose->t[2] };
    state.wristRot = matchResult.pose->R;   // HandMat3 和 std::array<double,9> 是同一底层类型，直接赋值
    state.jointAngles.assign(size_t(numJoints), 0.0);
    out.state = state;
    return out;
}

} // namespace mocap
