#pragma once
// ---------------------------------------------------------------------------
// 全手自动标定 pipeline —— 纯数学，零 Qt 依赖。
//
// 把前面几个独立模块串成一条完整的"从采集数据到标定结果"的链路：
//
//   1. HandRigidAutoCalib.hpp  —— 从晃手采集数据里自动找出手背刚体+局部模板
//   2. (本文件) 逐帧用手背模板 Kabsch 解算腕部位姿，把手指候选点从世界系
//      转换到腕部局部系(此时仍是阶段1那个任意朝向的局部系，还没轴系对齐)
//   3. FingerChainSegmentation.hpp —— 把腕部局部系下的手指候选点分成5组
//   4. HandAxisAlignment.hpp —— 用手背模板+5组的粗略锚点方向，算出把任意
//      朝向对齐到 hand/HandModel.hpp 约定(+X指手指方向、+Y指拇指侧)的旋转，
//      同时识别出哪一组是拇指
//   5. 把所有观测应用这个对齐旋转，重新表达到最终坐标系
//   6. FingerKinematicCalib.hpp —— 在对齐后的坐标系下，对每根链跑运动学拟合
//
// 【关键设计点，容易踩坑的地方】：第4步算出对齐旋转后，绝不能只对"已经在
// 旧坐标系拟合好的锚点"做一次矩�么变换就完事——因为 FingerKinematicCalib.hpp
// 里 fingerChainFK 的屈曲/外展是直接绕"当前坐标系"的Y/Z轴转的，不是绕每根
// 手指自己的局部轴(拇指例外，靠 baseRotZ 建立自己的局部系)。换句话说，关节
// 角的物理意义跟坐标系本身绑定，坐标系一换，必须重新拟合，不能简单地把
// 旧拟合结果的锚点旋转一下就当作新坐标系下的答案——本文件的实现顺序
// (先分组，用近节marker位置估算粗略锚点方向来定轴系，最后才对每条链跑
// 真正的运动学拟合)就是为了避免这个错误：真正的 calibrateFingerChain /
// calibrateThumbChain 调用，用的都是已经对齐过的观测数据。
//
// 前提假设(调用方需满足)：framesRigid 和 framesFingers 是同一次连续跟踪
// 会话产出的数据(或至少手背5点在两段数据里的 track id 编号一致)——阶段2
// 需要拿阶段1找到的手背刚体 id，去 framesFingers 里认出同一批点。如果是
// 两次完全独立的采集(track id 编号可能不一样)，本文件不负责重新对应，
// 调用方需要自己保证 id 一致或先做一次映射。
// ---------------------------------------------------------------------------
#include "estimate/HandRigidAutoCalib.hpp"
#include "estimate/FingerChainSegmentation.hpp"
#include "estimate/FingerKinematicCalib.hpp"
#include "estimate/HandAxisAlignment.hpp"
#include "hand/HandPose.hpp"
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <string>
#include <algorithm>

namespace mocap {

struct HandAutoCalibConfig {
    RigidAutoCalibConfig rigidCfg{};

    bool useTier1TimeWindows = true;      // true: segmentByTimeWindows；false: segmentByMotionCorrelation
    std::vector<TimeWindow> fingerWindows{};   // Tier1时必须给恰好5个窗口
    TimeWindowSegConfig timeWindowCfg{};
    MotionCorrSegConfig corrCfg{};

    FingerCalibConfig fingerCfg{};
    ThumbCalibConfig thumbCfg{};

    int minWristVisibleForPose = 3;   // 每帧至少要有几颗手背刚体点可见，才能解算这一帧的腕部位姿
    int minFramesWithPose = 20;
};

struct HandAutoCalibOutput {
    bool valid = false;
    std::string message;

    std::array<Vec3, 5> backMarkers{};        // 手背5点，已轴系对齐
    std::vector<int> backMarkerTrackIds;      // 对应的原始track id(供调用方核对)

    // 下标0..3 = 食/中/无名/小，按对齐后锚点的+Y分量从大到小排序得出
    // (桡侧->尺侧，即解剖学上食指最靠拇指、小指最远)——这是唯一能从
    // 几何上稳定推断的顺序，不依赖任何写死的编号假设。
    std::array<FingerChainParams, 4> fingerParams{};
    std::array<double, 4> fingerRmsMm{};
    ThumbChainParams thumbParams;
    double thumbRmsMm = 0.0;

    double rigidMaxPairStdMm = 0.0;
    double axisAlignThumbScore = 0.0;
};

namespace pipeline_detail {

inline std::vector<Vec3> extractGroupObsField(const std::vector<FrameTrackedPoints>& frames,
                                              const std::vector<int>& ids3, int which) {
    std::vector<Vec3> out;
    for (auto& fr : frames)
        for (size_t j = 0; j < fr.ids.size(); ++j)
            if (fr.ids[j] == ids3[size_t(which)]) { out.push_back(fr.positions[j]); break; }
    return out;
}

inline std::vector<std::array<Vec3, 3>> extractGroupObs(const std::vector<FrameTrackedPoints>& frames,
                                                         const std::vector<int>& ids3) {
    std::vector<std::array<Vec3, 3>> obs;
    for (auto& fr : frames) {
        std::array<std::optional<Vec3>, 3> pts;
        for (size_t j = 0; j < fr.ids.size(); ++j) {
            for (int k = 0; k < 3; ++k) if (fr.ids[j] == ids3[size_t(k)]) pts[size_t(k)] = fr.positions[j];
        }
        if (pts[0] && pts[1] && pts[2]) obs.push_back({*pts[0], *pts[1], *pts[2]});
    }
    return obs;
}

} // namespace pipeline_detail

inline HandAutoCalibOutput runHandAutoCalibPipeline(
        const std::vector<FrameTrackedPoints>& framesRigid,
        const std::vector<FrameTrackedPoints>& framesFingers,
        const HandAutoCalibConfig& cfg = HandAutoCalibConfig{}) {
    using namespace pipeline_detail;
    HandAutoCalibOutput out;

    // ---- 阶段1：手背刚体自动标定 ----
    const auto rigidResult = calibrateRigidClusters(framesRigid, cfg.rigidCfg);
    if (!rigidResult.valid) {
        out.message = "[阶段1-手背刚体标定] " + rigidResult.message;
        return out;
    }
    const auto& wristCluster = rigidResult.clusters[0];
    if (int(wristCluster.ids.size()) < 3) {
        out.message = "[阶段1-手背刚体标定] 找到的刚体点数不足3个，无法解算腕部位姿。";
        return out;
    }
    out.rigidMaxPairStdMm = wristCluster.maxPairStdMm;

    // ---- 阶段2：逐帧解腕部位姿，把手指候选点转换到腕部局部系(仍是阶段1
    // 那个任意朝向局部系，还没轴系对齐)。----
    std::unordered_set<int> wristIdSet(wristCluster.ids.begin(), wristCluster.ids.end());
    std::vector<FrameTrackedPoints> fingerFramesLocal;
    fingerFramesLocal.reserve(framesFingers.size());
    int framesWithPose = 0;

    for (const auto& fr : framesFingers) {
        std::vector<HandVec3> P, Q;
        for (size_t k = 0; k < wristCluster.ids.size(); ++k) {
            const int gid = wristCluster.ids[k];
            for (size_t j = 0; j < fr.ids.size(); ++j) {
                if (fr.ids[j] == gid) {
                    const auto& lp = wristCluster.localTemplate[k];
                    P.push_back({lp[0], lp[1], lp[2]});
                    Q.push_back({fr.positions[j][0], fr.positions[j][1], fr.positions[j][2]});
                    break;
                }
            }
        }
        if (int(P.size()) < cfg.minWristVisibleForPose) continue;

        HandMat3 R; HandVec3 t;
        kabsch(P, Q, R, t);   // world ≈ R*local + t
        ++framesWithPose;

        FrameTrackedPoints localFr;
        for (size_t j = 0; j < fr.ids.size(); ++j) {
            if (wristIdSet.count(fr.ids[j])) continue;
            const auto& w = fr.positions[j];
            const HandVec3 rel = { w[0]-t[0], w[1]-t[1], w[2]-t[2] };
            // local = R^T * rel（R是行主序3x3正交阵，转置即逆）
            const Vec3 local = {
                R[0]*rel[0] + R[3]*rel[1] + R[6]*rel[2],
                R[1]*rel[0] + R[4]*rel[1] + R[7]*rel[2],
                R[2]*rel[0] + R[5]*rel[1] + R[8]*rel[2],
            };
            localFr.ids.push_back(fr.ids[j]);
            localFr.positions.push_back(local);
        }
        fingerFramesLocal.push_back(localFr);
    }

    if (framesWithPose < cfg.minFramesWithPose) {
        out.message = "[阶段2-腕部位姿] 只有 " + std::to_string(framesWithPose) +
                     " 帧能解出腕部位姿(手背可见点不足 " + std::to_string(cfg.minWristVisibleForPose) +
                     " 颗)，采集数据不够，无法继续。";
        return out;
    }

    // ---- 阶段3：手指分组 ----
    FingerSegmentationResult segResult;
    if (cfg.useTier1TimeWindows) {
        if (cfg.fingerWindows.size() != 5) {
            out.message = "[阶段3-手指分组] Tier1(引导式)需要恰好5个时间窗口，实际给了 " +
                         std::to_string(cfg.fingerWindows.size()) + " 个。";
            return out;
        }
        segResult = segmentByTimeWindows(fingerFramesLocal, cfg.fingerWindows, cfg.timeWindowCfg);
    } else {
        segResult = segmentByMotionCorrelation(fingerFramesLocal, cfg.corrCfg);
    }
    if (!segResult.valid || int(segResult.groups.size()) != 5) {
        out.message = "[阶段3-手指分组] " + segResult.message +
                     (segResult.valid ? (" (只分出 " + std::to_string(segResult.groups.size()) + " 组，需要5组)") : "");
        return out;
    }

    // ---- 阶段4：粗略锚点估计(近节marker在整段采集里的位置中位数)，供
    // 轴系对齐判别拇指方向用——不需要精确值，只需要大致方向。用中位数
    // 而不是均值：如果这根手指自己有一段时间在活跃摆动(比如Tier1引导式
    // 采集里它自己的窗口)，活跃期间的平均角度可能跟静止期间不一样，均值
    // 会被这个不均衡的时间占比拉偏出几mm的系统性偏差；中位数对这种"一段
    // 时间分布明显不同"的情况更鲁棒。----
    std::vector<Vec3> roughAnchors(5);
    for (int g = 0; g < 5; ++g) {
        auto nearTraj = extractGroupObsField(fingerFramesLocal, segResult.groups[size_t(g)].orderedIds, 0);
        if (nearTraj.empty()) { roughAnchors[size_t(g)] = Vec3{0,0,0}; continue; }
        std::array<std::vector<double>,3> axisVals;
        for (auto& p : nearTraj) for (int k=0;k<3;++k) axisVals[size_t(k)].push_back(p[size_t(k)]);
        Vec3 med{0,0,0};
        for (int k=0;k<3;++k) {
            auto& v = axisVals[size_t(k)];
            std::nth_element(v.begin(), v.begin()+long(v.size()/2), v.end());
            med[size_t(k)] = v[v.size()/2];
        }
        roughAnchors[size_t(g)] = med;
    }

    // ---- 阶段5：轴系对齐 ----
    std::vector<Vec3> wristTemplateVec3(5);
    for (int i = 0; i < 5; ++i)
        wristTemplateVec3[size_t(i)] = { wristCluster.localTemplate[size_t(i)][0],
                                         wristCluster.localTemplate[size_t(i)][1],
                                         wristCluster.localTemplate[size_t(i)][2] };
    const auto align = computeAxisAlignment(wristTemplateVec3, roughAnchors);
    if (!align.valid) {
        out.message = "[阶段5-轴系对齐] " + align.message;
        return out;
    }
    out.axisAlignThumbScore = align.thumbOutlierScore;

    // ---- 阶段6：应用对齐，重新表达所有观测到最终坐标系。----
    for (auto& fr : fingerFramesLocal) for (auto& p : fr.positions) p = applyAxisAlignment(align, p);

    for (int i = 0; i < 5; ++i) out.backMarkers[size_t(i)] = applyAxisAlignment(align, wristTemplateVec3[size_t(i)]);
    out.backMarkerTrackIds = wristCluster.ids;

    std::vector<Vec3> alignedRoughAnchors(5);
    for (int g = 0; g < 5; ++g) alignedRoughAnchors[size_t(g)] = applyAxisAlignment(align, roughAnchors[size_t(g)]);

    // ---- 阶段7：对每根普通手指链跑运动学拟合(用对齐后的观测)。----
    const int thumbGroupIdx = align.thumbAnchorIndex;
    std::vector<int> regularGroupIdx;
    for (int g = 0; g < 5; ++g) if (g != thumbGroupIdx) regularGroupIdx.push_back(g);
    std::sort(regularGroupIdx.begin(), regularGroupIdx.end(), [&](int a, int b) {
        return alignedRoughAnchors[size_t(a)][1] > alignedRoughAnchors[size_t(b)][1];
    });

    for (int outIdx = 0; outIdx < 4; ++outIdx) {
        const int g = regularGroupIdx[size_t(outIdx)];
        const auto obs = extractGroupObs(fingerFramesLocal, segResult.groups[size_t(g)].orderedIds);
        const auto fit = calibrateFingerChain(obs, cfg.fingerCfg);
        if (!fit.valid) {
            out.message = "[阶段7-普通手指(下标" + std::to_string(outIdx) + ")运动学拟合] " + fit.message;
            return out;
        }
        out.fingerParams[size_t(outIdx)] = fit.params;
        out.fingerRmsMm[size_t(outIdx)] = fit.rmsMm;
    }

    // ---- 阶段8：拇指链拟合。----
    {
        const auto obs = extractGroupObs(fingerFramesLocal, segResult.groups[size_t(thumbGroupIdx)].orderedIds);
        const auto fit = calibrateThumbChain(obs, cfg.thumbCfg);
        if (!fit.valid) {
            out.message = "[阶段8-拇指运动学拟合] " + fit.message;
            return out;
        }
        out.thumbParams = fit.params;
        out.thumbRmsMm = fit.rmsMm;
    }

    out.valid = true;
    out.message = "全手自动标定成功。";
    return out;
}

} // namespace mocap
