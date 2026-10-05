#pragma once
// ---------------------------------------------------------------------------
// TrackFrameDebug.hpp —— 3D 点追踪器的逐帧内部状态（两个后端共用）
//
// 【为什么单独一个头文件】TemporalTracker 和 IekfPointTracker 是可以互换的
// 两套实现，谁也不该 include 谁。结构放在这里，两边各自 include，互不依赖。
//
// 【为什么需要这一层】录制格式里的 Point3DRec 有 predicted / usedViews /
// residualMm 三个字段，【定义了但从来没有被填过】——一直是 0。而真正决定
// 编号稳不稳的量（速度、协方差、关联距离、马氏距离、命中数、是不是刚新建）
// 一个都不在里面。
//
// "编号乱跳"是这套系统最常见的抱怨，而靠 Point3DRec 是查不出来的：它只看得到
// 编号变了，看不到原因是
//     · 门控没兜住（关联距离超了 -> 看 assocDistMm vs maxAssocDistMm）
//     · 协方差塌了（点被认为很确定，于是马氏门把正确观测拒之门外
//       -> 看 posVarTrace 和 assocMahaSq vs chiSquareGate）
//     · 关联被邻近的点抢走了（-> 看 obsIndex 有没有在两条轨迹间跳）
//     · 真点在 tentative 阶段反复夭折（-> 看 hits 和 justAcquired）
// 这四种的修法完全不同：分别是放宽 maxAssocDist、抬高 obsVarFloor、
// 开全局最优指派、降低 minHitsToConfirm。而只看编号的话，四者一模一样。
//
// 【用不上的字段留 -1，不是 0】TemporalTracker 没有协方差和马氏距离。
// 填 0 的话会被当成"测到了，值就是 0"——一个位置协方差为 0 的点意味着
// "绝对确定"，那是最危险的误读。-1 是不可能出现的值，一眼看得出是"没有"。
// ---------------------------------------------------------------------------

#include <vector>

namespace mocap {

struct TrackFrameDebugItem {
    int    id = -1;
    double pos[3]{0, 0, 0};
    double vel[3]{0, 0, 0};      // 单位/帧（不是每秒——帧率变了要自己换算）
    double posVarTrace = -1.0;   // 位置协方差的迹：这个点有多"虚"
    double velVarTrace = -1.0;
    double assocDistMm = -1.0;   // 本帧关联上的观测离预测多远（欧氏）
    double assocMahaSq = -1.0;   // 马氏距离平方，跟 chiSquareGate 直接比
    double qBoost = 1.0;         // 机动自适应过程噪声倍率
    double residualMm = -1.0;    // 关联到的那个 cluster 的三角化残差
    int    obsIndex = -1;        // 关联到的 cluster 下标；-1 = 本帧没关联上
    int    missedFrames = 0;
    int    hits = 0;
    bool   justAcquired = false;
    bool   coasting = false;
    bool   confirmed = false;
    bool   gateBypassed = false; // 走了双锚点回退（马氏门没兜住，靠上次位置救回来）
};

struct TrackFrameDebug {
    // 【所有轨迹都在里面，不只是对外发布的那些】未转正的轨迹恰恰是
    // "编号乱跳"的现场：一个真点如果反复在 tentative 阶段夭折，对外表现就是
    // 它不断拿新编号，而只记转正轨迹的话，夭折的那些在文件里根本不存在。
    std::vector<TrackFrameDebugItem> items;
    double chiSquareGate = -1.0;   // 本帧生效的马氏门限（-1 = 该后端没有）
    double maxAssocDistMm = -1.0;  // 本帧生效的关联距离上限
    int    nObs = 0;               // 本帧输入的 cluster 数
    int    nTracks = 0;            // 内部轨迹总数（含未转正）
    int    nConfirmed = 0;
    int    nNewborn = 0;
    int    nKilled = 0;
    bool   ran = false;            // update() 这一帧真的跑了
};

}  // namespace mocap
