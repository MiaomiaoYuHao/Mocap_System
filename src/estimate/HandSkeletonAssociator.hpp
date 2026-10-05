#pragma once
// ---------------------------------------------------------------------------
// hm20 手部 20 点骨骼关联 —— 替换 HandSkeletonAssociator.hpp
// 纯数学，零 Qt / 零 Eigen 依赖，只用 <array>/<vector>/<cmath>，可单测。
//
// 【与旧版 HandSkeletonAssociator 的差别，逐条对应交接文档第 5 条】
//   1. 不再先跑 HandColdStart。旧管线整条链路建立在"手背 5 点两两距离匹配
//      一定对"的假设上，而实测这 5 点距离区分度中位数只有 ~1.3mm，跟检测
//      噪声同量级 —— 第一步错，坐标系错，后面全错。新网络输入只用旋转/
//      平移/尺度不变量，候选点可以是【任意坐标系】下的原始点，网络内部
//      自己做去中心化和归一化。腕部位姿改为【标注完成之后】用 Kabsch 从
//      已标注的手背点解出来，顺序反过来了。
//   2. 分类从 16 类变 21 类（20 个标记点 + 1 个 dustbin），并且从"逐点独立
//      argmax"改成【匈牙利算法一对一指派】。旧版没有"每个标签最多用一次"
//      的约束，会出现四个点同时判成 index_mid、而 index_prox 一个都没有 ——
//      这正是截图里乱连线的直接原因。dustbin 列可重复使用，所以候选点数
//      不等于 20 完全没问题。
//   3. ONNX 输入从 3 个 tensor 变 6 个（见 kOnnxInputNames）。
//   4. 连线拓扑改为 15 条边 / 6 个互相独立的连通分量：手背闭合五边形 +
//      5 条手指折线。手指之间不连，手背与手指之间也不连（旧版的
//      fingerMountEdges 已删除）。
//
// 【本文件做什么 / 不做什么】
//   做：指派求解、腕部位姿(Kabsch)、五边形校验、连线拓扑、骨骼朝向。
//   不做：ONNX 会话本身（IHm20InferenceBackend 纯虚，按你的构建系统接）；
//         遮挡点的 IK 补全（见文件末尾 IHm20IkRefiner，默认退化为直接采用
//         网络 pos 头的预测，精度够看但不如 IK；IK 端口是剩余工作）。
// ---------------------------------------------------------------------------
#include <array>
#include <deque>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>
#include <cstdio>

#include "estimate/DorsumRigidSolver.hpp"
#include "estimate/Hm20SegRot.hpp"

// ===========================================================================
// 【文件地图】—— 改之前先看这里，别整包读
//
// 这个类有 73 个函数、92 个数据成员。按依赖关系统计过：92 个成员里有 80 个
// 只被 1~3 个函数碰。也就是说它不是一团乱麻，是【几台互不相干的状态机
// 挤在一个类里】。所以分片是按【职责】切的，不是按行号切的——
// 想改哪件事，只要读对应那一个文件就够，不用把三千行全拉进来。
//
// 类型与契约（独立头文件，可单独编译）：
//   Hm20AssocBase.hpp       别名 / 标签 / 拓扑 / detail 数学
//   Hm20AssocContract.hpp   推理与 IK 接口、SkeletonFrameResult、Config、拇指模型
//
// 类体分片（#include 在 class 内部，按职责分）：
//   Hm20Assoc_types   .ipp     17 行   类型与配置别名
//   Hm20Assoc_api     .ipp    389 行   对外接口 + 编排状态 + 整体复位
//   Hm20Assoc_process .ipp    609 行   每帧主流程（编排层）
//   Hm20Assoc_chain   .ipp    951 行   链式续解（遮挡点补全）
//   Hm20Assoc_dorsum  .ipp    447 行   手背：跟踪 / 刚体位姿 / 几何重标 / 手性投票
//   Hm20Assoc_thumb   .ipp    194 行   拇指专属：预测点回正 / 网络分段 / 旋前标定
//   Hm20Assoc_segrot  .ipp    246 行   骨骼朝向：分段四元数 / swing 构造 / 半球对齐
//   Hm20Assoc_geom    .ipp    273 行   骨长学习 / 骨长回正 / 预测点平滑 / anchor 自检
//
// 【怎么用这张地图】
//   要改遮挡补全 / PIP 保持 / DIP 耦合        -> chain
//   要改腕部系 / 手性判定 / 手背重标           -> dorsum
//   要改四元数、骨轴、swing 构造               -> segrot
//   要改骨长回正、预测点平滑、anchor 自检      -> geom
//   要改拇指的任何特判                         -> thumb
//   要改调用顺序 / 帧级流程                    -> process
//   要加 setter、改 configJson、加复位状态     -> api
//
// 【每个分片开头都显式写了 public:/private:】不依赖上一个分片以什么结尾，
// 调换下面的 include 顺序不会静默改变成员可见性。
// 每个分片可以单独做语法检查：tests/test_assoc_fragments.cpp
// ===========================================================================
#include "estimate/Hm20AssocBase.hpp"
#include "estimate/Hm20AssocContract.hpp"

namespace mocap {
namespace hm20 {

class Hm20SkeletonAssociator {
// 类型必须最先——它们被后面的函数当返回类型用。
#include "estimate/Hm20Assoc_types.ipp"
#include "estimate/Hm20Assoc_api.ipp"
#include "estimate/Hm20Assoc_process.ipp"
#include "estimate/Hm20Assoc_chain.ipp"
#include "estimate/Hm20Assoc_dorsum.ipp"
#include "estimate/Hm20Assoc_thumb.ipp"
#include "estimate/Hm20Assoc_segrot.ipp"
#include "estimate/Hm20Assoc_geom.ipp"
};

} // namespace hm20
} // namespace mocap

// ===========================================================================
// 接入点云测试界面（PointCloudTestDialog）
// ===========================================================================
//   1. 成员：std::unique_ptr<mocap::hm20::Hm20SkeletonAssociator> assoc_;
//      构造时传一个接了 onnxruntime 的 IHm20InferenceBackend 实现，
//      以及从标定导出的 61 维 tmpl（没有标定就传 tmplValid=false，能跑）。
//   2. 渲染循环里：
//        std::vector<std::pair<int, mocap::hm20::Vec3>> cands;
//        for (auto& p : points) cands.push_back({p.id, {p.pos.x(), p.pos.y(), p.pos.z()}});
//        auto skel = assoc_->process(cands);
//        if (skel.valid) {
//            QVector<QPair<int,int>> edges;
//            for (auto& e : mocap::hm20::skeletonEdges())      // 15 条
//                edges.push_back({10000 + e.first, 10000 + e.second});
//            widget_->setEdges(edges);                          // 20 点用 10000+label 当 id
//        }
//      6 个连通分量互相独立，也可以直接按 polylines() 逐条画折线，
//      那样从渲染层就不可能把手指跟手背连起来。
//   3. 手切换/长时间丢失后调 resetHistory()，避免拿一帧很旧的 prev 误导网络。
// ===========================================================================
