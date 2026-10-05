// =============================================================================
// Hm20AutoCalibTypes.hpp —— Hm20AutoCalib 的对外数据契约（枚举 / 配置 / 结果）。
// 纯数据结构 + 纯函数，零 Qt / 零 Eigen，可单测。
// 从 Hm20AutoCalib.hpp 拆出（原第 60~333 行），由 Hm20AutoCalib.hpp 统一 include。
// =============================================================================
#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

// Vec3 / Quat / detail 数学工具 都定义在 HandSkeletonAssociator.hpp。
#include "estimate/Hm20AssocContract.hpp"   // 【只要类型，不要那个类】原来这里 include 的是
                                            // HandSkeletonAssociator.hpp（闭包 5776 行），但本文件
                                            // 一次都没提到 Hm20SkeletonAssociator，要的只是
                                            // SkeletonFrameResult / Hm20Config / IHm20InferenceBackend
                                            // 这些类型 —— 它们在 contract 里（闭包 1557 行）。
                                            // estimate 目录下有 7 个头都犯了同一个错，每个多吃 4219 行，
                                            // 而这些头又被 UI 层层包含，代价是乘出来的。

namespace mocap {
namespace hm20 {
// add/sub/mul/dot/norm/cross/kabsch/matToQuat 都在 mocap::hm20::detail。
// 【必须自己写这一行】不能指望别的头文件先被 include 而把 detail 带进来 ——
// 那样单独 include 本文件就编不过，而且是"看 include 顺序脸色"的隐性依赖。
using namespace detail;

// ---------------------------------------------------------------------------
enum class AutoCalibStage : int {
    Idle = 0,        // 没看到合格的手
    Warmup,          // 手在了，等它稳下来
    Collecting,      // 正在攒手背样本
    Validating,      // 模板已冻结，正在用它自检（关键，见 onValidateFrame）
    Locked,          // 手背模板已锁定并在用；正在攒 anchor/骨长的样本
    Ready,           // 全部就绪（手背 + anchor + 骨长），IK 可以开
    Relocking        // 检测到贴点漂移，正在重标
};

// 本帧为什么没被采纳。UI 要拿它告诉操作者【具体该改什么】——
// "标定不动"如果只显示一个进度条不动，用户完全无从下手。
enum class AutoCalibReject : int {
    None = 0, NotEnoughPoints, LowConfidence, PentagonBad, MovingTooFast,
    SpreadTooSmall, NotRigid, WaitingSamples, AnchorImplausible
};

inline const char* autoCalibRejectName(AutoCalibReject r) {
    switch (r) {
        case AutoCalibReject::None:            return "";
        case AutoCalibReject::NotEnoughPoints: return "手背/手指点没认全";
        case AutoCalibReject::LowConfidence:   return "手背点置信度低";
        case AutoCalibReject::PentagonBad:     return "手背五点连线自交";
        case AutoCalibReject::MovingTooFast:   return "手动得太快，请放慢";
        case AutoCalibReject::SpreadTooSmall:  return "手背五点挨太近";
        case AutoCalibReject::NotRigid:        return "手背点距离不稳定(可能标错)";
        case AutoCalibReject::WaitingSamples:  return "样本积累中";
        // 束调整"收敛"了但解出的 anchor 在物理上说不通（典型：手不屈伸时
        // anchor 沿骨轴不可辨识，被解到手背反方向）。见 anchorPlausible()。
        case AutoCalibReject::AnchorImplausible: return "指根关节位置不合理(手指需屈伸)";
    }
    return "";
}

inline const char* autoCalibStageName(AutoCalibStage s) {
    switch (s) {
        case AutoCalibStage::Idle:       return "等待手进入视野";
        case AutoCalibStage::Warmup:     return "请把手摆稳";
        case AutoCalibStage::Collecting: return "正在标定手背";
        case AutoCalibStage::Validating: return "正在校验手背模板";
        case AutoCalibStage::Locked:     return "手背已锁定，正在学手指";
        case AutoCalibStage::Ready:      return "标定完成";
        case AutoCalibStage::Relocking:  return "检测到贴点移位，重新标定";
    }
    return "?";
}

struct AutoCalibConfig {
    // ---- 进入 Warmup / Collecting 的门槛 ----
    int    minDorsum          = 5;     // 手背至少认出几个点（要锁相位就得 5 个全有）
    int    minFinger          = 8;     // 手指点下限，保证画面里是完整一只手而不是半只
    double minDorsumConf      = 0.25;  // 手背点指派概率下限（手背天然只有 ~0.5，别设高）
    double maxSpeedMmPerFrame = 8.0;   // 手背质心帧间位移上限
    double maxRotDegPerFrame  = 5.0;   // 手背朝向帧间转角上限
    int    warmupFrames       = 24;    // 连续满足多少帧才允许开始采集（约 0.4~0.8 秒）

    // ---- 冻结后自检 ----
    // 【为什么必须有这一关】手背 5 点近似五重对称，网络在【还没有模板】时对
    // 手背的标法不唯一 —— 实测有整套标签反向排列的情况（半径序列跟真值正好
    // 倒过来）。刚体自检完全查不出来：同一组物理点换个标法，两两距离一模一样。
    // 一旦把这种模板冻结下去，Kabsch 解出的腕部系就是错的，后面 anchor 全废
    // （实测五指 anchor 平均错 44.6mm）。
    // 唯一可靠的判据是【把模板真正用起来看它自不自洽】：把模板下发给网络之后，
    // 网络会依据模板重新破对称，如果我们冻的那套标法跟网络在有模板条件下的
    // 输出不一致，Kabsch 残差会稳定地偏高（实测 0.79mm vs 3.19mm，4 倍分离）。
    int    validateFrames     = 90;
    // 【判据是相对噪声底的，不是一个绝对毫米数】
    // 固定阈值必然两头不讨好：设松了放过错标法（实测 2.2mm 就放过了一个
    // 标签反向的模板），设紧了在噪声大的现场永远标不成。
    // 正确的参考量是【手背自身的刚体一致性】：采集窗口里两两距离的标准差
    // σ 就是这套点位当前的噪声水平。模板标法对的时候 Kabsch 残差应该跟 σ
    // 同量级；标法错的时候残差由"形状对不上"主导，会明显高于 σ。
    double validateRmseMinMm  = 1.2;   // 下限，避免 σ 极小时阈值过严
    double validateRmseK      = 2.5;   // 阈值 = max(下限, K * σ)
    int    maxAttempts        = 6;

    // ---- 采集 ----
    int    collectFrames      = 48;    // 采集窗口长度
    // 手背两两距离的跨帧【标准差】上限。
    // 【不要改回极差(max-min)】48 帧样本、单点三角化噪声 σ≈1.5mm 时，距离的
    // 极差期望就有 4~7mm —— 用极差判刚体等于要求噪声为零，实测怎么摆都过不了，
    // 表现是"进度条卡在 Collecting 永远不动"。标准差跟噪声水平线性对应，
    // 才是能设阈值的量。
    double rigidTolMm         = 2.2;
    double minSpreadMm        = 12.0;  // 手背 5 点到质心的平均距离下限

    // ---- 锁定后的持续微调与漂移检测 ----
    double refineGain         = 0.015; // 模板 EMA 增益（每帧）
    double driftRmseMm        = 9.0;   // 手背 Kabsch 残差超过它
    int    driftFrames        = 90;    // 且连续这么多帧 => 判定贴点移位，重标

    // ---- anchor / 骨长 ----
    bool   fitAnchors         = true;
    bool   fitLengths         = true;
    int    anchorMinSamples   = 90;    // 每指近节球至少攒多少个样本
    double anchorMinSpreadDeg = 22.0;  // 样本在球心方向上的角展布下限（防退化）
    int    anchorMaxSamples   = 400;

    // ---- 束调整（anchor/骨长的精修）----
    bool   bundleRefine       = true;
    int    bundleShots        = 40;    // 姿势库容量
    int    bundleMinShots     = 16;    // 攒够多少张才开始解
    int    bundlePeriod       = 24;    // 每多少帧做一轮
    int    bundleIters        = 2;     // 每轮几次交替迭代
    double bundleDiversity    = 0.035;  // 新姿势跟库里所有姿势的最小差异度

    // ---- 拇指引导式标定 ----
    // 【为什么拇指要单独引导】拇指 CMC 是鞍状关节，还带一个逐人的轴向旋前
    // 耦合系数(thumbAxialK, 0.35~0.80)。自然活动里拇指的姿势展布往往不够，
    // 而这个系数只有在【大幅外展】时才可辨识(耦合项 = k * 外展角，外展角小
    // 的时候整项都接近 0，怎么拟合都无所谓)。四指靠自然屈伸就能收敛，
    // 拇指不行 —— 实测拇指 anchor 误差 14.8mm vs 四指 2.6~3.8mm。
    // 所以给拇指一个 3 秒的引导动作，把它的可辨识性一次做足。
    // 【默认关 —— 隐形标定】原来为 true 时会给出"拇指大幅外展再收回（张成 L 形）"
    // 这类规定动作提示，用户必须配合才能推进。两个问题：
    //   ① 破坏"无感"：用户得停下手上的事去做体操
    //   ② 【标不好是原理性的】hand_rig.py 里拇指旋前完全耦合在 CMC 外展上、
    //      没有常数基线，自标定要反解一个 rig 里不存在的自由度 —— 做多少次
    //      引导动作都收敛不到对的值。缺的是模型自由度，不是数据。
    // 关掉之后拇指参数一直用群体均值（那一根精度差些），四指照常独立收敛，
    // 而且 ikUsable() 本来就不因拇指没标好而挡 IK。
    bool   guidedThumb        = false;
    double thumbMinSpreadDeg  = 32.0;   // 近节球方向张角下限
    double thumbMinDpRangeMm  = 70.0;   // 远节球在腕部系里的活动范围下限
    bool   fitThumbAxialK     = true;
    // 拇指常数旋前是【解剖常数】，一个人一辈子不变。建议标定一次之后把拟合值
    // 填进 lockedThumbPronation0 并把这个关掉——每次会话重搜只会引入抖动。
    bool   fitThumbPronation0 = true;
    // Ready 之后，shot 数要比上次束调整时多这么多才重跑。按 shot 增量而不是
    // 按时间：手放着不动时不该重跑，只有真采到新姿势才值得重解。
    int    bundleRefitGrowth  = 12;
    // 【收敛停滞就彻底停】Ready 之后连续这么多次 refine 都没把残差压低
    // stallImproveRatio 以上，就不再 refine。
    // 真机踩到的：对着一个【刚体标定件】跑（不是手），手的模型根本拟合不上，
    // bundleRmse 在 28~35mm 之间来回晃永远收敛不了，却每 1.4 秒烧 280ms，
    // 而那 280ms 在推理线程上 —— 骨架冻住 290ms 再瞬移 36mm。
    int    bundleStallRuns    = 3;
    double stallImproveRatio  = 0.03;
    // 拇指旋前最多搜这么多次就定死。48 次全 shot 代价评估是束调整里最贵的
    // 一块，而它是解剖常数 —— 搜三次还定不下来，再搜也不会更准。
    int    thumbPronMaxTries  = 3;
    double lockedThumbPronation0 = 0.0;   // 非 0 时直接用它，不再搜

    // ---- 左右手自动判定 ----
    bool   detectHandedness   = true;
    double handednessMinMm    = 4.0;   // 手指点相对手背平面的平均有符号距离阈值
    // 【默认只上报、不改】—— 这是一次真机 bug 的修复，改回 true 之前先读完：
    //
    // 推断出的手性【不只是一个标志位】：下面 finalizeCollect 里判成"左手"时会把
    // 局部系的 Y 轴整个翻过来(Y=-Y)，于是存进 backMm 的手背模板【被镜像】。
    // 镜像后的五点模板拿去做 Kabsch 仍然能拟合出不大的残差(手背近平面 + 五点
    // 接近对称，反射≈绕面内轴转 180°)，所以 Validating 阶段的 RMSE 自检【查不出来】——
    // 结果就是：采集阶段一切正常，模板一提交(Collecting->Validating 的那一刻)
    // 手背和手指连线立刻错，而所有诊断数字看起来都正常。
    //
    // 而推断本身很脆：判据是手指点相对手背平面的平均有符号距离，阈值只有 4mm。
    // 采集阶段提示语是"请把手摆稳"，用户多半把手摊平 —— 摊平时这个距离本来就
    // 接近 0，噪声和个别手指标签错就足以把符号带偏。
    //
    // 面板上的「右手」是用户的【显式声明】，比一个 4mm 阈值的推断可信得多。
    // 所以默认：推断照算、冲突照报，但不拿它改任何东西。
    // 【默认关，这是第二个真机 bug 的修复，改回 true 之前先读完】
    //
    // 手背编号规范化会按极角把模板的 5 个点【重排】：tmp[i] = backMm[ord[i]]。
    // 但 Kabsch 是【按标签配对】的 —— tmplMm_[m] 配观测 markers[m]，而 markers
    // 的编号是网络给的。重排之后模板位置 m 装的其实是网络标签 ord[m] 的那个点，
    // 于是整个对应关系被打乱，解出来的腕部位姿整体转掉一格 ≈ 72°。
    //
    // 原设计押的是"网络看到新模板后会跟着重新编号"。但手背五点近正五边形，
    // 循环移位正是网络最分不清的情形 —— 押不中的时候没有任何东西会报错：
    // 五边形转 72° 几乎还是原来那个五边形，【Kabsch 残差依旧很小】，
    // Validating 阶段的 RMSE 自检完全看不出来。
    // 症状就是"标定阶段正常，一提交手掌和手指连线全错，而诊断数字全绿"。
    //
    // 关掉之后 index i 恒等于网络标签 i，对应关系不会被动过。
    // 代价：模板在不同会话之间的编号不再规范化 —— 而模板本来就是每次重新标的，
    // 这个代价基本不存在。
    // 【隐形标定】不给用户任何"请做XX动作"的引导，只在后台收集正常使用中
    // 自然出现的姿势，攒够覆盖度就热替换。理由：手指标定要的是"关节角覆盖
    // 得够广"，而用户正常用手本来就会覆盖 —— 规定动作只是让它快一点，不是必需。
    //
    // 【拇指阶段直接跳过】hand_rig.py 里拇指旋前完全耦合在 CMC 外展上、没有
    // 常数基线，自标定要从观测反解一个 rig 里根本不存在的自由度 —— 引导用户
    // 做多少次"张成 L 形"都标不好。这不是数据不够，是模型缺自由度。
    // 静态初值：等一个手指伸直的帧，用向导的闭式解直接量骨长。见 tryStaticSeed()。
    bool   staticSeed          = true;
    double staticSeedMaxDevMm  = 6.0;   // 共线偏差门限，超过=手是弯的，不收
    bool   silentMode = true;
    bool   canonicalizeDorsum = false;
    bool   applyHandedness    = false;
    bool   userIsRight        = true;   // 面板「右手」的当前值，作为生效值
};

// ---------------------------------------------------------------------------
// 一次自标定的完整产物
// ---------------------------------------------------------------------------
struct AutoCalibResult {
    std::array<Vec3, 5>  backMm{};                 // 手背 5 点（腕部系，mm）
    std::array<Vec3, 20> markersMm{};              // 中立位 20 点（腕部系，mm）
    std::array<Vec3, 5>  anchorsMm{};              // 5 个指根关节（腕部系，mm）
    std::array<std::array<double, 3>, 5> lengthsMm{};
    std::array<bool, 5>  anchorFitted{};           // 该指 anchor 是解出来的，不是抄常量
    std::array<bool, 5>  lengthFitted{};
    std::array<double, 5> anchorResidMm{};         // 球面拟合残差
    // 【拟合质量三件套】只看"收集进度 100%"是不够的 —— 那只说明样本攒够了，
    // 不说明解出来的参数是对的。真机上 IK 残差 18~40mm 而进度显示 100%，
    // 就是因为这两件事被混在同一个数字里。
    //   resid   球面拟合残差：大 = 这些样本根本不在一个球面上（标签串了/腕部系抖）
    //   spread  样本的角度覆盖：小 = 手指没怎么活动，球心在轴向上不可辨识
    //   n       样本数
    std::array<double, 5> anchorSpreadDeg{};
    std::array<int, 5>    anchorSamples{};
    std::array<bool, 5>   staticSeeded{};      // 该指已由静态伸直帧闭式定过初值
    std::array<double, 5> staticSeedDevMm{};   // 当时的共线偏差(手放得平不平)
    // |anchor - pp| 的跨帧统计。这两点在同一根骨头上，距离恒定，所以
    // std 直接反映 anchor 对不对 —— 见 process() 里的说明。
    std::array<double, 5> anchorRigidStdMm{};
    std::array<double, 5> anchorRigidMeanMm{};
    int wristPoseValidCount = 0;
    bool   isRight       = true;
    bool   handednessKnown = false;
    // 推断值与生效值分开存：isRight 是【实际用于建系/建模板/送网络】的那个，
    // isRightDetected 是自标定自己算出来的。两者不一致时 handednessConflict=true，
    // 由 UI 提示用户，而不是悄悄按推断值走。
    bool   isRightDetected = true;
    bool   handednessConflict = false;
    double handSignMm = 0.0;       // 手指点相对手背平面的平均有符号距离(判据本身)
    double handSignLatMm = 0.0;    // 横向判据：食指相对小指在横轴上的投影
    bool   handednessAgree = true; // 横向判据与拇指判据是否一致；false=拇指多半歪了
    bool   mirrorSuspect = false;  // 提交后自检发现模板疑似被镜像
    bool   dorsumWouldReorder = false;  // 规范化【本来会】重排（不管有没有真的重排）
    // 局部系是否已经对齐到【解剖学腕部系】（+X = 中立位手指方向、+Z = 手背法向）。
    // false 时局部系只是"一个自洽的系"，够 Kabsch 和网络用，但【不够 IK 用】——
    // 见 tryAnatomicalAlign() 的说明。IK 必须等这个标志为 true 才允许开。
    bool   frameAnatomical = false;
    // 冻结时手背编号被重排过（说明网络当时给的标法不是规范序）。
    // 重排后第一帧网络的标签还没跟上，Kabsch 残差会短暂抬高，属正常。
    bool   dorsumReordered = false;
    double thumbAxialK   = 0.576;     // 拇指轴向旋前耦合（逐人）
    bool   thumbAxialFitted = false;
    // 第一掌骨的【常数】解剖旋前(rad)。见 Hm20MarkerModel::thumbPronation0。
    // 【跟 thumbAxialK 的可辨识性完全不同】耦合项 = k*外展角，外展角一直很小时
    // 任何 k 都差不多、搜出来是噪声，所以那个要 thumbCoverage()>=0.6 才敢搜；
    // 而常数基线【中立位就在起作用】，只要拇指有过屈曲就能辨识，门松得多。
    double thumbPronation0 = 0.0;
    bool   thumbPronationFitted = false;
    // 碗底对比度 = sqrt(最差档均方 / 最优档均方)。>=2 才算真搜到了东西；
    // 接近 1 说明曲线是平的，量出来是噪声，此时不写入、保持 0(等价旧行为)。
    double thumbPronationContrast = 0.0;
    // 已定死，本会话不再重搜。第一掌骨的常数旋前是解剖常数，一个人一辈子不变，
    // 而每次束调整都重搜它要跑 48 次全 shot 代价评估 —— 那是每秒 250ms 卡顿
    // 里最大的一块。碗底对比度够明显（>=3）就锁。
    bool   thumbPronationLocked = false;
    // 束调整已判定收敛停滞、不再重跑。面板要显示 ——
    // 否则"为什么标定不动了"又会变成一轮盲查。
    bool   bundleStalled = false;
    // 【本次会话累计跑了几次束调整】这是判断"退避到底有没有生效"的唯一直接依据。
    // 我做过一次离线复算想验证退避，但从 .pcrec 重建 SkeletonFrameResult 会丢状态，
    // 复算跑不出实机的 Ready 态反复重解，等于证明不了。这个计数器录进文件之后，
    // 下一段录制就能直接看出来：它每秒涨一次 = 退避没生效。
    int    bundleRuns = 0;
    // 单次束调整的耗时(ms)，取最近一次。跟 latencyMs 的尖峰对得上就坐实了。
    double bundleLastMs = -1.0;
    double bundleRmseMm  = -1.0;      // 束调整的整体拟合残差；<0 = 还没跑过
    int    bundleShots   = 0;
    double handLenMm     = 185.0;
    bool   backValid     = false;
};

} // namespace hm20
} // namespace mocap
