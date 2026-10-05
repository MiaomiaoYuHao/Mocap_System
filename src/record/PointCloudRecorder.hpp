// =============================================================================
// PointCloudRecorder.hpp —— 原始数据流录制（.pcrec）
// =============================================================================
// 【为什么要录两级，而不是只录 3D 点】
// 这套系统有两级独立可调：
//     2D光斑(每相机+时间戳+协方差) --[IekfPointTracker,约30个参数]--> 3D候选点
//     3D候选点 --[hm20关联 + IK + 滤波 + 自标定]--> 骨架
// 只录 3D 点的话，第一级那三十个参数（关联距离、过程噪声、马氏门、密度门、
// 机动自适应Q……）一个都调不了，因为它们的输入根本没被保存下来。
// 所以格式里 2D 和 3D 都存，外加当前参数下的骨架输出【作为基线】——离线复现
// 时先比对基线，对得上才说明离线复算跟在线是同一回事，比对不上就是我这边的
// 复现有问题，而不是参数好坏的问题。这一步不做，后面所有调参结论都不可信。
//
// 【为什么必须存标定和参数快照】
// 3D 点只是结果，重算需要相机内外参；参数快照则是"这份数据是在什么设置下录的"
// 的唯一凭据。没有它，一个月后没人说得清某段数据当时开没开某个开关。
//
// 【格式设计取舍】
// 二进制 + JSON 头。不用纯 JSON/CSV 是因为体积：8相机 × 25光斑 × 120fps，
// JSON 大约 1MB/s，一分钟 60MB，还要上传给人分析。二进制约 0.6MB/s。
// 分块 + 每块自带长度，所以【中途崩溃/断电的文件依然可以读到最后一个完整块】——
// 录制场景下这个比校验和重要得多。
//
// 【线程】写盘在自己的线程里做，采集线程只往队列里塞一个 vector<char>。
// 采集线程绝不碰文件句柄：120fps 下一次几十毫秒的磁盘抖动就是一串丢帧。
// 队列有上限，满了丢块并计数（宁可丢数据也不能阻塞采集）。
// =============================================================================
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <array>
#include <utility>

namespace mocap {
namespace pcrec {

// ---------------------------------------------------------------------------
inline constexpr char     kMagic[8]   = {'M','C','P','C','R','E','C','\0'};
// v3 = 增加 Hm20Diag 块(类型9)。旧解析器遇到未知块类型按长度跳过，
// 所以 v2 的脚本读 v3 文件不会崩，只是看不到新块。
//
// v4 = 补齐【输出链路】和【手性证据链】。加块 13~17，并把一直没接线的
//      10/11 真正接上。加这一版的直接原因是两个查不下去的问题：
//
//   ① "关节角/四元数看着不对，握拳时像张开" —— v3 里【一个字节都没录】
//      关节角。solveJointAngles -> 角度平滑 -> ROM 映射 -> 速度限幅 -> UDP
//      这四级，任何一级把信号压平都是同一个症状，而文件里查不到是哪一级。
//      现在四级的值分开录（JointOut 块），一眼能看出信号在哪一级消失。
//
//   ② "手性判别出错，不知道错在哪" —— v3 只录了 4 个 bool，而真正决定
//      结果的量（横向判据 lat、两判据是否一致、几何锁定值、模型 known 位、
//      送进网络的 tmpl_.isRight、IK 用的手性）全都没录。四个 bool 全都
//      "看起来正常"而结果是错的，正是因为出错的那个量不在这四个里面。
//      现在整条证据链一帧一条（Handedness 块）。
//
// 旧解析器读 v4 会按长度跳过 13~17，不会崩。
//
// v5 = 补齐【中间级】和【上下游两端】。加块 18~29。
//      v4 解决的是"输出的那一个数是怎么来的"，但只解决了关节角那一路。
//      这一版把同样的原则推到其余每一路。三条主线：
//
//   ① 位置这一路只有首尾两级。posNet(网络原始) 和 posFinal(全部后处理+滤波
//      之后)，中间的 指派→几何→IK→骨长回正 四级一个都没有。关节角能分四级
//      定位，位置却只能二选一 —— 而位置错了关节角必然跟着错，位置这一路
//      恰恰是更上游、更该先查的那一路。现在六级全录（MarkerStage 块）。
//
//   ② 滤波器是【就地覆写 result】的，滤波前的值在这个系统里不存在第二份。
//      于是"几何本来就解错了"和"解对了被平滑吃掉了"在文件里完全一样。
//      这跟 v4 里 qSolve/qSmooth 分开录是同一个道理，只是当时只做了角度那一路。
//      现在滤波前后 + 滤波器内部状态（腕部系速度、停滞帧、硬重置、死区、
//      融合权重、逐点截止频率）都录（FilterDbg 块）。
//
//   ③ 模板只在文件头存一次，而自标定会在录制中途热替换它。也就是说
//      "第 N 帧真正送进网络的模板长什么样"，恰恰是手性问题的核心证据，
//      而文件里没有。现在模板一变就落盘一份全量快照（TemplateSnap 块），
//      并且带 fnv1a 指纹，一眼能看出哪一帧换了模板、换成了什么。
//
//      配套的还有事件流（Event 块）：手性翻转、模板被拒、看门狗松锁退模板、
//      模型切换、参数改动，全部带时间戳和原因文本。原来这些只在某一帧的
//      标量里留个痕，没有"何时、因为什么而变"这条线。
//
//      以及 IK 内部（IkDbg 块）。IK 的 refine() 内部会按自己的 isRight_ 把
//      左手镜像成右手系再解 —— 这是手性出错的一条【独立路径】，而它解出来的
//      4 自由度关节角、限位命中、实际用的骨长和 anchor，一个都不出 refine()。
//
//   其余补的：上游 2D↔3D 的对应关系（ClusterLink，哪台相机的第几个光斑
//   构成了这个 3D 点 —— 查粘连和鬼点的唯一线索，而 Track3D.support 里本来
//   就有，只是被压成位图丢掉了）、追踪器逐点内部状态（TrackDbg）、
//   时基质量与分阶段耗时（Timing）、UDP 输出闭环（UdpOut）、
//   全量指派矩阵（AssignFull，按需开）、自标定逐帧产物（AutoCalibDbg）。
//
// 旧解析器读 v5 会按长度跳过 18~29，不会崩。
// v6 = 加 ChainDbg(30)。
//
// v7 = 补齐【ROM 标定】这一路。加块 31/32，加事件码 16/17。
//      加这一版的直接原因：ROM 标定标不满（覆盖度长期 50% 上下）、
//      标完有几维是反的（握拳读数小、张开读数满），而 v6 的文件里
//      【查不出原因】—— JointOut 块只有 qSolve..qOut 四级的结果，
//      而 ROM 的问题出在这四级【之前】：
//
//   ① 逐维新鲜度没有记。solveJointAngles 在腕部系失效时【只更新 PIP】，
//      MCP 屈曲/外展保持上一帧，然后 fingerValid 仍然置 true。
//      文件里只有逐指的 fingerValid，于是「这一维是新算的」和
//      「这一维是上一帧的」在 qSolve 里长得完全一样 —— 而 ROM 采样
//      正是照着它采的，采进去的是一堆重复旧值。
//
//   ② 骨轴退化没有记。近节骨轴在 computeSegmentQuats 里有两条来源：
//        anchor->pp球   （正确）  需要 f!=0 && anchorsValid && wristPoseValid
//        pp球->mp球     （退化）  否则
//      而 solveJointAngles 里算 PIP 的第二条轴恒等于 pp球->mp球。
//      三个条件任一不满足，两条"骨轴"就是同一个向量，PIP=acos(1)=0。
//      拇指 f!=0 恒假 => 拇MCP 永远是 0；四指在手背被挡时退化 ——
//      而握拳恰恰是手背最容易被挡的时候。
//      这一条同时解释了「标不满」和「方向反了」，而文件里一个字节都没有。
//
//   ③ ROM 标定按钮按下/结束这两个瞬间没有标记。事后对着几千帧找
//      「标定是从哪一帧开始的」只能靠 RF_RomLearning 位做差分，
//      而那个位是逐帧采样的，按钮按下和第一帧生效之间还差一个投递延迟。
//
//      现在：块 31 存逐维标定全量状态（样本数、逐原因拒收数、方向及其
//      判据、两端相位中位、外推量、状态码），块 32 存逐帧解算内部量
//      （两条骨轴、dot(a0,a1)、铰链符号、proxAxisSrc、curl、逐维 dofState），
//      事件 16/17 存按钮的两个瞬间（带墙钟，跟帧号分开）。
//
// 旧解析器读 v7 会按长度跳过 31/32，不会崩。
//
// v8 = 补齐【ROM 映射器内部】和【标定配置】。加块 33，给 RomCalibRec 接一段
//      配置快照，并把一直没接线的 23(UdpOut) 真正接上。
//      加这一版的直接原因是三个在 v7 的文件里【查不出来】的现场问题：
//
//   ① 「ROM 标定完输出就冻住不动了」
//      Mapper::apply() 里通向"这一维再也不动"的路一共有四条：
//        NoData/AxisDead      -> neutral()   永久钉在中立位
//        dofState 不新鲜+hold -> last_       永久保持上一帧
//        span<1e-3            -> neutral()   区间塌了
//        t 恒被 clamp 到 0 或 1              贴在行程端点
//      四条路的【输出长得一模一样】：一个不动的数。而 v7 只录了这个数
//      （JointOut.qRom），四条路一条都分不开 —— 修法却完全不同：
//      第一条要回去查骨轴退化，第二条要查 dofState，第三条要重标，
//      第四条是标定区间本身没覆盖到当前姿态。
//      现在逐维逐帧记【走的是哪条分支】，外加连续冻结帧数（块 33）。
//
//   ② 「标定之前就有几维无缘无故一直触限」
//      未标定时 Mapper 走的是 out = clamp(q, tLo, tHi)。要解释"为什么触限"，
//      需要 q(有) 和 tLo/tHi(【一个字节都没有】)。目标行程可以被
//      setTargetRange() 改写，而改写后的值从来没出过 Mapper。
//      两个数不在文件里，这个问题就只能靠猜。现在 tgtLo/tgtHi 和
//      两端各自的触限位逐帧落盘。
//
//   ③ 「行程总是不够」
//      RomCalibRec 记了结果（覆盖度、样本数、拒收原因），但没记【判据】——
//      minRangeRad / minSamplesPerDof / acceptPredicted / extrapMinR2 …
//      这些阈值决定了同一批样本判成 Ok 还是 PriorFilled。有人调过阈值之后
//      录的数据，跟默认阈值下录的数据，在 v7 的文件里看起来完全一样。
//      现在配置快照跟结果存在同一条记录里。
//
// 旧解析器读 v8 会按长度跳过 33；RomCalibRec 变长了(2040->2112)，
// 旧脚本会拿 recBytes 一比对不上、报 schema 错误并跳过这一块 ——
// 这是设计好的行为（绝不静默错位），升级 tools/pcrec/pcrec.py 即可。
inline constexpr uint32_t kVersion    = 8;

// 【头文件修订号】跟 kVersion 不是一回事：kVersion 写进文件、给解析器看；
// 这个只给编译器看，每次改 Hm20DiagRec 的字段或新增块类型就 +1。
//
// 【为什么需要它】这几个头文件是互相依赖的：PointCloudTestDialog.cpp 用
// Hm20DiagRec 的字段，只更新 .cpp 不更新这个头，会得到一串
// "has no member named 'xxx'" —— 五个字段就是五条错误，而真正的原因
// （少拷了一个文件）一个字都没提。有了这个常数，static_assert 能一句话说清。
inline constexpr int kHeaderRevision = 9;

enum class ChunkType : uint8_t {
    CamBlobs   = 1,   // 一台相机一帧的 2D 光斑
    Points3D   = 2,   // 一帧的 3D 候选点（追踪后端的输出）
    Skeleton   = 3,   // 一帧的骨架输出（基线参考）
    Mark       = 4,   // 用户打的标记
    ParamDelta = 5,   // 录制中途改了参数
    Cluster3D  = 6,   // 多视角匹配的原始输出（追踪之前）
    Trailer    = 7,   // 收尾统计（丢块数等）。写在文件末尾
    ClockSync  = 8,   // 相机时钟 <-> 墙钟 的对应关系
    Hm20Diag   = 9,   // 骨架的逐帧诊断（SkeletonRec 装不下的那些）
    AssocInput = 10,  // 送进 hm20 的候选点【及其顺序】——离线复现必需
    NetRaw     = 11,  // 网络原始输出——分开"模型错"和"后处理错"的唯一依据
    CamSettings= 12,  // 相机运行参数（曝光/增益/阈值/ROI）
    // ---- v4 新增 ----
    JointOut   = 13,  // 关节角输出全链路（解算/平滑/ROM/限幅）+ 分段四元数
    Handedness = 14,  // 手性判定的全部证据（面板/自标定/几何/模型/实际生效）
    MarkerDbg  = 15,  // 逐 marker 的来源与位置（网络预测 vs 最终，搬了多远）
    RunFlags   = 16,  // 逐帧生效的功能开关
    StateJson  = 17,  // 1Hz 全量运行时状态 JSON（worker 早就在生成，一直没落盘）
    // ---- v5 新增 ----
    MarkerStage= 18,  // 20 点位置的六级流水（网络/指派/几何/IK/后处理/滤波后）
    FilterDbg  = 19,  // 滤波前后 + 滤波器逐点内部状态
    IkDbg      = 20,  // IK 逐指解、限位、代价、实际用的骨长/anchor/手性
    TemplateSnap=21,  // 本帧生效的模板全量快照（变化时写）
    Timing     = 22,  // 时基质量：各相机时间戳离散度、跳帧、队列、分阶段耗时
    UdpOut     = 23,  // 实际发出去的包（内容摘要 + 发送状态 + 序号）
    Event      = 24,  // 状态变更事件流（带原因文本）
    AutoCalibDbg=25,  // 自标定逐帧产物
    AssignFull = 26,  // 全量 log_assign 矩阵（按需开）
    TrackDbg   = 27,  // 3D 追踪器逐点内部状态
    ClusterLink= 28,  // 3D 点 <-> 2D 光斑的对应关系
    NetInput   = 29,  // 送进 ONNX 的归一化输入（按需开）
    // ---- v6 新增 ----
    ChainDbg   = 30,  // 链式续解逐指内部状态（遮挡点 81% 走这条路）
    // ---- v7 新增：ROM 标定这一路 ----
    RomCalib   = 31,  // ROM 标定的逐维全量状态（开始/结束/标定中 1Hz）
    JointSolve = 32,  // 关节角解算的逐帧内部量（两条骨轴、退化判据、curl）
    // ---- v8 新增 ----
    RomMapDbg  = 33,  // ROM 映射器逐维逐帧走了哪条分支 + 目标行程 + 冻结计数
    AngleChain = 34,  // 角度平滑器 + 速度限幅器的逐维隐藏状态
    ChunkStats = 35,  // 逐块类型的 写入/丢弃/字节 统计（收尾时写）
};

// ---------------------------------------------------------------------------
// 【丢块优先级】队列满时先丢哪一类。
//
// 【为什么必须分级】原来是"满了就丢当前这一块"，也就是【按到达顺序丢】。
// 后果是：一份 RomCalib 快照（一次会话只写 3 份，标定结束那一份【录不到就
// 只能重标】）和十万分之一块 2D 光斑，被丢掉的概率完全一样。
// 而一次几百毫秒的磁盘抖动，丢掉的恰好就是那一瞬间的东西 ——
// 排查时最想要的那几份，正是最容易在抖动里消失的那几份。
//
// 分级之后：高频大块先让路，低频不可再生的证据最后才丢。
// 门限按队列占用率给，越关键的门限越高：
//   Bulk       60%   高频、可再生（下一帧还有一份长得差不多的）
//   Diagnostic 85%   逐帧诊断，丢几帧不影响定性
//   Critical  100%   事件/标定/模板/参数：一次会话就几份，丢了无法补
//
// 【为什么不是"关键块阻塞等待"】阻塞采集线程 = 丢帧 = 数据本身就废了。
// 宁可在极端情况下丢一份关键块并【如实记进块 35】，也不能卡住采集。
enum class DropTier : uint8_t {
    Critical = 0,   // 一次会话几份，不可再生
    Diagnostic = 1, // 逐帧诊断，丢几帧不影响定性
    Bulk = 2,       // 高频大块，下一帧还有
};

inline DropTier dropTier(ChunkType t) {
    switch (t) {
        // 不可再生：事件、标定结果、模板、参数、时钟、收尾统计
        case ChunkType::Mark:
        case ChunkType::ParamDelta:
        case ChunkType::Trailer:
        case ChunkType::ClockSync:
        case ChunkType::CamSettings:
        case ChunkType::TemplateSnap:
        case ChunkType::Event:
        case ChunkType::StateJson:
        case ChunkType::RomCalib:
        case ChunkType::ChunkStats:
            return DropTier::Critical;
        // 逐帧诊断：小、密，但丢几帧不改变结论
        case ChunkType::Skeleton:
        case ChunkType::Hm20Diag:
        case ChunkType::JointOut:
        case ChunkType::JointSolve:
        case ChunkType::RomMapDbg:
        case ChunkType::AngleChain:
        case ChunkType::Handedness:
        case ChunkType::RunFlags:
        case ChunkType::UdpOut:
        case ChunkType::Timing:
            return DropTier::Diagnostic;
        default:
            return DropTier::Bulk;
    }
}

// 录制详细度。【为什么要有这个】v5 把每帧的块数从 6 个提到了 14 个左右，
// 全开约 1.4MB/s（v4 约 0.6MB/s）。一分钟 84MB —— 对 30 秒的排查性录制完全
// 可以接受，对"录半小时看看有没有偶发问题"就不行了。
//
// 【默认给 Full 而不是 Basic】录制这个动作本身是为了排查，而排查时最贵的
// 从来不是磁盘，是"录完发现关键那一项没开、得重录一遍"——尤其是偶发问题，
// 重录不一定复现得出来。所以默认全录，想省空间的人自己往下调。
//
// 【v8 重新划线：关节角这一路整条不受档位限制】
// 原来 13(JointOut) 在 Basic、32(JointSolve) 在 Full，也就是说 Basic 档录的
// 文件里"输出的角是什么"有，"它是怎么算出来的"没有。而现场报得最多的
// 那一类问题（标定完输出冻住 / 一直触限 / 行程不够），答案【全部】在后者里。
// 证据链只要断一环，剩下几环就只能证明"问题不在我这一级"，证不出在哪一级。
//
// 现在 13/32/33/34 + 23(UdpOut) + 31(RomCalib) 一律无条件写，合计约 2.6KB/帧，
// 120fps 下约 310KB/s。Basic 从 0.6 涨到 0.9MB/s —— 换来的是【Basic 档录的
// 文件也能查关节角问题】，而 Basic 恰恰是"随手录一段发过来"的人会选的档。
enum class Detail : int {
    Basic = 0,    // 关节角全链路 + 骨架 + 2D/3D。约 0.9MB/s
    Full  = 1,    // 加位置六级/滤波/IK/追踪/链式续解等（18~22, 25, 27, 28, 30）。约 1.7MB/s
    Paranoid = 2, // 再加全量指派矩阵和网络输入张量（26, 29）。约 2.9MB/s
};

// 【新块统一带一个自描述头】前 4 字节 = {recVersion, recBytes}。
// 解析器拿 recBytes 跟自己的 dtype.itemsize 对一下，不等就【大声报错】而不是
// 静默错位。这不是洁癖：v3 的 Hm20DiagRec 加过一次字段，只要 C++ 和 pcrec.py
// 有一边忘了同步，读出来的就是一堆看起来合理、实际全是错位垃圾的数
// —— 而错位的数不会报错，只会把分析引到错误结论上，比读不出来危险得多。
struct RecHdr {
    uint16_t recVersion = 1;
    uint16_t recBytes = 0;
};

// 录制协议。
// 【这是整个设计里最重要的一个字段】没有真值就没有"最优参数"这回事。
// 协议的作用是让每一段数据自带一组【物理不变量】，当近似真值用：
//   StaticHold   手完全不动 -> 任何变化都是噪声，抖动即误差
//   RigidMotion  手型固定、整体平移旋转 -> 20点两两距离全恒定，190个刚体约束
//   FingerFlex   手掌不动、五指屈伸 -> 骨长恒定 + 手背刚体
//   Occlusion    故意遮挡 -> 测标签稳定性、鬼点率、重捕
//   FreeMotion   实际应用动作 -> 综合表现，但可用的不变量最少
//   RigidBodyGt  贴了已知球间距的标定杆 -> 真·真值，可标定绝对精度
//   FistOpenCycle 张开↔握拳循环，每段之间打标记 -> 【判"输出角对不对"的唯一
//                协议】。它自带一个不依赖任何解算的物理真值：握拳时指尖到腕心
//                的距离必然显著小于张开时。有了这个真值，"输出说张开而实际握拳"
//                才能被【证实】，而不是停留在"我看着不对"。
//   HandednessChk 手掌朝上/朝下/侧立各转一遍，手指做明显屈伸 -> 专门喂手性判据。
//                手摊平时判据接近 0 判不出来，必须有屈曲才有信息量。
enum class Protocol : int {
    StaticHold = 0, RigidMotion, FingerFlex, Occlusion, FreeMotion, RigidBodyGt,
    FistOpenCycle, HandednessChk, Unspecified
};

inline const char* protocolKey(Protocol p) {
    switch (p) {
        case Protocol::StaticHold:    return "static_hold";
        case Protocol::RigidMotion:   return "rigid_motion";
        case Protocol::FingerFlex:    return "finger_flex";
        case Protocol::Occlusion:     return "occlusion";
        case Protocol::FreeMotion:    return "free_motion";
        case Protocol::RigidBodyGt:   return "rigid_body_gt";
        case Protocol::FistOpenCycle: return "fist_open_cycle";
        case Protocol::HandednessChk: return "handedness_check";
        default:                      return "unspecified";
    }
}

inline const char* protocolHint(Protocol p) {
    switch (p) {
        case Protocol::StaticHold:
            return "把手架稳完全不动，15秒。任何变化都是噪声，用来量抖动底噪。";
        case Protocol::RigidMotion:
            return "手型固定住（张开或握拳都行，手指别动），整只手平移+转动，30秒。"
                   "这一段最有价值：20点两两距离全部恒定，等于190个约束当真值。";
        case Protocol::FingerFlex:
            return "手掌位置基本不动，五指反复屈伸/张合，30秒。用来标骨长和IK。";
        case Protocol::Occlusion:
            return "故意用另一只手或物体挡住部分反光球，并把手转到极端角度，30秒。";
        case Protocol::FreeMotion:
            return "做你实际应用里的动作，60秒。测综合表现。";
        case Protocol::RigidBodyGt:
            return "录一根贴了3~5颗球、球间距已知的刚体标定杆，30秒。这是真值。";
        case Protocol::FistOpenCycle:
            return "五指张到最开停3秒 -> 握到最紧停3秒，来回5轮，共约30秒。"
                   "【每次切换时按一下打标记】。手掌尽量正对相机组、别整体移动。"
                   "这段能判\"输出角对不对\"：指尖到腕心的距离是不依赖解算的物理真值，"
                   "握拳时必然明显变小，跟输出的关节角一对就知道是哪一级反了/平了。";
        case Protocol::HandednessChk:
            return "手掌朝下、朝上、侧立各保持5秒，每种姿态下都做2~3次明显的屈伸，共约30秒。"
                   "【手摊平时手性判据接近0、判不出来】，必须有屈曲才有信息量——"
                   "这正是\"标定时一切正常、一提交就左右反\"的成因。";
        default:
            return "";
    }
}

// ---------------------------------------------------------------------------
struct Blob2D {
    float x = 0, y = 0;                // 亚像素质心（像素）
    float cxx = 0, cxy = 0, cyy = 0;   // 观测协方差（质心法没有就填 0）
    // 【下面这四个是判"两球粘连"的全部依据，不存等于放弃这条线索】
    // 单球的连通域外接框近似正方形；两球粘连的花生形会在连心线方向明显
    // 拉长（长宽比 >1.3）。粘连是本系统最难查的失效模式之一——它不报错、
    // 不丢点，只是悄悄给出一个位于两球之间的错误质心，然后被当成正常观测
    // 三角化、关联、写进结果。离线要能复现这种情况，就必须有面积和外接框。
    float area = 0;                    // 过阈值像素数
    float peak = 0;                    // 峰值亮度（判过曝/欠曝）
    uint16_t bw = 0, bh = 0;           // 外接框宽高，<0 的老路径填 0
};

struct Point3DRec {
    float x = 0, y = 0, z = 0;  // mm，世界系
    int32_t id = -1;            // 追踪 id
    uint8_t coasting = 0;       // 本帧靠预测（没有观测支撑）
    uint8_t predicted = 0;
    uint8_t usedViews = 0;
    float residualMm = -1.0f;
};

// 多视角匹配的一个 track。【为什么必须单独记这一层】
// IekfPointTracker 输出的 TrackedPoint 只有 position/id/missedFrames，把
// "这个点是被哪几台相机看到的、重投影残差多少"整个丢掉了。而这两样恰恰是
// 相机数缩放分析的全部依据：
//   · supportMask 让我可以【离线关掉某几台相机重跑】，直接量出 3->4->5 台
//     各自的精度曲线，而不是靠猜。
//   · residualPx 是这套机位当前的噪声底，所有"多少毫米算超标"的阈值都应该
//     表达成它的倍数，而不是写死的绝对值 —— 否则换机位就全废。
struct ClusterRec {
    float x = 0, y = 0, z = 0;
    float residualPx = -1.0f;
    uint32_t supportMask = 0;   // bit i = 第 i 台相机贡献了观测
    uint8_t nSupport = 0;
    uint8_t verified = 0;
    uint8_t _pad[2]{};
};

struct SkeletonRec {
    uint8_t valid = 0;
    uint8_t wristPoseValid = 0;
    uint8_t pentagonOk = 0;
    uint8_t numGhost = 0;
    float dorsumRmseMm = -1.0f;
    float pos[60]{};            // 20 点
    uint8_t observed[20]{};
    float conf[20]{};
    uint8_t segSource[16]{};
    float segQuat[64]{};
    float wristR[9]{};
    float wristT[3]{};
};

// hm20 的逐帧诊断（块类型 9）。
//
// 【为什么不直接扩 SkeletonRec】那个结构已经在 v2 文件里固化了，改字段会让
// 所有已录的文件读不出来。新开一个块，旧文件没有它而已，旧解析器遇到未知
// 块类型也会按长度正确跳过。
//
// 【SkeletonRec 记的是"结果是什么"，这里记的是"结果是怎么来的"】
// 手背标签是几何直接解出来的、还是裕度不够靠位姿连续性仲裁的；拇指走的是
// 几何/IK/网络/纯预测哪一档；腕部位姿是本帧解出来的还是沿用上一帧。
// 没有这一层，事后看到一段抖动只能猜，而猜出来的结论没法验证。
//
// 【手性那一组是重点】面板勾的、自标定采用的、自标定检测到的、模型判的，
// 四个来源分开记。"是不是哪里反了"这类问题只有把四个摆在一起才能一眼看出
// 是谁跟谁不一致 —— 合并成一个 bool 就永远查不出来了。
struct Hm20DiagRec {
    int64_t wallNs = 0;          // 墙钟。骨架结果是异步回来的，只能打墙钟
    int64_t frameTsNs = -1;      // 对应的相机帧时间戳，用来跟点云对齐

    // ---- 手背刚体 ----
    uint8_t dorsumReason = 1;    // 0=ok 1=候选不足 2=无可行解 3=歧义拒解 4=残差超限
    uint8_t dorsumInliers = 0;
    uint8_t dorsumByHistory = 0;
    uint8_t wristPoseValid = 0;
    uint8_t pentagonOk = 0;
    uint8_t numGhost = 0;
    uint8_t observedJoints = 0;
    uint8_t candidateCount = 0;
    // 【这是分辨"标签翻了"和"点丢了"的唯一依据】
    //   标签翻了：同一个物理点这帧叫 0、下帧叫 4  -> sourcePointId 不变、位置不变
    //   点丢/跳了：物理点本身换了              -> sourcePointId 变
    // 只有 observed[20] 的话，两种情况在文件里长得一模一样，
    // 于是"手背标签不稳定"这个问题在录制数据上根本无法被证实或证伪。
    int32_t sourcePointId[20]{};
    float   dorsumRmseMm = -1.f;
    float   dorsumMarginMm = -1.f;
    // 手背模板"最优错解"的残差。不随帧变，但每帧都记：这样任意截一段来分析
    // 都自带这个数，不用回头翻文件头。<3mm = 布局病态，标签必然会翻。
    float   dorsumSelfAmbMm = -1.f;

    // ---- 拇指 ----
    uint8_t thumbFixSkip = 0;
    uint8_t thumbFixed = 0;
    uint8_t netThumbSegs = 0;
    uint8_t dorsumRepaired = 0;      // 0=没修，1..5=修好的槽位+1
    uint8_t thumbTipOccluded = 0;
    float   dorsumRepairMoveMm = -1.f;
    // 拇指遮挡跟随：当帧的 IP 角、以及相对遮挡入口累计跟了多少。
    // drift 恒 0 = 跟随没生效。这是事后判断"改动有没有落地"的唯一依据。
    float   thumbIpPredDeg = -999.f;
    float   thumbIpDriftDeg = 0.f;
    // 束调整累计次数 / 最近一次耗时。用来直接判定"退避有没有生效"：
    // bundleRuns 每秒涨一次就是没生效。
    int32_t bundleRuns = 0;
    float   bundleLastMs = -1.f;
    uint8_t thumbPronationFitted = 0;
    float   thumbPronation0 = 0.f;
    float   thumbAxialK = 0.f;
    float   thumbPronationContrast = 0.f;
    float   thumbCoverage = 0.f;

    // ---- 手性：四个来源分开记 ----
    uint8_t handPanelIsRight = 0;    // 面板勾选
    uint8_t handAutoIsRight = 0;     // 自标定【采用】的
    uint8_t handAutoDetected = 0;    // 自标定【检测】到的（可能没被采用）
    uint8_t handAiIsRight = 0;       // 模型 hand_logit 判的
    uint8_t handednessKnown = 0;
    uint8_t handednessConflict = 0;
    uint8_t mirrorSuspect = 0;
    uint8_t jointMirrorActive = 0;
    float   handSignMm = 0.f;        // 判据强度，<4mm 表示手摊平、判不了

    // ---- IK / 遮挡 ----
    uint8_t ikActive = 0;
    uint8_t ikFingerCount = 0;
    uint8_t occludedHeld = 0;
    uint8_t chainContinued = 0;
    uint8_t ikFilled = 0;
    uint8_t ikFallback = 0;
    uint8_t fingerIkValid[5]{};
    uint8_t fingerObsCount[5]{};
    float   fingerIkRmseMm[5]{};

    // ---- 分段 ----
    uint8_t segSource[16]{};     // 0=None 1=Predicted 2=Geometry 3=Ik 4=Net
    float   segConf[16]{};

    // ---- 自标定 ----
    uint8_t autoStage = 0;
    uint8_t autoTemplateReady = 0;
    uint8_t autoIkUsable = 0;
    uint8_t autoDorsumReordered = 0;
    float   autoProgress = 0.f;
    float   autoBundleRmseMm = -1.f;
    float   anchorRigidStd[5]{};

    // ---- 网络段常数 / AI 头 ----
    uint8_t netSegSolved = 0;
    uint8_t netSegOk = 0;
    uint8_t hasAiPose = 0;
    uint8_t hasAiHand = 0;
    float   netSegCSpread = -1.f;
    float   netSegKOffset[3]{};
    float   aiPoseConf[5]{};
    float   latencyMs = -1.f;
};

// ===========================================================================
// 下面三个块是为了【离线能跑出跟在线一模一样的结果】而加的。
//
// 这是整个录制格式最重要的设计目标，比"字段多"重要得多：
// 离线复算跟在线对不上的话，任何调参结论都不可信 —— 你不知道差异是参数带来
// 的还是复现本身错了。所以每一级的【输入】和【输出】都要能对拍。
//
// 原来的格式缺了两处，恰好是最要命的两处：
//   ① 送进 hm20 的候选点列表【和它的顺序】没记。
//      Points3D 记的是追踪器的输出，但送进网络之前还有一次筛选和截断
//      (maxCandidates)，而且网络的指派对【输入顺序】是敏感的。
//      顺序不一样 -> 指派可能不一样 -> 离线永远对不上在线。
//   ② 网络的原始输出没记。
//      这是"模型判错了"和"后处理搞砸了"的唯一分界线。没有它，看到一个错误的
//      拇指姿态，你分不清是 pos 头本来就给歪了、还是 correctPredictedThumb
//      把链拧断了 —— 而这两件事的修法完全相反。
// ===========================================================================

// 送进 hm20 的候选点，【按实际送入顺序】。
// pointId 对应 Points3D 里的追踪 id，两边可以连起来。
struct AssocInputRec {
    int64_t tsNs = 0;
    uint16_t n = 0;              // 实际送入个数（已经过筛选和 maxCandidates 截断）
    uint16_t nBeforeCap = 0;     // 截断前有多少。两者不等说明有点被丢掉了
    uint8_t  _pad[4]{};
    // 后面紧跟 n 组 {float x,y,z; int32 pointId}
};

// 网络的原始输出。【不记全量 log_assign】(N+1)x21 float 在 120fps 下是 200KB/s，
// 一分钟 12MB，而 99% 的分析只需要 top-3。需要全量时打开 fullAssign 单独录。
struct NetTopK {
    int16_t label[3]{-1, -1, -1};  // 0..19 = marker, 20 = ghost/dustbin
    float   prob[3]{};             // 已过 softmax
};

struct NetRawRec {
    int64_t tsNs = 0;
    uint16_t nCand = 0;
    uint8_t  hasPose = 0;          // v6 模型没有姿态头，下面几组是 0
    uint8_t  hasSegR = 0;
    float   pos[60]{};             // pos 头的 20 点预测（世界系，已反归一化）
    float   center[3]{};           // 网络内部用的归一化中心/尺度。
    float   scale = 1.f;           // 【必须记】离线反归一化对不上就全错位
    float   missLogit[20]{};       // 每个 marker "本帧看不见"的 logit
    float   jointAng[20]{};        // rig 原始约定的 20 维关节角
    float   handLogit = 0.f;
    float   poseConf[5]{};
    float   segRot6d[96]{};        // 16 段 x 6，【手掌规范系】
    // 后面紧跟 nCand 个 NetTopK
};

// 相机运行参数。曝光/增益/阈值直接决定光斑质量，而光斑质量决定了后面一切。
// 不记的话，两次录制精度不同你分不清是参数变了还是相机设置变了。
struct CamSettingsRec {
    uint32_t camId = 0;
    int32_t  width = 0, height = 0;
    float    exposureUs = -1.f;
    float    gain = -1.f;
    float    fps = -1.f;
    int32_t  threshold = -1;       // 二值化阈值
    int32_t  minArea = -1, maxArea = -1;
    uint8_t  roiEnabled = 0;
    uint8_t  _pad[3]{};
    int32_t  roiX = 0, roiY = 0, roiW = 0, roiH = 0;
};

// ===========================================================================
// v4 新增块。设计原则跟上面一样：记的不是"结果是什么"，而是"结果是怎么来的"，
// 而且每一级的【输入】和【输出】分开记 —— 只有这样才能定位是哪一级出的错。
// ===========================================================================

// ---- 块 13：关节角输出全链路 -----------------------------------------------
//
// 【为什么必须四级分开记】UDP 上发出去的 16 个数，中间经过四级：
//     solveJointAngles  -> qSolve   （从分段四元数算出来的原始角）
//     PredictedAngleSmoother -> qSmooth（只平滑预测段，含恢复补偿）
//     RomMapper         -> qRom     （按标定行程归一化再映射到目标行程）
//     RateLimiter       -> qOut     （速度限幅，实际进 UDP 的）
// "握拳时输出看着像张开"这一个症状，四级里【每一级】都能单独造成：
//     qSolve 就不对        -> 分段四元数/手性/标签的问题
//     qSolve 对、qSmooth 平 -> 平滑器的恢复补偿吃掉了信号
//     qSmooth 对、qRom 平   -> ROM 没标定好，行程被压成一条线（最常见）
//     qRom 对、qOut 平      -> 速度限幅太狠，跟不上手
// 只录最后一个数的话，这四种分不开，而它们的修法完全不同。
//
// 【romLo/romHi 必须逐帧带】ROM 是映射的分母。分母不对，分子再准输出也是平的。
// 它一次会话里几乎不变，但每帧都记：这样任意截一段来分析都自带这个数。
// 实测最常见的失效就是某一维 hi-lo 接近 0 —— 那一维输出会恒等于行程端点，
// 表现正好是"手指怎么动都不动"或"永远张开"。
struct JointOutRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    float qSolve[16]{};        // ① 解算原始角（弧度）
    float qSmooth[16]{};       // ② 预测段平滑后
    float qRom[16]{};          // ③ ROM 映射后
    float qOut[16]{};          // ④ 速度限幅后 = 实际进 UDP 的那 16 个数
    float romLo[16]{};         // ROM 标定出来的下界（未标定时为 0）
    float romHi[16]{};

    // 分段四元数。【世界系和相对父节点都记】：世界系是 UDP segmentQuats 发的，
    // 相对父节点才是"这个关节自己转了多少"—— 判"握拳时手指有没有弯"要看后者，
    // 因为世界系四元数里整只手的朝向和手指的屈曲是混在一起的。
    float segQuatWorld[64]{};  // 16 段 × (w,x,y,z)
    float segQuatLocal[64]{};
    float wristQuat[4]{};
    float wristT[3]{};

    float romCoverage = -1.f;     // 屈曲维行程覆盖度 0..1，<0.5 = ROM 没标好
    float romAbdCoverage = -1.f;
    float dtSec = -1.f;
    float rateLimitRadPerSec = -1.f;
    float angSmoothAlpha = -1.f;
    // 本帧被速度限幅削掉最多的那一维，削了多少弧度。恒大于 0 = 限幅在持续
    // 削信号，那正是"跟不上手"。恒等于 0 = 限幅没起作用（可以排除它）。
    float maxRateClipRad = 0.f;
    // 本帧 |qOut - qSolve| 的最大值。这一个数就能回答"输出跟解算差多少"，
    // 不用先把四组 16 维排开看。
    float maxStageDeltaRad = 0.f;

    int32_t romSamples = 0;
    int32_t maxRateClipIdx = -1;
    int32_t maxStageDeltaIdx = -1;

    uint8_t segSource[16]{};      // 同 SegSource：0=None 1=Predicted 2=Geometry 3=Ik 4=Net
    uint8_t fingerValid[5]{};
    uint8_t fingerPredicted[5]{};
    uint8_t romReady = 0;
    uint8_t romLearning = 0;
    uint8_t rateLimitOn = 0;
    uint8_t angSmoothOn = 0;
    uint8_t mcpValid = 0;
    uint8_t wristValid = 0;
    uint8_t jointMirror = 0;      // 关节角输出被按手性镜像了
    uint8_t wristStale = 0;       // 腕部位姿已经沿用了多少帧（0=本帧新解的）
    uint8_t quatOutOn = 0;
    uint8_t filterActive = 0;
    uint8_t lowLatency = 0;       // 直通模式：平滑/限流/限幅全旁路
    uint8_t _pad1[5]{};
};

// ---- 块 14：手性证据链 -----------------------------------------------------
//
// 【为什么要把这么多字段单独开一个块】手性判错是本系统最难查的一类问题：
// 它不报错、不丢点，只是把整只手镜像掉，而【所有 RMSE 类的自检都查不出来】
// （手背近平面 + 五点近对称，反射≈绕面内轴转 180°，Kabsch 残差照样很小）。
// 唯一的查法是把"每一个环节各自认为的手性"摆在一起，看是哪两个开始不一致。
//
// 环节一共有六处，v3 只录了其中四处，而漏掉的两处恰好是最要命的：
//   · tmplIsRight —— 【送进网络和建模板用的那个】。它可能既不等于面板值、
//     也不等于自标定推断值（handDirty_/applyHandedness 两条路都能改它）。
//     它错了，网络的输入就是镜像的，后面全错，但前四个 bool 可以全部正常。
//   · ikIsRight —— IK 用的。它跟 tmplIsRight 不一致时会出现
//     "IK 按左手解、拇指旋前按右手补"，症状是拇指单独歪，其余正常。
//
// 【判据本身的值也要记，不能只记结论】autoIsRightDetected 是个 bool，
// 而它是由 handSignMm/handSignLatMm 跟 4mm 阈值比出来的。判据值 4.1mm 和
// 40mm 得到的是同一个 bool，可信度天差地别。只看 bool 会把"勉强过线的猜测"
// 当成"确定的结论"—— 实测这正是误判发生的地方。
struct HandednessRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    // —— 判据的原始值（不是结论）——
    float handSignMm = 0.f;       // 拇指侧判据：手指点相对手背平面的平均有符号距离
    float handSignLatMm = 0.f;    // 横向判据：食指相对小指在手背横轴上的投影
    float handednessMinMm = 0.f;  // 阈值（默认 4.0）。判据值离它多远 = 可信度
    float aiHandLogit = 0.f;      // 模型原始 logit。实测中位只有 0.044，
                                  // 而纯随机点云能到 0.196 —— 在这个量级上符号是噪声
    float aiHandConf = 0.f;       // 0~1 累积一致度
    float thumbRollSignedRad = 0.f;  // 拇指 roll 实际下发值（已按手性带符号）
    int32_t handSignN = 0;        // 判据的样本数。太少的话均值没有意义
    int32_t geoHandSign = 0;      // 关联器几何锁定值：0=未锁 +1=右 -1=左

    // —— 六处环节各自的手性 ——
    uint8_t panelIsRight = 0;        // ① 用户在面板上勾的（显式声明，最可信）
    uint8_t autoIsRightDetected = 0; // ② 自标定【推断】的
    uint8_t autoIsRight = 0;         // ③ 自标定【采用】的
    uint8_t aiIsRight = 0;           // ④ 模型判的（aiKnown=0 时【无意义】）
    uint8_t tmplIsRight = 0;         // ⑤ 【送进网络/建模板用的】
    uint8_t ikIsRight = 0;           // ⑥ IK 用的

    // —— 各判据的"敢不敢下结论" ——
    uint8_t autoKnown = 0;         // 自标定：两判据一致且过阈值
    uint8_t autoAgree = 0;         // 拇指判据与横向判据是否一致（不一致=拇指多半歪了）
    uint8_t autoConflict = 0;      // 推断与面板设置不符
    uint8_t autoMirrorSuspect = 0; // 提交后自检发现模板疑似被镜像
    uint8_t aiHas = 0;
    uint8_t aiKnown = 0;           // 【三态的关键位】为 0 时 aiIsRight 是没有意义的
    uint8_t aiLocked = 0;          // 已连续 30 帧一致

    // —— 配置：决定推断会不会真的改东西 ——
    uint8_t cfgDetectHandedness = 0;
    uint8_t cfgApplyHandedness = 0;   // 【重点】为 1 时一次误判就会把整个手背模板镜像掉
    uint8_t cfgJointMirrorAuto = 0;
    uint8_t jointMirrorActive = 0;    // 关节角输出这一帧真的被镜像了
    uint8_t autoCalibOn = 0;
    uint8_t autoStage = 0;
    uint8_t _pad1[3]{};
};

// ---- 块 15：逐 marker 的来源与位置 ----------------------------------------
//
// 【v3 只有一个 observed bool，不够】一个补出来的点，可能是 IK 解的、链式
// 续解的、网络 pos 头兜底的、或者干脆保持上一帧 —— 四者的误差特性差一个
// 数量级（实测 IK 骨轴 max 71°，网络预测 max 108°，保持则是无限滞后）。
// 只有一个 bool 的话，"这一帧这个点靠不靠谱"答不上来。
//
// 【posNet 和 posFinal 都要记】两者之差 = 后处理搬了这个点多远。
// 这是分开"模型给歪了"和"后处理拧错了"的直接依据：
//     posNet 就歪、posFinal 跟着歪  -> 模型/标签问题
//     posNet 正常、posFinal 歪      -> 后处理（骨长回正/平滑/发散限速）问题
// 这两件事的修法完全相反，而只看最终位置永远分不出来。
struct MarkerDbgRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    float posFinal[60]{};      // 最终位置（世界系 mm）
    float posNet[60]{};        // 网络 pos 头的原始预测（未经任何后处理）
    float conf[20]{};          // 指派概率
    float missLogit[20]{};     // 模型认为"这点本帧看不见"的 logit
    float netDeltaMm[20]{};    // |posFinal - posNet|，后处理把这个点搬了多远
    int32_t sourcePointId[20]{};

    // 0=未知 1=实测 2=IK补 3=链式续解 4=网络预测兜底 5=保持上一帧/发散限速
    uint8_t source[20]{};
    // bit0=骨长回正过 bit1=预测点平滑过 bit2=发散限速过 bit3=拇指回正过
    uint8_t flags[20]{};
    uint8_t observed[20]{};
    uint8_t hasNet = 0;        // 0 = 这一帧没抓到网络原始流（没开 captureDebugStreams）
    uint8_t _pad1[3]{};
};

// ---- 块 16：逐帧生效的功能开关 --------------------------------------------
//
// 【为什么不能只靠文件头的参数快照】文件头是按下录制按钮那一刻拼的，而这些
// 开关在录制中途会变：用户会点，自标定会改，dirty 标志会延迟一帧生效。
// "某个功能到底有没有生效"这个问题，只有逐帧记才答得准 —— 而它恰恰是
// 排查时问得最多的一个问题（"我明明打开了 IK，为什么没反应"）。
struct RunFlagsRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;
    uint32_t bits = 0;          // 见下面 RunFlagBit
    int32_t  _pad1 = 0;
    float thumbRollUiDeg = 0.f;
    float thumbPronation0Rad = 0.f;
    float occludedJumpGateMm = -1.f;
    float latencyMs = -1.f;
};

// bits 的位定义。【新增只能往后加，不能插中间】—— 插中间会让已录的文件
// 全部按错误的语义解读，而且不会报错。
enum RunFlagBit : uint32_t {
    RF_BackendReady    = 1u << 0,
    RF_IkEnabled       = 1u << 1,
    RF_IkOnlyOccluded  = 1u << 2,
    RF_ChainContinue   = 1u << 3,
    RF_FilterOn        = 1u << 4,
    RF_RateLimitOn     = 1u << 5,
    RF_RomLearning     = 1u << 6,
    RF_RomReady        = 1u << 7,
    RF_QuatOutOn       = 1u << 8,
    RF_AutoCalibOn     = 1u << 9,
    RF_DorsumRigid     = 1u << 10,
    RF_GeoRelabel      = 1u << 11,
    RF_NetSegCalib     = 1u << 12,
    RF_ThumbSegFromNet = 1u << 13,
    RF_ThumbPronationOn= 1u << 14,
    RF_CaptureDebug    = 1u << 15,
    RF_JointMirrorAuto = 1u << 16,
    RF_JointMirrorNow  = 1u << 17,
    RF_LowLatency      = 1u << 18,
    RF_FastModel       = 1u << 19,
    RF_ShowSkeleton    = 1u << 20,
    RF_Recording       = 1u << 21,
};

// ===========================================================================
// v5 新增块。
//
// 【贯穿这一版的一条原则】任何"就地覆写"的处理，前后两份都要留。
// 系统里现在有三处就地覆写：角度平滑（v4 已经拆开了）、位置滤波、
// 以及关联器内部那一串后处理。后两处的输入在覆写之后就不存在第二份了 ——
// 于是"输入本来就错"和"输入没错、被这一级改坏了"在文件里长得一模一样，
// 而这两件事的修法是相反的。这不是多录一点数据的问题，是这类问题到底
// 可不可查的问题。
// ===========================================================================

// ---- 块 18：20 点位置的六级流水 -------------------------------------------
//
// 【为什么位置也要分级，而且比角度更该分】v4 把关节角拆成了四级，因为
// "握拳输出像张开"每一级都能单独造成。位置这一路完全同理，而且它在角度
// 【上游】—— 位置错了角度必然跟着错，所以查的时候应该先看位置。
// 但 v3/v4 位置只有两级：网络原始(posNet) 和 全部处理完(posFinal)。
// 中间四级都是就地覆写，一步都留不下来。
//
// 六级的物理含义（跟 HandSkeletonAssociator::process 的步骤号对应）：
//   0 Net    网络 pos 头的原始预测（步骤 1）
//   1 Assoc  匈牙利指派 + 手背几何重定之后（步骤 3）。观测到的点是真实测量，
//            没观测到的还是网络值 —— 【这一级和上一级的差 = 指派改了什么】
//   2 Geom   拇指回正 + 骨轴/分段朝向之后（步骤 5）
//   3 Ik     IK 精修补点之后（步骤 6）
//   4 Post   预测点平滑 + 骨长回正 + 发散限速之后（关联器的最终输出）
//   5 Final  时序滤波之后（真正发出去的）
//
// 【stageMoveMm 是给人看的第一眼】每一级相对上一级把这个点搬了多远。
// 一眼扫过去哪一级的数突然变大，问题就在那一级 —— 不用先把六组 60 维排开。
struct MarkerStageRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    float pos[6][60]{};          // 六级 × 20 点 × xyz（世界系 mm）
    float stageMoveMm[6][20]{};  // 相对上一级的位移。第 0 级恒为 0
    // 本级有没有真的被填过。0 = 那一级没跑（比如 IK 没开），这时 pos 是
    // 上一级的拷贝而不是"没动"—— 两者含义完全不同，必须能分开。
    uint8_t stageValid[6]{};
    uint8_t _pad1[2]{};
};

// ---- 块 19：滤波前后 + 滤波器内部 ------------------------------------------
//
// 【这一块是 v5 里最该有的一块】Hm20PoseFilter::apply(result) 是【就地覆写】
// SkeletonFrameResult 的：滤波前的位置在这个系统里没有第二份。于是
//     "几何本来就把手指解错位置了"
//     "几何是对的，被 One-Euro 的滞后/软死区吃平了"
// 这两件事在录制文件里完全无法区分，而前者要去查关联和骨轴，后者要去调
// 截止频率 —— 方向相反。
//
// 【内部状态也要记，不能只记前后】One-Euro 的截止频率是【逐点逐帧自适应】的
// （随速度变），软死区和硬重置是条件触发的。只看输入输出的话，一个点没动
// 有三种原因：它真没动、落在死区里、或者速度估计塌了导致截止频率被压到最低。
// 三者的修法各不相同，而只有把 cutoffHz / deadzone / velLocal 摆出来才分得开。
struct FilterDbgRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    float posIn[60]{};        // 滤波【前】（= 关联器输出，MarkerStage 的第 4 级）
    float posOut[60]{};       // 滤波【后】（= MarkerStage 的第 5 级）
    float moveMm[20]{};       // |out-in|，滤波把这个点搬了多远
    // 【腕部系速度，不是世界系】世界系速度里混着整只手的刚体运动，
    // 拿它判断"这根手指自己在不在动"会被手的平移完全带偏。
    float velLocalMm[20]{};
    float staleFrames[20]{};  // 已经连续多少帧没有真观测
    float fuseWeight[20]{};   // 趋势外推 vs 网络预测的混合权重（0=全信网络）
    float cutoffHz[20]{};     // 本帧该点实际用的截止频率（自适应结果）

    uint8_t hardReset[20]{};  // 触发了硬重置（跳变太大，滤波器被清空重起）
    uint8_t deadzone[20]{};   // 落在软死区内，输出被钉住没动
    uint8_t fused[20]{};      // 这一帧走了 predictFuse 外推
    uint8_t observed[20]{};   // 冗余记一份：内部状态要跟观测位对着看才有意义

    float wristQuatIn[4]{};
    float wristQuatOut[4]{};
    float wristAngMoveDeg = 0.f;
    float segQuatMoveDeg[16]{};  // 每段四元数被滤波转了多少度
    float dtSec = -1.f;
    float posMinCutoffHz = -1.f;
    float posBeta = -1.f;
    float predictedSmooth = -1.f;

    uint8_t enabled = 0;
    uint8_t filterPositions = 0;
    uint8_t filterRotations = 0;
    uint8_t predictFuseOn = 0;
    uint8_t _pad1[4]{};
};

// ---- 块 20：IK 内部 --------------------------------------------------------
//
// 【为什么单独开一块】IK 现在对外只有 fingerIkValid + fingerIkRmseMm 两个数。
// 而 refine() 内部做的事情远不止这些，其中两件直接关系到本系统最难查的两类
// 问题：
//   · 它按【自己的 isRight_】把左手镜像成右手系再解。这个 isRight_ 跟
//     tmpl_.isRight 是两条独立的下发路径（setHandedness vs handDirty_），
//     不一致时症状是"拇指单独歪、其余正常" —— 而这是手性出错的一条独立路径。
//     v4 已经把 ikIsRight 这个 bool 录了，但解出来的角一个都没有，
//     所以只知道"用错了手性"，不知道"用错之后解成了什么样"。
//   · 逐指 4 自由度的解本身。它是 marker 位置的直接来源（FK 摆出来的），
//     位置不对时要往上查一级，查的就是这四个角有没有撞限位、有没有收敛。
//
// 【limitHit 必须逐维记】撞限位的那一维会被罚函数拉住不动，表现正好是
// "这根手指弯到一半就不动了"。而残差可能仍然很小（其余维补偿了），
// 所以光看 rmse 是发现不了的。
struct IkDbgRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    // 逐指 4 自由度：[0]=MCP屈 [1]=MCP展 [2]=PIP [3]=DIP（拇指为 CMC屈/展/MCP/IP）
    float q[5][4]{};          // 本帧解
    float qPrior[5][4]{};     // 上一帧解（时序先验的起点）
    float limitLo[5][4]{};    // 实际生效的限位
    float limitHi[5][4]{};
    float cost[5]{};          // 最终代价
    float rmseMm[5]{};        // marker 拟合残差
    float anchorMm[5][3]{};   // 本帧实际用的掌指关节 anchor（腕部系）
    float boneLenMm[5][3]{};  // 本帧实际用的三节骨长

    float handLenMm = -1.f;
    float thumbAxialK = -1.f;
    float thumbPronation0 = 0.f;
    float wPrior = -1.f;
    float wLimit = -1.f;
    float wCouple = -1.f;
    float dipCoupling = -1.f;
    // 【遮挡兜底】逐指学到的 MCP->PIP 耦合：PIP ≈ k*MCP + b。
    // 【为什么斜率和截距都要记】第一版用的是过原点回归（只有 k），实测
    // 学出来的符号是反的——因为 q[0]=0 不对应"手指伸直"这个解剖学零点。
    // 没有 b 就没法表达"基准角非零"这件事，符号必然被带偏。
    // 【为什么 ready 位不能省】k=0 有两种含义：真学出 0，或者压根没生效
    // （样本不够/mcp范围太窄/拇指）。只看 k 分不开，而这两者对
    // "遮挡时 PIP 为什么还是不动"的解释是相反的。
    float mcpPipCoupling[5]{};        // 斜率 k
    float mcpPipCouplingB[5]{};       // 截距 b（弧度）
    int32_t mcpPipCouplingSamples[5]{};
    uint8_t mcpCouplingReady[5]{};
    uint8_t _pad2[3]{};

    int32_t iters[5]{};       // 实际迭代次数
    int32_t nObs[5]{};        // 该指本帧被认领的 marker 数

    uint8_t limitHit[5][4]{}; // 该维顶在限位上
    uint8_t fingerValid[5]{};
    uint8_t fingerSolved[5]{};// 真的跑了求解（区别于"跳过、沿用先验"）
    uint8_t ikIsRight = 0;    // refine() 内部用的手性
    uint8_t paramsReady = 0;  // 五指参数都设过了才允许 refine
    uint8_t applied = 0;      // refine() 返回 true
    uint8_t _pad1 = 0;
};

// ---- 块 30：链式续解逐指内部状态 -------------------------------------------
//
// 【为什么单开一块】实测遮挡点的填充来源分布是：链式续解 81.1%、
// 网络原始兜底 17.0%、IK 补 1.9%。也就是说遮挡点的位置几乎全由链式续解
// 决定，而这一整条路径此前【没有任何内部量】被记录过——之前几轮一直在
// 改 IK，而 IK 只碰了 1.9% 的点，所以怎么改都看不到效果。
//
// 【最关键的两个量】
//   · caseB 的四个守卫（hasPipHold/hasPlane/pmLen/wChain>0）——实测有 87 帧
//     外部条件跟正常帧完全一样（近节可见、中节不可见），却回落到了网络
//     原始预测，侧偏 61.2° 一点没被纠正。区别只能在这四个里。
//   · wChain 淡出权重——它 <1 时按 (1-wChain) 把【网络原始预测】混回来，
//     而网络预测正是侧弯的源头。wChain 掉多少，就等于混回多少侧弯。
//     这是解释侧偏最直接的量。
struct ChainDbgRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    float coupledRad[5]{};    // 掌心相对耦合推出去多少（相对遮挡入口角）
    float pmLenMm[5]{};       // 学到的近节-中节球间距，<=0 = 还没学到
    float mdLenMm[5]{};       // 中节-远节球间距
    float weight[5]{};        // wChain。-1 = 本帧这一指没走到那段代码
    float aging[5]{};         // 老化量，wChain = clamp(1-aging)

    int32_t caseBFrames[5]{}; // 已连续多少帧处在情形B

    uint8_t hasPipHold[5]{};  // 守卫1
    uint8_t hasPlane[5]{};    // 守卫2
    uint8_t caseB[5]{};       // 四个守卫全过、真的走了情形B
    uint8_t useAnchor[5]{};   // 用 anchor 骨轴（准）还是沿用上一帧（会漂）
    uint8_t hasExc[5]{};      // 有直接 staleness 测量，否则退回帧数计时
    uint8_t chainContinued = 0;  // 本帧总共补出几个点

    // ---- anchor 下发/自检链路（v6.1）----
    // 【为什么这几个是关键】learnJointPose() 开头就是 if(!anchorsValid_) continue,
    // anchorsValid_ 为假 => 平面永不学 => hasPlane 恒假 => 链式续解永不启用
    // => 遮挡点全部回落到网络原始预测 => 侧弯原样输出。实测正是这条链。
    // verifyAnchors 是【一次性锁】(anchorsChecked_)，判过一次永不重试，
    // 所以判据当时的现场必须留下来，否则事后无从追。
    uint8_t anchorsSet = 0;
    uint8_t anchorsChecked = 0;
    uint8_t anchorsValid = 0;
    uint8_t tmplMmValid = 0;
    int32_t anchorsVerifyNSeen = -1;  // 自检时看到几根近节球（需>=3）
    int32_t anchorsVerifyNOk = -1;    // 其中几根通过（需>=nSeen-1）

    // ---- 逐指 anchor / 弯曲平面（v6.2）----
    // 【为什么上面那六个不够】它们全是【全局】量：anchorsValid 是一个 bool，
    // 说不出"五根里哪几根降级了"。而实测五根 anchor 的质量差得很远
    // （|anchor->pp| 24.8~73.7mm，出平面 17.8~59.7°），一个全局 bool 只能
    // 一起用或一起不用 —— 那正是要修的东西，也就必须能逐指看到结果。
    // 【padding 全部显式写出来】这个结构体是直接 fwrite 到文件里的，
    // 编译器补的隐式对齐洞在 C++ 侧看不见，但在 numpy dtype 里必须逐字节对上。
    // 写出来是为了让 pcrec.py 那边照抄即可，不用去猜编译器补了几个字节。
    uint8_t anchorOkPerFinger[5]{};   // 0 = 这根已退回 pp->mp
    uint8_t _padA[3]{};               // 189 -> 192，给下面的 int32 对齐
    // 弯曲平面的累加器吃进了多少帧。平面【只】从 pp/mp/dp 三点全见的帧学，
    // 所以它同时也是"这根手指有多少帧三点全见"的直接计量。
    // 【排查时先看这个数】个位数就说明平面基本没学到，此时 hasPlane 为真
    // 也不能信 —— 那是一两帧噪声撑起来的。
    int32_t planeSampleN[5]{};
    // 本帧实测的 anchor->pp 出平面角(度)。整条诊断链的核心量：
    // cross(u0,u1) 对 u0 的出平面分量有 1/sin(MCP角) 的放大，
    // 实测出平面 17.8~59.7° -> 学到的平面歪 18.8~77.9°。
    // -1 = 本帧量不出来（anchor 没在用，或三点不全）。
    float   anchorOutOfPlaneDeg[5]{};
    uint8_t _pad1[8]{};               // 232 -> 240，结构体按 int64 对齐到 8 的倍数
};

// ---- 块 21：模板快照 -------------------------------------------------------
//
// 【为什么必须逐次落盘，而不是只放文件头】文件头那一份是按下录制按钮那一刻
// 的模板。而自标定跑起来之后会【热替换】worker 里的 tmpl_（applyAutoCalib
// 置 tmplDirty_，下一帧生效），模型的条件输入就跟着换了。
// 于是"第 N 帧真正送进网络的模板长什么样"这个问题 —— 手性问题的核心证据 ——
// 在 v4 的文件里是查不到的。
//
// 【带指纹】fnv1a over 全部 float。模板有没有变、什么时候变的，比对一个
// 32 位数就够了，不用逐点比。指纹变了而 isRight 没变，说明是标定在微调；
// 指纹变了且手背点整体镜像，那就是 applyHandedness 把模板翻了 —— 这正是
// "标定时一切正常、一提交就左右反"的成因。
//
// 【reason 说明这次快照是因为什么写的】0=录制开始 1=模板热替换 2=手性变更
// 3=自标定提交 4=周期性重录（1Hz，防止中间漏掉一次变更）
struct TemplateSnapRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    float backMarkersMm[5][3]{};    // 手背 5 点（模板系）
    float markersMm[20][3]{};       // 全 20 点模板
    float normalized[20][3]{};      // 送进网络的归一化版本（网络真正看到的）
    float anchorsMm[5][3]{};        // 五指掌指关节 anchor
    float boneLenMm[5][3]{};
    float dipCoupling[5]{};

    uint32_t fingerprint = 0;       // fnv1a，模板变没变一眼可见
    int32_t  reason = 0;
    float    bundleRmseMm = -1.f;   // 这份模板是哪次标定出来的、残差多少
    float    selfAmbiguityMm = -1.f;// 手背模板"最优错解"残差。<3mm = 布局病态

    uint8_t isRight = 0;
    uint8_t valid = 0;
    uint8_t anchorsValid = 0;
    uint8_t backCalibrated = 0;
    uint8_t fingerCalibrated[5]{};
    uint8_t fromAutoCalib = 0;      // 1 = 自标定推上来的，0 = 用户标定/文件加载
    uint8_t _pad1[2]{};
};

// ---- 块 22：时基质量与分阶段耗时 -------------------------------------------
//
// 【为什么这块不是"锦上添花"】dt 是滤波、限幅、速度外推三处的分母。
// dt 不可信时，这三处算出来的东西全部不可信，而它们的输出看起来完全正常 ——
// 只是慢了/飘了。查到这一步之前，人会先怀疑参数、怀疑标定、怀疑模型，
// 把时间全花在错的地方。
//
// 【camTsSpreadUs 是多相机系统的头号隐患】各相机不同步时，同一"帧"里的
// 光斑其实来自不同时刻，三角化出来的点在手快速运动时会系统性偏移 ——
// 而这个偏移随速度变化，看起来就像"动起来就不准"，跟标定误差长得很像。
struct TimingRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    // —— 时基 ——
    float camTsSpreadUs = -1.f;   // 本帧参与的相机，时间戳最大-最小
    float camTsStdUs = -1.f;
    float frameDtMs = -1.f;       // 距上一帧
    float frameDtJitterMs = -1.f; // dt 的滑动标准差
    float wallMinusCamMs = -1.f;  // 墙钟与相机时基之差（漂移监控）

    // —— 分阶段耗时 ——
    float tDetectMs = -1.f;
    float tClusterMs = -1.f;
    float tTrackMs = -1.f;
    float tInferMs = -1.f;        // 纯 ONNX
    float tAssocMs = -1.f;        // 关联器 process 全程
    float tIkMs = -1.f;
    float tFilterMs = -1.f;
    float tJointMs = -1.f;
    float tTotalMs = -1.f;

    // —— 丢帧与背压 ——
    // 【worker 忙时新帧直接跳过】这是设计如此，但跳了多少必须知道：
    // 跳帧会让实际输出帧率远低于相机帧率，而下游看到的时间戳仍然连续，
    // 表现是"动作有延迟且发飘"，跟滤波调过头一模一样。
    int32_t skelSkipped = 0;      // 骨架 worker 累计跳过的帧
    int32_t skelProcessed = 0;
    int32_t recQueueDepth = 0;    // 录制队列深度（接近上限就要丢块了）
    int32_t recDropped = 0;
    int32_t camFrameGaps = 0;     // 相机侧检测到的帧号不连续次数
    int32_t nCamsThisFrame = 0;

    float   fpsIn = -1.f;         // 输入帧率（相机）
    float   fpsOut = -1.f;        // 输出帧率（骨架）
};

// ---- 块 23：UDP 输出闭环 ---------------------------------------------------
//
// 【为什么要记输出】整条链路的终点是 UDP 包，而"下游看到的不对"有一类原因
// 完全在系统内部查不到：包根本没发出去、发了但对端没配、或者两个包
// （M3DS 关节角 / M3DQ 四元数）的时间戳对不上导致下游插值出鬼。
// 不记的话，这类问题会被一路误判成解算问题，往上游查一整圈。
//
// 【记摘要不记全量字节】包内容就是 qOut 和 quatWorld/Local，那两样已经在
// 块 13 里了，再存一遍纯属重复。这里只记"发生了什么"：发没发、多大、
// 序号、错误码，以及一个校验和 —— 校验和跟块 13 重算出来的对不上，
// 就说明打包这一步本身有问题，这是唯一能发现它的办法。
struct UdpOutRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;      // 包里带的时间戳

    uint32_t m3dsCrc = 0;        // 载荷 fnv1a
    uint32_t m3dqCrc = 0;
    int32_t  m3dsBytes = 0;      // 0 = 这一帧没发
    int32_t  m3dqBytes = 0;
    int64_t  m3dsSeq = 0;        // 累计发包数
    int64_t  m3dqSeq = 0;
    int32_t  m3dsErr = 0;        // socket 错误码，0=ok，<0=没发
    int32_t  m3dqErr = 0;
    uint32_t m3dqFlags = 0;      // 包里那个 flags 字段（腕/mcp/五指有效位）
    int32_t  targetPort = 0;
    uint8_t  enabled = 0;
    uint8_t  quatEnabled = 0;
    uint8_t  _pad1[2]{};
};

// ---- 块 24：事件流 ---------------------------------------------------------
//
// 【为什么标量不够，必须有事件】v4 逐帧记了很多"当前值"。但排查时问的往往是
// "它是【什么时候】、【因为什么】变成这样的"。从一串逐帧标量里反推变更点，
// 要么得写脚本做差分（而且只能看到变了、看不到为什么），要么就漏掉。
//
// 尤其是手性：applyHandedness 触发一次镜像是【一个瞬间事件】，之后所有帧
// 看起来都"稳定地不对"。没有事件流的话，那个瞬间在文件里没有任何标记，
// 只能靠人眼在几千帧里找拐点。
//
// code: 1=手性变更 2=模板热替换 3=模板被拒 4=自标定阶段变化 5=看门狗松锁
//       6=看门狗退模板 7=模型切换 8=参数下发 9=IK开关 10=滤波开关
//       11=ROM 标定完成 12=跟踪重置 13=后端初始化 14=用户操作 15=其它
//       ---- v7 ----
//       16=按下"ROM标定"（UI 线程，墙钟）   17=按下"结束ROM"（UI 线程，墙钟）
//       18=ROM 采样真正开始的那一帧          19=ROM 采样真正结束的那一帧
//       ---- v8.1 ----
//       20=面板控件被改动（任意勾选框/数值框/下拉框/按钮，带旧值→新值）
//       21=录制开始     22=录制停止
//
// 【为什么 20 要覆盖"任意"控件，而不是挑几个重要的挑着记】
// 面板上有 87 个控件，而 1Hz 的 runtimeConfig 只覆盖其中 26 个 ——
// 剩下 61 个的值【在文件里一个字节都没有】。挑着记的问题在于：
// 排查时真正想问的往往是"我中间是不是手贱动过什么"，而这个问题
// 恰恰只有在【全都记】的前提下才答得出来。漏掉的那一个，
// 按墨菲定律就是出问题的那一个。
//
// 而且逐帧的 RunFlags 只有 20 个布尔位、runtimeConfig 是 1Hz 采样：
//   · 数值控件调了又调回来（1 秒内）在 runtimeConfig 里【完全看不见】
//   · 就算看见了，也只知道"它变了"，不知道是【谁在什么墙钟时刻】动的，
//     更不知道旧值是多少 —— 而"改回去"这个动作要靠旧值才能复现
//
// 21/22 记录开始/停止：文件头有开始时间，但"是正常停的还是出错停的"、
// "停的时候队列里还压着多少"，以前一点痕迹都没有。
//
// 【16/17 和 18/19 为什么要分开记】按钮回调跑在 GUI 线程上，而
// romLearning_ 是原子标志，worker 线程要到下一帧才读到它。两者之间隔着一个
// 投递延迟（实测几毫秒到几十毫秒不等，取决于 worker 忙不忙）。
// 只记按钮的话，对着帧号找"标定是从哪一帧开始的"会差几帧；
// 只记帧的话，又对不上用户"我是什么时候按的"这个主观时间轴。
// 两个都记，中间那段差值本身还能反映 worker 的积压情况。
// 后面紧跟 textLen 字节的 UTF-8 原因文本。
struct EventRec {
    RecHdr   hdr;
    int32_t  _pad0 = 0;
    int64_t  wallNs = 0;
    int64_t  frameTsNs = -1;
    int32_t  code = 0;
    int32_t  severity = 0;     // 0=信息 1=注意 2=异常
    float    valueA = 0.f;     // 事件相关的数值（比如变更前后的判据值）
    float    valueB = 0.f;
    int32_t  intA = 0;
    int32_t  intB = 0;
    uint16_t textLen = 0;
    uint8_t  _pad1[6]{};
};

// ---- 块 25：自标定逐帧产物 -------------------------------------------------
//
// 【为什么 1Hz 的 stateJson 不够】自标定的判据是在动作里累积的，而关键的
// 那几秒（手性刚判出来、模板刚提交）往往就在两次 1Hz 快照之间。
// 逐帧记二进制标量，体积可以忽略，而"判据是怎么一步步涨到阈值以上的"
// 这条曲线才是判断"这次结论可不可信"的依据 —— 一个刚过线就锁定的结论，
// 和一个远远超过阈值的结论，bool 是一样的，可信度天差地别。
struct AutoCalibDbgRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    float handSignMm = 0.f;         // 拇指侧判据
    float handSignLatMm = 0.f;      // 横向判据
    float handSignThreshMm = 4.f;
    float progress = 0.f;
    float bundleRmseMm = -1.f;
    float anchorResidMm[5]{};
    float anchorSpreadDeg[5]{};
    float anchorRigidStd[5]{};
    float boneLenMm[5][3]{};        // 当前估计的骨长
    float boneLenStdMm[5][3]{};     // 它的离散度 —— 收敛没收敛看这个
    float dorsumTmplDriftMm = 0.f;
    float tmplRejectedRmse = -1.f;
    float tmplAppliedRmse = -1.f;

    int32_t stage = 0;
    int32_t samples = 0;
    int32_t anchorSamples[5]{};
    int32_t bundleRuns = 0;
    int32_t attempts = 0;
    int32_t rejectCode = 0;         // 本帧被拒的原因码，0=没拒
    int32_t handSignN = 0;

    uint8_t templateReady = 0;
    uint8_t ikUsable = 0;
    uint8_t handednessKnown = 0;
    uint8_t handednessAgree = 0;
    uint8_t isRightDetected = 0;
    uint8_t isRightApplied = 0;
    uint8_t mirrorSuspect = 0;
    uint8_t dorsumReordered = 0;
    uint8_t anchorFitted[5]{};
    uint8_t staticSeeded[5]{};
    uint8_t calibRejected = 0;
    uint8_t _pad1[5]{};
};

// ---- 块 27：3D 追踪器逐点内部状态 ------------------------------------------
//
// 【为什么 Point3DRec 不够】那个结构里 predicted/usedViews/residualMm 三个
// 字段【定义了但从来没被填过】——一直是 0。而追踪器里真正决定编号稳不稳的
// 量（速度、协方差、关联距离、马氏距离、命中数、是否刚新建）一个都不在里面。
// "编号乱跳"这个最常见的抱怨，靠 Point3DRec 是查不出来的：
// 它只能看到编号变了，看不到是门控没兜住、还是协方差塌了、还是关联抢错了。
struct TrackDbgRec {
    int64_t tsNs = 0;
    uint16_t n = 0;
    uint16_t _pad0 = 0;
    uint32_t _pad1 = 0;
    // 后面紧跟 n 个 TrackDbgItem
};

struct TrackDbgItem {
    float x = 0, y = 0, z = 0;
    float vx = 0, vy = 0, vz = 0;   // 恒速模型速度（单位/帧）
    float posVarTrace = -1.f;       // 位置协方差的迹，反映这个点有多"虚"
    float velVarTrace = -1.f;
    float assocDistMm = -1.f;       // 本帧关联上的那个观测离预测多远
    float assocMahaSq = -1.f;       // 马氏距离平方（跟 chiSquareGate 对比）
    float qBoost = 1.f;             // 机动自适应过程噪声倍率
    float residualMm = -1.f;
    int32_t id = -1;
    int32_t obsIndex = -1;          // 关联到的 cluster 下标，-1=本帧没关联上
    int16_t missedFrames = 0;
    int16_t hits = 0;
    uint8_t justAcquired = 0;
    uint8_t coasting = 0;
    uint8_t confirmed = 0;
    uint8_t gateBypassed = 0;       // 走了双锚点回退
};

// ---- 块 28：3D 点 <-> 2D 光斑的对应关系 ------------------------------------
//
// 【这条线索本来就在手边，只是被丢掉了】MultiViewCluster 的 Track3D::support
// 是一串 (相机下标, 观测下标)，而录制时只把它压成了 supportMask 位图 ——
// 相机知道了，是那台相机的第几个光斑却没了。
//
// 【为什么这个下标要紧】本系统最难查的失效模式是两球粘连：它不报错、不丢点，
// 只是给出一个位于两球之间的错误质心，然后被正常三角化、关联、写进结果。
// 要证实它，必须能从一个可疑的 3D 点【反查】到构成它的那几个 2D 光斑，
// 再去看那些光斑的面积和外接框长宽比（Blob2D 里已经存了）。
// 没有这个下标，这条链就断在中间，粘连只能靠猜。
//
// unmatched 也记：某台相机有一堆没归入任何 track 的观测，要么是标定不准
// 导致极线对不上，要么是那台相机有额外反光 —— 两者都会推高鬼点率。
struct ClusterLinkRec {
    int64_t tsNs = 0;
    uint16_t nTracks = 0;
    uint16_t nUnmatched = 0;   // 后面 unmatched 项的总数
    uint8_t  nCams = 0;
    uint8_t  _pad0[3]{};
    // 后面紧跟：
    //   nTracks 个 { uint8 nSup; uint8 pad[3]; 然后 nSup 个 {uint8 cam; uint8 pad; uint16 obs} }
    //   nUnmatched 个 { uint8 cam; uint8 pad; uint16 obs }
};

// 【钉死尺寸】这几个数是 pcrec.py 里 dtype.itemsize 必须等于的值。
// 改字段忘了改另一边，编译期就断在这里，而不是让分析脚本读出一堆错位的垃圾数。
static_assert(sizeof(JointOutRec)   == 1032, "JointOutRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(HandednessRec) ==   80, "HandednessRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(MarkerDbgRec)  ==  888, "MarkerDbgRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(RunFlagsRec)   ==   48, "RunFlagsRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(Hm20DiagRec)   ==  368, "Hm20DiagRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(SkeletonRec)   ==  668, "SkeletonRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(NetRawRec)     ==  840, "NetRawRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(NetTopK)       ==   20, "NetTopK 尺寸变了，同步改 tools/pcrec/pcrec.py");
// ---- v5 ----
static_assert(sizeof(MarkerStageRec)  == 1952, "MarkerStageRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(FilterDbgRec)    == 1112, "FilterDbgRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(IkDbgRec)        ==  680, "IkDbgRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(TemplateSnapRec) ==  736, "TemplateSnapRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(TimingRec)       ==  112, "TimingRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(UdpOutRec)       ==   80, "UdpOutRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(EventRec)        ==   56, "EventRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(AutoCalibDbgRec) ==  304, "AutoCalibDbgRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(TrackDbgRec)     ==   16, "TrackDbgRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(TrackDbgItem)    ==   64, "TrackDbgItem 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(ClusterLinkRec)  ==   16, "ClusterLinkRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
// ===========================================================================
// v7 新增块：ROM 标定这一路
// ===========================================================================

// ---- 块 31：ROM 标定的逐维全量状态 -----------------------------------------
//
// 【为什么要整块单独存，而不是塞进 JointOut 的 romLo/romHi】
// romLo/romHi 只回答「区间是多少」，而标不满时要问的是「【为什么】是这么多」。
// 那个答案在这几个数里：
//   · nSamples / nSeen —— 这一维见到 900 帧只收了 40 个样本，说明门把它挡住了
//   · nReject[6]       —— 挡住它的是哪一道门。NotFresh 说明帧在保持上一帧
//                         （手背丢了），AxisDegen 说明骨轴退化（上游几何问题），
//                         两者的修法完全相反，而只看 nSamples 分不开
//   · sign / signDelta —— 「方向反了」这个症状的直接证据。signDelta 是
//                         握拳端中位减张开端中位，它为负就是反的
//   · openMed/closeMed —— 两端各自停在哪。两个数几乎相等 = 手根本没做到底
//   · extrapLo/HiRad   —— 行程里有多少是外推补的。全靠外推撑起来的覆盖度
//                         和真采到的覆盖度，可信度差很远
//
// 写入时机：romStart 一次、romFinish 一次、标定过程中 1Hz 一次。
// reason 区分这三种 —— 过程中的那几份能看出「覆盖度是怎么一步步涨上去的」，
// 一个刚过线的标定和一个远超阈值的标定，ready 是一样的。
struct RomCalibRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    // ---- 逐维 ----
    float   lo[16]{};            // 带符号空间 v=sign*q 的区间（映射的分母）
    float   hi[16]{};
    float   rawLo[16]{};         // 换算回原始 q 的区间，给人对着面板看
    float   rawHi[16]{};
    float   measLo[16]{};        // 【纯实测】区间，不含外推。跟 lo/hi 的差就是外推量
    float   measHi[16]{};
    float   coverage[16]{};      // (hi-lo)/解剖行程，封顶 1
    float   signDeltaRad[16]{};  // 握拳端中位 - 张开端中位（原始 q）。<0 = 方向是反的
    float   signCorr[16]{};      // 与 curl 的秩相关，仅参考
    float   openMedRad[16]{};    // 张开端中位
    float   closeMedRad[16]{};   // 握拳端中位
    float   curlSpread[16]{};    // 该指 curl 的 p90-p10。<0.15 = 手没真做开合
    float   fitSlope[16]{};      // v 对 curl 的拟合斜率
    float   fitR2[16]{};
    float   extrapLoRad[16]{};   // 两端各外推了多少弧度
    float   extrapHiRad[16]{};
    float   curlSeenLo[16]{};    // 该指全程 curl 的 p2/p98（含角度被拒的帧）
    float   curlSeenHi[16]{};
    float   curlUsedLo[16]{};    // 被接受样本覆盖到的 curl 区间
    float   curlUsedHi[16]{};

    int32_t nSamples[16]{};      // 被接受的样本数
    int32_t nSeen[16]{};         // 见到的总帧数
    int32_t nOpen[16]{}, nClose[16]{};
    // 逐维逐原因的拒收数。原因码见 rom::RejectCode：
    //   0=接受 1=不新鲜(保持上一帧) 2=骨轴退化 3=curl算不出 4=野点 5=只有预测值
    int32_t nReject[16][6]{};

    int8_t  sign[16]{};          // +1 = 屈曲使 q 变大；-1 = 变小（已自动纠正）
    uint8_t signResolved[16]{};  // 方向是判出来的(1)还是默认给的(0)
    // 状态码，见 rom::DofStatus：
    //   0=正常 1=行程不足已兜底 2=方向不明 3=样本不足 4=骨轴退化(上游问题)
    //   5=行程含外推
    uint8_t status[16]{};

    // ---- 汇总 ----
    float   coverageFlex = -1.f; // 11 个屈曲维的平均覆盖度。这就是面板上那个百分比
    float   coverageAbd  = -1.f;
    float   durationSec  = 0.f;
    int32_t nOkFlex = 0, nPriorFlex = 0, nDeadFlex = 0, nSignFlipped = 0;
    int32_t nFrames = 0, nFramesUsed = 0;
    int32_t reason = 0;          // 0=标定开始 1=标定中(1Hz) 2=标定结束 3=加载自文件
    uint8_t ready = 0;
    uint8_t accumulated = 0;     // 这一轮是补标（保留了上一轮样本）
    uint8_t _pad1[2]{};

    // ---- v8：标定配置快照（rom::Config 全量）------------------------------
    // 【为什么结果旁边必须放判据】上面那一堆是"标成了什么样"，而"为什么判成
    // 这样"取决于这组阈值：同一批样本，minRangeRad 从 0.35 调到 0.25，
    // 一半的维就从 PriorFilled 变成 Ok，覆盖度那个百分比一个字都没变。
    // 也就是说【两份看起来完全一样的记录可以是两个完全不同的结论】。
    //
    // 现场最常见的一句话是"我们上次好像调过那个阈值"—— 没有这段快照，
    // 这句话既证实不了也证伪不了，而它恰恰决定了后面所有分析的前提。
    //
    // 【为什么不放 stateJson 里就算了】stateJson 是 1Hz 的、可能被丢块、
    // 而且跟这一份标定结果没有帧级的绑定关系。配置和结果必须是同一条记录，
    // 否则"这份结果是在哪组阈值下算出来的"仍然要靠时间戳去猜。
    float   cfgMinRangeRad = 0.f;
    float   cfgMinCurlSpread = 0.f;
    float   cfgMinSignDeltaRad = 0.f;
    float   cfgOpenFrac = 0.f, cfgCloseFrac = 0.f;
    float   cfgEndQuantile = 0.f;
    float   cfgGlobalQLo = 0.f, cfgGlobalQHi = 0.f;
    float   cfgOutlierAbsRad = 0.f;
    float   cfgExtrapMinR2 = 0.f, cfgExtrapMinCurlSpan = 0.f, cfgExtrapMaxFrac = 0.f;
    float   cfgPriorSpanFrac = 0.f;
    int32_t cfgMinSamplesPerDof = 0;
    int32_t cfgMinOkFlexDofs = 0;
    int32_t cfgMaxSamplesPerDof = 0;
    uint8_t cfgLearnSign = 0, cfgAcceptPredicted = 0, cfgAcceptDegenerate = 0;
    uint8_t cfgExtrapolateByCurl = 0, cfgFillFromPrior = 0, cfgHoldOnDegenerate = 0;
    uint8_t _pad2[2]{};
};

// ---- 块 32：关节角解算的逐帧内部量 -----------------------------------------
//
// 【这一块补的是 JointOut 上游的那一截】
// JointOut 从 qSolve 开始记，而 qSolve 本身是怎么算出来的没有记：
//     PIP  = 两条骨轴的夹角
//     MCP  = 近节骨轴在腕部系里的两个方位角
// 「四元数是对的但角度不对」这句话之所以成立，就是因为坏在【选哪两条轴】上，
// 而轴本身从来没出过 solveJointAngles 这个函数。
//
// dotProxMid 是这一块里最该先看的一个数：它 ≈1 就说明两条"骨轴"其实是
// 同一根骨头，此时 PIP 恒等于 0 —— 不是不准，是结构性的零。
struct JointSolveRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    // ---- 两条骨轴（腕部系，单位化）。PIP 就是这两条的夹角 ----
    float axProx[5][3]{};        // a0：近节骨轴，取自 segQuat[近节] 第 0 列
    float axMid[5][3]{};         // a1：中节骨轴，直接由 pp球->mp球 算
    float axDist[5][3]{};        // a2：远节骨轴（拇指 IP 用）
    float axProxWorld[5][3]{};   // a0 的世界系版本，跟录下来的四元数直接对照

    float dotProxMid[5]{};       // 【先看这个】≈1 = 两条轴是同一根骨头
    float hingeSigned[5]{};      // dot(cross(a0,a1), hinge)，PIP 符号的来源
    float flexRaw[5]{}, abdRaw[5]{}, pipRaw[5]{}, ipRaw[5]{};  // 三个提取式的原始输出

    // ---- curl：方向判定和行程外推的唯一依据 ----
    // 【它为什么在角度失效时还活着】只用球心间距算，不经过腕部系/四元数/anchor
    float curlChain[5]{};        // |pp->dp| / (|pp->mp|+|mp->dp|)，1=伸直
    float curl[5]{};             // 归一化闭合度 0=张开 1=握到底
    float boneLenMm[5]{};        // |pp->mp|+|mp->dp|，curl 的分母
    uint8_t curlValid[5]{};
    uint8_t curlPredicted[5]{};  // curl 是靠预测球算的（骨长被回正过，仍可用）

    // ---- 近节骨轴的来源。【ROM 采样的门就架在这个字段上】----
    //   0 = anchor->pp球（正确） 1 = pp球->mp球（退化，PIP 恒为 0） 2 = 无
    uint8_t proxAxisSrc[5]{};
    // ---- 逐维状态，见 rom::DofState ----
    //   0=没算 1=保持上一帧 2=骨轴退化 3=预测段算的 4=实测
    // 【v6 缺的就是这一块】v6 只有逐指的 fingerValid，而腕部系失效时
    // 同一根手指里 PIP 是新算的、MCP 是保持的，fingerValid 给的是同一个 true。
    uint8_t dofState[16]{};
    // 本帧 ROM 采样对这 16 维各自的判决，见 rom::RejectCode。
    // 【为什么要逐帧记而不是只记总数】「这一维只攒到 40 个样本」是结论，
    // 而「在动作的哪一段被拒的」才是原因 —— 集中在握拳那几秒还是全程均匀，
    // 指向的问题完全不同。romLearning 为假时全 0。
    uint8_t romReject[16]{};

    uint8_t wristPoseValid = 0;
    uint8_t anchorsValid = 0;
    uint8_t mirrored = 0;
    uint8_t romLearning = 0;
    int32_t wristStale = 0;      // 腕部位姿已沿用了多少帧

    // ---- v8：PIP 的四元数解算路径 ------------------------------------------
    // 【为什么加这一组】v7 只录了"叉乘路径"的中间量（axProx/axMid/dotProxMid/
    // hingeSigned）。但实测 anchorsValid 恒为假时叉乘路径整个失效
    // （a0==a1 -> PIP=acos(1)=0），真正在用的是相对四元数那条路，而那条路的
    // 中间量【一个都没录】。于是"PIP 为什么是这个值"在文件里查不到。
    //
    // 这几个字段就是那条路的全部输入和输出，够复现整个计算：
    //   qRelPip = conj(segQuat[近节]) * segQuat[中节]
    //   pipFromQuat = 2*acos(|qRelPip.w|)
    // 对照 pipRaw 就能一眼看出本帧走的是哪条路、两条路差多少。
    float qRelPip[5][4]{};       // 中节相对近节的四元数 (w,x,y,z)
    float pipFromQuat[5]{};      // 上式解出的 PIP（弧度，无符号）
    float pipFromAxis[5]{};      // 叉乘路径解出的 PIP，保留下来做对照
    uint8_t pipUsedQuat[5]{};    // 本帧这一指的 PIP 最终取自哪条路：1=四元数 0=叉乘
    uint8_t _pad2[3]{};
};

// ---- 块 33：ROM 映射器的逐维逐帧分支（v8）----------------------------------
//
// 【这一块回答的是唯一一个问题：这一维为什么不动】
// Mapper::apply() 里通向"输出是一个不动的数"的路一共有四条，而它们产生的
// 输出【完全无法区分】—— 都是一个常数。四条路的成因和修法却毫无共同点：
//
//   branch=1 死维(NoData/AxisDead)  -> neutral()
//       成因在【标定阶段】：这一维样本太少或几乎全被 AxisDegen 拒了。
//       修法：回去看块 31 的 nReject，是上游几何问题，重标没用。
//
//   branch=2 保持上一帧(hold)        -> last_
//       成因在【本帧】：dofState 是 Held/Degenerate/Missing。
//       修法：看块 32 的 dofState 和 proxAxisSrc —— 腕部系或 anchor 的问题。
//       【这一条最像"死机"】：只要 dofState 一直不新鲜，输出就一直是同一个
//       数，而标定结果、覆盖度、romReady 全都显示正常。
//
//   branch=3 区间塌了(span<1e-3)     -> neutral()
//       成因在【标定结果】：hi-lo≈0。修法：重标这一维。
//
//   branch=4 正常映射，但 t 恒被钳到 0 或 1
//       成因：当前姿态落在标定区间【之外】。这不是 bug，是标定时没做到这个
//       姿态。修法：补标。clampLo/clampHi 逐帧记，恒 1 就是这一条。
//
// 【为什么 branch=0 也要记】未标定时走 clamp(q, tgtLo, tgtHi) 透传。
// "标定之前就一直触限"的全部解释就在 tgtLo/tgtHi 和 clampLo/clampHi 这四个
// 量里，而它们在 v7 里【一个都没有】—— 目标行程可以被 setTargetRange()
// 改写，改写后的值从来没出过 Mapper 这个类。
//
// 【uNorm 为什么要单独记而不是离线重算】重算需要 sign、lo、hi、tLo、tHi
// 五个量【以及 Mapper 内部那几个 clamp 的确切顺序】。离线复刻一遍，
// 复刻得对不对本身又要验证 —— 而这正是排查时最不该再引入的不确定性。
struct RomMapDbgRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    // v = sign*q，即带符号空间里的输入。跟块 13 的 qSmooth 对照可验证 sign。
    float   vSigned[16]{};
    // t = (v-lo)/span，【钳位之前】的值。<0 或 >1 就是本帧姿态在标定区间外，
    // 而钳位之后它恒等于 0 或 1 —— 那个 0/1 在输出里看不出是"到端点了"
    // 还是"信号本来就这么大"。
    float   uNorm[16]{};
    // 目标行程。默认取 jointLimits()，但 setTargetRange() 可以逐维改写，
    // 改写值以前从不落盘。tgtHi-tgtLo≈0 => 输出恒等于 tgtLo（又一条冻结路径）。
    float   tgtLo[16]{}, tgtHi[16]{};

    // 连续走 branch=2(保持) 的帧数。【这一个数就能定位"冻结"】
    // 它一路涨到几百，而 romReady/coverage 全部正常 —— 这是最常见的现场。
    int32_t heldRun[16]{};
    // 输出连续未变化(|Δ|<1e-6)的帧数。跟 heldRun 分开记：
    // 输出不变【不一定】是保持分支造成的，也可能是 branch=1/3 的 neutral、
    // 或 branch=4 但 t 恒被钳到同一端。两个数一起看才能定位到具体分支。
    int32_t frozenRun[16]{};

    uint8_t branch[16]{};    // 0=未标定clamp 1=死维neutral 2=保持上帧
                             // 3=区间塌neutral 4=正常映射
    uint8_t custom[16]{};    // 目标行程被 setTargetRange 改写过
    uint8_t clampLo[16]{};   // 本帧这一维被钳在下端
    uint8_t clampHi[16]{};   // 本帧这一维被钳在上端
    uint8_t status[16]{};    // rom::DofStatus，映射时实际读到的那一份
    uint8_t dofState[16]{};  // rom::DofState，branch=2 的判据【输入】。
                             // 跟块 32 冗余，但那一块只在 Full 档才写，
                             // 而"判据和判决必须在同一条记录里"——
                             // 跨块按时间戳去 join 是又一个可能出错的环节。
    int8_t  sign[16]{};      // 映射时实际用的方向

    // ---- 本帧汇总。【先看这四个数，再决定要不要展开 16 维】----
    int32_t nHeld = 0;        // 本帧走保持分支的维数
    int32_t nFrozen = 0;      // 本帧输出没变的维数（frozenRun>0）
    int32_t maxFrozenRun = 0; // 最长的连续冻结帧数
    int32_t maxFrozenIdx = -1;
    int32_t nClamped = 0;     // 本帧触限的维数（含未标定的透传钳位）
    int32_t nNeutral = 0;     // 本帧被钉在中立位的维数（branch=1 或 3）
    float   dtSec = -1.f;

    uint8_t romReady = 0;
    uint8_t holdOnDegenerate = 0;  // Mapper 的 cfgHold_。关掉它冻结路径②就没了
    uint8_t hasLast = 0;           // 保持器里已经有上一帧的值
    uint8_t mapperJustReset = 0;   // 本帧 Mapper::reset() 刚被调过（区间换了）
    uint8_t rateLimitOn = 0;
    uint8_t romLearning = 0;
    uint8_t _pad1[2]{};
};

// ---- 块 34：角度后处理链的隐藏状态（v8）------------------------------------
//
// 【这一块补的是 qSolve->qSmooth 和 qRom->qOut 这两段】
// 块 13 记了四级的【值】，块 33 记了中间那一级（ROM 映射）的【内部】。
// 剩下两级的内部一直是空白，而它们各自都能单独造成"输出不对/不动"：
//
//   · 平滑器的 off[] 是一个【直接加到输出上的逐维常量偏置】，
//     最大到 maxOffsetRad(0.7rad ≈ 40°)。它在系统里没有第二份 ——
//     apply() 就地算完就返回，外面只看得到 qSmooth。于是这两件事
//     在文件里长得完全一样：
//         ① 解算本来就偏了 40°
//         ② 解算是对的，被一个没还清的恢复补偿顶着
//     ② 的成因是"预测段跑飞过一次"，修法在遮挡处理上，跟 ① 毫无关系。
//
//   · 平滑器的 s[] 是【预测段的实际输出】。alpha 小的时候它跟不上真实
//     角度，症状是"握拳时手指弯得不够/像卡住"—— 而握拳恰恰全程都是
//     预测段（手背被四指挡住）。不记 s，这个症状会被误判成解算问题。
//
//   · 限幅器：qOut-qRom 确实等于削掉量，但只在限幅【开着】时成立。
//     关掉时两者恒等，"关着"和"开着但一次都没削"就分不开了 ——
//     而那正是排除这一级时要回答的问题。
struct AngleChainRec {
    RecHdr  hdr;
    int32_t _pad0 = 0;
    int64_t wallNs = 0;
    int64_t frameTsNs = -1;

    // ---- 平滑器 ----
    float   smState[16]{};        // s[]：预测段实际输出的那个值
    float   smOffset[16]{};       // off[]：【加在输出上的常量偏置】
    uint8_t smHas[16]{};
    uint8_t smOffClamped[16]{};   // 本帧 off 被 maxOffsetRad 截断 ——
                                  // 截断 = 预测段至少跑飞了 40°，一条很强的结论，
                                  // 而钳完之后 off 看起来就是个正常的 0.7
    uint8_t smWasPred[5]{};       // 上一帧该指是不是预测段（off 只在恢复帧产生）
    uint8_t smPredNow[5]{};
    uint8_t _pad1[6]{};
    float   smAlpha = -1.f;
    float   smDecay = -1.f;
    float   smMaxOffsetRad = -1.f;
    float   maxSmOffsetRad = 0.f;
    int32_t maxSmOffsetIdx = -1;
    int32_t nSmOffActive = 0;     // off 非零的维数

    // ---- 限幅器 ----
    float   rlClip[16]{};         // 逐维削掉量（带符号）。全 0 且 rlOn=1 = 没削过
    float   rlStepLimitRad = -1.f;// 本帧允许动的最大步长 = maxRadPerSec*max(dt,1e-3)
    float   rlMaxRadPerSec = -1.f;
    float   maxRlClipRad = 0.f;
    int32_t maxRlClipIdx = -1;
    int32_t nRlClipped = 0;

    float   dtSec = -1.f;
    uint8_t smOn = 0, rlOn = 0, rlHadState = 0, lowLatency = 0;
    uint8_t _pad2[4]{};
};

// ---- 块 35：逐块类型的写入/丢弃统计（v8）------------------------------------
//
// 【这一块回答的是"这一块为什么不在文件里"】以前只有 Trailer 里的三个总数，
// 于是同一个现象有三种完全不同的解释，而它们在文件里【无法区分】：
//   ① 这一类块压根没被调用（代码路径没走到 / 条件没满足）
//   ② 被 Detail 档位关掉了
//   ③ 写了，但队列满被丢了
// 三者的下一步动作毫无共同点：①去查为什么没触发、②重录时调档、③换盘或降档。
// 猜错一次就是白查一整轮，而"块不在文件里"恰恰是最常见的起点。
//
// 逐类型三个计数一摆，三者立刻分开：
//   pushed=0 dropped=0 -> ①或②（配合头里的 detail 字段就能定死是哪个）
//   pushed>0 dropped>0 -> ③，而且知道丢了多少、丢的是哪一类
struct ChunkStatsRec {
    RecHdr   hdr;
    int32_t  _pad0 = 0;
    int64_t  wallNs = 0;
    int32_t  detail = 0;          // 录制时的 Detail 档位
    int32_t  nTypes = 64;         // 下面三个数组的长度，固定 64（块类型上限）
    uint64_t pushed[64]{};        // 入队成功
    uint64_t dropped[64]{};       // 队列满被丢
    uint64_t bytes[64]{};         // 入队字节（含 5 字节块头）
    uint64_t queuePeak = 0;       // 队列深度峰值。逼近上限 = 丢块即将发生
    uint64_t droppedTotal = 0;
    uint64_t pushedTotal = 0;
    uint64_t bytesTotal = 0;
};

// 【192 -> 240】v6.2 加了三组逐指字段：anchorOkPerFinger[5] / planeSampleN[5]
// / anchorOutOfPlaneDeg[5]。**tools/pcrec/pcrec.py 的 ChainDbg dtype 必须同步改**，
// 否则旧解析器会拿 recBytes 一比对不上、直接报错（这是设计好的行为，
// 不会静默错位）。新增三个字段接在 anchorsVerifyNOk 之后、_pad1 之前。
static_assert(sizeof(ChainDbgRec)     ==  240, "ChainDbgRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
// ---- v7 ----
// 【504 -> 632】v8 给 JointSolveRec 加了 PIP 的四元数解算路径
// （qRelPip / pipFromQuat / pipFromAxis / pipUsedQuat + 3 字节补齐）。
// **tools/pcrec/pcrec.py 的 JointSolve dtype 必须同步改**，否则旧解析器
// 拿 recBytes 一比对不上会直接报错（这是设计好的行为，不会静默错位）。
static_assert(sizeof(JointSolveRec)   ==  632, "JointSolveRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
// ---- v8 ----
// 【2040 -> 2112】RomCalibRec 接了 rom::Config 快照。
// **tools/pcrec/pcrec.py 的 ROMCALIB dtype 必须同步改**，否则旧脚本拿
// recBytes 一比对不上会报 schema 错误并跳过整块（设计好的行为，不会静默错位）。
static_assert(sizeof(RomCalibRec)     == 2112, "RomCalibRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(RomMapDbgRec)    ==  560, "RomMapDbgRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(AngleChainRec)   ==  320, "AngleChainRec 尺寸变了，同步改 tools/pcrec/pcrec.py");
static_assert(sizeof(ChunkStatsRec)   == 1592, "ChunkStatsRec 尺寸变了，同步改 tools/pcrec/pcrec.py");

// ---------------------------------------------------------------------------
// 小工具：往字节缓冲追加
namespace io {
template <typename T> inline void put(std::vector<char>& b, const T& v) {
    static_assert(std::is_trivially_copyable<T>::value, "POD only");
    const size_t o = b.size();
    b.resize(o + sizeof(T));
    std::memcpy(b.data() + o, &v, sizeof(T));
}
inline void putBytes(std::vector<char>& b, const void* p, size_t n) {
    const size_t o = b.size();
    b.resize(o + n);
    std::memcpy(b.data() + o, p, n);
}
}  // namespace io

// ---------------------------------------------------------------------------
class Recorder {
public:
    ~Recorder() { stop(); }

    // headerJson 由调用方拼好（相机标定 + 参数快照 + 模板 + 协议 + 备注）。
    // 这里不解析它，只原样写进去——录制器不该懂业务字段，加一个参数就要改
    // 一次录制器的话，早晚会有人图省事不加。
    bool start(const std::string& path, const std::string& headerJson) {
        stop();
        out_.open(path, std::ios::binary | std::ios::trunc);
        if (!out_) return false;
        out_.write(kMagic, 8);
        const uint32_t ver = kVersion;
        out_.write(reinterpret_cast<const char*>(&ver), 4);
        const uint32_t hlen = uint32_t(headerJson.size());
        out_.write(reinterpret_cast<const char*>(&hlen), 4);
        out_.write(headerJson.data(), std::streamsize(hlen));
        if (!out_) { out_.close(); return false; }
        path_ = path;
        bytes_.store(16 + hlen);
        dropped_.store(0);
        chunks_.store(0);
        pushed_.store(0);
        run_.store(true);
        th_ = std::thread([this] { drain(); });
        return true;
    }

    void stop() {
        // 【顺序：先统计后收尾】ChunkStats 自己也要被计进 pushed，
        // 而 Trailer 里的总数是在它之后取的，两者对得上才说明统计没漏。
        if (run_.load()) writeChunkStats();
        if (run_.load()) writeTrailer(0);
        if (!run_.exchange(false)) return;
        cv_.notify_all();
        if (th_.joinable()) th_.join();
        if (out_.is_open()) { out_.flush(); out_.close(); }
    }

    bool recording() const { return run_.load(); }
    uint64_t bytesWritten() const { return bytes_.load(); }
    uint64_t chunksWritten() const { return chunks_.load(); }
    uint64_t droppedChunks() const { return dropped_.load(); }
    const std::string& path() const { return path_; }

    // ---- 各类块 ----
    // 收尾统计。【必须有】否则"这份文件到底丢没丢块"事后无从查证，
    // 而丢块意味着数据有洞，会把调参结论带偏却不留任何痕迹。
    void writeTrailer(int64_t tsNs) {
        std::vector<char> b;
        io::put(b, tsNs);
        // 【用 pushed_ 而不是 chunks_】chunks_ 是【写盘线程】数的，而 trailer 是在
        // 主线程 stop() 里生成的 —— 那一刻队列里通常还压着上千个块没落盘，
        // 于是记下来的数远小于真实值（实测 3003 个块记成 899）。
        // 而 trailer 的全部意义就是事后核对"有没有丢数据"，数字不准等于没有。
        io::put(b, uint64_t(pushed_.load()));
        io::put(b, uint64_t(dropped_.load()));
        io::put(b, uint64_t(bytes_.load()));
        push(ChunkType::Trailer, std::move(b));
    }

    // 相机时钟 <-> 墙钟。【必须有】3D 点用的是相机采集时间戳，而骨架结果是
    // 从工作线程异步回来的、只能打墙钟。两个时基不对齐的话，我无法确定
    // 某一帧骨架对应的是哪一帧点云 —— 比对基线这件事就做不成。
    void writeClockSync(int64_t camTsNs, int64_t wallNs) {
        if (!run_.load()) return;
        std::vector<char> b;
        io::put(b, camTsNs);
        io::put(b, wallNs);
        push(ChunkType::ClockSync, std::move(b));
    }

    void writeCamBlobs(uint32_t camId, int64_t tsNs, const std::vector<Blob2D>& blobs) {
        if (!run_.load()) return;
        std::vector<char> b;
        b.reserve(16 + blobs.size() * sizeof(Blob2D));
        io::put(b, camId);
        io::put(b, tsNs);
        io::put(b, uint16_t(blobs.size()));
        if (!blobs.empty()) io::putBytes(b, blobs.data(), blobs.size() * sizeof(Blob2D));
        push(ChunkType::CamBlobs, std::move(b));
    }

    void writePoints3D(int64_t tsNs, const std::vector<Point3DRec>& pts) {
        if (!run_.load()) return;
        std::vector<char> b;
        b.reserve(12 + pts.size() * sizeof(Point3DRec));
        io::put(b, tsNs);
        io::put(b, uint16_t(pts.size()));
        if (!pts.empty()) io::putBytes(b, pts.data(), pts.size() * sizeof(Point3DRec));
        push(ChunkType::Points3D, std::move(b));
    }

    void writeCluster(int64_t tsNs, const std::vector<ClusterRec>& tr) {
        if (!run_.load()) return;
        std::vector<char> b;
        b.reserve(12 + tr.size() * sizeof(ClusterRec));
        io::put(b, tsNs);
        io::put(b, uint16_t(tr.size()));
        if (!tr.empty()) io::putBytes(b, tr.data(), tr.size() * sizeof(ClusterRec));
        push(ChunkType::Cluster3D, std::move(b));
    }

    void writeSkeleton(int64_t tsNs, const SkeletonRec& s) {
        if (!run_.load()) return;
        std::vector<char> b;
        b.reserve(8 + sizeof(SkeletonRec));
        io::put(b, tsNs);
        io::put(b, s);
        push(ChunkType::Skeleton, std::move(b));
    }

    // 【注意签名】不额外传 tsNs —— 时间戳已经在结构体里（wallNs + frameTsNs
    // 两个都有）。放在一起是因为这两个时基的对应关系本身就是要分析的东西之一。
    void writeHm20Diag(const Hm20DiagRec& d) {
        if (!run_.load()) return;
        std::vector<char> b;
        b.reserve(sizeof(Hm20DiagRec));
        io::put(b, d);
        push(ChunkType::Hm20Diag, std::move(b));
    }

    void writeAssocInput(int64_t tsNs, int nBeforeCap,
                         const std::vector<std::pair<int, std::array<double,3>>>& cand) {
        if (!run_.load()) return;
        std::vector<char> b;
        b.reserve(16 + cand.size() * 16);
        AssocInputRec h;
        h.tsNs = tsNs;
        h.n = uint16_t(cand.size());
        h.nBeforeCap = uint16_t(nBeforeCap);
        io::put(b, h);
        for (const auto& c : cand) {
            io::put(b, float(c.second[0]));
            io::put(b, float(c.second[1]));
            io::put(b, float(c.second[2]));
            io::put(b, int32_t(c.first));
        }
        push(ChunkType::AssocInput, std::move(b));
    }

    void writeNetRaw(const NetRawRec& r, const std::vector<NetTopK>& topk) {
        if (!run_.load()) return;
        std::vector<char> b;
        b.reserve(sizeof(NetRawRec) + topk.size() * sizeof(NetTopK));
        io::put(b, r);
        if (!topk.empty()) io::putBytes(b, topk.data(), topk.size() * sizeof(NetTopK));
        push(ChunkType::NetRaw, std::move(b));
    }

    void writeCamSettings(const CamSettingsRec& c) {
        if (!run_.load()) return;
        std::vector<char> b;
        io::put(b, c);
        push(ChunkType::CamSettings, std::move(b));
    }

    // ---- v4 ----
    // 【统一走这个模板】每个新块自己填 hdr.recBytes = sizeof(自己)，解析器
    // 拿它跟 dtype.itemsize 对账。手写四遍同样的三行代码迟早会漏掉一个。
    template <typename T>
    void writeRec(ChunkType t, T r) {
        if (!run_.load()) return;
        r.hdr.recVersion = 1;
        r.hdr.recBytes = uint16_t(sizeof(T));
        std::vector<char> b;
        b.reserve(sizeof(T));
        io::put(b, r);
        push(t, std::move(b));
    }

    void writeJointOut(const JointOutRec& r)   { writeRec(ChunkType::JointOut, r); }
    void writeHandedness(const HandednessRec& r) { writeRec(ChunkType::Handedness, r); }
    void writeMarkerDbg(const MarkerDbgRec& r) { writeRec(ChunkType::MarkerDbg, r); }
    void writeRunFlags(const RunFlagsRec& r)   { writeRec(ChunkType::RunFlags, r); }

    // ---- v5 ----
    // 【详细度检查放在录制器里，不放在调用方】放调用方的话，十几个调用点
    // 每个都要写一遍 if，早晚漏一个 —— 漏掉的那个会在 Basic 档偷偷多写数据，
    // 而这种"少数几个块偶尔出现"的文件最难解析。
    void setDetail(Detail d) { detail_.store(int(d)); }
    Detail detail() const { return Detail(detail_.load()); }
    bool wantsFull() const { return detail_.load() >= int(Detail::Full); }
    bool wantsParanoid() const { return detail_.load() >= int(Detail::Paranoid); }

    void writeMarkerStage(const MarkerStageRec& r) { if (wantsFull()) writeRec(ChunkType::MarkerStage, r); }
    void writeFilterDbg(const FilterDbgRec& r)     { if (wantsFull()) writeRec(ChunkType::FilterDbg, r); }
    void writeIkDbg(const IkDbgRec& r)             { if (wantsFull()) writeRec(ChunkType::IkDbg, r); }
    void writeTiming(const TimingRec& r)           { if (wantsFull()) writeRec(ChunkType::Timing, r); }
    // 【UdpOut 解除档位限制】80 字节/帧，120fps 下 9.6KB/s，可以忽略。
    // 它是输出链路的最后一环：包发没发、两个包的时间戳对不对得上 ——
    // 这三类问题在系统内部完全查不到，会被一路误判成解算问题往上游白查。
    // 为省 9.6KB/s 把它放进 Full 档，是拿最贵的排查时间换最便宜的磁盘。
    void writeUdpOut(const UdpOutRec& r)           { writeRec(ChunkType::UdpOut, r); }
    void writeAutoCalibDbg(const AutoCalibDbgRec& r) { if (wantsFull()) writeRec(ChunkType::AutoCalibDbg, r); }
    void writeChainDbg(const ChainDbgRec& r)         { if (wantsFull()) writeRec(ChunkType::ChainDbg, r); }
    // ---- v7 ----
    // 【RomCalib 不受 Detail 档位限制】它一次会话只写几份（开始/1Hz/结束），
    // 体积可以忽略，而它是 ROM 问题的第一现场。Basic 档录的人恰恰最可能
    // 是"随手录一段发过来"的场景，那时候更不能少这块。
    void writeRomCalib(const RomCalibRec& r)   { writeRec(ChunkType::RomCalib, r); }
    // 【JointSolve 解除档位限制】632 字节/帧。跟 13/33/34 一起构成
    // 【关节角这一路的完整证据链】，而链条只要断一环，剩下几环就只能证明
    // "问题不在我这一级"，证不出在哪一级 —— 那等于没有。
    // 四块合计约 2.5KB/帧、120fps 下 300KB/s，是这条链能查得动的最低成本。
    void writeJointSolve(const JointSolveRec& r) { writeRec(ChunkType::JointSolve, r); }
    // ---- v8 ----
    // 【RomMapDbg 也不受 Detail 限制】560 字节/帧，120fps 下 67KB/s ——
    // 相对 Basic 档的 0.6MB/s 是 11%。而"标定完输出就不动了"是现场报得最多的
    // 一类问题，它【只能】在这一块里定位。为省这 11% 把它放进 Full 档，
    // 换来的是 Basic 档录的那些"随手录一段发过来"的文件对这个问题完全无解。
    void writeRomMapDbg(const RomMapDbgRec& r) { writeRec(ChunkType::RomMapDbg, r); }
    // 【AngleChain 同样不受 Detail 限制】320 字节/帧，120fps 下 38KB/s。
    // 它补的是 qSolve->qSmooth 和 qRom->qOut 这两段的内部，而"输出不对"
    // 的成因分布在这两段里的比例并不低（平滑器的 off 能顶到 40°）。
    void writeAngleChain(const AngleChainRec& r) { writeRec(ChunkType::AngleChain, r); }

    // 逐块类型统计。收尾时自动写一份，也可以中途手工写。
    void writeChunkStats() {
        if (!run_.load()) return;
        ChunkStatsRec r;
        r.wallNs = 0;
        r.detail = detail_.load();
        r.nTypes = 64;
        for (size_t i = 0; i < 64; ++i) {
            r.pushed[i]  = stPushed_[i].load(std::memory_order_relaxed);
            r.dropped[i] = stDropped_[i].load(std::memory_order_relaxed);
            r.bytes[i]   = stBytes_[i].load(std::memory_order_relaxed);
        }
        { std::lock_guard<std::mutex> lk(m_); r.queuePeak = uint64_t(queuePeak_); }
        r.pushedTotal  = pushed_.load();
        r.droppedTotal = dropped_.load();
        r.bytesTotal   = bytes_.load();
        // 【把自己和 Trailer 算进去】快照是在 push 之前取的，所以这两类的
        // 计数天然是 0 —— 而读的人看到 pushed=0 会判成"这一块一次都没写"，
        // 也就是"真 bug"。让统计自己报一个虚假的 bug，是最没必要的坑。
        r.pushed[size_t(ChunkType::ChunkStats)] += 1;
        r.pushed[size_t(ChunkType::Trailer)]    += 1;
        writeRec(ChunkType::ChunkStats, r);
    }

    // ---- 录制清单 ----
    // 【为什么要把"本次会写哪些块"写进文件头】
    // 分析脚本看到"文件里没有第 N 号块"，有三种成因，而它们【无法从文件本身
    // 区分】：① 这个 build 根本不认识这个块（版本老）② 认识但被 Detail 档位
    // 关掉了 ③ 认识、开着，但代码路径一次都没走到（真 bug）。
    // 三者的下一步动作毫无共同点：①升级 ②重录时调档 ③去查为什么没触发。
    //
    // detail 字段只解决了 ②，剩下 ① 和 ③ 还是分不开 —— 而清单一摆就分开了：
    // 清单里没有 = ①；清单里有 gated=true = ②；清单里有 gated=false 而块 35 的
    // pushed=0 = ③，那是真 bug，值得去查。
    //
    // 【为什么放在 Recorder 里而不是面板里拼】档位门限的逻辑在这里，
    // 拼两份迟早会分叉，而分叉之后清单说的和实际写的不一致 ——
    // 那种不一致【只会在你正拿它排查问题的时候骗你】。
    static std::string manifestJson(int detail) {
        struct E { ChunkType t; const char* name; size_t bytes; };
        static const E kAll[] = {
            {ChunkType::CamBlobs,    "CamBlobs",     0},
            {ChunkType::Points3D,    "Points3D",     0},
            {ChunkType::Skeleton,    "Skeleton",     sizeof(SkeletonRec)},
            {ChunkType::Mark,        "Mark",         0},
            {ChunkType::ParamDelta,  "ParamDelta",   0},
            {ChunkType::Cluster3D,   "Cluster3D",    0},
            {ChunkType::Trailer,     "Trailer",      0},
            {ChunkType::ClockSync,   "ClockSync",    0},
            {ChunkType::Hm20Diag,    "Hm20Diag",     sizeof(Hm20DiagRec)},
            {ChunkType::AssocInput,  "AssocInput",   0},
            {ChunkType::NetRaw,      "NetRaw",       sizeof(NetRawRec)},
            {ChunkType::CamSettings, "CamSettings",  sizeof(CamSettingsRec)},
            {ChunkType::JointOut,    "JointOut",     sizeof(JointOutRec)},
            {ChunkType::Handedness,  "Handedness",   sizeof(HandednessRec)},
            {ChunkType::MarkerDbg,   "MarkerDbg",    sizeof(MarkerDbgRec)},
            {ChunkType::RunFlags,    "RunFlags",     sizeof(RunFlagsRec)},
            {ChunkType::StateJson,   "StateJson",    0},
            {ChunkType::MarkerStage, "MarkerStage",  sizeof(MarkerStageRec)},
            {ChunkType::FilterDbg,   "FilterDbg",    sizeof(FilterDbgRec)},
            {ChunkType::IkDbg,       "IkDbg",        sizeof(IkDbgRec)},
            {ChunkType::TemplateSnap,"TemplateSnap", sizeof(TemplateSnapRec)},
            {ChunkType::Timing,      "Timing",       sizeof(TimingRec)},
            {ChunkType::UdpOut,      "UdpOut",       sizeof(UdpOutRec)},
            {ChunkType::Event,       "Event",        sizeof(EventRec)},
            {ChunkType::AutoCalibDbg,"AutoCalibDbg", sizeof(AutoCalibDbgRec)},
            {ChunkType::AssignFull,  "AssignFull",   0},
            {ChunkType::TrackDbg,    "TrackDbg",     sizeof(TrackDbgItem)},
            {ChunkType::ClusterLink, "ClusterLink",  0},
            {ChunkType::NetInput,    "NetInput",     0},
            {ChunkType::ChainDbg,    "ChainDbg",     sizeof(ChainDbgRec)},
            {ChunkType::RomCalib,    "RomCalib",     sizeof(RomCalibRec)},
            {ChunkType::JointSolve,  "JointSolve",   sizeof(JointSolveRec)},
            {ChunkType::RomMapDbg,   "RomMapDbg",    sizeof(RomMapDbgRec)},
            {ChunkType::AngleChain,  "AngleChain",   sizeof(AngleChainRec)},
            {ChunkType::ChunkStats,  "ChunkStats",   sizeof(ChunkStatsRec)},
        };
        auto gated = [detail](ChunkType t) -> const char* {
            switch (t) {
                case ChunkType::AssignFull:
                case ChunkType::NetInput:
                    return detail >= int(Detail::Paranoid) ? "" : "paranoid";
                case ChunkType::MarkerStage:
                case ChunkType::FilterDbg:
                case ChunkType::IkDbg:
                case ChunkType::Timing:
                case ChunkType::AutoCalibDbg:
                case ChunkType::ChainDbg:
                case ChunkType::TrackDbg:
                case ChunkType::ClusterLink:
                    return detail >= int(Detail::Full) ? "" : "full";
                default:
                    return "";   // 不受档位限制
            }
        };
        std::string j = "{\"recorderVersion\":" + std::to_string(kVersion);
        j += ",\"headerRevision\":" + std::to_string(kHeaderRevision);
        j += ",\"detail\":" + std::to_string(detail);
        j += ",\"blocks\":[";
        bool first = true;
        for (const E& e : kAll) {
            if (!first) j += ",";
            first = false;
            const char* g = gated(e.t);
            j += "{\"id\":" + std::to_string(int(e.t));
            j += ",\"name\":\"" + std::string(e.name) + "\"";
            j += ",\"recBytes\":" + std::to_string(e.bytes);
            j += ",\"tier\":" + std::to_string(int(dropTier(e.t)));
            j += ",\"enabled\":" + std::string(g[0] ? "false" : "true");
            if (g[0]) j += ",\"needs\":\"" + std::string(g) + "\"";
            j += "}";
        }
        j += "]}";
        return j;
    }

    // 队列占用率 0..1。给面板做"录制健康度"指示用 —— 【在丢块之前就能看见】，
    // 而丢块一旦发生，那一段数据已经有洞了，再提醒也补不回来。
    double queueLoad() const {
        std::lock_guard<std::mutex> lk(m_);
        return double(q_.size()) / double(kMaxQueue);
    }

    // 模板快照。【不受 Detail 限制】它是变化时才写的，一次录制通常只有几份，
    // 体积可以忽略，而它是手性问题的核心证据 —— 为了省几 KB 把它关掉，
    // 换来的是整段数据在这个问题上作废。
    void writeTemplateSnap(TemplateSnapRec r) {
        r.fingerprint = fingerprintTemplate(r);
        writeRec(ChunkType::TemplateSnap, r);
    }

    // 事件。【同样不受 Detail 限制】理由同上，而且事件是稀疏的。
    void writeEvent(EventRec r, const std::string& text) {
        if (!run_.load()) return;
        r.hdr.recVersion = 1;
        r.hdr.recBytes = uint16_t(sizeof(EventRec));
        r.textLen = uint16_t(text.size());
        std::vector<char> b;
        b.reserve(sizeof(EventRec) + text.size());
        io::put(b, r);
        if (!text.empty()) io::putBytes(b, text.data(), text.size());
        push(ChunkType::Event, std::move(b));
    }

    // 便捷重载：绝大多数事件只需要 code + 一句话。
    void writeEvent(int code, int severity, int64_t wallNs, int64_t frameTsNs,
                    const std::string& text, double a = 0.0, double b = 0.0,
                    int ia = 0, int ib = 0) {
        EventRec e;
        e.wallNs = wallNs; e.frameTsNs = frameTsNs;
        e.code = code; e.severity = severity;
        e.valueA = float(a); e.valueB = float(b);
        e.intA = ia; e.intB = ib;
        writeEvent(e, text);
    }

    void writeTrackDbg(int64_t tsNs, const std::vector<TrackDbgItem>& items) {
        if (!run_.load() || !wantsFull()) return;
        std::vector<char> b;
        b.reserve(sizeof(TrackDbgRec) + items.size() * sizeof(TrackDbgItem));
        TrackDbgRec h;
        h.tsNs = tsNs;
        h.n = uint16_t(items.size());
        io::put(b, h);
        if (!items.empty()) io::putBytes(b, items.data(), items.size() * sizeof(TrackDbgItem));
        push(ChunkType::TrackDbg, std::move(b));
    }

    // support: 每个 track 一串 (相机下标, 观测下标)。unmatched 同样格式。
    void writeClusterLink(int64_t tsNs, int nCams,
                          const std::vector<std::vector<std::pair<int,int>>>& support,
                          const std::vector<std::pair<int,int>>& unmatched) {
        if (!run_.load() || !wantsFull()) return;
        std::vector<char> b;
        size_t est = sizeof(ClusterLinkRec) + unmatched.size() * 4;
        for (const auto& s : support) est += 4 + s.size() * 4;
        b.reserve(est);
        ClusterLinkRec h;
        h.tsNs = tsNs;
        h.nTracks = uint16_t(support.size());
        h.nUnmatched = uint16_t(unmatched.size());
        h.nCams = uint8_t(nCams < 0 ? 0 : (nCams > 255 ? 255 : nCams));
        io::put(b, h);
        for (const auto& s : support) {
            io::put(b, uint8_t(s.size() > 255 ? 255 : s.size()));
            io::put(b, uint8_t(0)); io::put(b, uint16_t(0));
            for (const auto& p : s) {
                io::put(b, uint8_t(p.first  < 0 ? 255 : (p.first  > 255 ? 255 : p.first)));
                io::put(b, uint8_t(0));
                io::put(b, uint16_t(p.second < 0 ? 65535 : p.second));
            }
        }
        for (const auto& p : unmatched) {
            io::put(b, uint8_t(p.first  < 0 ? 255 : (p.first  > 255 ? 255 : p.first)));
            io::put(b, uint8_t(0));
            io::put(b, uint16_t(p.second < 0 ? 65535 : p.second));
        }
        push(ChunkType::ClusterLink, std::move(b));
    }

    // 全量指派矩阵。(nCand+1) × 21 的 log 概率。
    // 【只在 Paranoid 档】120fps 下约 200KB/s。但查"为什么这个点被判成鬼点"
    // 时，top-3 是不够的：需要看它在全部 21 个类上的分布，才知道是模型
    // 没主意（分布平）还是被别的点抢走了（次高很接近）。
    void writeAssignFull(int64_t tsNs, int nCand, int nClasses, const float* logAssign) {
        if (!run_.load() || !wantsParanoid() || !logAssign) return;
        const size_t cnt = size_t(nCand) * size_t(nClasses);
        std::vector<char> b;
        b.reserve(16 + cnt * 4);
        io::put(b, tsNs);
        io::put(b, uint16_t(nCand));
        io::put(b, uint16_t(nClasses));
        io::putBytes(b, logAssign, cnt * sizeof(float));
        push(ChunkType::AssignFull, std::move(b));
    }

    // 送进 ONNX 的归一化输入。【只在 Paranoid 档】
    // 有了它，离线可以【直接把同样的张量喂给 onnxruntime】重跑一次，
    // 复现就不再依赖"我把归一化重新实现对了"这个假设。
    void writeNetInput(int64_t tsNs, const std::vector<float>& cand,
                       const std::vector<float>& tmpl, const std::vector<float>& prev) {
        if (!run_.load() || !wantsParanoid()) return;
        std::vector<char> b;
        b.reserve(24 + (cand.size() + tmpl.size() + prev.size()) * 4);
        io::put(b, tsNs);
        io::put(b, uint32_t(cand.size()));
        io::put(b, uint32_t(tmpl.size()));
        io::put(b, uint32_t(prev.size()));
        if (!cand.empty()) io::putBytes(b, cand.data(), cand.size() * 4);
        if (!tmpl.empty()) io::putBytes(b, tmpl.data(), tmpl.size() * 4);
        if (!prev.empty()) io::putBytes(b, prev.data(), prev.size() * 4);
        push(ChunkType::NetInput, std::move(b));
    }

    // 模板指纹。【故意只覆盖几何，不含 reason/wallNs】——
    // 否则每次快照指纹都不一样，"模板变没变"就判不出来了，而那正是它唯一的用途。
    static uint32_t fingerprintTemplate(const TemplateSnapRec& r) {
        uint32_t h = 2166136261u;
        auto mix = [&h](const void* p, size_t n) {
            const unsigned char* b = static_cast<const unsigned char*>(p);
            for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 16777619u; }
        };
        mix(r.backMarkersMm, sizeof(r.backMarkersMm));
        mix(r.markersMm,     sizeof(r.markersMm));
        mix(r.normalized,    sizeof(r.normalized));
        mix(r.anchorsMm,     sizeof(r.anchorsMm));
        mix(r.boneLenMm,     sizeof(r.boneLenMm));
        mix(r.dipCoupling,   sizeof(r.dipCoupling));
        mix(&r.isRight,      sizeof(r.isRight));
        return h;
    }

    static uint32_t fnv1a(const void* p, size_t n) {
        uint32_t h = 2166136261u;
        const unsigned char* b = static_cast<const unsigned char*>(p);
        for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 16777619u; }
        return h;
    }

    // 录制队列当前深度。写进 TimingRec，用来判断"丢块是不是快发生了"。
    size_t queueDepth() const {
        std::lock_guard<std::mutex> lk(m_);
        return q_.size();
    }

    // worker 每秒生成一份约 2.5KB 的运行时状态 JSON（hm20 全部配置 + 自标定
    // 全部产物）。它一直只给面板看，从没落盘 —— 而"这段数据是在什么配置下
    // 录的"恰恰是事后归因的第一步。1Hz 下体积可以忽略。
    void writeStateJson(int64_t tsNs, const std::string& json) {
        if (!run_.load()) return;
        std::vector<char> b;
        b.reserve(16 + json.size());
        io::put(b, tsNs);
        io::put(b, uint32_t(json.size()));
        io::putBytes(b, json.data(), json.size());
        push(ChunkType::StateJson, std::move(b));
    }

    // kind: 0=普通标记 1=段开始 2=段结束 3=坏帧（操作者自己判断这段废了）
    void writeMark(int64_t tsNs, uint8_t kind, const std::string& text) {
        if (!run_.load()) return;
        std::vector<char> b;
        io::put(b, tsNs);
        io::put(b, kind);
        io::put(b, uint16_t(text.size()));
        io::putBytes(b, text.data(), text.size());
        push(ChunkType::Mark, std::move(b));
    }

    void writeParamDelta(int64_t tsNs, const std::string& json) {
        if (!run_.load()) return;
        std::vector<char> b;
        io::put(b, tsNs);
        io::put(b, uint32_t(json.size()));
        io::putBytes(b, json.data(), json.size());
        push(ChunkType::ParamDelta, std::move(b));
    }

private:
    void push(ChunkType t, std::vector<char>&& payload) {
        std::vector<char> chunk;
        chunk.reserve(5 + payload.size());
        io::put(chunk, uint8_t(t));
        io::put(chunk, uint32_t(payload.size()));
        io::putBytes(chunk, payload.data(), payload.size());
        const size_t nb = chunk.size();
        const size_t ti = size_t(uint8_t(t)) & 63u;
        {
            std::lock_guard<std::mutex> lk(m_);
            // 【满了就丢，绝不阻塞】采集线程被文件 IO 卡住 = 丢帧 = 数据本身
            // 就废了。丢块会被计数并写进块 35，分析时能看见。
            //
            // 【但丢谁是有讲究的】按到达顺序丢，等于让"一次会话只有 3 份、
            // 丢了得重标"的 RomCalib 和"下一帧还有一份"的 2D 光斑同概率牺牲。
            // 高频大块先让路，门限越关键越高。见 dropTier 上方的说明。
            static constexpr size_t kThresh[3] = {
                kMaxQueue,                  // Critical：撑到最后一刻
                kMaxQueue * 85 / 100,       // Diagnostic
                kMaxQueue * 60 / 100,       // Bulk：先让路
            };
            if (q_.size() >= kThresh[size_t(dropTier(t))]) {
                dropped_.fetch_add(1);
                stDropped_[ti].fetch_add(1, std::memory_order_relaxed);
                return;
            }
            q_.push_back(std::move(chunk));
            pushed_.fetch_add(1);
            stPushed_[ti].fetch_add(1, std::memory_order_relaxed);
            stBytes_[ti].fetch_add(nb, std::memory_order_relaxed);
            // 峰值。【它比"丢了多少"更早给出预警】队列长期在 50% 以上说明
            // 盘已经跟不上，只是还没丢到；等丢块出现时数据已经有洞了。
            if (q_.size() > queuePeak_) queuePeak_ = q_.size();
        }
        cv_.notify_one();
    }

    void drain() {
        std::vector<std::vector<char>> batch;
        while (true) {
            {
                std::unique_lock<std::mutex> lk(m_);
                cv_.wait(lk, [this] { return !q_.empty() || !run_.load(); });
                if (q_.empty() && !run_.load()) break;
                batch.clear();
                while (!q_.empty() && batch.size() < 256) {
                    batch.push_back(std::move(q_.front()));
                    q_.pop_front();
                }
            }
            // 【拼成一次 write，不是一块一次】v5 每帧的块数从 6 涨到 14 左右，
            // 120fps 下就是 1700 次 write/s。每次 write 都是一次系统调用 +
            // 一次 filebuf 检查，在机械盘/网络盘上这个开销会直接反压到队列，
            // 表现为丢块 —— 而丢块意味着数据有洞，会把分析结论带偏却不留痕迹。
            // 先拼进一块连续内存再一次写出，系统调用数降到 1/256。
            coalesce_.clear();
            size_t total = 0;
            for (const auto& c : batch) total += c.size();
            coalesce_.reserve(total);
            for (const auto& c : batch)
                coalesce_.insert(coalesce_.end(), c.begin(), c.end());
            if (!coalesce_.empty()) {
                out_.write(coalesce_.data(), std::streamsize(coalesce_.size()));
                bytes_.fetch_add(coalesce_.size());
                chunks_.fetch_add(batch.size());
            }
        }
        out_.flush();
    }

    // 【v5 抬高上限】块数翻了一倍多，原来的 4096 只够缓冲约 2.4 秒；
    // 一次几百毫秒的磁盘抖动就会开始丢块。16384 约合 9 秒，按每块平均
    // 1KB 算约 16MB 内存 —— 对一台跑得动 8 路相机的机器可以忽略。
    static constexpr size_t kMaxQueue = 16384;
    std::ofstream out_;
    std::string path_;
    std::thread th_;
    std::vector<char> coalesce_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    std::deque<std::vector<char>> q_;
    std::atomic<bool> run_{false};
    std::atomic<uint64_t> bytes_{0}, chunks_{0}, dropped_{0};
    // 逐块类型的计数。【为什么要逐类型】"这一块不在文件里"有三种成因
    // （没触发 / 被档位关掉 / 被丢了），三者的下一步动作毫无共同点，
    // 而只有总数的话它们无法区分。见 ChunkStatsRec 的说明。
    // 下标就是 ChunkType 的数值，64 是块类型上限（类型字段是 1 字节，
    // 实际用到 35，留够余量）。
    std::array<std::atomic<uint64_t>, 64> stPushed_{};
    std::array<std::atomic<uint64_t>, 64> stDropped_{};
    std::array<std::atomic<uint64_t>, 64> stBytes_{};
    size_t queuePeak_ = 0;   // 受 m_ 保护
    // 入队计数（主线程）。chunks_ 是落盘计数（写盘线程），两者在 stop() 那一刻
    // 差着整个队列的深度，不能混用。
    std::atomic<uint64_t> pushed_{0};
    // 【默认 Full】理由见 Detail 的说明：录制是为了排查，而排查时最贵的
    // 是"录完发现关键那项没开"。
    std::atomic<int> detail_{int(Detail::Full)};
};

}  // namespace pcrec
}  // namespace mocap
