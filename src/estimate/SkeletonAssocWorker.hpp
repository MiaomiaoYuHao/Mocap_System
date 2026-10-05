#pragma once
// ---------------------------------------------------------------------------
// 骨架关联独立工作线程——照 detect/DetectWorker.hpp / estimate/SmootherWorker.hpp
// 那套"独立线程 + 忙时新数据直接跳过(不排队、不积压)"的模式抄的。
//
// 为什么需要这个文件：Hm20SkeletonAssociator::process() 本身不慢(ONNX是<26点
// 的小模型)，但一旦 onnxruntime 没装好导致某次 Session::Run 意外卡顿，或者
// 未来模型换大了，这个调用直接堵在 PointCloudTestDialog 的渲染/追踪帧回调里，
// 会把整个点云面板一起拖慢甚至卡死——而点云本身的追踪跟AI骨架是两件独立的
// 事，不该互相拖累。丢到独立线程后：忙的时候新一帧直接跳过(不排队)，点云
// 面板永远只等自己的追踪流水线，骨架叠加慢/卡只影响骨架自己多久刷新一次。
//
// 【本次改动：迁到 hm20 契约】
// 原实现用的是上一代的 mocap::HandSkeletonAssociator / SkeletonTemplate /
// ISkeletonInferenceBackend / result.joints，这几个符号在当前的
// HandSkeletonAssociator.hpp 里【都已经不存在】(那个头文件已经换成
// mocap::hm20 那一代)，所以改之前这个文件是编译不过的。现在全部对到：
//     mocap::hm20::Hm20SkeletonAssociator
//     mocap::hm20::Hm20OnnxBackend      (6 输入 / 21 类)
//     mocap::hm20::Hm20Template
//     result.markers[20]                (原 result.joints)
// ---------------------------------------------------------------------------
#include <QVector3D>
#include <QVector>
#include <algorithm>
#include <vector>
#include "estimate/HandSkeletonAssociator.hpp"
#include "estimate/Hm20OnnxBackend.hpp"
#include "estimate/Hm20AutoCalib.hpp"
#include "estimate/Hm20PoseFilter.hpp"
#include "estimate/Hm20IkRefiner.hpp"
#include "estimate/Hm20JointAngles.hpp"
#include "estimate/RomCalibration.hpp"
// 【编译修复】HandVec3 定义在 hand/HandPose.hpp，不在上面两个头文件里。
// 漏了这行的症状：SkeletonCandVec 那一行报 "'HandVec3' was not declared"，
// 然后连锁出十几条 template argument invalid / qRegisterMetaType 失败，
// 但根因只有这一个。
#include "hand/HandPose.hpp"
#include <QObject>
#include <QElapsedTimer>
#include <QFileInfo>
#include <mutex>
#include <cstdio>
#include <QMetaType>
#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <utility>

Q_DECLARE_METATYPE(mocap::hm20::SkeletonFrameResult)

namespace mocap {

// 【bug修复】Q_ARG/Q_DECLARE_METATYPE 都是宏，预处理器按"裸逗号"切分参数——
// std::vector<std::pair<int,HandVec3>> 里 pair<int,HandVec3> 那个逗号会被
// 误当成宏的第3个实参，报"passed 3 arguments but takes just 2"。用一个不
// 含逗号的别名包起来，宏看到的只是一个不带逗号的类型名，问题从根上消失。
//
// 注意：HandVec3(hand/HandPose.hpp) 和 hm20::Vec3 都是 std::array<double,3>，
// 底层【类型】相同，所以 Hm20SkeletonAssociator::process() 直接吃这个 vector。
// 但【名字】不通用：HandVec3 必须 include hand/HandPose.hpp 才可见 —— 上一代
// 的 HandSkeletonAssociator.hpp 里带过这个名字，换成 mocap::hm20 那一代之后
// 就没有了，所以本文件顶部显式加了那个 include。
using SkeletonCandVec = std::vector<std::pair<int, HandVec3>>;

// 一次处理结果 + 诊断信息，方便UI直接判断"AI这一步到底有没有生效、卡在哪"，
// 不需要再去猜。
struct SkeletonAssocDiag {
    bool backendReady = false;      // onnxruntime有没有真正启用
    bool palmOk = false;            // 这一帧整体解出来没有(result.valid)
    int observedJoints = 0;         // 15个手指标签里，这一帧真被candidate认领的个数
    int predictedJoints = 0;        // 其余用网络 pos 头兜底的个数(=15-observedJoints)
    double latencyMs = 0.0;         // 本次process()总耗时(含ONNX推理)
    int candidateCount = 0;         // 喂进去的候选点总数
    std::string stageMessage;       // 失败/降级时具体卡在哪一步

    // 【新增】hm20 特有的诊断项，上一代没有对应字段
    bool wristPoseValid = false;    // 手背可见点>=3、腕部位姿是本帧解出来的；
                                    // false 表示沿用上一帧，segQuat[0] 不可信
    int numGhost = 0;               // 被判为 dustbin 的候选点数
    double dorsumRmseMm = -1.0;     // 手背 Kabsch 残差，<0 表示没解
    // ---- 手背刚体求解 ----
    int    dorsumInliers    = 0;    // 本帧刚体解用了几个内点(3..5)
    double dorsumMarginMm   = -1.0; // 最优解与次优【不同标签】解的残差差
    int    dorsumReason     = 1;    // 0=ok 1=候选不足 2=无可行解 3=歧义拒解 4=残差超限
    bool   dorsumByHistory  = false;// 裕度不够、靠位姿连续性拍板的
    // 手背模板"最优错解"的 Kabsch 残差。标定一次算一次，不随帧变。
    // 【这是这一屏最该看的数】<3mm = 贴点布局近似共面+近似反序对称，
    // 几何判据本身分不清正解和反序，任何算法都会来回翻 —— 该去重贴点，
    // 而不是继续在软件里调参数。
    double dorsumSelfAmbMm  = -1.0;
    // 连续"解不好"的帧数 / 本帧看门狗动作(0无 1松锁 2退模板) / 模板被改了多少
    int    dorsumBadStreak  = 0;
    int    dorsumRelock     = 0;
    double dorsumTmplDriftMm = 0.0;
    // 自标定的模板更新被挡下时，这里是那份被拒模板的 bundleRmse（-1=没挡）。
    // 【面板上要能看见】否则"为什么标定了不生效"会变成又一轮盲查。
    double tmplRejectedRmse = -1.0;
    double tmplAppliedRmse  = -1.0;
    // 束调整累计次数和最近一次耗时。【下一段录制就靠这两个数判断退避有没有生效】
    int    bundleRuns    = 0;
    double bundleLastMs  = -1.0;
    // 实际加载的模型文件名（不含路径）。切换失败时必须能看见 ——
    // 否则用户会以为自己在跑高速模式。
    QString loadedModelName;
    int    netThumbSegs     = 0;    // 本帧有几段拇指用了网络 seg_rot6d
    // ---- 贴点重捕 ----
    int    dorsumRepaired   = 0;    // 本帧就地修好了哪个手背槽位（0=没修，1..5=槽位+1）
    double dorsumRepairMoveMm = -1.0;
    // ---- 拇指遮挡跟随 ----
    // 【这两个是判断"跟随到底工作没有"的唯一依据】
    // predRad 是当帧给出的 IP 角；driftRad 是相对遮挡入口值累计跟了多少。
    // drift 恒为 0 = 跟随没生效（多半是 chainContinue 没打开）。
    double thumbIpPredRad   = -1.0;
    double thumbIpDriftRad  = 0.0;
    bool   thumbTipOccluded = false;
    // ---- 网络段常数的标定结果（一次性）----
    bool   netSegSolved     = false;
    bool   netSegOk         = false;
    std::string netSegWhy   = "还没跑";
    double netSegCSpread    = -1.0;
    // K 相对单位阵的转角。【交叉验证点】probe_thumb_axis.py 从模型的 seg_rot6d
    // 量到拇指铰链轴缺口 73°；这个是从你手上的实测几何反解的同一个量。
    // 落在 70~90° 才说明两条互不依赖的路对上了，才敢开 thumbSegFromNet。
    std::array<double, 3> netSegKOffset{{-1, -1, -1}};
    bool pentagonOk = false;        // 手背五边形自交校验

    // ---- 自标定 ----
    int         autoStage = 0;          // hm20::AutoCalibStage
    double      autoProgress = 0.0;     // 0..1
    std::string autoText;               // 给操作者看的一行提示
    std::string autoReject;             // 本帧被拒的原因（空=没拒）
    bool        autoTemplateReady = false;
    bool        autoIkUsable = false;   // 参数已收敛，IK 可以开
    bool        autoHandednessKnown = false;
    bool        autoIsRight = true;              // 【生效】的手性
    bool        autoIsRightDetected = true;      // 自标定【推断】的手性
    bool        autoHandednessConflict = false;  // 推断与面板设置不符
    double      autoHandSignMm = 0.0;            // 判据本身：离 4mm 阈值多远
    bool        autoMirrorSuspect = false;       // 冻结的模板疑似被镜像
    bool        jointMirrorActive = false;       // 关节角输出已按检测手性纠正符号
    int         thumbFixed = 0;                  // 本帧回正了几个拇指预测点
    int         thumbFixSkip = 0;                // 没执行的原因，见 SkeletonFrameResult
    int         ikFilled = 0;                    // 遮挡点里走 IK 摆出来的个数
    int         ikFallback = 0;                  // 退回网络 pos 头预测的个数
    int         chainContinued = 0;              // 顺着可见近端闭式续解出来的个数
    // 情形B（中远节都遮）里掌心相对耦合推出去多少度，逐指。
    // 【必须能在面板上看到】它恒为 0 有两种完全不同的原因：耦合关着，
    // 或者近节相对手掌根本没动。不上报的话"手指弯下去模型不跟"分不出是哪种。
    double      chainCoupledDeg[5] = {0,0,0,0,0};

    // ---- v7 姿态版 ----
    double      dorsumGeoMarginMm = -1.0;        // 几何重定的裕度(次解-正解)
    int         dorsumTrackN = 0;                // 手背轨迹累积帧数
    int         dorsumTrackFixed = 0;            // 本帧被轨迹投票纠正的手背点数
    bool        hasAiPose = false;               // 模型有没有姿态头(v6 没有)
    double      aiJointDeg[16] = {0};            // 已转成 M3DS 的 16 维，度
    bool        hasAiHand = false;
    // aiHandKnown=false 时 aiHandIsRight 【没有意义】，显示端要报"未定"。
    bool        aiHandKnown = false;
    bool        aiHandIsRight = true;
    double      aiHandConf = 0.0;
    bool        aiHandLocked = false;
    double      aiPoseConf[5] = {0,0,0,0,0};     // 逐指置信度(已过 sigmoid)
    int         occludedHeld = 0;                // 因发散/只信IK 被改成保持的个数
    int         calibRejected = 0;               // 自标定产物被验收闸门挡下：1=刚性σ不过
    // 【逐指】残差，跟上面那个标量中位数不同：中位数只在有指生效时才有意义，
    // 而"哪根没生效、它的残差是多少"恰恰是排查退档的关键。<0 = 该指没解。
    double      ikRmsePerFinger[5] = {-1,-1,-1,-1,-1};
    // anchor 拟合质量：解没解出来 / 球面残差 / 角度覆盖 / 样本数
    bool        anchorFitted[5]   = {false,false,false,false,false};
    double      anchorResidMm[5]  = {-1,-1,-1,-1,-1};
    double      anchorSpreadDeg[5]= {0,0,0,0,0};
    int         anchorSamples[5]  = {0,0,0,0,0};
    bool        staticSeeded[5]   = {false,false,false,false,false};
    double      staticSeedDevMm[5]= {-1,-1,-1,-1,-1};
    // |anchor-pp| 跨帧标准差：anchor 对不对的【预测型】判据，比拟合残差可信
    double      anchorRigidStd[5] = {-1,-1,-1,-1,-1};
    bool        autoDorsumReordered = false;     // 手背编号被规范化重排过
    bool        autoDorsumWouldReorder = false;  // 规范化本来会重排（已被默认关掉）
    double      autoBundleRmseMm = -1.0;
    int         autoAttempts = 0;
    qint64      frameTsNs = -1;
    int         jointValidFingers = 0;   // 本帧关节角是新算的手指数（0..5）
    bool        mcpValid = false;        // 掌指关节角是新算的（false=腕部系失效，只更新了PIP）
    double      thumbCoverage = 1.0;    // 拇指可辨识性覆盖度 0..1
    std::string thumbHint;              // 空 = 够了，不用提示
    double      thumbAxialK = 0.576;
    // 第一掌骨常数解剖旋前(rad)。0 = 还没标出来/曲线太平没敢写。
    double      thumbPronation0 = 0.0;
    bool        thumbPronationFitted = false;
    // sqrt(最差档均方/最优档均方)。<2 = 这轮 shot 里拇指没怎么动，量的是噪声。
    double      thumbPronationContrast = 0.0;
    // ---- IK ----
    bool        ikActive = false;       // 本帧真的有手指走了 IK
    int         ikFingerCount = 0;
    double      ikRmseMm = -1.0;        // 生效手指的中位残差
    // ---- 滤波 ----
    bool        filterActive = false;

    // =======================================================================
    // 【本次新增】关节角输出全链路 + 手性证据链 + 逐帧生效开关
    //
    // 为什么加在这里而不是新开一个信号：这个结构已经每帧随 resultReady 送到
    // 面板了，而录制器就在面板里。多开一路信号意味着两路可能对不上帧 ——
    // 而"这一帧的角度"和"这一帧的手性"必须是同一帧的，否则对照本身就是错的。
    // =======================================================================

    // ---- 关节角四级流水（弧度）----
    // 【必须四级都留】"握拳输出像张开"这一个症状，四级里每一级都能单独造成，
    // 而修法完全不同。只留最后一级 = 这四种永远分不开。
    std::array<double, 16> qSolve{};    // ① solveJointAngles 出来的原始角
    std::array<double, 16> qSmooth{};   // ② 预测段角度平滑之后
    std::array<double, 16> qRom{};      // ③ ROM 映射之后
    std::array<double, 16> qOut{};      // ④ 速度限幅之后 = 实际进 UDP 的
    std::array<double, 16> romLo{};     // ROM 标定区间。它是映射的分母，
    std::array<double, 16> romHi{};     // 分母塌了输出必然是平的（最常见的失效）
    // ---- v7：ROM v2 + 解算内部量 ----
    // 【为什么要整份带过来而不是只带几个标量】录制器在 UI 线程里，
    // 而这些量只有 worker 线程算得出来。resultReady 已经每帧过去一趟了，
    // 再开一路信号意味着两路可能对不上帧 —— 而"这一帧的角"和"这一帧的轴"
    // 必须是同一帧的，否则对照本身就是错的。
    hm20::JointSolveDebug   jointDbg{};
    std::array<uint8_t, 16> romReject{};   // 本帧逐维的采样判决，见 rom::RejectCode
    std::array<int8_t, 16>  romSign{};     // +1/-1，v2 判出来的方向
    std::array<uint8_t, 16> romStatus{};   // 见 rom::DofStatus
    // 标定结果全量快照。reason>=0 时说明这一帧该落盘（块 31）。
    hm20::rom::CalibResult  romResult{};
    int  romSnapReason = -1;   // <0=不用写 0=开始 1=标定中(1Hz) 2=结束
    bool romAccumulated = false;
    // ---- v8 ----
    // 映射器本帧的逐维内部量（走了哪条分支 / 目标行程 / 钳位 / 冻结计数）。
    // 【这是"标定完输出就不动了"唯一能定位的地方】通向"输出是一个不动的数"
    // 的路有四条，产生的输出完全一样，而修法毫无共同点。见块 33 的注释。
    hm20::rom::MapDebug     romMapDbg{};
    // Mapper 的 holdOnDegenerate 开关。【关掉它冻结路径②就整个没了】——
    // 所以"这一帧它是开着的吗"是判断冻结成因的前提条件，不能只靠猜默认值。
    bool                    romMapHoldOn = true;
    // 标定配置快照。跟 romResult 同一时机填 —— 结果和判据必须在同一条记录里，
    // 否则"这份结果是在哪组阈值下算出来的"还是要靠时间戳去猜。
    hm20::rom::Config       romCfg{};
    // 角度后处理链的隐藏状态（块 34）。
    // 【为什么这两份也必须整份带过来】平滑器的 off[] 是【直接加到输出上的
    // 逐维常量偏置】，最大 0.7rad(≈40°)，而它在系统里没有第二份 ——
    // apply() 就地算完就返回，外面只看得到 qSmooth。
    // 于是"解算本来就偏了 40°"和"解算对的、被没还清的恢复补偿顶着"
    // 在文件里长得完全一样，而两者的修法毫无关系。
    hm20::PredictedAngleSmoother::Debug angSmoothDbg{};
    hm20::RateLimiter::Debug            rateDbg{};
    std::array<double, 64> quatWorld{}; // 16 段世界系四元数
    std::array<double, 64> quatLocal{}; // 16 段相对父节点四元数。【判"手指弯没弯"
                                        // 要看这个】—— 世界系里整手朝向和屈曲是混的
    double  romCoverage = -1.0;
    double  romAbdCoverage = -1.0;
    double  angSmoothAlpha = -1.0;
    // 【配置也要逐帧带】decay 和 maxOffsetRad 决定了 off 还得多久、能顶多高，
    // 而 off 是加在输出上的。只记 off 不记这两个，"它为什么还不清零"就没法答。
    double  angSmoothDecay = -1.0;
    double  angSmoothMaxOffsetRad = -1.0;
    double  rateLimitRadPerSec = -1.0;
    double  dtSec = -1.0;
    double  maxRateClipRad = 0.0;       // 本帧被限幅削掉最多的那一维削了多少
    double  maxStageDeltaRad = 0.0;     // max|qOut - qSolve|
    int     maxRateClipIdx = -1;
    int     maxStageDeltaIdx = -1;
    int     romSamples = 0;
    int     wristStaleFrames = 0;       // 腕部位姿已沿用了多少帧（0=本帧新解）
    // 【这两个是关节角自己的有效位，不是 IK 的】
    // result.fingerIkValid 说的是"这根指本帧走了 IK 且残差合格"，而
    // ja_.fingerValid 说的是"这根指的角度本帧是新算的（不是保持上一帧）"。
    // 两者常常不一致：没走 IK 但几何解出来了，角度照样是新的。
    // 混用的后果是把"角度在更新"误判成"角度冻住了"，方向正好反。
    std::array<bool, 5> jointFingerValid{};
    std::array<bool, 5> jointFingerPredicted{};
    bool    romReady = false;
    bool    romLearningOn = false;
    bool    rateLimitOn = false;
    bool    angSmoothOn = false;
    bool    quatOutOn = false;
    bool    jointMirrorFlag = false;

    // ---- 手性证据链（v3 只有 4 个 bool，漏掉的恰好是最要命的两处）----
    double  handSignLatMm = 0.0;      // 横向判据的【原始值】，不是结论
    double  handednessMinMm = 4.0;    // 阈值。判据离它多远 = 这个结论有多可信
    double  aiHandLogit = 0.0;        // 模型原始 logit
    double  aiHandConfVal = 0.0;
    double  thumbRollSignedRad = 0.0; // 拇指 roll 实际下发值（已按手性带符号）
    int     handSignN = 0;
    int     geoHandSign = 0;          // 关联器几何锁定：0=未锁 +1=右 -1=左
    bool    handednessAgree = true;   // 拇指判据与横向判据是否一致
    bool    tmplIsRight = true;       // 【送进网络/建模板用的那个】——漏掉它
                                      // 就会出现"四个 bool 全正常而结果是镜像的"
    bool    ikIsRight = true;         // 【IK 用的】——跟上面不一致时拇指单独歪
    bool    cfgApplyHandedness = false;
    bool    cfgDetectHandedness = true;
    bool    cfgJointMirrorAuto = true;
    bool    autoCalibOn = false;

    // ---- 逐帧生效开关（位定义见 pcrec::RunFlagBit）----
    // 【为什么不能只靠文件头快照】头是按下按钮那一刻拼的，而这些开关中途会变：
    // 用户会点、自标定会改、dirty 标志会延迟一帧生效。"这功能到底生效没有"
    // 是排查时问得最多的问题，只有逐帧记才答得准。
    quint32 runFlags = 0;
    double  thumbRollUiDeg = 0.0;
    double  thumbPronation0Rad = 0.0;
    double  occludedJumpGateMm = -1.0;
    // 1Hz 刷新的全量运行时状态 JSON。非空时说明这一帧刚好刷新过，该落盘。
    std::string stateJson;

    // =======================================================================
    // v5 新增。原则跟 v4 一样：记的不是"结果是什么"，而是"结果是怎么来的"，
    // 而且每一级的输入和输出分开记。v4 把这条原则用在了关节角那一路，
    // 这一版推到其余每一路。
    // =======================================================================

    // ---- 滤波：前后 + 内部 ----
    // 【最该有的一项】Hm20PoseFilter::apply() 是就地覆写 result 的，
    // 滤波前的位置在系统里没有第二份。于是"几何本来就解错了"和
    // "解对了被平滑吃平了"在录制文件里完全一样，而两者修法相反。
    hm20::Hm20PoseFilter::FrameDebug filt{};

    // ---- IK 求解器内部 ----
    // 遮挡 marker 的位置是这些角 FK 出来的。位置不对时往上查一级，查的就是它们。
    hm20::IkFrameInfo ikInfo{};

    // ---- 位置第 5 级：滤波之后 ----
    // 0..4 级在 SkeletonFrameResult::stagePos 里（关联器填的），
    // 第 5 级只有 worker 知道，因为滤波是在这里做的。
    std::array<hm20::Vec3, hm20::kNumMarkers> stage5Pos{};
    bool stage5Valid = false;

    // ---- 模板快照 ----
    // 【为什么要逐次落盘】文件头那一份是按下录制按钮那一刻的模板，而自标定
    // 会在录制中途【热替换】worker 的 tmpl_（applyAutoCalib 置 tmplDirty_）。
    // 于是"第 N 帧真正送进网络的模板长什么样"——手性问题的核心证据——查不到。
    // 只在变化时和每秒一次填，平时 tmplSnapReason < 0 表示不用写。
    hm20::Hm20Template tmplSnapshot{};
    int  tmplSnapReason = -1;      // <0=不用写 0=开始 1=热替换 2=手性变更 3=自标定提交 4=周期
    bool tmplFromAutoCalib = false;
    double tmplBundleRmseMm = -1.0;
    double tmplSelfAmbMm = -1.0;
    std::array<std::array<double, 3>, 5> tmplBoneLenMm{};
    std::array<double, 5> tmplDipCoupling{};
    std::array<bool, 5> tmplFingerCalibrated{};

    // ---- 事件流 ----
    // 【为什么标量不够】排查时问的是"它【什么时候】、【因为什么】变成这样的"。
    // applyHandedness 触发的镜像是一个瞬间事件，之后所有帧都"稳定地不对"，
    // 没有事件流的话那个瞬间在文件里没有任何标记。
    struct Event {
        int code = 0;          // 见 pcrec 里的事件码表
        int severity = 0;      // 0=信息 1=注意 2=异常
        double valueA = 0.0, valueB = 0.0;
        int intA = 0, intB = 0;
        std::string text;
    };
    std::vector<Event> events;

    // ---- 分阶段耗时 ----
    // 【dt 是滤波/限幅/外推三处的分母】它不可信时这三处算出来的全不可信，
    // 而输出看起来完全正常，只是慢了、飘了。查到这一步之前人会先怀疑
    // 参数、标定、模型，把时间花在错的地方。
    double tInferMs = -1.0;
    double tAssocMs = -1.0;
    double tIkMs = -1.0;
    double tFilterMs = -1.0;
    double tJointMs = -1.0;
    int    skelProcessed = 0;   // worker 累计处理的帧
    int    skelSkipped = 0;     // 累计跳过的帧（忙时直接丢，设计如此，但要能看见）

    // ---- 全量配置转储 ----
    // 【这一项是"离线调参"的前提】要离线把某一级的参数调到最优，必须同时有
    // 那一级的【输入】【参数】【输出】。输入输出上面各块都有了，参数就是这里。
    // stateJson 已经带了 hm20 和自标定，但滤波/IK/ROM/限幅/角度平滑的配置
    // 一个都没有 —— 而它们恰恰是最需要调的几个。
    // 非空 = 这一帧刚好刷新过（1Hz + 配置变更时立刻刷）。
    std::string configJson;
};

class SkeletonAssocWorker : public QObject {
    Q_OBJECT
public:
    // 【重要修复——不要改回只用busy判断】主线程决定要不要转发这一帧前读
    // 这个原子计数(无锁、跨线程安全)。
    //
    // 为什么不是像 DetectWorker::busy 那样只用一个"正在处理中"的bool：
    // busy 只有在 worker 线程真正进入 processFrame 之后才会置true，而
    // QMetaObject::invokeMethod(QueuedConnection) 只是把调用"投递"进
    // worker 线程的事件队列、立刻返回。于是存在一个窗口：主线程投递了
    // 第1帧、worker线程还没被调度到、busy仍是false，主线程下一帧再来看
    // busy 还是false，于是又投递一次……worker线程越是被别的线程挤占
    // (相机多、CPU吃紧)，这个窗口越长、堆进队列的帧越多，每一条还都带
    // 着一份候选点向量的拷贝。表现就是"越跑越滞后 + 内存缓慢上涨"。
    // pending 在【主线程投递的那一刻】就自增，worker处理完才自减，所以
    // 队列里永远最多一条，从根上杜绝堆积。
    std::atomic<int> pending{0};

    explicit SkeletonAssocWorker(QObject* parent = nullptr) : QObject(parent) {}

    // 主线程在投递前调用：返回true表示"队列是空的，可以投"，同时把名额
    // 占掉。返回false表示上一帧还没处理完/还没被取走，这一帧直接丢弃
    // (不排队、不积压，跟 DetectWorker/SmootherWorker 一个策略)。
    bool tryAcquireSlot() {
        int expected = 0;
        if (pending.compare_exchange_strong(expected, 1)) return true;
        // 【跳帧必须计数】worker 忙时新帧直接丢是设计如此，但丢了多少一定要
        // 能看见：跳帧会让实际输出帧率远低于相机帧率，而下游收到的时间戳
        // 仍然连续，表现是"动作有延迟且发飘"—— 跟滤波调过头一模一样。
        // 不计数的话，这两件事永远分不开，会一直在滤波参数上白费功夫。
        skelSkipped_.fetch_add(1);
        return false;
    }

    // 主线程在 moveToThread + start() 之前调用一次：只传【模板和模型路径】，
    // 不传已经构造好的backend——onnxruntime的加载/建会话必须发生在worker
    // 线程里(见 ensureInitialized 的说明)。
    void configure(const hm20::Hm20Template& tmpl, const std::string& modelPath,
                   hm20::Hm20Config cfg = hm20::Hm20Config()) {
        tmpl_ = tmpl;
        // 【留一份标定值的副本】tmpl_ 会被 applyAutoCalib() 就地覆写，
        // 于是"退回标定值"就无处可退 —— 而重开面板之所以有效，正是因为它
        // 从 HandTemplateStore 重新读了一遍标定值再 configure()。
        // 手背复位要等价于重开面板，就必须能退回这一份。
        tmpl0_ = tmpl;
        tmpl0Valid_ = true;
        modelPath_ = modelPath;
        cfg_ = cfg;
    }

    // 面板构造时用标定值配好的 IK 参数，同样留一份 —— applyAutoCalib()
    // 会 setBackTemplate/setFingerParams 覆写它们。
    void snapshotIkParams(const std::array<hm20::Vec3, 5>& backMm,
                          const std::array<hm20::Vec3, 5>& anchorsMm,
                          const std::array<std::array<double, 3>, 5>& lengthsMm) {
        ik0Back_ = backMm; ik0Anchors_ = anchorsMm; ik0Lengths_ = lengthsMm;
        ik0Valid_ = true;
    }

    // -------------------------------------------------------------------------
    // 高速模式：换一份 INT8 量化模型 + 多线程。
    //
    // 【为什么做成"另一条路"而不是替换】INT8 在真实点云上标签一致率 98.7%，
    // 但那 1.3% 的分歧集中在【模型本来就没把握】的点上（被改变标签的点，
    // FP32 自己的置信度中位只有 0.370，没被改变的是 0.968）。遮挡多的时候
    // 分歧会变大（实测遮挡压力段 97.3%，逐帧最低 85.7%）。
    // 所以保留原模型这条路，让用户能一键切回来对照。
    //
    // 【线程数】Hm20OnnxBackend 的 intraOpThreads 默认是 1 —— 真机 30ms
    // 跟本机单核实测 26.2ms 几乎一致，说明一直在跑单线程。设成物理核数
    // 可能是免费的提速，而且不影响任何数值结果。
    //
    // 两者都要求重建 session，所以只能在下一次 ensureInitialized 时生效；
    // 这里设了标志，由 processOne 在合适的时机重建。
    void setFastModel(const std::string& fastPath, int intraOpThreads) {
        fastPath_ = fastPath;
        intraOpThreads_ = (intraOpThreads > 0) ? intraOpThreads : 1;
        modelDirty_.store(true);
    }
    void setFastMode(bool on) {
        if (fastMode_.exchange(on) != on) modelDirty_.store(true);
    }
    bool fastMode() const { return fastMode_.load(); }
    // 当前实际加载的是哪一份。UI 显示用 —— 切换失败（比如 INT8 文件不存在）
    // 时必须让用户看见，否则会以为切过去了。
    std::string loadedModel() const {
        std::lock_guard<std::mutex> lk(stateM_);
        return loadedModel_;
    }

    // IK 精修钩子。主线程在 moveToThread 之前调用一次即可。
    //
    // 【为什么要有它】computeSegmentQuats() 拿 marker[b]-marker[a] 当骨轴，
    // 而球贴在指节背侧、离骨轴 11~17mm 且相邻两节方位角不同 —— 连线必然偏离
    // 骨轴，HandSkeletonAssociator.hpp 里那句"这里剩下的 11.0° 中位偏差消不掉"
    // 说的就是这个。合成数据实测 10.2°中位/21.6°p90 -> 3.02°/8.08°，代价 0.12ms/帧。
    //
    // 【线程】Hm20IkRefiner 内部有时序先验(prevAngles_)，是有状态的，
    // 交出去之后主线程不要再碰它。ensureInitialized() 里才真正挂到 assoc_ 上。
    void setIkRefiner(std::shared_ptr<hm20::IHm20IkRefiner> ik) {
        pendingIk_ = std::move(ik);
        ikDirty_.store(true);
    }

    // 运行时开关：给 UI 做 IK 开/关对比用。关掉时退回网络 pos 头 + 两球连线。
    // ---- 自标定 / 滤波 的开关，主线程随时可调 ----
    // 手性。【标定向导没有这个选项，以前写死右手】写反的代价见
    // PointCloudTestDialog 里 isRightHand 那段注释。
    void setHandedness(bool isRight) {
        thumbRollDirty_.store(true);   // 旋前符号跟手性走，手性一变要重下发
        handIsRight_.store(isRight);
        handDirty_.store(true);
    }
    void setAutoCalibEnabled(bool on) { autoCalibOn_.store(on); }
    void setAutoCalibConfig(const hm20::AutoCalibConfig& c) {
        autoCfgPending_ = c; autoCfgDirty_.store(true);
    }
    void resetAutoCalib() { autoResetReq_.store(true); }
    void setFilterEnabled(bool on) { filterOn_.store(on); }
    // 是否额外发送分段四元数包（M3DQ）。默认开——它跟 M3DS 互不影响，
    // 下游不需要就按 magic 跳过。
    void setQuatOutputEnabled(bool on) { quatOutOn_.store(on); }
    // 【速度限幅开关】关掉 = 关节角直通，不做任何逐帧削峰。
    // 它跟时序滤波是两回事：滤波管"抖不抖"，限幅管"会不会突然甩一下"。
    // 但两者都会让输出滞后于真实动作，排查"跟手感"时必须能分别关掉。
    // 关掉时顺手 reset()，否则重新打开的第一帧会从很久以前的旧值开始爬。
    // 拇指 roll 偏置（弧度）。它是【建模参数】不是可测量 —— 绕骨轴自转在每节
    // 只有 1 颗球时不可观测，只能建模，所以必须让人能对着画面调。
    void setThumbRollOffset(double rad) { thumbRoll_.store(rad); thumbRollDirty_.store(true); }
    void setOccludedIkOnly(bool on) { ikOnly_.store(on); thumbRollDirty_.store(true); }
    // 手背标签用几何重定（穷举 120 种排列 + Kabsch）。仿真实测噪声 0.9mm 时
    // 定标签准确率 100%，而模型的手背逐点准确率只有 91~96%。
    void setDorsumGeoRelabel(bool on) { geoRelabel_.store(on); thumbRollDirty_.store(true); }
    // 手背刚体求解：在全部候选点里搜，取代 geoRelabel + tracklet 两条路。
    void setDorsumRigidSolve(bool on) { dorsumRigid_.store(on); thumbRollDirty_.store(true); }
    // 拇指段走网络 seg_rot6d 的两个开关。calib 只是攒样本、不影响出帧；
    // fromNet 才真正改输出，而且没标定出常数之前打开也不会生效。
    void setNetSegCalib(bool on)   { netSegCalib_.store(on);  thumbRollDirty_.store(true); }

    // -------------------------------------------------------------------------
    // 【给 .pcrec 文件头用】关联器 + 自标定的完整运行时状态，JSON。
    //
    // 走互斥量直接读，不走信号：录制是在【按下按钮的那一刻】写文件头的，
    // 而信号是异步的 —— 等它回来文件头早写完了。这里返回的是最近一次
    // 刷新的快照（每秒刷一次），对"这段录制是在什么设置下录的"这个用途足够。
    //
    // 【为什么必须有这个】现在文件头里一个 hm20 参数都没有：拇指旋前多少、
    // 手背走的哪条路、IK 残差门多少，事后全查不到。等于录了一堆无法归因的数据。
    // -------------------------------------------------------------------------
    std::string runtimeStateJson() const {
        std::lock_guard<std::mutex> lk(stateM_);
        return stateJson_;
    }
    void setThumbSegFromNet(bool on){ thumbSegNet_.store(on); thumbRollDirty_.store(true); }
    // 录制期间才开：每帧多拷 ~1.8KB（候选点+网络原始输出），
    // 常态运行不该背这个开销。见 SkeletonFrameResult::hasDebugStreams。
    void setCaptureDebugStreams(bool on) { capDebug_.store(on); thumbRollDirty_.store(true); }
    // 全量指派矩阵。跟上面分开是因为它是唯一体积显著的调试流（约 200KB/s），
    // 常态录制不该付这个代价，但查"这个点为什么被判成鬼点"时它是唯一依据。
    void setCaptureFullAssign(bool on) { capFullAssign_.store(on); thumbRollDirty_.store(true); }
    // 遮挡点的链式续解。关掉的话遮挡点全部退回网络 pos 头 —— 而 pos 头对拇指
    // 和四指的中远节都有"摆直"的先验，实测会把 IP 从 60° 塌到 8°。
    void setChainContinue(bool on) { chainCont_.store(on); thumbRollDirty_.store(true); }
    // 手背求解器复位。用户在面板上点一下，等价于原来"重开点云测试面板"，
    // 但不会连带丢掉自标定和 IK 的状态。
    void requestDorsumReset() { dorsumResetReq_.store(true); }
    // 拇指第一掌骨的常数解剖旋前(rad)。
    // 【为什么单独一条下发路径，不跟 thumbRoll_ 合并】thumbRollDirty_ 那条路上
    // 挂着 sgn = handIsRight ? +1 : -1 的翻号逻辑，而 thumbPronation0 在
    // markerFK 里【不该跟手性翻号】—— Hm20IkRefiner::refine() 内部已经把左手
    // 观测/anchor/backTemplate 一起镜像成右手系再解了。混在一起会造出
    // "面板切一下手性，拇指 IK 突然全废"这种坑。
    void setThumbPronation0(double rad) { thumbPron0_.store(rad); ikParamDirty_.store(true); }
    double thumbPronation0() const { return thumbPron0_.load(); }
    // 回正预测点的开关（量值跟 roll 共用一个参数）
    void setThumbPronationOn(bool on) { thumbPronOn_.store(on); thumbRollDirty_.store(true); }
    void setRateLimitEnabled(bool on) { rateLimitOn_.store(on); }
    // 关节角输出是否按【检测到的】物理手性自动纠正符号。关掉=完全按解算手性输出。
    void setJointMirrorAuto(bool on) { jointMirrorAuto_.store(on); if (!on) jointMirror_ = false; }
    bool jointMirrorActive() const { return jointMirror_; }
    bool rateLimitEnabled() const { return rateLimitOn_.load(); }
    // ---- ROM 标定：让操作者做一次"五指张开到底 -> 握拳到底"（约5秒）----
    // 【遥操作必须做】人手和机械手的行程不同，直接抄绝对角度会让机械手
    // 永远合不拢；而且我们的 MCP 角带一个未知常量零位偏置，归一化时
    // min/max 一起偏、正好抵消。见 Hm20JointAngles.hpp 的说明。
    // accumulate=true 是【补标】：保留上一轮样本继续攒。
    // 【为什么需要它】握拳时四指互相遮挡是物理事实，一次做不满很正常。
    // v1 只能推倒重来，于是用户反复重标反复不满；补标可以只针对没标满的
    // 那几维再做一次动作，已经标好的维在 finish() 里按"取更好的那份"合并。
    void romStart(bool accumulate = false) {
        romAccumPending_ = accumulate;
        romStartReq_.store(true);
        romLearning_.store(true);
    }
    bool romFinish() {
        romLearning_.store(false);
        romFinishReq_.store(true);
        return rom_.finish().ready;
    }
    double romCoverage() const { return rom_.coverage(); }
    bool romReady() const { return rom_.ready(); }
    // 逐维明细，给完成弹窗和录制块 31 用。
    const hm20::rom::CalibResult& romResult() const { return rom_.result(); }
    std::string romReport() const { return rom_.humanReport(); }
    std::string romJson() const { return rom_.toJson(); }
    void setRomConfig(const hm20::rom::Config& c) { rom_.configure(c); }
    void setFilterConfig(const hm20::PoseFilterConfig& c) {
        filtCfgPending_ = c; filtCfgDirty_.store(true);
    }
    // 自标定标定好之后，把模板发回主线程存盘/显示
    // （worker 线程发信号，主线程连队列连接接收）

    void setIkEnabled(bool on) {
        ikEnabled_.store(on);
        ikDirty_.store(true);
    }

    // 标定完成后热更新模板，不用重建线程。下一帧生效。
    // 注意这个函数会被【主线程】调用，而 tmplPending_ 由 worker 线程读取，
    // 所以用原子标志交接，不直接改 tmpl_。
    void updateTemplate(const hm20::Hm20Template& tmpl) {
        tmplPending_ = tmpl;
        tmplDirty_.store(true);
    }

public slots:
    // candidates: (原始track id, 世界坐标)。id 只回填给UI高亮，关联逻辑
    // 完全不依赖它跨帧稳定——这就是"点云ID怎么跳变都能正确连接"的落地点。
    void processFrame(SkeletonCandVec candidates, bool /*unusedBackendReady*/) {
        processFrameAt(std::move(candidates), -1);
    }

    // 带输入帧时间戳的版本。新调用方一律用这个。
    void processFrameAt(SkeletonCandVec candidates, qint64 frameTsNs, bool /*unusedBackendReady*/ = false) {
        inFrameTsNs_ = frameTsNs;
        // RAII：不管从哪条路径返回(包括 process() 里万一抛异常)，都保证
        // pending 被归还——漏还一次，之后主线程会永远认为"还在忙"，骨架
        // 从此再也不刷新(静默失效，最难查的一类bug)。
        struct SlotGuard {
            std::atomic<int>* p;
            ~SlotGuard() { p->store(0); }
        } guard{&pending};

        QElapsedTimer t; t.start();

        SkeletonAssocDiag diag;
        diag.candidateCount = int(candidates.size());

        // 【延迟初始化，且必须在worker线程里做】
        // onnxruntime 初始化 = 加载约5MB的DLL + 注册上千个ONNX算子schema +
        // 构图优化，Release约75ms，MinGW Debug下接近1秒；schema注册若有
        // 重复还会往stderr刷几千行(写Qt输出窗格是同步慢操作)。这一整套
        // 若放在GUI线程(对话框构造函数)里做，界面会明显卡住甚至像死机。
        // 放在这里：第一次真正要推理时才初始化，而且跑在worker线程上，
        // GUI线程完全不受影响——用户最多看到骨架晚一点出现。
        ensureInitialized();

        // 拇指 roll 偏置：纯参数，改了立刻生效，不用重建关联器。
        if (thumbRollDirty_.exchange(false) && assoc_) {
            // 【符号跟手性走】左手是右手的镜像：镜像 M=diag(1,-1,1) 把绕轴 n 转 θ
            // 映射成绕 M·n 转 -θ。所以同一个解剖旋前量，在左手上是反方向的。
            // 面板上填的是【解剖量的绝对值】(80~90°)，这里按手性给符号 ——
            // 否则左手用户会一路往正方向调、越调越错，永远找不到正确值。
            // 【用 tmpl_.isRight 而不是面板值】applyHandedness 打开时
            // tmpl_.isRight 可能是自标定推断出来的，而 ik->setHandedness() 用的
            // 也是它。两边不统一的话会出现"IK 按左手解、拇指旋前按右手补"。
            const double sgn = tmpl_.isRight ? 1.0 : -1.0;
            assoc_->setThumbRollOffset(sgn * thumbRoll_.load());
            assoc_->setOccludedIkOnly(ikOnly_.load());
            assoc_->setDorsumGeoRelabel(geoRelabel_.load());
            assoc_->setDorsumRigidSolve(dorsumRigid_.load());
            assoc_->setNetSegCalib(netSegCalib_.load());
            netSegSolved_.store(false);      // 开关一动就允许重解一次
            assoc_->setThumbSegFromNet(thumbSegNet_.load());
            assoc_->setCaptureDebugStreams(capDebug_.load());
            assoc_->setCaptureFullAssign(capFullAssign_.load());
            assoc_->setChainContinue(chainCont_.load());
            assoc_->setThumbPronation(thumbPronOn_.load() ? sgn * thumbRoll_.load() : 0.0);
        }

        // IK 挂载/开关：跟模板热更新一样，只在 worker 线程里落地。
        // 【放在推理之前】复位要在这一帧的 process() 生效，晚一帧就还是旧状态。
        // 【复位要覆盖全部跨帧状态，否则等价不了"重开面板"】
        // 实测上一版只复位关联器无效：IK 的时序先验 prevAngles_ 也会把错的
        // 姿态带下去；自标定的位姿历史同理。重开面板之所以有效，就是因为
        // 它把这几处一起重建了。
        if (dorsumResetReq_.exchange(false)) {
            // 【必须把模板和 IK 参数一起退回标定值】
            // 上一版只清运行时状态，等价不了重开面板 —— 因为面板重开会
            // 从 HandTemplateStore 重新读标定值、重建模板和 IK 参数，
            // 而这里不退回的话，自标定推过来的那份（可能就是坏的那份）
            // 会原样留着。用户的表现就是"复位了还是不对，得重开面板"。
            if (tmpl0Valid_) {
                tmpl_ = tmpl0_;
                autoTmplApplied_ = false;      // 让自标定重新走一遍验收
                lastAppliedRmse_ = 0.0;
                if (backend_) backend_->setTemplate(tmpl_);
                if (assoc_) {
                    assoc_->setNormalizedTemplate(tmpl_.packNormalized(), tmpl_.valid);
                    assoc_->setTemplateMm(tmpl_.markersMm);
                    if (tmpl_.anchorsValid) assoc_->setAnchorsMm(tmpl_.anchorsMm);
                }
            }
            if (ik0Valid_) {
                if (auto* ik = dynamic_cast<hm20::Hm20IkRefiner*>(pendingIk_.get())) {
                    ik->setBackTemplate(ik0Back_);
                    for (int f = 0; f < 5; ++f)
                        ik->setFingerParams(f, ik0Anchors_[size_t(f)], ik0Lengths_[size_t(f)]);
                }
            }
            if (assoc_) assoc_->resetDorsumRigid();
            if (pendingIk_) pendingIk_->resetPrior();
            // 【reset 而不是 resetTracking】resetTracking 只清位姿历史，
            // 攒下来的样本还在，下一帧就可能又把那份坏模板推回来。
            autoCalib_.reset();

            // ---- worker 自己这一层的时序状态，同样必须清 ----
            // 【为什么原来"复位了还得重开面板"】上面清的全是关联器和自标定
            // 里的状态，而 worker 这一层还挂着四个【跨帧累积】的东西，
            // 一个都没碰。重开面板会把它们全部重建，点复位却不会 ——
            // 差别就在这里。
            //
            //   filter_    位置/四元数的 One-Euro：内部停在【复位前那套错标签】
            //              解出来的位置上。复位后第一帧的真值跟它差很远，
            //              于是被当成跳变一路平滑过去，表现为"复位完还要飘几秒"。
            //   rate_      发散限速：每帧拿上一次的输出去夹当前解。上一次的
            //              输出是错的，就等于把新解一直往错的地方拽 ——
            //              这条最要命，它会让正确的解永远追不上来。
            //   angSmooth_ 遮挡期间的角度平滑，同理停在旧角度上。
            //   lastStamp_ dt 的基准。复位往往伴随一段卡顿，不重置的话
            //              复位后第一帧会算出一个巨大的 dt，把上面三个
            //              一起带偏。
            //
            // 判断标准跟关联器那边一样：新建 worker 时这个成员是什么，
            // 复位后就该是什么。
            filter_.reset();
            rate_.reset();
            angSmooth_.reset();
            lastStamp_.invalidate();
            lastDtSec_ = -1.0;
            // 关节角镜像是【自标定推断出来的】结论，而自标定刚被 reset()，
            // 结论也得跟着作废，否则会拿上一轮的手性继续翻符号。
            if (jointMirrorAuto_.load()) jointMirror_ = false;
            pendingEvents_.push_back({12, 0, 0.0, 0.0, 0, 0,
                std::string("手背重捕：已清空关联器/自标定/滤波/限幅/角度平滑"
                            "全部时序状态（等价于重开面板）")});
        }

        if (ikDirty_.exchange(false) && assoc_) {
            assoc_->setIkRefiner(ikEnabled_.load() ? pendingIk_ : nullptr);
        }

        // 拇指常数旋前：纯参数，跟手性无关，独立下发。见 setThumbPronation0()。
        if (ikParamDirty_.exchange(false)) {
            if (auto* ik = dynamic_cast<hm20::Hm20IkRefiner*>(pendingIk_.get()))
                ik->setThumbPronation0(thumbPron0_.load());
        }

        // 模板热更新：只在 worker 线程里落地，避免和推理读写竞争。
        if (tmplDirty_.exchange(false)) {
            const bool wasRight = tmpl_.isRight;
            const uint32_t oldFp = templateFingerprint(tmpl_);
            tmpl_ = tmplPending_;
            const uint32_t newFp = templateFingerprint(tmpl_);
            // 【模板一换就落盘 + 打事件】这是手性问题的第一现场。
            // 自标定推上来的模板可能跟原来差一个整体镜像，而镜像之后
            // 所有 RMSE 类自检都查不出来（手背近平面 + 五点近对称，
            // 反射≈绕面内轴转 180°，Kabsch 残差照样很小）。
            // 唯一能发现它的办法就是把前后两份模板的几何摆在一起比。
            markTemplateSnap(tmpl_.isRight != wasRight ? 2 : 1);
            pendingEvents_.push_back({2, tmpl_.isRight != wasRight ? 2 : 0,
                double(oldFp), double(newFp), wasRight ? 1 : 0, tmpl_.isRight ? 1 : 0,
                std::string("模板热替换：手性 ") + (wasRight ? "右" : "左") + " -> "
                    + (tmpl_.isRight ? "右" : "左")
                    + (tmpl_.isRight != wasRight ? "【手性跟着变了】" : "（手性未变）")});
            if (backend_) backend_->setTemplate(tmpl_);
            if (assoc_) {
                // 【不再重建 assoc_】原来这里 make_shared 一个新的关联器，把
                // 所有【学出来的】状态一起抹掉：手背重捕修好的模板、连续性锁、
                // 拇指 IP 跟随器学到的弯曲轴、dipK_、mdLen_……
                // 真机实测：自标定中途推了一份更差的模板(bundleRmse 20.7->31.3)，
                // 触发重建，之前两次成功的重捕全丢，整段之后再没恢复。
                // 换模板只是换两个成员，用 setter 就够，学习状态全部保留。
                assoc_->setNormalizedTemplate(tmpl_.packNormalized(), tmpl_.valid);
                assoc_->setTemplateMm(tmpl_.markersMm);
                if (tmpl_.anchorsValid) assoc_->setAnchorsMm(tmpl_.anchorsMm);
                if (pendingIk_ && ikEnabled_.load()) assoc_->setIkRefiner(pendingIk_);
            }
        }

        diag.backendReady = backendReady_;
        hm20::SkeletonFrameResult result;
        if (!backendReady_ || !assoc_) {
            diag.stageMessage = initError_.empty() ? "骨架AI后端未就绪" : initError_;
            diag.latencyMs = double(t.nsecsElapsed()) / 1.0e6;
            emit resultReady(result, diag);
            return;
        }

        QElapsedTimer ta; ta.start();
        result = assoc_->process(candidates);
        diag.tAssocMs = double(ta.nsecsElapsed()) / 1.0e6;
        ++skelProcessed_;
        diag.skelProcessed = skelProcessed_;
        diag.skelSkipped   = skelSkipped_.load();

        // ---- 在线闭环自标定 ----
        // 【放在 process() 之后】自标定要吃的是【关联结果】：标签、逐点概率、
        // 腕部位姿。没有这三样就只能对着一堆无序点瞎猜，那正是旧版
        // Hm20AutoTemplate 做不对的原因。
        if (handDirty_.exchange(false)) {
            const bool was = tmpl_.isRight;
            tmpl_.isRight = handIsRight_.load();
            if (auto* ik = dynamic_cast<hm20::Hm20IkRefiner*>(pendingIk_.get())) {
                ik->setHandedness(tmpl_.isRight);
                ikIsRight_ = tmpl_.isRight;
            }
            // 【这条路径必须留痕】handDirty_ 能把 tmpl_.isRight 改成一个既不等于
            // 面板值、也不等于自标定推断值的第三个值（时序上晚一帧）。
            // v4 记了 tmplIsRight 这个当前值，但没记它是【什么时候被谁改的】——
            // 而"标定时一切正常、一提交就左右反"要的正是这条时间线。
            if (was != tmpl_.isRight) {
                markTemplateSnap(2);
                pendingEvents_.push_back({1, 2, 0.0, 0.0, was ? 1 : 0, tmpl_.isRight ? 1 : 0,
                    std::string("手性下发：tmpl_.isRight ") + (was ? "右" : "左") + " -> "
                        + (tmpl_.isRight ? "右" : "左") + "（来源=面板/handDirty_，IK 同步更新）"});
            }
            rebuildAssoc();
        }
        if (autoCalibOn_.load()) {
            // 【每帧刷一次面板手性】自标定拿它当生效值。不这么做的话，
            // 自标定只会用它自己那个 4mm 阈值的推断，而那个推断一旦判反，
            // 会把整个手背模板镜像掉(见 AutoCalibConfig::applyHandedness)。
            autoCalib_.setUserHandedness(handIsRight_.load());
            // ---- 关节角输出手性 ----
            // 【解算手性和输出手性是两件事】实测左手用"右手"档连线更稳（不匹配
            // 时 IK 直接不生效，退回几何路径），但那样算出来的关节角是按右手系
            // 分解的，屈曲类各维符号是反的。既然自标定已经把物理手性判出来了，
            // 就用它来纠正输出，不必让用户在"连线稳"和"数对"之间二选一。
            // 只在【推断可信】(handednessKnown，即判据超过 4mm 阈值)时才动；
            // 判不出来就不猜，维持原样。
            if (jointMirrorAuto_.load()) {
                const auto& ac = autoCalib_.result();
                const bool want = ac.handednessKnown && ac.handednessConflict;
                // 【关节角镜像翻转要打事件】它把 16 维输出的屈曲类符号整个反过来，
                // 症状是"握拳输出像张开"—— 跟 ROM 塌了、跟解算失败长得一模一样。
                // 而它是【瞬间事件】，翻过去之后每一帧看起来都稳定地不对。
                if (want != jointMirror_) {
                    pendingEvents_.push_back({1, 2,
                        ac.handSignMm, ac.handSignLatMm, jointMirror_ ? 1 : 0, want ? 1 : 0,
                        std::string("关节角镜像 ") + (jointMirror_ ? "开->关" : "关->开")
                            + "（自标定判据 handSign=" + fmtNum(ac.handSignMm)
                            + "mm，known=" + (ac.handednessKnown ? "1" : "0")
                            + " conflict=" + (ac.handednessConflict ? "1" : "0") + "）"});
                }
                jointMirror_ = want;
            }
            if (autoCfgDirty_.exchange(false)) {
                autoCalib_.configure(autoCfgPending_);
                // configure() 覆盖整份 cfg_，会把上面刚设的 userIsRight 一起冲掉，
                // 必须补回来 —— 否则换配置那一帧的手性会退回默认值 true。
                autoCalib_.setUserHandedness(handIsRight_.load());
            }
            if (autoResetReq_.exchange(false)) { autoCalib_.reset(); autoTmplApplied_ = false; }
            const bool upd = autoCalib_.feed(result);
            if (upd || autoCalib_.takeDirty()) applyAutoCalib();
            fillAutoDiag(diag);
        }

        // ---- 时序滤波 ----
        // 【必须在自标定【之后】】自标定要的是未经平滑的原始观测：平滑过的点
        // 会让刚体自检的方差被人为压低，导致标错的模板也能通过检查。
        if (filterOn_.load()) {
            if (filtCfgDirty_.exchange(false)) filter_.configure(filtCfgPending_);
            const double dt = lastStamp_.isValid()
                            ? std::max(1e-3, double(lastStamp_.nsecsElapsed()) / 1.0e9) : -1.0;
            QElapsedTimer tf; tf.start();
            filter_.apply(result, dt);
            diag.tFilterMs = double(tf.nsecsElapsed()) / 1.0e6;
            lastDtSec_ = dt;
            diag.filterActive = true;
        }
        // 【无条件取，不放进 if】滤波关掉时 FrameDebug 的 ran=false，
        // 那本身就是要记录的信息 —— "这一帧没滤波"和"滤了但没动"完全不同。
        diag.filt = filter_.lastFrameDebug();
        // ---- 位置第 5 级：滤波之后 ----
        // 关联器填了 0..4 级（它看不到滤波），第 5 级只有这里知道。
        // 4 级和 5 级之差 = 滤波这一级干了什么，而滤波是就地覆写的，
        // 不在这里留一份，滤波前的值就永远拿不到了。
        for (int m = 0; m < hm20::kNumMarkers; ++m)
            diag.stage5Pos[size_t(m)] = result.markers[size_t(m)].posWorld;
        diag.stage5Valid = true;
        // IK 求解器内部（逐指 4 自由度解、限位命中、实际用的骨长/anchor/手性）
        diag.ikInfo = result.ikInfo;
        lastStamp_.start();

        // ---- 关节角：给 Unity / 机械手 ----
        // 【放在滤波之后】下游要的是平滑过的姿态；关节角是从 segQuat 算的，
        // 滤波先做，角度自然继承平滑结果，不需要再滤一遍。
        // 【ROM 标定期间放宽 measured()】握拳到底那一端中远节球必然被手掌
        // 挡住 —— 真实素材里中指三点全可见只有 31%，于是握得越紧、
        // fingerValid 越少、采到的样本越少，表现就是"怎么握都提示行程太短"。
        // 实测同一段 889 帧素材：放宽前每维只攒到 276~525 个样本，
        // 放宽后全部 889 个，coverage 51% -> 60%。
        //
        // 【只在标定时放宽】正常输出仍然只认几何/IK —— 拿补出来的点算角度，
        // 遥操作端会跟着抖。ROM 取 p2/p98 分位，对个别不准的样本免疫。
        const bool romOn = romLearning_.load();
        // 【beginPass 必须在 worker 线程里做】romStart() 跑在 GUI 线程上，
        // 而样本容器由 worker 线程读写。跟模板热更新走同一套原子交接。
        if (romStartReq_.exchange(false)) {
            rom_.beginPass(romAccumPending_);
            romStateDumpNs_ = -1;
            romSnapReq_ = 0;
        }
        // 【jdbg_ 必须无条件填 —— 它不只是调试出参】
        // dofState 是【功能输入】：romMap_.apply() 用它判"这一维本帧可不可信"，
        // 不可信就保持上一帧输出。而这条保持分支只在 rom_.ready() 之后才走得到，
        // 于是旧写法（不录制就传 nullptr、并把 jdbg_ 清成全 0）的后果被
        // "ROM 标定完成"这一刻精准引爆：
        //   JointSolveDebug{} 的 dofState 全 0 == DofState::Missing
        //   -> 16 维【全部】命中保持分支 -> 输出恒等于 last_
        //   -> romMap_.reset() 后的第 1 帧算出一个值，第 2 帧起永久冻结。
        // 症状就是"标完 ROM 关节角不动了；先点录制再标就正常"——
        // 录制期间 capDebug_ 为真，dofState 是真值，保持分支才按设计工作。
        // JointSolveDebug 只有几百字节，60~120Hz 下填它的开销可以忽略；
        // 【不要再按开关传 nullptr】，除非同时把 apply() 的 state 入参也断掉。
        ja_ = hm20::solveJointAngles(result, ja_, wristHold_, 24, jointMirror_, romOn,
                                     true, &jdbg_);
        // 【① 解算原始角：必须在平滑之前抓】下面一行 angSmooth_ 是【就地覆写】
        // ja_.q 的，抓晚一步就永远拿不到未经平滑的值了 —— 而"解算本身有没有
        // 解出来"和"解出来了但被平滑吃掉"是两个完全不同的问题。
        diag.qSolve = ja_.q;
        // 【ROM 采样用平滑前的值】平滑会把行程两端往里收，标出来的区间偏窄。
        //
        // 【v2 换成逐维采样】v1 传的是 JointAngleResult，里面只有逐指的
        // fingerValid —— 而腕部系失效时同一根手指里 PIP 是新算的、MCP 是
        // 保持上一帧的，fingerValid 对这两种情况给的是同一个 true。
        // v1 照单全收，于是把大量"张开时的 MCP 值"当成握拳样本收了进去。
        // v2 传逐维的 dofState（由 solveJointAngles 填），逐维判该不该收。
        romReject_ = {};
        if (romOn) {
            hm20::rom::FrameObs obs;
            obs.q     = ja_.q;
            obs.state = jdbg_.dofState;
            obs.curl  = jdbg_.curl;
            obs.curlValid = jdbg_.curlValid;
            obs.signRef = jdbg_.signRef;
            obs.signRefValid = jdbg_.signRefValid;
            obs.frameTsNs = inFrameTsNs_;
            romReject_ = rom_.observe(obs);
        }
        // ---- 采样真正开始/结束的那一帧，各打一个事件 ----
        // 【为什么不能只靠按钮事件】按钮回调在 GUI 线程，romLearning_ 是原子
        // 标志，worker 要到下一帧才读到。中间隔着一个投递延迟，对着帧号找
        // "标定从哪一帧开始"会差几帧。两头都记，差值本身还能反映积压。
        if (romOn != romLearnPrev_) {
            pendingEvents_.push_back({romOn ? 18 : 19, 0,
                romOn ? 0.0 : rom_.coverage(), 0.0,
                rom_.samples(), romAccumPending_ ? 1 : 0,
                romOn ? std::string("ROM 采样开始（worker 侧第一帧）")
                      : std::string("ROM 采样结束（worker 侧最后一帧）")});
            romLearnPrev_ = romOn;
        }
        // finish() 之后立刻把逐维结果打成事件 + 落 stateJson，
        // 这样即使用户没在录制时按结束，覆盖度和逐维状态也不会丢。
        if (romFinishReq_.exchange(false)) {
            const auto& rr = rom_.result();
            int nSignUnknown = 0;
            for (int i : hm20::rom::flexDofs())
                if (hm20::rom::DofStatus(rr.dof[size_t(i)].status) == hm20::rom::DofStatus::SignUnknown)
                    ++nSignUnknown;
            pendingEvents_.push_back({11, rr.ready ? 0 : 2,
                rr.coverageFlex, rr.coverageAbd, rr.nOkFlex, rr.nSignFlipped,
                std::string("ROM 标定") + (rr.ready ? "成功" : "未生效")
                    + "：屈曲覆盖 " + fmtNum(rr.coverageFlex * 100) + "%"
                    + "，达标 " + std::to_string(rr.nOkFlex) + "/11"
                    + "，方向纠正 " + std::to_string(rr.nSignFlipped)
                    + "，方向不明 " + std::to_string(nSignUnknown)
                    + "，失效 " + std::to_string(rr.nDeadFlex)});
            romMap_.reset();   // 区间换了，保持器里的旧值必须清掉
            romSnapReq_ = 2;
        }
        // 预测段的角度平滑。只作用于靠预测算出来的手指，实测的一帧不延迟。
        // 【dbg 出参：录制时必传】平滑器的 s/off 是就地覆写的，抓晚一步
        // 就永远拿不到了 —— 跟 qSolve 必须在平滑之前抓是同一个道理。
        ja_.q = angSmooth_.apply(ja_.q, ja_.fingerPredicted,
                                 capDebug_.load() ? &diag.angSmoothDbg : nullptr);
        diag.qSmooth = ja_.q;                                    // ②
        {
            // 【多传 dofState】映射器靠它对退化帧保持上一帧输出，
            // 而不是把 PIP≡0 这种结构性假值映射完发下去 ——
            // 那在遥操作上就是"握到底的瞬间机械手突然张开"。
            // 【dbg 出参：录制时必传】映射器内部的分支选择在系统里没有第二份，
            // 不在这里抓出来，事后就永远分不开"输出不动"的四种成因。
            // 不录制时传 nullptr，行为和开销跟以前逐位一致。
            const bool wantMapDbg = capDebug_.load();
            // 共同弯曲参考：signRef[1][0] 是四指共同 curl（候选1）。
            // 传给 Mapper 做“握拳只能增、张开只能减”的趋势守卫。
            const double bendRef = jdbg_.signRefValid[1][0]
                                 ? jdbg_.signRef[1][0] : -1.0;
            const auto qMapped = romMap_.apply(ja_.q, rom_, &jdbg_.dofState,
                                               wantMapDbg ? &diag.romMapDbg : nullptr,
                                               bendRef);
            diag.qRom = qMapped;                                 // ③
            const double dt = (lastDtSec_ > 1e-4) ? lastDtSec_ : (1.0 / 60.0);
            std::array<double, 16> qOut;
            if (rateLimitOn_.load()) {
                qOut = rate_.apply(qMapped, dt,
                                   capDebug_.load() ? &diag.rateDbg : nullptr);
            } else {
                // 【关掉时也要把 dbg 清成"没削"】不清的话上一帧的削幅会
                // 留在里面，看起来像限幅还在动 —— 而"关着"和"开着但没削"
                // 恰恰是这一级要分开的两件事。
                diag.rateDbg = hm20::RateLimiter::Debug{};
                // 直通。reset() 是必须的：不 reset 的话重新打开限幅时，
                // last 还停在关掉那一刻的旧值，第一帧会从那里慢慢爬回来。
                rate_.reset();
                qOut = qMapped;
            }
            diag.qOut = qOut;                                    // ④

            // ---- 逐级差分：把"信号在哪一级消失"变成两个数 ----
            // 【为什么要在这里算而不是留给离线】限幅削了多少，只有拿着
            // qMapped 和 qOut 同时在手才算得出来；离线虽然也能算，但那要求
            // 离线复刻限幅器的内部状态(last)，而复刻出来的东西对不对本身
            // 又要验证 —— 在线直接量掉，这一环就不用再怀疑了。
            diag.maxRateClipRad = 0.0; diag.maxRateClipIdx = -1;
            diag.maxStageDeltaRad = 0.0; diag.maxStageDeltaIdx = -1;
            for (int i = 0; i < 16; ++i) {
                const double clip = std::fabs(qOut[size_t(i)] - qMapped[size_t(i)]);
                if (clip > diag.maxRateClipRad) { diag.maxRateClipRad = clip; diag.maxRateClipIdx = i; }
                const double d0 = std::fabs(qOut[size_t(i)] - diag.qSolve[size_t(i)]);
                if (d0 > diag.maxStageDeltaRad) { diag.maxStageDeltaRad = d0; diag.maxStageDeltaIdx = i; }
            }
            // ---- ROM 区间本身 ----
            // 【必须逐帧带上】它是映射的分母。某一维 hi-lo≈0 时，那一维的输出
            // 恒等于行程端点 —— 症状正好是"这根手指怎么动都不动"或"永远张开"，
            // 而看输出值本身完全看不出原因。
            for (int i = 0; i < 16; ++i) {
                diag.romLo[size_t(i)] = rom_.lo(i);
                diag.romHi[size_t(i)] = rom_.hi(i);
            }
            for (int i = 0; i < 16; ++i) {
                diag.romSign[size_t(i)]   = int8_t(rom_.sign(i));
                diag.romStatus[size_t(i)] = uint8_t(rom_.status(i));
            }
            diag.jointDbg  = jdbg_;
            diag.romReject = romReject_;
            // ---- 块 31 的落盘时机：开始 / 标定中 1Hz / 结束 ----
            // 【为什么标定过程中也要写】覆盖度是一步步涨上来的，而"刚过线就
            // 停手"和"远超阈值"最后的 ready 是一样的，可信度天差地别。
            // 中间那几份还能看出是哪一段动作把行程撑起来的。
            diag.romSnapReason = -1;
            if (romSnapReq_ >= 0) { diag.romSnapReason = romSnapReq_; romSnapReq_ = -1; }
            else if (romOn) {
                const qint64 now = inFrameTsNs_ > 0 ? inFrameTsNs_ : 0;
                if (romStateDumpNs_ < 0 || now - romStateDumpNs_ > 1000000000LL) {
                    romStateDumpNs_ = now;
                    diag.romSnapReason = 1;
                }
            }
            if (diag.romSnapReason >= 0) {
                diag.romResult = rom_.result();
                diag.romAccumulated = romAccumPending_;
                // 判据跟结果一起走。同一批样本换一组阈值就是另一个结论，
                // 而两份记录在文件里看起来会完全一样。
                diag.romCfg = rom_.config();
            }
            diag.romMapHoldOn    = romMap_.holdOnDegenerate();
            diag.romReady        = rom_.ready();
            diag.romSamples      = rom_.samples();
            diag.romCoverage     = rom_.coverage();
            diag.romAbdCoverage  = rom_.abductionCoverage();
            diag.romLearningOn   = romOn;
            diag.rateLimitOn     = rateLimitOn_.load();
            diag.rateLimitRadPerSec = rate_.maxRadPerSec;
            diag.angSmoothAlpha  = angSmooth_.alpha;
            diag.angSmoothDecay  = angSmooth_.decay;
            diag.angSmoothMaxOffsetRad = angSmooth_.maxOffsetRad;
            diag.angSmoothOn     = (angSmooth_.alpha < 0.999);
            diag.dtSec           = dt;
            diag.jointMirrorFlag = jointMirror_;
            diag.quatOutOn       = quatOutOn_.load();
            diag.wristStaleFrames = result.wristPoseValid ? 0 : wristHold_.staleFrames;
            diag.jointFingerValid     = ja_.fingerValid;
            diag.jointFingerPredicted = ja_.fingerPredicted;
            fillQuatDiag(diag, result);

            QVector<double> q16; q16.reserve(16);
            for (int i = 0; i < 16; ++i) q16.push_back(qOut[size_t(i)]);
            QVector<double> r9; r9.reserve(9);
            for (int i = 0; i < 9; ++i) r9.push_back(result.wristR[size_t(i)]);
            emit handPoseForUdp(QVector3D(float(result.wristT[0]), float(result.wristT[1]),
                                          float(result.wristT[2])),
                                r9, q16, inFrameTsNs_);
            // ---- 只给 UI 的弯曲进度 ----
            // 横条显示“弯了多少”，不显示预测器的绝对角；数字列仍然是 qOut。
            QVector<double> bend5; bend5.reserve(5);
            for (int f = 0; f < 5; ++f)
                bend5.push_back(jdbg_.curlValid[size_t(f)] ? jdbg_.curl[size_t(f)] : 0.0);
            emit bendRefForUi(bend5, inFrameTsNs_);
            // ---- 对照用：解算原始角 vs 实际输出角 ----
            // 【为什么要单独发这一路】UDP 里那 16 个数是 ROM 映射 + 速度限幅
            // 之后的结果，光看它分不清两种完全不同的故障：
            //   ① 解算就没解出来（遮挡/标定坏）—— 原始角本身不动
            //   ② 解出来了但被 ROM 压平/被限幅削掉 —— 原始角在动，输出不动
            // 这两种的解法正好相反，所以必须能同屏对照。只给 UI 看，不进 UDP，
            // 没人连这个信号时 emit 的成本可以忽略。
            {
                QVector<double> raw16; raw16.reserve(16);
                for (int i = 0; i < 16; ++i) raw16.push_back(ja_.q[size_t(i)]);
                quint32 mask = 0;
                for (int f = 0; f < 5; ++f) if (ja_.fingerValid[size_t(f)]) mask |= (1u << f);
                if (ja_.mcpValid)   mask |= (1u << 5);
                if (ja_.wristValid) mask |= (1u << 6);
                if (rom_.ready())   mask |= (1u << 7);
                emit jointAnglesDebug(raw16, q16, mask, inFrameTsNs_);
            }
            for (int f = 0; f < 5; ++f) lastIkRmse_[f] = result.fingerIkRmseMm[size_t(f)];
            lastChain_        = result.chainContinued;
            for (int f = 0; f < 5; ++f)
                lastChainCoupledDeg_[f] =
                    result.chainCoupledRad[size_t(f)] * 57.29577951308232;
            // v7：模型姿态转成 M3DS 的 16 维再存，面板直接跟几何算的并排比。
            // 【不要在这里替换几何结果】观测充分时几何更准(5° vs 7.3°)，
            // 遮挡时模型更准(几何是 21.4°/max 107.9°)——取舍交给上层。
            lastGeoMargin_ = result.dorsumGeoMarginMm;
            lastTrkN_   = result.dorsumTrackN;
            lastTrkFix_ = result.dorsumTrackFixed;
            lastHasAiPose_ = result.hasAiJointAng;
            if (result.hasAiJointAng) {
                const auto j16 = hm20::jointAng20To16(result.aiJointAng);
                for (int i = 0; i < 16; ++i)
                    lastAiJointDeg_[i] = j16[size_t(i)] * 180.0 / M_PI;
            }
            lastHasAiHand_     = result.hasAiHand;
            lastAiHandKnown_   = result.aiHandKnown;
            lastAiHandRight_   = result.aiHandIsRight;
            lastAiHandConf_    = result.aiHandConf;
            lastAiHandLocked_  = result.aiHandLocked;
            if (result.hasAiPoseConf)
                for (int i = 0; i < 5; ++i)
                    lastAiPoseConf_[i] = 1.0 / (1.0 + std::exp(-result.aiPoseConf[size_t(i)]));
            lastHeld_         = result.occludedHeld;
            lastIkFilled_     = result.ikFilledMarkers;
            lastIkFallback_   = result.ikFallbackMarkers;
            lastThumbFixed_   = result.thumbFixed;
            lastThumbFixSkip_ = result.thumbFixSkip;
            diag.jointValidFingers = ja_.nValidFingers;
            diag.mcpValid = ja_.mcpValid;
        }

        // ---- 分段四元数（给想直接用四元数驱动 Unity 的下游）----
        if (quatOutOn_.load()) emitSegmentQuats(result);

        diag.frameTsNs = inFrameTsNs_;
        diag.palmOk = result.valid;
        diag.stageMessage = result.message;
        if (result.valid) {
            // 【改动】原来是 result.joints[5+j]，hm20 里叫 markers
            for (int j = 0; j < 15; ++j) {
                if (result.markers[size_t(5 + j)].observed) ++diag.observedJoints;
            }
            diag.predictedJoints = 15 - diag.observedJoints;
            diag.wristPoseValid = result.wristPoseValid;
            diag.numGhost       = result.numGhost;
            diag.dorsumRmseMm   = result.dorsumRmseMm;
            diag.dorsumInliers   = result.dorsumInliers;
            diag.dorsumMarginMm  = result.dorsumGeoMarginMm;
            diag.dorsumReason    = result.dorsumSolveReason;
            diag.dorsumByHistory = result.dorsumUsedHistory;
            diag.netThumbSegs    = result.netThumbSegs;
            diag.dorsumRepaired  = result.dorsumRepaired;
            diag.dorsumBadStreak = result.dorsumBadStreak;
            diag.dorsumRelock    = result.dorsumRelock;
            if (assoc_) diag.dorsumTmplDriftMm = assoc_->dorsumTemplateDriftMm();
            diag.tmplRejectedRmse = lastTmplRejected_;
            diag.tmplAppliedRmse  = lastAppliedRmse_;
            diag.bundleRuns   = autoCalib_.result().bundleRuns;
            diag.bundleLastMs = autoCalib_.result().bundleLastMs;
            // 【缓存文件名】它只在切换模型时变。每帧做一次
            // QFileInfo 构造 + std::string->QString 转换纯属浪费，
            // 而这是 120fps 的路径。
            diag.loadedModelName = loadedName_;
            diag.dorsumRepairMoveMm = result.dorsumRepairMoveMm;
            diag.thumbIpPredRad  = result.thumbIpPredRad;
            diag.thumbIpDriftRad = result.thumbIpDriftRad;
            diag.thumbTipOccluded = !result.markers[7].observed;
            // 【够样本就自动解一次，不做成按钮】这两个常数是一次性的标定量，
            // 解完就固定；做成按钮反而要求用户知道"什么时候该点"。
            // 解出来也【不会自动生效】——还要 thumbSegFromNet 显式打开，
            // 而打开之前应该先看 kOffsetDeg 落没落在 70~90°。
            if (assoc_ && netSegCalib_.load() && !netSegSolved_.load()
                && assoc_->netSegCSamples() >= 200 && assoc_->netSegKSamples() >= 200) {
                const auto rep = assoc_->solveNetSegConstants();
                netSegSolved_.store(true);
                lastNetSegOk_       = rep.ok;
                lastNetSegWhy_      = rep.why;
                lastNetSegCSpread_  = rep.cSpreadDeg;
                for (int j = 0; j < 3; ++j) lastNetSegKOffset_[j] = rep.kOffsetDeg[size_t(j)];
            }
            // 每秒刷一次运行时状态快照。放在这里而不是每帧：这份 JSON 约 2.5KB，
            // 120fps 下每帧重新生成纯属浪费，而它本来就几乎不变。
            if (!stateTimer_.isValid() || stateTimer_.elapsed() >= 1000) {
                stateTimer_.restart();
                std::string js = "{\"hm20\":";
                js += assoc_ ? assoc_->configJson() : "null";
                js += ",\"autocalib\":";
                js += autoCalib_.resultJson();
                js += ",\"worker\":{\"handIsRight\":";
                js += (handIsRight_.load() ? "true" : "false");
                js += ",\"tmplIsRight\":";
                js += (tmpl_.isRight ? "true" : "false");
                js += ",\"ikEnabled\":";
                js += (ikEnabled_.load() ? "true" : "false");
                js += ",\"thumbRollUi\":";
                { char t[48]; std::snprintf(t, sizeof(t), "%.5g", thumbRoll_.load()); js += t; }
                js += ",\"thumbPron0Ui\":";
                { char t[48]; std::snprintf(t, sizeof(t), "%.5g", thumbPron0_.load()); js += t; }
                js += "}}";
                std::lock_guard<std::mutex> lk(stateM_);
                stateJson_.swap(js);
                // 【顺手带给录制器】这份 JSON 一直只给面板看，从没落盘。
                // 而"这段数据是在什么配置下录的"是事后归因的第一步 ——
                // 一个月后没人说得清某段数据当时开没开某个开关，而参数快照
                // 是唯一凭据。1Hz、2.5KB，体积可以忽略。
                diag.stateJson = stateJson_;
            }
            diag.netSegSolved   = netSegSolved_.load();
            diag.netSegOk       = lastNetSegOk_;
            diag.netSegWhy      = lastNetSegWhy_;
            diag.netSegCSpread  = lastNetSegCSpread_;
            for (int j = 0; j < 3; ++j) diag.netSegKOffset[j] = lastNetSegKOffset_[j];
            if (assoc_) diag.dorsumSelfAmbMm = assoc_->dorsumSelfAmbiguityMm();
            diag.pentagonOk     = result.pentagonOk;
            diag.ikActive       = result.ikApplied;
            std::vector<double> rr;
            for (int f = 0; f < 5; ++f) if (result.fingerIkValid[size_t(f)]) {
                ++diag.ikFingerCount;
                if (result.fingerIkRmseMm[size_t(f)] == result.fingerIkRmseMm[size_t(f)])
                    rr.push_back(result.fingerIkRmseMm[size_t(f)]);
            }
            if (!rr.empty()) {
                std::sort(rr.begin(), rr.end());
                diag.ikRmseMm = rr[rr.size() / 2];
            }
        }
        diag.latencyMs = double(t.nsecsElapsed()) / 1.0e6;

        // 【无条件填，不放进 if (result.valid)】上面那一大块诊断是在
        // result.valid 里填的，而手性判错时 result.valid 完全可能是 true ——
        // 更麻烦的是解算失败(valid=false)的那些帧，恰恰是最需要知道
        // "当时各处的手性和开关是什么"的帧。放进条件分支等于在最需要的时候没有。
        fillHandednessDiag(diag, result);
        diag.runFlags           = buildRunFlags(diag);
        diag.thumbRollUiDeg     = thumbRoll_.load() * 180.0 / M_PI;
        diag.thumbPronation0Rad = thumbPron0_.load();
        diag.occludedJumpGateMm = cfg_.occludedJumpGateMm;

        // ---- v5：模板快照 / 事件流 / 配置转储 ----
        // 【周期性重录模板】即使这一秒没有变更也写一份。理由是变更检测本身
        // 可能漏（比如某条路径改了模板却没走 tmplDirty_），而周期快照能兜住：
        // 相邻两份的指纹不同就说明中间发生过一次没被记录的变更 ——
        // 这比"相信所有变更点都打过桩"可靠。1Hz、每份不到 1KB，可以忽略。
        if (!tmplTimer_.isValid() || tmplTimer_.elapsed() >= 1000) {
            tmplTimer_.restart();
            markTemplateSnap(4);
        }
        if (tmplSnapPending_ >= 0) {
            diag.tmplSnapshot   = tmpl_;
            diag.tmplSnapReason = tmplSnapPending_;
            diag.tmplFromAutoCalib   = autoTmplApplied_;
            diag.tmplBundleRmseMm    = lastAppliedRmse_;
            diag.tmplSelfAmbMm       = assoc_ ? assoc_->dorsumSelfAmbiguityMm() : -1.0;
            // 骨长和耦合系数不在 Hm20Template 里，从 IK 那边取【实际生效值】。
            if (auto* ik = dynamic_cast<hm20::Hm20IkRefiner*>(pendingIk_.get())) {
                (void)ik;
                for (int f = 0; f < 5; ++f) {
                    diag.tmplBoneLenMm[size_t(f)]   = result.ikInfo.boneLenMm[size_t(f)];
                    diag.tmplDipCoupling[size_t(f)] = result.ikInfo.dipCoupling;
                }
            }
            tmplSnapPending_ = -1;
        }
        if (!pendingEvents_.empty()) {
            diag.events.swap(pendingEvents_);
            pendingEvents_.clear();
        }
        // 配置转储：1Hz，跟 stateJson 同步。两者互补 —— stateJson 是 hm20 和
        // 自标定，configJson 是滤波/IK/ROM/限幅/角度平滑，合起来才齐。
        if (!cfgTimer_.isValid() || cfgTimer_.elapsed() >= 1000) {
            cfgTimer_.restart();
            diag.configJson = buildConfigJson();
        }

        emit resultReady(result, diag);
    }

signals:
    void resultReady(mocap::hm20::SkeletonFrameResult result, mocap::SkeletonAssocDiag diag);
    // 自标定标定完成 / 被推翻。主线程可据此存盘或提示。
    void autoCalibTemplateReady(mocap::hm20::Hm20Template tmpl);
    void autoCalibReset();
    // 【驱动 Unity / 机械手的输出】签名刻意跟 UdpSender::onHandPoseSmoothed
    // 完全一致，所以 MainWindow 里一行 connect 就能接上，UDP 包格式(M3DS)
    // 和 Unity 端一个字节都不用改。
    void handPoseForUdp(QVector3D wristPos, QVector<double> wristRot9,
                        QVector<double> jointAngles16, qint64 ts_ns);
    // 只给数据面板横条用的弯曲进度（不是 UDP 数据）。
    // 索引 0=拇 1=食 2=中 3=无 4=小；每根手指独立。
    void bendRefForUi(QVector<double> fingerBend5, qint64 ts_ns);
    // 分段四元数（世界系 + 相对父节点）。签名对齐 UdpSender::onSegmentQuats。
    void segmentQuatsForUdp(QVector3D wristPos, QVector<double> quatWorld64,
                            QVector<double> quatLocal64, QVector<int> segSource16,
                            quint32 flags, qint64 ts_ns);
    // 【只给 UI 看，不进 UDP】解算出来的原始关节角 + 实际输出的关节角，同一帧。
    // mask: bit0..4 该指本帧新算(不是保持上一帧)  bit5 mcpValid  bit6 wristValid
    //       bit7 ROM 已标定
    // 点云面板的输出监视器用它做"解算 vs 输出"对照，见 ui/HandOutputMonitor.hpp。
    void jointAnglesDebug(QVector<double> rawRad16, QVector<double> outRad16,
                          quint32 mask, qint64 ts_ns);

private:
    // 只在worker线程里被 processFrame 调用，只做一次。不需要加锁：这个
    // 对象moveToThread之后，它的槽函数只会在worker线程的事件循环里串行
    // 执行，不存在并发进入的可能。
    void ensureInitialized() {
        // 【模型切换也走这里】modelDirty_ 置位时重建 session。
        // 重建会丢掉 backend 的内部状态，但 backend 是无状态的（每帧独立推理），
        // 关联器和自标定的状态都不在 backend 里，所以切换是安全的。
        if (modelDirty_.exchange(false)) initTried_ = false;
        if (initTried_) return;
        initTried_ = true;

        // 高速模式：用 INT8 那份 + 多线程。文件不存在就退回原模型，
        // 并把实际加载的路径记下来给 UI —— 静默退回是最糟的，
        // 用户会以为自己在跑高速模式。
        std::string usePath = modelPath_;
        int useThreads = 1;
        if (fastMode_.load() && !fastPath_.empty()) {
            if (QFileInfo::exists(QString::fromStdString(fastPath_))) {
                usePath = fastPath_;
                useThreads = intraOpThreads_;
            }
        } else if (intraOpThreads_ > 1) {
            // 不开高速模式也可以只开多线程 —— 那一项不改变任何数值结果。
            useThreads = intraOpThreads_;
        }
        {
            std::lock_guard<std::mutex> lk(stateM_);
            loadedModel_ = usePath;
        }
        // 只在这里算一次；diag 每帧直接取。
        loadedName_ = QFileInfo(QString::fromStdString(usePath)).fileName();

        auto backend = std::make_shared<hm20::Hm20OnnxBackend>(usePath, useThreads);
        if (!backend->ready()) {
            backendReady_ = false;
            // Hm20OnnxBackend 在契约不匹配时会把模型真实的 I/O dump 进
            // lastError()，这里原样带回给UI——路径配成了上一代模型时，
            // 错误信息会直接点名"4输入!=6，那是上一代模型"。
            initError_ = backend->lastError();
            return;
        }
        backend->setTemplate(tmpl_);
        backend_ = std::move(backend);
        if (assoc_) {
            // 【只换后端】关联器已经存在（比如从 FP32 切到 INT8 高速模式），
            // 不要重建它 —— 重建会把手背重捕修好的模板、连续性锁、
            // 拇指跟随器学到的轴、dipK_/mdLen_ 全部抹掉。backend 是无状态的，
            // 换个指针就够。
            assoc_->setBackend(backend_);
            attachIk();
            backendReady_ = true;
            initError_.clear();
            return;
        }
        assoc_ = std::make_shared<hm20::Hm20SkeletonAssociator>(
                     backend_, tmpl_.packNormalized(), tmpl_.valid, cfg_);
        // 【别漏】Kabsch 用的是【毫米】模板，跟送进网络的归一化模板是两份
        // 不同单位的数据。不调这个，tmplMmValid_ 恒为 false，腕部位姿永远
        // 解不出来 —— 症状是 wristPoseValid 一直是 false、骨架整体不转。
        assoc_->setTemplateMm(tmpl_.markersMm);
        // 指根关节：近节骨朝向要用。不设的话尾部误差明显变差
        // (真 rig 实测 p90 30.4° -> 42.5°、max 54.3° -> 85.5°)
        if (tmpl_.anchorsValid) assoc_->setAnchorsMm(tmpl_.anchorsMm);
        attachIk();
        backendReady_ = true;
        initError_.clear();
    }

    // 自标定产出的参数下发到 associator / IK / 网络模板。
    // 全部在 worker 线程里做，主线程碰不到这些对象。
    void applyAutoCalib() {
        const auto& a = autoCalib_.result();
        if (!a.backValid) {                      // 漂移重标：退回无模板档
            autoTmplApplied_ = false;
            tmpl_.valid = false;
            rebuildAssoc();
            emit autoCalibReset();
            return;
        }
        // 【只接受不变差的模板】自标定是在线持续跑的，它的解会随着采到的姿势
        // 好坏起伏。真机实测过一次：会话中途它的 bundleRmse 从 20.7 涨到 31.3，
        // 却照样把那份更差的模板推了下去，把之前两次成功的手背重捕一起冲掉，
        // 整段之后再没恢复。用户的感受是"退出去手动标定再进来会好一点"。
        //
        // 判据用 bundleRmseMm：它就是这份模板对全部 shot 的拟合残差，
        // 涨了就是模板变差了。放 1.15 倍的余量，避免正常抖动被挡住。
        if (autoTmplApplied_ && a.bundleRmseMm > 0.0 && lastAppliedRmse_ > 0.0
            && a.bundleRmseMm > lastAppliedRmse_ * 1.15 + 0.5) {
            lastTmplRejected_ = a.bundleRmseMm;
            return;                              // 比在用的这份差，不换
        }
        lastAppliedRmse_  = (a.bundleRmseMm > 0.0) ? a.bundleRmseMm : lastAppliedRmse_;
        lastTmplRejected_ = -1.0;

        tmpl_.markersMm    = a.markersMm;
        tmpl_.anchorsMm    = a.anchorsMm;
        tmpl_.anchorsValid = true;
        tmpl_.valid        = true;
        // ---- 【验收闸门】自标定的产物必须比现状好，才准上线 ----
        // 【为什么必须有】自标定的定位是"锦上添花"：标好了 IK 能用、精度提升；
        // 标不好就该原样退回不标定的状态。但原来没有这个退路 —— 模板一提交
        // 就全链生效，包括喂给模型的 tmpl，于是一次坏标定会把【模型的标签能力】
        // 一起拖坏，表现成"自标定一开，连线就乱"。
        //
        // 一个可选的优化把基础功能拖坏，这是设计缺陷，不是调参问题。
        // 判据用已经在算的量，不新增计算：
        //   · 手背 Kabsch 残差不能比提交前差
        //   · anchor 的刚性 σ（|anchor-pp| 跨帧标准差）必须落在观测噪声量级
        // 任一不过 -> 不提交，保持现状，并把原因报到面板上。
        if (cfgAcceptGate_) {
            double worstStd = 0.0;
            int nChecked = 0;
            for (int f = 0; f < 5; ++f) {
                if (!a.anchorFitted[size_t(f)]) continue;
                const double sd = a.anchorRigidStdMm[size_t(f)];
                if (sd < 0) continue;                    // 样本还不够，不判
                worstStd = std::max(worstStd, sd);
                ++nChecked;
            }
            if (nChecked > 0 && worstStd > acceptMaxRigidStdMm_) {
                lastAcceptReject_ = 1;                   // 1 = 刚性σ 不过
                return;                                  // 不提交，保持现状
            }
            lastAcceptReject_ = 0;
        }

        // a.isRight 是【生效值】：默认就等于面板上的「右手」，只有显式打开
        // applyHandedness 时才会是自标定推断出来的那个。所以这行不会再悄悄
        // 把用户的设置改掉。
        if (a.handednessKnown) tmpl_.isRight = a.isRight;
        autoTmplApplied_ = true;
        // 【落盘自标定提交】reason=3 精确到帧地记录这次模板热替换，
        // 不再依赖 1Hz 周期快照兜底比对。
        markTemplateSnap(3);

        if (auto* ik = dynamic_cast<hm20::Hm20IkRefiner*>(pendingIk_.get())) {
            ik->setBackTemplate(a.backMm);
            ik->setHandedness(tmpl_.isRight);
            ikIsRight_ = tmpl_.isRight;
            ik->setThumbAxialK(a.thumbAxialK);
            // 【别漏这一行】束调整解出来的常数旋前必须下发，否则 refineThumbPronation
            // 白算 —— 它只改了 Hm20AutoCalib 内部那个临时 ik，活着的这个还是 0，
            // 拇指照样外翻，而且面板上什么异常都看不到。
            ik->setThumbPronation0(a.thumbPronation0);
            for (int f = 0; f < 5; ++f)
                ik->setFingerParams(f, a.anchorsMm[size_t(f)], a.lengthsMm[size_t(f)]);
            ik->resetPrior();
        }
        // 【不再 rebuildAssoc()】模板热更新那条路上，这个破坏性重建已经被
        // 专门删掉过，注释里还留着实测教训："把所有学出来的状态一起抹掉…
        // 之前两次成功的重捕全丢，整段之后再没恢复"。而【同一个重建在这条
        // 路上还活着】—— 自标定每接受一份新模板就 make_shared 一个新关联器，
        // 手背重捕修好的模板、骨长中位窗、拇指 IP 轴、手性锁全部归零。
        // 用户看到的就是"跑着跑着连线又乱了，得重开面板"。
        //
        // 换成跟那条路一样的做法：用 setter 换模板和 anchor，然后调
        // resetDorsumRigid() 清掉【跟腕部系绑定的】学习状态（弯曲平面、
        // 骨轴、遮挡入口参照都是腕部系量，模板一换它们的参照系就变了），
        // 而骨长中位窗、手性锁这些与模板无关的量保留下来。
        //
        // 【anchor 必须在这里下发】原来 setAnchorsMm() 只在构造关联器时调过，
        // 自标定中途算出来的 anchor 根本送不进去 —— 这正是"刚开面板时遮挡
        // 预测歪、点一次重标就好"的原因：重标逼自标定重跑，收敛后走到这里
        // 触发重建，新关联器才第一次拿到 anchor。
        if (backend_) backend_->setTemplate(tmpl_);
        if (assoc_) {
            assoc_->setNormalizedTemplate(tmpl_.packNormalized(), tmpl_.valid);
            assoc_->setTemplateMm(tmpl_.markersMm);
            if (tmpl_.anchorsValid) assoc_->setAnchorsMm(tmpl_.anchorsMm);
            assoc_->resetDorsumRigid();
            attachIk();
        } else {
            rebuildAssoc();          // 还没有关联器时才真的建一个
        }
        emit autoCalibTemplateReady(tmpl_);
    }

    void rebuildAssoc() {
        if (!backend_) return;
        backend_->setTemplate(tmpl_);
        assoc_ = std::make_shared<hm20::Hm20SkeletonAssociator>(
                     backend_, tmpl_.packNormalized(), tmpl_.valid, cfg_);
        assoc_->setTemplateMm(tmpl_.markersMm);
        if (tmpl_.anchorsValid) assoc_->setAnchorsMm(tmpl_.anchorsMm);
        attachIk();
    }

    // 【IK 的总闸】开 IK 需要三件事同时成立：用户勾了、有 refiner、
    // 并且参数确实可信。自标定没收敛就挂 IK，实测骨轴中位会从 12.3° 劣化到
    // 18.9° —— 比不开还差。宁可不精修。
    //
    // 【这是边沿触发】只在backend换/模板换/自标定提交新模板这几个【事件】发生
    // 的那一瞬间采样一次 ikUsable()。ikUsable() 本身是随 bundleRmse 连续
    // 变化的电平量：如果这几个事件都恰好赶在 bundleRmse 还没收敛的时候发生，
    // 之后 bundleRmse 慢慢降下去了，也没有任何代码会再来看一眼——IK 就被
    // 永久摘掉。fillAutoDiag() 里补了一次每帧的电平复检来兜底这个情况，
    // 这里把 lastIkAttached_ 同步一下，避免复检把这次边沿触发的结果
    // 误判成"又翻转了一次"、重复打事件。
    void attachIk() {
        if (!assoc_) return;
        const bool paramsOk = !autoCalibOn_.load() || !autoTmplApplied_
                            ? true : autoCalib_.ikUsable();
        const bool wantIk = pendingIk_ && ikEnabled_.load() && paramsOk;
        assoc_->setIkRefiner(wantIk ? pendingIk_ : nullptr);
        lastIkAttached_ = wantIk;
    }

    // 世界系分段四元数 -> 相对父节点。
    //
    // 【父子关系】骨架是 1 个手腕 + 5 指 ×3 节：
    //   段0 = 手腕（父 = 世界）
    //   段 1+3f+0 近节，父 = 手腕
    //   段 1+3f+1 中节，父 = 近节
    //   段 1+3f+2 远节，父 = 中节
    // localRot = inverse(parentWorldRot) * childWorldRot，Unity 里可以直接
    // 赋给 bone.localRotation。
    //
    // 【坐标系提醒】这里是右手系、毫米、Z 朝手背。Unity 是左手系、米、Y 朝上。
    // 换算放在 Unity 端做更合适：那边知道自己的骨骼绑定姿势，而我这边不知道。
    // 四元数符号规范化：统一到 w >= 0 那一侧。
    //
    // 【为什么要做】q 和 -q 表示【同一个旋转】，所以这一步在数学上不改变任何
    // 东西，Unity 的 localRotation 赋值也照样正确。但实测输出里 w 的符号是
    // 乱的：同一帧里"完全张开"的十六段，有的写 +1.000 有的写 -1.000
    // （拇近节 -1.000、中中节 +1.000、无中节 -1.000 混着来）。这会坑到两类
    // 下游代码，而且坑得很隐蔽：
    //
    //   ① 从 w 反解角度。`angle = 2*acos(w)` 是最常见的写法，
    //      而 w=-1 会算出 360° —— 一个"完全没转"的关节被读成"转了一整圈"。
    //      正确写法要取 |w|，但没人会想到自己拿到的 w 可能是负的。
    //   ② 逐帧插值。相邻两帧如果一帧 +q、下一帧 -q，朴素 lerp 会绕远路
    //      转一整圈。Unity 的 Slerp 内部做了半球对齐所以没事，但自己写的
    //      插值、或者 UDP 收下来先缓存再插值的代码，通常不做这件事。
    //
    // 【为什么不在时序对齐里顺带做】时序半球对齐（跟上一帧同号）保证的是
    // "不跳变"，它完全允许结果长期停在 w<0 那一侧 —— 两件事目标不同，
    // 都要做。这里放在最后一步，不影响前面任何时序连续性处理。
    //
    // 【一个必须知道的副作用】世界系和相对系是各自独立规范化的，所以
    // `world_child == world_parent ⊗ local_child` 这个等式，规范化之后
    // 【不再逐分量严格成立】—— 随机姿态下实测约 35% 的段会差一个整体负号。
    // 旋转本身完全一致（±q 校验 0 失败），Unity / Slerp / 转矩阵这些按
    // 四元数语义用的地方毫无影响。
    // 但如果以后有人写脚本【逐分量】校验父子链自洽性，必须按 ±q 比对
    // （取 min(|pred-act|, |pred+act|)），否则会看到一堆假报错。
    static hm20::Quat canonQuat(const hm20::Quat& q) {
        return (q[0] < 0.0) ? hm20::Quat{-q[0], -q[1], -q[2], -q[3]} : q;
    }

    void emitSegmentQuats(const hm20::SkeletonFrameResult& r) {
        auto qmul = [](const hm20::Quat& a, const hm20::Quat& b) {
            return hm20::Quat{ a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                               a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                               a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                               a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0] };
        };
        auto qconj = [](const hm20::Quat& q) {
            return hm20::Quat{q[0], -q[1], -q[2], -q[3]};
        };

        QVector<double> w64, l64;
        QVector<int> src16;
        w64.reserve(64); l64.reserve(64); src16.reserve(16);
        for (int s = 0; s < hm20::kNumSegments; ++s) {
            const hm20::Quat& qw = r.segQuat[size_t(s)];
            int parent = -1;                       // -1 = 世界
            if (s > 0) {
                const int j = (s - 1) % 3;         // 0近节 1中节 2远节
                parent = (j == 0) ? 0 : (s - 1);
            }
            // 【规范化后再发】见 canonQuat 的说明。世界系和相对系【各自】
            // 规范化 —— 它们是两个独立的量，下游可能只取其中一个用。
            const hm20::Quat qwC = canonQuat(qw);
            const hm20::Quat ql = canonQuat((parent < 0) ? qw
                                : qmul(qconj(r.segQuat[size_t(parent)]), qw));
            for (int k = 0; k < 4; ++k) w64.push_back(qwC[size_t(k)]);
            for (int k = 0; k < 4; ++k) l64.push_back(ql[size_t(k)]);
            src16.push_back(int(r.segSource[size_t(s)]));
        }
        quint32 flags = 0;
        if (r.wristPoseValid) flags |= 1u;
        if (ja_.mcpValid)     flags |= 2u;
        for (int f = 0; f < 5; ++f)
            if (ja_.fingerValid[size_t(f)]) flags |= (1u << (8 + f));

        emit segmentQuatsForUdp(QVector3D(float(r.wristT[0]), float(r.wristT[1]),
                                          float(r.wristT[2])),
                                w64, l64, src16, flags, inFrameTsNs_);
    }

    // 把世界系和【相对父节点】两种四元数都存进诊断。
    //
    // 【为什么两个都要】判"握拳时手指到底弯没弯"必须看相对父节点的那个：
    // 世界系四元数里，整只手的朝向和手指自身的屈曲是【混在一起】的 ——
    // 手整体转 90° 和手指弯 90°，在世界系里都表现为一个大角度变化。
    // 拿世界系四元数去看屈曲，会把手腕的转动误读成手指在动，反之亦然。
    // 而这正是"输出看着不对但说不清哪不对"最常见的来源。
    //
    // 【跟 emitSegmentQuats 用同一套父子关系和同一套公式】刻意复制而不是
    // 抽公共函数，是因为那边在 quatOutOn_ 关闭时【整个不执行】，而诊断
    // 必须无条件记录 —— 依赖它就会出现"关了四元数输出，录制里四元数也没了"。
    void fillQuatDiag(SkeletonAssocDiag& d, const hm20::SkeletonFrameResult& r) const {
        auto qmul = [](const hm20::Quat& a, const hm20::Quat& b) {
            return hm20::Quat{ a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                               a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                               a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                               a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0] };
        };
        auto qconj = [](const hm20::Quat& q) {
            return hm20::Quat{q[0], -q[1], -q[2], -q[3]};
        };
        for (int s = 0; s < hm20::kNumSegments; ++s) {
            const hm20::Quat& qw = r.segQuat[size_t(s)];
            int parent = -1;
            if (s > 0) {
                const int j = (s - 1) % 3;
                parent = (j == 0) ? 0 : (s - 1);
            }
            // 【录制里也要规范化】否则录制和 UDP 发出去的是两套符号，
            // 事后拿录制复核"下游收到的到底是什么"就对不上了。
            const hm20::Quat qwC = canonQuat(qw);
            const hm20::Quat ql = canonQuat((parent < 0) ? qw
                                : qmul(qconj(r.segQuat[size_t(parent)]), qw));
            for (int k = 0; k < 4; ++k) {
                d.quatWorld[size_t(s*4 + k)] = qwC[size_t(k)];
                d.quatLocal[size_t(s*4 + k)] = ql[size_t(k)];
            }
        }
    }

    // 手性证据链。【六处环节各自认为的手性都要记】——
    // 只记结论(4 个 bool)时出现过一次查不下去的情况：四个全"正常"，而实际
    // 送进网络的 tmpl_.isRight 是被 handDirty_ 那条路改过的另一个值。
    // 漏掉的那一处恰恰就是错的那一处，这不是巧合：没被记录的东西才会被漏看。
    void fillHandednessDiag(SkeletonAssocDiag& d, const hm20::SkeletonFrameResult& r) const {
        const auto& a = autoCalib_.result();
        const auto& c = autoCalib_.config();
        d.handSignLatMm       = a.handSignLatMm;
        d.handednessAgree     = a.handednessAgree;
        d.handednessMinMm     = c.handednessMinMm;
        d.cfgApplyHandedness  = c.applyHandedness;
        d.cfgDetectHandedness = c.detectHandedness;
        d.cfgJointMirrorAuto  = jointMirrorAuto_.load();
        d.autoCalibOn         = autoCalibOn_.load();
        d.geoHandSign         = r.geoHandSign;
        d.aiHandLogit         = r.netHandLogit;
        d.aiHandConfVal       = r.aiHandConf;
        // 【这两个是 v3 缺的那两处】
        d.tmplIsRight         = tmpl_.isRight;    // 送进网络/建模板用的
        d.ikIsRight           = ikIsRight_;       // IK 用的
        // 拇指 roll 的【下发值】，已经带了手性符号。它跟手性绑死：
        // 手性判反时这个值也会跟着反，症状是拇指单独朝相反方向歪 ——
        // 光看手性 bool 看不出来，看这个数一眼就知道。
        d.thumbRollSignedRad  = (tmpl_.isRight ? 1.0 : -1.0) * thumbRoll_.load();
        d.handSignN           = 0;   // AutoCalibResult 不外露样本数，留 0 表示未知
    }

    // -----------------------------------------------------------------------
    // v5 辅助
    // -----------------------------------------------------------------------
    static std::string fmtNum(double v) {
        char t[48];
        std::snprintf(t, sizeof(t), "%.4g", v);
        return std::string(t);
    }

    // 模板指纹。【只覆盖几何和手性】—— 掺进时间戳之类的东西，
    // "模板变没变"就判不出来了，而那正是它唯一的用途。
    static uint32_t templateFingerprint(const hm20::Hm20Template& t) {
        uint32_t h = 2166136261u;
        auto mix = [&h](const void* p, size_t n) {
            const unsigned char* b = static_cast<const unsigned char*>(p);
            for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 16777619u; }
        };
        for (int m = 0; m < hm20::kNumMarkers; ++m)
            for (int d = 0; d < 3; ++d) {
                const float v = float(t.markersMm[size_t(m)][size_t(d)]);
                mix(&v, sizeof(v));
            }
        for (int i = 0; i < 5; ++i)
            for (int d = 0; d < 3; ++d) {
                const float v = float(t.anchorsMm[size_t(i)][size_t(d)]);
                mix(&v, sizeof(v));
            }
        const uint8_t r = t.isRight ? 1 : 0;
        mix(&r, 1);
        return h;
    }

    void markTemplateSnap(int reason) {
        // 【取更"重要"的那个原因】同一帧里既有热替换又有手性变更时，
        // 手性变更是要紧的那个，别被后来的周期性快照盖掉。
        if (tmplSnapPending_ < 0 || (reason != 4 && reason < tmplSnapPending_))
            tmplSnapPending_ = reason;
        else if (tmplSnapPending_ == 4 && reason != 4)
            tmplSnapPending_ = reason;
    }

    // 全量配置转储。【这一项是离线调参的前提】
    // 要把某一级的参数调到最优，必须同时有那一级的输入、参数、输出。
    // 输入输出各块都有了，参数就在这里 —— 而 stateJson 只覆盖了 hm20 和
    // 自标定，滤波/IK/ROM/限幅/角度平滑一个都没有，恰恰是最需要调的几个。
    std::string buildConfigJson() const {
        const auto& fc = filter_.config();
        std::string js = "{\"filter\":{";
        auto kv = [](const char* k, double v) {
            char t[96];
            std::snprintf(t, sizeof(t), "\"%s\":%.6g", k, v);
            return std::string(t);
        };
        auto kb = [](const char* k, bool v) {
            return std::string("\"") + k + "\":" + (v ? "true" : "false");
        };
        js += kb("enabled", fc.enabled) + "," + kb("filterPositions", fc.filterPositions);
        js += "," + kb("filterRotations", fc.filterRotations);
        js += "," + kb("predictFuse", fc.predictFuse);
        js += "," + kv("fps", fc.fps);
        js += "," + kv("posMinCutoffHz", fc.posMinCutoffHz);
        js += "," + kv("posBeta", fc.posBeta);
        js += "," + kv("posDCutoffHz", fc.posDCutoffHz);
        js += "," + kv("posDeadbandMm", fc.posDeadbandMm);
        js += "," + kv("posJumpGateMm", fc.posJumpGateMm);
        js += "," + kv("predictedSmooth", fc.predictedSmooth);
        js += "," + kv("predictedFixedCutoffHz", fc.predictedFixedCutoffHz);
        js += "," + kv("rotMinCutoffHz", fc.rotMinCutoffHz);
        js += "," + kv("rotBeta", fc.rotBeta);
        js += "," + kv("rotDeadbandDeg", fc.rotDeadbandDeg);
        js += "," + kv("fuseVelAlpha", fc.fuseVelAlpha);
        js += "," + kv("fuseVelTauFrames", fc.fuseVelTauFrames);
        js += "," + kv("fuseWeightTauFr", fc.fuseWeightTauFr);
        js += "," + kv("fuseOutAlpha", fc.fuseOutAlpha);
        js += "},\"joint\":{";
        js += kv("rateLimitRadPerSec", rate_.maxRadPerSec);
        js += "," + kv("angSmoothAlpha", angSmooth_.alpha);
        js += "," + kb("romReady", rom_.ready());
        js += "," + kv("romSamples", rom_.samples());
        js += "," + kv("romCoverage", rom_.coverage());
        js += "," + kv("romAbdCoverage", rom_.abductionCoverage());
        js += ",\"romLo\":[";
        for (int i = 0; i < 16; ++i) { if (i) js += ","; js += fmtNum(rom_.lo(i)); }
        js += "],\"romHi\":[";
        for (int i = 0; i < 16; ++i) { if (i) js += ","; js += fmtNum(rom_.hi(i)); }
        // 【逐维明细也进 stateJson】romLo/romHi 只回答"区间是多少"，
        // 而标不满时要问的是"为什么是这么多"。答案在方向、拒收分布、
        // 两端中位这几个数里，它们全在 romDetail 这一份。
        js += "],\"romDetail\":" + rom_.toJson();
        js += "},\"ik\":{";
        if (auto* ik = dynamic_cast<const hm20::Hm20IkRefiner*>(pendingIk_.get())) {
            js += kv("thumbAxialK", ik->thumbAxialK());
            js += "," + kv("thumbPronation0", ik->thumbPronation0());
            js += "," + kb("isRight", ik->isRight());
        } else {
            js += "\"present\":false";
        }
        js += "},\"worker\":{";
        js += kb("ikEnabled", ikEnabled_.load());
        js += "," + kb("filterOn", filterOn_.load());
        js += "," + kb("rateLimitOn", rateLimitOn_.load());
        js += "," + kb("romLearning", romLearning_.load());
        js += "," + kb("quatOutOn", quatOutOn_.load());
        js += "," + kb("autoCalibOn", autoCalibOn_.load());
        js += "," + kb("jointMirrorAuto", jointMirrorAuto_.load());
        js += "," + kb("jointMirrorNow", jointMirror_);
        js += "," + kb("handIsRight", handIsRight_.load());
        js += "," + kb("tmplIsRight", tmpl_.isRight);
        js += "," + kb("ikIsRight", ikIsRight_);
        js += "," + kv("thumbRollRad", thumbRoll_.load());
        js += "," + kv("thumbPron0Rad", thumbPron0_.load());
        js += "," + kv("skelProcessed", double(skelProcessed_));
        js += "," + kv("skelSkipped", double(skelSkipped_.load()));
        char fp[32];
        std::snprintf(fp, sizeof(fp), "\"%08x\"", templateFingerprint(tmpl_));
        js += ",\"tmplFingerprint\":";
        js += fp;
        js += "}}";
        return js;
    }

    // 逐帧生效开关。位定义见 pcrec::RunFlagBit —— 【两处必须同步】，
    // 位序错开一位不会报错，只会让分析脚本读出一组看起来合理的错误开关。
    quint32 buildRunFlags(const SkeletonAssocDiag& d) const {
        quint32 b = 0;
        auto set = [&](bool on, int bit) { if (on) b |= (1u << bit); };
        set(backendReady_,            0);
        set(ikEnabled_.load(),        1);
        set(ikOnly_.load(),           2);
        set(chainCont_.load(),        3);
        set(filterOn_.load(),         4);
        set(rateLimitOn_.load(),      5);
        set(romLearning_.load(),      6);
        set(rom_.ready(),             7);
        set(quatOutOn_.load(),        8);
        set(autoCalibOn_.load(),      9);
        set(dorsumRigid_.load(),     10);
        set(geoRelabel_.load(),      11);
        set(netSegCalib_.load(),     12);
        set(thumbSegNet_.load(),     13);
        set(thumbPronOn_.load(),     14);
        set(capDebug_.load(),        15);
        set(jointMirrorAuto_.load(), 16);
        set(jointMirror_,            17);
        // 18 = 直通模式：worker 这一层看不到那个勾选框，由面板在写盘时补。
        set(fastMode_.load(),        19);
        (void)d;
        return b;
    }

    void fillAutoDiag(SkeletonAssocDiag& d) const {
        const auto& a = autoCalib_.result();
        d.autoStage             = int(autoCalib_.stage());
        // 【阶段跳变要打事件】自标定的每次阶段推进都会改变下游用的参数
        // （模板、anchor、骨长、手性），而这些变化的效果是从那一帧开始
        // 一直持续的。没有事件标记的话，事后只能在几千帧的标量曲线里
        // 用肉眼找拐点 —— 而拐点常常不明显，因为参数是渐变的。
        if (d.autoStage != lastAutoStage_) {
            const_cast<SkeletonAssocWorker*>(this)->pendingEvents_.push_back(
                {4, 0, autoCalib_.progress(), a.bundleRmseMm,
                 lastAutoStage_, d.autoStage,
                 std::string("自标定阶段 ") + std::to_string(lastAutoStage_) + " -> "
                     + std::to_string(d.autoStage) + "  " + autoCalib_.statusText()});
            const_cast<SkeletonAssocWorker*>(this)->lastAutoStage_ = d.autoStage;
        }
        // 【被拒也要打事件】"为什么标定了不生效"是高频问题，而拒绝原因
        // 原来只在 autoReject 这个逐帧字符串里，被后一帧覆盖掉就没了。
        const int rej = int(autoCalib_.lastReject());
        if (rej != 0 && rej != lastAutoReject_) {
            const_cast<SkeletonAssocWorker*>(this)->pendingEvents_.push_back(
                {3, 1, a.bundleRmseMm, lastAppliedRmse_, rej, 0,
                 std::string("自标定产物被拒：")
                     + hm20::autoCalibRejectName(autoCalib_.lastReject())});
        }
        const_cast<SkeletonAssocWorker*>(this)->lastAutoReject_ = rej;
        d.autoProgress          = autoCalib_.progress();
        d.autoText              = autoCalib_.statusText();
        d.autoReject            = hm20::autoCalibRejectName(autoCalib_.lastReject());
        d.autoTemplateReady     = a.backValid;
        d.autoIkUsable          = autoCalib_.ikUsable();
        // 【补第二把锁】attachIk() 原来只在"模板刚被自标定提交"那一瞬间被调用，
        // 是【边沿触发】的；而 ikUsable() 是随 bundleRmse 连续变化的【电平量】——
        // 自标定还在收敛、束调整还没跑完的那一刻恰好赶上模板提交，attachIk()
        // 就采样到一个偏高的 bundleRmse、判 paramsOk=false，把 IK 摘掉。
        // 之后 bundleRmse 慢慢降到远低于门限，但只要没有【下一次】模板提交，
        // 就没有任何代码会再去看一眼 ikUsable() 现在是不是已经变了——
        // IK 就这样被摘掉，永远摘掉。
        //
        // 这正是你这份录制里发生的事：ikUsable() 全程 100% 为 true，
        // 但 IK 从第 0 帧到最后一帧 applied 始终是 0% —— 唯一说得通的
        // 解释就是 attachIk() 在这段录制期间根本没被再调用过一次。
        //
        // 【为什么不干脆每帧都无条件调用 attachIk()】它内部还会碰
        // assoc_ 的其它状态；虽然目前看是安全的，但没必要每帧都走一遍
        // 那整段逻辑。改成只在 paramsOk 的电平【真的翻转】时才补调一次
        // setIkRefiner——检测本身是几个原子读 + 一次 bool 比较，
        // 代价可以忽略，而且不会引入 attachIk() 里其余代码路径的风险。
        {
            const bool paramsOk = !autoCalibOn_.load() || !autoTmplApplied_
                                 ? true : d.autoIkUsable;
            const bool wantIk = pendingIk_ && ikEnabled_.load() && paramsOk;
            if (wantIk != lastIkAttached_) {
                const_cast<SkeletonAssocWorker*>(this)->pendingEvents_.push_back(
                    {9, wantIk ? 0 : 1, a.bundleRmseMm, autoCalib_.ikRmseGate(),
                     lastIkAttached_ ? 1 : 0, wantIk ? 1 : 0,
                     std::string("IK 挂载状态电平复检翻转：") + (lastIkAttached_ ? "挂载" : "摘除")
                         + " -> " + (wantIk ? "挂载" : "摘除")
                         + "（bundleRmse=" + fmtNum(a.bundleRmseMm) + "mm，此前只在模板提交"
                           "那一帧采样过一次，之后 " + std::to_string(d.skelProcessed)
                         + " 帧里第一次被重新检查到）"});
                // 【复用 attachIk() 本身，不重复一遍它的判断逻辑】上面这次
                // paramsOk/wantIk 计算只用来决定"要不要打事件、值得不值得
                // 再调一次"，真正生效的挂载/摘除交给 attachIk() 自己去做——
                // 避免两处各写一份同样的条件，以后改了一处忘了改另一处。
                const_cast<SkeletonAssocWorker*>(this)->attachIk();
            }
        }
        for (int f = 0; f < 5; ++f) {
            d.anchorFitted[f]    = a.anchorFitted[size_t(f)];
            d.anchorResidMm[f]   = a.anchorFitted[size_t(f)] ? a.anchorResidMm[size_t(f)] : -1.0;
            d.anchorSpreadDeg[f] = a.anchorSpreadDeg[size_t(f)];
            d.anchorSamples[f]   = a.anchorSamples[size_t(f)];
            d.staticSeeded[f]    = a.staticSeeded[size_t(f)];
            d.staticSeedDevMm[f] = a.staticSeedDevMm[size_t(f)];
            d.anchorRigidStd[f]  = a.anchorRigidStdMm[size_t(f)];
        }
        d.autoHandednessKnown   = a.handednessKnown;
        d.autoIsRight           = a.isRight;
        d.autoIsRightDetected   = a.isRightDetected;
        d.autoHandednessConflict= a.handednessConflict;
        d.autoHandSignMm        = a.handSignMm;
        d.autoMirrorSuspect     = a.mirrorSuspect;
        d.jointMirrorActive     = jointMirror_;
        for (int f = 0; f < 5; ++f) d.ikRmsePerFinger[f] = lastIkRmse_[f];
        d.dorsumGeoMarginMm     = lastGeoMargin_;
        d.dorsumTrackN          = lastTrkN_;
        d.dorsumTrackFixed      = lastTrkFix_;
        d.hasAiPose             = lastHasAiPose_;
        d.hasAiHand             = lastHasAiHand_;
        d.aiHandKnown           = lastAiHandKnown_;
        d.aiHandIsRight         = lastAiHandRight_;
        d.aiHandConf            = lastAiHandConf_;
        d.aiHandLocked          = lastAiHandLocked_;
        for (int i = 0; i < 16; ++i) d.aiJointDeg[i] = lastAiJointDeg_[i];
        for (int i = 0; i < 5; ++i)  d.aiPoseConf[i] = lastAiPoseConf_[i];
        d.chainContinued        = lastChain_;
        for (int f = 0; f < 5; ++f) d.chainCoupledDeg[f] = lastChainCoupledDeg_[f];
        d.occludedHeld          = lastHeld_;
        d.calibRejected         = lastAcceptReject_;
        d.ikFilled              = lastIkFilled_;
        d.ikFallback            = lastIkFallback_;
        d.thumbFixed            = lastThumbFixed_;
        d.thumbFixSkip          = lastThumbFixSkip_;
        d.autoDorsumReordered   = a.dorsumReordered;
        d.autoDorsumWouldReorder= a.dorsumWouldReorder;
        d.autoBundleRmseMm      = a.bundleRmseMm;
        d.autoAttempts          = autoCalib_.attempts();
        d.thumbCoverage         = autoCalib_.thumbCoverage();
        d.thumbHint             = autoCalib_.thumbGuideHint();
        d.thumbAxialK           = a.thumbAxialK;
        d.thumbPronation0       = a.thumbPronation0;
        d.thumbPronationFitted  = a.thumbPronationFitted;
        d.thumbPronationContrast= a.thumbPronationContrast;
    }

    hm20::Hm20Template tmpl_{};
    // 标定值副本：手背复位要退回的目标，见 configure()/snapshotIkParams()。
    hm20::Hm20Template tmpl0_{};
    bool tmpl0Valid_ = false;
    std::array<hm20::Vec3, 5> ik0Back_{}, ik0Anchors_{};
    std::array<std::array<double, 3>, 5> ik0Lengths_{};
    bool ik0Valid_ = false;
    hm20::Hm20Template tmplPending_{};
    std::atomic<bool> tmplDirty_{false};
    hm20::Hm20Config cfg_{};
    std::string modelPath_;
    std::string fastPath_;               // INT8 那份的路径，空 = 没有高速模式可用
    std::string loadedModel_;            // 实际加载的，给 UI 显示
    int  intraOpThreads_ = 1;
    std::atomic<bool> fastMode_{false};
    std::atomic<bool> modelDirty_{false};
    // 实际加载的模型【文件名】，只在 ensureInitialized 里更新。
    // 只被 worker 线程读写（diag 填充也在 worker 线程），不需要加锁。
    QString loadedName_;
    std::shared_ptr<hm20::Hm20OnnxBackend> backend_;
    std::shared_ptr<hm20::Hm20SkeletonAssociator> assoc_;
    std::shared_ptr<hm20::IHm20IkRefiner> pendingIk_;
    std::atomic<bool> ikDirty_{false};
    std::atomic<bool> ikEnabled_{true};
    bool initTried_ = false;
    bool backendReady_ = false;
    std::string initError_;

    hm20::Hm20AutoCalib autoCalib_;
    hm20::AutoCalibConfig autoCfgPending_{};
    std::atomic<bool> autoCfgDirty_{false};
    std::atomic<bool> autoCalibOn_{false};
    std::atomic<bool> handIsRight_{true};
    std::atomic<bool> handDirty_{false};
    qint64 inFrameTsNs_ = -1;
    hm20::JointAngleResult ja_{};
    hm20::WristHold wristHold_{};
    // ---- ROM v2 ----
    // 【为什么整个换掉而不是补丁】v1 的三条隐含假设在真机上全部不成立，
    // 详见 RomCalibration.hpp 顶部。两个类都换了，接口刻意保持相似，
    // 但 apply() 多了一个 dofState 入参 —— 那正是 v1 缺的那块信息。
    hm20::rom::Calibrator rom_;
    hm20::rom::Mapper     romMap_;
    hm20::JointSolveDebug jdbg_{};
    // 本帧 ROM 采样对 16 维各自的判决，落进块 32。非标定期间全 0。
    std::array<uint8_t, 16> romReject_{};
    // romLearning_ 的上一帧值。用来在【采样真正开始/结束的那一帧】打事件 ——
    // 按钮事件（16/17）是 GUI 线程的墙钟，跟帧号之间隔着一个投递延迟。
    bool romLearnPrev_ = false;
    bool romAccumPending_ = false;
    qint64 romStateDumpNs_ = -1;
    int    romSnapReq_ = -1;   // 0=开始 1=1Hz 2=结束
    hm20::PredictedAngleSmoother angSmooth_;
    hm20::RateLimiter rate_;
    std::atomic<bool> romLearning_{false};
    std::atomic<bool> romStartReq_{false};
    std::atomic<bool> romFinishReq_{false};
    std::atomic<bool> quatOutOn_{true};
    std::atomic<bool> rateLimitOn_{true};
    std::atomic<double> thumbRoll_{1.40};
    std::atomic<bool>   thumbRollDirty_{true};
    std::atomic<bool>   thumbPronOn_{true};
    std::atomic<bool>   ikOnly_{true};
    std::atomic<bool>   geoRelabel_{true};
    std::atomic<bool>   dorsumRigid_{true};
    std::atomic<bool>   netSegCalib_{false};
    std::atomic<bool>   thumbSegNet_{false};
    std::atomic<bool>   capDebug_{false};
    std::atomic<bool> capFullAssign_{false};
    std::atomic<bool>   chainCont_{true};
    std::atomic<bool>   dorsumResetReq_{false};
    // 当前【在用的】那份模板的 bundleRmse，用来挡住更差的更新。
    double lastAppliedRmse_ = -1.0;
    double lastTmplRejected_ = -1.0;   // 最近一次被挡下的残差，给面板显示
    std::atomic<bool>   netSegSolved_{false};
    mutable std::mutex  stateM_;
    std::string         stateJson_ = "{}";
    QElapsedTimer       stateTimer_;
    bool                lastNetSegOk_ = false;
    std::string         lastNetSegWhy_ = "还没跑";
    double              lastNetSegCSpread_ = -1.0;
    std::array<double,3> lastNetSegKOffset_{{-1,-1,-1}};
    std::atomic<double> thumbPron0_{0.0};      // 0 = 沿用旧行为；标定后写入
    std::atomic<bool>   ikParamDirty_{true};
    bool   cfgAcceptGate_ = true;
    double acceptMaxRigidStdMm_ = 3.0;   // |anchor-pp| σ 上限；观测噪声是 1~2mm
    int    lastAcceptReject_ = 0;        // 0=通过 1=刚性σ不过
    // 关节角输出镜像：解算手性与物理手性不符时纠正屈曲类各维的符号。
    // jointMirror_ 只在 worker 线程读写，不需要原子；开关本身要跨线程。
    bool jointMirror_ = false;
    // IK refiner 当前用的手性。【必须单独跟踪】它是通过 ik->setHandedness()
    // 下发的，而 Hm20IkRefiner 没有 getter；更要紧的是 tmpl_.isRight 有【三条】
    // 独立的改写路径（面板 handDirty_、自标定 applyAutoCalib、dorsumReset 退回
    // 标定值），其中只有前两条会同步下发给 IK。三条路不同步时就会出现
    // "IK 按左手解、其余按右手走"，症状是拇指单独歪、其它手指正常 ——
    // 不把这个值单独记下来，这种情况在录制里完全不可见。
    bool ikIsRight_ = true;
    // ---- v5 ----
    // 【skelSkipped_ 必须是 atomic】tryAcquireSlot 在【主线程】调用，
    // 而读它的地方在 worker 线程。用普通 int 是数据竞争，而竞争出来的
    // 计数会偏小，正好是"看起来没怎么跳帧"的假象。
    std::atomic<int> skelSkipped_{0};
    int skelProcessed_ = 0;
    int tmplSnapPending_ = 0;      // 【初值 0】第一帧就写一份"录制开始"的模板
    std::vector<SkeletonAssocDiag::Event> pendingEvents_;
    QElapsedTimer tmplTimer_;
    QElapsedTimer cfgTimer_;
    int lastAutoStage_ = -1;
    int lastAutoReject_ = 0;
    int  lastIkFilled_ = 0, lastIkFallback_ = 0, lastChain_ = 0, lastHeld_ = 0;
    double lastChainCoupledDeg_[5] = {0,0,0,0,0};
    int    lastTrkN_ = 0, lastTrkFix_ = 0;
    double lastGeoMargin_ = -1.0;
    bool   lastHasAiPose_ = false, lastHasAiHand_ = false;
    bool   lastAiHandRight_ = true, lastAiHandLocked_ = false, lastAiHandKnown_ = false;
    double lastAiHandConf_ = 0.0;
    double lastAiJointDeg_[16] = {0};
    double lastAiPoseConf_[5] = {0,0,0,0,0};
    double lastIkRmse_[5] = {-1,-1,-1,-1,-1};
    int  lastThumbFixed_ = 0;      // 拇指回正的执行情况，由 processFrame 每帧写入
    int  lastThumbFixSkip_ = 0;
    std::atomic<bool> jointMirrorAuto_{true};
    double lastDtSec_ = 0.0;
    std::atomic<bool> autoResetReq_{false};
    bool autoTmplApplied_ = false;
    // 【IK 挂载状态的电平复检用】见 fillAutoDiag() 里那段说明——attachIk()
    // 本身是边沿触发的，这个成员让每帧都能便宜地判断"电平是否翻转了"，
    // 不需要每帧都重跑 attachIk() 的其余逻辑。初值 false 是安全的：
    // 真实状态第一次被复检到时，如果 wantIk=true，会正确触发一次挂载。
    bool lastIkAttached_ = false;

    hm20::Hm20PoseFilter filter_;
    hm20::PoseFilterConfig filtCfgPending_{};
    std::atomic<bool> filtCfgDirty_{false};
    std::atomic<bool> filterOn_{true};
    QElapsedTimer lastStamp_;
};

} // namespace mocap

Q_DECLARE_METATYPE(mocap::SkeletonAssocDiag)
Q_DECLARE_METATYPE(mocap::hm20::Hm20Template)
