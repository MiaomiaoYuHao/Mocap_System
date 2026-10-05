// ===========================================================================
// Hm20AssocContract.hpp —— hm20 关联的对外契约：ONNX 输入/输出名、推理后端
// 纯虚接口、IK 精修接口、SkeletonFrameResult、SegCanonAligner、Hm20Config、
// 拇指跟随器状态机(ThumbIpSwing/ThumbIpTracker)。
// 从 HandSkeletonAssociator.hpp 拆出（原第 376~1378 行）。
// ===========================================================================
#pragma once

#include <array>
#include <deque>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>
#include <cstdio>

// 点/矩阵别名、Label、detail 数学 在 Hm20AssocBase.hpp。
#include "estimate/Hm20AssocBase.hpp"
// matMul3 / matT3 / quatToMat3 / RotationAverager 在 Hm20SegRot.hpp。
#include "estimate/Hm20SegRot.hpp"

namespace mocap {
namespace hm20 {
// ===========================================================================
// ONNX 推理契约 —— 与 net.py / export_onnx.py 一一对应
// ===========================================================================
// 输入 6 个 tensor（顺序即 kOnnxInputNames）：
//   points      (1,N,3) float  候选点，【任意坐标系】，不需要预先转腕部系
//   mask        (1,N)   float  1=有效点，0=padding
//   tmpl        (1,61)  float  标定模板：中立位 20 点归一化坐标(60) + 左右手标志(1)
//   tmpl_valid  (1,20)  float  逐点模板有效位。【修改】原注释写的 (1,1) 是错的，
//                              与训练侧不一致。手背 5 个填 1.0（相位已由标定锁定），
//                              手指 15 个填 0.0（手指标定原理上做不到，也不需要）。
//   prev        (1,20,3)float  上一帧解出的 20 点（同 points 的坐标系）
//   prev_mask   (1,20)  float  1=上一帧该点可信
// 本层【消费】5 个输出（顺序即 kOnnxOutputNames）：
//   log_assign  (1,N+1,21) float  对数指派矩阵（最后一行/最后一列是 dustbin）
//   pos         (1,20,3)   float  20 点位置预测（含被遮挡的），与 points 同系
// 【注意】导出的 .onnx 里输出个数可能【多于】2 个——hm20_v6 / hm20_v6_sk10
// 实际是 6 个，另有 center / scale / ghost_logit / miss_logit 这几个训练用的
// 辅助头。onnxruntime 支持只请求输出子集，返回顺序 = 请求顺序，所以多余的头
// 完全不影响本层。后端【不要】拿输出个数做契约校验，只查名字（曾因此把正常
// 模型判成"不可用"，见 Hm20OnnxBackend.hpp::verifyContract）。
inline const std::array<const char*, 6>& kOnnxInputNames() {
    static const std::array<const char*, 6> n = {
        "points", "mask", "tmpl", "tmpl_valid", "prev", "prev_mask"};
    return n;
}
// 【v7 姿态版】在原来 2 个的基础上追加 3 个新头。
// 顺序就是 res[i] 的下标，加在【尾部】—— 前两个位置绝对不能动，
// 后端 run() 里是按 res[0]/res[1] 取的，插在中间会让所有旧代码静默错位。
//
// 新头的来历（训练侧 net.py）：
//   joint_ang  (1,20) 弧度  5指×4维。【注意是 20 不是 16】——训练侧跟
//              pose_prior 的 (5,4) 对齐，拇指也是 4 维；C++ 的 M3DS 16 维
//              是下游协议(拇指4+四指各3)，两者需要映射，见 jointAng20To16()。
//   hand_logit (1,)   >0 判右手。训练集准确率 98.5%，替代 Hm20AutoCalib 里
//              那套依赖拇指球的几何判据(拇指可见率只有 45%，经常判错)。
//   pose_conf  (1,5)  逐指姿态置信度。监督目标是"该指有几个 marker 可见/3"，
//              所以它直接就是"这一指的姿态该信几分"。
//
// 【v6 模型仍然能跑】后端按名字请求，缺的头会在 verifyContract 里被标出来，
// 对应字段保持 has*=false，上层退回原来的几何路径。
inline const std::array<const char*, 9>& kOnnxOutputNames() {
    // 【只能往尾部加】Hm20OnnxBackend 里到处用 slot("名字") 反查下标，
    // 但 Run() 的返回值是按这个数组的【顺序】排的，插在中间会让所有
    // 已有的解析整体错位。
    static const std::array<const char*, 9> n = {
        "log_assign", "pos", "joint_ang", "hand_logit", "pose_conf", "seg_rot6d",
        // 下面三个只给【离线复算】用，在线路径不消费：
        // center/scale 是网络内部的归一化参数，离线反归一化必须用这两个数，
        // 自己按点云重算会跟在线错位（网络用的是它自己算的那一份）。
        "center", "scale", "miss_logit"};
    return n;
}

struct InferenceInput {
    std::vector<Vec3> points;              // N 个候选点，任意坐标系（世界系直接给）
    std::array<float, 61> tmpl{};
    bool tmplValid = false;
    std::array<Vec3, 20> prev{};
    std::array<bool, 20> prevMask{};       // 全 false = 冷启动
};

struct InferenceOutput {
    // (N+1) x 21 的对数指派矩阵，行主序。行 N 是 dustbin 行。
    std::vector<float> logAssign;
    int rows = 0, cols = 0;
    std::array<Vec3, 20> pos{};            // 网络的 20 点位置预测

    // ---- v7 姿态版新增。v6 模型跑时这三个 has* 保持 false，上层照旧走几何 ----
    std::array<double, 20> jointAng{};     // 5指×4维，弧度。【20 不是 16】
    bool  hasJointAng = false;
    double handLogit = 0.0;                // >0 判右手
    bool  hasHandLogit = false;
    std::array<double, 5> poseConf{};      // 逐指姿态置信度(logit，未过 sigmoid)
    bool  hasPoseConf = false;

    // 16 段姿态，【手掌规范系】—— 不是世界系。
    // 实测(tools/hm20_diag/probe_thumb_axis.py 打在 hm20_v7 上)：把输入点云整体
    // 旋转平移之后 seg_rot6d 完全不变(seg0 与单位阵夹角 0.15°，旋转后变化 0.12°)，
    // 所以 segR[0] 恒为单位阵、其余是相对手掌的姿态。转世界系要：
    //     R_world(s) = wristR · C · segR[s]
    // C 是"你的模板系 -> 网络规范系"的常数，用 Hm20SegRot.hpp 里的
    // SegCanonAligner 从四指标一次即可。
    std::array<Mat3, kNumSegments> segR{};
    bool  hasSegR = false;

    // 网络内部的归一化参数 + 每点"本帧看不见"的 logit。只给离线复算用。
    Vec3   center{};
    double scale = 1.0;
    std::array<double, 20> missLogit{};
    bool   hasCenterScale = false;

    bool ok = false;
};

// -----------------------------------------------------------------------------
// 模型的 20 维关节角 -> C++ 侧 M3DS 的 16 维
//
// 【为什么维度不一样】训练侧跟 pose_prior.py 的 (5,4) 对齐：
//     拇指   [CMC屈, CMC展, MCP屈, IP屈]
//     非拇指 [MCP屈, MCP展, PIP屈, DIP屈]
// 而 M3DS 是 拇指4 + 四指各3(丢掉 DIP，下游用耦合系数推)。
// 这是两套【约定】不是两套数据，直接塞会全错位。
//
// 【符号约定要对齐】C++ 侧这轮修过两处：PIP 的 hinge 从 cross(a0,ẑ) 改成
// cross(ẑ,a0)(让屈曲为正)、左手镜像时 flexion 维取反。模型输出的是 rig 的
// 原始约定，没做这两步。所以接进 M3DS 之前【先在监视面板上并排看】，
// 确认符号和量级对得上再用 —— 否则会重现"握拳时 PIP 是负的"那个老问题。
inline std::array<double, 16> jointAng20To16(const std::array<double, 20>& a) {
    std::array<double, 16> o{};
    for (int i = 0; i < 4; ++i) o[size_t(i)] = a[size_t(i)];      // 拇指 4 维直通
    for (int f = 0; f < 4; ++f) {
        o[size_t(4 + 3 * f + 0)] = a[size_t(4 + 4 * f + 0)];      // MCP 屈
        o[size_t(4 + 3 * f + 1)] = a[size_t(4 + 4 * f + 1)];      // MCP 展
        o[size_t(4 + 3 * f + 2)] = a[size_t(4 + 4 * f + 2)];      // PIP 屈
        // a[4+4f+3] 是 DIP，M3DS 里没有这一维，丢弃（可用来校验耦合系数）
    }
    return o;
}

class IHm20InferenceBackend {
public:
    virtual ~IHm20InferenceBackend() = default;
    virtual InferenceOutput run(const InferenceInput& in) = 0;
};

// IK 一帧的逐指结果，回传给调用方做取舍和 UI 诊断。
//
// 【为什么必须逐指报】某根手指可见 marker < 2 时 IK 完全没有观测约束，解只由
// 先验和限位决定（= 中立位），跟真实姿势可能差 175°。调用方【必须】按
// fingerValid 决定要不要用 IK 的 segQuat，不能整帧一刀切。
struct IkFrameInfo {
    std::array<bool, 5>   fingerValid{};   // 该指 IK 解可信（可见 marker >= 2 且残差合格）
    std::array<double, 5> rmseMm{};        // 逐指 marker 拟合残差；NaN = 没有观测
    std::array<int, 5>    nObs{};          // 该指本帧被认领的 marker 数（0..3）

    // ---- v5：求解器内部 ----------------------------------------------------
    // 【为什么必须暴露这些】IK 现在对外只有上面三个数，而它内部做的事情里有
    // 两件直接关系到最难查的两类问题：
    //   · refine() 会按【自己的 isRight_】把左手镜像成右手系再解。这个
    //     isRight_ 跟 tmpl_.isRight 是两条独立的下发路径，不一致时症状是
    //     "拇指单独歪、其余正常" —— 这是手性出错的一条独立路径。只记
    //     ikIsRight 那个 bool 的话，只知道用错了手性，不知道错成了什么。
    //   · 解出来的 4 自由度角本身。它是遮挡 marker 位置的直接来源（FK 摆的），
    //     位置不对时要往上查一级，查的就是这四个角。
    //
    // 【limitHit 必须逐维记】撞限位的那一维被罚函数拉住不动，症状正好是
    // "这根手指弯到一半就停住"。而 rmse 可能仍然很小（其余维补偿了），
    // 所以只看残差永远发现不了。
    std::array<std::array<double, 4>, 5>  q{};         // 本帧解（腕部系，弧度）
    std::array<std::array<double, 4>, 5>  qPrior{};    // 起点（上一帧解）
    std::array<std::array<double, 4>, 5>  limitLo{};   // 实际生效的限位
    std::array<std::array<double, 4>, 5>  limitHi{};
    std::array<std::array<uint8_t, 4>, 5> limitHit{};  // 该维顶在限位上
    std::array<double, 5> cost{};                      // 最终代价
    std::array<int, 5>    iters{};                     // 实际迭代次数
    std::array<bool, 5>   solved{};                    // 真的跑了求解
    // 本帧实际用的模型参数。【必须逐帧记，不能只记配置】自标定会在线改它们，
    // 而"这一帧用的骨长是多少"才是解释这一帧结果的依据。
    std::array<std::array<double, 3>, 5> anchorMm{};
    std::array<std::array<double, 3>, 5> boneLenMm{};
    double handLenMm = -1.0;
    double thumbAxialK = -1.0;
    double thumbPronation0 = 0.0;
    double wPrior = -1.0, wLimit = -1.0, wCouple = -1.0, dipCoupling = -1.0;
    // 【遮挡兜底的 MCP->PIP 耦合，在线学习出来的】跟 dipCoupling 不同，
    // 这个是逐指的（钩握等手势下每根手指的独立程度不一样），而且样本量
    // 也要一起记——比例本身在样本不够时是默认值，不看样本数会把
    // "还没学出来、在用保守默认值"误当成"已经学出了这个精确比例"。
    std::array<double, 5> wMcpCouple{};
    std::array<double, 5> mcpPipCoupling{};        // 斜率 k
    std::array<double, 5> mcpPipCouplingB{};       // 截距 b（PIP≈k*MCP+b）
    std::array<int, 5>    mcpPipCouplingSamples{};
    // 【就绪位必须单独记】k=0 有两种完全不同的含义：一是"学出来斜率就是0"，
    // 二是"还没准备好、这条兜底根本没生效"。只看 k 的话分不开，而这两者
    // 对"遮挡时 PIP 为什么还是不动"的解释是相反的。
    std::array<bool, 5>   mcpCouplingReady{};
    bool isRight = true;
    bool paramsReady = false;
    bool applied = false;
};

// 遮挡点补全的 IK 精修钩子（可选）。不接的话默认直接用网络 pos 头的预测。
// 完整实现 = solve.py 的 Kabsch->逐指 LM 求 4 个关节角->FK，纯数学可直译。
class IHm20IkRefiner {
public:
    virtual ~IHm20IkRefiner() = default;
    // markers: 20 点（世界系），observed 标记哪些是真观测；就地精修未观测的点，
    // 并可回填 segQuat。返回 false 表示放弃精修（调用方保留网络预测值）。
    virtual bool refine(std::array<Vec3, 20>& markers, const std::array<bool, 20>& observed,
                        const Mat3& wristR, const Vec3& wristT,
                        std::array<Quat, kNumSegments>& segQuat) = 0;
    // 上一次 refine() 的逐指状态。默认全 false，老实现不用改也能编。
    virtual IkFrameInfo lastFrameInfo() const { return IkFrameInfo{}; }
    // 跟踪断了/换手时清时序先验，避免拿旧姿势把 IK 往错方向拉。
    virtual void resetPrior() {}
};

// ===========================================================================
// 结果
// ===========================================================================
struct SkeletonMarker {
    int label = Ghost;
    Vec3 posWorld{};
    bool observed = false;       // true=本帧真被观测到；false=网络/IK 补出来的
    double confidence = 0.0;     // 指派矩阵对应项的概率 exp(log_assign)
    int sourcePointId = -1;      // observed 时来自点云界面哪个点（UI 高亮用）
};

// 每个分段姿态是【怎么来的】。做动捕采集时下游必须能区分"测出来的"和
// "猜出来的"，否则一段 175° 的外推值会被当成真姿态写进 c3d/bvh。
enum class SegSource : int {
    None      = 0,   // 没有任何依据（冷启动/腕部位姿未解出）
    Predicted = 1,   // 两端 marker 都是网络 pos 头补的，只能当"大概在哪"用
    Geometry  = 2,   // 由真观测 marker 的连线算出（有 ~10° 贴球系统偏差）
    Ik        = 3,   // IK 解出（该指可见 marker >= 2 且残差合格），最可信
    Net       = 4,   // 网络 seg_rot6d 头直接给的姿态 + 标定出来的常数修正。
                     // 【不经过 marker 差分】—— 这是它相对 Predicted 的唯一优势：
                     // Predicted 是拿两个补出来的点做差，误差被骨长一除就放大；
                     // Net 不需要那两个点。默认关着，见 cfg_.thumbSegFromNet。
};

struct SkeletonFrameResult {
    bool valid = false;
    std::array<SkeletonMarker, kNumMarkers> markers{};
    std::array<Quat, kNumSegments> segQuat{};   // 16 个分段姿态（世界系）
    Mat3 wristR{1,0,0, 0,1,0, 0,0,1};
    Vec3 wristT{};
    Quat wristQuat{1, 0, 0, 0};
    int numObserved = 0;
    int numGhost = 0;                  // 被判为 dustbin 的候选点数
    double dorsumRmseMm = -1.0;        // Kabsch 残差；<0 表示腕部位姿没解出来
    bool wristPoseValid = false;
    // 拇指预测点回正的执行情况。thumbFixed = 本帧回正了几个点；
    // thumbFixSkip: 0=正常执行 1=参数为0 2=拇指三点全实测(不需要)
    //               3=无腕部位姿 4=手背<3点 5=已执行但用的是退化锚点 6=掌骨轴退化
    //               7=拇指部分可见(链条中间有实测点，转它会拧断链，交给 IK)
    int thumbFixed = 0;
    int thumbFixSkip = 0;
    // 遮挡点的位置来源分账：走 IK 摆出来的 / 退回网络 pos 头预测的。
    // 两者误差特性差很远，下游和排查都需要知道这一帧走的是哪条。
    int ikFilledMarkers = 0;
    int ikFallbackMarkers = 0;
    int chainContinued = 0;          // 本帧有几个遮挡点是顺着可见近端算出来的
    // 情形B（中节+远节都遮）里，掌心相对耦合相对"遮挡入口角"推出去多少 rad。
    // 【必须报出来】它恒为 0 有两种完全不同的原因：耦合关着(cfg_.occCouple)，
    // 或者近节相对手掌根本没动(手整体在平移)。前者是配置问题、后者是正常，
    // 不上报的话在面板上分不出来 —— 而"手指弯下去模型不动"正是要查这个。
    // 正号 = 比入口更屈，负号 = 更伸。索引是指号 0..4。
    std::array<double, 5> chainCoupledRad{};

    // ---- 链式续解的逐指内部状态（v6 新增）----
    //
    // 【为什么必须录】实测发现遮挡点里 81% 走链式续解、17% 直接回落到网络
    // 原始预测（src=4），而后者的侧偏完全不被纠正（实测无名指 61.2°->65.1°），
    // 是"指尖侧弯到不合理程度"最严重的一处。但那 87 帧的外部条件跟正常走
    // 链式续解的 130 帧【完全一样】（近节可见、中节不可见）——区别只在
    // 情形B 的四个守卫条件里某一个没过，而那四个量一个都没出过这个类。
    // 没有它们，"为什么这一帧链断了"就只能靠猜。
    //
    // 四个守卫（全部为真才走情形B，见 process() 里 caseB 的定义）：
    //   hasPipHold / hasPlane / pmLen>0 / wChain>0
    // 另外 wChain 本身是淡出权重：<1 时按 (1-wChain) 的比例把【网络原始
    // 预测】混回来——而网络预测正是侧弯的源头，所以 wChain 掉下来多少，
    // 就等于把多少侧弯混了回去。这个量是解释侧偏的直接依据。
    std::array<bool, 5>   chainHasPipHold{};
    std::array<bool, 5>   chainHasPlane{};
    std::array<double, 5> chainPmLenMm{};    // <=0 表示还没学到球间距
    std::array<double, 5> chainMdLenMm{};
    std::array<double, 5> chainWeight{};     // wChain，1=完全信链式解，0=完全回落网络
    std::array<double, 5> chainAging{};      // 老化量，wChain = clamp(1-aging)
    std::array<bool, 5>   chainCaseB{};      // 本帧这一指真的走了情形B
    std::array<bool, 5>   chainUseAnchor{};  // 用的 anchor 骨轴还是沿用上一帧
    std::array<int, 5>    chainCaseBFrames{};// 已经连续多少帧处在情形B
    std::array<bool, 5>   chainHasExc{};     // 有直接的 staleness 测量（否则退回帧数）

    // ---- anchor 下发/自检链路的状态（v6.1 新增）----
    //
    // 【为什么加这几个】实测发现 hasPlane 四指全为 0、链式续解 100% 失效、
    // 遮挡点 94.7% 直接回落到网络原始预测——而侧弯正是网络预测带进来的。
    // 追下去发现 learnJointPose() 开头就是 `if (!anchorsValid_) continue;`，
    // 也就是 anchorsValid_ 为假时【平面根本不学】，四个守卫里的 hasPlane
    // 永远为假，链式续解永远进不去。
    //
    // 而矛盾在于：模板快照里 anchorsValid=1、自标定 anchorFitted 五指
    // 全 100%、stage=5、progress=1.0，anchor 明明标好了；离线拿录制里的
    // 模板复算 verifyAnchors 的判据，四指也全部通过。也就是说
    // 【判据本身没问题，是它压根没走到、或者走到时用的不是这份数据】。
    //
    // verifyAnchors 有三个前置条件（anchorsSet_ / !anchorsChecked_ /
    // wristPoseValid），而 anchorsChecked_ 是【一次性锁】——判过一次就
    // 永不重试。这几个量不录出来，就只能继续猜。
    bool anchorsSet = false;        // setAnchorsMm 被调用过
    bool anchorsChecked = false;    // verifyAnchors 已经判过（一次性锁）
    bool anchorsValid = false;      // 自检通过、正在使用
    bool tmplMmValid = false;       // Kabsch 用的毫米模板已下发
    int  anchorsVerifyNSeen = -1;   // 上次自检时看到几根近节球（需 >=3）
    int  anchorsVerifyNOk = -1;     // 其中几根通过（需 >= nSeen-1）
    // 本帧有几个补出来的点被拉回了标定骨长。0 而遮挡点又不少，说明
    // pmLen_/mdLen_ 还没学到（要三点全可见过才学得到）。
    int boneLenSnapped = 0;
    double dorsumGeoRmseMm = -1.0;   // 几何重定的最优残差
    double dorsumGeoMarginMm = -1.0; // 次解与正解的残差差（判据的确定性）
    int dorsumGeoFixed = 0;          // 本帧被几何重定【换掉】的手背点数
    int dorsumTrackN = 0;            // 手背轨迹已累积帧数
    int dorsumTrackFixed = 0;        // 本帧被轨迹投票【纠正】的手背点数
    int dorsumGeoRelabeled = 0;      // 本帧被几何重定过标签(累计计数)
    // ---- 手背刚体求解（取代 relabelDorsumByGeometry + trackDorsum）----
    // 0=ok 1=候选点不足 2=无可行解 3=歧义拒解 4=残差超限
    int  dorsumSolveReason = 1;
    bool dorsumUsedHistory = false;  // true = 裕度不够，靠位姿连续性拍板的
    int  dorsumInliers     = 0;      // 本帧刚体解用了几个内点(3..5)
    // 本帧有几个【手指标签】因为它占的候选点被手背几何要回去而被退成未观测。
    // 【必须报出来】这个数长期非 0 说明网络在手背/手指边界上判错得很稳定，
    // 是模型问题，不是这一帧的偶然。0 是正常态。
    int  dorsumStoleFromFinger = 0;
    // 本帧有几对标签共用同一个 sourcePointId。硬不变量，正常恒为 0。
    int  dupClaimCount = 0;
    // 本帧有几个候选点因为离手部中位点太远被挡在网络之外。
    // 【必须报出来】正常恒为 0；一旦非 0，说明上游三角化吐出了物理上不可能的点。
    // 真机实测：一个 11192mm 的点（两视图、残差恒 0）会把网络的 scale 从 61mm
    // 顶到 2441mm，输出退化成 21 类均匀分布，整帧 20 个点全被置信度闸丢掉。
    // 挡住是治标；治本要在 Triangulation 里按视线夹角(parallax)拒收两视图退化解。
    int  assocOutliersDropped = 0;

    // ---- 逐指 anchor / 弯曲平面的现场 -------------------------------------
    // 【为什么这三个必须单列】anchorsValid_ 是一个全局 bool，它说不出
    // "五根里哪几根降级了"；chainHasPlane 是个 bool，它说不出"平面是从
    // 几帧学来的"。而改完之后如果误差还压不下去，第一件要分辨的事恰恰是：
    //   · 平面根本没学到（planeSampleN 很小 -> 这根手指的 dp 长期被遮）
    //   · 平面学到了但 anchor 没降级（anchorOk 仍为真 -> 判据阈值不对）
    //   · 两者都对，那问题在别处（轴对了、角度不对，或者根本不在这条链上）
    // 没有这三个数，这三种情况在输出上长得一模一样。

    // 逐指 anchor 自检结论。false = 已退回 pp->mp（纯实测量，不依赖标定）。
    std::array<bool, 5> anchorOkPerFinger{};
    // 弯曲平面的累加器吃进了多少帧。平面【只】从 pp/mp/dp 三点全见的帧学，
    // 所以这个数同时也是"这根手指有多少帧是三点全见"的直接计量。
    // 个位数就意味着平面基本没学到，此时 chainHasPlane 为真也不能信。
    std::array<int, 5>  planeSampleN{};
    // 本帧实测的 anchor->pp 出平面角(度)。这是整条诊断链的核心量：
    // cross(u0,u1) 对 u0 的出平面分量有 1/sin(MCP角) 的放大，实测
    // 出平面 17.8~59.7° -> 学到的平面歪 18.8~77.9°。
    // 【放进每帧输出，是为了不用再离线重算一遍】anchor 没在用、或本帧
    // 三点不全时为 -1。
    std::array<double, 5> anchorOutOfPlaneDeg{};
    int  netThumbSegs      = 0;      // 本帧有几段拇指用了网络 seg_rot6d
    // 本帧就地修复了哪个手背槽位（0=没修，1..5 = 槽位号+1）。
    // 【必须报出来】静默改模板很危险：万一判错，之后所有帧都基于一个错模板，
    // 而且不留任何痕迹。
    // 连续"解不好"的帧数；以及本帧看门狗做了什么（0=没动 1=松锁 2=退回模板）。
    // 【这两个是"必须重开面板"那类问题的入口】badStreak 一直涨说明卡住了。
    int    dorsumBadStreak   = 0;
    int    dorsumRelock      = 0;
    // 本帧微调过模板（点位小幅漂移的持续重适应）。累计值看 dorsumTmplDriftMm。
    // 几何手性的锁定值（0=未锁定）。跟模型的 aiHandIsRight 并列，
    // 由上层对照 —— 两者不一致时该提示用户，而不是悄悄按某一个走。
    int    geoHandSign       = 0;
    int    dorsumRefined     = 0;
    int    dorsumRepaired    = 0;
    double dorsumRepairMoveMm = -1.0;
    // 拇指 IP 的遮挡期预测：当前角度和本次遮挡累计的积分量（rad）。
    // 【看 drift】它恒为 0 说明摆动预测器没在跑（样本不够或 chainContinue 关着）。
    double thumbIpPredRad  = -99.0;
    double thumbIpDriftRad = 0.0;

    // ---- 离线复算用的原始流（只在 cfg_.captureDebugStreams 打开时填）----
    //
    // 【为什么必须记这两样】离线跑不出跟在线一样的结果，任何调参结论都不可信 ——
    // 你分不清差异是参数带来的还是复现本身错了。而复现的最小闭包恰好是这两处：
    //   · 送进网络的候选点【及其顺序】：网络的指派对输入顺序敏感，
    //     而 Points3D 记的是追踪器输出，中间还隔着筛选和 maxCandidates 截断。
    //   · 网络的原始输出：这是"模型判错了"和"后处理搞砸了"的唯一分界线。
    //     没有它，看到一个歪掉的拇指姿态，你分不清是 pos 头本来就给歪了、
    //     还是后处理把链拧断了 —— 而这两件事的修法完全相反。
    //
    // 默认关：常态运行不该为调试付出拷贝代价。录制开始时打开即可。
    bool   hasDebugStreams   = false;
    int    assocInputN       = 0;    // 实际送进网络的候选点数
    int    assocInputBeforeCap = 0;  // 截断前有多少，两者不等说明有点被丢了
    std::array<Vec3, 32> assocInputPos{};
    std::array<int, 32>  assocInputId{};
    // 每个候选点的 top-3 标签和概率。不记全量 log_assign：(N+1)x21 float
    // 在 120fps 下是 200KB/s，而 99% 的分析只要 top-3。
    std::array<std::array<int, 3>, 32>    assignTop3Label{};
    std::array<std::array<double, 3>, 32> assignTop3Prob{};
    // 网络的其余头。center/scale 【必须记】—— 离线反归一化自己重算会错位。
    std::array<Vec3, kNumMarkers>   netPos{};
    Vec3   netCenter{};
    double netScale = 1.0;
    std::array<double, kNumMarkers> netMissLogit{};
    std::array<double, kNumMarkers> netJointAng{};
    double netHandLogit = 0.0;
    std::array<double, 5> netPoseConf{};
    std::array<Mat3, kNumSegments> netSegR{};
    bool   netHasSegR = false;

    // ---- v7 姿态版：模型直接给出的姿态/手性，不经过任何几何链 ----
    // 【为什么原样透传】几何链在观测充分时更准(实测张开手中节 5° 以内，
    // 模型是 7.3°)，遮挡时则远差(骨轴中位 21.4°、max 107.9°)。
    // 所以不在这一层做取舍，把两者都给上层，让它按 poseConf 加权。
    std::array<double, 20> aiJointAng{};    // 5指×4维，弧度。用 jointAng20To16 转
    bool hasAiJointAng = false;
    std::array<double, 5> aiPoseConf{};     // 逐指置信度(logit)
    bool hasAiPoseConf = false;
    // 【三态，不是二选一】aiHandKnown=false 时 aiHandIsRight 【没有意义】，
    // 显示端必须报"未定"而不是把它当成一个答案。
    // 真机实测 hm20_v7 的 |hand_logit| 中位只有 0.044，而纯随机点云能给到
    // 0.196 —— 在这个量级上符号跟噪声分不开。没有"未定"这个状态，就只能
    // 在两个都没依据的答案里挑一个，然后还标上"已锁定"。
    bool aiHandKnown = false;               // 证据是否足以下结论
    bool aiHandIsRight = true;              // 仅当 aiHandKnown 时有意义
    double aiHandConf = 0.0;                // 0~1，累加值 / 同号累积上限
    bool aiHandLocked = false;              // 已连续 30 帧一致，可以采信
    bool hasAiHand = false;
    int occludedHeld = 0;            // 有几个因发散/只信IK 被改成保持上一帧
    bool pentagonOk = false;
    std::string message;

    // ---- 逐分段可信度（本次新增）----------------------------------------
    // 【为什么加】原来 segQuat 是 16 个无差别的四元数，调用方无从判断哪一个
    // 是"网络补的两个点连出来的"。做动捕这一点是致命的：一根全遮挡的手指
    // 照样会输出一个看起来很正常的四元数。
    std::array<SegSource, kNumSegments> segSource{};
    std::array<double, kNumSegments>    segConf{};      // 0..1，粗略可信度
    std::array<bool, 5>   fingerIkValid{};              // 该指本帧走了 IK
    std::array<double, 5> fingerIkRmseMm{};             // IK 拟合残差（mm）
    std::array<int, 5>    fingerObsCount{};             // 该指被认领的 marker 数
    bool ikApplied = false;                             // 本帧至少有一根手指用了 IK

    // ---- 逐 marker 的位置来源（本次新增）--------------------------------
    // 【为什么 observed 那个 bool 不够】一个"没被观测到"的点，位置可能来自
    // 四条完全不同的路：IK 解的、顺着可见近端链式续解的、网络 pos 头兜底的、
    // 或者干脆保持上一帧。四者的误差特性差一个数量级 ——
    // 实测 IK 骨轴 max 71°、网络预测 max 108°、而"保持"是无限滞后。
    // 现有的 ikFilledMarkers / chainContinued / occludedHeld 都是【计数】，
    // 回答不了"是哪个点走的哪条路"，而排查时问的恰恰是后者。
    //
    // 0=未知 1=实测 2=IK补 3=链式续解 4=网络预测兜底 5=保持上一帧/发散限速
    std::array<uint8_t, kNumMarkers> markerSource{};
    // bit0=骨长回正过 bit1=预测点平滑过 bit2=发散限速过 bit3=拇指回正过
    std::array<uint8_t, kNumMarkers> markerFlags{};

    // ---- v5：位置的中间级快照 -------------------------------------------
    //
    // 【为什么位置也要分级，而且比角度更该分】关节角已经拆成了四级，因为
    // "握拳输出像张开"每一级都能单独造成。位置这一路完全同理，而且它在
    // 角度的【上游】—— 位置错了角度必然跟着错，查的时候应该先看位置。
    // 但原来位置只有两级：netPos（网络原始）和最终的 markers[].posWorld。
    // 中间四级全是就地覆写，一步都留不下来，于是
    //     "指派把标签给错了"
    //     "标签对、几何骨轴算歪了"
    //     "几何对、IK 把遮挡点摆错了"
    //     "都对、骨长回正/发散限速把点拽跑了"
    // 这四种在文件里长得一模一样，而修法完全不同。
    //
    // 下标：0=Net 1=Assoc(指派+手背重定) 2=Geom(拇指回正+骨轴) 3=Ik 4=Post(平滑+骨长)
    // 第 5 级（滤波后）不在这里 —— 滤波是 worker 里做的，由 worker 补上。
    //
    // 【stageValid 不能省】某一级没跑（比如 IK 关着）时，stagePos 是上一级的
    // 拷贝。"没跑"和"跑了但没动这个点"含义完全不同，只有这个 bool 分得开。
    std::array<std::array<Vec3, kNumMarkers>, 5> stagePos{};
    std::array<bool, 5> stageValid{};

    // IK 求解器本帧的完整内部状态（见 IkFrameInfo 的说明）
    IkFrameInfo ikInfo{};

    // 全量指派矩阵。rows × cols 的 log 概率，行优先。
    // 【为什么 top-3 不够】查"这个点为什么被判成鬼点"时，需要看它在全部 21 类
    // 上的分布：分布平坦 = 模型没主意（该去看点质量/标定），次高很接近 =
    // 被别的点抢走了（该去看指派和阈值）。两者的修法不同，而 top-3 里
    // 只看得到前三个数，看不出分布形状。
    // 【只在 captureFullAssign 打开时填】120fps 下约 200KB/s。
    std::vector<float> logAssignFull;
    int logAssignRows = 0;
    int logAssignCols = 0;
};

// ===========================================================================
// 主类
// ---------------------------------------------------------------------------
// 标定 C（模板系 -> 网络规范系）和 K_s（拇指三段的常数修正）
//
// 用法（每帧，在 process() 之后；inf 是那一帧的 InferenceOutput）：
//     aligner.feed(result, inf);
// 攒够之后：
//     Mat3 C; if (aligner.solveC(C)) { ... }
//     std::array<Mat3,3> K; if (aligner.solveThumbK(C, K)) { ... }
//
// 然后拇指遮挡帧用：
//     R_world(s) = wristR · C · segR[s] · K[s-1]      (s = 1,2,3)
// ---------------------------------------------------------------------------
class SegCanonAligner {
public:
    struct Config {
        int    minSamples   = 200;
        double maxSpreadDeg = 15.0;   // 超过它说明假设不成立，别用
    };
    void setConfig(const Config& c) { cfg_ = c; }
    void reset() { cSamp_.clear(); for (auto& v : kRaw_) v.clear(); }

    int  cCount() const { return int(cSamp_.size()); }
    int  kCount() const { return int(kRaw_[0].size()); }

    // 【只收干净帧】C 用四指(它们的几何解可信)，K 用拇指三颗球全见的帧。
    void feed(const SkeletonFrameResult& r, const InferenceOutput& inf) {
        if (!r.valid || !r.wristPoseValid || !inf.hasSegR) return;
        const Mat3 Wt = matT3(r.wristR);

        // ---- C：四指的几何段。C = (wristRᵀ · R_geo(s)) · segR[s]ᵀ ----
        for (int f = 1; f < 5; ++f) {
            for (int j = 0; j < 3; ++j) {
                const int s = 1 + 3*f + j;
                if (r.segSource[size_t(s)] != SegSource::Geometry) continue;
                const Mat3 Rg = quatToMat3(r.segQuat[size_t(s)]);
                cSamp_.push_back(matMul3(matMul3(Wt, Rg), matT3(inf.segR[size_t(s)])));
            }
        }
        if (cSamp_.size() > 4000) cSamp_.erase(cSamp_.begin(), cSamp_.begin() + 2000);

        // ---- K：拇指三段，只在三颗球全见时收 ----
        bool thumbFull = true;
        for (int m = 5; m <= 7; ++m) if (!r.markers[size_t(m)].observed) thumbFull = false;
        if (thumbFull) {
            for (int j = 0; j < 3; ++j) {
                const int s = 1 + j;
                if (r.segSource[size_t(s)] != SegSource::Geometry) continue;
                kRaw_[size_t(j)].push_back({quatToMat3(r.segQuat[size_t(s)]), r.wristR,
                                            inf.segR[size_t(s)]});
                if (kRaw_[size_t(j)].size() > 2000)
                    kRaw_[size_t(j)].erase(kRaw_[size_t(j)].begin(),
                                           kRaw_[size_t(j)].begin() + 1000);
            }
        }
    }

    bool solveC(Mat3& C, double* spreadDeg = nullptr) const {
        if (int(cSamp_.size()) < cfg_.minSamples) return false;
        RotationAverager ra;
        for (const Mat3& R : cSamp_) ra.add(R);
        if (!ra.mean(C)) return false;
        const double sp = ra.spreadDeg(cSamp_);
        if (spreadDeg) *spreadDeg = sp;
        return sp >= 0.0 && sp <= cfg_.maxSpreadDeg;
    }

    // K[j] 对应 segment 1+j（拇指 掌骨/近节/远节）
    bool solveThumbK(const Mat3& C, std::array<Mat3, 3>& K,
                     std::array<double, 3>* spreadDeg = nullptr) const {
        for (int j = 0; j < 3; ++j) {
            const auto& v = kRaw_[size_t(j)];
            if (int(v.size()) < cfg_.minSamples) return false;
            RotationAverager ra;
            std::vector<Mat3> samp;
            samp.reserve(v.size());
            for (const Raw& x : v) {
                // K = (wristR·C·segR)ᵀ · R_geo
                const Mat3 pred = matMul3(matMul3(x.wristR, C), x.segR);
                const Mat3 k = matMul3(matT3(pred), x.Rgeo);
                samp.push_back(k);
                ra.add(k);
            }
            if (!ra.mean(K[size_t(j)])) return false;
            const double sp = ra.spreadDeg(samp);
            if (spreadDeg) (*spreadDeg)[size_t(j)] = sp;
            if (sp < 0.0 || sp > cfg_.maxSpreadDeg) return false;
        }
        return true;
    }

    // 诊断用：K 相对单位阵的转角。如果拇指先验没问题它该接近 0；
    // 实测应该是 70~90° —— 那正是 rig 缺掉的常数旋前。
    static double offsetDeg(const Mat3& K) {
        return RotationAverager::angleDeg(Mat3{{1,0,0, 0,1,0, 0,0,1}}, K);
    }

private:
    struct Raw { Mat3 Rgeo; Mat3 wristR; Mat3 segR; };
    Config cfg_{};
    std::vector<Mat3> cSamp_;
    std::array<std::vector<Raw>, 3>  kRaw_;
};


// ===========================================================================
struct Hm20Config {
    // 低于此概率的指派当作没认领，走补全。
    //
    // 【2026-08 修改：手背和手指必须分开设，原来共用一个是有害的】
    // 手指点的 top1 概率中位约 0.97，手背只有 0.50 左右 —— 不是因为手背判得
    // 差，而是手背 5 点近似五重对称，概率天然摊在几个相邻标签上，即便 argmax
    // 是对的，峰值也上不去。共用 0.55 的后果：实测 70% 以上的手背点被判进
    // dustbin，于是"手指连得好好的、手背一个都不认"，而且手背可见点 <3 会让
    // Kabsch 整段跳过 -> wristPoseValid 恒 false -> 腕部位姿和所有段朝向的
    // 参考系一起没了。
    // 0.55/0.30 这两个值来自离线阈值扫描（也是 PointCloudTestDialog 里原注释
    // 记的那组推荐值，只是当时没有字段可以分开设）。
    double minAssignProbFinger = 0.55;   // 手指 15 点（标签 5..19）
    double minAssignProbDorsum = 0.30;   // 手背 5 点（标签 0..4）
    bool useTemplate = true;
    bool usePrevFrame = true;      // 100~200fps 下这是信息量最大的输入，强烈建议开
    // 拇指三段的 roll 偏置（弧度）。0 = 沿用旧行为（拇指跟四指同一个 roll 参考）。
    // 默认 1.40 ≈ 80°，取自第一掌骨的解剖旋前量。这是一阶近似：真人个体差异
    // 在 ±15° 量级，而且旋前会随 CMC 外展变化 —— 但比 0 好得多。
    // 【怎么调】弯拇指，看面板上"拇 远节"相对父节点的四元数：轴应该以 z 为主
    //（屈曲），如果 x 分量很大说明还在打滚，加减 0.2 试。设 0 可完全退回旧行为。
    double thumbRollOffsetRad = 1.40;
    // 拇指【预测点】的旋前回正量（弧度）。0 = 关闭，只用网络原样输出。
    // 跟 thumbRollOffsetRad 是同一个物理量(rig 缺失的那个常数旋前)，但作用在
    // 不同输出上：这个改【位置】，那个改【四元数的 roll】。两者都需要，不是重复。
    // 默认 1.40 ≈ 80°。这是建模参数，没有观测能验证它，只能对着画面调。
    double thumbPronationRad = 1.40;

    // 【IK 用来干什么】两件事可以分开开关，因为实测收益方向相反：
    //   · 分段朝向：IK 明显更准（标定过的手 13.2°->8.8° 中位），值得开。
    //   · 遮挡点位置：IK 反而更差（5.8mm->7.7mm）。网络的 pos 头是端到端训出来
    //     的、见过整个姿势分布；IK 在该指只剩 2 个可见点时是纯外推，还受群体
    //     均值贴点参数的限制。所以默认【不让 IK 改位置】。
    // 只有当该指 3 个点全可见、且 IK 残差很小时，IK 补位才可能比网络好 ——
    // 但那种情况下本来也没有遮挡点要补。所以默认 false 是有依据的，不是保守。
    bool ikOverrideQuat    = true;    // IK 覆盖分段朝向
    // 【默认改为 true】遮挡点优先用 IK 摆出来的位置，而不是网络 pos 头的预测。
    // 仿真实测(真模型闭环 + 3相机 + 32%遮挡，tools/sim_jitter.py)：
    //                    骨轴中位  骨轴p90  骨轴max  拇误差  四误差  拇跳p90
    //   AI预测(原默认)       21.4    55.2   107.9   28.2   29.6   11.28
    //   IK补点(已标定)       27.4    63.0    71.3   10.0   10.3    6.36
    //   IK补点(免标定)       30.8    70.8    91.4   37.6   35.7   20.64  <- 全面更差
    //
    // 【为什么选 IK 而不是骨轴中位更好的 AI 预测】看 max 那一列：IK 摆出来的点
    // 天然满足骨长恒定和关节限位，物理上不可能出现 107° 那种翻转；中位差的
    // 那 6° 屏幕上看不出来，一次 107° 一眼就看见，驱动机械手还要出事。
    //
    // 【前提：必须标定过】免标定档七项输六项 —— 拿群体均值手型去摆你的手，
    // 系统性偏移比网络预测还大。所以自标定没到 Ready 时不该开这一路
    // （fingerIkValid 会自然为 false，逐指退回 AI 预测，见 ikFallbackMarkers）。
    bool ikFillOccluded    = true;    // IK 覆盖被遮挡 marker 的位置
    // 【遮挡点只信 IK】IK 补不上的遮挡点，保持上一帧位置，不用网络 pos 头预测。
    // 【为什么要这个】关节角那边一旦 fingerValid=false 就整根手指"保持"，
    // 而 marker 位置走的是另一条路(网络预测)，于是出现"角度冻住、点在飞"
    // 这种自相矛盾的状态 —— 角度都判定为不可信了，位置却还照单全收。
    // 打开之后两者对齐：要么都是新算的，要么都冻住。冻住至少物理上连续。
    // 【默认关 —— 这是回退】打开时 IK 补不上的遮挡点会【冻住】，而实测表明
    // 网络 pos 头对四指的外推是好用的，冻住反而更差（"预测点都不会外推了"）。
    // 现在遮挡点的优先级是：
    //   ① IK 补点   ② 链式续解   ③ 网络 pos 头预测   ④ 冻住(仅防发散)
    // ④ 不再是"IK 补不上就冻"，而是【只在网络预测明显发散时】才接管 ——
    // 见 occludedJumpGateMm。冻结是安全网，不是默认路径。
    bool occludedIkOnly    = false;
    // 网络预测的帧间跳变超过它就判为发散，改用保持。0 = 不设防。
    // 40mm/帧 在 45fps 下等于 1.8m/s，手指不可能有这个速度，超了必是发散。
    double occludedJumpGateMm = 40.0;
    // 远端被遮挡时顺着可见近端闭式续解（共面 + 标定骨长 + DIP 耦合）。
    // 见 continueChainForOccluded()。关掉 = 退回"冻住"。
    // 【默认关 —— 仿真判定在任何配置下都不如现状】
    // 中节骨轴误差(tools/test_chain_axis.py，真模型闭环)：
    //              3相机 中位/p90/max      4相机 中位/p90/max
    //   现状(差分)   26.5 / 124.5 / 140.5    14.8 / 42.3 /  84.2
    //   情形A(链式)  38.4 /  70.9 /  94.3    46.7 / 70.4 / 103.7   <- 中位全面更差
    //   情形B(外推)  43.8 / 114.5 / 131.2    42.7 / 90.1 / 122.5   <- 更差
    //
    // 根因是 DIP 耦合假设太粗：dip = k*pip 里的 k 是个平均值，而真人的
    // DIP/PIP 比例随个体和姿势变化很大。用它推出来的远节方向，误差比
    // "两个带噪点做差分"还大。而且情形A 在真实遮挡里很罕见(中节远节通常
    // 一起丢)，样本只有 34/40，收益面本来就小。
    //
    // 代码保留，作为已否决的实验记录；想复现打开即可。
    // 【2026-08 改默认为 true】这个开关关着时 continueChainForOccluded() 整个
    // 函数从不执行，所有遮挡点直接退回网络 pos 头。真机实测的后果：
    // 拇指指尖一被遮，预测的 IP 角从 ~60° 塌到 8~20°（几乎伸直），而遮挡本身
    // 就是高屈曲造成的，那一刻真值至少 45°。四指同理 —— 中节相对近节被预测出
    // 117~130° 这种超过 PIP 解剖限位(110°)的角度。
    // 而且 setChainContinue() 从 UI 到 worker 一路都没接出来，等于永远打不开。
    bool chainContinue     = true;
    // 段坐标系的 roll 沿链继承（true）还是每节独立用手背法向（false）。
    // 见 swingTo() 的说明：独立构造在外展出平面时会凭空造出扭转，
    // 而且骨轴接近手背法向时有退化分支（握拳时会撞上，实测单帧跳到 172°）。
    // 默认 true。留 false 是为了能 A/B 对照，确认之后可以删掉旧路径。
    bool chainRoll         = true;
    // 几何手性：弯折角小于这个值的骨节不投票（没信息，只会稀释真信号）。
    // 真机实测握紧时可见段的 PIP 夹角中位只有 12°，所以门限要明显高于它。
    double geoHandMinBendDeg = 25.0;
    // 累积量超过它就锁定。实测两段素材都在 11~32 帧（约半秒）内锁定，
    // 之后 100% 一致。调大更稳但锁得慢。
    double geoHandLockThresh = 4.0;
    // 情形B（中远节都丢、冻结姿势续解）最多撑多少帧。
    // 24fps 下 48 帧 = 2 秒 —— 短时遮挡冻结完胜网络 pos 头，
    // 但姿势【真的变了】才会旧 —— 老化按偏离算，见 occStaleRad0 那段实测表。
    // ---- 情形B 的"放手"判据 ----------------------------------------------
    // 【按姿势偏离老化，不按帧数】原来是 caseBFrames_ > 48 就硬切回网络预测。
    // 实测（合成，食指中节+远节全遮）：
    //
    //   手完全静止      0-49帧  50-99  100-149 150-199 200-249  250+
    //     超时48 误差     0.36    6.14    5.88    6.40    5.95    6.28 mm
    //     不超时 误差     0.36    0.33    0.29    0.39    0.34    0.30 mm
    //   遮挡期持续弯
    //     超时48 误差     1.54    5.83    7.47    7.52    7.01    7.14 mm
    //     不超时 误差     1.54    4.51    7.19    9.66   12.25   14.50 mm
    //
    // 两条结论：
    //  ① 静止时链式解【永远】优于回退（0.3mm vs 6.1mm，20 倍），按帧数老化
    //     没有任何道理 —— 而用户握着不动正是最常见的情形。
    //  ② 只有姿势真的在变，pipEntry_/planeLocal_ 才会变旧；交叉点在【约 120
    //     帧持续弯曲】处，不是 48 帧。
    //
    // 而每帧的转角增量分不出静止和运动（实测 0.012~0.015 vs 0.009~0.012 rad，
    // 全被噪声淹没）。能分出来的是【相对入口的绝对偏离】—— 它不累积噪声：
    // 静止时停在 ±0.012 rad，持续弯 100 帧涨到 0.35 rad。
    //
    // 所以老化量 = |chainCoupledRad|，在下面这个区间里线性淡出。
    double occStaleRad0 = 0.35;   // 偏离小于它：完全信链式解
    double occStaleRad1 = 0.70;   // 大于它：完全交回网络预测
    // 帧数只作兜底（拿不到驱动量时偏离恒为 0，得有个别的出口），
    // 同样是淡出不是硬切：caseBMaxFrames ~ 2*caseBMaxFrames 之间线性。
    int    caseBMaxFrames = 600;

    // ---- 弯曲平面的学习门限 ----------------------------------------------
    // planeLocal_ 是 cross(近节轴, 中节轴)。两轴接近共线时这个叉积【只剩噪声】，
    // 方向绕骨轴乱转 —— 而它随后被当成屈曲轴用，绕它转 dip 角就把远节甩到
    // 侧向去。真机症状正是"食指最远端朝中指方向侧弯"。
    // 原来的门限是 |cross| > 1e-3，对单位向量就是【夹角 > 0.06°】，等于没有门限。
    // 手指伸直的帧远多于弯曲的帧，于是学到的平面长期由噪声主导。
    // 14° 是取"能量比 = sin²(14°) ≈ 6%"，低于它认为叉积不可信。
    double planeLearnMinDeg = 14.0;

    // ---- 遮挡期的掌心相对耦合（近节可见时推远端继续弯）--------------------
    // 见 continueChainForOccluded 情形B 里的说明。
    bool   occCouple        = true;
    double occCoupleGain    = 1.0;    // Δ(PIP) / Δ(近节相对手掌的转角)
    double occCoupleMaxRad  = 1.20;   // 相对入口角的最大推算幅度（约 69°）

    // ---- 手性投票的证据门限 ------------------------------------------------
    // 【为什么不是一个绝对阈值】我先写的是 handVoteHyst = 1.5 的绝对死区，
    // 那是错的：真机实测 |handLogit| 中位只有 0.044，conf = tanh(0.044) ≈ 0.044，
    // 累加器稳态 |acc| = conf/(1-0.92) ≈ 0.55 —— 【永远够不到 1.5】。
    // 于是判决会一直停在初值上，把"偶尔错判"变成"永久锁死"。
    //
    // 门限必须跟着模型实际给出的置信度走：要求累加器达到"同号连续累积"
    // 理论上限的一定比例。这样模型自信时门限自然抬高、模型没把握时也不会
    // 因为够不到而卡在初值上。
    double handVoteRel = 0.60;

    // 置信度地板：低于这个平均 conf 就【什么都不判】。
    //
    // 【第一版我把这个数设错了】写的是 0.15，理由记的是"随机点云 0.196"——
    // 可 0.15 【小于】0.196，地板设在了噪声本底【之下】。加上判据本身是
    // 【相对】的（|acc| >= 0.6*cap），只要符号一致这个比值必然趋近 1.0，
    // 再弱的信号也能开门。于是模型稳定输出 +0.2 这种弱正值时，就会锁死在
    // 右手 —— 用户报的"AI判定一直是右手，之前不会"正是这么来的。
    //
    // 实测 hm20_v7（带时序上下文，prev 用上一帧模型输出）：
    //     |hand_logit| 中位 0.220     conf = tanh|logit| 中位 0.217
    //     纯随机点云    0.196         conf 0.194
    //     |logit| > 0.55 的帧只占 33%
    // 也就是说典型工作点跟噪声本底几乎重合。地板必须【明显高于】它：
    // 0.50 对应 |logit| >= 0.549，约是噪声本底的 2.8 倍。
    //
    // 代价是大多数时候报"未定"。这是【对的】—— 我们没有任何证据表明
    // 这个量级上的符号有意义，报一个没依据的答案会误导用户去改面板设定。
    double handConfFloor = 0.50;
    // ---- 遮挡点连续性的两个参数。在真实录制上按【两个判据】扫出来的 ----
    //
    // 素材：695 帧、8 个遮挡点、45 段遮挡（1787718792784_pcrec_刚体运动）。
    //
    // 【判据一：跳变有多大】（正常帧位移 0.80mm 作参照）
    //   alpha  decay | 转遮挡  恢复  最大跳变  实测帧偏差
    //   原始（不处理）|  4.61   7.14    39.3      0.00
    //    0.40  0.88  |  1.77   0.72    26.0      0.89
    //    0.80  0.88  |  3.38   0.83    23.1      0.87
    //
    // 【判据二：离真值多远】遮挡段最后一帧的预测 vs 恢复后的实测真值
    //   alpha=1.00（关）| 中位 6.74  p75  9.22  p90 15.34
    //   alpha=0.80      | 中位 6.41  p75  9.51  p90 15.26   <- 取这个
    //   alpha=0.60      | 中位 5.75  p75 10.17  p90 16.81
    //   alpha=0.40      | 中位 5.48  p75 11.42  p90 18.59   <- 尾部差 21%
    //   alpha=0.25      | 中位 7.23  p75 11.37  p90 21.11
    //
    // 【两个判据打架，必须都看】只看判据一会选 0.4（跳变最小）；
    // 但那是拿"离真值更远"换来的 —— 平滑有滞后，遮挡期手动得快时预测跟不上，
    // p90 从 15.34 涨到 18.59。输出更平但更不准，那不是改进。
    //
    // 0.80 是拐点：中位比不平滑还好一点（6.41 vs 6.74），p90 持平（15.26 vs 15.34），
    // 同时把恢复跳变从 7.14 压到 0.83。三项都不亏。

    // 预测点的时域平滑系数。只作用于【补出来的】点，实测点不碰。1.0 = 关闭。
    double predictSmoothAlpha = 0.8;
    // 恢复补偿的逐帧衰减率。0.88 在 24fps 下约 0.3 秒还清。0.0 = 关闭（旧行为）。
    // 【它不影响判据二】补偿只作用于恢复【之后】的实测帧，不改变遮挡期的预测。
    double recoverBlendDecay = 0.88;
    // 恢复补偿的偏移【钳幅】上限（不是丢弃门限）。超过的部分不补 ——
    // 位置误差不会被拖太远，但跳变仍然被摊开。
    // 真机恢复误差分布 p50 7.1 / p90 17.2 / p95 36.9 mm，有清晰断层；
    // 取 20 覆盖 p90 那一档，更大的 9% 靠钳幅后的多帧衰减消化。
    double recoverMaxOffsetMm = 20.0;
    // ---- 手背 tracklet（专业动捕的通用做法：先跟踪、再给整条轨迹定标签）----
    // 单帧 argmax 的逐点错误率约 8%，而手背只有 5 个标签且彼此最近，错了必然
    // 错到另一个手背点上（实测"手背<->手背"占全部错误 20%），一旦错就是整只手
    // 转 72°。轨迹级投票把几十帧的证据合起来，能把这类错压到接近 0。
    // 手背标签由【几何】重定，而不是信模型的分类。实测噪声 0.9mm 下 100%
    // 正确，正解与次解残差差 7.63mm。详见 relabelDorsumByGeometry()。
    bool   dorsumGeoRelabel      = true;
    double dorsumGeoMinMarginMm  = 2.0;   // 次解-正解 的残差差下限，不够就不动
    // 【改默认为 false】trackDorsum() 会把几十帧前的旧位置写回 markers 并
    // 置 observed=true —— 凭空创造观测。贪心最近邻门限 25mm/帧，手腕平移超过
    // 它就全部匹配失败、轨迹再也更新不了，5 个点被永久钉死在旧位置且回不来。
    // 由 dorsumRigidSolve 完整取代。
    bool   dorsumTracklet    = false;
    // 手背改成刚体求解：在【全部候选点】里搜，"谁是手背点"和"是哪一个"一起解，
    // 不依赖模型的手背分类。取代 dorsumGeoRelabel + dorsumTracklet 两条路。
    bool   dorsumRigidSolve  = true;
    // 拇指段在【几何/IK 都不可信】的帧改用网络 seg_rot6d + 标定常数。
    // 【默认关】它依赖两个还没在真数据上验过的假设：
    //   ① C(模板系->网络规范系)在整个会话里是常数
    //   ② K_s(拇指段的常数修正)也是常数
    // 两个都由 Hm20SegRot.hpp 的 SegCanonAligner 标定并自带 spreadDeg 自检。
    // 先看 spread <= 15° 且 offsetDeg(K) 落在 70~90°(= rig 缺掉的那个常数旋前)
    // 再打开 —— 对不上说明前面某个假设错了，硬接只会把一个错误换成另一个。
    bool   thumbSegFromNet   = false;
    // 是否每帧累积网络段常数的标定样本。开着只是往环形缓冲里塞数据，
    // 不影响出帧；标定完可以关掉。
    bool   netSegCalib       = false;
    // 录制离线复算所需的原始流。默认关，录制开始时打开。见
    // SkeletonFrameResult::hasDebugStreams 上的说明。
    // ---- 遮挡时的"网络方向 + 平面约束 + 时序平滑"融合 ----
    // 【为什么要有这个开关】它改变了遮挡点方向的来源（从耦合经验值改为
    // 网络当帧预测），是个行为性改动，出问题时要能一键退回旧行为对比。
    bool   occUseNetDir = true;
    // 平面内相位角的低通系数。实测（食指/中指/无名，caseB 帧）：
    //     alpha   跳变P95(mm)              交接误差(mm)
    //     0.5     6.53 / 7.80 / 8.54       4.37 / 7.31 / 3.52
    //     0.3     4.62 / 7.24 / 8.56       4.36 / 6.65 / 4.87
    //     0.2     3.67 / 7.09 / 8.73       4.60 / 6.43 / 5.31
    // 0.3 是拐点：再小跳变收益递减，而交接误差开始上升（滞后变大）。
    double occNetDirAlpha = 0.3;

    bool   captureDebugStreams = false;
    // 全量 log_assign。跟上面分开，因为它是唯一一个体积显著（约 200KB/s）的
    // 调试流 —— 常态录制不该付这个代价，但查指派问题时它是唯一的依据。
    bool   captureFullAssign = false;
    double dorsumTrackGateMm = 25.0;   // 帧间匹配门限。手背点间距 20mm+，
                                       // 帧间位移亚毫米，25mm 足够宽又不会配错
    int    dorsumTrackWarm   = 20;     // 攒够多少帧才敢用投票覆盖单帧结果
    double ikMaxRmseMm     = 12.0;

    // 中节骨的方向源。0=dp-pp（原实现）1=mp-pp 2=两端都观测时用 mp-pp 否则退回 dp-pp
    // 【为什么会有这个开关】球贴在每节指骨的远端附近，所以 pp球≈PIP处、
    // mp球≈DIP处、dp球≈指尖。按这个前提 mp-pp 正好张成中节骨，而 dp-pp
    // 多带了一节远节骨、跨过 DIP。近节和远节的选择在原注释里都有实测依据，
    // 唯独中节没有说明——所以要实测决定。
    int    midDirMode      = 0;    // 超过它认为 IK 没拟合上，本指退回几何法

    // 按标签取阈值。标签 <5 是手背，其余是手指。
    double minAssignProbFor(int label) const {
        return (label < 5) ? minAssignProbDorsum : minAssignProbFinger;
    }
};


// ---------------------------------------------------------------------------
// 拇指 IP 的遮挡期【连续】预测：靠可见近节的摆动。
//
// 【要解决的现象】拇指指尖球每次都是在 IP 弯过 ~45° 之后才消失的（一往掌心扣
// 就背对相机）。所以遮挡期间它还在动，而"冻住最后值"会把它停在入口角度。
// 真机实测：IP 真的在动的窗口里，保持的中位误差 12~24°，正好等于那段时间的
// 真实变化量 —— 一点都没跟上。
//
// 【为什么不是外推角速度】开环积分陈旧速度会漂，代码里另一处已经记过教训。
// 而且实测四段真实遮挡，入口速度 +13.6/+2.4/+4.4/+3.8 °/帧（在弯）、
// 出口速度 -2.2/-3.6/-2.7/-7.7 °/帧（在伸）—— 4/4 全都在遮挡期【掉头】了。
// 纯"持续增大"是所有模型里最差的：中位 57.3°，比保持还差一倍多。
//
// 【真正管用的信号：可见近节的摆动】m5/m6 在指尖遮挡时仍然可见，所以近节骨轴
// 在腕部系里的角速度 ω 每帧都测得到。拇长屈肌跨多个关节，整根拇指被驱动时
// 各节一起动 —— 于是 ω 里带着"现在在弯还是在伸"的信息，包括【掉头的时刻】。
//     dIP/dt ≈ c · ω        （c 是 3 维系数，在线学）
//     IP(t) = IP_last + Σ c·ω(τ)
// 这跟外推有本质区别：积分的是【当帧实测】的 ω，不是陈旧速度，所以不会漂，
// 而且拇指一掉头 ω 就变号，预测跟着掉头 —— 不需要预先知道遮挡多久。
//
// 【为什么是速度对速度、不是角度对角度】早先试过 IP ≈ a·MCP 的标量角度耦合，
// 把录制切成 6 段各自拟合，系数是 +0.05 / -0.08 / +0.33 / +0.73 / -0.70 / +1.37
// —— 会变号，拿去预测有时会把拇指往反方向弯。而这里的 c 在 5 折里是
// [2.84,2.10,-0.71] [2.80,2.05,-0.78] [3.75,2.52,-0.78] [4.16,3.32,-0.62]
// [2.69,2.03,-0.62]，同号同量级。静态角度可以是任意组合，被驱动时的运动不是。
//
// 【实测收益】两段独立录制，按可见段做 5 折（训练/测试完全不重叠），
// 只看 IP 真的在动的窗口：
//     旧录制 N=10  保持 12.0° -> 5.9°   (-51%)
//     旧录制 N=20  保持 14.8° -> 6.3°   (-57%)
//     新录制 N=5   保持 17.7° -> 3.0°   (-83%)
//     新录制 N=10  保持 23.8° -> 2.5°   (-89%)
//     10 折里赢 8 折
// ---------------------------------------------------------------------------
struct ThumbIpSwing {
    // 遗忘因子。0.995 在 24fps 下半衰期约 6 秒。
    double decay = 0.995;
    // 收缩增益。实测 1.0 最好（8/10 折），0.85/0.7 略差但更保守。
    // 只有一两段录制时想更稳可以调到 0.85。
    double gain = 1.0;
    // 岭正则。ω 三个分量在只做单平面运动时会共线，不加会解出巨大的系数。
    double ridge = 1e-4;
    // 单帧 IP 增量上限（rad）。再快的拇指也不会一帧转 12°，超了必是坏帧。
    double maxStepRad = 0.21;
    // 一次遮挡内累计偏离 IP_last 的上限（rad）。防止长遮挡里误差累积跑飞。
    double maxDriftRad = 1.2;

    // ω = 近节骨轴在【腕部系】里的角速度（因果差分：cross(u1[t-1], u1[t])）
    // dIp = 同两帧之间实测 IP 的变化（rad）。只在三球全可见时喂。
    void observe(const std::array<double, 3>& om, double dIp) {
        if (std::fabs(dIp) > maxStepRad) return;            // 坏帧
        double s = 0;
        for (int i = 0; i < 3; ++i) s += om[size_t(i)] * om[size_t(i)];
        if (!(s > 1e-12)) return;                           // 近节没动，这一对没信息
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j)
                A_[size_t(i * 3 + j)] = A_[size_t(i * 3 + j)] * decay
                                      + om[size_t(i)] * om[size_t(j)];
            b_[size_t(i)] = b_[size_t(i)] * decay + om[size_t(i)] * dIp;
        }
        n_ = n_ * decay + 1.0;
        dirty_ = true;
    }

    bool ready() const { return n_ >= 60.0; }

    // 本帧的 IP 增量。遮挡期每帧调一次，把返回值累加到保持住的角度上。
    double step(const std::array<double, 3>& om) const {
        if (!ready()) return 0.0;
        solve();
        double d = 0;
        for (int i = 0; i < 3; ++i) d += c_[size_t(i)] * om[size_t(i)];
        return std::clamp(d * gain, -maxStepRad, maxStepRad);
    }

    void reset() { A_.fill(0); b_.fill(0); c_.fill(0); n_ = 0; dirty_ = true; }
    double samples() const { return n_; }
    std::array<double, 3> coeff() const { solve(); return c_; }

private:
    // 3x3 岭回归，Cramer 法则。矩阵这么小不值得引入分解。
    void solve() const {
        if (!dirty_) return;
        dirty_ = false;
        std::array<double, 9> M = A_;
        const double lam = ridge * (A_[0] + A_[4] + A_[8] + 1e-12);
        M[0] += lam; M[4] += lam; M[8] += lam;
        const double det =
            M[0] * (M[4] * M[8] - M[5] * M[7])
          - M[1] * (M[3] * M[8] - M[5] * M[6])
          + M[2] * (M[3] * M[7] - M[4] * M[6]);
        if (!(std::fabs(det) > 1e-18)) { c_.fill(0); return; }
        const std::array<double, 9> inv{
            (M[4]*M[8]-M[5]*M[7])/det, (M[2]*M[7]-M[1]*M[8])/det, (M[1]*M[5]-M[2]*M[4])/det,
            (M[5]*M[6]-M[3]*M[8])/det, (M[0]*M[8]-M[2]*M[6])/det, (M[2]*M[3]-M[0]*M[5])/det,
            (M[3]*M[7]-M[4]*M[6])/det, (M[1]*M[6]-M[0]*M[7])/det, (M[0]*M[4]-M[1]*M[3])/det};
        for (int i = 0; i < 3; ++i) {
            double v = 0;
            for (int j = 0; j < 3; ++j) v += inv[size_t(i * 3 + j)] * b_[size_t(j)];
            c_[size_t(i)] = v;
        }
    }

    std::array<double, 9> A_{};
    std::array<double, 3> b_{};
    mutable std::array<double, 3> c_{};
    mutable bool dirty_ = true;
    double n_ = 0;
};

// ---------------------------------------------------------------------------
// 拇指 IP 的遮挡期跟随器 —— 用【仍然可见的掌骨/近节】的摆动方向驱动指尖。
//
// 【为什么"保持最后值"不够】真机实测四段遮挡的进出口速度：
//     入口 +13.6, +2.4, +4.4, +3.8 °/帧   (在弯)
//     出口  -2.2, -3.6, -2.7, -7.7 °/帧   (在伸)
// 4/4 全都是"进去时在弯、出来时在伸" —— 拇指在遮挡期间【掉头了】。
// 物理上很自然：扣进去 -> 到极限 -> 再伸出来，而球恰好在扣进去时消失、
// 伸出来时重现。保持会把它冻在入口的 ~60°，四段的出口误差中位 24.9°。
//
// 【为什么不能纯外推】入口速度 13.6°/帧 x 22 帧 = 300°。实测线性外推中位误差
// 57.3°，是所有模型里最差的，比保持还差一倍多。会掉头的弹道模型需要预先知道
// 遮挡总时长，运行时拿不到；换成固定 T 之后四段里三段几乎完美、一段炸到 50.8°。
//
// 【真正可用的信号：MCP 的摆动【方向】】
// m5/m6 在指尖遮挡时仍然可见，MCP 角每帧都测得到。关键的实测发现是：
//     ΔIP 与 ΔMCP 的【比值】不稳定（跨录制段 0.05~1.37，还会变号）
//     ΔIP 与 ΔMCP 的【符号】是稳的（|dMCP|>0.8°/帧 时一致率 ~70~100%）
// 之前用比值做耦合失败，正是因为拿不稳的量当了增益。而掉头判断只需要符号。
//
// 所以模型是：IP(t) = IP(t-1) + k * ΔMCP(t)，k 取正的固定值。
// 这跟角速度外推有本质区别 —— 驱动量 ΔMCP 是【当帧实测】的，不是积分出来的，
// 所以拇指掉头时它自己就跟着掉头，不会一路冲过去。
//
// 【k 和门限怎么定】都是量出来的，不是调出来的：
//   k = 2.35   两段真实录制里 |dIP|/|dMCP| 的中位数（n=186）
//   门限 0.8°/帧  低于这个 MCP 的变化基本是噪声，符号不可信，此时不动
//
// 【实测收益】四段真实遮挡对出口值的误差：
//     保持       中位 24.9°  最大 34.4°
//     本模型     中位  4.4°  最大  9.7°     <- 4/4 段全部改善
//   代价：拇指静止而手在动的时段会多出 ~2.7° 噪声（绝对值仍在 5° 内）。
// ---------------------------------------------------------------------------
struct ThumbIpTracker {
    // 见上：两者都是实测值。想更保守就调小 k（k=1.5 时真遮挡中位 11.5°、
    // 静止段代价只有 0.9°），想更激进调大（k=3.0 时最大误差最小）。
    double k        = 2.35;
    double gateRad  = 0.0140;   // 0.8°/帧
    double loRad    = -0.26;    // 拇指 IP 解剖范围
    double hiRad    = 1.48;

    // 指尖可见时调用：把当帧的实测值存下来，作为遮挡起点。
    void sync(double ipMeasured, double mcpMeasured) {
        ip_ = std::clamp(ipMeasured, loRad, hiRad);
        entry_ = ip_;                 // 指尖可见时不断刷新，遮挡开始那一帧就是起点
        mcp_ = mcpMeasured;
        has_ = true;
    }

    // 指尖被遮时每帧调用一次，喂当帧实测的 MCP 角。返回跟随后的 IP 角。
    // 【必须每帧都调】它是增量式的：跳帧会漏掉那一帧的 MCP 变化。
    double step(double mcpNow) {
        if (!has_) return ip_;
        const double d = mcpNow - mcp_;
        mcp_ = mcpNow;
        if (std::fabs(d) < gateRad) return ip_;   // MCP 没动，符号不可信，不动
        ip_ = std::clamp(ip_ + k * d, loRad, hiRad);
        return ip_;
    }

    bool   ready() const { return has_; }
    double angle() const { return ip_; }
    // 本次遮挡的起点值（最后一次指尖可见时量到的）。用于报"跟随了多少"。
    double entryAngle() const { return entry_; }
    void   reset() { has_ = false; ip_ = 0.0; mcp_ = 0.0; entry_ = 0.0; }

private:
    double ip_ = 0.0, mcp_ = 0.0, entry_ = 0.0;
    bool   has_ = false;
};

} // namespace hm20
} // namespace mocap
