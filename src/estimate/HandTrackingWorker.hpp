#pragma once
// ---------------------------------------------------------------------------
// 手部动捕 Qt 汇聚层：订阅各相机 ICamera::blobDetailsReady(带轮廓点的详细
// 检测结果)，按时间戳分桶，喂进 detect/DetectionOutput.hpp 的 3a~3d 遮挡
// 感知流水线得到 (mu,Sigma,confidence)，再驱动 HandTrackingPipeline(关联+
// IEKF)。跟 Triangulator.cpp 是同一个批处理模式（去抖计时器、CamBuf），
// 职责不同：Triangulator 做"2台起步、多台任意、不管身份"的通用三角化；
// 这个类做"身份已知(marker级)、手腕刚体+16关节角一体估计"的手部专用管线。
//
// 【这一版修的真实bug】之前 HandColdStart/HandModelAdapter 都写死用
// HandModel.hpp 的占位值，就算做了 HandCalibration.hpp/
// HandSelfCalibration.hpp 标定，结果也没地方存、没地方用——现在从
// HandTemplateStore 读取实际标定的模板，冷启动匹配、实时FK都用这份真实
// 数据，标定结果才真正生效。
//
// 【这一版加的诊断】冷启动每次尝试(不管成功失败)都会 emit
// coldStartDiagnosticsReady，带着"候选点够不够、距离匹配了几组、Kabsch
// 残差多少"这些具体原因，供 HandPoseDebugDialog 直接显示，而不是只有一个
// "未找到"让人瞎猜。
//
// 【这一版修的另一个真实bug】原来"已知像素半径"只有一个全局固定假设
// (物理半径 / 标称工作距离常量)，工作距离跨度大(比如15~50cm)时两头都
// 系统性偏差很大，3a~3d 约束圆拟合把圆心拟合偏，两视图对极几何验证失败，
// 表现成"预览能看到点，检测本身没问题，但手部追踪/标定拿不到候选点"——
// 见 estimate/RadiusRefine.hpp 顶部注释的完整分析。现在改成两遍流程：
//   第一遍(coarse)：不需要知道半径，直接用原始质心(Blob::cx,cy)做粗略
//   两视图/多视图匹配+三角化，只是为了拿到每个候选点的粗略深度。
//   第二遍(refined)：每个blob拿"离它最近的粗略候选点的深度"反推自己
//   该用的真实像素半径(estimate/RadiusRefine.hpp::refineExpectedRadius)，
//   不再是全局一个常量，而是每个点各自按自己的实际深度算——远近跨度再
//   大也不怕。拿这个精修半径重新喂3a~3d精修拟合，再做最终匹配。
//   旧的固定标称距离假设(nominalWorkingDistanceMm_)现在只是"这一帧粗算
//   都没匹配上任何候选点"时的兜底，不再是常态路径。
//
// 【无法在当前环境编译验证】这个文件依赖 Qt6，构建这份回答的沙箱没有装
// Qt6。数学部分(estimate/RadiusRefine.hpp)已经用合成数据独立编译验证过；
// 这个文件本身的 Qt 布线请在本地实际编译一遍。
// ---------------------------------------------------------------------------
#include "core/Types.hpp"
#include "calib/Calibration.hpp"
#include "calib/CalibrationStore.hpp"
#include "hand/HandTemplateStore.hpp"
#include "detect/CentroidDetector.hpp"         // Blob
#include "detect/BlobObservationAdapter.hpp"   // blobToObservations（真正的3a~3d接入点）+ BlobLocalizationMode
#include "reconstruct/Triangulation.hpp"      // undistortNormalize（冷启动候选点/粗算质心都要用）
#include "reconstruct/Epipolar.hpp"           // EpiMat3/EpiVec3
#include "reconstruct/MultiViewCluster.hpp"   // clusterMultiView（粗算阶段+冷启动都用它拿无标签候选3D点）
#include "reconstruct/TemporalTracker.hpp"    // 候选点广播给UI用——跨帧维持稳定ID，UI标定向导选点靠这个身份不能乱
#include "estimate/HandStateIEKF.hpp"
#include "estimate/MarkerAssociator.hpp"
#include "estimate/HandModelAdapter.hpp"
#include "estimate/HandColdStart.hpp"
#include "estimate/RadiusRefine.hpp"          // 深度反推半径的数学核心，见该文件顶部完整分析
#include "estimate/GateRadiusEstimator.hpp"   // 从已标定手部模板几何自动推算门控半径
#include "estimate/SmootherWorker.hpp"        // 延迟精修流(FixedLagSmoother)的独立线程worker，完全独立于实时IEKF
#include "reconstruct/OneEuroFilter.hpp"      // 输出端平滑——压制静止时的高频抖动

#include <QObject>
#include <QVector>
#include <QPointF>
#include <QVector3D>
#include <QTimer>
#include <QThread>
#include <QHash>
#include <QString>

#include <vector>
#include <array>
#include <optional>

namespace mocap {

class ICamera;

enum class TrackingMode {
    FullHand,       // 完整16维关节角IEKF，手背+手指一起估计(默认，原有行为)
    BackRigidOnly,  // 只测手背刚体，完全不碰手指——见 setTrackingMode 注释
};

class HandTrackingWorker : public QObject {
    Q_OBJECT
public:
    // templateStore 的所有权归调用方(通常是 MainWindow 长期持有一个实例，
    // 标定向导跟这个 worker 共用同一个 store——标定向导写入、这里读取)。
    explicit HandTrackingWorker(const QVector<ICamera*>& cams, CalibrationStore* calibStore,
                               HandTemplateStore* templateStore, QObject* parent = nullptr);
    ~HandTrackingWorker() override;   // 需要显式停止延迟精修流的线程(见.cpp)，不能只靠Qt父子对象自动清理——那套机制不管moveToThread过的线程

    int calibratedCount() const { return int(camPoses_.size()); }

    // 主动丢弃当前跟踪状态，下一帧重新走冷启动(比如 UI 上"重新捕获"按钮)。
    void resetTracking();

    // 标定向导用：把这一帧无标签三角化出的候选3D点(世界系)广播出来，供
    // 向导UI列出来让用户点选"这几个是手背5点""这个是手指方向提示点"。
    // 默认关闭——候选点广播是标定向导专用的调试通道，正常追踪时不需要
    // 额外算这个、也不需要占带宽发给UI，向导打开时才开，关闭向导就关掉。
    void setCandidateBroadcastEnabled(bool on) { candidateBroadcastOn_ = on; }

    // 手部模板(标定向导写完之后)在运行时更新了——重建 FK(冷启动用的模板
    // 由 coldStartFromFrames 每次都实时从 templateStore_ 读，不需要单独
    // 刷新；但 fk_ 是构造时绑定好的闭包，模板变了必须重新生成，否则实时
    // 追踪还在用旧的手指结构参数)。标定向导保存新模板后应该调用这个。
    void reloadTemplate();

    // 【调参用·抖动/漂移的主要旋钮】IEKF每帧predict()用的过程噪声——
    // 数值越大，滤波器越"相信新观测"、跟得越快但越容易被检测噪声带着
    // 抖；数值越小，越平滑但跟手的实际动作会有滞后感。手静止时抖得厉害，
    // 先把这三个往下调（比如各自减半）试试，这是最直接影响"静止时读数
    // 稳不稳"的参数，跟专业动捕系统调滤波器强弱是同一件事。
    void setProcessNoise(double posVar, double rotVar, double jointVar) {
        posProcessVar_ = posVar; rotProcessVar_ = rotVar; jointProcessVar_ = jointVar;
    }
    double posProcessVar() const { return posProcessVar_; }
    double rotProcessVar() const { return rotProcessVar_; }
    double jointProcessVar() const { return jointProcessVar_; }

    // 【调参用·真正的输出端平滑】One Euro Filter——压制静止时的高频抖动，
    // 跟上面的IEKF过程噪声是两道独立的关卡：过程噪声调的是滤波器"信不信
    // 新观测"，这里调的是"已经出来的读数还要不要再抹一层"。minCutoff越小
    // 越平滑(静止时更稳，但慢动作会有一点"黏滞感")；beta越大，动得快的
    // 时候截止频率抬得越高、越跟手、抖动压制越弱——抖动主要在静止时出现
    // 就调小minCutoff，跟手迟钝感明显就调大beta。位置/旋转/关节角三组
    // 独立设置，默认给的是比较保守(偏平滑)的起点。
    void setSmoothingParams(double posMinCutoff, double posBeta,
                            double rotMinCutoff, double rotBeta,
                            double jointMinCutoff, double jointBeta);
    std::array<double,2> posSmoothingParams() const { return {posMinCutoff_, posBeta_}; }
    std::array<double,2> rotSmoothingParams() const { return {rotMinCutoff_, rotBeta_}; }
    std::array<double,2> jointSmoothingParams() const { return {jointMinCutoff_, jointBeta_}; }

    // 【调参用】MarkerAssociator 的关联门控半径(归一化坐标)——手指并拢/
    // marker靠得近时，这个半径设太大容易把观测错配给邻居marker(表现成
    // "关节角乱跳")，设太小又容易在检测噪声稍大时直接关联不上(表现成
    // 数据丢失/更依赖兜底)。
    //
    // 【实现提醒】gateRadiusNorm 是 HandTrackingPipeline/MarkerAssociator
    // 构造时定死的，两者都没有暴露运行时可改的setter——所以这个setter
    // 内部会整个重建 pipeline_，代价是**当前的追踪状态会丢失，下一帧要
    // 重新走一次冷启动**，调参数时预期之内，不是bug。频繁调用(比如拖
    // 滑块每一帧都触发)会一直反复冷启动，建议UI侧做一下节流(比如拖动
    // 松手才真正应用，不要valueChanged就立刻调)。
    void setGateRadius(double gateRadiusNorm);
    double gateRadius() const { return gateRadiusNorm_; }

    // 【调参用】马氏距离门控的卡方阈值——见 MarkerAssociator.hpp 里
    // MarkerAssociator 构造函数的注释，默认9.21对应二维卡方分布99%置信
    // 区间。数值越大越宽松(更容易关联上，但也更容易错配)，越小越严格。
    // 跟gateRadius一样是构造时定死在pipeline_里的，改了要重建pipeline_。
    void setChiSquareGate(double chiSquareGate) { chiSquareGate_ = chiSquareGate; rebuildPipeline(); }
    double chiSquareGate() const { return chiSquareGate_; }

    // 【延迟精修流，完全独立于实时IEKF，见对话记录的架构设计】跑在独立
    // 线程上的 FixedLagSmoother——攒一个小窗口(默认7帧)联合优化，延迟约
    // 等于窗口帧数(默认帧率下大概几十毫秒)换更高精度，尤其对手指这种
    // 单帧可观测性弱的自由度收益明显。默认关闭，不占任何资源；打开才
    // 会真正建线程、跑计算。跟实时流通过独立的 handPoseSmoothedReady
    // 信号输出，不影响/不替代 handPoseReady 那条低延迟流——两条流并存，
    // 各自服务不同的消费者(前者给需要低延迟反馈的地方，后者给能接受
    // 延迟换精度的地方，比如最终数据输出/录制存档)。
    //
    // 【线程模型】照 detect/DetectWorker.hpp 的"独立线程+忙时丢帧不排队"
    // 抄的，见 SmootherWorker.hpp 顶部完整分析——不会因为这条流处理慢
    // 而拖慢实时追踪，最坏情况只是这条延迟流自己偶尔缺一帧。
    void setSmootherEnabled(bool on);
    bool smootherEnabled() const { return smootherEnabled_; }
    // 窗口大小/平滑先验强度——改了会重建这条延迟流(丢弃当前窗口内容，
    // 不影响实时IEKF)，不需要跟gateRadius/chiSquareGate那样重建整个
    // pipeline_，代价小得多。
    void setSmootherConfig(const SmootherConfig& cfg);
    SmootherConfig smootherConfig() const { return smootherCfg_; }
    // 估算当前配置下这条延迟流大概带来多少毫秒延迟——windowSize帧 ×
    // 典型帧间隔(按调用方给的fps估计，不知道就传个默认125fps的粗略值)，
    // 供UI直接显示给用户看"现在这个配置延迟大概多少"，不用自己心算。
    double estimatedSmootherLatencyMs(double assumedFps = 125.0) const {
        return assumedFps > 0.0 ? (double(smootherCfg_.windowSize) * 1000.0 / assumedFps) : -1.0;
    }

    // 【自动推算，第一块能立刻做的】从已标定的手部模板几何(不需要额外
    // 采集数据)算出一个跟这只手、这套贴球方案、这个工作距离真正匹配的
    // 门控半径建议——见 estimate/GateRadiusEstimator.hpp 顶部完整分析。
    // 只计算、不应用，方便UI先把推算依据(哪两颗marker最容易混淆、算出
    // 来的值)展示给用户看，用户确认后再点"应用"才真正调用setGateRadius
    // (那个会重建pipeline、丢失当前追踪状态)。
    GateRadiusEstimate estimateGateRadiusFromTemplate(double workingDistanceMm, double marginFraction = 0.4) const {
        return mocap::estimateGateRadius(fk_, workingDistanceMm, marginFraction);
    }

    // 【调参用】球的物理半径(mm)——直接决定"深度反推半径"这条链路第二遍
    // 精修用的绝对尺度。跟距离无关，量准了基本不用再改；默认4.5mm(对应
    // 9mm直径球)。
    void setMarkerGeometry(double physicalRadiusMm, double nominalWorkingDistanceMm) {
        physicalMarkerRadiusMm_ = physicalRadiusMm;
        nominalWorkingDistanceMm_ = nominalWorkingDistanceMm;
    }
    // 【新增】用户直接量出来的、这套贴球方案里最近两颗marker的真实物理
    // 间距(比如卡尺量出来15mm)——比GateRadiusEstimator从FK采样几个代表
    // 姿态反推出来的间距更可信(FK那套是基于占位/标定表几何算的估计值，
    // 实测是直接量出来的事实)。设了这个之后，门控半径的安全地板会用这个
    // 真实数字重新算，不再用某个拍脑袋/占位表算出来的常量兜底。
    void setMeasuredMinMarkerSpacingMm(double mm) {
        measuredMinMarkerSpacingMm_ = mm;
    }
    double measuredMinMarkerSpacingMm() const { return measuredMinMarkerSpacingMm_; }
    double physicalMarkerRadiusMm() const { return physicalMarkerRadiusMm_; }
    // nominalWorkingDistanceMm 现在只是"深度反推失败时"的兜底假设(比如
    // 冷启动第一帧、第一遍粗匹配还没凑出任何候选点时)，不再是常态路径——
    // 常态下每个候选点的半径由它自己反推出的真实深度决定，不再依赖这个
    // 单一常量对所有距离都适用。
    double nominalWorkingDistanceMm() const { return nominalWorkingDistanceMm_; }

    // 【调参用】两遍流程各自的两视图匹配阈值(归一化坐标)：
    //   coarse：第一遍，用原始质心(不知道半径)做粗略匹配定深度，质心噪声
    //   通常比精修后的圆心噪声大，阈值要放宽，否则粗算这一步本身就先失败了。
    //   refined：第二遍，用深度反推出的真实半径重新做3a~3d精修后的最终
    //   匹配，这时候观测应该已经准了，阈值可以收紧，避免真正的噪声/幽灵
    //   点被放进最终结果。
    void setCoarseClusterThresholds(double maxSampson, double maxReprojNorm) {
        coarseMaxSampson_ = maxSampson;
        coarseMaxReprojNorm_ = maxReprojNorm;
    }
    void setRefinedClusterThresholds(double maxSampson, double maxReprojNorm) {
        clusterMaxSampson_ = maxSampson;
        clusterMaxReprojNorm_ = maxReprojNorm;
    }
    double coarseMaxSampson() const { return coarseMaxSampson_; }
    double coarseMaxReprojNorm() const { return coarseMaxReprojNorm_; }
    double refinedMaxSampson() const { return clusterMaxSampson_; }
    double refinedMaxReprojNorm() const { return clusterMaxReprojNorm_; }

    // 【调参用】深度反推半径时的匹配门控(归一化坐标)——候选点投影到某台
    // 相机后，离这台相机的原始质心观测超过这个距离就不采信那个候选点的
    // 深度(防止把明显是别的marker的候选点误当成这个blob的深度来源)。
    void setRadiusRefineMatchGate(double maxNorm) { radiusRefineMatchGateNorm_ = maxNorm; }
    double radiusRefineMatchGate() const { return radiusRefineMatchGateNorm_; }

    // 【已废弃旧接口，保留兼容】原来的固定阈值调用，现在等价于同时设置
    // refined 那一组(常态使用的最终匹配阈值)；粗算阈值请用
    // setCoarseClusterThresholds 单独调，默认已经比这组宽松。
    void setClusterThresholds(double maxSampson, double maxReprojNorm) {
        clusterMaxSampson_ = maxSampson;
        clusterMaxReprojNorm_ = maxReprojNorm;
    }

    // 打开后每帧在控制台打印详细的中间结果(每台相机收到几个blob、算出的
    // 假设半径、blobToObservations产出几条观测、clusterMultiView凑出几个
    // 候选点)——排查"预览有点、追踪没候选点"这类问题的第一步就是打开这个，
    // 看数据到底在哪一步被吃掉了。默认开着，调试完记得关掉(正常运行不需要
    // 这些日志，且会明显拖慢控制台/影响帧率)。
    void setVerboseLogging(bool on) { verboseLogging_ = on; }

    // 【调参用·三选一】这一帧每个blob的2D定位用什么方法——见
    // detect/BlobObservationAdapter.hpp::BlobLocalizationMode 顶部注释。
    // 默认CircleFitOnly(原有行为不变)；实测发现有些场景质心比圆拟合更稳
    // (球清晰可见、信噪比好时质心本身已经接近最优，圆拟合的"已知半径"
    // 先验哪怕经过深度反推仍可能有残余误差)，圆拟合真正不可替代的场景
    // 是遮挡(质心会系统性偏向可见的那一侧)。三个都试一遍，按实际画面
    // 情况选，或者直接用Fused让它自动按弧长退化。
    void setLocalizationMode(BlobLocalizationMode mode) { localizationMode_ = mode; }
    BlobLocalizationMode localizationMode() const { return localizationMode_; }
    void setCentroidSigma(double sigmaPx) { centroidSigmaPx_ = sigmaPx; }
    double centroidSigma() const { return centroidSigmaPx_; }

    // 【调参用·测试手背时压制手指干扰】FullHand(默认)：完整16维关节角
    // IEKF，手背+手指一起估计。BackRigidOnly：只测手背刚体，完全不碰
    // 手指——每帧直接对候选点做距离匹配+Kabsch(复用冷启动那套数学，
    // 本来就验证过、够稳)，关节角固定给中性值(全0)，物理上就不存在
    // "手指乱飞"，因为手指压根没被估计。手指结构参数还是占位值/没标定
    // 完时，先用这个模式单独验证手背这条链路，不受手指干扰。
    void setTrackingMode(TrackingMode mode) { trackingMode_ = mode; }
    TrackingMode trackingMode() const { return trackingMode_; }

signals:
    void handPoseReady(QVector3D wristPos, QVector<double> wristRot9,
                      QVector<double> jointAngles16, qint64 ts_ns);
    void handNotFound(qint64 ts_ns);

    // 延迟精修流的输出——完全独立于上面的 handPoseReady，见
    // setSmootherEnabled 注释。只有开启这条流才会发；ts_ns 是这个结果
    // 对应的那一帧的原始时间戳(不是"现在"，因为它天然滞后窗口帧数)，
    // 消费者可以拿这个跟 handPoseReady 的 ts_ns 对上号，知道这是"回头
    // 修正过的第几帧"。
    void handPoseSmoothedReady(QVector3D wristPos, QVector<double> wristRot9,
                              QVector<double> jointAngles16, qint64 ts_ns);

    // 每次冷启动尝试(不管成不成功)都会发一次，供 UI 展示具体原因而不是
    // 干等"未找到"。成功时 diag.failReason==None。
    void coldStartDiagnosticsReady(int numCandidates3D, int numDistanceMatches,
                                   double bestRmsMm, int failReasonCode, QString summary,
                                   qint64 ts_ns);

    // 标定向导专用：这一帧的候选3D点(世界系)，仅在 setCandidateBroadcastEnabled(true)
    // 时才发。同时带上"当前是否有可信的手腕位姿、位姿本身"——手指自标定
    // 那一步需要把候选点转到手腕局部系，向导UI没有直接访问 pipeline_ 的
    // 权限，靠这个信号把需要的信息一起递出去。
    //
    // 【这一版修的真实bug】ids 跟 worldPoints 一一对应，是跨帧稳定的身份
    // (由 candidateTracker_ 一个 TemporalTracker 维护)，不是数组下标——
    // 原来直接广播 clusterMultiView 每帧算出来的裸数组，同一颗物理球在
    // 不同帧里的数组位置会跳来跳去，UI(HandCalibrationWizard里的
    // BackTemplateCalibPage)如果拿"这一帧的下标"当身份存起来，等下一帧
    // 数组顺序一变，之前存的下标就对应到别的物理点了——表现症状正是"点
    // 追踪抖、编号一直闪烁"，而且不只是视觉问题，是标定可能拿了错的点
    // 组合的正确性问题。现在 ids 是持久身份，UI 应该用 ids 而不是数组
    // 位置来记住"用户选的是哪个点"。
    // missedFrames 跟 worldPoints/ids 一一对应：0=这一帧真正看到的实测
    // 位置；>0=遮挡记忆(coasting)中，position是最后一次看到时的冻结旧值，
    // 数字是已连续丢失的帧数。UI应把>0的点画成暗色/空心，让用户一眼分清
    // "实测"和"记忆"。追加在信号末尾：Qt函数指针connect允许槽的参数比
    // 信号少，不关心这个字段的旧消费者(如AutoCalibPage)不需要任何改动。
    void candidatePointsReady(QVector<QVector3D> worldPoints, QVector<int> ids, bool hasWristPose,
                             QVector<double> wristRot9, QVector3D wristPos, qint64 ts_ns,
                             QVector<int> missedFrames);

    // 诊断用：这一帧两遍流程的中间结果——第一遍粗算出几个候选深度点、
    // 每台相机的blob里有几个成功反推出半径(matched)几个退回兜底
    // (fallback)。UI可以拿这个判断"是不是又卡在半径假设上了"，不用
    // 只盯着console日志看。
    void radiusRefineStatsReady(int coarsePointCount, QVector<int> matchedPerCam,
                               QVector<int> fallbackPerCam, qint64 ts_ns);

private slots:
    void onBlobDetails(quint32 camId, const QVector<Blob>& blobs, qint64 ts_ns);
    void onTimerFire();

private:
    struct CamBuf { QVector<Blob> blobs; qint64 ts = -1; };

    void processFrame(qint64 ts_ns);
    void processBackRigidOnlyFrame(const std::vector<CameraFrame>& frames, qint64 ts_ns);   // TrackingMode::BackRigidOnly专用分支，见setTrackingMode注释
    void rebuildPipeline();   // 构造函数和setGateRadius共用——重建pipeline_会丢失当前追踪状态，调用方需要清楚这一点
    std::optional<HandPoseState> coldStartFromFrames(const std::vector<CameraFrame>& frames);
    std::vector<Vec3> computeCandidatePoints(const std::vector<CameraFrame>& frames) const;

    // 第一遍：不需要知道半径，直接用各相机原始质心(Blob::cx,cy)做粗略
    // 两视图/多视图匹配+三角化，只为拿到每个候选点的粗略深度——供
    // estimateRefinedRadiusPx 反推每个blob各自该用的真实像素半径。
    std::vector<Vec3> computeCoarseDepthPoints(const QHash<quint32, CamBuf>& latestSnapshot) const;

    // 第二遍：给定某台相机某个blob的质心 + 第一遍算出的粗略候选点集，
    // 反推这个blob该用的期望像素半径；找不到匹配(见
    // RadiusRefine.hpp::refineExpectedRadius 的门控)时退回旧的兜底逻辑
    // (优先用当前状态估计的深度，其次用标称工作距离常量)。
    double estimateRefinedRadiusPx(size_t camIdx, const Blob& blob,
                                   const std::vector<Vec3>& coarseDepthPoints, bool* matchedOut = nullptr) const;
    double estimateFallbackRadiusPx(size_t camIdx) const;   // 旧逻辑，改名保留做兜底

    QVector<ICamera*> cams_;
    CalibrationStore* calibStore_;
    HandTemplateStore* templateStore_;

    std::vector<quint32> camIds_;
    std::vector<CamPose> camPoses_;
    std::vector<CameraCalibration> calibs_;

    // 【这是从单槽"最新帧覆盖"升级成"短历史+最近邻时间戳匹配"的地方】
    // 之前每台相机只保留"最新到达的那一帧"，配帧时不管这一帧到底是刚到
    // 还是已经等了好几毫秒——没有硬件同步码(见 SyncMonitor 相关讨论)时，
    // 各相机USB到达时刻天然有抖动，直接拿"当前各自最新"拼在一起，某台
    // 相机卡顿/掉帧那一下，就会用一帧明显更旧的数据去跟别的相机凑成
    // "同一帧"，三角化会悄悄用上时间不对齐的观测。现在每台相机保留最近
    // kHistoryDepth 帧的短历史，配帧时对每台相机搜索"离目标时间戳最近的
    // 那一帧"，且只有在容差内才采纳——采纳不到就让这台相机这一帧空缺
    // (下游 clusterMultiView/MarkerAssociator 本来就支持"某台相机这一帧
    // 没有观测"这个语义，不需要额外改动)，好过硬凑一帧对不上时间的数据
    // 进去。
    //
    // 容差不是写死的常量——写死一个"按120fps估的8ms"，相机换了帧率(或者
    // 干脆有几台相机帧率本来就不一样)这个数就跟着失配，得手改代码。改成
    // 每台相机自己实测到达间隔(EWMA)，容差从"当前实际跑得最慢那台相机的
    // 实测间隔"按比例算出来——换帧率/换相机，这个数自己跟着变，不用碰
    // 代码。见 currentSyncToleranceNs() 的完整推导。
    QHash<quint32, QVector<CamBuf>> history_;
    QHash<quint32, double> avgIntervalNs_;   // 每台相机的实测帧间隔EWMA
    static constexpr int kHistoryDepth = 6;              // 每台相机保留最近几帧供搜索，覆盖几个kBatchWindowMs的抖动余量足够
    static constexpr double kIntervalEwmaAlpha = 0.15;   // 平滑系数，不用哪一帧的瞬时抖动直接决定容差
    static constexpr qint64 kSyncToleranceFloorNs = 8'000'000;    // 地板：至少8ms，防止刚启动样本太少时容差被压得不合理地窄
    static constexpr qint64 kSyncToleranceCeilNs  = 40'000'000;   // 天花板：超过40ms说明哪台相机实测间隔本身就离谱(几乎要掉到25fps以下)，容差不能无限跟着放大掩盖这个问题
    qint64 currentSyncToleranceNs() const;
    QHash<quint32, CamBuf> matchSyncedSnapshot(qint64 targetTs) const;
    QTimer* pendingTimer_ = nullptr;
    static constexpr int kBatchWindowMs = 4;

    ForwardKinematicsFn fk_;
    std::optional<HandTrackingPipeline> pipeline_;

    double physicalMarkerRadiusMm_ = 4.5;
    double nominalWorkingDistanceMm_ = 250.0;   // 现在只是兜底，不是常态路径，见文件头注释
    // -1 = 用户没提供实测间距，门控地板退回旧的固定占位值(见reloadTemplate)；
    // >0 = 用这个真实测量值重新算地板，不再猜。
    double measuredMinMarkerSpacingMm_ = -1.0;

    double gateRadiusNorm_ = 0.05;
    double chiSquareGate_ = 9.21;

    // 延迟精修流——独立线程worker，默认不建(见setSmootherEnabled)。
    SmootherWorker* smootherWorker_ = nullptr;
    QThread* smootherThread_ = nullptr;
    bool smootherEnabled_ = false;
    SmootherConfig smootherCfg_;
    void startSmootherThreadIfNeeded();
    void stopSmootherThread();
    double posProcessVar_ = 0.5;
    double rotProcessVar_ = 1e-4;
    double jointProcessVar_ = 1e-4;

    ColdStartConfig coldStartCfg_;
    qint64 currentTs_ = 0;   // processFrame 当前正在处理的帧时间戳，供 coldStartFromFrames 的诊断信号使用
    bool candidateBroadcastOn_ = false;

    // 第一遍(粗算深度)用的阈值——质心噪声通常比精修后的圆心噪声大，比
    // refined 那组宽松，默认给了比 refined 默认值大一倍的起点。
    double coarseMaxSampson_ = 0.02;
    double coarseMaxReprojNorm_ = 0.02;
    // 第二遍(最终匹配)用的阈值——沿用原来的字段名保持向后兼容
    // (setClusterThresholds 这个旧接口还在，等价地设置这一组)。
    double clusterMaxSampson_ = 0.01;
    double clusterMaxReprojNorm_ = 0.01;
    // 深度反推半径这一步本身的匹配门控，见 setRadiusRefineMatchGate 注释。
    double radiusRefineMatchGateNorm_ = 0.03;

    BlobLocalizationMode localizationMode_ = BlobLocalizationMode::CircleFitOnly;
    double centroidSigmaPx_ = 0.3;
    TrackingMode trackingMode_ = TrackingMode::FullHand;

    // 候选点广播给标定向导UI用的跨帧稳定ID——见 candidatePointsReady 信号
    // 注释。参数按专业动捕的标准思路设：
    //   maxAssocDist=20mm：手静止时球只有检测噪声级别的抖动，20mm足够宽松。
    //   maxMissedFrames=36：≈300ms@120fps 的遮挡记忆。这个值是"帧数"不是
    //     "时长"，之前设5是按低帧率时代校的——升到120fps后5帧只有42ms，
    //     手指一次自遮挡轻松超过，ID被销毁、重现时领新编号，表现出来就是
    //     "编号疯狂增加、选中的球高亮丢失"。300ms是标定场景下"容忍一次
    //     自然遮挡"和"不把新点误认成旧点"之间的折中。
    //   minHitsToConfirm=6：新轨迹连续被看到6帧(50ms@120fps)才转正发布，
    //     之前是3帧——合成数据压力测试(手部移动+15%概率的偶发幽灵点)
    //     发现3帧这道门槛偶尔挡不住反复出现的幽灵点：同一个位置反复冒出
    //     的幽灵，只要偶然连续命中3帧就能混进确认轨迹，之后跟旁边的真点
    //     在同一个位置反复拉锯，表现出来就是编号来回横跳而不是稳定递增，
    //     比"编号疯涨"更隐蔽、更难查。3帧到6帧，偶然连续命中的概率不是
    //     线性减半，是指数下降(大致是p³到p⁶的量级差距，p是幽灵单帧命中
    //     概率)，同一场景下混入概率会降到原来的几十分之一。代价是真点也
    //     要多等3帧(120fps下25ms)才转正显示，肉眼基本无感。
    TemporalTracker candidateTracker_{20.0, 36, 6};

public:
    // 【这是"点云很稳、编号还是频繁增长"那个问题的同款修复】之前整个
    // 重新构造 candidateTracker_，代价是已确认的编号全部清空重来——调这
    // 几个参数本身在制造"编号在增长"的假象。改用运行时setter，只改参数、
    // 不清空已有轨迹和编号。
    void setCandidateTrackerParams(double maxAssocDistMm, int maxMissedFrames,
                                   int minHitsToConfirm = 3) {
        candidateTracker_.setMaxAssocDist(maxAssocDistMm);
        candidateTracker_.setMaxMissedFrames(maxMissedFrames);
        candidateTracker_.setMinHitsToConfirm(minHitsToConfirm);
    }

private:
    bool verboseLogging_ = true;   // 默认开着方便排查，调试完建议 setVerboseLogging(false)

    // 输出端平滑——见 setSmoothingParams 注释。旋转矩阵不能直接对9个数
    // 分别做低通(滤完不再是合法旋转)，转四元数滤波、renormalize、转回
    // 矩阵；四元数本身有双重覆盖(q和-q代表同一个旋转)，逐帧独立滤波前
    // 必须先跟上一帧的符号对齐，否则符号一翻转，看起来像是转了180度的
    // 巨大跳变——见 .cpp 里 applySmoothing() 的具体处理。
    OneEuroFilter3 posFilter_{1.0, 0.3};
    std::array<OneEuroFilter,4> quatFilters_{ OneEuroFilter(1.0,0.3), OneEuroFilter(1.0,0.3),
                                              OneEuroFilter(1.0,0.3), OneEuroFilter(1.0,0.3) };
    std::array<OneEuroFilter,16> jointFilters_{};   // 用默认参数构造，setSmoothingParams里统一设
    double posMinCutoff_ = 1.0, posBeta_ = 0.3;
    double rotMinCutoff_ = 1.0, rotBeta_ = 0.3;
    double jointMinCutoff_ = 1.0, jointBeta_ = 0.3;
    bool haveLastQuat_ = false;
    std::array<double,4> lastQuatRaw_{1,0,0,0};
    bool hadStateLastFrame_ = false;   // 追踪状态从"丢失"变成"刚找到"时，平滑滤波器要重置，不能拿丢失前的旧值去平滑一个全新的位姿

    void resetSmoothingFilters();
    // 旋转矩阵<->四元数，纯内部实现细节，不对外暴露。
    static std::array<double,4> matToQuat(const std::array<double,9>& R);
    static std::array<double,9> quatToMat(const std::array<double,4>& q);
};

} // namespace mocap
