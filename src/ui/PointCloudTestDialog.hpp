#pragma once
// ---------------------------------------------------------------------------
// 实时动捕（历史名「点云测试」，类名/文件名沿用）：不依赖手部模型/运动学链，纯粹"每台相机检测到的反光球 -> 多视角
// 聚类 -> 空间3D点云"这一层算法的独立可视化窗口。
//
// 背景：TriangulationDebugDialog 只解一个球（每台相机取检测到的第一个/
// 最大的blob）。这里用的是 reconstruct/MultiViewCluster.hpp 的
// clusterMultiView()——同一段代码正是 HandTrackingWorker 手背标定/冷启动
// 用来"不知道哪个点是哪颗marker、但知道这些点确实存在"的那一层，跟手部
// 模型完全无关，本来就该能独立跑起来看点云。
//
// 跨帧稳定编号用跟手背标定候选点同一套 TemporalTracker，不是每帧瞎编号——
// 点在空间中比较稳定地存在，编号就该跟着稳定，不然点云看起来会像"每帧
// 编号都在跳"，那反而是这个窗口最不该出现的假象。
//
// 明确不做的事：不自动连"骨架"——从空间里一堆点连成"哪个点该跟哪个点
// 连线"，需要知道这些点分别是哪颗marker(拓扑关系)，这正是手部模型标定
// 要解决的问题，这个窗口的定位是"标定之前，先看看点云本身准不准、稳不
// 稳"，不是绕过标定直接出骨架。
//
// 【记忆机制】maxSampson/maxReprojNorm(多视角聚类阈值) + maxAssocDist/
// maxMissedFrames/minHitsToConfirm(TemporalTracker跨帧关联参数，之前是
// 写死的20.0/36/6，现在暴露出来可调)全部存进 QSettings("MocapSystem",
// "PointCloudTestParams")，关掉窗口/重启程序后自动读回上次调好的值——
// "调到稳定为止"这句话隐含的前提就是调好的结果不该消失。
// ---------------------------------------------------------------------------
#include "reconstruct/MultiViewCluster.hpp"
#include "reconstruct/TemporalTracker.hpp"
#include "reconstruct/IekfPointTracker.hpp"   // 可选的卡尔曼追踪后端，见文件头注释
#include "record/PointCloudRecorder.hpp"   // 原始数据流录制，见文件头注释
#include "detect/DetectionOutput.hpp"    // 圆拟合法用：detectBalls()/Point2/Observation2D
#include "detect/CentroidDetector.hpp"   // Blob类型(轮廓点)，圆拟合法的输入来源
#include <QDialog>
#include <QVector>
#include <QVector3D>
#include <QHash>
#include <QSet>
#include <QPointF>
#include <QTimer>
#include <QElapsedTimer>
#include <QSettings>
#include <QThread>
#include <deque>
#include <vector>
#include "estimate/SkeletonAssocWorker.hpp"
#include "estimate/Hm20TemplateAdapter.hpp"   // 【新增】Hm20Template / makeHm20Template / fingerMountEdges

class QLabel;
class QListWidget;
class QListWidgetItem;
class QDoubleSpinBox;
class QSpinBox;
class QMouseEvent;
class QCheckBox;
class QComboBox;
class QPushButton;
class QLineEdit;
class QVBoxLayout;
class QHBoxLayout;

namespace mocap {

class CameraManager;
class CalibrationStore;
class ICamera;

// 点云渲染：固定世界坐标系的正交投影 + 鼠标拖拽旋转视角(orbit) + 滚轮缩放。
//
// 【跟旧版的区别，为什么改】旧版每一帧都拿这一帧的点自己的包围盒做autofit
// (缩放系数、画面中心随点的分布重新算一遍)——好处是点永远撑满画面，坏处是
// "点在世界里绝对没动，但因为别的点动了，包围盒变了，这个点在屏幕上却跟着
// 挪位置/缩放"，看起来像是整个点云在无规律抖动，实际上只是取景框在抖。
// 现在改成：世界原点固定映射到一个用户可拖拽平移的锚点，缩放系数由用户
// 滚轮控制(不随点数据变化)，视角由用户拖拽旋转(yaw/pitch，orbit相机)——
// 点在世界坐标系里的绝对位置changes多少，屏幕上就精确挪多少，画面本身
// (原点/缩放/视角)只在用户主动交互时才变，这才是"固定坐标场景"该有的样子。
class PointCloudWidget : public QWidget {
    Q_OBJECT
public:
    explicit PointCloudWidget(QWidget* parent = nullptr);
    struct Point { QVector3D pos; int id; bool coasting; bool predicted = false; };
    void setPoints(const QVector<Point>& pts);
    // 【新增】手动连线——两个点的id构成一条边，渲染时按id去points_里找
    // 当前位置连线；哪个端点这一帧不在points_里(比如被完全丢弃、不是
    // coasting那种还留着旧位置的情况)就跳过这条边，不强行连一条不存在
    // 的线。这是"绑定模式"手动点击连线用的，独立于下面的AI骨架叠加层。
    void setEdges(const QVector<QPair<int,int>>& edges);

    // 【新增，AI骨架叠加层——独立存储，不跟上面两个共用】
    // 之前的实现是把骨架点强行拼进points_、骨架边强行塞进edges_，结果是：
    // ①点云每帧刷新(setPoints)会把上一次画出来的骨架整体覆盖掉，AI比
    //   点云慢一帧就会看到骨架闪烁；②骨架边跟"绑定模式"手动连的线共用
    //   同一个edges_，谁后调用setEdges就把对方的线冲掉，AI开着的时候用户
    //   手动绑定的线会被每帧清空，反过来手动清线/连线也会把骨架冲掉。
    // 现在骨架点/边单独存一份，paintEvent里各画各的、互不覆盖：点云该
    // 多快刷多快，骨架该多久刷一次是AI线程自己的节奏，两条手动绑定线也
    // 完全不受影响。
    void setSkeletonOverlay(const QVector<Point>& pts, const QVector<QPair<int,int>>& edges);
    // 追踪点 id -> 语义标签短名（"index.dp" 之类）。骨架层每帧告诉点云层
    // "这个点被判成了什么"，点云层据此在点旁标注，取代原来那串没有含义的
    // 追踪序号 —— 序号只在调试点云本身时有用，看骨架对不对时它是噪声。
    void setPointLabels(const QHash<int, QString>& labels);
    // 骨架层自己的标签。预测点在点云层没有对应点，只能在这里标。
    void setSkeletonLabels(const QHash<int, QString>& labels);

    // 视角复位：yaw/pitch归零、缩放/平移回默认——鼠标拖多了转晕的时候用。
    void resetView();

signals:
    // 命中判定复用同一份"当前帧实际画出来的屏幕坐标"做最近点+命中半径
    // 判定，保证点击命中的就是画面上看到的那个点。
    void pointClicked(int id);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;
    void wheelEvent(QWheelEvent* ev) override;

private:
    QVector<Point> points_;
    QVector<QPair<int,int>> edges_;
    // paintEvent 里算出来的投影变换结果，mousePressEvent 复用同一份
    // (不是重新算一次)，保证点击命中检测跟屏幕上实际画的位置完全一致。
    // 【注意】这份只含points_(点云本身)，故意不把骨架叠加层的点混进来——
    // 命中检测/点击绑定连线是点云原有功能，骨架点用100000+的假id，混进来
    // 会导致点击附近骨架点时触发一次莫名其妙的"绑定"，属于新功能污染
    // 旧功能，不该发生。
    QHash<int, QPointF> lastScreenPosById_;

    // 【新增】AI骨架叠加层——独立存储，不参与命中检测，也不跟points_/edges_
    // 混用，见头文件 setSkeletonOverlay() 的说明。
    QVector<Point> skeletonPoints_;
    // 见 setPointLabels()。空 = 回退显示追踪 id。
    QHash<int, QString> pointLabels_;
    // 骨架点自己的标签（id -> 短名）。预测点在点云里没有对应的点，
    // 标签只能画在骨架层这一遍上。
    QHash<int, QString> skelLabels_;
    QVector<QPair<int,int>> skeletonEdges_;

    // ---- 固定世界坐标 + 用户控制的视角状态(不随点云数据变化) ----
    double yawDeg_ = 30.0;      // 绕世界Z轴(up)的水平旋转角
    double pitchDeg_ = -25.0;   // 俯仰角，负=从上往下看
    double scalePxPerMm_ = 0.6; // 缩放：每mm对应多少像素——固定值，不是autofit算出来的
    QPointF panPx_{0, 0};       // 世界原点在窗口里的屏幕偏移(像素)，默认放在窗口中心，可拖拽平移
    bool rotating_ = false;
    bool panning_ = false;
    QPoint lastMousePos_;

    // 世界坐标(mm) -> 屏幕坐标(px)：先按yaw/pitch做真正的3D旋转投影(正交，
    // 不透视)，再乘固定缩放、加平移偏移。跟旧版obliqueProject的区别是这里
    // 完全不看points_的包围盒，纯粹是"世界坐标系固定摆在那"的变换。
    QPointF worldToScreen(const QVector3D& p) const;
};

class PointCloudTestDialog : public QDialog {
    Q_OBJECT
signals:
    // 把骨架链路的输出转发给 MainWindow 的 UdpSender。
    // 【注意别插到本文件上面那个 PointCloudWidget 里】两个类都在这个头文件里，
    // 而且 PointCloudWidget 先出现、它有 signals: 段，按 "第一个 signals:"
    // 定位会插错类，MOC 生成的代码能编过、只有 MainWindow 里的 connect 会报
    // "is not a member of PointCloudTestDialog"。
    void handPoseForUdp(QVector3D wristPos, QVector<double> wristRot9,
                        QVector<double> jointAngles16, qint64 ts_ns);
    void bendRefForUi(QVector<double> fingerBend5, qint64 ts_ns);
    void segmentQuatsForUdp(QVector3D wristPos, QVector<double> quatWorld64,
                            QVector<double> quatLocal64, QVector<int> segSource16,
                            quint32 flags, qint64 ts_ns);

public:
    PointCloudTestDialog(CameraManager* mgr, CalibrationStore* store,
                         bool wasDetectOn, QWidget* parent = nullptr);
    ~PointCloudTestDialog() override;

    // 主窗口的 UDP 推送开关状态，转给输出监视面板显示。
    // 【值得单独接一根线】最常见的困惑是"面板明明在刷数，Unity 什么都收不到"，
    // 九成是主窗口那个「UDP推送」没打开。与其让人去翻另一个窗口，不如直接写在
    // 读数面板上。不调这个函数也不影响功能，只是面板上写"(未知)"。
    void setUdpStatus(bool enabled, const QString& target);

private slots:
    void rebuild();
    void onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns);
    void onBlobDetails(quint32 camId, const QVector<Blob>& blobs, qint64 ts_ns);
    void onBatchTimer();
    void onPointClicked(int id);
    void onBindModeToggled(bool on);
    void onClearEdges();

private:
    void tryCluster();
    // 【拆文件】构造里“参数侧栏分节填充”那一段（原构造函数 632~2379 行）抽成私有方法，
    // 实现在 PointCloudTestDialog_panel.cpp。三个参数是构造函数里创建后传进来的局部布局指针。
    void buildParamSections(QVBoxLayout* ctlCol, QHBoxLayout* quickRow, QHBoxLayout* statusStrip);

    CameraManager* mgr_;
    CalibrationStore* store_;
    bool wasDetectOn_ = true;
    QSet<ICamera*> weTurnedOn_;

    QListWidget* list_ = nullptr;
    QLabel* status_ = nullptr;
    // 【新增】检测定位算法切换——质心法(默认，CentroidDetector直接输出的
    // 灰度加权质心，走blobsReady信号) vs 圆拟合法(项目里已有的遮挡感知
    // 检测层3a~3d，detect/DetectionOutput.hpp::detectBalls()，走
    // blobDetailsReady信号，需要轮廓点+已知半径先验)。切换会触发rebuild()
    // (需要重新订阅不同的信号、开关轮廓采集，不是简单setter能做到的)。
    // 两种算法用同一套下游(聚类+追踪+统计)，方便直接对比。
    QCheckBox* useCircleFitChk_ = nullptr;
    QDoubleSpinBox* circleRadiusPxSpin_ = nullptr;   // 圆拟合法需要的已知球半径先验(像素，图像空间)
    // 【新增】圆拟合的轮廓点数上限(0=不限)。跟检测参数面板里的
    // minArea/maxArea/minCircularity 是【互补】不是重复：那三个按面积/圆度
    // 判"这个连通域算不算球"，在 CentroidDetector 里就过滤掉了；这个按
    // detectBalls 的真实成本变量——【轮廓点数】——设限。又细又弯带毛刺的斑
    // 面积可能完全达标(maxArea 拦不住)，周长却极大，而圆拟合代价随轮廓点数
    // 线性放大。超限的 blob 不丢，退回用检测器自带的灰度加权质心。
    // 详见 onBlobDetails() 里的完整说明。
    QSpinBox* maxContourPtsSpin_ = nullptr;
    // 借用轮廓采集开关——跟 weTurnedOn_/wasDetectOn_ 同一个思路：只有
    // "轮廓采集之前确实是关着的、是我们打开的"这种相机，关闭圆拟合模式/
    // 关窗口时才由我们负责关掉；如果某台相机的轮廓采集本来就因为别的
    // 消费者(比如同时开着的HandTrackingWorker)开着，我们不动它。
    QSet<ICamera*> weTurnedOnContour_;
    QDoubleSpinBox* maxSampsonSpin_ = nullptr;
    QDoubleSpinBox* maxReprojSpin_ = nullptr;
    // 【新增】MultiViewCluster::clusterMultiView 的 minSupportViews——
    // 一个候选点最少要几台相机同时验证到才算"存在"，见 MultiViewCluster.hpp
    // 里 effectiveMinSupport 的说明。默认3(第三台相机当裁判防幽灵)，调小
    // 到2可以在相机数>2但点经常被部分遮挡的场景下大幅减少"明明有效观测
    // 却被判定不存在"的情况，代价是幽灵点过滤能力下降。
    QSpinBox* minSupportSpin_ = nullptr;
    // 【新增】useVoting——是否启用多相机投票仲裁，相机数量少(尤其2台)时
    // 这套机制基本没有额外收益，可以关掉退回最简单的两视图三角化。
    QCheckBox* useVotingChk_ = nullptr;
    // 【新增】minRayAngleDeg——两视图种子的最小视线夹角，专治"远处突然
    // 冒出幽灵点"(病态三角化：视线夹角太小导致噪声在深度方向被急剧放大)。
    // 纯几何条件判断，跟场景大小/点的空间位置无关，默认0=不启用。
    QDoubleSpinBox* minRayAngleSpin_ = nullptr;
    // 【新增】ambiguityMargin——聚类投票的歧义边界(Lowe比率检验的绝对差
    // 版)。⚠实测结论：在稠密簇(如戴手套20球)里会误删真点、有害，仅对
    // 稀疏、球间距大的场景安全。默认0=关，手套场景务必保持0。做成开关只为
    // 保留能力、不删代码，不代表推荐开启。
    QDoubleSpinBox* ambiguityMarginSpin_ = nullptr;
    // 【新增】圆拟合法输出的Observation2D::sigma(每个观测各自的协方差，
    // 弧越短各向异性膨胀越厉害)之前算出来完全没被用上——这个开关接入
    // clusterMultiView本来就支持的马氏距离投票(obsCovPerCam参数)，让
    // 聚类门控按"这个观测有多可信"自适应，而不是固定的maxReprojNorm。
    // 只在圆拟合法模式下有意义(质心法没有单观测协方差可用，勾选了也没
    // 数据可传，自动跟没勾一样)。
    QCheckBox* useMahalanobisChk_ = nullptr;
    // 【记忆机制】TemporalTracker跨帧关联参数——之前是构造时写死的常量
    // (20.0mm/36帧/6帧确认)，这几个数直接决定点云"编号飙升/抖动"这类
    // 问题，现在暴露成可调控件，改动立刻生效(rebuild())并写入QSettings，
    // 下次打开(哪怕重启程序)自动读回上次调好的值。
    QDoubleSpinBox* assocDistSpin_ = nullptr;      // TemporalTracker::maxAssocDist_(mm)
    QSpinBox* maxMissedSpin_ = nullptr;            // TemporalTracker::maxMissedFrames_
    QSpinBox* minHitsSpin_ = nullptr;              // TemporalTracker::minHitsToConfirm_
    // 【新增】关联算法开关——贪心(默认) vs 匈牙利全局最优，见
    // TemporalTracker.hpp 文件头注释。勾选后调用运行时setter立即生效
    // (不重建tracker_、不清空已有编号)，方便直接在同一段数据上对比两种
    // 算法的表现；下面的稳定性统计会分别继续累计，勾选前后的数字放在
    // 一起看就是最直接的A/B对比。
    QCheckBox* useHungarianChk_ = nullptr;

    // 【新增】TemporalTracker几个更进阶的系数，同样做成可调+可选(默认值
    // 全部跟原来写死的常量一致，不勾选/不改动完全不影响原有行为)。见
    // TemporalTracker.hpp 对应setter的说明。
    QCheckBox* useAdaptiveCapChk_ = nullptr;
    QDoubleSpinBox* adaptiveCapMulSpin_ = nullptr;
    QCheckBox* useMissedRelaxChk_ = nullptr;
    QDoubleSpinBox* relaxGrowthSpin_ = nullptr;
    QDoubleSpinBox* relaxCapSpin_ = nullptr;
    // 【新增】速度平滑(EMA)/恒加速度模型——见TemporalTracker.hpp对应setter
    // 的说明，都默认关闭、不影响原有行为。
    QCheckBox* useVelSmoothChk_ = nullptr;
    QDoubleSpinBox* velSmoothAlphaSpin_ = nullptr;
    QCheckBox* useConstAccelChk_ = nullptr;
    // 【新增】追踪后端整体切换——启发式(TemporalTracker，默认) vs
    // IEKF卡尔曼(IekfPointTracker.hpp)，见该文件头注释。这是"整个换一套
    // 追踪逻辑"级别的切换，不是某个小参数，所以单独一个开关+两个过程
    // 噪声参数，且用rebuild()生效(两个追踪器都要重新构造，不能只切一个
    // 指针了事——各自维护的轨迹状态不能跨后端复用)。
    QCheckBox* useIekfBackendChk_ = nullptr;
    QDoubleSpinBox* iekfPosProcessVarSpin_ = nullptr;
    QDoubleSpinBox* iekfVelProcessVarSpin_ = nullptr;

    // 【新增·IEKF自适应门控相关开关】全部可独立开关，默认值按合成实验实测
    // 结论定(见 IekfPointTracker.hpp 文件头)。热改即可(不影响已有轨迹身份)，
    // 不需要rebuild()。tryCluster()每帧从这些控件实时把值灌进iekfTracker_。
    QCheckBox* iekfMahaGateChk_ = nullptr;        // 马氏/卡方门控(默认开)
    QDoubleSpinBox* iekfChiSquareSpin_ = nullptr; // 卡方阈值(3自由度)
    QCheckBox* iekfDualAnchorChk_ = nullptr;      // 双锚点关联(默认开·大晃动失点的主解药)
    QCheckBox* iekfCoastSuppressChk_ = nullptr;   // coast幽灵抑制(默认开)
    QDoubleSpinBox* iekfVelCapGainSpin_ = nullptr;// confirmed速度自适应上限增益(0=退回固定门控)
    QCheckBox* iekfManeuverQChk_ = nullptr;       // 机动自适应过程噪声(默认关·实测有害)
    QSpinBox* iekfMaxConfirmedSpin_ = nullptr;    // 基数上限(0=关；手套设20)
    // 【新增·密度自适应门控】把固定关联距离升级成随局部歧义度自适应：孤立
    // 候选(附近只有它一个)可够到很远→稀疏快速点速度上限大幅提高；稠密处
    // 自动收紧→零副作用。实测单点速度上限31→>104mm/帧，稠密手套coverage不降。
    QCheckBox* iekfDensityGateChk_ = nullptr;
    QDoubleSpinBox* iekfDensityReachSpin_ = nullptr;  // 孤立候选最大可够到 maxAssocDist 的倍数
    QDoubleSpinBox* iekfDensitySepSpin_ = nullptr;    // "孤立"判据:次近须>=本值×最近

    // 【新增】检测噪声(像素)——IEKF的 defaultObsVar_ 从硬编码常量改为按物理量
    // 反推：defaultObsVar = (噪声px / fx)²。旧硬编码 1e-5 在 fx≈900 下等价于
    // σ≈2.85px，比典型检测噪声(0.3~0.8px)粗了一个数量级，滤波器因此严重低估
    // 自己的测量精度、不敢信观测、状态滞后——这是压测里找到的单个最大杠杆。
    // 只在"没有勾选马氏距离门控"(即 clusterMultiView 没给出per-观测协方差、
    // IEKF退化用这个全局各向同性值)时真正生效；勾了马氏门控后每个观测有自己
    // 的真实协方差，这个值只在个别观测缺协方差时兜底。
    // fx 从当前已标定、参与聚类的相机实时取平均，不同相机焦距差异不大时这个
    // 近似足够；每帧都会用最新的活跃相机集合重新换算，切换相机组合后自动跟上。
    QDoubleSpinBox* iekfDetectNoisePxSpin_ = nullptr;

    // 【新增·③输出端One Euro滤波】跟三角化调试窗口验证过的是同一个算法
    // (OneEuroFilter3)，两个追踪后端(TemporalTracker/IekfPointTracker)共用
    // 这组控件——切换后端时都读同样的值，不需要维护两份重复UI。默认关，
    // 见cpp里连接处的完整说明(静止点抖动RMS可从1.37mm降到0.21mm)。
    QCheckBox* useOutputFilterChk_ = nullptr;
    QDoubleSpinBox* outputFilterMinCutoffSpin_ = nullptr;
    QDoubleSpinBox* outputFilterBetaSpin_ = nullptr;

    // 【新增·②鲁棒IRLS内部参数】此前硬编码在clusterMultiView调用处的
    // TriangulationRefineConfig{}默认值(0.01/15)，现在可调。
    QDoubleSpinBox* lmHuberDeltaSpin_ = nullptr;
    QSpinBox* lmMaxItersSpin_ = nullptr;

    // 【新增·标定不确定度门控项】clusterMultiView 的 calibSigmaNorm 参数此前
    // 只有函数签名、没有任何调用方传值，实际运行时永远是默认 0——也就是压测
    // 报告 §5.3 那个"2px 主点误差下召回率 0.13→0.99"的修复在真实程序里从未
    // 生效过。这里补上控件，让它真正可用。
    QDoubleSpinBox* calibSigmaNormSpin_ = nullptr;
    // 【新增】两视图降级通道：某点只被2台相机看见时是否仍然出点。
    QCheckBox*      twoViewFallbackChk_ = nullptr;
    QDoubleSpinBox* twoViewMinRayAngleSpin_ = nullptr;
    QDoubleSpinBox* maxFinalResidualSpin_ = nullptr;
    // ---- 观测时刻补偿（相机不同步 / 曝光中点）----
    QCheckBox*      iekfDesyncCompChk_ = nullptr;        // 用真实时间戳补相机间不同步
    QDoubleSpinBox* iekfExposureMsSpin_ = nullptr;       // 曝光时长(ms)，0=不补运动模糊
    QComboBox*      iekfExposureAnchorCombo_ = nullptr;  // 时间戳打在曝光窗口哪一端
    // 参考时刻与实测帧周期（由各相机时间戳推导，用于 dt 真实化）
    qint64          lastRefTsNs_ = -1;
    double          emaFrameNs_ = -1.0;
    std::vector<double> camOffEma_;                    // 每相机时刻偏移的EMA(帧)
    static constexpr double kOffUninit   = -9.0;       // 哨兵：尚未初始化
    static constexpr double kOffEmaAlpha = 0.05;       // ≈20帧有效窗口
    // 圆拟合法分流统计：上一帧走质心快路径的单球数 / 进圆拟合的花生数 /
    // 被兜底闸挡下(退回质心、不做圆拟合)的 blob 数。
    // lastFrameSkippedBlobs_ 长期大于0 说明画面里有轮廓过大的亮区(相机没
    // 摆好/阈值太低/曝光跑飞)，该去检测参数面板调 threshold、maxArea、
    // minCircularity——它把"以前只能靠界面卡死才发现的问题"变成一个可读数字。
    int lastFrameSingleBlobs_ = 0;
    int lastFramePeanutBlobs_ = 0;
    int lastFrameSkippedBlobs_ = 0;

    QLabel* obsVarPreviewLabel_ = nullptr;   // 显示"当前换算成方差=X.Xe-N，等效fx=NNN"，核对用

    // 记住上一次实际换算出来的 defaultObsVar，供状态栏/工具提示展示，方便
    // 核对"我现在填的px到底换算成了多大的方差"。
    double lastAppliedDefaultObsVar_ = 1e-5;

    // 【新增·补全此前未暴露到UI的全部参数】下面这批之前只存在于
    // IekfPointTracker.hpp/MultiViewCluster.hpp内部(有setter/参数位但UI没接
    // 控件、只能用默认值)。跟其余IEKF控件同一套热改逻辑：改了立即调运行时
    // setter，不rebuild()，不影响已有轨迹身份。
    QCheckBox* iekfUseAutoFloorChk_ = nullptr;     // 关联协方差地板：自动(0.35×关联距离)² vs 手动填值
    QDoubleSpinBox* iekfFloorVarSpin_ = nullptr;   // 手动地板值(mm²)，只在上面那个复选框不勾时生效
    QDoubleSpinBox* iekfConfirmedCapMaxSpin_ = nullptr;   // 已确认轨迹速度放宽的硬顶倍数
    QCheckBox* iekfGlobalAssignChk_ = nullptr;     // IEKF自己的全局最优指派(独立于TemporalTracker的"匈牙利")
    QCheckBox* iekfTentRampChk_ = nullptr;         // tentative速度爬坡
    QDoubleSpinBox* iekfTentRampHitsSpin_ = nullptr; // 爬坡到满速度放宽需要的命中次数
    QDoubleSpinBox* iekfManeuverThreshSpin_ = nullptr; // 机动信号阈值(归一化残差/关联距离比值)
    QDoubleSpinBox* iekfManeuverBoostMaxSpin_ = nullptr; // 机动触发后过程噪声放大倍率上限
    QDoubleSpinBox* iekfManeuverDecaySpin_ = nullptr;    // 机动boost每帧向1衰减的比例

    // 【新增·观测方差地板】填的是像素，内部按 (σ_px/fx)² 换算成归一化方差交给
    // IekfPointTracker::setObsVarFloor()。压测实测(20点手部场景)：检测层报的
    // 协方差过于乐观(σ=1e-4px)时，IEKF 近乎绝对相信观测，单次误关联就变成
    // 78mm 的状态跳变，300帧内 54 次编号跳变；加上 0.5px 地板后跳变归零、
    // RMSE 从 6.9mm 降到 0.001mm，而 0.3/0.8px 正常噪声下结果完全不变(零代价)。
    QDoubleSpinBox* iekfObsFloorPxSpin_ = nullptr;

    QCheckBox* useLmRefineChk_ = nullptr;          // 聚类阶段LM非线性精修候选点位置(默认开)
    QDoubleSpinBox* clusterChiSquareSpin_ = nullptr; // 聚类"考其余相机"投票用的马氏卡方阈值(跟IEKF那个是两码事)

    // 【新增】编号稳定性量化统计——回答"这个点云到底稳不稳"不能靠肉眼数，
    // 需要一个硬数字。做法：记下"统计起点"那一刻tracker_.totalIdsAssigned()
    // 的值，此后每帧算 (当前totalIdsAssigned() - 起点) = 统计区间内新增的
    // ID总数，跟同一区间内"同时活跃编号数的峰值"相除——理想情况(点没有
    // 被重新编号过)这个比值应该等于"实际反光球数量"，也就是说，如果峰值
    // 活跃数是5(比如5颗反光球都在)，新增ID总数也应该稳定在5左右；如果
    // 新增ID总数持续增长(6、7、8...)而峰值活跃数一直是5，说明同样这5颗
    // 球在不断被重新分配新编号，比值越大越不稳定。"可重置"是因为调参、
    // 切换算法这些动作之间需要能各自独立评估，不重置的话早期的抖动会
    // 一直污染后面调好之后的统计。
    QLabel* stabilityLabel_ = nullptr;
    QPushButton* resetStatsBtn_ = nullptr;
    int statsBaselineTotalIds_ = 0;    // 重置那一刻的 tracker_.totalIdsAssigned()
    int statsPeakActive_ = 0;          // 重置以来，同时活跃(confirmed)编号数的峰值
    void resetStabilityStats();        // 复位统计起点，不影响tracker_内部状态/已有编号
    void applyIekfDefaultObsVar();
    void applyIekfObsFloor();          // 把"观测方差地板(px)"按fx换算后灌给iekfTracker_     // 用当前活跃相机的fx，把"检测噪声(px)"换算成iekfTracker_的defaultObsVar

    // 【新增】逐段耗时统计——排查"编号流失是不是因为处理跟不上帧率、
    // 静默丢帧"这个假设专用。QElapsedTimer精度到纳秒，这几段操作通常在
    // 亚毫秒到几毫秒量级，用毫秒级的QTimer/elapsed()精度不够看出差异。
    // lastCallTimer_ 量的是"两次tryCluster()被调用之间实际隔了多久"——
    // 如果这个数字明显大于相机的名义帧间隔(比如60fps该是16.7ms，实测
    // 变成30+ms)，说明处理没跟上，latestBlobs_被后到的观测静默覆盖掉了
    // 中间那些帧，追踪器看到的"相邻两帧"其实间隔了不止一帧的运动量，
    // 关联门控自然更容易被打穿——这跟"点真的动得快"从现象上完全一样，
    // 但根因和解法完全不同，必须先测出来，不能猜。
    QLabel* timingLabel_ = nullptr;
    QElapsedTimer lastCallTimer_;
    bool lastCallTimerValid_ = false;
    double emaIntervalMs_ = 0.0;   // 两次tryCluster()调用的实际间隔(EMA平滑)
    double emaBuildMs_ = 0.0;      // 构建obsPerCam+协方差转换耗时(EMA)
    double emaClusterMs_ = 0.0;    // clusterMultiView()耗时(EMA)
    double emaTrackMs_ = 0.0;      // tracker_/iekfTracker_.update()耗时(EMA)
    double emaTotalMs_ = 0.0;      // 本函数总耗时(EMA)
    double maxTotalMs_ = 0.0;      // 峰值总耗时(自上次"重置统计"以来)

    // 【新增】骨骼绑定测试——手动点两个点连一条线，实时叠加渲染在点云上，
    // 看连出来的"骨架"动作对不对得上，不经过任何标定/运动学模型，纯粹
    // "这个点跟那个点应该是同一段骨头两端"这个人工判断。
    //
    // 【必须说清楚的限制，不是隐藏的坑】这里连的是 TemporalTracker 当前
    // 分配的原始id，不是稳定的解剖学身份——app重启、相机组合换过、或者
    // 某个点被遮挡太久超过"遮挡记忆帧数"直接销毁重新分配了新id，这份
    // 连线就对不上了，需要重连。这跟 BackTemplateCalibPage 保存的手部
    // 模板不是一回事：那边存的是真实3D几何(骨长/锚点)，重启后一样能用；
    // 这里存的只是"当前这次追踪会话里，这个编号跟那个编号连线"，不具备
    // 跨会话的身份稳定性，所以不做QSettings持久化——存了也大概率是错的，
    // 不如老实告诉你"这只在当前会话内有效"。
    QCheckBox* bindModeChk_ = nullptr;
    QPushButton* clearEdgesBtn_ = nullptr;
    QPushButton* resetViewBtn_ = nullptr;   // 视角转晕了一键归位——不影响点云数据/编号，纯视觉状态
    QLabel* bindStatusLabel_ = nullptr;
    QVector<QPair<int,int>> edges_;
    int pendingFirstId_ = -1;   // 绑定模式下，点了第一个点还没点第二个时，暂存第一个点的id；-1=没有待连接的点
    PointCloudWidget* view_ = nullptr;
    QSettings settings_{QStringLiteral("MocapSystem"), QStringLiteral("PointCloudTestParams")};

    struct CamEntry { ICamera* cam; QString deviceKey; };
    QVector<CamEntry> activeCams_;

    // covs跟pts一一对应(圆拟合法+勾选马氏距离门控时才非空；质心法/没勾选
    // 时恒为空，tryCluster()据此决定要不要把协方差传给clusterMultiView)。
    // 单位是像素(圆拟合法Observation2D::sigma的原始单位)，tryCluster()里
    // 转换成归一化坐标系再喂给clusterMultiView(那边要的是跟obsPerCam同一
    // 套坐标系下的协方差)。
    struct BlobBuf { QVector<QPointF> pts; QVector<Cov2> covs; qint64 ts = -1; };
    QHash<quint32, BlobBuf> latestBlobs_;
    QTimer* batchTimer_ = nullptr;
    // 【改动】不再是写死的12ms——这个窗口本质是"等同一时刻各相机的观测
    // 都到齐再一起处理"，不是给计算量腾时间(计算本身早就测出来只要
    // 0.7ms峰值，完全不是瓶颈)。写死的猜测值可能比相机真实帧间隔还长
    // (合并了两个不同时刻的真实帧，拉低有效追踪帧率)，也可能比相机间
    // 实际到达偏差还短(还没等齐所有相机就提前处理，凑不够minSupportViews
    // 白白丢点)——这两种情况都会被误判成"追踪不稳/帧率不够"，但根因和
    // 解法完全不同。做成可调参数，配合下面的到达偏差实测数据去调，而
    // 不是继续猜一个数字。
    QSpinBox* batchWindowSpin_ = nullptr;
    // 【新增】相机间实际到达时间偏差实测——每个批次第一台相机的观测
    // 到达时刻算作t=0，记录同一批次里其它相机各自的到达偏移，
    // tryCluster()处理这一批时算出最大偏移(spread)，这就是"批处理窗口
    // 至少要设多大，才不会漏掉还没到的相机"的下限参考值。
    QElapsedTimer batchClock_;
    QHash<quint32, double> arrivalMsThisBatch_;
    // 【窗口无关的到达偏差】每台相机【最近一次投递】的墙钟时刻，全局保留、
    // 不随批次清空。
    //
    // 原来用 arrivalMsThisBatch_（每批清空重记）算偏差是错的：批次的长度
    // 就是批处理窗口，窗口越宽、越多越晚的到达被算进同一批，测出来的偏差
    // 就越大 —— 于是"调大窗口 → 建议值也变大 → 永远追不上"。测量被它要
    // 评价的那个参数本身污染了。
    //
    // 换成"各相机最近一次投递时刻的离散度"：每台每帧周期都会投递一次，
    // 这个量只反映相机之间的相位差，跟批处理窗口没有任何关系。
    QHash<quint32, qint64> lastArrivalNs_;
    // 相机自己的投递周期(EMA)。用来算"窗口吃掉了多少帧"——
    // 判据只说"窗口合理"是不够的：窗口合理但输出帧率只有相机的一半，
    // 那才是用户真正在付的代价，而它在原来的提示里完全看不见。
    double emaCamPeriodMs_ = 0.0;
    QElapsedTimer arrivalClock_;
    double emaArrivalSpreadMs_ = 0.0;
    // 【为什么不再用峰值判据】maxArrivalSpreadMs_ 是开机以来的全局最大值、
    // 永不衰减：启动那一下或者某次系统调度抖动打出一个孤立的 40ms，
    // "窗口太紧"的警告就永远亮着，而实际 EMA 可能只有 2ms。
    // 一个永远亮着的警告等于没有警告，还会让人把真的问题也一起忽略掉。
    //
    // 改成【滑动窗口的高分位】：只看最近 N 次的分布，孤立尖峰会被 p95 滤掉，
    // 而真的持续偏大会被如实反映；窗口滚动所以它会自己恢复。
    std::deque<double> arrivalSpreadHist_;
    static constexpr int kArrivalHistN = 240;   // ~10s @24fps
    double p95ArrivalSpreadMs_ = 0.0;   // 最近窗口的 95 分位
    double maxArrivalSpreadMs_ = 0.0;   // 同窗口内的最大值，只作展示不作判据

    TemporalTracker tracker_{20.0, 36, 6};   // 跟候选点标定同一套参数(见HandTrackingWorker::candidateTracker_的说明)
    IekfPointTracker iekfTracker_{20.0, 36, 6};   // 可选后端，参数含义跟tracker_对齐，两者不会同时活跃

    // 【新增】骨骼关联叠加层——独立于上面两个点云追踪后端，纯粹"这一帧的
    // 点云该怎么连成骨架"。跟 HandTemplateStore 共用同一份标定(标定向导
    // 存的那份)，不需要重新标定。backend_ 是 onnxruntime 实现，见
    // estimate/Hm20OnnxBackend.hpp——没装 onnxruntime 时是个
    // 桩(ready()==false)，骨架显示自动关闭，不影响点云本身正常工作。
    std::unique_ptr<class HandTemplateStore> handTemplateStore_;
    // 【注意】这里故意不再持有 backend / associator ——onnxruntime 的加载、
    // 建会话、推理全部发生在 SkeletonAssocWorker 所在的工作线程里，GUI
    // 线程一概不碰(那些操作在Debug下接近1秒，还会往stderr刷大量日志，
    // 放在GUI线程会让界面卡住)。
    // 【改动】不再是unique_ptr——要交给独立线程里的SkeletonAssocWorker长期
    // 持有(configure()时传一份shared_ptr过去)，本对话框和worker线程共享
    // 同一个实例，process()内部只读不写，多线程调用是安全的(见
    // SkeletonAssocWorker.hpp头注释)。
    hm20::Hm20Template skeletonTemplateForRender_;   // 跟worker里用的是同一份，渲染挂载边(fingerMountEdges)要用
                                                     // 【改动】原类型 SkeletonTemplate 属于上一代，已不存在
    QCheckBox* showSkeletonChk_ = nullptr;

    // 【新增】骨骼关联独立工作线程——照 detect/DetectWorker.hpp那套"独立线程+
    // 忙时丢帧"的模式，process()本身不该慢，但一旦某次卡住(比如onnxruntime
    // 内部意外阻塞)，绝不能把点云面板一起拖下水。跟采集/检测线程同一个
    // 生命周期管理方式：dtor里quit()+wait()，不靠Qt父子对象自动清理
    // (moveToThread过的对象那套机制管不了)。
    QThread* skeletonThread_ = nullptr;
    class SkeletonAssocWorker* skeletonWorker_ = nullptr;
    // 【新增】IK 精修：手指分段朝向。computeSegmentQuats() 用 marker[b]-marker[a]
    // 当骨轴，而球贴在指节背侧、离骨轴 11~17mm 且相邻两节方位角不同 —— 连线
    // 必然偏离骨轴（HandSkeletonAssociator.hpp 里"剩下的 11.0° 中位偏差消不掉"
    // 说的就是这个）。合成数据实测 10.2°中位/21.6°p90 -> 3.02°/8.08°。
    // 【线程】它有时序先验(prevAngles_)，是有状态的；构造完就交给 worker 线程，
    // GUI 线程之后不再碰（下面只在 configure 之前 set 一次）。
    // 【存基类指针】具体类型 Hm20IkRefiner 只在 .cpp 里出现 —— 这个 hpp 刻意
    // 用前向声明避免拉重头文件（同上面的 class SkeletonAssocWorker*）。
    // IHm20IkRefiner 由 HandSkeletonAssociator.hpp 提供，已经可见。
    std::shared_ptr<hm20::IHm20IkRefiner> skeletonIk_;
    QCheckBox* useIkRefineChk_ = nullptr;
    // 诊断/统计——回答"AI这一步到底有没有生效、卡在哪"，不需要用户自己猜：
    QLabel* skeletonDiagLabel_ = nullptr;
    // ---- 【新增】输出链路延迟：开关 + 实测读数 ----
    // 【为什么单独成组】"连线不跟手"有四个来源，量级完全不同，混在一起只能靠猜：
    //   ① 骨架调用限流   原来写死 33ms(30Hz)：平均 +16ms、最坏 +33ms 的陈旧，
    //                    而点云本身可能跑 100fps 以上 —— 肉眼看就是"点在动、线在追"
    //   ② AI 线程被饿死   worker 线程原来固定 LowestPriority，4 路相机满载时
    //                    它抢不到核心，排队时间能远超推理时间本身
    //   ③ 时序滤波       Hm20PoseFilter，默认"均衡"档就是开着的
    //   ④ 速度限幅       只影响 UDP 那一路，不影响面板上的连线
    // 所以这里给一个总开关做 A/B，再把 ①②③④ 拆开，最后把实测毫秒数打在面板上：
    // 不靠"感觉慢"，靠读数。
    QCheckBox*   lowLatencyChk_ = nullptr;    // 直通：一键旁路整条输出滤波链
    QComboBox*   skelRateCombo_ = nullptr;    // 骨架刷新率（原来写死 30Hz）
    QCheckBox*   rateLimitChk_  = nullptr;    // 关节角速度限幅（UDP 那一路）
    QLabel*      latencyLabel_  = nullptr;    // 实测滞后读数
    QDoubleSpinBox* thumbRollSpin_ = nullptr; // 拇指 roll 偏置(度)——建模参数，只能对着画面调
    QCheckBox*   thumbPronChk_ = nullptr;     // 是否回正遮挡时网络补出来的拇指点
    QCheckBox*   ikOnlyChk_ = nullptr;        // 遮挡点只接受 IK 的位置，否则保持
    QCheckBox*   geoRelabelChk_ = nullptr;    // 手背标签用几何重定而不信模型
    int          skelDispatchMinMs_ = 33;     // 两次提交之间至少间隔多久，0=每帧都投
    // 端到端计时：单调时钟 + "这一帧是什么时候投出去的"。worker 同时最多在处理
    // 一帧（pending 槽位保证），所以结果回来时用同一个 dispatchNs_ 相减就是准确的
    // "提交 -> 结果回到 GUI"耗时，不需要给每帧编号。
    QElapsedTimer pipeClock_;
    qint64        dispatchNs_ = -1;
    double        emaE2eMs_ = -1.0;           // 端到端(排队+推理+回传)，EMA
    double        emaSkelIntervalMs_ = -1.0;  // 两次结果之间的实际间隔 = 骨架真实刷新率

    // 【新增】在线自标定 / 时序滤波
    QPushButton* romBtn_ = nullptr;
    QCheckBox* handRightChk_ = nullptr;
    QCheckBox* autoCalibChk_ = nullptr;
    QPushButton* autoCalibResetBtn_ = nullptr;
    QComboBox* smoothCombo_ = nullptr;

    // 【新增】输出数据监视面板 —— 把链路最终发出去的 M3DS/M3DQ 逐帧打印出来。
    // 【数据源刻意接的是本对话框自己的两个 signal】它们同时也是 MainWindow
    // 转给 UdpSender 的那两个，所以"面板上显示的"和"UDP 上发出去的"是同一次
    // emit 的两个消费者，逐位一致。不要改成从 onSkeletonResultReady 的 result
    // 重算 —— 那会跳过 时序滤波 / ROM 映射 / 速度限幅 这三步，而链路上最需要
    // 被看见的恰恰是这三步。
    // 默认不显示；不显示时 setActive(false)，槽函数直接 return、刷新定时器停转，
    // 对原有链路零开销。
    class HandOutputMonitor* outputMonitor_ = nullptr;
    QCheckBox* showOutputChk_ = nullptr;

    // ---- 原始数据流录制 ----
    mocap::pcrec::Recorder recorder_;
    // ROM 标定期间的引导文字（按钮旁边）
    QLabel* romHintLabel_ = nullptr;
    // 把 ROM 按钮的两个瞬间写进录制文件。
    // 【为什么单独抽一个函数】它要在 recorder_ 没在录制时静默跳过，
    // 而按钮回调里再写一遍 if 会让那段本来就长的 lambda 更难读。
    void recordRomEvent(int code, const QString& text, int intA);

    // ---- 面板控件操作的全量事件记录（v8.1，实现在 _uievents.cpp）----
    // 【为什么要全量】面板 87 个控件，而 1Hz 的 runtimeConfig 只覆盖 26 个，
    // 逐帧 RunFlags 只有 20 个布尔位 —— 剩下的值在文件里【一个字节都没有】。
    // 而且 runtimeConfig 是 1 秒一采样：一次"调大看看→不对→调回去"全程
    // 不到一秒，在文件里完全不存在，偏偏这种试探性调整是排查现场最常发生、
    // 事后最容易被本人忘掉的事。
    void installUiEventRecorder();
    void recordUiEvent(const QString& name, const QString& kind,
                       double newVal, double oldVal, const QString& newText,
                       qint64 wallMsOverride = -1);
    QString uiControlName(QWidget* w, int ordinal) const;
    // 【唯一的控件枚举口】快照和事件接线共用，保证同一个控件在两处叫同一个
    // 名字 —— 否则基线和增量 join 不起来，而那是这两份数据唯一的用法。
    QList<QPair<QWidget*, QString>> uiControls() const;
    std::string uiSnapshotJson() const;

    // ---- 块 23：UDP 输出闭环（v8 才真正接上）----
    // 【为什么这一块非有不可】链路最后一环在系统内部完全查不到：包根本没发、
    // 发了但对端没配、或者 M3DS/M3DQ 两个包的时间戳对不上导致下游插值出鬼。
    // 不记的话，这三类会被一路误判成解算问题，往上游白查一整圈 ——
    // 而"输出冻结"这个症状恰恰是三类都能造成的。
    //
    // 【只记摘要不记全量字节】包内容就是 qOut 和 quatWorld/Local，那两样已经
    // 在块 13 里了。这里记的是"发生了什么"：发没发、多大、序号、校验和。
    // 校验和跟块 13 重算出来的对不上，就说明打包这一步本身有问题 ——
    // 这是唯一能发现它的办法。
    //
    // 两个包由两个独立信号送来，先到的那个先填进 udpPend_，凑齐或换帧时落盘。
    // 【数据源是 UdpSender 的发包回执，不是这里重打一份包】
    // 重打的副本跟真正写进 socket 的字节是两份代码，迟早分叉 ——
    // 而那种不一致【只会在你正拿它排查问题的时候骗你】。
    // 回执里带的 crc 覆盖的是真正发出去的那串字节。
    void flushUdpOut();
    mocap::pcrec::UdpOutRec udpPend_{};
    bool   udpPendDirty_ = false;
    qint64 udpPendTs_ = -1;
    // MainWindow 告诉我们的 UdpSender 端点。【填哨兵而不是 0】——
    // 0 是一个合法端口，会被当成真读数。
    bool   udpEnabled_ = false;
    int    udpPort_ = -1;

public slots:
    // 接 UdpSender::handPacketSent。which: 0=M3DS 1=M3DQ
    void onUdpPacketSent(int which, int bytes, int err, quint64 seq,
                         quint32 crc, quint32 flags, qint64 tsNs);
    // MainWindow 在开关/目标变化时调。
    void setUdpEndpoint(bool enabled, int port);

private:
    QPushButton* recBtn_ = nullptr;
    QPushButton* recMarkBtn_ = nullptr;
    QComboBox* recProtoCombo_ = nullptr;
    // 录制详细度（精简/完整/全量）。见 .cpp 里那段 tooltip 的说明 ——
    // 默认"完整"，因为被就地覆写掉的中间级在系统里没有第二份，
    // 不在录制时留下来，事后就永远分不开是哪一级出的错。
    QComboBox* recDetailCombo_ = nullptr;
    QLineEdit* recNoteEdit_ = nullptr;
    QLabel* recStatusLabel_ = nullptr;
    QElapsedTimer recTimer_;
    QElapsedTimer recClockTimer_;
    QString recModelPath_;   // 录制头里留痕用
    bool recCalibWritten_ = false;
    // ---- v5 录制状态 ----
    // 【为什么帧间隔要自己算】refTsNs 的间隔就是这套系统真实的 dt，而它是
    // 滤波、限幅、速度外推三处的分母。dt 不可信时这三处算出来的东西全部
    // 不可信，但输出看起来完全正常，只是慢了、飘了 —— 人会先去怀疑参数、
    // 标定、模型，把时间花在错的地方。抖动比均值更要紧：平均 8.3ms 而抖动
    // 5ms 的流，跟稳定 8.3ms 的流，对滤波器是两回事。
    qint64 recLastFrameTsNs_ = -1;
    double recDtMean_ = 0.0;
    double recDtVar_ = 0.0;
    std::string buildRecordHeader() const;
    // 录制文件的目录：项目根/recordings。见 .cpp 里的说明。
    static QString recordingDir();
    QString recLastPath_;
    // 最近一帧的诊断快照。【录制头要用】录制是在 GUI 线程按下按钮的一刻拼头的，
    // 那一刻拿不到 worker 的内部状态；而诊断是每帧推过来的，留一份最新的即可。
    // 记录"实际生效值"比读控件可信：控件是用户想要什么，诊断是实际跑的是什么，
    // 两者可以不一致（参数下发有 dirty 标志、自标定会覆盖手性）。
    SkeletonAssocDiag lastDiagSnapshot_;
    class QPushButton* recOpenDirBtn_ = nullptr;
    class QPushButton* dorsumResetBtn_ = nullptr;
    // 高速模式(INT8)开关。找不到 INT8 模型时禁用 —— 见 addCamera 附近的说明。
    QCheckBox* fastModeChk_ = nullptr;
    void toggleRecording();
    void rebuildSkeletonTemplate();
    // 诊断文字限流——避免每帧setText触发整个参数面板重排(见
    // onSkeletonResultReady 里的说明)。
    QElapsedTimer skeletonDiagThrottle_;
    int skeletonDiagLastKind_ = -1;   // 状态类别(不可用/等冷启动/生效)，变了就立刻刷新不等限流
    bool skeletonOverlayActive_ = false;   // 叠加层当前是否有内容，避免每帧重复清空
    QElapsedTimer skeletonDispatchThrottle_;   // AI调用限流(约30Hz)，见tryCluster里的说明
    int skeletonDroppedFrames_ = 0;     // worker忙时被跳过的帧数(自上次清零起累计)
    int skeletonSubmittedFrames_ = 0;   // 同期提交给worker的帧数，配合上面这个算"丢帧率"
    double skeletonLastLatencyMs_ = -1.0;

    // 【改动】结果类型迁到 hm20 命名空间
    void onSkeletonResultReady(const hm20::SkeletonFrameResult& result, const SkeletonAssocDiag& diag);
};

} // namespace mocap
