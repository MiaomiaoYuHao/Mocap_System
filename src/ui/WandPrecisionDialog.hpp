#pragma once
// ---------------------------------------------------------------------------
// 标定杆精度验证窗口——拿一根量好的固定长度杆子(两端各一颗反光球，真值是
// 两颗球球心之间的距离)在捕捉空间里挥动，实时量出系统三角化出的距离，
// 跟真值对比，滚动统计出RMS误差/标准差/偏置这几个"这套系统到底多准"的
// 真实数字，不再是"感觉更稳了"这种主观判断。
//
// 【为什么是新开一个窗口，不是改 TriangulationDebugDialog】那个窗口是
// 单点设计(Triangulator::point3DReady 一次回调一个点)，这里需要同时
// 稳定跟踪两个点、且不能认错谁是谁(跨帧维持身份)——用的是
// reconstruct/MultiViewCluster.hpp::clusterMultiView(多点关联) +
// reconstruct/TemporalTracker.hpp(跨帧稳定ID)，这两个都是这个项目里
// 已经独立测过的模块，跟 Triangulator 是不同的机制，硬塞进单点设计的
// 窗口里风险比新开一个更大。
//
// 【定位模式，跟 HandTrackingWorker 用的是同一套，不是另起一套】三选一：
// 质心法(不需要轮廓，最简单)/圆拟合法(需要"已知半径"先验，用跟
// HandTrackingWorker 一样的两遍流程：先用原始质心做一次radius-free粗略
// 三角化拿深度，再给每个blob反推期望像素半径喂给3a~3d精修)/融合。选圆
// 拟合或融合时要填球的物理半径(mm)。想知道"当前实际部署的检测模式在
// 真实场景下精度到底多少"，就把这里的模式设成跟 HandTrackingWorker 当前
// 用的一致，测出来的数字才有代表性。
//
// 使用流程：勾选>=2台已标定相机 -> 选定位模式(圆拟合/融合的话填球半径)
// -> 填入杆子真实长度(两球球心距) -> 让捕捉空间里只有这根杆子(这一版
// 要求场景里稳定只有2个候选点，多了会被判定"场景不干净"直接跳过这一帧)
// -> 点"开始统计" -> 到处挥动杆子，尽量覆盖整个工作距离范围和不同姿态
// -> 看统计数字。
// ---------------------------------------------------------------------------
#include "calib/WandPrecisionStats.hpp"
#include "calib/Calibration.hpp"             // CameraIntrinsics
#include "detect/BlobObservationAdapter.hpp"  // BlobLocalizationMode/blobToObservations，跟HandTrackingWorker共用同一套定位模式
#include <QDialog>
#include <QVector>
#include <QSet>
#include <QHash>
#include <QTimer>
#include <array>
#include <optional>

class QLabel;
class QListWidget;
class QListWidgetItem;
class QDoubleSpinBox;
class QComboBox;
class QPushButton;
class QTextEdit;

namespace mocap {

class CameraManager;
class CalibrationStore;
class ICamera;
struct Blob;
class TemporalTracker;   // 前向声明——用裸指针+.cpp里完整类型定义，避免这个头文件拉进整条 reconstruct 依赖链

class WandPrecisionDialog : public QDialog {
    Q_OBJECT
public:
    // wasDetectOn：同 TriangulationDebugDialog 的约定——打开前检测本来
    // 是不是已经开着，关闭时恢复，不留副作用。
    WandPrecisionDialog(CameraManager* mgr, CalibrationStore* store, bool wasDetectOn, QWidget* parent = nullptr);
    ~WandPrecisionDialog() override;

private slots:
    void onCamerasChanged();
    void onBlobDetailsReady(quint32 camId, const QVector<Blob>& blobs, qint64 ts_ns);
    void onBatchTimerFire();
    void onStartStopClicked();
    void onResetClicked();
    void onTrueLengthChanged(double mm);
    void onModeChanged(int idx);
    void onPhysicalRadiusChanged(double mm);

private:
    void rebuildCameraSetup();
    void processBatch();
    void refreshStatsDisplay();

    CameraManager* mgr_;
    CalibrationStore* store_;
    bool wasDetectOn_ = true;
    QSet<ICamera*> weTurnedOnDetect_;    // 本窗口帮忙打开"检测"的那几台，关闭时要关回去
    QSet<ICamera*> weTurnedOnContour_;   // 本窗口帮忙打开"轮廓采集"的那几台，关闭时要关回去(不能动了别的消费者可能也需要的开关)

    QListWidget* camList_ = nullptr;
    QComboBox* modeCombo_ = nullptr;
    QDoubleSpinBox* radiusSpin_ = nullptr;
    QLabel* radiusLabel_ = nullptr;
    QDoubleSpinBox* trueLengthSpin_ = nullptr;
    QPushButton* startStopBtn_ = nullptr;
    QPushButton* resetBtn_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QTextEdit* statsView_ = nullptr;

    bool running_ = false;

    // 参与验证的相机——下标即内部相机序号，跟 Rs_/ts_/相机ID一一对应。
    std::vector<quint32> camIds_;
    std::vector<std::array<double,9>> camR_;
    std::vector<std::array<double,3>> camT_;
    std::vector<CameraIntrinsics> camIntr_;   // undistortNormalize需要完整的畸变系数，不能简化成4个数

    // 这一批(batch window)内每台相机最新的原始Blob(含轮廓，圆拟合/融合
    // 模式需要)。
    QHash<quint32, QVector<Blob>> latestBlobs_;
    QTimer* batchTimer_ = nullptr;
    static constexpr int kBatchWindowMs = 4;

    TemporalTracker* tracker_ = nullptr;   // 裸指针+前向声明，完整类型只在.cpp里可见；构造函数里new，析构函数里delete

    std::optional<WandPrecisionAccumulator> accumulator_;
    double trueLengthMm_ = 200.0;

    BlobLocalizationMode localizationMode_ = BlobLocalizationMode::CentroidOnly;
    double physicalRadiusMm_ = 4.5;               // 圆拟合/融合模式需要——球的物理半径(mm)
    double fallbackWorkingDistanceMm_ = 300.0;    // 深度反推半径这一步，粗算完全没匹配上任何候选点时的兜底假设(极少用到)

    qint64 frameCounter_ = 0;   // 没有真实时间戳来源时(纯本地批处理)，用递增计数当ts_ns喂给统计模块，只用来定位"第几帧"，不是真实时间
};

} // namespace mocap
