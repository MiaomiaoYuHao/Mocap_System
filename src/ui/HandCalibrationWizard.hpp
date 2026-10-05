#pragma once
// ---------------------------------------------------------------------------
// 手部标定向导——把 HandCalibration.hpp(手背模板) + HandSelfCalibration.hpp
// (手指结构自标定) 这两块数学，接成一个引导用户一步步完成标定的对话框。
// 数学本身跟这个文件里的一行代码都没有新写，全部复用已经验证过的模块，
// 这个文件只是"UI流程 + 数据关联(点选/最近邻延续)"的胶水。
//
// 两步：
//   步骤1(手背模板)：手静止，向导列出这一帧的无标签候选3D点，用户点选
//     哪5个是手背marker、哪1个大致在手指延伸方向(forward hint)，再选一个
//     手背朝向的大致世界系方向(dorsal hint，6选1)，调用
//     calibrateHandBackTemplate()，通过几何退化检查后允许保存。
//   步骤2(手指结构)：手背模板存好、追踪已经跑起来后，逐根手指标定：用户
//     先在候选点列表里点3下标出"这根手指当前的3颗marker"做一次性引导，
//     之后向导按"离上一帧最近"做逐帧延续关联(简化版最近邻，跟
//     TemporalTracker 同思路，但只服务这一根手指的3个点，不需要那么通用)，
//     用户活动手指几秒后点"停止并计算"，调用 calibrateFingerStructure/
//     calibrateThumbStructure。
//
// 【范围声明】步骤2的逐帧关联是简化实现：假设标定过程中只有正在标定的
// 这根手指在动、其它候选点(手背+别的手指)相对稳定，用最近邻延续足够可靠；
// 真实场景里如果标定时不小心带动了别的手指，关联可能跟错，向导会展示
// 实时残差/丢帧数，用户能感觉到不对劲(残差突然跳变)就重新开始采集，不是
// 完全的黑盒。
// ---------------------------------------------------------------------------
#include <QDialog>
#include <QWidget>
#include <QVector3D>
#include <QVector>
#include <QString>
#include <QFutureWatcher>
#include "hand/HandSelfCalibration.hpp"   // FingerCalibResult(存成员要完整类型，不能只前向声明)
#include "estimate/HandAutoCalibPipeline.hpp"   // HandAutoCalibOutput/FrameTrackedPoints(高级选项"全自动标定"用，同样要完整类型)
#include <array>
#include <vector>
#include <deque>
#include <optional>

class QLabel;
class QPushButton;
class QComboBox;
class QTableWidget;
class QTextEdit;
class QTabWidget;
class QMouseEvent;
class QPaintEvent;

namespace mocap {

class HandTrackingWorker;
class HandTemplateStore;

// ---------------------------------------------------------------------------
// 候选点散点可视化——手背模板标定这一步最大的可用性问题：表格里只有坐标
// 数字，人眼没法把"第3行"跟"我手指上那颗球"对上号。这个widget把
// latestPoints_ 投影成世界系俯视图(X-Y平面，mm)，每个点标上它在表格里的
// 行号，配色跟表格高亮完全一致(手背=蓝、方向提示=橙、未选=灰)，点哪个
// 圈效果等同于点表格对应行——双向直观，不用再靠数坐标猜。
// ---------------------------------------------------------------------------
class CandidatePointsScatterWidget : public QWidget {
    Q_OBJECT
public:
    explicit CandidatePointsScatterWidget(QWidget* parent = nullptr);
    // ids 跟 points 一一对应，是 HandTrackingWorker::candidatePointsReady
    // 带出来的跨帧稳定身份(不是数组下标)——backIds/forwardId 也是同一套
    // ID空间，不是"这一帧第几个"。圈上标的数字就是这个ID，同一颗物理球
    // 只要没连续丢帧太久，编号不会变，不会再出现"抖、编号一直闪烁"。
    // missedFrames 与 points 一一对应(可为空=全部按实测处理)：>0 的点画成
    // 暗色空心圈——那是遮挡记忆里的冻结位置，不是这一帧实测的，用户要能
    // 一眼分清(专业动捕软件对occluded marker的标准呈现方式)。
    void setPoints(const QVector<QVector3D>& points, const QVector<int>& ids,
                   const std::vector<int>& backIds, int forwardId,
                   const QVector<int>& missedFrames = {});

signals:
    void pointClicked(int id);   // 吐的是稳定ID，不是数组下标；点圈效果等同于点表格对应行

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;

private:
    QPointF toScreen(const QPointF& worldXY) const;   // 依赖当前包围盒缓存，只在paintEvent算好之后调用有效
    void recomputeTransform();

    QVector<QVector3D> points_;
    QVector<int> ids_;
    QVector<int> missedFrames_;
    std::vector<int> backIds_;
    int forwardId_ = -1;

    // paintEvent 里算好缓存，mousePressEvent 复用同一份变换做反向命中测试，
    // 保证"看到的圆点位置"和"能点中的位置"永远一致，不会各算一套。
    double scale_ = 1.0;
    QPointF centerWorld_{0,0};
    QPointF centerScreen_{0,0};

    // 迟滞式视图包围盒：以前每帧按当前点重算包围盒，任何一个点闪现/消失
    // (哪怕只有一帧)都会改变包围盒，进而改变缩放和中心，整幅图所有点在
    // 屏幕上集体跳一下——用户看到的"点闪烁抖动、图乱了"很大一部分是这个
    // 视图重排，不是点本身在动。改成：需要扩大时立刻扩(新点永远不会画到
    // 视野外)，收缩时每帧只向目标收一小步(单帧闪烁根本来不及影响视图)。
    bool viewBoxInit_ = false;
    double viewMinX_ = 0, viewMaxX_ = 0, viewMinY_ = 0, viewMaxY_ = 0;
};

// ---------------------------------------------------------------------------
// 步骤1：手背模板标定页
// ---------------------------------------------------------------------------
class BackTemplateCalibPage : public QDialog {
    Q_OBJECT
public:
    BackTemplateCalibPage(HandTrackingWorker* worker, HandTemplateStore* store, QWidget* parent = nullptr);

signals:
    void backTemplateCalibrated();   // 保存成功后通知外层——外层负责启用"跳过手指标定"按钮、提示可以去测试了

private slots:
    void onCandidatePoints(QVector<QVector3D> worldPoints, QVector<int> ids, bool hasWristPose,
                          QVector<double> wristRot9, QVector3D wristPos, qint64 ts_ns,
                          QVector<int> missedFrames);
    void onRowClicked(int row, int col);
    void onDorsalPresetChanged(int idx);
    void onCalibrateClicked();
    void onSaveClicked();

private:
    void refreshTable();
    void refreshSelectionLabel();
    void selectId(int id);      // 手背5点/方向提示点的实际选中逻辑，按稳定ID操作(不是数组下标)
    int idAt(int row) const;    // 表格行号 -> 这一帧的稳定ID；越界返回-1
    int rowOfId(int id) const;  // 稳定ID -> 这一帧表格里在第几行；这一帧没看到该ID返回-1

    HandTrackingWorker* worker_;
    HandTemplateStore* store_;

    QLabel* instructionLabel_ = nullptr;
    QTableWidget* pointTable_ = nullptr;
    CandidatePointsScatterWidget* scatter_ = nullptr;
    QComboBox* dorsalCombo_ = nullptr;
    QLabel* selectionLabel_ = nullptr;
    QLabel* resultLabel_ = nullptr;
    QPushButton* calibrateBtn_ = nullptr;
    QPushButton* saveBtn_ = nullptr;

    QVector<QVector3D> latestPoints_;
    QVector<int> latestIds_;            // 跟 latestPoints_ 一一对应的跨帧稳定ID
    QVector<int> latestMissed_;         // 一一对应的遮挡记忆帧数(0=实测,>0=coasting冻结位置)
    std::vector<int> backIndices_;      // 用户点选的5个手背点——存的是稳定ID，不是数组下标(尽管变量名沿用旧名字)
    int forwardIndex_ = -1;             // 用户点选的forward hint点——同样存的是稳定ID

    std::array<double,3> pendingTemplateLocal_[5]{};
    bool haveCalibratedResult_ = false;
};

// ---------------------------------------------------------------------------
// 步骤2：单根手指结构自标定页(拇指/四指通用界面，内部按是否拇指分派)
// ---------------------------------------------------------------------------
class FingerCalibPage : public QDialog {
    Q_OBJECT
public:
    FingerCalibPage(HandTrackingWorker* worker, HandTemplateStore* store, QWidget* parent = nullptr);

private slots:
    void onCandidatePoints(QVector<QVector3D> worldPoints, QVector<int> ids, bool hasWristPose,
                          QVector<double> wristRot9, QVector3D wristPos, qint64 ts_ns,
                          QVector<int> missedFrames);
    void onFingerComboChanged(int idx);
    void onTablePointClicked(int row, int col);
    void onStaticCalibClicked();
    void onStartCaptureClicked();
    void onStopAndComputeClicked();
    void onComputeFinished();   // QFutureWatcher::finished——后台计算结束后在这里把结果搬回GUI线程更新界面
    void onSaveClicked();

private:
    void refreshTable(const QVector<QVector3D>& worldPoints, const QVector<int>& ids,
                      const QVector<int>& missedFrames = {});
    std::array<double,3> toWristLocal(const QVector3D& worldPt,
                                      const QVector<double>& wristRot9, const QVector3D& wristPos) const;
    // 已知手背模板点(来自store_，腕部局部系)排除掉——候选点列表里混着手背
    // 5点和手指候选点，不排除的话用户没法一眼分清"这几个是手背、跟这根
    // 手指没关系"。tolMm 是判定"这个候选点其实就是某个已知手背marker"的
    // 距离容差。
    bool isKnownBackMarker(const std::array<double,3>& localPt, double tolMm = 15.0) const;

    HandTrackingWorker* worker_;
    HandTemplateStore* store_;

    QComboBox* fingerCombo_ = nullptr;
    QLabel* instructionLabel_ = nullptr;
    QTableWidget* pointTable_ = nullptr;
    CandidatePointsScatterWidget* scatter_ = nullptr;
    QLabel* trackingStatusLabel_ = nullptr;
    QLabel* bootstrapHint_[3] = {nullptr,nullptr,nullptr};
    QPushButton* staticBtn_ = nullptr;
    QPushButton* startBtn_ = nullptr;
    QPushButton* stopBtn_ = nullptr;
    QPushButton* saveBtn_ = nullptr;
    QLabel* captureStatusLabel_ = nullptr;
    QLabel* resultLabel_ = nullptr;
    QTextEdit* logView_ = nullptr;

    int bootstrapClickCount_ = 0;
    std::array<double,3> lastKnownLocal_[3]{};
    bool haveLastKnown_ = false;

    // ---- 静态姿势标定(见 calibrateFingerFromStaticPose) ----
    void finishStaticCalibration();
    static constexpr int kStaticFramesGoal = 45;   // 约0.4s@120fps的平均窗口
    bool staticCapturing_ = false;
    int staticAccumCount_ = 0;
    std::array<std::array<double,3>,3> staticAccum_{};
    FingerParam staticParam_{};          // 最近一次静态标定结果，做动态精修初值
    bool haveStaticParam_ = false;
    int staticParamFingerIdx_ = -1;      // 静态结果属于哪根手指(切手指后不能拿去当别根的初值)
    std::vector<int> bootstrapSelectedIds_;   // 跟lastKnownLocal_平行——记录已选3颗点各自的跨帧稳定ID，供表格/散点图高亮用
    QVector<int> displayedIds_;               // 跟当前pointTable_/scatter_显示的这批点一一对应的稳定ID(已排除手背点)

    bool capturing_ = false;
    std::vector<std::array<std::array<double,3>,3>> capturedPositions_;
    std::vector<std::array<bool,3>> capturedVisible_;
    int missedFrameStreak_ = 0;

    bool haveComputedResult_ = false;
    FingerCalibResult pendingFingerResult_;   // 四指(3自由度)分支的标定结果暂存
    FingerCalibResult pendingThumbResult_;    // 拇指(4自由度)分支的标定结果暂存(结构体形状相同，分开存避免混淆)

    // 后台异步计算——calibrateFingerStructure/calibrateThumbStructure 是
    // 秒级到分钟级的重计算(多起点全局重启的交替优化，223帧数据量下实测
    // 会让GUI线程卡到系统判定"未响应")，必须丢到后台线程跑，不能直接在
    // 按钮点击槽函数里同步调用。
    QFutureWatcher<FingerCalibResult>* computeWatcher_ = nullptr;
    int computingFingerIdx_ = -1;   // 后台计算正在跑的是哪根手指(0=拇指)，onComputeFinished要用这个决定结果存进pendingFingerResult_还是pendingThumbResult_
};

// ---------------------------------------------------------------------------
// 高级选项：全自动标定（实验性）—— 只需要自然挥手+活动手指几十秒到几
// 分钟，一次性同时标出手背模板+全部5指结构，不需要像①②那样手动点选
// 任何一颗点、不需要逐根手指分开采集。底层是 HandAutoCalibPipeline.hpp
// 那条"晃手聚类找手背刚体 -> 逐帧腕部Kabsch -> 手指候选点分组 -> 轴系
// 对齐 -> 逐链运动学拟合"的全自动链路——这条链路本身已经用合成数据充分
// 测过，但从没在真实标定流程里跑过，没有向导①②那样经过真实使用验证，
// 收敛质量(尤其手指分组这一步，靠运动相关性自动分组，遮挡/多指同时动
// 会干扰)不像①②那样有把握，所以放在"高级选项"而不是默认流程——想更
// 省事、愿意接受"可能一次不成功需要重采集"，就用这个；想要更可控、
// 每一步都能看见发生了什么，用①②那条引导式流程。
//
// 【已知的真实局限，不回避】拇指标定结果里的 baseRotZ(拇指CMC根部旋转
// 角)算出来会被丢弃——当前 hand/HandModel.hpp::thumbFK 这个角度是写死
// 45°的，没有开放成可调参数，自动标定即使解出一个更准的值也用不上，
// 除非以后把 HandModel.hpp 也改成支持这个参数。这不是这个页面的bug，
// 是两套代码之间一个明确存在、尚未打通的接口缺口，页面会在结果里如实
// 告诉你这一点，不会假装标准了。
// ---------------------------------------------------------------------------
class AutoCalibPage : public QDialog {
    Q_OBJECT
public:
    AutoCalibPage(HandTrackingWorker* worker, HandTemplateStore* store, QWidget* parent = nullptr);

private slots:
    void onCandidatePoints(QVector<QVector3D> worldPoints, QVector<int> ids, bool hasWristPose,
                          QVector<double> wristRot9, QVector3D wristPos, qint64 ts_ns);
    void onStartCaptureClicked();
    void onStopClicked();
    void onRunCalibClicked();
    void onCalibFinished();
    void onSaveClicked();

private:
    HandTrackingWorker* worker_;
    HandTemplateStore* store_;

    QLabel* instructionLabel_ = nullptr;
    QLabel* captureStatusLabel_ = nullptr;
    QPushButton* startBtn_ = nullptr;
    QPushButton* stopBtn_ = nullptr;
    QPushButton* runBtn_ = nullptr;
    QPushButton* saveBtn_ = nullptr;
    QTextEdit* resultView_ = nullptr;

    bool capturing_ = false;
    std::vector<FrameTrackedPoints> capturedFrames_;

    bool haveResult_ = false;
    HandAutoCalibOutput pendingResult_;

    QFutureWatcher<HandAutoCalibOutput>* computeWatcher_ = nullptr;
};

// ---------------------------------------------------------------------------
// 顶层向导：把两页装进 tab，步骤2默认禁用直到步骤1保存成功。
// ---------------------------------------------------------------------------
class HandCalibrationWizard : public QDialog {
    Q_OBJECT
public:
    HandCalibrationWizard(HandTrackingWorker* worker, HandTemplateStore* store, QWidget* parent = nullptr);
    ~HandCalibrationWizard() override;

private:
    HandTrackingWorker* worker_;
    HandTemplateStore* store_;
    QTabWidget* tabs_ = nullptr;
    BackTemplateCalibPage* backPage_ = nullptr;
    FingerCalibPage* fingerPage_ = nullptr;
    AutoCalibPage* autoPage_ = nullptr;
    QLabel* backStatusLabel_ = nullptr;     // "手背模板：未保存/已保存"，不管切到哪个tab都看得见
    QPushButton* skipFingerBtn_ = nullptr;  // "跳过手指标定，关闭向导去测试"——手背存了才能点，两个tab切来切去都在这
};

} // namespace mocap
