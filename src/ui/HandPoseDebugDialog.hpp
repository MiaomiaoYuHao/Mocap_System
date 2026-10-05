#pragma once
// ---------------------------------------------------------------------------
// 手部位姿调试对话框——订阅 HandTrackingWorker 的信号，把数字变成看得懂的
// 东西。对应之前建议的验证步骤 3~6：
//   3. 冷启动有没有触发 —— 状态灯 + "已跟踪/未找到"文字
//   4. 静止稳不稳       —— 数值读数本身肉眼看抖动，另外画了一条最近
//                          wristPos 的轨迹，静止时应该是一个紧凑的小团，
//                          不是一条越飘越远的线
//   5. 移动方向对不对    —— 世界系俯视轨迹图(X-Y)，手往哪挪，轨迹就该
//                          往哪画，坐标轴标了正方向
//   6. 手指角度对不对    —— 一个简化火柴人骨架图(用真实 HandModel::handFK
//                          算 20 球局部坐标 + 当前 wristRot 转一下方向，
//                          wristPos 不参与——骨架图故意脱离世界坐标、
//                          只看"手长什么姿势"，不然手一移动骨架图跟着
//                          满屏跑，没法专心看手指弯没弯)
//
// 【这一版】加了"手指屈曲测试"面板，整合进这同一个窗口，不再单独开一个
// 对话框——道理很简单：这个窗口本来就是看"追踪对不对"的地方，测试也是
// 看"追踪对不对"，没道理分成两个窗口、两份重复的诊断信息。测试面板复用
// 这里已经在订阅的全部信号(handPoseReady/handNotFound/coldStartDiagnostics/
// radiusRefineStats)，新增订阅 candidatePointsReady(之前没人接的信号，
// 候选3D点/遮挡记忆数量)。
//
// 测试面板选中某根手指时，右侧骨架图会切到"单指聚焦"模式——不再画全手
// 20颗点，只放大画这一根手指自己的三节连杆(手腕原点→近节→中节→远节)，
// 而不是在一整只小手里眯着眼睛找这一根手指有没有弯对，见
// HandSkeletonWidget::setFocusFinger。
//
// 不引入任何绘图库，纯 QPainter，保持这个调试工具本身足够简单、不会
// 又变成一个需要被验证的东西。
// ---------------------------------------------------------------------------
#include <QDialog>
#include <QVector3D>
#include <QVector>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QPair>
#include <QElapsedTimer>
#include <QTimer>
#include <deque>
#include <array>

class QLabel;
class QWidget;
class QComboBox;
class QPushButton;
class QCheckBox;
class QTableWidget;
class QTextEdit;

namespace mocap {

// 前向声明，避免这个对话框头文件强制拉进 Qt 相机/标定那一整套依赖——
// 用哪个 worker 类型只在 .cpp 里 connect 时才真正需要包含它的头。
class HandTrackingWorker;

// 骨架图/轨迹图的绘制逻辑放在一个独立的小 QWidget 里，不是直接在 QDialog
// 上画——这样将来想把这块单独摘出来嵌到别的窗口(比如跟预览画面并排放)
// 时，不用拆对话框布局代码。
class HandSkeletonWidget : public QWidget {
    Q_OBJECT
public:
    explicit HandSkeletonWidget(QWidget* parent = nullptr);
    // 喂入这一帧的手腕旋转(行主序3x3) + 16维关节角。骨架图只关心"手的
    // 姿势"，故意不用 wristPos——见文件头注释。
    void setPose(const QVector<double>& wristRot9, const QVector<double>& jointAngles16);
    void clearPose();

    // 单指聚焦模式：-1=正常画整只手(默认)；0~4=只放大画这一根手指自己的
    // 三节连杆(0=拇指,1=食指,2=中指,3=无名指,4=小指，顺序跟已有的
    // fingerBase/fingerColors数组一致，不是随便定的)。用于手指屈曲测试
    // 面板——选中"现在测哪根手指"时，肉眼看这一根弯没弯，比在一整只
    // 缩小的手里找容易得多。
    void setFocusFinger(int fingerIdx);

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QVector<double> rot9_;
    QVector<double> angles_;
    bool hasPose_ = false;
    int focusFinger_ = -1;
};

// 世界系俯视轨迹图：画最近 N 个 wristPos 在 X-Y 平面的投影，用来看移动
// 方向、静止时的抖动范围。
class TrajectoryWidget : public QWidget {
    Q_OBJECT
public:
    explicit TrajectoryWidget(QWidget* parent = nullptr);
    void addPoint(const QVector3D& worldPos);
    void clearTrail();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    std::deque<QPointF> trailXY_;   // (x,y)，最近 kMaxTrailPoints 个
    static constexpr int kMaxTrailPoints = 300;
};

class HandPoseDebugDialog : public QDialog {
    Q_OBJECT
public:
    explicit HandPoseDebugDialog(HandTrackingWorker* worker, QWidget* parent = nullptr);

private slots:
    void onHandPoseReady(QVector3D wristPos, QVector<double> wristRot9,
                        QVector<double> jointAngles16, qint64 ts_ns);
    void onHandNotFound(qint64 ts_ns);
    void onStalenessTick();
    void onColdStartDiagnostics(int numCandidates3D, int numDistanceMatches,
                               double bestRmsMm, int failReasonCode, QString summary, qint64 ts_ns);
    // 两遍流程(radius-free粗算深度 -> 反推真实半径精修)的中间结果，
    // 对应 HandTrackingWorker::radiusRefineStatsReady——把console日志里
    // "[粗算候选点=N]"这些数字摆到界面上，不用盯着console看。
    void onRadiusRefineStats(int coarsePointCount, QVector<int> matchedPerCam,
                            QVector<int> fallbackPerCam, qint64 ts_ns);
    // 延迟精修流(handPoseSmoothedReady)的对照显示——不重画骨架图/轨迹图
    // (那两个是"当前姿态长什么样"的主视图，画两份反而容易看混)，只显示
    // "这一路跟实时那一路比，改了多少"这个数字，这才是这条流存在的意义：
    // 验证它到底有没有让结果更准，而不是又画一套一模一样的骨架。
    void onHandPoseSmoothedReady(QVector3D wristPos, QVector<double> wristRot9,
                                 QVector<double> jointAngles16, qint64 ts_ns);
    // 【新增】候选3D点(多视角三角化聚类后)——之前没有任何UI接过这个信号，
    // 只在标定向导内部用。这里拿它显示"候选点数/遮挡记忆(coasting)数量"，
    // 帮助判断追踪不稳时具体卡在哪个环节。
    void onCandidatePoints(QVector<QVector3D> worldPoints, QVector<int> ids, bool hasWristPose,
                          QVector<double> wristRot9, QVector3D wristPos, qint64 ts_ns,
                          QVector<int> missedFrames);

    // 【新增】手指屈曲测试面板的交互
    void onTestFingerChanged(int index);
    void onTestRecordToggled();
    void onTestAnalyzeClicked();
    void onTestExportClicked();

private:
    void updateNumericReadout(const QVector3D& pos, const QVector<double>& rot9,
                              const QVector<double>& angles);
    void refreshFpsLabel();
    void updateCandidateLabel();
    void updateTestStatusLabel();

    // ---- 手指屈曲测试：数据结构 + 分析逻辑 ----
    struct TestRow {
        qint64 tsNs = 0;
        QString label;
        bool tracked = true;
        std::array<double, 16> joints{};
        int coarsePointCount = -1;
        int candidateCount = -1;
        int coastingCount = -1;
    };
    struct TestSegmentResult {
        QString label; int segIdx = 0; int nFrames = 0;
        double trackedRatio = 0.0, targetRangeRad = 0.0, jitterRatio = 0.0;
        double avgCoarsePointCount = -1.0, avgCoastingRatio = -1.0;   // 诊断字段的这一段均值，供建议生成用，也直接展示
        QString status;
        QStringList issues;         // 判定到的问题(事实陈述："幅度不够""疑似串扰")
        QStringList suggestions;    // 【新增】结合诊断信号的针对性建议——算法不知道真实手的姿态，
                                    // 只能说"如果现在这几个诊断数字长这样，历史上最常见的原因是这个"，
                                    // 不是确诊，测不准的话别照单全收，当排查方向用。
    };
    QVector<QPair<QString, QVector<TestRow>>> splitTestSegments() const;
    TestSegmentResult analyzeTestSegment(const QString& label, const QVector<TestRow>& segRows, int segIdx) const;

    QLabel* statusLabel_ = nullptr;
    QLabel* posLabel_ = nullptr;
    QLabel* rotLabel_ = nullptr;
    QLabel* jointsLabel_ = nullptr;
    QLabel* fpsLabel_ = nullptr;
    QLabel* tsAgeLabel_ = nullptr;
    QLabel* coldStartLabel_ = nullptr;   // 冷启动诊断：候选点数/距离匹配组数/最佳残差/失败原因人话
    QLabel* radiusRefineLabel_ = nullptr;   // 两遍流程诊断：粗算候选点数 + 每台相机matched/fallback计数
    QLabel* candidateLabel_ = nullptr;      // 候选3D点/遮挡记忆诊断(新增)
    QLabel* smoothedLabel_ = nullptr;   // 延迟精修流对照：跟同一时刻的实时结果比，位置/关节角差了多少

    HandSkeletonWidget* skeleton_ = nullptr;
    TrajectoryWidget* trajectory_ = nullptr;

    QElapsedTimer sinceLastPose_;
    QElapsedTimer fpsWindowTimer_;
    QTimer* stalenessTimer_ = nullptr;
    int framesInWindow_ = 0;
    qint64 lastTsNs_ = -1;

    struct RealtimeSample { qint64 ts_ns; QVector3D pos; QVector<double> angles; };
    std::deque<RealtimeSample> realtimeHistory_;
    static constexpr int kMaxHistorySamples = 300;

    // ---- 手指屈曲测试：UI ----
    QComboBox* testFingerCombo_ = nullptr;
    QCheckBox* testMarkIdleChk_ = nullptr;
    QPushButton* testRecordBtn_ = nullptr;
    QPushButton* testAnalyzeBtn_ = nullptr;
    QPushButton* testExportBtn_ = nullptr;
    QLabel* testStatusLabel_ = nullptr;
    QTableWidget* testResultTable_ = nullptr;
    QTextEdit* testIssuesText_ = nullptr;

    bool testRecording_ = false;
    QString testCurrentLabel_ = QStringLiteral("idle");
    QVector<TestRow> testRows_;

    // 诊断"当前值"缓存，updateCandidateLabel()/onRadiusRefineStats渲染用，
    // 也顺带写进testRows_每一行，供离线分析用。
    int lastCoarsePointCount_ = -1;
    QVector<int> lastMatchedPerCam_, lastFallbackPerCam_;
    int lastCandidateCount_ = -1, lastCoastingCount_ = -1;
};

} // namespace mocap
