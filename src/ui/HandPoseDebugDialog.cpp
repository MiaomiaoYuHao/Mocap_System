#include "ui/HandPoseDebugDialog.hpp"
#include "ui/Theme.hpp"
#include "ui/HandTrackingParamsPanel.hpp"
#include "estimate/HandTrackingWorker.hpp"
#include "hand/HandModel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QSplitter>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QCheckBox>
#include <QTableWidget>
#include <QHeaderView>
#include <QAbstractItemView>
#include <QTextEdit>
#include <QPainter>
#include <QPen>
#include <QFont>
#include <QColor>
#include <QScrollArea>
#include <QShortcut>
#include <QKeySequence>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QDateTime>
#include <cmath>
#include <limits>
#include <algorithm>

namespace mocap {

// ---------------------------------------------------------------------------
// 手指屈曲测试的判定参数 + 关节列布局——跟 tools/analyze_finger_flexion_test.py
// (如果你还留着离线脚本)保持同一套数字，两边改一处务必同步改另一处。
// ---------------------------------------------------------------------------
namespace {
constexpr double kMinFlexRangeRad = 0.25;
constexpr double kCrosstalkRatioWarn = 0.4;
constexpr double kCrosstalkRatioFail = 0.7;
constexpr double kJitterSignChangeRatioWarn = 0.35;
constexpr double kJitterDeadbandRad = 0.01;
constexpr double kMinTrackedRatio = 0.8;

struct FingerCols { int focusIdx; int primary; int secondary; QString nameCn; };
// focusIdx 对应 HandSkeletonWidget::setFocusFinger 的 0~4，顺序=拇食中无小，
// 跟骨架图里 fingerBase/fingerColors 数组的既有顺序一致，不是另起一套。
const QHash<QString, FingerCols>& fingerColsMap() {
    static const QHash<QString, FingerCols> m = {
        {"thumb",  {0, 3, 2,  QStringLiteral("拇指")}},
        {"index",  {1, 6, 4,  QStringLiteral("食指")}},
        {"middle", {2, 9, 7,  QStringLiteral("中指")}},
        {"ring",   {3, 12, 10, QStringLiteral("无名指")}},
        {"pinky",  {4, 15, 13, QStringLiteral("小指")}},
    };
    return m;
}
const QHash<QString, QVector<int>>& fingerAllFlexCols() {
    static const QHash<QString, QVector<int>> m = {
        {"thumb", {2, 3}}, {"index", {4, 6}}, {"middle", {7, 9}}, {"ring", {10, 12}}, {"pinky", {13, 15}},
    };
    return m;
}
} // namespace

// ---------------------------------------------------------------------------
// HandSkeletonWidget
// ---------------------------------------------------------------------------

HandSkeletonWidget::HandSkeletonWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 320);
}

void HandSkeletonWidget::setPose(const QVector<double>& wristRot9, const QVector<double>& jointAngles16) {
    rot9_ = wristRot9;
    angles_ = jointAngles16;
    hasPose_ = (rot9_.size() == 9 && angles_.size() == 16);
    update();
}

void HandSkeletonWidget::clearPose() {
    hasPose_ = false;
    update();
}

void HandSkeletonWidget::setFocusFinger(int fingerIdx) {
    if (focusFinger_ == fingerIdx) return;
    focusFinger_ = fingerIdx;
    update();
}

namespace {
QPointF obliqueProject(double x, double y, double z) {
    const double sx = x + y * 0.4;
    const double sy = -(z + y * 0.25);
    return QPointF(sx, sy);
}

QPointF rotateAndProject(const double R[9], const std::array<double,3>& p) {
    const double x = R[0]*p[0]+R[1]*p[1]+R[2]*p[2];
    const double y = R[3]*p[0]+R[4]*p[1]+R[5]*p[2];
    const double z = R[6]*p[0]+R[7]*p[1]+R[8]*p[2];
    return obliqueProject(x, y, z);
}

const char* jointShortName(int idx) {
    static const char* names[16] = {
        "CMC屈","CMC展","MCP屈","IP屈",
        "MCP屈","MCP展","PIP屈",
        "MCP屈","MCP展","PIP屈",
        "MCP屈","MCP展","PIP屈",
        "MCP屈","MCP展","PIP屈",
    };
    return (idx >= 0 && idx < 16) ? names[idx] : "?";
}
} // namespace

void HandSkeletonWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(0x14, 0x14, 0x14));

    if (!hasPose_) {
        painter.setPen(QColor(150, 150, 150));
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("(未跟踪)"));
        return;
    }

    double R[9];
    for (int i = 0; i < 9; ++i) R[i] = rot9_[i];

    std::array<double,16> q{};
    for (int i = 0; i < 16; ++i) q[size_t(i)] = angles_[i];

    std::array<std::array<double,3>,20> markers{};
    handFK(q, markers);

    const QPointF originPt = rotateAndProject(R, {0,0,0});

    std::array<QPointF,20> proj{};
    for (int i = 0; i < 20; ++i) proj[size_t(i)] = rotateAndProject(R, markers[size_t(i)]);

    const int fingerBase[5] = {5, 8, 11, 14, 17};
    const QColor fingerColors[5] = {
        QColor(230,180,90), QColor(120,200,120), QColor(120,180,230),
        QColor(200,120,200), QColor(220,120,120)
    };
    const int primaryJoint[5] = {3, 6, 9, 12, 15};
    const int secondaryJoint[5] = {2, 4, 7, 10, 13};

    // 单指聚焦模式：只画这一根手指自己的三节连杆，自动缩放到填满整个
    // 画布——正常模式下一整只缩小的手里，单根手指弯没弯很难看清楚。
    if (focusFinger_ >= 0 && focusFinger_ < 5) {
        const int b = fingerBase[focusFinger_];
        const QColor color = fingerColors[focusFinger_];

        double minX = originPt.x(), maxX = originPt.x(), minY = originPt.y(), maxY = originPt.y();
        for (int k = 0; k < 3; ++k) {
            const auto& p = proj[size_t(b+k)];
            minX = std::min(minX, p.x()); maxX = std::max(maxX, p.x());
            minY = std::min(minY, p.y()); maxY = std::max(maxY, p.y());
        }
        const double spanX = std::max(1.0, maxX - minX);
        const double spanY = std::max(1.0, maxY - minY);
        const double margin = 40.0;
        const double scale = std::min((width()-2*margin)/spanX, (height()-2*margin)/spanY);
        const QPointF center((minX+maxX)*0.5, (minY+maxY)*0.5);
        const QPointF widgetCenter(width()*0.5, height()*0.5);
        auto toScreen = [&](const QPointF& p) {
            return widgetCenter + QPointF((p.x()-center.x())*scale, (p.y()-center.y())*scale);
        };

        painter.setPen(QPen(color, 4.0));
        painter.drawLine(toScreen(originPt), toScreen(proj[size_t(b)]));
        painter.drawLine(toScreen(proj[size_t(b)]), toScreen(proj[size_t(b+1)]));
        painter.drawLine(toScreen(proj[size_t(b+1)]), toScreen(proj[size_t(b+2)]));

        painter.setBrush(color);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(toScreen(originPt), 5, 5);
        for (int k = 0; k < 3; ++k) painter.drawEllipse(toScreen(proj[size_t(b+k)]), 6, 6);

        painter.setPen(QColor(230,230,230));
        QFont bigFont = painter.font(); bigFont.setPointSize(13); bigFont.setBold(true);
        QFont smallFont = painter.font(); smallFont.setPointSize(9);

        painter.setFont(bigFont);
        const double primaryDeg = angles_[primaryJoint[focusFinger_]] * 180.0 / M_PI;
        painter.drawText(QPointF(12, 26), QStringLiteral("%1(主) = %2°")
            .arg(jointShortName(primaryJoint[focusFinger_])).arg(primaryDeg, 0, 'f', 1));

        painter.setFont(smallFont);
        const double secondaryDeg = angles_[secondaryJoint[focusFinger_]] * 180.0 / M_PI;
        painter.drawText(QPointF(12, 46), QStringLiteral("%1 = %2°")
            .arg(jointShortName(secondaryJoint[focusFinger_])).arg(secondaryDeg, 0, 'f', 1));

        return;
    }

    // 正常模式：画整只手（原有逻辑，不变）。
    double minX = originPt.x(), maxX = originPt.x(), minY = originPt.y(), maxY = originPt.y();
    for (const auto& p : proj) {
        minX = std::min(minX, p.x()); maxX = std::max(maxX, p.x());
        minY = std::min(minY, p.y()); maxY = std::max(maxY, p.y());
    }
    const double spanX = std::max(1.0, maxX - minX);
    const double spanY = std::max(1.0, maxY - minY);
    const double margin = 24.0;
    const double scale = std::min((width() - 2*margin) / spanX, (height() - 2*margin) / spanY);
    const QPointF center((minX+maxX)*0.5, (minY+maxY)*0.5);
    const QPointF widgetCenter(width()*0.5, height()*0.5);

    auto toScreen = [&](const QPointF& p) {
        return widgetCenter + QPointF((p.x()-center.x())*scale, (p.y()-center.y())*scale);
    };

    painter.setPen(QPen(QColor(90, 140, 200), 1.5));
    const int backLoop[6] = {0,1,3,4,2,0};
    for (int i = 0; i < 5; ++i)
        painter.drawLine(toScreen(proj[size_t(backLoop[i])]), toScreen(proj[size_t(backLoop[i+1])]));

    for (int f = 0; f < 5; ++f) {
        painter.setPen(QPen(fingerColors[f], 2.0));
        const int b = fingerBase[f];
        painter.drawLine(toScreen(originPt), toScreen(proj[size_t(b)]));
        painter.drawLine(toScreen(proj[size_t(b)]), toScreen(proj[size_t(b+1)]));
        painter.drawLine(toScreen(proj[size_t(b+1)]), toScreen(proj[size_t(b+2)]));

        painter.setBrush(fingerColors[f]);
        painter.setPen(Qt::NoPen);
        for (int k = 0; k < 3; ++k) painter.drawEllipse(toScreen(proj[size_t(b+k)]), 3, 3);
    }

    painter.setBrush(QColor(90,140,200));
    painter.setPen(Qt::NoPen);
    for (int i = 0; i < 5; ++i) painter.drawEllipse(toScreen(proj[size_t(i)]), 3, 3);
    painter.setBrush(Qt::white);
    painter.drawEllipse(toScreen(originPt), 4, 4);
}

// ---------------------------------------------------------------------------
// TrajectoryWidget（不变）
// ---------------------------------------------------------------------------

TrajectoryWidget::TrajectoryWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 240);
}

void TrajectoryWidget::addPoint(const QVector3D& worldPos) {
    trailXY_.push_back(QPointF(worldPos.x(), worldPos.y()));
    while (int(trailXY_.size()) > kMaxTrailPoints) trailXY_.pop_front();
    update();
}

void TrajectoryWidget::clearTrail() {
    trailXY_.clear();
    update();
}

void TrajectoryWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(0x14, 0x14, 0x14));

    painter.setPen(QColor(180, 180, 180));
    painter.drawText(8, 16, QStringLiteral("俯视 X-Y (mm)，用于核对移动方向"));

    if (trailXY_.size() < 2) {
        painter.setPen(QColor(120,120,120));
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("(等待轨迹数据)"));
        return;
    }

    double minX = trailXY_[0].x(), maxX = minX, minY = trailXY_[0].y(), maxY = minY;
    for (const auto& p : trailXY_) {
        minX = std::min(minX, p.x()); maxX = std::max(maxX, p.x());
        minY = std::min(minY, p.y()); maxY = std::max(maxY, p.y());
    }
    const double minSpanMm = 20.0;
    const double spanX = std::max(minSpanMm, maxX - minX);
    const double spanY = std::max(minSpanMm, maxY - minY);
    const double margin = 30.0;
    const double scale = std::min((width()-2*margin)/spanX, (height()-2*margin)/spanY);
    const QPointF centerWorld((minX+maxX)*0.5, (minY+maxY)*0.5);
    const QPointF centerScreen(width()*0.5, height()*0.5 + 8);

    auto toScreen = [&](const QPointF& p) {
        return centerScreen + QPointF((p.x()-centerWorld.x())*scale, -(p.y()-centerWorld.y())*scale);
    };

    painter.setPen(QPen(QColor(90,90,90), 1, Qt::DashLine));
    painter.drawLine(QPointF(margin, centerScreen.y()), QPointF(width()-margin, centerScreen.y()));
    painter.drawLine(QPointF(centerScreen.x(), margin), QPointF(centerScreen.x(), height()-margin));
    painter.setPen(QColor(140,140,140));
    painter.drawText(QPointF(width()-margin-14, centerScreen.y()-6), QStringLiteral("+X"));
    painter.drawText(QPointF(centerScreen.x()+6, margin+12), QStringLiteral("+Y"));

    for (size_t i = 1; i < trailXY_.size(); ++i) {
        const double t = double(i) / double(trailXY_.size());
        const QColor c(int(60+150*t), int(160+80*t), 220);
        painter.setPen(QPen(c, 2.0));
        painter.drawLine(toScreen(trailXY_[i-1]), toScreen(trailXY_[i]));
    }
    painter.setBrush(QColor(255,90,90));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(toScreen(trailXY_.back()), 4, 4);
}

// ---------------------------------------------------------------------------
// HandPoseDebugDialog
// ---------------------------------------------------------------------------

HandPoseDebugDialog::HandPoseDebugDialog(HandTrackingWorker* worker, QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(QStringLiteral("手部位姿调试 + 手指屈曲测试"));
    resize(980, 820);

    statusLabel_ = new QLabel(QStringLiteral("状态：未跟踪"), this);
    QFont statusFont = statusLabel_->font(); statusFont.setBold(true); statusFont.setPointSize(11);
    statusLabel_->setFont(statusFont);

    posLabel_ = new QLabel(QStringLiteral("位置：--"), this);
    rotLabel_ = new QLabel(QStringLiteral("旋转矩阵：--"), this);
    rotLabel_->setFont(QFont(QStringLiteral("Monospace")));
    jointsLabel_ = new QLabel(QStringLiteral("关节角：--"), this);
    jointsLabel_->setWordWrap(true);
    jointsLabel_->setFont(QFont(QStringLiteral("Monospace")));
    fpsLabel_ = new QLabel(QStringLiteral("帧率：--"), this);
    tsAgeLabel_ = new QLabel(QStringLiteral("距上次更新：--"), this);
    coldStartLabel_ = new QLabel(QStringLiteral("冷启动诊断：--"), this);
    coldStartLabel_->setWordWrap(true);
    coldStartLabel_->setStyleSheet(QStringLiteral("QLabel{padding:4px;}") + theme::monoCss(12.0));

    radiusRefineLabel_ = new QLabel(QStringLiteral("两遍流程诊断：--"), this);
    radiusRefineLabel_->setWordWrap(true);
    radiusRefineLabel_->setStyleSheet(QStringLiteral("QLabel{padding:4px;}") + theme::monoCss(12.0));

    candidateLabel_ = new QLabel(QStringLiteral("候选3D点诊断：--"), this);
    candidateLabel_->setWordWrap(true);
    candidateLabel_->setStyleSheet(QStringLiteral("QLabel{padding:4px;}") + theme::monoCss(12.0));

    smoothedLabel_ = new QLabel(QStringLiteral(
        "延迟精修流：未启用(去左侧参数面板底部\"延迟精修流\"分组勾选启用)"), this);
    smoothedLabel_->setWordWrap(true);
    smoothedLabel_->setStyleSheet(QStringLiteral("QLabel{padding:4px;}") + theme::monoCss(12.0));

    skeleton_ = new HandSkeletonWidget(this);
    trajectory_ = new TrajectoryWidget(this);

    auto* leftCol = new QVBoxLayout();
    leftCol->addWidget(statusLabel_);
    leftCol->addWidget(posLabel_);
    leftCol->addWidget(fpsLabel_);
    leftCol->addWidget(tsAgeLabel_);
    leftCol->addWidget(coldStartLabel_);
    leftCol->addWidget(radiusRefineLabel_);
    leftCol->addWidget(candidateLabel_);
    leftCol->addWidget(smoothedLabel_);
    leftCol->addWidget(rotLabel_);
    leftCol->addWidget(jointsLabel_);

    auto* paramsScroll = new QScrollArea(this);
    paramsScroll->setWidgetResizable(true);
    paramsScroll->setFrameShape(QFrame::NoFrame);
    paramsScroll->setWidget(new HandTrackingParamsPanel(worker, paramsScroll));
    leftCol->addWidget(paramsScroll, 1);

    auto* rightCol = new QVBoxLayout();
    rightCol->addWidget(skeleton_, 2);
    rightCol->addWidget(trajectory_, 1);

    auto* topRow = new QHBoxLayout();
    auto* leftWidget = new QWidget(this);
    leftWidget->setLayout(leftCol);
    leftWidget->setMaximumWidth(340);
    topRow->addWidget(leftWidget);
    topRow->addLayout(rightCol, 1);

    // ---- 手指屈曲测试面板 ----
    auto* testGroup = new QGroupBox(QStringLiteral("手指屈曲测试（外部标签自检）"), this);
    auto* testLayout = new QVBoxLayout(testGroup);

    auto* testInstr = new QLabel(QStringLiteral(
        "用法：下拉框选待测手指(右侧骨架图会切到该手指的放大三节连杆视图)→"
        "\u201c开始录制\u201d→从伸直到弯曲慢慢弯这根手指(其它手指尽量不动，可反复弯伸)→"
        "切下拉框测下一根(同一次录制可连续测完5根)→\u201c停止\u201d→\u201c分析\u201d当场出结果。"),
        testGroup);
    testInstr->setWordWrap(true);
    testLayout->addWidget(testInstr);

    auto* testRow1 = new QHBoxLayout();
    testRow1->addWidget(new QLabel(QStringLiteral("当前测试手指："), testGroup));
    testFingerCombo_ = new QComboBox(testGroup);
    testFingerCombo_->addItem(QStringLiteral("待机（骨架图恢复整手视图）"), QStringLiteral("idle"));
    testFingerCombo_->addItem(QStringLiteral("拇指"), QStringLiteral("thumb"));
    testFingerCombo_->addItem(QStringLiteral("食指"), QStringLiteral("index"));
    testFingerCombo_->addItem(QStringLiteral("中指"), QStringLiteral("middle"));
    testFingerCombo_->addItem(QStringLiteral("无名指"), QStringLiteral("ring"));
    testFingerCombo_->addItem(QStringLiteral("小指"), QStringLiteral("pinky"));
    connect(testFingerCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &HandPoseDebugDialog::onTestFingerChanged);
    testRow1->addWidget(testFingerCombo_, 1);
    testLayout->addLayout(testRow1);

    testMarkIdleChk_ = new QCheckBox(QStringLiteral("切换手指时自动插一帧\u201cidle\u201d分隔（推荐勾选，方便分段）"), testGroup);
    testMarkIdleChk_->setChecked(true);
    testLayout->addWidget(testMarkIdleChk_);

    auto* testRow2 = new QHBoxLayout();
    testRecordBtn_ = new QPushButton(QStringLiteral("开始录制"), testGroup);
    connect(testRecordBtn_, &QPushButton::clicked, this, &HandPoseDebugDialog::onTestRecordToggled);
    testRow2->addWidget(testRecordBtn_);
    testAnalyzeBtn_ = new QPushButton(QStringLiteral("分析"), testGroup);
    testAnalyzeBtn_->setEnabled(false);
    connect(testAnalyzeBtn_, &QPushButton::clicked, this, &HandPoseDebugDialog::onTestAnalyzeClicked);
    testRow2->addWidget(testAnalyzeBtn_);
    testExportBtn_ = new QPushButton(QStringLiteral("导出CSV…"), testGroup);
    testExportBtn_->setEnabled(false);
    connect(testExportBtn_, &QPushButton::clicked, this, &HandPoseDebugDialog::onTestExportClicked);
    testRow2->addWidget(testExportBtn_);
    testLayout->addLayout(testRow2);

    testStatusLabel_ = new QLabel(QStringLiteral("尚未开始录制。"), testGroup);
    testLayout->addWidget(testStatusLabel_);

    auto* testSplitter = new QSplitter(Qt::Horizontal, testGroup);
    testResultTable_ = new QTableWidget(0, 6, testGroup);
    testResultTable_->setHorizontalHeaderLabels({QStringLiteral("手指"), QStringLiteral("第几次"),
        QStringLiteral("帧数"), QStringLiteral("追踪率"), QStringLiteral("目标幅度(度)"), QStringLiteral("判定")});
    testResultTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    testResultTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    testSplitter->addWidget(testResultTable_);
    testIssuesText_ = new QTextEdit(testGroup);
    testIssuesText_->setReadOnly(true);
    testIssuesText_->setPlaceholderText(QStringLiteral("点\u201c分析\u201d后，具体问题（没动/串扰/抖动/追踪丢失）会列在这里。"));
    testSplitter->addWidget(testIssuesText_);
    testLayout->addWidget(testSplitter, 1);

    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(topRow, 3);
    mainLayout->addWidget(testGroup, 2);

    if (worker) {
        connect(worker, &HandTrackingWorker::handPoseReady, this, &HandPoseDebugDialog::onHandPoseReady);
        connect(worker, &HandTrackingWorker::handNotFound, this, &HandPoseDebugDialog::onHandNotFound);
        connect(worker, &HandTrackingWorker::coldStartDiagnosticsReady, this, &HandPoseDebugDialog::onColdStartDiagnostics);
        connect(worker, &HandTrackingWorker::radiusRefineStatsReady, this, &HandPoseDebugDialog::onRadiusRefineStats);
        connect(worker, &HandTrackingWorker::handPoseSmoothedReady, this, &HandPoseDebugDialog::onHandPoseSmoothedReady);
        connect(worker, &HandTrackingWorker::candidatePointsReady, this, &HandPoseDebugDialog::onCandidatePoints);
        // candidatePointsReady 默认不广播(标定向导专用信号，见
        // HandTrackingWorker::setCandidateBroadcastEnabled 顶部注释)，这里
        // 要展示候选点/遮挡记忆数量当诊断信息，必须显式打开，不然上面那行
        // connect永远收不到调用，candidateLabel_会一直停在初始文案。
        worker->setCandidateBroadcastEnabled(true);
    }

    auto* resetShortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
    connect(resetShortcut, &QShortcut::activated, this, [this, worker]() {
        if (worker) worker->resetTracking();
        skeleton_->clearPose();
        trajectory_->clearTrail();
        statusLabel_->setText(QStringLiteral("状态：<font color='#e0a030'>已手动重置(空格)，等待重新冷启动…</font>"));
        posLabel_->setText(QStringLiteral("位置：--"));
        rotLabel_->setText(QStringLiteral("旋转矩阵：--"));
        jointsLabel_->setText(QStringLiteral("关节角：--"));
        realtimeHistory_.clear();
        smoothedLabel_->setText(QStringLiteral("延迟精修流：(已重置，等待新窗口攒够数据)"));
    });

    sinceLastPose_.start();
    fpsWindowTimer_.start();

    stalenessTimer_ = new QTimer(this);
    connect(stalenessTimer_, &QTimer::timeout, this, &HandPoseDebugDialog::onStalenessTick);
    stalenessTimer_->start(200);
}

void HandPoseDebugDialog::onHandPoseReady(QVector3D wristPos, QVector<double> wristRot9,
                                          QVector<double> jointAngles16, qint64 ts_ns) {
    statusLabel_->setText(QStringLiteral("状态：<font color='#3ec46d'>已跟踪</font>"));
    coldStartLabel_->setText(QStringLiteral("冷启动诊断：(已跟踪，冷启动阶段结束)"));
    sinceLastPose_.restart();

    updateNumericReadout(wristPos, wristRot9, jointAngles16);
    skeleton_->setPose(wristRot9, jointAngles16);
    trajectory_->addPoint(wristPos);

    realtimeHistory_.push_back({ts_ns, wristPos, jointAngles16});
    if (int(realtimeHistory_.size()) > kMaxHistorySamples) realtimeHistory_.pop_front();

    ++framesInWindow_;
    if (fpsWindowTimer_.elapsed() >= 1000) {
        refreshFpsLabel();
        framesInWindow_ = 0;
        fpsWindowTimer_.restart();
    }
    lastTsNs_ = ts_ns;

    if (testRecording_) {
        TestRow r;
        r.tsNs = ts_ns; r.label = testCurrentLabel_; r.tracked = true;
        for (int i = 0; i < 16 && i < jointAngles16.size(); ++i) r.joints[size_t(i)] = jointAngles16[i];
        r.coarsePointCount = lastCoarsePointCount_;
        r.candidateCount = lastCandidateCount_;
        r.coastingCount = lastCoastingCount_;
        testRows_.push_back(r);
        if (testRows_.size() % 30 == 0) updateTestStatusLabel();
    }
}

void HandPoseDebugDialog::onStalenessTick() {
    tsAgeLabel_->setText(QStringLiteral("距上次更新：%1 ms").arg(sinceLastPose_.elapsed()));
}

void HandPoseDebugDialog::onColdStartDiagnostics(int numCandidates3D, int numDistanceMatches,
                                                 double bestRmsMm, int failReasonCode,
                                                 QString summary, qint64 /*ts_ns*/) {
    const bool success = (failReasonCode == 0);
    const QString color = success ? QStringLiteral("#3ec46d")
                        : (failReasonCode == 1 ? QStringLiteral("#e0a030") : QStringLiteral("#e05050"));

    QString rmsText = bestRmsMm >= 0.0 ? QString::number(bestRmsMm, 'f', 2) + QStringLiteral("mm")
                                       : QStringLiteral("(无候选组)");

    coldStartLabel_->setText(QStringLiteral(
        "冷启动诊断：<font color='%1'>%2</font><br>"
        "候选3D点：%3 个 &nbsp;|&nbsp; 距离匹配组：%4 组 &nbsp;|&nbsp; 最佳配准残差：%5")
        .arg(color, summary, QString::number(numCandidates3D), QString::number(numDistanceMatches), rmsText));
}

void HandPoseDebugDialog::onRadiusRefineStats(int coarsePointCount, QVector<int> matchedPerCam,
                                              QVector<int> fallbackPerCam, qint64 /*ts_ns*/) {
    lastCoarsePointCount_ = coarsePointCount;
    lastMatchedPerCam_ = matchedPerCam;
    lastFallbackPerCam_ = fallbackPerCam;

    const QString color = coarsePointCount >= 3 ? QStringLiteral("#3ec46d") : QStringLiteral("#e05050");

    QString perCamText;
    for (int i = 0; i < matchedPerCam.size(); ++i) {
        const int fb = (i < fallbackPerCam.size()) ? fallbackPerCam[i] : 0;
        perCamText += QStringLiteral("cam#%1: %2\u2713/%3\u517c\u5e95 &nbsp; ").arg(i).arg(matchedPerCam[i]).arg(fb);
    }
    if (perCamText.isEmpty()) perCamText = QStringLiteral("(无相机数据)");

    radiusRefineLabel_->setText(QStringLiteral(
        "两遍流程诊断：粗算候选点=<font color='%1'>%2</font> 个<br>%3")
        .arg(color, QString::number(coarsePointCount), perCamText));
}

void HandPoseDebugDialog::onCandidatePoints(QVector<QVector3D>, QVector<int>, bool, QVector<double>,
                                            QVector3D, qint64, QVector<int> missedFrames) {
    lastCandidateCount_ = missedFrames.size();
    lastCoastingCount_ = 0;
    for (int m : missedFrames) if (m > 0) ++lastCoastingCount_;
    updateCandidateLabel();
}

void HandPoseDebugDialog::updateCandidateLabel() {
    const QString color = lastCoastingCount_ > 0 ? QStringLiteral("#e0a030") : QStringLiteral("#3ec46d");
    candidateLabel_->setText(QStringLiteral(
        "候选3D点诊断：多视角三角化聚类后 <font color='%1'>%2</font> 个，"
        "其中遮挡记忆(coasting，用最后一次实测的旧值)：%3 个")
        .arg(color)
        .arg(lastCandidateCount_ >= 0 ? QString::number(lastCandidateCount_) : QStringLiteral("-"))
        .arg(lastCoastingCount_ >= 0 ? lastCoastingCount_ : 0));
}

void HandPoseDebugDialog::onHandNotFound(qint64 ts_ns) {
    statusLabel_->setText(QStringLiteral("状态：<font color='#e0a030'>未找到(冷启动中)</font>"));
    skeleton_->clearPose();
    lastTsNs_ = ts_ns;

    if (testRecording_) {
        TestRow r;
        r.tsNs = ts_ns; r.label = testCurrentLabel_; r.tracked = false;
        r.coarsePointCount = lastCoarsePointCount_;
        r.candidateCount = lastCandidateCount_;
        r.coastingCount = lastCoastingCount_;
        testRows_.push_back(r);
    }
}

void HandPoseDebugDialog::onHandPoseSmoothedReady(QVector3D wristPos, QVector<double> wristRot9,
                                                  QVector<double> jointAngles16, qint64 ts_ns) {
    Q_UNUSED(wristRot9);

    if (realtimeHistory_.empty()) {
        smoothedLabel_->setText(QStringLiteral(
            "延迟精修流：收到第一帧输出，但还没有可对齐的实时历史(等下一帧)"));
        return;
    }

    const RealtimeSample* best = nullptr;
    qint64 bestDiff = std::numeric_limits<qint64>::max();
    for (const auto& s : realtimeHistory_) {
        const qint64 diff = std::llabs(s.ts_ns - ts_ns);
        if (diff < bestDiff) { bestDiff = diff; best = &s; }
    }

    if (!best || jointAngles16.size() != 16 || best->angles.size() != 16) {
        smoothedLabel_->setText(QStringLiteral("延迟精修流：收到输出，但找不到可对齐的实时样本或数据形状不对"));
        return;
    }

    const double dx = double(wristPos.x()) - double(best->pos.x());
    const double dy = double(wristPos.y()) - double(best->pos.y());
    const double dz = double(wristPos.z()) - double(best->pos.z());
    const double posDeltaMm = std::sqrt(dx*dx + dy*dy + dz*dz);

    double maxJointDeltaRad = 0.0;
    for (int i = 0; i < 16; ++i)
        maxJointDeltaRad = std::max(maxJointDeltaRad, std::abs(jointAngles16[i] - best->angles[i]));

    smoothedLabel_->setText(QStringLiteral(
        "延迟精修流：跟同一时刻实时结果相比 位置差=%1mm 最大关节角差=%2\u00b0 (对齐时间差%3ms，UDP同步转发中)")
        .arg(posDeltaMm, 0, 'f', 2)
        .arg(maxJointDeltaRad * 180.0 / M_PI, 0, 'f', 2)
        .arg(bestDiff / 1e6, 0, 'f', 1));
}

void HandPoseDebugDialog::updateNumericReadout(const QVector3D& pos, const QVector<double>& rot9,
                                              const QVector<double>& angles) {
    posLabel_->setText(QStringLiteral("位置(mm)：x=%1  y=%2  z=%3")
        .arg(double(pos.x()), 0, 'f', 1).arg(double(pos.y()), 0, 'f', 1).arg(double(pos.z()), 0, 'f', 1));

    if (rot9.size() == 9) {
        rotLabel_->setText(QStringLiteral(
            "旋转矩阵：\n[%1 %2 %3]\n[%4 %5 %6]\n[%7 %8 %9]")
            .arg(rot9[0],6,'f',3).arg(rot9[1],6,'f',3).arg(rot9[2],6,'f',3)
            .arg(rot9[3],6,'f',3).arg(rot9[4],6,'f',3).arg(rot9[5],6,'f',3)
            .arg(rot9[6],6,'f',3).arg(rot9[7],6,'f',3).arg(rot9[8],6,'f',3));
    }

    if (angles.size() == 16) {
        static const char* names[16] = {
            "拇CMC屈","拇CMC展","拇MCP屈","拇IP屈",
            "食MCP屈","食MCP展","食PIP屈",
            "中MCP屈","中MCP展","中PIP屈",
            "无MCP屈","无MCP展","无PIP屈",
            "小MCP屈","小MCP展","小PIP屈",
        };
        QString text = QStringLiteral("关节角(弧度)：\n");
        for (int i = 0; i < 16; ++i) {
            text += QStringLiteral("%1=%2  ").arg(names[i]).arg(angles[i], 0, 'f', 2);
            if (i % 3 == 2) text += QStringLiteral("\n");
        }
        jointsLabel_->setText(text);
    }
}

void HandPoseDebugDialog::refreshFpsLabel() {
    fpsLabel_->setText(QStringLiteral("帧率：约 %1 fps").arg(framesInWindow_));
}

// ---------------------------------------------------------------------------
// 手指屈曲测试面板——交互 + 分析逻辑
// ---------------------------------------------------------------------------

void HandPoseDebugDialog::onTestFingerChanged(int) {
    const QString newLabel = testFingerCombo_->currentData().toString();
    if (testRecording_ && testMarkIdleChk_->isChecked() && newLabel != testCurrentLabel_) {
        testCurrentLabel_ = QStringLiteral("idle");
    }
    testCurrentLabel_ = newLabel;

    const auto& m = fingerColsMap();
    if (m.contains(newLabel)) skeleton_->setFocusFinger(m.value(newLabel).focusIdx);
    else skeleton_->setFocusFinger(-1);

    updateTestStatusLabel();
}

void HandPoseDebugDialog::onTestRecordToggled() {
    testRecording_ = !testRecording_;
    testRecordBtn_->setText(testRecording_ ? QStringLiteral("停止") : QStringLiteral("开始录制"));
    if (testRecording_) {
        testCurrentLabel_ = testFingerCombo_->currentData().toString();
    } else {
        const bool hasData = !testRows_.isEmpty();
        testAnalyzeBtn_->setEnabled(hasData);
        testExportBtn_->setEnabled(hasData);
    }
    updateTestStatusLabel();
}

void HandPoseDebugDialog::updateTestStatusLabel() {
    testStatusLabel_->setText(QStringLiteral("当前标签：%1\u3000已录制 %2 帧\u3000（%3）")
        .arg(testCurrentLabel_).arg(testRows_.size())
        .arg(testRecording_ ? QStringLiteral("录制中") : QStringLiteral("已停止")));
}

QVector<QPair<QString, QVector<HandPoseDebugDialog::TestRow>>> HandPoseDebugDialog::splitTestSegments() const {
    QVector<QPair<QString, QVector<TestRow>>> segments;
    QString curLabel; QVector<TestRow> curRows;
    const auto flush = [&]() {
        if (!curLabel.isEmpty() && !curRows.isEmpty()) segments.push_back({curLabel, curRows});
        curLabel.clear(); curRows.clear();
    };
    for (const auto& r : testRows_) {
        if (r.label == QStringLiteral("idle") || !fingerColsMap().contains(r.label)) { flush(); continue; }
        if (r.label != curLabel) { flush(); curLabel = r.label; }
        curRows.push_back(r);
    }
    flush();
    return segments;
}

HandPoseDebugDialog::TestSegmentResult HandPoseDebugDialog::analyzeTestSegment(
        const QString& label, const QVector<TestRow>& segRows, int segIdx) const {
    TestSegmentResult res;
    res.label = label; res.segIdx = segIdx; res.nFrames = segRows.size();
    res.status = QStringLiteral("PASS");
    if (segRows.isEmpty()) { res.status = QStringLiteral("FAIL"); res.issues << QStringLiteral("这一段没有任何数据"); return res; }

    const int nTracked = std::count_if(segRows.begin(), segRows.end(), [](const TestRow& r){ return r.tracked; });
    res.trackedRatio = double(nTracked) / double(segRows.size());
    bool hasTrackLoss = false, hasNoMotion = false, hasJitter = false, hasCrosstalk = false;

    if (res.trackedRatio < kMinTrackedRatio) {
        res.issues << QStringLiteral("追踪丢失严重(仅%1%帧追踪到手，低于%2%门槛)")
            .arg(res.trackedRatio*100.0, 0, 'f', 0).arg(kMinTrackedRatio*100.0, 0, 'f', 0);
        res.status = QStringLiteral("FAIL");
        hasTrackLoss = true;
    }

    // 【新增】把这一段录制时顺带记下的诊断字段(coarsePointCount/candidateCount/
    // coastingCount)取均值——这是"调参建议"能给得比拍脑袋准的关键：不是
    // 只看关节角判定结果本身，而是回头看"判定出问题的那段时间，管线上游
    // 到底是什么状态"，用这个状态反推大概率的根因方向。
    {
        double sumCoarse = 0.0; int nCoarse = 0;
        double sumCoastRatio = 0.0; int nCoastSamples = 0;
        for (const auto& r : segRows) {
            if (r.coarsePointCount >= 0) { sumCoarse += r.coarsePointCount; ++nCoarse; }
            if (r.candidateCount > 0) { sumCoastRatio += double(r.coastingCount) / double(r.candidateCount); ++nCoastSamples; }
        }
        res.avgCoarsePointCount = nCoarse > 0 ? sumCoarse / nCoarse : -1.0;
        res.avgCoastingRatio = nCoastSamples > 0 ? sumCoastRatio / nCoastSamples : -1.0;
    }

    auto rangeOf = [&](int jointIdx) -> double {
        double lo = 1e18, hi = -1e18;
        for (const auto& r : segRows) if (r.tracked) { lo = std::min(lo, r.joints[size_t(jointIdx)]); hi = std::max(hi, r.joints[size_t(jointIdx)]); }
        return (hi > lo) ? (hi - lo) : 0.0;
    };

    const FingerCols cols = fingerColsMap().value(label);
    const double primaryRange = rangeOf(cols.primary);
    const double secondaryRange = rangeOf(cols.secondary);
    res.targetRangeRad = std::max(primaryRange, secondaryRange);

    if (res.targetRangeRad < kMinFlexRangeRad) {
        res.issues << QStringLiteral("目标手指(%1)动作幅度只有%2rad(约%3度)，低于%4rad门槛——"
                                     "基本没动，或者追踪没跟上")
            .arg(cols.nameCn).arg(res.targetRangeRad, 0, 'f', 3).arg(res.targetRangeRad*57.3, 0, 'f', 1)
            .arg(kMinFlexRangeRad, 0, 'f', 2);
        res.status = QStringLiteral("FAIL");
        hasNoMotion = true;
    }

    {
        QVector<double> vals;
        for (const auto& r : segRows) if (r.tracked) vals.push_back(r.joints[size_t(cols.primary)]);
        QVector<int> signs;
        for (int i = 0; i + 1 < vals.size(); ++i) {
            const double d = vals[i+1] - vals[i];
            if (d > kJitterDeadbandRad) signs.push_back(1);
            else if (d < -kJitterDeadbandRad) signs.push_back(-1);
        }
        int changes = 0;
        for (int i = 0; i + 1 < signs.size(); ++i) if (signs[i] != signs[i+1]) ++changes;
        res.jitterRatio = signs.size() > 1 ? double(changes) / double(signs.size()) : 0.0;
        if (res.jitterRatio > kJitterSignChangeRatioWarn) {
            res.issues << QStringLiteral("目标关节一阶差分符号变化率%1%，高于%2%——"
                                         "疑似抖动/关联跳变，不是平滑的弯曲过程")
                .arg(res.jitterRatio*100.0, 0, 'f', 0).arg(kJitterSignChangeRatioWarn*100.0, 0, 'f', 0);
            if (res.status == QStringLiteral("PASS")) res.status = QStringLiteral("WARN");
            hasJitter = true;
        }
    }

    if (res.targetRangeRad > 1e-6) {
        QStringList crosstalkNotes;
        double worstRatio = 0.0;
        for (auto it = fingerAllFlexCols().constBegin(); it != fingerAllFlexCols().constEnd(); ++it) {
            if (it.key() == label) continue;
            double otherMax = 0.0;
            for (int idx : it.value()) otherMax = std::max(otherMax, rangeOf(idx));
            const double ratio = otherMax / res.targetRangeRad;
            worstRatio = std::max(worstRatio, ratio);
            if (ratio > kCrosstalkRatioWarn)
                crosstalkNotes << QStringLiteral("%1(幅度比%2%)").arg(fingerColsMap().value(it.key()).nameCn).arg(ratio*100.0, 0, 'f', 0);
        }
        if (!crosstalkNotes.isEmpty()) {
            const QString sev = worstRatio <= kCrosstalkRatioFail ? QStringLiteral("串扰") : QStringLiteral("严重串扰");
            res.issues << QStringLiteral("疑似%1——测%2时，这些手指也有明显幅度变化：%3"
                                         "（无名指/小指之间有一定生理性联动是正常的，"
                                         "轻微幅度不代表bug，这里只是超过%4%比例才报）")
                .arg(sev, cols.nameCn, crosstalkNotes.join(QStringLiteral("、"))).arg(kCrosstalkRatioWarn*100.0, 0, 'f', 0);
            if (worstRatio > kCrosstalkRatioFail) res.status = QStringLiteral("FAIL");
            else if (res.status == QStringLiteral("PASS")) res.status = QStringLiteral("WARN");
            hasCrosstalk = true;
        }
    }

    // ---- 【新增】调参建议：结合诊断信号推断大概率根因 ----
    // 算法不知道真实手的姿态，做不到"确诊"，只能做"假设此刻这几个诊断
    // 数字长这样，历史经验里最常见的原因是这个"这种条件推理——每条建议
    // 前面都标了触发它的具体条件，方便你自己判断这次是不是真的适用，
    // 不是把这当权威结论照单全收。
    const bool coarseLow = res.avgCoarsePointCount >= 0.0 && res.avgCoarsePointCount < 3.0;
    const bool coastingHigh = res.avgCoastingRatio >= 0.30;   // 3成以上候选点在遮挡记忆状态，算比较严重

    if (hasNoMotion && coarseLow) {
    } else if (hasNoMotion && !coarseLow) {
    }

    if (hasJitter && coastingHigh) {
    } else if (hasJitter && !coastingHigh) {
    }

    if (hasCrosstalk && !coarseLow) {
    }

    if (hasTrackLoss && coarseLow) {
        res.suggestions << QStringLiteral(
            "追踪频繁丢失同时候选点数据也普遍偏低，问题大概率在整体冷启动/连通性"
            "层面，不是这根手指的局部问题——参考上面\u201c冷启动诊断\u201d面板里最近一次"
            "失败的具体原因，先把冷启动稳定下来，再回头测单根手指的响应。");
    }

    return res;
}

void HandPoseDebugDialog::onTestAnalyzeClicked() {
    const auto segments = splitTestSegments();
    if (segments.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("分析"),
            QStringLiteral("没有找到任何非idle的测试段——检查录制时是不是一直停在\u201c待机\u201d没切换手指。"));
        return;
    }

    testResultTable_->setRowCount(0);
    testIssuesText_->clear();
    QHash<QString, int> perFingerCounter;
    QStringList issueLines;
    int nPass = 0, nWarn = 0, nFail = 0;

    for (const auto& seg : segments) {
        const QString& label = seg.first;
        const int segIdx = ++perFingerCounter[label];
        const TestSegmentResult r = analyzeTestSegment(label, seg.second, segIdx);

        const int row = testResultTable_->rowCount();
        testResultTable_->insertRow(row);
        auto setCell = [&](int col, const QString& text, const QColor& fg = QColor()) {
            auto* item = new QTableWidgetItem(text);
            if (fg.isValid()) item->setForeground(fg);
            testResultTable_->setItem(row, col, item);
        };
        const QColor statusColor = r.status == QStringLiteral("PASS") ? QColor("#3ec46d")
                                 : (r.status == QStringLiteral("WARN") ? QColor("#e0a030") : QColor("#e05050"));
        setCell(0, fingerColsMap().value(label).nameCn);
        setCell(1, QString::number(r.segIdx));
        setCell(2, QString::number(r.nFrames));
        setCell(3, QStringLiteral("%1%").arg(r.trackedRatio*100.0, 0, 'f', 0));
        setCell(4, QStringLiteral("%1").arg(r.targetRangeRad*57.3, 0, 'f', 1));
        setCell(5, r.status, statusColor);

        if (r.status == QStringLiteral("PASS")) ++nPass;
        else if (r.status == QStringLiteral("WARN")) ++nWarn;
        else ++nFail;

        if (!r.issues.isEmpty()) {
            issueLines << QStringLiteral("【%1 第%2次 - %3】").arg(fingerColsMap().value(label).nameCn).arg(r.segIdx).arg(r.status);
            for (const auto& issue : r.issues) issueLines << QStringLiteral("  - %1").arg(issue);
            // 诊断字段这段的均值——跟issues放一起，方便对照"判定结果"和"这段
            // 时间管线上游到底是什么状态"，不用切去别处翻。
            if (r.avgCoarsePointCount >= 0.0 || r.avgCoastingRatio >= 0.0) {
                QStringList diagBits;
                if (r.avgCoarsePointCount >= 0.0) diagBits << QStringLiteral("粗算候选点均值%1个").arg(r.avgCoarsePointCount, 0, 'f', 1);
                if (r.avgCoastingRatio >= 0.0) diagBits << QStringLiteral("遮挡记忆占比%1%").arg(r.avgCoastingRatio*100.0, 0, 'f', 0);
                issueLines << QStringLiteral("  [诊断参考] %1").arg(diagBits.join(QStringLiteral("，")));
            }
            if (!r.suggestions.isEmpty()) {
                issueLines << QStringLiteral("  [调参建议——算法不知道真实手的姿态，这是按当前诊断数字推断的大概率方向，不是确诊]");
                for (const auto& sug : r.suggestions) issueLines << QStringLiteral("    → %1").arg(sug);
            }
        }
    }

    QString summary = QStringLiteral("共%1个测试段。汇总：PASS %2 / WARN %3 / FAIL %4\n\n")
        .arg(segments.size()).arg(nPass).arg(nWarn).arg(nFail);
    if (nFail > 0) {
        summary += QStringLiteral("有FAIL项——先看下面具体是哪类问题(没动/串扰/追踪丢失)再决定往哪查。\n\n");
    } else if (nWarn > 0) {
        summary += QStringLiteral("没有FAIL但有WARN——基本能用，抖动/轻微串扰的迹象值得留意。\n\n");
    } else {
        summary += QStringLiteral("全部通过——至少在粗筛层面上，追踪端对这几个已知动作的响应符合预期。\n\n");
    }
    testIssuesText_->setPlainText(summary + issueLines.join('\n'));
}

void HandPoseDebugDialog::onTestExportClicked() {
    if (testRows_.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("导出"), QStringLiteral("还没有录到任何数据。"));
        return;
    }
    const QString defaultName = QStringLiteral("finger_flexion_test_%1.csv")
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_hhmmss")));
    const QString path = QFileDialog::getSaveFileName(this, QStringLiteral("导出CSV"), defaultName,
                                                       QStringLiteral("CSV (*.csv)"));
    if (path.isEmpty()) return;

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, QStringLiteral("导出失败"), QStringLiteral("无法写入文件：%1").arg(path));
        return;
    }
    QTextStream ts(&f);
    ts << "ts_ns,label,tracked,"
          "thumb_cmcFlex,thumb_cmcAbduct,thumb_mcp,thumb_ip,"
          "index_mcpFlex,index_mcpAbduct,index_pipFlex,"
          "middle_mcpFlex,middle_mcpAbduct,middle_pipFlex,"
          "ring_mcpFlex,ring_mcpAbduct,ring_pipFlex,"
          "pinky_mcpFlex,pinky_mcpAbduct,pinky_pipFlex,"
          "coarsePointCount,candidateCount,coastingCount\n";
    for (const auto& r : testRows_) {
        ts << r.tsNs << ',' << r.label << ',' << (r.tracked ? 1 : 0);
        for (int i = 0; i < 16; ++i) ts << ',' << QString::number(r.joints[size_t(i)], 'g', 8);
        ts << ',' << r.coarsePointCount << ',' << r.candidateCount << ',' << r.coastingCount << '\n';
    }
    f.close();

    QMessageBox::information(this, QStringLiteral("导出完成"),
        QStringLiteral("已导出 %1 行到：\n%2").arg(testRows_.size()).arg(path));
}

} // namespace mocap
