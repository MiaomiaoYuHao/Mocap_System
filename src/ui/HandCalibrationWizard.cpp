#include "ui/HandCalibrationWizard.hpp"
#include "ui/Theme.hpp"
#include "estimate/HandTrackingWorker.hpp"
#include "hand/HandTemplateStore.hpp"
#include "hand/HandCalibration.hpp"
#include "hand/HandSelfCalibration.hpp"
#include "hand/HandModel.hpp"
#include "hand/HandPose.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QTableWidget>
#include <QHeaderView>
#include <QTextEdit>
#include <QTabWidget>
#include <QGroupBox>
#include <QGridLayout>
#include <QSlider>
#include <QSettings>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>
#include <QFont>
#include <QtConcurrent/QtConcurrent>
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>

namespace mocap {

namespace {
// 世界系候选点表格的4列：下标/ID、X、Y、Z(mm)，两页共用同一套渲染逻辑。
// ids 可选(默认空)——BackTemplateCalibPage 传跨帧稳定ID进来，"#"列显示的
// 就是ID，不是"这一帧第几个"；FingerCalibPage 不靠这个身份做关联(它自己
// 按坐标值做最近邻延续，见文件头注释)，不传ids时"#"列退回显示行号，
// 保持原有行为不变。
void fillCandidateTable(QTableWidget* table, const QVector<QVector3D>& points,
                        const QVector<int>& ids = {}) {
    table->setRowCount(points.size());
    const bool haveIds = (ids.size() == points.size());
    for (int i = 0; i < points.size(); ++i) {
        table->setItem(i, 0, new QTableWidgetItem(QString::number(haveIds ? ids[i] : i)));
        table->setItem(i, 1, new QTableWidgetItem(QString::number(double(points[i].x()), 'f', 1)));
        table->setItem(i, 2, new QTableWidgetItem(QString::number(double(points[i].y()), 'f', 1)));
        table->setItem(i, 3, new QTableWidgetItem(QString::number(double(points[i].z()), 'f', 1)));
    }
}

QTableWidget* makeCandidateTable(QWidget* parent) {
    auto* t = new QTableWidget(0, 4, parent);
    t->setHorizontalHeaderLabels({QStringLiteral("#"), QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")});
    t->horizontalHeader()->setStretchLastSection(true);
    t->setSelectionBehavior(QAbstractItemView::SelectRows);
    t->setSelectionMode(QAbstractItemView::SingleSelection);
    t->setEditTriggers(QAbstractItemView::NoEditTriggers);
    return t;
}
} // namespace

// ---------------------------------------------------------------------------
// CandidatePointsScatterWidget
// ---------------------------------------------------------------------------

CandidatePointsScatterWidget::CandidatePointsScatterWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(180);
}

void CandidatePointsScatterWidget::setPoints(const QVector<QVector3D>& points, const QVector<int>& ids,
                                             const std::vector<int>& backIds, int forwardId,
                                             const QVector<int>& missedFrames) {
    points_ = points;
    ids_ = ids;
    missedFrames_ = missedFrames;
    backIds_ = backIds;
    forwardId_ = forwardId;
    update();
}

void CandidatePointsScatterWidget::recomputeTransform() {
    if (points_.isEmpty()) return;
    double minX = double(points_[0].x()), maxX = minX, minY = double(points_[0].y()), maxY = minY;
    for (const auto& p : points_) {
        minX = std::min(minX, double(p.x())); maxX = std::max(maxX, double(p.x()));
        minY = std::min(minY, double(p.y())); maxY = std::max(maxY, double(p.y()));
    }

    // 迟滞式更新(见头文件 viewBoxInit_ 注释)：扩大立即生效，收缩每帧只走
    // 一小步——单帧的幽灵点闪现/消失影响不到视图，整幅图不再跟着跳。
    if (!viewBoxInit_) {
        viewMinX_ = minX; viewMaxX_ = maxX; viewMinY_ = minY; viewMaxY_ = maxY;
        viewBoxInit_ = true;
    } else {
        constexpr double kShrinkAlpha = 0.03;   // 收缩速度：每帧向目标走3%，~1秒收敛@30fps刷新
        viewMinX_ = std::min(minX, viewMinX_ + (minX - viewMinX_) * kShrinkAlpha);
        viewMaxX_ = std::max(maxX, viewMaxX_ + (maxX - viewMaxX_) * kShrinkAlpha);
        viewMinY_ = std::min(minY, viewMinY_ + (minY - viewMinY_) * kShrinkAlpha);
        viewMaxY_ = std::max(maxY, viewMaxY_ + (maxY - viewMaxY_) * kShrinkAlpha);
    }

    // 候选点可能挤在一小坨(比如只有手背5点，彼此间距几十mm)，强制给个
    // 最小张幅，不然缩放系数会大到看不出层次——跟 HandPoseDebugDialog.cpp
    // 里 TrajectoryWidget 的同款处理是一个道理。
    const double minSpanMm = 60.0;
    const double spanX = std::max(minSpanMm, viewMaxX_ - viewMinX_);
    const double spanY = std::max(minSpanMm, viewMaxY_ - viewMinY_);
    const double margin = 28.0;
    scale_ = std::min((width() - 2*margin) / spanX, (height() - 2*margin) / spanY);
    centerWorld_ = QPointF((viewMinX_+viewMaxX_)*0.5, (viewMinY_+viewMaxY_)*0.5);
    centerScreen_ = QPointF(width()*0.5, height()*0.5);
}

QPointF CandidatePointsScatterWidget::toScreen(const QPointF& worldXY) const {
    // 世界 +Y 朝屏幕上方(取反屏幕Y)，+X 朝屏幕右方——跟 TrajectoryWidget
    // 同一个朝向约定，两个调试窗口看惯了不用重新适应方向。
    return centerScreen_ + QPointF((worldXY.x()-centerWorld_.x())*scale_,
                                   -(worldXY.y()-centerWorld_.y())*scale_);
}

void CandidatePointsScatterWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(0x14, 0x14, 0x14));

    painter.setPen(QColor(150,150,150));
    painter.drawText(8, 16, QStringLiteral("候选点俯视图(X-Y mm)——圈里数字是跨帧稳定编号(不会跳变)，直接点圈也能选"));

    if (points_.isEmpty()) {
        painter.setPen(QColor(120,120,120));
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("(暂无候选点，检查手是否在相机视野内)"));
        return;
    }

    recomputeTransform();

    QFont f = painter.font(); f.setPointSize(9); painter.setFont(f);

    for (int i = 0; i < points_.size(); ++i) {
        const QPointF sp = toScreen(QPointF(double(points_[i].x()), double(points_[i].y())));
        const int id = (i < ids_.size()) ? ids_[i] : i;   // 理论上ids_跟points_同长度，越界兜底显示数组位置
        const bool isBack = std::find(backIds_.begin(), backIds_.end(), id) != backIds_.end();
        const bool isForward = (id == forwardId_);
        // >0 = 遮挡记忆(coasting)：这个位置是最后一次真正看到时的冻结旧值，
        // 不是这一帧实测——画成暗色空心圈，用户一眼分清"实测"和"记忆"，
        // 也解释了"球没动但其实已经被挡住"这类困惑。
        const bool coasting = (i < missedFrames_.size()) && missedFrames_[i] > 0;

        QColor fill = QColor(120,120,120);        // 未选：灰
        if (isBack) fill = QColor(90,170,240);     // 手背点：蓝，跟表格高亮同色系
        if (isForward) fill = QColor(240,170,70);  // 方向提示点：橙，跟表格高亮同色系

        if (coasting) {
            painter.setBrush(Qt::NoBrush);                       // 空心
            painter.setPen(QPen(fill.darker(130), 2, Qt::DashLine));
        } else {
            painter.setBrush(fill);
            painter.setPen(QPen(Qt::white, 1));
        }
        painter.drawEllipse(sp, 9, 9);

        painter.setPen(coasting ? QColor(200,200,200) : QColor(Qt::black));
        painter.drawText(QRectF(sp.x()-9, sp.y()-9, 18, 18), Qt::AlignCenter, QString::number(id));
    }
}

void CandidatePointsScatterWidget::mousePressEvent(QMouseEvent* ev) {
    if (points_.isEmpty()) return;
    recomputeTransform();   // 保证跟当前这一帧的绘制用同一份变换，不会点偏

    const QPointF click = ev->position();
    int bestI = -1; double bestDist = 1e18;
    for (int i = 0; i < points_.size(); ++i) {
        const QPointF sp = toScreen(QPointF(double(points_[i].x()), double(points_[i].y())));
        const double dx = sp.x()-click.x(), dy = sp.y()-click.y();
        const double d = std::sqrt(dx*dx+dy*dy);
        if (d < bestDist) { bestDist = d; bestI = i; }
    }
    if (bestI >= 0 && bestDist <= 14.0) {   // 14px命中半径，比圆本身(9px)略宽松，好点中
        const int id = (bestI < ids_.size()) ? ids_[bestI] : bestI;
        emit pointClicked(id);
    }
}

// ---------------------------------------------------------------------------
// BackTemplateCalibPage
// ---------------------------------------------------------------------------

BackTemplateCalibPage::BackTemplateCalibPage(HandTrackingWorker* worker, HandTemplateStore* store, QWidget* parent)
    : QDialog(parent), worker_(worker), store_(store) {
    setWindowFlags(Qt::Widget);   // 作为tab页嵌入，不是独立弹窗

    instructionLabel_ = new QLabel(this);
    instructionLabel_->setWordWrap(true);

    // 【新增】候选点追踪参数——之前这三个数(maxAssocDist/maxMissedFrames/
    // minHitsToConfirm)是写死在HandTrackingWorker::candidateTracker_里的
    // 常量，手背标定这个阶段还没有手部模型可用，"编号飙升/不稳定/抖动"
    // 这类问题只能靠这几个参数直接调，之前没地方给你调，只能改代码重编译。
    // 现在做成滑块，带持久化(QSettings)——调到稳定为止之后，这个值会记
    // 住，不用每次重开都从默认值调起。
    auto* trackerGroup = new QGroupBox(QStringLiteral("候选点追踪参数（调到编号不再飙升/抖动为止）"), this);
    auto* trackerGrid = new QGridLayout(trackerGroup);
    // 【编译错误修复】QSettings 不可拷贝(拷贝构造函数是deleted的)——之前
    // 按值捕获进lambda(`[..., trackerSettings]`)编译不过。改成堆上分配、
    // 挂在this下(Qt父子对象生命周期管理，this销毁时自动清理)，lambda里
    // 捕获指针而不是值——指针可以随便拷贝，QSettings对象本身不需要拷贝。
    auto* trackerSettings = new QSettings(QStringLiteral("MocapSystem"), QStringLiteral("BackTemplateCandidateTracker"), this);

    struct Row { QSlider* slider; QLabel* valLabel; double scale; QString suffix; };
    auto addRow = [&](int row, const QString& label, const QString& key,
                      double minV, double maxV, double initV, double scale, const QString& suffix) -> Row {
        double loaded = trackerSettings->contains(key)
            ? std::clamp(trackerSettings->value(key).toDouble(), minV, maxV) : initV;
        auto* lbl = new QLabel(label, trackerGroup);
        auto* slider = new QSlider(Qt::Horizontal, trackerGroup);
        slider->setMinimum(int(std::round(minV * scale)));
        slider->setMaximum(int(std::round(maxV * scale)));
        slider->setValue(int(std::round(loaded * scale)));
        auto* valLabel = new QLabel(trackerGroup);
        valLabel->setMinimumWidth(64);
        valLabel->setText(QStringLiteral("%1%2").arg(loaded, 0, 'f', suffix == QStringLiteral("mm") ? 1 : 0).arg(suffix));
        trackerGrid->addWidget(lbl, row, 0);
        trackerGrid->addWidget(slider, row, 1);
        trackerGrid->addWidget(valLabel, row, 2);
        return {slider, valLabel, scale, suffix};
    };

    const Row assocRow = addRow(0, QStringLiteral("关联距离上限"), QStringLiteral("maxAssocDistMm"), 5.0, 60.0, 20.0, 10.0, QStringLiteral("mm"));
    const Row missedRow = addRow(1, QStringLiteral("遮挡记忆帧数"), QStringLiteral("maxMissedFrames"), 6.0, 120.0, 36.0, 1.0, QStringLiteral("帧"));
    const Row confirmRow = addRow(2, QStringLiteral("确认所需连续帧数"), QStringLiteral("minHitsToConfirm"), 2.0, 15.0, 6.0, 1.0, QStringLiteral("帧"));
    QSlider* assocSlider = assocRow.slider; QSlider* missedSlider = missedRow.slider; QSlider* confirmSlider = confirmRow.slider;

    auto applyTrackerParams = [this, assocSlider, missedSlider, confirmSlider, trackerSettings]() mutable {
        const double assoc = assocSlider->value() / 10.0;
        const int missed = missedSlider->value();
        const int confirm = confirmSlider->value();
        trackerSettings->setValue(QStringLiteral("maxAssocDistMm"), assoc);
        trackerSettings->setValue(QStringLiteral("maxMissedFrames"), missed);
        trackerSettings->setValue(QStringLiteral("minHitsToConfirm"), confirm);
        if (worker_) worker_->setCandidateTrackerParams(assoc, missed, confirm);
    };
    // 每个滑块自己刷新数值文字 + 统一触发applyTrackerParams(应用到worker+存档)。
    connect(assocSlider, &QSlider::valueChanged, this, [assocRow, applyTrackerParams](int v) mutable {
        assocRow.valLabel->setText(QStringLiteral("%1mm").arg(v / assocRow.scale, 0, 'f', 1));
        applyTrackerParams();
    });
    connect(missedSlider, &QSlider::valueChanged, this, [missedRow, applyTrackerParams](int v) mutable {
        missedRow.valLabel->setText(QStringLiteral("%1帧").arg(v));
        applyTrackerParams();
    });
    connect(confirmSlider, &QSlider::valueChanged, this, [confirmRow, applyTrackerParams](int v) mutable {
        confirmRow.valLabel->setText(QStringLiteral("%1帧").arg(v));
        applyTrackerParams();
    });
    // 构造时先按存档/默认值应用一次，不等用户去拖第一下滑块。
    applyTrackerParams();

    scatter_ = new CandidatePointsScatterWidget(this);
    connect(scatter_, &CandidatePointsScatterWidget::pointClicked, this, [this](int id){ selectId(id); });

    pointTable_ = makeCandidateTable(this);
    connect(pointTable_, &QTableWidget::cellClicked, this, &BackTemplateCalibPage::onRowClicked);

    auto* dorsalRow = new QHBoxLayout();
    dorsalRow->addWidget(new QLabel(QStringLiteral("手背朝向(大致选一个)："), this));
    dorsalCombo_ = new QComboBox(this);
    dorsalCombo_->addItems({QStringLiteral("+X"), QStringLiteral("-X"), QStringLiteral("+Y"),
                            QStringLiteral("-Y"), QStringLiteral("+Z"), QStringLiteral("-Z")});
    dorsalCombo_->setCurrentIndex(4);   // 默认 +Z，常见"手背朝上"的世界系约定
    dorsalRow->addWidget(dorsalCombo_);
    dorsalRow->addStretch(1);

    selectionLabel_ = new QLabel(this);
    selectionLabel_->setWordWrap(true);

    auto* resetBtn = new QPushButton(QStringLiteral("重新选择"), this);
    connect(resetBtn, &QPushButton::clicked, this, [this]{
        backIndices_.clear(); forwardIndex_ = -1; haveCalibratedResult_ = false;
        resultLabel_->setText(QStringLiteral("结果：--"));
        saveBtn_->setEnabled(false);
        refreshSelectionLabel();
    });

    calibrateBtn_ = new QPushButton(QStringLiteral("标定"), this);
    calibrateBtn_->setEnabled(false);
    connect(calibrateBtn_, &QPushButton::clicked, this, &BackTemplateCalibPage::onCalibrateClicked);

    resultLabel_ = new QLabel(QStringLiteral("结果：--"), this);
    resultLabel_->setWordWrap(true);

    saveBtn_ = new QPushButton(QStringLiteral("保存手背模板"), this);
    saveBtn_->setEnabled(false);
    connect(saveBtn_, &QPushButton::clicked, this, &BackTemplateCalibPage::onSaveClicked);

    auto* btnRow = new QHBoxLayout();
    btnRow->addWidget(resetBtn);
    btnRow->addWidget(calibrateBtn_);
    btnRow->addWidget(saveBtn_);
    btnRow->addStretch(1);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(instructionLabel_);
    layout->addWidget(trackerGroup);
    layout->addWidget(scatter_, 1);
    layout->addWidget(pointTable_, 1);
    layout->addLayout(dorsalRow);
    layout->addWidget(selectionLabel_);
    layout->addLayout(btnRow);
    layout->addWidget(resultLabel_);

    refreshSelectionLabel();

    if (worker_) {
        worker_->setCandidateBroadcastEnabled(true);
        connect(worker_, &HandTrackingWorker::candidatePointsReady, this, &BackTemplateCalibPage::onCandidatePoints);
    }
}

void BackTemplateCalibPage::onCandidatePoints(QVector<QVector3D> worldPoints, QVector<int> ids, bool /*hasWristPose*/,
                                              QVector<double> /*wristRot9*/, QVector3D /*wristPos*/, qint64 /*ts_ns*/,
                                              QVector<int> missedFrames) {
    latestPoints_ = worldPoints;
    latestIds_ = ids;
    latestMissed_ = missedFrames;
    refreshTable();
}

int BackTemplateCalibPage::idAt(int row) const {
    if (row < 0 || row >= latestIds_.size()) return -1;
    return latestIds_[row];
}

int BackTemplateCalibPage::rowOfId(int id) const {
    for (int i = 0; i < latestIds_.size(); ++i) if (latestIds_[i] == id) return i;
    return -1;
}

void BackTemplateCalibPage::refreshTable() {
    fillCandidateTable(pointTable_, latestPoints_, latestIds_);
    // 高亮已经点选过的行，让用户看得出"这个点已经标过了"，不用自己数——
    // 现在按ID匹配，不是按行号，就算这一帧候选点顺序变了，已经选中的
    // 物理点依然能正确高亮在它这一帧所在的那一行。
    for (int r = 0; r < pointTable_->rowCount(); ++r) {
        const int id = idAt(r);
        // 深色主题下这三档必须换成深色底 —— 原来是浅蓝/浅橙/白，配上浅色
        // 文字就是白字白底，整行等于看不见。
        QColor bg = theme::editorBg();                       // 未选：跟表格底同色
        bool isBack = std::find(backIndices_.begin(), backIndices_.end(), id) != backIndices_.end();
        if (isBack) bg = theme::selectBg();                  // 手背点：选中蓝
        if (id == forwardIndex_) bg = QColor(0x4A, 0x38, 0x12);  // 方向提示点：暗琥珀
        for (int c = 0; c < 4; ++c) if (auto* it = pointTable_->item(r, c)) it->setBackground(bg);
    }
    // 散点图跟表格用同一份 backIndices_/forwardIndex_(现在都是ID空间)，
    // 两边高亮永远同步，不需要另外维护一份状态。
    scatter_->setPoints(latestPoints_, latestIds_, backIndices_, forwardIndex_, latestMissed_);
}

void BackTemplateCalibPage::refreshSelectionLabel() {
    if (int(backIndices_.size()) < 5) {
        selectionLabel_->setText(QStringLiteral(
            "请在上面表格里依次点选手背的5个marker点(还需 %1 个)。手保持静止，"
            "确保候选点里能同时看到全部5颗——如果表格里点数一直不到5，"
            "检查相机是否都能看到手背、标定是否正常。")
            .arg(5 - int(backIndices_.size())));
    } else if (forwardIndex_ < 0) {
        selectionLabel_->setText(QStringLiteral(
            "已选好5个手背点。现在点选一个大致朝手指方向的参考点(比如中指尖此刻的位置，"
            "不需要精确，只要不跟手背法线平行就行)。"));
    } else {
        selectionLabel_->setText(QStringLiteral("已选好全部点位，点击「标定」。"));
        calibrateBtn_->setEnabled(true);
    }
}

void BackTemplateCalibPage::onRowClicked(int row, int /*col*/) {
    const int id = idAt(row);
    if (id < 0) return;
    selectId(id);
}

void BackTemplateCalibPage::selectId(int id) {
    if (int(backIndices_.size()) < 5) {
        if (std::find(backIndices_.begin(), backIndices_.end(), id) == backIndices_.end())
            backIndices_.push_back(id);
    } else if (forwardIndex_ < 0) {
        if (std::find(backIndices_.begin(), backIndices_.end(), id) == backIndices_.end())
            forwardIndex_ = id;
    }
    refreshTable();
    refreshSelectionLabel();
}

void BackTemplateCalibPage::onDorsalPresetChanged(int) {}

void BackTemplateCalibPage::onCalibrateClicked() {
    if (int(backIndices_.size()) != 5 || forwardIndex_ < 0) return;

    // backIndices_/forwardIndex_ 存的是跨帧稳定ID，不是latestPoints_里的
    // 数组下标——必须先查这个ID在"这一帧"表格里对应第几行，再去取坐标。
    // 万一之前选中的某个点这一帧恰好丢了(遮挡/暂时跟丢)，rowOfId会返回
    // -1，这里要明确拦住、报错，不能拿旧数据或者越界访问。
    std::vector<int> backRows(5, -1);
    bool allFound = true;
    for (int i = 0; i < 5; ++i) {
        backRows[size_t(i)] = rowOfId(backIndices_[size_t(i)]);
        if (backRows[size_t(i)] < 0) allFound = false;
    }
    const int forwardRow = rowOfId(forwardIndex_);
    if (!allFound || forwardRow < 0) {
        resultLabel_->setText(QStringLiteral(
            "<font color='#e05050'>标定失败：之前选中的某个点这一帧暂时看不到了(可能被遮挡/跟丢)，"
            "请保持手静止、确保全部选中的点都在候选点列表里，再点一次「标定」。</font>"));
        saveBtn_->setEnabled(false);
        haveCalibratedResult_ = false;
        return;
    }

    static const HandVec3 kDorsalDirs[6] = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };
    const HandVec3 dorsalHint = kDorsalDirs[dorsalCombo_->currentIndex()];

    std::array<HandVec3,5> worldPts{};
    for (int i = 0; i < 5; ++i) {
        const auto& p = latestPoints_[backRows[size_t(i)]];
        worldPts[size_t(i)] = { double(p.x()), double(p.y()), double(p.z()) };
    }
    const auto& fp = latestPoints_[forwardRow];
    const HandVec3 forwardHint = { double(fp.x()), double(fp.y()), double(fp.z()) };

    const auto calib = calibrateHandBackTemplate(worldPts, forwardHint, dorsalHint);

    if (!calib.valid) {
        resultLabel_->setText(QStringLiteral(
            "<font color='#e05050'>标定失败：%1</font><br>最小三角形面积：%2 mm²(要求 &gt;= 100)")
            .arg(QString::fromUtf8(calib.failReason)).arg(calib.geometryCheck.minTriangleAreaMm2, 0, 'f', 1));
        saveBtn_->setEnabled(false);
        haveCalibratedResult_ = false;
        return;
    }

    // 回代验证：用刚标定出来的模板 + Kabsch 反解一次，看重建残差多大——
    // 跟单测里做的"round-trip检验"是同一个逻辑，这里做给用户看，而不是
    // 只在开发阶段测过一次就假设现场也一样准。
    std::array<bool,5> allVisible{true,true,true,true,true};
    const auto pose = solveHandBackPose(calib.templateLocal, worldPts, allVisible);

    for (int i=0;i<5;++i) pendingTemplateLocal_[i] = calib.templateLocal[size_t(i)];
    haveCalibratedResult_ = true;
    saveBtn_->setEnabled(pose.confident);

    resultLabel_->setText(QStringLiteral(
        "<font color='%1'>标定%2</font><br>配准残差RMS：%3 mm &nbsp;|&nbsp; 最小三角形面积：%4 mm²")
        .arg(pose.confident ? QStringLiteral("#3ec46d") : QStringLiteral("#e0a030"),
            pose.confident ? QStringLiteral("成功，可以保存") : QStringLiteral("数值上跑通了，但配准残差偏大，建议重新采集"))
        .arg(pose.rms, 0, 'f', 2)
        .arg(calib.geometryCheck.minTriangleAreaMm2, 0, 'f', 1));
}

void BackTemplateCalibPage::onSaveClicked() {
    if (!haveCalibratedResult_ || !store_) return;
    std::array<HandVec3,5> templ{};
    for (int i=0;i<5;++i) templ[size_t(i)] = pendingTemplateLocal_[i];
    store_->setBackTemplate(templ);
    if (worker_) worker_->reloadTemplate();
    resultLabel_->setText(resultLabel_->text() + QStringLiteral("<br><b>已保存。</b>"));
    emit backTemplateCalibrated();
}

// ---------------------------------------------------------------------------
// FingerCalibPage
// ---------------------------------------------------------------------------

FingerCalibPage::FingerCalibPage(HandTrackingWorker* worker, HandTemplateStore* store, QWidget* parent)
    : QDialog(parent), worker_(worker), store_(store) {
    setWindowFlags(Qt::Widget);

    fingerCombo_ = new QComboBox(this);
    fingerCombo_->addItems({QStringLiteral("拇指"), QStringLiteral("食指"), QStringLiteral("中指"),
                            QStringLiteral("无名指"), QStringLiteral("小指")});
    connect(fingerCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &FingerCalibPage::onFingerComboChanged);

    instructionLabel_ = new QLabel(this);
    instructionLabel_->setWordWrap(true);

    trackingStatusLabel_ = new QLabel(QStringLiteral("追踪状态：--"), this);

    scatter_ = new CandidatePointsScatterWidget(this);
    connect(scatter_, &CandidatePointsScatterWidget::pointClicked, this, [this](int id) {
        const int row = displayedIds_.indexOf(id);
        if (row >= 0) onTablePointClicked(row, 0);
    });

    pointTable_ = makeCandidateTable(this);
    connect(pointTable_, &QTableWidget::cellClicked, this, &FingerCalibPage::onTablePointClicked);

    auto* bootstrapRow = new QHBoxLayout();
    for (int i = 0; i < 3; ++i) {
        bootstrapHint_[i] = new QLabel(QStringLiteral("○ 第%1颗(未选)").arg(i+1), this);
        bootstrapRow->addWidget(bootstrapHint_[i]);
    }
    bootstrapRow->addStretch(1);

    // 静态标定：行业标准的"单姿势骨架缩放"——手掌平放桌面五指伸直，
    // 保持不动半秒即出结果，不需要活动手指、没有迭代优化、不会陷局部
    // 极小。它既可以单独用(结果直接可保存)，也自动成为后续"动态精修"
    // 的初值(初值好，精修的全局重启次数从6降到2，速度约3倍)。
    staticBtn_ = new QPushButton(QStringLiteral("静态标定(手放平贴桌面)"), this);
    staticBtn_->setEnabled(false);
    connect(staticBtn_, &QPushButton::clicked, this, &FingerCalibPage::onStaticCalibClicked);

    startBtn_ = new QPushButton(QStringLiteral("开始采集"), this);
    startBtn_->setEnabled(false);
    connect(startBtn_, &QPushButton::clicked, this, &FingerCalibPage::onStartCaptureClicked);

    stopBtn_ = new QPushButton(QStringLiteral("停止并计算"), this);
    stopBtn_->setEnabled(false);
    connect(stopBtn_, &QPushButton::clicked, this, &FingerCalibPage::onStopAndComputeClicked);

    saveBtn_ = new QPushButton(QStringLiteral("保存这根手指的结构参数"), this);
    saveBtn_->setEnabled(false);
    connect(saveBtn_, &QPushButton::clicked, this, &FingerCalibPage::onSaveClicked);

    auto* btnRow = new QHBoxLayout();
    btnRow->addWidget(staticBtn_);
    btnRow->addWidget(startBtn_);
    btnRow->addWidget(stopBtn_);
    btnRow->addWidget(saveBtn_);
    btnRow->addStretch(1);

    captureStatusLabel_ = new QLabel(QStringLiteral("采集状态：未开始"), this);
    resultLabel_ = new QLabel(QStringLiteral("结果：--"), this);
    resultLabel_->setWordWrap(true);

    logView_ = new QTextEdit(this);
    logView_->setReadOnly(true);
    logView_->setMaximumHeight(80);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(fingerCombo_);
    layout->addWidget(instructionLabel_);
    layout->addWidget(trackingStatusLabel_);
    layout->addWidget(scatter_, 1);
    layout->addWidget(pointTable_, 1);
    layout->addLayout(bootstrapRow);
    layout->addLayout(btnRow);
    layout->addWidget(captureStatusLabel_);
    layout->addWidget(resultLabel_);
    layout->addWidget(logView_);

    onFingerComboChanged(0);

    if (worker_) {
        worker_->setCandidateBroadcastEnabled(true);
        connect(worker_, &HandTrackingWorker::candidatePointsReady, this, &FingerCalibPage::onCandidatePoints);
    }

    computeWatcher_ = new QFutureWatcher<FingerCalibResult>(this);
    connect(computeWatcher_, &QFutureWatcher<FingerCalibResult>::finished, this, &FingerCalibPage::onComputeFinished);
}

void FingerCalibPage::onFingerComboChanged(int idx) {
    static const char* names[5] = {"拇指(4自由度：CMC屈/展、MCP屈、IP屈)", "食指", "中指", "无名指", "小指"};
    instructionLabel_->setText(QStringLiteral(
        "正在标定：%1。请先在下方表格里【按从近端到远端的顺序】依次点选这根手指"
        "当前能看到的3颗marker(近节/中节/远节)，选完后点「开始采集」，然后自然地把"
        "这根手指弯曲、伸展、外展活动几秒(角度越多样越好)，最后点「停止并计算」。")
        .arg(QString::fromUtf8(names[idx])));
    bootstrapClickCount_ = 0;
    haveLastKnown_ = false;
    bootstrapSelectedIds_.clear();
    capturing_ = false;
    capturedPositions_.clear();
    capturedVisible_.clear();
    haveComputedResult_ = false;
    // 静态标定状态一并重置——上一根手指的静态结果不能拿来当这一根的初值
    staticCapturing_ = false;
    staticAccumCount_ = 0;
    haveStaticParam_ = false;
    staticParamFingerIdx_ = -1;
    if (staticBtn_) staticBtn_->setEnabled(false);
    startBtn_->setEnabled(false);
    stopBtn_->setEnabled(false);
    saveBtn_->setEnabled(false);
    resultLabel_->setText(QStringLiteral("结果：--"));
    for (int i=0;i<3;++i) bootstrapHint_[i]->setText(QStringLiteral("○ 第%1颗(未选)").arg(i+1));
}

std::array<double,3> FingerCalibPage::toWristLocal(const QVector3D& worldPt,
                                                    const QVector<double>& wristRot9, const QVector3D& wristPos) const {
    // local = R^T * (world - wristPos)，R是手腕->世界(行主序)，取转置等于
    // 世界->手腕(旋转矩阵的逆就是转置，前提R是正交阵，这里一定是)。
    const double dx = double(worldPt.x())-double(wristPos.x());
    const double dy = double(worldPt.y())-double(wristPos.y());
    const double dz = double(worldPt.z())-double(wristPos.z());
    return {
        wristRot9[0]*dx + wristRot9[3]*dy + wristRot9[6]*dz,
        wristRot9[1]*dx + wristRot9[4]*dy + wristRot9[7]*dz,
        wristRot9[2]*dx + wristRot9[5]*dy + wristRot9[8]*dz,
    };
}

bool FingerCalibPage::isKnownBackMarker(const std::array<double,3>& localPt, double tolMm) const {
    if (!store_) return false;
    const auto& back = store_->data().backMarkers;   // 已经是腕部局部系下的坐标，跟localPt同一个坐标系，可以直接比距离
    for (const auto& b : back) {
        const double dx = localPt[0]-b[0], dy = localPt[1]-b[1], dz = localPt[2]-b[2];
        if (std::sqrt(dx*dx+dy*dy+dz*dz) <= tolMm) return true;
    }
    return false;
}

void FingerCalibPage::onCandidatePoints(QVector<QVector3D> worldPoints, QVector<int> ids, bool hasWristPose,
                                        QVector<double> wristRot9, QVector3D wristPos, qint64 /*ts_ns*/,
                                        QVector<int> missedFrames) {
    trackingStatusLabel_->setText(hasWristPose
        ? QStringLiteral("追踪状态：<font color='#3ec46d'>已跟踪</font>(标定在手腕局部系下进行)")
        : QStringLiteral("追踪状态：<font color='#e0a030'>未跟踪</font>——请先完成步骤1(手背模板)并确保当前能稳定冷启动"));
    startBtn_->setEnabled(hasWristPose && bootstrapClickCount_ == 3 && !staticCapturing_);
    staticBtn_->setEnabled(hasWristPose && bootstrapClickCount_ == 3 && !staticCapturing_);
    if (!hasWristPose) return;

    // 表格显示手腕局部系坐标(而不是世界系)，这样用户看到的数字就是标定
    // 用的坐标系，直观一些。同时排除掉已知的手背5点——候选点列表里手背
    // 跟手指候选点混在一起，不排除的话用户没法一眼看出"这几个是手背，
    // 跟这根手指没关系"，容易选错。
    QVector<QVector3D> localPts;
    QVector<int> localIds;
    QVector<int> localMissed;
    const bool haveIds = (ids.size() == worldPoints.size());
    const bool haveMissed = (missedFrames.size() == worldPoints.size());
    localPts.reserve(worldPoints.size());
    localIds.reserve(worldPoints.size());
    localMissed.reserve(worldPoints.size());
    for (int i = 0; i < worldPoints.size(); ++i) {
        const auto lp = toWristLocal(worldPoints[i], wristRot9, wristPos);
        if (isKnownBackMarker(lp)) continue;   // 已知手背marker，从这根手指的候选列表里剔除
        localPts.push_back(QVector3D(float(lp[0]), float(lp[1]), float(lp[2])));
        localIds.push_back(haveIds ? ids[i] : i);
        localMissed.push_back(haveMissed ? missedFrames[i] : 0);
    }
    displayedIds_ = localIds;
    refreshTable(localPts, localIds, localMissed);

    if (!capturing_ && !staticCapturing_) return;

    // 采集中：把 lastKnownLocal_ 的3个位置关联到这一帧的候选点。
    //
    // 【互斥关联，修过一个真bug】以前是3颗marker各自独立找最近邻，彼此
    // 之间没有互斥——某颗marker被遮挡、场上只剩2个候选点时，中节和远节
    // 会双双关联到同一个候选点，并且都标成"可见"。这等于告诉优化器
    // "两颗相隔20多mm的marker在同一个空间位置"，FK无论如何拟合不出来，
    // 残差必然爆炸。合成数据复现(25%随机遮挡、0.3mm噪声、150帧)：
    //   无互斥：96.7%的帧发生重复指派，标定残差RMS=2.56mm
    //   互斥后：0帧重复指派，残差RMS=0.39mm(与无遮挡基线0.377持平)
    // 真实条件(遮挡更久、手腕位姿自身误差叠加)下无互斥版本会直接爆过
    // 5mm通过阈值——这就是"手指标定经常算出来残差很大"的主要原因。
    //
    // 做法：把所有(marker, 候选点)距离对按距离从小到大排序，依次贪心指派，
    // 每个候选点最多被领走一次、每颗marker最多领一个点。3x候选这种小规模
    // 下贪心和匈牙利算法结果几乎总是一致，不值得为它引入完整的最优指派。
    std::array<std::array<double,3>,3> frameMarkers{};
    std::array<bool,3> frameVisible{false,false,false};
    const double kMaxJumpMm = 25.0;   // 单帧最大允许位移，超过视为跟丢

    struct AssocPair { double d; int k; int i; };
    std::vector<AssocPair> assocPairs;
    for (int k = 0; k < 3; ++k) {
        for (int i = 0; i < localPts.size(); ++i) {
            const double dx = double(localPts[i].x()) - lastKnownLocal_[k][0];
            const double dy = double(localPts[i].y()) - lastKnownLocal_[k][1];
            const double dz = double(localPts[i].z()) - lastKnownLocal_[k][2];
            const double d = std::sqrt(dx*dx+dy*dy+dz*dz);
            if (d <= kMaxJumpMm) assocPairs.push_back({d, k, i});
        }
    }
    std::sort(assocPairs.begin(), assocPairs.end(),
              [](const AssocPair& a, const AssocPair& b){ return a.d < b.d; });
    std::array<bool,3> markerTaken{false,false,false};
    std::vector<bool> candTaken(size_t(localPts.size()), false);
    for (const AssocPair& p : assocPairs) {
        if (markerTaken[size_t(p.k)] || candTaken[size_t(p.i)]) continue;
        markerTaken[size_t(p.k)] = true;
        candTaken[size_t(p.i)] = true;
        frameMarkers[size_t(p.k)] = { double(localPts[p.i].x()), double(localPts[p.i].y()), double(localPts[p.i].z()) };
        frameVisible[size_t(p.k)] = true;
        lastKnownLocal_[p.k] = frameMarkers[size_t(p.k)];
    }

    // ---- 静态标定采集分支：手保持平放静止，攒够帧数取平均后闭式求解 ----
    if (staticCapturing_) {
        const bool all3 = frameVisible[0] && frameVisible[1] && frameVisible[2];
        if (all3) {
            for (int k = 0; k < 3; ++k)
                for (int d2 = 0; d2 < 3; ++d2)
                    staticAccum_[size_t(k)][size_t(d2)] += frameMarkers[size_t(k)][size_t(d2)];
            ++staticAccumCount_;
            captureStatusLabel_->setText(QStringLiteral(
                "静态标定采集中：%1/%2 帧(保持手平放不动)").arg(staticAccumCount_).arg(kStaticFramesGoal));
        }
        if (staticAccumCount_ >= kStaticFramesGoal) finishStaticCalibration();
        return;   // 静态采集不进动态采集的push逻辑
    }

    const int visibleCount = int(frameVisible[0])+int(frameVisible[1])+int(frameVisible[2]);
    if (visibleCount < 2) {
        ++missedFrameStreak_;
        if (missedFrameStreak_ % 15 == 1)
            logView_->append(QStringLiteral("[警告] 连续多帧关联不到2颗以上marker，可能跟丢了——如果手指动作幅度突然变化，建议停止重新采集。"));
    } else {
        missedFrameStreak_ = 0;
    }

    capturedPositions_.push_back(frameMarkers);
    capturedVisible_.push_back(frameVisible);
    captureStatusLabel_->setText(QStringLiteral("采集状态：进行中，已采集 %1 帧(连续跟丢 %2 帧)")
        .arg(capturedPositions_.size()).arg(missedFrameStreak_));
}

void FingerCalibPage::onTablePointClicked(int row, int /*col*/) {
    if (bootstrapClickCount_ >= 3) return;   // 已经点满3个，忽略后续点击(用户想重选请先切换手指或重新进入本页)
    if (row < 0 || row >= pointTable_->rowCount()) return;

    bool ok1=false, ok2=false, ok3=false;
    const double x = pointTable_->item(row,1)->text().toDouble(&ok1);
    const double y = pointTable_->item(row,2)->text().toDouble(&ok2);
    const double z = pointTable_->item(row,3)->text().toDouble(&ok3);
    if (!ok1 || !ok2 || !ok3) return;

    lastKnownLocal_[bootstrapClickCount_] = {x, y, z};
    if (row < displayedIds_.size()) bootstrapSelectedIds_.push_back(displayedIds_[row]);
    bootstrapHint_[bootstrapClickCount_]->setText(QStringLiteral("● 第%1颗(已选)").arg(bootstrapClickCount_+1));
    ++bootstrapClickCount_;

    if (bootstrapClickCount_ == 3) {
        haveLastKnown_ = true;
        startBtn_->setEnabled(true);
        logView_->append(QStringLiteral("[提示] 3颗marker已标记，可以点「开始采集」了。"));
    }
}

void FingerCalibPage::onStaticCalibClicked() {
    if (bootstrapClickCount_ != 3 || !haveLastKnown_) return;
    staticCapturing_ = true;
    staticAccumCount_ = 0;
    for (auto& a : staticAccum_) a = {0,0,0};
    staticBtn_->setEnabled(false);
    startBtn_->setEnabled(false);
    captureStatusLabel_->setText(QStringLiteral(
        "静态标定：请把手掌平放在桌面上、这根手指自然伸直，保持不动…"));
    logView_->append(QStringLiteral("[静态标定] 开始采集，保持手平放静止约半秒。"));
}

void FingerCalibPage::finishStaticCalibration() {
    staticCapturing_ = false;
    staticBtn_->setEnabled(true);
    startBtn_->setEnabled(true);

    std::array<double,3> avg[3];
    for (int k = 0; k < 3; ++k)
        for (int d2 = 0; d2 < 3; ++d2)
            avg[k][size_t(d2)] = staticAccum_[size_t(k)][size_t(d2)] / double(staticAccumCount_);

    const int fingerIdx = fingerCombo_->currentIndex();
    const auto r = calibrateFingerFromStaticPose(avg[0], avg[1], avg[2], fingerParam(fingerIdx));

    if (!r.valid) {
        captureStatusLabel_->setText(QStringLiteral("静态标定：未通过"));
        resultLabel_->setText(QStringLiteral(
            "<font color='#e05050'>静态标定未通过(共线偏差 %1 mm)——三颗marker没有排成一条直线，"
            "通常是手指没伸直/没放平。把手掌整个贴在桌面上、手指自然伸直再试一次。</font>")
            .arg(r.rmsMm, 0, 'f', 2));
        return;
    }

    // 结果直接可保存(走现有的保存路径)，同时记为后续动态精修的初值。
    staticParam_ = r.param;
    haveStaticParam_ = true;
    staticParamFingerIdx_ = fingerIdx;
    if (fingerIdx == 0) { pendingThumbResult_.param = r.param; pendingThumbResult_.valid = true; }
    else                { pendingFingerResult_.param = r.param; pendingFingerResult_.valid = true; }
    haveComputedResult_ = true;
    computingFingerIdx_ = fingerIdx;
    saveBtn_->setEnabled(true);

    captureStatusLabel_->setText(QStringLiteral("静态标定：完成(%1帧平均)").arg(staticAccumCount_));
    resultLabel_->setText(QStringLiteral(
        "结果：<font color='#3ec46d'>静态标定完成 ✓</font> 指骨长 %1 / %2 / %3 mm，共线偏差 %4 mm。"
        "<br>可以直接「保存」；想更准可以再点「开始采集」活动手指做动态精修"
        "(有了这个初值，精修会快很多)。")
        .arg(r.param.lengths[0], 0, 'f', 1).arg(r.param.lengths[1], 0, 'f', 1)
        .arg(r.param.lengths[2], 0, 'f', 1).arg(r.rmsMm, 0, 'f', 2));
    logView_->append(QStringLiteral("[静态标定] 完成，结果已就绪，可保存或继续动态精修。"));
}

void FingerCalibPage::onStartCaptureClicked() {
    if (!haveLastKnown_) return;
    capturing_ = true;
    capturedPositions_.clear();
    capturedVisible_.clear();
    missedFrameStreak_ = 0;
    startBtn_->setEnabled(false);
    stopBtn_->setEnabled(true);
    captureStatusLabel_->setText(QStringLiteral("采集状态：进行中，已采集 0 帧——现在开始活动这根手指"));
    logView_->append(QStringLiteral("[提示] 开始采集，请自然活动这根手指(屈伸、外展)几秒。"));
}

void FingerCalibPage::onStopAndComputeClicked() {
    capturing_ = false;
    stopBtn_->setEnabled(false);

    const int fingerIdx = fingerCombo_->currentIndex();   // 0拇指 1食 2中 3无名 4小指
    const int minFrames = fingerIdx == 0 ? 4 : 3;
    // 按【有效帧】(>=2颗可见)判断够不够，不按原始帧数——跟丢期间攒的帧
    // 反正进不了计算，比如"采了348帧但其中大半在跟丢"这种情况，按原始
    // 帧数放行，跑完一轮才发现有效帧不够，白等十几秒还得重采。
    int earlyValidCount = 0;
    for (const auto& v : capturedVisible_)
        if (int(v[0]) + int(v[1]) + int(v[2]) >= 2) ++earlyValidCount;
    if (earlyValidCount < minFrames * 3) {
        resultLabel_->setText(QStringLiteral(
            "<font color='#e05050'>有效帧太少(采集%1帧，其中有效%2帧)——有效指该帧能"
            "同时关联到至少2颗marker。请重新采集，动作放缓、保持marker尽量不被遮挡。</font>")
            .arg(capturedPositions_.size()).arg(earlyValidCount));
        return;
    }

    // calibrateFingerStructure/calibrateThumbStructure 是秒级到分钟级的
    // 重计算(多起点全局重启的交替优化，帧数一多——比如你这里223帧——
    // 实测会让GUI线程卡到系统判定"未响应")，必须丢到后台线程跑，不能直接
    // 在这个按钮点击槽函数里同步调用。用 QtConcurrent::run 扔给Qt的全局
    // 线程池，QFutureWatcher 负责在算完之后把结果送回GUI线程(见构造函数
    // 里的 connect)，这期间界面全程能响应、能看到"计算中"提示，不会再
    // 出现"未响应"。

    // 数据先按值拷贝出来给后台线程用(不能让后台线程直接摸 capturedPositions_
    // 这些成员——用户理论上可以在计算跑着的时候又点了别的按钮改动这些成员，
    // 那样会是典型的跨线程数据竞争。拷贝一份隔离开，后台线程只碰自己这份)。
    // 下面的降采样顺带就完成了这次拷贝，不用再多拷一遍。
    //
    // 【降采样】采集是每帧无条件 push_back 的，帧数 = 相机帧率 × 采集时长，
    // 而这套交替优化的耗时严格正比于帧数(实测约0.065秒/帧，1 vCPU/-O2)。
    // 相机升到120fps之后，同样"活动几秒"拿到的帧数直接翻倍：采集10秒就是
    // 1200帧(约80秒满载CPU)，采集30秒就是3600帧(约4分钟)——再叠加多路相机
    // 120fps采集+检测本身的CPU占用，整机会被拖到无响应。
    //
    // 关键是：多喂帧并不会更准。实测(合成数据、已知真值、0.3mm观测噪声)：
    //     30帧  -> anchor误差1.212mm  lengths误差2.543mm  耗时2.1s
    //    223帧  -> anchor误差1.430mm  lengths误差2.882mm  耗时14.7s
    //    400帧  -> anchor误差1.441mm  lengths误差3.024mm  耗时26.1s
    // 精度瓶颈在交替优化的局部极小值，不在数据量——同一段手指运动轨迹采
    // 30帧和采400帧，覆盖的角度多样性是一样的，多出来的帧只是重复信息。
    //
    // 所以这里均匀抽样(不是取前N帧——那样只会拿到运动的开头一小段，丢掉
    // 角度多样性，那才是真的会掉精度)，把计算量压到可接受范围。
    static constexpr int kMaxCalibFrames = 150;
    // 先把有效帧(>=2颗可见)挑出来再降采样——跟丢期间的垃圾帧(0~1颗可见)
    // 反正会被 calibrateFingerStructure 入口的 validCount 过滤掉，让它们
    // 占掉均匀抽样的名额纯属浪费(比如348帧里连续跟丢了几十帧，抽样按
    // 原始序列均匀取会把名额分给这些注定被扔掉的帧)。
    std::vector<int> validIdx;
    validIdx.reserve(capturedPositions_.size());
    for (int i = 0; i < int(capturedPositions_.size()); ++i) {
        const auto& v = capturedVisible_[size_t(i)];
        if (int(v[0]) + int(v[1]) + int(v[2]) >= 2) validIdx.push_back(i);
    }
    std::vector<std::array<std::array<double,3>,3>> capturedPositionsCopy;
    std::vector<std::array<bool,3>> capturedVisibleCopy;
    const int totalFrames = int(capturedPositions_.size());
    const int validFrames = int(validIdx.size());
    if (validFrames > kMaxCalibFrames) {
        capturedPositionsCopy.reserve(size_t(kMaxCalibFrames));
        capturedVisibleCopy.reserve(size_t(kMaxCalibFrames));
        for (int i = 0; i < kMaxCalibFrames; ++i) {
            // 均匀映射到有效帧序列，覆盖整段采集时间
            const size_t src = size_t(validIdx[size_t(qint64(i) * (validFrames - 1) / (kMaxCalibFrames - 1))]);
            capturedPositionsCopy.push_back(capturedPositions_[src]);
            capturedVisibleCopy.push_back(capturedVisible_[src]);
        }
    } else {
        capturedPositionsCopy.reserve(size_t(validFrames));
        capturedVisibleCopy.reserve(size_t(validFrames));
        for (int i : validIdx) {
            capturedPositionsCopy.push_back(capturedPositions_[size_t(i)]);
            capturedVisibleCopy.push_back(capturedVisible_[size_t(i)]);
        }
    }
    // 初值：优先用刚做过的静态标定结果(离真值近得多)，没有才退回默认
    // 模板。初值质量直接决定这套多重启交替优化需要几次重启才稳——实测
    // (合成数据150帧)：静态初值+2重启 与 模板初值+6重启 精度持平，耗时
    // 8.0s -> 2.6s。
    const bool useStaticInit = haveStaticParam_ && staticParamFingerIdx_ == fingerIdx;
    const FingerParam initialGuess = useStaticInit ? staticParam_ : fingerParam(fingerIdx);
    const int globalRestarts = useStaticInit ? 2 : 6;

    computingFingerIdx_ = fingerIdx;
    startBtn_->setEnabled(false);
    saveBtn_->setEnabled(false);
    fingerCombo_->setEnabled(false);   // 计算跑着的时候不让切手指，避免结果算完不知道该存到哪根手指头上
    // 如实告诉用户实际拿去算的是多少帧——采集状态那行显示的是"已采集N帧"
    // (原始帧数)，这里先剔除跟丢帧、再降采样，实际参与计算的数字可能小
    // 不少，不写清楚会让人以为丢数据了。
    if (validFrames > kMaxCalibFrames) {
        resultLabel_->setText(QStringLiteral(
            "结果：<font color='#e0a030'>计算中，请稍候…(采集%1帧，有效%2帧，均匀抽取%3帧参与计算"
            "——实测再多帧也不会更准，只会更慢；界面不会卡住)</font>")
            .arg(totalFrames).arg(validFrames).arg(kMaxCalibFrames));
    } else {
        resultLabel_->setText(QStringLiteral(
            "结果：<font color='#e0a030'>计算中，请稍候…(采集%1帧，有效%2帧，界面不会卡住)</font>")
            .arg(totalFrames).arg(validFrames));
    }

    QFuture<FingerCalibResult> future;
    if (fingerIdx == 0) {
        future = QtConcurrent::run([capturedPositionsCopy, capturedVisibleCopy, initialGuess, globalRestarts]() -> FingerCalibResult {
            std::vector<ThumbFrameObservation> frames(capturedPositionsCopy.size());
            for (size_t i = 0; i < capturedPositionsCopy.size(); ++i) {
                frames[i].markerPos = capturedPositionsCopy[i];
                frames[i].hasMarker = capturedVisibleCopy[i];
            }
            return calibrateThumbStructure(frames, initialGuess, 2, 12, globalRestarts);
        });
    } else {
        future = QtConcurrent::run([capturedPositionsCopy, capturedVisibleCopy, initialGuess, globalRestarts]() -> FingerCalibResult {
            std::vector<FingerFrameObservation> frames(capturedPositionsCopy.size());
            for (size_t i = 0; i < capturedPositionsCopy.size(); ++i) {
                frames[i].markerPos = capturedPositionsCopy[i];
                frames[i].hasMarker = capturedVisibleCopy[i];
            }
            // outerRounds 从12提到24：calibrateFingerStructure 现在多标一个
            // dipCoupling(DIP/PIP耦合系数，之前是写死的0.7常量)，这是个相对
            // 弱观测的方向(远节marker对它的敏感度不如对anchor/lengths那么
            // 直接)，合成数据验证过轮数越多收敛越贴近真值(12轮：0.7->0.66；
            // 40轮：0.7->0.61；真值0.55)，多给一点轮数明显有收益。这一步是
            // 后台异步算的("秒级到分钟级"，参考本函数上面注释)，多花的时间
            // 换更准的DIP系数划算。拇指没有DIP耦合这个概念(4个自由度都独立
            // 可观测)，不受影响，保持12不动。
            return calibrateFingerStructure(frames, initialGuess, 2, 24, globalRestarts);
        });
    }
    computeWatcher_->setFuture(future);
}

void FingerCalibPage::onComputeFinished() {
    const FingerCalibResult result = computeWatcher_->result();
    const bool isThumb = (computingFingerIdx_ == 0);

    haveComputedResult_ = result.valid;
    saveBtn_->setEnabled(result.valid);
    startBtn_->setEnabled(true);
    fingerCombo_->setEnabled(true);

    if (isThumb) pendingThumbResult_ = result;
    else pendingFingerResult_ = result;

    resultLabel_->setText(result.valid
        ? QStringLiteral("<font color='#3ec46d'>标定成功</font>——用了 %1 帧，拟合残差RMS %2 mm")
            .arg(result.framesUsed).arg(result.rmsMm, 0, 'f', 2)
        : QStringLiteral("<font color='#e05050'>标定未通过(残差RMS %1 mm，超过5mm阈值)</font>——建议重新采集，动作幅度更大一些")
            .arg(result.rmsMm, 0, 'f', 2));
}

void FingerCalibPage::onSaveClicked() {
    if (!haveComputedResult_ || !store_) return;
    const int fingerIdx = fingerCombo_->currentIndex();
    const FingerParam& p = fingerIdx == 0 ? pendingThumbResult_.param : pendingFingerResult_.param;
    store_->setFingerParam(fingerIdx, p);
    if (worker_) worker_->reloadTemplate();
    resultLabel_->setText(resultLabel_->text() + QStringLiteral("<br><b>已保存。</b>"));
}

void FingerCalibPage::refreshTable(const QVector<QVector3D>& worldPoints, const QVector<int>& ids,
                                   const QVector<int>& missedFrames) {
    fillCandidateTable(pointTable_, worldPoints, ids);
    // 高亮已经点选过的bootstrap点，跟BackTemplateCalibPage同一套配色语言——
    // 蓝色表示"已选中"，具体第几颗看上面的bootstrapHint_文字。
    for (int r = 0; r < pointTable_->rowCount(); ++r) {
        const int id = (r < ids.size()) ? ids[r] : -1;
        const bool selected = std::find(bootstrapSelectedIds_.begin(), bootstrapSelectedIds_.end(), id) != bootstrapSelectedIds_.end();
        const QColor bg = selected ? theme::selectBg() : theme::editorBg();
        for (int c = 0; c < 4; ++c) if (auto* it = pointTable_->item(r,c)) it->setBackground(bg);
    }
    scatter_->setPoints(worldPoints, ids, bootstrapSelectedIds_, -1, missedFrames);   // 这个页面没有forward hint点，传-1
}

// ---------------------------------------------------------------------------
// HandCalibrationWizard
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// AutoCalibPage("③ 全自动标定(实验性)"，高级选项)
// ---------------------------------------------------------------------------

AutoCalibPage::AutoCalibPage(HandTrackingWorker* worker, HandTemplateStore* store, QWidget* parent)
    : QDialog(parent), worker_(worker), store_(store) {
    setWindowFlags(Qt::Widget);

    instructionLabel_ = new QLabel(QStringLiteral(
        "<b>高级选项 · 全自动标定(实验性)</b><br>"
        "点「开始采集」后自然活动手指 30~60 秒，再点「停止采集」→「运行自动标定」。<br>"
        "<font color='#e0a030'>实验性：收敛质量不一定稳定，失败可重采，不影响①②的已有结果。</font>"), this);
    instructionLabel_->setWordWrap(true);

    captureStatusLabel_ = new QLabel(QStringLiteral("采集状态：未开始，已采集 0 帧"), this);

    startBtn_ = new QPushButton(QStringLiteral("开始采集"), this);
    stopBtn_ = new QPushButton(QStringLiteral("停止采集"), this);
    stopBtn_->setEnabled(false);
    runBtn_ = new QPushButton(QStringLiteral("运行自动标定"), this);
    runBtn_->setEnabled(false);
    saveBtn_ = new QPushButton(QStringLiteral("保存全部结果到模板(覆盖手背+全部5指)"), this);
    saveBtn_->setEnabled(false);

    connect(startBtn_, &QPushButton::clicked, this, &AutoCalibPage::onStartCaptureClicked);
    connect(stopBtn_, &QPushButton::clicked, this, &AutoCalibPage::onStopClicked);
    connect(runBtn_, &QPushButton::clicked, this, &AutoCalibPage::onRunCalibClicked);
    connect(saveBtn_, &QPushButton::clicked, this, &AutoCalibPage::onSaveClicked);

    resultView_ = new QTextEdit(this);
    resultView_->setReadOnly(true);
    resultView_->setPlaceholderText(QStringLiteral("运行结果会显示在这里……"));

    auto* btnRow = new QHBoxLayout();
    btnRow->addWidget(startBtn_);
    btnRow->addWidget(stopBtn_);
    btnRow->addWidget(runBtn_);
    btnRow->addWidget(saveBtn_);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(instructionLabel_);
    layout->addWidget(captureStatusLabel_);
    layout->addLayout(btnRow);
    layout->addWidget(resultView_, 1);

    if (worker_) {
        worker_->setCandidateBroadcastEnabled(true);
        connect(worker_, &HandTrackingWorker::candidatePointsReady, this, &AutoCalibPage::onCandidatePoints);
    }

    computeWatcher_ = new QFutureWatcher<HandAutoCalibOutput>(this);
    connect(computeWatcher_, &QFutureWatcher<HandAutoCalibOutput>::finished, this, &AutoCalibPage::onCalibFinished);
}

void AutoCalibPage::onCandidatePoints(QVector<QVector3D> worldPoints, QVector<int> ids, bool /*hasWristPose*/,
                                      QVector<double> /*wristRot9*/, QVector3D /*wristPos*/, qint64 /*ts_ns*/) {
    if (!capturing_) return;
    if (ids.size() != worldPoints.size()) return;   // 广播里ids/points长度对不上，诚实丢弃这一帧，不硬凑

    FrameTrackedPoints frame;
    frame.ids.reserve(ids.size());
    frame.positions.reserve(worldPoints.size());
    for (int i = 0; i < worldPoints.size(); ++i) {
        frame.ids.push_back(ids[i]);
        frame.positions.push_back({ double(worldPoints[i].x()), double(worldPoints[i].y()), double(worldPoints[i].z()) });
    }
    capturedFrames_.push_back(std::move(frame));
    captureStatusLabel_->setText(QStringLiteral("采集状态：进行中，已采集 %1 帧").arg(capturedFrames_.size()));
}

void AutoCalibPage::onStartCaptureClicked() {
    capturedFrames_.clear();
    capturing_ = true;
    startBtn_->setEnabled(false);
    stopBtn_->setEnabled(true);
    runBtn_->setEnabled(false);
    saveBtn_->setEnabled(false);
    haveResult_ = false;
    captureStatusLabel_->setText(QStringLiteral("采集状态：进行中，已采集 0 帧"));
    resultView_->clear();
}

void AutoCalibPage::onStopClicked() {
    capturing_ = false;
    startBtn_->setEnabled(true);
    stopBtn_->setEnabled(false);
    runBtn_->setEnabled(int(capturedFrames_.size()) >= 30);   // 太少帧直接不给点，省得跑一遍才告诉你不够
    captureStatusLabel_->setText(QStringLiteral("采集状态：已停止，共 %1 帧%2")
        .arg(capturedFrames_.size())
        .arg(int(capturedFrames_.size()) < 30 ? QStringLiteral("——帧数偏少，建议重新采集(至少几百帧、覆盖更多动作)") : QString()));
}

void AutoCalibPage::onRunCalibClicked() {
    // 这条链路是多阶段优化(刚体聚类+分组+轴系对齐+每根链的运动学拟合)，
    // 数据量一大同样是秒级到分钟级的重计算——跟手指结构手动标定那次
    // "点了就卡死"是完全一样的教训，必须丢到后台线程跑，不能同步调用。
    runBtn_->setEnabled(false);
    startBtn_->setEnabled(false);
    resultView_->setPlainText(QStringLiteral("计算中，请稍候……(可能需要几秒到一分钟，界面不会卡住)"));

    const auto framesCopy = capturedFrames_;   // 拷贝隔离，后台线程不碰UI线程可能还在改动的成员

    QFuture<HandAutoCalibOutput> future = QtConcurrent::run([framesCopy]() -> HandAutoCalibOutput {
        HandAutoCalibConfig cfg;
        cfg.useTier1TimeWindows = false;   // 用Tier2(运动相关性)自动分组——高级选项不该还要求用户预先规划5个时间窗口，那样就不"全自动"了
        return runHandAutoCalibPipeline(framesCopy, framesCopy, cfg);
    });
    computeWatcher_->setFuture(future);
}

void AutoCalibPage::onCalibFinished() {
    const HandAutoCalibOutput result = computeWatcher_->result();
    haveResult_ = result.valid;
    pendingResult_ = result;
    startBtn_->setEnabled(true);
    saveBtn_->setEnabled(result.valid);

    if (!result.valid) {
        resultView_->setPlainText(QStringLiteral("标定失败：%1\n\n建议：重新采集，尽量让全部marker持续可见、每根手指都有明显的屈伸动作，"
                                                  "避免多根手指同时快速晃动(会干扰自动分组)。")
            .arg(QString::fromStdString(result.message)));
        return;
    }

    QString text;
    text += QStringLiteral("标定成功。\n\n");
    text += QStringLiteral("手背刚体：最大点对距离标准差 %1 mm(越小说明这组点越像刚体，即误判为手背的可能性越低)\n")
        .arg(result.rigidMaxPairStdMm, 0, 'f', 2);
    static const QString names[4] = {QStringLiteral("食指"), QStringLiteral("中指"), QStringLiteral("无名指"), QStringLiteral("小指")};
    for (int i = 0; i < 4; ++i)
        text += QStringLiteral("%1：拟合残差RMS %2 mm\n").arg(names[i]).arg(result.fingerRmsMm[size_t(i)], 0, 'f', 2);
    text += QStringLiteral("拇指：拟合残差RMS %1 mm，轴系对齐置信度分数 %2\n")
        .arg(result.thumbRmsMm, 0, 'f', 2).arg(result.axisAlignThumbScore, 0, 'f', 2);
    resultView_->setPlainText(text);
}

void AutoCalibPage::onSaveClicked() {
    if (!haveResult_ || !store_) return;

    std::array<HandVec3,5> back{};
    for (int i=0;i<5;++i) back[size_t(i)] = { pendingResult_.backMarkers[size_t(i)][0],
                                             pendingResult_.backMarkers[size_t(i)][1],
                                             pendingResult_.backMarkers[size_t(i)][2] };
    store_->setBackTemplate(back);

    // fingerParams[0..3] = 食/中/无名/小 -> HandTemplateStore的fingerIdx 1..4
    // (0号槽位是拇指，见HandTemplateStore::setFingerParam注释)。
    for (int i=0;i<4;++i) {
        FingerParam p;
        p.anchor = { pendingResult_.fingerParams[size_t(i)].anchor[0],
                    pendingResult_.fingerParams[size_t(i)].anchor[1],
                    pendingResult_.fingerParams[size_t(i)].anchor[2] };
        p.lengths = pendingResult_.fingerParams[size_t(i)].lengths;
        p.dipCoupling = pendingResult_.fingerParams[size_t(i)].dipCoupling;
        store_->setFingerParam(1 + i, p);
    }

    // 拇指：baseRotZ丢弃(见上面的已知局限说明)，只存anchor+lengths。
    {
        FingerParam p;
        p.anchor = { pendingResult_.thumbParams.anchor[0], pendingResult_.thumbParams.anchor[1], pendingResult_.thumbParams.anchor[2] };
        p.lengths = pendingResult_.thumbParams.lengths;
        store_->setFingerParam(0, p);
    }

    if (worker_) worker_->reloadTemplate();
    resultView_->append(QStringLiteral("\n<b><font color='#3ec46d'>已保存全部结果到模板(手背+全部5指)。</font></b>"));
}

// ---------------------------------------------------------------------------
// HandCalibrationWizard
// ---------------------------------------------------------------------------

HandCalibrationWizard::HandCalibrationWizard(HandTrackingWorker* worker, HandTemplateStore* store, QWidget* parent)
    : QDialog(parent), worker_(worker), store_(store) {
    setWindowTitle(QStringLiteral("手部标定向导"));
    resize(720, 640);

    tabs_ = new QTabWidget(this);
    backPage_ = new BackTemplateCalibPage(worker_, store_, this);
    fingerPage_ = new FingerCalibPage(worker_, store_, this);
    autoPage_ = new AutoCalibPage(worker_, store_, this);

    tabs_->addTab(backPage_, QStringLiteral("① 手背模板"));
    tabs_->addTab(fingerPage_, QStringLiteral("② 手指结构"));
    tabs_->addTab(autoPage_, QStringLiteral("③ 全自动标定(高级)"));

    backStatusLabel_ = new QLabel(QStringLiteral("手背模板：<font color='#e0a030'>未保存</font>"), this);

    // 跳过按钮放在最外层(不在任何一个tab内部)——不管你切到①还是②，它
    // 都在同一个位置，点了直接关闭整个向导，不用你先切回①去找它、也
    // 不用担心"保存和跳过是不是同一次操作"这种疑虑：保存是保存(点手背
    // 页里的"保存手背模板")、跳过是跳过(这个按钮)，两个动作分开、互不
    // 依赖对方的可见性。
    skipFingerBtn_ = new QPushButton(QStringLiteral("跳过手指标定，关闭向导去测试"), this);
    skipFingerBtn_->setEnabled(false);
    connect(skipFingerBtn_, &QPushButton::clicked, this, &QDialog::accept);

    // 手背模板存完之后，只更新状态提示+解锁跳过按钮，不再强制切到②——
    // 切不切、什么时候切，交给用户自己决定，不要替他做这个选择。
    connect(backPage_, &BackTemplateCalibPage::backTemplateCalibrated, this, [this]{
        backStatusLabel_->setText(QStringLiteral("手背模板：<font color='#3ec46d'>已保存 ✓</font>"));
        skipFingerBtn_->setEnabled(true);
    });

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(backStatusLabel_);
    layout->addWidget(tabs_);

    auto* bottomRow = new QHBoxLayout();
    auto* closeBtn = new QPushButton(QStringLiteral("关闭"), this);
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
    bottomRow->addWidget(skipFingerBtn_);
    bottomRow->addStretch(1);
    bottomRow->addWidget(closeBtn);
    layout->addLayout(bottomRow);
}

HandCalibrationWizard::~HandCalibrationWizard() {
    // 向导关闭后停掉候选点广播——正常追踪不需要这份额外开销。
    if (worker_) worker_->setCandidateBroadcastEnabled(false);
}

} // namespace mocap
