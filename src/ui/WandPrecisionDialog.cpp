#include "ui/WandPrecisionDialog.hpp"
#include "ui/Theme.hpp"
#include "camera/CameraManager.hpp"
#include "camera/ICamera.hpp"
#include "calib/CalibrationStore.hpp"
#include "reconstruct/Triangulation.hpp"      // undistortNormalize
#include "reconstruct/MultiViewCluster.hpp"   // clusterMultiView(多点关联+LM+IRLS精修)
#include "reconstruct/TemporalTracker.hpp"    // 跨帧稳定ID，完整类型在这里才可见(头文件里只前向声明了)
#include "estimate/RadiusRefine.hpp"          // 深度反推半径——圆拟合/融合模式的两遍流程要用

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QTextEdit>
#include <cmath>

namespace mocap {

WandPrecisionDialog::WandPrecisionDialog(CameraManager* mgr, CalibrationStore* store, bool wasDetectOn, QWidget* parent)
    : QDialog(parent), mgr_(mgr), store_(store), wasDetectOn_(wasDetectOn) {
    setWindowTitle(QStringLiteral("标定杆精度验证"));
    resize(900, 720);

    // 【布局】跟点云测试/三角化调试同一套：上面是设置、中间是结果（占满剩余
    // 空间）、底部一条常驻读数条。statsView_ 才是这个窗口真正要看的东西，
    // 原来它跟一堆 spinbox 平摊高度，结果长一点就得滚。
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(0);

    auto* ctlHost = new QWidget;
    auto* ctl = new QVBoxLayout(ctlHost);
    ctl->setContentsMargins(10, 8, 10, 8);
    ctl->setSpacing(6);

    auto* statusHost = new QWidget;
    statusHost->setObjectName(QStringLiteral("panelStatusBar"));
    statusHost->setAttribute(Qt::WA_StyledBackground, true);
    auto* statusStrip = new QHBoxLayout(statusHost);
    statusStrip->setContentsMargins(10, 4, 10, 4);

    auto* hint = new QLabel(QStringLiteral(
        "真值 = 两颗球<b>球心</b>之间的距离（不是杆长、不是球面到球面）。\n"
        "在捕捉空间到处挥动，覆盖整个工作距离和不同姿态。\n"
        "<font color='#e0a030'>捕捉空间里必须只有这 2 个候选点；点数不对的帧会被跳过。</font>"), this);
    hint->setWordWrap(true);
    ctl->addWidget(hint);

    ctl->addWidget(new QLabel(QStringLiteral("勾选参与验证的相机（>=2 台已标定）：")));
    camList_ = new QListWidget(this);
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        if (c->deviceKey() == QStringLiteral("virt")) continue;
        const bool calibrated = store_->get(c->deviceKey()).isCalibrated();
        auto* item = new QListWidgetItem(
            QString("%1  %2").arg(calibrated ? QStringLiteral("●") : QStringLiteral("○"), c->name()), camList_);
        item->setData(Qt::UserRole, c->id());
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(calibrated ? Qt::Checked : Qt::Unchecked);
        if (!calibrated) item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }
    camList_->setMaximumHeight(140);
    ctl->addWidget(camList_);
    connect(camList_, &QListWidget::itemChanged, this, [this](QListWidgetItem*) { onCamerasChanged(); });

    // ---- 定位模式：跟 HandTrackingWorker 共用同一套 BlobLocalizationMode ----
    auto* modeRow = new QHBoxLayout();
    modeRow->addWidget(new QLabel(QStringLiteral("定位模式："), this));
    modeCombo_ = new QComboBox(this);
    modeCombo_->addItem(QStringLiteral("质心法(不需要球半径，最简单)"));
    modeCombo_->addItem(QStringLiteral("圆拟合法(需要球半径，抗遮挡)"));
    modeCombo_->addItem(QStringLiteral("融合(圆拟合可用就融合，否则退回质心)"));
    modeRow->addWidget(modeCombo_);
    modeRow->addStretch(1);
    ctl->addLayout(modeRow);
    connect(modeCombo_, &QComboBox::currentIndexChanged, this, &WandPrecisionDialog::onModeChanged);

    auto* radiusRow = new QHBoxLayout();
    radiusLabel_ = new QLabel(QStringLiteral("球物理半径(mm)："), this);
    radiusSpin_ = new QDoubleSpinBox(this);
    radiusSpin_->setRange(0.5, 30.0);
    radiusSpin_->setDecimals(2);
    radiusSpin_->setValue(physicalRadiusMm_);
    radiusRow->addWidget(radiusLabel_);
    radiusRow->addWidget(radiusSpin_);
    radiusRow->addStretch(1);
    ctl->addLayout(radiusRow);
    connect(radiusSpin_, &QDoubleSpinBox::valueChanged, this, &WandPrecisionDialog::onPhysicalRadiusChanged);
    radiusLabel_->setEnabled(false);   // 默认质心法，不需要半径——跟着模式切换enable/disable
    radiusSpin_->setEnabled(false);

    auto* lengthRow = new QHBoxLayout();
    lengthRow->addWidget(new QLabel(QStringLiteral("杆子真实长度(两球球心距, mm)："), this));
    trueLengthSpin_ = new QDoubleSpinBox(this);
    trueLengthSpin_->setRange(10.0, 2000.0);
    trueLengthSpin_->setDecimals(2);
    trueLengthSpin_->setValue(trueLengthMm_);
    lengthRow->addWidget(trueLengthSpin_);
    lengthRow->addStretch(1);
    ctl->addLayout(lengthRow);
    connect(trueLengthSpin_, &QDoubleSpinBox::valueChanged, this, &WandPrecisionDialog::onTrueLengthChanged);

    auto* btnRow = new QHBoxLayout();
    startStopBtn_ = new QPushButton(QStringLiteral("开始统计"), this);
    resetBtn_ = new QPushButton(QStringLiteral("重置统计"), this);
    btnRow->addWidget(startStopBtn_);
    btnRow->addWidget(resetBtn_);
    btnRow->addStretch(1);
    ctl->addLayout(btnRow);
    connect(startStopBtn_, &QPushButton::clicked, this, &WandPrecisionDialog::onStartStopClicked);
    connect(resetBtn_, &QPushButton::clicked, this, &WandPrecisionDialog::onResetClicked);

    v->addWidget(ctlHost);

    statsView_ = new QTextEdit(this);
    statsView_->setReadOnly(true);
    statsView_->setPlaceholderText(QStringLiteral("统计结果会显示在这里……"));
    // 结果是一堆数字，等宽才对得齐；这里是这个窗口的主角，给它全部剩余空间。
    statsView_->setStyleSheet(theme::monoCss(12.5));
    v->addWidget(statsView_, 1);

    statusLabel_ = new QLabel(QStringLiteral("勾选相机后开始"), this);
    statusStrip->addWidget(statusLabel_);
    statusStrip->addStretch(1);
    v->addWidget(statusHost);

    batchTimer_ = new QTimer(this);
    batchTimer_->setSingleShot(true);
    connect(batchTimer_, &QTimer::timeout, this, &WandPrecisionDialog::onBatchTimerFire);

    accumulator_.emplace(trueLengthMm_);
    // minHitsToConfirm=3：连续3帧确认后才发布，多视角聚类偶发的幽灵点
    // (通常只活1~2帧)进不了统计——否则一个幽灵闪现会让"恰好2个候选点"
    // 的帧判定失真。maxMissedFrames 保持小(5)：这里跟标定向导不同，遮挡
    // 记忆期间轨迹位置是冻结的旧值，量距场景宁可跳过这一帧也不能拿旧
    // 位置充数(下面消费处还额外按 missedFrames==0 过滤了一道)。
    tracker_ = new TemporalTracker(/*maxAssocDist=*/20.0, /*maxMissedFrames=*/5, /*minHitsToConfirm=*/3);

    rebuildCameraSetup();
}

WandPrecisionDialog::~WandPrecisionDialog() {
    if (!wasDetectOn_)
        for (ICamera* c : weTurnedOnDetect_) c->setDetectEnabled(false);
    // 轮廓采集不能像检测开关那样简单粗暴地"全关"——别的消费者(比如手部
    // 追踪)可能也需要它，只关掉本窗口自己打开过的那几台。
    for (ICamera* c : weTurnedOnContour_) c->setContourCollectionEnabled(false);
    delete tracker_;
}

void WandPrecisionDialog::onCamerasChanged() {
    rebuildCameraSetup();
}

void WandPrecisionDialog::onModeChanged(int idx) {
    localizationMode_ = idx==0 ? BlobLocalizationMode::CentroidOnly
                       : idx==1 ? BlobLocalizationMode::CircleFitOnly
                                 : BlobLocalizationMode::Fused;
    const bool needsRadius = (localizationMode_ != BlobLocalizationMode::CentroidOnly);
    radiusLabel_->setEnabled(needsRadius);
    radiusSpin_->setEnabled(needsRadius);
    rebuildCameraSetup();   // 圆拟合/融合需要轮廓采集，质心法不需要——模式切换要跟着重新决定要不要开轮廓采集
}

void WandPrecisionDialog::onPhysicalRadiusChanged(double mm) {
    physicalRadiusMm_ = mm;
}

void WandPrecisionDialog::rebuildCameraSetup() {
    // 断开之前那批相机的连接，避免同一个blobDetailsReady被重复连接多次。
    for (int i = 0; i < mgr_->count(); ++i)
        disconnect(mgr_->at(i), &ICamera::blobDetailsReady, this, &WandPrecisionDialog::onBlobDetailsReady);

    camIds_.clear(); camR_.clear(); camT_.clear(); camIntr_.clear();
    latestBlobs_.clear();

    QVector<ICamera*> chosen;
    for (int i = 0; i < camList_->count(); ++i) {
        QListWidgetItem* item = camList_->item(i);
        if (item->checkState() != Qt::Checked) continue;
        if (ICamera* c = mgr_->byId(item->data(Qt::UserRole).toUInt())) chosen << c;
    }

    // 自动开检测，同 TriangulationDebugDialog 的做法：不然blobDetailsReady
    // 永远不会来。
    if (!wasDetectOn_) {
        for (auto it = weTurnedOnDetect_.begin(); it != weTurnedOnDetect_.end(); ) {
            if (!chosen.contains(*it)) { (*it)->setDetectEnabled(false); it = weTurnedOnDetect_.erase(it); }
            else ++it;
        }
        for (ICamera* c : chosen) {
            if (!weTurnedOnDetect_.contains(c)) { c->setDetectEnabled(true); weTurnedOnDetect_.insert(c); }
        }
    }

    // 轮廓采集：圆拟合/融合模式才需要，质心法不需要(省flood
    // fill多记轮廓的开销)。只帮"这次真的需要、且当前没开"的相机打开，
    // 只关掉"本窗口自己打开过、现在不再需要"的那几台——不动本来就因为
    // 别的消费者而开着的相机，避免关掉了别人还在用的东西。
    const bool needsContour = (localizationMode_ != BlobLocalizationMode::CentroidOnly);
    for (auto it = weTurnedOnContour_.begin(); it != weTurnedOnContour_.end(); ) {
        if (!needsContour || !chosen.contains(*it)) { (*it)->setContourCollectionEnabled(false); it = weTurnedOnContour_.erase(it); }
        else ++it;
    }
    if (needsContour) {
        for (ICamera* c : chosen) {
            if (!c->contourCollectionEnabled()) { c->setContourCollectionEnabled(true); weTurnedOnContour_.insert(c); }
        }
    }

    if (chosen.size() < 2) {
        statusLabel_->setText(QStringLiteral("至少勾选两台已标定的相机"));
        return;
    }

    for (ICamera* c : chosen) {
        const CameraCalibration cal = store_->get(c->deviceKey());
        camIds_.push_back(c->id());
        camR_.push_back(cal.extr.R);
        camT_.push_back(cal.extr.t);
        camIntr_.push_back(cal.intr);
        connect(c, &ICamera::blobDetailsReady, this, &WandPrecisionDialog::onBlobDetailsReady);
    }

    statusLabel_->setText(QStringLiteral("已就绪：%1 台相机参与，拿杆子在共视区挥动").arg(chosen.size()));
}

void WandPrecisionDialog::onBlobDetailsReady(quint32 camId, const QVector<Blob>& blobs, qint64 /*ts_ns*/) {
    bool tracked = false;
    for (quint32 id : camIds_) if (id == camId) { tracked = true; break; }
    if (!tracked) return;

    latestBlobs_[camId] = blobs;
    if (!batchTimer_->isActive()) batchTimer_->start(kBatchWindowMs);
}

void WandPrecisionDialog::onBatchTimerFire() {
    processBatch();
}

void WandPrecisionDialog::processBatch() {
    if (camIds_.size() < 2) return;

    std::vector<EpiMat3> Rs; std::vector<EpiVec3> ts;
    for (size_t ci = 0; ci < camIds_.size(); ++ci) { Rs.push_back(camR_[ci]); ts.push_back(camT_[ci]); }

    // 第一遍：不管选的是哪个模式，都先用原始质心(Blob::cx/cy，不需要
    // 轮廓)做一次radius-free粗略三角化，拿到每个候选点的粗略深度——
    // 圆拟合/融合模式反推半径要用它；质心模式这一遍的结果直接就是最终
    // 结果(多算一次对调试工具来说不是问题，不追求极致性能)。
    std::vector<std::vector<std::array<double,2>>> coarseObsPerCam(camIds_.size());
    for (size_t ci = 0; ci < camIds_.size(); ++ci) {
        const auto it = latestBlobs_.constFind(camIds_[ci]);
        if (it == latestBlobs_.constEnd()) continue;
        for (const Blob& b : it.value()) {
            double nx, ny;
            undistortNormalize(camIntr_[ci], double(b.cx), double(b.cy), nx, ny);
            coarseObsPerCam[ci].push_back({nx, ny});
        }
    }

    ClusterResult finalCluster;

    if (localizationMode_ == BlobLocalizationMode::CentroidOnly) {
        finalCluster = clusterMultiView(Rs, ts, coarseObsPerCam, 0.01, 0.01, /*useLmRefine=*/true);
    } else {
        const auto coarseCluster = clusterMultiView(Rs, ts, coarseObsPerCam, 0.02, 0.02, /*useLmRefine=*/false);
        std::vector<std::array<double,3>> coarsePoints;
        for (const auto& tr : coarseCluster.tracks) coarsePoints.push_back(tr.point);

        std::vector<RfVec3> rfPoints;
        rfPoints.reserve(coarsePoints.size());
        for (const auto& p : coarsePoints) rfPoints.push_back({p[0], p[1], p[2]});

        std::vector<std::vector<std::array<double,2>>> refinedObsPerCam(camIds_.size());
        for (size_t ci = 0; ci < camIds_.size(); ++ci) {
            const auto it = latestBlobs_.constFind(camIds_[ci]);
            if (it == latestBlobs_.constEnd()) continue;

            RfMat3 R; RfVec3 t;
            for (int i=0;i<9;++i) R[size_t(i)] = camR_[ci][size_t(i)];
            for (int i=0;i<3;++i) t[size_t(i)] = camT_[ci][size_t(i)];

            for (const Blob& b : it.value()) {
                double nx, ny;
                undistortNormalize(camIntr_[ci], double(b.cx), double(b.cy), nx, ny);

                const auto rr = refineExpectedRadius(nx, ny, R, t, camIntr_[ci].fx, rfPoints, physicalRadiusMm_, 0.03);
                const double knownRadiusPx = rr.matched ? rr.expectedRadiusPx
                                                        : physicalRadiusMm_ * camIntr_[ci].fx / fallbackWorkingDistanceMm_;

                BlobObservationConfig cfg;
                cfg.knownRadiusPx = knownRadiusPx;
                cfg.mode = localizationMode_;
                cfg.centroidSigmaPx = 0.3;
                const auto obsList = blobToObservations(b, camIntr_[ci], cfg);
                for (const auto& o : obsList) refinedObsPerCam[ci].push_back({o.nx, o.ny});
            }
        }
        finalCluster = clusterMultiView(Rs, ts, refinedObsPerCam, 0.01, 0.01, /*useLmRefine=*/true);
    }

    const auto tracked = tracker_->update(finalCluster.tracks);

    ++frameCounter_;

    if (int(tracked.size()) != 2) {
        statusLabel_->setText(QStringLiteral("候选点数量=%1(需要恰好2个)——本帧跳过，检查场景里是不是只有这根杆子")
            .arg(tracked.size()));
        return;
    }

    // 【修过的隐性bug】遮挡记忆(coasting)期间轨迹的 position 是"最后一次
    // 真正看到时"的冻结旧值——拿它量距会把陈旧位置混进精度统计，测出来
    // 的误差不是系统真实精度。两个点都必须是这一帧真正看到的才计入。
    if (tracked[0].missedFrames != 0 || tracked[1].missedFrames != 0) {
        statusLabel_->setText(QStringLiteral("有球处于遮挡记忆状态(位置是旧值)——本帧跳过，不污染统计"));
        return;
    }

    const auto& a = tracked[0].position;
    const auto& b = tracked[1].position;
    const double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    const double dist = std::sqrt(dx*dx+dy*dy+dz*dz);

    statusLabel_->setText(QStringLiteral("当前测得距离：%1 mm（真值 %2 mm，误差 %3 mm）%4")
        .arg(dist, 0, 'f', 3).arg(trueLengthMm_, 0, 'f', 2).arg(dist-trueLengthMm_, 0, 'f', 3)
        .arg(running_ ? QStringLiteral("[统计中]") : QStringLiteral("[未开始统计，点“开始统计”]")));

    if (running_ && accumulator_) {
        accumulator_->addDistanceSample(dist, frameCounter_);
        refreshStatsDisplay();
    }
}

void WandPrecisionDialog::onStartStopClicked() {
    running_ = !running_;
    startStopBtn_->setText(running_ ? QStringLiteral("停止统计") : QStringLiteral("开始统计"));
}

void WandPrecisionDialog::onResetClicked() {
    accumulator_.emplace(trueLengthMm_);
    // TemporalTracker 没有暴露 reset() 方法(公开接口只有构造函数+update())，
    // 用删掉重新 new 一个达到同样效果，不需要为了这一个用途去改动那个
    // 已经独立测过的文件。
    delete tracker_;
    tracker_ = new TemporalTracker(20.0, 5, 3);
    refreshStatsDisplay();
}

void WandPrecisionDialog::onTrueLengthChanged(double mm) {
    trueLengthMm_ = mm;
    // 真值变了，之前攒的样本是按旧真值算的误差，混在一起没有意义——
    // 直接重置，明确告诉用户为什么清空了，不要悄悄让新旧真值的样本
    // 混在同一份统计里。
    accumulator_.emplace(trueLengthMm_);
    statusLabel_->setText(QStringLiteral("真实长度改成了 %1 mm，之前的统计已清空重新开始").arg(mm, 0, 'f', 2));
    refreshStatsDisplay();
}

void WandPrecisionDialog::refreshStatsDisplay() {
    if (!accumulator_) return;
    const auto rep = accumulator_->report();
    if (!rep.valid) {
        statsView_->setPlainText(QStringLiteral("(还没有样本)"));
        return;
    }
    QString text;
    text += QStringLiteral("样本数：%1\n").arg(rep.sampleCount);
    text += QStringLiteral("真值：%1 mm\n").arg(rep.trueLengthMm, 0, 'f', 2);
    text += QStringLiteral("测量均值：%1 mm\n").arg(rep.meanMeasuredMm, 0, 'f', 3);
    text += QStringLiteral("系统性偏置(bias)：%1 mm  %2\n").arg(rep.biasMm, 0, 'f', 3)
        .arg(std::abs(rep.biasMm) > 1.0 ? QStringLiteral("⚠ 明显偏大/偏小，可能是标定/半径假设有系统性偏差") : QString());
    text += QStringLiteral("标准差(纯随机噪声)：%1 mm\n").arg(rep.stdDevMm, 0, 'f', 3);
    text += QStringLiteral("<b>RMS误差(最终精度指标)：%1 mm</b>\n").arg(rep.rmsErrorMm, 0, 'f', 3);
    text += QStringLiteral("最大绝对误差：%1 mm(第%2帧附近)\n").arg(rep.maxAbsErrorMm, 0, 'f', 3).arg(rep.maxAbsErrorTs);
    statsView_->setHtml(text.replace("\n", "<br>"));
}

} // namespace mocap
