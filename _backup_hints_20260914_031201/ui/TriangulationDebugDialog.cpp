#include "ui/TriangulationDebugDialog.hpp"
#include "ui/Theme.hpp"
#include "ui/TriangulationView.hpp"
#include "camera/CameraManager.hpp"
#include "camera/ICamera.hpp"
#include "calib/CalibrationStore.hpp"
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <cmath>

namespace mocap {

namespace {
// 深色底上的三档横幅配色。原来是散在各处的手写十六进制（#6B1F1F/#1E5A3A/…），
// 现在统一从 Theme 取，改配色只改一个地方。
QString bannerCss(const QColor& bgc, const QColor& fg) {
    return QStringLiteral("padding:6px 8px; border-radius:2px; border-left:2px solid %1;"
                          "background:%2; color:%3;")
        .arg(theme::hex(fg), theme::hex(bgc), theme::hex(fg));
}
QString okBanner()   { return bannerCss(theme::okBannerBg(),   theme::okBannerFg()); }
QString warnBanner() { return bannerCss(theme::warnBannerBg(), theme::warnBannerFg()); }
QString badBanner()  { return bannerCss(theme::badBannerBg(),  theme::badBannerFg()); }
} // namespace

TriangulationDebugDialog::TriangulationDebugDialog(CameraManager* mgr, CalibrationStore* store,
                                                   bool wasDetectOn, QWidget* parent)
    : QDialog(parent), mgr_(mgr), store_(store), wasDetectOn_(wasDetectOn) {
    setWindowTitle(QStringLiteral("三角化调试（N 相机自适应）"));
    resize(1000, 780);

    // 【布局】跟点云测试面板同一套语言：3D 视图是主角、占满剩余空间，读数类
    // 标签（状态/抖动/跳变诊断）全部沉到底部一条常驻读数条上，不再夹在控件
    // 中间跟着一起往下挤。控件区自己收紧，不跟视图抢高度。
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(0);

    auto* ctlHost = new QWidget;
    auto* ctl = new QVBoxLayout(ctlHost);
    ctl->setContentsMargins(10, 8, 10, 8);
    ctl->setSpacing(6);

    // 底部读数条：objectName 挂上就自动拿到等宽字体 + 深底 + 顶部强调线，
    // 样式统一定义在 Theme.hpp，不在这里写内联样式。
    auto* statusHost = new QWidget;
    statusHost->setObjectName(QStringLiteral("panelStatusBar"));
    statusHost->setAttribute(Qt::WA_StyledBackground, true);
    auto* statusStrip = new QVBoxLayout(statusHost);
    statusStrip->setContentsMargins(10, 4, 10, 4);
    statusStrip->setSpacing(2);

    ctl->addWidget(new QLabel(QStringLiteral("勾选参与三角化的相机（>=2 台，任意两台看到即可解算，看到越多越准）：")));

    list_ = new QListWidget;
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        if (c->deviceKey() == QStringLiteral("virt")) continue;   // 虚拟相机无标定
        const bool calibrated = store_->get(c->deviceKey()).isCalibrated();
        auto* item = new QListWidgetItem(
            QString("%1  %2").arg(calibrated ? QStringLiteral("●") : QStringLiteral("○"), c->name()),
            list_);
        item->setData(Qt::UserRole, c->id());
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(calibrated ? Qt::Checked : Qt::Unchecked);   // 已标定的默认勾上
        if (!calibrated) item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }
    list_->setMaximumHeight(120);
    ctl->addWidget(list_);

    // ---- 精度算法开关：逐个测试用 ----
    // 三个都默认勾上（跟 Triangulator 头文件里的默认值一致：全开）。取消
    // 勾选立即生效（不用重新勾相机、不丢已积累的残差滑动窗口），配合下面
    // 的抖动统计，关一个、重置一次统计、看数字变化，就是最直接的对比。
    auto* algoRow = new QHBoxLayout;
    algoRow->addWidget(new QLabel(QStringLiteral("精度算法：")));
    chkInterp_ = new QCheckBox(QStringLiteral("①时间插值补偿"));
    chkInterp_->setChecked(true);
    chkInterp_->setToolTip(QStringLiteral(
        "把各相机的观测对齐到统一基准时刻再三角化，消除跟球速成正比的\n"
        "时间错位误差。关掉后退回逐相机各用各自最近一次观测（老行为）。"));
    algoRow->addWidget(new QLabel(QStringLiteral("②鲁棒模式：")));
    robustModeCombo_ = new QComboBox;
    robustModeCombo_->addItem(QStringLiteral("关闭"), int(Triangulator::RobustMode::Off));
    robustModeCombo_->addItem(QStringLiteral("硬剔除(会跳变)"), int(Triangulator::RobustMode::Hard));
    robustModeCombo_->addItem(QStringLiteral("软加权IRLS(推荐)"), int(Triangulator::RobustMode::Soft));
    robustModeCombo_->setCurrentIndex(2);   // 默认软加权
    robustModeCombo_->setToolTip(QStringLiteral(
        "硬剔除：跟大多数视角不一致的坏视角直接踢出解算，视角集合切换时\n"
        "解出的3D点会有台阶(悬崖/小跳变)。\n"
        "软加权(IRLS)：同样能压掉离群视角的影响，但用连续权重代替离散\n"
        "剔除，视角质量变化时权重平滑过渡，不会有台阶——推荐默认用这个，\n"
        "切到硬剔除只是为了对比看差异。"));
    chkFilter_ = new QCheckBox(QStringLiteral("③输出滤波(One Euro)"));
    chkFilter_->setChecked(true);
    chkFilter_->setToolTip(QStringLiteral(
        "对最终3D点做自适应低通：慢速多平滑、快速不拖尾。关掉后输出的是\n"
        "三角化的原始点，没有任何平滑处理。"));
    algoRow->addWidget(chkInterp_);
    algoRow->addWidget(robustModeCombo_);
    algoRow->addWidget(chkFilter_);
    algoRow->addStretch(1);
    ctl->addLayout(algoRow);

    auto onAlgoToggled = [this] { applyAlgoOptions(); resetJitterStats(); };
    connect(chkInterp_, &QCheckBox::toggled, this, onAlgoToggled);
    connect(robustModeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, onAlgoToggled);
    connect(chkFilter_, &QCheckBox::toggled, this, onAlgoToggled);

    // ---- 抖动统计：效果对比用 ----
    auto* jitterRow = new QHBoxLayout;
    jitterLabel_ = new QLabel(QStringLiteral("抖动：样本不足（拿标定物悬空静止几秒）"));
    resetJitterBtn_ = new QPushButton(QStringLiteral("重置统计"));
    resetJitterBtn_->setToolTip(QStringLiteral(
        "拿反光球/标定物悬空保持不动，点这个清零重新统计——上面的数值是\n"
        "最近约2秒输出点的位置标准差(mm)，球没动的话这个数就是纯抖动量。\n"
        "关掉某个开关、点一次这个、等数字稳定下来，跟关之前的数对比，\n"
        "就能看出这一项到底有没有用、有多大用。"));
    connect(resetJitterBtn_, &QPushButton::clicked, this, &TriangulationDebugDialog::resetJitterStats);
    jitterRow->addWidget(jitterLabel_, 1);
    jitterRow->addWidget(resetJitterBtn_);
    ctl->addLayout(jitterRow);

    // 匹配情况反馈：模板里登记了几台已标定相机，当前接入的相机里实际
    // 匹配上几台。三种情况：全部匹配（不用提示）、部分匹配（琥珀色，
    // 说明用哪几台够用就继续，不阻塞）、一台都没匹配上（红色，阻塞）。
    // 注意"部分匹配"不强求台数相等——只要匹配上的 >=2 台，照样能用，
    // 这是有意的设计：模板里的相机数不是必须凑齐的门槛，能对上几台就
    // 用几台。
    int templateTotal = 0;
    for (const QString& k : store_->keys())
        if (store_->get(k).isCalibrated()) ++templateTotal;

    int matched = 0;
    for (int i = 0; i < list_->count(); ++i)
        if (list_->item(i)->flags() & Qt::ItemIsEnabled) ++matched;

    if (matched == 0 && list_->count() > 0) {
        auto* warn = new QLabel(QStringLiteral(
            "⚠ 当前接入的相机，没有一台跟这个标定模板里的记录对得上号——"
            "可能是相机被重新插拔过（USB口变了会导致设备标识变化），"
            "或者选错了模板。去“标定模板库…”确认一下选的模板对不对。"));
        warn->setStyleSheet(badBanner());
        warn->setWordWrap(true);
        ctl->addWidget(warn);
    } else if (matched < templateTotal) {
        auto* info = new QLabel(QStringLiteral(
            "ℹ 模板共标定了 %1 台相机，当前接入的相机里有 %2 台能对应上——"
            "将只用这 %2 台参与三角化，模板里另外 %3 台没找到对应的实时相机"
            "（可能没接、或者被重新插拔过）。%4")
            .arg(templateTotal).arg(matched).arg(templateTotal - matched)
            .arg(matched < 2 ? QStringLiteral("但能匹配上的不足2台，三角化暂时无法进行。")
                              : QString()));
        info->setStyleSheet(warnBanner());
        info->setWordWrap(true);
        ctl->addWidget(info);
    }

    qualityBanner_ = new QLabel;
    qualityBanner_->setStyleSheet(QStringLiteral("padding:6px 8px;border-radius:2px;"));
    qualityBanner_->hide();   // 样本不够/还没触发过质量变化前不显示，避免一开始就摆条空横幅
    ctl->addWidget(qualityBanner_);

    v->addWidget(ctlHost);

    view_ = new TriangulationView;
    view_->setMinimumHeight(220);
    v->addWidget(view_, 1);          // 剩下的全给视图

    v->addWidget(statusHost);

    status_ = new QLabel(QStringLiteral("勾选相机后自动开始"));
    statusStrip->addWidget(status_);

    handoffLabel_ = new QLabel(QStringLiteral(
        "相机切换跳变诊断：(等待数据——静止或缓慢移动标定物几秒，再看这里的统计)"));
    handoffLabel_->setWordWrap(true);
    handoffLabel_->setStyleSheet(QStringLiteral("padding:4px;color:%1;").arg(theme::hex(theme::textDim())));
    statusStrip->addWidget(handoffLabel_);

    connect(list_, &QListWidget::itemChanged, this, [this](QListWidgetItem*) { rebuildTriangulator(); });
    rebuildTriangulator();
}

TriangulationDebugDialog::~TriangulationDebugDialog() {
    // 关闭窗口，恢复到打开之前的状态：本来就开着检测的不用管；本来没开的，
    // 把这次帮忙打开过的那几台关回去，不留副作用。
    if (!wasDetectOn_)
        for (ICamera* c : weTurnedOn_) c->setDetectEnabled(false);
}

void TriangulationDebugDialog::rebuildTriangulator() {
    delete tri_; tri_ = nullptr;
    qualityBanner_->hide();   // 换相机组合，之前的滑动平均状态不再适用，隐藏旧横幅重新积累
    view_->setTrusted(true);  // 同理重置画法，不带着上一组的"不可信"状态进新的一组
    resetJitterStats();       // 换相机组合，之前的抖动统计不再有参考意义

    QVector<ICamera*> chosen;
    for (int i = 0; i < list_->count(); ++i) {
        QListWidgetItem* item = list_->item(i);
        if (item->checkState() != Qt::Checked) continue;
        if (ICamera* c = mgr_->byId(item->data(Qt::UserRole).toUInt())) chosen << c;
    }

    // 自动开检测：如果打开这个窗口之前检测本来就没开（wasDetectOn_==false），
    // blobsReady 永远不会来，三角化点开跟没反应一样——这里按当前勾选的相机
    // 集合，动态开/关检测，而不是要求用户先自己去点 UDP 推送。放在数量检查
    // 之前执行，这样哪怕勾选数掉到2台以下，之前帮忙打开过的相机也能被
    // 正确关掉，不会漏掉清理。
    if (!wasDetectOn_) {
        for (auto it = weTurnedOn_.begin(); it != weTurnedOn_.end(); ) {
            if (!chosen.contains(*it)) { (*it)->setDetectEnabled(false); it = weTurnedOn_.erase(it); }
            else ++it;
        }
        for (ICamera* c : chosen) {
            if (!weTurnedOn_.contains(c)) { c->setDetectEnabled(true); weTurnedOn_.insert(c); }
        }
    }

    if (chosen.size() < 2) {
        status_->setText(QStringLiteral("至少勾选两台已标定的相机"));
        view_->setCameras({});
        return;
    }

    tri_ = new Triangulator(chosen, store_, this);
    connect(tri_, &Triangulator::point3DReady, this, &TriangulationDebugDialog::onPoint);
    connect(tri_, &Triangulator::qualityChanged, this, &TriangulationDebugDialog::onQuality);
    applyAlgoOptions();   // 新建的 Triangulator 默认全开，按当前复选框状态同步一次

    QVector<TriangulationView::CamPose> poses;
    for (ICamera* c : chosen) {
        const CameraCalibration cal = store_->get(c->deviceKey());
        poses << TriangulationView::CamPose{ c->name(), cal.extr.R, cal.extr.t };
    }
    view_->setCameras(poses);

    status_->setText(QStringLiteral("已就绪：%1 台相机参与，拿反光球在共视区晃动").arg(chosen.size()));
}

void TriangulationDebugDialog::onPoint(QVector3D pos, double residualMm, int usedViews, qint64,
                                       QVector<quint32> usedCamIds) {
    view_->addPoint(pos, residualMm, 0);
    status_->setText(QStringLiteral("坐标 (%1, %2, %3) mm   残差 %4 mm   本帧 %5 台相机")
        .arg(pos.x(),0,'f',1).arg(pos.y(),0,'f',1).arg(pos.z(),0,'f',1)
        .arg(residualMm,0,'f',2).arg(usedViews));

    // ---- 相机切换跳变诊断 ----
    // 用QSet比较而不是QVector顺序比较——同样几台相机、顺序不同不算"变了"，
    // 只关心集合本身有没有变化(哪台加入/退出)。
    const QSet<quint32> curSet(usedCamIds.begin(), usedCamIds.end());
    const QSet<quint32> lastSet(lastUsedCamIds_.begin(), lastUsedCamIds_.end());
    if (hasLastPos_ && curSet != lastSet && !lastSet.isEmpty()) {
        ++totalHandoffEvents_;
        const double jumpMm = (pos - lastPos_).length();
        if (jumpMm > kHandoffJumpMm) {
            ++handoffJumpCount_;
            const QSet<quint32> added = curSet - lastSet;
            const QSet<quint32> removed = lastSet - curSet;
            QStringList changeDesc;
            for (quint32 id : added) changeDesc << QStringLiteral("+cam#%1").arg(id);
            for (quint32 id : removed) changeDesc << QStringLiteral("-cam#%1").arg(id);
            handoffLabel_->setStyleSheet(QStringLiteral("padding:4px;color:%1;font-weight:bold;").arg(theme::hex(theme::warn())));
            handoffLabel_->setText(QStringLiteral(
                "⚠ 相机切换跳变：这一帧参与解算的相机变了(%1)，同时位置跳了 %2mm——"
                "累计 %3 次跳变 / %4 次相机切换事件。如果这个位置反复触发，"
                "问题大概率在这几台相机之间的标定自洽性(重投影残差、外参精度)，"
                "不是检测噪声——考虑重新标定，或者检查这几台相机的重投影误差。")
                .arg(changeDesc.join(QStringLiteral("，")))
                .arg(jumpMm, 0, 'f', 1)
                .arg(handoffJumpCount_).arg(totalHandoffEvents_));
        } else if (handoffJumpCount_ == 0) {
            // 还没出现过明显跳变时，用平静的语气报告统计，不要一直显示警告色。
            handoffLabel_->setStyleSheet(QStringLiteral("padding:4px;color:%1;").arg(theme::hex(theme::textDim())));
            handoffLabel_->setText(QStringLiteral(
                "相机切换跳变诊断：已发生 %1 次相机切换，暂无明显跳变(>%2mm)——目前看起来正常。")
                .arg(totalHandoffEvents_).arg(kHandoffJumpMm, 0, 'f', 0));
        }
    }
    lastUsedCamIds_ = usedCamIds;
    lastPos_ = pos;
    hasLastPos_ = true;

    jitterSamples_.push_back(pos);
    if (jitterSamples_.size() > kJitterWindow) jitterSamples_.removeFirst();
    updateJitterLabel();
}

void TriangulationDebugDialog::onQuality(TriangulationQuality quality, double avgResidualMm, int windowSize) {
    qualityBanner_->show();
    view_->setTrusted(quality != TriangulationQuality::Bad);
    switch (quality) {
    case TriangulationQuality::Good:
        qualityBanner_->setText(QStringLiteral("标定质量良好（近%1帧平均残差 %2mm）")
            .arg(windowSize).arg(avgResidualMm, 0, 'f', 1));
        qualityBanner_->setStyleSheet(okBanner());
        break;
    case TriangulationQuality::Warning:
        qualityBanner_->setText(QStringLiteral(
            "⚠ 残差偏大（近%1帧平均 %2mm）——留意相机是否被碰动过，或标定跟当前场景不完全匹配")
            .arg(windowSize).arg(avgResidualMm, 0, 'f', 1));
        qualityBanner_->setStyleSheet(warnBanner());
        break;
    case TriangulationQuality::Bad:
        qualityBanner_->setText(QStringLiteral(
            "✕ 残差持续很大（近%1帧平均 %2mm）——三角化在数学上仍会给出一个点（图中已改成空心虚线），"
            "但这只是给定错误标定参数下的最优凑合解，不代表真实位置。"
            "很可能是相机被碰动过或者选错了模板，建议重新标定或换个模板。")
            .arg(windowSize).arg(avgResidualMm, 0, 'f', 1));
        qualityBanner_->setStyleSheet(badBanner());
        break;
    }
}

void TriangulationDebugDialog::applyAlgoOptions() {
    if (!tri_) return;
    tri_->setTimeInterpolation(chkInterp_->isChecked());
    tri_->setRobustMode(Triangulator::RobustMode(robustModeCombo_->currentData().toInt()));
    // minCutoff<=0 关闭滤波（Triangulator.hpp 的约定）；开启时用它自己头
    // 文件里的默认参数(30.0, 0.5)，跟"默认全开"的行为完全一致，不引入
    // 这里私自调的另一套参数、免得跟默认值对不上号造成混淆。
    tri_->setOutputFilter(chkFilter_->isChecked() ? 30.0 : -1.0, 0.5);
}

void TriangulationDebugDialog::resetJitterStats() {
    jitterSamples_.clear();
    handoffJumpCount_ = 0;
    totalHandoffEvents_ = 0;
    hasLastPos_ = false;   // 重置后不要拿"重置前最后一个点"跟"重置后第一个点"比出一次假跳变
    handoffLabel_->setStyleSheet(QStringLiteral("padding:4px;color:%1;").arg(theme::hex(theme::textDim())));
    handoffLabel_->setText(QStringLiteral("相机切换跳变诊断：(已重置，等待新数据)"));
    updateJitterLabel();
}

void TriangulationDebugDialog::updateJitterLabel() {
    if (!jitterLabel_) return;
    if (jitterSamples_.size() < 5) {
        jitterLabel_->setText(QStringLiteral(
            "抖动：样本不足（拿标定物悬空静止几秒，样本 %1/5）").arg(jitterSamples_.size()));
        return;
    }
    // 静止物体的输出点理论上应该是同一个点，实际会因为抖动散开成一小团——
    // 这团点到自己质心的距离的均方根，就是纯抖动量(mm)，跟标定误差、
    // 三角化算法、滤波强度都有关，是"这组开关组合到底稳不稳"最直接的
    // 数字化体现。
    QVector3D mean(0, 0, 0);
    for (const auto& p : jitterSamples_) mean += p;
    mean /= float(jitterSamples_.size());
    double sumSq = 0;
    for (const auto& p : jitterSamples_) {
        const QVector3D d = p - mean;
        sumSq += double(QVector3D::dotProduct(d, d));
    }
    const double rms = std::sqrt(sumSq / jitterSamples_.size());
    jitterLabel_->setText(QStringLiteral(
        "抖动：%1 mm（最近 %2 帧位置标准差；标定物保持静止时这个数越小越好）")
        .arg(rms, 0, 'f', 3).arg(jitterSamples_.size()));
}

} // namespace mocap