#include "ui/CameraGrid.hpp"
#include "ui/GridLayout.hpp"
#include "camera/CameraManager.hpp"
#include "camera/ICamera.hpp"
#include <QLabel>
#include <QVBoxLayout>

namespace mocap {

CameraGrid::CameraGrid(CameraManager* mgr, QWidget* parent)
    : QWidget(parent), mgr_(mgr) {
    grid_ = new QGridLayout(this);
    // 1px 间距：宫格之间只留一条分隔线的宽度，跟 VS Code 的编辑器分屏一样，
    // 视觉上是"一整块被切开"，而不是"几张浮在背景上的卡片"。
    grid_->setContentsMargins(0, 0, 0, 0);
    grid_->setSpacing(1);

    // 【空态占位】开机（或相机全被移除）时这块区域是整片黑的，一点提示都没有 ——
    // 第一次上手的人会以为程序没起来。给一行低调的等待小字：主行说明状态，
    // 次行告诉下一步点哪里。颜色走 Theme.hpp 里 #gridHintTitle / #gridHintSub
    // 两条规则，不在本文件硬编码色值（跟 theme 顶部"颜色只在一处定义"的约定一致）。
    {
        auto* box = new QWidget(this);
        box->setObjectName(QStringLiteral("gridEmptyHint"));
        auto* col = new QVBoxLayout(box);
        col->setContentsMargins(0, 0, 0, 0);
        col->setSpacing(8);
        auto* title = new QLabel(QString::fromUtf8("请添加相机"), box);
        title->setObjectName(QStringLiteral("gridHintTitle"));
        title->setAlignment(Qt::AlignCenter);
        auto* sub = new QLabel(
            QString::fromUtf8("点击左上角「＋ 添加相机」接入摄像头，或添加虚拟测试相机"), box);
        sub->setObjectName(QStringLiteral("gridHintSub"));
        sub->setAlignment(Qt::AlignCenter);
        // 【上下各加一条弹簧，两行才会挨在一起居中】
        // QLabel 的默认垂直尺寸策略是 Preferred —— 它能被撑大。只写
        // addWidget(title) + addWidget(sub) 的话，QVBoxLayout 会把富余高度
        // 平摊给两个标签，于是标题被顶到上半区正中、副行被压到下半区正中，
        // 两行相隔好几百像素，看着像两句毫不相干的话。
        // 上下各一条 stretch 之后，富余高度全归弹簧，两个标签各自贴着文字大小、
        // 作为一个整体垂直居中。
        col->addStretch(1);
        col->addWidget(title);
        col->addWidget(sub);
        col->addStretch(1);
        // 纯展示：别把宫格上的双击放大 / 拖拽换位这些操作截走
        box->setAttribute(Qt::WA_TransparentForMouseEvents, true);
        emptyHint_ = box;
        emptyHint_->hide();
    }
    rebuild();
}

void CameraGrid::clearViews() {
    for (CameraView* v : views_) { grid_->removeWidget(v); v->deleteLater(); }
    views_.clear();
    for (int i = 0; i < 32; ++i) {
        grid_->setRowStretch(i, 0);
        grid_->setColumnStretch(i, 0);
    }
}

CameraView* CameraGrid::makeView(ICamera* cam) {
    auto* v = new CameraView(cam->id(), cam->name(), this);
    connect(cam, &ICamera::frameReady, v, &CameraView::onFrame);
    connect(cam, &ICamera::blobsReady, v, &CameraView::onBlobs);

    // 拖拽换位：交换顺序，Manager 发 countChanged -> MainWindow 触发 rebuild。
    connect(v, &CameraView::swapRequested, this,
            [this](quint32 a, quint32 b) { mgr_->swapByIds(a, b); });

    // 双击放大/还原。
    connect(v, &CameraView::soloToggled, this, [this](quint32 id) {
        soloId_ = (soloId_ == qint64(id)) ? -1 : qint64(id);
        rebuild();
    });

    connect(v, &CameraView::paramsRequested, this, &CameraGrid::paramsRequested);
    connect(v, &CameraView::removeRequested, this, &CameraGrid::removeRequested);

    views_.push_back(v);
    return v;
}

void CameraGrid::rebuild() {
    clearViews();
    const int n = mgr_->count();
    if (n == 0) {
        soloId_ = -1;
        // 空态：把提示铺满整个宫格区域、居中显示。
        grid_->addWidget(emptyHint_, 0, 0);
        grid_->setRowStretch(0, 1);
        grid_->setColumnStretch(0, 1);
        emptyHint_->show();
        return;
    }
    // 有相机了：必须 removeWidget，光 hide() 不够 —— 它还占着 (0,0) 那一格，
    // 会把第一台相机的格子挤掉。
    grid_->removeWidget(emptyHint_);
    emptyHint_->hide();

    // 被放大的相机若已移除，退出 solo。
    if (soloId_ >= 0 && !mgr_->byId(quint32(soloId_))) soloId_ = -1;

    if (soloId_ >= 0) {
        grid_->addWidget(makeView(mgr_->byId(quint32(soloId_))), 0, 0);
        grid_->setRowStretch(0, 1);
        grid_->setColumnStretch(0, 1);
        return;
    }

    const GridDim g = gridDimFor(n);
    for (int i = 0; i < n; ++i) {
        const Cell c = cellFor(i, g);
        grid_->addWidget(makeView(mgr_->at(i)), c.row, c.col);
    }
    for (int r = 0; r < g.rows; ++r) grid_->setRowStretch(r, 1);
    for (int c = 0; c < g.cols; ++c) grid_->setColumnStretch(c, 1);
}

} // namespace mocap