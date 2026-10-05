#include "ui/SyncMonitor.hpp"
#include "ui/Theme.hpp"
#include "camera/CameraManager.hpp"
#include "camera/ICamera.hpp"
#include "net/UdpSender.hpp"

#include <QVBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QTimer>
#include <QImage>
#include <algorithm>

namespace mocap {

SyncMonitor::SyncMonitor(CameraManager* mgr, UdpSender* udp, QWidget* parent)
    : QWidget(parent), mgr_(mgr), udp_(udp) {
    // 这个面板现在挂在侧边栏里，侧边栏顶部已经有一行标题了 —— 原来面板内
    // 那个加粗大标题就成了重复，去掉；顺便按 VS Code 的密度收紧边距。
    auto* v = new QVBoxLayout(this);
    v->setContentsMargins(0, 0, 0, 8);
    v->setSpacing(6);

    table_ = new QTableWidget(0, 5, this);
    QStringList heads;
    heads << QString::fromUtf8("\u76f8\u673a")      // 相机
          << QString::fromUtf8("fps")
          << QString::fromUtf8("\u5e27\u6570")      // 帧数
          << QString::fromUtf8("\u540c\u6b65\u504f\u5dee")  // 同步偏差
          << QString::fromUtf8("\u68c0\u6d4b");     // 检测
    table_->setHorizontalHeaderLabels(heads);
    table_->verticalHeader()->setVisible(false);
    table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    for (int c = 1; c < 5; ++c)
        table_->horizontalHeader()->setSectionResizeMode(c, QHeaderView::ResizeToContents);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->setSelectionMode(QAbstractItemView::NoSelection);
    table_->setShowGrid(false);
    // fps / 帧数 / 偏差 / 点数全是数字，等宽字体下列与列才对得齐，扫一眼
    // 就能看出哪一路掉队 —— 比例字体下数字宽度不一，看着永远是歪的。
    table_->setStyleSheet(theme::monoCss(12.0));
    v->addWidget(table_, 1);

    summary_ = new QLabel;
    summary_->setStyleSheet(QString("color:%1; padding: 0 10px;")
                                .arg(theme::hex(theme::textDim())));
    summary_->setWordWrap(true);
    v->addWidget(summary_);

    setMinimumWidth(240);   // 侧边栏能拖多窄，取决于这里

    refresh_ = new QTimer(this);
    connect(refresh_, &QTimer::timeout, this, &SyncMonitor::refresh);
    refresh_->start(250);   // 4Hz 刷新，监控不必更快

    rebuildRows();
}

void SyncMonitor::rebuildRows() {
    table_->setRowCount(mgr_->count());
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        auto* name = new QTableWidgetItem(c->name());
        table_->setItem(i, 0, name);
        for (int col = 1; col < 5; ++col)
            table_->setItem(i, col, new QTableWidgetItem("-"));
    }
    // 清理已移除相机的统计。
    QList<quint32> alive;
    for (int i = 0; i < mgr_->count(); ++i) alive << mgr_->at(i)->id();
    for (auto k : stats_.keys())
        if (!alive.contains(k)) stats_.remove(k);
}

void SyncMonitor::onFrame(quint32 camId, const QImage&, double fps) {
    Stat& s = stats_[camId];
    s.fps = fps;
    ++s.frames;
}

void SyncMonitor::onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns) {
    Stat& s = stats_[camId];
    s.blobs = pts.size();
    s.lastTs = ts_ns;
}

void SyncMonitor::refresh() {
    // 找“最新时间戳”的相机作参考，算各路偏差（同步健康度）。
    qint64 refTs = 0;
    for (int i = 0; i < mgr_->count(); ++i) {
        const Stat& s = stats_.value(mgr_->at(i)->id());
        if (s.lastTs > refTs) refTs = s.lastTs;
    }

    qint64 maxSkew = 0;
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        const Stat& s = stats_.value(c->id());
        if (table_->item(i, 0)) table_->item(i, 0)->setText(c->name());

        auto set = [&](int col, const QString& t, const QColor& fg = QColor()) {
            if (!table_->item(i, col)) table_->setItem(i, col, new QTableWidgetItem);
            auto* it = table_->item(i, col);
            it->setText(t);
            if (fg.isValid()) it->setForeground(fg);
        };

        set(1, s.frames ? QString::number(s.fps, 'f', 0) : "-");
        set(2, s.frames ? QString::number(s.frames) : "-");

        if (s.lastTs > 0 && refTs > 0) {
            const qint64 skewUs = (refTs - s.lastTs) / 1000;   // 微秒
            maxSkew = std::max(maxSkew, skewUs);
            QColor col = theme::good();
            if (skewUs > 5000)      col = theme::bad();    // >5ms 严重
            else if (skewUs > 1000) col = theme::warn();   // >1ms 偏差
            set(3, QString("%1 us").arg(skewUs), col);
        } else {
            set(3, "-");
        }
        set(4, s.frames ? QString::number(s.blobs) : "-");
    }

    const QString udpLine = (udp_ && udp_->isEnabled())
        ? QString::fromUtf8("UDP -> %1:%2  \u5df2\u53d1 %3 \u5305")   // 已发 N 包
              .arg(udp_->address().toString()).arg(udp_->port()).arg(udp_->packetsSent())
        : QString::fromUtf8("UDP \u672a\u5f00\u542f");                 // 未开启

    summary_->setText(QString::fromUtf8(
        "\u6700\u5927\u540c\u6b65\u504f\u5dee\uff1a%1 us\n%2")        // 最大同步偏差
        .arg(maxSkew).arg(udpLine));
}

} // namespace mocap
