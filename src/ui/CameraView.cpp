#include "ui/CameraView.hpp"
#include "ui/Theme.hpp"
#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QMouseEvent>
#include <QContextMenuEvent>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QDrag>
#include <QMenu>
#include <QApplication>

namespace mocap {

static const char* kMime = "application/x-mocap-camid";

// VS Code 全局直角。这里留 2px 只是为了让相邻格子的交界不至于完全糊在一起，
// 不是"卡片圆角"——想彻底方角把它改成 0 即可。
static constexpr qreal kRadius = 2.0;

CameraView::CameraView(quint32 camId, QString name, QWidget* parent)
    : QWidget(parent), camId_(camId), name_(std::move(name)) {
    setMinimumSize(160, 120);
    setAcceptDrops(true);
    setToolTip(QString::fromUtf8(
        "\u62d6\u62fd\u6362\u4f4d \u00b7 \u53cc\u51fb\u653e\u5927 \u00b7 \u53f3\u952e\u83dc\u5355"));
    // 拖拽换位 · 双击放大 · 右键菜单
}

void CameraView::onFrame(quint32 camId, const QImage& img, double fps) {
    if (camId != camId_) return;
    frame_ = img;      // QImage 隐式共享，拷贝很轻
    fps_ = fps;
    update();
}

void CameraView::onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64) {
    if (camId != camId_) return;
    blobs_ = pts;
    blobsAge_.restart();   // 记录这次更新的时间，供 paintEvent 判断是否已过期
    // 不 update()：等下一帧 onFrame 一起刷，避免重复重绘。
}

void CameraView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // 画面底：比编辑区底再暗一档，红外画面上的反光球才跳得出来。
    // 圆角从 10 收到 2 —— VS Code 全局都是直角，大圆角一出现就出戏。
    QRectF r = rect().adjusted(0.5, 0.5, -0.5, -0.5);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0x14, 0x14, 0x14));
    p.drawRoundedRect(r, kRadius, kRadius);

    QRect imgRect;
    if (!frame_.isNull()) {
        QImage scaled = frame_.scaled(size() - QSize(2, 2), Qt::KeepAspectRatio,
                                      Qt::FastTransformation);
        int x = (width()  - scaled.width())  / 2;
        int y = (height() - scaled.height()) / 2;
        imgRect = QRect(x, y, scaled.width(), scaled.height());
        p.save();
        QPainterPath clip; clip.addRoundedRect(r, kRadius, kRadius);
        p.setClipPath(clip);
        p.drawImage(x, y, scaled);
        p.restore();
    }

    // 检测结果是否“新鲜”：超过350ms没收到新的检测结果，就当作检测已经停了，
    // 哪怕 blobs_ 里还留着最后一次的点也不再画/不再计数——避免关闭检测/UDP
    // 推送后十字丝卡在最后位置永久不消失（根源是检测线程可能在关闭那一刻
    // 还有一帧在途，会比“清空”指令晚一点点送达，把清空结果又盖回去）。
    const bool blobsFresh = blobsAge_.isValid() && blobsAge_.elapsed() < 50;

    // —— 检测叠加：反光球十字丝（校准青）——
    if (!frame_.isNull() && blobsFresh && !blobs_.isEmpty() && imgRect.isValid()) {
        const double sx = double(imgRect.width())  / frame_.width();
        const double sy = double(imgRect.height()) / frame_.height();
        // 十字丝用 marker()（亮蓝）而不是 accent()：强调蓝 #0078D4 压在暗灰
        // 红外画面上偏沉，亮蓝在灰阶背景里对比最高，一眼能找到球。
        QPen pen(theme::marker()); pen.setWidthF(1.4);
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);
        for (const QPointF& b : blobs_) {
            const double px = imgRect.x() + b.x() * sx;
            const double py = imgRect.y() + b.y() * sy;
            p.drawEllipse(QPointF(px, py), 6, 6);
            p.drawLine(QPointF(px - 9, py), QPointF(px - 3, py));
            p.drawLine(QPointF(px + 3, py), QPointF(px + 9, py));
            p.drawLine(QPointF(px, py - 9), QPointF(px, py - 3));
            p.drawLine(QPointF(px, py + 3), QPointF(px, py + 9));
        }
    }

    // 细边框：跟全局分隔线同色，宫格看起来像被 1px 线切开的一块块面板
    p.setPen(QPen(theme::lineHard(), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(r, kRadius, kRadius);

    // —— 顶部信息胶囊 ——
    // 胶囊宽度按内容算，但必须封顶在预览框实际可用宽度内，否则窄格子里
    // 画出来的胶囊会被 widget 边界直接裁掉（视觉上就是“长条不够长”）。
    // 空间不够时按优先级优雅降级：先省略相机名字，再缩小字号，都不够
    // 才对整串文字做省略号——任何情况下都不会出现文字被硬生生切断。
    const bool live = !frame_.isNull();
    const QString statusSuffix = live
                                     ? QString("   %1 fps   %2\u00d7%3%4")
                                           .arg(fps_, 0, 'f', 0).arg(frame_.width()).arg(frame_.height())
                                           .arg((!blobsFresh || blobs_.isEmpty()) ? QString()
                                                                                  : QString::fromUtf8("   \u25cf %1").arg(blobs_.size()))  // ● N 检测数
                                     : QString::fromUtf8("   \u65e0\u4fe1\u53f7");                       // 无信号

    // 胶囊左侧内缩：live 时要给绿色圆点让位（更宽），无信号时不需要。
    // 总宽必须是"左内缩 + 右留白"，两者配平；之前固定用 +20 但 live 时左
    // 内缩是22，右边净宽度反而比non-live时还窄2px，正是右边缘被吃掉的根源。
    const qreal leftInset = live ? 21.0 : 9.0;
    const qreal rightMargin = 9.0;
    const qreal pad = leftInset + rightMargin;

    const qreal maxChipW = qMax(40.0, width() - 2.0);   // 直接贴到边框内侧
    QFont mono = p.font();
    mono.setFamily(QStringLiteral("Cascadia Mono"));
    mono.setStyleHint(QFont::Monospace);                // 没装 Cascadia 就回退到系统等宽
    qreal pointSize = 8.5;
    QString shownName = name_;
    QString info;
    int tw = 0;

    // 逐步降级，直到胶囊能放进可用宽度，或字号已经缩到底。
    for (int step = 0; step < 4; ++step) {
        mono.setPointSizeF(pointSize);
        p.setFont(mono);
        const QFontMetrics fm(mono);
        info = shownName + statusSuffix;
        tw = fm.horizontalAdvance(info);
        if (tw + pad <= maxChipW) break;

        if (step == 0) {
            // 第一步：名字省略号收尾，fps/分辨率/检测数保持完整不动。
            const int suffixW = fm.horizontalAdvance(statusSuffix);
            const int nameBudget = int(maxChipW - pad) - suffixW;
            if (nameBudget > fm.horizontalAdvance(QStringLiteral("…"))) {
                shownName = fm.elidedText(name_, Qt::ElideRight, nameBudget);
                continue;
            }
        }
        if (step <= 1) { pointSize -= 1.0; continue; }   // 名字已无法再省，缩字号
        // 最后一步兜底：整串按可用宽度省略，字号已在最小档。
        info = fm.elidedText(shownName + statusSuffix, Qt::ElideRight, int(maxChipW - pad));
        tw = fm.horizontalAdvance(info);
    }

    // 信息条不再是浮在画面上的圆角胶囊，改成贴着顶边的一条直角小条 ——
    // 跟编辑器标签页/状态栏是同一种"贴边的信息带"语言，不抢画面。
    const qreal chipW = qMin(qreal(tw) + pad, maxChipW);
    const qreal chipH = 20;
    QRectF chip(1, 1, chipW, chipH);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(0x18, 0x18, 0x18, 0xE0));
    p.drawRect(chip);
    if (live) { p.setBrush(theme::good()); p.setPen(Qt::NoPen);
        p.drawEllipse(QPointF(chip.x() + 11, chip.center().y()), 3, 3); }
    p.setPen(live ? theme::text() : theme::textDim());
    p.drawText(chip.adjusted(leftInset, 0, 0, 0),
               Qt::AlignVCenter | Qt::AlignLeft, info);
}

void CameraView::mousePressEvent(QMouseEvent* e) {
    if (e->button() == Qt::LeftButton) pressPos_ = e->pos();
    QWidget::mousePressEvent(e);
}

void CameraView::mouseMoveEvent(QMouseEvent* e) {
    if (!(e->buttons() & Qt::LeftButton)) return;
    if ((e->pos() - pressPos_).manhattanLength()
        < QApplication::startDragDistance()) return;

    auto* mime = new QMimeData;
    mime->setData(kMime, QByteArray::number(camId_));
    auto* drag = new QDrag(this);
    drag->setMimeData(mime);
    drag->setPixmap(grab().scaled(200, 150, Qt::KeepAspectRatio,
                                  Qt::SmoothTransformation));
    drag->exec(Qt::MoveAction);
}

void CameraView::mouseDoubleClickEvent(QMouseEvent*) {
    emit soloToggled(camId_);
}

void CameraView::contextMenuEvent(QContextMenuEvent* e) {
    QMenu m(this);
    QAction* aParam = m.addAction(QString::fromUtf8("\u53c2\u6570\u8bbe\u7f6e\u2026"));      // 参数设置…（含分辨率/帧率）
    QAction* aSolo  = m.addAction(QString::fromUtf8("\u653e\u5927 / \u8fd8\u539f"));          // 放大 / 还原
    m.addSeparator();
    QAction* aDel   = m.addAction(QString::fromUtf8("\u79fb\u9664\u6b64\u76f8\u673a"));       // 移除此相机
    QAction* r = m.exec(e->globalPos());
    if      (r == aParam) emit paramsRequested(camId_);
    else if (r == aSolo)  emit soloToggled(camId_);
    else if (r == aDel)   emit removeRequested(camId_);
}

void CameraView::dragEnterEvent(QDragEnterEvent* e) {
    if (e->mimeData()->hasFormat(kMime)) {
        const quint32 src = e->mimeData()->data(kMime).toUInt();
        if (src != camId_) { e->acceptProposedAction(); return; }
    }
    e->ignore();
}

void CameraView::dropEvent(QDropEvent* e) {
    const quint32 src = e->mimeData()->data(kMime).toUInt();
    e->acceptProposedAction();
    emit swapRequested(src, camId_);
}

} // namespace mocap