// ===========================================================================
// PointCloudTestDialog_widgets.hpp —— PointCloudTestDialog 内部 UI 辅助件。
// 从 PointCloudTestDialog.cpp 拆出（原第 76~255 行，当时在匿名命名空间里）：
//   ElidingLabel  读数条专用标签（可压到 40px + 省略号 + 悬停全文）
//   setBriefText  把“主干上屏、细节进 tooltip”写成接 QLabel* 的自由函数
//   FlowLayout    自动换行的行布局（官方示例 + 两处改动）
// 本头文件只被 PointCloudTestDialog 一族 .cpp 使用；类型放在 mocap 下即可。
// ===========================================================================
#pragma once

#include <QLabel>
#include <QLayout>
#include <QToolTip>
#include <QHelpEvent>
#include <QPainter>
#include <QList>

namespace mocap {

// ---------------------------------------------------------------------------
// 会自动换行的行布局。
//
// 【为什么需要它】参数从横向长龙改成右侧定宽栏之后，每一"行"里那 4~10 个
// 控件照样会超出栏宽 —— 只是把横向滚动条从窗口级挪到了侧栏级，没解决问题。
// 手工把每行拆成固定的两三列也不行：这里的控件宽度差距极大，一个
// "叠加显示骨架(AI关联，需要已训练模型)" 的复选框能顶三个 spinbox，任何
// 固定列数都会在某个宽度下崩掉。
//
// 流式布局按实际宽度排：放得下就并排，放不下就换行。侧栏拖宽拖窄都自适应，
// 永远不会溢出。这是 Qt 官方 flowlayout 示例的实现，只加了两处改动：
//   · addStretch() 空实现 —— 原有代码里有十几处调用，流式布局天然左对齐，
//     弹簧没有意义，留个空函数省得逐处删。
//   · hasHeightForWidth() 返回 true —— QScrollArea 在 widgetResizable 模式下
//     会专门查这个来决定内容高度（见 Qt 的 QScrollAreaPrivate::updateScrollBars），
//     不实现的话侧栏滚动范围会算错、底下的内容够不着。
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// 底部读数条专用的标签：宽度不够时用省略号截断，鼠标悬停给出完整内容。
//
// 【为什么必须有】读数条上那几段文本很长（"已连接3台相机，本帧检测到…"、
// "耗时统计：本帧总…｜帧间隔实测…｜构建观测…"），普通 QLabel 会按完整文本
// 索要宽度，几个加起来超出窗口宽，QHBoxLayout 只能硬挤，结果就是文本被拦腰
// 截断、甚至整行被挤出窗口底边看不见 —— 截图上最后一行"帧率:相机…"就是这么
// 没的。
//
// 关键是 minimumSizeHint 返回一个很小的宽度：告诉布局"我可以被压缩"，这样
// 布局才不会为了满足我而把别人挤爆。至于压缩后怎么显示，自己 paint 成省略号。
// 内容照旧由各处的 setText 写入，一行调用都不用改。
// ---------------------------------------------------------------------------
class ElidingLabel : public QLabel {
public:
    using QLabel::QLabel;

    QSize minimumSizeHint() const override {
        return QSize(40, QLabel::minimumSizeHint().height());   // 可被压到 40px
    }
    QSize sizeHint() const override {
        // 宽度别一开口就要满屏；高度锁死一行 —— 文本里带换行时 QLabel 的
        // sizeHint 会按多行算，那会把整条读数条撑成两三行高，正是截图里
        // 最后一行被挤出窗口底边的原因。
        //
        // 【宽度必须稳定，不能跟着文本长度走】这是"下方面板不停抖动"的根因：
        // 这些标签的内容是每帧刷新的读数（"滞后58ms"、"帧间隔28.9ms"、
        // "峰值同时在线 18 个"…），数字位数一变——9.7→10.3、99→100——
        // sizeHint 的宽度就跟着变，FlowLayout 于是重新排一遍，整条读数条
        // 里所有标签的位置都跳一下。一秒钟几十帧，看起来就是持续抖动。
        //
        // 解决办法是给一个【只增不减】的稳定宽度：文本变长时可以变宽
        // （否则会被无谓地截断），但变短时保持不变——读数的位数是在一个
        // 小范围内来回跳的，锁住上界之后就再也不会因为"这一帧少了一位数"
        // 而重排。上界本身封顶在 460px（跟原来一致），所以不会无限增长。
        const QSize s = QLabel::sizeHint();
        const int w = qMin(s.width(), 460);
        if (w > stableW_) stableW_ = w;
        return QSize(stableW_, fontMetrics().height() + 4);
    }


protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        // simplified()：把多行读数压成一行再截断 —— 读数条是一行高的，
        // 多行文本在这里只会被切掉下半截，压平至少能看见前半段。
        const QString full = text().simplified();
        p.setPen(palette().color(foregroundRole()));
        p.drawText(rect(), Qt::AlignVCenter | Qt::AlignLeft,
                   fontMetrics().elidedText(full, Qt::ElideRight, width()));
    }
    bool event(QEvent* e) override {
        // 没单独设过 tooltip 的，就把完整文本当 tooltip —— 截断了也能查全。
        // 显式设过 tooltip（setBrief）的走 QLabel 默认逻辑，别覆盖掉全文。
        if (e->type() == QEvent::ToolTip && toolTip().isEmpty() && !text().isEmpty()) {
            QToolTip::showText(static_cast<QHelpEvent*>(e)->globalPos(), text(), this);
            return true;
        }
        return QLabel::event(e);
    }

private:
    // 见 sizeHint() 的说明：只增不减的稳定宽度，用来消掉逐帧读数
    // 位数变化引起的布局重排（"下方面板不停抖动"的根因）。
    // mutable 是因为 sizeHint() 是 const 的。
    mutable int stableW_ = 0;
};

// 【主干上屏、细节进 tooltip】读数条就那么宽，而那几条文本
// （"滞后58ms = 推理15.3 + 排队28.1 + 陈旧15…"、"稳定性统计…新增/峰值…"、
// "耗时统计…｜构建观测…"）都长到必然被省略号截断 —— 而截断之后，
// 最要紧的那个数往往恰好在被切掉的那半截里。
// 与其让它自己截，不如显式分成两段：短的那段保证完整可见，
// 长的那段挂到 tooltip 上，鼠标一悬停就是全文。
//
// 【写成自由函数而不是 ElidingLabel 的成员】ElidingLabel 是这个 .cpp 里的
// 局部类型，头文件里看不到它，所以 latencyLabel_/stabilityLabel_/timingLabel_
// 三个成员只能声明成 QLabel*。做成成员函数的话这三处全都调不到
// （'class QLabel' has no member named 'setBrief'）。
// 自由函数接 QLabel*，对两种类型都能用，也不需要动头文件。
inline void setBriefText(QLabel* lb, const QString& brief, const QString& full) {
    if (!lb) return;
    if (lb->text() != brief) lb->setText(brief);
    if (lb->toolTip() != full) lb->setToolTip(full);
}

// 每个控件(带自己的"xxx："标签)独占一行的竖排布局。名字沿用 FlowLayout
// 只为不动上百处调用点，实际已经不再横向流式换行了。
//
// 【为什么放弃换行】换行版的高度取决于宽度，而这些行外面套的是
// QVBoxLayout + QScrollArea：QScrollArea 不走 heightForWidth，只认 sizeHint()，
// 而 sizeHint() 又得先知道实际宽度才能算出换了几行 —— 鸡生蛋问题。侧栏一
// 拖窄，某一行悄悄多换出一行，父布局却还按旧高度分配空间，下面的控件就
// 被压上来(截图里"观测方差地板"叠在"IEKF全局最优指派"上就是这样)。
// 试过在 setGeometry() 里 invalidate()、试过延迟到下一轮事件循环再
// invalidate()，都只能缓解不能根治 —— 布局激活期间的失效请求本来就不保证
// 被当轮采纳，何况多层嵌套各自还有缓存。
//
// 改成一组一行后，高度 = 各组高度之和，跟宽度彻底无关，sizeHint() 一次就
// 报得准，压到多窄都不会重叠。代价是宽的时候纵向更长，但这是个侧栏参数
// 面板，本来就要滚动，读起来反而更整齐。
class FlowLayout : public QLayout {
public:
    explicit FlowLayout(int hSpacing = 6, int vSpacing = 6)
        : hSpace_(hSpacing), vSpace_(vSpacing) { setContentsMargins(0, 0, 0, 0); }
    ~FlowLayout() override {
        while (QLayoutItem* it = takeAt(0)) delete it;
    }

    void addItem(QLayoutItem* item) override { items_.append(item); invalidate(); }
    void addStretch(int = 0) {}                       // 见类注释：兼容用的空实现

    int count() const override { return int(items_.size()); }
    QLayoutItem* itemAt(int i) const override { return items_.value(i, nullptr); }
    QLayoutItem* takeAt(int i) override {
        QLayoutItem* it = (i >= 0 && i < items_.size()) ? items_.takeAt(i) : nullptr;
        if (it) invalidate();
        return it;
    }

    Qt::Orientations expandingDirections() const override { return {}; }

    // 高度不再依赖宽度 —— 这正是本次重写的全部意义，别再改回 true。
    bool hasHeightForWidth() const override { return false; }

    void setGeometry(const QRect& r) override {
        QLayout::setGeometry(r);
        doLayout(r, false);
    }

    QSize sizeHint() const override { return calcSize(false); }

    // 最小宽度只保证「标签 + 一个能点的输入框」放得下，比 sizeHint 窄很多 ——
    // 否则侧栏被这几行顶住拖不窄、或者冒出横向滚动条。高度必须是完整的堆叠
    // 高度，一分不能少，那正是不重叠的保证。
    QSize minimumSize() const override { return calcSize(true); }

private:
    // "标签：" 后面紧跟的那个控件必须跟标签待在同一行 —— 否则会出现
    // "maxReprojNorm：" 停在行尾、它的输入框掉到下一行开头这种读不懂的排版。
    // 判据：文本以冒号结尾的 QLabel，跟下一个 item 绑成一组。
    static bool isStickyLabel(QLayoutItem* it) {
        auto* lb = qobject_cast<QLabel*>(it->widget());
        if (!lb) return false;
        const QString t = lb->text().trimmed();
        return t.endsWith(QLatin1Char(':')) || t.endsWith(QChar(0xFF1A));   // 半角/全角冒号
    }

    // 一组 = [标签, 控件] 或单个 item；返回这一组占几个 item。
    int groupLenAt(int i) const {
        return (isStickyLabel(items_[i]) && i + 1 < items_.size()) ? 2 : 1;
    }

    static constexpr int kMinFieldW = 48;   // 输入框最窄还能点得动的宽度

    QSize calcSize(bool minimal) const {
        int w = 0, h = 0;
        for (int i = 0; i < items_.size(); ) {
            const int n = groupLenAt(i);
            int gw = 0, gh = 0;
            for (int k = 0; k < n; ++k) {
                const QSize sz = items_[i + k]->sizeHint();
                int itemW = sz.width();
                if (minimal && k == n - 1 && n > 1)   // 只压输入框，标签留全
                    itemW = qMax(items_[i + k]->minimumSize().width(), kMinFieldW);
                gw += itemW + (k ? hSpace_ : 0);
                gh = qMax(gh, sz.height());
            }
            w = qMax(w, gw);
            h += gh + (i ? vSpace_ : 0);
            i += n;
        }
        const QMargins m = contentsMargins();
        return QSize(w + m.left() + m.right(), h + m.top() + m.bottom());
    }

    void doLayout(const QRect& rect, bool /*testOnly*/) const {
        const QMargins m = contentsMargins();
        const QRect eff = rect.adjusted(m.left(), m.top(), -m.right(), -m.bottom());
        int y = eff.y();

        for (int i = 0; i < items_.size(); ) {
            const int n = groupLenAt(i);
            int gh = 0, gw = 0;
            for (int k = 0; k < n; ++k) {
                const QSize sz = items_[i + k]->sizeHint();
                gh = qMax(gh, sz.height());
                gw += sz.width() + (k ? hSpace_ : 0);
            }

            // 组比可用宽度还宽时，压缩组里最后那个控件(输入框)，标签保持完整
            // 可读；压不下去就让它溢出，交给外面的横向滚动条，绝不叠到下一行。
            int overflow = qMax(0, gw - eff.width());

            int x = eff.x();
            for (int k = 0; k < n; ++k) {
                QSize sz = items_[i + k]->sizeHint();
                if (overflow > 0 && k == n - 1) {
                    const int minW = qMax(items_[i + k]->minimumSize().width(), kMinFieldW);
                    const int newW = qMax(minW, sz.width() - overflow);
                    sz.setWidth(newW);
                }
                // 同一行内各控件按行高垂直居中，高矮不一时不会歪
                const int dy = (gh - sz.height()) / 2;
                items_[i + k]->setGeometry(QRect(QPoint(x, y + dy), sz));
                x += sz.width() + hSpace_;
            }

            y += gh + vSpace_;
            i += n;
        }
    }

    QList<QLayoutItem*> items_;
    int hSpace_;
    int vSpace_;
};

} // namespace mocap
