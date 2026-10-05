#include "ui/HandOutputMonitor.hpp"
#include "net/HandPacket.hpp"     // 原始报文模式：跟 UdpSender 共用同一份打包代码
#include "hand/HandModel.hpp"     // jointLimits()：画行程条 / 判触限要用

#include <QPainter>
#include <QPainterPath>   // Qt6 起不再由 <QPainter> 间接带入，掌形圆角路径要用
#include "ui/Theme.hpp"   // 【统一配色】见下面 bgColor() 那段说明
#include <QPolygonF>
#include <QPen>
#include <QFont>
#include <QFontDatabase>
#include <QFontMetricsF>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QClipboard>
#include <QGuiApplication>
#include <QDateTime>
#include <QFile>
#include <QTextStream>
#include <QStandardPaths>
#include <QMessageBox>
#include <cmath>
#include <algorithm>

namespace mocap {

namespace {

// --- 16 维关节角的名字，布局与 HandModel.hpp / Hm20JointAngles.hpp 严格一致 ---
const char* const kJointName[16] = {
    "拇 CMC屈", "拇 CMC展", "拇 MCP",  "拇 IP",
    "食 MCP屈", "食 MCP展", "食 PIP",
    "中 MCP屈", "中 MCP展", "中 PIP",
    "无 MCP屈", "无 MCP展", "无 PIP",
    "小 MCP屈", "小 MCP展", "小 PIP",
};
// 关节 -> 手指(0拇 1食 2中 3无名 4小)，用来查 fingerValid 位
inline int jointFinger(int i) { return (i < 4) ? 0 : (1 + (i - 4) / 3); }

// --- 16 个分段的名字。段0=手背/腕，段 1+3f+j，f=0 是拇指 ---
const char* const kSegName[16] = {
    "腕/手背",
    "拇 掌骨", "拇 近节", "拇 远节",
    "食 近节", "食 中节", "食 远节",
    "中 近节", "中 中节", "中 远节",
    "无 近节", "无 中节", "无 远节",
    "小 近节", "小 中节", "小 远节",
};

// SegSource：0=None 1=Predicted 2=Geometry 3=IK
const char* const kSrcName[4] = { "无", "预测", "几何", "IK" };
inline QColor srcColor(int s) {
    // 【四档来源的语义色】三档取自 Theme，保证跟软件其余部分同一套语义：
    // 能用的=绿、需注意=黄、有问题=红。只有"几何"这一档用 Theme 的 marker 蓝
    // （#4FC1FF）——它既不是"好"也不是"坏"，是一个中性的信息态，
    // 用蓝色跟另外三档区分开，也跟画面上检测十字丝的蓝呼应。
    switch (s) {
        case 3:  return mocap::theme::good();     // IK：最可信
        case 2:  return mocap::theme::marker();   // 几何：有 ~10° 贴球偏差
        case 1:  return mocap::theme::warn();     // 预测：外推，不是测量
        default: return mocap::theme::bad();      // 无依据
    }
}

// 面板配色：【全部走 Theme.hpp，不再自己定义一套】
//
// 原来这里硬编码了一组偏蓝紫的深色（底 #141414、网格 #2C2E36、文字 #D8DCE4），
// 注释里的理由是"深底更省眼"。但 Theme 本来就是深色的（VS Code Dark Modern），
// 那个理由早就不成立了 —— 实际效果是这个面板跟软件其余部分差了小半个色系：
// 底色差 11 级灰、网格线偏蓝、再配上四角的亮蓝角标，整块看起来像贴上去的
// 另一个程序的界面，跟周围 VS Code 那种"克制的灰"割裂得很明显。
//
// 现在一律引用 Theme：底 #1F1F1F、分隔线 #2B2B2B、正文 #CCCCCC、
// 次要 #9D9D9D，语义色也用 Theme 的 good/warn/bad。
// 【只在这里改，下面所有调用点不用动】——原来的函数名全部保留。
inline QColor bgColor()   { return mocap::theme::editorBg(); }
inline QColor gridColor() { return mocap::theme::lineSoft(); }
inline QColor textColor() { return mocap::theme::text(); }
inline QColor dimColor()  { return mocap::theme::textDim(); }
inline QColor okColor()   { return mocap::theme::good(); }
inline QColor warnColor() { return mocap::theme::warn(); }
inline QColor badColor()  { return mocap::theme::bad(); }

inline QString f2(double v, int dec = 2) { return QString::number(v, 'f', dec); }
constexpr double kRad2Deg = 57.29577951308232;

// 行主序 3x3 -> 四元数 (w,x,y,z)。只为显示，用最稳的分支法。
std::array<double, 4> matToQuat(const std::array<double, 9>& R) {
    const double tr = R[0] + R[4] + R[8];
    std::array<double, 4> q{1, 0, 0, 0};
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        q = {0.25 * s, (R[7] - R[5]) / s, (R[2] - R[6]) / s, (R[3] - R[1]) / s};
    } else if (R[0] > R[4] && R[0] > R[8]) {
        const double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
        q = {(R[7] - R[5]) / s, 0.25 * s, (R[1] + R[3]) / s, (R[2] + R[6]) / s};
    } else if (R[4] > R[8]) {
        const double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
        q = {(R[2] - R[6]) / s, (R[1] + R[3]) / s, 0.25 * s, (R[5] + R[7]) / s};
    } else {
        const double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
        q = {(R[3] - R[1]) / s, (R[2] + R[6]) / s, (R[5] + R[7]) / s, 0.25 * s};
    }
    return q;
}

QString hexDump(const QByteArray& b, int maxLines, bool* truncated) {
    QString out;
    const int total = (b.size() + 15) / 16;
    const int lines = (maxLines > 0) ? std::min(total, maxLines) : total;
    if (truncated) *truncated = (lines < total);
    for (int i = 0; i < lines; ++i) {
        QString line = QStringLiteral("%1  ").arg(i * 16, 4, 16, QLatin1Char('0')).toUpper();
        for (int j = 0; j < 16; ++j) {
            const int k = i * 16 + j;
            if (k >= b.size()) { line += QStringLiteral("   "); continue; }
            line += QStringLiteral("%1 ").arg(quint8(b[k]), 2, 16, QLatin1Char('0')).toUpper();
            if (j == 7) line += QLatin1Char(' ');
        }
        out += line + QLatin1Char('\n');
    }
    return out;
}

} // namespace

// ===========================================================================
// HandOutputView
// ===========================================================================

HandOutputView::HandOutputView(QWidget* parent) : QWidget(parent) {
    setMinimumWidth(330);
    setMinimumHeight(220);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void HandOutputView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), bgColor());

    QFont f = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    f.setPointSizeF(9.0);
    p.setFont(f);

    if (!snap_.hasPose && !snap_.hasQuat) {
        p.setPen(dimColor());
        p.drawText(rect(), Qt::AlignCenter,
                   QStringLiteral("等待骨架输出…\n\n需要：勾选「叠加显示骨架」\n且模型已加载、手掌已识别"));
        return;
    }

    hiddenRows_ = 0;
    switch (mode_) {
        case Quats: paintQuats(p); break;
        case Bytes: paintBytes(p); break;
        default:    paintJoints(p); break;
    }

    // ---- 溢出提示：挤到右上角 ----
    // 原来是在内容【最下面】写一行"高度不够，用复制拿完整报文"——可那一行
    // 本身也在被截断的区域里，面板一矮它第一个消失，于是"内容不全"这件事
    // 恰恰在最需要提醒的时候没人告诉你。挪到右上角常驻，永远看得见。
    if (hiddenRows_ > 0) {
        const QFontMetricsF fm(p.font());
        const QString t = QStringLiteral("▾ 还有 %1 行未显示 · 拉高面板或用「复制」")
                              .arg(hiddenRows_);
        const double w = fm.horizontalAdvance(t) + 16;
        const QRectF pill(width() - w - 6, 3, w, fm.height() + 4);
        p.setPen(Qt::NoPen);
        p.setBrush(mocap::theme::warnBannerBg());
        p.drawRoundedRect(pill, 2, 2);
        p.setPen(QPen(warnColor(), 1));
        p.drawRoundedRect(pill, 2, 2);
        p.setPen(warnColor());
        p.drawText(pill, Qt::AlignCenter, t);
    }
}

// ===========================================================================
// 右侧仪表区
// ===========================================================================
// 三个模式原来都只画左半屏 —— 关节角表、四元数表、hex dump 全是窄窄一条，
// 右边一大片纯留白，同时下面的内容还因为高度不够被截掉。空间在那儿，只是
// 没人用。
//
// 这一组函数负责把右半屏填成一排"仪表"。选的都是【看一眼就知道链路状态】
// 的东西，不是为了填满而填：姿态罗盘替代读九个矩阵数、来源着色的手部拓扑
// 图替代逐段读"几何/IK/预测"、周期折线替代一个跳来跳去的瞬时数字。
//
// 视觉语言统一：1px 细框 + 四角角标（HUD 的味道由角标给，不是靠发光和渐变
// —— 那些在深色仪表盘上很快就腻，而且会跟主界面的 VS Code 风格打架）。
// ===========================================================================

// HUD 式外框：细边 + 四角角标 + 左上角标题。
void HandOutputView::drawHudFrame(QPainter& p, const QRectF& r, const QString& title) const {
    p.save();
    // 【朴素 1px 细框，不做四角角标】原来每个角画两段亮蓝(#3E84B8)短线，
    // 是那种"科幻 HUD"的做法 —— 放在 VS Code 这种克制的灰底里非常跳，
    // 是整块面板显得割裂的主要来源之一。
    // Theme 的风格要点写得很清楚：分隔靠 1px 细线，不靠装饰；强调色只用在
    // 焦点/选中/主按钮。这里既不是焦点也不是按钮，不该占用强调色。
    // 换成跟其余区块一致的 lineSoft 细框 + 极淡的内容底，层次靠灰阶堆。
    p.fillRect(r, mocap::theme::chrome());
    p.setPen(QPen(gridColor(), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(r);

    if (!title.isEmpty()) {
        QFont f = p.font();
        f.setPointSizeF(std::max(6.5, f.pointSizeF() - 1.0));
        p.setFont(f);
        const QFontMetricsF fm(f);
        const double tw = fm.horizontalAdvance(title) + 8.0;
        // 标题压在上边框上，先用底色把那一段框线擦掉，文字才不会跟线糊在一起
        // 擦除的是框线所在处的底 —— 上面 fillRect 用的是 chrome，这里必须一致，
        // 否则标题后面会留一条颜色不同的横带。
        p.fillRect(QRectF(r.left() + 9, r.top() - fm.height() / 2 + 1, tw, fm.height() - 2),
                   mocap::theme::chrome());
        p.setPen(dimColor());
        p.drawText(QPointF(r.left() + 13, r.top() + fm.ascent() - fm.height() / 2 + 1), title);
    }
    p.restore();
}

// 腕部姿态三轴罗盘。把 wristR 的三列（=三个体轴在世界系里的方向）用一个
// 固定的等距相机投影出来 —— 比盯着 9 个 1.0000/0.0000 判断"手转到哪了"快
// 一个数量级，而且手一动它就跟着转，是整个面板里唯一"活"的图形。
void HandOutputView::drawAttitudeGizmo(QPainter& p, const QRectF& r) const {
    p.save();
    const QPointF c = r.center();
    const double R = std::min(r.width(), r.height()) * 0.36;

    // 底盘：两个同心圈 + 十字，给旋转一个参照，不然轴转起来没有深度感
    p.setPen(QPen(mocap::theme::lineSoft(), 1));
    p.drawEllipse(c, R * 1.12, R * 1.12);
    p.drawEllipse(c, R * 0.56, R * 0.56);
    p.drawLine(QPointF(c.x() - R * 1.12, c.y()), QPointF(c.x() + R * 1.12, c.y()));
    p.drawLine(QPointF(c.x(), c.y() - R * 1.12), QPointF(c.x(), c.y() + R * 1.12));

    // 等距投影：世界 (x,y,z) -> 屏幕。y 轴朝上，所以取负。
    auto proj = [&](double x, double y, double z) {
        const double sx = (x - z) * 0.8660254;              // cos30
        const double sy = (x + z) * 0.5 - y;                // sin30
        return QPointF(c.x() + sx * R, c.y() + sy * R);
    };

    struct Axis { int col; QColor col3; const char* name; };
    const Axis axes[3] = {
        { 0, mocap::theme::bad(), "X" },
        { 1, mocap::theme::good(), "Y" },
        { 2, mocap::theme::marker(), "Z" },
    };
    // 先画指向背面的轴（投影 y 大的先画），近的压在远的上面
    int order[3] = {0, 1, 2};
    auto depth = [&](int i) {
        const auto& R9 = snap_.wristR;
        return R9[size_t(0 * 3 + axes[i].col)] + R9[size_t(2 * 3 + axes[i].col)];
    };
    std::sort(order, order + 3, [&](int a, int b) { return depth(a) > depth(b); });

    QFont f = p.font(); f.setPointSizeF(std::max(6.5, f.pointSizeF() - 1.0)); p.setFont(f);
    for (int oi = 0; oi < 3; ++oi) {
        const Axis& a = axes[order[oi]];
        // 第 col 列 = 该体轴在世界系里的单位向量（行主序，所以是 R[row*3+col]）
        const double vx = snap_.wristR[size_t(0 * 3 + a.col)];
        const double vy = snap_.wristR[size_t(1 * 3 + a.col)];
        const double vz = snap_.wristR[size_t(2 * 3 + a.col)];
        const QPointF tip = proj(vx, vy, vz);
        p.setPen(QPen(a.col3, 1.8, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(c, tip);
        p.setBrush(a.col3); p.setPen(Qt::NoPen);
        p.drawEllipse(tip, 2.6, 2.6);
        p.setPen(a.col3);
        p.drawText(tip + QPointF(4, -3), QString::fromUtf8(a.name));
    }
    p.restore();
}

// roll / pitch / yaw 三条水平指示条。欧拉角只用于显示（万向锁在这里无所谓），
// 但比四元数直观得多 —— "手腕现在偏了多少度"是能直接说出口的量。
void HandOutputView::drawEulerGauges(QPainter& p, const QRectF& r) const {
    p.save();
    const auto& R = snap_.wristR;
    // ZYX 顺序，返回角度
    const double pitch = std::asin(std::clamp(-R[6], -1.0, 1.0)) * kRad2Deg;
    const double roll  = std::atan2(R[7], R[8]) * kRad2Deg;
    const double yaw   = std::atan2(R[3], R[0]) * kRad2Deg;

    QFont f = p.font(); f.setPointSizeF(std::max(6.5, f.pointSizeF() - 0.5)); p.setFont(f);
    const QFontMetricsF fm(f);
    const struct { const char* n; double v; QColor c; } rows[3] = {
        { "roll ",  roll,  mocap::theme::bad() },
        { "pitch", pitch, mocap::theme::good() },
        { "yaw  ",  yaw,   mocap::theme::marker() },
    };
    const double rowH = r.height() / 3.0;
    const double labW = fm.horizontalAdvance(QStringLiteral("pitch")) + 6;
    const double numW = fm.horizontalAdvance(QStringLiteral("-000.0°")) + 6;
    for (int i = 0; i < 3; ++i) {
        const double cy = r.top() + rowH * (i + 0.5);
        p.setPen(dimColor());
        p.drawText(QPointF(r.left() + 4, cy + fm.ascent() / 2 - 1), QString::fromUtf8(rows[i].n));
        // 条：中点是 0°，左右各 180°
        const QRectF bar(r.left() + labW + 4, cy - 3, r.width() - labW - numW - 12, 6);
        p.setPen(Qt::NoPen);
        p.fillRect(bar, mocap::theme::chrome());
        const double mid = bar.center().x();
        const double u = std::clamp(rows[i].v / 180.0, -1.0, 1.0);
        QRectF fill = (u >= 0) ? QRectF(mid, bar.top(), bar.width() / 2 * u, bar.height())
                               : QRectF(mid + bar.width() / 2 * u, bar.top(),
                                        -bar.width() / 2 * u, bar.height());
        p.fillRect(fill, rows[i].c);
        p.setPen(QPen(mocap::theme::lineHard(), 1));
        p.drawLine(QPointF(mid, bar.top() - 1), QPointF(mid, bar.bottom() + 1));
        p.setPen(textColor());
        p.drawText(QRectF(bar.right() + 4, cy - fm.height() / 2, numW, fm.height()),
                   Qt::AlignRight | Qt::AlignVCenter, f2(rows[i].v, 1) + QStringLiteral("°"));
    }
    p.restore();
}

// 链路状态：检测 → 聚类 → 骨架 → ROM → UDP。
// 每一节亮/暗对应一个已有的标志位。"面板在刷数但 Unity 收不到"这类问题，
// 看一眼哪一节是暗的就定位了，不用挨个窗口翻开关。
void HandOutputView::drawPipeline(QPainter& p, const QRectF& r) const {
    p.save();
    QFont f = p.font(); f.setPointSizeF(std::max(6.5, f.pointSizeF() - 1.0)); p.setFont(f);
    const QFontMetricsF fm(f);

    const bool wristOk = snap_.hasQuat ? ((snap_.flags & 1u) != 0)
                                       : ((snap_.rawFlags >> 6) & 1u);
    const struct { const char* n; bool on; } st[5] = {
        { "观测",  snap_.hasPose },
        { "腕部",  wristOk },
        { "掌指",  bool((snap_.rawFlags >> 5) & 1u) },
        { "ROM",   bool((snap_.rawFlags >> 7) & 1u) },
        { "UDP",   udpOn_ },
    };
    const double n = 5;
    const double gap = 10.0;
    const double cw = (r.width() - gap * (n - 1) - 8) / n;
    for (int i = 0; i < 5; ++i) {
        const QRectF cell(r.left() + 4 + i * (cw + gap), r.center().y() - 9, cw, 18);
        const QColor c = st[i].on ? okColor() : mocap::theme::textFaint();
        p.setPen(Qt::NoPen);
        p.fillRect(cell, st[i].on ? QColor(c.red(), c.green(), c.blue(), 40) : mocap::theme::chrome());
        p.setPen(QPen(c, 1));
        p.drawRect(cell);
        p.setPen(st[i].on ? c : dimColor());
        p.drawText(cell, Qt::AlignCenter, QString::fromUtf8(st[i].n));
        if (i < 4) {   // 节与节之间的连接线，通了才是亮的
            const bool flow = st[i].on && st[i + 1].on;
            p.setPen(QPen(flow ? okColor() : mocap::theme::lineSoft(), 1));
            p.drawLine(QPointF(cell.right() + 1, cell.center().y()),
                       QPointF(cell.right() + gap - 1, cell.center().y()));
        }
    }
    p.restore();
}

// 逐指新鲜度：本帧是"新算出来的"还是"沿用上一帧"。rawFlags bit0..4。
void HandOutputView::drawFingerBars(QPainter& p, const QRectF& r) const {
    p.save();
    QFont f = p.font(); f.setPointSizeF(std::max(6.5, f.pointSizeF() - 1.0)); p.setFont(f);
    const QFontMetricsF fm(f);
    static const char* const fn[5] = {"拇", "食", "中", "无", "小"};
    const double cw = (r.width() - 8) / 5.0;
    for (int i = 0; i < 5; ++i) {
        const bool fresh = (snap_.rawFlags >> i) & 1u;
        const QRectF cell(r.left() + 4 + i * cw, r.top() + 4, cw - 4, r.height() - 8);
        const QRectF bar(cell.left(), cell.top(), cell.width(), cell.height() - fm.height() - 2);
        p.setPen(Qt::NoPen);
        p.fillRect(bar, mocap::theme::chrome());
        if (fresh) {
            p.fillRect(bar, mocap::theme::accent());
        } else {   // 保持：画成斜纹，跟"没数据"的纯暗区分开
            // 【裁剪】斜线的终点特意多画出 bar.height() 那一截，是为了让线条
            // 铺满格子右上角的三角区域；但没裁剪的话这些线会一路画出 bar 的
            // 右边界，把右边相邻手指的格子也糊上斜纹 —— 截图里那一大片斜线
            // 就是这么来的：5 个格子的斜纹全叠在了一起、越往右叠得越多。
            p.setClipRect(bar);
            p.setPen(QPen(mocap::theme::lineHard(), 1));
            for (double x = bar.left(); x < bar.right() + bar.height(); x += 4)
                p.drawLine(QPointF(x, bar.bottom()), QPointF(x - bar.height(), bar.top()));
            p.setPen(Qt::NoPen);
            p.setClipping(false);
        }
        p.setPen(fresh ? textColor() : dimColor());
        p.drawText(QRectF(cell.left(), bar.bottom(), cell.width(), fm.height() + 2),
                   Qt::AlignCenter, QString::fromUtf8(fn[i]));
    }
    p.restore();
}

// 输出周期滚动折线。一个瞬时数字（帧间隔 6.3ms）看不出抖动，一条线能。
// 横向自适应宽度，最多显示最近 kSparkN 个点。
void HandOutputView::drawStepSpark(QPainter& p, const QRectF& r) const {
    p.save();
    QFont f = p.font(); f.setPointSizeF(std::max(6.5, f.pointSizeF() - 1.0)); p.setFont(f);
    const QFontMetricsF fm(f);
    if (stepHist_.size() < 2) {
        p.setPen(dimColor());
        p.drawText(r, Qt::AlignCenter, QStringLiteral("采样中…"));
        p.restore();
        return;
    }
    double mn = stepHist_[0], mx = stepHist_[0], sum = 0;
    for (double v : stepHist_) { mn = std::min(mn, v); mx = std::max(mx, v); sum += v; }
    const double avg = sum / stepHist_.size();
    const double span = std::max(0.6, mx - mn);
    const QRectF plot = r.adjusted(4, 4, -4, -fm.height() - 3);

    // 均值参考线
    p.setPen(QPen(mocap::theme::lineSoft(), 1, Qt::DashLine));
    const double ay = plot.bottom() - (avg - mn) / span * plot.height();
    p.drawLine(QPointF(plot.left(), ay), QPointF(plot.right(), ay));

    QPolygonF poly;
    const double dx = plot.width() / double(kSparkN - 1);
    const int off = kSparkN - stepHist_.size();
    for (int i = 0; i < stepHist_.size(); ++i)
        poly << QPointF(plot.left() + (off + i) * dx,
                        plot.bottom() - (stepHist_[i] - mn) / span * plot.height());
    p.setPen(QPen(mocap::theme::marker(), 1.2));
    p.setBrush(Qt::NoBrush);
    p.drawPolyline(poly);
    if (!poly.isEmpty()) {   // 最新一点点亮
        p.setBrush(mocap::theme::marker()); p.setPen(Qt::NoPen);
        p.drawEllipse(poly.back(), 2.0, 2.0);
    }
    p.setPen(dimColor());
    p.drawText(QRectF(r.left() + 4, plot.bottom() + 1, r.width() - 8, fm.height() + 2),
               Qt::AlignLeft | Qt::AlignVCenter,
               QStringLiteral("周期 %1ms  峰谷 %2~%3").arg(f2(avg, 2), f2(mn, 1), f2(mx, 1)));
    p.restore();
}

// 手部拓扑图：一只示意手，16 段按 segSource 着色。
//
// 【为什么这个图比那张表有用】M3DQ 表里"来源"那一列要逐行读 16 次才能回答
// "现在有几段是真几何解出来的、几段在靠 IK 补、几段纯预测"。画成手的形状之
// 后是一眼的事 —— 而且能看出空间分布（比如"整根无名指都在预测"通常意味着
// 那根手指被挡住了，跟"每根手指的远节都在预测"是完全不同的两个问题）。
//
// 段序跟 kSegName 一致：0 = 腕/手背，1..15 = 五指 × (近节, 中节, 远节)。
void HandOutputView::drawHandTopology(QPainter& p, const QRectF& r) const {
    p.save();
    p.setRenderHint(QPainter::Antialiasing, true);

    // 归一化手模板：手背在下，五指向上呈扇形。坐标都在 [0,1]，最后按 r 缩放。
    // 拇指单独外撇，其余四指平行 —— 只求"认得出是只手"，不求解剖精确。
    // 每根手指：根部锚点 + 方向（度，0=正上，负=向左）+ 三节的长度比
    // 【curl = 每往远端走一节，方向再拐多少度】原来三节共线，画出来是五根
    // 笔直的射线，整体像把耙子而不是手。真实手指哪怕完全伸直，三节之间也
    // 有十几度的自然弯曲，加上之后轮廓一眼就认得出是手。
    // 拇指的 curl 给反号：它的弯曲方向跟四指相反（朝掌心内扣）。
    const struct { double ax, ay, deg, len, curl; } fingers[5] = {
        { 0.20, 0.66, -62.0, 0.30,  7.0 },   // 拇：从掌侧外撇
        { 0.36, 0.46, -16.0, 0.40, -9.0 },   // 食
        { 0.50, 0.42,  -3.0, 0.44, -9.0 },   // 中
        { 0.64, 0.44,  10.0, 0.41, -9.0 },   // 无
        { 0.77, 0.50,  24.0, 0.34, -8.0 },   // 小
    };
    const double segFrac[3] = { 0.42, 0.33, 0.25 };   // 近/中/远节长度占比

    const double S = std::min(r.width(), r.height() * 1.25);
    const double ox = r.center().x() - S * 0.5;
    const double oy = r.bottom() - S * 0.80;
    auto to = [&](double x, double y) { return QPointF(ox + x * S, oy + y * S); };

    // 手背：跟着第 0 段（腕/手背）的来源着色。
    // 【用圆角路径而不是四边形】原来是直接 drawPolygon 一个梯形，四个尖角
    // 配上五根直线，整体观感很硬。手掌本来就是圆的，倒个角的成本几乎为零。
    {
        const int src = std::clamp(snap_.segSource[0], 0, 3);
        const QColor c = srcColor(src);
        QPainterPath palm;
        const QPointF P0 = to(0.26, 0.74), P1 = to(0.84, 0.63);
        const QPointF P2 = to(0.81, 0.44), P3 = to(0.31, 0.49);
        palm.moveTo((P0 + P3) * 0.5);
        palm.quadTo(P0, (P0 + P1) * 0.5);      // 腕侧圆一点
        palm.quadTo(P1, (P1 + P2) * 0.5);
        palm.quadTo(P2, (P2 + P3) * 0.5);
        palm.quadTo(P3, (P3 + P0) * 0.5);
        palm.closeSubpath();
        p.setBrush(QColor(c.red(), c.green(), c.blue(), 40));
        p.setPen(QPen(QColor(c.red(), c.green(), c.blue(), 150), 1.2));
        p.drawPath(palm);
    }

    // 五指 × 三节
    for (int fi = 0; fi < 5; ++fi) {
        double deg = fingers[fi].deg;
        double cx = fingers[fi].ax, cy = fingers[fi].ay;
        for (int j = 0; j < 3; ++j) {
            const int seg = 1 + fi * 3 + j;
            const int src = std::clamp(snap_.segSource[size_t(seg)], 0, 3);
            // 【每节再拐一点】见上面 curl 的说明：三节共线会画成一根直射线。
            // 第 0 节保持根部方向，之后每节累加，越靠远端弯得越明显。
            if (j > 0) deg += fingers[fi].curl;
            const double rad = deg * 3.14159265358979 / 180.0;
            const double dx = std::sin(rad), dy = -std::cos(rad);
            const double L = fingers[fi].len * segFrac[j];
            const QPointF a = to(cx, cy);
            const QPointF b = to(cx + dx * L, cy + dy * L);
            const QColor c = srcColor(src);
            // 骨段本体：越可信画得越粗越亮
            const double wpx = (src >= 2) ? 5.0 : (src == 1 ? 3.6 : 2.6);
            // 【先描一层深色底再画本体】相邻手指靠得近时，纯色线叠在一起
            // 会糊成一片；加一圈底色等于给每根骨段勾了边，层次立刻出来。
            p.setPen(QPen(QColor(0x10, 0x10, 0x10, 200), wpx + 2.0,
                          Qt::SolidLine, Qt::RoundCap));
            p.drawLine(a, b);
            p.setPen(QPen(c, wpx, Qt::SolidLine, Qt::RoundCap));
            p.drawLine(a, b);
            // 关节点
            p.setPen(Qt::NoPen);
            p.setBrush(mocap::theme::chrome());
            p.drawEllipse(a, wpx * 0.52, wpx * 0.52);
            p.setBrush(c);
            p.drawEllipse(a, wpx * 0.30, wpx * 0.30);
            cx += dx * L; cy += dy * L;
        }
        // 指尖：画成小圆点，比骨段稍亮一点，作为收尾
        const QColor tc = srcColor(std::clamp(snap_.segSource[size_t(1 + fi * 3 + 2)], 0, 3));
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(0x10, 0x10, 0x10, 200));
        p.drawEllipse(to(cx, cy), 3.0, 3.0);
        p.setBrush(tc.lighter(125));
        p.drawEllipse(to(cx, cy), 2.1, 2.1);
    }

    // 图例：四种来源各画一小段，顺便统计各占几段 —— 这个计数就是"现在链路
    // 到底解出来多少"的一句话总结。
    QFont f = p.font(); f.setPointSizeF(std::max(6.5, f.pointSizeF() - 1.0)); p.setFont(f);
    const QFontMetricsF fm(f);
    int cnt[4] = {0, 0, 0, 0};
    for (int i = 0; i < 16; ++i) cnt[std::clamp(snap_.segSource[size_t(i)], 0, 3)]++;
    const double ly = r.bottom() - fm.height() - 2;
    double lx = r.left() + 6;
    for (int k = 3; k >= 0; --k) {
        p.setPen(QPen(srcColor(k), 3, Qt::SolidLine, Qt::RoundCap));
        p.drawLine(QPointF(lx, ly + fm.height() * 0.55), QPointF(lx + 10, ly + fm.height() * 0.55));
        p.setPen(cnt[k] ? textColor() : dimColor());
        const QString t = QStringLiteral("%1%2").arg(QString::fromUtf8(kSrcName[k])).arg(cnt[k]);
        p.drawText(QPointF(lx + 14, ly + fm.ascent()), t);
        lx += 14 + fm.horizontalAdvance(t) + 12;
    }
    p.restore();
}

// ---------------------------------------------------------------------------
// 模式①：关节角（M3DS）—— 默认，也是遥操作真正吃的那份
// ---------------------------------------------------------------------------
void HandOutputView::paintJoints(QPainter& p) {
    const QFontMetricsF fm(p.font());
    const double W = width();

    // ---- 单列还是双列 ----
    // 这个面板从"右侧竖条"挪到了"底部横条"，形状从窄高变成了宽扁。16 个关节
    // 排成一列的话，上面挤成一坨、右边空一大片、下面还要滚 —— 正是之前
    // 关节角下方那块浪费。够宽就拆两列，表格高度直接减半，宽度也吃满。
    // 阈值 720：低于这个宽度双列会把行程条压得没法看，退回单列。
    const int cols = (W >= 720.0) ? 2 : 1;
    const int rowsPerCol = (16 + cols - 1) / cols;
    // 表头 2 行 + 关节 rowsPerCol 行 + 腕部块 7 行，按这个总数把行距铺开
    const double lh = std::clamp((height() - 12.0) / (rowsPerCol + 9.0),
                                 fm.height() + 1.0, fm.height() + 9.0);
    double y = 4.0;

    // ---- 表头 ----
    p.setPen(dimColor());
    p.drawText(QPointF(6, y + fm.ascent()),
               QStringLiteral("M3DS · 关节角16维（弧度→这里按角度显示）"));
    y += lh;

    // 每一列的列内坐标。原来是一组写死的 cName/cRaw/cOut/cBar，现在按列号算，
    // 单列时 c==0 得到的结果跟原来一致。
    struct ColGeom { double name, raw, out, bar, barW, left, right; };
    const double gutter = 14.0;
    const double colW = (W - 8.0 - gutter * (cols - 1)) / cols;
    const double numW = fm.horizontalAdvance(QStringLiteral("-000.0 "));
    auto colGeom = [&](int c) {
        ColGeom g{};
        g.left  = 4.0 + c * (colW + gutter);
        g.right = g.left + colW;
        g.name  = g.left + 2.0;
        g.raw   = g.left + std::max(90.0, colW * 0.30);
        g.out   = g.raw + numW + 6.0;
        g.bar   = g.out + numW + 8.0;
        g.barW  = std::max(24.0, g.right - g.bar - 40.0);
        return g;
    };
    const ColGeom g0 = colGeom(0);

    p.setPen(dimColor());
    for (int c = 0; c < cols; ++c) {
        const ColGeom g = colGeom(c);
        p.drawText(QPointF(g.name, y + fm.ascent()), QStringLiteral("关节"));
        p.drawText(QPointF(g.raw,  y + fm.ascent()), QStringLiteral("解算°"));
        p.drawText(QPointF(g.out,  y + fm.ascent()), QStringLiteral("输出°"));
        p.drawText(QPointF(g.bar,  y + fm.ascent()), QStringLiteral("行程/状态"));
    }
    const double headY = y;
    y += lh - 1;
    p.setPen(gridColor());
    p.drawLine(QPointF(4, y), QPointF(W - 4, y));
    if (cols > 1)   // 两列之间画一条竖分隔线，扫读时不会串行
        p.drawLine(QPointF(colGeom(1).left - gutter / 2, headY - lh + 2),
                   QPointF(colGeom(1).left - gutter / 2, y + rowsPerCol * lh + 3));
    const double tableTop = y + 3;
    y = tableTop;

    const auto& lim = jointLimits();
    for (int i = 0; i < 16; ++i) {
        const int col = i / rowsPerCol;
        const ColGeom g = colGeom(col);
        const double cName = g.name, cRaw = g.raw, cOut = g.out, cBar = g.bar, barW = g.barW;
        y = tableTop + (i % rowsPerCol) * lh;
        const double lo = lim[size_t(i)].lo, hi = lim[size_t(i)].hi;
        const double out = snap_.q[size_t(i)];
        const bool fresh = (snap_.rawFlags >> jointFinger(i)) & 1u;
        // 掌指屈/展要腕部系，腕部系失效时它是"保持值"而不是新算的 —— 这两种
        // 情况在数字上看不出区别，必须标出来（bit5 = mcpValid）。
        const bool isMcp = (i == 0 || i == 1 || i == 4 || i == 5 || i == 7 ||
                            i == 8 || i == 10 || i == 11 || i == 13 || i == 14);
        const bool mcpOk = (snap_.rawFlags >> 5) & 1u;
        const bool held  = !fresh || (isMcp && !mcpOk);

        if (i % 2 == 0) p.fillRect(QRectF(g.left - 2, y - 1, colW, lh), QColor(0xFF, 0xFF, 0xFF, 8));

        p.setPen(held ? dimColor() : textColor());
        p.drawText(QPointF(cName, y + fm.ascent()), QString::fromUtf8(kJointName[i]));

        if (snap_.hasRaw) {
            p.setPen(dimColor());
            p.drawText(QPointF(cRaw, y + fm.ascent()), f2(snap_.qRaw[size_t(i)] * kRad2Deg, 1));
        }
        p.setPen(held ? dimColor() : textColor());
        p.drawText(QPointF(cOut, y + fm.ascent()), f2(out * kRad2Deg, 1));

        // 行程条：位置 = 该关节在【目标行程】里的归一化位置。贴到两端 = 触限，
        // 长期触限说明 ROM 没标定或标坏了（信号被压平，机械手会顶死）。
        const double span = std::max(1e-6, hi - lo);
        const double u = std::clamp((out - lo) / span, 0.0, 1.0);
        const QRectF bar(cBar, y + 3, barW, lh - 7);
        p.setPen(Qt::NoPen);
        p.fillRect(bar, mocap::theme::chrome());
        const bool atLimit = (u < 0.005 || u > 0.995);
        p.fillRect(QRectF(bar.left(), bar.top(), bar.width() * u, bar.height()),
                   atLimit ? warnColor() : (held ? mocap::theme::textFaint() : mocap::theme::accent()));
        // 零位刻度：负行程的关节(拇指CMC、四指外展)零位不在最左，标一下
        if (lo < -1e-6) {
            const double u0 = std::clamp((0.0 - lo) / span, 0.0, 1.0);
            p.setPen(QPen(mocap::theme::lineHard(), 1));
            p.drawLine(QPointF(bar.left() + bar.width() * u0, bar.top()),
                       QPointF(bar.left() + bar.width() * u0, bar.bottom()));
        }
        p.setPen(atLimit ? warnColor() : dimColor());
        p.drawText(QPointF(cBar + barW + 5, y + fm.ascent()),
                   atLimit ? QStringLiteral("触限") : (held ? QStringLiteral("保持") : QStringLiteral("新算")));
    }

    // ---- 腕部 ----
    // 表格现在可能是双列，y 已经不是"最后一行的位置"了，得按整张表的底边算。
    y = tableTop + rowsPerCol * lh + 4;
    const double cRaw = g0.raw;   // 腕部块始终排在第一列的坐标下
    const double wristTop = y;
    p.setPen(gridColor());
    p.drawLine(QPointF(4, y), QPointF(W - 4, y));
    y += 5;
    const bool wristOk = snap_.hasQuat ? ((snap_.flags & 1u) != 0)
                                       : ((snap_.rawFlags >> 6) & 1u);
    p.setPen(dimColor());
    p.drawText(QPointF(6, y + fm.ascent()), QStringLiteral("腕部位姿"));
    p.setPen(wristOk ? okColor() : warnColor());
    p.drawText(QPointF(cRaw, y + fm.ascent()),
               wristOk ? QStringLiteral("本帧解出") : QStringLiteral("外推(手背<3点)"));
    y += lh;
    p.setPen(textColor());
    p.drawText(QPointF(6, y + fm.ascent()),
               QStringLiteral("pos  %1  %2  %3  mm")
                   .arg(f2(snap_.wristPos.x(), 1), 8).arg(f2(snap_.wristPos.y(), 1), 8)
                   .arg(f2(snap_.wristPos.z(), 1), 8));
    y += lh;
    for (int r = 0; r < 3; ++r) {
        p.setPen(dimColor());
        p.drawText(QPointF(6, y + fm.ascent()),
                   QStringLiteral("R%1   %2  %3  %4")
                       .arg(r)
                       .arg(f2(snap_.wristR[size_t(r * 3 + 0)], 4), 8)
                       .arg(f2(snap_.wristR[size_t(r * 3 + 1)], 4), 8)
                       .arg(f2(snap_.wristR[size_t(r * 3 + 2)], 4), 8));
        y += lh;
    }
    const auto wq = matToQuat(snap_.wristR);
    p.setPen(dimColor());
    p.drawText(QPointF(6, y + fm.ascent()),
               QStringLiteral("quat %1 %2 %3 %4")
                   .arg(f2(wq[0], 3), 7).arg(f2(wq[1], 3), 7)
                   .arg(f2(wq[2], 3), 7).arg(f2(wq[3], 3), 7));

    // ---- 腕部块右侧的仪表带 ----
    // pos/R9/quat 那一栏只占了左边不到三分之一，右边一直是空的。这里按可用
    // 宽度依次摆下去，放不下的就不摆 —— 窄面板下退化成原来的样子，不会挤爆。
    // 腕部块 6 行 + 表头，放不下多少行就报多少
    {
        // 【为什么要减掉容差】原来是 int((needBottom-height())/lh)+1，
        // 只要 needBottom 比 height() 大【哪怕 0.1 像素】就报"还差 1 行"。
        // 而 needBottom = y + lh*2 本身就留了两行余量，于是内容明明已经
        // 完整画出来了，右上角还挂着"还有 1 行未显示" —— 提示反过来变成
        // 噪声，看久了就会连真的溢出也一起忽略掉。
        //
        // 改成只在【真的少了至少一整行】时才报：不足一行的溢出（多半是
        // 抗锯齿和行距取整的零头）忽略掉。
        const double needBottom = y + lh * 2;
        const double over = needBottom - height();
        if (over > lh * 0.5)
            hiddenRows_ = std::max(hiddenRows_, int(std::ceil(over / lh)));
    }

    const double instrTop = wristTop + 2;
    const double instrH   = std::max(0.0, height() - instrTop - 4);
    // 仪表起点必须让开左边腕部文本的【实际】宽度 —— 按 W 的百分比估会在窄
    // 面板下压到 "R0 1.0000 0.0000 0.0000" 那几行上面。
    const double wristTextW =
        6.0 + fm.horizontalAdvance(QStringLiteral("R0   -0.0000  -0.0000  -0.0000")) + 18.0;
    double ix = std::max(wristTextW, W * 0.28);
    if (instrH >= 58 && W - ix - 6 >= 260) {
        auto place = [&](double w) -> QRectF {
            const QRectF box(ix, instrTop + 7, w, instrH - 9);
            ix += w + 12;
            return box;
        };
        // 优先级从高到低：姿态罗盘 > 欧拉角 > 链路 > 逐指 > 周期折线。
        // 宽度不够时后面的自然就不画了。
        if (W - ix - 6 >= 110 && instrH >= 70) {
            const QRectF b = place(std::min(120.0, instrH * 1.15));
            drawHudFrame(p, b, QStringLiteral("腕部姿态"));
            drawAttitudeGizmo(p, b);
        }
        if (W - ix - 6 >= 170) {
            const QRectF b = place(190);
            drawHudFrame(p, b, QStringLiteral("欧拉角"));
            drawEulerGauges(p, b.adjusted(2, 6, -2, -4));
        }
        if (W - ix - 6 >= 250) {
            const QRectF b = place(std::min(300.0, W - ix - 18));
            drawHudFrame(p, b, QStringLiteral("链路"));
            drawPipeline(p, b.adjusted(2, 6, -2, -2));
        }
        if (W - ix - 6 >= 130) {
            const QRectF b = place(140);
            drawHudFrame(p, b, QStringLiteral("逐指新鲜度"));
            drawFingerBars(p, b.adjusted(2, 6, -2, -2));
        }
        if (W - ix - 6 >= 150) {
            const QRectF b = place(W - ix - 18);
            drawHudFrame(p, b, QStringLiteral("输出周期"));
            drawStepSpark(p, b.adjusted(2, 6, -2, -2));
        }
    }
}

// ---------------------------------------------------------------------------
// 模式②：分段四元数（M3DQ）
// ---------------------------------------------------------------------------
void HandOutputView::paintQuats(QPainter& p) {
    const QFontMetricsF fm(p.font());
    const double lh = std::clamp((height() - 12.0) / 22.0, fm.height() + 1.0, fm.height() + 9.0);
    const double W = width();
    double y = 4.0;

    if (!snap_.hasQuat) {
        p.setPen(dimColor());
        p.drawText(rect(), Qt::AlignCenter,
                   QStringLiteral("没有收到 M3DQ\n（分段四元数输出被关掉了？）"));
        return;
    }

    const double qw = fm.horizontalAdvance(QStringLiteral("-0.000 "));
    const double cName = 6.0;
    const double cSrc  = 62.0;
    const double cLocal = 104.0;
    const double cWorld = cLocal + qw * 4 + 12;
    // 表格本身最多用到这里，右边的宽度全部让给仪表区 —— 原来表格右边一大片
    // 是纯留白（数字列早就排完了，只是没人接管剩下的宽度）。
    const double tableW = cWorld + qw * 4 + 14;
    const bool showWorld = (W > tableW);
    const double instrX = showWorld ? tableW + 10 : 0.0;
    const bool hasInstr = showWorld && (W - instrX > 240);

    p.setPen(dimColor());
    p.drawText(QPointF(cName, y + fm.ascent()),
               QStringLiteral("M3DQ · 16 段  (w,x,y,z)"));
    y += lh;
    p.drawText(QPointF(cName,  y + fm.ascent()), QStringLiteral("分段"));
    p.drawText(QPointF(cSrc,   y + fm.ascent()), QStringLiteral("来源"));
    p.drawText(QPointF(cLocal, y + fm.ascent()), QStringLiteral("相对父节点(Unity用)"));
    if (showWorld) p.drawText(QPointF(cWorld, y + fm.ascent()), QStringLiteral("世界系"));
    y += lh - 1;
    p.setPen(gridColor());
    p.drawLine(QPointF(4, y), QPointF((hasInstr ? tableW : W) - 4, y));
    y += 3;
    const double tableTopY = y;

    for (int s = 0; s < 16; ++s) {
        if (s % 2 == 0)
            p.fillRect(QRectF(2, y - 1, (hasInstr ? tableW : W) - 4, lh), QColor(0xFF, 0xFF, 0xFF, 8));
        const int src = std::clamp(snap_.segSource[size_t(s)], 0, 3);
        p.setPen(src >= 2 ? textColor() : dimColor());
        p.drawText(QPointF(cName, y + fm.ascent()), QString::fromUtf8(kSegName[s]));
        p.setPen(srcColor(src));
        p.drawText(QPointF(cSrc, y + fm.ascent()), QString::fromUtf8(kSrcName[src]));
        p.setPen(src >= 2 ? textColor() : dimColor());
        for (int k = 0; k < 4; ++k) {
            p.drawText(QPointF(cLocal + qw * k, y + fm.ascent()),
                       f2(snap_.quatL[size_t(s * 4 + k)], 3));
            if (showWorld)
                p.drawText(QPointF(cWorld + qw * k, y + fm.ascent()),
                           f2(snap_.quatW[size_t(s * 4 + k)], 3));
        }
        y += lh;
    }

    y += 4;
    p.setPen(gridColor());
    // 【只画到表格区右边，不能横穿到仪表区】原来是 W-4（整个面板宽），
    // 于是这条分隔线、以及下面的 flags/逐指有效两行，全都伸进了右边
    // "分段来源·手部拓扑"和"链路"那两个框里，压在它们的边框和内容上。
    // 表格区的右界就是 instrX（仪表区起点）；没有仪表区时才铺满。
    const double tableRight = hasInstr ? (instrX - 8.0) : (W - 4.0);
    p.drawLine(QPointF(4, y), QPointF(tableRight, y));
    y += 5;
    p.setPen(dimColor());
    // 【下面这几行文字同样不能越界】它们比分隔线更容易被忽略：文字画到
    // 拓扑图上会跟手掌线条叠在一起，看起来像是拓扑图自己糊了。
    p.save();
    p.setClipRect(QRectF(0, 0, tableRight + 4.0, height()));
    p.drawText(QPointF(6, y + fm.ascent()),
               QStringLiteral("flags 0x%1   腕部%2  掌指%3")
                   .arg(snap_.flags, 4, 16, QLatin1Char('0'))
                   .arg((snap_.flags & 1u) ? QStringLiteral("✓") : QStringLiteral("✗"))
                   .arg((snap_.flags & 2u) ? QStringLiteral("✓") : QStringLiteral("✗")));
    y += lh;
    QString fingers;
    static const char* const fn[5] = {"拇", "食", "中", "无", "小"};
    for (int f = 0; f < 5; ++f)
        fingers += QString::fromUtf8(fn[f]) +
                   ((snap_.flags >> (8 + f)) & 1u ? QStringLiteral("✓ ") : QStringLiteral("✗ "));
    p.drawText(QPointF(6, y + fm.ascent()), QStringLiteral("逐指有效 ") + fingers);
    if (!showWorld) {
        y += lh;
        p.drawText(QPointF(6, y + fm.ascent()), QStringLiteral("(拉宽面板可同时看世界系)"));
    }
    p.restore();

    // ---- 右侧仪表区 ----
    if (hasInstr) {
        const double top = tableTopY - lh - 4;
        const double bot = height() - 6;
        const double availW = W - instrX - 6;
        // 拓扑图是这一页的主角：16 段的来源分布画成一只手，比逐行读那一列快
        const double topoW = std::min(availW * 0.46, 260.0);
        const QRectF topoBox(instrX, top + 7, topoW, bot - top - 9);
        drawHudFrame(p, topoBox, QStringLiteral("分段来源 · 手部拓扑"));
        drawHandTopology(p, topoBox.adjusted(4, 8, -4, -4));

        double x2 = instrX + topoW + 12;
        const double restW = W - x2 - 6;
        if (restW >= 240) {
            const double h2 = (bot - top - 9);
            const QRectF pipeBox(x2, top + 7, restW, h2 * 0.42);
            drawHudFrame(p, pipeBox, QStringLiteral("链路"));
            drawPipeline(p, pipeBox.adjusted(2, 6, -2, -2));

            const QRectF sparkBox(x2, pipeBox.bottom() + 12, restW, h2 - pipeBox.height() - 12);
            if (sparkBox.height() >= 44) {
                drawHudFrame(p, sparkBox, QStringLiteral("输出周期"));
                drawStepSpark(p, sparkBox.adjusted(2, 6, -2, -2));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 模式③：原始报文 —— 真正在网线上跑的那串字节
// ---------------------------------------------------------------------------
void HandOutputView::paintBytes(QPainter& p) {
    // 【重排】原来两条 dump 上下堆着、每行固定 16 字节，于是右边空一大半、
    // 下面被高度截断，还要在最底下写一行"高度不够"——而那行本身也可能正好
    // 被截掉。现在：左右两栏并排（M3DS | M3DQ），每行的字节数按栏宽算，
    // 宽面板下一行能塞 32 字节，行数直接减半。
    //
    // 另外给字节按【协议字段】上色。头一行本来就写着 magic|ts_ns|pos3|R9|q16，
    // 但对着一片 00 根本对不上是哪段；上了色就能直接看出"pos3 那 12 个字节
    // 全是 0"这种事，比逐个数偏移快得多。
    const QFontMetricsF fm(p.font());
    const double lh = fm.height() + 1.0;
    const double W = width(), H = height();
    const double chw = fm.horizontalAdvance(QLatin1Char('0'));

    // ---- 顶部状态行 ----
    double y = 4.0;
    p.setPen(udpOn_ ? okColor() : badColor());
    p.drawText(QPointF(6, y + fm.ascent()),
               QStringLiteral("目标 %1   UDP推送 %2")
                   .arg(udpTarget_.isEmpty() ? QStringLiteral("(未知)") : udpTarget_)
                   .arg(udpOn_ ? QStringLiteral("已开")
                               : QStringLiteral("【关闭中，Unity 收不到】")));
    y += lh + 2;
    const double bodyTop = y;

    // ---- 协议字段划分（字节偏移区间 -> 颜色 + 名字）----
    struct Field { int from, to; QColor c; const char* n; };
    const QVector<Field> fS = {
        {  0,   4, QColor(0x9C, 0xDC, 0xFE), "magic"  },
        {  4,  12, QColor(0xC5, 0x92, 0xE0), "ts_ns"  },
        { 12,  24, mocap::theme::good(), "pos3"   },
        { 24,  60, QColor(0xD7, 0xBA, 0x7D), "R9"     },
        { 60, 124, mocap::theme::marker(), "q16"    },
    };
    const QVector<Field> fQ = {
        {   0,   4, QColor(0x9C, 0xDC, 0xFE), "magic" },
        {   4,  12, QColor(0xC5, 0x92, 0xE0), "ts_ns" },
        {  12,  16, QColor(0xE0, 0xA0, 0x40), "flags" },
        {  16,  28, mocap::theme::good(), "pos3"  },
        {  28, 284, mocap::theme::marker(), "qW64"  },
        { 284, 540, QColor(0x4E, 0xC9, 0xB0), "qL64"  },
        { 540, 556, mocap::theme::bad(), "src16" },
    };
    auto fieldOf = [](const QVector<Field>& fs, int off) -> const Field* {
        for (const Field& f : fs) if (off >= f.from && off < f.to) return &f;
        return nullptr;
    };

    // ---- 一栏 dump ----
    // 返回没画下的行数，交给顶部的溢出药丸汇总。
    auto column = [&](const QRectF& box, const QByteArray& b,
                      const QVector<Field>& fs, const QString& title) -> int {
        // 每行字节数：先按栏宽算能塞几个 "XX "，再往下取到 8 的倍数（对齐好读）
        const double addrW = chw * 6.0;
        int per = int((box.width() - addrW - 4) / (chw * 3.0));
        per = std::max(8, (per / 8) * 8);
        per = std::min(per, 32);

        double yy = box.top();
        // 栏标题 + 字段图例
        p.setPen(dimColor());
        p.drawText(QPointF(box.left(), yy + fm.ascent()),
                   QStringLiteral("%1  %2B").arg(title).arg(b.size()));
        double lx = box.left() + fm.horizontalAdvance(QStringLiteral("%1  %2B  ").arg(title).arg(b.size()));
        for (const Field& f : fs) {
            const QString t = QString::fromUtf8(f.n);
            const double tw = fm.horizontalAdvance(t);
            if (lx + tw + 14 > box.right()) break;
            p.setPen(Qt::NoPen); p.setBrush(f.c);
            p.drawRect(QRectF(lx, yy + fm.ascent() - 6, 6, 6));
            p.setPen(f.c);
            p.drawText(QPointF(lx + 9, yy + fm.ascent()), t);
            lx += 9 + tw + 9;
        }
        yy += lh + 2;

        const int totalRows = (b.size() + per - 1) / per;
        const int fitRows = std::max(0, int((box.bottom() - yy) / lh));
        const int rows = std::min(totalRows, fitRows);
        for (int i = 0; i < rows; ++i) {
            const int base = i * per;
            double x = box.left();
            p.setPen(mocap::theme::textFaint());
            p.drawText(QPointF(x, yy + fm.ascent()),
                       QStringLiteral("%1").arg(base, 4, 16, QLatin1Char('0')).toUpper());
            x += addrW;
            for (int j = 0; j < per; ++j) {
                const int k = base + j;
                if (k >= b.size()) break;
                const Field* f = fieldOf(fs, k);
                const quint8 v = quint8(b[k]);
                // 全 0 的字节压暗：一屏 00 里，非零的那几个才是信息
                QColor c = f ? f->c : textColor();
                if (v == 0) c = QColor(c.red(), c.green(), c.blue(), 90);
                p.setPen(c);
                p.drawText(QPointF(x, yy + fm.ascent()),
                           QStringLiteral("%1").arg(v, 2, 16, QLatin1Char('0')).toUpper());
                x += chw * 3.0;
            }
            yy += lh;
        }
        return totalRows - rows;
    };

    // ---- 左右两栏；窄面板退回上下堆叠 ----
    int hidden = 0;
    if (W >= 900) {
        const double colW = (W - 18) / 2.0;
        hidden += column(QRectF(6, bodyTop, colW, H - bodyTop - 4), m3ds_, fS,
                         QStringLiteral("M3DS"));
        hidden += column(QRectF(6 + colW + 6, bodyTop, colW, H - bodyTop - 4), m3dq_, fQ,
                         QStringLiteral("M3DQ"));
        p.setPen(QPen(gridColor(), 1));
        p.drawLine(QPointF(6 + colW + 3, bodyTop), QPointF(6 + colW + 3, H - 4));
    } else {
        const double half = (H - bodyTop - 8) / 2.0;
        hidden += column(QRectF(6, bodyTop, W - 12, half), m3ds_, fS, QStringLiteral("M3DS"));
        hidden += column(QRectF(6, bodyTop + half + 6, W - 12, half), m3dq_, fQ,
                         QStringLiteral("M3DQ"));
    }
    hiddenRows_ = hidden;
}

QString HandOutputView::asText() const {
    QString t;
    t += QStringLiteral("# hand output @ %1\n")
             .arg(QDateTime::currentDateTime().toString(Qt::ISODateWithMs));
    t += QStringLiteral("ts_ns(M3DS)=%1  ts_ns(M3DQ)=%2  flags=0x%3\n")
             .arg(snap_.poseTsNs).arg(snap_.quatTsNs).arg(snap_.flags, 4, 16, QLatin1Char('0'));
    t += QStringLiteral("wristPos_mm  %1 %2 %3\n")
             .arg(snap_.wristPos.x()).arg(snap_.wristPos.y()).arg(snap_.wristPos.z());
    t += QStringLiteral("wristR       ");
    for (int i = 0; i < 9; ++i) t += f2(snap_.wristR[size_t(i)], 6) + QLatin1Char(' ');
    t += QLatin1Char('\n');
    t += QStringLiteral("\n# joint  raw_deg  out_deg  out_rad  fresh\n");
    for (int i = 0; i < 16; ++i) {
        t += QStringLiteral("%1  %2  %3  %4  %5\n")
                 .arg(QString::fromUtf8(kJointName[i]), -10)
                 .arg(f2(snap_.qRaw[size_t(i)] * kRad2Deg, 2), 8)
                 .arg(f2(snap_.q[size_t(i)] * kRad2Deg, 2), 8)
                 .arg(f2(snap_.q[size_t(i)], 5), 9)
                 .arg((snap_.rawFlags >> jointFinger(i)) & 1u ? 1 : 0);
    }
    t += QStringLiteral("\n# seg  src  localWXYZ  worldWXYZ\n");
    for (int s = 0; s < 16; ++s) {
        t += QStringLiteral("%1  %2 ").arg(QString::fromUtf8(kSegName[s]), -9)
                 .arg(QString::fromUtf8(kSrcName[std::clamp(snap_.segSource[size_t(s)], 0, 3)]), -4);
        for (int k = 0; k < 4; ++k) t += f2(snap_.quatL[size_t(s * 4 + k)], 5) + QLatin1Char(' ');
        t += QStringLiteral("  ");
        for (int k = 0; k < 4; ++k) t += f2(snap_.quatW[size_t(s * 4 + k)], 5) + QLatin1Char(' ');
        t += QLatin1Char('\n');
    }
    return t;
}

// 完整的十六进制 dump。跟 asText() 分开是因为报文只在"原始报文"模式才生成，
// 复制按钮会先按需补打一次再调这个，保证复制到的内容永远是完整的。
QString HandOutputView::rawText() const {
    QString t;
    t += QStringLiteral("# M3DS %1 bytes\n").arg(m3ds_.size());
    t += hexDump(m3ds_, 0, nullptr);
    t += QStringLiteral("# M3DQ %1 bytes\n").arg(m3dq_.size());
    t += hexDump(m3dq_, 0, nullptr);
    return t;
}

// ===========================================================================
// HandOutputMonitor
// ===========================================================================

HandOutputMonitor::HandOutputMonitor(QWidget* parent) : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(2, 2, 2, 2);
    root->setSpacing(3);

    auto* bar = new QHBoxLayout();
    bar->setSpacing(4);

    modeCombo_ = new QComboBox;
    modeCombo_->addItems({QStringLiteral("关节角(M3DS)"),
                          QStringLiteral("分段四元数(M3DQ)"),
                          QStringLiteral("原始报文")});
    modeCombo_->setToolTip(QStringLiteral(
        "关节角：遥操作/骨骼动画真正吃的那 16 个数，带解算原始值对照和行程条。\n"
        "分段四元数：16 段的世界系/相对父节点姿态 + 每段的来源(预测还是测量)。\n"
        "原始报文：UDP 上真正跑的字节，跟 Unity 端/Wireshark 逐字节对。"));
    bar->addWidget(modeCombo_);

    freezeChk_ = new QCheckBox(QStringLiteral("冻结"));
    freezeChk_->setToolTip(QStringLiteral(
        "停住画面看清某一帧的数字。数据照收不误(计数、CSV 都继续)，只是不刷新显示。"));
    bar->addWidget(freezeChk_);

    copyBtn_ = new QPushButton(QStringLiteral("复制"));
    copyBtn_->setToolTip(QStringLiteral("把当前这一帧的全部输出(含完整十六进制报文)复制到剪贴板。"));
    bar->addWidget(copyBtn_);

    csvBtn_ = new QPushButton(QStringLiteral("记CSV"));
    csvBtn_->setToolTip(QStringLiteral(
        "把每一帧输出逐行写进 CSV(文档目录)。列含 解算原始角 / 实际输出角 / 腕部位姿 / flags，\n"
        "用来事后画曲线找\"哪一帧开始不对\"，比盯着屏幕看数字靠谱得多。"));
    bar->addWidget(csvBtn_);
    bar->addStretch(1);
    root->addLayout(bar);

    view_ = new HandOutputView(this);
    root->addWidget(view_, 1);

    // 【固定宽度 + 不换行】理由同 PointCloudTestDialog 里 skeletonDiagLabel_ 的
    // 那段注释：这行字每 100ms 变一次，开自动换行会让整个面板反复重排。
    statusLabel_ = new QLabel(QStringLiteral("未启用"), this);
    statusLabel_->setWordWrap(false);
    statusLabel_->setTextFormat(Qt::PlainText);
    statusLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    root->addWidget(statusLabel_);

    timer_ = new QTimer(this);
    timer_->setInterval(100);        // 10Hz —— 人眼够用，GUI 成本恒定
    connect(timer_, &QTimer::timeout, this, &HandOutputMonitor::onTick);

    connect(modeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i) {
        view_->setMode(HandOutputView::Mode(i));
        // 【别漏】报文只在"原始报文"模式下才逐帧打包（省掉 120fps 白打 680 字节）。
        // 刚切过来时手上还没有字节，如果这时数据流恰好停了（手出视野/冻结），
        // 面板会一直显示 0B —— 看起来像"根本没在发"，是个会误导人的假象。
        // 所以切过来的当下先按当前快照补打一次。
        if (i == HandOutputView::Bytes) ensureRawPackets();
    });
    connect(copyBtn_, &QPushButton::clicked, this, &HandOutputMonitor::copyToClipboard);
    connect(csvBtn_, &QPushButton::clicked, this, &HandOutputMonitor::toggleCsv);

    setMinimumWidth(340);
}

HandOutputMonitor::~HandOutputMonitor() {
    if (csvOut_) csvOut_->flush();
}

void HandOutputMonitor::setActive(bool on) {
    active_ = on;
    if (on) {
        snap_ = HandOutputSnapshot();
        posePrev_ = quatPrev_ = 0;
        rateClock_.restart();
        timer_->start();
        statusLabel_->setText(QStringLiteral("等待输出…"));
    } else {
        timer_->stop();
    }
}

void HandOutputMonitor::setUdpStatus(bool enabled, const QString& target) {
    view_->setUdpStatus(enabled, target);
}

// ---- 数据入口：只做赋值，一个 UI 调用都没有 --------------------------------

void HandOutputMonitor::onHandPose(QVector3D wristPos, QVector<double> wristRot9,
                                   QVector<double> jointAngles16, qint64 ts_ns) {
    if (!active_) return;
    if (wristRot9.size() != 9 || jointAngles16.size() != 16) return;
    snap_.hasPose = true;
    snap_.wristPos = wristPos;
    for (int i = 0; i < 9; ++i)  snap_.wristR[size_t(i)] = wristRot9[i];
    for (int i = 0; i < 16; ++i) snap_.q[size_t(i)] = jointAngles16[i];
    snap_.poseTsNs = ts_ns;
    ++snap_.poseCount;
    snap_.lastArrivalMs = QDateTime::currentMSecsSinceEpoch();
    // ts 的增量（同一时基内部相减，原点抵消）。见头文件 tsStepMs 的说明。
    if (ts_ns > 0) {
        if (snap_.prevTsNs > 0 && ts_ns > snap_.prevTsNs) {
            const double stepMs = double(ts_ns - snap_.prevTsNs) / 1.0e6;
            // 相机重启/切换会让 ts 跳变，超过 1 秒的增量不当成周期
            if (stepMs < 1000.0) {
                snap_.tsStepMs = (snap_.tsStepValid)
                                     ? (0.1 * stepMs + 0.9 * snap_.tsStepMs) : stepMs;
                snap_.tsStepValid = true;
            }
        }
        snap_.prevTsNs = ts_ns;
    }

    // 原始报文只在该模式下才打包（120fps 下白打 600 字节没必要）
    if (view_->mode() == HandOutputView::Bytes)
        lastM3ds_ = wire::buildM3DS(wristPos, wristRot9, jointAngles16, ts_ns);

    if (csvOut_) writeCsvRow();
}

void HandOutputMonitor::onSegmentQuats(QVector3D wristPos, QVector<double> quatWorld64,
                                       QVector<double> quatLocal64, QVector<int> segSource16,
                                       quint32 flags, qint64 ts_ns) {
    if (!active_) return;
    if (quatWorld64.size() != 64 || quatLocal64.size() != 64 || segSource16.size() != 16) return;
    snap_.hasQuat = true;
    snap_.wristPos = wristPos;
    for (int i = 0; i < 64; ++i) { snap_.quatW[size_t(i)] = quatWorld64[i];
                                   snap_.quatL[size_t(i)] = quatLocal64[i]; }
    for (int i = 0; i < 16; ++i) snap_.segSource[size_t(i)] = segSource16[i];
    snap_.flags = flags;
    snap_.quatTsNs = ts_ns;
    ++snap_.quatCount;
    snap_.lastArrivalMs = QDateTime::currentMSecsSinceEpoch();

    if (view_->mode() == HandOutputView::Bytes)
        lastM3dq_ = wire::buildM3DQ(wristPos, quatWorld64, quatLocal64, segSource16, flags, ts_ns);
}

void HandOutputMonitor::onJointAnglesDebug(QVector<double> rawRad16, QVector<double> outRad16,
                                           quint32 mask, qint64 ts_ns) {
    if (!active_) return;
    if (rawRad16.size() != 16) return;
    Q_UNUSED(outRad16);   // 输出角以 M3DS 那一路为准（那才是真发出去的）
    Q_UNUSED(ts_ns);
    snap_.hasRaw = true;
    for (int i = 0; i < 16; ++i) snap_.qRaw[size_t(i)] = rawRad16[i];
    snap_.rawFlags = mask;
}

// ---- 10Hz 刷新 -------------------------------------------------------------

void HandOutputMonitor::onTick() {
    if (!rateClock_.isValid()) rateClock_.start();
    const double el = double(rateClock_.elapsed());
    if (el >= 500.0) {
        snap_.poseHz = double(snap_.poseCount - posePrev_) * 1000.0 / el;
        snap_.quatHz = double(snap_.quatCount - quatPrev_) * 1000.0 / el;
        posePrev_ = snap_.poseCount;
        quatPrev_ = snap_.quatCount;
        rateClock_.restart();
    }

    if (!freezeChk_->isChecked()) {
        view_->setRawPackets(lastM3ds_, lastM3dq_);
        view_->setSnapshot(snap_);
    }

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const qint64 age = (snap_.lastArrivalMs > 0) ? (now - snap_.lastArrivalMs) : -1;
    QString s;
    if (snap_.poseCount == 0 && snap_.quatCount == 0) {
        s = QStringLiteral("等待输出…（勾选「叠加显示骨架」并让手进视野）");
    } else if (age > 1000) {
        s = QStringLiteral("⚠ 已 %1s 没有新输出　M3DS %2 包　M3DQ %3 包")
                .arg(age / 1000).arg(snap_.poseCount).arg(snap_.quatCount);
    } else {
        s = QStringLiteral("M3DS %1Hz/%2包　M3DQ %3Hz/%4包　帧间隔%5ms%6")
                .arg(snap_.poseHz, 0, 'f', 0).arg(snap_.poseCount)
                .arg(snap_.quatHz, 0, 'f', 0).arg(snap_.quatCount)
                .arg(snap_.tsStepValid ? QString::number(snap_.tsStepMs, 'f', 1)
                                       : QStringLiteral("—"))
                .arg(csvOut_ ? QStringLiteral("　CSV %1行").arg(csvRows_) : QString());
    }
    if (s != statusLabel_->text()) statusLabel_->setText(s);
}

// ---- CSV -------------------------------------------------------------------

void HandOutputMonitor::toggleCsv() {
    if (csvOut_) {
        csvOut_->flush();
        csvOut_.reset();
        csvFile_.reset();
        csvBtn_->setText(QStringLiteral("记CSV"));
        QMessageBox::information(this, QStringLiteral("输出记录"),
            QStringLiteral("已写入 %1 行：\n%2").arg(csvRows_).arg(csvPath_));
        return;
    }
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    csvPath_ = QStringLiteral("%1/handout_%2.csv")
                   .arg(dir, QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
    csvFile_ = std::make_unique<QFile>(csvPath_);
    if (!csvFile_->open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        csvFile_.reset();
        QMessageBox::warning(this, QStringLiteral("输出记录"),
                             QStringLiteral("打不开文件：%1").arg(csvPath_));
        return;
    }
    csvOut_ = std::make_unique<QTextStream>(csvFile_.get());
    csvRows_ = 0;
    QString h = QStringLiteral("wall_ms,ts_ns,wx,wy,wz");
    for (int i = 0; i < 9; ++i)  h += QStringLiteral(",R%1").arg(i);
    for (int i = 0; i < 16; ++i) h += QStringLiteral(",raw_deg_%1").arg(i);
    for (int i = 0; i < 16; ++i) h += QStringLiteral(",out_deg_%1").arg(i);
    h += QStringLiteral(",rawFlags,quatFlags");
    for (int i = 0; i < 16; ++i) h += QStringLiteral(",src%1").arg(i);
    *csvOut_ << h << "\n";
    csvBtn_->setText(QStringLiteral("■ 停止"));
}

void HandOutputMonitor::writeCsvRow() {
    if (!csvOut_) return;
    QString r = QStringLiteral("%1,%2,%3,%4,%5")
                    .arg(QDateTime::currentMSecsSinceEpoch()).arg(snap_.poseTsNs)
                    .arg(snap_.wristPos.x()).arg(snap_.wristPos.y()).arg(snap_.wristPos.z());
    for (int i = 0; i < 9; ++i)  r += QStringLiteral(",%1").arg(snap_.wristR[size_t(i)], 0, 'f', 6);
    for (int i = 0; i < 16; ++i) r += QStringLiteral(",%1").arg(snap_.qRaw[size_t(i)] * kRad2Deg, 0, 'f', 3);
    for (int i = 0; i < 16; ++i) r += QStringLiteral(",%1").arg(snap_.q[size_t(i)] * kRad2Deg, 0, 'f', 3);
    r += QStringLiteral(",%1,%2").arg(snap_.rawFlags).arg(snap_.flags);
    for (int i = 0; i < 16; ++i) r += QStringLiteral(",%1").arg(snap_.segSource[size_t(i)]);
    *csvOut_ << r << "\n";
    // 每 120 行落一次盘：既不至于崩了全丢，也不会每帧都同步 IO。
    if ((++csvRows_ % 120) == 0) csvOut_->flush();
}

// 按当前快照补打两个报文（只在手上还没有字节时）。切到原始报文模式、按复制
// 按钮时都会调 —— 保证屏幕上/剪贴板里的字节永远跟最近一帧对得上。
void HandOutputMonitor::ensureRawPackets() {
    if (lastM3ds_.isEmpty() && snap_.hasPose) {
        QVector<double> r9, q16;
        for (int i = 0; i < 9; ++i)  r9.push_back(snap_.wristR[size_t(i)]);
        for (int i = 0; i < 16; ++i) q16.push_back(snap_.q[size_t(i)]);
        lastM3ds_ = wire::buildM3DS(snap_.wristPos, r9, q16, snap_.poseTsNs);
    }
    if (lastM3dq_.isEmpty() && snap_.hasQuat) {
        QVector<double> w64, l64; QVector<int> s16;
        for (int i = 0; i < 64; ++i) { w64.push_back(snap_.quatW[size_t(i)]);
                                       l64.push_back(snap_.quatL[size_t(i)]); }
        for (int i = 0; i < 16; ++i) s16.push_back(snap_.segSource[size_t(i)]);
        lastM3dq_ = wire::buildM3DQ(snap_.wristPos, w64, l64, s16, snap_.flags, snap_.quatTsNs);
    }
    view_->setRawPackets(lastM3ds_, lastM3dq_);
}

void HandOutputMonitor::copyToClipboard() {
    ensureRawPackets();
    QString t = view_->asText();
    t += QStringLiteral("\n") + view_->rawText();
    if (auto* cb = QGuiApplication::clipboard()) cb->setText(t);
    statusLabel_->setText(QStringLiteral("已复制当前帧到剪贴板"));
}

} // namespace mocap
