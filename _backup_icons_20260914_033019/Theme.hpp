#pragma once
// ---------------------------------------------------------------------------
// VS Code 风格深色主题（配色取自 VS Code «Dark Modern»）。
//
// 为什么整体换成深色：这套软件的主要画面（相机预览、三角化 3D 视图、点云、
// 手部骨架）本来就全是深底 —— 原来浅色的外壳套着一堆深色画布，边界处对比
// 生硬，长时间盯着也刺眼。换成 VS Code 那套配色之后，外壳跟画布是同一个色
// 系，视觉上是连续的一整块，暗环境（动捕棚基本都是暗的）里也不再晃眼。
//
// 风格要点（这几条就是「VS Code 那种感觉」的来源，改动时请保持）：
//   · 层次靠 3 档灰堆出来，不靠阴影：#181818 外壳 / #1F1F1F 内容 / #313131 控件
//   · 直角。圆角一律 2~4px，绝不做大圆角卡片
//   · 分隔靠 1px 细线（#2B2B2B），不靠留白
//   · 一个蓝强调色 #0078D4，只用在焦点/选中/主按钮/状态栏
//   · 滚动条是方的、悬浮在内容上、无箭头
//   · 数字读数一律等宽字体（fps/时间戳/坐标 —— 数据就该长得像数据）
//
// 全项目只在这里定义颜色，其它地方引用常量，避免散落的硬编码色值。
// ---------------------------------------------------------------------------
#include <QString>
#include <QLatin1String>
#include <QColor>
#include <QIcon>
#include <QPixmap>
#include <QPainter>
#include <QPainterPath>
#include <QPolygonF>
#include <QPalette>
#include <cmath>

namespace mocap::theme {

    // ======================= 调色板 =======================
    // —— 三档底色：外壳 / 内容 / 控件 ——
    inline QColor chrome() { return QColor(0x18, 0x18, 0x18); }  // 活动栏·侧边栏·标签条·标题栏
    inline QColor editorBg() { return QColor(0x1F, 0x1F, 0x1F); }  // 内容区（等价 VS Code 的编辑器底）
    inline QColor inputBg() { return QColor(0x31, 0x31, 0x31); }  // 输入框/下拉/次级按钮
    inline QColor hoverBg() { return QColor(0x2A, 0x2D, 0x2E); }  // 悬停
    inline QColor selectBg() { return QColor(0x04, 0x39, 0x5E); }  // 列表选中（VS Code 的深蓝）

    // —— 线条 ——
    inline QColor lineSoft() { return QColor(0x2B, 0x2B, 0x2B); }  // 区块分隔
    inline QColor lineHard() { return QColor(0x3C, 0x3C, 0x3C); }  // 控件描边

    // —— 文字 ——
    inline QColor text() { return QColor(0xCC, 0xCC, 0xCC); }  // 正文
    inline QColor textBright() { return QColor(0xFF, 0xFF, 0xFF); }  // 高亮/选中
    inline QColor textDim() { return QColor(0x9D, 0x9D, 0x9D); }  // 次要说明
    inline QColor textFaint() { return QColor(0x6E, 0x6E, 0x6E); }  // 禁用

    // —— 强调 ——
    inline QColor accent() { return QColor(0x00, 0x78, 0xD4); }  // 焦点/主按钮/状态栏
    inline QColor accentHover() { return QColor(0x1F, 0x8A, 0xD8); }
    inline QColor accentDim() { return selectBg(); }

    // —— 语义色（沿用 VS Code 的编辑器诊断色，深底上读数清楚）——
    inline QColor good() { return QColor(0x89, 0xD1, 0x85); }  // 同步正常
    inline QColor warn() { return QColor(0xD7, 0xBA, 0x7D); }  // 帧率偏差
    inline QColor bad() { return QColor(0xF1, 0x4C, 0x4C); }  // 掉线/严重不同步
    inline QColor rec() { return QColor(0xF1, 0x4C, 0x4C); }  // 采集红点
    inline QColor marker() { return QColor(0x4F, 0xC1, 0xFF); }  // 画面上的检测十字丝：亮蓝，红外灰画面上最跳

    // —— 横幅底色（成功/警告/错误，深底版，配 *Fg 前景）——
    inline QColor okBannerBg() { return QColor(0x15, 0x33, 0x22); }
    inline QColor okBannerFg() { return QColor(0x89, 0xD1, 0x85); }
    inline QColor warnBannerBg() { return QColor(0x3A, 0x2E, 0x0F); }
    inline QColor warnBannerFg() { return QColor(0xD7, 0xBA, 0x7D); }
    inline QColor badBannerBg() { return QColor(0x3A, 0x1A, 0x1A); }
    inline QColor badBannerFg() { return QColor(0xF4, 0x8B, 0x8B); }

    // —— 兼容旧接口（老代码里 bg()/surface()/surfaceAlt()/line() 仍在用）——
    inline QColor bg() { return chrome(); }
    inline QColor surface() { return editorBg(); }
    inline QColor surfaceAlt() { return hoverBg(); }
    inline QColor line() { return lineSoft(); }

    inline QString hex(const QColor& c) { return c.name(QColor::HexRgb); }

    // 字体族。UI 用系统无衬线，读数用等宽（跟 VS Code 编辑器一个路子）。
    inline QString uiFamily() {
        return QStringLiteral("\"Segoe UI\", \"Microsoft YaHei UI\", \"PingFang SC\", "
                              "\"Noto Sans CJK SC\", sans-serif");
    }
    inline QString monoFamily() {
        return QStringLiteral("\"Cascadia Mono\", Consolas, \"JetBrains Mono\", "
                              "\"DejaVu Sans Mono\", monospace");
    }
    // 给代码里 setStyleSheet 拼等宽读数用，省得每处再写一遍字体回退链。
    inline QString monoCss(double px = 12.5) {
        return QStringLiteral("font-family: %1; font-size: %2px;")
            .arg(monoFamily()).arg(px);
    }

    // ======================= 活动栏图标 =======================
    // VS Code 的活动栏是线描图标。项目里没有图标资源文件，与其塞一堆
    // 未必有字形的 Unicode 符号（□ 豆腐块在客户机上很常见），不如直接用
    // QPainter 画 —— 都是几根线的事，零依赖、任何 DPI 下都清晰。
    enum class Glyph {
        Grid,        // 相机宫格
        Sliders,     // 检测参数
        Pulse,       // 同步监控
        Target,      // 标定
        Extensions,  // 工具
        Gear,        // 设置/存为默认
        Plus,
        Minus,
        Broadcast,   // UDP 推送
        Contrast,    // 算法灰度
        Play         // 启动实时动捕
    };

    inline QPixmap glyphPixmap(Glyph g, const QColor& c, int size = 24) {
        const qreal dpr = 2.0;
        QPixmap pm(int(size * dpr), int(size * dpr));
        pm.setDevicePixelRatio(dpr);
        pm.fill(Qt::transparent);

        QPainter p(&pm);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.scale(size * dpr / 24.0, size * dpr / 24.0);   // 统一按 24x24 画布作图

        QPen pen(c);
        pen.setWidthF(1.5);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);

        switch (g) {
        case Glyph::Grid:
            p.drawRect(QRectF(4, 4, 7, 7));
            p.drawRect(QRectF(13, 4, 7, 7));
            p.drawRect(QRectF(4, 13, 7, 7));
            p.drawRect(QRectF(13, 13, 7, 7));
            break;
        case Glyph::Sliders: {
            const double ys[3] = { 7, 12, 17 };
            const double knob[3] = { 15, 9, 17 };
            for (int i = 0; i < 3; ++i) {
                p.drawLine(QPointF(4, ys[i]), QPointF(20, ys[i]));
                p.setBrush(c);
                p.drawEllipse(QPointF(knob[i], ys[i]), 2.1, 2.1);
                p.setBrush(Qt::NoBrush);
            }
            break;
        }
        case Glyph::Pulse: {
            QPolygonF poly;
            poly << QPointF(3, 12) << QPointF(7.5, 12) << QPointF(9.5, 6)
                << QPointF(13, 18) << QPointF(15.5, 12) << QPointF(21, 12);
            p.drawPolyline(poly);
            break;
        }
        case Glyph::Target:
            p.drawEllipse(QPointF(12, 12), 6.2, 6.2);
            p.drawLine(QPointF(12, 2.5), QPointF(12, 7));
            p.drawLine(QPointF(12, 17), QPointF(12, 21.5));
            p.drawLine(QPointF(2.5, 12), QPointF(7, 12));
            p.drawLine(QPointF(17, 12), QPointF(21.5, 12));
            p.setBrush(c);
            p.drawEllipse(QPointF(12, 12), 1.6, 1.6);
            p.setBrush(Qt::NoBrush);
            break;
        case Glyph::Extensions:
            p.drawRect(QRectF(4, 4, 7, 7));
            p.drawRect(QRectF(4, 13, 7, 7));
            p.drawRect(QRectF(13, 13, 7, 7));
            p.drawLine(QPointF(13, 7.5), QPointF(20, 7.5));   // 缺角那块画成加号，跟 Grid 区分开
            p.drawLine(QPointF(16.5, 4), QPointF(16.5, 11));
            break;
        case Glyph::Gear: {
            p.drawEllipse(QPointF(12, 12), 5.4, 5.4);
            p.drawEllipse(QPointF(12, 12), 2.0, 2.0);
            const double kPi = 3.14159265358979323846;   // MSVC 默认不给 M_PI，本地定义
            for (int i = 0; i < 8; ++i) {
                const double a = i * kPi / 4.0;
                p.drawLine(QPointF(12 + std::cos(a) * 6.4, 12 + std::sin(a) * 6.4),
                           QPointF(12 + std::cos(a) * 8.6, 12 + std::sin(a) * 8.6));
            }
            break;
        }
        case Glyph::Plus:
            p.drawLine(QPointF(12, 5), QPointF(12, 19));
            p.drawLine(QPointF(5, 12), QPointF(19, 12));
            break;
        case Glyph::Minus:
            p.drawLine(QPointF(5, 12), QPointF(19, 12));
            break;
        case Glyph::Broadcast:
            p.setBrush(c);
            p.drawEllipse(QPointF(12, 12), 2.2, 2.2);
            p.setBrush(Qt::NoBrush);
            p.drawArc(QRectF(6.5, 6.5, 11, 11), 30 * 16, 120 * 16);
            p.drawArc(QRectF(6.5, 6.5, 11, 11), 210 * 16, 120 * 16);
            p.drawArc(QRectF(2.5, 2.5, 19, 19), 30 * 16, 120 * 16);
            p.drawArc(QRectF(2.5, 2.5, 19, 19), 210 * 16, 120 * 16);
            break;
        case Glyph::Play: {
            QPainterPath tri;
            tri.moveTo(7.5, 4.5); tri.lineTo(19.0, 12.0); tri.lineTo(7.5, 19.5);
            tri.closeSubpath();
            p.fillPath(tri, c);
            break;
        }
        case Glyph::Contrast: {
            p.drawEllipse(QPointF(12, 12), 7.5, 7.5);
            QPainterPath half;                       // 右半实心：明暗对半，一眼是「灰度」
            half.moveTo(12, 4.5);
            half.arcTo(QRectF(4.5, 4.5, 15, 15), 90, -180);
            half.closeSubpath();
            p.fillPath(half, c);
            break;
        }
        }
        return pm;
    }

    // 活动栏用：未选中偏暗、选中/悬停变亮，跟 VS Code 一致。
    inline QIcon activityIcon(Glyph g, int size = 24) {
        const QColor dim(0x86, 0x86, 0x86);
        const QColor lit(0xD7, 0xD7, 0xD7);
        QIcon ic;
        ic.addPixmap(glyphPixmap(g, dim, size), QIcon::Normal, QIcon::Off);
        ic.addPixmap(glyphPixmap(g, lit, size), QIcon::Active, QIcon::Off);
        ic.addPixmap(glyphPixmap(g, lit, size), QIcon::Normal, QIcon::On);
        ic.addPixmap(glyphPixmap(g, lit, size), QIcon::Active, QIcon::On);
        ic.addPixmap(glyphPixmap(g, QColor(0x50, 0x50, 0x50), size), QIcon::Disabled, QIcon::Off);
        return ic;
    }

    // 普通按钮/标签条用：默认正文色，选中转强调蓝。
    inline QIcon toolIcon(Glyph g, int size = 16) {
        QIcon ic;
        ic.addPixmap(glyphPixmap(g, text(), size), QIcon::Normal, QIcon::Off);
        ic.addPixmap(glyphPixmap(g, textBright(), size), QIcon::Active, QIcon::Off);
        ic.addPixmap(glyphPixmap(g, QColor(0x4F, 0xC1, 0xFF), size), QIcon::Normal, QIcon::On);
        ic.addPixmap(glyphPixmap(g, QColor(0x4F, 0xC1, 0xFF), size), QIcon::Active, QIcon::On);
        ic.addPixmap(glyphPixmap(g, textFaint(), size), QIcon::Disabled, QIcon::Off);
        return ic;
    }

    // ======================= 全局样式表 =======================
    // 用 @TOKEN@ 占位再整体替换，比 QString::arg 的 %1..%9 好维护得多
    // （超过 9 个颜色时 %1 会跟 %10 抢匹配，是个很隐蔽的坑）。
    inline QString styleSheet() {
        QString s = QStringLiteral(R"QSS(
/* ===================== 基础 ===================== */
QWidget {
    background-color: @EDITOR@;
    color: @TEXT@;
    font-family: @UIFONT@;
    font-size: 13px;
}
QMainWindow, QDialog { background-color: @EDITOR@; }
QWidget:disabled { color: @FAINT@; }

/* 纯文字类控件永远不自带底色，否则在深浅不同的容器里会印出方块 */
QLabel, QCheckBox, QRadioButton, QGroupBox, QSplitter, QScrollArea,
QToolBar, QStatusBar::item, QDockWidget { background: transparent; }

/* QFrame 的 HLine/VLine：直接拿背景当线画，并夹到 1px，避免默认 3px 的粗线 */
QFrame[frameShape="4"] { background: @LINESOFT@; border: none; max-height: 1px; }
QFrame[frameShape="5"] { background: @LINESOFT@; border: none; max-width: 1px; }

/* ===================== 菜单栏 / 菜单 ===================== */
QMenuBar { background-color: @CHROME@; color: @TEXT@; border: none;
           border-bottom: 1px solid @LINESOFT@; padding: 2px 4px; }
QMenuBar::item { background: transparent; padding: 5px 10px; border-radius: 3px; }
QMenuBar::item:selected { background-color: @HOVER@; color: @BRIGHT@; }
QMenuBar::item:pressed  { background-color: @ACCENT@; color: #ffffff; }

QMenu { background-color: @EDITOR@; color: @TEXT@; border: 1px solid @LINEHARD@;
        border-radius: 4px; padding: 4px; }
QMenu::item { padding: 6px 26px 6px 26px; border-radius: 3px; }
QMenu::item:selected { background-color: @ACCENT@; color: #ffffff; }
QMenu::item:disabled { color: @FAINT@; background: transparent; }
QMenu::separator { height: 1px; background: @LINEHARD@; margin: 4px 10px; }
QMenu::indicator { width: 14px; height: 14px; left: 7px; }
QMenu::indicator:checked { background-color: @ACCENT@; border-radius: 2px; }

/* ===================== 工具栏 ===================== */
QToolBar { border: none; spacing: 2px; padding: 3px 6px; }
QToolBar::separator { background: @LINEHARD@; width: 1px; margin: 5px 6px; }
QToolButton { background: transparent; border: 1px solid transparent; border-radius: 3px;
              padding: 4px 8px; color: @TEXT@; }
QToolButton:hover   { background-color: @HOVER@; color: @BRIGHT@; }
QToolButton:pressed { background-color: @LINEHARD@; }
QToolButton:checked { background-color: @SELECT@; color: @BRIGHT@; }
QToolButton:disabled { color: @FAINT@; }
QToolButton::menu-indicator { image: none; }

/* ===================== 按钮 ===================== */
/* VS Code 的次级按钮是灰底无描边，主按钮是纯蓝 —— 直角、紧凑 */
QPushButton { background-color: @INPUT@; border: 1px solid @LINEHARD@; border-radius: 2px;
              padding: 5px 14px; color: @TEXT@; min-height: 18px; }
QPushButton:hover    { background-color: #3C3C3C; color: @BRIGHT@; }
QPushButton:pressed  { background-color: #464646; }
QPushButton:disabled { background-color: #2A2A2A; color: @FAINT@; border-color: #333333; }
QPushButton:default, QPushButton[primary="true"] {
              background-color: @ACCENT@; color: #ffffff; border: 1px solid @ACCENT@; }
QPushButton:default:hover, QPushButton[primary="true"]:hover {
              background-color: @ACCENTHOVER@; border-color: @ACCENTHOVER@; }
QPushButton:focus { border: 1px solid @ACCENT@; }

/* ===================== 输入类 ===================== */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QPlainTextEdit, QTextEdit {
              background-color: @INPUT@; border: 1px solid @LINEHARD@; border-radius: 2px;
              padding: 4px 6px; color: @TEXT@;
              selection-background-color: @ACCENT@; selection-color: #ffffff; }
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus,
QPlainTextEdit:focus, QTextEdit:focus { border: 1px solid @ACCENT@; }
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {
              background-color: #2A2A2A; color: @FAINT@; }
QPlainTextEdit, QTextEdit { padding: 6px; }

QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
              background: transparent; border: none; width: 14px; }
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {
              width: 0; height: 0; border-left: 3px solid transparent;
              border-right: 3px solid transparent; border-bottom: 4px solid @DIM@; }
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {
              width: 0; height: 0; border-left: 3px solid transparent;
              border-right: 3px solid transparent; border-top: 4px solid @DIM@; }

QComboBox::drop-down { border: none; width: 18px; }
QComboBox::down-arrow { width: 0; height: 0; border-left: 4px solid transparent;
              border-right: 4px solid transparent; border-top: 5px solid @DIM@; margin-right: 5px; }
QComboBox QAbstractItemView { background-color: @EDITOR@; border: 1px solid @LINEHARD@;
              border-radius: 0px; padding: 2px; outline: none;
              selection-background-color: @ACCENT@; selection-color: #ffffff; }

/* ===================== 选项卡（编辑器标签页那种）===================== */
QTabWidget::pane { border: 1px solid @LINESOFT@; border-radius: 0px; top: -1px;
                   background-color: @EDITOR@; }
QTabBar { background: @CHROME@; }
QTabBar::tab { background-color: @CHROME@; color: @DIM@; border: none;
               border-right: 1px solid @LINESOFT@; border-top: 1px solid transparent;
               padding: 7px 16px; }
QTabBar::tab:hover:!selected { color: @TEXT@; }
QTabBar::tab:selected { background-color: @EDITOR@; color: @BRIGHT@;
                        border-top: 1px solid @ACCENT@; }

/* ===================== 勾选 / 单选 ===================== */
QCheckBox { spacing: 7px; }
QCheckBox::indicator { width: 15px; height: 15px; border: 1px solid @LINEHARD@;
                       border-radius: 2px; background-color: @INPUT@; }
QCheckBox::indicator:hover { border-color: @ACCENT@; }
QCheckBox::indicator:checked { background-color: @ACCENT@; border-color: @ACCENT@; }
QCheckBox::indicator:disabled { background-color: #2A2A2A; border-color: #3A3A3A; }
QRadioButton { spacing: 7px; }
QRadioButton::indicator { width: 15px; height: 15px; border: 1px solid @LINEHARD@;
                          border-radius: 8px; background-color: @INPUT@; }
QRadioButton::indicator:hover { border-color: @ACCENT@; }
QRadioButton::indicator:checked { background-color: @ACCENT@; border: 4px solid @INPUT@; }

/* ===================== 滑条 ===================== */
QSlider::groove:horizontal { height: 3px; background: @LINEHARD@; border-radius: 1px; }
QSlider::sub-page:horizontal { background: @ACCENT@; border-radius: 1px; }
QSlider::handle:horizontal { background: #CCCCCC; border: none; width: 11px; height: 11px;
                             margin: -4px 0; border-radius: 5px; }
QSlider::handle:horizontal:hover { background: #FFFFFF; }
QSlider::handle:horizontal:disabled { background: #4A4A4A; }
QSlider::sub-page:horizontal:disabled { background: @LINEHARD@; }

/* ===================== 列表 / 树 / 表 ===================== */
QListWidget, QListView, QTreeWidget, QTreeView {
        background-color: @EDITOR@; border: 1px solid @LINESOFT@; border-radius: 0px;
        padding: 2px; outline: none; alternate-background-color: @EDITOR@; }
QListWidget::item, QListView::item, QTreeWidget::item, QTreeView::item {
        padding: 5px 8px; border-radius: 2px; border: none; }
QListWidget::item:hover, QListView::item:hover,
QTreeWidget::item:hover, QTreeView::item:hover { background-color: @HOVER@; }
QListWidget::item:selected, QListView::item:selected,
QTreeWidget::item:selected, QTreeView::item:selected {
        background-color: @SELECT@; color: @BRIGHT@; }
QListWidget::item:disabled, QTreeWidget::item:disabled { color: @FAINT@; }

QTableWidget, QTableView {
        background-color: @EDITOR@; border: 1px solid @LINESOFT@; border-radius: 0px;
        gridline-color: @LINESOFT@; outline: none;
        selection-background-color: @SELECT@; selection-color: @BRIGHT@; }
QTableWidget::item, QTableView::item { padding: 4px 6px; border: none; }
QTableWidget::item:selected, QTableView::item:selected { background-color: @SELECT@; }

QHeaderView { background-color: @CHROME@; border: none; }
QHeaderView::section {
        background-color: @CHROME@; color: @DIM@; border: none;
        border-right: 1px solid @LINESOFT@; border-bottom: 1px solid @LINESOFT@;
        padding: 5px 6px; font-size: 12px; }
QHeaderView::section:hover { color: @TEXT@; }
QTableCornerButton::section { background-color: @CHROME@; border: none; }

/* ===================== 分组框 ===================== */
QGroupBox { border: 1px solid @LINESOFT@; border-radius: 2px; margin-top: 9px;
            padding-top: 10px; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 5px;
                   color: @DIM@; font-size: 12px; }

/* ===================== 进度条 ===================== */
QProgressBar { background-color: @INPUT@; border: none; border-radius: 0px;
               text-align: center; color: @TEXT@; }
QProgressBar::chunk { background-color: @ACCENT@; border-radius: 0px; }

/* ===================== 滚动条（VS Code 式：方的、悬浮、无箭头）=========== */
QScrollBar:vertical { background: transparent; width: 14px; margin: 0; border: none; }
QScrollBar::handle:vertical { background: rgba(121,121,121,0.4); min-height: 30px;
                              border-radius: 0px; }
QScrollBar::handle:vertical:hover   { background: rgba(158,158,158,0.7); }
QScrollBar::handle:vertical:pressed { background: rgba(191,191,191,0.8); }
QScrollBar:horizontal { background: transparent; height: 14px; margin: 0; border: none; }
QScrollBar::handle:horizontal { background: rgba(121,121,121,0.4); min-width: 30px;
                                border-radius: 0px; }
QScrollBar::handle:horizontal:hover   { background: rgba(158,158,158,0.7); }
QScrollBar::handle:horizontal:pressed { background: rgba(191,191,191,0.8); }
QScrollBar::add-line, QScrollBar::sub-line { width: 0; height: 0; background: none; border: none; }
QScrollBar::add-page, QScrollBar::sub-page { background: none; }

/* ===================== 状态栏 ===================== */
/* 底部整条蓝 —— VS Code 最好认的一处特征 */
QStatusBar { background-color: @ACCENT@; color: #ffffff; border: none; min-height: 22px; }
QStatusBar::item { border: none; }
QStatusBar QLabel { background: transparent; color: #ffffff; padding: 0 8px; font-size: 12px; }
QStatusBar QSizeGrip { background: transparent; width: 0; height: 0; }

/* ===================== 停靠面板 / 分割条 ===================== */
QDockWidget { color: @DIM@; font-size: 11px; }
QDockWidget::title { background: @CHROME@; padding: 6px 10px; text-align: left;
                     border-bottom: 1px solid @LINESOFT@; }
QSplitter::handle { background-color: @LINESOFT@; }
QSplitter::handle:horizontal { width: 1px; }
QSplitter::handle:vertical   { height: 1px; }
QSplitter::handle:hover { background-color: @ACCENT@; }

/* ===================== 向导（标定向导用的 QWizard）===================== */
QWizard { background-color: @EDITOR@; }
QWizard QWidget { background-color: @EDITOR@; }

/* ===================== 提示气泡 ===================== */
QToolTip { background-color: @EDITOR@; color: @TEXT@; border: 1px solid @LINEHARD@;
           border-radius: 3px; padding: 5px 7px; }

/* ===================== 应用外壳（MainWindow 里用 objectName 挂上）========= */
#activityBar { background-color: @CHROME@; border-right: 1px solid @LINESOFT@; }
#activityBar QToolButton {
        background: transparent; border: none; border-left: 2px solid transparent;
        border-radius: 0px; padding: 0px; }
#activityBar QToolButton:hover   { background-color: rgba(255,255,255,0.06); }
#activityBar QToolButton:checked { border-left: 2px solid #D7D7D7; background: transparent; }

#sideBar { background-color: @CHROME@; }
#sideBarHeader { background-color: @CHROME@; }
#sideBarTitle  { color: @DIM@; font-size: 11px; padding: 8px 4px 8px 12px;
                 letter-spacing: 1px; }
#sidePane { background-color: @CHROME@; }
#sidePane QListWidget, #sidePane QTableWidget { background-color: @CHROME@; border: none; }
#sidePane QHeaderView::section { background-color: @CHROME@; }
/* 侧边栏里的按钮做成「一行一个、左对齐」的列表条目样式，跟资源管理器一致 */
#sidePane QPushButton {
        background: transparent; border: none; border-radius: 2px;
        padding: 6px 10px; text-align: left; color: @TEXT@; }
#sidePane QPushButton:hover   { background-color: @HOVER@; color: @BRIGHT@; }
#sidePane QPushButton:pressed { background-color: @SELECT@; }
#sideGroupLabel { color: @DIM@; font-size: 11px; padding: 10px 12px 4px 12px;
                  letter-spacing: 1px; }

#editorTabBar { background-color: @CHROME@; border-bottom: 1px solid @LINESOFT@; }
#editorTab { background-color: @EDITOR@; color: @BRIGHT@; padding: 7px 16px;
             border-top: 1px solid @ACCENT@; border-right: 1px solid @LINESOFT@; }
#editorTabBar QToolButton { padding: 4px 7px; }
#editorTabBar QLabel  { color: @DIM@; font-size: 12px; padding-left: 8px; }
#editorTabBar QComboBox { background-color: @INPUT@; padding: 3px 6px; min-height: 16px; }

#editorArea { background-color: @EDITOR@; }

/* 标签条右端的主行动按钮（▶ 实时动捕）。整条工具栏只有它是实心强调色 ——
   一个界面里只该有一个"最该点的按钮"，多了就等于没有重点。 */
#runButton { background-color: @ACCENT@; color: #ffffff; border: none;
             border-radius: 2px; padding: 5px 14px; font-weight: 600; }
#runButton:hover   { background-color: @ACCENTHOVER@; }
#runButton:pressed { background-color: #005A9E; }

/* ===== 面板通用外壳：标签条 / 侧栏 / 可折叠小节 / 底部读数条 =====
   实时动捕、三角化调试、标定杆验证等面板共用这套 objectName，
   加新面板时挂同样的名字就自动获得同一套外观，不用各写各的内联样式 */
#panelTabBar { background-color: @CHROME@; border-bottom: 1px solid @LINESOFT@; }
#panelTab    { background-color: @EDITOR@; color: @BRIGHT@; padding: 7px 16px;
            border-top: 1px solid @ACCENT@; border-right: 1px solid @LINESOFT@; }
#panelSideBar { background-color: @CHROME@; }

/* 可折叠小节的标题条：整行可点，悬停变亮，直角、无边框 —— 跟 VS Code
   设置页/资源管理器的分组标题是同一种控件语言 */
#panelSectionHead {
        background-color: @CHROME@; color: @DIM@; border: none; border-radius: 0px;
        padding: 7px 10px 7px 6px; text-align: left; font-size: 11px; letter-spacing: 1px;
        border-top: 1px solid @LINESOFT@; }
#panelSectionHead:hover   { background-color: @HOVER@; color: @BRIGHT@; }
#panelSectionHead:checked { color: @TEXT@; }
#panelSectionBody { background-color: @CHROME@; }
#panelSectionBody QLabel { color: @TEXT@; }

/* 底部读数条。蓝底会跟主窗口状态栏撞车（那是应用级状态，这是面板级），
   所以用深底 + 顶部一条强调色细线区分层级 */
/* 侧栏顶部的常用操作栏：钉住不折叠，用底部一条线跟下面的小节列表分开 */
#panelQuickBar { background-color: @CHROME@; border-bottom: 1px solid @LINEHARD@; }
#panelQuickBar QPushButton { background-color: @INPUT@; border: 1px solid @LINEHARD@;
                             padding: 5px 12px; }
#panelQuickBar QPushButton:hover { background-color: #3C3C3C; color: @BRIGHT@; }

#panelStatusBar { background-color: @CHROME@; border-top: 1px solid @ACCENT@; }
#panelStatusBar QLabel { color: @TEXT@; font-family: @MONOFONT@; font-size: 11px; }
#panelStatusBar QPushButton { padding: 2px 10px; font-size: 11px; }
#statusMono { font-family: @MONOFONT@; font-size: 11px; }

/* ===================== 相机宫格空态 ===================== */
/* 一台相机都没有（开机首次 / 全被移除）时，宫格区域是整片纯黑。
   给一行低调的等待小字：主行说明状态、次行指下一步点哪，跟 VS Code
   空编辑器的提示一个调子 —— 能看见但不抢眼。 */
#gridEmptyHint { background: transparent; }
#gridHintTitle { color: @DIM@;   font-size: 15px; }
#gridHintSub   { color: @FAINT@; font-size: 12px; }
)QSS");

        struct Tok { const char* key; QString val; };
        const Tok toks[] = {
            { "@CHROME@",      hex(chrome())      },
            { "@EDITOR@",      hex(editorBg())    },
            { "@INPUT@",       hex(inputBg())     },
            { "@HOVER@",       hex(hoverBg())     },
            { "@SELECT@",      hex(selectBg())    },
            { "@LINESOFT@",    hex(lineSoft())    },
            { "@LINEHARD@",    hex(lineHard())    },
            { "@TEXT@",        hex(text())        },
            { "@BRIGHT@",      hex(textBright())  },
            { "@DIM@",         hex(textDim())     },
            { "@FAINT@",       hex(textFaint())   },
            { "@ACCENTHOVER@", hex(accentHover()) },   // 必须排在 @ACCENT@ 前面
            { "@ACCENT@",      hex(accent())      },
            { "@UIFONT@",      uiFamily()         },
            { "@MONOFONT@",    monoFamily()       },
        };
        for (const Tok& t : toks) s.replace(QLatin1String(t.key), t.val);
        return s;
    }

    // 可选：在 main.cpp 里补一句 theme::applyPalette(app)，让 QMessageBox、
    // 原生下拉、以及少数不走样式表的地方也跟着变深色（样式表管不到调色板）。
    // 不调用也能跑，只是那几处会露出浅色。
    template <class TApp>
    inline void applyPalette(TApp& app) {
        QPalette pal;
        pal.setColor(QPalette::Window, editorBg());
        pal.setColor(QPalette::WindowText, text());
        pal.setColor(QPalette::Base, inputBg());
        pal.setColor(QPalette::AlternateBase, chrome());
        pal.setColor(QPalette::ToolTipBase, editorBg());
        pal.setColor(QPalette::ToolTipText, text());
        pal.setColor(QPalette::Text, text());
        pal.setColor(QPalette::Button, inputBg());
        pal.setColor(QPalette::ButtonText, text());
        pal.setColor(QPalette::BrightText, textBright());
        pal.setColor(QPalette::Link, QColor(0x4F, 0xC1, 0xFF));
        pal.setColor(QPalette::Highlight, accent());
        pal.setColor(QPalette::HighlightedText, Qt::white);
        pal.setColor(QPalette::Mid, textDim());          // CalibWizard 里用了 palette(mid)
        pal.setColor(QPalette::Dark, chrome());
        pal.setColor(QPalette::Light, lineHard());
        pal.setColor(QPalette::Disabled, QPalette::WindowText, textFaint());
        pal.setColor(QPalette::Disabled, QPalette::Text, textFaint());
        pal.setColor(QPalette::Disabled, QPalette::ButtonText, textFaint());
        app.setPalette(pal);
    }

} // namespace mocap::theme
