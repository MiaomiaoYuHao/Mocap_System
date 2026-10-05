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
    // 图标取自 Lucide（https://lucide.dev，ISC 许可）—— 24x24 线框图标库，
    // 1695 个图标、笔画风格统一、每个图标的语义由官方定义，不是我们自己
    // 画一个"看起来差不多"的形状。
    //
    // 【为什么把路径编译进代码，而不是装图标字体 / 运行时解析 SVG】
    //   · 图标字体要额外发一个 ttf，还要维护"码点 -> 图标"的映射；
    //   · 运行时解析 SVG 要给 Qt 加 SVG 模块依赖，还多一层解析失败的可能。
    // 下面 glyphPixmap() 里的 QPainterPath 是用 video/tools/genicons.py 从
    // Lucide 原始 SVG 路径数据【离线编译】出来的（圆弧已转成三次贝塞尔），
    // 结果就是普通 C++ 常量：零依赖、零运行时开销、任意 DPI 都清晰。
    //
    // 【要换图标或加图标】改 genicons.py 里的 MAP 跑一遍，把生成的 case 块
    // 贴回下面的 switch；枚举名与 Lucide 文件名一一对应。
    enum class Glyph {
        Camera,      // 相机      lucide/camera
        Sliders,     // 检测参数  lucide/sliders-horizontal
        Gauge,       // 同步监控  lucide/gauge
        Focus,       // 标定      lucide/focus
        Wrench,      // 工具      lucide/wrench
        Settings,    // 设置      lucide/settings
        Plus,        // 添加      lucide/plus
        Minus,       // 移除      lucide/minus
        RadioTower,  // UDP 推送  lucide/radio-tower
        Contrast,    // 算法灰度  lucide/contrast
        Play         // 实时动捕  lucide/play
    };

    inline QPixmap glyphPixmap(Glyph g, const QColor& c, int size = 24) {
        const qreal dpr = 2.0;
        QPixmap pm(int(size * dpr), int(size * dpr));
        pm.setDevicePixelRatio(dpr);
        pm.fill(Qt::transparent);

        QPainter p(&pm);
        p.setRenderHint(QPainter::Antialiasing, true);

        // 【这里以前有一句 p.scale(size*dpr/24.0, ...)，是个真 bug，已删】
        // QPainter 在 devicePixelRatio != 1 的 QPixmap 上作画时，Qt 已经自动
        // 把逻辑坐标按 DPR 放大过一次了。再手写一次 scale 等于放大 2×2=4 倍，
        // 24x24 的图形只有左上角四分之一落在 48x48 的位图里 —— 表现就是
        // 图标被裁掉大半（2x2 宫格只剩一个半方块、相机只剩半个机身），
        // 看着像"图标画得不对"，其实是根本没画全。
        // 现在只依赖 Qt 的 DPR 变换，24x24 图形正好铺满，任意 DPI 都清晰。

        QPen pen(c);
        pen.setWidthF(1.5);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        p.setPen(pen);
        p.setBrush(Qt::NoBrush);

        switch (g) {
        case Glyph::Camera: {
            static const QPainterPath kCamera = [] {
                QPainterPath q;
                q.moveTo(13.997, 4);
                q.cubicTo(14.732, 4, 15.4079, 4.4032, 15.757, 5.05);
                q.lineTo(16.243, 5.95);
                q.cubicTo(16.5921, 6.5968, 17.268, 7, 18.003, 7);
                q.lineTo(20, 7);
                q.cubicTo(21.1046, 7, 22, 7.8954, 22, 9);
                q.lineTo(22, 18);
                q.cubicTo(22, 19.1046, 21.1046, 20, 20, 20);
                q.lineTo(4, 20);
                q.cubicTo(2.8954, 20, 2, 19.1046, 2, 18);
                q.lineTo(2, 9);
                q.cubicTo(2, 7.8954, 2.8954, 7, 4, 7);
                q.lineTo(5.997, 7);
                q.cubicTo(6.7313, 7, 7.4065, 6.5977, 7.756, 5.952);
                q.lineTo(8.245, 5.048);
                q.cubicTo(8.5945, 4.4023, 9.2697, 4, 10.004, 4);
                q.closeSubpath();
                q.addEllipse(QPointF(12, 13), 3, 3);
                return q;
            }();
            p.drawPath(kCamera);
            break;
        }
        case Glyph::Sliders: {
            static const QPainterPath kSliders = [] {
                QPainterPath q;
                q.moveTo(10, 5);
                q.lineTo(3, 5);
                q.moveTo(12, 19);
                q.lineTo(3, 19);
                q.moveTo(14, 3);
                q.lineTo(14, 7);
                q.moveTo(16, 17);
                q.lineTo(16, 21);
                q.moveTo(21, 12);
                q.lineTo(12, 12);
                q.moveTo(21, 19);
                q.lineTo(16, 19);
                q.moveTo(21, 5);
                q.lineTo(14, 5);
                q.moveTo(8, 10);
                q.lineTo(8, 14);
                q.moveTo(8, 12);
                q.lineTo(3, 12);
                return q;
            }();
            p.drawPath(kSliders);
            break;
        }
        case Glyph::Gauge: {
            static const QPainterPath kGauge = [] {
                QPainterPath q;
                q.moveTo(12, 14);
                q.lineTo(16, 10);
                q.moveTo(3.34, 19);
                q.cubicTo(0.9133, 14.7972, 1.8544, 9.4588, 5.572, 6.3392);
                q.cubicTo(9.2896, 3.2197, 14.7104, 3.2197, 18.428, 6.3392);
                q.cubicTo(22.1456, 9.4588, 23.0867, 14.7972, 20.66, 19);
                return q;
            }();
            p.drawPath(kGauge);
            break;
        }
        case Glyph::Focus: {
            static const QPainterPath kFocus = [] {
                QPainterPath q;
                q.addEllipse(QPointF(12, 12), 3, 3);
                q.moveTo(3, 7);
                q.lineTo(3, 5);
                q.cubicTo(3, 3.8954, 3.8954, 3, 5, 3);
                q.lineTo(7, 3);
                q.moveTo(17, 3);
                q.lineTo(19, 3);
                q.cubicTo(20.1046, 3, 21, 3.8954, 21, 5);
                q.lineTo(21, 7);
                q.moveTo(21, 17);
                q.lineTo(21, 19);
                q.cubicTo(21, 20.1046, 20.1046, 21, 19, 21);
                q.lineTo(17, 21);
                q.moveTo(7, 21);
                q.lineTo(5, 21);
                q.cubicTo(3.8954, 21, 3, 20.1046, 3, 19);
                q.lineTo(3, 17);
                return q;
            }();
            p.drawPath(kFocus);
            break;
        }
        case Glyph::Wrench: {
            static const QPainterPath kWrench = [] {
                QPainterPath q;
                q.moveTo(14.7, 6.3);
                q.cubicTo(14.3189, 6.6888, 14.3189, 7.3112, 14.7, 7.7);
                q.lineTo(16.3, 9.3);
                q.cubicTo(16.6888, 9.6811, 17.3112, 9.6811, 17.7, 9.3);
                q.lineTo(20.806, 6.195);
                q.cubicTo(21.126, 5.873, 21.669, 5.975, 21.789, 6.413);
                q.cubicTo(22.4058, 8.6563, 21.6702, 11.0539, 19.9014, 12.5652);
                q.cubicTo(18.1326, 14.0766, 15.6496, 14.4292, 13.53, 13.47);
                q.lineTo(5.62, 21.38);
                q.cubicTo(4.7916, 22.2082, 3.4487, 22.2079, 2.6205, 21.3795);
                q.cubicTo(1.7923, 20.5511, 1.7926, 19.2082, 2.621, 18.38);
                q.lineTo(10.531, 10.47);
                q.cubicTo(9.5718, 8.3504, 9.9244, 5.8674, 11.4358, 4.0986);
                q.cubicTo(12.9471, 2.3298, 15.3447, 1.5942, 17.588, 2.211);
                q.cubicTo(18.026, 2.331, 18.128, 2.873, 17.807, 3.195);
                q.closeSubpath();
                return q;
            }();
            p.drawPath(kWrench);
            break;
        }
        case Glyph::Settings: {
            static const QPainterPath kSettings = [] {
                QPainterPath q;
                q.moveTo(9.671, 4.136);
                q.cubicTo(9.7852, 2.9348, 10.7939, 2.0174, 12.0005, 2.0174);
                q.cubicTo(13.2071, 2.0174, 14.2158, 2.9348, 14.33, 4.136);
                q.cubicTo(14.3972, 4.8959, 14.8307, 5.5754, 15.4915, 5.9567);
                q.cubicTo(16.1523, 6.3379, 16.9574, 6.3731, 17.649, 6.051);
                q.cubicTo(18.7452, 5.5533, 20.0402, 5.9686, 20.6425, 7.0111);
                q.cubicTo(21.2448, 8.0536, 20.9577, 9.3829, 19.979, 10.084);
                q.cubicTo(19.3547, 10.522, 18.983, 11.2369, 18.983, 11.9995);
                q.cubicTo(18.983, 12.7621, 19.3547, 13.477, 19.979, 13.915);
                q.cubicTo(20.9577, 14.6161, 21.2448, 15.9454, 20.6425, 16.9879);
                q.cubicTo(20.0402, 18.0304, 18.7452, 18.4457, 17.649, 17.948);
                q.cubicTo(16.9574, 17.6259, 16.1523, 17.6611, 15.4915, 18.0423);
                q.cubicTo(14.8307, 18.4236, 14.3972, 19.1031, 14.33, 19.863);
                q.cubicTo(14.2158, 21.0642, 13.2071, 21.9816, 12.0005, 21.9816);
                q.cubicTo(10.7939, 21.9816, 9.7852, 21.0642, 9.671, 19.863);
                q.cubicTo(9.6039, 19.1028, 9.1703, 18.423, 8.5092, 18.0417);
                q.cubicTo(7.8481, 17.6604, 7.0427, 17.6254, 6.351, 17.948);
                q.cubicTo(5.2548, 18.4457, 3.9598, 18.0304, 3.3575, 16.9879);
                q.cubicTo(2.7552, 15.9454, 3.0423, 14.6161, 4.021, 13.915);
                q.cubicTo(4.6453, 13.477, 5.017, 12.7621, 5.017, 11.9995);
                q.cubicTo(5.017, 11.2369, 4.6453, 10.522, 4.021, 10.084);
                q.cubicTo(3.0437, 9.3826, 2.7574, 8.0544, 3.359, 7.0127);
                q.cubicTo(3.9606, 5.971, 5.254, 5.5551, 6.35, 6.051);
                q.cubicTo(7.0416, 6.3731, 7.8467, 6.3379, 8.5075, 5.9567);
                q.cubicTo(9.1683, 5.5754, 9.6018, 4.8959, 9.669, 4.136);
                q.addEllipse(QPointF(12, 12), 3, 3);
                return q;
            }();
            p.drawPath(kSettings);
            break;
        }
        case Glyph::Plus: {
            static const QPainterPath kPlus = [] {
                QPainterPath q;
                q.moveTo(5, 12);
                q.lineTo(19, 12);
                q.moveTo(12, 5);
                q.lineTo(12, 19);
                return q;
            }();
            p.drawPath(kPlus);
            break;
        }
        case Glyph::Minus: {
            static const QPainterPath kMinus = [] {
                QPainterPath q;
                q.moveTo(5, 12);
                q.lineTo(19, 12);
                return q;
            }();
            p.drawPath(kMinus);
            break;
        }
        case Glyph::RadioTower: {
            static const QPainterPath kRadioTower = [] {
                QPainterPath q;
                q.moveTo(4.9, 16.1);
                q.cubicTo(1, 12.2, 1, 5.8, 4.9, 1.9);
                q.moveTo(7.8, 4.7);
                q.cubicTo(5.8483, 6.7223, 5.5187, 9.8115, 7, 12.2);
                q.addEllipse(QPointF(12, 9), 2, 2);
                q.moveTo(16.2, 4.8);
                q.cubicTo(18.2, 6.8, 18.46, 9.91, 17, 12.27);
                q.moveTo(19.1, 1.9);
                q.cubicTo(20.9723, 3.7684, 22.0244, 6.3049, 22.0244, 8.95);
                q.cubicTo(22.0244, 11.5951, 20.9723, 14.1316, 19.1, 16);
                q.moveTo(9.5, 18);
                q.lineTo(14.5, 18);
                q.moveTo(8, 22);
                q.lineTo(12, 11);
                q.lineTo(16, 22);
                return q;
            }();
            p.drawPath(kRadioTower);
            break;
        }
        case Glyph::Contrast: {
            static const QPainterPath kContrast = [] {
                QPainterPath q;
                q.addEllipse(QPointF(12, 12), 10, 10);
                q.moveTo(12, 18);
                q.cubicTo(15.3137, 18, 18, 15.3137, 18, 12);
                q.cubicTo(18, 8.6863, 15.3137, 6, 12, 6);
                q.lineTo(12, 18);
                q.closeSubpath();
                return q;
            }();
            p.drawPath(kContrast);
            break;
        }
        case Glyph::Play: {
            static const QPainterPath kPlay = [] {
                QPainterPath q;
                q.moveTo(5, 5);
                q.cubicTo(4.9998, 4.2837, 5.3827, 3.622, 6.0038, 3.2652);
                q.cubicTo(6.6248, 2.9084, 7.3893, 2.911, 8.008, 3.272);
                q.lineTo(20.005, 10.27);
                q.cubicTo(20.6211, 10.6275, 21.0006, 11.2858, 21.0012, 11.9981);
                q.cubicTo(21.0019, 12.7105, 20.6235, 13.3694, 20.008, 13.728);
                q.lineTo(8.008, 20.728);
                q.cubicTo(7.3893, 21.089, 6.6248, 21.0916, 6.0038, 20.7348);
                q.cubicTo(5.3827, 20.378, 4.9998, 19.7163, 5, 19);
                q.closeSubpath();
                return q;
            }();
            p.drawPath(kPlay);
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
