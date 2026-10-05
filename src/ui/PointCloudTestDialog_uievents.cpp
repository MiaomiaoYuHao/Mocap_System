// ===========================================================================
// PointCloudTestDialog_uievents.cpp —— 面板控件操作的全量事件记录。
//
// 【这个文件解决的是一个具体的排查困境】
// 面板上有 87 个控件。而文件里能反推出控件值的只有两处：
//   · 逐帧 RunFlags —— 20 个布尔位，只覆盖开关，不覆盖任何数值
//   · 1Hz runtimeConfig —— 26 个字段
// 也就是说【剩下 61 个控件的值，文件里一个字节都没有】。
//
// 更要命的是采样率：runtimeConfig 是 1 秒一份。一次"调大看看→不对→调回去"
// 的操作全程不到一秒，在文件里【完全不存在】—— 而这种试探性调整恰恰是
// 排查现场最常发生的事，也最容易在事后被本人忘掉。
//
// 于是有一类问题永远查不下去："录之前/录之中，是不是有人动过什么？"
// 这个问题不需要精妙的分析方法，只需要一份如实的操作流水。
//
// 【为什么用 findChildren 自动接线，而不是在 87 个控件上各写一句】
// 手写 87 句的真正代价不是打字，是【漏】：新加一个控件时没人会记得
// 补那一句，而漏掉的那一个按墨菲定律就是出问题的那一个。
// 自动接线的覆盖率天然是 100%，且新控件自动纳入 —— 这不是省事，
// 是把"完整性"从人的自觉变成机器的性质。
//
// 【控件名怎么来】优先 objectName()；按钮类用 text()（那正是用户点的
// 那几个字）；数值框/下拉框回退到"同一布局里前面最近的 QLabel"——
// 这个面板全部是"标签 + 控件"横排，命中率很高。都失败时退到
// "类型#序号"，并且【如实记进快照】而不是假装有名字：
// 一个叫 "QDoubleSpinBox#7" 的控件至少诚实，而一个猜错的名字会误导。
// ===========================================================================
#include "ui/PointCloudTestDialog.hpp"

#include <QAbstractButton>
#include <QAbstractSlider>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>

namespace mocap {

namespace {

// JSON 字符串转义。控件名里有中文、引号、反斜杠都可能。
std::string jstr(const QString& s) {
    std::string o = "\"";
    for (const QChar& c : s) {
        const ushort u = c.unicode();
        if (u == '"')       o += "\\\"";
        else if (u == '\\') o += "\\\\";
        else if (u == '\n') o += "\\n";
        else if (u == '\r') o += "\\r";
        else if (u == '\t') o += "\\t";
        else if (u < 0x20)  o += ' ';
        else                o += QString(c).toUtf8().constData();
    }
    o += "\"";
    return o;
}

// 找到【直接包含】该控件的那个布局。
// 【为什么必须递归找，不能用 parentWidget()->layout()】
// 面板全是「标签 + 控件」的横排行，而那个 QHBoxLayout 通常是嵌在
// 外层 QVBoxLayout 里的【子布局】，本身没有自己的 QWidget。
// 控件的 parentWidget() 是外层那个容器，parent->layout() 拿到的是
// 外层 QVBoxLayout —— 控件根本不是它的直接项，于是一个都找不到。
// 实测就是这么翻的车：所有数值框全退化成了 "QDoubleSpinBox#2" 这种编号名。
QLayout* owningLayout(QLayout* lay, QWidget* w) {
    if (!lay) return nullptr;
    for (int i = 0; i < lay->count(); ++i) {
        QLayoutItem* it = lay->itemAt(i);
        if (!it) continue;
        if (it->widget() == w) return lay;
        if (QLayout* sub = it->layout())
            if (QLayout* r = owningLayout(sub, w)) return r;
    }
    return nullptr;
}

// 在控件所在的那一行里，往前找最近的一个 QLabel 作为它的名字。
// 【为什么找"前面"的】这个面板全部是横排的「标签: 控件」，标签在左。
QString labelBefore(QWidget* w) {
    QWidget* parent = w->parentWidget();
    if (!parent) return QString();
    QLayout* lay = owningLayout(parent->layout(), w);
    if (!lay) return QString();
    int idx = -1;
    for (int i = 0; i < lay->count(); ++i)
        if (lay->itemAt(i) && lay->itemAt(i)->widget() == w) { idx = i; break; }
    if (idx < 0) return QString();
    for (int i = idx - 1; i >= 0; --i) {
        QLayoutItem* it = lay->itemAt(i);
        if (!it || !it->widget()) continue;
        if (auto* lb = qobject_cast<QLabel*>(it->widget())) {
            QString t = lb->text().trimmed();
            // 去掉尾部的冒号，跟 objectName 的风格统一
            while (t.endsWith(QLatin1Char(':')) || t.endsWith(QStringLiteral("：")))
                t.chop(1);
            if (!t.isEmpty()) return t;
        }
        // 【碰到另一个控件就停】再往前找就串到上一行去了，
        // 而一个张冠李戴的名字比没有名字危险得多。
        if (qobject_cast<QAbstractButton*>(it->widget()) ||
            qobject_cast<QAbstractSpinBox*>(it->widget()) ||
            qobject_cast<QComboBox*>(it->widget()) ||
            qobject_cast<QAbstractSlider*>(it->widget()))
            break;
    }
    return QString();
}

// Qt 内部件。QSpinBox 里嵌着一个 QLineEdit、QComboBox 里嵌着 QLineEdit
// 和一个下拉视图 —— 它们都是 findChildren 的合法结果，但【不是用户能操作
// 的控件】。不过滤的后果实测有两个，第二个尤其坏：
//   · 快照里混进一堆 "qt_spinbox_lineedit"，值跟宿主 spinbox 重复
//   · 它们参与序号计数，于是【快照里的序号和事件里的序号对不上】——
//     同一个控件在两处叫两个名字，基线和增量就再也 join 不起来了
bool isQtInternal(QWidget* w) {
    if (w->objectName().startsWith(QLatin1String("qt_"))) return true;
    for (QWidget* p = w->parentWidget(); p; p = p->parentWidget()) {
        if (qobject_cast<QAbstractSpinBox*>(p) || qobject_cast<QComboBox*>(p))
            return true;
    }
    return false;
}

// 是不是我们要记录的控件类型。
bool isTrackedControl(QWidget* w) {
    return qobject_cast<QCheckBox*>(w)       || qobject_cast<QRadioButton*>(w) ||
           qobject_cast<QComboBox*>(w)       || qobject_cast<QDoubleSpinBox*>(w) ||
           qobject_cast<QSpinBox*>(w)        || qobject_cast<QAbstractSlider*>(w) ||
           qobject_cast<QPushButton*>(w)     || qobject_cast<QLineEdit*>(w);
}

} // namespace

// ---------------------------------------------------------------------------
// 【唯一的控件枚举口】快照和事件接线都必须走这里。
//
// 分成两份各自 findChildren + 各自计数，实测立刻就错位了：快照把
// spinbox 内部的 QLineEdit 也算进序号，于是同一个控件在快照里叫
// "QSpinBox#4"、在事件里叫 "QSpinBox#3"。两处名字对不上，
// 基线和增量就【再也 join 不起来】—— 而那正是这两份数据唯一的用法。
// 过滤规则和序号只允许有一份实现。
// ---------------------------------------------------------------------------
QList<QPair<QWidget*, QString>> PointCloudTestDialog::uiControls() const {
    QList<QPair<QWidget*, QString>> out;
    int ord = 0;
    for (QWidget* w : findChildren<QWidget*>()) {
        if (!isTrackedControl(w) || isQtInternal(w)) continue;
        ++ord;
        out.append(qMakePair(w, uiControlName(w, ord)));
    }
    return out;
}

// ---------------------------------------------------------------------------
// 控件名。优先级：objectName > 按钮文字 > 前面的标签 > 类型#序号
// ---------------------------------------------------------------------------
QString PointCloudTestDialog::uiControlName(QWidget* w, int ordinal) const {
    if (!w) return QStringLiteral("?");
    if (!w->objectName().isEmpty()) return w->objectName();
    if (auto* b = qobject_cast<QAbstractButton*>(w)) {
        QString t = b->text().trimmed();
        t.remove(QLatin1Char('&'));            // 助记符
        if (!t.isEmpty()) return t;
    }
    const QString lb = labelBefore(w);
    if (!lb.isEmpty()) return lb;
    // 【退到类型#序号，而不是猜】猜错的名字会把人引到错误的控件上，
    // 而那种误导只会在你正拿它排查问题的时候发生。
    return QStringLiteral("%1#%2").arg(QString::fromLatin1(w->metaObject()->className()))
                                  .arg(ordinal);
}

// ---------------------------------------------------------------------------
// 写一条控件操作事件（code 20）。
// valueA=新值 valueB=旧值，都转成 double；下拉框用索引，按钮用 1。
// ---------------------------------------------------------------------------
void PointCloudTestDialog::recordUiEvent(const QString& name, const QString& kind,
                                         double newVal, double oldVal,
                                         const QString& newText, qint64 wallMsOverride)
{
    if (!recorder_.recording()) return;
    // ---- 【实测踩到的坑】时间戳必须能被调用方指定，不能永远取"现在" ----
    // 起因：romBtn_ 的 clicked() 上先接了 app 自己的处理器（结束标定时会弹一个
    // QMessageBox 展示结果），我们的录制器是后接的。Qt 对同一信号的多个
    // direct connection 按注册顺序【同步依次执行】——先接的那个如果内部
    // 调用了 box.exec()，会进入一个嵌套事件循环，直到用户把弹窗点掉才返回；
    // 后接的槽（也就是我们）要等它返回了才会被调用。
    //
    // 实测复现：两个槽接在同一个 clicked() 上，中间插一个 1.6 秒的模态框，
    //   [app 处理器]   t=...353
    //   [录制器处理器] t=...981   <- 晚了整整 1628ms，而这是【同一次点击】
    //
    // 后果：用户点一次"结束ROM"，事件 17（app 自己记的，弹窗前就落盘）和
    // 我们的事件 20（弹窗关掉之后才落盘）时间戳能差出一秒多，在 timeline()
    // 里看起来像是【两次点击】。这正是"我明明只点了一下，为什么看着像点了
    // 两次"的直接成因 —— 不是用户记错了，是我们自己的时间戳量错了地方。
    //
    // 修法：调用方在 pressed()（鼠标按下的那一刻，早于任何 clicked() 槽、
    // 不会被后面弹出的模态框拖慢）抢先记一次时间戳，传进来这里用；
    // 不传（-1）就退回"现在"，适用于没有这个风险的控件。
    const qint64 wallMs = (wallMsOverride > 0) ? wallMsOverride
                                               : QDateTime::currentMSecsSinceEpoch();
    const qint64 wall = wallMs * 1000000LL;
    QString txt = name + QStringLiteral(" [") + kind + QStringLiteral("] ");
    if (!newText.isEmpty()) txt += newText;
    else                    txt += QString::number(oldVal, 'g', 6)
                                 + QStringLiteral(" -> ")
                                 + QString::number(newVal, 'g', 6);
    // 【frameTsNs 传 -1 是刻意的】控件回调跑在 GUI 线程上，此刻的"当前帧"
    // 是上一帧的残留，填进去等于伪造一个精确到帧的时刻。
    // 墙钟是这条事件唯一诚实的时间轴；要跟帧对齐，靠前后两帧的墙钟去夹。
    recorder_.writeEvent(20, 0, wall, -1, txt.toStdString(), newVal, oldVal, 0, 0);
}

// ---------------------------------------------------------------------------
// 把面板上【每一个】控件的当前值打成 JSON。
//
// 【为什么光有增量事件不够，还必须有这份快照】
// 事件记的是"从旧值变成新值"。而一个【全程没被碰过】的控件不会产生任何
// 事件 —— 它的值在文件里就是空白。偏偏"没被碰过的那些"才是系统运行的
// 大背景：出问题时要先确认"其它东西都在默认位置"，才轮得到看变了的那几个。
//
// 开始和停止各写一份：开始那份是所有增量的基线，停止那份让你不必重放
// 一遍增量就能知道最终配置 —— 而重放本身是个会出错的步骤。
// ---------------------------------------------------------------------------
std::string PointCloudTestDialog::uiSnapshotJson() const {
    std::string j = "{\"controls\":[";
    bool first = true;
    for (const auto& pr : uiControls()) {
        QWidget* w = pr.first;
        QString kind, val, extra;
        if (auto* cb = qobject_cast<QCheckBox*>(w)) {
            kind = QStringLiteral("check");
            val = cb->isChecked() ? QStringLiteral("1") : QStringLiteral("0");
        } else if (auto* rb = qobject_cast<QRadioButton*>(w)) {
            kind = QStringLiteral("radio");
            val = rb->isChecked() ? QStringLiteral("1") : QStringLiteral("0");
        } else if (auto* co = qobject_cast<QComboBox*>(w)) {
            kind = QStringLiteral("combo");
            val = QString::number(co->currentIndex());
            extra = QStringLiteral(",\"text\":") + QString::fromStdString(jstr(co->currentText()));
        } else if (auto* ds = qobject_cast<QDoubleSpinBox*>(w)) {
            kind = QStringLiteral("dspin");
            val = QString::number(ds->value(), 'g', 9);
            extra = QStringLiteral(",\"min\":%1,\"max\":%2")
                        .arg(ds->minimum(), 0, 'g', 9).arg(ds->maximum(), 0, 'g', 9);
        } else if (auto* sp = qobject_cast<QSpinBox*>(w)) {
            kind = QStringLiteral("spin");
            val = QString::number(sp->value());
            extra = QStringLiteral(",\"min\":%1,\"max\":%2").arg(sp->minimum()).arg(sp->maximum());
        } else if (auto* sl = qobject_cast<QAbstractSlider*>(w)) {
            kind = QStringLiteral("slider");
            val = QString::number(sl->value());
            extra = QStringLiteral(",\"min\":%1,\"max\":%2").arg(sl->minimum()).arg(sl->maximum());
        } else if (auto* le = qobject_cast<QLineEdit*>(w)) {
            kind = QStringLiteral("text");
            val = QStringLiteral("0");
            extra = QStringLiteral(",\"text\":") + QString::fromStdString(jstr(le->text()));
        } else if (auto* pb = qobject_cast<QPushButton*>(w)) {
            // 【按钮也进快照】它没有"值"，但它的【可用性和文字】是状态：
            // "结束ROM" 这四个字出现在按钮上，本身就说明当时正在标定。
            kind = QStringLiteral("button");
            val = pb->isEnabled() ? QStringLiteral("1") : QStringLiteral("0");
            extra = QStringLiteral(",\"text\":") + QString::fromStdString(jstr(pb->text()));
        } else {
            continue;
        }
        if (!first) j += ",";
        first = false;
        j += "{\"name\":" + jstr(pr.second);
        j += ",\"kind\":" + jstr(kind);
        j += ",\"value\":" + val.toStdString();
        j += ",\"enabled\":" + std::string(w->isEnabled() ? "true" : "false");
        // 【tooltip 一起存】这些控件的 tooltip 里写着单位和量纲，
        // 而"0.35 是弧度还是度"这种问题，事后没有 tooltip 就只能翻源码。
        if (!w->toolTip().isEmpty()) {
            QString tip = w->toolTip().section(QLatin1Char('\n'), 0, 0).left(120);
            j += ",\"tip\":" + jstr(tip);
        }
        j += extra.toStdString();
        j += "}";
    }
    j += "]}";
    return j;
}

// ---------------------------------------------------------------------------
// 给面板上所有控件接上操作事件。在 buildParamSections() 末尾调一次。
//
// 【lambda 里捕的是 QPointer 语义上的裸指针】这些控件的生命周期跟对话框
// 一致，且 connect 的 context 就是 this，对话框析构时连接自动断开 ——
// 不会出现悬垂调用。
// ---------------------------------------------------------------------------
void PointCloudTestDialog::installUiEventRecorder() {
    for (const auto& pr : uiControls()) {
        QWidget* w = pr.first;
        const QString nm = pr.second;

        if (auto* cb = qobject_cast<QCheckBox*>(w)) {
            connect(cb, &QCheckBox::toggled, this, [this, nm](bool on) {
                recordUiEvent(nm, QStringLiteral("check"), on ? 1 : 0, on ? 0 : 1,
                              on ? QStringLiteral("关 -> 开") : QStringLiteral("开 -> 关"));
            });
        } else if (auto* rb = qobject_cast<QRadioButton*>(w)) {
            connect(rb, &QRadioButton::toggled, this, [this, nm](bool on) {
                // 【只记选中】一组单选里取消选中总是伴随另一个选中，
                // 两条一起记等于每次操作都写两遍，看起来像抖动。
                if (on) recordUiEvent(nm, QStringLiteral("radio"), 1, 0,
                                      QStringLiteral("被选中"));
            });
        } else if (auto* co = qobject_cast<QComboBox*>(w)) {
            // 旧值要在信号之外自己记：currentIndexChanged 只给新值。
            auto* prev = new int(co->currentIndex());
            connect(co, &QComboBox::destroyed, this, [prev]() { delete prev; });
            connect(co, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
                    [this, nm, co, prev](int i) {
                        recordUiEvent(nm, QStringLiteral("combo"), i, *prev,
                                      QStringLiteral("%1 -> %2(%3)")
                                          .arg(*prev).arg(i).arg(co->itemText(i)));
                        *prev = i;
                    });
        } else if (auto* ds = qobject_cast<QDoubleSpinBox*>(w)) {
            auto* prev = new double(ds->value());
            connect(ds, &QDoubleSpinBox::destroyed, this, [prev]() { delete prev; });
            connect(ds, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                    [this, nm, prev](double v) {
                        recordUiEvent(nm, QStringLiteral("dspin"), v, *prev, QString());
                        *prev = v;
                    });
        } else if (auto* sp = qobject_cast<QSpinBox*>(w)) {
            auto* prev = new int(sp->value());
            connect(sp, &QSpinBox::destroyed, this, [prev]() { delete prev; });
            connect(sp, QOverload<int>::of(&QSpinBox::valueChanged), this,
                    [this, nm, prev](int v) {
                        recordUiEvent(nm, QStringLiteral("spin"), v, *prev, QString());
                        *prev = v;
                    });
        } else if (auto* sl = qobject_cast<QAbstractSlider*>(w)) {
            auto* prev = new int(sl->value());
            connect(sl, &QAbstractSlider::destroyed, this, [prev]() { delete prev; });
            // 【用 sliderReleased 而不是 valueChanged】拖动滑条会每像素发一次，
            // 一次拖动能写出几百条事件，把真正的操作淹掉。
            connect(sl, &QAbstractSlider::sliderReleased, this, [this, nm, sl, prev]() {
                if (sl->value() != *prev) {
                    recordUiEvent(nm, QStringLiteral("slider"), sl->value(), *prev, QString());
                    *prev = sl->value();
                }
            });
        } else if (auto* pb = qobject_cast<QPushButton*>(w)) {
            // ---- 按钮：pressed() 抢时间戳 + 现读文字，clicked()/toggled() 才落盘 ----
            // 【原因见 recordUiEvent 里那段注释】按钮的 clicked()/toggled() 上可能
            // 还接着 app 自己的处理器，而且【很可能先接】（因为业务连线在
            // buildParamSections 里，我们的通用录制器在构造函数最后才接）。
            // 如果那个处理器里弹了模态框（romBtn_ 结束标定时就会），
            // 我们要是也接在 clicked()/toggled() 上取"现在"当时间戳，
            // 记下来的就是"用户把弹窗点掉的那一刻"，不是"用户点这个按钮的
            // 那一刻"——两者能差出一两秒，在时间轴上看着像点了两次。
            //
            // pressed() 是鼠标按下的瞬间发的，在 clicked() 之前、且不会被
            // 后面任何槽的阻塞拖慢（它是独立的一次信号发射）。这里先把
            // 时间戳和【此刻的按钮文字】存起来，落盘时用这份，不用"现在"
            // 读到的文字——按钮文字这时可能已经被业务处理器改掉了。
            auto* pressWall = new qint64(-1);
            auto* pressText = new QString();
            connect(pb, &QPushButton::destroyed, this, [pressWall, pressText]() {
                delete pressWall; delete pressText;
            });
            connect(pb, &QAbstractButton::pressed, this, [pb, pressWall, pressText]() {
                *pressWall = QDateTime::currentMSecsSinceEpoch();
                *pressText = pb->text();
            });
            if (pb->isCheckable()) {
                connect(pb, &QPushButton::toggled, this,
                        [this, nm, pressWall, pressText](bool on) {
                            // 没有对应的 pressed()（比如代码里 setChecked() 触发的），
                            // 退回"现在"——这类情况本来就不会被模态框拖慢。
                            recordUiEvent(nm, QStringLiteral("toggle"), on ? 1 : 0, on ? 0 : 1,
                                          (on ? QStringLiteral("按下：") : QStringLiteral("弹起："))
                                              + *pressText,
                                          *pressWall);
                            *pressWall = -1;
                        });
            } else {
                connect(pb, &QPushButton::clicked, this,
                        [this, nm, pressWall, pressText]() {
                            recordUiEvent(nm, QStringLiteral("click"), 1, 0,
                                          QStringLiteral("点击（此刻显示：") + *pressText
                                              + QStringLiteral("）"),
                                          *pressWall);
                            *pressWall = -1;
                        });
            }
        }
    }
}

} // namespace mocap
