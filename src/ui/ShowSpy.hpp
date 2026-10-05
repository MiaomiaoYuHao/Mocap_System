// ===========================================================================
// ShowSpy.hpp —— 临时诊断：把"谁弹出了那个小窗口"直接打出来
//
// 【怎么用】只要两步，都在 src/main.cpp 里：
//
//   ① 在文件顶部的 #include 区加一行：
//          #include "ui/ShowSpy.hpp"
//
//   ② 在 main() 里、QApplication 构造之后、MainWindow 创建之前，加一行：
//          mocap::installShowSpy(app);
//
//   跑起来点「实时动捕」，控制台会打出每个顶层窗口的显示/隐藏。
//   查完把这两行删掉，这个文件也可以删。
//
// 【为什么做成头文件】上一版我把类定义写在注释外、用法写在注释里，
// 结果只有用法那两行被粘进去，类定义没跟过去，直接编译不过。
// 现在整个东西在这一个文件里，加两行就行，粘不错。
//
// 【不需要 moc】类里没有 Q_OBJECT、没有信号槽，只重写了虚函数
// eventFilter()。所以不用改 CMakeLists，AUTOMOC 也不用扫它。
// ===========================================================================
#pragma once

#include <QCoreApplication>
#include <QDebug>
#include <QEvent>
#include <QMetaObject>
#include <QObject>
#include <QString>
#include <QWidget>

namespace mocap {
namespace diag {

class ShowSpy : public QObject {
public:
    explicit ShowSpy(QObject* parent = nullptr) : QObject(parent) {}

protected:
    bool eventFilter(QObject* o, QEvent* e) override {
        const bool show = (e->type() == QEvent::Show);
        const bool hide = (e->type() == QEvent::Hide);
        if (show || hide) {
            auto* w = qobject_cast<QWidget*>(o);
            // 【只打顶层窗口】子控件的 Show 事件一次几百个，全打出来没法看。
            // isWindow() 为真才是独立窗口 —— 截图里那个有标题栏的小框
            // 一定满足这条。
            if (w && w->isWindow()) {
                ++n_;
                qInfo().noquote().nospace()
                    << (show ? "[显示 #" : "[隐藏 #") << n_ << "] "
                    << "类=" << w->metaObject()->className()
                    << "  名=" << (w->objectName().isEmpty()
                                   ? QStringLiteral("(无)") : w->objectName())
                    << "  标题=" << (w->windowTitle().isEmpty()
                                     ? QStringLiteral("(空)") : w->windowTitle())
                    << "  几何=" << geom(w)
                    << "  最小=" << w->minimumWidth() << "x" << w->minimumHeight()
                    << "  标志=0x" << QString::number(quint32(w->windowFlags()), 16)
                    << "  父=" << (w->parentWidget()
                                   ? w->parentWidget()->metaObject()->className()
                                   : "(无父=顶层)");
            }
        }
        return QObject::eventFilter(o, e);
    }

private:
    static QString geom(const QWidget* w) {
        return QStringLiteral("%1,%2 %3x%4")
            .arg(w->geometry().x()).arg(w->geometry().y())
            .arg(w->geometry().width()).arg(w->geometry().height());
    }
    int n_ = 0;
};

}  // namespace diag

// app 拥有这个对象，不用自己管生命周期。
inline void installShowSpy(QCoreApplication& app) {
    auto* spy = new diag::ShowSpy(&app);
    app.installEventFilter(spy);
    qInfo().noquote() << "[ShowSpy] 已挂上，开始记录顶层窗口的显示/隐藏";
}

}  // namespace mocap

// ---------------------------------------------------------------------------
// 输出示例
//
//   [ShowSpy] 已挂上，开始记录顶层窗口的显示/隐藏
//   [显示 #1] 类=MainWindow  名=(无)  标题=...  几何=100,80 1360x860
//             最小=0x0  标志=0x1  父=(无父=顶层)
//   [显示 #2] 类=QWidget  名=(无)  标题=(空)  几何=560,290 180x80
//             最小=0x0  标志=0x1  父=(无父=顶层)          ← 嫌疑犯
//   [隐藏 #3] 类=QWidget ...
//   [显示 #4] 类=mocap::PointCloudTestDialog  标题=实时动捕（…）
//             几何=230,90 1460x900  最小=640x420  父=MainWindow
//
// 【怎么读】
//   · 类名直接点名。如果是裸 QWidget 且"父=(无父=顶层)"，
//     那就是某处 new QWidget() 忘了给父对象 —— 无父的 QWidget 就是顶层窗口。
//   · 标志是十六进制的 Qt::WindowFlags：
//        0x1 = Window        0x2 = Dialog       0x8 = Popup
//        0x00040000 = WindowTitleHint           0x00008000 = WindowMaximizeButtonHint
//        0x00004000 = WindowMinimizeButtonHint  0x08000000 = WindowCloseButtonHint
//     截图里"有最大化、有关闭、没有最小化"这个组合很特殊，标志一打出来就能对上。
//   · 如果同一个类的 [显示]/[隐藏] 成对刷好几轮，次数往往等于相机台数
//     或某个循环的轮数 —— 那就直接指向那个循环。
//
// 【如果控制台看不到输出】
//   Qt Creator 里在「项目 → 运行」勾上"在终端中运行"，
//   或者看「应用程序输出」那一栏（qInfo 默认走 qt.core 的默认 handler）。
// ---------------------------------------------------------------------------
