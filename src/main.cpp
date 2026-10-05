#include "ui/MainWindow.hpp"
#include "ui/Theme.hpp"
#include "camera/ICamera.hpp"            // qRegisterMetaType 需要看到 Blob 的完整定义
#include "estimate/SmootherWorker.hpp"   // qRegisterMetaType 需要看到 HandPoseState/SmootherMeasurementIn 的完整定义
#include <QApplication>
#include <QStyleFactory>
#include <QLocale>
#include <QFont>
#include <QTimer>
#include <cstdio>

int main(int argc, char* argv[])
{
    // 全局强制 C locale：避免某些 Windows 区域/数字格式设置下，Qt 数值控件
    // （QSpinBox 等）用非拉丁数字符号绘制，导致界面里的数字显示花掉。
    QLocale::setDefault(QLocale::c());

    // 跨线程排队传递的自定义类型，统一在这里注册，创建任何窗口/相机/
    // worker之前：
    //   - QVector<mocap::Blob>：ICamera.hpp 顶部注释里"建议"的这行之前
    //     一直没有真的加进来(翻这份 main.cpp 时发现的)——多相机跨线程
    //     采集，blobDetailsReady 用的是普通 connect()，Qt6 对这种情况的
    //     自动类型注册容忍度比较高，大概率一直侥幸没出问题，但严格说
    //     跟下面两行是同一类隐患，一并补上，不留着赌运气。
    //   - HandPoseState/QVector<mocap::SmootherMeasurementIn>：延迟精修流
    //     (HandTrackingWorker::setSmootherEnabled)跨线程用
    //     QMetaObject::invokeMethod + Q_ARG 调用 SmootherWorker::processFrame，
    //     这个调用路径比普通 connect() 更依赖类型被显式注册过，不注册
    //     大概率会直接出问题，不是"侥幸"能跳过的。
    qRegisterMetaType<QVector<mocap::Blob>>();
    qRegisterMetaType<mocap::HandPoseState>();
    qRegisterMetaType<QVector<mocap::SmootherMeasurementIn>>();

    QApplication app(argc, argv);
    QApplication::setOrganizationName(QStringLiteral("MocapLab"));
    QApplication::setApplicationName(QStringLiteral("MocapHost"));
    // 防御性设置：只有主窗口 MainWindow 自己关闭时才退出程序。默认的
    // "最后一个窗口关闭就退出"规则在某些模态子窗口（标定模板库/三角化
    // 调试这类弹窗）的关闭时序下可能被意外触发，导致关掉一个子窗口就
    // 把整个程序带走——这里显式关掉这条隐式规则，退出只认 MainWindow
    // 自己的关闭事件（或显式调用 qApp->quit()）。
    QApplication::setQuitOnLastWindowClosed(false);

    // Fusion 作基底，叠加扁平浅色样式表。
    app.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    QFont f = app.font();
    f.setPointSizeF(f.pointSizeF() > 0 ? 9.5 : 9.5);
    app.setFont(f);
    app.setStyleSheet(mocap::theme::styleSheet());

    mocap::MainWindow w;
    // 【开机全屏】动捕工位基本上一开就占满屏幕用，默认给最大化。
    // 用 showMaximized() 而不是 showFullScreen()：标题栏/菜单栏还在，
    // 不至于一开机就没法拖窗口；要真正的无边框全屏按 F11（见 buildMenuBar）。
    // restoreGeometry() 仍然照常生效 —— 它恢复的是"取消最大化之后"的那个尺寸。
    w.showMaximized();

    // 【截图开关】设了环境变量 MOCAP_SHOT=<png 路径> 就在启动几秒后把主窗口
    // 存成一张图然后退出。
    //
    // 【为什么要走 Qt 自己的 grab()，不用系统截屏】QWidget::grab() 是让 Qt 把
    // 控件自己重绘到一张 QPixmap 上，不经过 DWM/GPU 合成，也不需要"当前有交互
    // 桌面"。远程会话、无桌面环境，或者用 PrintWindow/CopyFromScreen 只能拿到
    // 半张图（下半截全黑、状态栏没画出来）的时候，这条路照样是完整的。
    // 拍宣传素材时也用得上：MOCAP_SHOT=D:\shot.png 启动一次就得到一张干净的主
    // 窗口图，不用手动截屏、不会被别的窗口挡住。
    const QByteArray shotPath = qgetenv("MOCAP_SHOT");
    if (!shotPath.isEmpty()) {
        QTimer::singleShot(4000, &w, [&w, shotPath] {
            const QString p = QString::fromLocal8Bit(shotPath);
            const bool ok = w.grab().save(p);
            fprintf(stderr, "[MOCAP_SHOT] %s -> %s\n", ok ? "saved" : "FAILED",
                    p.toLocal8Bit().constData());
            QCoreApplication::quit();
        });
    }
    return app.exec();
}