#pragma once
// ---------------------------------------------------------------------------
// 原生 DirectShow 采集原型(第一阶段：只验证"绕开 Qt/Media Foundation 能不能
// 真正拿到硬件级 120fps"这件事本身，不追求跟 WebcamCamera 现有功能对齐)。
//
// 背景(详见 开发同步README.md / 系统标定与测量操作手册.md 里这次的排查
// 记录)：AMCap(纯 DirectShow) 在这台机器/这颗相机上用 MJPG 1280x720 能跑
// 满 ~121fps；同一颗相机同一格式，走 Qt6 的 QCamera(Windows 下只有 Media
// Foundation 一条后端) 只能拿到 ~63fps。这个差距被反复测试确认发生在
// Qt/MF 这条软件路径内部——不是硬件/USB带宽的问题(未压缩格式测出来的
// "70~100fps"经确认是这颗红外相机把灰度数据套进YUY2容器、不是真正意义上
// 的彩色YUY2，厂商规格表里"YUY2只有10fps"针对的是别的场景，不代表这颗
// 相机；但压缩MJPG这条路径的63fps vs AMCap的121fps这个差距是真实、可重复
// 的，只能通过绕开Qt/MF来解决)。
//
// 这个类直接调 DirectShow API(跟 AMCap 同一套)，绕开 QCamera/Media
// Foundation 整条链路：
//   视频采集设备 -> (IAMStreamConfig 设成 MJPG@目标分辨率@目标帧率)
//   -> ISampleGrabber(拿到原始MJPG字节，回调在DirectShow自己的采集线程上)
//   -> Null Renderer(终结这条图，不需要DirectShow自己弹窗口渲染)
// 拿到压缩字节后用 Qt 自带的 JPEG 解码(QImage::fromData)解出来——这跟
// WebcamCamera::onFrame() 里对压缩帧做的事(f.toImage())本质上是同一件事，
// 唯一区别是这次解码发生在我们自己完全掌控的线程上，不经过 Frame Server /
// Media Foundation 内部转换那一层，这正是这次要验证的假设。
//
// 【已知依赖风险，不是代码写错了】ISampleGrabber 来自 qedit.dll(DirectShow
// Editing Services)，微软很多年前就标记它"已弃用"，部分较新的Windows SDK
// 不再自带 qedit.h，且系统上 qedit.dll 有时没被注册。这个文件里自己声明了
// 需要用到的 ISampleGrabber/ISampleGrabberCB 接口(不 #include <qedit.h>)，
// 避免"开发机上有qedit.h、目标机上没有"这种环境差异导致的编译失败——这是
// OpenCV/ffmpeg等项目处理同一个问题时的标准做法，不是我们自己发明的。
// 但如果 CoCreateInstance(CLSID_SampleGrabber,...) 在运行时失败，大概率
// 是目标机器上 qedit.dll 没注册，需要以管理员身份跑一次：
//     regsvr32 qedit.dll
// 这一点做不到"编译期就能保证"，只能等真机跑起来验证。
//
// 【范围声明】这是第一阶段原型，刻意只做："打开设备 -> 协商MJPG格式 ->
// 起流 -> 每帧解码 -> 报告真实测得的fps"，不接入 rawFrameReady/
// frameForDetect/预览节流/灰度输出这些 WebcamCamera 现有功能——先验证
// "绕开Qt能不能真解决问题"这件事本身，验证过了再考虑怎么并入正式代码，
// 不要一次性把两件风险都担了(COM代码能不能编过/跑起来 + 跟现有系统集成
// 对不对)。
// ---------------------------------------------------------------------------
#include <QObject>
#include <QImage>
#include <QString>
#include <QByteArray>

namespace mocap {

class DShowCapture : public QObject {
    Q_OBJECT
public:
    explicit DShowCapture(QObject* parent = nullptr);
    ~DShowCapture() override;

    // deviceId/friendlyName/nameOccurrenceIndex：跟 DShowControl 构造函数
    // 参数含义完全一致(内部直接调用 DShowControl::findMoniker 绑定同一台
    // 物理设备)，直接从 WebcamCamera::device().id()/description()/
    // nameOccurrenceIndex() 传进来。
    // width/height/fps：目标格式。协商时优先 NV12/YUY2 这类未压缩格式
    // (零解码直取灰度 Y 平面)，协商不到再退回压缩 MJPG；系统内置摄像头常见
    // 的 YUY2 也在这里被支持，否则这类相机无法通过 DirectShow 设置帧率。
    // enforceFps=true：除了把 fps 作为 AvgTimePerFrame 下发，还在回调层做抽帧
    // 限流——驱动若只支持离散帧间隔、把 15fps 量化回 30，应用层仍按 15 交付。
    // 返回 false 时看 errorOccurred 信号里的具体原因。
    bool start(const QByteArray& deviceId, const QString& friendlyName,
              int nameOccurrenceIndex, int width, int height, double fps,
              bool enforceFps = false);
    void stop();
    bool isRunning() const { return running_; }

    // 采集格式开关(供UI快捷按钮调)：true=优先无压缩 NV12(零解码、低发热，
    // 协商不到再退回 MJPG)；false=强制走老的 MJPG 压缩路径(每帧软解 JPEG)。
    // 格式是在 start() 里协商定下的，所以运行中切换需要 stop() 后重新 start()
    // 才生效——UI 那边点按钮时顺带重启一下采集即可。默认 true。
    void setPreferUncompressed(bool on) { preferUncompressed_ = on; }
    bool preferUncompressed() const { return preferUncompressed_; }

signals:
    // 每解码出一帧就发一次。ts_ns 是 DirectShow 采样时间换算出来的相对
    // 时间戳(不是绝对挂钟时间)——这是原型阶段的简化处理，真正并入
    // WebcamCamera 时需要跟现有 ts_ns 语义(camera_->cameraFormat()相关
    // 的那套)对齐，这里先不做这件事。
    void frameReady(QImage frame, qint64 ts_ns);
    // 每约1秒更新一次，基于"成功解码的帧数"计算的真实帧率——不是轮询
    // 次数，也不是DirectShow自己报的理论值，是这次真正关心的那个数字。
    void measuredFpsChanged(double fps);
    void errorOccurred(QString message);

private:
    struct Impl;   // 藏起所有 COM 类型，这个头文件保持不引 Windows 头
    Impl* impl_ = nullptr;
    bool running_ = false;
    bool preferUncompressed_ = false;   // 见 setPreferUncompressed()——默认走已验证能用的 MJPG，
                                         // NV12 是新路径，确认没问题前不默认开，避免合并后直接无信号
};

} // namespace mocap
