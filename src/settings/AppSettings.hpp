#pragma once
// 应用设置持久化（QSettings，Windows 下写注册表 HKCU\Software\MocapLab\MocapHost）。
// 承载“开机默认参数” + 上次的相机清单（下次启动自动恢复）+ 窗口几何。
#include "core/Types.hpp"
#include <QSettings>
#include <QStringList>
#include <QSize>

namespace mocap {

struct AppConfig {
    CameraParams defaults;        // 新相机套用的应用层默认参数
    int  autoStartCameras = 0;    // 无相机清单时，开机自动创建几路虚拟相机
    QStringList cameraKeys;       // 上次的相机清单："virt" 或 "web:<设备id>"
    QByteArray windowGeometry;    // 主窗口几何
};

// 单台相机的"格式(分辨率/像素格式/帧率) + 曝光"偏好——跟 AppConfig 那种
// 全局唯一值不同，这是按 deviceKey 区分、每台相机各存一份的。这两项以前
// 完全没有持久化：格式协商每次开机都走 pickBestFormat() 的固定优先级，
// 曝光每次开机都是驱动默认(通常是自动)，导致"这次调好的组合"下次开机就
// 丢了，必须去参数面板重新点一遍。见 CamParamDialog.cpp 里"应用格式"/
// "强制应用所选具体格式"和曝光行改动时的保存点，以及 CameraManager::
// addWebcam() 里开机自动恢复的逻辑。
struct CameraFormatExposurePrefs {
    bool hasFormat = false;
    QSize resolution;
    int    pixelFormat = -1;   // QVideoFrameFormat::PixelFormat 的整数值，-1=未保存过
    double frameRate = 0.0;    // 【格式身份的一部分】= 那条格式的 maxFrameRate，
                               // 配合 resolution+pixelFormat 在 formats() 里精确找回同一条。
                               // 【不要往这里写用户选的帧率】开机恢复是拿它跟
                               // f.maxFrameRate() 逐条比对来认格式的，写成 15 就
                               // 再也匹配不上那条 [15,30] 的格式，整个格式恢复失效。
    double targetFps = 0.0;    // 用户在区间内指定的帧率。0 = 没指定，按标称上限跑。
                               // 只有 DirectShow 引擎能兑现，见 WebcamCamera::setFormat。

    bool hasExposure = false;
    bool exposureAuto = true;
    long exposureValue = 0;
};

class AppSettings {
public:
    AppSettings();
    AppConfig load();
    void save(const AppConfig& cfg);

    CameraFormatExposurePrefs loadCameraPrefs(const QString& deviceKey);
    void saveCameraFormat(const QString& deviceKey, const QSize& resolution,
                          int pixelFormat, double frameRate, double targetFps = 0.0);
    void saveCameraExposure(const QString& deviceKey, bool isAuto, long value);

    // ---- 稳定相机编号（界面上那个 "#N"）----
    //
    // 【为什么必须持久化，不能现算】现算的序号有两种，都不能用：
    //   a) "同名设备里的第几个"（枚举序）—— 插拔一次就重排，而且添加对话框
    //      走 DirectShow 枚举、相机对象走 Qt 枚举，两份顺序不保证一致；
    //   b) "相机列表里的第几个"（mgr_ 下标）—— 先加 3 号再加 1 号就错位，
    //      而且拖拽换位会让编号跟着位置跑，"3 号相机"指的是哪台就说不清了。
    // 两种都会出现"同一台相机在两个界面里是两个号"，正是要修的问题。
    //
    // 这里按 DevicePath 派生的稳定 tag 存一份 tag -> N 的表，首次见到分配
    // 最小可用编号，此后【永远是这个号】：重启不变、枚举顺序变了不变、
    // 拖拽换位不变。tag 的含义见 CameraLabel.hpp —— 有序列号的设备换 USB 口
    // 也认得出，没序列号的按端口认（换口 = 新号，那本来就该当成另一台，
    // 标定外参也对不上了，相机重映射对话框就是处理这件事的）。
    int cameraNumber(const QByteArray& devicePath);

private:
    QSettings s_;
};

} // namespace mocap
