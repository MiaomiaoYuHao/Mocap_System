#pragma once
// ---------------------------------------------------------------------------
// DirectShow 属性控制：与 AMCap 属性页调用的是同一套驱动接口。
//   视频 Proc Amp  -> IAMVideoProcAmp   （亮度/对比度/色调/饱和度/清晰度/
//                                        伽玛/启用颜色/白平衡/逆光对比/增益）
//   照相机控制     -> IAMCameraControl  （全景/倾斜/滚动/缩放/曝光/光圈/焦点）
// 这些是内核流(KS)属性，直达驱动，即使画面正由 Qt(Media Foundation) 采集，
// 通常也能同时生效——这正是很多上位机的做法。
// 头文件不引 Windows 头（用 void* 存 COM 指针），保持其余代码干净。
// ---------------------------------------------------------------------------
#include <QByteArray>
#include <QString>
#include <QVector>

namespace mocap {

    struct DShowProp {
        long    id = 0;                 // VideoProcAmp_* 或 CameraControl_*
        bool    isCameraControl = false;
        QString name;                   // 中文显示名（与 AMCap 对齐）
        long    min = 0, max = 0, step = 1, def = 0;
        long    value = 0;
        bool    supported = false;      // 设备是否支持该属性
        bool    autoSupported = false;  // 是否支持“自动”
        bool    manualSupported = false;
        bool    isAuto = false;         // 当前处于自动模式
    };

    // 一台视频设备的轻量标识。
    // 【只读属性包，不打开设备】这是跟 Qt 的 QMediaDevices::videoInputs()
    // 的关键区别：Qt 会把每个 moniker 绑成 IBaseFilter 再查 IAMStreamConfig
    // 拿格式列表，等于逐个打开摄像头；有相机正在独占推流时那会阻塞。
    // 这里只做 BindToStorage 读 DevicePath / FriendlyName，微秒级，且不碰设备。
    struct DShowDevice {
        QByteArray devicePath;   // 内核符号链接，跟 Qt 的 QCameraDevice::id() 同源
        QString    friendlyName;
    };
    // 枚举当前所有视频输入设备。可安全地反复调用（比如定时刷新列表）。
    QVector<DShowDevice> enumerateVideoDevices();

    class DShowControl {
    public:
        // 用 Qt 的设备 id（符号链接）与友好名去匹配 DirectShow 设备。
        // nameOccurrenceIndex：当多台相机友好名完全相同（同型号红外相机常见）
        // 且路径匹配失败退化到按名字匹配时，用它区分“这是同名设备里的第几个”
        // （0-based），而不是像以前那样永远命中枚举到的第一个同名设备。
        DShowControl(const QByteArray& qtDeviceId, const QString& friendlyName,
            int nameOccurrenceIndex = 0);
        ~DShowControl();

        bool valid() const;                       // 是否成功绑定到设备
        QVector<DShowProp> properties();          // 查询全部属性（范围+当前值）
        bool set(long propId, bool isCameraControl, long value, bool isAuto);

        // 【共享】按跟构造函数完全一致的两遍匹配规则（路径优先，退化到
        // "同名设备里第几个"）找到对应的 DirectShow 设备 moniker。返回
        // true 时 *outMonikerIUnknown 是一个已 AddRef 过的 IMoniker*（用
        // void* 存，保持这个头文件不引 Windows 头的约定），调用方负责在
        // 用完之后 Release()。非 Windows 平台直接返回 false。
        //
        // 这个函数存在的意义：DShowCapture（原生 DirectShow 采集原型）需要
        // 绑定到"跟 DShowControl 属性控制同一台物理设备"，两处如果各写
        // 一份匹配逻辑，以后改枚举方式很容易漏改其中一处、导致两边连到不
        // 同的相机却互相看不出来。两处共用这一份，天然保证一致。
        static bool findMoniker(const QByteArray& qtDeviceId, const QString& friendlyName,
                                int nameOccurrenceIndex, void** outMonikerIUnknown);

    private:
        void* filter_ = nullptr;   // IBaseFilter*
        void* amp_ = nullptr;   // IAMVideoProcAmp*
        void* cam_ = nullptr;   // IAMCameraControl*
        bool  comInit_ = false;
    };

} // namespace mocap