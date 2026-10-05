#pragma once
// ---------------------------------------------------------------------------
// 系统摄像头：基于 Qt Multimedia（Windows 下走 Media Foundation）。
// QMediaDevices 枚举到的任何视频设备（内置摄像头、USB/UVC 红外相机…）
// 都可用本类接入。硬件参数（亮度/曝光/白平衡…）不在这里，
// 由 DShowControl 直连 DirectShow 属性接口（与 AMCap 同一套）。
// ---------------------------------------------------------------------------
#include "camera/ICamera.hpp"
#include "detect/CentroidDetector.hpp"
#include "detect/DetectWorker.hpp"
#include <QCameraDevice>
#include <QCameraFormat>
#include <QVideoFrameFormat>
#include <QSize>
#include <atomic>
#include <mutex>
#include <algorithm>
#include <cmath>

class QCamera;
class QVideoSink;
class QThread;

namespace mocap {

// 压缩格式(如 MJPEG)在同等标称帧率下通常比未压缩格式更能真实达到那个
// 帧率(USB带宽占用小)——这个判断在“启动时自动选格式”(pickBestFormat)
// 和“参数面板手动选格式”(CamParamDialog::applyCurrentFormatSelection)
// 两处都需要用同一个标准，避免两处各写一份、渐渐长出不一致的行为。
inline bool isCompressedFormat(QVideoFrameFormat::PixelFormat pf) {
    return pf == QVideoFrameFormat::Format_Jpeg;
}

// 这一档能不能【真的】跑出 wantFps —— 判据不是"数字在不在列表里"。
// QCameraFormat 是一条 (分辨率, 像素格式, minFrameRate, maxFrameRate) 描述符：
//   · maxFrameRate == wantFps  -> 标称档，两条引擎都能跑到
//   · min <= wantFps < max     -> 只是【区间包含】。QCameraFormat 表达不了
//                                 "在区间内取某个值"，Qt/Media Foundation 路径
//                                 会按标称上限跑；只有 DirectShow 路径能通过
//                                 IAMStreamConfig 设 AvgTimePerFrame 真正做到。
inline bool formatHitsExactly(const QCameraFormat& f, double wantFps) {
    return std::abs(double(f.maxFrameRate()) - wantFps) <= 0.5;
}
inline bool formatCoversRate(const QCameraFormat& f, double wantFps) {
    return wantFps >= double(f.minFrameRate()) - 0.5 &&
           wantFps <= double(f.maxFrameRate()) + 0.5;
}

// 在 fmts 里找“分辨率==res 且能跑出 wantFps”的一条。多条打平时优先选压缩
// 格式——原因：部分驱动/相机会对未压缩格式乐观地上报一个实际USB带宽扛不住
// 的标称帧率(比如同一分辨率下未压缩格式也标"120fps"，但实际交付时因为带宽
// 不够被迫压到60多)，压缩格式因为码率小，标称帧率更可信。
//
// 【三级优先，不能只按"距离最近"】原来的实现是
//     d = min(|max - want|, |min - want|)
// 也就是靠 minFrameRate 命中也算距离 0。于是"想要 15、有一条 [15,30] 的格式"
// 会被判成完美匹配选中，而 startWithFormat() 随后拿 maxFrameRate 去启动引擎，
// 实际跑 30 —— 用户选了 15，预览左上角的【实测】帧率却一直是 30，还查不出
// 哪一步丢的。现在把三种情况分开排队：
//   ① 有标称档恰好等于目标      -> 用它，两条引擎都能兑现
//   ② 没有，但有格式区间覆盖目标 -> 用它，能不能兑现取决于走哪条引擎，
//                                  由调用方决定是否提示（见 CamParamDialog）
//   ③ 都没有                    -> 退回"最接近"，保持老行为不返回空
inline QCameraFormat pickFormatMatching(const QList<QCameraFormat>& fmts,
                                         const QSize& res, double wantFps) {
    QCameraFormat exact, ranged, nearest;
    double exactD = 1e18, rangedD = 1e18, nearestD = 1e18;
    bool exactC = false, rangedC = false, nearestC = false;

    // 打平(±0.5fps)时压缩格式优先，三个档位共用同一条取舍规则。
    auto take = [](double d, double bd, bool comp, bool bcomp, bool isNull) {
        return isNull || d < bd - 0.5 || (d < bd + 0.5 && comp && !bcomp);
    };

    for (const QCameraFormat& f : fmts) {
        if (f.resolution() != res) continue;
        const bool comp = isCompressedFormat(f.pixelFormat());
        const double dMax = std::abs(double(f.maxFrameRate()) - wantFps);

        if (dMax <= 0.5 && take(dMax, exactD, comp, exactC, exact.isNull())) {
            exact = f; exactD = dMax; exactC = comp;
        }
        if (formatCoversRate(f, wantFps) &&
            take(0.0, rangedD, comp, rangedC, ranged.isNull())) {
            ranged = f; rangedD = 0.0; rangedC = comp;
        }
        const double dNear = std::min(dMax, std::abs(double(f.minFrameRate()) - wantFps));
        if (take(dNear, nearestD, comp, nearestC, nearest.isNull())) {
            nearest = f; nearestD = dNear; nearestC = comp;
        }
    }
    if (!exact.isNull())  return exact;
    if (!ranged.isNull()) return ranged;
    return nearest;
}

    class CaptureWorker;   // 采集/预览工作线程对象（见 .cpp）

    class WebcamCamera : public ICamera {
        Q_OBJECT
    public:
        WebcamCamera(quint32 id, const QCameraDevice& dev, QObject* parent = nullptr);
        ~WebcamCamera() override;

        bool start() override;
        void stop()  override;
        void applyParams(const CameraParams& p) override { params_ = p; }
        CameraParams params() const override { return params_; }

        QString deviceKey() const override;
        bool hasHardwareParams() const override { return true; }

        // 同型号相机友好名完全一样（DShowControl 路径匹配大概率失败时会退化按
        // 名字匹配），这里给出"我是同名设备里的第几个（0-based）"，构造
        // DShowControl 时必须传这个，否则同名设备全部会被绑到第一个上。
        int nameOccurrenceIndex() const;

        const QCameraDevice& device() const { return dev_; }

        // “格式”页（= AMCap 的 Video Capture Pin）：分辨率/帧率选择。
        QList<QCameraFormat> formats() const { return dev_.videoFormats(); }
        QCameraFormat currentFormat() const;
        // targetFps > 0：在这条格式的区间内【指定】帧率，而不是按标称上限跑。
        // 只有 DirectShow 引擎能兑现（IAMStreamConfig 设 AvgTimePerFrame）；
        // 走 Qt/Media Foundation 时会被忽略，因为 QCameraFormat 表达不了它。
        // 传 0 = 老行为：用格式自己的 maxFrameRate。
        bool setFormat(const QCameraFormat& f, double targetFps = 0.0);
        // 上一次 setFormat 请求的区间内帧率（0 = 没请求过，跑的是标称上限）。
        // 参数面板拿它做回读提示，存盘拿它记住"用户到底要的是多少"。
        double requestedFrameRate() const { return requestedFps_; }

        // 从驱动上报的格式里挑“最高帧率优先、再取较高分辨率”的一档。
        // 接入时自动应用，保证 USB 相机一上来就跑满帧，而不是撞到低帧率默认档。
        QCameraFormat bestFormat() const;

    signals:
        // 采集引擎每次(重)启动完成。【上层靠它补推硬件参数】——
        // setFormat() 内部是 stopCapture()+startWithFormat()，等于把设备关掉
        // 重开，之前推下去的曝光/亮度等会被驱动复位。谁想让硬件参数在换格式
        // 之后仍然成立，就必须接这个信号、在它之后再推一次，而不是推完就走。
        void captureEngineStarted(quint32 camId);

    public:

        // 算法通道开关：默认关闭。从主线程调用，转发给 worker（worker 里是 atomic）。
        void setRawFrameEnabled(bool on);
        bool rawFrameEnabled() const { return rawEnabled_; }

        // 质心检测（覆盖 ICamera）。跑在独立于采集/预览的检测线程里，互不拖累
        // （见 DetectWorker）。不再依赖算法通道开关，可单独打开。
        void setDetectEnabled(bool on) override;
        bool detectEnabled() const override { return detectOn_; }
        void setDetectParams(const DetectParams& p) override;

        void setGrayOutput(bool on) override;
        bool grayOutput() const override { return grayOut_; }

        void setPreviewMode(int m) override;

        // 手部动捕(3a~3d)需要轮廓点时打开——转发给 detectWorker_（原子量，
        // 跟 setDetectEnabled 转发 detectOn_ 是同一个模式）。实现见 .cpp：
        //   void WebcamCamera::setContourCollectionEnabled(bool on) {
        //       contourOn_ = on;
        //       if (detectWorker_) detectWorker_->setContourCollectionEnabled(on);
        //   }
        // 另外 detectWorker_ 的 blobDetailsReady 需要在构造/start()里跟
        // blobsReady 一样转发到 ICamera 自己的同名信号上（信号到信号连接），
        // 具体在哪一行取决于你 blobsReady 现在是怎么转发的——我没有
        // WebcamCamera.cpp，这一行需要你参照 blobsReady 现有的那行加一句
        // 同样写法的 blobDetailsReady 转发。
        void setContourCollectionEnabled(bool on) override;
        bool contourCollectionEnabled() const override { return contourOn_; }

    private:
        QCameraDevice dev_;
        CameraParams params_;
        double requestedFps_ = 0.0;   // 见 requestedFrameRate()
        CaptureWorker* worker_ = nullptr;   // 采集/预览：独立线程
        QThread* thread_ = nullptr;
        DetectWorker* detectWorker_ = nullptr;  // 检测：另一条独立线程，忙时丢帧
        QThread* detectThread_ = nullptr;

        // 主线程侧镜像的开关状态（start 时同步给 worker；运行中改直接写 worker）。
        std::atomic<bool> rawEnabled_{ false };
        std::atomic<bool> detectOn_{ false };
        std::atomic<bool> grayOut_{ false };
        std::atomic<int>  previewMode_{ 0 };
        std::atomic<bool> contourOn_{ false };
        std::mutex     dparamsMtx_;
        DetectParams   dparams_;   // 缓存：start() 前设置也不丢，start 时同步给 detectWorker_
    };

} // namespace mocap
