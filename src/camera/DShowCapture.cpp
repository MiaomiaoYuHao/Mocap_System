#include "camera/DShowCapture.hpp"
#include "camera/DShowControl.hpp"
#include "camera/JpegDecoder.hpp"

#ifdef _WIN32
// --------------------------------------------------------------------------
// Windows 实现
// --------------------------------------------------------------------------
#include <windows.h>
#include <dshow.h>
#include <QElapsedTimer>
#include <QDebug>
#include <QAbstractVideoBuffer>
#include <QMetaObject>
#include <QVideoFrame>
#include <QVideoFrameFormat>
#include <algorithm>
#include <cstring>
#include <memory>

namespace mocap {

namespace {

// 【跨相机统一时间基准】DirectShow 的 SampleTime 是"流时间"——相对于
// **这条图自己 Run() 的那一刻**，而每台相机都有一条自己独立的图(各自
// CoCreateInstance(CLSID_FilterGraph))，各自的零点是各自启动的时刻。
// 直接拿 SampleTime 当 ts_ns 用，两台先后启动的相机在同一个真实时刻报
// 出来的时间戳会差好几秒，下游 Triangulator 那道"多相机观测必须落在
// 12ms 窗口内"的门(tolNs_)就永远凑不齐两台，一个三角化点都出不来。
// 旧的 Qt/MF 路径没这个问题是因为 QVideoFrame::startTime() 走的是系统
// 级统一时钟，天然跨相机可比——换引擎时这个隐含前提必须自己补上。
//
// 这里用一个全进程共享的单调时钟：每条图在 Run() 成功后记下"我的零点
// 落在这个共享时钟的哪个位置"(graphStartNs)，之后每帧的 ts_ns =
// graphStartNs + SampleTime，各相机就都换算到同一根时间轴上了。
// 函数内静态变量的初始化在 C++11 起是线程安全的，多台相机并发第一次
// 调用不会重复初始化。
QElapsedTimer& sharedClock() {
    static QElapsedTimer clk = [] { QElapsedTimer t; t.start(); return t; }();
    return clk;
}

// ---- 手动声明 ISampleGrabber/ISampleGrabberCB，不依赖 <qedit.h> ----
// 原因见 DShowCapture.hpp 顶部注释：这两个接口来自已被微软标记弃用的
// qedit.dll，较新的 Windows SDK 不一定还带 qedit.h。这里用的 GUID 数值
// 是 DirectShow SDK 公开、稳定的常量(ffmpeg的libavdevice/dshow、OpenCV的
// dshow采集后端等开源项目用的都是同一组值)，自己声明接口可以彻底避免
// "开发机上有这个头文件、目标机上没有"这种环境差异导致编译失败。
const CLSID kClsidSampleGrabber =
    { 0xC1F400A0, 0x3F08, 0x11d3, { 0x9F, 0x0B, 0x00, 0x60, 0x08, 0x03, 0x9E, 0x37 } };
const CLSID kClsidNullRenderer =
    { 0xC1F400A4, 0x3F08, 0x11d3, { 0x9F, 0x0B, 0x00, 0x60, 0x08, 0x03, 0x9E, 0x37 } };
const IID kIidSampleGrabber =
    { 0x6B652FFF, 0x11FE, 0x4fce, { 0x92, 0xAD, 0x02, 0x66, 0xB5, 0xD7, 0xC7, 0x8F } };
const IID kIidSampleGrabberCB =
    { 0x0579154A, 0x2B53, 0x4994, { 0xB0, 0xD0, 0xE7, 0x73, 0x14, 0x8E, 0xFF, 0x85 } };

// NV12 的 media subtype GUID(FourCC 'NV12' = 0x3231564E，按 DirectShow
// "FourCC-0000-0010-8000-00AA00389B71" 的规则拼)。有些 Windows SDK 的
// dshow.h/uuids.h 里有 MEDIASUBTYPE_NV12、有些没有；跟本文件自声明
// ISampleGrabber 同样的道理，这里自己定义一个局部 GUID，不依赖 SDK 是否
// 提供，避免"开发机有、目标机没有"的编译差异。
const GUID kMediaSubtypeNV12 =
    { 0x3231564E, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 } };

// YUY2(打包 4:2:2)在系统内置摄像头里最常见——很多笔记本相机(例如 ASUS
// 5M WebCam)只上报 YUY2，既没有 NV12 也没有 MJPG。之前这里只试 NV12/MJPG，
// 于是这类相机走到 DirectShow 会返回"找不到匹配格式"，WebcamCamera 又退回
// Qt/Media Foundation，区间内指定的帧率被无声丢弃——这就是"选 15 还是 30"
// 的正主之一。YUY2 同样是未压缩格式，可以零解码直接取 Y 平面，只是 Y 是隔字节
// 存储的，需要逐像素抽取，不能像 NV12 那样整块搬。
const GUID kMediaSubtypeYUY2 =
    { 0x32595559, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 } };

struct ISampleGrabberCB : public IUnknown {
    virtual HRESULT __stdcall SampleCB(double SampleTime, IMediaSample* pSample) = 0;
    virtual HRESULT __stdcall BufferCB(double SampleTime, BYTE* pBuffer, long BufferLen) = 0;
};

struct ISampleGrabber : public IUnknown {
    virtual HRESULT __stdcall SetOneShot(BOOL OneShot) = 0;
    virtual HRESULT __stdcall SetMediaType(const AM_MEDIA_TYPE* pType) = 0;
    virtual HRESULT __stdcall GetConnectedMediaType(AM_MEDIA_TYPE* pType) = 0;
    virtual HRESULT __stdcall SetBufferSamples(BOOL BufferThem) = 0;
    virtual HRESULT __stdcall GetCurrentBuffer(long* pBufferSize, long* pBuffer) = 0;
    virtual HRESULT __stdcall GetCurrentSample(IMediaSample** ppSample) = 0;
    virtual HRESULT __stdcall SetCallback(ISampleGrabberCB* pCallback, long WhichMethodToCallback) = 0;
};

// 把 DirectShow 回调里的原始未压缩缓冲直接包成 QVideoFrame，交给 Qt 的
// 格式转换器转 RGB。以前 NV12/YUY2 为了省 CPU 只抽 Y 平面，彩色摄像头一
// 走 DirectShow 就永久变灰；现在只有全局"灰度输出"开关打开时才走零解码
// 灰度，否则用 Qt 的转换路径保留颜色。
class DShowVideoBuffer : public QAbstractVideoBuffer {
public:
    DShowVideoBuffer(const QVideoFrameFormat& fmt,
                     uchar* data0, int stride0, int size0,
                     uchar* data1 = nullptr, int stride1 = 0, int size1 = 0)
        : fmt_(fmt) {
        map_.planeCount = data1 ? 2 : 1;
        map_.data[0] = data0;
        map_.bytesPerLine[0] = stride0;
        map_.dataSize[0] = size0;
        if (data1) {
            map_.data[1] = data1;
            map_.bytesPerLine[1] = stride1;
            map_.dataSize[1] = size1;
        }
    }

    MapData map(QVideoFrame::MapMode) override { return map_; }
    void unmap() override {}
    QVideoFrameFormat format() const override { return fmt_; }

private:
    QVideoFrameFormat fmt_;
    MapData map_{};
};

static QImage decodeNv12Color(BYTE* buffer, long bufferLen, int W, int H)
{
    const int yStride = W;
    const int uvStride = W;
    const int uvH = (H + 1) / 2;
    const qint64 need = qint64(yStride) * H + qint64(uvStride) * uvH;
    if (!buffer || W <= 0 || H <= 0 || bufferLen < need) return {};

    QVideoFrameFormat fmt(QSize(W, H), QVideoFrameFormat::Format_NV12);
    // DirectShow 这里送来的 NV12 按行直接就是预览方向；不要根据 biHeight
    // 再翻一次，否则会上下颠倒。
    fmt.setScanLineDirection(QVideoFrameFormat::TopToBottom);
    QVideoFrame frame(std::make_unique<DShowVideoBuffer>(
        fmt, buffer, yStride, yStride * H,
        buffer + qint64(yStride) * H, uvStride, uvStride * uvH));
    return frame.toImage();
}

static QImage decodeYuy2Color(BYTE* buffer, long bufferLen, int W, int H)
{
    const int stride = W * 2;
    if (!buffer || W <= 0 || H <= 0 || bufferLen < qint64(stride) * H) return {};

    QVideoFrameFormat fmt(QSize(W, H), QVideoFrameFormat::Format_YUYV);
    fmt.setScanLineDirection(QVideoFrameFormat::TopToBottom);
    QVideoFrame frame(std::make_unique<DShowVideoBuffer>(
        fmt, buffer, stride, stride * H));
    return frame.toImage();
}

} // namespace

struct DShowCapture::Impl : public ISampleGrabberCB {
    // ---- IUnknown 假引用计数 ----
    // 这个回调对象的生命周期完全由 DShowCapture::Impl 自己的构造/析构
    // 控制，不需要真正的引用计数语义——DirectShow 采集回调对象用固定值
    // 应付 AddRef/Release 是常见写法，不是偷懒漏写。
    STDMETHODIMP_(ULONG) AddRef() override { return 1; }
    STDMETHODIMP_(ULONG) Release() override { return 1; }
    STDMETHODIMP QueryInterface(REFIID riid, void** ppv) override {
        if (!ppv) return E_POINTER;
        if (riid == IID_IUnknown || riid == kIidSampleGrabberCB) {
            *ppv = static_cast<ISampleGrabberCB*>(this);
            return S_OK;
        }
        *ppv = nullptr;
        return E_NOINTERFACE;
    }

    // 用 BufferCB(拿原始字节)而不是 SampleCB(拿 IMediaSample*)——不用碰
    // IMediaSample 的引用计数/生命周期管理，直接拷贝字节出来最省心。
    STDMETHODIMP SampleCB(double, IMediaSample*) override { return S_OK; }
    STDMETHODIMP BufferCB(double sampleTime, BYTE* buffer, long bufferLen) override {
        if (!owner || bufferLen <= 0 || !buffer) return S_OK;

        const qint64 ts_ns = graphStartNs + qint64(sampleTime * 1e9);

        if (!fpsClock.isValid()) { fpsClock.start(); fpsWindowStartMs = 0; }
        const qint64 nowMs = fpsClock.elapsed();
        if (nowMs - fpsWindowStartMs >= 1000) {
            const double fps = frameCount * 1000.0 / double(std::max<qint64>(1, nowMs - fpsWindowStartMs));
            frameCount = 0;
            fpsWindowStartMs = nowMs;
            emit owner->measuredFpsChanged(fps);
        }

        auto rateAllows = [&]() -> bool {
            if (targetFps <= 0.0) return true;
            const qint64 minIntervalNs = qint64(1000000000.0 / targetFps);
            const qint64 tolerance = std::max<qint64>(1000000LL, minIntervalNs / 3);
            if (lastEmitNs != 0 && (ts_ns - lastEmitNs) < (minIntervalNs - tolerance)) {
                ++droppedForRate;
                if (droppedForRate % 30 == 1) {
                    qWarning().noquote() << QStringLiteral(
                        "[DShowCapture] 抽帧限流生效：目标 %1fps，本帧间隔 %2ms，"
                        "已丢弃 %3 帧")
                        .arg(targetFps, 0, 'f', 1)
                        .arg(double(ts_ns - lastEmitNs) / 1e6, 0, 'f', 2)
                        .arg(droppedForRate);
                }
                return false;
            }
            return true;
        };
        if (!rateAllows()) return S_OK;

        auto markEmitted = [&]() {
            ++frameCount;
            lastEmitNs = ts_ns;
        };

        // 【关键线程边界】彩色 NV12/YUY2 绝不能在 BufferCB 里调用
        // QVideoFrame::toImage()。BufferCB 跑在 DirectShow 自己的采集线程上，
        // 切格式时 mediaControl->Stop() 必须等它返回；在回调里做 Qt 色彩转换，
        // Stop() 就会一直等，表现为整个程序卡死。这里只深拷贝原始字节并
        // queued invoke 到 DShowCapture 所属线程，转换在那里做。
        if ((useNv12 || useYuy2) && !isGrayDecode()) {
            const int W = nv12W, H = nv12H;
            const qint64 need = useNv12
                ? qint64(W) * H + qint64(W) * ((H + 1) / 2)
                : qint64(W) * 2 * H;
            if (W <= 0 || H <= 0 || bufferLen < need) {
                ++decodeFailCount;
                if (decodeFailCount % 10 == 1) {
                    qWarning().noquote() << QStringLiteral(
                        "[DShowCapture] 原始帧长度不足，丢弃这一帧(格式=%1, 收到字节数=%2, 需要=%3) 累计丢帧%4")
                        .arg(useNv12 ? QStringLiteral("NV12") : QStringLiteral("YUY2"))
                        .arg(bufferLen).arg(need).arg(decodeFailCount);
                }
                return S_OK;
            }

            const QByteArray raw(reinterpret_cast<const char*>(buffer), int(bufferLen));
            const int pixFormat = useNv12 ? int(QVideoFrameFormat::Format_NV12)
                                          : int(QVideoFrameFormat::Format_YUYV);
            markEmitted();
            QMetaObject::invokeMethod(owner,
                [owner = owner, raw, W, H, pixFormat, ts_ns]() mutable {
                    BYTE* p = reinterpret_cast<BYTE*>(const_cast<char*>(raw.constData()));
                    QImage img = (pixFormat == int(QVideoFrameFormat::Format_NV12))
                        ? decodeNv12Color(p, int(raw.size()), W, H)
                        : decodeYuy2Color(p, int(raw.size()), W, H);
                    if (!img.isNull())
                        emit owner->frameReady(img, ts_ns);
                }, Qt::QueuedConnection);
            return S_OK;
        }

        QImage img;
        if (useNv12) {
            const int W = nv12W, H = nv12H;
            if (W > 0 && H > 0 && bufferLen >= long(W) * H) {
                img = QImage(W, H, QImage::Format_Grayscale8);
                for (int y = 0; y < H; ++y)
                    std::memcpy(img.scanLine(y), buffer + y * W, W);
            }
        } else if (useYuy2) {
            const int W = nv12W, H = nv12H;
            const int srcStride = W * 2;
            if (W > 0 && H > 0 && bufferLen >= long(srcStride) * H) {
                img = QImage(W, H, QImage::Format_Grayscale8);
                for (int y = 0; y < H; ++y) {
                    const BYTE* row = buffer + y * srcStride;
                    uchar* dst = img.scanLine(y);
                    for (int x = 0; x < W; ++x) dst[x] = row[x * 2];
                }
            }
        } else {
            img = decodeMjpeg(reinterpret_cast<const uint8_t*>(buffer),
                              int(bufferLen));
        }

        if (!img.isNull()) {
            markEmitted();
            emit owner->frameReady(img, ts_ns);
        } else {
            ++decodeFailCount;
            if (decodeFailCount % 10 == 1) {
                qWarning().noquote() << QStringLiteral(
                    "[DShowCapture] 解码失败/数据不完整，丢弃这一帧(格式=%1, "
                    "收到字节数=%2) 累计丢帧%3")
                    .arg(useNv12 ? QStringLiteral("NV12")
                                 : useYuy2 ? QStringLiteral("YUY2")
                                           : QStringLiteral("MJPG"))
                    .arg(bufferLen).arg(decodeFailCount);
            }
        }

        return S_OK;
    }

    DShowCapture* owner = nullptr;
    // 采集格式：useNv12/useYuy2 分别表示未压缩 NV12/YUY2，两者都在回调里
    // 零解码直接取灰度 Y 平面(区别是 YUY2 的 Y 需要隔字节抽)；都 false 时
    // 是压缩 MJPG，走 Qt 软解。nv12W/nv12H 是协商生效的分辨率，未压缩格式
    // 解析必须知道(MJPG 自带尺寸不需要)，YUY2 也复用这两个字段。
    bool useNv12 = false;
    bool useYuy2 = false;
    int  nv12W = 0;
    int  nv12H = 0;
    qint64 frameCount = 0;
    qint64 fpsWindowStartMs = 0;
    qint64 decodeFailCount = 0;   // 见 BufferCB 里的诊断补丁注释
    double targetFps = 0.0;       // enforceFps=true 时启用；0=不限流
    qint64 lastEmitNs = 0;        // 上一次实际发出帧的 ts_ns(抽帧限流用)
    qint64 droppedForRate = 0;    // 因抽帧限流丢掉的帧数
    QElapsedTimer fpsClock;
    // 本图的流时间零点，落在 sharedClock() 那根共享时间轴上的位置。
    // Run() 成功之后立刻记一次，之后每帧用它把 SampleTime 换算成跨相机
    // 可比的绝对时间戳。
    qint64 graphStartNs = 0;

    IGraphBuilder*          graph = nullptr;
    ICaptureGraphBuilder2*  builder = nullptr;
    IBaseFilter*            captureFilter = nullptr;
    IBaseFilter*            grabberFilter = nullptr;
    IBaseFilter*            nullRendererFilter = nullptr;
    ISampleGrabber*         grabber = nullptr;
    IMediaControl*          mediaControl = nullptr;
    bool                    comInit = false;
};

DShowCapture::DShowCapture(QObject* parent) : QObject(parent) {}

DShowCapture::~DShowCapture() { stop(); }

bool DShowCapture::start(const QByteArray& deviceId, const QString& friendlyName,
                         int nameOccurrenceIndex, int width, int height, double fps,
                         bool enforceFps) {
    if (running_) return true;

    qWarning().noquote() << QStringLiteral(
        "[DShow] start 进入：name=%1 index=%2 size=%3x%4 fps=%5 enforce=%6")
        .arg(friendlyName).arg(nameOccurrenceIndex).arg(width).arg(height)
        .arg(fps, 0, 'f', 1).arg(enforceFps ? QStringLiteral("yes") : QStringLiteral("no"));

    impl_ = new Impl();
    impl_->owner = this;
    impl_->targetFps = (enforceFps && fps > 0.0) ? fps : 0.0;
    impl_->lastEmitNs = 0;
    impl_->droppedForRate = 0;

    // 用多线程套间(COINIT_MULTITHREADED)而不是 DShowControl 那边用的
    // 单线程套间——采集回调会在 DirectShow 自己创建的工作线程上被调用，
    // 不是我们能控制的线程，多线程套间模型更适合这种场景。如果某些老旧
    // filter 强制要求单线程套间导致连线失败，这是第一个该怀疑的点。
    const HRESULT hrInit = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    impl_->comInit = SUCCEEDED(hrInit);
    qWarning().noquote() << QStringLiteral("[DShow] start: CoInitializeEx hr=0x%1")
        .arg(quint32(hrInit), 8, 16, QLatin1Char('0'));

    void* monV = nullptr;
    if (!DShowControl::findMoniker(deviceId, friendlyName, nameOccurrenceIndex, &monV) || !monV) {
        emit errorOccurred(QStringLiteral("\u627e\u4e0d\u5230\u5339\u914d\u7684DirectShow\u8bbe\u5907(\u8def\u5f84/\u540d\u5b57\u90fd\u5bf9\u4e0d\u4e0a)"));
        stop();
        return false;
    }
    IMoniker* moniker = static_cast<IMoniker*>(monV);
    HRESULT hr = moniker->BindToObject(nullptr, nullptr, IID_IBaseFilter,
                                       reinterpret_cast<void**>(&impl_->captureFilter));
    moniker->Release();
    if (FAILED(hr) || !impl_->captureFilter) {
        emit errorOccurred(QStringLiteral("\u7ed1\u5b9a\u91c7\u96c6\u8bbe\u5907\u5931\u8d25"));
        stop(); return false;
    }
    qWarning().noquote() << QStringLiteral("[DShow] start: 绑定设备成功");

    hr = CoCreateInstance(CLSID_FilterGraph, nullptr, CLSCTX_INPROC_SERVER,
                         IID_IGraphBuilder, reinterpret_cast<void**>(&impl_->graph));
    if (FAILED(hr)) { emit errorOccurred(QStringLiteral("\u521b\u5efaFilterGraph\u5931\u8d25")); stop(); return false; }
    qWarning().noquote() << QStringLiteral("[DShow] start: FilterGraph 创建成功");

    hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, nullptr, CLSCTX_INPROC_SERVER,
                         IID_ICaptureGraphBuilder2, reinterpret_cast<void**>(&impl_->builder));
    if (FAILED(hr)) { emit errorOccurred(QStringLiteral("\u521b\u5efaCaptureGraphBuilder2\u5931\u8d25")); stop(); return false; }
    impl_->builder->SetFiltergraph(impl_->graph);
    qWarning().noquote() << QStringLiteral("[DShow] start: CaptureGraphBuilder2 创建成功");

    hr = impl_->graph->AddFilter(impl_->captureFilter, L"Capture");
    if (FAILED(hr)) { emit errorOccurred(QStringLiteral("\u91c7\u96c6\u8bbe\u5907\u52a0\u5165\u56fe\u5931\u8d25")); stop(); return false; }
    qWarning().noquote() << QStringLiteral("[DShow] start: 采集 Filter 加入图成功");

    // ---- 协商格式：在采集输出针脚上通过 IAMStreamConfig 找 MJPG@目标 ----
    IAMStreamConfig* streamConfig = nullptr;
    qWarning().noquote() << QStringLiteral("[DShow] start: 查找 IAMStreamConfig...");
    hr = impl_->builder->FindInterface(&PIN_CATEGORY_CAPTURE, &MEDIATYPE_Video,
                                       impl_->captureFilter, IID_IAMStreamConfig,
                                       reinterpret_cast<void**>(&streamConfig));
    if (FAILED(hr) || !streamConfig) {
        emit errorOccurred(QStringLiteral("\u62ff\u4e0d\u5230IAMStreamConfig\uff0c\u65e0\u6cd5\u8bbe\u7f6e\u683c\u5f0f"));
        stop(); return false;
    }
    qWarning().noquote() << QStringLiteral("[DShow] start: IAMStreamConfig 获取成功");

    int capCount = 0, capSize = 0;
    streamConfig->GetNumberOfCapabilities(&capCount, &capSize);
    qWarning().noquote() << QStringLiteral("[DShow] start: 格式能力数=%1 capSize=%2")
        .arg(capCount).arg(capSize);

    // 在给定分辨率下，尝试把某个子类型@目标fps设成生效格式。成功返回true。
    auto trySetFormat = [&](const GUID& wantSub) -> bool {
        const char* subName = (wantSub == kMediaSubtypeNV12) ? "NV12"
                            : (wantSub == kMediaSubtypeYUY2) ? "YUY2"
                            : (wantSub == MEDIASUBTYPE_MJPG) ? "MJPG" : "?";
        qWarning().noquote() << QStringLiteral("[DShow] 尝试 %1 @ %2x%3...")
            .arg(QString::fromLatin1(subName)).arg(width).arg(height);
        for (int i = 0; i < capCount; ++i) {
            AM_MEDIA_TYPE* mt = nullptr;
            VIDEO_STREAM_CONFIG_CAPS caps;
            if (FAILED(streamConfig->GetStreamCaps(i, &mt, reinterpret_cast<BYTE*>(&caps))) || !mt)
                continue;
            bool ok = false;
            if (mt->formattype == FORMAT_VideoInfo && mt->pbFormat && mt->subtype == wantSub) {
                VIDEOINFOHEADER* vih = reinterpret_cast<VIDEOINFOHEADER*>(mt->pbFormat);
                const long w = vih->bmiHeader.biWidth;
                const long h = qAbs(vih->bmiHeader.biHeight);
                if (w == width && h == height) {
                    // 帧间隔单位100ns，用目标fps反推；跟驱动允许的最小帧间隔
                    // (即最高帧率)取更保守的，不强设一个驱动都不支持的间隔。
                    const REFERENCE_TIME wantInterval = REFERENCE_TIME(10000000.0 / fps);
                    const REFERENCE_TIME useInterval  = std::max(caps.MinFrameInterval, wantInterval);
                    vih->AvgTimePerFrame = useInterval;
                    if (SUCCEEDED(streamConfig->SetFormat(mt))) {
                        ok = true;
                        // 【把驱动实际接受的帧间隔读回来 —— 别再靠猜】
                        // SetFormat 返回 S_OK 只代表"没报错"，不代表驱动照单全收：
                        // UVC 驱动通常只支持一组离散帧间隔，会把请求量化到最近的
                        // 一档；有的干脆忽略 AvgTimePerFrame 只认分辨率。
                        // 不读回来的话，"选了15却跑30"到底是我们没下发、驱动拒绝、
                        // 还是量化到了别的档，三种完全不同的原因在日志里长得一样。
                        AM_MEDIA_TYPE* got = nullptr;
                        if (SUCCEEDED(streamConfig->GetFormat(&got)) && got) {
                            if (got->formattype == FORMAT_VideoInfo && got->pbFormat) {
                                const VIDEOINFOHEADER* gv =
                                    reinterpret_cast<const VIDEOINFOHEADER*>(got->pbFormat);
                                const REFERENCE_TIME acc = gv->AvgTimePerFrame;
                                qWarning().noquote() << QStringLiteral(
                                    "[DShow] 帧率协商：请求 %1fps(间隔%2) 下发%3 "
                                    "驱动回读%4 => 实际约 %5fps%6")
                                    .arg(fps, 0, 'f', 1)
                                    // REFERENCE_TIME 是 LONGLONG，显式转 qlonglong
                                    // 免得撞上 QString::arg 的整型重载歧义
                                    .arg(qlonglong(wantInterval))
                                    .arg(qlonglong(useInterval))
                                    .arg(qlonglong(acc))
                                    .arg(acc > 0 ? 1e7 / double(acc) : 0.0, 0, 'f', 1)
                                    .arg(acc != useInterval
                                             ? QStringLiteral("  ← 驱动没照单全收（量化到它支持的档）")
                                             : QString());
                            }
                            if (got->cbFormat && got->pbFormat) CoTaskMemFree(got->pbFormat);
                            if (got->pUnk) got->pUnk->Release();
                            CoTaskMemFree(got);
                        }
                    }
                }
            }
            if (mt->cbFormat != 0 && mt->pbFormat) CoTaskMemFree(mt->pbFormat);
            if (mt->pUnk) mt->pUnk->Release();
            CoTaskMemFree(mt);
            if (ok) return true;
        }
        return false;
    };

    // 【格式协商】按开关决定：preferUncompressed_ 时优先 NV12/YUY2(未压缩格式
    // 零解码直取 Y 平面，省掉"每秒解几百张 JPEG"的 CPU/发热大头)，协商不到再
    // 退回 MJPG；开关关掉则优先走老的 MJPG 压缩路径，但 MJPG/NV12 都没有时
    // 仍要试 YUY2——系统内置摄像头(例如 ASUS 5M WebCam)常只有 YUY2，不试这条
    // 就会整个协商失败、退回到 Qt/Media Foundation，区间内帧率请求被无声丢弃。
    qWarning().noquote() << QStringLiteral("[DShow] start: 开始格式协商");
    bool formatSet = false;
    bool useNv12 = false;
    bool useYuy2 = false;
    if (preferUncompressed_ && trySetFormat(kMediaSubtypeNV12)) { formatSet = true; useNv12 = true; }
    else if (preferUncompressed_ && trySetFormat(kMediaSubtypeYUY2)) { formatSet = true; useYuy2 = true; }
    else if (trySetFormat(MEDIASUBTYPE_MJPG)) { formatSet = true; }
    else if (trySetFormat(kMediaSubtypeNV12)) { formatSet = true; useNv12 = true; }
    else if (trySetFormat(kMediaSubtypeYUY2)) { formatSet = true; useYuy2 = true; }

    // 【多路缓冲区，缓解运动时突发丢帧】MJPG 是逐帧独立压缩，画面越复杂
    // (球在动、边缘更多)单帧体积会明显比静止画面大，是典型的突发(bursty)
    // 数据——而默认情况下 DirectShow 采集针脚的分配器往往只给 1~2 个缓冲区。
    // 一旦某一帧因为运动导致体积变大、USB传输或本进程处理稍微慢半拍，缓冲区
    // 不够用就会在驱动/采集这一层直接丢帧，且这种丢帧应用层完全看不到(不会
    // 触发上面 BufferCB 里新加的解码失败日志，因为它压根没送到回调)。这里
    // 通过 IAMBufferNegotiation 主动跟这颗针脚申请多几个缓冲区，给这种瞬时
    // 突发一点缓冲余地——纯软件层面的改动，不额外占用 USB 带宽(缓冲区只是
    // 内存，不是要传输更多字节)，失败也不影响主流程(退回驱动默认值)。
    {
        IAMBufferNegotiation* bufNeg = nullptr;
        if (SUCCEEDED(streamConfig->QueryInterface(IID_IAMBufferNegotiation,
                                                    reinterpret_cast<void**>(&bufNeg))) && bufNeg) {
            ALLOCATOR_PROPERTIES props;
            ZeroMemory(&props, sizeof(props));
            if (SUCCEEDED(bufNeg->GetAllocatorProperties(&props))) {
                // 只在驱动给的默认缓冲数偏小时才往上调，且封顶 8——不是越多
                // 越好，缓冲区本身要占内存(尤其未压缩 NV12 一帧就是完整
                // 分辨率大小)，4 路一起开太多反而增加内存压力。
                if (props.cBuffers < 8) {
                    props.cBuffers = 8;
                    bufNeg->SuggestAllocatorProperties(&props);   // 失败无所谓，尽力而为
                }
            }
            bufNeg->Release();
        }
    }

    streamConfig->Release();

    if (!formatSet) {
        emit errorOccurred(QStringLiteral(
            "\u6ca1\u627e\u5230\u5339\u914d\u7684 NV12/YUY2/MJPG \u683c\u5f0f(\u68c0\u67e5\u5206\u8fa8\u7387\u4e0e\u5e27\u7387\u662f\u5426\u771f\u7684\u652f\u6301\u8fd9\u4e2a\u7ec4\u5408)"));
        stop(); return false;
    }

    qWarning().noquote() << QStringLiteral("[DShow] start: 格式协商成功 useNv12=%1 useYuy2=%2")
        .arg(useNv12).arg(useYuy2);

    // 把协商结果告诉回调：NV12/YUY2 走零解码取灰度 Y 平面，需要知道分辨率；
    // MJPG 走软解。
    impl_->useNv12 = useNv12;
    impl_->useYuy2 = useYuy2;
    impl_->nv12W = width;
    impl_->nv12H = height;

    // ---- Sample Grabber ----
    hr = CoCreateInstance(kClsidSampleGrabber, nullptr, CLSCTX_INPROC_SERVER,
                         IID_IBaseFilter, reinterpret_cast<void**>(&impl_->grabberFilter));
    if (FAILED(hr) || !impl_->grabberFilter) {
        emit errorOccurred(QStringLiteral(
            "\u521b\u5efaSampleGrabber\u5931\u8d25\uff0c\u5f88\u53ef\u80fd\u662fqedit.dll\u6ca1\u6ce8\u518c"
            "\u2014\u2014\u4ee5\u7ba1\u7406\u5458\u8eab\u4efd\u6267\u884c\u4e00\u6b21\uff1aregsvr32 qedit.dll"));
        stop(); return false;
    }
    hr = impl_->grabberFilter->QueryInterface(kIidSampleGrabber,
                                              reinterpret_cast<void**>(&impl_->grabber));
    if (FAILED(hr) || !impl_->grabber) {
        emit errorOccurred(QStringLiteral("SampleGrabber\u63a5\u53e3\u67e5\u8be2\u5931\u8d25"));
        stop(); return false;
    }

    AM_MEDIA_TYPE grabType; ZeroMemory(&grabType, sizeof(grabType));
    grabType.majortype = MEDIATYPE_Video;
    grabType.subtype = useNv12 ? kMediaSubtypeNV12
                      : useYuy2 ? kMediaSubtypeYUY2
                                : MEDIASUBTYPE_MJPG;   // 跟上面协商生效的格式一致
    impl_->grabber->SetMediaType(&grabType);
    impl_->grabber->SetOneShot(FALSE);
    impl_->grabber->SetBufferSamples(FALSE);
    impl_->grabber->SetCallback(impl_, 1);   // 1 = 走 BufferCB

    hr = impl_->graph->AddFilter(impl_->grabberFilter, L"Grabber");
    if (FAILED(hr)) { emit errorOccurred(QStringLiteral("SampleGrabber\u52a0\u5165\u56fe\u5931\u8d25")); stop(); return false; }
    qWarning().noquote() << QStringLiteral("[DShow] start: SampleGrabber 加入图成功");

    // ---- Null Renderer：终结这条图，不需要真的弹窗口渲染画面 ----
    hr = CoCreateInstance(kClsidNullRenderer, nullptr, CLSCTX_INPROC_SERVER,
                         IID_IBaseFilter, reinterpret_cast<void**>(&impl_->nullRendererFilter));
    if (FAILED(hr) || !impl_->nullRendererFilter) {
        emit errorOccurred(QStringLiteral("\u521b\u5efaNullRenderer\u5931\u8d25"));
        stop(); return false;
    }
    hr = impl_->graph->AddFilter(impl_->nullRendererFilter, L"NullRenderer");
    if (FAILED(hr)) { emit errorOccurred(QStringLiteral("NullRenderer\u52a0\u5165\u56fe\u5931\u8d25")); stop(); return false; }

    // ---- 连线：采集 -> SampleGrabber -> NullRenderer，交给 RenderStream
    // 按前面已经设置好的格式自动协商连接，不手动摸每个针脚。----
    hr = impl_->builder->RenderStream(&PIN_CATEGORY_CAPTURE, &MEDIATYPE_Video,
                                      impl_->captureFilter, impl_->grabberFilter,
                                      impl_->nullRendererFilter);
    if (FAILED(hr)) {
        emit errorOccurred(QStringLiteral(
            "\u91c7\u96c6\u56fe\u8fde\u7ebf\u5931\u8d25(RenderStream)\uff0c\u53ef\u80fd\u662f\u8fd9\u9891"
            "\u76f8\u673a\u5728\u8fd9\u4e2a\u5206\u8fa8\u7387/\u5e27\u7387\u7ec4\u5408\u4e0b\u4e0d\u652f\u6301"
            "SampleGrabber\u8981\u6c42\u7684\u4e2d\u95f4\u8f6c\u6362"));
        stop(); return false;
    }

    hr = impl_->graph->QueryInterface(IID_IMediaControl,
                                      reinterpret_cast<void**>(&impl_->mediaControl));
    if (FAILED(hr) || !impl_->mediaControl) {
        emit errorOccurred(QStringLiteral("\u62ff\u4e0d\u5230IMediaControl"));
        stop(); return false;
    }

    impl_->fpsClock.start();
    impl_->fpsWindowStartMs = 0;
    impl_->frameCount = 0;

    hr = impl_->mediaControl->Run();
    if (FAILED(hr)) {
        emit errorOccurred(QStringLiteral("\u542f\u52a8\u91c7\u96c6\u6d41\u5931\u8d25(Run)"));
        stop(); return false;
    }

    // 图已经在跑了——把"本图流时间的零点"锚定到共享时钟上，之后 BufferCB
    // 里每帧靠它换算出跨相机可比的 ts_ns(见 sharedClock() 注释)。必须在
    // Run() 之后记：Run() 之前图还没起来，零点无从谈起。
    impl_->graphStartNs = sharedClock().nsecsElapsed();
    qWarning().noquote() << QStringLiteral("[DShow] start: 图已 Run，开始出帧");

    running_ = true;
    return true;
}

void DShowCapture::stop() {
    if (impl_) {
        qWarning().noquote() << QStringLiteral("[DShow] stop 进入");
        if (impl_->mediaControl) {
            qWarning().noquote() << QStringLiteral("[DShow] stop: mediaControl->Stop 前");
            impl_->mediaControl->Stop();
            qWarning().noquote() << QStringLiteral("[DShow] stop: mediaControl->Stop 后");
            impl_->mediaControl->Release();
        }
        qWarning().noquote() << QStringLiteral("[DShow] stop: 释放 grabber 前");
        if (impl_->grabber)            impl_->grabber->Release();
        if (impl_->nullRendererFilter) impl_->nullRendererFilter->Release();
        if (impl_->grabberFilter)      impl_->grabberFilter->Release();
        if (impl_->captureFilter)      impl_->captureFilter->Release();
        if (impl_->builder)            impl_->builder->Release();
        qWarning().noquote() << QStringLiteral("[DShow] stop: 释放 graph 前");
        if (impl_->graph)              impl_->graph->Release();
        if (impl_->comInit)            CoUninitialize();
        delete impl_;
        impl_ = nullptr;
        qWarning().noquote() << QStringLiteral("[DShow] stop 完成");
    }
    running_ = false;
}

} // namespace mocap

#else
// --------------------------------------------------------------------------
// 非 Windows 桩：这套原型只服务于"验证Windows上绕开Qt/MF能不能解决帧率
// 问题"这一件事，非Windows平台没有这个问题(没有Media Foundation/Frame
// Server这层)，直接返回失败即可。
// --------------------------------------------------------------------------
namespace mocap {
DShowCapture::DShowCapture(QObject* parent) : QObject(parent) {}
DShowCapture::~DShowCapture() {}
bool DShowCapture::start(const QByteArray&, const QString&, int, int, int, double) {
    emit errorOccurred(QStringLiteral("DShowCapture\u4ec5\u652f\u6301Windows"));
    return false;
}
void DShowCapture::stop() {}
} // namespace mocap
#endif
