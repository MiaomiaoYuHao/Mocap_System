#include "camera/WebcamCamera.hpp"
#include "camera/DShowCapture.hpp"
#include "camera/JpegDecoder.hpp"
#include "camera/CameraLabel.hpp"   // makeCameraLabel()：见构造函数那段说明
#include "settings/AppSettings.hpp" // cameraNumber()：界面上那个稳定 #N
#include <QCamera>
#include <QVideoSink>
#include <QVideoFrame>
#include <QVideoFrameFormat>
#include <QDebug>
#include <QThread>
#include <QMediaCaptureSession>
#include <QMediaDevices>
#include <QElapsedTimer>
#include <QMetaObject>
#include <QTimer>
#include <algorithm>
#include <cstring>
#include <vector>
#include <atomic>
#include <mutex>

namespace mocap {

// isCompressedFormat() 现在在 WebcamCamera.hpp 里(inline，供 CamParamDialog.cpp
// 复用同一套判断标准)，这里不再重复定义。

static QCameraFormat pickBestFormat(const QList<QCameraFormat>& fmts) {
    QCameraFormat best;
    for (const QCameraFormat& f : fmts) {
        if (best.isNull()) { best = f; continue; }
        const bool fComp = isCompressedFormat(f.pixelFormat());
        const bool bComp = isCompressedFormat(best.pixelFormat());
        if (fComp != bComp) {
            if (fComp) best = f;
            continue;
        }
        const float fh = f.maxFrameRate(), bh = best.maxFrameRate();
        if (fh > bh + 0.5f) { best = f; continue; }
        if (std::abs(fh - bh) <= 0.5f) {
            const qint64 fa = qint64(f.resolution().width()) * f.resolution().height();
            const qint64 ba = qint64(best.resolution().width()) * best.resolution().height();
            if (fa > ba) best = f;
        }
    }
    return best;
}

class CaptureWorker : public QObject {
    Q_OBJECT
public:
    // nameOccurrenceIndex：跟 WebcamCamera::nameOccurrenceIndex() 同一个值，
    // 由外层(WebcamCamera::start())算好传进来——DShowCapture 绑定同名设备
    // 时需要这个数，CaptureWorker 自己算一遍容易跟外层用的不是同一份逻辑
    // (虽然现在实现上一致，但两处各算一遍本身就是隐患，直接传值更稳)。
    CaptureWorker(quint32 id, const QCameraDevice& dev, int nameOccurrenceIndex)
        : id_(id), dev_(dev), nameOccurrenceIndex_(nameOccurrenceIndex) {}

    std::atomic<bool> rawEnabled_{false};
    std::atomic<bool> detectOn_{false};
    std::atomic<bool> changingFormat_{false};
    std::atomic<bool> grayOut_{false};
    std::atomic<int>  previewMode_{0};
    std::atomic<int>  previewThr_{60};

    // 【整合DirectShow采集】usingDshow_ 时没有活的 QCamera 对象，这里返回
    // 我们主动决定并请求 DirectShow 使用的那个格式(lastAppliedFormat_)，
    // 不是一次实时的硬件回读——这跟走 QCamera 那条路时"读 camera_->
    // cameraFormat()"的verified程度不完全一样，是有意的取舍：AMCap已经
    // 验证过 MJPG@这个分辨率@这个帧率 在这颗相机上是真实生效的，见
    // DShowCapture.hpp 顶部注释里的完整排查记录。
    QCameraFormat currentFormat() const {
        if (usingDshow_) return lastAppliedFormat_;
        return camera_ ? camera_->cameraFormat() : QCameraFormat();
    }

public slots:
    void startCapture() {
        if (camera_ || dshow_) return;
        startWithFormat(bestFormat(), 0.0);   // 自动选档，按标称上限跑
    }

    void stopCapture() {
        if (dshow_) { dshow_->stop(); delete dshow_; dshow_ = nullptr; }
        usingDshow_ = false;

        if (pollTimer_) { pollTimer_->stop(); delete pollTimer_; pollTimer_ = nullptr; }
        if (camera_) {
            camera_->stop();
            session_.setCamera(nullptr);
            session_.setVideoSink(nullptr);
            delete camera_; camera_ = nullptr;
        }
        delete sink_; sink_ = nullptr;
    }

    void applyFormat(QCameraFormat f, double targetFps) {
        if (f.isNull()) return;
        changingFormat_ = true;
        qWarning().noquote() << QStringLiteral("[cam %1] applyFormat: stopCapture 前").arg(id_);
        stopCapture();
        qWarning().noquote() << QStringLiteral("[cam %1] applyFormat: stopCapture 后，startWithFormat 前").arg(id_);
        startWithFormat(f, targetFps);
        qWarning().noquote() << QStringLiteral("[cam %1] applyFormat: startWithFormat 后").arg(id_);
        changingFormat_ = false;
    }

    // 这条格式最终要跑的帧率：指定过就用指定值，否则标称上限。
    // 【必须保证 > 0】DShowCapture::start() 里是 10000000.0 / fps，
    // fps 为 0 会算出 inf，再转成 REFERENCE_TIME(int64) 是未定义行为。
    // 驱动确实可能上报 maxFrameRate == 0（CamParamDialog 那边填帧率下拉框时
    // 就写着 if (hi > 0)，说明遇到过）。这个洞在改动之前就在，只是那时这里
    // 直接写 target.maxFrameRate()，同样会漏过去 —— 既然动到这一行就一并堵上。
    double effectiveFps(const QCameraFormat& f) const {
        if (lastTargetFps_ > 0.0) return lastTargetFps_;
        const double nominal = double(f.maxFrameRate());
        return nominal > 0.0 ? nominal : 30.0;   // 兜底值，只为不把 0 送进除法
    }
    double requestedFps() const { return lastTargetFps_; }

    QCameraFormat bestFormat() const {
        return pickBestFormat(dev_.videoFormats());
    }

signals:
    void frameReady(quint32 camId, const QImage& img, double fps);
    void rawFrameReady(quint32 camId, const QImage& gray, qint64 ts_ns);
    void frameForDetect(quint32 camId, const QImage& img, qint64 ts_ns);
    // 【采集引擎每次(重)起来都发一次】startCapture() 和 applyFormat() 都会走到
    // startWithFormat()，而它是 new QCamera / new DShowCapture —— 也就是把设备
    // 重新打开一遍。多数 UVC 驱动在最后一个句柄关闭时把 ProcAmp/CameraControl
    // 复位回默认，所以【任何在这之前推下去的硬件参数都会被抹掉】。
    // 上层拿这个信号在"设备确实开好了"之后再补推一次保存的参数。
    void captureStarted();

private:
    // 【引擎选择】压缩(MJPG)格式在这颗相机上被反复验证过：Qt/Media
    // Foundation这条软件路径会把硬件真实支持的120fps拖到60多；同一格式
    // 绕开Qt直接走DirectShow能跑满(AMCap验证过、DShowCapture原型也验证
    // 过)。所以压缩格式优先尝试DirectShow引擎，其余情况(未压缩格式、
    // 非Windows平台、或DirectShow初始化失败——比如qedit.dll没注册)维持
    // 走原来的QCamera路径，不是所有场景都强推新引擎。
    void startWithFormat(const QCameraFormat& target, double targetFps) {
        lastAppliedFormat_ = target;
        // 【0 表示"没指定"，回落到标称上限】区间外的值一律不信：驱动上报的
        // [min,max] 之外强设帧间隔，SetFormat 要么失败要么给个我们没预期的档。
        lastTargetFps_ = (targetFps > 0.0 && formatCoversRate(target, targetFps))
                             ? targetFps : 0.0;
        qWarning().noquote() << QStringLiteral(
                                    "[cam %1] 请求格式 %2x%3@%4fps pixelFormat=%5 (compressed=%6) "
                                    "驱动区间=%7~%8fps %9")
                                    .arg(id_).arg(target.resolution().width()).arg(target.resolution().height())
                                    .arg(effectiveFps(target)).arg(int(target.pixelFormat()))
                                    .arg(isCompressedFormat(target.pixelFormat()) ? "yes" : "no")
                                    .arg(target.minFrameRate(), 0, 'f', 0)
                                    .arg(target.maxFrameRate(), 0, 'f', 0)
                                    .arg(lastTargetFps_ > 0.0
                                             ? QStringLiteral("显式目标=%1fps -> 优先DirectShow")
                                                   .arg(lastTargetFps_, 0, 'f', 0)
                                             : QStringLiteral("按标称上限"));

#ifdef _WIN32
        // 【什么时候走 DirectShow】两种情况：
        //   ① 压缩格式 —— 原有理由：Qt/Media Foundation 这条软件路径会把硬件
        //      真实支持的 120fps 拖到 60 多，绕开它直接走 DirectShow 能跑满。
        //   ② 用户指定了区间内帧率 —— 【只有 DirectShow 兑现得了】。
        //      QCameraFormat 是 (分辨率, 像素格式, min, max) 描述符，表达不了
        //      "在区间内取 15"，Qt 路径会按标称上限跑，请求被无声丢弃。
        //      DirectShow 这边是 IAMStreamConfig 直接设 AvgTimePerFrame。
        //
        // 【为什么②不限制像素格式】DShowCapture 的格式协商本来就同时试 NV12 和
        // MJPG（NV12 还是零解码路径），并不局限于压缩格式 —— 原来这里卡在
        // isCompressedFormat 上，纯粹是当初只为①写的。笔记本自带摄像头选中的
        // 常是 YUY2/NV12，于是"改 15fps 没反应"：请求连能兑现它的那条路都没走到。
        //
        // 协商不到就 return false，照旧退回 Qt，行为不比现在差。
        const bool needRateControl = lastTargetFps_ > 0.0;
        if (!target.isNull() && (isCompressedFormat(target.pixelFormat()) || needRateControl)) {
            if (startDshow(target)) { emit captureStarted(); return; }
            qWarning().noquote() << QStringLiteral(
                "[cam %1] DirectShow采集引擎启动失败，回退到Qt/Media Foundation路径"
                "%2").arg(id_).arg(needRateControl
                    ? QStringLiteral("(注意：指定的 %1fps 在 Qt 路径上无法兑现，"
                                     "实际会按标称 %2fps 跑)")
                          .arg(lastTargetFps_, 0, 'f', 0).arg(target.maxFrameRate(), 0, 'f', 0)
                    : QStringLiteral("(压缩格式下这条路径实测帧率可能明显偏低，"
                                     "见系统标定手册排查记录)"));
        }
#endif
        startQtCamera(target);
        // 【两条引擎路径都要发】不管走 DirectShow 还是 Qt，设备都被重新打开了，
        // 上层要补推的硬件参数一样多。漏发一条 = 那条路径上参数静默丢失。
        emit captureStarted();
    }

    bool startDshow(const QCameraFormat& target) {
        dshow_ = new DShowCapture(this);
        connect(dshow_, &DShowCapture::frameReady, this, [this](QImage frame, qint64 ts_ns) {
            deliverDecodedFrame(frame, ts_ns, lastAppliedFormat_.resolution());
        });
        connect(dshow_, &DShowCapture::measuredFpsChanged, this, [this](double fps) {
            fps_ = fps;
        });
        connect(dshow_, &DShowCapture::errorOccurred, this, [this](QString msg) {
            qWarning().noquote() << QStringLiteral("[cam %1] DirectShow采集错误: %2").arg(id_).arg(msg);
        });

        clock_.start(); fpsWindowStartMs_ = 0; fpsFrameCount_ = 0; fps_ = 0; lastPreviewMs_ = 0;

        // 【这里原来写死 target.maxFrameRate()，是"选了15还是跑30"的正主】
        // DShowCapture::start 的 fps 参数一路通到 AvgTimePerFrame = 1e7/fps，
        // 能力一直都在，只是用户请求的那个数从来没传进来。
        const bool ok = dshow_->start(dev_.id(), dev_.description(), nameOccurrenceIndex_,
                                      target.resolution().width(), target.resolution().height(),
                                      effectiveFps(target),
                                      lastTargetFps_ > 0.0);   // 只有区间内显式指定才启用抽帧兜底
        if (!ok) {
            delete dshow_; dshow_ = nullptr;
            return false;
        }
        usingDshow_ = true;
        return true;
    }

    void startQtCamera(const QCameraFormat& target) {
        camera_ = new QCamera(dev_);
        sink_   = new QVideoSink;
        session_.setCamera(camera_);
        session_.setVideoSink(sink_);
        if (!target.isNull()) camera_->setCameraFormat(target);

        lastPolledStartUs_ = -1;
        pollTimer_ = new QTimer(this);
        pollTimer_->setTimerType(Qt::PreciseTimer);
        connect(pollTimer_, &QTimer::timeout, this, [this] {
            if (sink_) onFrame(sink_->videoFrame());
        });
        pollTimer_->start(4);

        clock_.start(); fpsWindowStartMs_ = 0; fpsFrameCount_ = 0; fps_ = 0; lastPreviewMs_ = 0;
        camera_->start();
    }

    // 【共用】不管这一帧是从 QCamera(onFrame里解码) 还是从 DShowCapture
    // (自己的DirectShow采集回调里解码)拿到的，"这一帧该发给谁"这套判断
    // (原始流/检测/预览节流+残帧校验)完全一样，抽成一个方法两条引擎共用，
    // 不写两份容易走样的重复逻辑。
    void deliverDecodedFrame(const QImage& frame, qint64 ts_ns, const QSize& expectSize) {
        if (frame.isNull()) return;

        if (expectSize.isValid() && frame.size() != expectSize) {
            ++corruptFrames_;
            if (corruptFrames_ % 30 == 1)
                qWarning().noquote() << QStringLiteral("[cam %1] 丢弃残缺帧(尺寸%2x%3≠期望%4x%5) 累计%6")
                                            .arg(id_).arg(frame.width()).arg(frame.height())
                                            .arg(expectSize.width()).arg(expectSize.height()).arg(corruptFrames_);
            return;
        }

        const bool wantRaw = rawEnabled_;
        const bool wantDetect = detectOn_ && detectBusy_ && !detectBusy_->load();
        const qint64 nowMs = clock_.elapsed();
        const bool wantPreview = (nowMs - lastPreviewMs_ >= 33);
        if (!wantRaw && !wantDetect && !wantPreview) return;

        // 【去重转换】raw / detect(下游DetectWorker自己也会转) / 预览(灰度开关开着
        // 时的画面档、以及固定要灰度的阈值掩膜档)最多会在同一帧上各自独立调一次
        // convertToFormat(Grayscale8)——同一帧最坏情况被转 3 次，纯粹重复劳动。
        // 这里对同一帧只转一次、缓存下来，后面各处都拿这份缓存，而不是各转各的。
        // 已经是灰度的帧(比如 NV12/YUY2 零解码路径)直接用原帧，一次转换都不需要。
        QImage grayCache;
        bool haveGray = (frame.format() == QImage::Format_Grayscale8);
        if (haveGray) grayCache = frame;
        auto grayOf = [&](const QImage& src) -> const QImage& {
            if (!haveGray) {
                grayCache = src.convertToFormat(QImage::Format_Grayscale8);
                haveGray = true;
            }
            return grayCache;
        };

        if (wantRaw) {
            if (grayOut_ && frame.format() != QImage::Format_Grayscale8)
                emit rawFrameReady(id_, grayOf(frame), ts_ns);
            else
                emit rawFrameReady(id_, frame, ts_ns);
        }

        if (wantDetect) {
            // grayOut_ 打开时直接把(可能已经缓存好的)灰度帧发给检测线程，
            // DetectWorker::packGray() 一看格式已经是 Grayscale8 就会跳过它
            // 自己那次转换——避免同一帧在两个线程里各转一遍。
            if (grayOut_)
                emit frameForDetect(id_, grayOf(frame), ts_ns);
            else
                emit frameForDetect(id_, frame, ts_ns);
        }

        if (!wantPreview) return;
        lastPreviewMs_ = nowMs;

        // 预览模式现在只有 2 档：0=画面(彩/灰完全由上面的 grayOut_ 总开关决定，
        // 不再单独判断)、1=阈值掩膜(检测器的二值视界，固定基于灰度阈值化，
        // 跟 grayOut_ 开关无关——掩膜本来就得先转灰度才能二值化)。
        // 原来还有一档"算法输入"，是 grayOut_ 这个总开关统一到全部消费者之前
        // 遗留的产物：那时候只有这一档会看 grayOut_，"原画面"档不看，所以需要
        // 两档来区分。现在 grayOut_ 对画面档统一生效，两档变得完全等价，属于
        // 死选项，删掉。
        const int pm = previewMode_.load();
        QImage view = frame;
        if (pm == 1) {
            const QImage& g = grayOf(frame);
            const int thr = previewThr_.load();
            QImage m(g.width(), g.height(), QImage::Format_Grayscale8);
            for (int y = 0; y < g.height(); ++y) {
                const uchar* s = g.constScanLine(y);
                uchar* d = m.scanLine(y);
                for (int x = 0; x < g.width(); ++x) d[x] = s[x] >= thr ? 255 : 0;
            }
            view = m;
        } else if (grayOut_) {
            view = grayOf(frame);
        }
        emit frameReady(id_, view, fps_);
    }

private slots:
    void onFrame(const QVideoFrame& f) {
        if (changingFormat_ || !f.isValid()) return;

        const qint64 startUs = f.startTime();
        if (startUs >= 0 && startUs == lastPolledStartUs_) return;
        lastPolledStartUs_ = startUs;

        ++fpsFrameCount_;
        const qint64 nowMsForFps = clock_.elapsed();
        const qint64 winMs = nowMsForFps - fpsWindowStartMs_;
        if (winMs >= 500) {
            fps_ = fpsFrameCount_ * 1000.0 / winMs;
            fpsFrameCount_ = 0;
            fpsWindowStartMs_ = nowMsForFps;
        }

        // 【性能关键】f.toImage()是这个函数里最贵的一步(尤其压缩格式，
        // 要做真正的JPEG解码)——以前不管这一帧最终用不用得上，先解码了
        // 再说，预览节流(~33ms一次)是解码之后才生效的。但轮询定时器
        // (pollTimer_，4ms一次)和这里的解码是同一个线程：解码耗时一旦
        // 超过4ms(1280x720软件JPEG解码很容易超过)，下一次真正的轮询就会
        // 被这次解码拖后，相当于用解码速度反过来限制了"轮询跟得上的帧率"，
        // 即使传感器/驱动本身在真实吐120fps。这里先判断这一帧到底会不会
        // 被任何消费者用上(原始流/检测/预览节流窗口)，都用不上就直接跳过
        // 解码，把这部分CPU时间还给轮询定时器，让它跟得上真实到达节奏。
        //
        // 【注意】即便做了这个优化，压缩格式在这条Qt/Media Foundation路径
        // 上实测仍然明显低于硬件真实上限(约63fps对应120fps硬件能力)——
        // 这次排查已确认瓶颈不在这个解码步骤本身，而在更早的Media
        // Foundation/Frame Server管线内部(完整排查过程见系统标定手册)。
        // 这正是压缩格式现在优先走 startDshow() 那条路的原因；这条
        // onFrame() 路径只服务未压缩格式或DirectShow引擎不可用时的兜底。
        const bool wantRaw = rawEnabled_;
        const bool wantDetect = detectOn_ && detectBusy_ && !detectBusy_->load();
        const qint64 nowMs = clock_.elapsed();
        const bool wantPreview = (nowMs - lastPreviewMs_ >= 33);
        if (!wantRaw && !wantDetect && !wantPreview) return;

        qint64 ts_ns = f.startTime() >= 0 ? f.startTime() * 1000 : clock_.nsecsElapsed();

        QImage frame = f.toImage();
        // 灰度直解开关打开时，这条 Qt 兜底路径也要保证输出灰度(和主用的
        // DShowCapture 路径行为一致)。注意 turbo 灰度直解只在 DShowCapture 那条
        // 主路径上生效——那才是压缩格式实际走的路；这条 onFrame() 只服务未压缩
        // 或 DirectShow 不可用时的兜底，量很小，这里补一次 convertToFormat 即可，
        // 不值得为它引入 QVideoFrame::map 拆原始 JPEG 字节的复杂度。
        if (isGrayDecode() && !frame.isNull()
            && frame.format() != QImage::Format_Grayscale8) {
            frame = frame.convertToFormat(QImage::Format_Grayscale8);
        }
        const QSize expect = camera_ ? camera_->cameraFormat().resolution() : QSize();
        deliverDecodedFrame(frame, ts_ns, expect);
    }

private:
    quint32 id_;
    QCameraDevice dev_;
    int nameOccurrenceIndex_ = 0;

    QCamera* camera_ = nullptr;
    QVideoSink* sink_ = nullptr;
    QMediaCaptureSession session_;
    QTimer* pollTimer_ = nullptr;
    qint64  lastPolledStartUs_ = -1;

    DShowCapture* dshow_ = nullptr;
    bool usingDshow_ = false;
    QCameraFormat lastAppliedFormat_;
    double        lastTargetFps_ = 0.0;   // 区间内指定的帧率，0=按标称上限

    QElapsedTimer clock_;
    qint64 fpsWindowStartMs_ = 0;
    int    fpsFrameCount_ = 0;
    double fps_ = 0;
    qint64 lastPreviewMs_ = 0;
    quint64 corruptFrames_ = 0;

public:
    std::atomic<bool>* detectBusy_ = nullptr;
};

// 【显示名在这里定，是因为全 UI 都从 ICamera::name() 取名字】
// 预览宫格(CameraGrid)、捕捉窗口相机列表(PointCloudTestDialog_panel)、
// 标定对话框与标定向导(CalibrationDialog / CalibWizard)、同步监视器、
// 参数对话框、重映射对话框 —— 查过一遍，全部落到 c->name()。
// 而这里原来传的是 dev.description()，也就是 USB 描述符里的 iProduct：
// 六台同型号相机（VID_0EDE/PID_2076）厂商没写序列号，六个 iProduct 一字
// 不差，所以界面上必然是六行一模一样的 "USB Camera"。换任何枚举方式都一样，
// 那不是 bug，是设备本身没提供可区分的信息（见 CameraLabel.hpp 开头）。
//
// 【区分靠编号，不靠端口串】AppSettings::cameraNumber() 按 DevicePath 派生的
// 稳定 tag 持久化一份 tag -> #N 的表，首次见到分配、此后永远是这个号：
// 重启不变、枚举顺序变了不变、拖拽换位不变。添加相机对话框查的是同一张表，
// 所以"对话框里选的 #3"和"加完之后列表里的 #3"必然是同一台。
//
// 【为什么不再拼 [Port_#0002.Hub_#0004]】七台相机时每行长四十来个字符，
// 标定向导底部那条张数汇总直接溢出窗口右边，而用户要的只是"几号机位"。
// 位置串留给相机重映射对话框 —— 那里才是回答"这个号现在是哪台物理相机"的地方。
WebcamCamera::WebcamCamera(quint32 id, const QCameraDevice& dev, QObject* parent)
    : ICamera(id, makeCameraLabel(dev.description(), dev.id(),
                                  AppSettings().cameraNumber(dev.id())).display, parent),
      dev_(dev) {}

WebcamCamera::~WebcamCamera() { stop(); }

QCameraFormat WebcamCamera::bestFormat() const {
    return pickBestFormat(dev_.videoFormats());
}

bool WebcamCamera::start() {
    if (thread_) return true;

    qRegisterMetaType<QCameraFormat>("QCameraFormat");

    worker_ = new CaptureWorker(id_, dev_, nameOccurrenceIndex());
    thread_ = new QThread;
    worker_->moveToThread(thread_);

    worker_->rawEnabled_ = rawEnabled_.load();
    worker_->detectOn_   = detectOn_.load();
    worker_->grayOut_    = grayOut_.load();
    worker_->previewMode_ = previewMode_.load();
    { std::lock_guard<std::mutex> lk(dparamsMtx_); worker_->previewThr_ = dparams_.threshold; }

    connect(worker_, &CaptureWorker::frameReady,    this, &ICamera::frameReady);
    connect(worker_, &CaptureWorker::rawFrameReady, this, &ICamera::rawFrameReady);
    // 【跨线程，自动排队】worker 在采集线程里发，这里在主线程收 ——
    // 接收方（CameraManager）要构造 DShowControl 推参数，那必须在主线程做。
    connect(worker_, &CaptureWorker::captureStarted, this,
            [this] { emit captureEngineStarted(id_); });

    detectWorker_ = new DetectWorker;
    detectThread_ = new QThread;
    detectWorker_->moveToThread(detectThread_);
    worker_->detectBusy_ = &detectWorker_->busy;
    { std::lock_guard<std::mutex> lk(dparamsMtx_); detectWorker_->setParams(dparams_); }
    detectWorker_->setContourCollectionEnabled(contourOn_.load());   // 同步启动前设置的状态
    connect(worker_, &CaptureWorker::frameForDetect, detectWorker_, &DetectWorker::process);
    connect(detectWorker_, &DetectWorker::blobsReady, this, &ICamera::blobsReady);
    connect(detectWorker_, &DetectWorker::blobDetailsReady, this, &ICamera::blobDetailsReady);
    detectThread_->start();

    connect(thread_, &QThread::started, worker_, &CaptureWorker::startCapture);
    thread_->start();
    return true;
}

void WebcamCamera::stop() {
    if (!thread_) return;
    QMetaObject::invokeMethod(worker_, "stopCapture", Qt::BlockingQueuedConnection);
    thread_->quit();
    thread_->wait();
    delete worker_; worker_ = nullptr;
    delete thread_; thread_ = nullptr;

    if (detectThread_) {
        detectThread_->quit();
        detectThread_->wait();
        delete detectWorker_; detectWorker_ = nullptr;
        delete detectThread_; detectThread_ = nullptr;
    }
}

QString WebcamCamera::deviceKey() const {
    return QStringLiteral("web:") + QString::fromLatin1(dev_.id());
}

int WebcamCamera::nameOccurrenceIndex() const {
    const QString myId   = QString::fromUtf8(dev_.id());
    const QString myName = dev_.description();
    int idx = 0;
    for (const QCameraDevice& d : QMediaDevices::videoInputs()) {
        if (d.description() != myName) continue;
        if (QString::fromUtf8(d.id()) == myId) break;
        ++idx;
    }
    return idx;
}

QCameraFormat WebcamCamera::currentFormat() const {
    return worker_ ? worker_->currentFormat() : QCameraFormat();
}

bool WebcamCamera::setFormat(const QCameraFormat& f, double targetFps) {
    if (!worker_ || f.isNull()) return false;
    // 【主线程这份也要记】worker_ 里那份在采集线程上，参数面板/存盘在主线程读，
    // 跨线程读一个非原子的 double 是未定义行为。这里各存各的，值由同一次调用给定。
    requestedFps_ = (targetFps > 0.0 && formatCoversRate(f, targetFps)) ? targetFps : 0.0;
    QMetaObject::invokeMethod(worker_, "applyFormat", Qt::QueuedConnection,
                              Q_ARG(QCameraFormat, f), Q_ARG(double, targetFps));
    return true;
}

void WebcamCamera::setRawFrameEnabled(bool on) {
    rawEnabled_ = on;
    if (worker_) worker_->rawEnabled_ = on;
}

void WebcamCamera::setDetectEnabled(bool on) {
    detectOn_ = on;
    if (worker_) worker_->detectOn_ = on;
    if (!on) emit blobsReady(id_, {}, 0);
}

void WebcamCamera::setDetectParams(const DetectParams& p) {
    { std::lock_guard<std::mutex> lk(dparamsMtx_); dparams_ = p; }
    if (detectWorker_) detectWorker_->setParams(p);
    if (worker_) worker_->previewThr_ = p.threshold;
}

void WebcamCamera::setGrayOutput(bool on) {
    grayOut_ = on;
    if (worker_) worker_->grayOut_ = on;
}

void WebcamCamera::setPreviewMode(int m) {
    previewMode_ = m;
    if (worker_) worker_->previewMode_ = m;
}

void WebcamCamera::setContourCollectionEnabled(bool on) {
    contourOn_ = on;
    if (detectWorker_) detectWorker_->setContourCollectionEnabled(on);
}

} // namespace mocap

#include "WebcamCamera.moc"
