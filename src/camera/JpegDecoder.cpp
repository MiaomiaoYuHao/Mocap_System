#include "camera/JpegDecoder.hpp"
#include <QElapsedTimer>

#ifdef HAVE_TURBOJPEG
#include <turbojpeg.h>
#include <QThreadStorage>
#endif

namespace mocap {

// ---------------------------------------------------------------------------
// 【基准统计的实现】只用原子加法，不加锁——decodeMjpeg 在各相机自己的采集
// 线程上并发调用，原子 fetch_add 是几纳秒的事，跟一次 JPEG 解码(毫秒级)比
// 完全可以忽略，不会成为新的性能瓶颈。
// 用"总耗时(ns)+总帧数"而不是滑动平均，是故意的：resetDecodeStats() 一清零
// 就是全新的时间窗口，UI 每次读快照时用 总耗时/总帧数 现算平均值，逻辑最简单
// 也最不会算错。
//
// 【只统计成功样本——这是修过一次的地方】最初版本不管解码成功没成功、耗时
// 都记进平均值。这在数据本身干净时问题不大，但遇到 USB 传输截断导致大量
// 坏帧(见 DShowCapture 那边的丢帧日志)时就会出问题：turbo 遇到截断数据往往
// 很快就失败返回，这个"没真正解码完"的耗时被当正常样本混进平均值，会把
// 平均耗时不合理地拉低、失真放大"灰度直解比彩色快多少倍"这个数字——用一批
// 混着失败样本的数据去回答"解码到底快多少"，答案是不可信的。现在改成：
// 只有解码真正成功才计入耗时统计，失败单独计数，UI 上分开展示，不再混在
// 一起拉低/拉高平均值。
// ---------------------------------------------------------------------------
namespace {
std::atomic<quint64> g_turboGrayFrames{0};
std::atomic<quint64> g_turboGrayNs{0};
std::atomic<quint64> g_turboGrayFails{0};
std::atomic<quint64> g_qtColorFrames{0};
std::atomic<quint64> g_qtColorNs{0};
std::atomic<quint64> g_qtColorFails{0};

inline void recordTurboGray(qint64 ns, bool ok) {
    if (ok) {
        g_turboGrayFrames.fetch_add(1, std::memory_order_relaxed);
        g_turboGrayNs.fetch_add(quint64(ns), std::memory_order_relaxed);
    } else {
        g_turboGrayFails.fetch_add(1, std::memory_order_relaxed);
    }
}
inline void recordQtColor(qint64 ns, bool ok) {
    if (ok) {
        g_qtColorFrames.fetch_add(1, std::memory_order_relaxed);
        g_qtColorNs.fetch_add(quint64(ns), std::memory_order_relaxed);
    } else {
        g_qtColorFails.fetch_add(1, std::memory_order_relaxed);
    }
}
} // namespace

DecodeStatsSnapshot decodeStatsSnapshot() {
    DecodeStatsSnapshot s;
    s.turboGrayFrames = g_turboGrayFrames.load(std::memory_order_relaxed);
    const quint64 tgNs = g_turboGrayNs.load(std::memory_order_relaxed);
    s.turboGrayAvgMs = s.turboGrayFrames ? double(tgNs) / double(s.turboGrayFrames) / 1e6 : 0.0;
    s.turboGrayFails = g_turboGrayFails.load(std::memory_order_relaxed);

    s.qtColorFrames = g_qtColorFrames.load(std::memory_order_relaxed);
    const quint64 qcNs = g_qtColorNs.load(std::memory_order_relaxed);
    s.qtColorAvgMs = s.qtColorFrames ? double(qcNs) / double(s.qtColorFrames) / 1e6 : 0.0;
    s.qtColorFails = g_qtColorFails.load(std::memory_order_relaxed);
    return s;
}

void resetDecodeStats() {
    g_turboGrayFrames.store(0, std::memory_order_relaxed);
    g_turboGrayNs.store(0, std::memory_order_relaxed);
    g_turboGrayFails.store(0, std::memory_order_relaxed);
    g_qtColorFrames.store(0, std::memory_order_relaxed);
    g_qtColorNs.store(0, std::memory_order_relaxed);
    g_qtColorFails.store(0, std::memory_order_relaxed);
}

#ifdef HAVE_TURBOJPEG

bool turboAvailable() { return true; }

// tjhandle 不是线程安全的，但每个相机的解码固定发生在自己的采集线程上，
// 所以给每个线程一份独立的 handle(线程本地)，既避免每帧 tjInitDecompress/
// tjDestroy 的开销，又不用加锁。QThreadStorage 会在线程退出时自动清理。
namespace {
struct TjHandle {
    tjhandle h = nullptr;
    TjHandle() : h(tjInitDecompress()) {}
    ~TjHandle() { if (h) tjDestroy(h); }
};
QThreadStorage<TjHandle*>& tls() {
    static QThreadStorage<TjHandle*> s;
    return s;
}
tjhandle handleForThisThread() {
    if (!tls().hasLocalData()) tls().setLocalData(new TjHandle());
    return tls().localData()->h;
}
} // namespace

QImage decodeMjpeg(const uint8_t* data, int len) {
    if (!data || len <= 0) return {};

    // 灰度直解：只走 turbo 的 TJPF_GRAY 输出，单通道、跳过色度与颜色矩阵。
    if (isGrayDecode()) {
        tjhandle h = handleForThisThread();
        if (h) {
            int w = 0, hh = 0, subsamp = 0, colorspace = 0;
            if (tjDecompressHeader3(h, data, static_cast<unsigned long>(len),
                                    &w, &hh, &subsamp, &colorspace) == 0
                && w > 0 && hh > 0) {
                // QImage 每行 4 字节对齐；用它自己的 bytesPerLine 当 pitch，
                // 让 turbo 直接解进 QImage 的缓冲，零额外拷贝。
                // 计时只框住 tjDecompress2 这一次真正的解码调用，不把上面的
                // header 探测(很快、可忽略)算进"解码耗时"这个统计口径。
                QImage img(w, hh, QImage::Format_Grayscale8);
                QElapsedTimer t; t.start();
                const int rc = tjDecompress2(h, data, static_cast<unsigned long>(len),
                                             img.bits(), w, img.bytesPerLine(), hh,
                                             TJPF_GRAY, TJFLAG_FASTDCT);
                recordTurboGray(t.nsecsElapsed(), rc == 0);
                if (rc == 0) {
                    return img;
                }
            }
        }
        // turbo 灰度解失败(极少见，坏帧)——落到下面的 Qt 兜底，仍尽量给出灰度。
    }

    // 彩色路径 / turbo 失败兜底：Qt 解码。计时只框住 fromData 这次真正的解码，
    // 后面按需补的 convertToFormat(转灰度) 不计入"解码耗时"，避免两件事的
    // 耗时混在一起、对比不出"直解 vs 完整彩色解码"这个核心问题的干净数字。
    QElapsedTimer t; t.start();
    QImage img = QImage::fromData(data, len, "JPG");
    recordQtColor(t.nsecsElapsed(), !img.isNull());
    if (isGrayDecode() && !img.isNull()
        && img.format() != QImage::Format_Grayscale8) {
        img = img.convertToFormat(QImage::Format_Grayscale8);
    }
    return img;
}

#else  // 没有 libjpeg-turbo：纯 Qt 实现，功能等价、无性能收益。

bool turboAvailable() { return false; }

QImage decodeMjpeg(const uint8_t* data, int len) {
    if (!data || len <= 0) return {};
    QElapsedTimer t; t.start();
    QImage img = QImage::fromData(data, len, "JPG");
    recordQtColor(t.nsecsElapsed(), !img.isNull());
    if (isGrayDecode() && !img.isNull()
        && img.format() != QImage::Format_Grayscale8) {
        img = img.convertToFormat(QImage::Format_Grayscale8);
    }
    return img;
}

#endif

} // namespace mocap
