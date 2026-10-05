#include "detect/DetectWorker.hpp"
#include <cstring>

namespace mocap {

// 把任意 QImage 打包成检测器要的连续单通道 buffer。
static const uint8_t* packGray(const QImage& src, QImage& hold, std::vector<uint8_t>& tmp) {
    const QImage* g = &src;
    if (src.format() != QImage::Format_Grayscale8) {
        hold = src.convertToFormat(QImage::Format_Grayscale8);
        g = &hold;
    }
    const int w = g->width(), h = g->height();
    if (g->bytesPerLine() == w) return g->constBits();
    tmp.resize(size_t(w) * h);
    for (int y = 0; y < h; ++y)
        memcpy(tmp.data() + size_t(y) * w, g->constScanLine(y), size_t(w));
    return tmp.data();
}

void DetectWorker::process(quint32 camId, QImage img, qint64 ts_ns) {
    busy = true;   // 采集线程看到这个就不会再转发新帧过来，避免排队积压

    DetectParams p;
    { std::lock_guard<std::mutex> lk(mtx_); p = params_; }

    QImage hold;
    const uint8_t* buf = packGray(img, hold, scratch_);
    auto blobs = detector_.detect(buf, img.width(), img.height(), p, contourOn_.load());

    QVector<QPointF> pts;
    pts.reserve(int(blobs.size()));
    for (const Blob& b : blobs) pts.push_back(QPointF(b.cx, b.cy));
    emit blobsReady(camId, pts, ts_ns);

    // 跟 blobsReady 用同一次 detect() 结果，不重复跑检测；contourOn_ 关闭
    // 时每个 Blob::contour 都是空的，下游(BlobObservationAdapter)会优雅地
    // 对空轮廓返回 0 条观测，这里不需要额外判断要不要发这个信号。
    QVector<Blob> blobVec;
    blobVec.reserve(int(blobs.size()));
    for (const Blob& b : blobs) blobVec.push_back(b);
    emit blobDetailsReady(camId, blobVec, ts_ns);

    busy = false;   // 处理完才放行下一帧，天然按检测线程能跟上的速度走
}

} // namespace mocap
