#include "camera/VirtualCamera.hpp"
#include <QImage>
#include <cmath>

namespace mocap {

VirtualCamera::VirtualCamera(quint32 id, QString name, QObject* parent)
    : ICamera(id, std::move(name), parent) {
    phase_ = double(id) * 0.7;   // 每路错开，便于肉眼区分
    connect(&timer_, &QTimer::timeout, this, &VirtualCamera::tick);
}

bool VirtualCamera::start() {
    clock_.start();
    lastNs_ = 0;
    int interval = params_.target_fps > 0 ? 1000 / params_.target_fps : 33;
    timer_.start(interval < 1 ? 1 : interval);
    return true;
}

void VirtualCamera::stop() { timer_.stop(); }

void VirtualCamera::applyParams(const CameraParams& p) {
    params_ = p;
    if (timer_.isActive()) start();
}

void VirtualCamera::tick() {
    QImage img(width_, height_, QImage::Format_Grayscale8);
    img.fill(0);
    phase_ += 0.05;
    struct P { double cx, cy; };
    P spots[3] = {
        { width_ * 0.5 + 180 * std::cos(phase_),        height_ * 0.5 + 120 * std::sin(phase_) },
        { width_ * 0.5 + 120 * std::cos(phase_ * 1.7),  height_ * 0.5 + 160 * std::sin(phase_ * 1.3) },
        { width_ * 0.5 + 90  * std::cos(phase_ * 0.6),  height_ * 0.5 + 90  * std::sin(phase_ * 2.1) },
    };
    const double sigma = 1.6, s2 = 2 * sigma * sigma, peak = 235;
    for (const P& sp : spots) {
        int cx = int(sp.cx), cy = int(sp.cy), r = 5;
        for (int y = (cy - r < 0 ? 0 : cy - r); y <= (cy + r >= height_ ? height_ - 1 : cy + r); ++y) {
            uchar* row = img.scanLine(y);
            for (int x = (cx - r < 0 ? 0 : cx - r); x <= (cx + r >= width_ ? width_ - 1 : cx + r); ++x) {
                double dx = x - sp.cx, dy = y - sp.cy;
                double v = peak * std::exp(-(dx * dx + dy * dy) / s2);
                int nv = int(row[x]) + int(v);
                row[x] = uchar(nv > 255 ? 255 : nv);
            }
        }
    }

    qint64 ns = clock_.nsecsElapsed();
    if (lastNs_ > 0) {
        double dt = (ns - lastNs_) / 1e9;
        if (dt > 0) fps_ = 0.9 * fps_ + 0.1 * (1.0 / dt);
    }
    lastNs_ = ns;

    emit frameReady(id_, img, fps_);

    // 虚拟相机也跑检测：无硬件时即可演示预览叠加与 UDP 输出。
    if (detectOn_) {
        // collectContours 只在手部动捕真的需要轮廓时才打开(contourOn_)，
        // 关掉时 detect() 走跟以前完全一样的路径，不多算、不多拷贝。
        auto blobs = detector_.detect(img.constBits(), width_, height_, dparams_, contourOn_);

        QVector<QPointF> pts;
        pts.reserve(int(blobs.size()));
        for (const Blob& b : blobs) pts.push_back(QPointF(b.cx, b.cy));
        emit blobsReady(id_, pts, ns);

        // 详细版信号：跟 blobsReady 用同一次 detect() 结果，不重复跑检测。
        // 没开轮廓收集时 Blob::contour 都是空的，BlobObservationAdapter 会
        // 优雅地对每个空轮廓返回 0 条观测——发这个信号本身没有额外代价
        // (blobs 已经在手上了，只是换个容器类型)，所以不额外判断
        // contourOn_ 也发它，简化状态管理；真正的开销节省点是
        // detector_.detect() 那一行的 contourOn_ 参数，不是这里发不发信号。
        QVector<Blob> blobVec;
        blobVec.reserve(int(blobs.size()));
        for (const Blob& b : blobs) blobVec.push_back(b);
        emit blobDetailsReady(id_, blobVec, ns);
    }
}

} // namespace mocap
