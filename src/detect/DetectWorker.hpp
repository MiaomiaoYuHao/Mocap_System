#pragma once
// ---------------------------------------------------------------------------
// 独立检测线程。每台相机一个，与该相机的采集/预览线程彻底解耦：
//   - 采集线程只在本线程“空闲”时才转发一帧过来（busy_ 为 false）；
//   - 忙碌时新帧直接跳过（不排队、不积压），检测慢是检测线程自己的事，
//     绝不拖慢预览或采集——三条流水线互相独立，谁慢不连累别人。
// 这样即使检测在某些分辨率/相机数下跑不满帧，预览依然流畅如初。
// ---------------------------------------------------------------------------
#include "detect/CentroidDetector.hpp"
#include "camera/ICamera.hpp"   // Q_DECLARE_METATYPE(mocap::Blob)，blobDetailsReady要用
#include <QObject>
#include <QImage>
#include <QVector>
#include <QPointF>
#include <atomic>
#include <mutex>

namespace mocap {

class DetectWorker : public QObject {
    Q_OBJECT
public:
    // 采集线程在决定要不要转发一帧前，读这个原子标志（无锁、跨线程安全）。
    std::atomic<bool> busy{false};

    void setParams(const DetectParams& p) {
        std::lock_guard<std::mutex> lk(mtx_);
        params_ = p;
    }

    // 手部动捕(3a~3d)需要轮廓点时打开；关掉时 detect() 走跟以前完全一样
    // 的路径，不额外算、不额外拷贝。原子量，跟 busy 同样的跨线程读写模式
    // （主线程调用 setContourCollectionEnabled，检测线程里的 process() 读）。
    void setContourCollectionEnabled(bool on) { contourOn_ = on; }
    bool contourCollectionEnabled() const { return contourOn_; }

public slots:
    void process(quint32 camId, QImage img, qint64 ts_ns);

signals:
    void blobsReady(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns);
    // 详细版：跟 blobsReady 同一次 detect() 结果，带轮廓点（未开
    // contourCollectionEnabled 时每个 Blob::contour 都是空的）。
    void blobDetailsReady(quint32 camId, const QVector<Blob>& blobs, qint64 ts_ns);

private:
    std::mutex mtx_;
    DetectParams params_;
    CentroidDetector detector_;
    std::vector<uint8_t> scratch_;
    std::atomic<bool> contourOn_{false};
};

} // namespace mocap
