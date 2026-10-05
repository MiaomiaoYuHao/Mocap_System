#pragma once
// 虚拟相机：黑底 + 移动亮点，按目标帧率产帧。无硬件时测试布局/管线用。
#include "camera/ICamera.hpp"
#include "detect/CentroidDetector.hpp"
#include <QTimer>
#include <QElapsedTimer>

namespace mocap {

class VirtualCamera : public ICamera {
    Q_OBJECT
public:
    VirtualCamera(quint32 id, QString name, QObject* parent = nullptr);

    bool start() override;
    void stop()  override;
    void applyParams(const CameraParams& p) override;
    CameraParams params() const override { return params_; }
    // deviceKey() 用基类默认值 "virt"。

    void setDetectEnabled(bool on) override { detectOn_ = on; }
    bool detectEnabled() const override { return detectOn_; }
    void setDetectParams(const DetectParams& p) override { dparams_ = p; }
    void setGrayOutput(bool) override {}          // 虚拟相机本就输出灰度
    bool grayOutput() const override { return true; }

    // 手部动捕(3a~3d)需要轮廓点时，HandTrackingWorker 会调这个打开；不开
    // 就还是老样子只发质心，不额外算轮廓、不额外拷贝数据。
    void setContourCollectionEnabled(bool on) override { contourOn_ = on; }
    bool contourCollectionEnabled() const override { return contourOn_; }

private slots:
    void tick();

private:
    QTimer timer_;
    QElapsedTimer clock_;
    CameraParams params_;
    int width_ = 640, height_ = 480;
    double phase_ = 0.0;
    double fps_ = 0.0;
    qint64 lastNs_ = 0;
    bool detectOn_ = false;
    bool contourOn_ = false;
    DetectParams dparams_;
    CentroidDetector detector_;
};

} // namespace mocap
