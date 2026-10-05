#pragma once
// ---------------------------------------------------------------------------
// DShowCapture(原生DirectShow采集原型)的独立测试入口——完全不碰现有的
// WebcamCamera/CameraManager那套能跑的采集流程，纯粹用来验证"绕开Qt/MF
// 之后能不能真正拿到硬件级帧率"这一件事。跟正式功能集成是完全独立的两步，
// 见 DShowCapture.hpp 顶部注释。
// ---------------------------------------------------------------------------
#include <QDialog>

class QComboBox;
class QSpinBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace mocap {

class DShowCapture;

class DShowCaptureTestDialog : public QDialog {
    Q_OBJECT
public:
    explicit DShowCaptureTestDialog(QWidget* parent = nullptr);
    ~DShowCaptureTestDialog() override;

private slots:
    void onStartClicked();
    void onStopClicked();

private:
    void populateDevices();

    QComboBox* deviceCombo_ = nullptr;
    QSpinBox* widthBox_ = nullptr;
    QSpinBox* heightBox_ = nullptr;
    QDoubleSpinBox* fpsBox_ = nullptr;
    QPushButton* startBtn_ = nullptr;
    QPushButton* stopBtn_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QLabel* fpsLabel_ = nullptr;
    QLabel* previewLabel_ = nullptr;

    DShowCapture* capture_ = nullptr;
};

} // namespace mocap
