#include "ui/DShowCaptureTestDialog.hpp"
#include "ui/Theme.hpp"
#include "camera/DShowCapture.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QMediaDevices>
#include <QCameraDevice>
#include <QPixmap>
#include <QFont>

namespace mocap {

DShowCaptureTestDialog::DShowCaptureTestDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral("DirectShow \u91c7\u96c6\u539f\u578b(\u5b9e\u9a8c\u6027)"));
    resize(560, 520);

    auto* mainLay = new QVBoxLayout(this);

    auto* hint = new QLabel(QStringLiteral(
        "\u7ed5\u5f00 Qt/Media Foundation\uff0c\u76f4\u63a5\u7528 DirectShow \u91c7\u96c6\uff0c\u53ea\u4e3a\u9a8c\u8bc1"
        "\u80fd\u4e0d\u80fd\u62ff\u5230\u7845\u4ef6\u7ea7\u5e27\u7387\uff0c\u4e0d\u63a5\u5165\u4e3b\u7a0b\u5e8f\u73b0\u6709\u529f\u80fd\uff0c\u4e0d\u5f71\u54cd"
        "\u73b0\u6709\u91c7\u96c6\u3002\u9ed8\u8ba4\u53c2\u6570\u5bf9\u5e94\u4f60\u8fd9\u6b21\u6d4b\u8bd5\u786e\u8ba4\u8fc7\u7684 MJPG 1280\u00d7720@120fps\u3002"));
    hint->setWordWrap(true);
    mainLay->addWidget(hint);

    auto* form = new QFormLayout;
    deviceCombo_ = new QComboBox(this);
    form->addRow(QStringLiteral("\u8bbe\u5907"), deviceCombo_);

    widthBox_ = new QSpinBox(this);
    widthBox_->setRange(64, 7680);
    widthBox_->setValue(1280);
    form->addRow(QStringLiteral("\u5bbd(px)"), widthBox_);

    heightBox_ = new QSpinBox(this);
    heightBox_->setRange(64, 4320);
    heightBox_->setValue(720);
    form->addRow(QStringLiteral("\u9ad8(px)"), heightBox_);

    fpsBox_ = new QDoubleSpinBox(this);
    fpsBox_->setRange(1.0, 240.0);
    fpsBox_->setValue(120.0);
    form->addRow(QStringLiteral("\u76ee\u6807\u5e27\u7387"), fpsBox_);

    mainLay->addLayout(form);

    auto* btnRow = new QHBoxLayout;
    startBtn_ = new QPushButton(QStringLiteral("\u5f00\u59cb\u91c7\u96c6"), this);
    stopBtn_ = new QPushButton(QStringLiteral("\u505c\u6b62"), this);
    stopBtn_->setEnabled(false);
    btnRow->addWidget(startBtn_);
    btnRow->addWidget(stopBtn_);
    mainLay->addLayout(btnRow);

    statusLabel_ = new QLabel(QStringLiteral("\u72b6\u6001\uff1a\u672a\u5f00\u59cb"), this);
    mainLay->addWidget(statusLabel_);

    fpsLabel_ = new QLabel(QStringLiteral("\u5b9e\u6d4b\u5e27\u7387\uff1a--"), this);
    QFont fpsFont = fpsLabel_->font(); fpsFont.setPointSize(14); fpsFont.setBold(true);
    fpsLabel_->setFont(fpsFont);
    mainLay->addWidget(fpsLabel_);

    previewLabel_ = new QLabel(this);
    previewLabel_->setMinimumSize(320, 240);
    previewLabel_->setAlignment(Qt::AlignCenter);
    previewLabel_->setStyleSheet(QStringLiteral("QLabel { background:#141414; color:%1; border:1px solid %2; }")
                                     .arg(theme::hex(theme::textDim()), theme::hex(theme::lineHard())));
    previewLabel_->setText(QStringLiteral("(\u9884\u89c8)"));
    mainLay->addWidget(previewLabel_, 1);

    populateDevices();

    connect(startBtn_, &QPushButton::clicked, this, &DShowCaptureTestDialog::onStartClicked);
    connect(stopBtn_, &QPushButton::clicked, this, &DShowCaptureTestDialog::onStopClicked);
}

DShowCaptureTestDialog::~DShowCaptureTestDialog() {
    if (capture_) { capture_->stop(); delete capture_; }
}

void DShowCaptureTestDialog::populateDevices() {
    deviceCombo_->clear();
    const auto devices = QMediaDevices::videoInputs();
    for (const QCameraDevice& d : devices) {
        // occurrenceIndex 的算法跟 WebcamCamera::nameOccurrenceIndex() 保持
        // 一致(同名设备时区分"这是第几个")——两处如果算法不一样，选中的
        // 设备顺序会对不上，容易连到错的物理相机。
        int occurrence = 0;
        for (const QCameraDevice& e : devices) {
            if (&e == &d) break;
            if (e.description() == d.description()) ++occurrence;
        }
        const QString label = QStringLiteral("%1 (#%2)").arg(d.description()).arg(occurrence);
        // userData 存三元组：设备id + 名字occurrence，用QVariantList简单打包。
        QVariantList data; data << d.id() << d.description() << occurrence;
        deviceCombo_->addItem(label, data);
    }
    if (deviceCombo_->count() == 0)
        deviceCombo_->addItem(QStringLiteral("(\u672a\u68c0\u6d4b\u5230\u4efb\u4f55\u89c6\u9891\u8bbe\u5907)"));
}

void DShowCaptureTestDialog::onStartClicked() {
    if (capture_) return;
    if (deviceCombo_->currentData().isNull()) {
        statusLabel_->setText(QStringLiteral("\u72b6\u6001\uff1a\u6ca1\u6709\u53ef\u7528\u8bbe\u5907"));
        return;
    }
    const QVariantList data = deviceCombo_->currentData().toList();
    if (data.size() != 3) return;
    const QByteArray deviceId = data[0].toByteArray();
    const QString friendlyName = data[1].toString();
    const int occurrence = data[2].toInt();

    capture_ = new DShowCapture(this);
    connect(capture_, &DShowCapture::frameReady, this, [this](QImage frame, qint64) {
        previewLabel_->setPixmap(QPixmap::fromImage(frame).scaled(
            previewLabel_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    });
    connect(capture_, &DShowCapture::measuredFpsChanged, this, [this](double fps) {
        fpsLabel_->setText(QStringLiteral("\u5b9e\u6d4b\u5e27\u7387\uff1a%1 fps").arg(fps, 0, 'f', 1));
    });
    connect(capture_, &DShowCapture::errorOccurred, this, [this](QString msg) {
        statusLabel_->setText(QStringLiteral("\u72b6\u6001\uff1a\u9519\u8bef - %1").arg(msg));
    });

    const bool ok = capture_->start(deviceId, friendlyName, occurrence,
                                    widthBox_->value(), heightBox_->value(), fpsBox_->value());
    if (ok) {
        statusLabel_->setText(QStringLiteral("\u72b6\u6001\uff1a\u91c7\u96c6\u4e2d\u2026"));
        startBtn_->setEnabled(false);
        stopBtn_->setEnabled(true);
        deviceCombo_->setEnabled(false);
        widthBox_->setEnabled(false);
        heightBox_->setEnabled(false);
        fpsBox_->setEnabled(false);
    } else {
        delete capture_;
        capture_ = nullptr;
    }
}

void DShowCaptureTestDialog::onStopClicked() {
    if (!capture_) return;
    capture_->stop();
    delete capture_;
    capture_ = nullptr;

    statusLabel_->setText(QStringLiteral("\u72b6\u6001\uff1a\u5df2\u505c\u6b62"));
    fpsLabel_->setText(QStringLiteral("\u5b9e\u6d4b\u5e27\u7387\uff1a--"));
    previewLabel_->setText(QStringLiteral("(\u9884\u89c8)"));
    previewLabel_->setPixmap(QPixmap());
    startBtn_->setEnabled(true);
    stopBtn_->setEnabled(false);
    deviceCombo_->setEnabled(true);
    widthBox_->setEnabled(true);
    heightBox_->setEnabled(true);
    fpsBox_->setEnabled(true);
}

} // namespace mocap
