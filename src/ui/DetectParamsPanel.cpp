#include "ui/DetectParamsPanel.hpp"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

namespace mocap {

namespace {
// 阈值/面积/圆度这几个滑块，你们这轮反复重启测试改了无数次，之前只在
// 单次运行内存里，关一次程序就得重调。落盘到用户配置目录，跟
// CalibrationStore/ParamPresets 同一个思路——不需要额外的类，就是个简单
// struct，直接在这写两个自由函数。
QString settingsPath() {
    QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (dir.isEmpty()) dir = QDir::homePath();
    QDir().mkpath(dir);
    return dir + "/detect_params.json";
}

bool loadPersisted(DetectParams& p) {
    QFile f(settingsPath());
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) return false;
    const QJsonObject o = doc.object();
    if (o.isEmpty()) return false;
    p.threshold = o["threshold"].toInt(p.threshold);
    p.minArea = o["minArea"].toInt(p.minArea);
    p.maxArea = o["maxArea"].toInt(p.maxArea);
    p.minCircularity = float(o["minCircularity"].toDouble(p.minCircularity));
    return true;
}

void savePersisted(const DetectParams& p) {
    QJsonObject o;
    o["threshold"] = p.threshold;
    o["minArea"] = p.minArea;
    o["maxArea"] = p.maxArea;
    o["minCircularity"] = double(p.minCircularity);
    QFile f(settingsPath());
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        f.write(QJsonDocument(o).toJson(QJsonDocument::Compact));
}
} // namespace

DetectParamsPanel::DetectParamsPanel(const DetectParams& initial, QWidget* parent)
    : QWidget(parent), params_(initial) {
    // 有上次保存的值就用它覆盖调用方传进来的 initial——调用方（MainWindow）
    // 传的只是"没存过时"的兜底默认值。
    loadPersisted(params_);

    auto* form = new QFormLayout(this);
    form->setContentsMargins(10, 10, 10, 10);

    addRow(QStringLiteral("阈值"), 1, 254, params_.threshold,
           [](int v) { return QString::number(v); },
           [](DetectParams& p, int v) { p.threshold = v; });

    addRow(QStringLiteral("面积下限"), 0, 500, params_.minArea,
           [](int v) { return v > 0 ? QString::number(v) : QStringLiteral("不限"); },
           [](DetectParams& p, int v) { p.minArea = v; });

    // maxArea 0 = 不设上限（CentroidDetector 里 p.maxArea<=0 时按不限处理）。
    addRow(QStringLiteral("面积上限"), 0, 20000, params_.maxArea,
           [](int v) { return v > 0 ? QString::number(v) : QStringLiteral("不限"); },
           [](DetectParams& p, int v) { p.maxArea = v; });

    // 圆度用 0~100 代表 0.00~1.00，0 = 关闭圆度过滤。
    addRow(QStringLiteral("圆度"), 0, 100, int(params_.minCircularity * 100),
           [](int v) { return v > 0 ? QString::number(v / 100.0, 'f', 2) : QStringLiteral("关闭"); },
           [](DetectParams& p, int v) { p.minCircularity = v / 100.0f; });

    connect(this, &DetectParamsPanel::paramsChanged, this,
            [](const DetectParams& p) { savePersisted(p); });
}

QSlider* DetectParamsPanel::addRow(const QString& label, int lo, int hi, int initVal,
                                   std::function<QString(int)> displayText,
                                   std::function<void(DetectParams&, int)> apply) {
    auto* slider = new QSlider(Qt::Horizontal);
    slider->setRange(lo, hi);
    slider->setValue(initVal);
    slider->setFixedWidth(160);

    auto* valLabel = new QLabel(displayText(initVal));
    valLabel->setMinimumWidth(44);

    auto* row = new QHBoxLayout;
    row->addWidget(slider, 1);
    row->addWidget(valLabel);

    static_cast<QFormLayout*>(layout())->addRow(label, row);

    connect(slider, &QSlider::valueChanged, this,
            [this, valLabel, displayText, apply](int v) {
                valLabel->setText(displayText(v));
                apply(params_, v);
                emit paramsChanged(params_);
            });

    return slider;
}

} // namespace mocap