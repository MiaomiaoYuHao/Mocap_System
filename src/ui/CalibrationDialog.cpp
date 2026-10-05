#include "ui/CalibrationDialog.hpp"
#include "ui/Theme.hpp"
#include "camera/CameraManager.hpp"
#include "camera/ICamera.hpp"
#include "calib/CalibrationStore.hpp"

#include <QListWidget>
#include <QPlainTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QJsonDocument>
#include <QJsonObject>

namespace mocap {

CalibrationDialog::CalibrationDialog(CameraManager* mgr, CalibrationStore* store, QWidget* parent)
    : QDialog(parent), mgr_(mgr), store_(store) {
    setWindowTitle(QStringLiteral("相机标定（内参 / 外参）"));
    resize(720, 520);

    auto* root = new QHBoxLayout(this);

    // 左：相机列表
    auto* left = new QVBoxLayout;
    left->addWidget(new QLabel(QStringLiteral("相机")));
    list_ = new QListWidget;
    list_->setMinimumWidth(220);
    left->addWidget(list_, 1);
    root->addLayout(left);

    // 右：标定编辑
    auto* right = new QVBoxLayout;
    status_ = new QLabel;
    status_->setWordWrap(true);
    right->addWidget(status_);
    right->addWidget(new QLabel(QStringLiteral(
        "标定数据（JSON）。可手动填写，或从标定工具导出后粘贴/导入：")));
    editor_ = new QPlainTextEdit;
    editor_->setStyleSheet(theme::monoCss(12.5));
    right->addWidget(editor_, 1);

    auto* btns = new QHBoxLayout;
    auto* apply = new QPushButton(QStringLiteral("保存此相机"));
    auto* imp   = new QPushButton(QStringLiteral("导入 JSON…"));
    auto* exp   = new QPushButton(QStringLiteral("导出全部…"));
    btns->addWidget(apply); btns->addWidget(imp); btns->addWidget(exp);
    btns->addStretch(1);
    auto* close = new QPushButton(QStringLiteral("关闭"));
    btns->addWidget(close);
    right->addLayout(btns);
    root->addLayout(right, 1);

    connect(list_, &QListWidget::currentRowChanged, this, &CalibrationDialog::onSelectCamera);
    connect(apply, &QPushButton::clicked, this, &CalibrationDialog::applyCurrentJson);
    connect(imp,   &QPushButton::clicked, this, &CalibrationDialog::importJsonFile);
    connect(exp,   &QPushButton::clicked, this, &CalibrationDialog::exportJsonFile);
    connect(close, &QPushButton::clicked, this, &QDialog::accept);

    refreshList();
    if (list_->count() > 0) list_->setCurrentRow(0);
}

void CalibrationDialog::refreshList() {
    list_->clear();
    keys_.clear();
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        const QString key = c->deviceKey();
        keys_ << key;
        const bool done = store_->has(key) && store_->get(key).isCalibrated();
        list_->addItem(QString("%1  %2").arg(done ? QStringLiteral("●") : QStringLiteral("○"),
                                             c->name()));
    }
    const int n = store_->calibratedCount();
    status_->setText(QStringLiteral(
        "● 已标定  ○ 未标定 　|　 已标定 %1 台。三角化需要每台都有内参，"
        "且至少两台有外参（相对位姿）。").arg(n));
}

void CalibrationDialog::onSelectCamera(int row) {
    if (row < 0 || row >= keys_.size()) return;
    loadCameraToEditor(keys_[row]);
}

void CalibrationDialog::loadCameraToEditor(const QString& deviceKey) {
    CameraCalibration c = store_->has(deviceKey) ? store_->get(deviceKey)
                                                 : CameraCalibration{};
    c.deviceKey = deviceKey;
    const QJsonDocument doc(c.toJson());
    editor_->setPlainText(QString::fromUtf8(doc.toJson(QJsonDocument::Indented)));
}

void CalibrationDialog::applyCurrentJson() {
    const int row = list_->currentRow();
    if (row < 0 || row >= keys_.size()) return;

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(editor_->toPlainText().toUtf8(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject()) {
        QMessageBox::warning(this, QStringLiteral("标定"),
            QStringLiteral("JSON 解析失败：%1").arg(err.errorString()));
        return;
    }
    CameraCalibration c = CameraCalibration::fromJson(doc.object());
    c.deviceKey = keys_[row];   // 强制绑定到选中相机
    store_->set(c);
    if (!store_->save()) {
        QMessageBox::warning(this, QStringLiteral("标定"),
            QStringLiteral("保存失败：%1（检查磁盘空间或写入权限）").arg(store_->filePath()));
        return;
    }
    refreshList();
    list_->setCurrentRow(row);
    status_->setText(QStringLiteral("已保存到 %1").arg(store_->filePath()));
}

void CalibrationDialog::importJsonFile() {
    const QString path = QFileDialog::getOpenFileName(this,
        QStringLiteral("导入标定 JSON"), QString(), QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) return;
    editor_->setPlainText(QString::fromUtf8(f.readAll()));
    status_->setText(QStringLiteral("已载入到编辑框，检查后点“保存此相机”。"));
}

void CalibrationDialog::exportJsonFile() {
    const QString path = QFileDialog::getSaveFileName(this,
        QStringLiteral("导出全部标定"), QStringLiteral("calibration.json"),
        QStringLiteral("JSON (*.json)"));
    if (path.isEmpty()) return;
    // store 已经在自己的路径存了一份；这里另存一份到用户选的位置。
    QFile src(store_->filePath()), dst(path);
    if (src.open(QIODevice::ReadOnly) && dst.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        dst.write(src.readAll());
        status_->setText(QStringLiteral("已导出到 %1").arg(path));
    }
}

} // namespace mocap