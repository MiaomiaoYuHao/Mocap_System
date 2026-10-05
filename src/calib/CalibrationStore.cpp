#include "calib/CalibrationStore.hpp"
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

namespace mocap {

CalibrationStore::CalibrationStore(const QString& path) {
    if (!path.isEmpty()) {
        path_ = path;
    } else {
        QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
        if (dir.isEmpty()) dir = QDir::homePath();
        QDir().mkpath(dir);
        path_ = dir + "/calibration.json";
    }
    load();
    loadRemap();
}

QString CalibrationStore::resolveKey(const QString& deviceKey) const {
    // 自身就有标定，直接用自身——即使它也在 remap 表里，"自己有标定"优先
    // （避免明明这台已经重新标过了，却还被旧映射拽到别的 key 上）。
    if (map_.contains(deviceKey)) return deviceKey;
    // 自身没标定，但被映射到了某个有标定的旧 key，用那个旧 key。
    const QString mapped = remap_.value(deviceKey);
    if (!mapped.isEmpty() && map_.contains(mapped)) return mapped;
    return deviceKey;   // 没辙，返回自身（get 会得到空标定）
}

bool CalibrationStore::has(const QString& deviceKey) const {
    return map_.contains(resolveKey(deviceKey));
}

CameraCalibration CalibrationStore::get(const QString& deviceKey) const {
    const QString key = resolveKey(deviceKey);
    if (map_.contains(key)) {
        CameraCalibration c = map_.value(key);
        // 如果是经重映射拿到的旧标定，把 deviceKey 换成调用方传入的新 key，
        // 让上层（三角化）拿到的标识仍然是"这台实时相机"，内外参用旧标定。
        c.deviceKey = deviceKey;
        return c;
    }
    CameraCalibration c; c.deviceKey = deviceKey; return c;   // 空标定
}

void CalibrationStore::setRemap(const QString& newKey, const QString& oldKey) {
    if (newKey.isEmpty()) return;
    if (oldKey.isEmpty() || oldKey == newKey) { clearRemap(newKey); return; }
    remap_[newKey] = oldKey;
    saveRemap();
}

void CalibrationStore::clearRemap(const QString& newKey) {
    if (remap_.remove(newKey) > 0) saveRemap();
}

int CalibrationStore::calibratedCount() const {
    int n = 0;
    for (const auto& c : map_) if (c.isCalibrated()) ++n;
    return n;
}

bool CalibrationStore::load() {
    QFile f(path_);
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) return false;
    const QJsonArray arr = doc.object()["cameras"].toArray();
    map_.clear();
    for (const auto& v : arr) {
        CameraCalibration c = CameraCalibration::fromJson(v.toObject());
        if (!c.deviceKey.isEmpty()) map_[c.deviceKey] = c;
    }
    return true;
}

bool CalibrationStore::save() const {
    QJsonArray arr;
    for (const auto& c : map_) arr.append(c.toJson());
    QJsonObject root; root["cameras"] = arr; root["version"] = 1;
    QFile f(path_);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

// remap 存在 calibration.json 旁边的 calibration.remap.json——单独一个
// 文件，不混进标定数据本身，这样模板/标定库保持干净，删掉 remap 文件
// 只是回到"重插拔后需要重新映射一次"，不影响任何标定结果。
QString CalibrationStore::remapFilePath() const {
    QString p = path_;
    if (p.endsWith(".json")) p.chop(5);
    return p + ".remap.json";
}

bool CalibrationStore::loadRemap() {
    remap_.clear();
    QFile f(remapFilePath());
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) return false;
    const QJsonObject obj = doc.object()["remap"].toObject();
    for (auto it = obj.constBegin(); it != obj.constEnd(); ++it)
        remap_[it.key()] = it.value().toString();
    return true;
}

bool CalibrationStore::saveRemap() const {
    QJsonObject obj;
    for (auto it = remap_.constBegin(); it != remap_.constEnd(); ++it)
        obj[it.key()] = it.value();
    QJsonObject root; root["remap"] = obj; root["version"] = 1;
    QFile f(remapFilePath());
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

} // namespace mocap
