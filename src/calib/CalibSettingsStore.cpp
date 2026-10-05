#include "calib/CalibSettingsStore.hpp"
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

namespace mocap {

CalibSettingsStore::CalibSettingsStore(const QString& path) {
    if (!path.isEmpty()) {
        path_ = path;
    } else {
        QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
        if (dir.isEmpty()) dir = QDir::homePath();
        QDir().mkpath(dir);
        path_ = dir + "/calib_wizard_settings.json";
    }
    load();
}

bool CalibSettingsStore::load() {
    QFile f(path_);
    if (!f.open(QIODevice::ReadOnly)) return false;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) return false;
    const QJsonObject root = doc.object();

    const QJsonObject b = root["board"].toObject();
    if (!b.isEmpty()) {
        board_.dict = b["dict"].toString(board_.dict);
        board_.squaresX = b["squaresX"].toInt(board_.squaresX);
        board_.squaresY = b["squaresY"].toInt(board_.squaresY);
        board_.squareMM = b["squareMM"].toDouble(board_.squareMM);
        board_.markerMM = b["markerMM"].toDouble(board_.markerMM);
    }

    const QJsonArray w = root["world"].toArray();
    if (w.size() == 4) {
        for (int r = 0; r < 4; ++r) {
            const QJsonArray row = w[r].toArray();
            if (row.size() != 3) continue;
            for (int c = 0; c < 3; ++c) world_[r][c] = row[c].toDouble();
        }
    }

    // 老的设置文件里没有这个键，用 contains 判断而不是无脑取——缺失时保持
    // 默认（挥球），不会因为读到 toBool() 的默认 false 把老用户的"未设置"
    // 误当成"明确选了挥球"（这里默认值本来就是 false，效果一样，但用
    // contains 语义更清楚，将来默认值若改动也不会踩坑）。
    if (root.contains("useBoardExtrinsics"))
        useBoardExtrinsics_ = root["useBoardExtrinsics"].toBool(useBoardExtrinsics_);
    return true;
}

bool CalibSettingsStore::save() const {
    QJsonObject b;
    b["dict"] = board_.dict;
    b["squaresX"] = board_.squaresX;
    b["squaresY"] = board_.squaresY;
    b["squareMM"] = board_.squareMM;
    b["markerMM"] = board_.markerMM;

    QJsonArray w;
    for (const auto& row : world_) {
        QJsonArray r;
        for (double v : row) r << v;
        w << r;
    }

    QJsonObject root;
    root["board"] = b;
    root["world"] = w;
    root["useBoardExtrinsics"] = useBoardExtrinsics_;

    QFile f(path_);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

} // namespace mocap
