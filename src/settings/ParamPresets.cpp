#include "settings/ParamPresets.hpp"
#include "camera/WebcamCamera.hpp"
#include "camera/DShowControl.hpp"

#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

namespace mocap {

// 【新增槽位记得同步 .hpp 的 enum 和 sets_ 维度】JSON 里多一个 "boot" 段，
// 老配置文件没有这个键时 load() 读出来就是空的，向后兼容，不需要迁移。
static const char* kNames[ParamPresets::WhichCount] = { "board", "track", "boot" };

ParamPresets::ParamPresets() {
    QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (dir.isEmpty()) dir = QDir::homePath();
    QDir().mkpath(dir);
    path_ = dir + "/param_presets.json";
    load();
}

int ParamPresets::captureAll(Which w, const QList<WebcamCamera*>& cams) {
    int ok = 0;
    for (WebcamCamera* c : cams) {
        // 这里必须传裸的 device().description()（DirectShow FriendlyName），不能传
        // c->name()：USB 同名相机修复后 name() 已被 makeCameraLabel() 拼成带
        // "#机位号" 的显示名（2026-09 起不再带 [位置/哈希]，但仍然带 #N），
        // 而 DShowControl 退化的按名字匹配要跟 DirectShow 的 FriendlyName
        // 精确相等，传显示名会全部绑不上。
        DShowControl ctl(c->device().id(), c->device().description(), c->nameOccurrenceIndex());
        if (!ctl.valid()) continue;
        QVector<P> list;
        for (const DShowProp& p : ctl.properties()) {
            if (!p.supported) continue;
            list.push_back({ p.id, p.isCameraControl, p.value, p.isAuto });
        }
        if (list.isEmpty()) continue;
        sets_[w][c->deviceKey()] = list;
        ++ok;
    }
    if (ok > 0) save();
    return ok;
}

bool ParamPresets::hasFor(Which w, const QString& deviceKey) const {
    return sets_[w].contains(deviceKey);
}

int ParamPresets::applyAll(Which w, const QList<WebcamCamera*>& cams) const {
    int ok = 0;
    for (WebcamCamera* c : cams)
        if (applyTo(w, c)) ++ok;
    return ok;
}

// 单台版。applyAll 现在只是它的循环外壳，两条路径共用同一份下发逻辑，
// 不会出现"批量能套上、开机单台套不上"这种两份实现走样的问题。
bool ParamPresets::applyTo(Which w, WebcamCamera* c) const {
    if (!c) return false;
    {
        const auto it = sets_[w].constFind(c->deviceKey());
        if (it == sets_[w].constEnd()) return false;
        // 这里必须传裸的 device().description()（DirectShow FriendlyName），不能传
        // c->name()：USB 同名相机修复后 name() 已被 makeCameraLabel() 拼成带
        // "#机位号" 的显示名（2026-09 起不再带 [位置/哈希]，但仍然带 #N），
        // 而 DShowControl 退化的按名字匹配要跟 DirectShow 的 FriendlyName
        // 精确相等，传显示名会全部绑不上。
        DShowControl ctl(c->device().id(), c->device().description(), c->nameOccurrenceIndex());
        if (!ctl.valid()) return false;
        for (const P& p : it.value())
            ctl.set(p.id, p.cc, p.v, p.a);
    }
    return true;
}

bool ParamPresets::save() const {
    QJsonObject root;
    for (int w = 0; w < WhichCount; ++w) {
        QJsonObject setObj;
        for (auto it = sets_[w].constBegin(); it != sets_[w].constEnd(); ++it) {
            QJsonArray arr;
            for (const P& p : it.value()) {
                QJsonObject o;
                o["id"] = int(p.id); o["cc"] = p.cc;
                o["v"] = int(p.v);   o["a"] = p.a;
                arr << o;
            }
            setObj[it.key()] = arr;
        }
        root[kNames[w]] = setObj;
    }
    QFile f(path_);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return true;
}

void ParamPresets::load() {
    QFile f(path_);
    if (!f.open(QIODevice::ReadOnly)) return;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isObject()) return;
    for (int w = 0; w < WhichCount; ++w) {
        sets_[w].clear();
        const QJsonObject setObj = doc.object()[kNames[w]].toObject();
        for (auto it = setObj.constBegin(); it != setObj.constEnd(); ++it) {
            QVector<P> list;
            for (const auto& v : it.value().toArray()) {
                const QJsonObject o = v.toObject();
                list.push_back({ long(o["id"].toInt()), o["cc"].toBool(),
                                 long(o["v"].toInt()),  o["a"].toBool() });
            }
            if (!list.isEmpty()) sets_[w][it.key()] = list;
        }
    }
}

} // namespace mocap
