#include "calib/CalibrationLibrary.hpp"
#include "core/ProjectPaths.hpp"
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDateTime>
#include <QTextStream>

namespace mocap {

CalibrationLibrary::CalibrationLibrary() {
    QDir().mkpath(rootDir());
    migrateFromLegacyRootIfNeeded();   // 旧版存在系统配置目录里的模板搬进项目目录
    loadTemplateList();
    loadActiveId();
    pruneEmptyTemplates();   // 清掉"向导被取消/没跑完求解"留下的空壳模板
}

QString CalibrationLibrary::rootDir() const {
    // 运行时定位项目根目录，而不是用编译期焊死的 PROJECT_SOURCE_DIR——
    // 后者在"项目文件夹被改名/复制成新版本"之后会继续指向旧文件夹，
    // 导致标定模板存到老地方去。详见 core/ProjectPaths.hpp 顶部说明。
    return projectRootDir() + QStringLiteral("/calib_templates");
}

QString CalibrationLibrary::legacyRootDir() const {
    QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (dir.isEmpty()) dir = QDir::homePath();
    return dir + "/calib_templates";
}

void CalibrationLibrary::migrateFromLegacyRootIfNeeded() {
    const QString oldRoot = legacyRootDir();
    const QString newRoot = rootDir();
    if (oldRoot == newRoot) return;              // 理论上不会相等，保险起见
    QDir oldDir(oldRoot);
    if (!oldDir.exists()) return;                // 没有旧数据，全新安装，无事发生

    // 逐个模板文件夹搬。已经存在同名的就跳过——新目录里的数据优先，绝不
    // 覆盖（想象一下用户已经在新版重新标定过了，这里再拿旧的盖掉就毁了）。
    const QStringList entries = oldDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    int moved = 0;
    for (const QString& id : entries) {
        const QString src = oldRoot + "/" + id;
        const QString dst = newRoot + "/" + id;
        if (QDir(dst).exists()) continue;
        if (!QFile::exists(src + "/meta.json")) continue;   // 不是合法模板文件夹
        // 用拷贝而不是 rename：跨盘符（比如项目在 D 盘、AppData 在 C 盘）
        // 时 rename 会失败，而这恰恰是 Windows 上的常见情形。
        if (!QDir().mkpath(dst)) continue;
        bool ok = true;
        const QStringList files = QDir(src).entryList(QDir::Files);
        for (const QString& f : files)
            if (!QFile::copy(src + "/" + f, dst + "/" + f)) ok = false;
        if (ok) ++moved;
    }

    // active.txt 也一并带过来（当且仅当新目录还没有），否则用户会发现
    // "模板都在，但当前使用的那个变回默认了"。
    if (moved > 0 && !QFile::exists(newRoot + "/active.txt")
                  && QFile::exists(oldRoot + "/active.txt")) {
        QFile::copy(oldRoot + "/active.txt", newRoot + "/active.txt");
    }

    // 旧目录整个保留不删：迁移是拷贝语义，出任何意外用户都能自己回去找。
    // 只留一张便条说明发生了什么，免得日后看到两份一样的数据一头雾水。
    if (moved > 0) {
        QFile note(oldRoot + "/_已迁移到项目目录.txt");
        if (note.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            const QString msg =
                QStringLiteral("这些标定模板已于 %1 复制到：\n%2\n\n"
                               "程序现在只读取上面那个目录。这里的文件是复制过去的副本源，\n"
                               "保留未删，确认新位置一切正常后可以手动删除本文件夹。\n")
                    .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss")),
                         newRoot);
            note.write(msg.toUtf8());
        }
    }
}

QString CalibrationLibrary::templateDir(const QString& id) const {
    return rootDir() + "/" + id;
}

void CalibrationLibrary::loadTemplateList() {
    templates_.clear();
    const QDir root(rootDir());
    const QStringList dirs = root.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QString& d : dirs) {
        QFile f(templateDir(d) + "/meta.json");
        if (!f.open(QIODevice::ReadOnly)) continue;   // 没有 meta.json，不是合法模板文件夹，跳过
        const QJsonObject o = QJsonDocument::fromJson(f.readAll()).object();
        if (o.isEmpty()) continue;
        TemplateInfo t;
        t.id = o["id"].toString(d);
        t.displayName = o["displayName"].toString(t.id);
        t.createdAtMs = qint64(o["createdAtMs"].toDouble());
        templates_.push_back(t);
    }
}

void CalibrationLibrary::loadActiveId() {
    QFile f(rootDir() + "/active.txt");
    if (f.open(QIODevice::ReadOnly))
        activeId_ = QString::fromUtf8(f.readAll()).trimmed();
}

void CalibrationLibrary::saveActiveId() const {
    QFile f(rootDir() + "/active.txt");
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        f.write(activeId_.toUtf8());
}

bool CalibrationLibrary::writeMeta(const TemplateInfo& info) const {
    QJsonObject o;
    o["id"] = info.id;
    o["displayName"] = info.displayName;
    o["createdAtMs"] = double(info.createdAtMs);
    QFile f(templateDir(info.id) + "/meta.json");
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
    f.write(QJsonDocument(o).toJson(QJsonDocument::Indented));
    return true;
}

TemplateInfo CalibrationLibrary::templateInfo(const QString& id) const {
    for (const auto& t : templates_) if (t.id == id) return t;
    return TemplateInfo{};
}

QVector<CameraCalibration> CalibrationLibrary::templateCameras(const QString& id) const {
    QVector<CameraCalibration> out;
    CalibrationStore store(templateDir(id) + "/calibration.json");
    for (const QString& k : store.keys()) out.push_back(store.get(k));
    return out;
}

int CalibrationLibrary::cameraCount(const QString& id) const {
    CalibrationStore store(templateDir(id) + "/calibration.json");
    return store.keys().size();
}

CalibrationStore* CalibrationLibrary::beginNewTemplateStore(const QString& displayNameHint, QString* outId) {
    TemplateInfo t;
    t.id = "tmpl_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmsszzz");
    t.displayName = displayNameHint.isEmpty()
        ? QStringLiteral("标定 %1").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm"))
        : displayNameHint;
    t.createdAtMs = QDateTime::currentMSecsSinceEpoch();

    QDir().mkpath(templateDir(t.id));
    writeMeta(t);
    templates_.push_back(t);
    if (outId) *outId = t.id;

    return new CalibrationStore(templateDir(t.id) + "/calibration.json");
}

CalibrationStore* CalibrationLibrary::openTemplateStore(const QString& templateId) const {
    return new CalibrationStore(templateDir(templateId) + "/calibration.json");
}

bool CalibrationLibrary::renameTemplate(const QString& id, const QString& newName) {
    for (auto& t : templates_) {
        if (t.id != id) continue;
        t.displayName = newName;
        return writeMeta(t);
    }
    return false;
}

bool CalibrationLibrary::deleteTemplate(const QString& id) {
    if (!QDir(templateDir(id)).removeRecursively()) return false;
    for (int i = 0; i < templates_.size(); ++i)
        if (templates_[i].id == id) { templates_.removeAt(i); break; }
    if (activeId_ == id) { activeId_.clear(); saveActiveId(); }
    return true;
}

bool CalibrationLibrary::restoreTemplate(const TemplateInfo& meta, const QVector<CameraCalibration>& cams) {
    QDir().mkpath(templateDir(meta.id));
    if (!writeMeta(meta)) return false;

    CalibrationStore store(templateDir(meta.id) + "/calibration.json");
    for (const auto& c : cams) store.set(c);
    if (!store.save()) return false;

    // 避免重复插入（比如撤销时模板其实还在列表里没被真正移除的极端情况）。
    bool exists = false;
    for (const auto& t : templates_) if (t.id == meta.id) { exists = true; break; }
    if (!exists) templates_.push_back(meta);
    return true;
}

bool CalibrationLibrary::removeCameraFromTemplate(const QString& id, const QString& deviceKey) {
    CalibrationStore store(templateDir(id) + "/calibration.json");
    if (!store.has(deviceKey)) return false;
    store.remove(deviceKey);
    return store.save();
}

bool CalibrationLibrary::addCameraToTemplate(const QString& id, const CameraCalibration& cam) {
    CalibrationStore store(templateDir(id) + "/calibration.json");
    store.set(cam);
    return store.save();
}

bool CalibrationLibrary::renameCamera(const QString& templateId, const QString& deviceKey, const QString& alias) {
    CalibrationStore store(templateDir(templateId) + "/calibration.json");
    if (!store.has(deviceKey)) return false;
    CameraCalibration c = store.get(deviceKey);
    c.alias = alias;
    store.set(c);
    return store.save();
}

void CalibrationLibrary::pruneEmptyTemplates() {
    for (int i = templates_.size() - 1; i >= 0; --i) {
        const auto& t = templates_[i];
        if (t.id == activeId_) continue;         // 当前使用中的，哪怕暂时是空的也不清
        if (cameraCount(t.id) == 0) {
            QDir(templateDir(t.id)).removeRecursively();
            templates_.removeAt(i);
        }
    }
}

void CalibrationLibrary::setActiveTemplateId(const QString& id) {
    activeId_ = id;
    saveActiveId();
}

} // namespace mocap
