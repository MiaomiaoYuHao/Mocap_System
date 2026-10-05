#pragma once
// 标定库：按 deviceKey 保存每台相机的标定，落盘为 JSON。换机、重启不丢。
// 三角化模块从这里取内外参。
//
// ---- 重插拔重映射（remap）----
// deviceKey 是 Windows 设备路径，含 USB 端口信息——同一台物理相机换个
// USB 口/重新插拔后，deviceKey 会变，导致按 key 精确查标定时对不上、
// 标定"看起来失效了"。这里加一层运行时重映射表 remap_:「当前实时相机的
// 新 key」->「模板里存的旧 key」。get() 查不到新 key 时，自动经 remap_
// 转成旧 key 再查一次。映射不写进标定数据本身（模板保持干净），单独
// 持久化到 remap 文件，这样手动映射一次后，下次插同样的口能自动记住。
#include "calib/Calibration.hpp"
#include <QHash>
#include <QString>

namespace mocap {

class CalibrationStore {
public:
    // 默认存到用户配置目录下 calibration.json。
    explicit CalibrationStore(const QString& path = QString());

    // has/get 都会先走重映射：如果 deviceKey 本身没有标定、但它被 remap
    // 到了某个有标定的旧 key，就用那条旧标定（deviceKey 字段替换成传入的
    // 新 key，这样三角化拿到的仍然是"这台实时相机"的标识，其余内外参用
    // 旧标定）。
    bool has(const QString& deviceKey) const;
    CameraCalibration get(const QString& deviceKey) const;
    void set(const CameraCalibration& c) { map_[c.deviceKey] = c; }
    void remove(const QString& deviceKey) { map_.remove(deviceKey); }
    QList<QString> keys() const { return map_.keys(); }
    int calibratedCount() const;

    // ---- 重映射接口 ----
    // 把「实时相机的新 key」映射到「标定库里已有的旧 key」。newKey==oldKey
    // 或 oldKey 为空时，视为清除这条映射（相当于取消重映射）。设置后立刻
    // 持久化。
    void setRemap(const QString& newKey, const QString& oldKey);
    void clearRemap(const QString& newKey);
    QString remapTarget(const QString& newKey) const { return remap_.value(newKey); }
    QHash<QString, QString> allRemaps() const { return remap_; }

    // 解析出 deviceKey 最终指向哪条标定的 key：有 remap 且目标有标定就返回
    // 目标旧 key；否则返回自身。三角化/查询内部都走这个。
    QString resolveKey(const QString& deviceKey) const;

    bool load();
    bool save() const;
    QString filePath() const { return path_; }

private:
    QString remapFilePath() const;   // remap 单独存一个文件，不混进 calibration.json
    bool loadRemap();
    bool saveRemap() const;

    QString path_;
    QHash<QString, CameraCalibration> map_;
    QHash<QString, QString> remap_;   // 新key -> 旧key
};

} // namespace mocap
