#pragma once
// ---------------------------------------------------------------------------
// 标定模板库：每次标定的结果落成独立的一个文件夹（"模板"），互不覆盖、
// 互不合并。用户可以给模板改名、切换"当前使用哪个模板"、删除模板里的
// 单台相机（比如相机坏了）、删除整个模板。
//
// 磁盘布局（<项目目录>/calib_templates/ 下）：
//   tmpl_20260711_220530123/
//     meta.json          { "id":..., "displayName":..., "createdAtMs":... }
//     calibration.json   跟 CalibrationStore 现有格式完全一样
//   active.txt            单行，记录当前"使用哪个模板"的模板id
//
// 存在项目目录而不是 <AppConfigLocation>，跟标定会话（calib_sessions/）保持
// 一致：标定结果是这个项目的一部分，应该跟着项目走（拷贝项目目录/换机器/
// 备份时自然带上），而不是散落在系统的用户配置目录里。
// "项目目录"是【运行时】定位出来的（见 core/ProjectPaths.hpp），不是编译期
// 焊死的路径——所以把整个项目文件夹改名、挪走、复制成新版本，程序都能自己
// 找到当前所在的那份 calib_templates，不需要重新跑 CMake 配置。
// 旧版本存在 <AppConfigLocation>/calib_templates 的模板，构造时会自动搬过来
// 一次（见 migrateFromLegacyRootIfNeeded）。
//
// 复用现有 CalibrationStore 类做单个模板内部的读写——不改 CalibrationStore
// 一行代码，本类只是在它外面加了一层"多个、可命名、可切换"的管理。
//
// 这个设计顺带彻底解决了"幽灵相机残留"的问题：每次标定都是全新空白模板，
// 不存在"这次标定的相机跟上次的旧记录怎么合并/覆盖"这回事——从根源上
// 没有旧记录可残留。
// ---------------------------------------------------------------------------
#include "calib/CalibrationStore.hpp"
#include "calib/Calibration.hpp"
#include <QString>
#include <QVector>

namespace mocap {

struct TemplateInfo {
    QString id;            // 文件夹名，创建时生成，不可改，保证唯一
    QString displayName;   // 用户可改的名字
    qint64  createdAtMs = 0;
};

class CalibrationLibrary {
public:
    CalibrationLibrary();   // 扫描 calib_templates/ 下已有的全部模板

    QVector<TemplateInfo> templates() const { return templates_; }
    TemplateInfo templateInfo(const QString& id) const;          // 找不到返回 id 为空
    QVector<CameraCalibration> templateCameras(const QString& id) const;
    int cameraCount(const QString& id) const;

    // 新建一个空模板（标定向导写入用），立刻在磁盘建好文件夹+meta.json。
    // 返回的 CalibrationStore* 调用方持有所有权（用完自行 delete，或用
    // std::unique_ptr 接住）。displayNameHint 为空时用当前时间生成默认名。
    // outId 非空时回填新模板的 id，方便调用方后续查询/操作这个模板
    // （比如向导被取消、这个模板最终一台相机都没有时，判断要不要清理）。
    CalibrationStore* beginNewTemplateStore(const QString& displayNameHint = QString(),
                                            QString* outId = nullptr);

    // 打开已有模板对应的 store，调用方持有所有权。
    CalibrationStore* openTemplateStore(const QString& templateId) const;

    bool renameTemplate(const QString& id, const QString& newName);

    // 给模板里某台相机设置自定义别名（"左相机"这种），空字符串表示清除别名、
    // 回退显示自动提取的短标签。
    bool renameCamera(const QString& templateId, const QString& deviceKey, const QString& alias);

    // 删掉"一台相机都没有、且不是当前使用中"的模板——用于清理"标定向导
    // 被取消/没跑完求解"留下的空壳模板，构造时自动调用一次，不需要手动。
    void pruneEmptyTemplates();

    // 真删除整个模板文件夹。配合撤销功能使用前，请先用 templateInfo()/
    // templateCameras() 读一份快照，删除后如需撤销再调 restoreTemplate()。
    bool deleteTemplate(const QString& id);
    bool restoreTemplate(const TemplateInfo& meta, const QVector<CameraCalibration>& cams);

    // 同理，删除/恢复模板内某一台相机的标定，也是"先读后删"配合撤销。
    bool removeCameraFromTemplate(const QString& id, const QString& deviceKey);
    bool addCameraToTemplate(const QString& id, const CameraCalibration& cam);

    QString activeTemplateId() const { return activeId_; }
    void setActiveTemplateId(const QString& id);   // 落盘持久化

private:
    QString rootDir() const;
    // 旧版存储位置（<AppConfigLocation>/calib_templates）。保留只为了做一次性
    // 迁移——升级前标定好的模板都在那儿，不迁移的话用户升级后会发现模板
    // "凭空消失"（其实文件还在，只是新版不去那个目录找了）。
    QString legacyRootDir() const;
    // 把 legacyRootDir() 下的模板搬到 rootDir()，构造时自动调用一次。
    // 同名模板已存在时跳过（不覆盖新目录里的数据），搬完保留旧目录不删，
    // 让用户有反悔余地——磁盘上多一份几KB的json，比误删标定结果安全得多。
    void migrateFromLegacyRootIfNeeded();
    QString templateDir(const QString& id) const;
    void loadTemplateList();
    void loadActiveId();
    void saveActiveId() const;
    bool writeMeta(const TemplateInfo& info) const;

    QVector<TemplateInfo> templates_;
    QString activeId_;
};

} // namespace mocap
