#include "settings/AppSettings.hpp"
#include "camera/CameraLabel.hpp"   // shortDeviceTag()：编号的键要跟标签用同一套归一化
#include <QSet>

namespace mocap {

namespace {
// QSettings 用'/'做层级分隔符，deviceKey(尤其Windows设备路径)里可能带
// '/'、'\'、':'这些字符，直接拿去拼组名会被误解析成好几层——统一替换掉，
// 保证不管deviceKey长什么样都能对应到唯一一个安全的组名。
QString sanitizeDeviceKey(const QString& deviceKey) {
    QString s = deviceKey;
    s.replace(QLatin1Char('/'), QLatin1Char('_'));
    s.replace(QLatin1Char('\\'), QLatin1Char('_'));
    s.replace(QLatin1Char(':'), QLatin1Char('_'));
    return s;
}
} // namespace

AppSettings::AppSettings()
    : s_("MocapLab", "MocapHost") {}

AppConfig AppSettings::load() {
    AppConfig c;
    c.defaults.target_fps   = s_.value("cam/target_fps",  c.defaults.target_fps).toInt();
    c.defaults.threshold    = s_.value("cam/threshold",   c.defaults.threshold).toInt();
    c.defaults.show_overlay = s_.value("cam/show_overlay",c.defaults.show_overlay).toBool();
    c.autoStartCameras      = s_.value("app/auto_cameras",c.autoStartCameras).toInt();
    c.cameraKeys            = s_.value("app/camera_keys").toStringList();
    c.windowGeometry        = s_.value("app/geometry").toByteArray();
    return c;
}

void AppSettings::save(const AppConfig& c) {
    s_.setValue("cam/target_fps",   c.defaults.target_fps);
    s_.setValue("cam/threshold",    c.defaults.threshold);
    s_.setValue("cam/show_overlay", c.defaults.show_overlay);
    s_.setValue("app/auto_cameras", c.autoStartCameras);
    s_.setValue("app/camera_keys",  c.cameraKeys);
    s_.setValue("app/geometry",     c.windowGeometry);
    s_.sync();
}

CameraFormatExposurePrefs AppSettings::loadCameraPrefs(const QString& deviceKey) {
    CameraFormatExposurePrefs p;
    const QString g = QStringLiteral("cam_prefs/%1").arg(sanitizeDeviceKey(deviceKey));
    p.hasFormat    = s_.value(g + "/has_format", false).toBool();
    p.resolution   = QSize(s_.value(g + "/res_w", 0).toInt(), s_.value(g + "/res_h", 0).toInt());
    p.pixelFormat  = s_.value(g + "/pixel_format", -1).toInt();
    p.frameRate    = s_.value(g + "/frame_rate", 0.0).toDouble();
    // 【默认 0 = 老配置的行为】升级前存的偏好没有这个键，读出来是 0，
    // 也就是"按标称上限跑"，跟升级前一模一样，不需要迁移。
    p.targetFps    = s_.value(g + "/target_fps", 0.0).toDouble();
    p.hasExposure  = s_.value(g + "/has_exposure", false).toBool();
    p.exposureAuto = s_.value(g + "/exposure_auto", true).toBool();
    p.exposureValue = long(s_.value(g + "/exposure_value", 0).toLongLong());
    return p;
}

void AppSettings::saveCameraFormat(const QString& deviceKey, const QSize& resolution,
                                   int pixelFormat, double frameRate, double targetFps) {
    const QString g = QStringLiteral("cam_prefs/%1").arg(sanitizeDeviceKey(deviceKey));
    s_.setValue(g + "/has_format",   true);
    s_.setValue(g + "/res_w",       resolution.width());
    s_.setValue(g + "/res_h",       resolution.height());
    s_.setValue(g + "/pixel_format", pixelFormat);
    s_.setValue(g + "/frame_rate",   frameRate);
    s_.setValue(g + "/target_fps",   targetFps);
    s_.sync();
}

void AppSettings::saveCameraExposure(const QString& deviceKey, bool isAuto, long value) {
    const QString g = QStringLiteral("cam_prefs/%1").arg(sanitizeDeviceKey(deviceKey));
    s_.setValue(g + "/has_exposure",   true);
    s_.setValue(g + "/exposure_auto",  isAuto);
    s_.setValue(g + "/exposure_value", qlonglong(value));
    s_.sync();
}

int AppSettings::cameraNumber(const QByteArray& devicePath) {
    // 【键用 devicePathToInstanceId()，不是 shortDeviceTag()】两点考虑：
    //
    // 1) 必须能吸收两套枚举的写法差异。DirectShow 的 DevicePath 和 Qt 的
    //    QCameraDevice::id() 是同一条内核符号链接的两种写法（大小写、尾部
    //    \global、接口 GUID 都可能不同）。直接拿路径当键，同一台相机会分到
    //    两个号 —— 而"添加相机对话框走 DShow 枚举、相机对象走 Qt 枚举"
    //    恰恰就是这两条路。devicePathToInstanceId() 把两种写法归一化成同一个
    //    串（截 #{guid}、砍 \global、'#'→'\'、转大写），四种写法实测收敛。
    //
    // 2) 【为什么不用 shortDeviceTag()】它只取实例 ID 的最后一段哈希，比如
    //    8&2B37306D&0&0000 里的 2B37306D。那一段是父节点派生的，在这套
    //    七机位、两个 Hub 的现场，同一 Hub 下的设备是否必然不同没有把握 ——
    //    一旦相同，两台相机会拿到同一个号，而且是【静默】的：界面上就是两行
    //    "USB CAMERA #1"，看不出谁是谁，比原来显示端口串还糟。
    //    整条实例 ID 是 Windows 保证唯一的，不用赌。
    //    （tag 仍然有用，那是重映射对话框判断"换口了还是换机器了"的依据，
    //      跟"给用户一个能念的号"是两件事。）
    QString key = sanitizeDeviceKey(devicePathToInstanceId(devicePath));
    if (key.isEmpty()) {
        // 兜底：路径形态不认识时退回整条原始路径。分不到稳定键也好过不给号，
        // 但这种设备换个写法枚举出来会另占一个号 —— 不该发生，也不假装没事。
        key = sanitizeDeviceKey(QString::fromLatin1(devicePath));
        if (key.isEmpty()) return -1;
    }

    s_.beginGroup(QStringLiteral("cam_number"));
    int n = s_.value(key, 0).toInt();
    if (n <= 0) {
        // 分配最小可用编号。【不是"已有数量+1"】—— 那样删掉中间一台再加回来
        // 会撞号，两台相机同时显示 #3。
        QSet<int> used;
        for (const QString& k : s_.childKeys()) {
            const int v = s_.value(k, 0).toInt();
            if (v > 0) used.insert(v);
        }
        n = 1;
        while (used.contains(n)) ++n;
        s_.setValue(key, n);
        s_.sync();
    }
    s_.endGroup();
    return n;
}

} // namespace mocap
