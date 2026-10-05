#include "camera/CameraManager.hpp"
#include "camera/VirtualCamera.hpp"
#include "camera/WebcamCamera.hpp"
#include "settings/AppSettings.hpp"
#include <algorithm>

namespace mocap {

quint32 CameraManager::addVirtual(const CameraParams& defaults) {
    quint32 id = nextId_++;
    auto cam = std::make_unique<VirtualCamera>(
        id, QString::fromUtf8("\u865a\u62df Cam %1").arg(id), this);  // 虚拟 Cam N
    cam->applyParams(defaults);
    cam->start();
    cams_.push_back(std::move(cam));
    emit countChanged(count());
    return id;
}

quint32 CameraManager::addWebcam(const QCameraDevice& dev, const CameraParams& defaults) {
    quint32 id = nextId_++;
    auto cam = std::make_unique<WebcamCamera>(id, dev, this);
    cam->applyParams(defaults);
    cam->start();

    // 【只恢复格式，硬件参数不在这里】格式属于采集引擎的事（分辨率/帧率决定
    // 建图怎么建），归 CameraManager 管；亮度/曝光那一堆 DirectShow 属性属于
    // "开机默认"这条策略，归 MainWindow 管 —— 见 MainWindow::hookCamera() 里
    // 接 WebcamCamera::captureEngineStarted 的那段。
    //
    // 【为什么必须分开，不能图省事都塞这里】硬件参数有两个来源、有先后：
    // Boot 模板（显式快照）先套，AppSettings 里那份曝光（改一下就自动存的，
    // 总是更新）后套覆盖。而连接的触发顺序就是 connect 的调用顺序 ——
    // 如果这里连一个、MainWindow 再连一个，这里的必然先跑，
    // 顺序就反了：显式快照会盖掉用户后来改的曝光。统一在一处连，
    // 顺序就是那一处代码的书写顺序，看得见、改得动。
    {
        AppSettings settings;
        const auto prefs = settings.loadCameraPrefs(cam->deviceKey());
        if (prefs.hasFormat) {
            for (const QCameraFormat& f : cam->formats()) {
                if (f.resolution() == prefs.resolution &&
                    int(f.pixelFormat()) == prefs.pixelFormat &&
                    qFuzzyCompare(f.maxFrameRate() + 1.0f, float(prefs.frameRate) + 1.0f)) {
                    // 【第二个实参别漏】targetFps 是"区间内指定的帧率"，
                    // 不传的话开机恢复出来的永远是标称上限 —— 用户上次特意
                    // 调成 15fps，重启又变回 30，而且预览上看不出是哪一步丢的。
                    cam->setFormat(f, prefs.targetFps);
                    break;
                }
            }
            // 找不到匹配项(比如换了台不同型号的相机、但deviceKey凑巧撞上了)
            // 就什么都不做，维持 startCapture() 里 bestFormat() 选出来的
            // 默认格式——不强行套用一个跟当前设备对不上的保存值。
        }
    }

    cams_.push_back(std::move(cam));
    emit countChanged(count());
    return id;
}

void CameraManager::removeLast() {
    if (cams_.empty()) return;
    cams_.back()->stop();
    cams_.pop_back();
    emit countChanged(count());
}

void CameraManager::removeById(quint32 id) {
    for (auto it = cams_.begin(); it != cams_.end(); ++it) {
        if ((*it)->id() == id) {
            (*it)->stop();
            cams_.erase(it);
            emit countChanged(count());
            return;
        }
    }
}

void CameraManager::clear() {
    for (auto& c : cams_) c->stop();
    cams_.clear();
    emit countChanged(count());
}

void CameraManager::swapByIds(quint32 a, quint32 b) {
    int ia = -1, ib = -1;
    for (size_t i = 0; i < cams_.size(); ++i) {
        if (cams_[i]->id() == a) ia = int(i);
        if (cams_[i]->id() == b) ib = int(i);
    }
    if (ia < 0 || ib < 0 || ia == ib) return;
    std::swap(cams_[size_t(ia)], cams_[size_t(ib)]);
    emit countChanged(count());
}

ICamera* CameraManager::byId(quint32 id) const {
    for (const auto& c : cams_)
        if (c->id() == id) return c.get();
    return nullptr;
}

QSet<QString> CameraManager::usedDeviceKeys() const {
    QSet<QString> s;
    for (const auto& c : cams_) s.insert(c->deviceKey());
    return s;
}

} // namespace mocap
