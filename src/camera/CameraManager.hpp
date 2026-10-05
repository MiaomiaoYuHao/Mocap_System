#pragma once
// 相机管理器：增删相机、分配 id、拖拽换位、按 id 查找。
#include "camera/ICamera.hpp"
#include <QObject>
#include <QSet>
#include <QCameraDevice>
#include <vector>
#include <memory>

namespace mocap {

class CameraManager : public QObject {
    Q_OBJECT
public:
    explicit CameraManager(QObject* parent = nullptr) : QObject(parent) {}

    quint32 addVirtual(const CameraParams& defaults);
    quint32 addWebcam(const QCameraDevice& dev, const CameraParams& defaults);

    void removeLast();
    void removeById(quint32 id);
    void clear();

    // 拖拽换位：交换两台相机在网格中的顺序。
    void swapByIds(quint32 a, quint32 b);

    int count() const { return int(cams_.size()); }
    ICamera* at(int i) const { return cams_[size_t(i)].get(); }
    ICamera* byId(quint32 id) const;
    QSet<QString> usedDeviceKeys() const;   // 供“添加相机”对话框标记已添加设备

signals:
    void countChanged(int n);   // 数量或顺序变化（换位也发，触发网格重排）

private:
    std::vector<std::unique_ptr<ICamera>> cams_;
    quint32 nextId_ = 0;
};

} // namespace mocap
