#pragma once
// 网格容器：按 gridDimFor 正方形规则摆放面板；支持换位与单路放大(solo)。
#include "ui/CameraView.hpp"
#include <QWidget>
#include <QGridLayout>
#include <vector>

namespace mocap {

class CameraManager;
class ICamera;

class CameraGrid : public QWidget {
    Q_OBJECT
public:
    explicit CameraGrid(CameraManager* mgr, QWidget* parent = nullptr);

public slots:
    void rebuild();

signals:
    void paramsRequested(quint32 id);   // 转发给 MainWindow 开参数对话框
    void removeRequested(quint32 id);   // 转发给 MainWindow 执行移除

public:
private:
    CameraManager* mgr_;
    QGridLayout*   grid_;
    std::vector<CameraView*> views_;
    qint64 soloId_ = -1;                // >=0 表示放大显示该相机
    // 空态占位（"请添加相机"）。一台相机都没有时才铺在宫格上，有相机就摘掉。
    QWidget* emptyHint_ = nullptr;

    CameraView* makeView(ICamera* cam);
    void clearViews();
};

} // namespace mocap
