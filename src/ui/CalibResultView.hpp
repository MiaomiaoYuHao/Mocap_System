#pragma once
// ---------------------------------------------------------------------------
// 标定结果查看器：
//   左侧：相机列表；
//   右上：数学化表述 —— 内参矩阵 K、畸变向量、外参 [R|t]、相机中心 C = -Rᵀt；
//   右下：3D 位姿可视化 —— 世界坐标轴 + 地面网格 + 各相机视锥，鼠标拖拽旋转。
// ---------------------------------------------------------------------------
#include <QDialog>
#include <QWidget>
#include <QVector>
#include <QString>
#include <array>

class QListWidget;
class QPlainTextEdit;

namespace mocap {

class CalibrationStore;
struct CameraCalibration;

// 简易 3D 位姿视图：正交投影 + 偏航/俯仰拖拽，纯 QPainter 无 OpenGL。
class PoseView3D : public QWidget {
    Q_OBJECT
public:
    struct Cam { QString name; std::array<double,9> R; std::array<double,3> t; };
    explicit PoseView3D(QWidget* parent = nullptr);
    void setCameras(const QVector<Cam>& cams);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;

private:
    QVector<Cam> cams_;
    double yaw_ = 0.7, pitch_ = 0.5;
    QPoint lastPos_;
    double scale_ = 1.0;   // 自动适配

    QPointF project(double x, double y, double z) const;
    void fitScale();
};

class CalibResultView : public QDialog {
    Q_OBJECT
public:
    explicit CalibResultView(CalibrationStore* store, QWidget* parent = nullptr);

private:
    CalibrationStore* store_;
    QListWidget*    list_;
    QPlainTextEdit* text_;
    PoseView3D*     pose_;
    QStringList     keys_;

    void showCamera(int row);
    static QString formatMath(const CameraCalibration& c);
};

} // namespace mocap
