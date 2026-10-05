#pragma once
// 三角化实时调试用的简化 3D 视图：独立于 CalibResultView::PoseView3D，
// 不依赖它的私有实现，画世界坐标轴 + 两台相机位置 + 三角化出的点（带拖尾）。
#include <QWidget>
#include <QVector3D>
#include <QVector>
#include <array>

namespace mocap {

class TriangulationView : public QWidget {
    Q_OBJECT
public:
    struct CamPose { QString name; std::array<double,9> R; std::array<double,3> t; };

    explicit TriangulationView(QWidget* parent = nullptr);
    void setCameras(const QVector<CamPose>& cams);

public slots:
    void addPoint(QVector3D pos, double residualMm, qint64 ts_ns);
    // 持续质量（Triangulator 的滑动平均判断，不是单帧残差）：质量差时，
    // 点改成空心虚线画法，明确表示"这是数学上算出来的一个点，但残差
    // 持续偏大，物理上不可信"——数学唯一性≠物理正确性，画法上把这个
    // 区分体现出来，而不是不管可信不可信都用同一种实心圆表示。
    void setTrusted(bool trusted);

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;

private:
    QPointF project(double x, double y, double z) const;
    void fitScale();

    QVector<CamPose> cams_;
    QVector<QVector3D> trail_;   // 最近若干帧的三角化点，画拖尾
    double lastResidual_ = -1;
    bool trusted_ = true;
    double scale_ = 0.05;
    double yaw_ = 0.6, pitch_ = 0.35;
    QPoint lastPos_;
};

} // namespace mocap
