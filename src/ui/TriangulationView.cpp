#include "ui/TriangulationView.hpp"
#include <QPainter>
#include <QMouseEvent>
#include <cmath>
#include <algorithm>

namespace mocap {

TriangulationView::TriangulationView(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(360);
}

void TriangulationView::setCameras(const QVector<CamPose>& cams) {
    cams_ = cams;
    fitScale();
    update();
}

void TriangulationView::fitScale() {
    double maxR = 300.0;   // 至少留 300mm 视野
    for (const auto& c : cams_) {
        const auto& R = c.R; const auto& t = c.t;
        const double cx = -(R[0]*t[0] + R[3]*t[1] + R[6]*t[2]);
        const double cy = -(R[1]*t[0] + R[4]*t[1] + R[7]*t[2]);
        const double cz = -(R[2]*t[0] + R[5]*t[1] + R[8]*t[2]);
        maxR = std::max({maxR, std::abs(cx), std::abs(cy), std::abs(cz)});
    }
    scale_ = 150.0 / maxR;
}

QPointF TriangulationView::project(double x, double y, double z) const {
    const double cy_ = std::cos(yaw_), sy_ = std::sin(yaw_);
    const double cp_ = std::cos(pitch_), sp_ = std::sin(pitch_);
    const double x1 = cy_*x + sy_*y;
    const double y1 = -sy_*x + cy_*y;
    const double z2 = sp_*y1 + cp_*z;
    return QPointF(width()/2.0 + x1*scale_, height()/2.0 + 40 - z2*scale_);
}

void TriangulationView::mousePressEvent(QMouseEvent* e) { lastPos_ = e->pos(); }
void TriangulationView::mouseMoveEvent(QMouseEvent* e) {
    if (!(e->buttons() & Qt::LeftButton)) return;
    const QPoint d = e->pos() - lastPos_;
    lastPos_ = e->pos();
    yaw_ += d.x() * 0.01;
    pitch_ = std::clamp(pitch_ + d.y() * 0.01, -1.4, 1.4);
    update();
}

void TriangulationView::addPoint(QVector3D pos, double residualMm, qint64) {
    trail_.push_back(pos);
    if (trail_.size() > 60) trail_.pop_front();   // 约2秒拖尾（30fps下）
    lastResidual_ = residualMm;
    update();
}

void TriangulationView::setTrusted(bool trusted) {
    if (trusted_ == trusted) return;
    trusted_ = trusted;
    update();
}

void TriangulationView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0x14, 0x14, 0x14));

    // 地面网格（世界XY平面）
    const double g = 200.0;
    p.setPen(QPen(QColor(255,255,255,25), 1));
    for (int i = -4; i <= 4; ++i) {
        p.drawLine(project(i*g, -4*g, 0), project(i*g, 4*g, 0));
        p.drawLine(project(-4*g, i*g, 0), project(4*g, i*g, 0));
    }

    // 世界坐标轴
    auto axis = [&](double x, double y, double z, const QColor& col, const QString& lab) {
        p.setPen(QPen(col, 2));
        const QPointF o = project(0,0,0), e = project(x,y,z);
        p.drawLine(o, e);
        p.setPen(QColor(220,220,220));
        p.drawText(e + QPointF(4,-4), lab);
    };
    axis(300,0,0, QColor(0xD6,0x45,0x45), "X");
    axis(0,300,0, QColor(0x1E,0xA9,0x5A), "Y");
    axis(0,0,300, QColor(0x2B,0x6C,0xD4), "Z");

    // 相机位置（C = -R^T t）
    for (const auto& c : cams_) {
        const auto& R = c.R; const auto& t = c.t;
        const double cx = -(R[0]*t[0] + R[3]*t[1] + R[6]*t[2]);
        const double cy = -(R[1]*t[0] + R[4]*t[1] + R[7]*t[2]);
        const double cz = -(R[2]*t[0] + R[5]*t[1] + R[8]*t[2]);
        const QPointF pt = project(cx, cy, cz);
        p.setBrush(QColor(0x4A,0x8F,0xD6)); p.setPen(Qt::NoPen);
        p.drawEllipse(pt, 5, 5);
        p.setPen(QColor(200,200,200));
        p.drawText(pt + QPointF(8,-8), c.name);
    }

    // 拖尾：质量差时颜色更暗，弱化视觉存在感（提示"这段轨迹参考价值有限"）。
    if (trail_.size() > 1) {
        for (int i = 1; i < trail_.size(); ++i) {
            const double baseAlpha = trusted_ ? (40 + 180.0 * i / trail_.size())
                                              : (20 + 60.0 * i / trail_.size());
            p.setPen(QPen(QColor(0x1D,0x9E,0x75, int(baseAlpha)), 1.5,
                         trusted_ ? Qt::SolidLine : Qt::DashLine));
            p.drawLine(project(trail_[i-1].x(), trail_[i-1].y(), trail_[i-1].z()),
                       project(trail_[i].x(), trail_[i].y(), trail_[i].z()));
        }
    }

    // 当前点：可信时按 residual 单帧分级实心画（<5mm绿，<20mm黄，否则红）；
    // 不可信时（Triangulator 判定滑动平均残差持续偏大）统一改成空心虚线圆，
    // 明确跟"实心=可信"区分开——这是数学唯一解不等于物理正确解的具体体现，
    // 不能靠同一种画法糊弄过去。
    if (!trail_.isEmpty()) {
        const auto& last = trail_.back();
        const QPointF pt = project(last.x(), last.y(), last.z());
        if (trusted_) {
            QColor col = QColor(0x1E,0xA9,0x5A);
            if (lastResidual_ > 20) col = QColor(0xD6,0x45,0x45);
            else if (lastResidual_ > 5) col = QColor(0xE0,0xA0,0x30);
            p.setBrush(col); p.setPen(Qt::NoPen);
            p.drawEllipse(pt, 7, 7);
        } else {
            p.setBrush(Qt::NoBrush);
            p.setPen(QPen(QColor(0xD6,0x45,0x45), 2, Qt::DashLine));
            p.drawEllipse(pt, 9, 9);
            p.setPen(QColor(0xF0,0xB0,0xB0));
            p.drawText(pt + QPointF(12, 4), QStringLiteral("不可信"));
        }
    }

    p.setPen(QColor(180,180,180));
    p.drawText(rect().adjusted(10,6,-10,-6), Qt::AlignBottom|Qt::AlignLeft,
               QStringLiteral("拖动旋转视角 · 网格200mm · 绿/黄/红=残差从小到大"));
}

} // namespace mocap
