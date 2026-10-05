#include "ui/CalibResultView.hpp"
#include "ui/Theme.hpp"
#include "calib/CalibrationStore.hpp"
#include "settings/AppSettings.hpp"   // cameraNumber()：跟全 UI 共用的那张机位编号表

#include <QListWidget>
#include <QPlainTextEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QPainter>
#include <QMouseEvent>
#include <QLabel>
#include <QRegularExpression>
#include <cmath>
#include <algorithm>

namespace mocap {

namespace {
// deviceKey 是 Windows 设备路径（形如
// "web:\\?\usb#vid_0ede&pid_2076&mi_00#8&2b37306d&0&0000#{...}\global"），
// 本来就是拿来保证唯一性用的，不能随便改短——两台同型号相机全靠这串区分，
// 改了标定库/参数模板存取全部对不上号。这里只是显示层面提取一个人看得懂
// 的短标签，deviceKey 本身在别处一个字节都不动。
// 【2026-09 改：编号改用全局那张表，不再自己数】原来的 dedupeIndex 是
// "这份标定结果里同型号的第几台"，从 0 数起再 +1。它跟界面上到处显示的机位号
// 【不是一回事】：标定结果里只涉及 3 台同型号相机时，这里的 "#2" 完全可能是
// 机位 "#5"。两个都写作 #2 却指不同的相机，比干脆没有编号更容易看错。
// 现在直接问 AppSettings::cameraNumber() 要号 —— 它按归一化实例 ID 查表，
// deviceKey 的 "web:" 前缀 devicePathToInstanceId() 自己会剥掉，所以这里
// 跟 ICamera::name() 拿到的必然是同一个号。
QString shortDeviceLabel(const QString& deviceKey) {
    static const QRegularExpression reVid("vid_([0-9a-fA-F]{4})");
    static const QRegularExpression rePid("pid_([0-9a-fA-F]{4})");
    const auto mv = reVid.match(deviceKey);
    const auto mp = rePid.match(deviceKey);
    if (!mv.hasMatch() || !mp.hasMatch())
        return deviceKey;   // 解析不出 VID/PID（虚拟相机等）就老实显示原始值，
                            // 也不给它编号 —— 那张表只管真实 USB 设备。
    QString label = QStringLiteral("USB相机 %1:%2")
                        .arg(mv.captured(1).toUpper(), mp.captured(1).toUpper());
    const int n = AppSettings().cameraNumber(deviceKey.toLatin1());
    if (n > 0) label += QStringLiteral(" #%1").arg(n);
    return label;
}
} // namespace

// ============================ PoseView3D ============================
PoseView3D::PoseView3D(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(260);
    setMouseTracking(false);
}

void PoseView3D::setCameras(const QVector<Cam>& cams) {
    cams_ = cams;
    fitScale();
    update();
}

void PoseView3D::fitScale() {
    // 按相机中心分布自动适配缩放：C = -Rᵀ t
    double maxR = 100.0;   // 至少留 100mm 视野
    for (const Cam& c : cams_) {
        const auto& R = c.R; const auto& t = c.t;
        const double cx = -(R[0]*t[0] + R[3]*t[1] + R[6]*t[2]);
        const double cy = -(R[1]*t[0] + R[4]*t[1] + R[7]*t[2]);
        const double cz = -(R[2]*t[0] + R[5]*t[1] + R[8]*t[2]);
        maxR = std::max({maxR, std::abs(cx), std::abs(cy), std::abs(cz)});
    }
    scale_ = 130.0 / maxR;   // 130px 对应最远相机
}

QPointF PoseView3D::project(double x, double y, double z) const {
    // 偏航（绕 Z）+ 俯仰（绕 X'），再正交投影到屏幕。Z 朝上的右手系画法。
    const double cy_ = std::cos(yaw_),  sy_ = std::sin(yaw_);
    const double cp_ = std::cos(pitch_), sp_ = std::sin(pitch_);
    const double x1 =  cy_ * x + sy_ * y;
    const double y1 = -sy_ * x + cy_ * y;
    const double z1 = z;
    const double y2 = cp_ * y1 - sp_ * z1;   // 深度（不用）
    const double z2 = sp_ * y1 + cp_ * z1;
    Q_UNUSED(y2);
    return QPointF(width() / 2.0 + x1 * scale_,
                   height() / 2.0 + 40 - z2 * scale_);
}

void PoseView3D::mousePressEvent(QMouseEvent* e) { lastPos_ = e->pos(); }
void PoseView3D::mouseMoveEvent(QMouseEvent* e) {
    if (!(e->buttons() & Qt::LeftButton)) return;
    const QPoint d = e->pos() - lastPos_;
    lastPos_ = e->pos();
    yaw_   += d.x() * 0.01;
    pitch_ = std::clamp(pitch_ + d.y() * 0.01, -1.4, 1.4);
    update();
}

void PoseView3D::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), theme::surface());
    p.setPen(QPen(theme::line(), 1));
    p.drawRoundedRect(rect().adjusted(0, 0, -1, -1), 2, 2);   // 直角，跟全局一致

    // 地面网格（世界 XY 平面，Z 朝上）
    const double g = 100.0;   // 100mm 网格
    p.setPen(QPen(theme::surfaceAlt(), 1));
    for (int i = -3; i <= 3; ++i) {
        p.drawLine(project(i * g, -3 * g, 0), project(i * g, 3 * g, 0));
        p.drawLine(project(-3 * g, i * g, 0), project(3 * g, i * g, 0));
    }

    // 世界坐标轴
    auto axis = [&](double x, double y, double z, const QColor& col, const QString& lab) {
        p.setPen(QPen(col, 2));
        const QPointF o = project(0, 0, 0), e = project(x, y, z);
        p.drawLine(o, e);
        p.drawText(e + QPointF(4, -4), lab);
    };
    axis(150, 0, 0, QColor(0xD6, 0x45, 0x45), "X");
    axis(0, 150, 0, QColor(0x1E, 0xA9, 0x5A), "Y");
    axis(0, 0, 150, QColor(0x2B, 0x6C, 0xD4), "Z");

    // 相机：中心 C = -Rᵀt，视锥四角沿 Rᵀ 的相机系方向张开。
    for (const Cam& c : cams_) {
        const auto& R = c.R; const auto& t = c.t;
        auto RtDir = [&](double a, double b, double d) {
            // Rᵀ * (a,b,d)：把相机系方向转到世界系。
            return std::array<double,3>{
                R[0]*a + R[3]*b + R[6]*d,
                R[1]*a + R[4]*b + R[7]*d,
                R[2]*a + R[5]*b + R[8]*d };
        };
        const double Cx = -(R[0]*t[0] + R[3]*t[1] + R[6]*t[2]);
        const double Cy = -(R[1]*t[0] + R[4]*t[1] + R[7]*t[2]);
        const double Cz = -(R[2]*t[0] + R[5]*t[1] + R[8]*t[2]);

        const double L = 60.0;   // 视锥长度（mm 表意）
        const QPointF apex = project(Cx, Cy, Cz);
        QVector<QPointF> corners;
        const double cs[4][2] = {{-0.5,-0.35},{0.5,-0.35},{0.5,0.35},{-0.5,0.35}};
        for (auto& k : cs) {
            const auto d = RtDir(k[0], k[1], 1.0);
            corners << project(Cx + d[0]*L, Cy + d[1]*L, Cz + d[2]*L);
        }
        p.setPen(QPen(theme::accent(), 1.6));
        for (int i = 0; i < 4; ++i) {
            p.drawLine(apex, corners[i]);
            p.drawLine(corners[i], corners[(i + 1) % 4]);
        }
        p.setBrush(theme::accent());
        p.drawEllipse(apex, 3.5, 3.5);
        p.setPen(theme::text());
        p.drawText(apex + QPointF(6, -6), c.name);
    }

    p.setPen(theme::textDim());
    p.drawText(rect().adjusted(10, 6, -10, -6), Qt::AlignBottom | Qt::AlignLeft,
               QStringLiteral("拖动旋转视角 · 网格 100mm · 视锥朝向为相机光轴"));
}

// ============================ CalibResultView ============================
CalibResultView::CalibResultView(CalibrationStore* store, QWidget* parent)
    : QDialog(parent), store_(store) {
    setWindowTitle(QStringLiteral("标定结果"));
    resize(920, 640);

    auto* root = new QHBoxLayout(this);
    list_ = new QListWidget;
    list_->setFixedWidth(220);
    root->addWidget(list_);

    auto* right = new QVBoxLayout;
    text_ = new QPlainTextEdit;
    text_->setReadOnly(true);
    text_->setStyleSheet(theme::monoCss(12.5));
    right->addWidget(text_, 3);
    pose_ = new PoseView3D;
    right->addWidget(pose_, 2);
    root->addLayout(right, 1);

    // 载入
    keys_ = store_->keys();

    // 【原来这里有一段"撞名了再补 #1/#2"的去重】现在不需要了：
    // shortDeviceLabel() 拿的是 AppSettings::cameraNumber() 那个全局唯一的
    // 机位号，同型号相机天然不会撞。而且那段去重补的序号是"这份标定结果里
    // 的第几台"，跟界面上到处显示的机位号不是一回事 —— 两个都写作 #2 却指
    // 不同相机，比没有编号更容易看错，所以是连同去重一起删掉，不是简化。
    QStringList shortLabels(keys_.size());
    for (int i = 0; i < keys_.size(); ++i) shortLabels[i] = shortDeviceLabel(keys_[i]);

    QVector<PoseView3D::Cam> pcams;
    for (int i = 0; i < keys_.size(); ++i) {
        const QString& k = keys_[i];
        const CameraCalibration c = store_->get(k);
        list_->addItem(QString("%1  %2")
            .arg(c.isCalibrated() ? QStringLiteral("●") : QStringLiteral("○"), shortLabels[i]));
        if (c.extr.valid)
            pcams << PoseView3D::Cam{ shortLabels[i], c.extr.R, c.extr.t };
    }
    pose_->setCameras(pcams);

    connect(list_, &QListWidget::currentRowChanged, this, [this](int r) { showCamera(r); });
    if (!keys_.isEmpty()) list_->setCurrentRow(0);
    else text_->setPlainText(QStringLiteral("标定库为空。先跑一遍标定向导。"));
}

void CalibResultView::showCamera(int row) {
    if (row < 0 || row >= keys_.size()) return;
    text_->setPlainText(formatMath(store_->get(keys_[row])));
}

QString CalibResultView::formatMath(const CameraCalibration& c) {
    auto n = [](double v) { return QString::number(v, 'f', 3).rightJustified(10); };
    QString s;
    s += QStringLiteral("设备: %1\n").arg(shortDeviceLabel(c.deviceKey));
    s += QStringLiteral("（完整标识: %1）\n\n").arg(c.deviceKey);

    if (c.intr.valid) {
        s += QStringLiteral("── 内参（针孔模型，单位：像素） ──\n");
        s += QStringLiteral("      ⎡ %1  %2  %3 ⎤\n").arg(n(c.intr.fx), n(0.0), n(c.intr.cx));
        s += QStringLiteral("  K = ⎢ %1  %2  %3 ⎥\n").arg(n(0.0), n(c.intr.fy), n(c.intr.cy));
        s += QStringLiteral("      ⎣ %1  %2  %3 ⎦\n\n").arg(n(0.0), n(0.0), n(1.0));
        s += QStringLiteral("  畸变 (k1,k2,p1,p2,k3) = (%1, %2, %3, %4, %5)\n")
                 .arg(c.intr.k1, 0, 'f', 5).arg(c.intr.k2, 0, 'f', 5)
                 .arg(c.intr.p1, 0, 'f', 5).arg(c.intr.p2, 0, 'f', 5)
                 .arg(c.intr.k3, 0, 'f', 5);
        s += QStringLiteral("  标定分辨率 = %1 × %2\n\n").arg(c.intr.width).arg(c.intr.height);
    } else {
        s += QStringLiteral("── 内参：未标定 ──\n\n");
    }

    if (c.extr.valid) {
        const auto& R = c.extr.R; const auto& t = c.extr.t;
        s += QStringLiteral("── 外参（世界 → 相机：Xc = R·Xw + t，单位：mm） ──\n");
        for (int r = 0; r < 3; ++r)
            s += QStringLiteral("  %1 %2  %3  %4 %5   %6 %7 %8\n")
                     .arg(r == 1 ? "R =" : "   ")
                     .arg(r == 0 ? QStringLiteral("⎡") : r == 1 ? QStringLiteral("⎢") : QStringLiteral("⎣"),
                          n(R[r*3]), n(R[r*3+1]), n(R[r*3+2]))
                     .arg(r == 0 ? QStringLiteral("⎤") : r == 1 ? QStringLiteral("⎥") : QStringLiteral("⎦"))
                     .arg(r == 1 ? QStringLiteral("t = ") : QStringLiteral("    "))
                     .arg(r == 1 ? n(t[0]) + n(t[1]) + n(t[2]) : QString());
        const double Cx = -(R[0]*t[0] + R[3]*t[1] + R[6]*t[2]);
        const double Cy = -(R[1]*t[0] + R[4]*t[1] + R[7]*t[2]);
        const double Cz = -(R[2]*t[0] + R[5]*t[1] + R[8]*t[2]);
        s += QStringLiteral("\n  相机中心（世界系）C = -Rᵀt = (%1, %2, %3) mm\n")
                 .arg(Cx, 0, 'f', 1).arg(Cy, 0, 'f', 1).arg(Cz, 0, 'f', 1);
    } else {
        s += QStringLiteral("── 外参：未标定 ──\n");
    }
    return s;
}

} // namespace mocap