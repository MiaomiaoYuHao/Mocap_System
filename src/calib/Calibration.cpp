#include "calib/Calibration.hpp"
#include <QJsonArray>
#include <cmath>

namespace mocap {

QJsonObject CameraIntrinsics::toJson() const {
    QJsonObject o;
    o["fx"] = fx; o["fy"] = fy; o["cx"] = cx; o["cy"] = cy;
    o["k1"] = k1; o["k2"] = k2; o["k3"] = k3; o["p1"] = p1; o["p2"] = p2;
    o["width"] = width; o["height"] = height; o["valid"] = valid;
    return o;
}

CameraIntrinsics CameraIntrinsics::fromJson(const QJsonObject& o) {
    CameraIntrinsics c;
    c.fx = o["fx"].toDouble(); c.fy = o["fy"].toDouble();
    c.cx = o["cx"].toDouble(); c.cy = o["cy"].toDouble();
    c.k1 = o["k1"].toDouble(); c.k2 = o["k2"].toDouble(); c.k3 = o["k3"].toDouble();
    c.p1 = o["p1"].toDouble(); c.p2 = o["p2"].toDouble();
    c.width = o["width"].toInt(); c.height = o["height"].toInt();
    c.valid = o["valid"].toBool();
    return c;
}

CameraIntrinsics CameraIntrinsics::scaledTo(int newWidth, int newHeight) const {
    CameraIntrinsics c = *this;
    if (valid && width > 0 && height > 0 && (newWidth != width || newHeight != height)) {
        const double sx = double(newWidth) / double(width);
        const double sy = double(newHeight) / double(height);
        c.fx = fx * sx; c.cx = cx * sx;
        c.fy = fy * sy; c.cy = cy * sy;
        c.width = newWidth; c.height = newHeight;
        // k1/k2/k3/p1/p2 无量纲，原样保留。
    }
    return c;
}

QJsonObject CameraExtrinsics::toJson() const {
    QJsonObject o;
    QJsonArray ra, ta;
    for (double v : R) ra.append(v);
    for (double v : t) ta.append(v);
    o["R"] = ra; o["t"] = ta; o["valid"] = valid;
    return o;
}

CameraExtrinsics CameraExtrinsics::fromJson(const QJsonObject& o) {
    CameraExtrinsics e;
    const QJsonArray ra = o["R"].toArray(), ta = o["t"].toArray();
    for (int i = 0; i < 9 && i < ra.size(); ++i) e.R[i] = ra[i].toDouble();
    for (int i = 0; i < 3 && i < ta.size(); ++i) e.t[i] = ta[i].toDouble();
    e.valid = o["valid"].toBool();

    // 校验 R 是否是合法旋转矩阵。手动录入/外部导入的JSON很容易出现不正交、
    // 甚至带镜像（反射）的矩阵，这种脏数据如果被当成真标定用，后面三角化会
    // 得到错误结果且很难排查——这里直接把它标记为未标定，界面老实显示
    // “未标定”而不是悄悄拿着错误外参继续跑。
    //
    // 行列式检查只能排除明显缩放/反射，排不掉带剪切(shear)的矩阵——
    // 一个非正交但行列式恰好=1的矩阵能轻易骗过旧检查。这里再加一个
    // R·Rᵀ ≈ I 的正交性检查，两者都过才认为是合法旋转矩阵。
    const double det = e.R[0]*(e.R[4]*e.R[8]-e.R[5]*e.R[7])
                       - e.R[1]*(e.R[3]*e.R[8]-e.R[5]*e.R[6])
                       + e.R[2]*(e.R[3]*e.R[7]-e.R[4]*e.R[6]);

    double orthoErr = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double dot = e.R[i*3+0]*e.R[j*3+0] + e.R[i*3+1]*e.R[j*3+1] + e.R[i*3+2]*e.R[j*3+2];
            const double target = (i == j) ? 1.0 : 0.0;
            orthoErr += (dot - target) * (dot - target);
        }
    }

    if (std::abs(det - 1.0) > 0.01 || orthoErr > 1e-4) e.valid = false;

    return e;
}

QJsonObject CameraCalibration::toJson() const {
    QJsonObject o;
    o["deviceKey"] = deviceKey;
    o["alias"] = alias;
    o["intrinsics"] = intr.toJson();
    o["extrinsics"] = extr.toJson();
    return o;
}

CameraCalibration CameraCalibration::fromJson(const QJsonObject& o) {
    CameraCalibration c;
    c.deviceKey = o["deviceKey"].toString();
    c.alias = o["alias"].toString();   // 老文件没这个字段时，QJsonValue 缺省是 QJsonValue()，toString() 得到空字符串，向后兼容
    c.intr = CameraIntrinsics::fromJson(o["intrinsics"].toObject());
    c.extr = CameraExtrinsics::fromJson(o["extrinsics"].toObject());
    return c;
}

} // namespace mocap