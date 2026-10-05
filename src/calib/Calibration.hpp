#pragma once
// ---------------------------------------------------------------------------
// 相机标定数据（为多相机三角化预留）。约定与 OpenCV 一致，方便用标准标定工具
// （棋盘格/ChArUco）算出参数后直接导入。
//
//   内参 Intrinsics：针孔模型 fx,fy,cx,cy + 畸变 k1,k2,p1,p2,k3
//     把 3D 相机坐标投影到像素；标定畸变后做去畸变，直接影响 2D 点精度。
//   外参 Extrinsics：该相机在世界坐标系下的旋转 R(3x3) + 平移 t(3)
//     多相机三角化必须知道彼此相对位姿；世界系一般取某台“主相机”或标定板。
//
// 这一步只定义数据与序列化。三角化/标定算法是后续阶段的活，接口先留好。
// ---------------------------------------------------------------------------
#include <QString>
#include <QJsonObject>
#include <array>

namespace mocap {

    struct CameraIntrinsics {
        double fx = 0, fy = 0;      // 焦距（像素）
        double cx = 0, cy = 0;      // 主点（像素）
        double k1 = 0, k2 = 0, k3 = 0;   // 径向畸变
        double p1 = 0, p2 = 0;           // 切向畸变
        int    width = 0, height = 0;    // 标定时的分辨率（换分辨率需按比例缩放内参）
        bool   valid = false;            // 是否已标定

        QJsonObject toJson() const;
        static CameraIntrinsics fromJson(const QJsonObject&);

        // 把标定分辨率下的内参按比例缩放到运行时实际分辨率。畸变系数(k1/k2/k3/p1/p2)
        // 是无量纲的，不随分辨率变化，不缩放。三角化/去畸变前必须过这一步，否则
        // 换了分辨率（比如相机重新插拔选到了不同格式）内参会悄悄错位且不报错。
        CameraIntrinsics scaledTo(int newWidth, int newHeight) const;
    };

    struct CameraExtrinsics {
        // 世界->相机 的旋转（行主序 3x3）与平移（相机在世界系的位置由 -R^T t 得到）。
        std::array<double, 9> R = { 1,0,0, 0,1,0, 0,0,1 };
        std::array<double, 3> t = { 0, 0, 0 };
        bool valid = false;

        QJsonObject toJson() const;
        static CameraExtrinsics fromJson(const QJsonObject&);
    };

    // 一台相机的完整标定。deviceKey 用来和实际设备关联（换机不丢标定）。
    struct CameraCalibration {
        QString deviceKey;
        QString alias;   // 用户自定义别名（如"左相机"），空字符串表示未设置，
                          // 界面上应回退显示按 VID/PID 提取的短标签。向后兼容：
                          // 老的 calibration.json 里没有这个字段，反序列化时
                          // 直接得到空字符串，不影响老数据读取。
        CameraIntrinsics intr;
        CameraExtrinsics extr;

        bool isCalibrated() const { return intr.valid && extr.valid; }

        QJsonObject toJson() const;
        static CameraCalibration fromJson(const QJsonObject&);
    };

} // namespace mocap