#pragma once
// 应用层相机参数（“开机默认参数”的载体，由 AppSettings 持久化）。
// 注意：真实 USB 摄像头的硬件参数（亮度/曝光/白平衡…）走 DirectShow
// 属性接口（见 DShowControl / CamParamDialog），与 AMCap 调的是同一套。
#include <QString>

namespace mocap {

struct CameraParams {
    int    target_fps  = 30;     // 虚拟相机产帧率；真相机由“格式”页决定
    int    threshold   = 40;     // 2D 检测阈值（预留给后续质心检测叠加）
    bool   show_overlay = true;  // 预留：预览是否叠加检测点
};

} // namespace mocap
