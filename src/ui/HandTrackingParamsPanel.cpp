// 空实现文件：让 Qt 的 AUTOMOC 处理 HandTrackingParamsPanel.hpp 里的
// Q_OBJECT，生成 vtable/staticMetaObject 等符号（否则会出 undefined
// reference to vtable 链接错误）。这个类是纯头文件实现(没有配对的同名
// .cpp)，只被 HandPoseDebugDialog.cpp 间接 #include，AUTOMOC 对这种
// "孤儿" Q_OBJECT 头文件有时候扫描不到——项目里 ICamera.cpp 就是同一个
// 问题的先例，这里照抄同一个解法。
#include "ui/HandTrackingParamsPanel.hpp"
