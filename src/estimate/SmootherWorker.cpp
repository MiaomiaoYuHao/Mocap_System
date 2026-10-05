// 空实现文件：让 Qt 的 AUTOMOC 处理 SmootherWorker.hpp 里的 Q_OBJECT，
// 生成 vtable/staticMetaObject 等符号（否则会出 undefined reference to
// vtable 链接错误）。这个类是纯头文件实现，只被 HandTrackingWorker.cpp
// 间接 #include，AUTOMOC 对这种"孤儿" Q_OBJECT 头文件有时候扫描不到——
// 跟 ICamera.cpp / HandTrackingParamsPanel.cpp 是同一个问题、同一个解法。
#include "estimate/SmootherWorker.hpp"
