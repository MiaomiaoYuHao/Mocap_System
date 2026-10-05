# Xunbu Mocap System

Xunbu 是一套面向多相机光学动捕和手部跟踪的 Windows 上位机系统，覆盖相机采集、标定、二维检测、多视角三角化、时序跟踪、手部骨架估计、数据录制回放以及 Unity 接入。

> Main repository: [MiaomiaoYuHao/Mocap_System](https://github.com/MiaomiaoYuHao/Mocap_System)

## Demo

[观看 / 下载 Demo 视频](https://github.com/MiaomiaoYuHao/Mocap_System/blob/main/demo/Xunbu%20Demo%E6%9C%80%E7%BB%88%E7%89%88.mp4)

Demo 视频位于 `demo/`，使用 Git LFS 保存。

## 主要能力

- 多相机采集与同步，支持 DirectShow / UVC 相机和虚拟相机输入
- 标定板、相机内外参、连接关系与手背模板标定
- 离心/轮廓检测、多视角匹配、三角化、重投影检查和点云调试
- 点级 IEKF、时序关联、鬼点抑制和 One-Euro 滤波
- HM20 手部骨架关联、自动标定、IK 细化、关节角和分段四元数输出
- M3DS / M3DQ UDP 协议与 Unity 接收端
- `.pcrec` 全链路录制、离线重放、诊断和批量报告
- 参数面板的控件变化、起录/停录快照及事件时间轴记录

## 仓库结构

| 路径 | 内容 |
|---|---|
| `src/` | C++ / Qt6 上位机源码 |
| `tests/` | 单元测试与离线回归测试 |
| `tools/pcrec/` | `.pcrec` 录制格式、重放、诊断和报告工具 |
| `tools/unity/` | Unity 接收端和 UDP 数据监视器 |
| `tools/hand_diag/` | 手部跟踪诊断与回放程序 |
| `readme/` | 详细设计、审计、修复和交付文档 |
| `demo/` | Demo 视频 |
| `_backup_*/`、`_tmp_*` | 历史备份与临时对比源码 |

## 构建

系统使用 C++20 和 CMake 3.20 以上版本，核心界面依赖 Qt 6。

必需组件：

- Windows 10 / 11
- CMake 3.20+
- 支持 C++20 的 MSVC 或兼容编译器
- Qt 6：Widgets、Multimedia、Network、Concurrent

可选组件：

- libjpeg-turbo：启用 MJPG 灰度直解，不可用时自动回退到 Qt 解码
- ONNX Runtime：启用 HM20 骨架关联网络；没有模型时相关功能不可用

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
```

如果本机不需要可选依赖，可以关闭：

```powershell
cmake -S . -B build -DUSE_TURBOJPEG=OFF -DUSE_ONNXRUNTIME=OFF
cmake --build build --config Release --parallel
```

Visual Studio 生成器通常输出到：

```text
build\Release\mocap_host.exe
```

## 录制分析

```powershell
python tools\pcrec\pcrec.py 你的文件.pcrec --report
python tools\pcrec\pcrec.py 你的文件.pcrec --timeline
python tools\pcrec\pcrec.py 你的文件.pcrec --frozen
```

更多格式和工具说明见 `tools/pcrec/README.md`。

## Unity

将 `tools/unity/MocapHandReceiver.cs` 放入 Unity 工程，按 `tools/unity/README.md` 绑定 16 个手部骨骼。默认 UDP 端口为 `9010`。

## License

本项目原创代码和随仓库提供的 Demo 采用 [MIT License](LICENSE)。

第三方依赖（包括 Qt、libjpeg-turbo、ONNX Runtime 以及 Unity 等）继续遵循各自的许可证。
