#pragma once
// 添加相机对话框：列出系统里的全部视频设备，已添加的置灰；底部附“虚拟测试相机”。
// 列表每秒自动刷新，插拔即时可见。
//
// 【为什么不用 QMediaDevices 列设备】
// Qt 的 QMediaDevices::videoInputs() 在 Windows 上会把每个 moniker 绑成
// IBaseFilter、再查 IAMStreamConfig 拿支持的格式列表 —— 等于【逐个打开摄像头】。
// 有相机正在独占推流时那既慢又容易撞上。这里改用 DShowControl 提供的
// enumerateVideoDevices()，只读 moniker 的属性包（DevicePath / FriendlyName），
// 不碰设备本身，所以可以每秒轮询、不需要任何设备到达通知。
//
// 【为什么改成非模态】真机上出现过：对话框开着时插 USB 相机 -> 整个系统永久
// 无响应；对话框关着插则正常；插非相机 USB 也正常。实测把对话框的枚举整个
// 关掉（诊断开关）仍然卡，说明跟“我们枚不枚举”无关，剩下的唯一差别就是
// exec() 的模态循环。模态时 Qt 会禁用其它顶层窗口并在事件分发层拦截消息，
// 而系统的设备到达通知是同步发送的 —— 发送方等不到返回就一直挂着，
// 连带堵住整条 PnP 通知链，于是“整个系统”都无响应。
//
// 所以这里不再用 exec()，改成非模态 + accepted 信号回调。
//
// 【批量添加】接 6 台相机要开 6 次对话框、每次都要重新找设备，真机上很烦。
// 现在列表可批量选中（点一下选中、再点一下取消，不用按 Ctrl/Shift），
// 一次确定把选中的全部加进来。
#include <QDialog>
#include <QCameraDevice>
#include <QByteArray>
#include <QSet>
#include <QList>
#include <QString>

class QListWidget;
class QTimer;

namespace mocap {

class AddCameraDialog : public QDialog {
    Q_OBJECT
public:
    // 用户这一次确认要加入的一项。虚拟测试相机没有 QCameraDevice，
    // 用 isVirtual 区分；不要拿 device.isNull() 去判断，
    // 解析失败的实机也会是 null（那种情况我们不放进结果里）。
    struct Selection {
        bool         isVirtual = false;
        QCameraDevice device;
    };

    explicit AddCameraDialog(const QSet<QString>& usedKeys, QWidget* parent = nullptr);

    // 【只在 accepted 之后读】各项是在用户点确定的那一刻才去解析的，
    // 那一刻不会跟设备到达并发。顺序与列表里的选中顺序一致。
    QList<Selection> selection() const { return selection_; }

private slots:
    void refresh();
    void onAccept();

private:
    // 只存轻量标识，绝不留 QCameraDevice 对象 —— 那些对象在 Windows 后端
    // 关联着设备资源，热插拔期间一直握着不是好主意。
    // occ = 同名设备里的第几个（从 0 数）。
    //
    // 【为什么必须有它】三台一模一样的 USB 相机，description() 是同一个字符串。
    // 点确定时要把它解析成 QCameraDevice，如果路径匹配不上就退回按名字找 ——
    // 那样永远选中第一台，于是第二、三台拿到的其实是第一台，
    // 而它已经被占用，表现就是"选了相机显示无信号，只有第一个能用"。
    //
    // 项目里 DShowControl::findMoniker 早就有这套"同名取第 N 个"的机制，
    // 我在对话框里绕过了它。
    struct DevEntry { QByteArray id; QString desc; int occ = 0; };

    // 判断某个枚举到的设备是不是已经被添加过。不能直接比字符串 ——
    // 见 .cpp 里的说明（两套枚举给的 id 不保证逐字节相同）。
    bool isUsed(const DevEntry& d) const;

    // 把一个 DevEntry 解析成可以交给 WebcamCamera 的 QCameraDevice。
    // 原来这段"三梯匹配"内联在 onAccept() 里；批量添加要对每一项都做一次，
    // 抽出来避免复制三份。解析失败返回 false（设备被拔了 / 两套枚举对不上）。
    bool resolveDevice(const DevEntry& de, QCameraDevice* out) const;

    QListWidget* list_;
    QTimer*      poll_ = nullptr;
    QSet<QString> used_;
    QList<DevEntry> devs_;
    // 上一次列表的指纹：只有真的变了才重建列表，否则每秒重建会把
    // 用户正在做的选择清掉。
    QString sig_;
    // 用户点确定那一刻解析出来的结果（可能是多台）。
    QList<Selection> selection_;
};

} // namespace mocap
