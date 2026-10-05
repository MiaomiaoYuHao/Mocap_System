#pragma once
// ---------------------------------------------------------------------------
// 相机显示名 —— 六台同型号相机全叫 "USB Camera" 的解法
//
// 【为什么名字全一样，这不是 bug】
// 界面上显示的名字来自 DShowControl::enumerateVideoDevices() 读的
// FriendlyName，而 FriendlyName 来自 USB 描述符里的 iProduct 字符串。
// 现场这六台是同型号（VID_0EDE / PID_2076），厂商没在描述符里写序列号，
// 六个 iProduct 一字不差 —— 所以 Windows 给出的 FriendlyName 必然全同。
// 换任何枚举方式都一样：Qt 的 QCameraDevice::description()、Media Foundation、
// SetupDiGetDeviceRegistryProperty(SPDRP_FRIENDLYNAME)，读的是同一个串。
//
// 【但唯一信息一直在手上】DevicePath 是唯一的，而且 deviceKey() 已经在用它。
// 真机六台的差别全在实例 ID 那一段：
//     \\?\usb#vid_0ede&pid_2076&mi_00#8&2b37306d&0&0000#{e5323777-...}
//     \\?\usb#vid_0ede&pid_2076&mi_00#8&120940a &0&0000#{...}
//     \\?\usb#vid_0ede&pid_2076&mi_00#7&2deeb752&1&0000#{...}
//                                    ^^^^^^^^^^
// 设备【没有序列号】时，Windows 拿父 Hub + 端口生成这个哈希，所以它等价于
// "插在哪个口上"：换口会变，不换口恒定。设备【有序列号】时这一段就是序列号
// 本身，那才是真正跟着物理相机走的标识。两种情况本文件都认，并用
// fromSerial 区分 —— 这个区别对重映射对话框有意义：
//     fromSerial=true   -> 换口也认得出是同一台，可以自动重映射
//     fromSerial=false  -> 换口就是新 key，必须让用户人工确认（现有流程）
//
// 【三档用法】
//   L0  只用 shortDeviceTag()：纯字符串，零依赖，任何平台都能跑，
//       立刻把六行 "USB Camera" 变成可区分的。
//   L1  加 queryUsbLocation()：查出 "Port_#0002.Hub_#0004"，
//       告诉用户这台插在几号口。仅 Windows，需要链 cfgmgr32。
//   L2  alias：CalibrationLibrary::renameCamera() 里已经有了，
//       makeCameraLabel() 接受它并优先显示。
//
// 【接入点 —— 两处必须传同一个编号，否则又会"两个界面两个名字"】
//   src/camera/WebcamCamera.cpp  相机对象的名字，全 UI 都从这里取：
//       : ICamera(id, makeCameraLabel(dev.description(), dev.id(),
//                                     AppSettings().cameraNumber(dev.id())).display, parent)
//     预览宫格、侧边栏相机列表、标定对话框与向导、同步监视器、参数对话框、
//     重映射对话框，查过一遍全部落到 ICamera::name()。
//   src/ui/AddCameraDialog.cpp   refresh() 里用【一模一样的形式】：
//       makeCameraLabel(d.desc, d.id, settings.cameraNumber(d.id)).display
//     编号两边都来自 AppSettings::cameraNumber()，而它拿 devicePathToInstanceId()
//     归一化后的整条实例 ID 查表 —— DShow 枚举和 Qt 枚举给的两种路径写法会
//     归一到同一个串，所以两边拿到的必然是同一个号。
//     （不用 shortDeviceTag()：那只是实例 ID 的最后一段哈希，同 Hub 下是否
//       必然不同没把握，撞了就是两台相机都显示 #1，而且看不出来。）
//     【绝不要在任一侧现算序号】"同名设备里的第几个"会随插拔重排，
//     "相机列表里的第几个"跟添加顺序走，两种都会让两个界面对不上号。
//   src/ui/DShowCaptureTestDialog.cpp  自己拼的 "(#N)" 是枚举序，同样会漂，
//       换成本文件的 label。
//
// 【occurrence 只能用来显示】它是"同名设备里的第几个"，重新插拔会重排。
// 任何持久化（标定库、remap 表、配置文件）都必须用 DevicePath / tag，
// 绝不能用序号 —— 那正是 CameraRemapDialog 那一整个对话框要解决的问题。
// ---------------------------------------------------------------------------
#include <QtGlobal>      // Q_OS_WIN —— 别依赖它从 QString 传递进来
#include <QByteArray>
#include <QString>
#include <QStringList>

namespace mocap {

struct CameraLabel {
    QString display;              // 界面上显示的整行，形如 "USB CAMERA #3"
    QString tag;                  // 短标识：序列号，或端口派生的哈希
    // 【makeCameraLabel() 不再填这一项，永远是空】显示名里已经不带位置串了
    // （七机位时每行长四十来个字符，标定向导底部那条汇总直接溢出窗口）。
    // 谁真需要物理位置，自己调 queryUsbLocation(devicePath) —— 那是一次
    // cfgmgr32 查询，不该让每次列表刷新都为它付钱（添加相机对话框 1Hz 轮询）。
    QString location;
    bool    fromSerial = false;   // true = tag 是真序列号，换口也不变
};

// ---------------------------------------------------------------------------
// DevicePath -> 设备实例 ID。纯字符串变换，不碰任何 API。
//
//   \\?\usb#vid_0ede&pid_2076&mi_00#8&2b37306d&0&0000#{e5323777-...}
//   -> USB\VID_0EDE&PID_2076&MI_00\8&2B37306D&0&0000
//
// 变换规则：去掉 \\?\ 前缀，去掉尾部的 #{接口GUID}，把剩下的 # 换成 \，
// 整体转大写（CM_Locate_DevNodeW 不区分大小写，但转了便于比对和打日志）。
// ---------------------------------------------------------------------------
inline QString devicePathToInstanceId(const QByteArray& devicePath) {
    QString s = QString::fromLatin1(devicePath).trimmed();
    if (s.isEmpty()) return {};
    // 上层可能带着 deviceKey 的 "web:" 前缀传进来，容忍掉
    if (s.startsWith(QLatin1String("web:"), Qt::CaseInsensitive)) s = s.mid(4);
    if (s.startsWith(QLatin1String("\\\\?\\"))) s = s.mid(4);
    else if (s.startsWith(QLatin1String("\\\\.\\"))) s = s.mid(4);
    // 尾部的 #{...} 是接口类 GUID，同类设备完全一样，不是标识的一部分
    const int h = s.lastIndexOf(QLatin1Char('#'));
    if (h >= 0 && h + 1 < s.size() && s.at(h + 1) == QLatin1Char('{')) s = s.left(h);
    // 有些路径尾部还挂着 \global 之类的接口名
    if (s.endsWith(QLatin1String("\\global"), Qt::CaseInsensitive)) s.chop(7);
    s.replace(QLatin1Char('#'), QLatin1Char('\\'));
    return s.toUpper();
}

// ---------------------------------------------------------------------------
// 实例 ID 的最后一段就是"这一台"的标识。两种形态：
//   有序列号:  USB\VID_x&PID_y\A1B2C3D4          -> 整段就是序列号
//   无序列号:  USB\VID_x&PID_y&MI_00\8&2b37306d&0&0000
//              -> 第二个字段 2b37306d 是父 Hub+端口派生的哈希
// 返回空表示这条路径分辨不出来（不该发生，但不猜）。
// ---------------------------------------------------------------------------
inline QString shortDeviceTag(const QByteArray& devicePath, bool* fromSerial = nullptr) {
    if (fromSerial) *fromSerial = false;
    const QString inst = devicePathToInstanceId(devicePath);
    // 【必须先确认这确实是设备实例 ID】形如 USB\VID_x&PID_y\<实例>，至少两段。
    // 少了这一刀，随便一个字符串（比如上层传错了个别名进来）都会被当成
    // "单字段 = 真序列号"，然后 fromSerial=true 骗重映射去做自动匹配。
    if (inst.count(QLatin1Char('\\')) < 1) return {};
    const QString last = inst.section(QLatin1Char('\\'), -1);
    if (last.isEmpty()) return {};
    if (!last.contains(QLatin1Char('&'))) {
        // 单字段 = 真序列号。这是最好的情况：换 USB 口也认得出同一台。
        if (fromSerial) *fromSerial = true;
        return last;
    }
    const QStringList f = last.split(QLatin1Char('&'), Qt::SkipEmptyParts);
    // f = ["8", "2B37306D", "0", "0000"]：取第二段那个哈希。
    // 它已经足够区分同一台机器上的所有口，不必把 &0&0000 也带上 ——
    // 那两段是端口序和复合设备的接口号，对用户没有信息量。
    if (f.size() >= 2) return f.at(1);
    return last;
}

// ---------------------------------------------------------------------------
// L1：查 USB 物理位置。仅 Windows。
//
// MI_00 那个 UVC 接口子节点通常【没有】LOCATION_INFORMATION，
// 挂着它的复合设备节点才有。所以先查自己，拿不到就上溯【一层】。
// 【只上溯一层】再往上就是 Hub 本身，它的位置对同一 Hub 上的所有相机
// 完全相同 —— 拿回来会让六台里的四台又变成一样，比不查还糟。
// 只接受 "Port_" 开头的串，就是为了防这一条。
//
// 构建：MinGW 加 -lcfgmgr32；MSVC 下面的 pragma 已经带上了。
// ---------------------------------------------------------------------------
#ifdef Q_OS_WIN
QString queryUsbLocation(const QByteArray& devicePath);
#else
inline QString queryUsbLocation(const QByteArray&) { return {}; }
#endif

// ---------------------------------------------------------------------------
// 组装最终显示名。
//
//   有别名:  左前机位
//   无别名:  USB CAMERA #3
//   没给编号: USB CAMERA
//
// 【为什么不再显示 [Port_#0002.Hub_#0004] / [2B37306D]】
// 它确实是唯一能区分同型号相机的信息，但对着七台相机的界面实测下来是负担：
// 每行长出四十来个字符，标定向导底部那条张数汇总直接横着溢出窗口右边，
// 而用户真正要的只是"这是几号机位"。位置串该出现的地方是【相机重映射
// 对话框】——那里就是专门回答"这个号现在对应哪台物理相机"的，不是每一行
// 都挂着。
//
// 【编号从哪来】AppSettings::cameraNumber(devicePath)，按 DevicePath 派生的
// 稳定 tag 持久化，重启/重新枚举/拖拽换位都不变。绝不要在这里现算序号 ——
// 那正是"添加对话框 #3、退出去没号"那个 bug 的成因。
//
// number <= 0 表示"不知道是几号"，那一段就不加。
// ---------------------------------------------------------------------------
inline CameraLabel makeCameraLabel(const QString& friendlyName,
                                   const QByteArray& devicePath,
                                   int number = -1,
                                   const QString& alias = QString()) {
    CameraLabel out;
    out.tag = shortDeviceTag(devicePath, &out.fromSerial);
    // location 不再默认查：它只给重映射对话框用，而那里会自己调
    // queryUsbLocation()。原来每次列表刷新都对每台相机查一次 cfgmgr32，
    // 添加相机对话框是 1Hz 轮询的，白烧。

    const bool named = !alias.trimmed().isEmpty();
    QString base = named ? alias.trimmed() : friendlyName.trimmed();
    if (base.isEmpty()) base = QString::fromUtf8("(未命名设备)");

    QString text = base;
    // 别名是用户自己起的，再加序号只是噪音
    if (!named && number > 0) text += QStringLiteral(" #%1").arg(number);

    out.display = text;
    return out;
}

}  // namespace mocap
