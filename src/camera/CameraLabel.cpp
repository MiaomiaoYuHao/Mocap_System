// ---------------------------------------------------------------------------
// CameraLabel.cpp —— queryUsbLocation() 的 Windows 实现（唯一需要编译的部分）
//
// 头文件里的 devicePathToInstanceId / shortDeviceTag 是纯字符串，任何平台
// 都能用、也能单测。这里只放要碰 Windows 配置管理器的那一小段。
//
// 【链接】MinGW: 在 CMakeLists 里给这个 target 加 cfgmgr32
//                target_link_libraries(<target> PRIVATE cfgmgr32)
//         MSVC : 下面的 pragma 已经带上，不用改构建。
// ---------------------------------------------------------------------------
#include "camera/CameraLabel.hpp"

#ifdef Q_OS_WIN

#include <windows.h>
#include <cfgmgr32.h>

#include <string>

#ifdef _MSC_VER
#pragma comment(lib, "cfgmgr32.lib")
#endif

namespace mocap {
namespace {

// 读一个设备节点的 LOCATION_INFORMATION。读不到或不是 "Port_..." 就返回空。
//
// 【为什么只认 Port_ 开头】不同层级的节点在这个属性里塞的东西完全不同：
// 复合设备节点给 "Port_#0002.Hub_#0004"（正是我们要的），
// 而 Hub 节点给的是它自己的位置 —— 同一 Hub 上所有相机拿到的是同一个串，
// 用了等于白改。宁可返回空退回哈希，也不要一个看起来对、实际分不开的值。
QString readLocation(DEVINST dn) {
    wchar_t buf[512];
    ULONG len = sizeof(buf);
    ULONG type = 0;
    if (CM_Get_DevNode_Registry_PropertyW(dn, CM_DRP_LOCATION_INFORMATION,
                                          &type, buf, &len, 0) != CR_SUCCESS)
        return {};
    if (len < sizeof(wchar_t)) return {};
    buf[(len / sizeof(wchar_t)) - 1] = L'\0';   // 防驱动没给结尾 0
    const QString s = QString::fromWCharArray(buf).trimmed();
    if (!s.startsWith(QLatin1String("Port_"), Qt::CaseInsensitive)) return {};
    return s;
}

// 兜底：有些驱动不填 LOCATION_INFORMATION，但 CM_DRP_ADDRESS 里有端口号。
// 它只有端口没有 Hub，同一台机器上多个 Hub 时可能撞号 —— 所以是兜底，
// 而且标成 "Port_#N?" 让人一眼看出这是次一等的信息。
QString readAddressAsPort(DEVINST dn) {
    DWORD addr = 0;
    ULONG len = sizeof(addr);
    ULONG type = 0;
    if (CM_Get_DevNode_Registry_PropertyW(dn, CM_DRP_ADDRESS, &type, &addr, &len, 0) != CR_SUCCESS)
        return {};
    if (len != sizeof(addr) || addr == 0) return {};
    return QStringLiteral("Port_#%1?").arg(addr, 4, 10, QLatin1Char('0'));
}

}  // namespace

QString queryUsbLocation(const QByteArray& devicePath) {
    const QString inst = devicePathToInstanceId(devicePath);
    if (inst.isEmpty()) return {};

    std::wstring w = inst.toStdWString();
    DEVINST dn = 0;
    if (CM_Locate_DevNodeW(&dn, const_cast<wchar_t*>(w.c_str()),
                           CM_LOCATE_DEVNODE_NORMAL) != CR_SUCCESS)
        return {};

    // ① 先问这个节点自己（MI_00 的 UVC 接口，通常没有）
    QString s = readLocation(dn);
    if (!s.isEmpty()) return s;

    // ② 上溯【一层】到复合设备节点。只走一层，理由见 readLocation 的说明。
    DEVINST parent = 0;
    if (CM_Get_Parent(&parent, dn, 0) != CR_SUCCESS) return {};
    s = readLocation(parent);
    if (!s.isEmpty()) return s;

    // ③ 兜底：端口号
    s = readAddressAsPort(parent);
    if (!s.isEmpty()) return s;
    return readAddressAsPort(dn);
}

}  // namespace mocap

#endif  // Q_OS_WIN
