// ---------------------------------------------------------------------------
// test_camera_label.cpp —— DevicePath -> 稳定短标识 的变换规则
//
// 【只测字符串那半边】queryUsbLocation() 要碰 Windows 配置管理器，非 Windows
// 下头文件里是个返回空的 inline stub。所以这个测试跨平台都能跑，验的是变换
// 规则本身 —— 而规则出错的表现是"六台相机里有两台标识相同"，那正是这套东西
// 存在的全部理由，必须有一条自动化判据钉住。
//
// 样本是真机那六条 DevicePath，逐字取自 pcrec 头部的 cameras 字段。
// 手动跑：
//   g++ -std=c++17 -I src $(pkg-config --cflags --libs Qt6Core) \
//       tests/test_camera_label.cpp src/camera/CameraLabel.cpp -o /tmp/t && /tmp/t
// 判据是【退出码】，跟工程里其余 TU 一致。
// ---------------------------------------------------------------------------
#include "camera/CameraLabel.hpp"

#include <QSet>
#include <QString>

#include <cstdio>

using namespace mocap;

static int g_fail = 0;

static void check(bool ok, const char* what) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_fail;
}

// 真机六台，同型号（VID_0EDE / PID_2076），厂商没写序列号 —— FriendlyName
// 六个一字不差，唯一的区别就在实例 ID 那一段。
static const char* kPaths[6] = {
    "\\\\?\\usb#vid_0ede&pid_2076&mi_00#8&2b37306d&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\\global",
    "\\\\?\\usb#vid_0ede&pid_2076&mi_00#8&120940a&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\\global",
    "\\\\?\\usb#vid_0ede&pid_2076&mi_00#8&11991c9b&1&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\\global",
    "\\\\?\\usb#vid_0ede&pid_2076&mi_00#7&2deeb752&1&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\\global",
    "\\\\?\\usb#vid_0ede&pid_2076&mi_00#7&17c74e95&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\\global",
    "\\\\?\\usb#vid_0ede&pid_2076&mi_00#8&2452cd40&0&0000#{e5323777-f976-4f5b-9b55-b94699c46e44}\\global",
};

int main() {
    std::printf("== 实例 ID 变换 ==\n");
    const QString inst = devicePathToInstanceId(QByteArray(kPaths[0]));
    std::printf("  %s\n", inst.toUtf8().constData());
    check(inst == QStringLiteral("USB\\VID_0EDE&PID_2076&MI_00\\8&2B37306D&0&0000"),
          "去 \\\\?\\ 前缀、去 #{GUID} 尾、去 \\global、# 换 \\、转大写");

    std::printf("\n== 六台可区分（这条挂了就说明规则白写）==\n");
    QSet<QString> tags;
    for (int i = 0; i < 6; ++i) {
        const CameraLabel L = makeCameraLabel(QStringLiteral("USB Camera"),
                                              QByteArray(kPaths[i]), i);
        std::printf("  %-34s tag=%s\n", L.display.toUtf8().constData(),
                    L.tag.toUtf8().constData());
        tags.insert(L.tag);
    }
    check(tags.size() == 6, "六条 DevicePath 派生出六个互不相同的 tag");

    std::printf("\n== fromSerial 的两种形态 ==\n");
    // 【这个标志有实际用途，不是装饰】它决定 CameraRemapDialog 敢不敢自动匹配：
    //   true  = tag 是真序列号，换 USB 口也认得出同一台，可以自动重映射
    //   false = tag 由父 Hub+端口派生，换口就是新 key，必须人工确认
    // 所以误判成 true 的代价是"自动把标定绑到错的相机上"，必须钉住。
    bool ser = false;
    const QString noSerial = shortDeviceTag(QByteArray(kPaths[0]), &ser);
    check(noSerial == QStringLiteral("2B37306D") && !ser,
          "无序列号设备：取实例 ID 第二字段的哈希，fromSerial=false");

    ser = false;
    const QString withSerial =
        shortDeviceTag(QByteArray("\\\\?\\usb#vid_046d&pid_0825#A1B2C3D4#{guid}"), &ser);
    check(withSerial == QStringLiteral("A1B2C3D4") && ser,
          "有序列号设备：整段就是序列号，fromSerial=true");

    std::printf("\n== 垃圾输入不能被当成序列号 ==\n");
    // 上层传错了个别名/空串进来时，绝不能因为"它只有一个字段"就判成真序列号。
    for (const char* junk : {"", "garbage", "USB Camera", "web:"}) {
        ser = true;
        const QString t = shortDeviceTag(QByteArray(junk), &ser);
        std::printf("  %-12s -> tag=%-8s serial=%d\n", junk[0] ? junk : "(空)",
                    t.isEmpty() ? "(空)" : t.toUtf8().constData(), int(ser));
        check(t.isEmpty() && !ser, "非设备路径返回空且 fromSerial=false");
    }

    std::printf("\n== 前缀容错 / 别名 ==\n");
    check(shortDeviceTag(QByteArray("web:") + kPaths[0]) == QStringLiteral("2B37306D"),
          "带 deviceKey 的 web: 前缀也能解析");
    const QString aliased = makeCameraLabel(QStringLiteral("USB Camera"),
                                            QByteArray(kPaths[3]), 3,
                                            QString::fromUtf8("左前机位")).display;
    std::printf("  %s\n", aliased.toUtf8().constData());
    check(aliased.startsWith(QString::fromUtf8("左前机位")) &&
              !aliased.contains(QStringLiteral("#4")),
          "有别名时优先显示别名，且不再叠加序号");

    std::printf("\n%s  失败 %d 条\n", g_fail ? "==> 不通过" : "==> 全部通过", g_fail);
    return g_fail ? 1 : 0;
}
