#pragma once
// ---------------------------------------------------------------------------
// HandPacket.hpp —— M3DS / M3DQ 两个手部输出包的【唯一】字节布局定义
//
// 【为什么要单独拆出来】
// 原来打包逻辑只写在 UdpSender.cpp 的两个槽函数里。现在点云面板要把"发出去
// 的数据"原样打印/dump 成十六进制给人核对，如果监视器自己再写一份打包代码，
// 就有两份布局定义 —— 哪天改了包格式忘记同步另一份，监视器显示的字节和真正
// 发出去的字节不一致，而这种不一致恰恰只会在"你正拿它排查问题"的时候骗你，
// 是最坏的一类 bug。
//
// 所以：UdpSender 发的字节和监视器打印的字节，来自同一个函数的同一次调用路径。
// 打印的就是发出去的，这是这个文件存在的全部理由。
//
// 【长度校验放在这里】形状不对(rot9 不是9个、quat 不是64个…)返回空 QByteArray，
// 调用方判空即可 —— 语义跟原来"宁可丢一帧也不发半个包"完全一致：下游收到长度
// 不符的 payload 会解析错位，错位之后每个字段都是错的，比缺一帧难查得多。
//
// 【字节序】全小端、float32。跟 Unity(x86/ARM 小端)直接对得上，不需要转换。
// ---------------------------------------------------------------------------
#include <QByteArray>
#include <QDataStream>
#include <QIODevice>
#include <QVector>
#include <QVector3D>

namespace mocap {
namespace wire {

// 定长包，长度写死在这里当契约用：接收端可以先按长度粗筛，再看 magic。
inline constexpr int kM3dsSize = 4 + 8 + 3 * 4 + 9 * 4 + 16 * 4;              // 124
inline constexpr int kM3dqSize = 4 + 8 + 4 + 3 * 4 + 64 * 4 + 64 * 4 + 16;    // 556

// ---------------------------------------------------------------------------
// M3DS：腕部位姿 + 16 维关节角（弧度）
//   magic "M3DS"(4) | ts_ns i64 | wristPos 3×f32(mm) | wristRot9 9×f32(行主序)
//   | jointAngles16 16×f32(rad)
// ---------------------------------------------------------------------------
inline QByteArray buildM3DS(const QVector3D& wristPos,
                            const QVector<double>& wristRot9,
                            const QVector<double>& jointAngles16,
                            qint64 ts_ns) {
    if (wristRot9.size() != 9 || jointAngles16.size() != 16) return QByteArray();

    QByteArray pkt;
    pkt.reserve(kM3dsSize);
    QDataStream ds(&pkt, QIODevice::WriteOnly);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setFloatingPointPrecision(QDataStream::SinglePrecision);

    ds.writeRawData("M3DS", 4);
    ds << qint64(ts_ns);
    ds << float(wristPos.x()) << float(wristPos.y()) << float(wristPos.z());
    for (int i = 0; i < 9; ++i) ds << float(wristRot9[i]);
    for (int i = 0; i < 16; ++i) ds << float(jointAngles16[i]);
    return pkt;
}

// ---------------------------------------------------------------------------
// M3DQ：16 个分段四元数（世界系 + 相对父节点）+ 每段来源
//   magic "M3DQ"(4) | ts_ns i64 | flags u32 | wristPos 3×f32(mm)
//   | segQuatWorld 64×f32 (w,x,y,z)×16 | segQuatLocal 64×f32 | segSource 16×u8
//
// flags: bit0 wristPoseValid, bit1 mcpValid, bit8..12 五指关节角本帧有效
// segSource: 0=None 1=Predicted 2=Geometry 3=IK —— 下游必须看，Predicted 是
// 外推不是测量。
// ---------------------------------------------------------------------------
inline QByteArray buildM3DQ(const QVector3D& wristPos,
                            const QVector<double>& quatWorld64,
                            const QVector<double>& quatLocal64,
                            const QVector<int>& segSource16,
                            quint32 flags, qint64 ts_ns) {
    if (quatWorld64.size() != 64 || quatLocal64.size() != 64 || segSource16.size() != 16)
        return QByteArray();

    QByteArray pkt;
    pkt.reserve(kM3dqSize);
    QDataStream ds(&pkt, QIODevice::WriteOnly);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setFloatingPointPrecision(QDataStream::SinglePrecision);

    ds.writeRawData("M3DQ", 4);
    ds << qint64(ts_ns) << quint32(flags);
    ds << float(wristPos.x()) << float(wristPos.y()) << float(wristPos.z());
    for (int i = 0; i < 64; ++i) ds << float(quatWorld64[i]);
    for (int i = 0; i < 64; ++i) ds << float(quatLocal64[i]);
    for (int i = 0; i < 16; ++i) ds << quint8(segSource16[i] & 0xFF);
    return pkt;
}

} // namespace wire
} // namespace mocap
