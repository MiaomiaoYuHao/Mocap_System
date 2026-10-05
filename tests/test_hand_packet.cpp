// ===========================================================================
// test_hand_packet.cpp —— M3DS / M3DQ 字节布局的"黄金测试"
// ===========================================================================
// 【这个测试存在的唯一理由】
// 打包代码从 UdpSender.cpp 里搬到了 net/HandPacket.hpp（因为点云面板的输出
// 监视器要打印"真正发出去的字节"，两边必须共用同一份实现）。搬家本身是机械
// 操作，但它动的是【已经有下游在用】的线上格式 —— Unity 端一个字节都不能错位。
//
// 所以这里把搬家【之前】那份手写打包代码原样抄一份当参照物，逐字节比对。
// 以后谁再改 HandPacket.hpp，只要动了布局，这个测试立刻红。
//
// 不依赖 onnxruntime，只需要 Qt6::Core。
// ===========================================================================
#include "net/HandPacket.hpp"

#include <QByteArray>
#include <QDataStream>
#include <QVector>
#include <QVector3D>
#include <cmath>
#include <cstdio>
#include <cstring>

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("  FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); } \
} while (0)

// --------------------------------------------------------------------------
// 参照实现：UdpSender.cpp 重构【之前】的原始代码，逐行照抄，不要"顺手优化"
// --------------------------------------------------------------------------
static QByteArray refM3DS(const QVector3D& wristPos, const QVector<double>& wristRot9,
                          const QVector<double>& jointAngles16, qint64 ts_ns) {
    if (wristRot9.size() != 9 || jointAngles16.size() != 16) return QByteArray();
    QByteArray pkt;
    pkt.reserve(4 + 8 + 3*4 + 9*4 + 16*4);
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

static QByteArray refM3DQ(const QVector3D& wristPos, const QVector<double>& quatWorld64,
                          const QVector<double>& quatLocal64, const QVector<int>& segSource16,
                          quint32 flags, qint64 ts_ns) {
    if (quatWorld64.size() != 64 || quatLocal64.size() != 64 || segSource16.size() != 16)
        return QByteArray();
    QByteArray pkt;
    pkt.reserve(4 + 8 + 4 + 3*4 + 64*4 + 64*4 + 16);
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

int main() {
    using namespace mocap;

    // ---- 构造一组不对称的测试数据 -----------------------------------------
    // 【故意全部取不同的值】如果某两个字段被写反了，用一堆 0/1 或者对称数据
    // 是看不出来的 —— 那正是"错位类" bug 最容易溜过测试的方式。
    const QVector3D pos(123.5f, -47.25f, 806.125f);
    QVector<double> rot9, q16, w64, l64;
    QVector<int> src16;
    for (int i = 0; i < 9;  ++i) rot9.push_back(0.1 * i - 0.35);
    for (int i = 0; i < 16; ++i) q16.push_back(std::sin(0.37 * i) * 1.1);
    for (int i = 0; i < 64; ++i) w64.push_back(std::cos(0.11 * i));
    for (int i = 0; i < 64; ++i) l64.push_back(std::sin(0.13 * i));
    for (int i = 0; i < 16; ++i) src16.push_back(i % 4);
    const qint64 ts = 1723459200123456789LL;
    const quint32 flags = 0x1F03u;

    std::printf("== 长度契约 ==\n");
    const QByteArray a = wire::buildM3DS(pos, rot9, q16, ts);
    const QByteArray b = wire::buildM3DQ(pos, w64, l64, src16, flags, ts);
    CHECK(a.size() == wire::kM3dsSize, "M3DS 长度 == kM3dsSize");
    CHECK(a.size() == 124, "M3DS 是 124 字节");
    CHECK(b.size() == wire::kM3dqSize, "M3DQ 长度 == kM3dqSize");
    CHECK(b.size() == 556, "M3DQ 是 556 字节");

    std::printf("== 与重构前的实现逐字节一致 ==\n");
    CHECK(a == refM3DS(pos, rot9, q16, ts), "M3DS 字节完全一致");
    CHECK(b == refM3DQ(pos, w64, l64, src16, flags, ts), "M3DQ 字节完全一致");

    std::printf("== magic 与关键字段位置 ==\n");
    CHECK(a.left(4) == QByteArray("M3DS"), "M3DS magic 在 +0");
    CHECK(b.left(4) == QByteArray("M3DQ"), "M3DQ magic 在 +0");
    {
        // 小端 i64 时间戳应该原样落在 +4
        qint64 got = 0;
        std::memcpy(&got, a.constData() + 4, 8);
        CHECK(got == ts, "M3DS ts_ns 在 +4，小端");
        std::memcpy(&got, b.constData() + 4, 8);
        CHECK(got == ts, "M3DQ ts_ns 在 +4，小端");
        quint32 gf = 0;
        std::memcpy(&gf, b.constData() + 12, 4);
        CHECK(gf == flags, "M3DQ flags 在 +12");
        float fx = 0;
        std::memcpy(&fx, a.constData() + 12, 4);
        CHECK(std::fabs(double(fx) - 123.5) < 1e-6, "M3DS wristPos.x 在 +12");
        // segSource 是最后 16 个字节，逐个 u8
        for (int i = 0; i < 16; ++i)
            CHECK(quint8(b[b.size() - 16 + i]) == quint8(i % 4), "M3DQ segSource 落在包尾");
    }

    std::printf("== 形状不对时返回空（调用方据此整帧不发）==\n");
    CHECK(wire::buildM3DS(pos, QVector<double>{1,2,3}, q16, ts).isEmpty(), "rot9 长度错 -> 空");
    CHECK(wire::buildM3DS(pos, rot9, QVector<double>{}, ts).isEmpty(), "q16 长度错 -> 空");
    CHECK(wire::buildM3DQ(pos, w64, l64, QVector<int>{1,2}, flags, ts).isEmpty(), "src16 长度错 -> 空");
    {
        QVector<double> shortW = w64; shortW.pop_back();
        CHECK(wire::buildM3DQ(pos, shortW, l64, src16, flags, ts).isEmpty(), "qW 长度错 -> 空");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
