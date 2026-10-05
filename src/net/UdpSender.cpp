#include "net/UdpSender.hpp"
#include "net/HandPacket.hpp"   // M3DS/M3DQ 的唯一字节布局定义，见该文件头注释
#include <QByteArray>
#include <QDataStream>

namespace mocap {

UdpSender::UdpSender(QObject* parent)
    : QObject(parent), addr_(QHostAddress::LocalHost) {}

void UdpSender::onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns) {
    if (!enabled_) return;

    QByteArray pkt;
    pkt.reserve(18 + pts.size() * 8);
    QDataStream ds(&pkt, QIODevice::WriteOnly);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setFloatingPointPrecision(QDataStream::SinglePrecision);

    ds.writeRawData("M2D0", 4);
    ds << quint32(camId) << qint64(ts_ns)
       << quint16(pts.size() > 65535 ? 65535 : pts.size());
    const int n = pts.size() > 65535 ? 65535 : pts.size();
    for (int i = 0; i < n; ++i)
        ds << float(pts[i].x()) << float(pts[i].y());

    sock_.writeDatagram(pkt, addr_, port_);
    ++packets_;
}

void UdpSender::onHandPoseSmoothed(QVector3D wristPos, QVector<double> wristRot9,
                                   QVector<double> jointAngles16, qint64 ts_ns) {
    // 【两条 return 路径也要发回执】原来这两条是静默的：推送开关关着、
    // 或者打包失败，都直接返回、不留任何痕迹。而下游"收不到数据"时，
    // 这两件事跟"发了但对端没配"在系统内部完全分不开 ——
    // 三者的下一步动作毫无共同点，猜错一次就是白查一整轮。
    if (!enabled_) {
        emit handPacketSent(0, 0, -2, seqM3ds_, 0, 0, ts_ns);
        return;
    }
    // 【打包搬到 net/HandPacket.hpp】布局一个字节都没变，只是现在点云面板的
    // 输出监视器和这里调的是同一个函数 —— 屏幕上打印的十六进制就是这里发出去
    // 的那串字节，不存在"显示的和实际发的是两份代码"这种排查时最误导人的情况。
    // 形状不对(理论上不该发生)时 build 返回空 —— 不发半个包，宁可这一帧丢了
    // 也不要下游收到长度不符的payload导致解析错位。
    const QByteArray pkt = wire::buildM3DS(wristPos, wristRot9, jointAngles16, ts_ns);
    if (pkt.isEmpty()) {
        emit handPacketSent(0, 0, -1, seqM3ds_, 0, 0, ts_ns);
        return;
    }

    // 【writeDatagram 的返回值原来是被丢掉的】网络栈拒收（缓冲区满、
    // 路由不可达）在这套系统里等于没发生过 —— 症状是"面板在刷数、
    // Unity 收不到"，而系统内部一切正常。
    const qint64 wrote = sock_.writeDatagram(pkt, addr_, port_);
    const int err = (wrote == pkt.size()) ? 0 : int(sock_.error()) + 1;
    ++packets_; ++seqM3ds_;
    emit handPacketSent(0, int(pkt.size()), err, seqM3ds_,
                        fnv1a(pkt.constData(), pkt.size()), 0, ts_ns);
}

void UdpSender::onSegmentQuats(QVector3D wristPos, QVector<double> quatWorld64,
                               QVector<double> quatLocal64, QVector<int> segSource16,
                               quint32 flags, qint64 ts_ns) {
    if (!enabled_) {
        emit handPacketSent(1, 0, -2, seqM3dq_, 0, flags, ts_ns);
        return;
    }
    // 同上：布局定义在 net/HandPacket.hpp，跟监视器共用。形状不对就整帧不发，
    // 宁可丢一帧，也不要下游收到长度不符的 payload 而解析错位 —— 错位之后
    // 每一个字段都是错的，比缺一帧难查得多。
    const QByteArray pkt = wire::buildM3DQ(wristPos, quatWorld64, quatLocal64,
                                           segSource16, flags, ts_ns);
    if (pkt.isEmpty()) {
        emit handPacketSent(1, 0, -1, seqM3dq_, 0, flags, ts_ns);
        return;
    }

    const qint64 wrote = sock_.writeDatagram(pkt, addr_, port_);
    const int err = (wrote == pkt.size()) ? 0 : int(sock_.error()) + 1;
    ++packets_; ++seqM3dq_;
    emit handPacketSent(1, int(pkt.size()), err, seqM3dq_,
                        fnv1a(pkt.constData(), pkt.size()), flags, ts_ns);
}

} // namespace mocap
