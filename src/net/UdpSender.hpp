#pragma once
// ---------------------------------------------------------------------------
// UDP 推送：把各相机每帧检测到的 2D 质心打包成小端二进制包发给下游（Unity）。
// 这一版是“骨架”—— 先把「检测 -> 打包 -> 发送」的管道通了，Unity 端能实时
// 收到每台相机的 2D 点。后续接入三角化 + IK 后，把 payload 换成 3D 位姿即可，
// 端口/传输层不用改。
//
// 包格式（小端，每台相机每帧一个 UDP 包）：
//   magic "M2D0"(4) | camId u32 | ts_ns i64 | count u16 |
//   然后 count 个点：x f32, y f32
//
// 【新增】手部延迟精修流(3D位姿)包格式，同一个 socket/端口，magic 不同
// (下游按 magic 区分包类型，不需要开新端口)：
//   magic "M3DS"(4) | ts_ns i64 |
//   wristPos: x,y,z f32(3个，单位mm，跟UI显示的位置(mm)同一套单位) |
//   wristRot9: 9个 f32(行主序3x3) |
//   jointAngles16: 16个 f32(弧度，跟 HandModel.hpp 的16维布局一致)
// 只转发 handPoseSmoothedReady(延迟但更准的那一路)，不转发实时
// handPoseReady——如果下游也需要实时那一路，需要另外接一个 magic/包类型，
// 这里暂时没做，因为目前的需求明确是"让延迟精修流的结果真正被下游消费"。
//
// 无第三方依赖，用 QUdpSocket。默认发到 127.0.0.1:9010（Unity 端监听即可）。
// ---------------------------------------------------------------------------
#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QVector>
#include <QVector3D>
#include <QPointF>

namespace mocap {

class UdpSender : public QObject {
    Q_OBJECT
public:
    explicit UdpSender(QObject* parent = nullptr);

    void setTarget(const QHostAddress& addr, quint16 port) { addr_ = addr; port_ = port; }
    QHostAddress address() const { return addr_; }
    quint16 port() const { return port_; }

    void setEnabled(bool on) { enabled_ = on; }
    bool isEnabled() const { return enabled_; }

    quint64 packetsSent() const { return packets_; }

public slots:
    // 直接接相机的 blobsReady 信号。
    void onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns);

    // 直接接 HandTrackingWorker::handPoseSmoothedReady——延迟精修流的输出，
    // 不是实时 handPoseReady(那条没有转发，只走 UI；这里刻意只转发"延迟但
    // 更准"的这一路，跟 UI 对照显示是同一份数据的两个消费者)。包格式见
    // UdpSender.cpp 顶部注释，magic "M3DS"(S=Smoothed)跟 M2D0 用同一个
    // 端口也没关系，下游按 magic 的前3字节相同、第4字节区分即可。
    void onHandPoseSmoothed(QVector3D wristPos, QVector<double> wristRot9,
                            QVector<double> jointAngles16, qint64 ts_ns);

    // ---- 分段四元数（magic "M3DQ"，Q=Quaternion）----
    // 【为什么单独一个包，而不是往 M3DS 里塞】M3DS 已经有下游在用，往里加
    // 字段会让老接收端解析错位。新包独立，谁用谁解，互不影响；不需要的
    // 一方按 magic 跳过即可。
    //
    // 包体（小端，float32）：
    //   magic "M3DQ"        4B
    //   ts_ns               i64
    //   flags               u32   bit0: wristPoseValid  bit1: mcpValid
    //                             bit8..12: 五指关节角本帧有效(f0..f4)
    //   wristPos            3 × f32   mm，世界系
    //   segQuatWorld[16]    64 × f32  (w,x,y,z)，世界系绝对姿态
    //   segQuatLocal[16]    64 × f32  (w,x,y,z)，相对父节点，可直接喂
    //                                 Unity 的 bone.localRotation
    //   segSource[16]       16 × u8   0=None 1=Predicted 2=Geometry 3=IK
    //
    // 【下游必须看 segSource】Predicted 表示该段两端点都是网络补的，
    // 是外推不是测量；None 表示完全没依据。照单全收会把一段猜出来的
    // 姿态当成真数据用 —— 做动捕这是最危险的一类错误，因为它看起来很正常。
    //
    // 【关于 roll】每节指骨只有一颗球，绕骨轴的自转在数学上不可观测。
    // 四元数里的那一维是用手背法向填出来的，不是测量值。要真实的 roll
    // 只能每节贴第二颗球。用关节角驱动则天然规避这一点（铰链只有一个自由度）。
    void onSegmentQuats(QVector3D wristPos, QVector<double> quatWorld64,
                        QVector<double> quatLocal64, QVector<int> segSource16,
                        quint32 flags, qint64 ts_ns);

signals:
    // ---- 发包回执。【录制块 23 的唯一真值来源】-----------------------------
    //
    // 【为什么必须由这里发出来，不能在别处重建】
    // 原来这三个 slot 里有两条【静默 return】：enabled_ 为假、以及 build 返回
    // 空包（形状不对）。两条都不留任何痕迹 —— 而下游"收不到数据"时，
    // "没启用""打包失败""发了但对端没配"这三件事在系统内部完全查不到，
    // 会被一路误判成解算问题，往上游白查一整圈。
    //
    // 而且 writeDatagram 的返回值原来是【被丢掉的】：网络栈拒收（缓冲区满、
    // 路由不可达）在这套系统里等于没发生过。
    //
    // 【crc 覆盖的是真正写进 socket 的那串字节】不是别处重建的副本。
    // 离线拿块 13 的 qOut 重新打一份包算 crc，对不上就说明打包这一步本身
    // 有问题 —— 这是唯一能发现它的办法，而它只有在"两边不是同一份字节"
    // 时才成立。所以 crc 必须在这里算。
    //
    // which: 0=M3DS 1=M3DQ
    // err:   0=写成功  -1=打包失败(形状不对，整帧不发)  -2=推送开关是关的
    //        >0 = QAbstractSocket::SocketError + 1（0 是合法错误码，要错开）
    void handPacketSent(int which, int bytes, int err, quint64 seq,
                        quint32 crc, quint32 flags, qint64 ts_ns);

private:
    // fnv1a，跟 pcrec::Recorder::fnv1a 是同一个算法 —— 【必须同一个】，
    // 否则录下来的 crc 跟离线重算的永远对不上，这个字段就废了。
    static quint32 fnv1a(const char* p, int n) {
        quint32 h = 2166136261u;
        for (int i = 0; i < n; ++i) {
            h ^= quint8(p[i]); h *= 16777619u;
        }
        return h;
    }
    quint64 seqM3ds_ = 0, seqM3dq_ = 0;
    QUdpSocket sock_;
    QHostAddress addr_;
    quint16 port_ = 9010;
    bool enabled_ = false;
    quint64 packets_ = 0;
};

} // namespace mocap
