#pragma once
// ---------------------------------------------------------------------------
// 相机抽象接口。虚拟相机、系统摄像头(Qt Multimedia)、将来的工业相机SDK
// 都继承它，UI 只认这个接口。帧通过 frameReady 信号推送。
// ---------------------------------------------------------------------------
#include "core/Types.hpp"
#include "detect/CentroidDetector.hpp"
#include <QObject>
#include <QImage>
#include <QString>
#include <QVector>
#include <QPointF>
#include <QMetaType>

// Blob 要经 blobDetailsReady 跨线程传递(相机采集通常在独立线程)——跨线程
// 用的是 Qt::QueuedConnection，槽函数实际调用发生在事件循环里，参数得先
// 序列化进事件队列，Qt 的元对象系统要求参与这个过程的类型必须注册过，
// 否则要么编译期报错(直连时不会，排队连接时才会)要么运行时静默丢弃这个
// 信号、日志里给一句很容易被忽略的警告。QPointF 之类内建类型已经注册过，
// Blob 是我们自己的结构体，需要显式声明。放在这里而不是 CentroidDetector.hpp
// 里，是因为那个头文件刻意保持零 Qt 依赖（供纯逻辑单测），Q_DECLARE_METATYPE
// 需要 <QMetaType>，只应该在真正引入 Qt 的这一层(ICamera.hpp)做。另外
// QVector<Blob>(信号参数的实际类型)本身也建议在程序启动时跑一次
// qRegisterMetaType<QVector<mocap::Blob>>()（比如 main() 里，创建任何相机
// 之前）——漏了这一步的典型症状是：同线程直连时一切正常(直连不走事件
// 队列)，换成跨线程的相机采集线程后 blobDetailsReady 却什么都收不到，
// 且不一定报错，容易被误判成"手部估计逻辑有bug"而不是"少注册了一行"。
Q_DECLARE_METATYPE(mocap::Blob)

namespace mocap {

class ICamera : public QObject {
    Q_OBJECT
public:
    explicit ICamera(quint32 id, QString name, QObject* parent = nullptr)
        : QObject(parent), id_(id), name_(std::move(name)) {}
    ~ICamera() override = default;

    quint32 id() const { return id_; }
    QString name() const { return name_; }

    virtual bool start() = 0;
    virtual void stop()  = 0;
    virtual void applyParams(const CameraParams& p) = 0;
    virtual CameraParams params() const = 0;

    // 持久化用的设备键：虚拟相机为 "virt"；系统摄像头为 "web:<设备唯一id>"。
    virtual QString deviceKey() const { return QStringLiteral("virt"); }
    // 是否有硬件参数（DirectShow 属性页）。
    virtual bool hasHardwareParams() const { return false; }

    // 质心检测开关（默认关闭，开销按需产生）。开启后每帧检测反光球，
    // 结果经 blobsReady 广播给预览叠加与 UDP 输出。
    virtual void setDetectEnabled(bool) {}
    virtual bool detectEnabled() const { return false; }
    virtual void setDetectParams(const DetectParams&) {}

    // 算法通道可选：默认发相机原样帧；打开后转单通道灰度再发（省下游解析）。
    virtual void setGrayOutput(bool) {}
    virtual bool grayOutput() const { return false; }

    // 预览显示模式：0 画面（彩色/灰度由 setGrayOutput() 那个总开关决定，
    // 不在这里选）  1 阈值掩膜（检测器的二值视界，固定基于灰度阈值化）。
    // 注意 frameReady 不止喂 UI 预览小窗——标定向导、相机宫格等所有"要看画面"
    // 的消费者订阅的都是这同一个信号，所以这个模式其实影响的是"发给所有这些
    // 消费者的是什么"，不只是"你在预览里看到什么"。
    virtual void setPreviewMode(int) {}

    // 手部动捕(遮挡感知检测层 3a~3d)是否需要轮廓点。默认 false——大部分
    // 消费者(预览叠加、UDP 2D 输出)只要质心，帧内多存一份轮廓点是纯浪费
    // (内存 + flood fill 里多几行判断)。只有真正订阅了 blobDetailsReady
    // 的消费者(HandTrackingWorker)存在时才该打开，由上层(比如 MainWindow
    // 发现挂了 HandTrackingWorker 后)调用这个接口打开。
    virtual void setContourCollectionEnabled(bool) {}
    virtual bool contourCollectionEnabled() const { return false; }

signals:
    // 预览通道：图像 + 实时测得帧率。UI 订阅这个（可能是缩放后的小图）。
    void frameReady(quint32 camId, const QImage& img, double fps);

    // 算法通道：未经 RGB 转换的原始灰度/亮度帧，供动捕（2D 质心检测、
    // 多相机三角化）订阅。ts_ns 为采集时间戳（相机/驱动时钟，多相机同步用），
    // 非“到达 UI 的时刻”。img 尽量为 Format_Grayscale8，避免算法二次转换。
    // 默认不发；只有真正接入算法消费者后才有开销。
    void rawFrameReady(quint32 camId, const QImage& gray, qint64 ts_ns);

    // 检测通道：本帧检测到的反光球质心（像素坐标）+ 采集时间戳。
    // 供预览叠加十字丝与 UDP 输出订阅。
    void blobsReady(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns);

    // 检测通道(详细版)：跟 blobsReady 同一帧、同一批检测结果，但带完整
    // Blob(含轮廓点，前提是 setContourCollectionEnabled(true) 开着，否则
    // Blob::contour 是空的)。这是新加的信号，不改动 blobsReady 的既有语义
    // 和消费者——detect/DetectionOutput.hpp 那条 3a~3d 遮挡感知流水线
    // (经 HandTrackingWorker) 订阅这个信号，而不是 blobsReady，因为它需要
    // 轮廓点而不只是质心。两个信号应该在同一次 detect() 调用后一起 emit，
    // 不要为了这个信号单独再跑一次 detect()（那样等于 flood fill 两遍）。
    void blobDetailsReady(quint32 camId, const QVector<Blob>& blobs, qint64 ts_ns);

protected:
    quint32 id_;
    QString name_;
};

} // namespace mocap
