#pragma once
// ===========================================================================
// HandOutputMonitor —— 「链路最终发出去的到底是什么」实时打印面板
// ===========================================================================
// 挂在实时动捕面板 3D 视图下方，把 hm20 链路的两路输出（M3DS 关节角、
// M3DQ 分段四元数）逐帧打印出来。
//
// 【三条设计约束，都是被这个项目里已经踩过的坑逼出来的，改之前先读完】
//
// ① 数据源必须是【实际发出去的那两个信号】，不是拿 SkeletonFrameResult 重算。
//    PointCloudTestDialog::handPoseForUdp / segmentQuatsForUdp 这两个信号，
//    MainWindow 那边一头接 UdpSender、这边一头接监视器，是同一次 emit 的两个
//    消费者。如果监视器自己从 result 再算一遍，就把「时序滤波 -> ROM 映射 ->
//    速度限幅」这三步跳过去了 —— 而链路上最容易出问题、最需要被看见的恰恰
//    就是这三步（比如 ROM 没标定导致关节角被钳到限位，动作被压平）。
//    重算出来的数字很好看，但它不是发出去的那份，那样的面板不如没有。
//
// ② UI 刷新【不能由数据率驱动】。上游 120fps，16 个关节 + 32 个四元数，如果
//    每来一帧就 setText/update 一次，GUI 线程直接被吃满 —— 这跟
//    PointCloudTestDialog.cpp 里 skeletonDiagLabel_ 那段"别改回 setWordWrap
//    (true)"的注释是同一类问题，只是更严重。
//    做法：槽函数只往 snap_ 里塞值（纯赋值，一个 UI 调用都没有），另有一个
//    10Hz 的 QTimer 负责重绘。于是上游是 120Hz 还是 1000Hz，UI 开销恒定。
//
// ③ 数字区【自绘】，不用 QLabel 矩阵 / QTableWidget。48 个数字每 100ms 变一次，
//    用控件意味着每次都要走布局协商；自绘就是一次 paintEvent 画完，零控件、
//    零布局。项目里 PointCloudWidget 已经是这个路子。
//
// 【面板隐藏时零成本】setActive(false) 之后槽函数第一行就 return，定时器停掉。
// 默认不勾选 = 完全不影响原有链路的任何性能特征。
// ===========================================================================
#include <QWidget>
#include <QVector>
#include <QVector3D>
#include <QByteArray>
#include <QElapsedTimer>
#include <QString>
#include <array>
#include <memory>

class QComboBox;
class QCheckBox;
class QPushButton;
class QLabel;
class QTimer;
class QFile;
class QTextStream;

namespace mocap {

// ---------------------------------------------------------------------------
// 一帧输出的完整快照。槽函数往里塞、定时器往外读，都在 GUI 线程，不需要锁。
// ---------------------------------------------------------------------------
struct HandOutputSnapshot {
    // ---- M3DS（关节角，真正发出去的值：ROM 映射 + 速度限幅之后）----
    bool  hasPose = false;
    QVector3D wristPos;
    std::array<double, 9>  wristR{{1,0,0, 0,1,0, 0,0,1}};   // 行主序
    std::array<double, 16> q{};          // 弧度
    qint64 poseTsNs = -1;

    // ---- 解算出来的原始角（未经 ROM 映射/限幅），只用于对照，不进 UDP ----
    // 【为什么要它】看到"关节角不动"时，第一个要分清的是：解算就没解出来，
    // 还是解出来了但被 ROM/限幅压平了。这两种情况的解法完全相反（前者查遮挡
    // 和标定，后者重做 ROM 标定），而只看发出去的那一份根本分不清。
    bool  hasRaw = false;
    std::array<double, 16> qRaw{};
    quint32 rawFlags = 0;   // bit0..4 该指本帧新算  bit5 mcpValid  bit6 wristValid  bit7 romReady

    // ---- 只给数据面板横条用的“弯曲进度” ----
    // 【为什么不直接用 qOut】qOut 是预测/实测混合后的关节角，预测段可能只有
    // 满行程的 10%~40%。但操作者看横条时问的是“我弯了多少”，不是“这一帧
    // 预测器的角度是多少”。所以腕部/手指的可见弯曲参考单独送一路给 UI；
    // 数字列仍然显示真实 qOut，横条按 0..1 弯曲进度画。
    bool   hasBend = false;
    // 每根手指自己的弯曲进度 0..1，索引 0=拇 1=食 2=中 3=无 4=小。
    // 只用它画横条；数字列仍然是真实 qOut，UDP 也不受影响。
    std::array<double, 5> bendFinger{};

    // ---- M3DQ（分段四元数）----
    bool  hasQuat = false;
    std::array<double, 64> quatW{};      // 世界系 (w,x,y,z)×16
    std::array<double, 64> quatL{};      // 相对父节点
    std::array<int, 16>    segSource{};  // 0=None 1=Predicted 2=Geometry 3=IK
    quint32 flags = 0;
    qint64  quatTsNs = -1;

    // ---- 速率/延迟 ----
    double   poseHz = 0.0, quatHz = 0.0;
    quint64  poseCount = 0, quatCount = 0;
    // 包里的 ts_ns 到"这一帧送达监视器"之间的耗时。
    //
    // 【时基必须跟 ts_ns 一致】ts_ns 是相机帧时间戳，走的是采集侧的【单调时钟】
    // （开机起算）。原来这里拿 QDateTime::currentMSecsSinceEpoch()（Unix 纪元）
    // 去减它，算出来的是"1970 到现在"——真机上显示 1787590324629ms ≈ 56 年。
    // 那不是滞后，是两个时基的原点差。
    // 【不再算"ts 与墙钟的差"】那个减法从根上就不成立：
    // ts_ns 来自 WebcamCamera 的 f.startTime()（驱动时基）或每台相机 start()
    // 时各自归零的 QElapsedTimer —— 原点跟监视器毫无关系。真机上显示
    // 1787590324629ms ≈ 56 年，那不是滞后，是两个时基的原点差。
    //
    // 换成【同一时基内】能量的东西：相邻两包的 ts 增量，也就是输出周期。
    // 增量是同一个时钟内部的差，原点抵消掉了，这个数才有意义。
    double   tsStepMs = 0.0;     // 相邻两包 ts 的增量（= 输出周期）
    bool     tsStepValid = false;
    qint64   prevTsNs = -1;
    qint64   lastArrivalMs = -1; // 最近一次收到输出的【纪元】墙钟，只用于判"断流"
};

// ---------------------------------------------------------------------------
// 纯绘制区。没有任何子控件，全部靠 paintEvent。
// ---------------------------------------------------------------------------
class HandOutputView : public QWidget {
    Q_OBJECT
public:
    enum Mode { Joints = 0, Quats = 1, Bytes = 2 };

    explicit HandOutputView(QWidget* parent = nullptr);

    void setMode(Mode m) { mode_ = m; update(); }
    Mode mode() const { return mode_; }
    // 快照按值拷贝一份 —— 一帧 ~1KB，10Hz，可以忽略；换来的是绘制期间数据
    // 绝不会被新到的帧改掉（画到一半数字变了会出现同一屏上下不一致）。
    void setSnapshot(const HandOutputSnapshot& s) {
        snap_ = s;
        if (s.tsStepValid) {                       // 顺手攒一段输出周期的历史
            stepHist_.append(s.tsStepMs);
            while (stepHist_.size() > kSparkN) stepHist_.removeFirst();
        }
        update();
    }
    void setRawPackets(const QByteArray& m3ds, const QByteArray& m3dq) {
        m3ds_ = m3ds; m3dq_ = m3dq;
    }
    void setUdpStatus(bool enabled, const QString& target) {
        udpOn_ = enabled; udpTarget_ = target; update();
    }

    // 当前屏幕内容的纯文本版本，给"复制"按钮用。
    QString asText() const;
    // 两个报文的完整十六进制 dump（不受面板高度截断影响）。
    QString rawText() const;

protected:
    void paintEvent(QPaintEvent*) override;

private:
    void paintJoints(QPainter& p);
    void paintQuats(QPainter& p);
    void paintBytes(QPainter& p);

    // ---- 右侧仪表区（三个模式共用的一组小部件）----
    // 三种模式原来都只画左半屏，右边一大片是空的（宽度全浪费在留白上）。
    // 这些函数负责把右半屏填成一排"仪表"，每个都自带 HUD 式的角标边框。
    void drawHudFrame(QPainter& p, const QRectF& r, const QString& title) const;
    void drawAttitudeGizmo(QPainter& p, const QRectF& r) const;   // 腕部姿态三轴罗盘
    void drawEulerGauges(QPainter& p, const QRectF& r) const;     // roll/pitch/yaw 弧形表
    void drawPipeline(QPainter& p, const QRectF& r) const;        // 检测→聚类→骨架→ROM→UDP
    void drawFingerBars(QPainter& p, const QRectF& r) const;      // 逐指新鲜度
    void drawStepSpark(QPainter& p, const QRectF& r) const;       // 输出周期滚动折线
    void drawHandTopology(QPainter& p, const QRectF& r) const;    // 手部拓扑图（按来源着色）

    Mode mode_ = Joints;
    HandOutputSnapshot snap_;
    QByteArray m3ds_, m3dq_;
    bool    udpOn_ = false;
    QString udpTarget_;

    // 输出周期的滚动历史，给 drawStepSpark 用。定长环形缓冲，10Hz 写入，
    // 120 个点 ≈ 最近 12 秒。只是给人看的，丢几个点无所谓。
    static constexpr int kSparkN = 120;
    QVector<double> stepHist_;
    // 内容因高度不够被截掉的行数。>0 时在顶栏右上角挤一个提示药丸出来 ——
    // 原来是在最底下写一行"高度不够"，而那一行本身也可能正好被截掉。
    mutable int hiddenRows_ = 0;
};

// ---------------------------------------------------------------------------
// 监视器 = 控制条 + 绘制区。
// ---------------------------------------------------------------------------
class HandOutputMonitor : public QWidget {
    Q_OBJECT
public:
    explicit HandOutputMonitor(QWidget* parent = nullptr);
    ~HandOutputMonitor() override;

    // 面板收起时调 false：槽函数立刻 return、定时器停转，成本归零。
    void setActive(bool on);
    bool isActive() const { return active_; }

    // 主窗口的 UDP 开关状态。【值得显示】最常见的困惑是"面板明明在刷数，
    // Unity 什么都收不到"，九成是主窗口那个 UDP 推送没打开 —— 面板上直接
    // 写出来，比让人去翻另一个窗口强。
    void setUdpStatus(bool enabled, const QString& target);

public slots:
    // 签名跟 UdpSender::onHandPoseSmoothed / onSegmentQuats 完全一致 ——
    // 同一个信号可以同时接到 UdpSender 和这里，天然保证两边拿到的是同一份。
    void onHandPose(QVector3D wristPos, QVector<double> wristRot9,
                    QVector<double> jointAngles16, qint64 ts_ns);
    void onSegmentQuats(QVector3D wristPos, QVector<double> quatWorld64,
                        QVector<double> quatLocal64, QVector<int> segSource16,
                        quint32 flags, qint64 ts_ns);
    // 对照用：解算原始角 vs 实际输出角。来自 SkeletonAssocWorker::jointAnglesDebug。
    void onJointAnglesDebug(QVector<double> rawRad16, QVector<double> outRad16,
                            quint32 mask, qint64 ts_ns);
    // 只给数据面板横条用的弯曲进度。数字列/ UDP 不受它影响。
    void onBendRef(QVector<double> fingerBend5, qint64 ts_ns);

private:
    void onTick();          // 10Hz：把 snap_ 推给绘制区并刷状态行
    void ensureRawPackets();   // 按当前快照补打 M3DS/M3DQ 字节（见 .cpp 说明）
    void toggleCsv();
    void writeCsvRow();
    void copyToClipboard();

    HandOutputView* view_ = nullptr;
    QComboBox*  modeCombo_ = nullptr;
    QCheckBox*  freezeChk_ = nullptr;
    QPushButton* copyBtn_ = nullptr;
    QPushButton* csvBtn_  = nullptr;
    QLabel*     statusLabel_ = nullptr;
    // 横条弯曲参考的轻量平滑，避免 UI 尖峰；每根手指独立。
    std::array<double, 5> bendSmooth_{};
    bool bendSmoothInit_ = false;
    QTimer*     timer_ = nullptr;

    bool active_ = false;
    HandOutputSnapshot snap_;

    // 速率统计：数包，每 500ms 折算一次 Hz。不用逐帧间隔取倒数 —— 那个数
    // 抖得没法看，而且丢一帧就冒一个尖峰。
    QElapsedTimer rateClock_;
    quint64 posePrev_ = 0, quatPrev_ = 0;

    // 最近一次的原始报文字节（只在"原始报文"模式下才生成，避免 120Hz 白打包）
    QByteArray lastM3ds_, lastM3dq_;

    // CSV 记录
    std::unique_ptr<QFile>       csvFile_;
    std::unique_ptr<QTextStream> csvOut_;
    qint64 csvRows_ = 0;
    QString csvPath_;
};

} // namespace mocap
