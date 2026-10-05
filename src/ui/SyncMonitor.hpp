#pragma once
// ---------------------------------------------------------------------------
// 多相机同步 / 调试监控面板（右侧停靠）。逐相机显示：
//   实时 fps、累计帧数、最近采集时间戳、与“最快相机”的时间戳偏差（同步健康）、
//   本帧检测到的质心数。底部汇总：UDP 发包数、全局最大时间戳偏差。
// 时间戳偏差用颜色分级（绿=良好 / 黄=偏差 / 红=严重），一眼看出哪路掉队。
// ---------------------------------------------------------------------------
#include <QWidget>
#include <QHash>
#include <QVector>
#include <QPointF>
#include <QImage>

class QTableWidget;
class QLabel;
class QTimer;

namespace mocap {

class CameraManager;
class UdpSender;

class SyncMonitor : public QWidget {
    Q_OBJECT
public:
    SyncMonitor(CameraManager* mgr, UdpSender* udp, QWidget* parent = nullptr);

    // 由 MainWindow 在相机增删时调用，重建行。
    void rebuildRows();

public slots:
    void onFrame(quint32 camId, const QImage&, double fps);
    void onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns);

private:
    struct Stat {
        double  fps = 0;
        quint64 frames = 0;
        qint64  lastTs = 0;
        int     blobs = 0;
    };

    CameraManager* mgr_;
    UdpSender*     udp_;
    QTableWidget*  table_;
    QLabel*        summary_;
    QTimer*        refresh_;
    QHash<quint32, Stat> stats_;

    void refresh();
};

} // namespace mocap
