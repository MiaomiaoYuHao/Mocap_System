#pragma once
// 单路预览面板：绘制最新帧 + 叠加信息条 + 反光球检测十字丝。
// 交互：左键拖拽 -> 与目标面板换位；双击 -> 放大/还原；右键 -> 菜单。
#include <QWidget>
#include <QImage>
#include <QString>
#include <QPoint>
#include <QPointF>
#include <QVector>
#include <QElapsedTimer>

namespace mocap {

    class CameraView : public QWidget {
        Q_OBJECT
    public:
        explicit CameraView(quint32 camId, QString name, QWidget* parent = nullptr);
        quint32 camId() const { return camId_; }

    public slots:
        void onFrame(quint32 camId, const QImage& img, double fps);
        void onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns);

    signals:
        void swapRequested(quint32 srcId, quint32 dstId);  // 拖拽换位
        void soloToggled(quint32 id);                      // 双击放大/还原
        void paramsRequested(quint32 id);                  // 右键: 参数设置（含格式）
        void removeRequested(quint32 id);                  // 右键: 移除

    protected:
        void paintEvent(QPaintEvent*) override;
        void mousePressEvent(QMouseEvent*) override;
        void mouseMoveEvent(QMouseEvent*) override;
        void mouseDoubleClickEvent(QMouseEvent*) override;
        void contextMenuEvent(QContextMenuEvent*) override;
        void dragEnterEvent(QDragEnterEvent*) override;
        void dropEvent(QDropEvent*) override;

    private:
        quint32 camId_;
        QString name_;
        QImage  frame_;
        double  fps_ = 0.0;
        QVector<QPointF> blobs_;   // 最新一帧检测到的质心（原始帧坐标系）
        QElapsedTimer blobsAge_;  // 距上次收到检测结果过了多久：超时则视为“检测已停”，
        // 即使 blobs_ 里还有残留数据也不再画，避免关闭检测/UDP
        // 推送后十字丝永久卡在最后一个点上不消失。
        QPoint  pressPos_;
    };

} // namespace mocap