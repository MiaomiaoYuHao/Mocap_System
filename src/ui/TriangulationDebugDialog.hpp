#pragma once
// 三角化调试窗口：勾选任意 >=2 台已标定相机，实时 N 视图三角化单球位置
// 并可视化。加多少台勾多少台，算法自动适应。另外显示一条随"标定/场景
// 匹配度"变色的横幅（绿/黄/红），提醒残差是否持续偏大。
//
// 精度算法（①时间插值补偿 ②鲁棒剔除坏视角 ③输出端 One Euro 滤波，见
// Triangulator.hpp）默认全开，但光是"开着"看不出到底有没有用——这里加了
// 三个复选框，能单独关掉某一项、配合下面的"抖动统计"直接看数字变化：
// 拿球（或架子）悬空不动，点"重置统计"清零，观察 jitterLabel_ 的数值——
// 数值是最近若干帧输出点的位置标准差(mm)，球没动的话这个数就是纯抖动量，
// 关掉某个开关再重置一次、对比数值前后，就是最直接的"生没生效"证据。
#include "reconstruct/Triangulator.hpp"
#include <QDialog>
#include <QVector>
#include <QSet>
#include <QVector3D>

class QLabel;
class QListWidget;
class QCheckBox;
class QComboBox;
class QPushButton;

namespace mocap {

    class CameraManager;
    class CalibrationStore;
    class TriangulationView;
    class ICamera;

    class TriangulationDebugDialog : public QDialog {
        Q_OBJECT
    public:
        // wasDetectOn：打开这个窗口之前，检测（UDP推送那个总开关）本来是不是
        // 已经开着。窗口打开期间会自动帮勾选的相机开检测（否则 blobsReady
        // 永远不会来，点开跟没反应一样）；关闭窗口时恢复成打开前的状态——
        // 本来就开着的不用管，本来没开的，帮它关掉的这几台要关回去。
        TriangulationDebugDialog(CameraManager* mgr, CalibrationStore* store,
            bool wasDetectOn, QWidget* parent = nullptr);
        ~TriangulationDebugDialog() override;

    private slots:
        void rebuildTriangulator();
        void onPoint(QVector3D pos, double residualMm, int usedViews, qint64 ts_ns, QVector<quint32> usedCamIds);
        void onQuality(TriangulationQuality quality, double avgResidualMm, int windowSize);

    private:
        CameraManager* mgr_;
        CalibrationStore* store_;
        bool wasDetectOn_ = true;
        QSet<ICamera*> weTurnedOn_;   // 本窗口帮忙打开检测的那几台，关闭时要关回去
        QListWidget* list_;      // 勾选参与的相机
        QLabel* status_;
        QLabel* qualityBanner_;
        // 【新增】"相机接力"跳变诊断——见 Triangulator::point3DReady 里
        // usedCamIds 的说明。逐帧记上一帧参与解算的相机集合和位置，新的
        // 一帧如果集合变了、同时位置跳变幅度超过 kHandoffJumpMm，判定为
        // 一次"相机切换导致的跳变"，跟纯粹的检测噪声/球本身快速移动区分
        // 开——纯噪声不会跟相机集合变化同步出现，球快速移动時usedCamIds
        // 通常不变(还是那几台在看)。这个标签专门展示这类事件，不跟其它
        // 状态信息混在一起，方便你直接对照"跳变的位置"是不是反复出现在
        // 同一批相机切换点附近——如果是，问题在这几台相机的标定自洽性，
        // 不是检测噪声。
        QLabel* handoffLabel_ = nullptr;
        QVector<quint32> lastUsedCamIds_;
        QVector3D lastPos_;
        bool hasLastPos_ = false;
        int handoffJumpCount_ = 0;
        int totalHandoffEvents_ = 0;   // 相机集合变了，但没伴随明显跳变的次数——正常情况应该占多数
        static constexpr double kHandoffJumpMm = 10.0;   // 跳变幅度超过这个值才算"明显"，压掉正常的检测噪声级别抖动
        TriangulationView* view_;
        Triangulator* tri_ = nullptr;

        // ---- 精度算法开关（逐个测试用）----
        QCheckBox* chkInterp_ = nullptr;    // ①时间插值补偿
        QComboBox* robustModeCombo_ = nullptr;   // ②鲁棒模式：关闭/硬剔除/软加权IRLS
        QCheckBox* chkFilter_ = nullptr;    // ③输出端 One Euro 滤波
        void applyAlgoOptions();            // 把三个复选框的当前状态同步到 tri_（存在的话）

        // ---- 抖动统计（效果对比用）----
        // 拿标定物悬空静止，这段时间输出点的位置标准差就是纯抖动量——数值
        // 越小说明这一刻生效的这组开关组合越稳。不区分是哪个算法带来的改善，
        // 只反映"当前这组开关下，输出到底稳不稳"，配合逐个开关关闭对比就能
        // 看出每一项的贡献。
        QVector<QVector3D> jitterSamples_;
        static constexpr int kJitterWindow = 60;   // 约2秒窗口（30fps下）
        QLabel* jitterLabel_ = nullptr;
        QPushButton* resetJitterBtn_ = nullptr;
        void resetJitterStats();
        void updateJitterLabel();
    };

} // namespace mocap