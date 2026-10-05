#pragma once
#include "camera/CameraManager.hpp"
#include "ui/CameraGrid.hpp"
#include "ui/SyncMonitor.hpp"
#include "net/UdpSender.hpp"
#include "calib/CalibrationStore.hpp"
#include "hand/HandTemplateStore.hpp"
#include "settings/ParamPresets.hpp"
#include "settings/AppSettings.hpp"
#include <QMainWindow>
#include <QStringList>
#include <QHash>
#include <QVector>
#include <memory>

class QLabel;
class QTimer;
class QAction;
class QComboBox;
class QListWidget;
class QSplitter;
class QStackedWidget;
class QMenu;
class QWidget;

namespace mocap {

    class DetectParamsPanel;
    class HandTrackingWorker;

    class MainWindow : public QMainWindow {
        Q_OBJECT
    public:
        MainWindow();

    protected:
        void closeEvent(QCloseEvent*) override;

    private slots:
        void addCamera();
        void removeCamera();
        void saveAsDefault();
        void openCamParams(quint32 id);      // 参数（含格式/分辨率/帧率，已合并）
        void toggleUdp(bool on);             // 全局：UDP 推送 2D 点到 Unity
        void toggleMonitor(bool on);         // 显示/隐藏同步监控面板（现在是侧边栏的一页）
        void toggleGrayOutput(bool on);      // 全局：算法通道输出灰度（默认关=原样帧）
        void openCalibration();              // 打开标定向导（内外参）
        void openCalibrationLibrary();       // 标定模板库
        void openTriangulationDebug();       // 三角化调试
        void openLiveMocap();                // 实时动捕（正式运行台，原「点云测试」）
        void openWandPrecision();            // 标定杆精度验证
        void openDShowTest();                // DirectShow 采集原型（实验性）
        void openCameraRemap();              // 重插拔后把实时相机映射回模板已标定相机
        void openHandTracking();             // 打开手部动捕调试窗口（关联+IEKF+骨架/轨迹可视化+手指屈曲测试面板）
        void openHandCalibrationWizard();    // 打开手部标定向导（手背模板+手指结构自标定）
        void applyPreset(int which);         // 应用参数模板：0标定板 1追踪
        void savePreset(int which);          // 把当前参数存为模板
        void setPreviewModeAll(int mode);    // 预览显示：0画面(彩/灰看算法灰度开关) 1阈值掩膜

    private:
        // ---- 侧边栏页序号。跟活动栏按钮一一对应，加页时两边一起加 ----
        enum SidePane { PaneCameras = 0, PaneDetect = 1, PaneMonitor = 2,
                        PaneCalib = 3, PaneTools = 4, PaneCount = 5 };

        // ---- 界面搭建（纯 UI，不含任何业务逻辑）----
        void buildActions();       // 先把所有 QAction 建好，菜单/活动栏/标签条共用同一批
        void buildMenuBar();
        void buildShell();         // 活动栏 + 侧边栏 + 编辑区
        QWidget* buildActivityBar();
        QWidget* buildSideBar();
        QWidget* buildEditorArea();
        void buildStatusBar();
        QWidget* makeCamerasPane();
        QWidget* makeCalibPane();
        QWidget* makeToolsPane();
        void selectSidePane(int pane);        // 点活动栏：同一页再点一次就收起
        void setSideBarVisible(bool on);
        void refreshCameraList();             // 侧边栏「相机」页的列表

        CameraManager* mgr_;
        // 添加相机对话框（非模态，见 addCamera() 的说明）。开着时不重复弹。
        class AddCameraDialog* addCamDlg_ = nullptr;
        // 实时动捕窗口（非模态，见 openLiveMocap() 的说明）。开着时不重复弹，
        // 关闭时在 QDialog::finished 回调里清掉。
        // 【为什么不在这里持有 CalibrationStore】主窗口的成员先于它的子对象析构：
        // 主窗口关闭时，挂在成员上的 store 会在对话框之前消失，而对话框里存的是
        // 它的裸指针。store 改由 openLiveMocap() 里的 shared_ptr 捕获持有。
        class PointCloudTestDialog* liveMocapDlg_ = nullptr;
        CameraGrid* gridView_;
        SyncMonitor* monitor_ = nullptr;
        DetectParamsPanel* detectPanel_ = nullptr;   // 阈值/面积/圆度集中面板，hookCamera() 靠它取新相机的初始检测参数
        UdpSender* udp_ = nullptr;
        CalibrationStore  calib_;
        // 手部模板(手背5点 + 5根手指结构参数)的持久化——"手部追踪…"和
        // "手部标定向导…"共用同一份，向导写入、追踪窗口读取，用成员保证
        // 两边看到的是同一份数据、同一次运行期内标了立刻生效，不用重启。
        HandTemplateStore handTemplateStore_;
        ParamPresets      presets_;
        int               previewMode_ = 0;
        AppSettings    settings_;
        AppConfig      cfg_;

        // ---- VS Code 式外壳的部件 ----
        QSplitter*      splitter_    = nullptr;   // 侧边栏 | 编辑区
        QWidget*        sideBar_     = nullptr;
        QStackedWidget* sideStack_   = nullptr;
        QLabel*         sideTitle_   = nullptr;   // 侧边栏顶部那行小字标题
        QVector<QAction*> activityActions_;       // 活动栏按钮，序号即 SidePane
        QListWidget*    cameraList_  = nullptr;   // 侧边栏「相机」页
        QComboBox*      previewCombo_ = nullptr;  // 编辑区标签条右侧
        bool            sideVisible_ = true;
        int             lastSideWidth_ = 300;     // 收起前记住宽度，展开时还原

        // 菜单/活动栏/标签条共用的开关型动作（共用同一个 QAction 才能各处状态同步）
        QAction* actUdp_    = nullptr;
        QAction* actGray_   = nullptr;
        // 前置到标签条上的三个常用入口：实时动捕（主行动）、标定、标定模板库。
        // 跟菜单共用同一批 QAction，快捷键写在 QAction 上，两处都生效。
        QAction* actRun_       = nullptr;
        QAction* actCalib_     = nullptr;
        QAction* actCalibLib_  = nullptr;
        QMenu*   presetMenu_   = nullptr;   // 参数模板：应用/存为，菜单和标签条共用同一个
        QLabel*  calibTplLabel_ = nullptr;  // 状态栏：当前标定模板
        void refreshCalibTemplateLabel();

        // 解码耗时基准：状态栏常驻标签，每 2s 拉一次 decodeStatsSnapshot() 刷新。
        // 跟"算法灰度"开关联动——切换开关时 resetDecodeStats()，保证看到的是
        // 切换后这一段时间窗口的平均值，不是从程序启动混到现在的总平均。
        QLabel*  decodeStatsLabel_ = nullptr;
        QTimer*  decodeStatsTimer_ = nullptr;
        void refreshDecodeStatsLabel();
        bool           detectOn_ = false;
        bool           grayOut_ = false;
        // 相机重映射对话框用的模板 store。对话框构造要求 store* 活得比它久，
        // 用成员（unique_ptr）持有，openCameraRemap 打开时创建、用完释放。
        std::unique_ptr<CalibrationStore> remapStore_;

        void restoreCameras();               // 按 cfg_.cameraKeys 恢复上次的相机
        QStringList currentCameraKeys() const;
        // 【存盘用的清单，不等于当前活着的清单】开机时没枚举到的相机必须
        // 保留在模板里，否则模板会一次比一次少、且永远回不来。见 .cpp 说明。
        QStringList cameraKeysForSave() const;
        // 开机 restoreCameras() 时想恢复、但在 QMediaDevices 里没找到的 key。
        QStringList missingAtBoot_;
        void hookCamera(ICamera* cam);       // 把新相机接到检测/监控/UDP
        // 把这台相机保存过的硬件参数推回驱动。【只在采集引擎(重)启动完成后
        // 调用】—— 早于它推下去的值会被建图重开设备时抹掉。见 .cpp 说明。
        void restoreCameraHardwareParams(quint32 camId);
        QList<class WebcamCamera*> webcams() const;   // 全部真实相机
    };

} // namespace mocap
