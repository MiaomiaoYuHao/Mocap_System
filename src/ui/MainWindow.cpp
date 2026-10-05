#include "ui/MainWindow.hpp"
#include "ui/AddCameraDialog.hpp"
#include "ui/CamParamDialog.hpp"
#include "ui/CalibWizard.hpp"
#include "ui/DetectParamsPanel.hpp"
#include "ui/TriangulationDebugDialog.hpp"
#include "ui/PointCloudTestDialog.hpp"
#include "ui/WandPrecisionDialog.hpp"
#include "ui/DShowCaptureTestDialog.hpp"
#include "ui/CalibrationLibraryDialog.hpp"
#include "ui/CameraRemapDialog.hpp"
#include "ui/HandPoseDebugDialog.hpp"
#include "ui/HandCalibrationWizard.hpp"
#include "estimate/HandTrackingWorker.hpp"
#include "calib/CalibrationLibrary.hpp"
#include "ui/Theme.hpp"
#include <utility>              // std::as_const
#include "camera/ICamera.hpp"
#include "camera/WebcamCamera.hpp"
#include "camera/DShowControl.hpp"   // restoreCameraHardwareParams()：推曝光要用
#include "camera/JpegDecoder.hpp"

#include <QApplication>
#include <QMenuBar>
#include <QMenu>
#include <QStatusBar>
#include <QToolButton>
#include <QCloseEvent>
#include <QMessageBox>
#include <QMediaDevices>
#include <QCameraDevice>
#include <QLabel>
#include <QTimer>
#include <QSlider>
#include <QComboBox>
#include <QWidget>
#include <QAction>
#include <QKeySequence>
#include <QSplitter>
#include <QStackedWidget>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCursor>
#include <memory>

namespace mocap {

// ===========================================================================
// 界面外壳：照 VS Code 的四块布局搭
//
//   ┌──────────────────────────────────────────────────────┐
//   │ 菜单栏                                                │
//   ├────┬───────────────┬─────────────────────────────────┤
//   │活  │  侧边栏        │  标签条                          │
//   │动  │  (相机/检测/   ├─────────────────────────────────┤
//   │栏  │   监控/标定/   │  编辑区 = 相机宫格                │
//   │    │   工具)        │                                  │
//   ├────┴───────────────┴─────────────────────────────────┤
//   │ 状态栏（整条蓝）                                       │
//   └──────────────────────────────────────────────────────┘
//
// 换布局的直接动因：原来十几个功能全挤在一条工具栏上，窄一点就被折进那个
// 很难发现的 "»" 溢出菜单（加圆度滑块那次已经栽过一回）。改成「菜单栏放
// 全集 + 活动栏切侧边栏 + 侧边栏放常用入口」之后，无论以后再加多少功能，
// 横向都不会再被撑爆 —— 纵向列表想加多少加多少。
//
// 注意：这一轮只动外观和摆放位置。所有 QAction 连的槽、信号连线、相机/
// 检测/UDP 的开关语义跟改之前一模一样，没有任何行为变化。
// ===========================================================================

namespace {
    // 侧边栏里的一行入口按钮，长得像资源管理器的条目（左对齐、无边框）。
    QPushButton* paneButton(const QString& text, const QString& tip = QString()) {
        auto* b = new QPushButton(text);
        b->setCursor(Qt::PointingHandCursor);
        if (!tip.isEmpty()) b->setToolTip(tip);
        return b;
    }
    // 侧边栏里的小节标题（VS Code 的分组标题就是这种暗淡小字）
    QLabel* paneGroupLabel(const QString& text) {
        auto* l = new QLabel(text);
        l->setObjectName(QStringLiteral("sideGroupLabel"));
        return l;
    }
} // namespace

MainWindow::MainWindow()
{
    setWindowTitle(QStringLiteral("Mocap Host"));
    resize(1360, 860);

    cfg_ = settings_.load();
    if (!cfg_.windowGeometry.isEmpty())
        restoreGeometry(cfg_.windowGeometry);

    mgr_      = new CameraManager(this);
    gridView_ = new CameraGrid(mgr_, this);
    udp_      = new UdpSender(this);
    monitor_  = new SyncMonitor(mgr_, udp_, this);

    // 检测参数：阈值/面积下限/面积上限/圆度，集中放进侧边栏的一页，
    // 不再直接堆在工具栏上——上次加圆度滑块时就被工具栏挤进了不易发现的
    // "»"溢出菜单，挪进侧边栏后无论以后再加多少个参数都不会再撑爆工具栏。
    const DetectParams initialDp{ cfg_.defaults.threshold, 3, 2000, 64 };
    detectPanel_ = new DetectParamsPanel(initialDp, this);
    connect(detectPanel_, &DetectParamsPanel::paramsChanged, this, [this](const DetectParams& p) {
        cfg_.defaults.threshold = p.threshold;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectParams(p);
    });

    buildActions();
    buildMenuBar();
    buildShell();
    buildStatusBar();

    // ---- 信号连线 ----
    connect(mgr_, &CameraManager::countChanged, this, [this](int n) {
        gridView_->rebuild();
        monitor_->rebuildRows();
        refreshCameraList();
        statusBar()->showMessage(
            QStringLiteral("相机 %1 路   ·   拖拽换位 · 双击放大 · 右键调参").arg(n));
    });
    connect(gridView_, &CameraGrid::paramsRequested, this, &MainWindow::openCamParams);
    connect(gridView_, &CameraGrid::removeRequested, this, [this](quint32 id) {
        mgr_->removeById(id);
    });

    // ---- 启动：恢复上次相机；否则按设置建虚拟相机 ----
    if (!cfg_.cameraKeys.isEmpty()) {
        restoreCameras();
    } else {
        for (int i = 0; i < cfg_.autoStartCameras; ++i) mgr_->addVirtual(cfg_.defaults);
    }
    // 给已创建的相机补挂钩子。
    for (int i = 0; i < mgr_->count(); ++i) hookCamera(mgr_->at(i));
    refreshCameraList();

    if (mgr_->count() == 0)
        statusBar()->showMessage(QStringLiteral("点击“＋ 添加相机”接入摄像头，或添加虚拟测试相机"));

    // 开机停在「相机」页（跟 VS Code 默认开资源管理器一个意思）。
    // 这里不能走 selectSidePane —— 那个函数的语义是"再点一次就收起"，
    // 而侧边栏初始就是展开的、当前页也正好是 0，走它反而会一进来就收起。
    sideStack_->setCurrentIndex(PaneCameras);
    activityActions_[PaneCameras]->setChecked(true);
    refreshCalibTemplateLabel();
}

// ---------------------------------------------------------------------------
// 动作：菜单、活动栏、标签条共用同一批 QAction —— 共用之后勾选状态天然同步，
// 不需要手写"三处互相 setChecked"的同步代码（那种代码最容易漏一处）。
// ---------------------------------------------------------------------------
void MainWindow::buildActions()
{
    actUdp_ = new QAction(QStringLiteral("UDP 推送"), this);
    actUdp_->setCheckable(true);
    actUdp_->setIcon(theme::toolIcon(theme::Glyph::RadioTower));
    actUdp_->setToolTip(QStringLiteral(
        "开启后自动打开反光球检测（预览叠加十字丝），\n并把每帧 2D 点经 M2D0 协议推送到 Unity。"));
    connect(actUdp_, &QAction::toggled, this, &MainWindow::toggleUdp);

    actGray_ = new QAction(QStringLiteral("算法灰度"), this);
    actGray_->setCheckable(true);
    actGray_->setIcon(theme::toolIcon(theme::Glyph::Contrast));
    actGray_->setToolTip(QStringLiteral(
        "总开关：默认关=相机原样帧；开=转单通道灰度。\n"
        "生效范围是全部画面消费者：录制原始帧、检测算法(检测算法本来就内部\n"
        "自己转灰度、不受这个开关影响)、预览小窗、标定向导、相机宫格——不止\n"
        "预览一处。"));
    connect(actGray_, &QAction::toggled, this, &MainWindow::toggleGrayOutput);

    // ---- 前置到标签条的三个常用入口 ----
    // 实时动捕是这套软件真正的产出口（聚类→IEKF→骨架→推给 Unity），
    // 原来它叫"点云测试"、埋在「标定」菜单第五项，跟三角化调试这些诊断工具
    // 混在一起 —— 每天要点几十次的东西不该藏那么深。提到标签条右端、给成
    // 唯一的实心强调色按钮，再挂上 F5。
    actRun_ = new QAction(QStringLiteral("实时动捕"), this);
    actRun_->setIcon(theme::toolIcon(theme::Glyph::Play));
    actRun_->setShortcut(QKeySequence(Qt::Key_F5));
    actRun_->setToolTip(QStringLiteral(
        "启动实时动捕（F5）。\n"
        "需要先在「标定模板库…」里选好当前使用的标定模板。\n"
        "打开期间会临时打开反光球检测，关闭后恢复原状态。"));
    connect(actRun_, &QAction::triggered, this, &MainWindow::openLiveMocap);

    actCalib_ = new QAction(QStringLiteral("标定…"), this);
    actCalib_->setIcon(theme::toolIcon(theme::Glyph::Focus));
    connect(actCalib_, &QAction::triggered, this, &MainWindow::openCalibration);

    actCalibLib_ = new QAction(QStringLiteral("标定模板库…"), this);
    actCalibLib_->setToolTip(QStringLiteral(
        "切换/新建/删除标定模板，并指定当前使用的那一套。\n"
        "实时动捕和各项精度验证都读这里选中的模板。"));
    connect(actCalibLib_, &QAction::triggered, this, &MainWindow::openCalibrationLibrary);

    // 参数模板（检测阈值/面积/圆度那一组的预设）。菜单和标签条按钮共用同一个
    // QMenu —— 共用之后就不存在"改了一处忘了另一处"的问题。
    presetMenu_ = new QMenu(QStringLiteral("参数模板"), this);
    presetMenu_->addAction(QStringLiteral("应用「标定板」"), this, [this] { applyPreset(0); });
    presetMenu_->addAction(QStringLiteral("应用「追踪」"),   this, [this] { applyPreset(1); });
    presetMenu_->addSeparator();
    presetMenu_->addAction(QStringLiteral("把当前参数存为「标定板」"), this, [this] { savePreset(0); });
    presetMenu_->addAction(QStringLiteral("把当前参数存为「追踪」"),   this, [this] { savePreset(1); });
}

void MainWindow::buildMenuBar()
{
    auto* mb = menuBar();

    auto* mCam = mb->addMenu(QStringLiteral("相机(&C)"));
    mCam->addAction(QStringLiteral("添加相机…"), this, &MainWindow::addCamera);
    mCam->addAction(QStringLiteral("移除末位相机"), this, &MainWindow::removeCamera);
    mCam->addSeparator();
    mCam->addAction(QStringLiteral("存为开机默认"), this, &MainWindow::saveAsDefault);

    auto* mView = mb->addMenu(QStringLiteral("视图(&V)"));
    // 活动栏那几页在这里也给一份入口，键盘党不用去点图标
    mView->addAction(QStringLiteral("相机"),     this, [this] { selectSidePane(PaneCameras); });
    mView->addAction(QStringLiteral("检测参数"), this, [this] { selectSidePane(PaneDetect); });
    mView->addAction(QStringLiteral("同步监控"), this, [this] { toggleMonitor(true); });
    mView->addAction(QStringLiteral("标定"),     this, [this] { selectSidePane(PaneCalib); });
    mView->addAction(QStringLiteral("工具"),     this, [this] { selectSidePane(PaneTools); });
    mView->addSeparator();
    auto* actToggleSide = mView->addAction(QStringLiteral("显示 / 隐藏侧边栏"), this,
                     [this] { setSideBarVisible(!sideVisible_); });
    actToggleSide->setShortcut(QKeySequence(QStringLiteral("Ctrl+B")));   // 跟 VS Code 一致
    // 开机默认就是最大化（见 main.cpp）；F11 再切到无边框全屏，投外接大屏、
    // 或者想彻底去掉标题栏时用。再按一次还原成最大化，不是缩回小窗。
    mView->addSeparator();
    auto* actFull = mView->addAction(QStringLiteral("全屏 / 还原"), this, [this] {
        if (isFullScreen()) showMaximized();
        else                 showFullScreen();
    });
    actFull->setShortcut(QKeySequence(Qt::Key_F11));

    // ---- 运行：日常真正在用的东西集中在这里 ----
    // 之前「实时动捕」埋在标定菜单里跟诊断工具混着，UDP 推送和参数模板又在
    // 工具菜单里 —— 一次开工要横跨两个菜单。现在按"跑一次动捕需要什么"归拢：
    // 启动、喂给它的参数模板、它的输出开关，一个菜单里全有。
    auto* mRun = mb->addMenu(QStringLiteral("运行(&R)"));
    mRun->addAction(actRun_);
    mRun->addSeparator();
    mRun->addMenu(presetMenu_);
    mRun->addSeparator();
    mRun->addAction(actUdp_);
    mRun->addAction(actGray_);

    auto* mCalib = mb->addMenu(QStringLiteral("标定(&B)"));
    mCalib->addAction(actCalib_);
    mCalib->addAction(actCalibLib_);
    mCalib->addAction(QStringLiteral("相机重映射…"),    this, &MainWindow::openCameraRemap);
    mCalib->addSeparator();
    mCalib->addAction(QStringLiteral("三角化调试…"),      this, &MainWindow::openTriangulationDebug);
    mCalib->addAction(QStringLiteral("标定杆精度验证…"),  this, &MainWindow::openWandPrecision);

    // 手部两项和实验性采集原型并成一个「工具」菜单 —— 手部原来只有两项，
    // 单开一个顶级菜单不值当。
    auto* mTool = mb->addMenu(QStringLiteral("工具(&T)"));
    mTool->addAction(QStringLiteral("手部追踪…"),     this, &MainWindow::openHandTracking);
    mTool->addAction(QStringLiteral("手部标定向导…"), this, &MainWindow::openHandCalibrationWizard);
    mTool->addSeparator();
    mTool->addAction(QStringLiteral("DirectShow采集原型(实验性)…"), this, &MainWindow::openDShowTest);
}

// ---------------------------------------------------------------------------
// 外壳装配：活动栏 | 侧边栏 | 编辑区
// ---------------------------------------------------------------------------
void MainWindow::buildShell()
{
    auto* root = new QWidget;
    auto* h = new QHBoxLayout(root);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(0);

    h->addWidget(buildActivityBar());

    splitter_ = new QSplitter(Qt::Horizontal);
    splitter_->setHandleWidth(1);
    splitter_->setChildrenCollapsible(false);
    splitter_->addWidget(buildSideBar());
    splitter_->addWidget(buildEditorArea());
    splitter_->setStretchFactor(0, 0);   // 侧边栏定宽，拉窗口时把富余给编辑区
    splitter_->setStretchFactor(1, 1);
    splitter_->setSizes({ lastSideWidth_, 900 });
    h->addWidget(splitter_, 1);

    setCentralWidget(root);
}

QWidget* MainWindow::buildActivityBar()
{
    auto* bar = new QWidget;
    bar->setObjectName(QStringLiteral("activityBar"));
    bar->setAttribute(Qt::WA_StyledBackground, true);
    bar->setFixedWidth(48);

    auto* v = new QVBoxLayout(bar);
    v->setContentsMargins(0, 4, 0, 4);
    v->setSpacing(0);

    struct Item { theme::Glyph g; const char* title; const char* tip; };
    const Item items[PaneCount] = {
        { theme::Glyph::Camera,       "相机",     "当前接入的相机；在这里添加/移除" },
        { theme::Glyph::Sliders,    "检测参数", "阈值/面积下限/面积上限/圆度，边拖边看预览十字丝的变化" },
        { theme::Glyph::Gauge,      "同步监控", "各路 fps / 帧数 / 同步偏差 / 检测点数" },
        { theme::Glyph::Focus,     "标定",     "标定向导、模板库、三角化与精度验证" },
        { theme::Glyph::Wrench, "工具",     "手部追踪、参数模板、实验性采集原型" },
    };

    activityActions_.clear();
    for (int i = 0; i < PaneCount; ++i) {
        auto* a = new QAction(QString::fromUtf8(items[i].title), this);
        a->setCheckable(true);
        a->setIcon(theme::activityIcon(items[i].g));
        a->setToolTip(QString::fromUtf8(items[i].tip));
        connect(a, &QAction::triggered, this, [this, i] { selectSidePane(i); });
        activityActions_ << a;

        auto* b = new QToolButton;
        b->setDefaultAction(a);
        b->setIconSize(QSize(24, 24));
        b->setFixedSize(48, 48);
        b->setToolButtonStyle(Qt::ToolButtonIconOnly);
        v->addWidget(b);
    }

    v->addStretch(1);

    // 底部齿轮：跟 VS Code 一样把「设置」放在活动栏最下面
    auto* aSave = new QAction(QStringLiteral("存为开机默认"), this);
    aSave->setIcon(theme::activityIcon(theme::Glyph::Settings));
    aSave->setToolTip(QStringLiteral("记住当前相机布局，下次启动自动恢复"));
    connect(aSave, &QAction::triggered, this, &MainWindow::saveAsDefault);
    auto* bSave = new QToolButton;
    bSave->setDefaultAction(aSave);
    bSave->setIconSize(QSize(24, 24));
    bSave->setFixedSize(48, 48);
    v->addWidget(bSave);

    return bar;
}

QWidget* MainWindow::buildSideBar()
{
    sideBar_ = new QWidget;
    sideBar_->setObjectName(QStringLiteral("sideBar"));
    sideBar_->setAttribute(Qt::WA_StyledBackground, true);
    sideBar_->setMinimumWidth(220);

    auto* v = new QVBoxLayout(sideBar_);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(0);

    // 顶部那行暗淡小字标题
    auto* header = new QWidget;
    header->setObjectName(QStringLiteral("sideBarHeader"));
    header->setAttribute(Qt::WA_StyledBackground, true);
    auto* hh = new QHBoxLayout(header);
    hh->setContentsMargins(0, 0, 6, 0);
    sideTitle_ = new QLabel(QStringLiteral("相机"));
    sideTitle_->setObjectName(QStringLiteral("sideBarTitle"));
    hh->addWidget(sideTitle_);
    hh->addStretch(1);
    v->addWidget(header);

    sideStack_ = new QStackedWidget;
    sideStack_->setObjectName(QStringLiteral("sidePane"));
    sideStack_->setAttribute(Qt::WA_StyledBackground, true);
    // 顺序必须跟 SidePane 枚举一致
    sideStack_->addWidget(makeCamerasPane());     // PaneCameras
    // DetectParamsPanel / SyncMonitor 都是 QWidget 的子类 —— Qt 对自定义子类
    // 默认不画样式表里的背景（这是 QSS 一个很容易踩的坑），必须显式打开
    // WA_StyledBackground，否则这两页会露出编辑区那档更亮的底色。
    detectPanel_->setObjectName(QStringLiteral("sidePane"));
    detectPanel_->setAttribute(Qt::WA_StyledBackground, true);
    sideStack_->addWidget(detectPanel_);          // PaneDetect
    monitor_->setObjectName(QStringLiteral("sidePane"));
    monitor_->setAttribute(Qt::WA_StyledBackground, true);
    sideStack_->addWidget(monitor_);              // PaneMonitor
    sideStack_->addWidget(makeCalibPane());       // PaneCalib
    sideStack_->addWidget(makeToolsPane());       // PaneTools
    v->addWidget(sideStack_, 1);

    return sideBar_;
}

QWidget* MainWindow::makeCamerasPane()
{
    auto* pane = new QWidget;
    pane->setObjectName(QStringLiteral("sidePane"));
    pane->setAttribute(Qt::WA_StyledBackground, true);
    auto* v = new QVBoxLayout(pane);
    v->setContentsMargins(6, 2, 6, 8);
    v->setSpacing(1);

    auto* bAdd = paneButton(QStringLiteral("＋  添加相机"),
                            QStringLiteral("接入摄像头，或添加虚拟测试相机"));
    connect(bAdd, &QPushButton::clicked, this, &MainWindow::addCamera);
    v->addWidget(bAdd);

    auto* bDel = paneButton(QStringLiteral("－  移除末位相机"));
    connect(bDel, &QPushButton::clicked, this, &MainWindow::removeCamera);
    v->addWidget(bDel);

    // 相机页是开机默认停留的一页，把启动按钮也放这儿 —— 接好相机的下一步
    // 就是开跑，不该还要再去别处找入口。
    auto* bRunHere = paneButton(QStringLiteral("▶  实时动捕   F5"));
    connect(bRunHere, &QPushButton::clicked, this, &MainWindow::openLiveMocap);
    v->addWidget(bRunHere);

    v->addWidget(paneGroupLabel(QStringLiteral("已接入")));
    cameraList_ = new QListWidget;
    cameraList_->setSelectionMode(QAbstractItemView::NoSelection);
    cameraList_->setToolTip(QStringLiteral("双击画面放大 · 拖拽换位 · 右键调参"));
    v->addWidget(cameraList_, 1);

    return pane;
}

QWidget* MainWindow::makeCalibPane()
{
    auto* pane = new QWidget;
    pane->setObjectName(QStringLiteral("sidePane"));
    pane->setAttribute(Qt::WA_StyledBackground, true);
    auto* v = new QVBoxLayout(pane);
    v->setContentsMargins(6, 2, 6, 8);
    v->setSpacing(1);

    auto add = [&](const QString& t, void (MainWindow::*slot)()) {
        auto* b = paneButton(t);
        connect(b, &QPushButton::clicked, this, slot);
        v->addWidget(b);
    };

    v->addWidget(paneGroupLabel(QStringLiteral("标定")));
    add(QStringLiteral("标定…"),       &MainWindow::openCalibration);
    add(QStringLiteral("标定模板库…"), &MainWindow::openCalibrationLibrary);
    add(QStringLiteral("相机重映射…"), &MainWindow::openCameraRemap);

    // 参数模板从「工具」页挪到这里：标定板/追踪这两套预设本来就是标定流程的
    // 一部分（标板子时一套参数、跑追踪时另一套），原来跟手部追踪、实验性
    // 采集归在一起纯属分类错了。
    v->addWidget(paneGroupLabel(QStringLiteral("参数模板")));
    auto addPreset = [&](const QString& t, int which, bool save) {
        auto* b = paneButton(t);
        connect(b, &QPushButton::clicked, this,
                [this, which, save] { save ? savePreset(which) : applyPreset(which); });
        v->addWidget(b);
    };
    addPreset(QStringLiteral("应用「标定板」"), 0, false);
    addPreset(QStringLiteral("应用「追踪」"),   1, false);
    addPreset(QStringLiteral("存为「标定板」"), 0, true);
    addPreset(QStringLiteral("存为「追踪」"),   1, true);

    v->addWidget(paneGroupLabel(QStringLiteral("验证")));
    add(QStringLiteral("三角化调试…"),     &MainWindow::openTriangulationDebug);
    add(QStringLiteral("标定杆精度验证…"), &MainWindow::openWandPrecision);

    v->addStretch(1);
    return pane;
}

QWidget* MainWindow::makeToolsPane()
{
    auto* pane = new QWidget;
    pane->setObjectName(QStringLiteral("sidePane"));
    pane->setAttribute(Qt::WA_StyledBackground, true);
    auto* v = new QVBoxLayout(pane);
    v->setContentsMargins(6, 2, 6, 8);
    v->setSpacing(1);

    auto add = [&](const QString& t, void (MainWindow::*slot)()) {
        auto* b = paneButton(t);
        connect(b, &QPushButton::clicked, this, slot);
        v->addWidget(b);
    };

    v->addWidget(paneGroupLabel(QStringLiteral("手部")));
    add(QStringLiteral("手部追踪…"),     &MainWindow::openHandTracking);
    add(QStringLiteral("手部标定向导…"), &MainWindow::openHandCalibrationWizard);

    v->addWidget(paneGroupLabel(QStringLiteral("实验性")));
    add(QStringLiteral("DirectShow采集原型…"), &MainWindow::openDShowTest);

    v->addStretch(1);
    return pane;
}

// 编辑区 = 一条标签条 + 相机宫格。标签条右侧摆最常用的三个开关，
// 其余全进菜单/侧边栏 —— 这样横向再也挤不满。
QWidget* MainWindow::buildEditorArea()
{
    auto* area = new QWidget;
    area->setObjectName(QStringLiteral("editorArea"));
    area->setAttribute(Qt::WA_StyledBackground, true);
    auto* v = new QVBoxLayout(area);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(0);

    auto* tabBar = new QWidget;
    tabBar->setObjectName(QStringLiteral("editorTabBar"));
    tabBar->setAttribute(Qt::WA_StyledBackground, true);
    auto* h = new QHBoxLayout(tabBar);
    h->setContentsMargins(0, 0, 8, 0);
    h->setSpacing(4);

    auto* tab = new QLabel(QStringLiteral("相机预览"));
    tab->setObjectName(QStringLiteral("editorTab"));
    h->addWidget(tab);
    h->addStretch(1);

    // —— 预览显示：只剩"画面"和"阈值掩膜"两档。彩色/灰度不再由这里选，
    // 完全交给"算法灰度"总开关——那才是唯一决定画面消费者(预览/标定/
    // 宫格)拿到彩色还是灰度的地方，见 WebcamCamera::deliverDecodedFrame ——
    h->addWidget(new QLabel(QStringLiteral("预览")));
    previewCombo_ = new QComboBox;
    previewCombo_->addItems({ QStringLiteral("画面"),
                              QStringLiteral("阈值掩膜") });
    previewCombo_->setToolTip(QStringLiteral(
        "画面：正常画面，彩色还是灰度看\"算法灰度\"开关。\n"
        "阈值掩膜：检测器的二值视界——配合阈值滑块边拖边看，固定基于灰度，\n"
        "不受算法灰度开关影响。"));
    connect(previewCombo_, &QComboBox::currentIndexChanged,
            this, &MainWindow::setPreviewModeAll);
    h->addWidget(previewCombo_);

    auto* bGray = new QToolButton;
    bGray->setDefaultAction(actGray_);
    bGray->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    h->addWidget(bGray);

    auto* bUdp = new QToolButton;
    bUdp->setDefaultAction(actUdp_);
    bUdp->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    h->addWidget(bUdp);

    // 细竖线分组，跟菜单里的 separator 是一个意思：左边是"画面怎么显示"，
    // 右边是"要干什么"。
    auto vsep = [] {
        auto* f = new QFrame;
        f->setFrameShape(QFrame::VLine);
        return f;
    };
    h->addWidget(vsep());

    // 参数模板：跟「运行」菜单里挂的是同一个 QMenu 对象
    auto* bPreset = new QToolButton;
    bPreset->setText(QStringLiteral("参数模板 ▾"));
    bPreset->setToolTip(QStringLiteral("在「标定板」和「追踪」两套检测参数之间切换，或把当前参数存成模板。"));
    bPreset->setMenu(presetMenu_);
    bPreset->setPopupMode(QToolButton::InstantPopup);
    h->addWidget(bPreset);

    auto* bCalibLib = new QToolButton;
    bCalibLib->setDefaultAction(actCalibLib_);
    h->addWidget(bCalibLib);

    auto* bCalib = new QToolButton;
    bCalib->setDefaultAction(actCalib_);
    bCalib->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    h->addWidget(bCalib);

    h->addWidget(vsep());

    // 主行动：整个界面唯一的实心强调色按钮
    auto* bRun = new QToolButton;
    bRun->setObjectName(QStringLiteral("runButton"));
    bRun->setDefaultAction(actRun_);
    bRun->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    h->addWidget(bRun);

    v->addWidget(tabBar);
    v->addWidget(gridView_, 1);
    return area;
}

void MainWindow::buildStatusBar()
{
    // 【解码耗时基准】常驻在状态栏最右边，不用 showMessage(那个3秒就消失、
    // 还会跟别的临时提示打架)。addPermanentWidget 放的是右侧固定区域。
    // 每 2s 刷新一次——decodeStatsSnapshot() 只是几个原子读取，2s 一次的频率
    // 对这点开销来说完全谈不上负担，主要是给人眼看的，不需要更频繁。
    // 当前标定模板：实时动捕和各项精度验证全都读它，没选的话一按 F5 就被
    // 弹窗拦下 —— 与其等报错，不如常驻在状态栏上一眼可见。
    calibTplLabel_ = new QLabel;
    calibTplLabel_->setObjectName(QStringLiteral("statusMono"));
    statusBar()->addPermanentWidget(calibTplLabel_);

    decodeStatsLabel_ = new QLabel(QStringLiteral("解码耗时：暂无数据"));
    decodeStatsLabel_->setObjectName(QStringLiteral("statusMono"));   // 数字读数用等宽
    statusBar()->addPermanentWidget(decodeStatsLabel_);
    statusBar()->setSizeGripEnabled(false);
    decodeStatsTimer_ = new QTimer(this);
    connect(decodeStatsTimer_, &QTimer::timeout, this, &MainWindow::refreshDecodeStatsLabel);
    decodeStatsTimer_->start(2000);
}

// ---------------------------------------------------------------------------
// 侧边栏开合。点当前这一页的图标 = 收起（VS Code 就是这个手感）。
// ---------------------------------------------------------------------------
void MainWindow::selectSidePane(int pane)
{
    if (pane < 0 || pane >= PaneCount) return;

    static const char* kTitles[PaneCount] = {
        "相机", "检测参数", "同步 / 调试监控", "标定", "工具"
    };

    if (sideVisible_ && sideStack_->currentIndex() == pane) {
        setSideBarVisible(false);
    } else {
        sideStack_->setCurrentIndex(pane);
        sideTitle_->setText(QString::fromUtf8(kTitles[pane]));
        setSideBarVisible(true);
    }
    for (int i = 0; i < activityActions_.size(); ++i)
        activityActions_[i]->setChecked(sideVisible_ && i == pane);
}

void MainWindow::setSideBarVisible(bool on)
{
    if (!sideBar_) return;
    if (!on && sideBar_->isVisible() && sideBar_->width() > 0)
        lastSideWidth_ = sideBar_->width();       // 收起前记住宽度，展开时原样还原
    sideVisible_ = on;
    sideBar_->setVisible(on);
    if (on)
        splitter_->setSizes({ lastSideWidth_,
                              qMax(320, splitter_->width() - lastSideWidth_) });
    if (!on)
        // std::as_const：Qt 容器是隐式共享的，非 const 的 begin() 会先 detach()
        // ——引用计数 >1 时整份深拷。这里只是读，明确告诉编译器走 const 重载。
        for (QAction* a : std::as_const(activityActions_)) a->setChecked(false);
}

// 侧边栏「相机」页的列表。纯展示，跟宫格里的顺序一致。
void MainWindow::refreshCameraList()
{
    if (!cameraList_) return;
    cameraList_->clear();
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        const bool virt = c->deviceKey() == QStringLiteral("virt");
        // 【多参 arg，不要链式】链式是逐个替换的：第一次填进去的内容若自身
        // 含 "%2"，第二次 .arg() 会把它当占位符再填一次。c->name() 拼自驱动
        // 上报的 iProduct，含不含 % 我们说了不算。多参版本是同时替换。
        cameraList_->addItem(QStringLiteral("%1  %2")
                                 .arg(virt ? QStringLiteral("○") : QStringLiteral("●"),
                                      c->name()));
    }
    if (mgr_->count() == 0)
        cameraList_->addItem(QStringLiteral("（还没有相机）"));
}

// ---------------------------------------------------------------------------
// 以下几个入口原来是构造函数里的内联 lambda，挪成具名槽只是为了让构造函数
// 只管"搭界面"这一件事；函数体是原样搬过来的，行为没有任何改变。
// ---------------------------------------------------------------------------
void MainWindow::openCalibrationLibrary()
{
    CalibrationLibrary lib;
    CalibrationLibraryDialog dlg(&lib, this);
    dlg.exec();
    refreshCalibTemplateLabel();   // 里面很可能换了"当前使用"的那一套
}

void MainWindow::openTriangulationDebug()
{
    CalibrationLibrary lib;
    const QString activeId = lib.activeTemplateId();
    if (activeId.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("三角化调试"),
            QStringLiteral("还没有设为当前使用的标定模板，先去“标定模板库…”里选一个（至少2台相机）。"));
        return;
    }
    std::unique_ptr<CalibrationStore> store(lib.openTemplateStore(activeId));

    // 三角化靠检测跑起来才有 2D 点可用——不开检测点开这个窗口只会
    // 干看着球不动。这里记住"进来之前检测本来是开是关"（现在检测
    // 开关跟 UDP 推送共用同一个 detectOn_），没开就临时开一下；关闭
    // 调试窗口后按记下来的状态恢复，原来关着的重新关掉，原来就开着
    // 的（比如 UDP 推送本来就在跑）保持不动，不影响你原有的开关状态。
    const bool wasDetectOn = detectOn_;
    if (!wasDetectOn) {
        detectOn_ = true;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(true);
    }

    TriangulationDebugDialog dlg(mgr_, store.get(), detectOn_, this);
    dlg.exec();

    if (!wasDetectOn) {
        detectOn_ = false;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(false);
    }
}

void MainWindow::openLiveMocap()
{
    // 【非模态】这一段原来是 PointCloudTestDialog 建在栈上 + dlg.exec()。
    // exec() 是应用级模态：它会禁用其它顶层窗口，于是"实时动捕开着的时候主窗口
    // 整个点不动"。而这两件事本来就该能同时做 —— 一边盯着解算结果，一边翻标定
    // 模板 / 加相机 / 改参数，是日常用法。
    //
    // 改成 show() 之后有两条约束必须一起满足：
    //   ① 窗口不能再是栈对象 —— 它得活到用户自己关掉为止；
    //   ② CalibrationStore 必须活得比对话框久。原来它是本函数的局部 unique_ptr，
    //      随 exec() 返回而析构，正好赶在对话框之后；现在函数立刻返回，得用
    //      shared_ptr 把它钉在对话框的销毁之后。
    //      【不能挂在 MainWindow 成员上】主窗口的成员先于它的子对象析构。
    if (liveMocapDlg_) {                 // 已经开着就不要再开一个
        liveMocapDlg_->raise();
        liveMocapDlg_->activateWindow();
        return;
    }

    CalibrationLibrary lib;
    const QString activeId = lib.activeTemplateId();
    if (activeId.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("实时动捕"),
            QStringLiteral("还没有设为当前使用的标定模板，先去“标定模板库…”里选一个（至少2台相机）。"));
        return;
    }

    // shared_ptr：最后一份引用挂在下面的 destroyed 连接里。空 body 的 lambda
    // 只负责抓着它，连接随对话框析构而销毁 —— store 因此比对话框里的裸指针
    // store_ 活得更久，顺序不会反。
    std::shared_ptr<CalibrationStore> store(lib.openTemplateStore(activeId));
    if (!store) return;

    const bool wasDetectOn = detectOn_;
    if (!wasDetectOn) {
        detectOn_ = true;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(true);
    }

    // 【parent 必须是 nullptr，不能传 this】
    // 传 this 的话它就成了"被主窗口拥有"的窗口，Windows 会强制它永远压在
    // 主窗口之上 —— 表现就是"点后面那个窗口的任务栏按钮，它盖不上来"，
    // 因为拥有者窗口永远在下面。传 nullptr 之后它是彻底的独立顶层窗口，
    // z 序完全由用户点谁来定，跟其它程序的表现一致。
    // 代价是不再被 Qt 自动回收，所以 MainWindow::closeEvent 里要显式关掉它。
    auto* dlg = new PointCloudTestDialog(mgr_, store.get(), detectOn_, nullptr);
    liveMocapDlg_ = dlg;
    dlg->setAttribute(Qt::WA_DeleteOnClose);

    // 【原来 exec() 之后那几行，搬到这里】finished 在 QDialog::done() 里发出、
    // 对话框对象还没析构，此刻动 MainWindow 的成员是安全的。
    // 【必须用 finished 而不是 destroyed】destroyed 是在 ~QObject 里发的，
    // 主窗口自己关闭时它的成员已经先析构完了，那时回碰 detectOn_/mgr_ 是 UB。
    connect(dlg, &QDialog::finished, this, [this, wasDetectOn] {
        liveMocapDlg_ = nullptr;
        if (!wasDetectOn) {
            detectOn_ = false;
            for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(false);
        }
    });
    // 只为了让 store 活到对话框之后（见上面 shared_ptr 的说明），没有别的副作用。
    connect(dlg, &QObject::destroyed, dlg, [store] {});

    // 【把 hm20 骨架链路接到 UDP】原来 udp_ 只接旧的 HandTrackingWorker，
    // 新的 hm20 链路（实时动捕面板里那条）压根没有输出通路 —— 我们在
    // 面板里调的一切都到不了 Unity。包格式(M3DS)和端口都不变。
    connect(dlg, &PointCloudTestDialog::handPoseForUdp,
            udp_, &UdpSender::onHandPoseSmoothed);
    // 分段四元数（M3DQ）。跟 M3DS 同一个端口、不同 magic，互不影响。
    connect(dlg, &PointCloudTestDialog::segmentQuatsForUdp,
            udp_, &UdpSender::onSegmentQuats);
    // 把 UDP 推送的开关/目标告诉面板里的输出监视器 —— "面板在刷数但 Unity
    // 收不到"九成是这个开关没开，直接写在读数面板上，省得来回翻窗口。
    dlg->setUdpStatus(udp_->isEnabled(),
                      QStringLiteral("%1:%2").arg(udp_->address().toString())
                                             .arg(udp_->port()));
    // ---- 块 23：把发包回执接进录制（v8）----
    // 【为什么真值只能从这里来】UdpSender 里有两条静默 return（开关关着、
    // 打包失败），加上 writeDatagram 的返回值原来是被丢掉的。这三件事
    // 在系统内部完全查不到，而下游"收不到数据"时它们跟"发了但对端没配"
    // 完全分不开 —— 会被一路误判成解算问题，往上游白查一整圈。
    connect(udp_, &UdpSender::handPacketSent,
            dlg, &PointCloudTestDialog::onUdpPacketSent);
    dlg->setUdpEndpoint(udp_->isEnabled(), int(udp_->port()));
    dlg->show();
}

void MainWindow::openWandPrecision()
{
    CalibrationLibrary lib;
    const QString activeId = lib.activeTemplateId();
    if (activeId.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("标定杆精度验证"),
            QStringLiteral("还没有设为当前使用的标定模板，先去“标定模板库…”里选一个（至少2台相机）。"));
        return;
    }
    std::unique_ptr<CalibrationStore> store(lib.openTemplateStore(activeId));

    // 复用跟"三角化调试…"完全一样的"临时开检测、关闭时恢复"逻辑——
    // WandPrecisionDialog 内部还会按选的定位模式自己决定要不要额外
    // 打开轮廓采集(圆拟合/融合模式才需要)，那部分它自己管，这里只
    // 管最外层的检测总开关，跟三角化调试是同一个约定。
    const bool wasDetectOn = detectOn_;
    if (!wasDetectOn) {
        detectOn_ = true;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(true);
    }

    WandPrecisionDialog dlg(mgr_, store.get(), detectOn_, this);
    dlg.exec();

    if (!wasDetectOn) {
        detectOn_ = false;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(false);
    }
}

// 【实验性】DirectShow采集原型——见 DShowCapture.hpp 顶部注释，跟正式
// 采集流程(WebcamCamera/CameraManager)完全独立，只用来验证"绕开Qt/
// Media Foundation能不能拿到硬件级帧率"这一件事，不影响现有功能。
void MainWindow::openDShowTest()
{
    DShowCaptureTestDialog dlg(this);
    dlg.exec();
}

void MainWindow::refreshCalibTemplateLabel()
{
    if (!calibTplLabel_) return;
    CalibrationLibrary lib;
    const QString id = lib.activeTemplateId();
    if (id.isEmpty()) {
        calibTplLabel_->setText(QStringLiteral("标定模板：未选择"));
        calibTplLabel_->setToolTip(QStringLiteral(
            "还没有指定当前使用的标定模板。\n"
            "实时动捕、三角化调试、标定杆验证都需要它 —— 去「标定模板库…」里选一个。"));
    } else {
        calibTplLabel_->setText(QStringLiteral("标定模板：%1").arg(id));
        calibTplLabel_->setToolTip(QStringLiteral(
            "当前使用的标定模板。实时动捕和各项精度验证都读这一套内外参。\n"
            "换一套：「标定模板库…」。"));
    }
}

void MainWindow::refreshDecodeStatsLabel()
{
    if (!decodeStatsLabel_) return;
    const auto s = decodeStatsSnapshot();
    if (s.turboGrayFrames == 0 && s.qtColorFrames == 0
        && s.turboGrayFails == 0 && s.qtColorFails == 0) {
        decodeStatsLabel_->setText(QStringLiteral("解码耗时：暂无数据"));
        return;
    }
    QStringList parts;
    if (s.turboGrayFrames > 0)
        parts << QStringLiteral("灰度直解 %1ms/帧(成功%2帧%3)")
                     .arg(s.turboGrayAvgMs, 0, 'f', 2).arg(s.turboGrayFrames)
                     .arg(s.turboGrayFails > 0
                              ? QStringLiteral("，失败%1帧").arg(s.turboGrayFails)
                              : QString());
    if (s.qtColorFrames > 0)
        parts << QStringLiteral("彩色解码 %1ms/帧(成功%2帧%3)")
                     .arg(s.qtColorAvgMs, 0, 'f', 2).arg(s.qtColorFrames)
                     .arg(s.qtColorFails > 0
                              ? QStringLiteral("，失败%1帧").arg(s.qtColorFails)
                              : QString());
    // 只有失败、没有任何成功样本时(比如数据一直是坏的)，也得让人看到，
    // 不能什么都不显示——那样反而像"没在跑"，容易被误判成程序没反应。
    if (s.turboGrayFrames == 0 && s.turboGrayFails > 0)
        parts << QStringLiteral("灰度直解 全部失败(%1帧，坏数据)").arg(s.turboGrayFails);
    if (s.qtColorFrames == 0 && s.qtColorFails > 0)
        parts << QStringLiteral("彩色解码 全部失败(%1帧，坏数据)").arg(s.qtColorFails);
    // 两种数据都有时，顺手把倍率算出来摆在最前面——这才是你真正关心的
    // "到底省了多少"这个问题最直接的答案，不用自己拿两个数字心算。
    QString prefix;
    if (s.turboGrayFrames > 0 && s.qtColorFrames > 0 && s.turboGrayAvgMs > 0.0) {
        const double ratio = s.qtColorAvgMs / s.turboGrayAvgMs;
        prefix = QStringLiteral("解码快 %1× | ").arg(ratio, 0, 'f', 2);
    }
    decodeStatsLabel_->setText(QStringLiteral("解码耗时：%1%2").arg(prefix, parts.join(QStringLiteral("  |  "))));
}

void MainWindow::hookCamera(ICamera* cam)
{
    if (!cam) return;
    // 面板已经建好的话（构造函数走到这一步时一定已经建好），新相机直接继承
    // 面板上当前拖到的值，不再是构造时那个早已过时的 cfg_.defaults 快照。
    cam->setDetectParams(detectPanel_ ? detectPanel_->params()
                                       : DetectParams{ cfg_.defaults.threshold, 3, 2000, 64 });
    cam->setDetectEnabled(detectOn_);
    cam->setGrayOutput(grayOut_);
    cam->setPreviewMode(previewMode_);
    // 检测结果 -> 监控面板 + UDP（预览叠加由 CameraGrid 连到视图）。
    connect(cam, &ICamera::blobsReady, monitor_, &SyncMonitor::onBlobs,
            Qt::UniqueConnection);
    connect(cam, &ICamera::blobsReady, udp_, &UdpSender::onBlobs,
            Qt::UniqueConnection);
    connect(cam, &ICamera::frameReady, monitor_, &SyncMonitor::onFrame,
            Qt::UniqueConnection);

    // ---- 开机默认：硬件参数在采集引擎每次(重)起来之后补推 ----
    // 【为什么不能在建相机时推完就走】WebcamCamera::setFormat() 是排队执行的，
    // 真正跑的 applyFormat() 里是 stopCapture()+startWithFormat()，等于把设备
    // 关掉重开；多数 UVC 驱动在最后一个句柄关闭时把 ProcAmp/CameraControl
    // 复位回默认。推完就走 = 推下去的值必然被随后的重开抹掉。
    // 交互路径同理：调好参数再去改分辨率，一样会被抹掉，接这个信号一并修好。
    //
    // 【必须用成员函数指针，不能用 lambda】Qt::UniqueConnection 对 lambda /
    // functor 无效 —— 官方文档写明"只适用于连接到成员函数"。而且不是"无效
    // 就退化成普通连接"这么轻：Qt 内部剥掉 UniqueConnection 标志位的那一步
    // 在 if (type & UniqueConnection && slot) 里面，functor 的 slot 是 nullptr，
    // 这个位会残留进 connectionType，于是 connectionType == Qt::AutoConnection
    // 的判定不成立，跨线程信号被当成【直连】执行 —— DShowControl 的 COM 调用
    // 会跑到采集线程上去。上面那三条连接用的都是成员函数指针，照着来。
    if (auto* web = qobject_cast<WebcamCamera*>(cam)) {
        connect(web, &WebcamCamera::captureEngineStarted, this,
                &MainWindow::restoreCameraHardwareParams, Qt::UniqueConnection);
    }
}

// 把这台相机保存过的硬件参数推回驱动。只在采集引擎(重)启动完成后调用。
//
// 【两个来源，顺序不能反】
//   1) Boot 模板 —— 用户点「存为开机默认」那一刻的全量快照（17 项属性）。
//   2) AppSettings 里那份曝光 —— 在参数面板拖曝光行时【随手就存】的，
//      所以它总是不早于 Boot 快照。
// 先套 Boot、再套曝光：用户存过开机默认之后又单独调了曝光，下次开机拿到的
// 是他后调的那个值。反过来写就会用旧快照盖掉新调的曝光 —— 而且是静默的，
// 用户只会觉得"曝光又没记住"。
//
// 【为什么曝光还留着单独一份】参数面板的属性行是 valueChanged 触发的，
// 拖一下滑块要触发几十次。QSettings 扛得住(它自己攒着写)，但 Boot 模板落的是
// JSON 全文件重写 —— 每拖一像素重写一次文件不可接受。所以模板只在显式点
// 「存为开机默认」时写一次，曝光那条隐式路径维持原样。
void MainWindow::restoreCameraHardwareParams(quint32 camId)
{
    // 【入参是 camId 不是指针】为的是能用成员函数指针连接（见 hookCamera）。
    // 顺带白捡一层安全：相机在信号排队期间被移除的话，byId 返回 nullptr，
    // 而不是拿着一个已经析构的指针去调 COM。
    auto* web = qobject_cast<WebcamCamera*>(mgr_->byId(camId));
    if (!web) return;

    // 1) 全量快照
    presets_.applyTo(ParamPresets::Boot, web);

    // 2) 之后可能又单独改过的曝光
    const auto p = AppSettings().loadCameraPrefs(web->deviceKey());
    if (!p.hasExposure) return;
    // 【每次都重新读，不用缓存的快照】用户在两次引擎重启之间改过曝光的话，
    // 缓存值会把他刚调的又推回去。
    DShowControl ctrl(web->device().id(), web->device().description(),
                      web->nameOccurrenceIndex());
    if (!ctrl.valid()) return;
    for (const DShowProp& q : ctrl.properties()) {
        if (q.name == QString::fromUtf8("\u66dd\u5149")) {   // 曝光
            ctrl.set(q.id, q.isCameraControl, p.exposureValue, p.exposureAuto);
            break;
        }
    }
}

void MainWindow::restoreCameras()
{
    const auto avail = QMediaDevices::videoInputs();
    missingAtBoot_.clear();
    // as_const：循环体里不改 cfg_.cameraKeys（只往 missingAtBoot_ 里塞），
    // 走 const begin() 既避免 detach，也顺带把"这里只读"写进类型里。
    for (const QString& key : std::as_const(cfg_.cameraKeys)) {
        if (key == QStringLiteral("virt")) { mgr_->addVirtual(cfg_.defaults); continue; }
        if (key.startsWith(QStringLiteral("web:"))) {
            const QString devId = key.mid(4);
            bool found = false;
            for (const QCameraDevice& d : avail)
                if (QString::fromUtf8(d.id()) == devId) {
                    mgr_->addWebcam(d, cfg_.defaults); found = true; break;
                }
            // 【没找到不能静默跳过 —— 这是"模板一次比一次少"的成因】
            // avail 是构造函数里【一次性】取的快照。六台相机挂在 USB Hub 上
            // 冷启动时，PnP 枚举完成得比主窗口构造慢是常事，慢的那台这一轮
            // 就不在 avail 里。原来这里 for 循环找不到就什么都不做，然后
            // closeEvent 又把"当前活着的清单"无条件写回去 —— 那台相机就
            // 被永久从模板里抹掉了，之后每次开机都少一台，而且不会自己回来。
            // 记下来，存盘时保留（见 cameraKeysForSave），并且明确提示用户。
            if (!found) {
                missingAtBoot_ << key;
                qWarning().noquote()
                    << QStringLiteral("[启动] 开机默认里的相机未找到，本次跳过但保留在模板中：%1").arg(key);
            }
        }
    }
}

// 存盘用的相机清单。
//
// 【为什么不能直接用 currentCameraKeys()】那是"此刻活着的相机"，而模板要表达
// 的是"我希望开机有哪些相机"。两者在一种很常见的情况下会分叉：开机时某台
// 相机还没枚举出来。用前者存盘，这台相机就从模板里消失了，而且因为它已经
// 不在模板里，下次开机也不会再去找它 —— 一次偶发的枚举慢，变成永久性丢失。
//
// 规则：按上一份模板的顺序走一遍，活着的照抄；开机没找到、到现在也还没
// 出现的【保留】；本次运行中新加的接在后面。用户在界面上主动删掉的相机
// 既不在 live 里也不在 missingAtBoot_ 里，会被正常移除 —— 主动删除仍然生效。
QStringList MainWindow::cameraKeysForSave() const
{
    QStringList live = currentCameraKeys();
    QStringList out;
    for (const QString& k : cfg_.cameraKeys) {
        const int i = live.indexOf(k);
        if (i >= 0) { out << k; live.removeAt(i); }        // 还活着，位置照旧
        else if (missingAtBoot_.contains(k)) out << k;     // 开机没枚举到，留着
    }
    out += live;                                           // 本次新加的
    return out;
}

QStringList MainWindow::currentCameraKeys() const
{
    QStringList keys;
    for (int i = 0; i < mgr_->count(); ++i) keys << mgr_->at(i)->deviceKey();
    return keys;
}

void MainWindow::addCamera()
{
    // 【非模态】原来是 dlg.exec()。真机上出现过：对话框开着时插 USB 相机
    // -> 整个系统永久无响应；关着插则正常；插非相机 USB 也正常；
    // 把对话框的设备枚举整个关掉仍然卡 —— 说明跟枚举无关，
    // 剩下的唯一差别就是 exec() 的模态循环。
    //
    // 模态时 Qt 会禁用其它顶层窗口并在事件分发层拦截消息，而系统的设备到达
    // 通知是同步发送的：发送方等不到返回就一直挂着，连带堵住整条 PnP 通知链，
    // 于是"整个系统"都无响应，而且不会自己恢复。
    //
    // 改成非模态之后，通知窗口不再被模态拦截。代价是这个函数不能再"等结果"，
    // 所以改用 accepted 信号回调。
    if (addCamDlg_) {                 // 已经开着就不要再开一个
        addCamDlg_->raise();
        addCamDlg_->activateWindow();
        return;
    }
    auto* dlg = new AddCameraDialog(mgr_->usedDeviceKeys(), this);
    addCamDlg_ = dlg;
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    connect(dlg, &QDialog::accepted, this, [this, dlg] {
        // 【必须延到下一轮事件循环再建相机】accepted 是在 QDialog::done()
        // 里面发出的 —— 也就是对话框正在关闭、还没走完析构流程的时候。
        // 在那个时刻去建 DirectShow 滤镜图（会创建窗口、注册回调、
        // 起采集线程）不安全：原来 exec() 那版是在【返回之后】的干净栈上做的。
        //
        // 先把需要的东西拷出来，因为 dlg 带 WA_DeleteOnClose，
        // 排队执行时它已经被 deleteLater 掉了，lambda 里不能再碰它。
        //
        // 【批量】一次可能带回多台，逐个建。这里必须整份拷出来而不是只拷一个。
        const QList<AddCameraDialog::Selection> sel = dlg->selection();
        QMetaObject::invokeMethod(this, [this, sel] {
            // 【排队之后设备可能已经不在了】延迟一轮事件循环把这个窗口变大了：
            // 用户点确定之后、这个 lambda 执行之前，设备可以被拔掉。
            // 而 addWebcam 无条件 nextId_++ 并 start()，不检查设备有效性 ——
            // 结果是一个跑不起来的空相机占着位置，还得手动删掉。
            //
            // 批量时【不做"能加的先加上"】：用户挑的是一组要一起用的相机，
            // 少加几台会让共视/标定关系整个对不上，比整批不作声地残废更糟。
            // 这里仍然是逐台处理，但把失败数明确报出来，让人一眼知道少了谁。
            int added = 0, gone = 0;
            for (const AddCameraDialog::Selection& one : sel) {
                if (!one.isVirtual && one.device.isNull()) { ++gone; continue; }
                const quint32 id = one.isVirtual ? mgr_->addVirtual(cfg_.defaults)
                                                 : mgr_->addWebcam(one.device, cfg_.defaults);
                ICamera* cam = mgr_->byId(id);
                if (!cam) { ++gone; continue; }
                hookCamera(cam);
                ++added;
            }
            if (gone > 0) {
                QMessageBox::warning(this, QString::fromUtf8("添加相机"),
                    QString::fromUtf8("有 %1 台设备在点确定之后就不在了（可能刚被拔掉），没有添加。"
                                      "已经加进来的 %2 台不受影响，重新打开这个窗口可以补上。")
                        .arg(gone).arg(added));
            }
            if (added > 1) {
                statusBar()->showMessage(
                    QString::fromUtf8("已一次性添加 %1 台相机。").arg(added), 5000);
            }
        }, Qt::QueuedConnection);
    });
    // WA_DeleteOnClose 会在关闭时 deleteLater，这里只负责把指针清掉，
    // 否则下次点"添加"会看到一个悬空指针。
    connect(dlg, &QObject::destroyed, this, [this] { addCamDlg_ = nullptr; });
    dlg->show();
}

void MainWindow::removeCamera()
{
    if (mgr_->count() == 0) return;
    mgr_->removeLast();
}

void MainWindow::openCamParams(quint32 id)
{
    ICamera* cam = mgr_->byId(id);
    if (!cam) return;
    CamParamDialog dlg(cam, this);
    dlg.exec();
}

void MainWindow::toggleUdp(bool on)
{
    // UDP 与检测一体：开推送即开检测（预览自动出现十字丝），关则一起关。
    udp_->setEnabled(on);
    detectOn_ = on;
    for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(on);
    // 【两个窗口现在能同时操作了】主窗口按下这个开关时，实时动捕窗口上的
    // "UDP 推送：开/关"必须跟着变 —— 否则又回到"面板在刷数但 Unity 收不到、
    // 还看不出为什么"的老问题，而且这次两边还都点得动。
    if (liveMocapDlg_) {
        liveMocapDlg_->setUdpStatus(on,
            QStringLiteral("%1:%2").arg(udp_->address().toString()).arg(udp_->port()));
        liveMocapDlg_->setUdpEndpoint(on, int(udp_->port()));
    }
    statusBar()->showMessage(on
        ? QStringLiteral("检测 + UDP 推送已开启 -> %1:%2（M2D0 协议）")
              .arg(udp_->address().toString()).arg(udp_->port())
        : QStringLiteral("检测 + UDP 推送已关闭"), 3000);
}

void MainWindow::toggleMonitor(bool on)
{
    // 监控面板现在是侧边栏的一页（不再是浮动 dock）。开=切到那一页并展开
    // 侧边栏；关=只在当前正停在监控页时才收起，避免把用户正在看的别的页
    // 一起关掉。
    if (on) {
        sideStack_->setCurrentIndex(PaneMonitor);
        sideTitle_->setText(QStringLiteral("同步 / 调试监控"));
        setSideBarVisible(true);
    } else if (sideStack_->currentIndex() == PaneMonitor) {
        setSideBarVisible(false);
    }
    for (int i = 0; i < activityActions_.size(); ++i)
        activityActions_[i]->setChecked(sideVisible_ && sideStack_->currentIndex() == i);
}

void MainWindow::toggleGrayOutput(bool on)
{
    // 一个开关、两级生效，从源头到下游全走灰度：
    // 1) setGrayDecode：让 MJPG 在【解码源头】就直接解成单通道灰度(编了
    //    libjpeg-turbo 时跳过色度+颜色矩阵，是真正省 CPU 的地方)。打开后
    //    DShowCapture/onFrame 吐出来的帧本身就是 Grayscale8。
    // 2) setGrayOutput(grayOut_)：下游各消费者的“解码后转灰度”兜底。源头已经
    //    是灰度时，deliverDecodedFrame 里的 convertToFormat 会因格式已是
    //    Grayscale8 而整个跳过——所以这级在开了源头直解后基本零成本，留着是为了
    //    覆盖“源头没走 turbo/是彩色兜底帧”的情况，保证承诺一致。
    setGrayDecode(on);
    grayOut_ = on;
    for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setGrayOutput(on);
    // 切开关就清零统计——避免"开关刚打开、状态栏却还显示着关闭时段攒的旧
    // 平均值"这种误导人的情况，清零后立刻开始攒新数据，2s 内会看到刷新。
    resetDecodeStats();

    const QString turboNote = turboAvailable()
        ? QStringLiteral("（源头灰度直解已启用）")
        : QStringLiteral("（未编译 libjpeg-turbo，回退解码后转灰度，功能同、不省 CPU）");
    statusBar()->showMessage(on
        ? QStringLiteral("画面通道：单通道灰度 ") + turboNote
        : QStringLiteral("画面通道：相机原样帧（你调好的画面）"), 3000);
}

// 相机重映射入口：重插拔/换 USB 口后，实时相机的 deviceKey 变了、跟模板里
// 已标定的旧 key 对不上，标定"看起来失效"。这里把当前实时相机手动/半自动
// 对应回模板里已标定的相机，映射写进 CalibrationStore 的 remap 表（持久化，
// 不污染标定数据），三角化立刻按新映射取标定，不用重标。
void MainWindow::openCameraRemap()
{
    CalibrationLibrary lib;
    const QString activeId = lib.activeTemplateId();
    if (activeId.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("相机重映射"),
            QStringLiteral("还没有设为当前使用的标定模板——先去“标定模板库…”里选一个，"
                           "再来把当前相机对应回它标定过的相机。"));
        return;
    }
    // 重映射要读别名、写 remap，都在这个模板的 store 上进行；用成员保存，
    // 保证对话框存在期间 store 生命周期够长（对话框构造签名要求 store*
    // 活得比对话框久）。
    remapStore_.reset(lib.openTemplateStore(activeId));
    if (!remapStore_) return;

    // 模板里“已标定”的相机旧 key（未标定的没有重映射的意义，不列出来）。
    QVector<QString> templateKeys;
    for (const QString& k : remapStore_->keys())
        if (remapStore_->get(k).isCalibrated()) templateKeys << k;
    if (templateKeys.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("相机重映射"),
            QStringLiteral("当前模板里没有已标定的相机，没什么可映射的。"));
        return;
    }

    // 当前接入的实时相机（排除虚拟相机，它没有物理 deviceKey，映射无意义）。
    QVector<ICamera*> live;
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        if (c->deviceKey() == QStringLiteral("virt")) continue;
        live << c;
    }
    if (live.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("相机重映射"),
            QStringLiteral("当前没有接入真实相机，无法映射。先接上相机再来。"));
        return;
    }

    CameraRemapDialog dlg(live, templateKeys, remapStore_.get(), this);
    if (dlg.exec() == QDialog::Accepted) {
        statusBar()->showMessage(
            QStringLiteral("相机映射已更新——重新打开“三角化调试…”即可按新映射取标定，无需重标。"),
            5000);
    }
    // remap 已在对话框 onAccept 里 setRemap() 落盘；这个临时 store 用完即可
    // 释放。下次开三角化调试会用 openTemplateStore 重新打开同一个模板，
    // 构造时 loadRemap() 会把刚写的映射读回来，三角化据此取标定。
    remapStore_.reset();
}

// 手部动捕调试入口：跟"三角化调试…"是同一套前提（需要一个已标定的模板 +
// 检测跑起来才有2D点可用），复用同样的"借/还检测开关"处理——进来前检测
// 本来关着就临时打开，关闭窗口后按原状态恢复，不影响你原有的开关状态。
//
// HandTrackingWorker 在这里是栈上对象，随 exec() 结束、函数返回而析构；
// 它连到各相机 ICamera::blobsReady 的连接会在析构时被 Qt 自动断开
// （接收方销毁即断开），相机本身的生命周期由 mgr_ 管，不受影响。这跟
// "三角化调试…"里 TriangulationDebugDialog 的临时构造/析构是同一个思路，
// 不需要额外手动管理。
void MainWindow::openHandTracking()
{
    CalibrationLibrary lib;
    const QString activeId = lib.activeTemplateId();
    if (activeId.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("手部追踪"),
            QStringLiteral("还没有设为当前使用的标定模板，先去“标定模板库…”里选一个"
                          "（建议至少3台已标定相机——手背5点冷启动模板匹配和后续遮挡"
                          "互相印证都靠多相机协同，2台也能跑但鲁棒性会明显下降）。"));
        return;
    }
    std::unique_ptr<CalibrationStore> store(lib.openTemplateStore(activeId));
    if (!store) return;

    const bool wasDetectOn = detectOn_;
    if (!wasDetectOn) {
        detectOn_ = true;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(true);
    }

    // 传全部相机（含虚拟相机）进去，未标定的会被 HandTrackingWorker 内部
    // 按 deviceKey 自动过滤掉，不需要在这里手动挑——跟 webcams() 过滤"真实
    // 相机"的用途不一样，这里要的是"能查到标定的"，两个筛选条件不等价。
    QVector<ICamera*> cams;
    for (int i = 0; i < mgr_->count(); ++i) cams << mgr_->at(i);

    HandTrackingWorker worker(cams, store.get(), &handTemplateStore_, this);
    // 延迟精修流(handPoseSmoothedReady)转发给下游UDP——这条连接不依赖
    // "延迟精修流开关"是否打开：没开的话 worker 根本不会 emit 这个信号
    // (见 HandTrackingWorker::setSmootherEnabled)，也不依赖 udp_ 是否
    // enabled(UdpSender::onHandPoseSmoothed 内部会自己检查)，所以这里
    // 常驻连接、无副作用，用户在调参面板里勾"启用延迟精修流"那一刻起，
    // 下游才会真正开始收到 M3DS 包。
    connect(&worker, &HandTrackingWorker::handPoseSmoothedReady, udp_, &UdpSender::onHandPoseSmoothed);
    if (worker.calibratedCount() < 2) {
        QMessageBox::information(this, QStringLiteral("手部追踪"),
            QStringLiteral("当前模板里已标定的相机不足2台（现有%1台），无法关联/估计手部位姿。")
                .arg(worker.calibratedCount()));
    } else {
        if (worker.calibratedCount() < 3) {
            statusBar()->showMessage(QStringLiteral(
                "提示：只有 %1 台已标定相机，手背冷启动的鲁棒性会下降，建议 >= 3 台。")
                .arg(worker.calibratedCount()), 5000);
        }
        HandPoseDebugDialog dlg(&worker, this);
        dlg.exec();
    }

    if (!wasDetectOn) {
        detectOn_ = false;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(false);
    }
}

// 手部标定向导入口：跟"手部追踪…"共用同一套前提(已标定模板 + 检测跑起来)，
// 也是同样的"借/还检测开关"处理。向导需要一个 HandTrackingWorker 实例来
// 拿候选3D点(步骤1)和当前追踪的手腕位姿(步骤2转局部系用)——这里独立
// 新建一个，跟"手部追踪…"窗口各自持有各自的 worker，不共享同一个实例
// (两个窗口不会同时开，各自独立冷启动一次即可，不需要更复杂的共享生命周
// 期管理)。向导内部通过 store_->reloadTemplate() 让标定结果对这个 worker
// 立刻生效；下次再打开"手部追踪…"会重新构造 worker，从 handTemplateStore_
// 读到的自然就是标定过的最新数据。
void MainWindow::openHandCalibrationWizard()
{
    CalibrationLibrary lib;
    const QString activeId = lib.activeTemplateId();
    if (activeId.isEmpty()) {
        QMessageBox::information(this, QStringLiteral("手部标定向导"),
            QStringLiteral("还没有设为当前使用的标定模板，先去“标定模板库…”里选一个"
                          "（建议至少3台已标定相机）。"));
        return;
    }
    std::unique_ptr<CalibrationStore> store(lib.openTemplateStore(activeId));
    if (!store) return;

    const bool wasDetectOn = detectOn_;
    if (!wasDetectOn) {
        detectOn_ = true;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(true);
    }

    QVector<ICamera*> cams;
    for (int i = 0; i < mgr_->count(); ++i) cams << mgr_->at(i);

    HandTrackingWorker worker(cams, store.get(), &handTemplateStore_, this);
    if (worker.calibratedCount() < 2) {
        QMessageBox::information(this, QStringLiteral("手部标定向导"),
            QStringLiteral("当前模板里已标定的相机不足2台（现有%1台），无法三角化出候选点，标定没法做。")
                .arg(worker.calibratedCount()));
    } else {
        HandCalibrationWizard dlg(&worker, &handTemplateStore_, this);
        dlg.exec();
    }

    if (!wasDetectOn) {
        detectOn_ = false;
        for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setDetectEnabled(false);
    }
}

void MainWindow::openCalibration()
{
    // 每次标定都是独立的新模板（不再合并/覆盖旧结果）——CalibWizard 本身
    // 完全没改，只是这次给它的 CalibrationStore* 指向一个刚建好的空白
    // 模板文件夹，向导原样往里写，天然就是"每次一个新文件"。这也顺带
    // 从根源上解决了幽灵相机残留：新模板从空白开始，不存在"旧记录清不掉"
    // 这回事。标完之后去工具栏"标定模板库…"里管理/改名/设为当前使用。
    CalibrationLibrary lib;
    std::unique_ptr<CalibrationStore> store(lib.beginNewTemplateStore());
    CalibWizard dlg(mgr_, store.get(), &presets_, this);
    dlg.exec();
    refreshCalibTemplateLabel();   // 标完可能就成了"当前使用"的那一套
}

QList<WebcamCamera*> MainWindow::webcams() const
{
    QList<WebcamCamera*> out;
    for (int i = 0; i < mgr_->count(); ++i)
        if (auto* w = qobject_cast<WebcamCamera*>(mgr_->at(i))) out << w;
    return out;
}

void MainWindow::applyPreset(int which)
{
    const auto cams = webcams();
    if (cams.isEmpty()) {
        statusBar()->showMessage(QStringLiteral("没有真实相机可应用模板"), 3000);
        return;
    }
    const auto w = which == 0 ? ParamPresets::Board : ParamPresets::Track;
    if (!presets_.has(w)) {
        statusBar()->showMessage(QStringLiteral(
            "模板「%1」还没保存过：先把画面调满意，再点 存为 ▾")
            .arg(which == 0 ? QStringLiteral("标定板") : QStringLiteral("追踪")), 5000);
        return;
    }
    const int n = presets_.applyAll(w, cams);
    statusBar()->showMessage(QStringLiteral("已应用「%1」参数到 %2/%3 台相机")
        .arg(which == 0 ? QStringLiteral("标定板") : QStringLiteral("追踪"))
        .arg(n).arg(cams.size()), 4000);
}

void MainWindow::savePreset(int which)
{
    const auto cams = webcams();
    if (cams.isEmpty()) {
        statusBar()->showMessage(QStringLiteral("没有真实相机可保存"), 3000);
        return;
    }
    const auto w = which == 0 ? ParamPresets::Board : ParamPresets::Track;
    const int n = presets_.captureAll(w, cams);
    statusBar()->showMessage(QStringLiteral("已把 %1 台相机的当前参数存为「%2」模板（重启依然生效）")
        .arg(n)
        .arg(which == 0 ? QStringLiteral("标定板") : QStringLiteral("追踪")), 4000);
}

void MainWindow::setPreviewModeAll(int mode)
{
    previewMode_ = mode;
    for (int i = 0; i < mgr_->count(); ++i) mgr_->at(i)->setPreviewMode(mode);
}

// 「存为开机默认」= 把此刻的整套状态快照下来，开机原样恢复。
//
// 【改之前它名不副实】原来只存相机清单 + 窗口几何 + 三个应用层值
// (threshold/target_fps/show_overlay)，【完全不碰】每台相机的分辨率帧率和
// 参数面板里那 17 项硬件属性。那两样是"顺手存的"：格式在你点「应用格式」
// 那一刻才存，硬件属性里只有「曝光」在你拖那一行时才存，其余 16 项
// (亮度/对比度/色调/饱和度/清晰度/伽玛/启用颜色/白平衡/逆光对比/增益/
//  全景/倾斜/滚动/缩放/光圈/焦点) 一项都不存。
// 于是用户调好一整套画面、点了「存为开机默认」、下次开机发现只回来一部分。
void MainWindow::saveAsDefault()
{
    cfg_.cameraKeys = cameraKeysForSave();
    cfg_.windowGeometry = saveGeometry();
    settings_.save(cfg_);

    const auto cams = webcams();

    // ---- 硬件属性：全量快照进 Boot 模板 ----
    // 走 ParamPresets 而不是往 AppSettings 里再开一套 schema：它本来就是
    // "按 deviceKey 存整份 DirectShow 属性表、落 JSON"，Board/Track 两套已经
    // 在用、已经验证过，加个槽位就行。两套存储各存一半才是麻烦的开始。
    const int nProps = presets_.captureAll(ParamPresets::Boot, cams);

    // ---- 分辨率/帧率：逐台存当前生效值 ----
    // 存的是 currentFormat() 而不是下拉框选的值 —— 用户可能改了下拉框没点
    // 「应用格式」，那种情况下画面上跑的还是旧格式，快照要跟画面一致。
    AppSettings st;
    int nFmt = 0;
    for (WebcamCamera* c : cams) {
        const QCameraFormat f = c->currentFormat();
        if (f.isNull()) continue;   // 引擎还没起来，读不到就不写，别拿空值把好的覆盖了
        // 第 4 个实参是【格式身份】(标称上限，开机靠它认回同一条格式)，
        // 第 5 个才是【用户要的帧率】。两者混用会让格式恢复整个失效，
        // 见 AppSettings::CameraFormatExposurePrefs 的注释。
        st.saveCameraFormat(c->deviceKey(), f.resolution(), int(f.pixelFormat()),
                            f.maxFrameRate(), c->requestedFrameRate());
        ++nFmt;
    }

    // ---- 说清楚到底存进去多少 ----
    // 【不报"已保存"就完事】这个功能之前就是因为静默才让人以为存上了。
    // 相机数、属性快照数、格式数不一致时用户能立刻看出来哪一步没成
    // （典型：某台相机被别的程序独占，DShowControl 绑不上，属性就少一台）。
    QString msg = QStringLiteral("已存为开机默认：%1 台相机，硬件参数 %2 台，格式 %3 台")
                      .arg(cams.size()).arg(nProps).arg(nFmt);
    if (nProps < cams.size() || nFmt < cams.size())
        msg += QStringLiteral("（有相机没存上，多半是设备被占用或还没出图）");
    if (!missingAtBoot_.isEmpty())
        msg += QStringLiteral("；另有 %1 台本次开机未枚举到，已保留在模板中")
                   .arg(missingAtBoot_.size());
    statusBar()->showMessage(msg, 8000);
}

void MainWindow::closeEvent(QCloseEvent* e)
{
    // 【实时动捕窗口要在这里确定性地拆掉】它没有 parent（为了独立 z 序 + Aero
    // Snap，见 openLiveMocap），Qt 不会随主窗口回收它。两件事必须做全：
    //   ① close() —— 走完它自己的 finished 回调（清 liveMocapDlg_、还原检测
    //      开关）。不做的话主窗口关了它还挂在桌面上，而 main.cpp 关掉了
    //      quitOnLastWindowClosed，进程就成了没有主窗口的"僵尸"。
    //   ② delete —— 只 close() 的话，WA_DeleteOnClose 排的是 deleteLater，
    //      而下面紧接着就是 mgr_->clear() + quit()，事件循环等不到那一轮，
    //      对象会一直活到进程结束（泄漏），而且是在所有相机都已析构之后还存在。
    //      趁相机还活着直接析构，顺序就永远是"窗口先走、相机后走"。
    //      close() 已经触发过 finished，指针那时就被清成 nullptr 了，所以先用
    //      一个局部变量接住再删。deleteLater 与这个 delete 不冲突 ——
    //      ~QObject 会把挂在自己名下的待投递事件一并撤掉。
    if (PointCloudTestDialog* d = liveMocapDlg_) {
        d->close();
        delete d;
        liveMocapDlg_ = nullptr;      // finished 回调里已经清过一次，这里是兜底
    }

    cfg_.cameraKeys     = cameraKeysForSave();
    cfg_.windowGeometry = saveGeometry();
    settings_.save(cfg_);
    mgr_->clear();
    QMainWindow::closeEvent(e);
    // main.cpp 里关掉了 QApplication::setQuitOnLastWindowClosed（防止子
    // 弹窗关闭时误触发整个程序退出），所以这里必须显式退出——否则关掉
    // 主窗口后进程会变成没有任何窗口、但仍在后台挂着的"僵尸"状态。
    QApplication::quit();
}

} // namespace mocap