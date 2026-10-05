// ===========================================================================
// PointCloudTestDialog_panel.cpp —— PointCloudTestDialog::buildParamSections() 的实现。
// 从 PointCloudTestDialog.cpp 的构造函数拆出（原第 632~2379 行 + addSep 定义 596~610 行）：
// 构造右侧参数侧栏里所有“可折叠小节/行/控件”，并把读数类标签放进底部状态条。
// 仍是一个普通成员函数，靠 ui/PointCloudTestDialog.hpp 的类声明与其它 .cpp 共享状态。
// ===========================================================================
#include "ui/PointCloudTestDialog.hpp"
#include "ui/Theme.hpp"
#include "camera/CameraManager.hpp"
#include "camera/ICamera.hpp"

// 【版本闸】本文件用到了 Hm20DiagRec 里较新的字段，以及 v4 才有的
// JointOutRec / HandednessRec / MarkerDbgRec / RunFlagsRec 四个块。
// 如果 PointCloudRecorder.hpp 是旧版，下面这句会给出一条能读懂的错误，
// 而不是一串 "has no member named"。触发了就说明少拷了文件。
static_assert(mocap::pcrec::kHeaderRevision >= 6,
              "src/record/PointCloudRecorder.hpp 版本太旧：本文件需要 v4 的 "
              "JointOutRec / HandednessRec / MarkerDbgRec / RunFlagsRec 四个块。"
              "这四个文件必须一起更新："
              "record/PointCloudRecorder.hpp, ui/PointCloudTestDialog.cpp, "
              "estimate/SkeletonAssocWorker.hpp, estimate/HandSkeletonAssociator.hpp");
#include "calib/CalibrationStore.hpp"
#include "reconstruct/Triangulation.hpp"   // undistortNormalize
#include "hand/HandTemplateStore.hpp"
#include "estimate/HandSkeletonAssociator.hpp"
#include "estimate/Hm20TemplateAdapter.hpp"
#include "estimate/SkeletonAssocWorker.hpp"
#include "estimate/Hm20IkRefiner.hpp"
#include "ui/HandOutputMonitor.hpp"   // 【新增】输出数据实时打印面板
#include <QFrame>        // 状态条的 1px 竖分隔线
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QApplication>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPainter>
#include <QPen>
#include <QFont>
#include <QColor>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QCheckBox>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineEdit>
#include <QMessageBox>
#include <QStandardPaths>
#include <QComboBox>
#include <QPushButton>
#include <QSplitter>
#include <QScrollArea>
#include <QToolButton>
#include <QGuiApplication>
#include <QScreen>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QUrl>
#include <QDir>
#include <QCryptographicHash>
#include <QSysInfo>
#include <QFileInfo>
#include <QLayout>
#include <QToolTip>
#include <QHelpEvent>
#include <QPainter>
#include <QLayoutItem>
#include <memory>
#include <cmath>
#include <algorithm>
#include <limits>

#include "ui/PointCloudTestDialog_widgets.hpp"

namespace mocap {

void PointCloudTestDialog::buildParamSections(QVBoxLayout* ctlCol,
                                                QHBoxLayout* quickRow,
                                                QHBoxLayout* statusStrip) {
    // ============ 原构造函数第 596~610 行：状态条分组竖线 ============
    // 状态条分组用的 1px 竖线。跟 Theme 的风格要点一致：分隔靠细线不靠留白。
    // 【为什么读数条特别需要分隔】这一条上并排着四组来源完全不同的读数
    // （滞后/连接状态/编号稳定性/耗时），它们的数字长得很像（都是"名字+数字+
    // 单位"），挤在一起时眼睛分不出哪几个字是一组的 —— 截图里就是这个问题。
    // 竖线一画，四组的边界立刻清楚。
    auto addSep = [statusStrip]() {
        auto* line = new QFrame;
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Plain);
        line->setFixedWidth(1);
        line->setStyleSheet(QStringLiteral("color:#2B2B2B;"));   // theme::lineSoft
        statusStrip->addSpacing(4);
        statusStrip->addWidget(line);
        statusStrip->addSpacing(4);
    };

    // ============ 原构造函数第 632~2379 行：分节机制 + 各小节填充 ============
    // ---- 分节机制 ----
    // v 指向"当前小节的内容布局"，backendRow 指向"当前小节里正在填的那一行"。
    // 下面几千行原有的 v->addWidget / backendRow->addWidget 一个字都不用改，
    // 只是在主题切换处重新指一下这两个指针，控件就自动落到对的小节里。
    QVBoxLayout* v = nullptr;
    FlowLayout*  backendRow = nullptr;
    QVBoxLayout* secCluster = nullptr;      // 需要跨段回填的几个小节，存下来备用
    QVBoxLayout* secBackend = nullptr;
    QVBoxLayout* secSkeleton = nullptr;
    QVBoxLayout* secTime = nullptr;

    auto beginSection = [&](const QString& title, const QString& key,
                            bool openByDefault) -> QVBoxLayout* {
        // box 也无父，但它在 addWidget 之前没有任何 setVisible，所以没闪。
        // 这里不改成 new QWidget(ctlCol的宿主)，是因为 ctlCol 是个 QVBoxLayout、
        // 拿不到宿主控件；addWidget 时 Qt 会自动 reparent，行为是对的。
        auto* box = new QWidget;
        auto* bv = new QVBoxLayout(box);
        bv->setContentsMargins(0, 0, 0, 0);
        bv->setSpacing(0);

        const QString skey = QStringLiteral("sec.") + key;
        const bool open = settings_.value(skey, openByDefault).toBool();

        // head 同样原来是无父的。它没闪是因为中间没有 setVisible(true)，
        // 属于"侥幸"而不是"正确"——顺手一起收编，免得以后有人加一行就复发。
        auto* head = new QToolButton(box);
        head->setObjectName(QStringLiteral("panelSectionHead"));
        head->setCheckable(true);
        head->setChecked(open);
        head->setText(title);
        head->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        head->setArrowType(open ? Qt::DownArrow : Qt::RightArrow);
        head->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        bv->addWidget(head);

        // 【必须先给父对象，再 setVisible】原来这里是 `new QWidget;`（无父），
        // 然后在 addWidget() 之【前】就调了 setVisible(open)。
        //
        // Qt 里【无父的 QWidget 就是一个顶层窗口】。所以 open 为真时，
        // 这一句等于"把一个 22x16 的空窗口显示出来"——屏幕上真的会闪出一个
        // 带标题栏的小方框，紧接着 addWidget() 把它收编成子控件、窗口消失。
        // 每个折叠小节闪一次，实测点开「实时动捕」会连闪 11 次，
        // 然后主窗口才出来。ShowSpy 抓到的就是这一串：
        //     [显示] 类=QWidget 名=panelSectionBody 几何=640,276 22x16 父=(无父=顶层)
        //     [隐藏] 类=QWidget 名=panelSectionBody ...   ×11
        //
        // 两处改动，缺一不可：
        //   ① new QWidget(box) —— 建的时候就有父，从头到尾都是子控件
        //   ② setVisible 挪到 addWidget 之后 —— 就算将来又有人把父去掉，
        //      进了布局的控件也不会再变成顶层窗口
        auto* body = new QWidget(box);
        body->setObjectName(QStringLiteral("panelSectionBody"));
        body->setAttribute(Qt::WA_StyledBackground, true);
        auto* bodyLay = new QVBoxLayout(body);
        bodyLay->setContentsMargins(12, 6, 10, 10);
        bodyLay->setSpacing(6);
        bv->addWidget(body);
        body->setVisible(open);

        connect(head, &QToolButton::toggled, this, [this, head, body, skey](bool on) {
            body->setVisible(on);
            head->setArrowType(on ? Qt::DownArrow : Qt::RightArrow);
            settings_.setValue(skey, on);
        });

        ctlCol->addWidget(box);
        v = bodyLay;
        return bodyLay;
    };

    // 在当前小节里另起一行。原来所有控件挤在同一条 QHBoxLayout 上，正是
    // 横向撑爆的根源；现在每几个相关控件一行，纵向堆叠。
    auto newRow = [&]() -> FlowLayout* {
        auto* r = new FlowLayout();
        v->addLayout(r);
        return r;
    };
    v = beginSection(QStringLiteral("相机"), QStringLiteral("cams"), true);
    {
        // 侧栏是定宽的，这段长提示必须换行 —— 不然它会一直往右顶，把整个
        // 侧栏撑出横向滚动条（截图上就是被切掉半句的那行）。
        auto* camHint = new QLabel(QStringLiteral(
        "勾选参与聚类的相机（>=2台，看到越多台越能有效抑制幽灵点——见\n"
        "MultiViewCluster.hpp顶部说明，三视角以上才能真正验证一个候选点）："));
        camHint->setWordWrap(true);
        v->addWidget(camHint);
    }

    list_ = new QListWidget;
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        if (c->deviceKey() == QStringLiteral("virt")) continue;
        const bool calibrated = store_->get(c->deviceKey()).isCalibrated();
        auto* item = new QListWidgetItem(
            QString("%1  %2").arg(calibrated ? QStringLiteral("●") : QStringLiteral("○"), c->name()),
            list_);
        item->setData(Qt::UserRole, c->id());
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(calibrated ? Qt::Checked : Qt::Unchecked);
        if (!calibrated) item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }
    list_->setMaximumHeight(130);
    v->addWidget(list_);

    // ---- 全选 / 全不选 ----
    // 相机多起来之后逐个勾很烦，尤其是"先全关、只留两台对比一下"这种
    // 排查动作，要点十几下。
    // 【只作用于可勾选的项】没标定的相机是 disabled 的（上面 setFlags 去掉了
    // ItemIsEnabled），全选不该把它们也勾上 —— 那会让"参与聚类的相机数"
    // 变成一个下游算不出来的值。
    {
        auto* selRow = new FlowLayout();
        auto* btnAll = new QPushButton(QStringLiteral("全选"));
        auto* btnNone = new QPushButton(QStringLiteral("全不选"));
        btnAll->setToolTip(QStringLiteral("勾选全部【已标定】的相机。未标定的相机不会被勾上——\n"
                                          "它们没有内外参，参与聚类只会拖累结果。"));
        btnNone->setToolTip(QStringLiteral("取消勾选全部相机。用来做\"只留两台看看\"这类对比。"));
        auto setAll = [this](bool on) {
            for (int i = 0; i < list_->count(); ++i) {
                QListWidgetItem* it = list_->item(i);
                if (!(it->flags() & Qt::ItemIsEnabled)) continue;   // 未标定的跳过
                it->setCheckState(on ? Qt::Checked : Qt::Unchecked);
            }
        };
        connect(btnAll,  &QPushButton::clicked, this, [setAll]{ setAll(true);  });
        connect(btnNone, &QPushButton::clicked, this, [setAll]{ setAll(false); });
        selRow->addWidget(btnAll);
        selRow->addWidget(btnNone);
        v->addLayout(selRow);
    }

    secCluster = beginSection(QStringLiteral("多视角聚类"), QStringLiteral("cluster"), true);
    v = secCluster;
    auto* paramRow = new FlowLayout();
    paramRow->addWidget(new QLabel(QStringLiteral("maxSampson：")));
    maxSampsonSpin_ = new QDoubleSpinBox;
    maxSampsonSpin_->setRange(0.001, 2.0);
    maxSampsonSpin_->setSingleStep(0.001);
    maxSampsonSpin_->setDecimals(4);
    maxSampsonSpin_->setValue(settings_.value(QStringLiteral("maxSampson"), 0.02).toDouble());
    maxSampsonSpin_->setToolTip(QStringLiteral(
        "两视图种子阶段的极线剪枝阈值——越小越严格，漏检真点的风险越高；\n"
        "越大越宽松，幽灵点越容易混进来。"));
    paramRow->addWidget(maxSampsonSpin_);
    paramRow->addWidget(new QLabel(QStringLiteral("maxReprojNorm：")));
    maxReprojSpin_ = new QDoubleSpinBox;
    maxReprojSpin_->setRange(0.001, 2.0);
    maxReprojSpin_->setSingleStep(0.001);
    maxReprojSpin_->setDecimals(4);
    maxReprojSpin_->setValue(settings_.value(QStringLiteral("maxReprojNorm"), 0.03).toDouble());
    paramRow->addWidget(maxReprojSpin_);
    paramRow->addWidget(new QLabel(QStringLiteral("最少支持视角：")));
    minSupportSpin_ = new QSpinBox;
    minSupportSpin_->setRange(2, 32);
    minSupportSpin_->setValue(settings_.value(QStringLiteral("minSupportViews"), 3).toInt());
    paramRow->addWidget(minSupportSpin_);
    useVotingChk_ = new QCheckBox(QStringLiteral("多相机投票"), this);
    useVotingChk_->setChecked(settings_.value(QStringLiteral("useVoting"), true).toBool());
    paramRow->addWidget(useVotingChk_);
    paramRow->addWidget(new QLabel(QStringLiteral("最小视线夹角(°)：")));
    minRayAngleSpin_ = new QDoubleSpinBox;
    minRayAngleSpin_->setRange(0.0, 60.0);
    minRayAngleSpin_->setSingleStep(1.0);
    minRayAngleSpin_->setDecimals(1);
    minRayAngleSpin_->setValue(settings_.value(QStringLiteral("minRayAngleDeg"), 0.0).toDouble());
    paramRow->addWidget(minRayAngleSpin_);
    paramRow->addWidget(new QLabel(QStringLiteral("歧义边界⚠：")));
    ambiguityMarginSpin_ = new QDoubleSpinBox;
    ambiguityMarginSpin_->setRange(0.0, 0.2);
    ambiguityMarginSpin_->setSingleStep(0.005);
    ambiguityMarginSpin_->setDecimals(3);
    ambiguityMarginSpin_->setValue(settings_.value(QStringLiteral("ambiguityMargin"), 0.0).toDouble());
    paramRow->addWidget(ambiguityMarginSpin_);
    useLmRefineChk_ = new QCheckBox(QStringLiteral("LM非线性精修"), this);
    useLmRefineChk_->setChecked(settings_.value(QStringLiteral("useLmRefine"), true).toBool());
    paramRow->addWidget(useLmRefineChk_);
    paramRow->addWidget(new QLabel(QStringLiteral("聚类卡方阈值：")));
    clusterChiSquareSpin_ = new QDoubleSpinBox;
    clusterChiSquareSpin_->setRange(1.0, 40.0);
    clusterChiSquareSpin_->setSingleStep(1.0);
    clusterChiSquareSpin_->setDecimals(1);
    clusterChiSquareSpin_->setValue(settings_.value(QStringLiteral("clusterChiSquare"), 9.21).toDouble());
    paramRow->addWidget(clusterChiSquareSpin_);
    paramRow->addStretch(1);
    v->addLayout(paramRow);

    v = beginSection(QStringLiteral("检测定位"), QStringLiteral("detect"), false);
    // 【新增】检测定位算法切换——质心法(默认，现状) vs 圆拟合法(项目里
    // 已有的遮挡感知检测层3a~3d)。切换会触发rebuild()(需要重新订阅
    // blobsReady/blobDetailsReady中的一个、开关轮廓采集)，不是运行时热
    // setter能做到的，所以直接连到rebuild而不是走onXxxParamChanged那套。
    auto* detectAlgoRow = new FlowLayout();
    useCircleFitChk_ = new QCheckBox(QStringLiteral("圆拟合法（替代质心法）"), this);
    useCircleFitChk_->setChecked(settings_.value(QStringLiteral("useCircleFit"), false).toBool());
    detectAlgoRow->addWidget(useCircleFitChk_);
    detectAlgoRow->addWidget(new QLabel(QStringLiteral("球半径先验(像素)：")));
    circleRadiusPxSpin_ = new QDoubleSpinBox;
    circleRadiusPxSpin_->setRange(1.0, 200.0);
    circleRadiusPxSpin_->setSingleStep(0.5);
    circleRadiusPxSpin_->setDecimals(2);
    circleRadiusPxSpin_->setValue(settings_.value(QStringLiteral("circleRadiusPx"), 6.0).toDouble());
    detectAlgoRow->addWidget(circleRadiusPxSpin_);

    // 【新增】轮廓点数上限——圆拟合的兜底闸，跟检测参数面板里的
    // minArea/maxArea/minCircularity 是【互补】而不是重复的关系：
    //   · 那三个管的是"这个连通域算不算一个球"(面积、圆度)，在
    //     CentroidDetector 里就把不合格的丢掉了；
    //   · 这个管的是"就算它通过了上面的筛选，它的轮廓点数会不会大到
    //     让圆拟合失控"。detectBalls 的代价随【轮廓点数】线性放大，而
    //     轮廓点数跟面积不是一回事——又细又弯带毛刺的斑，面积可能很小
    //     (maxArea 拦不住)，周长却极大。实测抓到过面积达标、轮廓 601 点
    //     的 blob，单个就要几十万次带 sqrt 的距离计算。
    // 正常一颗反光球的轮廓也就几十个点，默认 200 留了很大余量，正常场景
    // 永远不会触发。超限的 blob 不丢，退回用检测器自带的灰度加权质心。
    // 【真正的解法仍然是把检测参数调对】(提高 threshold、收紧 maxArea、
    // 提高 minCircularity 把不规则块滤掉)，这个上限只是安全网。
    detectAlgoRow->addWidget(new QLabel(QStringLiteral("轮廓点数上限：")));
    maxContourPtsSpin_ = new QSpinBox;
    maxContourPtsSpin_->setRange(0, 5000);
    maxContourPtsSpin_->setSingleStep(50);
    maxContourPtsSpin_->setSpecialValueText(QStringLiteral("不限"));
    maxContourPtsSpin_->setValue(settings_.value(QStringLiteral("maxContourPts"), 200).toInt());
    connect(maxContourPtsSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v){
        settings_.setValue(QStringLiteral("maxContourPts"), v);
        // 不需要rebuild()——onBlobDetails每帧实时读这个控件的值。
    });
    detectAlgoRow->addWidget(maxContourPtsSpin_);

    useMahalanobisChk_ = new QCheckBox(QStringLiteral("马氏距离门控"), this);
    useMahalanobisChk_->setChecked(settings_.value(QStringLiteral("useMahalanobis"), false).toBool());
    detectAlgoRow->addWidget(useMahalanobisChk_);
    detectAlgoRow->addStretch(1);
    v->addLayout(detectAlgoRow);

    v = beginSection(QStringLiteral("跨帧追踪"), QStringLiteral("track"), false);
    // ---- TemporalTracker跨帧关联参数——之前是写死的20.0/36/6，现在暴露
    // 出来可调，跟上面聚类阈值是"点云稳不稳"这件事的另外半条链路：聚类
    // 阈值决定"这一帧到底解出了几个候选点"，这几个参数决定"这些候选点
    // 跨帧怎么保持稳定编号"——两边都可能是"点云不稳"的根因，都调得到
    // 才叫真正意义上的"调到稳定为止"。----
    auto* trackerRow = new FlowLayout();
    trackerRow->addWidget(new QLabel(QStringLiteral("关联距离(mm)：")));
    assocDistSpin_ = new QDoubleSpinBox;
    assocDistSpin_->setRange(2.0, 100.0);
    assocDistSpin_->setSingleStep(1.0);
    assocDistSpin_->setDecimals(1);
    assocDistSpin_->setValue(settings_.value(QStringLiteral("assocDistMm"), 20.0).toDouble());
    trackerRow->addWidget(assocDistSpin_);
    trackerRow->addWidget(new QLabel(QStringLiteral("遮挡记忆(帧)：")));
    maxMissedSpin_ = new QSpinBox;
    maxMissedSpin_->setRange(1, 300);
    maxMissedSpin_->setValue(settings_.value(QStringLiteral("maxMissedFrames"), 36).toInt());
    maxMissedSpin_->setToolTip(QStringLiteral(
        "一个点连续多少帧没被看到才彻底销毁编号(不是时长，是帧数——帧率\n"
        "越高，同样的时长对应的帧数越多)。"));
    trackerRow->addWidget(maxMissedSpin_);
    trackerRow->addWidget(new QLabel(QStringLiteral("确认门槛(帧)：")));
    minHitsSpin_ = new QSpinBox;
    minHitsSpin_->setRange(1, 30);
    minHitsSpin_->setValue(settings_.value(QStringLiteral("minHitsToConfirm"), 6).toInt());
    minHitsSpin_->setToolTip(QStringLiteral(
        "新出现的候选点要连续被看到这么多帧才算\"确认\"、公开发编号——调大\n"
        "能有效压掉幽灵点偶然混进来的概率，代价是真点也要多等几帧才显示。"));
    trackerRow->addWidget(minHitsSpin_);
    useHungarianChk_ = new QCheckBox(QStringLiteral("匈牙利全局最优指派"), this);
    useHungarianChk_->setChecked(settings_.value(QStringLiteral("useHungarian"), false).toBool());
    trackerRow->addWidget(useHungarianChk_);
    trackerRow->addStretch(1);
    v->addLayout(trackerRow);

    // 【新增】更进阶的几个稳定性相关系数——平时不太需要碰，但既然影响的
    // 是同一件"编号稳不稳"的事，也做成可调+可选，跟上面几个参数同一套
    // 存档/热更机制。单独一行，避免第一行太挤。
    auto* advRow = new FlowLayout();
    useAdaptiveCapChk_ = new QCheckBox(QStringLiteral("自适应收紧关联距离"), this);
    useAdaptiveCapChk_->setChecked(settings_.value(QStringLiteral("useAdaptiveCap"), true).toBool());
    useAdaptiveCapChk_->setToolTip(QStringLiteral(
        "默认开启：按\"这一帧点彼此的最近邻距离\"动态收紧关联半径，避免点\n"
        "密集时固定阈值形同虚设、互相抢关联。相机少/marker本来就稀疏时\n"
        "这层收紧通常没有必要，关掉后只用\"关联距离(mm)\"这一个固定值。"));
    advRow->addWidget(useAdaptiveCapChk_);
    advRow->addWidget(new QLabel(QStringLiteral("收紧系数：")));
    adaptiveCapMulSpin_ = new QDoubleSpinBox;
    adaptiveCapMulSpin_->setRange(0.05, 1.0);
    adaptiveCapMulSpin_->setSingleStep(0.05);
    adaptiveCapMulSpin_->setDecimals(2);
    adaptiveCapMulSpin_->setValue(settings_.value(QStringLiteral("adaptiveCapMultiplier"), 0.5).toDouble());
    adaptiveCapMulSpin_->setToolTip(QStringLiteral("关联半径最多收紧到\"最近邻距离×这个系数\"，默认0.5。"));
    advRow->addWidget(adaptiveCapMulSpin_);
    useMissedRelaxChk_ = new QCheckBox(QStringLiteral("丢帧门控放宽"), this);
    useMissedRelaxChk_->setChecked(settings_.value(QStringLiteral("useMissedRelax"), true).toBool());
    useMissedRelaxChk_->setToolTip(QStringLiteral(
        "默认开启：confirmed轨迹每连续丢一帧，关联半径按比例放宽，缓解\n"
        "快速运动/变向时恒速预测误差累积导致的连续误判丢失(这是之前\n"
        "\"共视区内一运动就重新编号\"问题的修复)。关掉退回\"门控半径固定\n"
        "不随丢帧变化\"的行为。"));
    advRow->addWidget(useMissedRelaxChk_);
    advRow->addWidget(new QLabel(QStringLiteral("放宽增量/帧：")));
    relaxGrowthSpin_ = new QDoubleSpinBox;
    relaxGrowthSpin_->setRange(0.0, 2.0);
    relaxGrowthSpin_->setSingleStep(0.1);
    relaxGrowthSpin_->setDecimals(2);
    relaxGrowthSpin_->setValue(settings_.value(QStringLiteral("relaxGrowthPerMissedFrame"), 0.4).toDouble());
    relaxGrowthSpin_->setToolTip(QStringLiteral("每连续丢一帧，门控半径在原基础上再放宽这么多倍(默认0.4＝+40%)。"));
    advRow->addWidget(relaxGrowthSpin_);
    advRow->addWidget(new QLabel(QStringLiteral("放宽上限倍数：")));
    relaxCapSpin_ = new QDoubleSpinBox;
    relaxCapSpin_->setRange(1.0, 10.0);
    relaxCapSpin_->setSingleStep(0.5);
    relaxCapSpin_->setDecimals(1);
    relaxCapSpin_->setValue(settings_.value(QStringLiteral("relaxCapMultiplier"), 3.0).toDouble());
    relaxCapSpin_->setToolTip(QStringLiteral("门控半径最多放宽到原始值的这么多倍(默认3.0)，防止无限放宽误关联到很远的点。"));
    advRow->addWidget(relaxCapSpin_);
    advRow->addStretch(1);
    v->addLayout(advRow);

    // 【新增】速度平滑(EMA) + 恒加速度模型——都默认关闭，见
    // TemporalTracker.hpp对应setter说明。同一行放，逻辑上都属于"运动
    // 模型怎么预测下一帧"这件事。
    auto* motionRow = new FlowLayout();
    useVelSmoothChk_ = new QCheckBox(QStringLiteral("速度平滑(EMA)"), this);
    useVelSmoothChk_->setChecked(settings_.value(QStringLiteral("useVelSmooth"), false).toBool());
    motionRow->addWidget(useVelSmoothChk_);
    motionRow->addWidget(new QLabel(QStringLiteral("alpha：")));
    velSmoothAlphaSpin_ = new QDoubleSpinBox;
    velSmoothAlphaSpin_->setRange(0.01, 1.0);
    velSmoothAlphaSpin_->setSingleStep(0.05);
    velSmoothAlphaSpin_->setDecimals(2);
    velSmoothAlphaSpin_->setValue(settings_.value(QStringLiteral("velSmoothAlpha"), 0.5).toDouble());
    velSmoothAlphaSpin_->setToolTip(QStringLiteral("越小越平滑(新观测贡献越少)，1.0等价于关闭平滑。"));
    motionRow->addWidget(velSmoothAlphaSpin_);
    useConstAccelChk_ = new QCheckBox(QStringLiteral("恒加速度模型"), this);
    useConstAccelChk_->setChecked(settings_.value(QStringLiteral("useConstAccel"), false).toBool());
    motionRow->addWidget(useConstAccelChk_);
    motionRow->addStretch(1);
    v->addLayout(motionRow);

    secBackend = beginSection(QStringLiteral("追踪后端 · IEKF"), QStringLiteral("backend"), false);
    v = secBackend;
    // 【新增】追踪后端整体切换——见头文件useIekfBackendChk_的说明。
    backendRow = newRow();
    useIekfBackendChk_ = new QCheckBox(QStringLiteral("IEKF卡尔曼追踪后端(替代上面这套启发式追踪)"), this);
    useIekfBackendChk_->setChecked(settings_.value(QStringLiteral("useIekfBackend"), false).toBool());
    backendRow->addWidget(useIekfBackendChk_);

    // 【新增】骨骼叠加开关 + 关联器构建。跟 HandTemplateStore 共用标定向导
    // 存的那份模板(不管有没有标定过——resetToPlaceholders()保证data()永远
    // 有值，占位值也能跑，只是骨架不准，这是刻意的：不该因为没标定就完全
    // 拿不到功能反馈)。onnxruntime 没装/没训练出模型时 skeletonBackend_->
    // ready()==false，勾选了也不会显示骨架(不崩，只是没效果)，见
    // Hm20OnnxBackend.hpp 桩实现的说明。
    showSkeletonChk_ = new QCheckBox(QStringLiteral("叠加显示骨架(AI关联，需要已训练模型)"), this);
    showSkeletonChk_->setChecked(settings_.value(QStringLiteral("showSkeleton"), false).toBool());
    secSkeleton = beginSection(QStringLiteral("骨架关联（AI）"), QStringLiteral("skel"), false);
    v = secSkeleton;
    backendRow = newRow();
    backendRow->addWidget(showSkeletonChk_);

    // IK 开/关 —— 真机上一眼能对比出有没有用，比看数字直观。
    useIkRefineChk_ = new QCheckBox(QStringLiteral("IK精修手指姿态"));
    useIkRefineChk_->setChecked(true);
    connect(useIkRefineChk_, &QCheckBox::toggled, this, [this](bool on) {
        if (skeletonWorker_) skeletonWorker_->setIkEnabled(on);
    });
    backendRow->addWidget(useIkRefineChk_);

    // 高速模式：换 INT8 量化模型。默认关 —— 它会改变数值结果，
    // 必须让用户主动选，而且能一键切回来对照。
    fastModeChk_ = new QCheckBox(QStringLiteral("高速模式(INT8)"));
    connect(fastModeChk_, &QCheckBox::toggled, this, [this](bool on) {
        settings_.setValue(QStringLiteral("hm20FastMode"), on);
        if (skeletonWorker_) skeletonWorker_->setFastMode(on);
    });
    backendRow->addWidget(fastModeChk_);

    // ---- 手性 ----
    // 【手背复位】把原来"必须重开点云测试面板"变成一个按钮。
    // 求解器有三个进去出不来的状态（连续性锁、被重捕改过的模板、
    // 求解失败->不能重捕的死锁），这个按钮一次全清，但不动自标定和 IK。
    dorsumResetBtn_ = new QPushButton(QStringLiteral("手背复位"));
    dorsumResetBtn_->setToolTip(QStringLiteral(
        "手背连线乱了、或者某个点挪开之后一直修不回来时点它。\n"
        "等价于重开本面板，但不会丢掉自标定和 IK 的状态。\n"
        "会清掉：连续性锁、重捕缓冲、被重捕改过的模板（退回标定值）。"));
    connect(dorsumResetBtn_, &QPushButton::clicked, this, [this] {
        if (skeletonWorker_) skeletonWorker_->requestDorsumReset();
    });
    quickRow->addWidget(dorsumResetBtn_);      // 见上面「顶部常用操作」的说明

    handRightChk_ = new QCheckBox(QStringLiteral("右手"));
    handRightChk_->setChecked(settings_.value(QStringLiteral("handIsRight"), true).toBool());
    connect(handRightChk_, &QCheckBox::toggled, this, [this](bool on) {
        settings_.setValue(QStringLiteral("handIsRight"), on);
        rebuildSkeletonTemplate();
    });
    // 【启动时必须无条件下发一次，不能只靠 toggled】
    // setChecked(false) 在一个默认就是 false 的复选框上【不发 toggled 信号】，
    // 于是从 QSettings 恢复成"左手"时这条路径完全不执行，worker 的
    // handIsRight_ 一直停在默认值 true。
    //
    // 后果是一整条链：面板显示左手 -> 实际按右手解 -> 自标定镜像反 ->
    // bundleRmseMm 降不到 4.0 以下 -> ikUsable() 恒 false -> IK 全程不生效 ->
    // thumbPronation0 标得再准也没人用 -> 拇指外翻。
    // 实测录制里 handPanelIsRight=false 而 autoIsRight=true，就是这么来的。
    QMetaObject::invokeMethod(this, [this] { rebuildSkeletonTemplate(); },
                              Qt::QueuedConnection);
    backendRow->addWidget(handRightChk_);

    // 【手性不自动判，靠面板设定】
    //
    // 几何上无法从 marker 位置可靠判定手性 —— 这是数学结论：镜像 M（det=-1）下，
    // 从手上点算的法向 n' = -M·n、叉积 (X×V)' = -M(X×V)，乘积恒不变号。
    // 一只左手和它的镜像在几何上完全等价。（试过四种构造，全部不变号。）
    //
    // 所以不弹窗打断，只做两件事：勾选框标题写清楚，状态栏常驻显示生效值。
    // AI 判断只作参考显示，不改任何东西。
    handRightChk_->setText(QStringLiteral("右手（左手请取消勾选）"));

    backendRow = newRow();
    // ---- ROM 标定（遥操作用）----
    romBtn_ = new QPushButton(QStringLiteral("ROM标定"));
    connect(romBtn_, &QPushButton::clicked, this, [this] {
        if (!skeletonWorker_) return;
        if (romBtn_->text() == QStringLiteral("ROM标定")) {
            // 【补标】按住 Ctrl 再点 = 保留上一轮样本继续攒。
            // 握拳时四指互相遮挡是物理事实，一次做不满很正常；补标可以只针对
            // 没标满的那几维再做一次动作，已经标好的维不会被这一轮冲掉。
            const bool accum = (QApplication::keyboardModifiers() & Qt::ControlModifier)
                               && skeletonWorker_->romReady();
            // ---- 事件 16：按钮按下的那一刻（GUI 线程墙钟）----
            // 【为什么单独记，不指望 RF_RomLearning 位】那个位是逐帧采样的，
            // 而按钮按下和 worker 第一帧读到它之间隔着一个投递延迟。
            // 事后对着几千帧找"标定是从哪一帧开始的"，只靠位做差分会差几帧。
            // worker 侧还会在真正开始采样的那一帧打事件 18，两者的差
            // 本身就是投递延迟，能反映 worker 忙不忙。
            recordRomEvent(16, accum ? QStringLiteral("按下 ROM标定（补标：保留上一轮样本）")
                                     : QStringLiteral("按下 ROM标定（重新开始）"),
                           accum ? 1 : 0);
            skeletonWorker_->romStart(accum);
            romBtn_->setText(accum ? QStringLiteral("■ 结束补标")
                                   : QStringLiteral("■ 结束ROM"));
            if (romHintLabel_)
                romHintLabel_->setText(QStringLiteral(
                    "标定中：五指张开到底 → 握拳到底，来回两三遍，约5秒"));
        } else {
            recordRomEvent(17, QStringLiteral("按下 结束ROM"), 0);
            const bool ok = skeletonWorker_->romFinish();
            romBtn_->setText(QStringLiteral("ROM标定"));

            // ---- 结果弹窗：逐维明细 ----
            // 【为什么不能只说"行程太小，标定未生效"】用户看到那句话之后
            // 【不知道该改哪一根手指的动作】，只能整体重做一遍，而重做一遍
            // 大概率还是同样的结果 —— 因为真正卡住的往往不是动作幅度，
            // 而是"手背被挡导致腕部位姿失效"这类跟动作幅度无关的原因。
            const QString rep = QString::fromStdString(skeletonWorker_->romReport());
            QMessageBox box(this);
            box.setWindowTitle(QStringLiteral("ROM标定"));
            box.setIcon(ok ? QMessageBox::Information : QMessageBox::Warning);
            box.setText(ok ? QStringLiteral("标定成功，屈曲行程覆盖 %1%。")
                                .arg(int(skeletonWorker_->romCoverage() * 100))
                           : QStringLiteral("标定未生效：达标的屈曲维不够。\n"
                                            "看下面的逐维明细，按住 Ctrl 再点「ROM标定」"
                                            "可以只补标没标满的那几维。"));
            box.setDetailedText(rep);
            box.setStandardButtons(QMessageBox::Ok);
            box.exec();
            if (romHintLabel_) romHintLabel_->setText(QString());
            // ---- 把整份报告也写进录制文件 ----
            // 【为什么弹窗之外还要落盘】弹窗关掉就没了，而这份报告是
            // "标定当时到底发生了什么"的唯一人类可读版本。走 ParamDelta 那条路，
            // 跟运行时配置同路，离线脚本已经会读了，不用新加解析代码。
            if (recorder_.recording()) {
                recorder_.writeParamDelta(QDateTime::currentMSecsSinceEpoch() * 1000000LL,
                    std::string("{\"romFinish\":") + skeletonWorker_->romJson() + "}");
            }
        }
    });
    backendRow->addWidget(romBtn_);
    // 标定期间的引导文字。【为什么要有】v1 只有按钮文字变了，用户按下之后
    // 没有任何提示告诉他现在该做什么动作、做多久，全靠记住 tooltip。
    romHintLabel_ = new QLabel(QString());
    romHintLabel_->setStyleSheet(QStringLiteral("color:#e0a030;"));
    backendRow->addWidget(romHintLabel_);

    // ---- 在线自标定 ----
    autoCalibChk_ = new QCheckBox(QStringLiteral("在线自标定(免手工标定)"));
    autoCalibChk_->setChecked(settings_.value(QStringLiteral("skelAutoCalib"), false).toBool());
    connect(autoCalibChk_, &QCheckBox::toggled, this, [this](bool on) {
        settings_.setValue(QStringLiteral("skelAutoCalib"), on);
        if (skeletonWorker_) skeletonWorker_->setAutoCalibEnabled(on);
    });
    backendRow->addWidget(autoCalibChk_);

    // 文案补全：这个按钮原来紧挨着「在线自标定」复选框，光写"重标"能看懂；
    // 现在钉到侧栏顶部、脱离了那个上下文，得说清楚重标的是什么。
    autoCalibResetBtn_ = new QPushButton(QStringLiteral("自标定重标"));
    autoCalibResetBtn_->setToolTip(QStringLiteral("丢弃当前自标定结果，从头开始。换人/重贴反光球之后按。"));
    connect(autoCalibResetBtn_, &QPushButton::clicked, this, [this] {
        if (skeletonWorker_) skeletonWorker_->resetAutoCalib();
    });
    quickRow->addWidget(autoCalibResetBtn_);   // 同上，钉到侧栏顶部
    quickRow->addStretch(1);

    // ---- 时序滤波 ----
    backendRow->addWidget(new QLabel(QStringLiteral("平滑:")));
    smoothCombo_ = new QComboBox();
    smoothCombo_->addItems({QStringLiteral("关"), QStringLiteral("轻"),
                            QStringLiteral("均衡"), QStringLiteral("强")});
    smoothCombo_->setCurrentIndex(settings_.value(QStringLiteral("skelSmooth"), 2).toInt());
    connect(smoothCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int i) {
        settings_.setValue(QStringLiteral("skelSmooth"), i);
        if (!skeletonWorker_) return;
        auto c = hm20::makePoseFilterPreset(hm20::PoseFilterPreset(i));
        skeletonWorker_->setFilterConfig(c);
        // 直通模式下这一路被旁路（控件也是灰的，正常走不到这里；加这个判断是防
        // 止将来有人从代码里 setCurrentIndex 把直通模式悄悄破掉）。
        const bool lowLat = lowLatencyChk_ && lowLatencyChk_->isChecked();
        skeletonWorker_->setFilterEnabled(!lowLat && i != 0);
    });
    backendRow->addWidget(smoothCombo_);

    v = beginSection(QStringLiteral("录制"), QStringLiteral("rec"), false);
    backendRow = newRow();
    // ---- 原始数据流录制 ----
    recProtoCombo_ = new QComboBox();
    recProtoCombo_->addItems({QStringLiteral("静止保持"), QStringLiteral("刚体运动"),
                              QStringLiteral("手指屈伸"), QStringLiteral("遮挡压力"),
                              QStringLiteral("自由动作"), QStringLiteral("刚体标定件"),
                              QStringLiteral("张开握拳循环"), QStringLiteral("手性排查")});
    recProtoCombo_->setCurrentIndex(1);
    backendRow->addWidget(recProtoCombo_);

    // ---- 录制详细度 ----
    // 【默认全录】录制这个动作本身就是为了排查，而排查时最贵的从来不是磁盘，
    // 是"录完发现关键那一项没开、得重录一遍"—— 偶发问题重录不一定复现得出来。
    recDetailCombo_ = new QComboBox();
    recDetailCombo_->addItems({QStringLiteral("精简"), QStringLiteral("完整"),
                               QStringLiteral("全量")});
    recDetailCombo_->setCurrentIndex(1);
    backendRow->addWidget(recDetailCombo_);

    recNoteEdit_ = new QLineEdit();
    recNoteEdit_->setPlaceholderText(QStringLiteral("备注(可空)"));
    recNoteEdit_->setFixedWidth(140);
    backendRow->addWidget(recNoteEdit_);

    recBtn_ = new QPushButton(QStringLiteral("● 录制"));
    connect(recBtn_, &QPushButton::clicked, this, &PointCloudTestDialog::toggleRecording);
    backendRow->addWidget(recBtn_);

    recMarkBtn_ = new QPushButton(QStringLiteral("打标记"));
    recMarkBtn_->setEnabled(false);
    recMarkBtn_->setToolTip(QStringLiteral(
        "在当前时刻打一个标记。用来切分\"这一段是刚体运动/这一段我手抖了作废\"。\n"
        "分析时会按标记切段，比事后靠时间猜可靠得多。"));
    connect(recMarkBtn_, &QPushButton::clicked, this, [this] {
        if (!recorder_.recording()) return;
        recorder_.writeMark(QDateTime::currentMSecsSinceEpoch() * 1000000LL, 0,
                            recNoteEdit_->text().toStdString());
    });
    backendRow->addWidget(recMarkBtn_);

    // 【打开目录】录完最常见的下一步就是把文件发给别人，而"文件在哪"是这一步
    // 最常见的卡点。以前存在系统「文档」目录，中文用户名 + 深路径，找起来很烦。
    recOpenDirBtn_ = new QPushButton(QStringLiteral("打开录制目录"));
    connect(recOpenDirBtn_, &QPushButton::clicked, this, [this] {
        const QString d = recLastPath_.isEmpty() ? recordingDir()
                                                 : QFileInfo(recLastPath_).absolutePath();
        QDesktopServices::openUrl(QUrl::fromLocalFile(d));
    });
    backendRow->addWidget(recOpenDirBtn_);

    recStatusLabel_ = new QLabel(QStringLiteral("未录制"));
    recStatusLabel_->setFixedWidth(180);
    backendRow->addWidget(recStatusLabel_);

    v = beginSection(QStringLiteral("输出与延迟"), QStringLiteral("out"), false);
    backendRow = newRow();
    // ---- 输出数据实时打印 ----
    // 【放在这一行，紧挨着骨架/录制】它们是同一类东西：都是"这条链路到底在
    // 干什么"的可观测性，不是算法参数。
    showOutputChk_ = new QCheckBox(QStringLiteral("打印输出数据"));
    showOutputChk_->setChecked(settings_.value(QStringLiteral("showOutputMonitor"), false).toBool());
    backendRow->addWidget(showOutputChk_);

    backendRow = newRow();
    // ---- 输出链路延迟 ----
    // 【这一组是为了回答"连线为什么不跟手"】不是加功能，是把原来写死的两个
    // 数字(33ms 限流、LowestPriority)和两条滤波链暴露出来，并且给出实测毫秒数。
    lowLatencyChk_ = new QCheckBox(QStringLiteral("低延迟直通"));
    // 【故意不从 QSettings 恢复】直通是诊断工具不是常用档。上一版持久化的后果是：
    // 上次为了做 A/B 勾了一下，之后每次启动平滑都是被旁路的，而界面上没有任何
    // 提示，表现成"平滑功能坏了"。每次启动都从关闭开始。
    lowLatencyChk_->setChecked(false);
    backendRow->addWidget(lowLatencyChk_);

    backendRow->addWidget(new QLabel(QStringLiteral("骨架刷新:")));
    skelRateCombo_ = new QComboBox();
    skelRateCombo_->addItems({QStringLiteral("30Hz"), QStringLiteral("60Hz"),
                              QStringLiteral("120Hz"), QStringLiteral("不限流")});
    skelRateCombo_->setCurrentIndex(settings_.value(QStringLiteral("skelDispatchRate"), 0).toInt());
    backendRow->addWidget(skelRateCombo_);

    rateLimitChk_ = new QCheckBox(QStringLiteral("速度限幅"));
    rateLimitChk_->setChecked(settings_.value(QStringLiteral("skelRateLimit"), true).toBool());
    backendRow->addWidget(rateLimitChk_);

    latencyLabel_ = new ElidingLabel(QStringLiteral("滞后：—"), this);
    // 固定宽度、不换行，理由同 skeletonDiagLabel_（每 250ms 变一次的文字，
    // 开自动换行会让整个参数面板反复重排）。
    latencyLabel_->setWordWrap(false);
    latencyLabel_->setTextFormat(Qt::PlainText);
    // 原来定死宽度是为了防止数字变化导致布局抖动；现在标签自己会省略号截断，
    // 给个上限就够了，剩下的宽度让给别的读数。
    latencyLabel_->setMaximumWidth(300);
    statusStrip->addWidget(latencyLabel_);   // 读数归状态条，不占参数栏

    v = secSkeleton;
    backendRow = newRow();
    // ---- 拇指 roll 偏置 ----
    // 【为什么必须能调，而不是写个常数】绕骨轴的自转在每节只有 1 颗球时
    // 【几何上不可观测】—— 球绕骨轴转一圈位置完全不变。所以这个量测不出来，
    // 只能建模；既然是建模参数，就没有任何观测能验证它，只能对着画面调。
    backendRow->addWidget(new QLabel(QStringLiteral("拇指roll:")));
    thumbRollSpin_ = new QDoubleSpinBox();
    thumbRollSpin_->setRange(-180.0, 180.0);
    thumbRollSpin_->setDecimals(0);
    thumbRollSpin_->setSingleStep(5.0);
    thumbRollSpin_->setSuffix(QStringLiteral("°"));
    thumbRollSpin_->setValue(settings_.value(QStringLiteral("thumbRollDeg"), 80.0).toDouble());
    backendRow->addWidget(thumbRollSpin_);

    thumbPronChk_ = new QCheckBox(QStringLiteral("回正预测点"));
    thumbPronChk_->setChecked(settings_.value(QStringLiteral("thumbPronOn"), true).toBool());
    backendRow->addWidget(thumbPronChk_);

    ikOnlyChk_ = new QCheckBox(QStringLiteral("遮挡点只用IK"));
    // 【默认关】开着会让 IK 补不上的遮挡点冻住，而网络 pos 头对四指的外推
    // 实测是好用的 —— 冻住反而更差。留作诊断用（想看"纯 IK 能补多少"时勾上）。
    ikOnlyChk_->setChecked(settings_.value(QStringLiteral("occludedIkOnly"), false).toBool());
    backendRow->addWidget(ikOnlyChk_);

    geoRelabelChk_ = new QCheckBox(QStringLiteral("手背几何定标签"));
    geoRelabelChk_->setChecked(settings_.value(QStringLiteral("dorsumGeoRelabel"), true).toBool());
    backendRow->addWidget(geoRelabelChk_);

    handTemplateStore_ = std::make_unique<HandTemplateStore>();
    {
        // 【改动】HandTemplateStore 存的是"手背5点 + 每指 anchor/lengths"，
        // hm20 网络要的是"中立位 20 点坐标"，中间需要一次合成，近似程度见
        // Hm20TemplateAdapter.hpp 的头注释。
        //
        // 【isRight 目前写死，记得接】HandTemplateData 里没有左右手字段。
        // 写反的后果不是小事：条件向量整体镜像，手背编号会错乱——正是
        // 之前修过的"手背编号左右手不统一"那个 bug 的复发路径。等标定
        // 向导补上左右手选项后从那里读。
        // 【曾经是 const bool isRightHand = true;】注释还写着"由标定向导里选的
        // 那个"，但标定向导【根本没有左右手选项】——写死右手，左手用户无从
        // 更改。实测左手受试者的数据：手性判据 dot(X×Y, 手背外法向) = -0.862，
        // 明确是左手，而系统一直按右手在跑。
        // 危害有两处：packNormalized() 的 hand_sign 那一位标反（送进网络的
        // 条件向量手性错误），以及 Hm20IkRefiner 的镜像方向反（左手 IK 的
        // 外展方向整个反掉，残差常年超门限，表现为"左手 IK 永远不生效"）。
        const bool isRightHand = handRightChk_ ? handRightChk_->isChecked()
                                               : settings_.value(QStringLiteral("handIsRight"), true).toBool();
        hm20::Hm20Template skelTmpl =
            hm20::makeHm20Template(handTemplateStore_->data(), isRightHand);
        skeletonTemplateForRender_ = skelTmpl;
        // 【模型选择：v7 优先，自动回退 v6】
        // 【为什么不写死一个】树里同时存在两代模型，而适配层对两者都兼容
        // (v7 多的姿态/手性头缺失时 has* 保持 false，走原来的几何路径)。
        // 硬编码一个名字的后果是换模型要改代码重编，而且改错了只会在运行时
        // 报"模型不可用"——这个文件的注释里已经记着一次同类教训。
        // 顺序：QSettings 指定 > hm20_v7.onnx > hm20_v6_sk10.onnx > hm20_v6.onnx
        const QString appDir = QCoreApplication::applicationDirPath();
        QString onnxPath = settings_.value(QStringLiteral("hm20ModelPath")).toString();
        if (onnxPath.isEmpty() || !QFileInfo::exists(onnxPath)) {
            const QStringList cands = {
                QStringLiteral("/hm20_v7.onnx"),
                QStringLiteral("/hm20_v6_sk10.onnx"),
                QStringLiteral("/hm20_v6.onnx"),
            };
            onnxPath.clear();
            for (const QString& c : cands) {
                if (QFileInfo::exists(appDir + c)) { onnxPath = appDir + c; break; }
            }
            // 一个都没找到时仍然给出默认名，让后端去报"文件不存在"——
            // 那条错误信息比这里静默失败清楚得多。
            if (onnxPath.isEmpty()) onnxPath = appDir + QStringLiteral("/hm20_v7.onnx");
        }
        qInfo("[hm20] 使用模型: %s", qUtf8Printable(onnxPath));

        // 【高速模式的候选模型】INT8 量化版，跟原模型放同一个目录。
        // 找不到就没有高速模式可切，UI 上那个勾会被禁用 —— 不静默失败。
        //
        // 命名约定：原模型 xxx.onnx  ->  高速版 xxx_int8.onnx
        // 转换脚本见 tools/quantize_hm20.py。
        QString fastPath = settings_.value(QStringLiteral("hm20FastModelPath")).toString();
        if (fastPath.isEmpty() || !QFileInfo::exists(fastPath)) {
            QString guess = onnxPath;
            guess.replace(QStringLiteral(".onnx"), QStringLiteral("_int8.onnx"));
            fastPath = QFileInfo::exists(guess) ? guess : QString();
        }
        if (!fastPath.isEmpty())
            qInfo("[hm20] 高速模式可用: %s", qUtf8Printable(fastPath));

        // 【重要——不要把backend/session的创建搬回这里】
        // 这里是【GUI线程】(对话框构造函数)。onnxruntime 的初始化包含：
        // 加载约5MB的DLL、注册上千个ONNX算子schema、构图并做图优化。
        // Release下约75ms，MinGW Debug(-O0)下会放大到接近1秒；更糟的是
        // schema注册若出现重复会往stderr刷几千行警告，而往Qt Creator的
        // 输出窗格写stderr是【同步且很慢】的操作——这一整套放在GUI线程，
        // 直接表现为"界面卡住不动"。
        // 所以这里只传【路径和模板】给worker，真正的加载/建会话在worker
        // 线程里首次用到时才做(见 SkeletonAssocWorker::ensureInitialized)，
        // GUI线程从头到尾不碰 onnxruntime。
        // 【改动】限定名从 mocap:: 变成 mocap::hm20::。漏改的症状跟原注释里
        // 记的那次一模一样：排队调用解析不到类型 -> 槽函数永远不执行 ->
        // 骨架毫无反应，而且不报任何错。
        qRegisterMetaType<mocap::hm20::SkeletonFrameResult>("mocap::hm20::SkeletonFrameResult");
        qRegisterMetaType<mocap::SkeletonAssocDiag>("mocap::SkeletonAssocDiag");
        qRegisterMetaType<mocap::SkeletonCandVec>("mocap::SkeletonCandVec");
        // Q_ARG 会把类型名按字面量字符串化(在namespace mocap内写
        // Q_ARG(SkeletonCandVec,...)得到的是"SkeletonCandVec")，跟上面按
        // "mocap::SkeletonCandVec"注册的名字对不上会导致排队调用失败、
        // 槽函数永远不执行(表现为骨架毫无反应)。两个名字都注册上，避免
        // 依赖具体Qt版本的解析细节。
        qRegisterMetaType<mocap::SkeletonCandVec>("SkeletonCandVec");
        qRegisterMetaType<mocap::hm20::SkeletonFrameResult>("SkeletonFrameResult");
        qRegisterMetaType<mocap::SkeletonAssocDiag>("SkeletonAssocDiag");

        skeletonWorker_ = new SkeletonAssocWorker();
        skeletonThread_ = new QThread(this);
        skeletonWorker_->moveToThread(skeletonThread_);
        // 阈值来自离线扫描：手指 0.55、手背 0.30。
        // 【为什么手背要低这么多】手背 5 点近似五重对称，概率天然摊在几个相邻
        // 标签上，top1 中位只有 ~0.50（手指是 ~0.97）。共用 0.55 时七成以上的
        // 手背点会被扔进 dustbin——表现就是"手指连得很好、手背基本不认"，
        // 并且手背可见点 <3 会让 Kabsch 整段跳过、腕部位姿彻底拿不到。
        // 详见 Hm20Config::minAssignProbFor 的说明。
        hm20::Hm20Config skelCfg;
        // 【0.55 -> 0.35，依据是你自己机位上的实测，不是仿真】
        // 原来的 0.55 来自仿真离线扫描（6相机、遮挡轻）。在真机 3 相机、
        // 手指屈曲遮挡重的条件下最优点明显更低。七段真实录制的复现结果：
        //
        //   段        认领/20(0.55->0.35)  手背Kabsch      腕部有效率
        //   静止      19.76 -> 19.76       0.52 -> 0.52    100% -> 100%   逐位相同
        //   刚体①     19.95 -> 19.95       0.53 -> 0.53    100% -> 100%   逐位相同
        //   手指屈伸  11.72 -> 13.24      11.88 -> 8.51     46% ->  60%   ★
        //   遮挡①     13.24 -> 13.70      10.62 -> 10.39    64% ->  61%
        //   遮挡②     15.54 -> 16.00       0.92 -> 0.94     61% ->  60%
        //
        // 简单场景【逐位不变】（点都是高置信度的，阈值根本碰不到），
        // 难场景全面改善；再降到 0.30 开始回弹。所以不是在难段上过拟合。
        skelCfg.minAssignProbFinger = 0.35;
        // 手背【不要跟着降】。实测 0.35/0.15 全面劣于 0.35/0.30 ——
        // 放进来的低置信度手背点是错的，会污染 Kabsch，残差反而变大。
        skelCfg.minAssignProbDorsum = 0.30;
        skelCfg.useTemplate   = true;
        skelCfg.usePrevFrame  = true;   // 120fps 下信息量最大的输入

        // ---- IK 精修（手指分段朝向）----
        // anchor/lengths 【必须用标定值】：HandTemplateStore 里存的是逐用户的，
        // 而 hand/HandModel.hpp 的 fingerParam() 是写死的"平均手"常量。
        // 用常量的代价：IK 的 marker 残差从 ~1.8mm 涨到 ~4.6mm。
        {
            // setBackTemplate/setFingerParams 是派生类才有的，所以先用具体类型
            // 配好，再存成基类指针（成员声明见 .hpp 里的说明）。
            auto ik = std::make_shared<hm20::Hm20IkRefiner>();
            const auto& hts = handTemplateStore_->data();
            ik->setBackTemplate(hts.backMarkers);
            std::array<hm20::Vec3, 5> ik0Anchors{};
            std::array<std::array<double, 3>, 5> ik0Lengths{};
            for (int f = 0; f < 5; ++f) {
                const auto& fp = hts.fingerParams[std::size_t(f)];
                ik->setFingerParams(f, fp.anchor,
                                    {fp.lengths[0], fp.lengths[1], fp.lengths[2]});
                ik0Anchors[std::size_t(f)] = fp.anchor;
                ik0Lengths[std::size_t(f)] = {fp.lengths[0], fp.lengths[1], fp.lengths[2]};
            }
            skeletonIk_ = ik;
            skeletonWorker_->setIkRefiner(skeletonIk_);
            // 【把标定值存一份给 worker】"手背复位"要退回的就是这一份。
            // 自标定会 setBackTemplate/setFingerParams 就地覆写 IK，
            // 不留副本的话复位无处可退 —— 那正是"复位不等价于重开面板"的
            // 一半原因（另一半是模板本身，见 worker 里 configure() 的说明）。
            skeletonWorker_->snapshotIkParams(hts.backMarkers, ik0Anchors, ik0Lengths);
        }

        skeletonWorker_->configure(skelTmpl, onnxPath.toStdString(), skelCfg);
        // 高速模式的两个参数。线程数取物理核数（QThread::idealThreadCount 返回
        // 的是逻辑核，超线程那一半对这种算子密集的图基本没收益，还会抢缓存，
        // 所以砍一半、下限 1）。
        // 【注意】线程数这一项不改变任何数值结果，只影响速度；
        // 换 INT8 模型才会改变数值，所以两者由同一个勾控制、但作用不同 ——
        // 见 setFastModel/setFastMode 的说明。
        {
            const int phys = std::max(1, QThread::idealThreadCount() / 2);
            skeletonWorker_->setFastModel(fastPath.toStdString(), phys);
            skeletonWorker_->setFastMode(
                settings_.value(QStringLiteral("hm20FastMode"), false).toBool()
                && !fastPath.isEmpty());
            qInfo("[hm20] 推理线程数: %d", phys);
            // 【禁用逻辑必须放在这里】勾选框是在 UI 构造时创建的，那时还不知道
            // INT8 模型在不在。放在那边会让勾永远可点，用户勾了却没生效 ——
            // 静默失败是最糟的形式。
            if (fastModeChk_) {
                const bool ok = !fastPath.isEmpty();
                fastModeChk_->setEnabled(ok);
                fastModeChk_->setChecked(
                    ok && settings_.value(QStringLiteral("hm20FastMode"), false).toBool());
                if (!ok)
                    fastModeChk_->setText(QStringLiteral("高速模式(未找到INT8模型)"));
            }
        }
        recModelPath_ = onnxPath;
        // 初始状态一次性下发。【别漏】控件是在 worker 之前建的，
        // toggled/currentIndexChanged 那时还没连上 worker，不补这一发的话
        // 勾选状态和 worker 里的实际开关会不一致（表现为"明明勾了却没生效"）。
        skeletonWorker_->setAutoCalibEnabled(autoCalibChk_->isChecked());
        // 关节角直通到 MainWindow 的 UdpSender（M3DS 包，Unity 端不用改）
        connect(skeletonWorker_, &SkeletonAssocWorker::handPoseForUdp,
                this, &PointCloudTestDialog::handPoseForUdp, Qt::QueuedConnection);
        connect(skeletonWorker_, &SkeletonAssocWorker::bendRefForUi,
                this, &PointCloudTestDialog::bendRefForUi, Qt::QueuedConnection);
        connect(skeletonWorker_, &SkeletonAssocWorker::segmentQuatsForUdp,
                this, &PointCloudTestDialog::segmentQuatsForUdp, Qt::QueuedConnection);
        {
            const int si = smoothCombo_->currentIndex();
            skeletonWorker_->setFilterConfig(hm20::makePoseFilterPreset(hm20::PoseFilterPreset(si)));
            skeletonWorker_->setFilterEnabled(si != 0);
        }
        skeletonWorker_->setRateLimitEnabled(rateLimitChk_->isChecked());
        connect(skeletonWorker_, &SkeletonAssocWorker::resultReady,
               this, &PointCloudTestDialog::onSkeletonResultReady);
        // 【低优先级启动】骨架叠加是辅助显示功能，重要性远低于相机采集、
        // 检测、追踪和GUI响应。给它最低优先级，操作系统在CPU紧张时会优先
        // 保证那些线程，AI慢一点只是骨架刷新率降低(30Hz本来就有余量)，
        // 不会反过来把主流程拖垮。
        // 【优先级不再写死 LowestPriority】原注释的理由(AI 慢一点只是骨架刷新率
        // 降低)只在"CPU 有余量"时成立。4 路 120fps 满载时，最低优先级的线程可能
        // 迟迟排不上，排队时间远超推理本身 —— 表现就是"骨架一顿一顿地追手"，
        // 而看 latencyMs 又只有几毫秒，完全查不出问题。所以做成跟"低延迟直通"
        // 联动，并且把排队时间单独测出来打在面板上。
        skeletonThread_->start(lowLatencyChk_->isChecked() ? QThread::NormalPriority
                                                           : QThread::LowestPriority);
        pipeClock_.start();
    }

    skeletonDiagLabel_ = new QLabel(QStringLiteral("骨骼AI：未启用"), this);
    // 【重要——不要改回 setWordWrap(true)】这个标签的文字每帧都在变(耗时
    // 数字一直跳)，如果开自动换行，会有两个严重后果：
    //  ①每次setText都触发 heightForWidth 重算 -> 整个参数面板重新布局，
    //    参数面板控件数以百计，AI每秒出上百次结果时直接把GUI线程吃满；
    //  ②更致命：自动换行的QLabel放在布局里是Qt的经典陷阱——标签高度影响
    //    可用宽度、宽度又反过来影响高度，两个尺寸来回震荡，布局系统可能
    //    永远收敛不了，GUI线程原地打转 = 真正的"莫名卡死"。
    // 所以这里固定成单行、固定宽度，完全不参与布局尺寸协商；完整信息
    // (可能很长的错误原因)放 tooltip，鼠标悬停查看，tooltip 不触发布局。
    skeletonDiagLabel_->setWordWrap(false);
    skeletonDiagLabel_->setTextFormat(Qt::PlainText);
    skeletonDiagLabel_->setFixedWidth(300);   // 侧栏定宽 ~400，360 会顶出横向滚动条
    skeletonDiagLabel_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    v = secSkeleton;
    backendRow = newRow();
    backendRow->addWidget(skeletonDiagLabel_);
    v = secBackend;
    backendRow = newRow();
    backendRow->addWidget(new QLabel(QStringLiteral("位置过程噪声：")));
    iekfPosProcessVarSpin_ = new QDoubleSpinBox;
    iekfPosProcessVarSpin_->setRange(0.01, 1000.0);
    iekfPosProcessVarSpin_->setSingleStep(1.0);
    iekfPosProcessVarSpin_->setDecimals(2);
    iekfPosProcessVarSpin_->setValue(settings_.value(QStringLiteral("iekfPosProcessVar"), 4.0).toDouble());
    iekfPosProcessVarSpin_->setToolTip(QStringLiteral("每帧预测阶段累加给位置不确定度的方差(mm²)，越大越\"信新观测\"、越小越\"信运动模型\"。"));
    backendRow->addWidget(iekfPosProcessVarSpin_);
    backendRow->addWidget(new QLabel(QStringLiteral("速度过程噪声：")));
    iekfVelProcessVarSpin_ = new QDoubleSpinBox;
    iekfVelProcessVarSpin_->setRange(0.01, 10000.0);
    iekfVelProcessVarSpin_->setSingleStep(5.0);
    iekfVelProcessVarSpin_->setDecimals(2);
    iekfVelProcessVarSpin_->setValue(settings_.value(QStringLiteral("iekfVelProcessVar"), 25.0).toDouble());
    iekfVelProcessVarSpin_->setToolTip(QStringLiteral("同上，作用在速度分量((mm/帧)²)，决定滤波器对变速的响应快慢。"));
    backendRow->addWidget(iekfVelProcessVarSpin_);

    // 【新增·IEKF自适应门控开关行】只在勾了IEKF后端时有意义。全部热改生效
    // (tryCluster()每帧从控件把值灌进iekfTracker_)，默认值按合成实验实测
    // 定：马氏门控/双锚点/coast抑制 默认开(有净收益或无害)；机动自适应过程
    // 噪声 默认关(实测有害)；基数上限 默认0=关(手套设20，是安全网非根治)。
    auto* iekfRow = new FlowLayout();
    iekfMahaGateChk_ = new QCheckBox(QStringLiteral("马氏门控"), this);
    iekfMahaGateChk_->setChecked(settings_.value(QStringLiteral("iekfMahaGate"), true).toBool());
    iekfMahaGateChk_->setToolTip(QStringLiteral(
        "用滤波器自身协方差做卡方门控，替代固定欧氏圆。不确定时门控自动放宽、\n"
        "确定时收紧。实测静止点+4mm噪声编号churn 12→1。默认开。"));
    iekfRow->addWidget(iekfMahaGateChk_);
    iekfRow->addWidget(new QLabel(QStringLiteral("卡方阈值：")));
    iekfChiSquareSpin_ = new QDoubleSpinBox;
    iekfChiSquareSpin_->setRange(1.0, 40.0);
    iekfChiSquareSpin_->setSingleStep(1.0);
    iekfChiSquareSpin_->setDecimals(1);
    iekfChiSquareSpin_->setValue(settings_.value(QStringLiteral("iekfChiSquare"), 16.0).toDouble());
    iekfChiSquareSpin_->setToolTip(QStringLiteral("3自由度卡方门控阈值。7.81=95%,11.34=99%,14.16=99.7%。默认16(略宽给余量)。"));
    iekfRow->addWidget(iekfChiSquareSpin_);

    iekfDualAnchorChk_ = new QCheckBox(QStringLiteral("双锚点"), this);
    iekfDualAnchorChk_->setChecked(settings_.value(QStringLiteral("iekfDualAnchor"), true).toBool());
    iekfRow->addWidget(iekfDualAnchorChk_);

    iekfCoastSuppressChk_ = new QCheckBox(QStringLiteral("coast抑制"), this);
    iekfCoastSuppressChk_->setChecked(settings_.value(QStringLiteral("iekfCoastSuppress"), true).toBool());
    iekfCoastSuppressChk_->setToolTip(QStringLiteral(
        "正在coast的确认轨迹附近不生成竞争新轨迹，留给它下一帧自己捡回(从\n"
        "TemporalTracker移植)。有双锚点时基本冗余，但无害、可能在多点场景帮忙。默认开。"));
    iekfRow->addWidget(iekfCoastSuppressChk_);

    iekfRow->addWidget(new QLabel(QStringLiteral("速度上限增益：")));
    iekfVelCapGainSpin_ = new QDoubleSpinBox;
    iekfVelCapGainSpin_->setRange(0.0, 8.0);
    iekfVelCapGainSpin_->setSingleStep(0.5);
    iekfVelCapGainSpin_->setDecimals(2);
    iekfVelCapGainSpin_->setValue(settings_.value(QStringLiteral("iekfVelCapGain"), 1.5).toDouble());
    iekfVelCapGainSpin_->setToolTip(QStringLiteral(
        "confirmed真点的关联硬上限=maxAssocDist+本增益×估计速度，让快点门控\n"
        "按自身速度张开。设0=退回固定门控(配合下面各开关全关即完全复现旧行为)。默认1.5。"));
    iekfRow->addWidget(iekfVelCapGainSpin_);

    iekfManeuverQChk_ = new QCheckBox(QStringLiteral("机动自适应Q⚠"), this);
    iekfManeuverQChk_->setChecked(settings_.value(QStringLiteral("iekfManeuverQ"), false).toBool());
    iekfManeuverQChk_->setToolTip(QStringLiteral(
        "⚠实测有害，务必保持关！机动时放大过程噪声，本意是撑大协方差椭球、\n"
        "门控变宽利于兜住机动，但实际会让状态估计跟着噪声跑、恒速预测飞出去，\n"
        "churn反而暴涨(80mm/8Hz 49→400+)。保留仅为不删能力。默认关。"));
    iekfRow->addWidget(iekfManeuverQChk_);

    iekfRow->addWidget(new QLabel(QStringLiteral("基数上限：")));
    iekfMaxConfirmedSpin_ = new QSpinBox;
    iekfMaxConfirmedSpin_->setRange(0, 200);
    iekfMaxConfirmedSpin_->setSingleStep(1);
    iekfMaxConfirmedSpin_->setValue(settings_.value(QStringLiteral("iekfMaxConfirmed"), 0).toInt());
    iekfRow->addWidget(iekfMaxConfirmedSpin_);

    iekfDensityGateChk_ = new QCheckBox(QStringLiteral("密度自适应门控"), this);
    iekfDensityGateChk_->setChecked(settings_.value(QStringLiteral("iekfDensityGate"), true).toBool());
    iekfRow->addWidget(iekfDensityGateChk_);
    iekfRow->addWidget(new QLabel(QStringLiteral("够到倍数：")));
    iekfDensityReachSpin_ = new QDoubleSpinBox;
    iekfDensityReachSpin_->setRange(1.0, 40.0);
    iekfDensityReachSpin_->setSingleStep(1.0);
    iekfDensityReachSpin_->setDecimals(1);
    iekfDensityReachSpin_->setValue(settings_.value(QStringLiteral("iekfDensityReach"), 12.0).toDouble());
    iekfDensityReachSpin_->setToolTip(QStringLiteral("孤立候选最大可够到 关联距离 的这个倍数。越大越能追更快的稀疏点。"));
    iekfRow->addWidget(iekfDensityReachSpin_);
    // 「孤立判据：」原先后面紧跟的是另一个标签「观测方差地板(px)：」，
    // 两个标签被 isStickyLabel 粘成一组，它自己的 densitySep 输入框反而
    // 被甩到后面 —— 截图上就是文字挨着文字、框对不上号。先把它跟自己的
    // 输入框放一起，再放观测方差地板那一对。
    iekfRow->addWidget(new QLabel(QStringLiteral("孤立判据：")));
    iekfDensitySepSpin_ = new QDoubleSpinBox;
    iekfDensitySepSpin_->setRange(1.5, 10.0);
    iekfDensitySepSpin_->setSingleStep(0.5);
    iekfDensitySepSpin_->setDecimals(1);
    iekfDensitySepSpin_->setValue(settings_.value(QStringLiteral("iekfDensitySep"), 2.5).toDouble());
    iekfDensitySepSpin_->setToolTip(QStringLiteral("次近候选须>=本值×最近，才认为最近这个\"孤立\"、可以放开够到。越大越保守(越不容易放开)。"));
    iekfRow->addWidget(iekfDensitySepSpin_);

    iekfRow->addWidget(new QLabel(QStringLiteral("观测方差地板(px)：")));
    iekfObsFloorPxSpin_ = new QDoubleSpinBox;
    iekfObsFloorPxSpin_->setRange(0.0, 5.0);
    iekfObsFloorPxSpin_->setSingleStep(0.1);
    iekfObsFloorPxSpin_->setDecimals(2);
    iekfObsFloorPxSpin_->setValue(settings_.value(QStringLiteral("iekfObsFloorPx"), 0.0).toDouble());
    connect(iekfObsFloorPxSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double){
        settings_.setValue(QStringLiteral("iekfObsFloorPx"), iekfObsFloorPxSpin_->value());
        applyIekfObsFloor();
    });
    iekfRow->addWidget(iekfObsFloorPxSpin_);
    iekfRow->addStretch(1);
    v->addLayout(iekfRow);

    // 【新增·补全先前只在代码里、没有UI控件的IEKF参数】另起一行，跟第一行
    // (马氏/双锚/coast/速度增益/机动Q/基数上限/密度门控)区分开，避免第一行
    // 过长。这里全是"已经实现、默认值经过压测验证，但没暴露"的参数。
    auto* iekfRow2 = new FlowLayout();

    iekfGlobalAssignChk_ = new QCheckBox(QStringLiteral("IEKF全局最优指派"), this);
    iekfGlobalAssignChk_->setChecked(settings_.value(QStringLiteral("iekfGlobalAssign"), true).toBool());
    iekfRow2->addWidget(iekfGlobalAssignChk_);

    iekfTentRampChk_ = new QCheckBox(QStringLiteral("tentative速度爬坡"), this);
    iekfTentRampChk_->setChecked(settings_.value(QStringLiteral("iekfTentRamp"), true).toBool());
    iekfRow2->addWidget(iekfTentRampChk_);
    iekfRow2->addWidget(new QLabel(QStringLiteral("爬坡命中数：")));
    iekfTentRampHitsSpin_ = new QDoubleSpinBox;
    iekfTentRampHitsSpin_->setRange(1.0, 20.0);
    iekfTentRampHitsSpin_->setSingleStep(1.0);
    iekfTentRampHitsSpin_->setDecimals(0);
    iekfTentRampHitsSpin_->setValue(settings_.value(QStringLiteral("iekfTentRampHits"), 3.0).toDouble());
    iekfTentRampHitsSpin_->setToolTip(QStringLiteral("连续命中这么多次后，新轨迹拿到跟已确认轨迹一样的完整速度放宽。"));
    iekfRow2->addWidget(iekfTentRampHitsSpin_);

    iekfRow2->addWidget(new QLabel(QStringLiteral("速度硬顶倍数：")));
    iekfConfirmedCapMaxSpin_ = new QDoubleSpinBox;
    iekfConfirmedCapMaxSpin_->setRange(1.0, 20.0);
    iekfConfirmedCapMaxSpin_->setSingleStep(0.5);
    iekfConfirmedCapMaxSpin_->setDecimals(1);
    iekfConfirmedCapMaxSpin_->setValue(settings_.value(QStringLiteral("iekfConfirmedCapMax"), 8.0).toDouble());
    iekfRow2->addWidget(iekfConfirmedCapMaxSpin_);

    iekfRow2->addStretch(1);
    v->addLayout(iekfRow2);

    // 第三行：协方差地板 + 机动自适应Q的三个子参数(只在勾了"机动自适应Q⚠"
    // 时才真正生效，但地板值不管勾没勾都在用)。
    auto* iekfRow3 = new FlowLayout();

    iekfUseAutoFloorChk_ = new QCheckBox(QStringLiteral("协方差地板自动"), this);
    iekfUseAutoFloorChk_->setChecked(settings_.value(QStringLiteral("iekfUseAutoFloor"), true).toBool());
    iekfRow3->addWidget(iekfUseAutoFloorChk_);
    iekfRow3->addWidget(new QLabel(QStringLiteral("手动地板(mm²)：")));
    iekfFloorVarSpin_ = new QDoubleSpinBox;
    iekfFloorVarSpin_->setRange(0.01, 1000.0);
    iekfFloorVarSpin_->setSingleStep(1.0);
    iekfFloorVarSpin_->setDecimals(2);
    iekfFloorVarSpin_->setValue(settings_.value(QStringLiteral("iekfFloorVar"), 25.0).toDouble());
    iekfFloorVarSpin_->setToolTip(QStringLiteral("只在左边\"协方差地板自动\"不勾时生效。"));
    iekfRow3->addWidget(iekfFloorVarSpin_);

    iekfRow3->addWidget(new QLabel(QStringLiteral("机动阈值：")));
    iekfManeuverThreshSpin_ = new QDoubleSpinBox;
    iekfManeuverThreshSpin_->setRange(0.01, 2.0);
    iekfManeuverThreshSpin_->setSingleStep(0.05);
    iekfManeuverThreshSpin_->setDecimals(2);
    iekfManeuverThreshSpin_->setValue(settings_.value(QStringLiteral("iekfManeuverThresh"), 0.3).toDouble());
    iekfManeuverThreshSpin_->setToolTip(QStringLiteral(
        "只在勾了\"机动自适应Q⚠\"时生效。残差占关联距离的比例超过这个阈值，\n"
        "判定为\"这条轨迹正在机动(急动/反向)\"，触发过程噪声临时放大。"));
    iekfRow3->addWidget(iekfManeuverThreshSpin_);
    iekfRow3->addWidget(new QLabel(QStringLiteral("Boost上限：")));
    iekfManeuverBoostMaxSpin_ = new QDoubleSpinBox;
    iekfManeuverBoostMaxSpin_->setRange(1.0, 100.0);
    iekfManeuverBoostMaxSpin_->setSingleStep(1.0);
    iekfManeuverBoostMaxSpin_->setDecimals(1);
    iekfManeuverBoostMaxSpin_->setValue(settings_.value(QStringLiteral("iekfManeuverBoostMax"), 40.0).toDouble());
    iekfRow3->addWidget(iekfManeuverBoostMaxSpin_);
    iekfRow3->addWidget(new QLabel(QStringLiteral("Boost衰减：")));
    iekfManeuverDecaySpin_ = new QDoubleSpinBox;
    iekfManeuverDecaySpin_->setRange(0.5, 0.999);
    iekfManeuverDecaySpin_->setSingleStep(0.01);
    iekfManeuverDecaySpin_->setDecimals(3);
    iekfManeuverDecaySpin_->setValue(settings_.value(QStringLiteral("iekfManeuverDecay"), 0.85).toDouble());
    iekfManeuverDecaySpin_->setToolTip(QStringLiteral("Boost倍率每帧向1衰减的比例，越接近1衰减越慢(机动结束后门控宽松状态持续越久)。"));
    iekfRow3->addWidget(iekfManeuverDecaySpin_);
    iekfRow3->addStretch(1);
    v->addLayout(iekfRow3);

    v = beginSection(QStringLiteral("输出滤波（One Euro）"), QStringLiteral("filter"), false);
    // 【新增·③输出端One Euro滤波】跟三角化调试窗口验证过的是同一个算法
    // (OneEuroFilter3)，这里独立移植进两个追踪后端各自的InternalTrack。
    // 只平滑对外发布的position，不反馈进内部滤波/运动模型状态，不会跟
    // IEKF自己的协方差估计打架。两个后端(IEKF/启发式)共用同一组控件，
    // 切换后端时都读这几个值——避免维护两份重复的UI。默认关：点云测试
    // 本身是用来看"原始精度"的工具，不该默默把信号平滑掉，需要的场景
    // 手动开、配合下面"抖动"相关统计对比开关前后的差异。
    auto* outputFilterRow = new FlowLayout();
    useOutputFilterChk_ = new QCheckBox(QStringLiteral("③输出端One Euro滤波"), this);
    useOutputFilterChk_->setChecked(settings_.value(QStringLiteral("useOutputFilter"), false).toBool());
    connect(useOutputFilterChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("useOutputFilter"), v);
        tracker_.setUseOutputFilter(v);
        iekfTracker_.setUseOutputFilter(v);
    });
    outputFilterRow->addWidget(useOutputFilterChk_);
    outputFilterRow->addWidget(new QLabel(QStringLiteral("minCutoff：")));
    outputFilterMinCutoffSpin_ = new QDoubleSpinBox;
    outputFilterMinCutoffSpin_->setRange(0.1, 100.0);
    outputFilterMinCutoffSpin_->setSingleStep(1.0);
    outputFilterMinCutoffSpin_->setDecimals(1);
    outputFilterMinCutoffSpin_->setValue(settings_.value(QStringLiteral("outputFilterMinCutoff"), 30.0).toDouble());
    outputFilterMinCutoffSpin_->setToolTip(QStringLiteral("慢速时的基础截止频率(Hz)，越小越平滑(但越\"黏\")。默认值取自三角化调试窗口已验证的一组。"));
    outputFilterRow->addWidget(outputFilterMinCutoffSpin_);
    outputFilterRow->addWidget(new QLabel(QStringLiteral("beta：")));
    outputFilterBetaSpin_ = new QDoubleSpinBox;
    outputFilterBetaSpin_->setRange(0.0, 5.0);
    outputFilterBetaSpin_->setSingleStep(0.05);
    outputFilterBetaSpin_->setDecimals(2);
    outputFilterBetaSpin_->setValue(settings_.value(QStringLiteral("outputFilterBeta"), 0.5).toDouble());
    outputFilterBetaSpin_->setToolTip(QStringLiteral("速度对截止频率的影响系数，越大则快速运动时越跟手(抗延迟)，但快速抖动的抑制也越弱。"));
    outputFilterRow->addWidget(outputFilterBetaSpin_);
    auto applyOutputFilterParams = [this]{
        settings_.setValue(QStringLiteral("outputFilterMinCutoff"), outputFilterMinCutoffSpin_->value());
        settings_.setValue(QStringLiteral("outputFilterBeta"), outputFilterBetaSpin_->value());
        tracker_.setOutputFilterParams(outputFilterMinCutoffSpin_->value(), outputFilterBetaSpin_->value());
        iekfTracker_.setOutputFilterParams(outputFilterMinCutoffSpin_->value(), outputFilterBetaSpin_->value());
    };
    connect(outputFilterMinCutoffSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyOutputFilterParams](double){ applyOutputFilterParams(); });
    connect(outputFilterBetaSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyOutputFilterParams](double){ applyOutputFilterParams(); });
    outputFilterRow->addStretch(1);
    v->addLayout(outputFilterRow);

    v = secCluster;   // IRLS 精修属于聚类那一步，回填到「多视角聚类」小节
    // 【新增·②鲁棒IRLS的两个内部参数】此前只能用TriangulationRefineConfig{}
    // 默认值(0.01/15)，硬编码在clusterMultiView调用处，现在可调。这两个
    // 参数控制的正是"这一版鲁棒IRLS软加权到底激不激进"，跟聚类那一行的
    // maxSampson/maxReprojNorm是完全不同的两道关卡：那两个是"要不要接纳
    // 这个观测参与三角化"的硬性资格判定；这两个是"已经接纳的观测里，谁的
    // 权重该打多少折"的软性精修判定。
    auto* lmRow = new FlowLayout();
    lmRow->addWidget(new QLabel(QStringLiteral("②IRLS Huber阈值：")));
    lmHuberDeltaSpin_ = new QDoubleSpinBox;
    lmHuberDeltaSpin_->setRange(0.0005, 0.2);
    lmHuberDeltaSpin_->setSingleStep(0.001);
    lmHuberDeltaSpin_->setDecimals(4);
    lmHuberDeltaSpin_->setValue(settings_.value(QStringLiteral("lmHuberDelta"), 0.003).toDouble());
    lmRow->addWidget(lmHuberDeltaSpin_);
    lmRow->addWidget(new QLabel(QStringLiteral("LM最大迭代：")));
    lmMaxItersSpin_ = new QSpinBox;
    lmMaxItersSpin_->setRange(1, 100);
    lmMaxItersSpin_->setValue(settings_.value(QStringLiteral("lmMaxIters"), 15).toInt());
    lmMaxItersSpin_->setToolTip(QStringLiteral("LM迭代精修的最大轮数，通常5~10轮内收敛，调大基本不影响结果只影响极端情况下的计算量。"));
    lmRow->addWidget(lmMaxItersSpin_);
    lmRow->addWidget(new QLabel(QStringLiteral("标定不确定度σ：")));
    calibSigmaNormSpin_ = new QDoubleSpinBox;
    calibSigmaNormSpin_->setRange(0.0, 0.05);
    calibSigmaNormSpin_->setSingleStep(0.0005);
    calibSigmaNormSpin_->setDecimals(4);
    // 【默认值改动】原来是 0.0（=关闭）。改成 0.0015，这是压测台上量出来的
    // 折中点——这个参数是单调权衡，不是越大越好：
    //   往大调，抗标定误差越强：主点误差2px 场景 ID跳变 343→3、幽灵率
    //   0.49→0.009；标定热漂移场景 ID跳变 513→5.5、幽灵率 0.345→0.006；
    //   检测器过于自信(报的σ只有真值40%)时 ID跳变 104→0。
    //   往大调，抗重尾噪声越弱：门控放宽的同时也放进了远超3σ的坏样本，
    //   重尾场景 RMSE 0.692→0.857mm、P95 1.09→1.75mm；重遮挡场景 ID跳变
    //   37→46。
    // 取 0.0015 是因为收益侧几乎已经吃满（再到 0.002 只多拿 ID跳变 3→0），
    // 而代价侧还没怎么涨。干净/高精度场景 0 与 0.0015 完全一致（无代价）。
    // 你的标定质量比压测台假设的好就往下调，差就往上调（0.002~0.004 都用过）。
    calibSigmaNormSpin_->setValue(settings_.value(QStringLiteral("calibSigmaNorm"), 0.0015).toDouble());
    lmRow->addWidget(calibSigmaNormSpin_);
    lmRow->addStretch(1);
    v->addLayout(lmRow);

    secTime = beginSection(QStringLiteral("时间对齐"), QStringLiteral("time"), false);
    v = secTime;
    // ---- 观测时刻补偿（相机不同步 / 运动模糊曝光中点）----
    // 这一整行对应的机制早就写在 PointIEKF.hpp / IekfPointTracker.hpp 里了
    // (CameraMeasurement::dt + projectAndJacobian 的速度外推)，但一直没有
    // 任何调用方去设置那两个 setter，dt 恒为 0，等于死代码。这里把它接上。
    auto* dtRow = new FlowLayout();
    iekfDesyncCompChk_ = new QCheckBox(QStringLiteral("相机不同步补偿"));
    iekfDesyncCompChk_->setChecked(settings_.value(QStringLiteral("iekfDesyncComp"), true).toBool());
    dtRow->addWidget(iekfDesyncCompChk_);

    dtRow->addWidget(new QLabel(QStringLiteral("曝光时长(ms)：")));
    iekfExposureMsSpin_ = new QDoubleSpinBox;
    iekfExposureMsSpin_->setRange(0.0, 50.0);
    iekfExposureMsSpin_->setSingleStep(0.5);
    iekfExposureMsSpin_->setDecimals(2);
    iekfExposureMsSpin_->setValue(settings_.value(QStringLiteral("iekfExposureMs"), 0.0).toDouble());
    dtRow->addWidget(iekfExposureMsSpin_);

    dtRow->addWidget(new QLabel(QStringLiteral("时间戳位置：")));
    iekfExposureAnchorCombo_ = new QComboBox;
    iekfExposureAnchorCombo_->addItem(QStringLiteral("曝光结束(UVC/DShow常见)"));
    iekfExposureAnchorCombo_->addItem(QStringLiteral("曝光开始"));
    iekfExposureAnchorCombo_->addItem(QStringLiteral("已是曝光中点"));
    iekfExposureAnchorCombo_->setCurrentIndex(settings_.value(QStringLiteral("iekfExposureAnchor"), 0).toInt());
    dtRow->addWidget(iekfExposureAnchorCombo_);
    dtRow->addStretch(1);
    v->addLayout(dtRow);

    v = secCluster;   // 也是聚类阶段的降级策略，回填到「多视角聚类」
    // ---- 两视图降级通道（解决"点只被2台相机看到就整个消失"）----
    auto* twoViewRow = new FlowLayout();
    twoViewFallbackChk_ = new QCheckBox(QStringLiteral("允许2视角出点"));
    twoViewFallbackChk_->setChecked(
        settings_.value(QStringLiteral("twoViewFallback"), true).toBool());
    twoViewRow->addWidget(twoViewFallbackChk_);
    twoViewRow->addWidget(new QLabel(QStringLiteral("2视角最小夹角(°)：")));
    twoViewMinRayAngleSpin_ = new QDoubleSpinBox;
    twoViewMinRayAngleSpin_->setRange(0.0, 60.0);
    twoViewMinRayAngleSpin_->setSingleStep(1.0);
    twoViewMinRayAngleSpin_->setDecimals(1);
    twoViewMinRayAngleSpin_->setValue(
        settings_.value(QStringLiteral("twoViewMinRayAngle"), 10.0).toDouble());
    twoViewRow->addWidget(twoViewMinRayAngleSpin_);
    twoViewRow->addWidget(new QLabel(QStringLiteral("最终残差上限：")));
    maxFinalResidualSpin_ = new QDoubleSpinBox;
    maxFinalResidualSpin_->setRange(0.0, 0.2);
    maxFinalResidualSpin_->setSingleStep(0.002);
    maxFinalResidualSpin_->setDecimals(4);
    maxFinalResidualSpin_->setValue(
        settings_.value(QStringLiteral("maxFinalResidual"), 0.01).toDouble());
    twoViewRow->addWidget(maxFinalResidualSpin_);
    twoViewRow->addStretch(1);
    v->addLayout(twoViewRow);

    v = secBackend;   // 检测噪声是喂给 IEKF 的，回填到「追踪后端」
    // 【新增】检测噪声(像素)——见头文件 iekfDetectNoisePxSpin_ 的完整说明。
    // 单独起一行放在 IEKF 高级选项下面，跟"位置/速度过程噪声"那行区分开，
    // 强调它是"观测噪声"(告诉滤波器测量有多准)，跟"过程噪声"(告诉滤波器
    // 运动模型有多准)是两件完全不同的事——两者经常被混着调，容易调乱。
    auto* iekfNoiseRow = new FlowLayout();
    iekfNoiseRow->addWidget(new QLabel(QStringLiteral("检测噪声(像素)：")));
    iekfDetectNoisePxSpin_ = new QDoubleSpinBox;
    iekfDetectNoisePxSpin_->setRange(0.05, 10.0);
    iekfDetectNoisePxSpin_->setSingleStep(0.05);
    iekfDetectNoisePxSpin_->setDecimals(2);
    iekfDetectNoisePxSpin_->setValue(settings_.value(QStringLiteral("iekfDetectNoisePx"), 0.8).toDouble());
    connect(iekfDetectNoisePxSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double){
        settings_.setValue(QStringLiteral("iekfDetectNoisePx"), iekfDetectNoisePxSpin_->value());
        applyIekfDefaultObsVar();
    });
    iekfNoiseRow->addWidget(iekfDetectNoisePxSpin_);
    obsVarPreviewLabel_ = new QLabel();
    obsVarPreviewLabel_->setStyleSheet(QStringLiteral("color:%1;").arg(theme::hex(theme::textDim())));
    iekfNoiseRow->addWidget(obsVarPreviewLabel_);
    iekfNoiseRow->addStretch(1);
    v->addLayout(iekfNoiseRow);


    // 【读数搬到底部状态条】status_/稳定性/耗时/滞后这四个都是"看"的东西，
    // 原来跟"调"的控件混在同一条滚动区里，调参时得来回滚动才能看见效果。
    // 现在统一沉到窗口底部一条常驻状态条上，跟点云同屏，永远可见。
    status_ = new ElidingLabel(QStringLiteral("勾选相机后自动开始"));
    addSep();
    statusStrip->addWidget(status_);
    // 【弹簧放这里，不放最后】status_（"已连接3台相机，本帧检测到…"）是这一条
    // 上唯一长度会大幅变化的一段，让它左对齐、后面的读数右对齐，中间由弹簧
    // 吸收变化 —— 这样右边那几组读数的位置就是固定的，眼睛有固定落点。
    // 原来弹簧在最末尾，所有东西挤在左半边、右边一大片空（见截图）。
    statusStrip->addStretch(1);

    // 【新增】编号稳定性统计——独立于status_(那个是"这一帧解出了几个点"
    // 这种瞬时状态)，这个是"从上次重置到现在，累计新增了多少编号 vs
    // 同期最多同时活跃多少编号"，回答的是"稳不稳"这个跨帧的问题。
    auto* statsRow = new QHBoxLayout();
    stabilityLabel_ = new ElidingLabel(QStringLiteral("稳定性统计：（尚无数据）"));
    statsRow->addWidget(stabilityLabel_, 1);
    resetStatsBtn_ = new QPushButton(QStringLiteral("重置统计"), this);
    resetStatsBtn_->setToolTip(QStringLiteral(
        "只清空下面这行的计数起点，不影响正在追踪的点/编号——调完参数或\n"
        "切换算法后点一下，从这一刻开始重新统计，避免早期抖动污染新数据。"));
    connect(resetStatsBtn_, &QPushButton::clicked, this, [this]{ resetStabilityStats(); });
    statsRow->addWidget(resetStatsBtn_);
    addSep();
    statusStrip->addLayout(statsRow);

    // 【新增】耗时统计——排查"是不是处理跟不上帧率、静默丢帧"专用，
    // 见头文件对应成员的说明。同一个"重置统计"按钮一起清零，逻辑上
    // 都是"这一段测试从这一刻开始重新算"。
    timingLabel_ = new ElidingLabel(QStringLiteral("耗时统计：（尚无数据）"));
    addSep();
    statusStrip->addWidget(timingLabel_);

    v = secTime;      // 批处理窗口也是时间对齐问题，回填到「时间对齐」
    // 【新增】批处理窗口——见头文件batchWindowSpin_的说明，本质是"等同一
    // 时刻各相机的观测都到齐"，不是给计算腾时间。
    auto* batchRow = new FlowLayout();
    batchRow->addWidget(new QLabel(QStringLiteral("批处理窗口(ms)：")));
    batchWindowSpin_ = new QSpinBox;
    batchWindowSpin_->setRange(1, 100);
    batchWindowSpin_->setValue(settings_.value(QStringLiteral("batchWindowMs"), 12).toInt());
    connect(batchWindowSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v){
        settings_.setValue(QStringLiteral("batchWindowMs"), v);
    });
    batchRow->addWidget(batchWindowSpin_);
    batchRow->addStretch(1);
    v->addLayout(batchRow);

    v = beginSection(QStringLiteral("绑定与视角"), QStringLiteral("bind"), true);
    // 【新增】骨骼绑定测试——手动点两个点连一条线，看连出来的"骨架"
    // 动作对不对得上，不经过任何标定/运动学模型。
    auto* bindRow = new FlowLayout();
    bindModeChk_ = new QCheckBox(QStringLiteral("绑定模式（勾上后依次点两个点连一条线）"), this);
    connect(bindModeChk_, &QCheckBox::toggled, this, &PointCloudTestDialog::onBindModeToggled);
    bindRow->addWidget(bindModeChk_);
    clearEdgesBtn_ = new QPushButton(QStringLiteral("清除全部连线"), this);
    connect(clearEdgesBtn_, &QPushButton::clicked, this, &PointCloudTestDialog::onClearEdges);
    bindRow->addWidget(clearEdgesBtn_);
    resetViewBtn_ = new QPushButton(QStringLiteral("复位视角"), this);
    resetViewBtn_->setToolTip(QStringLiteral("转晕了点这个——只重置观察视角(旋转/缩放/平移)，不影响点云数据或编号。"));
    bindRow->addWidget(resetViewBtn_);
    bindRow->addStretch(1);
    v->addLayout(bindRow);

    bindStatusLabel_ = new QLabel(QStringLiteral(
        "未开启绑定模式。开启后点第一个点、再点第二个点即可连线。"
        "（连的是临时编号：相机组合变化、或点被重捕后会失效。）\n"
        "视角：左键拖拽旋转，右键/中键拖拽平移，滚轮缩放。"), this);
    bindStatusLabel_->setWordWrap(true);
    v->addWidget(bindStatusLabel_);
}

} // namespace mocap
