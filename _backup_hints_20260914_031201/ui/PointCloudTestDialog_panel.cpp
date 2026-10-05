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
    maxReprojSpin_->setToolTip(QStringLiteral(
        "用候选点去考其余相机时的重投影误差阈值，通常跟maxSampson取同一\n"
        "量级(见MultiViewCluster.hpp说明)。⚠ 调大这个值会更容易接纳病态\n"
        "三角化(两条视线夹角很小导致的解)，如果发现偶尔在很远的地方冒出\n"
        "幽灵点，优先去调下面的\"最小视线夹角\"，而不是一味调紧这个数——\n"
        "两者是不同维度的问题，调紧这个会连累大量正常观测。"));
    paramRow->addWidget(maxReprojSpin_);
    paramRow->addWidget(new QLabel(QStringLiteral("最少支持视角：")));
    minSupportSpin_ = new QSpinBox;
    minSupportSpin_->setRange(2, 32);
    minSupportSpin_->setValue(settings_.value(QStringLiteral("minSupportViews"), 3).toInt());
    minSupportSpin_->setToolTip(QStringLiteral(
        "一个候选点至少要有多少台相机同时验证到才算\"追踪到\"。默认3=第三台\n"
        "相机当裁判，能有效压掉极线歧义产生的幽灵点，代价是点被部分相机\n"
        "遮挡(哪怕剩下的相机数≥2)时也会被判定\"这一帧不存在\"——总相机数\n"
        "越多，被这个门槛卡住的概率反而越高(遮挡2台在4台系统里稀松平常)。\n"
        "调到2＝只要至少2台相机看到就认，追踪更连续，但失去第三方验证，\n"
        "极线歧义场景更容易出幽灵点，请配合下面的\"稳定性统计\"观察是否\n"
        "出现幽灵编号一闪而过的情况。相机数少(比如就2台)时这个参数不太\n"
        "起作用，直接看下面的\"多相机投票\"开关。"));
    paramRow->addWidget(minSupportSpin_);
    useVotingChk_ = new QCheckBox(QStringLiteral("多相机投票"), this);
    useVotingChk_->setChecked(settings_.value(QStringLiteral("useVoting"), true).toBool());
    useVotingChk_->setToolTip(QStringLiteral(
        "默认开启：每个候选点会拿去\"考\"其余全部相机，按支持视角数仲裁，\n"
        "抗两点共线歧义能力更强，但相机数量少(尤其只有2台)时这套机制\n"
        "基本没有额外收益(没有第三台相机可以投票)，还多算一层开销。\n"
        "关闭后退回最简单的两视图三角化，直接按两视图残差从小到大\n"
        "抢占观测，不再对每个候选去考其余相机——相机数少、想要最省事\n"
        "直接的追踪时可以关掉；关闭后 minSupportViews 这个\n"
        "参数不再生效(两视图种子本来就只有2视角支持)。"));
    paramRow->addWidget(useVotingChk_);
    paramRow->addWidget(new QLabel(QStringLiteral("最小视线夹角(°)：")));
    minRayAngleSpin_ = new QDoubleSpinBox;
    minRayAngleSpin_->setRange(0.0, 60.0);
    minRayAngleSpin_->setSingleStep(1.0);
    minRayAngleSpin_->setDecimals(1);
    minRayAngleSpin_->setValue(settings_.value(QStringLiteral("minRayAngleDeg"), 0.0).toDouble());
    minRayAngleSpin_->setToolTip(QStringLiteral(
        "两台相机看同一个候选点的视线夹角低于这个值就直接拒绝，不管它的\n"
        "重投影误差看起来多小——专治\"在很远的地方突然冒出一个点\"这类\n"
        "幽灵：根因通常是两条视线夹角太小(近乎平行)，微小噪声在深度方向\n"
        "被急剧放大，2D重投影误差依然很小，但3D坐标可能跑到很远。默认0＝\n"
        "不启用(保持旧行为)。这是纯几何条件判断，跟场景大小/点的实际位置\n"
        "无关，不需要像\"最大追踪半径\"那样为每个场景重新设边界。建议从\n"
        "5~10度开始试，太大会连累相机数少、基线本来就有限时的正常观测。"));
    paramRow->addWidget(minRayAngleSpin_);
    paramRow->addWidget(new QLabel(QStringLiteral("歧义边界⚠：")));
    ambiguityMarginSpin_ = new QDoubleSpinBox;
    ambiguityMarginSpin_->setRange(0.0, 0.2);
    ambiguityMarginSpin_->setSingleStep(0.005);
    ambiguityMarginSpin_->setDecimals(3);
    ambiguityMarginSpin_->setValue(settings_.value(QStringLiteral("ambiguityMargin"), 0.0).toDouble());
    ambiguityMarginSpin_->setToolTip(QStringLiteral(
        "⚠稠密场景(如戴手套20球)务必保持0！这是投票阶段的歧义边界：一台\n"
        "相机里次佳观测与最佳观测的重投影距离差小于这个值时，认为它分不清\n"
        "该投给谁，这一票作废。实测结论：稠密簇里会把真点旁边的邻居误判成\n"
        "歧义、连累真点凑不够支持视角而消失(20球覆盖率18.7→12.6)。只对稀疏、\n"
        "球间距远的场景安全。默认0=关，保留此控件只为不删能力，不代表推荐。"));
    paramRow->addWidget(ambiguityMarginSpin_);
    useLmRefineChk_ = new QCheckBox(QStringLiteral("LM非线性精修"), this);
    useLmRefineChk_->setChecked(settings_.value(QStringLiteral("useLmRefine"), true).toBool());
    useLmRefineChk_->setToolTip(QStringLiteral(
        "候选点先用线性DLT算一个初值，再跑几步Levenberg-Marquardt非线性精修，\n"
        "降低多相机重投影残差(比纯线性解更准，尤其相机数>=3时)。默认开，\n"
        "关掉能省一点计算，精度会下降，排查\"是不是LM本身引入了什么问题\"\n"
        "时才需要关。之前这个开关一直硬编码true，没有UI控件。"));
    paramRow->addWidget(useLmRefineChk_);
    paramRow->addWidget(new QLabel(QStringLiteral("聚类卡方阈值：")));
    clusterChiSquareSpin_ = new QDoubleSpinBox;
    clusterChiSquareSpin_->setRange(1.0, 40.0);
    clusterChiSquareSpin_->setSingleStep(1.0);
    clusterChiSquareSpin_->setDecimals(1);
    clusterChiSquareSpin_->setValue(settings_.value(QStringLiteral("clusterChiSquare"), 9.21).toDouble());
    clusterChiSquareSpin_->setToolTip(QStringLiteral(
        "⚠跟下面IEKF那个\"卡方阈值\"是两个完全独立的门槛，别搞混：这个用在\n"
        "聚类阶段——候选点去\"考\"其余相机时，勾了马氏距离门控后用这个阈值\n"
        "判断某台相机的观测算不算\"对得上\"(3自由度卡方，9.21≈99%)。下面IEKF\n"
        "那个用在追踪阶段——新观测跟已有轨迹预测位置的关联门控。两者调大都是\n"
        "变宽松、调小都是变严格，但作用的环节完全不同，之前这个只能硬编码\n"
        "9.21，现在可以单独调。"));
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
    useCircleFitChk_->setToolTip(QStringLiteral(
        "默认关闭=质心法：CentroidDetector直接输出灰度加权质心，简单快。\n"
        "勾选后改用圆拟合法：用轮廓点做约束圆拟合+弧长门控(跟手部追踪\n"
        "IEKF用的是同一套算法)，半遮挡场景下理论上定位更准，但需要一个\n"
        "已知球半径先验(右边那个数)，先验给得不准反而会引入系统性偏差。\n"
        "只采纳弧长达标(≥60°)的可信观测，弧太短直接丢弃不参与三角化。"));
    detectAlgoRow->addWidget(useCircleFitChk_);
    detectAlgoRow->addWidget(new QLabel(QStringLiteral("球半径先验(像素)：")));
    circleRadiusPxSpin_ = new QDoubleSpinBox;
    circleRadiusPxSpin_->setRange(1.0, 200.0);
    circleRadiusPxSpin_->setSingleStep(0.5);
    circleRadiusPxSpin_->setDecimals(2);
    circleRadiusPxSpin_->setValue(settings_.value(QStringLiteral("circleRadiusPx"), 6.0).toDouble());
    circleRadiusPxSpin_->setToolTip(QStringLiteral(
        "圆拟合法需要的已知球半径先验，单位是图像像素(不是mm)——反光球\n"
        "在画面里的成像半径，越靠近相机越大，跟距离强相关。这里没有像\n"
        "HandTrackingWorker那样基于深度估计动态算(那需要手部模型/迭代\n"
        "估计基础设施)，直接用一个固定值代表\"典型工作距离下的成像半径\"，\n"
        "自己观察一下画面里反光球实际的像素半径，填个接近的数。"));
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
    maxContourPtsSpin_->setToolTip(QStringLiteral(
        "圆拟合只对轮廓点数不超过这个值的 blob 生效；超过的退回质心法\n"
        "(点不丢，只是不做亚像素圆拟合)。0=不限制。\n"
        "圆拟合代价随轮廓点数线性放大，正常球的轮廓几十点，几百点说明\n"
        "那不是球而是不规则亮区——正解是去检测参数里提高阈值/圆度下限\n"
        "把它滤掉，这里只是防止界面被拖住的兜底。\n"
        "状态栏\"圆拟合分流\"里的\"跳过\"数就是被这条挡下的 blob 数。"));
    connect(maxContourPtsSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v){
        settings_.setValue(QStringLiteral("maxContourPts"), v);
        // 不需要rebuild()——onBlobDetails每帧实时读这个控件的值。
    });
    detectAlgoRow->addWidget(maxContourPtsSpin_);

    useMahalanobisChk_ = new QCheckBox(QStringLiteral("马氏距离门控"), this);
    useMahalanobisChk_->setChecked(settings_.value(QStringLiteral("useMahalanobis"), false).toBool());
    useMahalanobisChk_->setToolTip(QStringLiteral(
        "只在圆拟合法下有意义：把每个观测各自的协方差(弧越短、越不确定，\n"
        "协方差自动越大)接入多相机投票的门控判据，取代固定的maxReprojNorm\n"
        "硬阈值——可信的观测门控自动收紧，不太可信的观测门控自动放宽，\n"
        "比\"一刀切\"的固定阈值更精细。质心法模式下没有单观测协方差可用，\n"
        "勾选了也不会生效。"));
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
    assocDistSpin_->setToolTip(QStringLiteral(
        "TemporalTracker的关联距离上限(mm)——注意这只是配置上限，实际生效\n"
        "值还会按当前这一帧点云的实测密度自适应收紧(见TemporalTracker.hpp\n"
        "里effectiveMaxAssocDist的说明)，这个数配的是\"最多允许多松\"。"));
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
    useHungarianChk_->setToolTip(QStringLiteral(
        "默认关闭=贪心最近邻(先到先得，计算量小)。勾选后改用匈牙利算法做\n"
        "全局最优指派——两个点距离很近/运动轨迹交叉时更不容易错配，代价是\n"
        "计算量O(N^3)，点数几十个量级下可忽略。切换立即生效，不清空已有\n"
        "编号，方便直接对比切换前后的稳定性统计数字。"));
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
    useVelSmoothChk_->setToolTip(QStringLiteral(
        "默认关闭：速度=纯位置差分(对带噪声的位置数据做数值微分，噪声被\n"
        "放大)。勾选后新速度＝\"这一帧差分值\"和\"上一次平滑速度\"的加权\n"
        "平均，权重见右边的alpha。速度更干净，预测(尤其丢帧期间的外推)\n"
        "更稳，是这几项里成本最低、最推荐先试的一个。"));
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
    useConstAccelChk_->setToolTip(QStringLiteral(
        "默认关闭：预测用恒速假设。勾选后额外估计加速度，预测改用二阶\n"
        "外推(pos+v·dt+0.5·a·dt²)，运动变向/加减速的瞬间理论上跟得更紧。\n"
        "⚠ 加速度是速度的差分，比速度本身对噪声更敏感，建议配合左边的\n"
        "速度平滑一起开——单独开在噪声较大或变速幅度小的场景可能反而更\n"
        "不稳，请用\"稳定性统计\"实测对比，不要凭直觉断定它一定更好。"));
    motionRow->addWidget(useConstAccelChk_);
    motionRow->addStretch(1);
    v->addLayout(motionRow);

    secBackend = beginSection(QStringLiteral("追踪后端 · IEKF"), QStringLiteral("backend"), false);
    v = secBackend;
    // 【新增】追踪后端整体切换——见头文件useIekfBackendChk_的说明。
    backendRow = newRow();
    useIekfBackendChk_ = new QCheckBox(QStringLiteral("IEKF卡尔曼追踪后端(替代上面这套启发式追踪)"), this);
    useIekfBackendChk_->setChecked(settings_.value(QStringLiteral("useIekfBackend"), false).toBool());
    useIekfBackendChk_->setToolTip(QStringLiteral(
        "默认关闭＝用上面这套TemporalTracker(恒速/恒加速度+距离阈值门控)。\n"
        "勾选后换成IekfPointTracker：不是先三角化出一个点再对点做运动学\n"
        "预测，而是对每台相机各自的2D观测直接做贝叶斯(卡尔曼)融合，速度\n"
        "是滤波估计出来的(不是差分)，门控原则上更精确。计算量比启发式版\n"
        "明显更高(每条轨迹维护6维协方差矩阵)，切换会完整rebuild()(两套\n"
        "追踪器的轨迹状态不能混用，编号从头开始)。"));
    backendRow->addWidget(useIekfBackendChk_);

    // 【新增】骨骼叠加开关 + 关联器构建。跟 HandTemplateStore 共用标定向导
    // 存的那份模板(不管有没有标定过——resetToPlaceholders()保证data()永远
    // 有值，占位值也能跑，只是骨架不准，这是刻意的：不该因为没标定就完全
    // 拿不到功能反馈)。onnxruntime 没装/没训练出模型时 skeletonBackend_->
    // ready()==false，勾选了也不会显示骨架(不崩，只是没效果)，见
    // Hm20OnnxBackend.hpp 桩实现的说明。
    showSkeletonChk_ = new QCheckBox(QStringLiteral("叠加显示骨架(AI关联，需要已训练模型)"), this);
    showSkeletonChk_->setChecked(settings_.value(QStringLiteral("showSkeleton"), false).toBool());
    showSkeletonChk_->setToolTip(QStringLiteral(
        "手掌位姿用HandColdStart确定性匹配(不是AI)，手指15点归属和遮挡点\n"
        "位置预测用 tools/skeleton_assoc/ 训练出的模型(ONNX)。模型没训练/\n"
        "onnxruntime没装时勾选也不会有效果，状态栏会提示。"));
    secSkeleton = beginSection(QStringLiteral("骨架关联（AI）"), QStringLiteral("skel"), false);
    v = secSkeleton;
    backendRow = newRow();
    backendRow->addWidget(showSkeletonChk_);

    // IK 开/关 —— 真机上一眼能对比出有没有用，比看数字直观。
    useIkRefineChk_ = new QCheckBox(QStringLiteral("IK精修手指姿态"));
    useIkRefineChk_->setChecked(true);
    useIkRefineChk_->setToolTip(QStringLiteral(
        "关：手指朝向 = 两球连线当骨轴，约 10° 系统偏差（标签全对也消不掉，\n"
        "    因为球贴在指节背侧、离骨轴 11~17mm，相邻两节方位角还不同）\n"
        "开：Kabsch -> 逐指 LM 解 4 个关节角 -> FK，约 3°，代价 0.12ms/帧\n"
        "\n"
        "注意：某根手指可见 marker < 2 个时 IK 无解，那根仍退回网络预测。"));
    connect(useIkRefineChk_, &QCheckBox::toggled, this, [this](bool on) {
        if (skeletonWorker_) skeletonWorker_->setIkEnabled(on);
    });
    backendRow->addWidget(useIkRefineChk_);

    // 高速模式：换 INT8 量化模型。默认关 —— 它会改变数值结果，
    // 必须让用户主动选，而且能一键切回来对照。
    fastModeChk_ = new QCheckBox(QStringLiteral("高速模式(INT8)"));
    fastModeChk_->setToolTip(QStringLiteral(
        "换成 INT8 量化模型推理。实测（三段真实录制、360 帧）：\n"
        "  单线程耗时  26.7ms -> 9.6ms   模型体积 140MB -> 36MB\n"
        "  标签一致率  98.71%\n"
        "  pos 误差    中位 0.95mm  p95 1.73mm\n"
        "\n"
        "分歧集中在模型本来就没把握的点上：被改变标签的点，原模型自己的\n"
        "置信度中位只有 0.370，而没被改变的是 0.968 —— 那些点本来就过不了\n"
        "关联器的概率门限。\n"
        "\n"
        "但遮挡多时分歧会变大（遮挡压力段一致率 97.3%，逐帧最低 85.7%）。\n"
        "所以建议先录同一段动作两次对比：看 pcrec_report 里的\n"
        "dorsumReason 分布、刚体不变量中位、numGhost 三个数有没有变差。\n"
        "\n"
        "灰掉说明没找到 INT8 模型 —— 用 tools/quantize_hm20.py 生成，\n"
        "命名 xxx_int8.onnx 放在原模型旁边。"));
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
    handRightChk_->setToolTip(QStringLiteral(
        "勾选=右手，不勾=左手。\n"
        "【标定向导里没有这个选项，以前是写死右手的】左手用户必须在这里改。\n"
        "影响两处：送进网络的条件向量里的手性位；以及 IK 的左右手镜像\n"
        "（写反时左手 IK 的外展方向整个反掉，残差常年超门限、永远不生效）。\n"
        "下方状态栏会显示系统从点云自动判出来的手性，跟这里不一致时会提示。"));
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
    romBtn_->setToolTip(QStringLiteral(
        "遥操作前做一次：按下后【五指张开到底 -> 握拳到底】来回两三遍，约5秒，再按一次结束。\n\n"
        "【为什么必须做】\n"
        "· 人手和机械手/Unity模型的行程不同（人MCP约-10~100°，机械手可能只有0~90°），\n"
        "  直接抄绝对角度会让机械手永远合不拢、也过不了伸。\n"
        "· 我们算出的掌指角带一个未知的常量零位偏置（腕部系的+X不一定正好是解剖\n"
        "  中立位方向）。归一化时 min 和 max 一起偏，相减正好抵消。\n\n"
        "实测：不做时关节触限率 100%（信号被压平）；做完后触限率 0~9%、行程利用 70~96%。"));
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
    autoCalibChk_->setToolTip(QStringLiteral(
        "拿关联结果做闭环标定：手背模板 + 指根 anchor + 骨长，全程不用手工标定向导。\n"
        "\n"
        "【对操作者的要求】不是一打开就开始标，要满足下面这些才会推进：\n"
        "  1) 整只手进视野：手背5点全被认出 + 手指至少8点\n"
        "  2) 把手摆稳约0.4秒(帧间位移<8mm、转角<5°)\n"
        "  3) 缓慢转动手腕约0.8秒 -> 冻结手背模板\n"
        "  4) 冻结后自动校验90帧，Kabsch残差站不住就推翻重标(最多6次)\n"
        "  5) 反复屈伸五指 -> 解出指根anchor和骨长，这一步最耗时\n"
        "\n"
        "面板状态栏会实时显示卡在哪一步、以及本帧被拒的原因。\n"
        "标定完成前 IK 会被自动禁用——参数没收敛就开 IK 比不开还差。"));
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
    smoothCombo_->setToolTip(QStringLiteral(
        "对20个点位置、腕部位姿、16个分段四元数做自适应时序滤波(One-Euro + 球面插值)。\n"
        "实测(6相机+不同步+遮挡, 14秒):\n"
        "  关    位置中位1.32mm  高频抖动2.771  分段帧间跳变中位4.02°\n"
        "  轻    位置中位1.48mm  抖动1.742      跳变3.55°\n"
        "  均衡  位置中位1.65mm  抖动1.500      跳变3.37°  <- 默认，位置p90反而更好\n"
        "  强    位置中位4.51mm  抖动0.542      跳变2.71°  <- 抖动最小，但滞后3.4mm\n"
        "\n"
        "被遮挡的点和\"猜出来的\"分段会自动加强平滑；marker重新出现时滤波器硬复位，不会拖尾。\n"
        "\n"
        "【\"关\"就是直通，这一档不做任何时序处理】但要注意：觉得连线不跟手时，\n"
        "先看右边的滞后读数再动这里 —— 滤波带来的滞后通常只有几毫秒，而\n"
        "\"骨架刷新\"限流带来的陈旧平均就有 16ms，量级差一个数量级。\n"
        "想一次性全关掉做 A/B，用旁边的\"低延迟直通\"。"));
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
    recProtoCombo_->setToolTip(QStringLiteral(
        "录制协议 —— 【这一项决定了录出来的数据能不能用来调参】\n"
        "没有真值就没有\"最优参数\"这回事。协议的作用是让每一段数据自带一组物理\n"
        "不变量，当近似真值用：\n"
        "  静止保持  手架稳完全不动15秒 -> 任何变化都是噪声，量抖动底噪\n"
        "  刚体运动  手型固定(手指别动)、整只手平移旋转30秒 -> 20点两两距离全恒定，\n"
        "            等于190个约束当真值。【这一段最有价值，务必录】\n"
        "  手指屈伸  手掌不动、五指反复屈伸30秒 -> 骨长恒定+手背刚体，标IK和自标定\n"
        "  遮挡压力  故意遮挡+极端角度30秒 -> 测标签稳定性、鬼点率、重捕\n"
        "  自由动作  你实际应用的动作60秒 -> 测综合表现\n"
        "  刚体标定件 贴了已知球间距的杆30秒 -> 真·真值，标绝对精度\n"
        "  张开握拳循环 张到最开停3秒->握到最紧停3秒，来回5轮，每次切换按一下标记\n"
        "            -> 【判\"输出的关节角对不对\"就录这个】。指尖到腕心的距离是\n"
        "            不依赖任何解算的物理真值，握拳时必然显著变小；拿它跟输出的\n"
        "            关节角一对，就知道是解算/平滑/ROM/限幅哪一级把信号弄反或压平了。\n"
        "  手性排查  手掌朝下/朝上/侧立各5秒，每种姿态做2~3次明显屈伸，共30秒\n"
        "            -> 【手摊平时手性判据接近0、判不出来】，必须有屈曲才有信息量。\n"
        "            这正是\"标定时一切正常、一提交就左右反\"的成因。"));
    backendRow->addWidget(recProtoCombo_);

    // ---- 录制详细度 ----
    // 【默认全录】录制这个动作本身就是为了排查，而排查时最贵的从来不是磁盘，
    // 是"录完发现关键那一项没开、得重录一遍"—— 偶发问题重录不一定复现得出来。
    recDetailCombo_ = new QComboBox();
    recDetailCombo_->addItems({QStringLiteral("精简"), QStringLiteral("完整"),
                               QStringLiteral("全量")});
    recDetailCombo_->setCurrentIndex(1);
    recDetailCombo_->setToolTip(QStringLiteral(
        "录制详细度。决定写哪些诊断块，直接影响文件体积。\n\n"
        "  精简  只写 v4 的块（2D光斑/3D点/骨架/关节角/手性/逐点来源）。\n"
        "        约 0.6MB/s。跟旧版行为一致。\n"
        "  完整  加 v5 的中间级：位置六级流水、滤波前后+滤波器内部、IK 求解器\n"
        "        内部、模板快照、时基与分阶段耗时、事件流、自标定逐帧、\n"
        "        追踪器内部、3D点↔2D光斑对应。约 1.4MB/s。【默认，推荐】\n"
        "  全量  再加全量指派矩阵和送进 ONNX 的输入张量。约 2.6MB/s。\n"
        "        只有查\"这个点为什么被判成鬼点\"、或者要离线原样重跑模型时才需要。\n\n"
        "【为什么默认是完整而不是精简】\n"
        "系统里有三处是\"就地覆写\"的：角度平滑、位置滤波、关联器后处理。\n"
        "被覆写掉的输入在这套系统里没有第二份 —— 不在录制时留一份，\n"
        "事后就永远分不开\"上一级本来就错了\"和\"这一级把它改坏了\"，\n"
        "而这两件事的修法是相反的。\n"
        "30~60 秒的排查性录制，完整档一分钟约 84MB，没有必要为此省。\n"
        "要连录十几分钟看偶发问题，再调到精简。"));
    backendRow->addWidget(recDetailCombo_);

    recNoteEdit_ = new QLineEdit();
    recNoteEdit_->setPlaceholderText(QStringLiteral("备注(可空)"));
    recNoteEdit_->setFixedWidth(140);
    backendRow->addWidget(recNoteEdit_);

    recBtn_ = new QPushButton(QStringLiteral("● 录制"));
    recBtn_->setToolTip(QStringLiteral(
        "录下原始数据流(.pcrec)：每相机2D光斑+时间戳+协方差、3D候选点、"
        "骨架输出、相机内外参、全部面板参数快照。\n"
        "【为什么要存2D】只存3D点的话，IEKF追踪那三十个参数一个也调不了——"
        "它们的输入没被保存下来。\n"
        "写盘在独立线程，采集线程不会被磁盘卡住。\n\n"
        "录完先自己体检：python3 tools/pcrec/pcrec.py check 你的文件.pcrec"));
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
    recOpenDirBtn_->setToolTip(QStringLiteral(
        "录制文件保存在【项目目录/recordings/】下，不再是系统「文档」目录。\n"
        "每个 .pcrec 旁边有一个同名 .json，是可以直接用记事本看的头信息\n"
        "（相机内外参、全部参数快照、手部模板、模型指纹、运行环境）。\n"
        "打包发人时两个文件一起发。"));
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
    showOutputChk_->setToolTip(QStringLiteral(
        "在右侧开一块面板，把这条链路【最终发出去】的数据逐帧打印出来：\n"
        "  · M3DS：16 维关节角（Unity/机械手真正吃的那份），带\"解算原始角 vs 实际输出角\"对照\n"
        "  · M3DQ：16 段四元数 + 每段来源（预测/几何/IK）\n"
        "  · 原始报文：UDP 上真正跑的字节，可直接跟 Unity 端或 Wireshark 逐字节对\n"
        "\n"
        "【显示的就是发出去的】面板接的是转给 UdpSender 的同一个信号，不是重新算一遍——\n"
        "所以时序滤波、ROM 映射、速度限幅这三步的效果都能在这里看见。\n"
        "\n"
        "刷新固定 10Hz（数据 120Hz 也一样），不勾选时完全不参与计算。"));
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
    lowLatencyChk_->setToolTip(QStringLiteral(
        "一键旁路整条输出滤波链，用来做 A/B：勾上→晃手→取消→再晃手，差别一眼可见。\n"
        "勾上时同时做四件事：\n"
        "  ① 骨架刷新不限流（原来写死 30Hz，平均 +16ms、最坏 +33ms 的陈旧）\n"
        "  ② AI 线程优先级 最低→普通（相机满载时它原来抢不到核心，排队比推理还久）\n"
        "  ③ 时序滤波关（Hm20PoseFilter，默认\"均衡\"档是开着的）\n"
        "  ④ 关节角速度限幅关 + 点云输出端 One-Euro 关\n"
        "\n"
        "【代价】抖动会明显变大，尤其被遮挡的手指。它是诊断工具，不是推荐设置——\n"
        "确认过\"关掉就跟手了\"之后，回去逐项打开，找到你能接受的那一档。\n"
        "右边的实测滞后读数会告诉你每一项各值多少毫秒。\n"
        "\n"
        "勾上期间被旁路的那几个控件会变灰，不会覆盖你原来的设置。"));
    backendRow->addWidget(lowLatencyChk_);

    backendRow->addWidget(new QLabel(QStringLiteral("骨架刷新:")));
    skelRateCombo_ = new QComboBox();
    skelRateCombo_->addItems({QStringLiteral("30Hz"), QStringLiteral("60Hz"),
                              QStringLiteral("120Hz"), QStringLiteral("不限流")});
    skelRateCombo_->setCurrentIndex(settings_.value(QStringLiteral("skelDispatchRate"), 0).toInt());
    skelRateCombo_->setToolTip(QStringLiteral(
        "多久给 AI 线程投一帧。【这一项原来写死 30Hz，是\"连线追着点跑\"最直接的来源】：\n"
        "点云自己可能跑 100fps 以上，骨架只有 30fps，画面上就是点已经动了、线还停在\n"
        "上一次的位置，平均慢 16ms、最坏慢 33ms。\n"
        "\n"
        "调高的代价是 AI 线程 CPU 占用成比例上涨（推理耗时×帧率）。先看右边\n"
        "\"推理\"那个数字：推理 3ms 的话 120Hz 也只占 36% 一个核，尽管调；\n"
        "推理 25ms 的话连 60Hz 都跑不满，调了也只是白排队（丢帧率会涨上去）。\n"
        "\n"
        "\"不限流\"= 每个追踪帧都投，worker 忙就跳过，等于让 AI 尽全力跑。"));
    backendRow->addWidget(skelRateCombo_);

    rateLimitChk_ = new QCheckBox(QStringLiteral("速度限幅"));
    rateLimitChk_->setChecked(settings_.value(QStringLiteral("skelRateLimit"), true).toBool());
    rateLimitChk_->setToolTip(QStringLiteral(
        "关节角每秒最多变 8rad(≈458°)。【只作用在 UDP 那一路，不影响面板上的连线】——\n"
        "所以\"面板里连线跟不上手\"跟它无关，但\"Unity 里手指跟不上\"可能就是它。\n"
        "\n"
        "它防的是标签串了/重捕那一帧甩出几十度的阶跃：伺服收到超出自身能力的阶跃，\n"
        "要么饱和过冲、要么报保护停机。遥操作里这是安全问题，不是画质问题。\n"
        "关掉之前先想清楚下游是软件模型还是真机械手。"));
    backendRow->addWidget(rateLimitChk_);

    latencyLabel_ = new ElidingLabel(QStringLiteral("滞后：—"), this);
    // 固定宽度、不换行，理由同 skeletonDiagLabel_（每 250ms 变一次的文字，
    // 开自动换行会让整个参数面板反复重排）。
    latencyLabel_->setWordWrap(false);
    latencyLabel_->setTextFormat(Qt::PlainText);
    // 原来定死宽度是为了防止数字变化导致布局抖动；现在标签自己会省略号截断，
    // 给个上限就够了，剩下的宽度让给别的读数。
    latencyLabel_->setMaximumWidth(300);
    latencyLabel_->setToolTip(QStringLiteral(
        "叠加层相对真实动作的滞后，实测值，不是估的：\n"
        "  推理  = worker 里 process() 本身的耗时（含 ONNX）\n"
        "  排队  = 投递到 worker 真正开始跑之间的等待 + 结果回传。\n"
        "         这一项大 = AI 线程抢不到 CPU（优先级/核心不够），不是模型慢\n"
        "  陈旧  = 限流造成的\"这条线已经画了多久没更新\"，平均 = 刷新间隔的一半\n"
        "总滞后 ≈ 推理 + 排队 + 陈旧，这就是你眼睛看到的\"线比点慢多少\"。"));
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
    thumbRollSpin_->setToolTip(QStringLiteral(
        "拇指三段(掌骨/近节/远节)绕骨轴的额外旋转。\n"
        "\n"
        "四指的背侧朝向跟手背一致，roll 参考取手背法线是对的；\n"
        "拇指第一掌骨有 80~90° 的解剖旋前，背侧朝向差了近 90°——\n"
        "不补这个偏置，拇指的弯曲平面会按四指来摆，表现为\"弯曲轴朝前不朝手心\"。\n"
        "\n"
        "【怎么调】弯拇指，看「分段四元数」页里\"拇 远节\"相对父节点的四元数：\n"
        "转轴应该以 z 为主、x 分量接近 0（x 大 = 还在绕骨轴打滚）。每次 ±5° 试。\n"
        "\n"
        "设 0 = 完全退回旧行为。默认 80°，个体差异在 ±15°，\n"
        "而且真实旋前会随 CMC 外展变化——这是一阶近似，不是精确解剖模型。"));
    backendRow->addWidget(thumbRollSpin_);

    thumbPronChk_ = new QCheckBox(QStringLiteral("回正预测点"));
    thumbPronChk_->setChecked(settings_.value(QStringLiteral("thumbPronOn"), true).toBool());
    thumbPronChk_->setToolTip(QStringLiteral(
        "把【遮挡时网络补出来的】拇指点，绕(CMC锚点, 掌骨轴)转回左边那个角度。\n"
        "实测点一律不动 —— 观测到的本来就是对的。\n"
        "\n"
        "【为什么需要】训练用的 hand_rig.py 里拇指旋前完全耦合在 CMC 外展上，\n"
        "没有常数基线：按它自己的关键姿态表，握拳时旋前只有 4~8°，而真人是 80~90°\n"
        "且中立位就有。所以模型学到的先验是\"拇指跟四指一样往前弯\"，一遮挡就暴露。\n"
        "\n"
        "【三条局限】\n"
        "  ① 掌骨轴只能从球估，带 11~16mm 贴球偏置，方向误差 20° 量级——一阶补偿\n"
        "  ② 要腕部位姿有效 + anchor 已标定，否则自动跳过\n"
        "  ③ 【修不了先验本身】只能把点摆进正确的平面，摆过去也不一定像真人握拳。\n"
        "     根治要改 rig 重训。\n"
        "\n"
        "如果哪天 rig 改了重训过，这里要关掉——别两边都补。"));
    backendRow->addWidget(thumbPronChk_);

    ikOnlyChk_ = new QCheckBox(QStringLiteral("遮挡点只用IK"));
    // 【默认关】开着会让 IK 补不上的遮挡点冻住，而网络 pos 头对四指的外推
    // 实测是好用的 —— 冻住反而更差。留作诊断用（想看"纯 IK 能补多少"时勾上）。
    ikOnlyChk_->setChecked(settings_.value(QStringLiteral("occludedIkOnly"), false).toBool());
    ikOnlyChk_->setToolTip(QStringLiteral(
        "遮挡的 marker 只接受 IK 摆出来的位置；IK 补不上的【保持上一帧】，\n"
        "不使用网络 pos 头的预测。\n"
        "\n"
        "【为什么需要】关节角那边 fingerValid=false 时整根手指\"保持\"，而 marker\n"
        "位置走的是另一条路(网络预测)——于是出现\"角度冻住、点却在飞\"这种自相\n"
        "矛盾的状态。角度都判定为不可信了，位置却还照单全收，说不通。\n"
        "打开后两者对齐：要么都是新算的，要么都冻住。\n"
        "\n"
        "代价：遮挡期间那颗点不动。但它本来也没有可信位置，segSource 已经\n"
        "把这件事告诉下游了；僵住至少物理上连续，比乱飞好判断。\n"
        "\n"
        "看诊断行 “遮挡点：IK补X / AI预测Y” 判断有没有东西可用。"));
    backendRow->addWidget(ikOnlyChk_);

    geoRelabelChk_ = new QCheckBox(QStringLiteral("手背几何定标签"));
    geoRelabelChk_->setChecked(settings_.value(QStringLiteral("dorsumGeoRelabel"), true).toBool());
    geoRelabelChk_->setToolTip(QStringLiteral(
        "手背 5 点的标签不信模型，改用【穷举 120 种排列 + Kabsch】重新定。\n"
        "\n"
        "【为什么】模型要在 21 类里给每个点分类，手背 5 点彼此空间最近，\n"
        "错了必然错到隔壁 —— 实测手背逐点准确率 91~96%，而\"手背<->手背\"\n"
        "占全部错误的 20%，一旦错就是整只手转 72°。\n"
        "而这本来是个刚体配准问题，有闭式最优解。仿真实测(真实模板)：\n"
        "  噪声 0.9mm -> 定标签准确率 100.0%，正解与次解差 7.63mm\n"
        "  噪声 4.0mm -> 仍有 98.2%\n"
        "裕度远超噪声，比模型可靠一个量级。\n"
        "\n"
        "【分工】模型负责\"哪些点属于手背\"(二分类，它擅长)，\n"
        "几何负责\"这 5 个点各是哪一个\"(配准，闭式解)。\n"
        "\n"
        "只在 5 点全可见、且最优与次优的残差差 >2mm 时才动手 ——\n"
        "裕度不够说明这帧观测本身有问题(粘连/重影)，此时重排是在赌。\n"
        "代价：120 次 3x3 SVD，单帧几微秒。\n"
        "\n"
        "看诊断行的\"几何裕度\"：正常 7mm+，接近 0 就该查观测质量。"));
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
    iekfDualAnchorChk_->setToolTip(QStringLiteral(
        "关联时同时对\"恒速预测点\"和\"上次实见位置\"算距离取小——这是\"单点\n"
        "大晃动开卡尔曼反而失点\"的主解药：反向甩手瞬间点回到上次位置附近，\n"
        "靠第二个锚点照样关联上，不必等滤波器把速度纠正过来。实测minHits=15、\n"
        "80mm/8Hz下可见帧 0→156。默认开。"));
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
    iekfMaxConfirmedSpin_->setToolTip(QStringLiteral(
        "手套上球数等基数先验(无遮挡时可见真点数≤此值)。>0时每帧只发布存活\n"
        "最久(hits最多)的前N条确认轨迹，把\"屏幕冒出第21个点\"硬压掉。0=关。\n"
        "注意：这是安全网不是根治——它只挡\"多出来\"的幽灵，挡不住\"顶替真点\n"
        "(总数仍≤N但点是错的)\"那种，根治仍靠检测质量/最小视线夹角/三视角仲裁。\n"
        "你的手套20球就设20。"));
    iekfRow->addWidget(iekfMaxConfirmedSpin_);

    iekfDensityGateChk_ = new QCheckBox(QStringLiteral("密度自适应门控"), this);
    iekfDensityGateChk_->setChecked(settings_.value(QStringLiteral("iekfDensityGate"), true).toBool());
    iekfDensityGateChk_->setToolTip(QStringLiteral(
        "把固定关联距离升级成随\"局部歧义度\"自适应：如果一个候选点附近只有它\n"
        "一个(其余都远得多)，认它就是安全的，不管多远——这是\"单点大晃动能追\n"
        "多快\"的关键(实测速度上限31→>104mm/帧)。稠密处近邻一堆时自动收紧，\n"
        "不放松抗幽灵能力(实测20球coverage不降、churn/ghost还更低)。默认开。"));
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
    iekfObsFloorPxSpin_->setToolTip(QStringLiteral(
        "再准的检测器也不可能比这更准——给IEKF吃进去的观测协方差设一个下限。\n"
        "\n"
        "为什么需要：检测层报的协方差只描述\"质心抖动\"(零均值、逐帧独立)，不含\n"
        "标定偏差、畸变残留、球体部分遮挡导致的质心系统性偏移。检测器越好、报的\n"
        "协方差越小，滤波器就越\"绝对相信\"这次测量，于是任何一次误关联都变成\n"
        "确信无疑的巨大跳变——轨迹被拽飞、关联不上、拿新编号。\n"
        "\n"
        "压测实测(20点手部场景，检测协方差被钉到σ=1e-4px)：\n"
        "  地板关：RMSE 6.9mm，单点最大误差 78.7mm，300帧内 54 次编号跳变\n"
        "  地板0.5px：RMSE 0.001mm，最大误差 0.001mm，编号跳变 0\n"
        "  正常噪声(0.3/0.8px)下开关结果完全一致 —— 零代价的安全网。\n"
        "\n"
        "填 0 = 关闭(旧行为)。建议填你系统标定/建模精度的量级，通常 0.3~1.0px。"));
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
    iekfGlobalAssignChk_->setToolTip(QStringLiteral(
        "⚠跟上面追踪参数行里的\"匈牙利全局最优指派\"是两个独立开关：那个只\n"
        "接了启发式后端(TemporalTracker)，勾了IEKF后端时那个复选框不起\n"
        "任何作用。这个才是IEKF自己的全局指派开关——本帧所有\"过门\"的\n"
        "(轨迹,观测)配对按分数从小到大排，全局地先成全最匹配的那些，而不是\n"
        "按轨迹顺序贪心一个个抢。默认开，防止相邻两点编号互换(实测20点云\n"
        "单帧位移12.6mm/帧时换号率从2.69降到0.53)。"));
    iekfRow2->addWidget(iekfGlobalAssignChk_);

    iekfTentRampChk_ = new QCheckBox(QStringLiteral("tentative速度爬坡"), this);
    iekfTentRampChk_->setChecked(settings_.value(QStringLiteral("iekfTentRamp"), true).toBool());
    iekfTentRampChk_->setToolTip(QStringLiteral(
        "新轨迹(还没转正)默认被死死摁在\"关联距离\"这个硬上限，不给任何速度\n"
        "放宽——这是防幽灵的代价，但也造成一个结构性天花板：能追多快的新点，\n"
        "只由\"关联距离/帧\"这一个数决定，跟卡尔曼、协方差、密度门控都无关。\n"
        "开启后，连续命中>=2次的新轨迹按命中数逐步解锁速度放宽，命中1次的\n"
        "(刚出生、大概率是幽灵)仍然钉死。默认开。"));
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
    iekfConfirmedCapMaxSpin_->setToolTip(QStringLiteral(
        "已确认轨迹的关联半径 = 关联距离 + 速度上限增益×自身速度，丢帧期间\n"
        "再放宽，但无论怎么放，最终都夹到\"关联距离×这个倍数\"作为绝对硬顶——\n"
        "同一个数也用来给coast期间的幽灵抑制区定半径，两处口径必须一致(此前\n"
        "版本这里有过\"先夹后乘\"和\"先乘后夹\"不一致的bug，已修)。调大＝\n"
        "更能追极端快速运动，但幽灵抑制区也跟着变大；调小反之。默认8。"));
    iekfRow2->addWidget(iekfConfirmedCapMaxSpin_);

    iekfRow2->addStretch(1);
    v->addLayout(iekfRow2);

    // 第三行：协方差地板 + 机动自适应Q的三个子参数(只在勾了"机动自适应Q⚠"
    // 时才真正生效，但地板值不管勾没勾都在用)。
    auto* iekfRow3 = new FlowLayout();

    iekfUseAutoFloorChk_ = new QCheckBox(QStringLiteral("协方差地板自动"), this);
    iekfUseAutoFloorChk_->setChecked(settings_.value(QStringLiteral("iekfUseAutoFloor"), true).toBool());
    iekfUseAutoFloorChk_->setToolTip(QStringLiteral(
        "马氏门控用滤波器预测协方差做卡方检验前，会先给协方差加一个\"地板\"\n"
        "(不让协方差小到门控变成一根针，正常抖动都通不过)。默认自动＝\n"
        "(0.35×关联距离)²，随关联距离等比例缩放，一般不需要手动管。不勾时\n"
        "用右边手填的固定值(mm²)，适合你想让地板跟关联距离脱钩单独调的场景。"));
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
    useOutputFilterChk_->setToolTip(QStringLiteral(
        "跟\"三角化调试\"窗口里验证过的③输出滤波是同一个算法(OneEuroFilter)，\n"
        "这里独立应用到实时动捕的两个追踪后端(IEKF/启发式都支持，切换后端\n"
        "共用这组参数)。只平滑最终发布的坐标，不影响追踪内部状态，也不影响\n"
        "编号/关联逻辑。实测：静止点抖动RMS可从1.37mm降到0.21mm。默认关——\n"
        "这是个测试工具，原始抖动量是你判断其它参数好坏的重要信号，平滑掉了\n"
        "反而看不出问题，等其它参数调好、只想要更干净的最终输出时再开。"));
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
    lmHuberDeltaSpin_->setToolTip(QStringLiteral(
        "归一化坐标单位(×相机fx≈像素)。LM精修阶段，某台相机的重投影残差\n"
        "超过这个值就开始被降权(残差越大权重越低)，这是鲁棒IRLS抗坏视角/\n"
        "遮挡噪声的核心参数。只在勾了上面\"LM非线性精修\"时生效。调小＝\n"
        "更激进地怀疑\"跟别人对不上\"的观测(更抗幽灵，但也可能误伤正常噪声\n"
        "范围内的观测)；调大＝更宽容。默认0.01。"));
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
    calibSigmaNormSpin_->setToolTip(QStringLiteral(
        "标定误差的等效标准差（归一化坐标单位；×焦距fx≈像素）。\n"
        "\n"
        "为什么需要它：检测层协方差只描述\"检测噪声\"——零均值、逐帧独立、多帧\n"
        "平均会抵消；而\"标定偏差\"是系统性的，全画幅全时段恒定，不会因为多看\n"
        "几帧就变小。两者统计性质完全不同，但马氏门控原先只建模了前者，于是\n"
        "标定偏差把整个噪声预算吃光，真观测被自己的门控判成外点。\n"
        "\n"
        "填上一个跟你实际标定质量相称的值，按方差可加性并入门控即可。\n"
        "压测实测(主点误差2px)：0 时召回率 0.13，0.002 时召回率 0.99。\n"
        "标定完美时设不设都一样(0.9933→0.9933)，所以这不是\"无脑放宽门控\"。\n"
        "\n"
        "【这是个单调权衡，不是越大越好】\n"
        "  调大 -> 抗标定误差强：主点误差2px 时 ID跳变 343->3、幽灵率 0.49->0.009；\n"
        "          标定热漂移 ID跳变 513->5.5；检测器过于自信时 ID跳变 104->0。\n"
        "  调大 -> 抗重尾噪声弱：门控放宽也放进了远超3σ的坏样本，重尾场景\n"
        "          RMSE 0.69->0.86mm、P95 1.09->1.75mm；重遮挡场景 ID跳变 37->46。\n"
        "\n"
        "默认 0.0015（f=900 时≈1.35px）：收益侧基本吃满，代价侧还没怎么涨。\n"
        "干净/高精度场景 0 与 0.0015 结果完全一致，所以这不是\"无脑放宽门控\"。\n"
        "标定好就往下调，标定差就往上调（0.002~0.004 区间都验过，仍有效）。"));
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
    iekfDesyncCompChk_->setToolTip(QStringLiteral(
        "各台相机的曝光时刻本来就不在同一瞬间（没有硬件外触发时普遍存在）。\n"
        "勾选后，用每台相机自己上报的采集时间戳跟本帧参考时刻（各相机时间戳\n"
        "的中位数）之差，算出该相机的观测时刻偏移，交给 IEKF 的观测模型：\n"
        "用当前速度把预测位置外推到【这次观测真正对应的时刻】再投影。\n"
        "\n"
        "不补偿时，残差里混进了一段\"目标在这段时间里走了多远\"，这段位移正比\n"
        "于速度，于是速度越快偏差越大，表现为\"快速运动时精度突然变差\"。\n"
        "\n"
        "不需要填任何参数——时间戳本来就在数据里。偏移量夹在±0.5帧内，超出\n"
        "范围更可能是时间戳本身异常，硬补只会更糟。\n"
        "压测实测(3500mm/s + 2ms不同步)：RMSE 2.82mm -> 1.96mm；\n"
        "2000mm/s 不同步场景 ID跳变 90 -> 58。"));
    dtRow->addWidget(iekfDesyncCompChk_);

    dtRow->addWidget(new QLabel(QStringLiteral("曝光时长(ms)：")));
    iekfExposureMsSpin_ = new QDoubleSpinBox;
    iekfExposureMsSpin_->setRange(0.0, 50.0);
    iekfExposureMsSpin_->setSingleStep(0.5);
    iekfExposureMsSpin_->setDecimals(2);
    iekfExposureMsSpin_->setValue(settings_.value(QStringLiteral("iekfExposureMs"), 0.0).toDouble());
    iekfExposureMsSpin_->setToolTip(QStringLiteral(
        "运动模糊补偿：拖尾光斑的质心落在【曝光窗口中点】，不是窗口端点。\n"
        "所以带模糊的观测实际对应的是\"时间戳 ± 曝光时长/2\"那个时刻。\n"
        "填 0 = 不补偿（默认）。填你实际用的曝光时长即可。\n"
        "\n"
        "压测实测(2000mm/s、模糊增益1.0)：RMSE 9.39mm -> 0.56mm。\n"
        "\n"
        "注意符号由右边的\"时间戳位置\"决定，填错方向会让误差变大而不是变小，\n"
        "所以我没有替你猜——这取决于你的相机驱动把时间戳打在哪一端。"));
    dtRow->addWidget(iekfExposureMsSpin_);

    dtRow->addWidget(new QLabel(QStringLiteral("时间戳位置：")));
    iekfExposureAnchorCombo_ = new QComboBox;
    iekfExposureAnchorCombo_->addItem(QStringLiteral("曝光结束(UVC/DShow常见)"));
    iekfExposureAnchorCombo_->addItem(QStringLiteral("曝光开始"));
    iekfExposureAnchorCombo_->addItem(QStringLiteral("已是曝光中点"));
    iekfExposureAnchorCombo_->setCurrentIndex(settings_.value(QStringLiteral("iekfExposureAnchor"), 0).toInt());
    iekfExposureAnchorCombo_->setToolTip(QStringLiteral(
        "你的相机驱动把帧时间戳打在曝光窗口的哪个位置，决定补偿的符号：\n"
        "  曝光结束 -> 质心对应的时刻比时间戳【早】半个曝光，dt 取负；\n"
        "  曝光开始 -> 质心对应的时刻比时间戳【晚】半个曝光，dt 取正；\n"
        "  已是中点 -> 驱动已经做过这件事，不需要再补。\n"
        "\n"
        "拿不准就用最直接的办法定：在高速运动下把曝光时长填上，两个方向各\n"
        "试一次，看\"稳定性统计\"里的RMSE往哪边降——降的那个就是对的。\n"
        "符号填反会让误差明显变大（不是变差一点，是成倍变差），很好分辨。"));
    dtRow->addWidget(iekfExposureAnchorCombo_);
    dtRow->addStretch(1);
    v->addLayout(dtRow);

    v = secCluster;   // 也是聚类阶段的降级策略，回填到「多视角聚类」
    // ---- 两视图降级通道（解决"点只被2台相机看到就整个消失"）----
    auto* twoViewRow = new FlowLayout();
    twoViewFallbackChk_ = new QCheckBox(QStringLiteral("允许2视角出点"));
    twoViewFallbackChk_->setChecked(
        settings_.value(QStringLiteral("twoViewFallback"), true).toBool());
    twoViewFallbackChk_->setToolTip(QStringLiteral(
        "关闭时：4台相机就硬性要求>=3台支持，一个点被其中两台拍得清清楚楚，\n"
        "只要另外两台因遮挡/贴太近没认出来，这个点在3D点云里就直接消失。\n"
        "\n"
        "打开时：>=2个支持视角就出点（三角化的几何下限本来就是2），但必须\n"
        "满足右边的最小视线夹角；只有2视角撑着的点会被标记为未完全验证。\n"
        "抗幽灵的责任交给追踪器的确认帧数——真实被遮挡的点在延续已有轨迹，\n"
        "幽灵点是凭空冒出来的，用时间一致性区分比用单帧支持视角数准得多。\n"
        "\n"
        "压测实测（4相机，1个点仅2台可见）：关闭时召回率0.74，打开时0.99；\n"
        "密集点+互相遮挡场景：0.20 -> 0.79，幽灵率仅从0升到0.014。"));
    twoViewRow->addWidget(twoViewFallbackChk_);
    twoViewRow->addWidget(new QLabel(QStringLiteral("2视角最小夹角(°)：")));
    twoViewMinRayAngleSpin_ = new QDoubleSpinBox;
    twoViewMinRayAngleSpin_->setRange(0.0, 60.0);
    twoViewMinRayAngleSpin_->setSingleStep(1.0);
    twoViewMinRayAngleSpin_->setDecimals(1);
    twoViewMinRayAngleSpin_->setValue(
        settings_.value(QStringLiteral("twoViewMinRayAngle"), 10.0).toDouble());
    twoViewMinRayAngleSpin_->setToolTip(QStringLiteral(
        "只有2个支持视角时额外要求的两条视线夹角下限。\n"
        "夹角小=深度方向病态=2视图幽灵的高发区（极线约束几乎不约束深度，\n"
        "随便两个观测都能\"匹配上\"），所以必须挡掉。\n"
        "调大更保守（更少幽灵、也更容易丢点），调小更激进。0=不检查。\n"
        "只影响2视角的点，3视角以上的点不受这个参数影响。"));
    twoViewRow->addWidget(twoViewMinRayAngleSpin_);
    twoViewRow->addWidget(new QLabel(QStringLiteral("最终残差上限：")));
    maxFinalResidualSpin_ = new QDoubleSpinBox;
    maxFinalResidualSpin_->setRange(0.0, 0.2);
    maxFinalResidualSpin_->setSingleStep(0.002);
    maxFinalResidualSpin_->setDecimals(4);
    maxFinalResidualSpin_->setValue(
        settings_.value(QStringLiteral("maxFinalResidual"), 0.01).toDouble());
    maxFinalResidualSpin_->setToolTip(QStringLiteral(
        "提交前对重解出来的点做的最后一道重投影残差检查（归一化单位，\n"
        "×焦距fx≈像素）。0=不检查。\n"
        "\n"
        "配合\"冲突后部分提交\"使用：当一个候选的部分支持观测被更强的候选\n"
        "抢走后，现在不再整体放弃（那是手指并拢时莫名少点的主因），而是\n"
        "用剩下的视角重新三角化——这道闸门负责确认重解出来的点仍然自洽。"));
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
    iekfDetectNoisePxSpin_->setToolTip(QStringLiteral(
        "你的检测层实际抖动大概多少像素，就填多少——IEKF内部按\n"
        "(噪声px / 相机焦距fx)² 换算成观测方差，告诉滤波器\"这次测量该信多少\"。\n"
        "只在没勾选\"马氏距离门控\"时真正生效(那种情况下每个观测有自己的真实\n"
        "协方差，这个值只在个别观测缺协方差时兜底)。\n"
        "旧版硬编码方差等价于σ≈2.85px，比多数系统的真实检测噪声(0.3~0.8px)粗\n"
        "了一个数量级——滤波器因此不敢信观测、状态滞后，这是实测里能找到的\n"
        "单项收益最大的参数。不确定就先填0.8，观察点云抖不抖/追不追得上再调：\n"
        "填小了(比真实噪声更小)→滤波器过度自信，抖动被误当成真实运动放大；\n"
        "填大了→滤波器不敢信新测量，响应慢、容易在快速运动下滞后丢点。"));
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
    batchWindowSpin_->setToolTip(QStringLiteral(
        "收到第一台相机的观测后，再等这么久，把这个窗口内所有相机的观测\n"
        "当成\"同一帧\"一起处理。太大：如果比相机真实帧间隔还长，会把两个\n"
        "不同时刻的真实帧误合并成一帧，拉低有效追踪帧率。太小：可能在\n"
        "还没等到全部相机的观测时就提前处理了，参与聚类的相机数变少，\n"
        "容易凑不够\"最少支持视角\"而被拒。往下看\"相机到达偏差实测\"这个\n"
        "数字，这个窗口至少要比它大，否则必然经常漏相机。"));
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
        "未开启绑定模式。开启后点云上的点可以点击——先点第一个点、再点第二个点连一条线。"
        "⚠ 连的是当前追踪会话的临时编号，不是永久身份：相机组合变了、或者某个点被遮挡\n"
        "太久重新分配了新编号，连线会跟着失效需要重连，不会持久化保存。\n"
        "视角：固定世界坐标系——左键拖拽旋转，右键/中键拖拽平移，滚轮缩放。点在\n"
        "空间中的绝对位置不变，画面就不会跟着自动缩放/居中乱跳。"), this);
    bindStatusLabel_->setWordWrap(true);
    v->addWidget(bindStatusLabel_);
}

} // namespace mocap
