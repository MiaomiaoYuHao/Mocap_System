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


// 【拆文件】本文件保留主构造与 PointCloudWidget 渲染实现；其余拆到：
//   PointCloudTestDialog_widgets.hpp    ElidingLabel / FlowLayout / setBriefText
//   PointCloudTestDialog_panel.cpp      buildParamSections()（原构造 632~2379 行）
//   PointCloudTestDialog_pipeline.cpp   析构 + onSkeletonResultReady + rebuild + 录制文件头
//   PointCloudTestDialog_capture.cpp    recordingDir ~ tryCluster（录制/采集/聚类）
namespace mocap {

// ---------------------------------------------------------------------------
// PointCloudWidget
// ---------------------------------------------------------------------------

PointCloudWidget::PointCloudWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(400, 400);
    setMouseTracking(false);
}

void PointCloudWidget::setPoints(const QVector<Point>& pts) {
    points_ = pts;
    update();
}

void PointCloudWidget::setEdges(const QVector<QPair<int,int>>& edges) {
    edges_ = edges;
    update();
}

void PointCloudWidget::setPointLabels(const QHash<int, QString>& labels) {
    pointLabels_ = labels;
    update();
}

void PointCloudWidget::setSkeletonLabels(const QHash<int, QString>& labels) {
    skelLabels_ = labels;
    update();
}

void PointCloudWidget::setSkeletonOverlay(const QVector<Point>& pts, const QVector<QPair<int,int>>& edges) {
    skeletonPoints_ = pts;
    skeletonEdges_ = edges;
    update();
}

void PointCloudWidget::resetView() {
    yawDeg_ = 30.0;
    pitchDeg_ = -25.0;
    scalePxPerMm_ = 0.6;
    panPx_ = QPointF(0, 0);
    update();
}

QPointF PointCloudWidget::worldToScreen(const QVector3D& p) const {
    // 固定世界坐标系的正交投影：先绕世界Z轴(up)做yaw水平旋转，再绕旋转后的
    // X轴做pitch俯仰——标准orbit相机变换，不依赖当前这一帧点云的分布，
    // 视角只由用户拖拽决定。跟旧版autofit的本质区别：世界原点(0,0,0)
    // 永远映射到 widgetCenter+panPx_ 这个固定屏幕位置(除非用户手动平移)，
    // 点在世界里绝对没动，屏幕上就绝对不会因为"别的点动了"而跟着挪。
    const double yaw = yawDeg_ * M_PI / 180.0;
    const double pitch = pitchDeg_ * M_PI / 180.0;
    const double x = double(p.x()), y = double(p.y()), z = double(p.z());

    const double x1 = x*std::cos(yaw) - y*std::sin(yaw);
    const double y1 = x*std::sin(yaw) + y*std::cos(yaw);
    const double z1 = z;

    const double y2 = y1*std::cos(pitch) - z1*std::sin(pitch);
    const double z2 = y1*std::sin(pitch) + z1*std::cos(pitch);
    const double x2 = x1;

    const QPointF widgetCenter(width()*0.5, height()*0.5);
    return widgetCenter + panPx_ + QPointF(x2*scalePxPerMm_, -z2*scalePxPerMm_);
}

void PointCloudWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(0x14, 0x14, 0x14));   // 跟宫格预览同一档暗底

    // 固定坐标系参照：世界原点+三条短轴线，即使点云是空的也画出来，
    // 让"这是个绝对坐标场景，不是围着点自动取景"这件事看得见摸得着。
    // 也顺带给旋转/缩放操作一个视觉锚点。
    {
        const QVector3D O(0,0,0);
        const float axisLen = float(50.0 / std::max(0.05, scalePxPerMm_ / 0.6)); // 视觉长度随缩放级别微调，但不随点云数据变
        const QPointF pO = worldToScreen(O);
        painter.setPen(QPen(QColor(180, 70, 70), 1.5));
        painter.drawLine(pO, worldToScreen(QVector3D(axisLen,0,0)));   // X 红
        painter.setPen(QPen(QColor(70, 180, 90), 1.5));
        painter.drawLine(pO, worldToScreen(QVector3D(0,axisLen,0)));   // Y 绿
        painter.setPen(QPen(QColor(70, 110, 200), 1.5));
        painter.drawLine(pO, worldToScreen(QVector3D(0,0,axisLen)));   // Z 蓝(up)
        painter.setPen(QColor(120,120,120));
        painter.drawEllipse(pO, 2.5, 2.5);
    }

    if (points_.isEmpty() && skeletonPoints_.isEmpty()) {
        // 【注意】判空要把骨架叠加层也算上。原来只判 points_ 就 return，
        // 结果是"点云为空但AI已经预测出骨架"时，下面的骨架绘制代码根本
        // 轮不到执行(整只手全被遮挡、只靠回归预测时就是这种情况)。
        painter.setPen(QColor(140, 140, 140));
        painter.drawText(rect(), Qt::AlignCenter, QStringLiteral("(暂无点——检查相机勾选/检测是否开启)"));
        return;
    }

    QFont idFont = painter.font(); idFont.setPointSize(8);
    painter.setFont(idFont);

    // 先记下这一帧每个id的屏幕坐标——连线渲染和点击命中检测都要用，
    // 只算一次，保证两者用的是同一份坐标，不会因为分别计算而错位。
    lastScreenPosById_.clear();
    for (const auto& p : points_)
        lastScreenPosById_[p.id] = worldToScreen(p.pos);

    // 手动连线——画在点下面，不挡住点本身和编号文字。哪个端点这一帧
    // 不在points_里(比如id已经被销毁重分配)就跳过这条边，不强行连一条
    // 找不到端点的线。
    painter.setPen(QPen(QColor(90, 200, 140), 2.0));
    for (const auto& e : edges_) {
        const auto itA = lastScreenPosById_.constFind(e.first);
        const auto itB = lastScreenPosById_.constFind(e.second);
        if (itA == lastScreenPosById_.constEnd() || itB == lastScreenPosById_.constEnd()) continue;
        painter.drawLine(itA.value(), itB.value());
    }

    for (const auto& p : points_) {
        const QPointF screenPt = lastScreenPosById_.value(p.id);
        // 遮挡记忆(coasting)中的点用暗色空心圆，实测的用亮色实心圆——
        // 跟 candidatePointsReady 的 missedFrames 语义一致(见
        // HandTrackingWorker.hpp 里那段注释)，一眼分清"这是刚测到的"还是
        // "已经看不见、用的是最后一次的旧位置"。
        const bool claimed = pointLabels_.contains(p.id);
        if (p.coasting) {
            painter.setPen(QPen(QColor(140,140,90), 1.5));
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(screenPt, 5, 5);
        } else if (!claimed) {
            // 没被骨架认领：灰色实心，弱化但仍然可见 ——
            // 完全不画的话就看不出"这里还有个点没被用上"了。
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(120, 120, 120));
            painter.drawEllipse(screenPt, 4, 4);
        } else {
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor(230, 200, 90));
            painter.drawEllipse(screenPt, 5, 5);
        }
        // 【标注语义标签，不是追踪序号】序号只在调试点云本身时有用；
        // 看骨架对不对的时候，"这个点被判成了 index.dp 还是 mid.dp"才是信息。
        // 没有标签的点（没被骨架认领的杂点）回退显示序号，否则它们会变成
        // 一片没有任何标识的圆点，反而不好排查。
        // 【只标语义，不标追踪序号】序号只在调试点云本身时有用；看骨架对不对
        // 的时候它是纯噪声，20 个数字铺在屏幕上反而盖住了真正要看的东西。
        // 没被骨架认领的点连标签都不画 —— 它已经用暗色圆点表达"没被认领"了，
        // 再挂个数字只是把视线拉过去。
        const auto itLbl = pointLabels_.constFind(p.id);
        if (itLbl != pointLabels_.constEnd()) {
            painter.setPen(QColor(120, 230, 120));      // 绿色，跟点云的黄、骨架的橙分开
            painter.drawText(screenPt + QPointF(7, -7), itLbl.value());
        }
    }

    // ---- AI骨架叠加层：完全独立的一遍绘制，不进lastScreenPosById_(不
    // 参与点击命中检测/手动绑定)，也不受上面点云刷新影响——skeletonPoints_/
    // skeletonEdges_只有 setSkeletonOverlay() 会改，跟points_/edges_各自
    // 独立更新，这就是"AI画线"和"点云显示"互不冲突的关键。用单独的临时
    // map算屏幕坐标，不复用/污染lastScreenPosById_。----
    if (!skeletonPoints_.isEmpty()) {
        QHash<int, QPointF> skelScreenPos;
        skelScreenPos.reserve(skeletonPoints_.size());
        for (const auto& p : skeletonPoints_)
            skelScreenPos[p.id] = worldToScreen(p.pos);

        painter.setPen(QPen(QColor(230, 150, 60), 2.0));   // 橙色，跟手动绑定线(绿色)区分开
        for (const auto& e : skeletonEdges_) {
            const auto itA = skelScreenPos.constFind(e.first);
            const auto itB = skelScreenPos.constFind(e.second);
            if (itA == skelScreenPos.constEnd() || itB == skelScreenPos.constEnd()) continue;
            painter.drawLine(itA.value(), itB.value());
        }

        for (const auto& p : skeletonPoints_) {
            const QPointF screenPt = skelScreenPos.value(p.id);
            if (p.predicted) {
                // 遮挡关节的AI预测位置——蓝色空心方块，不是真实观测。
                painter.setPen(QPen(QColor(90,150,220), 1.5));
                painter.setBrush(Qt::NoBrush);
                painter.drawRect(QRectF(screenPt.x()-4, screenPt.y()-4, 8, 8));
                // 预测点也要标签：它在点云层没有对应点，不在这里标就永远没有。
                // 用偏蓝的绿区分"这个标签属于补出来的点"，免得跟实测点混淆。
                const auto it = skelLabels_.constFind(p.id);
                if (it != skelLabels_.constEnd()) {
                    painter.setPen(QColor(110, 200, 180));
                    painter.drawText(screenPt + QPointF(7, -7), it.value());
                }
            } else {
                // 真实观测到、被AI认领的关节——橙色实心圆，跟点云本身的
                // 黄色实心圆区分开，一眼能看出"这是骨架点不是普通点云点"。
                painter.setPen(Qt::NoPen);
                painter.setBrush(QColor(230, 150, 60));
                painter.drawEllipse(screenPt, 4, 4);
            }
        }
    }
}

void PointCloudWidget::mousePressEvent(QMouseEvent* ev) {
    lastMousePos_ = ev->pos();
    if (ev->button() == Qt::LeftButton) {
        rotating_ = true;
        return;   // 左键拖拽=旋转视角；点击(未拖动)命中检测放在release里判断，
                  // 避免"拖了一下但没怎么移动"被误判成点击。
    }
    if (ev->button() == Qt::RightButton || ev->button() == Qt::MiddleButton) {
        panning_ = true;
        return;
    }
}

void PointCloudWidget::mouseMoveEvent(QMouseEvent* ev) {
    const QPoint delta = ev->pos() - lastMousePos_;
    lastMousePos_ = ev->pos();
    if (rotating_) {
        yawDeg_ += delta.x() * 0.4;
        pitchDeg_ = std::clamp(pitchDeg_ - delta.y() * 0.4, -89.0, 89.0);
        update();
    } else if (panning_) {
        panPx_ += QPointF(delta);
        update();
    }
}

void PointCloudWidget::mouseReleaseEvent(QMouseEvent* ev) {
    if (ev->button() == Qt::LeftButton) {
        rotating_ = false;
        // 命中判定：用按下时记录、release时的位置做最近点+命中半径判定，
        // 是否属于"拖拽"还是"点击"由调用方(绑定模式)自己承受一点误差——
        // 14px的命中半径本来就比正常手指拖拽产生的漂移大，实际使用中
        // 拖着转视角时不会精确落在某个点的命中圈内，冲突概率很低。
        if (!lastScreenPosById_.isEmpty()) {
            const QPointF click = ev->position();
            int bestId = -1; double bestDist = 1e18;
            for (auto it = lastScreenPosById_.constBegin(); it != lastScreenPosById_.constEnd(); ++it) {
                const double dx = it.value().x()-click.x(), dy = it.value().y()-click.y();
                const double d = std::sqrt(dx*dx+dy*dy);
                if (d < bestDist) { bestDist = d; bestId = it.key(); }
            }
            if (bestId >= 0 && bestDist <= 14.0) emit pointClicked(bestId);
        }
    }
    if (ev->button() == Qt::RightButton || ev->button() == Qt::MiddleButton) panning_ = false;
}

void PointCloudWidget::wheelEvent(QWheelEvent* ev) {
    // 滚轮缩放：以当前缩放系数为基准做指数缩放(每120 delta约±10%)，
    // 比线性增减在大范围缩放时手感更均匀。夹一个合理范围防止缩到0或
    // 缩到离谱大导致鼠标交互失灵。
    const double factor = std::pow(1.1, ev->angleDelta().y() / 120.0);
    scalePxPerMm_ = std::clamp(scalePxPerMm_ * factor, 0.02, 20.0);
    update();
}

// ---------------------------------------------------------------------------
// PointCloudTestDialog
// ---------------------------------------------------------------------------


PointCloudTestDialog::PointCloudTestDialog(CameraManager* mgr, CalibrationStore* store,
                                           bool wasDetectOn, QWidget* parent)
    : QDialog(parent), mgr_(mgr), store_(store), wasDetectOn_(wasDetectOn) {
    // 【改名】这个面板早就不是"测试"了 —— 多视角聚类 → IEKF 追踪 → 骨架
    // 关联 → M3DS/M3DQ 推给 Unity，整条产线都在这里，它就是正式的运行台。
    // 类名和文件名保持 PointCloudTestDialog 不动（改了要连带改构建脚本），
    // 只换对外显示的名字。
    setWindowTitle(QStringLiteral("实时动捕（多视角聚类 · IEKF · 骨架输出）"));

    // 【要像一个普通 Windows 窗口，而不是一个对话框】
    // QDialog 默认带 Qt::Dialog 类型，Windows 于是把它当"对话框"处理：
    //   · 不给最大化按钮；
    //   · Aero Snap（拖到屏幕左/右边缘自动贴半屏、拖到顶边最大化）不生效；
    //   · 配合"被 MainWindow 拥有"的关系，它永远压在主窗口之上，两个窗口没法
    //     靠点任务栏 / 点窗口来正常互相切前后。
    // 换成纯 Qt::Window + 系统菜单 + 最小化/最大化/关闭按钮之后，它就跟记事本
    // 一样是普通顶层窗口：有独立的 z 序、独立的任务栏按钮、能贴边分屏。
    //
    // 【必须在 restoreGeometry() 之前设】改窗口标志会让 Qt 重建原生窗口，
    // 那一步会把窗口位置/尺寸重置掉；设在恢复几何之前，存下来的位置才生效。
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowSystemMenuHint |
                   Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint |
                   Qt::WindowCloseButtonHint);

    // ------------------------------------------------------------------
    // 【布局重构·第二版】上一版是"上方可滚动参数区 + 下方 3D 视图"的纵向
    // 分栏。两个毛病没解决：
    //   ① 参数是横着长的 —— 光"追踪后端"那一条 QHBoxLayout 就串了三十多个
    //      控件，窗口再宽也会顶出横向滚动条，找一个开关得左右拖着找；
    //   ② 点云视图被压在下半屏，而它才是这个窗口存在的理由。参数是拿来
    //      调的，点云是拿来看的，看的东西不该给调的东西让位。
    //
    // 现在换成编辑器 + 右侧栏的分法：
    //   ┌───────────────────────────────┬──────────┐
    //   │  点云视图（主角）              │ 参数侧栏  │
    //   │  ├ 输出监视面板（可拖宽）       │ 按主题分节│
    //   ├───────────────────────────────┴──────────┤
    //   │ 状态条：点数 / 稳定性 / 耗时 / 滞后（等宽）  │
    //   └──────────────────────────────────────────┘
    //
    // 三件事一起做的：参数按主题拆成【可折叠小节】（一次只展开在调的那一
    // 节，其余收起来，纵向长度立刻从两千像素掉到几百）；控件【纵向排】而不
    // 是横向串（侧栏定宽，永远不会再顶出横向滚动条）；读数类标签全部沉到
    // 【底部状态条】，跟点云同屏，不再占参数区的位置也不用滚动去找。
    //
    // 小节的展开/收起状态按 key 存进 QSettings —— 你上次在调哪一节，下次
    // 打开还是展开着的。
    // ------------------------------------------------------------------
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    // 顶部标签条：做成编辑器标签页的样子，右侧放两个布局按钮
    auto* topBarHost = new QWidget;
    topBarHost->setObjectName(QStringLiteral("panelTabBar"));
    topBarHost->setAttribute(Qt::WA_StyledBackground, true);
    auto* topBar = new QHBoxLayout(topBarHost);
    topBar->setContentsMargins(0, 0, 8, 0);
    topBar->setSpacing(4);
    auto* tabLabel = new QLabel(QStringLiteral("实时动捕"));
    tabLabel->setObjectName(QStringLiteral("panelTab"));
    topBar->addWidget(tabLabel);
    topBar->addStretch(1);
    auto* collapseBtn = new QToolButton;
    collapseBtn->setCheckable(true);
    collapseBtn->setText(QStringLiteral("隐藏参数栏"));
    collapseBtn->setToolTip(QStringLiteral(
        "把右侧参数栏整个收起来，把全部空间让给 3D 点云视图。\n"
        "再点一次展开。也可以直接拖动中间的分隔条自由分配。"));
    topBar->addWidget(collapseBtn);
    auto* resetLayoutBtn = new QToolButton;
    resetLayoutBtn->setText(QStringLiteral("重置布局"));
    resetLayoutBtn->setToolTip(QStringLiteral("恢复默认窗口大小和分隔条位置。"));
    topBar->addWidget(resetLayoutBtn);
    root->addWidget(topBarHost);

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->setChildrenCollapsible(true);
    splitter->setHandleWidth(1);
    root->addWidget(splitter, 1);

    // 底部状态条：所有"读数"类标签的归宿。等宽字体，跟主窗口状态栏同一种
    // 语言 —— 一眼扫过去就知道现在跑得怎么样，不用去参数堆里翻。
    auto* statusHost = new QWidget;
    statusHost->setObjectName(QStringLiteral("panelStatusBar"));
    statusHost->setAttribute(Qt::WA_StyledBackground, true);
    auto* statusStrip = new QHBoxLayout(statusHost);
    statusStrip->setContentsMargins(10, 3, 10, 3);
    // 【间距收紧到 8】原来 14 是为了让相邻读数不糊在一起，但那是用留白
    // 代替分隔线的做法——留白一多，整条就散，四组读数看起来像四段互不相干
    // 的话。现在改用 1px 竖线分组（下面 addSep），间距就可以收回来。
    statusStrip->setSpacing(8);
    root->addWidget(statusHost);


    // ---- 参数侧栏容器 ----
    auto* ctlHost = new QWidget;
    ctlHost->setObjectName(QStringLiteral("panelSideBar"));
    ctlHost->setAttribute(Qt::WA_StyledBackground, true);
    auto* ctlCol = new QVBoxLayout(ctlHost);
    ctlCol->setContentsMargins(0, 0, 0, 12);
    ctlCol->setSpacing(0);

    // ---- 顶部常用操作（不随小节折叠，永远可见）----
    // 「手部复位」和「重标」是运行中要反复点的两个动作，原来分别埋在
    // 「骨架关联(AI)」小节里 —— 每次都得先展开小节、滚下去、点完再收起来。
    // 挑出来钉在侧栏顶部；按钮本身还是同两个对象、同两个槽，只是换了位置。
    auto* quickHost = new QWidget;
    quickHost->setObjectName(QStringLiteral("panelQuickBar"));
    quickHost->setAttribute(Qt::WA_StyledBackground, true);
    auto* quickRow = new QHBoxLayout(quickHost);
    quickRow->setContentsMargins(8, 6, 8, 6);
    quickRow->setSpacing(6);
    ctlCol->addWidget(quickHost);

    // 【拆文件】“参数侧栏分节填充”已抽成私有方法（PointCloudTestDialog_panel.cpp）。
    buildParamSections(ctlCol, quickRow, statusStrip);

    ctlCol->addStretch(1);   // 小节全部收起时把空白顶到下面，标题条不会散开

    // ---- 组装 splitter：左=3D视图（主角），右=参数侧栏 ----
    auto* ctlScroll = new QScrollArea;
    ctlScroll->setWidget(ctlHost);
    ctlScroll->setWidgetResizable(true);
    ctlScroll->setFrameShape(QFrame::NoFrame);
    // 侧栏定宽 + 控件纵向排 = 永远不需要横向滚动条。这是这次重构最实在的
    // 一条收益：以前那条三十多个控件的长龙，窗口开多大都得左右拖。
    ctlScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    ctlScroll->setMinimumWidth(0);            // 允许被完全折叠
    ctlHost->setMinimumWidth(330);

    view_ = new PointCloudWidget;
    view_->setMinimumHeight(160);
    view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    connect(view_, &PointCloudWidget::pointClicked, this, &PointCloudTestDialog::onPointClicked);
    connect(resetViewBtn_, &QPushButton::clicked, view_, &PointCloudWidget::resetView);

    // 【新增】3D视图 + 输出数据面板 横向并排，中间可拖分隔条。
    // 【为什么不是塞进上面那个参数区】参数区是"往里输入"，这块是"往外读数"，
    // 而且要跟点云同时盯着看（点云在抖 vs 输出在跳，是不是同一件事，一眼就能
    // 判断）。塞进能滚动的参数区里就得来回滚，那还不如没有。
    // 【数据栏改到底部】原来它是点云右边的一根竖条，两个后果：点云被挤成
    // 窄高的一条（3D 场景最不该是这个形状），而数据栏自己又是窄高的，16 个
    // 关节只能排成一列、下面空一大片。
    //
    // 改成横在点云下方（就是 VS Code 里终端面板的位置）之后：点云吃满整个
    // 宽度、变成正常的横向视野；数据栏变宽扁，关节表自动切双列（见
    // HandOutputView::paintJoints），高度减半、宽度吃满，那块浪费没了。
    auto* viewRow = new QSplitter(Qt::Vertical, this);
    viewRow->setChildrenCollapsible(true);
    viewRow->setHandleWidth(1);
    viewRow->addWidget(view_);

    outputMonitor_ = new HandOutputMonitor(this);
    outputMonitor_->setVisible(showOutputChk_->isChecked());
    outputMonitor_->setActive(showOutputChk_->isChecked());
    viewRow->addWidget(outputMonitor_);
    viewRow->setStretchFactor(0, 1);   // 拉窗口时空间优先给点云，数据栏保持高度
    viewRow->setStretchFactor(1, 0);
    splitter->addWidget(viewRow);       // index 0 = 视图
    splitter->addWidget(ctlScroll);     // index 1 = 参数侧栏

    // 【关键接线】接的是本对话框自己的两个 signal —— MainWindow 那头把同样这
    // 两个 signal 接到了 UdpSender。同一次 emit、两个消费者，所以面板上打印
    // 的数值和 UDP 包里的字节必然一致，不存在"显示的和实际发的是两份代码"
    // 这种排查时最误导人的情况。
    connect(this, &PointCloudTestDialog::handPoseForUdp,
            outputMonitor_, &HandOutputMonitor::onHandPose);
    connect(this, &PointCloudTestDialog::bendRefForUi,
            outputMonitor_, &HandOutputMonitor::onBendRef);
    connect(this, &PointCloudTestDialog::segmentQuatsForUdp,
            outputMonitor_, &HandOutputMonitor::onSegmentQuats);
    // 解算原始角（未经 ROM 映射/限幅）只给面板做对照，不进 UDP。
    // worker 在自己的线程里 emit，必须排队投递。
    if (skeletonWorker_) {
        connect(skeletonWorker_, &SkeletonAssocWorker::jointAnglesDebug,
                outputMonitor_, &HandOutputMonitor::onJointAnglesDebug, Qt::QueuedConnection);
    }
    // ---- 输出链路延迟：三个控件的接线 ----
    // 【为什么放在这里而不是控件创建处】这几个 lambda 要用 skeletonWorker_ /
    // skeletonThread_，而它们是在控件之后才建的。跟 showOutputChk_ 同一个理由。
    auto applyDispatchRate = [this] {
        static const int kMs[4] = {33, 16, 8, 0};   // 30 / 60 / 120 / 不限流
        const int i = std::clamp(skelRateCombo_->currentIndex(), 0, 3);
        skelDispatchMinMs_ = lowLatencyChk_->isChecked() ? 0 : kMs[i];
    };
    auto applyLowLatency = [this, applyDispatchRate] {
        const bool on = lowLatencyChk_->isChecked();
        applyDispatchRate();
        if (skeletonWorker_) {
            // 【别改控件的勾选状态】直接命令 worker，控件只是变灰。改控件会触发
            // 它们自己的 connect 把 QSettings 覆盖掉 —— 用户退出直通后设置就没了。
            const int si = smoothCombo_->currentIndex();
            skeletonWorker_->setFilterEnabled(!on && si != 0);
            skeletonWorker_->setRateLimitEnabled(!on && rateLimitChk_->isChecked());
        }
        if (skeletonThread_)
            skeletonThread_->setPriority(on ? QThread::NormalPriority : QThread::LowestPriority);
        // 【不再置灰】上一版把这几个控件 setEnabled(false)，结果是用户看到"平滑"
        // 变灰、以为功能坏了 —— 一个被压住又不解释为什么的控件，比没有更糟。
        // 现在改成：控件一直可用，谁被碰了就自动退出直通（见下面各自的 connect）。
        settings_.setValue(QStringLiteral("skelLowLatency"), on);
        if (latencyLabel_ && on)
            latencyLabel_->setText(QStringLiteral("直通中：平滑/限流/限幅已旁路"));
    };
    connect(lowLatencyChk_, &QCheckBox::toggled, this, [applyLowLatency](bool){ applyLowLatency(); });
    connect(skelRateCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this, applyDispatchRate](int i) {
        settings_.setValue(QStringLiteral("skelDispatchRate"), i);
        if (lowLatencyChk_->isChecked()) { lowLatencyChk_->setChecked(false); return; }
        applyDispatchRate();
    });
    // 碰任何一个分项设置 = 用户要手动控制了，自动退出直通。
    // 这样"改了平滑没反应"这种事不会再发生。
    auto releaseLowLatency = [this] {
        if (lowLatencyChk_ && lowLatencyChk_->isChecked()) lowLatencyChk_->setChecked(false);
    };
    connect(smoothCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [releaseLowLatency](int){ releaseLowLatency(); });
    connect(useOutputFilterChk_, &QCheckBox::toggled, this,
            [releaseLowLatency](bool){ releaseLowLatency(); });

    connect(geoRelabelChk_, &QCheckBox::toggled, this, [this](bool on) {
        settings_.setValue(QStringLiteral("dorsumGeoRelabel"), on);
        if (skeletonWorker_) skeletonWorker_->setDorsumGeoRelabel(on);
    });
    if (skeletonWorker_) skeletonWorker_->setDorsumGeoRelabel(geoRelabelChk_->isChecked());

    connect(ikOnlyChk_, &QCheckBox::toggled, this, [this](bool on) {
        settings_.setValue(QStringLiteral("occludedIkOnly"), on);
        if (skeletonWorker_) skeletonWorker_->setOccludedIkOnly(on);
    });
    if (skeletonWorker_) skeletonWorker_->setOccludedIkOnly(ikOnlyChk_->isChecked());

    connect(thumbPronChk_, &QCheckBox::toggled, this, [this](bool on) {
        settings_.setValue(QStringLiteral("thumbPronOn"), on);
        if (skeletonWorker_) skeletonWorker_->setThumbPronationOn(on);
    });
    if (skeletonWorker_) skeletonWorker_->setThumbPronationOn(thumbPronChk_->isChecked());

    connect(thumbRollSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this](double deg) {
        settings_.setValue(QStringLiteral("thumbRollDeg"), deg);
        if (skeletonWorker_) skeletonWorker_->setThumbRollOffset(deg * M_PI / 180.0);
    });
    if (skeletonWorker_)
        skeletonWorker_->setThumbRollOffset(thumbRollSpin_->value() * M_PI / 180.0);

    connect(rateLimitChk_, &QCheckBox::toggled, this, [this](bool on) {
        settings_.setValue(QStringLiteral("skelRateLimit"), on);
        if (lowLatencyChk_->isChecked()) { lowLatencyChk_->setChecked(false); return; }
        if (skeletonWorker_) skeletonWorker_->setRateLimitEnabled(on);
    });
    applyLowLatency();   // 初始状态一次性下发（含从 QSettings 读回的直通状态）

    connect(showOutputChk_, &QCheckBox::toggled, this, [this, viewRow](bool on) {
        settings_.setValue(QStringLiteral("showOutputMonitor"), on);
        outputMonitor_->setVisible(on);
        outputMonitor_->setActive(on);     // 关掉时槽函数直接 return，成本归零
        if (on && viewRow->sizes().value(1) < 40) {
            const int total = viewRow->sizes().value(0) + viewRow->sizes().value(1);
            viewRow->setSizes({std::max(240, total - 340), 340});   // 现在分的是高度
        }
    });

    // 拉窗口时多出来的空间全给视图，侧栏保持宽度（索引已换：0=视图 1=侧栏）
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 0);

    // 折叠按钮：记住折叠前的分配，展开时还原。侧栏现在是 index 1。
    auto lastSizes = std::make_shared<QList<int>>();   // 用shared_ptr随lambda生命周期走，不裸new
    connect(collapseBtn, &QToolButton::toggled, this, [splitter, collapseBtn, lastSizes](bool on){
        if (on) {
            *lastSizes = splitter->sizes();
            const int total = lastSizes->value(0) + lastSizes->value(1);
            splitter->setSizes({total, 0});
            collapseBtn->setText(QStringLiteral("显示参数栏"));
        } else {
            if (lastSizes->size() == 2 && lastSizes->value(1) > 0) splitter->setSizes(*lastSizes);
            else splitter->setSizes({900, 400});
            collapseBtn->setText(QStringLiteral("隐藏参数栏"));
        }
    });

    // 默认尺寸：视图占大头。参数改成侧栏之后需要的是宽度不是高度，所以
    // 比上一版更宽、略矮。
    QSize defSize(1460, 900);
    if (auto* scr = QGuiApplication::primaryScreen()) {
        const QSize avail = scr->availableGeometry().size();
        defSize.setWidth(std::min(defSize.width(), avail.width() - 40));
        defSize.setHeight(std::min(defSize.height(), avail.height() - 60));
    }
    setSizeGripEnabled(true);
    setMinimumSize(640, 420);

    // 恢复上次的窗口几何 / 分隔条位置
    const QByteArray savedGeom = settings_.value(QStringLiteral("dlgGeometry")).toByteArray();
    if (!savedGeom.isEmpty()) restoreGeometry(savedGeom);
    else resize(defSize);
    // 【换了 key】上一版这条分隔条是纵向的，存的状态按高度算；直接 restore
    // 到现在这条横向分隔条上会得到一个荒唐的宽度分配（侧栏 300px 高的状态
    // 被当成 300px 宽用还算凑合，真正的问题是老用户会拿到一个没调过的布局
    // 却以为是自己调的）。换个 key，从默认值重新开始。
    const QByteArray savedSplit = settings_.value(QStringLiteral("dlgSplitterMain2")).toByteArray();
    if (!savedSplit.isEmpty()) splitter->restoreState(savedSplit);
    else splitter->setSizes({defSize.width() - 380, 380});
    // 横向那条（点云 | 输出面板）单独存一份 key，跟上面那条互不干扰
    // 换 key：这条分隔条从横向变成纵向了，旧状态存的是宽度分配，restore 过来
    // 会得到一个莫名其妙的高度。
    const QByteArray savedSplitH = settings_.value(QStringLiteral("dlgSplitterPanelV")).toByteArray();
    if (!savedSplitH.isEmpty()) viewRow->restoreState(savedSplitH);
    // 左窗格现在只有 defSize.width()-400（右边让给了参数栏），默认值得跟着收
    else viewRow->setSizes({defSize.height() - 340, 340});

    connect(resetLayoutBtn, &QToolButton::clicked, this,
            [this, splitter, defSize, collapseBtn]{
        collapseBtn->setChecked(false);
        resize(defSize);
        splitter->setSizes({defSize.width() - 380, 380});
    });

    // 关闭时把布局记下来（QDialog::finished 在 accept/reject/关闭时都会发）
    connect(this, &QDialog::finished, this, [this, splitter, viewRow](int){
        settings_.setValue(QStringLiteral("dlgGeometry"), saveGeometry());
        settings_.setValue(QStringLiteral("dlgSplitterMain2"), splitter->saveState());
        settings_.setValue(QStringLiteral("dlgSplitterPanelV"), viewRow->saveState());
    });

    batchTimer_ = new QTimer(this);
    batchTimer_->setSingleShot(true);
    connect(batchTimer_, &QTimer::timeout, this, &PointCloudTestDialog::onBatchTimer);

    connect(list_, &QListWidget::itemChanged, this, [this](QListWidgetItem*) { rebuild(); });

    // 【新增】检测算法切换/半径先验改了——存档 + 完整rebuild()。这两个
    // 参数(尤其算法切换)牵扯到"订阅哪个信号、开不开轮廓采集"，不是简单
    // 调个setter能带过的，用rebuild()是对的，代价是编号会清空重来——
    // 切换检测算法本来就该被当成"这是一次全新的追踪会话"，两种算法测出
    // 来的编号本来就不是同一码事，不需要强行保留跨算法的编号连续性。
    connect(useCircleFitChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("useCircleFit"), v);
        rebuild();
    });
    connect(circleRadiusPxSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("circleRadiusPx"), v);
        if (useCircleFitChk_->isChecked()) rebuild();   // 质心法模式下半径先验不生效，改了不用重建
    });
    connect(useMahalanobisChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("useMahalanobis"), v);
        // 不需要rebuild()——tryCluster()每次都从这个checkbox实时读值，
        // 只是决定要不要把已经在算的协方差传给clusterMultiView，跟
        // maxSampson/maxReprojNorm一样是"纯读取spinbox当前值"的路径。
    });

    // 【记忆机制】任何一个参数改了：写回QSettings + 重建TemporalTracker
    // (聚类阈值不需要重建tracker，但为了逻辑简单统一走rebuild()，代价是
    // 聚类阈值变了也会顺带清一次跨帧编号——这是可以接受的：调聚类阈值
    // 这种事本来就该有掉重来的心理预期，不然"稳不稳"这个问题本身就没法
    // 干净地判断)。
    // 【这是"点云很稳、编号还是频繁增长"那个问题的修复】之前不管改哪个
    // 参数都调 rebuild()，rebuild() 会把 tracker_ 整个重新构造，代价是
    // 已确认的全部编号被清空重来——调参这个动作本身在制造"编号在增长"
    // 的假象，追踪本身可能一直很稳。现在分两条独立的路径：
    //   ① 聚类阈值(maxSampson/maxReprojNorm)：只需要存档，完全不用碰
    //      tracker_——tryCluster() 每次都是从 spinbox 实时读当前值，
    //      改这两个数不需要重建任何东西，之前调 rebuild() 纯属多余。
    //   ② 追踪参数(assocDist/maxMissed/minHits)：用刚加的运行时 setter，
    //      只改参数、不清空已有轨迹和编号——已经在追踪的点不会因为你
    //      调这几个数就被迫重新分配编号。
    // 真正需要完整重建(相机组合变了，本来就该从零开始)的路径还是走
    // rebuild()，只是不再被参数改动误触发。
    auto onClusterParamChanged = [this] {
        settings_.setValue(QStringLiteral("maxSampson"), maxSampsonSpin_->value());
        settings_.setValue(QStringLiteral("maxReprojNorm"), maxReprojSpin_->value());
        settings_.setValue(QStringLiteral("minSupportViews"), minSupportSpin_->value());
        settings_.setValue(QStringLiteral("useVoting"), useVotingChk_->isChecked());
        settings_.setValue(QStringLiteral("minRayAngleDeg"), minRayAngleSpin_->value());
        settings_.setValue(QStringLiteral("ambiguityMargin"), ambiguityMarginSpin_->value());
        settings_.setValue(QStringLiteral("useLmRefine"), useLmRefineChk_->isChecked());
        settings_.setValue(QStringLiteral("clusterChiSquare"), clusterChiSquareSpin_->value());
        settings_.setValue(QStringLiteral("lmHuberDelta"), lmHuberDeltaSpin_->value());
        settings_.setValue(QStringLiteral("lmMaxIters"), lmMaxItersSpin_->value());
        settings_.setValue(QStringLiteral("calibSigmaNorm"), calibSigmaNormSpin_->value());
        settings_.setValue(QStringLiteral("iekfDesyncComp"), iekfDesyncCompChk_->isChecked());
        settings_.setValue(QStringLiteral("iekfExposureMs"), iekfExposureMsSpin_->value());
        settings_.setValue(QStringLiteral("iekfExposureAnchor"), iekfExposureAnchorCombo_->currentIndex());
        settings_.setValue(QStringLiteral("twoViewFallback"), twoViewFallbackChk_->isChecked());
        settings_.setValue(QStringLiteral("twoViewMinRayAngle"), twoViewMinRayAngleSpin_->value());
        settings_.setValue(QStringLiteral("maxFinalResidual"), maxFinalResidualSpin_->value());
    };
    auto onTrackerParamChanged = [this] {
        settings_.setValue(QStringLiteral("assocDistMm"), assocDistSpin_->value());
        settings_.setValue(QStringLiteral("maxMissedFrames"), maxMissedSpin_->value());
        settings_.setValue(QStringLiteral("minHitsToConfirm"), minHitsSpin_->value());
        settings_.setValue(QStringLiteral("useHungarian"), useHungarianChk_->isChecked());
        settings_.setValue(QStringLiteral("useAdaptiveCap"), useAdaptiveCapChk_->isChecked());
        settings_.setValue(QStringLiteral("adaptiveCapMultiplier"), adaptiveCapMulSpin_->value());
        settings_.setValue(QStringLiteral("useMissedRelax"), useMissedRelaxChk_->isChecked());
        settings_.setValue(QStringLiteral("relaxGrowthPerMissedFrame"), relaxGrowthSpin_->value());
        settings_.setValue(QStringLiteral("relaxCapMultiplier"), relaxCapSpin_->value());
        settings_.setValue(QStringLiteral("useVelSmooth"), useVelSmoothChk_->isChecked());
        settings_.setValue(QStringLiteral("velSmoothAlpha"), velSmoothAlphaSpin_->value());
        settings_.setValue(QStringLiteral("useConstAccel"), useConstAccelChk_->isChecked());
        tracker_.setMaxAssocDist(assocDistSpin_->value());
        tracker_.setMaxMissedFrames(maxMissedSpin_->value());
        tracker_.setMinHitsToConfirm(minHitsSpin_->value());
        tracker_.setUseOptimalAssignment(useHungarianChk_->isChecked());
        tracker_.setUseAdaptiveAssocCap(useAdaptiveCapChk_->isChecked());
        tracker_.setAdaptiveCapMultiplier(adaptiveCapMulSpin_->value());
        tracker_.setUseMissedFrameRelax(useMissedRelaxChk_->isChecked());
        tracker_.setRelaxGrowthPerMissedFrame(relaxGrowthSpin_->value());
        tracker_.setRelaxCapMultiplier(relaxCapSpin_->value());
        tracker_.setUseVelocitySmoothing(useVelSmoothChk_->isChecked());
        tracker_.setVelocitySmoothingAlpha(velSmoothAlphaSpin_->value());
        tracker_.setUseConstantAcceleration(useConstAccelChk_->isChecked());
        // iekfTracker_的关联/生命周期参数(assocDist/maxMissed/minHits)
        // 跟tracker_共用同一批控件——两个后端切换时应该看到同一套数字，
        // 不需要为IEKF后端单独维护一份assocDist等参数的UI。
        iekfTracker_.setMaxAssocDist(assocDistSpin_->value());
        iekfTracker_.setMaxMissedFrames(maxMissedSpin_->value());
        iekfTracker_.setMinHitsToConfirm(minHitsSpin_->value());
    };
    connect(maxSampsonSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(maxReprojSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(minSupportSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [onClusterParamChanged](int){ onClusterParamChanged(); });
    connect(useVotingChk_, &QCheckBox::toggled, this, [onClusterParamChanged](bool){ onClusterParamChanged(); });
    connect(minRayAngleSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(ambiguityMarginSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(useLmRefineChk_, &QCheckBox::toggled, this, [onClusterParamChanged](bool){ onClusterParamChanged(); });
    connect(clusterChiSquareSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(lmHuberDeltaSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(lmMaxItersSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [onClusterParamChanged](int){ onClusterParamChanged(); });
    connect(calibSigmaNormSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(twoViewFallbackChk_, &QCheckBox::toggled, this, [onClusterParamChanged](bool){ onClusterParamChanged(); });
    connect(twoViewMinRayAngleSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(maxFinalResidualSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    // 【新增】观测时刻补偿的三个控件也接上重算触发，跟其它聚类参数一致——
    // 否则改了这三个要手动再触发一次才生效，交互上不一致。
    connect(iekfDesyncCompChk_, &QCheckBox::toggled, this, [onClusterParamChanged](bool){ onClusterParamChanged(); });
    connect(iekfExposureMsSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onClusterParamChanged](double){ onClusterParamChanged(); });
    connect(iekfExposureAnchorCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [onClusterParamChanged](int){ onClusterParamChanged(); });
    connect(assocDistSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onTrackerParamChanged](double){ onTrackerParamChanged(); });
    connect(maxMissedSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [onTrackerParamChanged](int){ onTrackerParamChanged(); });
    connect(minHitsSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [onTrackerParamChanged](int){ onTrackerParamChanged(); });
    // 【算法开关，点了立即生效】跟上面几个追踪参数走同一条路径——只调
    // 运行时setter，不碰tracker_的构造/析构，已有编号不受影响。这正是
    // 用户要求的"点了之后马上生效，有个对比"：切换前后的稳定性统计数字
    // 分别累计，肉眼就能看出差别，不需要重启这个窗口或重新连相机。
    connect(useHungarianChk_, &QCheckBox::toggled, this, [onTrackerParamChanged](bool){ onTrackerParamChanged(); });
    connect(useAdaptiveCapChk_, &QCheckBox::toggled, this, [onTrackerParamChanged](bool){ onTrackerParamChanged(); });
    connect(adaptiveCapMulSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onTrackerParamChanged](double){ onTrackerParamChanged(); });
    connect(useMissedRelaxChk_, &QCheckBox::toggled, this, [onTrackerParamChanged](bool){ onTrackerParamChanged(); });
    connect(relaxGrowthSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onTrackerParamChanged](double){ onTrackerParamChanged(); });
    connect(relaxCapSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onTrackerParamChanged](double){ onTrackerParamChanged(); });
    connect(useVelSmoothChk_, &QCheckBox::toggled, this, [onTrackerParamChanged](bool){ onTrackerParamChanged(); });
    connect(velSmoothAlphaSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [onTrackerParamChanged](double){ onTrackerParamChanged(); });
    connect(useConstAccelChk_, &QCheckBox::toggled, this, [onTrackerParamChanged](bool){ onTrackerParamChanged(); });

    // IEKF后端切换/过程噪声——走rebuild()，两个追踪器的轨迹状态不能跨
    // 后端复用，切换本来就该被当成一次新的追踪会话。
    connect(useIekfBackendChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("useIekfBackend"), v);
        rebuild();
    });
    // 骨架叠加纯粹是显示层，不影响任何追踪器状态，不需要 rebuild()——下一次
    // tryCluster() 自然会按新的勾选状态决定要不要调用 skeletonAssoc_。
    connect(showSkeletonChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("showSkeleton"), v);
    });
    connect(iekfPosProcessVarSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfPosProcessVar"), v);
        iekfTracker_.setPosProcessVar(v);   // 这个可以热改，不需要rebuild(不影响已有轨迹的身份，只影响后续预测)
    });
    connect(iekfVelProcessVarSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfVelProcessVar"), v);
        iekfTracker_.setVelProcessVar(v);
    });

    // 【新增·IEKF自适应门控开关的存档+热改】全部只调运行时setter，不rebuild()
    // (不影响已有轨迹身份，切换前后稳定性统计分别累计，肉眼即可对拍)。
    connect(iekfMahaGateChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("iekfMahaGate"), v);
        iekfTracker_.setUseMahalanobisGate(v);
    });
    connect(iekfChiSquareSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfChiSquare"), v);
        iekfTracker_.setChiSquareGate(v);
    });
    connect(iekfDualAnchorChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("iekfDualAnchor"), v);
        iekfTracker_.setUseDualAnchorGate(v);
    });
    connect(iekfCoastSuppressChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("iekfCoastSuppress"), v);
        iekfTracker_.setUseCoastGhostSuppression(v);
    });
    connect(iekfVelCapGainSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfVelCapGain"), v);
        iekfTracker_.setVelCapGain(v);
    });
    connect(iekfManeuverQChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("iekfManeuverQ"), v);
        iekfTracker_.setUseManeuverAdaptiveQ(v);
    });
    connect(iekfMaxConfirmedSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int v){
        settings_.setValue(QStringLiteral("iekfMaxConfirmed"), v);
        iekfTracker_.setMaxConfirmedTracks(v);
    });
    connect(iekfDensityGateChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("iekfDensityGate"), v);
        iekfTracker_.setUseDensityGate(v);
    });
    connect(iekfDensityReachSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfDensityReach"), v);
        iekfTracker_.setDensityReachMult(v);
    });
    connect(iekfDensitySepSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfDensitySep"), v);
        iekfTracker_.setDensityAmbiguitySep(v);
    });

    // 【新增·补全先前未暴露的IEKF参数】同一套热改模式：只调运行时setter，
    // 不rebuild()，不影响已有轨迹身份。
    connect(iekfGlobalAssignChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("iekfGlobalAssign"), v);
        iekfTracker_.setUseGlobalAssignment(v);
    });
    connect(iekfTentRampChk_, &QCheckBox::toggled, this, [this](bool v){
        settings_.setValue(QStringLiteral("iekfTentRamp"), v);
        iekfTracker_.setUseTentativeVelRamp(v);
    });
    connect(iekfTentRampHitsSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfTentRampHits"), v);
        iekfTracker_.setTentativeRampHits(v);
    });
    connect(iekfConfirmedCapMaxSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfConfirmedCapMax"), v);
        iekfTracker_.setConfirmedCapMax(v);
    });
    // 协方差地板：两个控件共同决定实际传给setAssocPosFloorVar的值——自动
    // 勾上时传-1(IekfPointTracker内部据此转回自动公式)，不勾时传手动值。
    auto applyFloorVar = [this]{
        const bool autoFloor = iekfUseAutoFloorChk_->isChecked();
        settings_.setValue(QStringLiteral("iekfUseAutoFloor"), autoFloor);
        settings_.setValue(QStringLiteral("iekfFloorVar"), iekfFloorVarSpin_->value());
        iekfFloorVarSpin_->setEnabled(!autoFloor);
        iekfTracker_.setAssocPosFloorVar(autoFloor ? -1.0 : iekfFloorVarSpin_->value());
    };
    connect(iekfUseAutoFloorChk_, &QCheckBox::toggled, this, [applyFloorVar](bool){ applyFloorVar(); });
    connect(iekfFloorVarSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyFloorVar](double){ applyFloorVar(); });
    connect(iekfManeuverThreshSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfManeuverThresh"), v);
        iekfTracker_.setManeuverResidThresh(v);
    });
    connect(iekfManeuverBoostMaxSpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfManeuverBoostMax"), v);
        iekfTracker_.setManeuverBoostMax(v);
    });
    connect(iekfManeuverDecaySpin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v){
        settings_.setValue(QStringLiteral("iekfManeuverDecay"), v);
        iekfTracker_.setManeuverBoostDecay(v);
    });

    rebuild();

    // ---- 给面板上每一个控件接上操作事件（v8.1）----
    // 【放在构造函数最末尾，不是 buildParamSections 末尾】它是一次性的
    // findChildren 扫描，而 outputMonitor_ 在 buildParamSections 之后才建 ——
    // 早调一步，那一整块的控件就【永远不会被记录】。这种"部分覆盖"是最坏的
    // 情况：事件流看起来完整、实际有洞，会让人得出"这段时间没人动过"的
    // 错误结论，而错误结论比没有结论更贵。
    installUiEventRecorder();
}


} // namespace mocap
