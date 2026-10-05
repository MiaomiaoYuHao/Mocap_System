// ===========================================================================
// PointCloudTestDialog_capture.cpp —— PointCloudTestDialog 的“录制/采集/聚类”分片。
// 从 PointCloudTestDialog.cpp 拆出（原第 4393~5491 行）：recordingDir ~ tryCluster。
// 仍是该类成员函数，与其它 .cpp 共用头文件。
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

// 录制文件放哪。
//
// 【原来放 QStandardPaths::DocumentsLocation 的问题】那是系统"文档"目录，
// 跟项目没关系：换台机器路径就变、跟 git 仓库离散、打包发给别人时要满硬盘找、
// 而且中文用户名下的路径经常把后续脚本坑掉。录制文件是【项目的产物】，
// 就该躺在项目里。
//
// 找项目根的办法：从可执行文件所在目录往上走，找到含 CMakeLists.txt 的那一层
// （build/ 子目录的情形因此能正确跳出去）。找不到就退回可执行文件目录 ——
// 退化档也仍然比系统文档目录好，起码是自包含的。
QString PointCloudTestDialog::recordingDir() {
    QDir d(QCoreApplication::applicationDirPath());
    for (int up = 0; up < 6; ++up) {
        if (d.exists(QStringLiteral("CMakeLists.txt"))) break;
        if (!d.cdUp()) break;
    }
    if (!d.exists(QStringLiteral("CMakeLists.txt")))
        d = QDir(QCoreApplication::applicationDirPath());
    d.mkpath(QStringLiteral("recordings"));
    d.cd(QStringLiteral("recordings"));
    return d.absolutePath();
}

// ---------------------------------------------------------------------------
// ROM 按钮事件落盘。
//
// 【墙钟而不是帧时间戳】按钮回调跑在 GUI 线程上，这一刻并没有"当前帧"这个
// 概念可用 —— 骨架帧是 worker 线程异步推过来的，GUI 这边拿到的最新一帧
// 可能已经是几十毫秒前的了。硬凑一个帧号只会造出一个看起来精确、
// 实际对不上的时间。所以这里只写墙钟，frameTsNs 留 -1；
// 帧对齐由 worker 侧的事件 18/19 负责，两者的差就是投递延迟。
// ---------------------------------------------------------------------------
void PointCloudTestDialog::recordRomEvent(int code, const QString& text, int intA)
{
    if (!recorder_.recording()) return;
    const qint64 wall = QDateTime::currentMSecsSinceEpoch() * 1000000LL;
    const double cov = skeletonWorker_ ? skeletonWorker_->romCoverage() : -1.0;
    recorder_.writeEvent(code, 0, wall, -1, text.toStdString(), cov, 0.0, intA, 0);
    // 【同时打一个 Mark】Mark 是给"在时间轴上一眼找到这里"用的，
    // 现有的报告脚本已经会把它画出来。事件是给机器读的，Mark 是给人看的，
    // 两个都留着，找的时候不用先想该看哪一个。
    recorder_.writeMark(wall, 2, text.toStdString());
}

// ---------------------------------------------------------------------------
// UDP 输出闭环落盘（块 23）。
//
// 【为什么按时间戳配对而不是各写各的】M3DS 和 M3DQ 是两个独立的包，
// 下游要靠时间戳把两者对齐。分开写两条记录的话，"这两个包的 ts 对不对得上"
// 这个问题反而更难查 —— 那正是下游插值出鬼的成因之一。配成一条，
// 两个 ts 不一致时【当场就能看出来】。
//
// 【数据全部来自 UdpSender 的回执】包括 err<0 的两种"没发"：
//   -1 = 打包失败（形状不对，整帧不发）
//   -2 = 推送开关是关的
// 这两条原来在 UdpSender 里是静默 return，一点痕迹都没有 —— 而下游
// "收不到数据"时，它们跟"发了但对端没配"完全分不开。
// ---------------------------------------------------------------------------
void PointCloudTestDialog::onUdpPacketSent(int which, int bytes, int err,
                                           quint64 seq, quint32 crc,
                                           quint32 flags, qint64 tsNs)
{
    if (!recorder_.recording()) return;
    // 换帧了：把上一帧攒的那条先落盘。缺的那一半 bytes 留 0 ——
    // 【0 就是"这一帧没发这个包"】，是一条真结论，不是缺失值。
    if (udpPendDirty_ && tsNs != udpPendTs_) flushUdpOut();

    if (!udpPendDirty_) {
        udpPend_ = mocap::pcrec::UdpOutRec{};
        udpPend_.wallNs = QDateTime::currentMSecsSinceEpoch() * 1000000LL;
        udpPend_.frameTsNs = tsNs;
        udpPendTs_ = tsNs;
        udpPendDirty_ = true;
    }
    if (which == 0) {
        udpPend_.m3dsBytes = int32_t(bytes);
        udpPend_.m3dsCrc   = crc;
        udpPend_.m3dsSeq   = int64_t(seq);
        udpPend_.m3dsErr   = int32_t(err);
    } else {
        udpPend_.m3dqBytes  = int32_t(bytes);
        udpPend_.m3dqCrc    = crc;
        udpPend_.m3dqSeq    = int64_t(seq);
        udpPend_.m3dqErr    = int32_t(err);
        udpPend_.m3dqFlags  = flags;
        udpPend_.quatEnabled = uint8_t(bytes > 0);
    }
}

void PointCloudTestDialog::flushUdpOut()
{
    if (!udpPendDirty_) return;
    udpPendDirty_ = false;
    if (!recorder_.recording()) return;
    udpPend_.enabled    = uint8_t(udpEnabled_ ? 1 : 0);
    udpPend_.targetPort = int32_t(udpPort_);
    recorder_.writeUdpOut(udpPend_);
}

void PointCloudTestDialog::setUdpEndpoint(bool enabled, int port)
{
    udpEnabled_ = enabled;
    udpPort_ = port;
    // 【开关一变就打一个事件】"面板在刷数但 Unity 收不到"九成是这个开关，
    // 而它是什么时候被关的、关之前发过多少包，只有事件流答得出来。
    if (recorder_.recording()) {
        const qint64 wall = QDateTime::currentMSecsSinceEpoch() * 1000000LL;
        recorder_.writeEvent(14, enabled ? 0 : 1, wall, -1,
                             enabled ? std::string("UDP 推送已开启")
                                     : std::string("UDP 推送已关闭"),
                             0.0, 0.0, port, 0);
    }
}

void PointCloudTestDialog::toggleRecording() {
    if (recorder_.recording()) {
        // ---- 停录前先把最终配置和停止事件写进去 ----
        // 【顺序要紧】stop() 之后一个字节都写不进去了。而"停的时候是什么
        // 配置"恰恰是最该有的一份 —— 否则要靠重放整条增量事件流才能算出来，
        // 而重放本身是个会出错的步骤。
        const qint64 wall = QDateTime::currentMSecsSinceEpoch() * 1000000LL;
        recorder_.writeParamDelta(wall,
            std::string("{\"uiSnapshotStop\":") + uiSnapshotJson() + "}");
        recorder_.writeEvent(22, 0, wall, -1,
                             std::string("停止录制"),
                             double(recorder_.bytesWritten()) / 1048576.0,
                             recorder_.queueLoad(),
                             int(recorder_.chunksWritten()),
                             int(recorder_.droppedChunks()));
        recorder_.stop();
        // 停录就关掉原始流采集：常态运行不该为调试每帧多拷 ~1.8KB。
        if (skeletonWorker_) skeletonWorker_->setCaptureDebugStreams(false);
        recBtn_->setText(QStringLiteral("● 录制"));
        recMarkBtn_->setEnabled(false);
        // 【把完整路径打出来】录完最常见的下一步就是"把文件发给别人"，
        // 而找不到文件是这一步最常见的卡点。同时提示 .json 边车文件的存在。
        const QString p = QString::fromStdString(recorder_.path());
        recStatusLabel_->setText(QStringLiteral("已保存 %1 MB → %2  (同名 .json 是可直接看的头信息)")
            .arg(double(recorder_.bytesWritten()) / 1048576.0, 0, 'f', 1)
            .arg(QDir::toNativeSeparators(p)));
        recLastPath_ = p;
        if (recOpenDirBtn_) recOpenDirBtn_->setEnabled(true);
        return;
    }
    const QString dir = recordingDir();
    const QString fn = QStringLiteral("%1/pcrec_%2_%3.pcrec")
        .arg(dir)
        .arg(recProtoCombo_->currentText())
        .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
    recCalibWritten_ = false;
    recLastFrameTsNs_ = -1;
    recDtMean_ = recDtVar_ = 0.0;
    // 【详细度：默认全录】录制这个动作本身就是为了排查，而排查时最贵的
    // 从来不是磁盘，是"录完发现关键那一项没开、得重录一遍"—— 尤其是偶发
    // 问题，重录不一定复现得出来。Full 约 1.4MB/s（一分钟 84MB），
    // 对 30~60 秒的排查性录制完全可以接受。
    // 面板上有下拉框就按下拉框，没有就 Full。
    recorder_.setDetail(recDetailCombo_
        ? mocap::pcrec::Detail(qBound(0, recDetailCombo_->currentIndex(), 2))
        : mocap::pcrec::Detail::Full);
    if (!recorder_.start(fn.toStdString(), buildRecordHeader())) {
        recStatusLabel_->setText(QStringLiteral("打开文件失败"));
        return;
    }
    // 【只在录制期间打开】NetRaw + AssocInput 是离线复算的最小闭包：
    // 没有"送进网络的候选点及顺序"，离线的指派结果就跟在线对不上；
    // 没有网络原始输出，就分不开"模型判错了"和"后处理搞砸了"。
    // v5 起，六级位置快照也挂在这个开关上 —— 它们同样只在录制时才有意义。
    if (skeletonWorker_) {
        skeletonWorker_->setCaptureDebugStreams(true);
        // 全量指派矩阵单独开：约 200KB/s，只有 Paranoid 档才值得。
        skeletonWorker_->setCaptureFullAssign(recorder_.wantsParanoid());
    }
    // ---- 起录第一件事：把 87 个控件的当前值全打进去 ----
    // 【为什么必须有这份基线】增量事件只记"被改动过的"。一个全程没被碰过的
    // 控件不产生任何事件，它的值在文件里就是空白 —— 而"没被碰过的那些"
    // 才是系统运行的大背景：出问题时要先确认其它东西都在预期位置，
    // 才轮得到看变了的那几个。
    {
        const qint64 wall0 = QDateTime::currentMSecsSinceEpoch() * 1000000LL;
        recorder_.writeParamDelta(wall0,
            std::string("{\"uiSnapshotStart\":") + uiSnapshotJson() + "}");
        recorder_.writeEvent(21, 0, wall0, -1,
                             std::string("开始录制：") + fn.toStdString(),
                             0.0, 0.0,
                             recDetailCombo_ ? recDetailCombo_->currentIndex() : 1, 0);
    }
    recTimer_.restart();
    recBtn_->setText(QStringLiteral("■ 停止"));
    recMarkBtn_->setEnabled(true);
    recStatusLabel_->setText(QStringLiteral("录制中… → %1")
        .arg(QDir::toNativeSeparators(dir)));
}

void PointCloudTestDialog::onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns) {
    const bool isNewBatch = !batchTimer_->isActive();
    if (isNewBatch) { batchClock_.restart(); arrivalMsThisBatch_.clear(); }
    arrivalMsThisBatch_[camId] = double(batchClock_.nsecsElapsed()) / 1.0e6;
    // 窗口无关的到达时刻：全局单调时钟，不随批次清空。见头文件说明。
    if (!arrivalClock_.isValid()) arrivalClock_.start();
    {   // 顺便量这台相机自己的投递周期，用来算窗口的帧率代价
        const qint64 nowNs = arrivalClock_.nsecsElapsed();
        auto it = lastArrivalNs_.constFind(camId);
        if (it != lastArrivalNs_.constEnd()) {
            const double dtMs = double(nowNs - it.value()) / 1.0e6;
            if (dtMs > 0.2 && dtMs < 200.0)
                emaCamPeriodMs_ = (emaCamPeriodMs_ <= 0.0) ? dtMs
                                                           : (0.05 * dtMs + 0.95 * emaCamPeriodMs_);
        }
        lastArrivalNs_[camId] = nowNs;
    }
    latestBlobs_[camId] = {pts, {}, ts_ns};   // 质心法没有单观测协方差，covs留空
    if (recorder_.recording()) {
        std::vector<mocap::pcrec::Blob2D> bs;
        bs.reserve(size_t(pts.size()));
        for (const auto& q : pts) bs.push_back({float(q.x()), float(q.y()), 0,0,0, 0,0,0,0});
        recorder_.writeCamBlobs(camId, ts_ns, bs);
    }
    if (isNewBatch) batchTimer_->start(batchWindowSpin_->value());
}

void PointCloudTestDialog::onBlobDetails(quint32 camId, const QVector<Blob>& blobs, qint64 ts_ns) {
    // 【新增】圆拟合法：每个连通域的轮廓点(Blob::contour)独立跑一遍
    // detect/DetectionOutput.hpp::detectBalls()——跟HandTrackingWorker用
    // 的是同一套遮挡感知拟合(3a~3d：约束圆拟合+弧长门控+双球分离)，只是
    // 这里的knownRadius来自用户直接填的"球半径(像素)"这个先验值，而不是
    // HandTrackingWorker那边基于深度估计动态算出来的knownRadiusPx——这个
    // 独立测试窗口没有手部模型、没有深度估计基础设施，直接让用户根据
    // 自己相机的实际成像给一个先验半径是最简单诚实的做法。
    //
    // 只采纳usable=true的观测(弧长足够、拟合可信)——跟HandTrackingWorker
    // 的策略不同：那边"哪怕低置信度也比完全没观测强"是它自己的取舍(见
    // DetectionOutput.hpp文件头注释)，这里的目的是纯粹对比"圆拟合法本身
    // 定位准不准/稳不稳"，掺进不可信的观测会污染这个对比，不采纳。
    //
    // 【新增】同时收集每个可用观测的协方差(obs.sigma，像素单位)，供
    // tryCluster()在开启"马氏距离门控"时使用——是否要转成归一化坐标系、
    // 要不要真的喂给clusterMultiView，是tryCluster()的决定，这里只负责
    // 如实带出来，不在这一层做取舍。
    //
    // ===================================================================
    // 【为什么这里还需要两道闸——它跟检测参数面板不是重复的】
    // 这个槽函数跑在【GUI线程】上(相机检测线程 emit，跨线程排队投递过来)。
    // detectBalls 对单个候选 blob 的代价大约是"650~1300 遍轮廓点的带 sqrt
    // 距离计算"(约束圆拟合30次LM迭代 + 双圆拟合20轮外层×每轮两次30迭代)，
    // 也就是说【代价随轮廓点数线性放大】。
    //
    // 检测参数面板里的 minArea/maxArea/minCircularity 已经在
    // CentroidDetector 里把"不像球"的连通域过滤掉了，那是第一道也是【正确
    // 的】防线——画面正常时(纯黑底 + 几十个小圆亮斑)根本轮不到下面这两道闸。
    // 但它们管的是【面积和圆度】，而这里的成本变量是【轮廓点数】，两者不是
    // 一回事：又细又弯带毛刺的斑，面积可能完全在 maxArea 之内，周长却极大
    // (实测抓到过面积达标、轮廓 601 点的 blob)。所以：
    //   闸① 轮廓点数上限(可在界面上调，默认200，0=不限)：按成本变量本身
    //        设限，补上面积/圆度覆盖不到的那一类。
    //   闸② 单帧时间预算(2ms，刻意【不】做成可调参数)：这是结构性保险，
    //        不是调参旋钮。任何基于形状的参数都要求用户【先把它调对】，
    //        而调参这个动作本身要求界面是活的——界面卡死→没法调参→一直
    //        卡死，这个死环必须从结构上切断，不能再依赖另一个需要调的数。
    //
    // 【必须说清楚：这两道闸不是解法，是安全网】相机没摆好/阈值不对时，
    // 正解永远是去检测参数面板把 threshold 提上去、maxArea 收紧、
    // minCircularity 调高，把不规则亮区在检测层就滤掉。闸的唯一职责是
    // 保证那种时候界面还活着，让你有机会去调。触发次数会如实显示在状态栏
    // 的"圆拟合分流：… / 跳过N"里，长期不为0 就是在提示你去调参数。
    // ===================================================================
    const double radiusPx = circleRadiusPxSpin_->value();

    // 闸①：0 表示不限制(完全交给检测参数面板去管)。
    const int maxContourPts = maxContourPtsSpin_->value();
    // 闸②：单帧圆拟合总预算。刻意硬编码，理由见上面那段说明。
    constexpr double kFitBudgetMs = 2.0;
    QElapsedTimer fitBudget; fitBudget.start();

    QVector<QPointF> pts;
    QVector<Cov2> covs;
    pts.reserve(blobs.size() * 2);   // 双球场景一个blob可能产出2个观测
    covs.reserve(blobs.size() * 2);
    // 【架构·质心为主 + 花生补圆拟合】按 blob 分流，而不是整条链路二选一：
    //   · 外接框近正方形(长宽比<=1.3)的 blob = 正常单球，直接用检测器已经
    //     算好的灰度加权质心(Blob::cx/cy)，完全不进圆拟合——不建轮廓 vector、
    //     不跑 fitArcWithGating、不跑双圆。单球本来质心法就又快又准(实测对
    //     对称单球质心到真值 0.048px，比圆拟合的 0.071px 还略好)。
    //   · 只有长宽比>1.3(或外接框缺失,保守当可疑)的花生形 blob 才跑
    //     detectBalls 做双圆拆分。密集摆位下这类是少数,昂贵的圆拟合只花在
    //     真正需要的 blob 上。
    // 这样单球零圆拟合开销,是"勾了圆拟合法就整帧全跑 detectBalls 卡爆"的根治。
    int nSingle = 0, nPeanut = 0, nSkipped = 0;
    for (const Blob& b : blobs) {
        // 外接框长宽比预筛：先判这个 blob 走哪条路。
        bool mayBeMerged = true;
        if (b.bboxW > 0 && b.bboxH > 0) {
            const double lo = double(std::min(b.bboxW, b.bboxH));
            const double hi = double(std::max(b.bboxW, b.bboxH));
            mayBeMerged = (lo <= 0.0) || (hi / lo > 1.3);
        }

        if (!mayBeMerged) {
            // 单球快路径：直接用检测器自带的灰度加权质心，零圆拟合。
            // 协方差留空(covs 里放一个默认标记)，跟质心法一致——马氏门控
            // 缺协方差时会走兜底 defaultObsVar，见 tryCluster()。
            pts.push_back(QPointF(b.cx, b.cy));
            covs.push_back(Cov2{});   // 空协方差 = 用全局兜底信任度
            ++nSingle;
            continue;
        }

        // 【闸①·轮廓点数 / 闸②·时间预算】任一触发都退回自带质心。
        // 注意这里【不丢点】——点还在，只是这一帧不对它做亚像素圆拟合，
        // 精度略降，换来的是 GUI 线程永远能回来。
        const bool contourTooBig = (maxContourPts > 0) && (int(b.contour.size()) > maxContourPts);
        const bool budgetSpent   = fitBudget.nsecsElapsed() > qint64(kFitBudgetMs * 1e6);
        if (b.contour.empty() || contourTooBig || budgetSpent) {
            pts.push_back(QPointF(b.cx, b.cy));
            covs.push_back(Cov2{});
            if (contourTooBig || budgetSpent) ++nSkipped;
            continue;
        }

        std::vector<Point2> contour;
        contour.reserve(b.contour.size());
        for (const auto& c : b.contour) contour.push_back({double(c[0]), double(c[1])});
        const auto out = detectBalls(contour, radiusPx, /*mayBeMerged=*/true);
        if (!out.valid) {
            // 拆分本身没成:退回自带质心兜底。
            pts.push_back(QPointF(b.cx, b.cy));
            covs.push_back(Cov2{});
            continue;
        }
        ++nPeanut;
        for (const auto& obs : out.observations) {
            if (!obs.usable) continue;
            pts.push_back(QPointF(obs.mu.x, obs.mu.y));
            covs.push_back(obs.sigma);
        }
    }
    // 分流统计：供调试观察"单球快路径 vs 花生圆拟合 vs 被闸挡下"的比例。
    // nSkipped 长期大于0 = 画面里存在轮廓过大的亮区，说明检测参数(阈值/
    // 最大面积/圆度下限)或相机摆位需要调——这正是以前只能靠"界面卡死"
    // 才能发现的信息，现在它变成状态栏上一个可读的数字。
    lastFrameSingleBlobs_ = nSingle;
    lastFramePeanutBlobs_ = nPeanut;
    lastFrameSkippedBlobs_ = nSkipped;

    const bool isNewBatch = !batchTimer_->isActive();
    if (isNewBatch) { batchClock_.restart(); arrivalMsThisBatch_.clear(); }
    arrivalMsThisBatch_[camId] = double(batchClock_.nsecsElapsed()) / 1.0e6;
    if (!arrivalClock_.isValid()) arrivalClock_.start();
    {   const qint64 nowNs = arrivalClock_.nsecsElapsed();
        auto it = lastArrivalNs_.constFind(camId);
        if (it != lastArrivalNs_.constEnd()) {
            const double dtMs = double(nowNs - it.value()) / 1.0e6;
            if (dtMs > 0.2 && dtMs < 200.0)
                emaCamPeriodMs_ = (emaCamPeriodMs_ <= 0.0) ? dtMs
                                                           : (0.05 * dtMs + 0.95 * emaCamPeriodMs_);
        }
        lastArrivalNs_[camId] = nowNs;
    }
    latestBlobs_[camId] = {pts, covs, ts_ns};
    if (recorder_.recording()) {
        std::vector<mocap::pcrec::Blob2D> bs;
        bs.reserve(size_t(pts.size()));
        const bool hc = (covs.size() == pts.size());
        for (int i = 0; i < pts.size(); ++i)
            bs.push_back({float(pts[i].x()), float(pts[i].y()),
                          hc ? float(covs[i].xx) : 0.f, hc ? float(covs[i].xy) : 0.f,
                          hc ? float(covs[i].yy) : 0.f,
                          float(blobs[i].area), float(blobs[i].peak),
                          uint16_t(blobs[i].bboxW > 0 ? blobs[i].bboxW : 0),
                          uint16_t(blobs[i].bboxH > 0 ? blobs[i].bboxH : 0)});
        recorder_.writeCamBlobs(camId, ts_ns, bs);
    }
    if (isNewBatch) batchTimer_->start(batchWindowSpin_->value());
}

void PointCloudTestDialog::onBatchTimer() {
    tryCluster();
}

void PointCloudTestDialog::onBindModeToggled(bool on) {
    pendingFirstId_ = -1;   // 切换模式时清掉待连接状态，避免"上次点了一半"的残留状态串到新一轮
    bindStatusLabel_->setText(on
        ? QStringLiteral("绑定模式已开启——点第一个点，再点第二个点连一条线。当前已有 %1 条连线。").arg(edges_.size())
        : QStringLiteral("绑定模式已关闭。当前已有 %1 条连线（关闭模式不会清空已连的线）。").arg(edges_.size()));
}

void PointCloudTestDialog::applyIekfObsFloor() {
    // 用当前已标定、参与聚类的相机的fx，把用户填的"观测方差地板(px)"换算成
    // 归一化坐标下的方差：floor = (px/fx)²。跟 applyIekfDefaultObsVar() 同一套
    // 取fx逻辑，理由见那边的说明。
    double fxSum = 0.0; int fxCount = 0;
    for (const auto& e : activeCams_) {
        const CameraCalibration calib = store_->get(e.deviceKey);
        if (!calib.isCalibrated()) continue;
        fxSum += (calib.intr.fx + calib.intr.fy) * 0.5;
        ++fxCount;
    }
    const double fx = fxCount > 0 ? (fxSum / fxCount) : 900.0;
    const double px = iekfObsFloorPxSpin_ ? iekfObsFloorPxSpin_->value() : 0.0;
    const double sn = px / std::max(1e-6, fx);
    iekfTracker_.setObsVarFloor(px > 0.0 ? sn * sn : 0.0);
}

void PointCloudTestDialog::applyIekfDefaultObsVar() {
    // 用当前"已标定、参与聚类"的相机的fx取平均，把用户填的"检测噪声(px)"
    // 换算成IEKF需要的归一化坐标方差：obsVar = (噪声px / fx)²。
    // 这一步只影响 IEKF 在"没有per-观测协方差"时的兜底信任度(见
    // IekfPointTracker::defaultObsVar_ 的用法)——勾了"马氏距离门控"后，每个
    // 观测都带着自己在 buildObservations() 里算出来的真实协方差，这个全局值
    // 只在个别观测缺协方差时才用得上；没勾马氏门控时，这个值就是IEKF对
    // *所有*观测的统一信任度，直接决定它敢不敢信新测量、滞不滞后。
    //
    // 没有已标定相机时(比如还没连相机)，用一个保守的默认fx=900做换算，
    // 只是让预览文字有内容可看，不影响真正跑数据时的逐帧换算——activeCams_
    // 一旦有已标定相机，下一帧tryCluster()走到这里就会用上真实fx。
    double fxSum = 0.0; int fxCount = 0;
    for (const auto& e : activeCams_) {
        const CameraCalibration calib = store_->get(e.deviceKey);
        if (!calib.isCalibrated()) continue;
        fxSum += (calib.intr.fx + calib.intr.fy) * 0.5;
        ++fxCount;
    }
    const double fx = fxCount > 0 ? (fxSum / fxCount) : 900.0;
    const double noisePx = iekfDetectNoisePxSpin_ ? iekfDetectNoisePxSpin_->value() : 0.8;
    const double obsVar = (noisePx / std::max(1e-6, fx)) * (noisePx / std::max(1e-6, fx));

    iekfTracker_.setDefaultObsVar(obsVar);
    lastAppliedDefaultObsVar_ = obsVar;

    if (obsVarPreviewLabel_) {
        obsVarPreviewLabel_->setText(QStringLiteral(
            "≈方差 %1（等效fx=%2%3）")
            .arg(obsVar, 0, 'e', 2)
            .arg(fx, 0, 'f', 0)
            .arg(fxCount > 0 ? QStringLiteral("") : QStringLiteral("·尚无已标定相机，用默认值预览")));
    }
}

void PointCloudTestDialog::resetStabilityStats() {
    // 起点=此刻当前活跃后端已经发过多少个ID(不是清零内部计数器，那样会
    // 跟"已有的、还在追踪中的编号"产生冲突)——之后只看"这之后又新增了
    // 多少个"，配合"这之后同时活跃过的峰值"，就是一段独立可比的统计
    // 区间，不受重置之前那段历史的污染。两个后端各自独立计数，切换后端
    // 本身也会走rebuild()->这里，天然是"新的一段统计"，不会把两个后端
    // 的数字混在一起看。
    const bool useIekf = useIekfBackendChk_->isChecked();
    statsBaselineTotalIds_ = useIekf ? iekfTracker_.totalIdsAssigned() : tracker_.totalIdsAssigned();
    statsPeakActive_ = useIekf ? iekfTracker_.activeCount() : tracker_.activeCount();
    stabilityLabel_->setText(QStringLiteral("稳定性统计：已重置，等待数据…"));

    // 耗时统计一并清零——EMA从头累积，峰值重新计，避免早期(比如刚打开
    // 窗口、相机还没热起来)的异常值污染后面的读数。lastCallTimerValid_
    // 置false是因为"重置"这个动作本身会打断处理节奏，下一次tryCluster()
    // 量到的间隔是"从点了重置按钮到下一帧"而不是真实的帧间处理间隔，
    // 这一次的间隔样本应该丢弃，不计入统计。
    emaIntervalMs_ = emaBuildMs_ = emaClusterMs_ = emaTrackMs_ = emaTotalMs_ = maxTotalMs_ = 0.0;
    lastCallTimerValid_ = false;
    emaArrivalSpreadMs_ = 0.0;
    maxArrivalSpreadMs_ = 0.0;
    timingLabel_->setText(QStringLiteral("耗时统计：已重置，等待数据…"));
}

void PointCloudTestDialog::onClearEdges() {
    edges_.clear();
    pendingFirstId_ = -1;
    view_->setEdges(edges_);
    bindStatusLabel_->setText(QStringLiteral("已清除全部连线。"));
}

void PointCloudTestDialog::onPointClicked(int id) {
    if (!bindModeChk_->isChecked()) return;   // 不在绑定模式下，点击点云纯粹是看，不触发连线逻辑

    if (pendingFirstId_ < 0) {
        pendingFirstId_ = id;
        bindStatusLabel_->setText(QStringLiteral("已选中点 #%1，再点一个点完成连线（当前 %2 条连线）。")
            .arg(id).arg(edges_.size()));
        return;
    }
    if (pendingFirstId_ == id) {
        // 点了同一个点两次——当成"取消这次选择"，不连自己到自己。
        pendingFirstId_ = -1;
        bindStatusLabel_->setText(QStringLiteral("已取消选择（当前 %1 条连线）。").arg(edges_.size()));
        return;
    }

    const int a = pendingFirstId_, b = id;
    pendingFirstId_ = -1;
    // 去重：同一对点(不分先后顺序)已经连过就不再重复添加。
    const bool exists = std::any_of(edges_.begin(), edges_.end(), [a, b](const QPair<int,int>& e) {
        return (e.first == a && e.second == b) || (e.first == b && e.second == a);
    });
    if (exists) {
        bindStatusLabel_->setText(QStringLiteral("#%1 - #%2 这条线已经连过了（当前 %3 条连线）。")
            .arg(a).arg(b).arg(edges_.size()));
        return;
    }
    edges_.push_back({a, b});
    view_->setEdges(edges_);
    bindStatusLabel_->setText(QStringLiteral("已连接 #%1 - #%2（当前 %3 条连线）。")
        .arg(a).arg(b).arg(edges_.size()));
}

void PointCloudTestDialog::tryCluster() {
    QElapsedTimer totalTimer; totalTimer.start();
    // 【新增】帧间隔实测——两次tryCluster()真正被调用之间隔了多久，跟
    // "相机名义帧间隔"对比就能看出是不是在丢帧，见头文件对应成员说明。
    double intervalMsThisCall = -1.0;
    if (lastCallTimerValid_) intervalMsThisCall = double(lastCallTimer_.nsecsElapsed()) / 1.0e6;
    lastCallTimer_.restart();
    lastCallTimerValid_ = true;

    // 【新增】本批次相机到达偏差——arrivalMsThisBatch_是onBlobs/
    // onBlobDetails在这个批次窗口内实时记录的，这里只是读出来算个spread，
    // 不代表下一批次的情况(每批次都会被onBlobs/onBlobDetails清空重记)。
    // 【窗口无关】各相机"最近一次投递时刻"的离散度。
    // 每台相机每个帧周期都会投递一次，所以任意时刻各自的最近投递最多差一个
    // 帧周期 —— 这个量只反映相机之间的相位差，跟批处理窗口无关。
    //
    // 停摆的相机单独剔掉：它的最近投递会停在很久以前，算进去会让偏差爆掉，
    // 而"某台相机不出帧"是另一个问题，不该混进"窗口设多大"的判据里。
    double arrivalSpreadMsThisCall = -1.0;
    if (lastArrivalNs_.size() >= 2) {
        const qint64 nowNs = arrivalClock_.nsecsElapsed();
        // 停摆判据：超过 5 个帧周期没投递就不算它（帧周期用实测的 emaIntervalMs_，
        // 还没测出来时退回 20ms，够宽松）
        // 【用的是上一帧的 emaIntervalMs_】它在本函数末尾才更新。这里只拿它
        // 定"停摆"阈值(5 个帧周期)，差一帧完全不影响判定，不是 bug。
        const double periodMs = (emaIntervalMs_ > 0.5) ? emaIntervalMs_ : 20.0;
        const qint64 staleNs = qint64(periodMs * 5.0 * 1.0e6);
        qint64 mn = std::numeric_limits<qint64>::max(), mx = std::numeric_limits<qint64>::min();
        int n = 0;
        for (auto it = lastArrivalNs_.constBegin(); it != lastArrivalNs_.constEnd(); ++it) {
            if (nowNs - it.value() > staleNs) continue;   // 停摆，不计
            mn = std::min(mn, it.value());
            mx = std::max(mx, it.value());
            ++n;
        }
        if (n >= 2) arrivalSpreadMsThisCall = double(mx - mn) / 1.0e6;
    }

    QElapsedTimer stageTimer; stageTimer.start();

    // 【新增】每帧重新换算一次defaultObsVar——activeCams_的标定状态可能在
    // 对话框开着的时候变化(比如用户中途重新标定了某台相机)，这里保证
    // "检测噪声(px)"这个UI值跟IEKF内部实际用的方差始终对得上，不需要用户
    // 手动触发。开销是循环几台相机读一次标定，跟下面构建观测的循环同量级，
    // 可忽略。
    if (useIekfBackendChk_->isChecked()) applyIekfDefaultObsVar();
    if (useIekfBackendChk_->isChecked()) applyIekfObsFloor();

    std::vector<EpiMat3> Rs; std::vector<EpiVec3> ts;
    std::vector<std::vector<std::array<double,2>>> obsPerCam;
    Rs.reserve(activeCams_.size()); ts.reserve(activeCams_.size()); obsPerCam.reserve(activeCams_.size());

    // 【新增】只在"圆拟合法 + 勾了马氏距离门控"时才真正构建协方差数组，
    // 其余情况(质心法，或者没勾)传nullptr给clusterMultiView，退化成旧的
    // 固定maxReprojNorm门控——不无中生有编一份假协方差。
    const bool wantMahalanobis = useCircleFitChk_->isChecked() && useMahalanobisChk_->isChecked();
    std::vector<std::vector<Cov2>> obsCovPerCam;
    if (wantMahalanobis) obsCovPerCam.reserve(activeCams_.size());

    int totalBlobs = 0;
    // 【新增】每台相机这一帧观测的采集时间戳（ns）。这个数据 BlobBuf 里一直
    // 就有(onBlobs/onBlobDetails 都收了 ts_ns)，只是从来没被用过——而
    // IekfPointTracker 的"dt真实化"和"相机不同步补偿"要的正是它。
    // -1 表示该相机这一帧没有有效时间戳。
    std::vector<qint64> camTsNs;
    camTsNs.reserve(activeCams_.size());
    for (const auto& e : activeCams_) {
        const CameraCalibration calib = store_->get(e.deviceKey);
        if (!calib.isCalibrated()) {
            obsPerCam.push_back({}); Rs.push_back({}); ts.push_back({});
            if (wantMahalanobis) obsCovPerCam.push_back({});
            camTsNs.push_back(-1);
            continue;
        }
        Rs.push_back(calib.extr.R);
        ts.push_back(calib.extr.t);

        std::vector<std::array<double,2>> obs;
        std::vector<Cov2> obsCov;
        const auto it = latestBlobs_.constFind(e.cam->id());
        camTsNs.push_back(it != latestBlobs_.constEnd() ? it.value().ts : qint64(-1));
        if (it != latestBlobs_.constEnd()) {
            const auto& buf = it.value();
            const bool haveCov = wantMahalanobis && buf.covs.size() == buf.pts.size();
            for (int i = 0; i < buf.pts.size(); ++i) {
                double nx, ny;
                undistortNormalize(calib.intr, buf.pts[i].x(), buf.pts[i].y(), nx, ny);
                obs.push_back({nx, ny});
                if (wantMahalanobis) {
                    // 像素协方差 -> 归一化坐标系协方差，一阶近似：只用内参
                    // 的线性缩放部分(diag(1/fx,1/fy))，忽略去畸变本身的局部
                    // 雅可比曲率——小畸变/靠近主点时这个近似成立，跟项目
                    // 其它地方"雅可比≈线性近似"是同一处理方式(参见
                    // FixedLagSmoother.hpp旋转平滑先验的类似说明)。
                    //
                    // 【逐观测判有效】原来是整台相机级的 haveCov 判断，但现在
                    // "质心为主+花生补圆拟合"的分流会让同一台相机里【混着】两
                    // 种观测：单球走质心快路径、只带一个空 Cov2(全0)占位；花生
                    // 走圆拟合、带真实协方差。此时 buf.covs.size()==pts.size()
                    // 成立(都push了)，但单球那份是全0——不能当真协方差用，否则
                    // invert2x2(0,0,0) 必然失败，那个单球观测会被马氏门控错误
                    // 丢弃。所以改成逐观测看这份协方差本身是否有效(对角元>0)，
                    // 无效的(单球占位/防御性缺失)一律走各向同性兜底。
                    const Cov2& covPx = (haveCov ? buf.covs[size_t(i)] : Cov2{});
                    const bool covValid = covPx.xx > 0.0 && covPx.yy > 0.0;
                    if (covValid) {
                        const double ifx = 1.0/std::max(1e-6, calib.intr.fx);
                        const double ify = 1.0/std::max(1e-6, calib.intr.fy);
                        obsCov.push_back({covPx.xx*ifx*ifx, covPx.xy*ifx*ify, covPx.yy*ify*ify});
                    } else {
                        // 单球质心 / 缺协方差：给一个跟质心法一致的保守各向同性
                        // 方差。质心法本来就没有逐观测协方差，走这条兜底跟纯
                        // 质心法模式下的行为一致，不额外惩罚单球。
                        const double f = std::max(1e-6, (calib.intr.fx+calib.intr.fy)*0.5);
                        const double fallbackVar = 4.0/(f*f);   // 约等于2px标准差换算到归一化坐标
                        obsCov.push_back({fallbackVar, 0.0, fallbackVar});
                    }
                }
            }
        }
        totalBlobs += int(obs.size());
        obsPerCam.push_back(std::move(obs));
        if (wantMahalanobis) obsCovPerCam.push_back(std::move(obsCov));
    }

    if (totalBlobs == 0) {
        status_->setText(QStringLiteral("已连接 %1 台相机，暂无检测到的反光球").arg(activeCams_.size()));
        return;
    }
    const double buildMsThisCall = double(stageTimer.nsecsElapsed()) / 1.0e6;
    stageTimer.restart();

    // ===================================================================
    // 【新增接线】把真实时间戳喂给 IEKF 追踪器，让第三、四轮做的两件事真正生效：
    //   ① dt 真实化：update() 不传 ts_ns 时内部按"每次调用恰好过了一帧"处理，
    //      丢帧/掉速时预测步全错。这里传真实时间戳。
    //   ② 相机不同步补偿：各相机曝光时刻本来就不在同一瞬间，偏移量就是
    //      各自时间戳跟参考时刻的差。这个数一直躺在 BlobBuf::ts 里没人用，
    //      于是 CameraMeasurement::dt 恒为 0，整条补偿链路等于死代码。
    // 两件事都不需要用户填任何参数——数据本来就有。
    // ===================================================================
    qint64 refTsNs = -1;
    {
        // 参考时刻取各相机时间戳的【均值】。一开始我写的是中位数(想着更抗
        // 单台相机的时间戳异常)，压测台上实测是错的：中位数会让"各相机偏移
        // 量的均值"不为零，等价于把整个输出在时间轴上平移了那么多，在高速
        // 运动下直接变成位置偏差——desync 场景 RMSE 从 2.245 涨到 2.682，
        // 补偿了反而比不补偿更差。改成均值后同一场景是 2.195(略优于不补偿)，
        // 且 P95 从 3.349 降到 1.733。
        // 抗异常值的责任交给下面的 ±0.5 帧夹紧，那道闸门足够了。
        long double acc = 0; int n = 0;
        for (qint64 v : camTsNs) if (v > 0) { acc += static_cast<long double>(v); ++n; }
        if (n > 0) refTsNs = static_cast<qint64>(acc / n);
    }
    if (refTsNs > 0) {
        // 帧周期：用相邻两次参考时刻的间隔做 EMA。不用 UI 上的标称帧率，
        // 因为实际帧率经常达不到标称值(USB带宽、曝光时长都会拖慢)，而
        // dt 折算错了会直接让预测步的过程噪声按错误的帧数累加。
        if (lastRefTsNs_ > 0 && refTsNs > lastRefTsNs_) {
            const double dtNs = double(refTsNs - lastRefTsNs_);
            // 只接受合理区间内的样本(1ms~200ms)，避免暂停/断流后的巨大间隔
            // 污染估计值。
            if (dtNs > 1e6 && dtNs < 2e8)
                emaFrameNs_ = (emaFrameNs_ <= 0) ? dtNs : (0.9*emaFrameNs_ + 0.1*dtNs);
        }
        lastRefTsNs_ = refTsNs;
        if (emaFrameNs_ > 0) iekfTracker_.setNominalFrameNs(qint64(emaFrameNs_));

        // 每相机时刻偏移（单位：帧）。夹在 ±0.5 帧内——超过半帧的"偏移"更
        // 可能是时间戳本身出了问题(丢帧错配、时钟跳变)，硬补进去只会更糟。
        if (iekfDesyncCompChk_->isChecked() && emaFrameNs_ > 0) {
            if (camOffEma_.size() != camTsNs.size())
                camOffEma_.assign(camTsNs.size(), kOffUninit);
            for (size_t i = 0; i < camTsNs.size(); ++i) {
                if (camTsNs[i] <= 0) continue;
                double o = double(camTsNs[i] - refTsNs) / emaFrameNs_;
                o = std::max(-0.5, std::min(0.5, o));   // ±0.5帧夹紧
                // 【必须平滑】真实的相机不同步是准恒定的系统量(各相机时钟
                // 起点不同)，而驱动打时间戳的抖动是零均值白噪声。直接拿每帧
                // 瞬时值当偏移用，等于把这段白噪声乘以速度注入残差——实测在
                // 完全没有不同步的干净场景反而让 RMSE 从 0.576 涨到 0.597。
                // EMA 把噪声压掉、把系统量留下，同时还能跟上时钟慢漂。
                camOffEma_[i] = (camOffEma_[i] <= kOffUninit) ? o
                              : (1.0 - kOffEmaAlpha)*camOffEma_[i] + kOffEmaAlpha*o;
            }
            std::vector<double> offs(camOffEma_.size(), 0.0);
            for (size_t i = 0; i < camOffEma_.size(); ++i)
                offs[i] = (camOffEma_[i] <= kOffUninit) ? 0.0 : camOffEma_[i];
            iekfTracker_.setCameraTimeOffsets(offs);
        } else {
            camOffEma_.clear();
            iekfTracker_.setCameraTimeOffsets({});
        }
    }
    // 曝光中点补偿（运动模糊）：拖尾光斑的质心落在曝光窗口【中点】，而
    // 时间戳打在窗口的哪一端取决于驱动，所以符号必须由用户按自己的设备定，
    // 不能替他猜——上一轮我就在这个符号上栽过。默认曝光时长填 0 = 不补偿。
    {
        const double expMs = iekfExposureMsSpin_->value();
        if (expMs > 0.0 && emaFrameNs_ > 0) {
            const double halfExpFrames = (expMs*1e6*0.5)/emaFrameNs_;
            // 组合框：0=时间戳在曝光结束(UVC/DShow常见，质心时刻更早，dt取负)
            //         1=时间戳在曝光开始(dt取正)
            //         2=时间戳已是曝光中点(无需补偿)
            const int mode = iekfExposureAnchorCombo_->currentIndex();
            const double em = (mode == 0) ? -halfExpFrames
                            : (mode == 1) ? +halfExpFrames : 0.0;
            iekfTracker_.setExposureMidOffset(em);
        } else {
            iekfTracker_.setExposureMidOffset(0.0);
        }
    }

    const auto cluster = clusterMultiView(Rs, ts, obsPerCam,
                                          maxSampsonSpin_->value(), maxReprojSpin_->value(),
                                          /*useLmRefine=*/useLmRefineChk_->isChecked(),
                                          wantMahalanobis ? &obsCovPerCam : nullptr,
                                          /*chiSquareGate=*/clusterChiSquareSpin_->value(), minSupportSpin_->value(),
                                          useVotingChk_->isChecked(), minRayAngleSpin_->value(),
                                          ambiguityMarginSpin_->value(),
                                          lmHuberDeltaSpin_->value(), lmMaxItersSpin_->value(),
                                          calibSigmaNormSpin_->value(),
                                          twoViewFallbackChk_->isChecked(),
                                          twoViewMinRayAngleSpin_->value(),
                                          maxFinalResidualSpin_->value());
    const double clusterMsThisCall = double(stageTimer.nsecsElapsed()) / 1.0e6;
    stageTimer.restart();

    std::vector<Track3D> tracksForId;
    tracksForId.reserve(cluster.tracks.size());
    for (const auto& tr : cluster.tracks) tracksForId.push_back(tr);

    // 【新增】追踪后端切换——IEKF后端需要Rs/ts/obsPerCam/obsCovPerCam这些
    // "原始每相机观测"信息(不只是三角化完的3D点)，这几个变量本来就已经
    // 在这个函数里算好了，直接透传。
    const bool useIekf = useIekfBackendChk_->isChecked();
    const auto tracked = useIekf
        // 【新增】最后一个实参 refTsNs 就是 dt 真实化要的真实时间戳；原来不传
        //   这个参数，内部一律按"过了整一帧"处理，丢帧时预测步三重失准。
        ? iekfTracker_.update(tracksForId, Rs, ts, obsPerCam, wantMahalanobis ? &obsCovPerCam : nullptr, refTsNs)
        : tracker_.update(tracksForId);
    const double trackMsThisCall = double(stageTimer.nsecsElapsed()) / 1.0e6;

    QVector<PointCloudWidget::Point> pts;
    pts.reserve(int(tracked.size()));
    for (const auto& tp : tracked) {
        pts.push_back({QVector3D(float(tp.position[0]), float(tp.position[1]), float(tp.position[2])),
                       tp.id, tp.missedFrames > 0});
    }
    view_->setPoints(pts);

    if (recorder_.recording()) {
        // 【相机标定只写一次】它在一次录制里不变，逐帧写是纯浪费。
        // 写成 ParamDelta 块而不是塞进文件头：录制开始时 Rs/ts 还没算出来，
        // 等第一帧真正跑完 tryCluster 才有——放头里就得往前挪一堆逻辑。
        if (!recCalibWritten_ && !Rs.empty()) {
            // 【必须用 QJsonDocument 生成，不能手拼字符串】
            // 上一版是 QStringLiteral().arg() 拼出来的，deviceKey 里的 Windows
            // 设备路径 web:\\?\usb#... 原样进了 JSON —— 那个 \u 被解析器当成
            // unicode 转义，整块读不出来。已录出来的文件靠 pcrec.py 的容错
            // 解析救回，但源头必须堵住。
            QJsonArray cams;
            for (size_t c = 0; c < Rs.size(); ++c) {
                QJsonObject o;
                o[QStringLiteral("idx")] = int(c);
                QJsonArray R, T;
                for (int i = 0; i < 9; ++i) R.append(Rs[c][size_t(i)]);
                for (int i = 0; i < 3; ++i) T.append(ts[c][size_t(i)]);
                o[QStringLiteral("R")] = R;
                o[QStringLiteral("t")] = T;
                if (c < size_t(activeCams_.size())) {
                    const CameraCalibration cal = store_->get(activeCams_[int(c)].deviceKey);
                    const auto& K = cal.intr;
                    // 【内参必须写】只有 R/t 是无法重新三角化的：2D 观测要先经
                    // undistortNormalize 变成归一化视线方向才能用，而那一步全靠
                    // fx/fy/cx/cy/k1..p2。漏掉内参 = 这份录制的 2D 数据全部作废。
                    o[QStringLiteral("fx")] = K.fx;  o[QStringLiteral("fy")] = K.fy;
                    o[QStringLiteral("cx")] = K.cx;  o[QStringLiteral("cy")] = K.cy;
                    o[QStringLiteral("k1")] = K.k1;  o[QStringLiteral("k2")] = K.k2;
                    o[QStringLiteral("k3")] = K.k3;
                    o[QStringLiteral("p1")] = K.p1;  o[QStringLiteral("p2")] = K.p2;
                    o[QStringLiteral("w")]  = K.width;
                    o[QStringLiteral("h")]  = K.height;
                    o[QStringLiteral("key")] = activeCams_[int(c)].deviceKey;
                }
                cams.append(o);
            }
            QJsonObject root;
            root[QStringLiteral("cameras")] = cams;
            recorder_.writeParamDelta(refTsNs,
                QString::fromUtf8(QJsonDocument(root).toJson(QJsonDocument::Compact)).toStdString());
            recCalibWritten_ = true;

            // ---- 相机运行参数 ----
            // 【为什么跟标定一起写而不是写在文件头】文件头是按下按钮那一刻拼的，
            // 那时相机可能还没 start()；这里是第一帧真到了之后，参数已经生效。
            //
            // 【接口只给得出这些】ICamera 暴露的是 params()(target_fps/threshold)
            // 和几个 const getter。曝光/增益/分辨率【不在 ICamera 接口里】——
            // 真相机走的是 DShow 的属性页，没有统一的读取路径。要记的话得先给
            // ICamera 加 getter，那是另一件事，这里不假装记了。
            if (mgr_) {
                for (int i = 0; i < mgr_->count(); ++i) {
                    ICamera* c = mgr_->at(i);
                    if (!c) continue;
                    const CameraParams cp = c->params();
                    mocap::pcrec::CamSettingsRec cs;
                    cs.camId      = c->id();
                    cs.fps        = float(cp.target_fps);
                    cs.threshold  = cp.threshold;
                    // 下面几个 ICamera 给不出来，留 -1 表示"没记到"而不是"是 0"
                    cs.width = cs.height = -1;
                    cs.exposureUs = cs.gain = -1.f;
                    cs.minArea = cs.maxArea = -1;
                    recorder_.writeCamSettings(cs);
                }
            }
        }
        std::vector<mocap::pcrec::ClusterRec> cr;
        cr.reserve(cluster.tracks.size());
        for (const auto& t2 : cluster.tracks) {
            mocap::pcrec::ClusterRec c;
            c.x = float(t2.point[0]); c.y = float(t2.point[1]); c.z = float(t2.point[2]);
            c.residualPx = float(t2.residual);
            c.verified = uint8_t(t2.verified);
            c.nSupport = uint8_t(t2.support.size());
            for (const auto& sp : t2.support)
                if (sp.first >= 0 && sp.first < 32) c.supportMask |= (1u << sp.first);
            cr.push_back(c);
        }
        recorder_.writeCluster(refTsNs, cr);
        if (!recClockTimer_.isValid() || recClockTimer_.elapsed() >= 1000) {
            recClockTimer_.restart();
            recorder_.writeClockSync(refTsNs, QDateTime::currentMSecsSinceEpoch() * 1000000LL);
        }

        std::vector<mocap::pcrec::Point3DRec> rp;
        rp.reserve(tracked.size());
        for (const auto& tp : tracked) {
            mocap::pcrec::Point3DRec q;
            q.x = float(tp.position[0]); q.y = float(tp.position[1]); q.z = float(tp.position[2]);
            q.id = tp.id;
            q.coasting = uint8_t(tp.missedFrames > 0);
            // 【这三个字段原来定义了却从没填过，一直是 0】
            // 0 是合法值：usedViews=0 会被读成"没有任何相机看到它"，
            // residualMm=0 会被读成"完美三角化"。两者都是最危险的误读方向。
            // 现在从追踪调试和簇结果里回填 —— 找不到对应簇时留 -1/0，
            // 而 -1 是不可能出现的残差，一眼看得出是"没测到"。
            q.predicted = uint8_t(tp.missedFrames > 0);
            q.usedViews = 0;
            q.residualMm = -1.0f;
            rp.push_back(q);
        }
        // 用追踪器的 obsIndex 把 3D 点接回它这一帧关联到的簇，
        // 从簇上取支持相机数和残差 —— 这两个量本来就在 cluster.tracks 里，
        // 只是过去没人把它们跟追踪 id 对上。
        {
            const auto& td = useIekf ? iekfTracker_.lastFrameDebug()
                                     : tracker_.lastFrameDebug();
            for (const auto& it : td.items) {
                if (it.obsIndex < 0 || it.obsIndex >= int(cluster.tracks.size())) continue;
                for (auto& q : rp) {
                    if (q.id != it.id) continue;
                    const auto& ct = cluster.tracks[size_t(it.obsIndex)];
                    q.usedViews  = uint8_t(ct.support.size());
                    q.residualMm = float(ct.residual);
                    break;
                }
            }
        }
        recorder_.writePoints3D(refTsNs, rp);

        // ---- 块 27：追踪器逐点内部状态 ----
        // 【"编号乱跳"唯一能查的地方】Point3DRec 只看得到编号变了，看不到
        // 是门控没兜住、协方差塌了、还是关联被邻近点抢走了 —— 三者修法完全
        // 不同（放宽 maxAssocDist / 抬高 obsVarFloor / 开全局最优指派）。
        {
            const auto& td = useIekf ? iekfTracker_.lastFrameDebug()
                                     : tracker_.lastFrameDebug();
            std::vector<mocap::pcrec::TrackDbgItem> ti;
            ti.reserve(td.items.size());
            for (const auto& s : td.items) {
                mocap::pcrec::TrackDbgItem d;
                d.x = float(s.pos[0]); d.y = float(s.pos[1]); d.z = float(s.pos[2]);
                d.vx = float(s.vel[0]); d.vy = float(s.vel[1]); d.vz = float(s.vel[2]);
                d.posVarTrace = float(s.posVarTrace);
                d.velVarTrace = float(s.velVarTrace);
                d.assocDistMm = float(s.assocDistMm);
                d.assocMahaSq = float(s.assocMahaSq);
                d.qBoost      = float(s.qBoost);
                d.residualMm  = float(s.residualMm);
                d.id          = s.id;
                d.obsIndex    = s.obsIndex;
                d.missedFrames = int16_t(std::clamp(s.missedFrames, -32768, 32767));
                d.hits         = int16_t(std::clamp(s.hits, -32768, 32767));
                d.justAcquired = uint8_t(s.justAcquired);
                d.coasting     = uint8_t(s.coasting);
                d.confirmed    = uint8_t(s.confirmed);
                d.gateBypassed = uint8_t(s.gateBypassed);
                ti.push_back(d);
            }
            recorder_.writeTrackDbg(refTsNs, ti);
        }

        // ---- 块 28：3D 点 <-> 2D 光斑的对应关系 ----
        // 【这条线索本来就在手边，只是被丢掉了】Track3D::support 是一串
        // (相机下标, 观测下标)，而块 6 只把它压成了 supportMask 位图 ——
        // 相机知道了，是那台相机的第几个光斑却没了。
        // 要证实"两球粘连"必须能从一个可疑的 3D 点【反查】到构成它的那几个
        // 2D 光斑，再去看那些光斑的面积和外接框长宽比（块 1 里已经有了）。
        // 没有这个下标，这条链就断在中间，粘连只能靠猜。
        {
            std::vector<std::vector<std::pair<int,int>>> sup;
            sup.reserve(cluster.tracks.size());
            for (const auto& t2 : cluster.tracks) sup.push_back(t2.support);
            std::vector<std::pair<int,int>> unm;
            for (size_t c = 0; c < cluster.unmatched.size(); ++c)
                for (int oi : cluster.unmatched[c]) unm.push_back({int(c), oi});
            recorder_.writeClusterLink(refTsNs, int(Rs.size()), sup, unm);
        }

        // ---- 块 22：时基质量与分阶段耗时 ----
        // 【dt 是滤波/限幅/外推三处的分母】它不可信时这三处算出来的全不可信，
        // 而输出看起来完全正常，只是慢了、飘了 —— 跟滤波调过头一模一样。
        // camTsSpread 是多相机系统的头号隐患：各相机不同步时同一"帧"里的光斑
        // 其实来自不同时刻，三角化出来的点在手快速运动时会系统性偏移，
        // 而这个偏移随速度变化，看起来就像"动起来就不准"，跟标定误差很像。
        {
            mocap::pcrec::TimingRec tm;
            tm.wallNs = QDateTime::currentMSecsSinceEpoch() * 1000000LL;
            tm.frameTsNs = refTsNs;
            // 【不用 INT64_MAX】那个宏来自 <cstdint>，这里是间接引入的，
            // MSVC 上不保证可见。用 numeric_limits 是同一个值且一定在。
            int64_t tmin = std::numeric_limits<int64_t>::max();
            int64_t tmax = std::numeric_limits<int64_t>::lowest();
            int nc = 0;
            double sum = 0.0, sum2 = 0.0;
            // 【直接用本帧的 camTsNs】它就在这个函数里，是各相机各自的采集
            // 时间戳（refTsNs 是它们的均值）。离散度大 = 相机没同步。
            for (qint64 e : camTsNs) {
                if (e <= 0) continue;
                tmin = std::min(tmin, e); tmax = std::max(tmax, e);
                sum += double(e); sum2 += double(e) * double(e);
                ++nc;
            }
            if (nc >= 2) {
                tm.camTsSpreadUs = float(double(tmax - tmin) / 1000.0);
                const double mean = sum / nc;
                const double var = std::max(0.0, sum2 / nc - mean * mean);
                tm.camTsStdUs = float(std::sqrt(var) / 1000.0);
            }
            tm.nCamsThisFrame = nc;
            if (recLastFrameTsNs_ > 0 && refTsNs > recLastFrameTsNs_) {
                const double dtMs = double(refTsNs - recLastFrameTsNs_) / 1.0e6;
                tm.frameDtMs = float(dtMs);
                // dt 的滑动标准差。【抖动比均值重要】平均 8.3ms 而抖动 5ms 的
                // 流，跟稳定 8.3ms 的流，对滤波器是两回事。
                recDtMean_ = recDtMean_ * 0.95 + dtMs * 0.05;
                recDtVar_  = recDtVar_ * 0.95 + (dtMs - recDtMean_) * (dtMs - recDtMean_) * 0.05;
                tm.frameDtJitterMs = float(std::sqrt(std::max(0.0, recDtVar_)));
                if (dtMs > 1e-6) tm.fpsIn = float(1000.0 / dtMs);
            }
            recLastFrameTsNs_ = refTsNs;
            tm.wallMinusCamMs = float(double(tm.wallNs - refTsNs) / 1.0e6);
            tm.tClusterMs = float(clusterMsThisCall);
            tm.tTrackMs   = float(trackMsThisCall);
            tm.tTotalMs   = float(clusterMsThisCall + trackMsThisCall);
            // 骨架侧的耗时和跳帧从上一帧诊断快照里取（异步回来的，天然晚一拍）
            tm.tInferMs  = float(lastDiagSnapshot_.latencyMs);
            tm.tAssocMs  = float(lastDiagSnapshot_.tAssocMs);
            tm.tFilterMs = float(lastDiagSnapshot_.tFilterMs);
            tm.skelProcessed = lastDiagSnapshot_.skelProcessed;
            tm.skelSkipped   = lastDiagSnapshot_.skelSkipped;
            tm.recQueueDepth = int32_t(recorder_.queueDepth());
            tm.recDropped    = int32_t(recorder_.droppedChunks());
            recorder_.writeTiming(tm);
        }
    }

    // 【改动】骨骼叠加——开关关闭时完全不提交任何帧给worker(AI真正意义上
    // "不介入"，不是介入了只是不显示)；开关打开时只把这一帧候选点丢给
    // 独立线程，不等结果、不阻塞这里——点云面板该多流畅还是多流畅，画面
    // 上的骨架用的是上一次收到的结果，跟这一帧点云异步刷新，不同帧号也
    // 没关系(反正是独立叠加层，setSkeletonOverlay不影响setPoints这条路径，
    // 见PointCloudWidget的说明)。
    if (showSkeletonChk_->isChecked() && skeletonWorker_) {
        // 【限流到约30Hz——别改成每帧都投】骨架叠加是画给人看的，30Hz
        // 肉眼已经完全跟得上；而点云追踪本身可能跑到100~200fps。如果
        // 每帧都投递，AI线程就要按追踪帧率满负荷跑，白白吃掉数倍CPU，
        // 跟采集/检测/追踪线程抢核心，最后拖慢的是整个程序。限流之后
        // AI线程的负载直接降到原来的几分之一，视觉上完全看不出区别。
        // 【原来这里写死 33ms(30Hz)】理由是"30Hz 肉眼够用"——对"看骨架姿态对不对"
        // 成立，对"跟手感"不成立：点云自己跑 100fps+ 时，画面上就是点已经动了、
        // 线还停在最多 33ms 前的位置。现在由面板上的"骨架刷新"决定，0=不限流。
        const bool dueForDispatch = skelDispatchMinMs_ <= 0
                                    || !skeletonDispatchThrottle_.isValid()
                                    || skeletonDispatchThrottle_.elapsed() >= skelDispatchMinMs_;
        if (dueForDispatch) {
            if (skeletonWorker_->tryAcquireSlot()) {
                skeletonDispatchThrottle_.restart();
                SkeletonCandVec cands;
                cands.reserve(size_t(pts.size()));
                for (const auto& p : pts)
                    cands.push_back({p.id, {double(p.pos.x()), double(p.pos.y()), double(p.pos.z())}});
                ++skeletonSubmittedFrames_;
                // 投递时刻（单调时钟）。worker 同时最多处理一帧，所以结果回来时
                // 直接相减就是"提交 -> 结果回到 GUI"的真实耗时，不需要帧编号。
                dispatchNs_ = pipeClock_.isValid() ? pipeClock_.nsecsElapsed() : -1;
                QMetaObject::invokeMethod(skeletonWorker_, "processFrameAt", Qt::QueuedConnection,
                                          Q_ARG(SkeletonCandVec, cands),
                                          Q_ARG(qint64, refTsNs),
                                          Q_ARG(bool, true));   // 就绪与否由worker自己判断
            } else {
                // 到点了想投但worker还在忙上一帧——这才算真正的"丢帧"，
                // 说明AI处理速度跟不上30Hz。画面上保留上一次的骨架不动，
                // 比"卡住等它"或"硬塞进队列积压"都更不容易拖慢UI。
                // 注意：因为限流(dueForDispatch为false)主动跳过的帧【不】
                // 计入这里，否则追踪跑120fps时丢帧率会永远显示75%，这个
                // 诊断数字就失去意义了。
                ++skeletonDroppedFrames_;
            }
        }
    } else if (!showSkeletonChk_->isChecked()) {
        // 开关关闭：不提交新帧，并清掉已有骨架叠加。
        // 【只在"刚切换到关闭"那一次做】——这个分支在开关关着的时候每帧
        // 都会进来(而关着才是默认状态)，无条件每帧调 setSkeletonOverlay
        // (内部会 update() 请求重绘)+setText，纯属白烧GUI线程。用一个标志
        // 保证清理只做一次。
        // 用独立的 setSkeletonOverlay({},{})，不是 setEdges({})——后者是
        // "绑定模式"手动连线专用的存储，误用同一个接口会把用户手动连的线
        // 也一起清掉。
        if (skeletonOverlayActive_) {
            skeletonOverlayActive_ = false;
            view_->setSkeletonOverlay({}, {});
            // 【两处清空点都要清标签】否则骨架关掉后，上一次的标签会一直挂在
            // 点云上，指向早就换掉的追踪 id —— 看起来像"标错了"。
            view_->setPointLabels({});
            view_->setSkeletonLabels({});
            skeletonDiagLabel_->setText(QStringLiteral("骨骼AI：未启用"));
            skeletonDiagLabel_->setToolTip(QString());
            if (latencyLabel_) latencyLabel_->setText(QStringLiteral("滞后：—"));
            emaE2eMs_ = emaSkelIntervalMs_ = -1.0;   // 下次打开重新开始统计，别混上一段的数
            skeletonDiagLastKind_ = -1;   // 下次打开时立刻刷新状态，不被限流挡住
        }
    }

    int coasting = 0;
    for (const auto& tp : tracked) if (tp.missedFrames > 0) ++coasting;
    // 【新增】圆拟合法模式下附带分流统计。"跳过"长期不为0 = 画面里有轮廓
    // 过大的亮区被兜底闸挡下(那些点退回质心，没丢)，提示去检测参数面板调
    // 阈值/最大面积/圆度下限——以前这种情况只会表现为界面卡死，现在是一个
    // 能读的数字。
    QString fitInfo;
    if (useCircleFitChk_->isChecked()) {
        fitInfo = QStringLiteral("  [圆拟合分流：单球%1 / 花生%2 / 跳过%3]")
            .arg(lastFrameSingleBlobs_).arg(lastFramePeanutBlobs_).arg(lastFrameSkippedBlobs_);
        if (lastFrameSkippedBlobs_ > 0)
            fitInfo += QStringLiteral(" ⚠ 有轮廓过大的亮区，建议在检测参数里提高阈值/圆度下限");
    }
    status_->setText(QStringLiteral(
        "已连接 %1 台相机，本帧检测到 %2 个反光球观测 -> 聚类验证出 %3 个空间点"
        "（其中遮挡记忆中 %4 个，未验证/歧义丢弃 %5 组观测）%6")
        .arg(activeCams_.size()).arg(totalBlobs).arg(tracked.size()).arg(coasting)
        .arg(int(cluster.unmatched.size())).arg(fitInfo));

    // 【新增】编号稳定性统计——见头文件里 statsBaselineTotalIds_/
    // statsPeakActive_ 的说明。newIdsSinceReset：这段统计区间内一共冒出过
    // 多少个新编号(含已经又被销毁的)；peak：这段区间同时活跃过的confirmed
    // 编号数的峰值。比值接近1.0说明峰值这些点从头到尾基本没被重新编号；
    // 明显大于1说明同样这些物理点在反复重新分配编号，数字越大越不稳。
    // 两个后端都实现了同名的totalIdsAssigned()/activeCount()，这里不需要
    // 分支，直接按useIekf选用哪一个当前活跃的追踪器读数字。
    const int currentTotalIds = useIekf ? iekfTracker_.totalIdsAssigned() : tracker_.totalIdsAssigned();
    const int currentActive = useIekf ? iekfTracker_.activeCount() : tracker_.activeCount();
    const int newIdsSinceReset = currentTotalIds - statsBaselineTotalIds_;
    statsPeakActive_ = std::max(statsPeakActive_, currentActive);
    const double ratio = statsPeakActive_ > 0
        ? double(newIdsSinceReset) / double(statsPeakActive_)
        : 0.0;
    const QString backendLabel = useIekf
        ? QStringLiteral("IEKF卡尔曼")
        : (useHungarianChk_->isChecked() ? QStringLiteral("贪心/启发式-匈牙利") : QStringLiteral("贪心/启发式"));
    // 主干只留最能说明问题的两个数：比值（越接近 1.0 越稳）和峰值在线数。
    // 后端名、累计新增数这些"看一眼就够"的进 tooltip。
    setBriefText(stabilityLabel_,
        QStringLiteral("编号稳定 %1 ×  在线 %2")
            .arg(ratio, 0, 'f', 2).arg(statsPeakActive_),
        QStringLiteral(
            "稳定性统计（%1）\n"
            "  累计新增编号 %2 个\n"
            "  峰值同时在线 %3 个\n"
            "  新增/峰值 = %4（越接近 1.0 越稳；明显大于 1 说明同样这些\n"
            "  物理点在反复被重新编号）")
            .arg(backendLabel).arg(newIdsSinceReset).arg(statsPeakActive_).arg(ratio, 0, 'f', 2));

    // 【新增】耗时统计——EMA平滑(alpha=0.15，兼顾"跟得上变化"和"不被单帧
    // 噪声带偏")，峰值不平滑(峰值就该是峰值，平滑了就看不出偶发的卡顿)。
    // renderMs不计入(view_->setPoints()已经在上面调用过了，Qt的实际重绘
    // 发生在事件循环稍后，这里量不准，干脆不测，避免一个不准的数字造成
    // 误导)。
    const double totalMsThisCall = double(totalTimer.nsecsElapsed()) / 1.0e6;
    const double alpha = 0.15;
    auto ema = [alpha](double& acc, double sample) { acc = (acc <= 0.0) ? sample : (alpha*sample + (1.0-alpha)*acc); };
    ema(emaBuildMs_, buildMsThisCall);
    ema(emaClusterMs_, clusterMsThisCall);
    ema(emaTrackMs_, trackMsThisCall);
    ema(emaTotalMs_, totalMsThisCall);
    maxTotalMs_ = std::max(maxTotalMs_, totalMsThisCall);
    if (intervalMsThisCall >= 0.0) ema(emaIntervalMs_, intervalMsThisCall);
    if (arrivalSpreadMsThisCall >= 0.0) {
        ema(emaArrivalSpreadMs_, arrivalSpreadMsThisCall);
        // 滑动窗口，不再累积全局最大值。见头文件里 arrivalSpreadHist_ 的说明。
        arrivalSpreadHist_.push_back(arrivalSpreadMsThisCall);
        while (int(arrivalSpreadHist_.size()) > kArrivalHistN) arrivalSpreadHist_.pop_front();
        if (arrivalSpreadHist_.size() >= 20) {
            std::vector<double> v(arrivalSpreadHist_.begin(), arrivalSpreadHist_.end());
            const size_t k = size_t(v.size() * 95 / 100);
            std::nth_element(v.begin(), v.begin() + k, v.end());
            p95ArrivalSpreadMs_ = v[k];
            maxArrivalSpreadMs_ = *std::max_element(v.begin(), v.end());
        }
    }

    // 判断"是不是在丢帧"的粗略信号：处理总耗时如果经常逼近甚至超过实测
    // 的帧间隔本身，说明批处理定时器还没到点、下一批观测就已经把
    // latestBlobs_覆盖掉了——这不是精确证明，但足够当一个"该不该继续往
    // 下查"的信号灯。
    const bool possiblyStarved = emaIntervalMs_ > 0.0 && emaTotalMs_ > emaIntervalMs_ * 0.7;
    // 批处理窗口是否合理。判据换成【最近 10 秒的 95 分位】，不再用全局峰值。
    //
    // 两个方向都判，因为设太大同样有代价：批处理定时器要等满窗口才出帧，
    // 窗口开到 50ms 就等于给整条链路白加 50ms 延迟。原来只提示"太紧"，
    // 用户为了消掉警告会一路往大调，反而把延迟做上去了。
    const int    winMs = batchWindowSpin_->value();
    const double p95   = p95ArrivalSpreadMs_;
    const bool haveStat = arrivalSpreadHist_.size() >= 20;
    // 太紧：95% 的帧都要求窗口至少这么宽，留 1.4 倍余量还不够就是真紧
    const bool windowTooTight = haveStat && p95 > 0.0 && winMs < p95 * 1.4;
    // 太松：p95 只用得上窗口的三分之一，多出来的部分是纯延迟
    const bool windowTooLoose = haveStat && p95 > 0.0 && winMs > p95 * 3.0 + 4.0;
    // 建议值：覆盖 95 分位并留 1.5 倍余量，钳在 spin 的范围内
    const int suggestMs = haveStat && p95 > 0.0
        ? std::clamp(int(std::ceil(p95 * 1.5)) + 1, 1, 100) : winMs;

    // 【窗口的帧率代价】只说"窗口合理"是不够的。批处理定时器要等满窗口才出帧，
    // 所以输出帧率会被窗口量化成相机帧率的 1/N。真机上见过窗口 12ms、
    // 相机 120fps、输出只有 60fps —— 一半的相机帧被丢掉了，而当时的提示
    // 只显示"✓ 窗口合理"，用户完全看不出自己在付这个代价。
    const double camFps = (emaCamPeriodMs_ > 0.5) ? 1000.0 / emaCamPeriodMs_ : 0.0;
    const double outFps = (emaIntervalMs_   > 0.5) ? 1000.0 / emaIntervalMs_   : 0.0;
    const bool rateLoss = camFps > 1.0 && outFps > 1.0 && outFps < camFps * 0.7;
    // 主干只留"总耗时 + 输入输出帧率"，其余（分阶段耗时、到达偏差、
    // 批处理窗口建议）都是排查时才看的，进 tooltip。
    // 【警告要保留在主干】⚠ 那几条是"现在正在出问题"，藏进 tooltip 等于没有。
    QString warnBrief;
    if (possiblyStarved)  warnBrief += QStringLiteral("  ⚠可能丢帧");
    if (windowTooTight)   warnBrief += QStringLiteral("  ⚠窗口太紧");
    else if (windowTooLoose) warnBrief += QStringLiteral("  ⚠窗口偏大");
    if (rateLoss)         warnBrief += QStringLiteral("  ⚠帧率被窗口吃掉");
    setBriefText(timingLabel_,
        QStringLiteral("耗时 %1ms   %2→%3fps%4")
            .arg(emaTotalMs_, 0, 'f', 2)
            .arg(camFps > 1.0 ? QString::number(camFps, 'f', 0) : QStringLiteral("—"))
            .arg(outFps > 1.0 ? QString::number(outFps, 'f', 0) : QStringLiteral("—"))
            .arg(warnBrief),
        QStringLiteral(
        "耗时统计：本帧总 %1ms（峰值 %2ms） | 帧间隔实测 %3ms | "
        "构建观测 %4ms / 聚类 %5ms / 追踪 %6ms%7\n"
        "相机到达偏差 %8ms（近10秒 p95 %9ms，批处理窗口 %10ms）%11\n"
        "帧率：相机 %12fps → 输出 %13fps%14")
        .arg(emaTotalMs_, 0, 'f', 2).arg(maxTotalMs_, 0, 'f', 2)
        .arg(emaIntervalMs_ > 0.0 ? QString::number(emaIntervalMs_, 'f', 2) : QStringLiteral("—"))
        .arg(emaBuildMs_, 0, 'f', 2).arg(emaClusterMs_, 0, 'f', 2).arg(emaTrackMs_, 0, 'f', 2)
        .arg(possiblyStarved ? QStringLiteral("  ⚠ 处理耗时接近/超过帧间隔，可能在静默丢帧") : QString())
        .arg(emaArrivalSpreadMs_ > 0.0 ? QString::number(emaArrivalSpreadMs_, 'f', 2) : QStringLiteral("—"))
        .arg(haveStat ? QString::number(p95, 'f', 2) : QStringLiteral("—"))
        .arg(winMs)
        .arg(windowTooTight
                 ? QStringLiteral("  ⚠ 窗口太紧，会漏掉还没到的相机 → 建议 %1ms").arg(suggestMs)
                 : (windowTooLoose
                        ? QStringLiteral("  ⚠ 窗口偏大，多出来的是纯延迟 → 建议 %1ms").arg(suggestMs)
                        : (haveStat ? QStringLiteral("  ✓ 窗口对齐相机没问题") : QString())))
        .arg(camFps > 1.0 ? QString::number(camFps, 'f', 0) : QStringLiteral("—"))
        .arg(outFps > 1.0 ? QString::number(outFps, 'f', 0) : QStringLiteral("—"))
        .arg(rateLoss
                 ? QStringLiteral("   ⚠ 窗口把 %1% 的相机帧吃掉了：定时器要等满 %2ms 才出帧，"
                                  "而相机每 %3ms 就来一轮。想提帧率只能缩窗口，"
                                  "而缩窗口的下限是到达偏差 %4ms —— 真正该压的是这个偏差。")
                       .arg(int((1.0 - outFps / camFps) * 100.0))
                       .arg(winMs)
                       .arg(emaCamPeriodMs_, 0, 'f', 1)
                       .arg(haveStat ? QString::number(p95, 'f', 1) : QStringLiteral("?"))
                 : QString()));   // <- 这里闭合的是 setBrief 的第二个参数(全文)
}

} // namespace mocap
