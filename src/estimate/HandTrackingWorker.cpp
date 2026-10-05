#include "estimate/HandTrackingWorker.hpp"
#include "camera/ICamera.hpp"
#include <QDebug>
#include <QMetaObject>
#include <algorithm>
#include <cstdlib>

namespace mocap {

HandTrackingWorker::HandTrackingWorker(const QVector<ICamera*>& cams, CalibrationStore* calibStore,
                                       HandTemplateStore* templateStore, QObject* parent)
    : QObject(parent), cams_(cams), calibStore_(calibStore), templateStore_(templateStore) {
    for (ICamera* c : cams_) {
        if (!c || !calibStore_) continue;
        const QString key = c->deviceKey();
        const CameraCalibration calib = calibStore_->get(key);
        if (!calib.isCalibrated()) continue;

        camIds_.push_back(c->id());
        calibs_.push_back(calib);

        CamPose pose;
        pose.R = calib.extr.R;
        pose.t = calib.extr.t;
        camPoses_.push_back(pose);

        c->setContourCollectionEnabled(true);
        connect(c, &ICamera::blobDetailsReady, this, &HandTrackingWorker::onBlobDetails);
    }

    reloadTemplate();   // 首次构造也走一遍，用 templateStore_ 里已有的(标定过的或占位的)数据建 fk_
    rebuildPipeline();
    setSmoothingParams(posMinCutoff_, posBeta_, rotMinCutoff_, rotBeta_, jointMinCutoff_, jointBeta_);
    // ↑ jointFilters_/quatFilters_数组初始化时用的是OneEuroFilter自己的
    // 默认构造参数，跟上面几个成员变量声明的默认值不一定一致，这里显式
    // 同步一次，避免"代码读起来默认是0.3，实际跑起来却是构造函数默认值"
    // 这种隐藏的不一致。

    pendingTimer_ = new QTimer(this);
    pendingTimer_->setSingleShot(true);
    connect(pendingTimer_, &QTimer::timeout, this, &HandTrackingWorker::onTimerFire);
}

void HandTrackingWorker::rebuildPipeline() {
    ColdStartFn coldStart = [this](const std::vector<CameraFrame>& frames) {
        return coldStartFromFrames(frames);
    };
    pipeline_.emplace(fk_, kHandNumJoints, gateRadiusNorm_, coldStart, chiSquareGate_);
}

HandTrackingWorker::~HandTrackingWorker() {
    stopSmootherThread();
}

void HandTrackingWorker::startSmootherThreadIfNeeded() {
    if (smootherThread_) return;   // 已经建过了，不重复建

    smootherWorker_ = new SmootherWorker();   // 不给parent——马上要moveToThread，Qt不允许有parent的对象跨线程移动
    smootherThread_ = new QThread(this);
    smootherWorker_->moveToThread(smootherThread_);

    // configure()必须在线程start()之前、还在当前(GUI)线程上调用——见
    // SmootherWorker::configure()注释，这不是线程安全地随时可调的方法。
    smootherWorker_->configure(fk_, kHandNumJoints, camPoses_, smootherCfg_);

    connect(smootherWorker_, &SmootherWorker::smoothedFrameReady, this, &HandTrackingWorker::handPoseSmoothedReady);
    smootherThread_->start();
}

void HandTrackingWorker::stopSmootherThread() {
    if (!smootherThread_) return;
    // 跟 WebcamCamera::stop() 清理 detectThread_/detectWorker_ 是同一套
    // 写法：quit()+wait()确保线程真的跑完了再delete，不用deleteLater那套
    // (那套更适合"不知道调用方在哪个线程调用stop"的场景，这里stop永远
    // 在GUI线程调用，quit()+wait()之后直接delete更直接、跟项目里已有的
    // 先例保持一致)。
    smootherThread_->quit();
    smootherThread_->wait();
    delete smootherWorker_; smootherWorker_ = nullptr;
    delete smootherThread_; smootherThread_ = nullptr;
}

void HandTrackingWorker::setSmootherEnabled(bool on) {
    smootherEnabled_ = on;
    if (on) startSmootherThreadIfNeeded();
    else stopSmootherThread();
}

void HandTrackingWorker::setSmootherConfig(const SmootherConfig& cfg) {
    smootherCfg_ = cfg;
    if (smootherEnabled_) {
        // 窗口配置变了，重建这条线程(比重新发一份configure()简单可靠——
        // 窗口大小变了，旧窗口里攒的帧数、内部矩阵维度都跟新配置对不上，
        // 直接重开一条干净的线程/worker，比试图"原地重配置"更不容易出错，
        // 而且这个操作不频繁，重建的代价可以接受)。
        stopSmootherThread();
        startSmootherThreadIfNeeded();
    }
}

void HandTrackingWorker::setGateRadius(double gateRadiusNorm) {
    gateRadiusNorm_ = gateRadiusNorm;
    rebuildPipeline();   // gateRadius是构造时定死的，只能整个重建；当前追踪状态会丢失，下一帧重新冷启动
}

void HandTrackingWorker::setSmoothingParams(double posMinCutoff, double posBeta,
                                            double rotMinCutoff, double rotBeta,
                                            double jointMinCutoff, double jointBeta) {
    posMinCutoff_ = posMinCutoff; posBeta_ = posBeta;
    rotMinCutoff_ = rotMinCutoff; rotBeta_ = rotBeta;
    jointMinCutoff_ = jointMinCutoff; jointBeta_ = jointBeta;

    posFilter_.setParams(posMinCutoff_, posBeta_);
    for (auto& f : quatFilters_) f.setParams(rotMinCutoff_, rotBeta_);
    for (auto& f : jointFilters_) f.setParams(jointMinCutoff_, jointBeta_);
}

void HandTrackingWorker::resetSmoothingFilters() {
    posFilter_.reset();
    for (auto& f : quatFilters_) f.reset();
    for (auto& f : jointFilters_) f.reset();
    haveLastQuat_ = false;
}

// 标准的行主序3x3旋转矩阵->四元数(w,x,y,z)转换(Shepperd法，按迹的符号
// 选数值最稳的分支，避免对角元素接近-1时开方精度爆炸)。已经用2000个
// 随机旋转矩阵验证过往返转换误差在机器精度级别，不是拍脑袋写的公式。
std::array<double,4> HandTrackingWorker::matToQuat(const std::array<double,9>& R) {
    const double m00=R[0], m01=R[1], m02=R[2];
    const double m10=R[3], m11=R[4], m12=R[5];
    const double m20=R[6], m21=R[7], m22=R[8];
    const double trace = m00+m11+m22;
    double qw,qx,qy,qz;
    if (trace > 0.0) {
        const double s = std::sqrt(trace+1.0)*2.0;
        qw = 0.25*s; qx=(m21-m12)/s; qy=(m02-m20)/s; qz=(m10-m01)/s;
    } else if (m00>m11 && m00>m22) {
        const double s = std::sqrt(1.0+m00-m11-m22)*2.0;
        qw=(m21-m12)/s; qx=0.25*s; qy=(m01+m10)/s; qz=(m02+m20)/s;
    } else if (m11>m22) {
        const double s = std::sqrt(1.0+m11-m00-m22)*2.0;
        qw=(m02-m20)/s; qx=(m01+m10)/s; qy=0.25*s; qz=(m12+m21)/s;
    } else {
        const double s = std::sqrt(1.0+m22-m00-m11)*2.0;
        qw=(m10-m01)/s; qx=(m02+m20)/s; qy=(m12+m21)/s; qz=0.25*s;
    }
    return {qw,qx,qy,qz};
}

std::array<double,9> HandTrackingWorker::quatToMat(const std::array<double,4>& q) {
    double w=q[0], x=q[1], y=q[2], z=q[3];
    const double n = std::sqrt(w*w+x*x+y*y+z*z);
    if (n > 1e-12) { w/=n; x/=n; y/=n; z/=n; }
    return {
        1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w),
        2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w),
        2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)
    };
}

void HandTrackingWorker::reloadTemplate() {
    if (!templateStore_) {
        fk_ = makeHandForwardKinematics();
    } else {
        const auto& data = templateStore_->data();
        fk_ = makeHandForwardKinematicsFromTemplate(data.backMarkers, data.fingerParams);

        // 【这是"手指之间串扰"最直接的成因，不是猜的】gateRadiusNorm_之前
        // 是跟这只手的贴球方案完全无关的固定常量(0.05)——相邻手指的MCP
        // marker(比如食指和中指的掌骨关节球)实测间距可能只有一两厘米，
        // 固定0.05在常见工作距离下换算出来的实际容许间距经常比这个还大，
        // 门控形同虚设，容易关联错位。模板标定完(isComplete())之后，用
        // 真实几何重新推算一次门控半径。
        //
        // 【这是上一版的教训，别再犯】门控半径要同时满足两个互相拉扯的
        // 约束：够紧才能防跨手指串扰，又要够松才能兜住正常追踪时FK预测
        // 本身的误差(检测噪声、标定残余误差、快速运动下的滞后)——上一版
        // 直接采信GateRadiusEstimator默认marginFraction=0.4算出的值(只留
        // 最近marker间距的40%当门控半径)，只顾了防串扰这一头，没有反过来
        // 检查这么紧的门控还留不留得出追踪预测误差的余量。旧的固定值0.05
        // 虽然会串扰，但至少"能追踪"——这次改用marginFraction=0.7(少拿
        // 走点安全边际)，并且不允许自动推算的值低于kGateRadiusAutoFloor
        // 这个地板(比旧默认值0.05略紧，但不会紧到上一版0.4那么激进)。如果
        // 几何分析算出的"防串扰安全值"比这个地板还紧，说明这只手/这套
        // 贴球方案的marker间距本身偏小，门控半径可能怎么调都两头不讨好
        // (要么松了串扰、要么紧了追不上)——这种情况只在日志里报警，不
        // 静默地把追踪用的门控收到可能导致"基本不可用"的程度，让用户
        // 知道这是需要正视的真实矛盾，不是调个参数就能两全的。
        if (data.isComplete()) {
            constexpr double kGateRadiusMarginFraction = 0.7;   // 从0.4放宽到0.7，少拿走点安全边际给追踪噪声留余量
            // 地板不再是拍脑袋的固定占位值——如果调用方通过
            // setMeasuredMinMarkerSpacingMm() 提供了真实卡尺量出来的最小
            // marker间距，用同一套公式(间距×安全折扣/工作距离)从这个真实
            // 数字算出地板；没提供才退回旧的0.045占位值兜底(那是基于占位
            // 人体测量学表估的，不代表你这只手，只是"没有更好数据时"的
            // 保守选择)。
            constexpr double kGateRadiusAutoFloorFallback = 0.045;
            const double gateRadiusAutoFloor = (measuredMinMarkerSpacingMm_ > 0.0)
                ? (measuredMinMarkerSpacingMm_ * kGateRadiusMarginFraction / nominalWorkingDistanceMm_)
                : kGateRadiusAutoFloorFallback;
            const auto est = estimateGateRadiusFromTemplate(nominalWorkingDistanceMm_, kGateRadiusMarginFraction);
            if (est.valid) {
                const double applied = std::max(est.recommendedGateRadiusNorm, gateRadiusAutoFloor);
                gateRadiusNorm_ = applied;
                if (est.recommendedGateRadiusNorm < gateRadiusAutoFloor) {
                    qWarning().noquote() << QStringLiteral(
                        "[HandTracking] 门控半径几何分析建议值(%1)比安全地板(%2%3)还紧——"
                        "这只手/这套贴球方案的marker间距可能偏小，实际门控半径用地板值%2"
                        "(没有直接采信更紧的推算值)。如果追踪时仍有跨手指串扰，这是真实存在"
                        "的marker间距矛盾，不是这个门控参数能单独解决的，需要考虑贴球方案本身。%4")
                        .arg(est.recommendedGateRadiusNorm).arg(gateRadiusAutoFloor)
                        .arg(measuredMinMarkerSpacingMm_ > 0.0
                                 ? QStringLiteral("，按实测最小间距%1mm算出").arg(measuredMinMarkerSpacingMm_)
                                 : QStringLiteral("，占位值兜底，建议用setMeasuredMinMarkerSpacingMm()提供真实测量值"))
                        .arg(QString::fromStdString(est.message));
                } else {
                    qInfo().noquote() << QStringLiteral("[HandTracking] 门控半径已按标定模板自动更新：%1（实际采用 %2）")
                        .arg(QString::fromStdString(est.message)).arg(applied);
                }
            } else {
                qWarning().noquote() << QStringLiteral("[HandTracking] 门控半径自动推算失败，沿用当前值(%1)：%2")
                    .arg(gateRadiusNorm_).arg(QString::fromStdString(est.message));
            }
        }
    }

    // 【必须重建】fk_ 只是这个类的成员，pipeline_ 和它内部的 associator_ 在
    // 构造时各自**按值拷贝**了一份 fk_(见 HandTrackingPipeline 构造函数：
    // fk_(fk) + associator_(fk, ...))。所以光把成员 fk_ 换掉，正在跑的实时
    // 追踪流里那两份拷贝纹丝不动，还在用旧模板——表现出来就是"标定向导明明
    // 保存成功了，追踪效果一点没变"，极容易被误判成算法问题去调滑块。
    // 重建会丢当前追踪状态、下一帧重新冷启动，但标定刚完成时本来就该拿新
    // 模板重新冷启动，这正是期望行为。
    rebuildPipeline();

    // 延迟精修流同理：SmootherWorker 的 fk 是在 startSmootherThreadIfNeeded()
    // 里 configure() 时一次性传进去的，模板变了必须重配。configure() 不是
    // 线程安全的随时可调方法(见 SmootherWorker::configure 注释)，所以这里
    // 走"停掉再重建"这条已经验证过的路径，跟 setSmootherConfig() 一致。
    if (smootherEnabled_) {
        stopSmootherThread();
        startSmootherThreadIfNeeded();
    }
}

void HandTrackingWorker::resetTracking() {
    if (pipeline_) pipeline_->reset();
    hadStateLastFrame_ = false;   // 下次重新拿到状态时，processFrame会据此重置平滑滤波器
    if (smootherWorker_) QMetaObject::invokeMethod(smootherWorker_, "resetWindow", Qt::QueuedConnection);
}

void HandTrackingWorker::onBlobDetails(quint32 camId, const QVector<Blob>& blobs, qint64 ts_ns) {
    bool tracked = false;
    for (quint32 id : camIds_) if (id == camId) { tracked = true; break; }
    if (!tracked) return;

    QVector<CamBuf>& hist = history_[camId];
    // 实测帧间隔EWMA——用这一帧到达前history里最后一帧的时间戳算间隔，
    // 在push新帧之前算，不然就跟自己比了。第一次(hist为空)没有上一帧可比，
    // 跳过，等第二帧来了自然就有了，不需要特殊初始化成某个假定值。
    if (!hist.isEmpty()) {
        const double dt = double(ts_ns - hist.back().ts);
        if (dt > 0) {
            double& avg = avgIntervalNs_[camId];
            avg = (avg <= 0.0) ? dt : (kIntervalEwmaAlpha * dt + (1.0 - kIntervalEwmaAlpha) * avg);
        }
    }
    hist.push_back({blobs, ts_ns});
    while (hist.size() > kHistoryDepth) hist.pop_front();   // 只留最近几帧，够最近邻搜索的余量就行，不需要无限攒

    if (!pendingTimer_->isActive()) pendingTimer_->start(kBatchWindowMs);
}

void HandTrackingWorker::onTimerFire() {
    qint64 maxTs = -1;
    for (auto it = history_.constBegin(); it != history_.constEnd(); ++it)
        if (!it.value().isEmpty() && it.value().back().ts > maxTs) maxTs = it.value().back().ts;
    if (maxTs < 0) return;
    processFrame(maxTs);
}

// 容差从实测数据算，不是写死的常量——取全部相机里"实测帧间隔最大"(跑得
// 最慢)的那一个当基准，乘1.2当容差。
// 【这是教训，别再犯】没有硬件同步码时，多台USB相机的到达时刻抖动比想
// 象的大——尤其几台相机共用同一个USB hub带宽时，正常情况下抖动就可能
// 到帧间隔的一半甚至更多，不是异常。这个容差最早定的0.6倍偏紧，只顾了
// "防止拿明显过期的数据凑数"这一头，没有反过来验证这么紧的容差会不会
// 让大量正常的USB到达抖动也被误判成"太旧"而被排除——一旦大部分帧只剩
// 1、2台相机能用(clusterMultiView的三角化通常需要至少2~3台)，三角化会
// 频繁失败，表现出来就是追踪断断续续甚至基本不可用。改成1.2倍(允许比
// 一个完整帧周期还宽松一点)，宁可容差松一点、多接纳一点时间上不完全
// 精确的观测，也不要让正常抖动被系统性地当成异常排除掉——这跟
// matchSyncedSnapshot本身"排除真正过期数据"的设计初衷不矛盾，只是把
// "正常抖动"和"真正过期"这两者的分界线往后挪，不要挪得太激进。
qint64 HandTrackingWorker::currentSyncToleranceNs() const {
    double maxIntervalNs = 0.0;
    for (auto it = avgIntervalNs_.constBegin(); it != avgIntervalNs_.constEnd(); ++it)
        maxIntervalNs = std::max(maxIntervalNs, it.value());
    if (maxIntervalNs <= 0.0) return kSyncToleranceFloorNs;   // 还没攒够样本(比如刚启动/只到过一帧)，用地板兜底
    const qint64 tol = qint64(maxIntervalNs * 1.2);
    return std::clamp(tol, kSyncToleranceFloorNs, kSyncToleranceCeilNs);
}

// 【最近邻时间戳配帧，取代之前"各相机各自最新"的硬凑】对目标时刻targetTs，
// 每台相机从自己最近的 kHistoryDepth 帧历史里找时间戳最接近的那一帧；
// 差距超过当前自适应容差(currentSyncToleranceNs())就不采纳，让这台相机
// 这一帧空缺——好过拿一帧明显不同步的数据滥竽充数(拿相机A卡顿15ms前的
// 旧观测硬当成跟相机B同一时刻，三角化会用上事实上不对齐的两条视线，误差
// 比"这一帧这台相机缺席"更难查)。没有硬件同步码的前提下，这是纯软件层面
// 能做到的最接近"真正同步"的配帧方式。
QHash<quint32, HandTrackingWorker::CamBuf> HandTrackingWorker::matchSyncedSnapshot(qint64 targetTs) const {
    const qint64 tol = currentSyncToleranceNs();
    QHash<quint32, CamBuf> out;
    for (auto it = history_.constBegin(); it != history_.constEnd(); ++it) {
        const QVector<CamBuf>& hist = it.value();
        if (hist.isEmpty()) continue;
        int bestIdx = 0; qint64 bestDiff = std::abs(hist[0].ts - targetTs);
        for (int i = 1; i < hist.size(); ++i) {
            const qint64 diff = std::abs(hist[i].ts - targetTs);
            if (diff < bestDiff) { bestDiff = diff; bestIdx = i; }
        }
        if (bestDiff <= tol) out[it.key()] = hist[bestIdx];
        // 超容差：这台相机在这次配帧里干脆不出现在out里，下游按"这台相机
        // 这一帧没有观测"处理，语义上跟它这一刻真的没检测到东西是一样的，
        // 不需要额外的"缺席"标记。
    }
    return out;
}

double HandTrackingWorker::estimateFallbackRadiusPx(size_t camIdx) const {
    const double fx = calibs_[camIdx].intr.fx;
    if (pipeline_ && pipeline_->hasState()) {
        const auto& st = pipeline_->state();
        const auto& R = camPoses_[camIdx].R; const auto& t = camPoses_[camIdx].t;
        const double Zc = R[6]*st.wristPos[0] + R[7]*st.wristPos[1] + R[8]*st.wristPos[2] + t[2];
        if (Zc > 50.0) return physicalMarkerRadiusMm_ * fx / Zc;
    }
    return physicalMarkerRadiusMm_ * fx / nominalWorkingDistanceMm_;
}

std::vector<Vec3> HandTrackingWorker::computeCoarseDepthPoints(const QHash<quint32, CamBuf>& latestSnapshot) const {
    // 每台相机的原始质心(去畸变归一化坐标)——完全不需要知道半径，flood
    // fill 的加权质心本来就跟"这个亮斑该拟合成多大的圆"没有关系。
    std::vector<std::vector<std::array<double,2>>> obsPerCam(camPoses_.size());
    for (size_t ci = 0; ci < camPoses_.size(); ++ci) {
        const auto it = latestSnapshot.constFind(camIds_[ci]);
        if (it == latestSnapshot.constEnd()) continue;
        for (const Blob& blob : it.value().blobs) {
            double nx, ny;
            undistortNormalize(calibs_[ci].intr, double(blob.cx), double(blob.cy), nx, ny);
            obsPerCam[ci].push_back({nx, ny});
        }
    }

    std::vector<EpiMat3> Rs; std::vector<EpiVec3> ts;
    Rs.reserve(camPoses_.size()); ts.reserve(camPoses_.size());
    for (const auto& cp : camPoses_) { Rs.push_back(cp.R); ts.push_back(cp.t); }

    // 阈值用 coarse 那组(比 refined 松)——质心噪声比精修后的圆心噪声大，
    // 这一遍的目标只是"够用的粗略深度"，不是最终结果，不需要卡得很紧。
    const auto cluster = clusterMultiView(Rs, ts, obsPerCam, coarseMaxSampson_, coarseMaxReprojNorm_);
    std::vector<Vec3> points;
    points.reserve(cluster.tracks.size());
    for (const auto& tr : cluster.tracks) points.push_back(tr.point);
    return points;
}

double HandTrackingWorker::estimateRefinedRadiusPx(size_t camIdx, const Blob& blob,
                                                   const std::vector<Vec3>& coarseDepthPoints, bool* matchedOut) const {
    double nx, ny;
    undistortNormalize(calibs_[camIdx].intr, double(blob.cx), double(blob.cy), nx, ny);

    RfMat3 R; RfVec3 t;
    for (int i=0;i<9;++i) R[size_t(i)] = camPoses_[camIdx].R[size_t(i)];
    for (int i=0;i<3;++i) t[size_t(i)] = camPoses_[camIdx].t[size_t(i)];

    std::vector<RfVec3> rfPoints;
    rfPoints.reserve(coarseDepthPoints.size());
    for (auto& p : coarseDepthPoints) rfPoints.push_back({p[0], p[1], p[2]});

    const auto result = refineExpectedRadius(nx, ny, R, t, calibs_[camIdx].intr.fx,
                                             rfPoints, physicalMarkerRadiusMm_, radiusRefineMatchGateNorm_);
    if (matchedOut) *matchedOut = result.matched;
    if (result.matched) return result.expectedRadiusPx;
    return estimateFallbackRadiusPx(camIdx);   // 粗算这一步没匹配上，退回旧的兜底假设
}

void HandTrackingWorker::processFrame(qint64 ts_ns) {
    currentTs_ = ts_ns;

    // 先按最近邻时间戳给每台相机挑一帧(容差内)，后面这一整帧处理都用这份
    // "已配对好"的快照，不再直接摸 history_——避免中途别的相机又来了新
    // 数据、这一帧处理到一半基准悄悄变了。
    const auto snapshot = matchSyncedSnapshot(ts_ns);

    // ---- 第一遍：radius-free 粗算深度。----
    const auto coarseDepthPoints = computeCoarseDepthPoints(snapshot);

    // ---- 第二遍：每个blob各自反推期望半径，重新做3a~3d精修。----
    std::vector<CameraFrame> frames;
    frames.reserve(camPoses_.size());

    QVector<int> matchedPerCam(int(camPoses_.size()), 0);
    QVector<int> fallbackPerCam(int(camPoses_.size()), 0);

    for (size_t ci = 0; ci < camPoses_.size(); ++ci) {
        CameraFrame cf;
        cf.camIndex = int(ci);
        cf.cam = &camPoses_[ci];

        const auto it = snapshot.constFind(camIds_[ci]);
        int blobCount = 0, contourSum = 0, obsCount = 0;
        double lastKnownRadiusPx = -1.0;
        if (it != snapshot.constEnd()) {
            blobCount = int(it.value().blobs.size());
            for (const Blob& blob : it.value().blobs) {
                contourSum += int(blob.contour.size());

                bool matched = false;
                const double knownRadiusPx = estimateRefinedRadiusPx(ci, blob, coarseDepthPoints, &matched);
                if (matched) ++matchedPerCam[int(ci)]; else ++fallbackPerCam[int(ci)];
                lastKnownRadiusPx = knownRadiusPx;

                BlobObservationConfig cfg;
                cfg.knownRadiusPx = knownRadiusPx;
                cfg.mode = localizationMode_;
                cfg.centroidSigmaPx = centroidSigmaPx_;
                auto obs = blobToObservations(blob, calibs_[ci].intr, cfg);
                obsCount += int(obs.size());
                for (auto& d : obs) cf.detections.push_back(d);
            }
        }
        if (verboseLogging_) {
            qDebug().noquote() << QStringLiteral(
                "[HandTracking] cam#%1(id=%2): blob=%3 轮廓点总数=%4 半径(末个blob)=%5px fx=%6 -> 观测数=%7 [粗算候选点=%8]")
                .arg(int(ci)).arg(camIds_[ci]).arg(blobCount).arg(contourSum)
                .arg(lastKnownRadiusPx, 0, 'f', 2).arg(calibs_[ci].intr.fx, 0, 'f', 1).arg(obsCount)
                .arg(int(coarseDepthPoints.size()));
        }
        frames.push_back(cf);
    }

    emit radiusRefineStatsReady(int(coarseDepthPoints.size()), matchedPerCam, fallbackPerCam, ts_ns);

    if (candidateBroadcastOn_) {
        const auto candidates = computeCandidatePoints(frames);

        // 喂进 candidateTracker_ 拿跨帧稳定ID——见 candidatePointsReady
        // 信号注释，UI靠这个ID记住"用户选的是哪个点"，不能再用数组下标。
        // Track3D 的 verified/residual 这两个字段在这里已经被
        // computeCandidatePoints 丢掉了(它只保留了 .point)，给默认值——
        // TemporalTracker 的关联逻辑本身只看位置+时间，不消费这两个字段，
        // 不影响跨帧身份追踪的正确性。
        std::vector<Track3D> tracksForId;
        tracksForId.reserve(candidates.size());
        for (const auto& p : candidates) {
            Track3D t; t.point = p; t.verified = true; t.residual = 0.0;
            tracksForId.push_back(t);
        }
        const auto tracked = candidateTracker_.update(tracksForId);

        QVector<QVector3D> qpts;
        QVector<int> ids;
        QVector<int> missed;
        qpts.reserve(int(tracked.size()));
        ids.reserve(int(tracked.size()));
        missed.reserve(int(tracked.size()));
        for (const auto& tp : tracked) {
            qpts.push_back(QVector3D(float(tp.position[0]), float(tp.position[1]), float(tp.position[2])));
            ids.push_back(tp.id);
            missed.push_back(tp.missedFrames);
        }

        const bool hasWrist = pipeline_ && pipeline_->hasState();
        QVector<double> rot9;
        QVector3D wpos;
        if (hasWrist) {
            const auto& st = pipeline_->state();
            rot9.resize(9);
            for (int i=0;i<9;++i) rot9[i] = st.wristRot[size_t(i)];
            wpos = QVector3D(float(st.wristPos[0]), float(st.wristPos[1]), float(st.wristPos[2]));
        }
        emit candidatePointsReady(qpts, ids, hasWrist, rot9, wpos, ts_ns, missed);
    }

    if (trackingMode_ == TrackingMode::BackRigidOnly) {
        processBackRigidOnlyFrame(frames, ts_ns);
        return;
    }

    const bool ok = pipeline_->step(frames, posProcessVar_, rotProcessVar_, jointProcessVar_);
    if (!ok || !pipeline_->hasState()) {
        emit handNotFound(ts_ns);
        hadStateLastFrame_ = false;
        return;
    }

    // 【这一版修的真实缺口】限位夹紧直接写回IEKF内部的名义状态，不是只
    // 夹"要送出去的那份拷贝"——之前的做法下，滤波器内部信念可能还在
    // 解剖学不可能的范围外，下一帧的关联/预测继续拿着这个越界状态滚，
    // 相当于允许误差累积。现在每次update()之后立刻夹一次，内部状态
    // 从这一刻起就是合法的，下一帧的FK线性化/marker投影都基于合法关节角。
    // (仍然是"只夹均值、不夹协方差"的朴素裁剪，不是严格约束EKF，见
    // HandStateIEKF::clampJointAngles 注释里的诚实局限说明。)
    pipeline_->clampJointAngles([](std::vector<double>& angles) {
        if (angles.size() != kHandNumJoints) return;
        std::array<double,16> q{};
        for (int i = 0; i < kHandNumJoints; ++i) q[size_t(i)] = angles[size_t(i)];
        const auto clamped = clampToLimits(q);
        for (int i = 0; i < kHandNumJoints; ++i) angles[size_t(i)] = clamped[size_t(i)];
    });

    // 延迟精修流(可选)——复用同一份已经关联好marker身份的观测
    // (pipeline_->lastMeasurements())，不重新关联一遍。跨线程只传camIndex
    // (数值)，不传HandCameraMeasurement里的裸指针，见SmootherWorker.hpp
    // 顶部完整分析。忙时(上一帧还没处理完)直接跳过这一帧，不排队——
    // 跟DetectWorker同一个"丢帧不积压"的原则。
    if (smootherEnabled_ && smootherWorker_ && !smootherWorker_->busy.load()) {
        const auto& lastMeas = pipeline_->lastMeasurements();
        QVector<SmootherMeasurementIn> measIn;
        measIn.reserve(int(lastMeas.size()));
        for (const auto& m : lastMeas) {
            if (!m.cam) continue;
            SmootherMeasurementIn mi;
            mi.camIndex = int(m.cam - camPoses_.data());   // camPoses_是std::vector，元素连续存储，指针减法能正确定位下标
            if (mi.camIndex < 0 || size_t(mi.camIndex) >= camPoses_.size()) continue;   // 万一cam指针不是指向camPoses_(理论上不该发生)，诚实丢弃这条而不是算出乱码下标
            mi.markerIndex = m.markerIndex;
            mi.nx = m.nx; mi.ny = m.ny; mi.sigma = m.sigma;
            measIn.push_back(mi);
        }
        QMetaObject::invokeMethod(smootherWorker_, "processFrame", Qt::QueuedConnection,
                                  Q_ARG(qint64, ts_ns), Q_ARG(HandPoseState, pipeline_->state()),
                                  Q_ARG(QVector<SmootherMeasurementIn>, measIn));
    }

    // 追踪状态从"丢失/冷启动中"变成"刚找到"——平滑滤波器不能拿丢失前的
    // 旧值去平滑一个全新的位姿(那样会把"跳变到新位置"这个正确行为也当成
    // "抖动"给平滑掉、造成一段虚假的滑行过渡)，这个瞬间必须重置全部滤波器。
    if (!hadStateLastFrame_) resetSmoothingFilters();
    hadStateLastFrame_ = true;

    const auto& st = pipeline_->state();

    // ---- 输出端平滑(One Euro Filter)。----
    // 位置：三分量各自独立的自适应低通，静止时抖动被压制，快速移动时
    // 自动抬高截止频率、不拖尾。
    const std::array<double,3> smoothedPos = posFilter_.filter(
        {st.wristPos[0], st.wristPos[1], st.wristPos[2]}, ts_ns);

    // 旋转：矩阵不能直接对9个数分别滤(滤完不再是合法旋转矩阵)，转四元数、
    // 各分量独立滤波、renormalize、转回矩阵。四元数有双重覆盖(q和-q代表
    // 同一个旋转)，跟上一帧比较点积，符号对不上就整体取反，否则符号
    // 翻转会被滤波器当成一次180度的巨大跳变。
    std::array<double,4> rawQuat = matToQuat(st.wristRot);
    if (haveLastQuat_) {
        double dot = 0.0;
        for (int i=0;i<4;++i) dot += rawQuat[size_t(i)]*lastQuatRaw_[size_t(i)];
        if (dot < 0.0) for (auto& c : rawQuat) c = -c;
    }
    lastQuatRaw_ = rawQuat;
    haveLastQuat_ = true;

    std::array<double,4> smoothedQuat{};
    for (int i=0;i<4;++i) smoothedQuat[size_t(i)] = quatFilters_[size_t(i)].filter(rawQuat[size_t(i)], ts_ns);
    const auto smoothedRot = quatToMat(smoothedQuat);

    QVector<double> rot9(9);
    for (int i = 0; i < 9; ++i) rot9[i] = smoothedRot[size_t(i)];

    // 关节角：每个自由度独立一个OneEuroFilter(不同手指、不同关节的运动
    // 速度差异很大，不共享一组截止频率参数才对，但为简化调参，UI上暴露
    // 的是统一的minCutoff/beta，16个滤波器各自维护自己的内部状态即可)。
    QVector<double> angles(int(st.jointAngles.size()));
    for (int i = 0; i < int(st.jointAngles.size()) && i < 16; ++i)
        angles[i] = jointFilters_[size_t(i)].filter(st.jointAngles[size_t(i)], ts_ns);
    for (int i = 16; i < int(st.jointAngles.size()); ++i) angles[i] = st.jointAngles[size_t(i)];   // 万一关节数以后不是16，多出来的原样传，不硬套滤波器数组

    // 关节角限位夹紧——这是第二道防线，放在平滑之后。第一道防线在
    // pipeline_->step()成功后立刻夹了IEKF内部的名义状态(见上面
    // pipeline_->clampJointAngles(...)那段)，这里再夹一次纯粹是给
    // OneEuroFilter平滑之后的输出兜底(理论上凸组合不会重新越界，因为
    // jointLimits是简单的[lo,hi]区间约束、平滑是加权平均，但多一道免费
    // 的检查没有坏处，不依赖这个理论保证)。
    if (angles.size() == kHandNumJoints) {
        std::array<double,16> q{};
        for (int i = 0; i < kHandNumJoints; ++i) q[size_t(i)] = angles[i];
        const auto clamped = clampToLimits(q);
        for (int i = 0; i < kHandNumJoints; ++i) angles[i] = clamped[size_t(i)];
    }

    emit handPoseReady(QVector3D(float(smoothedPos[0]), float(smoothedPos[1]), float(smoothedPos[2])),
                      rot9, angles, ts_ns);
}

std::vector<Vec3> HandTrackingWorker::computeCandidatePoints(const std::vector<CameraFrame>& frames) const {
    std::vector<std::vector<std::array<double,2>>> obsPerCam(camPoses_.size());
    std::vector<std::vector<Cov2>> obsCovPerCam(camPoses_.size());
    for (const auto& frame : frames) {
        if (frame.camIndex < 0 || size_t(frame.camIndex) >= obsPerCam.size()) continue;
        auto& dst = obsPerCam[size_t(frame.camIndex)];
        auto& dstCov = obsCovPerCam[size_t(frame.camIndex)];
        for (const auto& det : frame.detections) { dst.push_back({det.nx, det.ny}); dstCov.push_back(det.sigma); }
    }
    std::vector<EpiMat3> Rs; std::vector<EpiVec3> ts;
    Rs.reserve(camPoses_.size()); ts.reserve(camPoses_.size());
    for (const auto& cp : camPoses_) { Rs.push_back(cp.R); ts.push_back(cp.t); }

    // 传入 obsCovPerCam：投票阶段改用马氏距离(每个观测按自己在
    // BlobObservationAdapter 里算出的真实不确定度自适应门控)，不再是
    // 固定的 clusterMaxReprojNorm_ 一刀切——见 MultiViewCluster.hpp 里
    // clusterMultiView() 该参数的说明。
    const auto cluster = clusterMultiView(Rs, ts, obsPerCam, clusterMaxSampson_, clusterMaxReprojNorm_,
                                          /*useLmRefine=*/true, &obsCovPerCam);
    std::vector<Vec3> candidates;
    candidates.reserve(cluster.tracks.size());
    for (const auto& tr : cluster.tracks) candidates.push_back(tr.point);

    if (verboseLogging_) {
        QString perCam;
        for (size_t i=0;i<obsPerCam.size();++i) perCam += QStringLiteral("cam#%1=%2帧观测 ").arg(i).arg(int(obsPerCam[i].size()));
        qDebug().noquote() << QStringLiteral(
            "[HandTracking] 输入给clusterMultiView: %1 -> 三角化候选点=%2 (阈值 maxSampson=%3 maxReprojNorm=%4，马氏距离投票已启用)")
            .arg(perCam).arg(int(candidates.size())).arg(clusterMaxSampson_, 0, 'g', 3).arg(clusterMaxReprojNorm_, 0, 'g', 3);
    }
    return candidates;
}

std::optional<HandPoseState> HandTrackingWorker::coldStartFromFrames(
        const std::vector<CameraFrame>& frames) {
    const auto candidates = computeCandidatePoints(frames);

    const auto& templ = templateStore_ ? templateStore_->data().backMarkers : handBackMarkers();
    const auto result = coldStartHandPose(candidates, templ, kHandNumJoints, coldStartCfg_);

    // 不管成不成功都发诊断，UI 靠这个告诉用户具体卡在哪一步——这是本轮
    // 修复的核心：以前失败了只有"没找到"，现在有"候选点几个/距离匹配几组/
    // 最佳残差多少"这些具体数字。
    emit coldStartDiagnosticsReady(result.diag.numCandidates3D, result.diag.numDistanceMatchesFound,
                                   result.diag.bestRms, int(result.diag.failReason),
                                   QString::fromUtf8(result.diag.summary()), currentTs_);

    return result.state;
}

// TrackingMode::BackRigidOnly 专用分支——只测手背刚体，完全不碰手指。
// 每帧直接对候选点做距离匹配+Kabsch(复用 HandColdStart.hpp 那套数学，
// 不是每帧"冷启动"的意思，是这套数学本身就适合逐帧独立求解：手背5点是
// 刚体，不需要像16维关节角IEKF那样跨帧维持一个复杂状态，每帧独立解一次
// 位姿、再用同一套OneEuroFilter平滑，比硬套完整IEKF却把手指角度锁死更
// 干净——物理上根本不存在"手指乱飞"，因为从头到尾没有任何手指相关的
// 计算发生。
void HandTrackingWorker::processBackRigidOnlyFrame(const std::vector<CameraFrame>& frames, qint64 ts_ns) {
    const auto candidates = computeCandidatePoints(frames);
    const auto& templ = templateStore_ ? templateStore_->data().backMarkers : handBackMarkers();
    const auto result = coldStartHandPose(candidates, templ, kHandNumJoints, coldStartCfg_);

    emit coldStartDiagnosticsReady(result.diag.numCandidates3D, result.diag.numDistanceMatchesFound,
                                   result.diag.bestRms, int(result.diag.failReason),
                                   QString::fromUtf8(result.diag.summary()), ts_ns);

    if (!result.state) {
        emit handNotFound(ts_ns);
        hadStateLastFrame_ = false;   // 下次重新捕获时，平滑滤波器要重置(见processFrame里同样的处理)
        return;
    }

    if (!hadStateLastFrame_) resetSmoothingFilters();
    hadStateLastFrame_ = true;

    const auto& st = *result.state;

    const std::array<double,3> smoothedPos = posFilter_.filter(
        {st.wristPos[0], st.wristPos[1], st.wristPos[2]}, ts_ns);

    std::array<double,4> rawQuat = matToQuat(st.wristRot);
    if (haveLastQuat_) {
        double dot = 0.0;
        for (int i=0;i<4;++i) dot += rawQuat[size_t(i)]*lastQuatRaw_[size_t(i)];
        if (dot < 0.0) for (auto& c : rawQuat) c = -c;
    }
    lastQuatRaw_ = rawQuat;
    haveLastQuat_ = true;

    std::array<double,4> smoothedQuat{};
    for (int i=0;i<4;++i) smoothedQuat[size_t(i)] = quatFilters_[size_t(i)].filter(rawQuat[size_t(i)], ts_ns);
    const auto smoothedRot = quatToMat(smoothedQuat);

    QVector<double> rot9(9);
    for (int i = 0; i < 9; ++i) rot9[i] = smoothedRot[size_t(i)];

    // 关节角固定为中性值(全0，伸直手型)——这个模式压根不估计手指，给个
    // 确定的、不会"乱飞"的值，骨架图显示出来就是一个安安静静的伸直手型，
    // 只有手腕会跟着真实动作动。
    QVector<double> angles(kHandNumJoints, 0.0);

    emit handPoseReady(QVector3D(float(smoothedPos[0]), float(smoothedPos[1]), float(smoothedPos[2])),
                      rot9, angles, ts_ns);
}

} // namespace mocap
