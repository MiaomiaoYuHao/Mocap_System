// ===========================================================================
// PointCloudTestDialog_pipeline.cpp —— PointCloudTestDialog 的“结果处理管线”分片。
// 从 PointCloudTestDialog.cpp 拆出（原第 2827~4391 行）：析构、onSkeletonResultReady、
// rebuild / rebuildSkeletonTemplate、buildRecordHeader。仍是该类成员函数，与其它 .cpp 共用头文件。
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

PointCloudTestDialog::~PointCloudTestDialog() {
    if (!wasDetectOn_)
        for (ICamera* c : weTurnedOn_) c->setDetectEnabled(false);
    for (ICamera* c : weTurnedOnContour_) c->setContourCollectionEnabled(false);

    // 骨骼关联线程收尾。
    // 【重要——不能用 wait()，哪怕带超时】之前的写法是 quit()+wait(3000)：
    // 表面上有超时保护，但只要worker线程当时没有正好处于空闲、事件循环
    // 没能立刻响应quit()，这个 wait() 就是【同步阻塞在GUI主线程】上，
    // 用户点关闭按钮那一下的事件处理函数里卡住不返回——体感上跟真正
    // 死锁没区别("点了没反应")，只是最终会在3秒内自己恢复。这正是
    // "关闭点云直接卡死"的原因。
    //
    // 正确做法：GUI线程完全不等待，用Qt文档推荐的标准模式——线程退出
    // (finished信号)时才自我清理，谁也不卡谁：
    //   quit()立刻返回(只是投递一个退出请求，不等它真正生效)
    //   -> worker线程事件循环退出、发出finished
    //   -> finished信号触发worker和thread各自deleteLater()
    // 这条链路里GUI线程从头到尾没有任何阻塞调用，不管worker那边是1毫秒
    // 内退出还是卡了10秒才退出，"关闭这个对话框"这个操作本身永远是
    // 立即返回的。
    if (skeletonThread_) {
        // 先断开投递给"this"的连接：对话框马上就要析构了，不能再让worker
        // 之后emit的resultReady调用一个已经不存在的onSkeletonResultReady。
        if (skeletonWorker_) disconnect(skeletonWorker_, nullptr, this, nullptr);

        QThread* th = skeletonThread_;
        SkeletonAssocWorker* wk = skeletonWorker_;
        if (wk) connect(th, &QThread::finished, wk, &QObject::deleteLater);
        connect(th, &QThread::finished, th, &QObject::deleteLater);

        // QThread对象本身的线程亲和性是"创建它的线程"(这里是GUI线程)，
        // 不是它管理的那个worker线程，所以 setParent(nullptr) 是必须的——
        // 否则 QDialog 的 QObject 子对象自动清理机制会在对话框析构时试图
        // delete 这个 QThread，跟上面 deleteLater() 那条路径重复delete，
        // 直接崩溃。
        th->setParent(nullptr);
        th->quit();   // 不等待，立刻返回

        skeletonWorker_ = nullptr;
        skeletonThread_ = nullptr;
    }
}

void PointCloudTestDialog::setUdpStatus(bool enabled, const QString& target) {
    if (outputMonitor_) outputMonitor_->setUdpStatus(enabled, target);
}

// 骨骼关联结果异步回调——跑在主线程(QueuedConnection跨线程投递回来的)，
// 可以放心碰UI控件。回答"AI这一步到底有没有生效、卡在哪"：
//   - backend没ready：onnxruntime没装/模型没训练出来，最常见的"没生效"原因
//   - palmOk==false：手掌5点冷启动没解出来(非AI那步)，AI根本没轮到跑
//   - palmOk==true：显示耗时 + 15个手指关节里几个是真观测到、几个是AI预测
//     填补的遮挡点——预测数量长期偏高说明候选点没喂够/标定模板跟实际手
//     型偏差大，不是模型本身的问题。
void PointCloudTestDialog::onSkeletonResultReady(const hm20::SkeletonFrameResult& result, const SkeletonAssocDiag& diag) {
    lastDiagSnapshot_ = diag;
    skeletonLastLatencyMs_ = diag.latencyMs;

    // ---- 端到端滞后实测 ----
    // 【为什么不能只看 diag.latencyMs】那个只量 process() 内部。真正决定"跟不跟手"
    // 的是三段之和，而其中两段在 worker 外面：
    //   排队 = 投递 -> worker 真正开跑 + 结果回传，AI 线程抢不到 CPU 时这段最大
    //   陈旧 = 限流造成的"这条线画上去之后还要停留多久"，平均 = 刷新间隔的一半
    // 三段分开量，才能知道该调哪一个：排队大就提优先级，推理大就降刷新率或
    // 换 Release 构建，陈旧大就把"骨架刷新"调高。
    if (dispatchNs_ >= 0 && pipeClock_.isValid()) {
        const double e2e = double(pipeClock_.nsecsElapsed() - dispatchNs_) / 1.0e6;
        emaE2eMs_ = (emaE2eMs_ < 0) ? e2e : (emaE2eMs_ * 0.9 + e2e * 0.1);
        dispatchNs_ = -1;
    }
    {
        static thread_local qint64 lastResultNs = -1;
        const qint64 nowNs = pipeClock_.isValid() ? pipeClock_.nsecsElapsed() : -1;
        if (nowNs >= 0 && lastResultNs >= 0) {
            const double iv = double(nowNs - lastResultNs) / 1.0e6;
            if (iv > 0.0 && iv < 2000.0)
                emaSkelIntervalMs_ = (emaSkelIntervalMs_ < 0) ? iv
                                                              : (emaSkelIntervalMs_ * 0.9 + iv * 0.1);
        }
        lastResultNs = nowNs;
    }

    if (recorder_.recording()) {
        mocap::pcrec::SkeletonRec sr;
        sr.valid = uint8_t(result.valid);
        sr.wristPoseValid = uint8_t(result.wristPoseValid);
        sr.pentagonOk = uint8_t(result.pentagonOk);
        sr.numGhost = uint8_t(std::min(result.numGhost, 255));
        sr.dorsumRmseMm = float(result.dorsumRmseMm);
        for (int m = 0; m < 20; ++m) {
            for (int d = 0; d < 3; ++d) sr.pos[m*3+d] = float(result.markers[size_t(m)].posWorld[size_t(d)]);
            sr.observed[m] = uint8_t(result.markers[size_t(m)].observed);
            sr.conf[m] = float(result.markers[size_t(m)].confidence);
        }
        for (int g = 0; g < 16; ++g) {
            sr.segSource[g] = uint8_t(result.segSource[size_t(g)]);
            for (int k = 0; k < 4; ++k) sr.segQuat[g*4+k] = float(result.segQuat[size_t(g)][size_t(k)]);
        }
        for (int i = 0; i < 9; ++i) sr.wristR[i] = float(result.wristR[size_t(i)]);
        for (int i = 0; i < 3; ++i) sr.wristT[i] = float(result.wristT[size_t(i)]);
        // 【用输入帧时间戳，不是墙钟】墙钟晚一个处理延迟（实测约30ms），
        // 用它记录的话骨架帧跟点云帧对不上，离线复现无从做起。
        recorder_.writeSkeleton(
            diag.frameTsNs > 0 ? diag.frameTsNs : QDateTime::currentMSecsSinceEpoch()*1000000LL, sr);

        // ---- 逐帧 hm20 诊断（块类型 9）----
        // SkeletonRec 只有"结果是什么"，这个块记的是"结果是怎么来的"：
        // 手背标签是几何解的还是连续性仲裁的、拇指走的几何/IK/网络/预测哪一档、
        // 腕部位姿是本帧解出来的还是沿用上一帧。事后看到一段抖动能直接归因，
        // 而不是只能猜。
        mocap::pcrec::Hm20DiagRec hd;
        hd.wallNs    = QDateTime::currentMSecsSinceEpoch() * 1000000LL;
        hd.frameTsNs = diag.frameTsNs;
        auto u8 = [](int v) { return uint8_t(std::clamp(v, 0, 255)); };

        hd.dorsumReason    = u8(diag.dorsumReason);
        hd.dorsumInliers   = u8(diag.dorsumInliers);
        hd.dorsumByHistory = uint8_t(diag.dorsumByHistory);
        hd.wristPoseValid  = uint8_t(diag.wristPoseValid);
        hd.pentagonOk      = uint8_t(diag.pentagonOk);
        hd.numGhost        = u8(diag.numGhost);
        hd.observedJoints  = u8(diag.observedJoints);
        hd.candidateCount  = u8(diag.candidateCount);
        hd.dorsumRmseMm    = float(diag.dorsumRmseMm);
        hd.dorsumMarginMm  = float(diag.dorsumMarginMm);
        hd.dorsumSelfAmbMm = float(diag.dorsumSelfAmbMm);

        hd.thumbFixSkip           = u8(diag.thumbFixSkip);
        hd.thumbFixed             = u8(diag.thumbFixed);
        hd.netThumbSegs           = u8(diag.netThumbSegs);
        hd.dorsumRepaired         = u8(diag.dorsumRepaired);
        // 【一直没填，本次分析才发现】没有它就分不清"标签翻了"和"点丢了" ——
        // 这两种在画面上长得一样，成因和修法完全不同。
        for (int m = 0; m < 20; ++m)
            hd.sourcePointId[m] = int32_t(result.markers[size_t(m)].sourcePointId);
        hd.thumbTipOccluded       = uint8_t(diag.thumbTipOccluded);
        hd.dorsumRepairMoveMm     = float(diag.dorsumRepairMoveMm);
        hd.thumbIpPredDeg         = float(diag.thumbIpPredRad * 180.0 / M_PI);
        hd.thumbIpDriftDeg        = float(diag.thumbIpDriftRad * 180.0 / M_PI);
        hd.bundleRuns             = int32_t(diag.bundleRuns);
        hd.bundleLastMs           = float(diag.bundleLastMs);
        hd.thumbPronationFitted   = uint8_t(diag.thumbPronationFitted);
        hd.thumbPronation0        = float(diag.thumbPronation0);
        hd.thumbAxialK            = float(diag.thumbAxialK);
        hd.thumbPronationContrast = float(diag.thumbPronationContrast);
        hd.thumbCoverage          = float(diag.thumbCoverage);

        hd.handPanelIsRight   = uint8_t(handRightChk_ ? handRightChk_->isChecked() : true);
        hd.handAutoIsRight    = uint8_t(diag.autoIsRight);
        hd.handAutoDetected   = uint8_t(diag.autoIsRightDetected);
        hd.handAiIsRight      = uint8_t(diag.aiHandIsRight);
        hd.handednessKnown    = uint8_t(diag.autoHandednessKnown);
        hd.handednessConflict = uint8_t(diag.autoHandednessConflict);
        hd.mirrorSuspect      = uint8_t(diag.autoMirrorSuspect);
        hd.jointMirrorActive  = uint8_t(diag.jointMirrorActive);
        hd.handSignMm         = float(diag.autoHandSignMm);

        hd.ikActive       = uint8_t(diag.ikActive);
        hd.ikFingerCount  = u8(diag.ikFingerCount);
        hd.occludedHeld   = u8(result.occludedHeld);
        hd.chainContinued = u8(diag.chainContinued);
        hd.ikFilled       = u8(diag.ikFilled);
        hd.ikFallback     = u8(diag.ikFallback);
        for (int f = 0; f < 5; ++f) {
            hd.fingerIkRmseMm[f]  = float(result.fingerIkRmseMm[size_t(f)]);
            hd.fingerIkValid[f]   = uint8_t(result.fingerIkValid[size_t(f)]);
            hd.fingerObsCount[f]  = u8(result.fingerObsCount[size_t(f)]);
        }
        for (int g = 0; g < 16; ++g) {
            hd.segSource[g] = uint8_t(result.segSource[size_t(g)]);
            hd.segConf[g]   = float(result.segConf[size_t(g)]);
        }

        hd.autoStage           = u8(diag.autoStage);
        hd.autoTemplateReady   = uint8_t(diag.autoTemplateReady);
        hd.autoIkUsable        = uint8_t(diag.autoIkUsable);
        hd.autoDorsumReordered = uint8_t(diag.autoDorsumReordered);
        hd.autoProgress        = float(diag.autoProgress);
        hd.autoBundleRmseMm    = float(diag.autoBundleRmseMm);
        for (int f = 0; f < 5; ++f) hd.anchorRigidStd[f] = float(diag.anchorRigidStd[f]);

        hd.netSegSolved   = uint8_t(diag.netSegSolved);
        hd.netSegOk       = uint8_t(diag.netSegOk);
        hd.hasAiPose      = uint8_t(diag.hasAiPose);
        hd.hasAiHand      = uint8_t(diag.hasAiHand);
        hd.netSegCSpread  = float(diag.netSegCSpread);
        for (int j = 0; j < 3; ++j) hd.netSegKOffset[j] = float(diag.netSegKOffset[size_t(j)]);
        for (int j = 0; j < 5; ++j) hd.aiPoseConf[j] = float(diag.aiPoseConf[j]);
        hd.latencyMs = float(diag.latencyMs);
        recorder_.writeHm20Diag(hd);

        // ===================================================================
        // v4：把断掉的证据链接上
        // ===================================================================

        // ---- 块 13：关节角输出全链路 ----
        // 【这是"输出的角/四元数对不对"唯一能查的地方】v3 里这一整级是空白的：
        // UDP 上那 16 个数经过解算->平滑->ROM->限幅四级，只录最后一个数的话，
        // "握拳输出像张开"这一个症状对应的四种成因完全分不开。
        {
            mocap::pcrec::JointOutRec jo;
            jo.wallNs    = hd.wallNs;
            jo.frameTsNs = diag.frameTsNs;
            for (int i = 0; i < 16; ++i) {
                jo.qSolve[i]  = float(diag.qSolve[size_t(i)]);
                jo.qSmooth[i] = float(diag.qSmooth[size_t(i)]);
                jo.qRom[i]    = float(diag.qRom[size_t(i)]);
                jo.qOut[i]    = float(diag.qOut[size_t(i)]);
                jo.romLo[i]   = float(diag.romLo[size_t(i)]);
                jo.romHi[i]   = float(diag.romHi[size_t(i)]);
                jo.segSource[i] = uint8_t(result.segSource[size_t(i)]);
            }
            for (int i = 0; i < 64; ++i) {
                jo.segQuatWorld[i] = float(diag.quatWorld[size_t(i)]);
                jo.segQuatLocal[i] = float(diag.quatLocal[size_t(i)]);
            }
            for (int k = 0; k < 4; ++k) jo.wristQuat[k] = float(result.wristQuat[size_t(k)]);
            for (int k = 0; k < 3; ++k) jo.wristT[k]    = float(result.wristT[size_t(k)]);
            for (int f = 0; f < 5; ++f) {
                jo.fingerValid[f]     = uint8_t(diag.jointFingerValid[size_t(f)]);
                jo.fingerPredicted[f] = uint8_t(diag.jointFingerPredicted[size_t(f)]);
            }
            jo.romCoverage        = float(diag.romCoverage);
            jo.romAbdCoverage     = float(diag.romAbdCoverage);
            jo.dtSec              = float(diag.dtSec);
            jo.rateLimitRadPerSec = float(diag.rateLimitRadPerSec);
            jo.angSmoothAlpha     = float(diag.angSmoothAlpha);
            jo.maxRateClipRad     = float(diag.maxRateClipRad);
            jo.maxStageDeltaRad   = float(diag.maxStageDeltaRad);
            jo.romSamples         = int32_t(diag.romSamples);
            jo.maxRateClipIdx     = int32_t(diag.maxRateClipIdx);
            jo.maxStageDeltaIdx   = int32_t(diag.maxStageDeltaIdx);
            jo.romReady      = uint8_t(diag.romReady);
            jo.romLearning   = uint8_t(diag.romLearningOn);
            jo.rateLimitOn   = uint8_t(diag.rateLimitOn);
            jo.angSmoothOn   = uint8_t(diag.angSmoothOn);
            jo.mcpValid      = uint8_t(diag.mcpValid);
            jo.wristValid    = uint8_t(diag.wristPoseValid);
            jo.jointMirror   = uint8_t(diag.jointMirrorFlag);
            jo.wristStale    = u8(diag.wristStaleFrames);
            jo.quatOutOn     = uint8_t(diag.quatOutOn);
            jo.filterActive  = uint8_t(diag.filterActive);
            jo.lowLatency    = uint8_t(lowLatencyChk_ && lowLatencyChk_->isChecked());
            recorder_.writeJointOut(jo);
        }

        // ---- 块 14：手性证据链 ----
        // 【六处环节各自的手性都记下来】v3 只有 4 个 bool，而 tmplIsRight
        // （真正送进网络的那个）和 ikIsRight 都不在里面 —— 它们各有独立的
        // 改写路径，可以跟面板值和自标定推断值都不一样。那时四个 bool 全"正常"
        // 而输出是镜像的，从文件里查不出任何异常。
        {
            mocap::pcrec::HandednessRec hr;
            hr.wallNs    = hd.wallNs;
            hr.frameTsNs = diag.frameTsNs;
            hr.handSignMm        = float(diag.autoHandSignMm);
            hr.handSignLatMm     = float(diag.handSignLatMm);
            hr.handednessMinMm   = float(diag.handednessMinMm);
            hr.aiHandLogit       = float(diag.aiHandLogit);
            hr.aiHandConf        = float(diag.aiHandConfVal);
            hr.thumbRollSignedRad= float(diag.thumbRollSignedRad);
            hr.handSignN         = int32_t(diag.handSignN);
            hr.geoHandSign       = int32_t(diag.geoHandSign);
            hr.panelIsRight        = uint8_t(handRightChk_ ? handRightChk_->isChecked() : true);
            hr.autoIsRightDetected = uint8_t(diag.autoIsRightDetected);
            hr.autoIsRight         = uint8_t(diag.autoIsRight);
            hr.aiIsRight           = uint8_t(diag.aiHandIsRight);
            hr.tmplIsRight         = uint8_t(diag.tmplIsRight);
            hr.ikIsRight           = uint8_t(diag.ikIsRight);
            hr.autoKnown           = uint8_t(diag.autoHandednessKnown);
            hr.autoAgree           = uint8_t(diag.handednessAgree);
            hr.autoConflict        = uint8_t(diag.autoHandednessConflict);
            hr.autoMirrorSuspect   = uint8_t(diag.autoMirrorSuspect);
            hr.aiHas               = uint8_t(diag.hasAiHand);
            hr.aiKnown             = uint8_t(diag.aiHandKnown);
            hr.aiLocked            = uint8_t(diag.aiHandLocked);
            hr.cfgDetectHandedness = uint8_t(diag.cfgDetectHandedness);
            hr.cfgApplyHandedness  = uint8_t(diag.cfgApplyHandedness);
            hr.cfgJointMirrorAuto  = uint8_t(diag.cfgJointMirrorAuto);
            hr.jointMirrorActive   = uint8_t(diag.jointMirrorActive);
            hr.autoCalibOn         = uint8_t(diag.autoCalibOn);
            hr.autoStage           = u8(diag.autoStage);
            recorder_.writeHandedness(hr);
        }

        // ---- 块 15：逐 marker 来源与位置 ----
        // 【posNet 和 posFinal 都记】两者之差 = 后处理把这个点搬了多远，
        // 这是分开"模型给歪了"和"后处理拧错了"的直接依据 ——
        // 只看最终位置的话这两件事长得一模一样，而修法完全相反。
        {
            mocap::pcrec::MarkerDbgRec mr;
            mr.wallNs    = hd.wallNs;
            mr.frameTsNs = diag.frameTsNs;
            mr.hasNet    = uint8_t(result.hasDebugStreams);
            for (int m = 0; m < 20; ++m) {
                const auto& mk = result.markers[size_t(m)];
                for (int d = 0; d < 3; ++d)
                    mr.posFinal[m*3+d] = float(mk.posWorld[size_t(d)]);
                mr.observed[m]      = uint8_t(mk.observed);
                mr.conf[m]          = float(mk.confidence);
                mr.sourcePointId[m] = int32_t(mk.sourcePointId);
                mr.source[m]        = result.markerSource[size_t(m)];
                mr.flags[m]         = result.markerFlags[size_t(m)];
                if (result.hasDebugStreams) {
                    double d2 = 0.0;
                    for (int d = 0; d < 3; ++d) {
                        const double v = result.netPos[size_t(m)][size_t(d)];
                        mr.posNet[m*3+d] = float(v);
                        const double e = mk.posWorld[size_t(d)] - v;
                        d2 += e * e;
                    }
                    mr.netDeltaMm[m] = float(std::sqrt(d2));
                    mr.missLogit[m]  = float(result.netMissLogit[size_t(m)]);
                } else {
                    // 【填 -1 而不是 0】0 是一个合法的位置/残差值，会被当成
                    // "后处理没动过这个点"。-1 是不可能出现的距离，一眼看得出
                    // 是"没记到"。这个区别在事后分析时是决定性的。
                    mr.netDeltaMm[m] = -1.f;
                    mr.missLogit[m]  = -999.f;
                }
            }
            recorder_.writeMarkerDbg(mr);
        }

        // ---- 块 16：逐帧生效开关 ----
        {
            mocap::pcrec::RunFlagsRec rf;
            rf.wallNs    = hd.wallNs;
            rf.frameTsNs = diag.frameTsNs;
            rf.bits      = uint32_t(diag.runFlags);
            // 位 18/20/21 worker 看不到（是面板这一层的状态），在这里补齐。
            if (lowLatencyChk_ && lowLatencyChk_->isChecked())
                rf.bits |= mocap::pcrec::RF_LowLatency;
            if (showSkeletonChk_ && showSkeletonChk_->isChecked())
                rf.bits |= mocap::pcrec::RF_ShowSkeleton;
            rf.bits |= mocap::pcrec::RF_Recording;
            rf.thumbRollUiDeg     = float(diag.thumbRollUiDeg);
            rf.thumbPronation0Rad = float(diag.thumbPronation0Rad);
            rf.occludedJumpGateMm = float(diag.occludedJumpGateMm);
            rf.latencyMs          = float(diag.latencyMs);
            recorder_.writeRunFlags(rf);
        }

        // ---- 块 17：1Hz 全量运行时状态 ----
        // worker 早就在生成这份 JSON（hm20 全部配置 + 自标定全部产物），
        // 但它只给面板看、从没落盘。非空 = 这一帧刚好刷新过。
        if (!diag.stateJson.empty())
            recorder_.writeStateJson(hd.wallNs, diag.stateJson);

        // ===================================================================
        // v5：中间级 + 上下游两端
        // ===================================================================

        // ---- 块 18：20 点位置的六级流水 ----
        // 【比关节角四级更该有】位置在角度的上游，位置错了角度必然跟着错。
        // 0..4 级由关联器填（它看不到滤波），第 5 级只有 worker 知道。
        {
            mocap::pcrec::MarkerStageRec ms;
            ms.wallNs = hd.wallNs;
            ms.frameTsNs = diag.frameTsNs;
            for (int s = 0; s < 5; ++s) {
                ms.stageValid[s] = uint8_t(result.stageValid[size_t(s)]);
                for (int m = 0; m < 20; ++m)
                    for (int d = 0; d < 3; ++d)
                        ms.pos[s][m*3+d] = float(result.stagePos[size_t(s)][size_t(m)][size_t(d)]);
            }
            ms.stageValid[5] = uint8_t(diag.stage5Valid);
            for (int m = 0; m < 20; ++m)
                for (int d = 0; d < 3; ++d)
                    ms.pos[5][m*3+d] = float(diag.stage5Pos[size_t(m)][size_t(d)]);
            // 【逐级位移在线算】一眼扫过去哪一级的数突然变大，问题就在那一级，
            // 不用先把六组 60 维排开对着看。第 0 级恒为 0（没有上一级）。
            for (int s = 1; s < 6; ++s) {
                for (int m = 0; m < 20; ++m) {
                    double d2 = 0.0;
                    for (int d = 0; d < 3; ++d) {
                        const double e = double(ms.pos[s][m*3+d]) - double(ms.pos[s-1][m*3+d]);
                        d2 += e * e;
                    }
                    ms.stageMoveMm[s][m] = float(std::sqrt(d2));
                }
            }
            recorder_.writeMarkerStage(ms);
        }

        // ---- 块 19：滤波前后 + 滤波器内部 ----
        // 【v5 里最该有的一块】Hm20PoseFilter::apply() 是就地覆写 result 的，
        // 滤波前的位置在系统里没有第二份。于是"几何本来就解错了"和
        // "解对了被 One-Euro 吃平了"在文件里完全一样，而两者修法相反。
        {
            const auto& fb = diag.filt;
            mocap::pcrec::FilterDbgRec fd;
            fd.wallNs = hd.wallNs;
            fd.frameTsNs = diag.frameTsNs;
            for (int m = 0; m < 20; ++m) {
                for (int d = 0; d < 3; ++d) {
                    fd.posIn[m*3+d]  = float(fb.posIn[size_t(m)][size_t(d)]);
                    fd.posOut[m*3+d] = float(fb.posOut[size_t(m)][size_t(d)]);
                }
                fd.moveMm[m]      = float(fb.moveMm[size_t(m)]);
                fd.velLocalMm[m]  = float(fb.velLocalMm[size_t(m)]);
                fd.staleFrames[m] = float(fb.staleFrames[size_t(m)]);
                fd.fuseWeight[m]  = float(fb.fuseWeight[size_t(m)]);
                fd.cutoffHz[m]    = float(fb.cutoffHz[size_t(m)]);
                fd.hardReset[m]   = uint8_t(fb.hardReset[size_t(m)]);
                fd.deadzone[m]    = uint8_t(fb.deadzone[size_t(m)]);
                fd.fused[m]       = uint8_t(fb.fused[size_t(m)]);
                fd.observed[m]    = uint8_t(fb.observed[size_t(m)]);
            }
            for (int k = 0; k < 4; ++k) {
                fd.wristQuatIn[k]  = float(fb.wristQuatIn[size_t(k)]);
                fd.wristQuatOut[k] = float(fb.wristQuatOut[size_t(k)]);
            }
            fd.wristAngMoveDeg = float(fb.wristAngMoveDeg);
            for (int s = 0; s < 16; ++s)
                fd.segQuatMoveDeg[s] = float(fb.segQuatMoveDeg[size_t(s)]);
            fd.dtSec = float(fb.dtSec);
            fd.enabled = uint8_t(fb.ran);
            recorder_.writeFilterDbg(fd);
        }

        // ---- 块 20：IK 求解器内部 ----
        // 遮挡点的位置是这些角 FK 出来的。位置不对时往上查一级，查的就是它们。
        // 【limitHit 是重点】撞限位的那一维被罚函数拉住不动，症状正好是
        // "这根手指弯到一半就停住"，而残差可能仍然很小（其余维补偿了）。
        {
            const auto& ii = diag.ikInfo;
            mocap::pcrec::IkDbgRec ik;
            ik.wallNs = hd.wallNs;
            ik.frameTsNs = diag.frameTsNs;
            for (int f = 0; f < 5; ++f) {
                for (int k = 0; k < 4; ++k) {
                    ik.q[f][k]        = float(ii.q[size_t(f)][size_t(k)]);
                    ik.qPrior[f][k]   = float(ii.qPrior[size_t(f)][size_t(k)]);
                    ik.limitLo[f][k]  = float(ii.limitLo[size_t(f)][size_t(k)]);
                    ik.limitHi[f][k]  = float(ii.limitHi[size_t(f)][size_t(k)]);
                    ik.limitHit[f][k] = ii.limitHit[size_t(f)][size_t(k)];
                }
                for (int d = 0; d < 3; ++d) {
                    ik.anchorMm[f][d]  = float(ii.anchorMm[size_t(f)][size_t(d)]);
                    ik.boneLenMm[f][d] = float(ii.boneLenMm[size_t(f)][size_t(d)]);
                }
                ik.cost[f]         = float(ii.cost[size_t(f)]);
                ik.rmseMm[f]       = float(ii.rmseMm[size_t(f)]);
                ik.iters[f]        = int32_t(ii.iters[size_t(f)]);
                ik.nObs[f]         = int32_t(ii.nObs[size_t(f)]);
                ik.fingerValid[f]  = uint8_t(ii.fingerValid[size_t(f)]);
                ik.fingerSolved[f] = uint8_t(ii.solved[size_t(f)]);
            }
            ik.handLenMm       = float(ii.handLenMm);
            ik.thumbAxialK     = float(ii.thumbAxialK);
            ik.thumbPronation0 = float(ii.thumbPronation0);
            ik.wPrior      = float(ii.wPrior);
            ik.wLimit      = float(ii.wLimit);
            ik.wCouple     = float(ii.wCouple);
            ik.dipCoupling = float(ii.dipCoupling);
            for (int f = 0; f < 5; ++f) {
                ik.mcpPipCoupling[f]        = float(ii.mcpPipCoupling[size_t(f)]);
                ik.mcpPipCouplingB[f]       = float(ii.mcpPipCouplingB[size_t(f)]);
                ik.mcpPipCouplingSamples[f] = int32_t(ii.mcpPipCouplingSamples[size_t(f)]);
                ik.mcpCouplingReady[f]      = uint8_t(ii.mcpCouplingReady[size_t(f)]);
            }
            ik.ikIsRight   = uint8_t(ii.isRight);
            ik.paramsReady = uint8_t(ii.paramsReady);
            ik.applied     = uint8_t(ii.applied);
            recorder_.writeIkDbg(ik);
        }

        // ---- 块 21：模板快照 ----
        // 【手性问题的核心证据】文件头那一份是按下录制按钮那一刻的模板，
        // 而自标定会在录制中途热替换它 —— 第 N 帧真正送进网络的模板长什么样，
        // v4 的文件里查不到。带 fnv1a 指纹，比对一个 32 位数就知道变没变。
        if (diag.tmplSnapReason >= 0) {
            mocap::pcrec::TemplateSnapRec ts2;
            ts2.wallNs = hd.wallNs;
            ts2.frameTsNs = diag.frameTsNs;
            ts2.reason = int32_t(diag.tmplSnapReason);
            const auto& T = diag.tmplSnapshot;
            for (int m = 0; m < 20; ++m)
                for (int d = 0; d < 3; ++d)
                    ts2.markersMm[m][d] = float(T.markersMm[size_t(m)][size_t(d)]);
            for (int i = 0; i < 5; ++i)
                for (int d = 0; d < 3; ++d) {
                    ts2.backMarkersMm[i][d] = float(T.markersMm[size_t(i)][size_t(d)]);
                    ts2.anchorsMm[i][d]     = float(T.anchorsMm[size_t(i)][size_t(d)]);
                    ts2.boneLenMm[i][d]     = float(diag.tmplBoneLenMm[size_t(i)][size_t(d)]);
                }
            // 【归一化版本 = 网络真正看到的那份】原始 markersMm 和它差一个
            // 中心化+尺度归一化。手性判错时，两者未必同时看得出异常 ——
            // packNormalized 的最后一维直接就是 ±1 的 hand_sign。
            {
                const auto nz = T.packNormalized();
                for (int m = 0; m < 20; ++m)
                    for (int d = 0; d < 3; ++d)
                        ts2.normalized[m][d] = nz[size_t(m*3+d)];
            }
            for (int i = 0; i < 5; ++i) {
                ts2.dipCoupling[i]      = float(diag.tmplDipCoupling[size_t(i)]);
                ts2.fingerCalibrated[i] = uint8_t(diag.tmplFingerCalibrated[size_t(i)]);
            }
            ts2.isRight          = uint8_t(T.isRight);
            ts2.valid            = uint8_t(T.valid);
            ts2.anchorsValid     = uint8_t(T.anchorsValid);
            ts2.backCalibrated   = uint8_t(T.valid);
            ts2.fromAutoCalib    = uint8_t(diag.tmplFromAutoCalib);
            ts2.bundleRmseMm     = float(diag.tmplBundleRmseMm);
            ts2.selfAmbiguityMm  = float(diag.tmplSelfAmbMm);
            recorder_.writeTemplateSnap(ts2);   // 指纹由录制器算
        }

        // ---- 块 30：链式续解逐指内部状态 ----
        // 【为什么这块最要紧】实测遮挡点的填充来源：链式续解 81.1%、
        // 网络原始兜底 17.0%、IK 补 1.9%。遮挡点位置几乎全由链式续解决定，
        // 而这条路径此前没有任何内部量被记录过——前几轮一直在改 IK，
        // 而 IK 只碰了 1.9% 的点，所以怎么改都看不到效果。
        {
            mocap::pcrec::ChainDbgRec cd;
            cd.wallNs = hd.wallNs;
            cd.frameTsNs = diag.frameTsNs;
            for (int f = 0; f < 5; ++f) {
                cd.coupledRad[f]  = float(result.chainCoupledRad[size_t(f)]);
                cd.pmLenMm[f]     = float(result.chainPmLenMm[size_t(f)]);
                cd.mdLenMm[f]     = float(result.chainMdLenMm[size_t(f)]);
                cd.weight[f]      = float(result.chainWeight[size_t(f)]);
                cd.aging[f]       = float(result.chainAging[size_t(f)]);
                cd.caseBFrames[f] = int32_t(result.chainCaseBFrames[size_t(f)]);
                cd.hasPipHold[f]  = uint8_t(result.chainHasPipHold[size_t(f)]);
                cd.hasPlane[f]    = uint8_t(result.chainHasPlane[size_t(f)]);
                cd.caseB[f]       = uint8_t(result.chainCaseB[size_t(f)]);
                cd.useAnchor[f]   = uint8_t(result.chainUseAnchor[size_t(f)]);
                cd.hasExc[f]      = uint8_t(result.chainHasExc[size_t(f)]);
                // ---- v6.2 逐指 anchor / 平面（见 ChainDbgRec 里的说明）----
                cd.anchorOkPerFinger[f]   = uint8_t(result.anchorOkPerFinger[size_t(f)]);
                cd.planeSampleN[f]        = int32_t(result.planeSampleN[size_t(f)]);
                cd.anchorOutOfPlaneDeg[f] = float(result.anchorOutOfPlaneDeg[size_t(f)]);
            }
            cd.chainContinued = uint8_t(std::min(result.chainContinued, 255));
            // anchor 下发/自检链路——定位"为什么 hasPlane 恒为 0"的关键六个量
            cd.anchorsSet         = uint8_t(result.anchorsSet);
            cd.anchorsChecked     = uint8_t(result.anchorsChecked);
            cd.anchorsValid       = uint8_t(result.anchorsValid);
            cd.tmplMmValid        = uint8_t(result.tmplMmValid);
            cd.anchorsVerifyNSeen = int32_t(result.anchorsVerifyNSeen);
            cd.anchorsVerifyNOk   = int32_t(result.anchorsVerifyNOk);
            recorder_.writeChainDbg(cd);
        }

        // ---- 块 32：关节角解算的逐帧内部量（v7）----
        //
        // 【这一块补的是 JointOut 上游的那一截】JointOut 从 qSolve 开始记，
        // 而 qSolve 本身是怎么算出来的没有记。「四元数是对的但角度不对」
        // 之所以能同时成立，就是因为坏在【选哪两条骨轴】上，而轴从来没有
        // 出过 solveJointAngles 这个函数。
        //
        // 先看 dotProxMid：它 ≈1 说明两条"骨轴"其实是同一根骨头，
        // 此时 PIP = acos(1) = 0 —— 不是不准，是结构性的零。
        {
            const auto& jd = diag.jointDbg;
            mocap::pcrec::JointSolveRec js;
            js.wallNs    = hd.wallNs;
            js.frameTsNs = diag.frameTsNs;
            for (int f = 0; f < 5; ++f) {
                for (int c = 0; c < 3; ++c) {
                    js.axProx[f][c]      = float(jd.axProx[size_t(f)][size_t(c)]);
                    js.axMid[f][c]       = float(jd.axMid[size_t(f)][size_t(c)]);
                    js.axDist[f][c]      = float(jd.axDist[size_t(f)][size_t(c)]);
                    js.axProxWorld[f][c] = float(jd.axProxWorld[size_t(f)][size_t(c)]);
                }
                js.dotProxMid[f]    = float(jd.dotProxMid[size_t(f)]);
                js.hingeSigned[f]   = float(jd.hingeSigned[size_t(f)]);
                js.flexRaw[f]       = float(jd.flexRaw[size_t(f)]);
                js.abdRaw[f]        = float(jd.abdRaw[size_t(f)]);
                js.pipRaw[f]        = float(jd.pipRaw[size_t(f)]);
                js.ipRaw[f]         = float(jd.ipRaw[size_t(f)]);
                js.curlChain[f]     = float(jd.curlChain[size_t(f)]);
                js.curl[f]          = float(jd.curl[size_t(f)]);
                js.boneLenMm[f]     = float(jd.boneLenMm[size_t(f)]);
                js.curlValid[f]     = jd.curlValid[size_t(f)];
                js.curlPredicted[f] = jd.curlPredicted[size_t(f)];
                js.proxAxisSrc[f]   = jd.proxAxisSrc[size_t(f)];
                // ---- v8：PIP 的四元数解算路径 ----
                // 两条路都落盘。anchorsValid 为假时叉乘路径恒等于 0，
                // 真正在用的是 pipFromQuat；pipUsedQuat 说明本帧取的哪条，
                // pipFromAxis 留作对照，用来回答"这一帧两条路差多少"。
                for (int k = 0; k < 4; ++k)
                    js.qRelPip[f][k] = float(jd.qRelPip[size_t(f)][size_t(k)]);
                js.pipFromQuat[f] = float(jd.pipFromQuat[size_t(f)]);
                js.pipFromAxis[f] = float(jd.pipFromAxis[size_t(f)]);
                js.pipUsedQuat[f] = jd.pipUsedQuat[size_t(f)];
            }
            for (int i = 0; i < 16; ++i) {
                js.dofState[i]  = jd.dofState[size_t(i)];
                js.romReject[i] = diag.romReject[size_t(i)];
            }
            js.wristPoseValid = uint8_t(jd.wristPoseValid);
            js.anchorsValid   = uint8_t(jd.anchorsValid);
            js.mirrored       = uint8_t(jd.mirrored);
            js.romLearning    = uint8_t(diag.romLearningOn);
            js.wristStale     = int32_t(jd.wristStale);
            recorder_.writeJointSolve(js);
        }

        // ---- 块 23：把上一帧攒好的 UDP 输出记录落盘 ----
        // 【放在这里而不是信号回调里】两个包由两个独立信号送来，先到的那个
        // 不知道后面还有没有另一个。骨架结果这一刻两个都已经发过了，
        // 此时落盘才能把"这一帧发了哪几个包"记完整。
        flushUdpOut();

        // ---- 块 33：ROM 映射器逐维分支（v8）----
        // 【"标定完输出就冻住"唯一能定位的地方】Mapper::apply() 里通向
        // "输出是一个不动的数"的路一共有四条，产生的输出【完全一样】：
        //   1 死维 -> 永久钉中立位   2 保持上帧 -> 永久停在旧值
        //   3 区间塌 -> 中立位       4 正常映射但 t 恒被钳到 0 或 1
        // 而修法毫无共同点：①要回去查骨轴退化、②查 dofState、③重标、
        // ④补标那个姿态。只录 qRom 的话四条一条都分不开。
        //
        // 【branch=0（未标定透传）也要记】"标定之前就一直触限"的全部解释
        // 就在 tgtLo/tgtHi 和两端的钳位位上 —— 目标行程可以被
        // setTargetRange() 改写，而改写后的值以前从来没出过 Mapper。
        {
            const auto& md = diag.romMapDbg;
            mocap::pcrec::RomMapDbgRec rm;
            rm.wallNs    = hd.wallNs;
            rm.frameTsNs = diag.frameTsNs;
            for (int i = 0; i < 16; ++i) {
                const size_t u = size_t(i);
                rm.vSigned[i]   = float(md.vSigned[u]);
                rm.uNorm[i]     = float(md.uNorm[u]);
                rm.tgtLo[i]     = float(md.tgtLo[u]);
                rm.tgtHi[i]     = float(md.tgtHi[u]);
                rm.heldRun[i]   = md.heldRun[u];
                rm.frozenRun[i] = md.frozenRun[u];
                rm.branch[i]    = md.branch[u];
                rm.custom[i]    = md.custom[u];
                rm.clampLo[i]   = md.clampLo[u];
                rm.clampHi[i]   = md.clampHi[u];
                rm.status[i]    = md.status[u];
                rm.sign[i]      = md.sign[u];
                // dofState 是 branch=2 的判据【输入】。跟块 32 冗余，但那一块
                // 只在 Full 档写，而判据和判决分在两块里就得按时间戳 join ——
                // 又一个可能出错的环节，排查时不该有。
                rm.dofState[i]  = diag.jointDbg.dofState[u];
            }
            rm.nHeld        = int32_t(md.nHeld);
            rm.nFrozen      = int32_t(md.nFrozen);
            rm.maxFrozenRun = int32_t(md.maxFrozenRun);
            rm.maxFrozenIdx = int32_t(md.maxFrozenIdx);
            rm.nClamped     = int32_t(md.nClamped);
            rm.nNeutral     = int32_t(md.nNeutral);
            rm.dtSec        = float(diag.dtSec);
            rm.romReady         = uint8_t(diag.romReady);
            rm.holdOnDegenerate = uint8_t(diag.romMapHoldOn);
            rm.hasLast          = uint8_t(md.hasLast);
            rm.mapperJustReset  = uint8_t(md.justReset);
            rm.rateLimitOn      = uint8_t(diag.rateLimitOn);
            rm.romLearning      = uint8_t(diag.romLearningOn);
            recorder_.writeRomMapDbg(rm);
        }

        // ---- 块 34：角度后处理链的隐藏状态（v8）----
        // 【补的是 qSolve->qSmooth 和 qRom->qOut 这两段】块 13 记了四级的值，
        // 块 33 记了中间那一级的内部，剩下这两级的内部一直是空白。
        // 平滑器的 off[] 是【直接加到输出上的常量偏置】（最大 40°），
        // 它在系统里没有第二份 —— 于是"解算本来就偏了"和"解算对的、
        // 被一个没还清的恢复补偿顶着"这两件事，在文件里长得完全一样。
        {
            const auto& sd = diag.angSmoothDbg;
            const auto& rd = diag.rateDbg;
            mocap::pcrec::AngleChainRec ac2;
            ac2.wallNs    = hd.wallNs;
            ac2.frameTsNs = diag.frameTsNs;
            for (int i = 0; i < 16; ++i) {
                const size_t u = size_t(i);
                ac2.smState[i]      = float(sd.state[u]);
                ac2.smOffset[i]     = float(sd.offset[u]);
                ac2.smHas[i]        = sd.has[u];
                ac2.smOffClamped[i] = sd.offClamped[u];
                ac2.rlClip[i]       = float(rd.clip[u]);
            }
            for (int f = 0; f < 5; ++f) {
                ac2.smWasPred[f] = sd.wasPred[size_t(f)];
                ac2.smPredNow[f] = sd.predNow[size_t(f)];
            }
            ac2.smAlpha         = float(diag.angSmoothAlpha);
            ac2.smDecay         = float(diag.angSmoothDecay);
            ac2.smMaxOffsetRad  = float(diag.angSmoothMaxOffsetRad);
            ac2.maxSmOffsetRad  = float(sd.maxOffsetSeen);
            ac2.maxSmOffsetIdx  = int32_t(sd.maxOffsetIdx);
            ac2.nSmOffActive    = int32_t(sd.nOffActive);
            ac2.rlStepLimitRad  = float(rd.stepLimitRad);
            ac2.rlMaxRadPerSec  = float(diag.rateLimitRadPerSec);
            ac2.maxRlClipRad    = float(rd.maxClip);
            ac2.maxRlClipIdx    = int32_t(rd.maxClipIdx);
            ac2.nRlClipped      = int32_t(rd.nClipped);
            ac2.dtSec           = float(diag.dtSec);
            ac2.smOn            = uint8_t(diag.angSmoothOn);
            ac2.rlOn            = uint8_t(diag.rateLimitOn);
            ac2.rlHadState      = rd.hadState;
            ac2.lowLatency      = uint8_t(lowLatencyChk_ && lowLatencyChk_->isChecked());
            recorder_.writeAngleChain(ac2);
        }

        // ---- 块 31：ROM 标定逐维全量状态（v7）----
        // 只在【开始 / 标定中 1Hz / 结束】写，一次会话几十份，体积可忽略。
        // 【为什么标定过程中也要写】覆盖度是一步步涨上来的，而"刚过线就停手"
        // 和"远超阈值"最后的 ready 是一样的，可信度天差地别；中间那几份
        // 还能看出是哪一段动作把行程撑起来的。
        if (diag.romSnapReason >= 0) {
            const auto& rr = diag.romResult;
            mocap::pcrec::RomCalibRec rc;
            rc.wallNs    = hd.wallNs;
            rc.frameTsNs = diag.frameTsNs;
            rc.reason    = int32_t(diag.romSnapReason);
            for (int i = 0; i < 16; ++i) {
                const auto& d = rr.dof[size_t(i)];
                rc.lo[i]           = float(d.lo);
                rc.hi[i]           = float(d.hi);
                rc.rawLo[i]        = float(d.rawLo);
                rc.rawHi[i]        = float(d.rawHi);
                rc.measLo[i]       = float(d.measLo);
                rc.measHi[i]       = float(d.measHi);
                rc.coverage[i]     = float(d.coverage);
                rc.signDeltaRad[i] = float(d.signDeltaRad);
                rc.signCorr[i]     = float(d.signCorr);
                rc.openMedRad[i]   = float(d.openMed);
                rc.closeMedRad[i]  = float(d.closeMed);
                rc.curlSpread[i]   = float(d.curlSpread);
                rc.fitSlope[i]     = float(d.fitSlope);
                rc.fitR2[i]        = float(d.fitR2);
                rc.extrapLoRad[i]  = float(d.extrapLoRad);
                rc.extrapHiRad[i]  = float(d.extrapHiRad);
                rc.curlSeenLo[i]   = float(d.curlSeenLo);
                rc.curlSeenHi[i]   = float(d.curlSeenHi);
                rc.curlUsedLo[i]   = float(d.curlUsedLo);
                rc.curlUsedHi[i]   = float(d.curlUsedHi);
                rc.nSamples[i]     = int32_t(d.nSamples);
                rc.nSeen[i]        = int32_t(d.nSeen);
                rc.nOpen[i]        = int32_t(d.nOpen);
                rc.nClose[i]       = int32_t(d.nClose);
                for (int c = 0; c < 6; ++c) rc.nReject[i][c] = int32_t(d.nReject[size_t(c)]);
                rc.sign[i]         = int8_t(d.sign);
                rc.signResolved[i] = uint8_t(d.signResolved);
                rc.status[i]       = d.status;
            }
            rc.coverageFlex = float(rr.coverageFlex);
            rc.coverageAbd  = float(rr.coverageAbd);
            rc.durationSec  = float(rr.durationSec);
            rc.nOkFlex      = int32_t(rr.nOkFlex);
            rc.nPriorFlex   = int32_t(rr.nPriorFlex);
            rc.nDeadFlex    = int32_t(rr.nDeadFlex);
            rc.nSignFlipped = int32_t(rr.nSignFlipped);
            rc.nFrames      = int32_t(rr.nFrames);
            rc.nFramesUsed  = int32_t(rr.nFramesUsed);
            rc.ready        = uint8_t(rr.ready);
            rc.accumulated  = uint8_t(diag.romAccumulated);
            // ---- v8：判据跟结果一起存 ----
            // 【为什么必须存】同一批样本，minRangeRad 从 0.35 调到 0.25，
            // 一半的维就从 PriorFilled 变成 Ok，而覆盖度那个百分比一个字
            // 都没变 —— 两份看起来完全一样的记录可以是两个不同的结论。
            {
                const auto& c = diag.romCfg;
                rc.cfgMinRangeRad       = float(c.minRangeRad);
                rc.cfgMinCurlSpread     = float(c.minCurlSpread);
                rc.cfgMinSignDeltaRad   = float(c.minSignDeltaRad);
                rc.cfgOpenFrac          = float(c.openFrac);
                rc.cfgCloseFrac         = float(c.closeFrac);
                rc.cfgEndQuantile       = float(c.endQuantile);
                rc.cfgGlobalQLo         = float(c.globalQuantileLo);
                rc.cfgGlobalQHi         = float(c.globalQuantileHi);
                rc.cfgOutlierAbsRad     = float(c.outlierAbsRad);
                rc.cfgExtrapMinR2       = float(c.extrapMinR2);
                rc.cfgExtrapMinCurlSpan = float(c.extrapMinCurlSpan);
                rc.cfgExtrapMaxFrac     = float(c.extrapMaxFrac);
                rc.cfgPriorSpanFrac     = float(c.priorSpanFrac);
                rc.cfgMinSamplesPerDof  = int32_t(c.minSamplesPerDof);
                rc.cfgMinOkFlexDofs     = int32_t(c.minOkFlexDofs);
                rc.cfgMaxSamplesPerDof  = int32_t(c.maxSamplesPerDof);
                rc.cfgLearnSign          = uint8_t(c.learnSign);
                rc.cfgAcceptPredicted    = uint8_t(c.acceptPredicted);
                rc.cfgAcceptDegenerate   = uint8_t(c.acceptDegenerate);
                rc.cfgExtrapolateByCurl  = uint8_t(c.extrapolateByCurl);
                rc.cfgFillFromPrior      = uint8_t(c.fillFromPrior);
                rc.cfgHoldOnDegenerate   = uint8_t(c.holdOnDegenerate);
            }
            recorder_.writeRomCalib(rc);
        }

        // ---- 块 24：事件流 ----
        // 【applyHandedness 触发的镜像是一个瞬间事件】之后所有帧都"稳定地
        // 不对"，没有事件流的话那个瞬间在文件里没有任何标记，只能靠人眼
        // 在几千帧里找拐点 —— 而拐点常常不明显，因为参数是渐变的。
        for (const auto& ev : diag.events)
            recorder_.writeEvent(ev.code, ev.severity, hd.wallNs, diag.frameTsNs,
                                 ev.text, ev.valueA, ev.valueB, ev.intA, ev.intB);

        // ---- 块 25：自标定逐帧产物 ----
        // 【1Hz 的 stateJson 不够】判据是在动作里累积的，而关键的那几秒
        // （手性刚判出来、模板刚提交）往往就在两次快照之间。而且一个刚过线
        // 就锁定的结论和一个远超阈值的结论，bool 一样，可信度天差地别。
        {
            mocap::pcrec::AutoCalibDbgRec ac;
            ac.wallNs = hd.wallNs;
            ac.frameTsNs = diag.frameTsNs;
            ac.handSignMm       = float(diag.autoHandSignMm);
            ac.handSignLatMm    = float(diag.handSignLatMm);
            ac.handSignThreshMm = float(diag.handednessMinMm);
            ac.progress         = float(diag.autoProgress);
            ac.bundleRmseMm     = float(diag.autoBundleRmseMm);
            for (int f = 0; f < 5; ++f) {
                ac.anchorResidMm[f]   = float(diag.anchorResidMm[f]);
                ac.anchorSpreadDeg[f] = float(diag.anchorSpreadDeg[f]);
                ac.anchorRigidStd[f]  = float(diag.anchorRigidStd[f]);
                ac.anchorSamples[f]   = int32_t(diag.anchorSamples[f]);
                ac.anchorFitted[f]    = uint8_t(diag.anchorFitted[f]);
                ac.staticSeeded[f]    = uint8_t(diag.staticSeeded[f]);
                for (int d = 0; d < 3; ++d)
                    ac.boneLenMm[f][d] = float(diag.tmplBoneLenMm[size_t(f)][size_t(d)]);
            }
            ac.dorsumTmplDriftMm = float(diag.dorsumTmplDriftMm);
            ac.tmplRejectedRmse  = float(diag.tmplRejectedRmse);
            ac.tmplAppliedRmse   = float(diag.tmplAppliedRmse);
            ac.stage             = int32_t(diag.autoStage);
            ac.bundleRuns        = int32_t(diag.bundleRuns);
            ac.attempts          = int32_t(diag.autoAttempts);
            ac.templateReady     = uint8_t(diag.autoTemplateReady);
            ac.ikUsable          = uint8_t(diag.autoIkUsable);
            ac.handednessKnown   = uint8_t(diag.autoHandednessKnown);
            ac.handednessAgree   = uint8_t(diag.handednessAgree);
            ac.isRightDetected   = uint8_t(diag.autoIsRightDetected);
            ac.isRightApplied    = uint8_t(diag.autoIsRight);
            ac.mirrorSuspect     = uint8_t(diag.autoMirrorSuspect);
            ac.dorsumReordered   = uint8_t(diag.autoDorsumReordered);
            ac.calibRejected     = uint8_t(diag.calibRejected);
            recorder_.writeAutoCalibDbg(ac);
        }

        // ---- 块 26：全量指派矩阵（Paranoid 档才写）----
        // 【top-3 不够的地方】"这个点为什么被判成鬼点"的答案在分布的形状里：
        // 分布平坦 = 模型没主意（去看点质量和标定）；次高紧贴最高 = 被邻近
        // 标签抢走了（去看阈值和指派）。两者修法不同，而 top-3 看不出形状。
        if (!result.logAssignFull.empty())
            recorder_.writeAssignFull(diag.frameTsNs > 0 ? diag.frameTsNs : hd.wallNs,
                                      result.logAssignRows, result.logAssignCols,
                                      result.logAssignFull.data());

        // ---- 全量配置转储（1Hz）----
        // 【这一项是离线调参的前提】要把某一级的参数调到最优，必须同时有
        // 那一级的输入、参数、输出。输入输出上面各块都有了，参数就在这里 ——
        // 而 stateJson 只覆盖 hm20 和自标定，滤波/IK/ROM/限幅/角度平滑
        // 一个都没有，恰恰是最需要调的几个。走 ParamDelta 块，跟相机标定同路。
        if (!diag.configJson.empty())
            recorder_.writeParamDelta(hd.wallNs,
                std::string("{\"runtimeConfig\":") + diag.configJson + "}");

        // ---- 块 10 / 11：离线复算的最小闭包 ----
        // 【这两个块在 v3 里定义了但从来没被调用过】——录制开始时确实调了
        // setCaptureDebugStreams(true)，result 里也确实填了，然后就丢掉了。
        // 也就是说头文件里郑重写着的"离线复算最小闭包"，实际上一个字节都
        // 没进过文件。没有它们：
        //   · 送进网络的候选点【顺序】不知道 -> 离线的指派结果永远对不上在线
        //   · 网络原始输出不知道 -> 分不开"模型判错了"和"后处理搞砸了"
        if (result.hasDebugStreams) {
            const int64_t ats = diag.frameTsNs > 0 ? diag.frameTsNs : hd.wallNs;
            std::vector<std::pair<int, std::array<double,3>>> cand;
            const int nIn = std::min(result.assocInputN, 32);
            cand.reserve(size_t(nIn > 0 ? nIn : 0));
            for (int i = 0; i < nIn; ++i)
                cand.push_back({result.assocInputId[size_t(i)],
                                {result.assocInputPos[size_t(i)][0],
                                 result.assocInputPos[size_t(i)][1],
                                 result.assocInputPos[size_t(i)][2]}});
            recorder_.writeAssocInput(ats, result.assocInputBeforeCap, cand);

            mocap::pcrec::NetRawRec nr;
            nr.tsNs    = ats;
            nr.nCand   = uint16_t(nIn > 0 ? nIn : 0);
            nr.hasPose = uint8_t(result.hasAiJointAng);
            nr.hasSegR = uint8_t(result.netHasSegR);
            for (int m = 0; m < 20; ++m) {
                for (int d = 0; d < 3; ++d)
                    nr.pos[m*3+d] = float(result.netPos[size_t(m)][size_t(d)]);
                nr.missLogit[m] = float(result.netMissLogit[size_t(m)]);
                nr.jointAng[m]  = float(result.netJointAng[size_t(m)]);
            }
            for (int d = 0; d < 3; ++d) nr.center[d] = float(result.netCenter[size_t(d)]);
            nr.scale     = float(result.netScale);
            nr.handLogit = float(result.netHandLogit);
            for (int f = 0; f < 5; ++f) nr.poseConf[f] = float(result.netPoseConf[size_t(f)]);
            if (result.netHasSegR) {
                // segR 是 3x3，网络那头是 6D（前两列）。这里存前两列，跟
                // NetRawRec::segRot6d 的约定一致：16 段 × 6。
                for (int s = 0; s < 16; ++s)
                    for (int c = 0; c < 2; ++c)
                        for (int r2 = 0; r2 < 3; ++r2)
                            nr.segRot6d[s*6 + c*3 + r2] =
                                float(result.netSegR[size_t(s)][size_t(r2*3 + c)]);
            }
            std::vector<mocap::pcrec::NetTopK> tk;
            tk.reserve(size_t(nIn > 0 ? nIn : 0));
            for (int i = 0; i < nIn; ++i) {
                mocap::pcrec::NetTopK t3;
                for (int k = 0; k < 3; ++k) {
                    t3.label[k] = int16_t(result.assignTop3Label[size_t(i)][size_t(k)]);
                    t3.prob[k]  = float(result.assignTop3Prob[size_t(i)][size_t(k)]);
                }
                tk.push_back(t3);
            }
            recorder_.writeNetRaw(nr, tk);
        }
    }

    if (!showSkeletonChk_->isChecked()) return;   // 结果回来的时候用户可能已经关掉开关了，不画

    if (result.valid) {
        QHash<int, QString> lblMap;      // 追踪点 id -> 语义标签（实测点，标在点云层）
    QHash<int, QString> skelLblMap;  // 骨架点 id -> 语义标签（预测点，标在骨架层）
    QVector<PointCloudWidget::Point> skelPts;
        skelPts.reserve(20);
        for (int i = 0; i < 20; ++i) {
            // 【改动】hm20 里叫 markers，不再是 joints
            const auto& m = result.markers[size_t(i)];
            // 【三档，不是两档】原来只按 observed 分"实测/预测"，把两种完全不同的
            // 状态混成了一种：
            //   ① 真的没有这个点，位置是补出来的        -> 该标预测
            //   ② 点是真的、模型也标对了，只是手背刚体求解这一帧没验过它
            //      -> 位置可信，标成预测会让人以为识别错了
            // 实际现象就是"明明识别到了也对了，为什么还显示预测"。
            // 用 confidence 区分：>0 说明 sourcePointId 指向一个真实候选点。
            const bool trulyPredicted = !m.observed && m.confidence <= 0.0;
            skelPts.push_back({QVector3D(float(m.posWorld[0]), float(m.posWorld[1]), float(m.posWorld[2])),
                               100000 + i, false, trulyPredicted});
            // 把"这个候选点被判成了什么"回传给点云层，取代追踪序号。
            // 【只标真正被观测认领的】m.sourcePointId >= 0 才说明它对应一个
            // 真实候选点；补出来的点没有对应点云点，标了会指到不存在的 id 上。
            const QString shortName = QString::fromLatin1(hm20::labelShortName(i));
            // 实测点：标在点云层（那里画的是真实候选点）
            if (m.sourcePointId >= 0) lblMap.insert(m.sourcePointId, shortName);
            // 预测点：点云层没有它，只能标在骨架层
            else                      skelLblMap.insert(100000 + i, shortName);
        }

        QVector<QPair<int,int>> skelEdges;
        for (auto& e : hm20::skeletonEdges())
            skelEdges.push_back({100000 + int(e.first), 100000 + int(e.second)});
        // 【修复】补上手掌->每指第一节的挂载边——skeletonEdges()只给"指骨间"
        // 的边，挂载边跟具体手型模板有关，见 HandSkeletonAssociator.hpp
        // 里 fingerMountEdges() 的说明(现已移到 Hm20TemplateAdapter.hpp)，不然5根手指渲染出来会跟手掌轮廓
        // 完全脱开。
        // fingerMountEdges 原本随上一代 associator 一起没了，现在在
        // Hm20TemplateAdapter.hpp 里按同样语义重建（最近邻，纯渲染用）
        for (auto& e : hm20::fingerMountEdges(skeletonTemplateForRender_))
            skelEdges.push_back({100000 + int(e.first), 100000 + int(e.second)});

        // 【改动】独立叠加层——不再跟lastCloudPts_拼一起塞进setPoints()，
        // 也不再用setEdges()(那个是"绑定模式"手动连线专用的，跟这里共用
        // 会互相覆盖，见PointCloudWidget::setSkeletonOverlay()的说明)。
        // 点云该多快刷多快，骨架多久刷一次是AI线程自己的节奏，两者完全
        // 独立更新，不会因为谁比谁慢就互相清空/闪烁。
        view_->setSkeletonOverlay(skelPts, skelEdges);
        view_->setPointLabels(lblMap);
        view_->setSkeletonLabels(skelLblMap);
        skeletonOverlayActive_ = true;
    } else {
        if (skeletonOverlayActive_) {
            view_->setSkeletonOverlay({}, {});
            // 【两处清空点都要清标签】否则骨架关掉后，上一次的标签会一直挂在
            // 点云上，指向早就换掉的追踪 id —— 看起来像"标错了"。
            view_->setPointLabels({});
            view_->setSkeletonLabels({});
            skeletonOverlayActive_ = false;
        }
    }

    const double dropRatio = (skeletonSubmittedFrames_ + skeletonDroppedFrames_) > 0
        ? double(skeletonDroppedFrames_) / double(skeletonSubmittedFrames_ + skeletonDroppedFrames_)
        : 0.0;
    // 窗口滚动清零——只反映"最近这一段"的丢帧情况，不然跑久了老数据把
    // 比例摊薄，看不出最近是不是卡了。
    if (skeletonSubmittedFrames_ + skeletonDroppedFrames_ > 300) {
        skeletonSubmittedFrames_ = 0; skeletonDroppedFrames_ = 0;
    }

    // ---- 诊断文字 ----
    // 【限流，别每帧刷】文字里的耗时数字每帧都在变，而 setText 会触发
    // 布局重算。AI每秒能出上百个结果，每次都刷 = GUI线程被布局计算吃满。
    // 这里做两层保护：①短文本进标签(固定宽度、不换行，不影响布局)，
    // 完整信息进 tooltip；②除非"状态类别"变了(可用<->不可用<->冷启动
    // 失败)，否则最多 4Hz 刷新一次——出错时用户能立刻看到，正常运行时
    // 数字慢点跳完全不影响判断。
    QString shortText, fullText;
    int stateKind = 0;
    if (!diag.backendReady) {
        stateKind = 1;
        // 失败原因由worker线程在初始化时抓到、通过diag.stageMessage带回来——
        // GUI线程不再持有backend(onnxruntime的一切都在worker线程里)。
        const QString detail = QString::fromStdString(diag.stageMessage);
        shortText = QStringLiteral("骨骼AI：不可用（悬停查看原因）");
        fullText  = QStringLiteral("骨骼AI：不可用 —— %1").arg(detail);
    } else if (!diag.palmOk) {
        stateKind = 2;
        shortText = QStringLiteral("骨骼AI：等待手掌识别（候选点%1）").arg(diag.candidateCount);
        fullText  = QStringLiteral("骨骼AI：卡在手掌冷启动(非AI那步) —— %1（候选点%2个）")
                        .arg(QString::fromStdString(diag.stageMessage)).arg(diag.candidateCount);
    } else {
        stateKind = 3;
        // 【改动】末尾补一个腕部位姿标志。wristPoseValid==false 表示手背可见
        // 点不足3个、腕部位姿是沿用上一帧外推的，此时 segQuat[0] 和所有手指段
        // 的参考系都不可信。做数据采集时这个标志应该一路带进录制文件，事后
        // 能筛掉这些帧。
        shortText = QStringLiteral("骨骼AI：生效 %1ms 观测%2/预测%3 丢帧%4%%5")
                        .arg(diag.latencyMs, 0, 'f', 1)
                        .arg(diag.observedJoints).arg(diag.predictedJoints)
                        .arg(dropRatio*100.0, 0, 'f', 0)
                        .arg(diag.wristPoseValid ? QString() : QStringLiteral(" [腕部外推]"));
        fullText  = QStringLiteral("骨骼AI：生效中，单帧耗时%1ms，15个手指关节中%2个真实观测/"
                                   "%3个AI预测填补，本线程丢帧率%4%（忙时跳过，不影响点云面板流畅度）")
                        .arg(diag.latencyMs, 0, 'f', 1)
                        .arg(diag.observedJoints).arg(diag.predictedJoints)
                        .arg(dropRatio*100.0, 0, 'f', 0);
    }

    // ---- 手背刚体求解状态：单独一行 ----
    // 【为什么值得占一行】手背标签一翻，腕部系整个转 180°，全手的连线一起错，
    // 但画面上看起来只是"手突然朝向不对"，很难跟"标签翻了"联系起来。
    // 把内点数/残差/裕度摆出来，翻不翻当场可见。
    if (diag.backendReady && diag.dorsumReason >= 0) {
        static const char* kReason[5] = {
            "", "候选点不足", "无可行解", "歧义拒解(本帧不给腕部位姿)", "残差超限"};
        QString d;
        if (diag.dorsumReason == 0) {
            d = QStringLiteral("手背刚体：内点%1/5  残差%2mm  裕度%3mm%4")
                    .arg(diag.dorsumInliers)
                    .arg(diag.dorsumRmseMm, 0, 'f', 2)
                    .arg(diag.dorsumMarginMm > 1e8 ? 999.0 : diag.dorsumMarginMm, 0, 'f', 1)
                    .arg(diag.dorsumByHistory ? QStringLiteral("  [连续性仲裁]") : QString());
        } else {
            const int r = std::clamp(diag.dorsumReason, 0, 4);
            d = QStringLiteral("手背刚体：未解出 —— %1").arg(QString::fromUtf8(kReason[r]));
        }
        if (diag.dorsumSelfAmbMm > 0.0) {
            d += QStringLiteral("   自歧义%1mm%2")
                     .arg(diag.dorsumSelfAmbMm, 0, 'f', 2)
                     .arg(diag.dorsumSelfAmbMm < 3.0
                              ? QStringLiteral(" ← 贴点布局病态，重贴比调参数管用")
                              : QString());
        }
        if (diag.netThumbSegs > 0)
            d += QStringLiteral("   拇指段走网络×%1").arg(diag.netThumbSegs);
        fullText += QStringLiteral("\n") + d;
    }

    // ---- 推理耗时和当前模型 ----
    // 【模型名要显示出来】高速模式切换失败会静默退回原模型，
    // 不显示的话用户会以为切过去了，然后困惑于"为什么没变快"。
    if (!diag.loadedModelName.isEmpty()) {
        fullText += QStringLiteral("\n推理：%1   单帧 %2ms")
                        .arg(diag.loadedModelName)
                        .arg(diag.latencyMs, 0, 'f', 1);
    }

    // ---- 拇指遮挡跟随 / 贴点重捕：这一行是判断"改动有没有落地"的入口 ----
    // 【为什么单独一行】这两件事都是"平时看不见、出事才知道"的类型。
    // 不把计数器摆出来，就只能靠肉眼看画面猜 —— 而这次三个最费时间的 bug
    // （手性没下发、chainContinue 默认关、swing 半截）全都是"代码看着在跑、
    // 计数器却是 0"，摆出来能省掉整轮排查。
    {
        QStringList bits;
        if (diag.thumbTipOccluded) {
            if (diag.thumbIpPredRad > -90.0) {
                bits << QStringLiteral("拇指尖遮挡 跟随IP %1°(已跟 %2°)")
                            .arg(diag.thumbIpPredRad * 180.0 / M_PI, 0, 'f', 0)
                            .arg(diag.thumbIpDriftRad * 180.0 / M_PI, 0, 'f', 1);
            } else {
                bits << QStringLiteral("拇指尖遮挡 但跟随未生效(chainContinue 关?)");
            }
        }
        if (diag.chainContinued > 0)
            bits << QStringLiteral("链式续解×%1").arg(diag.chainContinued);
        // 掌心相对耦合：中远节都被遮时，靠"近端那颗可见球相对手掌怎么动"
        // 推出来的继续屈曲量。只在真的推了才显示，免得刷屏。
        // 【怎么看】手指在遮挡期弯下去而这里一直是 0 —— 说明近节相对手掌没动
        // （多半是整只手在平移），不是耦合坏了；若同时 chainContinued 也是 0，
        // 那就是续解根本没跑（骨长/平面还没学到，或 chainContinue 关着）。
        {
            static const char* fn[5] = {"拇", "食", "中", "无", "小"};
            QStringList cp;
            for (int f = 0; f < 5; ++f)
                if (std::fabs(diag.chainCoupledDeg[f]) > 1.0)
                    cp << QStringLiteral("%1%2°").arg(QString::fromUtf8(fn[f]))
                          .arg(diag.chainCoupledDeg[f], 0, 'f', 0);
            if (!cp.isEmpty())
                bits << QStringLiteral("遮挡续弯 %1").arg(cp.join(QStringLiteral(" ")));
        }
        if (diag.dorsumBadStreak > 5)
            bits << QStringLiteral("⚠连续解不好×%1").arg(diag.dorsumBadStreak);
        if (diag.dorsumRelock == 1) bits << QStringLiteral("[看门狗:已松开连续性锁]");
        if (diag.dorsumRelock == 2) bits << QStringLiteral("[看门狗:模板已退回标定值]");
        if (diag.tmplRejectedRmse > 0.0)
            bits << QStringLiteral("⚠自标定模板被挡(残差%1 > 在用的%2)")
                        .arg(diag.tmplRejectedRmse, 0, 'f', 1)
                        .arg(diag.tmplAppliedRmse, 0, 'f', 1);
        if (diag.dorsumTmplDriftMm > 0.5)
            bits << QStringLiteral("模板已被重捕改动%1mm").arg(diag.dorsumTmplDriftMm, 0, 'f', 1);
        if (diag.dorsumRepaired > 0)
            bits << QStringLiteral("★手背模板已修复 槽位%1 移动%2mm")
                        .arg(diag.dorsumRepaired - 1)
                        .arg(diag.dorsumRepairMoveMm, 0, 'f', 1);
        if (!bits.isEmpty())
            fullText += QStringLiteral("\n遮挡处理：") + bits.join(QStringLiteral("   "));
    }

    // ---- 网络段常数的标定结果 ----
    // 【kOffsetDeg 是交叉验证点，不是普通诊断】probe_thumb_axis.py 从模型的
    // seg_rot6d 量到拇指铰链轴缺口 73°；这里的 K 转角是从实测几何反解的同一个量。
    // 两条互不依赖的路对上了(70~90°)才敢开 thumbSegFromNet。所以这行要显眼。
    if (diag.netSegSolved) {
        if (diag.netSegOk) {
            fullText += QStringLiteral("\n网络段常数：C离散%1°  拇指修正 %2°/%3°/%4°%5")
                            .arg(diag.netSegCSpread, 0, 'f', 1)
                            .arg(diag.netSegKOffset[0], 0, 'f', 0)
                            .arg(diag.netSegKOffset[1], 0, 'f', 0)
                            .arg(diag.netSegKOffset[2], 0, 'f', 0)
                            .arg((diag.netSegKOffset[0] >= 60.0 && diag.netSegKOffset[0] <= 100.0)
                                     ? QStringLiteral("  ← 与模型量到的 73° 对上了，可以开 thumbSegFromNet")
                                     : QStringLiteral("  ← 不在 70~90°，先别开，把这个数发出来看"));
        } else {
            fullText += QStringLiteral("\n网络段常数：未解出 —— %1")
                            .arg(QString::fromStdString(diag.netSegWhy));
        }
    }

    // ---- 自标定状态 ----
    // 【单独一行、且必须显示"为什么没推进"】只给一个不动的进度条，用户完全
    // 无从下手。autoReject 会直接说"手动得太快，请放慢"/"手背点没认全"这种
    // 能照着做的话。
    if (autoCalibChk_ && autoCalibChk_->isChecked() && diag.backendReady) {
        stateKind = 100 + diag.autoStage;
        QString a = QString::fromStdString(diag.autoText);
        if (!diag.autoReject.empty())
            a += QStringLiteral("  [%1]").arg(QString::fromStdString(diag.autoReject));
        // 【手性冲突要顶到短文案里】自标定推断的手性跟面板设置不符时，说明
        // 那 4mm 阈值的判据踩在边界上。它现在【不会】再自动改设置，但用户
        // 有权知道 —— 尤其当他确实拿的是左手时，这条提示就是让他去改开关的信号。
        if (diag.autoHandednessConflict)
            shortText += QStringLiteral("\n⚠ 自动判为%1手(依据%2mm)，与设置不符——已按设置走")
                            .arg(diag.autoIsRightDetected ? QStringLiteral("右") : QStringLiteral("左"))
                            .arg(diag.autoHandSignMm, 0, 'f', 1);
        if (diag.autoMirrorSuspect)
            shortText += QStringLiteral("\n⚠ 冻结的手背模板疑似镜像，已拒绝并重采");
        // 【放进常显文字，不是 tooltip】这条是当前排查"标定一提交就错"的关键判据，
        // 需要在标定跑的过程中一直盯着。tooltip 要悬停才出来、还会自己消失，
        // 排查时根本用不上。只在真的触发时才占一行，平时不出现。
        if (diag.autoDorsumWouldReorder || diag.autoDorsumReordered)
            shortText += diag.autoDorsumReordered
                ? QStringLiteral("\n⚠ 手背编号已重排——模板与网络标签可能错位")
                : QStringLiteral("\n手背编号：本来会重排，已按默认关掉（对应关系保持原样）");
        shortText += QStringLiteral("\n") + a;
        fullText  += QStringLiteral("\n%1\n手性：%2%3   IK许可：%4   束调整残差：%5")
                        .arg(a)
                        .arg(diag.autoIsRight ? QStringLiteral("右手") : QStringLiteral("左手"))
                        .arg(diag.autoHandednessKnown ? QString() : QStringLiteral("(未确定，按设置值)"))
                        .arg(diag.autoIkUsable ? QStringLiteral("已放行")
                                               : QStringLiteral("未放行(参数未收敛，IK已自动禁用)"))
                        .arg(diag.autoBundleRmseMm < 0 ? QStringLiteral("—")
                             : QString::number(diag.autoBundleRmseMm, 'f', 2) + QStringLiteral("mm"));
        // 提交那一刻到底改了什么，全写出来 —— "标定阶段正常、一提交就错"这类
        // 问题，能看见提交内容才查得动。
        fullText += QStringLiteral("\n自动判手性：%1(判据%2mm，阈值4mm)%3   手背编号重排：%4")
                        .arg(diag.autoIsRightDetected ? QStringLiteral("右") : QStringLiteral("左"))
                        .arg(diag.autoHandSignMm, 0, 'f', 2)
                        .arg(diag.autoHandednessConflict ? QStringLiteral("  ← 与设置冲突") : QString())
                        .arg(diag.autoDorsumReordered ? QStringLiteral("有")
                             : (diag.autoDorsumWouldReorder
                                ? QStringLiteral("本来会重排，已按默认关掉")
                                : QStringLiteral("无")));
    }
    if (!diag.thumbHint.empty())
        shortText += QStringLiteral("\n拇指：%1  %2")
                        .arg(int(diag.thumbCoverage * 100)).arg(QString::fromStdString(diag.thumbHint));
    // 【遮挡点走哪条路要顶到常显文字里，且【不能套在 ikActive 里】】
    // 上一版把它写进了 if(diag.ikActive) —— 于是 IK 没激活时整条不显示，
    // 而"IK 为什么没激活"恰恰是最需要看见的情况。诊断的可见性不该依赖
    // 被诊断的那个东西正常工作。
    // 【花括号别省】这里已经栽过两次：注释挡住 if 的作用域、单语句 if 里
    // 声明变量。多行体一律带花括号。
    if (diag.ikFilled + diag.ikFallback + diag.chainContinued > 0) {
        // 【别用 %n 占位 + 链式 arg】QString::arg 按【编号最小】替换，上一版
        // 串里没有 %2，于是第一个 arg(ikFallback) 抢占了 %3 的位置，链式那栏
        // 印成了"(IK未激活)" —— 诊断行自己先错了，看不出链式到底补了几个。
        // 改成一次性 arg 顺序拼，编号和参数一一对应，不会再串。
        const int aiCnt = std::max(0, diag.ikFallback - diag.chainContinued - diag.occludedHeld);
        shortText += QStringLiteral("\n遮挡点：IK%1 / 链式%2 / AI%3 / 保持%4%5")
                        .arg(diag.ikFilled)
                        .arg(diag.chainContinued)
                        .arg(aiCnt)
                        .arg(diag.occludedHeld)
                        .arg(diag.ikActive ? QString() : QStringLiteral("（IK未激活）"));
    }
    // 手背 tracklet 的工作情况。【纠正数长期为 0】说明单帧判断本来就对，
    // 这条没白做也没坏事；【长期 >0】说明它每帧都在救场，那才是它的价值。
    if (diag.dorsumTrackN > 0) {
        shortText += QStringLiteral("\n手背 轨迹%1帧").arg(diag.dorsumTrackN);
        if (diag.dorsumTrackFixed > 0)
            shortText += QStringLiteral(" 纠正%1").arg(diag.dorsumTrackFixed);
        // 几何重定的裕度：正常 7mm+，接近 0 说明这帧观测有问题(粘连/重影)
        if (diag.dorsumGeoMarginMm >= 0)
            shortText += QStringLiteral(" 几何裕度%1mm")
                            .arg(diag.dorsumGeoMarginMm, 0, 'f', 1);
    }
    // ---- v7 姿态版：模型直接给的手性和姿态 ----
    // 【为什么并排显示而不是直接替换】几何链在观测充分时更准(实测张开手
    // 中节 5° 以内，模型 7.3°)，遮挡时则远差(骨轴中位 21.4°、max 107.9°)。
    // 先看着两者的差，确认符号/量级对得上、遮挡时模型确实更好，再决定
    // 要不要在下游做加权融合。直接换掉是这个项目已经犯过几次的错。
    // 【当前生效的手性常驻显示】它决定 IK 镜像和网络条件向量，
    // 设反了整只手是镜像的 —— 这种错在画面上不明显（手看着还是手），
    // 但下游全错。所以让它一直可见，不要藏在勾选框里。
    {
        const bool panelRight = handRightChk_ && handRightChk_->isChecked();
        shortText += QStringLiteral("\n手性：%1（面板设定，生效值）")
                        .arg(panelRight ? QStringLiteral("右手") : QStringLiteral("左手"));
    }
    if (diag.hasAiHand) {
        // 【模型手性只作参考，不改任何东西】
        // 模型这一路的训练准确率标称 98.5%，但【线上这个工作点对不上】：
        // 拿 hm20_v7 在真机录制上实测，|hand_logit| 中位只有 0.044，而喂
        // 纯随机点云能给到 0.196 —— 也就是随机噪声比真手还"确信是右手"。
        // 在这个量级上 `logit > 0` 这个判据没有意义。
        //
        // 所以显示分三态。没有"未定"这一档的话，就只能在两个都没依据的
        // 答案里挑一个再标上"已锁定" —— 左手用户看到的就是自信的"右手"。
        const bool panelRight = handRightChk_ && handRightChk_->isChecked();
        if (!diag.aiHandKnown) {
            shortText += QStringLiteral("\nAI判断：未定（证据不足，以面板设定为准）");
        } else {
            const bool disagree = diag.aiHandLocked && (diag.aiHandIsRight != panelRight);
            shortText += QStringLiteral("\nAI判断：%1 置信%2%3%4")
                            .arg(diag.aiHandIsRight ? QStringLiteral("右手") : QStringLiteral("左手"))
                            .arg(int(diag.aiHandConf * 100))
                            .arg(diag.aiHandLocked ? QStringLiteral("(已锁定)")
                                                   : QStringLiteral("(投票中)"))
                            .arg(disagree ? QStringLiteral("  ⚠与设定不符，请核对面板的「右手」勾选")
                                          : QStringLiteral("  (仅供参考)"));
        }
    }
    if (diag.hasAiPose) {
        // 【拆成两行】原来挤在一行里，面板宽度不够会被右边的控件挡掉后半截。
        // 姿态和置信度各占一行，每行都短，窄面板也能看全。
        shortText += QStringLiteral("\nAI姿态 拇%1 食%2/%3 中%4°")
                        .arg(diag.aiJointDeg[0], 0, 'f', 0)
                        .arg(diag.aiJointDeg[4], 0, 'f', 0)
                        .arg(diag.aiJointDeg[6], 0, 'f', 0)
                        .arg(diag.aiJointDeg[9], 0, 'f', 0);
        QString pc;
        for (int f = 0; f < 5; ++f)
            pc += QStringLiteral("%1 ").arg(diag.aiPoseConf[f], 0, 'f', 2);
        shortText += QStringLiteral("\n逐指可信 ") + pc;
    }
    // 逐指 IK 残差：残差大=anchor/骨长跟这只手对不上或标签串了；
    // 残差正常却仍不生效=可见 marker 不够，是几何遮挡。两者解法完全不同。
    if (diag.ikActive) {
        QString rs;
        static const char* const fn[5] = {"拇", "食", "中", "无", "小"};
        for (int f = 0; f < 5; ++f)
            rs += QStringLiteral("%1%2 ").arg(QString::fromUtf8(fn[f]))
                     .arg(diag.ikRmsePerFinger[f] < 0 ? QStringLiteral("—")
                          : QString::number(diag.ikRmsePerFinger[f], 'f', 1));
        shortText += QStringLiteral("\nIK残差 ") + rs;
        // 【anchor 拟合质量】IK 残差大到底是 anchor 不准、还是这根手指本来就
        // 没解出来，光看 IK 残差分不清。把三件套摆出来：
        //   ✓/✗ 解没解出来   球面残差   角度覆盖(°)   样本数
        // 残差大 = 样本不在一个球面上(标签串/腕部系抖)；覆盖小 = 手指没怎么动，
        // 球心在轴向上不可辨识。这两种的解法完全不同。
        QString as;
        for (int f = 0; f < 5; ++f) {
            as += QStringLiteral("%1%2 ").arg(QString::fromUtf8(fn[f]))
                     .arg(!diag.anchorFitted[f]
                          ? QStringLiteral("✗%1样").arg(diag.anchorSamples[f])
                          : diag.staticSeeded[f]
                          ? QStringLiteral("静%1mm").arg(diag.staticSeedDevMm[f], 0, 'f', 1)
                          : QStringLiteral("%1mm/%2°").arg(diag.anchorResidMm[f], 0, 'f', 1)
                                                      .arg(diag.anchorSpreadDeg[f], 0, 'f', 0));
        }
        shortText += QStringLiteral("\nanchor ") + as;
        // 【这一行才是判断 anchor 对不对的依据】上面那行是拟合残差(自己评自己)，
        // 这行是 |anchor-pp| 的跨帧标准差 —— 两点在同一根骨头上，距离恒定，
        // std 应该落在观测噪声量级(1~2mm)。大于 5mm 基本可以断定 anchor 是错的。
        QString rg;
        bool anyRigid = false;
        for (int f = 0; f < 5; ++f) {
            if (diag.anchorRigidStd[f] < 0) { rg += QStringLiteral("%1— ").arg(QString::fromUtf8(fn[f])); continue; }
            anyRigid = true;
            rg += QStringLiteral("%1%2 ").arg(QString::fromUtf8(fn[f]))
                     .arg(diag.anchorRigidStd[f], 0, 'f', 1);
        }
        // 验收闸门拦下时必须说出来 —— 否则用户只看到"标定跑完了但没变化"，
        // 完全不知道系统其实是【主动拒绝】了一个坏结果。
        if (diag.calibRejected == 1)
            shortText += QStringLiteral("\n⚠ 自标定未通过验收(刚性σ超标)，已保持原状");
        if (anyRigid)
            shortText += QStringLiteral("\n刚性σ ") + rg + QStringLiteral("mm(应<2)");
    }
        fullText += QStringLiteral("\nIK：%1指生效，残差中位%2mm")
                        .arg(diag.ikFingerCount).arg(diag.ikRmseMm, 0, 'f', 2);

    const bool stateChanged = (stateKind != skeletonDiagLastKind_);
    if (!skeletonDiagThrottle_.isValid()) skeletonDiagThrottle_.start();
    if (stateChanged || skeletonDiagThrottle_.elapsed() >= 250) {
        skeletonDiagThrottle_.restart();
        skeletonDiagLastKind_ = stateKind;
        if (shortText != skeletonDiagLabel_->text()) skeletonDiagLabel_->setText(shortText);
        // ---- 滞后读数 ----
        // 陈旧度取"实际刷新间隔的一半"：这条线画上去之后平均还要停留这么久才被
        // 换掉，是它相对真实动作多出来的平均滞后。用实测间隔而不是设定的限流值，
        // 因为 worker 跟不上时真实间隔会比设定值大（那才是你眼睛看到的）。
        if (latencyLabel_) {
            const double queueMs = std::max(0.0, emaE2eMs_ - diag.latencyMs);
            const double staleMs = (emaSkelIntervalMs_ > 0) ? emaSkelIntervalMs_ * 0.5 : 0.0;
            const double totalMs = (emaE2eMs_ > 0 ? emaE2eMs_ : diag.latencyMs) + staleMs;
            // 主干只留"总滞后 + 骨架帧率"——这两个数直接对应你眼睛看到的
            // "线比点慢多少"和"卡不卡"。三项拆分是排查时才要看的，进 tooltip。
            setBriefText(latencyLabel_,
                QStringLiteral("滞后 %1ms   骨架 %2Hz")
                    .arg(totalMs, 0, 'f', 0)
                    .arg(emaSkelIntervalMs_ > 0 ? 1000.0 / emaSkelIntervalMs_ : 0.0, 0, 'f', 0),
                QStringLiteral(
                    "总滞后 %1ms = 推理 %2 + 排队 %3 + 陈旧 %4\n"
                    "  推理  模型跑一帧的时间\n"
                    "  排队  帧在队列里等待处理的时间（worker 跟不上就涨）\n"
                    "  陈旧  这条线画上去之后平均还要停留多久才被换掉\n"
                    "        （取实际刷新间隔的一半）\n"
                    "骨架输出 %5Hz")
                    .arg(totalMs, 0, 'f', 0)
                    .arg(diag.latencyMs, 0, 'f', 1)
                    .arg(queueMs, 0, 'f', 1)
                    .arg(staleMs, 0, 'f', 0)
                    .arg(emaSkelIntervalMs_ > 0 ? 1000.0 / emaSkelIntervalMs_ : 0.0, 0, 'f', 0));
        }
    if (recorder_.recording() && recStatusLabel_) {
        recStatusLabel_->setText(QStringLiteral("录制中 %1s  %2MB  丢块%3")
            .arg(recTimer_.elapsed() / 1000)
            .arg(double(recorder_.bytesWritten()) / 1048576.0, 0, 'f', 1)
            .arg(recorder_.droppedChunks()));
    }
        skeletonDiagLabel_->setToolTip(fullText);
    }
}

void PointCloudTestDialog::rebuild() {
    // 断开之前订阅的信号——不管上次是哪种模式，两个信号都尝试断开一次，
    // disconnect对没连过的信号是安全的no-op，不需要额外记录"上次是哪种"。
    for (const auto& e : activeCams_) {
        disconnect(e.cam, &ICamera::blobsReady, this, &PointCloudTestDialog::onBlobs);
        disconnect(e.cam, &ICamera::blobDetailsReady, this, &PointCloudTestDialog::onBlobDetails);
    }
    // 归还上一轮借用的轮廓采集开关——同下面"借"的逻辑对称，相机组合变了
    // 或者切回质心法，都不该让这些相机白白继续多算轮廓。
    for (ICamera* c : weTurnedOnContour_) c->setContourCollectionEnabled(false);
    weTurnedOnContour_.clear();
    activeCams_.clear();
    latestBlobs_.clear();
    // 用控件当前值(可能来自QSettings存档，也可能是用户刚改的)构造，不再
    // 是写死的20.0/36/6——首次进 rebuild() 是构造函数末尾调用的，那时候
    // 三个spinbox已经建好、值已经从settings_加载完毕，不存在读到默认
    // 构造spinbox(值为0)的时序问题。
    tracker_ = TemporalTracker(assocDistSpin_->value(), maxMissedSpin_->value(), minHitsSpin_->value());
    tracker_.setUseOptimalAssignment(useHungarianChk_->isChecked());
    tracker_.setUseAdaptiveAssocCap(useAdaptiveCapChk_->isChecked());
    tracker_.setAdaptiveCapMultiplier(adaptiveCapMulSpin_->value());
    tracker_.setUseMissedFrameRelax(useMissedRelaxChk_->isChecked());
    tracker_.setRelaxGrowthPerMissedFrame(relaxGrowthSpin_->value());
    tracker_.setRelaxCapMultiplier(relaxCapSpin_->value());
    tracker_.setUseVelocitySmoothing(useVelSmoothChk_->isChecked());
    tracker_.setVelocitySmoothingAlpha(velSmoothAlphaSpin_->value());
    tracker_.setUseConstantAcceleration(useConstAccelChk_->isChecked());
    iekfTracker_ = IekfPointTracker(assocDistSpin_->value(), maxMissedSpin_->value(), minHitsSpin_->value());
    iekfTracker_.setPosProcessVar(iekfPosProcessVarSpin_->value());
    iekfTracker_.setVelProcessVar(iekfVelProcessVarSpin_->value());
    // 新建的iekfTracker_要把自适应门控开关的当前UI值一并灌进去(否则新构造的
    // 实例只带头文件默认值，跟界面上显示的不一致)。
    iekfTracker_.setUseMahalanobisGate(iekfMahaGateChk_->isChecked());
    iekfTracker_.setChiSquareGate(iekfChiSquareSpin_->value());
    iekfTracker_.setUseDualAnchorGate(iekfDualAnchorChk_->isChecked());
    iekfTracker_.setUseCoastGhostSuppression(iekfCoastSuppressChk_->isChecked());
    iekfTracker_.setVelCapGain(iekfVelCapGainSpin_->value());
    iekfTracker_.setUseManeuverAdaptiveQ(iekfManeuverQChk_->isChecked());
    iekfTracker_.setMaxConfirmedTracks(iekfMaxConfirmedSpin_->value());
    iekfTracker_.setUseDensityGate(iekfDensityGateChk_->isChecked());
    iekfTracker_.setDensityReachMult(iekfDensityReachSpin_->value());
    iekfTracker_.setDensityAmbiguitySep(iekfDensitySepSpin_->value());
    iekfTracker_.setUseGlobalAssignment(iekfGlobalAssignChk_->isChecked());
    iekfTracker_.setUseTentativeVelRamp(iekfTentRampChk_->isChecked());
    iekfTracker_.setTentativeRampHits(iekfTentRampHitsSpin_->value());
    iekfTracker_.setConfirmedCapMax(iekfConfirmedCapMaxSpin_->value());
    iekfFloorVarSpin_->setEnabled(!iekfUseAutoFloorChk_->isChecked());
    iekfTracker_.setAssocPosFloorVar(iekfUseAutoFloorChk_->isChecked() ? -1.0 : iekfFloorVarSpin_->value());
    iekfTracker_.setManeuverResidThresh(iekfManeuverThreshSpin_->value());
    iekfTracker_.setManeuverBoostMax(iekfManeuverBoostMaxSpin_->value());
    iekfTracker_.setManeuverBoostDecay(iekfManeuverDecaySpin_->value());
    // 【低延迟直通会压掉这一项】直通模式下不改控件本身的勾选状态（免得覆盖
    // 用户设置），只是这一路不生效；退出直通立刻恢复。
    const bool lowLat = lowLatencyChk_ && lowLatencyChk_->isChecked();
    tracker_.setUseOutputFilter(!lowLat && useOutputFilterChk_->isChecked());
    tracker_.setOutputFilterParams(outputFilterMinCutoffSpin_->value(), outputFilterBetaSpin_->value());
    iekfTracker_.setUseOutputFilter(!lowLat && useOutputFilterChk_->isChecked());
    iekfTracker_.setOutputFilterParams(outputFilterMinCutoffSpin_->value(), outputFilterBetaSpin_->value());
    applyIekfDefaultObsVar();   // 用当前已标定相机的fx把"检测噪声(px)"换算成defaultObsVar
    applyIekfObsFloor();
    resetStabilityStats();   // 相机组合变了/重建tracker_本来就该视为"新的一段统计"
    // 【新增】重建 iekfTracker_ 时，观测时刻补偿的推导状态也要一并清空——
    // 相机数量可能变了、帧率可能不同，旧的帧周期估计(emaFrameNs_)和旧的
    // 每相机偏移(camOffEma_)残留到新一段会给出错误的 dt。lastRefTsNs_ 也要
    // 清，否则跨相机切换那一帧会算出一个巨大的假 dt。
    lastRefTsNs_ = -1;
    emaFrameNs_  = -1.0;
    camOffEma_.clear();

    QVector<ICamera*> chosen;
    for (int i = 0; i < list_->count(); ++i) {
        QListWidgetItem* item = list_->item(i);
        if (item->checkState() != Qt::Checked) continue;
        if (ICamera* c = mgr_->byId(item->data(Qt::UserRole).toUInt())) chosen << c;
    }

    // 同 TriangulationDebugDialog：按当前勾选动态借/还检测开关。
    if (!wasDetectOn_) {
        for (auto it = weTurnedOn_.begin(); it != weTurnedOn_.end(); ) {
            if (!chosen.contains(*it)) { (*it)->setDetectEnabled(false); it = weTurnedOn_.erase(it); }
            else ++it;
        }
        for (ICamera* c : chosen) {
            if (!weTurnedOn_.contains(c)) { c->setDetectEnabled(true); weTurnedOn_.insert(c); }
        }
    }

    // 圆拟合法需要轮廓点——只在这个模式下才借用setContourCollectionEnabled，
    // 质心法完全不需要，不多产生这份开销。"借"的规则：这台相机轮廓采集
    // 当前确实是关着的才由我们打开并记住(意味着我们负责关)；已经开着
    // (比如同时挂着HandTrackingWorker)就不动它，也不会记进
    // weTurnedOnContour_，自然不会在归还时被误关掉。
    if (useCircleFitChk_->isChecked()) {
        for (ICamera* c : chosen) {
            if (!c->contourCollectionEnabled()) {
                c->setContourCollectionEnabled(true);
                weTurnedOnContour_.insert(c);
            }
        }
    }

    if (chosen.size() < 2) {
        status_->setText(QStringLiteral("至少勾选两台已标定的相机"));
        view_->setPoints({});
        return;
    }

    for (ICamera* c : chosen) {
        activeCams_.push_back({c, c->deviceKey()});
        if (useCircleFitChk_->isChecked())
            connect(c, &ICamera::blobDetailsReady, this, &PointCloudTestDialog::onBlobDetails);
        else
            connect(c, &ICamera::blobsReady, this, &PointCloudTestDialog::onBlobs);
    }
    status_->setText(QStringLiteral("已连接 %1 台相机（%2），等待观测…")
        .arg(chosen.size())
        .arg(useCircleFitChk_->isChecked() ? QStringLiteral("圆拟合法") : QStringLiteral("质心法")));
}

// 手性改变时重新合成模板并下发。【不重建 worker/线程】——那会重新加载
// onnxruntime（Debug 下接近 1 秒），而这里只是换一位手性标志。
void PointCloudTestDialog::rebuildSkeletonTemplate() {
    if (!skeletonWorker_ || !handTemplateStore_) return;
    const bool isRight = handRightChk_ ? handRightChk_->isChecked() : true;
    hm20::Hm20Template t = hm20::makeHm20Template(handTemplateStore_->data(), isRight);
    skeletonTemplateForRender_ = t;
    skeletonWorker_->updateTemplate(t);
    skeletonWorker_->setHandedness(isRight);
}

std::string PointCloudTestDialog::buildRecordHeader() const {
    // 参数快照。【必须逐个列，不能偷懒】一个月后没人说得清某段数据当时
    // 开没开某个开关，而参数快照是唯一的凭据。漏掉的字段等于永久丢失。
    static const char* kProto[] = {"static_hold","rigid_motion","finger_flex",
                                   "occlusion","free_motion","rigid_body_gt",
                                   "fist_open_cycle","handedness_check"};
    QJsonObject params;
    auto putD = [&](const char* k, double v) { params[QLatin1String(k)] = v; };
    auto putI = [&](const char* k, int v)    { params[QLatin1String(k)] = v; };
    auto putB = [&](const char* k, bool v)   { params[QLatin1String(k)] = v; };

    putB("useIekfBackend",  useIekfBackendChk_->isChecked());
    putB("useCircleFit",    useCircleFitChk_->isChecked());
    putB("useMahalanobis",  useMahalanobisChk_->isChecked());
    putD("assocDist",       assocDistSpin_->value());
    putI("maxMissed",       maxMissedSpin_->value());
    putI("minHits",         minHitsSpin_->value());
    putD("twoViewMinRayAngle", twoViewMinRayAngleSpin_->value());
    putD("maxFinalResidual",   maxFinalResidualSpin_->value());
    putD("outputFilterMinCutoff", outputFilterMinCutoffSpin_->value());
    putD("outputFilterBeta",      outputFilterBetaSpin_->value());
    putB("showSkeleton",   showSkeletonChk_->isChecked());
    putB("useIkRefine",    useIkRefineChk_->isChecked());
    putB("autoCalib",      autoCalibChk_->isChecked());
    putI("smoothPreset",   smoothCombo_->currentIndex());

    // ---- 聚类 / 三角化参数（上一轮录制全部漏掉了）----
    // 【为什么漏掉是致命的】"每帧只出 16 个簇（应为 20）"是你的录制里确认的
    // 第二大问题，而漏了这组参数的话，我连"当时用的什么阈值"都不知道，
    // 更不可能离线扫参。这不是"多记一点总没坏处"，是"没有它就没法做调参"。
    putD("maxSampson",        maxSampsonSpin_->value());
    putD("maxReproj",         maxReprojSpin_->value());
    putI("minSupport",        minSupportSpin_->value());
    putB("useVoting",         useVotingChk_->isChecked());
    putD("minRayAngle",       minRayAngleSpin_->value());
    putD("ambiguityMargin",   ambiguityMarginSpin_->value());
    putD("lmHuberDelta",      lmHuberDeltaSpin_->value());
    putI("lmMaxIters",        lmMaxItersSpin_->value());
    putD("calibSigmaNorm",    calibSigmaNormSpin_->value());
    putB("twoViewFallback",   twoViewFallbackChk_->isChecked());
    putB("useLmRefine",       useLmRefineChk_->isChecked());
    putD("clusterChiSquare",  clusterChiSquareSpin_->value());

    // ---- IEKF 追踪后端参数 ----
    // 你用的是 IEKF 后端。上一轮录制里追踪 ID 重生了 207 次（20 个点
    // 理想只要 20 个 ID），这组参数是排查它的唯一依据。
    if (useIekfBackendChk_->isChecked()) {
        if (iekfFloorVarSpin_)       putD("iekfFloorVar",       iekfFloorVarSpin_->value());
        if (iekfChiSquareSpin_)      putD("iekfChiSquareGate",  iekfChiSquareSpin_->value());
        if (iekfPosProcessVarSpin_)  putD("iekfPosProcessVar",  iekfPosProcessVarSpin_->value());
        if (iekfVelProcessVarSpin_)  putD("iekfVelProcessVar",  iekfVelProcessVarSpin_->value());
        if (iekfDualAnchorChk_)      putB("iekfDualAnchor",     iekfDualAnchorChk_->isChecked());
        if (iekfGlobalAssignChk_)    putB("iekfGlobalAssign",   iekfGlobalAssignChk_->isChecked());
        if (iekfDesyncCompChk_)      putB("iekfDesyncComp",     iekfDesyncCompChk_->isChecked());
        if (iekfMahaGateChk_)        putB("iekfMahaGate",       iekfMahaGateChk_->isChecked());
        if (iekfManeuverQChk_)       putB("iekfManeuverQ",      iekfManeuverQChk_->isChecked());
        if (iekfCoastSuppressChk_)   putB("iekfCoastSuppress",  iekfCoastSuppressChk_->isChecked());
        if (iekfDensityGateChk_)     putB("iekfDensityGate",    iekfDensityGateChk_->isChecked());
        if (iekfDetectNoisePxSpin_)  putD("iekfDetectNoisePx",  iekfDetectNoisePxSpin_->value());
        if (iekfUseAutoFloorChk_)    putB("iekfUseAutoFloor",   iekfUseAutoFloorChk_->isChecked());
        if (iekfVelCapGainSpin_)     putD("iekfVelCapGain",     iekfVelCapGainSpin_->value());
        if (iekfMaxConfirmedSpin_)   putI("iekfMaxConfirmed",   iekfMaxConfirmedSpin_->value());
    }

    // ---- 追踪共用参数 ----
    putB("useHungarian",      useHungarianChk_->isChecked());
    putB("useAdaptiveCap",    useAdaptiveCapChk_->isChecked());
    putD("adaptiveCapMul",    adaptiveCapMulSpin_->value());
    putB("useMissedRelax",    useMissedRelaxChk_->isChecked());
    putD("relaxGrowth",       relaxGrowthSpin_->value());
    // 【补齐这四个】上一轮漏了，而它们全都是面板上可调的 ——
    // 参数快照的意义就是"事后知道当时用的什么值"，漏一个就等于那一项
    // 永远没法离线调（连"当时设的多少"都查不到）。
    if (relaxCapSpin_)         putD("relaxCapMul",   relaxCapSpin_->value());
    if (useVelSmoothChk_)      putB("useVelSmooth",  useVelSmoothChk_->isChecked());
    if (velSmoothAlphaSpin_)   putD("velSmoothAlpha", velSmoothAlphaSpin_->value());
    if (useConstAccelChk_)     putB("useConstAccel", useConstAccelChk_->isChecked());
    if (circleRadiusPxSpin_)   putD("circleRadiusPx", circleRadiusPxSpin_->value());
    if (maxContourPtsSpin_)    putI("maxContourPts",  maxContourPtsSpin_->value());

    // ---- IK RMSE 门限（这一轮的核心发现）----
    // 你的录制里 bundleRmse=20mm，而 cfgIkRmseGate 原来硬编码 4mm，
    // IK 被永久锁死。现在这个值可调了（见 Hm20AutoCalib.hpp 的改动），
    // 记下来才知道"这次录的时候门限放了多少"。
    putD("ikRmseGate",        hm20::Hm20AutoCalib::cfgIkRmseGate());

    // ---- hm20 层（原来一个都没记）----
    // 【为什么补这一块】上面那些全是 2D->3D 那一级的参数。而"拇指外翻"
    // "手背标签翻转"这类问题全发生在 hm20 这一级，它的参数一个都没进快照 ——
    // 等于录了一堆数据却记不下它是在什么设置下录的，事后没法归因。
    if (handRightChk_)   putB("handPanelIsRight", handRightChk_->isChecked());
    // 【单位】控件是【度】，而下发时转了弧度(setThumbRollOffset 里 *M_PI/180)。
    // 这里原来直接写控件值，于是文件里记成 "thumbRollOffsetRad = -180" ——
    // 分析时会被当成 -180 弧度(≈ -28.6 圈)，直接导向错误结论。
    // 运行时一直是对的，只有【记录】是错的，所以特别难发现。
    if (thumbRollSpin_) {
        putD("thumbRollOffsetDeg", thumbRollSpin_->value());
        putD("thumbRollOffsetRad", thumbRollSpin_->value() * M_PI / 180.0);
    }
    if (thumbPronChk_)   putB("thumbPronationOn", thumbPronChk_->isChecked());
    // 下面这些是从上一帧诊断里读的【实际生效值】，比读控件更可信：
    // 控件是"用户想要什么"，诊断是"实际跑的是什么"，两者可以不一致
    // （参数下发有 dirty 标志、自标定会覆盖手性……）。归因要看后者。
    putD("thumbAxialK",            lastDiagSnapshot_.thumbAxialK);
    putD("thumbPronation0",        lastDiagSnapshot_.thumbPronation0);
    putB("thumbPronationFitted",   lastDiagSnapshot_.thumbPronationFitted);
    putD("thumbPronationContrast", lastDiagSnapshot_.thumbPronationContrast);
    putD("thumbCoverage",          lastDiagSnapshot_.thumbCoverage);
    putD("dorsumSelfAmbMm",        lastDiagSnapshot_.dorsumSelfAmbMm);
    putB("autoHandednessKnown",    lastDiagSnapshot_.autoHandednessKnown);
    putB("autoIsRight",            lastDiagSnapshot_.autoIsRight);
    putB("autoIsRightDetected",    lastDiagSnapshot_.autoIsRightDetected);
    putB("autoHandednessConflict", lastDiagSnapshot_.autoHandednessConflict);
    putD("autoHandSignMm",         lastDiagSnapshot_.autoHandSignMm);
    putB("autoMirrorSuspect",      lastDiagSnapshot_.autoMirrorSuspect);
    putB("jointMirrorActive",      lastDiagSnapshot_.jointMirrorActive);
    putB("netSegSolved",           lastDiagSnapshot_.netSegSolved);
    putB("netSegOk",               lastDiagSnapshot_.netSegOk);
    putD("netSegCSpread",          lastDiagSnapshot_.netSegCSpread);
    for (int j = 0; j < 3; ++j)
        params[QStringLiteral("netSegKOffset%1").arg(j)] = lastDiagSnapshot_.netSegKOffset[size_t(j)];
    putD("autoBundleRmseMm",       lastDiagSnapshot_.autoBundleRmseMm);
    for (int f = 0; f < 5; ++f)
        params[QStringLiteral("anchorRigidStd%1").arg(f)] = lastDiagSnapshot_.anchorRigidStd[f];

    QJsonObject h;
    h[QStringLiteral("format")]   = QStringLiteral("mocap_pc_record");
    h[QStringLiteral("version")]  = 1;
    // 【录制详细度必须进文件头】没有它的话，分析脚本看到"没有 FilterDbg 块"
    // 无法区分两种情况：用户选了精简档（正常，该提示"要查这个问题请用完整档
    // 重录"），还是接线坏了没写出来（bug，该报错）。没有这个字段就只能猜一个，
    // 猜错就会把人引向错误的方向。
    h[QStringLiteral("detail")] = recDetailCombo_ ? recDetailCombo_->currentIndex() : 1;
    h[QStringLiteral("detailName")] = QLatin1String(
        (recDetailCombo_ && recDetailCombo_->currentIndex() == 0) ? "basic"
        : ((recDetailCombo_ && recDetailCombo_->currentIndex() == 2) ? "paranoid" : "full"));
    h[QStringLiteral("recorderVersion")] = int(mocap::pcrec::kVersion);
    // ---- 录制清单（v8）----
    // 【它解决的是"这一块为什么不在文件里"】detail 字段只区分开了"被档位
    // 关掉"，剩下两种成因还是分不开：这个 build 根本不认识这个块（版本老），
    // 还是认识、开着、但代码路径一次都没走到（真 bug）。
    // 清单一摆就分开了：清单里没有 = 版本老；enabled=false = 档位关的；
    // enabled=true 而块 35 里 pushed=0 = 真 bug，值得去查。
    //
    // 【从 Recorder 里取而不是在这里拼】门限逻辑在那边，拼两份迟早分叉，
    // 而分叉之后清单说的和实际写的不一致 —— 那种不一致只会在你正拿它
    // 排查问题的时候骗你。
    {
        const int det = recDetailCombo_ ? recDetailCombo_->currentIndex() : 1;
        const QByteArray mj =
            QByteArray::fromStdString(mocap::pcrec::Recorder::manifestJson(det));
        QJsonParseError pe{};
        const QJsonDocument md = QJsonDocument::fromJson(mj, &pe);
        if (pe.error == QJsonParseError::NoError && md.isObject())
            h[QStringLiteral("manifest")] = md.object();
    }
    h[QStringLiteral("created")]  = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    h[QStringLiteral("protocol")] = QLatin1String(
        kProto[qBound(0, recProtoCombo_->currentIndex(), 7)]);
    h[QStringLiteral("note")]     = recNoteEdit_->text();
    h[QStringLiteral("params")]   = params;

    QJsonArray cams;
    for (const auto& e : activeCams_) cams.append(e.deviceKey);
    h[QStringLiteral("cameras")] = cams;

    // 【手部模板必须记】它是关联网络的条件输入，换一份模板整个指派结果就变了。
    // 不记的话，事后没人知道这份数据是在哪套标定下录的，复现无从谈起。
    {
        if (handTemplateStore_) {
        const HandTemplateData& td = handTemplateStore_->data();
        QJsonArray back;
        for (const auto& v : td.backMarkers)
            for (int d = 0; d < 3; ++d) back.append(v[size_t(d)]);
        // 【手指参数也要记】只记 backMarkers 的话，离线重建模板时那 15 个
        // 手指点只能拿群体均值凑，跟在线实际送进网络的条件向量对不上，
        // 复现就不是同一回事了。这次分析里我就吃了这个亏。
        QJsonArray fp;
        for (int f = 0; f < 5; ++f) {
            QJsonObject o;
            QJsonArray an, ln;
            for (int d = 0; d < 3; ++d) an.append(td.fingerParams[size_t(f)].anchor[size_t(d)]);
            for (int d = 0; d < 3; ++d) ln.append(td.fingerParams[size_t(f)].lengths[size_t(d)]);
            o[QStringLiteral("anchor")] = an;
            o[QStringLiteral("lengths")] = ln;
            o[QStringLiteral("dipCoupling")] = td.fingerParams[size_t(f)].dipCoupling;
            o[QStringLiteral("calibrated")] = td.fingerCalibrated[size_t(f)];
            fp.append(o);
        }
        QJsonObject t;
        t[QStringLiteral("backCalibrated")] = td.backCalibrated;
        t[QStringLiteral("backMarkers")] = back;
        t[QStringLiteral("fingerParams")] = fp;
        t[QStringLiteral("isRight")] = handRightChk_ ? handRightChk_->isChecked() : true;
        h[QStringLiteral("template")] = t;
        }
    }
    // 模型身份。换了模型这份调参结论就不成立了，必须留痕。
    // 【光记路径不够】路径一样但文件被换过的情况太常见了（下了个新版覆盖上去）。
    // 记大小 + 前 1MB 的 sha1：全文件 hash 对 140MB 的模型太慢，会卡住录制启动；
    // 前 1MB 足以区分不同导出，且是常数时间。
    {
        QJsonObject m;
        m[QStringLiteral("path")] = recModelPath_;
        QFileInfo mi(recModelPath_);
        m[QStringLiteral("exists")] = mi.exists();
        m[QStringLiteral("sizeBytes")] = qint64(mi.size());
        m[QStringLiteral("modified")] = mi.lastModified().toUTC().toString(Qt::ISODate);
        QFile mf(recModelPath_);
        if (mf.open(QIODevice::ReadOnly)) {
            QCryptographicHash hash(QCryptographicHash::Sha1);
            hash.addData(mf.read(1024 * 1024));
            m[QStringLiteral("sha1First1MB")] = QString::fromLatin1(hash.result().toHex());
        }
        h[QStringLiteral("model")] = m;
    }

    // 运行环境。换机器/换构建之后结论还成不成立，靠这个判断。
    {
        QJsonObject e;
        e[QStringLiteral("os")] = QSysInfo::prettyProductName();
        e[QStringLiteral("kernel")] = QSysInfo::kernelVersion();
        e[QStringLiteral("arch")] = QSysInfo::currentCpuArchitecture();
        e[QStringLiteral("qt")] = QStringLiteral(QT_VERSION_STR);
        e[QStringLiteral("buildDate")] = QStringLiteral(__DATE__ " " __TIME__);
        e[QStringLiteral("appDir")] = QCoreApplication::applicationDirPath();
        h[QStringLiteral("env")] = e;
    }

    // 【这一段是给分析者看的】说明这份文件里有什么、该先看哪里。
    // 一份自解释的数据比一份需要问作者才能读的数据有用得多。
    {
        QJsonObject g;
        g[QStringLiteral("chunks")] = QStringLiteral(
            "1=CamBlobs(每相机2D光斑) 2=Points3D 3=Skeleton 4=Mark "
            "5=ParamDelta(含相机内外参) 6=Cluster3D(含supportMask/residualPx) "
            "7=Trailer 8=ClockSync 9=Hm20Diag(逐帧hm20诊断)");
        h[QStringLiteral("readme")] = g;
    }

    return QString::fromUtf8(QJsonDocument(h).toJson(QJsonDocument::Compact)).toStdString();
}

} // namespace mocap
