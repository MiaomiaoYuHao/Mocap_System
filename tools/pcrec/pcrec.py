#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pcrec.py —— .pcrec v3~v8 读取库

v8 新增一块，给一块接了尾巴，并把一直没接线的 23 真正接上：
    33 RomMapDbg  ROM 映射器逐维逐帧走了哪条分支 + 目标行程 + 冻结计数
    31 RomCalib   尾部接上 rom::Config 全量快照（2040 -> 2112 字节）
    23 UdpOut     v5 就定义了结构体，但 C++ 侧从来没调过 writeUdpOut

【v8 修的三个"在文件里查不出来"】
  ① 「ROM 标定完输出就冻住不动了」
     Mapper::apply() 里通向"输出是一个不动的数"的路一共四条，输出完全一样：
       死维->中立位 / 保持上帧 / 区间塌->中立位 / 正常映射但 t 恒被钳到端点
     修法毫无共同点。块 33 逐维逐帧记走的是哪条 + 连续冻结帧数。
     入口：r.why_frozen()
  ② 「标定之前就有几维一直触限」
     未标定时走 clamp(q, tgtLo, tgtHi)，而 tgtLo/tgtHi 在 v7 里【一个字节
     都没有】—— 它可以被 setTargetRange() 改写。块 33 逐帧记。
     入口：r.why_clamped()
  ③ 「行程总是不够」
     v7 记了结果没记判据。同一批样本换一组阈值就是另一个结论，而两份记录
     看起来完全一样。块 31 尾部接了配置快照。入口：r.why_rom_short()

【JOINTSOLVE 从 504 变成 632】C++ 侧 v8 给它加了 PIP 的四元数解算路径
（qRelPip/pipFromQuat/pipFromAxis/pipUsedQuat）。旧脚本读新文件会拿
recBytes 一比对不上、报 schema 错误并【跳过整块】—— 也就是"关节角解算
内部量"这一块整个读不出来。本版两种布局都认得，旧的自动升级并打标记。

v7 新增两块，专门为了查 ROM 标定：

    31 RomCalib    ROM 逐维全量状态（样本/拒收原因/方向/外推量/状态码）
    32 JointSolve  关节角解算逐帧内部量（两条骨轴/dot/铰链符号/curl/逐维状态）
并加了事件码 16~19（ROM 按钮的两个瞬间 + 采样真正起止的那两帧）。


v4 相对 v3 新增五个块，并把一直没接线的 10/11 真正接上：
    13 JointOut    关节角输出全链路（解算/平滑/ROM/限幅）+ 两套四元数
    14 Handedness  手性判定的全部证据（六处环节各自的手性 + 判据原始值）
    15 MarkerDbg   逐 marker 来源与位置（网络预测 vs 最终，搬了多远）
    16 RunFlags    逐帧生效开关
    17 StateJson   1Hz 全量运行时状态 JSON

【dtype 必须和 C++ 结构体逐字节对上】
这不是洁癖：错位的 dtype 不会报错，只会读出一堆【看起来完全合理】的数，
然后把分析引向错误结论——比读不出来危险得多。所以新块的载荷前 4 字节是
{recVersion:u2, recBytes:u2}，解析时拿 recBytes 跟 dtype.itemsize 对账，
不等就记下错误并跳过这一块，绝不猜。

下面每个 dtype 的字段顺序【严格照抄】C++ 声明顺序，align=True 对应 C++
的自然对齐。改任何一边都必须改另一边，C++ 那侧有 static_assert 钉着尺寸。
"""
import json
import struct
import sys
from dataclasses import dataclass, field

import numpy as np

MAGIC = b"MCPCREC\0"
CT_CAM_BLOBS, CT_POINTS3D, CT_SKELETON, CT_MARK, CT_PARAM_DELTA = 1, 2, 3, 4, 5
CT_CLUSTER3D, CT_TRAILER, CT_CLOCKSYNC, CT_HM20DIAG = 6, 7, 8, 9
CT_ASSOC_INPUT, CT_NETRAW, CT_CAMSETTINGS = 10, 11, 12
CT_JOINTOUT, CT_HANDEDNESS, CT_MARKERDBG, CT_RUNFLAGS, CT_STATEJSON = 13, 14, 15, 16, 17
# ---- v5 ----
CT_MARKERSTAGE, CT_FILTERDBG, CT_IKDBG, CT_TEMPLATESNAP = 18, 19, 20, 21
CT_TIMING, CT_UDPOUT, CT_EVENT, CT_AUTOCALIBDBG = 22, 23, 24, 25
CT_ASSIGNFULL, CT_TRACKDBG, CT_CLUSTERLINK, CT_NETINPUT = 26, 27, 28, 29
# ---- v6 / v7 ----
CT_CHAINDBG = 30
CT_ROMCALIB, CT_JOINTSOLVE = 31, 32
# ---- v8 ----
CT_ROMMAPDBG, CT_ANGLECHAIN, CT_CHUNKSTATS = 33, 34, 35

BLOB2D = np.dtype([("x", "<f4"), ("y", "<f4"), ("cxx", "<f4"), ("cxy", "<f4"),
                   ("cyy", "<f4"), ("area", "<f4"), ("peak", "<f4"),
                   ("bw", "<u2"), ("bh", "<u2")], align=True)
POINT3D = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("id", "<i4"),
                    ("coasting", "u1"), ("predicted", "u1"), ("usedViews", "u1"),
                    ("_p", "u1"), ("residualMm", "<f4")], align=True)
CLUSTER = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("residualPx", "<f4"),
                    ("supportMask", "<u4"), ("nSupport", "u1"), ("verified", "u1"),
                    ("_p", "u1", 2)], align=True)
SKEL = np.dtype([("valid", "u1"), ("wristPoseValid", "u1"), ("pentagonOk", "u1"),
                 ("numGhost", "u1"), ("dorsumRmseMm", "<f4"),
                 ("pos", "<f4", 60), ("observed", "u1", 20), ("conf", "<f4", 20),
                 ("segSource", "u1", 16), ("segQuat", "<f4", 64),
                 ("wristR", "<f4", 9), ("wristT", "<f4", 3)], align=True)
NETRAW = np.dtype([("tsNs", "<i8"), ("nCand", "<u2"), ("hasPose", "u1"), ("hasSegR", "u1"),
                   ("pos", "<f4", 60), ("center", "<f4", 3), ("scale", "<f4"),
                   ("missLogit", "<f4", 20), ("jointAng", "<f4", 20), ("handLogit", "<f4"),
                   ("poseConf", "<f4", 5), ("segRot6d", "<f4", 96)], align=True)
NETTOPK = np.dtype([("label", "<i2", 3), ("prob", "<f4", 3)], align=True)
ASSOCIN = np.dtype([("tsNs", "<i8"), ("n", "<u2"), ("nBeforeCap", "<u2"),
                    ("_p", "u1", 4)], align=True)
CAMSET = np.dtype([("camId", "<u4"), ("width", "<i4"), ("height", "<i4"),
                   ("exposureUs", "<f4"), ("gain", "<f4"), ("fps", "<f4"),
                   ("threshold", "<i4"), ("minArea", "<i4"), ("maxArea", "<i4"),
                   ("roiEnabled", "u1"), ("_p", "u1", 3), ("roiX", "<i4"),
                   ("roiY", "<i4"), ("roiW", "<i4"), ("roiH", "<i4")], align=True)

# ---- v4 -------------------------------------------------------------------
_HDR = [("recVersion", "<u2"), ("recBytes", "<u2"), ("_pad0", "<i4")]

JOINTOUT = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("qSolve", "<f4", 16), ("qSmooth", "<f4", 16),
    ("qRom", "<f4", 16), ("qOut", "<f4", 16),
    ("romLo", "<f4", 16), ("romHi", "<f4", 16),
    ("segQuatWorld", "<f4", 64), ("segQuatLocal", "<f4", 64),
    ("wristQuat", "<f4", 4), ("wristT", "<f4", 3),
    ("romCoverage", "<f4"), ("romAbdCoverage", "<f4"), ("dtSec", "<f4"),
    ("rateLimitRadPerSec", "<f4"), ("angSmoothAlpha", "<f4"),
    ("maxRateClipRad", "<f4"), ("maxStageDeltaRad", "<f4"),
    ("romSamples", "<i4"), ("maxRateClipIdx", "<i4"), ("maxStageDeltaIdx", "<i4"),
    ("segSource", "u1", 16), ("fingerValid", "u1", 5), ("fingerPredicted", "u1", 5),
    ("romReady", "u1"), ("romLearning", "u1"), ("rateLimitOn", "u1"),
    ("angSmoothOn", "u1"), ("mcpValid", "u1"), ("wristValid", "u1"),
    ("jointMirror", "u1"), ("wristStale", "u1"), ("quatOutOn", "u1"),
    ("filterActive", "u1"), ("lowLatency", "u1"), ("_pad1", "u1", 5),
], align=True)

HANDEDNESS = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("handSignMm", "<f4"), ("handSignLatMm", "<f4"), ("handednessMinMm", "<f4"),
    ("aiHandLogit", "<f4"), ("aiHandConf", "<f4"), ("thumbRollSignedRad", "<f4"),
    ("handSignN", "<i4"), ("geoHandSign", "<i4"),
    ("panelIsRight", "u1"), ("autoIsRightDetected", "u1"), ("autoIsRight", "u1"),
    ("aiIsRight", "u1"), ("tmplIsRight", "u1"), ("ikIsRight", "u1"),
    ("autoKnown", "u1"), ("autoAgree", "u1"), ("autoConflict", "u1"),
    ("autoMirrorSuspect", "u1"), ("aiHas", "u1"), ("aiKnown", "u1"), ("aiLocked", "u1"),
    ("cfgDetectHandedness", "u1"), ("cfgApplyHandedness", "u1"),
    ("cfgJointMirrorAuto", "u1"), ("jointMirrorActive", "u1"),
    ("autoCalibOn", "u1"), ("autoStage", "u1"), ("_pad1", "u1", 3),
], align=True)

MARKERDBG = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("posFinal", "<f4", 60), ("posNet", "<f4", 60),
    ("conf", "<f4", 20), ("missLogit", "<f4", 20), ("netDeltaMm", "<f4", 20),
    ("sourcePointId", "<i4", 20),
    ("source", "u1", 20), ("flags", "u1", 20), ("observed", "u1", 20),
    ("hasNet", "u1"), ("_pad1", "u1", 3),
], align=True)

RUNFLAGS = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("bits", "<u4"), ("_pad1", "<i4"),
    ("thumbRollUiDeg", "<f4"), ("thumbPronation0Rad", "<f4"),
    ("occludedJumpGateMm", "<f4"), ("latencyMs", "<f4"),
], align=True)

# ---- v5 -------------------------------------------------------------------
# 【六级流水】0=网络 1=指派 2=几何 3=IK 4=后处理 5=滤波后。
# 名字在这里定义一次，报告和诊断脚本都从这里取 —— 两处各写一份迟早会错开，
# 而错开之后画出来的图看起来完全正常，只是每一级都标成了它的邻居。
STAGE_NAMES = ["网络", "指派", "几何", "IK", "后处理", "滤波后"]

MARKERSTAGE = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("pos", "<f4", (6, 60)), ("stageMoveMm", "<f4", (6, 20)),
    ("stageValid", "u1", 6), ("_pad1", "u1", 2),
], align=True)

FILTERDBG = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("posIn", "<f4", 60), ("posOut", "<f4", 60),
    ("moveMm", "<f4", 20), ("velLocalMm", "<f4", 20), ("staleFrames", "<f4", 20),
    ("fuseWeight", "<f4", 20), ("cutoffHz", "<f4", 20),
    ("hardReset", "u1", 20), ("deadzone", "u1", 20), ("fused", "u1", 20),
    ("observed", "u1", 20),
    ("wristQuatIn", "<f4", 4), ("wristQuatOut", "<f4", 4),
    ("wristAngMoveDeg", "<f4"), ("segQuatMoveDeg", "<f4", 16),
    ("dtSec", "<f4"), ("posMinCutoffHz", "<f4"), ("posBeta", "<f4"),
    ("predictedSmooth", "<f4"),
    ("enabled", "u1"), ("filterPositions", "u1"), ("filterRotations", "u1"),
    ("predictFuseOn", "u1"), ("_pad1", "u1", 4),
], align=True)

# 【这里曾经错位过，留个记号】C++ 的 IkDbgRec 后来加了 MCP-PIP 耦合那五个
# 数组（mcpPipCoupling / ...B / ...Samples / mcpCouplingReady），尺寸从 608
# 变成 680，而这边没跟上。后果不是报错，是 _take() 的对账发现 recBytes 不等
# 就【整块跳过】—— 也就是说 IK 内部那一路在分析里一直是空的，而 summary 里
# 显示的 "IK内部 0" 看起来就像"这次没录到"。
# 这正是文件顶部反复强调"两边必须逐字节对上"的那个坑，它真的踩了一次。
IKDBG = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("q", "<f4", (5, 4)), ("qPrior", "<f4", (5, 4)),
    ("limitLo", "<f4", (5, 4)), ("limitHi", "<f4", (5, 4)),
    ("cost", "<f4", 5), ("rmseMm", "<f4", 5),
    ("anchorMm", "<f4", (5, 3)), ("boneLenMm", "<f4", (5, 3)),
    ("handLenMm", "<f4"), ("thumbAxialK", "<f4"), ("thumbPronation0", "<f4"),
    ("wPrior", "<f4"), ("wLimit", "<f4"), ("wCouple", "<f4"), ("dipCoupling", "<f4"),
    ("mcpPipCoupling", "<f4", 5), ("mcpPipCouplingB", "<f4", 5),
    ("mcpPipCouplingSamples", "<i4", 5),
    ("mcpCouplingReady", "u1", 5), ("_pad2", "u1", 3),
    ("iters", "<i4", 5), ("nObs", "<i4", 5),
    ("limitHit", "u1", (5, 4)), ("fingerValid", "u1", 5), ("fingerSolved", "u1", 5),
    ("ikIsRight", "u1"), ("paramsReady", "u1"), ("applied", "u1"), ("_pad1", "u1"),
], align=True)

TEMPLATESNAP = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("backMarkersMm", "<f4", (5, 3)), ("markersMm", "<f4", (20, 3)),
    ("normalized", "<f4", (20, 3)), ("anchorsMm", "<f4", (5, 3)),
    ("boneLenMm", "<f4", (5, 3)), ("dipCoupling", "<f4", 5),
    ("fingerprint", "<u4"), ("reason", "<i4"),
    ("bundleRmseMm", "<f4"), ("selfAmbiguityMm", "<f4"),
    ("isRight", "u1"), ("valid", "u1"), ("anchorsValid", "u1"),
    ("backCalibrated", "u1"), ("fingerCalibrated", "u1", 5),
    ("fromAutoCalib", "u1"), ("_pad1", "u1", 2),
], align=True)

TIMING = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("camTsSpreadUs", "<f4"), ("camTsStdUs", "<f4"), ("frameDtMs", "<f4"),
    ("frameDtJitterMs", "<f4"), ("wallMinusCamMs", "<f4"),
    ("tDetectMs", "<f4"), ("tClusterMs", "<f4"), ("tTrackMs", "<f4"),
    ("tInferMs", "<f4"), ("tAssocMs", "<f4"), ("tIkMs", "<f4"),
    ("tFilterMs", "<f4"), ("tJointMs", "<f4"), ("tTotalMs", "<f4"),
    ("skelSkipped", "<i4"), ("skelProcessed", "<i4"), ("recQueueDepth", "<i4"),
    ("recDropped", "<i4"), ("camFrameGaps", "<i4"), ("nCamsThisFrame", "<i4"),
    ("fpsIn", "<f4"), ("fpsOut", "<f4"),
], align=True)

UDPOUT = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("m3dsCrc", "<u4"), ("m3dqCrc", "<u4"),
    ("m3dsBytes", "<i4"), ("m3dqBytes", "<i4"),
    ("m3dsSeq", "<i8"), ("m3dqSeq", "<i8"),
    ("m3dsErr", "<i4"), ("m3dqErr", "<i4"),
    ("m3dqFlags", "<u4"), ("targetPort", "<i4"),
    ("enabled", "u1"), ("quatEnabled", "u1"), ("_pad1", "u1", 2),
], align=True)

EVENT = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("code", "<i4"), ("severity", "<i4"),
    ("valueA", "<f4"), ("valueB", "<f4"),
    ("intA", "<i4"), ("intB", "<i4"),
    ("textLen", "<u2"), ("_pad1", "u1", 6),
], align=True)

AUTOCALIBDBG = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("handSignMm", "<f4"), ("handSignLatMm", "<f4"), ("handSignThreshMm", "<f4"),
    ("progress", "<f4"), ("bundleRmseMm", "<f4"),
    ("anchorResidMm", "<f4", 5), ("anchorSpreadDeg", "<f4", 5),
    ("anchorRigidStd", "<f4", 5),
    ("boneLenMm", "<f4", (5, 3)), ("boneLenStdMm", "<f4", (5, 3)),
    ("dorsumTmplDriftMm", "<f4"), ("tmplRejectedRmse", "<f4"), ("tmplAppliedRmse", "<f4"),
    ("stage", "<i4"), ("samples", "<i4"), ("anchorSamples", "<i4", 5),
    ("bundleRuns", "<i4"), ("attempts", "<i4"), ("rejectCode", "<i4"),
    ("handSignN", "<i4"),
    ("templateReady", "u1"), ("ikUsable", "u1"), ("handednessKnown", "u1"),
    ("handednessAgree", "u1"), ("isRightDetected", "u1"), ("isRightApplied", "u1"),
    ("mirrorSuspect", "u1"), ("dorsumReordered", "u1"),
    ("anchorFitted", "u1", 5), ("staticSeeded", "u1", 5),
    ("calibRejected", "u1"), ("_pad1", "u1", 5),
], align=True)

# ---- v6 ----
# 【v6.2 长了三组逐指字段，192 -> 240】原来这块全是【全局】量：anchorsValid
# 是一个 bool，说不出"五根里哪几根降级了"。而实测五根 anchor 的质量差得很远
# （|anchor->pp| 24.8~73.7mm、出平面 17.8~59.7°），一个全局 bool 只能一起用
# 或一起不用 —— 那正是要修的东西，也就必须能逐指看到结果。
#
# C++ 侧的 padding 是显式写出来的（_padA / _pad1），这边照抄即可。
# 下面每个字段的偏移都跟 C++ 的 offsetof 逐个对过：
#   anchorsVerifyNOk 180 / anchorOkPerFinger 184 / _padA 189
#   planeSampleN 192 / anchorOutOfPlaneDeg 212 / _pad1 232 / 总长 240
CHAINDBG = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("coupledRad", "<f4", 5), ("pmLenMm", "<f4", 5), ("mdLenMm", "<f4", 5),
    ("weight", "<f4", 5), ("aging", "<f4", 5),
    ("caseBFrames", "<i4", 5),
    ("hasPipHold", "u1", 5), ("hasPlane", "u1", 5), ("caseB", "u1", 5),
    ("useAnchor", "u1", 5), ("hasExc", "u1", 5), ("chainContinued", "u1"),
    ("anchorsSet", "u1"), ("anchorsChecked", "u1"), ("anchorsValid", "u1"),
    ("tmplMmValid", "u1"),
    ("anchorsVerifyNSeen", "<i4"), ("anchorsVerifyNOk", "<i4"),
    # ---- v6.2 逐指 anchor / 弯曲平面 ----
    ("anchorOkPerFinger", "u1", 5),    # 0 = 这根已退回 pp->mp（纯实测量）
    ("_padA", "u1", 3),                # 189 -> 192，给下面的 int32 对齐
    ("planeSampleN", "<i4", 5),        # 弯曲平面累加器吃进了多少帧
    ("anchorOutOfPlaneDeg", "<f4", 5), # anchor->pp 出平面角(度)，-1 = 本帧量不出来
    ("_pad1", "u1", 8),                # 232 -> 240
], align=True)

# 【旧版 192 字节的同一块，专门为了还能读之前录的文件】
# _take 的规矩是"对不上就报错，绝不猜"——这条规矩是对的，但如果只留新 dtype，
# 后果是【所有历史录制的 chaindbg 整块读不出来】。而这次改动的全部意义就是
# 拿新跑的一遍去跟旧的那份基线比，基线读不了就没得比了。
# 所以旧布局单列一份，读到时升级成新布局、新字段填哨兵值（见 _chaindbg_upgrade）。
CHAINDBG_V1 = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("coupledRad", "<f4", 5), ("pmLenMm", "<f4", 5), ("mdLenMm", "<f4", 5),
    ("weight", "<f4", 5), ("aging", "<f4", 5),
    ("caseBFrames", "<i4", 5),
    ("hasPipHold", "u1", 5), ("hasPlane", "u1", 5), ("caseB", "u1", 5),
    ("useAnchor", "u1", 5), ("hasExc", "u1", 5), ("chainContinued", "u1"),
    ("anchorsSet", "u1"), ("anchorsChecked", "u1"), ("anchorsValid", "u1"),
    ("tmplMmValid", "u1"),
    ("anchorsVerifyNSeen", "<i4"), ("anchorsVerifyNOk", "<i4"),
    ("_pad1", "u1", 2),
], align=True)

# ---- v7：ROM 标定这一路 ----------------------------------------------------
# 【读这两块的顺序】先看 JOINTSOLVE 的 dotProxMid 和 proxAxisSrc，
# 再看 ROMCALIB 的 nReject / sign / signDeltaRad。
# 前者回答"角度是怎么算出来的"，后者回答"标定为什么标成这样"，
# 而绝大多数"标不满 / 方向反了"的案子，答案都在前者里。
# 【v7 的布局保留着】旧录制文件还得读。两个都试，都对不上才报 schema 错误 ——
# "绝不猜"这条规矩没变，变的只是认得的版本从一个变成两个。
_ROMCALIB_BODY = _HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("lo", "<f4", 16), ("hi", "<f4", 16),
    ("rawLo", "<f4", 16), ("rawHi", "<f4", 16),
    ("measLo", "<f4", 16), ("measHi", "<f4", 16),
    ("coverage", "<f4", 16),
    ("signDeltaRad", "<f4", 16), ("signCorr", "<f4", 16),
    ("openMedRad", "<f4", 16), ("closeMedRad", "<f4", 16),
    ("curlSpread", "<f4", 16),
    ("fitSlope", "<f4", 16), ("fitR2", "<f4", 16),
    ("extrapLoRad", "<f4", 16), ("extrapHiRad", "<f4", 16),
    ("curlSeenLo", "<f4", 16), ("curlSeenHi", "<f4", 16),
    ("curlUsedLo", "<f4", 16), ("curlUsedHi", "<f4", 16),
    ("nSamples", "<i4", 16), ("nSeen", "<i4", 16),
    ("nOpen", "<i4", 16), ("nClose", "<i4", 16),
    ("nReject", "<i4", (16, 6)),
    ("sign", "i1", 16), ("signResolved", "u1", 16), ("status", "u1", 16),
    ("coverageFlex", "<f4"), ("coverageAbd", "<f4"), ("durationSec", "<f4"),
    ("nOkFlex", "<i4"), ("nPriorFlex", "<i4"), ("nDeadFlex", "<i4"),
    ("nSignFlipped", "<i4"), ("nFrames", "<i4"), ("nFramesUsed", "<i4"),
    ("reason", "<i4"),
    ("ready", "u1"), ("accumulated", "u1"), ("_pad1", "u1", 2),
]
ROMCALIB_V7 = np.dtype(_ROMCALIB_BODY, align=True)

# v8：尾部接上 rom::Config 全量快照。
# 【为什么结果旁边必须放判据】同一批样本，minRangeRad 从 0.35 调到 0.25，
# 一半的维就从 PriorFilled 变成 Ok，而 coverageFlex 一个字都没变 ——
# 也就是说【两份看起来完全一样的记录可以是两个不同的结论】。
# 现场最常见的一句"我们上次好像调过那个阈值"，没有这段快照既证实不了
# 也证伪不了，而它恰恰决定了后面所有分析的前提。
# 【为什么不用 ROMCALIB_V7.descr + [...] 拼】descr 里带着 v7 的【尾部填充】，
# 拼接时那几个字节会被保留下来，而 C++ 追加字段时【复用】的正是这段尾部填充：
#   C++  _pad1 在 2034..2035，cfgMinRangeRad 落在 2036
#   拼接 descr 会把 cfgMinRangeRad 放到 2040
# 两边 itemsize 都是 2112，尺寸闸【查不出来】，读出来的却是整体错位一格的数 ——
# 而错位的数看起来完全合理（0.55 出现在 outlierAbsRad 上，谁都不会起疑）。
# 这正是本文件开头说的"比读不出来危险得多"的那一类。
# 所以：新旧两个 dtype 都从同一份字段列表用 align=True 重建，
# 让 numpy 自己按 C 的自然对齐排，跟编译器算的是同一套规则。
_ROMCALIB_CFG = [
    ("cfgMinRangeRad", "<f4"), ("cfgMinCurlSpread", "<f4"),
    ("cfgMinSignDeltaRad", "<f4"),
    ("cfgOpenFrac", "<f4"), ("cfgCloseFrac", "<f4"), ("cfgEndQuantile", "<f4"),
    ("cfgGlobalQLo", "<f4"), ("cfgGlobalQHi", "<f4"), ("cfgOutlierAbsRad", "<f4"),
    ("cfgExtrapMinR2", "<f4"), ("cfgExtrapMinCurlSpan", "<f4"),
    ("cfgExtrapMaxFrac", "<f4"), ("cfgPriorSpanFrac", "<f4"),
    ("cfgMinSamplesPerDof", "<i4"), ("cfgMinOkFlexDofs", "<i4"),
    ("cfgMaxSamplesPerDof", "<i4"),
    ("cfgLearnSign", "u1"), ("cfgAcceptPredicted", "u1"),
    ("cfgAcceptDegenerate", "u1"), ("cfgExtrapolateByCurl", "u1"),
    ("cfgFillFromPrior", "u1"), ("cfgHoldOnDegenerate", "u1"),
    ("_pad2", "u1", 2),
]
ROMCALIB = np.dtype(_ROMCALIB_BODY + _ROMCALIB_CFG, align=True)

# 【v7 的 504 字节布局保留着】v8 给 PIP 加了四元数解算路径。
# 【为什么这件事很要紧】C++ 侧改了而脚本没跟上时，_take 会拿 recBytes
# 一比对不上，然后【跳过整块】—— 症状是"关节角解算内部量一条都没有"，
# 而文件里其实是有的。这一类"读不出来"最容易被误当成"没录"。
_JOINTSOLVE_BODY = _HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("axProx", "<f4", (5, 3)), ("axMid", "<f4", (5, 3)),
    ("axDist", "<f4", (5, 3)), ("axProxWorld", "<f4", (5, 3)),
    ("dotProxMid", "<f4", 5), ("hingeSigned", "<f4", 5),
    ("flexRaw", "<f4", 5), ("abdRaw", "<f4", 5),
    ("pipRaw", "<f4", 5), ("ipRaw", "<f4", 5),
    ("curlChain", "<f4", 5), ("curl", "<f4", 5), ("boneLenMm", "<f4", 5),
    ("curlValid", "u1", 5), ("curlPredicted", "u1", 5), ("proxAxisSrc", "u1", 5),
    ("dofState", "u1", 16), ("romReject", "u1", 16),
    ("wristPoseValid", "u1"), ("anchorsValid", "u1"),
    ("mirrored", "u1"), ("romLearning", "u1"),
    ("wristStale", "<i4"),
]
JOINTSOLVE_V7 = np.dtype(_JOINTSOLVE_BODY, align=True)

# v8：PIP 的四元数解算路径。
# 【为什么非加不可】实测 anchorsValid 恒为假时叉乘路径整个失效
# （a0==a1 -> pipFromAxis = acos(1) = 0），真正在用的是相对四元数那条路，
# 而那条路的中间量在 v7 里【一个都没录】—— 于是"PIP 为什么是这个值"
# 在文件里查不到。pipUsedQuat 说明本帧走的哪条，两条的值都留着做对照。
# 同样从字段列表重建，理由见 ROMCALIB 上方那段。
JOINTSOLVE = np.dtype(_JOINTSOLVE_BODY + [
    ("qRelPip", "<f4", (5, 4)),
    ("pipFromQuat", "<f4", 5),
    ("pipFromAxis", "<f4", 5),
    ("pipUsedQuat", "u1", 5),
    ("_pad2", "u1", 3),
], align=True)

# ---- 块 33：ROM 映射器逐维逐帧分支（v8）------------------------------------
#
# 【这一块只回答一个问题：这一维为什么不动】
# 通向"输出是一个不动的数"的路一共四条，产生的输出【完全无法区分】，
# 而成因和修法毫无共同点：
#   branch=1 死维      成因在标定阶段（样本太少/几乎全被 AxisDegen 拒）
#                      -> 回去看块 31 的 nReject，是上游几何问题，重标没用
#   branch=2 保持上帧  成因在本帧（dofState 不新鲜）
#                      -> 看块 32 的 dofState / proxAxisSrc
#                      【这一条最像"死机"】：标定结果、覆盖度、romReady 全正常
#   branch=3 区间塌了  hi-lo≈0 -> 重标这一维
#   branch=4 正常映射但 t 恒被钳到 0 或 1
#                      不是 bug，是标定时没做到这个姿态 -> 补标
# branch=0 是未标定时的透传钳位，"标定之前就一直触限"的全部解释就在
# tgtLo/tgtHi + clampLo/clampHi 这四个量里。
ROMMAPDBG = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("vSigned", "<f4", 16),     # sign*q
    ("uNorm", "<f4", 16),       # (v-lo)/span，【钳位之前】。<0 或 >1 = 姿态在区间外
    ("tgtLo", "<f4", 16), ("tgtHi", "<f4", 16),
    ("heldRun", "<i4", 16),     # 连续走"保持"分支的帧数
    ("frozenRun", "<i4", 16),   # 输出连续没变的帧数
    ("branch", "u1", 16), ("custom", "u1", 16),
    ("clampLo", "u1", 16), ("clampHi", "u1", 16),
    ("status", "u1", 16), ("dofState", "u1", 16),
    ("sign", "i1", 16),
    ("nHeld", "<i4"), ("nFrozen", "<i4"),
    ("maxFrozenRun", "<i4"), ("maxFrozenIdx", "<i4"),
    ("nClamped", "<i4"), ("nNeutral", "<i4"),
    ("dtSec", "<f4"),
    ("romReady", "u1"), ("holdOnDegenerate", "u1"), ("hasLast", "u1"),
    ("mapperJustReset", "u1"), ("rateLimitOn", "u1"), ("romLearning", "u1"),
    ("_pad1", "u1", 2),
], align=True)

MAP_BRANCH_NAME = {
    0: "未标定-透传钳位", 1: "死维-钉中立位", 2: "保持上一帧",
    3: "区间塌-钉中立位", 4: "正常映射",
}

# ---- 块 34：角度后处理链的隐藏状态（v8）------------------------------------
#
# 块 13 记了四级的【值】，块 33 记了中间那一级的【内部】，这一块补剩下两级。
#
# 【smOffset 是这一块里最该先看的】它是【直接加到输出上的逐维常量偏置】，
# 最大到 smMaxOffsetRad(0.7rad ≈ 40°)，而它在系统里没有第二份 ——
# 于是这两件事在文件里长得完全一样：
#     ① 解算本来就偏了 40°
#     ② 解算是对的，被一个没还清的恢复补偿顶着
# ② 的成因是"预测段跑飞过一次"，修法在遮挡处理上，跟 ① 毫无关系。
#
# smOffClamped=1 = off 被钳住 = 预测段至少跑飞了 40°，一条很强的结论；
# 钳完之后 off 看起来就是个正常的 0.7，痕迹没了，所以必须单独记。
ANGLECHAIN = np.dtype(_HDR + [
    ("wallNs", "<i8"), ("frameTsNs", "<i8"),
    ("smState", "<f4", 16),      # s[]：预测段实际输出的那个值
    ("smOffset", "<f4", 16),     # off[]：加在输出上的常量偏置
    ("smHas", "u1", 16), ("smOffClamped", "u1", 16),
    ("smWasPred", "u1", 5), ("smPredNow", "u1", 5), ("_pad1", "u1", 6),
    ("smAlpha", "<f4"), ("smDecay", "<f4"), ("smMaxOffsetRad", "<f4"),
    ("maxSmOffsetRad", "<f4"), ("maxSmOffsetIdx", "<i4"), ("nSmOffActive", "<i4"),
    ("rlClip", "<f4", 16),       # 逐维削掉量。全 0 且 rlOn=1 = 没削过
    ("rlStepLimitRad", "<f4"), ("rlMaxRadPerSec", "<f4"),
    ("maxRlClipRad", "<f4"), ("maxRlClipIdx", "<i4"), ("nRlClipped", "<i4"),
    ("dtSec", "<f4"),
    ("smOn", "u1"), ("rlOn", "u1"), ("rlHadState", "u1"), ("lowLatency", "u1"),
    ("_pad2", "u1", 4),
], align=True)

# ---- 块 35：逐块类型的写入/丢弃统计（v8）------------------------------------
#
# 【这一块回答"这一块为什么不在文件里"】以前只有 Trailer 的三个总数，
# 于是同一个现象有三种解释，而它们无法区分：
#   ① 压根没被调用（代码路径没走到）② 被 Detail 档位关掉 ③ 队列满被丢
# 三者的下一步动作毫无共同点，猜错一次就是白查一整轮。
# 配合文件头的 manifest：
#   清单里没有            -> 这个 build 版本老，不认识这个块
#   enabled=false        -> ②，重录时调档
#   enabled=true, pushed=0 -> ①，真 bug，去查为什么没触发
#   dropped>0            -> ③，换盘或降档
CHUNKSTATS = np.dtype(_HDR + [
    ("wallNs", "<i8"),
    ("detail", "<i4"), ("nTypes", "<i4"),
    ("pushed", "<u8", 64), ("dropped", "<u8", 64), ("bytes", "<u8", 64),
    ("queuePeak", "<u8"),      # 队列深度峰值。【比"丢了多少"更早预警】——
                               # 长期过半说明盘跟不上，等丢块出现时已经有洞了
    ("droppedTotal", "<u8"), ("pushedTotal", "<u8"), ("bytesTotal", "<u8"),
], align=True)

CHUNK_NAME = {
    1: "CamBlobs", 2: "Points3D", 3: "Skeleton", 4: "Mark", 5: "ParamDelta",
    6: "Cluster3D", 7: "Trailer", 8: "ClockSync", 9: "Hm20Diag",
    10: "AssocInput", 11: "NetRaw", 12: "CamSettings",
    13: "JointOut", 14: "Handedness", 15: "MarkerDbg", 16: "RunFlags",
    17: "StateJson", 18: "MarkerStage", 19: "FilterDbg", 20: "IkDbg",
    21: "TemplateSnap", 22: "Timing", 23: "UdpOut", 24: "Event",
    25: "AutoCalibDbg", 26: "AssignFull", 27: "TrackDbg", 28: "ClusterLink",
    29: "NetInput", 30: "ChainDbg", 31: "RomCalib", 32: "JointSolve",
    33: "RomMapDbg", 34: "AngleChain", 35: "ChunkStats",
}
DROP_TIER_NAME = {0: "关键-最后才丢", 1: "诊断", 2: "大块-先让路"}

# UdpOutRec 的错误码。【负数是"根本没发"，正数是"发了但网络栈拒了"】
# v8 之前 UdpSender 里这两条负数路径是【静默 return】，一点痕迹都没有 ——
# 而下游"收不到数据"时，它们跟"发了但对端没配"完全分不开，
# 三者的下一步动作毫无共同点。
UDP_ERR_NAME = {
    0: "已发出",
    -1: "打包失败(形状不对，整帧不发)",
    -2: "UDP 推送开关是关的",
}


def udp_err(code):
    code = int(code)
    if code in UDP_ERR_NAME:
        return UDP_ERR_NAME[code]
    return f"socket 错误 {code - 1}"

# 逐维状态 / 拒收原因 / 标定状态的人话表。【跟 C++ 那边是同一组编号】
DOF_STATE_NAME = {0: "没算", 1: "保持上帧", 2: "骨轴退化", 3: "预测段", 4: "实测"}
ROM_REJECT_NAME = {0: "接受", 1: "不新鲜", 2: "骨轴退化", 3: "curl无效",
                   4: "野点", 5: "只有预测值"}
ROM_STATUS_NAME = {0: "正常", 1: "行程不足-已兜底", 2: "方向不明",
                   3: "样本不足", 4: "骨轴退化(上游)", 5: "含外推"}
PROX_AXIS_NAME = {0: "anchor->pp(正确)", 1: "pp->mp(退化)", 2: "无"}
ROM_REASON_NAME = {0: "标定开始", 1: "标定中(1Hz)", 2: "标定结束", 3: "自文件加载"}

TRACKDBG_HDR = np.dtype([("tsNs", "<i8"), ("n", "<u2"), ("_p0", "<u2"),
                         ("_p1", "<u4")], align=True)
TRACKDBG_ITEM = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("vx", "<f4"), ("vy", "<f4"), ("vz", "<f4"),
    ("posVarTrace", "<f4"), ("velVarTrace", "<f4"),
    ("assocDistMm", "<f4"), ("assocMahaSq", "<f4"), ("qBoost", "<f4"),
    ("residualMm", "<f4"),
    ("id", "<i4"), ("obsIndex", "<i4"),
    ("missedFrames", "<i2"), ("hits", "<i2"),
    ("justAcquired", "u1"), ("coasting", "u1"), ("confirmed", "u1"),
    ("gateBypassed", "u1"),
], align=True)

CLUSTERLINK_HDR = np.dtype([("tsNs", "<i8"), ("nTracks", "<u2"), ("nUnmatched", "<u2"),
                            ("nCams", "u1"), ("_p", "u1", 3)], align=True)

# 事件码 -> 人话。【跟 C++ 那边的注释是同一张表】
MARK_KIND = {0: "打标记(手动)", 1: "打标记(自动)", 2: "打标记(ROM)"}

EVENT_NAME = {
    1: "手性变更", 2: "模板热替换", 3: "模板被拒", 4: "自标定阶段变化",
    5: "看门狗松锁", 6: "看门狗退模板", 7: "模型切换", 8: "参数下发",
    9: "IK开关", 10: "滤波开关", 11: "ROM标定完成", 12: "跟踪重置",
    13: "后端初始化", 14: "用户操作", 15: "其它",
    16: "按下ROM标定", 17: "按下结束ROM",
    18: "ROM采样开始(帧)", 19: "ROM采样结束(帧)",
    # ---- v8.1 ----
    20: "面板操作", 21: "开始录制", 22: "停止录制",
}
TEMPLATE_REASON = {0: "录制开始", 1: "模板热替换", 2: "手性变更",
                   3: "自标定提交", 4: "周期重录"}

# 【尺寸闸】跟 C++ 那边的 static_assert 是同一组数。任何一边改了字段而另一边
# 没跟上，导入时立刻炸，而不是让分析读出错位的垃圾。
_EXPECT = {"JOINTOUT": 1032, "HANDEDNESS": 80, "MARKERDBG": 888, "RUNFLAGS": 48,
           "IKDBG": 680,
           "SKEL": 668, "NETRAW": 840, "NETTOPK": 20,
           "MARKERSTAGE": 1952, "FILTERDBG": 1112,
           "TEMPLATESNAP": 736, "TIMING": 112, "UDPOUT": 80, "EVENT": 56,
           "AUTOCALIBDBG": 304, "TRACKDBG_HDR": 16, "TRACKDBG_ITEM": 64,
           "CLUSTERLINK_HDR": 16,
           "CHAINDBG": 240, "CHAINDBG_V1": 192,
           # 【新旧两套都钉住】旧布局不是"历史包袱"，它是【还在读的文件的格式】。
           # 只钉新版的话，哪天有人顺手改了 _V7 的字段，旧文件会静默错位 ——
           # 而旧文件恰恰是没法重录的那一批。
           "ROMCALIB_V7": 2040, "ROMCALIB": 2112,
           "JOINTSOLVE_V7": 504, "JOINTSOLVE": 632,
           "ROMMAPDBG": 560, "ANGLECHAIN": 320, "CHUNKSTATS": 1592}
for _n, _sz in _EXPECT.items():
    _dt = globals()[_n]
    if _dt.itemsize != _sz:
        raise RuntimeError(
            f"{_n} dtype 尺寸 {_dt.itemsize} != C++ 的 {_sz}。"
            f"改了 PointCloudRecorder.hpp 就必须同步改这里，否则读出来的是错位的数。")

# 【偏移闸】—— 光钉总尺寸【不够】。
# 实测踩过一次：ROMCALIB 用 ROMCALIB_V7.descr + [...] 拼出来，
# descr 带着 v7 的尾部填充，而 C++ 追加字段时【复用】的正是那段填充：
#     C++   cfgMinRangeRad 落在 2036
#     拼接  cfgMinRangeRad 落在 2040
# 两边 itemsize 都是 2112，尺寸闸一个字都没报，读出来的却是整体错位一格的数 ——
# 而错位的数【看起来完全合理】（0.55 出现在 outlierAbsRad 上，谁都不会起疑）。
# 这正是文件开头说的"比读不出来危险得多"的那一类。
# 所以每个结构体再钉几个【尾部和数组】字段的偏移：尾部对得上，说明中间
# 每一个字段的对齐都跟编译器算的一致。
_EXPECT_OFF = {
    ("ROMCALIB", "nReject"): 1560,
    ("ROMCALIB", "cfgLearnSign"): 2100,
    ("JOINTSOLVE", "qRelPip"): 500,
    ("JOINTSOLVE", "pipUsedQuat"): 620,
    ("ROMMAPDBG", "branch"): 408,
    ("ROMMAPDBG", "nHeld"): 520,
    ("ANGLECHAIN", "smAlpha"): 200,
    ("ANGLECHAIN", "rlClip"): 224,
    ("CHUNKSTATS", "pushed"): 24,
    ("CHUNKSTATS", "queuePeak"): 1560,
}
for (_n, _f), _o in _EXPECT_OFF.items():
    _got = globals()[_n].fields[_f][1]
    if _got != _o:
        raise RuntimeError(
            f"{_n}.{_f} 偏移 {_got} != C++ 的 {_o}。dtype 的对齐跟 C++ 对不上，"
            f"读出来的每个字段都是错位的 —— 而错位的数看起来完全合理。")

# marker 来源枚举（对应 SkeletonFrameResult::markerSource）
SRC_NAME = {0: "未知", 1: "实测", 2: "IK补", 3: "链式续解", 4: "网络预测", 5: "保持上帧"}
# markerFlags 位
MF_BONESNAP, MF_SMOOTHED, MF_JUMPLIMIT, MF_THUMBFIX = 1, 2, 4, 8

# RunFlagsRec.bits 位定义。【必须和 pcrec::RunFlagBit 一字不差】
RUNFLAG_NAMES = [
    "backendReady", "ikEnabled", "ikOnlyOccluded", "chainContinue", "filterOn",
    "rateLimitOn", "romLearning", "romReady", "quatOutOn", "autoCalibOn",
    "dorsumRigid", "geoRelabel", "netSegCalib", "thumbSegFromNet", "thumbPronationOn",
    "captureDebug", "jointMirrorAuto", "jointMirrorNow", "lowLatency", "fastModel",
    "showSkeleton", "recording",
]

# 16 维关节角的名字，跟 HandModel.hpp 的布局一致
JOINT_NAMES = ["拇CMC屈", "拇CMC展", "拇MCP", "拇IP",
               "食MCP屈", "食MCP展", "食PIP",
               "中MCP屈", "中MCP展", "中PIP",
               "无MCP屈", "无MCP展", "无PIP",
               "小MCP屈", "小MCP展", "小PIP"]
# 屈曲类维度（握拳时应该变大的那些）。外展维不在内 —— 张开/握拳这个动作
# 本来就不采外展，把它算进去只会稀释信号。
FLEX_IDX = [0, 2, 3, 4, 6, 7, 9, 10, 12, 13, 15]
ABD_IDX = [1, 5, 8, 11, 14]
# 指尖 marker 下标（远节球）。markers 布局：0..4 手背，5..7 拇指，8..19 四指
TIP_IDX = [7, 10, 13, 16, 19]


def _diag_dtype():
    return np.dtype([
        ("wallNs", "<i8"), ("frameTsNs", "<i8"),
        ("dorsumReason", "u1"), ("dorsumInliers", "u1"), ("dorsumByHistory", "u1"),
        ("wristPoseValid", "u1"), ("pentagonOk", "u1"), ("numGhost", "u1"),
        ("observedJoints", "u1"), ("candidateCount", "u1"),
        ("sourcePointId", "<i4", 20),
        ("dorsumRmseMm", "<f4"), ("dorsumMarginMm", "<f4"), ("dorsumSelfAmbMm", "<f4"),
        ("thumbFixSkip", "u1"), ("thumbFixed", "u1"), ("netThumbSegs", "u1"),
        ("dorsumRepaired", "u1"), ("thumbTipOccluded", "u1"),
        ("dorsumRepairMoveMm", "<f4"),
        ("thumbIpPredDeg", "<f4"), ("thumbIpDriftDeg", "<f4"),
        ("bundleRuns", "<i4"), ("bundleLastMs", "<f4"),
        ("thumbPronationFitted", "u1"),
        ("thumbPronation0", "<f4"), ("thumbAxialK", "<f4"),
        ("thumbPronationContrast", "<f4"), ("thumbCoverage", "<f4"),
        ("handPanelIsRight", "u1"), ("handAutoIsRight", "u1"),
        ("handAutoDetected", "u1"), ("handAiIsRight", "u1"),
        ("handednessKnown", "u1"), ("handednessConflict", "u1"),
        ("mirrorSuspect", "u1"), ("jointMirrorActive", "u1"), ("handSignMm", "<f4"),
        ("ikActive", "u1"), ("ikFingerCount", "u1"), ("occludedHeld", "u1"),
        ("chainContinued", "u1"), ("ikFilled", "u1"), ("ikFallback", "u1"),
        ("fingerIkValid", "u1", 5), ("fingerObsCount", "u1", 5),
        ("fingerIkRmseMm", "<f4", 5),
        ("segSource", "u1", 16), ("segConf", "<f4", 16),
        ("autoStage", "u1"), ("autoTemplateReady", "u1"), ("autoIkUsable", "u1"),
        ("autoDorsumReordered", "u1"),
        ("autoProgress", "<f4"), ("autoBundleRmseMm", "<f4"),
        ("anchorRigidStd", "<f4", 5),
        ("netSegSolved", "u1"), ("netSegOk", "u1"), ("hasAiPose", "u1"), ("hasAiHand", "u1"),
        ("netSegCSpread", "<f4"), ("netSegKOffset", "<f4", 3),
        ("aiPoseConf", "<f4", 5), ("latencyMs", "<f4"),
    ], align=True)


@dataclass
class Recording:
    path: str = ""
    version: int = 0
    header: dict = field(default_factory=dict)
    blobs: dict = field(default_factory=dict)
    points3d: list = field(default_factory=list)
    cluster: list = field(default_factory=list)
    skeleton: list = field(default_factory=list)
    diag: np.ndarray = None
    jointout: np.ndarray = None
    handed: np.ndarray = None
    markerdbg: np.ndarray = None
    runflags: np.ndarray = None
    statejson: list = field(default_factory=list)
    assoc_input: list = field(default_factory=list)
    netraw: list = field(default_factory=list)
    camsettings: list = field(default_factory=list)
    marks: list = field(default_factory=list)
    param_deltas: list = field(default_factory=list)
    clocksync: list = field(default_factory=list)
    # ---- v5 ----
    stages: np.ndarray = None       # 20 点位置的六级流水
    filterdbg: np.ndarray = None    # 滤波前后 + 滤波器内部
    ikdbg: np.ndarray = None        # IK 内部
    timing: np.ndarray = None       # 时基与耗时
    udpout: np.ndarray = None       # 输出闭环
    autocalib: np.ndarray = None    # 自标定逐帧
    templates: list = field(default_factory=list)  # [(rec, )] 模板快照，变化时才有
    events: list = field(default_factory=list)     # [(rec, text)]
    trackdbg: list = field(default_factory=list)   # [(ts, items)]
    clusterlink: list = field(default_factory=list)# [(ts, nCams, support, unmatched)]
    assignfull: list = field(default_factory=list) # [(ts, (nCand,nClass) 矩阵)]
    netinput: list = field(default_factory=list)   # [(ts, cand, tmpl, prev)]
    # ---- v6 / v7 ----
    chaindbg: np.ndarray = None     # 链式续解逐指
    # True = 这份文件是旧版 192 字节布局，v6.2 那三个逐指字段是升级时填的哨兵值，
    # 不是真读数。见 _chaindbg_upgrade。
    chaindbg_legacy: bool = False
    romcalib: np.ndarray = None     # ROM 标定逐维全量（开始/1Hz/结束各一份）
    jointsolve: np.ndarray = None   # 关节角解算逐帧内部量（两条骨轴/退化/curl）
    # ---- v8 ----
    rommap: np.ndarray = None       # ROM 映射器逐维分支 + 目标行程 + 冻结计数
    anglechain: np.ndarray = None   # 平滑器/限幅器的隐藏状态
    chunkstats: np.ndarray = None   # 逐块类型的 写入/丢弃/字节
    # 新字段是升级时填的哨兵，不是真读数
    romcalib_legacy: bool = False
    jointsolve_legacy: bool = False
    trailer: dict = field(default_factory=dict)
    truncated: bool = False
    unknown_chunks: dict = field(default_factory=dict)
    schema_errors: list = field(default_factory=list)

    # =====================================================================
    # 诊断入口。【为什么把它们放在库里而不是各写各的脚本】
    # 每个人自己拼一遍"哪几维冻了"，拼法各不相同，结论就没法互相对照 ——
    # 而排查最怕的就是两个人看同一份文件得出不同结论却不知道为什么。
    # =====================================================================

    def timeline(self, ui=True):
        """整场录制的时间轴：操作事件 + 标记 + 标定节点，按墙钟排。

        【为什么要有这一个】排查的第一个动作永远是"先看看这一场都发生了什么"。
        以前这件事要分别去翻 events / marks / romcalib 三处再手工对齐时间 ——
        而对齐时间正是最容易出错、且错了不会报错的一步。

        ui=False 可以滤掉面板操作，只看关键节点。
        """
        rows = []
        t0 = None
        for ev, txt in self.events:
            w = int(ev["wallNs"])
            if int(ev["code"]) == 21:
                t0 = w
        for ev, txt in self.events:
            code = int(ev["code"])
            if not ui and code == 20:
                continue
            rows.append((int(ev["wallNs"]), code, EVENT_NAME.get(code, code), txt,
                         int(ev["frameTsNs"])))
        # marks 是 (wallNs, kind, text) 的三元组，不是 numpy 记录 —— 跟 events
        # 的结构不同。【这一处实测踩过】三元组按二元组解包会当场炸，
        # 而炸在这里比悄悄少一类标记好：少了的话没人会发现时间轴不全。
        for mk in (self.marks or []):
            rows.append((int(mk[0]), -1,
                         MARK_KIND.get(int(mk[1]), "打标记"), str(mk[2]), -1))
        rows.sort(key=lambda r: r[0])
        if t0 is None and rows:
            t0 = rows[0][0]
        out = []
        for k, (w, code, name, txt, fts) in enumerate(rows):
            rel = (w - t0) / 1e9 if t0 else 0.0
            frame = f"  帧{fts}" if fts > 0 else ""
            # 【v8.1 修过的一个坑，读文件的人也该知道】ROM 按钮这类"点一下切一次
            # 状态、切换时还弹模态框"的按钮，旧版录制器会把同一次点击错记成
            # 两条相隔一两秒的事件——现在已经用 pressed() 抢时间戳修掉了，
            # 但如果你读到的是旧版本录的文件，这条提示能帮你认出这种假象：
            # 同一个按钮名字、时间差在 3 秒以内，大概率是同一次点击被拆开了。
            note = ""
            if k > 0:
                pw, pcode, pname, ptxt, _ = rows[k-1]
                if pcode == 20 == code and pname == name:
                    dt = (w - pw) / 1e9
                    if 0 < dt < 3.0:
                        note = (f"   <- 跟上一条同名，只差 {dt:.2f}s，"
                               "很可能是同一次点击被旧版拆成两条")
            out.append(f"[{rel:7.2f}s] {name:<12} {txt}{frame}{note}")
        if not out:
            return ["没有事件 —— v8.1 之前录的文件不记面板操作"]
        return out

    def ui_snapshot(self, which="start"):
        """录制开始/停止那一刻，面板上每个控件的值。

        which: "start" | "stop"
        【为什么这份快照不可替代】增量事件只记被改动过的控件；一个全程没被
        碰过的控件不产生任何事件，它的值在文件里就是空白。而"没被碰过的那些"
        才是系统运行的大背景 —— 出问题时要先确认其它东西都在预期位置，
        才轮得到看变了的那几个。
        """
        # param_deltas 里存的已经是解析好的 dict（load() 就 json.loads 过了），
        # 不是原始字符串 —— 再 loads 一次会当场炸。
        key = "uiSnapshotStart" if which == "start" else "uiSnapshotStop"
        for _, d in (self.param_deltas or []):
            if isinstance(d, dict) and key in d:
                return d[key].get("controls", [])
        return []

    def ui_changes(self):
        """把面板操作事件整理成"控件 -> 改动次数 / 最终值"。"""
        from collections import OrderedDict
        agg = OrderedDict()
        for ev, txt in self.events:
            if int(ev["code"]) != 20:
                continue
            name = txt.split(" [")[0]
            a = agg.setdefault(name, {"n": 0, "first": None, "last": None, "txt": ""})
            a["n"] += 1
            if a["first"] is None:
                a["first"] = float(ev["valueB"])
            a["last"] = float(ev["valueA"])
            a["txt"] = txt
        if not agg:
            return ["整场录制没有任何面板操作 —— 这本身是一条结论"]
        out = [f"{'控件':<22}{'次数':>4}{'起始':>12}{'最终':>12}"]
        for k, a in agg.items():
            out.append(f"{k:<22}{a['n']:>4}{a['first']:>12.4g}{a['last']:>12.4g}")
        return out

    def fist_cycles(self, thresh=0.25):
        """从 curl 里数出握拳次数和每次的起止帧。

        【为什么用 curl 而不是关节角】curl 只用球心间距算，不经过腕部系、
        不经过四元数、不经过 anchor —— 关节角失效的那些帧里它还活着。
        这正是它能当标定相位判据的原因，拿来数握拳同理。

        thresh: 归一化 curl 的高低阈值（迟滞用 thresh 和 1-thresh）。
        """
        if self.jointsolve is None or not len(self.jointsolve):
            return ["没有块 32（JointSolve）"]
        js = self.jointsolve
        # 四指 curl 的均值，忽略无效
        c = js["curl"][:, 1:]
        v = js["curlValid"][:, 1:].astype(bool)
        cm = np.where(v, c, np.nan)
        with np.errstate(invalid="ignore"):
            curl = np.nanmean(cm, axis=1)
        ok = ~np.isnan(curl)
        if ok.sum() < 10:
            return ["curl 有效帧太少，数不出握拳"]
        lo, hi = np.nanpercentile(curl, 5), np.nanpercentile(curl, 95)
        if hi - lo < 1e-6:
            return ["curl 全程没有变化 —— 没有做出握拳/张开动作"]
        u = (curl - lo) / (hi - lo)
        # 迟滞：低于 thresh 算张开，高于 1-thresh 算握拳
        state, cycles, start = 0, [], None
        for i, x in enumerate(u):
            if np.isnan(x):
                continue
            if state == 0 and x > 1 - thresh:
                state, start = 1, i
            elif state == 1 and x < thresh:
                state = 0
                if start is not None:
                    cycles.append((start, i))
        ts = js["frameTsNs"]
        out = [f"共 {len(cycles)} 次握拳（curl 归一化，迟滞阈值 {thresh}）"]
        for k, (a, b) in enumerate(cycles):
            dur = (int(ts[b]) - int(ts[a])) / 1e9 if ts[a] > 0 and ts[b] > 0 else 0
            out.append(f"  第{k+1}次  帧 {a}..{b}  ({dur:.1f}s)")
        return out

    def why_frozen(self, min_run=30):
        """哪几维的输出不动了，以及【是哪条路造成的】。

        通向"输出是一个不动的数"的路有四条，输出完全一样而修法毫无共同点。
        这个函数把每一维停在哪条分支、停了多久、当时的判据是什么摆出来。

        min_run: 连续多少帧不变才算"冻结"。默认 30（120fps 下 0.25 秒）。
        """
        if self.rommap is None or not len(self.rommap):
            return ["没有块 33（RomMapDbg）—— 见 block_report() 查为什么"]
        rm = self.rommap
        out = []
        last = rm[-1]
        for i in range(16):
            run = int(last["frozenRun"][i])
            if run < min_run:
                continue
            br = int(last["branch"][i])
            line = f"{JOINT_NAMES[i]:<8} 已冻结 {run} 帧  分支={MAP_BRANCH_NAME.get(br, br)}"
            if br == 1:
                st = int(last["status"][i])
                line += (f"  status={ROM_STATUS_NAME.get(st, st)}"
                         "\n    -> 成因在【标定阶段】：这一维样本太少或几乎全被拒。"
                         "\n       看 why_rom_short()，是上游几何问题的话重标没用。")
            elif br == 2:
                ds = int(last["dofState"][i])
                held = int(last["heldRun"][i])
                line += (f"  dofState={DOF_STATE_NAME.get(ds, ds)}  已保持 {held} 帧"
                         "\n    -> 成因在【本帧】：这一维不新鲜。看块 32 的 "
                         "proxAxisSrc / wristPoseValid / anchorsValid。"
                         "\n       holdOnDegenerate 关掉这条路就没了，"
                         "但那样发出去的是结构性假值。")
            elif br == 3:
                line += ("\n    -> 标定区间塌了(hi-lo≈0)。重标这一维。")
            elif br == 4:
                lo = int(rm["clampLo"][:, i].sum())
                hi = int(rm["clampHi"][:, i].sum())
                n = len(rm)
                line += (f"  钳下端 {lo*100//n}%  钳上端 {hi*100//n}%"
                         "\n    -> 姿态落在标定区间【之外】。不是 bug，是标定时"
                         "没做到这个姿态 —— 补标。")
            elif br == 0:
                line += ("\n    -> 还没标定，走的是 clamp(q, tgtLo, tgtHi) 透传。"
                         "看 why_clamped()。")
            out.append(line)
        if not out:
            return [f"末帧没有连续 {min_run} 帧不变的维。输出没有冻结。"]
        # 【holdOnDegenerate 的状态必须一起报】它决定了分支 2 存不存在，
        # 只报"哪一维冻了"而不报这个开关，读的人没法判断该不该去动它。
        out.append(f"（holdOnDegenerate={bool(last['holdOnDegenerate'])}  "
                   f"romReady={bool(last['romReady'])}  "
                   f"mapperJustReset 出现过 {int(rm['mapperJustReset'].sum())} 次）")
        return out

    def why_clamped(self, frac=0.5):
        """哪几维长期触限，以及当时的目标行程是多少。

        【未标定时这是最常见的一类误判】走的是 clamp(q, tgtLo, tgtHi)，
        而 q 带一个未知的常量零位偏置（hm20 腕部系的 +X 不是解剖中立位方向）。
        偏置一大就整段贴在限位上 —— 而光看输出，贴限位和"手真的做到了极限"
        长得一模一样。tgtLo/tgtHi 是 v8 才开始记的。
        """
        if self.rommap is None or not len(self.rommap):
            return ["没有块 33（RomMapDbg）"]
        rm = self.rommap
        n = len(rm)
        out = []
        for i in range(16):
            lo = int(rm["clampLo"][:, i].sum())
            hi = int(rm["clampHi"][:, i].sum())
            if max(lo, hi) < frac * n:
                continue
            end = "下端" if lo >= hi else "上端"
            pct = max(lo, hi) * 100 // n
            # 【分支取"触限那些帧"里的众数，不取末帧】触限往往发生在标定之前，
            # 而末帧多半已经标定完、走的是另一条分支 —— 拿末帧的分支去解释
            # 之前的触限，等于答非所问。
            sel = (rm["clampLo"][:, i] | rm["clampHi"][:, i]).astype(bool)
            br = int(np.bincount(rm["branch"][sel, i]).argmax()) if sel.any() else 0
            k = int(np.argmax(sel))          # 第一帧触限时的目标行程
            tl = float(rm["tgtLo"][k, i])
            th = float(rm["tgtHi"][k, i])
            cus = "自定义" if rm["custom"][k, i] else "默认"
            out.append(
                f"{JOINT_NAMES[i]:<8} {pct}% 的帧贴在{end}  "
                f"目标行程[{np.degrees(tl):.0f}°, {np.degrees(th):.0f}°]({cus})  "
                f"分支={MAP_BRANCH_NAME.get(br, br)}")
        if not out:
            return ["没有长期触限的维。"]
        out.append("  -> branch=0 时是【未标定透传】：先看 JointOut.qSolve 的均值"
                   "是不是整体偏在行程外，那是常量零位偏置，ROM 归一化会吸收掉它。")
        return out

    def why_output_missing(self):
        """下游收不到数据时，先跑这个。

        "面板在刷数但 Unity 收不到"有四种成因，而它们在 v8 之前【全部无痕】：
          开关是关的 / 打包失败 / 网络栈拒收 / 发出去了但对端没配。
        前三种这里直接给出答案；只有第四种才轮到去查对端。
        """
        if self.udpout is None or not len(self.udpout):
            return ["没有块 23（UdpOut）—— 见 block_report()"]
        uo = self.udpout
        out = []
        for which, pre in ((0, "m3ds"), (1, "m3dq")):
            errs = uo[pre + "Err"]
            name = "M3DS(关节角)" if which == 0 else "M3DQ(四元数)"
            vals, cnt = np.unique(errs, return_counts=True)
            parts = [f"{udp_err(v)} {c*100//len(uo)}%" for v, c in zip(vals, cnt)]
            out.append(f"{name}  " + "　".join(parts))
            ok = int((errs == 0).sum())
            if ok:
                seq = uo[pre + "Seq"]
                # 【序号跳变=丢帧，而丢的是发送端还是接收端在这里就分开了】
                d = np.diff(seq[errs == 0])
                gaps = int((d > 1).sum())
                if gaps:
                    out.append(f"  序号跳变 {gaps} 次 —— 发送端自己就漏了帧，"
                               "不用去查对端")
        tgt = uo["targetPort"][-1]
        en = uo["enabled"][-1]
        out.append(f"目标端口 {int(tgt) if int(tgt) >= 0 else '未知'}  "
                   f"开关={'开' if en == 1 else ('关' if en == 0 else '未知')}")
        # 两个包的时间戳对不上会让下游插值出鬼 —— 这一条只有配对记录查得出来
        both = uo[(uo["m3dsBytes"] > 0) & (uo["m3dqBytes"] > 0)]
        if len(both):
            out.append(f"两包配对 {len(both)}/{len(uo)} 帧（同一 frameTsNs）")
        return out

    def why_rom_short(self):
        """ROM 为什么标不满：逐维摆出覆盖度、样本数、逐原因拒收数和判据。"""
        if self.romcalib is None or not len(self.romcalib):
            return ["没有块 31（RomCalib）—— 这一轮没做过 ROM 标定？"]
        fin = self.romcalib[self.romcalib["reason"] == 2]
        rc = fin[-1] if len(fin) else self.romcalib[-1]
        out = [f"覆盖 {rc['coverageFlex']*100:.0f}%  达标 {rc['nOkFlex']}/11  "
               f"方向纠正 {rc['nSignFlipped']}  失效 {rc['nDeadFlex']}  "
               f"ready={bool(rc['ready'])}  用时 {rc['durationSec']:.1f}s"]
        if self.romcalib_legacy:
            out.append("【旧版 2040 字节 RomCalib】配置快照是升级填的哨兵(-1)，不是真读数")
        elif float(rc["cfgMinRangeRad"]) >= 0:
            out.append(
                f"判据：minRange {np.degrees(rc['cfgMinRangeRad']):.0f}°  "
                f"minSamples {rc['cfgMinSamplesPerDof']}  "
                f"达标数 {rc['cfgMinOkFlexDofs']}/11  "
                f"收预测={bool(rc['cfgAcceptPredicted'])}  "
                f"外推={bool(rc['cfgExtrapolateByCurl'])}(R²>{rc['cfgExtrapMinR2']:.2f})  "
                f"退化帧保持={bool(rc['cfgHoldOnDegenerate'])}")
        out.append("")
        for i in range(16):
            st = int(rc["status"][i])
            if st == 0:
                continue
            rej = rc["nReject"][i]
            seen = int(rc["nSeen"][i]) or 1
            worst = int(np.argmax(rej[1:])) + 1
            out.append(
                f"{JOINT_NAMES[i]:<8} {ROM_STATUS_NAME.get(st, st):<14} "
                f"覆盖 {rc['coverage'][i]*100:3.0f}%  样本 {rc['nSamples'][i]:>4}/{seen}  "
                f"主因 {ROM_REJECT_NAME.get(worst, worst)} {rej[worst]*100//seen}%  "
                f"sign={rc['sign'][i]:+d}{'' if rc['signResolved'][i] else '(默认)'}")
        # 【骨轴退化率要一起报】它是 AxisDegen 拒收的上游成因，
        # 而"多标几次"对它完全无效 —— 不摆出来，人一定会先去重标。
        if self.jointsolve is not None and len(self.jointsolve):
            js = self.jointsolve
            degen = (js["proxAxisSrc"] == 1).mean(axis=0) * 100
            out += ["", "近节骨轴退化率(拇/食/中/无/小) "
                    + " ".join(f"{d:.0f}%" for d in degen)
                    + "   ← 退化=a0 与 a1 同向，叉乘路径的 PIP 恒为 0"]
            if "pipUsedQuat" in js.dtype.names and not self.jointsolve_legacy:
                uq = js["pipUsedQuat"].mean(axis=0) * 100
                out.append("走四元数路径的比例          "
                           + " ".join(f"{d:.0f}%" for d in uq)
                           + "   ← 这条路解出来的是真值，不该被当退化拒掉")
        return out

    def block_report(self):
        """每一块：清单里有没有、开没开、写了多少、丢了多少。

        【它回答的是"这一块为什么不在文件里"】三种成因下一步动作毫无共同点：
          清单里没有             -> 录制端版本老，不认识这个块
          enabled=false         -> 被 Detail 档位关掉，重录时调档
          enabled=true, pushed=0 -> 代码路径一次都没走到，真 bug
          dropped>0             -> 队列满丢的，换盘或降档
        """
        man = self.header.get("manifest") or {}
        blocks = {int(b["id"]): b for b in man.get("blocks", [])}
        cs = self.chunkstats
        out = []
        if not blocks:
            out.append("文件头没有 manifest（v8 之前录的）——"
                       "只能靠 detail 字段猜，分不开\"版本老\"和\"没触发\"")
        if cs is None:
            out.append("没有块 35（ChunkStats）—— 分不开\"没写\"和\"被丢了\"")
        if not blocks and cs is None:
            return out
        out.append(f"{'块':<4}{'名字':<14}{'开':<4}{'写入':>9}{'丢弃':>8}{'字节':>12}  结论")
        for cid in sorted(set(blocks) | ({i for i in range(64)
                                          if cs is not None and cs["pushed"][i]} if cs is not None else set())):
            b = blocks.get(cid)
            name = b["name"] if b else CHUNK_NAME.get(cid, f"?{cid}")
            en = (b["enabled"] if b else True)
            pu = int(cs["pushed"][cid]) if cs is not None else -1
            dr = int(cs["dropped"][cid]) if cs is not None else -1
            by = int(cs["bytes"][cid]) if cs is not None else -1
            if b is None:
                note = "清单里没有：录制端版本老"
            elif not en:
                note = f"档位关掉（需要 {b.get('needs','?')}）"
            elif pu == 0 and cid in (CT_TRAILER, CT_CHUNKSTATS):
                # 这两块是在统计快照之后才 push 的，计数天然滞后一帧。
                # 录制端 v8 起会自己补上；老文件里是 0，不是 bug。
                note = "（收尾块，计数在快照之后）"
            elif pu == 0:
                note = "【开着但一次都没写 —— 真 bug，去查为什么没触发】"
            elif dr > 0:
                note = f"丢了 {dr*100//max(1,pu+dr)}%，队列满"
            else:
                note = ""
            out.append(f"{cid:<4}{name:<14}{'是' if en else '否':<4}"
                       f"{pu:>9}{dr:>8}{by:>12}  {note}")
        if cs is not None:
            peak = int(cs["queuePeak"])
            out.append(f"队列峰值 {peak}/16384 ({peak*100//16384}%)   "
                       "【长期过半就说明盘跟不上，等丢块出现时数据已经有洞了】")
        return out

    def chain(self, dof, frame=-1):
        """把一维关节角在【全链路每一级】上的值和判据摆成一行一级。

        这是"信号在哪一级消失"的直接答案：从解算内部一路到发出去的包。
        """
        out = [f"=== {JOINT_NAMES[dof]} (第 {dof} 维)  帧 {frame} ==="]
        d = np.degrees
        if self.jointsolve is not None and len(self.jointsolve):
            js = self.jointsolve[frame]
            f = 0 if dof < 4 else 1 + (dof - 4) // 3
            out += [f"0 解算内部  proxAxisSrc={PROX_AXIS_NAME.get(int(js['proxAxisSrc'][f]))}"
                    f"  dot(a0,a1)={js['dotProxMid'][f]:.4f}"
                    f"  curl={js['curl'][f]:.3f}"
                    f"  dofState={DOF_STATE_NAME.get(int(js['dofState'][dof]))}",
                    f"            wristPoseValid={bool(js['wristPoseValid'])}"
                    f"  anchorsValid={bool(js['anchorsValid'])}"
                    f"  wristStale={int(js['wristStale'])}"]
        if self.jointout is not None and len(self.jointout):
            jo = self.jointout[frame]
            out += [f"1 qSolve     {d(jo['qSolve'][dof]):8.2f}°",
                    f"2 qSmooth    {d(jo['qSmooth'][dof]):8.2f}°"]
        if self.anglechain is not None and len(self.anglechain):
            ac = self.anglechain[frame]
            out.append(f"  平滑器     off={d(ac['smOffset'][dof]):+.2f}°"
                       f"{' 【被钳住=预测段跑飞≥40°】' if ac['smOffClamped'][dof] else ''}"
                       f"  s={d(ac['smState'][dof]):.2f}°  alpha={ac['smAlpha']:.2f}"
                       f"  on={bool(ac['smOn'])}")
        if self.rommap is not None and len(self.rommap):
            rm = self.rommap[frame]
            out.append(f"  ROM映射    分支={MAP_BRANCH_NAME.get(int(rm['branch'][dof]))}"
                       f"  u={rm['uNorm'][dof]:+.3f}"
                       f"  目标[{d(rm['tgtLo'][dof]):.0f}°,{d(rm['tgtHi'][dof]):.0f}°]"
                       f"  sign={int(rm['sign'][dof]):+d}"
                       f"  冻结 {int(rm['frozenRun'][dof])} 帧")
        if self.jointout is not None and len(self.jointout):
            jo = self.jointout[frame]
            out += [f"3 qRom       {d(jo['qRom'][dof]):8.2f}°"
                    f"   ROM区间[{d(jo['romLo'][dof]):.0f}°,{d(jo['romHi'][dof]):.0f}°]",
                    f"4 qOut       {d(jo['qOut'][dof]):8.2f}°"]
        if self.anglechain is not None and len(self.anglechain):
            ac = self.anglechain[frame]
            out.append(f"  限幅器     削={d(ac['rlClip'][dof]):+.2f}°"
                       f"  步长上限={d(ac['rlStepLimitRad']):.2f}°  on={bool(ac['rlOn'])}")
        if self.udpout is not None and len(self.udpout):
            uo = self.udpout[frame]
            out.append(f"5 UDP       M3DS {int(uo['m3dsBytes'])}B seq={int(uo['m3dsSeq'])} "
                       f"{udp_err(uo['m3dsErr'])}")
            out.append(f"            M3DQ {int(uo['m3dqBytes'])}B seq={int(uo['m3dqSeq'])} "
                       f"{udp_err(uo['m3dqErr'])}")
        return out

    def summary(self):
        n = lambda a: 0 if a is None else len(a)
        s = [f"文件      {self.path}",
             f"版本      v{self.version}" + ("  [文件被截断]" if self.truncated else ""),
             f"协议      {self.header.get('protocol','?')}   备注 {self.header.get('note','')}",
             f"相机      {len(self.blobs)} 台，2D 帧 {sum(len(v) for v in self.blobs.values())}",
             f"骨架帧    {len(self.skeleton)}",
             f"3D点帧    {len(self.points3d)}   簇帧 {len(self.cluster)}",
             f"诊断帧    {n(self.diag)}",
             f"网络原始  {len(self.netraw)}   关联输入 {len(self.assoc_input)}",
             f"关节角帧  {n(self.jointout)}   手性帧 {n(self.handed)}",
             f"逐点来源  {n(self.markerdbg)}   开关帧 {n(self.runflags)}"
             f"   状态JSON {len(self.statejson)}"]
        if self.version >= 5:
            s += [f"位置六级  {n(self.stages)}   滤波前后 {n(self.filterdbg)}"
                  f"   IK内部 {n(self.ikdbg)}",
                  f"时基耗时  {n(self.timing)}   自标定 {n(self.autocalib)}"
                  f"   UDP输出 {n(self.udpout)}",
                  f"模板快照  {len(self.templates)}   事件 {len(self.events)}"
                  f"   追踪内部 {len(self.trackdbg)}   2D关联 {len(self.clusterlink)}"]
            if self.assignfull or self.netinput:
                s.append(f"全量指派  {len(self.assignfull)}   网络输入 {len(self.netinput)}")
            # 模板指纹变化：手性问题的第一现场，直接摆在概览里
            if len(self.templates) > 1:
                fps = [int(t["fingerprint"]) for t in self.templates]
                if len(set(fps)) > 1:
                    s.append(f"          【模板在录制中途变过 {len(set(fps))} 次】"
                             f"见 templates，逐份比对 isRight 和手背点")
        if self.version >= 7:
            s += [f"ROM标定   {n(self.romcalib)} 份   解算内部 {n(self.jointsolve)}"
                  f"   链式续解 {n(self.chaindbg)}"]
            # 【概览里就把最要命的两个数摆出来】"标不满"和"方向反了"这两个症状，
            # 十有八九在这两行里就能定性，不用先跑分析脚本。
            if self.jointsolve is not None and len(self.jointsolve):
                js = self.jointsolve
                degen = (js["proxAxisSrc"] == 1).mean(axis=0) * 100
                s.append("          近节骨轴退化率(拇/食/中/无/小) "
                         + " ".join(f"{d:.0f}%" for d in degen)
                         + "   ← 退化时 PIP 恒等于 0")
            if self.romcalib is not None and len(self.romcalib):
                fin = self.romcalib[self.romcalib["reason"] == 2]
                last = fin[-1] if len(fin) else self.romcalib[-1]
                s.append(f"          最后一份：覆盖 {last['coverageFlex']*100:.0f}% "
                         f"达标 {last['nOkFlex']}/11 方向纠正 {last['nSignFlipped']} "
                         f"失效 {last['nDeadFlex']} ready={bool(last['ready'])}")
        if self.version >= 8:
            s += [f"映射分支  {n(self.rommap)}   后处理链 {n(self.anglechain)}"
                  f"   UDP输出 {n(self.udpout)}"]
            if self.romcalib_legacy or self.jointsolve_legacy:
                s.append("          【旧布局升级】"
                         + ("RomCalib 的配置快照 " if self.romcalib_legacy else "")
                         + ("JointSolve 的四元数路径 " if self.jointsolve_legacy else "")
                         + "是哨兵值，不是真读数")
            # 【冻结直接摆在概览里】"标定完输出就不动了"是现场报得最多的一类，
            # 而它的答案是一个数：末帧最长的连续冻结帧数。
            if self.rommap is not None and len(self.rommap):
                rm = self.rommap[-1]
                if int(rm["maxFrozenRun"]) > 30:
                    i = int(rm["maxFrozenIdx"])
                    s.append(f"          【输出冻结】{JOINT_NAMES[i]} 已 "
                             f"{int(rm['maxFrozenRun'])} 帧不变，分支="
                             f"{MAP_BRANCH_NAME.get(int(rm['branch'][i]))}"
                             f"　→ why_frozen()")
                if int(rm["nClamped"]) or (self.rommap["clampLo"] | self.rommap["clampHi"]).any():
                    nc = int((self.rommap["clampLo"] | self.rommap["clampHi"]).any(axis=0).sum())
                    s.append(f"          触限维数 {nc}/16　→ why_clamped()")
            # 【发不出去也摆出来】"面板在刷数但 Unity 收不到"最常见的成因
            # 就是开关没开，而它在 v8 之前完全无痕。
            if self.udpout is not None and len(self.udpout):
                bad = int((self.udpout["m3dsErr"] != 0).sum())
                if bad:
                    out_pct = bad * 100 // len(self.udpout)
                    s.append(f"          【M3DS {out_pct}% 没发出去】"
                             f"{udp_err(self.udpout['m3dsErr'][-1])}"
                             f"　→ why_output_missing()")
            # 【丢块也摆出来】丢块 = 数据有洞，会把结论带偏却不留痕迹。
            if self.chunkstats is not None:
                dr = int(self.chunkstats["droppedTotal"])
                pk = int(self.chunkstats["queuePeak"])
                if dr:
                    s.append(f"          【丢块 {dr}】队列峰值 {pk}/16384"
                             f"　→ block_report() 看丢的是哪几类")
                elif pk > 8192:
                    s.append(f"          队列峰值 {pk}/16384（过半，盘快跟不上了）")
            rom_ev = [e for e in self.events if int(e[0]["code"]) in (16, 17, 18, 19)]
        elif self.version >= 7:
            rom_ev = [e for e in self.events if int(e[0]["code"]) in (16, 17, 18, 19)]
        else:
            rom_ev = []
        if self.version >= 7:
            if rom_ev:
                s.append(f"          ROM 事件 {len(rom_ev)} 条（按钮 16/17，帧对齐 18/19）")
            # 【逐指 anchor / 弯曲平面，也摆进概览】理由同上面那两条：
            # "遮挡点摆歪了"这个症状，十有八九在这三行里就能定性。
            # 读的顺序是固定的：先看平面学到没有，再看 anchor 降级没有，
            # 最后才看出平面角具体多大 —— 前两个是"能不能信"，第三个是"有多坏"。
            if self.chaindbg is not None and len(self.chaindbg):
                cd = self.chaindbg
                fin = "拇/食/中/无/小"
                if self.chaindbg_legacy:
                    s.append("          【旧版 192 字节 chaindbg】逐指 anchor/平面"
                             "三个量是升级填的哨兵(-1)，不是真读数")
                else:
                    # 平面累加器吃了多少帧。平面【只】从 pp/mp/dp 三点全见的帧学，
                    # 所以这个数同时也是"这根手指有多少帧三点全见"的直接计量。
                    # 【取 max 不取最后一帧】这个计数只在 resetHistory 时清零，
                    # 其余时候单调增。要回答的问题是"这根手指到底有没有攒够过"，
                    # 那就该看它最好的时候到过多少 —— 末帧可能正好在一次复位之后。
                    psn = cd["planeSampleN"].max(axis=0)
                    s.append(f"          平面样本数({fin}) "
                             + " ".join(f"{int(v)}" for v in psn)
                             + "   ← 个位数=平面基本没学到，此时 hasPlane 为真也不能信")
                    # 逐指 anchor 结论。0 = 已退回 pp->mp（纯实测量，不依赖标定）。
                    # 【取末帧】这是个单向锁，降级了就不再翻回来，末帧就是最终结论。
                    aok = cd["anchorOkPerFinger"][-1]
                    s.append(f"          anchor可用({fin}) "
                             + " ".join("是" if v else "否" for v in aok)
                             + "   ← 否=已退回 pp->mp，是预期的降级不是故障")
                    # 出平面角：整条诊断链的核心量。cross(u0,u1) 对 u0 的出平面
                    # 分量有 1/sin(MCP角) 的放大，实测 17.8~59.7° -> 平面歪 18.8~77.9°。
                    oop = cd["anchorOutOfPlaneDeg"]
                    med = []
                    for f in range(5):
                        col = oop[:, f]
                        col = col[col >= 0]          # <0 = 本帧量不出来，剔掉
                        med.append(f"{np.median(col):.0f}°" if len(col) else "—")
                    s.append(f"          anchor出平面中位({fin}) " + " ".join(med)
                             + "   ← >25° 就该被降级；— 表示全程没在用 anchor")
                # 链式续解的守卫通过率：hasPipHold 大面积为 0 是 v6.2 的【预期行为】
                # （无 anchor 就不写假的 MCP 角，见 Hm20Assoc_chain.ipp），
                # 不写出来的话很容易被当成"改坏了"。
                hp = cd["hasPipHold"].mean(axis=0) * 100
                s.append(f"          hasPipHold率({fin}) "
                         + " ".join(f"{v:.0f}%" for v in hp)
                         + ("   ← 旧版行为：无 anchor 时写的是恒 0 的假角度"
                            if self.chaindbg_legacy else
                            "   ← v6.2 起无 anchor 就不写，低是对的"))
        if self.trailer:
            s.append(f"结尾统计  块 {self.trailer['chunks']}  丢块 {self.trailer['dropped']}"
                     + ("   ← 有丢块，数据有洞" if self.trailer["dropped"] else ""))
        else:
            s.append("结尾统计  【缺失】录制没有正常停止")
        if self.unknown_chunks:
            s.append(f"未知块    {self.unknown_chunks}")
        if self.schema_errors:
            s.append("【格式不匹配】" + "; ".join(sorted(set(self.schema_errors))))
        # v3 文件读到这里会显示一堆 0 —— 明说原因，别让人以为是数据坏了
        if self.version < 4:
            s.append("注意      这是 v3 文件，没有关节角/手性/逐点来源三个块。"
                     "要查输出角和手性问题必须用 v4 重录。")
        return "\n".join(s)


def _chaindbg_upgrade(v_old):
    """把旧版 192 字节的 ChainDbgRec 升级成新版 240 字节的布局。

    新字段填【哨兵值】，不填 0 —— 因为 0 在这三个量里都是有意义的读数：
      anchorOkPerFinger = 0  是"这根 anchor 被降级了"（一条很强的结论）
      planeSampleN      = 0  是"平面一帧都没学到"（也是一条很强的结论）
    旧文件里这两件事都【无从得知】，填 0 等于凭空造出一个结论。
    所以：
      anchorOkPerFinger   -> 跟旧版的全局 anchorsValid 走。那是当时唯一存在的
                             信息，逐指结论在旧文件里根本没被记录过。
      planeSampleN        -> -1，"这个文件没有这个量"
      anchorOutOfPlaneDeg -> -1，跟 C++ 侧"本帧量不出来"用同一个哨兵
    读分析脚本时按 <0 过滤掉即可，不会跟真实读数混在一起。
    """
    a = np.zeros(1, dtype=CHAINDBG)
    for name in CHAINDBG_V1.names:
        if name.startswith("_"):
            continue
        a[name][0] = v_old[name]
    a["anchorOkPerFinger"][0] = np.uint8(v_old["anchorsValid"])
    a["planeSampleN"][0] = -1
    a["anchorOutOfPlaneDeg"][0] = -1.0
    # recBytes 保持文件里的原值：这条记录【确实】是从 192 字节的文件里读的，
    # 改成 240 会让"这份数据是哪个版本录的"这个信息消失。
    return a[0]


def _romcalib_upgrade(v_old):
    """v7 的 2040 字节 RomCalibRec 升级到 v8 的 2112。

    新增的是 rom::Config 快照。【全部填哨兵 -1，绝不填默认值】——
    填 0.35（当时的默认 minRangeRad）等于替这份文件断言"当时没人调过阈值"，
    而那恰恰是要查的事情本身。-1 的含义是"这个文件没有这个量"，
    读脚本按 <0 过滤即可，不会跟真实读数混在一起。
    """
    a = np.zeros(1, dtype=ROMCALIB)
    for name in ROMCALIB_V7.names:
        if name.startswith("_"):
            continue
        a[name][0] = v_old[name]
    for name in ROMCALIB.names:
        if name.startswith("cfg"):
            a[name][0] = -1 if ROMCALIB[name].kind in "fi" else 255
    return a[0]


def _jointsolve_upgrade(v_old):
    """v7 的 504 字节 JointSolveRec 升级到 v8 的 632。

    新增的是 PIP 的四元数解算路径。【pipFromQuat 填 -1 而不是 0】——
    0 是一条合法读数（"PIP 解出来就是 0"），而那正是骨轴退化的症状，
    填 0 会凭空造出一个"退化"的结论。
    pipFromAxis 用旧的 pipRaw：v7 时代只有叉乘一条路，pipRaw 就是它。
    pipUsedQuat 填 0：那个年代确实没走四元数路径。
    """
    a = np.zeros(1, dtype=JOINTSOLVE)
    for name in JOINTSOLVE_V7.names:
        if name.startswith("_"):
            continue
        a[name][0] = v_old[name]
    a["qRelPip"][0] = np.nan
    a["pipFromQuat"][0] = -1.0
    a["pipFromAxis"][0] = v_old["pipRaw"]
    a["pipUsedQuat"][0] = 0
    return a[0]


def _take(rec, dt, payload, name):
    """按自描述头对账后再解析。不匹配就记错误并返回 None —— 【绝不猜】。"""
    if len(payload) < 4:
        return None
    _ver, nbytes = struct.unpack_from("<HH", payload, 0)
    if nbytes != dt.itemsize:
        rec.schema_errors.append(
            f"{name}: 文件里每条 {nbytes} 字节，本脚本的 dtype 是 {dt.itemsize} 字节")
        return None
    if len(payload) < dt.itemsize:
        return None
    return np.frombuffer(payload, dt, 1, 0)[0]


def _parse_cluster_link(p):
    """块 28 是变长嵌套的，单独拆一个函数。

    布局：头 | nTracks 个 {u8 nSup; pad3; nSup 个 (u8 cam, pad, u16 obs)} | nUnmatched 个 (同)
    """
    h = np.frombuffer(p, CLUSTERLINK_HDR, 1, 0)[0]
    off = CLUSTERLINK_HDR.itemsize
    support = []
    for _ in range(int(h["nTracks"])):
        nsup = p[off]
        off += 4
        s = []
        for _ in range(nsup):
            cam = p[off]
            obs = struct.unpack_from("<H", p, off + 2)[0]
            s.append((int(cam), int(obs)))
            off += 4
        support.append(s)
    unmatched = []
    for _ in range(int(h["nUnmatched"])):
        cam = p[off]
        obs = struct.unpack_from("<H", p, off + 2)[0]
        unmatched.append((int(cam), int(obs)))
        off += 4
    return (int(h["tsNs"]), int(h["nCams"]), support, unmatched)


def load(path, want=None):
    r = Recording(path=path)
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < 16 or raw[:8] != MAGIC:
        raise ValueError("不是 .pcrec 文件")
    r.version = struct.unpack_from("<I", raw, 8)[0]
    hlen = struct.unpack_from("<I", raw, 12)[0]
    try:
        r.header = json.loads(raw[16:16 + hlen].decode("utf-8", "replace"))
    except Exception as e:
        r.header = {"_parse_error": str(e)}
    off = 16 + hlen
    rows, dt = [], _diag_dtype()
    jo_rows, hd_rows, md_rows, rf_rows = [], [], [], []
    ms_rows, fd_rows, ik_rows, tm_rows, uo_rows, ac_rows = [], [], [], [], [], []
    cd_rows, rc_rows, jsv_rows = [], [], []
    # ---- v8 ----
    rmp_rows, angc_rows = [], []
    # 这份文件的 romcalib / jointsolve 是不是从 v7 布局升级来的。
    # 【必须记下来】升级后新字段是哨兵值，跟"真的量出来是这样"完全是两回事。
    rc_legacy = False
    jsv_legacy = False
    # 这份文件的 chaindbg 是不是从旧版 192 字节布局升级来的。
    # 【必须记下来】升级后新字段是哨兵值(-1)，跟"真的量出来是这样"完全是两回事，
    # 概览里要能一眼看出来，否则拿旧文件当新文件读会得出反的结论。
    cd_legacy = False
    while off + 5 <= len(raw):
        ct = raw[off]
        clen = struct.unpack_from("<I", raw, off + 1)[0]
        body = off + 5
        if body + clen > len(raw):
            r.truncated = True
            break
        p = raw[body:body + clen]
        off = body + clen
        if want is not None and ct not in want:
            continue
        try:
            if ct == CT_CAM_BLOBS:
                cam = struct.unpack_from("<I", p, 0)[0]
                ts = struct.unpack_from("<q", p, 4)[0]
                nb = struct.unpack_from("<H", p, 12)[0]
                r.blobs.setdefault(cam, []).append((ts, np.frombuffer(p, BLOB2D, nb, 14)))
            elif ct == CT_POINTS3D:
                ts = struct.unpack_from("<q", p, 0)[0]
                nb = struct.unpack_from("<H", p, 8)[0]
                r.points3d.append((ts, np.frombuffer(p, POINT3D, nb, 10)))
            elif ct == CT_CLUSTER3D:
                ts = struct.unpack_from("<q", p, 0)[0]
                nb = struct.unpack_from("<H", p, 8)[0]
                r.cluster.append((ts, np.frombuffer(p, CLUSTER, nb, 10)))
            elif ct == CT_SKELETON:
                ts = struct.unpack_from("<q", p, 0)[0]
                r.skeleton.append((ts, np.frombuffer(p, SKEL, 1, 8)[0]))
            elif ct == CT_HM20DIAG:
                rows.append(np.frombuffer(p, dt, 1, 0)[0])
            elif ct == CT_JOINTOUT:
                v = _take(r, JOINTOUT, p, "JointOutRec")
                if v is not None:
                    jo_rows.append(v)
            elif ct == CT_HANDEDNESS:
                v = _take(r, HANDEDNESS, p, "HandednessRec")
                if v is not None:
                    hd_rows.append(v)
            elif ct == CT_MARKERDBG:
                v = _take(r, MARKERDBG, p, "MarkerDbgRec")
                if v is not None:
                    md_rows.append(v)
            elif ct == CT_RUNFLAGS:
                v = _take(r, RUNFLAGS, p, "RunFlagsRec")
                if v is not None:
                    rf_rows.append(v)
            elif ct == CT_STATEJSON:
                ts = struct.unpack_from("<q", p, 0)[0]
                jl = struct.unpack_from("<I", p, 8)[0]
                txt = p[12:12 + jl].decode("utf-8", "replace")
                try:
                    r.statejson.append((ts, json.loads(txt)))
                except Exception:
                    r.statejson.append((ts, {"_raw": txt}))
            elif ct == CT_NETRAW:
                h = np.frombuffer(p, NETRAW, 1, 0)[0]
                r.netraw.append((h, np.frombuffer(p, NETTOPK, int(h["nCand"]), NETRAW.itemsize)))
            elif ct == CT_ASSOC_INPUT:
                h = np.frombuffer(p, ASSOCIN, 1, 0)[0]
                pt = np.frombuffer(p, np.dtype([("x", "<f4"), ("y", "<f4"),
                                                ("z", "<f4"), ("id", "<i4")]),
                                   int(h["n"]), ASSOCIN.itemsize)
                r.assoc_input.append((h, pt))
            elif ct == CT_CAMSETTINGS:
                r.camsettings.append(np.frombuffer(p, CAMSET, 1, 0)[0])
            elif ct == CT_MARK:
                ts = struct.unpack_from("<q", p, 0)[0]
                tl = struct.unpack_from("<H", p, 9)[0]
                r.marks.append((ts, p[8], p[11:11 + tl].decode("utf-8", "replace")))
            elif ct == CT_PARAM_DELTA:
                ts = struct.unpack_from("<q", p, 0)[0]
                jl = struct.unpack_from("<I", p, 8)[0]
                txt = p[12:12 + jl].decode("utf-8", "replace")
                try:
                    r.param_deltas.append((ts, json.loads(txt)))
                except Exception:
                    r.param_deltas.append((ts, {"_raw": txt}))
            elif ct == CT_MARKERSTAGE:
                v = _take(r, MARKERSTAGE, p, "MarkerStageRec")
                if v is not None:
                    ms_rows.append(v)
            elif ct == CT_FILTERDBG:
                v = _take(r, FILTERDBG, p, "FilterDbgRec")
                if v is not None:
                    fd_rows.append(v)
            elif ct == CT_IKDBG:
                v = _take(r, IKDBG, p, "IkDbgRec")
                if v is not None:
                    ik_rows.append(v)
            elif ct == CT_TIMING:
                v = _take(r, TIMING, p, "TimingRec")
                if v is not None:
                    tm_rows.append(v)
            elif ct == CT_UDPOUT:
                v = _take(r, UDPOUT, p, "UdpOutRec")
                if v is not None:
                    uo_rows.append(v)
            elif ct == CT_AUTOCALIBDBG:
                v = _take(r, AUTOCALIBDBG, p, "AutoCalibDbgRec")
                if v is not None:
                    ac_rows.append(v)
            elif ct == CT_TEMPLATESNAP:
                v = _take(r, TEMPLATESNAP, p, "TemplateSnapRec")
                if v is not None:
                    r.templates.append(v)
            elif ct == CT_EVENT:
                v = _take(r, EVENT, p, "EventRec")
                if v is not None:
                    tl = int(v["textLen"])
                    txt = p[EVENT.itemsize:EVENT.itemsize + tl].decode("utf-8", "replace")
                    r.events.append((v, txt))
            elif ct == CT_CHAINDBG:
                # 【先试新版，对不上再试旧版】不能只试一个：只留新的，历史录制
                # 的 chaindbg 整块读不出来；只留旧的，新录制读不出来。
                # 两个都试过还对不上，才让 _take 记 schema_errors —— "绝不猜"
                # 这条规矩没变，变的只是"认得的版本"从一个变成两个。
                _n = struct.unpack_from("<HH", p, 0)[1] if len(p) >= 4 else 0
                if _n == CHAINDBG_V1.itemsize:
                    v = _take(r, CHAINDBG_V1, p, "ChainDbgRec(v1)")
                    if v is not None:
                        v = _chaindbg_upgrade(v)
                        cd_legacy = True
                else:
                    v = _take(r, CHAINDBG, p, "ChainDbgRec")
                if v is not None:
                    cd_rows.append(v)
            elif ct == CT_ROMCALIB:
                # 【新旧两个布局都试】只认新版的话，历史录制里的 ROM 标定
                # 整块读不出来 —— 而那批文件恰恰是没法重录的。
                _n = struct.unpack_from("<HH", p, 0)[1] if len(p) >= 4 else 0
                if _n == ROMCALIB_V7.itemsize:
                    v = _take(r, ROMCALIB_V7, p, "RomCalibRec(v7)")
                    if v is not None:
                        v = _romcalib_upgrade(v)
                        rc_legacy = True
                else:
                    v = _take(r, ROMCALIB, p, "RomCalibRec")
                if v is not None:
                    rc_rows.append(v)
            elif ct == CT_JOINTSOLVE:
                _n = struct.unpack_from("<HH", p, 0)[1] if len(p) >= 4 else 0
                if _n == JOINTSOLVE_V7.itemsize:
                    v = _take(r, JOINTSOLVE_V7, p, "JointSolveRec(v7)")
                    if v is not None:
                        v = _jointsolve_upgrade(v)
                        jsv_legacy = True
                else:
                    v = _take(r, JOINTSOLVE, p, "JointSolveRec")
                if v is not None:
                    jsv_rows.append(v)
            elif ct == CT_ROMMAPDBG:
                v = _take(r, ROMMAPDBG, p, "RomMapDbgRec")
                if v is not None:
                    rmp_rows.append(v)
            elif ct == CT_ANGLECHAIN:
                v = _take(r, ANGLECHAIN, p, "AngleChainRec")
                if v is not None:
                    angc_rows.append(v)
            elif ct == CT_CHUNKSTATS:
                v = _take(r, CHUNKSTATS, p, "ChunkStatsRec")
                if v is not None:
                    r.chunkstats = v
            elif ct == CT_TRACKDBG:
                h = np.frombuffer(p, TRACKDBG_HDR, 1, 0)[0]
                items = np.frombuffer(p, TRACKDBG_ITEM, int(h["n"]), TRACKDBG_HDR.itemsize)
                r.trackdbg.append((int(h["tsNs"]), items))
            elif ct == CT_CLUSTERLINK:
                r.clusterlink.append(_parse_cluster_link(p))
            elif ct == CT_ASSIGNFULL:
                ts = struct.unpack_from("<q", p, 0)[0]
                nc, ncls = struct.unpack_from("<HH", p, 8)
                m = np.frombuffer(p, "<f4", nc * ncls, 12).reshape(nc, ncls)
                r.assignfull.append((ts, m))
            elif ct == CT_NETINPUT:
                ts = struct.unpack_from("<q", p, 0)[0]
                a, b, c = struct.unpack_from("<III", p, 8)
                o = 20
                cand = np.frombuffer(p, "<f4", a, o); o += a * 4
                tpl = np.frombuffer(p, "<f4", b, o); o += b * 4
                prv = np.frombuffer(p, "<f4", c, o)
                r.netinput.append((ts, cand, tpl, prv))
            elif ct == CT_CLOCKSYNC:
                r.clocksync.append(struct.unpack_from("<qq", p, 0))
            elif ct == CT_TRAILER:
                ts, ch, dr, by = struct.unpack_from("<qQQQ", p, 0)
                r.trailer = {"tsNs": ts, "chunks": ch, "dropped": dr, "bytes": by}
            else:
                r.unknown_chunks[ct] = r.unknown_chunks.get(ct, 0) + 1
        except Exception:
            r.unknown_chunks[f"bad{ct}"] = r.unknown_chunks.get(f"bad{ct}", 0) + 1
    if rows:
        r.diag = np.array(rows, dtype=dt)
    if jo_rows:
        r.jointout = np.array(jo_rows, dtype=JOINTOUT)
    if hd_rows:
        r.handed = np.array(hd_rows, dtype=HANDEDNESS)
    if md_rows:
        r.markerdbg = np.array(md_rows, dtype=MARKERDBG)
    if rf_rows:
        r.runflags = np.array(rf_rows, dtype=RUNFLAGS)
    if ms_rows:
        r.stages = np.array(ms_rows, dtype=MARKERSTAGE)
    if fd_rows:
        r.filterdbg = np.array(fd_rows, dtype=FILTERDBG)
    if ik_rows:
        r.ikdbg = np.array(ik_rows, dtype=IKDBG)
    if tm_rows:
        r.timing = np.array(tm_rows, dtype=TIMING)
    if uo_rows:
        r.udpout = np.array(uo_rows, dtype=UDPOUT)
    if ac_rows:
        r.autocalib = np.array(ac_rows, dtype=AUTOCALIBDBG)
    if cd_rows:
        r.chaindbg = np.array(cd_rows, dtype=CHAINDBG)
        r.chaindbg_legacy = cd_legacy
    if rc_rows:
        r.romcalib = np.array(rc_rows, dtype=ROMCALIB)
    if jsv_rows:
        r.jointsolve = np.array(jsv_rows, dtype=JOINTSOLVE)
    r.romcalib_legacy = rc_legacy
    r.jointsolve_legacy = jsv_legacy
    if rmp_rows:
        r.rommap = np.array(rmp_rows, dtype=ROMMAPDBG)
    if angc_rows:
        r.anglechain = np.array(angc_rows, dtype=ANGLECHAIN)
    return r


def calib(r):
    for _, j in r.param_deltas:
        if isinstance(j, dict) and "cameras" in j:
            return j["cameras"]
    return []


def flag_dict(bits):
    """把 RunFlagsRec.bits 展开成可读的字典。"""
    return {n: bool(int(bits) & (1 << i)) for i, n in enumerate(RUNFLAG_NAMES)}


def report(path_or_rec):
    """一条命令把整场录制讲完：时间轴 -> 动作 -> 标定 -> 输出 -> 块健康度。

    【为什么要有这个】排查的第一步永远是"先看看这一场发生了什么"，而这一步
    以前要分别调七八个函数再手工对齐时间 —— 对齐时间正是最容易出错、
    且错了不会报错的一步。顺序是按【因果】排的，不是按块号：
    先看操作（人做了什么），再看动作（手做了什么），再看标定（系统学到了
    什么），最后才看输出（结果对不对）。倒过来看必然会先怀疑输出。
    """
    r = load(path_or_rec) if isinstance(path_or_rec, str) else path_or_rec
    L = []
    def sec(title, lines):
        L.append("")
        L.append("=" * 72)
        L.append(f"  {title}")
        L.append("=" * 72)
        L.extend(lines if isinstance(lines, list) else [lines])
    sec("概览", r.summary().splitlines())
    sec("① 时间轴：你做了什么", r.timeline())
    sec("② 面板改动汇总", r.ui_changes())
    sec("③ 握拳次数（从 curl 数，不依赖关节角）", r.fist_cycles())
    sec("④ ROM 为什么标不满", r.why_rom_short())
    sec("⑤ 输出冻结", r.why_frozen())
    sec("⑥ 长期触限", r.why_clamped())
    sec("⑦ 下游收不到数据？", r.why_output_missing())
    sec("⑧ 块健康度（这一块为什么不在文件里）", r.block_report())
    if r.schema_errors:
        sec("⚠ schema 错误（有块被跳过了，结论可能不全）", r.schema_errors)
    return "\n".join(L)


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[2] == "--report":
        print(report(sys.argv[1]))
    elif len(sys.argv) > 2 and sys.argv[2] in ("--timeline", "--chain", "--frozen"):
        rr = load(sys.argv[1])
        if sys.argv[2] == "--timeline":
            print("\n".join(rr.timeline()))
        elif sys.argv[2] == "--frozen":
            print("\n".join(rr.why_frozen()))
        else:
            print("\n".join(rr.chain(int(sys.argv[3]) if len(sys.argv) > 3 else 6)))
    else:
        print(load(sys.argv[1]).summary())
        print("\n（想看全场报告：python pcrec.py 文件.pcrec --report）")
