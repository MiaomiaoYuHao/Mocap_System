#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pcrec_diagnose.py —— 从一段 v4 录制里定位"输出不对"到底出在哪一级

用法：
    python3 tools/hand_diag/pcrec_diagnose.py 你的文件.pcrec
    python3 tools/hand_diag/pcrec_diagnose.py 你的文件.pcrec --section hand
    python3 tools/hand_diag/pcrec_diagnose.py 你的文件.pcrec --dump-frames 100 140

=============================================================================
【这个脚本的核心思路：先立真值，再逐级对拍】
=============================================================================
"我看着输出不对"没法验证，也没法反驳。要把它变成能查的东西，必须先有一个
【不依赖被怀疑的那条链路】的真值。这里用的是：

    指尖球到腕部原点的距离

它只由三角化的 3D 点算出来，完全不经过分段四元数、关节角、ROM、限幅。
握拳时它必然显著变小，张开时变大 —— 这是几何事实，不是算法结论。

有了它，"握拳时输出像张开"就从主观描述变成了一个可判定的命题：
    corr(指尖距离, 输出屈曲角) 应该是【强负相关】
    如果是正相关  -> 某一级把符号弄反了
    如果接近 0    -> 某一级把信号压平了
再逐级看 qSolve/qSmooth/qRom/qOut 各自的相关性和行程，就知道是哪一级。

【只用实测帧算真值】指尖被遮挡时它的位置是补出来的，拿补出来的点当真值
等于用被怀疑的链路验证它自己。所以真值只在四指指尖都是实测的帧上算。
=============================================================================
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "pcrec"))
import pcrec  # noqa: E402

R2D = 180.0 / np.pi


# ---------------------------------------------------------------------------
def _hr(title):
    return "\n" + "=" * 74 + f"\n{title}\n" + "=" * 74


def _sub(title):
    return f"\n-- {title} " + "-" * max(0, 68 - len(title))


def _pct(a):
    return 100.0 * float(np.mean(a)) if len(a) else float("nan")


# ---------------------------------------------------------------------------
# 手性
# ---------------------------------------------------------------------------
def diagnose_handedness(r, out):
    out.append(_hr("一、手性"))
    if r.handed is None or len(r.handed) == 0:
        out.append("没有 Handedness 块。v3 文件或者根本没跑骨架 —— 手性问题查不了，请用 v4 重录。")
        return
    h = r.handed
    n = len(h)

    # ---- 六处环节各自的手性，逐帧比对 ----
    # 【这是整个手性排查的核心表】不是看某一个 bool 对不对，而是看这六个
    # 从哪一处开始不一致 —— 不一致的那个接缝就是 bug 所在。
    srcs = [("① 面板勾选     panelIsRight", h["panelIsRight"]),
            ("② 自标定推断   autoDetected", h["autoIsRightDetected"]),
            ("③ 自标定采用   autoIsRight ", h["autoIsRight"]),
            ("④ 模型判定     aiIsRight   ", h["aiIsRight"]),
            ("⑤ 送进网络     tmplIsRight ", h["tmplIsRight"]),
            ("⑥ IK 使用      ikIsRight   ", h["ikIsRight"])]
    out.append(_sub("六处环节各自认为的手性（右手占比）"))
    for name, v in srcs:
        maj = "右" if np.mean(v) >= 0.5 else "左"
        flip = int(np.sum(np.abs(np.diff(v.astype(int))) > 0))
        out.append(f"  {name}  右手 {_pct(v):5.1f}%   多数={maj}   中途翻转 {flip} 次")

    out.append(_sub("关键一致性检查"))
    panel = h["panelIsRight"].astype(bool)
    tmpl = h["tmplIsRight"].astype(bool)
    ik = h["ikIsRight"].astype(bool)
    ai_known = h["aiKnown"].astype(bool)

    # 【最要命的一处】送进网络的手性 != 面板值。
    # 它是 v3 完全没记的量，而它错了的话：网络的输入就是镜像的，
    # 后面全错，但 v3 记的那四个 bool 可以全部"正常"。
    bad_tmpl = panel != tmpl
    if np.any(bad_tmpl):
        out.append(f"  ★★★ tmplIsRight 与面板不一致：{_pct(bad_tmpl):.1f}% 的帧")
        out.append("      这是【送进网络和用来建模板】的那个手性。它跟面板不一致，")
        out.append("      意味着网络看到的是镜像的条件输入 —— 标签、手背位姿、")
        out.append("      整条链都会跟着错，而面板上的勾选看起来完全正常。")
        out.append("      去查三条改写路径：handDirty_ / applyAutoCalib / dorsumReset。")
    else:
        out.append("  ✓ tmplIsRight 与面板一致（送进网络的手性没有被偷偷改掉）")

    bad_ik = tmpl != ik
    if np.any(bad_ik):
        out.append(f"  ★★ ikIsRight 与 tmplIsRight 不一致：{_pct(bad_ik):.1f}% 的帧")
        out.append("      症状是【拇指单独歪、其它手指正常】：IK 按一只手解、")
        out.append("      拇指旋前按另一只手补。tmpl_.isRight 有三条改写路径，")
        out.append("      只有两条会同步下发给 IK —— 漏的那条就是这里。")
    else:
        out.append("  ✓ ikIsRight 与 tmplIsRight 一致")

    # ---- 判据本身的强度，而不是结论 ----
    # 【为什么要看这个】autoIsRightDetected 是个 bool，由判据跟 4mm 阈值比出来。
    # 判据 4.1mm 和 40mm 得到同一个 bool，可信度天差地别。
    out.append(_sub("判据强度（结论可信度的来源）"))
    thr = float(np.median(h["handednessMinMm"]))
    sgn = h["handSignMm"].astype(float)
    lat = h["handSignLatMm"].astype(float)
    out.append(f"  阈值 handednessMinMm = {thr:.2f} mm")
    out.append(f"  拇指判据 handSignMm    中位 {np.median(sgn):+8.2f} mm   "
               f"|值|<阈值的帧 {_pct(np.abs(sgn) < thr):5.1f}%")
    out.append(f"  横向判据 handSignLatMm 中位 {np.median(lat):+8.2f} mm")
    marg = np.abs(sgn) - thr
    weak = np.mean((marg > -1.0) & (marg < 1.0))
    if weak > 0.2:
        out.append(f"  ★ 有 {100*weak:.1f}% 的帧判据就贴在阈值上下 1mm 内 —— ")
        out.append("    这个区间里符号是噪声决定的，结论会来回翻。")
        out.append("    多半是手摊平了：手摊平时手指点几乎就在手背平面上，判据接近 0。")
        out.append("    用「手性排查」协议重录（掌朝下/朝上/侧立各做明显屈伸）。")

    agree = h["autoAgree"].astype(bool)
    if np.mean(~agree) > 0.05:
        out.append(f"  ★ 两个判据不一致的帧占 {_pct(~agree):.1f}%")
        out.append("    拇指判据和横向判据打架 = 拇指标签多半歪了（拇指是全手最")
        out.append("    容易丢和误配的球）。此时 handednessKnown 会被压成 false，")
        out.append("    自标定不下结论、沿用面板值 —— 这是【设计如此】，不是 bug。")

    # ---- 几何锁 和 模型 ----
    out.append(_sub("几何锁 / 模型判定"))
    geo = h["geoHandSign"].astype(int)
    locked = geo != 0
    if np.any(locked):
        gv = geo[locked]
        gr = float(np.mean(gv > 0))
        out.append(f"  几何手性已锁定：{_pct(locked):.1f}% 的帧，锁定值 右手占 {100*gr:.1f}%")
        geo_right = gr >= 0.5
        if np.any(locked) and geo_right != bool(np.mean(panel) >= 0.5):
            out.append("  ★★★ 几何锁定的手性与面板【相反】。")
            out.append("      几何判据用的是'手指屈曲必然朝掌侧'这个物理事实，")
            out.append("      它是本系统里唯一有外部参照的手性判据 —— 任何只用手上的点")
            out.append("      算法向的判据在数学上都区分不了左右手（镜像下两个负号抵消）。")
            out.append("      所以这一条冲突时，【优先怀疑面板勾错了】，而不是几何错了。")
    else:
        out.append("  几何手性从未锁定 —— 说明整段里手指弯折都不够充分。")
        out.append("  锁定需要弯折角超过门限的帧累积；握拳时中远节球被手掌挡住，")
        out.append("  剩下看得见的那段恰恰弯折最小，正是这种情况。")

    if np.any(ai_known):
        ar = h["aiIsRight"][ai_known]
        out.append(f"  模型判定：{_pct(ai_known):.1f}% 的帧敢下结论，其中右手 {_pct(ar):.1f}%")
    else:
        out.append("  模型从未给出可信的手性判定（aiKnown 全 false）。")
        out.append("  【这是正常的，不是故障】实测 hm20_v7 的 |hand_logit| 中位只有 0.044，")
        out.append("  而纯随机点云能到 0.196 —— 在这个量级上符号跟噪声分不开。")
    lg = np.abs(h["aiHandLogit"].astype(float))
    out.append(f"  |hand_logit| 中位 {np.median(lg):.4f}"
               f"（<0.2 = 模型没有意见，别拿它当依据）")

    # ---- 配置：推断会不会真的改东西 ----
    out.append(_sub("配置"))
    ap = bool(np.median(h["cfgApplyHandedness"]))
    out.append(f"  cfgApplyHandedness = {ap}"
               + ("   ★ 打开着：一次误判就会把整个手背模板镜像掉，"
                  "而 Kabsch 残差自检查不出来" if ap else "   （只上报不改，安全）"))
    out.append(f"  cfgDetectHandedness = {bool(np.median(h['cfgDetectHandedness']))}")
    out.append(f"  cfgJointMirrorAuto  = {bool(np.median(h['cfgJointMirrorAuto']))}")
    jm = h["jointMirrorActive"].astype(bool)
    out.append(f"  关节角输出被镜像的帧：{_pct(jm):.1f}%")
    if 0.05 < np.mean(jm) < 0.95:
        out.append("  ★ 镜像状态在录制中途变过 —— 那一刻输出的屈曲类各维会整体反号。")
        out.append("    下游看到的是手指瞬间反向，很像'跳变'，实际是坐标约定切换。")

    if np.any(h["autoMirrorSuspect"].astype(bool)):
        out.append(f"  ★★ autoMirrorSuspect 触发 {_pct(h['autoMirrorSuspect'].astype(bool)):.1f}% "
                   "—— 自标定自检怀疑冻结的模板被镜像了。")


# ---------------------------------------------------------------------------
# 真值：指尖到腕心的距离
# ---------------------------------------------------------------------------
def grip_signal(r):
    """返回 (ts, grip, trusted)。grip 越小 = 握得越紧。

    【只用四指，不含拇指】拇指指尖到腕心的距离在张开和握拳时变化远小于四指，
    而且拇指是最容易误配的球，混进来只会稀释信号。
    trusted = 这一帧四个指尖是否【全部实测】。真值只在 trusted 帧上有意义 ——
    用补出来的点当真值，等于拿被怀疑的链路验证它自己。
    """
    if r.markerdbg is None or len(r.markerdbg) == 0:
        return None, None, None
    md = r.markerdbg
    pos = md["posFinal"].reshape(len(md), 20, 3).astype(float)
    src = md["source"].astype(int)
    # 腕心：用手背五点的质心，而不是 wristT —— wristT 来自刚体解，
    # 解不出来时会沿用上一帧，那时它不是本帧的量。手背五点的质心永远是本帧的。
    wrist = pos[:, 0:5, :].mean(axis=1)
    tips = [10, 13, 16, 19]                      # 食/中/无/小 的远节球
    d = np.linalg.norm(pos[:, tips, :] - wrist[:, None, :], axis=2)
    grip = d.mean(axis=1)
    trusted = np.all(src[:, tips] == 1, axis=1)
    return md["frameTsNs"].astype(np.int64), grip, trusted


def _corr(a, b):
    if len(a) < 8:
        return float("nan")
    sa, sb = np.std(a), np.std(b)
    if sa < 1e-9 or sb < 1e-9:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def _span(v):
    """p2..p98 行程。用分位而不是 min/max —— 一个野点就能把 min/max 废掉。"""
    if len(v) < 8:
        return 0.0
    return float(np.percentile(v, 98) - np.percentile(v, 2))


# ---------------------------------------------------------------------------
# 关节角输出链
# ---------------------------------------------------------------------------
def diagnose_joints(r, out):
    out.append(_hr("二、关节角 / 四元数输出链"))
    if r.jointout is None or len(r.jointout) == 0:
        out.append("没有 JointOut 块。v3 文件 —— 输出角的问题查不了，请用 v4 重录。")
        return
    jo = r.jointout
    n = len(jo)
    out.append(f"帧数 {n}")

    # ---- 各级的行程 ----
    # 【这一张表就能定位是哪一级把信号弄没的】
    stages = [("① 解算 qSolve ", jo["qSolve"]),
              ("② 平滑 qSmooth", jo["qSmooth"]),
              ("③ ROM  qRom   ", jo["qRom"]),
              ("④ 限幅 qOut   ", jo["qOut"])]
    out.append(_sub("四级流水各自的行程（屈曲维，度；p2..p98）"))
    out.append("  " + " " * 16 + "".join(f"{pcrec.JOINT_NAMES[i]:>9s}" for i in pcrec.FLEX_IDX[:6]))
    spans = {}
    for name, q in stages:
        q = q.astype(float)
        sp = [_span(q[:, i]) * R2D for i in pcrec.FLEX_IDX]
        spans[name] = np.array(sp)
        out.append(f"  {name}  " + "".join(f"{v:9.1f}" for v in sp[:6]))
    out.append("  （只显示前 6 维，完整判断看下面的结论）")

    out.append(_sub("哪一级把信号压掉了"))
    prev_name, prev = None, None
    for name, _ in stages:
        cur = spans[name]
        if prev is not None:
            # 逐维比：某一维在这一级掉了 60% 以上就点名
            with np.errstate(divide="ignore", invalid="ignore"):
                ratio = np.where(prev > 1e-6, cur / prev, 1.0)
            killed = [pcrec.JOINT_NAMES[pcrec.FLEX_IDX[k]]
                      for k in range(len(ratio)) if ratio[k] < 0.4 and prev[k] > 5.0]
            if killed:
                out.append(f"  ★★ {prev_name} -> {name}：{', '.join(killed)} 行程掉了 60% 以上")
        prev_name, prev = name, cur
    tot = spans["① 解算 qSolve "]
    fin = spans["④ 限幅 qOut   "]
    dead = [pcrec.JOINT_NAMES[pcrec.FLEX_IDX[k]] for k in range(len(fin)) if fin[k] < 3.0]
    if dead:
        out.append(f"  ★★ 输出行程小于 3° 的维（等于没有信号）：{', '.join(dead)}")
    if float(np.median(tot)) < 5.0:
        out.append("  ★★★ 解算这一级行程就已经很小了 —— 问题不在 ROM/限幅，")
        out.append("      在更上游：分段四元数、标签、手性、或者手根本没怎么动。")

    # ---- ROM ----
    out.append(_sub("ROM 标定状态（映射的分母）"))
    ready = bool(np.median(jo["romReady"]))
    out.append(f"  romReady = {ready}   样本 {int(np.median(jo['romSamples']))}"
               f"   屈曲覆盖度 {float(np.median(jo['romCoverage'])):.2f}"
               f"   外展覆盖度 {float(np.median(jo['romAbdCoverage'])):.2f}")
    if not ready:
        out.append("  ROM 未标定 -> RomMapper 走的是【直接钳位透传】那条路。")
        out.append("  钳位用的是解剖限位，而我们的 MCP 角带一个未知的常量零位偏置，")
        out.append("  偏置一大就整体撞限位、被压成一条直线 —— 这正是'输出恒定不动'。")
    else:
        lo = jo["romLo"].astype(float)[-1]
        hi = jo["romHi"].astype(float)[-1]
        narrow = [(pcrec.JOINT_NAMES[i], (hi[i] - lo[i]) * R2D)
                  for i in range(16) if (hi[i] - lo[i]) < 0.05]
        if narrow:
            out.append("  ★★ 下面这些维的 ROM 区间接近 0（映射分母塌了，输出会恒等于行程端点）：")
            for nm, sp in narrow:
                out.append(f"       {nm}  区间 {sp:.2f}°")
            out.append("     成因通常是标定时那一维根本没动到。重做 ROM 标定。")

    # ---- 限幅 ----
    out.append(_sub("速度限幅"))
    on = bool(np.median(jo["rateLimitOn"]))
    clip = jo["maxRateClipRad"].astype(float) * R2D
    out.append(f"  rateLimitOn = {on}   上限 {float(np.median(jo['rateLimitRadPerSec'])):.1f} rad/s")
    out.append(f"  每帧被削掉的最大量：中位 {np.median(clip):.2f}°   p95 {np.percentile(clip, 95):.2f}°"
               f"   非零帧占 {_pct(clip > 0.01):.1f}%")
    if np.median(clip) > 0.5:
        out.append("  ★★ 限幅在【持续】削信号（中位就大于 0.5°/帧）。这正是'跟不上手'。")
        out.append("     要么调高上限，要么上游有跳变在反复触发它。")

    # ---- 平滑 ----
    out.append(_sub("预测段角度平滑"))
    out.append(f"  angSmoothOn = {bool(np.median(jo['angSmoothOn']))}"
               f"   alpha = {float(np.median(jo['angSmoothAlpha'])):.2f}"
               f"（1.0=关闭，越小越平滑也越滞后）")
    fp = jo["fingerPredicted"].astype(bool)
    out.append("  各指走预测段的帧占比：" +
               "  ".join(f"{'拇食中无小'[f]}{_pct(fp[:, f]):.0f}%" for f in range(5)))

    # ---- 有效性 ----
    out.append(_sub("角度有效性"))
    fv = jo["fingerValid"].astype(bool)
    out.append("  各指角度本帧新算（非保持上一帧）的占比：" +
               "  ".join(f"{'拇食中无小'[f]}{_pct(fv[:, f]):.0f}%" for f in range(5)))
    out.append(f"  mcpValid {_pct(jo['mcpValid'].astype(bool)):.1f}%"
               f"   wristValid {_pct(jo['wristValid'].astype(bool)):.1f}%"
               f"   腕部沿用帧数中位 {int(np.median(jo['wristStale']))}")
    low = [f for f in range(5) if np.mean(fv[:, f]) < 0.5]
    if low:
        out.append(f"  ★★ 这些指有一半以上的帧角度是【保持上一帧】的："
                   f"{', '.join('拇食中无小'[f] for f in low)}")
        out.append("     表现就是'手指弯下去机械手卡住不动'。多半是那几指的球被挡了。")


# ---------------------------------------------------------------------------
# 真值对拍：这是回答"握拳输出像张开"的地方
# ---------------------------------------------------------------------------
def diagnose_truth(r, out):
    out.append(_hr("三、真值对拍：输出的屈曲角跟真实握拳程度对得上吗"))
    ts_g, grip, trusted = grip_signal(r)
    if grip is None:
        out.append("没有 MarkerDbg 块，立不了真值。请用 v4 重录。")
        return
    if r.jointout is None or len(r.jointout) == 0:
        out.append("没有 JointOut 块。")
        return
    jo = r.jointout
    ts_j = jo["frameTsNs"].astype(np.int64)

    # 按时间戳对齐两个块。它们在同一次 onSkeletonResultReady 里写的，
    # 时间戳应当逐帧相同 —— 不同则说明有块被丢了（队列满），如实报告。
    common, ig, ij = np.intersect1d(ts_g, ts_j, return_indices=True)
    if len(common) < 20:
        out.append(f"能对齐的帧只有 {len(common)} 帧，太少，无法判断。")
        out.append("（两个块是同一次回调里写的，对不齐说明录制时有丢块，看 trailer。）")
        return
    g = grip[ig]
    tr = trusted[ig]
    out.append(f"对齐 {len(common)} 帧，其中四指指尖【全实测】的可信帧 "
               f"{int(tr.sum())} 帧（{100*tr.mean():.1f}%）")
    if tr.sum() < 20:
        out.append("★ 可信帧太少 —— 指尖长期被遮挡。这种数据里'真值'本身就是补出来的，")
        out.append("  任何结论都不可靠。请把手正对相机组重录一段「张开握拳循环」。")
        use = np.ones(len(g), dtype=bool)
        out.append("  下面退而用全部帧，结论仅供参考。")
    else:
        use = tr

    gg = g[use]
    out.append(f"指尖到腕心距离：中位 {np.median(gg):.1f} mm   "
               f"p2 {np.percentile(gg,2):.1f}   p98 {np.percentile(gg,98):.1f}   "
               f"行程 {_span(gg):.1f} mm")
    if _span(gg) < 15.0:
        out.append("★★ 这段录制里手【几乎没有真的张开和握紧】（行程 <15mm）。")
        out.append("   没有动作就没有信号，任何'输出不跟手'的结论都无从谈起。")
        out.append("   请按「张开握拳循环」协议重录：张到最开停3秒 -> 握到最紧停3秒，5轮。")
        return

    # ---- 逐级相关性 ----
    # 【符号约定】grip 小 = 握紧 = 屈曲角大，所以正确的相关系数应当是【负】的。
    out.append(_sub("各级屈曲角 vs 真值的相关性（应为强负相关，|ρ|>0.7）"))
    stages = [("① 解算 qSolve ", jo["qSolve"]),
              ("② 平滑 qSmooth", jo["qSmooth"]),
              ("③ ROM  qRom   ", jo["qRom"]),
              ("④ 限幅 qOut   ", jo["qOut"])]
    verdicts = {}
    for name, q in stages:
        qq = q.astype(float)[ij][use]
        flex = qq[:, pcrec.FLEX_IDX].mean(axis=1)
        rho = _corr(gg, flex)
        sp = _span(flex) * R2D
        if np.isnan(rho):
            tag = "无法判断（信号无变化）"
        elif rho < -0.7:
            tag = "✓ 正常"
        elif rho > 0.5:
            tag = "★★★ 符号反了"
        elif abs(rho) < 0.3:
            tag = "★★ 信号丢失/被压平"
        else:
            tag = "★ 相关性弱"
        verdicts[name] = (rho, sp, tag)
        out.append(f"  {name}   ρ = {rho:+.3f}   行程 {sp:6.1f}°   {tag}")

    # ---- 结论 ----
    out.append(_sub("结论"))
    order = [s[0] for s in stages]
    first_bad = None
    for name in order:
        rho, sp, _ = verdicts[name]
        if np.isnan(rho) or rho > -0.5 or sp < 3.0:
            first_bad = name
            break
    if first_bad is None:
        out.append("  四级全部与真值强负相关、行程充足 —— 输出的关节角是【对的】。")
        out.append("  如果下游（Unity/机械手）看着还是不对，问题在下游的坐标系约定")
        out.append("  或骨骼绑定，不在本系统的解算链里。看下面第四节的四元数约定。")
    elif first_bad == order[0]:
        rho = verdicts[order[0]][0]
        if not np.isnan(rho) and rho > 0.5:
            out.append("  ★★★ 【解算这一级符号就是反的】。")
            out.append("      问题在 solveJointAngles 之前或之内，不在 ROM/限幅。")
            out.append("      按可能性排序去查：")
            out.append("        1. 手性 —— 看第一节。jointMirror 会把屈曲类各维整体反号，")
            out.append("           而外展维不反。如果只有屈曲维反、外展维正常，几乎必然是它。")
            out.append("        2. PIP 的铰链方向 —— Hm20JointAngles.hpp 里 hinge=cross(ẑ,a0)")
            out.append("           那段。写反过一次（cross(a0,ẑ)），症状正是握拳时 PIP 为负、")
            out.append("           被 clamp 成 0，PIP 这一路永远没信号。")
            out.append("        3. 腕部系的 Z 轴朝向 —— 它决定 flex=atan2(-a[2],·) 的符号。")
        else:
            out.append("  ★★ 【解算这一级就没有信号】。")
            out.append("      不是 ROM/限幅的问题。往上游查：分段四元数是不是真的在动、")
            out.append("      标签是不是稳定、手指的球是不是长期被遮挡（看第二节的有效性）。")
    else:
        prev = order[order.index(first_bad) - 1]
        out.append(f"  ★★ 信号在 {prev.strip()} 之后、{first_bad.strip()} 这一级坏掉。")
        if "ROM" in first_bad:
            out.append("      -> ROM 映射。看第二节的 ROM 区间：多半是某几维 hi-lo 塌了，")
            out.append("         或者压根没标定（未标定时走钳位透传，会撞解剖限位）。")
            out.append("         重做 ROM 标定：五指张开到底 <-> 握拳到底，来回两三遍。")
        elif "限幅" in first_bad:
            out.append("      -> 速度限幅削得太狠。调高 maxRadPerSec，或先排除上游跳变。")
        elif "平滑" in first_bad:
            out.append("      -> 预测段角度平滑。alpha 太小（滞后）或恢复补偿在持续拉偏。")
            out.append("         注意它只作用于【预测段】—— 如果那几指长期走预测，")
            out.append("         说明球被挡了，根子还是遮挡，不是平滑参数。")

    # ---- 逐维细看：哪几维反了 ----
    # 【屈曲维反、外展维不反】是手性镜像的指纹，单独列出来
    qq = jo["qOut"].astype(float)[ij][use]
    out.append(_sub("逐维相关性（找手性镜像的指纹：屈曲维反号而外展维不反）"))
    fl_bad = []
    for i in pcrec.FLEX_IDX:
        rho = _corr(gg, qq[:, i])
        if not np.isnan(rho) and rho > 0.4:
            fl_bad.append(pcrec.JOINT_NAMES[i])
    ab_bad = []
    for i in pcrec.ABD_IDX:
        rho = _corr(gg, qq[:, i])
        if not np.isnan(rho) and abs(rho) > 0.6:
            ab_bad.append(f"{pcrec.JOINT_NAMES[i]}({rho:+.2f})")
    out.append(f"  屈曲维中与真值【正相关】(即反号)的：{', '.join(fl_bad) if fl_bad else '无'}")
    out.append(f"  外展维中相关性显著的：{', '.join(ab_bad) if ab_bad else '无'}")
    if len(fl_bad) >= 6 and not ab_bad:
        out.append("  ★★★ 大部分屈曲维反号、外展维不受影响 —— 这是【手性镜像】的典型指纹。")
        out.append("      镜像 N=diag(1,1,-1) 只反 z：flex=atan2(-a[2],·) 反号、")
        out.append("      abd=atan2(a[1],a[0]) 不变、PIP 反号。跟观察到的完全一致。")
        out.append("      去看第一节，重点是 tmplIsRight 和 jointMirrorActive。")


# ---------------------------------------------------------------------------
# 逐点来源 / 开关
# ---------------------------------------------------------------------------
def diagnose_markers(r, out):
    out.append(_hr("四、逐点来源与后处理搬动量"))
    if r.markerdbg is None or len(r.markerdbg) == 0:
        out.append("没有 MarkerDbg 块。")
        return
    md = r.markerdbg
    src = md["source"].astype(int)
    out.append(_sub("每个 marker 的位置来源占比"))
    names = ["背0", "背1", "背2", "背3", "背4",
             "拇MC", "拇PP", "拇DP",
             "食PP", "食MP", "食DP", "中PP", "中MP", "中DP",
             "无PP", "无MP", "无DP", "小PP", "小MP", "小DP"]
    hdr = "  marker  " + "".join(f"{pcrec.SRC_NAME[k]:>10s}" for k in range(1, 6))
    out.append(hdr)
    for m in range(20):
        row = "".join(f"{100.0*np.mean(src[:, m] == k):9.1f}%" for k in range(1, 6))
        flag = "  ★ 实测率低" if np.mean(src[:, m] == 1) < 0.4 else ""
        out.append(f"  {names[m]:6s}  {row}{flag}")

    if bool(np.median(md["hasNet"])):
        out.append(_sub("后处理把点搬了多远（|最终 - 网络原始预测|，mm）"))
        d = md["netDeltaMm"].astype(float)
        d = np.where(d < 0, np.nan, d)
        with np.errstate(invalid="ignore"):
            med = np.nanmedian(d, axis=0)
            p95 = np.nanpercentile(d, 95, axis=0)
        for m in range(20):
            if not np.isfinite(med[m]):
                continue
            tag = ""
            if med[m] > 15.0:
                tag = "  ★ 后处理搬动很大：模型给的和最终输出差很多"
            out.append(f"  {names[m]:6s}  中位 {med[m]:6.2f}   p95 {p95[m]:6.2f}{tag}")
        out.append("  【怎么用这张表】某点最终位置看着不对时：")
        out.append("    搬动量小 -> 模型/标签本来就给歪了，去查网络输入和指派")
        out.append("    搬动量大 -> 后处理（骨长回正/平滑/发散限速）拧的，去查那三步")
        out.append("  这两件事的修法完全相反。")
    else:
        out.append("\n  没有网络原始流（captureDebugStreams 没开）—— 分不开'模型错'和'后处理错'。")

    fl = md["flags"].astype(int)
    out.append(_sub("后处理动作触发率"))
    for bit, nm in [(pcrec.MF_BONESNAP, "骨长回正"), (pcrec.MF_JUMPLIMIT, "发散限速"),
                    (pcrec.MF_THUMBFIX, "拇指回正")]:
        hit = np.mean((fl & bit) != 0)
        out.append(f"  {nm}  触发帧·点占比 {100*hit:.2f}%")


def diagnose_flags(r, out):
    out.append(_hr("五、录制期间实际生效的开关"))
    if r.runflags is None or len(r.runflags) == 0:
        out.append("没有 RunFlags 块。")
        return
    rf = r.runflags
    bits = rf["bits"].astype(np.uint32)
    out.append(_sub("开关（打开帧占比；不是 0% 或 100% 就说明录制中途变过）"))
    for i, nm in enumerate(pcrec.RUNFLAG_NAMES):
        on = (bits & np.uint32(1 << i)) != 0
        p = 100.0 * np.mean(on)
        mark = "   ★ 中途变过" if 1.0 < p < 99.0 else ""
        out.append(f"  {nm:20s} {p:6.1f}%{mark}")
    out.append(_sub("延迟"))
    lat = rf["latencyMs"].astype(float)
    out.append(f"  process() 耗时  中位 {np.median(lat):.2f} ms   p95 {np.percentile(lat,95):.2f} ms")


def dump_frames(r, lo, hi, out):
    out.append(_hr(f"逐帧明细 [{lo}, {hi})"))
    jo, hd, md = r.jointout, r.handed, r.markerdbg
    n = len(jo) if jo is not None else 0
    for k in range(max(0, lo), min(n, hi)):
        q = jo["qOut"].astype(float)[k] * R2D
        s = jo["qSolve"].astype(float)[k] * R2D
        out.append(f"\n帧 {k}  ts={int(jo['frameTsNs'][k])}")
        out.append("  qSolve " + " ".join(f"{v:6.1f}" for v in s))
        out.append("  qOut   " + " ".join(f"{v:6.1f}" for v in q))
        if hd is not None and k < len(hd):
            h = hd[k]
            out.append(f"  手性 panel={int(h['panelIsRight'])} tmpl={int(h['tmplIsRight'])} "
                       f"ik={int(h['ikIsRight'])} auto={int(h['autoIsRight'])} "
                       f"geo={int(h['geoHandSign'])} mirror={int(h['jointMirrorActive'])} "
                       f"sign={float(h['handSignMm']):.2f}mm")
        if md is not None and k < len(md):
            src = md["source"].astype(int)[k]
            out.append("  来源   " + " ".join(str(v) for v in src)
                       + "   (1实测 2IK 3链式 4网络 5保持)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path")
    ap.add_argument("--section", default="all",
                    choices=["all", "hand", "joint", "truth", "marker", "flags"])
    ap.add_argument("--dump-frames", nargs=2, type=int, metavar=("LO", "HI"))
    a = ap.parse_args()

    r = pcrec.load(a.path)
    out = [r.summary()]
    if r.schema_errors:
        out.append("\n★★★ 结构体尺寸对不上，上面列出的块【没有被解析】。")
        out.append("    C++ 和 pcrec.py 有一边改了字段而另一边没跟上，两边一起更新。")
    if a.section in ("all", "hand"):
        diagnose_handedness(r, out)
    if a.section in ("all", "joint"):
        diagnose_joints(r, out)
    if a.section in ("all", "truth"):
        diagnose_truth(r, out)
    if a.section in ("all", "marker"):
        diagnose_markers(r, out)
    if a.section in ("all", "flags"):
        diagnose_flags(r, out)
    if a.dump_frames:
        dump_frames(r, a.dump_frames[0], a.dump_frames[1], out)
    print("\n".join(out))


if __name__ == "__main__":
    main()
