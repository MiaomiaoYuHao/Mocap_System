#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rom_report.py —— ROM 标定 / 关节角输出链路的定位报告

    python3 tools/pcrec/rom_report.py recordings/xxx/capture.pcrec
    python3 tools/pcrec/rom_report.py 标定段.pcrec 标定后段.pcrec

【这个脚本回答什么】
按下 ROM 标定到点击结束之间，每一维的样本是怎么攒起来的、被什么门挡掉的、
方向是怎么判出来的、行程里有多少是外推补的。以及标定完之后，输出的角
到底是「解算就没解出来」还是「解出来了被哪一级压平/弄反了」。

【读的顺序，别跳】
  1. 骨轴退化率      —— 它 ≈100% 的维，PIP 恒等于 0，后面所有分析都不用看了
  2. ROM 事件时间轴  —— 确认标定区间找对了
  3. 逐维标定明细    —— 每一维为什么是这个覆盖度
  4. 四级流水        —— 标定后段用，看信号在哪一级消失
"""
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcrec import (load, JOINT_NAMES, FLEX_IDX, ABD_IDX, EVENT_NAME,
                   DOF_STATE_NAME, ROM_REJECT_NAME, ROM_STATUS_NAME,
                   PROX_AXIS_NAME, ROM_REASON_NAME)

R2D = 57.29577951308232
FINGERS = ["拇", "食", "中", "无", "小"]


def hr(t=""):
    print("\n" + "=" * 78)
    if t:
        print(t)
        print("=" * 78)


def sec1_axis(r):
    """近节骨轴退化 —— 一切的源头，先看这个。"""
    hr("① 近节骨轴：算 PIP 的两条轴是不是同一根骨头")
    js = r.jointsolve
    if js is None or not len(js):
        print("  没有 JointSolve 块（需要 v7 录制、且详细度 >= Full）。")
        print("  没有它就查不了这一节 —— 而「标不满/方向反了」十有八九就断在这里。")
        return None
    print("  近节骨轴有两条来源（见 computeSegmentQuats）：")
    print("    anchor->pp球   正确    需要 f!=0 且 anchorsValid 且 wristPoseValid")
    print("    pp球->mp球     退化    否则")
    print("  而算 PIP 的第二条轴恒等于 pp球->mp球。退化时两条轴重合，")
    print("  PIP = acos(1) = 0 —— 不是不准，是结构性的零。\n")
    print("  指   退化率   dot(a0,a1)中位   PIP恒零占比   wristValid   anchorsValid")
    for f in range(5):
        degen = (js["proxAxisSrc"][:, f] == 1)
        dot = js["dotProxMid"][:, f]
        pipz = np.abs(js["pipRaw"][:, f]) < np.deg2rad(1.0)
        print(f"  {FINGERS[f]}   {degen.mean()*100:5.1f}%   {np.median(dot):+12.4f}   "
              f"{pipz.mean()*100:9.1f}%   {js['wristPoseValid'].mean()*100:8.1f}%   "
              f"{js['anchorsValid'].mean()*100:8.1f}%")
    print()
    if (js["proxAxisSrc"][:, 0] == 1).mean() > 0.99:
        print("  【拇指 100% 退化】这是结构性的：computeSegmentQuats 里的条件是")
        print("   `f != 0 && ...`，拇指永远走退化分支。也就是说【拇MCP 从来没工作过】，")
        print("   它的值恒为 0，ROM 无论怎么标都救不了 —— 得改上游取轴。")
    four = np.array([(js["proxAxisSrc"][:, f] == 1).mean() for f in range(1, 5)])
    if four.max() > 0.25:
        print(f"  【四指退化率最高 {four.max()*100:.0f}%】退化只跟 wristPoseValid/anchorsValid")
        print("   有关，跟手指动作幅度无关。所以「再标一次、握紧一点」改善不了它。")
        if js["anchorsValid"].mean() < 0.5:
            print("   → 主因是 anchorsValid 为假：anchor 自检没过。查 ChainDbg 的")
            print("     anchorsVerifyNSeen/NOk，那是个一次性锁，判过一次不再重试。")
        else:
            print("   → 主因是 wristPoseValid 为假：手背被四指挡住。这是握拳时的常态，")
            print("     只能靠 ROM 的 curl 外推补，或者改上游用保持的腕部位姿。")
    return js


def sec2_events(r):
    """ROM 事件时间轴。"""
    hr("② ROM 事件时间轴")
    ev = [(e, t) for (e, t) in r.events if int(e["code"]) in (11, 16, 17, 18, 19)]
    if not ev:
        print("  文件里没有 ROM 事件。")
        print("  · 如果这段素材本来就没做标定，正常。")
        print("  · 如果做了，说明录制版本 < v7 —— 按钮的两个瞬间当时还没被记录。")
        return None
    t0 = min(int(e["wallNs"]) for e, _ in ev)
    print("   相对秒   码  名称                文本")
    for e, t in ev:
        rel = (int(e["wallNs"]) - t0) / 1e9
        print(f"  {rel:8.2f}  {int(e['code']):3d}  {EVENT_NAME.get(int(e['code']),'?'):<16s}  {t}")
    b16 = [int(e["wallNs"]) for e, _ in ev if int(e["code"]) == 16]
    f18 = [int(e["wallNs"]) for e, _ in ev if int(e["code"]) == 18]
    if b16 and f18:
        d = (f18[0] - b16[0]) / 1e6
        print(f"\n  按钮按下 → worker 真正开始采样：{d:.1f} ms")
        if d > 200:
            print("  【这个延迟偏大】worker 在积压。积压期间的帧是被丢掉的（设计如此），")
            print("   也就是说标定动作的头几帧根本没进采样。看 Timing 块的 skelSkipped。")
    return ev


def sec3_rom(r):
    """逐维标定明细。"""
    hr("③ ROM 标定逐维明细")
    rc = r.romcalib
    if rc is None or not len(rc):
        print("  没有 RomCalib 块。要么录制版本 < v7，要么这段没做标定。")
        return None
    fin = rc[rc["reason"] == 2]
    rec = fin[-1] if len(fin) else rc[-1]
    print(f"  取第 {len(rc)} 份快照（{ROM_REASON_NAME.get(int(rec['reason']),'?')}）"
          f"，补标={bool(rec['accumulated'])}")
    print(f"  屈曲覆盖 {rec['coverageFlex']*100:.1f}%   外展覆盖 {rec['coverageAbd']*100:.1f}%")
    print(f"  达标 {rec['nOkFlex']}/11   兜底 {rec['nPriorFlex']}   失效 {rec['nDeadFlex']}"
          f"   方向纠正 {rec['nSignFlipped']}")
    print(f"  采样 {rec['nFrames']} 帧（有效 {rec['nFramesUsed']}），历时 {rec['durationSec']:.1f} s")
    print(f"  ready = {bool(rec['ready'])}\n")

    print("  维         覆盖  样本/见到  方向 判据(°)  张开→握拳(°)   实测区间(°)"
          "     外推(°)  状态")
    for i in range(16):
        tag = "*" if i in ABD_IDX else " "
        n, seen = int(rec["nSamples"][i]), int(rec["nSeen"][i])
        sgn = int(rec["sign"][i])
        sres = "" if rec["signResolved"][i] else "?"
        print(f" {tag}{JOINT_NAMES[i]:<8s} {rec['coverage'][i]*100:5.0f}% "
              f"{n:5d}/{seen:5d} {sgn:+2d}{sres:1s} "
              f"{rec['signDeltaRad'][i]*R2D:+7.1f}  "
              f"{rec['openMedRad'][i]*R2D:+6.1f}→{rec['closeMedRad'][i]*R2D:+6.1f}  "
              f"[{rec['measLo'][i]*R2D:+6.1f},{rec['measHi'][i]*R2D:+6.1f}] "
              f"{rec['extrapLoRad'][i]*R2D:+5.1f}/{rec['extrapHiRad'][i]*R2D:+5.1f}  "
              f"{ROM_STATUS_NAME.get(int(rec['status'][i]),'?')}")
    print("  （* = 外展维，不参与达标计数；方向列的 ? = 没判出来，按 +1 处理）")

    # ---- 拒收原因 ----
    print("\n  逐维拒收原因（占见到帧数的比例）：")
    print("  维         接受  不新鲜 骨轴退化 curl无效  野点  只有预测")
    for i in range(16):
        seen = max(1, int(rec["nSeen"][i]))
        row = rec["nReject"][i]
        print(f"   {JOINT_NAMES[i]:<8s} " + " ".join(
            f"{row[c]/seen*100:6.1f}%" for c in range(6)))

    # ---- 结论 ----
    print()
    flipped = [i for i in range(16) if int(rec["sign"][i]) < 0]
    if flipped:
        print(f"  【方向反的维】{', '.join(JOINT_NAMES[i] for i in flipped)}")
        print("   这些维握拳时读数【变小】。v2 已经自动纠正（映射时乘 sign），")
        print("   但根因仍在上游 —— 值得顺着 JointSolve 的 hingeSigned 查一下。")
    dead = [i for i in range(16) if int(rec["status"][i]) == 4]
    if dead:
        print(f"  【骨轴退化致死的维】{', '.join(JOINT_NAMES[i] for i in dead)}")
        print("   这几维输出被冻结在解剖中立位。重标一万次也没用，是上游取轴的问题。")
    extrap = [i for i in range(16) if int(rec["status"][i]) == 5]
    if extrap:
        print(f"  【行程含外推的维】{', '.join(JOINT_NAMES[i] for i in extrap)}")
        print("   斜率由几百个真实样本拟合，只有末端一小段是推的。")
        print("   如果这些维在实际遥操作里手感「到不了底」，把 extrapMaxFrac 调大。")
    noopen = [i for i in FLEX_IDX if rec["curlSpread"][i] < 0.15]
    if noopen:
        print(f"  【curl 没拉开的指】{', '.join(JOINT_NAMES[i] for i in noopen)}")
        print("   说明这几根手指在标定期间【根本没做完整的张开→握拳】，")
        print("   方向判不出来、区间也不可信。这一条是真的要重做动作。")
    return rec


def sec4_chain(r, learning_only=False):
    """关节角四级流水 —— 标定后段用。"""
    hr("④ 关节角四级流水：信号在哪一级消失")
    jo = r.jointout
    if jo is None or not len(jo):
        print("  没有 JointOut 块。")
        return
    m = np.ones(len(jo), bool)
    if learning_only:
        m = jo["romLearning"] == 0
        if m.sum() < 30:
            m = np.ones(len(jo), bool)
    jo = jo[m]
    print(f"  统计 {len(jo)} 帧（romReady={jo['romReady'].mean()*100:.0f}%"
          f"，限幅开={jo['rateLimitOn'].mean()*100:.0f}%）\n")
    print("  维         qSolve动幅  qSmooth  qRom   qOut   触限率  结论")
    for i in range(16):
        rng = [np.percentile(jo[k][:, i], 98) - np.percentile(jo[k][:, i], 2)
               for k in ("qSolve", "qSmooth", "qRom", "qOut")]
        lo, hi = jo["romLo"][:, i], jo["romHi"][:, i]
        at = np.mean((np.abs(jo["qOut"][:, i] - np.percentile(jo["qOut"][:, i], 2)) < 1e-4) |
                     (np.abs(jo["qOut"][:, i] - np.percentile(jo["qOut"][:, i], 98)) < 1e-4))
        # 哪一级把行程吃掉了
        note = ""
        if rng[0] < np.deg2rad(3):
            note = "解算就没信号（查骨轴/标签/手性）"
        elif rng[1] < rng[0] * 0.5:
            note = "平滑吃掉了信号"
        elif rng[2] < np.deg2rad(3):
            note = "ROM 把行程压平了（查区间分母）"
        elif rng[3] < rng[2] * 0.5:
            note = "速度限幅削得太狠"
        # 反向
        if len(jo) > 60:
            with np.errstate(invalid="ignore", divide="ignore"):
                c = np.corrcoef(jo["qSolve"][:, i], jo["qOut"][:, i])[0, 1]
            if np.isfinite(c) and c < -0.3:
                note = (note + " / " if note else "") + "输出与解算反号"
        print(f"   {JOINT_NAMES[i]:<8s} " + " ".join(f"{v*R2D:7.1f}" for v in rng)
              + f"  {at*100:5.1f}%  {note}")
    print("\n  （动幅 = p98-p2，单位 度。四级依次是 解算→平滑→ROM映射→速度限幅）")

    clip = jo["maxRateClipRad"]
    if np.median(clip) > 1e-4:
        print(f"  【速度限幅在持续削信号】中位 {np.median(clip)*R2D:.2f}°/帧 —— 那就是「跟不上手」。")
    if r.jointsolve is not None and len(r.jointsolve):
        js = r.jointsolve
        n = min(len(js), len(jo))
        held = (js["dofState"][:n] == 1).mean(axis=0)
        worst = np.argsort(-held)[:4]
        print("  逐维「保持上一帧」占比最高的四维：" + "  ".join(
            f"{JOINT_NAMES[i]} {held[i]*100:.0f}%" for i in worst))


def sec5_state(r):
    """标定期间逐维状态分布 —— 解释「为什么样本这么少」。"""
    js = r.jointsolve
    if js is None or not len(js):
        return
    learn = js["romLearning"] == 1
    if learn.sum() < 30:
        return
    hr("⑤ 标定期间逐维状态分布（只统计 romLearning=1 的帧）")
    sub = js[learn]
    print(f"  标定期间共 {len(sub)} 帧\n")
    print("  维          没算  保持上帧 骨轴退化  预测段   实测")
    for i in range(16):
        d = sub["dofState"][:, i]
        print(f"   {JOINT_NAMES[i]:<8s} " + " ".join(
            f"{(d == k).mean()*100:7.1f}%" for k in range(5)))
    print("\n  【怎么读】「保持上帧」高 = 那一维在标定时大量采到的是旧值，")
    print("  行程会被削掉一截（v1 正是照单全收的）；「骨轴退化」高 = 结构问题，")
    print("  跟动作幅度无关。curl 中位/跨度：")
    for f in range(5):
        cv = sub["curl"][:, f][sub["curlValid"][:, f] == 1]
        if len(cv) < 10:
            print(f"   {FINGERS[f]}   curl 无效")
            continue
        p10, p90 = np.percentile(cv, [10, 90])
        flag = "  ← 没做完整开合" if (p90 - p10) < 0.15 else ""
        print(f"   {FINGERS[f]}   中位 {np.median(cv):.2f}   p10-p90 {p10:.2f}~{p90:.2f}"
              f"（跨度 {p90-p10:.2f}）{flag}")


def report(path):
    hr(f"文件 {path}")
    r = load(path)
    print(r.summary())
    js = sec1_axis(r)
    sec2_events(r)
    rec = sec3_rom(r)
    sec5_state(r)
    sec4_chain(r, learning_only=(rec is not None))
    return r


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for p in sys.argv[1:]:
        report(p)
    print()
