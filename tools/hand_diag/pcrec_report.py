#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pcrec_report.py —— 一条命令出诊断报告

    python pcrec_report.py recordings/xxx/capture.pcrec

设计意图
--------
调参循环慢的根本原因不是"数据不够"，是【每次都要重新想一遍该看什么】。
这个脚本把"该看什么"固化下来：每一节对应一个【已知的失效模式】，
给出一个可判定的结论，而不是把数字倒出来让人自己悟。

格式统一：量到的数 -> 判据 -> 结论 -> 该怎么办。
没有"该怎么办"的检查项不放进来，那只是噪声。

不做的事
--------
· 不猜。判据不足时明确说"这段数据判不了"并说明缺什么 ——
  基于 3 帧样本的结论比不报更糟。
· 不替代离线复算。这里全是【在线记录下来的】数字，回答"当时发生了什么"，
  不回答"换个参数会怎样"。
"""
import sys
import numpy as np
# 【必须显式指到 tools/pcrec】否则 Python 会先找脚本自己所在的目录，
# 加载到 tools/hand_diag/pcrec.py 那份停在 v3 的旧库——它不认识块 30/31/32，
# 而且带着 IkDbg 608/680 的尺寸错位。后果不是报错，是【静默少几块】：
# 报告照常输出，只是 IK 内部和 ROM 那几节空着，看起来像"这次没录到"。
# 隔壁 pcrec_diagnose.py 和 gen_v4_selftest.py 都写了这三行，就它漏了。
import os as _os
sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "pcrec"))
from pcrec import load, calib

OK, WARN, BAD, INFO = "✅", "⚠️ ", "❌", "  "


def hr(t=""):
    print("\n" + "=" * 74)
    if t:
        print(t)
        print("=" * 74)


def pct(x):
    return f"{100.0 * x:.1f}%"


def hget(r, *keys, default=None):
    """header 里同一个量可能在 params(扁平) 或 hm20/autocalib(嵌套) 下。
    录制程序不同版本写法不一样，读取端两种都要认，否则会误报'缺参数'。"""
    for src in (r.header.get("params", {}), r.header.get("hm20", {}),
                r.header.get("autocalib", {}), r.header):
        if isinstance(src, dict):
            for k in keys:
                if k in src:
                    return src[k]
    return default


# ---------------------------------------------------------------------------
def sec_overview(r):
    hr("0. 概览")
    print(r.summary())
    if r.trailer.get("dropped", 0):
        print(f"\n{BAD} 有 {r.trailer['dropped']} 个块被丢弃（队列满）。数据有洞，"
              "下面所有比例类结论都要打折看。")
    if r.camsettings:
        print("\n相机运行参数：")
        for c in r.camsettings:
            print(f"  cam{c['camId']}  {c['width']}x{c['height']}  曝光{c['exposureUs']:.0f}us  "
                  f"增益{c['gain']:.1f}  阈值{c['threshold']}  fps{c['fps']:.0f}")
    else:
        print(f"\n{WARN}没有相机运行参数块。曝光/增益/阈值直接决定光斑质量，"
              "两次录制精度不同时会分不清是算法参数变了还是相机设置变了。")


# ---------------------------------------------------------------------------
def sec_dorsum(r):
    hr("1. 手背刚体：标签稳不稳，为什么")
    d = r.diag
    if d is None or len(d) == 0:
        print(f"{BAD} 没有 Hm20Diag 块，这一节判不了。")
        return

    amb = float(np.median(d["dorsumSelfAmbMm"]))
    print(f"自歧义（模板最优错解的 Kabsch 残差）: {amb:.2f} mm")
    if amb < 0:
        print(f"  {WARN}没算出来。多半是 tmplMm 还没设就开录了。")
    elif amb < 3.0:
        print(f"  {BAD} 【最该先解决的问题】<3mm 说明手背 5 点近似共面 + 近似反序对称。")
        print(f"     平面点集的镜像在三维里是一个真旋转，Kabsch 没资格区分五边形")
        print(f"     '正着读'和'倒着读'。错解残差跟观测噪声同量级，单帧判就是抛硬币。")
        print(f"     -> 去重贴手背 5 点，让极角间隔明显不均匀。改代码没用。")
    elif amb < 6.0:
        print(f"  {WARN}勉强。噪声大的时候偶尔会翻。")
    else:
        print(f"  {OK} 布局好，标签不会翻。")

    reason = d["dorsumReason"]
    names = {0: "解出来了", 1: "候选点不足", 2: "无可行解", 3: "歧义拒解", 4: "残差超限"}
    print("\n每帧的求解结果分布：")
    for k in sorted(set(reason.tolist())):
        print(f"  {names.get(int(k), k):12s} {pct(float((reason==k).mean())):>7}   "
              f"{int((reason==k).sum())} 帧")
    okf = float((reason == 0).mean())
    if okf < 0.9:
        print(f"  {WARN}只有 {pct(okf)} 的帧解出了手背刚体。")
        if float((reason == 3).mean()) > 0.1:
            print(f"     '歧义拒解' 占 {pct(float((reason==3).mean()))} —— 这是【建锁】困难，"
                  "不是稳态问题。配合自歧义看。")

    hist = d["dorsumByHistory"][reason == 0]
    if len(hist):
        hf = float(hist.mean())
        print(f"\n解出来的帧里，靠位姿连续性仲裁的占 {pct(hf)}")
        if hf > 0.3:
            print(f"  {WARN}超过 30%。单帧裕度长期不够，一直靠时序硬撑 —— 丢锁就回不来。")
        else:
            print(f"  {OK} 大部分帧靠几何本身就能定，时序只是兜底。")

    mm_all = d["dorsumMarginMm"][reason == 0]
    uniq = float((mm_all > 1e6).mean())   # 1e9 = 哨兵：只找到唯一解，没有竞争解
    m = mm_all[(mm_all > 0) & (mm_all < 1e6)]
    print(f"裕度：{pct(uniq)} 的帧只有唯一解（无竞争，最好的情况）")
    if len(m):
        print(f"      其余帧中位 {np.median(m):.2f} mm，5% 分位 {np.percentile(m,5):.2f} mm")
        if np.percentile(m, 5) < 1.0:
            print(f"  {WARN}有帧裕度不到 1mm，那些帧是靠时序才没翻的。")

    # ---- 贴点重捕：球掉了重贴回去，模板有没有跟上 ----
    if r.skeleton:
        obs = np.array([s["observed"][:5] for _, s in r.skeleton]).astype(bool)
        for k in range(5):
            v = obs[:, k]
            if v.all():
                continue
            idx = np.where(~v)[0]
            runs, st = [], idx[0]
            for a, b in zip(idx[:-1], idx[1:]):
                if b != a + 1:
                    runs.append((st, a)); st = b
            runs.append((st, idx[-1]))
            lo, hi = max(runs, key=lambda ab: ab[1] - ab[0])
            dur = (hi - lo + 1) / max(len(v), 1)
            if dur > 0.15:
                print(f"\n  {BAD} 手背 m{k} 有一段连续丢失 {hi-lo+1} 帧"
                      f"（占全长 {pct(dur)}），到第 {hi} 帧还没回来。")
                print(f"     如果那颗球是掉了重贴的：重贴位置离模板通常 14~18mm，")
                print(f"     而收点容差只有 6mm，求解器会【永远】拒绝它 —— 一直 4/5 内点。")
                print(f"     -> 需要 DorsumRigidSolver::observeForRepair 的贴点重捕。")

    # ---- 标签翻转：只有 sourcePointId 能查 ----
    sp = d["sourcePointId"]
    if sp.shape[1] >= 5:
        flips = 0
        for k in range(5):
            col = sp[:, k]
            vv = col[col >= 0]
            if len(vv) > 1:
                flips += int((np.diff(vv) != 0).sum())
        print(f"\n手背 5 个标签的 sourcePointId 变更次数合计: {flips}")
        print(f"  （标签翻了 = 同一物理点换了编号；点丢了 = id 变成 -1）")
        if flips > len(d) * 0.02:
            print(f"  {BAD} 平均每 {len(d)/max(flips,1):.0f} 帧就换一次，太频繁。")
        elif flips:
            print(f"  {WARN}偶发。")
        else:
            print(f"  {OK} 全程没变过。")


# ---------------------------------------------------------------------------
def sec_thumb(r):
    hr("2. 拇指：外翻的根因在哪一段")
    d = r.diag
    if d is None:
        print(f"{BAD} 无诊断块。")
        return
    p0 = float(np.median(d["thumbPronation0"]))
    ct = float(np.median(d["thumbPronationContrast"]))
    cov = float(np.median(d["thumbCoverage"]))
    fitted = bool(np.median(d["thumbPronationFitted"]) > 0.5)
    print(f"thumbPronation0 = {p0:+.3f} rad ({np.degrees(p0):+.0f}°)   "
          f"已拟合={fitted}   碗底对比度={ct:.2f}   拇指覆盖度={cov:.2f}")
    if abs(p0) < 1e-6:
        print(f"  {BAD} 是 0，等于没补第一掌骨的常数旋前。")
        print(f"     模型的拇指屈曲铰链轴跟四指只差 17.4°（实测），真人要 80~90°，")
        print(f"     缺口约 73° —— 这就是'每一次都不往手心弯'的直接原因。")
        if ct < 2.0:
            print(f"     对比度 {ct:.2f} < 2：这轮拇指没怎么动，量出来是噪声，所以没敢写。")
            print(f"     -> 重录【手掌不动、拇指反复对掌抵住小指根】30 秒。")
    elif ct < 2.0:
        print(f"  {WARN}值写进去了但对比度只有 {ct:.2f}，可信度低。")
    else:
        print(f"  {OK} 幅值 {abs(np.degrees(p0)):.0f}° 落在预期的 80~90° 附近，对比度也够。")

    sk = d["thumbFixSkip"]
    names = {0: "正常执行", 1: "参数为0", 2: "三点全实测", 3: "无腕部位姿",
             4: "手背<3点", 5: "退化锚点", 6: "掌骨轴退化", 7: "部分可见(交给IK)"}
    print("\ncorrectPredictedThumb 的执行情况：")
    for k in sorted(set(sk.tolist())):
        print(f"  {names.get(int(k), k):16s} {pct(float((sk==k).mean())):>7}")
    if float((sk == 5).mean()) > 0.05:
        print(f"  {BAD} 有 {pct(float((sk==5).mean()))} 的帧走退化锚点（拿手背质心当 CMC）。")
        print(f"     绕一条偏了 15mm 的直线转 80°，点位移误差约 19mm，比一节指骨还长。")

    v = d["fingerIkValid"][:, 0]
    rm = d["fingerIkRmseMm"][:, 0]
    good = rm[(rm > 0) & (rm < 1e6)]
    if len(good):
        print(f"\n拇指 IK：生效率 {pct(float(v.mean()))}，残差中位 {np.median(good):.1f} mm")
    else:
        print(f"\n拇指 IK：没有残差样本")
    print(f"  对比四指生效率 {pct(float(d['fingerIkValid'][:,1:].mean()))}")

    # ---- IK 总闸：不查这个会把"拇指没修好"归错因 ----
    if float(d["ikActive"].mean()) < 0.01:
        print(f"\n  {BAD} 【IK 全程 0% 生效】—— 上面那些拇指参数【一个都没被用上】。")
        br = float(np.median(d["autoBundleRmseMm"]))
        print(f"     autoIkUsable={pct(float(d['autoIkUsable'].mean()))}  "
              f"autoStage={int(np.median(d['autoStage']))}  autoBundleRmseMm={br:.2f}")
        if br >= 4.0:
            print(f"     ikUsable() 要求 bundleRmseMm < 4.0，实测 {br:.2f} —— 这就是总闸。")
            print(f"     thumbPronation0 只被 markerFK 消费，而 markerFK 只在 IK 里跑。")
            print(f"     所以标定得再好也等于没标。-> 先解决束调整残差，再谈拇指。")

    ss = d["segSource"][:, 1:4]
    lab = {0: "None", 1: "Predicted", 2: "Geometry", 3: "Ik", 4: "Net"}
    print("\n拇指三段的姿态来源：")
    for j, nm in enumerate(["掌骨", "近节", "远节"]):
        row = ss[:, j]
        print(f"  {nm}  " + "  ".join(f"{lab.get(int(k),k)}{pct(float((row==k).mean()))}"
                                      for k in sorted(set(row.tolist()))))
    if float((ss == 1).mean()) > 0.4:
        print(f"  {WARN}Predicted 占比高。Predicted 段的骨轴 = 两个【网络补出来的点】之差，")
        print(f"     误差被骨长一除就放大。这正是 seg_rot6d 那条路要替掉的。")


# ---------------------------------------------------------------------------
def sec_hand(r):
    hr("3. 手性：到底哪里反了")
    d = r.diag
    if d is None:
        print(f"{BAD} 无诊断块。")
        return
    src = {"面板勾选": d["handPanelIsRight"], "自标定采用": d["handAutoIsRight"],
           "自标定检测": d["handAutoDetected"], "模型判定": d["handAiIsRight"]}
    print("四个来源各自的众数（1=右手 0=左手）：")
    vals = {}
    for k, v in src.items():
        m = int(round(float(np.median(v))))
        vals[k] = m
        print(f"  {k:10s} {'右手' if m else '左手'}    (右手占比 {pct(float(v.mean()))})")

    sig = float(np.median(d["handSignMm"]))
    print(f"\n判据强度 handSignMm 中位 {sig:.2f} mm   (阈值 4mm)   "
          f"已判定帧占比 {pct(float(d['handednessKnown'].mean()))}")
    if abs(sig) < 4.0:
        print(f"  {WARN}判据低于阈值 —— 手大部分时间摊平，几何上就分不出手性。")
        print(f"     '检测'那一路在这种情况下不可信，不要拿它当依据。")

    if len(set(vals.values())) > 1:
        print(f"\n  {BAD} 四个来源不一致：")
        for k, m in vals.items():
            print(f"       {k} = {'右' if m else '左'}")
        if abs(sig) >= 4.0:
            print(f"     判据强度够（{sig:.1f}mm），这是【真的不一致】而不是判不出来。")
            print(f"     -> 重点查 handPanelIsRight vs handAutoIsRight：")
            print(f"        面板下发没生效的话，worker 的 handIsRight_ 会停在默认值 true，")
            print(f"        然后自标定按右手解一只左手 -> bundleRmse 降不下去 -> IK 全废。")
        else:
            print(f"     但判据强度不够，先把手立起来重录，让 handSignMm 上到 4mm 以上。")
    else:
        print(f"\n  {OK} 四个来源一致。")

    if float(d["mirrorSuspect"].mean()) > 0.1:
        print(f"  {WARN}mirrorSuspect 在 {pct(float(d['mirrorSuspect'].mean()))} 的帧里为真。")


# ---------------------------------------------------------------------------
def sec_rigid(r):
    hr("4. 刚体不变量（只有 rigid_motion 协议有意义）")
    proto = r.header.get("protocol", "")
    if proto != "rigid_motion":
        print(f"{INFO}协议是 {proto}，不是 rigid_motion，这一节跳过。")
        print(f"{INFO}rigid_motion 段最有价值：手型固定只做整体平移旋转时，20 点两两距离")
        print(f"{INFO}全部恒定 = 190 个刚体约束，是唯一不需要外部真值的精度判据。")
        return
    if not r.skeleton:
        print(f"{BAD} 没有骨架帧。")
        return
    P = np.array([s["pos"].reshape(20, 3) for _, s in r.skeleton])
    O = np.array([s["observed"] for _, s in r.skeleton]).astype(bool)
    iu = np.triu_indices(20, 1)
    D = np.linalg.norm(P[:, :, None, :] - P[:, None, :, :], axis=-1)[:, iu[0], iu[1]]
    M = (O[:, iu[0]] & O[:, iu[1]])
    stds, names = [], []
    for k in range(D.shape[1]):
        v = D[M[:, k], k]
        if len(v) >= 30:
            stds.append(v.std()); names.append((iu[0][k], iu[1][k]))
    if not stds:
        print(f"{WARN}可用点对不足（每对至少 30 帧同时可见），判不了。")
        return
    stds = np.array(stds)
    print(f"{len(stds)} 个点对的距离标准差：中位 {np.median(stds):.2f} mm  "
          f"p90 {np.percentile(stds,90):.2f}  最大 {stds.max():.2f}")
    print(f"（手型固定时这些距离物理上恒定，标准差就是端到端误差）")
    for w in np.argsort(stds)[-5:][::-1]:
        print(f"   点{names[w][0]:2d}-点{names[w][1]:2d}   σ={stds[w]:.2f} mm")
    if np.median(stds) > 3.0:
        print(f"  {BAD} 中位 >3mm，端到端精度差。先看是不是标签在翻（第 1 节）。")
    elif np.median(stds) > 1.5:
        print(f"  {WARN}中位 {np.median(stds):.1f}mm，有改进空间。")
    else:
        print(f"  {OK} 中位 {np.median(stds):.1f}mm。")


# ---------------------------------------------------------------------------
def sec_pipeline(r):
    hr("5. 前级：光斑 / 三角化 / 相机贡献")
    if r.cluster:
        res = np.concatenate([c["residualPx"] for _, c in r.cluster])
        res = res[res >= 0]
        ns = np.concatenate([c["nSupport"] for _, c in r.cluster])
        if len(res):
            print(f"三角化重投影残差: 中位 {np.median(res):.2f} px  "
                  f"p95 {np.percentile(res,95):.2f} px")
            print(f"  【所有毫米级阈值都该表达成它的倍数】写死绝对值的话换机位就全废。")
        print("支撑相机数分布: " + "  ".join(
            f"{k}台{pct(float((ns==k).mean()))}" for k in sorted(set(ns.tolist()))))
        if float((ns <= 2).mean()) > 0.2:
            print(f"  {WARN}超过 20% 的点只有 ≤2 台相机支撑，深度方向误差会明显偏大。")
    if r.blobs:
        allb = np.concatenate([b for v in r.blobs.values() for _, b in v])
        if len(allb) and allb["area"].max() > 0:
            ar = allb["area"]
            print(f"\n光斑面积: 中位 {np.median(ar):.0f} px  p95 {np.percentile(ar,95):.0f}")
            bw, bh = allb["bw"].astype(float), allb["bh"].astype(float)
            ok = (bw > 0) & (bh > 0)
            if ok.sum():
                ratio = np.maximum(bw[ok], bh[ok]) / np.maximum(np.minimum(bw[ok], bh[ok]), 1)
                stuck = float((ratio > 1.3).mean())
                print(f"外接框长宽比 >1.3 的占 {pct(stuck)}  （粘连球的特征）")
                if stuck > 0.05:
                    print(f"  {BAD} 粘连是最难查的失效模式：不报错、不丢点，只是悄悄给出一个")
                    print(f"     位于两球之间的错误质心，然后被当成正常观测一路用下去。")
                    print(f"     -> 调低曝光/阈值，或把贴得太近的球分开。")
            peak = allb["peak"]
            if peak.max() > 0:
                sat = float((peak >= 254).mean())
                print(f"峰值饱和(>=254)占 {pct(sat)}")
                if sat > 0.1:
                    print(f"  {WARN}过曝会让质心偏移且面积失真 -> 降曝光。")
        else:
            print(f"\n{WARN}光斑只有坐标，没有 area/peak/bbox。走的是老路径，粘连检测做不了。")
    if r.diag is not None:
        lat = r.diag["latencyMs"]
        lat = lat[lat > 0]
        if len(lat):
            print(f"\nhm20 单帧耗时: 中位 {np.median(lat):.1f} ms  "
                  f"p99 {np.percentile(lat,99):.1f} ms")


# ---------------------------------------------------------------------------
def sec_replay(r):
    hr("6. 离线复现的可行性")
    have, miss = [], []
    (have if r.assoc_input else miss).append("AssocInput（送进网络的候选点及顺序）")
    (have if r.netraw else miss).append("NetRaw（网络原始输出）")
    (have if calib(r) else miss).append("相机内外参")
    (have if hget(r, "dorsumSelfAmbiguityMm", "thumbRollOffsetRad", "assocDist")
     is not None else miss).append("参数快照")
    (have if hget(r, "thumbPronation0", "autoBundleRmseMm") is not None
     else miss).append("自标定当前结果")
    (have if r.blobs else miss).append("2D 光斑")
    for h in have:
        print(f"  {OK} {h}")
    for m in miss:
        print(f"  {BAD} 缺 {m}")
    if miss:
        print(f"\n缺任何一项，离线复算都无法跟在线对拍 —— 而对不上的时候你分不清")
        print(f"是参数的差异还是复现本身错了，调参结论全部不可信。")
    else:
        print(f"\n{OK} 齐了。可以逐级对拍：2D->3D->候选点->网络输出->骨架。")
        h0 = r.netraw[0][0]
        print(f"     网络归一化: center={np.round(h0['center'],1)} scale={h0['scale']:.1f}")
        print(f"     （离线反归一化必须用这两个数，自己重算会错位）")


# ---------------------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    r = load(sys.argv[1])
    for fn in (sec_overview, sec_dorsum, sec_thumb, sec_hand,
               sec_rigid, sec_pipeline, sec_replay):
        try:
            fn(r)
        except Exception as e:
            print(f"\n{BAD} [{fn.__name__}] 这一节出错: {type(e).__name__}: {e}")
    hr("结束")
    print("有拿不准的把这份报告连同 .pcrec + meta.json 一起发出来。")


if __name__ == "__main__":
    main()
