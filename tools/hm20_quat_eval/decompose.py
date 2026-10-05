"""
把 L0（零噪声、标签全对）那 24.3° 拆开，看多少是"贴球偏置"，多少是"代码写法"。

三种取向来源对比，全部对同一根骨的真值骨轴：
  A) 代码现状       computeSegmentQuats 实际用的 marker 对
  B) 最优可用marker对  在 3 个 marker 的所有配对里，挑对这根骨最好的那个
  C) 理论下限        该 marker 对能达到的最好情况（= B 的统计）
再单独给出"marker对 vs 它自己跨越的那段骨"的偏差，用来对齐你测的 10.2°。
"""
import numpy as np
from hand_rig import HandRig, axis_angle_deg, FINGERS, SEGS
from seg_quat import compute_segment_quats, gt_segment_frames, N_SEG, SEG_NAMES

FPS = 120.0
rig = HandRig("right")

codeA = [[] for _ in range(N_SEG)]
bestB = [[] for _ in range(N_SEG)]
bestsrc = [dict() for _ in range(N_SEG)]
offset_only = []          # marker对 vs 同名骨（拟合你测的 10.2°）

for i in range(1500):
    t = i / FPS
    P, Qgt, Qpalm, Bone = rig.forward(t, motion="grasp")
    Dgt = gt_segment_frames(Qgt, Bone)
    _, D = compute_segment_quats(P, np.eye(3))

    for f in range(5):
        base = 5 + f * 3
        # 所有可用 marker 配对方向
        pairs = {}
        for a in range(3):
            for b in range(3):
                if a == b:
                    continue
                v = P[base + b] - P[base + a]
                pairs[(a, b)] = v / (np.linalg.norm(v) + 1e-12)
        for k in range(3):
            s = 1 + f * 3 + k
            codeA[s].append(axis_angle_deg(D[s], Dgt[s]))
            errs = {pk: axis_angle_deg(pv, Dgt[s]) for pk, pv in pairs.items()}
            pk = min(errs, key=errs.get)
            bestB[s].append(errs[pk])
            bestsrc[s][pk] = bestsrc[s].get(pk, 0) + 1

        # 贴球偏置本身：近节骨真值 vs (prox marker -> mid marker)
        # 这两者在解剖上"名义对应"，其差就是纯偏置项
        v = P[base + 1] - P[base + 0]
        v = v / np.linalg.norm(v)
        o, tp = Bone[(FINGERS[f], "prox")]
        bone = (tp - o) / np.linalg.norm(tp - o)
        offset_only.append(axis_angle_deg(v, bone))


def st(a):
    a = np.array([x for x in a if np.isfinite(x)])
    return np.percentile(a, 50), np.percentile(a, 90), a.max()


print("=" * 78)
print("L0 误差分解（零噪声、标签100%正确）")
print("=" * 78)
print(f"{'segment':16s} {'A 代码现状':>22s} {'B 最优marker对':>24s}")
print(f"{'':16s} {'中位':>7s}{'p90':>7s}{'max':>7s}  {'中位':>7s}{'p90':>7s}{'max':>7s}   最优源")
for s in range(1, N_SEG):
    a = st(codeA[s]); b = st(bestB[s])
    src = max(bestsrc[s], key=bestsrc[s].get)
    names = ["pp", "mp", "dp"]
    print(f"{SEG_NAMES[s]:16s} {a[0]:6.1f}°{a[1]:6.1f}°{a[2]:6.1f}°  "
          f"{b[0]:6.1f}°{b[1]:6.1f}°{b[2]:6.1f}°   {names[src[0]]}->{names[src[1]]}")

allA = [x for s in range(1, N_SEG) for x in codeA[s]]
allB = [x for s in range(1, N_SEG) for x in bestB[s]]
print("-" * 78)
print(f"合计  代码现状: 中位 {st(allA)[0]:.1f}°  p90 {st(allA)[1]:.1f}°  max {st(allA)[2]:.1f}°")
print(f"合计  最优配对: 中位 {st(allB)[0]:.1f}°  p90 {st(allB)[1]:.1f}°  max {st(allB)[2]:.1f}°")
print(f"      => 改成最优配对能消掉的部分: 中位 {st(allA)[0]-st(allB)[0]:.1f}°")
print()
o = st(offset_only)
print(f"纯贴球偏置项（近节骨 vs prox->mid 连线）: 中位 {o[0]:.1f}°  p90 {o[1]:.1f}°  max {o[2]:.1f}°")
print("  （你实测的骨轴偏差是 中位 10.2° / p90 21.6° / max 53.3°，用于校准本模型）")
