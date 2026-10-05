"""
手指段姿态四元数可靠性分析。

误差分三层剥离，逐层叠加，看每层各贡献多少：
  L0 完美：真值 marker，标签 100% 正确，无噪声 -> 只剩几何/代码结构误差
  L1 +点云噪声：按 track_results.jsonl 标定出的 3D 误差水平
  L2 +标签错误：按 v6 实测混淆率注入（漏检 4.96% / 手背互换 0.49% / 手指判别 3.74%）

同时单独统计 boneFrame 参考向量分支切换（|x_z| 跨 0.9）导致的四元数跳变。
"""
import numpy as np
import sys
from hand_rig import HandRig, quat_angle_deg, axis_angle_deg, mat2quat, FINGERS, SEGS
from seg_quat import (compute_segment_quats, gt_segment_frames, bone_frame,
                      ref_switch_flag, SEG_NAMES, N_SEG)

FPS = 120.0


def kabsch(src, dst):
    """src,dst: (n,3) -> R,t 使 R@src+t ~ dst"""
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cd - R @ cs


def run(noise_mm=0.0, label_err=False, frames=1200, motion="grasp", seed=1,
        dropout=0.0):
    rng = np.random.default_rng(seed)
    rig = HandRig("right")

    # 中立位模板（用于 Kabsch 手背对齐）
    P0, _, _, _ = rig.forward(0.0, motion="static", speed_mm_s=0.0, amp_mm=0.0)
    R0, T0 = rig.palm_pose(0.0, 0.0, 0.0)
    tmpl_dorsum = (P0[:5] - T0) @ R0   # 转回手掌局部系

    axis_err = [[] for _ in range(N_SEG)]     # 骨轴方向误差（度）
    quat_jump = [[] for _ in range(N_SEG)]    # 相邻帧四元数变化（度）
    gt_jump = [[] for _ in range(N_SEG)]      # 真值相邻帧变化（度）
    ref_flips = np.zeros(N_SEG, dtype=int)
    prevQ = None
    prevRef = None
    prevGT = None
    wrist_jump = []
    dup_check = []

    for i in range(frames):
        t = i / FPS
        P, Qgt, Qpalm, Bone = rig.forward(t, motion=motion)
        Pn = P.copy()

        if noise_mm > 0:
            Pn += rng.normal(0, noise_mm, Pn.shape)

        seen = np.ones(20, dtype=bool)
        if dropout > 0:
            seen = rng.random(20) > dropout

        if label_err:
            # 手背互换 0.49%（同类互换，对手指无影响，但影响 Kabsch）
            if rng.random() < 0.0049:
                a, b = rng.choice(5, 2, replace=False)
                Pn[[a, b]] = Pn[[b, a]]
            # 手指判别错误 3.74%：同指相邻节互换 或 邻指同节互换
            for f in range(5):
                if rng.random() < 0.0374:
                    base = 5 + f * 3
                    if rng.random() < 0.6:
                        k = rng.integers(0, 2)
                        Pn[[base + k, base + k + 1]] = Pn[[base + k + 1, base + k]]
                    else:
                        g = (f + 1) % 5
                        k = rng.integers(0, 3)
                        Pn[[base + k, 5 + g * 3 + k]] = Pn[[5 + g * 3 + k, base + k]]
            # 漏检 4.96%：该点没被认领 -> 实际系统会用网络 pos 头兜底，
            # 这里用"上一帧位置"近似兜底（等价于预测滞后一帧）
            miss = rng.random(20) < 0.0496
            seen = seen & ~miss

        # 腕部位姿 Kabsch（手背 5 点）
        idx = [m for m in range(5) if seen[m]]
        if len(idx) >= 3:
            R, T = kabsch(tmpl_dorsum[idx], Pn[idx])
        else:
            R, T = np.eye(3), np.zeros(3)
        wq = mat2quat(R)

        Q, D = compute_segment_quats(Pn, R)
        Dgt = gt_segment_frames(Qgt, Bone)

        # 骨轴方向误差
        for s in range(1, N_SEG):
            axis_err[s].append(axis_angle_deg(D[s], Dgt[s]))

        # 帧间跳变
        if prevQ is not None:
            for s in range(1, N_SEG):
                quat_jump[s].append(quat_angle_deg(Q[s], prevQ[s]))
                gt_jump[s].append(axis_angle_deg(Dgt[s], prevGT[s]))
                if ref_switch_flag(D[s]) != prevRef[s]:
                    ref_flips[s] += 1
            wrist_jump.append(quat_angle_deg(wq, prevWQ))
        prevQ = Q.copy()
        prevGT = Dgt.copy()
        prevWQ = wq
        prevRef = np.array([ref_switch_flag(D[s]) for s in range(N_SEG)])

        # 结构性重复检查：seg(prox) 与 seg(mid) 是否恒等
        for f in range(5):
            dup_check.append(quat_angle_deg(Q[1 + f * 3], Q[2 + f * 3]))

    return dict(axis_err=axis_err, quat_jump=quat_jump, gt_jump=gt_jump,
                ref_flips=ref_flips, wrist_jump=wrist_jump,
                dup=np.array(dup_check))


def pct(a, p):
    a = np.asarray([x for x in a if np.isfinite(x)])
    return float(np.percentile(a, p)) if len(a) else float("nan")


def report(tag, r):
    print(f"\n{'='*74}\n{tag}\n{'='*74}")
    print(f"{'segment':16s} {'骨轴误差 中位':>12s} {'p90':>8s} {'max':>8s} "
          f"{'帧间跳变p99':>11s} {'ref翻转':>8s}")
    for s in range(1, N_SEG):
        print(f"{SEG_NAMES[s]:16s} {pct(r['axis_err'][s],50):11.1f}° "
              f"{pct(r['axis_err'][s],90):7.1f}° {pct(r['axis_err'][s],100):7.1f}° "
              f"{pct(r['quat_jump'][s],99):10.1f}° {r['ref_flips'][s]:7d}")
    allax = [x for s in range(1, N_SEG) for x in r['axis_err'][s]]
    print(f"{'-'*74}")
    print(f"全指节骨轴误差: 中位 {pct(allax,50):.1f}°  p90 {pct(allax,90):.1f}°  "
          f"max {pct(allax,100):.1f}°")
    print(f"腕部四元数帧间跳变: 中位 {pct(r['wrist_jump'],50):.2f}°  "
          f"p99 {pct(r['wrist_jump'],99):.2f}°  max {pct(r['wrist_jump'],100):.2f}°")
    print(f"近节/中节四元数差异: 中位 {np.median(r['dup']):.6f}°  "
          f"max {r['dup'].max():.6f}°   <- 0 表示两节恒等")


if __name__ == "__main__":
    print("mocap_stress 点云误差水平（从 track_results.jsonl 标定）:")
    print("  理想标定: p50=0.50mm  p95=0.97mm   召回 99.3%")
    print("  真实标定: p50=46.9mm  p95=75.3mm   召回 14.7%")

    report("L0  完美输入：真值marker + 标签全对 + 无噪声", run(0.0, False))
    report("L1  + 点云噪声 0.5mm（理想标定实测水平）", run(0.5, False))
    report("L2  + 点云噪声 0.5mm + v6 标签错误注入", run(0.5, True))
    report("L3  + 点云噪声 2.0mm（p90 工况）+ v6 标签错误", run(2.0, True))
