#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe_dorsum_reversal.py —— 量你自己那 5 个手背点能不能被稳定定标签。

用法:
    python probe_dorsum_reversal.py                       # 用文档里记的那组半径当参考
    python probe_dorsum_reversal.py hand_template.json    # 用你真机标定出来的模板
       (Windows 一般在 文档/…/hand_template.json，HandTemplateStore 写的那个)

它算什么
--------
把手背 5 点模板跟它自己的 119 种【错误排列】逐个做 Kabsch，取残差最小的那个。
这个数就是"最像正解的错解"，也就是几何定标签的确定性上限：

    > 6 mm   布局好，标签不会翻
    3 ~ 6 mm 勉强，噪声大时偶尔翻
    < 3 mm   布局病态，任何算法都会在正解和错解之间来回翻

为什么会病态
------------
手背 5 点近似共面。平面点集的【镜像】在三维里是一个真旋转（把平面翻个面），
而 Kabsch 只解旋转不解手性，所以它没资格区分"五边形正着读"和"倒着读"。
当 5 个点又接近正五边形时，反序读法几乎完美地叠回自己身上。

SKELETON_UPGRADE.md 里记过一次真实事故：
    GT 手背半径序列:  [28.2, 35.2, 21.8, 23.7, 34.4]
    早期冻结的模板:   [34.5, 23.8, 21.9, 35.4, 28.3]   <- 正好倒过来
那次归因成"五重对称"，真正的机制是共面 + 近似反序对称。

自标定的校验阈值是 max(1.2mm, 2.5σ) ≈ 3~5mm。如果你的自歧义低于它，
**一个反序的模板能通过校验**，而且事后没有任何诊断数字会报警。
"""
import sys
import json
import itertools
import numpy as np


def kabsch(src, dst):
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = cd - R @ cs
    return R, t, float(np.sqrt(((src @ R.T + t - dst) ** 2).sum(1).mean()))


def self_ambiguity(P):
    best = (1e18, None)
    for perm in itertools.permutations(range(5)):
        if perm == (0, 1, 2, 3, 4):
            continue
        _, _, r = kabsch(P, P[list(perm)])
        if r < best[0]:
            best = (r, perm)
    return best


def describe(P):
    c = P.mean(0)
    d = P - c
    # 主平面
    _, s, Vt = np.linalg.svd(d - d.mean(0))
    n = Vt[2]
    flat = float(np.abs(d @ n).max() * 2)
    rad = np.linalg.norm(d - np.outer(d @ n, n), axis=1)
    e1 = Vt[0]; e2 = Vt[1]
    ang = np.degrees(np.arctan2(d @ e2, d @ e1)) % 360
    order = np.argsort(ang)
    gaps = np.diff(np.concatenate([ang[order], [ang[order][0] + 360]]))
    return flat, rad, ang, gaps


def report(name, P):
    P = np.asarray(P, float)
    P = P - P.mean(0)
    r, perm = self_ambiguity(P)
    flat, rad, ang, gaps = describe(P)
    print(f"\n===== {name} =====")
    print(f"  离平面起伏(峰峰值) : {flat:6.2f} mm      (共面 => 反序歧义无法用残差破)")
    print(f"  到质心的半径       : {np.array2string(np.round(rad,1))}  mm")
    print(f"  绕质心的极角间隔   : {np.array2string(np.round(gaps,0))}  ° (越接近 72,72,72,72,72 越糟)")
    print(f"  两两距离           : {np.array2string(np.round(np.linalg.norm(P[:,None]-P[None,:],axis=-1)[np.triu_indices(5,1)],1))} mm")
    rev = perm == (0, 4, 3, 2, 1) or perm == (4, 3, 2, 1, 0)
    verdict = ("布局好，标签不会翻" if r > 6 else
               "勉强，噪声大时偶尔翻" if r > 3 else
               "★ 布局病态，必须重贴点 ★")
    print(f"\n  最优错解残差       : {r:6.2f} mm   错解 = {perm}{'  (反序)' if rev else ''}")
    print(f"  判定               : {verdict}")
    if r < 3:
        print("\n  怎么改：不要贴成一圈近似均匀的五边形。让【极角间隔明显不均匀】"
              "\n          效果最好（实测 1.8 -> 8.2mm），其次是把其中一个点垫高"
              "\n          10mm 以上打破共面（1.8 -> 5.7mm）。两个一起做能到 10.9mm。"
              "\n          这是零代码、零重训的改动，比继续调任何参数都划算。")
    return r


def main():
    # 参考布局：文档里记的真实半径 + 近似均匀角度
    rad = np.array([28.2, 35.2, 21.8, 23.7, 34.4])
    ang = np.deg2rad([0, 74, 145, 212, 289])
    ref = np.stack([rad * np.cos(ang), rad * np.sin(ang), np.zeros(5)], 1)
    report("参考：近似正五边形 + 共面（= 现状的样子）", ref)

    angB = np.deg2rad([0, 40, 95, 190, 250])
    refB = np.stack([rad * np.cos(angB), rad * np.sin(angB), np.zeros(5)], 1)
    report("对照：只把极角改成明显不均匀", refB)

    refC = refB.copy(); refC[1, 2] = 12.0
    report("对照：极角不均匀 + 1 号点垫高 12mm", refC)

    if len(sys.argv) > 1:
        with open(sys.argv[1], "r", encoding="utf-8") as f:
            root = json.load(f)
        back = root.get("backMarkers")
        if not back:
            sys.exit("这个 json 里没有 backMarkers 字段")
        P = np.array([[p[0], p[1], p[2]] for p in back], float)
        report(f"★ 你的真实模板：{sys.argv[1]}", P)
    else:
        print("\n（没传 hand_template.json —— 传进来才能量到你自己那只手的数。"
              "\n  这是本次唯一不用信我的数字，务必自己跑一遍。）")


if __name__ == "__main__":
    main()
