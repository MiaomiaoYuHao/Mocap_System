# -*- coding: utf-8 -*-
"""rom_trend_check.py —— 真实录制上的“弯曲→增加”回归

用法：
    python tools/pcrec/rom_trend_check.py 录制.pcrec build/rom_offline_run_new.exe build

它会：
1. 取录制里 romLearning=1 的帧段，跑新版 ROM 标定；
2. 解析标定出的 sign / lo / hi；
3. 对 ROM 之后的每一次握拳，按新版输出链做一次离线映射；
4. 断言 11 个屈曲维在每次握拳里都满足 fist_output > open_output。

这一步专门防“达标了但趋势反了”的回归。
"""
import re
import subprocess
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import pcrec as P

# 与 hand/HandModel.hpp::jointLimits() 同步。只用于离线回归。
TARGET_LIMITS = np.array([
    [-0.35, 1.00], [-0.17, 1.15], [-0.17, 0.95], [-0.26, 1.40],
    [-0.35, 1.60], [-0.44, 0.44], [0.00, 1.92],
    [-0.30, 1.60], [-0.28, 0.28], [0.00, 1.92],
    [-0.30, 1.60], [-0.28, 0.28], [0.00, 1.92],
    [-0.35, 1.62], [-0.48, 0.48], [0.00, 1.92],
], dtype=float)


def vec(lines, tag):
    for line in lines:
        if line.startswith(tag + " "):
            return np.array([float(x) for x in line.split()[1:]])
    raise RuntimeError(f"标定输出里找不到 {tag}")


def run_calibration(rec, romrun, workdir):
    r = P.load(rec)
    learn = np.where(r.jointout["romLearning"].astype(bool))[0]
    if len(learn) == 0:
        raise RuntimeError("录制里没有 romLearning=1 的帧")
    obs = Path(workdir) / "rom_trend_check.obs"
    subprocess.check_call([
        sys.executable, str(HERE / "rom_offline_dump.py"), rec, str(obs), "1.0",
        str(int(learn[0])), str(int(learn[-1] + 1)),
    ])
    txt = subprocess.check_output([romrun, "gate", str(obs)], text=True, errors="replace")
    lines = txt.splitlines()
    return r, vec(lines, "SIGNS").astype(int), vec(lines, "LO"), vec(lines, "HI"), txt


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2
    rec, romrun, workdir = sys.argv[1:]
    Path(workdir).mkdir(parents=True, exist_ok=True)
    r, sign, lo, hi, report = run_calibration(rec, romrun, workdir)

    q = r.jointout["qSolve"].astype(float)
    # 与新版 solveJointAngles 对齐：PIP 是弯曲量，恒非负。
    q[:, [6, 9, 12, 15]] = np.abs(q[:, [6, 9, 12, 15]])
    mapped = np.zeros_like(q)
    for i in range(16):
        v = sign[i] * q[:, i]
        t = np.clip((v - lo[i]) / max(1e-9, hi[i] - lo[i]), 0.0, 1.0)
        mapped[:, i] = TARGET_LIMITS[i, 0] + t * (TARGET_LIMITS[i, 1] - TARGET_LIMITS[i, 0])

    js = r.jointsolve
    own_curl = js["curl"].astype(float)
    own_valid = js["curlValid"].astype(bool)
    common = np.nanmean(np.where(own_valid[:, 1:], own_curl[:, 1:], np.nan), axis=1)
    learn_end = int(np.where(r.jointout["romLearning"].astype(bool))[0][-1])
    post = np.arange(learn_end + 1, len(q))

    # 每个屈曲维选“与输出最一致”的弯曲参考：本指 curl 或四指共同 curl。
    # 拇MCP/IP 通常选本指；四指 MCP/PIP 通常选共同；拇CMC 由数据决定。
    refs = {}
    for i in P.FLEX_IDX:
        f = 0 if i < 4 else 1 + (i - 4) // 3
        cands = [("own", own_curl[:, f], own_valid[:, f].astype(bool)),
                 ("common", common, np.isfinite(common))]
        best = None
        for name, ref, ok in cands:
            m = ok & np.isfinite(ref)
            if m[post].sum() < 20:
                continue
            a = mapped[post][m[post], i]
            b = ref[post][m[post]]
            if np.std(a) < 1e-12 or np.std(b) < 1e-12:
                continue
            ar = np.argsort(np.argsort(a))
            br = np.argsort(np.argsort(b))
            c = float(np.corrcoef(ar, br)[0, 1])
            if best is None or abs(c) > best[0]:
                best = (abs(c), c, name, ref, ok)
        if best is None:
            raise RuntimeError(f"{P.JOINT_NAMES[i]} 没有可用的弯曲参考")
        refs[i] = best

    cycles = []
    for line in r.fist_cycles():
        m = re.search(r"帧\s*(\d+)\.\.(\d+)", line)
        if m:
            cycles.append((int(m.group(1)), int(m.group(2))))

    bad = 0
    checked = 0
    print("ROM 后握拳循环趋势：delta = 输出(握拳) - 输出(张开)，单位度")
    for ci, (a, b) in enumerate(cycles):
        if b <= learn_end:
            continue
        deltas = {}
        for i in P.FLEX_IDX:
            _, _, _, ref, ok = refs[i]
            seg = np.arange(max(0, a - 8), min(len(q), b + 9))
            m = ok[seg] & np.isfinite(ref[seg])
            if m.sum() < 2:
                deltas[i] = 0.0
                continue
            c = ref[seg][m]
            frames = seg[m]
            fo = int(frames[int(np.argmin(c))])
            fc = int(frames[int(np.argmax(c))])
            deltas[i] = float(np.degrees(mapped[fc, i] - mapped[fo, i]))
        bad_here = [i for i in P.FLEX_IDX if deltas[i] <= 0.0]
        checked += 1
        print(f"  第{ci + 1}次: " + " ".join(
            f"{P.JOINT_NAMES[i]}={deltas[i]:+.1f}" for i in P.FLEX_IDX))
        if bad_here:
            bad += len(bad_here)
            print("    [FAIL] 这些屈曲维没有随握拳增加: " +
                  ", ".join(P.JOINT_NAMES[i] for i in bad_here))

    if checked == 0:
        print("[FAIL] ROM 后没有可检查的握拳循环")
        return 1
    if bad:
        print(f"\n==== 有用例失败：{bad} 个屈曲维方向不对 ====")
        print(report)
        return 1
    print(f"\n==== 全部通过：{checked} 次 ROM 后握拳，11 个屈曲维趋势都正确 ====")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
