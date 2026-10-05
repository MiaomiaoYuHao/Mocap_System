#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_v4_selftest.py —— 造一段合成的 v4 .pcrec，用来自测解析和诊断脚本

【为什么需要它】pcrec_diagnose.py 的结论有很强的指向性（"符号反了""ROM 塌了"）。
一个会给出错误结论的诊断脚本比没有诊断脚本更糟：它会让人照着错误方向去改代码。
所以每种故障都要有一段【已知答案】的数据来验证脚本确实认得出来。

字节布局直接用 pcrec.py 的 dtype 写，而那些 dtype 已经跟 C++ 的 offsetof
逐字段对过账，所以这里生成的文件和真机录出来的在格式上是同一回事。

用法：
    python3 gen_v4_selftest.py ok.pcrec        --case ok       期望：全部正常
    python3 gen_v4_selftest.py mir.pcrec       --case mirror   期望：报"符号反了/手性镜像"
    python3 gen_v4_selftest.py rom.pcrec       --case rom_dead 期望：报"ROM 这一级坏掉"
    python3 gen_v4_selftest.py hnd.pcrec       --case tmpl_flip 期望：报 tmplIsRight 不一致
"""
import argparse
import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pcrec"))
import pcrec  # noqa: E402


def chunk(ct, payload):
    return bytes([ct]) + struct.pack("<I", len(payload)) + payload


def rec_bytes(dt, row):
    row["recVersion"] = 1
    row["recBytes"] = dt.itemsize
    return row.tobytes()


def build(case, nframes=600):
    hdr = {"format": "mocap_pc_record", "version": 1,
           "protocol": "fist_open_cycle",
           "note": f"synthetic selftest case={case}",
           "params": {"handPanelIsRight": True}}
    hj = json.dumps(hdr, ensure_ascii=False).encode("utf-8")
    buf = bytearray()
    buf += pcrec.MAGIC + struct.pack("<I", 4) + struct.pack("<I", len(hj)) + hj

    t = np.arange(nframes)
    # 张开<->握拳，5 个循环。u=0 张开，u=1 握到最紧
    u = 0.5 * (1 - np.cos(2 * np.pi * 5 * t / nframes))

    mirror = (case == "mirror")
    rom_dead = (case == "rom_dead")
    tmpl_flip = (case == "tmpl_flip")

    for k in range(nframes):
        ts = int(1_000_000_000 + k * 8_333_333)
        uu = float(u[k])

        # ---- MarkerDbg：真值来自这里。握紧 -> 指尖到腕心距离变小 ----
        md = np.zeros(1, dtype=pcrec.MARKERDBG)[0]
        md["wallNs"] = ts
        md["frameTsNs"] = ts
        md["hasNet"] = 1
        pos = np.zeros((20, 3))
        # 手背五点摆在原点附近，质心 = 腕心
        back = np.array([[15, 10, 0], [-15, 10, 0], [-15, -10, 0],
                         [15, -10, 0], [0, 0, 0]], dtype=float)
        pos[0:5] = back
        # 四指：张开时指尖离腕心 95mm，握紧时 40mm
        d_tip = 95.0 - 55.0 * uu
        for f in range(4):
            ang = np.deg2rad(-30 + 20 * f)
            base = 5 + 3 + 3 * f          # 8,11,14,17 = pp
            for j, frac in enumerate([0.45, 0.72, 1.0]):
                d = d_tip * frac
                pos[base + j] = [d * np.cos(ang), d * np.sin(ang), 0.0]
        pos[5:8] = [[20, 25, 0], [35, 35, 0], [45, 42, 0]]
        md["posFinal"] = (pos + np.random.normal(0, 0.3, pos.shape)).ravel().astype(np.float32)
        md["posNet"] = (pos + np.random.normal(0, 1.2, pos.shape)).ravel().astype(np.float32)
        md["netDeltaMm"] = np.linalg.norm(
            md["posFinal"].reshape(20, 3) - md["posNet"].reshape(20, 3), axis=1).astype(np.float32)
        md["source"] = np.ones(20, dtype=np.uint8)      # 全部实测 -> 真值可信
        md["observed"] = np.ones(20, dtype=np.uint8)
        md["conf"] = np.full(20, 0.95, dtype=np.float32)
        md["missLogit"] = np.full(20, -3.0, dtype=np.float32)
        md["sourcePointId"] = np.arange(20, dtype=np.int32)
        buf += chunk(pcrec.CT_MARKERDBG, rec_bytes(pcrec.MARKERDBG, md))

        # ---- JointOut：屈曲角随 uu 增大（正确）或减小（mirror） ----
        jo = np.zeros(1, dtype=pcrec.JOINTOUT)[0]
        jo["wallNs"] = ts
        jo["frameTsNs"] = ts
        q = np.zeros(16, dtype=np.float64)
        sgn = -1.0 if mirror else 1.0
        for i in pcrec.FLEX_IDX:
            q[i] = sgn * (0.05 + 1.30 * uu) + np.random.normal(0, 0.004)
        for i in pcrec.ABD_IDX:
            q[i] = 0.20 + np.random.normal(0, 0.006)     # 外展基本不动
        jo["qSolve"] = q.astype(np.float32)
        jo["qSmooth"] = q.astype(np.float32)
        if rom_dead:
            # ROM 区间塌掉 -> 映射后恒等于行程端点
            jo["qRom"] = np.full(16, 0.9, dtype=np.float32)
            jo["qOut"] = np.full(16, 0.9, dtype=np.float32)
            jo["romLo"] = np.full(16, 0.5, dtype=np.float32)
            jo["romHi"] = np.full(16, 0.51, dtype=np.float32)
        else:
            jo["qRom"] = q.astype(np.float32)
            jo["qOut"] = q.astype(np.float32)
            jo["romLo"] = np.full(16, -0.1, dtype=np.float32)
            jo["romHi"] = np.full(16, 1.5, dtype=np.float32)
        jo["romReady"] = 1
        jo["romSamples"] = 900
        jo["romCoverage"] = 0.72
        jo["romAbdCoverage"] = 0.21
        jo["rateLimitOn"] = 1
        jo["rateLimitRadPerSec"] = 8.0
        jo["angSmoothOn"] = 1
        jo["angSmoothAlpha"] = 0.5
        jo["dtSec"] = 1.0 / 120.0
        jo["fingerValid"] = np.ones(5, dtype=np.uint8)
        jo["mcpValid"] = 1
        jo["wristValid"] = 1
        jo["quatOutOn"] = 1
        jo["segSource"] = np.full(16, 2, dtype=np.uint8)
        jo["maxStageDeltaRad"] = float(np.max(np.abs(jo["qOut"] - jo["qSolve"])))
        buf += chunk(pcrec.CT_JOINTOUT, rec_bytes(pcrec.JOINTOUT, jo))

        # ---- Handedness ----
        h = np.zeros(1, dtype=pcrec.HANDEDNESS)[0]
        h["wallNs"] = ts
        h["frameTsNs"] = ts
        h["panelIsRight"] = 1
        h["autoIsRightDetected"] = 1
        h["autoIsRight"] = 1
        h["aiIsRight"] = 1
        h["tmplIsRight"] = 0 if tmpl_flip else 1
        h["ikIsRight"] = 0 if tmpl_flip else 1
        h["autoKnown"] = 1
        h["autoAgree"] = 1
        h["aiHas"] = 1
        h["aiKnown"] = 0
        h["handSignMm"] = -12.5
        h["handSignLatMm"] = 8.2
        h["handednessMinMm"] = 4.0
        h["aiHandLogit"] = 0.04
        h["geoHandSign"] = 1
        h["cfgDetectHandedness"] = 1
        h["cfgJointMirrorAuto"] = 1
        h["jointMirrorActive"] = 1 if mirror else 0
        h["autoCalibOn"] = 1
        buf += chunk(pcrec.CT_HANDEDNESS, rec_bytes(pcrec.HANDEDNESS, h))

        # ---- RunFlags ----
        rf = np.zeros(1, dtype=pcrec.RUNFLAGS)[0]
        rf["wallNs"] = ts
        rf["frameTsNs"] = ts
        bits = 0
        for name in ["backendReady", "ikEnabled", "chainContinue", "filterOn",
                     "rateLimitOn", "romReady", "quatOutOn", "autoCalibOn",
                     "dorsumRigid", "captureDebug", "jointMirrorAuto",
                     "showSkeleton", "recording"]:
            bits |= 1 << pcrec.RUNFLAG_NAMES.index(name)
        if mirror:
            bits |= 1 << pcrec.RUNFLAG_NAMES.index("jointMirrorNow")
        rf["bits"] = bits
        rf["latencyMs"] = 6.5
        rf["thumbRollUiDeg"] = 80.0
        rf["occludedJumpGateMm"] = 40.0
        buf += chunk(pcrec.CT_RUNFLAGS, rec_bytes(pcrec.RUNFLAGS, rf))

    buf += chunk(pcrec.CT_TRAILER, struct.pack("<qQQQ", 0, nframes * 4, 0, len(buf)))
    return bytes(buf)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--case", default="ok",
                    choices=["ok", "mirror", "rom_dead", "tmpl_flip"])
    a = ap.parse_args()
    np.random.seed(7)
    open(a.out, "wb").write(build(a.case))
    print(f"写出 {a.out}  case={a.case}")
