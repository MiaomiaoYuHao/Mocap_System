#!/usr/bin/env python3
"""
ONNX 推理延迟基准 —— 在【线上那台机器】上跑，决定模型能开多大。

用法：
    python bench_onnx.py hm20_assoc.onnx
    python bench_onnx.py hm20_assoc.onnx --points 24 --threads 4 --provider cpu

为什么要单独测：训练在云端 5090 上跑得多快跟线上没关系。线上是 C++ +
onnxruntime、逐帧调用、100~200fps，决定成败的是【单帧延迟】而不是吞吐。
20 个点的序列极短，GPU 上大部分时间花在 kernel 启动而不是算力，所以
CPU provider 有时反而更快——必须实测，别猜。

判据（按你 100~200fps 的帧率）：
    200fps -> 每帧预算 5.0ms，推理最好 <= 1.5ms（要给三角化/求解/渲染留位置）
    100fps -> 每帧预算 10.0ms，推理最好 <= 3.0ms
"""
from __future__ import annotations

import argparse
import time

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("onnx")
    ap.add_argument("--points", type=int, default=22,
                    help="候选点数（20 个真点 + 若干幽灵点，按你现场实际情况给）")
    ap.add_argument("--threads", type=int, default=4, help="CPU 线程数")
    ap.add_argument("--provider", default="cpu", choices=["cpu", "cuda"])
    ap.add_argument("--warmup", type=int, default=50)
    ap.add_argument("--iters", type=int, default=500)
    a = ap.parse_args()

    import onnxruntime as ort

    so = ort.SessionOptions()
    so.intra_op_num_threads = a.threads
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    prov = ["CUDAExecutionProvider"] if a.provider == "cuda" else ["CPUExecutionProvider"]
    sess = ort.InferenceSession(a.onnx, so, providers=prov)

    N = a.points
    rng = np.random.default_rng(0)
    feed = {
        "points": (rng.normal(size=(1, N, 3)) * 60.0).astype(np.float32),
        "mask": np.ones((1, N), np.float32),
        "tmpl": rng.normal(size=(1, 61)).astype(np.float32),
        "tmpl_valid": np.ones((1, 1), np.float32),
        "prev": (rng.normal(size=(1, 20, 3)) * 60.0).astype(np.float32),
        "prev_mask": np.ones((1, 20), np.float32),
    }
    names = {i.name for i in sess.get_inputs()}
    feed = {k: v for k, v in feed.items() if k in names}
    missing = names - set(feed)
    if missing:
        raise SystemExit(f"ONNX 需要但脚本没提供的输入: {missing}")

    for _ in range(a.warmup):
        sess.run(None, feed)

    ts = []
    for _ in range(a.iters):
        t0 = time.perf_counter()
        sess.run(None, feed)
        ts.append((time.perf_counter() - t0) * 1000.0)
    ts = np.array(ts)

    print(f"provider={a.provider} threads={a.threads} points={N}")
    print(f"  中位 {np.median(ts):.3f} ms | p95 {np.percentile(ts, 95):.3f} ms "
          f"| p99 {np.percentile(ts, 99):.3f} ms | max {ts.max():.3f} ms")
    print(f"  等效帧率上限（只算推理）: {1000.0/np.median(ts):.0f} fps")
    for fps, budget in ((200, 1.5), (100, 3.0)):
        ok = "OK" if np.percentile(ts, 95) <= budget else "偏慢"
        print(f"  {fps}fps 目标(推理预算 {budget}ms): {ok}")


if __name__ == "__main__":
    main()
