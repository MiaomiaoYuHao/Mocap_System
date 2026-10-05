#!/usr/bin/env python3
"""
hm20 推理延迟一键实测（独立包，跟项目文件无关）
==========================================================================
在【你实际部署的那台机器】上跑：

    python bench_all.py

它会依次：导出几个规模的模型 -> 校验 ONNX 与 PyTorch 数值一致 ->
在 CPU / CUDA 上各测一遍延迟 -> 打一张表告诉你哪个规模能上线。

权重是随机的 —— 延迟只取决于网络结构和输入形状，跟训没训过无关，
所以不需要等训练完就能定"模型能开多大"。

可选参数：
    --points 22       现场典型候选点数（你说杂点几乎没有，那就是 20 出头）
    --threads 4       CPU 线程数
    --iters 300       每组测多少次
    --only 384x12     只测某一个规模
"""
from __future__ import annotations
import argparse, os, sys, time

try:
    import numpy as np, torch
except ImportError:
    sys.exit("需要 numpy 和 torch：pip install torch numpy")
from net import ExportWrapper, MarkerLabelNet, NUM_MARKERS

CONFIGS = [("256x8", 256, 8, 8, 96), ("320x10", 320, 10, 10, 128),
           ("384x12", 384, 12, 12, 128), ("448x12", 448, 12, 8, 128)]


def build_and_export(dim, depth, heads, prank, path, trace_n=14):
    m = MarkerLabelNet(dim=dim, depth=depth, heads=heads, knn=8, edge_emb=48,
                       sinkhorn_iters=50, pos_rank=prank).eval()
    args = (torch.zeros(1, trace_n, 3), torch.ones(1, trace_n),
            torch.zeros(1, m.cond[0].in_features - 1), torch.ones(1, 1),
            torch.zeros(1, NUM_MARKERS, 3), torch.ones(1, NUM_MARKERS))
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        torch.onnx.export(
            ExportWrapper(m, sinkhorn=False).eval(), args, path,
            input_names=["points", "mask", "tmpl", "tmpl_valid", "prev", "prev_mask"],
            output_names=["log_assign", "pos", "center", "scale"],
            dynamic_axes={"points": {1: "N"}, "mask": {1: "N"}, "log_assign": {1: "N1"}},
            opset_version=17, do_constant_folding=True)
    return m, sum(p.numel() for p in m.parameters())


def make_feed(N, rng):
    nv = max(4, N - 2)
    return {"points": (rng.normal(size=(1, N, 3)) * 60).astype(np.float32),
            "mask": np.concatenate([np.ones((1, nv)), np.zeros((1, N - nv))], 1).astype(np.float32),
            "tmpl": rng.normal(size=(1, 61)).astype(np.float32),
            "tmpl_valid": np.ones((1, 1), np.float32),
            "prev": (rng.normal(size=(1, NUM_MARKERS, 3)) * 60).astype(np.float32),
            "prev_mask": np.ones((1, NUM_MARKERS), np.float32)}


def verify(m, path, ort):
    w = ExportWrapper(m, sinkhorn=False).eval()
    sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(0)
    worst = 0.0
    for N in (9, 14, 22, 30):
        f = make_feed(N, rng)
        with torch.no_grad():
            ref = w(*[torch.from_numpy(f[k]) for k in
                      ("points", "mask", "tmpl", "tmpl_valid", "prev", "prev_mask")])
        got = sess.run(None, f)
        worst = max(worst, float(np.abs(ref[0].numpy() - got[0]).max()),
                    float(np.abs(ref[1].numpy() - got[1]).max()))
    return worst


def bench(path, provider, N, threads, iters, ort):
    so = ort.SessionOptions()
    so.intra_op_num_threads = threads
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    prov = "CUDAExecutionProvider" if provider == "cuda" else "CPUExecutionProvider"
    if prov not in ort.get_available_providers():
        return None
    try:
        sess = ort.InferenceSession(path, so, providers=[prov])
    except Exception:
        return None
    f = make_feed(N, np.random.default_rng(1))
    for _ in range(30):
        sess.run(None, f)
    ts = []
    for _ in range(iters):
        t0 = time.perf_counter(); sess.run(None, f); ts.append((time.perf_counter()-t0)*1e3)
    return np.array(ts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--points", type=int, default=22)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--iters", type=int, default=300)
    ap.add_argument("--only", default=None)
    a = ap.parse_args()

    try:
        import onnxruntime as ort
    except ImportError:
        sys.exit("需要 onnxruntime：pip install onnxruntime   （有 N 卡可装 onnxruntime-gpu）")

    print(f"torch {torch.__version__} | onnxruntime {ort.__version__}")
    print(f"可用 provider: {ort.get_available_providers()}")
    print(f"候选点数 {a.points} | CPU 线程 {a.threads} | 每组 {a.iters} 次\n")

    rows = []
    for name, dim, depth, heads, prank in CONFIGS:
        if a.only and a.only != name:
            continue
        path = f"_bench_{name}.onnx"
        print(f"--- dim{dim} depth{depth} 导出中 ...", flush=True)
        m, npar = build_and_export(dim, depth, heads, prank, path)
        d = verify(m, path, ort)
        ok = "OK" if d < 1e-3 else f"不一致 {d:.1e}"
        r = {"name": name, "par": npar, "verify": ok}
        for prov in ("cpu", "cuda"):
            t = bench(path, prov, a.points, a.threads, a.iters, ort)
            r[prov] = None if t is None else (float(np.median(t)), float(np.percentile(t, 95)))
        rows.append(r)
        os.remove(path)

    print("\n" + "=" * 78)
    print(f"{'配置':<10}{'参数量':<9}{'校验':<9}{'CPU 中位/p95':<20}{'CUDA 中位/p95':<20}")
    print("-" * 78)
    for r in rows:
        def fmt(v):
            return "不可用" if v is None else f"{v[0]:.2f} / {v[1]:.2f} ms"
        print(f"{r['name']:<10}{r['par']/1e6:<9.1f}{r['verify']:<9}"
              f"{fmt(r['cpu']):<20}{fmt(r['cuda']):<20}")
    print("=" * 78)
    print("\n【怎么读】p95 = 95% 的帧比这个快。实时系统看 p95，不看平均。")
    print("帧预算 = 1000/fps 毫秒，推理只应占其中一部分（检测、三角化、IK、渲染都要分）。")
    print("参考判据（假设推理占 30% 预算）：200fps -> p95<=1.5ms；100fps -> p95<=3ms")
    for r in rows:
        best = None
        for k in ("cuda", "cpu"):
            if r[k] and (best is None or r[k][1] < best[1][1]):
                best = (k, r[k])
        if best:
            p95 = best[1][1]
            v = "可上 200fps" if p95 <= 1.5 else "可上 100fps" if p95 <= 3 else \
                f"上限约 {1000/p95*0.3:.0f}fps（按占 30% 预算）"
            print(f"  {r['name']:<8} 最快 {best[0]:<5} p95 {p95:6.2f} ms  ->  {v}")


if __name__ == "__main__":
    main()
