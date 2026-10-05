#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe_thumb_axis.py —— 量一件事：hm20 模型学到的拇指屈曲平面，跟真人差多少。

用法:
    pip install onnxruntime numpy
    python probe_thumb_axis.py hm20_v7.onnx

为什么要量这个
--------------
拇指"外翻/不往手心弯"如果是模型先验的问题，那么无论怎么调 thumbPronationRad /
thumbRollOffsetRad 都只是在给错先验打补丁。这个脚本直接把模型逼到它自己学到的
手型流形上，读它的 seg_rot6d 头，算每根手指相邻两段的相对旋转轴（= 屈曲铰链轴），
再看拇指的铰链轴跟四指差几度。

    真人：拇指铰链轴跟四指差 80~90°（这就是"对掌"）
    如果模型给出的远小于 80°，说明它把拇指当成了第五根手指。

方法
----
1. 随机点云 -> 模型的 pos 头 -> 拿输出当新输入，迭代 8 次。
   pos 头是端到端训出来的，这个不动点会落在模型学到的手型流形上
   （脚本会打印骨间距/手背两两距离让你确认它确实像只手）。
2. 读 seg_rot6d (1,16,6)，6D -> 旋转矩阵。
   seg_rot6d[0] 恒为单位阵，说明它是【手掌规范系】下的 16 段姿态，
   跟世界摆放无关（脚本会顺带验一次这个不变性）。
3. 相邻两段 A、B 的相对旋转 A^T B 的转轴，就是那一节的屈曲铰链轴。

这个脚本不需要相机、不需要标定、不需要你的手，只需要 .onnx 文件。
"""
import sys
import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    sys.exit("需要 onnxruntime:  pip install onnxruntime")

MODEL = sys.argv[1] if len(sys.argv) > 1 else "hm20_v7.onnx"
FN = ["拇指", "食指", "中指", "无名", "小指"]


def rot6d_to_R(v):
    a1, a2 = v[..., :3], v[..., 3:]
    b1 = a1 / np.linalg.norm(a1, axis=-1, keepdims=True)
    b2 = a2 - (b1 * a2).sum(-1, keepdims=True) * b1
    b2 = b2 / np.linalg.norm(b2, axis=-1, keepdims=True)
    return np.stack([b1, b2, np.cross(b1, b2)], axis=-1)   # 列 = 基向量


def main():
    s = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
    names = [o.name for o in s.get_outputs()]
    if "seg_rot6d" not in names:
        sys.exit("这个模型没有 seg_rot6d 输出（多半是 v6）。换 v7 再跑。")

    tmpl = np.zeros((1, 61), np.float32)
    tmpl[0, 60] = 1.0                       # 右手；左手填 -1，结论是镜像的
    tv = np.zeros((1, 20), np.float32)      # 不给模板，只看模型的纯几何先验

    def run(P):
        n = len(P)
        outs = s.run(None, {
            "points": P[None].astype(np.float32),
            "mask": np.ones((1, n), np.float32),
            "tmpl": tmpl, "tmpl_valid": tv,
            "prev": np.zeros((1, 20, 3), np.float32),
            "prev_mask": np.zeros((1, 20), np.float32)})
        return dict(zip(names, outs))

    # ---- ① 先验一次：seg_rot6d 到底在哪个系里 ----
    rng = np.random.default_rng(11)
    P = rng.normal(0, 45, (20, 3)).astype(np.float32)
    for _ in range(8):
        P = run(P)["pos"][0].copy()
    o1 = run(P)
    th = 0.7
    Rw = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1.0]])
    o2 = run(P @ Rw.T + np.array([137., -42., 89.]))
    S1, S2 = rot6d_to_R(o1["seg_rot6d"][0]), rot6d_to_R(o2["seg_rot6d"][0])
    d_same = np.degrees(np.arccos(np.clip((np.trace(S1[0].T @ S2[0]) - 1) / 2, -1, 1)))
    d_eq = np.degrees(np.arccos(np.clip((np.trace((Rw @ S1[0]).T @ S2[0]) - 1) / 2, -1, 1)))
    print("① seg_rot6d 的坐标系")
    print(f"   世界系整体旋转后，seg0 变化 {d_same:.2f}°；若按世界系解读则差 {d_eq:.2f}°")
    print(f"   -> {'手掌规范系（旋转不变）' if d_same < 5 else '世界系'}；"
          f"seg0 与单位阵夹角 "
          f"{np.degrees(np.arccos(np.clip((np.trace(S1[0]) - 1) / 2, -1, 1))):.2f}°")
    print()

    # ---- ② 屈曲铰链轴 ----
    AX = {f: [] for f in range(5)}
    last = None
    for _ in range(60):
        P = rng.normal(0, 45, (20, 3)).astype(np.float32)
        for _ in range(8):
            P = run(P)["pos"][0].copy()
        o = run(P)
        last = P
        S = rot6d_to_R(o["seg_rot6d"][0])
        for f in range(5):
            for j in range(2):
                A, B = S[1 + 3 * f + j], S[1 + 3 * f + j + 1]
                Rr = A.T @ B
                ang = np.arccos(np.clip((np.trace(Rr) - 1) / 2, -1, 1))
                if ang < 0.08:
                    continue
                w = np.array([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0],
                              Rr[1, 0] - Rr[0, 1]]) / (2 * np.sin(ang))
                if w[1] < 0:
                    w = -w                       # 统一半球，只关心轴不关心正负
                AX[f].append(w)

    print("② 屈曲铰链轴（手掌规范系：+X 远端 / +Y 桡侧 / +Z 手背外法向）")
    print(f"   {'手指':<6}{'n':>5}{'平均轴':>28}{'与四指轴夹角':>14}{'离散度':>9}")
    ref = np.array([0., 1., 0.])
    thumb_ang = None
    for f in range(5):
        A = np.array(AX[f])
        m = A.mean(0); m /= np.linalg.norm(m)
        sp = np.degrees(np.arccos(np.clip(A @ m, -1, 1))).std()
        a = np.degrees(np.arccos(np.clip(m @ ref, -1, 1)))
        if f == 0:
            thumb_ang = a
        print(f"   {FN[f]:<6}{len(A):>5}{np.array2string(np.round(m,3)):>28}{a:>13.1f}°{sp:>8.1f}°")

    print()
    d = np.linalg.norm(last[[6, 7, 9, 10, 12, 13]] - last[[5, 6, 8, 9, 11, 12]], axis=1)
    print(f"   自检：收敛样本像不像手？骨间距 {np.round(d,1)} mm（正常 15~40）")
    dd = np.linalg.norm(last[:5, None] - last[None, :5], axis=-1)[np.triu_indices(5, 1)]
    print(f"                       手背两两 {np.round(dd,1)} mm（正常 20~55）")
    print()
    print("③ 结论")
    print(f"   模型的拇指铰链轴与四指差 {thumb_ang:.1f}°，真人应该是 80~90°。")
    print(f"   缺口约 {90 - thumb_ang:.0f}°，这就是需要补的第一掌骨常数旋前。")
    print("   缺口 > 50° => 拇指外翻是【模型先验】的问题，靠调补偿参数只能缓解，")
    print("                 正解是在 markerFK 里补常数旋前 + 用可观测帧标定它。")


if __name__ == "__main__":
    main()
