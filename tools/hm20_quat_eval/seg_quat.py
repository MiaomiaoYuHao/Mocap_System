"""
HandSkeletonAssociator.hpp 里 computeSegmentQuats / boneFrame 的 1:1 Python 移植。
逐行对照 C++，不做任何"顺手修正"——目的是量化现状，不是量化理想。

C++ 原文（546-569 行）：
    static void computeSegmentQuats(SkeletonFrameResult& out) {
        out.segQuat[0] = matToQuat(out.wristR);
        for (int f = 0; f < 5; ++f)
            for (int k = 0; k < 3; ++k) {
                const int m = 5 + f * 3 + k;
                const int a = (k == 0) ? m : (m - 1);
                const int b = (k == 0) ? (m + 1) : m;
                const Vec3 dir = sub(P[b], P[a]);
                out.segQuat[markerSegment(m)] = matToQuat(boneFrame(dir));
            }
    }
    static Mat3 boneFrame(const Vec3& dir) {
        const Vec3 x = normalize(dir);
        const Vec3 ref = (std::abs(x[2]) < 0.9) ? Vec3{0,0,1} : Vec3{0,1,0};
        const Vec3 y = normalize(cross(ref, x));
        const Vec3 z = cross(x, y);
        return {x0,y0,z0, x1,y1,z1, x2,y2,z2};   // 列=基向量
    }
"""
import numpy as np
from hand_rig import mat2quat, FINGERS, SEGS

N_SEG = 16


def bone_frame(dir_vec):
    """1:1 移植 boneFrame。返回 3x3，列 = 基向量。"""
    n = np.linalg.norm(dir_vec)
    if n < 1e-12:
        return np.eye(3)
    x = dir_vec / n
    ref = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
    y = np.cross(ref, x)
    ny = np.linalg.norm(y)
    if ny < 1e-12:
        return np.eye(3)
    y = y / ny
    z = np.cross(x, y)
    return np.column_stack([x, y, z])


def marker_segment(m):
    return 0 if m < 5 else 1 + (m - 5)


def compute_segment_quats(P, wristR):
    """
    P: (20,3) 世界系 marker
    wristR: 3x3
    返回 (16,4) 四元数，以及 (16,3) 每段所用的骨轴方向（seg0 为 None 占位）
    """
    Q = np.zeros((N_SEG, 4))
    D = np.zeros((N_SEG, 3))
    Q[0] = mat2quat(wristR)
    D[0] = wristR[:, 0]
    for f in range(5):
        for k in range(3):
            m = 5 + f * 3 + k
            a = m if k == 0 else (m - 1)
            b = (m + 1) if k == 0 else m
            d = P[b] - P[a]
            s = marker_segment(m)
            Q[s] = mat2quat(bone_frame(d))
            D[s] = d / (np.linalg.norm(d) + 1e-12)
    return Q, D


def ref_switch_flag(dir_unit):
    """boneFrame 里参考向量的分支：True 表示走了 {0,1,0} 分支。"""
    return abs(dir_unit[2]) >= 0.9


def gt_segment_frames(Qgt, Bone):
    """
    真值：每节骨自己的方向（骨轴 = 骨近端->远端）。
    返回 (16,3) 单位方向，index 与 compute_segment_quats 对齐。
    seg 1..3 = thumb mc/pp/dp, 4..6 = index pp/mp/dp, ...
    """
    D = np.zeros((N_SEG, 3))
    for fi, f in enumerate(FINGERS):
        for si, s in enumerate(SEGS):
            o, t = Bone[(f, s)]
            v = t - o
            D[1 + fi * 3 + si] = v / (np.linalg.norm(v) + 1e-12)
    return D


SEG_NAMES = ["wrist"] + [f"{f}_{s}" for f in FINGERS for s in SEGS]
