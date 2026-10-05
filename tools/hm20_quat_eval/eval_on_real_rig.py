"""用 tools/skeleton_assoc/hand_rig.py（训练时真正用的手模型）重算分段四元数误差。

之前的评估用的是我另建的近似 rig（球心离骨轴 11~17mm）。真 rig 的 standoff
是 5.3~6.7mm，且带皮肤滑移、per-subject 解剖差异、左右手镜像。这份脚本直接
吃 forward_kinematics 的 seg_R 真值，不再有任何我自己拍的参数。
"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "skeleton_assoc"))
import hand_rig as HR
import pose_prior as PP

N_SEG = 16


def bone_frame_old(d):
    """现状：全局参考向量 + |x_z|<0.9 硬分支"""
    x = d / (np.linalg.norm(d) + 1e-12)
    ref = np.array([0., 0., 1.]) if abs(x[2]) < 0.9 else np.array([0., 1., 0.])
    y = np.cross(ref, x); n = np.linalg.norm(y)
    if n < 1e-12: return np.eye(3)
    y /= n
    return np.column_stack([x, y, np.cross(x, y)])


def bone_frame_new(d, wristR, prevY):
    """补丁后：手背法线，退化时用上一帧 y 轴做 Gram-Schmidt"""
    n = np.linalg.norm(d)
    if n < 1e-12: return np.eye(3)
    x = d / n
    y = np.cross(wristR[:, 2], x); ny = np.linalg.norm(y)
    if ny < 0.15:
        proj = float(prevY @ x)
        yc = prevY - proj * x; nyc = np.linalg.norm(yc)
        if nyc > 1e-6: y, ny = yc, nyc
        else:
            y = np.cross(wristR[:, 0], x); ny = np.linalg.norm(y)
            if ny < 1e-12: return np.eye(3)
    y = y / ny
    return np.column_stack([x, y, np.cross(x, y)])


def dirs_old(P):
    D = np.zeros((N_SEG, 3))
    for f in range(5):
        b = 5 + f*3
        D[1+f*3+0] = P[b+1]-P[b+0]
        D[1+f*3+1] = P[b+1]-P[b+0]      # 现状：与近节完全相同
        D[1+f*3+2] = P[b+2]-P[b+1]
    return D


def dirs_new(P):
    D = np.zeros((N_SEG, 3))
    for f in range(5):
        b = 5 + f*3
        D[1+f*3+0] = P[b+1]-P[b+0]
        D[1+f*3+1] = P[b+2]-P[b+0]      # 补丁：跨两节长基线
        D[1+f*3+2] = P[b+2]-P[b+1]
    return D


def ang(a, b):
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9: return np.nan
    return float(np.degrees(np.arccos(np.clip(a@b/(na*nb), -1, 1))))


def run(n_sub=64, n_frames=240, noise_mm=0.5, seed=0):
    rng = np.random.default_rng(seed)
    sub = HR.sample_subjects(rng, n_sub)
    errO = [[] for _ in range(N_SEG)]
    errN = [[] for _ in range(N_SEG)]
    for t in range(n_frames):
        ang_t = PP.sample_poses(rng, n_sub)
        markers, seg_R, seg_o, joints = HR.forward_kinematics(sub, ang_t)
        M = markers + rng.normal(0, noise_mm, markers.shape)
        for b in range(n_sub):
            Do, Dn = dirs_old(M[b]), dirs_new(M[b])
            for s in range(1, N_SEG):
                gt = seg_R[b, s][:, 0]          # 该段骨轴 = seg_R 第一列
                errO[s].append(ang(Do[s], gt))
                errN[s].append(ang(Dn[s], gt))
    return errO, errN


def st(a):
    a = np.array([x for x in a if np.isfinite(x)])
    return np.percentile(a,50), np.percentile(a,90), a.max()


if __name__ == "__main__":
    import topology
    errO, errN = run()
    print(f"{'segment':16s}{'现状中位':>9s}{'p90':>7s}{'max':>7s}   {'补丁中位':>9s}{'p90':>7s}{'max':>7s}")
    for s in range(1, N_SEG):
        o, n = st(errO[s]), st(errN[s])
        print(f"{topology.SEG_NAMES[s]:16s}{o[0]:8.1f}°{o[1]:6.1f}°{o[2]:6.1f}°   {n[0]:8.1f}°{n[1]:6.1f}°{n[2]:6.1f}°")
    AO = [x for s in range(1,N_SEG) for x in errO[s]]
    AN = [x for s in range(1,N_SEG) for x in errN[s]]
    print("-"*66)
    print(f"合计  现状: 中位 {st(AO)[0]:.1f}°  p90 {st(AO)[1]:.1f}°  max {st(AO)[2]:.1f}°")
    print(f"合计  补丁: 中位 {st(AN)[0]:.1f}°  p90 {st(AN)[1]:.1f}°  max {st(AN)[2]:.1f}°")
