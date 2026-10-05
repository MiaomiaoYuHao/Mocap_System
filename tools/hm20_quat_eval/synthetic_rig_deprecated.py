"""
hm20 手部 20 点刚体/关节模型 —— 为 mocap_stress 压测台提供带真值姿态的手。

现有压测台的"标记点局部构型"是 marker_spread_mm 撒出来的随机刚体点云，
没有关节、没有骨骼，因此无法评估手指段四元数。本模块替换掉那一段。

点位定义（与 hm20 标签一致）：
    0..4    手背 5 点（dorsum），用于解腕部位姿
    5..19   手指 15 点，每指 3 点 = 近节/中节/远节 的**指节背侧**贴球
            顺序：thumb, index, middle, ring, pinky
            每指内顺序：proximal, middle, distal

关键建模点（对应你总结第二节）：
    球贴在指节背侧、离骨轴 11~17mm，且相邻两节的偏置方向不同。
    这是系统偏差的来源，标签 100% 正确也消不掉。
"""
import numpy as np

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
SEGS = ["prox", "mid", "dist"]

# 骨长 mm（成年男性中位，thumb 用 MC/PP/DP 近似三节）
BONE_LEN = {
    "thumb":  [46.0, 31.6, 21.7],
    "index":  [44.6, 26.3, 17.4],
    "middle": [49.7, 31.8, 19.8],
    "ring":   [45.5, 28.8, 18.8],
    "pinky":  [36.6, 21.0, 15.8],
}

# MCP 关节在手背坐标系中的位置 mm（x 向指尖，y 向拇指侧，z 向手背外法线）
MCP_POS = {
    "thumb":  np.array([12.0,  38.0, -6.0]),
    "index":  np.array([88.0,  22.0,  0.0]),
    "middle": np.array([92.0,   3.0,  1.0]),
    "ring":   np.array([88.0, -16.0,  0.5]),
    "pinky":  np.array([80.0, -34.0, -1.5]),
}

# 手背 5 个 marker 在手背坐标系中的位置 mm
DORSUM_MARKERS = np.array([
    [ 18.0,   2.0, 14.0],   # 腕心偏近端
    [ 62.0,  26.0, 16.0],   # 第二掌骨
    [ 66.0, -28.0, 15.0],   # 第五掌骨
    [ 30.0, -22.0, 13.0],   # 尺侧
    [ 34.0,  24.0, 15.0],   # 桡侧
])

# 每指每节的球心相对骨轴的偏置：(沿骨轴比例, 背侧距离mm, 侧向角度deg)
# 背侧距离 11~17mm，侧向角逐节不同 —— 这是骨轴偏差的直接来源
MARKER_OFFSET = {
    "thumb":  [(0.55, 13.0,  -18.0), (0.60, 12.0,   8.0), (0.55, 11.0,  22.0)],
    "index":  [(0.58, 15.0,   10.0), (0.60, 13.5, -12.0), (0.52, 11.5,   6.0)],
    "middle": [(0.58, 16.5,    4.0), (0.60, 14.0,  14.0), (0.52, 12.0, -10.0)],
    "ring":   [(0.58, 15.5,   -8.0), (0.60, 13.0,   6.0), (0.52, 11.5,  16.0)],
    "pinky":  [(0.58, 14.0,   12.0), (0.60, 12.0, -14.0), (0.52, 11.0,   4.0)],
}


def rodrigues(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def mat2quat(R):
    """返回 (w,x,y,z)，w>=0 规范化。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def quat_angle_deg(q1, q2):
    """两个四元数之间的旋转角（度），已处理双覆盖。"""
    d = abs(float(np.dot(q1, q2)))
    d = min(1.0, max(-1.0, d))
    return float(np.degrees(2.0 * np.arccos(d)))


def axis_angle_deg(a, b):
    """两个方向向量夹角（度）。"""
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    c = float(np.dot(a, b) / (na * nb))
    return float(np.degrees(np.arccos(min(1.0, max(-1.0, c)))))


class HandRig:
    """20 点手部关节模型。给定关节角 -> 世界系 marker 位置 + 每段真值四元数。"""

    N_MARKERS = 20

    def __init__(self, handed="right"):
        self.handed = handed
        self.sign = 1.0 if handed == "right" else -1.0

    # ---- 关节角动画：屈伸 + 外展，各指相位不同 ----
    def joint_angles(self, t, motion="grasp"):
        """返回 dict[finger] = [(flex_deg, abd_deg) x3]，近/中/远节。"""
        out = {}
        for fi, f in enumerate(FINGERS):
            ph = fi * 0.6
            if motion == "grasp":
                # 抓握循环：0 -> 全屈 -> 0
                u = 0.5 * (1 - np.cos(2 * np.pi * 0.35 * t + ph))
            elif motion == "wave":
                u = 0.5 * (1 + np.sin(2 * np.pi * 0.8 * t + ph))
            elif motion == "static":
                u = 0.25
            else:
                u = 0.5 * (1 - np.cos(2 * np.pi * 0.35 * t + ph))
            if f == "thumb":
                flex = [u * 45, u * 40, u * 55]
                abd = [12 - u * 20, 0, 0]
            else:
                flex = [u * 80, u * 95, u * 70]
                abd = [(fi - 2) * 6 * (1 - u) + 3 * np.sin(1.3 * t + ph), 0, 0]
            out[f] = list(zip(flex, abd))
        return out

    # ---- 手掌整体位姿：沿用压测台的 8 字轨迹 ----
    def palm_pose(self, t, speed_mm_s=700.0, amp_mm=300.0):
        w = speed_mm_s / max(1.0, amp_mm)
        T = np.array([amp_mm * np.sin(w * t),
                      amp_mm * np.sin(2 * w * t) * 0.6,
                      120.0 * np.sin(w * t * 0.7)])
        R = rodrigues(np.array([0.6 * np.sin(w * t * 0.8),
                                0.4 * np.cos(w * t * 0.5),
                                w * t * 0.3]))
        return R, T

    def forward(self, t, motion="grasp", speed_mm_s=700.0, amp_mm=300.0):
        """
        返回:
          P    (20,3) 世界系 marker 位置 mm
          Qgt  dict[(finger,seg)] = 真值段四元数 (w,x,y,z) 世界系
          Qpalm 手背整体真值四元数
          Bone dict[(finger,seg)] = (骨近端点, 骨远端点) 世界系
        """
        Rp, Tp = self.palm_pose(t, speed_mm_s, amp_mm)
        ang = self.joint_angles(t, motion)

        P = np.zeros((self.N_MARKERS, 3))
        Qgt, Bone = {}, {}

        # 手背 5 点
        for i in range(5):
            L = DORSUM_MARKERS[i].copy()
            L[1] *= self.sign
            P[i] = Rp @ L + Tp
        Qpalm = mat2quat(Rp)

        idx = 5
        for f in FINGERS:
            base = MCP_POS[f].copy()
            base[1] *= self.sign
            # 段坐标系初值 = 手掌系
            Rseg = Rp.copy()
            origin = Rp @ base + Tp
            for s in range(3):
                flex, abd = ang[f][s]
                # 屈曲绕 -y（掌屈），外展绕 z
                Rj = Rseg @ rodrigues(np.array([0, -np.radians(flex) * (1 if s else 1), 0])) \
                          @ rodrigues(np.array([0, 0, np.radians(abd) * self.sign]))
                Rseg = Rj
                L = BONE_LEN[f][s]
                tip = origin + Rseg @ np.array([L, 0, 0])

                Qgt[(f, SEGS[s])] = mat2quat(Rseg)
                Bone[(f, SEGS[s])] = (origin.copy(), tip.copy())

                # marker：沿骨轴 frac 处，向背侧偏 d，再绕骨轴转 side 角
                frac, d, side = MARKER_OFFSET[f][s]
                a = np.radians(side) * self.sign
                local_off = np.array([0.0, -np.sin(a) * d * self.sign, np.cos(a) * d])
                P[idx] = origin + Rseg @ (np.array([L * frac, 0, 0]) + local_off)
                idx += 1
                origin = tip

        return P, Qgt, Qpalm, Bone
