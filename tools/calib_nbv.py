#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定位姿引导（Next-Best-View）。

【解决什么问题】
压测发现：同样的采集协议、同样的帧数，只因为板子位姿抽到哪几个，标定结果
差 2.2~3.2 倍（6 个随机种子，4帧平均下最好 4.42px、最差 14.07px）。
也就是说现在"操作者随便摆、摆够 15 个就算完"的流程，每次标定都在抽奖，
而且重投影误差完全看不出这次抽得好不好（实测 corr 只有 0.564）。

这个方差本身就是可以拿走的收益：如果能稳定选到好位姿集，等于白拿 2~3 倍
精度，零硬件成本、零额外采集时间。

【怎么做】
标准的最优实验设计（D-optimal），不需要机器学习：
  1. 每个已采位姿都对内参贡献一份 Fisher 信息 J^T·J；
  2. 但每加一个位姿也会引入 6 个新的位姿未知量，所以要用 Schur 补把位姿块
     边缘化掉，得到这个位姿【净贡献给内参】的信息量：
         M_i = A^T·A - A^T·B·(B^T·B)^-1·B^T·A
     其中 A = ∂投影/∂内参，B = ∂投影/∂该位姿的 rvec,tvec。
     不做这一步会严重高估正对相机那类位姿的价值——它们看着点很多，
     但提供的信息大部分被自己的位姿未知量吃掉了，这正是平面标定退化的本质。
  3. 候选位姿里挑使 det(ΣM + M_cand) 增长最多的那个，翻译成人话告诉操作者。

【为什么用 D-optimal 而不是 A-optimal】
D（行列式）对应参数联合置信椭球的体积，对"某个方向完全没约束"这种退化
最敏感——而我们要防的恰恰是退化（实测板子近乎正对相机时焦距误差 9.4%，
重投影误差却毫无异常）。A（迹）会被数值大的参数主导，对退化不够灵敏。
"""
import math
import numpy as np
import cv2


# ---------------------------------------------------------------------------
def _jacobian_blocks(objp, rvec, tvec, K, dist):
    """
    返回 (A, B)：A=∂投影/∂内参[fx,fy,cx,cy,k1,k2,p1,p2,k3]，B=∂投影/∂位姿[rvec,tvec]。
    cv2.projectPoints 的 jacobian 列顺序是 [rvec(3), tvec(3), f(2), c(2), dist(n)]。
    """
    _, J = cv2.projectPoints(objp, rvec, tvec, K, dist)
    if J is None or J.shape[1] < 10:
        return None, None
    B = J[:, 0:6]                     # 位姿块
    A = J[:, 6:6 + 9]                 # fx,fy,cx,cy,k1,k2,p1,p2,k3
    if A.shape[1] < 9:                # 畸变参数少于5个时补零，保持维度一致
        A = np.hstack([A, np.zeros((A.shape[0], 9 - A.shape[1]))])
    return A, B


def pose_information(objp, rvec, tvec, K, dist, ridge=1e-9):
    """单个位姿对内参的净信息贡献（已边缘化掉自身位姿未知量）。"""
    A, B = _jacobian_blocks(objp, rvec, tvec, K, dist)
    if A is None:
        return None
    BtB = B.T @ B
    # 位姿块必然满秩(6)，但数值上可能病态——加极小岭项保证可逆
    BtB += np.eye(6) * (ridge * max(1.0, float(np.trace(BtB))))
    try:
        BtB_inv = np.linalg.inv(BtB)
    except np.linalg.LinAlgError:
        return None
    AtB = A.T @ B
    return A.T @ A - AtB @ BtB_inv @ AtB.T


def total_information(objp_list, rvecs, tvecs, K, dist):
    M = np.zeros((9, 9))
    for objp, rv, tv in zip(objp_list, rvecs, tvecs):
        m = pose_information(objp, rv, tv, K, dist)
        if m is not None:
            M += m
    return M


def _logdet_psd(M, ridge_rel=1e-12):
    """数值稳健的 log det（半正定矩阵）。信息不足时返回 -inf。"""
    M = 0.5 * (M + M.T)
    w = np.linalg.eigvalsh(M)
    floor = max(w.max(), 1.0) * ridge_rel
    w = np.maximum(w, floor)
    if (w <= floor).all():
        return -np.inf
    return float(np.sum(np.log(w)))


# ---------------------------------------------------------------------------
def candidate_poses(board_mm, img_size, K, n_grid=3, tilts=(0.0, 0.40, 0.65),
                    yaws=(0.0, 0.40, -0.40), dists=(0.55, 0.75, 1.0)):
    """
    生成候选位姿网格：画面 n_grid×n_grid 个区域 × 倾斜角 × 偏航角 × 距离。
    距离用板宽的倍数表达，跟具体板子尺寸解耦。
    """
    W, H = img_size
    bw, bh = board_mm
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    out = []
    for gi in range(n_grid):
        for gj in range(n_grid):
            u = W * (gi + 0.5) / n_grid
            v = H * (gj + 0.5) / n_grid
            for dm in dists:
                z = bw / dm * 1.0        # 板宽占画面比例越大 z 越小
                z = max(300.0, min(1600.0, z))
                for tl in tilts:
                    for yw in yaws:
                        if abs(tl) < 1e-6 and abs(yw) > 1e-6:
                            continue     # 去重
                        rvec = np.array([tl, yw, 0.0])
                        tvec = np.array([(u - cx) / fx * z - bw / 2,
                                         (v - cy) / fy * z - bh / 2, z])
                        out.append((rvec, tvec, (gi, gj, tl, yw, dm)))
    return out


def describe_pose(meta, n_grid, img_size):
    """把候选位姿翻译成操作者能照做的中文指令。"""
    gi, gj, tl, yw, dm = meta
    col = ["左", "中", "右"] if n_grid == 3 else [str(i + 1) for i in range(n_grid)]
    row = ["上", "中", "下"] if n_grid == 3 else [str(i + 1) for i in range(n_grid)]
    where = f"画面{row[gj]}{col[gi]}"
    if abs(tl) < 0.1 and abs(yw) < 0.1:
        att = "正对相机"
    else:
        parts = []
        if abs(tl) >= 0.1:
            parts.append(f"上下倾斜约{abs(math.degrees(tl)):.0f}°"
                         f"（{'上边压低' if tl > 0 else '上边抬高'}）")
        if abs(yw) >= 0.1:
            parts.append(f"左右转约{abs(math.degrees(yw)):.0f}°"
                         f"（{'右边推远' if yw > 0 else '左边推远'}）")
        att = "，".join(parts)
    size = {0.55: "板子占画面约一半（拿远一点）",
            0.75: "板子占画面约七成",
            1.0: "板子占满画面（凑近一点）"}.get(dm, "")
    return f"把板子放到【{where}】，{att}，{size}"


# ---------------------------------------------------------------------------
def _board_fully_visible(board_mm, rvec, tvec, K, img_size, margin=8.0):
    """候选位姿下板子四角是否都落在画面内。

    【为什么必须查】D-optimal 只看信息量，不知道"这个位姿实际拍不拍得到"。
    它会偏爱又近又偏的极端位姿（那样透视变化最剧烈、信息量最大），但那种
    位姿板子有一大半在画面外，检出的角点反而更少更差。不加这道检查，
    引导会把操作者指挥到一堆拍不全的位置去，结果远差于随便摆。"""
    bw, bh = board_mm
    corners = np.array([[0, 0, 0], [bw, 0, 0], [bw, bh, 0], [0, bh, 0]], float)
    R, _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3))
    t = np.asarray(tvec, float).reshape(3)
    cam = (R @ corners.T).T + t
    if np.any(cam[:, 2] < 150.0):
        return False
    u = K[0, 0] * cam[:, 0] / cam[:, 2] + K[0, 2]
    v = K[1, 1] * cam[:, 1] / cam[:, 2] + K[1, 2]
    W, H = img_size
    return bool(np.all(u > margin) and np.all(u < W - margin) and
                np.all(v > margin) and np.all(v < H - margin))


def suggest_next_pose(objp_list, rvecs, tvecs, K, dist, board_mm, img_size,
                      n_grid=3, top_k=3, exclude_keys=None):
    """
    给出接下来最该补的几个位姿。
    返回 [(增益, 人话描述, (rvec,tvec), key), ...]，按增益从大到小。
    增益 = 加上这个位姿后 log det(信息矩阵) 的增量，越大越值得补。

    exclude_keys：已经采过的候选键集合，必须传！
    【血的教训】不传的话贪心会每次都挑中同一个位姿——加一个位姿对信息矩阵
    的改变很小，最优候选基本不变，于是连续 9 次都指向同一个地方，最后得到
    9 张几乎一样的图。实测这样做出来的标定几何误差 40~92px，比"随便摆"
    的 3~9px 还差一个数量级。位姿引导的价值全在【多样性】上，不在单点最优。
    """
    exclude_keys = exclude_keys or set()
    M0 = total_information(objp_list, rvecs, tvecs, K, dist)
    base = _logdet_psd(M0)
    if not np.isfinite(base):
        base = None   # 信息还不够，任何位姿都是有益的，直接按绝对值排

    # 用第一个位姿的板点当候选位姿的板点（同一块板，objp 都一样）
    objp = objp_list[0] if objp_list else None
    if objp is None:
        return []

    scored = []
    for rvec, tvec, meta in candidate_poses(board_mm, img_size, K, n_grid=n_grid):
        if meta in exclude_keys:
            continue
        if not _board_fully_visible(board_mm, rvec, tvec, K, img_size):
            continue
        m = pose_information(objp, rvec, tvec, K, dist)
        if m is None:
            continue
        ld = _logdet_psd(M0 + m)
        if not np.isfinite(ld):
            continue
        gain = ld - base if base is not None else ld
        scored.append((gain, meta, rvec, tvec))
    scored.sort(key=lambda x: -x[0])

    # 去重：同一个画面区域只保留最好的一个，避免建议里三条都在同一个角落
    seen, out = set(), []
    for gain, meta, rvec, tvec in scored:
        key = (meta[0], meta[1])
        if key in seen:
            continue
        seen.add(key)
        out.append((gain, describe_pose(meta, n_grid, img_size), (rvec, tvec), meta))
        if len(out) >= top_k:
            break
    return out


def coverage_report(rvecs, tvecs, K, img_size, n_grid=3):
    """
    位姿覆盖诊断：画面分区覆盖 + 姿态角张开度。
    返回 (人话诊断列表, 是否存在退化风险)。
    """
    W, H = img_size
    msgs, risky = [], False
    if not rvecs:
        return ["没有可用位姿"], True

    # 板中心落在画面哪个分区
    hit = np.zeros((n_grid, n_grid), int)
    for tv in tvecs:
        z = float(tv.ravel()[2])
        if z <= 1e-6:
            continue
        u = K[0, 0] * float(tv.ravel()[0]) / z + K[0, 2]
        v = K[1, 1] * float(tv.ravel()[1]) / z + K[1, 2]
        gi = min(n_grid - 1, max(0, int(u / W * n_grid)))
        gj = min(n_grid - 1, max(0, int(v / H * n_grid)))
        hit[gi, gj] += 1
    empty = int((hit == 0).sum())
    if empty:
        msgs.append(f"画面 {n_grid*n_grid} 个分区里有 {empty} 个从没放过板子"
                    f"——边角区域没有数据，畸变系数主要靠边缘约束，会明显偏。")
        if empty >= n_grid * n_grid // 2:
            risky = True

    # 姿态张开度：板法线相对平均法线的张角
    normals = np.array([cv2.Rodrigues(rv)[0][:, 2] for rv in rvecs])
    mean_n = normals.mean(0)
    mean_n /= (np.linalg.norm(mean_n) + 1e-12)
    ang = np.degrees(np.arccos(np.clip(normals @ mean_n, -1, 1)))
    spread = float(np.percentile(ang, 90))
    if spread < 12.0:
        msgs.append(f"板子姿态过于单一（法线张角仅 {spread:.1f}°）——这是平面标定的"
                    f"退化构型，焦距和距离在数学上分不开，实测焦距误差可达 9%，"
                    f"而重投影误差完全不报警。")
        risky = True
    elif spread < 20.0:
        msgs.append(f"板子姿态多样性偏低（法线张角 {spread:.1f}°，建议 >20°）。")

    # 距离多样性
    zs = np.array([float(tv.ravel()[2]) for tv in tvecs])
    if zs.max() / max(1e-6, zs.min()) < 1.35:
        msgs.append(f"所有位姿距离几乎一样（{zs.min():.0f}~{zs.max():.0f}mm）——"
                    f"远近变化能有效约束焦距，建议拉开到 1.5 倍以上。")

    if not msgs:
        msgs.append("位姿覆盖良好。")
    return msgs, risky
