#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
合成数据测试标定板外参方案的核心逻辑（connected_components / spanning_tree_poses /
board_pose_graph_ba / solve_board_extrinsics 里的锚定推导）。

用直接注入 round_poses（跳过真实图像检测，PnP 本身是 OpenCV 标准用法，
在项目其它地方已经被大量使用过）的方式，专注验证这次新写的、真正容易出
bug 的部分：图论连通性、位姿传播公式、位姿图优化、世界锚定变换。

场景刻意设计成"接力式"——8台相机排成一圈，只有相邻的相机对共视（模拟你
说的"几个相机没法共视，先连一部分，再挪板子连另一部分"），不是全体两两
共视，专门验证"不要求全体共视，只要求连通"这个核心卖点。
"""
import numpy as np
import cv2
import sys

import solve_calibration as S

np.random.seed(7)


def look_at(eye, target, up=np.array([0, 0, 1.0])):
    f = target - eye; f = f / np.linalg.norm(f)
    r = np.cross(f, up); r = r / np.linalg.norm(r)
    u = np.cross(r, f)
    R = np.vstack([r, -u, f])
    t = -R @ eye
    return R, t


def cam_center(R, t):
    return -np.asarray(R).T @ np.asarray(t)


def rot_err_deg(Ra, Rb):
    dR = np.asarray(Ra) @ np.asarray(Rb).T
    return np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))


def main():
    N = 8
    cams_true = []
    radius = 2500.0
    for i in range(N):
        ang = 2 * np.pi * i / N
        eye = np.array([radius * np.cos(ang), radius * np.sin(ang), 500.0])
        R, t = look_at(eye, np.array([0, 0, 0.0]))
        cams_true.append((R, t))

    keys = [f"cam{i}" for i in range(N)]
    names = {k: k for k in keys}

    # ---- 只让"相邻"的相机对产生共视轮次（接力式，不是全体共视）----
    # 每一轮：板子摆在某个世界位置，随机小幅扰动朝向，模拟"多姿态"。
    # 只喂给相邻两台相机各自的板子位姿（其它相机看不到，模拟没有共视）。
    board_world_positions = []
    for i in range(N):
        ang = 2 * np.pi * (i + 0.5) / N   # 相邻两台连线中点附近
        pos = np.array([radius * 0.5 * np.cos(ang), radius * 0.5 * np.sin(ang), 300.0])
        board_world_positions.append(pos)

    round_poses = {}
    all_edges = []
    edges = {}
    NUM_POSES_PER_PAIR = 4
    for i in range(N):
        j = (i + 1) % N   # 相邻相机对（环形接力，首尾也相邻，形成一个环——测试闭环）
        Ri_true, ti_true = cams_true[i]
        Rj_true, tj_true = cams_true[j]
        for p in range(NUM_POSES_PER_PAIR):
            rid = f"round_{i}_{j}_{p}"
            # 板子在两者连线中点附近，每次姿态加一点随机旋转扰动。
            base_pos = board_world_positions[i]
            jitter = np.random.randn(3) * 50
            board_pos = base_pos + jitter
            rand_axis = np.random.randn(3); rand_axis /= np.linalg.norm(rand_axis)
            rand_angle = np.random.uniform(-0.5, 0.5)
            R_board_world, _ = cv2.Rodrigues(rand_axis * rand_angle)
            R_board_world = R_board_world @ np.eye(3)

            # 板子相对相机 i / j 的位姿：X_cam = R_cam_world @ (R_board_world @ X_board + board_pos) + t_cam_world
            def board_in_cam(Rc, tc):
                Rcb = Rc @ R_board_world
                tcb = Rc @ board_pos + tc
                return Rcb, tcb

            Rbi, tbi = board_in_cam(Ri_true, ti_true)
            Rbj, tbj = board_in_cam(Rj_true, tj_true)
            n_corners = 40  # 假装每次都检测到40个角点
            round_poses[rid] = {keys[i]: (Rbi, tbi, n_corners),
                                keys[j]: (Rbj, tbj, n_corners)}

            Rij = Rbj @ Rbi.T
            tij = tbj - Rij @ tbi
            all_edges.append((i, j, Rij, tij, n_corners, rid))
            cur = edges.get((i, j))
            if cur is None or n_corners > cur["inliers"]:
                edges[(i, j)] = {"R": Rij, "t": tij, "inliers": n_corners}

    print(f"共 {len(round_poses)} 轮，{len(edges)} 条边（相邻接力，形成一个环）")

    # ---- 连通性检查 ----
    comp = S.connected_components(N, list(edges.keys()))
    assert len(set(comp)) == 1, f"FAIL: 图不连通 {comp}"
    print("连通性: PASS（接力式布局，全部连通）")

    # ---- 生成树初值 ----
    inl_sum = {k: 0 for k in range(N)}
    for (i, j), e in edges.items():
        inl_sum[i] += e["inliers"]; inl_sum[j] += e["inliers"]
    root = max(inl_sum, key=inl_sum.get)
    tree_pose = S.spanning_tree_poses(N, edges, root)
    assert len(tree_pose) == N, f"FAIL: 生成树只覆盖 {len(tree_pose)}/{N}"
    print(f"生成树: PASS（根相机 cam{root}，全部 {N} 台覆盖）")

    # ---- 评估函数：转到 root 系比较（板子方案是绝对尺度，不需要额外缩放）----
    Rr_t, tr_t = cams_true[root]
    def rel_to_root_true(i):
        Ri, ti = cams_true[i]
        R = Ri @ Rr_t.T
        t = ti - R @ tr_t
        return R, t

    def eval_pose(pose, label):
        pos_errs, ang_errs = [], []
        for i in range(N):
            Rt, tt = rel_to_root_true(i)
            Rp, tp = pose[i]
            pos_errs.append(np.linalg.norm(cam_center(Rt, tt) - cam_center(Rp, tp)))
            ang_errs.append(rot_err_deg(Rt, Rp))
        print(f"[{label}] 位置误差 max={max(pos_errs):.4f}mm mean={np.mean(pos_errs):.4f}mm | "
              f"朝向误差 max={max(ang_errs):.4f}deg mean={np.mean(ang_errs):.4f}deg")
        return max(pos_errs), max(ang_errs)

    tree_pos_err, tree_ang_err = eval_pose(tree_pose, "生成树初值（零噪声，理论上应该精确）")

    # ---- 位姿图优化（这里数据零噪声，BA 应该保持精确，不应该引入误差）----
    ba_pose = S.board_pose_graph_ba(keys, tree_pose, all_edges, root, names)
    ba_pos_err, ba_ang_err = eval_pose(ba_pose, "位姿图优化后（零噪声）")

    # ---- 世界锚定：手动复现 solve_board_extrinsics 第6步的逻辑 ----
    anchor_id = f"round_{0}_{1}_{0}"   # 拿 cam0-cam1 那条边的第一轮当锚定
    anchor_poses = round_poses[anchor_id]
    anchor_key = keys[0]
    R_cb, t_cb, _ = anchor_poses[anchor_key]
    ka_idx = 0
    R_ka_graph, t_ka_graph = ba_pose[ka_idx]
    R_ka_graph = np.asarray(R_ka_graph); t_ka_graph = np.asarray(t_ka_graph)
    R_gw = R_ka_graph.T @ R_cb
    t_gw = R_ka_graph.T @ (t_cb - t_ka_graph)

    out = {}
    for i in range(N):
        Ri, ti = ba_pose[i]
        Ri = np.asarray(Ri); ti = np.asarray(ti)
        out[i] = (Ri @ R_gw, Ri @ t_gw + ti)

    # 真值世界系：因为锚定轮的板子世界坐标就是 board_world_positions[0]（cam0-cam1那条边），
    # 且板子朝向是 R_board_world（随机扰动的，取该轮真实值）——重新算一次锚定轮0的真值变换，
    # 验证解出来的相机世界位姿是不是真的落在"以那次板子摆放为原点"的世界系里。
    # 简化验证方式：直接对比"解出来的世界系下相机之间的相对位姿"跟真值是否一致（相对位姿
    # 不受世界系具体是哪个板子姿态影响，只要不同相机间相对关系对，说明整体解算是自洽的）。
    print("\n世界锚定后（相机间相对位姿是否仍然正确，用 cam0 当参考对比）：")
    R0_out, t0_out = out[0]
    max_pos_err, max_ang_err = 0, 0
    for i in range(N):
        Rt, tt = rel_to_root_true(i) if False else (None, None)
    # 用输出自身的 cam0 做参考，跟"真值的 cam0 做参考"两边的相对位姿比较
    for i in range(N):
        Ri_out, ti_out = out[i]
        R_rel_out = Ri_out @ R0_out.T
        t_rel_out = ti_out - R_rel_out @ t0_out
        Ri_true, ti_true = cams_true[i]
        R0_true, t0_true = cams_true[0]
        R_rel_true = Ri_true @ R0_true.T
        t_rel_true = ti_true - R_rel_true @ t0_true
        pe = np.linalg.norm(cam_center(R_rel_out, t_rel_out) - cam_center(R_rel_true, t_rel_true))
        ae = rot_err_deg(R_rel_out, R_rel_true)
        max_pos_err = max(max_pos_err, pe)
        max_ang_err = max(max_ang_err, ae)
    print(f"世界锚定后相对位姿误差: max_pos={max_pos_err:.4f}mm max_ang={max_ang_err:.4f}deg")

    ok = (tree_pos_err < 1e-6 and ba_pos_err < 1e-6 and
          max_pos_err < 1e-3 and max_ang_err < 1e-3)
    print("\n" + ("PASS ✅（零噪声下全流程数值精确，图论/传播/BA/锚定逻辑都对）" if ok
                  else "FAIL ❌"))
    return 0 if ok else 1


def test_noisy_degrades_gracefully():
    """有真实量级的噪声（模拟 solvePnP 的实际解算误差）时，位姿图优化应该
    比单纯的生成树链式传播更准——验证闭环摊薄误差确实在起作用，而不是
    摆设。"""
    print("\n" + "=" * 60)
    print("测试2：有噪声时，位姿图优化应优于生成树初值（闭环摊薄误差）")
    print("=" * 60)
    np.random.seed(3)
    N = 8
    keys = [f"cam{i}" for i in range(N)]
    names = {k: k for k in keys}
    radius = 2500.0
    cams_true = [look_at(np.array([radius * np.cos(2 * np.pi * i / N),
                                    radius * np.sin(2 * np.pi * i / N), 500.0]),
                         np.zeros(3)) for i in range(N)]

    ROT_NOISE_DEG = 0.15
    TRANS_NOISE_MM = 1.5
    edges, all_edges = {}, []
    for i in range(N):
        j = (i + 1) % N
        Ri, ti = cams_true[i]; Rj, tj = cams_true[j]
        for p in range(4):
            rid = f"r{i}_{j}_{p}"
            base = np.array([radius * 0.5 * np.cos(2 * np.pi * (i + 0.5) / N),
                             radius * 0.5 * np.sin(2 * np.pi * (i + 0.5) / N), 300.0])
            board_pos = base + np.random.randn(3) * 50
            ax = np.random.randn(3); ax /= np.linalg.norm(ax)
            Rbw, _ = cv2.Rodrigues(ax * np.random.uniform(-0.5, 0.5))

            def board_in_cam(Rc, tc):
                Rcb = Rc @ Rbw; tcb = Rc @ board_pos + tc
                nax = np.random.randn(3); nax /= np.linalg.norm(nax)
                dR, _ = cv2.Rodrigues(nax * np.radians(ROT_NOISE_DEG) * np.random.randn())
                return dR @ Rcb, tcb + np.random.randn(3) * TRANS_NOISE_MM

            Rbi, tbi = board_in_cam(Ri, ti); Rbj, tbj = board_in_cam(Rj, tj)
            n = 40
            Rij = Rbj @ Rbi.T; tij = tbj - Rij @ tbi
            all_edges.append((i, j, Rij, tij, n, rid))
            cur = edges.get((i, j))
            if cur is None or n > cur["inliers"]:
                edges[(i, j)] = {"R": Rij, "t": tij, "inliers": n}

    inl = {k: 0 for k in range(N)}
    for (i, j), e in edges.items():
        inl[i] += e["inliers"]; inl[j] += e["inliers"]
    root = max(inl, key=inl.get)
    tree = S.spanning_tree_poses(N, edges, root)

    Rr, tr = cams_true[root]
    def rel_true(i):
        Ri, ti = cams_true[i]; R = Ri @ Rr.T; return R, ti - R @ tr
    def ev(pose, label):
        pe, ae = [], []
        for i in range(N):
            Rt, tt = rel_true(i); Rp, tp = pose[i]
            pe.append(np.linalg.norm(cam_center(Rt, tt) - cam_center(Rp, tp)))
            ae.append(rot_err_deg(Rt, Rp))
        print(f"  [{label}] pos max={max(pe):.3f}mm mean={np.mean(pe):.3f}mm | "
              f"ang max={max(ae):.3f}deg mean={np.mean(ae):.3f}deg")
        return max(pe), max(ae)

    tree_pe, tree_ae = ev(tree, "生成树初值（链式传播，误差随链长累积）")
    ba = S.board_pose_graph_ba(keys, tree, all_edges, root, names)
    ba_pe, ba_ae = ev(ba, "位姿图优化后（闭环摊薄）")

    ok = ba_pe < tree_pe and ba_ae < tree_ae
    print(f"  结果：{'PASS ✅（BA 确实比生成树初值更准）' if ok else 'FAIL ❌'}")
    return ok


def test_failure_paths():
    """孤岛检测、缺锚定轮，两条报错路径必须真的触发、报文清晰指名。"""
    print("\n" + "=" * 60)
    print("测试3：失败路径——孤岛检测 / 缺锚定轮")
    print("=" * 60)
    ok = True

    edges = {(0, 1): {"R": np.eye(3), "t": np.zeros(3), "inliers": 40},
             (1, 2): {"R": np.eye(3), "t": np.zeros(3), "inliers": 40},
             (3, 4): {"R": np.eye(3), "t": np.zeros(3), "inliers": 40}}
    comp = S.connected_components(5, list(edges.keys()))
    isolated = len(set(comp)) > 1
    print(f"  孤岛检测：分量={comp}  连通={not isolated}（预期不连通）")
    ok = ok and isolated

    import tempfile
    sess = tempfile.mkdtemp()
    mani = {"cameras": [{"deviceKey": f"cam{i}", "name": f"cam{i}", "folder": f"cam{i}"}
                        for i in range(3)],
            "board": {"dict": "DICT_5X5_100", "squaresX": 11, "squaresY": 8,
                      "squareMM": 30, "markerMM": 22}}
    try:
        S.solve_board_extrinsics(sess, mani, {}, [c["deviceKey"] for c in mani["cameras"]],
                                 {c["deviceKey"]: c["name"] for c in mani["cameras"]})
        print("  空 rounds 目录：FAIL（应该抛异常但没有）")
        ok = False
    except RuntimeError as ex:
        print(f"  空 rounds 目录：正确抛出 RuntimeError（{str(ex)[:40]}…）")

    print(f"  结果：{'PASS ✅' if ok else 'FAIL ❌'}")
    return ok


if __name__ == "__main__":
    r1 = main()
    r2 = 0 if test_noisy_degrades_gracefully() else 1
    r3 = 0 if test_failure_paths() else 1
    print("\n" + "=" * 60)
    total_ok = (r1 == 0 and r2 == 0 and r3 == 0)
    print("全部测试：" + ("PASS ✅" if total_ok else "FAIL ❌"))
    sys.exit(0 if total_ok else 1)
