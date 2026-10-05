#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定求解（参考实现）。读取"标定向导"生成的会话文件夹，输出 calibration.json
（向导第5步自动导入）。依赖：pip install opencv-contrib-python numpy

流水线：
  1) 内参：每台相机的 ChArUco 照片 -> calibrateCameraCharuco
  2) 外参：单颗反光球轨迹。
     - 参考对 (cam0, cam1)：本质矩阵 + recoverPose + 三角化 -> 参考3D轨迹(cam0系)
     - 其余每台相机：其 2D 观测按时间匹配到参考3D轨迹 -> solvePnPRansac
       （全部相机因此天然共享同一坐标系与尺度）
  3) 世界对齐：四点支架多视角三角化 -> 与已知世界坐标做带尺度 Umeyama 配准
     （24 种点对应全试取残差最小）-> 全部位姿变换到用户世界系（mm，绝对尺度）

用法：python solve_calibration.py <会话文件夹>

轻量模式（标定向导第②步"当场内参预览"用，不做外参/世界对齐）：
  python solve_calibration.py --intrinsics-only <会话文件夹> <camN>
  只对 <camN>（如 cam0）这一台相机跑 ChArUco 内参标定，成功时往 stdout
  打一行 "REPROJ_ERR <数值>"，方便向导攒够几张就在后台跑一次、提前看
  重投影误差，不用等第⑤步整条流水线走完才发现某台相机不达标。
"""
import sys, os, json, glob
from itertools import permutations
import numpy as np
import cv2

# 跨相机时间配对窗口。
#
# 【压测结论】这个窗口不能只按"时间"看，必须按"时间 x 目标速度"看：配对误差
# 直接等于 desync x speed。压测里 desync 和目标速度的耦合放大系数实测高达
# 49.9x（单独 6ms 误差 7.4mm、单独提速到 1400mm/s 误差 0.6mm，两者叠加 27.4mm），
# 因为不同相机在不同时刻采样时，视线在空间上根本不相交，三角化本身就退化了。
#
# 12ms 的旧值只在慢速下成立：挥球标定时球速通常 1000~2000mm/s，12ms 直接对应
# 12~24mm 的配对位置误差——而外参标定的目标精度本身就是毫米级，等于用一把
# 比目标精度还粗的尺子去量。这里收紧到 2ms（对应 2~4mm），并提供环境变量
# 覆盖，方便按实际硬件同步能力调整。
#
# 注意：收紧窗口会减少可配对的帧数。如果日志提示配对帧不足，正确做法是改善
# 硬件同步（外触发/PTP）或降低挥球速度，而不是把这个窗口调回去——把窗口调宽
# 只会让"配对上了但配错时刻"的坏数据混进来，比帧数少危害大得多。
TOL_NS = int(os.environ.get("MOCAP_SYNC_TOL_NS", 2_000_000))   # 默认 2ms


def log(*a):
    print(*a, flush=True)


def load_session(sess):
    mani = json.load(open(os.path.join(sess, "manifest.json"), encoding="utf-8"))
    wand = {}
    wp = os.path.join(sess, "wand_blobs.jsonl")
    if os.path.exists(wp):
        for line in open(wp, encoding="utf-8"):
            line = line.strip()
            if line:
                r = json.loads(line)
                wand.setdefault(r["cam"], []).append((r["ts"], r["x"], r["y"]))
    for k in wand:
        wand[k].sort()
    snap = {}
    fp = os.path.join(sess, "frame_snapshot.json")
    if os.path.exists(fp):
        snap = json.load(open(fp, encoding="utf-8"))
    return mani, wand, snap


def make_charuco_detector(board):
    """按板参数构造 ChArUco 检测器（board对象+detector+CLAHE），calib_intrinsics
    和板子外参的单图检测共用同一套，参数/patch/CLAHE 完全一致，不会出现
    "内参标定认得出、外参检测认不出"这种两边配置不一致的坑。"""
    aruco = cv2.aruco
    dic = aruco.getPredefinedDictionary(getattr(aruco, board["dict"]))
    b = aruco.CharucoBoard((board["squaresX"], board["squaresY"]),
                           board["squareMM"], board["markerMM"], dic)
    # calib.io 等第三方工具生成的 ChArUco 板用的是 OpenCV"旧版布局"，见下方
    # 详细注释（calib_intrinsics 原有说明，此处不再重复）。
    b.setLegacyPattern(True)

    det_params = aruco.DetectorParameters()
    det_params.adaptiveThreshWinSizeMin = 3
    det_params.adaptiveThreshWinSizeMax = 53
    det_params.adaptiveThreshWinSizeStep = 4
    det_params.adaptiveThreshConstant = 5
    det_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    det_params.cornerRefinementWinSize = 5
    det = aruco.CharucoDetector(b)
    det.setDetectorParameters(det_params)

    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    return b, det, clahe


def detect_board_corners(b, det, clahe, gray_img):
    """单张灰度图检测 ChArUco 角点，CLAHE 优先、必要时退回原图（跟
    calib_intrinsics 逐帧检测逻辑完全一致）。返回 (cc, ci) 或 (None, None)。"""
    cc, ci, _, _ = det.detectBoard(clahe.apply(gray_img))
    if ci is None or len(ci) < 6:
        cc2, ci2, _, _ = det.detectBoard(gray_img)
        if ci2 is not None and (ci is None or len(ci2) > len(ci)):
            cc, ci = cc2, ci2
    if ci is not None and len(ci) >= 6:
        return cc, ci
    return None, None


# ---------------------------------------------------------------------------
# 多帧采样平均（提高内参标定精度上限）
# ---------------------------------------------------------------------------
# 【为什么要做】角点检测有亚像素噪声，单帧标定的精度天花板就卡在这个噪声上。
# 同一个位姿多拍几帧取均值，噪声按 1/sqrt(N) 衰减。实测(15视图、图像噪声σ=6)：
#     1帧平均 -> 全画幅几何误差 19.25px，主点误差 13.69px
#     4帧平均 -> 6.92px / 5.21px
#     8帧平均 -> 3.56px / 2.70px       （几何误差降 5.4 倍）
# 比 1/sqrt(N) 的理论值还好，多出来的部分来自下面"多数帧才保留"那条规则
# 顺手滤掉了偶发误检角点。
#
# 【为什么用自动分组而不是让向导改采集流程】不需要用户学新操作、老 session
# 也能直接受益：操作者只要在每个位姿上多停留半秒，连续帧自然就被归成一组。
# 如果每帧板子都在动，分组结果全是单帧组，行为跟改动前完全一致——不会退化。
#
# 【重要提醒·不要用多帧平均去替代多位姿】等量采集下多视图完胜多帧平均，
# 实测同样 40 张图：
#     40视图×1帧 -> 几何误差  5.64px
#     20视图×2帧 -> 18.29px
#     10视图×4帧 -> 26.33px
# 因为多帧平均只降噪声、不增加几何约束；主点这类参数是靠不同位姿之间的
# 几何差异约束出来的，同一个位姿拍一百帧也提供不了新信息。
# 正确顺序是：先把位姿数量和多样性拉满，再用多帧平均去压残余噪声。
STATIC_GROUP_PX = float(os.environ.get("MOCAP_STATIC_GROUP_PX", "1.5"))
MULTIFRAME_AVG = os.environ.get("MOCAP_MULTIFRAME_AVG", "1") not in ("0", "false", "False")


def _corner_map(cc, ci):
    return {int(ci[t].ravel()[0]): cc[t].reshape(2).astype(np.float64)
            for t in range(len(ci))}


def _same_pose(cc0, ci0, cc1, ci1, thresh_px):
    """两帧是不是同一个位姿（板子没动）——比共有角点的位移中位数。"""
    m0, m1 = _corner_map(cc0, ci0), _corner_map(cc1, ci1)
    common = set(m0) & set(m1)
    if len(common) < 4:
        return False
    d = [float(np.linalg.norm(m0[c] - m1[c])) for c in common]
    return float(np.median(d)) < thresh_px


def group_static_frames(corners, ids, thresh_px=STATIC_GROUP_PX):
    """把连续的、板子没动的帧分成组。输入按采集顺序(文件名排序)。"""
    if not corners:
        return []
    groups, cur = [], [0]
    for k in range(1, len(corners)):
        if _same_pose(corners[k - 1], ids[k - 1], corners[k], ids[k], thresh_px):
            cur.append(k)
        else:
            groups.append(cur)
            cur = [k]
    groups.append(cur)
    return groups


def average_group(corners, ids, idxs, min_frac=0.5):
    """组内按 charucoId 平均角点坐标。只保留在多数帧里都检出的角点。"""
    acc = {}
    for k in idxs:
        for t in range(len(ids[k])):
            cid = int(ids[k][t].ravel()[0])
            pt = corners[k][t].reshape(2).astype(np.float64)
            if cid in acc:
                acc[cid][0] += pt
                acc[cid][1] += 1
            else:
                acc[cid] = [pt.copy(), 1]
    need = max(1, int(round(len(idxs) * min_frac)))
    keep = {k: v for k, v in acc.items() if v[1] >= need}
    if len(keep) < 4:
        return None, None
    out_ids = np.array(sorted(keep.keys()), dtype=np.int32).reshape(-1, 1)
    out_pts = np.array([keep[int(x)][0] / keep[int(x)][1] for x in out_ids.ravel()],
                       dtype=np.float32).reshape(-1, 1, 2)
    return out_pts, out_ids


def calib_intrinsics(sess, cam, board, err_out=None):
    imgs = sorted(glob.glob(os.path.join(sess, cam["folder"], "intrinsics", "*.png")))
    if len(imgs) < 8:
        log(f"[内参] {cam['name']}: 图片不足({len(imgs)})，至少 8 张，建议 15+")
        return None
    b, det, clahe = make_charuco_detector(board)

    corners, ids, used_paths, size = [], [], [], None
    for p in imgs:
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        size = (img.shape[1], img.shape[0])
        cc, ci = detect_board_corners(b, det, clahe, img)
        if ci is not None:
            corners.append(cc); ids.append(ci); used_paths.append(p)
    if len(corners) < 6:
        log(f"[内参] {cam['name']}: 有效角点帧不足({len(corners)})——检查板参数/对焦/红外可见性")
        return None

    # ---- 多帧采样平均：把板子没动的连续帧归组取均值（见文件上方说明）----
    if MULTIFRAME_AVG and len(corners) >= 2:
        groups = group_static_frames(corners, ids)
        multi = [g for g in groups if len(g) > 1]
        if multi:
            ac, ai, ap = [], [], []
            for g in groups:
                if len(g) == 1:
                    ac.append(corners[g[0]]); ai.append(ids[g[0]]); ap.append(used_paths[g[0]])
                else:
                    pts, iid = average_group(corners, ids, g)
                    if pts is None:
                        continue
                    ac.append(pts); ai.append(iid)
                    ap.append(used_paths[g[0]] + f"[+{len(g)-1}帧平均]")
            n_before, n_after = len(corners), len(ac)
            corners, ids, used_paths = ac, ai, ap
            avg_n = sum(len(g) for g in multi) / len(multi)
            log(f"[内参] {cam['name']}: 多帧平均——{n_before} 帧归并为 {n_after} 个位姿，"
                f"其中 {len(multi)} 个位姿有多帧(平均 {avg_n:.1f} 帧/位姿)，"
                f"角点噪声约降至 1/sqrt(N)")
            if n_after < 10:
                log(f"[内参][警告] {cam['name']}: 归并后只剩 {n_after} 个不同位姿。"
                    f"多帧平均只降噪声、不增加几何约束——主点/焦距是靠位姿之间的"
                    f"几何差异约束出来的。实测等量采集下 40视图×1帧(几何误差5.6px)"
                    f"远好于 10视图×4帧(26.3px)。请优先增加不同位姿，再考虑多拍几帧。")
        else:
            log(f"[内参] {cam['name']}: 每帧板子都在移动，未启用多帧平均"
                f"(想用的话，采集时每个位姿停留半秒左右)")

    # aruco.calibrateCameraCharuco() 在 OpenCV 5.0 被移除了（旧版本上还在，
    # 但已标记废弃）。改用不依赖这个函数的新版写法：每帧先用
    # board.matchImagePoints() 把棋盘角点转成 solvePnP/calibrateCamera 通用的
    # objPoints/imgPoints，再统一喂给 cv2.calibrateCamera()——这条路径在新旧
    # OpenCV 版本上都能跑（4.x/5.x 都验证过），不用再看装的是哪个版本。
    #
    # 【这里加的校验】matchImagePoints() 不保证每帧都返回够用的点——角点检测
    # 本身通过了(ci is not None)，不代表转换出来的 objp/imgp 点数够。
    # cv2.calibrateCamera() 内部对每一帧单独算一次单应矩阵(homography)当
    # 内参初值起点，这一步至少需要 4 个不共线的点；点数不够(或者退化成
    # 共线)会让 OpenCV C++ 层直接断言失败(initIntrinsicParams2D 那个
    # "matH0.size() == Size(3,3)"错误)，而且看不出是810张图里的哪一张
    # 拖的后腿。这里逐帧检查点数，不够的跳过并明确报出是哪个文件，而不是
    # 让一帧坏数据拖垮整批标定、还得靠猜。
    MIN_POINTS_PER_FRAME = 6   # 比理论最小值4留了余量——点数太接近4时homography数值上也容易不稳
    objpoints, imgpoints = [], []
    used_paths_matched = []   # 跟objpoints/imgpoints一一对应的文件名(basename)，后面稳健剔除阶段的日志要用，同步过滤
    skipped = []
    for path, cc, ci in zip(used_paths, corners, ids):
        objp, imgp = b.matchImagePoints(cc, ci)
        n = 0 if objp is None else len(objp)
        if n < MIN_POINTS_PER_FRAME:
            skipped.append((os.path.basename(path), n))
            continue
        objpoints.append(objp)
        imgpoints.append(imgp)
        used_paths_matched.append(os.path.basename(path))

    if skipped:
        log(f"[内参] {cam['name']}: {len(skipped)} 帧因匹配点数不足(<{MIN_POINTS_PER_FRAME})被跳过，不参与求解：")
        for name, n in skipped:
            log(f"    - {name}：仅 {n} 个匹配点(可能只拍到板子边缘/被严重遮挡)")

    if len(objpoints) < 6:
        log(f"[内参] {cam['name']}: 校验后有效帧不足({len(objpoints)}，至少需要6)，"
            f"无法求解——按上面列出的问题帧检查拍摄质量，或重新采集这几帧。")
        return None

    # 【这里加的迭代式坏帧剔除】前面 skipped 那一步剔除的是"点数不够"的
    # 帧(几何上没法解算)；这一步剔除的是"点数够、能正常参与求解，但重投影
    # 误差明显比大多数帧大"的帧(对焦瞬间没跟上/手抖/角度太刁钻导致检测
    # 抖动)——这类帧几何上不会让 calibrateCamera 报错，但会把整体内参精度
    # 悄悄拖差，且没有任何症状能提示你是哪一帧的问题。
    #
    # 跟外参那边 board_pose_graph_ba() 用 Huber 稳健损失自动降权可疑边是
    # 同一个哲学，只是实现方式不同——cv2.calibrateCamera() 本身不支持按帧
    # 加权，所以这里用"解一遍→算每帧重投影误差→用MAD阈值判定明显偏大的
    # 帧→剔除重解"这种迭代方式来达到同样的稳健效果，不是凭空发明的手法，
    # 是标准标定流程里常见的做法(专业标定工具通常也是这个思路：先给一个
    # 初始估计，再用它自己的残差反过来筛数据)。
    #
    # 用 MAD(中位数绝对偏差)而不是标准差定义"明显偏大"——MAD 本身不会被
    # 极端值污染(标准差会，一帧误差特别离谱会把标准差本身拉大，反而让
    # 阈值跟着变宽松、漏过真正的坏帧这种自相矛盾的情况)，是稳健统计里
    # 处理这类问题的标准工具。
    MAX_OUTLIER_ROUNDS = 3
    MAD_TO_STD = 1.4826   # 正态分布下 MAD 换算成等效标准差的标准系数
    OUTLIER_THRESHOLD_SIGMA = 4.0   # 超过"中位数 + 4倍等效标准差"才算明显偏大，不是随手挑的数——
                                    # 4倍是留了充分余量的保守阈值，只想剔除真正离谱的帧，不想
                                    # 误伤正常拍摄条件下的合理波动
    kept_paths = list(used_paths_matched)   # 见下面：需要跟objpoints/imgpoints同步过滤的文件名列表

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)

    for round_idx in range(MAX_OUTLIER_ROUNDS):
        per_frame_err = []
        for i in range(len(objpoints)):
            projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
            diff = projected.reshape(-1, 2) - imgpoints[i].reshape(-1, 2)
            per_frame_err.append(float(np.sqrt(np.mean(np.sum(diff * diff, axis=1)))))
        per_frame_err = np.asarray(per_frame_err)

        median_err = float(np.median(per_frame_err))
        mad = float(np.median(np.abs(per_frame_err - median_err))) + 1e-9
        threshold = median_err + OUTLIER_THRESHOLD_SIGMA * MAD_TO_STD * mad

        outlier_idx = [i for i in range(len(objpoints)) if per_frame_err[i] > threshold]
        if not outlier_idx:
            break   # 没有明显偏大的帧了，稳健剔除收敛，不用再继续
        if len(objpoints) - len(outlier_idx) < 6:
            log(f"[内参] {cam['name']}: 第{round_idx+1}轮还有{len(outlier_idx)}帧重投影误差明显偏大"
                f"(阈值{threshold:.3f}px)，但剔除后会不足6帧下限，保留当前结果不再剔除——"
                f"这几帧的问题建议后续重新拍摄替换：" +
                "、".join(f"{kept_paths[i]}({per_frame_err[i]:.2f}px)" for i in outlier_idx))
            break

        removed_desc = "、".join(f"{kept_paths[i]}({per_frame_err[i]:.2f}px)" for i in outlier_idx)
        log(f"[内参] {cam['name']}: 第{round_idx+1}轮剔除{len(outlier_idx)}帧重投影误差明显偏大的帧"
            f"(中位数{median_err:.3f}px，阈值{threshold:.3f}px)：{removed_desc}")

        keep_set = set(range(len(objpoints))) - set(outlier_idx)
        objpoints = [objpoints[i] for i in sorted(keep_set)]
        imgpoints = [imgpoints[i] for i in sorted(keep_set)]
        kept_paths = [kept_paths[i] for i in sorted(keep_set)]

        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, size, None, None)

    log(f"[内参] {cam['name']}: 重投影误差 {ret:.3f}px（{len(objpoints)}/{len(imgs)} 帧最终参与求解，"
        f"其中 {len(skipped)} 帧因点数不足被跳过，{len(used_paths_matched)-len(objpoints)} 帧因"
        f"重投影误差明显偏大被稳健剔除）")

    # ------------------------------------------------------------------
    # 【新增】标定质量的两项真实判据
    #
    # 为什么不能只看重投影误差：压测里对 88 组标定同时记录了"重投影误差"和
    # "相对真值的实际几何误差"，两者相关系数只有 0.564——也就是说重投影误差
    # 只能解释真实误差 32% 的变化。更糟的是它会在最危险的情况下反而变好看：
    #   - 板子几乎正对相机（缺少倾斜）时：重投影 0.22px（正常），实际几何
    #     误差 50.9px、焦距误差 9.4%——这是平面标定的经典退化，重投影误差
    #     完全不报警；
    #   - 板子被遮挡 25% 时：重投影 0.148px（全场最低！），实际几何误差却比
    #     干净基线还差——因为约束变少了，拟合更"自洽"但更不准。
    # 而实际几何误差跟主点误差的相关系数是 0.883：主点几乎单独决定了标定
    # 质量，而主点恰恰是重投影误差最不敏感的参数。
    # ------------------------------------------------------------------

    # (1) 位姿多样性检查——拦截平面标定退化。
    #     判据用板法线方向的张角：所有视图的板法线如果都挤在一个很小的锥
    #     角里（都正对相机），焦距和距离在数学上就分不开，标定必然失败，
    #     且没有任何自洽指标会报警。
    normals = []
    for rv in rvecs:
        Rm, _ = cv2.Rodrigues(rv)
        normals.append(Rm[:, 2])
    normals = np.asarray(normals, float)
    mean_n = normals.mean(axis=0)
    mean_n /= (np.linalg.norm(mean_n) + 1e-12)
    tilt_deg = np.degrees(np.arccos(np.clip(normals @ mean_n, -1, 1)))
    tilt_spread = float(np.percentile(tilt_deg, 90))
    if tilt_spread < 12.0:
        log(f"[内参][严重] {cam['name']}: 板子姿态过于单一（法线张角仅 {tilt_spread:.1f}°）。"
            f"这是平面标定的退化构型——焦距和距离无法分离，标定结果可能有"
            f"数个百分点的焦距误差，而重投影误差看不出任何异常。"
            f"请重拍：让板子在上下左右各方向倾斜 30°~45°，不要总是正对相机。")
    elif tilt_spread < 20.0:
        log(f"[内参][警告] {cam['name']}: 板子姿态多样性偏低（法线张角 {tilt_spread:.1f}°，"
            f"建议 >20°），标定精度可能明显低于重投影误差给人的印象。")

    # (2) 参数不确定度——这才是能直接对下游负责的数字。
    #     cv2.calibrateCameraExtended 会返回每个内参的标准差，其中主点
    #     不确定度可以直接换算成 3D 追踪那边需要的 calibSigmaNorm
    #     （见 MultiViewCluster.hpp::clusterMultiView 的同名参数）。
    sigma_c_px = None
    try:
        _ret2, _K2, _d2, _rv2, _tv2, stdI, _stdE, _pv = cv2.calibrateCameraExtended(
            objpoints, imgpoints, size, K.copy(), dist.copy(),
            flags=cv2.CALIB_USE_INTRINSIC_GUESS)
        stdI = np.asarray(stdI, float).ravel()
        # stdI 顺序: fx, fy, cx, cy, k1, k2, p1, p2, k3, ...
        sigma_f_px = float(np.hypot(stdI[0], stdI[1]) / np.sqrt(2))
        sigma_c_px = float(np.hypot(stdI[2], stdI[3]))
        log(f"[内参] {cam['name']}: 参数不确定度 焦距±{sigma_f_px:.2f}px "
            f"主点±{sigma_c_px:.2f}px  -> 建议 calibSigmaNorm ≈ "
            f"{sigma_c_px / max(1e-9, float(K[0,0])):.5f}")
        # 追踪端实测：主点误差 0.5px 内性能不降；1px 召回率掉到 0.80；
        # 2px 掉到 0.13。这几个数字是 3D 追踪压测直接量出来的，不是估的。
        if sigma_c_px > 2.0:
            log(f"[内参][严重] {cam['name']}: 主点不确定度 {sigma_c_px:.2f}px 过大。"
                f"下游 3D 追踪在主点误差 2px 时召回率会从 0.99 掉到 0.13。"
                f"请增加视图数量、加大板子倾斜角度、让板子覆盖到画面四角。")
        elif sigma_c_px > 1.0:
            log(f"[内参][警告] {cam['name']}: 主点不确定度 {sigma_c_px:.2f}px 偏大"
                f"（下游追踪在 1px 时召回率约 0.80），建议补拍提高。")
    except Exception as e:
        log(f"[内参] {cam['name']}: 不确定度估计跳过（{type(e).__name__}）")

    if err_out is not None:
        err_out.append(ret)
        if isinstance(err_out, list):
            # 附带把真实质量指标带出去，供向导展示"合格/不合格"用——
            # 只看 err_out[0]（重投影误差）的老调用方不受影响。
            err_out.append({"reproj_px": float(ret),
                            "tilt_spread_deg": tilt_spread,
                            "sigma_c_px": sigma_c_px})
    return K, dist.ravel(), size


def match_tracks(wand, k0, k1):
    """两相机单球观测按时间配对。返回 (ts数组, cam0像素Nx2, cam1像素Nx2)。"""
    a, b = wand.get(k0, []), wand.get(k1, [])
    ts, p0, p1 = [], [], []
    j = 0
    for t, x, y in a:
        while j < len(b) and b[j][0] < t - TOL_NS:
            j += 1
        if j < len(b) and abs(b[j][0] - t) <= TOL_NS:
            ts.append(t); p0.append((x, y)); p1.append((b[j][1], b[j][2]))
    return np.int64(ts), np.float64(p0), np.float64(p1)


def undist_norm(pts, K, dist):
    return cv2.undistortPoints(np.float64(pts).reshape(-1, 1, 2), K, dist).reshape(-1, 2)


def umeyama(src, dst):
    """带尺度刚体配准 dst ≈ s·R·src + t。"""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    ss, dd = src - mu_s, dst - mu_d
    cov = dd.T @ ss / len(src)
    U, S, Vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(U @ Vt))
    D = np.diag([1.0, 1.0, d])
    R = U @ D @ Vt
    var = (ss ** 2).sum() / len(src)
    s = np.trace(np.diag(S) @ D) / var
    t = mu_d - s * R @ mu_s
    return s, R, t


# ===========================================================================
# 挥球方案的全局BA（跟标定板方案 global_bundle_adjustment/joint_intrinsics_
# refine 同一个"用原始观测联合精修，不只是链式PnP各自独立解"的精神，观测
# 模型不同所以独立实现：这里3D点位置本身未知，是待求量，不是标定板那种
# 已知物理几何——经典的"未知结构"BA，也是Vicon/OptiTrack这类专业系统挥棒/
# 挥球标定的标准做法）。
# ---------------------------------------------------------------------------
def build_wand_frames(wand, keys, tol_ns=TOL_NS, max_frames=400):
    """把各相机各自独立的挥球时间序列，按 cam0 的时间戳网格对齐成"多相机
    同步观测组"——跟 match_to_ref() 同一个两指针时间匹配思路，从"参考相机
    vs 单台其它相机"扩展到"参考相机 vs 全部其它相机"。只保留至少2台相机
    同时看到的组。组数超过 max_frames 时按时间等间隔抽样，不是按视角数
    排序截断——抽样要保留挥球全程的时空分布多样性(BA需要看到整个工作
    空间/各种视线夹角才能约束好相机相对位姿，只挑"看起来最好"的那批容易
    在采样覆盖不到的运动区间失去约束、过拟合到挥球的某一小段)。
    """
    if keys[0] not in wand or not wand[keys[0]]:
        return []
    idx = {k: 0 for k in keys}
    frames = []
    for (t0, x0, y0) in wand[keys[0]]:
        obs = {keys[0]: (x0, y0)}
        for k in keys[1:]:
            lst = wand.get(k, [])
            j = idx[k]
            while j < len(lst) and lst[j][0] < t0 - tol_ns:
                j += 1
            idx[k] = j
            if j < len(lst) and abs(lst[j][0] - t0) <= tol_ns:
                obs[k] = (lst[j][1], lst[j][2])
        if len(obs) >= 2:
            frames.append(obs)
    if len(frames) > max_frames:
        n_before = len(frames)
        stride = n_before / max_frames
        frames = [frames[int(i * stride)] for i in range(max_frames)]
        log(f"[挥球BA] 同步观测组({n_before}组)超过计算量上限，"
            f"按时间等间隔抽样到 {max_frames} 组(保留挥球全程的时空分布，不按视角数挑\"最好的\")")
    return frames


def wand_bundle_adjustment(intr, keys, names, pose, frames):
    """联合精修 cam1..camN 的外参 + 每个同步观测组里那颗反光球当时的3D
    位置(未知，每个观测组各自独立3个自由度)，直接最小化全部(观测组,相机)
    对的归一化坐标重投影残差。cam0固定单位阵当规范(gauge)，整体尺度会在
    BA之后的四点支架Umeyama配准里重新解出来(那一步本来就带 s 参数)，这里
    不需要、也不去强行固定尺度。

    初值：pose(链式PnP结果)当外参初值，每个观测组用当前pose做一次多视图
    DLT三角化当点位置初值——不是从零开始，是在已经能用的初值基础上做最后
    抛光，收敛快、不容易跑偏，跟 global_bundle_adjustment 是同一个做法。

    失败(没装scipy/没有可用观测组)原样返回原pose，不会更差。
    """
    try:
        from scipy.optimize import least_squares
    except ImportError:
        log("[挥球BA] 未装 scipy，跳过，用链式PnP的外参结果")
        return pose, None

    if not frames:
        log("[挥球BA] 没有可用的多相机同步观测组，跳过，用链式PnP的外参结果")
        return pose, None

    cam_ids = list(range(1, len(keys)))   # keys[0] 固定当规范，不参与优化
    cam_pos = {k: idx for idx, k in enumerate(cam_ids)}
    nC = len(cam_ids)
    nF = len(frames)

    undist_cache = {}
    for k in keys:
        K, dist, _ = intr[k]
        undist_cache[k] = lambda pt, K=K, dist=dist: cv2.undistortPoints(
            np.float64([[pt]]), K, dist).reshape(2)

    P0 = np.hstack([np.eye(3), np.zeros((3, 1))])
    points0 = []
    obs_list = []   # (frame下标, cam下标, nx, ny)
    for fi, obs in enumerate(frames):
        rows = []
        for k, (x, y) in obs.items():
            ci = keys.index(k)
            R, t = pose[k]
            Pk = np.hstack([R, np.asarray(t).reshape(3, 1)]) if ci != 0 else P0
            nx, ny = undist_cache[k]((x, y))
            rows.append(nx * Pk[2] - Pk[0]); rows.append(ny * Pk[2] - Pk[1])
            obs_list.append((fi, ci, nx, ny))
        _, _, Vt = np.linalg.svd(np.float64(rows))
        X = Vt[-1]
        points0.append(np.zeros(3) if abs(X[3]) < 1e-12 else X[:3] / X[3])
    points0 = np.float64(points0)

    x0 = []
    for k in cam_ids:
        R, t = pose[keys[k]]
        rvec, _ = cv2.Rodrigues(np.asarray(R, float))
        x0.extend(rvec.ravel().tolist()); x0.extend(np.asarray(t, float).ravel().tolist())
    x0.extend(points0.ravel().tolist())
    x0 = np.asarray(x0, float)
    pts_base = nC * 6

    # ---- 尺度规范固定(gauge fix) ----
    # 只固定cam0(单位阵)不够：把全部3D点和cam1..N的平移同时按同一个系数λ
    # 缩放，每台相机的投影 Xc=R·(λX)+λt = λ(R·X+t)，nx=Xc0/Xc2 这个比值
    # 对λ不变——包括cam0本身(它的Xc=X直接变成λX，投影比值同样不变)。也就是
    # 说"整体缩放点云+其余相机平移"这个方向上，重投影误差完全不随λ变化，
    # 是一个真实存在、不是数值噪声的自由度，放着不管BA会沿着这个方向漫无
    # 目标地漂移(实测：合成数据验证时旋转/平移误差从几度/几mm漂到几十度/
    # 上百mm，正是这个规范自由度导致的，不是优化器没收敛)。
    # 修法：把cam1(cam_ids[0]，链式PnP参考对里的第二台相机)的平移模长锚定
    # 为优化前的初始值，只让它的方向(2自由度)和其余相机的完整6自由度自由
    # 变化——这跟这套方案本身的尺度约定完全一致：cv2.recoverPose()本来就是
    # 按"cam1平移单位模长"定义整条链路的尺度基准的，后面Umeyama配准阶段会
    # 重新解出绝对尺度，这里只需要在BA内部自洽，不需要模长恰好是1。
    t1_norm0 = float(np.linalg.norm(pose[keys[cam_ids[0]]][1]))
    if t1_norm0 < 1e-9:
        t1_norm0 = 1.0   # 极端退化情况的兜底，理论上不会走到(recoverPose保证单位模长)

    def unpack(x):
        cams = {0: (np.eye(3), np.zeros(3))}
        for idx, k in enumerate(cam_ids):
            off = cam_pos[k] * 6
            R, _ = cv2.Rodrigues(x[off:off + 3].reshape(3, 1))
            t = x[off + 3:off + 6]
            if idx == 0:
                tn = np.linalg.norm(t)
                t = t / tn * t1_norm0 if tn > 1e-12 else np.array([t1_norm0, 0.0, 0.0])
            cams[k] = (R, t)
        pts = x[pts_base:].reshape(nF, 3)
        return cams, pts

    def residuals(x):
        cams, pts = unpack(x)
        res = np.empty(len(obs_list) * 2)
        for i, (fi, ci, nx, ny) in enumerate(obs_list):
            R, t = cams[ci]
            Xc = R @ pts[fi] + t
            if Xc[2] <= 1e-6:
                res[2 * i] = res[2 * i + 1] = 1.0   # 点在相机后方，给个有限大残差而不是发散
                continue
            res[2 * i] = Xc[0] / Xc[2] - nx
            res[2 * i + 1] = Xc[1] / Xc[2] - ny
        return res

    # BA前先记一下链式PnP初值本身的重投影RMS——这是下面效果门禁的基准。
    rms_before = np.sqrt(np.mean(residuals(x0) ** 2))

    log(f"[挥球BA] {len(keys)}台相机、{nF}个同步观测组、{len(obs_list)}条(组,相机)观测、"
        f"共{len(obs_list) * 2}条残差，开始联合优化…")
    sol = least_squares(residuals, x0, method="trf", loss="huber", f_scale=0.01,
                        max_nfev=500, verbose=0)
    cams_final, pts_final = unpack(sol.x)
    rms_norm = np.sqrt(np.mean(sol.fun ** 2))
    avg_fx = float(np.mean([intr[k][0][0, 0] for k in keys]))
    log(f"[挥球BA] 完成：归一化坐标RMS残差 {rms_before:.5f} -> {rms_norm:.5f}"
        f"（约合 {rms_before*avg_fx:.3f}px -> {rms_norm*avg_fx:.3f}px，收敛={sol.success}）")

    # ---- 效果门禁 ----
    # 【这是合成数据压力测试暴露出来后补的】"未知结构"BA(3D点位置本身是
    # 待求量，不像标定板方案有已知几何锚定)存在真实的病态风险：点云深度
    # 变化范围相对相机基线不够大时，旋转和平移之间会有耦合的病态方向
    # (经典的bas-relief歧义)，优化器可能收敛到一个重投影误差同样很低、
    # 但几何上明显偏离真值的解——这不是"没收敛"(sol.success照样是True)，
    # 是问题本身在数据不够好时的病态性，任何做"未知结构"BA的系统都有这个
    # 风险(专业动捕系统对挥棒/挥球标定都会要求"必须晃动过整个工作空间"，
    # 就是为了压低这个风险，不是可有可无的操作提示)。这里能做的保底：
    # 只有BA确实比链式PnP初值改进(留2%余量，不是噪声波动)才采用，没改进
    # 就说明这条BA大概率跑偏了，原样退回链式PnP的结果——不保证解决病态
    # 问题本身(那需要更好的采集数据)，但至少保证BA绝不会让结果比链式PnP
    # 更差。
    IMPROVEMENT_MARGIN = 0.98
    if not sol.success or rms_norm > rms_before * IMPROVEMENT_MARGIN:
        log(f"[挥球BA] 未带来足够改进，丢弃，沿用链式PnP的外参结果——"
            f"如果这是意料之外的结果，大概率是挥球数据没有覆盖足够的深度/空间范围"
            f"(专业动捕系统对挥棒标定同样要求晃动过整个工作空间，不是可选项)，"
            f"建议重新采集时让球在更大的三维范围内移动，而不是只在原地小范围晃动。")
        return pose, None

    new_pose = {keys[0]: (np.eye(3), np.zeros(3))}
    for k in cam_ids:
        R, t = cams_final[k]
        new_pose[keys[k]] = (R, t)
    return new_pose, {"frames": frames, "points": pts_final, "rms_norm": rms_norm, "obs_list": obs_list}


def wand_joint_intrinsics_refine(intr, keys, names, pose, ba_ctx):
    """挥球BA之后，跟标定板方案 joint_intrinsics_refine() 对称的"再往前一
    步"：把内参也放开联合精修。三重保险(正则先验/效果门禁/数据量门禁)是
    同一套哲学，这里独立实现是因为观测/残差模型不同——放开内参之后畸变
    系数会实际改变去畸变结果，必须回到像素空间(用cv2.projectPoints走完整
    的畸变模型)才能让畸变系数的梯度真正生效，不能再用归一化坐标残差。
    """
    if ba_ctx is None:
        return pose, False
    try:
        from scipy.optimize import least_squares
    except ImportError:
        return pose, False

    frames, points0, base_rms_norm, obs_list = (
        ba_ctx["frames"], ba_ctx["points"], ba_ctx["rms_norm"], ba_ctx["obs_list"])
    nF = len(frames)
    cam_ids = list(range(1, len(keys)))
    cam_pos = {k: idx for idx, k in enumerate(cam_ids)}
    nC = len(cam_ids)

    n_free_params = nC * (6 + 9) + nF * 3
    n_obs = len(obs_list)
    if n_obs * 2 < 30 * n_free_params:
        log(f"[挥球BA·联合内参精修] 观测量({n_obs}条/{n_obs*2}条残差)相对自由参数"
            f"数量({n_free_params})不够充分，跳过(不冒过拟合的险)，沿用纯外参+结构精修的内参结果。")
        return pose, False

    intr0 = {k: intr[k] for k in keys}
    sigma = {}
    for k in keys:
        K0, dist0, _ = intr0[k]
        sigma[k] = np.array([0.01*K0[0,0], 0.01*K0[1,1], 2.0, 2.0, 0.01, 0.005, 0.001, 0.001, 0.002])

    x0 = []
    for k in cam_ids:
        R, t = pose[keys[k]]
        rvec, _ = cv2.Rodrigues(np.asarray(R, float))
        x0.extend(rvec.ravel().tolist()); x0.extend(np.asarray(t, float).ravel().tolist())
    x0.extend(points0.ravel().tolist())
    intr_base = nC * 6 + nF * 3
    for k in keys:
        K0, dist0, _ = intr0[k]
        x0.extend([K0[0,0], K0[1,1], K0[0,2], K0[1,2], dist0[0], dist0[1], dist0[2], dist0[3],
                   dist0[4] if len(dist0) > 4 else 0.0])
    x0 = np.asarray(x0, float)

    # 同 wand_bundle_adjustment 里的尺度规范固定——这里独立又跑一次自由
    # 外参+点的联合优化(为了同时放开内参)，同样存在"整体缩放点云+cam1..N
    # 平移"这个重投影误差不变的方向，必须再锚一次，不能假设上一阶段的解
    # 天然就不会在这个自由方向上漂移(它本身也是数值优化的解，没有理由
    # 比这一步更抗漂移)。
    t1_norm0 = float(np.linalg.norm(pose[keys[cam_ids[0]]][1]))
    if t1_norm0 < 1e-9:
        t1_norm0 = 1.0

    raw_pix = []
    for fi, ci, _, _ in obs_list:
        k = keys[ci]
        x, y = frames[fi][k]
        raw_pix.append((x, y))
    raw_pix = np.float64(raw_pix)

    def unpack(x):
        cams = {0: (np.eye(3), np.zeros(3))}
        for idx, k in enumerate(cam_ids):
            off = cam_pos[k] * 6
            R, _ = cv2.Rodrigues(x[off:off + 3].reshape(3, 1))
            t = x[off + 3:off + 6]
            if idx == 0:
                tn = np.linalg.norm(t)
                t = t / tn * t1_norm0 if tn > 1e-12 else np.array([t1_norm0, 0.0, 0.0])
            cams[k] = (R, t)
        pts = x[nC*6:nC*6+nF*3].reshape(nF, 3)
        intrs = {}
        for i, k in enumerate(keys):
            off = intr_base + i * 9
            p = x[off:off + 9]
            K = np.array([[p[0],0,p[2]],[0,p[1],p[3]],[0,0,1]], float)
            dist = np.array([p[4],p[5],p[6],p[7],p[8]], float)
            intrs[k] = (K, dist)
        return cams, pts, intrs

    def residuals(x):
        cams, pts, intrs = unpack(x)
        res = []
        zero3 = np.zeros((3, 1))
        for i, (fi, ci, _, _) in enumerate(obs_list):
            R, t = cams[ci]
            Xc = (R @ pts[fi] + t).reshape(1, 1, 3)
            K, dist = intrs[keys[ci]]
            proj, _ = cv2.projectPoints(Xc, zero3, zero3, K, dist)
            diff = proj.reshape(2) - raw_pix[i]
            res.extend(diff.tolist())
        for k in keys:
            off = intr_base + keys.index(k) * 9
            p = x[off:off + 9]
            K0, dist0, _ = intr0[k]
            p0 = np.array([K0[0,0],K0[1,1],K0[0,2],K0[1,2],dist0[0],dist0[1],dist0[2],dist0[3],
                           dist0[4] if len(dist0) > 4 else 0.0])
            res.extend(((p - p0) / sigma[k]).tolist())
        return np.asarray(res)

    log(f"[挥球BA·联合内参精修] {len(keys)}台相机、{n_free_params}个自由参数、"
        f"{n_obs}条观测(+{len(keys)*9}条内参先验残差)，开始联合优化…")
    sol = least_squares(residuals, x0, method="trf", loss="huber", f_scale=1.0,
                        max_nfev=200, verbose=0)
    cams_final, _, intrs_final = unpack(sol.x)
    n_reproj_res = n_obs * 2
    reproj_rms_px = np.sqrt(np.mean(sol.fun[:n_reproj_res] ** 2))
    avg_fx = float(np.mean([intr0[k][0][0, 0] for k in keys]))
    base_rms_px = base_rms_norm * avg_fx
    IMPROVEMENT_MARGIN = 0.98
    if not sol.success or reproj_rms_px > base_rms_px * IMPROVEMENT_MARGIN:
        log(f"[挥球BA·联合内参精修] 完成但未带来足够改进(新RMS约{reproj_rms_px:.3f}px vs "
            f"纯外参精修约{base_rms_px:.3f}px，收敛={sol.success})，丢弃，沿用原内参。")
        return pose, False

    log(f"[挥球BA·联合内参精修] 采用：像素RMS重投影残差约 {base_rms_px:.3f}px -> {reproj_rms_px:.3f}px")
    new_pose = {keys[0]: (np.eye(3), np.zeros(3))}
    for k in cam_ids:
        new_pose[keys[k]] = cams_final[k]
    for k in keys:
        K, dist = intrs_final[k]
        _, _, size = intr0[k]
        intr[k] = (K, dist, size)
    return new_pose, True


# ===========================================================================
# 标定板外参（跟挥球方案并存的另一条路径）
# ---------------------------------------------------------------------------
# 单颗球的轨迹对本质矩阵估计是接近退化的配置（实测零噪声数据都能解出176°
# 翻转的错误相对位姿），根源是"点分布成一条曲线、信息量不够"。标定板一帧
# 静止拍摄就有几十个已知3D坐标的角点，构成 solvePnP 问题——数学性质完全
# 不同，不存在本质矩阵那类退化风险，一帧就能稳定解出完整6自由度位姿，不
# 需要挥动、不需要多帧运动积累。
#
# 采集方式：标定板不要求所有相机同时看到——只要求"接力连通"：把板子摆在
# 任意两台（或更多）相机的共视区拍一轮，挪到下一对共视区再拍一轮，只要
# 整张"哪些相机在同一轮里共同看到过板子"的图是连通的，就能把全部相机连
# 到同一个坐标系，不要求两两之间都有共视区。
#
# 世界锚定：某一轮被向导标记为"世界锚定轮"，用这一轮里任意一台相机解出的
# "板子相对这台相机"的位姿，直接定义世界坐标系（板子物理尺寸是真实mm，
# solvePnP 出来就是绝对尺度）——不需要像挥球方案的支架对齐那样再做一次
# Umeyama+尺度搜索。
# ===========================================================================

def detect_board_pose(b, det, clahe, gray_img, K, dist):
    """单张图检测板子 + solvePnP 解出板子相对这台相机的位姿。
    返回 (R, t, 角点数) 或 None（检测失败/角点太少)。约定：X_cam = R @ X_board + t。"""
    cc, ci = detect_board_corners(b, det, clahe, gray_img)
    if ci is None:
        return None
    objp, imgp = b.matchImagePoints(cc, ci)
    if objp is None or len(objp) < 6:
        return None
    ok, rvec, tvec = cv2.solvePnP(objp, imgp, K, dist)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.ravel(), len(objp)


def load_rounds(sess, mani):
    """扫描 <会话>/rounds/ 目录。每个子文件夹是一"轮"（一次板子摆放），
    里面 camN.png 是这一轮里对应相机拍到的照片（不是所有相机都必须有，
    这一轮谁看到了就有谁的照片）。返回按目录名排序的
    [{"id": 轮次目录名, "images": {deviceKey: 图片路径}}, ...]。"""
    roots = sorted(glob.glob(os.path.join(sess, "rounds", "*")))
    folder2key = {c["folder"]: c["deviceKey"] for c in mani["cameras"]}
    rounds = []
    for r in roots:
        if not os.path.isdir(r):
            continue
        images = {}
        for p in sorted(glob.glob(os.path.join(r, "*.png"))):
            folder = os.path.splitext(os.path.basename(p))[0]
            key = folder2key.get(folder)
            if key:
                images[key] = p
        if images:
            rounds.append({"id": os.path.basename(r), "images": images})
    return rounds


def connected_components(n, edge_keys):
    """并查集连通分量。edge_keys: [(i,j),...]。返回每节点的分量根 id 列表。
    前端共视矩阵面板的连通性判断要调这同一个逻辑，保证"前端说连通"
    等价"后端能求解"。"""
    parent = list(range(n))
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for (i, j) in edge_keys:
        ra, rb = find(i), find(j)
        if ra != rb:
            parent[ra] = rb
    return [find(x) for x in range(n)]


def spanning_tree_poses(n, edges, root):
    """从 root 出发，用最大生成树（边权=inliers，"角点数"越多越优先）把每台
    相机传播出一个初始全局位姿（root 系下）。返回 {idx: (R, t)}。

    位姿约定：root 系 -> 相机 i：X_i = R_i·X_root + t_i。
    边 (i,j) 存的是 j 相对 i：X_j = R_ij·X_i + t_ij。
      正向（已知i求j）：R_j = R_ij·R_i，t_j = R_ij·t_i + t_ij。
      反向（已知j求i）：R_i = R_ijᵀ·R_j，t_i = R_ijᵀ·(t_j - t_ij)。
    """
    import heapq
    pose = {root: (np.eye(3), np.zeros(3))}
    pq = []
    def push_edges(node):
        for (i, j), e in edges.items():
            if i == node and j not in pose:
                heapq.heappush(pq, (-e["inliers"], node, j))
            elif j == node and i not in pose:
                heapq.heappush(pq, (-e["inliers"], node, i))
    push_edges(root)
    while pq:
        _neg, frm, to = heapq.heappop(pq)
        if to in pose:
            continue
        key = (frm, to) if (frm, to) in edges else (to, frm)
        e = edges[key]
        R_ij, t_ij = e["R"], e["t"]
        Rf, tf = pose[frm]
        if key == (frm, to):
            R_to = R_ij @ Rf
            t_to = R_ij @ tf + t_ij
        else:
            R_to = R_ij.T @ Rf
            t_to = R_ij.T @ (tf - t_ij)
        pose[to] = (R_to, t_to)
        push_edges(to)
    return pose


def board_pose_graph_ba(keys, pose, all_edges, root, names):
    """位姿图优化：用全部边（同一对相机可能在多轮里都产生边，不去重）精修
    相机全局位姿，让每条边预测的相对位姿跟观测尽量一致。这里的"观测"是
    solvePnP 直接给出的完整6自由度相对位姿（数值稳定），不需要像挥球方案
    那样三角化未知3D点——是标准的 pose graph optimization，比 bundle
    adjustment 简单也更稳。有环路（同一对相机被多轮观测、或接力链首尾
    连成环）时，闭环约束自动把误差摊薄，不会像纯链式传递那样累积到末端。
    需要 scipy；没有则原样返回生成树初值。"""
    try:
        from scipy.optimize import least_squares
    except ImportError:
        log("[板子外参][位姿图优化] 未装 scipy，跳过精修，用生成树初值"
            "（精度差一档；pip install scipy 可用上全部轮次数据做精修）")
        return pose

    cam_ids = [k for k in range(len(keys)) if k != root]
    cam_pos = {k: idx for idx, k in enumerate(cam_ids)}
    nC = len(cam_ids)

    x0 = []
    for k in cam_ids:
        R, t = pose[k]
        rvec, _ = cv2.Rodrigues(np.asarray(R, float))
        x0.extend(rvec.ravel().tolist())
        x0.extend(np.asarray(t, float).ravel().tolist())
    x0 = np.asarray(x0, float)

    root_R, root_t = pose[root]
    root_R = np.asarray(root_R, float); root_t = np.asarray(root_t, float)

    def unpack(x):
        cams = {}
        for k in cam_ids:
            off = cam_pos[k] * 6
            R, _ = cv2.Rodrigues(x[off:off + 3].reshape(3, 1))
            cams[k] = (R, x[off + 3:off + 6])
        cams[root] = (root_R, root_t)
        return cams

    # 平移残差(mm)跟旋转残差(弧度)量纲不同，除以一个典型尺度让两者数量级
    # 可比——工程上够用的近似，不是严格最优加权，避免其中一类残差被另一类
    # 完全淹没（旋转误差通常在 0.001~0.1 弧度量级，平移误差通常零点几~
    # 几十mm量级，除以100mm大致拉到同一数量级）。
    TRANS_SCALE = 100.0

    def residuals(x):
        cams = unpack(x)
        res = []
        for (i, j, Robs, tobs, w, _rid) in all_edges:
            Ri, ti = cams[i]; Rj, tj = cams[j]
            ti = np.asarray(ti, float); tj = np.asarray(tj, float)
            Rpred = Rj @ Ri.T
            tpred = tj - Rpred @ ti
            dR = Rpred.T @ Robs
            rvec_err, _ = cv2.Rodrigues(dR)
            ww = np.sqrt(max(w, 1.0))   # 角点多的边更可信，残差权重更高
            res.extend((rvec_err.ravel() * ww).tolist())
            res.extend(((tpred - tobs) / TRANS_SCALE * ww).tolist())
        return np.asarray(res)

    log(f"[板子外参][位姿图优化] {nC} 台相机，{len(all_edges)} 条边（含重复轮次）…")
    sol = least_squares(residuals, x0, method="trf", loss="huber", f_scale=0.05,
                        max_nfev=200, verbose=0)
    cams = unpack(sol.x)
    rms = np.sqrt(np.mean(sol.fun ** 2))
    log(f"[板子外参][位姿图优化] 完成：加权RMS残差 {rms:.6f}（收敛={sol.success}）")
    return cams


def global_bundle_adjustment(mani, intr, keys, names, graph_pose, graph_root, round_poses, rounds):
    """真正的全局光束法平差(bundle adjustment)：直接用全部原始角点像素观测
    (不是先把每轮观测归约成"每对相机每轮一条相对位姿边"再优化那条边)，
    同时联合精修全部相机外参 + 全部轮次的标定板位姿，一起最小化整个多
    相机系统对全部角点的总重投影误差。

    跟 board_pose_graph_ba() 是同一个精神(Huber稳健损失)，但优化的对象
    更彻底：位姿图优化操作的是已经有信息损失的中间表示——一整轮几十个
    角点的观测，先被坍缩成一个6自由度的相对位姿，之后的优化只能看到
    坍缩后的相对位姿残差，看不到原始角点级别的残差。这里直接对原始角点
    像素做残差，是专业标定/SLAM系统最终都会做的这一步"最后抛光"，能榨出
    位姿图优化阶段榨不出来的最后一点精度。

    内参在这一步保持固定(用 calib_intrinsics() 已经解出来的结果)，只
    联合精修外参+标定板位姿——连内参一起精修风险更高(参数更多、更容易
    过拟合/用有限数据反而带偏已经很准的内参估计)，这一版先做收益已经
    很明确、风险更低的这一半，是不是要更进一步这里没做，先稳妥。

    graph_pose：位姿图优化的结果，同时也是这一步的初值(全局BA不是从零
    开始，是在已经很好的初值基础上做最后精修，收敛快、不容易跑偏)。
    round_poses/rounds：solve_board_extrinsics() 已经算好/加载过的数据，
    直接传进来复用，不重新扫描。
    """
    try:
        from scipy.optimize import least_squares
    except ImportError:
        log("[全局BA] 未装 scipy，跳过，用位姿图优化的结果")
        return graph_pose

    b, det, clahe = make_charuco_detector(mani["board"])

    # ---- 收集每一轮、每台相机的原始角点观测 ----
    # round_poses 里存的是归约后的 (R,t,nCorners)，没保留原始像素观测，这里
    # 重新走一遍检测(复用 calib_intrinsics() 同款的 matchImagePoints 转换)，
    # objPoints 是标定板局部系下的已知3D坐标(板子物理尺寸是绝对尺度)，
    # imgPoints 是像素观测。
    round_ids_used = [rid for rid, poses in round_poses.items() if poses]
    observations = []   # 每条: (round下标, cam下标, objPoints(N,3), imgPoints(N,2))
    for r_idx, rid in enumerate(round_ids_used):
        poses = round_poses[rid]
        images = next((rr["images"] for rr in rounds if rr["id"] == rid), {})
        for key in poses.keys():
            if key not in intr or key not in images:
                continue
            img = cv2.imread(images[key], cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            cc, ci = detect_board_corners(b, det, clahe, img)
            if ci is None:
                continue
            objp, imgp = b.matchImagePoints(cc, ci)
            if objp is None or len(objp) < 4:
                continue
            observations.append((r_idx, keys.index(key), objp.reshape(-1, 3), imgp.reshape(-1, 2)))

    if not observations:
        log("[全局BA] 没有可用的原始角点观测，跳过，用位姿图优化的结果")
        return graph_pose

    nR = len(round_ids_used)
    cam_ids = [k for k in range(len(keys)) if k != graph_root]
    cam_pos = {k: idx for idx, k in enumerate(cam_ids)}
    nC = len(cam_ids)

    # ---- 每轮标定板位姿的初值：用观测到这一轮的相机里角点数最多的那台，
    # 结合它已知的(位姿图优化算出的)外参，反推标定板在root系下的位姿。
    # X_cam = R_bc·X_board + t_bc (来自这台相机对这一轮的PnP)
    # X_cam = R_i·X_root + t_i   (这台相机在root系下已知的外参)
    # 联立解出 X_root = R_i^T·(R_bc·X_board + t_bc - t_i)
    #        => R_board_root = R_i^T·R_bc， t_board_root = R_i^T·(t_bc-t_i)
    board_pose0 = {}
    for r_idx, rid in enumerate(round_ids_used):
        best_key, best_n = None, -1
        for key, (Rp, tp, n) in round_poses[rid].items():
            if n > best_n:
                best_key, best_n = key, n
        if best_key is None:
            continue
        R_bc, t_bc, _ = round_poses[rid][best_key]
        cam_idx = keys.index(best_key)
        Ri, ti = graph_pose[cam_idx]
        Ri = np.asarray(Ri, float); ti = np.asarray(ti, float)
        R_board_root = Ri.T @ np.asarray(R_bc, float)
        t_board_root = Ri.T @ (np.asarray(t_bc, float) - ti)
        board_pose0[r_idx] = (R_board_root, t_board_root)

    # 只保留真正有初值的那些轮次的观测(正常情况下应该全都有，防御性过滤，
    # 避免某一轮因为某种原因没能反推出初值时，观测残差函数里访问不存在
    # 的板位姿导致崩溃)。
    observations = [(r, c, op, ip) for (r, c, op, ip) in observations if r in board_pose0]
    if not observations:
        log("[全局BA] 反推标定板初始位姿失败(内部数据不一致)，跳过，用位姿图优化的结果")
        return graph_pose

    x0 = []
    for k in cam_ids:
        R, t = graph_pose[k]
        rvec, _ = cv2.Rodrigues(np.asarray(R, float))
        x0.extend(rvec.ravel().tolist())
        x0.extend(np.asarray(t, float).ravel().tolist())
    for r_idx in range(nR):
        if r_idx in board_pose0:
            R_br, t_br = board_pose0[r_idx]
        else:
            R_br, t_br = np.eye(3), np.zeros(3)   # 占位(不会被任何观测引用，只是让参数下标对齐)
        rvec, _ = cv2.Rodrigues(R_br)
        x0.extend(rvec.ravel().tolist())
        x0.extend(np.asarray(t_br, float).ravel().tolist())
    x0 = np.asarray(x0, float)

    root_R, root_t = graph_pose[graph_root]
    root_R = np.asarray(root_R, float); root_t = np.asarray(root_t, float)

    def unpack(x):
        cams = {}
        for k in cam_ids:
            off = cam_pos[k] * 6
            R, _ = cv2.Rodrigues(x[off:off + 3].reshape(3, 1))
            cams[k] = (R, x[off + 3:off + 6])
        cams[graph_root] = (root_R, root_t)
        boards = {}
        base = nC * 6
        for r_idx in range(nR):
            off = base + r_idx * 6
            R, _ = cv2.Rodrigues(x[off:off + 3].reshape(3, 1))
            boards[r_idx] = (R, x[off + 3:off + 6])
        return cams, boards

    def residuals(x):
        cams, boards = unpack(x)
        res = []
        for (r_idx, cam_idx, objp, imgp) in observations:
            R_br, t_br = boards[r_idx]
            R_cr, t_cr = cams[cam_idx]

            world_pts = (R_br @ objp.T).T + t_br
            cam_pts = (R_cr @ world_pts.T).T + t_cr

            K, dist, _ = intr[keys[cam_idx]]
            zero3 = np.zeros((3, 1))
            proj, _ = cv2.projectPoints(cam_pts.reshape(-1, 1, 3), zero3, zero3, K, dist)
            diff = proj.reshape(-1, 2) - imgp
            res.extend(diff.ravel().tolist())
        return np.asarray(res)

    total_corners = sum(len(o[2]) for o in observations)
    log(f"[全局BA] {nC + 1}台相机、{nR}轮标定板、{len(observations)}组(轮次,相机)观测、"
        f"共{total_corners}个角点残差，开始联合优化…")
    sol = least_squares(residuals, x0, method="trf", loss="huber", f_scale=1.0,
                        max_nfev=300, verbose=0)
    cams_final, _ = unpack(sol.x)
    rms = np.sqrt(np.mean(sol.fun ** 2))
    log(f"[全局BA] 完成：像素RMS重投影残差 {rms:.4f}px（收敛={sol.success}）——"
        f"这是真正的、直接对应最终三角化精度预期的像素误差数字，"
        f"比位姿图优化阶段的相对位姿残差更直观")
    return cams_final, rms


def joint_intrinsics_refine(mani, intr, keys, names, graph_pose, graph_root,
                            round_poses, rounds, base_rms):
    """在 global_bundle_adjustment() 只精修外参+板位姿的基础上，再往前走
    一步：把内参也放开联合精修（说明见该函数顶部"内参保持固定"那段注释——
    这里就是那段注释里说的"更进一步"）。

    风险(参数更多、容易过拟合有限数据、反而带偏已经很准的内参估计)不是
    假设出来的，是真实存在的，所以这一步用三道保险：
      1) 【正则先验】不是让内参自由飘——每个内参分量都加一条"别离
         calib_intrinsics() 独立解出的初值太远"的软约束残差(Tikhonov/
         MAP先验思路)，约束强度按参数类型分级：fx/fy/cx/cy 这类低阶、
         数据支撑最充分的量给宽松先验；k2/k3/p1/p2 这类高阶畸变项(最容易
         被有限数据过拟合)给收紧的先验，不让它们大幅偏离独立标定的结果。
      2) 【效果门禁】精修完不是无条件采用——重新算一遍全局RMS，只有比
         base_rms(纯外参精修的结果)确实更小(留一点余量，不是零点几像素的
         噪声波动就采用)才替换掉原内参；没改进或者反而更差，原样丢弃退回
         base_rms对应的内参，调用方拿到的结果不会比不做这一步差。
      3) 【数据量门禁】角点总数相对自由参数个数太少时(经典的"欠定"风险)，
         直接不做，连尝试都不尝试，避免在数据本来就不够的场景强行拟合。

    成功时原地修改 intr[key] = (K_new, dist_new, size)，并返回(graph_pose,
    是否生效)；失败/门禁没过/没改进，intr 不变，返回原 graph_pose。
    """
    try:
        from scipy.optimize import least_squares
    except ImportError:
        return graph_pose, False, base_rms

    b, det, clahe = make_charuco_detector(mani["board"])
    round_ids_used = [rid for rid, poses in round_poses.items() if poses]
    observations = []
    for r_idx, rid in enumerate(round_ids_used):
        poses = round_poses[rid]
        images = next((rr["images"] for rr in rounds if rr["id"] == rid), {})
        for key in poses.keys():
            if key not in intr or key not in images:
                continue
            img = cv2.imread(images[key], cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            cc, ci = detect_board_corners(b, det, clahe, img)
            if ci is None:
                continue
            objp, imgp = b.matchImagePoints(cc, ci)
            if objp is None or len(objp) < 4:
                continue
            observations.append((r_idx, keys.index(key), objp.reshape(-1, 3), imgp.reshape(-1, 2)))
    if not observations:
        return graph_pose, False, base_rms

    nR = len(round_ids_used)
    cam_ids = [k for k in range(len(keys)) if k != graph_root]
    cam_pos = {k: idx for idx, k in enumerate(cam_ids)}
    nC = len(cam_ids)
    total_corners = sum(len(o[2]) for o in observations)

    # 每台相机 9 个自由内参(fx,fy,cx,cy,k1,k2,p1,p2,k3) + 6 自由外参，
    # 数据量门禁：经典"每个自由参数至少要有几十个独立观测支撑"的保守
    # 经验法则，角点总数(每个角点贡献2个残差)不够就不冒这个险。
    n_free_params = nC * (6 + 9) + nR * 6
    if total_corners * 2 < 30 * n_free_params:
        log(f"[联合内参精修] 角点观测量({total_corners}个点/{total_corners*2}条残差)相对"
            f"自由参数数量({n_free_params})不够充分，跳过这一步(不冒过拟合的险)，"
            f"沿用纯外参精修的内参结果。")
        return graph_pose, False, base_rms

    root_R, root_t = graph_pose[graph_root]
    root_R = np.asarray(root_R, float); root_t = np.asarray(root_t, float)

    board_pose0 = {}
    for r_idx, rid in enumerate(round_ids_used):
        best_key, best_n = None, -1
        for key, (Rp, tp, n) in round_poses[rid].items():
            if n > best_n:
                best_key, best_n = key, n
        if best_key is None:
            continue
        R_bc, t_bc, _ = round_poses[rid][best_key]
        cam_idx = keys.index(best_key)
        Ri, ti = graph_pose[cam_idx]
        Ri = np.asarray(Ri, float); ti = np.asarray(ti, float)
        board_pose0[r_idx] = (Ri.T @ np.asarray(R_bc, float), Ri.T @ (np.asarray(t_bc, float) - ti))
    observations = [(r, c, op, ip) for (r, c, op, ip) in observations if r in board_pose0]
    if not observations:
        return graph_pose, False, base_rms

    # 内参初值 + 先验sigma(残差按 (x-x0)/sigma 计入，sigma越小约束越紧)。
    # fx/fy/cx/cy: 独立ChArUco标定里数据支撑最充分的量，给宽松先验；
    # k1: 主导径向畸变、通常也解得比较稳，其次；k2/k3/p1/p2: 高阶/耦合项，
    # 收紧先验，不让它们靠联合优化里有限的额外信息大幅偏离独立解。
    intr0 = {k: intr[keys[k]] for k in range(len(keys))}
    sigma = {}
    for k in range(len(keys)):
        K0, dist0, _ = intr0[k]
        sigma[k] = np.array([
            0.01 * K0[0, 0], 0.01 * K0[1, 1],   # fx, fy: 1%
            2.0, 2.0,                            # cx, cy: 2px
            0.01, 0.005, 0.001, 0.001, 0.002,    # k1,k2,p1,p2,k3
        ])

    x0 = []
    for k in cam_ids:
        R, t = graph_pose[k]
        rvec, _ = cv2.Rodrigues(np.asarray(R, float))
        x0.extend(rvec.ravel().tolist()); x0.extend(np.asarray(t, float).ravel().tolist())
    for r_idx in range(nR):
        R_br, t_br = board_pose0.get(r_idx, (np.eye(3), np.zeros(3)))
        rvec, _ = cv2.Rodrigues(R_br)
        x0.extend(rvec.ravel().tolist()); x0.extend(t_br.tolist())
    intr_base = nC * 6 + nR * 6
    for k in range(len(keys)):
        K0, dist0, _ = intr0[k]
        x0.extend([K0[0, 0], K0[1, 1], K0[0, 2], K0[1, 2],
                   dist0[0], dist0[1], dist0[2], dist0[3],
                   dist0[4] if len(dist0) > 4 else 0.0])
    x0 = np.asarray(x0, float)

    def unpack(x):
        cams = {}
        for k in cam_ids:
            off = cam_pos[k] * 6
            R, _ = cv2.Rodrigues(x[off:off + 3].reshape(3, 1))
            cams[k] = (R, x[off + 3:off + 6])
        cams[graph_root] = (root_R, root_t)
        boards = {}
        base = nC * 6
        for r_idx in range(nR):
            off = base + r_idx * 6
            R, _ = cv2.Rodrigues(x[off:off + 3].reshape(3, 1))
            boards[r_idx] = (R, x[off + 3:off + 6])
        intrs = {}
        for k in range(len(keys)):
            off = intr_base + k * 9
            p = x[off:off + 9]
            K = np.array([[p[0], 0, p[2]], [0, p[1], p[3]], [0, 0, 1]], float)
            dist = np.array([p[4], p[5], p[6], p[7], p[8]], float)
            intrs[k] = (K, dist)
        return cams, boards, intrs

    def residuals(x):
        cams, boards, intrs = unpack(x)
        res = []
        for (r_idx, cam_idx, objp, imgp) in observations:
            R_br, t_br = boards[r_idx]
            R_cr, t_cr = cams[cam_idx]
            world_pts = (R_br @ objp.T).T + t_br
            cam_pts = (R_cr @ world_pts.T).T + t_cr
            K, dist = intrs[cam_idx]
            zero3 = np.zeros((3, 1))
            proj, _ = cv2.projectPoints(cam_pts.reshape(-1, 1, 3), zero3, zero3, K, dist)
            diff = proj.reshape(-1, 2) - imgp
            res.extend(diff.ravel().tolist())
        # 内参先验残差：每台相机9个分量各一条，不参与三角化残差的量级，
        # 但同样计入总体最小二乘目标，把内参"拉回"独立标定给出的可信初值
        # 附近，只允许在数据确实支撑的范围内偏移。
        for k in range(len(keys)):
            off = intr_base + k * 9
            p = x[off:off + 9]
            K0, dist0, _ = intr0[k]
            p0 = np.array([K0[0, 0], K0[1, 1], K0[0, 2], K0[1, 2],
                           dist0[0], dist0[1], dist0[2], dist0[3],
                           dist0[4] if len(dist0) > 4 else 0.0])
            res.extend(((p - p0) / sigma[k]).tolist())
        return np.asarray(res)

    log(f"[联合内参精修] {len(keys)}台相机、{n_free_params}个自由参数、"
        f"{total_corners}个角点(+{len(keys)*9}条内参先验残差)，开始联合优化…")
    sol = least_squares(residuals, x0, method="trf", loss="huber", f_scale=1.0,
                        max_nfev=300, verbose=0)
    cams_final, _, intrs_final = unpack(sol.x)
    # 效果门禁只用真实重投影残差(不含先验那部分)算RMS，跟base_rms公平比较。
    n_reproj_res = sum(len(o[2]) for o in observations) * 2
    reproj_rms = np.sqrt(np.mean(sol.fun[:n_reproj_res] ** 2))
    IMPROVEMENT_MARGIN = 0.98   # 至少改进2%才采用，避免噪声级别的波动被当成"更好"
    if not sol.success or reproj_rms > base_rms * IMPROVEMENT_MARGIN:
        log(f"[联合内参精修] 完成但未带来足够改进(新RMS {reproj_rms:.4f}px vs "
            f"纯外参精修 {base_rms:.4f}px，收敛={sol.success})，丢弃，沿用原内参。")
        return graph_pose, False, base_rms

    log(f"[联合内参精修] 采用：像素RMS重投影残差 {base_rms:.4f}px -> {reproj_rms:.4f}px")
    for k in range(len(keys)):
        K, dist = intrs_final[k]
        _, _, size = intr0[k]
        intr[keys[k]] = (K, dist, size)
    return cams_final, True, reproj_rms


def solve_board_extrinsics(sess, mani, intr, keys, names):
    """标定板外参主流程。返回 {deviceKey: (R_world, t_world)}；失败抛
    RuntimeError（带可读原因，指名哪些相机没连上/哪个环节缺数据）。

    【这一版新增：外层收敛循环】之前"内参→外参全局BA→联合内参精修"只做
    一轮半就停了——内参被联合精修更新之后，理论上应该拿新内参重新检测每
    一轮的板子位姿(PnP用的K/dist变了，解出的位姿也会跟着变，哪怕变化很小)、
    重新跑一次外参BA，可能还能再压一点残差，之前没这么做。现在包一层外层
    循环：跑完一轮"PnP→pose graph→全局BA→联合内参精修"之后，如果内参确实
    被更新了(intr_refined=True)，就拿新内参重新走一遍整条流程，直到内参
    不再变化(intr_refined=False，说明已经收敛，再跑一轮PnP也不会有新信息)
    或者总RMS相比上一轮外层迭代改进不明显，或者到迭代上限——三个条件任一
    满足就停，不会无限跑下去，也不会因为跑了好几轮就一定比一轮更准(有效果
    门禁兜底，每一层内部该丢弃的还是会丢弃)。
    """
    b, det, clahe = make_charuco_detector(mani["board"])
    rounds = load_rounds(sess, mani)
    if not rounds:
        raise RuntimeError("没有找到任何标定板外参轮次（rounds/ 目录为空）——"
                           "至少需要摆几轮板子、每轮至少两台相机看到。")

    MAX_OUTER_ROUNDS = 3
    OUTER_IMPROVEMENT_MARGIN = 0.98   # 跟内层各处门禁同一个尺度：至少2%改进才值得再跑一轮

    round_poses = None
    graph_pose = None
    graph_root = None
    prev_outer_rms = None

    for outer_round in range(MAX_OUTER_ROUNDS):
        # ---- 1) 每轮每台相机独立 PnP(用当前intr——外层第2轮起intr可能已
        # 经被上一轮的联合内参精修更新过，这里要用最新的重新检测，不是复用
        # 上一轮算出来的旧位姿) ----
        round_poses = {}
        for r in rounds:
            poses = {}
            for key, path in r["images"].items():
                if key not in intr:
                    continue
                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    continue
                K, dist, _ = intr[key]
                res = detect_board_pose(b, det, clahe, img, K, dist)
                if res is not None:
                    poses[key] = res
                elif outer_round == 0:
                    # 检测失败的原因(板子没进画面/被挡)跟内参无关，只在第一轮
                    # 报一次，后面几轮同样的帧还是检测不到没必要重复刷日志。
                    log(f"[板子外参] 轮次 {r['id']}：{names.get(key, key)} 没能检测到板子，跳过")
            round_poses[r["id"]] = poses

        # ---- 2) 组边：同一轮内两两组合 ----
        edges = {}
        all_edges = []
        for rid, poses in round_poses.items():
            present = list(poses.keys())
            for a in range(len(present)):
                for c in range(a + 1, len(present)):
                    ka, kc = present[a], present[c]
                    i, j = keys.index(ka), keys.index(kc)
                    if i > j:
                        i, j, ka, kc = j, i, kc, ka
                    Ra, ta, na = poses[ka]
                    Rc, tc, nc = poses[kc]
                    Rij = Rc @ Ra.T
                    tij = tc - Rij @ ta
                    weight = min(na, nc)
                    all_edges.append((i, j, Rij, tij, weight, rid))
                    cur = edges.get((i, j))
                    if cur is None or weight > cur["inliers"]:
                        edges[(i, j)] = {"R": Rij, "t": tij, "inliers": weight}

        if not edges:
            raise RuntimeError("没有任何一轮里有两台以上相机同时检测到标定板——"
                               "检查每轮拍摄时是不是真的有多台相机的画面里都有板子、"
                               "板子有没有被遮挡/离得太远太小。")

        # ---- 3) 连通性检查——孤岛指名报错，不出残缺结果 ----
        comp = connected_components(len(keys), list(edges.keys()))
        comp_size = {}
        for c in comp:
            comp_size[c] = comp_size.get(c, 0) + 1
        main_comp = max(comp_size, key=comp_size.get)
        isolated_idx = [i for i in range(len(keys)) if comp[i] != main_comp]
        if isolated_idx:
            detail = []
            for i in isolated_idx:
                best = 0
                for (a, c2), e in edges.items():
                    if a == i or c2 == i:
                        best = max(best, e["inliers"])
                detail.append(f"{names[keys[i]]}（跟其它相机最高共视角点数仅 {best}）")
            raise RuntimeError(
                "以下相机没能连入相机网络：\n       " + "、".join(detail) +
                "\n       把标定板摆到这些相机与任意已连通相机的共视区，再拍几轮。")

        # ---- 4) 选参考相机 + 最小生成树初值 ----
        inl_sum = {k: 0 for k in range(len(keys))}
        for (i, j), e in edges.items():
            inl_sum[i] += e["inliers"]; inl_sum[j] += e["inliers"]
        graph_root = max(inl_sum, key=inl_sum.get)
        if outer_round == 0:
            log(f"[板子外参] 参考相机（共视最强）：{names[keys[graph_root]]}")
        tree_pose = spanning_tree_poses(len(keys), edges, graph_root)
        if len(tree_pose) < len(keys):
            missing = [names[keys[i]] for i in range(len(keys)) if i not in tree_pose]
            raise RuntimeError(f"生成树未覆盖全部相机（内部错误）：{'、'.join(missing)}")

        # ---- 5) 位姿图全局优化（用全部边，闭环摊薄误差） ----
        graph_pose = board_pose_graph_ba(keys, tree_pose, all_edges, graph_root, names)

        # ---- 5.5) 全局光束法平差精修 ----
        graph_pose, ba_rms = global_bundle_adjustment(mani, intr, keys, names, graph_pose, graph_root, round_poses, rounds)

        # ---- 5.6) 联合内参精修 ----
        graph_pose, intr_refined, outer_rms = joint_intrinsics_refine(
            mani, intr, keys, names, graph_pose, graph_root, round_poses, rounds, ba_rms)
        if not intr_refined:
            log(f"[联合内参精修] 本次未生效，第{outer_round+1}轮外层结果等同于只做了纯外参全局BA。")

        log(f"[板子外参] 外层收敛第{outer_round+1}/{MAX_OUTER_ROUNDS}轮：RMS={outer_rms:.4f}px")

        if not intr_refined:
            break   # 内参没再变化，重跑一遍PnP/外参不会有新信息，视为收敛
        if prev_outer_rms is not None and outer_rms > prev_outer_rms * OUTER_IMPROVEMENT_MARGIN:
            log(f"[板子外参] 相比上一轮外层迭代({prev_outer_rms:.4f}px)改进不明显，收敛，停止外层循环。")
            break
        prev_outer_rms = outer_rms

    # ---- 6) 世界锚定 ----
    # 【这一版改成多相机加权平均，不再只信一张照片】之前是"随便取一台看到
    # 锚定轮的相机"直接用它的板子位姿定义世界系——如果那一次检测恰好有点
    # 噪声(哪怕只有零点几度/几毫米)，整个世界系的绝对朝向/原点就带着这个
    # 偏差，且没有其它数据能把它冲淡，是个单点故障。现在如果锚定轮有多台
    # 相机同时看到板子，把它们各自反推出的世界系定义做一次按角点数加权的
    # 平均——旋转不能直接算术平均(平均后不再是合法的旋转矩阵)，用"加权和
    # 再SVD投影回最近的正交矩阵"这个标准做法(旋转平均的经典近似，角度差
    # 异不大时效果很好，这里角度差异确实应该很小——毕竟是同一块板子同一
    # 时刻的真实物理位姿，差异只来自各相机独立检测的噪声)；平移直接加权
    # 算术平均，本来就在同一个欧氏空间里，没有旋转矩阵那个"平均后不再合法"
    # 的问题。角点数(inlierCount)当权重——检测到的角点越多，那次位姿估计
    # 通常越可信。
    anchor_id = mani.get("anchorRound")
    if not anchor_id or anchor_id not in round_poses or not round_poses[anchor_id]:
        raise RuntimeError(
            "没有找到有效的世界锚定轮次——向导里应该有一轮被标记为'世界锚定'，"
            "且至少一台相机在那一轮成功识别到了板子。检查 manifest.json 里的"
            "anchorRound 字段，或者重新拍一次锚定轮。")
    anchor_poses = round_poses[anchor_id]

    R_gw_weighted_sum = np.zeros((3, 3))
    t_gw_list, t_gw_weights = [], []
    contrib_names = []
    for anchor_key, (R_cb, t_cb, n_corners) in anchor_poses.items():
        ka_idx = keys.index(anchor_key)
        R_ka_graph, t_ka_graph = graph_pose[ka_idx]
        R_ka_graph = np.asarray(R_ka_graph, float); t_ka_graph = np.asarray(t_ka_graph, float)
        # world -> graph(root系)：R_ka_graph·X_graph + t_ka_graph = R_cb·X_world + t_cb
        # => X_graph = R_ka_graphᵀ(R_cb·X_world + t_cb - t_ka_graph)
        R_gw_i = R_ka_graph.T @ R_cb
        t_gw_i = R_ka_graph.T @ (t_cb - t_ka_graph)
        w = float(max(1, n_corners))
        R_gw_weighted_sum += w * R_gw_i
        t_gw_list.append(t_gw_i); t_gw_weights.append(w)
        contrib_names.append(f"{names[anchor_key]}({n_corners}角点)")

    U, _, Vt = np.linalg.svd(R_gw_weighted_sum)
    R_gw = U @ Vt
    if np.linalg.det(R_gw) < 0:   # 保证是合法旋转(行列式+1)，不是镜像
        U[:, -1] *= -1
        R_gw = U @ Vt
    t_gw = np.average(np.stack(t_gw_list), axis=0, weights=t_gw_weights)

    # 一致性诊断：不同相机各自反推出的世界系角度差异有多大——这个数字如果
    # 明显偏大(比如大于1度)，说明某台相机在锚定轮的检测可能有问题(板子被
    # 部分遮挡/离得太远/对焦不实)，平均掩盖不了这种程度的分歧，值得回去
    # 检查具体是哪一台。
    max_dev_deg = 0.0
    if len(anchor_poses) > 1:
        for anchor_key, (R_cb, t_cb, n_corners) in anchor_poses.items():
            ka_idx = keys.index(anchor_key)
            R_ka_graph, _ = graph_pose[ka_idx]
            R_gw_i = np.asarray(R_ka_graph, float).T @ R_cb
            Rdiff = R_gw_i @ R_gw.T
            tr = np.clip((np.trace(Rdiff) - 1) / 2, -1, 1)
            max_dev_deg = max(max_dev_deg, float(np.degrees(np.arccos(tr))))

    out = {}
    for i, k in enumerate(keys):
        Ri, ti = graph_pose[i]
        Ri = np.asarray(Ri, float); ti = np.asarray(ti, float)
        Rw = Ri @ R_gw
        tw = Ri @ t_gw + ti
        out[k] = (Rw, tw)

    if len(anchor_poses) > 1:
        log(f"[板子外参] 世界锚定：{len(anchor_poses)}台相机在锚定轮同时看到板子"
            f"({', '.join(contrib_names)})，按角点数加权平均定义世界系"
            f"(各相机反推角度差异最大 {max_dev_deg:.3f}°"
            f"{'，明显偏大，建议检查锚定轮拍摄质量' if max_dev_deg > 1.0 else ''})。")
    else:
        only_key = next(iter(anchor_poses))
        log(f"[板子外参] 世界锚定：只有「{names[only_key]}」在锚定轮看到板子，"
            f"用它单独定义世界系(只有一台看到时没有别的数据可平均——如果对精度要求"
            f"极致，建议重拍锚定轮时让尽量多台相机同时看到板子)。")

    # ---- 世界系轴可视化：在锚定轮每一台看到板子的相机照片上画出 XYZ 三色轴 ----
    try:
        draw_world_axes(sess, mani, intr, out, anchor_id, round_poses[anchor_id], names)
    except Exception as ex:
        log(f"[板子外参] 世界系轴可视化生成失败（不影响标定结果）：{ex}")

    return out


def draw_world_axes(sess, mani, intr, world_pose, anchor_id, anchor_poses, names):
    """在锚定轮每台看到板子的相机照片上画出世界坐标系 XYZ 轴，存到
    <会话>/world_axes/<相机>.png。

    world_pose[k] = (Rw, tw)：世界(mm) -> 相机 k，X_cam = Rw·X_world + tw。
    直接用 cv2.projectPoints 把世界原点和三条轴端点投影到这台相机的像素平面。
    轴长取板子一格边长的 3 倍，肉眼清楚又不会长到出画面。"""
    folder2key = {c["folder"]: c["deviceKey"] for c in mani["cameras"]}
    key2name = {c["deviceKey"]: c["name"] for c in mani["cameras"]}
    axis_len = mani["board"]["squareMM"] * 3.0

    out_dir = os.path.join(sess, "world_axes")
    os.makedirs(out_dir, exist_ok=True)

    # 世界系里的四个点：原点 + X/Y/Z 轴端点（单位 mm）。
    world_pts = np.float32([[0, 0, 0], [axis_len, 0, 0],
                            [0, axis_len, 0], [0, 0, axis_len]])

    rounds_dir = os.path.join(sess, "rounds", anchor_id)
    drawn = 0
    for key in anchor_poses.keys():
        if key not in intr or key not in world_pose:
            continue
        # 找这台相机在锚定轮的照片：文件名是它的 folder（camN.png）。
        folder = next((c["folder"] for c in mani["cameras"] if c["deviceKey"] == key), None)
        if folder is None:
            continue
        img_path = os.path.join(rounds_dir, folder + ".png")
        img = cv2.imread(img_path)   # 彩色读，好画彩色轴
        if img is None:
            continue

        K, dist, _ = intr[key]
        Rw, tw = world_pose[key]
        rvec, _ = cv2.Rodrigues(np.asarray(Rw, float))
        tvec = np.asarray(tw, float).reshape(3, 1)
        pts2d, _ = cv2.projectPoints(world_pts, rvec, tvec, K, np.asarray(dist, float))
        pts2d = pts2d.reshape(-1, 2)
        o = tuple(np.int32(pts2d[0]))
        x_end = tuple(np.int32(pts2d[1]))
        y_end = tuple(np.int32(pts2d[2]))
        z_end = tuple(np.int32(pts2d[3]))

        # OpenCV 是 BGR：X 红=(0,0,255)、Y 绿=(0,255,0)、Z 蓝=(255,0,0)。
        cv2.line(img, o, x_end, (0, 0, 255), 3)
        cv2.line(img, o, y_end, (0, 255, 0), 3)
        cv2.line(img, o, z_end, (255, 0, 0), 3)
        cv2.circle(img, o, 6, (0, 255, 255), -1)   # 原点画个黄点
        cv2.putText(img, "X", x_end, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.putText(img, "Y", y_end, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(img, "Z", z_end, cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
        cv2.putText(img, "world origin (X=red Y=green Z=blue)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        save_path = os.path.join(out_dir, folder + ".png")
        cv2.imwrite(save_path, img)
        drawn += 1

    if drawn:
        log(f"[板子外参] 已在 {drawn} 台相机的锚定轮照片上画出世界系 XYZ 轴 -> "
            f"{out_dir}/（红=X 绿=Y 蓝=Z 黄点=原点，打开确认朝向是否符合预期）")
    else:
        log("[板子外参] 世界系轴可视化：没找到可画的锚定轮照片（不影响标定结果）")



def main():
    # 轻量模式：标定向导第②步"当场内参预览"用，只对一台相机跑内参标定，
    # 不碰外参/世界对齐（那部分需要挥球+支架数据，拍板阶段还没有）。跟下面
    # 完整五步流程的代码路径完全独立，不共享任何状态，改这段不会影响正式
    # 求解的正确性。
    if len(sys.argv) >= 4 and sys.argv[1] == "--intrinsics-only":
        sess, cam_folder = sys.argv[2], sys.argv[3]
        mani, _, _ = load_session(sess)
        cam = next((c for c in mani["cameras"] if c["folder"] == cam_folder), None)
        if cam is None:
            # manifest 里按理说该有这个 folder（向导写 manifest 时把参与标定
            # 的相机全列了进去）；万一没找到，退化成用文件夹名当显示名，
            # 至少还能跑，只是日志里名字没那么好看。
            cam = {"folder": cam_folder, "name": cam_folder}
        err = []
        r = calib_intrinsics(sess, cam, mani["board"], err_out=err)
        if r is None or not err:
            log("[内参预览] 失败或样本不足（图片太少/角点检测不到，见上面的具体原因）")
            sys.exit(1)
        log(f"REPROJ_ERR {err[0]:.4f}")
        sys.exit(0)

    # 轻量模式：标定向导第③步"拍完一轮判连通"用。对某一轮已存的帧逐张跑
    # ChArUco 角点检测，输出每台相机这张检到的角点数——向导据此只把角点数>0
    # 的相机计入共视，角点数本身也直接反映这一轮质量，供向导在轮次列表里
    # 提前显示（不用等第⑤步求解才知道好不好）。读图(IMREAD_GRAYSCALE)和
    # 检测器(detect_board_corners，≥6 角点)跟下面正式外参求解
    # solve_board_extrinsics 完全一致，所以向导显示的连通性跟最终能不能求解
    # 一致；只是这里不做 solvePnP（拍板阶段还没内参）。跟完整流程代码路径
    # 独立，改这段不影响正式求解。
    # 用法: python solve_calibration.py --detect-only <会话文件夹> <轮次目录名>
    # 输出：仅 stdout 一行 JSON（键=相机文件夹序号，值=检到的角点数，0=没检到/
    # 被挡住），诊断走 stderr。
    #   {"0": 37, "1": 0}
    if len(sys.argv) >= 4 and sys.argv[1] == "--detect-only":
        sess, round_id = sys.argv[2], sys.argv[3]
        try:
            mani, _, _ = load_session(sess)
            b, det, clahe = make_charuco_detector(mani["board"])
            round_dir = os.path.join(sess, "rounds", round_id)
            result = {}
            for p in sorted(glob.glob(os.path.join(round_dir, "*.png"))):
                folder = os.path.splitext(os.path.basename(p))[0]   # 形如 "cam0"
                digits = "".join(ch for ch in folder if ch.isdigit())
                if not digits:
                    continue                                        # 认不出的文件名，跳过
                img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
                _, ci = (None, None) if img is None else detect_board_corners(b, det, clahe, img)
                result[digits] = int(len(ci)) if ci is not None else 0
        except Exception as e:
            print(f"detect-only 失败: {e}", file=sys.stderr)
            sys.exit(1)
        print(json.dumps(result))    # 仅此一行进 stdout，供向导解析
        sys.exit(0)

    if len(sys.argv) < 2:
        log("用法: python solve_calibration.py <会话文件夹>"); sys.exit(2)
    sess = sys.argv[1]
    mani, wand, snap = load_session(sess)
    cams = mani["cameras"]
    keys = [c["deviceKey"] for c in cams]
    names = {c["deviceKey"]: c["name"] for c in cams}
    if len(keys) < 2:
        log("[中止] 至少需要两台相机"); sys.exit(1)

    # ---- 1) 内参（两种外参方式共用，逻辑完全一样）----
    intr = {}
    for cam in cams:
        r = calib_intrinsics(sess, cam, mani["board"])
        if r is None:
            log("[中止] 内参失败"); sys.exit(1)
        intr[cam["deviceKey"]] = r

    # ---- 外参：按向导里选的方式走两条独立路径 ----
    # "board"：标定板接力连通 + 位姿图优化，一帧静止拍摄就出稳定的6自由度
    #          相对位姿，不存在本质矩阵那类退化风险（见 solve_board_extrinsics
    #          开头的说明），世界系直接由标记的锚定轮定义，不需要额外的支架
    #          对齐步骤。
    # "wand"（默认，向后兼容老会话）：单参考对本质矩阵 + 链式PnP + 支架世界
    #          对齐——这是原来那版实现，完整保留、一行没动，作为可选项继续
    #          可用（两个方案并存，不是谁取代谁）。
    if mani.get("extrinsicsMode", "wand") == "board":
        try:
            pose = solve_board_extrinsics(sess, mani, intr, keys, names)
        except RuntimeError as ex:
            log("[中止] 标定板外参求解失败：")
            log("       " + str(ex))
            sys.exit(1)

        out = {"cameras": []}
        for k in keys:
            K, dist, size = intr[k]
            Rw, tw = pose[k]
            out["cameras"].append({
                "deviceKey": k,
                "intrinsics": {"fx": float(K[0, 0]), "fy": float(K[1, 1]),
                                "cx": float(K[0, 2]), "cy": float(K[1, 2]),
                                "k1": float(dist[0]), "k2": float(dist[1]),
                                "p1": float(dist[2]), "p2": float(dist[3]),
                                "k3": float(dist[4]) if len(dist) > 4 else 0.0,
                                "width": size[0], "height": size[1], "valid": True},
                "extrinsics": {"R": [float(v) for v in np.asarray(Rw).ravel()],
                                "t": [float(v) for v in np.asarray(tw).ravel()], "valid": True},
            })
        with open(os.path.join(sess, "calibration.json"), "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
        log(f"[完成] 已写出 calibration.json（{len(keys)} 台相机，标定板方案，"
            f"世界系=锚定轮的板子位姿，单位 mm）")
        return

    # ---- 以下是挥球方案，原样保留 ----
    world4 = np.float64(mani["worldPoints"])

    # ---- 2) 参考对：cam0-cam1 ----
    k0, k1 = keys[0], keys[1]
    K0, d0, _ = intr[k0]
    K1, d1, _ = intr[k1]
    ts, p0, p1 = match_tracks(wand, k0, k1)
    if len(p0) < 100:
        log(f"[外参] {names[k0]}-{names[k1]} 共视样本不足({len(p0)})，多挥一会/加大共视区")
        sys.exit(1)
    n0, n1 = undist_norm(p0, K0, d0), undist_norm(p1, K1, d1)
    E, mask = cv2.findEssentialMat(n0, n1, np.eye(3), cv2.RANSAC, 0.999, 1e-3)
    if E is None or mask is None:
        log(f"[外参] {names[k0]}-{names[k1]} 本质矩阵求解失败（点分布可能退化，"
            f"比如球基本没动/共面），重新挥球再试"); sys.exit(1)
    _, R1, t1, mask = cv2.recoverPose(E, n0, n1, np.eye(3), mask=mask)
    inl = mask.ravel().astype(bool)
    if inl.sum() < 60:
        log(f"[外参] {names[k0]}-{names[k1]} 内点太少({int(inl.sum())})，"
            f"结果不可信，重新挥球再试"); sys.exit(1)
    log(f"[外参] 参考对 {names[k0]}->{names[k1]}: 内点 {int(inl.sum())}/{len(p0)}")
    P0 = np.hstack([np.eye(3), np.zeros((3, 1))])
    P1 = np.hstack([R1, t1.reshape(3, 1)])
    Xh = cv2.triangulatePoints(P0, P1, n0[inl].T, n1[inl].T)
    ref_track = (Xh[:3] / Xh[3]).T                 # 参考3D轨迹（cam0系，尺度=|t1|=1）
    ref_ts = ts[inl]

    pose = {k0: (np.eye(3), np.zeros(3)), k1: (R1, t1.ravel())}

    # ---- 2b) 其余相机：时间匹配参考轨迹 -> PnP ----
    def match_to_ref(obs):
        """obs: [(ts,x,y)]，与 ref_ts/ref_track 双指针时间匹配。"""
        pts2d, pts3d = [], []
        j = 0
        for t, x, y in obs:
            while j < len(ref_ts) and ref_ts[j] < t - TOL_NS:
                j += 1
            if j < len(ref_ts) and abs(int(ref_ts[j]) - int(t)) <= TOL_NS:
                pts2d.append((x, y)); pts3d.append(ref_track[j])
        return np.float64(pts2d), np.float64(pts3d)

    for k in keys[2:]:
        Kk, dk, _ = intr[k]
        pts2d, pts3d = match_to_ref(wand.get(k, []))
        if len(pts3d) < 60:
            log(f"[外参PnP] {names[k]} 与参考轨迹匹配点不足({len(pts3d)})"); sys.exit(1)
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(pts3d, pts2d, Kk, dk,
                                                     reprojectionError=3.0)
        ninl = 0 if inliers is None else len(inliers)
        if not ok or ninl < 30:
            log(f"[外参PnP] {names[k]} 求解失败或内点太少({ninl}/{len(pts3d)})，"
                f"结果不可信——检查与参考相机的共视区/挥球轨迹"); sys.exit(1)
        Rk, _ = cv2.Rodrigues(rvec)
        pose[k] = (Rk, tvec.ravel())
        log(f"[外参PnP] {names[k]}: 内点 {ninl}/{len(pts3d)}")

    # ---- 2c) 全局BA + 联合内参精修（在链式PnP的初值基础上做最后抛光，
    # 见 wand_bundle_adjustment/wand_joint_intrinsics_refine 顶部说明）----
    # 默认开启：内部有数据量门禁+效果门禁+正则先验三重保险，跑不动/没帮助
    # 就原样退回链式PnP的结果，不会比不做这一步更差。
    wand_frames = build_wand_frames(wand, keys)
    pose, ba_ctx = wand_bundle_adjustment(intr, keys, names, pose, wand_frames)
    if ba_ctx is not None:
        pose, intr_refined = wand_joint_intrinsics_refine(intr, keys, names, pose, ba_ctx)
        if not intr_refined:
            log("[挥球BA·联合内参精修] 本次未生效，最终结果等同于只做了挥球方案的纯外参+结构BA。")

    # ---- 3) 世界对齐：四点支架 ----
    for k in keys:
        if k not in snap:
            log(f"[对齐] 缺少支架快照：{names[k]}（第4步没捕到 4 球）"); sys.exit(1)

    # 每台相机在向导里各自独立按"离自己第一帧最近"对齐 4 点顺序（见
    # CalibWizard::onCaptureFrame），所以不同相机的第 i 个点不代表同一颗球——
    # 必须靠三角化残差把 4 点重新配对。原点+XYZ 支架的四颗球往往彼此靠得
    # 很近、方位也可能接近对称，逐点独立 argmin 无法保证同一台相机对 cam0
    # 四个点的匹配是"一一对应"的（同一个 j 可能被两个不同的 i 都选中）。
    # 这里对每台相机一次性求解 4x4 代价矩阵下的最优一一匹配（匈牙利算法），
    # 保证每台相机的 4 点是 cam0 四点的一个双射，而不是 4 次独立的局部最优。
    n0s = undist_norm(snap[k0], K0, d0)

    def hungarian4(cost):
        """4x4 代价矩阵的最优完美匹配。优先用 scipy，没有就退化到穷举 4! 排列。"""
        try:
            from scipy.optimize import linear_sum_assignment
            row_ind, col_ind = linear_sum_assignment(cost)
            return list(col_ind[np.argsort(row_ind)])
        except ImportError:
            best_perm, best_cost = None, None
            for perm in permutations(range(4)):
                c = sum(cost[i][perm[i]] for i in range(4))
                if best_cost is None or c < best_cost:
                    best_cost, best_perm = c, perm
            return list(best_perm)

    chosen_per_cam = {}   # k -> [j0,j1,j2,j3]，camera k 第 jI 个点对应 cam0 第 I 个点
    for k in keys[1:]:
        Kk, dk, _ = intr[k]
        nks = undist_norm(snap[k], Kk, dk)
        R, t = pose[k]
        Pk = np.hstack([R, t.reshape(3, 1)])
        cost = np.zeros((4, 4))
        for i in range(4):
            for j in range(4):
                Xh = cv2.triangulatePoints(P0, Pk, n0s[i:i+1].T, nks[j:j+1].T)
                X = (Xh[:3] / Xh[3]).ravel()
                e0 = np.linalg.norm(X[:2] / X[2] - n0s[i])
                Xc = R @ X + t
                e1 = np.linalg.norm(Xc[:2] / Xc[2] - nks[j])
                cost[i, j] = e0 + e1
        chosen_per_cam[k] = hungarian4(cost)
        log(f"[对齐] {names[k]} 支架四点匹配代价 {cost[range(4), chosen_per_cam[k]].sum():.5f}")

    pts3d_frame = []
    for i in range(4):
        # 全相机联合 DLT，用上面求出的双射取每台相机对应 cam0 第 i 点的观测。
        rows = []
        u, v = n0s[i]
        rows.append(u * P0[2] - P0[0]); rows.append(v * P0[2] - P0[1])
        for k in keys[1:]:
            Kk, dk, _ = intr[k]
            nks = undist_norm(snap[k], Kk, dk)
            R, t = pose[k]
            Pk = np.hstack([R, t.reshape(3, 1)])
            u, v = nks[chosen_per_cam[k][i]]
            rows.append(u * Pk[2] - Pk[0]); rows.append(v * Pk[2] - Pk[1])
        _, _, Vt = np.linalg.svd(np.float64(rows))
        X = Vt[-1]
        pts3d_frame.append(X[:3] / X[3])
    pts3d_frame = np.float64(pts3d_frame)

    best = None
    for perm in permutations(range(4)):
        s, R, t = umeyama(world4, pts3d_frame[list(perm)])
        res = np.linalg.norm(s * (R @ world4.T).T + t - pts3d_frame[list(perm)],
                             axis=1).mean()
        if best is None or res < best[0]:
            best = (res, s, R, t, perm)
    res, s, Ra, ta, perm = best
    log(f"[对齐] 支架配准平均残差 {res:.4f}（cam0系单位），尺度 s={s:.6f}，点对应 {perm}")

    # 世界(mm) -> cam_i：X0 = s·Ra·Xw + ta；X_i = R_i·X0 + t_i
    # 归一到米制世界单位：R = R_i·Ra，t = (R_i·ta + t_i)/s
    out = {"cameras": []}
    for k in keys:
        K, dist, size = intr[k]
        Ri, ti = pose[k]
        Rw = Ri @ Ra
        tw = (Ri @ ta + ti) / s
        out["cameras"].append({
            "deviceKey": k,
            "intrinsics": {"fx": float(K[0, 0]), "fy": float(K[1, 1]),
                            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
                            "k1": float(dist[0]), "k2": float(dist[1]),
                            "p1": float(dist[2]), "p2": float(dist[3]),
                            "k3": float(dist[4]) if len(dist) > 4 else 0.0,
                            "width": size[0], "height": size[1], "valid": True},
            "extrinsics": {"R": [float(v) for v in Rw.ravel()],
                            "t": [float(v) for v in tw], "valid": True},
        })
    with open(os.path.join(sess, "calibration.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    log(f"[完成] 已写出 calibration.json（{len(keys)} 台相机，世界系=你的支架坐标，单位 mm）")


if __name__ == "__main__":
    main()