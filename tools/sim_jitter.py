"""
遮挡预测跳变量化台 —— 闭环跑真模型，量"被遮挡的点抖多少"。

和训练里的评估不同，这里刻意做成【连续序列 + 闭环 prev】：
  · 姿势在关键帧之间平滑插值，相邻帧差很小 —— 真实录制就是这样
  · prev 喂【上一帧模型自己的 pos 输出】，不是真值扰动
    训练时 prev 是"同受试者另一姿势混一点"，闭环时是"自己上一帧的输出"，
    这两者的分布不一样，而线上跑的是后者。跳变问题只在闭环里才暴露。
  · 相机组固定、世界朝向固定 —— 把无关变量全钉死，只留姿势在动

指标（全部按 mm）：
  jump   = ||pos_t - pos_{t-1}||   帧间跳变，这是"抖不抖"
  err    = ||pos_t - gt_t||        绝对误差，这是"准不准"
按 观测/遮挡 和 拇指/四指 分层 —— 用户报告的正是"拇指遮挡时跳"。
"""
from __future__ import annotations

import sys
import numpy as np
import onnxruntime as ort

sys.path.insert(0, "/home/claude/ai/ai")
from hand_rig import sample_subjects, forward_kinematics, rest_pose_markers
from pose_prior import LIMITS_LO, LIMITS_HI, rest_pose
from synth import template_feature
from occlusion import sample_cameras, visibility
from solve import solve_hand

MODEL = "/mnt/user-data/uploads/1786716425597_hm20_v6_sk10.onnx"
THUMB = [5, 6, 7]
FINGERS = list(range(8, 20))
DORSUM = list(range(5))


def smooth_sequence(rng, n_frames, hold=12):
    """开→握→开 的平滑序列。关键帧之间用余弦插值，保证相邻帧差很小。"""
    open_ = rest_pose(1)[0].copy()
    fist = np.clip(LIMITS_LO[0] * 0 + np.array([
        [0.35, 0.18, 0.80, 1.10],      # thumb
        [1.35, 0.05, 1.70, 1.10],
        [1.40, 0.02, 1.75, 1.10],
        [1.40, -0.02, 1.75, 1.10],
        [1.40, -0.05, 1.70, 1.10],
    ]), LIMITS_LO, LIMITS_HI)
    thumb_out = open_.copy(); thumb_out[0] = [0.20, 1.05, 0.10, 0.15]   # 拇指张成 L 形
    keys = [open_, thumb_out, open_, fist, open_]
    seq = []
    per = max(2, (n_frames - hold * len(keys)) // (len(keys) - 1))
    for k in range(len(keys) - 1):
        a, b = keys[k], keys[k + 1]
        for i in range(per):
            u = 0.5 - 0.5 * np.cos(np.pi * i / per)      # 余弦缓入缓出
            seq.append(a + (b - a) * u)
        for _ in range(hold):
            seq.append(b.copy())
    return np.array(seq[:n_frames], dtype=np.float64)


def run(n_frames=240, n_cam=3, noise_mm=0.9, seed=7, left_hand=True, verbose=True,
        mode='closed', tag='', decay_n=8, ema_a=0.35, calibrated=True, freeze_n=10,
        a_fast=0.9, a_slow=0.05, v_a=0.3, v_tau=6.0, trend_w=0.6,
        tau_w=8.0, a_out=0.25, axis_hold=False, ax_max=48):
    rng = np.random.default_rng(seed)
    sub = sample_subjects(rng, 1, left_hand_p=1.0 if left_hand else 0.0)
    sub1 = sub                      # size=1，solve_hand 要的就是这个
    if not calibrated:
        # 【免标定档】用另一个随机受试者当"群体均值手型"喂给 solve_hand。
        # 真机上没做手指标定时就是这个处境：手型是猜的，不是这只手的。
        # 只把手性对齐（手性是知道的），其余全用别人的尺寸。
        m = sample_subjects(np.random.default_rng(seed + 991), 1,
                            left_hand_p=1.0 if left_hand else 0.0)
        sub1 = m
    rest, _, _, _ = rest_pose_markers(sub)
    tmpl = template_feature(rest, sub.sign).astype(np.float32)      # (1,61)

    # 只发手背 5 点有效，跟 C++ 侧一致（手指标定做不到）
    tv = np.zeros((1, 20), np.float32)
    tv[0, :5] = 1.0

    ang = smooth_sequence(rng, n_frames)                            # (T,5,4)
    T = len(ang)
    big = type(sub)(**{k: np.repeat(getattr(sub, k), T, axis=0)
                       for k in sub.__dataclass_fields__})
    mk, segR, segO, joints = forward_kinematics(big, ang)           # 腕部局部系

    cams = sample_cameras(rng, 1, n_cam=n_cam)
    cams = np.repeat(cams, T, axis=0)
    vis, nviews = visibility(mk.astype(np.float32), segO.astype(np.float32),
                             segR.astype(np.float32), joints.astype(np.float32), cams)
    vis = vis.astype(bool)

    sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
    prev = mk[0].astype(np.float32).copy()          # 第一帧用真值起步
    prev_mask = np.ones((1, 20), np.float32)
    stale = np.zeros(20, np.int32)
    last_seen_pos = mk[0].astype(np.float32).copy()
    ema_state = mk[0].astype(np.float32).copy()
    out_state = mk[0].astype(np.float32).copy()
    stale_o = np.zeros(20, np.float64)
    obs_p = mk[0].astype(np.float32).copy()
    obs_t = np.zeros(20, np.float64)
    vel = np.zeros((20, 3), np.float32)
    shape = np.zeros((5, 3, 3), np.float32); anch = np.zeros((5, 3), np.float32)
    anch_p = np.zeros((5, 3), np.float32); vel_f = np.zeros((5, 3), np.float32)
    age_f = np.zeros(5, np.float64); have_f = np.zeros(5, bool)
    ax_last = np.zeros((5,3), np.float64); ax_have = np.zeros(5, bool)
    ax_age = np.zeros(5, np.int64)

    rec = []
    last_pos = None
    last_ang = None
    fk_out = None
    for t in range(T):
        seen = vis[t]
        pts = mk[t][seen] + rng.normal(0, noise_mm, (seen.sum(), 3))
        if len(pts) < 3:
            rec.append(None)
            continue
        out = sess.run(None, {
            "points": pts[None].astype(np.float32),
            "mask": np.ones((1, len(pts)), np.float32),
            "tmpl": tmpl,
            "tmpl_valid": tv,
            "prev": prev[None].astype(np.float32),
            "prev_mask": prev_mask,
        })
        pos = out[1][0]                              # (20,3)

        # ---- FK 重建：拿【本帧观测】解姿态再摆出全部 20 点 ----
        # 【为什么这条有希望】它的输入完全来自当前帧的观测，不含任何历史误差，
        # 所以能真正切断"自己的误差喂回自己"这条正反馈。
        # ---- C 路线：用"最近一次实测 + 运动趋势"外推，完全不信网络的遮挡预测 ----
        # 【思路来源：用户】遮挡点的网络预测是外推，而"上一次真看见它时它在哪、
        # 往哪个方向以多快速度动"是【实测】。与其信一个没有观测约束的预测，
        # 不如信一段刚刚测到的运动，再让它随时间衰减到停住。
        # 速度在腕部系里估 —— 否则整只手平移会被算进手指的运动趋势里。
        if mode in ('rigid', 'rigid_fuse'):
            # ---- 整指刚体外推：保形，不逐点各走各的 ----
            # 【为什么必须保形】骨轴方向是【两点之差】，不是位置本身。两颗球各自
            # 平滑外推、各自误差都不大，差分方向照样可以偏几十度 —— 中节骨轴
            # (pp->dp) 两球才三四十毫米，各偏 10mm 就是几十度。
            # 所以外推的对象改成"最后一次实测时这根手指的三点构型"：它天然满足
            # 骨长和关节角约束，整体跟着运动趋势走，形状不变 => 骨轴方向不会崩。
            for f in range(5):
                ids = [5 + 3*f, 6 + 3*f, 7 + 3*f]
                vis_f = [i for i in ids if seen[i]]
                occ_f = [i for i in ids if not seen[i]]
                if not occ_f:
                    shape[f] = pos[ids] - pos[ids].mean(0)      # 全可见：更新构型
                    anch[f] = pos[ids].mean(0)
                    vel_f[f] = vel_f[f]*(1-v_a) + (anch[f]-anch_p[f])*v_a
                    anch_p[f] = anch[f].copy(); age_f[f] = 0
                    have_f[f] = True
                    continue
                if not have_f[f]:
                    continue                                    # 没有构型可用
                age_f[f] += 1
                a = age_f[f]
                if vis_f:
                    # 还有球可见：用可见球把构型"钉"回去（平移对齐），比纯外推准
                    off = np.mean([pos[i] - (anch[f] + shape[f][ids.index(i)])
                                   for i in vis_f], axis=0)
                    ctr = anch[f] + off
                else:
                    ctr = anch[f] + vel_f[f] * a * np.exp(-a / float(v_tau))
                for i in occ_f:
                    ext = ctr + shape[f][ids.index(i)]
                    if mode == 'rigid':
                        pos[i] = ext
                    else:
                        w = np.exp(-a / float(tau_w))
                        mix = ext * w + pos[i] * (1 - w)
                        pos[i] = out_state[i] * (1 - a_out) + mix * a_out

        if mode == 'fuse':
            # 【融合】按遮挡时长在"趋势"和"网络"之间过渡，然后【再统一低通】。
            # 上一轮 trend_blend 直接按固定比例混，跳变 max 反而 27.8 —— 因为网络
            # 那一份每帧在跳，混进来就把解析平滑的轨迹污染了。
            # 关键是【混完再滤】：低通把网络带进来的抖动削掉，同时保留它的长期锚定。
            #   刚遮挡：速度是刚测到的，趋势最准 -> 偏趋势
            #   遮挡久：趋势已衰减到停住、开始漂 -> 让网络接管
            for m in range(20):
                if seen[m]:
                    dtf = max(1, t - int(obs_t[m]))
                    vel[m] = vel[m] * (1 - v_a) + ((pos[m] - obs_p[m]) / dtf) * v_a
                    obs_p[m] = pos[m]; obs_t[m] = t
                else:
                    age = t - obs_t[m]
                    ext = obs_p[m] + vel[m] * age * np.exp(-age / float(v_tau))
                    w = np.exp(-age / float(tau_w))          # 1=全趋势 0=全网络
                    mix = ext * w + pos[m] * (1 - w)
                    pos[m] = out_state[m] * (1 - a_out) + mix * a_out   # 混完再滤

        if mode in ('trend', 'trend_blend'):
            for m in range(20):
                if seen[m]:
                    dtf = max(1, t - int(obs_t[m]))
                    v_new = (pos[m] - obs_p[m]) / dtf
                    vel[m] = vel[m] * (1 - v_a) + v_new * v_a     # 速度本身也要平滑
                    obs_p[m] = pos[m]; obs_t[m] = t
                else:
                    age = t - obs_t[m]
                    damp = np.exp(-age / float(v_tau))            # 越久越停住
                    ext = obs_p[m] + vel[m] * age * damp
                    pos[m] = ext if mode == 'trend' else \
                             ext * trend_w + pos[m] * (1 - trend_w)

        # ---- B 路线：免标定，只压跳变，不追求准 ----
        # 【为什么这样设计】遮挡点的位置信息本来就不够，追准是追不到的。
        # 但"不跳"是能做到的：遮挡越久越不信新预测，直到完全冻住。
        # 冻住的代价是遮挡期间那个点不动，可它本来也没有可信位置，
        # segSource=预测 已经把这件事告诉下游了。
        if mode in ('smooth', 'freeze', 'adapt'):
            sm = pos.copy()
            for m in np.where(~seen)[0]:
                if mode == 'smooth':
                    a = ema_a
                else:
                    # 【方向修正】刚丢的头几帧几乎不平滑 —— 那时上一帧的估计还很新鲜，
                    # 强行平滑只会让它跟不上真实运动(四指的短时遮挡就毁在这)。
                    # 丢得越久越往冻结走 —— 那时已经没有可信位置，乱飞只有坏处。
                    k = min(stale_o[m] / float(freeze_n), 1.0)
                    a = a_fast + (a_slow - a_fast) * k
                sm[m] = out_state[m] * (1.0 - a) + pos[m] * a
            pos = sm
        out_state[seen] = pos[seen]
        out_state[~seen] = pos[~seen]
        stale_o[seen] = 0
        stale_o[~seen] += 1

        if mode in ('fk_prev', 'fk_out'):
            lab = np.where(seen)[0]
            r = solve_hand(mk[t][seen] + rng.normal(0, noise_mm, (seen.sum(), 3)),
                           lab, sub1, prior_angles=last_ang)
            if r.get("ok", True) and "markers" in r:
                fk_out = np.asarray(r["markers"], np.float32)
                last_ang = np.asarray(r["angles"], float).reshape(5, 4) if "angles" in r else last_ang
            if mode == 'fk_out' and fk_out is not None:
                pos = pos.copy()
                pos[~seen] = fk_out[~seen]           # 遮挡点直接用 FK 摆出来的
        jump = None if last_pos is None else np.linalg.norm(pos - last_pos, axis=-1)
        err = np.linalg.norm(pos - mk[t], axis=-1)
        # 中节骨轴 = pp->dp（marker 5+3f 与 5+3f+2 的差分），跟 C++ 的取法一致
        axerr = []
        for f in range(5):
            a, b = 5 + 3*f, 7 + 3*f
            v1 = pos[b] - pos[a]; v2 = mk[t][b] - mk[t][a]
            n1 = np.linalg.norm(v1); n2 = np.linalg.norm(v2)
            if n1 < 1e-6 or n2 < 1e-6:
                axerr.append(np.nan); continue
            u1 = v1 / n1
            both = (not seen[a]) and (not seen[b])
            if axis_hold:
                # 【两端都遮挡就冻住方向】真值的骨轴帧间转角中位 0°、p99 6°，
                # 而两球基线只有 40~55mm、遮挡点位置误差 10~30mm ——
                # 用它重算方向必然引入 20~40° 噪声。信息不够时保持，不猜。
                if both and ax_have[f] and ax_age[f] < ax_max:
                    u1 = ax_last[f]; ax_age[f] += 1
                elif not both:
                    ax_last[f] = u1; ax_have[f] = True; ax_age[f] = 0
                else:
                    ax_age[f] += 1
            c = float(np.clip(np.dot(u1, v2 / n2), -1, 1))
            axerr.append(np.degrees(np.arccos(c)))
        both_occ = [(not seen[5+3*f]) and (not seen[7+3*f]) for f in range(5)]
        rec.append(dict(seen=seen.copy(), jump=jump, err=err,
                        axerr=np.array(axerr), both_occ=np.array(both_occ)))
        last_pos = pos.copy()
        # mode: closed=把自己的输出整份喂回去(线上现状)
        #       open  =喂真值(上界，用来分离"模型弱"和"闭环发散")
        #       gated =只喂本帧真观测到的点，遮挡点 prev_mask=0(候选修法)
        if mode == 'open':
            # 【上一帧】真值 = "完美跟踪"的真实上界。喂当前帧真值是作弊：
            # 遮挡点的答案就在输入里，模型抄一遍就有 3mm，那个数没有意义。
            prev = mk[t].astype(np.float32).copy(); prev_mask = np.ones((1,20), np.float32)
        elif mode == 'gt_prev':
            prev = mk[max(0, t-1)].astype(np.float32).copy(); prev_mask = np.ones((1,20), np.float32)
        elif mode == 'gated':
            prev = pos.copy(); prev_mask = seen.astype(np.float32)[None]
        else:
            # 遮挡持续帧数：观测到就清零
            stale[seen] = 0
            stale[~seen] += 1
            if mode == 'closed':
                prev = pos.copy(); prev_mask = np.ones((1,20), np.float32)
            elif mode == 'hold':
                # ① 遮挡点的 prev 冻在【最后一次实测位置】，不吃自己的预测
                prev = pos.copy()
                prev[~seen] = last_seen_pos[~seen]
                prev_mask = np.ones((1,20), np.float32)
            elif mode == 'decay':
                # ② prev_mask 按遮挡帧数线性衰减 1.0 -> 0（decay_n 帧衰完）
                prev = pos.copy()
                w = np.clip(1.0 - stale / float(decay_n), 0.0, 1.0)
                prev_mask = w.astype(np.float32)[None]
            elif mode == 'hold_decay':
                # ③ ①+②：冻在最后实测位置，同时权重衰减
                prev = pos.copy()
                prev[~seen] = last_seen_pos[~seen]
                w = np.clip(1.0 - stale / float(decay_n), 0.0, 1.0)
                prev_mask = w.astype(np.float32)[None]
            elif mode == 'fk_prev':
                prev = pos.copy()
                if fk_out is not None: prev[~seen] = fk_out[~seen]
                prev_mask = np.ones((1,20), np.float32)
            elif mode == 'fk_out':
                prev = pos.copy(); prev_mask = np.ones((1,20), np.float32)
            elif mode in ('trend', 'trend_blend', 'fuse', 'rigid', 'rigid_fuse'):
                prev = pos.copy(); prev_mask = np.ones((1,20), np.float32)
            elif mode in ('smooth', 'freeze', 'adapt'):
                prev = pos.copy(); prev_mask = np.ones((1,20), np.float32)
            elif mode == 'ema':
                # ④ 遮挡点回灌前先做时序滤波（一阶低通），观测点原样
                sm = pos.copy()
                sm[~seen] = ema_state[~seen] * (1 - ema_a) + pos[~seen] * ema_a
                ema_state = sm.copy()
                prev = sm; prev_mask = np.ones((1,20), np.float32)
            else:
                prev = pos.copy(); prev_mask = np.ones((1,20), np.float32)
        last_seen_pos[seen] = pos[seen]

    def stat(field, idxs, want_seen):
        v = []
        for r in rec:
            if r is None or r[field] is None:
                continue
            for m in idxs:
                if bool(r["seen"][m]) == want_seen:
                    v.append(r[field][m])
        if not v:
            return None
        v = np.array(v)
        return dict(n=len(v), med=float(np.median(v)), p90=float(np.percentile(v, 90)),
                    p99=float(np.percentile(v, 99)), mx=float(v.max()))

    # 两端球都被遮挡时的骨轴方向误差 —— 这才是"中节转 150°"对应的量
    ax = []
    for r in rec:
        if r is None: continue
        for f in range(5):
            if r["both_occ"][f] and not np.isnan(r["axerr"][f]):
                ax.append(r["axerr"][f])
    res = {}
    res["_axis"] = (dict(n=len(ax), med=float(np.median(ax)),
                         p90=float(np.percentile(ax,90)), mx=float(np.max(ax)))
                    if ax else None)
    for name, idxs in [("拇指", THUMB), ("四指", FINGERS), ("手背", DORSUM)]:
        for seen_flag, tag in [(True, "观测"), (False, "遮挡")]:
            res[f"{name}·{tag}"] = dict(jump=stat("jump", idxs, seen_flag),
                                        err=stat("err", idxs, seen_flag))
    if verbose:
        occ = 1.0 - vis.mean()
        print(f"[{tag or mode}] 相机{n_cam}台 帧数{T} 噪声{noise_mm}mm "
              f"{'左手' if left_hand else '右手'} 遮挡率{occ*100:.1f}%")
        print(f"{'分层':<10}{'n':>6}{'跳变中位':>10}{'p90':>8}{'p99':>8}{'max':>8}"
              f"{'  |':>4}{'误差中位':>10}{'p90':>8}{'max':>8}")
        for k, v in res.items():
            j, e = v["jump"], v["err"]
            if j is None:
                continue
            print(f"{k:<10}{j['n']:>6}{j['med']:>10.2f}{j['p90']:>8.2f}"
                  f"{j['p99']:>8.2f}{j['mx']:>8.2f}{'  |':>4}"
                  f"{e['med']:>10.2f}{e['p90']:>8.2f}{e['mx']:>8.2f}")
    return res


if __name__ == "__main__":
    for nc in (3, 4, 6):
        run(n_cam=nc)
        print()
