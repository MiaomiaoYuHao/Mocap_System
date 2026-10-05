#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# 手指屈曲测试离线分析——配合 ui/FingerFlexionTestDialog.hpp 导出的CSV使用。
#
# 解决的问题：标定/追踪的残差数字（rmsMm之类）再好看，也没法自证"这就是
# 真实的手"——同一批数据能被一个凑巧拟合得很好但实际是错的模型解释，这是
# 数学上的硬限制。这个脚本用的是完全不同维度的检查：录制时人自己知道"现在
# 弯的是哪根手指"，这个标签是模型完全看不到、不依赖这批数据本身的外部信息，
# 拿它去对照追踪端的实际输出，能抓住"该动的没动""不该动的跟着动了(串扰)"
# "动的过程里上蹿下跳(抖动)"这几类数字残差抓不住、但一眼假的错误。
#
# 用法：
#   python3 analyze_finger_flexion_test.py finger_flexion_test_XXXX.csv
#   python3 analyze_finger_flexion_test.py finger_flexion_test_XXXX.csv --plot
#
# 局限（不回避）：这是"抓明显错"的粗筛，不是精度验证——"该弯25度只弯了15度"
# 这种方向对、幅度不准的问题，这个脚本抓不住，得靠标定阶段的切分验证/残差
# 数字去查。这个脚本擅长的是"完全不对"级别的错误。
# -----------------------------------------------------------------------------
import argparse
import csv
import sys
from collections import defaultdict

# 跟 hand/HandModel.hpp::jointLimits() 的16维布局、ui/FingerFlexionTestDialog.cpp
# 导出的列名严格对应。每根手指关心的主信号是 pipFlex(近指间关节屈曲，幅度
# 通常最大最直观)，mcpFlex 当次要信号一起看。拇指没有pip，用mcp+ip。
FINGER_COLUMNS = {
    "thumb":  {"primary": "thumb_ip", "secondary": "thumb_mcp"},
    "index":  {"primary": "index_pipFlex", "secondary": "index_mcpFlex"},
    "middle": {"primary": "middle_pipFlex", "secondary": "middle_mcpFlex"},
    "ring":   {"primary": "ring_pipFlex", "secondary": "ring_mcpFlex"},
    "pinky":  {"primary": "pinky_pipFlex", "secondary": "pinky_mcpFlex"},
}
ALL_FLEX_COLUMNS = {
    "thumb": ["thumb_mcp", "thumb_ip"],
    "index": ["index_mcpFlex", "index_pipFlex"],
    "middle": ["middle_mcpFlex", "middle_pipFlex"],
    "ring": ["ring_mcpFlex", "ring_pipFlex"],
    "pinky": ["pinky_mcpFlex", "pinky_pipFlex"],
}

MIN_FLEX_RANGE_RAD = 0.25       # 约14度——低于这个幅度判定"目标手指基本没动"
CROSSTALK_RATIO_WARN = 0.4      # 其它手指幅度超过目标手指这个比例，判定"疑似串扰"
CROSSTALK_RATIO_FAIL = 0.7      # 超过这个比例，直接判失败（不只是警告）
JITTER_SIGN_CHANGE_RATIO = 0.35 # 一阶差分符号变化次数/总帧数，超过这个比例判"疑似抖动"
JITTER_DEADBAND_RAD = 0.01      # 差分小于这个值不计入符号变化统计(避免把量化噪声当抖动)
MIN_TRACKED_RATIO = 0.8         # 这一段里"追踪到手"的帧比例，低于这个判"追踪丢失严重"


def load_rows(path):
    with open(path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    for r in rows:
        r['ts_ns'] = int(r['ts_ns'])
        r['tracked'] = r['tracked'] == '1'
        for k in list(r.keys()):
            if k not in ('label', 'tracked', 'ts_ns'):
                r[k] = float(r[k]) if r[k] != '' else 0.0
    return rows


def split_segments(rows):
    """按label切段——同一个非idle标签连续出现的一串行，算一段。idle行本身
    不进任何段，只当分隔符用。返回 [(label, [rows...]), ...]，同一根手指
    如果测了好几次(反复弯了几段)，会产生好几个独立的段，各自单独判定。"""
    segments = []
    cur_label, cur_rows = None, []
    for r in rows:
        lbl = r['label']
        if lbl == 'idle' or lbl not in FINGER_COLUMNS:
            if cur_label is not None and cur_rows:
                segments.append((cur_label, cur_rows))
            cur_label, cur_rows = None, []
            continue
        if lbl != cur_label:
            if cur_label is not None and cur_rows:
                segments.append((cur_label, cur_rows))
            cur_label, cur_rows = lbl, []
        cur_rows.append(r)
    if cur_label is not None and cur_rows:
        segments.append((cur_label, cur_rows))
    return segments


def value_range(rows, col):
    vals = [r[col] for r in rows if r['tracked']]
    if not vals:
        return 0.0, 0.0, 0.0
    return max(vals) - min(vals), min(vals), max(vals)


def jitter_ratio(rows, col):
    vals = [r[col] for r in rows if r['tracked']]
    if len(vals) < 3:
        return 0.0
    diffs = [vals[i+1] - vals[i] for i in range(len(vals)-1)]
    signs = [1 if d > JITTER_DEADBAND_RAD else (-1 if d < -JITTER_DEADBAND_RAD else 0) for d in diffs]
    signs = [s for s in signs if s != 0]
    if len(signs) < 2:
        return 0.0
    changes = sum(1 for i in range(len(signs)-1) if signs[i] != signs[i+1])
    return changes / len(signs)


def analyze_segment(label, rows, seg_idx):
    n = len(rows)
    tracked_ratio = sum(1 for r in rows if r['tracked']) / n if n else 0.0
    issues = []
    status = "PASS"

    if tracked_ratio < MIN_TRACKED_RATIO:
        issues.append(f"追踪丢失严重(仅{tracked_ratio*100:.0f}%帧追踪到手，低于{MIN_TRACKED_RATIO*100:.0f}%)")
        status = "FAIL"

    primary_col = FINGER_COLUMNS[label]["primary"]
    secondary_col = FINGER_COLUMNS[label]["secondary"]
    primary_range, _, _ = value_range(rows, primary_col)
    secondary_range, _, _ = value_range(rows, secondary_col)
    target_range = max(primary_range, secondary_range)

    if target_range < MIN_FLEX_RANGE_RAD:
        issues.append(f"目标手指({label})动作幅度只有{target_range:.3f}rad(约{target_range*57.3:.1f}度)，"
                      f"低于{MIN_FLEX_RANGE_RAD:.2f}rad门槛——基本没动，或者追踪没跟上")
        status = "FAIL"

    jitter = jitter_ratio(rows, primary_col)
    if jitter > JITTER_SIGN_CHANGE_RATIO:
        issues.append(f"目标关节({primary_col})一阶差分符号变化率{jitter*100:.0f}%，"
                      f"高于{JITTER_SIGN_CHANGE_RATIO*100:.0f}%——疑似抖动/关联跳变，"
                      f"不是平滑的弯曲过程")
        if status == "PASS":
            status = "WARN"

    crosstalk_notes = []
    if target_range > 1e-6:
        for other_label, cols in ALL_FLEX_COLUMNS.items():
            if other_label == label:
                continue
            other_max_range = max(value_range(rows, c)[0] for c in cols)
            ratio = other_max_range / target_range
            if ratio > CROSSTALK_RATIO_WARN:
                crosstalk_notes.append(f"{other_label}(幅度比{ratio*100:.0f}%)")
    if crosstalk_notes:
        worst_ratio = max(
            max(value_range(rows, c)[0] for c in cols) / target_range
            for ol, cols in ALL_FLEX_COLUMNS.items() if ol != label
        ) if target_range > 1e-6 else 0.0
        sev = "串扰" if worst_ratio <= CROSSTALK_RATIO_FAIL else "严重串扰"
        issues.append(f"疑似{sev}——测{label}时，这些手指也有明显幅度变化：{', '.join(crosstalk_notes)}"
                      f"（注意：无名指/小指之间有一定生理性联动是正常的，"
                      f"轻微幅度不代表bug，这里只是超过{CROSSTALK_RATIO_WARN*100:.0f}%比例才报）")
        if worst_ratio > CROSSTALK_RATIO_FAIL:
            status = "FAIL"
        elif status == "PASS":
            status = "WARN"

    return {
        "label": label, "seg_idx": seg_idx, "n_frames": n,
        "tracked_ratio": tracked_ratio, "target_range_rad": target_range,
        "jitter_ratio": jitter, "status": status, "issues": issues,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv_path")
    ap.add_argument("--plot", action="store_true", help="每段额外画一张关节角随时间变化的图(PNG)，需要matplotlib")
    args = ap.parse_args()

    rows = load_rows(args.csv_path)
    if not rows:
        print("CSV是空的，没有可分析的数据。")
        sys.exit(1)

    segments = split_segments(rows)
    if not segments:
        print("没有找到任何非idle的测试段——检查录制时是不是一直停在“待机”没切换手指。")
        sys.exit(1)

    results = []
    per_finger_counter = defaultdict(int)
    for label, seg_rows in segments:
        per_finger_counter[label] += 1
        results.append(analyze_segment(label, seg_rows, per_finger_counter[label]))

    print(f"\n共 {len(segments)} 个测试段（来自 {len(rows)} 帧原始数据）：\n")
    print(f"{'手指':<8}{'第几次':<8}{'帧数':<8}{'追踪率':<10}{'目标幅度(度)':<14}{'抖动率':<10}{'判定':<8}")
    print("-" * 70)
    name_cn = {"thumb": "拇指", "index": "食指", "middle": "中指", "ring": "无名指", "pinky": "小指"}
    for r in results:
        print(f"{name_cn[r['label']]:<8}{r['seg_idx']:<8}{r['n_frames']:<8}"
              f"{r['tracked_ratio']*100:>6.0f}%   {r['target_range_rad']*57.3:>10.1f}    "
              f"{r['jitter_ratio']*100:>6.0f}%    {r['status']}")
        for issue in r["issues"]:
            print(f"    - {issue}")
    print()

    n_fail = sum(1 for r in results if r["status"] == "FAIL")
    n_warn = sum(1 for r in results if r["status"] == "WARN")
    n_pass = sum(1 for r in results if r["status"] == "PASS")
    print(f"汇总：PASS {n_pass} / WARN {n_warn} / FAIL {n_fail}")
    if n_fail:
        print("有FAIL项——这几段测试没有达到基本的“目标手指真的动了、别的手指没跟着乱动”的门槛，"
              "先看上面具体是哪类问题（没动/严重串扰/追踪丢失）再决定往哪查。")
    elif n_warn:
        print("没有FAIL但有WARN——基本能用，但抖动/轻微串扰的迹象值得留意，"
              "如果实际使用体验也感觉不对，回头看WARN那几行的具体数字。")
    else:
        print("全部通过——至少在“该动的动了、不该动的没乱动、过程不抖”这个粗筛层面上，"
              "追踪端对这几个已知动作的响应是符合预期的。")

    if args.plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n--plot 需要 matplotlib，没装的话先 pip install matplotlib --break-system-packages")
            return
        for label, seg_rows in segments:
            t0 = seg_rows[0]['ts_ns']
            ts = [(r['ts_ns'] - t0) / 1e6 for r in seg_rows]   # 毫秒
            fig, ax = plt.subplots(figsize=(9, 4))
            for finger, cols in ALL_FLEX_COLUMNS.items():
                for c in cols:
                    vals = [r[c] for r in seg_rows]
                    style = '-' if finger == label else '--'
                    lw = 2.0 if finger == label else 0.8
                    alpha = 1.0 if finger == label else 0.5
                    ax.plot(ts, vals, style, linewidth=lw, alpha=alpha,
                           label=f"{name_cn[finger]}.{c.split('_')[-1]}" if finger == label else None)
            ax.set_xlabel("time (ms)"); ax.set_ylabel("joint angle (rad)")
            ax.set_title(f"测试段：{name_cn[label]}（实线粗=目标手指，虚线细=其它手指参考）")
            ax.legend(loc='upper right', fontsize=8)
            out_png = f"flexion_test_{label}_{per_finger_counter[label]}.png"
            fig.tight_layout()
            fig.savefig(out_png, dpi=120)
            plt.close(fig)
            print(f"已保存图：{out_png}")


if __name__ == "__main__":
    main()
