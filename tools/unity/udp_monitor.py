#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""udp_monitor.py —— 实时监视 UDP 输出（M3DS 关节角 / M3DQ 分段四元数）

    python udp_monitor.py              # 默认 9010，全部显示
    python udp_monitor.py --port 9010 --mode quat
    python udp_monitor.py --raw        # 只打印包长和类型，排查网络用

【为什么要有这个】面板上的诊断栏只有汇总数字，Unity 的 Inspector 只有几个
标志位。绑骨骼之前你需要看到【每一个关节此刻的实际数值】，才能判断：
  · 是不是真的在动（数值有没有变）
  · 动的方向对不对（弯手指时该关节的角是不是在增大）
  · 哪些段是"测出来的"、哪些是"猜出来的"（segSource）
这三件事在 Unity 里看模型动画是看不清的 —— 模型不动可能是没数据、
可能是骨骼没绑对、也可能是数据全被判为不可信而跳过了。

【和 Unity 同时收】UDP 是单播到 127.0.0.1，同一端口【只有一个进程能收到】。
所以调试时先关掉 Unity 用这个看，确认数据没问题再开 Unity。
如果要同时收，把发送端改成组播或再开一个端口。
"""
import argparse
import socket
import struct
import sys
import time

SEG_NAMES = ["手腕"] + [f"{n}{p}" for n in ["拇", "食", "中", "无", "小"]
                        for p in ["近节", "中节", "远节"]]
JOINT_NAMES = ["拇CMC屈", "拇CMC展", "拇MCP", "拇IP"] + \
    [f"{n}{p}" for n in ["食", "中", "无", "小"] for p in ["MCP屈", "MCP展", "PIP"]]
SRC_NAMES = {0: "无依据", 1: "网络猜的", 2: "几何测的", 3: "IK解的"}
SRC_MARK = {0: "✗", 1: "?", 2: "✓", 3: "★"}


def quat_to_euler_deg(w, x, y, z):
    """转成绕 X/Y/Z 的欧拉角（度），只为了肉眼判断方向，不用于驱动。"""
    import math
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return tuple(a * 57.29577951308232 for a in (roll, pitch, yaw))


def parse_m3ds(d):
    need = 4 + 8 + (3 + 9 + 16) * 4
    if len(d) < need:
        return None
    ts, = struct.unpack_from("<q", d, 4)
    vals = struct.unpack_from("<28f", d, 12)
    return dict(ts=ts, wrist=vals[0:3], rot9=vals[3:12], joints=vals[12:28])


def parse_m3dq(d):
    need = 4 + 8 + 4 + 3 * 4 + 64 * 4 + 64 * 4 + 16
    if len(d) < need:
        return None
    ts, flags = struct.unpack_from("<qI", d, 4)
    wrist = struct.unpack_from("<3f", d, 16)
    qw = struct.unpack_from("<64f", d, 28)
    ql = struct.unpack_from("<64f", d, 28 + 256)
    src = struct.unpack_from("<16B", d, 28 + 512)
    return dict(ts=ts, flags=flags, wrist=wrist,
                qw=[qw[i * 4:i * 4 + 4] for i in range(16)],
                ql=[ql[i * 4:i * 4 + 4] for i in range(16)],
                src=list(src))


def fmt_flags(f):
    parts = []
    parts.append("腕部位姿" + ("✓" if f & 1 else "✗"))
    parts.append("掌指角" + ("✓" if f & 2 else "保持"))
    fingers = "".join("✓" if f & (1 << (8 + i)) else "·" for i in range(5))
    parts.append(f"五指[{fingers}]")
    return "  ".join(parts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=9010)
    ap.add_argument("--mode", choices=["both", "joint", "quat"], default="both")
    ap.add_argument("--hz", type=float, default=4.0, help="刷新频率，默认4Hz")
    ap.add_argument("--raw", action="store_true", help="只打包长和类型")
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", args.port))
    s.settimeout(2.0)
    print(f"监听 0.0.0.0:{args.port}   Ctrl+C 退出")
    print("提示：UDP 单播同一端口只有一个进程能收到 —— 调试时先关掉 Unity\n")

    last = 0.0
    n_ds = n_dq = n_2d = n_other = 0
    t0 = time.time()
    latest = {}
    while True:
        try:
            d, _ = s.recvfrom(65535)
        except socket.timeout:
            # 【超时也要渲染】否则发送端一停，屏幕就永远停在最早那一帧，
            # 看起来像"数据卡住了"，实际只是没再刷新过。
            if latest:
                last = 0.0
                d = None
            else:
                print(f"\r[{time.time()-t0:6.1f}s] 没收到包 —— 检查 UDP 是否已启用、"
                      f"端口是否一致、发送端有没有勾上 UDP 输出",
                      end="", flush=True)
                continue
        except KeyboardInterrupt:
            break

        magic = b""
        if d is not None and len(d) >= 4:
            magic = bytes(d[:4])
        if magic == b"M3DS":
            n_ds += 1; latest["ds"] = parse_m3ds(d)
        elif magic == b"M3DQ":
            n_dq += 1; latest["dq"] = parse_m3dq(d)
        elif magic == b"M2D0":
            n_2d += 1
        elif magic:
            n_other += 1

        if args.raw and magic:
            print(f"{magic.decode('ascii','replace')}  {len(d)} 字节   "
                  f"M3DS={n_ds} M3DQ={n_dq} M2D0={n_2d} 其它={n_other}")
            continue

        now = time.time()
        if now - last < 1.0 / max(args.hz, 0.5):
            continue
        last = now

        print("\033[2J\033[H", end="")     # 清屏
        el = now - t0
        print(f"[{el:6.1f}s]  M3DS(关节角) {n_ds}包 {n_ds/max(el,1e-9):5.1f}Hz   "
              f"M3DQ(四元数) {n_dq}包 {n_dq/max(el,1e-9):5.1f}Hz   "
              f"M2D0(2D光斑) {n_2d}包")

        dq = latest.get("dq")
        if dq:
            print(f"\n{fmt_flags(dq['flags'])}")
            print(f"腕部位置 (mm): {dq['wrist'][0]:8.1f} {dq['wrist'][1]:8.1f} {dq['wrist'][2]:8.1f}")

        if args.mode in ("both", "quat") and dq:
            print(f"\n{'分段':<8}{'来源':<10}{'local四元数 (w,x,y,z)':<40}{'等效欧拉角(度)':<24}")
            print("-" * 84)
            for i in range(16):
                q = dq["ql"][i]
                e = quat_to_euler_deg(*q)
                src = dq["src"][i]
                print(f"{SEG_NAMES[i]:<8}{SRC_MARK.get(src,'?')} {SRC_NAMES.get(src,'?'):<7}"
                      f"{q[0]:7.3f}{q[1]:8.3f}{q[2]:8.3f}{q[3]:8.3f}      "
                      f"{e[0]:7.1f}{e[1]:8.1f}{e[2]:8.1f}")
            nm = sum(1 for v in dq["src"] if v >= 2)
            print(f"\n可信分段 {nm}/16   ★=IK ✓=几何测的 ?=网络猜的 ✗=无依据")
            print("【猜出来的不要当测量值用】Unity 端 skipUnmeasuredSegments 就是跳过这些")

        ds = latest.get("ds")
        if args.mode in ("both", "joint") and ds:
            print(f"\n{'关节':<10}{'弧度':>9}{'度':>9}    {'':<20}")
            print("-" * 50)
            for i, nm2 in enumerate(JOINT_NAMES):
                rad = ds["joints"][i]
                deg = rad * 57.29577951308232
                bar = "#" * max(0, min(30, int(abs(deg) / 4)))
                print(f"{nm2:<10}{rad:>9.3f}{deg:>9.1f}    {bar}")
            print("\n【全是0或全都不动】多半是没做 ROM 标定 —— 角度撞限位被钳平了")

    print(f"\n\n收到 M3DS={n_ds} M3DQ={n_dq} M2D0={n_2d} 其它={n_other}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
        sys.exit(0)
