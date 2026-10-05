#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""replay_udp.py —— 把 .pcrec 素材按真实管线重放成 M3DS/M3DQ UDP 包。

    用法:
      python tools/pcrec/replay_udp.py 文件.pcrec --selftest        # 只校验不发包
      python tools/pcrec/replay_udp.py 文件.pcrec --dump out.bin    # 写出原始 UDP 字节
      python tools/pcrec/replay_udp.py 文件.pcrec --send            # 发往 127.0.0.1:9010
      python tools/pcrec/replay_udp.py 文件.pcrec --send --port 9011 --speed 1.0

    数据来源: JointOut 块(最终 qOut / segQuat / 手性 flags) + Skeleton 块(wristR)。
    与 C++ UdpSender / HandPacket.hpp 的字节布局完全一致。
"""
import sys, os, struct, socket, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pcrec as P

M3DS_SIZE = 4 + 8 + 3*4 + 9*4 + 16*4       # 124
M3DQ_SIZE = 4 + 8 + 4 + 3*4 + 64*4 + 64*4 + 16  # 556

def quat_to_r9(w, x, y, z):
    # 行主序 3x3, 与 C++ matToQuat 逆运算一致
    return [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
            2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
            2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]

def build_m3ds(wristT, wristR9, joints16, ts):
    return struct.pack('<4s q 3f 9f 16f', b'M3DS', int(ts),
                       float(wristT[0]), float(wristT[1]), float(wristT[2]),
                       *[float(v) for v in wristR9],
                       *[float(v) for v in joints16])

def build_m3dq(wristT, quatW64, quatL64, segSrc16, flags, ts):
    return struct.pack('<4s q I 3f 64f 64f 16B', b'M3DQ', int(ts), int(flags),
                       float(wristT[0]), float(wristT[1]), float(wristT[2]),
                       *[float(v) for v in quatW64],
                       *[float(v) for v in quatL64],
                       *[int(v) & 0xff for v in segSrc16])

def align(r):
    jo = r.jointout
    sk = r.skeleton
    n = min(len(jo), len(sk))
    # 骨架块逐帧与关节角帧对齐；取第 0 帧 ts 校一次。
    jo_ts = jo['frameTsNs'].astype(np.int64)
    return n, jo, sk

def frames(path):
    r = P.load(path)
    n, jo, sk = align(r)
    out = []
    for i in range(n):
        j = jo[i]
        _, s = sk[i]
        wristT = j['wristT'].astype(float)
        # wristR9 直接用骨架块(与 emit 使用的 result.wristR 同源)
        wr = s['wristR'].astype(float).tolist()
        joints = j['qOut'].astype(float).tolist()
        flags = (int(j['wristValid']) & 1) | ((int(j['mcpValid']) & 1) << 1)
        for f in range(5):
            flags |= (int(j['fingerValid'][f]) & 1) << (8 + f)
        qw = j['segQuatWorld'].astype(float).tolist()
        ql = j['segQuatLocal'].astype(float).tolist()
        ss = j['segSource'].astype(np.uint8).tolist()
        ts = int(j['frameTsNs'])
        out.append((ts, wristT, wr, joints, flags, qw, ql, ss))
    return r, out

def selftest(path):
    r, fr = frames(path)
    print(f"帧 {len(fr)}")
    assert fr, "无帧"
    ts_prev = None
    for i,(ts,wt,wr,joints,flags,qw,ql,ss) in enumerate(fr):
        b1 = build_m3ds(wt, wr, joints, ts)
        b2 = build_m3dq(wt, qw, ql, ss, flags, ts)
        assert len(b1) == M3DS_SIZE, f"m3ds len {len(b1)}"
        assert len(b2) == M3DQ_SIZE, f"m3dq len {len(b2)}"
        assert b1[:4] == b'M3DS' and b2[:4] == b'M3DQ'
        # 解码回读校验
        rb_ts = struct.unpack_from('<q', b1, 4)[0]
        rb_joints = struct.unpack_from('<16f', b1, 60)
        assert rb_ts == int(ts)
        assert np.allclose(rb_joints, joints, atol=1e-6)
        if ts_prev is not None and ts < ts_prev:
            print(f"  !! 时间戳回退 @帧{i} {ts_prev} -> {ts}")
        ts_prev = ts
    # 统计
    dts = np.diff(np.array([f[0] for f in fr], dtype=np.int64))/1e9
    dts = dts[dts>0]
    print(f"  M3DS {M3DS_SIZE}B  M3DQ {M3DQ_SIZE}B  ts 单调 OK")
    print(f"  帧间隔 中位 {np.median(dts)*1000:.1f}ms  p95 {np.percentile(dts,95)*1000:.1f}ms")
    print(f"  首帧 ts {fr[0][0]}  末帧 ts {fr[-1][0]}")
    print("  SELFTEST PASS")
    return fr

def dump(path, out):
    fr = frames(path)[1]
    with open(out, 'wb') as f:
        for ts,wt,wr,joints,flags,qw,ql,ss in fr:
            f.write(build_m3ds(wt, wr, joints, ts))
            f.write(build_m3dq(wt, qw, ql, ss, flags, ts))
    print(f"写 {out}: {len(fr)*2} 个包")

def send(path, port, speed):
    fr = frames(path)[1]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"发送 {len(fr)} 帧 x2 到 127.0.0.1:{port}  speed={speed}")
    t0 = None
    for i,(ts,wt,wr,joints,flags,qw,ql,ss) in enumerate(fr):
        b1 = build_m3ds(wt, wr, joints, ts)
        b2 = build_m3dq(wt, qw, ql, ss, flags, ts)
        sock.sendto(b1, ('127.0.0.1', port))
        sock.sendto(b2, ('127.0.0.1', port))
        if speed > 0:
            if t0 is None: t0 = time.time(); first_ts = ts
            target = (ts - first_ts)/1e9/speed
            now = time.time() - t0
            if target > now: time.sleep(target - now)
        if i % 200 == 0:
            print(f"  {i+1}/{len(fr)}")
    print("发送完成")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print(__doc__); sys.exit(1)
    path = sys.argv[1]
    cmd = sys.argv[2]
    if cmd == '--selftest':
        selftest(path)
    elif cmd == '--dump':
        dump(path, sys.argv[3] if len(sys.argv) > 3 else 'replay.bin')
    elif cmd == '--send':
        port = 9010; speed = 1.0
        if '--port' in sys.argv: port = int(sys.argv[sys.argv.index('--port')+1])
        if '--speed' in sys.argv: speed = float(sys.argv[sys.argv.index('--speed')+1])
        send(path, port, speed)
