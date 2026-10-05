#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取本上位机采集的 .mcap 原始灰度帧文件，供动捕算法解析。
文件格式（小端）见 C++ 端 FrameRecorder.hpp。此脚本零依赖（可选 numpy）。

用法:
    python read_mcap.py cam0_20260101_120000.mcap
会打印头信息并逐帧读取，演示如何拿到 (timestamp_ns, HxW 灰度数组)。
把 process(ts, frame) 换成你的质心检测即可。
"""
import struct
import sys


def read_mcap(path):
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"MCGF":
            raise ValueError("不是 MCGF 文件: %r" % magic)
        version, cam_id, w, h, _reserved = struct.unpack("<IIIII", f.read(20))
        yield {"version": version, "cam_id": cam_id, "width": w, "height": h}

        frame_bytes = w * h
        while True:
            hdr = f.read(12)  # int64 ts + uint32 len
            if len(hdr) < 12:
                break
            ts_ns, data_len = struct.unpack("<qI", hdr)
            data = f.read(data_len)
            if len(data) < data_len:
                break
            yield {"ts_ns": ts_ns, "width": w, "height": h, "pixels": data}


def main():
    if len(sys.argv) < 2:
        print("用法: python read_mcap.py <file.mcap>")
        return
    it = read_mcap(sys.argv[1])
    header = next(it)
    print("头:", header)

    try:
        import numpy as np
        have_np = True
    except ImportError:
        have_np = False

    n = 0
    first_ts = None
    last_ts = None
    for rec in it:
        ts = rec["ts_ns"]
        if first_ts is None:
            first_ts = ts
        last_ts = ts
        # ---- 在这里接你的动捕算法 ----
        if have_np:
            frame = np.frombuffer(rec["pixels"], dtype=np.uint8).reshape(
                rec["height"], rec["width"])
            # 示例：最亮像素位置（真正的质心检测请替换这里）
            # idx = int(frame.argmax()); y, x = divmod(idx, rec["width"])
            _ = frame
        n += 1

    dur = (last_ts - first_ts) / 1e9 if (first_ts is not None and n > 1) else 0.0
    fps = (n - 1) / dur if dur > 0 else 0.0
    print("共 %d 帧, 时长 %.2fs, 平均 %.1f fps" % (n, dur, fps))


if __name__ == "__main__":
    main()
