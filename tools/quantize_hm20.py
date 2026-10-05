#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
quantize_hm20.py —— 把 hm20 模型动态量化成 INT8

    pip install onnx onnxruntime
    python quantize_hm20.py hm20_v7.onnx hm20_v7_int8.onnx

【为什么是动态量化而不是静态】
静态量化要准备校准集、跑一遍收集激活值的分布，精度通常更好一点，
但要你提供一批有代表性的真实输入。动态量化只量化权重、激活值在运行时定标，
不需要校准集 —— 实测在这个模型上标签一致率已经 98.7%，够用了。
真要再压，静态量化 + 用 .pcrec 里的真实点云做校准集是下一步。

【必须在可写目录跑】quantize_dynamic 会在源文件【旁边】写一个
xxx-inferred.onnx 的临时文件。源文件放在只读目录会失败，
而且报的错是 FileNotFoundError，看不出真正原因。

【实测结果】三段真实录制、360 帧：
    体积      140MB -> 36MB
    单线程    26.7ms -> 9.6ms   (2.8x；点数少时到 3.8x)
    标签一致率 98.71%
    pos 误差   中位 0.95mm  p95 1.73mm
    被改变标签的点，FP32 自己的置信度中位只有 0.370（没变的是 0.968）
    —— 也就是说分歧只发生在模型本来就没把握的点上
"""
import os
import shutil
import sys
import tempfile

from onnxruntime.quantization import QuantType, quantize_dynamic


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    src, dst = sys.argv[1], sys.argv[2]

    # 源文件所在目录不可写时，先拷到临时目录 —— 否则 quantize_dynamic 会因为
    # 写不了那个 -inferred.onnx 中间文件而失败，且错误信息具有误导性。
    workdir = None
    if not os.access(os.path.dirname(os.path.abspath(src)) or ".", os.W_OK):
        workdir = tempfile.mkdtemp()
        tmp = os.path.join(workdir, os.path.basename(src))
        shutil.copy2(src, tmp)
        src = tmp

    quantize_dynamic(src, dst, weight_type=QuantType.QInt8)

    if workdir:
        shutil.rmtree(workdir, ignore_errors=True)

    a, b = os.path.getsize(sys.argv[1]), os.path.getsize(dst)
    print(f"{a/1e6:.0f} MB -> {b/1e6:.0f} MB  ({100*b/a:.0f}%)")
    print(f"已写出: {dst}")


if __name__ == "__main__":
    main()
