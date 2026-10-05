# tools/hm20_diag

分两类：**分析录制的**（看真机跑成什么样）和**验证代码的**（看改动有没有生效）。

---

## 一、分析录制

| 文件 | 用途 | 本次是否在包里 |
|---|---|---|
| `pcrec.py` | `.pcrec` 读取库。⚠️ 必须跟 `src/record/PointCloudRecorder.hpp` 同版本，否则解析错位 | ✅ |
| `pcrec_report.py` | 一条命令出诊断报告，七节 | 早前给过，本包未重发 |
| `pcrec_replay.cpp` | 离线复算 / 参数扫描，不需要 onnxruntime | 早前给过 |
| `gen_test_pcrec.cpp` | 造测试用 `.pcrec`，改格式后跑往返验证 | 早前给过 |
| `probe_thumb_axis.py` | 量模型的拇指屈曲平面偏多少（只要 `.onnx`） | 早前给过 |
| `probe_dorsum_reversal.py` | 量手背模板能不能被稳定定标签 | 早前给过 |

标"早前给过"的在之前的交付里，本次没重发（内容没变）。缺的话说一声我补。

```bash
python pcrec_report.py recordings/xxx/capture.pcrec     # 每次录完先跑这个
```

---

## 二、验证代码改动有没有生效

**为什么需要这一类**：这个项目里出现过三次「说已修、其实没进文件」——
编辑脚本中途失败、只拿了 `.cpp` 没拿 `.hpp`、字段声明了没人引用。
共同特征是**看着是绿的，实际没覆盖到**。所以每次改完时间常数或恢复逻辑，
跑这几个，别只读源码。

| 文件 | 验什么 | 期望 |
|---|---|---|
| `verify_effective_config.cpp` | 实际生效的配置值 | 见文件头 |
| `verify_time_constants.cpp` | 四个恢复机制第几帧动作 | 0.68 / 0.20 / 0.96 / 0.84 s |
| `verify_reset_clears_prior.cpp` | 「手背复位」有没有清网络时序先验 | 20 → 0 |
| `verify_bundle_async.cpp` | 束调整在不在主线程 | **目前触发不了，是失败状态** |

```bash
g++ -std=c++20 -O2 -I src tools/hm20_diag/verify_time_constants.cpp -o vb && ./vb
```

### 三条关于验证方式的教训

1. **看退出码，不要 grep**。`grep -E "^\S*error"` 匹配不上 gcc 的
   `文件:行:列: error:`（空格挡住了），害我连报三轮假的"零 error"。
2. **配置对 ≠ 逻辑跑**。`bundleRefitGrowth` 曾经声明了但没有任何地方引用，
   配置打印正常，行为完全没变。所以要有 `verify_time_constants` 这种行为测试。
3. **新代码路径必须有针对性测试**。空间门控的下标 bug 没被原有蒙特卡洛抓到，
   因为那些用例的杂点散布刚好让门控不过滤，`P` 和 `pts` 相同，
   新路径根本没被走到。写完新测试要用「回退修复」对照确认它能变红。

---

## 三、专项测试

| 文件 | 验什么 |
|---|---|
| `test_dorsum_gating.cpp` | 空间门控的下标映射（回退修复会变红） |
| `test_dorsum_repair.cpp` | 贴点重捕：真实模板 + 15mm 偏移 |
| `test_dorsum_lockin.cpp` | 死锁复现（**目前是"复现得出、修不好"**） |
| `test_template_refine.cpp` | 持续微调能不能跟上容差内的漂移 |
| `test_template_runaway.cpp` | 4 分钟不漂，模板会不会自己走飞 |
| `test_thumb_tracker.cpp` | 拇指遮挡跟随（喂 `.pcrec` 直接跑） |
| `test_thumb_sign_coupling.py` | ΔIP 与 ΔMCP 的符号一致率 + 门限扫描 |
| `test_autocalib_replay.cpp` | 自标定复算（**有局限：重建 result 会丢状态**） |

---

## 目前已知没解决的

- **异步束调整没验证过** —— 见 `verify_bundle_async.cpp` 文件头。
  真机看 `.pcrec` 里的 `bundleRuns` / `bundleLastMs`。
- **`test_dorsum_lockin.cpp` 复现的死锁修不好** —— 造出来的场景跟真机
  "连线全乱"不是一回事（那一版手指落点拟合是好的）。
- **两个 ui 文件从来没编译过** —— 本机没 Qt。
