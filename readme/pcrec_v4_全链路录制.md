# pcrec v4：全链路录制与诊断

## 这一版解决什么问题

v3 能回答"点在哪、标签对不对"，但回答不了下面这两类问题——因为相关的数据
**一个字节都没录**：

1. **"系统输出的关节角/四元数合理吗？握拳时数据看上去像张开"**
   关节角从解算到 UDP 要过四级（解算 → 预测段平滑 → ROM 映射 → 速度限幅），
   v3 里这四级全是空白。而这四级**每一级都能单独造成同一个症状**，修法完全相反。

2. **"手性判别出错，不知道错在哪"**
   v3 只录了 4 个 bool。真正决定结果的量——**送进网络的那个手性
   `tmplIsRight`**、IK 用的手性、横向判据的原始值、几何锁定值、模型的
   `aiHandKnown` 位——一个都没录。于是出现过四个 bool 全"正常"而输出是镜像的情况：
   **没被记录的东西才会被漏看，这不是巧合。**

顺带修掉一个真问题：`writeNetRaw`(块11) 和 `writeAssocInput`(块10) 在 v3
里定义了但**全项目从未被调用**。头文件注释郑重写着的"离线复算最小闭包"，
实际上从没进过任何一个录制文件。

---

## 新增的块

| 块 | 名字 | 记什么 | 回答什么问题 |
|----|------|--------|--------------|
| 13 | JointOut | qSolve/qSmooth/qRom/qOut 四级角度、世界系+相对父节点两套四元数、ROM 区间、限幅削减量 | 输出角在哪一级坏的 |
| 14 | Handedness | 六处环节各自的手性 + 判据**原始值**（不是结论） | 手性从哪个接缝开始不一致 |
| 15 | MarkerDbg | 逐点来源（实测/IK/链式/网络/保持）+ 网络原始预测位置 vs 最终位置 | 是模型给歪了还是后处理拧的 |
| 16 | RunFlags | 逐帧生效的 22 个开关 | 这功能到底生效没有 |
| 17 | StateJson | 1Hz 全量运行时状态（worker 早就在生成，从没落盘） | 这段数据是在什么配置下录的 |
| 10/11 | AssocInput / NetRaw | **补上接线** | 离线复算能不能跟在线对上 |

### 为什么四级角度要分开记

只录最后一个数的话，下面四种成因永远分不开：

```
qSolve 就不对          → 分段四元数 / 标签 / 手性的问题
qSolve 对、qSmooth 平   → 平滑器的恢复补偿吃掉了信号
qSmooth 对、qRom 平     → ROM 某一维 hi-lo≈0，输出恒等于行程端点（最常见）
qRom 对、qOut 平        → 速度限幅削太狠
```

### 为什么四元数要记两套

**判"手指到底弯没弯"必须看相对父节点的那个。** 世界系四元数里，整只手的
朝向和手指自身的屈曲是混在一起的——手整体转 90° 和手指弯 90°，在世界系里
都表现为一个大角度变化。拿世界系去看屈曲，会把手腕的转动误读成手指在动。

### 为什么手性要记"判据值"而不只是"结论"

`autoIsRightDetected` 是个 bool，由判据跟 4mm 阈值比出来。
**判据 4.1mm 和 40mm 得到的是同一个 bool，可信度天差地别。**
只看 bool 会把"勉强过线的猜测"当成"确定的结论"——误判恰恰发生在这里。

---

## 怎么录

### 查"输出的关节角对不对" → 协议选「张开握拳循环」

> 五指张到最开停 3 秒 → 握到最紧停 3 秒，来回 5 轮，共约 30 秒。
> **每次切换时按一下「标记」。** 手掌尽量正对相机组，别整体移动。

**为什么必须是这个动作**：这段数据自带一个不依赖任何解算的物理真值——
**指尖球到腕心的距离**。它只由三角化的 3D 点算出来，完全不经过分段四元数、
关节角、ROM、限幅。握拳时它必然显著变小，这是几何事实不是算法结论。

有了它，"握拳输出像张开"就从主观描述变成可判定的命题：

```
corr(指尖距离, 输出屈曲角) 应当是强负相关
    正相关   → 某一级把符号弄反了
    接近 0   → 某一级把信号压平了
```

再逐级看四级各自的相关性和行程，就知道是哪一级。

**手要正对相机组**：真值只在四指指尖**全部实测**的帧上有效。指尖被遮挡时
它的位置是补出来的，拿补出来的点当真值等于用被怀疑的链路验证它自己。

### 查手性 → 协议选「手性排查」

> 手掌朝下、朝上、侧立各保持 5 秒，每种姿态下都做 2~3 次**明显的屈伸**，共约 30 秒。

**为什么必须有屈曲**：手摊平时手指点几乎就在手背平面上，有符号距离接近 0，
判据没有信息量。而自标定阶段的提示语是"请把手摆稳"，用户多半会把手摊平——
**这正是"标定时一切正常、一提交就左右反"的成因。**

---

## 怎么分析

```bash
# 体检：看块齐不齐、有没有丢块
python3 tools/pcrec/pcrec.py 你的文件.pcrec

# 全面诊断
python3 tools/hand_diag/pcrec_diagnose.py 你的文件.pcrec

# 只看某一节
python3 tools/hand_diag/pcrec_diagnose.py 你的文件.pcrec --section hand
python3 tools/hand_diag/pcrec_diagnose.py 你的文件.pcrec --section truth

# 逐帧明细（怀疑某一段时用）
python3 tools/hand_diag/pcrec_diagnose.py 你的文件.pcrec --dump-frames 100 140
```

诊断脚本分五节：手性 / 关节角输出链 / **真值对拍** / 逐点来源 / 生效开关。
第三节是结论所在，前两节是证据。

### 自测

诊断脚本的结论指向性很强（"符号反了""ROM 塌了"）。
**一个会给出错误结论的诊断脚本比没有诊断脚本更糟**——它会让人照着错误方向改代码。
所以每种故障都配了一段已知答案的合成数据：

```bash
for c in ok mirror rom_dead tmpl_flip; do
  python3 tools/hand_diag/gen_v4_selftest.py /tmp/$c.pcrec --case $c
  python3 tools/hand_diag/pcrec_diagnose.py /tmp/$c.pcrec
done
```

期望：`ok` 全绿；`mirror` 报"符号反了"并识别出手性镜像指纹；
`rom_dead` 定位到 ROM 这一级；`tmpl_flip` 报 `tmplIsRight` 与面板不一致。

---

## 格式对账机制

新块的载荷前 4 字节是 `{recVersion:u2, recBytes:u2}`。解析时拿 `recBytes`
跟 numpy 的 `dtype.itemsize` 对账，不等就**记录错误并跳过，绝不猜**。
C++ 那侧有 `static_assert` 钉死尺寸，Python 那侧导入时校验。

**这不是洁癖**：错位的 dtype 不会报错，只会读出一堆**看起来完全合理**的数，
然后把分析引向错误结论——比读不出来危险得多。v3 的 `Hm20DiagRec` 加过一次
字段，只要 C++ 和 `pcrec.py` 有一边忘了同步，就会发生这种事。

改字段时**必须两边一起改**：
- `src/record/PointCloudRecorder.hpp`（结构体 + static_assert）
- `tools/pcrec/pcrec.py`（dtype + `_EXPECT`）

---

## 改动的文件

| 文件 | 改了什么 |
|------|----------|
| `src/record/PointCloudRecorder.hpp` | v4 格式：5 个新块、自描述头、尺寸 static_assert、2 个新协议 |
| `src/estimate/HandSkeletonAssociator.hpp` | `SkeletonFrameResult` 加 `markerSource[20]`/`markerFlags[20]`，末尾集中归档逐点来源 |
| `src/estimate/SkeletonAssocWorker.hpp` | `SkeletonAssocDiag` 扩四级角度/两套四元数/ROM 状态/手性证据链/runFlags；新增 `ikIsRight_` 跟踪 |
| `src/ui/PointCloudTestDialog.cpp` | 写 5 个新块 + 补上块 10/11 的接线；协议下拉框加 2 项 |
| `tools/pcrec/pcrec.py` | v4 解析 + 尺寸对账 |
| `tools/hand_diag/pcrec_diagnose.py` | **新增**，五节诊断 |
| `tools/hand_diag/gen_v4_selftest.py` | **新增**，四种已知故障的合成数据 |

### 一处需要注意的实现细节

`qSolve` 必须在 `angSmooth_.apply()` **之前**抓——那一行是**就地覆写**
`ja_.q` 的，抓晚一步就永远拿不到未经平滑的原始角了。而"解算本身没解出来"
和"解出来了但被平滑吃掉"是两个完全不同的问题。
