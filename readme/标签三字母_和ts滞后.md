# 三字母标签 + ts 滞后那个天文数字

---

## 一、`ts滞后1787590324629ms` —— 那不是滞后，是两个时基的原点差

```cpp
snap_.lastArrivalMs = QDateTime::currentMSecsSinceEpoch();   // Unix 纪元
snap_.tsLagMs = double(snap_.lastArrivalMs) - double(ts_ns) / 1.0e6;
```

`ts_ns` 顺着链路查上去是 `WebcamCamera.cpp:303`：

```cpp
qint64 ts_ns = f.startTime() >= 0 ? f.startTime() * 1000 : clock_.nsecsElapsed();
```

两种来源，**原点都跟监视器无关**：
- `f.startTime()` 是驱动自己的时基
- `clock_` 是**每台相机 start() 时各自归零**的 `QElapsedTimer`

拿纪元毫秒去减一个开机起算的数，得到的就是"1970 到现在" ≈ 56 年 = 你看到的那个数。

### 改法：不减，改成量同一时基内的增量

原点抵消掉才有意义。所以显示**相邻两包的 ts 增量**，也就是输出周期：

```
M3DS 26Hz/11555包　M3DQ 26Hz/11555包　帧间隔38.5ms
```

带跳变保护（相机重启会让 ts 突变，>1s 的增量不计入）和 EMA 平滑。
`tsStepValid` 为假时显示 `—`，不给假数字。

**没动 `lastArrivalMs`** —— 它还被"断流判定"用着，那里是纪元减纪元，本来就是对的。
所以是新增一路，不是改时基。

> 想真正量端到端滞后，得让采集侧和显示侧共用一个时钟，或者记录两者的偏移
> （`.pcrec` 里的 `ClockSync` 块就是干这个的）。那是另一件事，这次没做。

---

## 二、标签统一三字母

```
bk0 bk1 bk2 bk3 bk4        手背
tmc tpp tdp                拇指（掌骨/近节/远节）
ipp imp idp                食指
mpp mmp mdp                中指
rpp rmp rdp                无名指
lpp lmp ldp                小指
```

**小指用 `l` 不是 `p`**：pinky 的 p 会跟 pp(proximal) 撞成 `ppp`，
读起来分不清哪个 p 是手指、哪个是节段。`little finger` 是标准解剖学叫法。

验过：20 个全是三字母、无重复、越界返回空串。

---

## 三、预测点也标 + 未认领的只标灰

| 点的状态 | 圆点 | 标签 |
|---|---|---|
| 实测 + 被骨架认领 | 黄色实心 | **绿色** `idp` |
| 遮挡记忆(coasting) | 暗黄空心 | 绿色（如果被认领） |
| **预测点**（补出来的） | 蓝色空心方块 | **青绿色** `tdp` ← 新增 |
| **没被认领** | **灰色实心，小一圈** | **不标** ← 改了 |

两个实现要点：

**预测点的标签只能画在骨架层。** 它在点云里没有对应的点，
用它的 id 去点云层标会指到不存在或者已被别的点占用的 id 上 ——
那种错不会崩，只会静默标错位置。所以新增了一路 `setSkeletonLabels()`。

**未认领的点连标签都不画，但圆点保留（灰色）。** 完全不画的话就看不出
"这里还有个点没被用上"了 —— 而那恰恰是排查多余点时要找的东西。

---

## 核对

这几个文件要 Qt，我编不了。能验的：

```
20 个标签：全三字母、无重复、越界返回空串              ✅
括号配平（四个文件，剔字符串/注释）                    ✅
setSkeletonLabels  hpp 1 / cpp 4（定义+下发+两处清空）  ✅
skelLabels_        hpp 1 / cpp 3                        ✅
tsStepMs/tsStepValid/prevTsNs  声明↔引用对应            ✅
旧的 tsLagMs 残留                                       0 处 ✅
M3DQ 格式串：外层 6 个占位符对 6 个 .arg               ✅
                （第 7 个 .arg 是 "CSV %1行" 里的嵌套）
```

最后一条单独查过：**Qt 的 `.arg` 数量对不上不会编译报错**，
只会运行时静默输出错文本。

---

## 改动文件

```
src/estimate/HandSkeletonAssociator.hpp   labelShortName 改三字母
src/ui/PointCloudTestDialog.hpp           setSkeletonLabels + skelLabels_
src/ui/PointCloudTestDialog.cpp           分流填充、下发、绘制三档
src/ui/HandOutputMonitor.hpp              tsLagMs → tsStepMs/tsStepValid/prevTsNs
src/ui/HandOutputMonitor.cpp              算增量、改文案
```

`CalibWizard` 那两个（连拍提速 + 裁掉严格选拔）之前给过，这轮没动。
