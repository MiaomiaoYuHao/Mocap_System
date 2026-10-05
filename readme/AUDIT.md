# 项目审查结果

## 一、发现的错误（已修）

### 1. `hand_rig.py` 显示我之前的姿态评估用错了模型

`tools/skeleton_assoc/hand_rig.py` 是训练时真正用的手模型（带解剖学采样、皮肤滑移、
左右手镜像），`forward_kinematics()` 直接返回 `seg_R (B,16,3,3)` 真值段姿态。
我之前用的是自建的近似 rig，球心 standoff 拍成 11~17mm，而真 rig 是 **5.3~6.7mm**
（`MARKER_BALL_RADIUS_MM + MARKER_SEAT_MM + GLOVE_THICK_MM`）。

用真 rig 重算后推翻了一个关键判断：**球心贴在每节指骨的远端附近**（`_fit_setbacks`
的结果），所以 `pp球 -> mp球` 这个向量跨的其实是【中节】骨，不是近节。
近节骨没有任何一对 marker 能张成它。

各方向源对各段的中位误差（64 受试者 × 200 姿态，0.5mm 噪声）：

```
segment      anchor->pp   pp->mp   pp->dp   mp->dp
index_pp         25.5°    26.7°    43.9°    65.4°
index_mp         70.0°    17.3°     4.0°    20.8°
index_dp        101.7°    49.6°    32.3°    12.6°
```

我上一版补丁选的是 `pp->mp / pp->dp / mp->dp`，中位 10.2° 但 p90 42.5°、max 85.5°。
改成 `anchor->pp / pp->dp / mp->dp`（拇指掌骨段例外，仍用 `pp->mp`，因为 CMC 的
轴向旋前会把 anchor 到球心的向量带偏）：

```
                中位     p90     p99     max
现状           18.1°   47.6°   71.4°   86.1°
最终补丁       12.8°   30.8°   40.0°   61.7°
```

中位略逊于上一版，但尾部好得多。做数据采集喂 IK，尾部比中位重要，所以选这个。
`anchor` 来自标定模板的 `fingerParams[].anchor`，经腕部位姿变换到世界系，
因此依赖 `wristPoseValid`；不满足时自动退回 `pp->mp`。

### 2. Kabsch 用了归一化模板（原代码的 bug）

`tmplMarker()` 直接读 `tmpl_` 前 60 维，但那是送给 ONNX 的**归一化**模板
（除过 `handScale`，约 180mm）。拿它和毫米观测做 Kabsch，两边差约 180 倍尺度，
而 Kabsch 不解尺度，于是旋转被硬拧到某个使残差最小的方向。症状隐蔽：
`dorsumRmseMm` 会很大但没人检查，表现出来只是"腕部四元数乱跳"。
已拆成 `tmplMm_` 单独存，加 `setTemplateMm()`；没设置就不解腕部位姿。

### 3. `boneFrame` 的退化分支（我上一版的假修）

把全局参考换成手背法线只是把突变点从 `|x_z|=0.9` 挪到 `|sin|=0.15`，硬分支还在。
单测 scenario5 让骨轴缓慢扫过退化区，照出仍跳 149.72°。改成退化区用上一帧
该段的 y 轴做 Gram-Schmidt，出退化区立刻回到手背法线绝对参考。现在 0.45°
（= 每步骨轴自身转角，完全连续）。

## 二、删掉的多余文件

| 文件/目录 | 原因 |
|---|---|
| `src/estimate/OnnxSkeletonInferenceBackend.hpp` | 上一代后端，依赖的 `ISkeletonInferenceBackend`/`SkeletonTemplate` 已不存在，编译不过，且无人引用 |
| `tests/test_hand_skeleton_associator_rotation.cpp` | 同上，用旧 API；且不在任何 CMake 目标里，属于孤儿 |
| `tools/*/__pycache__/` | Python 字节码缓存 |
| `.qtcreator/CMakeLists.txt.user` | IDE 本地状态，机器相关，不该入库 |
| 根目录 `CMAKE_CHANGES.md` `FILE_PLACEMENT.md` `POINTCLOUD_DIALOG_CHANGES.md` `README.md` | 我上一轮的交付说明，是操作指引不是项目文档 |
| CMakeLists 里 `skeleton_assoc.onnx` 的拷贝规则 | 配套后端已删，规则失效 |

删除后修掉了 4 处指向已删文件的注释。

## 三、重命名

- `tools/#U538b#U529b...md` → `tools/压力测试与优化方案.md`（文件名 Unicode 转义没解开）
- `tools/hm20_quat_eval/hand_rig.py` → `synthetic_rig_deprecated.py`
  与 `tools/skeleton_assoc/hand_rig.py` 同名但内容完全不同，极易误 import。
  新增 `tools/hm20_quat_eval/eval_on_real_rig.py`，直接吃真 rig 的 `seg_R`。

## 四、对账结果（都干净）

- CMake 源文件列表 ↔ `src/**/*.cpp`：无缺失、无遗漏
- CMake 测试目标 ↔ `tests/*.cpp`：删掉 rotation 后只剩 `test_hand_coldstart.cpp`
  一个孤儿（存在但没进 CMake，未动，可能是有意保留的历史文件）
- `cmake configure` 解析通过（Qt6 打桩验证）
- `tests/test_hand_skeleton_associator.cpp` 就地编译运行：43 passed, 0 failed

## 五、仍未做

1. `IHm20IkRefiner` 没有任何实现。真 rig 上剩余的 12.8° 中位偏差只有 IK 能消。
2. 左右手写死 `isRightHand = true`（`PointCloudTestDialog.cpp`）。
   `HandTemplateData` 没有这个字段，标定向导补上后要接。
3. 手指/手背分离阈值 0.55/0.30。`Hm20Config` 只有一个 `minAssignProb`，
   要拆得改 `process()` 里按 `j<5` 分流。
4. `listDirectPeDependencies()`（126 错误码诊断）随旧文件一起删了，
   需要时从 git 历史捞。
5. Qt 相关文件（Dialog / Worker / Backend）在本环境编译不了，未验证。
