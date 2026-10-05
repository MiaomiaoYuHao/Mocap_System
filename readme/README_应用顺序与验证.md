# 拇指外翻 + 手背刚体标签：补丁包与验证协议

本包对应上一条回复里的第 4/5/6 条。**按顺序做，每一步都有一个可读的数字告诉你它有没有生效** ——
你之前卡住的一大半原因是改完之后没有判据，只能靠"看着像不像"。

| 文件 | 放到 | 干什么 |
|---|---|---|
| `DorsumRigidSolver.hpp` | `src/estimate/` | 手背 5 点刚体求解（新文件，取代 `relabelDorsumByGeometry` + `trackDorsum`） |
| `probe_dorsum_reversal.py` | 随便哪 | 量你自己的手背模板能不能被稳定定标签。**先跑这个** |
| `probe_thumb_axis.py` | 随便哪 | 量模型的拇指屈曲平面偏了多少。零依赖，只要 .onnx |
| 本文件 | — | 调用点补丁 + 验证协议 |

---

## 第 0 步：先量两个数（10 分钟，不改代码）

```bash
pip install onnxruntime numpy
python probe_thumb_axis.py hm20_v7.onnx
python probe_dorsum_reversal.py "C:/Users/你/Documents/.../hand_template.json"
```

拿到两个数字：

- **拇指铰链轴与四指的夹角**。我在你上传的 `hm20_v7.onnx` 上量到 **17.4°**（真人 80~90°）。
- **手背模板的自歧义**（最优错解的 Kabsch 残差）。参考布局是 **1.77mm**，`< 3mm` 就是病态。

这两个数决定后面每一步值不值得做。**如果自歧义 < 3mm，第 4 步（重贴点）的收益比所有代码改动加起来都大。**

---

## 第 1 步：止血（3 处改动，10 分钟）

### 1.1 停掉伪造观测

`HandSkeletonAssociator.hpp` 的 `trackDorsum()` 末尾：

```cpp
        for (int k = 0; k < kD; ++k) {
            if (!fixedOk[size_t(k)]) continue;
            ...
            out.markers[size_t(k)].posWorld = fixed[size_t(k)];
            out.markers[size_t(k)].observed = true;      // ← 删掉这一行
        }
```

`fixed[k]` 来自 `trkPos_[t]`，而 `trkPos_` 只在贪心最近邻匹配成功时更新，`trkHas_` 一旦置真永不清零。
手腕平移超过 `dorsumTrackGateMm = 25mm/帧`（45fps 下 1.1m/s）时全部匹配失败 →
5 个点被永久钉在旧位置，还标成"已观测"，**而且再也回不来**。这就是"连线里有几个点固定了、
明明那里没有点"。

更麻烦的是 `prevPos_/prevMask_` 是在 `trackDorsum` **之后**采样的（`process()` 第 793 行附近），
伪造的观测会回灌给网络当上一帧先验，形成正反馈。删掉这一行同时切断反馈。

### 1.2 关掉 tracklet

`Hm20Config::dorsumTracklet = false`（或面板上关）。第 2 步的刚体求解会完整取代它。

### 1.3 关掉拇指位置回正

`Hm20Config::thumbPronationRad = 0`（面板上把"回正预测点"取消勾选）。

原因：`correctPredictedThumb()` **只转 `observed==false` 的点**。你截图里的状态是
marker5(掌骨)、marker6(近节)**实测**、marker7(远节)预测 —— 它只把 marker7 绕掌骨轴转 80°，
5/6 原地不动。于是 `拇 近节` 骨轴 = p7−p5、`拇 远节` 骨轴 = p7−p6，两个方向都不再有物理意义。
你面板上 `拇 近节` 的相对父节点四元数 `(0.511, 0.507, -0.560, -0.410)` **对应 118°**，
而拇指 MCP 的解剖限位是 0.95 rad = **54°** —— 这个数字本身就说明拇指链是断的，不是偏的。

> 关掉之后拇指仍然会外翻（那是模型先验，第 3 步才修），但至少不会再多出一个假的 118° 关节角。

**验证**：`dorsumTrackFixed` 应该恒为 0；`thumbFixed` 恒为 0；
连线里"钉住的点"当场消失。如果没消失，说明那个点是别的路径来的，告诉我。

---

## 第 2 步：手背改成刚体求解（主体改动）

### 2.1 加文件

`DorsumRigidSolver.hpp` 丢进 `src/estimate/`，CMakeLists 的头文件列表里加一行。

它做的事：每帧在**全部候选点**里找"三点边长同时对得上模板某个三元组"的种子 → Kabsch →
把整份模板投过去收内点 → 用内点重解。**"谁是手背点"和"是哪一个"一起解，不依赖模型的手背分类。**
支持 3/4/5 可见。歧义时用**位姿连续性**破（反序错解对应腕部转 ~180°，一票否决），
冷启动没有历史时用**手指落点**破，都用不上就**拒解**。

### 2.2 `Hm20SkeletonAssociator` 里加成员和函数

在类的 private 段加：

```cpp
    DorsumRigidSolver dorsum_;
```

在 `Hm20Config` 里加：

```cpp
    bool   dorsumRigidSolve = true;   // 取代 dorsumGeoRelabel + dorsumTracklet
```

在 `SkeletonFrameResult` 里加两个诊断字段（面板要显示）：

```cpp
    int  dorsumSolveReason = 1;   // 0=ok 1=候选不足 2=无可行解 3=歧义拒解 4=残差超限
    bool dorsumUsedHistory = false;
```

然后加这个成员函数（放在 `relabelDorsumByGeometry` 旁边）：

```cpp
    void solveDorsumRigid(SkeletonFrameResult& out,
                          std::array<bool, kNumMarkers>& seen,
                          const std::vector<std::pair<int, Vec3>>& candidates, int N) {
        out.dorsumGeoRmseMm   = -1.0;
        out.dorsumGeoMarginMm = -1.0;
        out.dorsumGeoFixed    = 0;
        out.dorsumSolveReason = 1;
        if (!cfg_.dorsumRigidSolve || !tmplMmValid_) return;

        if (!dorsum_.templateValid()) {
            std::array<Vec3, 5> T{};
            for (int m = 0; m < 5; ++m) T[size_t(m)] = tmplMm_[size_t(m)];
            dorsum_.setTemplate(T);
            std::array<Vec3, 15> F{};
            for (int m = 0; m < 15; ++m) F[size_t(m)] = tmplMm_[size_t(m + 5)];
            dorsum_.setFingerTemplate(F);      // 冷启动歧义靠它破
        }

        std::vector<Vec3> pts;
        pts.reserve(size_t(N));
        for (int i = 0; i < N; ++i) pts.push_back(candidates[size_t(i)].second);

        // 手指点只用于仲裁，不参与刚体拟合
        std::vector<DorsumRigidSolver::FingerObs> fobs;
        for (int m = 5; m < kNumMarkers; ++m)
            if (seen[size_t(m)])
                fobs.push_back({m, out.markers[size_t(m)].posWorld});

        const DorsumSolveResult r = dorsum_.solve(pts, fobs);
        out.dorsumGeoRmseMm    = r.rmseMm;
        out.dorsumGeoMarginMm  = r.marginMm;
        out.dorsumSolveReason  = r.reason;
        out.dorsumUsedHistory  = r.usedHistory;

        if (!r.ok) {
            // 【解不出来就承认解不出来】不要退回模型的手背标签——那正是要替掉的东西。
            // 5 个手背标签全部标成未观测，第 4 步会自然走"沿用上一帧腕部位姿"，
            // wristPoseValid=false，下游据此知道这一帧的腕部是外推的。
            for (int m = 0; m < 5; ++m) {
                seen[size_t(m)] = false;
                out.markers[size_t(m)].observed = false;
                out.markers[size_t(m)].confidence = 0.0;
                out.markers[size_t(m)].sourcePointId = -1;
            }
            return;
        }

        for (int k = 0; k < 5; ++k) {
            const int j = r.pointOf[size_t(k)];
            if (j >= 0) {
                if (out.markers[size_t(k)].sourcePointId != candidates[size_t(j)].first)
                    ++out.dorsumGeoFixed;
                out.markers[size_t(k)] = {k, candidates[size_t(j)].second, true, 1.0,
                                          candidates[size_t(j)].first};
                seen[size_t(k)] = true;
            } else {
                // 没匹配上的手背点：用刚体位姿反投影补。比网络 pos 头准
                //（这一帧的刚体位姿刚被别的点解出来），但【必须标未观测】。
                out.markers[size_t(k)] = {k, dorsum_.project(r.R, r.t, k), false, 0.0, -1};
                seen[size_t(k)] = false;
            }
        }
        dorsumR_ = r.R; dorsumT_ = r.t; dorsumPoseOk_ = true;
        dorsum_.acceptHistory(r.R);      // 存成下一帧的连续性参考
    }
```

再加三个成员：

```cpp
    Mat3 dorsumR_{1,0,0, 0,1,0, 0,0,1};
    Vec3 dorsumT_{};
    bool dorsumPoseOk_ = false;
```

### 2.3 改调用点

`process()` 里，把这两行

```cpp
        relabelDorsumByGeometry(out, seen);
        ...
        trackDorsum(out, seen);
```

替换成一行：

```cpp
        solveDorsumRigid(out, seen, candidates, N);
```

**位置不要动**：必须仍然在 `prevPos_/prevMask_` 采样之前、Kabsch 之前。

### 2.4 第 4 步的 Kabsch 直接用求解器的位姿

刚体求解已经解出了 `R,t`，不要再拿"重排后的点"做一次 Kabsch（会得到同一个答案但多算一遍，
而且缺点时它的可见点选择跟求解器不一致）。把第 4 步改成：

```cpp
        wristPoseValid_ = false;
        dorsumPoseOk_ = false;   // 由 solveDorsumRigid 置位
        if (dorsumPoseOk_) {
            out.wristR = dorsumR_;
            out.wristT = dorsumT_;
            out.dorsumRmseMm = out.dorsumGeoRmseMm;
            wristPoseValid_ = true;
            lastWristR_ = out.wristR; lastWristT_ = out.wristT; hasLastWrist_ = true;
        } else if (hasLastWrist_) {
            out.wristR = lastWristR_; out.wristT = lastWristT_;
            out.message = "手背刚体未解出，腕部位姿沿用上一帧";
        } else {
            out.message = "手背刚体未解出且无历史";
        }
```

（`dorsumPoseOk_` 的清零要放在 `solveDorsumRigid` 调用**之前**，我上面写反了顺序，
实现时在 `solveDorsumRigid` 开头 `dorsumPoseOk_ = false;` 即可。）

### 2.5 面板加一行诊断

```
手背刚体: 内点4/5  残差0.83mm  裕度9.1mm  [连续性仲裁]     自歧义 1.77mm
```

`自歧义` 直接读 `dorsum_.selfAmbiguityMm()`，标定一次算一次，不用每帧算。

### 2.6 这套东西的实测数（跑的是这份 C++，不是仿真重写版）

400 次/组，随机世界位姿 + 高斯噪声 + 随机遮挡 + 杂点：

**布局 A = 你现在这种（自歧义 1.77mm）**

| 噪声 | 遮挡 | 杂点 | 有历史 | 正确 | **错标** | 拒解 |
|---|---|---|---|---|---|---|
| 0.9mm | 0 | 0 | 无 | 0.182 | **0.000** | 0.818 |
| 0.9mm | 0 | 0 | 有 | **1.000** | **0.000** | 0.000 |
| 0.9mm | 1 | 10 | 有 | 0.998 | 0.003 | 0.000 |
| 0.9mm | 2 | 10 | 有 | 0.955 | 0.015 | 0.030 |
| 2.0mm | 0 | 10 | 有 | 0.905 | 0.075 | 0.020 |

**布局 B = 只把手背 5 点的极角改成明显不均匀（自歧义 8.21mm）**

| 噪声 | 遮挡 | 杂点 | 有历史 | 正确 | 错标 | 拒解 |
|---|---|---|---|---|---|---|
| 0.9mm | 0 | 0 | 无 | **1.000** | 0.000 | 0.000 |
| 0.9mm | 1 | 0 | 无 | 0.873 | 0.000 | 0.128 |
| 0.9mm | 2 | 10 | 有 | 0.970 | 0.020 | 0.010 |

**怎么读这两张表：**

1. **锁上之后（有历史），当前布局就能做到 100% 正确、0% 错标**。你的问题不是稳态，是**建锁**。
2. **冷启动（无历史）当前布局只有 18% 能解，82% 拒解，但错标 0%**。
   现在的代码在这种情况下是**猜**（残差差 1.77mm 就当判据），猜错就是腕部翻 180°。
   改成拒解之后，"偶尔几帧没有腕部位姿"取代了"偶尔整只手翻过去"。
3. **布局 B 把冷启动从 18% 拉到 100%**。这就是为什么重贴点比改代码值钱。
4. 噪声 2.0mm + 遮挡 2 个那几档明显变差 —— 3 个点定刚体本来就勉强。
   如果你的手背经常只剩 3 个点可见，那是相机布置的问题（`改动总览.md` 里
   建议加第 4 台相机那条）。

---

## 第 3 步：拇指常数旋前（根因）

### 3.1 `Hm20IkRefiner.hpp` 加一个参数、改一行

```cpp
struct Hm20MarkerModel {
    ...
    double thumbAxialK = 0.576;
    // 【新增】第一掌骨的【常数】解剖旋前(rad)。
    // 训练用的 hand_rig.py 里旋前完全耦合在 CMC 外展上(axial = k * a[1])，
    // 没有常数基线：握拳时 CMC展=0.18、k∈[0.35,0.80] -> 旋前只有 4~8°。
    // 而真人第一掌骨中立位就有 80~90° 旋前——这正是拇指能对掌的原因。
    // 实测(probe_thumb_axis.py 打在 hm20_v7 上)：模型的拇指屈曲铰链轴跟四指
    // 只差 17.4°，真人应该差 80~90°，缺口 ~73°。
    // 【符号】refine() 内部已经把左手镜像成右手系再解，所以这个常数
    //         在 markerFK 里【跟手性无关】，不要在这里翻号。
    double thumbPronation0 = 0.0;    // 默认 0 = 完全沿用旧行为
};
```

`markerFK()` 里：

```cpp
    M3 R = mul(rotZ(q[1]), rotY(q[0]));
    if (finger == 0)
        R = mul(R, rotX(mm.thumbPronation0 + mm.thumbAxialK * q[1]));   // ← 加了 thumbPronation0
```

加 setter：

```cpp
    void setThumbPronation0(double rad) { mm_.thumbPronation0 = std::clamp(rad, -2.0, 2.0); }
    double thumbPronation0() const { return mm_.thumbPronation0; }
```

**这一行是全包里最高杠杆的改动**，因为它同时修三件事：

1. 拇指 IK 第一次有可能拟合上一个真正对掌的拇指 → 残差降到 `ikMaxRmseMm` 以下 →
   `fingerIkValid[0]` 从 false 变 true → 你截图里的 **`拇✗` 变 `拇✓`**。
2. `ikFillOccluded` 摆出来的拇指点从此在正确的平面里 —— 不需要事后再绕轴转一次。
3. 自标定的球面拟合不再把这 73° 的系统误差当噪声吸进球心 →
   拇指 anchor 误差应该从 9~14mm 掉到接近四指的 3mm 量级（`SKELETON_UPGRADE.md` 附录 A 里
   那个"拇指差 4 倍"的谜底）。

### 3.2 符号和幅值怎么定：不要靠眼睛

从 rig 的构造推：`R = rotZ(q1)·rotY(q0)·rotX(a)`，之后 MCP/IP 的 `rotY` 都发生在
`rotX(a)` **之后**的局部系里，所以铰链轴在父系里是 `rotZ(q1)·rotY(q0)·(0, cos a, sin a)`。
`q0≈0` 时化简成 `rotZ(q1)·(0, cos a, sin a)`：

| a | 铰链轴 | 屈曲时远节往哪走 | 生理 |
|---|---|---|---|
| 0（现状） | (−sin q1, cos q1, 0) | 直接压向掌面 (−Z)，不横过手掌 | 像第五根手指 |
| **−π/2** | (0, 0, −1) | 向尺侧横过手掌 (−Y) | **对掌** ✓ |
| +π/2 | (0, 0, +1) | 向桡侧甩出去 (+Y) | 外翻 ✗ |

所以**幅值 ≈ 1.4~1.57 rad，符号按上表应该是负的**。但这条推导依赖我对
`rotX/rotY/rotZ` 与 `Vec3` 行主序约定的读法，**我没有真机验过**。所以：

**做一次 A/B，30 秒定死它。** 手摆成"拇指对掌抵住小指根"这种拇指三点都能被看见的姿势，
`thumbPronation0` 分别设 `-1.5 / 0 / +1.5`，看 `fingerIkRmseMm[0]`：

- 正确的那个符号，残差应该从 10mm+ 掉到 3~5mm，`拇✓` 亮起来；
- 错误的符号会**更差**（把 73° 缺口变成 163°）。

**残差是可读的数字，不是"看着像不像"。** 这就是这一步跟以前调 `thumbRollOffsetRad`
最大的区别 —— 那个参数作用在 roll 上，而 roll 在每节只有 1 颗球时**不可观测**，
所以永远没有数字能告诉你调对了没有。`thumbPronation0` 作用在**可观测**的 marker 位置上。

### 3.3 让束调整自己解它

`Hm20AutoCalib` 里已经有对 `thumbAxialK` 的一维网格搜（0.30~0.90 步长 0.025）。
把 `thumbPronation0` 加进去做二维搜（或先粗搜 `thumbPronation0` ∈ [−1.8, −1.0] ∪ [1.0, 1.8]，
再固定它搜 `thumbAxialK`）。可辨识性判据沿用现有那条：拇指外展角覆盖度不够时不搜。

**代价**：一维网格 33 点 → 二维 33×17 ≈ 560 点。束调整是离线的（收集阶段），不在每帧路径上，
可以接受。真正省事的做法是：**`thumbPronation0` 只搜一次并锁死**（它是解剖常数，
一个人一辈子不变），`thumbAxialK` 每次会话重搜。

### 3.4 然后删掉 `correctPredictedThumb()`

它是给"没有 IK"那条路打的补丁，而且有三个结构性问题：

- 只转未观测的点，链条中间有实测点时会把链拧断（第 1.3 节）；
- `anchorsValid_` 为假时用手背质心当 CMC 枢轴，偏十几毫米。绕一条偏了 15mm 的**直线**
  转 80°，点位移误差 ≈ 15 × 2sin40° ≈ **19mm**，比一节指骨还长（`thumbFixSkip=5` 这一档）；
- 掌骨轴用 `unit(marker5 − anchor)` 估，带 11~16mm 的径向贴球偏置，方向误差 20° 量级。

第 3.1 步生效之后它没有存在价值。如果你想保守一点，就把它的触发条件收紧成
"**拇指三点全部未观测**"（此时转整条链是自洽的），其余情况一律跳过。

---

## 第 4 步：接 `seg_rot6d`（拇指的兜底路径）

`hm20_v7.onnx` 有 `seg_rot6d (1,16,6)` 输出，`Hm20OnnxBackend.hpp:371` 检测了它
（`haveOut_.seg`）但**从来没请求、没消费**。

我验过它的性质（`probe_thumb_axis.py` 第 ① 节会复现）：
- 输入整体旋转平移后 `seg_rot6d` 完全不变，`seg_rot6d[0]` 恒为单位阵（误差 0.12°）
- → 它是**手掌规范系下的 16 段姿态**，`R_world(s) = wristR · C · seg_rot6d[s]`，`C` 是常数

### 4.1 后端

`kOnnxOutputNames()` **尾部**追加（前面的下标绝对不能动）：

```cpp
inline const std::array<const char*, 6>& kOnnxOutputNames() {
    static const std::array<const char*, 6> n = {
        "log_assign", "pos", "joint_ang", "hand_logit", "pose_conf", "seg_rot6d"};
    return n;
}
```

`InferenceOutput` 加：

```cpp
    std::array<Mat3, 16> segR{};     // 手掌规范系
    bool hasSegR = false;
```

`Hm20OnnxBackend::run()` 里照 `pose_conf` 那段的写法加：

```cpp
    const int iSr = slot("seg_rot6d");
    if (valid(iSr) && haveOut_.seg) {
        auto sh = res[size_t(iSr)].GetTensorTypeAndShapeInfo().GetShape();
        if (sh.size() == 3 && sh[1] == 16 && sh[2] == 6) {
            const float* sr = res[size_t(iSr)].GetTensorData<float>();
            for (int s = 0; s < 16; ++s) out.segR[size_t(s)] = rot6dToMat(&sr[s * 6]);
            out.hasSegR = true;
        }
    }
```

`rot6dToMat`（Gram-Schmidt，跟训练侧一致）：

```cpp
inline Mat3 rot6dToMat(const float* v) {
    Vec3 a1{double(v[0]), double(v[1]), double(v[2])};
    Vec3 a2{double(v[3]), double(v[4]), double(v[5])};
    Vec3 b1 = detail::normalize(a1);
    const double p = detail::dot(b1, a2);
    Vec3 b2 = detail::normalize(Vec3{a2[0]-p*b1[0], a2[1]-p*b1[1], a2[2]-p*b1[2]});
    Vec3 b3 = detail::cross(b1, b2);
    return Mat3{b1[0], b2[0], b3[0], b1[1], b2[1], b3[1], b1[2], b2[2], b3[2]};  // 列=基
}
```

### 4.2 标定常数 `C`（模板系 → 网络规范系）

用**四指**（它们的几何解可信）在线做旋转平均：

```
C ≈ mean_s,frames [ (wristR)ᵀ · R_geo_world(s) · seg_rot6d[s]ᵀ ]
```

只在 `segSource[s] == Geometry` 且 `wristPoseValid` 的帧累计，四元数平均后重正交化。
几百帧就收敛，标一次存起来。

### 4.3 用法

拇指三段在 `fingerIkValid[0] == false` 的帧改用：

```
R_world(s) = wristR · C · seg_rot6d[s] · K_s
```

`K_s` 是拇指三段的常数修正（就是那 73° 缺口在姿态侧的体现）。
**`K_s` 要在"拇指三点全部可见"的帧上标定**（文档说拇指可见率约 45%，样本足够）：
那些帧的几何解可信，直接解 `K_s = (wristR·C·seg_rot6d[s])ᵀ · R_geo_world(s)`，做旋转平均。

这样得到的是**从你自己那只手上量出来的**修正量，不是对着画面调出来的常数，
而且手性自动正确 —— 因为它是从你的实测数据里解出来的。

---

## 第 5 步（可选但收益最大）：重贴手背 5 点

如果第 0 步量出来自歧义 `< 3mm`：

- 不要贴成一圈近似均匀的五边形
- **让绕质心的极角间隔明显不均匀**（实测 1.77 → 8.21mm，冷启动正确率 18% → 100%）
- 其次把其中一个点垫高 ≥10mm 打破共面（1.77 → 5.71mm）
- 两个一起做能到 10.93mm

**唯一的风险**：布局偏出模型的训练分布，"手背 vs 手指"这个二分类可能退化。
所以改完盯着面板的手背认领数看。不过第 2 步之后手背标签已经完全不靠模型了，
模型只需要判对"这个点属于手背"这个二分类 —— 而且刚体求解连这个都不需要
（它在全部候选点里搜）。所以这个风险比听上去小。

---

## 一句话优先级

| | 改动量 | 解决什么 | 判据 |
|---|---|---|---|
| 第 1 步 | 3 行 | 钉住的点、正反馈 | `dorsumTrackFixed==0`、钉点消失 |
| 第 2 步 | 1 个文件 + 1 个函数 | 手背标签翻转、错的时候不再谎报 | 稳态错标 0%、`dorsumSolveReason` |
| 第 3 步 | **1 行 + 1 参数** | **拇指外翻的根因** | `fingerIkRmseMm[0]` 10mm+ → 3~5mm，`拇✓` |
| 第 5 步 | 贴点 | 冷启动建锁 18% → 100% | 自歧义 1.77 → 8.2mm |
| 第 4 步 | 后端 + 标定 | 拇指遮挡时的兜底 | 拇指段跳变 p99 |

**第 3 步的那一行是整个包里最该先做的。** 它只有一行，而且它是唯一一处能解释
"为什么每一次都外翻、从来没有一次是对的"的地方 —— 确定性的现象必须有确定性的原因，
73° 的常数缺口正是这样一个原因。

---

## 我没验证到的部分（说清楚，免得又白花时间）

| 项 | 状态 |
|---|---|
| `DorsumRigidSolver.hpp` 编译 + 蒙特卡洛 | ✅ g++ -std=c++20 编译零 error，上面那两张表是这份代码跑出来的 |
| `seg_rot6d` 是手掌规范系、seg0 恒为单位阵 | ✅ 在你上传的 `hm20_v7.onnx` 上实测 |
| 拇指铰链轴 17.4°、四指 0.3~1.7° | ✅ 同上，60 组样本 |
| 手背自歧义 1.77mm | ⚠️ 用的是文档里记的半径 + **我假设的近似均匀角度**。你的真实模板必须自己跑 `probe_dorsum_reversal.py` |
| `thumbPronation0` 的**符号** | ❌ 只有推导，没有实测。必须做 3.2 那个 A/B |
| 刚体求解在真实点云上的表现 | ❌ 蒙特卡洛用的是高斯噪声 + 均匀杂点，跟真机的粘连/重影不是一回事 |
| Qt 层（面板新增诊断行） | ❌ 本机没有你的工程，没编过 |
