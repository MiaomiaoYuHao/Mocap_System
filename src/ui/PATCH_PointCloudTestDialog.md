# 接 IK：改 PointCloudTestDialog

## 1. 加 include（文件头部，第 9 行 `SkeletonAssocWorker.hpp` 之后）

```cpp
#include "estimate/Hm20IkRefiner.hpp"
```

## 2. 加成员（PointCloudTestDialog.hpp，跟 skeletonWorker_ 放一起）

```cpp
std::shared_ptr<hm20::Hm20IkRefiner> skeletonIk_;
```

## 3. 构造 IK 并挂上（PointCloudTestDialog.cpp，`skeletonWorker_->configure(...)` 那一行【之前】插入）

```cpp
        // ---- IK 精修：手指分段朝向 ----
        // computeSegmentQuats() 用 marker[b]-marker[a] 当骨轴，而球贴在指节
        // 背侧、离骨轴 11~17mm 且相邻两节方位角不同 —— 连线必然偏离骨轴，
        // 该文件 615 行的注释也承认"这里剩下的 11.0° 中位偏差消不掉"。
        // 合成数据实测：10.2° 中位 / 21.6° p90  ->  3.02° 中位 / 8.08° p90。
        // 代价 0.12ms/帧，对 30Hz 无影响。
        skeletonIk_ = std::make_shared<hm20::Hm20IkRefiner>();
        skeletonIk_->setBackTemplate(skelTmpl.backMarkers);   // 字段名按 Hm20Template 实际的改
        for (int f = 0; f < 5; ++f) {
            const auto& fp = fingerParam(f);
            skeletonIk_->setFingerParams(
                f, Vec3{fp.anchor[0], fp.anchor[1], fp.anchor[2]},
                {fp.lengths[0], fp.lengths[1], fp.lengths[2]});
        }
        skeletonWorker_->setIkRefiner(skeletonIk_);   // 若 worker 没有这个转发，
                                                      // 见下面第 5 条
```

## 4. 【必须确认】anchor / lengths 的来源

上面用的是 `fingerParam(f)` 里写死的常量。如果 `HandTemplateStore` 标定出了
逐用户的 anchor/lengths，**用标定值**，否则 IK 拟合的是"平均手"，
marker 残差会从 1.8mm 涨到 4.6mm 左右。

`Hm20TemplateAdapter.hpp` 里应该已经做过这个合成，直接复用它的结果最好。

## 5. worker 侧的转发

`SkeletonAssocWorker` 目前没有 `setIkRefiner()`。两条路：

**A. 加一个转发（推荐）** —— 在 SkeletonAssocWorker.hpp 里：

```cpp
    void setIkRefiner(std::shared_ptr<hm20::IHm20IkRefiner> ik) {
        QMutexLocker lk(&mutex_);          // 按该文件已有的同步方式
        pendingIk_ = std::move(ik);
    }
    // ensureInitialized() 里 associator 建好之后：
    //     if (pendingIk_) assoc_->setIkRefiner(pendingIk_);
```

**B. 在 configure() 里一起传** —— 给 `Hm20Config` 加一个
`std::shared_ptr<IHm20IkRefiner> ik;` 字段，`ensureInitialized()` 里
`assoc_->setIkRefiner(cfg_.ik)`。

【注意线程】`Hm20IkRefiner` 有内部状态（`prevAngles_` 时序先验），
只能被 worker 线程用。GUI 线程构造完就交出去，之后别再碰它。

## 6. 冷启动/丢失重捕时清先验

跟踪断了以后 `prevAngles_` 是旧姿势，会把 IK 往错的方向拉。
在"骨架连续多帧无效"或"用户切换手"的地方调：

```cpp
skeletonIk_->resetPrior();     // 下一帧走多起点搜索
```

（`Hm20SkeletonAssociator` 里如果已经有重捕检测，接到那里最好。）

## 7. 验收

`refine()` 只在 `fingerValid`（该指 ≥2 个 marker 可见）时覆盖 segQuat，
否则保留网络预测。所以接上之后：

- 手指连线的**朝向**应该明显更稳（不再随两球连线抖）
- 被遮挡的点位置由 FK 摆出来，骨长恒定、关节在限位内
- 真观测点的位置**不会被改**（IK 只补遮挡点，观测本身更准）

打 `R.rmseMm[f]` 和 `R.nObs[f]` 能定位大部分问题：
rmse 持续 >8mm 说明 anchor/lengths 跟实际手对不上。
