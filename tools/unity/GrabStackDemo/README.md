# Unity 抓取 + 堆叠 Demo（GrabStackDemo）

运动学抓取方案：用“拇指-食指-中指捏合”判定抓取，物体挂到手掌锚点，
松开时在堆叠区上方就吸到整齐槽位，否则自然掉落。相比纯物理抓取，对
标记点抖动更鲁棒，demo 不容易翻车。

## 场景搭建步骤
1. 把 `MocapHandReceiver.cs` 挂好并绑定 `wrist` + `segments[16]`，确认手能跟动。
2. 建一个手掌锚点空物体：放到手模型掌心位置，作为 `palmAnchor`。
   （空物体不随骨骼动，拖到 `DemoGrabber.palmAnchor`。）
3. 建几个方块：每个挂 `Rigidbody`(Use Gravity 勾上) + `BoxCollider` + `GrabbableObject`。
   方块建议 0.05m 左右。放一张桌子/地面（带 Collider）。
4. 建一个 `StackZone` 空物体放在桌面堆叠位，调 `cellSize`/`snapRadius`。
5. 建 `DemoGrabber`：拖 `receiver`、`stackZone`、三个指尖、`palmAnchor`。
   指尖可点组件右键 `自动填充指尖` 自动从 receiver.segments 取（3/6/9）。
6. （可选）`ObjectSpawner` 生成一排方块；`OrbitCamera` 环绕拍摄。

## 关键旋钮
- 抓不住：调小 `pinchGrabDist` 或把方块放近（`grabRadius` 加大）。
- 抓太松/容易误抓：调大 `pinchGrabDist`、减小 `grabRadius`。
- 松开抖动导致物体反复掉：`pinchReleaseDist` 要比 `pinchGrabDist` 大（默认 0.05 vs 0.02）。
- 堆叠对不齐：调 `StackZone.cellSize` 到方块尺寸 + 一点间隙。

## 建议
- 手摊开/捏合动作做慢一点、幅度大一点，demo 更稳。
- 想让手在场景里“走过去抓”：`MocapHandReceiver.applyWristPosition = true`，
  并把物体/堆叠区放在你手能到的范围；否则固定手腕、把物体放在指尖可达范围。
- 想更炫：给 `GrabbableObject` 换材质/加描边，堆叠成功加音效或 +1 计数。