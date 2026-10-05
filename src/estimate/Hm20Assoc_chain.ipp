// ===========================================================================
// Hm20Assoc_chain.ipp —— Hm20SkeletonAssociator 类体分片：链式续解（遮挡点补全）
//
// 实测遮挡点里 81% 走这条路，是补全的主力。
// 自带一整套跨帧学习状态：关节平面、PIP 保持、DIP 耦合、拇指 IP 轴。
// 【这些状态只有本文件的两个函数碰】所以改这里不用读别的文件。
//
// 【本文件被 #include 在 class 体内部】内容即类内定义（隐式 inline）。
// 不要加 include guard、不要加 namespace。开头的 access 说明符是显式写死的，
// 不依赖上一个分片以什么结尾——调换 include 顺序不会改变可见性。
// 独立语法检查：见 tests/test_assoc_fragments.cpp
// ===========================================================================
private:
    // 【腕部系】各指的弯曲平面法向。存世界系是错的：它是相对手掌的解剖量，
    // 遮挡期间手一转，世界系的值就不再对应同一个解剖平面。
    mutable std::array<Vec3, 5> planeLocal_{};
    mutable std::array<bool, 5> hasPlane_{};
    // 【未归一化的叉积累加器】见 learnJointPose 里那段说明：|cross| = |a||b|sin(θ)，
    // 而 sin(θ) 正比于这一帧对法向【方向】的信息量。直接累加原始叉积就等价于
    // 按信息量加权平均 —— 伸直帧自己贡献得少，不需要 planeLearnMinDeg 那道硬门。
    mutable std::array<Vec3, 5> planeAcc_{};
    mutable std::array<bool, 5> hasPlaneAcc_{};
    // 累加器吃进了多少帧。【必须报出来】平面现在【只】从三点全见的帧学，
    // 这一根手指的 dp 长期被遮的话就一帧都学不到 —— 而 hasPlane_ 只说
    // \"有没有\"，说不出\"够不够\"。排查\"改完还是不准\"时，先看这个数。
    mutable std::array<int, 5>  planeSampleN_{};
    // anchor 的【出平面】自检投票。verifyAnchors 查长度、查"比 pp 更靠近手背
    // 中心"，唯独查不了方向 —— 而链式续解坏就坏在方向上。这里有实测平面在手，
    // 直接量 anchor->pp 偏出平面多少。
    mutable std::array<int, 5> anchorPlaneN_{};
    mutable std::array<int, 5> anchorPlaneOk_{};
    // 【PIP 关节的弯曲平面，跟 planeLocal_(MCP平面) 必须分开】
    // 此前两者共用 planeLocal_ 一个变量，是实打实的 bug：
    //   planeLocal_    = cross(anchor->pp, pp->mp)   MCP 弯曲平面(learnJointPose 写)
    //   pipPlaneLocal_ = cross(pp->mp,     mp->dp)   PIP 弯曲平面(远节段写)
    // 两者差一整个骨段的夹角。你的录制实测：食指/中指两套法向夹角中位
    // 【41.0°】，而 caseB 摆出来的中节出平面误差 41.9°/44.8° —— 精确吻合。
    // 真实手指几乎完美共面（中节可见帧出平面角中位仅 1.6°），所以那 41°
    // 全部是串用变量造成的，不是模型局限、也不是"平面在深屈曲时失效"
    // （实测侧偏与屈曲深度基本无关：食指 34.3°→36.0°，是恒定偏差）。
    // 远节在弯曲平面内的相位角（融合路径的时序状态）。见 continueChainForOccluded
    // 里那段说明：网络给方向、这个低通给连续性。
    mutable std::array<double, 5> dipPhase_{};
    mutable std::array<bool, 5>   hasDipPhase_{};
    mutable std::array<Vec3, 5> pipPlaneLocal_{};
    mutable std::array<bool, 5> hasPipPlane_{};
    // 当前的 planeLocal_ 是否只是解剖学种子（还没被真实观测替换过）。
    // 种子用于"没有 anchor 时也能立起弯曲平面"，第一次量到真值就整体换掉。
    mutable std::array<bool, 5> planeSeeded_{};
    // 【腕部系】上一帧的近节骨轴。同上，原来存世界系，遮挡时手一转手指就外摆。
    mutable std::array<Vec3, 5>   prevU0Local_{};
    mutable std::array<bool, 5>   hasPrevU0_{};
    // 遮挡入口参照：进入"中节也丢"那一刻的近节骨轴(腕部系)和当时的 PIP 角。
    // 情形B 的推算是相对它的【绝对量】，不是逐帧累加 —— 见那里的说明。
    mutable std::array<Vec3, 5>   u0Entry_{};
    mutable std::array<double, 5> pipEntry_{};
    mutable std::array<bool, 5>   hasEntry_{};
    // 无标定退化档的入口参照：近节球在手掌系里的位置 + 屈曲时的运动方向。
    // anchorsValid_ 为假时 u0 是沿用来的、自己跟自己比恒为 0，拿不到驱动量；
    // 而近节球【位置】始终是实测的，见情形B 里的说明。
    mutable std::array<Vec3, 5>   pEntryLocal_{};
    mutable std::array<Vec3, 5>   flexDirEntry_{};
    mutable std::array<bool, 5>   hasPEntry_{};
    mutable std::array<double, 5> pipHold_{};     // 最后一次量到的 PIP 角，遮挡时从它推
    mutable std::array<bool, 5>   hasPipHold_{};
    mutable double thumbIpAng_ = 0.0;
    mutable Vec3   thumbIpAxisLocal_{};
    mutable bool   hasThumbIp_ = false;
    mutable std::array<bool, kNumMarkers> chainFilledThisFrame_{};
    // 各指连续处于"中节也丢"状态的帧数。见 caseB 的时效说明。
    mutable std::array<int, 5> caseBFrames_{};
    // 遮挡期相对入口姿势的偏离(rad)。老化判据 —— 静止时它停在噪声本底上，
    // 姿势真变了才增长。见 Hm20Config::occStaleRad0 那段实测表。
    mutable std::array<double, 5> coupleExc_{};
    mutable std::array<bool, 5>   hasExc_{};   // coupleExc_ 是否真的测到了
public:
private:
    bool   aiHandIsRight_ = true;
public:

    void TestHook_chainContinue(SkeletonFrameResult& r,
                                const std::array<bool, kNumMarkers>& seen,
                                const std::array<Vec3, 5>& anchors) {
        anchorsMm_ = anchors; anchorsValid_ = true;
        learnBoneLengths(r, seen);
        learnJointPose(r, seen);
        continueChainForOccluded(r, seen);
    }
private:

    // 每节骨骼朝向。手背段用 Kabsch 解出的腕部 R。
    //
    // 【修改·配对】原实现里 k==0 与 k==1 算出的是同一个向量：
    //     k==0: m=5+3f     -> a=m,   b=m+1  => P[5+3f+1] - P[5+3f]
    //     k==1: m=5+3f+1   -> a=m-1, b=m    => P[5+3f+1] - P[5+3f]   ← 完全相同
    // 结果是每根手指的近节和中节四元数【恒等】（实测差异中位 0.000000°），
    // 3 节骨只有 2 个独立朝向。现改为：
    //     近节 prox: pp -> mp   （保持原向量，实测它对近节最准）
    //     中节 mid : pp -> dp   （跨两节的长基线，对中节骨轴偏差最小）
    //     远节 dist: mp -> dp
    // 标定过的关节手模型上实测（0.5mm 点云噪声）：骨轴误差中位 13.0°->11.0°，
    // p90 23.3°->20.0°，且近节/中节四元数差异中位 8.8°（不再恒等）。
    //
    // 注意：3 个 marker 只能给出 2 个独立差分向量，求 3 节骨朝向本质上是
    // 欠定的，上面只是"在可用配对里选最优"，不是解。真正的解要靠
    // IHm20IkRefiner。这里剩下的 11.0° 中位偏差消不掉。
    // -------------------------------------------------------------------------
    // 拇指预测点的旋前回正
    //
    // 【为什么需要】训练用的 hand_rig.py 里，拇指掌骨的轴向旋前【完全耦合在
    // CMC 外展上】(axial = thumb_axial_k * a[1])，没有常数基线。代入它自己的
    // 关键姿态表：握拳时 CMC展=0.18、k∈[0.35,0.80] -> 旋前只有 4~8°。
    // 而真人第一掌骨的解剖旋前是 80~90° 且【中立位就有】—— 这正是拇指能对掌
    // 的原因。所以模型学到的先验是"拇指跟四指一样往前弯"。
    // 观测到的时候没事(网络只是给点打标签)；一旦遮挡、pos 头必须【生成】位置，
    // 这个错先验就直接体现成"远节朝前伸而不是朝掌心"。
    //
    // 【补偿形式】rig 里缺的是一个绕掌骨长轴的常数 rot_x(Δ)。绕轴旋转不改变
    // 轴本身，所以掌骨【方向】是对的，错的是 CMC 之后被整体转掉的那部分。
    // 于是补偿 = 绕(CMC锚点, 掌骨轴)把预测点转回 Δ。
    //
    // 【三条局限，用之前必须知道】
    //  ① 掌骨轴只能估：手上只有球没有骨头，unit(marker5 - anchor) 带着
    //     11~16mm 的径向贴球偏置，方向误差 20° 量级。绕一个偏 20° 的轴转 80°，
    //     能把方向拉回大半，但【不精确】。这是一阶补偿，不是解。
    //  ② 要 wristPoseValid + anchor 有效，否则没有 CMC 锚点，直接跳过。
    //  ③ 【它修不了先验本身】。几何回正只能把预测点摆进正确的平面，摆过去之后
    //     那个姿势仍然不一定像真人握拳。根治要改 rig 重训。
    //
    // 【只动 observed==false 的点】实测点一律不碰 —— 实测的本来就是对的，
    // 碰了只会把好的弄坏。
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // 链式续解：远端被遮挡时，顺着【可见的近端】把它算出来，而不是冻住。
    //
    // 【跟之前那个失败的"融合外推"有本质区别】那次是在三维空间里自由外推位置，
    // 三个自由度全放开，结果骨轴方向从 21.4° 崩到 40.5°。这里不一样：
    //   · 平面 —— 由可见的 anchor->pp->mp 钉死，远节必须共面（手指是平面连杆）
    //   · 长度 —— 用标定出来的骨长，不可变
    //   · 只剩【弯曲角】一个自由度，而它由 DIP 耦合从可见的 PIP 角推出
    // 三个自由度砍到零，摆出来的点天然在人体可达范围内，不会偏出去。
    //
    // 【为什么不等 IK】IK 要整根手指的残差过门限才放行；而这里只要近端两颗球
    // 可见就能算，是闭式的、无迭代的。IK 退档的帧正是最需要它的帧。
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // 骨长回正：补出来的点，方向信任估计、距离用标定值。
    //
    // 【为什么只动补出来的点】实测点来自三角化，亚毫米，动它只会变差。
    // 【父点必须是可见的】拿一个同样是补出来的点当基准，误差会沿链累积。
    // 所以只在"父可见、子不可见"这一档上做 —— 那正是最常见的遮挡形态。
    // -------------------------------------------------------------------------
    // 三点全可见时量骨长。【必须独立于 chainContinue】
    //
    // 原来这个学习埋在 continueChainForOccluded() 里，而那个函数开头就
    // `if (!cfg_.chainContinue) return;` —— 开关一关，pmLen_/mdLen_ 永远学不到，
    // 于是依赖它的骨长回正也永远不生效。而骨长是标定量，跟"要不要做链式续解"
    // 是两件独立的事。
    // 三点全可见时，把【关节角和弯曲平面】也记下来。
    //
    // 【为什么必须在这里记】原来 pipHold_/planeN_ 只在 continueChainForOccluded
    // 的 caseA 分支里更新，而那个函数只在【有遮挡】时才做事 ——
    // 于是"可见时不记、遮挡时没得用"，caseB 的前置条件永远不满足。
    // 实测：拇指骨长学到了（35.0mm），但 hasPipHold_ 和 hasPlane_ 恒为 0，
    // 链式续解一次都跑不起来。
    //
    // 拇指尤其明显：它走 thumbOwnAxis 分支，那一支【不更新 hasPlane_】，
    // 所以连 caseA 都不给它记平面。
    // 【out 改成非 const】这个函数原本只读 out，现在还要把三个逐指诊断量
    // 写回去（anchorOkPerFinger / planeSampleN / anchorOutOfPlaneDeg）。
    // 它们只能在这里算 —— 出平面角要同时拿到 anchor->pp 和【当帧刚学到的】
    // 实测平面，全帧只有这一个地方两者都在手上。
    void learnJointPose(SkeletonFrameResult& out,
                        const std::array<bool, kNumMarkers>& seen) {
        using namespace detail;
        // 【逐指诊断先无条件置默认值】下面每一条 continue 都会跳过一根手指，
        // 不先置默认值的话，被跳过的那几根会留着【上一帧】的数字 —— 而
        // "这一帧没算"和"这一帧算出来是这个值"在排查时是完全不同的两件事。
        for (int f = 0; f < 5; ++f) {
            out.anchorOkPerFinger[size_t(f)]    = anchorsValid_ && anchorOk_[size_t(f)];
            out.planeSampleN[size_t(f)]         = planeSampleN_[size_t(f)];
            out.anchorOutOfPlaneDeg[size_t(f)]  = -1.0;   // -1 = 本帧量不出来
        }
        for (int f = 0; f < 5; ++f) {
            const int pp = 5 + 3*f, mp = pp + 1, dp = pp + 2;
            if (!seen[size_t(pp)] || !seen[size_t(mp)]) continue;
            // 近节骨轴 u0：有 anchor 就用 anchor->pp（准）。
            //
            // 【没有 anchor 时不再直接放弃】原来这里是 `if (!anchorsValid_) continue;`，
            // 后果是 hasPlane_ 永远学不到 -> 链式续解的 caseB 守卫永远不过 ->
            // 遮挡点 100% 回落到网络原始预测，而网络预测带着 40~60° 的侧偏。
            // 实测两份录制都栽在这里：anchor 要么被解到手背反方向（verifyAnchors
            // 正确拦下），要么五指塌缩成一团（anchor 间距 3.9~6.4mm，真实应
            // 60~70mm）—— 也就是说【anchor 不可靠是常态】，而整条遮挡预测链
            // 却把它当成硬前提。
            //
            // 退化方案：用 pp->mp 和 mp->dp 这两个【纯实测骨轴】学平面。
            // 少了 anchor 只是拿不到 MCP 的绝对角度（pipHold_ 那一路），
            // 但弯曲平面本身完全定得出来 —— 而平面才是链式续解真正依赖的量。
            // 这条路不引入任何标定依赖，三点全见时就能学到。
            Vec3 u0{}, u1{};
            bool haveMcpAngle = false;
            if (anchorsValid_ && anchorOk_[size_t(f)]) {
                // 跟 continueChainForOccluded 里的写法保持一致（那里是手写展开的）
                const Vec3& am = anchorsMm_[size_t(f)];
                const Vec3 A{out.wristR[0]*am[0] + out.wristR[1]*am[1] + out.wristR[2]*am[2] + out.wristT[0],
                             out.wristR[3]*am[0] + out.wristR[4]*am[1] + out.wristR[5]*am[2] + out.wristT[1],
                             out.wristR[6]*am[0] + out.wristR[7]*am[1] + out.wristR[8]*am[2] + out.wristT[2]};
                u0 = sub(out.markers[size_t(pp)].posWorld, A);
                u1 = sub(out.markers[size_t(mp)].posWorld,
                         out.markers[size_t(pp)].posWorld);
                haveMcpAngle = true;
            } else {
                // 无 anchor：用两段实测骨轴。这样定出来的平面跟有 anchor 时
                // 【是同一个平面】——手指三节共面，用哪两段算叉积都一样。
                if (!seen[size_t(dp)]) continue;   // 需要三点全见
                u0 = sub(out.markers[size_t(mp)].posWorld,
                         out.markers[size_t(pp)].posWorld);
                u1 = sub(out.markers[size_t(dp)].posWorld,
                         out.markers[size_t(mp)].posWorld);
            }
            const double n0 = norm(u0), n1 = norm(u1);
            if (n0 < 1e-6 || n1 < 1e-6) continue;
            u0 = mul(u0, 1.0/n0); u1 = mul(u1, 1.0/n1);
            // 【MCP 角只在有 anchor 时才有意义】没有 anchor 时上面算的是
            // PIP 角，不能拿它当 pipHold_（那是遮挡入口的 MCP 参照角）。
            // 写错的话 caseB 会按一个错的起始角摆中节，比不摆更糟。
            if (haveMcpAngle) {
                pipHold_[size_t(f)] = std::acos(std::clamp(dot(u0, u1), -1.0, 1.0));
                hasPipHold_[size_t(f)] = true;
            }

            // ================= 弯曲平面：只从实测三点学，永不走 anchor =========
            // 【为什么把平面和 MCP 角拆开】原来两者都从同一对 (u0,u1) 出来，
            // 有 anchor 时 u0 = anchor->pp。
            //
            // 对【角度】没问题：pipEntry 是用这个 anchor 量的，caseB 里又用
            // 同一个 anchor 把它转回去，系统偏差自己抵消了。真机反解出来的
            // 旋转角 f1 53.2° / 真值 49.2°，f4 66.6° / 62.4° —— 确实对得上。
            //
            // 对【平面】不抵消。法向是方向量，歪多少就是多少，而且
            // cross(u0,u1) 对 u0 的出平面分量有 1/sin(MCP角) 的放大。
            // 真机实测（三点全实测 + 骨长自洽 + 无重号的干净帧，n=272~301）：
            //     指   u0=anchor->pp 出平面   学到的平面 vs 真平面
            //     f0        36.6°                  77.9°
            //     f1        40.1°                  53.7°
            //     f2        59.7°                  63.3°
            //     f3        30.8°                  36.6°
            //     f4        17.8°                  18.8°
            // 真平面法向的 bootstrap 不确定度只有 0.7~2.8°，所以这不是参照
            // 不准，是真的歪了。后果：continueChainForOccluded 绕一个歪
            // 19~78° 的轴做 Rodrigues —— 转的角度大致对，转的方向全错，
            // 手指不往掌心弯而是往侧面摆。无名指最惨：骨轴方向误差 114.4°，
            // 而同一批遮挡段里"冻结上一帧"只有 28.8°、网络 pos 头只有 22.1°。
            //
            // cross(pp->mp, mp->dp) 【定义上】就是真平面，唯一的问题是弯折角
            // 小时信噪比低（真机 PIP 张角中位仅 12~25°）。那是【方差】，
            // 可以靠累加压下去；anchor 那条路是【偏差】，累加多少帧都消不掉。
            if (!seen[size_t(dp)]) continue;
            const Vec3 a1 = sub(out.markers[size_t(mp)].posWorld,
                                out.markers[size_t(pp)].posWorld);
            const Vec3 b1 = sub(out.markers[size_t(dp)].posWorld,
                                out.markers[size_t(mp)].posWorld);
            const Vec3 nrW = cross(a1, b1);
            if (norm(nrW) < 1e-9) continue;
            // 【累加未归一化的叉积，不设门限】|cross| = |a||b|sin(θ) 本身就是
            // 这一帧的信息量权重，伸直帧自动贡献得少。
            // 【原来那道门的代价】planeLearnMinDeg=14°，而真机 PIP 张角中位
            // f0 12.3°、f2 12.5° —— 这两根 62%/69% 的帧被整帧扔掉。正是因为
            // 扔得太狠，才不得不退回 anchor 那条有偏的路。
            {
                Vec3 nrL = matVecT(out.wristR, nrW);        // 腕部系
                Vec3 acc = hasPlaneAcc_[size_t(f)] ? planeAcc_[size_t(f)] : Vec3{};

                // ---- 半球（符号）必须锚在解剖上，不能只锚在自己的历史上 ----
                // 【为什么符号是要命的】caseB 里转角 a 恒为正：
                //     u1 = Rodrigues(u0, nrmB, a),  a = clamp(pipEntry + addRad, 0, 2.62)
                // 绕 +nrmB 转正角是屈曲，绕 -nrmB 转正角就是【背伸】。
                // 也就是说法向翻个号，整根手指往反方向摆 180°。诊断里
                // \"22% 的遮挡段位移方向 cos<0（整段往反方向走）\"，量级上正好
                // 是这一条能造出来的。
                //
                // 只跟自己的历史对齐（原来那种写法）等于把符号交给【第一帧】：
                // 第一帧要是从一个过伸/噪声帧起的号，之后每一帧都会被拉去跟
                // 那个坏号同向，再也纠不回来 —— 而且看不出来，因为法向本身
                // 一直很\"稳\"。
                //
                // 解剖参照：腕部系 Z=手背法向、Y=屈曲轴、X=远端方向，
                // 所以 lat = cross(Z, 骨轴) 就是该指的屈曲轴。正常屈曲时
                //     cross(pp->mp, mp->dp) = +lat · sin(PIP角)
                // （代进 a1=(cosα,0,-sinα)、a2=(cosβ,0,-sinβ)、β>α 可以直接验），
                // 两者【本来就该同号】。
                //
                // 【留 0.30 的余量：拿不准就不动】拇指的屈曲平面相对手背旋前
                // 近 90°，dot 落在 0 附近，那时这个判据是噪声 —— 交给历史对齐。
                const Vec3 dorsalL{0, 0, 1};
                const Vec3 boneL = matVecT(out.wristR, a1);
                Vec3 latL = cross(dorsalL, boneL);
                const double ln = norm(latL);
                bool signPinned = false;
                if (ln > 1e-6) {
                    latL = mul(latL, 1.0 / ln);
                    const double c = dot(nrL, latL) / std::max(1e-12, norm(nrL));
                    if (c < -0.30)      { nrL = mul(nrL, -1.0); signPinned = true; }
                    else if (c > 0.30)  { signPinned = true; }
                }
                // 解剖判据不决断时（拇指、骨轴贴近手背法向）才退回历史对齐。
                if (!signPinned && hasPlaneAcc_[size_t(f)] && dot(nrL, acc) < 0.0)
                    nrL = mul(nrL, -1.0);

                // 遗忘因子 0.97 ≈ 30 帧时间常数（60fps 下半秒）：姿势真变了跟得上，
                // 单帧噪声主导不了。
                acc = add(mul(acc, 0.97), nrL);
                planeAcc_[size_t(f)] = acc;
                hasPlaneAcc_[size_t(f)] = true;
                if (planeSampleN_[size_t(f)] < 1000000) ++planeSampleN_[size_t(f)];
                const double an = norm(acc);
                if (an > 1e-9) {
                    planeLocal_[size_t(f)] = mul(acc, 1.0 / an);
                    hasPlane_[size_t(f)] = true;
                    planeSeeded_[size_t(f)] = false;   // 这是实测值，不是种子
                }
            }

            // ---- 顺带给 anchor 做一次【真正管用的】自检 ----
            // 【为什么 verifyAnchors 不够】它查 |anchor->pp| 落在 15~60mm、
            // 且 anchor 比 pp 更靠近手背中心。真机五根里它只拦下小指 ——
            // 而食指(出平面 40.1°)和无名(30.8°)照样放行，那两根的平面分别歪
            // 53.7° 和 36.6°。长度对、方向错，正是最难查的那一档。
            // 这里有实测平面在手，直接量会被 1/sin 放大的那个分量。
            if (haveMcpAngle && hasPlane_[size_t(f)]) {
                ++anchorPlaneN_[size_t(f)];
                const double s = std::fabs(dot(u0, planeLocal_[size_t(f)]));
                // 报出实测出平面角(度)。u0 和 planeLocal_ 都是单位向量，
                // |dot| 就是出平面角的 sin。
                out.anchorOutOfPlaneDeg[size_t(f)] =
                    std::asin(std::clamp(s, 0.0, 1.0)) * 180.0 / 3.14159265358979323846;
                if (s < kAnchorMaxOutOfPlaneSin)
                    ++anchorPlaneOk_[size_t(f)];
                // 攒够样本再降级，别让开头几帧的噪声一票否决
                if (anchorPlaneN_[size_t(f)] >= kAnchorPlaneVoteMin &&
                    double(anchorPlaneOk_[size_t(f)]) < 0.5 * double(anchorPlaneN_[size_t(f)]))
                    anchorOk_[size_t(f)] = false;
            }
            // 本帧刚更新过的两个量，重新报一次 —— 上面那轮默认值是在累加器
            // 跑之前置的，降级也可能就发生在这一帧。
            out.planeSampleN[size_t(f)]      = planeSampleN_[size_t(f)];
            out.anchorOkPerFinger[size_t(f)] = anchorsValid_ && anchorOk_[size_t(f)];
        }
    }

    void continueChainForOccluded(SkeletonFrameResult& out,
                                  const std::array<bool, kNumMarkers>& seen) const {
        using namespace detail;
        // 【不再强依赖 anchorsValid_】链式续解要 anchor 只是为了定"近节骨轴"的
        // 起点方向。而近节骨轴完全可以用 pp->mp 近似 —— 那是【实测的两颗球】，
        // 不需要任何标定。
        // 原来卡在 anchorsValid_ 上的后果：自标定没走到"完成"时 anchor 不提交，
        // 链式续解第一行就 return，一个点都补不上（真机实测 IK0/链式0/AI8）。
        // 而链式续解本该是【独立于标定】的能力 —— 它用的全是实测量：球间距、
        // PIP 角、平面法向，没有一个来自标定。被标定状态卡住是没道理的。
        if (!cfg_.chainContinue || !out.wristPoseValid) return;

        for (int f = 0; f < 5; ++f) {
            // 【逐指判断 anchor 能不能用】原来是一个全局 bool。真机实测五根
            // anchor 的质量差得很远：|anchor->pp| 从 24.8mm 到 73.7mm，
            // anchor->pp 与实测 pp->mp 的夹角从 43° 到 79°。一个全局 bool
            // 只能一起用或一起不用 —— 结果是好的那几根被坏的那根拖着一起
            // 退化，或者（现状）坏的那根被好的那几根带着一起放行。
            // 见 verifyAnchors()。
            const bool useAnchor = anchorsValid_ && anchorOk_[size_t(f)];
            const int pp = 5 + 3*f, mp = pp + 1, dp = pp + 2;
            // 近节必须可见 —— 它定平面、定 PIP 的起始边，没有它整条链无从谈起。
            if (!seen[size_t(pp)]) continue;
            // 两种可续解的情形：
            //   A. 只丢远节        -> PIP 角【直接量得出来】
            //   B. 中节+远节都丢   -> PIP 角量不出来，用【最后一次量到的值 +
            //                        近节角速度推它继续弯】。握拳时这是常态，
            //                        上一版漏了这个分支，于是四指整个退回网络预测，
            //                        中节相对近节出现 135~154° 这种不存在的角度。
            // 【只保留情形 A】仿真实测(tools/test_chain_axis.py，真模型闭环)：
            //                    中节骨轴误差  中位   p90    max
            //   现状(预测点差分)              22.9°  70.3  115.7
            //   情形B(PIP靠近节角速度外推)    43.8° 114.5  131.2   <- 差一倍
            //
            // 情形B 的错误在于：dPip 是从【近节骨轴的转角】推出来的，而近节转 1°
            // 不代表 PIP 也转 1° —— 这两个关节不是刚性联动。开环积几十帧就到 40°+。
            // 而网络预测的点虽然位置误差 10~30mm，却【每帧都被真观测约束】，
            // 不会自己漂。差分放大是坏事，但"每帧重新锚定"胜过"开环积分"。
            //
            // 情形A 不一样：PIP 角是【当帧量出来的】，不是推的，所以可靠。
            const bool caseA = seen[size_t(mp)] && !seen[size_t(dp)];

            // 【拇指多一种可续解的情形】中节+远节都丢，但掌骨和近节还在。
            //
            // 四指在这种情形下确实推不了（情形B，见上面那段：靠近节角速度
            // 外推 PIP，实测比不做还差一倍）。但拇指不一样 ——
            // 它的 ThumbIpTracker 用【当帧实测的 MCP 角】驱动 IP，
            // 而 MCP 角只需要 anchor->pp 和 pp->mp 两个方向：
            //   · anchor->pp 需要 anchor（模板给的）和 pp（tmc 球）
            //   · pp->mp     需要 tmc 和 tpp 两个球
            // 也就是说【只要 tpp 还在，驱动量就在】，跟 tdp 丢不丢无关。
            // 这跟四指的开环外推是两回事：那个没有当帧观测，这个有。
            //
            // 真机症状：拇指远端两节一遮挡就整个退回网络 pos 头，
            // 位置跳变、连线是直的不符合人体 —— 因为 `if (!caseA) continue`
            // 把它拦在门外，跟踪器根本没机会跑。
            //
            // 【tpp 也丢的话仍然推不了】那时 MCP 角量不出来，没有驱动量。
            // 那种情况下退回网络 pos 是对的，不硬猜。
            // 【情形B 重新启用，但改成"冻结角度"而不是"外推角度"】
            // 需要的东西：近端点可见（上面已查 seen[pp]）、角度量到过、
            // 弯曲平面学到过、骨长标定过。缺任何一个都不猜。
            // 【情形B 有时效】冻结姿势的前提是"这段时间里姿势没大变"。
            // 实测（合成刚体运动 + 姿势正弦变化）：
            //     姿势变化      tpp    tdp     网络pos头
            //     不变(纯刚体)  0.00   0.00    9.1 / 10.8
            //     ±9°           3.52   6.71    9.1 / 10.8
            //     ±20°          8.19  15.57    9.1 / 10.8   <- tdp 反而更差
            // 也就是说短时遮挡冻结完胜，长时遮挡（姿势早变了）不如网络。
            //
            // 【但"变旧"要按姿势偏离算，不是按帧数】原来是
            //     caseBFrames_ > 48  ->  硬切回网络预测
            // 真机症状：从可见转遮挡很顺，但【保持遮挡一会儿就弹开来】。
            // 合成复现：第 60 帧起遮挡，第 108 帧（= 60 + 48）chainContinued
            // 从 2 掉到 0，帧间位移 0.4mm -> 7mm 并一直抖下去。
            // 见 occStaleRad0 那段的完整实测表：手静止时链式解永远优于回退
            // （0.3mm vs 6.1mm），按帧数老化毫无道理；只有姿势在变才会旧。
            if (!seen[size_t(mp)]) ++caseBFrames_[size_t(f)];
            else                  { caseBFrames_[size_t(f)] = 0;
                                    coupleExc_[size_t(f)] = 0.0;
                                    hasExc_[size_t(f)] = false; }
            // 老化量：相对遮挡入口姿势偏离了多少（rad），拿不到就退回帧数。
            const double s0 = std::max(1e-6, cfg_.occStaleRad0);
            const double s1 = std::max(s0 + 1e-6, cfg_.occStaleRad1);
            const double byExc = (std::fabs(coupleExc_[size_t(f)]) - s0) / (s1 - s0);
            const double fMax  = std::max(1, cfg_.caseBMaxFrames);
            const double byFrm = (double(caseBFrames_[size_t(f)]) - fMax) / fMax;
            // 【帧数只在拿不到偏离量时才用】帧数是 staleness 的【代用品】，
            // 有直接测量就不该再用它：实测手静止时链式解永远优于回退，
            // 而帧数兜底会让静止握 6 秒以上也开始淡出，那正是要修的症状。
            const double aging = hasExc_[size_t(f)] ? byExc : std::max(byExc, byFrm);
            // 【淡出，不是硬切】夹到 [0,1]。
            // w=1 完全信链式解，w=0 完全交回网络预测，中间线性混合 ——
            // 交叉点附近两者精度本来就相当，混合不吃亏，却把断崖抹掉了。
            const double wChain = std::clamp(1.0 - aging, 0.0, 1.0);
            const bool caseB = !seen[size_t(mp)]
                            && wChain > 0.0
                            && hasPipHold_[size_t(f)] && hasPlane_[size_t(f)]
                            && pmLen_[size_t(f)] > 1e-3;
            // ---- 逐指链式续解诊断（见 SkeletonFrameResult 里的说明）----
            // 【无条件记，不放在 caseB 里】恰恰是 caseB 为假的那些帧最需要
            // 知道原因——四个守卫哪个没过。放进 if 里就只能看到成功的那些。
            out.chainHasPipHold[size_t(f)]  = hasPipHold_[size_t(f)];
            out.chainHasPlane[size_t(f)]    = hasPlane_[size_t(f)];
            out.chainPmLenMm[size_t(f)]     = pmLen_[size_t(f)];
            out.chainMdLenMm[size_t(f)]     = mdLen_[size_t(f)];
            out.chainWeight[size_t(f)]      = wChain;
            out.chainAging[size_t(f)]       = aging;
            out.chainCaseB[size_t(f)]       = caseB;
            out.chainUseAnchor[size_t(f)]   = useAnchor;
            out.chainCaseBFrames[size_t(f)] = caseBFrames_[size_t(f)];
            out.chainHasExc[size_t(f)]      = hasExc_[size_t(f)];

            const Vec3 P = out.markers[size_t(pp)].posWorld;
            // 近节骨轴 u0：有 anchor 就用 anchor->pp（更准），没有就退回
            // "上一帧的 pp->mp 方向"—— 后者是实测量，不依赖任何标定。
            //
            // 【沿用上一帧必须在腕部系里沿用】原来存的是【世界系】方向：
            //     prevU0_[f] = u0;            // 世界
            //     u0 = prevU0_[f];            // 下一帧原样取回
            // 遮挡期间手一转，P 跟着手走、方向却钉在世界里不动 —— 于是整根
            // 手指相对手掌越张越开。拇指最明显（它的 pp 是掌骨球，力臂最长），
            // 真机症状就是"两个球一遮就整个飞了外摆、跳变、不连续"。
            // 骨轴相对【手掌】才是姿势量，存进腕部系、每帧用当帧 wristR 取回，
            // 纯刚体运动下它恒等于上一帧，一点都不会漂。
            Vec3 u0{};
            if (useAnchor) {
                const Vec3& am = anchorsMm_[size_t(f)];
                const Vec3 A{out.wristR[0]*am[0] + out.wristR[1]*am[1] + out.wristR[2]*am[2] + out.wristT[0],
                             out.wristR[3]*am[0] + out.wristR[4]*am[1] + out.wristR[5]*am[2] + out.wristT[1],
                             out.wristR[6]*am[0] + out.wristR[7]*am[1] + out.wristR[8]*am[2] + out.wristT[2]};
                u0 = sub(P, A);
            } else if (seen[size_t(mp)]) {
                u0 = sub(out.markers[size_t(mp)].posWorld, P);   // 情形A：直接可测
            } else if (hasPrevU0_[size_t(f)]) {
                u0 = matVec(out.wristR, prevU0Local_[size_t(f)]);  // 情形B：腕部系里沿用
            } else continue;
            const double n0 = norm(u0);
            if (n0 < 1e-6) continue;
            u0 = mul(u0, 1.0/n0);
            // 当帧的近节骨轴，换算到腕部系 —— 后面的耦合推算全在这个系里做
            const Vec3 u0Loc = matVecT(out.wristR, u0);

            // ---- 遮挡入口的参照：进入情形B 的那一帧记下"近节在手掌里的朝向"
            // 和"当时量到的 PIP 角"。之后每一帧都拿【当帧实测的 u0Loc】跟这个
            // 固定参照比，得到一个【绝对】的相对转角 —— 不是逐帧累加，所以
            // 不存在开环积分的漂移。见下面情形B 里的推算。
            if (seen[size_t(mp)]) {                    // 中节可见 = 参照随时更新
                u0Entry_[size_t(f)] = u0Loc;
                pipEntry_[size_t(f)] = pipHold_[size_t(f)];
                hasEntry_[size_t(f)] = hasPipHold_[size_t(f)];
                // 无标定退化档的参照：近节球在【手掌系】里的位置，以及它在
                // 屈曲时的运动方向。后者是 planeLocal x 骨轴 —— 绕 planeLocal
                // 转动时，骨轴上一点的线速度方向正是这个叉积。
                if (hasPlane_[size_t(f)]) {
                    pEntryLocal_[size_t(f)] = matVecT(out.wristR, sub(P, out.wristT));
                    Vec3 fd = cross(planeLocal_[size_t(f)], u0Loc);
                    const double fn = norm(fd);
                    if (fn > 1e-6) {
                        flexDirEntry_[size_t(f)] = mul(fd, 1.0 / fn);
                        hasPEntry_[size_t(f)] = true;
                    } else hasPEntry_[size_t(f)] = false;
                } else hasPEntry_[size_t(f)] = false;
            }
            prevU0Local_[size_t(f)] = u0Loc; hasPrevU0_[size_t(f)] = true;

            // ================== 这道闸原来【不存在】 ==========================
            // caseB 在上面算了出来，然后全文件再没有第二处引用它 —— 没有
            // `if (!caseA && !caseB) continue;`。后果有三条，你报的问题里占三条：
            //
            //  ① 三颗球【全可见】时 caseA = seen[mp] && !seen[dp] = false，
            //     于是照样落进下面的 else 分支，把【实测的】中节和远节位置
            //     用 FK 摆出来的点覆写掉。这就是"连线的点不在实际观测点位置上"
            //     —— 而且它每一帧都在发生，不只是遮挡恢复之后。
            //     合成实测：全可见 200/200 帧都跑了续解，实测点被推开
            //     p90 = 8.4mm、最大 11.8mm。
            //
            //  ② caseB 的四个前置条件（角度学到过、平面学到过、骨长标定过、
            //     没超时）一个都没被强制。hasPlane_ 为假时 planeN_ 是【零向量】，
            //     下面拿它做 Rodrigues 就是绕零轴转：
            //         kx = cross(0,u0) = 0,  nv = 0  ->  u1 = u0*cos(a)
            //     方向退化成近节骨轴本身、长度还不是 1。手指被摆成一根直棍
            //     指着掌骨方向 —— 拇指的"整个飞了外摆、连线是直的"就是这个。
            //
            //  ③ 函数末尾那段"三点全可见时在线学习"读的是 out.markers，而它
            //     已经被 ① 覆写过了。于是平面、骨长、dip/pip 耦合比、拇指 IP
            //     轴全部在【拿自己摆出来的点当观测】学习，自己证明自己，
            //     一路慢慢漂走。
            if (!caseA && !caseB) continue;

            Vec3 M, u1;
            if (caseA) {
                M = out.markers[size_t(mp)].posWorld;
                u1 = sub(M, P);
                const double n1 = norm(u1);
                if (n1 < 1e-6) continue;
                u1 = mul(u1, 1.0/n1);
                // 【MCP 角只有拿得到掌骨方向时才存在】useAnchor 为假时
                // u0 退化成 pp->mp，而 u1 【就是同一个向量】，
                // acos(dot(u0,u1)) 【恒等于 0】。
                //
                // 写下去的后果不是"少学一点"，是【毒】：hasPipHold_ 被置真，
                // caseB 的四道守卫全过，然后拿 a = pipEntry(=0) 去做 Rodrigues
                // —— 手指被摆成一根直的。本文件自己测过这一档：
                // 食指 p90 网络回退 ~15mm、硬摆直 18.7mm，【比不做还差】。
                //
                // 所以拿不到就不写：hasPipHold_ 保持假 -> caseB 不触发 ->
                // 老实退回网络 pos 头。那是这一档里已知最好的选择。
                //
                // 【这条在改成逐指 anchorOk_ 之后才真正咬人】以前 anchorsValid_
                // 是个全局 bool、在这份录制里恰好为真，u0 一直是 anchor->pp，
                // 这里永远写的是个真角度（虽然有偏）。改成逐指之后五根里降级
                // 四根，这四根就全部走进上面那个"摆成直棍"的分支。
                if (useAnchor) {
                    pipHold_[size_t(f)] = std::acos(std::clamp(dot(u0, u1), -1.0, 1.0));
                    hasPipHold_[size_t(f)] = true;
                }
                pmLen_[size_t(f)] = n1;   // 近节球->中节球 的实测间距（跟 anchor 无关，照记）
            } else {
                // 情形 B：中节也得自己摆出来（条件在上面 caseB 里已经查过）
                // 【冻结角度，不外推】
                //
                // 原来这里是 `pipHold_ + dPip` —— 用近节骨轴的转角去推 PIP。
                // 那个被实测否掉过：近节转 1° 不代表 PIP 也转 1°，两个关节不是
                // 刚性联动，开环积几十帧就到 40°+（仿真：中位 43.8° vs 不做 22.9°）。
                //
                // 但"不外推"不等于"什么都不做"。角度冻住、【位置跟着可见的近端点走】
                // 是另一回事：关节角不漂（是上次真量到的），而整根手指跟着
                // 近节刚性平移旋转 —— 手一动它就跟着动，这比退回网络 pos 头
                // （每帧独立估计、位置乱跳）好得多。
                //
                // 这也是四指和拇指共用的逻辑：只要近端那个点可见 + 骨长标定过，
                // 就能把后面几节摆出来。
                // 【2026-08 改：冻结 -> 掌心相对耦合】
                //
                // 冻结的问题你说得很准："还有一个点没被遮挡，看它和手掌刚体的
                // 相对运动是可以推出内弯动作的"。近节球可见时，近节相对手掌的
                // 转角是【当帧实测量】，而握拳时 MCP 和 PIP 是同向联动的 ——
                // 冻结等于把这个免费的观测量扔掉，手指弯下去而模型不动。
                //
                // 【跟以前被否掉的那版的区别，这是关键】旧版 dPip 是
                //     每帧 acos(dot(u0, prevU0_)) 然后【逐帧累加】，而且在
                //     【世界系】里量 —— 手一转就产生假的屈曲增量，开环积分，
                //     几十帧漂到 40°+。实测中位 43.8° 比不做的 22.9° 还差一倍。
                //
                // 现在这版：
                //   · 在【腕部系】里量 —— 纯刚体运动产生的 dTheta 恒等于 0
                //   · 相对【遮挡入口那一帧的固定参照】量，不是逐帧累加 ——
                //     没有积分，就没有漂移；近节转回去，推算值精确回到入口值
                //   · 幅度钳在 occCoupleMaxRad 以内，坏参照不会把手指甩出去
                // 也就是说它是【闭环】的：每一帧都由当帧那颗可见球重新锚定。
                const Vec3 nrmB = matVec(out.wristR, planeLocal_[size_t(f)]);
                double a = pipHold_[size_t(f)];
                if (cfg_.occCouple && hasEntry_[size_t(f)]) {
                    // ---- 驱动量：近节相对手掌转了多少 ----
                    // 两条路，取决于有没有 anchor：
                    //
                    // ① anchorsValid_：u0 = anchor->pp 是【真正的近节骨轴】，
                    //    直接量它相对入口姿态的转角，最准。
                    //
                    // ② 没有 anchor：这时 u0 只是"上一帧沿用下来的方向"，
                    //    自己跟自己比恒等于 0，一点驱动量都拿不到 ——
                    //    而这正是自标定没走到 Ready 时的常态，也就是你说
                    //    "拇指两个球一遮就飞"最容易出现的场合。
                    //    但【近节球本身的位置是实测的】：它长在近节指骨上、
                    //    在 MCP 远侧，关节一屈它就在手掌系里划过一段弧。
                    //    把这段位移投影到屈曲方向上再除以半径就是转角。
                    //    半径用 pmLen_（实测的球间距）当代理 —— 它比真实的
                    //    MCP->近节球距离偏大，所以这条路是【偏保守】的，
                    //    宁可推不够也不推过头。而且它同样是纯实测量，
                    //    不引入任何标定依赖。
                    double dTheta = 0.0;
                    if (useAnchor) {
                        const Vec3& e = u0Entry_[size_t(f)];
                        const double cA = std::clamp(dot(u0Loc, e), -1.0, 1.0);
                        dTheta = std::acos(cA);
                        // 符号：planeLocal_ 的定义是"绕它把 u0 转 +pip 得到 u1"，
                        // 所以 +号 = 继续屈曲。
                        const Vec3 axL = cross(e, u0Loc);
                        if (dot(axL, planeLocal_[size_t(f)]) < 0.0) dTheta = -dTheta;
                    } else if (hasPEntry_[size_t(f)]) {
                        const Vec3 d = sub(matVecT(out.wristR, sub(P, out.wristT)),
                                           pEntryLocal_[size_t(f)]);
                        const double r = std::clamp(pmLen_[size_t(f)], 15.0, 60.0);
                        dTheta = dot(d, flexDirEntry_[size_t(f)]) / r;
                    }
                    const double lim = std::max(0.0, cfg_.occCoupleMaxRad);
                    const double addRad = std::clamp(cfg_.occCoupleGain * dTheta, -lim, lim);
                    a = std::clamp(pipEntry_[size_t(f)] + addRad, 0.0, 2.62);   // <=150°
                    out.chainCoupledRad[size_t(f)] = addRad;
                    // 给下一帧算老化用：相对入口姿势偏离了多少。
                    // 【用绝对偏离而不是逐帧增量】逐帧增量分不出静止和运动
                    // （实测静止 0.012~0.015 rad、持续弯 0.009~0.012 rad，
                    // 全被噪声淹没）；绝对偏离不累积噪声，静止时就停在 ±0.012。
                    coupleExc_[size_t(f)] = addRad;
                    hasExc_[size_t(f)] = true;    // 这一指有直接的 staleness 测量
                }
                const double c = std::cos(a), sn = std::sin(a);
                const Vec3 kx = cross(nrmB, u0);
                const double nv = dot(nrmB, u0);
                u1 = Vec3{u0[0]*c + kx[0]*sn + nrmB[0]*nv*(1.0-c),
                          u0[1]*c + kx[1]*sn + nrmB[1]*nv*(1.0-c),
                          u0[2]*c + kx[2]*sn + nrmB[2]*nv*(1.0-c)};
                M = add(P, mul(u1, pmLen_[size_t(f)]));
                // 【淡出混合】wChain<1 时按权重混回网络预测，避免断崖。
                // 混合的是【位置】：两个解都在同一坐标系里，且交叉点附近
                // 精度相当，线性插值不会插出一个比两端都差的点。
                if (wChain < 1.0) {
                    const Vec3 fb = out.markers[size_t(mp)].posWorld;  // 网络预测
                    M = add(mul(fb, 1.0 - wChain), mul(M, wChain));
                }
                out.markers[size_t(mp)].posWorld = M;
                ++out.chainContinued;
                chainFilledThisFrame_[size_t(mp)] = true;
            }

            // 手指平面的法向。近节和中节几乎共线时这个叉乘会退化 —— 那时
            // 弯曲角本来也接近 0，沿用上一帧的法向即可，不要硬解。
            Vec3 nrm{};
            // 【拇指走自己的轴，平面退化不该卡住它】MCP 伸直时 cross(u0,u1)
            // 必然退化，而那恰恰是拇指最常见的姿势（只弯指尖）。原来这里
            // 会 continue 掉，等于拇指遮挡时一个点都补不上。
            const bool thumbOwnAxis = (f == 0 && hasThumbIp_);
            // 【原来这里有个 sinGate】它给 caseA 那次 cross(u0,u1) 的写入把门。
            // 那次写入已经停掉（理由见下），门也就跟着没有对象了，一并删掉 ——
            // 留着一个没人读的 std::sin 只会让下一个人去猜它管的是哪一路。

            // ---- 旋转轴的选取：只读平面，不在这里学平面 --------------------
            // ============ 【caseA 原来在这里写 pipPlaneLocal_，已停掉】=========
            // 原来是：
            //     nrm = cross(u0, u1);
            //     if (|nrm| > sinGate) { pipPlaneLocal_ = matVecT(wristR, nrm); }
            //
            // 【它写进去的根本不是 PIP 平面】caseA 里
            //     u0 = anchor->pp（useAnchor 时）
            //     u1 = pp->mp
            // 所以 cross(u0,u1) 是【MCP 平面】。而它自己的注释白纸黑字写着
            // "这里(远节段) = cross(pp->mp, mp->dp)" —— 代码和注释说反了。
            // 那段注释当初正是为了修"两个关节的弯曲平面串用一个变量"才加的，
            // 结果是换了个变量名、把同一件事又做了一遍。
            //
            // 【而且它只在有 anchor 时才活】useAnchor 为假时 u0 == u1，
            // cross 恒为 0、过不了 sinGate。也就是说这条路【专门】在 anchor
            // 有效时，把 anchor 的出平面偏差（实测 17.8~59.7°）灌进远节的
            // 旋转轴。停掉它是纯粹的去偏：无 anchor 的场景一点都不受影响。
            //
            // 【停掉之后 PIP 平面从哪来】本函数末尾"三点全可见"那段的
            // cross(a1,a2) —— 那才是定义上的 PIP 平面，而且更新时机多得多
            // （caseA 实测中指只有 58/202 帧够条件，全可见帧远比它多）。
            //
            // 于是三个分支现在【统一成只读】，取轴的优先级也统一：
            //     PIP 平面  ->  MCP 平面  ->  解剖学近似  ->  放弃
            // 拇指仍然先看 PIP 平面、拿不到就用 u1 占位（下面会被 rotAx 覆盖）。
            if (thumbOwnAxis) {
                nrm = hasPipPlane_[size_t(f)] ? matVec(out.wristR, pipPlaneLocal_[size_t(f)])
                                              : u1;   // 下面会被 rotAx 覆盖
            } else {
                // 【必须检查学到没有】hasPlane_ 为假时那个成员是【零向量】，
                // 下面的 Rodrigues 绕零轴旋转，结果是任意方向。
                //
                // 真机症状：刚打开点云面板、手还没动过时，几根手指（食指最明显）
                // 的预测指尖朝一边弯折，不符合人体；点"重标"之后手动几帧、
                // 平面学到了，就正常了。
                if (hasPipPlane_[size_t(f)]) {
                    // 远节绕的是 PIP 关节，用 PIP 平面。
                    nrm = matVec(out.wristR, pipPlaneLocal_[size_t(f)]);
                } else if (hasPlane_[size_t(f)]) {
                    // PIP 平面还没学到时退回 MCP 平面。
                    // 【现在这两个是同一个量了】改完之后 planeLocal_ 也是从
                    // cross(pp->mp, mp->dp) 学的，只是滤波方式不同（累加器
                    // vs 0.9/0.1 低通）。所以这一退不再是"差 41°的将就"，
                    // 而是同一个平面的另一个估计。
                    nrm = matVec(out.wristR, planeLocal_[size_t(f)]);
                } else {
                    // 都没学到就退回【解剖学默认】：弯曲平面法向 ≈ 掌骨扇形的
                    // 侧向。手指屈伸在"包含手背法向"的平面内，所以它的法向
                    // 垂直于手背法向和骨轴。近似，但比零向量强得多。
                    const Vec3 dorsal{out.wristR[2], out.wristR[5], out.wristR[8]};
                    Vec3 g = cross(dorsal, u1);
                    const double gn = norm(g);
                    if (gn < 1e-6) continue;      // 骨轴与手背法向平行，退化，不猜
                    nrm = mul(g, 1.0 / gn);
                }
            }

            // PIP 角（可见部分量出来的）-> DIP 角
            // 【耦合比例是【自己学的】，不是拿常数】关联器这一层拿不到 fingerParam，
            // 而且逐人逐指的耦合差异不小。做法：两颗球都可见时顺手记下
            // dip/pip 的比值，遮挡时拿它推。没学到就退回 0.66（解剖学常用值）。
            double dipAng;
            Vec3 rotAx = nrm;
            // 【融合路径的产物】走网络方向融合时，远节骨轴直接由平面内基
            // 算出来，不再走下面的 Rodrigues —— 两者等价，但融合这条已经
            // 有了单位方向向量，再转一次没必要，还会引入数值误差。
            Vec3 fuseDir{};
            bool haveFuseDir = false;
            if (f == 0 && hasThumbIp_) {
                // 拇指：IP 不跟 MCP 耦合，用最后一次【量到的】IP 角；
                // 弯曲轴从局部系用当帧的 u1 重建，所以手怎么转它都跟着对。
                Vec3 F1, F2, F3;
                if (thumbLocalFrame(u1, out.wristR, F1, F2, F3)) {
                    const Vec3& L = thumbIpAxisLocal_;
                    Vec3 ax{F1[0]*L[0] + F2[0]*L[1] + F3[0]*L[2],
                            F1[1]*L[0] + F2[1]*L[1] + F3[1]*L[2],
                            F1[2]*L[0] + F2[2]*L[1] + F3[2]*L[2]};
                    const double na = norm(ax);
                    if (na > 1e-6) rotAx = mul(ax, 1.0 / na);
                }
                // 【连续跟随】用当帧实测的 MCP 角驱动 IP，不是冻住最后值。
                // 拇指掉头时 MCP 也跟着掉头（实测 ΔIP 与 ΔMCP 的符号一致率
                // ~70~100%），所以它自己会转回来，不像外推那样一路冲过去。
                // 两段录制共 17 段真实遮挡实测（对出口值的误差）：
                //     保持     中位 24.6~30.3°   最大 34.4~56.2°
                //     本模型   中位  5.4~ 8.7°   最大  9.7~37.0°
                // 【驱动量必须来自实测，不能来自自己摆出来的 u1】
                // 中节被补出来时，上面那段是用 pmLen_ 和平面把 u1 摆出来的 ——
                // 拿它算 MCP 角就是"自己证明自己"，跟情形B 被否掉的理由一样，
                // 开环几十帧就漂走。
                //
                // 中节实测时（caseA）u1 是真的，可以驱动；
                // 中节也是补的时候，退回"保持最后一次量到的 IP 角" ——
                // 保持不如跟随，但远好过用假驱动量把它推飞。
                const bool driveReal = seen[size_t(mp)];
                const double mcpNow = std::acos(std::clamp(dot(u0, u1), -1.0, 1.0));
                dipAng = (thumbTrk_.ready() && driveReal)
                             ? thumbTrk_.step(mcpNow)
                             : std::clamp(thumbIpAng_, -0.26, 1.40);
                out.thumbIpPredRad  = dipAng;
                // 本次遮挡至今，跟随器相对入口值累计偏移了多少。
                // 【必须报真值】它恒为 0 的话，面板上就永远看不出跟随到底有没有工作。
                out.thumbIpDriftRad = dipAng - thumbTrk_.entryAngle();
            } else {
                // ---- 四指远节：网络方向 + 平面约束 + 时序平滑 的融合 ----
                //
                // 【为什么不用耦合比推】原来是 dipAng = k * pipAng（k 自学的
                // dip/pip 比值）。它完全不看网络这一帧说了什么 —— 等于把一个
                // 有真实信息的输入扔掉，换成一条固定的经验关系。
                //
                // 而实测（用每指自己的弯曲平面量"出平面角"，即真正的侧弯）：
                //     指    真值(可见)   网络@遮挡   链式@遮挡
                //     食指     1.6°        2.4°       41.9°
                //     中指     7.2°       18.9°       44.8°
                // 网络的【方向】几乎和真值一样好，链式反而差一个数量级。
                // 但连续性正相反：遮挡点帧间跳变 P95，网络 15~21mm、链式 2.7~10.8mm。
                //
                // 也就是说两者错在【不同维度】：网络错在时间轴（抖），
                // 链式错在空间轴（出平面）。所以正确的做法不是加权平均，
                // 而是分解到正交分量后各取所长：
                //   · 方向  取网络的，但【投影到弯曲平面内】—— 出平面分量
                //           正是侧弯，投影掉之后数学上恒为 0
                //   · 相位  平面内的那个角做时序低通 —— 网络的抖动是平面【内】的，
                //           投影管不着，得靠这一层压
                //   · 长度  取学到的骨长 mdLen_ —— 这是链式唯一真正有信息的量
                //
                // 【平滑角度而不是位置】平滑位置会把骨长一起平滑掉，
                // 手指长度就会随运动伸缩。平滑角度则骨长恒定。
                //
                // 实测融合结果（alpha=0.3）：出平面角 0.0°（数学保证），
                // 跳变 P95 食指 4.62 / 中指 7.24 / 无名 8.56mm ——
                // 比网络好 3~4 倍，中指和无名甚至比链式还好；
                // 遮挡恢复时的交接误差 3.5~7.3mm，也优于原来的 9~17mm。
                //
                // 【网络值不可用时退回原来的耦合推算】netPos 没有、或者
                // 投影后长度退化（网络方向恰好垂直于平面）时，
                // 没有可用的方向信息，此时耦合比是最好的兜底。
                const double pipAng = std::acos(std::clamp(dot(u0, u1), -1.0, 1.0));
                const double k = (dipK_[size_t(f)] > 0.0) ? dipK_[size_t(f)] : 0.66;
                dipAng = std::clamp(k, 0.0, 1.2) * pipAng;

                // 平面内正交基：e1 = 中节骨轴在平面内的分量，e2 = nrm × e1。
                // 用当帧的 u1 建基，所以手怎么转它都跟着对，不需要额外的系变换。
                Vec3 e1 = sub(u1, mul(nrm, dot(u1, nrm)));
                const double e1n = norm(e1);
                const Vec3& netDp = out.netPos[size_t(dp)];
                const Vec3& netMp = out.netPos[size_t(mp)];
                Vec3 vN = sub(netDp, netMp);
                Vec3 vP = sub(vN, mul(nrm, dot(vN, nrm)));   // 去掉出平面分量
                const double vPn = norm(vP);
                if (e1n > 1e-6 && vPn > 1e-6 && cfg_.occUseNetDir) {
                    e1 = mul(e1, 1.0/e1n);
                    const Vec3 e2 = cross(nrm, e1);
                    vP = mul(vP, 1.0/vPn);
                    // 网络这一帧给出的"平面内相位角"
                    const double angNet = std::atan2(dot(vP, e2), dot(vP, e1));
                    // 时序低通。【在角度域做差，处理 ±π 绕回】直接对角度做
                    // 低通会在 π/-π 交界处产生一整圈的假跳变。
                    double target = angNet;
                    if (hasDipPhase_[size_t(f)]) {
                        const double prev = dipPhase_[size_t(f)];
                        const double d = std::atan2(std::sin(angNet - prev),
                                                    std::cos(angNet - prev));
                        target = prev + std::clamp(cfg_.occNetDirAlpha, 0.0, 1.0) * d;
                    }
                    dipPhase_[size_t(f)] = target;
                    hasDipPhase_[size_t(f)] = true;
                    dipAng = target;
                    // 【标记走了融合路径】下面用 e1/e2 直接摆点，不再走 Rodrigues
                    fuseDir = add(mul(e1, std::cos(target)), mul(e2, std::sin(target)));
                    haveFuseDir = true;
                }
                dipAng = std::clamp(k, 0.0, 1.2) * pipAng;
            }

            // 远节骨轴。融合路径已经直接给出单位方向；否则走 Rodrigues：
            // 中节骨轴绕 rotAx 转 dipAng。
            Vec3 u2;
            if (haveFuseDir) {
                u2 = fuseDir;
            } else {
                const double c = std::cos(dipAng), sn = std::sin(dipAng);
                const Vec3 kx = cross(rotAx, u1);
                const double nv = dot(rotAx, u1);
                u2 = Vec3{u1[0]*c + kx[0]*sn + rotAx[0]*nv*(1.0-c),
                          u1[1]*c + kx[1]*sn + rotAx[1]*nv*(1.0-c),
                          u1[2]*c + kx[2]*sn + rotAx[2]*nv*(1.0-c)};
            }

            // 球间距：优先用最近一次两颗都可见时的实测值（含贴球偏置），
            // 没有就退回标定骨长。实测值比名义骨长更贴近这只手的贴法。
            const double L = mdLen_[size_t(f)];
            if (!(L > 1e-3)) continue;      // 还没测到过这根的球间距，不猜
            // 远节同样淡出。【必须跟中节用同一个 wChain】否则两点各混各的，
            // 混合中间态的 mp-dp 间距会被拉开 —— 而 snapToBoneLength 在
            // process() 里跑在这之后，能把长度收回去，但方向已经被拆歪了。
            Vec3 D = add(M, mul(u2, L));
            if (wChain < 1.0) {
                const Vec3 fb = out.markers[size_t(dp)].posWorld;   // 网络预测
                D = add(mul(fb, 1.0 - wChain), mul(D, wChain));
            }
            out.markers[size_t(dp)].posWorld = D;
            ++out.chainContinued;
            chainFilledThisFrame_[size_t(dp)] = true;
        }
        // ---- 全可见时在线学习：球间距 和 dip/pip 耦合比 ----
        // 这两个量都用【这只手、这次贴法】的实测值，比任何常数或标定参数更贴。
        // 用一阶低通慢慢跟，避免个别坏帧带偏。
        for (int f = 0; f < 5; ++f) {
            const int pp = 5 + 3*f, mp = pp + 1, dp = pp + 2;
            if (!seen[size_t(pp)] || !seen[size_t(mp)] || !seen[size_t(dp)]) continue;
            const Vec3 P = out.markers[size_t(pp)].posWorld;
            const Vec3 M = out.markers[size_t(mp)].posWorld;
            const Vec3 D = out.markers[size_t(dp)].posWorld;
            // 【没有 anchor 时也要能立起弯曲平面】—— 必须在这里做，不能放在
            // 上面那个循环里：全可见帧会在 caseA/caseB 闸门处 continue，
            // 根本走不到。而"先全可见、再遮挡"正是实际的使用顺序，平面就该
            // 在可见期立好。
            //
            // 症状：刚打开点云面板、自标定还没出结果时，弯手指做遮挡预测，
            // 远端朝一侧歪；在全可见下点一次"重标"、等自标定收敛之后就正常了。
            //
            // 机理：没有 anchor 时 a0 退化成 M-P，而 a1 【就是同一个向量】，
            // cross(a0,a1) 恒等于 0 —— 平面永远学不到。于是遮挡时 caseA 走
            // continue（远节不重建）、caseB 被闸门挡掉，遮挡点整个落回网络的
            // 位置预测，那就是"歪"的来源。而 anchor 只在【构造关联器】时
            // 下发过一次，自标定中途算出来的根本没送进来，所以不点"重标"
            // 就一直是这个状态。
            //
            // 解剖轴是现成的、不需要任何标定：HandModel::fingerFK 里骨骼沿
            // 局部 X、屈曲是 rotY、外展是 rotZ，也就是腕部系
            //     Z = 手背法向,  Y = 屈曲轴,  X = 远端方向
            // （handBackMarkers 的 Z 只有 2~8mm，扁平方向，正好印证）
            // 所以 cross(Z, 骨轴) 就是该指的屈曲轴，绕它转正角 = 屈曲
            // （rotY 把 +X 转向 -Z，而 -Z 是掌侧，符号对得上）。
            //
            // 只当【种子】用：等 anchor 到位、弯折超过 planeLearnMinDeg 时，
            // 下面的 cross(a0,a1) 会把它 refine 掉。
            // 【拇指不种】拇指的屈曲平面相对手背旋前近 90°，cross(Z,骨轴)
            // 对它是【错的轴】。实测把种子用到拇指上，遮挡误差 p90 从
            // 5.9mm 劣化到 52.1mm —— 比不种坏一个数量级。
            // 拇指有自己的路（thumbOwnAxis / thumbIpAxisLocal_ / thumbTrk_），
            // 让它等 anchor 到位后从 cross(a0,a1) 老实学。
            //
            // 【没有 anchor 就不种】没 anchor 时 a0 退化成 M-P，而 a1 就是
            // 同一个向量，于是 pipHold_ = acos(dot(a0,a1)) 【恒等于 0】——
            // 链式续解会把手指摆成一根直的，比直接落回网络预测还差
            // （实测食指 p90：网络回退 ~15mm，硬种 18.7mm）。
            // 平面立起来了但关节角是假的，等于把一个坏解喂给下游。
            // 这个窗口的正确解法是【把 anchor 送进来】——见 SkeletonAssocWorker
            // 里 applyAutoCalib() 那段：anchor 原来只在构造关联器时下发过一次。
            if (f > 0 && anchorsValid_ && anchorOk_[size_t(f)] && !hasPlane_[size_t(f)]) {
                const Vec3 bone = sub(M, P);
                const Vec3 boneL = matVecT(out.wristR, bone);
                const Vec3 seed = cross(Vec3{0, 0, 1}, boneL);
                const double sn = norm(seed);
                // 骨轴太贴近手背法向就没有屈曲平面可言（|cross| = |bone|·sinθ）
                if (sn > 0.20 * norm(bone)) {
                    planeLocal_[size_t(f)] = mul(seed, 1.0 / sn);
                    hasPlane_[size_t(f)] = true;
                    planeSeeded_[size_t(f)] = true;   // 只是猜的，见下面的整体替换
                }
            }
            // 【骨长的 EMA 已删除】learnBoneLengths() 已经用 240 帧滑动窗口
            // 中位数在算同样这两个量了，而这里的一阶低通【写的是同一个成员】，
            // 每个全可见帧都把中位数的结果按 0.9/0.1 拉回瞬时值 —— 等于把
            // "EMA 换成中位数"那次改动又冲掉了大半。中位数对离群值免疫是
            // 它存在的全部理由，留着这行就没有意义了。
            // 【近节骨轴的起点：有 anchor 用 anchor，没有就用 pp->mp 近似】
            // 平面法向和 PIP 起始角都是【比值/方向】类的量，对起点的小偏差
            // 不敏感；而整个跳过的代价是遮挡时链式续解完全没得用。
            Vec3 a0;
            if (anchorsValid_ && anchorOk_[size_t(f)]) {
                const Vec3& am = anchorsMm_[size_t(f)];
                const Vec3 A{out.wristR[0]*am[0] + out.wristR[1]*am[1] + out.wristR[2]*am[2] + out.wristT[0],
                             out.wristR[3]*am[0] + out.wristR[4]*am[1] + out.wristR[5]*am[2] + out.wristT[1],
                             out.wristR[6]*am[0] + out.wristR[7]*am[1] + out.wristR[8]*am[2] + out.wristT[2]};
                a0 = sub(P, A);
            } else {
                a0 = sub(M, P);       // 退化：拿中节骨轴当近节方向的近似
            }
            Vec3 a1 = sub(M, P), a2 = sub(D, M);
            const double m0 = norm(a0), m1 = norm(a1), m2 = norm(a2);
            if (m0 < 1e-6 || m1 < 1e-6 || m2 < 1e-6) continue;
            a0 = mul(a0, 1.0/m0); a1 = mul(a1, 1.0/m1); a2 = mul(a2, 1.0/m2);
            // ---- 【已删除：这里原来还有第二个 planeLocal_ 写入点】----------
            // 原来这一块做的是：
            //     nr = cross(a0, a1);        // a0 = anchor->pp（useAnchor 时）
            //     planeLocal_ = 0.9*planeLocal_ + 0.1*normalize(nr);
            //
            // 【为什么必须删】它跟 learnJointPose 里那个累加器【触发条件完全
            // 相同】（都要求 pp/mp/dp 三点全见），所以在覆盖面上是纯冗余；
            // 而公式却还是有偏的那一条 —— u0 = anchor->pp 出平面 17.8~59.7°。
            //
            // 【它不是良性的】帧内顺序是 learnJointPose -> continueChainForOccluded：
            //     ① learnJointPose        planeLocal_ = 干净累加值
            //     ② 本函数上面的补点循环   用的就是这个干净值        ✓
            //     ③ 本块（原来在这）       planeLocal_ = 0.9*干净 + 0.1*anchor
            // 而【进入遮挡的那一刻】dp 不再可见，learnJointPose 从此不再覆写，
            // 于是整段遮挡用的就是 ③ 留下的那个被污染的值。也就是说这 10%
            // 不是"末尾残留"，恰恰是每一段遮挡的起点。
            //
            // 【它原来的存在理由已经不成立】注释写的是"全可见帧会在 caseA/caseB
            // 闸门处 continue，走不到上面那个循环，所以平面必须在这里学"。
            // 那条理由针对的是上面的补点循环 —— 而 learnJointPose 是独立的
            // 一个函数、在 process() 里单独调用，根本不经过那道闸。
            //
            // 唯一从这里丢掉的东西是它的解剖学定号逻辑（cross(Z,骨轴) 那段），
            // 已经原样搬进 learnJointPose 的累加器里了 —— 而且更该在那儿，
            // 因为符号错一次就是整根手指反向 180°。

            // ---- 顺便学 PIP 平面 ----
            // 【为什么放在这里】这个分支的前提就是三点全可见，a1/a2 都是
            // 实测骨轴，正是学 PIP 平面最可靠的时机。原来 PIP 平面只在
            // continueChainForOccluded 的情形A（中节可见、远节遮）里才更新，
            // 那个时机罕见得多 —— 实测中指只有 58/202 帧够条件。
            // 学不到就只能退回 MCP 平面，而两者差 41°，正是侧弯的来源。
            {
                Vec3 nr2 = cross(a1, a2);
                const double n2 = norm(nr2);
                const double gate2 = std::sin(std::clamp(cfg_.planeLearnMinDeg, 0.0, 60.0)
                                              * 3.14159265358979323846 / 180.0);
                if (n2 > gate2) {
                    nr2 = mul(nr2, 1.0/n2);
                    Vec3 nl2 = matVecT(out.wristR, nr2);
                    // 半球对齐：跟 MCP 平面同侧。两个关节的弯曲方向是一致的
                    // （手指往同一边弯），所以拿已经定好号的 MCP 平面当锚，
                    // 比自己跟自己的历史对齐更不容易被一次坏值带跑。
                    if (hasPlane_[size_t(f)] && dot(nl2, planeLocal_[size_t(f)]) < 0.0)
                        nl2 = mul(nl2, -1.0);
                    if (hasPipPlane_[size_t(f)]) {
                        if (dot(nl2, pipPlaneLocal_[size_t(f)]) < 0.0) nl2 = mul(nl2, -1.0);
                        Vec3 mix = add(mul(pipPlaneLocal_[size_t(f)], 0.9), mul(nl2, 0.1));
                        const double mn = norm(mix);
                        if (mn > 1e-6) pipPlaneLocal_[size_t(f)] = mul(mix, 1.0/mn);
                    } else {
                        pipPlaneLocal_[size_t(f)] = nl2;
                    }
                    hasPipPlane_[size_t(f)] = true;
                }
            }

            const double pipA = std::acos(std::clamp(dot(a0, a1), -1.0, 1.0));
            const double dipA = std::acos(std::clamp(dot(a1, a2), -1.0, 1.0));
            // PIP 起始值同理：三点全可见时就该记下来，别等情形A。
            // 【同 caseA 那处：没有 anchor 就不写】这里 a0 在无 anchor 时退化成
            // M-P，跟 a1 是同一个向量，pipA 恒等于 0。写下去等于给 caseB 递一个
            // "手指是直的"的假起始角，而 caseB 的守卫只查 hasPipHold_ 真不真、
            // 不查它有没有意义。本文件上面那段实测（网络 15mm vs 硬摆直 18.7mm）
            // 说的就是这个分支。
            //
            // 【这是三个 pipHold_ 写入点里的第三个】另外两个是 learnJointPose
            // （本来就有 haveMcpAngle 守卫）和 caseA。三处必须用同一个判据，
            // 漏一处就等于没加 —— 它们写的是同一个成员。
            if (anchorsValid_ && anchorOk_[size_t(f)]) {
                pipHold_[size_t(f)] = pipA;
                hasPipHold_[size_t(f)] = true;
            }
            // 【只在弯得够明显时学】pip 接近 0 时比值是 0/0，学出来全是噪声。
            if (pipA > 0.35) {
                const double kk = std::clamp(dipA / pipA, 0.0, 1.2);
                dipK_[size_t(f)] = (dipK_[size_t(f)] > 0.0)
                                 ? dipK_[size_t(f)] * 0.95 + kk * 0.05 : kk;
            }

            // ---- 拇指专用：直接学 IP 角本身 + 它的弯曲轴 ----
            //
            // 【为什么拇指不能用 dipK 耦合】四指的 DIP 和 PIP 通过指深屈肌腱和
            // 斜支持带真的联动（≈2/3），所以 dip = k*pip 成立。拇指的 IP 由拇长
            // 屈肌【单独】驱动 —— 掌指关节完全伸直时照样能只弯指尖（这是标准的
            // 拇指 IP 独立性测试）。把它当第五根手指的后果，真机实测到过：
            // MCP 直的时候 pipA≈0 -> dipAng = 0.66*0 ≈ 0 -> 整根拇指被摆成直棍，
            // M3DQ 面板上「拇 近节」和「拇 远节」的相对四元数双双 ≈ 单位。
            //
            // 【弯曲轴为什么要存局部系】世界系的轴每帧都在变（手在动）。而 IP 的
            // 解剖屈曲轴相对【近节骨自身】是固定的。所以把它换算到以 u1 为 X 轴、
            // 腕背法向定第二轴的局部系里存起来，遮挡时再用当帧的 u1 重建出来。
            // 这样拇指跟着手转、跟着 MCP 动，指尖始终朝正确的方向弯。
            // 摆动积分预测器的在线学习：ω = 近节骨轴在腕部系里的因果角速度
            // 指尖可见 -> 把实测的 IP/MCP 同步给跟随器，作为下次遮挡的起点
            if (f == 0) thumbTrk_.sync(dipA, pipA);
            if (f == 0 && dipA > 0.20) {
                const Vec3 ax = cross(a1, a2);
                const double na = norm(ax);
                if (na > 1e-3) {
                    Vec3 F1, F2, F3;
                    if (thumbLocalFrame(a1, out.wristR, F1, F2, F3)) {
                        const Vec3 u = mul(ax, 1.0 / na);
                        const Vec3 loc{dot(u, F1), dot(u, F2), dot(u, F3)};
                        // 一阶低通，跟 mdLen_/dipK_ 同样的理由：别被个别坏帧带偏
                        if (hasThumbIp_) {
                            thumbIpAng_ = thumbIpAng_ * 0.9 + dipA * 0.1;
                            for (int c = 0; c < 3; ++c)
                                thumbIpAxisLocal_[size_t(c)] =
                                    thumbIpAxisLocal_[size_t(c)] * 0.9 + loc[size_t(c)] * 0.1;
                        } else {
                            thumbIpAng_ = dipA;
                            thumbIpAxisLocal_ = loc;
                            hasThumbIp_ = true;
                        }
                    }
                }
            }
        }
    }
