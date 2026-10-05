// ===========================================================================
// Hm20Assoc_geom.ipp —— Hm20SkeletonAssociator 类体分片：骨长学习 / 骨长回正 / 预测点平滑 / anchor 自检
//
// 都是"拿几何约束修一修"的后处理，彼此独立，改一个不影响另一个。
//
// 【本文件被 #include 在 class 体内部】内容即类内定义（隐式 inline）。
// 不要加 include guard、不要加 namespace。开头的 access 说明符是显式写死的，
// 不依赖上一个分片以什么结尾——调换 include 顺序不会改变可见性。
// 独立语法检查：见 tests/test_assoc_fragments.cpp
// ===========================================================================
private:

    // 腕部位姿缓存：手背可见点 <3 的帧沿用它，而不是去做全 20 点 Kabsch
    std::array<Vec3, 5> anchorsMm_{};
    bool anchorsValid_ = false;
    bool anchorsSet_ = false;
    bool anchorsChecked_ = false;
    // verifyAnchors 上次判据的现场（见 SkeletonFrameResult 里的说明）。
    // -1 = 还从没判过。
    int anchorsVerifyNSeen_ = -1;
    int anchorsVerifyNOk_ = -1;
    // ---- 【新】逐指 anchor 结论 + 多帧投票 -----------------------------------
    // 原来只有一个全局 bool，判据是 nOk >= nSeen - 1，注释写的是
    // "允许一根指有遮挡/野点"。但【遮挡是随机的、坏 anchor 是恒定的】——
    // 这条容忍度把两件完全不同的事当成了同一件。真机录制正好踩中：
    // nSeen=4 / nOk=3 恰好卡在门限上通过，而没过的那一根（小指）
    // |anchor->pp| = 73.7mm，比一节近节骨长一倍；中指的 anchor->pp 与实测
    // pp->mp 夹角中位 78.9°、p90 93.8°，方向近乎垂直甚至翻到反侧。
    // 于是这两根手指整条链拿一个方向偏 65~79° 的 u0 去摆遮挡点。
    //
    // 改成【逐指结论】：坏的那根退回 pp->mp（纯实测量、不依赖任何标定），
    // 好的那几根照用 anchor。全局 anchorsValid_ 保留原语义（有任何一根可用），
    // 这样 thumb / segrot 那两处的现有判断不用跟着改。
    std::array<bool, 5> anchorOk_{};
    // 一帧定终身太脆：判据里的量本身带噪声，而这个锁一旦上就永不重试。
    // 改成多帧投票，攒够样本再锁。
    std::array<int, 5> anchorVoteOk_{};
    std::array<int, 5> anchorVoteN_{};
    static constexpr int kAnchorVoteMin = 20;    // 每指至少这么多帧才下结论
    // 【投票必须有个截止日】"等每根参评的手指都攒够 20 帧"听起来很稳，但它
    // 有个活锁：某根手指的 pp 在开头被看到过一两次、之后一直被遮，它的票数
    // 就永远停在 1~2，永远不满 20，于是 verifyAnchors 每帧都在第一道循环
    // 里 return，anchorsChecked_ 【永不置位】。
    // 失效方向是安全的（anchorOk_ 全假 -> 全退回 pp->mp），但代价是
    // anchorsVerifyNSeen_/NOk_ 这两个现场也永远写不出来 —— 而排查时第一个
    // 要看的就是它们。所以给一个帧预算兜底：到点就按现有证据定案，
    // 票数不够的那几根判为不可用（保守方向）。
    // 240 帧 @60fps = 4 秒，比任何一次正常的手部遮挡都长。
    static constexpr int kAnchorVoteMaxFrames = 240;
    int anchorVoteFrames_ = 0;
    static constexpr double kAnchorMinCos = 0.2588;   // cos(75°)
    // 【出平面判据】anchor->pp 相对实测弯曲平面的出平面角上限，sin(25°)。
    // 这一条比长度/夹角都关键：cross(anchor->pp, pp->mp) 对 u0 的出平面分量
    // 有 1/sin(MCP角) 的放大，真机 f1 出平面 40.1° -> 平面歪 53.7°。
    // 判在 learnJointPose 里（那里才有实测平面），阈值放这里跟其余 anchor
    // 常量在一处，改的时候不会漏。
    static constexpr double kAnchorMaxOutOfPlaneSin = 0.4226;   // sin(25°)
    static constexpr int    kAnchorPlaneVoteMin = 60;
    mutable std::array<double, 5> pmLen_{};       // 近节球->中节球 实测间距
    mutable std::array<double, 5> dipK_{};      // 在线学到的 dip/pip 耦合比
    mutable std::array<double, 5> mdLen_{};     // 中节球->远节球的实测间距
    // 拇指 IP：角度 + 弯曲轴（存在近节骨自身的局部系里）。见 thumbLocalFrame()。
    // 预测点的平滑状态。可见时跟着实测值走（等于随时复位），
    // 转遮挡的那一帧就能无缝接上，不会跳。
    mutable std::array<Vec3, kNumMarkers> smoothPos_{};
    mutable std::array<bool, kNumMarkers> smoothHas_{};
    // 恢复补偿：记住"预测位置 - 实测位置"，按帧衰减地加回去，
    // 把一次性跳变摊开。见 smoothPredicted 的说明。
    // 骨长的滑动窗口样本。用中位数而不是 EMA —— 见 learnBoneLengths 的说明。
    mutable std::array<std::deque<double>, 5> pmBuf_{}, mdBuf_{};
    mutable std::array<Vec3, kNumMarkers> enterOff_{};   // 转遮挡台阶补偿（见 smoothPredicted）
    mutable std::array<bool, kNumMarkers> seen0_{};   // 上一帧的可见状态
    mutable ThumbIpTracker thumbTrk_;
    mutable std::array<bool, kNumMarkers> snappedThisFrame_{};
public:
    // 骨长回正也要能单独驱动 —— 它跟 smoothPredicted 是两步，
    // 测试只驱动后者的话，量到的骨长是没回正过的（我第一版就漏了）。
    // pmLen_/mdLen_ 由 learnBoneLengths 在可见帧上学，这里一并驱动。
    double TestHook_pmLen(int f) const { return pmLen_[size_t(f)]; }
    double TestHook_mdLen(int f) const { return mdLen_[size_t(f)]; }
    // 【顺序跟 process() 里保持一致：平滑在前、回正在后】
    // 复刻版和真版走偏过一次（第一版顺序放错），那种情况下测试全绿而真机没变。
    void TestHook_snapToBoneLength(SkeletonFrameResult& r,
                                   const std::array<bool, kNumMarkers>& seen,
                                   TestHook_ContState& st) {
        (void)st;
        learnBoneLengths(r, seen);
        snapToBoneLength(r, seen);
    }

    void TestHook_smoothPredicted(SkeletonFrameResult& r,
                                  const std::array<bool, kNumMarkers>& seen,
                                  TestHook_ContState& st) {
        smoothPos_ = st.s; enterOff_ = st.off;
        smoothHas_ = st.has; seen0_ = st.seen0;
        smoothPredicted(r, seen);
        snapToBoneLength(r, seen);      // 回正拿最终话语权，见 process() 里的说明
        st.s = smoothPos_; st.off = enterOff_;
        st.has = smoothHas_; st.seen0 = seen0_;
    }
private:

    void learnBoneLengths(const SkeletonFrameResult& out,
                          const std::array<bool, kNumMarkers>& seen) {
        using namespace detail;
        // 【用中位数不用 EMA】骨长是常量，而实测值本身在飘：
        // 真机三点全可见时的实测骨长离散度（p5~p95 / 中位）——
        //     指0  5.2%   指1 38.4%   指2 23.1%   指3 11.1%   指4 17.5%
        // 食指飘到 38%。EMA 会【跟着这些噪声走】，学出来的"标定值"本身不稳，
        // 拿它去回正等于把噪声搬到遮挡点上。第一版我用的就是 EMA，
        // 实测回正后骨长离散度只从 49.3% 降到 39.1%，远没到位。
        //
        // 中位数对离群值免疫：一段里有 30% 的帧测歪了也不影响结果。
        // 窗口 240 帧（约 10 秒）足够覆盖手型变化，又不会记住太久以前的。
        for (int f = 0; f < 5; ++f) {
            const int pp = 5 + 3*f, mp = pp + 1, dp = pp + 2;
            auto push = [&](std::deque<double>& buf, double& outLen, int a, int b) {
                if (!seen[size_t(a)] || !seen[size_t(b)]) return;
                const double n = norm(sub(out.markers[size_t(b)].posWorld,
                                          out.markers[size_t(a)].posWorld));
                if (!(n > 1e-3 && n < 120.0)) return;
                buf.push_back(n);
                while (buf.size() > 240) buf.pop_front();
                if (buf.size() >= 12) {
                    std::vector<double> v(buf.begin(), buf.end());
                    std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
                    outLen = v[v.size()/2];
                } else if (outLen <= 0.0) {
                    outLen = n;                 // 样本还不够，先用当前值顶上
                }
            };
            push(pmBuf_[size_t(f)], pmLen_[size_t(f)], pp, mp);
            push(mdBuf_[size_t(f)], mdLen_[size_t(f)], mp, dp);
        }
    }

    void snapToBoneLength(SkeletonFrameResult& out, const std::array<bool, kNumMarkers>& seen) {
        using namespace detail;
        for (int f = 0; f < 5; ++f) {
            const int pp = 5 + 3*f, mp = pp + 1, dp = pp + 2;
            // 【沿链逐级往下】父点只要"可信"就能当基准 —— 可信 = 实测，
            // 或者【已经被这一步回正过】。
            //
            // 只认实测父点的话，链条第二级永远得不到约束：实测中 mp 和 dp
            // 常常一起被遮挡，dp 的父 mp 也是补的。实测代价：mp-dp 在
            // 16.50~18.61 之间飘（真值 18.00），而回正后能收到 ±0.2mm。
            //
            // 沿链的前提是【先回正父、再回正子】—— 下面 fix 的调用顺序保证了这点。
            std::array<bool, kNumMarkers> trusted{};
            for (int m = 0; m < kNumMarkers; ++m) trusted[size_t(m)] = seen[size_t(m)];
            auto fix = [&](int parent, int child, double L) {
                if (L <= 1e-3) return;                       // 还没标定出这一节
                if (!trusted[size_t(parent)] || seen[size_t(child)]) return;
                const Vec3 P = out.markers[size_t(parent)].posWorld;
                Vec3 d = sub(out.markers[size_t(child)].posWorld, P);
                const double n = norm(d);
                if (n < 1e-6) return;                        // 重合，方向无意义
                const double s = L / n;
                out.markers[size_t(child)].posWorld =
                    Vec3{P[0] + d[0]*s, P[1] + d[1]*s, P[2] + d[2]*s};
                trusted[size_t(child)] = true;      // 回正过了，可以当下一级的基准
                ++out.boneLenSnapped;
                snappedThisFrame_[size_t(child)] = true;
            };
            // pp->mp 用 pmLen_，mp->dp 用 mdLen_（都在三点全可见时学）
            fix(pp, mp, pmLen_[size_t(f)]);
            fix(mp, dp, mdLen_[size_t(f)]);
        }
    }

    // -------------------------------------------------------------------------
    // 预测点的时域平滑。
    //
    // 【只平滑补出来的点】实测点是三角化来的，平滑只会加延迟。
    // 【从可见转遮挡的那一帧要接上】不接的话位置会跳一下 —— 因为来源换了：
    // 上一帧是三角化的真值，这一帧是网络估计，两者本来就有偏差。
    // 转换帧用上一帧的【实测位置】初始化滤波器，后面才逐步跟到估计值上。
    //
    // 【为什么是低通不是外推】代码里记着一次教训（continueChainForOccluded
    // 情形B）：开环外推几十帧就漂到 40°+。低通只滞后不发散。
    // INT8 高速模式下网络 pos 的帧间抖动放大约 10 倍（二阶差分 0.20 -> 1.94mm），
    // 这一步正是为它准备的。
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // 遮挡点的连续性处理。两个方向都要管，而且【恢复方向更严重】。
    //
    // 真机实测（695 帧、8 个点、共 45 段遮挡）：
    //     正常帧位移   0.80mm
    //     转遮挡跳变   4.61mm  (5.8x)   最大 21.0mm
    //     恢复跳变     7.14mm  (8.9x)   最大 39.3mm   <- 更糟
    //
    // 【恢复跳变不是 bug，是"误差一次性还清"】查过成因：遮挡 42~90 帧期间
    // 预测累积了 6~14mm 误差，恢复那一瞬间跳回真值。占手实际移动量的 14~37%。
    // 光平滑预测点解决不了这个 —— 误差本来就在，问题是它在【一帧内】还清。
    //
    // 所以两件事分开做：
    //   ① 转遮挡：滤波器用最后的实测值起步，来源切换不产生台阶
    //   ② 恢复  ：记下"预测位置 - 实测位置"这个偏移，之后【按帧衰减】地
    //             加回去，把一次性跳变摊到十几帧上。位置最终收敛到实测值，
    //             不引入稳态误差。
    //
    // 【为什么不是简单低通实测值】那会让实测点也带上滞后 —— 实测是亚毫米的
    // 三角化结果，本来就准，压它是净损失。这里只补偿【偏移】，实测值本身
    // 一帧都不延迟。
    // -------------------------------------------------------------------------
    void smoothPredicted(SkeletonFrameResult& out, const std::array<bool, kNumMarkers>& seen) {
        using namespace detail;
        const double decay = std::clamp(cfg_.recoverBlendDecay, 0.0, 0.99);
        for (int m = 0; m < kNumMarkers; ++m) {
            Vec3& s = smoothPos_[size_t(m)];
            Vec3& off = enterOff_[size_t(m)];
            const Vec3 raw = out.markers[size_t(m)].posWorld;

            if (seen[size_t(m)]) {
                // ---- 实测帧 ----
                // 【实测点一定画在实测位置上】
                // 三角化出来的位置是亚毫米真值，把它推开去换视觉平滑是净损失，
                // 而且画面上一眼能看出不对（"连的点不在点上"）。
                out.markers[size_t(m)].posWorld = raw;
                s = raw;                                  // 下次转遮挡从实测值起步
                off = Vec3{};                             // 补偿只在遮挡期存在
                smoothHas_[size_t(m)] = true;
            } else {
                // ---- 预测帧 ----
                // 【转遮挡的台阶：记成偏移、按帧衰减，而不是靠低通拖】
                //
                // 来源换了就一定有台阶：上一帧是三角化真值，这一帧是 FK/网络
                // 估计，两者本来就差几毫米。原来靠 predictSmoothAlpha 从实测值
                // 慢慢爬过去 —— 但那等于给【整段遮挡】都加上滞后，
                // 而滞后在遮挡期手动得快时是要命的（扫描表：alpha 0.8->0.2
                // 遮挡误差中位 0.95 -> 4.87mm）。
                //
                // 拆成两件互不干扰的事：
                //   · alpha 只管压预测本身的噪声，不再兼职"接台阶"
                //   · 台阶单独记成 off，按 recoverBlendDecay 衰减，几帧内归零
                // 入口帧 off = 上一帧实测 - 本帧预测，于是输出恰好等于上一帧
                // 实测位置，台阶【精确为 0】；之后 off 自己退场，预测值不被延迟。
                //
                // 只作用于预测点：它们没有对应的点云点，画在哪都不会"不在点上"。
                //
                // 【原来的 recoverOff_ 是死的】它在【恢复】那一刻记偏移，可那之后
                // 点已经是实测的了，根本没有预测点可补 —— 写进去、钳幅、衰减，
                // 然后从没被加到任何输出上。方向反了，补的时机也反了。
                if (!smoothHas_[size_t(m)]) {             // 从没见过，无起点
                    s = raw; smoothHas_[size_t(m)] = true;
                } else if (seen0_[size_t(m)]) {
                    // 转遮挡的那一帧
                    off = sub(s, raw);
                    const double n = norm(off);
                    // 钳幅：预测彻底跑飞时（标签换了点之类）别把台阶原样背上，
                    // 那会把点拖到一个跟手无关的位置上待十几帧。
                    if (n > cfg_.recoverMaxOffsetMm)
                        off = mul(off, cfg_.recoverMaxOffsetMm / n);
                    s = raw;                              // 滤波器直接接到预测上，不拖
                } else {
                    const double a = cfg_.predictSmoothAlpha;
                    s = Vec3{s[0] + a*(raw[0]-s[0]),
                             s[1] + a*(raw[1]-s[1]),
                             s[2] + a*(raw[2]-s[2])};
                    off = mul(off, decay);
                    if (norm(off) < 0.05) off = Vec3{};
                }
                out.markers[size_t(m)].posWorld = add(s, off);
            }
            seen0_[size_t(m)] = seen[size_t(m)];
        }
    }

    // 拿【本帧真实观测】校验 anchor 跟手背模板是不是同一个坐标系。
    //
    // 判据：anchor 到该指近节球的距离必须是【一节近节骨】的量级(15~60mm)，
    // 且 anchor 必须比近节球更靠近手背中心。差一整个坐标系原点时，这两条
    // 必定同时崩掉 —— 真机实测群体 anchor 配用户模板时，"pp球-anchor" 与
    // 真实骨方向的夹角是 111°~157°，五根手指全反。
    void verifyAnchors(const SkeletonFrameResult& out) {
        if (!anchorsSet_ || anchorsChecked_ || !out.wristPoseValid) return;
        Vec3 c{0, 0, 0};
        for (int i = 0; i < 5; ++i) c = detail::add(c, tmplMm_[size_t(i)]);
        c = detail::mul(c, 0.2);
        // 世界 -> 腕部系（wristR 按行存，转置就是 R^T * v）
        auto toWrist = [&out](const Vec3& p) {
            return detail::matVecT(out.wristR, detail::sub(p, out.wristT));
        };
        int nOk = 0, nSeen = 0;
        for (int f = 1; f < 5; ++f) {          // 拇指 CMC 不参与判据，它本来就不走 anchor
            const int pp = 5 + f * 3, mp = pp + 1;
            if (!out.markers[size_t(pp)].observed) continue;
            ++nSeen;
            const Vec3 loc  = toWrist(out.markers[size_t(pp)].posWorld);
            const double bone = detail::norm(detail::sub(loc, anchorsMm_[size_t(f)]));
            const double dA = detail::norm(detail::sub(anchorsMm_[size_t(f)], c));
            const double dP = detail::norm(detail::sub(loc, c));
            bool ok = (bone >= 15.0 && bone <= 60.0 && dA < dP);
            // 【第三条判据：方向】anchor->pp 和 pp->mp 是同一根手指相邻的两段，
            // 夹角物理上进不了 90°。只查长度不查方向的话，一个"长度碰巧落在
            // 15~60mm 里、方向却偏 79°"的 anchor 照样过——而 u0 = anchor->pp
            // 正是链式续解摆点的【方向】依据，长度对方向错，点就往斜里飞。
            if (ok && out.markers[size_t(mp)].observed) {
                const Vec3 a = detail::sub(loc, anchorsMm_[size_t(f)]);
                const Vec3 b = detail::sub(toWrist(out.markers[size_t(mp)].posWorld), loc);
                const double na = detail::norm(a), nb = detail::norm(b);
                if (na < 1e-6 || nb < 1e-6 ||
                    detail::dot(a, b) / (na * nb) < kAnchorMinCos) ok = false;
            }
            ++anchorVoteN_[size_t(f)];
            if (ok) { ++anchorVoteOk_[size_t(f)]; ++nOk; }
        }
        if (nSeen < 3) return;                 // 观测不够，下一帧再判
        ++anchorVoteFrames_;

        // ---- 攒够样本才锁，但有截止日 ----
        // 【不再一帧定终身】判据里的量本身带噪声（遮挡、野点、腕部位姿抖动），
        // 而这个锁一旦上就永不重试。要求每根参评的手指都攒到 kAnchorVoteMin 帧。
        // 【超时就按现有证据定案】理由见 kAnchorVoteMaxFrames。
        const bool timeUp = (anchorVoteFrames_ >= kAnchorVoteMaxFrames);
        if (!timeUp) {
            for (int f = 1; f < 5; ++f)
                if (anchorVoteN_[size_t(f)] > 0 && anchorVoteN_[size_t(f)] < kAnchorVoteMin) return;
        }

        int nGood = 0;
        for (int f = 1; f < 5; ++f) {
            // 票数不够的一律判不可用。【保守方向是对的】退回 pp->mp 用的是
            // 纯实测量，代价有上限；而放行一个没验过的 anchor，代价是整条链
            // 绕着一根偏 19~78° 的轴摆点 —— 那正是这次要修的东西。
            if (anchorVoteN_[size_t(f)] < kAnchorVoteMin) { anchorOk_[size_t(f)] = false; continue; }
            // 八成以上的帧通过才算这根 anchor 可用。恒定坏的 anchor 通过率接近 0，
            // 偶发遮挡/野点造成的失败远到不了 20%，两者分得很开。
            anchorOk_[size_t(f)] =
                (double(anchorVoteOk_[size_t(f)]) >= 0.8 * double(anchorVoteN_[size_t(f)]));
            if (anchorOk_[size_t(f)]) ++nGood;
        }
        // 拇指不参与判据，跟着四指的整体结论走（它本来也另有 thumbLocalFrame 兜底）
        anchorOk_[0] = (nGood >= 2);
        anchorsValid_ = (nGood >= 1);          // 有任何一根可用，全局就算"在用"
        anchorsChecked_ = true;
        // 【记下判据的两个计数】anchorsValid_ 为假时，光看这个 bool 分不出
        // 是"观测不够一直没判"还是"判了但没通过"，更分不出差几根。
        // 判完就永不重试，所以当时的现场必须留下来。
        anchorsVerifyNSeen_ = nSeen;
        anchorsVerifyNOk_ = nOk;
    }

    static bool anchorsPlausibleUnused(const std::array<Vec3, 5>& a,
                                       const std::array<Vec3, 20>& tmpl, bool hasTmpl) {
        if (!hasTmpl) return false;          // 没有模板就无从判断，宁可不用
        Vec3 c{0, 0, 0};
        for (int i = 0; i < 5; ++i) c = detail::add(c, tmpl[size_t(i)]);
        c = detail::mul(c, 0.2);
        for (int f = 0; f < 5; ++f) {
            const Vec3 pp = tmpl[size_t(5 + f * 3)];          // 该指近节球（模板中立位）
            const double dAnchor = detail::norm(detail::sub(a[size_t(f)], c));
            const double dPp     = detail::norm(detail::sub(pp, c));
            if (dPp < 1.0) return false;                     // 模板本身没有手指点
            if (dAnchor >= dPp) return false;                // anchor 比近节球还远 -> 系错了
            if (dAnchor < 10.0 || dAnchor > 100.0) return false;
            // 再查一次距离：anchor 到近节球应当是一节近节骨的长度量级
            const double bone = detail::norm(detail::sub(pp, a[size_t(f)]));
            if (bone < 12.0 || bone > 70.0) return false;
        }
        return true;
    }
