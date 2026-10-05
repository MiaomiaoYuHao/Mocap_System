// ===========================================================================
// Hm20Assoc_segrot.ipp —— Hm20SkeletonAssociator 类体分片：骨骼朝向：分段四元数 / swing 构造 / 半球对齐
//
// 从点位置算每节骨的朝向。下游的关节角就是从这里的四元数取出来的，
// 所以"四元数是对的但角度不对"这类问题的上半截在这个文件里。
//
// 【本文件被 #include 在 class 体内部】内容即类内定义（隐式 inline）。
// 不要加 include guard、不要加 namespace。开头的 access 说明符是显式写死的，
// 不依赖上一个分片以什么结尾——调换 include 顺序不会改变可见性。
// 独立语法检查：见 tests/test_assoc_fragments.cpp
// ===========================================================================
private:
    std::array<Vec3, kNumSegments> prevSegY_{};   // 各段上一帧的 y 轴，退化姿态下保持连续
    std::array<Vec3, kNumMarkers> tmplMm_{};   // Kabsch 用，毫米
    bool tmplMmValid_ = false;

    // 四元数半球连续性用的上一帧值
    std::array<Quat, kNumSegments> prevSegQuat_{};
    Quat prevWristQuat_{1, 0, 0, 0};
    bool hasPrevQuat_ = false;
public:
    // 单元测试钩子：让离线验证程序能直接驱动 computeSegmentQuats
    static void TestHook_computeSegmentQuats(SkeletonFrameResult& r,
                                            std::array<Vec3, kNumSegments>& prevSegY) {
        static const std::array<Vec3, 5> noAnchors{};
        computeSegmentQuats(r, prevSegY, noAnchors, false, 0);
    }
    // 让离线验证程序能直接对比两种 roll 构造。
    static void TestHook_computeSegmentQuats(SkeletonFrameResult& r,
                                            std::array<Vec3, kNumSegments>& prevSegY,
                                            bool chainRoll) {
        static const std::array<Vec3, 5> noAnchors{};
        computeSegmentQuats(r, prevSegY, noAnchors, false, 0, 0.0, chainRoll);
    }

    // 带 anchor 的版本：真实路径里 sProx = anchor->pp（近节骨），
    // 不带 anchor 会退回 pp->mp（中节骨），测出来的东西完全不同。
    static void TestHook_computeSegmentQuatsA(SkeletonFrameResult& r,
                                              std::array<Vec3, kNumSegments>& prevSegY,
                                              const std::array<Vec3, 5>& anchors,
                                              bool chainRoll) {
        computeSegmentQuats(r, prevSegY, anchors, true, 0, 0.0, chainRoll);
    }

    static Mat3 TestHook_swingTo(const Mat3& parent, const Vec3& dir, double roll) {
        return swingTo(parent, dir, roll);
    }
    static void TestHook_computeSegmentQuatsRoll(SkeletonFrameResult& r,
                                                std::array<Vec3, kNumSegments>& prevSegY,
                                                bool chainRoll, double thumbRoll) {
        static const std::array<Vec3, 5> noAnchors{};
        computeSegmentQuats(r, prevSegY, noAnchors, false, 0, thumbRoll, chainRoll);
    }
private:

    static void computeSegmentQuats(SkeletonFrameResult& out,
                                    std::array<Vec3, kNumSegments>& prevSegY,
                                    const std::array<Vec3, 5>& anchorsMm,
                                    bool anchorsValid, int midMode,
                                    double thumbRoll = 0.0, bool chainRoll = true) {
        using namespace detail;
        out.segQuat[0] = matToQuat(out.wristR);
        for (int f = 0; f < 5; ++f) {
            const int pp = 5 + f * 3, mp = pp + 1, dp = pp + 2;
            const Vec3 dirProx = sub(out.markers[size_t(mp)].posWorld,
                                     out.markers[size_t(pp)].posWorld);
            Vec3 dirMid  = sub(out.markers[size_t(dp)].posWorld,
                               out.markers[size_t(pp)].posWorld);
            if (midMode == 1 ||
                (midMode == 2 && out.markers[size_t(pp)].observed && out.markers[size_t(mp)].observed))
                dirMid = sub(out.markers[size_t(mp)].posWorld, out.markers[size_t(pp)].posWorld);
            const Vec3 dirDist = sub(out.markers[size_t(dp)].posWorld,
                                     out.markers[size_t(mp)].posWorld);
            // 【近节骨的方向源】球心贴在每节指骨的【远端】附近（真 rig 的
            // setback 拟合结果），所以 pp球->mp球 这个向量跨的其实是【中节】
            // 骨，不是近节。近节骨没有任何一对 marker 能张成它——只能用
            // "该指根关节 -> pp球"。根关节由模板 anchor 经腕部位姿变换得到，
            // 所以这条路径依赖 wristPoseValid；不满足时退回 pp->mp。
            //
            // 在训练用的 hand_rig.py 上实测（64 受试者 × 200 姿态，0.5mm 噪声）：
            //   近节用 pp->mp : index 26.7° middle 28.1° ring 32.8° pinky 36.9°
            //   近节用 anchor : index 25.5° middle 24.0° ring 25.3° pinky 29.0°
            // 整体 p90 46.9°->30.4°、max 85.5°->54.3°。中位从 18.3° 到 13.0°。
            // 拇指例外：掌骨段用 pp->mp(16.5°) 比 anchor(24.2°) 好，因为拇指
            // 的 CMC 有轴向旋转，anchor 到球心的向量被旋前带偏。
            Vec3 dirProxUse = dirProx;
            if (f != 0 && anchorsValid && out.wristPoseValid) {
                const Vec3& a = anchorsMm[size_t(f)];
                const Vec3 root = {out.wristR[0]*a[0]+out.wristR[1]*a[1]+out.wristR[2]*a[2]+out.wristT[0],
                                   out.wristR[3]*a[0]+out.wristR[4]*a[1]+out.wristR[5]*a[2]+out.wristT[1],
                                   out.wristR[6]*a[0]+out.wristR[7]*a[1]+out.wristR[8]*a[2]+out.wristT[2]};
                dirProxUse = sub(out.markers[size_t(pp)].posWorld, root);
            }
            const int sProx = markerSegment(pp), sMid = markerSegment(mp), sDist = markerSegment(dp);
            // 拇指三段带解剖旋前偏置，四指为 0。见 boneFrame / thumbRollOffsetRad。
            if (chainRoll) {
                // 【沿链继承 roll】手背(可测的 3 自由度) -> 近节 -> 中节 -> 远节，
                // 每一级只做最小旋转(swing)，roll 原样带下来。见 swingTo() 的说明。
                // 【roll 偏移只加一次】segRollOffset 对拇指三段返回同一个值，
                // 那是为【旧的独立构造】设计的：每段各自基于手背法向，各加一次
                // 才对。沿链继承时第一段的偏移已经被下面两段继承下去了，
                // 再加就变成 2 倍、3 倍 —— 实测 thumbRoll=20° 时远节相对腕部
                // 累计到 60°。而 thumbRoll 的本意是"第一掌骨相对手背的常数旋前"，
                // 只该出现一次。
                const Mat3 fProx = swingTo(out.wristR, dirProxUse, segRollOffset(sProx, thumbRoll));
                const Mat3 fMid  = swingTo(fProx,      dirMid,     0.0);
                const Mat3 fDist = swingTo(fMid,       dirDist,    0.0);
                out.segQuat[size_t(sProx)] = matToQuat(fProx);
                out.segQuat[size_t(sMid)]  = matToQuat(fMid);
                out.segQuat[size_t(sDist)] = matToQuat(fDist);
            } else {
                // 旧路径：每节独立拿手背法向做 Gram-Schmidt。留着是为了能 A/B 对照，
                // 确认新路径在真机上确实更好之后可以删。
                out.segQuat[size_t(sProx)] = matToQuat(boneFrame(dirProxUse, out.wristR,
                                                       prevSegY[size_t(sProx)], segRollOffset(sProx, thumbRoll)));
                out.segQuat[size_t(sMid)]  = matToQuat(boneFrame(dirMid,  out.wristR,
                                                       prevSegY[size_t(sMid)],  segRollOffset(sMid, thumbRoll)));
                out.segQuat[size_t(sDist)] = matToQuat(boneFrame(dirDist, out.wristR,
                                                       prevSegY[size_t(sDist)], segRollOffset(sDist, thumbRoll)));
            }
        }
    }

    // 回填本帧的 y 轴，供下一帧在退化姿态下保持连续。
    // 【必须在 IK 覆盖之后调】否则存的是被 IK 丢弃的那一版几何解的 y 轴，
    // 下一帧的退化分支会拿一个跟当前输出对不上的参考做 Gram-Schmidt。
    static void storeSegY(const SkeletonFrameResult& out,
                          std::array<Vec3, kNumSegments>& prevSegY) {
        for (int s = 0; s < kNumSegments; ++s) {
            const Quat& q = out.segQuat[size_t(s)];
            const double w=q[0], x=q[1], y=q[2], z=q[3];
            prevSegY[size_t(s)] = {2*(x*y-z*w), 1-2*(x*x+z*z), 2*(y*z+x*w)};
        }
    }

    // -------------------------------------------------------------------------
    // 沿链继承 roll：把父节点的坐标系【最小旋转】到对齐本节骨轴。
    //
    // 【为什么这样才对】一条连线只给出骨轴（2 自由度），绕骨轴的第 3 个自由度
    // 点云里根本不存在。原来的做法是每一节都独立拿手背法向做 Gram-Schmidt ——
    // 那是一个【约定】，不是测量，而且它有个坏性质：外展一旦出平面，构造本身
    // 就会凭空造出扭转。数值验证（手背法向为 z，中节偏出平面 15°）：
    //     近节屈曲 20° -> 假扭转  9.6°
    //     近节屈曲 40° -> 假扭转 21.5°
    //     近节屈曲 60° -> 假扭转 53.1°
    // 屈得越深越离谱。而人的 PIP/DIP 是纯铰链，真骨【不会】扭转。
    //
    // 最小旋转（swing，不含 twist）正好对应铰链：父节点绕垂直于两轴的方向转
    // 到对齐，roll 原样带下来。于是：
    //   · 外展不再造假扭转
    //   · 不再依赖手背法向 -> 原来那个"骨轴≈手背法向"的退化分支自然消失
    //     （握拳时远节骨轴会转到那个方向，实测单帧最大跳到 172°）
    //   · 跟"关节是单弯曲轴"这个物理事实一致
    //
    // 根节点仍然是手背：5 个点能张成完整的 3 自由度坐标系，那是真的可测量。
    static Mat3 swingTo(const Mat3& parent, const Vec3& dir, double rollOffset = 0.0) {
        using namespace detail;
        const double n = std::sqrt(dot(dir, dir));
        if (!(n > 1e-9)) return parent;
        const Vec3 xc = {dir[0]/n, dir[1]/n, dir[2]/n};
        const Vec3 xp = {parent[0], parent[3], parent[6]};   // 行主序，第 1 列

        Vec3 y = {parent[1], parent[4], parent[7]};
        Vec3 z = {parent[2], parent[5], parent[8]};

        const Vec3 ax = cross(xp, xc);
        const double s = std::sqrt(dot(ax, ax));
        const double c = std::clamp(dot(xp, xc), -1.0, 1.0);
        if (s > 1e-9) {
            const Vec3 k = {ax[0]/s, ax[1]/s, ax[2]/s};
            const double ang = std::atan2(s, c);
            const double ca = std::cos(ang), sa = std::sin(ang);
            auto rot = [&](const Vec3& v) {
                const Vec3 kv = cross(k, v);
                const double kd = dot(k, v);
                return Vec3{v[0]*ca + kv[0]*sa + k[0]*kd*(1.0-ca),
                            v[1]*ca + kv[1]*sa + k[1]*kd*(1.0-ca),
                            v[2]*ca + kv[2]*sa + k[2]*kd*(1.0-ca)};
            };
            y = rot(y); z = rot(z);
        } else if (c < 0.0) {
            // 骨轴与父轴反向。手指屈曲最大 110°，物理上到不了这里；
            // 真出现说明标签错了或者点位坏了。绕 y 翻 180° 保证仍是右手系。
            y = {-y[0], -y[1], -y[2]};
        }
        // 正交化，防止长链上累积漂移
        const double py = dot(y, xc);
        y = {y[0]-py*xc[0], y[1]-py*xc[1], y[2]-py*xc[2]};
        const double ny = std::sqrt(dot(y, y));
        if (ny > 1e-9) { y = {y[0]/ny, y[1]/ny, y[2]/ny}; }
        z = cross(xc, y);

        if (rollOffset != 0.0) {
            // 拇指的 CMC 旋前是【真实存在】的那一维，铰链假设对它不成立，
            // 所以仍然要单独绕骨轴补一个常数。见 segRollOffset。
            const double cr = std::cos(rollOffset), sr = std::sin(rollOffset);
            const Vec3 y2{ y[0]*cr + z[0]*sr, y[1]*cr + z[1]*sr, y[2]*cr + z[2]*sr };
            const Vec3 z2{-y[0]*sr + z[0]*cr, -y[1]*sr + z[1]*cr, -y[2]*sr + z[2]*cr };
            y = y2; z = z2;
        }
        return {xc[0], y[0], z[0],
                xc[1], y[1], z[1],
                xc[2], y[2], z[2]};
    }

    static Mat3 boneFrame(const Vec3& dir, const Mat3& wristR, const Vec3& prevY,
                          double rollOffset = 0.0) {
        using namespace detail;
        const double n = std::sqrt(dot(dir, dir));
        if (!(n > 1e-9)) return {1,0,0, 0,1,0, 0,0,1};
        const Vec3 x = {dir[0]/n, dir[1]/n, dir[2]/n};
        // wristR 行主序，第 3 列 = 手背法线
        Vec3 ref = {wristR[2], wristR[5], wristR[8]};
        Vec3 y = cross(ref, x);
        double ny = std::sqrt(dot(y, y));
        if (ny < 0.15) {
            // 【真退化】骨轴≈手背法线。此时再换一个固定参考向量仍然是硬分支，
            // 只是把突变点挪了个位置而已（这条路我走过一次，单测 scenario5
            // 直接照出来了）。改用上一帧的 y 轴投影正交化：跨过退化区时朝向
            // 平滑延续，出了退化区又立刻回到手背法线这个绝对参考，不漂移。
            const double proj = dot(prevY, x);
            Vec3 yc = {prevY[0] - proj*x[0], prevY[1] - proj*x[1], prevY[2] - proj*x[2]};
            double nyc = std::sqrt(dot(yc, yc));
            if (nyc > 1e-6) { y = yc; ny = nyc; }
            else {
                // 冷启动且恰好退化：只能挑一个跟 x 不共线的固定向量
                ref = {wristR[0], wristR[3], wristR[6]};
                y = cross(ref, x);
                ny = std::sqrt(dot(y, y));
                if (ny < 1e-9) return {1,0,0, 0,1,0, 0,0,1};
            }
        }
        y = {y[0]/ny, y[1]/ny, y[2]/ny};
        Vec3 z = cross(x, y);
        if (rollOffset != 0.0) {
            // 绕 x（骨轴）转 rollOffset：y,z 在各自平面内旋转，x 不动。
            // 放在正交化【之后】，保证转的是一组已经标准正交的基。
            const double c = std::cos(rollOffset), sn = std::sin(rollOffset);
            const Vec3 y2{ y[0]*c + z[0]*sn, y[1]*c + z[1]*sn, y[2]*c + z[2]*sn };
            const Vec3 z2{-y[0]*sn + z[0]*c, -y[1]*sn + z[1]*c, -y[2]*sn + z[2]*c };
            y = y2; z = z2;
        }
        return {x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]};   // 列=基向量
    }

    // 跟上一帧取同半球。q 和 -q 表示同一个旋转，但符号翻转会让下游的
    // 插值/差分/导出看到一次 180° 的假跳变。
    void alignQuatHemisphere(SkeletonFrameResult& out) {
        auto fix = [](Quat& q, const Quat& ref) {
            const double d = q[0]*ref[0] + q[1]*ref[1] + q[2]*ref[2] + q[3]*ref[3];
            if (d < 0) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3]; }
        };
        if (hasPrevQuat_) {
            for (int s = 0; s < kNumSegments; ++s) fix(out.segQuat[size_t(s)], prevSegQuat_[size_t(s)]);
            fix(out.wristQuat, prevWristQuat_);
        }
        prevSegQuat_ = out.segQuat;
        prevWristQuat_ = out.wristQuat;
        hasPrevQuat_ = true;
    }
