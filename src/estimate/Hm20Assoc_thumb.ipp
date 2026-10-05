// ===========================================================================
// Hm20Assoc_thumb.ipp —— Hm20SkeletonAssociator 类体分片：拇指专属：预测点回正 / 网络分段 / 旋前标定
//
// 拇指跟四指的运动学不一样（CMC 是鞍状关节），所有特判集中在这。
//
// 【本文件被 #include 在 class 体内部】内容即类内定义（隐式 inline）。
// 不要加 include guard、不要加 namespace。开头的 access 说明符是显式写死的，
// 不依赖上一个分片以什么结尾——调换 include 顺序不会改变可见性。
// 独立语法检查：见 tests/test_assoc_fragments.cpp
// ===========================================================================
private:
    mutable std::array<bool, kNumMarkers> thumbFixedThisFrame_{};
    // 网络段姿态的两个标定常数（模板系->规范系 C，拇指三段的常数修正 K）
    Mat3 netC_{1,0,0, 0,1,0, 0,0,1};
    std::array<Mat3, 3> netK_{};
    bool netConstValid_ = false;
    SegCanonAligner netSegAlignHolder_;   // 见 segAlign_
    SegCanonAligner& segAlign_ = netSegAlignHolder_;
public:
    void clearNetSegConstants() { netConstValid_ = false; }
    bool netSegConstantsValid() const { return netConstValid_; }

    // ---- 网络段常数的在线标定 ----
    // 【为什么放在关联器内部】标定要同时看到 out(几何解) 和 inf(网络的 segR)，
    // 而 InferenceOutput 只在 process() 里活着。放外面就得把它整个端到 worker
    // 层，接口变宽还容易忘记同步；放里面一行 feed 就完事。
    void setNetSegCalib(bool on) { cfg_.netSegCalib = on; }
    int  netSegCSamples() const { return segAlign_.cCount(); }
    int  netSegKSamples() const { return segAlign_.kCount(); }
    void resetNetSegCalib() { segAlign_.reset(); }

    // 解算并安装两个常数。返回的 report 里 kOffsetDeg 是【交叉验证点】：
    // probe_thumb_axis.py 量到模型的拇指铰链轴缺口是 73°，如果这里解出来的
    // K 转角落在 70~90°，说明两条互不依赖的路给出了同一个结论；
    // 如果它接近 0 或者散得离谱，说明前面某个假设错了，别开 thumbSegFromNet。
    NetSegCalibReport solveNetSegConstants() {
        NetSegCalibReport r;
        r.cSamples = segAlign_.cCount();
        r.kSamples = segAlign_.kCount();
        Mat3 C{};
        if (!segAlign_.solveC(C, &r.cSpreadDeg)) {
            r.why = (r.cSamples < 200) ? "C 的样本不够(要 200+，四指几何段)"
                                       : "C 的离散度超 15°：模板系->规范系不是常数，假设不成立";
            return r;
        }
        std::array<Mat3, 3> K{};
        if (!segAlign_.solveThumbK(C, K, &r.kSpreadDeg)) {
            r.why = (r.kSamples < 200) ? "K 的样本不够(要 200+，拇指三球全见的帧)"
                                       : "K 的离散度超 15°：拇指段的修正不是常数，别用";
            return r;
        }
        for (int j = 0; j < 3; ++j) r.kOffsetDeg[size_t(j)] = SegCanonAligner::offsetDeg(K[size_t(j)]);
        setNetSegConstants(C, K);
        r.ok = true;
        r.why = "OK";
        return r;
    }
private:
// ===========================================================================
// Hm20SkeletonAssociator_impl3.ipp —— Hm20SkeletonAssociator 类体分片 3/3。
// 【重要】被 #include 在 class 体内部（见 HandSkeletonAssociator.hpp），类内定义即 inline。
// 原 HandSkeletonAssociator.hpp 第 3468~4497 行（手背重标/私有实现与数据成员）。
// ===========================================================================

    // 以近节骨轴 u1 为第一轴、腕背外法向定第二轴的正交局部系。
    // 【为什么用腕背法向而不是随便找个垂直向量】随便找的话每帧朝向都可能翻，
    // 存进去的局部坐标就没有可比性。腕背法向跟着手走，是稳定的参考。
    static bool thumbLocalFrame(const Vec3& u1, const Mat3& wristR,
                                Vec3& F1, Vec3& F2, Vec3& F3) {
        using namespace detail;
        F1 = u1;
        const Vec3 dorsal{wristR[2], wristR[5], wristR[8]};   // 腕部系 Z 的世界方向
        const double p = dot(dorsal, F1);
        Vec3 e2{dorsal[0] - p * F1[0], dorsal[1] - p * F1[1], dorsal[2] - p * F1[2]};
        const double n2 = norm(e2);
        if (n2 < 1e-3) return false;      // 近节骨轴跟腕背法向共线，定不了
        F2 = mul(e2, 1.0 / n2);
        F3 = cross(F1, F2);
        return true;
    }

    void correctPredictedThumb(SkeletonFrameResult& out,
                               const std::array<bool, kNumMarkers>& seen) const {
        using namespace detail;
        // 【每一条跳过都要上报】上一版全写成静默 return，结果是"调参数画面没反应"
        // 而看不出为什么 —— 一个什么都不做又不吭声的功能，比没有这个功能更浪费时间。
        out.thumbFixed = 0;
        out.ikFilledMarkers = 0;
        out.chainContinued = 0;
        out.chainCoupledRad.fill(0.0);
        // 【必须每帧清】不清的话，某一指本帧压根没进链式续解那段循环时
        // （比如三点全见），留下的是上一帧的值——而分析时会把它当成本帧
        // 的真实状态读，那比没有数据更糟。
        out.chainHasPipHold.fill(false);
        out.chainHasPlane.fill(false);
        out.chainPmLenMm.fill(-1.0);
        out.chainMdLenMm.fill(-1.0);
        out.chainWeight.fill(-1.0);      // -1 = 本帧这一指没走到那段代码
        out.chainAging.fill(-1.0);
        out.chainCaseB.fill(false);
        out.chainUseAnchor.fill(false);
        out.chainCaseBFrames.fill(0);
        out.chainHasExc.fill(false);
        out.occludedHeld = 0;
        ikFilledThisFrame_.fill(false);
        chainFilledThisFrame_.fill(false);
        heldThisFrame_.fill(false);
        snappedThisFrame_.fill(false);
        jumpLimitedThisFrame_.fill(false);
        thumbFixedThisFrame_.fill(false);
        const double d = cfg_.thumbPronationRad;
        if (!(std::fabs(d) > 1e-6)) { out.thumbFixSkip = 1; return; }   // 参数=0，关着

        // 有没有需要回正的点？三颗全是实测就没什么可做的（这是【正常】状态，
        // 不是错误：拇指没被遮住的时候本来就不该动它）。
        int nSeen = 0;
        for (int m = 5; m <= 7; ++m) if (seen[size_t(m)]) ++nSeen;
        if (nSeen == 3) { out.thumbFixSkip = 2; return; }

        // 【2026-08 收紧】只在拇指三颗球【全部】未观测时才动手。
        //
        // 旧行为是"只转 observed==false 的点"，链条中间有实测点时会把链拧断：
        // 掌骨球(5)、近节球(6)实测、远节球(7)预测 —— 这是最常见的遮挡形态 ——
        // 只把 7 绕掌骨轴转 80°，5/6 原地不动，于是
        //     近节骨轴 = p7 - p5   远节骨轴 = p7 - p6
        // 两个方向都不再有物理意义。实测面板上"拇 近节"的相对父节点四元数
        // 对应 118°，而拇指 MCP 的解剖限位只有 0.95 rad = 54° —— 这个数字本身
        // 就说明链是断的，不是偏的。
        //
        // 旋前是整条 CMC 后链的刚体自转，只有整条链一起转才自洽。
        // 部分遮挡的情形交给 IK(markerFK 里的 thumbPronation0)去补 ——
        // 那条路是用真实观测约束着解的，不会凭空拧断链条。
        if (nSeen != 0) { out.thumbFixSkip = 7; return; }

        if (!wristPoseValid_) { out.thumbFixSkip = 3; return; }         // 没腕部系，转轴无从谈起

        // CMC 锚点：优先用标定出来的；没有就退化成手背质心。
        // 【退化档为什么还值得做】转轴方向取"锚点 -> 掌骨球"，手背质心比真 CMC
        // 偏尺侧十几毫米，转轴方向因此偏几度到十几度 —— 比 anchorsValid_ 之前
        // 什么都不做要好。自标定到 Ready 之后会自动切回精确档。
        Vec3 A;
        if (anchorsValid_) {
            const Vec3& a = anchorsMm_[0];
            A = Vec3{out.wristR[0]*a[0] + out.wristR[1]*a[1] + out.wristR[2]*a[2] + out.wristT[0],
                     out.wristR[3]*a[0] + out.wristR[4]*a[1] + out.wristR[5]*a[2] + out.wristT[1],
                     out.wristR[6]*a[0] + out.wristR[7]*a[1] + out.wristR[8]*a[2] + out.wristT[2]};
        } else {
            Vec3 c{0, 0, 0};
            int nd = 0;
            for (int m = 0; m < 5; ++m) if (seen[size_t(m)]) {
                const Vec3& p = out.markers[size_t(m)].posWorld;
                c = {c[0] + p[0], c[1] + p[1], c[2] + p[2]}; ++nd;
            }
            if (nd < 3) { out.thumbFixSkip = 4; return; }   // 手背都看不见，放弃
            A = Vec3{c[0]/nd, c[1]/nd, c[2]/nd};
            out.thumbFixSkip = 5;                            // 5 = 生效但走的是退化锚点
        }

        // 转轴 = 掌骨长轴的估计。优先用实测的掌骨球；它也没观测到就用预测值
        // （比没有强，但误差更大）。
        Vec3 n = sub(out.markers[5].posWorld, A);
        const double nn = std::sqrt(dot(n, n));
        if (!(nn > 1e-6)) { out.thumbFixSkip = 6; return; }
        n = {n[0]/nn, n[1]/nn, n[2]/nn};

        // Rodrigues：绕 (A, n) 转 d
        const double c = std::cos(d), s2 = std::sin(d);
        for (int m = 5; m <= 7; ++m) {
            if (seen[size_t(m)]) continue;               // 实测点不动
            const Vec3 v = sub(out.markers[size_t(m)].posWorld, A);
            const Vec3 k = cross(n, v);
            const double nv = dot(n, v);
            out.markers[size_t(m)].posWorld = {
                A[0] + v[0]*c + k[0]*s2 + n[0]*nv*(1.0-c),
                A[1] + v[1]*c + k[1]*s2 + n[1]*nv*(1.0-c),
                A[2] + v[2]*c + k[2]*s2 + n[2]*nv*(1.0-c)};
            ++out.thumbFixed;
            thumbFixedThisFrame_[size_t(m)] = true;
        }
    }

    void applyNetThumbSegments(SkeletonFrameResult& out, const InferenceOutput& inf) {
        using namespace detail;
        out.netThumbSegs = 0;
        if (!cfg_.thumbSegFromNet || !netConstValid_) return;
        if (!inf.hasSegR || !out.wristPoseValid) return;

        for (int j = 0; j < 3; ++j) {
            const int seg = 1 + j;
            // 几何/IK 可信就别动它
            const SegSource src = out.segSource[size_t(seg)];
            if (src == SegSource::Geometry || src == SegSource::Ik) continue;

            // R = wristR · C · segR[seg] · K[j]
            const Mat3 a = matMul(out.wristR, netC_);
            const Mat3 b = matMul(a, inf.segR[size_t(seg)]);
            const Mat3 R = matMul(b, netK_[size_t(j)]);
            const Vec3 axis{R[0], R[3], R[6]};             // 列 0 = 骨轴
            if (!(norm(axis) > 1e-6)) continue;
            out.segQuat[size_t(seg)] =
                matToQuat(boneFrame(axis, out.wristR, prevSegY_[size_t(seg)],
                                    segRollOffset(seg, cfg_.thumbRollOffsetRad)));
            out.segSource[size_t(seg)] = SegSource::Net;
            out.segConf[size_t(seg)] = 0.6;               // 比几何低、比纯预测高
            ++out.netThumbSegs;
        }
    }
