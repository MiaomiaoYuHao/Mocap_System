// ===========================================================================
// Hm20Assoc_dorsum.ipp —— Hm20SkeletonAssociator 类体分片：手背：跟踪 / 刚体位姿 / 几何重标 / 手性投票
//
// 腕部系的来源。手背一错，后面所有腕部系下的量全错，
// 而且错得很稳定、看不出来——查"角度不对但四元数是对的"时先看这里。
//
// 【本文件被 #include 在 class 体内部】内容即类内定义（隐式 inline）。
// 不要加 include guard、不要加 namespace。开头的 access 说明符是显式写死的，
// 不依赖上一个分片以什么结尾——调换 include 顺序不会改变可见性。
// 独立语法检查：见 tests/test_assoc_fragments.cpp
// ===========================================================================
private:
    // 手背刚体求解状态
    DorsumRigidSolver dorsum_;
    Mat3 dorsumR_{1,0,0, 0,1,0, 0,0,1};
    Vec3 dorsumT_{};
    bool dorsumPoseOk_ = false;
    // 手背 tracklet 状态：5 条轨迹的位置 + 各自对 5 个标签的票数【已停用，见 cfg_.dorsumTracklet】
    mutable std::array<Vec3, 5> trkPos_{};
    mutable std::array<bool, 5> trkHas_{};
    mutable std::array<std::array<int, 5>, 5> trkVote_{};
    mutable bool trkInit_ = false;
    mutable int  trkFrames_ = 0;
    // ---- 几何手性：弯折方向累积 + 一次性锁定。见 accumulateGeoHandedness() ----
    double geoHandAcc_ = 0.0;
    bool   geoRefValid_ = false;
public:
    // 模板被重捕改动了多少（相对标定值）。>0 说明发生过重捕。
    double dorsumTemplateDriftMm() const { return dorsum_.templateDriftMm(); }
    // 面板要显示的那个数：手背模板"最优错解"的 Kabsch 残差。
    // <3mm = 布局病态（近似共面 + 近似反序对称），该去重贴点而不是继续调参数。
    // 标定一次算一次，不在每帧路径上。
    double dorsumSelfAmbiguityMm() const { return dorsum_.selfAmbiguityMm(); }

    // -------------------------------------------------------------------------
    // 几何手性：靠"手指往掌侧弯"这个物理事实定出手背法向的【绝对朝向】。
    //
    // 【为什么必须有外部参照】手性不是形状的内蕴属性。任何"只用手上的点、
    // 法向也从这些点算"的判据都【必然】区分不了左右手：
    //     镜像 M（det=-1）下  n' = -M·n，(X×V)' = -M(X×V)
    //     所以 (X×V)'·n' = (X×V)·n —— 两个负号抵消，恒不变号。
    // 我试过两版这类判据（手背五边形有符号面积、四指近节排列），
    // 实测都是"镜像后不变号"。这是数学结论，不是实现问题。
    //
    // 唯一可用的外部事实：反光球贴在手背【外】侧，手指屈曲时往【掌】侧走。
    // 所以看"骨节相对前一节往哪边弯"，就能定出法向的绝对朝向。
    //
    // 【为什么要按弯折角加权、并设门限】真机实测（889 帧握拳素材）：
    // 握紧时中远节球被手掌挡住，剩下看得见的那一段【恰恰是弯折最小的】——
    // PIP 夹角中位只有 12°，弯折量趋近 0，判据没有信息。
    // 不设门限的话这些无信息帧会把真信号稀释掉（实测一致率只有 62%）。
    //
    // 【为什么锁定】手性在一次会话里是常量。实测阈值 4.0 时两段素材都在
    // 11~32 帧（约半秒）内锁定，之后 100% 一致、永不翻转 ——
    // 这正是"遮挡时也要判对"的落地方式：在信息充足时定下来，
    // 之后遮挡再多也不重判。
    // -------------------------------------------------------------------------
    void accumulateGeoHandedness(const SkeletonFrameResult& out,
                                 const std::array<bool, kNumMarkers>& seen) {
        using namespace detail;
        if (geoHandLocked_ != 0) return;                 // 已锁定，不再改
        for (int m = 0; m < 5; ++m) if (!seen[size_t(m)]) return;   // 手背要全可见

        // 手背最佳拟合平面的法向（SVD 比三点叉积稳）
        Vec3 c{0, 0, 0};
        for (int m = 0; m < 5; ++m) c = add(c, out.markers[size_t(m)].posWorld);
        c = mul(c, 0.2);
        double cov[9] = {0,0,0,0,0,0,0,0,0};
        for (int m = 0; m < 5; ++m) {
            const Vec3 d = sub(out.markers[size_t(m)].posWorld, c);
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) cov[i*3+j] += d[size_t(i)] * d[size_t(j)];
        }
        // 最小特征向量 = 法向。5 个点用幂迭代求最小方向：对 (trace*I - cov) 迭代。
        double tr = cov[0] + cov[4] + cov[8];
        double A[9];
        for (int i = 0; i < 9; ++i) A[i] = ((i % 4 == 0) ? tr : 0.0) - cov[i];
        Vec3 n{1, 0, 0};
        for (int it = 0; it < 24; ++it) {
            Vec3 v{A[0]*n[0]+A[1]*n[1]+A[2]*n[2],
                   A[3]*n[0]+A[4]*n[1]+A[5]*n[2],
                   A[6]*n[0]+A[7]*n[1]+A[8]*n[2]};
            const double vn = norm(v);
            if (vn < 1e-12) return;
            n = mul(v, 1.0 / vn);
        }
        // 【同向化必须用自己的第一帧，不能用 wristR】
        //
        // 我先用的是 wristR 的第 3 列，实测两段同一只手的录制算出【相反】的符号
        // （dot 中位 +0.793 vs -0.884），因为 wristR 是 Kabsch 对【模板】求解的，
        // 模板手性一变它就翻 —— 它不是绝对参照，拿它同向化等于把结论建在
        // 待判定的量上。
        //
        // 改用自己记住的第一帧法向：它只要求【会话内一致】，
        // 而绝对朝向由后面"手指往掌侧弯"那个物理事实定出来。
        if (!geoRefValid_) { geoRef_ = n; geoRefValid_ = true; }
        else if (dot(n, geoRef_) < 0) n = mul(n, -1.0);

        for (int f = 0; f < 5; ++f) {
            const int a = 5 + 3*f, b = a + 1, d = a + 2;
            if (!seen[size_t(a)] || !seen[size_t(b)] || !seen[size_t(d)]) continue;
            Vec3 u0 = sub(out.markers[size_t(b)].posWorld, out.markers[size_t(a)].posWorld);
            Vec3 u1 = sub(out.markers[size_t(d)].posWorld, out.markers[size_t(b)].posWorld);
            const double L0 = norm(u0), L1 = norm(u1);
            if (L0 < 1e-6 || L1 < 1e-6) continue;
            u0 = mul(u0, 1.0 / L0); u1 = mul(u1, 1.0 / L1);
            const double cosA = std::clamp(dot(u0, u1), -1.0, 1.0);
            const double angDeg = std::acos(cosA) * 180.0 / M_PI;
            if (angDeg < cfg_.geoHandMinBendDeg) continue;   // 弯折不够，没信息
            const Vec3 perp = sub(u1, mul(u0, dot(u1, u0)));
            geoHandAcc_ += dot(perp, n) * (angDeg / 90.0);
        }
        if (std::fabs(geoHandAcc_) >= cfg_.geoHandLockThresh)
            geoHandLocked_ = (geoHandAcc_ >= 0) ? 1 : -1;
    }
    // 0 = 还没锁定；否则是锁定的符号。符号跟"哪只手"的对应关系由
    // 调用方结合模板手性确定 —— 这里只保证【同一只手符号恒定、镜像会变号】。
    int geoHandedness() const { return geoHandLocked_; }

    void TestHook_geoHand(const SkeletonFrameResult& r,
                          const std::array<bool, kNumMarkers>& seen) {
        accumulateGeoHandedness(r, seen);
    }
private:

    // -------------------------------------------------------------------------
    // 手背标签重定：丢掉模型给的【具体是哪个】，用穷举排列 + Kabsch 重新定。
    //
    // 【为什么这么做】我们一直在让模型做一件几何做得更好的事。
    // 模型要在 21 类里给每个点分类，而手背 5 点彼此空间最近，错了必然错到
    // 隔壁 —— 实测手背逐点准确率 91~96%，"手背<->手背"占全部错误的 20%。
    // 而几何只需要在 120 种排列里选一个，实测（真实 rig + 观测噪声）：
    //     噪声 0.5mm -> 100.0%    噪声 0.9mm(实际水平) -> 100.0%
    //     噪声 2.0mm ->  99.8%    噪声 4.0mm           ->  98.2%
    //     正解与次解的残差差 中位 7.63mm  <- 远超噪声，判据极确定
    //
    // 正确的分工：
    //     "哪些点属于手背"      -> 模型（语义问题，它要分手背/手指/幽灵）
    //     "这 5 个点各是哪一个" -> 几何（纯刚体配准，有闭式最优解）
    // 模型只需判对"手背 vs 非手背"这个二分类，比 21 分类容易得多。
    //
    // 【这也解释了之前的努力为什么收效有限】pent / sig / tracklet 投票都是在
    // 帮模型把一个它不该做的任务做好一点，而正解是把任务拿走。
    //
    // 【代价几乎为零】120 次 3x3 SVD，单帧几微秒，不到一次模型推理的万分之一。
    //
    // 【跟 tracklet 互补】几何管单帧的确定性；残差差不够时（罕见）不动，
    // 交给 tracklet 用时序兜底。两者不冲突。
    // -------------------------------------------------------------------------
    void relabelDorsumByGeometry(SkeletonFrameResult& out,
                                 std::array<bool, kNumMarkers>& seen) const {
        if (!cfg_.dorsumGeoRelabel) return;
        constexpr int kD = 5;
        // 5 点全可见才做：缺点时排列的自由度不够，穷举会挑出错解，
        // 宁可这一帧不动，也不要用不确定的重排把本来对的结果改坏。
        for (int m = 0; m < kD; ++m) if (!seen[size_t(m)]) return;

        std::vector<Vec3> tmpl(kD), obs(kD);
        for (int m = 0; m < kD; ++m) {
            tmpl[size_t(m)] = tmplMarker(m);
            obs[size_t(m)] = out.markers[size_t(m)].posWorld;
        }

        // 穷举 5! = 120 种排列，记下最优和次优的残差
        std::array<int, kD> perm{0, 1, 2, 3, 4}, best{0, 1, 2, 3, 4};
        double bestR = 1e18, secR = 1e18;
        std::vector<Vec3> cand(kD);
        std::array<double, 9> R{}; Vec3 t{};
        do {
            for (int i = 0; i < kD; ++i) cand[size_t(i)] = obs[size_t(perm[size_t(i)])];
            const double r = detail::kabsch(tmpl, cand, R, t);
            if (r < bestR) { secR = bestR; bestR = r; best = perm; }
            else if (r < secR) { secR = r; }
        } while (std::next_permutation(perm.begin(), perm.end()));

        out.dorsumGeoRmseMm = bestR;
        out.dorsumGeoMarginMm = secR - bestR;
        // 裕度不够说明这一帧的观测本身有问题（粘连/重影），重排是在赌
        if (secR - bestR < cfg_.dorsumGeoMinMarginMm) return;

        bool changed = false;
        for (int i = 0; i < kD; ++i) if (best[size_t(i)] != i) changed = true;
        if (!changed) return;
        // 只交换位置，不改 observed —— 重排是把已有的 5 个观测重新配对，
        // 不凭空创造观测。
        for (int i = 0; i < kD; ++i)
            out.markers[size_t(i)].posWorld = obs[size_t(best[size_t(i)])];
        ++out.dorsumGeoRelabeled;
    }

    // -------------------------------------------------------------------------
    // 手背刚体求解 —— 取代 relabelDorsumByGeometry() + trackDorsum()
    //
    // 【跟被取代的两条路的区别】
    //   · relabelDorsumByGeometry 只在模型给出的那 5 个手背点之间做排列，
    //     且要求 5 个全被认领。模型把手指点/杂点判成手背时它无从发现；
    //     缺一个点就完全不动。
    //   · trackDorsum 用点位移(25mm 门限)做跨帧关联，快速运动下必然丢锁，
    //     丢锁之后没有恢复路径，还会把旧位置写回并谎称 observed=true。
    //   · 这里改成：在【全部候选点】里做刚体匹配，"谁是手背点"和"是哪一个"
    //     一起解；歧义时用【位姿连续性】破(反序错解对应腕部转~180°，一票否决)，
    //     冷启动没有历史时用手指落点破；都用不上就【拒解】，绝不猜。
    //
    // 【为什么必须能拒解】手背 5 点近似共面 + 近似反序对称时，反序错解的
    // Kabsch 残差只比正解高 1.8mm 左右，跟观测噪声同量级。单帧看残差就是抛
    // 硬币，而抛错一次腕部系整个翻 180°，全手的连线一起错。宁可这一帧没有
    // 腕部位姿(wristPoseValid=false，下游沿用上一帧)，也不要一个翻过去的。
    // -------------------------------------------------------------------------
    void solveDorsumRigid(SkeletonFrameResult& out,
                          std::array<bool, kNumMarkers>& seen,
                          const std::vector<std::pair<int, Vec3>>& candidates, int N) {
        dorsumPoseOk_ = false;      // 【必须在最前面】后面每条 return 都靠它表示"没解出来"
        out.dorsumGeoRmseMm   = -1.0;
        out.dorsumGeoMarginMm = -1.0;
        out.dorsumGeoFixed    = 0;
        out.dorsumInliers     = 0;
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

        // 手指点只用于【歧义仲裁】，不参与刚体拟合
        std::vector<DorsumRigidSolver::FingerObs> fobs;
        for (int m = 5; m < kNumMarkers; ++m)
            if (seen[size_t(m)])
                fobs.push_back(DorsumRigidSolver::FingerObs{m, out.markers[size_t(m)].posWorld});

        const DorsumSolveResult r = dorsum_.solve(pts, fobs);
        out.dorsumGeoRmseMm   = r.rmseMm;
        out.dorsumGeoMarginMm = r.marginMm;
        out.dorsumSolveReason = r.reason;
        out.dorsumUsedHistory = r.usedHistory;
        out.dorsumInliers     = r.nMatched;

        // ---- 丢锁看门狗 ----
        // 【必须在 return 之前、且不管 r.ok 与否都要跑】它要处理的正是
        // "一直解不出来"的那种情况；放在 r.ok 分支里就永远等不到它。
        const int wd = dorsum_.watchdog(r);
        out.dorsumBadStreak = dorsum_.badStreak();
        out.dorsumRelock = wd;
        if (wd == 1) out.message = "手背丢锁：已松开连续性锁，等几何重新建锁";
        if (wd == 2) out.message = "手背长时间解不出：模板已退回标定值（等同重开面板）";

        if (!r.ok) {
            // 【解不出来就承认解不出来】不退回"就当模型是对的"，第 4 步会自然
            // 走"沿用上一帧腕部位姿"，wristPoseValid=false 让下游知道。
            //
            // 【但要区分两件事】这里的位置【是模型标好的、通常是对的】，
            // 只是我们没能用几何把它验证一遍。这跟"这个点根本不存在、是补出来的"
            // 完全不是一回事，可上一版把两者都写成 observed=false + id=-1，
            // 于是画面上出现"位置明明对、编号也在，却画成预测方块"。
            //
            // 保留 sourcePointId：它指向一个【真实存在的候选点】。
            // 用 confidence 区分档位，渲染层据此画成第三种样式而不是"预测"。
            for (int m = 0; m < 5; ++m) {
                seen[size_t(m)] = false;                 // 不参与 Kabsch，这一点不变
                out.markers[size_t(m)].observed = false;
                // >0 且 <0.5 = "点是真的，但几何没验过"；0 = "纯补出来的"
                out.markers[size_t(m)].confidence =
                    (out.markers[size_t(m)].sourcePointId >= 0) ? 0.35 : 0.0;
            }
            return;
        }

        for (int k = 0; k < 5; ++k) {
            const int j = r.pointOf[size_t(k)];
            if (j >= 0) {
                if (out.markers[size_t(k)].sourcePointId != candidates[size_t(j)].first)
                    ++out.dorsumGeoFixed;
                // ================== 抢点必须退旧标签 ==============================
                // 几何在【全部候选点】里重新认领手背标签，这是有意的（"把这件事
                // 从模型手里拿走"）。但它认领的那个点，可能上一步已经被匈牙利
                // 指派给某个【手指】标签了 —— 网络在手背这 5 个点上基本是在猜
                // （5 点两两距离区分度中位仅 ~1.3mm），偶尔会把手背球判成 f4pp
                // 或 f0pp。几何把球要回来是对的，但【没人通知手指那一边松手】。
                //
                // 后果：同一个 sourcePointId 挂在两个标签上，两个都 observed=true。
                // 真机实测 143/621 帧（23%）中招，w3↔f4pp 123 帧、w0↔f0pp 20 帧，
                // 两个标签位置完全重合（0.0mm，正常间距 38mm）。小指的 pp 因此
                // 坐到手背上，|f4pp-f4mp| 从 25mm 变成 47mm —— 而 pp 正是链式
                // 续解整条链的起点 P，起点错了后面全错。
                //
                // 一对一是这条管线的硬不变量（匈牙利存在的全部理由就是它），
                // 不该有任何一步能悄悄破坏它。
                for (int m = 5; m < kNumMarkers; ++m) {
                    if (!seen[size_t(m)]) continue;
                    if (out.markers[size_t(m)].sourcePointId != candidates[size_t(j)].first) continue;
                    // 退回"未观测"：位置留着当种子，但 observed/id 清掉，
                    // 下游（链式续解 / IK / 骨长回正）会把它当遮挡点重新算，
                    // 那正是它应得的待遇 —— 它本来就没被真的观测到。
                    out.markers[size_t(m)].observed = false;
                    out.markers[size_t(m)].confidence = 0.0;
                    out.markers[size_t(m)].sourcePointId = -1;
                    seen[size_t(m)] = false;
                    ++out.dorsumStoleFromFinger;
                }
                out.markers[size_t(k)] = SkeletonMarker{k, candidates[size_t(j)].second, true, 1.0,
                                                        candidates[size_t(j)].first};
                seen[size_t(k)] = true;
            } else {
                // 没匹配上的手背点：用刚体位姿反投影补。比网络 pos 头准(这一帧的
                // 刚体位姿刚被别的点解出来)，但【必须标未观测】——它是补的不是测的。
                out.markers[size_t(k)] = SkeletonMarker{k, dorsum_.project(r.R, r.t, k), false, 0.0, -1};
                seen[size_t(k)] = false;
            }
        }
        dorsumR_ = r.R;
        dorsumT_ = r.t;
        dorsumPoseOk_ = true;
        dorsum_.acceptHistory(r.R);      // 存成下一帧的连续性参考
        dorsum_.acceptCentroid(r.t);     // 位置历史，下一帧用它把候选点剪到附近

        // ---- 持续微调模板（点位小幅漂移的重适应）----
        // 跟重捕互补：重捕管"漂到匹配不上"，这个管"漂一点点还匹配得上"。
        // 后者原来完全没人管 —— 漂移在 inlierTolMm 以内时点仍然匹配，
        // 模板永不更新，误差就一直挂在残差里。5 个点的一阶低通，微秒级。
        if (dorsum_.refineTemplate(pts, r)) ++out.dorsumRefined;

        // ---- 贴点重捕 ----
        // 球掉了重贴回去时，新位置离模板 14~18mm（实测），而收点容差只有 6mm，
        // 于是求解器会【永远】拒绝它 —— 一直 4/5 内点直到重做整套标定。
        // 这里在更大半径里盯着那个"没人认领的点"，确认它在腕部系里的局部坐标
        // 是常数（=刚性固定在手上）之后，就地把模板改过去。
        const auto ev = dorsum_.observeForRepair(pts, r);
        out.dorsumRepaired = ev.happened ? (ev.slot + 1) : 0;
        out.dorsumRepairMoveMm = ev.happened ? ev.moveMm : -1.0;
        if (ev.happened || ev.rejected) {
            out.message = std::string(ev.happened ? "手背模板已就地修复(槽位 "
                                                  : "手背模板修复被拒(槽位 ")
                        + std::to_string(ev.slot) + "): " + ev.why;
        }
    }

    void trackDorsum(SkeletonFrameResult& out,
                     const std::array<bool, kNumMarkers>& seen) const {
        using namespace detail;
        if (!cfg_.dorsumTracklet) return;
        constexpr int kD = 5;

        // 本帧手背 5 个标签各自的观测位置（没观测到的跳过）
        std::array<Vec3, kD> cur{};
        std::array<bool, kD> curOk{};
        int nCur = 0;
        for (int m = 0; m < kD; ++m) {
            curOk[size_t(m)] = seen[size_t(m)];
            if (curOk[size_t(m)]) { cur[size_t(m)] = out.markers[size_t(m)].posWorld; ++nCur; }
        }
        if (nCur < 3) {                      // 手背看不清，这一帧不更新轨迹
            out.dorsumTrackN = trkFrames_;
            return;
        }

        if (!trkInit_) {
            for (int m = 0; m < kD; ++m) {
                trkPos_[size_t(m)] = cur[size_t(m)];
                trkHas_[size_t(m)] = curOk[size_t(m)];
                trkVote_[size_t(m)].fill(0);
                if (curOk[size_t(m)]) trkVote_[size_t(m)][size_t(m)] = 1;
            }
            trkInit_ = true; trkFrames_ = 1;
            out.dorsumTrackN = trkFrames_;
            return;
        }

        // ---- 最近邻把本帧的观测点匹配到已有轨迹 ----
        // 【贪心而不是匈牙利】5 个点、刚体运动、帧间位移亚毫米级，贪心足够；
        // 上一帧位置和本帧观测的距离远小于点间距(最小 20mm+)，不会配错。
        std::array<int, kD> assign{};
        assign.fill(-1);
        std::array<bool, kD> used{};
        for (int m = 0; m < kD; ++m) {
            if (!curOk[size_t(m)]) continue;
            int best = -1; double bd = cfg_.dorsumTrackGateMm;
            for (int t = 0; t < kD; ++t) {
                if (used[size_t(t)] || !trkHas_[size_t(t)]) continue;
                const double d = norm(sub(cur[size_t(m)], trkPos_[size_t(t)]));
                if (d < bd) { bd = d; best = t; }
            }
            if (best >= 0) { assign[size_t(m)] = best; used[size_t(best)] = true; }
        }

        // ---- 投票 + 位置更新 ----
        for (int m = 0; m < kD; ++m) {
            const int t = assign[size_t(m)];
            if (t < 0) continue;
            trkPos_[size_t(t)] = cur[size_t(m)];
            trkHas_[size_t(t)] = true;
            // 票数带衰减：贴法或握持方式变了时能慢慢改过来，不会被历史锁死
            for (int k = 0; k < kD; ++k)
                trkVote_[size_t(t)][size_t(k)] =
                    int(trkVote_[size_t(t)][size_t(k)] * 0.98);
            trkVote_[size_t(t)][size_t(m)] += 100;
        }
        ++trkFrames_;

        // ---- 用轨迹的多数票覆盖单帧标签 ----
        // 攒够帧数才敢覆盖：太早时投票本身就是噪声。
        if (trkFrames_ < cfg_.dorsumTrackWarm) { out.dorsumTrackN = trkFrames_; return; }

        // 每条轨迹选它得票最高的标签；冲突时(两条轨迹指向同一标签)只信票多的
        std::array<int, kD> trkLabel{}; trkLabel.fill(-1);
        std::array<int, kD> labOwner{}; labOwner.fill(-1);
        for (int t = 0; t < kD; ++t) {
            if (!trkHas_[size_t(t)]) continue;
            int bk = -1, bv = 0, sec = 0;
            for (int k = 0; k < kD; ++k) {
                const int v = trkVote_[size_t(t)][size_t(k)];
                if (v > bv) { sec = bv; bv = v; bk = k; }
                else if (v > sec) sec = v;
            }
            // 【票差不够就不覆盖】领先票必须明显高于第二名，否则这条轨迹
            // 自己也没想清楚，硬覆盖只会把对的改成错的。
            if (bk < 0 || bv < sec * 2) continue;
            trkLabel[size_t(t)] = bk;
            const int o = labOwner[size_t(bk)];
            if (o < 0) labOwner[size_t(bk)] = t;
            else {
                // 两条轨迹抢同一个标签：票多的赢，输的这轮不覆盖
                const int vo = trkVote_[size_t(o)][size_t(bk)];
                if (bv > vo) { trkLabel[size_t(o)] = -1; labOwner[size_t(bk)] = t; }
                else trkLabel[size_t(t)] = -1;
            }
        }

        // 把轨迹的结论写回 markers：轨迹 t 的位置应该挂在 trkLabel[t] 这个标签上
        std::array<Vec3, kD> fixed{};
        std::array<bool, kD> fixedOk{};
        int nFix = 0;
        for (int t = 0; t < kD; ++t) {
            const int k = trkLabel[size_t(t)];
            if (k < 0 || !trkHas_[size_t(t)]) continue;
            fixed[size_t(k)] = trkPos_[size_t(t)];
            fixedOk[size_t(k)] = true; ++nFix;
        }
        if (nFix < 3) { out.dorsumTrackN = trkFrames_; return; }
        int changed = 0;
        for (int k = 0; k < kD; ++k) {
            if (!fixedOk[size_t(k)]) continue;
            if (out.markers[size_t(k)].observed &&
                norm(sub(out.markers[size_t(k)].posWorld, fixed[size_t(k)])) > 1e-6)
                ++changed;
            out.markers[size_t(k)].posWorld = fixed[size_t(k)];
            out.markers[size_t(k)].observed = true;
        }
        out.dorsumTrackN = trkFrames_;
        out.dorsumTrackFixed = changed;
    }

    static bool checkPentagon(const SkeletonFrameResult& out) {
        using namespace detail;
        std::array<Vec3, 5> P{};
        Vec3 c{0, 0, 0};
        for (int i = 0; i < 5; ++i) { P[size_t(i)] = out.markers[size_t(i)].posWorld; c = add(c, P[size_t(i)]); }
        c = mul(c, 0.2);
        // 用协方差最小特征向量当法向，投到平面
        Mat3 C{};
        for (int i = 0; i < 5; ++i) {
            const Vec3 d = sub(P[size_t(i)], c);
            for (int r = 0; r < 3; ++r) for (int k = 0; k < 3; ++k)
                C[size_t(r*3+k)] += d[size_t(r)] * d[size_t(k)];
        }
        Mat3 V; Vec3 w;
        jacobiEigenSym3(C, V, w);
        const Vec3 e1 = {V[0], V[3], V[6]}, e2 = {V[1], V[4], V[7]};
        std::array<std::array<double,2>, 5> Q{};
        for (int i = 0; i < 5; ++i) {
            const Vec3 d = sub(P[size_t(i)], c);
            Q[size_t(i)] = {dot(d, e1), dot(d, e2)};
        }
        for (int i = 0; i < 5; ++i)
            for (int j = i + 1; j < 5; ++j) {
                const int i2 = (i + 1) % 5, j2 = (j + 1) % 5;
                if (i == j || i2 == j || j2 == i) continue;
                if (segsCross(Q[size_t(i)], Q[size_t(i2)], Q[size_t(j)], Q[size_t(j2)])) return false;
            }
        return true;
    }
