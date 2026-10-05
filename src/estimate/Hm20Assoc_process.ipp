// ===========================================================================
// Hm20Assoc_process.ipp —— Hm20SkeletonAssociator 类体分片：每帧主流程（编排层）
//
// process() 只做编排：推理 -> 指派 -> 腕部位姿 -> 补全 -> 朝向 -> 调试流。
// 【它自己不实现算法】任何一段具体算法都应该在下面某个职责文件里，
// 这里只保留调用顺序和帧级状态。改算法别改这个文件。
//
// 【本文件被 #include 在 class 体内部】内容即类内定义（隐式 inline）。
// 不要加 include guard、不要加 namespace。开头的 access 说明符是显式写死的，
// 不依赖上一个分片以什么结尾——调换 include 顺序不会改变可见性。
// 独立语法检查：见 tests/test_assoc_fragments.cpp
// ===========================================================================
public:

    // candidates: (点云界面的 track id, 世界坐标)。id 只回填给 UI，关联逻辑
    // 完全不依赖它跨帧稳定 —— 这就是"点云 ID 怎么跳变都能正确连接"的落地点。
    SkeletonFrameResult process(const std::vector<std::pair<int, Vec3>>& candidatesIn) {
        using namespace detail;
        SkeletonFrameResult out;

        // ---- 0. 离群候选点闸：一个坏点会毁掉【整帧】-------------------------
        // 【为什么必须在这里挡】网络的归一化（center / scale）是模型【内部】
        // 算的，我们改不了；而它用的是全体输入点的质心和展布。
        //
        // 真机实测（静止保持录制，489 帧）：第 107 帧有【一个】点被三角化到
        //     id=2001  (4461, 7782, -6832)   离手部中位点 11192mm
        // 手掌本身的展布只有 98mm —— 这个点远了 114 倍。后果是一条完整的链：
        //     center  [16,112,11]  ->  [239,497,-331]
        //     scale   61mm         ->  2441mm
        //     于是 20 个真实点在归一化空间里全被压进 4% 的范围，网络看到的是
        //     一个退化的点团，输出【21 类均匀分布】（实测每类 0.05 ≈ 1/21）
        //     -> 20 个候选点全部过不了置信度闸 -> numGhost=20
        //     -> 15 个标签同时丢失 -> 整只手闪一下变成预测
        // 这就是"静止的时候突然会消失点变成预测"。全程 489 帧里只有这一帧，
        // 但一帧就足够看见。
        //
        // 【为什么上游的残差闸挡不住】那个点 usedViews=2、residualMm=0.0000。
        // 两视图三角化在数学上总能让两条射线"相交"（取最近点），视线越接近
        // 平行，交点飞得越远、而残差【恒等于 0】。也就是说残差对这一档
        // 完全没有分辨力 —— 这正是它漏过来的原因。真正的判据是视线夹角
        // （parallax），那该在 Triangulation 里加；这里是最后一道防线。
        //
        // 【判据用中位数不用均值】均值本身就会被离群点拖走，拿它当参照等于
        // 让坏点自己给自己发通行证。20 个点里混 1~2 个离群点时中位数纹丝不动。
        // 阈值取"离中位点 > kCandMaxSpreadMm"，600mm 是人手直径的三倍多，
        // 任何真实的手部标记球都不可能落在外面。
        std::vector<std::pair<int, Vec3>> candFiltered;
        {
            const size_t n0 = candidatesIn.size();
            if (n0 >= 5) {
                std::vector<double> xs, ys, zs;
                xs.reserve(n0); ys.reserve(n0); zs.reserve(n0);
                for (const auto& c : candidatesIn) {
                    xs.push_back(c.second[0]);
                    ys.push_back(c.second[1]);
                    zs.push_back(c.second[2]);
                }
                auto med = [](std::vector<double>& v) {
                    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
                    return v[v.size() / 2];
                };
                const Vec3 c0{med(xs), med(ys), med(zs)};
                candFiltered.reserve(n0);
                for (const auto& c : candidatesIn)
                    if (norm(sub(c.second, c0)) <= kCandMaxSpreadMm)
                        candFiltered.push_back(c);
                // 【全被剔光就整批放行】能走到这儿说明中位数本身就在离群区，
                // 那是另一种故障（比如整只手真的不在标定体积里）。宁可把这一帧
                // 原样交给网络，也不要凭空造出一个"候选点不足"的假象。
                if (candFiltered.size() < 5) candFiltered = candidatesIn;
            } else {
                candFiltered = candidatesIn;
            }
            out.assocOutliersDropped = int(n0) - int(candFiltered.size());
        }
        const std::vector<std::pair<int, Vec3>>& candidates = candFiltered;

        const int N = int(std::min(candidates.size(), size_t(kMaxPoints)));
        if (N < 3) { out.message = "候选点不足 3 个"; return out; }

        // ---- 1. 推理（不需要任何预处理，网络输入是旋转/平移/尺度不变量）----
        InferenceInput in;
        in.points.reserve(size_t(N));
        for (int i = 0; i < N; ++i) in.points.push_back(candidates[size_t(i)].second);
        in.tmpl = tmpl_;
        in.tmplValid = cfg_.useTemplate && tmplValid_;
        if (cfg_.usePrevFrame && hasPrev_) { in.prev = prevPos_; in.prevMask = prevMask_; }

        const InferenceOutput inf = backend_->run(in);

        // ---- 手性投票（v7 新头）----
        // 【为什么要投票而不是每帧直接用】手性在一次会话里是【常量】——
        // 用户不会训练途中换手。逐帧信会让偶发的错判直接翻转整只手的模板系
        // (Y 翻转 -> anchor 全错 -> IK 残差爆掉)，代价远大于慢几帧才锁定。
        // 【为什么锁定后不再改】跟 WristHold 同一个模式：一个稳定的判定 +
        // 用户可覆盖，比一个"随时可能翻"的自动值可靠。
        // 训练集准确率 98.5%，30 帧一致 (@45fps 约 0.7 秒) 足够可信。
        if (inf.hasHandLogit) {
            const bool r = inf.handLogit > 0.0;
            const double conf = std::tanh(std::fabs(inf.handLogit));

            // 【改成滑动多数表决，不是"连续一致"】
            //
            // 原来是 `if (r == handVote_) ++n; else { handVote_ = r; n = 1; }` ——
            // 【一帧不一致就清零重来】。真机症状：指尖一被遮挡，候选点少一个，
            // 模型的 hand_logit 翻一下，计数就归零，锁定永远建立不起来。
            // 而 aiHandIsRight_ 是【每帧无条件更新】的，UI 显示的就是那个逐帧值
            // —— 于是"指尖被遮挡马上手性错判"。
            //
            // 上面那段注释写着"逐帧信会让偶发错判翻转整只手"，
            // 但实现里 aiHandIsRight_ 恰恰是逐帧的。注释描述的保护没落到实处。
            //
            // 【为什么用置信度加权】遮挡时模型本来就没把握，|handLogit| 小；
            // 让它自己的置信度决定投票权重，比一票一权更合理。
            handVoteAcc_ = 0.92 * handVoteAcc_ + (r ? conf : -conf);
            // 模型平均有多大把握（同一个时间常数，好跟累加器直接比）
            handConfAvg_ = 0.92 * handConfAvg_ + 0.08 * conf;

            // ---- 证据够不够下结论 ----
            // 上限 = 假如每一帧都同号、一直累到稳态时累加器会是多少。
            // 拿实际累加值跟它比，得到一个【与模型自信程度无关】的一致性度量。
            const double cap = handConfAvg_ / 0.08;
            const bool enough = (handConfAvg_ >= cfg_.handConfFloor)
                             && (std::fabs(handVoteAcc_) >= cfg_.handVoteRel * cap);

            if (enough) {
                const bool major = (handVoteAcc_ >= 0.0);
                if (handKnown_ && major == handVote_) ++handVoteN_;
                else { handVote_ = major; handVoteN_ = 1; }
                handKnown_ = true;
                if (!aiHandLocked_ && handVoteN_ >= 30) {
                    aiHandLocked_ = true;
                    aiHandLockedValue_ = major;
                }
            } else if (!aiHandLocked_) {
                // 【证据不足就明说不知道，不要猜】
                // 原来这里没有"不知道"这个状态，`major = (acc >= 0)` 在
                // acc 停在 0 附近时逐帧抖；而如果给它一个初值再加死区，
                // 就会稳定地报那个初值 —— 对左手用户就是稳定报右手。
                // 两种都比"未定"差：前者是抖，后者是自信地错。
                handKnown_ = false;
                handVoteN_ = 0;
            }

            // 【锁定必须真的锁住】原来 aiHandLockedValue_ 只被写、从没被读过，
            // out.aiHandIsRight 永远取逐帧的多数 —— 面板上显示"(已锁定)"，
            // 而那个值还在跳。锁的语义没有落到输出上，等于没锁。
            aiHandKnown_ = aiHandLocked_ || handKnown_;
            aiHandIsRight_ = aiHandLocked_ ? aiHandLockedValue_ : handVote_;
            aiHandConf_ = aiHandKnown_
                        ? std::min(1.0, std::fabs(handVoteAcc_) / std::max(cap, 1e-6))
                        : 0.0;
        }
        out.aiHandKnown = aiHandKnown_;
        out.aiHandIsRight = aiHandIsRight_;
        out.aiHandConf = aiHandConf_;
        out.aiHandLocked = aiHandLocked_;
        out.hasAiHand = inf.hasHandLogit;
        // 姿态原样透传给上层，由上层决定信几分（见 poseConf）
        out.aiJointAng = inf.jointAng;
        out.hasAiJointAng = inf.hasJointAng;
        out.aiPoseConf = inf.poseConf;
        out.hasAiPoseConf = inf.hasPoseConf;
        // 【防御】不信任后端一定按约定返回。尺寸对不上就直接失败，绝不按
        // N 去索引一个更短的 vector —— 那是越界读，属于未定义行为。
        if (!inf.ok || inf.cols != kNumClasses || inf.rows < N ||
            inf.logAssign.size() < size_t(inf.rows) * size_t(inf.cols)) {
            out.message = "推理后端返回的张量尺寸与契约不符";
            return out;
        }

        // ---- 2. 匈牙利一对一指派 ----
        // 代价 = -log_assign。dustbin 列（第 20 列）可被多个点重复使用，所以
        // 把它复制 N 份变成 N 个等价列，再做标准的一对一指派。
        const int nc = kNumMarkers + N;
        std::vector<double> cost(size_t(N) * size_t(nc));
        for (int i = 0; i < N; ++i) {
            const float* row = &inf.logAssign[size_t(i) * size_t(kNumClasses)];
            for (int j = 0; j < kNumMarkers; ++j)
                cost[size_t(i) * size_t(nc) + size_t(j)] = -double(row[j]);
            for (int j = kNumMarkers; j < nc; ++j)
                cost[size_t(i) * size_t(nc) + size_t(j)] = -double(row[Ghost]);
        }
        std::vector<int> assign;
        hungarian(cost, N, nc, assign);

        // ---- 3. 落回 20 个标签 ----
        std::array<bool, kNumMarkers> seen{};
        seen.fill(false);
        for (int i = 0; i < N; ++i) {
            const int j = assign[size_t(i)];
            if (j < 0 || j >= kNumMarkers) { ++out.numGhost; continue; }
            const double p = std::exp(double(inf.logAssign[size_t(i) * size_t(kNumClasses) + size_t(j)]));
            // 手背/手指分别用各自的阈值，见 Hm20Config::minAssignProbFor 的说明
            if (p < cfg_.minAssignProbFor(j)) { ++out.numGhost; continue; }
            seen[size_t(j)] = true;
            out.markers[size_t(j)] = {j, candidates[size_t(i)].second, true, p,
                                      candidates[size_t(i)].first};
        }
        // 没被认领的标签 -> 先用网络 pos 头的预测兜底
        for (int m = 0; m < kNumMarkers; ++m) if (!seen[size_t(m)]) {
            out.markers[size_t(m)] = {m, inf.pos[size_t(m)], false, 0.0, -1};
        }
        out.numObserved = int(std::count(seen.begin(), seen.end(), true));

        // ---- 手背标签重定：把这件事从模型手里拿走，交给几何 ----
        solveDorsumRigid(out, seen, candidates, N);

        // ---- v5 快照：第 0 级（网络原始）和第 1 级（指派 + 手背重定后）----
        // 【这两级的差就是"指派改了什么"】第 0 级是模型 pos 头对 20 个标签的
        // 无条件预测；第 1 级里被认领的标签换成了真实测量、没认领的仍是模型值。
        // 所以 |stage1-stage0| 只在【被认领的点】上非零，而它的大小 =
        // 模型的位置预测离真实测量有多远 —— 这是判断模型准不准的直接量，
        // 而且不依赖任何下游处理。
        if (cfg_.captureDebugStreams) {
            for (int m = 0; m < kNumMarkers; ++m) {
                out.stagePos[0][size_t(m)] = inf.pos[size_t(m)];
                out.stagePos[1][size_t(m)] = out.markers[size_t(m)].posWorld;
            }
            out.stageValid[0] = true;
            out.stageValid[1] = true;
        }

        // ---- 手背 tracklet：跨帧跟踪 + 标签投票 ----
        // 【为什么专门给手背做】它是刚体，跨帧的物理球就是同一批；而单帧
        // argmax 的逐点错误率约 8%，手背只有 5 个标签且彼此空间最近，错了
        // 必然错到另一个手背点上 —— 实测"手背<->手背"占全部错误的 20%，
        // 是第三大错误类型，而且一旦错就是整只手转 72°(腕部位姿全变)。
        //
        // 【为什么 tracklet 有效】这是专业动捕(OptiTrack/Vicon/RoMo)的通用做法：
        // 先做点的时序跟踪，再给【整条轨迹】一个标签，而不是每帧独立判。
        // 一条轨迹几十帧的证据合起来，单帧 8% 的错误率会被压到接近 0；
        // 而且轨迹定了之后中间那些模糊帧自动跟着定，不会出现单帧跳变。
        //
        // 【为什么不做硬判定】投票是软的：证据不足时保持上一次的结论，
        // 不强行每帧重选。这对应文献里的"多假设、不做硬判定"。
        // trackDorsum(out, seen);   // 【已停用】见 Hm20Config::dorsumTracklet

        // ---- 【解耦点】在这里就把 prev 定下来，用【网络原始输出】 ----
        // 姿态侧的后处理（拇指回正、链式续解、遮挡限速、时序滤波）从这一行
        // 之后才发生。原来 prev 是在函数末尾从 out.markers 取的 —— 那时它已经
        // 被那些后处理改过了，于是【姿态侧的假设被喂回了模型的输入】。
        //
        // 这是一条隐蔽的耦合：模型的标签能力本来跟姿态无关，却因为这条回灌，
        // 一旦姿态侧的某个假设不对(比如 anchor 坏了导致链式续解摆错位置)，
        // 下一帧模型看到的 prev 就是错的，标签跟着退化 —— 表现就是
        // "自标定一开，连线就乱"。
        //
        // 模型该看到的是它自己上一帧的输出，加上真观测点。姿态怎么修是下游
        // 的事，不该回头影响上游。
        for (int m = 0; m < kNumMarkers; ++m) {
            prevPos_[size_t(m)] = out.markers[size_t(m)].posWorld;
            prevMask_[size_t(m)] = out.markers[size_t(m)].observed;
        }
        hasPrev_ = true;

        // ---- 4. 腕部位姿：Kabsch（注意顺序 —— 是标注【之后】才解，不是之前）----
        // 模板手背点 <-> 观测手背点。
        //
        // 【修改】手背点不足 3 个时【不再】退化到全 20 点 Kabsch。
        // 原因：手指是活动的，模板里存的是中立位手指位置，拿它去和当前帧
        // 屈曲状态下的手指点做刚体拟合，等于把手指的关节运动硬当成腕部的
        // 刚体运动。实测（120fps 抓握，漏检率 4.96%）这条兜底路径会让腕部
        // 四元数帧间跳变的最大值从 3.7° 冲到 102.2° —— 手一动腕就乱转。
        // 宁可这一帧不给腕部位姿（wristPoseValid=false，下游沿用上一帧或
        // 走 IK），也不要给一个被手指姿态污染的错值。
        // 【2026-08 改】不再在这里单独跑一次 Kabsch。solveDorsumRigid() 已经用
        // 它自己挑出来的内点解出了 R,t —— 这里再解一次的话，"用哪几个点"的选择
        // 跟求解器不一致（它按内点、这里按 seen），缺点时两边会得到不同的位姿，
        // 而下游(IK/anchor/骨轴)全部建在 wristR 上，两套位姿会打架。
        wristPoseValid_ = false;
        if (dorsumPoseOk_) {
            out.wristR = dorsumR_;
            out.wristT = dorsumT_;
            out.dorsumRmseMm = out.dorsumGeoRmseMm;
            wristPoseValid_ = true;
            lastWristR_ = out.wristR;
            lastWristT_ = out.wristT;
            hasLastWrist_ = true;
        } else if (hasLastWrist_) {
            // 沿用上一帧。dorsumRmseMm 保持 -1，调用方据此知道这一帧的腕部
            // 姿态是外推的、不是本帧解出来的。
            out.wristR = lastWristR_;
            out.wristT = lastWristT_;
            out.message = "手背刚体未解出，腕部位姿沿用上一帧";
        } else {
            out.message = "手背刚体未解出且无历史，腕部位姿未解出";
        }
        out.wristPoseValid = wristPoseValid_;
        out.wristQuat = matToQuat(out.wristR);

        // ---- 5. 分段朝向：几何算，不是网络输出（保证物理自洽）----
        // 【顺序修正 2026-08 —— 这是"开 IK 反而更差"的直接原因】
        // 原来的顺序是【先 IK、后 computeSegmentQuats】，而 computeSegmentQuats
        // 无条件重写全部 16 个 segQuat —— IK 辛苦解出来的分段朝向在下一行就被
        // 整个覆盖掉了，一个都没留下。IK 唯一的净效果变成"把网络 pos 头补的
        // 遮挡点换成 FK 摆出来的点"，而那些点【比网络预测更差】（群体均值贴点
        // 参数，残差中位 4.6mm，网络 pos 头是端到端训出来的），于是拿它们去连
        // 两球算骨轴，结果比不开 IK 还烂。
        // 真机实测（6 相机 + 不同步 + 遮挡 + 粘连，两个随机受试者）：
        //     IK 关          骨轴中位 10.80° / 13.22°     遮挡点 6.40 / 5.76 mm
        //     IK 开(修复前)  骨轴中位 11.30° / 15.53°     遮挡点 6.98 / 7.67 mm  ← 更差
        // 现在改成【先几何、后 IK】，IK 只覆盖它真正解得动的那几根手指。
        verifyAnchors(out);      // 【必须在算分段朝向之前】，见其内部说明
        // anchor 链路状态归档（见 SkeletonFrameResult 里的说明）
        out.anchorsSet           = anchorsSet_;
        out.anchorsChecked       = anchorsChecked_;
        out.anchorsValid         = anchorsValid_;
        out.tmplMmValid          = tmplMmValid_;
        out.anchorsVerifyNSeen   = anchorsVerifyNSeen_;
        out.anchorsVerifyNOk     = anchorsVerifyNOk_;
        correctPredictedThumb(out, seen);   // 【必须在算分段朝向之前】它改的是骨轴的输入
        computeSegmentQuats(out, prevSegY_, anchorsMm_, anchorsValid_, cfg_.midDirMode,
                            cfg_.thumbRollOffsetRad, cfg_.chainRoll);
        for (int f = 0; f < 5; ++f) {
            int n = 0;
            for (int j = 0; j < 3; ++j) if (seen[size_t(5 + 3*f + j)]) ++n;
            out.fingerObsCount[size_t(f)] = n;
        }
        markSegmentSources(out);

        // ---- v5 快照：第 2 级（拇指回正 + 骨轴/分段朝向之后）----
        // correctPredictedThumb 会【改点的位置】（不只是朝向），所以这一级
        // 跟上一级的差不为零。拇指单独歪的问题，差值集中在 5..7 三个点上时
        // 就能确认是这一步干的，而不用再去怀疑 IK 或滤波。
        if (cfg_.captureDebugStreams) {
            for (int m = 0; m < kNumMarkers; ++m)
                out.stagePos[2][size_t(m)] = out.markers[size_t(m)].posWorld;
            out.stageValid[2] = true;
        }

        // ---- 6. 可选 IK 精修（遮挡点补全 + 分段朝向）----
        // 【必须有腕部位姿才能做】IK 在【腕部系】里解，wristR/wristT 是唯一的
        // 世界<->腕部换算。wristPoseValid=false 时 wristR 要么是单位阵（冷启动）
        // 要么是好几帧之前的旧值，把世界点按它转进"腕部系"得到的是一堆几何上
        // 无意义的坐标，IK 会认认真真地拟合它，然后输出一组自洽但完全错误的
        // 关节角。宁可这一帧不精修。
        if (ik_ && out.wristPoseValid) {
            std::array<Vec3, 20> mk{};
            std::array<bool, 20> ob{};
            for (int m = 0; m < kNumMarkers; ++m) {
                mk[size_t(m)] = out.markers[size_t(m)].posWorld;
                ob[size_t(m)] = out.markers[size_t(m)].observed;
            }
            std::array<Quat, kNumSegments> ikQuat = out.segQuat;
            if (ik_->refine(mk, ob, out.wristR, out.wristT, ikQuat)) {
                const IkFrameInfo info = ik_->lastFrameInfo();
                // 【整份存进 result】原来只取了 rmseMm 和 fingerValid 两项，
                // 其余（解出来的 4 自由度角、限位命中、实际用的骨长/anchor、
                // 求解器用的手性）在这一行之后就丢了。而遮挡点的位置正是这些角
                // FK 出来的 —— 位置不对时要往上查一级，查的就是它们。
                out.ikInfo = info;
                out.ikInfo.applied = true;
                for (int f = 0; f < 5; ++f) {
                    out.fingerIkRmseMm[size_t(f)] = info.rmseMm[size_t(f)];
                    // 残差门限：拟合不上说明 anchor/骨长跟这只手对不上，或者
                    // 标签串了。这种时候 IK 的解不比几何法可信，别覆盖。
                    const bool okRms = !(info.rmseMm[size_t(f)] > cfg_.ikMaxRmseMm);
                    if (!info.fingerValid[size_t(f)] || !okRms) continue;
                    out.fingerIkValid[size_t(f)] = true;
                    out.ikApplied = true;
                    for (int j = 0; j < 3; ++j) {
                        const int m = 5 + 3*f + j;
                        const int s = 1 + 3*f + j;
                        // 【只取骨轴，roll 仍走 boneFrame】
                        // IK 的 segR 是【解剖学】框架（roll 由 MCP 外展+屈曲的
                        // 运动学模型隐含给出），而几何法的 roll 是【手背法线】
                        // 参考系。两套约定的 roll 相差可以到 180°，于是同一根
                        // 手指在"这一帧走 IK、下一帧退回几何"时四元数整个翻过去
                        // —— 实测分段帧间跳变 p99 从 50° 冲到 178°。
                        // 而绕骨轴的自转在每节只有 1 个 marker 时【本来就不可
                        // 观测】，两套 roll 谁也不比谁更真。所以统一用 boneFrame，
                        // 换来的是全程同一个约定、切换源时不跳。
                        const Mat3 Rik = quatToMat(ikQuat[size_t(s)]);
                        const Vec3 axis{Rik[0], Rik[3], Rik[6]};   // 列 0 = 骨轴
                        if (cfg_.ikOverrideQuat) {
                            // 【必须跟几何路径用同一套 roll 约定】上面那段注释记的
                            // 教训在 chainRoll 引入之后又活了一次：几何走 swingTo、
                            // IK 走 boneFrame 的话，同一根手指在"这一帧走 IK、
                            // 下一帧退回几何"时四元数会跳。实测外展 30° 时两套
                            // 约定差 29.6°（近节 16.5°、中节 20.9°、远节 29.6°）。
                            //
                            // 链式模式下父节点取【本帧已经算好的】父段坐标系：
                            // 段号 1/4/7/10/13 是各指第一节，父是腕部；其余父是前一段。
                            // 那些父段在本轮循环里已经写好了（s 从小到大遍历）。
                            const Mat3 parent =
                                (!cfg_.chainRoll) ? Mat3{}
                                : ((s % 3 == 1) ? out.wristR
                                                : quatToMat(out.segQuat[size_t(s - 1)]));
                            out.segQuat[size_t(s)] = cfg_.chainRoll
                                ? matToQuat(swingTo(parent, axis,
                                        (s % 3 == 1) ? segRollOffset(s, cfg_.thumbRollOffsetRad)
                                                     : 0.0))
                                : matToQuat(boneFrame(axis, out.wristR, prevSegY_[size_t(s)],
                                                    segRollOffset(s, cfg_.thumbRollOffsetRad)));
                            out.segSource[size_t(s)] = SegSource::Ik;
                            out.segConf[size_t(s)] = 1.0;
                        }
                        if (cfg_.ikFillOccluded && !ob[size_t(m)]) {
                            out.markers[size_t(m)].posWorld = mk[size_t(m)];
                            ++out.ikFilledMarkers;
                            ikFilledThisFrame_[size_t(m)] = true;
                        }
                    }
                }
            }
        }

        // ---- 网络原始预测：无条件保存 ----
        // 【为什么不能只在录制时存】continueChainForOccluded 的融合路径要用
        // out.netPos 当方向来源（见那里的说明：网络给方向、链式给连续性）。
        // 而 netPos 原来只在 captureDebugStreams() 里赋值，那个函数又只在
        // cfg_.captureDebugStreams 为真时调用 —— 也就是【只有录制时才有值】。
        // 不录制时 netPos 全是零向量，vN 退化，融合分支被 vPn>1e-6 挡掉，
        // 静默退回旧的耦合推算。
        //
        // 这种"开了录制才正常"的失效最难查：录下来的数据里融合是生效的，
        // 而用户平时用的时候不生效，两边看到的是两套行为。
        // 拷贝 20 个 Vec3 的成本可以忽略，无条件存。
        out.netPos     = inf.pos;

        // ---- 6a2. 离线复算用的原始流 ----
        if (cfg_.captureDebugStreams) captureDebugStreams(out, inf, candidates, N);

        // ---- v5 快照：第 3 级（IK 精修之后）----
        // 【stageValid[3] 取决于 IK 有没有真跑】IK 关着、或者腕部位姿无效导致
        // 整块被跳过时，这一级是第 2 级的拷贝。把这两种情况混起来的话，
        // "开了 IK 为什么没反应"就答不上来 —— 而这是排查时问得最多的一句。
        if (cfg_.captureDebugStreams) {
            for (int m = 0; m < kNumMarkers; ++m)
                out.stagePos[3][size_t(m)] = out.markers[size_t(m)].posWorld;
            out.stageValid[3] = (ik_ != nullptr && out.wristPoseValid);
        }

        // ---- 6b. 拇指段的网络兜底（默认关，见 cfg_.thumbSegFromNet）----
        if (cfg_.netSegCalib) segAlign_.feed(out, inf);
        applyNetThumbSegments(out, inf);

        // 【退档要可见】遮挡点走 IK 还是退回 AI 预测，原来是隐式的：某根手指
        // fingerValid=false 或残差超门限时会静默退档，而两者的误差特性差很远
        // (IK 骨轴 max 71° vs AI 108°)。不上报的话，"这一帧到底用的哪条路"
        // 只能靠猜 —— 而这正是排查时最需要知道的。
        out.ikFallbackMarkers = 0;
        for (int m = 5; m < kNumMarkers; ++m)
            if (!out.markers[size_t(m)].observed) ++out.ikFallbackMarkers;
        out.ikFallbackMarkers -= out.ikFilledMarkers;
        if (out.ikFallbackMarkers < 0) out.ikFallbackMarkers = 0;

        // 【链式续解】必须在"保持"之前：能顺着可见部分算出来的，就不该冻住。
        // 几何手性：只在弯折充分的帧累积，够了就锁定，之后遮挡再多也不重判。
        accumulateGeoHandedness(out, seen);
        out.geoHandSign = geoHandLocked_;

        // 【先学骨长，再用】学习独立于 chainContinue 开关，见 learnBoneLengths。
        learnBoneLengths(out, seen);
        learnJointPose(out, seen);      // 可见时记角度和平面，遮挡时才有得用
        continueChainForOccluded(out, seen);

        // 遮挡点只信 IK：IK 没补上的，保持上一帧位置（而不是网络预测）。
        // 【放在统计之后】这样 ikFallbackMarkers 记录的仍然是"IK 没补上几个"，
        // 不会因为改成保持就把这个数抹掉 —— 那个数是排查退档用的。
        for (int m = 5; m < kNumMarkers; ++m) {
            const bool obs = out.markers[size_t(m)].observed;
            const bool solved = ikFilledThisFrame_[size_t(m)] || chainFilledThisFrame_[size_t(m)];
            if (!obs && !solved && hasPrevMarker_[size_t(m)]) {
                // 走到这里 = 只剩网络 pos 头的预测。默认【采纳】它 —— 实测它对
                // 四指外推是好用的。只有两种情况才改用保持：
                //   ① occludedIkOnly 打开（用户显式要求只信 IK）
                //   ② 这一帧跳得离谱（发散），此时保持是安全网
                const double jump = norm(sub(out.markers[size_t(m)].posWorld,
                                             prevMarkerPos_[size_t(m)]));
                const bool diverged = (cfg_.occludedJumpGateMm > 0.0
                                       && jump > cfg_.occludedJumpGateMm);
                if (cfg_.occludedIkOnly) {
                    out.markers[size_t(m)].posWorld = prevMarkerPos_[size_t(m)];
                    ++out.occludedHeld;
                    heldThisFrame_[size_t(m)] = true;
                } else if (diverged) {
                    // 【限速，不是定住】定住的点是不可用的：它既不跟手，又看不出
                    // 自己已经不跟手了。改成把这一帧的位移【截断到门限】——
                    // 点仍然朝正确方向走，只是走不快，下一帧继续走。
                    // 发散是暂态，限速能让它自己爬回来；定住则会一直卡在原地。
                    const Vec3 d = sub(out.markers[size_t(m)].posWorld, prevMarkerPos_[size_t(m)]);
                    const double n = norm(d);
                    if (n > 1e-9) {
                        const double k = cfg_.occludedJumpGateMm / n;
                        out.markers[size_t(m)].posWorld = add(prevMarkerPos_[size_t(m)], mul(d, k));
                    }
                    ++out.occludedHeld;
                    // 【跟"保持"分开记】限速是"朝对的方向走得慢"，保持是"完全不动"。
                    // 两者都会让 occludedHeld 涨，但前者会自己爬回来、后者不会 ——
                    // 看到一段跟不上手时，这两种的处置完全相反。
                    jumpLimitedThisFrame_[size_t(m)] = true;
                }
            }
            prevMarkerPos_[size_t(m)] = out.markers[size_t(m)].posWorld;
            hasPrevMarker_[size_t(m)] = true;
        }

        // 【顺序要紧，第一版我放错了】这两步必须在上面那段"发散限速/保持"
        // 【之后】、prevMarkerPos_ 回填【之前】。
        //
        // 放在前面的后果实测过：先回正骨长、再平滑，然后那段限速逻辑又拿
        // 【未回正的网络原始值】去判发散并覆写位置，把前面做的全冲掉 ——
        // 骨长从 25mm 变成 71mm，转遮挡那一帧跳 29.76mm。
        //
        // 放在回填之后同样不行：平滑后的值不会成为下一帧的参考，滤波器等于没接上。
        //
        // 【2026-08 改：两者内部换序，平滑在前、回正在后】
        // 平滑是【逐点独立】的：转遮挡的台阶补偿给每个 marker 各加一个自己的
        // 偏移向量，而 mp 和 dp 的偏移不一样 —— 于是刚回正好的 mp-dp 间距
        // 又被这两个不同的向量差拆开了。实测 test_predict_sanity：
        // 骨长离散度从 11.9% 涨到 44.4%。
        // 回正放在最后，骨长就有了最终话语权：它只动【未观测】的子点、
        // 只改到父点的距离、不改方向，所以台阶补偿的切向分量（占大头）
        // 保留下来，而径向分量让位给骨长约束。两个都要的话只能这么排。
        smoothPredicted(out, seen);
        snapToBoneLength(out, seen);

        // ---- v5 快照：第 4 级（预测点平滑 + 骨长回正 + 发散限速之后）----
        // 这是关联器的最终输出，也是【送进滤波器的输入】。worker 会在滤波
        // 之后补上第 5 级 —— 两者之差就是滤波器干的事，而滤波是就地覆写的，
        // 不在这里留一份的话，滤波前的值在整个系统里就不存在第二份了。
        if (cfg_.captureDebugStreams) {
            for (int m = 0; m < kNumMarkers; ++m)
                out.stagePos[4][size_t(m)] = out.markers[size_t(m)].posWorld;
            out.stageValid[4] = true;
        }
        for (int m = 5; m < kNumMarkers; ++m)
            if (!out.markers[size_t(m)].observed)
                prevMarkerPos_[size_t(m)] = out.markers[size_t(m)].posWorld;

        storeSegY(out, prevSegY_);

        // ---- 7. 四元数半球连续性 ----
        // matToQuat 强制 w>=0。q 和 -q 是同一个旋转，但对下游【任何】做四元数
        // 插值、差分、写文件的消费者来说，w 跨过 0 时符号整体翻转就是一次
        // 无中生有的 180° 跳变。这里统一跟上一帧取同半球，代价为零。
        alignQuatHemisphere(out);

        // ---- 8. 手背五边形校验（正常情况恒成立，不成立说明这组点摆得病态）----
        out.pentagonOk = checkPentagon(out);

        // ---- 9. 历史帧已在第 6 步之前存过 ----
        // 【不要在这里再存一次】那样存的是后处理【之后】的位置，会把姿态侧的
        // 假设回灌给模型。解耦的关键就在于此，见上面那段说明。

        // ---- 10. 逐 marker 来源归档 ----
        // 【放在最后、集中在一处】前面每一步都在改点的位置，散着记必然漏。
        // 优先级从"最可信"往下排：实测 > IK > 链式续解 > 保持 > 网络兜底。
        // 【顺序不能改】一个点可能同时满足多条（比如 IK 补完又被骨长回正过），
        // 主来源取最可信的那个，其余的进 flags —— 两者混成一个字段的话，
        // "这个位置本质上是哪来的"就答不上来了。
        for (int m = 0; m < kNumMarkers; ++m) {
            const size_t um = size_t(m);
            uint8_t src;
            if (out.markers[um].observed)          src = 1;   // 实测
            else if (ikFilledThisFrame_[um])       src = 2;   // IK 补
            else if (chainFilledThisFrame_[um])    src = 3;   // 链式续解
            else if (heldThisFrame_[um])           src = 5;   // 保持上一帧
            else                                   src = 4;   // 网络 pos 头兜底
            out.markerSource[um] = src;
            uint8_t fl = 0;
            if (snappedThisFrame_[um])     fl |= 0x01;
            if (!out.markers[um].observed) fl |= 0x02;   // 未观测点才进平滑
            if (jumpLimitedThisFrame_[um]) fl |= 0x04;
            if (thumbFixedThisFrame_[um])  fl |= 0x08;
            out.markerFlags[um] = fl;
        }

        // ---- 11. 不变量：一个候选点只能有一个主人 ----------------------------
        // 【为什么要有这一段】匈牙利指派保证一对一，但它之后还有五六步会改
        // sourcePointId（手背刚体重定、tracklet 投票、几何重标、贴点重捕）。
        // 其中任何一步只要"认领了新点却没通知旧主人松手"，就会出现两个标签
        // 挂同一个 id —— 而这件事在每一级单看都完全正常：位置有、编号有、
        // observed=true、置信度 1.0。只有【横着比】才看得出来。
        // 真机实测 143/621 帧中招，一直没被发现，就是因为没有任何一处横着比过。
        // 这里只统计不修（修在各自的产生点上），让它在诊断里露头。
        out.dupClaimCount = 0;
        for (int a = 0; a < kNumMarkers; ++a) {
            const int ia = out.markers[size_t(a)].sourcePointId;
            if (ia < 0 || !out.markers[size_t(a)].observed) continue;
            for (int b = a + 1; b < kNumMarkers; ++b) {
                if (!out.markers[size_t(b)].observed) continue;
                if (out.markers[size_t(b)].sourcePointId == ia) ++out.dupClaimCount;
            }
        }
        if (out.dupClaimCount > 0)
            out.message = "标签重号：有候选点被两个标签同时认领";

        out.valid = true;
        if (out.dupClaimCount == 0) out.message = "ok";
        return out;
    }
private:

    // 每个分段的姿态是"测的"还是"猜的"。规则跟 computeSegmentQuats 的取向源
    // 一一对应：某段的朝向由哪两个点的差分决定，就看那两个点是不是真观测。
    //   近节 prox: (anchor 或 pp) -> mp        anchor 来自模板，只要 wristPoseValid 就算已知
    //   中节 mid : pp -> dp
    //   远节 dist: mp -> dp
    // -------------------------------------------------------------------------
    // 拇指三段改用网络 seg_rot6d。
    //
    // 【只在几何和 IK 都不可信的段上生效】拇指三颗球全见时几何解是好的，
    // IK 合格时 IK 更好；这条路是给"拇指被遮住"那 55% 的帧兜底的。
    //
    // 【为什么它比现在的 Predicted 好】Predicted 段的骨轴 = 两个【网络补出来的
    // 点】之差。补点的误差被骨长(20~30mm)一除就放大成角度误差。seg_rot6d 直接
    // 给姿态，不需要那两个点，误差不经过这一步放大。
    //
    // 【为什么还需要 K_s】seg_rot6d 跟 pos 头训练自同一个 rig，而那个 rig 的
    // 拇指缺了常数旋前(实测铰链轴只偏 17.4°，真人要 80~90°)。所以网络给的拇指
    // 姿态本身也是外翻的，需要一个常数修正。好消息是这个常数可以从【拇指三球
    // 全见的帧】上量出来(见 Hm20SegRot.hpp 的 SegCanonAligner)，手性自动正确。
    //
    // 【统一走 boneFrame】跟 IK 覆盖那段同样的理由：不同来源各自的 roll 约定
    // 不一样，混着用会让手指在"这一帧走网络、下一帧退回几何"时四元数整个翻过去。
    // 而 roll 在每节只有 1 颗球时本来就不可观测，两套谁也不比谁更真。
    // -------------------------------------------------------------------------
    // 把这一帧的网络输入/输出原样拷进 result，供录制层落盘。
    // 【只拷不算】这里刻意不做任何加工 —— 一旦在这里"顺手整理一下"，
    // 落盘的就不再是网络真正吐出来的东西，离线对拍就失去意义了。
    void captureDebugStreams(SkeletonFrameResult& out, const InferenceOutput& inf,
                             const std::vector<std::pair<int, Vec3>>& candidates,
                             int N) const {
        constexpr int kMaxCap = 32;
        out.hasDebugStreams = true;
        out.assocInputBeforeCap = int(candidates.size());
        const int n = std::min(N, kMaxCap);
        out.assocInputN = n;
        for (int i = 0; i < n; ++i) {
            out.assocInputPos[size_t(i)] = candidates[size_t(i)].second;
            out.assocInputId[size_t(i)]  = candidates[size_t(i)].first;
        }

        // log_assign 是 log-softmax，取 top-3 之后 exp 回概率
        if (inf.rows > 0 && inf.cols >= kNumClasses) {
            for (int i = 0; i < n && i < inf.rows; ++i) {
                std::array<int, 3> bl{{-1, -1, -1}};
                std::array<double, 3> bp{{-1e18, -1e18, -1e18}};
                for (int c = 0; c < kNumClasses; ++c) {
                    const double v = double(inf.logAssign[size_t(i) * size_t(inf.cols) + size_t(c)]);
                    if (v > bp[0])      { bp[2]=bp[1]; bl[2]=bl[1]; bp[1]=bp[0]; bl[1]=bl[0]; bp[0]=v; bl[0]=c; }
                    else if (v > bp[1]) { bp[2]=bp[1]; bl[2]=bl[1]; bp[1]=v; bl[1]=c; }
                    else if (v > bp[2]) { bp[2]=v; bl[2]=c; }
                }
                for (int k = 0; k < 3; ++k)
                    bp[size_t(k)] = (bl[size_t(k)] >= 0) ? std::exp(bp[size_t(k)]) : 0.0;
                out.assignTop3Label[size_t(i)] = bl;
                out.assignTop3Prob[size_t(i)]  = bp;
            }
        }

        out.netPos       = inf.pos;
        out.netCenter    = inf.center;
        out.netScale     = inf.scale;
        out.netMissLogit = inf.missLogit;
        out.netJointAng  = inf.jointAng;
        out.netHandLogit = inf.handLogit;
        out.netPoseConf  = inf.poseConf;
        out.netSegR      = inf.segR;
        out.netHasSegR   = inf.hasSegR;

        // ---- 全量指派矩阵 ----
        // 【为什么 top-3 不够】"这个点为什么被判成鬼点"是排查时的高频问题，
        // 而答案在【分布的形状】里：分布平坦说明模型没主意（该去看点质量和
        // 标定），次高紧贴最高说明是被邻近标签抢走了（该去看阈值和指派）。
        // top-3 只有三个数，看不出形状。
        // 单独开关，因为 120fps 下约 200KB/s —— 常态录制不该付这个代价。
        if (cfg_.captureFullAssign && inf.rows > 0 && inf.cols > 0) {
            out.logAssignRows = std::min(inf.rows, kMaxCap);
            out.logAssignCols = inf.cols;
            out.logAssignFull.assign(
                inf.logAssign.begin(),
                inf.logAssign.begin() + size_t(out.logAssignRows) * size_t(inf.cols));
        }
    }

    static void markSegmentSources(SkeletonFrameResult& out) {
        out.segSource[0] = out.wristPoseValid ? SegSource::Geometry : SegSource::None;
        out.segConf[0]   = out.wristPoseValid ? 1.0 : 0.0;
        for (int f = 0; f < 5; ++f) {
            const int pp = 5 + f*3, mp = pp + 1, dp = pp + 2;
            const bool oPp = out.markers[size_t(pp)].observed;
            const bool oMp = out.markers[size_t(mp)].observed;
            const bool oDp = out.markers[size_t(dp)].observed;
            const bool anchorOk = out.wristPoseValid;
            // 近节：拇指恒用 pp->mp（CMC 旋前会把 anchor 向量带偏，见下方说明）
            const bool proxMeasured = (f == 0) ? (oPp && oMp)
                                              : ((anchorOk && oPp) || (oPp && oMp));
            auto set = [&](int s, bool measured) {
                out.segSource[size_t(s)] = measured ? SegSource::Geometry : SegSource::Predicted;
                out.segConf[size_t(s)]   = measured ? 0.7 : 0.2;
            };
            set(1 + f*3 + 0, proxMeasured);
            set(1 + f*3 + 1, oPp && oDp);
            set(1 + f*3 + 2, oMp && oDp);
        }
    }
