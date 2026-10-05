// ===========================================================================
// Hm20Assoc_api.ipp —— Hm20SkeletonAssociator 类体分片：对外接口 + 编排状态 + 整体复位
//
// 构造、setter/getter、configJson、resetHistory/resetDorsumRigid。
// 【resetDorsumRigid 天然横跨所有组】它要把每一组的状态清干净，
// 所以它放这里而不是任何单一职责组里。新增状态时记得在它里面加一行，
// 漏了的表现是"换手/重捕之后行为受上一次残留影响"，很难查。
//
// 【本文件被 #include 在 class 体内部】内容即类内定义（隐式 inline）。
// 不要加 include guard、不要加 namespace。开头的 access 说明符是显式写死的，
// 不依赖上一个分片以什么结尾——调换 include 顺序不会改变可见性。
// 独立语法检查：见 tests/test_assoc_fragments.cpp
// ===========================================================================
private:

    std::shared_ptr<IHm20InferenceBackend> backend_;
    std::shared_ptr<IHm20IkRefiner> ik_;
    std::array<float, 61> tmpl_{};
    bool tmplValid_ = false;
    Config cfg_;

    std::array<Vec3, 20> prevPos_{};
    std::array<bool, 20> prevMask_{};
    bool hasPrev_ = false;

    Mat3 lastWristR_{1,0,0, 0,1,0, 0,0,1};
    Vec3 lastWristT_{};
    bool hasLastWrist_ = false;
    // mutable：这三个是帧间缓存，process() 里更新；跟 const 成员函数无关的状态。
    mutable std::array<Vec3, kNumMarkers> prevMarkerPos_{};
    mutable std::array<bool, kNumMarkers> hasPrevMarker_{};
    mutable std::array<bool, kNumMarkers> ikFilledThisFrame_{};
    // 【跟上面两个是一套】有了 ikFilled/chainFilled 还差三条路才能把一个点的
    // 来源说全：保持上一帧、骨长回正、发散限速。少任何一条，markerSource 就
    // 会把它归进"网络预测兜底"，而那三种的误差特性跟网络预测完全不同。
    mutable std::array<bool, kNumMarkers> heldThisFrame_{};
    mutable std::array<bool, kNumMarkers> jumpLimitedThisFrame_{};
    // 手性投票状态
    bool   handVote_ = true;
    int    handVoteN_ = 0;
    // 手性投票的滑动累积（置信度加权）。见 handLogit 那段的说明。
    // 时间常数 1/(1-0.92) ≈ 12 帧：单帧翻转扛得住，真换手半秒内能跟上。
    double handVoteAcc_ = 0.0;
    // 模型平均置信度（跟累加器同一个时间常数，好直接比）
    double handConfAvg_ = 0.0;
    // 证据够不够下结论。false = 【未定】，不是"左"也不是"右"。
    // 没有这个状态就只能二选一，而在 |logit|~0.044 的量级上二选一就是抛硬币。
    bool   handKnown_ = false;
    bool   aiHandKnown_ = false;
    int    geoHandLocked_ = 0;      // 0=未锁  +1/-1=锁定值
    // 法向同向化的参照：本会话第一帧的手背法向。只要求会话内一致，
    // 绝对朝向由弯折方向定。见 accumulateGeoHandedness 的说明。
    Vec3   geoRef_{};
    double aiHandConf_ = 0.0;
    bool   aiHandLocked_ = false;
    bool   aiHandLockedValue_ = true;
    bool wristPoseValid_ = false;
public:
// ===========================================================================
// Hm20SkeletonAssociator_impl1.ipp —— Hm20SkeletonAssociator 类体分片 1/3。
// 【重要】本文件被 #include 在 class Hm20SkeletonAssociator { ... } 的类体内部，
// 内容即“类内定义”（隐式 inline），与拆分前完全等价。不要加 include guard / 命名空间。
// 原 HandSkeletonAssociator.hpp 第 1382~2405 行（public API 区）。
// ===========================================================================

    explicit Hm20SkeletonAssociator(std::shared_ptr<IHm20InferenceBackend> backend,
                                    std::array<float, 61> tmpl = {}, bool tmplValid = false,
                                    Config cfg = Config())
        : backend_(std::move(backend)), tmpl_(tmpl), tmplValid_(tmplValid), cfg_(cfg) {}

    void setIkRefiner(std::shared_ptr<IHm20IkRefiner> ik) { ik_ = std::move(ik); }

    // 【只换推理后端，不动任何学习状态】
    // backend 是无状态的：每帧独立推理，不跨帧携带东西。所以切换模型
    // （比如 FP32 <-> INT8 高速模式）只需要换这个指针。
    //
    // 原来 worker 的 ensureInitialized 里是整个 make_shared 一个新关联器，
    // 那会把手背重捕修好的模板、连续性锁、拇指跟随器学到的弯曲轴、
    // dipK_/mdLen_ 全部抹掉 —— 跟几轮前查出来的"自标定推模板导致重捕全丢"
    // 是同一个坑，换模型时又会踩一次。
    void setBackend(std::shared_ptr<IHm20InferenceBackend> b) { backend_ = std::move(b); }
    // 拇指三段的 roll 偏置，运行时可调（每帧读一次，不需要重建关联器）。
    // 见 Hm20Config::thumbRollOffsetRad 的说明。
    void setThumbRollOffset(double rad) { cfg_.thumbRollOffsetRad = rad; }
    double thumbRollOffset() const { return cfg_.thumbRollOffsetRad; }
    // 拇指预测点的旋前回正量。跟上面是同一个物理量、作用在不同输出上。
    void setOccludedIkOnly(bool on) { cfg_.occludedIkOnly = on; }
    void setChainContinue(bool on) { cfg_.chainContinue = on; }
    bool chainContinue() const { return cfg_.chainContinue; }
    void setDorsumTracklet(bool on) { cfg_.dorsumTracklet = on; }
    void setDorsumGeoRelabel(bool on) { cfg_.dorsumGeoRelabel = on; }
    void resetDorsumTracklet() { trkInit_ = false; trkFrames_ = 0; }
    void setDorsumRigidSolve(bool on) { cfg_.dorsumRigidSolve = on; }
    void setThumbSegFromNet(bool on) { cfg_.thumbSegFromNet = on; }
    void setCaptureDebugStreams(bool on) { cfg_.captureDebugStreams = on; }
    void setCaptureFullAssign(bool on) { cfg_.captureFullAssign = on; }
    // 注入 Hm20SegRot.hpp 里 SegCanonAligner 标定出来的两个常数。
    // 没注入之前 thumbSegFromNet 即使打开也不会生效（netConstValid_ 为假）。
    void setNetSegConstants(const Mat3& C, const std::array<Mat3, 3>& K) {
        netC_ = C; netK_ = K; netConstValid_ = true;
    }
    // 【等价于"重开点云测试面板"，但只复位手背求解器】
    // 求解器有三个"进去出不来"的状态：连续性锁 prevR_、被重捕改过的模板、
    // 以及"求解失败 -> 不能重捕 -> 一直失败"的死锁。原来唯一的出口是重建
    // 整个关联器（= 重开面板），代价太大而且会连带丢掉自标定和 IK 的状态。
    // 【等价于"重开点云测试面板"，但保留标定】
    //
    // 上一版只清了手背求解器的历史和模板，实测无效 —— 因为漏掉了最关键的一个：
    // prevPos_/prevMask_ 是【喂回网络的上一帧标签先验】。标签一旦乱掉，
    // 乱的先验每帧都被送回网络，把它往错的方向拽，形成自我维持的循环。
    // 不清它，光清几何那边没用，所以还是得重开面板。
    //
    // 这里把【所有跨帧携带的状态】一次清干净，标定参数(模板/anchor/骨长)保留。
    void resetDorsumRigid() {
        // ① 网络的时序先验 —— 自我维持错误的根源，必须清
        prevPos_ = {};
        prevMask_ = {};
        hasPrev_ = false;
        // ② 腕部位姿历史（否则解不出时会沿用一个错的）
        hasLastWrist_ = false;
        lastWristR_ = Mat3{1,0,0, 0,1,0, 0,0,1};
        lastWristT_ = Vec3{};
        // ③ 各段/各指的连续性状态
        prevSegY_ = {};
        // 手性证据：重捕之后标签可能整个换了点，之前累积的投票不再对应同一只手。
        // 【锁也要清】原来这里写的是 `if (!aiHandLocked_) {...}` —— 锁一旦扣上
        // 就没有任何出口，连用户手动点复位都清不掉。而锁是可能扣错的
        // （见 handConfFloor 那段：地板设错时会锁死在右手）。
        // 复位本来就是用户在说"现在不对，从头来"，那就必须包括这个锁。
        aiHandLocked_ = false;
        aiHandLockedValue_ = false;
        handVoteAcc_ = 0.0;
        handConfAvg_ = 0.0;
        handKnown_ = false;
        handVoteN_ = 0;
        aiHandKnown_ = false;
        handVote_ = true;
        aiHandConf_ = 0.0;
        planeLocal_ = {};
        hasPlane_ = {};
        planeSeeded_ = {};
        // 【累加器必须跟 planeLocal_ 一起清】漏了它的后果不是"少清一个"，
        // 是【复位当场失效】：planeLocal_ 清成零，但下一个三点全见的帧
        // learnJointPose 会立刻从 planeAcc_ 里把上一只手/上一套标签学到的
        // 平面重新归一化写回去 —— 用户看到的就是"点了复位，一帧之后又歪回去"。
        // 判据还是这个文件开头那条：新建一个关联器时它是什么，复位后就该是什么。
        planeAcc_ = {};
        hasPlaneAcc_ = {};
        planeSampleN_ = {};
        // anchor 的出平面投票同理：它记的是"这根 anchor 在【那套标签下】
        // 偏出平面多少"，标签换了就不再对应同一件事。
        anchorPlaneN_ = {};
        anchorPlaneOk_ = {};
        // 【PIP 平面跟 MCP 平面一起清】理由同下：重捕之后标签可能整个换了点，
        // 留着旧平面会把手指摆到错的姿势上。
        pipPlaneLocal_ = {};
        hasPipPlane_ = {};
        // 相位是跨帧累积的时序状态，重捕后标签可能整个换了点，必须清。
        dipPhase_ = {};
        hasDipPhase_ = {};
        // 【新增，必须一起清】遮挡入口参照和腕部系骨轴都是"基于当时那套标签
        // 学到的"，重捕之后标签可能整个换了点，留着会把手指摆到错的姿势上。
        prevU0Local_ = {};
        hasPrevU0_ = {};
        u0Entry_ = {};
        pipEntry_ = {};
        hasEntry_ = {};
        pEntryLocal_ = {};
        flexDirEntry_ = {};
        hasPEntry_ = {};
        caseBFrames_ = {};
        coupleExc_ = {};
        hasExc_ = {};
        // ④ 拇指遮挡跟随（弯曲轴和同步点都可能是基于错标签学的）
        thumbTrk_.reset();
        hasThumbIp_ = false;
        thumbIpAng_ = 0.0;
        // ⑤ 已停用的 tracklet，一并清掉免得留脏数据
        trkInit_ = false;
        trkHas_ = {};
        trkPos_ = {};
        trkVote_ = {};
        trkFrames_ = 0;
        // ⑥ 【第二版补齐：下面这些原来全漏了，所以"手背复位"等价不了重开面板】
        //
        // 判断标准很简单：新建一个关联器时这个成员是什么，复位后就该是什么。
        // 按这个标准逐条对了一遍，漏的都在这儿。少一条，就会出现"复位了还是
        // 不对、必须重开面板"——因为那一条把旧标签下学到的东西带了过来。
        //
        // prevMarkerPos_ 是【发散限速/保持】的参照点：标签乱掉之后它停在错的
        // 位置上，而限速逻辑每帧拿它去夹当前解，等于把点钉死在错处修不回来。
        // 这条是"连线乱了一直修不回来"最直接的一条。
        prevMarkerPos_ = {};
        hasPrevMarker_ = {};
        // pipHold_/dipK_ 是【姿态】量不是标定量：在错标签下学到的关节角和
        // dip/pip 耦合比，会经 pipEntry_ 直接进遮挡重建。
        // 注意 pmLen_/mdLen_（骨长）【故意保留】——那才是标定量。
        pipHold_ = {};
        hasPipHold_ = {};
        dipK_ = {};
        // 时域平滑：smoothHas_ 为真而 smoothPos_ 停在旧位置时，
        // 复位后第一个预测帧会从那个旧位置起跳。
        smoothPos_ = {};
        smoothHas_ = {};
        seen0_ = {};
        enterOff_ = {};
        // 分段四元数的连续性（半球对齐）：旧四元数会把新解拉到相反半球去。
        prevSegQuat_ = {};
        prevWristQuat_ = Quat{1, 0, 0, 0};
        hasPrevQuat_ = false;
        // 拇指 IP 轴（hasThumbIp_ 已经清了，这里一并归零免得留残值）
        thumbIpAxisLocal_ = Vec3{};
        // 几何手性的累积（会话内一次性锁定，标签换了就不再对应同一只手）
        geoHandAcc_ = 0.0;
        geoHandLocked_ = 0;
        geoRef_ = Vec3{};
        geoRefValid_ = false;
        wristPoseValid_ = false;
        // anchor 自检要重做：模板可能刚被换过，而自检是拿【实际观测】比的，
        // 不重做的话旧结论会跟着新模板一起用。
        anchorsChecked_ = false;
        // 【票也要清，不然"重做"是假的】自检改成多帧投票之后，只清
        // anchorsChecked_ 会让下一帧的 verifyAnchors 拿着【旧标签下攒的票】
        // 立刻凑够 kAnchorVoteMin 并当场下结论 —— 看起来重判了，实际用的
        // 全是复位前的证据，一帧新数据都没吃。
        anchorVoteN_ = {};
        anchorVoteOk_ = {};
        anchorOk_ = {};
        anchorVoteFrames_ = 0;
        // ⑦ 手背刚体求解器
        dorsum_.resetHistory();
        dorsumPoseOk_ = false;
        // 模板也退回标定值：重捕可能把它改坏了，而原来那条路只在
        // templateValid() 为假时才重设，等于永远不会再设。
        if (tmplMmValid_) {
            std::array<Vec3, 5> T{};
            for (int m = 0; m < 5; ++m) T[size_t(m)] = tmplMm_[size_t(m)];
            dorsum_.setTemplate(T);
            std::array<Vec3, 15> F{};
            for (int m = 0; m < 15; ++m) F[size_t(m)] = tmplMm_[size_t(m + 5)];
            dorsum_.setFingerTemplate(F);
        }
    }

    // -------------------------------------------------------------------------
    // 把【全部】运行时状态导成 JSON，给 .pcrec 文件头用。
    //
    // 【为什么必须逐个列、不能挑着写】录制文件是事后唯一的凭据。漏掉一个开关，
    // 一个月后就说不清"这段数据当时那个开关开没开"，而调参结论全建立在这上面。
    // 现在头里【一个 hm20 参数都没有】—— 拇指旋前多少、手背用的哪条路、
    // IK 残差门多少，全都查不到，等于录了一堆没法归因的数据。
    //
    // 纯 std::string，不依赖 Qt：这样它能被单元测试直接编译和验证，
    // 而 UI 那边只要一行 QJsonDocument::fromJson 就能嵌进文件头。
    // -------------------------------------------------------------------------
    std::string configJson() const {
        std::string o = "{";
        auto b = [&](const char* k, bool v) {
            o += "\""; o += k; o += "\":"; o += (v ? "true" : "false"); o += ","; };
        auto d = [&](const char* k, double v) {
            char buf[64]; std::snprintf(buf, sizeof(buf), "%.6g", v);
            o += "\""; o += k; o += "\":"; o += buf; o += ","; };
        auto i = [&](const char* k, long long v) {
            o += "\""; o += k; o += "\":"; o += std::to_string(v); o += ","; };

        // ---- 指派/门限 ----
        d("minAssignProbDorsum", cfg_.minAssignProbDorsum);
        d("minAssignProbFinger", cfg_.minAssignProbFinger);
        b("useTemplate",         cfg_.useTemplate);
        b("usePrevFrame",        cfg_.usePrevFrame);
        // ---- 手背 ----
        b("dorsumRigidSolve",    cfg_.dorsumRigidSolve);
        b("dorsumGeoRelabel",    cfg_.dorsumGeoRelabel);
        b("dorsumTracklet",      cfg_.dorsumTracklet);
        d("dorsumGeoMinMarginMm",cfg_.dorsumGeoMinMarginMm);
        d("dorsumTrackGateMm",   cfg_.dorsumTrackGateMm);
        i("dorsumTrackWarm",     cfg_.dorsumTrackWarm);
        // 【这个数决定这只手能不能被稳定定标签】<3mm = 贴点布局近似共面+近似
        // 反序对称，几何判据本身分不清正解和反序。录进去才能事后归因。
        d("dorsumSelfAmbiguityMm", dorsum_.selfAmbiguityMm());
        // ---- 拇指 ----
        d("thumbPronationRad",   cfg_.thumbPronationRad);
        d("thumbRollOffsetRad",  cfg_.thumbRollOffsetRad);
        b("thumbSegFromNet",     cfg_.thumbSegFromNet);
        b("netSegCalib",         cfg_.netSegCalib);
        b("netSegConstValid",    netConstValid_);
        // ---- 分段/IK ----
        i("midDirMode",          int(cfg_.midDirMode));
        b("ikOverrideQuat",      cfg_.ikOverrideQuat);
        b("ikFillOccluded",      cfg_.ikFillOccluded);
        d("ikMaxRmseMm",         cfg_.ikMaxRmseMm);
        b("occludedIkOnly",      cfg_.occludedIkOnly);
        d("occludedJumpGateMm",  cfg_.occludedJumpGateMm);
        b("chainContinue",       cfg_.chainContinue);
        b("chainRoll",           cfg_.chainRoll);
        d("predictSmoothAlpha",  cfg_.predictSmoothAlpha);
        b("useIkRefine",         ik_ != nullptr);
        // ---- 模板 ----
        b("tmplValid",           tmplValid_);
        b("tmplMmValid",         tmplMmValid_);
        b("anchorsValid",        anchorsValid_);
        b("wristPoseValid",      wristPoseValid_);
        o += "\"tmplMm\":[";
        for (int m = 0; m < kNumMarkers; ++m)
            for (int c = 0; c < 3; ++c) {
                char buf[48]; std::snprintf(buf, sizeof(buf), "%.4f", tmplMm_[size_t(m)][size_t(c)]);
                o += buf; if (m != kNumMarkers - 1 || c != 2) o += ",";
            }
        o += "],\"anchorsMm\":[";
        for (int f = 0; f < 5; ++f)
            for (int c = 0; c < 3; ++c) {
                char buf[48]; std::snprintf(buf, sizeof(buf), "%.4f", anchorsMm_[size_t(f)][size_t(c)]);
                o += buf; if (f != 4 || c != 2) o += ",";
            }
        o += "]}";
        return o;
    }
    void setThumbPronation(double rad) { cfg_.thumbPronationRad = rad; }
    double thumbPronation() const { return cfg_.thumbPronationRad; }
    void resetHistory() { hasPrev_ = false; }

    double geoHandAcc() const { return geoHandAcc_; }

private:
    // Kabsch 用的模板点。
    //
    // 【修复·单位】原实现直接读 tmpl_ 的前 60 维当模板点：
    //     return {tmpl_[m*3+0], tmpl_[m*3+1], tmpl_[m*3+2]};
    // 但 tmpl_ 是【送给 ONNX 的归一化模板】——packNormalized() 里整体除过
    // handScale（量级约 180mm）。而 markers[].posWorld 是毫米。拿归一化模板
    // 去和毫米观测做 Kabsch，两边差了约 180 倍的尺度：Kabsch 只解旋转+平移、
    // 不解尺度，于是旋转会被硬拧到一个使残差最小的方向上，解出来的 wristR
    // 基本是错的，且随手指姿态变化剧烈抖动。
    //
    // 症状很隐蔽：dorsumRmseMm 会是个很大的数，但没有任何地方检查它；
    // wristQuat 每帧乱跳，看起来像"标定不准"或"模型不行"。
    //
    // 所以毫米副本必须单独存一份。tmplMm_ 没设置时（tmplMmValid_==false）
    // 就不解腕部位姿——宁可不给，也不给一个尺度错了的。
    Vec3 tmplMarker(int m) const { return tmplMm_[size_t(m)]; }

public:
    // 设置 Kabsch 用的毫米模板。构造时传的 tmpl(61维) 是给网络的归一化版本，
    // 两者来源相同但单位不同，见 Hm20Template::markersMm / packNormalized()。
    // 【换模板不该重建整个关联器】送进网络的归一化模板只是两个成员，换它
    // 不需要 make_shared 一个新对象。而重建会把所有【学出来的】状态一起抹掉：
    // 手背重捕修好的模板、连续性锁、拇指 IP 跟随器学到的弯曲轴、dipK_、mdLen_…
    // 真机实测过一次：自标定在会话中途推了一份更差的模板（bundleRmse 20.7->31.3），
    // 触发重建，之前两次成功的手背重捕全部丢失，之后再没恢复。
    void setNormalizedTemplate(const std::array<float, 61>& t, bool valid) {
        tmpl_ = t;
        tmplValid_ = valid;
    }

    void setTemplateMm(const std::array<Vec3, kNumMarkers>& mm) {
        tmplMm_ = mm;
        tmplMmValid_ = true;
        // 【立刻喂给刚体求解器，不要等第一帧】否则录制开始时写文件头，
        // dorsumSelfAmbiguityMm 还是 -1 —— 而那个数正是事后归因"手背标签
        // 到底为什么翻"的第一依据，写进文件的时候必须已经算好。
        std::array<Vec3, 5> T{};
        for (int m = 0; m < 5; ++m) T[size_t(m)] = tmplMm_[size_t(m)];
        dorsum_.setTemplate(T);
        std::array<Vec3, 15> F{};
        for (int m = 0; m < 15; ++m) F[size_t(m)] = tmplMm_[size_t(m + 5)];
        dorsum_.setFingerTemplate(F);
        dorsum_.resetHistory();      // 模板换了，旧的位姿历史不再可比
    }
    // 5 个指根关节(MCP/CMC)在腕部系的位置，毫米。用于解近节骨朝向，见
    // computeSegmentQuats 里的说明。不设置就退化到 pp->mp（尾部会明显变差）。
    // 设置指根 anchor（【必须与 setTemplateMm 的手背模板同一个坐标系】）。
    //
    // 【为什么要自检】曾经这里被喂进 HandModel 腕部系的群体均值常量
    // （食指 anchor=(85,20,0)），而手背模板是标定向导存的、质心在原点的
    // hm20 腕部系。两个原点差约 43mm，而近节骨才 40mm —— 真机实测
    // "pp球 - anchor" 与真实骨方向的夹角是 111°~157°，五根手指全反。
    // 后果是【5 个近节分段的四元数全错】，占手指分段的三分之一，而且
    // 不报任何错。仿真里两套系恰好一致，所以这个 bug 在仿真上完全测不出来。
    //
    // 【自检必须拿【实际观测】比，不能只查模板内部自洽】
    // 第一版判据是"anchor 到手背质心的距离在 15~110mm 内"——放行了错的系
    // （食指群体 anchor 到质心 87.5mm，完全合理，可真实近节球只在 62mm）。
    // 第二版改成"anchor 必须比模板里的近节球更靠近手腕"——【仍然放行】，
    // 因为模板的 15 个手指点【就是用同一批 anchor 合成的】，拿它当参照是
    // 循环论证，怎么查都自洽。
    // 唯一有效的参照是运行时【真正观测到的】近节球位置。所以自检不在
    // setAnchorsMm 里做（那时还没有观测），而是放到第一帧解出腕部位姿之后。
    void setAnchorsMm(const std::array<Vec3, 5>& a) {
        anchorsMm_ = a;
        anchorsSet_ = true;
        anchorsValid_ = false;      // 等第一帧有观测了再判，见 verifyAnchors()
        anchorsChecked_ = false;
        // 【票是给上一套 anchor 投的】自标定收敛过程里 applyAutoCalib() 会
        // 反复下发新 anchor。不清票的话，新 anchor 一进来就继承旧 anchor 攒
        // 的通过率并立刻定案 —— 而"新旧 anchor 哪个好"恰恰是这次要判的事。
        anchorVoteN_ = {};
        anchorVoteOk_ = {};
        anchorOk_ = {};
        anchorVoteFrames_ = 0;
        anchorPlaneN_ = {};
        anchorPlaneOk_ = {};
    }
// ===========================================================================
// Hm20SkeletonAssociator_impl2.ipp —— Hm20SkeletonAssociator 类体分片 2/3。
// 【重要】被 #include 在 class 体内部（见 HandSkeletonAssociator.hpp），类内定义即 inline。
// 原 HandSkeletonAssociator.hpp 第 2406~3467 行（模板/平滑/拇指等实现）。
// ===========================================================================
    bool templateMmValid() const { return tmplMmValid_; }
    // anchor 是否通过了同系自检、正在被使用。false = 已退回 pp->mp。
    bool anchorsInUse() const { return anchorsValid_; }

private:

    // 【修改·参考向量】原实现用全局常量做参考：
    //     ref = (std::abs(x[2]) < 0.9) ? {0,0,1} : {0,1,0};
    // 这个三元表达式是一个硬分支：骨轴的世界 z 分量越过 0.9 时参考向量突变，
    // 骨骼明明连续运动，四元数却会无预警翻转。实测（零噪声、标签100%正确）
    // 就已经翻了 14 次、单帧最大跳 156°；加 0.5mm 噪声后骨轴在阈值附近抖动，
    // 翻转次数涨到 156 次。这是纯代码导致的跳变，与模型和点云质量都无关。
    //
    // 改为以手背法线（腕部 R 的第 3 列）为参考并做正交化：随手一起连续变化，
    // 且绕骨轴的 roll 从此有解剖意义（不再是骨轴方向的任意函数）。只有当骨轴
    // 几乎平行于手背法线时才切到掌面方向，那是真正的几何退化，且切换阈值
    // 取在 |sin| < 0.15，远离常用姿态。实测跳变最大值 172.8° -> 28.9°。
    //
    // 仍需注意：绕骨轴的自转（旋前/旋后）在只有 1 个 marker/节的条件下【不可
    // 观测】。这里给出的 roll 是由手背姿态推出来的合理值，不是测出来的。
    // 下游若要用 roll，必须清楚这一点。
    // prevY: 上一帧本段的 y 轴。只在"骨轴几乎平行于手背法线"这个真退化姿态
    // 下才用得上——此时手背法线给不出有效的正交方向，改用上一帧的 y 轴做
    // Gram-Schmidt，保证连续。正常姿态下完全不参与，所以不会累积漂移。
    // rollOffset：绕骨轴把 y/z 轴预旋一个常数（弧度）。
    // 【为什么需要它】roll 参考统一取手背法线。四指的背侧朝向跟手背一致，这个
    // 参考是对的；但【拇指第一掌骨有 80~90° 的解剖旋前】，它的背侧朝向跟手背差
    // 了近 90°。给拇指跟四指一样的 roll，等于把它的弯曲平面按四指来摆 ——
    // 表现就是"拇指弯曲轴朝前，不朝手心"，不符合生理。
    // 而绕骨轴自转在每节只有 1 个 marker 时【不可观测】，测不出来只能建模，
    // 所以这里给拇指补一个常数偏置。这是一阶近似，不是精确解剖模型。
    // 段号 -> roll 偏置。段 1/2/3 是拇指掌骨/近节/远节。
    static double segRollOffset(int seg, double thumbRoll) {
        return (seg >= 1 && seg <= 3) ? thumbRoll : 0.0;
    }


