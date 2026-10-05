// =============================================================================
// Hm20JointAngles.hpp —— 把 hm20 骨架结果转成 HandModel 的 16 维关节角
// =============================================================================
// 【为什么需要这一层】
// hm20 链路的输出是"20 个带标签的点 + 16 个世界系四元数"，而下游（Unity、
// 机械手）要的是【关节角】—— 骨骼动画和伺服都是角度驱动的，没人吃世界系
// 四元数。现有的 UDP 包 (magic "M3DS") 早就定义好了 16 维关节角的契约，
// 而且 UdpSender 一直接的是旧的 HandTrackingWorker。这个文件把 hm20 接上去，
// 包格式一个字节都不用改，Unity 端一行不用动。
//
// 【16 维布局，跟 HandModel.hpp 严格一致】
//   q[0..3]   拇指  CMC屈曲, CMC外展, MCP, IP
//   q[4..6]   食指  MCP屈曲, MCP外展, PIP
//   q[7..9]   中指 / q[10..12] 无名 / q[13..15] 小指
// 四指每根只有 3 个角，DIP 不在里面 —— fingerFK 内部按耦合系数从 PIP 推。
// 这跟 hm20 的解剖假设正好一致（PIP/DIP 是纯铰链、DIP 与 PIP 强相关）。
//
// 【怎么算：直接从四元数算相对旋转】
// 关节角就是【相邻分段的相对旋转】，而分段四元数已经有了：
//   PIP  = 近节骨轴 与 中节骨轴 的夹角        —— 纯相对量，不需要任何标定
//   DIP  = 中节 与 远节 的夹角                —— 同上（四指的 DIP 不进 16 维，
//                                                由下游按耦合系数从 PIP 推）
//   MCP屈曲/外展 = 近节骨轴在【腕部系】里的两个方位角
//                  腕部系来自手背模板（标定向导第 1 步就有），也不需要手指标定
//
// 【绕过的两条弯路，都在真机数据上被否掉了，记下来免得再走】
// 弯路一：拿 handFK 反解（搜一组角让 FK 摆出的球贴合观测）。看着最"正统"，
//   实际不行：handBackMarkers() 是群体均值常量 {15,0,2}{55,22,5}...，
//   而用户标定的模板质心在原点、两两距离指纹也不同。两套腕部系用 Kabsch
//   桥接的残差是 7.4mm —— 近节骨才 40mm，7.4mm 位置误差≈10° 角度误差，
//   跟几何法自身的偏差同量级，等于白做。
// 弯路二：以为"MCP 零位对不上"是主要问题。实际主因是【anchor 用错了坐标系】
//   （见 HandSkeletonAssociator::verifyAnchors），近节骨轴本身就错 111°~157°，
//   拿它算夹角当然荒唐。修好 anchor 之后这里直接算就是对的。
//
// 【MCP 的常量零位偏置怎么办】hm20 腕部系的 +X 未必正好是解剖学中立位方向，
// 所以 MCP 屈曲/外展会带一个常量偏置。对遥操作【不用管】：下游做 ROM 归一化
// （让用户张开到底、握拳到底，记录每关节的 min/max 再映射到机械手行程）时，
// 常量偏置会被 min 和 max 一起吸收掉。遥操作要的是单调性和可重复性。
// =============================================================================
#pragma once

#include <array>
#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>

#include "estimate/Hm20AssocContract.hpp"   // 【只要类型，不要那个类】原来这里 include 的是
                                            // HandSkeletonAssociator.hpp（闭包 5776 行），但本文件
                                            // 一次都没提到 Hm20SkeletonAssociator，要的只是
                                            // SkeletonFrameResult / Hm20Config / IHm20InferenceBackend
                                            // 这些类型 —— 它们在 contract 里（闭包 1557 行）。
                                            // estimate 目录下有 7 个头都犯了同一个错，每个多吃 4219 行，
                                            // 而这些头又被 UI 层层包含，代价是乘出来的。
#include "hand/HandModel.hpp"

namespace mocap {
namespace hm20 {

// =============================================================================
// 【本次新增】关节角解算的逐帧内部量
//
// 【为什么必须把这些抬出来】排查"输出角不对"时问的第一个问题是
// 「到底是四元数错了，还是从四元数取角这一步错了」。而现有的记录里
// 只有四元数和最终角度两头，中间那几个真正决定结果的量——
// 两条骨轴、它们的夹角余弦、铰链符号、近节轴到底取的是哪一根骨头——
// 一个都不出这个函数。于是「四元数验证过是对的」和「角度是错的」
// 两件事同时成立，而中间断成两截，谁也接不上。
//
// 这个结构体把那一截补上。它是【纯出参】，不参与任何计算，
// 传 nullptr 时一条指令都不多执行 —— 现有结果逐位不变。
// =============================================================================
struct JointSolveDebug {
    // ---- 两条骨轴（腕部系，已单位化）。PIP 就是这两条的夹角 ----
    std::array<Vec3, 5> axProx{};      // a0：近节骨轴，取自 segQuat[近节] 的第 0 列
    std::array<Vec3, 5> axMid{};       // a1：中节骨轴，直接由 pp球->mp球 算
    std::array<Vec3, 5> axDist{};      // a2：远节骨轴（拇指 IP 用）
    std::array<Vec3, 5> axProxWorld{}; // a0 的世界系版本，跟四元数直接对照用

    // 【最关键的一个数】dot(a0,a1)。它 ≈1 说明两条"骨轴"其实是同一根骨头，
    // 此时 PIP = acos(1) = 0 —— 不是不准，是恒等于零。
    // 发生条件见 proxAxisSrc。
    std::array<double, 5> dotProxMid{};
    std::array<double, 5> hingeSigned{};   // dot(cross(a0,a1), hinge)，PIP 的符号来源

    // ---- 三个提取式各自的输出（未经"保持上一帧"覆盖）----
    // 【为什么要单独记】out.q 里存的可能是上一帧的值，而这里存的一定是
    // 本帧算出来的。两者不等 = 这一维本帧被保持了，这是采样时最该排除的情况。
    std::array<double, 5> flexRaw{}, abdRaw{}, pipRaw{}, ipRaw{};

    // ---- 近节骨轴的来源。【ROM 采样的门就架在这个字段上】----
    //   0 = anchor->pp球，这是正确的近节骨
    //   1 = pp球->mp球，【退化】——跟 a1 是同一个向量，PIP 恒为 0
    //   2 = 算不出来
    // 退化条件：computeSegmentQuats 里 `f != 0 && anchorsValid && wristPoseValid`
    // 不满足时就走 pp->mp。也就是说：拇指【永远退化】，四指在手背被挡住时退化。
    std::array<uint8_t, 5> proxAxisSrc{};

    // ---- 卷曲度参考信号。ROM 判方向和外推全靠它 ----
    // 【为什么它能在角度失效时还活着】它只用球心之间的【距离】算，
    // 不经过腕部系、不经过四元数、不经过 anchor。上面那些退化一条都影响不到它。
    //   curlChain = |pp->dp| / (|pp->mp| + |mp->dp|)   1=伸直 越小=越卷
    //   curl      = 归一化到 0..1 的闭合度            0=张开 1=握到底
    std::array<double, 5>  curlChain{};
    std::array<double, 5>  curl{};
    std::array<uint8_t, 5> curlValid{};
    // 该指的 curl 是不是靠预测球算出来的。预测球的骨长是被回正过的，
    // 所以分母几乎恒定、分子仍是真信号 —— 可用，但要能在离线时区分开。
    std::array<uint8_t, 5> curlPredicted{};
    std::array<double, 5>  boneLenMm{};   // |pp->mp| + |mp->dp|，curl 的分母

    // ---- 逐维多候选方向参考（ROM 判方向用）----
    // 候选0：本指 curl；候选1：四指共同 curl；候选2：拇CMC屈专用参考。
    // 统一约定：数值越大 = 越接近握拳。
    std::array<std::array<double, 16>, 3>  signRef{};
    std::array<std::array<uint8_t, 16>, 3> signRefValid{};

    // ---- 逐维状态。对应 rom::DofState ----
    // 【v1 缺的就是这一块】v1 只有 fingerValid（逐指一个 bool），
    // 而腕部系失效时同一根手指里 PIP 是新算的、MCP 是保持的，
    // fingerValid 对这两种情况给的是同一个 true。
    std::array<uint8_t, 16> dofState{};

    // ---- PIP 的四元数解算路径（v8）----
    // 【为什么必须记】anchorsValid 为假时叉乘路径整个失效（a0==a1 ->
    // PIP=acos(1)=0），真正在用的是相对四元数那条路。不记这几个量，
    // "PIP 为什么是这个值"就查不到。两条路都记，才能回答"该不该切"。
    std::array<std::array<double, 4>, 5> qRelPip{};  // conj(q近节)*q中节
    std::array<double, 5>  pipFromQuat{};   // 2*acos(|w|)，无符号
    std::array<double, 5>  pipFromAxis{};   // 叉乘路径的结果，对照用
    std::array<uint8_t, 5> pipUsedQuat{};   // 本帧取自哪条路：1=四元数 0=叉乘

    bool wristPoseValid = false;
    bool anchorsValid   = false;
    int  wristStale     = 0;
    bool mirrored       = false;
};

// 【近节/中节骨轴退化判据】|dot(a0,a1)| 超过这个值就认为两条"骨轴"实际是
// 同一根，PIP 的叉乘解法失效，改走相对四元数。0.995 ≈ 5.7°。
// 【为什么不是 1.0-eps】anchorsValid 为真时两条轴也可能碰巧接近平行——
// 那是"手指真的伸直了"，此时 PIP≈0 本来就是正确答案，走哪条路都对；
// 留 5.7° 的余量是为了让那种情形也走四元数路径，行为一致、少一个分支差异。
inline constexpr double kProxMidDegenCos = 0.995;

struct JointAngleResult {
    std::array<double, 16> q{};        // 弧度，布局同 HandModel
    std::array<bool, 5> fingerValid{};
    // 这根手指本帧是不是靠预测段算出来的（近节或中节非几何/IK）。
    // 给角度平滑用：只平滑预测的，实测的一帧都不延迟。
    std::array<bool, 5> fingerPredicted{}; // 该指的角是这一帧算出来的（不是保持上一帧）
    bool wristValid = false;
    // MCP 屈曲/外展这一帧是不是新算的。false = 腕部系不可用，只更新了 PIP。
    // 【下游要判它】否则会把保持值当成"手掌张角没变"，而实际是没测到。
    bool mcpValid = false;
    int  nValidFingers = 0;
};

// -----------------------------------------------------------------------------
// 从 SkeletonFrameResult 算 16 维关节角。
//
// prev: 上一帧结果。某指本帧算不出来时【保持上一帧】而不是清零 ——
//       清零 = 手指瞬间弹回伸直，对遥操作是危险动作（机械手会突然张开）。
//       调用方靠 fingerValid 判断这一帧是新值还是保持值。
// -----------------------------------------------------------------------------
// wristHold: 腕部位姿短时失效时的保持器。见 solveJointAngles 里的说明。
struct WristHold {
    Mat3 R{1,0,0, 0,1,0, 0,0,1};
    bool has = false;
    int  staleFrames = 0;
};

// mirrorOut：输出手性与解算手性不一致时置 true（比如物理上是左手，但为了标签
// 质量把系统手性设成了右手 —— 实测这种搭配连线更稳）。含义见 axisWrist 上方。
inline JointAngleResult solveJointAngles(const SkeletonFrameResult& r,
                                         const JointAngleResult& prev,
                                         WristHold& hold,
                                         int maxStaleFrames = 24,
                                         bool mirrorOut = false,
                                         bool romRelaxed = false,
                                         bool acceptPredicted = true,
                                         JointSolveDebug* dbg = nullptr) {
    using namespace detail;
    JointAngleResult out = prev;
    // 【dbg 是纯出参】默认 nullptr。所有写 dbg 的地方都在 if (dbg) 里，
    // 不传就一条指令都不多执行，现有结果逐位不变。
    if (dbg) {
        *dbg = JointSolveDebug{};
        dbg->wristPoseValid = r.wristPoseValid;
        dbg->anchorsValid   = r.anchorsValid;
        dbg->mirrored       = mirrorOut;
        // 【所有维先标成 Held】下面每算出一维就改写成 Measured/Predicted。
        // 默认值取 Held 而不是 Missing，是因为 out 一开始就是 prev 的拷贝：
        // 没被改写的维，它的值确实就是上一帧的。
        for (int i = 0; i < 16; ++i) dbg->dofState[size_t(i)] = 1;  // rom::DofState::Held
    }
    out.fingerValid = {};
    out.fingerPredicted = {};
    out.nValidFingers = 0;
    out.wristValid = r.wristPoseValid;
    if (!r.valid) return out;

    // 【腕部位姿失效时沿用上一帧的腕部朝向，不要整只手停摆】
    // MCP 屈曲/外展是相对腕部系定义的，所以第一版要求 wristPoseValid。
    // 真机实测这个门太狠：手指屈伸时手背常只剩 3 个点，腕部位姿失效，
    // 结果逐指有效率只有 11~28% —— 也就是八成的帧关节角全部保持上一帧。
    // 对遥操作的表现就是"手指弯下去时机械手卡住不动"，这不是精度问题，是跟不上。
    // 手腕的转动比手指慢得多，短时间（默认 24 帧 = 0.2 秒 @120fps）内沿用
    // 上一帧的腕部朝向，引入的误差远小于"整只手不动"的代价。
    // 超过 maxStaleFrames 仍未恢复才放弃 —— 那时手腕可能真的转过了。
    // 【MCP 和 PIP 必须分开判】PIP/DIP 是相邻骨轴的【相对】夹角，纯局部量，
    // 跟腕部系毫无关系；只有 MCP 屈曲/外展需要腕部系当参考。
    // 手指屈伸时手背被自己的手指挡住是常态（真机实测有 16% 的帧手背一个点
    // 都看不到），如果因为腕部系没了就把整只手的角度全冻住，遥操作上的表现
    // 是"手指弯下去机械手卡住不动"。而 PIP 恰恰是"手握没握"最主要的信号 ——
    // 让它继续更新，机械手至少能跟着握拳张开，只是掌指张角滞后。
    Mat3 WR = r.wristR;
    bool mcpOk = true;
    if (r.wristPoseValid) { hold.R = r.wristR; hold.has = true; hold.staleFrames = 0; }
    else if (hold.has && ++hold.staleFrames <= maxStaleFrames) { WR = hold.R; }
    else { mcpOk = false; }
    out.mcpValid = mcpOk;
    if (dbg) dbg->wristStale = r.wristPoseValid ? 0 : hold.staleFrames;

    // ---- 输出手性镜像 ----
    // 【为什么是"把 z 取反"，而不是"给某几维加负号"】
    // 腕部系是按 +X 指向、+Y 拇指侧、Z=X×Y 造的。对右手 Z 指向手背外侧；
    // 拿同一套构造去描述左手，Z 会指向【掌侧】—— X、Y 的解剖含义不变，只有
    // Z 反了。也就是说：同一姿势的左手，在这个系里的坐标 = 等效右手的坐标
    // 把 z 取反（这是个反射 N=diag(1,1,-1)）。
    //
    // 于是各维怎么变，不需要我逐个去猜，代入下面的提取式即可：
    //   flex = atan2(-a[2], |xy|)      z 反 -> 【屈曲反号】
    //   abd  = atan2( a[1], a[0])      只用 x,y -> 【外展不变】
    //   pip  符号由 dot(cross(a0,a1), hinge) 定，cross 的 xy 分量反、hinge 不变
    //                                  -> 【PIP 反号】
    // 所以真正反的是【屈曲类】(屈曲/PIP/IP)，外展那几维反而不变。
    //
    // 在源头把骨轴的 z 取反，上面三条就自动全对了；比在末尾按下标表加负号
    // 可靠得多 —— 下标表要靠人去数哪几维是屈曲，数错一个就静默出错。
    const double mz = mirrorOut ? -1.0 : 1.0;
    auto axisWrist = [&](int seg) {
        const Mat3 M = quatToMat(r.segQuat[size_t(seg)]);
        const Vec3 v{M[0], M[3], M[6]};                       // 世界系骨轴（列0）
        return Vec3{ WR[0]*v[0] + WR[3]*v[1] + WR[6]*v[2],
                     WR[1]*v[0] + WR[4]*v[1] + WR[7]*v[2],
                     mz * (WR[2]*v[0] + WR[5]*v[1] + WR[8]*v[2]) };
    };
    // 【遮挡时用预测值，不要冻结上一帧】
    //
    // 原来 measured() 只认几何/IK，Predicted 被挡在外面，结果是
    // 那根手指的角度【保持上一帧】—— 手指弯下去、机械手卡住不动。
    //
    // 拿两段真实录制量过（恢复瞬间的角度 vs 恢复后的实测真值）：
    //
    //             冻结上一帧              用预测值
    //   握拳段    中位 20.71°  p90 51.99°   中位  1.64°  p90 22.00°
    //   运动段    中位  3.42°  p90 22.13°   中位  0.36°  p90  1.02°
    //
    // 握拳时冻结的中位误差 20.7° —— 那正是"手指弯下去机械手卡住"。
    // 用预测值好一个数量级。
    //
    // 【为什么现在敢用了】写下那个 gate 的时候预测确实不可信。
    // 之后陆续加了：链式续解（顺着可见部分算）、骨长回正（回正后与标定值
    // 偏差恒为 0.000mm）、时域平滑（alpha 0.8）、恢复补偿。
    // 现在的预测位置在恢复瞬间的中位误差只有 6.4mm。
    //
    // acceptPredicted 留成参数是为了能一键退回旧行为做对照。
    auto measured = [&](int seg) {
        const SegSource s = r.segSource[size_t(seg)];
        if (s == SegSource::Geometry || s == SegSource::Ik) return true;
        return (acceptPredicted || romRelaxed) && s == SegSource::Predicted;
    };
    auto unit = [](Vec3 v, bool& ok) {
        const double n = norm(v);
        ok = (n > 1e-6);
        return ok ? mul(v, 1.0 / n) : v;
    };

    // ---- 卷曲度 curl：【在主循环之外先全算一遍】----
    // 【为什么不能放在循环里】循环体开头就有 `if (!measured(...)) continue;`，
    // 而 curl 恰恰在那些被 continue 掉的帧上最有价值 —— 握到底那一段
    // 中远节球被手掌挡住，角度算不出来，但三个球的位置仍然是有的
    // （链式续解补的，骨长被回正过），距离比值照样能算。
    // ROM 的方向判定和行程外推全靠这一段，放错位置整个链条就断了。
    if (dbg) {
        for (int f = 0; f < 5; ++f) {
            const int pp = 5 + 3*f, mp = pp + 1, dp = pp + 2;
            const Vec3 vPM = sub(r.markers[size_t(mp)].posWorld, r.markers[size_t(pp)].posWorld);
            const Vec3 vMD = sub(r.markers[size_t(dp)].posWorld, r.markers[size_t(mp)].posWorld);
            const Vec3 vPD = sub(r.markers[size_t(dp)].posWorld, r.markers[size_t(pp)].posWorld);
            const double lPM = norm(vPM), lMD = norm(vMD), lPD = norm(vPD);
            const double den = lPM + lMD;
            dbg->boneLenMm[size_t(f)] = den;
            if (den < 1e-3 || lPM < 1e-3 || lMD < 1e-3) { dbg->curlValid[size_t(f)] = 0; continue; }
            dbg->curlChain[size_t(f)] = lPD / den;       // 弦长比，只留作对照
            // ---- curl 用【折角】而不是弦长比 ----
            //
            // 【踩过的坑，别改回去】第一版用的是弦长比 lPD/(lPM+lMD)。
            // 它是余弦型的：折角 0->30° 弦长比只挪了 0.05，而真实开合动作
            // 大部分时间恰恰就落在这一段。拿三段真实录制量过，五指的
            // curl 跨度只有 0.02~0.31，全部低于方向判定所需的 0.15，
            // 结果是【16 维里 5~8 维判不出方向】—— 而数据本身是好的，
            // 手确实在开合，只是被这个刻度压没了。
            //
            // 折角本身是线性的：同样 0->30°，归一化之后挪了 0.33。
            // 换成折角之后同三段素材的跨度回到 0.3~0.9。
            //
            // 【为什么这个刻度换得起】ROM 只拿 curl 做两件事：判方向的符号、
            // 拟合外推的斜率。前者只关心单调，后者关心线性 —— 而弦长比恰好
            // 在这两点上都比折角差。
            const double cosFold = dot(vPM, vMD) / (lPM * lMD);
            const double fold = std::acos(std::clamp(cosFold, -1.0, 1.0));
            // 1.6rad≈92°，DIP 的解剖上限。超过的钳掉，不影响单调。
            dbg->curl[size_t(f)] = std::clamp(fold / 1.6, 0.0, 1.0);
            dbg->curlValid[size_t(f)] = 1;
            const SegSource sm = r.segSource[size_t(2 + f*3)];
            const SegSource sd = r.segSource[size_t(3 + f*3)];
            dbg->curlPredicted[size_t(f)] =
                uint8_t(!(sm == SegSource::Geometry || sm == SegSource::Ik) ||
                        !(sd == SegSource::Geometry || sd == SegSource::Ik));
        }

        // ---- 逐维多候选方向参考 ----
        // 四指一起握拳/张开，所以“四指共同 curl”比单指自己的 curl
        // 抗噪。拇CMC屈的专用参考用拇指尖到食指MCP的距离：越近=越屈曲。
        for (int c = 0; c < 3; ++c)
            for (int i = 0; i < 16; ++i) {
                dbg->signRef[size_t(c)][size_t(i)] = 0.0;
                dbg->signRefValid[size_t(c)][size_t(i)] = 0;
            }
        double commonCurl = 0.0;
        int commonN = 0;
        for (int f = 1; f < 5; ++f)
            if (dbg->curlValid[size_t(f)]) { commonCurl += dbg->curl[size_t(f)]; ++commonN; }
        if (commonN > 0) commonCurl /= double(commonN);
        for (int i = 0; i < 16; ++i) {
            if (i == 1 || i == 5 || i == 8 || i == 11 || i == 14) continue;
            const int f = (i < 4) ? 0 : 1 + (i - 4) / 3;
            dbg->signRef[0][size_t(i)] = dbg->curl[size_t(f)];
            dbg->signRefValid[0][size_t(i)] = dbg->curlValid[size_t(f)];
            if (commonN >= 2) {
                dbg->signRef[1][size_t(i)] = commonCurl;
                dbg->signRefValid[1][size_t(i)] = 1;
            }
        }
        {
            const Vec3 tip = r.markers[7].posWorld;
            const Vec3 idxPp = r.markers[8].posWorld;
            const Vec3 thumbMcp = r.markers[5].posWorld;
            const double denom = norm(sub(tip, thumbMcp));
            const double dist  = norm(sub(tip, idxPp));
            if (denom > 1e-3 && std::isfinite(dist) && std::isfinite(denom)) {
                dbg->signRef[2][0] = -dist / denom;
                dbg->signRefValid[2][0] = 1;
            }
        }
    }

    for (int f = 0; f < 5; ++f) {
        const int sProx = 1 + f*3, sMid = sProx + 1, sDist = sMid + 1;
        if (!measured(sProx) || !measured(sMid)) continue;
        out.fingerPredicted[size_t(f)] =
            !(r.segSource[size_t(sProx)] == SegSource::Geometry ||
              r.segSource[size_t(sProx)] == SegSource::Ik) ||
            !(r.segSource[size_t(sMid)] == SegSource::Geometry ||
              r.segSource[size_t(sMid)] == SegSource::Ik);
        bool ok0 = false, ok1 = false;
        const Vec3 a0 = unit(axisWrist(sProx), ok0);

        // ---- PIP 的第二条骨轴：必须是【中节骨】----
        //
        // 【贴球几何】球心在每节指骨的远端附近（HandSkeletonAssociator.hpp
        // 里 dirProxUse 那段注释记着实测结果），于是：
        //     anchor -> pp球  = 近节骨   （赋给 sProx）
        //     pp球   -> mp球  = 中节骨   ← 【没有被赋给任何一段】
        //     mp球   -> dp球  = 远节骨   （赋给 sDist）
        // 而 sMid 在 midDirMode=0 时是 pp->dp，【跨了中节+远节两节】。
        //
        // 拿 sProx 跟 sMid 求夹角，算出来的是折线的转折角，不是 PIP 关节角。
        // 实测压缩比恒定在 0.42：真实 40° 只读出 16.6°，100° 读出 39°。
        // 真机对得上：放松张开手的截图里 PIP 读 10.8~16.5°，
        // 除以 0.42 得 26~39° —— 那才是放松手 PIP 的正常范围（30~45°）。
        // 手是对的，是读数被压缩了约 2.4 倍。
        //
        // 【也不能用 sDist】那是远节骨，跟近节骨的夹角是 PIP+DIP 的和。
        //
        // 中节骨向量在 markers 里现成就有，直接取，不经过 segQuat。
        const int mPp = 5 + 3*f, mMp = mPp + 1;
        Vec3 midBone = sub(r.markers[size_t(mMp)].posWorld,
                           r.markers[size_t(mPp)].posWorld);
        // 转到腕部系（跟 axisWrist 一致的变换）
        Vec3 a1w{WR[0]*midBone[0] + WR[3]*midBone[1] + WR[6]*midBone[2],
                 WR[1]*midBone[0] + WR[4]*midBone[1] + WR[7]*midBone[2],
                 WR[2]*midBone[0] + WR[5]*midBone[1] + WR[8]*midBone[2]};
        if (mirrorOut) a1w[2] = -a1w[2];
        const Vec3 a1 = unit(a1w, ok1);
        if (!ok0 || !ok1) continue;

        // ---- 记录两条骨轴，以及"它们是不是同一根骨头" ----
        // 【这是整条链上最该被记下来的一个数】computeSegmentQuats 里
        //     dirProxUse = anchor->pp球     需要 (f!=0 && anchorsValid && wristPoseValid)
        //     dirProxUse = pp球->mp球       否则退化
        // 而这里的 a1 恒等于 pp球->mp球。三个条件任一不满足，a0 和 a1 就是
        // 同一个向量，PIP = acos(1) = 0。拇指的 f!=0 恒为假，所以【拇MCP 永远是 0】。
        if (dbg) {
            const size_t uf = size_t(f);
            dbg->axProx[uf] = a0; dbg->axMid[uf] = a1;
            const Mat3 Mw = quatToMat(r.segQuat[size_t(sProx)]);
            dbg->axProxWorld[uf] = Vec3{Mw[0], Mw[3], Mw[6]};
            const double dpm = dot(a0, a1);
            dbg->dotProxMid[uf] = dpm;
            const bool anchorPath = (f != 0) && r.anchorsValid && r.wristPoseValid;
            // 【判据用两条：结构条件 + 实测夹角】只看结构条件会漏掉
            // "条件满足但 anchor 本身是坏的"那一类；只看夹角会把真实的
            // 手指伸直（PIP 确实接近 0）误判成退化。两条都不满足才算好。
            dbg->proxAxisSrc[uf] = uint8_t(anchorPath ? 0 : 1);
        }

        // MCP（拇指是 CMC）：腕部系约定 +X 手指方向、+Y 拇指侧、+Z 手背法向。
        // 屈曲 = 骨轴往掌侧压下去；外展 = 在手掌平面内左右摆。
        // 用 atan2 不用 asin：靠近极值时 asin 数值条件差且丢符号。
        const double flex = std::atan2(-a0[2], std::sqrt(a0[0]*a0[0] + a0[1]*a0[1]));
        const double abd  = std::atan2(a0[1], a0[0]);
        if (dbg) { dbg->flexRaw[size_t(f)] = flex; dbg->abdRaw[size_t(f)] = abd; }

        // PIP：先算带符号夹角用于诊断，最后统一取绝对值输出弯曲量。
        // 这是刻意的：PIP 目标行程 [0,1.92] 不表示反屈，而 hingeSigned
        // 在真机的不同握拳循环里会翻号；保住“弯曲=增加”比保留反屈更重要。
        double pip = std::acos(std::clamp(dot(a0, a1), -1.0, 1.0));
        // 【铰链方向修正 —— 这是个真 bug，不是约定】
        // 原来写的是 cross(a0, ẑ)：手指沿 +X 时铰链 = -Y，于是 PIP【屈曲算出来是负的】。
        // 而同一个文件里 MCP 屈曲 flex=atan2(-a[2],|xy|) 屈曲是【正】的，
        // jointLimits() 里 PIP 也是 {0.0, 1.8}（正数=屈曲）。三者对不上。
        //
        // 后果：手指一弯 PIP 就是负数，被 clamp 到 0 —— 真机上握拳时实测
        // 食PIP -31.7 / 中PIP -19.4 / 无PIP -11.1，全部钳成 0，PIP 这一路
        // 永远输不出信号。而 ROM 标定【救不了】它：ROM 只把观测区间映射到目标
        // 区间，一个反向的信号映射完还是反向的（握拳→0、张开→最大）。
        //
        // 改成 cross(ẑ, a0)，铰链反向，屈曲为正，与 MCP 和限位表一致。
        Vec3 hinge = cross(Vec3{0, 0, 1}, a0);
        bool okh = false;
        hinge = unit(hinge, okh);
        const double hs = okh ? dot(cross(a0, a1), hinge) : 0.0;
        if (okh && hs < 0) pip = -pip;
        // 【PIP 统一取弯曲量，不再输出带符号值】
        // 铰链符号在个别手指/姿态会翻：真机录制里同一根小指、
        // 同样都是握拳，不同循环的 hingeSigned 正负交替，
        // 一翻就把整指趋势翻过去。PIP 的目标行程是 [0,1.92]，
        // 本来就不表示反屈；ROM 和遥操作只关心“弯曲量”。
        // 取绝对值后恒定满足：弯曲增加、张开减小。
        pip = std::abs(pip);
        // 【在可能被四元数路径覆盖之前先存一份】这是对照量：两条路都算出来、
        // 都记下来，才能在离线时回答"这一帧到底差多少、该不该切"。
        if (dbg) dbg->pipFromAxis[size_t(f)] = float(pip);

        // =====================================================================
        // 【骨轴退化时改用相对四元数直接解 PIP】
        //
        // 上面这套"两条骨轴求夹角"有一个前提：a0 和 a1 得真的是两根不同的骨。
        // 而 a0 = axisWrist(sProx) 取自 segQuat[近节]，它在 computeSegmentQuats
        // 里的来源是：
        //     dirProxUse = anchor->pp球   需要 (f!=0 && anchorsValid && wristPoseValid)
        //     dirProxUse = pp球->mp球     否则退化
        // 而这里的 a1 【恒等于】pp球->mp球。所以只要 anchorsValid 为假，
        // a0 和 a1 就是同一个向量，PIP = acos(1) = 0 —— 不是不准，是恒等于零。
        //
        // 【实测证据】用户的真机录制(212帧，握拳/张开循环)：
        //   · anchorsValid 全程为 0（五根 anchor 投票全部未通过），
        //     proxAxisSrc 五指 212/212 帧全部 = 1（退化分支）
        //   · dotProxMid 中位数 0.9998~1.0000，>0.99 的帧占 85.8~96.2%
        //     —— 两条"骨轴"确实是同一根
        //   · 于是 qSolve 里：拇MCP屈 平摊→握拳只变 +0.006 rad，
        //     中PIP +0.006、名PIP +0.005、小PIP +0.025 —— 全部是死的
        //   · pipRaw 与 curl(独立信号，只用球心距离算)的相关系数
        //     只有 -0.196~+0.371，方向还各不相同，正是"有些反着增大"的来源
        //
        // 【为什么用相对四元数是对的，而且不需要 anchor】
        // segQuat[中节] 相对 segQuat[近节] 的旋转，物理含义就是"中节相对近节
        // 转了多少"——这正是 PIP 关节角的定义，不多不少。它只依赖两段各自的
        // 朝向，完全不经过 anchor、不经过腕部系、不做叉乘，所以 anchorsValid
        // 为假时它照样成立。四元数本身的正确性已经单独验证过（见
        // MocapConvert.FromMocapQuat 的说明）。
        //
        // 【实测验证】同一份录制，用 qRel 解出的 PIP 与 curl 的相关系数：
        //   可信段(segSource>=2)上 食 +0.994 / 中 +0.995 / 名 +0.962 / 小 +0.994
        // 对比原路径的 +0.371 / -0.072 / +0.019 / -0.196。
        //
        // 【为什么保留原路径而不是直接删掉】anchor 正常可用时，原路径的
        // a0 是"anchor->pp球"，那是一条【跨过 MCP 关节】的真实骨轴，
        // 对 MCP 屈曲/外展的解算比四元数更直接；这里只替换退化情形下的 PIP，
        // 不动 MCP 那两维，改动范围最小、可回退。
        // =====================================================================
        const bool proxDegenNow =
            !((f != 0) && r.anchorsValid && r.wristPoseValid) ||
            std::fabs(dot(a0, a1)) > kProxMidDegenCos;
        if (proxDegenNow) {
            // qRel = conj(q_prox) * q_mid，即"中节相对近节"
            const Quat& qP = r.segQuat[size_t(sProx)];
            const Quat& qM = r.segQuat[size_t(sMid)];
            const Quat qPc{qP[0], -qP[1], -qP[2], -qP[3]};
            const Quat qRel{
                qPc[0]*qM[0] - qPc[1]*qM[1] - qPc[2]*qM[2] - qPc[3]*qM[3],
                qPc[0]*qM[1] + qPc[1]*qM[0] + qPc[2]*qM[3] - qPc[3]*qM[2],
                qPc[0]*qM[2] - qPc[1]*qM[3] + qPc[2]*qM[0] + qPc[3]*qM[1],
                qPc[0]*qM[3] + qPc[1]*qM[2] - qPc[2]*qM[1] + qPc[3]*qM[0]};
            // 【取无符号转角，不再按转轴定号】
            // PIP/DIP 是铰链关节，解剖上只能单向屈曲（限位表也是 {0.0, 1.8}，
            // 纯正数），所以"屈曲量"本身就是无符号量，不需要额外定号。
            //
            // 【试过按转轴定号，实测更差，所以不用】在这份录制上按 qRel 转轴的
            // y 分量定号，相关系数掉到 0.545~0.994 并出现负值——因为退化帧里
            // 转轴本身方向不稳，用一个不稳的量去给一个稳的量定号，等于把噪声
            // 引进结果。无符号角在同一份数据上是 0.962~0.995，五指方向全部一致
            // （平摊 3~5° → 握拳 18~20°）。
            //
            // 【|w| 而不是 w】q 和 -q 是同一个旋转，不取绝对值会让同一姿态在
            // 相邻两帧之间跳出 2π-θ 的假变化。
            pip = 2.0 * std::acos(std::clamp(std::fabs(qRel[0]), 0.0, 1.0));
            if (dbg) {
                const size_t uf = size_t(f);
                for (int k = 0; k < 4; ++k) dbg->qRelPip[uf][k] = float(qRel[size_t(k)]);
                dbg->pipFromQuat[uf] = float(pip);
                dbg->pipUsedQuat[uf] = 1;
            }
        }
        if (dbg && !proxDegenNow) dbg->pipUsedQuat[size_t(f)] = 0;
        if (dbg) { dbg->hingeSigned[size_t(f)] = hs; dbg->pipRaw[size_t(f)] = pip; }

        // ---- 逐维状态：这一维本帧到底是怎么来的 ----
        // 【为什么必须逐维而不是逐指】就在下面这个 if 里：腕部系没了的时候
        // PIP 是新算的、MCP 那两维是保持上一帧的，而 fingerValid 对这两种
        // 情况给的是同一个 true。ROM v1 照着 fingerValid 采样，于是把大量
        // 「张开时的 MCP 值」当成握拳时的样本收了进去 —— 行程被这样削掉一截。
        // 【关键修复：走了四元数路径就不能再报 Degenerate】
        //
        // 原来这里是 `proxDegen = (proxAxisSrc != 0)`，即"只要 anchor 路径没走成
        // 就把 PIP 标成退化(2)"。而 rom::Mapper::apply() 里，ROM 标定完成后：
        //     DofState::Degenerate -> out[u] = last_[u]     （永久保持上一帧）
        // 并且 ROM 标定阶段 observe() 会把这些帧按 AxisDegen 拒收，于是这一维
        // 攒不够样本，finish() 判成 DofStatus::AxisDead，apply() 里又变成
        //     DofStatus::AxisDead -> out[u] = neutral()     （永久钉在中立位）
        //
        // 两条路都通向"标定完成后这一维再也不动"——这正是"ROM标定完关节角
        // 直接固定"的机制。实测该用户录制里 dofState 的 PIP 五维
        // (下标 2/6/9/12/15) 在 212/212 帧里【全部】是 2。
        //
        // 现在退化情形已经由相对四元数正确解出，值是真实测量得来的，
        // 不再是"acos(1)=0 的假值"，所以状态必须如实报成 Measured/Predicted，
        // 否则 ROM 会继续拒收这些完全可用的样本，白白把这一维判死。
        const uint8_t stPip = uint8_t(out.fingerPredicted[size_t(f)] ? 3 : 4); // Predicted/Measured
        const uint8_t stMcp = uint8_t(out.fingerPredicted[size_t(f)] ? 3 : 4);
        auto mark = [&](int idx, uint8_t st) { if (dbg) dbg->dofState[size_t(idx)] = st; };

        if (!mcpOk) {
            // 腕部系没了：只更新 PIP（拇指是 MCP），MCP 那两位保持上一帧
            if (f == 0) { out.q[2] = pip; mark(2, stPip); }
            else {
                const int b2 = 4 + (f - 1) * 3 + 2;
                out.q[size_t(b2)] = pip; mark(b2, stPip);
            }
            out.fingerValid[size_t(f)] = true;
            ++out.nValidFingers;
            continue;
        }
        if (f == 0) {
            double ip = out.q[3];
            if (measured(sDist)) {
                bool ok2 = false;
                const Vec3 a2 = unit(axisWrist(sDist), ok2);
                if (ok2) ip = std::acos(std::clamp(dot(a1, a2), -1.0, 1.0));
            }
            out.q[0] = flex; out.q[1] = abd; out.q[2] = pip; out.q[3] = ip;
            mark(0, stMcp); mark(1, stMcp); mark(2, stPip);
            // 拇指 IP 只在远节可用时才是新的，否则沿用上一帧 —— 别一起标成新的
            if (measured(sDist)) { mark(3, stMcp); if (dbg) dbg->ipRaw[0] = ip; }
        } else {
            const int b = 4 + (f - 1) * 3;
            out.q[size_t(b)] = flex; out.q[size_t(b+1)] = abd; out.q[size_t(b+2)] = pip;
            mark(b, stMcp); mark(b+1, stMcp); mark(b+2, stPip);
        }
        out.fingerValid[size_t(f)] = true;
        ++out.nValidFingers;
    }

    // 【这里【故意不钳位】】
    // 第一版在这里就 clampToLimits，实测把信号压死了：静止摊开的手
    // 食MCP屈 恒为 0.0、食MCP展 恒为 17.2、拇CMC展 恒为 45.8，三个都
    // 100% 贴在限位上。原因不是手在乱动，而是【常量零位偏置】——
    // hm20 腕部系的 +X 未必正好是解剖学中立位方向，MCP 角整体平移之后
    // 超出限位，被钳位压成一条直线，偏置和信号一起没了。
    // 正确顺序是：原始角 -> ROM 归一化(吸收偏置) -> 映射到目标行程 -> 钳位。
    // 所以钳位交给 RomMapper::apply()，这里输出【未钳位的原始角】。
    return out;
}

// =============================================================================
// ROM 标定与映射
//
// 【为什么遥操作必须有这一层】
// 一是人手和机械手/Unity 模型的行程根本不同：人的 MCP 大约 -10°~100°，
// 机械手可能只有 0°~90°，传动比也不一样。直接抄绝对角度的结果是机械手
// 永远合不拢、也过不了伸。
// 二是我们的 MCP 角带一个未知的常量零位偏置（hm20 腕部系的 +X 不是解剖学
// 中立位方向）。ROM 归一化时 min 和 max 一起偏，相减【正好抵消】。
// 遥操作真正要的是单调性和可重复性，不是绝对准确 —— 这一层同时解决两件事。
//
// 【操作者要做什么】一次「五指张开到底 → 握拳到底」，来回两三遍，约 5 秒。
// 期间调用 observe()，结束调用 finish()。
// =============================================================================
// 【已废弃 —— 保留只为了能 A/B 对照，新代码一律用 rom::Calibrator】
// 三条隐含假设在真机上全部不成立（逐维新鲜度、方向、骨轴退化），
// 详见 estimate/RomCalibration.hpp 顶部。SkeletonAssocWorker 已经切走。
//
// 【为什么要在这里关掉 deprecated 警告】下面 RomMapper::apply() 的签名里
// 写着 `const RomCalibrator&` —— 两个类都标了 deprecated，于是【头文件自己
// 引用自己】就会触发一条警告，而且它会跟着 include 链一路复制到每个包含
// 这个头的翻译单元里（moc 生成的那几个 .cpp 也算），一次构建刷出十几条。
//
// 关掉的只是【这一段内部】的自引用。属性本身留着，所以任何【外部】新写的
// 代码用到 v1 仍然照常警告 —— 那才是这个属性存在的意义。
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#elif defined(_MSC_VER)
#  pragma warning(push)
#  pragma warning(disable : 4996)
#endif
class [[deprecated("用 hm20::rom::Calibrator，见 RomCalibration.hpp")]] RomCalibrator {
public:
    void reset() { lo_ = {}; hi_ = {}; n_ = 0; done_ = false;
                   for (auto& v : samp_) v.clear(); }
    bool ready() const { return done_; }
    int  samples() const { return n_; }
    int  sampleCount(int i) const { return int(samp_[size_t(i)].size()); }

    // 【存样本、不存 min/max】
    // 第一版直接累 min/max，实测被野点毁掉：真机数据里 食MCP外展 的原始角
    // 出现过 -177°~128° 的范围（物理上不可能），ROM 按这个极值拉伸，
    // 正常动作只占到映射行程的一小段。一个野点就能把整条映射废掉。
    // 改成存样本、取 p2/p98 —— 丢掉两端各 2%，正常人手的实际行程不受影响。
    // 【只丢无效指，不要求"全程几何解"】
    //
    // 真机踩到的死结：ROM 标定要求操作者握拳到底，而【握拳时中远节球正好
    // 被手掌挡住】—— 真实素材里中指三点全可见只有 31%。
    // solveJointAngles 的 measured() 只认几何解和 IK 解，Predicted 不算，
    // 于是握得越紧、fingerValid 越少、采到的样本越少。
    // 实测 889 帧握拳素材里 ikFallback 中位 8（74% 的帧有遮挡补点），
    // 结果就是"怎么握都提示行程太短"。
    //
    // fingerValid 为假时【PIP 那一维仍然可能是有效的】——
    // solveJointAngles 里 mcpOk 为假时会单独更新 PIP 并置 fingerValid=true，
    // 反过来 fingerValid 为假只说明这根指这一帧完全没解出来。
    // 所以这里保持"无效就跳过"，真正要放宽的是 measured() 那一层，
    // 见 solveJointAngles 里 romRelaxed 的说明。
    void observe(const JointAngleResult& r) {
        for (int f = 0; f < 5; ++f) {
            if (!r.fingerValid[size_t(f)]) continue;
            const int base = (f == 0) ? 0 : 4 + (f - 1) * 3;
            const int ndof = (f == 0) ? 4 : 3;
            for (int k = 0; k < ndof; ++k) {
                const size_t i = size_t(base + k);
                if (samp_[i].size() < kMaxSamples) samp_[i].push_back(r.q[i]);
            }
        }
        ++n_;
    }

    // 至少要看到足够大的行程才算标定成功。行程太小说明操作者没真的张开/握紧，
    // 此时归一化会把噪声放大成满行程 —— 那比不归一化更危险。
    bool finish(double minRangeRad = 0.35) {
        int okCount = 0;
        for (int i = 0; i < 16; ++i) {
            auto& v = samp_[size_t(i)];
            if (v.size() < 30) { lo_[size_t(i)] = hi_[size_t(i)] = 0.0; continue; }
            std::sort(v.begin(), v.end());
            lo_[size_t(i)] = v[size_t(0.02 * double(v.size() - 1) + 0.5)];
            hi_[size_t(i)] = v[size_t(0.98 * double(v.size() - 1) + 0.5)];
            // 【只数屈曲维】外展维（下标 1/5/8/11/14）在"张开→握拳"这个
            // 引导动作下天然采不到行程 —— 那是左右张合，跟屈伸是两个自由度。
            // 把它们算进达标数，等于要求 11 个屈曲维里 8 个达标，凭空加严。
            const bool isAbd = (i == 1 || i == 5 || i == 8 || i == 11 || i == 14);
            if (!isAbd && hi_[size_t(i)] - lo_[size_t(i)] >= minRangeRad) ++okCount;
        }
        // 11 个屈曲维里至少 8 个达标。
        done_ = (okCount >= 8);
        return done_;
    }

    double lo(int i) const { return lo_[size_t(i)]; }
    double hi(int i) const { return hi_[size_t(i)]; }
    // 观测到的行程占该关节解剖限位的比例。
    //
    // 【只统计屈曲维，不含外展】
    // 原来是 16 个关节一起平均，而标定引导的动作是"张开→握拳" ——
    // 那个动作【本来就不采外展】：外展是手指左右张合，跟屈伸是两个自由度。
    // 5 个外展维在这个动作下只能采到 11~25% 的解剖行程，把平均值死死压住。
    //
    // 按各维实际能采到的量算过：16 维一起平均的理论上限只有 45%，
    // 也就是说【怎么做都到不了 70%】—— 用户以为自己没做到位，其实是指标的问题。
    // 只算屈曲维（11 个：4 个拇指 + 4 指各 MCP屈 + PIP）之后上限回到 90%+。
    //
    // 外展要不要标定另说：它的零位偏置确实需要吸收（真机实测食指 23°、
    // 小指 -1.3°，那是掌骨扇形的固有偏置）。但那需要一个【单独的引导动作】
    // ——五指并拢再尽量张开——不能混在张开握拳里一起算分。
    double coverage() const {
        const auto& lim = jointLimits();
        // 外展维的下标：拇CMC展=1，食/中/无/小 MCP展 = 5,8,11,14
        auto isAbduction = [](int i) {
            return i == 1 || i == 5 || i == 8 || i == 11 || i == 14;
        };
        double s = 0; int n = 0;
        for (int i = 0; i < 16; ++i) {
            if (isAbduction(i)) continue;
            const double full = lim[size_t(i)].hi - lim[size_t(i)].lo;
            if (full <= 1e-6) continue;
            s += std::min(1.0, (hi_[size_t(i)] - lo_[size_t(i)]) / full);
            ++n;
        }
        return n ? s / double(n) : 0.0;
    }
    // 外展维单独看，给"要不要再做一次张合动作"用。
    double abductionCoverage() const {
        const auto& lim = jointLimits();
        const int idx[5] = {1, 5, 8, 11, 14};
        double s = 0;
        for (int k = 0; k < 5; ++k) {
            const size_t i = size_t(idx[k]);
            const double full = lim[i].hi - lim[i].lo;
            if (full > 1e-6) s += std::min(1.0, (hi_[i] - lo_[i]) / full);
        }
        return s / 5.0;
    }

private:
    static constexpr size_t kMaxSamples = 3000;
    std::array<std::vector<double>, 16> samp_{};
    std::array<double, 16> lo_{}, hi_{};
    int n_ = 0;
    bool done_ = false;
};

// -----------------------------------------------------------------------------
// 把原始角按标定出来的 ROM 归一化，再映射到目标（Unity 模型 / 机械手）的行程。
// 目标行程默认取 HandModel 的解剖限位；驱动真机械手时换成它自己的关节行程。
// -----------------------------------------------------------------------------
// 【已废弃 —— 见上】apply() 缺 dofState 入参，无法对退化帧保持输出。
class [[deprecated("用 hm20::rom::Mapper，见 RomCalibration.hpp")]] RomMapper {
public:
    void setTargetRange(int joint, double lo, double hi) {
        tLo_[size_t(joint)] = lo; tHi_[size_t(joint)] = hi; custom_[size_t(joint)] = true;
    }

    std::array<double, 16> apply(const std::array<double, 16>& q,
                                 const RomCalibrator& rom) const {
        const auto& lim = jointLimits();
        std::array<double, 16> out{};
        for (int i = 0; i < 16; ++i) {
            const size_t j = size_t(i);
            const double tlo = custom_[j] ? tLo_[j] : lim[j].lo;
            const double thi = custom_[j] ? tHi_[j] : lim[j].hi;
            if (!rom.ready()) {
                // 没标定就直接钳位透传。能动，但机械手的行程用不满。
                out[j] = std::clamp(q[j], tlo, thi);
                continue;
            }
            const double span = rom.hi(i) - rom.lo(i);
            if (span < 1e-3) { out[j] = std::clamp(q[j], tlo, thi); continue; }
            const double u = std::clamp((q[j] - rom.lo(i)) / span, 0.0, 1.0);
            // 【钳位在这里做，不在解算里】先归一化吸收常量偏置，再映射，最后钳。
            out[j] = std::clamp(tlo + u * (thi - tlo), tlo, thi);
        }
        return out;
    }

private:
    std::array<double, 16> tLo_{}, tHi_{};
    std::array<bool, 16> custom_{};
};
// ---- v1 废弃区结束，恢复 deprecated 警告 ----
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic pop
#elif defined(_MSC_VER)
#  pragma warning(pop)
#endif

// -----------------------------------------------------------------------------
// 速度限幅。
//
// 【为什么位置平滑不够、必须单独做这个】平滑管的是"抖不抖"，限幅管的是
// "会不会突然甩一下"。伺服收到超出自身能力的阶跃只有两种结局：饱和后过冲，
// 或者报保护停机。而骨架在标签串了/重捕的那一帧完全可能给出几十度的跳变。
// 遥操作里这是安全问题，不是画质问题。
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// 预测段的角度平滑。
//
// 【为什么位置平滑之后还要这一层】位置平滑压的是每个球的抖动，但关节角是
// 【两个骨轴的夹角】—— 两端各自的残余抖动在求角时会放大，尤其骨短的时候。
// 实测：遮挡段接受预测之后，角度二阶差分 p95 从 2.81° 涨到 6.77°。
//
// 【只平滑预测段】实测段一帧都不延迟 —— 那是亚毫米三角化的结果，压它是净损失。
// 跟位置那边一样：可见时滤波器跟着实测值走，转遮挡那一帧无缝接上。
// -----------------------------------------------------------------------------
struct PredictedAngleSmoother {
    // 【0.5 是在两段真实录制上按两个判据扫出来的】
    //
    //   握拳段                抖动p95   恢复误差中位
    //   冻结上一帧（旧行为）    2.81°       20.71°
    //   用预测 alpha=1.00      6.77°        1.64°
    //   用预测 alpha=0.50      4.28°        3.34°   <- 取这个
    //   用预测 alpha=0.35      3.71°        4.72°
    //   用预测 alpha=0.20      3.30°        9.29°
    //
    // 0.5 处抖动降 37%，恢复误差仍然比冻结好 6 倍。
    // 再往下抖动只再降 0.57°，误差却涨 1.4° —— 不划算。
    //
    // 【冻结的抖动 2.81° 不是"更平稳"】它中位是 0.00°，因为它根本不动。
    // 拿不动换来的平稳没有意义，代价是 20.7° 的滞后 ——
    // 那正是"手指弯下去机械手卡住不动"。
    double alpha = 0.5;                  // 1.0 = 关闭
    // 恢复补偿：把一次性跳变摊到十几帧上。0.88 在 24fps 下约 0.3 秒还清。
    double decay = 0.88;
    double maxOffsetRad = 0.7;      // 约 40°，超过说明预测跑飞，钳住免得拖太远
    std::array<double, 16> s{}, off{};
    std::array<bool, 16> has{};
    std::array<bool, 5>  wasPred{};

    // -----------------------------------------------------------------------
    // 【出参：平滑器的隐藏状态】
    // off[] 是【直接加到输出上的一个逐维常量偏置】，最大能到 maxOffsetRad
    // (0.7rad ≈ 40°)，而它在系统里【没有第二份】—— apply() 是就地算完就返回，
    // 外面只看得到 qSmooth。于是这两件事在文件里长得完全一样：
    //     ① 解算本来就偏了 40°
    //     ② 解算是对的，被一个没还清的恢复补偿顶着
    // 而 ② 的成因是"预测段跑飞过一次"，修法在遮挡处理上，跟 ① 毫无关系。
    //
    // s[] 同理：预测段输出的就是 s，alpha 小的时候它跟不上真实角度 ——
    // 症状是"握拳时手指弯得不够/像卡住"，而握拳恰恰全程都是预测段。
    // 不记 s，这个症状会被一路误判成解算问题。
    struct Debug {
        std::array<double, 16>  state{};       // s[]
        std::array<double, 16>  offset{};      // off[]
        std::array<uint8_t, 16> has{};
        std::array<uint8_t, 16> offClamped{};  // 本帧 off 被 maxOffsetRad 截断
        std::array<uint8_t, 5>  wasPred{};     // 上一帧该指是不是预测段
        std::array<uint8_t, 5>  predNow{};
        int    nOffActive = 0;                 // off 非零的维数
        double maxOffsetSeen = 0.0;
        int    maxOffsetIdx = -1;
    };

    // 【手背重捕/复位时必须能清】s/off/has/wasPred 都是跨帧累积的：
    // 复位前那套错标签解出来的角度会留在 s 里，复位后第一帧的正确角度
    // 反而被当成跳变平滑掉，表现就是"复位完还得飘几秒才对"。
    // 新建 worker 时这几个成员是什么，复位后就该是什么。
    void reset() { s = {}; off = {}; has = {}; wasPred = {}; }

    // 【恢复补偿放在角度这一层，不放在位置上】
    //
    // 位置那边试过：把补偿加到实测 marker 上，恢复后头几帧骨架点会浮在
    // 点云点旁边（第0帧 p90 6.96mm、最大 17.60mm，要十几帧才收敛）——
    // 真机症状是"连的点不在点上"，画面上一眼就看出错。
    // 三角化的位置是亚毫米真值，推开它换平滑是净损失。
    //
    // 而角度是遥操作端真正消费的东西，在这里补偿：位置画面保持真值，
    // 下游拿到的仍然是连续的。两个目标不再冲突。
    // dbg 传 nullptr 时行为跟以前【逐位一致】，一条多余指令都不执行。
    std::array<double, 16> apply(const std::array<double, 16>& q,
                                 const std::array<bool, 5>& fingerPredicted,
                                 Debug* dbg = nullptr) {
        std::array<double, 16> out = q;
        if (dbg) {
            *dbg = Debug{};
            for (int f = 0; f < 5; ++f) {
                // 【wasPred 必须在循环改写它之前抓】它是"上一帧"的值，
                // 而下面每根手指算完就会被覆盖成本帧的值。抓晚一步，
                // 记下来的就变成了本帧 —— 那正好把"恢复那一帧"这个
                // 唯一会产生 off 的时刻抹掉了。
                dbg->wasPred[size_t(f)] = uint8_t(wasPred[size_t(f)]);
                dbg->predNow[size_t(f)] = uint8_t(fingerPredicted[size_t(f)]);
            }
        }
        for (int f = 0; f < 5; ++f) {
            const int base = (f == 0) ? 0 : 4 + (f - 1) * 3;
            const int nd   = (f == 0) ? 4 : 3;
            const bool pred = fingerPredicted[size_t(f)];
            for (int k = 0; k < nd; ++k) {
                const size_t i = size_t(base + k);
                if (!has[i]) { s[i] = q[i]; has[i] = true; off[i] = 0.0; out[i] = q[i]; continue; }
                if (pred) {
                    s[i] += alpha * (q[i] - s[i]);
                    off[i] = 0.0;                    // 遮挡期不做恢复补偿
                    out[i] = s[i];
                } else {
                    // 恢复那一帧：把"预测值 - 实测值"记成偏移，之后按帧衰减地还回去
                    if (wasPred[size_t(f)]) {
                        off[i] = s[i] - q[i];
                        const double lim = maxOffsetRad;
                        // 【截断本身要留痕】off 被钳住 = 预测段跑飞了至少 40°，
                        // 那是一条很强的结论（遮挡期的解算完全不可信）。
                        // 钳完之后 off 看起来就是个正常的 0.7，痕迹没了。
                        if (off[i] >  lim) { off[i] =  lim; if (dbg) dbg->offClamped[i] = 1; }
                        if (off[i] < -lim) { off[i] = -lim; if (dbg) dbg->offClamped[i] = 1; }
                    }
                    off[i] *= decay;
                    if (std::fabs(off[i]) < 0.001) off[i] = 0.0;
                    out[i] = q[i] + off[i];
                    s[i] = out[i];
                }
            }
            wasPred[size_t(f)] = pred;
        }
        if (dbg) {
            for (int i = 0; i < 16; ++i) {
                const size_t u = size_t(i);
                dbg->state[u]  = s[u];
                dbg->offset[u] = off[u];
                dbg->has[u]    = uint8_t(has[u]);
                if (off[u] != 0.0) ++dbg->nOffActive;
                if (std::fabs(off[u]) > dbg->maxOffsetSeen) {
                    dbg->maxOffsetSeen = std::fabs(off[u]);
                    dbg->maxOffsetIdx = i;
                }
            }
        }
        return out;
    }
};

struct RateLimiter {
    double maxRadPerSec = 8.0;     // ≈460°/s，正常人手屈伸远低于此
    std::array<double, 16> last{};
    bool has = false;

    // 【出参】逐维削掉了多少。
    // 【为什么不靠 qOut-qRom 离线相减】那个差值确实等于削掉量，但它只在
    // 限幅【开着】的时候成立；关掉时 qOut==qRom，差值恒为 0，而"关着"和
    // "开着但一次都没削"在文件里就分不开了 —— 那正是排除限幅这一级时
    // 要回答的问题。stepLimitRad 还给出"这一帧最多允许动多少"，
    // 它跟 dt 有关，离线拿一个平均 dt 去推会推错。
    struct Debug {
        std::array<double, 16>  clip{};       // 带符号，被削掉的量
        double stepLimitRad = -1.0;           // maxRadPerSec * max(dt,1e-3)
        int    nClipped = 0;
        double maxClip = 0.0;
        int    maxClipIdx = -1;
        uint8_t hadState = 0;                 // 本帧之前限幅器里有没有上一帧值
    };

    std::array<double, 16> apply(const std::array<double, 16>& q, double dtSec,
                                 Debug* dbg = nullptr) {
        if (dbg) { *dbg = Debug{}; dbg->hadState = uint8_t(has); }
        if (!has) { last = q; has = true; return q; }
        const double lim = maxRadPerSec * std::max(dtSec, 1e-3);
        if (dbg) dbg->stepLimitRad = lim;
        for (int i = 0; i < 16; ++i) {
            const double d = q[size_t(i)] - last[size_t(i)];
            const double dc = std::clamp(d, -lim, lim);
            last[size_t(i)] += dc;
            if (dbg && dc != d) {
                const double c = dc - d;
                dbg->clip[size_t(i)] = c;
                ++dbg->nClipped;
                if (std::fabs(c) > std::fabs(dbg->maxClip)) {
                    dbg->maxClip = c; dbg->maxClipIdx = i;
                }
            }
        }
        return last;
    }
    void reset() { has = false; }
};

}  // namespace hm20
}  // namespace mocap
