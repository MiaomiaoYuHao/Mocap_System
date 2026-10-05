#pragma once
// ===========================================================================
// DorsumRigidSolver.hpp  —— 手背 5 点刚体求解（放到 src/estimate/）
//
// 【它取代什么】
//   Hm20SkeletonAssociator 里的 relabelDorsumByGeometry() + trackDorsum()。
//   那两个函数各自的问题：
//     · relabelDorsumByGeometry 要求手背 5 点【全部】被模型认领才做，缺一个
//       就完全不动；而且它只在模型给出的那 5 个点之间做排列，模型把手指点
//       或杂点判成手背时它无从发现。
//     · trackDorsum 会把几十帧前的旧位置写回 markers 并置 observed=true ——
//       凭空创造观测。贪心最近邻的门限是 25mm/帧，手腕平移超过它（45fps 下
//       1.1m/s）就全部匹配失败，轨迹再也更新不了，于是 5 个点被永久钉死在
//       旧位置。这就是"连线里有几个点固定了、明明那里没有点"的直接来源。
//       而且 prevPos_/prevMask_ 是在 trackDorsum 之后采样的，伪造的观测会
//       回灌给网络当先验，形成正反馈，自己锁死自己。
//
// 【它怎么做】跟 OptiTrack/Vicon 的 rigid body solve 同一个路子：
//   ① 手背模板是刚体，10 条两两距离是它的指纹。
//   ② 每帧在【全部候选点】里找"三点边长同时对得上模板某个三元组"的种子，
//      Kabsch 解位姿，把整份模板投过去收内点，再用内点重解。
//      —— 谁是手背点、谁是哪一个，两件事一起解，不依赖模型的手背分类。
//   ③ 支持 3/4/5 个可见点。没匹配上的槽位用刚体位姿反投影补，
//      **标 observed=false**，绝不谎称观测到了。
//   ④ 歧义（margin 不够）时用【位姿连续性】破，而不是点位移：
//      手背五点近似共面 + 近似反序对称，"反序"错解的 Kabsch 残差只有
//      1.8mm 量级，跟观测噪声同量级，光看残差分不清；但反序错解对应的腕部
//      旋转跟上一帧差约 180°，用帧间转角一票否决，干净利落。
//      （手腕一帧转 180° 不可能，但手腕一帧移动 25mm 很容易——所以判据必须
//        建在位姿上，不能建在点位移上。）
//
// 【复杂度】M 个候选点时最坏 O(20·M²) 次边长比较 + 少量 Kabsch。
//   M=26 实测远低于一次 ONNX 推理的 1%。
//
// 【零依赖】本文件不 include 任何工程头，只用 <array>/<vector>/<cmath>。
//   —— 必须如此：Hm20SkeletonAssociator 要拿 DorsumRigidSolver 当成员，
//      如果本文件反过来 include HandSkeletonAssociator.hpp 就是循环 include，
//      谁先被包含谁就编不过。所以这里自带一份最小数学（下面的 drs 命名空间）。
//      drs::V3/drs::M9 的 typedef 跟关联器逐位一致（std::array<double,3>/<double,9>，
//      行主序），两边可以直接互传，不需要任何转换。
// ===========================================================================

#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

namespace mocap {
namespace hm20 {

namespace drs {   // 本文件私有的最小数学，刻意不依赖 HandSkeletonAssociator.hpp
                  // —— 否则 DorsumRigidSolver 要 include 关联器、关联器又要拿它
                  //    当成员，构成循环 include，谁先被包含就编不过。
using V3 = std::array<double, 3>;
using M9 = std::array<double, 9>;   // 行主序

inline V3 sub(const V3& a, const V3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
inline double dot(const V3& a, const V3& b){ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline double norm(const V3& a) { return std::sqrt(dot(a, a)); }

// 4x4 对称矩阵的 Jacobi 特征分解，返回最大特征值对应的特征向量。
// A 行主序 16 元；out 为该特征向量。
inline void jacobiTop4(const std::array<double,16>& A_in, std::array<double,4>& out) {
    std::array<double,16> A = A_in;
    std::array<double,16> V{{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};
    for (int sweep = 0; sweep < 24; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < 4; ++p) for (int q = p+1; q < 4; ++q) off += A[size_t(p*4+q)]*A[size_t(p*4+q)];
        if (off < 1e-24) break;
        for (int p = 0; p < 4; ++p) for (int q = p+1; q < 4; ++q) {
            const double apq = A[size_t(p*4+q)];
            if (std::fabs(apq) < 1e-18) continue;
            const double theta = (A[size_t(q*4+q)] - A[size_t(p*4+p)]) / (2.0*apq);
            const double t = (theta >= 0 ? 1.0 : -1.0) /
                             (std::fabs(theta) + std::sqrt(theta*theta + 1.0));
            const double c = 1.0/std::sqrt(t*t + 1.0), s = t*c;
            for (int k = 0; k < 4; ++k) {
                const double akp = A[size_t(k*4+p)], akq = A[size_t(k*4+q)];
                A[size_t(k*4+p)] = c*akp - s*akq;
                A[size_t(k*4+q)] = s*akp + c*akq;
            }
            for (int k = 0; k < 4; ++k) {
                const double apk = A[size_t(p*4+k)], aqk = A[size_t(q*4+k)];
                A[size_t(p*4+k)] = c*apk - s*aqk;
                A[size_t(q*4+k)] = s*apk + c*aqk;
            }
            for (int k = 0; k < 4; ++k) {
                const double vkp = V[size_t(k*4+p)], vkq = V[size_t(k*4+q)];
                V[size_t(k*4+p)] = c*vkp - s*vkq;
                V[size_t(k*4+q)] = s*vkp + c*vkq;
            }
        }
    }
    int best = 0;
    for (int i = 1; i < 4; ++i) if (A[size_t(i*4+i)] > A[size_t(best*4+best)]) best = i;
    for (int k = 0; k < 4; ++k) out[size_t(k)] = V[size_t(k*4+best)];
    double n = std::sqrt(out[0]*out[0]+out[1]*out[1]+out[2]*out[2]+out[3]*out[3]);
    if (n > 1e-12) for (int k = 0; k < 4; ++k) out[size_t(k)] /= n;
    else { out = {1,0,0,0}; }
}

// Horn 四元数法的 Kabsch。相对 SVD 法的好处：解出来【天然是真旋转】，
// 不会出现需要事后翻第三列的反射解 —— 而手背 5 点近似共面，SVD 路径正是
// 在共面时最容易踩到 sigma3≈0 的病态分支。返回 RMSE，失败返回 -1。
inline double kabschHorn(const std::vector<V3>& src, const std::vector<V3>& dst,
                         M9& R, V3& t) {
    const size_t n = src.size();
    if (n < 3 || dst.size() != n) return -1.0;
    V3 cs{0,0,0}, cd{0,0,0};
    for (size_t i = 0; i < n; ++i) for (int d = 0; d < 3; ++d) {
        cs[size_t(d)] += src[i][size_t(d)]; cd[size_t(d)] += dst[i][size_t(d)];
    }
    for (int d = 0; d < 3; ++d) { cs[size_t(d)] /= double(n); cd[size_t(d)] /= double(n); }

    double S[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    for (size_t i = 0; i < n; ++i) {
        const V3 a = sub(src[i], cs), b = sub(dst[i], cd);
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) S[r][c] += a[size_t(r)]*b[size_t(c)];
    }
    const double sxx=S[0][0], sxy=S[0][1], sxz=S[0][2];
    const double syx=S[1][0], syy=S[1][1], syz=S[1][2];
    const double szx=S[2][0], szy=S[2][1], szz=S[2][2];
    const std::array<double,16> N{{
        sxx+syy+szz, syz-szy,      szx-sxz,      sxy-syx,
        syz-szy,     sxx-syy-szz,  sxy+syx,      szx+sxz,
        szx-sxz,     sxy+syx,     -sxx+syy-szz,  syz+szy,
        sxy-syx,     szx+sxz,      syz+szy,     -sxx-syy+szz}};
    std::array<double,4> q{};
    jacobiTop4(N, q);
    const double w=q[0], x=q[1], y=q[2], z=q[3];
    R = M9{{1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w),
            2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w),
            2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)}};
    for (int d = 0; d < 3; ++d)
        t[size_t(d)] = cd[size_t(d)] - (R[size_t(d*3+0)]*cs[0] + R[size_t(d*3+1)]*cs[1] + R[size_t(d*3+2)]*cs[2]);

    double se = 0.0;
    for (size_t i = 0; i < n; ++i) {
        for (int d = 0; d < 3; ++d) {
            const double p = R[size_t(d*3+0)]*src[i][0] + R[size_t(d*3+1)]*src[i][1]
                           + R[size_t(d*3+2)]*src[i][2] + t[size_t(d)];
            const double e = p - dst[i][size_t(d)];
            se += e*e;
        }
    }
    return std::sqrt(se / double(n));
}
}  // namespace drs

// 跟 HandSkeletonAssociator.hpp 里的 typedef 完全相同；两个头文件都出现时
// 是同一个类型别名，不冲突。
#ifndef MOCAP_HM20_VEC_TYPES_DEFINED
#define MOCAP_HM20_VEC_TYPES_DEFINED
#endif
using DrsVec3 = drs::V3;
using DrsMat3 = drs::M9;

struct DorsumSolveResult {
    bool   ok        = false;
    // 模板槽 k(0..4) -> 候选点下标；-1 表示本帧这个手背点没被观测到
    std::array<int, 5> pointOf{{-1, -1, -1, -1, -1}};
    drs::M9   R{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    drs::V3   t{{0, 0, 0}};
    double rmseMm    = -1.0;   // 内点的 Kabsch 残差
    double marginMm  = -1.0;   // 次优【不同标签】解与最优解的残差差
    int    nMatched  = 0;
    bool   usedHistory = false;  // true = margin 不够，靠位姿连续性拍板的
    // 当前解的手指落点合理度(mm)，以及"存在明显更合理的备选"这个判定。
    // 后者为真且持续，说明连续性锁到了错的分支 —— 残差看不出来的那种错。
    double fingerScoreMm = -1.0;
    bool   fingerImplausible = false;
    // 0=ok 1=候选点不足 2=没有可行解 3=歧义且无历史(拒解) 4=残差超限
    int    reason    = 1;
    const char* reasonText() const {
        switch (reason) {
            case 0: return "ok";
            case 1: return "候选点不足";
            case 2: return "没有可行解(模板与观测对不上)";
            case 3: return "标签歧义且无历史，本帧拒解";
            case 4: return "残差超限";
            default: return "?";
        }
    }
};

class DorsumRigidSolver {
public:
    struct Config {
        // 三点种子的边长容差。取 4mm：观测噪声 1~2mm，模板本身 <1mm，
        // 留 2 倍余量。放太大会让种子数量爆炸且引入错误种子。
        double pairTolMm   = 4.0;
        // 位姿投影之后收内点的容差。比 pairTolMm 松一点，因为它累积了位姿误差。
        double inlierTolMm = 6.0;
        // 最终接受阈值。超过它说明这组点根本不是那个刚体（贴点移位/粘连）。
        double maxRmseMm   = 4.0;
        // 直接采纳所需的裕度。达不到就走连续性仲裁。
        double minMarginMm = 2.0;
        // 歧义仲裁时允许的帧间腕部转角上限。反序错解是 ~180°，35° 足够宽松
        // 又能一票否决它。手腕最快的抖动在 120fps 下也远低于这个数。
        double maxStepDeg  = 35.0;
        // 歧义仲裁时，参与比较的候选解的残差上限（相对最优解的倍数）。
        double ambiguousRmseRatio = 1.8;
        int    minPoints   = 3;

        // ---- 贴点重捕（球掉了重贴回去）----
        // 【为什么必须有这一环】手背模板是标定时冻结的，之后再不更新。而你
        // 物理上不可能把掉下来的球贴回原来 6mm(inlierTolMm) 以内 —— 实测重贴
        // 后离原位 14~18mm。于是求解器会【永远】拒绝它：它没做错，那个点确实
        // 不符合模板；缺的是模板修复这一环。表现就是"球贴回去了但系统不认"，
        // 而且一直是 4/5 内点，直到重新做整套标定。
        //
        // 【判据不能只看"附近有个点"】遮挡物、反光、另一只手的球都可能出现在
        // 附近。真正区分"重贴的球"和"路过的点"的是：重贴的球【刚性固定在手上】，
        // 所以它在腕部系里的局部坐标是【常数】；路过的点不是。
        // 所以这里累计的是局部坐标，看它的散布，而不是看它离预测位置多近。
        bool   repairEnabled     = true;
        double repairSearchMm    = 45.0;   // 搜索半径，远大于 inlierTolMm
        // 【重捕的观察期】45 帧 ≈ 1.8 秒，用户会觉得"贴回去了半天不认"。
        // 缩到 18 帧 ≈ 0.7 秒。安全性不靠帧数堆，靠 repairMaxScatterMm：
        // 刚性固定的球在腕部系里几乎不动，18 帧足够把它和游走的杂点分开
        // （实测三种误触发场景在 18 帧下仍然 0 误修，见 test_dorsum_repair）。
        int    repairMinFrames   = 18;     // ~0.7s
        double repairMaxScatterMm= 2.5;    // 局部坐标散布上限（刚性 => 应该很小）
        double repairMinPairMm   = 12.0;   // 改完之后跟其它 4 点的最小间距
        double repairMinSelfAmbMm= 2.0;

        // ---- 持续微调（点位小幅漂移的重适应）----
        // 【为什么重捕不够】重捕只处理"完全匹配不上"的槽位（漂移 > inlierTolMm）。
        // 漂移在容差以内时点仍然匹配得上，于是模板【永远不更新】，那点误差就
        // 一直挂在残差里 —— 表现就是"点位微变化重适应很慢"，实际是根本不适应。
        // 胶带蠕变、轻微碰撞造成的几毫米位移全落在这一档。
        //
        // 做法：解得好的帧上，把每个模板点朝实测的局部坐标拉一点点。
        // 这是标准的 rigid body 模板自适应，不需要触发条件、不需要异步 ——
        // 5 个点的一阶低通，微秒级。
        bool   refineEnabled  = true;
        double refineAlpha    = 0.05;   // 时间常数 ~20 帧 ≈ 0.8s @25fps
        double refineMaxRmse  = 2.0;    // 解得不好的帧不学，免得把误差学进去
        int    refineMinInliers = 4;
        double refineMaxDriftMm = 25.0; // 相对标定值的累计漂移上限，防跑飞

        // ---- 丢锁看门狗 ----
        // 【为什么必须有】求解器有三个"进去出不来"的状态：
        //   ① prevR_ 一旦锁到错的分支，连续性仲裁会永远偏向它
        //   ② 模板被一次错误的重捕改坏之后没有回头路
        //   ③ observeForRepair 要求 r.ok，求解失败就没法重捕 -> 死锁
        // 真机现象：手背某个点挪开、连线全乱之后修不回来，必须重开面板 ——
        // 重开等于重建关联器，那是当时唯一的复位路径。
        //
        // 两级恢复，先轻后重：
        //   一级：只松开连续性锁，让几何自己重新建锁（代价最小，多数情况够了）
        //   二级：把模板退回标定时那一份，清掉重捕缓冲（等于面板里的"重开"）
        // 【时间常数按"用户能忍多久"定，不是按"稳妥"定】
        // 骨架输出 23~28fps，20 帧 = 0.8 秒，90 帧 = 3.5 秒 —— 乱了之后要等
        // 三秒半才自动退回模板，用户早就手动重开面板了，等于这道闸白设。
        // 松锁本身几乎无代价（大不了下一帧几何重新选一次），激进一点更好。
        int relockAfterBad   = 6;     // ~0.25s
        int revertAfterBad   = 25;    // ~1.0s
        // 判"解不好"的标准：解不出来，或内点太少，或残差超限
        int    badMinInliers = 4;
        double badRmseMm     = 3.0;
        // 【残差判不出"标签排错"】排错的解残差往往是好的 —— 还是那五个点，
        // 只是编号换了圈。真机现象就是"连线全乱，但看着解出来了"。
        // 能揭穿它的只有【手指落点】：编号错了腕部系就跟着错，手指会落到离
        // 中立位很远的地方。所以持续拿 fingerScore 校验，发现明显更合理的
        // 备选时就认为锁错了。
        double implausibleRatio       = 0.55;  // 备选好这么多倍 -> 判定锁错
        // 手指落点连续这么多帧说"锁错了"就松锁。~0.25s。
        int    relockAfterImplausible = 6;
        // 单帧参与匹配的最大候选点数（防病态帧把耗时拉爆）。
        int    maxCandidates = 32;

        // ---- 耗时控制 ----
        // 【为什么必须有】种子搜索是 O(模板对 x 候选对)，剪枝靠"边长对不上"。
        // 一旦一堆杂点挤在手背附近（反光、另一只手、贴球聚团），剪枝失效，
        // 实测 32 点全挤在一起时单次 24.8ms —— 是 120fps 整帧预算的 3 倍，
        // 跟整个 hm20 推理一个量级。Debug 构建再乘 3~10 倍。
        //
        // 两道闸：
        //  ① 有上一帧位姿时，只看它附近的点。手背一帧动不了多远，
        //     这一刀通常把 32 个候选砍到 6~8 个。
        //  ② 种子数硬上限，兜住①也失效的病态帧（冷启动 + 杂点聚团）。
        double gateWithHistoryMm = 70.0;   // 上一帧手背质心周围的搜索半径
        int    maxSeeds          = 400;    // 单帧最多评估多少个三点种子
    };

    void setConfig(const Config& c) { cfg_ = c; }
    const Config& config() const { return cfg_; }

    // 模板：5 个手背点，【毫米】，跟 Hm20SkeletonAssociator::setTemplateMm 同一个系。
    // 【只在标定时调】它会把 tmpl0_（退回基准）也刷新。
    // 微调/重捕改模板要用 applyTemplateShape()，那个不动基准 ——
    // 否则基准跟着漂，"累计漂移上限"和"退回标定值"这两道闸全部失效。
    void setTemplate(const std::array<drs::V3, 5>& mm) {
        tmpl0_ = mm;
        applyTemplateShape(mm);
        bad_ = 0;
        (void)0;
        tmplValid_ = true;
        // 顺手算一次"最优错误标签的残差"——这是这只手能不能被稳定定标签的
        // 上限指标。<3mm 说明贴点布局本身近似对称，任何算法都会在这两个解
        // 之间来回翻，应该去重贴点而不是继续调参数。见 selfAmbiguityMm()。
        selfAmb_ = computeSelfAmbiguity();
    }
    // 换模板形状但【不动 tmpl0_ 基准】。微调和重捕都走这个。
    void applyTemplateShape(const std::array<drs::V3, 5>& mm) {
        tmpl_ = mm;
        rebuildPairTable();
        tmplValid_ = true;
        selfAmb_ = computeSelfAmbiguity();
    }
    void rebuildPairTable() {
        for (int i = 0; i < 5; ++i)
            for (int j = 0; j < 5; ++j)
                td_[size_t(i * 5 + j)] =
                    drs::norm(drs::sub(tmpl_[size_t(i)], tmpl_[size_t(j)]));
    }

    bool templateValid() const { return tmplValid_; }

    // 中立位的 15 个手指点（毫米，跟手背模板同一个系）。可选，但强烈建议设：
    // 它是【冷启动歧义】唯一的破法。手背反序错解会把腕部系整个翻过去，
    // 手指点因此落到离中立位模板很远的地方；正解则不会。手指弯着也没关系，
    // 弯曲量级(0~60mm)远小于反序造成的错位(2 倍手宽)。
    void setFingerTemplate(const std::array<drs::V3, 15>& mm) {
        fTmpl_ = mm; fTmplValid_ = true;
    }
    bool fingerTemplateValid() const { return fTmplValid_; }

    // 【最该看的一个数】把模板自己跟自己的 120 种排列做 Kabsch，取最优错解的残差。
    //   > 6mm  ：布局很好，标签不会翻
    //   3~6mm  ：勉强，噪声大时偶尔翻
    //   < 3mm  ：布局病态（近似正五边形/近似反序对称），必须重贴点
    double selfAmbiguityMm() const { return selfAmb_; }

    // 一次模板修复事件。【必须报出来】静默改模板是很危险的事：
    // 万一判错，之后所有帧都会基于一个错的模板，而且没有任何痕迹。
    struct RepairEvent {
        bool   happened = false;
        int    slot = -1;
        double moveMm = 0;        // 新旧位置差多远
        double scatterMm = 0;     // 采样的局部坐标散布
        int    frames = 0;
        double newSelfAmbMm = 0;
        bool   rejected = false;  // 通过了稳定性判据但被安全闸挡下
        const char* why = "";
    };

    void resetHistory() { hasPrev_ = false; bad_ = 0; for (auto& b : rbuf_) b.clear(); }

    // 丢锁看门狗。每帧在 solve() 之后调一次，把本帧的结果喂进来。
    // 返回值：0=没动 1=松开了连续性锁 2=退回了原始模板
    int watchdog(const DorsumSolveResult& r) {
        // 【两条独立的"坏"】残差坏(解不出来) 和 标签坏(排错但残差好)。
        // 只看前者会漏掉"连线全乱"那种，那正是要重开面板才好的那类。
        if (r.ok && r.fingerImplausible) {
            if (++implaus_ >= cfg_.relockAfterImplausible) {
                implaus_ = 0; bad_ = 0;
                hasPrev_ = false;                  // 松开连续性锁，让几何重新选
                for (auto& b : rbuf_) b.clear();
                return 1;
            }
        } else {
            implaus_ = 0;
        }
        const bool good = r.ok && r.nMatched >= cfg_.badMinInliers
                       && r.rmseMm >= 0.0 && r.rmseMm <= cfg_.badRmseMm;
        if (good) { bad_ = 0; return 0; }
        ++bad_;
        if (bad_ == cfg_.revertAfterBad) {
            // 二级：连模板都退回去。等价于用户手动重开面板。
            const double keepAmb = selfAmb_;
            setTemplate(tmpl0_);          // 会重算 td_/selfAmb_ 并把 bad_ 清零
            (void)keepAmb;
            hasPrev_ = false;
            for (auto& b : rbuf_) b.clear();
            return 2;
        }
        if (bad_ == cfg_.relockAfterBad) {
            // 一级：只松开连续性锁。歧义时会退回"拒解"，让几何重新建锁。
            hasPrev_ = false;
            for (auto& b : rbuf_) b.clear();
            return 1;
        }
        return 0;
    }
    int badStreak() const { return bad_; }
    // 模板被重捕改过没有（跟标定时那份比）
    double templateDriftMm() const {
        double m = 0;
        for (int k = 0; k < 5; ++k)
            m = std::max(m, drs::norm(drs::sub(tmpl_[size_t(k)], tmpl0_[size_t(k)])));
        return m;
    }
    // 上层接受了某一帧的解之后调它，把位姿存成下一帧的连续性参考。
    void acceptHistory(const drs::M9& R) { prevR_ = R; hasPrev_ = true; }
    // 位置历史，用于空间门控。跟 acceptHistory 分开是因为姿态用于歧义仲裁、
    // 位置只用于剪候选点，两者失效条件不同。
    void acceptCentroid(const drs::V3& t) { prevT_ = t; hasPrevT_ = true; }
    bool hasHistory() const { return hasPrev_; }

    // 观测到的一个手指点：label 取 5..19（跟 hm20 标签编号一致）
    struct FingerObs { int label; drs::V3 pos; };

    // pts       : 本帧【全部】候选点的世界坐标（不要只给模型判成手背的那 5 个）
    // fingerObs : 本帧被模型认领的手指点。只用于【歧义仲裁】，不参与刚体拟合。
    //             传空也能跑，但冷启动遇到歧义时只能拒解。
    DorsumSolveResult solve(const std::vector<drs::V3>& pts,
                            const std::vector<FingerObs>& fingerObs = {}) const {
        DorsumSolveResult out;
        if (!tmplValid_) { out.reason = 1; return out; }
        // ---- ① 空间门控：有上一帧位置时只看附近的点 ----
        // 手背一帧移动不了 70mm。这一刀是整个函数最有效的提速手段，
        // 因为种子搜索的复杂度是候选点数的平方级。
        std::vector<drs::V3> near;
        std::vector<int> nearIdx;
        const std::vector<drs::V3>* pool = &pts;
        if (hasPrevT_ && int(pts.size()) > cfg_.minPoints) {
            near.reserve(pts.size());
            for (size_t j = 0; j < pts.size(); ++j)
                if (drs::norm(drs::sub(pts[j], prevT_)) <= cfg_.gateWithHistoryMm) {
                    near.push_back(pts[j]);
                    nearIdx.push_back(int(j));
                }
            // 门内点太少说明手跳走了或者历史过时，退回全量搜
            if (int(near.size()) >= cfg_.minPoints) pool = &near;
            else { near.clear(); nearIdx.clear(); }
        }
        const std::vector<drs::V3>& P = *pool;
        const bool gated = !nearIdx.empty();

        const int M = int(std::min(P.size(), size_t(cfg_.maxCandidates)));
        if (M < cfg_.minPoints) { out.reason = 1; return out; }

        // 候选点两两距离
        std::vector<double> D(size_t(M) * size_t(M), 0.0);
        for (int a = 0; a < M; ++a)
            for (int b = a + 1; b < M; ++b) {
                const double d = drs::norm(drs::sub(P[size_t(a)], P[size_t(b)]));
                D[size_t(a * M + b)] = d;
                D[size_t(b * M + a)] = d;
            }

        // 去重用：assignment -> 最好残差 / 位姿
        struct Cand { std::array<int, 5> a{}; double r = 0; int n = 0; drs::M9 R{}; drs::V3 t{}; };
        std::vector<Cand> cands;
        cands.reserve(32);

        // 种子数硬上限：兜住"空间门控也失效"的病态帧（冷启动 + 杂点聚团）。
        // 到上限就停止搜种子，用已经找到的候选出解 —— 宁可这一帧解得糙一点，
        // 也不要单帧 25ms 把整条流水线堵住。
        int seeds = 0;
        std::vector<drs::V3> src, dst;
        src.reserve(5); dst.reserve(5);
        drs::M9 R{}; drs::V3 tt{};

        // ---- 种子搜索：模板有序三元组 x 候选有序三元组，边长逐条剪枝 ----
        // 有序遍历会把同一个对应关系找到 6 次，靠下面的 assignment 去重吸收。
        for (int i = 0; i < 5; ++i) {
            for (int j = 0; j < 5; ++j) {
                if (j == i) continue;
                const double tij = td_[size_t(i * 5 + j)];
                for (int a = 0; a < M; ++a) {
                    for (int b = 0; b < M; ++b) {
                        if (b == a) continue;
                        if (std::fabs(D[size_t(a * M + b)] - tij) > cfg_.pairTolMm) continue;
                        for (int k = 0; k < 5; ++k) {
                            if (k == i || k == j) continue;
                            const double tik = td_[size_t(i * 5 + k)];
                            const double tjk = td_[size_t(j * 5 + k)];
                            for (int c = 0; c < M; ++c) {
                                if (c == a || c == b) continue;
                                if (std::fabs(D[size_t(a * M + c)] - tik) > cfg_.pairTolMm) continue;
                                if (std::fabs(D[size_t(b * M + c)] - tjk) > cfg_.pairTolMm) continue;
                                if (++seeds > cfg_.maxSeeds) { i = j = k = 5; a = b = c = M; continue; }

                                // 三点 Kabsch -> 位姿
                                src.clear(); dst.clear();
                                src.push_back(tmpl_[size_t(i)]); dst.push_back(P[size_t(a)]);
                                src.push_back(tmpl_[size_t(j)]); dst.push_back(P[size_t(b)]);
                                src.push_back(tmpl_[size_t(k)]); dst.push_back(P[size_t(c)]);
                                if (drs::kabschHorn(src, dst, R, tt) < 0) continue;

                                // 整份模板投过去收内点（全局最近优先，一对一）
                                std::array<drs::V3, 5> proj{};
                                for (int m = 0; m < 5; ++m) proj[size_t(m)] = xform(R, tt, tmpl_[size_t(m)]);
                                std::array<int, 5> asg{{-1, -1, -1, -1, -1}};
                                greedyMatch(proj, P, M, cfg_.inlierTolMm, asg);

                                int n = 0;
                                for (int m = 0; m < 5; ++m) if (asg[size_t(m)] >= 0) ++n;
                                if (n < cfg_.minPoints) continue;

                                // 用全部内点重解
                                src.clear(); dst.clear();
                                for (int m = 0; m < 5; ++m) if (asg[size_t(m)] >= 0) {
                                    src.push_back(tmpl_[size_t(m)]);
                                    dst.push_back(P[size_t(asg[size_t(m)])]);
                                }
                                const double r = drs::kabschHorn(src, dst, R, tt);
                                if (r < 0) continue;

                                // 去重
                                bool dup = false;
                                for (auto& cd : cands) {
                                    if (cd.a != asg) continue;
                                    dup = true;
                                    if (r < cd.r) { cd.r = r; cd.R = R; cd.t = tt; }
                                    break;
                                }
                                if (!dup) cands.push_back(Cand{asg, r, n, R, tt});
                            }
                        }
                    }
                }
            }
        }

        if (cands.empty()) { out.reason = 2; return out; }

        // 排序：内点多的优先，其次残差小的
        std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) {
            if (x.n != y.n) return x.n > y.n;
            return x.r < y.r;
        });

        const Cand& best = cands.front();
        // margin：同样内点数、但标签【不同】的最好解与它的残差差
        double sec = -1.0;
        for (size_t q = 1; q < cands.size(); ++q) {
            if (cands[q].n != best.n) continue;
            sec = cands[q].r; break;
        }
        const double margin = (sec < 0.0) ? 1e9 : (sec - best.r);

        const Cand* pick = &best;
        bool usedHist = false;
        if (margin < cfg_.minMarginMm) {
            // ---- 歧义仲裁 ----
            // 【为什么不赌残差】手背五点近似共面 + 近似反序对称时，"反序"错解
            // 的残差只比正解高 1.8mm 左右，跟观测噪声同量级。单帧看残差就是
            // 抛硬币，而抛错一次腕部系整个翻 180°，全手的连线一起错。
            // 所以这里换两个【跟残差正交】的判据，按可靠性排序：
            //   ① 位姿连续性：手腕一帧不可能转 180°，反序解一票否决。最强。
            //   ② 手指落点：反序把腕部系翻过去，手指点会落到离中立位模板
            //      很远的地方。冷启动没有历史时靠它。
            //   都用不上就拒解 —— 宁可这一帧没有腕部位姿，也不要一个翻过去的。
            const double lim = best.r * cfg_.ambiguousRmseRatio + 0.5;
            std::vector<const Cand*> pool;
            for (const auto& cd : cands)
                if (cd.n == best.n && cd.r <= lim) pool.push_back(&cd);

            if (hasPrev_) {
                double bestDeg = 1e9; const Cand* sel = nullptr;
                for (const Cand* cd : pool) {
                    const double deg = rotAngleDeg(prevR_, cd->R);
                    if (deg < bestDeg) { bestDeg = deg; sel = cd; }
                }
                if (sel && bestDeg <= cfg_.maxStepDeg) { pick = sel; usedHist = true; }
                else { out.rmseMm = best.r; out.marginMm = margin;
                       out.nMatched = best.n; out.reason = 3; return out; }
            } else if (fTmplValid_ && !fingerObs.empty() && pool.size() > 1) {
                double b1 = 1e18, b2 = 1e18; const Cand* sel = nullptr;
                for (const Cand* cd : pool) {
                    const double sc = fingerScore(cd->R, cd->t, fingerObs);
                    if (sc < 0) continue;
                    if (sc < b1) { b2 = b1; b1 = sc; sel = cd; }
                    else if (sc < b2) { b2 = sc; }
                }
                // 要求赢得明显：次优必须比最优差 50% 以上，否则这一帧不表态
                if (sel && b2 > b1 * 1.5) { pick = sel; }
                else { out.rmseMm = best.r; out.marginMm = margin;
                       out.nMatched = best.n; out.reason = 3; return out; }
            } else if (pool.size() > 1) {
                out.rmseMm = best.r; out.marginMm = margin;
                out.nMatched = best.n; out.reason = 3; return out;
            }
        }

        if (pick->r > cfg_.maxRmseMm) {
            out.rmseMm = pick->r; out.marginMm = margin; out.nMatched = pick->n;
            out.reason = 4; return out;
        }

        // ---- 持续的手指落点校验（不只是冷启动那一次）----
        out.fingerImplausible = false;
        if (fTmplValid_ && !fingerObs.empty() && cands.size() > 1) {
            const double sPick = fingerScore(pick->R, pick->t, fingerObs);
            if (sPick > 0) {
                double sBest = sPick;
                for (const Cand& cd : cands) {
                    if (cd.n != pick->n || cd.a == pick->a) continue;
                    if (cd.r > pick->r * 2.5 + 1.0) continue;   // 残差离谱的不算备选
                    const double sc = fingerScore(cd.R, cd.t, fingerObs);
                    if (sc > 0 && sc < sBest) sBest = sc;
                }
                out.fingerScoreMm = sPick;
                if (sBest < sPick * cfg_.implausibleRatio) out.fingerImplausible = true;
            }
        }

        out.ok = true;
        out.pointOf = pick->a;
        // 【必须映射回原始下标】空间门控之后 pick->a 里存的是门内数组的下标，
        // 调用方拿它去索引原始点云会取到完全不相干的点 —— 这种错不会崩，
        // 只会让手背标签静默地指向错误的点，是最难查的那类。
        if (gated)
            for (int k = 0; k < 5; ++k)
                if (out.pointOf[size_t(k)] >= 0)
                    out.pointOf[size_t(k)] = nearIdx[size_t(out.pointOf[size_t(k)])];
        out.R = pick->R;
        out.t = pick->t;
        out.rmseMm = pick->r;
        out.marginMm = margin;
        out.nMatched = pick->n;
        out.usedHistory = usedHist;
        out.reason = 0;
        return out;
    }

    // -------------------------------------------------------------------------
    // 贴点重捕：在一次成功的 solve() 之后调用。
    //
    // 对每个【本帧没匹配上】的槽位，在更大的半径里找一个空闲候选点，把它换算成
    // 腕部系的局部坐标累计起来。攒够帧数且散布够小 => 这是一颗刚性固定在手上的
    // 球，只是位置跟模板对不上 => 就地把模板改成实测位置。
    //
    // 槽位一旦重新匹配上（球又被看见了），它的累计缓冲立刻清空 —— 短暂遮挡
    // 不该触发模板修改。
    // -------------------------------------------------------------------------
    RepairEvent observeForRepair(const std::vector<drs::V3>& pts,
                                 const DorsumSolveResult& r) {
        RepairEvent ev;
        if (!cfg_.repairEnabled || !tmplValid_ || !r.ok) return ev;
        // 位姿本身不可信时不能拿来算局部坐标
        if (r.nMatched < 3 || r.rmseMm > cfg_.maxRmseMm) return ev;

        std::vector<char> used(pts.size(), 0);
        for (int k = 0; k < 5; ++k)
            if (r.pointOf[size_t(k)] >= 0 && r.pointOf[size_t(k)] < int(pts.size()))
                used[size_t(r.pointOf[size_t(k)])] = 1;

        for (int k = 0; k < 5; ++k) {
            if (r.pointOf[size_t(k)] >= 0) { rbuf_[size_t(k)].clear(); continue; }

            const drs::V3 pred = xform(r.R, r.t, tmpl_[size_t(k)]);
            int best = -1; double bestD = cfg_.repairSearchMm;
            for (size_t j = 0; j < pts.size(); ++j) {
                if (used[j]) continue;
                const double d = drs::norm(drs::sub(pts[j], pred));
                if (d < bestD) { bestD = d; best = int(j); }
            }
            if (best < 0) { rbuf_[size_t(k)].clear(); continue; }

            // 世界 -> 腕部系局部坐标： Rᵀ(p - t)
            const drs::V3 q = pts[size_t(best)];
            const double dx = q[0]-r.t[0], dy = q[1]-r.t[1], dz = q[2]-r.t[2];
            const drs::V3 loc{{r.R[0]*dx + r.R[3]*dy + r.R[6]*dz,
                               r.R[1]*dx + r.R[4]*dy + r.R[7]*dz,
                               r.R[2]*dx + r.R[5]*dy + r.R[8]*dz}};
            auto& b = rbuf_[size_t(k)];
            b.push_back(loc);
            if (int(b.size()) > cfg_.repairMinFrames * 3)
                b.erase(b.begin(), b.begin() + b.size() / 3);
            if (int(b.size()) < cfg_.repairMinFrames) continue;

            // 散布：刚性固定的球在腕部系里应该几乎不动
            drs::V3 mean{{0,0,0}};
            for (const auto& v : b) for (int c = 0; c < 3; ++c) mean[size_t(c)] += v[size_t(c)];
            for (int c = 0; c < 3; ++c) mean[size_t(c)] /= double(b.size());
            double sc = 0;
            for (const auto& v : b) sc += drs::norm(drs::sub(v, mean));
            sc /= double(b.size());
            if (sc > cfg_.repairMaxScatterMm) continue;   // 不是刚性的，继续观察

            // ---- 安全闸：改模板之前先验，不过就回滚 ----
            ev.slot = k;
            ev.scatterMm = sc;
            ev.frames = int(b.size());
            ev.moveMm = drs::norm(drs::sub(mean, tmpl_[size_t(k)]));

            double minPair = 1e18;
            for (int m = 0; m < 5; ++m) if (m != k)
                minPair = std::min(minPair, drs::norm(drs::sub(mean, tmpl_[size_t(m)])));
            if (minPair < cfg_.repairMinPairMm) {
                ev.rejected = true; ev.why = "新位置跟别的手背点太近，会引入歧义";
                b.clear(); return ev;
            }

            const std::array<drs::V3, 5> save = tmpl_;
            const double saveAmb = selfAmb_;
            std::array<drs::V3, 5> nt = tmpl_;
            nt[size_t(k)] = mean;
            applyTemplateShape(nt);                // 不动 tmpl0_，退回闸才有意义
            if (selfAmb_ < cfg_.repairMinSelfAmbMm) {
                applyTemplateShape(save); selfAmb_ = saveAmb;
                ev.rejected = true; ev.why = "改完之后自歧义会塌，标签将开始翻转";
                b.clear(); return ev;
            }
            ev.newSelfAmbMm = selfAmb_;
            ev.happened = true;
            ev.why = "重贴的球已就位，模板已就地更新";
            b.clear();
            return ev;                             // 一帧最多修一个槽位
        }
        return ev;
    }

    // -------------------------------------------------------------------------
    // 持续微调模板：把每个模板点朝当帧实测的局部坐标拉 alpha。
    // 在 solve() 成功之后调，跟 observeForRepair 互补 ——
    // 那个管"漂太远匹配不上"，这个管"漂一点点还匹配得上"。
    //
    // 【必须消掉规范自由度】只把点朝观测拉的话，整份模板会慢慢连带整体
    // 旋转/平移一起漂（Kabsch 总能把它对上，残差看不出来），而腕部系是由
    // 模板定义的 —— 模板一转，anchor 和 IK 全跟着歪，且没有任何指标会报警。
    // 所以拉完之后再用 Kabsch 把新模板对回标定值：形状的变化保留，
    // 位姿的漂移消掉。
    // -------------------------------------------------------------------------
    bool refineTemplate(const std::vector<drs::V3>& pts, const DorsumSolveResult& r) {
        if (!cfg_.refineEnabled || !tmplValid_ || !r.ok) return false;
        if (r.nMatched < cfg_.refineMinInliers) return false;
        if (!(r.rmseMm >= 0.0 && r.rmseMm <= cfg_.refineMaxRmse)) return false;

        std::array<drs::V3, 5> nt = tmpl_;
        int n = 0;
        for (int k = 0; k < 5; ++k) {
            const int j = r.pointOf[size_t(k)];
            if (j < 0 || j >= int(pts.size())) continue;
            // 世界 -> 模板系： Rᵀ(p - t)
            const double dx = pts[size_t(j)][0] - r.t[0];
            const double dy = pts[size_t(j)][1] - r.t[1];
            const double dz = pts[size_t(j)][2] - r.t[2];
            const drs::V3 loc{{r.R[0]*dx + r.R[3]*dy + r.R[6]*dz,
                               r.R[1]*dx + r.R[4]*dy + r.R[7]*dz,
                               r.R[2]*dx + r.R[5]*dy + r.R[8]*dz}};
            for (int c = 0; c < 3; ++c)
                nt[size_t(k)][size_t(c)] +=
                    cfg_.refineAlpha * (loc[size_t(c)] - nt[size_t(k)][size_t(c)]);
            ++n;
        }
        if (n < cfg_.refineMinInliers) return false;

        // 消规范自由度：把新模板刚性对回标定模板
        {
            std::vector<drs::V3> a(nt.begin(), nt.end()), b(tmpl0_.begin(), tmpl0_.end());
            drs::M9 Rg{}; drs::V3 tg{};
            if (drs::kabschHorn(a, b, Rg, tg) >= 0)
                for (int k = 0; k < 5; ++k) nt[size_t(k)] = xform(Rg, tg, nt[size_t(k)]);
        }

        // 安全闸：累计漂移不能超上限，自歧义不能塌
        double maxD = 0;
        for (int k = 0; k < 5; ++k)
            maxD = std::max(maxD, drs::norm(drs::sub(nt[size_t(k)], tmpl0_[size_t(k)])));
        if (maxD > cfg_.refineMaxDriftMm) return false;

        const std::array<drs::V3, 5> save = tmpl_;
        const double saveAmb = selfAmb_;
        applyTemplateShape(nt);          // 不动 tmpl0_
        if (selfAmb_ < cfg_.repairMinSelfAmbMm) {
            applyTemplateShape(save); selfAmb_ = saveAmb;
            return false;
        }
        return true;
    }

    // 没匹配上的手背槽位，用刚体位姿把模板点投出来（比网络 pos 头准，
    // 因为这一帧的刚体位姿刚刚被别的点解出来了）。调用方负责把 observed
    // 置为 false —— 这是"补出来的"，不是观测。
    drs::V3 project(const drs::M9& R, const drs::V3& t, int slot) const {
        return xform(R, t, tmpl_[size_t(slot)]);
    }

private:
    static drs::V3 xform(const drs::M9& R, const drs::V3& t, const drs::V3& p) {
        return drs::V3{{R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + t[0],
                     R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + t[1],
                     R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + t[2]}};
    }

    // trace(AᵀB) = A 与 B 的逐元素点积
    static double rotAngleDeg(const drs::M9& A, const drs::M9& B) {
        double tr = 0.0;
        for (int q = 0; q < 9; ++q) tr += A[size_t(q)] * B[size_t(q)];
        const double c = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
        return std::acos(c) * 57.29577951308232;
    }

    // 全局最近优先的一对一匹配（5x M 很小，排序法足够，不需要匈牙利）
    static void greedyMatch(const std::array<drs::V3, 5>& proj,
                            const std::vector<drs::V3>& pts, int M, double tol,
                            std::array<int, 5>& asg) {
        struct E { double d; int k; int j; };
        std::vector<E> es;
        es.reserve(size_t(5 * M));
        for (int k = 0; k < 5; ++k)
            for (int j = 0; j < M; ++j) {
                const double d = drs::norm(drs::sub(pts[size_t(j)], proj[size_t(k)]));
                if (d <= tol) es.push_back(E{d, k, j});
            }
        std::sort(es.begin(), es.end(), [](const E& x, const E& y) { return x.d < y.d; });
        std::vector<char> usedPt(size_t(M), 0);
        for (const E& e : es) {
            if (asg[size_t(e.k)] >= 0 || usedPt[size_t(e.j)]) continue;
            asg[size_t(e.k)] = e.j;
            usedPt[size_t(e.j)] = 1;
        }
    }

    // 候选位姿的"手指落点合理度"：把观测到的手指点转到候选腕部系，
    // 跟中立位模板比。越小越像。返回 -1 表示样本不足。
    double fingerScore(const drs::M9& R, const drs::V3& t,
                       const std::vector<FingerObs>& obs) const {
        double s = 0.0; int n = 0;
        for (const FingerObs& o : obs) {
            const int idx = o.label - 5;
            if (idx < 0 || idx >= 15) continue;
            const drs::V3 p = xform(R, t, fTmpl_[size_t(idx)]);
            s += drs::norm(drs::sub(p, o.pos));
            ++n;
        }
        return (n >= 3) ? (s / double(n)) : -1.0;
    }

    double computeSelfAmbiguity() const {
        std::array<int, 5> perm{{0, 1, 2, 3, 4}};
        std::vector<drs::V3> src(5), dst(5);
        for (int i = 0; i < 5; ++i) src[size_t(i)] = tmpl_[size_t(i)];
        double best = 1e18;
        drs::M9 R{}; drs::V3 t{};
        do {
            bool ident = true;
            for (int i = 0; i < 5; ++i) if (perm[size_t(i)] != i) { ident = false; break; }
            if (ident) continue;
            for (int i = 0; i < 5; ++i) dst[size_t(i)] = tmpl_[size_t(perm[size_t(i)])];
            const double r = drs::kabschHorn(src, dst, R, t);
            if (r >= 0 && r < best) best = r;
        } while (std::next_permutation(perm.begin(), perm.end()));
        return best;
    }

    Config cfg_{};
    std::array<drs::V3, 5> tmpl_{};
    std::array<drs::V3, 5> tmpl0_{};      // 标定时那一份，退回用
    int bad_ = 0;                          // 连续"解不好"的帧数
    int implaus_ = 0;                      // 连续"手指落点说锁错了"的帧数
    std::array<double, 25> td_{};
    bool tmplValid_ = false;
    std::array<drs::V3, 15> fTmpl_{};
    bool fTmplValid_ = false;
    double selfAmb_ = -1.0;

    mutable drs::M9 prevR_{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    mutable bool hasPrev_ = false;
    mutable drs::V3 prevT_{};
    mutable bool hasPrevT_ = false;
    // 每个槽位的"疑似重贴球"局部坐标累计
    std::array<std::vector<drs::V3>, 5> rbuf_;
};

}  // namespace hm20
}  // namespace mocap
