// =============================================================================
// Hm20IkRefiner.hpp —— IHm20IkRefiner 的具体实现
// =============================================================================
// 【它解决什么】
// HandSkeletonAssociator::computeSegmentQuats() 现在拿 marker[b]-marker[a] 当
// 骨轴。但反光球贴在指节【背侧皮肤】上，球心离骨轴有 11~17mm 的径向偏置，
// 而且相邻两节的偏置方位角不同 —— 两球连线【必然】偏离骨轴。按训练数据
// (hand_rig.py) 的实际参数蒙特卡洛：
//     骨轴夹角偏差  中位 10.2°   p90 21.6°   最大 53.3°
// 这是【系统偏差】，标签 100% 正确也消不掉，滤波也去不掉。
// 做动捕采集，手指姿态带十几度固定偏差不能接受。
//
// 正确路径：Kabsch 解腕部位姿 -> 逐指 LM 解关节角 -> FK 拿 16 个分段朝向。
// 这样骨长恒定、关节在限位内、父子分段首尾相接，四元数天然自洽，
// 被遮挡的点也不是"猜一个位置"而是"按这只手的尺寸摆出来"。
//
// =============================================================================
// 【关键：marker 模型必须跟训练数据一致，否则 IK 在拟合一个错的东西】
// =============================================================================
// hand/HandModel.hpp 的 fingerFK 把 marker 放在【骨中点】、径向偏置为 0：
//     out[0] = (T * transMat(Lp/2, 0, 0)).translation();
// 而训练语料 (hand_rig.py forward_kinematics) 用的是：
//     local = [L - u,  r*sin(psi),  r*cos(psi)]
// 两处差异都是系统性的：
//   · 轴向：(L-u)/L 实测均值 0.632，不是 0.5
//   · 径向：r ≈ 11~16mm，不是 0        <- 这一项是那 10° 偏差的主因
// 直接用现有 fingerFK 做 IK，等于让 IK 去拟合一个跟观测模型不符的预测，
// 残差消不掉，白做。所以这里【重写】了 markerFK。
//
// =============================================================================
// 【个体参数拿不到，只能用群体均值 —— 这是精度的硬下界】
// =============================================================================
// u / r / psi 是逐人逐指的量，而 HandCalibration.hpp 里写明手指的 anchor+骨长
// "两个方程、三个未知数"做不到自动标定，HandTemplateStore 也只存 backMarkers。
// 所以这里用 hand_rig.py 采样分布的均值代替。实测代价：
//     marker 位置残差  中位 4.61mm   p90 7.94mm
// 这是"不标定手指"的理论下界，IK 再准也降不到这以下。
// 但它仍然远好于现状：4.6mm 的位置残差对应的角度误差约 3~5°，
// 而 marker->marker 当骨轴是 10.2° 中位、21.6° p90。
//
// 想再往下压只有一条路：让用户做一次手指标定（张开手 + 握拳两个姿势，
// 用 FingerKinematicCalib 反解 u/r），能把残差降到 1~2mm。是否值得由你定。
// =============================================================================
#pragma once

#include <array>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

#include "estimate/Hm20AssocContract.hpp"   // 【只要类型，不要那个类】本文件一次都没提到
                                            // Hm20SkeletonAssociator，要的只是 SkeletonFrameResult
                                            // 这些类型。注意原来这行没写 estimate/ 前缀，
                                            // 靠的是"同目录"解析——批量检查 include 时很容易漏掉。

namespace mocap {
namespace hm20 {

// 【命名空间】Vec3/Mat3/Quat/kNumSegments/IHm20IkRefiner 在 mocap::hm20，
// 而 add/sub/mul/dot/norm/matToQuat 在 mocap::hm20::detail —— 见
// HandSkeletonAssociator.hpp。放错层的症状是满屏 "'Mat3' does not name a type"
// + "did you mean 'mocap::hm20::detail::mul'"，而且 M3 一坏，下面所有用到它的
// 成员(segR 等)都跟着消失，报错会放大到几十条，但根因只有这一个。
using namespace detail;

// -----------------------------------------------------------------------------
// 贴点模型的群体均值（来自 hand_rig.py 采样 20000 个受试者的统计）
// -----------------------------------------------------------------------------
struct Hm20MarkerModel {
    // 回退量占骨长的比例。marker 距【近端】关节的轴向距离 = (1 - uRatio) * L
    // 实测逐指逐节均值；C++ 原来的 fingerFK 相当于 uRatio 恒等于 0.5
    static constexpr double kURatio[5][3] = {
        {0.243, 0.511, 0.502},   // thumb  MC/PP/DP
        {0.273, 0.417, 0.351},   // index  PP/MP/DP
        {0.263, 0.473, 0.397},   // middle
        {0.268, 0.458, 0.390},   // ring
        {0.275, 0.361, 0.332},   // pinky
    };
    // 径向偏置基准值(mm)，对应手长 185mm；实际按 handLen/185 线性缩放。
    // = 骨半径 + 软组织 + 球座 + 球半径 + 手套
    static constexpr double kRadialMm[5][3] = {
        {15.60, 14.29, 13.10},
        {14.29, 12.99, 12.00},
        {14.60, 13.20, 12.09},
        {14.10, 12.80, 11.81},
        {13.10, 12.00, 11.30},
    };
    // 径向偏置的方位角(rad)，0 = 正背侧。拇指的球通常偏桡背侧。
    static constexpr double kAzimuth[5] = {-0.553, -0.004, -0.001, 0.003, 0.001};
    // 皮肤滑移：关节弯曲时球相对骨头往远端滑、并被抬高
    static constexpr double kSkinDu = 1.50;   // mm/rad，u 减小
    static constexpr double kSkinDr = 0.60;   // mm/rad，r 增大
    // 拇指 CMC 外展时掌骨的轴向旋前耦合系数
    static constexpr double kThumbAxialK = 0.576;

    double handLenMm = 185.0;   // 由手背模板估计，见 estimateHandLen()

    // 拇指 CMC 外展 -> 掌骨轴向旋前的耦合系数。
    // 【这是逐人参数，不是常量】训练语料里它在 0.35~0.80 之间逐人随机，
    // 而这里原来写死 0.576。写死的代价不是"稍微不准"：外展 1 rad 时轴向
    // 偏差可到 0.22 rad，rotX 之后再复合 rotY(近节屈曲)，远节骨轴被歪掉
    // 约 11°、dp 球位置错 6mm 以上。球面拟合会把这部分当噪声吸收进球心 ——
    // 这就是拇指 anchor 误差(14.8mm)比四指(2.6~3.8mm)差 4 倍的主因。
    // 由 Hm20AutoCalib 的束调整解出来，解不出来时退回这个群体均值。
    double thumbAxialK = 0.576;

    // 第一掌骨的【常数】解剖旋前(rad)。0 = 完全沿用旧行为。
    //
    // 【rig 缺了什么】训练用的 hand_rig.py 里旋前完全耦合在 CMC 外展上
    // (axial = thumb_axial_k * a[1])，没有常数基线：握拳时 CMC展=0.18、
    // k∈[0.35,0.80] -> 旋前只有 4~8°。而真人第一掌骨【中立位就有】80~90°
    // 旋前 —— 这正是拇指能对掌的原因。
    //
    // 【实测】拿 hm20_v7.onnx 的 seg_rot6d 头量它自己学到的屈曲铰链轴
    // (tools/hm20_diag/probe_thumb_axis.py，60 组自洽样本)：
    //     四指  与 +Y 轴差 0.3~1.7°，离散度 1.7~4.2°
    //     拇指  与 +Y 轴差 17.4°，   离散度 29.0°
    // 真人拇指应该差 80~90°，缺口约 73°。所以模型和 rig 都把拇指当成了
    // 第五根手指，往掌面压但不横过手掌 —— 这就是"外翻/不往手心弯"。
    //
    // 【后果不只是姿态难看】markerFK 摆不出一个对掌的拇指，于是拇指 IK 的
    // 残差常年超 ikMaxRmseMm、fingerValid[0] 恒 false(面板上的"拇✗")，
    // 而且自标定的球面拟合会把这 73° 的系统误差当噪声吸进球心 ——
    // 这就是"拇指 anchor 误差 9~14mm、四指只有 3mm"的谜底。
    //
    // 【符号】refine() 内部已把左手镜像成右手系再解(obsL/anchorL/backL 一起
    // 翻 Y)，所以这个常数在 markerFK 里【跟手性无关】，不要在这里翻号。
    // 幅值 ~1.4~1.57 rad。符号请用 ThumbPronationCalib 扫出来，别靠眼睛。
    double thumbPronation0 = 0.0;
};

// -----------------------------------------------------------------------------
// 关节限位(rad)，跟 pose_prior.py 的 LIMITS_LO/HI 一致
//   非拇指 [MCP屈, MCP展, PIP屈, DIP屈]
//   拇指   [CMC屈, CMC展, MCP屈, IP屈]
// -----------------------------------------------------------------------------
inline const double (&ikLimitLo())[5][4] {
    static const double v[5][4] = {
        {-0.35, -0.17, -0.17, -0.26},
        {-0.35, -0.44,  0.00, -0.17},
        {-0.30, -0.28,  0.00, -0.17},
        {-0.30, -0.28,  0.00, -0.17},
        {-0.35, -0.48,  0.00, -0.17},
    };
    return v;
}
inline const double (&ikLimitHi())[5][4] {
    static const double v[5][4] = {
        {1.00, 1.15, 0.95, 1.40},
        {1.60, 0.44, 1.92, 1.40},
        {1.60, 0.28, 1.92, 1.40},
        {1.60, 0.28, 1.92, 1.40},
        {1.62, 0.48, 1.92, 1.40},
    };
    return v;
}

// -----------------------------------------------------------------------------
// 小工具：3x3 旋转
// -----------------------------------------------------------------------------
// Mat3 已在 HandSkeletonAssociator.hpp 里定义为 std::array<double,9>（行主序）
using M3 = Mat3;

inline M3 rotY(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {c, 0, s,  0, 1, 0,  -s, 0, c};
}
inline M3 rotZ(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {c, -s, 0,  s, c, 0,  0, 0, 1};
}
inline M3 rotX(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {1, 0, 0,  0, c, -s,  0, s, c};
}
inline M3 mul(const M3& A, const M3& B) {
    M3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += A[size_t(i * 3 + k)] * B[size_t(k * 3 + j)];
            C[size_t(i * 3 + j)] = s;
        }
    return C;
}
inline M3 mul3(const M3& A, const M3& B) {
    M3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += A[size_t(i*3+k)] * B[size_t(k*3+j)];
            C[size_t(i*3+j)] = s;
        }
    return C;
}
inline Vec3 applyRot(const M3& R, const Vec3& v) {
    return { R[0]*v[0] + R[1]*v[1] + R[2]*v[2],
             R[3]*v[0] + R[4]*v[1] + R[5]*v[2],
             R[6]*v[0] + R[7]*v[1] + R[8]*v[2] };
}

// -----------------------------------------------------------------------------
// 逐指 FK —— 【必须跟 hand_rig.py forward_kinematics 逐行一致】
//
//   markers[j] = o_j + R_j * [ L_j - u_j,  r_j*sin(psi),  r_j*cos(psi) ]
//   o_{j+1}    = o_j + R_j * [ L_j, 0, 0 ]
//   u_j = uRatio*L_j - kSkinDu*|theta_j|      (夹在 [0.01L, 0.95L])
//   r_j = radial*(handLen/185) + kSkinDr*|theta_j|
//
// 返回 3 个 marker 位置 + 3 个分段旋转（供 segQuat 用）
// -----------------------------------------------------------------------------
struct FingerFkOut {
    std::array<Vec3, 3> marker;
    std::array<M3, 3>   segR;
    std::array<Vec3, 3> segO;
};

inline FingerFkOut markerFK(int finger,
                            const Vec3& anchor,
                            const std::array<double, 3>& L,
                            const std::array<double, 4>& q,
                            const Hm20MarkerModel& mm) {
    FingerFkOut out{};
    const double scale = mm.handLenMm / 185.0;
    const double psi = Hm20MarkerModel::kAzimuth[size_t(finger)];

    // 根关节旋转：先外展(Z)再屈曲(Y)。拇指额外带 CMC 外展耦合出的轴向旋前。
    M3 R = mul(rotZ(q[1]), rotY(q[0]));
    if (finger == 0)
        R = mul(R, rotX(mm.thumbPronation0 + mm.thumbAxialK * q[1]));

    // 该段【远端】关节角，决定皮肤滑移量
    const double jointAng[3] = {q[0], q[2], q[3]};

    Vec3 o = anchor;
    for (int j = 0; j < 3; ++j) {
        if (j > 0) R = mul(R, rotY(q[size_t(j + 1)]));
        out.segR[size_t(j)] = R;
        out.segO[size_t(j)] = o;

        const double th = std::fabs(jointAng[j]);
        double u = Hm20MarkerModel::kURatio[size_t(finger)][size_t(j)] * L[size_t(j)]
                 - Hm20MarkerModel::kSkinDu * th;
        u = std::clamp(u, 0.01 * L[size_t(j)], 0.95 * L[size_t(j)]);
        double r = Hm20MarkerModel::kRadialMm[size_t(finger)][size_t(j)] * scale
                 + Hm20MarkerModel::kSkinDr * th;
        r = std::max(r, 1.0);

        // 【关键差异】不是 transMat(L/2,0,0)：轴向是 L-u，且有径向偏置
        const Vec3 local{ L[size_t(j)] - u, r * std::sin(psi), r * std::cos(psi) };
        out.marker[size_t(j)] = add(o, applyRot(R, local));

        o = add(o, applyRot(R, Vec3{L[size_t(j)], 0, 0}));
    }
    return out;
}

// -----------------------------------------------------------------------------
// 逐指 Levenberg-Marquardt
//
// 残差 = FK(marker) - 观测(marker)  +  限位罚项 + 时序先验 + DIP 耦合先验
// 4 个未知数、最多 9 个观测方程，用有限差分雅可比，规模极小。
// -----------------------------------------------------------------------------
struct IkOptions {
    int    iters       = 30;
    double wPrior      = 0.05;   // 上一帧解的先验权重（遮挡严重时唯一的锚）
    double wLimit      = 6.0;    // 超限位的罚权重
    double wCouple     = 0.5;    // DIP ≈ 0.72*PIP (Landsmeer)
    double dipCoupling = 0.72;
    double eps         = 1e-4;   // 有限差分步长
    bool   multiStart  = true;   // 冷启动时用 4 个起点（摊平/放松/半握/全握）

    // ---- 外展(MCP展)的额外约束 ----
    //
    // 【实测问题】录制里 MCP展 的限位命中率：食指 100%、小指 100%、
    // 中指 97.6%、拇指 88.6%——常年顶死在限位上，而且实际值还【越界】了
    // （食指限位 ±25.2°，实测恒定 28.1°；小指限位 -27.5°，实测 -30.4°）。
    // 越界还停在那里，说明 wLimit=6.0 这个软罚被 marker 拟合项直接压过去了。
    //
    // 【为什么偏偏是外展被滥用】外展是这 4 个自由度里"最不受观测约束"的一个：
    // 遮挡时 PIP/DIP 没有数据、被先验拉住不能动，优化器要让 FK 出的那颗
    // 唯一可见 marker 贴近观测，就只剩外展这一个还能自由活动的方向可用。
    // 于是它被当成万能补偿项，把本该由 PIP 屈曲承担的运动全接过去——
    // 表现就是"指尖侧弯到不合理的程度"。
    //
    // 【两层修复】
    //  ① 外展越界时用显著更硬的罚（wAbductionLimit），让它真的越不出去，
    //     而不是像现在这样"罚了但压不住"。
    //  ② 观测不足时直接把外展【锁】在上一帧的值附近（wAbductionLock）——
    //     没有观测数据支撑的自由度，本来就不该自由活动。锁住之后，
    //     优化器要拟合那颗可见 marker 就只能去动屈曲，这正是我们想要的。
    double wAbductionLimit = 40.0;  // 外展越界罚权重。远高于通用 wLimit=6.0——
                                     // 通用值对屈曲够用（屈曲有 marker 数据
                                     // 约束着，不容易跑飞），但外展在遮挡时
                                     // 几乎不受约束，需要硬得多的边界。
    double wAbductionLock  = 2.5;   // 观测不足时把外展钉在先验附近的权重。
                                     // 比 wPrior=0.05 高两个量级——那个是给
                                     // 所有自由度的弱时序先验，这里要的是
                                     // "这个自由度本帧没有信息，别乱动"。
    int    abductionLockMinObs = 2; // 观测数少于这个值就锁外展。设 2 是因为
                                     // 1 个 marker(3个方程)约束 4 个自由度必然
                                     // 欠定；2 个 marker(6个方程)才勉强够。

    // ---- MCP->PIP 耦合（遮挡时的兜底，不是解剖学硬约束）----
    //
    // 【为什么要加这个】实测发现：当一根手指只有掌指关节附近那一颗 marker
    // 能看到（中节、指尖两颗被自身遮挡——手指越弯、越容易挡住自己，
    // 握拳恰恰是这种情况最严重的时候），PIP/DIP 完全没有观测数据，
    // 4 自由度只剩 wPrior/wLimit 在起作用，两者都把角度往"接近伸直"拉，
    // 于是 PIP 冻结在 0° 附近——不管这根手指此刻实际弯到多少度。
    // 而优化器为了让 FK 出的那一颗 marker 尽量贴近观测位置，会把本该由
    // PIP 承担的运动挪到 MCP 展（侧弯）上去凑，这就是"手指侧弯到不合理
    // 程度"的直接成因。
    //
    // 【为什么在线学习，不用固定常数】跟 dipCoupling 不一样，MCP-PIP 之间
    // 没有 Landsmeer 韧带那种硬解剖耦合，因人、因手势而异（"钩握"就是
    // PIP 大弯、MCP 几乎不动的反例），编一个死常数等于又犯一次"没验证的
    // 默认值"的错。
    //
    // 【必须是带截距的回归，不能过原点】——这是实测踩出来的坑：第一版用
    // 过原点最小二乘 pip≈k·mcp，用真实录制验证时符号是反的（学出 -1.7，
    // 而皮尔逊相关系数明明是 +0.91）。根因是 q[0]=0 这个坐标零点，对不同
    // 手指来说都不对应"手指完全伸直"这个解剖学零点——有的手指静止基准角
    // 本身就是负的。过原点假设"mcp=0时pip必须=0"，这个假设不成立，
    // 拿它去拟合非零基准的数据，斜率符号会被基准偏移带偏。改用标准线性
    // 回归 pip≈k·mcp+b（协方差法），这样不管两个自由度各自的零点在哪，
    // 拟合出来的斜率符号反映的都是"真实的变化趋势"。
    //
    // 【必须做方差门控】哪怕符号修对了，斜率对"训练时观测到的 mcp range"
    // 极度敏感——某次录制里如果 mcp 只在很窄的角度范围里活动（这次实测
    // 中指只有 10.7° 的跨度），线性回归会给出一个陡到离谱的斜率
    // （实测 k≈8.9，即 1° MCP 变化对应近 9° PIP 目标变化），一旦遮挡时
    // MCP 解到训练范围之外，外推出的 PIP 目标可能远超关节实际能达到的
    // 角度。所以斜率必须在"观测到的 mcp 标准差够大"时才启用，不够时保持
    // 不加这条残差（等价于回退到修复前的行为——不会比不修复更差）。
    //
    // 【必须有衰减，不能终身累加】这个累加器原来是全会话终身累加、从不
    // 清零的，实测同一个 app 运行期间录了 5 段不同动作，早期探索性/标定
    // 阶段的数据永远洗不掉、样本数只涨不消，新数据的影响被摊得越来越薄。
    // 改成指数衰减的加权累加，让近期数据主导估计，陈旧数据自然淡出。
    //
    // 【必须排除拇指】拇指是 CMC 关节主导，跟其余四指的 MCP 运动学完全
    // 不是一回事，已经有专门的轴向/常数旋前标定路径（thumbAxialK /
    // thumbPronation0），不该再套一层通用的 MCP->PIP 耦合。
    //
    // 见 Hm20IkRefiner::updateMcpPipCoupling() / mcpPipCoupling() 的实现。
    double wMcpCouple  = 0.35;   // 比 wCouple(DIP<-PIP的0.5) 更保守——
                                  // MCP-PIP 协同不是硬解剖耦合，兜底力度
                                  // 要弱于真正的腱索约束，避免在"钩握"这类
                                  // 独立弯曲的手势上把 PIP 硬拽向错误角度。
    int    mcpCouplingMinSamples = 30;   // 至少这么多"三点全见"的帧才开始
                                          // 信任在线学出来的比例
    double mcpCouplingMinMcpStdRad = 0.09; // 约5°。mcp的观测标准差小于这个
                                            // 就不启用回归（避免窄范围外推）
    double mcpCouplingKMin = 0.0;   // 学出来的斜率钳制下限——负值意味着
                                     // 往错误方向拉，比不拉还糟，直接不信
    double mcpCouplingKMax = 3.0;   // 上限——防止陡峭到失控的外推
    double mcpCouplingDecay = 0.999; // 指数衰减系数，约合"记住最近上千个
                                      // 有效样本"，更早的数据影响指数淡出
};

class Hm20IkRefiner : public IHm20IkRefiner {
public:
    explicit Hm20IkRefiner(IkOptions opt = {}) : opt_(opt) {}

    // 当前对每根手指学到的 MCP->PIP 耦合斜率（协方差法，带截距）。
    // 【返回 0 意味着"这条兜底还没准备好，不生效"】不是意味着"学出了0这个
    // 斜率"——没准备好时（样本不够 / mcp 观测范围太窄）残差里根本不会加
    // 这一项，见 residuals() 里的调用方式，返回 0 只是给一个安全的默认。
    // 拇指(finger==0)恒返回 0——它是 CMC 关节主导，运动学跟其余四指不是
    // 一回事，不该套用这个兜底。
    double mcpPipCouplingSlope(int finger) const {
        if (finger == 0) return 0.0;
        return mcpCouplingReady(finger) ? mcpCoupleFit_[size_t(finger)].slope(opt_) : 0.0;
    }
    // 配套的截距——必须跟斜率一起用，见 residuals() 里的 (k*mcp+b)。
    double mcpPipCouplingIntercept(int finger) const {
        if (finger == 0) return 0.0;
        return mcpCouplingReady(finger) ? mcpCoupleFit_[size_t(finger)].intercept(opt_) : 0.0;
    }
    bool mcpCouplingReady(int finger) const {
        if (finger == 0) return false;
        const auto& s = mcpCoupleFit_[size_t(finger)];
        if (s.n < opt_.mcpCouplingMinSamples || s.sumW < 1e-6) return false;
        const double meanMcp = s.sumMcp / s.sumW;
        const double varMcp  = std::max(0.0, s.sumMcpMcp / s.sumW - meanMcp * meanMcp);
        return std::sqrt(varMcp) >= opt_.mcpCouplingMinMcpStdRad;
    }
    int mcpPipCouplingSamples(int finger) const { return mcpCoupleFit_[size_t(finger)].n; }

    // 由手背模板估计手长，用来缩放径向偏置。
    // 手背 5 点的展布跟手长强相关；没有更好的来源时这是可用的近似。
    void setHandLenFromBackTemplate(const std::array<Vec3, 5>& back) {
        Vec3 c{0, 0, 0};
        for (const auto& p : back) c = add(c, p);
        c = mul(c, 1.0 / 5.0);           // HandSkeletonAssociator 里叫 mul 不叫 scale
        double s = 0;
        for (const auto& p : back) s += norm(sub(p, c));
        s /= 5.0;
        // 实测 hand_rig：手背 5 点平均半径 ≈ 0.115 * handLen
        mm_.handLenMm = std::clamp(s / 0.115, 150.0, 220.0);
    }

    // 左右手。默认右手。写反的后果见 refine() 里镜像那段的说明。
    void setHandedness(bool isRight) { isRight_ = isRight; }
    // 拇指轴向旋前耦合系数（逐人）。见 Hm20MarkerModel::thumbAxialK 的说明。
    void setThumbAxialK(double k) { mm_.thumbAxialK = std::clamp(k, 0.20, 1.00); }
    double thumbAxialK() const { return mm_.thumbAxialK; }
    // 拇指第一掌骨的常数解剖旋前。见 Hm20MarkerModel::thumbPronation0。
    // 【跟 thumbRollOffsetRad 的区别】那个作用在 roll 上，而 roll 在每节只有
    // 1 颗球时【不可观测】，所以永远没有数字能告诉你调对没有；这个作用在
    // marker 位置上，是【可观测】的 —— fingerIkRmseMm[0] 就是它的判据。
    void setThumbPronation0(double rad) { mm_.thumbPronation0 = std::clamp(rad, -2.0, 2.0); }
    double thumbPronation0() const { return mm_.thumbPronation0; }
    bool isRight() const { return isRight_; }

    void setFingerParams(int f, const Vec3& anchor, const std::array<double, 3>& len) {
        anchor_[size_t(f)] = anchor;
        len_[size_t(f)]    = len;
        if (f == 4) paramsReady_ = true;      // 五指都设过了才允许 refine
    }


    // =========================================================================
    // IHm20IkRefiner 的实现 —— 这是 HandSkeletonAssociator 真正调用的入口
    // =========================================================================
    // markers 是【世界系】的 20 点，先用 wristR/wristT 转进腕部系再 IK，
    // 解完再转回世界系。segQuat 同理要左乘 wristR。
    //
    // 【只覆盖 fingerValid 的手指】某指 3 个 marker 全被遮挡时 IK 没有任何
    // 观测约束，解只由先验/限位决定（=中立位），跟真实姿势可能差 175°。
    // 那种情况下保留网络 pos 头的预测更安全 —— 它至少见过训练分布。
    bool refine(std::array<Vec3, 20>& markers, const std::array<bool, 20>& observed,
                const Mat3& wristR, const Vec3& wristT,
                std::array<Quat, kNumSegments>& segQuat) override {
        if (!paramsReady_) return false;

        // 世界系 -> 腕部系：p_w = R * p_l + t  =>  p_l = Rᵀ (p_w - t)
        std::array<Vec3, 20> obsL{};
        for (int m = 0; m < 20; ++m) {
            const Vec3 d = sub(markers[size_t(m)], wristT);
            obsL[size_t(m)] = { wristR[0]*d[0] + wristR[3]*d[1] + wristR[6]*d[2],
                                wristR[1]*d[0] + wristR[4]*d[1] + wristR[7]*d[2],
                                wristR[2]*d[0] + wristR[5]*d[1] + wristR[8]*d[2] };
        }

        // 【左手镜像】markerFK 里 rotZ(外展)·rotY(屈曲) 和径向偏置的方位角 psi
        // 都是按【右手】写的（跟训练用的 hand_rig 一致 —— 它对左手的做法就是
        // 拿右手解算完再镜像）。不做这一步的后果不是"稍微不准"：外展方向整个
        // 反了，握拳并指这类姿势根本拟合不出来，残差常年 10mm 以上、被残差门
        // 挡掉，表现为"左手 IK 永远不生效"。
        // 镜像矩阵 M = diag(1,-1,1)，M = Mᵀ = M⁻¹，所以来回都是它。
        //
        // 【anchor 和手背模板必须一起翻】—— 原来只翻了观测，这是个真 bug：
        // anchor_/backTemplate_ 存的是【腕部系】里的量（自标定判成左手时，
        // 那个系的 Y 已经被翻成小指侧了），而 obsL 在这里又被翻回"拇指侧"。
        // 两者差一个 Y 反号，IK 从第一次迭代起就在拟合一个错位的目标，残差
        // 常年 10mm 以上、被残差门挡掉 —— 症状正是上面注释想避免的那个
        // "左手 IK 永远不生效"。翻观测只做了一半。
        // M = diag(1,-1,1) 是对合的，所以三者用同一个操作翻到同一个系里。
        std::array<Vec3, 5> anchorL = anchor_;
        std::array<Vec3, 5> backL   = backTemplate_;
        if (!isRight_) {
            for (int m = 0; m < 20; ++m) obsL[size_t(m)][1] = -obsL[size_t(m)][1];
            for (int i = 0; i < 5; ++i) { anchorL[size_t(i)][1] = -anchorL[size_t(i)][1];
                                          backL[size_t(i)][1]   = -backL[size_t(i)][1]; }
        }
        // solve() 走成员 anchor_，所以临时换进去、解完换回来。
        const auto anchorSave = anchor_;
        anchor_ = anchorL;
        // 【先把先验存下来】下一行 solve 之后 prevAngles_ 就被本帧解覆写了，
        // 而"这一帧是从哪儿起步的"是判断收敛问题的必要信息 ——
        // 起点离解很远 + 迭代跑满 = 多起点没生效或时序先验被污染了。
        const auto priorSnapshot = prevAngles_;
        const Result R = solve(obsL, observed, prevAngles_, backL);
        anchor_ = anchorSave;
        if (!R.ok) return false;
        prevAngles_ = R.angles;      // 时序先验：下一帧从这里起步，避免多起点搜索

        // 逐指状态回传给调用方。【调用方必须看它】——见 IkFrameInfo 的说明。
        //
        // 【v5：把内部量一起传出去】原来只传三项，而遮挡 marker 的位置正是
        // 下面那些角 FK 出来的 —— 位置不对时要往上查一级，查的就是它们。
        // 尤其 isRight：refine() 内部按它把左手镜像成右手系再解，它跟
        // tmpl_.isRight 是两条独立的下发路径，不一致时拇指会单独歪。
        // 那种情况下上面三项全部正常，从文件里查不出任何异常。
        for (int f = 0; f < 5; ++f) {
            const size_t uf = size_t(f);
            lastInfo_.fingerValid[uf] = R.fingerValid[uf];
            lastInfo_.rmseMm[uf]      = R.rmseMm[uf];
            lastInfo_.nObs[uf]        = R.nObs[uf];
            lastInfo_.cost[uf]        = R.cost[uf];
            lastInfo_.iters[uf]       = R.iters[uf];
            lastInfo_.solved[uf]      = R.solved[uf];
            lastInfo_.q[uf]           = R.angles[uf];
            lastInfo_.qPrior[uf]      = priorSnapshot[uf];
            for (int k = 0; k < 4; ++k) {
                lastInfo_.limitLo[uf][size_t(k)] = ikLimitLo()[f][k];
                lastInfo_.limitHi[uf][size_t(k)] = ikLimitHi()[f][k];
                // 【留 1e-3 余量】求解时限位是带 ±0.05 松弛的（见 lmSolve 里的
                // clamp），严格相等判定会漏掉所有真正顶住的情况。
                const double v = R.angles[uf][size_t(k)];
                lastInfo_.limitHit[uf][size_t(k)] =
                    uint8_t(v <= ikLimitLo()[f][k] + 1e-3 || v >= ikLimitHi()[f][k] - 1e-3);
            }
            // 【存镜像回来之前的腕部系值】anchorL/lenL 是 refine() 内部按手性
            // 镜像过的版本，存那个才对得上上面的 q。
            lastInfo_.anchorMm[uf]  = anchorL[uf];
            lastInfo_.boneLenMm[uf] = len_[uf];
        }
        lastInfo_.handLenMm       = mm_.handLenMm;
        lastInfo_.thumbAxialK     = mm_.thumbAxialK;
        lastInfo_.thumbPronation0 = mm_.thumbPronation0;
        lastInfo_.wPrior      = opt_.wPrior;
        lastInfo_.wLimit      = opt_.wLimit;
        lastInfo_.wCouple     = opt_.wCouple;
        lastInfo_.dipCoupling = opt_.dipCoupling;
        for (int f = 0; f < 5; ++f) {
            lastInfo_.wMcpCouple[size_t(f)]            = opt_.wMcpCouple;
            lastInfo_.mcpPipCoupling[size_t(f)]         = mcpPipCouplingSlope(f);
            lastInfo_.mcpPipCouplingB[size_t(f)]        = mcpPipCouplingIntercept(f);
            lastInfo_.mcpPipCouplingSamples[size_t(f)]  = mcpPipCouplingSamples(f);
            lastInfo_.mcpCouplingReady[size_t(f)]       = mcpCouplingReady(f);
        }
        lastInfo_.isRight     = isRight_;
        lastInfo_.paramsReady = paramsReady_;
        lastInfo_.applied     = true;

        for (int f = 0; f < 5; ++f) {
            if (!R.fingerValid[size_t(f)]) continue;
            for (int j = 0; j < 3; ++j) {
                const int m = 5 + 3 * f + j;
                // 【真观测不覆盖】观测本身比 IK 拟合更准（IK 受群体均值贴点
                // 参数的限制，位置残差中位 4.6mm）。IK 只补被遮挡的点。
                if (!observed[size_t(m)]) {
                    Vec3 p = R.markers[size_t(m)];
                    if (!isRight_) p[1] = -p[1];          // 镜像回来
                    markers[size_t(m)] = add(Vec3{
                        wristR[0]*p[0] + wristR[1]*p[1] + wristR[2]*p[2],
                        wristR[3]*p[0] + wristR[4]*p[1] + wristR[5]*p[2],
                        wristR[6]*p[0] + wristR[7]*p[1] + wristR[8]*p[2]}, wristT);
                }
                // segQuat 一律用 IK 的结果：marker->marker 当骨轴有 10.2° 中位
                // 系统偏差，即使两端都是真观测也消不掉。
                // 左手：旋转的镜像是 M R M（M = diag(1,-1,1)），不是简单转置。
                M3 Rl = R.segR[size_t(1 + 3 * f + j)];
                if (!isRight_) {
                    // M R M：把第 2 行和第 2 列各取一次反号，交点(1,1)两次抵消
                    Rl[1] = -Rl[1]; Rl[3] = -Rl[3]; Rl[5] = -Rl[5]; Rl[7] = -Rl[7];
                }
                const M3 Rw = mul3(wristR, Rl);
                segQuat[size_t(1 + 3 * f + j)] = matToQuat(Rw);
            }
        }
        return true;
    }

    IkFrameInfo lastFrameInfo() const override { return lastInfo_; }

    // 冷启动/重捕时清掉时序先验，下一帧走多起点搜索
    void resetPrior() override { prevAngles_ = {}; lastInfo_ = IkFrameInfo{}; }

    void setBackTemplate(const std::array<Vec3, 5>& back) {
        backTemplate_ = back;
        setHandLenFromBackTemplate(back);
    }

    // -------------------------------------------------------------------------
    // 主入口（也可单独调用）：给定腕部系下的观测 marker（seen 标记哪些可用），解出 16 个关节角，
    // 再 FK 出全部 20 个 marker 和 16 个分段朝向。
    //
    // obsWrist  20 个观测点（腕部系）。未观测到的位置任意，由 seen 决定是否使用。
    // seen      哪些 marker 被网络认领了
    // prior     上一帧的关节角（没有就传全 0 = 中立位）
    // -------------------------------------------------------------------------
    struct Result {
        std::array<std::array<double, 4>, 5> angles{};
        std::array<Vec3, 20>                 markers{};
        std::array<M3, 16>                   segR{};
        std::array<Vec3, 16>                 segO{};
        std::array<double, 5>                rmseMm{};   // 逐指拟合残差
        // 【必须逐指报有效性】某根手指 3 个 marker 全被遮挡时，IK 没有任何
        // 观测约束，解只由先验/限位决定 —— 输出的是中立位，跟真实姿势可能差
        // 180°。实测 200 帧随机姿势 + 15% 掉点，最大朝向误差 175.9°，
        // 全部来自这种情况。下游【必须】看这个标志，不能无脑用 segQuat。
        //   nObs >= 2 : 姿态可信
        //   nObs == 1 : 位置对得上，但绕骨轴的旋转不可观（欠定）
        //   nObs == 0 : 纯外推，只能当"这根手指大概在哪"用
        std::array<int, 5>                   nObs{};
        std::array<bool, 5>                  fingerValid{};
        // ---- v5：求解过程本身的量 ----
        // 【为什么要往外传】残差 rmseMm 只说明"拟合得好不好"，不说明"为什么"。
        // 代价卡在高位 + 迭代跑满 = 没收敛（该看起点和限位）；
        // 代价很低但残差大 = 罚项占了大头（观测太少，解是先验编出来的）。
        // 两者的修法完全不同，而只看 rmse 分不开。
        std::array<double, 5>                cost{};
        std::array<int, 5>                   iters{};
        std::array<bool, 5>                  solved{};
        bool ok = false;
    };

    Result solve(const std::array<Vec3, 20>& obsWrist,
                 const std::array<bool, 20>& seen,
                 const std::array<std::array<double, 4>, 5>& prior,
                 const std::array<Vec3, 5>& backTemplate) {
        Result R{};
        for (int i = 0; i < 5; ++i) R.markers[size_t(i)] = backTemplate[size_t(i)];
        R.segR[0] = M3{1,0,0, 0,1,0, 0,0,1};
        R.segO[0] = Vec3{0, 0, 0};

        for (int f = 0; f < 5; ++f) {
            int nobs = 0;
            for (int j = 0; j < 3; ++j) if (seen[size_t(5 + 3 * f + j)]) ++nobs;
            R.nObs[size_t(f)] = nobs;
            R.fingerValid[size_t(f)] = (nobs >= 2);

            std::array<double, 4> q = prior[size_t(f)];
            double best = 1e18;
            std::array<double, 4> bestQ = q;

            // 冷启动多起点：从"摊平"出发拟合握拳的观测很容易卡在局部极小 ——
            // 表现就是"手一弯，补出来的点全飘"。手指只有 4 个自由度，
            // 4 个起点各跑一遍成本可忽略。有上一帧解时只用它做唯一起点。
            std::vector<std::array<double, 4>> starts;
            const bool cold = (prior[size_t(f)][0] == 0.0 && prior[size_t(f)][2] == 0.0);
            if (opt_.multiStart && cold) {
                starts = {{{0,0,0,0}},
                          {{0.30, 0.05, 0.45, 0.32}},
                          {{0.82, 0.03, 1.00, 0.68}},
                          {{1.45, 0.02, 1.78, 1.20}}};
            } else {
                starts = {q};
            }

            int itersUsed = 0;
            for (const auto& st : starts) {
                std::array<double, 4> qq = st;
                int it = 0;
                const double c = lmSolve(f, obsWrist, seen, prior[size_t(f)], qq, &it);
                itersUsed = std::max(itersUsed, it);
                if (c < best) { best = c; bestQ = qq; }
            }
            R.angles[size_t(f)] = bestQ;
            R.cost[size_t(f)]   = best;
            R.iters[size_t(f)]  = itersUsed;
            R.solved[size_t(f)] = true;
            // 【学习用这一帧刚解出来的 bestQ】只有 nobs>=3 时才会真的采样
            // （见 updateMcpPipCoupling 内部判断），数据不足的帧调用了也是
            // 直接跳过，不会污染在线估计。
            updateMcpPipCoupling(f, nobs, bestQ);
            // 观测数不足时 best 里几乎全是先验/限位罚项，开方出来没有物理含义，
            // 直接标成 NaN 免得下游拿它当拟合质量用
            R.rmseMm[size_t(f)] = (nobs > 0)
                ? std::sqrt(std::max(best, 0.0) / std::max(nobs * 3, 1))
                : std::numeric_limits<double>::quiet_NaN();

            const auto fk = markerFK(f, anchor_[size_t(f)], len_[size_t(f)], bestQ, mm_);
            for (int j = 0; j < 3; ++j) {
                R.markers[size_t(5 + 3 * f + j)] = fk.marker[size_t(j)];
                R.segR[size_t(1 + 3 * f + j)]    = fk.segR[size_t(j)];
                R.segO[size_t(1 + 3 * f + j)]    = fk.segO[size_t(j)];
            }
        }
        R.ok = true;
        return R;
    }

private:
    // 【在线学习 MCP->PIP 耦合的累加器：带截距的加权最小二乘 + 指数衰减】
    //
    // 【为什么不能用过原点回归】第一版用的是过原点最小二乘（k=Σmp/Σmm），
    // 用真实录制验证时符号是反的——皮尔逊相关系数明明是 +0.91，学出来的
    // 斜率却是 -1.7。根因：q[0]=0 这个坐标零点不对应"手指伸直"这个解剖学
    // 零点，不同手指的静止基准角本身就有正有负。过原点回归强制"mcp=0时
    // pip必须=0"，这个假设不成立，拟合非零基准的数据符号会被带偏。
    // 现在用标准协方差法算带截距的斜率，不管两个自由度各自的零点在哪，
    // 拟合出来的都是真实的"变化趋势"。
    //
    // 【为什么要衰减】不衰减的话是终身累加：同一次 app 运行录了好几段
    // 不同动作，早期探索性/标定阶段的数据永远洗不掉，样本数只涨不消，
    // 新数据的影响被摊得越来越薄。指数衰减让近期数据主导估计。
    struct CoupleFit {
        double sumW = 0.0, sumMcp = 0.0, sumPip = 0.0, sumMcpMcp = 0.0, sumMcpPip = 0.0;
        int    n = 0;   // 原始样本计数（不衰减），只用于判断"够不够开始学"

        double rawSlope() const {
            const double denom = sumW * sumMcpMcp - sumMcp * sumMcp;
            return std::abs(denom) > 1e-9 ? (sumW * sumMcpPip - sumMcp * sumPip) / denom : 0.0;
        }
        // 【钳制】即使符号修对了，斜率对训练时观测到的 mcp 范围极度敏感——
        // 范围越窄，斜率的方差越大，越容易学出一个陡到离谱的值。方差门控
        // (mcpCouplingReady)已经挡掉了"范围太窄不可信"的情况，这里再加一层
        // 钳制兜底：即使方差门控通过了，也不允许斜率超出物理合理区间——
        // 负值意味着往错误方向拉（比不拉还糟，直接不会通过 kMin=0 这道），
        // 过大意味着外推一旦越出训练范围就可能失控。
        double slope(const IkOptions& opt) const {
            return std::clamp(rawSlope(), opt.mcpCouplingKMin, opt.mcpCouplingKMax);
        }
        double intercept(const IkOptions& opt) const {
            if (sumW < 1e-6) return 0.0;
            return (sumPip - slope(opt) * sumMcp) / sumW;
        }
    };
    std::array<CoupleFit, 5> mcpCoupleFit_{};

    // 【为什么用 solved 之后的 bestQ，而不是残差里那份】这里要的是"当三个
    // marker 都能看到时，MCP 和 PIP 真实解出来是什么关系"——这正是 bestQ，
    // 用它才对。放在 solve() 里、拿到 bestQ 之后立刻调用。
    //
    // 【拇指不学】它是 CMC 关节主导，运动学跟其余四指不是一回事，已经有
    // 专门的轴向/常数旋前标定路径，不该再套一层通用耦合——学了也不用
    // （mcpPipCouplingSlope 对 finger==0 恒返回 0），干脆不喂数据进去。
    void updateMcpPipCoupling(int f, int nobs, const std::array<double, 4>& q) {
        if (f == 0) return;
        if (nobs < 3) return;                                  // 数据不全的解不可信，不学
        const double mcp = q[0], pip = q[2];
        auto& s = mcpCoupleFit_[size_t(f)];
        const double d = opt_.mcpCouplingDecay;
        s.sumW *= d; s.sumMcp *= d; s.sumPip *= d; s.sumMcpMcp *= d; s.sumMcpPip *= d;
        s.sumW += 1.0;
        s.sumMcp += mcp;
        s.sumPip += pip;
        s.sumMcpMcp += mcp * mcp;
        s.sumMcpPip += mcp * pip;
        ++s.n;
    }

    // 残差向量（供 Gauss-Newton 用）。最多 20 维：
    //   9 = 3 个 marker x 3 坐标（只算被观测到的）
    //   4 = 时序先验  4 = 限位罚  1 = 外展锁(遮挡时)  1 = DIP耦合  1 = MCP->PIP耦合
    // 【为什么不能用标量 cost 的差分梯度】那样得到的只是 1x4 的梯度，
    // 拿 g·gᵀ 当 Hessian 是秩 1 近似，收敛极慢且方向常常是错的
    // （实测关节角误差 7.3°）。用残差雅可比的 JᵀJ 才是真正的 Gauss-Newton。
    int residuals(int f, const std::array<Vec3, 20>& obs, const std::array<bool, 20>& seen,
                  const std::array<double, 4>& prior, const std::array<double, 4>& q,
                  std::array<double, 20>& r) const {
        const auto fk = markerFK(f, anchor_[size_t(f)], len_[size_t(f)], q, mm_);
        int n = 0;
        for (int j = 0; j < 3; ++j) {
            const int m = 5 + 3 * f + j;
            if (!seen[size_t(m)]) continue;
            const Vec3 d = sub(fk.marker[size_t(j)], obs[size_t(m)]);
            r[size_t(n++)] = d[0]; r[size_t(n++)] = d[1]; r[size_t(n++)] = d[2];
        }
        for (int k = 0; k < 4; ++k)
            r[size_t(n++)] = opt_.wPrior * (q[size_t(k)] - prior[size_t(k)]);
        for (int k = 0; k < 4; ++k) {
            const double over = std::max(q[size_t(k)] - ikLimitHi()[f][k], 0.0)
                              + std::min(q[size_t(k)] - ikLimitLo()[f][k], 0.0);
            // 【外展用更硬的罚】见 IkOptions::wAbductionLimit 的说明——
            // 实测外展常年顶死甚至越出限位，通用的 wLimit 压不住它。
            // k==1 是外展（非拇指是 MCP展，拇指是 CMC展）。
            const double w = (k == 1) ? opt_.wAbductionLimit : opt_.wLimit;
            r[size_t(n++)] = w * over;
        }
        // 【观测不足时锁住外展】外展是 4 个自由度里最不受观测约束的一个：
        // 遮挡时 PIP/DIP 被先验拉住不能动，优化器就只剩外展这一个自由方向
        // 可以用来"够"那颗唯一可见的 marker，于是把本该由屈曲承担的运动
        // 全接过去——这就是"指尖侧弯到不合理程度"的直接机制。
        // 没有观测数据支撑的自由度不该自由活动，钉在上一帧附近即可。
        {
            int nobsF = 0;
            for (int j = 0; j < 3; ++j) if (seen[size_t(5 + 3 * f + j)]) ++nobsF;
            if (nobsF < opt_.abductionLockMinObs)
                r[size_t(n++)] = opt_.wAbductionLock * (q[1] - prior[1]);
        }
        r[size_t(n++)] = opt_.wCouple * (q[3] - opt_.dipCoupling * q[2]);
        // 【MCP->PIP 兜底，见 IkOptions::wMcpCouple 的说明】
        // 【只在这条兜底"准备好了"时才加这一项】没准备好（样本不够 /
        // mcp 观测范围太窄 / 拇指）时直接不加，等价于回退到修复前的行为——
        // 不会比不修复更差。这比"加一项但斜率填 0"要好：填 0 意味着
        // "PIP 应该等于常数 b"，那是一个真实存在的约束，会去拽 PIP；
        // 而不加这一项才是真正的"不表态"。
        if (mcpCouplingReady(f)) {
            // 权重恒定挂在这里，不做"数据够不够"的显式开关——数据充分时
            // marker 残差(上面 9 维，单位 mm，量级通常是几到几十)远大于
            // 这一项(单位是弧度乘一个 <1 的权重，量级 0.1 左右)，兜底项在
            // JᵀJ 里自然被压成可忽略的修正；数据不足时 marker 那部分残差
            // 项数量为 0，兜底项就是唯一还在给 PIP 提供方向信息的东西。
            // 不用显式 if 判断"够不够"，让残差量级自己说话——这样也不会在
            // "数据刚好卡在够与不够之间"时产生突变。
            const double target = mcpPipCouplingSlope(f) * q[0] + mcpPipCouplingIntercept(f);
            r[size_t(n++)] = opt_.wMcpCouple * (q[2] - target);
        }
        return n;
    }

    // 带接受/拒绝的 Levenberg-Marquardt（Gauss-Newton + 阻尼）
    double lmSolve(int f, const std::array<Vec3, 20>& obs, const std::array<bool, 20>& seen,
                   const std::array<double, 4>& prior, std::array<double, 4>& q,
                   int* itersOut = nullptr) const {
        double lam = 1e-3;
        if (itersOut) *itersOut = 0;
        std::array<double, 20> r0{}, rp{};
        int n = residuals(f, obs, seen, prior, q, r0);
        double cur = 0; for (int i = 0; i < n; ++i) cur += r0[size_t(i)] * r0[size_t(i)];

        for (int it = 0; it < opt_.iters; ++it) {
            if (itersOut) *itersOut = it + 1;
            // 残差雅可比 J (n x 4)，中心差分精度更好但两倍开销；这里前向差分够用
            double J[20][4];
            for (int k = 0; k < 4; ++k) {
                auto qp = q; qp[size_t(k)] += opt_.eps;
                residuals(f, obs, seen, prior, qp, rp);
                for (int i = 0; i < n; ++i)
                    J[i][k] = (rp[size_t(i)] - r0[size_t(i)]) / opt_.eps;
            }
            // JᵀJ dx = -Jᵀr
            double H[4][4] = {}, g[4] = {};
            for (int i = 0; i < n; ++i) {
                for (int a2 = 0; a2 < 4; ++a2) {
                    g[a2] += J[i][a2] * r0[size_t(i)];
                    for (int b2 = 0; b2 < 4; ++b2) H[a2][b2] += J[i][a2] * J[i][b2];
                }
            }
            // Marquardt 阻尼：按对角线缩放，比加单位阵对尺度更鲁棒
            for (int k = 0; k < 4; ++k) H[k][k] += lam * (H[k][k] + 1e-9);

            double A[4][5];
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) A[i][j] = H[i][j];
                A[i][4] = -g[i];
            }
            bool sing = false;
            for (int c = 0; c < 4 && !sing; ++c) {
                int p = c;
                for (int r2 = c + 1; r2 < 4; ++r2)
                    if (std::fabs(A[r2][c]) > std::fabs(A[p][c])) p = r2;
                if (std::fabs(A[p][c]) < 1e-14) { sing = true; break; }
                for (int j = 0; j < 5; ++j) std::swap(A[c][j], A[p][j]);
                for (int r2 = c + 1; r2 < 4; ++r2) {
                    const double m = A[r2][c] / A[c][c];
                    for (int j = c; j < 5; ++j) A[r2][j] -= m * A[c][j];
                }
            }
            if (sing) { lam = std::min(lam * 10.0, 1e6); continue; }
            double dx[4];
            for (int i = 3; i >= 0; --i) {
                double s2 = A[i][4];
                for (int j = i + 1; j < 4; ++j) s2 -= A[i][j] * dx[j];
                dx[i] = s2 / A[i][i];
            }

            auto cand = q;
            for (int k = 0; k < 4; ++k)
                cand[size_t(k)] = std::clamp(q[size_t(k)] + std::clamp(dx[k], -0.5, 0.5),
                                             ikLimitLo()[f][k] - 0.05, ikLimitHi()[f][k] + 0.05);
            const int n2 = residuals(f, obs, seen, prior, cand, rp);
            double nc = 0; for (int i = 0; i < n2; ++i) nc += rp[size_t(i)] * rp[size_t(i)];
            if (nc < cur) {
                q = cand; cur = nc; r0 = rp; n = n2;
                lam = std::max(lam * 0.3, 1e-8);
                if (std::fabs(dx[0]) + std::fabs(dx[1]) + std::fabs(dx[2]) + std::fabs(dx[3]) < 1e-7)
                    break;                       // 收敛，提前退出省时间
            } else {
                lam = std::min(lam * 5.0, 1e6);
            }
        }
        return cur;
    }

    IkOptions opt_;
    Hm20MarkerModel mm_;
    IkFrameInfo lastInfo_{};
    std::array<std::array<double, 4>, 5> prevAngles_{};   // 时序先验
    std::array<Vec3, 5> backTemplate_{};
    bool paramsReady_ = false;
    bool isRight_ = true;
    std::array<Vec3, 5> anchor_{};
    std::array<std::array<double, 3>, 5> len_{};
};

}  // namespace hm20
}  // namespace mocap
