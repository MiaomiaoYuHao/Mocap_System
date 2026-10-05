#pragma once
// ---------------------------------------------------------------------------
// 手指结构自标定(方案3：自然摆动，不额外贴marker) —— 纯数学，零 Qt 依赖。
//
// 回答"是否需要所有相机同时看清"：不需要。这里的输入是"每一帧、每颗
// marker在手腕局部系下的3D观测(缺失的标记为不可见)"——每一帧这些观测
// 本身怎么三角化出来的(哪两台或几台相机看到的、跟别的帧是不是同一组
// 相机)完全是上游的事，这个模块不关心，也不要求同一组相机、不要求全部
// 相机都看到。它只要求：足够多帧里，每根手指的关节角覆盖得足够多样，
// 且每帧至少能找到该手指的2颗marker(见下面 kMinMarkersPerFrame)。
//
// 前提：这个自标定必须在"手腕局部系"下做，不是世界系——因为要解的是
// "手指相对手腕的结构"，如果手腕本身也在动，世界系下的marker轨迹会把
// "手腕怎么动"和"手指怎么动"混在一起，解不出手指结构。所以调用方需要
// 先有一个可信的逐帧手腕位姿(来自 HandStateIEKF 或至少 HandColdStart 的
// solveHandBackPose)，把每一帧的世界系marker观测转换到那一帧的手腕局部
// 系，再喂给这里——手腕位姿的求解完全不依赖手指结构参数(solveHandBackPose
// 只用手背5点)，所以这个依赖顺序是通的，不是死循环。
//
// 数学做法：交替优化(跟 TwoCircleFit.hpp 的双圆交替分配、MarkerAssociator
// 的关联+IEKF是同一个"交替优化"哲学，不是巧合，这类问题分块解通常比一次性
// 联合求解好写、好调、足够准)：
//   1. 结构参数(anchor+lengths)固定在当前估计，对每一帧独立做一次逆运动学
//      (给定这帧观测到的marker，解出这帧的关节角)——每帧互相独立，缺哪帧
//      缺哪颗都不影响别的帧。
//   2. 每帧关节角固定，汇总所有帧的残差，重新解一次结构参数(anchor+lengths
//      对所有帧共享，跨帧联合最小二乘)。
//   3. 重复 1<->2 几轮，通常很快收敛(结构参数一开始就用现有占位值当起点，
//      不是从零猜)。
// ---------------------------------------------------------------------------
#include "hand/HandModel.hpp"
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <random>

namespace mocap {

// 某一帧、某根手指的观测：3颗marker各自在手腕局部系下的位置，缺失的用
// hasMarker标记false(值随便填，不会被用到)。
struct FingerFrameObservation {
    std::array<std::array<double,3>,3> markerPos{};
    std::array<bool,3> hasMarker{false,false,false};
    int validCount() const { return int(hasMarker[0])+int(hasMarker[1])+int(hasMarker[2]); }
};

// 拇指用4个自由度，markerPos/hasMarker语义相同(掌骨/近/远节3颗marker)。
using ThumbFrameObservation = FingerFrameObservation;

struct FingerCalibResult {
    FingerParam param{};
    double rmsMm = -1.0;
    int framesUsed = 0;
    bool valid = false;
};

namespace selfcal_detail {

using Vec = std::vector<double>;

inline Vec solveLinearNxN(std::vector<Vec> A, Vec b) {
    const int n = int(b.size());
    for (int col=0; col<n; ++col) {
        int piv = col;
        for (int r=col+1;r<n;++r) if (std::abs(A[size_t(r)][size_t(col)]) > std::abs(A[size_t(piv)][size_t(col)])) piv=r;
        std::swap(A[size_t(col)], A[size_t(piv)]); std::swap(b[size_t(col)], b[size_t(piv)]);
        if (std::abs(A[size_t(col)][size_t(col)]) < 1e-14) continue;   // 退化，跳过这一列(结果里对应分量不会被正确更新)
        for (int r=0;r<n;++r) {
            if (r==col) continue;
            const double f = A[size_t(r)][size_t(col)] / A[size_t(col)][size_t(col)];
            for (int c=0;c<n;++c) A[size_t(r)][size_t(c)] -= f*A[size_t(col)][size_t(c)];
            b[size_t(r)] -= f*b[size_t(col)];
        }
    }
    Vec x(size_t(n), 0.0);
    for (int i=0;i<n;++i) x[size_t(i)] = std::abs(A[size_t(i)][size_t(i)])>1e-14 ? b[size_t(i)]/A[size_t(i)][size_t(i)] : 0.0;
    return x;
}

// 通用小规模 Levenberg-Marquardt，数值雅可比(中心差分)。residualFn: 给定
// unknowns，返回残差向量。用于这个文件里"逐帧IK"和"跨帧结构"两处，维度
// 都不大(<=6)，手写矩阵运算足够，不引入线性代数库。
template <typename ResidualFn>
inline Vec solveLM(Vec x, ResidualFn residualFn, int maxIters, double h = 1e-6) {
    const int n = int(x.size());
    auto cost = [&](const Vec& xx) {
        const auto r = residualFn(xx);
        double s=0; for (double v : r) s += v*v; return s;
    };
    double lambda = 1e-3;
    double curCost = cost(x);

    for (int iter=0; iter<maxIters; ++iter) {
        const auto r0 = residualFn(x);
        const int m = int(r0.size());
        if (m == 0) break;

        const size_t mSz = size_t(m), nSz = size_t(n);
        std::vector<Vec> J(mSz, Vec(nSz));
        for (int k=0;k<n;++k) {
            Vec xp=x, xm=x; xp[size_t(k)]+=h; xm[size_t(k)]-=h;
            const auto rp = residualFn(xp), rm = residualFn(xm);
            for (int i=0;i<m;++i) J[size_t(i)][size_t(k)] = (rp[size_t(i)]-rm[size_t(i)])/(2*h);
        }

        std::vector<Vec> JtJ(size_t(n), Vec(size_t(n), 0.0));
        Vec JtR(size_t(n), 0.0);
        for (int a=0;a<n;++a) {
            for (int b=0;b<n;++b) { double s=0; for (int i=0;i<m;++i) s += J[size_t(i)][size_t(a)]*J[size_t(i)][size_t(b)]; JtJ[size_t(a)][size_t(b)]=s; }
            double s=0; for (int i=0;i<m;++i) s += J[size_t(i)][size_t(a)]*r0[size_t(i)]; JtR[size_t(a)]=s;
        }
        for (int i=0;i<n;++i) JtJ[size_t(i)][size_t(i)] *= (1.0+lambda);
        const Vec delta = solveLinearNxN(JtJ, JtR);

        Vec xNew(nSz);
        for (int i=0;i<n;++i) xNew[size_t(i)] = x[size_t(i)] - delta[size_t(i)];
        const double newCost = cost(xNew);

        if (newCost < curCost) {
            x = xNew; curCost = newCost;
            lambda = std::max(lambda*0.5, 1e-12);
            double deltaNorm=0; for (double d : delta) deltaNorm += d*d;
            if (std::sqrt(deltaNorm) < 1e-9) break;
        } else {
            lambda *= 2.0;
            if (lambda > 1e12) break;
        }
    }
    return x;
}

// 逐帧IK的局部极小值问题：结构参数一开始离真值较远时，从单一初值(通常是
// 上一轮解出来的角度)做梯度下降容易稳定收敛到一个"跟错误结构自洽"的错误
// 关节角，进而让结构反解也跟着长期收敛到一个有偏但看似residual还不错的
// 错误解(交替优化常见的局部极小值陷阱，不是这里独有的问题)。用多个随机
// 起点各自收敛、取残差最小的一个，能有效降低撞进同一个错误吸引域的概率
// (不同随机起点大概率落进不同吸引域，其中大概率包含真正的全局最优)。
template <typename ResidualFn>
inline Vec solveLMMultiStart(const Vec& warmStart, ResidualFn residualFn, int maxIters,
                             int numRandomRestarts, double restartSpread, std::mt19937& rng) {
    auto cost = [&](const Vec& xx) { const auto r = residualFn(xx); double s=0; for (double v:r) s+=v*v; return s; };

    Vec best = solveLM(warmStart, residualFn, maxIters);
    double bestCost = cost(best);

    std::normal_distribution<double> pert(0.0, restartSpread);
    for (int t=0; t<numRandomRestarts; ++t) {
        Vec x0 = warmStart;
        for (double& v : x0) v += pert(rng);
        const auto sol = solveLM(x0, residualFn, maxIters);
        const double c = cost(sol);
        if (c < bestCost) { best = sol; bestCost = c; }
    }
    return best;
}

} // namespace selfcal_detail

// ---- 四指(非拇指)结构自标定：3自由度(mcpFlex,mcpAbduct,pipFlex) ----
//
// 交替优化(逐帧IK <-> 跨帧结构)在结构初值离真值较远时，容易稳定收敛到一个
// "跟错误结构自洽"的局部极小值——不同随机起点的逐帧IK大概率落进不同的
// 吸引域，所以这里把"整套交替优化流程"本身也做成多次不同随机种子独立跑，
// 取残差最小的一次，而不是只在单帧IK这一层做多起点(单帧IK多起点在结构
// 严重偏离真值时不足以摆脱系统性偏差，实测验证过：加了单帧多起点后偏差
// 从4.9mm降到2.6mm，还不够干净，得连整个交替流程一起多跑几次)。
inline FingerCalibResult calibrateFingerStructure(
        const std::vector<FingerFrameObservation>& frames,
        const FingerParam& initialGuess,
        int minMarkersPerFrame = 2,
        int outerRounds = 12,
        int globalRestarts = 6) {
    using namespace selfcal_detail;

    std::vector<int> usableFrames;
    for (int f=0; f<int(frames.size()); ++f)
        if (frames[size_t(f)].validCount() >= minMarkersPerFrame) usableFrames.push_back(f);
    if (int(usableFrames.size()) < 3) return FingerCalibResult{};

    auto fkResidualForFrame = [&](const FingerParam& p, const std::array<double,3>& q,
                                  const FingerFrameObservation& obs) {
        std::array<double,3> out3[3];
        fingerFK(p, q[0], q[1], q[2], out3);
        Vec r;
        for (int k=0;k<3;++k) if (obs.hasMarker[size_t(k)])
            for (int d=0;d<3;++d) r.push_back(out3[k][size_t(d)] - obs.markerPos[size_t(k)][size_t(d)]);
        return r;
    };

    auto runOnce = [&](uint32_t seed, bool perturbStruct) {
        std::mt19937 rng(seed);
        FingerParam param = initialGuess;
        if (perturbStruct) {
            std::normal_distribution<double> structPert(0.0, 8.0);   // mm，结构初值的扰动幅度
            for (double& v : param.anchor) v += structPert(rng);
            for (double& v : param.lengths) v += structPert(rng);
            std::normal_distribution<double> dipPert(0.0, 0.15);   // dipCoupling是无量纲比例，扰动幅度小得多
            param.dipCoupling = std::clamp(param.dipCoupling + dipPert(rng), 0.2, 1.3);
        }
        std::vector<std::array<double,3>> qPerFrame(usableFrames.size(), {0.0,0.0,0.0});

        for (int round=0; round<outerRounds; ++round) {
            for (size_t fi=0; fi<usableFrames.size(); ++fi) {
                const auto& obs = frames[size_t(usableFrames[fi])];
                Vec x0 = { qPerFrame[fi][0], qPerFrame[fi][1], qPerFrame[fi][2] };
                auto resFn = [&](const Vec& xv) { return fkResidualForFrame(param, {xv[0],xv[1],xv[2]}, obs); };
                const auto xSol = solveLMMultiStart(x0, resFn, 30, 4, 0.6, rng);
                qPerFrame[fi] = { xSol[0], xSol[1], xSol[2] };
            }

            // 结构参数从6维(anchor3+lengths3)扩到7维，加入dipCoupling——
            // 远节那颗marker的位置本来就同时由pipFlex和dipFlex(=dipCoupling*
            // pipFlex)共同决定，这条信息在观测里本来就有，之前只是没让拟合
            // 去用它，一直用固定0.7、标多准的数据都改不动这部分系统性误差。
            // 现在跟anchor/lengths一样，用全部帧的残差联合解出来。
            Vec structX = { param.anchor[0], param.anchor[1], param.anchor[2],
                            param.lengths[0], param.lengths[1], param.lengths[2],
                            param.dipCoupling };
            auto structResFn = [&](const Vec& xv) {
                FingerParam p; p.anchor = {xv[0],xv[1],xv[2]}; p.lengths = {xv[3],xv[4],xv[5]};
                p.dipCoupling = xv[6];
                Vec r;
                for (size_t fi=0; fi<usableFrames.size(); ++fi) {
                    const auto& obs = frames[size_t(usableFrames[fi])];
                    const auto rf = fkResidualForFrame(p, qPerFrame[fi], obs);
                    r.insert(r.end(), rf.begin(), rf.end());
                }
                return r;
            };
            const auto structSol = solveLM(structX, structResFn, 30);
            param.anchor = {structSol[0], structSol[1], structSol[2]};
            param.lengths = {structSol[3], structSol[4], structSol[5]};
            // dipCoupling是相对弱观测的方向(远节marker的位置对它的敏感度比
            // 对anchor/lengths低，尤其pipFlex本身幅度小的帧里几乎没有信息量)，
            // 数据不够干净时数值解可能跑到不physical的区间(比如负数、大于2)，
            // clamp到一个宽松但物理合理的范围兜底，不让弱观测方向上的噪声
            // 污染掉本来就该稳的anchor/lengths收敛。
            param.dipCoupling = std::clamp(structSol[6], 0.2, 1.3);
        }

        double sq=0; int cnt=0;
        for (size_t fi=0; fi<usableFrames.size(); ++fi) {
            const auto& obs = frames[size_t(usableFrames[fi])];
            const auto rf = fkResidualForFrame(param, qPerFrame[fi], obs);
            for (double v : rf) { sq += v*v; ++cnt; }
        }
        FingerCalibResult r;
        r.param = param;
        r.rmsMm = cnt>0 ? std::sqrt(sq/cnt) : -1.0;
        r.framesUsed = int(usableFrames.size());
        return r;
    };

    FingerCalibResult best = runOnce(0xC0FFEE, false);
    for (int t=1; t<globalRestarts; ++t) {
        const auto r = runOnce(0xC0FFEE + uint32_t(t)*7919u, true);
        if (r.rmsMm >= 0.0 && (best.rmsMm < 0.0 || r.rmsMm < best.rmsMm)) best = r;
    }
    best.valid = (best.rmsMm >= 0.0) && (best.rmsMm < 5.0);   // 5mm 阈值：跟 HandPose.hpp 的confident判据同量级
    return best;
}

// ---- 拇指结构自标定：4自由度(cmcFlex,cmcAbduct,mcpFlex,ipFlex) ----
// ---------------------------------------------------------------------------
// 静态姿势标定(行业标准做法：商业动捕系统的手部骨架缩放都是从一个静态
// 参考姿势——手掌平放、手指自然伸直——一次性完成的，动态精修只是可选的
// 第二步。这个函数就是那"一次性缩放"的数学)。
//
// 前提假设：手指处于伸直平放姿势，所有关节角≈0。此时FK退化成三颗marker
// 沿手指方向共线(marker贴在每节指骨中点，见 HandModel.hpp fingerFK)：
//   p0 = anchor + d·(L1/2)
//   p1 = anchor + d·(L1 + L2/2)
//   p2 = anchor + d·(L1 + L2 + L3/2)
// 由此：方向 d 直接由 p2-p0 测出；相邻距离给出两个方程
//   |p1-p0| = (L1+L2)/2,  |p2-p1| = (L2+L3)/2
// 两个方程三个未知长度，第三个自由度用该手指默认模板的长度比例做先验闭合
// (人手指骨比例的个体差异远小于绝对长度差异——这正是所有商业系统"模板+
// 缩放"路线成立的解剖学基础)。之后 anchor = p0 - d·(L1/2)，全部闭式解，
// 微秒级完成，没有迭代、没有局部极小、没有对运动多样性的要求。
//
// 拇指同样适用：thumbFK 在全零角度下同样共线(固定的rotZ(45°)只是旋转
// 方向，方向本来就是从数据测的)。
//
// rmsMm 字段在这里的含义是"共线性偏差"(p1 到 p0-p2 连线的垂直距离，mm)
// ——它衡量"手放得平不平"：偏差大说明标定时手指是弯的，前提不成立，
// 结果不可信。valid 会按这个偏差和长度合理性做门控。
// ---------------------------------------------------------------------------
inline FingerCalibResult calibrateFingerFromStaticPose(
        const std::array<double,3>& p0,
        const std::array<double,3>& p1,
        const std::array<double,3>& p2,
        const FingerParam& ratioPrior,
        double maxColinearityDevMm = 6.0) {
    FingerCalibResult r;

    const std::array<double,3> v02 = { p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2] };
    const double n02 = std::sqrt(v02[0]*v02[0]+v02[1]*v02[1]+v02[2]*v02[2]);
    if (n02 < 20.0) return r;   // 三点跨度不足2cm，不像一根手指
    const std::array<double,3> d = { v02[0]/n02, v02[1]/n02, v02[2]/n02 };

    // 共线性偏差：p1 到 p0-p2 连线的垂直距离
    const std::array<double,3> v01 = { p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2] };
    const double along = v01[0]*d[0]+v01[1]*d[1]+v01[2]*d[2];
    const std::array<double,3> perp = { v01[0]-along*d[0], v01[1]-along*d[1], v01[2]-along*d[2] };
    const double colinearityDev = std::sqrt(perp[0]*perp[0]+perp[1]*perp[1]+perp[2]*perp[2]);
    r.rmsMm = colinearityDev;

    const double a = std::sqrt(v01[0]*v01[0]+v01[1]*v01[1]+v01[2]*v01[2]);   // |p1-p0|
    const std::array<double,3> v12 = { p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2] };
    const double b = std::sqrt(v12[0]*v12[0]+v12[1]*v12[1]+v12[2]*v12[2]);   // |p2-p1|

    // L1+L2=2a, L2+L3=2b；令 t=L2，用比例先验 r1=L1/L2, r3=L3/L2 闭合：
    // 最小化 (L1-r1·t)² + (L3-r3·t)²，其中 L1=2a-t, L3=2b-t。
    // 令 u=1+r1, v=1+r3，驻点：t = 2(a·u + b·v)/(u²+v²)。
    const double prior2 = std::max(1e-6, ratioPrior.lengths[1]);
    const double r1 = ratioPrior.lengths[0] / prior2;
    const double r3 = ratioPrior.lengths[2] / prior2;
    const double u = 1.0 + r1, v = 1.0 + r3;
    const double t = 2.0*(a*u + b*v) / (u*u + v*v);
    const double L1 = 2.0*a - t, L2 = t, L3 = 2.0*b - t;

    r.param.lengths = { L1, L2, L3 };
    r.param.anchor = { p0[0] - d[0]*L1*0.5, p0[1] - d[1]*L1*0.5, p0[2] - d[2]*L1*0.5 };
    // dipCoupling 这里不动、保持默认值——静态伸直姿势下 pipFlex≈0，
    // dipFlex=dipCoupling*pipFlex 恒等于0，不管dipCoupling取什么值marker
    // 位置都一样，数学上不可观测，不是遗漏。真正标定它要靠
    // calibrateFingerStructure 那条动态精修路径(pipFlex不为0的帧才有
    // 信息量)，这里产出的param.dipCoupling原样传下去当那边的初值。
    r.framesUsed = 1;

    // 合理性门控：三节长度都要落在人手指骨的物理范围里，且手确实放平了。
    const bool lengthsOk = L1 > 8.0 && L1 < 90.0 && L2 > 5.0 && L2 < 70.0 && L3 > 4.0 && L3 < 60.0;
    r.valid = lengthsOk && colinearityDev <= maxColinearityDevMm;
    return r;
}

inline FingerCalibResult calibrateThumbStructure(
        const std::vector<ThumbFrameObservation>& frames,
        const FingerParam& initialGuess,
        int minMarkersPerFrame = 2,
        int outerRounds = 12,
        int globalRestarts = 6) {
    using namespace selfcal_detail;

    std::vector<int> usableFrames;
    for (int f=0; f<int(frames.size()); ++f)
        if (frames[size_t(f)].validCount() >= minMarkersPerFrame) usableFrames.push_back(f);
    if (int(usableFrames.size()) < 4) return FingerCalibResult{};   // 4自由度，帧数门槛比四指略高

    auto fkResidualForFrame = [&](const FingerParam& p, const std::array<double,4>& q,
                                  const ThumbFrameObservation& obs) {
        std::array<double,3> out3[3];
        thumbFK(p, q[0], q[1], q[2], q[3], out3);
        Vec r;
        for (int k=0;k<3;++k) if (obs.hasMarker[size_t(k)])
            for (int d=0;d<3;++d) r.push_back(out3[k][size_t(d)] - obs.markerPos[size_t(k)][size_t(d)]);
        return r;
    };

    auto runOnce = [&](uint32_t seed, bool perturbStruct) {
        std::mt19937 rng(seed);
        FingerParam param = initialGuess;
        if (perturbStruct) {
            std::normal_distribution<double> structPert(0.0, 8.0);
            for (double& v : param.anchor) v += structPert(rng);
            for (double& v : param.lengths) v += structPert(rng);
        }
        std::vector<std::array<double,4>> qPerFrame(usableFrames.size(), {0.0,0.0,0.0,0.0});

        for (int round=0; round<outerRounds; ++round) {
            for (size_t fi=0; fi<usableFrames.size(); ++fi) {
                const auto& obs = frames[size_t(usableFrames[fi])];
                Vec x0 = { qPerFrame[fi][0], qPerFrame[fi][1], qPerFrame[fi][2], qPerFrame[fi][3] };
                auto resFn = [&](const Vec& xv) { return fkResidualForFrame(param, {xv[0],xv[1],xv[2],xv[3]}, obs); };
                const auto xSol = solveLMMultiStart(x0, resFn, 30, 4, 0.6, rng);
                qPerFrame[fi] = { xSol[0], xSol[1], xSol[2], xSol[3] };
            }

            Vec structX = { param.anchor[0], param.anchor[1], param.anchor[2],
                            param.lengths[0], param.lengths[1], param.lengths[2] };
            auto structResFn = [&](const Vec& xv) {
                FingerParam p; p.anchor = {xv[0],xv[1],xv[2]}; p.lengths = {xv[3],xv[4],xv[5]};
                Vec r;
                for (size_t fi=0; fi<usableFrames.size(); ++fi) {
                    const auto& obs = frames[size_t(usableFrames[fi])];
                    const auto rf = fkResidualForFrame(p, qPerFrame[fi], obs);
                    r.insert(r.end(), rf.begin(), rf.end());
                }
                return r;
            };
            const auto structSol = solveLM(structX, structResFn, 30);
            param.anchor = {structSol[0], structSol[1], structSol[2]};
            param.lengths = {structSol[3], structSol[4], structSol[5]};
        }

        double sq=0; int cnt=0;
        for (size_t fi=0; fi<usableFrames.size(); ++fi) {
            const auto& obs = frames[size_t(usableFrames[fi])];
            const auto rf = fkResidualForFrame(param, qPerFrame[fi], obs);
            for (double v : rf) { sq += v*v; ++cnt; }
        }
        FingerCalibResult r;
        r.param = param;
        r.rmsMm = cnt>0 ? std::sqrt(sq/cnt) : -1.0;
        r.framesUsed = int(usableFrames.size());
        return r;
    };

    FingerCalibResult best = runOnce(0xC0FFEE, false);
    for (int t=1; t<globalRestarts; ++t) {
        const auto r = runOnce(0xC0FFEE + uint32_t(t)*7919u, true);
        if (r.rmsMm >= 0.0 && (best.rmsMm < 0.0 || r.rmsMm < best.rmsMm)) best = r;
    }
    best.valid = (best.rmsMm >= 0.0) && (best.rmsMm < 5.0);
    return best;
}

} // namespace mocap
