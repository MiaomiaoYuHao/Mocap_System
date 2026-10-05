#pragma once
// ---------------------------------------------------------------------------
// 手指运动学自动标定——纯数学，零 Qt 依赖。
//
// 解决的问题：手背刚体标定完之后，给一根手指若干帧的3颗观测点(已转换到
// 腕部局部系、已按近/中/远节排好序——排序和"这3颗点属于哪根手指"是分组
// 层的职责，见 FingerChainSegmentation.hpp，本文件不关心)，拟合出：
//   - 锚点位置(该指MCP/CMC关节在腕部局部系的位置)
//   - 各节连杆长度
//   - 拇指额外多一个"根部方向角"(README讨论过的45度占位常量，现在也标定)
//   - 每一帧的关节角(屈曲/外展/...)
// 全都是待求未知量，不再是 hand/HandModel.hpp 里那份写死的人体测量学占位表。
//
// 数学方法：交替最小二乘(block coordinate descent)——
//   (a) 固定当前的锚点/连杆参数，逐帧只解那一帧的关节角(2~4维小问题)；
//   (b) 固定所有帧刚解出的关节角，把全部帧的观测一起喂进去解锚点/连杆
//       (6~7维，帧数越多越超定，越稳)；
//   反复(a)(b)直到收敛。每一步都是小规模非线性最小二乘，用同一个通用的
//   Levenberg-Marquardt(有限差分雅可比)求解器。
//
// 正向运动学公式跟 hand/HandModel.hpp 的 fingerFK/thumbFK 完全一致(同一套
// 齐次变换写法)，区别只是这里 anchor/lengths/baseRotZ 是"待求参数"而不是
// 常量表——两个文件不应该产生数值上不一致的模型，将来 HandModel.hpp 的
// 占位表就是被这里标定出的结果替换掉。
//
// 诚实失败：屈伸范围太窄(标定出的连杆长度置信区间会很宽)、残差过大(贴球
// 错位/分组错误/关节自由度超出模型假设)都在 message 里给出具体数值和建议，
// 不吞掉。
// ---------------------------------------------------------------------------
#include "estimate/PointIEKF.hpp"   // Vec3
#include "hand/HandModel.hpp"       // Mat4/rotX/rotY/rotZ/transMat（复用同一套齐次变换，不重新发明）
#include <vector>
#include <array>
#include <functional>
#include <cmath>
#include <string>
#include <algorithm>

namespace mocap {

    constexpr double kDipCouplingCalib = 0.7;   // 跟 HandModel.hpp 的 kDipCoupling 保持一致

    // ---------------------------------------------------------------------------
    // 通用小规模 Levenberg-Marquardt（有限差分雅可比）。用于本文件里两种规模的
    // 拟合问题：逐帧关节角(参数2~4维/残差9维)、全局锚点+连杆(参数6~7维/残差
    // 帧数x9维，通常严重超定)。矩阵求解用最朴素的列主元高斯消元——参数维度
    // 都在个位数，不需要更复杂的数值库。
    // ---------------------------------------------------------------------------
    namespace fit_detail {

        inline bool solveLinearSystem(std::vector<double> A, std::vector<double> b, int n, std::vector<double>& x) {
            x.assign(size_t(n), 0.0);
            for (int col = 0; col < n; ++col) {
                int piv = col; double best = std::abs(A[size_t(col) * n + col]);
                for (int r = col + 1; r < n; ++r) {
                    const double v = std::abs(A[size_t(r) * n + col]);
                    if (v > best) { best = v; piv = r; }
                }
                if (best < 1e-14) return false;
                if (piv != col) {
                    for (int c = 0; c < n; ++c) std::swap(A[size_t(col) * n + c], A[size_t(piv) * n + c]);
                    std::swap(b[size_t(col)], b[size_t(piv)]);
                }
                for (int r = col + 1; r < n; ++r) {
                    const double f = A[size_t(r) * n + col] / A[size_t(col) * n + col];
                    for (int c = col; c < n; ++c) A[size_t(r) * n + c] -= f * A[size_t(col) * n + c];
                    b[size_t(r)] -= f * b[size_t(col)];
                }
            }
            for (int r = n - 1; r >= 0; --r) {
                double s = b[size_t(r)];
                for (int c = r + 1; c < n; ++c) s -= A[size_t(r) * n + c] * x[size_t(c)];
                x[size_t(r)] = s / A[size_t(r) * n + r];
            }
            return true;
        }

        inline void levenbergMarquardt(std::vector<double>& params,
            const std::function<std::vector<double>(const std::vector<double>&)>& residualFn,
            int maxIters) {
            const int n = int(params.size());
            double lambda = 1e-3;
            std::vector<double> res = residualFn(params);
            double cost = 0.0;
            for (double r : res) cost += r * r;

            for (int iter = 0; iter < maxIters; ++iter) {
                const int m = int(res.size());
                std::vector<double> J(size_t(m) * size_t(n));
                for (int j = 0; j < n; ++j) {
                    std::vector<double> pp = params;
                    const double h = std::max(1e-6, std::abs(params[size_t(j)]) * 1e-5);
                    pp[size_t(j)] += h;
                    const auto resPlus = residualFn(pp);
                    for (int i = 0; i < m; ++i) J[size_t(i) * size_t(n) + size_t(j)] = (resPlus[size_t(i)] - res[size_t(i)]) / h;
                }
                std::vector<double> JtJ(size_t(n) * size_t(n), 0.0), Jtr(size_t(n), 0.0);
                for (int a = 0; a < n; ++a) {
                    for (int b = 0; b < n; ++b) {
                        double s = 0.0;
                        for (int i = 0; i < m; ++i) s += J[size_t(i) * size_t(n) + size_t(a)] * J[size_t(i) * size_t(n) + size_t(b)];
                        JtJ[size_t(a) * size_t(n) + size_t(b)] = s;
                    }
                    double s = 0.0;
                    for (int i = 0; i < m; ++i) s += J[size_t(i) * size_t(n) + size_t(a)] * res[size_t(i)];
                    Jtr[size_t(a)] = s;
                }

                bool improved = false;
                for (int tries = 0; tries < 12; ++tries) {
                    std::vector<double> A = JtJ;
                    for (int d = 0; d < n; ++d) A[size_t(d) * size_t(n) + size_t(d)] *= (1.0 + lambda);
                    const size_t un = size_t(n);
                    std::vector<double> negJtr(un);
                    for (int i = 0; i < n; ++i) negJtr[size_t(i)] = -Jtr[size_t(i)];
                    std::vector<double> dx;
                    if (!solveLinearSystem(A, negJtr, n, dx)) { lambda *= 10.0; continue; }

                    std::vector<double> newParams = params;
                    for (int i = 0; i < n; ++i) newParams[size_t(i)] += dx[size_t(i)];
                    const auto newRes = residualFn(newParams);
                    double newCost = 0.0;
                    for (double r : newRes) newCost += r * r;

                    if (newCost < cost) {
                        params = newParams; res = newRes; cost = newCost;
                        lambda = std::max(lambda * 0.5, 1e-12);
                        improved = true;
                        break;
                    }
                    lambda *= 10.0;
                }
                if (!improved || cost < 1e-8) break;
            }
        }

    } // namespace fit_detail

    // ---------------------------------------------------------------------------
    // 普通四指（食/中/无名/小）的链参数与正向运动学——公式跟 HandModel.hpp 的
    // fingerFK 完全一致，anchor/lengths 从常量变成参数。
    // ---------------------------------------------------------------------------
    struct FingerChainParams {
        Vec3 anchor{ 0, 0, 0 };
        std::array<double, 3> lengths{ 40, 25, 20 };   // Lp(近节), Lm(中节), Ld(远节)
        double dipCoupling = kDipCouplingCalib;   // 同 hand/HandModel.hpp::FingerParam.dipCoupling，见那边注释
    };

    inline std::array<Vec3, 3> fingerChainFK(const FingerChainParams& p,
        double mcpFlex, double mcpAbduct, double pipFlex) {
        const double Lp = p.lengths[0], Lm = p.lengths[1], Ld = p.lengths[2];
        const double dipFlex = p.dipCoupling * pipFlex;

        Mat4 T = transMat(p.anchor[0], p.anchor[1], p.anchor[2]);
        T = T * rotZ(mcpAbduct) * rotY(mcpFlex);

        const auto out0 = (T * transMat(Lp / 2, 0, 0)).translation();
        T = T * transMat(Lp, 0, 0);
        T = T * rotY(pipFlex);
        const auto out1 = (T * transMat(Lm / 2, 0, 0)).translation();
        T = T * transMat(Lm, 0, 0);
        T = T * rotY(dipFlex);
        const auto out2 = (T * transMat(Ld / 2, 0, 0)).translation();

        return { Vec3{out0[0], out0[1], out0[2]}, Vec3{out1[0], out1[1], out1[2]}, Vec3{out2[0], out2[1], out2[2]} };
    }

    // ---------------------------------------------------------------------------
    // 拇指的链参数与正向运动学——公式跟 HandModel.hpp 的 thumbFK 一致，唯一的
    // 结构差异：baseRotZ(模拟CMC根部方向、个体差异很大)从写死的 pi/4 常量
    // 变成待标定参数。
    // ---------------------------------------------------------------------------
    struct ThumbChainParams {
        Vec3 anchor{ 0, 0, 0 };
        double baseRotZ = M_PI / 4.0;
        std::array<double, 3> lengths{ 40, 30, 25 };   // Lmeta, Lp, Ld
    };

    inline std::array<Vec3, 3> thumbChainFK(const ThumbChainParams& p,
        double cmcFlex, double cmcAbduct, double mcpFlex, double ipFlex) {
        const double Lmeta = p.lengths[0], Lp = p.lengths[1], Ld = p.lengths[2];

        Mat4 T = transMat(p.anchor[0], p.anchor[1], p.anchor[2]) * rotZ(p.baseRotZ);
        T = T * rotZ(cmcAbduct) * rotY(cmcFlex);

        const auto out0 = (T * transMat(Lmeta / 2, 0, 0)).translation();
        T = T * transMat(Lmeta, 0, 0);
        T = T * rotY(mcpFlex);
        const auto out1 = (T * transMat(Lp / 2, 0, 0)).translation();
        T = T * transMat(Lp, 0, 0);
        T = T * rotY(ipFlex);
        const auto out2 = (T * transMat(Ld / 2, 0, 0)).translation();

        return { Vec3{out0[0], out0[1], out0[2]}, Vec3{out1[0], out1[1], out1[2]}, Vec3{out2[0], out2[1], out2[2]} };
    }

    struct FingerCalibConfig {
        int maxOuterIters = 60;
        int angleFitIters = 40;
        int globalFitIters = 80;
        double minFramesRequired = 20;
        double minRomDeg = 15.0;      // 观测到的屈曲角范围下限，太窄说明标定不可信
        double maxAcceptRmsMm = 3.0;
    };

    struct FingerChainCalibResult {
        bool valid = false;
        FingerChainParams params;
        std::vector<std::array<double, 3>> anglesPerFrame;   // mcpFlex, mcpAbduct, pipFlex
        double rmsMm = -1.0;
        double romFlexDeg = 0.0;
        int framesUsed = 0;
        std::string message;
    };

    // obs[frame] = {近节观测, 中节观测, 远节观测}，均已在腕部局部系下、已排好序。
    inline FingerChainCalibResult calibrateFingerChain(const std::vector<std::array<Vec3, 3>>& obs,
        const FingerCalibConfig& cfg = FingerCalibConfig{}) {
        FingerChainCalibResult out;
        const int F = int(obs.size());
        if (F < int(cfg.minFramesRequired)) {
            out.message = "帧数不足(实际 " + std::to_string(F) + " 帧，至少需要 " +
                std::to_string(int(cfg.minFramesRequired)) + " 帧)，覆盖不了足够的屈伸范围，标定结果不可信。";
            return out;
        }

        // 初始化：锚点用近节观测的均值粗估；连杆长度用相邻marker间距的粗略
        // 换算(marker在段中点，段长约等于中点间距的2倍附近，只是给LM一个
        // 合理起点，精确值靠后续全局拟合收敛)。
        Vec3 anchorInit{ 0, 0, 0 };
        for (auto& fr : obs) for (int k = 0; k < 3; ++k) anchorInit[size_t(k)] += fr[0][size_t(k)] / double(F);
        double d0 = 0.0, d1 = 0.0;
        for (auto& fr : obs) {
            double dx = fr[0][0] - anchorInit[0], dy = fr[0][1] - anchorInit[1], dz = fr[0][2] - anchorInit[2];
            d0 += std::sqrt(dx * dx + dy * dy + dz * dz) / double(F);
            double ex = fr[1][0] - fr[0][0], ey = fr[1][1] - fr[0][1], ez = fr[1][2] - fr[0][2];
            d1 += std::sqrt(ex * ex + ey * ey + ez * ez) / double(F);
        }
        FingerChainParams params;
        params.anchor = anchorInit;
        params.lengths = { std::max(10.0, d0 * 2.0), std::max(10.0, d1 * 1.2), std::max(10.0, d1) };

        std::vector<std::array<double, 3>> angles(size_t(F), std::array<double, 3>{0.1, 0.0, 0.1});

        double prevCost = 1e18;
        for (int outerIt = 0; outerIt < cfg.maxOuterIters; ++outerIt) {
            // (a) 固定params，逐帧解angles。
            for (int f = 0; f < F; ++f) {
                std::vector<double> a = { angles[size_t(f)][0], angles[size_t(f)][1], angles[size_t(f)][2] };
                auto residualFn = [&](const std::vector<double>& v) {
                    const auto pred = fingerChainFK(params, v[0], v[1], v[2]);
                    std::vector<double> r(9);
                    for (int k = 0; k < 3; ++k)
                        for (int c = 0; c < 3; ++c)
                            r[size_t(k * 3 + c)] = pred[size_t(k)][size_t(c)] - obs[size_t(f)][size_t(k)][size_t(c)];
                    return r;
                    };
                fit_detail::levenbergMarquardt(a, residualFn, cfg.angleFitIters);
                angles[size_t(f)] = { a[0], a[1], a[2] };
            }

            // (b) 固定angles，解全局params(7维: anchor3 + lengths3 + dipCoupling1)。
            std::vector<double> gp = { params.anchor[0], params.anchor[1], params.anchor[2],
                                       params.lengths[0], params.lengths[1], params.lengths[2],
                                       params.dipCoupling };
            auto residualFnGlobal = [&](const std::vector<double>& v) {
                FingerChainParams p; p.anchor = { v[0],v[1],v[2] }; p.lengths = { v[3],v[4],v[5] };
                p.dipCoupling = v[6];
                std::vector<double> r(size_t(F) * 9);
                for (int f = 0; f < F; ++f) {
                    const auto pred = fingerChainFK(p, angles[size_t(f)][0], angles[size_t(f)][1], angles[size_t(f)][2]);
                    for (int k = 0; k < 3; ++k)
                        for (int c = 0; c < 3; ++c)
                            r[size_t((f * 3 + k) * 3 + c)] = pred[size_t(k)][size_t(c)] - obs[size_t(f)][size_t(k)][size_t(c)];
                }
                return r;
                };
            fit_detail::levenbergMarquardt(gp, residualFnGlobal, cfg.globalFitIters);
            params.anchor = { gp[0],gp[1],gp[2] };
            params.lengths = { gp[3],gp[4],gp[5] };
            // 同 hand/HandSelfCalibration.hpp 里的clamp——dipCoupling是弱观测
            // 方向，数据不够干净时数值解可能跑到不physical的区间，兜底一下。
            params.dipCoupling = std::clamp(gp[6], 0.2, 1.3);

            const auto finalRes = residualFnGlobal(gp);
            double cost = 0.0; for (double r : finalRes) cost += r * r;
            if (std::abs(prevCost - cost) < 1e-6) break;
            prevCost = cost;
        }

        double sq = 0.0; int cnt = 0;
        for (int f = 0; f < F; ++f) {
            const auto pred = fingerChainFK(params, angles[size_t(f)][0], angles[size_t(f)][1], angles[size_t(f)][2]);
            for (int k = 0; k < 3; ++k) {
                const double dx = pred[size_t(k)][0] - obs[size_t(f)][size_t(k)][0];
                const double dy = pred[size_t(k)][1] - obs[size_t(f)][size_t(k)][1];
                const double dz = pred[size_t(k)][2] - obs[size_t(f)][size_t(k)][2];
                sq += dx * dx + dy * dy + dz * dz; ++cnt;
            }
        }
        out.rmsMm = std::sqrt(sq / double(cnt));
        out.params = params;
        out.anglesPerFrame = angles;
        out.framesUsed = F;

        double minFlex = 1e18, maxFlex = -1e18;
        for (auto& a : angles) { minFlex = std::min(minFlex, a[0]); maxFlex = std::max(maxFlex, a[0]); }
        out.romFlexDeg = (maxFlex - minFlex) * 180.0 / M_PI;

        if (out.romFlexDeg < cfg.minRomDeg) {
            out.valid = false;
            out.message = "该指屈伸范围过窄(观测到约 " + std::to_string(out.romFlexDeg) +
                " 度，低于 " + std::to_string(cfg.minRomDeg) +
                " 度门槛)，标定出的连杆长度/锚点置信区间会很宽，建议重新采集时加大屈伸动作幅度。";
            return out;
        }
        if (out.rmsMm > cfg.maxAcceptRmsMm) {
            out.valid = false;
            out.message = "拟合残差偏大(" + std::to_string(out.rmsMm) + "mm，门槛 " +
                std::to_string(cfg.maxAcceptRmsMm) +
                "mm)，可能贴球位置有误、分组把别的指头的点混进来了，或该指存在本模型未覆盖的额外自由度。";
            return out;
        }
        out.valid = true;
        out.message = "标定成功：残差 " + std::to_string(out.rmsMm) + "mm，观测屈伸范围 " +
            std::to_string(out.romFlexDeg) + " 度，用了 " + std::to_string(F) + " 帧。";
        return out;
    }

    struct ThumbCalibConfig {
        int maxOuterIters = 70;
        int angleFitIters = 50;
        int globalFitIters = 100;
        double minFramesRequired = 20;
        double minRomDeg = 15.0;
        double maxAcceptRmsMm = 3.0;
    };

    struct ThumbCalibResult {
        bool valid = false;
        ThumbChainParams params;
        std::vector<std::array<double, 4>> anglesPerFrame;   // cmcFlex, cmcAbduct, mcpFlex, ipFlex
        double rmsMm = -1.0;
        double romFlexDeg = 0.0;
        int framesUsed = 0;
        std::string message;
    };

    inline ThumbCalibResult calibrateThumbChain(const std::vector<std::array<Vec3, 3>>& obs,
        const ThumbCalibConfig& cfg = ThumbCalibConfig{}) {
        ThumbCalibResult out;
        const int F = int(obs.size());
        if (F < int(cfg.minFramesRequired)) {
            out.message = "帧数不足(实际 " + std::to_string(F) + " 帧，至少需要 " +
                std::to_string(int(cfg.minFramesRequired)) + " 帧)，标定结果不可信。";
            return out;
        }

        Vec3 anchorInit{ 0, 0, 0 };
        for (auto& fr : obs) for (int k = 0; k < 3; ++k) anchorInit[size_t(k)] += fr[0][size_t(k)] / double(F);
        double d0 = 0.0, d1 = 0.0;
        for (auto& fr : obs) {
            double dx = fr[0][0] - anchorInit[0], dy = fr[0][1] - anchorInit[1], dz = fr[0][2] - anchorInit[2];
            d0 += std::sqrt(dx * dx + dy * dy + dz * dz) / double(F);
            double ex = fr[1][0] - fr[0][0], ey = fr[1][1] - fr[0][1], ez = fr[1][2] - fr[0][2];
            d1 += std::sqrt(ex * ex + ey * ey + ez * ez) / double(F);
        }
        ThumbChainParams params;
        params.anchor = anchorInit;
        params.baseRotZ = M_PI / 4.0;   // 初值沿用旧占位角度，但会被标定更新
        params.lengths = { std::max(10.0, d0 * 1.6), std::max(10.0, d1 * 1.2), std::max(10.0, d1) };

        std::vector<std::array<double, 4>> angles(size_t(F), std::array<double, 4>{0.1, 0.0, 0.1, 0.1});

        double prevCost = 1e18;
        for (int outerIt = 0; outerIt < cfg.maxOuterIters; ++outerIt) {
            for (int f = 0; f < F; ++f) {
                std::vector<double> a = { angles[size_t(f)][0], angles[size_t(f)][1], angles[size_t(f)][2], angles[size_t(f)][3] };
                auto residualFn = [&](const std::vector<double>& v) {
                    const auto pred = thumbChainFK(params, v[0], v[1], v[2], v[3]);
                    std::vector<double> r(9);
                    for (int k = 0; k < 3; ++k)
                        for (int c = 0; c < 3; ++c)
                            r[size_t(k * 3 + c)] = pred[size_t(k)][size_t(c)] - obs[size_t(f)][size_t(k)][size_t(c)];
                    return r;
                    };
                fit_detail::levenbergMarquardt(a, residualFn, cfg.angleFitIters);
                angles[size_t(f)] = { a[0], a[1], a[2], a[3] };
            }

            // 全局params: anchor3 + baseRotZ1 + lengths3 = 7维。
            std::vector<double> gp = { params.anchor[0], params.anchor[1], params.anchor[2], params.baseRotZ,
                                       params.lengths[0], params.lengths[1], params.lengths[2] };
            auto residualFnGlobal = [&](const std::vector<double>& v) {
                ThumbChainParams p; p.anchor = { v[0],v[1],v[2] }; p.baseRotZ = v[3]; p.lengths = { v[4],v[5],v[6] };
                std::vector<double> r(size_t(F) * 9);
                for (int f = 0; f < F; ++f) {
                    const auto& a = angles[size_t(f)];
                    const auto pred = thumbChainFK(p, a[0], a[1], a[2], a[3]);
                    for (int k = 0; k < 3; ++k)
                        for (int c = 0; c < 3; ++c)
                            r[size_t((f * 3 + k) * 3 + c)] = pred[size_t(k)][size_t(c)] - obs[size_t(f)][size_t(k)][size_t(c)];
                }
                return r;
                };
            fit_detail::levenbergMarquardt(gp, residualFnGlobal, cfg.globalFitIters);
            params.anchor = { gp[0],gp[1],gp[2] }; params.baseRotZ = gp[3]; params.lengths = { gp[4],gp[5],gp[6] };

            const auto finalRes = residualFnGlobal(gp);
            double cost = 0.0; for (double r : finalRes) cost += r * r;
            if (std::abs(prevCost - cost) < 1e-6) break;
            prevCost = cost;
        }

        double sq = 0.0; int cnt = 0;
        for (int f = 0; f < F; ++f) {
            const auto& a = angles[size_t(f)];
            const auto pred = thumbChainFK(params, a[0], a[1], a[2], a[3]);
            for (int k = 0; k < 3; ++k) {
                const double dx = pred[size_t(k)][0] - obs[size_t(f)][size_t(k)][0];
                const double dy = pred[size_t(k)][1] - obs[size_t(f)][size_t(k)][1];
                const double dz = pred[size_t(k)][2] - obs[size_t(f)][size_t(k)][2];
                sq += dx * dx + dy * dy + dz * dz; ++cnt;
            }
        }
        out.rmsMm = std::sqrt(sq / double(cnt));

        // 关键一步：baseRotZ 和 cmcAbduct 都是绕同一根Z轴在同一个位置连续
        // 旋转，rotZ(baseRotZ)*rotZ(cmcAbduct) = rotZ(baseRotZ+cmcAbduct)——
        // 两者只有"和"能从marker运动里观测到，单独拆开在数学上不可辨识
        // (baseRotZ+1度、所有帧cmcAbduct-1度，marker位置完全不变，LM不会
        // 收敛到某个特定拆法，停在哪全看初值)。用"外展角的帧间均值定义为0"
        // 这个约定做 gauge fixing：把均值折算进baseRotZ，之后baseRotZ表示
        // "平均根部朝向"，每帧cmcAbduct表示"相对这个均值的偏移"——这是一次
        // 精确的重参数化(旋转合成律保证每帧FK输出完全不变)，不是重新拟合，
        // 只是把本来就不可分的量按一个明确约定拆开报告，不能指望这个约定
        // 之外还原出"真实"的根部角度，因为那个概念本身没有唯一定义。
        {
            double meanAbduct = 0.0;
            for (auto& a : angles) meanAbduct += a[1] / double(F);
            params.baseRotZ += meanAbduct;
            for (auto& a : angles) a[1] -= meanAbduct;
        }

        out.params = params;
        out.anglesPerFrame = angles;
        out.framesUsed = F;

        double minFlex = 1e18, maxFlex = -1e18;
        for (auto& a : angles) { minFlex = std::min(minFlex, a[2]); maxFlex = std::max(maxFlex, a[2]); }   // mcpFlex
        out.romFlexDeg = (maxFlex - minFlex) * 180.0 / M_PI;

        if (out.romFlexDeg < cfg.minRomDeg) {
            out.valid = false;
            out.message = "拇指屈伸范围过窄(观测到约 " + std::to_string(out.romFlexDeg) +
                " 度)，标定结果不可信，建议加大屈伸/外展动作幅度重新采集。";
            return out;
        }
        if (out.rmsMm > cfg.maxAcceptRmsMm) {
            out.valid = false;
            out.message = "拟合残差偏大(" + std::to_string(out.rmsMm) + "mm)，可能贴球位置有误或分组错误。";
            return out;
        }
        out.valid = true;
        out.message = "拇指标定成功：残差 " + std::to_string(out.rmsMm) + "mm，根部方向角标定为 " +
            std::to_string(params.baseRotZ * 180.0 / M_PI) + " 度，用了 " + std::to_string(F) + " 帧。";
        return out;
    }

} // namespace mocap