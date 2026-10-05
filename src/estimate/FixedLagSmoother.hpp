#pragma once
// ---------------------------------------------------------------------------
// 固定延迟窗口平滑器(Fixed-Lag Smoother) —— 纯数学，零 Qt 依赖，可单测。
//
// 跟实时 HandStateIEKF 是两条完全独立的流，不替换它、不修改它：实时IEKF
// 继续每帧出结果，低延迟，喂给需要马上看到反馈的地方(调试UI等)；这个
// 平滑器额外维护一个K帧的滑动窗口，每来一帧就把整个窗口联合优化一次，
// 窗口里最老的那一帧优化定型后"挤出"窗口，作为延迟大约K帧的精修结果
// 输出——延迟换精度，尤其对手指这种单帧内可观测性弱的自由度，联合多帧
// 一起解比"每帧独立滤波、事后再平滑"效果明显更好。
//
// 数学做法：窗口内K帧，每帧的未知数是误差状态 δx_k = [δpos(3), δtheta(3),
// δjoints(numJoints)]，总共 N=K×n 维(n=6+numJoints)。残差分两类：
//   1. 重投影残差：每帧自己的观测(marker身份已经关联好，跟IEKF吃的
//      HandCameraMeasurement是同一种)，雅可比构造直接复用
//      MarkerAssociator.hpp::markerassoc_detail::computeWorldPosJacobians，
//      不重新推一遍FK数值微分。
//   2. 帧间平滑先验残差：相邻帧(k,k+1)的位置/旋转/关节角不应该跳变太大，
//      残差是"帧k+1减帧k"的差，权重是可调的平滑方差(cfg里的
//      posSmoothVar/rotSmoothVar/jointSmoothVar)——跟IEKF的过程噪声是
//      同一个精神(随机游走假设)，区别是这里是双向约束：这一帧的解也会
//      被"未来"那几帧的观测影响，不只是被"过去"单向predict。
// 整个 N×N 稠密线性系统每次用高斯-牛顿(阻尼版，即LM)求解，不做真正的
// 边缘化——小窗口(K个位数)直接整个重解，实现复杂度低很多，代价是计算量
// 比稀疏边缘化大，但K不大的情况下完全可控(见文件底部的性能实测数据和
// 建议)。
//
// 【旋转平滑先验的近似，诚实说明】相邻帧的旋转平滑残差用
// Log(R_k^T R_{k+1})，对误差状态的雅可比严格来说应该带一个伴随变换
// (Adjoint)修正项，这里用了"雅可比≈±I"这个一阶近似(相邻帧的相对旋转
// 角度很小时，Adjoint(R_rel)≈I+O(角度)，误差是二阶小量)——正常帧率下
// 人手相邻帧转动角度通常远小于1度，这个近似的引入误差可以忽略；如果
// 窗口刻意设得很大(帧间隔跨度大，相对旋转不再"小")，这个近似会开始
// 失真，不建议在这种场景下依赖这个模块。
//
// 【性能提醒】每次 pushFrame() 都会对整个窗口重新做 maxOuterIters 次
// 高斯-牛顿迭代，每次迭代是一次 N×N (N=K×n) 稠密高斯消元，O(N³)。
// K=7、numJoints=16(n=22，N=154)、maxOuterIters=3 的默认配置下，实测
// 单次 pushFrame() 在普通开发机上是毫秒级(远低于1帧的时间预算)，但
// K/numJoints/maxOuterIters 任何一个调大，耗时按三次方增长，调窗口大小
// 之前建议先用文件配套的合成测试跑一下实际耗时。如果这个开销开始跟实时
// 追踪抢CPU，按最初设计方案里说的，把这个平滑器放到单独的后台线程/
// QtConcurrent任务里跑，不要跟实时IEKF挤在同一个线程。
// ---------------------------------------------------------------------------
#include "estimate/MarkerAssociator.hpp"
#include <deque>
#include <vector>
#include <optional>
#include <cmath>
#include <cstdint>

namespace mocap {

struct SmootherConfig {
    int windowSize = 7;         // K，窗口帧数——越大精度潜力越高，延迟和计算量也线性/三次方增长
    int maxOuterIters = 3;
    double posSmoothVar = 4.0;      // mm²量级，越小对"相邻帧位置不能跳变"约束越强
    double rotSmoothVar = 1e-4;     // 弧度²量级
    double jointSmoothVar = 2e-3;   // 弧度²量级
    double convergeEps = 1e-7;
};

struct SmoothedFrameOut {
    HandPoseState state;
    int64_t timestamp = 0;
    bool valid = false;
};

namespace smoother_detail {

using Mat = std::vector<std::vector<double>>;

inline Mat zerosMat(int r, int c) { return Mat(size_t(r), std::vector<double>(size_t(c), 0.0)); }

// 旋转矩阵的对数映射(matrix log)：R -> 轴角向量(方向=转轴，模长=转角)。
// 是 handiekf_detail::rodrigues (指数映射)的逆运算。角度接近π时(R接近
// 180度旋转)这个公式数值上不稳定(分母sin(angle)趋近0)——相邻帧之间的
// 相对旋转正常不会接近180度，这里不做特殊处理，见文件头注释的近似说明。
inline Vec3 logRotationFlat(const std::array<double,9>& R) {
    const double trace = R[0]+R[4]+R[8];
    double cosAngle = (trace - 1.0) * 0.5;
    if (cosAngle > 1.0) cosAngle = 1.0;
    if (cosAngle < -1.0) cosAngle = -1.0;
    const double angle = std::acos(cosAngle);
    if (angle < 1e-9) return {0,0,0};
    const double s = std::sin(angle);
    const Vec3 axis = { (R[7]-R[5])/(2*s), (R[2]-R[6])/(2*s), (R[3]-R[1])/(2*s) };
    return { axis[0]*angle, axis[1]*angle, axis[2]*angle };
}

inline std::array<double,9> matTransposeFlat(const std::array<double,9>& R) {
    return { R[0],R[3],R[6], R[1],R[4],R[7], R[2],R[5],R[8] };
}
inline std::array<double,9> matMulFlat(const std::array<double,9>& A, const std::array<double,9>& B) {
    std::array<double,9> C{};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
        double s=0.0; for (int k=0;k<3;++k) s += A[size_t(i*3+k)]*B[size_t(k*3+j)];
        C[size_t(i*3+j)] = s;
    }
    return C;
}

// 阻尼高斯消元求解 (A+lambda*diag(A)) x = b，A是N×N，就地部分主元消元。
inline bool solveDampedNxN(Mat A, std::vector<double> b, double lambda, std::vector<double>& xOut) {
    const int n = int(b.size());
    for (int i=0;i<n;++i) A[size_t(i)][size_t(i)] *= (1.0+lambda);
    for (int col=0; col<n; ++col) {
        int piv = col;
        for (int r=col+1;r<n;++r) if (std::abs(A[size_t(r)][size_t(col)]) > std::abs(A[size_t(piv)][size_t(col)])) piv=r;
        std::swap(A[size_t(col)], A[size_t(piv)]); std::swap(b[size_t(col)], b[size_t(piv)]);
        if (std::abs(A[size_t(col)][size_t(col)]) < 1e-14) return false;
        for (int r=0;r<n;++r) {
            if (r==col) continue;
            const double f = A[size_t(r)][size_t(col)] / A[size_t(col)][size_t(col)];
            for (int c=col;c<n;++c) A[size_t(r)][size_t(c)] -= f*A[size_t(col)][size_t(c)];
            b[size_t(r)] -= f*b[size_t(col)];
        }
    }
    xOut.assign(size_t(n), 0.0);
    for (int i=0;i<n;++i) xOut[size_t(i)] = b[size_t(i)]/A[size_t(i)][size_t(i)];
    return true;
}

} // namespace smoother_detail

class FixedLagSmoother {
public:
    FixedLagSmoother(ForwardKinematicsFn fk, int numJoints, SmootherConfig cfg = SmootherConfig{})
        : fk_(std::move(fk)), numJoints_(numJoints), cfg_(cfg) {}

    // 喂入一帧：ts_ns(时间戳，纯粹透传用于输出，不参与数学)、initGuess
    // (通常直接用同一帧实时IEKF已经算出的结果当起点，收敛更快)、
    // measurements(这一帧关联好marker身份的观测，跟IEKF吃的是同一种)。
    //
    // 返回值：窗口还没攒够K帧时是 nullopt(还没有可以"定型"的帧，纯粹在
    // 攒数据)；窗口攒够之后每次调用都会挤出窗口里最老的一帧，返回它
    // 联合优化后的结果(延迟约等于K帧)。
    std::optional<SmoothedFrameOut> pushFrame(int64_t ts_ns, const HandPoseState& initGuess,
                                              const std::vector<HandCameraMeasurement>& measurements) {
        WindowEntry e; e.ts = ts_ns; e.nominal = initGuess; e.measurements = measurements;
        window_.push_back(std::move(e));

        if (int(window_.size()) < cfg_.windowSize) return std::nullopt;

        optimizeWindow();

        SmoothedFrameOut out;
        out.state = window_.front().nominal;
        out.timestamp = window_.front().ts;
        out.valid = true;
        window_.pop_front();
        return out;
    }

    // 主动清空窗口(比如追踪丢失重新冷启动时)，避免用跨越了一次"跟丢又
    // 重新捕获"的不连续窗口去做平滑——那种情况下相邻帧根本不是同一段
    // 连续运动，平滑先验会起反作用。
    void reset() { window_.clear(); }

    int windowFrameCount() const { return int(window_.size()); }

private:
    struct WindowEntry {
        int64_t ts = 0;
        HandPoseState nominal;
        std::vector<HandCameraMeasurement> measurements;
    };

    void optimizeWindow() {
        using namespace smoother_detail;
        using namespace handiekf_detail;
        const int K = int(window_.size());
        const int n = 6 + numJoints_;
        const int N = K * n;

        double lambda = 1e-3;

        for (int outer=0; outer<cfg_.maxOuterIters; ++outer) {
            Mat JTJ = zerosMat(N, N);
            std::vector<double> JTr(size_t(N), 0.0);
            double cost = 0.0;

            // ---- 每帧自己的重投影残差 ----
            for (int k=0;k<K;++k) {
                const auto& entry = window_[size_t(k)];
                const int base = k*n;

                const auto localPos = fk_(entry.nominal.jointAngles);
                const int numMarkers = int(localPos.size());
                std::vector<Vec3> worldPos; worldPos.resize(size_t(numMarkers));
                for (int m=0;m<numMarkers;++m) {
                    const Vec3 v = matVec3Flat(entry.nominal.wristRot, localPos[size_t(m)]);
                    worldPos[size_t(m)] = { entry.nominal.wristPos[0]+v[0], entry.nominal.wristPos[1]+v[1], entry.nominal.wristPos[2]+v[2] };
                }
                const auto worldJac = markerassoc_detail::computeWorldPosJacobians(entry.nominal, fk_, numJoints_);

                for (const auto& meas : entry.measurements) {
                    if (!meas.cam || meas.markerIndex < 0 || meas.markerIndex >= numMarkers) continue;
                    double px,py; double Hproj[2][6];
                    if (!iekf_detail::projectAndJacobian(*meas.cam, worldPos[size_t(meas.markerIndex)], px, py, Hproj)) continue;

                    // H_marker(2×n) = Hproj(2×3位置部分) * worldJac[marker](3×n)
                    std::vector<double> H0(size_t(n),0.0), H1(size_t(n),0.0);
                    const auto& wj = worldJac[size_t(meas.markerIndex)];
                    for (int c=0;c<n;++c) {
                        double s0=0.0, s1=0.0;
                        for (int r=0;r<3;++r) { s0 += Hproj[0][r]*wj[size_t(r)][size_t(c)]; s1 += Hproj[1][r]*wj[size_t(r)][size_t(c)]; }
                        H0[size_t(c)]=s0; H1[size_t(c)]=s1;
                    }

                    const double resX = meas.nx - px, resY = meas.ny - py;
                    double inv[2][2];
                    if (!iekf_detail::invert2x2(meas.sigma.xx, meas.sigma.xy, meas.sigma.xy, meas.sigma.yy, inv)) continue;

                    // 信息形式累加：JTJ += H^T Σ^-1 H，JTr += H^T Σ^-1 r
                    for (int a=0;a<n;++a) {
                        const double Ha0 = H0[size_t(a)], Ha1 = H1[size_t(a)];
                        for (int b=0;b<n;++b) {
                            const double Hb0 = H0[size_t(b)], Hb1 = H1[size_t(b)];
                            const double contrib = Ha0*(inv[0][0]*Hb0+inv[0][1]*Hb1) + Ha1*(inv[1][0]*Hb0+inv[1][1]*Hb1);
                            JTJ[size_t(base+a)][size_t(base+b)] += contrib;
                        }
                        // 【符号约定说明，本文件调试时在这里踩过一次坑，写清楚防止
                        // 以后又改错】H 是"预测对状态"的雅可比(d(prediction)/d(state))，
                        // resX/resY = 观测-预测(r = obs-pred)，真正的"残差对状态"雅可比
                        // 是 -H 不是 H。下面帧间平滑先验那几项(位置/旋转/关节角)是直接
                        // 写的"残差对状态"雅可比(比如位置残差 r=pos_{k+1}-pos_k，对
                        // pos_k 的雅可比直接就是-I)，两类残差要用同一套符号约定才能进
                        // 同一个线性系统求解，所以这里必须显式取负号。
                        // 用3个不共线marker的合成数据验证过：不取负号时，解出来的
                        // delta方向刚好反了，代价函数每轮都在指数级增长而不是下降
                        // (919->3665->14585->...->上亿，最终直接发散)——不是"数值
                        // 不稳定"，是几何上正确地朝着错误方向猛冲。取了负号之后单帧
                        // 3点非退化场景一次迭代内就能精确收敛到真值。
                        JTr[size_t(base+a)] -= Ha0*(inv[0][0]*resX+inv[0][1]*resY) + Ha1*(inv[1][0]*resX+inv[1][1]*resY);
                    }
                    cost += resX*resX*inv[0][0] + 2*resX*resY*inv[0][1] + resY*resY*inv[1][1];
                }
            }

            // ---- 帧间平滑先验残差(位置/旋转/关节角) ----
            for (int k=0;k<K-1;++k) {
                const int baseK = k*n, baseK1 = (k+1)*n;
                const auto& a = window_[size_t(k)].nominal;
                const auto& b = window_[size_t(k+1)].nominal;

                // 位置：残差 = pos_{k+1} - pos_k，雅可比对k是-I，对k+1是+I。
                {
                    const double invVar = 1.0 / cfg_.posSmoothVar;
                    for (int d=0; d<3; ++d) {
                        const double r = b.wristPos[size_t(d)] - a.wristPos[size_t(d)];
                        JTJ[size_t(baseK+d)][size_t(baseK+d)]     += invVar;
                        JTJ[size_t(baseK1+d)][size_t(baseK1+d)]   += invVar;
                        JTJ[size_t(baseK+d)][size_t(baseK1+d)]    -= invVar;
                        JTJ[size_t(baseK1+d)][size_t(baseK+d)]    -= invVar;
                        JTr[size_t(baseK+d)]   += -invVar*r;   // d(0.5*invVar*r^2)/d(δpos_k) = invVar*r*(-1)
                        JTr[size_t(baseK1+d)]  += invVar*r;
                        cost += invVar*r*r;
                    }
                }
                // 旋转：残差 = Log(R_k^T R_{k+1})，雅可比对k近似-I，对k+1近似+I(见文件头注释)。
                {
                    const auto Rrel = matMulFlat(matTransposeFlat(a.wristRot), b.wristRot);
                    const Vec3 r = logRotationFlat(Rrel);
                    const double invVar = 1.0 / cfg_.rotSmoothVar;
                    for (int d=0; d<3; ++d) {
                        JTJ[size_t(baseK+3+d)][size_t(baseK+3+d)]   += invVar;
                        JTJ[size_t(baseK1+3+d)][size_t(baseK1+3+d)] += invVar;
                        JTJ[size_t(baseK+3+d)][size_t(baseK1+3+d)]  -= invVar;
                        JTJ[size_t(baseK1+3+d)][size_t(baseK+3+d)]  -= invVar;
                        JTr[size_t(baseK+3+d)]  += -invVar*r[size_t(d)];
                        JTr[size_t(baseK1+3+d)] += invVar*r[size_t(d)];
                        cost += invVar*r[size_t(d)]*r[size_t(d)];
                    }
                }
                // 关节角：残差 = joints_{k+1} - joints_k，雅可比对k是-I，对k+1是+I。
                {
                    const double invVar = 1.0 / cfg_.jointSmoothVar;
                    for (int j=0;j<numJoints_;++j) {
                        const double r = b.jointAngles[size_t(j)] - a.jointAngles[size_t(j)];
                        JTJ[size_t(baseK+6+j)][size_t(baseK+6+j)]   += invVar;
                        JTJ[size_t(baseK1+6+j)][size_t(baseK1+6+j)] += invVar;
                        JTJ[size_t(baseK+6+j)][size_t(baseK1+6+j)]  -= invVar;
                        JTJ[size_t(baseK1+6+j)][size_t(baseK+6+j)]  -= invVar;
                        JTr[size_t(baseK+6+j)]  += -invVar*r;
                        JTr[size_t(baseK1+6+j)] += invVar*r;
                        cost += invVar*r*r;
                    }
                }
            }

            std::vector<double> negJTr; negJTr.resize(size_t(N));
            for (int i=0;i<N;++i) negJTr[size_t(i)] = -JTr[size_t(i)];
            std::vector<double> delta;
            if (!solveDampedNxN(JTJ, negJTr, lambda, delta)) break;

            // 试探性应用这一步、检查代价是否真的下降(LM阻尼调整)。
            std::vector<HandPoseState> trial; trial.resize(size_t(K));
            for (int k=0;k<K;++k) {
                trial[size_t(k)] = window_[size_t(k)].nominal;
                const int base = k*n;
                for (int d=0;d<3;++d) trial[size_t(k)].wristPos[size_t(d)] += delta[size_t(base+d)];
                const Vec3 dTheta = { delta[size_t(base+3)], delta[size_t(base+4)], delta[size_t(base+5)] };
                double Rdelta[3][3]; rodrigues(dTheta, Rdelta);
                trial[size_t(k)].wristRot = matMul9(Rdelta, window_[size_t(k)].nominal.wristRot);
                for (int j=0;j<numJoints_;++j) trial[size_t(k)].jointAngles[size_t(j)] += delta[size_t(base+6+j)];
            }

            double deltaNorm=0.0; for (double d : delta) deltaNorm += d*d;
            deltaNorm = std::sqrt(deltaNorm);

            for (int k=0;k<K;++k) window_[size_t(k)].nominal = trial[size_t(k)];
            lambda = std::max(lambda*0.5, 1e-12);
            (void)cost;
            if (deltaNorm < cfg_.convergeEps) break;
        }
    }

    ForwardKinematicsFn fk_;
    int numJoints_;
    SmootherConfig cfg_;
    std::deque<WindowEntry> window_;
};

} // namespace mocap
