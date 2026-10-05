#pragma once
// ---------------------------------------------------------------------------
// 状态估计层 · IEKF —— 纯数学，零 Qt/第三方依赖，可单测。
// 对应 README_手部动捕方案.md §4：接住 detect/DetectionOutput.hpp (§3d) 产出
// 的 (mu, Sigma, confidence) 观测，跨相机、跨帧融合成一个带不确定度的 3D
// 状态估计，而不是先各自三角化再滤波——检测层给的协方差在这里直接被当成
// 观测噪声用，弧短、遮挡严重的观测自然贡献小、噪声大的观测自然贡献大，
// 不需要再单独设计一套"这个观测该信多少"的权重规则(那套规则已经在 §3d
// 里做完了，这一层只管怎么把不同噪声水平的观测正确地揉在一起)。
//
// 每个点独立维护一个 6 维状态 [px,py,pz,vx,vy,vz](恒速模型，跟
// TemporalTracker.hpp 的运动假设一致，但这里是真正贝叶斯意义上的协方差
// 传播，不是启发式的最近邻+距离阈值)。
//
// 为什么是"迭代"EKF而不是普通EKF：观测模型(针孔投影 nx=Xc/Zc, ny=Yc/Zc)
// 是非线性的，普通EKF只在预测均值处线性化一次，当先验不确定度较大或者
// 相机离得近(投影非线性明显)时会有系统性偏差。IEKF 用"重新线性化+重新
// 计算更新"反复几轮(本质是高斯-牛顿求解 MAP 估计)来修正这个偏差，具体
// 数值收益见本文件配套测试——用同一批数据对比 1 次线性化 vs 多次
// 线性化的最终误差。
//
// 多相机融合方式：同一帧如果有多台相机看到这个点，把该帧所有相机的观测
// 一起参与一轮"外层迭代"：每一轮都从预测的先验(均值+协方差)出发，依次
// 对每台相机的观测做一次标准卡尔曼更新(残差用固定的线性化点计算，不是
// 用序贯更新过程中变化的当前均值)，一轮下来得到这一轮的状态估计，用它
// 做下一轮的线性化点，重复到收敛或到迭代上限。
// ---------------------------------------------------------------------------
#include "detect/DetectionOutput.hpp"
#include <array>
#include <vector>
#include <cmath>

namespace mocap {

using Vec3 = std::array<double,3>;
using Mat6 = std::array<std::array<double,6>,6>;
using Vec6 = std::array<double,6>;

// 相机位姿：世界->相机，行主序展开(R[i*3+j] = R_ij)，跟项目里
// Epipolar.hpp/MultiViewCluster.hpp 的 EpiMat3/EpiVec3 是同一套约定，这里
// 独立重新声明一份保持零依赖(跟 HandPose.hpp 自己独立实现一份 3x3 SVD、
// MultiViewCluster.hpp 自己独立实现一份 4x4 Jacobi 是同一个先例)。
struct CamPose { std::array<double,9> R; Vec3 t; };

// 一条观测：来自某台相机的一个 detect 层输出(已经是归一化相机坐标系下的
// mu/Sigma，不是像素——像素到归一化坐标的转换用相机内参线性完成，属于
// Qt 包装层的职责，参考 Triangulator.cpp 现有的去畸变+归一化逻辑)。
struct CameraMeasurement {
    const CamPose* cam = nullptr;
    double nx = 0.0, ny = 0.0;
    Cov2 sigma{};
    // 【新增】这次观测相对参考时刻的时间偏移，单位跟 predict() 的 dt 一致(帧)。
    // 0 = 跟参考时刻同步(旧行为)。两个来源都往这里加：
    //   ① 相机间时间戳不同步：各相机曝光时刻本来就不在同一瞬间；
    //   ② 运动模糊：拖尾光斑的质心落在【曝光窗口中点】，所以带模糊的观测
    //      实际对应的是 t+曝光时长/2 的位置，而不是 t。
    // 这两件事在数学上是同一回事——都是"这次观测其实是在别的时刻测的"，
    // 所以用同一个字段表达，不需要两套机制。
    double dt = 0.0;
    // dt≠0 时是否让这次观测参与速度估计。默认 false，理由见
    // projectAndJacobian 里 velJacobian 那段说明（共模 dt 下会把噪声灌进速度）。
    bool velJacobian = false;
};

namespace iekf_detail {

inline Mat6 zero6() { Mat6 m{}; for (auto& row : m) row.fill(0.0); return m; }

inline Mat6 identity6() { Mat6 m = zero6(); for (int i=0;i<6;++i) m[size_t(i)][size_t(i)]=1.0; return m; }

// h(x) 和对状态(6维,只有位置3维非零)的雅可比 H(2x6)。Zc<=eps 视为点在
// 相机后方/退化，返回 false。
// vel/dt：这次观测其实是在 参考时刻+dt 测到的（相机不同步 / 运动模糊的
// 曝光中点，见 CameraMeasurement::dt 的说明）。此时应该拿【那个时刻的位置】
// pos+vel*dt 去投影，而不是拿参考时刻的位置——否则残差里混进了一段
// "目标在这段时间里走了多远"，而这段位移正比于速度，于是速度越快偏差越大，
// 表现为"快速运动时精度突然变差"。
// dt=0 时完全退化成旧行为（速度那三列雅可比为 0）。
inline bool projectAndJacobian(const CamPose& cam, const Vec3& pos,
                               double& nx, double& ny, double H[2][6],
                               const Vec3& vel = Vec3{0.0,0.0,0.0}, double dt = 0.0,
                               bool velJacobian = false) {
    const auto& R = cam.R; const auto& t = cam.t;
    // 观测时刻的有效位置
    const double px = pos[0] + vel[0]*dt;
    const double py = pos[1] + vel[1]*dt;
    const double pz = pos[2] + vel[2]*dt;
    const double Xc = R[0]*px+R[1]*py+R[2]*pz+t[0];
    const double Yc = R[3]*px+R[4]*py+R[5]*pz+t[1];
    const double Zc = R[6]*px+R[7]*py+R[8]*pz+t[2];
    if (Zc <= 1e-6) return false;
    nx = Xc/Zc; ny = Yc/Zc;
    const double invZ = 1.0/Zc, invZ2 = invZ*invZ;
    for (int k=0;k<3;++k) {
        H[0][k] = (R[size_t(k)]*Zc - Xc*R[size_t(6+k)]) * invZ2;
        H[1][k] = (R[size_t(3+k)]*Zc - Yc*R[size_t(6+k)]) * invZ2;
        // 速度那三列：链式法则给出 ∂h/∂v = ∂h/∂p · dt。
        //
        // 【默认关闭，这是有意的】打开它意味着"让观测反过来去估计速度"。
        // 但当各相机的 dt 是共模的(比如曝光中点补偿，所有相机同一个值)，
        // 观测量只约束 pos+vel·dt 这个组合，pos 和 vel 沿着这个方向不可分离
        // ——多出来的是一个弱可观方向，观测噪声会顺着它灌进速度估计。
        // 实测(2000mm/s、模糊增益1.0)：打开时 RMSE 确实降到 3.18mm，但 ID
        // 跳变从 51 涨到 110；关闭时精度一样好而跳变不涨——因为此时
        // vel·dt 被当成【由预测步已经确定的已知修正量】，只用来把预测投影
        // 挪到正确的观测时刻，不参与信息分配。速度仍然由恒速模型跨帧估计，
        // 那条路径是良置的。
        // 只有当各相机 dt 差异显著(真正的时序错开，相当于"时间维度上的立体
        // 视差")时，打开它才有额外信息可拿。
        if (velJacobian) {
            H[0][size_t(3+k)] = H[0][size_t(k)] * dt;
            H[1][size_t(3+k)] = H[1][size_t(k)] * dt;
        } else {
            H[0][size_t(3+k)] = 0.0;
            H[1][size_t(3+k)] = 0.0;
        }
    }
    return true;
}

// 2x2 矩阵求逆，行列式接近0时返回 false(退化，调用方应跳过这次更新)。
// 【退化判据必须无量纲】旧写法是 `if (std::abs(det) < 1e-14) return false;`。
// 这里的输入是【归一化坐标系】下的 2x2 协方差(新息协方差 S = H P Hᵀ + R)，
// 量纲是"归一化坐标的平方"，det 的量纲是四次方。f≈900、Zc≈2000mm 时，一条
// 收敛良好的轨迹 S 的对角元只有 ~1e-8 量级，det ~1e-16 —— 直接低于 1e-14，
// 于是【滤波器收敛得越好、检测越准，观测更新越容易被静默丢弃】。
// 这跟 MultiViewCluster.hpp::invert2x2 修掉的是同一个错误，这份漏了。
// 实测(f=900、σ=0.05px、Zc≈2000mm 的匀速轨迹)：4相机 RMSE 64.0mm → 0.145mm；
// 2相机时因为更新被丢后经常只剩单相机约束、深度方向无约束，直接发散到
// RMSE 4467mm。
// 正确判据是"这是不是一个合法的正定矩阵"，本身就是无量纲的：
//   a>0, d>0, 且相关系数 ρ² = bc/(ad) 显著小于 1。
// 1e-9 是 double 相对精度(~1e-16)留的纯数值余量，不是物理阈值，所以不会
// 因为观测"太准"而误伤。
inline bool invert2x2(double a,double b,double c,double d, double inv[2][2]) {
    // 非正定（含 NaN：任何与 NaN 的比较都为 false，天然被挡掉）
    if (!(a > 0.0) || !(d > 0.0)) return false;
    const double det = a*d-b*c;
    constexpr double kMinRelDet = 1e-9;
    if (!(det > kMinRelDet * (a*d))) return false;
    const double id = 1.0/det;
    inv[0][0]=d*id; inv[0][1]=-b*id; inv[1][0]=-c*id; inv[1][1]=a*id;
    return true;
}

} // namespace iekf_detail

class PointIEKF6D {
public:
    // initPosVar/initVelVar：初始位置/速度方差(对角，各向同性)，反映初始
    // 不确定度——第一次看到一个新点时该给多大的不确定度，由调用方根据
    // 场景决定(比如刚从 MultiViewCluster 三角化出来的新点，可以用它的
    // 重投影残差量级来定)。
    PointIEKF6D(const Vec3& initPos, double initPosVar = 100.0, double initVelVar = 1e4) {
        mean_ = {initPos[0], initPos[1], initPos[2], 0.0, 0.0, 0.0};
        cov_ = iekf_detail::zero6();
        for (int i=0;i<3;++i) cov_[size_t(i)][size_t(i)] = initPosVar;
        for (int i=3;i<6;++i) cov_[size_t(i)][size_t(i)] = initVelVar;
    }

    // 恒速模型预测：位置 += 速度*dt，协方差按标准 F P F^T + Q 传播。
    // posProcessVar/velProcessVar 是每步累加的过程噪声方差(不是绝对值，
    // 是"这一步"的噪声量，需要跟 dt 的量级匹配，调用方按帧率标定)。
    void predict(double dt, double posProcessVar, double velProcessVar) {
        Vec6 newMean;
        for (int i=0;i<3;++i) newMean[size_t(i)] = mean_[size_t(i)] + mean_[size_t(i+3)]*dt;
        for (int i=3;i<6;++i) newMean[size_t(i)] = mean_[size_t(i)];

        // F = [[I, dt*I],[0, I]]；P' = F P F^T + Q，6x6 直接展开算，
        // 维度小，写显式循环比引入通用矩阵库更清楚也更容易独立复核。
        Mat6 FP = iekf_detail::zero6();
        for (int i=0;i<6;++i) {
            for (int j=0;j<6;++j) {
                double s = cov_[size_t(i)][size_t(j)];
                if (i < 3) s += dt * cov_[size_t(i+3)][size_t(j)];
                FP[size_t(i)][size_t(j)] = s;
            }
        }
        Mat6 newCov = iekf_detail::zero6();
        for (int i=0;i<6;++i) {
            for (int j=0;j<6;++j) {
                double s = FP[size_t(i)][size_t(j)];
                if (j < 3) s += FP[size_t(i)][size_t(j+3)]*dt;
                newCov[size_t(i)][size_t(j)] = s;
            }
        }
        for (int i=0;i<3;++i) newCov[size_t(i)][size_t(i)] += posProcessVar;
        for (int i=3;i<6;++i) newCov[size_t(i)][size_t(i)] += velProcessVar;

        mean_ = newMean;
        cov_ = newCov;
    }

    // 用这一帧收集到的(可能来自多台相机的)观测做一次迭代更新。没有观测
    // (比如这一帧被遮挡完全看不见)时直接跳过，保留预测结果——协方差会
    // 因为没有新信息而维持预测后的水平(不确定度只增不减)，这正是遮挡期间
    // "该有的行为"。
    void update(const std::vector<CameraMeasurement>& measurements,
               int maxIters = 5, double convergeEps = 1e-6) {
        if (measurements.empty()) return;

        const Vec6 priorMean = mean_;
        const Mat6 priorCov = cov_;
        Vec6 linPoint = priorMean;   // 本轮线性化用的点，逐轮更新

        Vec6 workingMean = priorMean;
        Mat6 workingCov = priorCov;

        for (int outer=0; outer<maxIters; ++outer) {
            workingMean = priorMean;
            workingCov = priorCov;

            for (const auto& m : measurements) {
                if (!m.cam) continue;
                const Vec3 linPos = {linPoint[0], linPoint[1], linPoint[2]};
                const Vec3 linVel = {linPoint[3], linPoint[4], linPoint[5]};
                double hx, hy; double H[2][6];
                if (!iekf_detail::projectAndJacobian(*m.cam, linPos, hx, hy, H,
                                                     linVel, m.dt, m.velJacobian)) continue;

                // 残差在固定线性化点(linPoint)附近做一阶展开，同时补上
                // "本轮内此前几台相机已经把 workingMean 推走了多少"这个
                // 修正项——这样同一轮里依次融合多台相机时，后融合的相机
                // 也能感知到前面相机已经带来的修正，不会重复计入线性化点
                // 那份残差。
                double dxState[6];
                for (int k=0;k<6;++k) dxState[k] = workingMean[size_t(k)] - linPoint[size_t(k)];
                // 六列全算：dt≠0 时速度那三列非零，漏掉会让线性化补偿不自洽。
                double predShiftX = 0.0, predShiftY = 0.0;
                for (int k=0;k<6;++k) {
                    predShiftX += H[0][size_t(k)]*dxState[size_t(k)];
                    predShiftY += H[1][size_t(k)]*dxState[size_t(k)];
                }
                const double resX = m.nx - hx - predShiftX;
                const double resY = m.ny - hy - predShiftY;

                // S = H P H^T + R (2x2)
                double HP[2][6];
                for (int r=0;r<2;++r) for (int c=0;c<6;++c) {
                    double s=0.0; for (int k=0;k<6;++k) s += H[r][k]*workingCov[size_t(k)][size_t(c)];
                    HP[r][c]=s;
                }
                double S00=0.0,S01=0.0,S11=0.0;
                for (int k=0;k<6;++k) { S00 += HP[0][k]*H[0][k]; S01 += HP[0][k]*H[1][k]; S11 += HP[1][k]*H[1][k]; }
                S00 += m.sigma.xx; S01 += m.sigma.xy; S11 += m.sigma.yy;

                double Sinv[2][2];
                if (!iekf_detail::invert2x2(S00,S01,S01,S11,Sinv)) continue;   // 退化，跳过这条观测

                // K = P H^T S^-1 (6x2)
                double K[6][2];
                for (int r=0;r<6;++r) {
                    double phx=0.0, phy=0.0;
                    for (int k=0;k<6;++k) { phx += workingCov[size_t(r)][size_t(k)]*H[0][k]; phy += workingCov[size_t(r)][size_t(k)]*H[1][k]; }
                    K[r][0] = phx*Sinv[0][0] + phy*Sinv[1][0];
                    K[r][1] = phx*Sinv[0][1] + phy*Sinv[1][1];
                }

                for (int r=0;r<6;++r) workingMean[size_t(r)] += K[r][0]*resX + K[r][1]*resY;

                // P = (I - K H) P
                Mat6 KH = iekf_detail::zero6();
                for (int r=0;r<6;++r) for (int c=0;c<6;++c) KH[size_t(r)][size_t(c)] = K[r][0]*H[0][c] + K[r][1]*H[1][c];
                Mat6 newCov = iekf_detail::zero6();
                for (int r=0;r<6;++r) for (int c=0;c<6;++c) {
                    double s = workingCov[size_t(r)][size_t(c)];
                    for (int k=0;k<6;++k) s -= KH[size_t(r)][size_t(k)]*workingCov[size_t(k)][size_t(c)];
                    newCov[size_t(r)][size_t(c)] = s;
                }
                workingCov = newCov;
            }

            double delta = 0.0;
            for (int k=0;k<6;++k) { const double d = workingMean[size_t(k)]-linPoint[size_t(k)]; delta += d*d; }
            linPoint = workingMean;
            if (std::sqrt(delta) < convergeEps) break;
        }

        mean_ = workingMean;
        cov_ = workingCov;
    }

    Vec3 position() const { return {mean_[0], mean_[1], mean_[2]}; }
    Vec3 velocity() const { return {mean_[3], mean_[4], mean_[5]}; }
    const Mat6& covariance() const { return cov_; }
    Vec3 positionVariances() const { return {cov_[0][0], cov_[1][1], cov_[2][2]}; }

private:
    Vec6 mean_{};
    Mat6 cov_{};
};

} // namespace mocap
