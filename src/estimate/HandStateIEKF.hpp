#pragma once
// ---------------------------------------------------------------------------
// 状态估计层 · 手部位姿 IEKF —— 纯数学，零 Qt 依赖，可单测。
// 对应 README §4 的延伸：把 PointIEKF.hpp(每个点独立滤波) 升级成"手腕刚体
// 位姿 + 16 维关节角"一体状态，用正向运动学(FK)直接当观测模型，而不是先
// 把 20 颗球分别滤波成独立 3D 点、再用 HandPose.hpp 的 Kabsch 单独拟合刚
// 体——那种两步走的做法丢信息:每颗球的观测噪声独立传播，不知道"这 20 颗
// 球其实是同一个刚体+16个关节角联合决定的，互相之间有强约束"，本该更准
// (观测多、自由度少)的地方反而没利用上。
//
// 【范围声明——这一层依赖两个还没在本项目内建好的前提，用注入的方式解耦，
// 不在这里假装已经解决】：
//   1. 正向运动学本身：不在这个头文件里重新实现 HandModel.hpp 的 FK(那
//      属于手部运动学模型的职责，不应该在状态估计层里复制一份)，而是
//      通过 ForwardKinematicsFn 回调注入——调用方把 HandModel.hpp 的
//      FK 包一层传进来即可，状态估计层完全不关心 FK 内部是怎么算的，
//      只要求"给一组关节角，吐出所有 marker 在手腕局部系的位置"这个
//      接口。这样真正接入项目时不需要这个文件的 API 跟 HandModel.hpp
//      的实际签名对齐，减少两边耦合出错的风险。
//   2. Marker 身份对应(这一帧检测到的哪个观测对应第几号 marker)：这个
//      模块假设调用方已经解决了"谁是谁"的问题(每条 CameraMeasurement
//      带一个 markerIndex)，本身不做数据关联。现实中这需要一个初始的
//      刚体模板匹配(手背5点用 HandPose.hpp 的 Kabsch 冷启动定位) +
//      每帧用上一帧的位姿预测各 marker 的期望像素位置去认领新观测(类似
//      TemporalTracker.hpp 的思路，但作用在投影平面而不是3D点)，这部分
//      对应关联逻辑不在本文件范围内，是下一步要补的。
//
// 因为正向运动学的具体解析雅可比依赖 HandModel.hpp 内部实现细节(可能是
// 一串旋转矩阵乘法链)，这里不要求调用方提供解析雅可比，而是用中心差分
// 数值求 d(局部marker位置)/d(关节角)——16~20 个自由度、几十颗 marker，
// 这点计算量对 mocap 帧率不是瓶颈，换来的是跟任何 FK 实现都能直接对接、
// 不用同步维护两份雅可比推导。
//
// 手腕旋转用"世界系左扰动 + 每轮迭代后 Rodrigues 折算回名义旋转矩阵"的
// 误差状态(error-state)写法，这是 SO(3) 状态在 (I)EKF 里的标准处理方式，
// 不是发明新东西:局部扰动 δθ 满足 R_new ≈ Exp(δθ)·R_nominal，其中
// Exp(δθ) 是 Rodrigues 公式；worldPos 对 δθ 的雅可比是 -[v]_x(v 是
// marker 到手腕原点的世界系向量的叉乘反对称矩阵)，推导过程见函数内注释。
// ---------------------------------------------------------------------------
#include "estimate/PointIEKF.hpp"   // 复用 CamPose / Cov2 / 相机投影约定
#include <vector>
#include <array>
#include <functional>
#include <cmath>

namespace mocap {

// 给一组关节角，返回所有 marker 在"手腕局部系"的 3D 位置(顺序固定、跟
// markerIndex 对应)。真正接入时传入包了一层 HandModel.hpp::forwardKinematics
// 的 lambda 即可。
using ForwardKinematicsFn = std::function<std::vector<Vec3>(const std::vector<double>&)>;

struct HandPoseState {
    Vec3 wristPos{0,0,0};
    std::array<double,9> wristRot{1,0,0, 0,1,0, 0,0,1};   // 行主序，手腕->世界
    std::vector<double> jointAngles;   // 长度 = numJoints
};

// 一条观测：某台相机看到的、已知对应第 markerIndex 号 marker 的 2D 检测
// (来自 DetectionOutput.hpp，归一化相机坐标系下的 mu/Sigma)。
struct HandCameraMeasurement {
    const CamPose* cam = nullptr;
    int markerIndex = -1;
    double nx = 0.0, ny = 0.0;
    Cov2 sigma{};
};

namespace handiekf_detail {

using Mat = std::vector<std::vector<double>>;

inline Mat zeros(int r, int c) { return Mat(size_t(r), std::vector<double>(size_t(c), 0.0)); }
inline Mat identity(int n) { Mat m = zeros(n,n); for (int i=0;i<n;++i) m[size_t(i)][size_t(i)]=1.0; return m; }

// v x (叉乘) 的反对称矩阵 [v]_x，满足 [v]_x * w = v × w。
inline void skew(const Vec3& v, double out[3][3]) {
    out[0][0]=0;      out[0][1]=-v[2];  out[0][2]=v[1];
    out[1][0]=v[2];   out[1][1]=0;      out[1][2]=-v[0];
    out[2][0]=-v[1];  out[2][1]=v[0];   out[2][2]=0;
}

// Rodrigues: 轴角向量 theta(方向=转轴，模长=转角，弧度) -> 3x3 旋转矩阵。
inline void rodrigues(const Vec3& theta, double R[3][3]) {
    const double angle = std::sqrt(theta[0]*theta[0]+theta[1]*theta[1]+theta[2]*theta[2]);
    if (angle < 1e-12) { for(int i=0;i<3;++i) for(int j=0;j<3;++j) R[i][j] = (i==j)?1.0:0.0; return; }
    const Vec3 axis = {theta[0]/angle, theta[1]/angle, theta[2]/angle};
    double K[3][3]; skew(axis, K);
    const double s = std::sin(angle), c = std::cos(angle);
    // R = I + s*K + (1-c)*K^2
    double K2[3][3];
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
        double sum=0.0; for (int k=0;k<3;++k) sum += K[i][k]*K[k][j];
        K2[i][j]=sum;
    }
    for (int i=0;i<3;++i) for (int j=0;j<3;++j)
        R[i][j] = (i==j?1.0:0.0) + s*K[i][j] + (1.0-c)*K2[i][j];
}

inline Vec3 matVec3(const double R[3][3], const Vec3& v) {
    return { R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
             R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
             R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2] };
}
inline Vec3 matVec3Flat(const std::array<double,9>& R, const Vec3& v) {
    return { R[0]*v[0]+R[1]*v[1]+R[2]*v[2],
             R[3]*v[0]+R[4]*v[1]+R[5]*v[2],
             R[6]*v[0]+R[7]*v[1]+R[8]*v[2] };
}
inline std::array<double,9> matMul9(const double A[3][3], const std::array<double,9>& B) {
    std::array<double,9> out{};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) {
        double s=0.0; for (int k=0;k<3;++k) s += A[i][k]*B[size_t(k*3+j)];
        out[size_t(i*3+j)] = s;
    }
    return out;
}

} // namespace handiekf_detail

class HandStateIEKF {
public:
    HandStateIEKF(ForwardKinematicsFn fk, int numJoints, const HandPoseState& initState,
                 double initPosVar = 100.0, double initRotVar = 0.05, double initJointVar = 0.05)
        : fk_(std::move(fk)), numJoints_(numJoints), nominal_(initState) {
        const int n = dim();
        cov_ = handiekf_detail::identity(n);
        for (int i=0;i<3;++i) cov_[size_t(i)][size_t(i)] = initPosVar;
        for (int i=3;i<6;++i) cov_[size_t(i)][size_t(i)] = initRotVar;
        for (int i=6;i<n;++i) cov_[size_t(i)][size_t(i)] = initJointVar;
    }

    int dim() const { return 6 + numJoints_; }

    // 随机游走过程模型(没有单独的速度状态——手腕位姿+关节角之间已经有强
    // 运动学约束，速度动力学留给上层根据实际情况扩展；这里只做"不确定度
    // 随时间增长"这一步，均值不变)。
    void predict(double posProcessVar, double rotProcessVar, double jointProcessVar) {
        const int n = dim();
        for (int i=0;i<3;++i) cov_[size_t(i)][size_t(i)] += posProcessVar;
        for (int i=3;i<6;++i) cov_[size_t(i)][size_t(i)] += rotProcessVar;
        for (int i=6;i<n;++i) cov_[size_t(i)][size_t(i)] += jointProcessVar;
    }

    // 迭代更新：每一轮用当前名义状态(nominal_)重新算 FK + 雅可比，在切
    // 空间(误差状态)里依次序贯融合这一帧所有相机的观测，这一轮融合完的
    // 误差量折算回名义状态(手腕旋转用 Rodrigues 合成，位置/关节角直接
    // 相加)，误差状态清零，进入下一轮重新线性化，直到收敛或到迭代上限。
    void update(const std::vector<HandCameraMeasurement>& measurements,
               int maxIters = 6, double convergeEps = 1e-7) {
        if (measurements.empty()) return;
        using namespace handiekf_detail;
        const int n = dim();

        for (int outer=0; outer<maxIters; ++outer) {
            const size_t numJointsSz = size_t(numJoints_);
            // 当前名义状态下，所有 marker 的局部位置 + 对关节角的数值雅可比。
            const auto localPos = fk_(nominal_.jointAngles);
            const int numMarkers = int(localPos.size());
            const size_t numMarkersSz = size_t(numMarkers);
            std::vector<std::vector<Vec3>> dLocalDJoint(numJointsSz);   // [joint][marker]
            {
                const double h = 1e-5;
                for (int j=0;j<numJoints_;++j) {
                    auto anglesPlus = nominal_.jointAngles, anglesMinus = nominal_.jointAngles;
                    anglesPlus[size_t(j)] += h; anglesMinus[size_t(j)] -= h;
                    const auto lp = fk_(anglesPlus), lm = fk_(anglesMinus);
                    std::vector<Vec3> d(numMarkersSz);
                    for (int m=0;m<numMarkers;++m)
                        d[size_t(m)] = { (lp[size_t(m)][0]-lm[size_t(m)][0])/(2*h),
                                        (lp[size_t(m)][1]-lm[size_t(m)][1])/(2*h),
                                        (lp[size_t(m)][2]-lm[size_t(m)][2])/(2*h) };
                    dLocalDJoint[size_t(j)] = d;
                }
            }

            Vec6dyn workingErr(size_t(n), 0.0);   // 误差状态，本轮从 0 开始
            Mat workingCov = cov_;
            const Vec6dyn linPoint(size_t(n), 0.0);   // 线性化点固定在"本轮开始时的0误差"

            for (const auto& meas : measurements) {
                if (!meas.cam || meas.markerIndex < 0 || meas.markerIndex >= numMarkers) continue;
                const Vec3& lp3 = localPos[size_t(meas.markerIndex)];
                const Vec3 v = matVec3Flat(nominal_.wristRot, lp3);   // 手腕原点->marker，世界系方向
                const Vec3 worldPos = { nominal_.wristPos[0]+v[0], nominal_.wristPos[1]+v[1], nominal_.wristPos[2]+v[2] };

                double nx, ny; double Hproj[2][6];
                if (!iekf_detail::projectAndJacobian(*meas.cam, worldPos, nx, ny, Hproj)) continue;
                // Hproj 是按 PointIEKF 的 6 维(位置+速度)状态算的，这里只用它
                // 对位置的 2x3 部分(前3列)，速度列在这里没意义。

                double skewV[3][3]; skew(v, skewV);

                // H_full: 2 x n，列布局 [wristPos(3) | rotPerturb(3) | joints(numJoints_)]
                std::vector<double> Hrow0(size_t(n),0.0), Hrow1(size_t(n),0.0);
                for (int k=0;k<3;++k) {
                    Hrow0[size_t(k)] = Hproj[0][k];           // d(nx)/d(wristPos_k) = Hproj行 * I
                    Hrow1[size_t(k)] = Hproj[1][k];
                }
                // d(worldPos)/d(rotPerturb) = -[v]_x ；再链式乘 Hproj(2x3)。
                for (int k=0;k<3;++k) {
                    double c0=0.0, c1=0.0;
                    for (int r=0;r<3;++r) { c0 += Hproj[0][r]*(-skewV[r][k]); c1 += Hproj[1][r]*(-skewV[r][k]); }
                    Hrow0[size_t(3+k)] = c0; Hrow1[size_t(3+k)] = c1;
                }
                // d(worldPos)/d(joint_j) = R_nominal * dLocal/dJoint_j(该marker那一份)。
                for (int j=0;j<numJoints_;++j) {
                    const Vec3 rotated = matVec3Flat(nominal_.wristRot, dLocalDJoint[size_t(j)][size_t(meas.markerIndex)]);
                    double c0=0.0, c1=0.0;
                    for (int r=0;r<3;++r) { const double comp = (r==0?rotated[0]:(r==1?rotated[1]:rotated[2])); c0 += Hproj[0][r]*comp; c1 += Hproj[1][r]*comp; }
                    Hrow0[size_t(6+j)] = c0; Hrow1[size_t(6+j)] = c1;
                }

                // 残差：固定线性化点(全0误差)算出的预测 h(nominal)，加上"本轮
                // 已经融合过的其它相机带来的误差修正"这个线性外推项——跟
                // PointIEKF 的同一个技巧，允许同一轮里依次序贯融合多个观测。
                double predShiftX=0.0, predShiftY=0.0;
                for (int k=0;k<n;++k) { predShiftX += Hrow0[size_t(k)]*(workingErr[size_t(k)]-linPoint[size_t(k)]);
                                        predShiftY += Hrow1[size_t(k)]*(workingErr[size_t(k)]-linPoint[size_t(k)]); }
                const double resX = meas.nx - nx - predShiftX;
                const double resY = meas.ny - ny - predShiftY;

                // S = H P H^T + R (2x2)
                std::vector<double> PHt0(size_t(n),0.0), PHt1(size_t(n),0.0);
                for (int r=0;r<n;++r) {
                    double s0=0.0, s1=0.0;
                    for (int k=0;k<n;++k) { s0 += workingCov[size_t(r)][size_t(k)]*Hrow0[size_t(k)]; s1 += workingCov[size_t(r)][size_t(k)]*Hrow1[size_t(k)]; }
                    PHt0[size_t(r)]=s0; PHt1[size_t(r)]=s1;
                }
                double S00=meas.sigma.xx, S01=meas.sigma.xy, S11=meas.sigma.yy;
                for (int k=0;k<n;++k) { S00 += Hrow0[size_t(k)]*PHt0[size_t(k)]; S01 += Hrow0[size_t(k)]*PHt1[size_t(k)]; S11 += Hrow1[size_t(k)]*PHt1[size_t(k)]; }

                double Sinv[2][2];
                if (!iekf_detail::invert2x2(S00,S01,S01,S11,Sinv)) continue;

                const size_t nn = size_t(n);
                std::vector<double> K0(nn), K1(nn);
                for (int r=0;r<n;++r) { K0[size_t(r)] = PHt0[size_t(r)]*Sinv[0][0]+PHt1[size_t(r)]*Sinv[1][0];
                                        K1[size_t(r)] = PHt0[size_t(r)]*Sinv[0][1]+PHt1[size_t(r)]*Sinv[1][1]; }

                for (int r=0;r<n;++r) workingErr[size_t(r)] += K0[size_t(r)]*resX + K1[size_t(r)]*resY;

                // P = (I-KH)P
                Mat newCov = zeros(n,n);
                Mat KH = zeros(n,n);
                for (int r=0;r<n;++r) for (int c=0;c<n;++c) KH[size_t(r)][size_t(c)] = K0[size_t(r)]*Hrow0[size_t(c)] + K1[size_t(r)]*Hrow1[size_t(c)];
                for (int r=0;r<n;++r) for (int c=0;c<n;++c) {
                    double s = workingCov[size_t(r)][size_t(c)];
                    for (int k=0;k<n;++k) s -= KH[size_t(r)][size_t(k)]*workingCov[size_t(k)][size_t(c)];
                    newCov[size_t(r)][size_t(c)] = s;
                }
                workingCov = newCov;
            }

            // 把这一轮的误差状态折算回名义状态。
            const Vec3 dPos = {workingErr[0], workingErr[1], workingErr[2]};
            const Vec3 dTheta = {workingErr[3], workingErr[4], workingErr[5]};
            double Rdelta[3][3]; rodrigues(dTheta, Rdelta);
            nominal_.wristRot = matMul9(Rdelta, nominal_.wristRot);
            nominal_.wristPos = { nominal_.wristPos[0]+dPos[0], nominal_.wristPos[1]+dPos[1], nominal_.wristPos[2]+dPos[2] };
            for (int j=0;j<numJoints_;++j) nominal_.jointAngles[size_t(j)] += workingErr[size_t(6+j)];

            cov_ = workingCov;

            double deltaNorm=0.0; for (int k=0;k<n;++k) deltaNorm += workingErr[size_t(k)]*workingErr[size_t(k)];
            if (std::sqrt(deltaNorm) < convergeEps) break;
        }
    }

    const HandPoseState& state() const { return nominal_; }
    const std::vector<std::vector<double>>& covariance() const { return cov_; }

    // 【这一版新增，修的是真实缺口】关节角限位夹紧——直接修改内部名义
    // 状态(不是只给调用方一份夹紧过的输出拷贝)。之前的做法是"送出去的
    // 那份合法，但滤波器自己心里那份信念可能还在越界"，下一帧继续拿这个
    // 越界的内部状态做预测/关联，等于在拿一个不干净的东西继续滚雪球。
    // 这个类本身不知道具体的关节限位数值是多少(那是 hand/HandModel.hpp
    // 的职责，不应该在状态估计层里耦合)，调用方传入一个"就地修改
    // jointAngles"的函数(通常是 clampToLimits 包一层)。
    //
    // 【诚实的局限】这是"只夹均值、不动协方差"的朴素裁剪(naive clipping)，
    // 不是严格意义上的约束卡尔曼滤波(约束方向上的协方差没有跟着收窄)——
    // 比什么都不做好得多，但不是数学上最优的处理。真正严格的约束EKF需要
    // 在约束流形上做投影或者加伪观测，是更大的工作量，这里先用性价比更
    // 高的这一版。
    void clampJointAngles(const std::function<void(std::vector<double>&)>& clampFn) {
        clampFn(nominal_.jointAngles);
    }

private:
    using Vec6dyn = std::vector<double>;
    ForwardKinematicsFn fk_;
    int numJoints_;
    HandPoseState nominal_;
    std::vector<std::vector<double>> cov_;
};

} // namespace mocap
