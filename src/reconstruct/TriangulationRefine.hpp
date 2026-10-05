#pragma once
// ---------------------------------------------------------------------------
// 三角化精修：LM迭代 + IRLS鲁棒估计 —— 纯数学，零 Qt/第三方依赖，可单测。
//
// 解决的问题：MultiViewCluster.hpp::triangulateNView 用的是 DLT(4x4矩阵
// 最小特征向量)，这是一个代数最优解——最小化的是一个跟真实重投影误差
// 不完全等价的代数残差(每个观测贡献一对线性方程，DLT解的是这些线性方程
// 组成的齐次系统的最小二乘，不是直接最小化 Σ|投影(X)-观测|²这个真正
// 关心的量)。噪声小、视角夹角好的时候两者几乎没差别；但畸变残差较大、
// 观测噪声较大、或者视角夹角比较刁钻的时候，代数解跟真正的重投影误差
// 最优解能差出几mm——这正是 README §5 提到的"跟OptiTrack这类专业系统
// 的差距主要在这一层"的具体所指。
//
// 这里在 DLT 结果基础上，用 Levenberg-Marquardt 对 3D 点坐标(3个未知数)
// 做迭代精修，直接最小化真实的(非线性)重投影误差——DLT 解本来就是很好
// 的初值，LM 通常 5~10 次迭代内收敛，计算量很小，压力不在这一步。
//
// 同时做 IRLS(迭代重加权最小二乘)：每一轮 LM 迭代之后，用当前点算出
// 每个观测的重投影残差，超过 Huber 阈值的观测自动降权(残差越大权重越
// 低)，下一轮 LM 用降权后的残差重新拟合——这是"软剔除"坏视角的标准
// 做法(比如某台相机因为遮挡/检测噪声，这一帧的观测明显跟其它相机对不
// 上)，不需要像 RANSAC 那样穷举子集，权重连续地把可疑观测的影响压低，
// 而不是非黑即白地整个丢弃。
//
// 跟 MultiViewCluster.hpp 的关系：那边的 triangulateNView(DLT) 负责"快速
// 给一个初始解"，这个模块负责"如果你在乎精度，在DLT解基础上再精修一步"——
// 两者不冲突，这个模块的输入就是把 DLT 解当初值传进来，输出精修后的点 +
// 每个观测最终的权重(供调用方判断"这一帧是不是有一台相机数据明显有问题"，
// 权重很低的视角可以从 track 里剔除或者标记为不可信)。
// ---------------------------------------------------------------------------
#include "reconstruct/Epipolar.hpp"
#include <vector>
#include <cmath>

namespace mocap {

struct TriangulationObservation {
    EpiMat3 R{};
    EpiVec3 t{};
    double nx = 0.0, ny = 0.0;
};

struct TriangulationRefineConfig {
    int maxIters = 15;
    // Huber阈值(归一化坐标单位)——重投影残差幅度超过这个值开始被降权。
    // 典型检测/三角化噪声在零点几到几个千分之一(归一化坐标)量级，默认
    // 给0.01是留了几倍余量的保守值，明显的坏视角(残差是好视角的好几倍)
    // 才会被显著降权，不会误伤正常噪声范围内的观测。
    double huberDelta = 0.01;
    double convergeEps = 1e-9;   // 参数更新量(归一化坐标下)小于这个就认为收敛
};

struct TriangulationRefineResult {
    EpiVec3 point{};
    double weightedRmsNorm = -1.0;      // 加权后的RMS重投影残差(归一化坐标)，供诊断
    std::vector<double> viewWeights;    // 跟输入obs一一对应，1.0=完全信任，趋近0=基本当噪声处理
    bool converged = false;
    int iterationsUsed = 0;
};

namespace triref_detail {

inline bool reprojectAndJac(const EpiMat3& R, const EpiVec3& t, const EpiVec3& Xw,
                            double& nx, double& ny, double J[2][3]) {
    const double Xc = R[0]*Xw[0]+R[1]*Xw[1]+R[2]*Xw[2]+t[0];
    const double Yc = R[3]*Xw[0]+R[4]*Xw[1]+R[5]*Xw[2]+t[1];
    const double Zc = R[6]*Xw[0]+R[7]*Xw[1]+R[8]*Xw[2]+t[2];
    if (Zc <= 1e-6) return false;
    nx = Xc/Zc; ny = Yc/Zc;
    const double invZ = 1.0/Zc, invZ2 = invZ*invZ;
    for (int k=0;k<3;++k) {
        J[0][k] = (R[size_t(k)]*Zc - Xc*R[size_t(6+k)]) * invZ2;
        J[1][k] = (R[size_t(3+k)]*Zc - Yc*R[size_t(6+k)]) * invZ2;
    }
    return true;
}

// Huber权重：残差幅度(|resX|,|resY|合成的欧氏范数)超过delta就按
// delta/残差 降权，否则权重恒为1——标准Huber M估计的权重形式。
inline double huberWeight(double residualNorm, double delta) {
    if (residualNorm <= delta || residualNorm < 1e-12) return 1.0;
    return delta / residualNorm;
}

inline bool solve3x3(const double A[3][3], const double b[3], double x[3]) {
    double M[3][4];
    for (int i=0;i<3;++i) { for (int j=0;j<3;++j) M[i][j]=A[i][j]; M[i][3]=b[i]; }
    for (int col=0; col<3; ++col) {
        int piv = col;
        for (int r=col+1; r<3; ++r) if (std::abs(M[r][col]) > std::abs(M[piv][col])) piv = r;
        if (std::abs(M[piv][col]) < 1e-14) return false;
        if (piv != col) for (int j=0;j<4;++j) std::swap(M[col][j], M[piv][j]);
        for (int r=0;r<3;++r) {
            if (r==col) continue;
            const double f = M[r][col]/M[col][col];
            for (int j=0;j<4;++j) M[r][j] -= f*M[col][j];
        }
    }
    for (int i=0;i<3;++i) x[i] = M[i][3]/M[i][i];
    return true;
}

} // namespace triref_detail

// initial: 通常是 triangulateNView(DLT) 的结果，作为LM的起始点。
// obs: 至少2个视角的观测(跟DLT输入应该是同一批，或者其超集/子集——调用方
// 决定用哪些视角参与精修)。
inline TriangulationRefineResult refineTriangulationLM(
        const EpiVec3& initial, const std::vector<TriangulationObservation>& obs,
        const TriangulationRefineConfig& cfg = TriangulationRefineConfig{}) {
    using namespace triref_detail;
    TriangulationRefineResult out;
    out.point = initial;
    const size_t n = obs.size();
    out.viewWeights.assign(n, 1.0);
    if (n < 2) return out;   // 少于2个观测数学上不可解，原样返回初值，不假装精修过

    EpiVec3 X = initial;
    double lambda = 1e-3;

    auto computeCostJTJ = [&](const EpiVec3& pt, std::vector<double>& weights,
                              double JTJ[3][3], double JTr[3], double& cost, bool updateWeights) -> void {
        for (int i=0;i<3;++i) { JTr[i]=0.0; for (int j=0;j<3;++j) JTJ[i][j]=0.0; }
        cost = 0.0;
        for (size_t k=0;k<n;++k) {
            double px,py; double J[2][3];
            if (!reprojectAndJac(obs[k].R, obs[k].t, pt, px, py, J)) { weights[k]=0.0; continue; }
            const double rx = px - obs[k].nx, ry = py - obs[k].ny;
            const double rnorm = std::sqrt(rx*rx+ry*ry);
            if (updateWeights) weights[k] = huberWeight(rnorm, cfg.huberDelta);
            const double w = weights[k];

            for (int a=0;a<3;++a) {
                for (int b=0;b<3;++b) JTJ[a][b] += w*(J[0][a]*J[0][b] + J[1][a]*J[1][b]);
                JTr[a] += w*(J[0][a]*rx + J[1][a]*ry);
            }
            cost += w*(rx*rx+ry*ry);
        }
    };

    double JTJ[3][3], JTr[3], curCost;
    std::vector<double> weights = out.viewWeights;
    computeCostJTJ(X, weights, JTJ, JTr, curCost, /*updateWeights=*/true);

    int iter = 0;
    for (; iter<cfg.maxIters; ++iter) {
        double A[3][3];
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) A[i][j] = JTJ[i][j] + (i==j? lambda*JTJ[i][j] : 0.0);
        double negJTr[3] = { -JTr[0], -JTr[1], -JTr[2] };
        double delta[3];
        if (!solve3x3(A, negJTr, delta)) break;

        const EpiVec3 Xnew = { X[0]+delta[0], X[1]+delta[1], X[2]+delta[2] };
        std::vector<double> weightsNew = weights;
        double newJTJ[3][3], newJTr[3], newCost;
        computeCostJTJ(Xnew, weightsNew, newJTJ, newJTr, newCost, /*updateWeights=*/true);

        if (newCost < curCost) {
            X = Xnew; curCost = newCost; weights = weightsNew;
            for (int i=0;i<3;++i) { JTr[i]=newJTr[i]; for (int j=0;j<3;++j) JTJ[i][j]=newJTJ[i][j]; }
            lambda = std::max(lambda*0.5, 1e-12);
            const double deltaNorm = std::sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);
            if (deltaNorm < cfg.convergeEps) { out.converged = true; ++iter; break; }
        } else {
            lambda *= 2.0;
            if (lambda > 1e12) break;
        }
    }

    out.point = X;
    out.viewWeights = weights;
    out.iterationsUsed = iter;

    double sumW=0.0, sumWSq=0.0;
    for (size_t k=0;k<n;++k) {
        double px,py; double J[2][3];
        if (!reprojectAndJac(obs[k].R, obs[k].t, X, px, py, J)) continue;
        const double rx = px-obs[k].nx, ry = py-obs[k].ny;
        sumWSq += weights[k]*(rx*rx+ry*ry);
        sumW += weights[k];
    }
    out.weightedRmsNorm = sumW > 1e-12 ? std::sqrt(sumWSq/sumW) : -1.0;
    return out;
}

} // namespace mocap
