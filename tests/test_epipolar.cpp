// 极线几何验证：合成已知 3D 点 + 已知相机位姿，正向投影成两组归一化观测，
// 验证极线约束的核心性质——对应点对 Sampson 距离≈0，非对应点对显著>0。
// 纯数学，g++ -std=c++17 直接跑，零 Qt 依赖。
#include "reconstruct/Epipolar.hpp"
#include <cstdio>
#include <cmath>
#include <array>
#include <vector>

using namespace mocap;

// 把世界点用 (R,t) 投到某相机的归一化坐标 (nx,ny)=(Xc/Zc, Yc/Zc)。
// 返回 false 表示点在相机后方（Zc<=0），这种点不该参与。
static bool projectNorm(const EpiMat3& R, const EpiVec3& t,
                        const EpiVec3& Xw, double& nx, double& ny) {
    const double Xc = R[0]*Xw[0]+R[1]*Xw[1]+R[2]*Xw[2]+t[0];
    const double Yc = R[3]*Xw[0]+R[4]*Xw[1]+R[5]*Xw[2]+t[1];
    const double Zc = R[6]*Xw[0]+R[7]*Xw[1]+R[8]*Xw[2]+t[2];
    if (Zc <= 1e-6) return false;
    nx = Xc / Zc; ny = Yc / Zc;
    return true;
}

// 绕 Y 轴转 a 弧度的旋转矩阵（行主序）
static EpiMat3 rotY(double a) {
    double c=std::cos(a), s=std::sin(a);
    return { c,0,s, 0,1,0, -s,0,c };
}

int main() {
    bool ok = true;
    const EpiMat3 I = {1,0,0, 0,1,0, 0,0,1};

    // ---- 场景：两台相机看同一组 3D 点 ----
    // 相机0：在原点，看向 +Z（世界系与相机0系重合）
    const EpiMat3 R0 = I;
    const EpiVec3 t0 = {0,0,0};
    // 相机1：向右平移 200mm 并稍微转向内侧（一般位姿，不是纯平移）
    const EpiMat3 R1 = rotY(-0.3);
    // 世界->相机1 的 t1：让相机1 的世界位置在 (200,0,0)。C=-R^T t => t=-R C
    const EpiVec3 C1 = {200,0,0};
    const EpiVec3 t1 = { -(R1[0]*C1[0]+R1[1]*C1[1]+R1[2]*C1[2]),
                         -(R1[3]*C1[0]+R1[4]*C1[1]+R1[5]*C1[2]),
                         -(R1[6]*C1[0]+R1[7]*C1[1]+R1[8]*C1[2]) };

    const EpiMat3 E = essentialFromRelativePose(R0, t0, R1, t1);

    // 5 个已知 3D 点，都在两相机前方
    std::vector<EpiVec3> pts = {
        {  0,   0, 1000},
        { 80,  40, 1200},
        {-60,  50,  900},
        { 30, -70, 1100},
        {-40, -30, 1300},
    };

    // 投影到两台相机的归一化坐标
    std::vector<std::array<double,2>> obs0, obs1;
    for (auto& X : pts) {
        double nx,ny; 
        bool v0 = projectNorm(R0,t0,X,nx,ny); std::array<double,2> a0{nx,ny};
        bool v1 = projectNorm(R1,t1,X,nx,ny); std::array<double,2> a1{nx,ny};
        if (!v0 || !v1) { printf("FAIL: 合成点在相机后方，测试构造有误\n"); return 1; }
        obs0.push_back(a0); obs1.push_back(a1);
    }

    // ---- 测试1：对应点对的 Sampson 距离应≈0 ----
    {
        double maxD = 0;
        for (size_t i=0;i<pts.size();++i) {
            double d = sampsonDistance(E, obs0[i][0],obs0[i][1], obs1[i][0],obs1[i][1]);
            if (d>maxD) maxD=d;
        }
        bool pass = maxD < 1e-9;
        printf("[对应点对] 最大 Sampson 距离=%.2e (应≈0) %s\n", maxD, pass?"OK":"FAIL");
        ok = ok && pass;
    }

    // ---- 测试2：非对应点对的 Sampson 距离应显著>0 ----
    {
        double minD = 1e18;
        for (size_t i=0;i<pts.size();++i)
            for (size_t j=0;j<pts.size();++j) {
                if (i==j) continue;
                double d = sampsonDistance(E, obs0[i][0],obs0[i][1], obs1[j][0],obs1[j][1]);
                if (d<minD) minD=d;
            }
        // 非对应点对最小距离应远大于对应点对（数量级差异）。
        bool pass = minD > 1e-4;
        printf("[非对应点对] 最小 Sampson 距离=%.4f (应显著>0) %s\n", minD, pass?"OK":"FAIL");
        ok = ok && pass;
    }

    // ---- 测试3：剪枝用例——给 cam0 的一个点，在 cam1 的候选里，
    //      正确对应的那个 Sampson 距离必须是最小的 ----
    {
        bool all_correct = true;
        for (size_t i=0;i<pts.size();++i) {
            double bestD=1e18; int bestJ=-1;
            for (size_t j=0;j<pts.size();++j) {
                double d = sampsonDistance(E, obs0[i][0],obs0[i][1], obs1[j][0],obs1[j][1]);
                if (d<bestD){bestD=d;bestJ=int(j);}
            }
            if (bestJ != int(i)) { all_correct=false;
                printf("  cam0点%zu 极线最近的是 cam1点%d（应为%zu）\n", i, bestJ, i); }
        }
        printf("[剪枝] 每个 cam0 点的极线最近邻都是正确对应 %s\n", all_correct?"OK":"FAIL");
        ok = ok && all_correct;
    }

    // ---- 测试4：纯平移退化情形也要成立（相机1只右移，不转）----
    {
        const EpiMat3 R1p = I;
        const EpiVec3 t1p = {-200,0,0};   // 世界->相机1，纯平移
        const EpiMat3 Ep = essentialFromRelativePose(R0,t0,R1p,t1p);
        double maxD=0;
        for (auto& X : pts) {
            double nx,ny; projectNorm(R0,t0,X,nx,ny); double a0x=nx,a0y=ny;
            projectNorm(R1p,t1p,X,nx,ny);
            double d = sampsonDistance(Ep, a0x,a0y, nx,ny);
            if (d>maxD)maxD=d;
        }
        bool pass = maxD < 1e-9;
        printf("[纯平移] 对应点对最大 Sampson 距离=%.2e (应≈0) %s\n", maxD, pass?"OK":"FAIL");
        ok = ok && pass;
    }

    printf("\n%s\n", ok ? "epipolar C++: ALL PASS" : "epipolar C++: FAIL");
    return ok ? 0 : 1;
}
