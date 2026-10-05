// TriangulationRefine.hpp 单元测试。
//
// 覆盖范围：
//   1. 干净多视图数据(无噪声)：精修后的点应该精确收敛到真值，所有权重
//      接近1.0(没有什么可以鲁棒化的，权重不应该被无谓压低)。
//   2. 核心价值——混入一个坏视角(比如某台相机检测严重跑偏)：精修后的
//      点应该明显更接近真值(相比不做IRLS、直接平均对待所有视角)，坏
//      视角自己的权重应该被显著压低。
//   3. 收敛性：给一个偏离真值较远的初值(模拟"DLT解本身也不太准"的情况)，
//      LM应该还是能收敛到接近真值。
//   4. 观测数不足(只有1个)：诚实返回原始初值，不假装精修过。
//   5. 加权RMS残差应该能正确反映"排除坏视角权重之后的真实拟合优度"，
//      不会被坏视角的巨大原始残差拉高。
#include "reconstruct/TriangulationRefine.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static EpiMat3 lookAtR(const EpiVec3& camPos, const EpiVec3& target, EpiVec3& outT) {
    EpiVec3 fwd = { target[0]-camPos[0], target[1]-camPos[1], target[2]-camPos[2] };
    double n = std::sqrt(fwd[0]*fwd[0]+fwd[1]*fwd[1]+fwd[2]*fwd[2]);
    fwd = {fwd[0]/n, fwd[1]/n, fwd[2]/n};
    EpiVec3 up = {0,0,1};
    if (std::abs(fwd[0]*up[0]+fwd[1]*up[1]+fwd[2]*up[2]) > 0.99) up = {0,1,0};
    EpiVec3 right = { fwd[1]*up[2]-fwd[2]*up[1], fwd[2]*up[0]-fwd[0]*up[2], fwd[0]*up[1]-fwd[1]*up[0] };
    double rn = std::sqrt(right[0]*right[0]+right[1]*right[1]+right[2]*right[2]);
    right = {right[0]/rn, right[1]/rn, right[2]/rn};
    EpiVec3 camUp = { right[1]*fwd[2]-right[2]*fwd[1], right[2]*fwd[0]-right[0]*fwd[2], right[0]*fwd[1]-right[1]*fwd[0] };
    EpiMat3 R = { right[0],right[1],right[2], -camUp[0],-camUp[1],-camUp[2], fwd[0],fwd[1],fwd[2] };
    for (int r=0;r<3;++r) { double s=0; for (int k=0;k<3;++k) s += R[size_t(r*3+k)]*camPos[size_t(k)]; outT[size_t(r)] = -s; }
    return R;
}

static bool project(const EpiMat3& R, const EpiVec3& t, const EpiVec3& Xw, double& nx, double& ny) {
    const double Xc=R[0]*Xw[0]+R[1]*Xw[1]+R[2]*Xw[2]+t[0];
    const double Yc=R[3]*Xw[0]+R[4]*Xw[1]+R[5]*Xw[2]+t[1];
    const double Zc=R[6]*Xw[0]+R[7]*Xw[1]+R[8]*Xw[2]+t[2];
    if (Zc<=1e-6) return false;
    nx=Xc/Zc; ny=Yc/Zc; return true;
}

int main() {
    const EpiVec3 truePoint = {30.0, -15.0, 480.0};

    // 4台相机围绕真值点摆放，视角夹开一些。
    std::vector<EpiVec3> camPositions = { {0,0,0}, {300,0,50}, {-200,150,-30}, {100,-250,80} };
    std::vector<TriangulationObservation> cleanObs;
    for (auto& cp : camPositions) {
        EpiVec3 t; EpiMat3 R = lookAtR(cp, truePoint, t);
        double nx,ny; project(R,t,truePoint,nx,ny);
        cleanObs.push_back({R,t,nx,ny});
    }

    // ---- 场景 1：无噪声，应该精确收敛到真值，权重全部接近1。----
    {
        // 从一个略偏的初值出发(不是直接给真值，测试LM真的在优化，不是原样返回)。
        const EpiVec3 initGuess = {truePoint[0]+5, truePoint[1]-3, truePoint[2]+8};
        const auto result = refineTriangulationLM(initGuess, cleanObs);
        const double err = std::sqrt(
            (result.point[0]-truePoint[0])*(result.point[0]-truePoint[0]) +
            (result.point[1]-truePoint[1])*(result.point[1]-truePoint[1]) +
            (result.point[2]-truePoint[2])*(result.point[2]-truePoint[2]));
        char msg[200];
        std::snprintf(msg, sizeof(msg), "clean data: converges to true point within 0.01mm (got error=%.6fmm)", err);
        CHECK(err < 0.01, msg);
        bool allWeightsHigh = true;
        for (double w : result.viewWeights) if (w < 0.99) allWeightsHigh = false;
        CHECK(allWeightsHigh, "clean data: all view weights stay near 1.0 (nothing to robustify against)");
    }

    // ---- 场景 2(核心)：混入一个坏视角，精修点应该明显更接近真值，
    // 坏视角自己的权重应该被显著压低。----
    {
        auto obsWithOutlier = cleanObs;
        // 第3个视角(下标2)的检测严重跑偏——模拟遮挡/误检导致的坏观测。
        obsWithOutlier[2].nx += 0.15;
        obsWithOutlier[2].ny -= 0.10;

        const auto result = refineTriangulationLM(truePoint, obsWithOutlier);
        const double err = std::sqrt(
            (result.point[0]-truePoint[0])*(result.point[0]-truePoint[0]) +
            (result.point[1]-truePoint[1])*(result.point[1]-truePoint[1]) +
            (result.point[2]-truePoint[2])*(result.point[2]-truePoint[2]));
        char msg[220];
        std::snprintf(msg, sizeof(msg), "outlier view: refined point stays close to truth despite one badly corrupted view (error=%.3fmm)", err);
        CHECK(err < 5.0, msg);   // 3个好视角仍能约束住，误差应该远小于坏视角本身造成的偏移量级

        std::snprintf(msg, sizeof(msg), "outlier view: the corrupted view's own weight is significantly downweighted (got %.4f)", result.viewWeights[2]);
        CHECK(result.viewWeights[2] < 0.3, msg);

        bool othersStayHigh = true;
        for (size_t k=0;k<result.viewWeights.size();++k) if (k!=2 && result.viewWeights[k] < 0.9) othersStayHigh=false;
        CHECK(othersStayHigh, "outlier view: the 3 good views are not penalized, only the actual bad one is");
    }

    // ---- 场景 3：初值偏离真值较远，LM应该还是能收敛回真值附近
    // (模拟"DLT本身给的初值也不太准"的情况)。----
    {
        const EpiVec3 farInit = {truePoint[0]+40, truePoint[1]-35, truePoint[2]+60};
        const auto result = refineTriangulationLM(farInit, cleanObs);
        const double err = std::sqrt(
            (result.point[0]-truePoint[0])*(result.point[0]-truePoint[0]) +
            (result.point[1]-truePoint[1])*(result.point[1]-truePoint[1]) +
            (result.point[2]-truePoint[2])*(result.point[2]-truePoint[2]));
        char msg[200];
        std::snprintf(msg, sizeof(msg), "far initial guess: still converges close to truth (error=%.4fmm)", err);
        CHECK(err < 0.1, msg);
        CHECK(result.converged, "far initial guess: reports converged=true");
    }

    // ---- 场景 4：观测数不足(只有1个)，诚实返回原始初值，不崩溃、不假装精修过。----
    {
        std::vector<TriangulationObservation> single = { cleanObs[0] };
        const EpiVec3 someInit = {1,2,3};
        const auto result = refineTriangulationLM(someInit, single);
        CHECK(result.point[0]==1 && result.point[1]==2 && result.point[2]==3,
              "insufficient observations (n<2): honestly returns the original initial guess unchanged");
    }

    // ---- 场景 5：加权RMS应该反映"排除坏视角权重之后"的真实拟合优度，
    // 不会被坏视角的巨大原始残差拉得很高。----
    {
        auto obsWithOutlier = cleanObs;
        obsWithOutlier[2].nx += 0.15;
        obsWithOutlier[2].ny -= 0.10;
        const auto result = refineTriangulationLM(truePoint, obsWithOutlier);
        char msg[200];
        std::snprintf(msg, sizeof(msg), "weighted RMS stays small (not dominated by the outlier's huge raw residual) despite one badly corrupted view (got %.5f)", result.weightedRmsNorm);
        // 注意：Huber权重是渐近趋近0，不是精确归零，加权RMS必然还残留
        // 坏视角一点点贡献——阈值不该指望降到接近0，只要求"远小于坏视角
        // 原始残差量级(~0.18)"，证明确实被压制住了，不是没压制。
        CHECK(result.weightedRmsNorm >= 0.0 && result.weightedRmsNorm < 0.05, msg);
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
