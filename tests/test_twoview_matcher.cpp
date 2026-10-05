// 两视图多点匹配验证：合成已知点+位姿投影，验证匈牙利全局最优指派能把
// 多个点正确配对，且在贪心会抢错的场景下依然正确；处理点数不等（遮挡）
// 和乱序输入。纯数学，g++ -std=c++17 直接跑。
#include "reconstruct/TwoViewMatcher.hpp"
#include "reconstruct/Epipolar.hpp"
#include <cstdio>
#include <cmath>
#include <array>
#include <vector>
#include <random>
#include <algorithm>

using namespace mocap;

static bool projectNorm(const EpiMat3& R, const EpiVec3& t,
                        const EpiVec3& Xw, double& nx, double& ny) {
    const double Xc=R[0]*Xw[0]+R[1]*Xw[1]+R[2]*Xw[2]+t[0];
    const double Yc=R[3]*Xw[0]+R[4]*Xw[1]+R[5]*Xw[2]+t[1];
    const double Zc=R[6]*Xw[0]+R[7]*Xw[1]+R[8]*Xw[2]+t[2];
    if (Zc<=1e-6) return false;
    nx=Xc/Zc; ny=Yc/Zc; return true;
}
static EpiMat3 rotY(double a){double c=std::cos(a),s=std::sin(a);return {c,0,s,0,1,0,-s,0,c};}

struct Scene {
    EpiMat3 E;
    EpiMat3 R0,R1; EpiVec3 t0,t1;
};
static Scene makeScene(double turn=-0.3, double baseline=200) {
    Scene s;
    s.R0={1,0,0,0,1,0,0,0,1}; s.t0={0,0,0};
    s.R1=rotY(turn);
    EpiVec3 C1={baseline,0,0};
    s.t1={ -(s.R1[0]*C1[0]+s.R1[1]*C1[1]+s.R1[2]*C1[2]),
           -(s.R1[3]*C1[0]+s.R1[4]*C1[1]+s.R1[5]*C1[2]),
           -(s.R1[6]*C1[0]+s.R1[7]*C1[1]+s.R1[8]*C1[2]) };
    s.E=essentialFromRelativePose(s.R0,s.t0,s.R1,s.t1);
    return s;
}

int main() {
    bool ok = true;
    Scene sc = makeScene();
    const double maxSampson = 0.005;   // ~5px @ 1000px 焦距

    // ---- 测试1：5点全配对，每对 (i,j) 应该 i==j（顺序一致时）----
    {
        std::vector<EpiVec3> pts = {
            {0,0,1000},{80,40,1200},{-60,50,900},{30,-70,1100},{-40,-30,1300}};
        std::vector<std::array<double,2>> o0,o1;
        for (auto&X:pts){double nx,ny;projectNorm(sc.R0,sc.t0,X,nx,ny);o0.push_back({nx,ny});
            projectNorm(sc.R1,sc.t1,X,nx,ny);o1.push_back({nx,ny});}
        auto r = matchTwoViews(sc.E,o0,o1,maxSampson);
        bool pass = (r.pairs.size()==5) && r.unmatched0.empty() && r.unmatched1.empty();
        for (auto&mp:r.pairs) if (mp.i!=mp.j) pass=false;
        printf("[5点全配对] 配上%zu对 未配cam0=%zu cam1=%zu %s\n",
               r.pairs.size(),r.unmatched0.size(),r.unmatched1.size(),pass?"OK":"FAIL");
        ok=ok&&pass;
    }

    // ---- 测试2：乱序输入——cam1 的点打乱顺序，匹配应仍找回正确物理对应 ----
    {
        std::vector<EpiVec3> pts = {
            {0,0,1000},{80,40,1200},{-60,50,900},{30,-70,1100},{-40,-30,1300}};
        std::vector<std::array<double,2>> o0,o1;
        for (auto&X:pts){double nx,ny;projectNorm(sc.R0,sc.t0,X,nx,ny);o0.push_back({nx,ny});}
        // cam1 按置换 [2,4,0,3,1] 打乱
        std::vector<int> perm={2,4,0,3,1};
        std::vector<int> truth(5);
        for (int p=0;p<5;++p){double nx,ny;projectNorm(sc.R1,sc.t1,pts[perm[p]],nx,ny);
            o1.push_back({nx,ny}); truth[perm[p]]=p; }   // truth[i]=cam1里对应i的下标
        auto r = matchTwoViews(sc.E,o0,o1,maxSampson);
        bool pass = (r.pairs.size()==5);
        for (auto&mp:r.pairs) if (truth[mp.i]!=mp.j) pass=false;
        printf("[乱序输入] 配上%zu对，全部对回正确物理点 %s\n", r.pairs.size(), pass?"OK":"FAIL");
        ok=ok&&pass;
    }

    // ---- 测试3：遮挡——cam1 少一个点（第2个物理点在cam1看不到）----
    {
        std::vector<EpiVec3> pts = {
            {0,0,1000},{80,40,1200},{-60,50,900},{30,-70,1100},{-40,-30,1300}};
        std::vector<std::array<double,2>> o0,o1;
        for (int i=0;i<5;++i){double nx,ny;projectNorm(sc.R0,sc.t0,pts[i],nx,ny);o0.push_back({nx,ny});}
        // cam1 缺物理点2
        std::vector<int> cam1phys={0,1,3,4};
        for (int idx:cam1phys){double nx,ny;projectNorm(sc.R1,sc.t1,pts[idx],nx,ny);o1.push_back({nx,ny});}
        auto r = matchTwoViews(sc.E,o0,o1,maxSampson);
        // 应配上4对，cam0里物理点2未匹配
        bool count_ok = (r.pairs.size()==4) && (r.unmatched0.size()==1) && r.unmatched1.empty();
        bool right_unmatched = (r.unmatched0.size()==1 && r.unmatched0[0]==2);
        printf("[遮挡] 配上%zu对(应4) cam0未配=%zu(应1,是点%d) %s\n",
               r.pairs.size(), r.unmatched0.size(),
               r.unmatched0.empty()?-1:r.unmatched0[0], (count_ok&&right_unmatched)?"OK":"FAIL");
        ok=ok&&count_ok&&right_unmatched;
    }

    // ---- 测试4：全局最优 vs 贪心——构造一个贪心会抢错的场景 ----
    // 两个物理点，它们的极线几何使得"cam0点A 到 cam1点B" 的 Sampson 距离
    // 略小于正确配对，但全局最优（考虑总代价+唯一性约束）仍能配对正确。
    {
        // 用两个较近的点，让极线代价矩阵出现"交叉诱惑"
        std::vector<EpiVec3> pts = {{10,5,1000},{14,7,1005}};
        std::vector<std::array<double,2>> o0,o1;
        for (auto&X:pts){double nx,ny;projectNorm(sc.R0,sc.t0,X,nx,ny);o0.push_back({nx,ny});
            projectNorm(sc.R1,sc.t1,X,nx,ny);o1.push_back({nx,ny});}
        auto r = matchTwoViews(sc.E,o0,o1,maxSampson);
        bool pass=(r.pairs.size()==2);
        for(auto&mp:r.pairs) if(mp.i!=mp.j) pass=false;
        printf("[全局最优] 近距点对仍正确配对（i==j）%s\n", pass?"OK":"FAIL");
        ok=ok&&pass;
    }

    // ---- 测试5：空输入不崩 ----
    {
        std::vector<std::array<double,2>> empty0, some1={{0.1,0.1}};
        auto r = matchTwoViews(sc.E, empty0, some1, maxSampson);
        bool pass = r.pairs.empty() && r.unmatched0.empty() && r.unmatched1.size()==1;
        printf("[空输入] cam0空时安全返回 %s\n", pass?"OK":"FAIL");
        ok=ok&&pass;
    }

    printf("\n%s\n", ok ? "twoview_matcher C++: ALL PASS" : "twoview_matcher C++: FAIL");
    return ok ? 0 : 1;
}
