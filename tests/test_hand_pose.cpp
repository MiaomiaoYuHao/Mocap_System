// C++ 手背位姿求解验证。重点复核自带 3x3 SVD/Kabsch 的正确性（这是移植里
// 唯一新写的数学，Python 用的是 numpy.linalg.svd，C++ 是手写雅可比，必须
// 独立验证等价）。g++ -std=c++17 直接跑。
#include "hand/HandPose.hpp"
#include "hand/HandModel.hpp"
#include <cstdio>
#include <cmath>
#include <random>
#include <array>

using namespace mocap;

static double rotErrDeg(const HandMat3& A, const HandMat3& B) {
    // trace(A B^T)
    HandMat3 Bt = detail::transpose3(B);
    HandMat3 AB = detail::matmul3(A, Bt);
    double tr = AB[0]+AB[4]+AB[8];
    double c = (tr-1)/2; if(c>1)c=1; if(c<-1)c=-1;
    return std::acos(c)*180.0/M_PI;
}
static HandMat3 randRot(std::mt19937& g) {
    std::normal_distribution<double> nd(0,1);
    std::uniform_real_distribution<double> ud(0,2*M_PI);
    double v[3]={nd(g),nd(g),nd(g)};
    double n=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    v[0]/=n;v[1]/=n;v[2]/=n;
    double a=ud(g), c=std::cos(a), s=std::sin(a), C=1-c;
    return {
        c+v[0]*v[0]*C,      v[0]*v[1]*C-v[2]*s, v[0]*v[2]*C+v[1]*s,
        v[1]*v[0]*C+v[2]*s, c+v[1]*v[1]*C,      v[1]*v[2]*C-v[0]*s,
        v[2]*v[0]*C-v[1]*s, v[2]*v[1]*C+v[0]*s, c+v[2]*v[2]*C
    };
}

int main() {
    bool ok = true;
    std::mt19937 g(12345);
    std::normal_distribution<double> nd(0,1);

    std::array<HandVec3,5> tpl;
    for (int i=0;i<5;++i) tpl[i] = handBackMarkers()[i];
    std::array<bool,5> allVis = {true,true,true,true,true};

    // ---- 测试1：无噪声精确还原（验证自带SVD/Kabsch正确）----
    {
        HandMat3 Rt = randRot(g);
        HandVec3 tt = {100,-50,300};
        std::array<HandVec3,5> obs;
        for (int i=0;i<5;++i) obs[i]=transformPoint(tpl[i],Rt,tt);
        auto r = solveHandBackPose(tpl, obs, allVis);
        double re=rotErrDeg(r.R,Rt);
        double te=std::sqrt((r.t[0]-tt[0])*(r.t[0]-tt[0])+(r.t[1]-tt[1])*(r.t[1]-tt[1])+(r.t[2]-tt[2])*(r.t[2]-tt[2]));
        printf("[无噪声] rms=%.2e 旋转误差=%.2e° 位置误差=%.2e mm %s\n",
               r.rms, re, te, (re<1e-4&&te<1e-4)?"OK":"FAIL");
        ok = ok && re<1e-4 && te<1e-4;   // 手写雅可比SVD精度~1e-6°，非LAPACK但足够
    }

    // ---- 测试2：噪声鲁棒性（应约2.1°/mm，与Python一致）----
    {
        double sumRot=0; int cnt=0;
        for (int it=0; it<1000; ++it) {
            HandMat3 Rt=randRot(g); HandVec3 tt={nd(g)*100,nd(g)*100,nd(g)*100};
            std::array<HandVec3,5> obs;
            for(int i=0;i<5;++i){ HandVec3 w=transformPoint(tpl[i],Rt,tt);
                obs[i]={w[0]+nd(g),w[1]+nd(g),w[2]+nd(g)}; }
            auto r=solveHandBackPose(tpl,obs,allVis);
            if(r.confident){ sumRot+=rotErrDeg(r.R,Rt); cnt++; }
        }
        double avg=sumRot/cnt;
        printf("[1mm噪声] 姿态误差均值=%.3f° (期望~2.1°，marker几何决定) %s\n",
               avg, (avg<2.6)?"OK":"FAIL");
        ok = ok && avg<2.6;
    }

    // ---- 测试3：缺失点（剩3可解，剩2报不可信）----
    {
        HandMat3 Rt=randRot(g); HandVec3 tt={50,50,200};
        std::array<HandVec3,5> obs;
        for(int i=0;i<5;++i){ HandVec3 w=transformPoint(tpl[i],Rt,tt);
            obs[i]={w[0]+nd(g)*0.5,w[1]+nd(g)*0.5,w[2]+nd(g)*0.5}; }
        std::array<bool,5> m3={true,true,true,false,false};
        auto r3=solveHandBackPose(tpl,obs,m3);
        printf("[剩3球] 可信=%d 姿态误差=%.2f° %s\n", r3.confident,
               r3.confident?rotErrDeg(r3.R,Rt):-1, (r3.confident)?"OK":"FAIL");
        ok = ok && r3.confident;
        std::array<bool,5> m2={true,true,false,false,false};
        auto r2=solveHandBackPose(tpl,obs,m2);
        printf("[剩2球] 可信=%d (应0) %s\n", r2.confident, (!r2.confident)?"OK":"FAIL");
        ok = ok && !r2.confident;
    }

    // ---- 测试4：坏点检测（残差应变大）----
    {
        HandMat3 Rt=randRot(g); HandVec3 tt={0,0,250};
        std::array<HandVec3,5> obs;
        for(int i=0;i<5;++i) obs[i]=transformPoint(tpl[i],Rt,tt);
        obs[2]={obs[2][0]+30,obs[2][1]-20,obs[2][2]+15};
        auto r=solveHandBackPose(tpl,obs,allVis);
        printf("[坏点] rms=%.2f 可信=%d (应0) %s\n", r.rms, r.confident, (!r.confident)?"OK":"FAIL");
        ok = ok && !r.confident;
    }

    // ---- 测试5：端到端 FK+pose 闭环 ----
    {
        std::array<double,16> q{}; q[6]=0.8; q[9]=1.0; q[4]=0.3;
        std::array<std::array<double,3>,20> local;
        handFK(q, local);
        HandMat3 Rt=randRot(g); HandVec3 tt={120,-80,400};
        std::array<std::array<double,3>,20> world;
        for(int i=0;i<20;++i) world[i]=transformPoint(local[i],Rt,tt);
        std::array<HandVec3,5> back;
        for(int i=0;i<5;++i) back[i]=world[i];
        auto r=solveHandBackPose(tpl, back, allVis);
        // 用反解姿态重建全部20球
        double maxErr=0;
        for(int i=0;i<20;++i){ HandVec3 rec=transformPoint(local[i],r.R,r.t);
            double e=std::sqrt((rec[0]-world[i][0])*(rec[0]-world[i][0])
                +(rec[1]-world[i][1])*(rec[1]-world[i][1])+(rec[2]-world[i][2])*(rec[2]-world[i][2]));
            if(e>maxErr)maxErr=e; }
        printf("[端到端] 反解姿态重建20球最大误差=%.2e mm %s\n", maxErr, (maxErr<1e-6)?"OK":"FAIL");
        ok = ok && maxErr<1e-6;
    }

    // ---- 测试6：手背模板几何体检（防实测替换占位坐标时引入退化配置）----
    {
        // 用当前实际的手背模板（HandModel.hpp 里的 handBackMarkers()）跑一遍：
        // 目前是占位值，但已按"非对称+高度差"设计，应该通过体检。
        std::array<HandVec3,5> real;
        for (int i=0;i<5;++i) real[i] = handBackMarkers()[i];
        auto rReal = checkHandBackTemplateGeometry(real);
        printf("[模板体检-当前值] 最小三角面积=%.1f mm^2 (最差组合 {%d,%d,%d}) 通过=%d %s\n",
               rReal.minTriangleAreaMm2, rReal.worstI, rReal.worstJ, rReal.worstK,
               rReal.ok, rReal.ok?"OK":"FAIL");
        ok = ok && rReal.ok;

        // 反向验证：故意构造一组近似共线的退化模板，体检必须能拦住它——
        // 不能只是"现在的模板恰好没退化"，得验证检查本身真的会报警。
        std::array<HandVec3,5> degenerate = {{
            {0,0,0}, {10,0.01,0}, {20,-0.01,0}, {30,0.02,0}, {40,-0.02,0}
        }};
        auto rBad = checkHandBackTemplateGeometry(degenerate);
        printf("[模板体检-故意退化] 最小三角面积=%.4f mm^2 通过=%d (应为0) %s\n",
               rBad.minTriangleAreaMm2, rBad.ok, (!rBad.ok)?"OK":"FAIL");
        ok = ok && !rBad.ok;
    }

    printf("\n%s\n", ok ? "hand_pose C++: ALL PASS" : "hand_pose C++: FAIL");
    return ok ? 0 : 1;
}
