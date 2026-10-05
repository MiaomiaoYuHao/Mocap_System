// verify_time_constants.cpp —— 四个恢复机制到底第几帧动作
//
//   g++ -std=c++20 -O2 -I src tools/hm20_diag/verify_time_constants.cpp -o vb && ./vb
//
// 【为什么需要这个】配置值对，不代表逻辑会跑。这个会话里出现过一次
// "bundleRefitGrowth 声明了、但没有任何地方引用它"，配置打印是正常的，
// 行为完全没变。所以要造出触发条件，实测它第几帧动作。
//
// 期望输出（±2 帧内）：
//   ① 贴点重捕      第 17 帧   0.68s
//   ② 看门狗松锁    坏  5 帧   0.20s
//   ③ 退回原始模板  坏 24 帧   0.96s
//   ④ 持续微调      第 21 帧   0.84s   (到 63%，即一个时间常数)

#include "estimate/DorsumRigidSolver.hpp"
#include <cstdio>
#include <random>
using namespace mocap::hm20; using V3=drs::V3;
static std::array<V3,5> T{{{-20.1,13.4,0.7},{12.1,25.2,1.4},{18.7,2.5,-3.1},
                           {10.7,-25.0,2.4},{-21.4,-16.2,-1.4}}};
static std::vector<V3> mk(const std::array<V3,5>& real,double a,std::mt19937&g,int clutter=10){
    std::normal_distribution<double> nz(0,0.9),mv(0,1.2);
    drs::M9 R{{std::cos(a),-std::sin(a),0,std::sin(a),std::cos(a),0,0,0,1}};
    V3 t{{200+mv(g),-30+mv(g),400+mv(g)}};
    std::vector<V3> p;
    for(int k=0;k<5;++k){const V3&q=real[size_t(k)];
        p.push_back({{R[0]*q[0]+R[1]*q[1]+R[2]*q[2]+t[0]+nz(g),
                      R[3]*q[0]+R[4]*q[1]+R[5]*q[2]+t[1]+nz(g),
                      R[6]*q[0]+R[7]*q[1]+R[8]*q[2]+t[2]+nz(g)}});}
    for(int i=0;i<clutter;++i) p.push_back({{t[0]+mv(g)*50,t[1]+mv(g)*50,t[2]+mv(g)*50}});
    return p;
}
int main(){
    const double FPS=25.0;
    // ---- ① 重捕：18 帧观察期 ----
    {   DorsumRigidSolver S; S.setTemplate(T);
        std::mt19937 g(4); std::array<V3,5> real=T;
        real[0]={-20.1+11,13.4-9,0.7+5};      // 重贴，偏 15mm
        int hit=-1;
        for(int f=0;f<200;++f){ auto p=mk(real,f*0.004,g);
            auto r=S.solve(p); if(r.ok){S.acceptHistory(r.R);S.acceptCentroid(r.t);}
            auto e=S.observeForRepair(p,r);
            if(e.happened&&hit<0) hit=f; }
        std::printf("① 贴点重捕      第 %3d 帧触发  = %.2fs   %s\n",hit,hit/FPS,
            (hit>=0&&hit<40)?"✅ 生效":"❌"); }
    // ---- ② 看门狗松锁：连续 6 帧解不好 ----
    {   DorsumRigidSolver S; S.setTemplate(T);
        std::mt19937 g(4); int relock=-1;
        for(int f=0;f<120;++f){
            std::vector<V3> p;
            if(f<30) p=mk(T,f*0.004,g);              // 正常，建锁
            else { std::normal_distribution<double> mv(0,1.2);   // 之后全是杂点
                   for(int i=0;i<12;++i) p.push_back({{200+mv(g)*60,-30+mv(g)*60,400+mv(g)*60}}); }
            auto r=S.solve(p); if(r.ok){S.acceptHistory(r.R);S.acceptCentroid(r.t);}
            int wd=S.watchdog(r);
            if(wd==1&&relock<0) relock=f-30; }
        std::printf("② 看门狗松锁    坏 %3d 帧后触发 = %.2fs   %s\n",relock,relock/FPS,
            (relock>0&&relock<=8)?"✅ 生效":"❌"); }
    // ---- ③ 退回模板：连续 25 帧 ----
    {   DorsumRigidSolver S; S.setTemplate(T);
        std::mt19937 g(4); int revert=-1;
        for(int f=0;f<200;++f){
            std::vector<V3> p;
            if(f<30) p=mk(T,f*0.004,g);
            else { std::normal_distribution<double> mv(0,1.2);
                   for(int i=0;i<12;++i) p.push_back({{200+mv(g)*60,-30+mv(g)*60,400+mv(g)*60}}); }
            auto r=S.solve(p); if(r.ok){S.acceptHistory(r.R);S.acceptCentroid(r.t);}
            int wd=S.watchdog(r);
            if(wd==2&&revert<0) revert=f-30; }
        std::printf("③ 退回原始模板  坏 %3d 帧后触发 = %.2fs   %s\n",revert,revert/FPS,
            (revert>0&&revert<=30)?"✅ 生效":"❌"); }
    // ---- ④ 持续微调：τ≈0.8s ----
    {   DorsumRigidSolver S; S.setTemplate(T);
        std::mt19937 g(4); std::array<V3,5> real=T;
        for(int f=0;f<60;++f){auto p=mk(real,f*0.004,g);
            auto r=S.solve(p); if(r.ok){S.acceptHistory(r.R);S.acceptCentroid(r.t);S.refineTemplate(p,r);} }
        real[2]={18.7+3.0,2.5+2.5,-3.1+1.5};   // 突然漂 4.3mm
        double d0=S.templateDriftMm(); int t63=-1;
        for(int f=0;f<200;++f){auto p=mk(real,f*0.004,g);
            auto r=S.solve(p); if(r.ok){S.acceptHistory(r.R);S.acceptCentroid(r.t);S.refineTemplate(p,r);}
            if(t63<0 && S.templateDriftMm()-d0 > 0.63*2.9) t63=f; }
        std::printf("④ 持续微调      第 %3d 帧到 63%%  = %.2fs   %s\n",t63,t63/FPS,
            (t63>0&&t63<40)?"✅ 生效":"❌"); }
    return 0;
}
