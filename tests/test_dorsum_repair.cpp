#include "estimate/DorsumRigidSolver.hpp"
#include <cstdio>
#include <random>
using namespace mocap::hm20;
using V3=drs::V3;
int main(){
    // 你这次录制里的真实手背模板
    std::array<V3,5> T{{{-20.1,13.4,0.7},{12.1,25.2,1.4},{18.7,2.5,-3.1},
                        {10.7,-25.0,2.4},{-21.4,-16.2,-1.4}}};
    DorsumRigidSolver S; S.setTemplate(T);
    std::printf("原模板自歧义 %.2f mm\n\n", S.selfAmbiguityMm());
    // m0 在 17s 掉了，重贴到偏离原位 16mm 的地方（实测 14~18mm）
    std::array<V3,5> real=T; real[0]={-20.1+11.0, 13.4-9.0, 0.7+5.0};
    std::printf("重贴后 m0 偏离原位 %.1f mm\n",
      std::sqrt(11.0*11.0+9.0*9.0+5.0*5.0));
    std::mt19937 g(4); std::normal_distribution<double> nz(0,0.9), mv(0,1.5);
    int firstFix=-1; double ang=0;
    for(int f=0; f<400; ++f){
        ang += 0.004;
        drs::M9 R{{std::cos(ang),-std::sin(ang),0, std::sin(ang),std::cos(ang),0, 0,0,1}};
        V3 t{{200+mv(g), -30+mv(g), 400+mv(g)}};
        std::vector<V3> pts;
        for(int k=0;k<5;++k){
            const V3&p=real[size_t(k)];
            pts.push_back({{R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0]+nz(g),
                            R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1]+nz(g),
                            R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2]+nz(g)}});
        }
        for(int q=0;q<14;++q) pts.push_back({{t[0]+mv(g)*40,t[1]+mv(g)*40,t[2]+mv(g)*40}});
        auto r=S.solve(pts);
        if(r.ok) S.acceptHistory(r.R);
        auto ev=S.observeForRepair(pts,r);
        if(f%50==0||ev.happened||ev.rejected)
            std::printf("f%3d 内点%d/5 rmse%.2f  %s\n",f,r.nMatched,r.rmseMm,
                ev.happened?"★模板已修复":(ev.rejected?ev.why:""));
        if(ev.happened&&firstFix<0){ firstFix=f;
            std::printf("     槽位%d 移动%.1fmm 散布%.2fmm 用了%d帧 新自歧义%.2fmm\n     -> %s\n",
              ev.slot,ev.moveMm,ev.scatterMm,ev.frames,ev.newSelfAmbMm,ev.why);}
    }
    std::printf("\n首次修复在第 %d 帧（约 %.1f 秒 @24fps）\n",firstFix,firstFix/24.0);
    return 0;
}
