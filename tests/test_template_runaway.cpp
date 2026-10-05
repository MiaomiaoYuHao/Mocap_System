#include "estimate/DorsumRigidSolver.hpp"
#include <cstdio>
#include <random>
#include <chrono>
using namespace mocap::hm20; using V3=drs::V3;
int main(){
    std::array<V3,5> T{{{-20.1,13.4,0.7},{12.1,25.2,1.4},{18.7,2.5,-3.1},
                        {10.7,-25.0,2.4},{-21.4,-16.2,-1.4}}};
    DorsumRigidSolver S; S.setTemplate(T);
    std::mt19937 g(9); std::normal_distribution<double> nz(0,1.2), mv(0,1.5);
    std::printf("=== 贴点【完全不漂】，跑 6000 帧(4分钟@25fps)，看模板会不会随机游走 ===\n");
    std::printf("  %-8s%12s%12s%10s\n","帧","模板漂移mm","自歧义mm","残差mm");
    double totalMs=0; int nSolve=0;
    for(int f=0; f<6000; ++f){
        double a=f*0.002;
        drs::M9 R{{std::cos(a),-std::sin(a),0,std::sin(a),std::cos(a),0,0,0,1}};
        V3 t{{200+mv(g),-30+mv(g),400+mv(g)}};
        std::vector<V3> pts;
        for(int k=0;k<5;++k){const V3&p=T[size_t(k)];
            pts.push_back({{R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0]+nz(g),
                            R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1]+nz(g),
                            R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2]+nz(g)}});}
        for(int q=0;q<12;++q) pts.push_back({{t[0]+mv(g)*50,t[1]+mv(g)*50,t[2]+mv(g)*50}});
        auto r=S.solve(pts);
        if(r.ok){S.acceptHistory(r.R);S.acceptCentroid(r.t);
            auto t0=std::chrono::steady_clock::now();
            S.refineTemplate(pts,r);
            totalMs+=std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-t0).count(); ++nSolve;}
        if(f%1000==0||f==5999)
            std::printf("  %-8d%12.2f%12.2f%10.2f\n",f,S.templateDriftMm(),
                        S.selfAmbiguityMm(),r.rmseMm);
    }
    std::printf("\n  最终漂移 %.2fmm  %s（噪声 1.2mm，漂移应该在噪声量级以内）\n",
        S.templateDriftMm(), S.templateDriftMm()<2.0?"✅ 没有跑飞":"❌ 在游走");
    std::printf("  refineTemplate 单次 %.4f ms  (@120fps 占 %.3f%%)\n",
        totalMs/nSolve, 100*(totalMs/nSolve)/8.33);
    return 0;
}
