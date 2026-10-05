#include "estimate/DorsumRigidSolver.hpp"
#include <cstdio>
#include <random>
using namespace mocap::hm20; using V3=drs::V3;
static void run(bool refineOn){
    std::array<V3,5> T{{{-20.1,13.4,0.7},{12.1,25.2,1.4},{18.7,2.5,-3.1},
                        {10.7,-25.0,2.4},{-21.4,-16.2,-1.4}}};
    DorsumRigidSolver S; auto c=S.config(); c.refineEnabled=refineOn; S.setConfig(c);
    S.setTemplate(T);
    std::mt19937 g(3); std::normal_distribution<double> nz(0,0.9), mv(0,1.2);
    std::array<V3,5> real=T;
    std::printf("  %-7s%8s%10s%10s%10s\n","帧","真实漂移","残差mm","模板漂移","内点");
    for(int f=0; f<900; ++f){
        // 2 号点在 100~200 帧之间线性漂 4.5mm（在 inlierTol=6mm 以内）
        if(f>=100&&f<200){ double u=(f-100)/100.0;
            real[2]={18.7+3.0*u, 2.5+2.5*u, -3.1+1.5*u}; }
        double drift=drs::norm(drs::sub(real[2],T[2]));
        double a=f*0.003;
        drs::M9 R{{std::cos(a),-std::sin(a),0,std::sin(a),std::cos(a),0,0,0,1}};
        V3 t{{200+mv(g),-30+mv(g),400+mv(g)}};
        std::vector<V3> pts;
        for(int k=0;k<5;++k){const V3&p=real[size_t(k)];
            pts.push_back({{R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0]+nz(g),
                            R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1]+nz(g),
                            R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2]+nz(g)}});}
        for(int q=0;q<10;++q) pts.push_back({{t[0]+mv(g)*50,t[1]+mv(g)*50,t[2]+mv(g)*50}});
        auto r=S.solve(pts);
        if(r.ok){S.acceptHistory(r.R);S.acceptCentroid(r.t); S.refineTemplate(pts,r);}
        if(f%100==0||f==250||f==899)
            std::printf("  %-7d%8.1f%10.2f%10.1f%10d\n",f,drift,r.rmseMm,
                        S.templateDriftMm(),r.nMatched);
    }
}
int main(){
    std::puts("=== 关闭持续微调（现状）===");   run(false);
    std::puts("\n=== 打开持续微调 ===");        run(true);
    return 0;
}
