#include "estimate/DorsumRigidSolver.hpp"
#include <cstdio>
#include <random>
using namespace mocap::hm20; using V3=drs::V3;
int main(int argc,char**argv){
    const bool wdOn = (argc>1 && std::string(argv[1])=="on");
    std::array<V3,5> T{{{-20.1,13.4,0.7},{12.1,25.2,1.4},{18.7,2.5,-3.1},
                        {10.7,-25.0,2.4},{-21.4,-16.2,-1.4}}};
    std::array<V3,15> Ft{}; // 中立位手指：都在 +X 远端，明显偏离手背平面
    for(int i=0;i<15;++i) Ft[size_t(i)]={40.0+3.0*i, 8.0-1.2*i, -12.0};
    DorsumRigidSolver S; S.setTemplate(T); S.setFingerTemplate(Ft);
    std::mt19937 g(7); std::normal_distribution<double> nz(0,0.9), mv(0,1.2);
    // 阶段1(0-59): 正常建锁
    // 阶段2(60+):  把 0 号点挪到「让反序排列拟合得更好」的位置 -> 标签排错、残差却好
    std::array<V3,5> real=T;
    int wrongFrom=-1, backAt=-1, relockAt=-1; double ang=0;
    for(int f=0; f<300; ++f){
        if(f==60){ real[0]=V3{{-24.5,-13.0,-1.0}};  }   // 挪到接近 4 号槽位
        ang+=0.003;
        drs::M9 R{{std::cos(ang),-std::sin(ang),0,std::sin(ang),std::cos(ang),0,0,0,1}};
        V3 t{{200+mv(g),-30+mv(g),400+mv(g)}};
        auto W=[&](const V3&p){return V3{{R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0]+nz(g),
            R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1]+nz(g),
            R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2]+nz(g)}};};
        std::vector<V3> pts;
        for(int k=0;k<5;++k) pts.push_back(W(real[size_t(k)]));
        std::vector<DorsumRigidSolver::FingerObs> fo;
        for(int i=0;i<15;++i) fo.push_back({5+i, W(Ft[size_t(i)])});
        for(int q=0;q<10;++q) pts.push_back({{t[0]+mv(g)*40,t[1]+mv(g)*40,t[2]+mv(g)*40}});
        auto r=S.solve(pts,fo);
        if(r.ok) S.acceptHistory(r.R);
        int wd = wdOn ? S.watchdog(r) : 0;
        if(wd==1&&relockAt<0) relockAt=f;
        // 标签对不对：0 号槽位应该指向 pts[0]
        bool lab_ok = r.ok && r.pointOf[0]==0;
        if(f>=60 && !lab_ok && wrongFrom<0) wrongFrom=f;
        if(wrongFrom>=0 && lab_ok && backAt<0) backAt=f;
        if(f%40==0||wd) std::printf("f%3d 内点%d/5 rmse%5.2f 标签%s 手指%5.1fmm %s %s\n",
            f,r.nMatched,r.rmseMm, lab_ok?"对":"错", r.fingerScoreMm,
            r.fingerImplausible?"[不合理]":"", wd==1?"<<松锁":"");
    }
    std::printf("\n看门狗 %s: 标签从 f%d 开始错  松锁@%d  恢复正确@%s\n",
        wdOn?"开":"关", wrongFrom, relockAt,
        backAt<0?"从未恢复 ❌":(std::to_string(backAt)+" ✅").c_str());
    return 0;
}
