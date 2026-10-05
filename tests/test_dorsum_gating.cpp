// 专门验空间门控：杂点散得很开，门控会真的过滤掉大部分，
// 这时 pts 和 P 的下标空间不同，重映射错了就会标错点。
#include "estimate/DorsumRigidSolver.hpp"
#include <cstdio>
#include <random>
using namespace mocap::hm20; using V3=drs::V3;
int main(){
    std::array<V3,5> T{{{-20.1,13.4,0.7},{12.1,25.2,1.4},{18.7,2.5,-3.1},
                        {10.7,-25.0,2.4},{-21.4,-16.2,-1.4}}};
    DorsumRigidSolver S; S.setTemplate(T);
    std::mt19937 g(5); std::normal_distribution<double> nz(0,0.9), far(0,300);
    int ok=0,wrong=0,abst=0,gatedFrames=0;
    for(int f=0; f<600; ++f){
        double a=f*0.004;
        drs::M9 R{{std::cos(a),-std::sin(a),0,std::sin(a),std::cos(a),0,0,0,1}};
        V3 t{{200.0,-30.0,400.0}};
        std::vector<V3> pts; std::vector<int> gt(5,-1);
        // 先放 20 个远处杂点，手背点【放在后面】—— 下标空间差异最大
        for(int q=0;q<20;++q) pts.push_back({{t[0]+far(g),t[1]+far(g),t[2]+far(g)}});
        for(int k=0;k<5;++k){ const V3&p=T[size_t(k)];
            gt[size_t(k)]=int(pts.size());
            pts.push_back({{R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0]+nz(g),
                            R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1]+nz(g),
                            R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2]+nz(g)}}); }
        auto r=S.solve(pts);
        if(r.ok){ S.acceptHistory(r.R); S.acceptCentroid(r.t); if(f>0)++gatedFrames; }
        if(!r.ok){++abst;continue;}
        bool bad=false;
        for(int k=0;k<5;++k) if(r.pointOf[size_t(k)]>=0 && r.pointOf[size_t(k)]!=gt[size_t(k)]) bad=true;
        if(bad)++wrong; else ++ok;
    }
    std::printf("杂点散布 ±300mm，门控半径 70mm（门控会滤掉绝大多数杂点）\n");
    std::printf("  手背点放在数组【末尾】(下标 20~24)，重映射错了必然标错\n\n");
    std::printf("  正确 %d   错标 %d   拒解 %d   （门控生效帧 %d）\n",ok,wrong,abst,gatedFrames);
    std::printf("  %s\n", wrong==0 ? "✅ 下标映射正确" : "❌ 下标映射有问题");
    return 0;
}
