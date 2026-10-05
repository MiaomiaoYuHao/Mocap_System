#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
#include <cmath>
#include <random>
using namespace mocap::hm20;
using namespace mocap::hm20::detail;
struct FB : IHm20InferenceBackend {
    std::array<Vec3,20> pos{}; double jitter=0; mutable std::mt19937 g{7};
    std::vector<int> lab;                     // 第 i 个候选点的真实标签
    InferenceOutput run(const InferenceInput& in) override {
        InferenceOutput o; const int n=int(in.points.size());
        o.rows=n+1;o.cols=kNumClasses;
        o.logAssign.assign(size_t(o.rows)*size_t(o.cols),-30.0f);
        // 【只给实际存在的候选点高概率】上一版按下标给，等于让模型"认领"
        // 了根本没喂进来的标签，于是遮挡点被标成 observed，测试测不到东西。
        for(int i=0;i<n&&i<int(lab.size());++i)
            o.logAssign[size_t(i)*size_t(kNumClasses)+size_t(lab[size_t(i)])]=-0.01f;
        std::normal_distribution<double> nz(0,jitter);
        for(int m=0;m<20;++m) o.pos[size_t(m)]=
            Vec3{pos[size_t(m)][0]+nz(g),pos[size_t(m)][1]+nz(g),pos[size_t(m)][2]+nz(g)};
        o.ok=true; return o; }
};
int main(){
    auto be=std::make_shared<FB>();
    Hm20Config cfg; cfg.chainContinue=false;      // 强制走 pos 头那条路
    Hm20SkeletonAssociator a(be,{},false,cfg);
    std::array<Vec3,kNumMarkers> tm{};
    double rad[5]={28.2,35.2,21.8,23.7,34.4},ang[5]={0,74,145,212,289};
    for(int i=0;i<5;++i){double t=ang[i]*M_PI/180;tm[size_t(i)]={rad[i]*std::cos(t),rad[i]*std::sin(t),0};}
    for(int i=5;i<20;++i) tm[size_t(i)]={40.0+3.0*i,10.0-1.5*i,-5.0};
    a.setTemplateMm(tm);
    // 真实手：食指三点，骨长固定 25 / 18
    auto truth=[&](double t){
        std::array<Vec3,20> P{};
        for(int k=0;k<5;++k) P[size_t(k)]={tm[size_t(k)][0]+200,tm[size_t(k)][1]-30,tm[size_t(k)][2]+400};
        Vec3 pp{240,-30,400};
        double b=0.5*std::sin(t*0.05);
        Vec3 u1{std::cos(b),0,-std::sin(b)}, u2{std::cos(b*1.6),0,-std::sin(b*1.6)};
        P[8]=pp; P[9]={pp[0]+u1[0]*25,pp[1],pp[2]+u1[2]*25};
        P[10]={P[9][0]+u2[0]*18,P[9][1],P[9][2]+u2[2]*18};
        for(int k=11;k<20;++k) P[size_t(k)]={300.0+k,-30.0,400.0};
        return P; };
    std::printf("食指：pp 一直可见，mp/dp 从第 60 帧起遮挡。网络 pos 加 2mm 抖动\n");
    std::printf("%-6s%8s%10s%10s%12s\n","帧","状态","pp-mp","mp-dp","dp帧间位移");
    Vec3 prevDp{}; bool first=true;
    for(int f=0; f<100; ++f){
        auto T=truth(f); be->pos=T; be->jitter=2.0;
        std::vector<std::pair<int,Vec3>> cand; be->lab.clear();
        for(int m=0;m<20;++m){
            if(f>=60 && (m==9||m==10)) continue;      // 遮挡
            cand.push_back({1000+m,T[size_t(m)]}); be->lab.push_back(m);
        }
        auto r=a.process(cand);
        double L=norm(sub(r.markers[9].posWorld,r.markers[8].posWorld));
        double L2=norm(sub(r.markers[10].posWorld,r.markers[9].posWorld));
        double jm= first?0:norm(sub(r.markers[10].posWorld,prevDp));
        prevDp=r.markers[10].posWorld; first=false;
        if(f==55||f==59||f==60||f==61||f==65||f==80||f==99)
            std::printf("%-6d%8s%10.2f%10.2f%12.2f  snap=%d\n",f,
                (f>=60?"遮挡":"可见"),L,L2,jm,
                r.boneLenSnapped);
    }
    std::printf("\n真实骨长 pp->mp = 25.00mm\n");
    return 0;
}
