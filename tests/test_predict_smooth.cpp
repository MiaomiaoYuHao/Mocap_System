#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
#include <cmath>
#include <random>
using namespace mocap::hm20;
using namespace mocap::hm20::detail;
struct FB : IHm20InferenceBackend {
    std::array<Vec3,20> pos{}; double jitter=0; std::vector<int> lab;
    mutable std::mt19937 g{11};
    InferenceOutput run(const InferenceInput& in) override {
        InferenceOutput o; const int n=int(in.points.size());
        o.rows=n+1;o.cols=kNumClasses;
        o.logAssign.assign(size_t(o.rows)*size_t(o.cols),-30.0f);
        for(int i=0;i<n&&i<int(lab.size());++i)
            o.logAssign[size_t(i)*size_t(kNumClasses)+size_t(lab[size_t(i)])]=-0.01f;
        std::normal_distribution<double> nz(0,jitter);
        for(int m=0;m<20;++m) o.pos[size_t(m)]=
            Vec3{pos[size_t(m)][0]+nz(g),pos[size_t(m)][1]+nz(g),pos[size_t(m)][2]+nz(g)};
        o.ok=true; return o; }
};
static double run(double alpha,double jit,double& lag){
    auto be=std::make_shared<FB>();
    Hm20Config cfg; cfg.chainContinue=false; cfg.predictSmoothAlpha=alpha;
    Hm20SkeletonAssociator a(be,{},false,cfg);
    std::array<Vec3,kNumMarkers> tm{};
    double rad[5]={28.2,35.2,21.8,23.7,34.4},ang[5]={0,74,145,212,289};
    for(int i=0;i<5;++i){double t=ang[i]*M_PI/180;tm[size_t(i)]={rad[i]*std::cos(t),rad[i]*std::sin(t),0};}
    for(int i=5;i<20;++i) tm[size_t(i)]={40.0+3.0*i,10.0-1.5*i,-5.0};
    a.setTemplateMm(tm);
    std::vector<Vec3> got,want;
    for(int f=0; f<260; ++f){
        std::array<Vec3,20> T{};
        for(int k=0;k<5;++k) T[size_t(k)]={tm[size_t(k)][0]+200,tm[size_t(k)][1]-30,tm[size_t(k)][2]+400};
        Vec3 pp{240,-30,400};
        double b=0.4*std::sin(f*0.06);
        Vec3 u1{std::cos(b),0,-std::sin(b)}, u2{std::cos(b*1.6),0,-std::sin(b*1.6)};
        T[8]=pp; T[9]={pp[0]+u1[0]*25,pp[1],pp[2]+u1[2]*25};
        T[10]={T[9][0]+u2[0]*18,T[9][1],T[9][2]+u2[2]*18};
        for(int k=11;k<20;++k) T[size_t(k)]={300.0+k,-30.0,400.0};
        be->pos=T; be->jitter=jit;
        std::vector<std::pair<int,Vec3>> cand; be->lab.clear();
        for(int m=0;m<20;++m){ if(f>=60&&(m==9||m==10)) continue;
            cand.push_back({1000+m,T[size_t(m)]}); be->lab.push_back(m); }
        auto r=a.process(cand);
        if(f>=80){ got.push_back(r.markers[10].posWorld); want.push_back(T[10]); }
    }
    // 抖动 = 二阶差分（真实运动是平滑的正弦，二阶差分小）
    double jsum=0; int jn=0;
    for(size_t i=2;i<got.size();++i){
        Vec3 d{got[i][0]-2*got[i-1][0]+got[i-2][0],
               got[i][1]-2*got[i-1][1]+got[i-2][1],
               got[i][2]-2*got[i-1][2]+got[i-2][2]};
        jsum+=norm(d); ++jn; }
    // 滞后 = 跟真值的距离
    double lsum=0; for(size_t i=0;i<got.size();++i) lsum+=norm(sub(got[i],want[i]));
    lag=lsum/double(got.size());
    return jsum/double(jn);
}
int main(){
    std::printf("食指远节遮挡，网络 pos 带抖动。alpha=1.0 等于关闭平滑\n\n");
    std::printf("%-10s%14s%14s%14s\n","alpha","抖动(二阶差分)","跟真值距离","时间常数");
    for(double a:{1.0,0.6,0.35,0.2,0.1}){
        double lag=0; double j=run(a,2.0,lag);
        std::printf("%-10.2f%14.3f%14.2f%12.0f帧\n",a,j,lag,1.0/a);
    }
    std::printf("\n（抖动 2mm 模拟 INT8：实测二阶差分 0.20 -> 1.94mm，放大 9.7 倍）\n");
    return 0;
}
