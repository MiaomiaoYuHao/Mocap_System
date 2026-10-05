// verify_reset_clears_prior.cpp —— 「手背复位」到底清没清网络的时序先验
//
//   g++ -std=c++20 -O2 -I src tools/hm20_diag/verify_reset_clears_prior.cpp -o vr && ./vr
//
// 【为什么单独验这一条】prevPos_/prevMask_ 是喂回网络的【上一帧标签先验】。
// 标签一旦乱掉，乱的先验每帧都被送回网络，把它往错的方向拽 —— 自我维持。
// 复位按钮第一版无效，就是因为只清了几何那边、漏了这个。
//
// 做法：用一个假后端把 in.prevMask 的置位数截下来。复位前应该是 20，
// 复位后【下一帧】应该是 0。这是唯一能直接看到"先验被清了"的办法 ——
// 从外面看不出来，因为它不体现在任何输出字段上。
//
// 期望输出：
//   跑 5 帧后 prevMask 置位数 = 20
//   复位后下一帧            = 0

#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
using namespace mocap::hm20;
struct FB : IHm20InferenceBackend {
    InferenceOutput run(const InferenceInput& in) override {
        InferenceOutput o; const int N=int(in.points.size());
        o.rows=N+1;o.cols=kNumClasses;
        o.logAssign.assign(size_t(o.rows)*size_t(o.cols),-20.0f);
        for(int i=0;i<N&&i<20;++i) o.logAssign[size_t(i*kNumClasses+i)]=-0.01f;
        // 把收到的 prev 记下来 —— 复位有没有清掉它，看这个
        lastPrevSum=0; for(int m=0;m<20;++m) lastPrevSum+=in.prevMask[size_t(m)]?1:0;
        o.ok=true; return o; }
    mutable int lastPrevSum=0;
};
int main(){
    auto be=std::make_shared<FB>();
    Hm20SkeletonAssociator a(be,{},false);
    std::array<Vec3,kNumMarkers> mm{};
    double rad[5]={28.2,35.2,21.8,23.7,34.4},ang[5]={0,74,145,212,289};
    for(int i=0;i<5;++i){double t=ang[i]*3.14159265358979/180.0;
        mm[size_t(i)]={rad[i]*std::cos(t),rad[i]*std::sin(t),0.0};}
    for(int i=5;i<20;++i) mm[size_t(i)]={40.0+3.0*i,10.0-1.5*i,-5.0};
    a.setTemplateMm(mm);
    std::vector<std::pair<int,Vec3>> cand;
    for(int i=0;i<20;++i) cand.push_back({1000+i,Vec3{mm[size_t(i)][0]+200,mm[size_t(i)][1]-30,mm[size_t(i)][2]+11}});
    for(int f=0;f<5;++f) a.process(cand);
    std::printf("  跑 5 帧后，网络收到的 prevMask 置位数 = %d\n", be->lastPrevSum);
    a.resetDorsumRigid();
    a.process(cand);
    std::printf("  复位后【下一帧】网络收到的 prevMask 置位数 = %d   %s\n",
        be->lastPrevSum, be->lastPrevSum==0?"✅ 时序先验确实被清了":"❌ 没清掉");
    return 0;
}
