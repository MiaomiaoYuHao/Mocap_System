// verify_bundle_async.cpp —— 束调整是不是真的离开了主线程
//
//   g++ -std=c++20 -O2 -I src tools/hm20_diag/verify_bundle_async.cpp -o va && ./va
//
// 【这个测试目前是失败的，留着是为了以后能补上】
// 我试过两种办法触发束调整，都不成：
//   ① 从 .pcrec 复算：重建 SkeletonFrameResult 会丢状态，自标定走不到采集阶段
//   ② 合成手：过不了自标定的静止播种/漂移检查等门限，stage 停在 Idle
// 所以"异步生效了"这句话【没有被验证过】，别当成已完成。
//
// std::async(std::launch::async) + wait_for(0) 这个机制本身不会阻塞，
// 这是语言保证的。不确定的是【启动点会不会被走到】——
// 而那正是 bundleRefitGrowth 那次"声明了没人用"的同一类问题。
//
// 真机上直接看这两个数（已录进 .pcrec 的 Hm20Diag 块）：
//   bundleRuns 一直 0            -> 启动点没被走到
//   bundleRuns 涨、latencyMs 无尖峰 -> 异步生效
//   bundleRuns 涨、latencyMs 有尖峰 -> 没生效

#include "estimate/Hm20AutoCalib.hpp"
#include <cstdio>
#include <chrono>
#include <thread>
#include <random>
using namespace mocap::hm20;
int main(){
    Hm20AutoCalib ac;
    // 造静止手 + 缓慢变化的姿势，喂到它自己走完 Collecting->Ready
    std::mt19937 g(7); std::normal_distribution<double> nz(0,0.4);
    double maxMs=0; int frames=0, firstBundleFrame=-1;
    auto mkFrame=[&](int f){
        SkeletonFrameResult r; r.valid=true; r.wristPoseValid=true; r.pentagonOk=true;
        r.dorsumRmseMm=0.6;
        double bend=0.6*std::sin(f*0.02);
        double back[5][3]={{-20.1,13.4,0.7},{12.1,25.2,1.4},{18.7,2.5,-3.1},
                           {10.7,-25.0,2.4},{-21.4,-16.2,-1.4}};
        for(int m=0;m<5;++m){ r.markers[size_t(m)].label=m; r.markers[size_t(m)].observed=true;
            r.markers[size_t(m)].sourcePointId=m;
            r.markers[size_t(m)].posWorld={back[m][0]+nz(g),back[m][1]+nz(g),back[m][2]+nz(g)}; }
        for(int fi=0;fi<5;++fi) for(int j=0;j<3;++j){
            int m=5+fi*3+j; r.markers[size_t(m)].label=m; r.markers[size_t(m)].observed=true;
            r.markers[size_t(m)].sourcePointId=m;
            double L=25.0*(j+1)*std::cos(bend*(j+1)*0.5);
            r.markers[size_t(m)].posWorld={20.0+L, -20.0+12.0*fi+nz(g),
                                           -25.0*(j+1)*std::sin(bend*(j+1)*0.5)+nz(g)};}
        r.wristR={1,0,0, 0,1,0, 0,0,1}; r.wristT={0,0,0};
        for(int s=0;s<16;++s) r.segSource[size_t(s)]=SegSource::Geometry;
        return r; };
    for(int f=0; f<3000; ++f){
        auto r=mkFrame(f);
        auto t0=std::chrono::steady_clock::now();
        ac.feed(r);
        double ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-t0).count();
        if(ms>maxMs) maxMs=ms; ++frames;
        if(ac.result().bundleRuns>0 && firstBundleFrame<0) firstBundleFrame=f;
    }
    for(int i=0;i<300;++i){ ac.pollBundleJob();
        std::this_thread::sleep_for(std::chrono::milliseconds(3)); }
    const auto& R=ac.result();
    std::printf("  %d 帧   stage=%d shots=%d\n",frames,int(ac.stage()),R.bundleShots);
    std::printf("  后台束调整跑了 %d 次，最近一次耗时 %.1fms\n", R.bundleRuns, R.bundleLastMs);
    std::printf("  主线程单帧最大 %.2fms\n", maxMs);
    if(R.bundleRuns==0){ std::puts("  ⚠ 束调整仍未触发，这个测试不能下结论"); return 1; }
    std::printf("  %s\n", (maxMs < R.bundleLastMs*0.3)
        ? "✅ 束调整在后台：主线程最大耗时远小于束调整本身"
        : "❌ 主线程仍然被阻塞");
    return 0;
}
