// 把录制里的骨架帧喂进 Hm20AutoCalib，数束调整触发几次、总耗时多少。
// 用 cfg 模拟"修复前"（退避关掉、停滞不停、拇指旋前不限次）做对照。
#include "estimate/Hm20AutoCalib.hpp"
#include "record/PointCloudRecorder.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <chrono>
using namespace mocap;
using namespace mocap::hm20;
static std::vector<pcrec::SkeletonRec> loadSk(const char* p){
    std::ifstream f(p,std::ios::binary);
    std::vector<char> raw((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
    uint32_t hl; std::memcpy(&hl,raw.data()+12,4); size_t o=16+hl;
    std::vector<pcrec::SkeletonRec> v;
    while(o+5<=raw.size()){ uint8_t ct=uint8_t(raw[o]); uint32_t cl;
        std::memcpy(&cl,raw.data()+o+1,4); if(o+5+cl>raw.size())break;
        if(ct==uint8_t(pcrec::ChunkType::Skeleton)){pcrec::SkeletonRec s;
            std::memcpy(&s,raw.data()+o+5+8,sizeof(s)); v.push_back(s);} o+=5+cl; }
    return v;
}
static SkeletonFrameResult toResult(const pcrec::SkeletonRec& s){
    SkeletonFrameResult r;
    r.valid = s.valid!=0; r.wristPoseValid = s.wristPoseValid!=0;
    r.pentagonOk = s.pentagonOk!=0; r.dorsumRmseMm = s.dorsumRmseMm;
    for(int m=0;m<20;++m){
        r.markers[size_t(m)].label=m;
        r.markers[size_t(m)].posWorld={s.pos[m*3],s.pos[m*3+1],s.pos[m*3+2]};
        r.markers[size_t(m)].observed = s.observed[m]!=0;
        r.markers[size_t(m)].confidence = s.conf[m];
        r.markers[size_t(m)].sourcePointId = s.observed[m]? m : -1;
    }
    for(int i=0;i<9;++i) r.wristR[size_t(i)]=s.wristR[i];
    for(int i=0;i<3;++i) r.wristT[size_t(i)]=s.wristT[i];
    for(int g=0;g<16;++g){ for(int i=0;i<4;++i) r.segQuat[size_t(g)][size_t(i)]=s.segQuat[g*4+i];
        r.segSource[size_t(g)]=SegSource(s.segSource[g]); }
    return r;
}
struct Stat{ int feeds=0; double totalMs=0, maxMs=0; int spikes=0; };
static Stat run(const std::vector<pcrec::SkeletonRec>& sk, bool fixed){
    Hm20AutoCalib ac;
    AutoCalibConfig c;               // 默认 = 修复后
    if(!fixed){                      // 模拟修复前
        c.bundleRefitGrowth = 0;     // 每次都重跑
        c.bundleStallRuns   = 1<<28; // 永不判停滞
        c.thumbPronMaxTries = 1<<28; // 拇指旋前无限次搜
    }
    ac.configure(c);
    Stat st;
    for(const auto& s : sk){
        auto r=toResult(s);
        auto t0=std::chrono::steady_clock::now();
        ac.feed(r);
        double ms=std::chrono::duration<double,std::milli>(
            std::chrono::steady_clock::now()-t0).count();
        st.feeds++; st.totalMs+=ms; if(ms>st.maxMs)st.maxMs=ms; if(ms>3.0)st.spikes++;   // >3ms = 这一帧跑了束调整
    }
    const auto& R=ac.result();
    std::printf("   [%s] 终态 stage=%d shots=%d frameAnatomical=%d backValid=%d bundleRmse=%.2f\n",
        fixed?"修复后":"修复前", int(ac.stage()), R.bundleShots, (int)R.frameAnatomical,
        (int)R.backValid, R.bundleRmseMm);
    return st;
}
int main(int argc,char**argv){
    auto sk=loadSk(argv[1]);
    std::printf("录制: %s\n骨架帧 %zu\n\n", argv[1], sk.size());
    Stat a=run(sk,false), b=run(sk,true);
    std::printf("%-14s%10s%12s%12s%14s\n","","帧数","总耗时ms","单帧max","束调整次数");
    std::printf("%-14s%10d%12.1f%12.1f%12d\n","修复前",a.feeds,a.totalMs,a.maxMs,a.spikes);
    std::printf("%-14s%10d%12.1f%12.1f%12d\n","修复后",b.feeds,b.totalMs,b.maxMs,b.spikes);
    if(a.totalMs>0) std::printf("\n自标定总开销降到 %.0f%%   尖峰次数 %d -> %d\n",
        100*b.totalMs/a.totalMs, a.spikes, b.spikes);
    return 0;
}
