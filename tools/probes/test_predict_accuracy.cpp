// 唯一有意义的判据：处理后的预测位置，跟【恢复瞬间的实测真值】差多少
#include "estimate/HandSkeletonAssociator.hpp"
#include "record/PointCloudRecorder.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
using namespace mocap; using namespace mocap::hm20; using namespace mocap::hm20::detail;
int main(int argc,char**argv){
    // 【自动补的入参守卫】这个文件原来在 tests/ 里，但它需要一个 .pcrec 数据
    // 文件当参数。CI 或者"把 tests 全跑一遍"的时候没人给参数，argv[1] 是空指针，
    // 于是它段错误 —— 而段错误在汇总里跟"断言失败"长得一模一样。
    // 结果是 58 个测试里有 7 个长期红着，整个测试套没法当门禁用：
    // 一旦有真的回归混进这 7 个里，没人分得出来。
    // 现在它挪到了 tools/probes/，并且没给参数时打印用法正常退出。
    if (argc < 2) {
        std::fprintf(stderr,
            "用法: %s <capture.pcrec> [更多参数]\n"
            "这是需要真实录制素材的分析探针，不是自包含单测。\n", argv[0]);
        return 0;
    }

    std::ifstream f(argv[1],std::ios::binary);
    std::vector<char> raw((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
    uint32_t hl; std::memcpy(&hl,raw.data()+12,4); size_t o=16+hl;
    std::vector<pcrec::SkeletonRec> sk;
    while(o+5<=raw.size()){uint8_t ct=uint8_t(raw[o]);uint32_t cl;std::memcpy(&cl,raw.data()+o+1,4);
        if(o+5+cl>raw.size())break;
        if(ct==uint8_t(pcrec::ChunkType::Skeleton)){pcrec::SkeletonRec s;
            std::memcpy(&s,raw.data()+o+5+8,sizeof(s));sk.push_back(s);} o+=5+cl;}
    const int N=int(sk.size());
    auto V=[&](int fr,int m){return sk[size_t(fr)].observed[m]!=0;};
    auto run=[&](bool snap,bool cont,std::vector<std::array<Vec3,20>>& out){
        out.clear(); out.resize(size_t(N));
        Hm20Config cfg; if(!cont){cfg.predictSmoothAlpha=1.0;cfg.recoverBlendDecay=0.0;}
        Hm20SkeletonAssociator a(nullptr,{},false,cfg);
        Hm20SkeletonAssociator::TestHook_ContState st;
        for(int fr=0;fr<N;++fr){
            SkeletonFrameResult r; std::array<bool,kNumMarkers> seen{};
            for(int m=0;m<20;++m){
                r.markers[size_t(m)].posWorld={sk[size_t(fr)].pos[m*3],sk[size_t(fr)].pos[m*3+1],sk[size_t(fr)].pos[m*3+2]};
                r.markers[size_t(m)].observed=V(fr,m); seen[size_t(m)]=V(fr,m); }
            if(snap) a.TestHook_snapToBoneLength(r,seen,st);
            if(cont) a.TestHook_smoothPredicted(r,seen,st);
            for(int m=0;m<20;++m) out[size_t(fr)][size_t(m)]=r.markers[size_t(m)].posWorld; }
    };
    auto score=[&](const char* nm,std::vector<std::array<Vec3,20>>& X){
        // 遮挡段【最后一帧】的预测 vs 恢复后第一帧的实测 = 预测误差
        std::vector<double> e;
        for(int m=5;m<20;++m) for(int fr=0;fr+1<N;++fr)
            if(!V(fr,m)&&V(fr+1,m)){
                Vec3 t{sk[size_t(fr+1)].pos[m*3],sk[size_t(fr+1)].pos[m*3+1],sk[size_t(fr+1)].pos[m*3+2]};
                e.push_back(norm(sub(X[size_t(fr)][size_t(m)],t)));
            }
        std::sort(e.begin(),e.end());
        std::printf("%-22s n=%-4zu 中位%6.2f  p75%6.2f  p90%6.2f  max%7.2f\n",
            nm,e.size(),e[e.size()/2],e[e.size()*3/4],e[e.size()*9/10],e.back());
    };
    std::vector<std::array<Vec3,20>> X;
    std::printf("预测误差 = 遮挡段最后一帧的预测位置 vs 恢复后的实测真值 (mm)\n");
    std::printf("骨长回正全程开启（它是纯收益）\n\n");
    // 扫 alpha：平滑越重越滞后
    for(double a:{1.0,0.8,0.6,0.4,0.25}){
        Hm20Config cfg; cfg.predictSmoothAlpha=a; cfg.recoverBlendDecay=0.88;
        X.clear(); X.resize(size_t(N));
        Hm20SkeletonAssociator A(nullptr,{},false,cfg);
        Hm20SkeletonAssociator::TestHook_ContState st;
        for(int fr=0;fr<N;++fr){
            SkeletonFrameResult r; std::array<bool,kNumMarkers> seen{};
            for(int m=0;m<20;++m){
                r.markers[size_t(m)].posWorld={sk[size_t(fr)].pos[m*3],sk[size_t(fr)].pos[m*3+1],sk[size_t(fr)].pos[m*3+2]};
                r.markers[size_t(m)].observed=V(fr,m); seen[size_t(m)]=V(fr,m); }
            A.TestHook_snapToBoneLength(r,seen,st);
            A.TestHook_smoothPredicted(r,seen,st);
            for(int m=0;m<20;++m) X[size_t(fr)][size_t(m)]=r.markers[size_t(m)].posWorld; }
        char nm[48]; std::snprintf(nm,sizeof(nm),"alpha=%.2f",a);
        score(nm,X);
    }
    return 0;
}
