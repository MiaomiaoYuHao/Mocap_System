// 处理之后的预测位置合不合理：骨长、解剖角、跑飞、越界
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
    auto run=[&](bool on, std::vector<std::array<Vec3,20>>& out){
        out.clear(); out.resize(size_t(N));
        Hm20Config cfg; if(!on){cfg.predictSmoothAlpha=1.0;cfg.recoverBlendDecay=0.0;}
        Hm20SkeletonAssociator a(nullptr,{},false,cfg);
        Hm20SkeletonAssociator::TestHook_ContState st;
        for(int fr=0;fr<N;++fr){
            SkeletonFrameResult r; std::array<bool,kNumMarkers> seen{};
            for(int m=0;m<20;++m){
                r.markers[size_t(m)].posWorld={sk[size_t(fr)].pos[m*3],sk[size_t(fr)].pos[m*3+1],sk[size_t(fr)].pos[m*3+2]};
                r.markers[size_t(m)].observed=sk[size_t(fr)].observed[m]!=0;
                seen[size_t(m)]=r.markers[size_t(m)].observed; }
            a.TestHook_snapToBoneLength(r,seen,st);      // 骨长回正
            a.TestHook_smoothPredicted(r,seen,st);
            for(int m=0;m<20;++m) out[size_t(fr)][size_t(m)]=r.markers[size_t(m)].posWorld; }
    };
    std::vector<std::array<Vec3,20>> A,B; run(false,A); run(true,B);
    auto V=[&](int fr,int m){return sk[size_t(fr)].observed[m]!=0;};
    auto rep=[&](const char* nm, std::vector<std::array<Vec3,20>>& X){
        // 骨长（只看遮挡帧）
        std::vector<double> blf[5]; int outlier=0, tot=0;
        std::vector<double> pipv;
        for(int fr=0;fr<N;++fr) for(int fg=0;fg<5;++fg){
            int pp=5+3*fg,mp=pp+1,dp=pp+2;
            if(V(fr,mp)&&V(fr,dp)) continue;         // 只看含预测点的
            double L1=norm(sub(X[size_t(fr)][size_t(mp)],X[size_t(fr)][size_t(pp)]));
            double L2=norm(sub(X[size_t(fr)][size_t(dp)],X[size_t(fr)][size_t(mp)]));
            ++tot;
            if(L1<8||L1>60||L2<5||L2>50) ++outlier;
            blf[fg].push_back(L2);
            // PIP 角
            Vec3 u0=sub(X[size_t(fr)][size_t(mp)],X[size_t(fr)][size_t(pp)]);
            Vec3 u1=sub(X[size_t(fr)][size_t(dp)],X[size_t(fr)][size_t(mp)]);
            double n0=norm(u0),n1=norm(u1);
            if(n0>1e-6&&n1>1e-6)
                pipv.push_back(std::acos(std::clamp(dot(u0,u1)/(n0*n1),-1.0,1.0))*180/M_PI);
        }
        std::sort(pipv.begin(),pipv.end());
        // 【按指分开】五根手指骨长本来就不同（mdLen 实测 20.4~28.1mm），
        // 混在一起统计出来的"离散度"大部分是指间差异，不是抖动。
        double cvsum=0; int cvn=0;
        for(int fg=0;fg<5;++fg){
            if(blf[fg].size()<20) continue;
            std::sort(blf[fg].begin(),blf[fg].end());
            double med=blf[fg][blf[fg].size()/2];
            double p5=blf[fg][blf[fg].size()/20], p95=blf[fg][blf[fg].size()*19/20];
            cvsum += (p95-p5)/std::max(med,1e-6); ++cvn;
        }
        // 离手背质心的距离（跑飞检测）
        double maxR=0;
        for(int fr=0;fr<N;++fr){
            Vec3 c{0,0,0};
            for(int k=0;k<5;++k){c[0]+=X[size_t(fr)][size_t(k)][0]/5;c[1]+=X[size_t(fr)][size_t(k)][1]/5;c[2]+=X[size_t(fr)][size_t(k)][2]/5;}
            for(int m=5;m<20;++m) if(!V(fr,m))
                maxR=std::max(maxR,norm(sub(X[size_t(fr)][size_t(m)],c)));
        }
        std::printf("%-8s 骨长离散(逐指 p5~p95 / 中位)%6.1f%% | 离谱%3d/%d | PIP中位%5.1f p99%6.1f | 离手背最远%6.1f\n",
            nm, 100.0*cvsum/std::max(cvn,1), outlier, tot,
            pipv[pipv.size()/2], pipv[pipv.size()*99/100], maxR);
    };
    std::printf("只统计【含预测点】的手指-帧\n\n");
    rep("处理前",A); rep("处理后",B);
    std::printf("\n（手长约 185mm，指节球间距正常 15~30mm；PIP 解剖上限约 110°）\n");
    return 0;
}
