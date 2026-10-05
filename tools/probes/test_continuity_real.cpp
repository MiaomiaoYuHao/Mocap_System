// 用真实 .pcrec 驱动【关联器里真正的那份代码】，确认跟离线扫描结果一致
#include "estimate/HandSkeletonAssociator.hpp"
#include "record/PointCloudRecorder.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <algorithm>
using namespace mocap;
using namespace mocap::hm20;
using namespace mocap::hm20::detail;
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
    // 直接驱动 TestHook：把录制的位置/可见性喂进 smoothPredicted
    std::vector<std::array<Vec3,20>> out; out.resize(size_t(N));
    Hm20SkeletonAssociator assoc(nullptr, {}, false, Hm20Config{});
    Hm20SkeletonAssociator::TestHook_ContState st;
    for(int fr=0;fr<N;++fr){
        SkeletonFrameResult r;
        std::array<bool,kNumMarkers> seen{};
        for(int m=0;m<20;++m){
            r.markers[size_t(m)].posWorld={sk[size_t(fr)].pos[m*3],sk[size_t(fr)].pos[m*3+1],sk[size_t(fr)].pos[m*3+2]};
            r.markers[size_t(m)].observed = sk[size_t(fr)].observed[m]!=0;
            seen[size_t(m)] = r.markers[size_t(m)].observed;
        }
        assoc.TestHook_smoothPredicted(r,seen,st);
        for(int m=0;m<20;++m) out[size_t(fr)][size_t(m)]=r.markers[size_t(m)].posWorld;

    }
    auto V=[&](int fr,int m){return sk[size_t(fr)].observed[m]!=0;};
    std::vector<double> T,B,Nn; double mx=0,es=0; int en=0;
    for(int m=0;m<20;++m){
        bool anyOcc=false; for(int fr=0;fr<N;++fr) if(!V(fr,m)) anyOcc=true;
        if(!anyOcc) continue;
        for(int fr=0;fr+1<N;++fr){
            double d=norm(sub(out[size_t(fr+1)][size_t(m)],out[size_t(fr)][size_t(m)]));
            if(V(fr,m)&&!V(fr+1,m)) T.push_back(d);
            else if(!V(fr,m)&&V(fr+1,m)) B.push_back(d);
            else if(V(fr,m)&&V(fr+1,m)) Nn.push_back(d);
            mx=std::max(mx,d);
        }
        for(int fr=0;fr<N;++fr) if(V(fr,m)){
            Vec3 p{sk[size_t(fr)].pos[m*3],sk[size_t(fr)].pos[m*3+1],sk[size_t(fr)].pos[m*3+2]};
            es+=norm(sub(out[size_t(fr)][size_t(m)],p)); ++en; }
    }
    auto med=[](std::vector<double> v){std::sort(v.begin(),v.end());return v.empty()?0.0:v[v.size()/2];};
    std::printf("=== 关联器里真正那份代码，跑你这段素材 ===\n");
    std::printf("  正常帧位移   %.2f mm\n",med(Nn));
    std::printf("  转遮挡跳变   %.2f mm   (原始 4.61)\n",med(T));
    std::printf("  恢复跳变     %.2f mm   (原始 7.14)\n",med(B));
    std::printf("  最大跳变     %.1f mm   (原始 39.3)\n",mx);
    std::printf("  实测帧偏差   %.2f mm\n",en?es/en:0.0);
    // 【门限按"两个判据都不亏"定，不是按"跳变最小"定】
    //
    // 转遮挡放到 4.0 是有意的：把它压到 2.5 需要 alpha=0.4，而那会让
    // 预测的 p90 误差从 15.34 涨到 18.59（差 21%）—— 输出更平但更不准。
    // 平滑的滞后在遮挡期手动得快时跟不上，那不是改进，是把问题换了个形式。
    //
    // 现在这组（alpha=0.8）：转遮挡 4.61->3.38、恢复 7.14->0.83、
    // 最大 39.3->23.1，同时预测误差中位和 p90 都【略优于】不处理。
    const bool ok = med(T) < 4.0 && med(B) < 1.5 && mx < 30.0 && (en?es/en:0) < 1.2;
    std::printf("\n  %s\n", ok?"✅ 连续性达标":"❌ 未达标");
    return ok?0:1;
}
