// 用【两段真实录制】验 C++ 实现，必须复现 Python 的数字
#include "estimate/HandSkeletonAssociator.hpp"
#include "record/PointCloudRecorder.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace mocap;
static std::vector<pcrec::SkeletonRec> loadSk(const char*p){
    std::ifstream f(p,std::ios::binary);
    std::vector<char> raw((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
    uint32_t hl; std::memcpy(&hl,raw.data()+12,4); size_t o=16+hl;
    std::vector<pcrec::SkeletonRec> v;
    while(o+5<=raw.size()){ uint8_t ct=uint8_t(raw[o]); uint32_t cl; std::memcpy(&cl,raw.data()+o+1,4);
        if(o+5+cl>raw.size())break;
        if(ct==uint8_t(pcrec::ChunkType::Skeleton)){pcrec::SkeletonRec s;
            std::memcpy(&s,raw.data()+o+5+8,sizeof(s)); v.push_back(s);}
        o+=5+cl;}
    return v;
}
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

    auto sk=loadSk(argv[1]);
    size_t N=sk.size();
    std::vector<double> mcp(N,NAN), ip(N,NAN); std::vector<char> vis(N,0);
    auto g=[&](const pcrec::SkeletonRec&s,int m){return std::array<double,3>{s.pos[m*3],s.pos[m*3+1],s.pos[m*3+2]};};
    for(size_t i=0;i<N;++i){ const auto&s=sk[i];
        if(!(s.observed[5]&&s.observed[6]))continue;
        auto P=g(s,5),M=g(s,6); std::array<double,3> c{0,0,0};
        for(int m=0;m<5;++m){auto q=g(s,m);for(int d=0;d<3;++d)c[size_t(d)]+=q[size_t(d)]/5;}
        auto nz=[](std::array<double,3> a){double n=std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
            return std::array<double,3>{a[0]/n,a[1]/n,a[2]/n};};
        auto sb=[](auto a,auto b){return std::array<double,3>{a[0]-b[0],a[1]-b[1],a[2]-b[2]};};
        auto u0=nz(sb(P,c)),u1=nz(sb(M,P));
        mcp[i]=std::acos(std::clamp(u0[0]*u1[0]+u0[1]*u1[1]+u0[2]*u1[2],-1.0,1.0));
        if(!s.observed[7])continue;
        auto D=g(s,7); auto u2=nz(sb(D,M));
        ip[i]=std::acos(std::clamp(u1[0]*u2[0]+u1[1]*u2[1]+u1[2]*u2[2],-1.0,1.0));
        vis[i]=1;
    }
    // 找遮挡段，逐帧 step()
    std::vector<double> eh,et;
    size_t st=SIZE_MAX;
    for(size_t i=0;i<N;++i){
        if(!vis[i]&&st==SIZE_MAX) st=i;
        if(vis[i]&&st!=SIZE_MAX){
            if(st>=1&&vis[st-1]&&!std::isnan(ip[i])){
                hm20::ThumbIpTracker T; T.sync(ip[st-1],mcp[st-1]);
                for(size_t j=st;j<=i;++j) if(!std::isnan(mcp[j])) T.step(mcp[j]);
                eh.push_back(std::fabs(ip[i]-ip[st-1]));
                et.push_back(std::fabs(ip[i]-T.angle()));
                std::printf("  遮挡 %2zu 帧  入口%5.1f° 真值出口%5.1f°   保持误差%6.1f°  跟随误差%6.1f°\n",
                    i-st, ip[st-1]*57.2958, ip[i]*57.2958, eh.back()*57.2958, et.back()*57.2958);
            }
            st=SIZE_MAX;
        }
    }
    if(eh.empty()){std::puts("  没有可评的遮挡段");return 0;}
    auto med=[](std::vector<double> v){std::sort(v.begin(),v.end());return v[v.size()/2]*57.2958;};
    auto mx =[](const std::vector<double>&v){return *std::max_element(v.begin(),v.end())*57.2958;};
    std::printf("\n  保持: 中位 %.1f° 最大 %.1f°    跟随MCP: 中位 %.1f° 最大 %.1f°   (%zu 段)\n",
        med(eh),mx(eh),med(et),mx(et),eh.size());
    return 0;
}
