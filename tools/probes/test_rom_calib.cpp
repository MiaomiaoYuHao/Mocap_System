// 用真实素材跑一遍 ROM 标定，看每一维实际采到多少
#include "estimate/HandSkeletonAssociator.hpp"
#include "estimate/Hm20JointAngles.hpp"
#include "record/PointCloudRecorder.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <cmath>
using namespace mocap; using namespace mocap::hm20;
static const char* NM[16]={"拇CMC屈","拇CMC展","拇MCP","拇IP",
 "食MCP屈","食MCP展","食PIP","中MCP屈","中MCP展","中PIP",
 "无MCP屈","无MCP展","无PIP","小MCP屈","小MCP展","小PIP"};
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
    RomCalibrator rom; JointAngleResult prev{}; WristHold hold{};
    int nvalid=0;
    std::array<int,16> nsamp{};
    for(const auto& s : sk){
        SkeletonFrameResult r; r.valid=s.valid!=0; r.wristPoseValid=s.wristPoseValid!=0;
        for(int m=0;m<20;++m){
            r.markers[size_t(m)].label=m;
            r.markers[size_t(m)].posWorld={s.pos[m*3],s.pos[m*3+1],s.pos[m*3+2]};
            r.markers[size_t(m)].observed=s.observed[m]!=0; }
        for(int i=0;i<9;++i) r.wristR[size_t(i)]=s.wristR[i];
        for(int i=0;i<3;++i) r.wristT[size_t(i)]=s.wristT[i];
        for(int g=0;g<16;++g){for(int i=0;i<4;++i) r.segQuat[size_t(g)][size_t(i)]=s.segQuat[g*4+i];
            r.segSource[size_t(g)]=SegSource(s.segSource[g]);}
        auto ja=solveJointAngles(r,prev,hold,24,false,true);  // ROM 放宽
        prev=ja;
        for(int f2=0;f2<5;++f2) if(ja.fingerValid[size_t(f2)]) ++nvalid;
        rom.observe(ja);
    }
    // 每一维攒了多少样本（finish 里 <30 直接置零）
    std::printf("每维样本数（finish 要求 >=30）:\n");
    for(int f2=0;f2<5;++f2){
        const int base=(f2==0)?0:4+(f2-1)*3, nd=(f2==0)?4:3;
        std::printf("  指%d: ",f2);
        for(int k=0;k<nd;++k) std::printf("%s=%d ",NM[base+k],rom.sampleCount(base+k));
        std::printf("\n");
    }
    std::printf("observe 调用次数 %d\n\n", rom.samples());
    const bool ok = rom.finish();
    std::printf("素材 %zu 帧，逐指有效累计 %d 指帧\n\n", sk.size(), nvalid);
    std::printf("%-10s%10s%10s%12s%10s\n","维","lo(度)","hi(度)","行程(度)","达标?");
    const auto& lim=jointLimits();
    int okc=0;
    for(int i=0;i<16;++i){
        double d=(rom.hi(i)-rom.lo(i))*180/M_PI;
        bool pass = (rom.hi(i)-rom.lo(i))>=0.35;
        bool isAbd=(i==1||i==5||i==8||i==11||i==14);
        if(!isAbd && pass) ++okc;
        std::printf("%-10s%10.1f%10.1f%12.1f%10s%s\n",NM[i],
            rom.lo(i)*180/M_PI, rom.hi(i)*180/M_PI, d,
            pass?"是":"否", isAbd?"  (外展，不计)":"");
    }
    std::printf("\n屈曲维达标 %d/11（需要>=8）   finish()=%s\n",okc,ok?"成功":"失败");
    std::printf("coverage = %.0f%%   外展 coverage = %.0f%%\n",
        100*rom.coverage(), 100*rom.abductionCoverage());
    return 0;
}
