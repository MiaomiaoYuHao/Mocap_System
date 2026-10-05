// 遮挡时：冻结上一帧 vs 用预测值。判据 = 恢复瞬间离实测真值多远
#include "estimate/Hm20JointAngles.hpp"
#include "record/PointCloudRecorder.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
using namespace mocap; using namespace mocap::hm20;
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
    auto mk=[&](int fr){
        SkeletonFrameResult r; r.valid=sk[size_t(fr)].valid!=0;
        r.wristPoseValid=sk[size_t(fr)].wristPoseValid!=0;
        for(int m=0;m<20;++m){ r.markers[size_t(m)].label=m;
            r.markers[size_t(m)].posWorld={sk[size_t(fr)].pos[m*3],sk[size_t(fr)].pos[m*3+1],sk[size_t(fr)].pos[m*3+2]};
            r.markers[size_t(m)].observed=sk[size_t(fr)].observed[m]!=0; }
        for(int i=0;i<9;++i) r.wristR[size_t(i)]=sk[size_t(fr)].wristR[i];
        for(int i=0;i<3;++i) r.wristT[size_t(i)]=sk[size_t(fr)].wristT[i];
        for(int g=0;g<16;++g){for(int i=0;i<4;++i) r.segQuat[size_t(g)][size_t(i)]=sk[size_t(fr)].segQuat[g*4+i];
            r.segSource[size_t(g)]=SegSource(sk[size_t(fr)].segSource[g]);}
        return r; };
    auto isGeo=[&](int fr,int seg){ return sk[size_t(fr)].segSource[seg]==uint8_t(SegSource::Geometry)
                                        || sk[size_t(fr)].segSource[seg]==uint8_t(SegSource::Ik); };
    auto run=[&](bool relax, std::vector<std::array<double,16>>& out){
        out.clear(); out.resize(size_t(N));
        JointAngleResult prev{}; WristHold hold{};
        for(int fr=0;fr<N;++fr){
            auto r=mk(fr);
            auto ja=solveJointAngles(r,prev,hold,24,false,relax);
            prev=ja; out[size_t(fr)]=ja.q; }
    };
    std::vector<std::array<double,16>> H,P;
    run(false,H);   // 冻结（现状）
    run(true ,P);   // 用预测
    // 恢复瞬间：某指从"近节或中节非几何"变成"两段都几何"
    std::vector<double> eH,eP; int nseg=0;
    for(int fg=0;fg<5;++fg){
        const int sp=1+3*fg, sm=sp+1;
        const int base=(fg==0)?0:4+(fg-1)*3, nd=(fg==0)?4:3;
        for(int fr=1;fr<N;++fr){
            const bool now=isGeo(fr,sp)&&isGeo(fr,sm);
            const bool bef=isGeo(fr-1,sp)&&isGeo(fr-1,sm);
            if(!(now&&!bef)) continue;
            ++nseg;
            // 恢复后第一帧是实测真值：H 和 P 在这一帧都等于真值，
            // 所以比【上一帧】的输出离它多远
            for(int k=0;k<nd;++k){
                const size_t i=size_t(base+k);
                const double truth=H[size_t(fr)][i];   // 两条路在实测帧一致
                const double dh=std::fabs(H[size_t(fr-1)][i]-truth)*180/M_PI;
                const double dp=std::fabs(P[size_t(fr-1)][i]-truth)*180/M_PI;
                eH.push_back(dh); eP.push_back(dp);
                if(dh>90||dp>90)
                    std::printf("    大误差 指%d 维%zu 帧%d: 冻结%.0f° 预测%.0f° 真值%.0f°\n",
                        fg,i,fr,dh,dp,truth*180/M_PI);
            }
        }
    }
    auto st=[&](const char* nm, std::vector<double> v){
        std::sort(v.begin(),v.end());
        std::printf("  %-12s 中位%6.2f°  p75%6.2f°  p90%6.2f°  max%7.1f°\n",
            nm,v[v.size()/2],v[v.size()*3/4],v[v.size()*9/10],v.back()); };
    std::printf("恢复瞬间：遮挡最后一帧的关节角 vs 恢复后的实测真值\n");
    std::printf("%d 次恢复，%zu 个关节样本\n\n",nseg,eH.size());
    st("冻结上一帧",eH);
    st("用预测值",eP);
    return 0;
}
