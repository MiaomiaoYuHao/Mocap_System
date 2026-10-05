// 用你的两段真实录制验 C++ 实现，跟 Python 的数对不对得上
#include "estimate/HandSkeletonAssociator.hpp"
#include "record/PointCloudRecorder.hpp"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace mocap;
using V3=std::array<double,3>;
static V3 cr(const V3&a,const V3&b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
static double dt3(const V3&a,const V3&b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
static V3 nz(V3 a){double n=std::sqrt(dt3(a,a));return {a[0]/n,a[1]/n,a[2]/n};}
struct Fr{ bool vis,vp; double ip; V3 u1; };
static std::vector<Fr> load(const char*f){
    std::ifstream in(f,std::ios::binary);
    std::vector<char> raw((std::istreambuf_iterator<char>(in)),std::istreambuf_iterator<char>());
    uint32_t hl; std::memcpy(&hl,raw.data()+12,4); size_t off=16+hl;
    std::vector<Fr> out;
    while(off+5<=raw.size()){
        uint8_t ct=uint8_t(raw[off]); uint32_t cl; std::memcpy(&cl,raw.data()+off+1,4);
        if(off+5+cl>raw.size())break;
        if(ct==uint8_t(pcrec::ChunkType::Skeleton)){
            pcrec::SkeletonRec s; std::memcpy(&s,raw.data()+off+5+8,sizeof(s));
            Fr fr{}; fr.vp=s.wristPoseValid!=0; fr.vis=fr.vp&&s.observed[7];
            if(fr.vp){
                auto L=[&](int m){ V3 g{s.pos[m*3]-s.wristT[0],s.pos[m*3+1]-s.wristT[1],s.pos[m*3+2]-s.wristT[2]};
                    return V3{s.wristR[0]*g[0]+s.wristR[3]*g[1]+s.wristR[6]*g[2],
                              s.wristR[1]*g[0]+s.wristR[4]*g[1]+s.wristR[7]*g[2],
                              s.wristR[2]*g[0]+s.wristR[5]*g[1]+s.wristR[8]*g[2]};};
                V3 P=L(5),M=L(6);
                fr.u1=nz(V3{M[0]-P[0],M[1]-P[1],M[2]-P[2]});
                if(fr.vis){ V3 D=L(7); V3 u2=nz(V3{D[0]-M[0],D[1]-M[1],D[2]-M[2]});
                    fr.ip=std::acos(std::clamp(dt3(fr.u1,u2),-1.0,1.0)); }
            }
            out.push_back(fr);
        }
        off+=5+cl;
    }
    return out;
}
int main(int argc,char**argv){
    for(int q=1;q<argc;++q){
        auto F=load(argv[q]); size_t n=F.size();
        std::vector<V3> OM(n,{0,0,0});
        for(size_t i=1;i<n;++i) if(F[i].vp&&F[i-1].vp) OM[i]=cr(F[i-1].u1,F[i].u1);
        // 可见段
        std::vector<std::pair<int,int>> runs; int st=-1;
        for(size_t i=0;i<n;++i){ if(F[i].vis&&st<0)st=int(i);
            if(!F[i].vis&&st>=0){ if(int(i)-1-st>=12)runs.push_back({st,int(i)-1}); st=-1;} }
        if(st>=0&&int(n)-1-st>=12) runs.push_back({st,int(n)-1});
        std::printf("\n=== %s   %zu 帧, 可见段 %zu ===\n",
            strrchr(argv[q],'/')+1, n, runs.size());
        for(int N : {5,10,20}){
            std::vector<double> eh,es; int wins=0,tot=0;
            for(int k=0;k<5;++k){
                hm20::ThumbIpSwing sw;
                for(size_t ri=0;ri<runs.size();++ri){ if(int(ri)%5==k)continue;
                    for(int i=runs[ri].first+1;i<=runs[ri].second;++i)
                        if(F[i].vis&&F[i-1].vis) sw.observe(OM[size_t(i)],F[i].ip-F[i-1].ip); }
                if(!sw.ready())continue;
                std::vector<double> fh,fs;
                for(size_t ri=0;ri<runs.size();++ri){ if(int(ri)%5!=k)continue;
                    for(int s0=runs[ri].first+2;s0<runs[ri].second-N;++s0){
                        if(!F[s0].vis||!F[s0+N].vis)continue;
                        double tru=F[s0+N].ip;
                        if(std::fabs(tru-F[s0].ip)*57.3<3.0)continue;
                        double p=F[s0].ip;
                        for(int t=1;t<=N;++t) p+=sw.step(OM[size_t(s0+t)]);
                        p=std::clamp(p,-0.26,1.57);
                        fh.push_back(std::fabs(tru-F[s0].ip)); fs.push_back(std::fabs(tru-p)); } }
                if(fh.size()>=8){ ++tot;
                    auto md=[](std::vector<double> v){std::sort(v.begin(),v.end());return v[v.size()/2];};
                    if(md(fs)<md(fh))++wins; }
                eh.insert(eh.end(),fh.begin(),fh.end()); es.insert(es.end(),fs.begin(),fs.end());
            }
            if(eh.size()<10){std::printf("  N=%-3d 样本不足\n",N);continue;}
            auto md=[](std::vector<double> v){std::sort(v.begin(),v.end());return v[v.size()/2]*57.2958;};
            std::printf("  N=%-3d 样本%4zu   保持 %5.1f°   摆动积分 %5.1f°   改善 %3.0f%%   赢%d/%d折\n",
                N,eh.size(),md(eh),md(es),100*(md(eh)-md(es))/md(eh),wins,tot);
        }
    }
    return 0;
}
