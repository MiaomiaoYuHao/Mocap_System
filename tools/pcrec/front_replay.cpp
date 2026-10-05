// ===========================================================================
// front_replay.cpp —— 2D 光斑 -> 多视角聚类 -> 追踪 的离线重放 + 参数扫描
//
//   用法:
//     python tools/pcrec/make_config.py 录制.pcrec cfg.txt
//     g++ -std=c++2a -O2 -D_USE_MATH_DEFINES -I src tools/pcrec/front_replay.cpp -o build/front_replay.exe
//     ./front_replay cfg.txt 录制.pcrec                 # 基线(用录制参数)
//     ./front_replay cfg.txt 录制.pcrec --sweep maxSampson=0.002,0.003,0.005
//
// 只依赖纯数学头文件, 不链接 Qt / onnxruntime。
// ===========================================================================
#include "reconstruct/MultiViewCluster.hpp"
#include "reconstruct/IekfPointTracker.hpp"
#include "reconstruct/TemporalTracker.hpp"
#include "record/PointCloudRecorder.hpp"

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <cmath>

using namespace mocap;

// ---------------------------------------------------------------------------
// 极简 key=value 配置读取
// ---------------------------------------------------------------------------
struct Cfg {
    std::map<std::string,std::string> m;
    void load(const char* path){
        std::ifstream f(path); if(!f){std::fprintf(stderr,"打不开配置 %s\n",path); return;}
        std::string line;
        while(std::getline(f,line)){
            if(line.empty()||line[0]=='#') continue;
            auto e=line.find('=');
            if(e==std::string::npos) continue;
            m[line.substr(0,e)]=line.substr(e+1);
        }
    }
    bool has(const std::string&k) const{return m.count(k)!=0;}
    double d(const std::string&k,double def=0.0) const{auto it=m.find(k); if(it==m.end()) return def; return std::strtod(it->second.c_str(),nullptr);}
    int   i(const std::string&k,int def=0) const{auto it=m.find(k); if(it==m.end()) return def; return std::atoi(it->second.c_str());}
    bool  b(const std::string&k,bool def=false) const{auto it=m.find(k); if(it==m.end()) return def; return std::atoi(it->second.c_str())!=0;}
    std::string s(const std::string&k,const std::string&def="") const{auto it=m.find(k); if(it==m.end()) return def; return it->second;}
};

// ---------------------------------------------------------------------------
// 相机内参(只留去畸变需要的字段), 自己实现 undistortNormalize, 不依赖 Qt
// ---------------------------------------------------------------------------
struct Intr { double fx,fy,cx,cy,k1,k2,k3,p1,p2; };
static void undistortNormalize(const Intr& I, double px, double py, double& nx, double& ny){
    double xd=(px-I.cx)/I.fx, yd=(py-I.cy)/I.fy;
    double x=xd, y=yd;
    for(int it=0;it<8;++it){
        double r2=x*x+y*y;
        double ic=1.0/(1.0+I.k1*r2+I.k2*r2*r2+I.k3*r2*r2*r2);
        double dx=2.0*I.p1*x*y+I.p2*(r2+2.0*x*x);
        double dy=I.p1*(r2+2.0*y*y)+2.0*I.p2*x*y;
        x=(xd-dx)*ic; y=(yd-dy)*ic;
    }
    nx=x; ny=y;
}

// ---------------------------------------------------------------------------
// .pcrec 二进制读取(只取需要块)
// ---------------------------------------------------------------------------
struct CamFrame { int64_t ts=0; std::vector<pcrec::Blob2D> blobs; };
struct Rec {
    std::string header;
    std::map<int,std::vector<CamFrame>> cam;                 // camId -> 帧序列
    std::vector<std::pair<int64_t,std::vector<pcrec::ClusterRec>>> cluster;   // 在线聚类基线
    std::vector<std::pair<int64_t,std::vector<pcrec::Point3DRec>>> points3d;  // 在线追踪基线
    std::vector<std::pair<int64_t,float>> skelRmse;          // 手背 Kabsch 残差(在线骨架, mm)
    int64_t t0 = -1;                                          // 参考帧时基起点
};

static bool loadRec(const char* path, Rec& out){
    std::ifstream f(path,std::ios::binary); if(!f){std::fprintf(stderr,"打不开 %s\n",path);return false;}
    std::vector<char> raw((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
    if(raw.size()<16 || std::memcmp(raw.data(),"MCPCREC\0",8)!=0){std::fprintf(stderr,"不是 .pcrec\n");return false;}
    uint32_t ver=0,hlen=0; std::memcpy(&ver,raw.data()+8,4); std::memcpy(&hlen,raw.data()+12,4);
    out.header.assign(raw.data()+16,hlen);
    size_t off=16+hlen;
    while(off+5<=raw.size()){
        uint8_t ct=uint8_t(raw[off]); uint32_t clen=0; std::memcpy(&clen,raw.data()+off+1,4);
        size_t body=off+5; if(body+clen>raw.size()) break;
        const char* p=raw.data()+body; off=body+clen;
        if(ct==uint8_t(pcrec::ChunkType::CamBlobs)){
            uint32_t camId; int64_t ts; uint16_t n;
            std::memcpy(&camId,p,4); std::memcpy(&ts,p+4,8); std::memcpy(&n,p+12,2);
            CamFrame cf; cf.ts=ts; cf.blobs.resize(n);
            if(n) std::memcpy(cf.blobs.data(),p+14,size_t(n)*sizeof(pcrec::Blob2D));
            out.cam[int(camId)].push_back(std::move(cf));
        } else if(ct==uint8_t(pcrec::ChunkType::Cluster3D)){
            int64_t ts; uint16_t n; std::memcpy(&ts,p,8); std::memcpy(&n,p+8,2);
            std::vector<pcrec::ClusterRec> v(n);
            if(n) std::memcpy(v.data(),p+10,size_t(n)*sizeof(pcrec::ClusterRec));
            out.cluster.push_back({ts,std::move(v)});
        } else if(ct==uint8_t(pcrec::ChunkType::Points3D)){
            int64_t ts; uint16_t n; std::memcpy(&ts,p,8); std::memcpy(&n,p+8,2);
            std::vector<pcrec::Point3DRec> v(n);
            if(n) std::memcpy(v.data(),p+10,size_t(n)*sizeof(pcrec::Point3DRec));
            out.points3d.push_back({ts,std::move(v)});
        } else if(ct==uint8_t(pcrec::ChunkType::Skeleton)){
            int64_t ts; std::memcpy(&ts,p,8);
            pcrec::SkeletonRec sk; std::memcpy(&sk,p+8,sizeof(sk));
            if(sk.dorsumRmseMm >= 0.0f) out.skelRmse.push_back({ts, sk.dorsumRmseMm});
        }
    }
    if(!out.points3d.empty()) out.t0=out.points3d[0].first;
    return true;
}

// ---------------------------------------------------------------------------
// 聚类参数结构
// ---------------------------------------------------------------------------
struct ClusterParams {
    double maxSampson=0.01, maxReproj=0.01;
    int minSupport=2;
    bool useLmRefine=true, useVoting=true;
    double clusterChiSquare=9.21;
    double minRayAngle=0.0, ambiguityMargin=0.0;
    double lmHuberDelta=0.01; int lmMaxIters=15;
    double calibSigmaNorm=0.0;
    bool twoViewFallback=true; double twoViewMinRayAngle=10.0;
    double maxFinalResidual=-1.0;
};

static void loadClusterParams(const Cfg& c, ClusterParams& p){
    p.maxSampson=c.d("maxSampson",p.maxSampson);
    p.maxReproj=c.d("maxReproj",p.maxReproj);
    p.minSupport=c.i("minSupport",p.minSupport);
    p.useLmRefine=c.b("useLmRefine",p.useLmRefine);
    p.useVoting=c.b("useVoting",p.useVoting);
    p.clusterChiSquare=c.d("clusterChiSquare",p.clusterChiSquare);
    p.minRayAngle=c.d("minRayAngle",p.minRayAngle);
    p.ambiguityMargin=c.d("ambiguityMargin",p.ambiguityMargin);
    p.lmHuberDelta=c.d("lmHuberDelta",p.lmHuberDelta);
    p.lmMaxIters=c.i("lmMaxIters",p.lmMaxIters);
    p.calibSigmaNorm=c.d("calibSigmaNorm",p.calibSigmaNorm);
    p.twoViewFallback=c.b("twoViewFallback",p.twoViewFallback);
    p.twoViewMinRayAngle=c.d("twoViewMinRayAngle",p.twoViewMinRayAngle);
    p.maxFinalResidual=c.d("maxFinalResidual",p.maxFinalResidual);
}

static bool applyKey(ClusterParams& p, const std::string& k, double v){
    if(k=="maxSampson"){p.maxSampson=v;return true;}
    if(k=="maxReproj"){p.maxReproj=v;return true;}
    if(k=="minSupport"){p.minSupport=int(v);return true;}
    if(k=="useLmRefine"){p.useLmRefine=v!=0;return true;}
    if(k=="useVoting"){p.useVoting=v!=0;return true;}
    if(k=="clusterChiSquare"){p.clusterChiSquare=v;return true;}
    if(k=="minRayAngle"){p.minRayAngle=v;return true;}
    if(k=="ambiguityMargin"){p.ambiguityMargin=v;return true;}
    if(k=="lmHuberDelta"){p.lmHuberDelta=v;return true;}
    if(k=="lmMaxIters"){p.lmMaxIters=int(v);return true;}
    if(k=="calibSigmaNorm"){p.calibSigmaNorm=v;return true;}
    if(k=="twoViewFallback"){p.twoViewFallback=v!=0;return true;}
    if(k=="twoViewMinRayAngle"){p.twoViewMinRayAngle=v;return true;}
    if(k=="maxFinalResidual"){p.maxFinalResidual=v;return true;}
    return false;
}

// ---------------------------------------------------------------------------
// 结果指标
// ---------------------------------------------------------------------------
struct Stats {
    double nCluster=0, nTracked=0, pairStdMed=0, pairStdP90=0, pairCount=0;
    double nOver20=0;             // 超过20个已确认点(疑似幽灵)的帧占比
    double clusterMatchMed=0, clusterMatchP90=0, clusterMatchRate=0;
};

// 对拍: 复算 cluster tracks 跟在线 ClusterRec 对齐(贪心最近邻)
static void matchCluster(const std::vector<Track3D>& a, const std::vector<pcrec::ClusterRec>& b,
                         double& med, double& p90, double& rate){
    if(b.empty()||a.empty()){med=p90=rate=0;return;}
    std::vector<char> used(b.size(),0);
    std::vector<double> err; int matched=0;
    for(const auto& t:a){
        double best=1e18; int bi=-1;
        for(size_t j=0;j<b.size();++j){
            if(used[j]) continue;
            double dx=t.point[0]-double(b[j].x), dy=t.point[1]-double(b[j].y), dz=t.point[2]-double(b[j].z);
            double e=std::sqrt(dx*dx+dy*dy+dz*dz);
            if(e<best){best=e;bi=int(j);}
        }
        if(bi>=0 && best<15.0){used[size_t(bi)]=1; err.push_back(best); ++matched;}
    }
    if(err.empty()){med=p90=rate=0;return;}
    std::sort(err.begin(),err.end());
    med=err[err.size()/2];
    p90=err[size_t(std::min<size_t>(err.size()-1, err.size()*90/100))];
    rate=100.0*matched/b.size();
}

static double getv(const std::map<std::string,double>& ov, const Cfg& cfg, const std::string& k, double d){
    auto it=ov.find(k); return it!=ov.end() ? it->second : cfg.d(k,d);
}
static int geti(const std::map<std::string,double>& ov, const Cfg& cfg, const std::string& k, int d){
    auto it=ov.find(k); return it!=ov.end() ? int(it->second) : cfg.i(k,d);
}
static bool getb(const std::map<std::string,double>& ov, const Cfg& cfg, const std::string& k, bool d){
    auto it=ov.find(k); return it!=ov.end() ? (it->second!=0.0) : cfg.b(k,d);
}

static Stats run(Rec& rec, const Cfg& cfg, const ClusterParams& cp, const std::map<std::string,double>& ov, bool verbose){
    // 标定: camId 升序 -> idx
    std::vector<int> camIds;
    for(const auto& kv:rec.cam) camIds.push_back(kv.first);
    std::sort(camIds.begin(),camIds.end());
    const int nc=int(camIds.size());
    std::vector<int> camToIdx; camToIdx.resize(64,-1);
    for(int i=0;i<nc;++i) camToIdx[size_t(camIds[size_t(i)])]=i;

    std::vector<EpiMat3> R(nc); std::vector<EpiVec3> t(nc); std::vector<Intr> intr(nc);
    for(int i=0;i<nc;++i){
        Intr I; I.fx=cfg.d("cam."+std::to_string(i)+".fx",1.0); I.fy=cfg.d("cam."+std::to_string(i)+".fy",I.fx);
        I.cx=cfg.d("cam."+std::to_string(i)+".cx",0); I.cy=cfg.d("cam."+std::to_string(i)+".cy",0);
        I.k1=cfg.d("cam."+std::to_string(i)+".k1",0); I.k2=cfg.d("cam."+std::to_string(i)+".k2",0); I.k3=cfg.d("cam."+std::to_string(i)+".k3",0);
        I.p1=cfg.d("cam."+std::to_string(i)+".p1",0); I.p2=cfg.d("cam."+std::to_string(i)+".p2",0);
        intr[size_t(i)]=I;
        for(int k=0;k<9;++k) R[size_t(i)][size_t(k)]=cfg.d("cam."+std::to_string(i)+".R."+std::to_string(k), k%4==0?1.0:0.0);
        for(int k=0;k<3;++k) t[size_t(i)][size_t(k)]=cfg.d("cam."+std::to_string(i)+".t."+std::to_string(k),0.0);
    }

    // 追踪器
    const bool useIekf = getb(ov,cfg,"useIekfBackend",true);
    IekfPointTracker iekf(getv(ov,cfg,"assocDist",30.0), geti(ov,cfg,"maxMissed",10), geti(ov,cfg,"minHits",1));
    TemporalTracker tmp(getv(ov,cfg,"assocDist",30.0), geti(ov,cfg,"maxMissed",10), geti(ov,cfg,"minHits",1));
    if(useIekf){
        iekf.setPosProcessVar(getv(ov,cfg,"iekfPosProcessVar",0.01));
        iekf.setVelProcessVar(getv(ov,cfg,"iekfVelProcessVar",100.0));
        iekf.setUseMahalanobisGate(getb(ov,cfg,"iekfMahaGate",true));
        iekf.setChiSquareGate(getv(ov,cfg,"iekfChiSquareGate",16));
        iekf.setUseDualAnchorGate(getb(ov,cfg,"iekfDualAnchor",true));
        iekf.setUseCoastGhostSuppression(getb(ov,cfg,"iekfCoastSuppress",true));
        iekf.setVelCapGain(getv(ov,cfg,"iekfVelCapGain",0.5));
        iekf.setUseManeuverAdaptiveQ(getb(ov,cfg,"iekfManeuverQ",true));
        iekf.setMaxConfirmedTracks(geti(ov,cfg,"iekfMaxConfirmed",20));
        iekf.setUseDensityGate(getb(ov,cfg,"iekfDensityGate",true));
        iekf.setUseGlobalAssignment(getb(ov,cfg,"iekfGlobalAssign",true));
        double fx=0; for(int i=0;i<nc;++i) fx+= (intr[size_t(i)].fx+intr[size_t(i)].fy)*0.5; fx/=std::max(1,nc);
        double noise=getv(ov,cfg,"iekfDetectNoisePx",0.05); double sn=noise/std::max(1e-6,fx);
        iekf.setDefaultObsVar(sn*sn);
        iekf.setAssocPosFloorVar(getb(ov,cfg,"iekfUseAutoFloor",true) ? -1.0 : getv(ov,cfg,"iekfFloorVar",200.0));
    } else {
        tmp.setUseOptimalAssignment(getb(ov,cfg,"useHungarian",false));
        tmp.setUseAdaptiveAssocCap(getb(ov,cfg,"useAdaptiveCap",true));
        tmp.setAdaptiveCapMultiplier(getv(ov,cfg,"adaptiveCapMul",0.5));
        tmp.setUseMissedFrameRelax(getb(ov,cfg,"useMissedRelax",true));
        tmp.setRelaxGrowthPerMissedFrame(getv(ov,cfg,"relaxGrowth",1.6));
        tmp.setRelaxCapMultiplier(getv(ov,cfg,"relaxCapMul",3.0));
        tmp.setUseVelocitySmoothing(getb(ov,cfg,"useVelSmooth",false));
        tmp.setVelocitySmoothingAlpha(getv(ov,cfg,"velSmoothAlpha",0.5));
        tmp.setUseConstantAcceleration(getb(ov,cfg,"useConstAccel",false));
    }

    // 帧时基: 用 recorded points3d 的 refTs 序列(cluster 同 ts)
    std::vector<int64_t> frameTs;
    frameTs.reserve(rec.points3d.size());
    for(const auto& f:rec.points3d) frameTs.push_back(f.first);
    // 每个相机各自指针
    std::vector<size_t> ptr(nc,0);

    // 自适应 σ(重投影残差 EMA)。ov 里 adaptive=1 开启, kSamp/kRep/kFinal 为倍数。
    const bool adaptive = getb(ov,cfg,"adaptive",false);
    const double kSamp = getv(ov,cfg,"kSamp", 30.0);
    const double kRep  = getv(ov,cfg,"kRep", 30.0);
    const double kFin  = getv(ov,cfg,"kFinal", 10.0);
    const double sampFloor = getv(ov,cfg,"sampFloor", 0.001);
    const double repFloor  = getv(ov,cfg,"repFloor", 0.001);
    double emaSigma = 0.0;

    // 近似深度: 相机中心到世界原点的中位距离(手在标定板原点附近)
    double depth = 0;
    {
        std::vector<double> dd; dd.reserve(nc);
        for(int i=0;i<nc;++i){ double n=0; for(int k=0;k<3;++k) n+=t[size_t(i)][size_t(k)]*t[size_t(i)][size_t(k)]; dd.push_back(std::sqrt(n)); }
        std::sort(dd.begin(),dd.end()); depth = dd[dd.size()/2];
    }
    if(depth < 1.0) depth = 1.0;
    size_t skelPtr = 0;

    Stats st;
    double sumCluster=0,sumTracked=0,sumOver=0; int nfr=0;
    double matchMedSum=0,matchP90Sum=0,matchRateSum=0;
    // 对 tracker 输出按 id 收集位置
    std::map<int,std::vector<std::array<double,3>>> idpos;

    for(size_t k=0;k<frameTs.size();++k){
        int64_t ref=frameTs[k];
        std::vector<std::vector<std::array<double,2>>> obsPerCam(nc);
        for(int i=0;i<nc;++i){
            const auto& seq=rec.cam[camIds[size_t(i)]];
            if(seq.empty()){continue;}
            size_t p=ptr[size_t(i)];
            double best=std::fabs(double(seq[p].ts-ref));
            while(p+1<seq.size() && std::fabs(double(seq[p+1].ts-ref))<=best){
                ++p; best=std::fabs(double(seq[p].ts-ref));
            }
            ptr[size_t(i)]=p;
            auto& obs=obsPerCam[size_t(i)];
            obs.reserve(seq[p].blobs.size());
            for(const auto& b:seq[p].blobs){
                double nx,ny; undistortNormalize(intr[size_t(i)],double(b.x),double(b.y),nx,ny);
                obs.push_back({nx,ny});
            }
        }

        double effSamp=cp.maxSampson, effRep=cp.maxReproj, effFin=cp.maxFinalResidual;
        if(adaptive && emaSigma>0.0){
            // adaptiveMode=1: 纯倍数; =2: 只放宽不收紧(max(fixed, k*sigma))
            const int amode = geti(ov,cfg,"adaptiveMode",1);
            if(amode==2){
                effSamp = std::max(cp.maxSampson, kSamp*emaSigma);
                effRep  = std::max(cp.maxReproj,  kRep*emaSigma);
                effFin  = std::max(cp.maxFinalResidual, kFin*emaSigma);
            } else {
                effSamp = std::max(sampFloor, kSamp*emaSigma);
                effRep  = std::max(repFloor,  kRep*emaSigma);
                effFin  = std::max(0.0, kFin*emaSigma);
            }
        }
        const auto cluster = clusterMultiView(R,t,obsPerCam,
            effSamp,effRep,cp.useLmRefine,nullptr,cp.clusterChiSquare,cp.minSupport,
            cp.useVoting,cp.minRayAngle,cp.ambiguityMargin,cp.lmHuberDelta,cp.lmMaxIters,
            cp.calibSigmaNorm,cp.twoViewFallback,cp.twoViewMinRayAngle,effFin);
        {
            std::vector<double> rs;
            for(const auto& t:cluster.tracks) if(t.residual>0.0) rs.push_back(t.residual);
            double sigmaRep=0.0;
            if(!rs.empty()){ std::sort(rs.begin(),rs.end()); sigmaRep=rs[rs.size()/2]; }
            double kab=0.0;
            while(skelPtr+1 < rec.skelRmse.size() &&
                  std::fabs(double(rec.skelRmse[skelPtr+1].first-ref)) <= std::fabs(double(rec.skelRmse[skelPtr].first-ref)))
                ++skelPtr;
            if(skelPtr < rec.skelRmse.size()) kab = double(rec.skelRmse[skelPtr].second) / depth;
            double frameSigma = sigmaRep;
            if(kab > frameSigma) frameSigma = kab;   // 联合: 取更保守(更大)
            if(frameSigma > 0.0)
                emaSigma = (emaSigma<=0.0) ? frameSigma : (0.9*emaSigma + 0.1*frameSigma);
        }
        std::vector<Track3D> trk; trk.reserve(cluster.tracks.size());
        for(const auto& t:cluster.tracks) trk.push_back(t);
        const auto tracked = useIekf ? iekf.update(trk,R,t,obsPerCam,nullptr,ref)
                                     : tmp.update(trk,ref);
        sumCluster+=double(cluster.tracks.size());
        sumTracked+=double(tracked.size());
        if(tracked.size()>20) sumOver+=1.0;

        if(k<rec.cluster.size()){
            double a,b,c; matchCluster(trk,rec.cluster[k].second,a,b,c);
            matchMedSum+=a; matchP90Sum+=b; matchRateSum+=c;
        }
        // 收集刚体不变量需要的逐 id 位置(只取这一帧真实观测到的点)
        for(const auto& tp:tracked){
            if(tp.missedFrames==0) idpos[tp.id].push_back(tp.position);
        }
        ++nfr;
    }
    st.nCluster=sumCluster/nfr;
    st.nTracked=sumTracked/nfr;
    st.nOver20=100.0*sumOver/nfr;
    st.clusterMatchMed=matchMedSum/nfr; st.clusterMatchP90=matchP90Sum/nfr; st.clusterMatchRate=matchRateSum/nfr;

    // 刚体不变量: id 对距离标准差
    std::vector<int> ids; for(const auto& kv:idpos) if(kv.second.size()>=30) ids.push_back(kv.first);
    std::vector<double> stds;
    for(size_t a=0;a<ids.size();++a) for(size_t b=a+1;b<ids.size();++b){
        const auto& A=idpos[ids[a]]; const auto& B=idpos[ids[b]];
        // 时间对齐用最小长度
        size_t n=std::min(A.size(),B.size());
        if(n<30) continue;
        std::vector<double> d(n);
        for(size_t q=0;q<n;++q){double dx=A[q][0]-B[q][0],dy=A[q][1]-B[q][1],dz=A[q][2]-B[q][2]; d[q]=std::sqrt(dx*dx+dy*dy+dz*dz);}
        double mean=0; for(double v:d) mean+=v; mean/=n;
        double var=0; for(double v:d){double e=v-mean; var+=e*e;} var/=n;
        stds.push_back(std::sqrt(var));
    }
    if(!stds.empty()){
        std::sort(stds.begin(),stds.end());
        st.pairStdMed=stds[stds.size()/2];
        st.pairStdP90=stds[size_t(std::min<size_t>(stds.size()-1, stds.size()*90/100))];
        st.pairCount=double(stds.size());
    }
    return st;
}

static void printStats(const char* tag, const Stats& s){
    std::printf("%-26s nClus=%4.1f nTrk=%4.1f >20=%.0f%%  pairMed=%5.2fmm p90=%5.2fmm(n=%3.0f)  对拍:med=%5.2fmm p90=%5.2fmm rate=%.1f%%\n",
        tag,s.nCluster,s.nTracked,s.nOver20,s.pairStdMed,s.pairStdP90,s.pairCount,
        s.clusterMatchMed,s.clusterMatchP90,s.clusterMatchRate);
}

int main(int argc,char**argv){
    if(argc<3){std::puts("用法: front_replay cfg.txt file.pcrec [--sweep k=v1,v2,...]"); return 1;}
    Cfg cfg; cfg.load(argv[1]);
    Rec rec; if(!loadRec(argv[2],rec)) return 1;
    std::printf("帧 %zu  相机 %zu  协议 %s  后端 %s\n",
        rec.points3d.size(), rec.cam.size(), cfg.s("protocol","?").c_str(),
        cfg.b("useIekfBackend",true)?"IEKF":"Temporal");

    ClusterParams base; loadClusterParams(cfg,base);

    std::map<std::string,double> ov;   // 非聚类参数(tracker 等)的覆盖
    std::string sweepKey; std::vector<double> sweepVals;
    for(int i=3;i<argc;++i){
        std::string a=argv[i];
        if(a=="--set" && i+1<argc){
            std::string kv=argv[++i]; auto e=kv.find('=');
            if(e==std::string::npos) continue;
            std::string k=kv.substr(0,e); double v=std::strtod(kv.c_str()+e+1,nullptr);
            if(!applyKey(base,k,v)) ov[k]=v;
        } else if(a=="--sweep" && i+1<argc){
            std::string kv=argv[++i]; auto e=kv.find('=');
            if(e==std::string::npos) continue;
            sweepKey=kv.substr(0,e);
            const char* q=kv.c_str()+e+1;
            while(*q){sweepVals.push_back(std::strtod(q,const_cast<char**>(&q))); if(*q==',')++q; else break;}
        }
    }
    bool sweepIsCluster = false;
    if(!sweepKey.empty()){ ClusterParams t=base; sweepIsCluster = applyKey(t, sweepKey, sweepVals.empty()?0.0:sweepVals[0]); }

    std::printf("\n【基线】用录制参数复算(跟在线 ClusterRec 对拍, 越接近说明重放越忠实)\n");
    printStats("基线(录制参数)", run(rec,cfg,base,ov,true));

    if(!sweepKey.empty()){
        std::printf("\n【扫描】%s\n",sweepKey.c_str());
        for(double v:sweepVals){
            char tag[64]; std::snprintf(tag,sizeof(tag),"  %s=%g",sweepKey.c_str(),v);
            if(sweepIsCluster){
                ClusterParams c=base; applyKey(c,sweepKey,v);
                printStats(tag,run(rec,cfg,c,ov,false));
            } else {
                auto o=ov; o[sweepKey]=v;
                printStats(tag,run(rec,cfg,base,o,false));
            }
        }
    }
    return 0;
}
