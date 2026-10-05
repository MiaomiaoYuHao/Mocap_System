#include "record/PointCloudRecorder.hpp"
#include <cstdio>
#include <cmath>
using namespace mocap::pcrec;
int main(){
    Recorder R;
    // 用真机那组手背模板，这样复算工具能真的解出刚体
    const char* hdr = "{\"format\":\"mocap_pc_record\",\"protocol\":\"rigid_motion\","
      "\"note\":\"roundtrip\","
      "\"template\":{\"backMarkers\":[[-20.1,13.4,0.7],[12.1,25.2,1.4],"
      "[18.7,2.5,-3.1],[10.7,-25.0,2.4],[-21.4,-16.2,-1.4]]},"
      "\"params\":{\"thumbPronation0\":-1.46,\"thumbPronationContrast\":3.1,"
      "\"thumbRollOffsetRad\":-3.14159}}";
    if(!R.start("/tmp/rt.pcrec", hdr)) { std::puts("start fail"); return 1; }
    for(int f=0; f<300; ++f){
        int64_t ts = 1000000LL*f*8;
        std::vector<Blob2D> bs;
        for(int i=0;i<20;++i) bs.push_back({100.f+i*7.f, 200.f+f*0.3f, 0,0,0, 42.f, 210.f, 7,7});
        for(int c=0;c<4;++c) R.writeCamBlobs(uint32_t(c), ts, bs);

        // 手背 5 点按真机模板摆，整体做刚体平移；其余 15 点随便放
        const double BK[5][3]={{-20.1,13.4,0.7},{12.1,25.2,1.4},{18.7,2.5,-3.1},
                               {10.7,-25.0,2.4},{-21.4,-16.2,-1.4}};
        const double ox=200+f*0.2, oy=-30+f*0.1, oz=400;
        std::vector<Point3DRec> pts;
        for(int i=0;i<20;++i){ Point3DRec q;
            if(i<5){ q.x=float(BK[i][0]+ox); q.y=float(BK[i][1]+oy); q.z=float(BK[i][2]+oz); }
            else   { q.x=float(ox+30+i*6); q.y=float(oy+i*3); q.z=float(oz+10); }
            q.id=i; q.usedViews=3; q.residualMm=0.4f; pts.push_back(q);}
        R.writePoints3D(ts, pts);

        std::vector<ClusterRec> cl;
        for(int i=0;i<20;++i){ ClusterRec c; c.x=float(i*10); c.residualPx=0.6f;
            c.nSupport=3; c.supportMask=0b1011; c.verified=1; cl.push_back(c);}
        R.writeCluster(ts, cl);

        std::vector<std::pair<int,std::array<double,3>>> cand;
        for(int i=0;i<20;++i) cand.push_back({i,{double(pts[size_t(i)].x),
            double(pts[size_t(i)].y),double(pts[size_t(i)].z)}});
        R.writeAssocInput(ts, 23, cand);

        NetRawRec nr; nr.tsNs=ts; nr.nCand=20; nr.hasPose=1; nr.hasSegR=1;
        nr.scale=95.f; nr.handLogit=4.2f;
        for(int i=0;i<60;++i) nr.pos[i]=float(i);
        std::vector<NetTopK> tk(20);
        for(int i=0;i<20;++i){ tk[i].label[0]=int16_t(i); tk[i].prob[0]=0.9f;
            tk[i].label[1]=20; tk[i].prob[1]=0.05f; }
        R.writeNetRaw(nr, tk);

        SkeletonRec s; s.valid=1; s.wristPoseValid=1; s.dorsumRmseMm=0.83f;
        for(int i=0;i<20;++i){ s.observed[i]=uint8_t(i!=7);
            s.pos[i*3+0]=pts[size_t(i)].x; s.pos[i*3+1]=pts[size_t(i)].y;
            s.pos[i*3+2]=pts[size_t(i)].z; }
        R.writeSkeleton(ts, s);

        Hm20DiagRec d; d.wallNs=ts+500; d.frameTsNs=ts;
        // 前 60 帧模拟"冷启动歧义拒解"，之后锁上
        d.dorsumReason = uint8_t(f<60 ? 3 : 0);
        d.dorsumInliers = uint8_t(f<60 ? 0 : 5);
        d.dorsumByHistory = uint8_t(f>=60 && (f%7==0));
        d.dorsumSelfAmbMm=1.769f; d.dorsumMarginMm=1.8f; d.dorsumRmseMm=0.83f;
        d.handPanelIsRight=1; d.handAutoDetected=0; d.handAutoIsRight=1;  // 故意不一致
        d.handSignMm=2.1f; d.thumbFixSkip=7; d.thumbPronation0=-1.46f;
        d.thumbPronationContrast=3.1f; d.thumbPronationFitted=1;
        for(int i=0;i<20;++i) d.sourcePointId[i]= (i==7? -1 : 1000+i);
        // 模拟手背标签在两个物理点之间来回翻
        if(f>=60 && (f/13)%2==1){ d.sourcePointId[0]=1004; d.sourcePointId[4]=1000; }
        d.fingerIkValid[0]=0; for(int i=1;i<5;++i) d.fingerIkValid[i]=1;
        d.fingerIkRmseMm[0]=14.2f; for(int i=1;i<5;++i) d.fingerIkRmseMm[i]=3.1f;
        for(int g=0;g<16;++g) d.segSource[g]= (g>=1&&g<=3)?1:2;
        d.latencyMs=6.4f; d.candidateCount=23; d.observedJoints=14;
        R.writeHm20Diag(d);

        if(f%120==0) R.writeClockSync(ts, ts+7777);
    }
    CamSettingsRec cs; cs.camId=0; cs.width=1280; cs.height=800; cs.exposureUs=300;
    cs.gain=8; cs.fps=120; cs.threshold=180; cs.minArea=4; cs.maxArea=400;
    R.writeCamSettings(cs);
    R.writeMark(0,1,"段开始");
    R.stop();
    std::printf("写完 /tmp/rt.pcrec  %llu 字节  丢块 %llu\n",
        (unsigned long long)R.bytesWritten(),(unsigned long long)R.droppedChunks());
    return 0;
}
