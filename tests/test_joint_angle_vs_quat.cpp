// 同一组骨轴，分别走"关节角"和"四元数"两条路，看是不是恒等
#include "estimate/HandSkeletonAssociator.hpp"
#include "estimate/Hm20JointAngles.hpp"
#include <cstdio>
#include <cmath>
using namespace mocap::hm20;
using namespace mocap::hm20::detail;
static double qang(const Quat& q){return 2*std::acos(std::min(1.0,std::fabs(q[0])))*180.0/M_PI;}
static Quat relq(const Quat& p,const Quat& c){
    // p^-1 * c
    Quat pi{p[0],-p[1],-p[2],-p[3]};
    return Quat{pi[0]*c[0]-pi[1]*c[1]-pi[2]*c[2]-pi[3]*c[3],
                pi[0]*c[1]+pi[1]*c[0]+pi[2]*c[3]-pi[3]*c[2],
                pi[0]*c[2]-pi[1]*c[3]+pi[2]*c[0]+pi[3]*c[1],
                pi[0]*c[3]+pi[1]*c[2]-pi[2]*c[1]+pi[3]*c[0]};
}
int main(){
    int fail = 0;
    std::printf("食指：MCP 屈曲 F、外展 A，PIP 角 P。看两路输出是否恒等\n\n");
    std::printf("%-22s%12s%12s%12s%12s\n","设定(真实PIP)","PIP(角度路)","PIP(四元数)","MCP(角度路)","MCP(四元数)");
    auto ax=[](double f,double a){double fr=f*M_PI/180,ar=a*M_PI/180;
        return Vec3{std::cos(fr)*std::cos(ar),std::sin(ar),-std::sin(fr)*std::cos(ar)};};
    for(auto t : {std::array<double,3>{25,0,40}, {25,20,40}, {10,23,16}, {0,0,30}}){
        double F=t[0],A=t[1],P=t[2];
        SkeletonFrameResult r; r.valid=true; r.wristPoseValid=true;
        r.wristR={1,0,0, 0,1,0, 0,0,1};
        for(int m=0;m<20;++m){r.markers[size_t(m)].label=m;r.markers[size_t(m)].observed=true;}
        Vec3 root{40,0,0};
        // 【三段方向必须各不相同】上次就栽在这：让中节和远节共用一个方向，
        // dirProx 和 dirMid 平行，PIP 恒为 0，看着像"算不出来"。
        //   dirProx = pp->mp   dirMid = pp->dp   dirDist = mp->dp
        // 所以要让 mp、dp 各自沿不同方向排。
        // 近节骨方向 u0；中节骨相对它弯 P（这就是 PIP 的定义）；
        // 远节沿中节骨延长（DIP=0），这样 mp->dp 就是中节骨的方向。
        // 【按真实贴球几何造】球在指骨远端：
        //   anchor->pp = 近节骨(方向 u0)   pp->mp = 中节骨(u1)   mp->dp = 远节骨(u2)
        // PIP = u0 与 u1 的夹角 = P。DIP 设 15°。
        Vec3 u0=ax(F,A), u1=ax(F+P,A), u2=ax(F+P+15,A);
        Vec3 pp{root[0]+u0[0]*30,root[1]+u0[1]*30,root[2]+u0[2]*30};
        Vec3 mp{pp[0]+u1[0]*25,pp[1]+u1[1]*25,pp[2]+u1[2]*25};
        Vec3 dp{mp[0]+u2[0]*18,mp[1]+u2[1]*18,mp[2]+u2[2]*18};
        r.markers[8].posWorld=pp;r.markers[9].posWorld=mp;r.markers[10].posWorld=dp;
        // 【必须设 segSource】solveJointAngles 的 measured() 要求几何或 IK，
        // 默认 None 会让整根手指被 continue 掉，两路都返回 0 —— 那不是结论。
        for(int sg=0;sg<kNumSegments;++sg) r.segSource[size_t(sg)]=SegSource::Geometry;
        // 【anchor 必须设】不设的话 sProx 退回 pp->mp，那【就是中节骨】，
        // 跟 PIP 的第二条轴完全相同，夹角恒为 0 —— 测的根本不是真实路径。
        // 真实路径里 sProx = anchor->pp = 近节骨。
        std::array<Vec3,5> anchors{}; anchors[1]=root;   // 食指根关节
        std::array<Vec3,kNumSegments> py{};
        Hm20SkeletonAssociator::TestHook_computeSegmentQuatsA(r,py,anchors,true);
        JointAngleResult prev{}; WristHold hold{};
        auto ja=solveJointAngles(r,prev,hold);
        double pipA=ja.q[6]*180/M_PI;             // 食 PIP
        // 新版 solveJointAngles 把 PIP 统一成“弯曲量”：恒非负。
        // 否则 hingeSigned 在不同握拳循环里翻号，会把整指趋势翻过去。
        if (pipA < -1e-6) {
            std::printf("  [FAIL] PIP 出现负弯曲量：%.3f deg\n", pipA);
            ++fail;
        }
        double mcpF=ja.q[4]*180/M_PI, mcpAb=ja.q[5]*180/M_PI;
        double pipQ=0;
        double mcpQ=qang(relq(r.segQuat[0],r.segQuat[4]));
        double mcpAx=std::acos(std::cos(mcpF*M_PI/180)*std::cos(mcpAb*M_PI/180))*180/M_PI;
        char nm[48]; std::snprintf(nm,sizeof(nm),"F=%.0f A=%.0f P=%.0f",F,A,P);
        std::printf("%-22s%12.1f%12s%12.1f%12.1f\n",nm,pipA,"-",mcpAx,mcpQ);
    }
    std::printf("\n两列应该恒等；不等说明两条路对'骨轴'的取法不同\n");
    std::printf("%s\n", fail ? "有用例失败" : "全部通过");
    return fail ? 1 : 0;
}
