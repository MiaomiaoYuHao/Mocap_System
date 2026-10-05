#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
#include <cmath>
using namespace mocap::hm20;
using namespace mocap::hm20::detail;
static double twistDeg(const Quat& qa, const Quat& qb){
    // A^T B 的扭转分量（绕 A 的 x 轴）
    auto m=[](const Quat&q){ double w=q[0],x=q[1],y=q[2],z=q[3];
        return Mat3{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
                    2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
                    2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}; };
    Mat3 A=m(qa),B=m(qb),R{};
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){double s=0;
        for(int k=0;k<3;++k)s+=A[size_t(k*3+i)]*B[size_t(k*3+j)]; R[size_t(i*3+j)]=s;}
    double t=R[0]+R[4]+R[8],w,x;
    if(t>0){double s=std::sqrt(t+1)*2; w=0.25*s; x=(R[7]-R[5])/s;}
    else {w=1;x=0;}
    double n=std::sqrt(w*w+x*x); if(n<1e-12)return 0;
    return std::fabs(2*std::atan2(std::fabs(x)/n,std::fabs(w)/n))*180.0/M_PI;
}
// 造一个只有食指、指定屈曲/外展的骨架
static SkeletonFrameResult mk(double flexP,double flexM,double abdM){
    SkeletonFrameResult r; r.valid=true; r.wristPoseValid=true;
    r.wristR={1,0,0, 0,1,0, 0,0,1}; r.wristT={0,0,0};
    auto ax=[](double f,double a){ double fr=f*M_PI/180, ar=a*M_PI/180;
        return Vec3{std::cos(fr)*std::cos(ar), std::sin(ar), -std::sin(fr)*std::cos(ar)}; };
    for(int m=0;m<20;++m){ r.markers[size_t(m)].label=m; r.markers[size_t(m)].observed=true; }
    Vec3 root{40,0,0};
    // dirProx = pp->mp（贴球在骨远端，见 computeSegmentQuats 的说明）
    // dirMid  = pp->dp   dirDist = mp->dp
    // 要让三段各不相同，三个球必须各自沿不同方向排。
    Vec3 a1=ax(flexP,0), a2=ax(flexM,abdM), a3=ax(flexM+25,abdM);
    Vec3 pp{root[0]+a1[0]*30, root[1]+a1[1]*30, root[2]+a1[2]*30};
    Vec3 mp{pp[0]+a2[0]*25, pp[1]+a2[1]*25, pp[2]+a2[2]*25};
    Vec3 dp{mp[0]+a3[0]*18, mp[1]+a3[1]*18, mp[2]+a3[2]*18};
    r.markers[8].posWorld=pp; r.markers[9].posWorld=mp; r.markers[10].posWorld=dp;
    return r;
}
int main(){
    std::array<Vec3,kNumSegments> py{};
    auto run=[&](SkeletonFrameResult r,bool chain){
        std::array<Vec3,kNumSegments> p{};
        Hm20SkeletonAssociator::TestHook_computeSegmentQuats(r,p,chain);
        return r; };
    std::printf("=== 外展出平面 15° 时的假扭转（近节-中节）===\n");
    std::printf("  %-10s %14s %14s\n","近节屈曲","旧:独立构造","新:沿链继承");
    for(double a:{20.,40.,60.,80.}){
        auto ro=run(mk(a,a+20,15),false), rn=run(mk(a,a+20,15),true);
        std::printf("  %-7.0f°%15.2f°%15.2f°\n",a,
            twistDeg(ro.segQuat[4],ro.segQuat[6]), twistDeg(rn.segQuat[4],rn.segQuat[6]));
    }
    std::printf("\n=== 纯屈曲（两者都应为 0）===\n");
    for(double a:{20.,50.}){
        auto ro=run(mk(a,a+25,0),false), rn=run(mk(a,a+25,0),true);
        std::printf("  近节%-4.0f°  旧 %.3f°   新 %.3f°\n",a,
            twistDeg(ro.segQuat[4],ro.segQuat[6]), twistDeg(rn.segQuat[4],rn.segQuat[6]));
    }
    std::printf("\n=== 退化：骨轴扫过手背法向（握拳，屈曲 70->110°）===\n");
    double omax=0,nmax=0; Quat op{},np{}; bool first=true;
    for(double a=70;a<=110;a+=0.5){
        auto ro=run(mk(a-20,a,0),false), rn=run(mk(a-20,a,0),true);
        if(!first){ omax=std::max(omax,twistDeg(op,ro.segQuat[5]));
                    nmax=std::max(nmax,twistDeg(np,rn.segQuat[5])); }
        op=ro.segQuat[5]; np=rn.segQuat[5]; first=false;
    }
    std::printf("  相邻帧最大跳变   旧 %.1f°   新 %.1f°\n",omax,nmax);
    return 0;
}
