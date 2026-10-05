#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
#include <cmath>
#include <random>
using namespace mocap::hm20;
using namespace mocap::hm20::detail;
static SkeletonFrameResult mk(double f1,double f2,double f3,double ab){
    SkeletonFrameResult r; r.valid=true; r.wristPoseValid=true;
    r.wristR={1,0,0, 0,1,0, 0,0,1};
    auto ax=[](double f,double a){double fr=f*M_PI/180,ar=a*M_PI/180;
        return Vec3{std::cos(fr)*std::cos(ar),std::sin(ar),-std::sin(fr)*std::cos(ar)};};
    for(int m=0;m<20;++m){r.markers[size_t(m)].label=m;r.markers[size_t(m)].observed=true;}
    Vec3 root{40,0,0}, a1=ax(f1,0),a2=ax(f2,ab),a3=ax(f3,ab);
    Vec3 pp{root[0]+a1[0]*30,root[1]+a1[1]*30,root[2]+a1[2]*30};
    Vec3 mp{pp[0]+a2[0]*25,pp[1]+a2[1]*25,pp[2]+a2[2]*25};
    Vec3 dp{mp[0]+a3[0]*18,mp[1]+a3[1]*18,mp[2]+a3[2]*18};
    r.markers[8].posWorld=pp;r.markers[9].posWorld=mp;r.markers[10].posWorld=dp;
    return r;
}
static double orthoErr(const Quat& q){
    double w=q[0],x=q[1],y=q[2],z=q[3];
    double n=std::sqrt(w*w+x*x+y*y+z*z);
    return std::fabs(n-1.0);
}
static double det(const Quat& q){
    // 四元数转矩阵必然是正交且 det=+1，这里查的是四元数本身有没有退化
    return q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
}
int main(){
    std::array<Vec3,kNumSegments> p{};
    std::printf("=== ① 四元数归一性（长链累积漂移）===\n");
    double worst=0;
    for(double f=0;f<=100;f+=5) for(double ab=-25;ab<=25;ab+=5){
        auto r=mk(f,f+20,f+40,ab); std::array<Vec3,kNumSegments> pp{};
        Hm20SkeletonAssociator::TestHook_computeSegmentQuats(r,pp,true);
        for(int s=4;s<=6;++s) worst=std::max(worst,orthoErr(r.segQuat[size_t(s)]));
    }
    std::printf("  最大 |‖q‖-1| = %.2e   %s\n",worst,worst<1e-6?"✅":"❌");

    std::printf("\n=== ② 连续性：相邻帧微小变化会不会跳 ===\n");
    Quat prev{}; bool first=true; double maxJump=0;
    for(double f=0;f<=110;f+=0.25){
        auto r=mk(f,f+20,f+40,10); std::array<Vec3,kNumSegments> pp{};
        Hm20SkeletonAssociator::TestHook_computeSegmentQuats(r,pp,true);
        const Quat& q=r.segQuat[6];
        if(!first){
            double d=q[0]*prev[0]+q[1]*prev[1]+q[2]*prev[2]+q[3]*prev[3];
            double ang=2*std::acos(std::min(1.0,std::fabs(d)))*180/M_PI;
            maxJump=std::max(maxJump,ang);
        }
        prev=q; first=false;
    }
    std::printf("  屈曲 0->110° 步长 0.25°，远节相邻帧最大转角 %.3f°  %s\n",
        maxJump, maxJump<3.0?"✅ 连续":"❌ 有跳变");

    std::printf("\n=== ③ 骨轴是否被如实保留（第 1 列 == 实测方向）===\n");
    double axErr=0;
    for(double f=10;f<=90;f+=10){
        auto r=mk(f,f+20,f+40,15); std::array<Vec3,kNumSegments> pp{};
        Vec3 pP=r.markers[8].posWorld,mP=r.markers[9].posWorld,dP=r.markers[10].posWorld;
        Hm20SkeletonAssociator::TestHook_computeSegmentQuats(r,pp,true);
        const Quat& q=r.segQuat[6];   // 远节：mp->dp
        double w=q[0],x=q[1],y=q[2],z=q[3];
        Vec3 col0{1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w)};
        Vec3 want=normalize(sub(dP,mP));
        axErr=std::max(axErr,std::acos(std::clamp(dot(col0,want),-1.0,1.0))*180/M_PI);
    }
    std::printf("  骨轴最大偏差 %.4f°  %s\n",axErr,axErr<0.01?"✅":"❌");
    return 0;
}
