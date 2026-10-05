#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
#include <random>
using namespace mocap::hm20;
using namespace mocap::hm20::detail;
// 复刻 thumbLocalFrame 的逻辑，验证"学一次、手转到别处还能重建出正确的轴"
static bool frameOf(const Vec3& u1,const Mat3& W,Vec3&F1,Vec3&F2,Vec3&F3){
    F1=u1; Vec3 d{{W[2],W[5],W[8]}}; double p=dot(d,F1);
    Vec3 e2{{d[0]-p*F1[0],d[1]-p*F1[1],d[2]-p*F1[2]}};
    double n=norm(e2); if(n<1e-3) return false;
    F2=mul(e2,1.0/n); F3=cross(F1,F2); return true;
}
int main(){
    std::mt19937 g(3); std::normal_distribution<double> nd(0,1);
    // 学习帧：手在朝向 A，量到 IP 轴
    Mat3 WA{{1,0,0, 0,1,0, 0,0,1}};
    Vec3 u1A=normalize(Vec3{{0.78,0.55,-0.30}});
    Vec3 axA=normalize(cross(u1A,Vec3{{0,0,1}}));      // 真实 IP 屈曲轴
    Vec3 F1,F2,F3; frameOf(u1A,WA,F1,F2,F3);
    Vec3 loc{{dot(axA,F1),dot(axA,F2),dot(axA,F3)}};
    std::printf("学习帧：局部系里的轴 = (%.3f, %.3f, %.3f)\n\n",loc[0],loc[1],loc[2]);
    std::printf("%-8s %-38s %s\n","测试","手转到新朝向后重建的轴","与真值夹角");
    double worst=0;
    for(int t=0;t<6;++t){
        // 随机把整只手（腕部系 + 近节骨轴）刚性转到别处
        double q[4]; double nn=0; for(int i=0;i<4;++i){q[i]=nd(g);nn+=q[i]*q[i];}
        nn=std::sqrt(nn); for(int i=0;i<4;++i)q[i]/=nn;
        double w=q[0],x=q[1],y=q[2],z=q[3];
        Mat3 R{{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
                2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
                2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}};
        Mat3 WB=matMul(R,WA);
        Vec3 u1B=matVec(R,u1A);
        Vec3 axTrue=matVec(R,axA);                      // 刚性转过去的真值
        Vec3 G1,G2,G3; frameOf(u1B,WB,G1,G2,G3);
        Vec3 rec{{G1[0]*loc[0]+G2[0]*loc[1]+G3[0]*loc[2],
                  G1[1]*loc[0]+G2[1]*loc[1]+G3[1]*loc[2],
                  G1[2]*loc[0]+G2[2]*loc[1]+G3[2]*loc[2]}};
        rec=normalize(rec);
        double a=std::acos(std::clamp(dot(rec,axTrue),-1.0,1.0))*180/M_PI;
        worst=std::max(worst,a);
        std::printf("朝向%d   (%6.3f,%6.3f,%6.3f)              %.4f°\n",t,rec[0],rec[1],rec[2],a);
    }
    std::printf("\n最大误差 %.4f°  %s\n",worst,worst<0.01?"✅ 手怎么转轴都跟着对":"❌");
    return 0;
}
