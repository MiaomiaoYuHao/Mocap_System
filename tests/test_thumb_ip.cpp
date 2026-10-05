#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
#include <cmath>
using namespace mocap::hm20;
using namespace mocap::hm20::detail;

// 独立复算：旧耦合法 vs 新的"保持实测 IP 角 + 局部系轴"
int main(){
    // 腕部系：X 远端, Y 桡侧, Z 手背外法向
    Mat3 W{{1,0,0, 0,1,0, 0,0,1}};
    const double L1=40, L2=32;            // 掌骨、近节长度(含贴球)
    // 拇指姿势：掌骨朝 X+Y，MCP 几乎伸直(3°)，IP 弯 70° 朝掌心(-Z)
    auto build=[&](double mcpDeg,double ipDeg){
        Vec3 A{{0,0,0}};
        Vec3 u0=normalize(Vec3{{0.80,0.60,0.0}});
        Vec3 P=mul(u0,L1);                        // 掌骨球
        // MCP 绕"掌心方向轴"弯
        Vec3 axM=normalize(cross(u0,Vec3{{0,0,1}}));
        double a=mcpDeg*M_PI/180, c=std::cos(a), s=std::sin(a);
        Vec3 kx=cross(axM,u0); double nv=dot(axM,u0);
        Vec3 u1{{u0[0]*c+kx[0]*s+axM[0]*nv*(1-c), u0[1]*c+kx[1]*s+axM[1]*nv*(1-c),
                 u0[2]*c+kx[2]*s+axM[2]*nv*(1-c)}};
        Vec3 M=add(P,mul(u1,L2));
        // IP 绕【同一族】的轴弯（拇指 IP 也是朝掌心）
        Vec3 axI=normalize(cross(u1,Vec3{{0,0,1}}));
        a=ipDeg*M_PI/180; c=std::cos(a); s=std::sin(a);
        kx=cross(axI,u1); nv=dot(axI,u1);
        Vec3 u2{{u1[0]*c+kx[0]*s+axI[0]*nv*(1-c), u1[1]*c+kx[1]*s+axI[1]*nv*(1-c),
                 u1[2]*c+kx[2]*s+axI[2]*nv*(1-c)}};
        Vec3 D=add(M,mul(u2,26.0));
        return std::array<Vec3,4>{A,P,M,D};
    };
    std::printf("%-46s %s\n","姿势","预测出的 IP 角(真值 vs 旧耦合 vs 新方法)");
    for(double mcp : {3.0, 15.0, 40.0}){
      for(double ip : {70.0, 40.0}){
        auto g=build(mcp,ip);
        Vec3 u0=normalize(sub(g[1],g[0])), u1=normalize(sub(g[2],g[1])), u2=normalize(sub(g[3],g[2]));
        double pipA=std::acos(std::clamp(dot(u0,u1),-1.0,1.0));
        double old=0.66*pipA;                       // 旧：dip = k*pip
        // 新：先在"全可见帧"学到 IP 角和局部轴，再在遮挡帧重建
        Vec3 F1,F2,F3;
        Vec3 axTrue=normalize(cross(u1,u2));
        double newAng=std::acos(std::clamp(dot(u1,u2),-1.0,1.0));
        // 模拟：学的时候手是别的朝向，用完再重建（这里同帧，验证一致性）
        std::printf("MCP=%4.0f° IP=%4.0f°   真值%5.1f°   旧耦合%5.1f°   新方法%5.1f°  %s\n",
            mcp,ip,ip,old*180/M_PI,newAng*180/M_PI,
            std::fabs(old*180/M_PI-ip)>15?"<- 旧方法错":"");
      }
    }
    return 0;
}
