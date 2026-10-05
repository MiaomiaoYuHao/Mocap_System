// tests/test_dorsum_rigid_mc.cpp
// 手背刚体求解器的蒙特卡洛评测。零依赖，可单独编：
//     g++ -std=c++20 -O2 -I src tests/test_dorsum_rigid_mc.cpp -o mc && ./mc
// 布局A = 近似正五边形(现状)  布局B = 极角明显不均匀
// 关注 "错标" 那一列：它是唯一会让整只手翻过去的错误。
#include "estimate/DorsumRigidSolver.hpp"
#include <cstdio>
#include <random>
using namespace mocap::hm20;
using Vec3 = mocap::hm20::drs::V3;
using Mat3 = mocap::hm20::drs::M9;
static Mat3 randR(std::mt19937& g){
    std::normal_distribution<double> nd(0,1);
    // 随机四元数 -> R
    double q[4]; double n=0; for(int i=0;i<4;++i){q[i]=nd(g);n+=q[i]*q[i];} n=std::sqrt(n);
    for(int i=0;i<4;++i)q[i]/=n;
    double w=q[0],x=q[1],y=q[2],z=q[3];
    return Mat3{{1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
                 2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
                 2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)}};
}
static Vec3 ap(const Mat3&R,const Vec3&t,const Vec3&p){
    return Vec3{{R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0],R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1],R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2]}};
}
int main(int argc,char**argv){
    // 布局 A = 现状(近似正五边形共面)；布局 B = 角度不均匀
    double radA[5]={28.2,35.2,21.8,23.7,34.4}, angA[5]={0,74,145,212,289};
    double radB[5]={28.2,35.2,21.8,23.7,34.4}, angB[5]={0,40,95,190,250};
    for(int layout=0; layout<2; ++layout){
        const double*rad = layout? radB:radA; const double*ang = layout? angB:angA;
        std::array<Vec3,5> T{}; Vec3 c{{0,0,0}};
        for(int i=0;i<5;++i){double a=ang[i]*3.14159265358979/180.0;
            T[i]={rad[i]*std::cos(a),rad[i]*std::sin(a),0.0}; for(int d=0;d<3;++d)c[d]+=T[i][d]/5;}
        for(int i=0;i<5;++i) for(int d=0;d<3;++d) T[i][d]-=c[d];
        DorsumRigidSolver S; S.setTemplate(T);
        std::printf("\n===== 布局%c  自歧义(最优错解残差)=%.2f mm =====\n", 'A'+layout, S.selfAmbiguityMm());
        std::printf("%6s %5s %5s %6s | %7s %7s %7s   %s\n","噪声","遮挡","杂点","历史","正确","错标","拒解","(400 次/组)");
        for(double noise : {0.9, 2.0}) for(int occ : {0,1,2}) for(int clut : {0,10}) for(int useHist : {0,1}) {
            std::mt19937 g(1234u + unsigned(noise*10)+unsigned(occ*7)+unsigned(clut)+unsigned(useHist*3));
            std::normal_distribution<double> nz(0,noise), big(0,60), pos(0,300);
            int ok=0,wrong=0,abst=0;
            S.resetHistory();
            for(int it=0; it<400; ++it){
                Mat3 R=randR(g); Vec3 t{{pos(g),pos(g),pos(g)}};
                std::vector<Vec3> pts; std::vector<int> gtSlot;
                // 随机丢 occ 个
                std::array<int,5> keep{{0,1,2,3,4}};
                std::shuffle(keep.begin(),keep.end(),g);
                std::vector<int> vis(keep.begin(), keep.begin()+(5-occ));
                std::sort(vis.begin(),vis.end());
                for(int k : vis){ Vec3 p=ap(R,t,T[size_t(k)]); for(int d=0;d<3;++d)p[d]+=nz(g);
                    pts.push_back(p); gtSlot.push_back(k); }
                for(int q=0;q<clut;++q) pts.push_back(Vec3{{t[0]+big(g),t[1]+big(g),t[2]+big(g)}});
                if(useHist) S.acceptHistory(R);   // 上一帧的真位姿当历史
                else S.resetHistory();
                auto r=S.solve(pts);
                if(!r.ok){++abst; continue;}
                bool bad=false;
                for(int k=0;k<5;++k){
                    int a=r.pointOf[size_t(k)];
                    if(a<0) continue;
                    // a 必须指向 gtSlot 中值为 k 的那个下标
                    if(a>=int(gtSlot.size()) || gtSlot[size_t(a)]!=k){bad=true;break;}
                }
                if(bad)++wrong; else ++ok;
            }
            std::printf("%6.1f %5d %5d %6s | %7.3f %7.3f %7.3f\n",noise,occ,clut,useHist?"有":"无",
                ok/400.0,wrong/400.0,abst/400.0);
        }
    }
    return 0;
}
