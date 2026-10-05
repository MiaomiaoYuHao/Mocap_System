// 多视图仲裁验证。重点用例是"幽灵点剔除"：构造一个几何上真实满足两视图
// 极线约束、但没有第三台相机证实的假观测，验证它不会被误判成真实3D点，
// 也不会因此抢占掉真实点该用的观测（这是贪心算法一个容易踩的坑，见
// MultiViewCluster.hpp 里"未验证的种子不应 claim 观测"这条设计）。
// 纯数学，g++ -std=c++17 直接跑。
#include "reconstruct/MultiViewCluster.hpp"
#include <cstdio>
#include <cmath>
#include <array>
#include <vector>

using namespace mocap;

static bool projectNorm(const EpiMat3& R, const EpiVec3& t, const EpiVec3& Xw,
                        double& nx, double& ny) {
    const double Xc=R[0]*Xw[0]+R[1]*Xw[1]+R[2]*Xw[2]+t[0];
    const double Yc=R[3]*Xw[0]+R[4]*Xw[1]+R[5]*Xw[2]+t[1];
    const double Zc=R[6]*Xw[0]+R[7]*Xw[1]+R[8]*Xw[2]+t[2];
    if (Zc<=1e-6) return false;
    nx=Xc/Zc; ny=Yc/Zc; return true;
}
static EpiMat3 rotY(double a){double c=std::cos(a),s=std::sin(a);return {c,0,s,0,1,0,-s,0,c};}
static EpiVec3 tFromCenter(const EpiMat3&R, const EpiVec3&C){
    return { -(R[0]*C[0]+R[1]*C[1]+R[2]*C[2]),
             -(R[3]*C[0]+R[4]*C[1]+R[5]*C[2]),
             -(R[6]*C[0]+R[7]*C[1]+R[8]*C[2]) };
}
static double dist3(const EpiVec3&a,const std::array<double,3>&b){
    double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

int main() {
    bool ok = true;

    // ---- 三相机场景 ----
    const EpiMat3 I = {1,0,0,0,1,0,0,0,1};
    std::vector<EpiMat3> Rs(3); std::vector<EpiVec3> ts(3);
    Rs[0]=I; ts[0]={0,0,0};                                    // cam0：世界原点，无旋转
    Rs[1]=rotY(-0.3); ts[1]=tFromCenter(Rs[1], {200,0,0});      // cam1：右移+转向
    Rs[2]=rotY(0.25); ts[2]=tFromCenter(Rs[2], {-150,100,-50}); // cam2：另一个一般位姿

    const EpiVec3 P1 = {10,5,1000};
    const EpiVec3 P2 = {-30,20,900};

    std::vector<std::vector<std::array<double,2>>> obs(3);
    for (int c=0;c<3;++c) {
        double nx,ny;
        projectNorm(Rs[size_t(c)],ts[size_t(c)],P1,nx,ny); obs[size_t(c)].push_back({nx,ny});
        projectNorm(Rs[size_t(c)],ts[size_t(c)],P2,nx,ny); obs[size_t(c)].push_back({nx,ny});
    }
    // 此时每台相机 obs[c] = [P1的观测, P2的观测]（下标0,1）

    // ---- 注入幽灵观测：沿 cam0->P1 的同一条射线，换个深度的点 P1'，
    //      投影进 cam1。cam0 是世界原点+无旋转，这条射线就是过原点和 P1
    //      的直线，P1' = k*P1（k!=1）跟 P1 在 cam0 里的归一化坐标完全相同——
    //      这正是两视图算法分不清"是P1还是P1'"的真实几何根源，不是构造巧合。
    const EpiVec3 P1_ghost_3d = { P1[0]*1.5, P1[1]*1.5, P1[2]*1.5 };   // 同射线，深度变了
    double gx,gy; projectNorm(Rs[1],ts[1],P1_ghost_3d,gx,gy);
    const int ghostIdx = int(obs[1].size());   // 幽灵观测在 cam1 列表里的下标
    obs[1].push_back({gx,gy});
    // 验证一下这个幽灵观测确实跟 cam0 的 P1 观测满足两视图极线约束（不是编
    // 造巧合，是这条射线本身的几何性质）：
    {
        const EpiMat3 E01 = essentialFromRelativePose(Rs[0],ts[0],Rs[1],ts[1]);
        const double d = sampsonDistance(E01, obs[0][0][0],obs[0][0][1], gx,gy);
        printf("[前置检查] 幽灵观测与cam0的P1观测两视图 Sampson距离=%.2e（应≈0，确认歧义真实存在）\n", d);
        ok = ok && (d < 1e-9);
    }

    // ---- 跑多视图仲裁 ----
    auto r = clusterMultiView(Rs, ts, obs, /*maxSampson=*/0.01, /*maxReprojNorm=*/0.01);

    // ---- 断言1：产出恰好2条 verified track（P1真实点、P2），幽灵没有变成第3条 ----
    {
        bool pass = (r.tracks.size()==2);
        printf("[track数量] 产出%zu条verified track（应为2，幽灵不应算数）%s\n",
               r.tracks.size(), pass?"OK":"FAIL");
        ok = ok && pass;
    }

    // ---- 断言2：两条 track 分别精确对应 P1、P2（重建误差应在机器精度级）----
    {
        bool foundP1=false, foundP2=false;
        double errP1=-1, errP2=-1;
        for (auto& t : r.tracks) {
            double e1=dist3(P1,t.point), e2=dist3(P2,t.point);
            if (e1<1e-6){foundP1=true; errP1=e1;}
            if (e2<1e-6){foundP2=true; errP2=e2;}
        }
        bool pass = foundP1 && foundP2;
        printf("[重建精度] P1误差=%.2e P2误差=%.2e 两点都精确重建=%d %s\n",
               errP1, errP2, pass, pass?"OK":"FAIL");
        ok = ok && pass;
    }

    // ---- 断言3（核心）：P1这条track，cam1的支持必须是"真实观测"(下标0)，
    //      不是幽灵(下标ghostIdx)——证明真假被正确分辨，不是蒙对的 ----
    {
        bool found=false, correct=false;
        for (auto& t : r.tracks) {
            if (dist3(P1,t.point) > 1e-6) continue;
            found = true;
            for (auto& sp : t.support)
                if (sp.first==1) correct = (sp.second==0);   // cam1的支持应是下标0(真)，不是ghostIdx
        }
        printf("[真假辨别] P1的track里cam1用的是真实观测(非幽灵) found=%d correct=%d %s\n",
               found, correct, (found&&correct)?"OK":"FAIL");
        ok = ok && found && correct;
    }

    // ---- 断言4：幽灵观测本身应该留在 cam1 的 unmatched 列表里 ----
    {
        bool pass=false;
        for (int idx : r.unmatched[1]) if (idx==ghostIdx) pass=true;
        printf("[幽灵归宿] 幽灵观测(下标%d)留在cam1的unmatched里 %s\n", ghostIdx, pass?"OK":"FAIL");
        ok = ok && pass;
    }

    // ---- 断言5：cam0/cam2 没有遗留任何 unmatched（真实点全部正确归位）----
    {
        bool pass = r.unmatched[0].empty() && r.unmatched[2].empty();
        printf("[无遗漏] cam0/cam2 unmatched都为空 %s\n", pass?"OK":"FAIL");
        ok = ok && pass;
    }

    // ---- 测试：遮挡——P2 在 cam2 看不到时（3相机系统里只剩2视角支持）----
    //
    // 【这个用例的断言在两视图降级通道上线后改过】
    // 旧断言是"只产出1条track(P1)，P2 因为凑不够3视角被丢弃"。那正是用户
    // 实际反馈的问题：一个点被其中两台相机拍得清清楚楚，只要另外的相机
    // 遮挡了，它在3D点云里就直接消失——而三角化几何上只需要2个视角。
    // 现在的正确行为分两种模式，两种都要测：
    //   allowTwoViewFallback=true (默认)：P2 应该出点，但 verified=false，
    //     标明它只有2个视角撑着、没经过第三方交叉验证；
    //   allowTwoViewFallback=false：退回旧行为，只有 P1。
    {
        std::vector<std::vector<std::array<double,2>>> obsOcc(3);
        for (int c=0;c<3;++c) { double nx,ny; projectNorm(Rs[size_t(c)],ts[size_t(c)],P1,nx,ny); obsOcc[size_t(c)].push_back({nx,ny}); }
        // P2 只给 cam0、cam1，不给 cam2（模拟被挡）
        for (int c=0;c<2;++c) { double nx,ny; projectNorm(Rs[size_t(c)],ts[size_t(c)],P2,nx,ny); obsOcc[size_t(c)].push_back({nx,ny}); }

        // 模式A：默认——P2 应该出点且 verified=false，P1 出点且 verified=true
        auto rOcc = clusterMultiView(Rs, ts, obsOcc, 0.01, 0.01);
        bool foundP1=false, foundP2=false, p2Unverified=false, p1Verified=false;
        for (const auto& tk : rOcc.tracks) {
            if (dist3(P1, tk.point) < 1e-6) { foundP1=true; p1Verified = tk.verified; }
            if (dist3(P2, tk.point) < 1e-6) { foundP2=true; p2Unverified = !tk.verified; }
        }
        bool passA = foundP1 && p1Verified && foundP2 && p2Unverified;
        printf("[遮挡·降级通道] P2只2视角时仍出点且verified=false %s\n", passA?"OK":"FAIL");
        ok = ok && passA;

        // 模式B：关掉降级通道——退回旧行为，只有 P1
        auto rOld = clusterMultiView(Rs, ts, obsOcc, 0.01, 0.01, true, nullptr,
                                     9.21, -1, true, 0.0, 0.0, 0.01, 15, 0.0,
                                     /*allowTwoViewFallback=*/false);
        bool pass = (rOld.tracks.size()==1) && (dist3(P1,rOld.tracks[0].point)<1e-6);
        printf("[遮挡·关闭降级] 只产出1条track(P1)（旧行为） %s\n", pass?"OK":"FAIL");
        ok = ok && pass;
    }

    // ---- 测试：只有2台相机的系统——2视角支持就该算数（没有第3台可验证）----
    {
        std::vector<EpiMat3> Rs2={Rs[0],Rs[1]}; std::vector<EpiVec3> ts2={ts[0],ts[1]};
        std::vector<std::vector<std::array<double,2>>> obs2(2);
        for (int c=0;c<2;++c){double nx,ny;projectNorm(Rs2[size_t(c)],ts2[size_t(c)],P1,nx,ny);obs2[size_t(c)].push_back({nx,ny});}
        auto r2 = clusterMultiView(Rs2, ts2, obs2, 0.01, 0.01);
        bool pass = (r2.tracks.size()==1) && r2.tracks[0].verified && dist3(P1,r2.tracks[0].point)<1e-6;
        printf("[双相机系统] 2视角支持即可verified %s\n", pass?"OK":"FAIL");
        ok = ok && pass;
    }

    printf("\n%s\n", ok ? "multiview_cluster C++: ALL PASS" : "multiview_cluster C++: FAIL");
    return ok ? 0 : 1;
}
