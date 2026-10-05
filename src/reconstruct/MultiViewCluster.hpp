#pragma once
// ---------------------------------------------------------------------------
// 多视图仲裁 —— 纯数学，零 Qt/第三方依赖，可单测。
//
// TwoViewMatcher.hpp 自己在注释里承认了一个天生盲点：如果 cam1 的两个点
// 恰好落在同一条极线上，光凭两台相机的极线约束分不清谁是谁（Sampson 距离
// 都很小），这是真实存在的几何歧义，不是实现问题。解决办法是让第三台
// （及以上）相机来当裁判——一个候选 3D 点如果是真的，把它重投影回所有能
// 看到它的相机，每台的重投影误差都该很小；幽灵点（两条无关视线凑巧靠得
// 近）通常在第三台相机上就露馅。
//
// 流程（对应主方案的"贪心生成候选 + 全局冲突消解"）：
//   1. 对每一对相机跑 TwoViewMatcher，生成两视图候选种子（一对匹配的观测）。
//   2. 每个种子先用两条视线的最近点三角化出一个初始 3D 点。
//   3. 拿这个初始点去"考"其余每一台相机：它的观测里有没有哪个点，重投影
//      误差落在阈值内？有就吸收进这个候选的支持集合。
//   4. 用全部支持视角重新做一次 N 视图 DLT 精解（比两视线最近点更准）。
//   5. 支持视角 >=3（或者总共就只有2台相机时 >=2）才算"验证通过"；只有
//      2 视图支持、且系统里明明有更多相机能看的候选，标记为"未验证"——
//      这正是"两点共线歧义"的信号：两视图看着像，但没人证实。
//   6. 冲突消解：如果两个候选抢同一个观测，支持视角数更多的赢；打平比
//      总残差，输的那个候选放弃该观测（可能因此支持数不够、被降级或丢弃）。
//
// 跟 Triangulation.hpp 的关系：那边的 triangulateMultiViewRobust 做的是
// "已知是同一个点，多视角一起解、剔除坏视角"；这里做的是"还不知道是不是
// 同一个点，用多视角互相验证来确定"——两者互补，数学核心（DLT、最近点）
// 有相似之处，但因为要保持这个模块零 Qt 依赖（CameraIntrinsics/Extrinsics
// 定义在 Calibration.hpp，引了 Qt 头），这里用 Epipolar.hpp 同款的裸数组
// (EpiMat3/EpiVec3) 独立实现一份，跟 HandPose.hpp 自己独立实现一份 3x3
// Jacobi（不复用 Triangulation.cpp 的 4x4 版本）是同一个先例。
// ---------------------------------------------------------------------------
#include "reconstruct/Epipolar.hpp"
#include "reconstruct/TwoViewMatcher.hpp"
#include "reconstruct/TriangulationRefine.hpp"
#include "detect/DetectionOutput.hpp"   // Cov2——只借这一个零Qt依赖的小结构体，见下方马氏距离投票的说明
#include <vector>
#include <array>
#include <utility>
#include <algorithm>
#include <cmath>
#include <functional>

// 【线程可用性探测】MinGW 有两种线程模型：posix 模型提供 std::thread，
// win32 模型的 <thread> 是空的（std::thread 根本不存在）。直接用会编译失败，
// 而且报的是一串莫名其妙的 parse 错误。这里显式探测，不可用就退回串行——
// 功能完全不受影响，只是投票不并行。
#include <thread>
#if defined(_MSC_VER) || defined(_GLIBCXX_HAS_GTHREADS) || defined(__cpp_lib_thread)
  #define MOCAP_HAS_STD_THREAD 1
#else
  #define MOCAP_HAS_STD_THREAD 0
#endif

namespace mocap {

struct Track3D {
    std::array<double, 3> point{0, 0, 0};
    std::vector<std::pair<int, int>> support;   // (相机下标, 观测下标)，按相机下标升序
    double residual = -1.0;   // 平均重投影误差（近似像素单位）
    bool   verified = false;  // 支持视角数达标（>=3，或总相机数恰好2时>=2）
};

struct ClusterResult {
    std::vector<Track3D> tracks;
    std::vector<std::vector<int>> unmatched;   // 每台相机剩下没归入任何 track 的观测下标
};

namespace cluster_detail {

// 两条视线的最近点（跟 Triangulation.cpp 的 closestPointBetweenRays 同数学，
// 这里用裸 R,t 重新实现一份以保持零 Qt）。c=相机中心(世界系)，d=视线单位方向。
inline EpiVec3 cameraCenter(const EpiMat3& R, const EpiVec3& t) {
    // C = -R^T t
    return { -(R[0]*t[0]+R[3]*t[1]+R[6]*t[2]),
             -(R[1]*t[0]+R[4]*t[1]+R[7]*t[2]),
             -(R[2]*t[0]+R[5]*t[1]+R[8]*t[2]) };
}
inline EpiVec3 rayDir(const EpiMat3& R, double nx, double ny) {
    EpiVec3 d = { R[0]*nx+R[3]*ny+R[6], R[1]*nx+R[4]*ny+R[7], R[2]*nx+R[5]*ny+R[8] };
    const double n = std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
    if (n > 1e-12) { d[0]/=n; d[1]/=n; d[2]/=n; }
    return d;
}

struct RayPairResult { EpiVec3 point{}; double residual = -1; bool valid = false; };

inline RayPairResult triangulateRayPair(const EpiMat3& R0, const EpiVec3& t0, double nx0, double ny0,
                                        const EpiMat3& R1, const EpiVec3& t1, double nx1, double ny1) {
    const EpiVec3 c0 = cameraCenter(R0,t0), c1 = cameraCenter(R1,t1);
    const EpiVec3 d0 = rayDir(R0,nx0,ny0), d1 = rayDir(R1,nx1,ny1);
    const EpiVec3 w0 = { c0[0]-c1[0], c0[1]-c1[1], c0[2]-c1[2] };
    const double a = d0[0]*d0[0]+d0[1]*d0[1]+d0[2]*d0[2];
    const double b = d0[0]*d1[0]+d0[1]*d1[1]+d0[2]*d1[2];
    const double c = d1[0]*d1[0]+d1[1]*d1[1]+d1[2]*d1[2];
    const double d = d0[0]*w0[0]+d0[1]*w0[1]+d0[2]*w0[2];
    const double e = d1[0]*w0[0]+d1[1]*w0[1]+d1[2]*w0[2];
    const double denom = a*c-b*b;
    RayPairResult r;
    if (std::abs(denom) < 1e-9) { r.valid=false; return r; }
    const double s=(b*e-c*d)/denom, tt=(a*e-b*d)/denom;
    const EpiVec3 q0={c0[0]+s*d0[0],c0[1]+s*d0[1],c0[2]+s*d0[2]};
    const EpiVec3 q1={c1[0]+tt*d1[0],c1[1]+tt*d1[1],c1[2]+tt*d1[2]};
    r.point = {(q0[0]+q1[0])*0.5,(q0[1]+q1[1])*0.5,(q0[2]+q1[2])*0.5};
    const double dx=q0[0]-q1[0],dy=q0[1]-q1[1],dz=q0[2]-q1[2];
    r.residual = std::sqrt(dx*dx+dy*dy+dz*dz);
    r.valid = true;
    return r;
}

// 把世界点重投影到某相机的归一化坐标；Zc<=0（点在相机后方）返回 false。
inline bool reprojectNorm(const EpiMat3& R, const EpiVec3& t, const EpiVec3& Xw,
                          double& nx, double& ny) {
    const double Xc=R[0]*Xw[0]+R[1]*Xw[1]+R[2]*Xw[2]+t[0];
    const double Yc=R[3]*Xw[0]+R[4]*Xw[1]+R[5]*Xw[2]+t[1];
    const double Zc=R[6]*Xw[0]+R[7]*Xw[1]+R[8]*Xw[2]+t[2];
    if (Zc <= 1e-6) return false;
    nx=Xc/Zc; ny=Yc/Zc; return true;
}

// N 视图 DLT：4x4 对称矩阵最小特征向量（雅可比旋转法），跟 Triangulation.cpp
// 的 smallestEigenvector4 同算法、独立实现（原因见文件头注释）。
inline void smallestEigenvector4(const double M[4][4], double out[4]) {
    double A[4][4];
    for (int i=0;i<4;++i) for (int j=0;j<4;++j) A[i][j]=M[i][j];
    double V[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    for (int sweep=0; sweep<50; ++sweep) {
        int p=0,q=1; double mx=0;
        for (int i=0;i<4;++i) for(int j=i+1;j<4;++j)
            if (std::abs(A[i][j])>mx){mx=std::abs(A[i][j]);p=i;q=j;}
        if (mx<1e-18) break;
        const double app=A[p][p],aqq=A[q][q],apq=A[p][q];
        const double phi=0.5*std::atan2(2*apq,aqq-app);
        const double c=std::cos(phi), s=std::sin(phi);
        for (int i=0;i<4;++i){double aip=A[i][p],aiq=A[i][q];A[i][p]=c*aip-s*aiq;A[i][q]=s*aip+c*aiq;}
        for (int i=0;i<4;++i){double api=A[p][i],aqi=A[q][i];A[p][i]=c*api-s*aqi;A[q][i]=s*api+c*aqi;}
        for (int i=0;i<4;++i){double vip=V[i][p],viq=V[i][q];V[i][p]=c*vip-s*viq;V[i][q]=s*vip+c*viq;}
    }
    int minIdx=0; double minVal=A[0][0];
    for (int i=1;i<4;++i) if (A[i][i]<minVal){minVal=A[i][i];minIdx=i;}
    for (int i=0;i<4;++i) out[i]=V[i][minIdx];
}

// 用一批 (R,t,nx,ny) 观测做 N 视图 DLT 三角化，返回世界点。至少要2个观测。
inline bool triangulateNView(const std::vector<const EpiMat3*>& Rs,
                             const std::vector<const EpiVec3*>& ts,
                             const std::vector<std::array<double,2>>& nxy,
                             EpiVec3& outPoint) {
    const int n = int(Rs.size());
    if (n < 2) return false;
    double ATA[4][4] = {{0}};
    for (int v=0; v<n; ++v) {
        const EpiMat3& R = *Rs[v]; const EpiVec3& t = *ts[v];
        const double nx=nxy[v][0], ny=nxy[v][1];
        const double P0[4]={R[0],R[1],R[2],t[0]};
        const double P1[4]={R[3],R[4],R[5],t[1]};
        const double P2[4]={R[6],R[7],R[8],t[2]};
        double row0[4],row1[4];
        for (int k=0;k<4;++k){ row0[k]=nx*P2[k]-P0[k]; row1[k]=ny*P2[k]-P1[k]; }
        for (int a=0;a<4;++a) for(int b=0;b<4;++b) ATA[a][b]+=row0[a]*row0[b]+row1[a]*row1[b];
    }
    double X[4]; smallestEigenvector4(ATA, X);
    if (std::abs(X[3]) < 1e-12) return false;
    outPoint = {X[0]/X[3], X[1]/X[3], X[2]/X[3]};
    return true;
}

// 2x2对称协方差求逆（跟 estimate/HandStateIEKF.hpp::iekf_detail::invert2x2 同
// 数学，独立实现一份保持本文件零 Qt/零跨层依赖——同一先例见文件头注释
// 关于 smallestEigenvector4 的说明）。矩阵退化返回false。
//
// 【重要】退化判据必须是"无量纲"的，不能用绝对阈值。
// 这里的输入是**归一化坐标系**下的协方差：var = (sigma_px / f)^2。
// f≈900 时，一个检测得很干净的观测 sigma=0.1px -> var≈1.2e-8 -> det≈1.5e-16。
// 旧代码写的是 `if (std::abs(det) < 1e-15) return false;`——一个绝对阈值，
// 量纲是"归一化坐标的四次方"。后果是：**检测质量越好，det 越小，越容易被
// 判成"退化矩阵"直接丢弃**。实测 f=900 时，任何 sigma < 0.18px 的观测都会
// 被这道判据静默吃掉，导致 clusterMultiView 的马氏投票拿不到任何支持视角，
// 整条管线一个点都发布不出来，且不报任何错。症状是反直觉的：把检测器调好、
// 换更好的镜头、把标记球擦干净，系统反而彻底不工作。
// 而且这个阈值等效的 sigma 门限 ∝ 1/f²，换个焦距的相机门限就跟着变，
// 表现为"A 相机能跑、B 相机死机"。
//
// 正确的判据是判"这个协方差是不是合法的正定矩阵"，而这本身就是无量纲的：
//   xx>0, yy>0, 且相关系数 rho² = xy²/(xx·yy) 必须显著小于 1。
// 跟单位、跟焦距、跟检测质量都无关——只有真正病态（某个方向方差为0，或
// 两个方向完全相关退化成一条直线）才会被拒。
inline bool invert2x2(double xx, double xy, double yy, double out[2][2]) {
    // 非正定（含 NaN：任何与 NaN 的比较都为 false，所以这里天然把 NaN 挡掉）
    if (!(xx > 0.0) || !(yy > 0.0)) return false;
    const double prod = xx * yy;
    // rho² 必须 <= 1 - kMinRelDet。1e-9 是纯数值余量（double 有 ~1e-16 的
    // 相对精度），不是物理阈值，所以不会因为观测"太准"而误伤。
    constexpr double kMinRelDet = 1e-9;
    const double det = prod - xy * xy;
    if (!(det > kMinRelDet * prod)) return false;
    const double invDet = 1.0/det;
    out[0][0] = yy*invDet; out[0][1] = -xy*invDet;
    out[1][0] = -xy*invDet; out[1][1] = xx*invDet;
    return true;
}

// 两条视线方向的夹角(度)，取绝对值(不区分方向/反方向)——三角化质量的
// 核心几何量：夹角越接近0°(两条视线近乎平行)，同样大小的像素噪声在
// 深度方向上被放大得越厉害，三角化解出来的3D点误差可能远超"重投影误差
// 看起来很小"给人的错觉；夹角接近90°时误差放大效应最小、三角化最稳。
// 见 clusterMultiView() 里 minRayAngleDeg 参数的完整说明。
inline double rayAngleDeg(const EpiVec3& d0, const EpiVec3& d1) {
    double dot = d0[0]*d1[0] + d0[1]*d1[1] + d0[2]*d1[2];
    dot = std::max(-1.0, std::min(1.0, std::abs(dot)));
    return std::acos(dot) * 180.0 / M_PI;
}

// 已知3D点和两台相机时的视线夹角。注意 rayAngleDeg() 要求**单位向量**
// (它直接把点积当cos用)，从"点-相机中心"算出来的向量模长是米级的，必须
// 先归一化再传进去，否则点积被钳到1.0、夹角恒等于0°。
inline double rayAngleAtPointDeg(const EpiVec3& X,
                                 const EpiMat3& Ra, const EpiVec3& ta,
                                 const EpiMat3& Rb, const EpiVec3& tb) {
    const EpiVec3 ca = cameraCenter(Ra, ta);
    const EpiVec3 cb = cameraCenter(Rb, tb);
    EpiVec3 da{X[0]-ca[0], X[1]-ca[1], X[2]-ca[2]};
    EpiVec3 db{X[0]-cb[0], X[1]-cb[1], X[2]-cb[2]};
    const double na = std::sqrt(da[0]*da[0]+da[1]*da[1]+da[2]*da[2]);
    const double nb = std::sqrt(db[0]*db[0]+db[1]*db[1]+db[2]*db[2]);
    if (na < 1e-9 || nb < 1e-9) return 0.0;
    da = {da[0]/na, da[1]/na, da[2]/na};
    db = {db[0]/nb, db[1]/nb, db[2]/nb};
    return rayAngleDeg(da, db);
}

// 支持集里最大的一对视线夹角——两视图降级通道的几何质量指标。
inline double bestSupportRayAngleDeg(const EpiVec3& X,
        const std::vector<EpiMat3>& Rs, const std::vector<EpiVec3>& ts,
        const std::vector<std::pair<int,int>>& support) {
    double best = 0.0;
    for (size_t a=0; a+1<support.size(); ++a)
        for (size_t b=a+1; b<support.size(); ++b)
            best = std::max(best, rayAngleAtPointDeg(X,
                Rs[size_t(support[a].first)], ts[size_t(support[a].first)],
                Rs[size_t(support[b].first)], ts[size_t(support[b].first)]));
    return best;
}

} // namespace cluster_detail

// 多视图仲裁主函数。Rs/ts：每台相机的世界->相机位姿（下标即相机编号）；
// obsPerCam：每台相机的归一化观测列表。maxSampson 是两视图种子阶段的极线
// 剪枝阈值（同 TwoViewMatcher）；maxReprojNorm 是"用候选点去考其余相机"时
// 的重投影误差阈值（归一化坐标单位，跟 maxSampson 通常取同一个量级）。
//
// 【这一版新增】useLmRefine：支持视角数确定之后，DLT给的是代数最优解，
// 不是直接最小化真实重投影误差的解——两者噪声小时几乎没差别，噪声大/
// 视角夹角刁钻时能差出几mm，见 TriangulationRefine.hpp 顶部完整分析。
// 默认开启，在DLT解基础上加一轮LM精修(通常5~10次迭代，计算量很小)，
// 同时做IRLS：如果某个支持视角这一帧的观测明显跟其它视角对不上(比如
// 检测噪声突然变大、局部遮挡导致亚像素偏差)，会被自动软性降权，不会
// 让一个坏视角把最终点拉偏太多。Track3D.residual 相应地从"简单平均
// 残差"换成"IRLS加权RMS残差"，数值上通常会更小(坏视角被压低了权重)——
// 如果你的调用方有依赖这个字段具体数值的逻辑(比如硬编码的残差阈值)，
// 升级后建议重新核对一下阈值是否仍然合适。
// 【这一版新增】obsCovPerCam：跟 obsPerCam 形状一一对应的每观测协方差
// (来自 detect/BlobObservationAdapter.hpp 按弧长/遮挡算出的真实不确定度，
// 不是各向同性的固定值)。传了(且跟obsPerCam形状对得上)时，"用候选点去
// 考其余相机"这一步改用马氏距离(chiSquareGate，默认9.21，跟
// MarkerAssociator.hpp 同一套统计学取值)做主要判据——弧短/遮挡严重、
// 协方差本来就大的观测，门控自动放宽；清晰完整的观测，门控自动收紧。
// maxReprojNorm 保留作硬性兜底上限(双保险，跟 MarkerAssociator 的
// gateRadiusNorm 是同一个精神)：协方差异常时马氏距离可能被撑得不合理地
// 宽，兜底上限确保候选对的实际归一化坐标距离不会超过这个绝对值。
// 不传协方差(默认，nullptr)或形状对不上时，诚实退化成只用maxReprojNorm
// 的固定欧氏距离门控——等价于加入马氏距离投票之前的旧行为，调用方在
// 还没接入检测层协方差的场景(比如RadiusRefine第一遍粗算，那时候半径都
// 还没确定，压根没有有意义的sigma)不需要为了这个新参数造一份假协方差。
// 【这一版新增】minSupportViews：多相机投票阶段"验证通过"需要的最少支持
// 视角数。默认-1=自动(沿用旧行为：总相机数恰好2时用2，否则死板要求3)——
// 旧行为的问题：3视角投票的本意是防幽灵点(第三台相机当裁判)，但它被
// 意外跟"你总共配置了几台相机"绑死了，没考虑"一个点被部分相机遮挡是
// 完全正常的场景"。比如配了4台相机、这个点被手挡住了2台，剩下2台清楚
// 看到，旧逻辑因为总相机数4≠2，照样要求3视角支持，永远验证不过——
// 这个点明明有效观测，却被判定"不存在"。
// 调用方可以显式传2(允许"至少2台相机看到就算追踪到")，代价是放弃了
// 第三方验证：极线歧义/两点共线场景下，一个巧合凑近的幽灵点也只需要
// 2视角就能通过，更容易混入结果。这是"更容易追踪到"和"更抗幽灵点"
// 之间的真实取舍，不存在两头都要的免费午餐，所以做成可调参数而不是
// 直接把默认值改掉——需要哪种取舍由调用方按场景决定。
// 显式传入的值会被夹在[2, nc]之间：小于2没有意义(2视角是三角化的数学
// 下限)，大于nc不可能达到(总共就没那么多相机)。
// 【这一版新增】useVoting：是否启用"多相机投票"仲裁(默认true)。这套机制
// 本来是为了解决"两点共线歧义"——用第三台(及以上)相机的观测反过来验证
// 两视图候选，见文件头注释。相机数量少(尤其只有2台)时，压根没有第三台
// 相机可以投票，这套机制自动退化成约等于"直接按两视图残差取"，多算的
// 那一层investigate其余相机的开销纯属浪费；而如果相机数刚好3~4台但常
// 出现部分相机被遮挡的场景，"投票"里"支持视角数多的优先"这个排序本身
// 不是问题，问题主要在 minSupportViews 那道门槛(已经做成可调参数)。
// 但仍然给一个可以整体关掉投票、退回最原始逻辑的开关，覆盖"相机数量少、
// 想要最简单直接的两视图三角化，不要任何多视角仲裁"这种场景：关闭后，
// 直接按两视图种子的Sampson残差从小到大贪心抢占观测(旧版实现思路，
// 见本文件更早版本的注释)，不再对每个候选去"考"其余相机、不再按支持
// 视角数排序——省掉这层计算，但也放弃了它带来的抗歧义能力，
// minSupportViews参数在关闭投票时不生效(两视图种子本来就只有2视角支持)。
// 【这一版新增】minRayAngleDeg：两视图种子的最小视线夹角(度)，默认0.0=
// 不启用(保持原行为)。这是"远处突然冒出一个幽灵点"这类现象的源头级
// 修复：这种幽灵点的根因通常不是空间位置本身有问题，而是三角化用的两条
// 视线夹角太小(近乎平行)——这种几何条件下，微小的像素/检测噪声在深度
// 方向上会被急剧放大，重投影误差可以依然很小(在maxReprojNorm门槛内)，
// 但反算出来的3D点可能被推到很远的地方。之前唯一能防住这类解的办法是
// 把maxReprojNorm调紧，但那是2D归一化坐标下的误差，跟"这条视线本身
// 适不适合拿来三角化"是两个维度的问题，调紧2D阈值治标不治本，还会误伤
// 大量正常的高质量观测。
// 这里换成从几何条件本身拦截：两条视线夹角低于这个阈值，这个两视图种子
// 从一开始就不被采纳为候选(不管它的重投影误差看起来多小)——不需要预设
// 场景的"最大追踪范围"，纯粹是相机相对姿态决定的固有几何条件，跟点的
// 空间位置、场景大小完全无关，换场景不用重新调。
// 建议值：完全避免病态解通常设 15~30 度就足够(见标准三角化最小夹角
// 经验规则)；设0保持关闭(完全不过滤，旧行为)；相机数量少、基线本来就
// 有限的场景下不建议设太大(会把大量本来有效但夹角一般的观测也拒掉)，
// 从小往大调、观察"稳定性统计"和"未验证/歧义丢弃"数量的变化再决定。
// 【这一版新增】allowTwoViewFallback / twoViewMinRayAngleDeg：
//   两视图降级通道。旧行为是"4台相机就硬性要求>=3个支持视角"，于是出现了
//   一个非常反直觉的现象：**一个点被其中两台相机拍得清清楚楚，只要另外两台
//   因为遮挡/贴太近没认出来，这个点在3D点云里就直接消失。**
//   但三角化在几何上只需要2个视角。要求3个视角的初衷是抗幽灵(两视图匹配可能
//   是极线上的巧合)，不是几何必需——把它写成硬门槛，等于用"宁可丢真点也不
//   放过假点"的策略换鲁棒性，而在手指并拢、多点密集这种最需要它工作的场景下，
//   恰恰是遮挡最严重、最容易只剩2个视角的时候。
//
//   现在改成：>=2 个支持视角就可以出点，但只有 >=effectiveMinSupport 的才标记
//   Track3D::verified=true；只有2个视角支撑的点必须额外满足"两条视线夹角
//   >= twoViewMinRayAngleDeg"才被接受——因为2视图幽灵几乎都来自近乎平行的
//   退化几何(夹角小的时候极线约束几乎不约束深度，随便两个观测都能"匹配上")，
//   而夹角足够大的2视图解在几何上是可信的。
//   剩下的抗幽灵责任交给下游追踪器的确认逻辑(minHitsToConfirm)：真实被遮挡的
//   点是在延续一条已有轨迹，幽灵点是凭空冒出来的，用时间一致性区分比用单帧
//   支持视角数区分要准得多。
//   allowTwoViewFallback=false 可退回旧行为(用于对拍)。
//
// 【这一版新增】maxFinalResidualNorm：最终提交前的重投影残差上限(归一化单位)。
//   <=0 表示不检查(旧行为)。配合下面的"冲突后部分提交"使用——支持视角被更强
//   的候选抢走一部分之后，用剩下的视角重新三角化，这道闸门负责确认重解出来的
//   点仍然自洽，不自洽就丢弃。
//
// 【这一版新增】calibSigmaNorm：标定不确定度的等效标准差（归一化坐标单位），
// 默认 0.0 = 关闭（保持旧行为）。见函数体内马氏距离那段的完整说明——一句话
// 概括：检测层协方差只描述检测噪声，不含标定偏差，而实测标定偏差对追踪的
// 破坏力远大于检测噪声（0.5px 主点误差 ≈ 召回率 0.993→0.797）。这个参数把
// 标定偏差按方差可加性并进门控，让门控从"只容忍检测抖动"变成"容忍检测抖动
// + 标定偏差"。
// 取值建议：标定报告给出的主点/外参不确定度换算到归一化坐标（除以焦距）。
// 例如主点不确定度 3px、f=900 -> 3/900 ≈ 0.0033。没有不确定度估计时，可以
// 用一个保守的经验值（0.002~0.004），也远好过按 0 处理。
// 注意这不是"把门控无脑放宽"：它只放宽到跟标定质量相称的程度，标定越好
// 这一项越小、门控越紧，标定越差门控越松——这正是我们想要的自适应行为，
// 而不是拍一个固定的宽阈值把幽灵点也放进来。
namespace cluster_detail {

// 三角化候选：一个种子 + 它在各相机上的支持观测。
// 放在命名空间作用域（而不是 clusterMultiView 函数内部）是为了兼容性——
// 局部类塞进 std::vector 再在 lambda 里赋值，MinGW GCC 上会触发 parse 错误。
// 名字特意叫 VoteCandidate 而不是 Candidate——clusterMultiView 函数体最上面
// 本来就有一句 "using namespace cluster_detail;"，如果这里也叫 Candidate、
// 下面又写一句 "using Candidate = cluster_detail::Candidate;"，两条引入同一
// 名字的路径叠在一起会让 MinGW 的解析器产生歧义，直接把后面第一处用到
// Candidate 的声明解析错，引发一长串连锁报错。改个独一无二的名字，从根上
// 让这类歧义不可能发生，比排查"为什么这两种引入方式在这个编译器上冲突"
// 更省事也更稳妥。
struct VoteCandidate {
    size_t seedIdx = 0;
    std::vector<std::pair<int,int>> support;
    double avgErr = 0.0;
    bool   fullySupported = false;   // 支持视角数是否达到 effectiveMinSupport
};

} // namespace cluster_detail

inline ClusterResult clusterMultiView(
        const std::vector<EpiMat3>& Rs, const std::vector<EpiVec3>& ts,
        const std::vector<std::vector<std::array<double,2>>>& obsPerCam,
        double maxSampson, double maxReprojNorm, bool useLmRefine = true,
        const std::vector<std::vector<Cov2>>* obsCovPerCam = nullptr,
        double chiSquareGate = 9.21, int minSupportViews = -1, bool useVoting = true,
        double minRayAngleDeg = 0.0, double ambiguityMargin = 0.0,
        double lmHuberDelta = 0.01, int lmMaxIters = 15,
        double calibSigmaNorm = 0.0,
        bool allowTwoViewFallback = true, double twoViewMinRayAngleDeg = 10.0,
        double maxFinalResidualNorm = -1.0) {
    using namespace cluster_detail;
    ClusterResult out;
    const int nc = int(Rs.size());
    const size_t ncz = size_t(nc);
    out.unmatched.assign(ncz, {});
    if (nc < 2) {
        for (int c=0;c<nc;++c)
            for (int k=0;k<int(obsPerCam[size_t(c)].size());++k) out.unmatched[size_t(c)].push_back(k);
        return out;
    }

    // 实际生效的最小支持视角数：-1(自动)沿用旧行为；显式传入的值夹在
    // [2, nc]之间(见函数头注释)。
    const int effectiveMinSupport = (minSupportViews > 0)
        ? std::min(nc, std::max(2, minSupportViews))
        : ((nc == 2) ? 2 : 3);

    // 预计算所有相机对的本质矩阵——相机不动，算一次复用，见 Epipolar.hpp 注释。
    std::vector<std::vector<EpiMat3>> E(ncz, std::vector<EpiMat3>(ncz));
    for (int i=0;i<nc;++i) for (int j=0;j<nc;++j) if (i!=j)
        E[size_t(i)][size_t(j)] = essentialFromRelativePose(Rs[size_t(i)],ts[size_t(i)],Rs[size_t(j)],ts[size_t(j)]);

    // 候选种子：每对相机的两视图匹配 + 三角化出的初始点。
    struct Seed { int ci,cj,oi,oj; EpiVec3 point; double seedResidual; };
    std::vector<Seed> seeds;
    for (int i=0;i<nc;++i) for (int j=i+1;j<nc;++j) {
        auto mr = matchTwoViews(E[size_t(i)][size_t(j)], obsPerCam[size_t(i)], obsPerCam[size_t(j)], maxSampson);
        for (auto& mp : mr.pairs) {
            const double nx0=obsPerCam[size_t(i)][size_t(mp.i)][0], ny0=obsPerCam[size_t(i)][size_t(mp.i)][1];
            const double nx1=obsPerCam[size_t(j)][size_t(mp.j)][0], ny1=obsPerCam[size_t(j)][size_t(mp.j)][1];
            // 视线夹角过滤——在三角化之前就拦截，见函数头minRayAngleDeg
            // 的完整说明。跟maxReprojNorm是两道完全独立的门：这道看的是
            // "这两条视线的几何条件本身适不适合三角化"，不看噪声也不看
            // 三角化的具体结果，纯粹由相机相对姿态+观测方向决定。
            if (minRayAngleDeg > 0.0) {
                const EpiVec3 d0 = rayDir(Rs[size_t(i)], nx0, ny0);
                const EpiVec3 d1 = rayDir(Rs[size_t(j)], nx1, ny1);
                if (rayAngleDeg(d0, d1) < minRayAngleDeg) continue;
            }
            auto tr = triangulateRayPair(Rs[size_t(i)],ts[size_t(i)],nx0,ny0,
                                         Rs[size_t(j)],ts[size_t(j)],nx1,ny1);
            if (!tr.valid) continue;
            seeds.push_back({i,j,mp.i,mp.j,tr.point,tr.residual});
        }
    }
    // ---- 多相机投票（取代"按两视图残差顺序贪心抢跑"）----
    // 旧版按两视图种子的Sampson残差从小到大处理，先到先得地抢观测——这个
    // 顺序本身只反映了"这两台相机看起来像不像同一个点"，没用到第三台
    // 相机的信息。问题是：两点共线歧义/花生歧义场景下，一个错误配对
    // (幽灵)的两视图残差完全可能比正确配对更小(纯几何巧合，见文件头注释)，
    // 一旦它排在前面先把观测占了，真正有更多相机佐证的候选反而抢不到。
    //
    // 改成两阶段：先不认领任何观测，给每个种子独立地"考"一遍其余全部
    // 相机(不管观测有没有被别的种子看中)，得到它完整的支持集合(共识强度)；
    // 再按"支持视角数多的优先，打平比平均重投影误差"重新排序候选——这才
    // 是真正意义上的多相机投票：一个候选能不能赢，看的是有多少台独立
    // 相机认它，而不是它跟哪个种子最先被枚举到。真正提交(认领观测)时
    // 才检查是否与已提交的更强候选冲突，冲突就整体放弃(而不是抢救式地
    // 复用被抢走前的支持集合，那样容易产生跟别的track共享观测的脏数据)。
    // 协方差形状必须跟obsPerCam严格对齐才启用马氏距离，形状对不上(比如
    // 调用方传了旧代码路径、还没升级)一律诚实退化，不强行凑用——同
    // MarkerAssociator::associate() 的"cov维度对不上就诚实退化"是同一原则。
    bool useMahalanobis = (obsCovPerCam != nullptr) && (obsCovPerCam->size() == ncz);
    if (useMahalanobis) {
        for (int c=0;c<nc && useMahalanobis;++c)
            if ((*obsCovPerCam)[size_t(c)].size() != obsPerCam[size_t(c)].size()) useMahalanobis = false;
    }

    // VoteCandidate 定义见文件上方 cluster_detail 命名空间。函数顶部已有
    // "using namespace cluster_detail;"，不需要再单独声明别名——上一版在这里
    // 多写了一句 using 别名，反而跟 using namespace 撞出解析歧义，见类型定义
    // 处的说明。直接用 unqualified 的 VoteCandidate 即可。
    std::vector<VoteCandidate> candidates;
    candidates.reserve(seeds.size());
    // 【并行化】每个 seed 的投票完全独立：只读 obsPerCam / obsCovPerCam /
    // Rs / ts，产出一个独立的 candidate，seed 之间没有任何数据依赖。
    // 把循环体抽成 voteOne(si, out)——逻辑与原串行版一字不改，只是把
    // "candidates.push_back(x)" 换成 "写进调用方给的槽位"，这样并行时
    // 每个线程写自己的槽、互不冲突，且结果顺序严格等于串行顺序(按 si 索引
    // 落位，不是先算完先入队)，保证并行/串行输出逐位一致、可复现。
    // 槽位用 optional 语义表达"这个 seed 没产出候选"(原来的 continue)。
    // 【命名坑】这里绝对不能叫 slots——在 Qt 工程里 "slots" 是
    // qobjectdefs.h 定义的宏(#define slots Q_SLOTS，而 Q_SLOTS 展开为空)，
    // 只要这个翻译单元的 include 链路上有任何一个 QObject 头文件(本文件
    // 通过 HandTrackingWorker.hpp 这类调用方间接满足)，预处理器就会把
    // "std::vector<VoteCandidate> slots;" 展开成
    // "std::vector<VoteCandidate> ;"——变量名凭空消失，报的正是
    // "declaration does not declare anything"，下一行 "slots.resize(...)"
    // 则变成开头就是"."，报"expected primary-expression before '.' token"。
    // 上一版死活复现不出来，就是因为孤立编译这个头文件时没拉进 Qt 头、
    // 宏没定义，这是检查的盲区。教训：Qt 工程里 slots/signals/emit/
    // foreach/forever 都是保留字，标识符要避开这几个词。
    std::vector<VoteCandidate> voteSlots;
    voteSlots.resize(seeds.size());
    std::vector<char> slotUsed;
    slotUsed.assign(seeds.size(), 0);

    auto voteOne = [&](size_t si) {
        const auto& sd = seeds[si];
        std::vector<std::pair<int,int>> support = { {sd.ci,sd.oi}, {sd.cj,sd.oj} };

        if (!useVoting) {
            // 【新增】投票关闭：不去"考"其余相机，直接拿两视图种子本身当
            // 候选——相机数量少(尤其只有2台)时第三方投票本来就没有意义，
            // 省掉这一层O(nc)的额外重投影计算。排序阶段下面那行
            // support.size()对所有候选都恒为2，天然退化成纯按seedResidual
            // 从小到大排序，等价于文件头注释里说的"旧版贪心抢跑"逻辑。
            // effectiveMinSupport/minSupportViews在这个分支不生效——两视图
            // 种子只有2视角支持，要求更多没有意义。
            // 第4个字段 fullySupported：投票关闭时两视图种子恒为2视角支持，
            // 按旧语义算"达标"(minSupportViews 在这个分支本来就不生效)。
            VoteCandidate cand;
            cand.seedIdx        = si;
            cand.support        = std::move(support);
            cand.avgErr         = sd.seedResidual;
            cand.fullySupported = true;
            voteSlots[si] = std::move(cand);
            slotUsed[si] = 1;
            return;
        }

        double errSum = 0.0; int errCnt = 0;
        for (int k=0;k<nc;++k) {
            if (k==sd.ci || k==sd.cj) continue;
            // 注意：这一遍不看 claimed——投票阶段要的是"这个候选客观上有
            // 多少台相机的观测支持它"，跟别的候选有没有先抢到观测无关，
            // 抢占冲突留到下面提交阶段按共识强度顺序解决。
            //
            // 【性能】种子点在相机 k 上的重投影只依赖 sd.point 和相机 k，
            // 跟具体观测 o 无关——提到 o 循环外算一次，别在内层每个观测都
            // 重算一遍（原来 reprojectNorm 在内层被调 obs 次，纯重复计算）。
            // 稠密场景观测多时这一处就能明显减负，且结果完全不变。
            double px, py;
            if (!reprojectNorm(Rs[size_t(k)],ts[size_t(k)],sd.point,px,py)) continue;

            int bestObs=-1; double bestScore=1e18; double bestEuclid=1e18;
            double secondEuclid=1e18;   // 次佳观测的重投影距离(歧义边界用)
            for (int o=0;o<int(obsPerCam[size_t(k)].size());++o) {
                const double dx=px-obsPerCam[size_t(k)][size_t(o)][0], dy=py-obsPerCam[size_t(k)][size_t(o)][1];
                const double euclid = std::sqrt(dx*dx+dy*dy);
                if (euclid > maxReprojNorm) continue;   // 硬性兜底上限，双保险第一道，马氏距离算不算都先过这关
                double score = euclid;   // 退化情形：直接用欧氏距离当排序/门控分数
                if (useMahalanobis) {
                    const auto& cov = (*obsCovPerCam)[size_t(k)][size_t(o)];
                    // 【关键】检测层协方差只描述"检测噪声"，不含"标定偏差"。
                    // 这两者统计性质完全不同：检测噪声零均值、逐帧独立、多帧
                    // 平均会抵消；标定偏差是**系统性**的，同一台相机全画幅、
                    // 全时段恒定，不会因为多看几帧就变小。把它漏掉的后果实测
                    // 非常严重：0.5px 的主点误差就让召回率从 0.993 掉到 0.797、
                    // ID 跳变从 0 涨到 160；2px 主点误差直接掉到 0.13。
                    // 而标定模块在重投影误差 0.18px（看起来堪称完美）时，实际
                    // 主点误差就有 3.8px——也就是说"标定报告很漂亮"和"追踪跑得
                    // 动"之间没有必然联系，缺的就是这一项。
                    //
                    // 这里按方差可加性把标定不确定度并进去（两个独立误差源的
                    // 方差直接相加）。calibSigmaNorm 是归一化坐标下的标定等效
                    // 标准差，调用方按"标定报告里的主点/外参不确定度 ÷ 焦距"
                    // 估一个值；给 0 则完全退化成旧行为。
                    const double cs2 = calibSigmaNorm * calibSigmaNorm;
                    double inv[2][2];
                    if (!invert2x2(cov.xx + cs2, cov.xy, cov.yy + cs2, inv)) continue;   // 协方差退化，跳过这个候选观测
                    const double m2 = dx*dx*inv[0][0] + 2.0*dx*dy*inv[0][1] + dy*dy*inv[1][1];
                    if (m2 > chiSquareGate) continue;   // 马氏距离检验，第二道
                    score = m2;
                }
                if (score < bestScore) { secondEuclid=bestEuclid; bestScore=score; bestEuclid=euclid; bestObs=o; }
                else if (euclid < secondEuclid) { secondEuclid=euclid; }
            }
            if (bestObs>=0) {
                // 【歧义边界过滤】如果这台相机里"次佳观测"跟"最佳观测"的重投影
                // 距离差不出来(secondEuclid - bestEuclid < ambiguityMargin)，说明
                // 这台相机分不清这个候选点到底对应哪个观测——两个观测都像。这种
                // "分不清"的支持票不可靠：幽灵点常靠凑巧同时贴近好几个真观测来
                // 骗够支持视角数,这道闸门专门作废这种票,让幽灵凑不够 support 被丢。
                // 默认 ambiguityMargin<=0 关闭(保持旧行为)。用欧氏重投影距离判(可
                // 解释、跟maha开不开无关)。
                if (ambiguityMargin <= 0.0 || (secondEuclid - bestEuclid) >= ambiguityMargin) {
                    support.push_back({k,bestObs}); errSum+=bestEuclid; ++errCnt;
                }
            }
        }
        // 【改】支持视角门槛：从"硬性>=3"改成"几何最少2个，够不到
        // effectiveMinSupport 的走降级通道"。见函数头 allowTwoViewFallback 说明。
        const int nSup = int(support.size());
        if (nSup < 2) return;                         // 2 是三角化的几何下限，不可退让
        const bool fullySupported = nSup >= effectiveMinSupport;
        if (!fullySupported) {
            if (!allowTwoViewFallback) return;        // 关掉降级通道 = 旧行为
            // 降级通道的补偿条件：支持集里必须存在一对夹角足够大的视线。
            // 夹角小 = 深度方向病态 = 2视图幽灵的高发区，必须挡掉。
            if (twoViewMinRayAngleDeg > 0.0 &&
                bestSupportRayAngleDeg(sd.point, Rs, ts, support) < twoViewMinRayAngleDeg)
                return;
        }
        VoteCandidate cand;
        cand.seedIdx        = si;
        cand.support        = std::move(support);
        cand.avgErr         = (errCnt > 0) ? (errSum / errCnt) : sd.seedResidual;
        cand.fullySupported = fullySupported;
        voteSlots[si] = std::move(cand);
        slotUsed[si] = 1;
    };

    // 派发：seed 少时串行（开线程的固定开销反而更贵），多时按硬件并发数分块。
    // 阈值 64 是经验值——低于这个规模并行收益被线程创建/同步吃掉。
    const size_t kParallelMinSeeds = 64;
    size_t nThreads = 1;
#if MOCAP_HAS_STD_THREAD
    {
        unsigned hw = std::thread::hardware_concurrency();
        if (hw == 0) hw = 1;
        if (seeds.size() >= kParallelMinSeeds && hw > 1) {
            const size_t byWork = (seeds.size() + 31) / 32;   // 每线程至少摊到 32 个 seed
            nThreads = (size_t(hw) < byWork) ? size_t(hw) : byWork;
        }
    }
#endif
    if (nThreads <= 1) {
        for (size_t si = 0; si < seeds.size(); ++si) voteOne(si);
    }
#if MOCAP_HAS_STD_THREAD
    else {
        std::vector<std::thread> pool;
        pool.reserve(nThreads);
        const size_t chunk = (seeds.size() + nThreads - 1) / nThreads;
        for (size_t t = 0; t < nThreads; ++t) {
            const size_t lo = t * chunk;
            size_t hi = lo + chunk;
            if (hi > seeds.size()) hi = seeds.size();
            if (lo >= hi) break;
            pool.emplace_back([&voteOne, lo, hi]() { for (size_t si = lo; si < hi; ++si) voteOne(si); });
        }
        for (size_t t = 0; t < pool.size(); ++t) pool[t].join();
    }
#endif

    // 按 si 顺序收集，等价于串行时的 push_back 顺序。
    candidates.clear();
    for (size_t si=0; si<seeds.size(); ++si)
        if (slotUsed[si]) candidates.push_back(std::move(voteSlots[si]));

    // 共识强度优先：先按"是否达到完整支持视角数"分层(达标的先拿观测，降级
    // 通道出来的点不能抢在它们前面)，再按支持视角数，最后按平均重投影误差。
    std::sort(candidates.begin(), candidates.end(), [](const VoteCandidate&a,const VoteCandidate&b){
        if (a.fullySupported != b.fullySupported) return a.fullySupported;
        if (a.support.size() != b.support.size()) return a.support.size() > b.support.size();
        if (a.avgErr != b.avgErr) return a.avgErr < b.avgErr;
        // 【确定性】前三项全相等时必须还有一个稳定的打破平局的依据。
        // 原来这里直接 return false(视为等价)，而 std::sort 是【不稳定】排序：
        // 等价元素的相对顺序由实现决定，同样的输入可能排出不同的顺序，
        // 进而让下面"按顺序抢占观测"的结果不同——表现为同一份数据偶尔
        // 输出不一样的点集。稠密场景下 avgErr 完全相等并不罕见(对称构型、
        // 同一批共面点)。用 seedIdx(枚举顺序，天然唯一)兜底，保证严格确定。
        return a.seedIdx < b.seedIdx;
    });

    std::vector<std::vector<char>> claimed(ncz);
    for (int c=0;c<nc;++c) claimed[size_t(c)].assign(obsPerCam[size_t(c)].size(), 0);

    for (auto& cand : candidates) {
        // 提交前重新核对：投票阶段没看claimed，此刻可能有观测已经被更强
        // (排序更靠前)的候选拿走——整体放弃而不是"减员后凑合提交"，避免
        // 用一个本来是为另一个3D点算的观测拼出一个不自洽的track。
        // 【改】旧行为是"只要有一个观测被抢走就整体放弃这个候选"。
        // 这是手指并拢时"莫名其妙少一个点"的主要来源：两个真实标记点靠得近，
        // 排序靠前的那个先把某台相机的观测拿走，另一个真实点即使在其余相机里
        // 支持得好好的，也会被整体丢弃——丢的是一个真点，不是幽灵。
        // 现在改成：去掉被抢走的观测，用剩下的重新校验；只要剩余支持仍达标
        // 就重新三角化后提交。旧注释担心的"用为别的点算的观测拼出不自洽的
        // track"由两道闸门兜底：(1) 被抢走的观测已经被剔除，不会被复用；
        // (2) 重解之后过 maxFinalResidualNorm 残差闸门，不自洽的会被丢弃。
        std::vector<std::pair<int,int>> support;
        support.reserve(cand.support.size());
        for (auto& s : cand.support)
            if (!claimed[size_t(s.first)][size_t(s.second)]) support.push_back(s);

        const int nLeft = int(support.size());
        if (nLeft < 2) continue;                       // 几何下限
        bool stillFull = nLeft >= effectiveMinSupport;
        if (!stillFull && !allowTwoViewFallback) continue;
        if (nLeft != int(cand.support.size())) {
            // 减员了：重新按降级通道的标准检查视线夹角
            if (!stillFull && twoViewMinRayAngleDeg > 0.0 &&
                bestSupportRayAngleDeg(seeds[size_t(cand.seedIdx)].point, Rs, ts, support)
                    < twoViewMinRayAngleDeg)
                continue;
        }
        std::sort(support.begin(), support.end());
        std::vector<const EpiMat3*> Rv; std::vector<const EpiVec3*> tv;
        std::vector<std::array<double,2>> nxy;
        for (auto& s : support) {
            Rv.push_back(&Rs[size_t(s.first)]); tv.push_back(&ts[size_t(s.first)]);
            nxy.push_back(obsPerCam[size_t(s.first)][size_t(s.second)]);
        }
        EpiVec3 refined;
        if (!triangulateNView(Rv, tv, nxy, refined)) continue;

        double finalResidual = -1.0;
        if (useLmRefine) {
            std::vector<TriangulationObservation> refineObs;
            refineObs.reserve(support.size());
            for (size_t i=0;i<support.size();++i) refineObs.push_back({*Rv[i], *tv[i], nxy[i][0], nxy[i][1]});
            // 【新增】huberDelta/maxIters此前只能用TriangulationRefineConfig{}
            // 的默认值(0.01/15)，硬编码在这里，现在从函数参数传入，方便按实际
            // 场景(检测噪声、遮挡严重度)调整鲁棒IRLS加权的敏感度：huberDelta
            // 越小，越容易把观测判定为"坏视角"降权(更激进的抗幽灵/抗遮挡噪声，
            // 但正常噪声范围内的观测也可能被误伤而丢失信息)；越大则越宽容。
            TriangulationRefineConfig refineCfg;
            refineCfg.huberDelta = lmHuberDelta;
            refineCfg.maxIters = lmMaxIters;
            const auto refineResult = refineTriangulationLM(refined, refineObs, refineCfg);
            refined = refineResult.point;
            finalResidual = refineResult.weightedRmsNorm;
        } else {
            double sumErr=0; int cnt=0;
            for (auto& s : support) {
                double px,py;
                if (reprojectNorm(Rs[size_t(s.first)],ts[size_t(s.first)],refined,px,py)) {
                    const double dx=px-obsPerCam[size_t(s.first)][size_t(s.second)][0];
                    const double dy=py-obsPerCam[size_t(s.first)][size_t(s.second)][1];
                    sumErr += std::sqrt(dx*dx+dy*dy); ++cnt;
                }
            }
            finalResidual = cnt>0 ? sumErr/cnt : -1.0;
        }

        // 【新增】最终残差闸门：重解之后如果跟观测对不上，说明这组支持视角
        // 本来就不该凑在一起(典型情况是减员后剩下的视角其实属于另一个点)。
        if (maxFinalResidualNorm > 0.0 && finalResidual > 0.0 &&
            finalResidual > maxFinalResidualNorm) continue;

        Track3D track;
        track.point = {refined[0],refined[1],refined[2]};
        track.support = support;
        track.residual = finalResidual;
        // 【改】如实标记，不再恒为 true。下游(追踪器/UI)可以据此区分
        // "3+视角交叉验证过的点"和"只有2个视角撑着的点"——后者仍然是有效
        // 输出，只是置信度低一档，值得用更严的确认帧数或在UI上区别显示。
        track.verified = stillFull;
        out.tracks.push_back(track);
        for (auto& s : support) claimed[size_t(s.first)][size_t(s.second)] = 1;
    }

    for (int c=0;c<nc;++c)
        for (int o=0;o<int(claimed[size_t(c)].size());++o)
            if (!claimed[size_t(c)][size_t(o)]) out.unmatched[size_t(c)].push_back(o);
    return out;
}

} // namespace mocap
