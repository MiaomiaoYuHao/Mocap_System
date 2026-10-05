#pragma once
// ---------------------------------------------------------------------------
// 手背刚体位姿求解（Kabsch 配准）—— 纯数学，零 Qt 依赖，可单测。
//
// 给定手背 5 球的世界系三角化位置（可缺失 1-2 颗）+ 手背模板（手腕系已知
// 位置），求手腕系->世界系的刚体变换 (R,t)。是 hand_pose.py 的 C++ 移植，
// 算法已在 Python 侧用合成数据验证（噪声鲁棒、缺失点、坏点检测、端到端闭环、
// 翻转检测）。移植正确性由 test_hand_pose.cpp 独立复核。
//
// Kabsch 需要 3x3 SVD。这里自带一个紧凑的对称特征分解 + SVD（雅可比法），
// 跟 Triangulation.cpp 里的 smallestEigenvector4 同思路，不引入 Eigen。
// ---------------------------------------------------------------------------
#include <array>
#include <cmath>
#include <vector>

namespace mocap {

// 类型名加了 Hand 前缀（HandVec3/HandMat3），不用通用的 Vec3/Mat3——那两个字
// 太通用，直接躺在共享的 mocap 命名空间下，容易被以后别的模块（比如某天加个
// 通用线性代数工具）撞名，加前缀把冲突面收窄到这个文件自己的关注点上。
using HandVec3 = std::array<double, 3>;
using HandMat3 = std::array<double, 9>;   // 行主序

struct HandBackPose {
    HandMat3 R{};
    HandVec3 t{};
    int  nUsed = 0;
    double rms = -1.0;
    bool confident = false;
    bool flipped = false;
};

namespace detail {

inline HandMat3 matmul3(const HandMat3& A, const HandMat3& B) {
    HandMat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += A[i*3+k]*B[k*3+j];
            C[i*3+j] = s;
        }
    return C;
}
inline HandMat3 transpose3(const HandMat3& A) {
    return { A[0],A[3],A[6], A[1],A[4],A[7], A[2],A[5],A[8] };
}
inline double det3(const HandMat3& A) {
    return A[0]*(A[4]*A[8]-A[5]*A[7])
         - A[1]*(A[3]*A[8]-A[5]*A[6])
         + A[2]*(A[3]*A[7]-A[4]*A[6]);
}
inline HandVec3 matvec3(const HandMat3& A, const HandVec3& v) {
    return { A[0]*v[0]+A[1]*v[1]+A[2]*v[2],
             A[3]*v[0]+A[4]*v[1]+A[5]*v[2],
             A[6]*v[0]+A[7]*v[1]+A[8]*v[2] };
}

// 对称 3x3 矩阵的雅可比特征分解：A = V diag(w) V^T，返回特征向量矩阵 V（列为
// 特征向量）。用于 SVD：对 M^T M 和 M M^T 各做一次。
inline void jacobiEigenSym3(const HandMat3& Ain, HandMat3& V, HandVec3& w) {
    double A[3][3];
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) A[i][j]=Ain[i*3+j];
    double Vm[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (int sweep=0; sweep<50; ++sweep) {
        // 最大非对角元
        int p=0,q=1; double mx=std::abs(A[0][1]);
        if (std::abs(A[0][2])>mx){mx=std::abs(A[0][2]);p=0;q=2;}
        if (std::abs(A[1][2])>mx){mx=std::abs(A[1][2]);p=1;q=2;}
        if (mx < 1e-18) break;
        double app=A[p][p], aqq=A[q][q], apq=A[p][q];
        double phi=0.5*std::atan2(2*apq, aqq-app);
        double c=std::cos(phi), s=std::sin(phi);
        for (int i=0;i<3;++i){ double aip=A[i][p],aiq=A[i][q];
            A[i][p]=c*aip-s*aiq; A[i][q]=s*aip+c*aiq; }
        for (int i=0;i<3;++i){ double api=A[p][i],aqi=A[q][i];
            A[p][i]=c*api-s*aqi; A[q][i]=s*api+c*aqi; }
        for (int i=0;i<3;++i){ double vip=Vm[i][p],viq=Vm[i][q];
            Vm[i][p]=c*vip-s*viq; Vm[i][q]=s*vip+c*viq; }
    }
    for (int i=0;i<3;++i){ w[i]=A[i][i];
        for (int j=0;j<3;++j) V[j*3+i]=Vm[j][i]; }
}

} // namespace detail

// Kabsch：求 R,t 使 R*P_i + t ≈ Q_i。P,Q 为对应点集（同长度，>=3）。
// R 保证是纯旋转（det=+1，排除镜像）。
inline void kabsch(const std::vector<HandVec3>& P, const std::vector<HandVec3>& Q,
                   HandMat3& R, HandVec3& t) {
    using namespace detail;
    const int n = int(P.size());
    HandVec3 Pc{0,0,0}, Qc{0,0,0};
    for (int i=0;i<n;++i) for (int k=0;k<3;++k){ Pc[k]+=P[i][k]; Qc[k]+=Q[i][k]; }
    for (int k=0;k<3;++k){ Pc[k]/=n; Qc[k]/=n; }

    // H = sum (P-Pc)(Q-Qc)^T  (3x3)
    HandMat3 H{};
    for (int i=0;i<n;++i){
        double px=P[i][0]-Pc[0], py=P[i][1]-Pc[1], pz=P[i][2]-Pc[2];
        double qx=Q[i][0]-Qc[0], qy=Q[i][1]-Qc[1], qz=Q[i][2]-Qc[2];
        H[0]+=px*qx; H[1]+=px*qy; H[2]+=px*qz;
        H[3]+=py*qx; H[4]+=py*qy; H[5]+=py*qz;
        H[6]+=pz*qx; H[7]+=pz*qy; H[8]+=pz*qz;
    }

    // SVD via eigendecomposition: H = U S V^T
    // V = eigenvectors of H^T H ; U = eigenvectors of H H^T
    HandMat3 HtH = matmul3(transpose3(H), H);
    HandMat3 HHt = matmul3(H, transpose3(H));
    HandMat3 Vv, Uu; HandVec3 wv, wu;
    jacobiEigenSym3(HtH, Vv, wv);
    jacobiEigenSym3(HHt, Uu, wu);

    // 需要把 U 的列符号对齐到 H V = U S（雅可比返回的特征向量符号/顺序不定）。
    // 稳妥做法：直接用 R = V * diag(1,1,d) * U^T 的经典 Kabsch 公式，但要保证
    // U,V 的奇异值排序一致。这里按特征值降序重排两边。
    int vo[3]={0,1,2}, uo[3]={0,1,2};
    auto sortDesc=[](HandVec3& w,int* o){
        for(int a=0;a<3;++a)for(int b=a+1;b<3;++b)
            if(w[o[b]]>w[o[a]]){int tmp=o[a];o[a]=o[b];o[b]=tmp;}
    };
    sortDesc(wv,vo); sortDesc(wu,uo);

    HandMat3 Vs{}, Us{};
    for(int c=0;c<3;++c){
        for(int r=0;r<3;++r){ Vs[r*3+c]=Vv[r*3+vo[c]]; Us[r*3+c]=Uu[r*3+uo[c]]; }
    }
    // 修正每一列 U 的符号，使 H*V_col 与 U_col 同向（保证 H=U S V^T 而非带符号翻转）
    for(int c=0;c<3;++c){
        HandVec3 vcol{Vs[c],Vs[3+c],Vs[6+c]};
        HandVec3 hv=matvec3(H,vcol);
        HandVec3 ucol{Us[c],Us[3+c],Us[6+c]};
        double dot=hv[0]*ucol[0]+hv[1]*ucol[1]+hv[2]*ucol[2];
        if(dot<0){ Us[c]=-Us[c]; Us[3+c]=-Us[3+c]; Us[6+c]=-Us[6+c]; }
    }

    // R = V * diag(1,1,d) * U^T，d=sign(det(V U^T)) 排除镜像
    HandMat3 Ut = transpose3(Us);
    HandMat3 VUt = matmul3(Vs, Ut);
    double d = det3(VUt) < 0 ? -1.0 : 1.0;
    HandMat3 Dm = { 1,0,0, 0,1,0, 0,0,d };
    R = matmul3(matmul3(Vs, Dm), Ut);

    // t = Qc - R Pc
    HandVec3 RPc = matvec3(R, Pc);
    t = { Qc[0]-RPc[0], Qc[1]-RPc[1], Qc[2]-RPc[2] };
}

// 手背位姿主求解。template5/observed5：手背5球（手腕系模板 / 世界系观测）。
// visibleMask：5 个 bool，哪几颗可见。upHintLocal/World（可选，非零启用）：冷启动
// 翻转检测的先验方向。返回见 HandBackPose。
inline HandBackPose solveHandBackPose(
        const std::array<HandVec3,5>& templ, const std::array<HandVec3,5>& observed,
        const std::array<bool,5>& visibleMask, int minPoints = 3,
        const HandVec3* upHintLocal = nullptr, const HandVec3* upHintWorld = nullptr) {
    using namespace detail;
    HandBackPose out;

    std::vector<HandVec3> P, Q;
    for (int i=0;i<5;++i) if (visibleMask[i]) { P.push_back(templ[i]); Q.push_back(observed[i]); }
    out.nUsed = int(P.size());
    if (out.nUsed < minPoints) { out.confident=false; return out; }

    kabsch(P, Q, out.R, out.t);

    // 配准残差 RMS
    double sq=0;
    for (size_t i=0;i<P.size();++i){
        HandVec3 rp=matvec3(out.R,P[i]);
        double ex=rp[0]+out.t[0]-Q[i][0];
        double ey=rp[1]+out.t[1]-Q[i][1];
        double ez=rp[2]+out.t[2]-Q[i][2];
        sq += ex*ex+ey*ey+ez*ez;
    }
    out.rms = std::sqrt(sq / double(P.size()));

    // 翻转检测
    out.flipped = false;
    if (upHintLocal && upHintWorld) {
        HandVec3 mapped = matvec3(out.R, *upHintLocal);
        double dot = mapped[0]*(*upHintWorld)[0]+mapped[1]*(*upHintWorld)[1]+mapped[2]*(*upHintWorld)[2];
        if (dot < 0) out.flipped = true;
    }

    out.confident = (out.rms < 5.0) && (out.nUsed >= minPoints) && (!out.flipped);
    return out;
}

// 把手腕系点变换到世界系
inline HandVec3 transformPoint(const HandVec3& p, const HandMat3& R, const HandVec3& t) {
    return { R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0],
             R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1],
             R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2] };
}

// ---------------------------------------------------------------------------
// 手背模板几何体检——防"实测替换占位坐标时悄悄引入退化配置"。
//
// solveHandBackPose 最少 3 颗手背球可解，但数学上 3 点共线时绕那条线的旋转
// 是欠定的（Kabsch 的 H 矩阵沿该方向奇异），且这种退化在无噪声/低噪声时
// rms 残差检查未必能拦住——共线点配准可以有很小的残差，只是绕共线轴的
// 旋转本身没法唯一确定。
//
// 手背 5 球具体贴哪，遮挡时剩哪 3 颗不由算法控制（由实际拍摄角度决定），
// 所以稳健的要求是"任意 3 颗组合都不能接近共线"，而不是"至少存在一组够好
// 的组合"。这里枚举全部 C(5,3)=10 种三点组合，用三角形面积衡量共线程度
// （面积越接近0越危险），取最坏的一组。
//
// 用法：手背 marker 坐标从占位值换成实测值后，重跑一遍带这个检查的测试
// （见 test_hand_pose.cpp 新增的模板体检用例），面积不达标会明确报出是
// 哪三颗球的组合有问题，而不是等到真实遮挡场景下才发现姿态解不稳定。
struct TemplateGeometryCheck {
    bool ok = true;
    double minTriangleAreaMm2 = 0.0;
    int worstI = -1, worstJ = -1, worstK = -1;   // 面积最小（最接近退化）的那一组
};

inline double triangleArea3(const HandVec3& a, const HandVec3& b, const HandVec3& c) {
    const double ux = b[0]-a[0], uy = b[1]-a[1], uz = b[2]-a[2];
    const double vx = c[0]-a[0], vy = c[1]-a[1], vz = c[2]-a[2];
    const double cx = uy*vz - uz*vy, cy = uz*vx - ux*vz, cz = ux*vy - uy*vx;
    return 0.5 * std::sqrt(cx*cx + cy*cy + cz*cz);
}

// minAreaMm2 默认 100 mm²——相当于一个边长约 15mm 的等边三角形，是"明显不共线"
// 与"接近共线、数值上开始不稳"之间一个宽松但不算随意的分界；如果你的手背
// marker 排布本来就更紧凑，酌情调低，但调低前想清楚这是在放宽稳健性下限。
inline TemplateGeometryCheck checkHandBackTemplateGeometry(
        const std::array<HandVec3, 5>& templ, double minAreaMm2 = 100.0) {
    TemplateGeometryCheck r;
    r.minTriangleAreaMm2 = 1e300;
    for (int i = 0; i < 5; ++i)
        for (int j = i+1; j < 5; ++j)
            for (int k = j+1; k < 5; ++k) {
                const double a = triangleArea3(templ[i], templ[j], templ[k]);
                if (a < r.minTriangleAreaMm2) {
                    r.minTriangleAreaMm2 = a;
                    r.worstI = i; r.worstJ = j; r.worstK = k;
                }
            }
    r.ok = r.minTriangleAreaMm2 >= minAreaMm2;
    return r;
}

} // namespace mocap
