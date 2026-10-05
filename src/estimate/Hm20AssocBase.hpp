// ===========================================================================
// Hm20AssocBase.hpp —— hm20 关联的基础定义：点/矩阵别名、21 类标签体系、
// 连线拓扑、以及 mocap::hm20::detail 里的 3x3 小线代。
// 从 HandSkeletonAssociator.hpp 拆出（原第 45~374 行）。
// ===========================================================================
#pragma once

#include <array>
#include <deque>
#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>
#include <cstdio>

namespace mocap {
namespace hm20 {
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;   // 行主序
using Quat = std::array<double, 4>;   // (w,x,y,z)

// ===========================================================================
// 标签体系 —— 与 Python 侧 topology.py 逐位对齐，这是跨语言契约，不要改动编号
// ===========================================================================
enum Label : int {
    Dorsum0 = 0, Dorsum1, Dorsum2, Dorsum3, Dorsum4,
    ThumbMc = 5, ThumbPp, ThumbDp,          // 拇指是 掌骨/近节/远节
    IndexPp = 8, IndexMp, IndexDp,          // 其余四指是 近节/中节/远节
    MiddlePp = 11, MiddleMp, MiddleDp,
    RingPp = 14, RingMp, RingDp,
    PinkyPp = 17, PinkyMp, PinkyDp,
    Ghost = 20                              // dustbin：幽灵点/杂点/被遮挡标签
};
// 标签的英文短名，给界面标注用。刻意用小写 + 短，因为它要画在点旁边，
// 20 个标签同屏时长名字会糊成一片。
// 命名对齐上面的 enum：pp=proximal, mp=middle, dp=distal, mc=metacarpal。
inline const char* labelShortName(int label) {
    switch (label) {
        // 手背 5 点：b0..b4 只有两个字符，补个 k 凑齐三位对齐更好看
        case Dorsum0:  return "bk0";
        case Dorsum1:  return "bk1";
        case Dorsum2:  return "bk2";
        case Dorsum3:  return "bk3";
        case Dorsum4:  return "bk4";
        // 手指：首字母 + 节段。t=thumb i=index m=middle r=ring l=little
        // 【小指用 l 不是 p】pinky 的 p 会跟 pp(proximal) 撞成 "ppp"，
        // 读起来分不清哪个 p 是手指哪个是节段；little finger 是标准解剖学叫法。
        case ThumbMc:  return "tmc";   // 拇指多一节掌骨
        case ThumbPp:  return "tpp";
        case ThumbDp:  return "tdp";
        case IndexPp:  return "ipp";
        case IndexMp:  return "imp";
        case IndexDp:  return "idp";
        case MiddlePp: return "mpp";
        case MiddleMp: return "mmp";
        case MiddleDp: return "mdp";
        case RingPp:   return "rpp";
        case RingMp:   return "rmp";
        case RingDp:   return "rdp";
        case PinkyPp:  return "lpp";
        case PinkyMp:  return "lmp";
        case PinkyDp:  return "ldp";
        case Ghost:    return "gho";
        default:       return "";
    }
}

constexpr int kNumMarkers = 20;
constexpr int kNumClasses = 21;
constexpr int kNumSegments = 16;            // 0=手背, 1..15=各指节
constexpr int kMaxPoints = 26;              // 与 net.py MAX_POINTS 一致
// 候选点到"手部中位点"的最大容许距离(mm)。超过就不送进网络 —— 见
// Hm20Assoc_process.ipp 里 process() 开头那段。
// 【为什么是 600】真机手掌展布(最远点到中位点)实测约 98mm，人手最大也就
// 200mm 出头。600 留了三倍余量，任何真实的标记球都不会被误伤；而实测那个
// 坏点在 11192mm，差着一个半数量级，两者之间宽得根本不需要调参。
constexpr double kCandMaxSpreadMm = 600.0;
// 【修改】32 -> 26。20 个真实标记 + 余量。留 32 的代价是 Sinkhorn/匈牙利都按
// N 的规模放大，且给杂点更多进入指派矩阵的机会；实测 26 足够覆盖手部区域的
// 候选点数，超出部分本来就会被 process() 里的 std::min 截断。

inline const char* labelName(int i) {
    static const char* n[kNumClasses] = {
        "dorsum0","dorsum1","dorsum2","dorsum3","dorsum4",
        "thumb_mc","thumb_pp","thumb_dp",
        "index_pp","index_mp","index_dp",
        "middle_pp","middle_mp","middle_dp",
        "ring_pp","ring_mp","ring_dp",
        "pinky_pp","pinky_mp","pinky_dp","ghost"};
    return (i >= 0 && i < kNumClasses) ? n[i] : "?";
}

// marker -> segment：手背 5 点都挂在 segment 0，手指第 k 个点挂在自己那节骨
inline int markerSegment(int m) { return (m < 5) ? 0 : (1 + (m - 5)); }

// ---------------------------------------------------------------------------
// 连线拓扑：15 条边，6 个互相独立的连通分量
// ---------------------------------------------------------------------------
struct Polyline {
    const char* name;
    bool closed;                 // 手背环闭合，手指折线开放
    std::array<int, 5> idx;
    int n;
};

inline const std::array<Polyline, 6>& polylines() {
    static const std::array<Polyline, 6> p = {{
        {"dorsum", true,  {0, 1, 2, 3, 4}, 5},
        {"thumb",  false, {5, 6, 7, -1, -1}, 3},
        {"index",  false, {8, 9, 10, -1, -1}, 3},
        {"middle", false, {11, 12, 13, -1, -1}, 3},
        {"ring",   false, {14, 15, 16, -1, -1}, 3},
        {"pinky",  false, {17, 18, 19, -1, -1}, 3},
    }};
    return p;
}

// 展开成 15 条边。手背 0-1-2-3-4-0 恒为简单（不自交）五边形：编号规则是
// "在手背平面内绕 5 点质心按极角排序"，而质心必在 5 点凸包内 => 多边形关于
// 质心星形 => 必然不自交、只有外围一圈、内部为空。所以手背点【用户随便贴】
// 都成立，不需要任何运行时求解。
inline const std::vector<std::pair<int, int>>& skeletonEdges() {
    static const std::vector<std::pair<int, int>> e = [] {
        std::vector<std::pair<int, int>> v;
        for (const auto& pl : polylines()) {
            for (int k = 0; k + 1 < pl.n; ++k) v.push_back({pl.idx[size_t(k)], pl.idx[size_t(k + 1)]});
            if (pl.closed) v.push_back({pl.idx[size_t(pl.n - 1)], pl.idx[0]});
        }
        return v;
    }();
    return e;   // size() == 15
}

// ===========================================================================
// 小型线代（3x3）
// ===========================================================================
namespace detail {

inline Vec3 sub(const Vec3& a, const Vec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
inline Vec3 add(const Vec3& a, const Vec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
inline Vec3 mul(const Vec3& a, double s)      { return {a[0]*s, a[1]*s, a[2]*s}; }
inline double dot(const Vec3& a, const Vec3& b){ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
inline Vec3 cross(const Vec3& a, const Vec3& b){
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(const Vec3& a) {
    const double n = norm(a);
    return n < 1e-12 ? Vec3{1, 0, 0} : Vec3{a[0]/n, a[1]/n, a[2]/n};
}
inline Vec3 matVec(const Mat3& R, const Vec3& v) {
    return {R[0]*v[0]+R[1]*v[1]+R[2]*v[2], R[3]*v[0]+R[4]*v[1]+R[5]*v[2], R[6]*v[0]+R[7]*v[1]+R[8]*v[2]};
}
inline Vec3 matVecT(const Mat3& R, const Vec3& v) {   // R^T * v
    return {R[0]*v[0]+R[3]*v[1]+R[6]*v[2], R[1]*v[0]+R[4]*v[1]+R[7]*v[2], R[2]*v[0]+R[5]*v[1]+R[8]*v[2]};
}
inline Mat3 matMul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            double s = 0;
            for (int k = 0; k < 3; ++k) s += A[size_t(i*3+k)] * B[size_t(k*3+j)];
            C[size_t(i*3+j)] = s;
        }
    return C;
}
inline Mat3 transpose(const Mat3& A) {
    return {A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]};
}
inline double det3(const Mat3& A) {
    return A[0]*(A[4]*A[8]-A[5]*A[7]) - A[1]*(A[3]*A[8]-A[5]*A[6]) + A[2]*(A[3]*A[7]-A[4]*A[6]);
}

// 对称 3x3 的 Jacobi 特征分解：A = V diag(w) V^T，w 降序。用于 Kabsch 的 SVD。
inline void jacobiEigenSym3(const Mat3& A_in, Mat3& V, Vec3& w) {
    Mat3 A = A_in;
    V = {1,0,0, 0,1,0, 0,0,1};
    for (int sweep = 0; sweep < 24; ++sweep) {
        double off = A[1]*A[1] + A[2]*A[2] + A[5]*A[5];
        if (off < 1e-24) break;
        const int pq[3][2] = {{0,1}, {0,2}, {1,2}};
        for (auto& e : pq) {
            const int p = e[0], q = e[1];
            const double apq = A[size_t(p*3+q)];
            if (std::abs(apq) < 1e-18) continue;
            const double app = A[size_t(p*3+p)], aqq = A[size_t(q*3+q)];
            const double theta = (aqq - app) / (2.0 * apq);
            const double t = (theta >= 0 ? 1.0 : -1.0) /
                             (std::abs(theta) + std::sqrt(theta*theta + 1.0));
            const double c = 1.0 / std::sqrt(t*t + 1.0), s = t * c;
            Mat3 J = {1,0,0, 0,1,0, 0,0,1};
            J[size_t(p*3+p)] = c; J[size_t(q*3+q)] = c;
            J[size_t(p*3+q)] = s; J[size_t(q*3+p)] = -s;
            A = matMul(matMul(transpose(J), A), J);
            V = matMul(V, J);
        }
    }
    w = {A[0], A[4], A[8]};
    // 按特征值降序重排列（列 = 特征向量）
    int ord[3] = {0, 1, 2};
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (w[size_t(ord[j])] > w[size_t(ord[i])]) std::swap(ord[i], ord[j]);
    Mat3 V2{}; Vec3 w2{};
    for (int c = 0; c < 3; ++c) {
        w2[size_t(c)] = w[size_t(ord[c])];
        for (int r = 0; r < 3; ++r) V2[size_t(r*3+c)] = V[size_t(r*3+ord[c])];
    }
    V = V2; w = w2;
}

// 旋转矩阵 -> 四元数 (w,x,y,z)，Shepperd 分支法，数值稳定
inline Quat matToQuat(const Mat3& R) {
    const double tr = R[0] + R[4] + R[8];
    double w, x, y, z;
    if (tr > 0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s; x = (R[7]-R[5]) / s; y = (R[2]-R[6]) / s; z = (R[3]-R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        const double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
        w = (R[7]-R[5]) / s; x = 0.25 * s; y = (R[1]+R[3]) / s; z = (R[2]+R[6]) / s;
    } else if (R[4] > R[8]) {
        const double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
        w = (R[2]-R[6]) / s; x = (R[1]+R[3]) / s; y = 0.25 * s; z = (R[5]+R[7]) / s;
    } else {
        const double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
        w = (R[3]-R[1]) / s; x = (R[2]+R[6]) / s; y = (R[5]+R[7]) / s; z = 0.25 * s;
    }
    if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
    const double n = std::sqrt(w*w + x*x + y*y + z*z);
    return {w/n, x/n, y/n, z/n};
}

// 四元数 (w,x,y,z) -> 旋转矩阵（行主序）。matToQuat 的逆。
inline Mat3 quatToMat(const Quat& q) {
    const double w=q[0], x=q[1], y=q[2], z=q[3];
    return {1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w),
            2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w),
            2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)};
}

// Kabsch：求 R,t 使 sum |R*src_i + t - dst_i|^2 最小（无缩放，det(R)=+1）
// 返回 RMSE；点数 < 3 时返回 -1 表示失败。
inline double kabsch(const std::vector<Vec3>& src, const std::vector<Vec3>& dst,
                     Mat3& R, Vec3& t) {
    const size_t n = src.size();
    if (n < 3 || dst.size() != n) return -1.0;
    Vec3 cs{0,0,0}, cd{0,0,0};
    for (size_t i = 0; i < n; ++i) { cs = add(cs, src[i]); cd = add(cd, dst[i]); }
    cs = mul(cs, 1.0 / double(n)); cd = mul(cd, 1.0 / double(n));

    Mat3 H{};   // H = sum (src-cs)(dst-cd)^T
    for (size_t i = 0; i < n; ++i) {
        const Vec3 a = sub(src[i], cs), b = sub(dst[i], cd);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) H[size_t(r*3+c)] += a[size_t(r)] * b[size_t(c)];
    }
    // SVD via 对称分解：H^T H = V S^2 V^T，U = H V S^-1，R = V diag(1,1,d) U^T ... 
    // 这里直接对 H H^T 与 H^T H 各做一次，避免小奇异值除零。
    Mat3 HtH = matMul(transpose(H), H);
    Mat3 V; Vec3 w;
    jacobiEigenSym3(HtH, V, w);
    Vec3 sv = {std::sqrt(std::max(w[0], 0.0)), std::sqrt(std::max(w[1], 0.0)),
               std::sqrt(std::max(w[2], 0.0))};
    if (sv[0] < 1e-9) return -1.0;                 // 全部点重合，退化
    // U 的列 = H * V_col / sigma；第三列用叉乘补，避免 sigma3≈0（共线/共面）时炸掉
    Mat3 U{};
    for (int c = 0; c < 2; ++c) {
        Vec3 vc = {V[size_t(0*3+c)], V[size_t(1*3+c)], V[size_t(2*3+c)]};
        Vec3 uc = matVec(H, vc);
        const double s = std::max(sv[size_t(c)], 1e-12);
        uc = mul(uc, 1.0 / s);
        uc = normalize(uc);
        for (int r = 0; r < 3; ++r) U[size_t(r*3+c)] = uc[size_t(r)];
    }
    {
        Vec3 u0 = {U[0], U[3], U[6]}, u1 = {U[1], U[4], U[7]};
        Vec3 u2 = normalize(cross(u0, u1));
        for (int r = 0; r < 3; ++r) U[size_t(r*3+2)] = u2[size_t(r)];
        Vec3 v0 = {V[0], V[3], V[6]}, v1 = {V[1], V[4], V[7]};
        Vec3 v2 = normalize(cross(v0, v1));
        for (int r = 0; r < 3; ++r) V[size_t(r*3+2)] = v2[size_t(r)];
    }
    // R = U V^T（注意 H 的定义方向：src->dst，所以是 U*V^T 的转置关系）
    Mat3 Rm = matMul(U, transpose(V));
    Rm = transpose(Rm);                            // 对应 H = Σ a b^T 的方向
    if (det3(Rm) < 0) {                            // 反射修正：翻最小奇异向量
        for (int r = 0; r < 3; ++r) V[size_t(r*3+2)] = -V[size_t(r*3+2)];
        Rm = transpose(matMul(U, transpose(V)));
    }
    R = Rm;
    t = sub(cd, matVec(R, cs));

    double se = 0;
    for (size_t i = 0; i < n; ++i) {
        const Vec3 d = sub(add(matVec(R, src[i]), t), dst[i]);
        se += dot(d, d);
    }
    return std::sqrt(se / double(n));
}

// ---------------------------------------------------------------------------
// 匈牙利算法（JV 最短增广路），矩形代价矩阵，rows <= cols，最小化总代价。
// assignment[r] = 该行分到的列，未分配为 -1。O(rows^2 * cols)。
// ---------------------------------------------------------------------------
inline void hungarian(const std::vector<double>& cost, int nr, int nc,
                      std::vector<int>& assignment) {
    const double INF = std::numeric_limits<double>::infinity();
    std::vector<double> u(size_t(nr) + 1, 0.0), v(size_t(nc) + 1, 0.0);
    std::vector<int> p(size_t(nc) + 1, 0), way(size_t(nc) + 1, 0);
    for (int i = 1; i <= nr; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(size_t(nc) + 1, INF);
        std::vector<char> used(size_t(nc) + 1, 0);
        do {
            used[size_t(j0)] = 1;
            const int i0 = p[size_t(j0)];
            double delta = INF; int j1 = 0;
            for (int j = 1; j <= nc; ++j) if (!used[size_t(j)]) {
                const double cur = cost[size_t((i0 - 1) * nc + (j - 1))] - u[size_t(i0)] - v[size_t(j)];
                if (cur < minv[size_t(j)]) { minv[size_t(j)] = cur; way[size_t(j)] = j0; }
                if (minv[size_t(j)] < delta) { delta = minv[size_t(j)]; j1 = j; }
            }
            for (int j = 0; j <= nc; ++j) {
                if (used[size_t(j)]) { u[size_t(p[size_t(j)])] += delta; v[size_t(j)] -= delta; }
                else                 { minv[size_t(j)] -= delta; }
            }
            j0 = j1;
        } while (p[size_t(j0)] != 0);
        do {
            const int j1 = way[size_t(j0)];
            p[size_t(j0)] = p[size_t(j1)];
            j0 = j1;
        } while (j0);
    }
    assignment.assign(size_t(nr), -1);
    for (int j = 1; j <= nc; ++j) if (p[size_t(j)] > 0)
        assignment[size_t(p[size_t(j)] - 1)] = j - 1;
}

// 二维线段真交叉判定（共享端点不算），用于五边形自交校验
inline bool segsCross(const std::array<double,2>& a, const std::array<double,2>& b,
                      const std::array<double,2>& c, const std::array<double,2>& d) {
    auto cr = [](const std::array<double,2>& o, const std::array<double,2>& p,
                 const std::array<double,2>& q) {
        return (p[0]-o[0])*(q[1]-o[1]) - (p[1]-o[1])*(q[0]-o[0]);
    };
    const double d1 = cr(c, d, a), d2 = cr(c, d, b), d3 = cr(a, b, c), d4 = cr(a, b, d);
    return (d1 * d2 < -1e-12) && (d3 * d4 < -1e-12);
}

} // namespace detail

} // namespace hm20
} // namespace mocap
