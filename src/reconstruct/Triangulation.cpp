#include "reconstruct/Triangulation.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace mocap {

void undistortNormalize(const CameraIntrinsics& intr, double px, double py,
                         double& nx, double& ny) {
    // 先用内参把像素坐标转成"假设无畸变"的归一化坐标，作为迭代初值。
    const double xd = (px - intr.cx) / intr.fx;
    const double yd = (py - intr.cy) / intr.fy;

    double x = xd, y = yd;
    // 定点迭代去畸变。畸变量通常不大，20 次迭代对常见镜头畸变系数
    // 足够收敛；每次迭代开销很小（几次乘加），不必做提前退出判断。
    for (int i = 0; i < 20; ++i) {
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double icdist = 1.0 / (1.0 + intr.k1 * r2 + intr.k2 * r4 + intr.k3 * r6);
        const double deltaX = 2.0 * intr.p1 * x * y + intr.p2 * (r2 + 2.0 * x * x);
        const double deltaY = intr.p1 * (r2 + 2.0 * y * y) + 2.0 * intr.p2 * x * y;
        x = (xd - deltaX) * icdist;
        y = (yd - deltaY) * icdist;
    }
    nx = x;
    ny = y;
}

std::array<double, 3> cameraCenterWorld(const CameraExtrinsics& extr) {
    // 与 CalibResultView::PoseView3D::fitScale() 里用的公式是同一套，
    // 后续可以考虑把这个函数挪到 CameraExtrinsics 自己身上做成方法，
    // 避免同一个公式在两个文件里各写一份。
    const auto& R = extr.R;
    const auto& t = extr.t;
    return {
        -(R[0] * t[0] + R[3] * t[1] + R[6] * t[2]),
        -(R[1] * t[0] + R[4] * t[1] + R[7] * t[2]),
        -(R[2] * t[0] + R[5] * t[1] + R[8] * t[2])
    };
}

std::array<double, 3> rayDirectionWorld(const CameraExtrinsics& extr, double nx, double ny) {
    // 相机系下的方向 (nx,ny,1)，转世界系：world = R^T * camDir
    // （Xc=R·Xw+t 这个约定下，方向向量的坐标变换不含平移，用 R^T）。
    const auto& R = extr.R;
    std::array<double, 3> d = {
        R[0] * nx + R[3] * ny + R[6] * 1.0,
        R[1] * nx + R[4] * ny + R[7] * 1.0,
        R[2] * nx + R[5] * ny + R[8] * 1.0
    };
    const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (n > 1e-12) { d[0] /= n; d[1] /= n; d[2] /= n; }
    return d;
}

TriangulateResult closestPointBetweenRays(
    const std::array<double, 3>& c0, const std::array<double, 3>& d0,
    const std::array<double, 3>& c1, const std::array<double, 3>& d1) {
    // 经典的"两条直线最近点"闭式解。两线 P0=c0+s*d0, P1=c1+t*d1（d0,d1单位向量）。
    // 令 w0=c0-c1，对 s,t 各自求偏导=0，解一个 2x2 线性方程组即可，
    // 不需要 SVD——这是只处理两台相机场景才能享受到的简化。
    const std::array<double, 3> w0 = { c0[0] - c1[0], c0[1] - c1[1], c0[2] - c1[2] };
    const double a = d0[0] * d0[0] + d0[1] * d0[1] + d0[2] * d0[2];   // ==1（单位向量）
    const double b = d0[0] * d1[0] + d0[1] * d1[1] + d0[2] * d1[2];
    const double c = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2];   // ==1
    const double d = d0[0] * w0[0] + d0[1] * w0[1] + d0[2] * w0[2];
    const double e = d1[0] * w0[0] + d1[1] * w0[1] + d1[2] * w0[2];
    const double denom = a * c - b * b;

    TriangulateResult r;
    // denom≈0 表示两条视线（近似）平行——两台相机基线夹角太小，三角化
    // 在几何上是退化的，深度方向的误差会被急剧放大。这种情况不是"算法
    // 出问题"，是拍摄几何本身不支持三角化，直接判失败，让上层去处理
    // （比如提示"两台相机夹角太小，挪开一点或换个角度再试"），而不是
    // 硬凑一个不可信的数值出来。
    if (std::abs(denom) < 1e-9) { r.valid = false; return r; }

    const double s = (b * e - c * d) / denom;
    const double t = (a * e - b * d) / denom;

    const std::array<double, 3> q0 = { c0[0] + s * d0[0], c0[1] + s * d0[1], c0[2] + s * d0[2] };
    const std::array<double, 3> q1 = { c1[0] + t * d1[0], c1[1] + t * d1[1], c1[2] + t * d1[2] };

    r.point = { (q0[0] + q1[0]) * 0.5, (q0[1] + q1[1]) * 0.5, (q0[2] + q1[2]) * 0.5 };
    const double dx = q0[0] - q1[0], dy = q0[1] - q1[1], dz = q0[2] - q1[2];
    r.residual = std::sqrt(dx * dx + dy * dy + dz * dz);
    r.valid = true;
    return r;
}

TriangulateResult triangulateTwoViews(
    const CameraIntrinsics& intr0, const CameraExtrinsics& extr0, double px0, double py0,
    const CameraIntrinsics& intr1, const CameraExtrinsics& extr1, double px1, double py1) {
    if (!intr0.valid || !extr0.valid || !intr1.valid || !extr1.valid)
        return TriangulateResult{};   // valid=false，未标定不产出虚假结果

    double nx0, ny0, nx1, ny1;
    undistortNormalize(intr0, px0, py0, nx0, ny0);
    undistortNormalize(intr1, px1, py1, nx1, ny1);

    const auto c0 = cameraCenterWorld(extr0);
    const auto c1 = cameraCenterWorld(extr1);
    const auto d0 = rayDirectionWorld(extr0, nx0, ny0);
    const auto d1 = rayDirectionWorld(extr1, nx1, ny1);

    return closestPointBetweenRays(c0, d0, c1, d1);
}

// ---------------------------------------------------------------------------
// N 视图三角化的支撑代码
// ---------------------------------------------------------------------------
namespace {

// 4x4 对称矩阵的雅可比特征分解，返回最小特征值对应的特征向量。
// DLT 求 A x = 0 的最小二乘解 = A^T A 最小特征值对应的特征向量，A^T A 是
// 4x4 对称正定（半）矩阵，用雅可比旋转求解稳定且实现短小，不必引入 Eigen。
void smallestEigenvector4(const double M_in[4][4], double out[4]) {
    double A[4][4];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) A[i][j] = M_in[i][j];

    double V[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    for (int sweep = 0; sweep < 50; ++sweep) {
        // 找最大的非对角元
        int p = 0, q = 1; double mx = 0;
        for (int i = 0; i < 4; ++i)
            for (int j = i + 1; j < 4; ++j)
                if (std::abs(A[i][j]) > mx) { mx = std::abs(A[i][j]); p = i; q = j; }
        if (mx < 1e-18) break;   // 已充分对角化

        const double app = A[p][p], aqq = A[q][q], apq = A[p][q];
        const double phi = 0.5 * std::atan2(2 * apq, aqq - app);
        const double c = std::cos(phi), s = std::sin(phi);

        for (int i = 0; i < 4; ++i) {
            const double aip = A[i][p], aiq = A[i][q];
            A[i][p] = c * aip - s * aiq;
            A[i][q] = s * aip + c * aiq;
        }
        for (int i = 0; i < 4; ++i) {
            const double api = A[p][i], aqi = A[q][i];
            A[p][i] = c * api - s * aqi;
            A[q][i] = s * api + c * aqi;
        }
        for (int i = 0; i < 4; ++i) {
            const double vip = V[i][p], viq = V[i][q];
            V[i][p] = c * vip - s * viq;
            V[i][q] = s * vip + c * viq;
        }
    }

    int minIdx = 0; double minVal = A[0][0];
    for (int i = 1; i < 4; ++i) if (A[i][i] < minVal) { minVal = A[i][i]; minIdx = i; }
    for (int i = 0; i < 4; ++i) out[i] = V[i][minIdx];
}

} // namespace

namespace {
// 给定一组视角（rays + 各自的 DLT 两行约束已经预处理成 P 矩阵形式），
// 解出 3D 点，并回填每个视角到该点的垂距。抽出来是为了让"全部视角解一次
// → 剔除坏视角 → 剩余视角再解一次"能复用同一段求解逻辑，不重复写 DLT。
struct PreparedView {
    std::array<double, 3> c, d;     // 相机中心、视线单位方向（世界系）
    double nx, ny;                  // 去畸变归一化坐标
    const CameraExtrinsics* extr;   // 复用其 R,t 构造 DLT 行
};

// 用 subset 里指定下标的视角求解；perpOut（可选）回填 subset 中每个视角
// 到解出点的垂距（顺序与 subset 一致）。
bool solveSubset(const std::vector<PreparedView>& views, const std::vector<int>& subset,
                 std::array<double, 3>& outPoint, std::vector<double>* perpOut) {
    if (int(subset.size()) < 2) return false;
    double ATA[4][4] = {{0}};
    for (int idx : subset) {
        const PreparedView& v = views[idx];
        const auto& R = v.extr->R; const auto& t = v.extr->t;
        const double P0[4] = { R[0], R[1], R[2], t[0] };
        const double P1[4] = { R[3], R[4], R[5], t[1] };
        const double P2[4] = { R[6], R[7], R[8], t[2] };
        double row0[4], row1[4];
        for (int k = 0; k < 4; ++k) {
            row0[k] = v.nx * P2[k] - P0[k];
            row1[k] = v.ny * P2[k] - P1[k];
        }
        for (int a = 0; a < 4; ++a)
            for (int b = 0; b < 4; ++b)
                ATA[a][b] += row0[a]*row0[b] + row1[a]*row1[b];
    }
    double X[4];
    smallestEigenvector4(ATA, X);
    if (std::abs(X[3]) < 1e-12) return false;
    outPoint = { X[0]/X[3], X[1]/X[3], X[2]/X[3] };
    if (perpOut) {
        perpOut->clear();
        for (int idx : subset) {
            const PreparedView& v = views[idx];
            const std::array<double,3> w = { outPoint[0]-v.c[0], outPoint[1]-v.c[1], outPoint[2]-v.c[2] };
            const double proj = w[0]*v.d[0] + w[1]*v.d[1] + w[2]*v.d[2];
            const std::array<double,3> perp = { w[0]-proj*v.d[0], w[1]-proj*v.d[1], w[2]-proj*v.d[2] };
            perpOut->push_back(std::sqrt(perp[0]*perp[0]+perp[1]*perp[1]+perp[2]*perp[2]));
        }
    }
    return true;
}

// 把原始观测预处理成 PreparedView（去畸变 + 视线），过滤掉未标定的。
// idxMap（可选）回填 prepared[i] 对应原始 obs 的下标，供 droppedMask 回写。
std::vector<PreparedView> prepareViews(const ViewObservation* obs, int count,
                                       std::vector<int>* idxMap) {
    std::vector<PreparedView> views;
    if (idxMap) idxMap->clear();
    for (int i = 0; i < count; ++i) {
        const ViewObservation& o = obs[i];
        if (!o.intr || !o.extr || !o.intr->valid || !o.extr->valid) continue;
        PreparedView v;
        undistortNormalize(*o.intr, o.px, o.py, v.nx, v.ny);
        v.c = cameraCenterWorld(*o.extr);
        v.d = rayDirectionWorld(*o.extr, v.nx, v.ny);
        v.extr = o.extr;
        views.push_back(v);
        if (idxMap) idxMap->push_back(i);
    }
    return views;
}

// 加权版：每个视角的两行DLT约束乘以 sqrt(weight) 再累加进 ATA，等效于
// 该视角对最终解的贡献按 weight 加权——weight=1时退化成跟 solveSubset()
// 完全一样的无权最小二乘，weight=0时这个视角形同没参与，权重在两者之间
// 时贡献平滑地介于两者之间，不是"要么全信要么不信"的二选一。
bool solveSubsetWeighted(const std::vector<PreparedView>& views, const std::vector<int>& subset,
                         const std::vector<double>& weights,
                         std::array<double, 3>& outPoint, std::vector<double>* perpOut) {
    if (int(subset.size()) < 2) return false;
    double ATA[4][4] = {{0}};
    for (size_t si = 0; si < subset.size(); ++si) {
        const PreparedView& v = views[size_t(subset[si])];
        const double w = std::sqrt(std::max(0.0, weights[si]));
        const auto& R = v.extr->R; const auto& t = v.extr->t;
        const double P0[4] = { R[0], R[1], R[2], t[0] };
        const double P1[4] = { R[3], R[4], R[5], t[1] };
        const double P2[4] = { R[6], R[7], R[8], t[2] };
        double row0[4], row1[4];
        for (int k = 0; k < 4; ++k) {
            row0[k] = w * (v.nx * P2[k] - P0[k]);
            row1[k] = w * (v.ny * P2[k] - P1[k]);
        }
        for (int a = 0; a < 4; ++a)
            for (int b = 0; b < 4; ++b)
                ATA[a][b] += row0[a]*row0[b] + row1[a]*row1[b];
    }
    double X[4];
    smallestEigenvector4(ATA, X);
    if (std::abs(X[3]) < 1e-12) return false;
    outPoint = { X[0]/X[3], X[1]/X[3], X[2]/X[3] };
    if (perpOut) {
        perpOut->clear();
        for (int idx : subset) {
            const PreparedView& v = views[size_t(idx)];
            const std::array<double,3> pw = { outPoint[0]-v.c[0], outPoint[1]-v.c[1], outPoint[2]-v.c[2] };
            const double proj = pw[0]*v.d[0] + pw[1]*v.d[1] + pw[2]*v.d[2];
            const std::array<double,3> perp = { pw[0]-proj*v.d[0], pw[1]-proj*v.d[1], pw[2]-proj*v.d[2] };
            perpOut->push_back(std::sqrt(perp[0]*perp[0]+perp[1]*perp[1]+perp[2]*perp[2]));
        }
    }
    return true;
}

} // namespace

TriangulateResult triangulateMultiViewRobustSoft(const ViewObservation* obs, int count,
                                                  double madScale, double* weightsOut) {
    std::vector<int> idxMap;
    std::vector<PreparedView> views = prepareViews(obs, count, &idxMap);
    if (weightsOut) for (int i = 0; i < count; ++i) weightsOut[i] = 0.0;

    TriangulateResult r;
    if (int(views.size()) < 2) { r.valid = false; return r; }

    std::vector<int> subset(views.size());
    for (size_t i = 0; i < views.size(); ++i) subset[i] = int(i);
    std::vector<double> weights(views.size(), 1.0);

    constexpr double kAbsPerpRejectMm = 15.0;   // 跟 triangulateMultiViewRobust 同一个绝对判据
    constexpr int kIrlsIters = 4;                // 实测4轮足够收敛，垂距量级下几乎不再变化

    std::array<double,3> pt{};
    std::vector<double> perp;
    for (int iter = 0; iter < kIrlsIters; ++iter) {
        if (!solveSubsetWeighted(views, subset, weights, pt, &perp)) {
            r.valid = false; return r;
        }
        if (iter == kIrlsIters - 1) break;   // 最后一轮解完就退出，不用再算一次没人用的新权重

        // 统计判据：跟硬剔除版本同一套(median + madScale*1.4826*MAD)。
        std::vector<double> sorted = perp;
        std::sort(sorted.begin(), sorted.end());
        const double median = sorted[sorted.size()/2];
        std::vector<double> absdev;
        absdev.reserve(perp.size());
        for (double p : perp) absdev.push_back(std::abs(p - median));
        std::sort(absdev.begin(), absdev.end());
        const double mad = absdev[absdev.size()/2];
        const double statThresh = (mad >= 1e-9) ? (median + madScale * (1.4826 * mad))
                                                 : kAbsPerpRejectMm;
        // 两条判据(统计/绝对)取更紧的那个当Tukey biweight的截止点c——
        // 跟硬剔除版本"任一触发即剔除"是同一个精神(取更保守的那个)，只是
        // 硬剔除版本剔的是"超过阈值"的视角，这里降的是"接近/超过阈值"
        // 视角的权重，越过c权重平滑降到恰好0(Tukey biweight的性质)。
        const double c = std::max(1e-6, std::min(statThresh, kAbsPerpRejectMm));

        for (size_t i = 0; i < perp.size(); ++i) {
            const double ratio = perp[i] / c;
            weights[i] = (ratio < 1.0) ? std::pow(1.0 - ratio*ratio, 2.0) : 0.0;
        }
    }

    r.point = pt;
    // residual 跟其它函数保持同一语义(加权平均垂距，用最终权重加权平均，
    // 而不是简单算术平均——低权重的离群视角不应该把这个数字拉高，它本来
    // 就已经在解算里被压低了影响力)。
    double wSum = 0.0, wPerpSum = 0.0;
    for (size_t i = 0; i < perp.size(); ++i) { wSum += weights[i]; wPerpSum += weights[i]*perp[i]; }
    r.residual = (wSum > 1e-9) ? (wPerpSum / wSum) : (perp.empty() ? -1.0 : perp[0]);
    r.valid = true;

    if (weightsOut)
        for (size_t i = 0; i < subset.size(); ++i) weightsOut[size_t(idxMap[size_t(subset[i])])] = weights[i];

    return r;
}

TriangulateResult triangulateMultiView(const ViewObservation* obs, int count) {
    std::vector<PreparedView> views = prepareViews(obs, count, nullptr);
    TriangulateResult r;
    if (int(views.size()) < 2) { r.valid = false; return r; }

    std::vector<int> all(views.size());
    for (size_t i = 0; i < views.size(); ++i) all[i] = int(i);
    std::vector<double> perp;
    if (!solveSubset(views, all, r.point, &perp)) { r.valid = false; return r; }

    double sum = 0;
    for (double p : perp) sum += p;
    r.residual = sum / perp.size();
    r.valid = true;
    return r;
}

TriangulateResult triangulateMultiViewRobust(const ViewObservation* obs, int count,
                                              int minKeep, double madScale, bool* droppedMask) {
    std::vector<int> idxMap;
    std::vector<PreparedView> views = prepareViews(obs, count, &idxMap);
    if (droppedMask) for (int i = 0; i < count; ++i) droppedMask[i] = false;

    TriangulateResult r;
    if (int(views.size()) < 2) { r.valid = false; return r; }

    // 当前保留的视角集合，从全部开始。
    std::vector<int> keep(views.size());
    for (size_t i = 0; i < views.size(); ++i) keep[i] = int(i);

    std::array<double,3> pt;
    std::vector<double> perp;
    if (!solveSubset(views, keep, pt, &perp)) { r.valid = false; return r; }

    // 迭代式剔除：每一轮用"当前保留集合"重解、重算垂距、重判离群，只剔掉
    // 最坏的那一个，再用剩下的重解。不能拿"被坏视角污染的全量解"一次性
    // 判所有视角——单次判据在小样本下会失效：坏视角把解、中位数、MAD
    // 一起带偏，反而稀释了自己的离群性，落在阈值内剔不掉（实测 4 台相机、
    // 1 台偏 25px 正是如此：坏点垂距 28mm，但它把 median 抬到 15、MAD 抬到
    // 10，阈值算出来 59mm，剔不掉自己）。
    //
    // 判据用两条取并集：
    //   (a) 统计判据：median + madScale*(1.4826*MAD)，适应不同场景的噪声水平；
    //   (b) 绝对物理判据：kAbsPerpRejectMm。多相机动捕里，标定正常、时间
    //       对齐后，好视角到解的垂距物理上就该是几毫米量级——超过这个绝对
    //       值几乎不可能是"好视角只是噪声大一点"，而是真出问题了（检测跳点/
    //       时间错位/被误检的反光）。绝对判据专门兜住 (a) 在小样本下被坏点
    //       稀释、失灵的情况。
    // 两条任一触发即认为最坏视角离群、剔除。
    constexpr double kAbsPerpRejectMm = 15.0;
    while (int(keep.size()) > minKeep && int(keep.size()) > 2) {
        std::vector<double> sorted = perp;
        std::sort(sorted.begin(), sorted.end());
        const double median = sorted[sorted.size()/2];
        std::vector<double> absdev;
        absdev.reserve(perp.size());
        for (double p : perp) absdev.push_back(std::abs(p - median));
        std::sort(absdev.begin(), absdev.end());
        const double mad = absdev[absdev.size()/2];

        int worstLocal = 0;
        for (size_t i = 1; i < perp.size(); ++i)
            if (perp[i] > perp[worstLocal]) worstLocal = int(i);
        const double worst = perp[worstLocal];

        const double statThresh = median + madScale * (1.4826 * mad);
        const bool statOutlier = (mad >= 1e-9) && (worst > statThresh);
        const bool absOutlier  = (worst > kAbsPerpRejectMm);
        if (!statOutlier && !absOutlier) break;   // 最坏的也不算离群，停止

        keep.erase(keep.begin() + worstLocal);
        if (!solveSubset(views, keep, pt, &perp)) break;   // 退化则用上一轮结果
    }

    r.point = pt;
    double sum = 0; for (double p : perp) sum += p;
    r.residual = perp.empty() ? -1.0 : sum / perp.size();
    r.valid = true;

    // 回填哪些原始视角被剔除了。
    if (droppedMask) {
        std::vector<bool> kept(views.size(), false);
        for (int idx : keep) kept[idx] = true;
        for (size_t i = 0; i < views.size(); ++i)
            if (!kept[i]) droppedMask[idxMap[i]] = true;
    }
    return r;
}

} // namespace mocap
