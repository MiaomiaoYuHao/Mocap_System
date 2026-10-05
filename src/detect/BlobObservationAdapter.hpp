#pragma once
// ---------------------------------------------------------------------------
// 把 CentroidDetector::Blob 的轮廓点接进 detect/DetectionOutput.hpp 的
// 3a~3d 流水线，再把结果转换成 estimate 层要的 RawDetection(归一化坐标)。
// 这是之前提到的 TODO 的落地文件——观测协方差从固定近似换成真的按弧长/
// 遮挡自适应。
//
// 设计选择：3a~3d 的圆拟合在**像素空间**做，不是归一化空间。原因：
//   1. ArcCircleFit.hpp 里弧长->sigma 的经验表是在像素尺度上标定的
//      (0.18px~5px那张表)，直接搬到归一化坐标(通常是 1e-3 量级的数)
//      单位对不上。
//   2. "已知半径"这个先验在像素空间最自然获取(轮廓本身就是像素坐标)，
//      不需要先转换坐标系再算半径。
// 所以流程是：像素空间做 3a~3d 拟合 -> 只在最后把 (mu, Sigma) 这个结果
// 转换到归一化坐标(mu 用 undistortNormalize，Sigma 用该点处的局部雅可比
// 传播，因为畸变模型非线性，不能直接除以焦距了事)。
//
// 一个 Blob 经过 detectBalls() 后可能产出 0/1/2 个观测(单球/花生双球/
// 全部弃用)，所以这里的转换函数输出的是 vector<RawDetection>，调用方
// (HandTrackingWorker) 要相应地把每个 blob 展开成 0~2 条 RawDetection，
// 不能假设"一个 blob 对应一条观测"。
// ---------------------------------------------------------------------------
#include "detect/CentroidDetector.hpp"
#include "detect/DetectionOutput.hpp"
#include "reconstruct/Triangulation.hpp"   // undistortNormalize
#include "calib/Calibration.hpp"           // CameraIntrinsics
#include "estimate/MarkerAssociator.hpp"   // RawDetection, Cov2
#include <vector>
#include <cmath>
#include <algorithm>   // std::min/max（外接框长宽比预筛用）

namespace mocap {

// 【本轮新增】三种2D定位模式可选——起因是实测发现纯圆拟合在有些场景下
// 反而不如质心稳(圆拟合依赖"已知半径"这个先验，先验哪怕经过深度反推、
// 仍然可能有残余误差；而质心法在球清晰可见、信噪比好的时候本身就已经
// 接近最优、几乎不需要额外先验)。圆拟合真正的优势场景是遮挡——半遮挡时
// 质心会系统性偏向"可见的那一侧"，不是球的真实几何中心，这时候已知半径
// 约束才有不可替代的价值。三选一或融合，具体用哪个取决于你的实际遮挡
// 频率/画面质量，两边都测过之后再定。
enum class BlobLocalizationMode {
    CentroidOnly,   // 只用flood fill的加权质心，不跑3a~3d，不需要轮廓点、不依赖半径先验
    CircleFitOnly,  // 原有行为：只用3a~3d的约束圆拟合结果
    Fused,          // 两者都算，圆拟合可用时做逆方差高斯融合，圆拟合被判定Discard时退回质心
                    // (不是"要么用圆拟合要么完全没有观测"这种断崖式的，退化更平滑)
};

// 半径先验来源的说明(调用方需要知道怎么填 knownRadiusPx，不是这个文件
// 决定的)：物理反光球半径(mm，marker 出厂尺寸) × 相机焦距(px) / 到该
// marker 的估计深度(mm)。深度理想情况下来自上一帧 HandStateIEKF 估出的
// 手腕位置(再加上该相机的标定外参算出这台相机到手的距离)；冷启动阶段
// 还没有这个反馈，退回用捕捉空间的一个标称工作距离常量。这个换算发生在
// HandTrackingWorker 里，不在这个文件里做，这个文件只管"给定一个已知
// 像素半径，怎么把轮廓点变成观测"这一步。
struct BlobObservationConfig {
    double knownRadiusPx = 6.0;     // 这一帧、这台相机、这个先验半径的估计值(像素)
    BlobLocalizationMode mode = BlobLocalizationMode::CircleFitOnly;   // 默认保持原有行为，不默认改变已验证过的路径
    double centroidSigmaPx = 0.3;   // 质心估计本身的噪声假设(像素标准差)——CentroidOnly/Fused两种模式都用得上，
                                    // 球清晰可见时质心通常能到零点几像素级精度，跟ArcCircleFit.hpp里High档(>=90°)
                                    // 的0.18~0.19px是同一个量级，给0.3是留了点余量的保守值。
};

namespace blobobs_detail {

// 在像素点 (px,py) 附近用中心差分数值求 undistortNormalize 的 2x2 雅可比，
// 传播协方差 Sigma_norm = J * Sigma_px * J^T。畸变模型是非线性的
// (Triangulation.cpp 里定点迭代求逆畸变)，不能简单地除以焦距了事——
// 焦距只是雅可比对角线上的主导项，畸变越大、越靠画面边缘，非对角项和
// 非对角耦合越不能忽略。
inline void propagateCovariance(const CameraIntrinsics& intr, double px, double py,
                                const Cov2& sigmaPx, double& nx0, double& ny0, Cov2& sigmaNorm) {
    undistortNormalize(intr, px, py, nx0, ny0);

    const double h = 0.5;   // 半像素步长做中心差分，足够小、不会被畸变的高阶项污染
    double nxPx, nyPx, nxMx, nyMx, nxPy, nyPy, nxMy, nyMy;
    undistortNormalize(intr, px+h, py, nxPx, nyPx);
    undistortNormalize(intr, px-h, py, nxMx, nyMx);
    undistortNormalize(intr, px, py+h, nxPy, nyPy);
    undistortNormalize(intr, px, py-h, nxMy, nyMy);

    // J = [[d(nx)/d(px), d(nx)/d(py)], [d(ny)/d(px), d(ny)/d(py)]]
    const double J00 = (nxPx-nxMx)/(2*h), J01 = (nxPy-nxMy)/(2*h);
    const double J10 = (nyPx-nyMx)/(2*h), J11 = (nyPy-nyMy)/(2*h);

    // Sigma_norm = J * Sigma_px * J^T (2x2)
    const double a = J00*sigmaPx.xx + J01*sigmaPx.xy;
    const double b = J00*sigmaPx.xy + J01*sigmaPx.yy;
    const double c = J10*sigmaPx.xx + J11*sigmaPx.xy;
    const double d = J10*sigmaPx.xy + J11*sigmaPx.yy;
    sigmaNorm.xx = a*J00 + b*J01;
    sigmaNorm.xy = a*J10 + b*J11;
    sigmaNorm.yy = c*J10 + d*J11;
}

// 2x2矩阵求逆，行列式接近0时返回false(退化协方差，调用方应有兜底)。
inline bool invert2x2(const Cov2& s, double inv[2][2]) {
    const double det = s.xx*s.yy - s.xy*s.xy;
    if (std::abs(det) < 1e-12) return false;
    const double id = 1.0/det;
    inv[0][0]=s.yy*id; inv[0][1]=-s.xy*id; inv[1][0]=-s.xy*id; inv[1][1]=s.xx*id;
    return true;
}

// 两个独立2D高斯估计的逆方差融合(信息形式相加)——数学上就是一步不用
// 显式卡尔曼增益公式的卡尔曼更新：两个独立观测各自的置信度(协方差的逆，
// 也叫信息矩阵)相加，均值按各自信息矩阵加权。协方差越小(越自信)的那个
// 观测，在融合结果里占的权重天然越大，不需要另外设计"该信谁多一点"的
// 规则。任何一个协方差退化(不可逆)时，直接采用另一个，不强行融合一个
// 没意义的结果。
inline void fuseGaussians2D(const Point2& mu1, const Cov2& s1, const Point2& mu2, const Cov2& s2,
                            Point2& muOut, Cov2& sOut) {
    double inv1[2][2], inv2[2][2];
    const bool ok1 = invert2x2(s1, inv1);
    const bool ok2 = invert2x2(s2, inv2);
    if (!ok1 && !ok2) { muOut = mu1; sOut = s1; return; }   // 两个都退化，随便回退一个，不应该发生
    if (!ok1) { muOut = mu2; sOut = s2; return; }
    if (!ok2) { muOut = mu1; sOut = s1; return; }

    const double sumInv00 = inv1[0][0]+inv2[0][0], sumInv01 = inv1[0][1]+inv2[0][1];
    const double sumInv10 = inv1[1][0]+inv2[1][0], sumInv11 = inv1[1][1]+inv2[1][1];
    const double det = sumInv00*sumInv11 - sumInv01*sumInv10;
    if (std::abs(det) < 1e-12) { muOut = mu1; sOut = s1; return; }   // 退化兜底
    const double id = 1.0/det;

    sOut.xx = sumInv11*id; sOut.xy = -sumInv01*id; sOut.yy = sumInv00*id;

    const double iv0 = inv1[0][0]*mu1.x + inv1[0][1]*mu1.y + inv2[0][0]*mu2.x + inv2[0][1]*mu2.y;
    const double iv1 = inv1[1][0]*mu1.x + inv1[1][1]*mu1.y + inv2[1][0]*mu2.x + inv2[1][1]*mu2.y;
    muOut.x = sOut.xx*iv0 + sOut.xy*iv1;
    muOut.y = sOut.xy*iv0 + sOut.yy*iv1;
}

} // namespace blobobs_detail

// 主转换函数：一个 Blob(带轮廓，collectContours=true 时 CentroidDetector
// 才会填) -> 0~2 条归一化坐标的 RawDetection。CentroidOnly 模式不需要
// 轮廓点也能工作(直接用 blob.cx/cy)，其余两种模式仍然需要轮廓。
inline std::vector<RawDetection> blobToObservations(
        const Blob& blob, const CameraIntrinsics& intr, const BlobObservationConfig& cfg) {
    std::vector<RawDetection> out;

    auto pushCentroidOnly = [&]() {
        RawDetection det;
        const Cov2 sigmaPx{ cfg.centroidSigmaPx*cfg.centroidSigmaPx, 0.0, cfg.centroidSigmaPx*cfg.centroidSigmaPx };
        blobobs_detail::propagateCovariance(intr, double(blob.cx), double(blob.cy), sigmaPx, det.nx, det.ny, det.sigma);
        out.push_back(det);
    };

    if (cfg.mode == BlobLocalizationMode::CentroidOnly) {
        pushCentroidOnly();
        return out;
    }

    if (blob.contour.size() < 2) {
        // 没轮廓数据(没开collectContours)或点太少——CircleFitOnly模式下
        // 只能诚实返回空(原有行为不变)；Fused模式没有理由跟着一起交白卷，
        // 质心不需要轮廓，退回质心结果，比"什么都不给"更有用。
        if (cfg.mode == BlobLocalizationMode::Fused) pushCentroidOnly();
        return out;
    }

    std::vector<Point2> pts;
    pts.reserve(blob.contour.size());
    for (const auto& p : blob.contour) pts.push_back({double(p[0]), double(p[1])});

    // 【性能】外接框长宽比预筛：单球近正方形，粘连花生形明显拉长。只有可疑
    // (长宽比>1.3 或外接框缺失)才让 detectBalls 试双圆拆分，其余走单球快路径。
    // detectBalls 单次~107us，密集摆位(多球×多相机)下无脑全跑会卡爆整条手部
    // 追踪管线。实测整帧检测加速~5x、对真正粘连零漏检(间距≥1r)。
    bool mayBeMerged = true;
    if (blob.bboxW > 0 && blob.bboxH > 0) {
        const int lo = std::min(blob.bboxW, blob.bboxH);
        const int hi = std::max(blob.bboxW, blob.bboxH);
        mayBeMerged = (lo <= 0) || (double(hi)/double(lo) > 1.3);
    }
    const auto detection = detectBalls(pts, cfg.knownRadiusPx, mayBeMerged);
    if (!detection.valid) {
        if (cfg.mode == BlobLocalizationMode::Fused) pushCentroidOnly();
        return out;
    }

    if (detection.model == BallModel::Double) {
        // 花生双球场景：整个blob的质心是两颗球糊在一起的质心，跟任何一颗
        // 单独的真实中心都对不上，融合没有意义——两种模式都直接用圆拟合
        // 分离出来的两个观测(Fused模式在"两球分离"这件事上，本来就只有
        // 圆拟合能做，质心法根本给不出"两个"位置)。
        for (const auto& obs : detection.observations) {
            if (!obs.usable) continue;
            RawDetection det;
            blobobs_detail::propagateCovariance(intr, obs.mu.x, obs.mu.y, obs.sigma, det.nx, det.ny, det.sigma);
            out.push_back(det);
        }
        return out;
    }

    // Single 模型：这里才是 CircleFitOnly vs Fused 真正分叉的地方。
    const auto& obs = detection.observations.empty() ? Observation2D{} : detection.observations[0];
    const bool haveUsableArc = !detection.observations.empty() && obs.usable;

    if (cfg.mode == BlobLocalizationMode::CircleFitOnly) {
        if (!haveUsableArc) return out;   // 原有行为：Discard就是没有观测，诚实报告
        RawDetection det;
        blobobs_detail::propagateCovariance(intr, obs.mu.x, obs.mu.y, obs.sigma, det.nx, det.ny, det.sigma);
        out.push_back(det);
        return out;
    }

    // Fused：圆拟合可用就融合，不可用(弧太短被Discard/取不到观测)就退回
    // 质心——不是断崖式的"要么圆拟合要么什么都没有"，弧长从充分到不足的
    // 过程中，观测质量是平滑退化到"纯质心"，而不是突然消失。
    const Point2 centroidMu{ double(blob.cx), double(blob.cy) };
    const Cov2 centroidSigma{ cfg.centroidSigmaPx*cfg.centroidSigmaPx, 0.0, cfg.centroidSigmaPx*cfg.centroidSigmaPx };

    if (!haveUsableArc) {
        pushCentroidOnly();
        return out;
    }

    Point2 fusedMu; Cov2 fusedSigma;
    blobobs_detail::fuseGaussians2D(centroidMu, centroidSigma, obs.mu, obs.sigma, fusedMu, fusedSigma);

    RawDetection det;
    blobobs_detail::propagateCovariance(intr, fusedMu.x, fusedMu.y, fusedSigma, det.nx, det.ny, det.sigma);
    out.push_back(det);
    return out;
}

} // namespace mocap
