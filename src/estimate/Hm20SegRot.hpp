#pragma once
// ===========================================================================
// Hm20SegRot.hpp —— 把 hm20_v7 的 seg_rot6d 头接进来用
//
// 【这个输出一直存在但从没被消费过】
// Hm20OnnxBackend.hpp 里 haveOut_.seg = hasOutput("seg_rot6d") 早就检测了它，
// 启动日志也打印"seg_rot6d 有"，但 kOnnxOutputNames() 从来没请求它，
// 所以每帧都白算了一份 (1,16,6)。
//
// 【它是什么】实测(tools/hm20_diag/probe_thumb_axis.py 打在 hm20_v7.onnx 上)：
//   · 把输入点云整体旋转平移，seg_rot6d 完全不变(变化 0.12°)
//   · seg_rot6d[0] 恒为单位阵(与 I 的夹角 0.15°)
//   => 它是【手掌规范系】下的 16 段姿态，跟世界摆放无关。转世界系：
//          R_world(s) = wristR · C · segR[s]
//      C = 你的模板系 -> 网络规范系，是个【常数】，标一次存起来。
//
// 【为什么值得接】现在拇指远节的姿态是 (p7 - p6) 两个球做差得来的，而 p7 经常
// 是网络补出来的预测点 —— 拿一个预测点去做差分，误差被距离一除就放大了。
// seg_rot6d 不需要那两个球，它直接给姿态。这是拇指遮挡时唯一还站得住的来源。
//
// 【但它救不了"外翻"本身】seg_rot6d 跟 pos 头训练自同一个 rig，而那个 rig 的
// 拇指缺了常数旋前(实测铰链轴只偏 17.4°，真人要 80~90°)。所以拇指三段还需要
// 一个常数修正 K_s。好消息是 K_s 可以【从你自己的手上量】：用拇指三颗球全部
// 可见的帧(文档说可见率约 45%，样本足够)，那些帧的几何解可信，
//      K_s = (wristR·C·segR[s])ᵀ · R_geo_world(s)
// 对几百帧做旋转平均即可。手性自动正确，因为它是从你的实测数据里解出来的。
//
// 【零依赖】本文件不 include 任何工程头。跟 DorsumRigidSolver.hpp 同样的原因：
//   Hm20OnnxBackend.hpp 和 HandSkeletonAssociator.hpp 都要用它，而它俩之间
//   本来就有 include 关系，任何一边反过来被它 include 都会成环。
//   Mat3/Quat 的 typedef 跟关联器逐位一致，两边直接互传，不需要转换。
// 依赖 SkeletonFrameResult 的那部分(SegCanonAligner)放在
// HandSkeletonAssociator.hpp 里，因为那些类型定义在那儿。
// ===========================================================================

#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

namespace mocap {
namespace hm20 {

// 跟 HandSkeletonAssociator.hpp 里的 typedef 逐位一致。两个头文件都出现时
// 是同一个类型别名，编译器视作同一类型，不冲突。
#ifndef MOCAP_HM20_MAT3_ALIAS
#define MOCAP_HM20_MAT3_ALIAS
using Vec3 = std::array<double, 3>;   // (x,y,z)
using Mat3 = std::array<double, 9>;   // 行主序
using Quat = std::array<double, 4>;   // (w,x,y,z)
#endif

// ---------------------------------------------------------------------------
// 6D -> 旋转矩阵（Gram-Schmidt，跟训练侧 rotation_6d_to_matrix 一致）
// 返回行主序 Mat3，列是基向量。
// ---------------------------------------------------------------------------
inline Mat3 rot6dToMat(const float* v) {
    double a1[3] = {double(v[0]), double(v[1]), double(v[2])};
    double a2[3] = {double(v[3]), double(v[4]), double(v[5])};
    double n1 = std::sqrt(a1[0]*a1[0] + a1[1]*a1[1] + a1[2]*a1[2]);
    if (n1 < 1e-12) return Mat3{{1,0,0, 0,1,0, 0,0,1}};
    double b1[3] = {a1[0]/n1, a1[1]/n1, a1[2]/n1};
    const double p = b1[0]*a2[0] + b1[1]*a2[1] + b1[2]*a2[2];
    double b2[3] = {a2[0] - p*b1[0], a2[1] - p*b1[1], a2[2] - p*b1[2]};
    const double n2 = std::sqrt(b2[0]*b2[0] + b2[1]*b2[1] + b2[2]*b2[2]);
    if (n2 < 1e-12) return Mat3{{1,0,0, 0,1,0, 0,0,1}};
    for (int i = 0; i < 3; ++i) b2[i] /= n2;
    const double b3[3] = {b1[1]*b2[2] - b1[2]*b2[1],
                          b1[2]*b2[0] - b1[0]*b2[2],
                          b1[0]*b2[1] - b1[1]*b2[0]};
    return Mat3{{b1[0], b2[0], b3[0],
                 b1[1], b2[1], b3[1],
                 b1[2], b2[2], b3[2]}};
}

// ---------------------------------------------------------------------------
// 旋转平均（chordal L2）：四元数半球对齐后求和，再重正交化。
// 用在两处：估 C(模板系->规范系) 和估 K_s(拇指常数修正)。都是常数，
// 攒几百帧就收敛。
// ---------------------------------------------------------------------------
class RotationAverager {
public:
    void reset() { acc_ = {0,0,0,0}; n_ = 0; }
    int  count() const { return n_; }

    void add(const Mat3& R) {
        const Quat q = matToQuatLocal(R);
        // 四元数有 q 和 -q 两个表示，不对齐半球会互相抵消
        double d = 0.0;
        for (int i = 0; i < 4; ++i) d += acc_[size_t(i)] * q[size_t(i)];
        const double s = (n_ == 0 || d >= 0.0) ? 1.0 : -1.0;
        for (int i = 0; i < 4; ++i) acc_[size_t(i)] += s * q[size_t(i)];
        ++n_;
    }

    bool mean(Mat3& out) const {
        if (n_ == 0) return false;
        double nn = 0.0;
        for (int i = 0; i < 4; ++i) nn += acc_[size_t(i)] * acc_[size_t(i)];
        nn = std::sqrt(nn);
        if (nn < 1e-12) return false;
        const double w = acc_[0]/nn, x = acc_[1]/nn, y = acc_[2]/nn, z = acc_[3]/nn;
        out = Mat3{{1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w),
                    2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w),
                    2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)}};
        return true;
    }

    // 离散度：样本与均值的平均夹角(度)。> 15° 说明这批样本里混了错帧，
    // 或者 C/K_s 根本不是常数（那就说明前面某个假设错了，别硬用）。
    double spreadDeg(const std::vector<Mat3>& samples) const {
        Mat3 m{};
        if (!mean(m) || samples.empty()) return -1.0;
        double s = 0.0;
        for (const Mat3& R : samples) s += angleDeg(m, R);
        return s / double(samples.size());
    }

    static double angleDeg(const Mat3& A, const Mat3& B) {
        double tr = 0.0;                       // trace(AᵀB) = 逐元素点积
        for (int i = 0; i < 9; ++i) tr += A[size_t(i)] * B[size_t(i)];
        return std::acos(std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5))) * 57.29577951308232;
    }

private:
    static Quat matToQuatLocal(const Mat3& R) {
        const double t = R[0] + R[4] + R[8];
        Quat q{};
        if (t > 0.0) {
            const double s = std::sqrt(t + 1.0) * 2.0;
            q = {0.25*s, (R[7]-R[5])/s, (R[2]-R[6])/s, (R[3]-R[1])/s};
        } else if (R[0] > R[4] && R[0] > R[8]) {
            const double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
            q = {(R[7]-R[5])/s, 0.25*s, (R[1]+R[3])/s, (R[2]+R[6])/s};
        } else if (R[4] > R[8]) {
            const double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
            q = {(R[2]-R[6])/s, (R[1]+R[3])/s, 0.25*s, (R[5]+R[7])/s};
        } else {
            const double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
            q = {(R[3]-R[1])/s, (R[2]+R[6])/s, (R[5]+R[7])/s, 0.25*s};
        }
        double n = 0.0;
        for (int i = 0; i < 4; ++i) n += q[size_t(i)]*q[size_t(i)];
        n = std::sqrt(n);
        if (n > 1e-12) for (int i = 0; i < 4; ++i) q[size_t(i)] /= n;
        return q;
    }

    Quat acc_{0,0,0,0};
    int  n_ = 0;
};

inline Mat3 matMul3(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += A[size_t(r*3+k)] * B[size_t(k*3+c)];
            C[size_t(r*3+c)] = s;
        }
    return C;
}
inline Mat3 matT3(const Mat3& A) {
    return Mat3{{A[0],A[3],A[6], A[1],A[4],A[7], A[2],A[5],A[8]}};
}
inline Mat3 quatToMat3(const Quat& q) {
    const double w=q[0], x=q[1], y=q[2], z=q[3];
    return Mat3{{1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w),
                 2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w),
                 2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)}};
}

}  // namespace hm20
}  // namespace mocap
