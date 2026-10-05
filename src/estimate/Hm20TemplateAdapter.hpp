#pragma once
// ---------------------------------------------------------------------------
// Hm20TemplateAdapter.hpp
//
// HandTemplateStore 存的是【手背5点 + 每指 anchor/lengths】，而 hm20 网络的
// tmpl 要的是【中立位 20 个 marker 的坐标】。这两者不是同一种东西，需要一次
// 合成：手背5点直接用，手指15点由 anchor 出发、按 lengths 沿中立位方向排出来。
//
// 【这是近似，要知道它近似在哪】
//   1. 中立位方向取"由腕心指向该指 anchor"的径向，因为 HandTemplateData 里
//      没有存手指的中立位朝向。真实中立位手指略有屈曲和外展，这里当成完全
//      伸直、无外展。
//   2. marker 放在每节骨的中点，【不含】背侧 11~17mm 的贴球偏置——模板是
//      名义几何，偏置属于实际佩戴，不该进模板。
//   3. 合成结果只用于网络的条件向量。由于 packNormalized() 会整体除以
//      handScale，网络看到的是【形状比例】，上面两条近似带来的绝对误差被
//      归一化掉了大半；影响的是比例关系，不是尺寸。
//
// 如果标定向导以后能直接存下中立位 20 点，就用那份替掉 synthesize()，
// 精度会更好。现在 HandTemplateStore 没有那个字段。
// ---------------------------------------------------------------------------
#include "estimate/Hm20OnnxBackend.hpp"
#include "hand/HandTemplateStore.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace mocap {
namespace hm20 {

// HandTemplateData -> Hm20Template
// isRight: HandTemplateData 里没有左右手字段，由调用方给（标定向导里选的那个）。
inline Hm20Template makeHm20Template(const HandTemplateData& src, bool isRight) {
    Hm20Template t;
    t.isRight = isRight;
    // valid 只表示"手背相位已标定"——tmpl_valid 的手背 5 位由它决定。
    // 手指 15 位恒为 0（手指标定原理上做不到），见 packValidMask()。
    t.valid = src.backCalibrated;

    // ---- 手背 5 点：直接用 ----
    for (int i = 0; i < 5; ++i) t.markersMm[size_t(i)] = src.backMarkers[size_t(i)];

    // ---- 指根关节：直接来自标定，解近节骨朝向要用 ----
    for (int f = 0; f < 5; ++f) t.anchorsMm[size_t(f)] = src.fingerParams[size_t(f)].anchor;
    t.anchorsValid = true;

    // ---- 手指 15 点：从 anchor + lengths 合成 ----
    for (int f = 0; f < 5; ++f) {
        const auto& fp = src.fingerParams[size_t(f)];
        const HandVec3& a = fp.anchor;

        // 中立位方向 = 腕心 -> anchor 的径向单位向量
        double n = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
        HandVec3 dir = (n > 1e-6) ? HandVec3{a[0]/n, a[1]/n, a[2]/n}
                                  : HandVec3{1.0, 0.0, 0.0};   // anchor 在原点时的兜底

        // 沿 dir 依次排三节；marker 落在每节中点
        HandVec3 joint = a;
        for (int k = 0; k < 3; ++k) {
            const double L = fp.lengths[size_t(k)];
            const int m = 5 + f * 3 + k;
            t.markersMm[size_t(m)] = {joint[0] + dir[0] * L * 0.5,
                                      joint[1] + dir[1] * L * 0.5,
                                      joint[2] + dir[2] * L * 0.5};
            joint = {joint[0] + dir[0] * L, joint[1] + dir[1] * L, joint[2] + dir[2] * L};
        }
    }
    return t;
}

// 手背 -> 每指第一节的挂载边。原 fingerMountEdges() 随上一代 associator 一起
// 没了，这里按同样语义重建：每指的近节 marker 连到手背 5 点里离它最近的那个。
// 纯渲染用，不参与任何解算。
inline std::vector<std::pair<int, int>> fingerMountEdges(const Hm20Template& t) {
    std::vector<std::pair<int, int>> e;
    e.reserve(5);
    for (int f = 0; f < 5; ++f) {
        const int pp = 5 + f * 3;
        const Vec3& p = t.markersMm[size_t(pp)];
        int best = 0;
        double bestD = 1e300;
        for (int d = 0; d < 5; ++d) {
            const Vec3& q = t.markersMm[size_t(d)];
            const double dx = p[0]-q[0], dy = p[1]-q[1], dz = p[2]-q[2];
            const double dd = dx*dx + dy*dy + dz*dz;
            if (dd < bestD) { bestD = dd; best = d; }
        }
        e.push_back({best, pp});
    }
    return e;
}

} // namespace hm20
} // namespace mocap
