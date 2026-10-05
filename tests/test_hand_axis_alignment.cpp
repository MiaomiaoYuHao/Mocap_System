// HandAxisAlignment.hpp 单元测试——合成数据验证"把任意朝向的局部系对齐到
// +X指手指方向、+Y指拇指侧"这个几何算法。
//
// 覆盖范围：
//   1. 基本正确性：构造已知的手背模板+5指锚点(拇指在其中一个明显偏离
//      共线模式的位置)，套一个随机旋转+平移模拟"MDS重建出来的任意朝向"，
//      对齐后应恢复出正确的+X/+Y方向(用对齐后各锚点的坐标符号/相对关系
//      核对，而不是要求跟某个"标准答案"数值上完全一致——因为对齐算法
//      本身就是在定义这个标准)，且正确识别出哪个是拇指。
//   2. 手指链锚点数量不对：诚实返回invalid。
//   3. 5根锚点近似共线共面(无法区分拇指)：诚实返回invalid，不瞎选。
//   4. 应用对齐(applyAxisAlignment)后，手背模板里"手指方向"分量应为正
//      (在新坐标系里，锚点普遍在+X一侧)。
#include "estimate/HandAxisAlignment.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static std::array<double,9> randRot(std::mt19937& g) {
    std::normal_distribution<double> nd(0,1);
    std::uniform_real_distribution<double> ud(0,2*M_PI);
    double v[3]={nd(g),nd(g),nd(g)};
    double n=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    v[0]/=n;v[1]/=n;v[2]/=n;
    double a=ud(g), c=std::cos(a), s=std::sin(a), C=1-c;
    return {
        c+v[0]*v[0]*C,      v[0]*v[1]*C-v[2]*s, v[0]*v[2]*C+v[1]*s,
        v[1]*v[0]*C+v[2]*s, c+v[1]*v[1]*C,      v[1]*v[2]*C-v[0]*s,
        v[2]*v[0]*C-v[1]*s, v[2]*v[1]*C+v[0]*s, c+v[2]*v[2]*C
    };
}
static Vec3 applyRT(const std::array<double,9>& R, const Vec3& t, const Vec3& p) {
    return { R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0],
             R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1],
             R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2] };
}

int main() {
    std::mt19937 rng(20260725);

    // "真值"世界：手背5点模板 + 5指锚点(下标0=拇指，明显偏离其余4指的
    // 共线排布)，这些坐标本身就是按 HandModel.hpp 的约定给的(+X手指方向、
    // +Y拇指侧)，方便核对对齐结果是否正确恢复了这个约定。
    const std::vector<Vec3> wristTemplateTrue = {
        {15,0,2}, {55,22,5}, {50,-25,8}, {35,30,6}, {60,3,3}
    };
    const std::vector<Vec3> anchorsTrue = {
        {30,40,5},    // 0 = 拇指：明显偏向+Y一侧，偏离其余4指的共线排布
        {85,20,0},    // 1 = 食指
        {88,3,0},     // 2 = 中指
        {85,-14,0},   // 3 = 无名指
        {80,-30,0},   // 4 = 小指
    };

    // ---- 场景 1：套随机旋转+平移模拟"MDS重建出来的任意朝向"，对齐后
    // 应该恢复出正确的手指方向识别、且新坐标系里手指普遍在+X一侧、拇指
    // 在+Y一侧。----
    {
        const auto R = randRot(rng);
        const Vec3 t = {123, -45, 678};

        std::vector<Vec3> wristTemplateArb, anchorsArb;
        for (auto& p : wristTemplateTrue) wristTemplateArb.push_back(applyRT(R, t, p));
        for (auto& p : anchorsTrue) anchorsArb.push_back(applyRT(R, t, p));

        const auto align = computeAxisAlignment(wristTemplateArb, anchorsArb);
        CHECK(align.valid, "random-orientation scene: alignment succeeds");
        if (align.valid) {
            CHECK(align.thumbAnchorIndex == 0, "random-orientation scene: correctly identifies anchor index 0 as the thumb");

            // 对齐后重新表达所有锚点，检查：拇指的+Y分量应明显大于0且
            // 明显大于其余4指；除拇指外，其余4指的+X分量都应为正(手指
            // 方向朝外)。
            std::vector<Vec3> aligned;
            for (auto& p : anchorsArb) aligned.push_back(applyAxisAlignment(align, p));

            bool othersPositiveX = true;
            for (int i=1;i<5;++i) if (aligned[size_t(i)][0] <= 0) othersPositiveX = false;
            CHECK(othersPositiveX, "random-orientation scene: non-thumb anchors have positive +X (distal direction) after alignment");

            double thumbY = aligned[0][1];
            double maxOtherY = -1e18;
            for (int i=1;i<5;++i) maxOtherY = std::max(maxOtherY, aligned[size_t(i)][1]);
            char msg[160];
            std::snprintf(msg, sizeof(msg), "random-orientation scene: thumb's +Y clearly exceeds other fingers' +Y (thumbY=%.2f maxOtherY=%.2f)", thumbY, maxOtherY);
            CHECK(thumbY > maxOtherY + 5.0, msg);
        }
    }

    // ---- 场景 2：手指链锚点数量不对(比如分组只成功分出4根)——诚实
    // 返回invalid。----
    {
        std::vector<Vec3> wristTemplateArb = wristTemplateTrue;
        std::vector<Vec3> anchorsArb = { anchorsTrue[0], anchorsTrue[1], anchorsTrue[2], anchorsTrue[3] };   // 只有4个
        const auto align = computeAxisAlignment(wristTemplateArb, anchorsArb);
        CHECK(!align.valid, "wrong anchor count (4 instead of 5): honestly reports invalid");
    }

    // ---- 场景 3：5根锚点近似共线(无法区分谁是拇指)——诚实返回invalid，
    // 不瞎选一个当拇指。----
    {
        std::vector<Vec3> wristTemplateArb = wristTemplateTrue;
        // 5个点几乎排成一条直线，没有谁明显偏离。
        std::vector<Vec3> anchorsArb = {
            {80,20,0}, {85,10,0}, {88,0,0}, {85,-10,0}, {80,-20,0}
        };
        const auto align = computeAxisAlignment(wristTemplateArb, anchorsArb);
        CHECK(!align.valid, "near-collinear anchors (no clear thumb outlier): honestly reports invalid rather than guessing");
    }

    // ---- 场景 4：不套随机变换(单位变换)时，对齐结果应该跟真值本身
    // 的方向约定基本吻合(核对对齐算法在"已经是正确朝向"的输入下不会
    // 无端把东西转歪)。----
    {
        const auto align = computeAxisAlignment(wristTemplateTrue, anchorsTrue);
        CHECK(align.valid, "identity-orientation scene: alignment succeeds");
        if (align.valid) {
            CHECK(align.thumbAnchorIndex == 0, "identity-orientation scene: correctly identifies thumb");
            std::vector<Vec3> aligned;
            for (auto& p : anchorsTrue) aligned.push_back(applyAxisAlignment(align, p));
            bool othersPositiveX = true;
            for (int i=1;i<5;++i) if (aligned[size_t(i)][0] <= 0) othersPositiveX = false;
            CHECK(othersPositiveX, "identity-orientation scene: non-thumb anchors still have positive +X");
        }
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
