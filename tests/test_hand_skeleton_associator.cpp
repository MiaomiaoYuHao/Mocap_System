// HandSkeletonAssociator.hpp 的胶水逻辑测试——不需要真的加载ONNX模型/不
// 需要onnxruntime。用一个"直接给定指派矩阵"的假后端(FakeBackend)，只验证
// "推理输出 -> 匈牙利指派 -> 落标签 -> Kabsch腕部位姿 -> 分段朝向"这条链路
// 本身对不对；AI模型的分类准确率是训练时另外衡量的，不是这个单测的职责。
//
// 【本次重写：迁到 hm20 契约】原测试写的是上一代接口
//     ISkeletonInferenceBackend::run(const InferenceInput&, const SkeletonTemplate&)
//     InferenceOutput::perPointLogits (1,N,16) / jointPrediction (15,3)
//     SkeletonFrameResult::joints / palmDiag / joints[i].orientation
// 这些符号在 hm20 版本里都不存在。新契约是：
//     IHm20InferenceBackend::run(const InferenceInput&)
//     InferenceOutput::logAssign (N+1)x21 对数指派矩阵 / pos (20,3)
//     SkeletonFrameResult::markers / segQuat[16] / wristPoseValid
//
// 场景 4/5/6 是本次三处修复的回归测试，改回去就会红。
#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <memory>

using namespace mocap::hm20;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

// ---------------------------------------------------------------------------
// 假后端：直接给定 (N+1)x21 的对数指派矩阵和 20 点位置预测。
// ---------------------------------------------------------------------------
class FakeBackend : public IHm20InferenceBackend {
public:
    InferenceOutput fixedOutput;
    int lastN = -1;
    InferenceOutput run(const InferenceInput& in) override {
        lastN = int(in.points.size());
        return fixedOutput;
    }
};

// 构造一个"第 i 个点就是第 i 个标签"的理想指派矩阵。
// rows = N+1（最后一行是 dustbin 行），cols = 21。
static InferenceOutput makeIdentityAssign(int N, const std::array<Vec3,20>& posPred,
                                          const std::vector<int>& labelOfPoint) {
    InferenceOutput o;
    o.rows = N + 1;
    o.cols = kNumClasses;
    o.logAssign.assign(size_t(o.rows) * size_t(o.cols), -20.0f);   // exp(-20)≈2e-9
    for (int i = 0; i < N; ++i) {
        const int lab = labelOfPoint[size_t(i)];
        // log(0.99)≈-0.01：远高于 minAssignProb 默认 0.15
        o.logAssign[size_t(i) * size_t(kNumClasses) + size_t(lab)] = -0.01f;
    }
    o.pos = posPred;
    o.ok = true;
    return o;
}

// 中立位 20 点模板（毫米），手背 5 点 + 每指 3 点。数值只要自洽即可。
static std::array<Vec3,20> makeTemplateMm() {
    std::array<Vec3,20> m{};
    m[0] = {18, 2, 14};   m[1] = {62, 26, 16};  m[2] = {66, -28, 15};
    m[3] = {30, -22, 13}; m[4] = {34, 24, 15};
    const double anchor[5][3] = {{12,38,-6},{88,22,0},{92,3,1},{88,-16,0},{80,-34,-1}};
    const double len[5][3]    = {{46,31,21},{44,26,17},{49,31,19},{45,28,18},{36,21,15}};
    for (int f = 0; f < 5; ++f) {
        double n = 0; for (int c = 0; c < 3; ++c) n += anchor[f][c]*anchor[f][c];
        n = std::sqrt(n);
        const double d[3] = {anchor[f][0]/n, anchor[f][1]/n, anchor[f][2]/n};
        double joint[3] = {anchor[f][0], anchor[f][1], anchor[f][2]};
        for (int k = 0; k < 3; ++k) {
            const double L = len[f][k];
            m[size_t(5 + f*3 + k)] = {joint[0] + d[0]*L*0.5,
                                      joint[1] + d[1]*L*0.5,
                                      joint[2] + d[2]*L*0.5};
            for (int c = 0; c < 3; ++c) joint[c] += d[c]*L;
        }
    }
    return m;
}

static std::array<float,61> packTmpl(const std::array<Vec3,20>& m, bool isRight) {
    // 与 Hm20Template::packNormalized() 同一套归一化
    const Vec3& a = m[11];
    double s = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    for (int k = 11; k < 13; ++k) {
        const Vec3& p = m[size_t(k)]; const Vec3& q = m[size_t(k+1)];
        const double dx=q[0]-p[0], dy=q[1]-p[1], dz=q[2]-p[2];
        s += std::sqrt(dx*dx+dy*dy+dz*dz);
    }
    if (s < 1e-6) s = 1.0;
    std::array<float,61> t{};
    int k = 0;
    for (int i = 0; i < 20; ++i) for (int c = 0; c < 3; ++c) t[size_t(k++)] = float(m[size_t(i)][size_t(c)] / s);
    t[60] = isRight ? 1.0f : 0.0f;
    return t;
}

static double quatAngleDeg(const Quat& a, const Quat& b) {
    double d = std::fabs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
    if (d > 1.0) d = 1.0;
    return 2.0 * std::acos(d) * 180.0 / M_PI;
}

int main() {
    const std::array<Vec3,20> tmplMm = makeTemplateMm();
    const std::array<float,61> tmpl = packTmpl(tmplMm, true);

    // 世界变换：绕 z 转 30°，再平移
    const double c = std::cos(30.0*M_PI/180.0), s2 = std::sin(30.0*M_PI/180.0);
    const Mat3 R = {c, -s2, 0,  s2, c, 0,  0, 0, 1};   // 行主序
    const Vec3 T = {200, 100, 900};
    auto toWorld = [&](const Vec3& l) -> Vec3 {
        return { R[0]*l[0]+R[1]*l[1]+R[2]*l[2] + T[0],
                 R[3]*l[0]+R[4]*l[1]+R[5]*l[2] + T[1],
                 R[6]*l[0]+R[7]*l[1]+R[8]*l[2] + T[2] };
    };

    std::array<Vec3,20> world{};
    for (int i = 0; i < 20; ++i) world[size_t(i)] = toWorld(tmplMm[size_t(i)]);

    // ---- 场景1：20 点全见，标签全对 ----
    // 验证：①解得出 ②全部 observed ③腕部位姿等于真值 R,T ④wristPoseValid
    {
        auto backend = std::make_shared<FakeBackend>();
        std::vector<std::pair<int,Vec3>> cands;
        std::vector<int> labels;
        for (int i = 0; i < 20; ++i) { cands.push_back({i, world[size_t(i)]}); labels.push_back(i); }
        backend->fixedOutput = makeIdentityAssign(20, world, labels);

        Hm20SkeletonAssociator assoc(backend, tmpl, /*tmplValid=*/true);
        assoc.setTemplateMm(tmplMm);   // Kabsch 用毫米模板，跟网络的归一化模板分开
        const SkeletonFrameResult res = assoc.process(cands);

        CHECK(res.valid, "scenario1: 20点全见时应当解出结果");
        CHECK(res.numObserved == 20, "scenario1: 20个标签全部被认领");
        CHECK(res.wristPoseValid, "scenario1: 手背5点全见 -> 腕部位姿是本帧解出来的");
        CHECK(res.dorsumRmseMm >= 0.0 && res.dorsumRmseMm < 1e-6,
              "scenario1: 无噪声时 Kabsch 残差应当接近 0");
        double rerr = 0;
        for (int i = 0; i < 9; ++i) rerr += std::fabs(res.wristR[size_t(i)] - R[size_t(i)]);
        CHECK(rerr < 1e-6, "scenario1: 解出的腕部旋转等于构造时用的真值");
        for (int i = 0; i < 20; ++i)
            CHECK(res.markers[size_t(i)].observed, "scenario1: 每个标签都标记为 observed");
        CHECK(res.markers[0].sourcePointId >= 0, "scenario1: observed 的标签带回源点 id");
    }

    // ---- 场景2：候选点不足 3 个 -> 如实报失败，不硬凑 ----
    {
        auto backend = std::make_shared<FakeBackend>();
        Hm20SkeletonAssociator assoc(backend, tmpl, true);
        std::vector<std::pair<int,Vec3>> cands = {{0,{0,0,900}}, {1,{10,10,900}}};
        const SkeletonFrameResult res = assoc.process(cands);
        CHECK(!res.valid, "scenario2: 候选点<3 时返回 invalid，不编造结果");
        CHECK(!res.message.empty(), "scenario2: 失败原因如实报出，不静默");
    }

    // ---- 场景3：后端返回的张量尺寸不符合契约 -> 防御性拒绝，不越界读 ----
    {
        auto backend = std::make_shared<FakeBackend>();
        std::vector<std::pair<int,Vec3>> cands;
        std::vector<int> labels;
        for (int i = 0; i < 20; ++i) { cands.push_back({i, world[size_t(i)]}); labels.push_back(i); }
        backend->fixedOutput = makeIdentityAssign(20, world, labels);
        backend->fixedOutput.cols = 16;              // 冒充上一代模型的 16 类
        Hm20SkeletonAssociator assoc(backend, tmpl, true);
        assoc.setTemplateMm(tmplMm);
        const SkeletonFrameResult res = assoc.process(cands);
        CHECK(!res.valid, "scenario3: 列数不等于 21 时必须拒绝（上一代模型误配的典型症状）");
    }

    // ---- 场景4【回归】：近节与中节的四元数不得恒等 ----
    // 原实现 computeSegmentQuats 里 k==0 与 k==1 算出的是同一个向量，
    // 导致每根手指 3 节骨只有 2 个独立朝向。改回去这条就会红。
    {
        auto backend = std::make_shared<FakeBackend>();
        std::vector<std::pair<int,Vec3>> cands;
        std::vector<int> labels;
        // 让手指弯一点，否则三节共线时"恒等"和"不恒等"分不出来
        std::array<Vec3,20> bent = world;
        for (int f = 0; f < 5; ++f) {
            bent[size_t(5+f*3+1)][2] += 8.0;
            bent[size_t(5+f*3+2)][2] += 22.0;
        }
        for (int i = 0; i < 20; ++i) { cands.push_back({i, bent[size_t(i)]}); labels.push_back(i); }
        backend->fixedOutput = makeIdentityAssign(20, bent, labels);

        Hm20SkeletonAssociator assoc(backend, tmpl, true);
        assoc.setTemplateMm(tmplMm);
        const SkeletonFrameResult res = assoc.process(cands);
        CHECK(res.valid, "scenario4: 弯曲手指时应当解出结果");
        for (int f = 0; f < 5; ++f) {
            const double d = quatAngleDeg(res.segQuat[size_t(1+f*3)], res.segQuat[size_t(2+f*3)]);
            CHECK(d > 1e-3, "scenario4[回归]: 近节与中节的四元数不得恒等");
        }
    }

    // ---- 场景5【回归】：boneFrame 不得因骨轴世界 z 分量跨 0.9 而突变 ----
    // 原实现 ref = (abs(x[2])<0.9) ? {0,0,1} : {0,1,0} 是硬分支，骨骼连续
    // 运动时四元数会无预警翻转。这里让某一指的骨轴缓慢扫过该阈值，检查
    // 相邻两次的四元数变化不出现跳变。
    {
        double maxJump = 0.0;
        Quat prev{1,0,0,0};
        bool hasPrev = false;
        // 【注意】必须复用同一个 associator 实例。退化姿态下的连续性依赖
        // 上一帧的段 y 轴，每步新建实例等于每帧都冷启动，测不出连续性
        // （我第一版就是这么写的，白红了一轮）。
        auto backend = std::make_shared<FakeBackend>();
        Hm20SkeletonAssociator assoc(backend, tmpl, true);
        assoc.setTemplateMm(tmplMm);
        for (int step = 0; step <= 200; ++step) {
            // 让食指近节骨轴从接近水平转到接近竖直，扫过 |x_z| = 0.9
            const double th = (M_PI/2.0) * double(step) / 200.0;
            std::array<Vec3,20> p = world;
            const Vec3 base = world[8];
            p[9]  = {base[0] + 40.0*std::cos(th), base[1], base[2] + 40.0*std::sin(th)};
            p[10] = {base[0] + 70.0*std::cos(th), base[1], base[2] + 70.0*std::sin(th)};

            std::vector<std::pair<int,Vec3>> cands;
            std::vector<int> labels;
            for (int i = 0; i < 20; ++i) { cands.push_back({i, p[size_t(i)]}); labels.push_back(i); }
            backend->fixedOutput = makeIdentityAssign(20, p, labels);
            const SkeletonFrameResult res = assoc.process(cands);
            if (!res.valid) continue;
            const Quat& q = res.segQuat[4];          // index_prox
            if (hasPrev) maxJump = std::max(maxJump, quatAngleDeg(q, prev));
            prev = q; hasPrev = true;
        }
        // 每步骨轴只转 0.45°，四元数的相邻变化应当同量级。原实现在阈值处
        // 会出现几十度的突跳。
        std::printf("  [场景5] 骨轴扫过 |x_z|=0.9 时的最大帧间四元数跳变: %.2f°\n", maxJump);
        CHECK(maxJump < 5.0, "scenario5[回归]: boneFrame 参考向量不得在 |x_z|=0.9 处突变");
    }

    // ---- 场景6【回归】：手背可见点<3 时不得退化到全 20 点 Kabsch ----
    // 模板里的手指是中立位，拿它跟当前帧弯曲的手指做刚体拟合，等于把关节
    // 运动当成腕部刚体运动。现在的行为：不解，沿用上一帧，wristPoseValid=false。
    {
        auto backend = std::make_shared<FakeBackend>();
        Hm20SkeletonAssociator assoc(backend, tmpl, true);
        assoc.setTemplateMm(tmplMm);

        // 先喂一帧完整的，建立"上一帧腕部位姿"
        {
            std::vector<std::pair<int,Vec3>> cands;
            std::vector<int> labels;
            for (int i = 0; i < 20; ++i) { cands.push_back({i, world[size_t(i)]}); labels.push_back(i); }
            backend->fixedOutput = makeIdentityAssign(20, world, labels);
            const SkeletonFrameResult r0 = assoc.process(cands);
            CHECK(r0.wristPoseValid, "scenario6: 第一帧手背全见，位姿有效");
        }

        // 第二帧：手背只剩 2 个点可见，手指全见且大幅弯曲
        std::array<Vec3,20> p = world;
        for (int f = 0; f < 5; ++f)
            for (int k = 0; k < 3; ++k) p[size_t(5+f*3+k)][2] += 30.0 * (k + 1);

        std::vector<std::pair<int,Vec3>> cands;
        std::vector<int> labels;
        for (int i = 0; i < 2; ++i)  { cands.push_back({i, p[size_t(i)]}); labels.push_back(i); }
        for (int i = 5; i < 20; ++i) { cands.push_back({i, p[size_t(i)]}); labels.push_back(i); }
        backend->fixedOutput = makeIdentityAssign(int(cands.size()), p, labels);

        const SkeletonFrameResult res = assoc.process(cands);
        CHECK(res.valid, "scenario6: 手背点不足时仍应返回有效结果（只是位姿是外推的）");
        CHECK(!res.wristPoseValid, "scenario6[回归]: 手背可见点<3 时 wristPoseValid 必须为 false");
        CHECK(res.dorsumRmseMm < 0.0, "scenario6[回归]: 没解 Kabsch 时残差保持 -1，不伪造");
        double rerr = 0;
        for (int i = 0; i < 9; ++i) rerr += std::fabs(res.wristR[size_t(i)] - R[size_t(i)]);
        CHECK(rerr < 1e-6, "scenario6[回归]: 腕部位姿沿用上一帧，未被弯曲的手指污染");
    }

    // ---- 场景7：分段朝向的 X 轴应当沿骨轴方向 ----
    {
        auto backend = std::make_shared<FakeBackend>();
        std::array<Vec3,20> p = world;
        p[9]  = {world[8][0] + 40, world[8][1], world[8][2]};
        p[10] = {world[8][0] + 70, world[8][1], world[8][2] + 10};
        std::vector<std::pair<int,Vec3>> cands;
        std::vector<int> labels;
        for (int i = 0; i < 20; ++i) { cands.push_back({i, p[size_t(i)]}); labels.push_back(i); }
        backend->fixedOutput = makeIdentityAssign(20, p, labels);

        Hm20SkeletonAssociator assoc(backend, tmpl, true);
        assoc.setTemplateMm(tmplMm);
        const SkeletonFrameResult res = assoc.process(cands);
        CHECK(res.valid, "scenario7: 应当解出结果");

        // 近节段用 pp->mp 的方向
        Vec3 dir = {p[9][0]-p[8][0], p[9][1]-p[8][1], p[9][2]-p[8][2]};
        const double n = std::sqrt(dir[0]*dir[0]+dir[1]*dir[1]+dir[2]*dir[2]);
        dir = {dir[0]/n, dir[1]/n, dir[2]/n};

        // 四元数转回矩阵，取第一列
        const Quat& q = res.segQuat[4];
        const double w=q[0], x=q[1], y=q[2], z=q[3];
        const Vec3 xAxis = {1-2*(y*y+z*z), 2*(x*y+z*w), 2*(x*z-y*w)};
        const double dot = xAxis[0]*dir[0] + xAxis[1]*dir[1] + xAxis[2]*dir[2];
        CHECK(std::fabs(dot) > 0.999, "scenario7: 分段朝向的 X 轴沿骨轴方向");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail > 0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
