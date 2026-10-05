// HandColdStart.hpp 单元测试——距离约束回溯搜索是这一轮新写的、之前完全
// 没验证过的数学（区别于 HandPose.hpp 的 Kabsch，那个已经被 test_hand_pose.cpp
// 独立复核过；这里测的是"在一堆无标签候选点里找出哪5个是手背模板"这一层，
// Kabsch 本身不重复验证）。
//
// 覆盖范围：
//   1. 干净场景：候选点=手背5点(加噪)+若干手指点(装饰性干扰)，乱序，应该
//      正确找出手背5点组合并解出接近真值的 (R,t)。
//   2. 候选点不足5个：诚实返回 nullopt，不崩溃。
//   3. 候选点里没有任何一组匹配模板距离模式：诚实返回 nullopt。
//   4. 容差边界：候选点距离跟模板差一点点(在 tolerance 内)应该仍然匹配；
//      差太多(超出 tolerance)不该匹配。
//   5. 多组候选都通过距离检验时，应该选 RMS 最小、且过 confident 判定的
//      那一组，不是随便选第一个。
//   6. coldStartHandPose 封装：产出的 HandPoseState.jointAngles 应为中性
//      初值(0)、长度正确，wristPos/wristRot 来自选中的最佳配准。
//   7. 候选点数量较大时(模拟20颗全手球+噪声)不应该因组合爆炸而失控太久
//      (用 maxAcceptedMatches 限制，只验证"跑得完、结果仍正确"，不做计时)。
#include "estimate/HandColdStart.hpp"
#include <cstdio>
#include <cmath>
#include <random>
#include <algorithm>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static HandMat3 randRot(std::mt19937& g) {
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
static double dist3(const std::array<double,3>& a, const std::array<double,3>& b) {
    const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}
static Vec3 toVec3(const HandVec3& v) { return {v[0], v[1], v[2]}; }

int main() {
    std::mt19937 rng(20260720);
    std::normal_distribution<double> smallNoise(0.0, 0.5);   // mm，模拟三角化噪声

    const auto& templ = handBackMarkers();

    // ---- 场景 1：干净场景——手背5点(带噪声) + 一堆装饰性干扰点(模拟手指
    // 球，距离模式跟手背模板不匹配)，全部打乱顺序，应该正确挑出手背5点
    // 并解出接近真值的位姿。----
    {
        const HandMat3 Rt = randRot(rng);
        const HandVec3 tt = {80, -20, 500};

        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            HandVec3 w = transformPoint(templ[size_t(i)], Rt, tt);
            candidates.push_back({ w[0]+smallNoise(rng), w[1]+smallNoise(rng), w[2]+smallNoise(rng) });
        }
        // 装饰性干扰点：随便撒几个跟手背模板距离模式对不上的点。
        candidates.push_back({tt[0]+200, tt[1]+50, tt[2]-30});
        candidates.push_back({tt[0]-150, tt[1]+80, tt[2]+40});
        candidates.push_back({tt[0]+90, tt[1]-160, tt[2]+10});
        std::shuffle(candidates.begin(), candidates.end(), rng);

        const auto pose = matchHandBackTemplate(candidates);
        CHECK(pose.has_value(), "clean scene: found a confident match among candidates + decoys");
        if (pose) {
            double rotErrFrob = 0.0;
            for (int i=0;i<9;++i) { const double d = pose->R[size_t(i)]-Rt[size_t(i)]; rotErrFrob += d*d; }
            rotErrFrob = std::sqrt(rotErrFrob);
            const double tErr = dist3(pose->t, tt);
            char msg[160];
            std::snprintf(msg, sizeof(msg), "clean scene: recovered pose close to truth (rotErr=%.4f, posErr=%.3fmm, rms=%.3f)",
                         rotErrFrob, tErr, pose->rms);
            // 阈值留了余量（0.5mm级噪声、5点配准的期望误差量级），避免因
            // 单次随机噪声抽样偏大而误报——见旁注：多组随机种子压测过。
            CHECK(rotErrFrob < 0.15 && tErr < 5.0, msg);
        }
    }

    // ---- 场景 2：候选点不足5个——诚实返回 nullopt，不崩溃。----
    {
        std::vector<Vec3> tiny = { {0,0,0}, {10,0,0}, {0,10,0}, {5,5,5} };
        const auto pose = matchHandBackTemplate(tiny);
        CHECK(!pose.has_value(), "too few candidates (<5): honestly returns nullopt");
    }

    // ---- 场景 3：候选点里没有任何一组匹配模板距离模式(比如全部随机撒开，
    // 两两距离跟模板完全不匹配)——诚实返回 nullopt。----
    {
        std::vector<Vec3> randomPts;
        std::uniform_real_distribution<double> spread(-500,500);
        for (int i=0;i<8;++i) randomPts.push_back({spread(rng), spread(rng), spread(rng)});
        const auto pose = matchHandBackTemplate(randomPts);
        CHECK(!pose.has_value(), "no matching subset among random scattered points: honestly returns nullopt");
    }

    // ---- 场景 4a：容差边界内——候选点距离比模板略有偏差(在 distToleranceMm
    // 之内)，仍应匹配成功。----
    {
        const HandMat3 Rt = {1,0,0, 0,1,0, 0,0,1};   // 单位旋转，简化验证
        const HandVec3 tt = {0,0,400};
        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            HandVec3 w = transformPoint(templ[size_t(i)], Rt, tt);
            // 加一个固定小偏移(2mm级)，在默认 tolerance(8mm)内。
            candidates.push_back({ w[0]+1.5, w[1]-1.0, w[2]+0.8 });
        }
        const auto pose = matchHandBackTemplate(candidates);
        CHECK(pose.has_value(), "within-tolerance distance perturbation: still matches");
    }

    // ---- 场景 4b：容差边界外——把模板距离整体拉伸一个明显超出容差的比例
    // (比如每条边都被"实际手背"和"模板"之间量出来的巨大偏差撑大)，不该
    // 匹配成功(距离检验应该正确拦住"形状对不上"的情况)。----
    {
        const HandVec3 tt = {0,0,400};
        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            // 把模板点整体从质心放大2倍距离，破坏两两距离模式(不再是刚体等距变换)。
            HandVec3 scaled = { templ[size_t(i)][0]*2.5, templ[size_t(i)][1]*2.5, templ[size_t(i)][2]*2.5 };
            candidates.push_back({ scaled[0]+tt[0], scaled[1]+tt[1], scaled[2]+tt[2] });
        }
        const auto pose = matchHandBackTemplate(candidates);
        CHECK(!pose.has_value(), "distance pattern scaled far beyond tolerance: correctly rejected");
    }

    // ---- 场景 5：多组候选都可能通过距离检验时，应选 RMS 最小且confident
    // 的那组——构造真手背5点 + 另一组"距离模式恰好也接近模板"的伪装点
    // (对同一模板加了较大但仍在容差内的扰动，RMS 会更差)，验证最终选中
    // 的是残差更小的那一组。----
    {
        const HandMat3 Rt = randRot(rng);
        const HandVec3 tt = {30, 40, 450};

        std::vector<Vec3> candidates;
        // 真实一组：噪声很小。
        for (int i=0;i<5;++i) {
            HandVec3 w = transformPoint(templ[size_t(i)], Rt, tt);
            candidates.push_back({ w[0]+smallNoise(rng)*0.2, w[1]+smallNoise(rng)*0.2, w[2]+smallNoise(rng)*0.2 });
        }
        // 伪装一组：同样的旋转/平移，但每个点加较大扰动(仍在容差内，但会
        // 让 RMS 明显更差)，摆在别处避免跟真实组距离太近产生跨组误配对。
        const HandVec3 tt2 = {30, 40, 800};
        std::normal_distribution<double> bigNoise(0.0, 4.0);
        for (int i=0;i<5;++i) {
            HandVec3 w = transformPoint(templ[size_t(i)], Rt, tt2);
            candidates.push_back({ w[0]+bigNoise(rng), w[1]+bigNoise(rng), w[2]+bigNoise(rng) });
        }
        std::shuffle(candidates.begin(), candidates.end(), rng);

        const auto pose = matchHandBackTemplate(candidates);
        CHECK(pose.has_value(), "two plausible groups present: still finds a confident match");
        if (pose) {
            // 应该选中残差更小的一组——判据：还原出的平移应更接近tt(真实小噪声组)
            // 而不是tt2(大噪声组)，因为前者RMS更低。
            const double errToClean = dist3(pose->t, tt);
            const double errToNoisy = dist3(pose->t, tt2);
            char msg[160];
            std::snprintf(msg, sizeof(msg), "picks lower-RMS group over higher-noise group (errClean=%.2f errNoisy=%.2f rms=%.3f)",
                         errToClean, errToNoisy, pose->rms);
            CHECK(errToClean < errToNoisy, msg);
        }
    }

    // ---- 场景 6：coldStartHandPose 封装——产出的 HandPoseState 关节角
    // 长度正确、全为中性初值0，wristPos/wristRot 来自选中的最佳配准。----
    {
        const HandMat3 Rt = {1,0,0, 0,1,0, 0,0,1};
        const HandVec3 tt = {10,-10,600};
        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            HandVec3 w = transformPoint(templ[size_t(i)], Rt, tt);
            candidates.push_back(toVec3(w));
        }
        const auto state = coldStartHandPose(candidates, kHandNumJoints);
        CHECK(state.has_value(), "coldStartHandPose: succeeds on clean 5-point input");
        if (state) {
            CHECK(int(state->jointAngles.size()) == kHandNumJoints, "coldStartHandPose: jointAngles has correct length");
            bool allZero = true;
            for (double a : state->jointAngles) if (a != 0.0) allZero = false;
            CHECK(allZero, "coldStartHandPose: joint angles initialized to neutral (0)");
            const double posErr = dist3({state->wristPos[0],state->wristPos[1],state->wristPos[2]}, tt);
            CHECK(posErr < 1e-6, "coldStartHandPose: wristPos matches solved translation");
        }
    }

    // ---- 场景 7：候选点数量较大(模拟20颗全手球都混进来，只有5颗是手背)
    // 时不应该失控——用 maxAcceptedMatches 兜底，结果仍应正确找到手背5点。
    // 这里只验证正确性和"跑完了"，不量时间(计时在不同机器上没有稳定基线，
    // 意义不大)。----
    {
        const HandMat3 Rt = randRot(rng);
        const HandVec3 tt = {0, 0, 550};

        std::vector<Vec3> candidates;
        for (int i=0;i<5;++i) {
            HandVec3 w = transformPoint(templ[size_t(i)], Rt, tt);
            candidates.push_back({ w[0]+smallNoise(rng)*0.3, w[1]+smallNoise(rng)*0.3, w[2]+smallNoise(rng)*0.3 });
        }
        // 额外15个点模拟手指球：用跟手背模板距离数量级不同的随机分布，
        // 减少偶然凑出匹配距离模式的概率(15点里两两距离都刻意撒得比较散)。
        std::uniform_real_distribution<double> fingerSpread(-120, 120);
        for (int i=0;i<15;++i) {
            candidates.push_back({ tt[0]+fingerSpread(rng), tt[1]+fingerSpread(rng), tt[2]+fingerSpread(rng)*0.3 });
        }
        std::shuffle(candidates.begin(), candidates.end(), rng);

        const auto pose = matchHandBackTemplate(candidates);
        CHECK(pose.has_value(), "20-point crowded scene: search completes and finds a match");
        if (pose) {
            const double tErr = dist3(pose->t, tt);
            char msg[128];
            std::snprintf(msg, sizeof(msg), "20-point crowded scene: recovered pose still close to truth (posErr=%.3fmm)", tErr);
            CHECK(tErr < 5.0, msg);
        }
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
