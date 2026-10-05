#include "hand/HandCalibration.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static double dist3(const HandVec3& a, const HandVec3& b) {
    const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

// 构造一个任意的世界系刚体位姿(轴角旋转 + 平移)，用来把"局部系点"转到
// "世界系点"，模拟真实标定时手在空间中的某个姿势。
static void makeArbitraryPose(double rodrigues[3], HandVec3& t, HandMat3& R) {
    const double ang = std::sqrt(rodrigues[0]*rodrigues[0]+rodrigues[1]*rodrigues[1]+rodrigues[2]*rodrigues[2]);
    if (ang < 1e-9) { R = {1,0,0,0,1,0,0,0,1}; return; }
    const HandVec3 axis{rodrigues[0]/ang, rodrigues[1]/ang, rodrigues[2]/ang};
    double K[3][3] = {{0,-axis[2],axis[1]},{axis[2],0,-axis[0]},{-axis[1],axis[0],0}};
    double K2[3][3];
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) { double s=0; for (int k=0;k<3;++k) s+=K[i][k]*K[k][j]; K2[i][j]=s; }
    const double s=std::sin(ang), c=std::cos(ang);
    double Rm[3][3];
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) Rm[i][j] = (i==j?1.0:0.0) + s*K[i][j] + (1.0-c)*K2[i][j];
    R = { Rm[0][0],Rm[0][1],Rm[0][2], Rm[1][0],Rm[1][1],Rm[1][2], Rm[2][0],Rm[2][1],Rm[2][2] };
    (void)t;
}

int main() {
    std::mt19937 rng(20260720);
    std::uniform_real_distribution<double> jitter(-3.0, 3.0);   // 模拟三角化噪声(mm)
    std::normal_distribution<double> noise(0.0, 0.3);

    // ---- 场景 1：随便设计一套"跟占位模板完全不一样"的5点贴法(任意分布，
    // 非共线)，验证自动标定能正确还原——用推出来的模板 + 一个已知的真实
    // 手腕位姿，重建世界系坐标，看跟真值对不对得上；再用现有的 Kabsch
    // (solveHandBackPose)拿推出来的模板去反解这个位姿，看解不解得对，这
    // 才是真正有意义的检验(不是自己骗自己转一圈算了又转回去)。----
    {
        // "随便怎么贴"——故意用一套跟 HandModel.hpp 占位模板长得完全不同的
        // 局部坐标，模拟"用户自己随意贴的5个点"。
        const std::array<HandVec3,5> trueLocalTemplate = {{
            {5, 40, 0}, {70, -10, 15}, {20, -35, -8}, {60, 25, 5}, {0, 0, 0}
        }};

        double rod[3] = {0.3, -0.2, 0.4};
        HandVec3 t{120, -50, 800};
        HandMat3 R; makeArbitraryPose(rod, t, R);

        std::array<HandVec3,5> worldPoints{};
        for (int i=0;i<5;++i) {
            const auto wp = transformPoint(trueLocalTemplate[size_t(i)], R, t);
            worldPoints[size_t(i)] = { wp[0]+jitter(rng)*0.05, wp[1]+jitter(rng)*0.05, wp[2]+jitter(rng)*0.05 };
        }

        // forward hint：局部系里手指方向大致是 +X，找一个"局部+X方向上"的
        // 世界点(比如中指尖，这里用局部(100,0,0)代表，转到世界系)。
        const auto forwardLocal = HandVec3{100, 0, 0};
        const auto forwardWorld = transformPoint(forwardLocal, R, t);
        // dorsal hint：局部+Z方向在世界系下的朝向(标定时你知道自己让手背
        // 朝哪，这里直接算真值来模拟"你告诉系统的大致朝向")。
        const HandVec3 dorsalWorldDir = { R[6], R[7], R[8] };   // R的第3行 = 局部Z轴在世界系的分量

        auto calib = calibrateHandBackTemplate(worldPoints, forwardWorld, dorsalWorldDir);
        CHECK(calib.valid, "arbitrary non-degenerate placement: calibration succeeds");
        if (!calib.valid) std::printf("  failReason: %s\n", calib.failReason);

        if (calib.valid) {
            // 真正的检验：拿标定出来的模板 + 现成的 Kabsch，反解出一个位姿，
            // 再用这个位姿把模板变换回世界系，看跟三角化观测对不对得上。
            // 注意：calib.templateLocal 是在"标定函数自己定义的局部系"
            // (质心为原点、PCA/前向提示定的轴)下表达的，这个局部系一般
            // 不等于 trueLocalTemplate 用的那个(任意选的)局部系——两者是
            // 同一个刚体的两种不同参数化，直接比较两个 R 矩阵是比错了
            // 参照系，应该比"重建出来的世界点跟观测对不对得上"，这才是
            // 标定结果能不能被后续系统正确使用的真实检验标准。
            std::array<bool,5> allVisible{true,true,true,true,true};
            auto pose = solveHandBackPose(calib.templateLocal, worldPoints, allVisible);
            CHECK(pose.confident, "derived template + Kabsch: pose solve is confident");

            double maxReconErr = 0.0;
            for (int i=0;i<5;++i) {
                const auto reconstructed = transformPoint(calib.templateLocal[size_t(i)], pose.R, pose.t);
                maxReconErr = std::max(maxReconErr, dist3(reconstructed, worldPoints[size_t(i)]));
            }
            char msg[128];
            std::snprintf(msg, sizeof(msg), "derived template: round-trip reconstruction matches observed world points (max err=%.3fmm)", maxReconErr);
            CHECK(maxReconErr < 1.0, msg);
        }
    }

    // ---- 场景 2：换一套完全不同的"随便贴法"(again，形状、间距都不一样)，
    // 同样应该成功——证明"符合理论要求(不共线)就能被识别"这句话，不是
    // 只测了一种巧合能通过的布局。----
    {
        const std::array<HandVec3,5> trueLocalTemplate = {{
            {-20, 10, 30}, {45, 45, -5}, {10, -60, 0}, {80, 0, 20}, {0, 20, -30}
        }};
        double rod[3] = {-0.1, 0.35, -0.15};
        HandVec3 t{-30, 80, 600};
        HandMat3 R; makeArbitraryPose(rod, t, R);

        std::array<HandVec3,5> worldPoints{};
        for (int i=0;i<5;++i) worldPoints[size_t(i)] = transformPoint(trueLocalTemplate[size_t(i)], R, t);

        const auto forwardWorld = transformPoint(HandVec3{100,0,0}, R, t);
        const HandVec3 dorsalWorldDir = { R[6], R[7], R[8] };

        auto calib = calibrateHandBackTemplate(worldPoints, forwardWorld, dorsalWorldDir);
        CHECK(calib.valid, "second arbitrary placement: calibration also succeeds (not a fluke of the first layout)");
    }

    // ---- 场景 3：故意贴成(接近)一条直线——不满足"不共线"这个理论要求，
    // 应该被拒绝，而不是硬给出一个数值上能算但几何上退化的模板。----
    {
        const std::array<HandVec3,5> collinearLocal = {{
            {0,0,0}, {10,0.01,0}, {20,-0.01,0}, {30,0.02,0}, {40,0,0}
        }};
        HandMat3 R = {1,0,0,0,1,0,0,0,1};
        HandVec3 t{0,0,500};
        std::array<HandVec3,5> worldPoints{};
        for (int i=0;i<5;++i) worldPoints[size_t(i)] = transformPoint(collinearLocal[size_t(i)], R, t);

        const auto forwardWorld = transformPoint(HandVec3{100,50,0}, R, t);
        const HandVec3 dorsalWorldDir{0,0,1};

        auto calib = calibrateHandBackTemplate(worldPoints, forwardWorld, dorsalWorldDir);
        CHECK(!calib.valid, "near-collinear placement: correctly rejected by non-degeneracy check, not silently accepted");
    }

    // ---- 场景 4：forward hint 跟法向平行(没法投影出+X方向)——诚实报告
    // 失败，不产出一个胡乱的模板。用完全共面的模板(z全为0)保证PCA法向
    // 严格沿+Z，这样"沿法向"这个退化条件才是精确构造的，不受点云本身
    // 轻微不共面带来的法向偏斜影响。----
    {
        const std::array<HandVec3,5> planarLocalTemplate = {{
            {5, 40, 0}, {70, -10, 0}, {20, -35, 0}, {60, 25, 0}, {0, 0, 0}
        }};
        HandMat3 R = {1,0,0, 0,1,0, 0,0,1};
        HandVec3 t{0,0,500};
        std::array<HandVec3,5> worldPoints{};
        for (int i=0;i<5;++i) worldPoints[size_t(i)] = transformPoint(planarLocalTemplate[size_t(i)], R, t);

        const HandVec3 dorsalWorldDir{0,0,1};
        HandVec3 centroid{0,0,0};
        for (const auto& p : worldPoints) { centroid[0]+=p[0]; centroid[1]+=p[1]; centroid[2]+=p[2]; }
        for (double& c : centroid) c /= 5.0;
        // forward hint 严格沿质心正上方(纯+Z方向)——点云完全共面时PCA法向
        // 精确是(0,0,1)，减去该法向分量后 xRaw 精确归零，触发退化判定。
        const HandVec3 badForward = { centroid[0], centroid[1], centroid[2] + 50.0 };

        auto calib = calibrateHandBackTemplate(worldPoints, badForward, dorsalWorldDir);
        CHECK(!calib.valid, "forward hint parallel to dorsal normal: honestly fails instead of fabricating a frame");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
