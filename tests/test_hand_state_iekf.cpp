#include "estimate/HandStateIEKF.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static Vec3 cross(const Vec3&a, const Vec3&b){ return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]}; }
static Vec3 sub(const Vec3&a,const Vec3&b){ return {a[0]-b[0],a[1]-b[1],a[2]-b[2]}; }
static Vec3 normalize(const Vec3&v){ const double n=std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); return {v[0]/n,v[1]/n,v[2]/n}; }

static CamPose lookAt(const Vec3& camPos, const Vec3& target) {
    const Vec3 forward = normalize(sub(target, camPos));
    Vec3 worldUp = {0,0,1};
    if (std::abs(forward[0]*worldUp[0]+forward[1]*worldUp[1]+forward[2]*worldUp[2]) > 0.99) worldUp = {0,1,0};
    const Vec3 right = normalize(cross(forward, worldUp));
    const Vec3 camUp = cross(right, forward);
    CamPose cam;
    cam.R = { right[0],right[1],right[2], -camUp[0],-camUp[1],-camUp[2], forward[0],forward[1],forward[2] };
    for (int r=0;r<3;++r) { double s=0.0; for (int k=0;k<3;++k) s += cam.R[size_t(r*3+k)]*camPos[size_t(k)]; cam.t[size_t(r)] = -s; }
    return cam;
}
static bool project(const CamPose& cam, const Vec3& p, double& nx, double& ny) {
    const auto& R=cam.R; const auto& t=cam.t;
    const double Xc=R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0];
    const double Yc=R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1];
    const double Zc=R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2];
    if (Zc<=1e-6) return false;
    nx=Xc/Zc; ny=Yc/Zc; return true;
}
static double dist3(const Vec3&a,const Vec3&b){const double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];return std::sqrt(dx*dx+dy*dy+dz*dz);}

// 玩具 FK：一根两关节的"手指"，模拟真实 HandModel.hpp 的接口形状(关节角
// 数组 -> marker 局部位置数组)，但结构简化成好验证的程度：
//   marker0: 手腕原点固定点 (0,0,0)
//   marker1: 第一节末端，长度 L1，绕本地 Z 轴转 angle0
//   marker2: 第二节末端，在第一节基础上绕本地 Z 轴再转 angle1，长度 L2
//   marker3: 手腕系另一个固定点 (0,20,0)（模拟手背上跟手指无关的固定 marker）
static std::vector<Vec3> toyFK(const std::vector<double>& angles) {
    const double L1=40.0, L2=35.0;
    const double a0 = angles[0], a1 = angles[0]+angles[1];
    std::vector<Vec3> out;
    out.push_back({0,0,0});
    out.push_back({L1*std::cos(a0), L1*std::sin(a0), 0});
    out.push_back({L1*std::cos(a0)+L2*std::cos(a1), L1*std::sin(a0)+L2*std::sin(a1), 0});
    out.push_back({0,20,0});
    return out;
}

int main() {
    std::mt19937 rng(20260718);
    std::normal_distribution<double> noise(0.0, 0.003);

    const std::vector<CamPose> cams = {
        lookAt({2000,0,500}, {0,0,500}),
        lookAt({-1000,1800,600}, {0,0,500}),
        lookAt({-1000,-1800,700}, {0,0,500}),
    };
    const Cov2 sigma{0.003*0.003, 0.0, 0.003*0.003};

    // 真值：手腕位姿 + 两个关节角。
    const Vec3 trueWristPos{50, 20, 480};
    double trueRodrigues[3] = {0.1, -0.15, 0.05};   // 小转角，构造一个非单位旋转
    double trueR[3][3];
    {
        const double ang = std::sqrt(trueRodrigues[0]*trueRodrigues[0]+trueRodrigues[1]*trueRodrigues[1]+trueRodrigues[2]*trueRodrigues[2]);
        const Vec3 axis{trueRodrigues[0]/ang, trueRodrigues[1]/ang, trueRodrigues[2]/ang};
        double K[3][3] = {{0,-axis[2],axis[1]},{axis[2],0,-axis[0]},{-axis[1],axis[0],0}};
        double K2[3][3];
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) { double s=0; for (int k=0;k<3;++k) s+=K[i][k]*K[k][j]; K2[i][j]=s; }
        const double s=std::sin(ang), c=std::cos(ang);
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) trueR[i][j] = (i==j?1.0:0.0) + s*K[i][j] + (1.0-c)*K2[i][j];
    }
    const std::vector<double> trueAngles = {0.4, 0.6};

    auto worldMarkers = [&](const std::vector<double>& angles) {
        auto local = toyFK(angles);
        std::vector<Vec3> world;
        for (auto& lp : local) {
            Vec3 rotated = { trueR[0][0]*lp[0]+trueR[0][1]*lp[1]+trueR[0][2]*lp[2],
                            trueR[1][0]*lp[0]+trueR[1][1]*lp[1]+trueR[1][2]*lp[2],
                            trueR[2][0]*lp[0]+trueR[2][1]*lp[1]+trueR[2][2]*lp[2] };
            world.push_back({ rotated[0]+trueWristPos[0], rotated[1]+trueWristPos[1], rotated[2]+trueWristPos[2] });
        }
        return world;
    };

    auto makeFrame = [&](const std::vector<double>& angles) {
        const auto world = worldMarkers(angles);
        std::vector<HandCameraMeasurement> ms;
        for (const auto& cam : cams) {
            for (int m=0; m<int(world.size()); ++m) {
                double nx, ny;
                if (!project(cam, world[size_t(m)], nx, ny)) continue;
                HandCameraMeasurement meas;
                meas.cam = &cam; meas.markerIndex = m;
                meas.nx = nx + noise(rng); meas.ny = ny + noise(rng);
                meas.sigma = sigma;
                ms.push_back(meas);
            }
        }
        return ms;
    };

    // ---- 场景 1：静态收敛——从一个明显偏离真值的初始猜测出发，多帧融合
    // 后手腕位置、旋转、关节角都应该收敛到接近真值。----
    {
        HandPoseState init;
        init.wristPos = {0,0,400};
        init.wristRot = {1,0,0, 0,1,0, 0,0,1};   // 单位旋转，真值是有旋转的，故意给错
        init.jointAngles = {0.0, 0.0};

        HandStateIEKF filt(toyFK, 2, init, /*initPosVar=*/1e5, /*initRotVar=*/1.0, /*initJointVar=*/1.0);

        for (int f=0; f<100; ++f) {
            filt.predict(0.05, 1e-5, 1e-5);
            filt.update(makeFrame(trueAngles));
        }

        const auto& st = filt.state();
        const double posErr = dist3(st.wristPos, trueWristPos);
        char msg[128];
        std::snprintf(msg, sizeof(msg), "wrist position converges close to truth (err=%.2fmm)", posErr);
        CHECK(posErr < 5.0, msg);

        double rotErrSq=0.0;
        for (int i=0;i<9;++i) { const double d = st.wristRot[size_t(i)] - trueR[i/3][i%3]; rotErrSq += d*d; }
        std::snprintf(msg, sizeof(msg), "wrist rotation converges close to truth (frobenius err=%.4f)", std::sqrt(rotErrSq));
        CHECK(std::sqrt(rotErrSq) < 0.12, msg);

        std::snprintf(msg, sizeof(msg), "joint angle 0 converges (est=%.3f true=%.3f)", st.jointAngles[0], trueAngles[0]);
        CHECK(std::abs(st.jointAngles[0]-trueAngles[0]) < 0.06, msg);
        std::snprintf(msg, sizeof(msg), "joint angle 1 converges (est=%.3f true=%.3f)", st.jointAngles[1], trueAngles[1]);
        CHECK(std::abs(st.jointAngles[1]-trueAngles[1]) < 0.06, msg);
    }

    // ---- 场景 2：只用手背上的固定 marker(0号和3号，跟关节角无关)去看，
    // 手腕刚体位姿本身应该也能收敛——验证"关节角部分观测不到时不影响
    // 刚体部分的估计"这个解耦性质。----
    {
        HandPoseState init;
        init.wristPos = {0,0,400};
        init.wristRot = {1,0,0, 0,1,0, 0,0,1};
        init.jointAngles = {0.0, 0.0};
        HandStateIEKF filt(toyFK, 2, init, 1e5, 1.0, 1.0);

        for (int f=0; f<100; ++f) {
            filt.predict(0.05, 1e-5, 1e-5);
            auto full = makeFrame(trueAngles);
            std::vector<HandCameraMeasurement> backOnly;
            for (auto& m : full) if (m.markerIndex==0 || m.markerIndex==3) backOnly.push_back(m);
            filt.update(backOnly);
        }
        const double posErr = dist3(filt.state().wristPos, trueWristPos);
        char msg[128];
        std::snprintf(msg, sizeof(msg), "wrist-only markers still converge wrist position (err=%.2fmm)", posErr);
        CHECK(posErr < 8.0, msg);
    }

    // ---- 场景 3：退化情况不崩溃——空观测时状态原样保留，不产生 NaN。----
    {
        HandPoseState init; init.wristPos={0,0,400}; init.wristRot={1,0,0,0,1,0,0,0,1}; init.jointAngles={0,0};
        HandStateIEKF filt(toyFK, 2, init, 100.0, 0.1, 0.1);
        filt.predict(1.0, 0.01, 0.01);
        filt.update({});
        const auto& st = filt.state();
        CHECK(std::isfinite(st.wristPos[0]) && std::isfinite(st.jointAngles[0]), "empty measurement update: state stays finite");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
