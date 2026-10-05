#include "estimate/PointIEKF.hpp"
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

// 用"看向某点"的方式构造相机位姿(世界->相机，行主序 R[i*3+j]=R_ij)。
static CamPose lookAt(const Vec3& camPos, const Vec3& target) {
    const Vec3 forward = normalize(sub(target, camPos));
    Vec3 worldUp = {0,0,1};
    if (std::abs(forward[0]*worldUp[0]+forward[1]*worldUp[1]+forward[2]*worldUp[2]) > 0.99)
        worldUp = {0,1,0};
    const Vec3 right = normalize(cross(forward, worldUp));
    const Vec3 camUp = cross(right, forward);

    CamPose cam;
    cam.R = { right[0],right[1],right[2],
             -camUp[0],-camUp[1],-camUp[2],
              forward[0],forward[1],forward[2] };
    // t = -R*camPos
    for (int r=0;r<3;++r) {
        double s=0.0;
        for (int k=0;k<3;++k) s += cam.R[size_t(r*3+k)]*camPos[size_t(k)];
        cam.t[size_t(r)] = -s;
    }
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

static double dist3(const Vec3&a,const Vec3&b){
    const double dx=a[0]-b[0],dy=a[1]-b[1],dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

int main() {
    std::mt19937 rng(20260717);
    std::normal_distribution<double> noise(0.0, 0.003);   // 归一化坐标下的观测噪声

    // 三台相机围绕原点附近的观测体积摆放，看向体积中心。
    const std::vector<CamPose> cams = {
        lookAt({2000,0,500}, {0,0,500}),
        lookAt({-1000,1800,600}, {0,0,500}),
        lookAt({-1000,-1800,700}, {0,0,500}),
    };
    const Cov2 measSigma{0.003*0.003, 0.0, 0.003*0.003};

    auto makeMeasurements = [&](const Vec3& truePos, const std::vector<int>& activeCams) {
        std::vector<CameraMeasurement> ms;
        for (int ci : activeCams) {
            double nx, ny;
            if (!project(cams[size_t(ci)], truePos, nx, ny)) continue;
            CameraMeasurement m;
            m.cam = &cams[size_t(ci)];
            m.nx = nx + noise(rng); m.ny = ny + noise(rng);
            m.sigma = measSigma;
            ms.push_back(m);
        }
        return ms;
    };

    // ---- 场景 1：静止点，3 相机持续观测，多帧后应该收敛到接近真值，且
    // 位置不确定度(方差)应该随着观测积累而下降。----
    {
        const Vec3 truePos{50, 30, 480};
        PointIEKF6D filt({0,0,0}, /*initPosVar=*/1e6, /*initVelVar=*/1e4);

        const Vec3 initVar = filt.positionVariances();
        for (int f=0; f<15; ++f) {
            filt.predict(1.0/120.0, /*posProcessVar=*/1.0, /*velProcessVar=*/50.0);
            filt.update(makeMeasurements(truePos, {0,1,2}));
        }
        const double err = dist3(filt.position(), truePos);
        CHECK(err < 10.0, "static point: converges within 10mm of truth after 15 frames of 3-cam fusion");

        const Vec3 finalVar = filt.positionVariances();
        CHECK(finalVar[0] < initVar[0] && finalVar[1] < initVar[1] && finalVar[2] < initVar[2],
              "static point: position variance shrinks as observations accumulate");
    }

    // ---- 场景 2：匀速运动的点，恒速过程模型应该能跟上轨迹，稳态误差
    // 应该维持在小量级(不会随时间发散)。----
    {
        const Vec3 startPos{-200, 100, 500};
        const Vec3 vel{40.0, -20.0, 0.0};   // mm/s
        PointIEKF6D filt(startPos, /*initPosVar=*/25.0, /*initVelVar=*/1e3);

        const double dt = 1.0/120.0;
        double maxErrAfterWarmup = 0.0;
        for (int f=0; f<60; ++f) {
            const double tsec = double(f)*dt;
            const Vec3 truePos{ startPos[0]+vel[0]*tsec, startPos[1]+vel[1]*tsec, startPos[2]+vel[2]*tsec };
            filt.predict(dt, 1.0, 50.0);
            filt.update(makeMeasurements(truePos, {0,1,2}));
            if (f > 20) {   // 跳过前 20 帧的收敛暖启动阶段
                const Vec3 nextTrue{ startPos[0]+vel[0]*tsec, startPos[1]+vel[1]*tsec, startPos[2]+vel[2]*tsec };
                maxErrAfterWarmup = std::max(maxErrAfterWarmup, dist3(filt.position(), nextTrue));
            }
        }
        CHECK(maxErrAfterWarmup < 8.0, "constant-velocity motion: steady-state tracking error stays small");
    }

    // ---- 场景 3：遮挡——中途几帧只有1台或0台相机能看到，之后恢复到3台。
    // 遮挡期间协方差应该只增不减(没有新信息)，恢复后应该能重新收紧并
    // 继续正确跟踪。----
    {
        const Vec3 truePos{0, 0, 500};
        PointIEKF6D filt(truePos, 25.0, 1e3);
        for (int f=0; f<10; ++f) { filt.predict(1.0/120.0,1.0,50.0); filt.update(makeMeasurements(truePos,{0,1,2})); }
        const Vec3 varBeforeOcclusion = filt.positionVariances();

        // 遮挡 8 帧：完全没有观测。
        for (int f=0; f<8; ++f) { filt.predict(1.0/120.0,1.0,50.0); filt.update({}); }
        const Vec3 varDuringOcclusion = filt.positionVariances();
        CHECK(varDuringOcclusion[0] >= varBeforeOcclusion[0] && varDuringOcclusion[2] >= varBeforeOcclusion[2],
              "occlusion: variance does not decrease with zero observations");

        // 恢复：3 相机重新看到，应该重新收紧并回到真值附近。
        for (int f=0; f<15; ++f) { filt.predict(1.0/120.0,1.0,50.0); filt.update(makeMeasurements(truePos,{0,1,2})); }
        const double errAfterRecover = dist3(filt.position(), truePos);
        CHECK(errAfterRecover < 10.0, "occlusion recovery: re-converges close to truth once cameras see it again");
    }

    // ---- 场景 4：迭代重线性化确实有用——用完全能观测(3相机)的配置，但故意
    // 把初始猜测设得离真值很远(先验不确定度也大)，制造强非线性场景(远离
    // 线性化点时针孔投影的一阶近似误差更大)。用同一批观测数据对比"只
    // 线性化1次"和"迭代8次"，迭代版本的最终误差应该更小或至少不差。----
    {
        const Vec3 truePos{50, 30, 480};
        const Vec3 farInitGuess{400, 400, 750};   // 离真值 ~500mm 远的糟糕初始猜测

        std::vector<std::vector<CameraMeasurement>> frames;
        {
            std::mt19937 rngGen(555);
            std::normal_distribution<double> n2(0.0,0.003);
            for (int f=0; f<3; ++f) {
                std::vector<CameraMeasurement> ms;
                for (const auto& c : cams) {
                    double nx, ny; if (!project(c, truePos, nx, ny)) continue;
                    CameraMeasurement m; m.cam=&c; m.nx=nx+n2(rngGen); m.ny=ny+n2(rngGen); m.sigma=measSigma;
                    ms.push_back(m);
                }
                frames.push_back(ms);
            }
        }

        PointIEKF6D filtSinglePass(farInitGuess, 3e5, 1e3);
        PointIEKF6D filtIterated(farInitGuess, 3e5, 1e3);

        for (const auto& frame : frames) {
            filtSinglePass.predict(1.0/120.0, 1.0, 50.0);
            filtSinglePass.update(frame, /*maxIters=*/1);
            filtIterated.predict(1.0/120.0, 1.0, 50.0);
            filtIterated.update(frame, /*maxIters=*/8);
        }

        const double errSingle = dist3(filtSinglePass.position(), truePos);
        const double errIterated = dist3(filtIterated.position(), truePos);
        char msg[160];
        std::snprintf(msg, sizeof(msg), "iterated relinearization helps under strong nonlinearity (1-iter err=%.2f, 8-iter err=%.2f)",
                     errSingle, errIterated);
        CHECK(errIterated <= errSingle + 1e-6, msg);
    }

    // ---- 场景 5：退化情况不崩溃——空观测列表(遮挡完全)时预测结果原样
    // 保留，不产生 NaN。----
    {
        PointIEKF6D filt({10,10,10}, 100.0, 1e3);
        filt.predict(1.0/120.0, 1.0, 50.0);
        filt.update({});
        const auto p = filt.position();
        CHECK(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]),
              "empty measurement update: state stays finite, no NaN");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
