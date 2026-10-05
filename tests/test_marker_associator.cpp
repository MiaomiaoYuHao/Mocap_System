#include "estimate/MarkerAssociator.hpp"
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

// 玩具手模型：手腕局部系里 6 颗散开的 marker（前两颗是手指链，后四颗是
// 手背上跟关节角无关的固定点），关节角有 2 个自由度。散开是为了让投影
// 到相机里彼此分得清（关联层要靠这个）。
static std::vector<Vec3> toyFK6(const std::vector<double>& angles) {
    const double L1=40.0, L2=35.0;
    const double a0=angles[0], a1=angles[0]+angles[1];
    return {
        {L1*std::cos(a0), L1*std::sin(a0), 0},
        {L1*std::cos(a0)+L2*std::cos(a1), L1*std::sin(a0)+L2*std::sin(a1), 0},
        {0,0,0}, {0,40,0}, {-30,20,10}, {30,20,-10},
    };
}

int main() {
    std::mt19937 rng(20260719);
    std::normal_distribution<double> noise(0.0, 0.002);

    const std::vector<CamPose> cams = {
        lookAt({2000,0,500}, {0,0,500}),
        lookAt({-1000,1800,600}, {0,0,500}),
        lookAt({-1000,-1800,700}, {0,0,500}),
    };
    const Cov2 sigma{0.002*0.002, 0.0, 0.002*0.002};

    const Vec3 trueWristPos{40, 20, 480};
    const std::array<double,9> trueRot{1,0,0, 0,1,0, 0,0,1};   // 单位旋转，保持测试聚焦在关联逻辑
    const std::vector<double> trueAngles{0.4, 0.6};

    auto worldMarkers = [&](const std::vector<double>& angles) {
        auto local = toyFK6(angles);
        std::vector<Vec3> world;
        for (auto& lp : local) {
            Vec3 v = { trueRot[0]*lp[0]+trueRot[1]*lp[1]+trueRot[2]*lp[2],
                       trueRot[3]*lp[0]+trueRot[4]*lp[1]+trueRot[5]*lp[2],
                       trueRot[6]*lp[0]+trueRot[7]*lp[1]+trueRot[8]*lp[2] };
            world.push_back({ v[0]+trueWristPos[0], v[1]+trueWristPos[1], v[2]+trueWristPos[2] });
        }
        return world;
    };

    // 构造这一帧每台相机的原始检测（无标签，顺序故意打乱以确保关联不是
    // 靠"检测顺序恰好等于 marker 顺序"作弊）。
    auto buildFrames = [&](const std::vector<double>& angles, bool addDecoy) {
        const auto world = worldMarkers(angles);
        std::vector<CameraFrame> frames;
        for (int ci=0; ci<int(cams.size()); ++ci) {
            CameraFrame cf; cf.camIndex=ci; cf.cam=&cams[size_t(ci)];
            for (int m=0;m<int(world.size());++m) {
                double nx,ny;
                if (!project(cams[size_t(ci)], world[size_t(m)], nx, ny)) continue;
                RawDetection d; d.nx=nx+noise(rng); d.ny=ny+noise(rng); d.sigma=sigma;
                cf.detections.push_back(d);
            }
            if (addDecoy) {
                // 一个跟任何 marker 都不对应的干扰点（远离所有期望投影）。
                double nx,ny; project(cams[size_t(ci)], {trueWristPos[0]+300, trueWristPos[1]+300, trueWristPos[2]}, nx, ny);
                RawDetection d; d.nx=nx; d.ny=ny; d.sigma=sigma;
                cf.detections.push_back(d);
            }
            std::shuffle(cf.detections.begin(), cf.detections.end(), rng);
            frames.push_back(cf);
        }
        return frames;
    };

    ForwardKinematicsFn fk = toyFK6;

    // ---- 场景 1：给定接近真值的当前状态，关联结果里每条 measurement 的
    // markerIndex 打得对不对——用"该 measurement 的观测应该接近它 claim 的
    // 那个 marker 的真实投影"来验证标签正确。----
    {
        HandPoseState cur; cur.wristPos=trueWristPos; cur.wristRot=trueRot; cur.jointAngles=trueAngles;
        MarkerAssociator assoc(fk, /*gateRadiusNorm=*/0.02);
        auto frames = buildFrames(trueAngles, /*addDecoy=*/false);
        auto result = assoc.associate(cur, frames);

        const auto world = worldMarkers(trueAngles);
        bool allLabelsCorrect = true;
        for (const auto& meas : result.measurements) {
            double ex, ey;
            project(*meas.cam, world[size_t(meas.markerIndex)], ex, ey);
            const double err = std::sqrt((meas.nx-ex)*(meas.nx-ex)+(meas.ny-ey)*(meas.ny-ey));
            if (err > 0.01) allLabelsCorrect = false;   // 标签错了的话观测会离它 claim 的 marker 很远
        }
        CHECK(allLabelsCorrect, "association labels every measurement with the correct markerIndex");
        CHECK(int(result.measurements.size()) == 3*6, "all 6 markers associated in all 3 cameras");
        CHECK(result.unassigned.empty(), "no spurious unassigned detections when input is clean");
    }

    // ---- 场景 2：加入干扰点，干扰点应该落在 unassigned 里，不该被任何
    // marker 误认领（门控半径挡住它）。----
    {
        HandPoseState cur; cur.wristPos=trueWristPos; cur.wristRot=trueRot; cur.jointAngles=trueAngles;
        MarkerAssociator assoc(fk, 0.02);
        auto frames = buildFrames(trueAngles, /*addDecoy=*/true);
        auto result = assoc.associate(cur, frames);
        CHECK(int(result.measurements.size()) == 3*6, "decoy present: still exactly 6 real markers per camera associated");
        CHECK(int(result.unassigned.size()) == 3, "decoy present: exactly the 3 decoys (one per camera) left unassigned");
    }

    // ---- 场景 3：当前状态有一定偏差（模拟上一帧滤波结果不完美），只要
    // 偏差没大到超过门控半径，关联仍应正确——验证这层对预测误差有容忍度。----
    {
        HandPoseState cur; cur.wristPos={trueWristPos[0]+5, trueWristPos[1]-4, trueWristPos[2]+3};
        cur.wristRot=trueRot; cur.jointAngles={trueAngles[0]+0.02, trueAngles[1]-0.02};
        MarkerAssociator assoc(fk, 0.03);
        auto frames = buildFrames(trueAngles, false);
        auto result = assoc.associate(cur, frames);
        CHECK(int(result.measurements.size()) == 3*6, "tolerates moderate prior error: all markers still associated");
    }

    // ---- 场景 4：完整闭环 pipeline —— 冷启动(直接给真值附近的初始位姿) +
    // 逐帧关联+IEKF更新，跑一段后状态应该收敛/维持在真值附近。这是检测层
    // 到手部状态估计之间的端到端验证。----
    {
        ColdStartFn coldStart = [&](const std::vector<CameraFrame>&) -> std::optional<HandPoseState> {
            HandPoseState s;
            s.wristPos = {trueWristPos[0]+15, trueWristPos[1]+15, trueWristPos[2]-15};   // 冷启动是个粗略估计
            s.wristRot = trueRot;
            s.jointAngles = {0.2, 0.3};   // 关节角初值也粗略
            return s;
        };

        HandTrackingPipeline pipe(fk, /*numJoints=*/2, /*gateRadiusNorm=*/0.05, coldStart);

        bool everStepped = false;
        double sumJ0=0.0, sumJ1=0.0; int avgCount=0;
        Vec3 sumPos{0,0,0};
        for (int f=0; f<120; ++f) {
            auto frames = buildFrames(trueAngles, /*addDecoy=*/true);
            const bool ok = pipe.step(frames, /*posProcessVar=*/0.05, /*rotProcessVar=*/1e-5, /*jointProcessVar=*/1e-5);
            everStepped = everStepped || ok;
            if (f >= 100 && pipe.hasState()) {   // 收敛后取稳态区间做平均（弱可观测的关节角逐帧有噪声，均值才是收敛量）
                const auto& s = pipe.state();
                sumJ0 += s.jointAngles[0]; sumJ1 += s.jointAngles[1];
                sumPos[0]+=s.wristPos[0]; sumPos[1]+=s.wristPos[1]; sumPos[2]+=s.wristPos[2];
                ++avgCount;
            }
        }
        CHECK(everStepped && pipe.hasState(), "pipeline: cold-started and maintained a state");

        const Vec3 avgPos{sumPos[0]/avgCount, sumPos[1]/avgCount, sumPos[2]/avgCount};
        const double avgJ0 = sumJ0/avgCount, avgJ1 = sumJ1/avgCount;
        const double posErr = dist3(avgPos, trueWristPos);
        char msg[128];
        std::snprintf(msg, sizeof(msg), "pipeline end-to-end: steady-state wrist position converges to truth (err=%.2fmm)", posErr);
        CHECK(posErr < 5.0, msg);
        std::snprintf(msg, sizeof(msg), "pipeline end-to-end: steady-state joint 0 converges (avg=%.3f true=0.400)", avgJ0);
        CHECK(std::abs(avgJ0-trueAngles[0]) < 0.08, msg);
        std::snprintf(msg, sizeof(msg), "pipeline end-to-end: steady-state joint 1 converges (avg=%.3f true=0.600)", avgJ1);
        CHECK(std::abs(avgJ1-trueAngles[1]) < 0.08, msg);

        // 干扰点每帧都在，闭环里应该被持续丢进 unassigned，不污染状态。
        CHECK(int(pipe.lastUnassigned().size()) == 3, "pipeline: decoys consistently rejected as unassigned each frame");
    }

    // ---- 场景 5：冷启动失败（回调返回 nullopt）时，pipeline 跳过该帧、
    // 不崩溃、不产生状态。----
    {
        ColdStartFn neverStarts = [](const std::vector<CameraFrame>&) -> std::optional<HandPoseState> { return std::nullopt; };
        HandTrackingPipeline pipe(fk, 2, 0.05, neverStarts);
        auto frames = buildFrames(trueAngles, false);
        const bool ok = pipe.step(frames);
        CHECK(!ok && !pipe.hasState(), "pipeline: gracefully skips frame when cold-start cannot establish a pose");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
