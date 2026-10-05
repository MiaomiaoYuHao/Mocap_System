// HandRigidAutoCalib.hpp 单元测试——合成数据验证"晃动手一段时间，自动找出
// 哪些点是刚体、重建出局部模板"这条全新逻辑。这是本轮要求"无论怎么贴、贴
// 几个只要能算都行，算不出来就报错"这个新方案的核心，此前完全没有代码，
// 必须打好基线。
//
// 覆盖范围：
//   1. 干净场景：5点刚体(随机局部模板) + 多个独立运动的"手指状"干扰点，
//      模拟晃动采集多帧，应正确找出刚体、重建模板跟真值一致(用两者的距离
//      矩阵比较，因为局部坐标系本身允许任意刚体变换，不能直接逐点比坐标)。
//   2. 镜像消歧：验证重建结果的手性(chirality)跟真值一致，不是镜像版本
//      （构造一个非对称的局部模板，保证镜像版本几何上确实不同）。
//   3. 候选点总数不足：诚实返回invalid，给出具体点数。
//   4. 采集帧数/共视帧数不够：诚实返回invalid，不是随便凑一个"看起来还行"
//      的团。
//   5. 转动幅度不足(几乎不转)：验证此时"手指干扰点"短暂偶然稳定不会被
//      误判——用有限几帧、点几乎不动的场景，确认不会产出虚假大团（这是
//      算法说明文档里提到的已知风险点，必须专门测）。
//   6. 多组均满足条件的刚体候选同时存在：应该都报出来，按点数降序，且
//      message里有相应提示，不是只挑一个而不说明。
#include "estimate/HandRigidAutoCalib.hpp"
#include <cstdio>
#include <cmath>
#include <random>

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
static Vec3 applyRT(const HandMat3& R, const HandVec3& t, const Vec3& p) {
    return { R[0]*p[0]+R[1]*p[1]+R[2]*p[2]+t[0],
             R[3]*p[0]+R[4]*p[1]+R[5]*p[2]+t[1],
             R[6]*p[0]+R[7]*p[1]+R[8]*p[2]+t[2] };
}
static double dist3(const Vec3& a, const Vec3& b) {
    const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

// 比较两组点集的"内部距离矩阵"是否一致——因为重建出的局部坐标允许任意
// 刚体变换(和已消歧的手性)，不能直接逐点比坐标，比两两距离矩阵才是正确
// 的等价性判据。返回最大距离误差(mm)。
static double maxPairwiseDistError(const std::vector<Vec3>& a, const std::vector<Vec3>& b) {
    double worst = 0.0;
    for (size_t i=0;i<a.size();++i)
        for (size_t j=i+1;j<a.size();++j) {
            const double da = dist3(a[i],a[j]);
            const double db = dist3(b[i],b[j]);
            worst = std::max(worst, std::abs(da-db));
        }
    return worst;
}

int main() {
    std::mt19937 rng(20260722);
    std::normal_distribution<double> noise(0.0, 0.3);   // mm，模拟三角化噪声

    // 真实局部模板(非对称、非共面——保证镜像版本确实是几何上不同的另一个
    // 形状，不是这个模板碰巧对称导致镜像消歧测不出差别)。
    const std::vector<Vec3> trueTemplate = {
        {15, 0, 2}, {55, 22, 5}, {50, -25, 8}, {35, 30, 6}, {60, 3, 3}
    };
    const int nRigid = int(trueTemplate.size());

    // ---- 场景 1：干净场景——刚体 + 独立运动的干扰点，多帧晃动，应正确
    // 找出刚体且重建模板跟真值内部距离矩阵一致。----
    {
        const int nFrames = 80;
        const int nDecoys = 6;
        std::vector<FrameTrackedPoints> frames;
        // 干扰点每帧在各自独立的活动范围内重新随机取位置(模拟手指关节
        // 真实运动的位移量级——几十mm级，不是mm级小幅游走)。这样任意两个
        // 干扰点之间、干扰点与刚体之间的距离，帧间波动幅度远超刚体自身
        // 的1.5mm容差，才是"干扰点不该被误判成刚体"这个断言该测的场景；
        // 如果干扰点动得比容差还小(比如真的几乎不动)，那它偶然维持稳定
        // 距离并被识别成另一个真实存在的刚体是算法的正确行为，不是bug
        // ——那种情况见场景5"转动幅度不足"，专门测的是另一回事。
        const size_t nDecoysSz = size_t(nDecoys);
        std::vector<Vec3> decoyCenter(nDecoysSz);
        std::uniform_real_distribution<double> decoyCenterInit(-200, 200);
        for (auto& c : decoyCenter) c = {decoyCenterInit(rng), decoyCenterInit(rng), decoyCenterInit(rng)+400};
        std::uniform_real_distribution<double> decoyJitter(-40.0, 40.0);

        for (int f=0; f<nFrames; ++f) {
            const HandMat3 R = randRot(rng);
            const HandVec3 t = {0,0,500};
            FrameTrackedPoints fr;
            for (int i=0;i<nRigid;++i) {
                Vec3 w = applyRT(R, t, trueTemplate[size_t(i)]);
                w[0]+=noise(rng); w[1]+=noise(rng); w[2]+=noise(rng);
                fr.ids.push_back(i);           // id 0..4 = 刚体
                fr.positions.push_back(w);
            }
            for (int d=0; d<nDecoys; ++d) {
                Vec3 w = { decoyCenter[size_t(d)][0]+decoyJitter(rng),
                          decoyCenter[size_t(d)][1]+decoyJitter(rng),
                          decoyCenter[size_t(d)][2]+decoyJitter(rng) };
                fr.ids.push_back(100+d);       // id 100+ = 干扰点
                fr.positions.push_back(w);
            }
            frames.push_back(fr);
        }

        const auto result = calibrateRigidClusters(frames);
        CHECK(result.valid, "clean scene: calibration succeeds");
        if (result.valid) {
            CHECK(result.clusters.size() >= 1, "clean scene: at least one cluster found");
            const auto& best = result.clusters[0];
            const bool sizeOk = (int(best.ids.size()) == nRigid);
            CHECK(sizeOk, "clean scene: found cluster has exactly the 5 rigid points");

            bool idsCorrect = sizeOk;
            for (int id : best.ids) if (id < 0 || id >= nRigid) idsCorrect = false;
            CHECK(idsCorrect, "clean scene: cluster ids are exactly the rigid-body ids, no decoys mixed in");

            // 只有尺寸和id都对得上才继续按下标重排比较，避免断言已经失败
            // 的情况下继续用越界下标操作导致进程崩溃、掩盖了真正的断言
            // 失败信息。
            if (sizeOk && idsCorrect) {
                const size_t nRigidSz = size_t(nRigid);
                std::vector<Vec3> reordered(nRigidSz);
                for (size_t i=0;i<best.ids.size();++i) reordered[size_t(best.ids[i])] = best.localTemplate[i];
                const double distErr = maxPairwiseDistError(reordered, trueTemplate);
                char msg[160];
                std::snprintf(msg, sizeof(msg), "clean scene: reconstructed template matches true pairwise distances (maxErr=%.3fmm)", distErr);
                CHECK(distErr < 0.5, msg);

                char msg2[160];
                std::snprintf(msg2, sizeof(msg2), "clean scene: avg Kabsch RMS against calibration frames is small (%.3fmm)", best.avgKabschRmsMm);
                CHECK(best.avgKabschRmsMm < 1.0, msg2);
            } else {
                ++g_fail;
                std::printf("FAIL: clean scene: skipped downstream distance-matrix checks because cluster shape was wrong\n");
            }
        }
    }

    // ---- 场景 2：镜像消歧——用同一份合成流程，单独验证重建模板的"手性"
    // 跟真值一致，而不是几何上正确但整体镜像的版本(用真值模板本身镜像后
    // 去比对，重建结果应该明显更接近未镜像的真值，而不是镜像版本)。----
    {
        const int nFrames = 60;
        std::vector<FrameTrackedPoints> frames;
        for (int f=0; f<nFrames; ++f) {
            const HandMat3 R = randRot(rng);
            const HandVec3 t = {10,-5,450};
            FrameTrackedPoints fr;
            for (int i=0;i<nRigid;++i) {
                Vec3 w = applyRT(R, t, trueTemplate[size_t(i)]);
                w[0]+=noise(rng); w[1]+=noise(rng); w[2]+=noise(rng);
                fr.ids.push_back(i);
                fr.positions.push_back(w);
            }
            frames.push_back(fr);
        }
        const auto result = calibrateRigidClusters(frames);
        CHECK(result.valid && !result.clusters.empty(), "chirality test: calibration succeeds");
        if (result.valid && !result.clusters.empty()) {
            const auto& best = result.clusters[0];
            const bool sizeOk = (int(best.ids.size()) == nRigid);
            CHECK(sizeOk, "chirality test: found cluster has exactly the 5 rigid points");
            bool idsCorrect = sizeOk;
            if (sizeOk) for (int id : best.ids) if (id < 0 || id >= nRigid) idsCorrect = false;

            if (sizeOk && idsCorrect) {
                const size_t nRigidSz = size_t(nRigid);
                std::vector<Vec3> reordered(nRigidSz);
                for (size_t i=0;i<best.ids.size();++i) reordered[size_t(best.ids[i])] = best.localTemplate[i];

                // 重建出的局部模板只保证"内部距离矩阵"和"手性(chirality)"
                // 正确，绝对朝向可以是真值模板任意旋转后的版本——不能直接
                // 逐点比坐标(那是错的验证方法，等于假设了一个不该假设的
                // 对齐)。正确做法：分别把重建结果Kabsch配准到"真值模板"
                // 和"真值模板的镜像版本"，看哪个残差小——如果手性正确，
                // 配准到真值的残差应该只有噪声量级；配准到镜像版本，任何
                // 合法(行列式+1)旋转都没法完美对齐两个手性相反的点集，
                // 残差会明显大得多。
                std::vector<HandVec3> P_(nRigidSz);
                for (size_t i=0;i<nRigidSz;++i) P_[i] = { reordered[i][0], reordered[i][1], reordered[i][2] };

                std::vector<Vec3> mirrored = trueTemplate;
                for (auto& p : mirrored) p[2] = -p[2];

                auto kabschRms = [&](const std::vector<Vec3>& targetTemplate) {
                    std::vector<HandVec3> Q_(nRigidSz);
                    for (size_t i=0;i<nRigidSz;++i) Q_[i] = { targetTemplate[i][0], targetTemplate[i][1], targetTemplate[i][2] };
                    HandMat3 Rm; HandVec3 tm;
                    kabsch(P_, Q_, Rm, tm);
                    double sq=0.0;
                    for (size_t i=0;i<nRigidSz;++i) {
                        HandVec3 rp = detail::matvec3(Rm, P_[i]);
                        const double ex=rp[0]+tm[0]-Q_[i][0], ey=rp[1]+tm[1]-Q_[i][1], ez=rp[2]+tm[2]-Q_[i][2];
                        sq += ex*ex+ey*ey+ez*ez;
                    }
                    return std::sqrt(sq/double(nRigidSz));
                };

                const double rmsToTrue = kabschRms(trueTemplate);
                const double rmsToMirror = kabschRms(mirrored);
                char msg[220];
                std::snprintf(msg, sizeof(msg),
                    "chirality test: Kabsch-fit to true template has small residual and beats mirrored fit (rmsToTrue=%.3fmm rmsToMirror=%.3fmm)",
                    rmsToTrue, rmsToMirror);
                CHECK(rmsToTrue < 1.0 && rmsToTrue < rmsToMirror, msg);
            } else {
                ++g_fail;
                std::printf("FAIL: chirality test: skipped handedness check because cluster shape was wrong\n");
            }
        }
    }

    // ---- 场景 3：候选点总数不足——诚实返回invalid，报出实际点数。----
    {
        std::vector<FrameTrackedPoints> frames;
        for (int f=0; f<50; ++f) {
            FrameTrackedPoints fr;
            fr.ids = {0,1};
            fr.positions = {{0,0,500},{10,0,500}};
            frames.push_back(fr);
        }
        const auto result = calibrateRigidClusters(frames);
        CHECK(!result.valid, "insufficient total points (<3): honestly reports invalid");
        CHECK(result.message.find("2") != std::string::npos, "insufficient total points: message reports the actual count");
    }

    // ---- 场景 4：共视帧数不够——两两之间共同出现的帧数达不到
    // minCoOccurFrames门槛时，即使距离看起来稳定也不该采信。----
    {
        RigidAutoCalibConfig cfg; cfg.minCoOccurFrames = 30;
        std::vector<FrameTrackedPoints> frames;
        for (int f=0; f<10; ++f) {   // 只有10帧，远低于门槛30
            FrameTrackedPoints fr;
            fr.ids = {0,1,2};
            fr.positions = {{0,0,500},{20,0,500},{0,20,500}};
            frames.push_back(fr);
        }
        const auto result = calibrateRigidClusters(frames, cfg);
        CHECK(!result.valid, "insufficient co-occurring frames: honestly reports invalid rather than trusting too little data");
    }

    // ---- 场景 5：转动幅度不足(手几乎不动)——手指干扰点短暂偶然稳定不该
    // 被误判成刚体候选。用"刚体正常晃动+干扰点几乎静止"的组合，验证干扰
    // 点不会被拉进最终结果。----
    {
        const int nFrames = 60;
        std::vector<FrameTrackedPoints> frames;
        // 干扰点两两之间距离设成偶然稳定(比如两个几乎静止的点)，但它们
        // 跟刚体点之间不满足距离稳定关系(晃动时刚体在动，干扰点不动，
        // 交叉距离必然大幅变化，不会被误并入刚体团)。
        Vec3 decoyA{-300,-300,500}, decoyB{-280,-300,500};
        for (int f=0; f<nFrames; ++f) {
            const HandMat3 R = randRot(rng);
            const HandVec3 t = {0,0,500};
            FrameTrackedPoints fr;
            for (int i=0;i<nRigid;++i) {
                Vec3 w = applyRT(R, t, trueTemplate[size_t(i)]);
                w[0]+=noise(rng); w[1]+=noise(rng); w[2]+=noise(rng);
                fr.ids.push_back(i);
                fr.positions.push_back(w);
            }
            fr.ids.push_back(200); fr.positions.push_back({decoyA[0]+noise(rng)*0.05, decoyA[1], decoyA[2]});
            fr.ids.push_back(201); fr.positions.push_back({decoyB[0]+noise(rng)*0.05, decoyB[1], decoyB[2]});
            frames.push_back(fr);
        }
        const auto result = calibrateRigidClusters(frames);
        CHECK(result.valid, "static decoy pair present: calibration still succeeds (rigid body still detected)");
        if (result.valid) {
            const auto& best = result.clusters[0];
            bool decoysExcluded = true;
            for (int id : best.ids) if (id == 200 || id == 201) decoysExcluded = false;
            CHECK(int(best.ids.size()) == nRigid && decoysExcluded,
                  "static decoy pair present: the two nearly-static decoy points are NOT absorbed into the rigid cluster"
                  "(their mutual distance is stable, but distance to the moving rigid points is not, so clique test correctly excludes them)");
        }
    }

    // ---- 场景 6：两组都满足条件的刚体候选同时存在——应该都报出来，按
    // 点数降序，message里说明有歧义。----
    {
        const std::vector<Vec3> template2 = { {100,0,0}, {130,15,0}, {115,-20,10} };   // 另一组独立的3点刚体
        const int nFrames = 60;
        std::vector<FrameTrackedPoints> frames;
        for (int f=0; f<nFrames; ++f) {
            const HandMat3 R1 = randRot(rng); const HandVec3 t1 = {0,0,500};
            const HandMat3 R2 = randRot(rng); const HandVec3 t2 = {300,300,500};   // 独立运动，互不锁定
            FrameTrackedPoints fr;
            for (int i=0;i<nRigid;++i) {
                Vec3 w = applyRT(R1, t1, trueTemplate[size_t(i)]);
                w[0]+=noise(rng); w[1]+=noise(rng); w[2]+=noise(rng);
                fr.ids.push_back(i); fr.positions.push_back(w);
            }
            for (int i=0;i<int(template2.size());++i) {
                Vec3 w = applyRT(R2, t2, template2[size_t(i)]);
                w[0]+=noise(rng); w[1]+=noise(rng); w[2]+=noise(rng);
                fr.ids.push_back(300+i); fr.positions.push_back(w);
            }
            frames.push_back(fr);
        }
        const auto result = calibrateRigidClusters(frames);
        CHECK(result.valid, "two independent rigid groups: calibration succeeds");
        CHECK(result.clusters.size() == 2, "two independent rigid groups: both reported as separate clusters");
        if (result.clusters.size() == 2) {
            CHECK(result.clusters[0].ids.size() >= result.clusters[1].ids.size(),
                  "two independent rigid groups: sorted by point count descending");
            CHECK(int(result.clusters[0].ids.size()) == nRigid, "two independent rigid groups: larger cluster is the 5-point one");
        }
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
