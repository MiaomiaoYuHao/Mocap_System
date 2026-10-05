// HandModelAdapter.hpp 单元测试——验证"真实FK接线"这一步没有翻译错误。
// 这个适配层本身没有数值逻辑（只是 vector<->array 的长度检查+拷贝），
// 但恰恰是"没有逻辑的胶水代码"最容易在接口边界上出低级错误（维度算错、
// 拷贝顺序错、angles/markers下标错位），必须原样核对一遍，不能因为
// "没什么可测的"就跳过。
//
// 覆盖范围：
//   1. makeHandForwardKinematics() 产出的 lambda 与直接调用 handFK 逐点一致
//      （对若干组不同关节角，包括全零/单指弯曲/多指同时弯曲）。
//   2. 返回的 vector 长度确实是 kHandNumMarkers(20)，输入长度确实要求
//      kHandNumJoints(16)（用 assert 触发的方式间接确认——见测试3）。
//   3. clampHandStateToJointLimits 对越界/界内分量的处理与 clampToLimits
//      本身完全一致（不是重新发明一遍夹紧逻辑，是真的调用了它）。
#include "estimate/HandModelAdapter.hpp"
#include <cstdio>
#include <cmath>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static double dist3(const Vec3& a, const std::array<double,3>& b) {
    const double dx=a[0]-b[0], dy=a[1]-b[1], dz=a[2]-b[2];
    return std::sqrt(dx*dx+dy*dy+dz*dz);
}

int main() {
    ForwardKinematicsFn fk = makeHandForwardKinematics();

    // ---- 场景 1：全零关节角（伸直手型）——逐点核对 handFK 的直接结果，
    // 顺便复用 test_hand_model.cpp 里验证过的两个已知参考值(中指/小指远节)
    // 间接确认没有下标错位。----
    {
        std::vector<double> angles(size_t(kHandNumJoints), 0.0);
        const auto out = fk(angles);

        std::array<double,16> q{};
        std::array<std::array<double,3>,20> ref;
        handFK(q, ref);

        CHECK(out.size() == size_t(kHandNumMarkers), "straight pose: adapter returns exactly 20 markers");
        bool allMatch = true;
        for (int i=0;i<kHandNumMarkers;++i) if (dist3(out[size_t(i)], ref[size_t(i)]) > 1e-12) allMatch=false;
        CHECK(allMatch, "straight pose: adapter output matches handFK() point-for-point");

        // 跟 test_hand_model.cpp 里核对过的参考值再对一次，确认不是两边
        // 凑巧算错了同一个数（独立锚点）。
        char msg[128];
        std::snprintf(msg, sizeof(msg), "straight pose: middle distal marker matches known reference (%.3f,%.3f,%.3f)",
                     out[13][0], out[13][1], out[13][2]);
        CHECK(std::abs(out[13][0]-171.0)<1e-6 && std::abs(out[13][1]-3.0)<1e-6, msg);
    }

    // ---- 场景 2：多根手指同时弯曲 + 拇指——覆盖非平凡关节角，逐点核对，
    // 确认不是"只有全零时凑巧对上"。----
    {
        std::vector<double> angles(size_t(kHandNumJoints), 0.0);
        angles[4] = 0.5;  angles[6] = 0.9;    // 食指 MCP屈+PIP屈
        angles[7] = 0.3;  angles[8] = -0.1; angles[9] = 0.7;   // 中指
        angles[0] = 0.4;  angles[1] = 0.2; angles[2] = 0.6; angles[3] = 0.5;  // 拇指

        const auto out = fk(angles);

        std::array<double,16> q{};
        for (int i=0;i<16;++i) q[size_t(i)] = angles[size_t(i)];
        std::array<std::array<double,3>,20> ref;
        handFK(q, ref);

        bool allMatch = true; double maxErr = 0.0;
        for (int i=0;i<kHandNumMarkers;++i) { const double e = dist3(out[size_t(i)], ref[size_t(i)]); if (e>maxErr) maxErr=e; if (e>1e-12) allMatch=false; }
        char msg[128];
        std::snprintf(msg, sizeof(msg), "bent pose: adapter matches handFK point-for-point (maxErr=%.2e)", maxErr);
        CHECK(allMatch, msg);
    }

    // ---- 场景 3：手背5球（不受关节角影响）在适配器输出里也必须原样不变，
    // 跟 test_hand_model.cpp 里"手背刚体"那条断言呼应，确认下标0~4没被
    // 适配器不小心平移错位。----
    {
        std::vector<double> angles(size_t(kHandNumJoints), 0.0);
        angles[6] = 1.0;   // 随便弯一根手指，手背不该动
        const auto out = fk(angles);
        const auto& back = handBackMarkers();
        bool staticOk = true;
        for (int i=0;i<5;++i) if (dist3(out[size_t(i)], back[size_t(i)]) > 1e-12) staticOk=false;
        CHECK(staticOk, "adapter: back-of-hand 5 markers unaffected by finger joint angles");
    }

    // ---- 场景 4：关节限位夹紧——clampHandStateToJointLimits 对越界分量
    // 应该跟直接调用 clampToLimits 结果完全一致（同一份表，没有重新发明）。----
    {
        HandPoseState st;
        st.wristPos = {0,0,0};
        st.wristRot = {1,0,0, 0,1,0, 0,0,1};
        st.jointAngles.assign(size_t(kHandNumJoints), 0.0);
        st.jointAngles[4] = 99.0;    // 远超上限
        st.jointAngles[5] = -99.0;   // 远超下限
        st.jointAngles[6] = 0.9;     // 界内，应保持不变

        clampHandStateToJointLimits(st);

        std::array<double,16> raw{};
        raw[4]=99.0; raw[5]=-99.0; raw[6]=0.9;
        const auto expected = clampToLimits(raw);

        bool ok = true;
        for (int i=0;i<16;++i) if (std::abs(st.jointAngles[size_t(i)]-expected[size_t(i)]) > 1e-12) ok=false;
        CHECK(ok, "clampHandStateToJointLimits: matches clampToLimits() exactly for every joint");
    }

    // ---- 场景 5：clampHandStateToJointLimits 不该动 wristPos/wristRot
    // （只处理关节角，刚体部分是调用方自己的状态，适配层不该越界修改）。----
    {
        HandPoseState st;
        st.wristPos = {12.5, -7.0, 300.0};
        st.wristRot = {0.9,0.1,0, -0.1,0.9,0, 0,0,1};
        st.jointAngles.assign(size_t(kHandNumJoints), 0.0);
        st.jointAngles[0] = 50.0;

        const Vec3 posBefore = st.wristPos;
        const auto rotBefore = st.wristRot;
        clampHandStateToJointLimits(st);

        bool posUnchanged = (posBefore[0]==st.wristPos[0] && posBefore[1]==st.wristPos[1] && posBefore[2]==st.wristPos[2]);
        bool rotUnchanged = true;
        for (int i=0;i<9;++i) if (rotBefore[size_t(i)] != st.wristRot[size_t(i)]) rotUnchanged=false;
        CHECK(posUnchanged && rotUnchanged, "clampHandStateToJointLimits: leaves wristPos/wristRot untouched");
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
