#pragma once
// ---------------------------------------------------------------------------
// 把 hand/HandModel.hpp 的真实 FK 包成 estimate 层要的 ForwardKinematicsFn。
// 之前 HandStateIEKF.hpp/MarkerAssociator.hpp 用的是一个玩具 FK 占位，这个
// 文件是唯一需要改的接线点——两个头文件本身不用动。
//
// 类型换算说明：HandModel.hpp 用的是 std::array<double,16> / std::array<
// std::array<double,3>,20>（固定长度，编译期已知手指数量），estimate 层用
// 的是 std::vector<double> / std::vector<Vec3>（因为 HandStateIEKF 设计成
// 关节数量可配置，不绑死 16）。这里做的转换只是长度检查 + 逐元素拷贝，没有
// 数值逻辑，出错的话大概率是关节数/marker数假设跟 HandModel.hpp 实际不一致，
// 断言会先炸出来而不是产出静默错误的结果。
// ---------------------------------------------------------------------------
#include "hand/HandModel.hpp"
#include "hand/HandPose.hpp"
#include "estimate/HandStateIEKF.hpp"
#include <cassert>

namespace mocap {

constexpr int kHandNumJoints = 16;
constexpr int kHandNumMarkers = 20;

// 直接传给 HandStateIEKF / MarkerAssociator 的构造函数。用的是
// HandModel.hpp 写死的占位 fingerParam()/handBackMarkers()——保留这个
// 版本是因为它零额外依赖(不需要 HandTemplateStore/Qt)，适合纯数学单测
// 场景(比如这个项目目前所有 HandStateIEKF 的单测)。真机接入请用下面的
// makeHandForwardKinematicsFromTemplate，用标定出来的真实参数。
inline ForwardKinematicsFn makeHandForwardKinematics() {
    return [](const std::vector<double>& angles) -> std::vector<Vec3> {
        assert(angles.size() == size_t(kHandNumJoints) &&
              "HandStateIEKF 的关节角向量长度必须跟 HandModel.hpp 的 16 维布局一致");
        std::array<double,16> q{};
        for (int i=0;i<kHandNumJoints;++i) q[size_t(i)] = angles[size_t(i)];

        std::array<std::array<double,3>,20> out{};
        handFK(q, out);

        std::vector<Vec3> result(size_t{kHandNumMarkers});
        for (int i=0;i<kHandNumMarkers;++i)
            result[size_t(i)] = { out[size_t(i)][0], out[size_t(i)][1], out[size_t(i)][2] };
        return result;
    };
}

// 【真机接入用这个】用标定出来的手背模板 + 5根手指结构参数做 FK，而不是
// HandModel.hpp 写死的占位值——这是让 HandCalibration.hpp/
// HandSelfCalibration.hpp 的标定结果真正影响实时追踪精度的接线点(之前
// 标定结果哪儿都没接，这里补上)。backMarkers/fingerParams 通常来自
// HandTemplateStore::data()，跟 HandColdStart 用的应该是同一份，两边
// 不同步的话冷启动匹配出来的位姿和实时FK用的手型会对不上。
inline ForwardKinematicsFn makeHandForwardKinematicsFromTemplate(
        const std::array<HandVec3,5>& backMarkers,
        const std::array<FingerParam,5>& fingerParams) {
    return [backMarkers, fingerParams](const std::vector<double>& angles) -> std::vector<Vec3> {
        assert(angles.size() == size_t(kHandNumJoints) &&
              "HandStateIEKF 的关节角向量长度必须跟 HandModel.hpp 的 16 维布局一致");

        std::vector<Vec3> result(size_t{kHandNumMarkers});
        for (int i=0;i<5;++i) result[size_t(i)] = { backMarkers[size_t(i)][0], backMarkers[size_t(i)][1], backMarkers[size_t(i)][2] };

        std::array<double,3> thumbOut[3];
        thumbFK(fingerParams[0], angles[0], angles[1], angles[2], angles[3], thumbOut);
        for (int k=0;k<3;++k) result[size_t(5+k)] = { thumbOut[k][0], thumbOut[k][1], thumbOut[k][2] };

        for (int f=0; f<4; ++f) {
            const int base = 4 + f*3;
            std::array<double,3> fOut[3];
            fingerFK(fingerParams[size_t(1+f)], angles[size_t(base)], angles[size_t(base+1)], angles[size_t(base+2)], fOut);
            for (int k=0;k<3;++k) result[size_t(8+f*3+k)] = { fOut[k][0], fOut[k][1], fOut[k][2] };
        }
        return result;
    };
}

// 关节角限幅：接入 clampToLimits，供调用方在每次 IEKF 更新后夹一下，避免
// 解跑到解剖学不可能的姿势（HandStateIEKF 本身不知道 ROM 这回事，这一步
// 是调用方在拿到 state() 之后自己做的后处理，不影响滤波数学本身）。
inline void clampHandStateToJointLimits(HandPoseState& state) {
    assert(state.jointAngles.size() == size_t(kHandNumJoints));
    std::array<double,16> q{};
    for (int i=0;i<kHandNumJoints;++i) q[size_t(i)] = state.jointAngles[size_t(i)];
    const auto clamped = clampToLimits(q);
    for (int i=0;i<kHandNumJoints;++i) state.jointAngles[size_t(i)] = clamped[size_t(i)];
}

} // namespace mocap
