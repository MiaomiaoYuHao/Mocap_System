#pragma once
// ---------------------------------------------------------------------------
// 状态估计层 · Marker 数据关联 —— 纯数学，零 Qt 依赖，可单测。
//
// 这是 HandStateIEKF.hpp 明确留出的那块缺口:"这一帧某台相机检测到的某个
// 2D 观测，到底对应第几号 marker?" HandStateIEKF 假设调用方已经解决了这个
// 问题(每条 HandCameraMeasurement 带 markerIndex)，这个文件就是解决它的。
//
// 核心思路(跟 TemporalTracker.hpp 一脉相承，但作用在投影平面而不是3D):
//   用当前(上一帧滤波后的)手部状态 + FK，把所有 marker 投影到每台相机，
//   得到每个 marker 在每台相机里的"期望像素/归一化位置";然后把这一帧
//   每台相机实际检测到的观测，按"离哪个期望位置最近且在门控范围内"贪心
//   认领。认领上的观测就带上那个 markerIndex 喂给 HandStateIEKF;没认领上
//   的观测留作 unassigned(可能是噪声、也可能是别的东西闯进来)。
//
// 【这一版修的真实缺口】"门控范围"之前是一个跟状态不确定度完全无关的
// 固定欧氏距离半径(gateRadiusNorm)——冷启动刚完成、快速运动之后这些
// "预测本来就该更不确定"的时刻，固定半径要么太紧(漏配，表现成追踪
// 断续)要么太松(错配，表现成手指姿态乱跳)。现在改成自适应的马氏距离
// 门控：用IEKF自己维护的状态协方差(HandStateIEKF::covariance())，通过
// 投影雅可比把它传播到每个marker在每台相机的2D归一化坐标上，算出"这个
// marker预测位置本身有多不确定"的一个2×2协方差，再加上这一帧具体检测
// 自己的观测噪声(det.sigma)，两者相加就是这次匹配"预期残差应该有多大"
// 的完整协方差——用马氏距离(而不是欧氏距离)在这个协方差意义下判断"这个
// 检测跟这个marker够不够近"，天然做到了"越不确定的时候门控越宽松、越
// 确定的时候门控越收紧"，不需要调用方手动猜一个固定半径该给多大。
//
// 保留了原来的固定欧氏距离(gateRadiusNorm)作为一道硬性兜底上限——马氏
// 距离在协方差异常(比如刚冷启动、协方差还很大)时可能把门控允许的范围
// 撑得不合理地大，固定半径确保不管协方差算出来多离谱，候选对的实际像素
// 距离始终不会超过这个绝对上限，双保险。
//
// 为什么贪心够用(而不是每台相机内部做匈牙利全局指派):同一台相机里
// 20 颗 marker 的期望投影通常分得很开(手不会缩成一个点)，门控设得比
// marker 间距小就几乎不会撞;真正容易混的是"两颗 marker 投影到同一台相机
// 恰好靠得很近"，但那种情况本来就是 §3c 花生歧义的范畴，靠这一台相机也
// 分不清，得靠别的相机——所以这里贪心认领 + 冲突时让马氏距离更近的赢，
// 剩下那个留 unassigned 交给别的相机的观测去覆盖，是恰当的。
//
// 冷启动(第一帧还没有可用的先验状态)不在这个文件里做:那需要"手背5点无
// 标签 -> 用 HandPose.hpp 的 Kabsch 穷举/匹配出刚体位姿"，依赖 HandPose.hpp
// 的实际接口。这里跟 HandStateIEKF 处理 FK 的方式一样，把冷启动也做成一个
// 可注入的回调(ColdStartFn)——没有先验状态时调用它拿一个初始 HandPoseState，
// 有先验状态时直接用先验预测关联。这样冷启动的具体实现(Kabsch 模板匹配)
// 可以独立替换/测试，不绑死在关联逻辑里。
// ---------------------------------------------------------------------------
#include "estimate/HandStateIEKF.hpp"
#include <vector>
#include <array>
#include <functional>
#include <cmath>
#include <optional>
#include <algorithm>

namespace mocap {

// 一台相机这一帧的原始检测(还没打 marker 标签)。来自 DetectionOutput.hpp
// 的每个 usable 观测，归一化相机坐标 + 协方差。
struct RawDetection {
    double nx = 0.0, ny = 0.0;
    Cov2 sigma{};
};

// 每台相机一帧的检测集合。camIndex 指向调用方维护的相机数组。
struct CameraFrame {
    int camIndex = -1;
    const CamPose* cam = nullptr;
    std::vector<RawDetection> detections;
};

struct AssociationResult {
    std::vector<HandCameraMeasurement> measurements;   // 打好 markerIndex 标签、可直接喂 HandStateIEKF
    // 每台相机里没被任何 marker 认领的检测下标(供上层判断是否有干扰点/漏检)。
    std::vector<std::pair<int,int>> unassigned;        // (camIndex 在输入数组中的下标, detection 下标)
};

// 冷启动回调：给定这一帧所有相机的检测，返回一个初始手部状态估计。没有可用
// 先验时(第一帧、或长时间完全丢失后)调用。返回 nullopt 表示这一帧还凑不出
// 可信的初始位姿(比如可见 marker 太少)，调用方应跳过这一帧。
using ColdStartFn = std::function<std::optional<HandPoseState>(const std::vector<CameraFrame>&)>;

namespace markerassoc_detail {

using Mat = std::vector<std::vector<double>>;

// 对当前状态，数值求每个marker"世界坐标对误差状态"的3×n雅可比(n=6+
// numJoints，列布局跟 HandStateIEKF::update() 内部一致:
// [wristPos(3) | rotPerturb(3) | joints(numJoints)])——这一步不涉及任何
// 具体相机，跟相机投影无关的部分只用算一次，每台相机各自的投影雅可比
// (2×3)在调用方那边跟这个3×n矩阵相乘即可得到该marker在该相机下的2×n
// 雅可比，不需要为每台相机重新数值微分FK(FK数值微分是这里最贵的部分，
// 只算一次分摊到所有相机，而不是相机数×这个成本)。
inline std::vector<Mat> computeWorldPosJacobians(const HandPoseState& state,
        const ForwardKinematicsFn& fk, int numJoints) {
    using namespace handiekf_detail;
    const int n = 6 + numJoints;
    const auto localPos = fk(state.jointAngles);
    const int numMarkers = int(localPos.size());
    const size_t numMarkersSz = size_t(numMarkers);

    std::vector<std::vector<Vec3>> dLocalDJoint;   // [joint][marker]
    dLocalDJoint.resize(size_t(numJoints));
    const double h = 1e-5;
    for (int j=0;j<numJoints;++j) {
        auto anglesPlus = state.jointAngles, anglesMinus = state.jointAngles;
        anglesPlus[size_t(j)] += h; anglesMinus[size_t(j)] -= h;
        const auto lp = fk(anglesPlus), lm = fk(anglesMinus);
        std::vector<Vec3> d(numMarkersSz);
        for (int m=0;m<numMarkers;++m)
            d[size_t(m)] = { (lp[size_t(m)][0]-lm[size_t(m)][0])/(2*h),
                            (lp[size_t(m)][1]-lm[size_t(m)][1])/(2*h),
                            (lp[size_t(m)][2]-lm[size_t(m)][2])/(2*h) };
        dLocalDJoint[size_t(j)] = d;
    }

    std::vector<Mat> out(numMarkersSz, Mat(3, std::vector<double>(size_t(n), 0.0)));
    for (int m=0;m<numMarkers;++m) {
        const Vec3& lp3 = localPos[size_t(m)];
        const Vec3 v = matVec3Flat(state.wristRot, lp3);
        double skewV[3][3]; skew(v, skewV);

        for (int r=0;r<3;++r) out[size_t(m)][size_t(r)][size_t(r)] = 1.0;               // d(worldPos)/d(wristPos) = I
        for (int r=0;r<3;++r) for (int c=0;c<3;++c)
            out[size_t(m)][size_t(r)][size_t(3+c)] = -skewV[r][c];                      // d(worldPos)/d(rotPerturb) = -[v]_x
        for (int j=0;j<numJoints;++j) {
            const Vec3 rotated = matVec3Flat(state.wristRot, dLocalDJoint[size_t(j)][size_t(m)]);
            out[size_t(m)][0][size_t(6+j)] = rotated[0];
            out[size_t(m)][1][size_t(6+j)] = rotated[1];
            out[size_t(m)][2][size_t(6+j)] = rotated[2];
        }
    }
    return out;
}

// Hmc(2×n) = Hproj(2×3，只取前3列的位置部分) * worldJac(3×n)，再算
// Sigma2D(2×2) = Hmc * cov(n×n) * Hmc^T——marker在某台相机下、由状态不
// 确定度传播出来的2D投影协方差，不含这一帧具体检测自己的观测噪声(那部分
// 在真正跟某个检测配对、判断马氏距离时再加，因为不同候选检测的噪声不同)。
inline Cov2 propagateStateCovTo2D(const double Hproj[2][6], const Mat& worldJac, const Mat& cov, int n) {
    std::vector<double> Hmc0(size_t(n), 0.0), Hmc1(size_t(n), 0.0);
    for (int k=0;k<n;++k) {
        double s0=0.0, s1=0.0;
        for (int r=0;r<3;++r) { s0 += Hproj[0][r]*worldJac[size_t(r)][size_t(k)]; s1 += Hproj[1][r]*worldJac[size_t(r)][size_t(k)]; }
        Hmc0[size_t(k)] = s0; Hmc1[size_t(k)] = s1;
    }
    std::vector<double> PHt0(size_t(n),0.0), PHt1(size_t(n),0.0);
    for (int r=0;r<n;++r) {
        double s0=0.0, s1=0.0;
        for (int k=0;k<n;++k) { s0 += cov[size_t(r)][size_t(k)]*Hmc0[size_t(k)]; s1 += cov[size_t(r)][size_t(k)]*Hmc1[size_t(k)]; }
        PHt0[size_t(r)]=s0; PHt1[size_t(r)]=s1;
    }
    Cov2 out;
    double xx=0.0, xy=0.0, yy=0.0;
    for (int k=0;k<n;++k) { xx += Hmc0[size_t(k)]*PHt0[size_t(k)]; xy += Hmc0[size_t(k)]*PHt1[size_t(k)]; yy += Hmc1[size_t(k)]*PHt1[size_t(k)]; }
    out.xx=xx; out.xy=xy; out.yy=yy;
    return out;
}

} // namespace markerassoc_detail

class MarkerAssociator {
public:
    // fk: 跟 HandStateIEKF 用的是同一个 FK 回调(调用方传同一个即可)。
    // gateRadiusNorm: 硬性兜底的欧氏距离上限，归一化相机坐标单位——不管
    //   马氏距离算出来的门控多宽松，候选对的实际像素距离超过这个绝对值
    //   就直接拒绝，防止协方差异常(比如刚冷启动、初始协方差很大)时门控
    //   被撑得不合理地大。
    // chiSquareGate: 马氏距离平方的接受阈值。默认9.21对应二维卡方分布
    //   99%置信区间(标准的统计学取值，Vicon/OptiTrack这类专业系统的
    //   marker关联门控通常也是这个量级的选择，不是随便挑的数)——数值
    //   越大越宽松(更容易关联上，但也更容易错配)，越小越严格。
    MarkerAssociator(ForwardKinematicsFn fk, double gateRadiusNorm, double chiSquareGate = 9.21)
        : fk_(std::move(fk)), gateRadius_(gateRadiusNorm), chiSquareGate_(chiSquareGate) {}

    // 用当前状态(通常是上一帧 predict() 之后、update() 之前的先验状态)+
    // 该状态对应的协方差，对这一帧所有相机的检测做关联。cov 传空(size==0)
    // 时退化成只用固定欧氏半径门控(等价于旧行为，供冷启动首帧还没有
    // 意义明确的协方差、或者调用方不想用自适应门控时使用)。
    // 【兼容重载】不传协方差 = 只用固定欧氏半径门控，等价于加入马氏距离
    // 门控之前的旧行为。保留这个重载有两个理由：(1) 冷启动首帧还没有意义
    // 明确的协方差；(2) 调用方不想用自适应门控时应该有一条明确的路，而不是
    // 被迫构造一个假的协方差。语义就是上面 cov 注释里写的"传空退化"。
    AssociationResult associate(const HandPoseState& state,
                                const std::vector<CameraFrame>& frames) const {
        return associate(state, {}, frames);
    }

    AssociationResult associate(const HandPoseState& state,
                                const std::vector<std::vector<double>>& cov,
                                const std::vector<CameraFrame>& frames) const {
        using namespace markerassoc_detail;
        AssociationResult out;
        const auto localPos = fk_(state.jointAngles);
        const int numMarkers = int(localPos.size());
        const size_t numMarkersSz = size_t(numMarkers);
        const int numJoints = int(state.jointAngles.size());
        const int n = 6 + numJoints;
        const bool useMahalanobis = (int(cov.size()) == n);   // 协方差维度对不上就诚实退化，不强行用

        // 预计算每个 marker 的世界位置。
        std::vector<Vec3> worldPos(numMarkersSz);
        for (int m=0;m<numMarkers;++m) {
            const Vec3& lp = localPos[size_t(m)];
            Vec3 v = handiekf_detail::matVec3Flat(state.wristRot, lp);
            worldPos[size_t(m)] = { state.wristPos[0]+v[0], state.wristPos[1]+v[1], state.wristPos[2]+v[2] };
        }

        // 每个marker"世界坐标对误差状态"的雅可比——跟相机无关的部分只算
        // 一次，摊给下面所有相机用。cov维度对不上(不用马氏距离)时不需要
        // 算这个，省一遍FK数值微分。
        std::vector<Mat> worldJac;
        if (useMahalanobis) worldJac = computeWorldPosJacobians(state, fk_, numJoints);

        for (int fi=0; fi<int(frames.size()); ++fi) {
            const auto& frame = frames[size_t(fi)];
            if (!frame.cam) continue;

            // 每个 marker 在这台相机的期望归一化投影(投影失败=在相机后方的
            // 标成不可用)，以及(如果用马氏距离)该marker由状态不确定度带来
            // 的2D投影协方差。
            std::vector<std::array<double,2>> expected(numMarkersSz);
            std::vector<Cov2> stateSigma2D(numMarkersSz);
            std::vector<char> projOk(numMarkersSz, 0);
            for (int m=0;m<numMarkers;++m) {
                double px,py; double Hproj[2][6];
                if (!iekf_detail::projectAndJacobian(*frame.cam, worldPos[size_t(m)], px, py, Hproj)) continue;
                expected[size_t(m)] = {px,py}; projOk[size_t(m)] = 1;
                if (useMahalanobis) stateSigma2D[size_t(m)] = propagateStateCovTo2D(Hproj, worldJac[size_t(m)], cov, n);
            }

            // 候选对：先过硬性欧氏距离上限(兜底)，再(如果可用)过马氏距离
            // 检验；按马氏距离(可用时)或欧氏距离(退化时)从小到大排序贪心
            // 认领，marker 和 detection 各自只能被认领一次。
            struct Cand { double score; int marker; int det; };
            std::vector<Cand> cands;
            for (int m=0;m<numMarkers;++m) {
                if (!projOk[size_t(m)]) continue;
                for (int d=0; d<int(frame.detections.size()); ++d) {
                    const double dx = frame.detections[size_t(d)].nx - expected[size_t(m)][0];
                    const double dy = frame.detections[size_t(d)].ny - expected[size_t(m)][1];
                    const double euclidDist = std::sqrt(dx*dx+dy*dy);
                    if (euclidDist > gateRadius_) continue;   // 硬性上限，双保险的第一道

                    if (!useMahalanobis) {
                        cands.push_back({euclidDist, m, d});
                        continue;
                    }

                    const auto& detSigma = frame.detections[size_t(d)].sigma;
                    const double totalXX = stateSigma2D[size_t(m)].xx + detSigma.xx;
                    const double totalXY = stateSigma2D[size_t(m)].xy + detSigma.xy;
                    const double totalYY = stateSigma2D[size_t(m)].yy + detSigma.yy;
                    double inv[2][2];
                    if (!iekf_detail::invert2x2(totalXX, totalXY, totalXY, totalYY, inv)) continue;   // 协方差退化，跳过这个候选对
                    const double mdist2 = dx*dx*inv[0][0] + 2.0*dx*dy*inv[0][1] + dy*dy*inv[1][1];
                    if (mdist2 > chiSquareGate_) continue;   // 马氏距离检验，第二道

                    cands.push_back({mdist2, m, d});
                }
            }
            std::sort(cands.begin(), cands.end(), [](const Cand&a,const Cand&b){return a.score<b.score;});

            std::vector<char> markerTaken(numMarkersSz, 0);
            std::vector<char> detTaken(frame.detections.size(), 0);
            for (const auto& c : cands) {
                if (markerTaken[size_t(c.marker)] || detTaken[size_t(c.det)]) continue;
                markerTaken[size_t(c.marker)] = 1;
                detTaken[size_t(c.det)] = 1;
                const auto& det = frame.detections[size_t(c.det)];
                HandCameraMeasurement meas;
                meas.cam = frame.cam;
                meas.markerIndex = c.marker;
                meas.nx = det.nx; meas.ny = det.ny;
                meas.sigma = det.sigma;
                out.measurements.push_back(meas);
            }
            for (int d=0; d<int(frame.detections.size()); ++d)
                if (!detTaken[size_t(d)]) out.unassigned.push_back({fi, d});
        }
        return out;
    }

private:
    ForwardKinematicsFn fk_;
    double gateRadius_;
    double chiSquareGate_;
};

// ---------------------------------------------------------------------------
// 便捷的一体化驱动器：把"冷启动 or 用先验关联 -> 关联 -> IEKF 更新"这条
// 每帧流程串起来，是检测层输出到手部状态估计之间的完整闭环入口。上层每帧
// 拿到各相机的 usable 检测后，构造 CameraFrame 列表调 step() 即可，不用自己
// 管"现在到底该冷启动还是该用先验"这个状态机。
// ---------------------------------------------------------------------------
class HandTrackingPipeline {
public:
    HandTrackingPipeline(ForwardKinematicsFn fk, int numJoints, double gateRadiusNorm,
                        ColdStartFn coldStart, double chiSquareGate = 9.21)
        : fk_(fk), numJoints_(numJoints),
          associator_(fk, gateRadiusNorm, chiSquareGate), coldStart_(std::move(coldStart)) {}

    // 处理一帧。返回 true 表示这一帧成功更新了状态(可以从 state() 取结果)；
    // false 表示既没有可用先验、冷启动也没凑出初始位姿，这一帧被跳过。
    bool step(const std::vector<CameraFrame>& frames,
             double posProcessVar = 1.0, double rotProcessVar = 1e-4, double jointProcessVar = 1e-4) {
        if (!filter_) {
            auto init = coldStart_(frames);
            if (!init) return false;
            filter_.emplace(fk_, numJoints_, *init);
            // 冷启动这一帧也顺带做一次关联+更新，让初始位姿被观测精修一下。
        }

        filter_->predict(posProcessVar, rotProcessVar, jointProcessVar);
        // 用predict()之后(已经叠加过程噪声)、update()之前的协方差做关联
        // 门控——这正是"这一帧观测到来之前，我们对每个marker预测位置有多
        // 不确定"的协方差，用它做自适应门控语义上是对的。
        const auto assoc = associator_.associate(filter_->state(), filter_->covariance(), frames);
        lastUnassigned_ = assoc.unassigned;
        lastMeasurements_ = assoc.measurements;   // 供外部消费者(比如延迟精修流)复用同一份关联结果，不用重新关联一遍
        filter_->update(assoc.measurements);
        return true;
    }

    bool hasState() const { return filter_.has_value(); }
    const HandPoseState& state() const { return filter_->state(); }
    const std::vector<std::pair<int,int>>& lastUnassigned() const { return lastUnassigned_; }
    // 这一帧打好marker身份标签的观测——跟喂给实时IEKF::update()的是完全
    // 同一份，供其它消费者(比如FixedLagSmoother那条独立的延迟精修流)复用，
    // 不用重新跑一遍关联。
    const std::vector<HandCameraMeasurement>& lastMeasurements() const { return lastMeasurements_; }

    // 主动放弃当前状态(比如上层判断已经跟丢了)，下一帧重新冷启动。
    void reset() { filter_.reset(); }

    // 关节角限位夹紧的透传——见 HandStateIEKF::clampJointAngles 注释。
    // filter_还没建立(没有先验状态)时什么都不做，不算错误。
    void clampJointAngles(const std::function<void(std::vector<double>&)>& clampFn) {
        if (filter_) filter_->clampJointAngles(clampFn);
    }

private:
    ForwardKinematicsFn fk_;
    int numJoints_;
    MarkerAssociator associator_;
    ColdStartFn coldStart_;
    std::optional<HandStateIEKF> filter_;
    std::vector<std::pair<int,int>> lastUnassigned_;
    std::vector<HandCameraMeasurement> lastMeasurements_;
};

} // namespace mocap
