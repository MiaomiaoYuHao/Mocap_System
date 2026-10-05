#pragma once
// ---------------------------------------------------------------------------
// 精度验证 · 标定杆统计 —— 纯数学，零 Qt 依赖，可单测。
//
// 拿一根量好的固定长度标定杆(两端各一颗反光球，真值是两颗球球心之间的
// 距离——不是杆子物理长度、不是球表面到表面，见对话里的完整说明)，在
// 捕捉空间里到处挥动，每一帧量出两颗球三角化后的实际距离，喂给这个模块，
// 滚动统计出这套系统的真实精度数字：RMS误差、标准差、最大误差、偏置
// (systematic bias，测量均值减真值——不为零说明系统性偏大/偏小，不只是
// 随机噪声)。
//
// 这是这个项目第一次有"拿真值量出一个具体精度数字"的地方——之前所有
// "感觉更稳了"都是主观判断，没有可以摆出来对比的数字。这个模块只负责
// 统计计算这一层，不管检测/三角化怎么来的两个点位置，调用方(UI层)负责
// 每帧把两个三角化出来的3D点位置喂进来。
// ---------------------------------------------------------------------------
#include <array>
#include <vector>
#include <cmath>
#include <deque>
#include <cstdint>

namespace mocap {

using WandVec3 = std::array<double, 3>;

struct WandSample {
    double measuredLengthMm = 0.0;   // 这一帧量出的两球距离
    double errorMm = 0.0;             // measuredLengthMm - trueLengthMm(带符号，正=量大了，负=量小了)
    int64_t ts_ns = 0;
};

struct WandPrecisionReport {
    int sampleCount = 0;
    double trueLengthMm = 0.0;

    double meanMeasuredMm = 0.0;
    double biasMm = 0.0;          // meanMeasured - trueLength，带符号系统性偏置
    double stdDevMm = 0.0;        // 测量值本身的标准差(随机噪声水平，去掉了系统性偏置的影响)
    double rmsErrorMm = 0.0;      // sqrt(mean((measured-true)^2))，同时包含偏置和随机噪声——这是最终该看的"精度"总分
    double maxAbsErrorMm = 0.0;
    int64_t maxAbsErrorTs = 0;    // 最大误差出现在哪一帧(供回放定位是不是在某个特定区域精度差)

    bool valid = false;
};

class WandPrecisionAccumulator {
public:
    explicit WandPrecisionAccumulator(double trueLengthMm) : trueLengthMm_(trueLengthMm) {}

    // 喂入这一帧两颗球的世界系3D位置，内部算距离、跟真值比较、记录。
    // 距离小于一个明显不合理的下限(比如<1mm，说明两个点被误判成同一个
    // 点/三角化出了问题)时诚实丢弃这个样本，不计入统计——一个荒谬的
    // 异常值会把RMS带偏得完全失去参考意义。
    void addFrame(const WandVec3& ballA, const WandVec3& ballB, int64_t ts_ns) {
        const double dx = ballA[0]-ballB[0], dy = ballA[1]-ballB[1], dz = ballA[2]-ballB[2];
        const double dist = std::sqrt(dx*dx+dy*dy+dz*dz);
        if (dist < 1.0) return;   // 明显异常(两点几乎重合)，不计入统计

        WandSample s;
        s.measuredLengthMm = dist;
        s.errorMm = dist - trueLengthMm_;
        s.ts_ns = ts_ns;
        samples_.push_back(s);
    }

    // 直接喂距离值的重载——如果调用方自己已经算好距离(比如复用已有的
    // 三角化+配对逻辑，只想要这一层的统计)，不需要强迫走3D点这条路径。
    void addDistanceSample(double measuredLengthMm, int64_t ts_ns) {
        if (measuredLengthMm < 1.0) return;
        WandSample s;
        s.measuredLengthMm = measuredLengthMm;
        s.errorMm = measuredLengthMm - trueLengthMm_;
        s.ts_ns = ts_ns;
        samples_.push_back(s);
    }

    void reset() { samples_.clear(); }

    int sampleCount() const { return int(samples_.size()); }

    WandPrecisionReport report() const {
        WandPrecisionReport out;
        out.trueLengthMm = trueLengthMm_;
        out.sampleCount = int(samples_.size());
        if (samples_.empty()) return out;

        double sumMeasured = 0.0;
        for (const auto& s : samples_) sumMeasured += s.measuredLengthMm;
        out.meanMeasuredMm = sumMeasured / double(samples_.size());
        out.biasMm = out.meanMeasuredMm - trueLengthMm_;

        double sumSqDevFromMean = 0.0, sumSqErr = 0.0;
        double maxAbsErr = -1.0; int64_t maxAbsErrTs = 0;
        for (const auto& s : samples_) {
            const double devFromMean = s.measuredLengthMm - out.meanMeasuredMm;
            sumSqDevFromMean += devFromMean*devFromMean;
            sumSqErr += s.errorMm*s.errorMm;
            const double absErr = std::abs(s.errorMm);
            if (absErr > maxAbsErr) { maxAbsErr = absErr; maxAbsErrTs = s.ts_ns; }
        }
        // 标准差用 n-1(样本标准差)，单个样本时没有意义，特判成0而不是除0。
        out.stdDevMm = samples_.size() > 1 ? std::sqrt(sumSqDevFromMean / double(samples_.size()-1)) : 0.0;
        out.rmsErrorMm = std::sqrt(sumSqErr / double(samples_.size()));
        out.maxAbsErrorMm = maxAbsErr;
        out.maxAbsErrorTs = maxAbsErrTs;
        out.valid = true;
        return out;
    }

    const std::deque<WandSample>& samples() const { return samples_; }

private:
    double trueLengthMm_;
    std::deque<WandSample> samples_;
};

} // namespace mocap
