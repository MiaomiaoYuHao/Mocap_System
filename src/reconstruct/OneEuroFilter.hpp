#pragma once
// ---------------------------------------------------------------------------
// One Euro Filter（Casiez et al. 2012）——为"实时、低延迟、抗抖动"场景
// 设计的自适应低通滤波，鼠标/手势/动捕追踪的业界常用做法。
//
// 核心思想：普通低通滤波要在"平滑程度"和"延迟"之间二选一——截止频率调低
// 则平滑但拖尾明显，调高则跟手但抖动压不住。One Euro 的做法是让截止频率
// 随信号速度自适应：
//   - 信号变化慢时（比如球悬停/慢移），说明抖动主要是噪声，用低截止频率
//     大力平滑；
//   - 信号变化快时（球快速挥动），说明是真实运动，抬高截止频率、几乎不
//     平滑，把延迟降到最低，不拖尾。
// 参数只有三个，含义直观：
//   minCutoff：慢速时的基础截止频率（Hz）。越小越平滑（但慢速时越"黏"）。
//   beta：速度对截止频率的影响系数。越大则越快时越跟手（抗延迟），但快速
//         抖动的抑制也越弱。抖动主要来自快速运动就调小 beta，来自静止
//         噪声就调小 minCutoff。
//   dCutoff：对"速度估计"本身再做一次低通的截止频率（Hz），default 1.0
//         一般不用动，用来防止速度估计自己抖导致截止频率乱跳。
//
// 无 Qt / 无第三方依赖，纯 double 运算，可单测。每个标量分量一个滤波器
// 实例；3D 点用 OneEuroFilter3 包三个。时间用纳秒时间戳驱动（跟本项目
// 其它地方的 ts_ns 一致），内部换算成秒计算采样率——这样即使帧率抖动/
// 丢帧，滤波行为也正确，不依赖"固定帧率"假设。
// ---------------------------------------------------------------------------
#include <cmath>
#include <array>
#include <cstdint>

namespace mocap {

class OneEuroFilter {
public:
    OneEuroFilter(double minCutoff = 1.0, double beta = 0.0, double dCutoff = 1.0)
        : minCutoff_(minCutoff), beta_(beta), dCutoff_(dCutoff) {}

    void setParams(double minCutoff, double beta, double dCutoff = 1.0) {
        minCutoff_ = minCutoff; beta_ = beta; dCutoff_ = dCutoff;
    }

    void reset() { initialized_ = false; }

    // 传入一个新样本（value）及其时间戳（ns），返回滤波后的值。
    double filter(double value, int64_t ts_ns) {
        if (!initialized_) {
            initialized_ = true;
            lastTs_ = ts_ns;
            xPrev_ = value;
            dxPrev_ = 0.0;
            return value;
        }
        // 采样间隔（秒）。时间戳异常（相等或倒退）时退回一个安全的小 dt，
        // 避免除零/负频率——实时流里偶发的时间戳乱序不该让滤波器炸掉。
        double dt = double(ts_ns - lastTs_) * 1e-9;
        if (dt <= 0.0) dt = 1e-3;
        lastTs_ = ts_ns;

        // 1) 估计速度（对原始差分做一次 dCutoff 的低通，防止速度本身抖）。
        const double dx = (value - xPrev_) / dt;
        const double edx = alpha(dCutoff_, dt);
        const double dxHat = edx * dx + (1.0 - edx) * dxPrev_;
        dxPrev_ = dxHat;

        // 2) 根据速度自适应抬高截止频率：速度越大，cutoff 越高，越跟手。
        const double cutoff = minCutoff_ + beta_ * std::abs(dxHat);

        // 3) 用自适应 cutoff 对值本身做低通。
        const double ex = alpha(cutoff, dt);
        const double xHat = ex * value + (1.0 - ex) * xPrev_;
        xPrev_ = xHat;
        return xHat;
    }

private:
    // 一阶低通的平滑因子：把截止频率 fc 和采样间隔 dt 换算成 [0,1] 的
    // 混合系数。tau=1/(2π·fc) 是时间常数，alpha=1/(1+tau/dt)。
    static double alpha(double fc, double dt) {
        const double tau = 1.0 / (2.0 * M_PI * fc);
        return 1.0 / (1.0 + tau / dt);
    }

    double minCutoff_, beta_, dCutoff_;
    bool   initialized_ = false;
    int64_t lastTs_ = 0;
    double xPrev_ = 0.0, dxPrev_ = 0.0;
};

// 3D 点滤波：三个分量各一个独立的 One Euro 滤波器，共享同一组参数。
class OneEuroFilter3 {
public:
    OneEuroFilter3(double minCutoff = 1.0, double beta = 0.0, double dCutoff = 1.0)
        : fx_(minCutoff, beta, dCutoff), fy_(minCutoff, beta, dCutoff), fz_(minCutoff, beta, dCutoff) {}

    void setParams(double minCutoff, double beta, double dCutoff = 1.0) {
        fx_.setParams(minCutoff, beta, dCutoff);
        fy_.setParams(minCutoff, beta, dCutoff);
        fz_.setParams(minCutoff, beta, dCutoff);
    }

    void reset() { fx_.reset(); fy_.reset(); fz_.reset(); }

    std::array<double,3> filter(const std::array<double,3>& p, int64_t ts_ns) {
        return { fx_.filter(p[0], ts_ns), fy_.filter(p[1], ts_ns), fz_.filter(p[2], ts_ns) };
    }

private:
    OneEuroFilter fx_, fy_, fz_;
};

} // namespace mocap
