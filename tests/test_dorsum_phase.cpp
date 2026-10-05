// ===========================================================================
// test_dorsum_phase.cpp —— 量化证明：手背编号循环移位，Kabsch 残差【查不出来】
// ===========================================================================
// 这个测试不是验某个函数对不对，是验一条【判断】对不对：
//
//   "Validating 阶段用 dorsumRmseMm 中位数做自检，就能保证冻结的模板是对的"
//
// 结论是不能。手背五点接近正五边形，把模板的编号循环移一格，Kabsch 仍然能
// 拟出很小的残差（五边形转 72° 几乎还是原来那个五边形），但解出来的腕部姿态
// 整体转掉约 72°。残差看着没事，姿态全错 —— 这正是真机上"标定阶段正常、
// 一提交连线全错、而诊断数字全绿"的机制。
//
// 所以 Hm20AutoCalib 里的手背编号规范化默认关掉了（canonicalizeDorsum=false）：
// 它会重排模板，而 Kabsch 是按标签配对的，重排就等于制造这种移位。
//
// 编译运行：g++ -std=c++20 tests/test_dorsum_phase.cpp -o t && ./t
// ===========================================================================
#include <array>
#include <cmath>
#include <cstdio>
#include <algorithm>

using V = std::array<double, 3>;
using M = std::array<double, 9>;   // 行主序

static V sub(const V& a, const V& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
static double dot(const V& a, const V& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }

// 3x3 Kabsch：解 R 使 sum |R*p_i + t - q_i|^2 最小。用雅可比 SVD 的简化版
// （对称矩阵特征分解），5 点规模足够。
static void kabsch(const std::array<V,5>& P, const std::array<V,5>& Q, M& R, double& rmse) {
    V cp{0,0,0}, cq{0,0,0};
    for (int i=0;i<5;++i) for (int k=0;k<3;++k) { cp[k]+=P[i][k]/5.0; cq[k]+=Q[i][k]/5.0; }
    double H[3][3]{};
    for (int i=0;i<5;++i) {
        const V a = sub(P[i], cp), b = sub(Q[i], cq);
        for (int r=0;r<3;++r) for (int c=0;c<3;++c) H[r][c] += a[r]*b[c];
    }
    // 用 9 参数上的梯度下降求最优旋转（小规模、只为测试，不追求效率）
    double q[4]{1,0,0,0};
    auto quatToM = [](const double* s) {
        const double w=s[0],x=s[1],y=s[2],z=s[3];
        const double n=std::sqrt(w*w+x*x+y*y+z*z);
        const double W=w/n,X=x/n,Y=y/n,Z=z/n;
        return M{1-2*(Y*Y+Z*Z), 2*(X*Y-W*Z),   2*(X*Z+W*Y),
                 2*(X*Y+W*Z),   1-2*(X*X+Z*Z), 2*(Y*Z-W*X),
                 2*(X*Z-W*Y),   2*(Y*Z+W*X),   1-2*(X*X+Y*Y)};
    };
    auto score = [&](const double* s) {            // 最大化 trace(R^T H)
        const M Rm = quatToM(s);
        double t=0; for (int r=0;r<3;++r) for (int c=0;c<3;++c) t += Rm[r*3+c]*H[c][r];   // trace(R H)，不是 trace(R H^T)——写反会解出转置
        return t;
    };
    double step = 0.3;
    for (int it=0; it<4000; ++it) {
        const double base = score(q);
        bool improved=false;
        for (int k=0;k<4;++k) for (int sgn=-1; sgn<=1; sgn+=2) {
            double t[4]{q[0],q[1],q[2],q[3]}; t[k]+=sgn*step;
            if (score(t) > base) { std::copy(t,t+4,q); improved=true; goto next; }
        }
        next:
        if (!improved) { step *= 0.6; if (step < 1e-9) break; }
    }
    R = quatToM(q);
    double se=0;
    for (int i=0;i<5;++i) {
        const V a = sub(P[i], cp);
        const V pr{R[0]*a[0]+R[1]*a[1]+R[2]*a[2],
                   R[3]*a[0]+R[4]*a[1]+R[5]*a[2],
                   R[6]*a[0]+R[7]*a[1]+R[8]*a[2]};
        const V d = sub(pr, sub(Q[i], cq));
        se += dot(d,d);
    }
    rmse = std::sqrt(se/5.0);
}

static double angleDeg(const M& R) {          // 旋转矩阵的转角
    const double tr = R[0]+R[4]+R[8];
    return std::acos(std::clamp((tr-1.0)/2.0, -1.0, 1.0)) * 57.29577951;
}

int main() {
    // 手背 5 点：半径 ~28mm 的近正五边形，带一点不规则（真手不是正五边形）
    std::array<V,5> tmpl{};
    const double jitter[5] = {0.0, 2.5, -1.8, 1.2, -2.0};   // mm 级不规则
    for (int i=0;i<5;++i) {
        const double a = i * 2.0 * M_PI / 5.0;
        const double r = 28.0 + jitter[i];
        tmpl[size_t(i)] = V{r*std::cos(a), r*std::sin(a), 0.6*jitter[i]};
    }

    // 观测 = 模板原样（零噪声、姿态为单位阵）—— 最理想的情况
    std::array<V,5> obs = tmpl;

    M R0; double rmse0;
    kabsch(tmpl, obs, R0, rmse0);
    std::printf("== 编号正确 ==\n   残差 %.3f mm   解出转角 %.2f°\n", rmse0, angleDeg(R0));

    // Validating 阶段的阈值是 max(validateRmseMinMm=1.2, validateRmseK*sigma)。
    // sigma 是采集期手背点的抖动，真机上 1~2mm 量级，K 取 3 的话阈值约 3~6mm。
    // 这里取 5mm 当典型值。
    const double kThr = 5.0;
    std::printf("== 编号循环移位（模板被重排，观测标签没变）==\n");
    std::printf("   （自检阈值按典型值 %.1fmm 算）\n", kThr);
    int bad = 0, sneak = 0;
    for (int shift = 1; shift <= 4; ++shift) {
        std::array<V,5> shifted{};
        for (int i=0;i<5;++i) shifted[size_t(i)] = tmpl[size_t((i+shift)%5)];
        M R; double rmse;
        kabsch(shifted, obs, R, rmse);
        const double ang = angleDeg(R);
        std::printf("   移%d格: 残差 %5.2f mm   解出转角 %6.2f°   %s\n",
                    shift, rmse, ang,
                    rmse < kThr ? "← 自检放行，但姿态已错" : "(这一格残差够大，能被拦下)");
        if (ang < 30.0) { std::printf("     FAIL: 移位却没转角？\n"); ++bad; }
        if (rmse < kThr && ang > 30.0) ++sneak;
    }

    std::printf("\n结论：4 种移位里有 %d 种残差低于自检阈值 —— 残差是正常的毫米级，\n"
                "      而腕部姿态整体转掉了 72~144°。\n"
                "      只看 dorsumRmseMm 的自检【拦不住】这类错误，\n"
                "      所以 canonicalizeDorsum 默认关掉：不重排就不会制造这种移位。\n", sneak);
    if (sneak == 0) { std::printf("  注意：本次没有移位能骗过自检，结论需重新评估\n"); ++bad; }
    return bad;
}
