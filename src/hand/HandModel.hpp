#pragma once
// ---------------------------------------------------------------------------
// 手部运动学模型（正向运动学 FK）—— 纯数学，零 Qt/第三方依赖，可单测。
//
// 给定手指关节角（16 维），算出全手 20 颗反光球在"手腕局部系"下的 3D 位置。
// 是整条手部动捕管线的地基（README §2、§6 第 1 步）。
//
// 本文件是 tools/handmocap/hand_model.py 参考实现的 C++ 移植——那份 Python
// 已用合成数据验证过全部性质（指骨刚性、屈曲方向、DIP 耦合、外展方向、输出
// 契约），这里逐行对应翻译，只是语言转换，不引入新的算法风险。移植正确性
// 由 test_hand_model.cpp 用与 Python 相同的断言独立复核。
//
// 坐标约定（与 Python 版严格一致）
//   手腕局部系：原点在腕部参考点，+X 指手指方向（远端），+Y 指拇指侧（桡侧），
//   +Z 指手背外侧（背侧）。右手系。长度单位 mm，角度单位 弧度。
//   屈曲绕 +Y 轴（手指向掌心 -Z 卷）；外展绕 +Z 轴。
//
// 关节角向量布局（16 维，顺序固定，与 Python 版一致）
//   [0]拇指CMC屈 [1]拇指CMC展 [2]拇指MCP屈 [3]拇指IP屈
//   [4]食MCP屈 [5]食MCP展 [6]食PIP屈
//   [7]中MCP屈 [8]中MCP展 [9]中PIP屈
//   [10]无MCP屈 [11]无MCP展 [12]无PIP屈
//   [13]小MCP屈 [14]小MCP展 [15]小PIP屈
//
// 输出顺序（20×3，顺序固定，与 Python 版一致）
//   [0:5]手背5球 [5:8]拇指(掌/近/远) [8:11]食(近/中/远) [11:14]中
//   [14:17]无名 [17:20]小指
// ---------------------------------------------------------------------------
#include <array>
#include <cmath>

namespace mocap {

// 4x4 齐次变换，行主序。用最小实现，不引入矩阵库。
struct Mat4 {
    std::array<double, 16> m{};   // 行主序：m[row*4+col]

    static Mat4 identity() {
        Mat4 r;
        r.m = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
        return r;
    }
    // 矩阵乘 this * o
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double s = 0;
                for (int k = 0; k < 4; ++k) s += m[i*4+k] * o.m[k*4+j];
                r.m[i*4+j] = s;
            }
        return r;
    }
    // 取平移列（变换后的原点位置）
    std::array<double,3> translation() const { return { m[3], m[7], m[11] }; }
    // 变换一个点 (x,y,0..)：返回 this * [x,y,z,1] 的前三维
    std::array<double,3> apply(double x, double y, double z) const {
        return {
            m[0]*x + m[1]*y + m[2]*z + m[3],
            m[4]*x + m[5]*y + m[6]*z + m[7],
            m[8]*x + m[9]*y + m[10]*z + m[11]
        };
    }
};

inline Mat4 rotX(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat4 r = Mat4::identity();
    r.m[5]=c; r.m[6]=-s; r.m[9]=s; r.m[10]=c; return r;
}
inline Mat4 rotY(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat4 r = Mat4::identity();
    r.m[0]=c; r.m[2]=s; r.m[8]=-s; r.m[10]=c; return r;
}
inline Mat4 rotZ(double a) {
    double c = std::cos(a), s = std::sin(a);
    Mat4 r = Mat4::identity();
    r.m[0]=c; r.m[1]=-s; r.m[4]=s; r.m[5]=c; return r;
}
inline Mat4 transMat(double x, double y, double z) {
    Mat4 r = Mat4::identity();
    r.m[3]=x; r.m[7]=y; r.m[11]=z; return r;
}

constexpr double kDipCoupling = 0.7;   // DIP = 0.7 * PIP（README §2.1）——现在只作为"未标定时"的默认值

// 手指尺寸参数（占位人体测量学值，mm）。C++ 侧固定为常量表，实测后替换。
struct FingerParam {
    std::array<double,3> anchor;    // MCP/CMC 关节在手腕系的位置
    std::array<double,3> lengths;   // 三节指骨长
    // DIP = dipCoupling * PIP 的耦合系数。之前是全局写死的 kDipCoupling
    // 常量，每个人/每根手指的真实耦合比例其实不完全一致(差10%~20%很常见)，
    // 之前不管标定数据多好、多干净都改不动这部分系统性误差——因为模型
    // 根本没给它留自由度。现在变成可标定量，默认值沿用旧常量，没标定过
    // 的手指(占位表/新建FingerParam时)行为不变。带默认成员初始值，旧代码
    // 里 {{...},{...}} 这种只给anchor/lengths两个字段的聚合初始化继续合法
    // (C++14起聚合初始化允许省略带默认值的尾部成员)。
    double dipCoupling = kDipCoupling;
};

// 与 Python HAND_PARAMS 完全一致
inline const FingerParam& fingerParam(int idx) {
    // 0=thumb 1=index 2=middle 3=ring 4=pinky
    static const FingerParam params[5] = {
        {{30, 40, 5}, {40, 30, 25}},   // thumb (掌骨/近/远)
        {{85, 20, 0}, {40, 25, 20}},   // index
        {{88,  3, 0}, {45, 27, 22}},   // middle
        {{85,-14, 0}, {42, 26, 21}},   // ring
        {{80,-30, 0}, {33, 20, 17}},   // pinky
    };
    return params[idx];
}

// 手背 5 球（手腕系），与 Python HAND_BACK_MARKERS 一致
inline const std::array<std::array<double,3>,5>& handBackMarkers() {
    static const std::array<std::array<double,3>,5> b = {{
        {15,  0, 2}, {55, 22, 5}, {50,-25, 8}, {35, 30, 6}, {60, 3, 3}
    }};
    return b;
}

// 四指 FK：写进 out[0..2]（近/中/远节球，手腕系）
inline void fingerFK(const FingerParam& p, double mcpFlex, double mcpAbduct,
                     double pipFlex, std::array<double,3>* out) {
    const double Lp = p.lengths[0], Lm = p.lengths[1], Ld = p.lengths[2];
    const double dipFlex = p.dipCoupling * pipFlex;

    Mat4 T = transMat(p.anchor[0], p.anchor[1], p.anchor[2]);
    T = T * rotZ(mcpAbduct) * rotY(mcpFlex);      // MCP：先外展再屈曲

    out[0] = (T * transMat(Lp/2, 0, 0)).translation();
    T = T * transMat(Lp, 0, 0);

    T = T * rotY(pipFlex);                          // PIP
    out[1] = (T * transMat(Lm/2, 0, 0)).translation();
    T = T * transMat(Lm, 0, 0);

    T = T * rotY(dipFlex);                          // DIP（耦合）
    out[2] = (T * transMat(Ld/2, 0, 0)).translation();
}

// 拇指 FK：写进 out[0..2]（掌骨/近/远节球）
inline void thumbFK(const FingerParam& p, double cmcFlex, double cmcAbduct,
                    double mcpFlex, double ipFlex, std::array<double,3>* out) {
    const double Lmeta = p.lengths[0], Lp = p.lengths[1], Ld = p.lengths[2];

    Mat4 T = transMat(p.anchor[0], p.anchor[1], p.anchor[2]) * rotZ(M_PI/4.0);
    T = T * rotZ(cmcAbduct) * rotY(cmcFlex);

    out[0] = (T * transMat(Lmeta/2, 0, 0)).translation();
    T = T * transMat(Lmeta, 0, 0);

    T = T * rotY(mcpFlex);
    out[1] = (T * transMat(Lp/2, 0, 0)).translation();
    T = T * transMat(Lp, 0, 0);

    T = T * rotY(ipFlex);
    out[2] = (T * transMat(Ld/2, 0, 0)).translation();
}

// 全手 FK：q 是 16 维关节角，out 是 20 个球（手腕系）。
inline void handFK(const std::array<double,16>& q,
                   std::array<std::array<double,3>,20>& out) {
    // 手背 5 球（刚体固定点）
    const auto& back = handBackMarkers();
    for (int i = 0; i < 5; ++i) out[i] = back[i];

    // 拇指
    thumbFK(fingerParam(0), q[0], q[1], q[2], q[3], &out[5]);

    // 四指：食(4..6) 中(7..9) 无(10..12) 小(13..15)
    for (int f = 0; f < 4; ++f) {
        const int base = 4 + f * 3;
        fingerFK(fingerParam(1 + f), q[base], q[base+1], q[base+2], &out[8 + f*3]);
    }
}

// ---------------------------------------------------------------------------
// 关节活动范围（ROM）约束——移植自 hand_model.py 的 JOINT_LIMITS/clamp_to_limits。
// 用于状态估计（阶段二/三，IK/滤波）时把解限制在解剖学合理范围内，避免优化器
// 收敛到"数学上满足观测、但人手根本做不到"的关节角。跟 Python 版数值/顺序
// 严格一致（16 维，跟 q 向量布局同一顺序，见文件顶部注释）。
// ---------------------------------------------------------------------------
struct JointLimit { double lo, hi; };

inline const std::array<JointLimit, 16>& jointLimits() {
    static const std::array<JointLimit, 16> lim = {{
        // 【跟训练 rig 的 pose_prior.py::LIMITS_LO/HI 对齐】
        // 原来这张表比 rig 还窄，后果是【模型能生成的姿势，输出端表示不出来】：
        //   · 四指 MCP展 原来 ±0.30(±17.2°)，而 rig 是 食±0.44 / 小±0.48
        //     —— 真机实测小指外展 30.9° 被硬钳到 17.2°，张开手的扇形被压平
        //   · 所有屈曲维 lo=0，而 rig 允许 -0.35 —— 手指自然反张全部钳成 0，
        //     摊平手时一半的维显示"触限"，行程条空着
        // 解算出来的值被一张比模型自己还保守的表削掉，这是凭空制造的不可用。
        {-0.35, 1.00}, {-0.17, 1.15}, {-0.17, 0.95}, {-0.26, 1.40},  // 拇 CMC屈/展, MCP, IP
        {-0.35, 1.60}, {-0.44, 0.44}, {0.00, 1.92},                  // 食 MCP屈/展, PIP
        {-0.30, 1.60}, {-0.28, 0.28}, {0.00, 1.92},                  // 中
        {-0.30, 1.60}, {-0.28, 0.28}, {0.00, 1.92},                  // 无名
        {-0.35, 1.62}, {-0.48, 0.48}, {0.00, 1.92},                  // 小
    }};
    return lim;
}

// 把关节角逐分量夹到 jointLimits() 范围内。不用 <algorithm> 是刻意的——保持
// 这个头文件"纯 std 基础类型，零额外 include"的最小依赖原则不变。
inline std::array<double, 16> clampToLimits(const std::array<double, 16>& q) {
    const auto& lim = jointLimits();
    std::array<double, 16> out{};
    for (int i = 0; i < 16; ++i) {
        double v = q[i];
        if (v < lim[i].lo) v = lim[i].lo;
        if (v > lim[i].hi) v = lim[i].hi;
        out[i] = v;
    }
    return out;
}

} // namespace mocap
