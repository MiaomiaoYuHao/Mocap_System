#pragma once
// ---------------------------------------------------------------------------
// 手背模板自动标定 —— 纯数学，零 Qt 依赖，可单测。
//
// 回答"我怎么贴，只要符合理论要求就能被识别"这个问题：手背5点模板本质上
// 只是"这5颗球在手腕局部系下的相对位置"，`HandModel.hpp` 里 handBackMarkers()
// 现在写死的那5个坐标只是示意值——只要你实际贴的位置满足"任意3颗不共线"
// (HandPose.hpp::checkHandBackTemplateGeometry 检查的那个性质)，这个模板
// 完全可以从"你实际贴的位置"自动算出来，不需要你贴成跟占位坐标一样的形状。
//
// 流程：让手保持静止，5颗手背marker都被至少2台相机看到，三角化出它们的
// 世界系坐标；再给两个"方向提示"：
//   1. forwardHintWorldPoint —— 手指方向上的一个参考点(比如中指尖此刻的
//      世界系位置，或者贴一颗临时marker在手指延伸方向上，标定完就可以
//      摘掉，它不进最终模板)。
//   2. dorsalHintDirectionWorld —— 手背朝向的世界系方向(比如标定时让手背
//      朝上，这个方向就是世界系 +Z；这个方向不需要精确，只用来消解"手背
//      法线"的符号二义性，随便给个大致朝向就行)。
// 有了这两个提示 + 5个点本身的空间分布，就能唯一定出手腕局部系的3根轴，
// 把5个世界系坐标转换成局部系坐标——这就是新的模板，直接替换
// handBackMarkers() 里那5行占位数字。
//
// 【明确的数学限制，没有回避】这个自动标定只覆盖手背5点模板。16个关节
// 长度参数(fingerParam 的 anchor/lengths)做不到同样的全自动——原因见文件
// 底部注释，不是没做，是信息量不够，秤不出来。
// ---------------------------------------------------------------------------
#include "hand/HandPose.hpp"   // HandVec3/HandMat3、checkHandBackTemplateGeometry
#include <array>
#include <cmath>

namespace mocap {

struct HandBackCalibrationResult {
    std::array<HandVec3, 5> templateLocal{};   // 直接替换 handBackMarkers() 的返回值
    HandVec3 originWorld{};                     // 标定时刻手腕原点的世界系位置(供参考/调试)
    HandMat3 rotWorldToLocal{};                 // 供调试/复核：世界系 -> 局部系 的旋转
    TemplateGeometryCheck geometryCheck{};
    bool valid = false;
    const char* failReason = "";
};

namespace handcalib_detail {

inline HandVec3 sub(const HandVec3& a, const HandVec3& b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
inline HandVec3 add(const HandVec3& a, const HandVec3& b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
inline HandVec3 scale(const HandVec3& a, double s) { return {a[0]*s, a[1]*s, a[2]*s}; }
inline double dot(const HandVec3& a, const HandVec3& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline HandVec3 cross(const HandVec3& a, const HandVec3& b) {
    return { a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0] };
}
inline double norm(const HandVec3& a) { return std::sqrt(dot(a,a)); }
inline HandVec3 normalize(const HandVec3& a) {
    const double n = norm(a);
    return n > 1e-9 ? scale(a, 1.0/n) : HandVec3{1,0,0};
}

// 5个点的最佳拟合平面法向——3x3 协方差矩阵最小特征值对应的特征向量。
// 独立实现一份 3x3 对称特征分解(不复用 HandPose.hpp::detail 里那份)，
// 跟项目里"每个模块保持零依赖、各自独立实现同一类小工具"是同一个先例
// (MultiViewCluster/HandPose 各自独立实现 Jacobi 特征分解就是这么处理的)。
inline HandVec3 planeNormalViaPCA(const std::array<HandVec3,5>& pts, const HandVec3& centroid) {
    double cov[3][3] = {{0}};
    for (const auto& p : pts) {
        const HandVec3 d = sub(p, centroid);
        for (int i=0;i<3;++i) for (int j=0;j<3;++j) cov[i][j] += d[i]*d[j];
    }
    double A[3][3]; for(int i=0;i<3;++i) for(int j=0;j<3;++j) A[i][j]=cov[i][j];
    double V[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (int sweep=0; sweep<50; ++sweep) {
        int p=0,q=1; double mx=std::abs(A[0][1]);
        if (std::abs(A[0][2])>mx){mx=std::abs(A[0][2]);p=0;q=2;}
        if (std::abs(A[1][2])>mx){mx=std::abs(A[1][2]);p=1;q=2;}
        if (mx<1e-18) break;
        const double app=A[p][p],aqq=A[q][q],apq=A[p][q];
        const double phi=0.5*std::atan2(2*apq, aqq-app);
        const double c=std::cos(phi), s=std::sin(phi);
        for (int i=0;i<3;++i){ double aip=A[i][p],aiq=A[i][q]; A[i][p]=c*aip-s*aiq; A[i][q]=s*aip+c*aiq; }
        for (int i=0;i<3;++i){ double api=A[p][i],aqi=A[q][i]; A[p][i]=c*api-s*aqi; A[q][i]=s*api+c*aqi; }
        for (int i=0;i<3;++i){ double vip=V[i][p],viq=V[i][q]; V[i][p]=c*vip-s*viq; V[i][q]=s*vip+c*viq; }
    }
    int minIdx=0; double minVal=A[0][0];
    for (int i=1;i<3;++i) if (A[i][i]<minVal){minVal=A[i][i];minIdx=i;}
    return { V[0][minIdx], V[1][minIdx], V[2][minIdx] };
}

} // namespace handcalib_detail

// 主入口。worldPoints 顺序无所谓(不需要跟 handBackMarkers() 的顺序对应，
// 那本来就是要被这个函数重新定义的东西)——但要保证同一次调用里，后续
// 每一帧检测到的这5颗球要能按某种方式(比如 HandColdStart 的距离匹配)
// 稳定映射回这里用的顺序，这是调用方(标定向导UI)自己要处理的事，不是这
// 个函数的职责。
inline HandBackCalibrationResult calibrateHandBackTemplate(
        const std::array<HandVec3,5>& worldPoints,
        const HandVec3& forwardHintWorldPoint,
        const HandVec3& dorsalHintDirectionWorld,
        double minTriangleAreaMm2 = 100.0) {
    using namespace handcalib_detail;
    HandBackCalibrationResult out;

    HandVec3 centroid{0,0,0};
    for (const auto& p : worldPoints) centroid = add(centroid, p);
    centroid = scale(centroid, 1.0/5.0);

    // 局部 +Z：手背法向，用平面拟合定方向，再用 dorsalHint 消解符号二义性
    // (法向本身正反都满足"垂直于点云"，必须靠外部提示选一个)。
    HandVec3 zAxis = planeNormalViaPCA(worldPoints, centroid);
    if (dot(zAxis, dorsalHintDirectionWorld) < 0) zAxis = scale(zAxis, -1.0);
    zAxis = normalize(zAxis);

    // 局部 +X：手指方向提示投影到手背平面内(去掉法向分量，保证跟Z正交)。
    HandVec3 xRaw = sub(forwardHintWorldPoint, centroid);
    const double xDotZ = dot(xRaw, zAxis);
    HandVec3 xAxis = sub(xRaw, scale(zAxis, xDotZ));
    if (norm(xAxis) < 1e-6) {
        out.valid = false; out.failReason = "forward hint point too close to being parallel with the dorsal normal, cannot determine +X";
        return out;
    }
    xAxis = normalize(xAxis);

    // 局部 +Y = Z x X，凑成右手系(近似"桡侧"方向；具体是不是真的指向
    // 拇指侧取决于 dorsalHint 给的朝向约定，标定向导UI应该在这一步之后
    // 提示用户确认拇指是不是在 +Y 那一侧，不对就说明 dorsalHint 给反了)。
    const HandVec3 yAxis = cross(zAxis, xAxis);

    // R_world_to_local：行 = 各局部轴在世界系下的分量，R*(world-origin) = local。
    out.rotWorldToLocal = {
        xAxis[0], xAxis[1], xAxis[2],
        yAxis[0], yAxis[1], yAxis[2],
        zAxis[0], zAxis[1], zAxis[2],
    };
    out.originWorld = centroid;

    for (int i=0;i<5;++i) {
        const HandVec3 d = sub(worldPoints[size_t(i)], centroid);
        out.templateLocal[size_t(i)] = {
            dot(d, xAxis), dot(d, yAxis), dot(d, zAxis)
        };
    }

    out.geometryCheck = checkHandBackTemplateGeometry(out.templateLocal, minTriangleAreaMm2);
    out.valid = out.geometryCheck.ok;
    if (!out.valid) out.failReason = "derived template fails the non-degeneracy check (some 3 points nearly collinear) -- pick a different placement, don't stick all 5 in (near) a straight line";
    return out;
}

} // namespace mocap

// ---------------------------------------------------------------------------
// 【为什么关节长度做不到同样的全自动】
//
// fingerParam() 的 anchor(MCP/CMC关节在手腕系的位置) + lengths(三节指骨长)
// 这两组参数，跟手背模板不是同一类问题——手背5点本身就是要标定的东西
// (marker位置)，关节参数描述的是"骨骼结构"，而骨骼上的关节本身没有贴
// marker(marker贴在每节指骨中点，不是关节上，这是 handFK 的既有约定，
// 见 fingerFK 里 out[k] = 各节中点)。
//
// 假设某根手指伸直(q=0)，三颗marker在局部系里落在同一条直线上，分别是:
//   p0 = anchor + (Lp/2, 0, 0)
//   p1 = anchor + (Lp + Lm/2, 0, 0)
//   p2 = anchor + (Lp + Lm + Ld/2, 0, 0)
// 从三角化出的 p0,p1,p2 只能解出:
//   |p1-p0| = (Lp+Lm)/2
//   |p2-p1| = (Lm+Ld)/2
// 两个方程、三个未知数(Lp,Lm,Ld)，anchor 本身也解不出来(它在p0往回
// Lp/2那么远，Lp本身就是未知数)——三个marker共线，天然缺一个约束，不是
// 算法不够聪明，是这三个点提供的信息量不够反解四个量(anchor + 3节长度)。
//
// 两个可行的解决办法，任选其一，但都需要额外输入，不是纯自动:
//   1. 老办法：拿卡尺分别量三节指骨长度，手动填 fingerParam()。
//   2. 标定时临时加marker：在MCP/PIP/DIP关节本身贴一颗临时marker(标定完
//      就摘掉)，多出来的点能唯一定出anchor和各节长度，之后就能完全自动。
// 这个文件目前只做手背模板(信息量够、能全自动)，关节参数这块如果你想要
// 全自动，需要采用第2种"临时关节marker"的标定协议，这个我可以另外实现，
// 但要先跟你确认贴临时marker这个流程你能接受再动手，不是可以凭空变出来。
// ---------------------------------------------------------------------------
