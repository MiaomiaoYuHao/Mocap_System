#pragma once
// ---------------------------------------------------------------------------
// 2D 质心检测：在灰度帧上找红外反光球的亮斑，输出亚像素质心。
// 算法：阈值二值化 -> 连通域(4邻域 flood fill) -> 面积过滤 -> 灰度加权质心。
// 纯逻辑、无 Qt 依赖，方便单元测试；输入是裸灰度 buffer(行优先, 每像素1字节)。
// 这是动捕流水线的最前端，检测结果既用于预览叠加，也用于 UDP 上抛给下游三角化。
// ---------------------------------------------------------------------------
#include <cstdint>
#include <vector>
#include <array>

namespace mocap {

    struct Blob {
        float cx = 0, cy = 0;   // 亚像素质心（像素坐标）
        int   area = 0;         // 过阈值像素数
        float peak = 0;         // 峰值亮度

        // 连通域外接框（整数像素）。flood fill 过程中本来就在算(圆度过滤用)，
        // 这里如实存下来，几乎零额外成本。下游可以用它做极廉价的"疑似粘连"
        // 预筛：单球外接框近似正方形(bw≈bh)，两球粘连的花生形会在连心线方向
        // 明显拉长(长宽比>1.3)。有了这个预筛，绝大多数单球 blob 不必进入昂贵
        // 的双圆拟合(detectBalls 单次 ~107us，密集摆位下无脑全跑会直接卡爆)。
        // bw<0 / bh<0 表示未填充(老的构造路径)，预筛时应视为"无法判断"→保守
        // 地当作可疑，避免漏拆。
        int   bboxMinX = 0, bboxMinY = 0, bboxW = -1, bboxH = -1;

        // 连通域边界像素（整数像素坐标，未去畸变/未归一化）。只有调用
        // detect() 时传 collectContours=true 才会填充；默认关闭，不影响
        // 现有调用方（预览叠加、UDP 输出只要质心）的性能。
        //
        // 供 detect/DetectionOutput.hpp 那条遮挡感知检测流水线(3a~3d)用：
        // 那边要的是"这个连通域的轮廓点"而不是质心，才能做已知半径约束
        // 拟合/半遮挡弧长门控/花生双圆分离。质心检测本身(这个类的主职责)
        // 不知道、也不需要知道半径先验或遮挡判断这些——它只负责把轮廓点
        // 如实吐出来，3a~3d 的数学在别处。
        std::vector<std::array<float,2>> contour;
    };

    struct DetectParams {
        int   threshold = 60;   // 亮度阈值（0-255）
        int   minArea = 3;    // 最小面积，滤噪点
        int   maxArea = 2000; // 最大面积，滤大片反光/漏光；<=0 表示不设上限（滑块拖到底可关闭此过滤）
        int   maxBlobs = 64;   // 最多输出多少个（防爆）
        float minCircularity = 0.0f;  // 圆度下限[0-1]；<=0 关闭圆度过滤，越接近1越严格
    };

    class CentroidDetector {
    public:
        // gray: 灰度 buffer(w*h, 行优先)。返回检测到的质心列表（按面积降序）。
        // collectContours=true 时额外收集每个连通域的边界像素(见 Blob::contour
        // 注释)；默认 false，保持现有调用方零额外开销。
        std::vector<Blob> detect(const uint8_t* gray, int w, int h,
            const DetectParams& p, bool collectContours = false);

    private:
        std::vector<int32_t> stack_;   // flood fill 复用，避免每帧分配
        std::vector<uint8_t> visited_;
        std::vector<int>     fgIdxBuf_;   // SIMD 扫描出的前景像素下标，分块复用
    };

} // namespace mocap