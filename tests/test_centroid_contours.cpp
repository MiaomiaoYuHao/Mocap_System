#include "detect/CentroidDetector.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

using namespace mocap;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { \
    if (cond) { ++g_pass; } \
    else { ++g_fail; std::printf("FAIL: %s\n", msg); } \
} while(0)

static std::vector<uint8_t> makeDisk(int w, int h, double cx, double cy, double r, uint8_t bg=0, uint8_t fg=200) {
    std::vector<uint8_t> img(size_t(w*h), bg);
    for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
        const double dx=x-cx, dy=y-cy;
        if (dx*dx+dy*dy <= r*r) img[size_t(y*w+x)] = fg;
    }
    return img;
}

int main() {
    // ---- 场景 1：默认(不传 collectContours)行为不变——向后兼容，老调用方
    // 不用改代码。----
    {
        auto img = makeDisk(20,20, 10,10, 4);
        CentroidDetector det;
        DetectParams p; p.threshold=100; p.minArea=1;
        auto blobs = det.detect(img.data(), 20, 20, p);   // 老式4参数调用
        CHECK(blobs.size()==1, "backward-compat: 4-arg call still detects the blob");
        CHECK(blobs[0].contour.empty(), "backward-compat: contour empty when collectContours not requested");
    }

    // ---- 场景 2：开启轮廓收集，边界点数量应该显著小于总面积(不是把整个
    // 连通域都当轮廓)，且边界点确实分布在圆周附近(用"点到圆心距离接近
    // 半径"这个几何性质验证，不是随便统计个数字)。----
    {
        const double cx=15, cy=15, r=6;
        auto img = makeDisk(30,30, cx, cy, r);
        CentroidDetector det;
        DetectParams p; p.threshold=100; p.minArea=1;
        auto blobs = det.detect(img.data(), 30, 30, p, /*collectContours=*/true);
        CHECK(blobs.size()==1, "contour collection: still detects exactly one blob");
        if (!blobs.empty()) {
            const auto& b = blobs[0];
            CHECK(!b.contour.empty(), "contour collection: contour is non-empty when requested");
            CHECK(int(b.contour.size()) < b.area, "contour: boundary point count less than total area (not the whole blob)");

            double maxDevFromRadius = 0.0;
            for (const auto& pt : b.contour) {
                const double dx = double(pt[0])-cx, dy = double(pt[1])-cy;
                const double d = std::sqrt(dx*dx+dy*dy);
                maxDevFromRadius = std::max(maxDevFromRadius, std::abs(d-r));
            }
            char msg[128];
            std::snprintf(msg, sizeof(msg), "contour points lie near the true circle boundary (max dev=%.2fpx)", maxDevFromRadius);
            CHECK(maxDevFromRadius < 1.5, msg);   // 像素离散化引入的容差
        }
    }

    // ---- 场景 3：花生形(两个相邻圆盘)——contour 应该同时包含两个圆各自
    // 的边界，不会把整坨当成一个简单凸形轮廓丢失细节(用"轮廓点里同时存在
    // 靠近圆A和靠近圆B的点"来验证，不直接调用3c，那是另一层的职责，这里
    // 只验证 CentroidDetector 如实吐出了轮廓，没有在检测层就把两个球的
    // 边界信息弄丢)。----
    {
        const int w=40,h=25;
        std::vector<uint8_t> img(size_t(w*h), 0);
        const double r=6, cAx=15, cBx=25, cy=12;
        for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
            double dxA=x-cAx, dyA=y-cy, dxB=x-cBx, dyB=y-cy;
            if (dxA*dxA+dyA*dyA<=r*r || dxB*dxB+dyB*dyB<=r*r) img[size_t(y*w+x)]=200;
        }
        CentroidDetector det;
        DetectParams p; p.threshold=100; p.minArea=1; p.maxArea=0;
        auto blobs = det.detect(img.data(), w, h, p, true);
        CHECK(blobs.size()==1, "peanut shape: still one connected component (as expected, they touch/overlap)");
        if (!blobs.empty()) {
            bool nearA=false, nearB=false;
            for (const auto& pt : blobs[0].contour) {
                const double dxA = double(pt[0])-cAx, dyA = double(pt[1])-cy;
                const double dxB = double(pt[0])-cBx, dyB = double(pt[1])-cy;
                if (std::sqrt(dxA*dxA+dyA*dyA) < r*0.3) nearA = true;
                if (std::sqrt(dxB*dxB+dyB*dyB) < r*0.3) nearB = true;
            }
            // 注意：靠近各自圆心的轮廓点应该找不到(轮廓在边缘，不在圆心)，
            // 这里反过来验证——如果轮廓点离圆心很近，说明轮廓提取把内部
            // 点也当成了边界，逻辑有问题。
            CHECK(!nearA && !nearB, "peanut shape: contour points stay near the boundary, not the interior");
        }
    }

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    if (g_fail>0) { std::printf("SOME TESTS FAILED\n"); return 1; }
    std::printf("ALL PASS\n");
    return 0;
}
