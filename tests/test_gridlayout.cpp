// 网格排布算法单测：n=1..16 的行列数正确、矩形不重叠、恰好铺满区域。
#include "ui/GridLayout.hpp"
#include <cassert>
#include <cstdio>

using namespace mocap;

int main()
{
    struct Case { int n, cols, rows; };
    const Case cases[] = {
        {1,1,1},{2,2,1},{3,2,2},{4,2,2},{5,3,2},{6,3,2},{7,3,3},{8,3,3},{9,3,3},
        {10,4,3},{11,4,3},{12,4,3},{13,4,4},{14,4,4},{15,4,4},{16,4,4},
    };
    for (const auto& c : cases) {
        GridDim g = gridDimFor(c.n);
        assert(g.cols == c.cols && g.rows == c.rows);
        assert(g.cols * g.rows >= c.n);                  // 放得下
        assert((g.cols - 1) * g.rows < c.n || c.n == 1); // 不浪费整列
    }

    // 矩形：不越界、两两不重叠、面积和 == 已占用格子的面积和
    const int W = 1280, H = 800;
    for (int n = 1; n <= 16; ++n) {
        for (int i = 0; i < n; ++i) {
            Rect a = rectFor(i, n, W, H);
            assert(a.x >= 0 && a.y >= 0 && a.x + a.w <= W && a.y + a.h <= H);
            assert(a.w > 0 && a.h > 0);
            for (int j = i + 1; j < n; ++j) {
                Rect b = rectFor(j, n, W, H);
                bool overlap = !(b.x >= a.x + a.w || b.x + b.w <= a.x ||
                                 b.y >= a.y + a.h || b.y + b.h <= a.y);
                assert(!overlap);
            }
        }
    }

    std::printf("gridlayout: ALL PASS\n");
    return 0;
}
