#pragma once
// 正方形网格排布算法。纯逻辑、无 Qt 依赖。
//   cols = ceil(sqrt(n)), rows = ceil(n/cols)，行优先填充，末行缺角。
//   n=4 -> 2x2 满方，n=9 -> 3x3 满方，其余缺角。
#include <vector>

namespace mocap {

struct GridDim { int rows = 0; int cols = 0; };

inline GridDim gridDimFor(int n) {
    if (n <= 0) return {0, 0};
    int cols = 1;
    while (cols * cols < n) ++cols;       // cols = ceil(sqrt(n))，纯整数
    int rows = (n + cols - 1) / cols;     // ceil(n/cols)
    return {rows, cols};
}

struct Cell { int row; int col; };
inline Cell cellFor(int i, const GridDim& g) {
    return { i / g.cols, i % g.cols };
}

struct Rect { int x, y, w, h; };
inline Rect rectFor(int i, int n, int W, int H) {
    GridDim g = gridDimFor(n);
    Cell c = cellFor(i, g);
    int baseW = W / g.cols, remW = W % g.cols;
    int baseH = H / g.rows, remH = H % g.rows;
    int w = baseW + (c.col < remW ? 1 : 0);
    int h = baseH + (c.row < remH ? 1 : 0);
    int x = baseW * c.col + (c.col < remW ? c.col : remW);
    int y = baseH * c.row + (c.row < remH ? c.row : remH);
    return { x, y, w, h };
}

} // namespace mocap
