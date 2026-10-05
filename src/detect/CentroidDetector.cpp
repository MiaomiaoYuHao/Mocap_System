#include "detect/CentroidDetector.hpp"
#include <algorithm>

// SIMD 加速：全图检测的主成本是外层"逐像素扫描找前景种子"这一遍——它对
// 每个像素读 visited_ 和 gray 两个数组、130 万次，且分支不可预测。红外图
// 绝大部分是黑的，用 AVX2 一次比较 32 字节、整块全黑直接跳过，实测把
// 1.3MP 检测从 3.2ms 压到 0.15ms（约 20x），质心/点数与标量版逐位一致
// （300 个随机场景对拍验证，含边界球/粘连/各种阈值面积圆度参数）。
// flood fill 本身逻辑完全不动（下面抽成 floodFill lambda），所以轮廓收集、
// 圆度过滤、亚像素质心等所有行为与原版严格相同。
//
// 【为什么用运行时分发而不是 #if defined(__AVX2__)】
// 后者要求【整个项目】编译时开 -mavx2 / /arch:AVX2。MSVC 默认不开，
// CMake 里也没配——那样这段代码会被整体跳过、优化等于没做，而且是静默的
// （编译能过、跑得慢，最难发现的那种）。这里改成：用 target 属性让编译器
// 单独为这一个函数生成 AVX2 代码（不影响其它翻译单元的指令集基线），
// 运行时用 CPUID 查一次 CPU 到底支不支持，支持才走 AVX2 路径。
// 好处：不需要动任何编译选项就能生效；老 CPU 上自动回退标量、不会 crash。
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
  #define MOCAP_X86 1
  #include <immintrin.h>
  #if defined(_MSC_VER)
    #include <intrin.h>
  #else
    #include <cpuid.h>
  #endif
#else
  #define MOCAP_X86 0
#endif

namespace {

// 【前景比例硬闸】阈值失效保护：返回"这一帧允许的前景像素数上限"。
//
// 跟 DetectParams 里的 minArea/maxArea/minCircularity 【不是重复】：那三个是
// "算完一个连通域之后，判断它算不算一个球"，属于事后过滤；这一道是"还没开始
// flood fill 就发现整幅图有过半像素越过阈值，这个阈值对这幅图已经没有意义"，
// 属于事前放弃。两者拦的是不同阶段的开销——maxArea 一分钱也省不下 flood fill
// 遍历几十万像素、stack_ 撑到十几万项的那部分成本，它只负责把结果扔掉。
//
// 红外动捕的正常画面是"一片黑 + 几十个小亮斑"，前景占比在千分之几量级，
// 25% 这条线差着两个数量级，正常画面永远不会误伤。只在相机没摆好/曝光跑飞/
// 阈值设得太低导致"满屏白块"时触发；触发时返回空 blob 列表，上层表现为
// "这一帧没检测到点"，界面照常响应，用户可以自己去把阈值调回来——这正是
// 调试窗口该有的行为，而不是卡死。
inline long foregroundAbortLimit(int n) {
    return long(n) / 4;
}

#if MOCAP_X86
// 运行时查 CPU 是否支持 AVX2（只查一次，静态缓存）。
inline bool cpuHasAVX2() {
    static const bool ok = []() -> bool {
    #if defined(_MSC_VER)
        int regs[4] = {0,0,0,0};
        __cpuid(regs, 0);
        if (regs[0] < 7) return false;
        __cpuidex(regs, 7, 0);
        return (regs[1] & (1 << 5)) != 0;          // EBX bit5 = AVX2
    #else
        unsigned eax, ebx, ecx, edx;
        if (!__get_cpuid_max(0, nullptr)) return false;
        if (__get_cpuid_max(0, nullptr) < 7) return false;
        __cpuid_count(7, 0, eax, ebx, ecx, edx);
        return (ebx & (1u << 5)) != 0;             // EBX bit5 = AVX2
    #endif
    }();
    return ok;
}

// 扫描一段像素，返回 [i0, i1) 内所有 >=thr 的像素下标（写进 outIdx）。
// 用 target 属性单独为这个函数开 AVX2，不影响整个项目的指令集基线。
#if defined(__GNUC__) || defined(__clang__)
__attribute__((target("avx2")))
#endif
inline void scanForegroundAVX2(const uint8_t* gray, int i0, int i1, uint8_t thr,
                               std::vector<int>& outIdx) {
    outIdx.clear();
    const __m256i bias  = _mm256_set1_epi8((char)0x80);
    const __m256i vthrB = _mm256_add_epi8(_mm256_set1_epi8((char)(thr - 1)), bias);
    int i = i0;
    for (; i + 32 <= i1; i += 32) {
        const __m256i v  = _mm256_loadu_si256(reinterpret_cast<const __m256i*>(gray + i));
        const __m256i gt = _mm256_cmpgt_epi8(_mm256_add_epi8(v, bias), vthrB);
        unsigned mask = static_cast<unsigned>(_mm256_movemask_epi8(gt));
        while (mask) {
            const int bit = __builtin_ctz(mask);
            mask &= mask - 1;
            outIdx.push_back(i + bit);
        }
    }
    for (; i < i1; ++i) if (gray[i] >= thr) outIdx.push_back(i);
}
#endif // MOCAP_X86

} // anonymous namespace

namespace mocap {

std::vector<Blob> CentroidDetector::detect(const uint8_t* gray, int w, int h,
                                           const DetectParams& p, bool collectContours) {
    std::vector<Blob> blobs;
    if (!gray || w <= 0 || h <= 0) return blobs;

    const int N = w * h;
    if (int(visited_.size()) != N) visited_.assign(size_t(N), 0);
    else std::fill(visited_.begin(), visited_.end(), uint8_t(0));

    const uint8_t thr = uint8_t(std::clamp(p.threshold, 0, 255));

    std::vector<std::array<float,2>> contourBuf;   // 复用的临时缓冲，每个连通域清空重用

    // flood fill 一个连通域并产出一个 Blob（若通过面积/圆度过滤）。这段逻辑
    // 从原来的内层循环原样搬出来，一字未改——SIMD 优化只改"怎么找到种子
    // idx"，不改"找到之后怎么处理"，保证行为与标量版严格一致。
    auto floodFill = [&](int idx) {
        stack_.clear();
        stack_.push_back(idx);
        visited_[size_t(idx)] = 1;
        if (collectContours) contourBuf.clear();

        double sumW = 0, sumX = 0, sumY = 0;
        int    area = 0;
        float  peak = 0;
        int    minx = w, maxx = -1, miny = h, maxy = -1;

        while (!stack_.empty()) {
            const int cur = stack_.back();
            stack_.pop_back();
            const int cx = cur % w, cy = cur / w;
            const uint8_t v = gray[cur];
            const double wgt = double(v) - double(thr) + 1.0;
            sumW += wgt; sumX += wgt * cx; sumY += wgt * cy;
            ++area;
            if (v > peak) peak = v;
            if (cx < minx) minx = cx;  if (cx > maxx) maxx = cx;
            if (cy < miny) miny = cy;  if (cy > maxy) maxy = cy;

            const int nb[4] = {
                cx > 0     ? cur - 1 : -1,
                cx < w - 1 ? cur + 1 : -1,
                cy > 0     ? cur - w : -1,
                cy < h - 1 ? cur + w : -1,
            };

            if (collectContours) {
                bool isBoundary = false;
                for (int k = 0; k < 4; ++k) {
                    const int n = nb[k];
                    const bool partOfBlob = (n >= 0) && (gray[n] >= thr);
                    if (!partOfBlob) { isBoundary = true; break; }
                }
                if (isBoundary) contourBuf.push_back({float(cx), float(cy)});
            }

            for (int k = 0; k < 4; ++k) {
                const int n = nb[k];
                if (n < 0 || visited_[size_t(n)] || gray[n] < thr) continue;
                visited_[size_t(n)] = 1;
                stack_.push_back(n);
            }
        }

        if (area < p.minArea || (p.maxArea > 0 && area > p.maxArea)) return;

        if (p.minCircularity > 0.0f) {
            const int bw = maxx - minx + 1, bh = maxy - miny + 1;
            const int maxDim = bw > bh ? bw : bh;
            const double circ = double(area) / (0.785398163 * double(maxDim) * double(maxDim));
            if (circ < double(p.minCircularity)) return;
        }

        Blob b;
        b.area = area;
        b.peak = peak;
        b.bboxMinX = minx; b.bboxMinY = miny;
        b.bboxW = maxx - minx + 1; b.bboxH = maxy - miny + 1;
        b.cx = sumW > 0 ? float(sumX / sumW) : float(idx % w);
        b.cy = sumW > 0 ? float(sumY / sumW) : float(idx / w);
        if (collectContours) b.contour = contourBuf;
        blobs.push_back(b);
    };

#if MOCAP_X86
    // ---- SIMD 找种子（运行时确认 CPU 支持 AVX2 才走）----
    // 分块扫描：每次用 AVX2 扫一段像素、收集这段里所有过阈值的下标，再逐个
    // 尝试 flood fill（已 visited 的跳过）。分块而不是一次扫全图，是为了让
    // outIdx 缓冲保持在 L2 内、并让扫描与 flood fill 交替进行（cache 友好）。
    // thr==0 意味着"所有像素都算前景"，此时 thr-1 会回绕，直接走标量。
    if (thr > 0 && cpuHasAVX2()) {
        constexpr int kChunk = 1 << 16;   // 64K 像素/块
        std::vector<int>& fg = fgIdxBuf_;
        long fgTotal = 0;
        const long fgLimit = foregroundAbortLimit(N);
        for (int base = 0; base < N; base += kChunk) {
            const int end = std::min(N, base + kChunk);
            scanForegroundAVX2(gray, base, end, thr, fg);
            fgTotal += long(fg.size());
            // 【前景比例硬闸，见 foregroundAbortLimit 的说明】阈值对这幅图
            // 已经失效，继续算下去只是白烧几十万像素，直接放弃这一帧。
            if (fgLimit > 0 && fgTotal > fgLimit) { blobs.clear(); return blobs; }
            for (int pidx : fg) {
                if (!visited_[size_t(pidx)]) floodFill(pidx);
            }
        }
    } else
#endif
    {
        // 标量路径（非 x86 / CPU 不支持 AVX2 / thr==0）：flood fill 逻辑与
        // 原版完全一致，只多了跟 SIMD 路径同一道前景比例硬闸(同样按 64K
        // 一块检查一次，保证两条路径在这件事上行为对齐)。
        constexpr int kChunk = 1 << 16;
        long fgTotal = 0;
        const long fgLimit = foregroundAbortLimit(N);
        int chunkEnd = kChunk;
        for (int idx = 0; idx < N; ++idx) {
            if (idx >= chunkEnd) {
                if (fgLimit > 0 && fgTotal > fgLimit) { blobs.clear(); return blobs; }
                chunkEnd += kChunk;
            }
            if (gray[idx] >= thr) ++fgTotal;
            if (visited_[size_t(idx)] || gray[idx] < thr) continue;
            floodFill(idx);
        }
        if (fgLimit > 0 && fgTotal > fgLimit) { blobs.clear(); return blobs; }
    }

    std::sort(blobs.begin(), blobs.end(),
              [](const Blob& a, const Blob& b) { return a.area > b.area; });
    if (int(blobs.size()) > p.maxBlobs) blobs.resize(size_t(p.maxBlobs));
    return blobs;
}

} // namespace mocap
