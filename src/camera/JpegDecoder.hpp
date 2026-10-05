#pragma once
// ---------------------------------------------------------------------------
// MJPG 帧的统一解码入口。项目里两个采集引擎——DShowCapture(主用，原生
// DirectShow) 和 WebcamCamera::onFrame(Qt/Media Foundation 兜底)——原本各自
// 直接调 QImage::fromData / QVideoFrame::toImage 解 JPEG，各解各的。现在统一
// 收拢到这一个函数，好处是"灰度直解"这个优化只在一个地方实现、一个开关统管，
// 任何取画面的源头都自动走同一条路径，不会有某条路径漏掉。
//
// 【灰度直解是什么、为什么对这套场景是净赚】
// 相机装了红外滤光片，画面本身就是灰的——但 MJPG 仍然是按彩色 JPEG 编码的，
// 里面照样有 Y + Cb + Cr 三个平面，只是 Cb/Cr 数值接近中性(灰)。默认解码
// (QImage::fromData)会把三个平面全解、做 YCbCr→RGB 矩阵运算、输出 3 通道——
// 也就是每帧都在花 CPU 解一堆注定要丢弃的中性灰色度数据，纯浪费。
// libjpeg-turbo 设 TJPF_GRAY 后，解码器直接跳过色度平面的上采样和颜色矩阵，
// 只输出 Y 单通道。对"黑底白球+滤光片"画面零信息损失，且：
//   - 解码本身更快(不解色度、不做颜色矩阵，turbo 还带 SIMD)；
//   - 输出直接是 Grayscale8，下游检测/录制/预览全链路 1/3 数据量，
//     且省掉后续的 convertToFormat(Grayscale8)。
// 4 路×120fps=480 帧/秒 的解码密度下，这一步是全链路最重的一环，省一半解码
// 时间直接兑现成"更不易丢帧 + 主机发热腰斩 + 给下游算法腾出 CPU"。
//
// 【开关语义】全局开关 setGrayDecode(true) 打开后，所有走本函数的 MJPG 解码
// 都尽力灰度直解。是否真的走 turbo 取决于编译时有没有 HAVE_TURBOJPEG：
//   - 编了 turbo：走 tjDecompress2(...TJPF_GRAY...)，真正的灰度直解。
//   - 没编 turbo：回退到 QImage::fromData 解出彩色、再按需转灰度——功能不变、
//     只是没有性能收益。这样缺依赖也能编过、能跑，turbo 是纯加速项、不是硬依赖。
// ---------------------------------------------------------------------------
#include <QImage>
#include <QByteArray>
#include <atomic>
#include <cstdint>

namespace mocap {

// 全局灰度直解开关。默认关(false)——保持和历史行为一致(解出彩色原样帧)，
// 由上层 UI(“算法灰度/灰度直解”功能键)显式打开。原子，因为解码发生在各相机
// 的采集线程上，开关可能在主线程被拨动。
inline std::atomic<bool>& grayDecodeEnabled() {
    static std::atomic<bool> on{false};
    return on;
}
inline void setGrayDecode(bool on) { grayDecodeEnabled().store(on); }
inline bool isGrayDecode()         { return grayDecodeEnabled().load(); }

// 编译期能力查询：true 表示这份构建真的链了 libjpeg-turbo、灰度直解能生效。
// UI 可以用它决定要不要把开关标成“(未编译 turbo，仅回退)”之类的提示。
bool turboAvailable();

// ---------------------------------------------------------------------------
// 【解码耗时基准】——不是估算，是每次真实调用 decodeMjpeg() 时实测的数字。
// 按走的是哪条解码路径分两类分别累计：
//   turboGray ：libjpeg-turbo 的 TJPF_GRAY 直解(跳过色度+颜色矩阵那条)。
//   qtColor   ：QImage::fromData 的完整彩色解码(不管解完要不要再转灰度，
//               "解码"这一步本身的耗时以这个为准，后面转灰度算另一笔账，
//               不计入这里——避免把两件事的耗时混在一起看不清各自贡献)。
// 开着"算法灰度"开关跑一段时间、再关掉跑一段时间，对比这两组数字，就是
// 你机器上灰度直解实际省了多少的真实答案，不是理论推算。
// ---------------------------------------------------------------------------
struct DecodeStatsSnapshot {
    quint64 turboGrayFrames = 0;   // 只计成功解码的帧数
    double  turboGrayAvgMs  = 0.0; // 只用成功样本算的平均每帧解码耗时(毫秒)
    quint64 turboGrayFails  = 0;   // 解码失败次数(坏帧/截断数据)，不计入上面的平均值
    quint64 qtColorFrames   = 0;
    double  qtColorAvgMs    = 0.0;
    quint64 qtColorFails    = 0;
};

// 取当前累计快照(只读，不重置)。挂个定时器每隔一两秒调一次拿来更新 UI 即可，
// 内部只是几个原子读取，开销可忽略。
DecodeStatsSnapshot decodeStatsSnapshot();

// 清空累计——切换"算法灰度"开关前后各清一次，方便看某一段时间窗口内的
// 平均值，而不是从程序启动到现在混在一起的总平均。
void resetDecodeStats();

// 把一段 MJPG/JPEG 字节解成 QImage。
//   - grayDecodeEnabled() 为 true 且编了 turbo：直接解成 Format_Grayscale8。
//   - 否则：QImage::fromData 解出彩色；若开关为 true 再 convertToFormat 转灰度
//     (保证“开关打开→输出一定是灰度”这个承诺不因缺 turbo 而失效，只是不省 CPU)。
// 返回空 QImage 表示解码失败(残帧/坏数据)，调用方按老逻辑丢弃即可。
QImage decodeMjpeg(const uint8_t* data, int len);

// QByteArray 便捷重载。
inline QImage decodeMjpeg(const QByteArray& bytes) {
    return decodeMjpeg(reinterpret_cast<const uint8_t*>(bytes.constData()),
                       bytes.size());
}

} // namespace mocap
