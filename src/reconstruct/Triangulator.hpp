#pragma once
// ---------------------------------------------------------------------------
// 实时 N 视图三角化（N>=2 自适应）：订阅任意多台相机的 blobsReady，把落在
// 同一时间窗（±tolNs，仿 solve_calibration.py 的 TOL_NS）内的各相机观测
// 凑成一组，只要有 >=2 台看到就三角化——加多少台相机，算法自动适应多少台，
// 不需要为不同台数写不同分支（DLT 天然支持任意台数，多一台多两行约束）。
//
// 单球场景：每台相机取检测到的第一个 blob（CentroidDetector 已按面积降序）。
// 多球/多标记点场景需要先解决"哪个2D点对应哪个球"的身份匹配，不在本类
// 职责内——那是后续 MarkerLabeler 的活。
//
// 时间窗成组策略：以"任一相机来了新观测"为触发，回看每台相机最近一次观测，
// 把时间戳落在 [maxTs - tolNs, maxTs] 内的都算作"同一时刻"，凑齐后三角化
// 并清空已用观测，避免同一批观测被重复解算。
//
// ---- 标定/场景匹配度反馈 ----
// 每次三角化的 residual（各相机视线到解出的点的平均垂距）是单帧噪声很大
// 的量，偶尔飙高很正常（检测抖动/球一闪而过），不能拿单帧说话。这里维护
// 一个滑动平均窗口，只有"平均残差持续偏大"才判定为"标定可能跟当前物理
// 场景不匹配"（典型原因：相机被碰动过、或者选错了模板），并且只在质量
// 等级发生变化时才发一次 qualityChanged 信号，不会每帧刷屏。
// ---------------------------------------------------------------------------
#include "reconstruct/Triangulation.hpp"
#include "reconstruct/OneEuroFilter.hpp"
#include "calib/CalibrationStore.hpp"
#include <QObject>
#include <QVector3D>
#include <QPointF>
#include <QVector>
#include <QHash>
#include <QTimer>

namespace mocap {

class ICamera;

enum class TriangulationQuality { Good, Warning, Bad };

class Triangulator : public QObject {
    Q_OBJECT
public:
    // cams：参与三角化的相机（>=2 台）。store 生命周期需比本类长，本类不持有。
    Triangulator(const QVector<ICamera*>& cams, CalibrationStore* store,
                 QObject* parent = nullptr);

    void setTimeToleranceNs(qint64 ns) { tolNs_ = ns; }

    // 至少有两台相机已标定（内外参都 valid）才可能出结果。
    int calibratedCount() const;

    // 滑动平均残差(mm)的判档阈值：<=goodMm 良好，<=warnMm 一般值得留意，
    // 再往上判定"差"。默认值对应"正常应在几毫米量级"这个经验判断。
    void setQualityThresholds(double goodMm, double warnMm) { goodMm_ = goodMm; warnMm_ = warnMm; }

    // ---- 精度/去抖相关开关（默认全开，抖动明显改善；排查问题时可单独关） ----
    // 时间插值补偿：多台相机曝光时刻天然不对齐，球在动时直接拿"最近一次
    // 观测"的原始像素配对会产生跟球速成正比的误差（表现为抖）。开启后，
    // 每台相机维护最近两次观测，三角化时把各自的 2D 位置插值/外推到统一
    // 的基准时刻再解算，从根上消除这类时间错位误差。
    void setTimeInterpolation(bool on) { timeInterp_ = on; }
    // 鲁棒模式：三选一。
    //   Off  ：不做任何鲁棒处理，全部视角等权重直接解——排查问题时用，
    //          对照"完全不处理"这个基线。
    //   Hard ：踢掉跟大多数视角不一致的坏视角，只用剩下一致的视角求解。
    //          问题：这是离散的"留/弃"决策，某视角的质量连续变化、
    //          跨过阈值的那一刻，参与解算的视角集合突变，解出的3D点
    //          会有一个台阶——多相机覆盖范围有重叠边界的场景里，这个
    //          台阶正好卡在"某台相机质量刚好跨过阈值"的空间位置，表现
    //          出来就是"某些区域/路线固定跳变"。
    //   Soft ：IRLS软加权(见 triangulateMultiViewRobustSoft 顶部说明)，
    //          用连续权重代替离散的留/弃，同样的离群抑制能力，但视角
    //          质量变化时权重平滑过渡，不会有台阶。默认用这个。
    enum class RobustMode { Off, Hard, Soft };
    void setRobustMode(RobustMode mode) { robustMode_ = mode; }
    // 向后兼容的旧接口——on=true 等价于 Soft(新默认，不是旧的Hard，因为
    // Soft在能做到的一切上都不比Hard差，还消除了台阶，没有理由继续默认
    // 用会跳变的版本)，on=false 等价于 Off。
    void setRobustRejection(bool on) { robustMode_ = on ? RobustMode::Soft : RobustMode::Off; }
    // 输出端 One Euro 滤波：对最终 3D 点做自适应低通，慢速多平滑、快速
    // 不拖尾。传 minCutoff<=0 关闭滤波。
    void setOutputFilter(double minCutoff, double beta, double dCutoff = 1.0) {
        filterMinCutoff_ = minCutoff; filterBeta_ = beta; filterDCutoff_ = dCutoff;
        filter_.setParams(minCutoff > 0 ? minCutoff : 1.0, beta, dCutoff);
        filterEnabled_ = (minCutoff > 0);
    }

signals:
    // 三角化成功：世界系坐标(mm) + 平均视线垂距(mm) + 参与解算的相机数
    // + 具体是哪几台相机(camId)参与了这次解算。
    // usedCamIds 不只是让 UI 显示"这一帧几台相机一起算的"，更关键的用途：
    // 逐帧对比 usedCamIds 集合有没有变化——如果输出点位置在某个位置附近
    // 反复出现小跳变/悬崖，而这个位置恰好是某台相机开始/停止参与解算的
    // 边界(比如那台相机的重投影残差在这附近正好跨过鲁棒剔除的阈值)，
    // 说明这不是检测噪声，是"相机接力"artifact——不同相机子集之间标定
    // 的系统性差异(哪怕很小)，换一批相机重新解出来的坐标就会有一个台阶。
    // 这是多相机三角化的经典毛病，好几个相机看同一个点、覆盖范围有重叠
    // 又有边界的场景下几乎必然存在，程度取决于相机之间标定的自洽程度。
    void point3DReady(QVector3D pos, double residualMm, int usedViews, qint64 ts_ns,
                      QVector<quint32> usedCamIds);

    // 只在质量等级变化时触发（Good/Warning/Bad 三档），带上当前滑动平均值
    // 和窗口内样本数，供 UI 做"持续偏大就报警"，不被单帧噪声刷屏。
    void qualityChanged(TriangulationQuality quality, double avgResidualMm, int windowSize);

private slots:
    void onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns);

private:
    void tryTriangulate();

    // 每台相机保留最近两次观测，用于把 2D 位置插值/外推到统一基准时刻
    // （①时间插值补偿）。prev 是上上次，cur 是最近一次；只有一次观测时
    // prev.have==false，退回不插值直接用 cur。
    //
    // consumedTs 单独记录"这台相机最近一次被某轮三角化用掉的样本时间戳"，
    // 不能偷懒复用 cur.have 来表示"已消费"——如果消费时把 cur.have 置
    // false，下一帧 onBlobs 里 `prev = cur` 会把这个"已消费"标记也带进
    // prev，导致 prev.have 变 false，插值条件 `prev.have && cur.have`
    // 永远满足不了（稳态下：每帧都参与三角化 -> 每帧都被消费 -> 下一帧
    // prev 永远是"已消费"状态 -> 插值永久失效，静默退化成插值出现前的
    // 逐帧最近点方案，是这版最容易被忽略的一个坑）。改成时间戳判断"这个
    // 样本是不是比上次消费的还新"，prev/cur 的 have 只反映"是不是收到过
    // 真实观测"，永不因为消费而被清掉。
    struct Obs { QPointF pt; qint64 ts = -1; bool have = false; };
    struct CamBuf { Obs prev, cur; qint64 consumedTs = -1; };

    // 把某台相机的 2D 观测线性插值/外推到目标时刻 targetTs。有两点且
    // targetTs 在合理范围内就用两点连线求值；否则退回最近点（cur）。
    // 返回是否得到可用的点。
    bool sampleAt(const CamBuf& buf, qint64 targetTs, QPointF& out) const;

    QVector<ICamera*> cams_;
    CalibrationStore* store_;
    qint64 tolNs_ = 12000000;               // 12ms，同 solve_calibration.py 的 TOL_NS
    QHash<quint32, CamBuf> latest_;         // camId -> 最近两次观测
    QHash<quint32, QString> deviceKeys_;    // camId -> deviceKey（缓存，避免每帧查）

    bool   timeInterp_ = true;              // ①时间插值补偿，默认开
    RobustMode robustMode_ = RobustMode::Soft;   // ②鲁棒模式，默认软加权(IRLS)
    OneEuroFilter3 filter_{ 30.0, 0.5 };    // ③输出端 One Euro 滤波
    bool   filterEnabled_ = true;
    double filterMinCutoff_ = 30.0, filterBeta_ = 0.5, filterDCutoff_ = 1.0;

    // 批处理去抖：多台相机是各自独立的采集/检测线程，同一时刻的观测到达
    // Triangulator 的先后顺序天然会错开几毫秒。如果一来观测就立刻
    // tryTriangulate()，会出现"贪心两两配对"问题——比如3台相机A/B/C几乎
    // 同时看到球，A先到触发一次（这时B/C还没到，凑不够2台，跳过）；B紧
    // 跟着到，这次A+B凑够2台就地三角化并把A/B标记"已消费"；C再到时，
    // A/B已经被消费掉了，C自己凑不够2台，这一轮又被跳过——结果永远只有
    // 2台参与，即使3台其实都在同一个共视窗口内。这个 pendingTimer_ 让
    // 第一个到达的观测先攒一小段时间（kBatchWindowMs），把这段时间内陆续
    // 到达的其它相机观测也收进来，再统一跑一次 tryTriangulate()，避免
    // 过早地把还没到齐的这一批"抢跑"消费掉。
    QTimer* pendingTimer_ = nullptr;
    static constexpr int kBatchWindowMs = 12;   // 小于 tolNs_(12ms)，只是让近乎同时的观测有机会都到齐

    QVector<double> residualWindow_;
    static constexpr int kWindowSize = 30;   // 滑动窗口取最近多少帧
    static constexpr int kMinSamples = 8;    // 样本不够时先别急着判断，避免开局几帧噪声就报警
    double goodMm_ = 8.0, warnMm_ = 25.0;
    TriangulationQuality lastQuality_ = TriangulationQuality::Good;
    bool hasQuality_ = false;
};

} // namespace mocap
