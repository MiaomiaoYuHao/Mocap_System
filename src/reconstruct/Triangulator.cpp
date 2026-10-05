#include "reconstruct/Triangulator.hpp"
#include "camera/ICamera.hpp"
#include <algorithm>
#include <vector>
#include <memory>

namespace mocap {

Triangulator::Triangulator(const QVector<ICamera*>& cams, CalibrationStore* store, QObject* parent)
    : QObject(parent), cams_(cams), store_(store) {
    for (ICamera* c : cams_) {
        if (!c) continue;
        deviceKeys_.insert(c->id(), c->deviceKey());
        connect(c, &ICamera::blobsReady, this, &Triangulator::onBlobs);
    }

    pendingTimer_ = new QTimer(this);
    pendingTimer_->setSingleShot(true);
    connect(pendingTimer_, &QTimer::timeout, this, &Triangulator::tryTriangulate);
}

int Triangulator::calibratedCount() const {
    int n = 0;
    for (ICamera* c : cams_) {
        if (!c || !store_) continue;
        if (store_->get(c->deviceKey()).isCalibrated()) ++n;
    }
    return n;
}

void Triangulator::onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns) {
    if (pts.isEmpty()) return;
    if (!deviceKeys_.contains(camId)) return;   // 不是本三角化组里的相机
    // 滚动保留最近两次观测：上一次的 cur 变成 prev，新的进 cur。两点
    // 用于把这台相机的 2D 位置插值/外推到统一基准时刻（时间插值补偿）。
    CamBuf& buf = latest_[camId];
    buf.prev = buf.cur;
    buf.cur = Obs{ pts.first(), ts_ns, true };
    // 去抖：这一批里第一个到达的观测启动计时器，之后同一小段窗口
    // (kBatchWindowMs) 内陆续到达的其它相机观测只更新 latest_、不重复
    // 启动计时器——等窗口到期后统一跑一次 tryTriangulate()，让原本几乎
    // 同时到达但顺序错开几毫秒的多台相机观测有机会都被收进同一轮，
    // 不会被"先到的两台"贪心配对消费掉、导致晚到的第三台永远赶不上。
    if (!pendingTimer_->isActive()) pendingTimer_->start(kBatchWindowMs);
}

// 把某台相机的 2D 观测插值/外推到 targetTs。两点都在时用线性内插/外推；
// 只有一点或时间跨度异常时退回最近点。外推限制在一个采样间隔内，避免
// 目标时刻离得太远时线性模型失效外推出离谱的位置。
bool Triangulator::sampleAt(const CamBuf& buf, qint64 targetTs, QPointF& out) const {
    if (!buf.cur.have) return false;
    if (!buf.prev.have) { out = buf.cur.pt; return true; }   // 只有一点，无法插值
    const qint64 dt = buf.cur.ts - buf.prev.ts;
    if (dt <= 0) { out = buf.cur.pt; return true; }           // 时间戳异常，退回最近点
    // 线性参数 u：u=0 在 prev，u=1 在 cur，u>1 是往 cur 之后外推。
    double u = double(targetTs - buf.prev.ts) / double(dt);
    // 外推上限：最多外推一个采样间隔（u<=2），再远线性模型不可信。
    if (u > 2.0) u = 2.0;
    if (u < -1.0) u = -1.0;
    out = QPointF(buf.prev.pt.x() + u * (buf.cur.pt.x() - buf.prev.pt.x()),
                  buf.prev.pt.y() + u * (buf.cur.pt.y() - buf.prev.pt.y()));
    return true;
}

void Triangulator::tryTriangulate() {
    // 找当前所有相机里"尚未被消费过的最新样本"的时间戳，作为"这一时刻"
    // 的基准。全部相机都插值/外推到这个统一时刻，消除各相机曝光时刻不
    // 对齐带来的、跟球速成正比的三角化误差（①时间插值补偿）。
    // 注意：判断"尚未消费"用 cur.ts > consumedTs，不能用 cur.have——
    // cur.have 现在永远为 true（只要收到过至少一次观测），专门留给
    // sampleAt() 判断"有没有数据"，跟"这份数据这一轮能不能用"是两件事，
    // 混在一起就是 prev 被消费标记污染、插值永久失效那个坑（见 .hpp
    // CamBuf 的注释）。
    qint64 maxTs = -1;
    for (auto it = latest_.constBegin(); it != latest_.constEnd(); ++it) {
        const CamBuf& buf = it.value();
        if (buf.cur.have && buf.cur.ts > buf.consumedTs && buf.cur.ts > maxTs)
            maxTs = buf.cur.ts;
    }
    if (maxTs < 0) return;

    // 凑齐时间窗内的观测，构造 N 视图输入。用 vector 持有标定副本，保证
    // ViewObservation 里的指针在 triangulateMultiView 调用期间有效。
    std::vector<CameraCalibration> calibs;
    std::vector<QPointF> pixels;
    std::vector<quint32> usedIds;
    calibs.reserve(cams_.size());
    pixels.reserve(cams_.size());

    for (auto it = latest_.constBegin(); it != latest_.constEnd(); ++it) {
        const CamBuf& buf = it.value();
        if (!buf.cur.have) continue;
        if (buf.cur.ts <= buf.consumedTs) continue;   // 这份样本上一轮已经用过，不重复用
        if (maxTs - buf.cur.ts > tolNs_) continue;   // 最近观测都超出时间窗，这台这轮不参与
        const QString key = deviceKeys_.value(it.key());
        const CameraCalibration c = store_->get(key);
        if (!c.isCalibrated()) continue;

        // 把这台相机的 2D 位置对齐到统一基准时刻 maxTs。开了时间插值就
        // 插值/外推，否则直接用最近观测（老行为）。
        QPointF pt;
        if (timeInterp_) {
            if (!sampleAt(buf, maxTs, pt)) continue;
        } else {
            pt = buf.cur.pt;
        }
        calibs.push_back(c);
        pixels.push_back(pt);
        usedIds.push_back(it.key());
    }

    if (calibs.size() < 2) return;   // 有效同时刻观测不足两台

    std::vector<ViewObservation> obs;
    obs.reserve(calibs.size());
    for (size_t i = 0; i < calibs.size(); ++i)
        obs.push_back(ViewObservation{ &calibs[i].intr, &calibs[i].extr,
                                       pixels[i].x(), pixels[i].y() });

    // droppedMask[i] 对应 obs[i]/usedIds[i]——鲁棒剔除(robust_==true)时，
    // 这台相机的观测有没有被判定为坏视角、剔出这次解算。非鲁棒模式下
    // 传nullptr，全部视角都算参与（保持跟原来一致的行为）。
    // droppedMask/weights[i] 对应 obs[i]/usedIds[i]——Hard模式下是"被剔除了
    // 没有"，Soft模式下是连续权重，这里统一转换成"算不算参与"的布尔判断
    // (权重>=0.5才算)，方便下游 usedCamIds 的构造逻辑不用关心具体是哪种
    // 模式算出来的。Off模式全部视角都算参与，保持跟原来一致的行为。
    std::vector<bool> dropped(obs.size(), false);
    TriangulateResult r;
    if (robustMode_ == RobustMode::Hard) {
        auto maskBuf = std::make_unique<bool[]>(obs.size());
        r = triangulateMultiViewRobust(obs.data(), int(obs.size()), 2, 3.0, maskBuf.get());
        for (size_t i = 0; i < obs.size(); ++i) dropped[i] = maskBuf[i];
    } else if (robustMode_ == RobustMode::Soft) {
        auto weightsBuf = std::make_unique<double[]>(obs.size());
        r = triangulateMultiViewRobustSoft(obs.data(), int(obs.size()), 3.0, weightsBuf.get());
        for (size_t i = 0; i < obs.size(); ++i) dropped[i] = (weightsBuf[i] < 0.5);
    } else {
        r = triangulateMultiView(obs.data(), int(obs.size()));
    }

    QVector<quint32> usedCamIds;
    for (size_t i = 0; i < usedIds.size(); ++i)
        if (!dropped[i]) usedCamIds.push_back(usedIds[i]);

    // 消费掉参与本次解算的观测，避免下次用陈旧数据重复三角化。记
    // consumedTs（这份样本的时间戳），而不是清 cur.have——cur.have 永远
    // 表示"收到过观测"，prev/cur 的滚动历史完整保留，下一帧新观测进来
    // 插值不会被"上一份被消费过"这件事打断（这正是修复前的坑：清 cur.have
    // 会在下一帧 `prev = cur` 时把"已消费"状态带进 prev，稳态下插值永远
    // 进不去 `prev.have && cur.have` 的分支，静默退化成插值出现前的
    // 逐帧最近点方案）。
    for (quint32 id : usedIds) latest_[id].consumedTs = latest_[id].cur.ts;

    if (!r.valid) return;

    // ③输出端 One Euro 滤波：慢速多平滑、快速不拖尾。
    std::array<double,3> outPt = r.point;
    if (filterEnabled_)
        outPt = filter_.filter(r.point, maxTs);

    emit point3DReady(QVector3D(float(outPt[0]), float(outPt[1]), float(outPt[2])),
                      r.residual, int(usedCamIds.size()), maxTs, usedCamIds);

    // ---- 滑动平均残差，判断标定/场景是否匹配 ----
    // 用滤波前的原始 residual，反映的是三角化本身的一致性（滤波是输出端
    // 美化，不该影响"标定质量"这个诊断量）。
    residualWindow_.push_back(r.residual);
    if (residualWindow_.size() > kWindowSize) residualWindow_.removeFirst();

    if (residualWindow_.size() >= kMinSamples) {
        double sum = 0;
        for (double v : residualWindow_) sum += v;
        const double avg = sum / residualWindow_.size();

        const TriangulationQuality q = avg <= goodMm_   ? TriangulationQuality::Good
                                      : avg <= warnMm_   ? TriangulationQuality::Warning
                                                          : TriangulationQuality::Bad;
        if (!hasQuality_ || q != lastQuality_) {
            hasQuality_ = true;
            lastQuality_ = q;
            emit qualityChanged(q, avg, residualWindow_.size());
        }
    }
}

} // namespace mocap
