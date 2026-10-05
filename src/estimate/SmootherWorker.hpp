#pragma once
// ---------------------------------------------------------------------------
// 延迟精修流的独立工作线程——照 detect/DetectWorker.hpp 那套"独立线程 +
// 忙时新数据直接跳过(不排队、不积压)"的模式抄的，同一个道理：这条流本身
// 每帧要做一次窗口联合优化(毫秒级)，跟实时IEKF挤在GUI线程里跑迟早出问题
// (参考之前手指结构标定同步跑导致UI"未响应"那次教训)，丢到独立线程，
// 忙的时候丢帧不排队——丢一帧的代价是这个窗口的精修结果缺了一帧观测，
// 不是什么大问题，比"排队攒积压导致越来越滞后"好得多。
//
// 【跨线程数据安全，这是这个文件存在的主要原因】HandCameraMeasurement
// 内部是个裸指针(const CamPose* cam)，指向 HandTrackingWorker 那边
// camPoses_ 数组里的元素——直接把这个指针原样传给另一个线程持有，即使
// 实践中 camPoses_ 构造完基本不再变、大概率不出事，也是留了一个隐患
// (万一以后哪天camPoses_被resize/重建，那些指针就全悬空了，而且是那种
// "偶发、只在特定时机复现"的诡异bug，最难排查的一类)。这里的做法是：
// SmootherWorker 自己持有一份 camPoses_ 的稳定拷贝(configure()时给一次)，
// 跨线程传输的观测只带 camIndex(纯数值，不是指针)，真正的 HandCameraMeasurement
// 在这个线程内部用"camIndex查自己那份拷贝"重新构造，指针全程只在本线程
// 内部有效，不存在跨线程悬空指针的可能。
// ---------------------------------------------------------------------------
#include "estimate/FixedLagSmoother.hpp"
#include <QObject>
#include <QVector>
#include <QVector3D>
#include <QMetaType>
#include <atomic>
#include <optional>

// HandPoseState/SmootherMeasurementIn 要跨线程排队传递(QueuedConnection)，
// Qt的元对象系统要求参与这个过程的自定义类型必须注册过，否则同线程直连
// 时正常、换成跨线程排队连接后收不到信号，且不一定报错——跟 ICamera.hpp
// 顶部关于 Blob 类型注册的那段注释是同一个坑，这里预先声明好。
Q_DECLARE_METATYPE(mocap::HandPoseState)

namespace mocap {

// 跨线程传输用的观测——只带camIndex(数值)，不带CamPose指针，见文件头
// 注释的完整说明。
struct SmootherMeasurementIn {
    int camIndex = -1;
    int markerIndex = -1;
    double nx = 0.0, ny = 0.0;
    Cov2 sigma{};
};

} // namespace mocap
Q_DECLARE_METATYPE(mocap::SmootherMeasurementIn)
Q_DECLARE_METATYPE(QVector<mocap::SmootherMeasurementIn>)

namespace mocap {

class SmootherWorker : public QObject {
    Q_OBJECT
public:
    // 采集/主线程在决定要不要转发这一帧前，读这个原子标志(无锁、跨线程
    // 安全)——跟 DetectWorker::busy 同一个模式。
    std::atomic<bool> busy{false};

    explicit SmootherWorker(QObject* parent = nullptr) : QObject(parent) {}

    // 主线程在把这个worker moveToThread、线程start()之前调用一次，建立
    // 好FK+相机位姿的稳定拷贝——不要在线程已经开始处理帧之后再调这个，
    // camPosesCopy_不是线程安全地随时可替换的(处理中的窗口里的
    // HandCameraMeasurement::cam 指针指向它，替换掉会造成悬空指针，
    // 这个方法本身也不是线程安全的，只能在线程启动前的建立阶段调用)。
    void configure(ForwardKinematicsFn fk, int numJoints, const std::vector<CamPose>& camPoses,
                  const SmootherConfig& cfg) {
        fk_ = std::move(fk);
        numJoints_ = numJoints;
        camPosesCopy_ = camPoses;
        smoother_.emplace(fk_, numJoints_, cfg);
    }

public slots:
    void processFrame(qint64 ts_ns, HandPoseState initGuess, QVector<SmootherMeasurementIn> measIn) {
        busy = true;
        if (!smoother_) { busy = false; return; }

        std::vector<HandCameraMeasurement> meas;
        meas.reserve(size_t(measIn.size()));
        for (const auto& m : measIn) {
            if (m.camIndex < 0 || size_t(m.camIndex) >= camPosesCopy_.size()) continue;
            HandCameraMeasurement hm;
            hm.cam = &camPosesCopy_[size_t(m.camIndex)];   // 指向本线程自己持有的稳定拷贝，不是跨线程指针
            hm.markerIndex = m.markerIndex;
            hm.nx = m.nx; hm.ny = m.ny; hm.sigma = m.sigma;
            meas.push_back(hm);
        }

        const auto out = smoother_->pushFrame(int64_t(ts_ns), initGuess, meas);
        if (out) {
            QVector<double> rot9(9);
            for (int i=0;i<9;++i) rot9[i] = out->state.wristRot[size_t(i)];
            QVector<double> angles(int(out->state.jointAngles.size()));
            for (int i=0;i<int(out->state.jointAngles.size());++i) angles[i] = out->state.jointAngles[size_t(i)];
            emit smoothedFrameReady(
                QVector3D(float(out->state.wristPos[0]), float(out->state.wristPos[1]), float(out->state.wristPos[2])),
                rot9, angles, qint64(out->timestamp));
        }
        busy = false;
    }

    // 追踪重新冷启动/主动重置时，窗口里跨越了一次"跟丢又重新捕获"的不
    // 连续数据不该继续参与平滑(见FixedLagSmoother.hpp::reset()注释)。
    void resetWindow() { if (smoother_) smoother_->reset(); }

signals:
    void smoothedFrameReady(QVector3D wristPos, QVector<double> wristRot9, QVector<double> jointAngles, qint64 ts_ns);

private:
    ForwardKinematicsFn fk_;
    int numJoints_ = 16;
    std::vector<CamPose> camPosesCopy_;
    std::optional<FixedLagSmoother> smoother_;
};

} // namespace mocap
