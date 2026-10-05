#pragma once
// ---------------------------------------------------------------------------
// 手部追踪参数实时调节面板——每个滑块直接调用 HandTrackingWorker 已有的
// setter，不缓存自己的一份状态，避免"面板显示的值"和"worker实际在用的值"
// 不同步。构造时从 worker 当前值反读一遍初始化滑块位置，之后单向：
// 拖滑块 -> 调用 worker 的 setter，不做反向绑定(worker 不会在没有 UI
// 交互的情况下自己改这些值，不需要面板监听 worker 的变化)。
//
// 覆盖的参数(全部对应 HandTrackingWorker.hpp 已有的 public setter)：
//   - 球物理半径(mm)——量出来大致就不用怎么改，给个粗调滑块。
//   - 标称工作距离(mm)——现在只是"两遍流程第一遍都没匹配上任何候选点"
//     时的兜底假设，不再是常态路径，但保留可调，用于兜底场景也不至于
//     离谱。
//   - 粗算(coarse)两视图匹配阈值——第一遍radius-free匹配，质心噪声大，
//     默认比精修阈值松；调这个主要是看"粗算候选点数量"这个诊断量有没有
//     反应，判断是不是这一步本身先失败了。
//   - 精修(refined)两视图匹配阈值——旧的 clusterMaxSampson/ReprojNorm，
//     现在是两遍流程里第二遍(用反推出的真实半径精修后)的最终匹配阈值。
//   - 深度反推匹配门控——决定一个blob离粗算候选点多远就不采信那个候选
//     点的深度，太松会误认领风马牛不相及的候选点，太紧会导致大量blob
//     退回兜底假设、白做了两遍流程。
//
// 【无法在当前环境编译验证】依赖 Qt6 Widgets，构建这份回答的沙箱没有装
// Qt6，请在本地实际编译一遍。没有拿到项目里 DetectParamsPanel.cpp 的
// 具体样式，这里是一个独立的、自成一体的面板，跟现有面板风格如果对不上
// 请按你项目的既有风格调整——这不影响它跟 HandTrackingWorker 的接线是否
// 正确，样式是纯外观问题。
// ---------------------------------------------------------------------------
#include "estimate/HandTrackingWorker.hpp"
#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QString>
#include <cmath>

namespace mocap {

class HandTrackingParamsPanel : public QWidget {
    Q_OBJECT
public:
    // worker 的生命周期必须比这个面板长(典型用法：HandPoseDebugDialog
    // 持有 worker 指针，把它传给这个面板，面板作为 dialog 的子部件，
    // dialog 关闭时两者一起析构，不存在 worker 先于面板销毁的情况——
    // 如果你的用法不是这样，需要在 worker 销毁前手动把这个面板也销毁
    // 或者置空内部指针，不然滑块拖动时会访问悬空指针)。
    explicit HandTrackingParamsPanel(HandTrackingWorker* worker, QWidget* parent = nullptr)
        : QWidget(parent), worker_(worker) {
        auto* root = new QVBoxLayout(this);

        root->addWidget(buildTrackingModeGroup());
        root->addWidget(buildLocalizationModeGroup());
        root->addWidget(buildMarkerGeometryGroup());
        root->addWidget(buildCoarseThresholdGroup());
        root->addWidget(buildRefinedThresholdGroup());
        root->addWidget(buildRadiusGateGroup());
        root->addWidget(buildProcessNoiseGroup());
        root->addWidget(buildAssocGateGroup());
        root->addWidget(buildSmoothingGroup());
        root->addWidget(buildFixedLagSmootherGroup());
        root->addStretch(1);
    }

private:
    // 滑块用整数刻度，实际值按 scale 缩放成小数——QSlider 本身只支持整数，
    // 这是 Qt 里最常见的绕法，不引入额外的第三方浮点滑块控件。
    struct ScaledSlider {
        QSlider* slider;
        QLabel* valueLabel;
        double scale;
        QString suffix;
    };

    ScaledSlider addScaledRow(QGridLayout* grid, int row, const QString& labelText,
                              double minVal, double maxVal, double initVal, double scale,
                              const QString& suffix) {
        auto* label = new QLabel(labelText, this);
        auto* slider = new QSlider(Qt::Horizontal, this);
        slider->setMinimum(int(std::round(minVal * scale)));
        slider->setMaximum(int(std::round(maxVal * scale)));
        slider->setValue(int(std::round(initVal * scale)));
        auto* valueLabel = new QLabel(this);
        valueLabel->setMinimumWidth(70);
        valueLabel->setText(formatValue(initVal, suffix));

        grid->addWidget(label, row, 0);
        grid->addWidget(slider, row, 1);
        grid->addWidget(valueLabel, row, 2);

        return { slider, valueLabel, scale, suffix };
    }

    static QString formatValue(double v, const QString& suffix) {
        return QStringLiteral("%1%2").arg(v, 0, 'f', (suffix == "px" || suffix == "mm") ? 1 : 4).arg(suffix);
    }

    QGroupBox* buildTrackingModeGroup() {
        auto* box = new QGroupBox(QStringLiteral("追踪模式"), this);
        auto* layout = new QVBoxLayout(box);

        auto* combo = new QComboBox(this);
        combo->addItem(QStringLiteral("完整手部(手背+16维手指关节角)"));
        combo->addItem(QStringLiteral("仅手背刚体(测试手背用，完全屏蔽手指干扰)"));
        const auto curMode = worker_ ? worker_->trackingMode() : TrackingMode::FullHand;
        combo->setCurrentIndex(curMode == TrackingMode::BackRigidOnly ? 1 : 0);
        connect(combo, &QComboBox::currentIndexChanged, this, [this](int idx) {
            if (!worker_) return;
            worker_->setTrackingMode(idx == 1 ? TrackingMode::BackRigidOnly : TrackingMode::FullHand);
        });
        layout->addWidget(combo);

        auto* hint = new QLabel(QStringLiteral(
            "手指结构还没标定/只想先验证手背这条链路时选第二项——每帧只对"
            "手背5点做距离匹配+Kabsch，完全不涉及手指关节角估计，骨架图会"
            "显示一个安静的伸直手型，不会出现手指乱跳的情况。"), this);
        hint->setWordWrap(true);
        layout->addWidget(hint);

        return box;
    }

    QGroupBox* buildLocalizationModeGroup() {
        auto* box = new QGroupBox(QStringLiteral("2D定位方式(每个blob怎么算中心点)"), this);
        auto* layout = new QVBoxLayout(box);

        auto* combo = new QComboBox(this);
        combo->addItem(QStringLiteral("质心法(快，遮挡时会偏，画面干净时最稳)"));
        combo->addItem(QStringLiteral("圆拟合法(抗遮挡，依赖已知半径先验)"));
        combo->addItem(QStringLiteral("融合(圆拟合可用就融合，弧太短自动退回质心)"));
        const auto curMode = worker_ ? worker_->localizationMode() : BlobLocalizationMode::CircleFitOnly;
        combo->setCurrentIndex(curMode == BlobLocalizationMode::CentroidOnly ? 0
                              : curMode == BlobLocalizationMode::CircleFitOnly ? 1 : 2);
        connect(combo, &QComboBox::currentIndexChanged, this, [this](int idx) {
            if (!worker_) return;
            const BlobLocalizationMode m = idx==0 ? BlobLocalizationMode::CentroidOnly
                                          : idx==1 ? BlobLocalizationMode::CircleFitOnly
                                                    : BlobLocalizationMode::Fused;
            worker_->setLocalizationMode(m);
        });
        layout->addWidget(combo);

        auto* grid = new QGridLayout();
        const double curSigma = worker_ ? worker_->centroidSigma() : 0.3;
        auto sigmaRow = addScaledRow(grid, 0, QStringLiteral("质心噪声假设(px)"), 0.05, 2.0, curSigma, 1000.0, QStringLiteral("px"));
        connect(sigmaRow.slider, &QSlider::valueChanged, this, [this, sigmaRow](int v) {
            const double val = double(v) / sigmaRow.scale;
            sigmaRow.valueLabel->setText(formatValue(val, sigmaRow.suffix));
            if (worker_) worker_->setCentroidSigma(val);
        });
        layout->addLayout(grid);

        return box;
    }

    QGroupBox* buildMarkerGeometryGroup() {
        auto* box = new QGroupBox(QStringLiteral("Marker 几何"), this);
        auto* grid = new QGridLayout(box);

        const double curRadius = worker_ ? worker_->physicalMarkerRadiusMm() : 4.5;
        const double curDist   = worker_ ? worker_->nominalWorkingDistanceMm() : 250.0;

        // 半径滑块：1.0~15.0mm，0.1mm粒度(scale=10)——覆盖常见反光球尺寸
        // (你的9mm直径/4.5mm半径球在这个范围正中间)。
        auto radiusRow = addScaledRow(grid, 0, QStringLiteral("球物理半径"), 1.0, 15.0, curRadius, 10.0, QStringLiteral("mm"));
        connect(radiusRow.slider, &QSlider::valueChanged, this, [this, radiusRow](int v) {
            const double mm = double(v) / radiusRow.scale;
            radiusRow.valueLabel->setText(formatValue(mm, radiusRow.suffix));
            if (worker_) worker_->setMarkerGeometry(mm, worker_->nominalWorkingDistanceMm());
        });

        // 标称距离滑块：50~1000mm，1mm粒度——现在只是兜底假设，范围给宽
        // 一点没关系，不再是决定成败的常态路径。
        auto distRow = addScaledRow(grid, 1, QStringLiteral("标称工作距离(兜底)"), 50.0, 1000.0, curDist, 1.0, QStringLiteral("mm"));
        connect(distRow.slider, &QSlider::valueChanged, this, [this, distRow](int v) {
            const double mm = double(v) / distRow.scale;
            distRow.valueLabel->setText(formatValue(mm, distRow.suffix));
            if (worker_) worker_->setMarkerGeometry(worker_->physicalMarkerRadiusMm(), mm);
        });

        return box;
    }

    QGroupBox* buildCoarseThresholdGroup() {
        auto* box = new QGroupBox(QStringLiteral("第一遍·粗算深度阈值(质心匹配，不需要知道半径)"), this);
        auto* grid = new QGridLayout(box);

        const double curSampson = worker_ ? worker_->coarseMaxSampson() : 0.02;
        const double curReproj  = worker_ ? worker_->coarseMaxReprojNorm() : 0.02;

        // 0.001~0.10，scale=10000，粒度0.0001——归一化坐标下的阈值本来就是
        // 小数，这个粒度足够精细地调。
        auto sampsonRow = addScaledRow(grid, 0, QStringLiteral("maxSampson"), 0.001, 0.10, curSampson, 10000.0, QStringLiteral(""));
        auto reprojRow  = addScaledRow(grid, 1, QStringLiteral("maxReprojNorm"), 0.001, 0.10, curReproj, 10000.0, QStringLiteral(""));

        connect(sampsonRow.slider, &QSlider::valueChanged, this, [this, sampsonRow, reprojRow](int v) {
            const double val = double(v) / sampsonRow.scale;
            sampsonRow.valueLabel->setText(formatValue(val, QStringLiteral("")));
            if (worker_) worker_->setCoarseClusterThresholds(val, double(reprojRow.slider->value()) / reprojRow.scale);
        });
        connect(reprojRow.slider, &QSlider::valueChanged, this, [this, sampsonRow, reprojRow](int v) {
            const double val = double(v) / reprojRow.scale;
            reprojRow.valueLabel->setText(formatValue(val, QStringLiteral("")));
            if (worker_) worker_->setCoarseClusterThresholds(double(sampsonRow.slider->value()) / sampsonRow.scale, val);
        });

        return box;
    }

    QGroupBox* buildRefinedThresholdGroup() {
        auto* box = new QGroupBox(QStringLiteral("第二遍·精修最终匹配阈值(反推真实半径重新拟合后)"), this);
        auto* grid = new QGridLayout(box);

        const double curSampson = worker_ ? worker_->refinedMaxSampson() : 0.01;
        const double curReproj  = worker_ ? worker_->refinedMaxReprojNorm() : 0.01;

        auto sampsonRow = addScaledRow(grid, 0, QStringLiteral("maxSampson"), 0.001, 0.10, curSampson, 10000.0, QStringLiteral(""));
        auto reprojRow  = addScaledRow(grid, 1, QStringLiteral("maxReprojNorm"), 0.001, 0.10, curReproj, 10000.0, QStringLiteral(""));

        connect(sampsonRow.slider, &QSlider::valueChanged, this, [this, sampsonRow, reprojRow](int v) {
            const double val = double(v) / sampsonRow.scale;
            sampsonRow.valueLabel->setText(formatValue(val, QStringLiteral("")));
            if (worker_) worker_->setRefinedClusterThresholds(val, double(reprojRow.slider->value()) / reprojRow.scale);
        });
        connect(reprojRow.slider, &QSlider::valueChanged, this, [this, sampsonRow, reprojRow](int v) {
            const double val = double(v) / reprojRow.scale;
            reprojRow.valueLabel->setText(formatValue(val, QStringLiteral("")));
            if (worker_) worker_->setRefinedClusterThresholds(double(sampsonRow.slider->value()) / sampsonRow.scale, val);
        });

        return box;
    }

    QGroupBox* buildRadiusGateGroup() {
        auto* box = new QGroupBox(QStringLiteral("深度反推·候选点匹配门控"), this);
        auto* grid = new QGridLayout(box);

        const double curGate = worker_ ? worker_->radiusRefineMatchGate() : 0.03;
        auto gateRow = addScaledRow(grid, 0, QStringLiteral("maxMatchNorm"), 0.005, 0.15, curGate, 10000.0, QStringLiteral(""));
        connect(gateRow.slider, &QSlider::valueChanged, this, [this, gateRow](int v) {
            const double val = double(v) / gateRow.scale;
            gateRow.valueLabel->setText(formatValue(val, QStringLiteral("")));
            if (worker_) worker_->setRadiusRefineMatchGate(val);
        });

        return box;
    }

    QGroupBox* buildProcessNoiseGroup() {
        auto* box = new QGroupBox(QStringLiteral("IEKF过程噪声(抖动/跟手速度的主要旋钮)"), this);
        auto* grid = new QGridLayout(box);

        const double curPos = worker_ ? worker_->posProcessVar() : 0.5;
        const double curRot = worker_ ? worker_->rotProcessVar() : 1e-4;
        const double curJoint = worker_ ? worker_->jointProcessVar() : 1e-4;

        // 位置过程噪声：0.01~5.0，scale=1000。手静止时抖得厉害，先把这个
        // 往下拖，越小越平滑但跟手实际动作的滞后感越明显。
        auto posRow = addScaledRow(grid, 0, QStringLiteral("posProcessVar"), 0.01, 5.0, curPos, 1000.0, QStringLiteral(""));
        // 旋转/关节角过程噪声通常比位置小几个数量级，范围单独给，避免
        // 一个滑块覆盖1e-6~5这种跨度导致中间大部分刻度都挤在一起没法调。
        auto rotRow = addScaledRow(grid, 1, QStringLiteral("rotProcessVar"), 1e-6, 0.01, curRot, 1000000.0, QStringLiteral(""));
        auto jointRow = addScaledRow(grid, 2, QStringLiteral("jointProcessVar"), 1e-6, 0.01, curJoint, 1000000.0, QStringLiteral(""));

        auto applyAll = [this, posRow, rotRow, jointRow]() {
            if (!worker_) return;
            worker_->setProcessNoise(double(posRow.slider->value())/posRow.scale,
                                     double(rotRow.slider->value())/rotRow.scale,
                                     double(jointRow.slider->value())/jointRow.scale);
        };
        connect(posRow.slider, &QSlider::valueChanged, this, [posRow, applyAll](int v) {
            posRow.valueLabel->setText(formatValue(double(v)/posRow.scale, QStringLiteral(""))); applyAll();
        });
        connect(rotRow.slider, &QSlider::valueChanged, this, [rotRow, applyAll](int v) {
            rotRow.valueLabel->setText(formatValue(double(v)/rotRow.scale, QStringLiteral(""))); applyAll();
        });
        connect(jointRow.slider, &QSlider::valueChanged, this, [jointRow, applyAll](int v) {
            jointRow.valueLabel->setText(formatValue(double(v)/jointRow.scale, QStringLiteral(""))); applyAll();
        });

        return box;
    }

    QGroupBox* buildAssocGateGroup() {
        auto* box = new QGroupBox(QStringLiteral("Marker关联门控(自适应马氏距离+硬性欧氏上限兜底)"), this);
        auto* layout = new QVBoxLayout(box);

        auto* grid = new QGridLayout();
        const double curGate = worker_ ? worker_->gateRadius() : 0.05;
        auto gateRow = addScaledRow(grid, 0, QStringLiteral("gateRadiusNorm(硬性欧氏上限)"), 0.01, 0.20, curGate, 10000.0, QStringLiteral(""));

        // 马氏距离卡方阈值——这是自适应门控真正生效的旋钮，gateRadiusNorm
        // 只是兜底上限。数值越大越宽松，默认9.21是二维卡方分布99%置信
        // 区间(专业动捕系统常用的量级，不是随手给的数)。
        const double curChiSq = worker_ ? worker_->chiSquareGate() : 9.21;
        auto chiSqRow = addScaledRow(grid, 1, QStringLiteral("chiSquareGate(马氏距离阈值)"), 1.0, 30.0, curChiSq, 100.0, QStringLiteral(""));

        // 注意：这两个参数改了都会整个重建追踪pipeline、丢失当前追踪
        // 状态、下一帧重新冷启动(见 HandTrackingWorker::setGateRadius/
        // setChiSquareGate 注释)——用 sliderReleased 而不是 valueChanged，
        // 拖动过程中不会每一帧都触发重建，松手才真正应用一次。
        connect(gateRow.slider, &QSlider::valueChanged, this, [gateRow](int v) {
            gateRow.valueLabel->setText(formatValue(double(v)/gateRow.scale, QStringLiteral("")));
        });
        connect(gateRow.slider, &QSlider::sliderReleased, this, [this, gateRow]() {
            if (worker_) worker_->setGateRadius(double(gateRow.slider->value())/gateRow.scale);
        });
        connect(chiSqRow.slider, &QSlider::valueChanged, this, [chiSqRow](int v) {
            chiSqRow.valueLabel->setText(formatValue(double(v)/chiSqRow.scale, QStringLiteral("")));
        });
        connect(chiSqRow.slider, &QSlider::sliderReleased, this, [this, chiSqRow]() {
            if (worker_) worker_->setChiSquareGate(double(chiSqRow.slider->value())/chiSqRow.scale);
        });
        layout->addLayout(grid);

        // 自动推算：不需要额外采集数据，直接从已标定的手部模板几何算出
        // 一个跟这只手真正匹配的建议值——见
        // estimate/GateRadiusEstimator.hpp 顶部完整分析。点击即计算+应用
        // (这本身就是一次显式的用户确认动作，不需要再额外弹确认)。
        auto* autoBtn = new QPushButton(QStringLiteral("按当前手部模板自动推算(需先完成手背+手指标定)"), this);
        auto* autoResultLabel = new QLabel(QStringLiteral("(尚未推算)"), this);
        autoResultLabel->setWordWrap(true);
        connect(autoBtn, &QPushButton::clicked, this, [this, gateRow, autoResultLabel]() {
            if (!worker_) return;
            const double workingDist = worker_->nominalWorkingDistanceMm();
            const auto est = worker_->estimateGateRadiusFromTemplate(workingDist);
            if (!est.valid) {
                autoResultLabel->setText(QStringLiteral("<font color='#e05050'>推算失败：%1</font>").arg(QString::fromStdString(est.message)));
                return;
            }
            autoResultLabel->setText(QString::fromStdString(est.message));
            gateRow.slider->setValue(int(std::round(est.recommendedGateRadiusNorm * gateRow.scale)));
            worker_->setGateRadius(est.recommendedGateRadiusNorm);
        });
        layout->addWidget(autoBtn);
        layout->addWidget(autoResultLabel);

        return box;
    }

    QGroupBox* buildSmoothingGroup() {
        auto* box = new QGroupBox(QStringLiteral("输出平滑(One Euro Filter，真正压制静止时的抖动)"), this);
        auto* grid = new QGridLayout(box);

        const auto curPos = worker_ ? worker_->posSmoothingParams() : std::array<double,2>{1.0,0.3};
        const auto curRot = worker_ ? worker_->rotSmoothingParams() : std::array<double,2>{1.0,0.3};
        const auto curJoint = worker_ ? worker_->jointSmoothingParams() : std::array<double,2>{1.0,0.3};

        // minCutoff: 0.05~3.0(越小越平滑，静止时抖动小的方向调这个)；
        // beta: 0~2.0(越大越跟手，快速运动时的抗延迟能力，抖动主要在
        // 快速挥动时出现才调这个，静止抖动跟beta关系不大)。
        auto posCutRow = addScaledRow(grid, 0, QStringLiteral("位置 minCutoff"), 0.05, 3.0, curPos[0], 1000.0, QStringLiteral(""));
        auto posBetaRow = addScaledRow(grid, 1, QStringLiteral("位置 beta"), 0.0, 2.0, curPos[1], 1000.0, QStringLiteral(""));
        auto rotCutRow = addScaledRow(grid, 2, QStringLiteral("旋转 minCutoff"), 0.05, 3.0, curRot[0], 1000.0, QStringLiteral(""));
        auto rotBetaRow = addScaledRow(grid, 3, QStringLiteral("旋转 beta"), 0.0, 2.0, curRot[1], 1000.0, QStringLiteral(""));
        auto jointCutRow = addScaledRow(grid, 4, QStringLiteral("关节角 minCutoff"), 0.05, 3.0, curJoint[0], 1000.0, QStringLiteral(""));
        auto jointBetaRow = addScaledRow(grid, 5, QStringLiteral("关节角 beta"), 0.0, 2.0, curJoint[1], 1000.0, QStringLiteral(""));

        auto applyAll = [this, posCutRow, posBetaRow, rotCutRow, rotBetaRow, jointCutRow, jointBetaRow]() {
            if (!worker_) return;
            worker_->setSmoothingParams(
                double(posCutRow.slider->value())/posCutRow.scale, double(posBetaRow.slider->value())/posBetaRow.scale,
                double(rotCutRow.slider->value())/rotCutRow.scale, double(rotBetaRow.slider->value())/rotBetaRow.scale,
                double(jointCutRow.slider->value())/jointCutRow.scale, double(jointBetaRow.slider->value())/jointBetaRow.scale);
        };
        // 逐个显式connect、按值捕获各自的ScaledSlider——不能像上面那样
        // 拿局部变量的地址塞进一个循环里连接(那几个ScaledSlider是这个
        // 函数的栈上局部变量，函数返回后就没了，之后信号触发时lambda
        // 里的指针会是悬空的，是真实bug不是风格问题，这里刻意按值拷贝
        // 一份进每个lambda，生命周期跟连接本身绑定，没有这个风险)。
        connect(posCutRow.slider, &QSlider::valueChanged, this, [posCutRow, applyAll](int v) {
            posCutRow.valueLabel->setText(formatValue(double(v)/posCutRow.scale, QStringLiteral(""))); applyAll();
        });
        connect(posBetaRow.slider, &QSlider::valueChanged, this, [posBetaRow, applyAll](int v) {
            posBetaRow.valueLabel->setText(formatValue(double(v)/posBetaRow.scale, QStringLiteral(""))); applyAll();
        });
        connect(rotCutRow.slider, &QSlider::valueChanged, this, [rotCutRow, applyAll](int v) {
            rotCutRow.valueLabel->setText(formatValue(double(v)/rotCutRow.scale, QStringLiteral(""))); applyAll();
        });
        connect(rotBetaRow.slider, &QSlider::valueChanged, this, [rotBetaRow, applyAll](int v) {
            rotBetaRow.valueLabel->setText(formatValue(double(v)/rotBetaRow.scale, QStringLiteral(""))); applyAll();
        });
        connect(jointCutRow.slider, &QSlider::valueChanged, this, [jointCutRow, applyAll](int v) {
            jointCutRow.valueLabel->setText(formatValue(double(v)/jointCutRow.scale, QStringLiteral(""))); applyAll();
        });
        connect(jointBetaRow.slider, &QSlider::valueChanged, this, [jointBetaRow, applyAll](int v) {
            jointBetaRow.valueLabel->setText(formatValue(double(v)/jointBetaRow.scale, QStringLiteral(""))); applyAll();
        });

        return box;
    }

    QGroupBox* buildFixedLagSmootherGroup() {
        auto* box = new QGroupBox(QStringLiteral("延迟精修流(实验性，独立于实时输出，另开线程跑)"), this);
        auto* layout = new QVBoxLayout(box);

        auto* hint = new QLabel(QStringLiteral(
            "独立于实时追踪的处理流：多帧联合优化，用固定延迟换精度。默认关闭。"), this);
        hint->setWordWrap(true);
        layout->addWidget(hint);

        auto* enableBox = new QCheckBox(QStringLiteral("启用延迟精修流"), this);
        enableBox->setChecked(worker_ && worker_->smootherEnabled());
        layout->addWidget(enableBox);

        auto* grid = new QGridLayout();
        const int curWindow = worker_ ? worker_->smootherConfig().windowSize : 7;
        auto windowRow = addScaledRow(grid, 0, QStringLiteral("窗口帧数"), 2, 15, double(curWindow), 1.0, QStringLiteral("帧"));
        layout->addLayout(grid);

        auto* latencyLabel = new QLabel(this);
        auto updateLatencyLabel = [this, latencyLabel](int windowFrames) {
            const double ms = worker_ ? worker_->estimatedSmootherLatencyMs() : (windowFrames * 1000.0 / 125.0);
            latencyLabel->setText(QStringLiteral("按125fps估算，当前窗口大约带来 %1 ms 延迟(按你实际帧率会有出入，仅供参考)")
                .arg(ms, 0, 'f', 0));
        };
        updateLatencyLabel(curWindow);
        layout->addWidget(latencyLabel);

        connect(windowRow.slider, &QSlider::valueChanged, this, [windowRow, updateLatencyLabel](int v) {
            const double val = double(v) / windowRow.scale;
            windowRow.valueLabel->setText(formatValue(val, windowRow.suffix));
            updateLatencyLabel(int(val));
        });
        // 窗口大小改了要重建这条流的线程(见 setSmootherConfig 注释)，用
        // sliderReleased 而不是 valueChanged——跟门控半径那几个参数同一个
        // 道理，拖动过程中不用每一帧都重建一次线程。
        connect(windowRow.slider, &QSlider::sliderReleased, this, [this, windowRow]() {
            if (!worker_) return;
            SmootherConfig cfg = worker_->smootherConfig();
            cfg.windowSize = int(std::round(double(windowRow.slider->value()) / windowRow.scale));
            worker_->setSmootherConfig(cfg);
        });

        connect(enableBox, &QCheckBox::toggled, this, [this](bool on) {
            if (worker_) worker_->setSmootherEnabled(on);
        });

        return box;
    }

    HandTrackingWorker* worker_ = nullptr;
};

} // namespace mocap
