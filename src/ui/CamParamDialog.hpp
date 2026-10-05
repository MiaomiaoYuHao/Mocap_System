#pragma once
// ---------------------------------------------------------------------------
// 相机属性对话框（AMCap 同款布局）：
//   页签1 “视频 Proc Amp”：亮度/对比度/色调/饱和度/清晰度/伽玛/启用颜色/
//                          白平衡/逆光对比/增益
//   页签2 “照相机控制”  ：全景/倾斜/滚动/缩放/曝光/光圈/焦点
//   每行 = 滑条 + 数值框 + “自动”勾选；不支持的属性置灰；“默认值”一键恢复。
//   页签3 “格式”        ：分辨率/帧率选择（= AMCap 的 Video Capture Pin）。
// 修改即时生效（比 AMCap 的“应用”更直接）。
// ---------------------------------------------------------------------------
#include "camera/DShowControl.hpp"
#include <QDialog>
#include <QVector>
#include <QCameraFormat>

class QSlider;
class QSpinBox;
class QCheckBox;
class QComboBox;
class QLabel;

namespace mocap {

class ICamera;
class WebcamCamera;

class CamParamDialog : public QDialog {
    Q_OBJECT
public:
    explicit CamParamDialog(ICamera* cam, QWidget* parent = nullptr);
    ~CamParamDialog() override;

private:
    struct Row {
        int propIndex = -1;
        QSlider*  slider = nullptr;
        QSpinBox* box    = nullptr;
        QCheckBox* autoC = nullptr;
    };

    WebcamCamera* web_ = nullptr;
    DShowControl* ctrl_ = nullptr;
    QVector<DShowProp> props_;
    QVector<Row> rows_;

    // 格式页：分辨率与帧率两个下拉联动，数据都来自驱动实际上报的 formats()。
    QComboBox* resCombo_ = nullptr;
    QComboBox* fpsCombo_ = nullptr;
    QList<QCameraFormat> allFormats_;
    // 应用格式后，实际生效的格式回读显示——不假设"下拉框选了120就一定
    // 交付120"，格式协商是异步的(WebcamCamera::setFormat通过Qt::QueuedConnection
    // 投到相机线程)，也可能因为驱动/带宽原因悄悄落到另一条格式，所以点了
    // "应用格式"之后单独读一次 web_->currentFormat() 摆出来，而不是直接信
    // 下拉框此刻显示的文字。
    QLabel* formatReadbackLabel_ = nullptr;
    // 「选的帧率能不能真的跑到」的提示。空 = 能，不打扰。
    // 见 .cpp 的 noteRateFeasibility()：区间内指定帧率只有 DirectShow 引擎
    // 兑现得了，未压缩格式走 Qt 路径时请求会被无声丢弃 —— 必须说出来。
    QLabel* rateHintLabel_ = nullptr;

    // 【调试用】"帧率"下拉框是按数字去重过的，看不出同一个数字背后可能
    // 对应驱动上报的多条不同格式(比如同为120fps，一条压缩一条未压缩)。
    // 这个下拉框把当前分辨率下驱动原始上报的每一条格式都摆出来，不做任何
    // "最接近"的猜测匹配——选哪条就应用哪条，方便直接对比压缩/未压缩两种
    // 格式实际交付的帧率差异。
    QComboBox* exactFormatCombo_ = nullptr;
    void populateExactFormatsForCurrentRes();
    void applyExactFormatSelection();
    void dumpAllFormatsToLog();
    // 应用格式(不管走哪条路径：快速/调试强制指定)之后，延迟读回真正生效
    // 的格式并更新 formatReadbackLabel_——两处应用逻辑共用同一份，避免
    // 各写一份、以后行为渐渐不一致。
    void scheduleFormatReadback();
    // 实际执行重试的那一步——DirectShow引擎建图可能比预期慢，单次
    // 延迟不一定够，失败后按固定间隔再试几次，见 scheduleFormatReadback()
    // 注释。attempt 从0开始数，达到上限还读不到才真正报"失败"。
    void tryReadbackFormat(int attempt);

    void populateFpsForCurrentRes();
    void applyCurrentFormatSelection();
    void noteRateFeasibility(const QCameraFormat& f, double wantFps);

    QWidget* buildPropTab(bool cameraControl);
    QWidget* buildFormatTab();
    void applyRow(const Row& r);
    void setRowUi(const Row& r);
};

} // namespace mocap
