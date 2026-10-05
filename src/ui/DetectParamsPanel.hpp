#pragma once
// ---------------------------------------------------------------------------
// 检测参数集中面板：阈值 / 面积下限 / 面积上限 / 圆度，四个滑块集中在一处。
// 之前这几个滑块直接堆在顶部工具栏，工具栏内容一多 Qt 会把放不下的控件
// 挤进不易发现的"»"溢出菜单里（圆度滑块就被坑过一次）。改成独立的可停靠
// 面板后，不管以后再加多少个检测参数，都不会再撑爆工具栏。
//
// 面板自己持有一份当前值（params()可读），每次任意滑块变化都会把完整的
// DetectParams 通过 paramsChanged 信号广播出去，调用方（MainWindow）只管
// 接这一个信号往所有相机分发，不用关心是哪个滑块动的。
// ---------------------------------------------------------------------------
#include "detect/CentroidDetector.hpp"
#include <QWidget>
#include <functional>

class QSlider;
class QLabel;

namespace mocap {

class DetectParamsPanel : public QWidget {
    Q_OBJECT
public:
    explicit DetectParamsPanel(const DetectParams& initial, QWidget* parent = nullptr);

    DetectParams params() const { return params_; }

signals:
    void paramsChanged(const DetectParams& p);

private:
    // 加一行"标签+滑块+数值"，slider 的原始整数值通过 toValue 转成 DetectParams
    // 里实际要用的类型（int 或 0~100 映射到 0~1 的 float），displayText 负责
    // 把这个值格式化成标签文字（比如 0 显示成"不限"/"关闭"）。
    QSlider* addRow(const QString& label, int lo, int hi, int initVal,
                    std::function<QString(int)> displayText,
                    std::function<void(DetectParams&, int)> apply);

    DetectParams params_;
};

} // namespace mocap
