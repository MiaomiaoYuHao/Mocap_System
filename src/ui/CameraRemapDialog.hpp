#pragma once
// ---------------------------------------------------------------------------
// 相机重映射对话框：解决"重插拔/换 USB 口后标定失效"的问题。
//
// deviceKey 是 Windows 设备路径，含 USB 端口信息——同一台物理相机换个口
// 重新插拔后 key 会变，标定库按 key 精确查就对不上了，表现为"模板里 4 台
// 相机，当前只有 2 台能对应上"。本对话框让用户把「当前实时相机」手动
// （或半自动）对应到「模板里已标定的相机」，映射写进 CalibrationStore 的
// remap 表（单独持久化，不污染标定数据）。映射一次后，之后插同样的口能
// 自动记住。
//
// 交互设计：
//   - 左列：模板里每一台已标定的相机（旧 key，显示 VID/PID 短标签 + 别名）。
//   - 右列：一个下拉框，选"当前哪台实时相机对应它"。
//   - 打开时自动按 VID/PID 预匹配：同型号相机 VID/PID 相同，能自动配上
//     大部分；只有"同型号多台"（VID/PID 完全一样、分不出谁是谁）时才需要
//     人工区分，这时用每行的「看画面」按钮弹出实时预览，对着相机挥挥手
//     确认是哪一台。
//   - 确定后把映射写入 store，三角化立刻按新映射取标定，不用重标。
// ---------------------------------------------------------------------------
#include <QDialog>
#include <QVector>
#include <QString>
#include <QHash>

class QComboBox;
class QLabel;

namespace mocap {

class CalibrationStore;
class ICamera;
class CameraCalibration;

class CameraRemapDialog : public QDialog {
    Q_OBJECT
public:
    // liveCams：当前接入的实时相机；templateKeys：模板里已标定相机的
    // deviceKey 列表（旧 key）。store 用于读别名/写 remap，生命周期需比
    // 本对话框长。
    CameraRemapDialog(const QVector<ICamera*>& liveCams,
                      const QVector<QString>& templateKeys,
                      CalibrationStore* store,
                      QWidget* parent = nullptr);

private slots:
    void onAccept();
    void onAutoMatch();       // 重新跑一次 VID/PID 自动预匹配
    void onPeek(int rowIdx);  // 「看画面」：弹实时预览确认是哪台相机

private:
    struct Row {
        QString  oldKey;       // 模板里的旧 key（已标定）
        QLabel*  leftLabel;    // 显示这台标定相机的短标签/别名
        QComboBox* combo;      // 选当前哪台实时相机对应它
    };

    // deviceKey -> "USB相机 VID:PID"短标签（复用 CalibrationLibraryDialog
    // 里同款提取逻辑）。alias 非空时优先显示 alias。
    QString shortLabel(const QString& deviceKey, const QString& alias = QString()) const;
    // 提取 VID:PID，用于自动预匹配；提取不到返回空串。
    QString vidPid(const QString& deviceKey) const;
    void buildRows();
    void applyAutoMatch();

    QVector<ICamera*> liveCams_;
    QVector<QString>  templateKeys_;
    CalibrationStore* store_;
    QVector<Row>      rows_;
};

} // namespace mocap
