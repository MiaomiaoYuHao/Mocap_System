#pragma once
// 标定入口对话框。当前阶段：
//   - 列出所有相机及其标定状态（未标定/已标定）。
//   - 支持手动填入内参(fx,fy,cx,cy,畸变)与外参(R,t)，或从 JSON 导入。
//   - 保存到 CalibrationStore（落盘，按 deviceKey）。
// 后续阶段会在此加“自动标定向导”（棋盘格/ChArUco 采集 -> 算内外参）。
#include <QDialog>
#include <QStringList>

class QListWidget;
class QStackedWidget;
class QPlainTextEdit;
class QLabel;

namespace mocap {

class CameraManager;
class CalibrationStore;

class CalibrationDialog : public QDialog {
    Q_OBJECT
public:
    CalibrationDialog(CameraManager* mgr, CalibrationStore* store, QWidget* parent = nullptr);

private slots:
    void onSelectCamera(int row);
    void applyCurrentJson();
    void importJsonFile();
    void exportJsonFile();

private:
    CameraManager*    mgr_;
    CalibrationStore* store_;
    QListWidget*      list_;
    QPlainTextEdit*   editor_;   // 当前相机标定的 JSON（可手动改/粘贴）
    QLabel*           status_;
    QStringList       keys_;     // 行 -> deviceKey

    void refreshList();
    void loadCameraToEditor(const QString& deviceKey);
};

} // namespace mocap
