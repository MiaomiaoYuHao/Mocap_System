#pragma once
// 标定模板库浏览器：左侧树（模板 > 相机），可重命名/设为当前使用/删除
// 单台相机/删除整个模板；删除操作都进 QUndoStack，Ctrl+Z 撤销。
// 下方复用 TriangulationView 做 3D 预览，选中哪个模板就显示哪个模板的
// 相机布局。
#include "calib/CalibrationLibrary.hpp"
#include <QDialog>

class QTreeWidget;
class QTreeWidgetItem;
class QLabel;
class QUndoStack;

namespace mocap {

class TriangulationView;

class CalibrationLibraryDialog : public QDialog {
    Q_OBJECT
public:
    explicit CalibrationLibraryDialog(CalibrationLibrary* lib, QWidget* parent = nullptr);
    ~CalibrationLibraryDialog() override;

private slots:
    void refreshTree();
    void onSelectionChanged();
    void onRenameTemplate();
    void onRenameCamera();
    void onSetActive();
    void onDeleteCamera();
    void onDeleteTemplate();

private:
    // 当前选中项属于哪个模板（相机行返回其父模板id，模板行返回自身id）。
    QString selectedTemplateId() const;
    // 若当前选中的是相机行，返回其 deviceKey；否则返回空字符串。
    QString selectedCameraKey() const;

    CalibrationLibrary* lib_;
    QTreeWidget* tree_;
    QLabel* status_;
    TriangulationView* view3d_;
    QUndoStack* undo_;
};

} // namespace mocap
