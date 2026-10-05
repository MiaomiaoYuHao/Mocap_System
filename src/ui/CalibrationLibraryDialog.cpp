#include "ui/CalibrationLibraryDialog.hpp"
#include "ui/TriangulationView.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTreeWidget>
#include <QLabel>
#include <QPushButton>
#include <QUndoStack>
#include <QUndoCommand>
#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QKeySequence>
#include <QRegularExpression>

namespace mocap {

namespace {

// 没设置别名时的默认显示：从 deviceKey（Windows 设备路径）里提取 VID/PID
// 拼成"USB相机 XXXX:XXXX"这种短标签，跟 CalibResultView 里同样的思路一致，
// 只是这里独立实现一份（两处都只是纯显示逻辑，没有共享状态，没必要为了
// 一个小函数去改公共头文件、引入新的耦合）。
QString shortLabel(const CameraCalibration& c) {
    if (!c.alias.isEmpty()) return c.alias;
    static const QRegularExpression reVid("vid_([0-9a-fA-F]{4})");
    static const QRegularExpression rePid("pid_([0-9a-fA-F]{4})");
    const auto mv = reVid.match(c.deviceKey);
    const auto mp = rePid.match(c.deviceKey);
    if (mv.hasMatch() && mp.hasMatch())
        return QStringLiteral("USB相机 %1:%2").arg(mv.captured(1).toUpper(), mp.captured(1).toUpper());
    return c.deviceKey;   // 解析不出就老实显示原始值
}

} // namespace

namespace {

// ---- 撤销命令：都遵循"push() 时才真正执行"的 QUndoCommand 标准用法，
// 调用方在 push 之前只负责"读快照"，不提前做真正的删除。 ----

class DeleteCameraCommand : public QUndoCommand {
public:
    DeleteCameraCommand(CalibrationLibrary* lib, QString tplId, CameraCalibration removed)
        : QUndoCommand(QStringLiteral("删除相机标定：%1").arg(removed.deviceKey)),
          lib_(lib), tplId_(std::move(tplId)), cam_(std::move(removed)) {}
    void redo() override { lib_->removeCameraFromTemplate(tplId_, cam_.deviceKey); }
    void undo() override { lib_->addCameraToTemplate(tplId_, cam_); }
private:
    CalibrationLibrary* lib_;
    QString tplId_;
    CameraCalibration cam_;
};

class DeleteTemplateCommand : public QUndoCommand {
public:
    DeleteTemplateCommand(CalibrationLibrary* lib, TemplateInfo meta,
                          QVector<CameraCalibration> cams, bool wasActive)
        : QUndoCommand(QStringLiteral("删除标定模板：%1").arg(meta.displayName)),
          lib_(lib), meta_(std::move(meta)), cams_(std::move(cams)), wasActive_(wasActive) {}
    void redo() override { lib_->deleteTemplate(meta_.id); }
    void undo() override {
        lib_->restoreTemplate(meta_, cams_);
        if (wasActive_) lib_->setActiveTemplateId(meta_.id);
    }
private:
    CalibrationLibrary* lib_;
    TemplateInfo meta_;
    QVector<CameraCalibration> cams_;
    bool wasActive_;
};

} // namespace

CalibrationLibraryDialog::CalibrationLibraryDialog(CalibrationLibrary* lib, QWidget* parent)
    : QDialog(parent), lib_(lib) {
    setWindowTitle(QStringLiteral("标定模板库"));
    resize(820, 720);

    undo_ = new QUndoStack(this);
    QAction* undoAction = undo_->createUndoAction(this, QStringLiteral("撤销"));
    undoAction->setShortcut(QKeySequence::Undo);   // Ctrl+Z
    addAction(undoAction);
    // 关键修复：撤销/重做发生时（不管是按 Ctrl+Z、还是点“撤销”按钮），
    // 底层数据（磁盘上的 calibration.json）已经被 undo()/redo() 改回去了，
    // 但树状列表不会自己知道要刷新——之前漏了这一行，导致"数据其实改
    // 回来了，但界面看起来像撤销没生效"。indexChanged 在 push/undo/redo
    // 任何一种情况下都会触发，这里统一处理，不用在每个操作点各自记得刷新。
    connect(undo_, &QUndoStack::indexChanged, this, &CalibrationLibraryDialog::refreshTree);

    auto* v = new QVBoxLayout(this);
    v->addWidget(new QLabel(QStringLiteral(
        "每次标定都是独立的模板，互不覆盖。选中模板下方3D预览显示该模板的相机布局；"
        "双击/右键可重命名、设为当前使用、删除单台相机或整个模板（Ctrl+Z 撤销删除）。")));

    tree_ = new QTreeWidget;
    tree_->setColumnCount(1);
    tree_->setHeaderHidden(true);
    v->addWidget(tree_, 1);

    auto* btnRow = new QHBoxLayout;
    auto* btnActive = new QPushButton(QStringLiteral("设为当前使用"));
    auto* btnRename = new QPushButton(QStringLiteral("重命名模板"));
    auto* btnRenameCam = new QPushButton(QStringLiteral("重命名相机"));
    auto* btnDelCam = new QPushButton(QStringLiteral("删除选中相机"));
    auto* btnDelTpl = new QPushButton(QStringLiteral("删除整个模板"));
    auto* btnUndo   = new QPushButton(QStringLiteral("撤销 (Ctrl+Z)"));
    connect(btnUndo, &QPushButton::clicked, undo_, &QUndoStack::undo);
    btnRow->addWidget(btnActive); btnRow->addWidget(btnRename); btnRow->addWidget(btnRenameCam);
    btnRow->addWidget(btnDelCam); btnRow->addWidget(btnDelTpl);
    btnRow->addStretch(1); btnRow->addWidget(btnUndo);
    v->addLayout(btnRow);

    connect(btnActive, &QPushButton::clicked, this, &CalibrationLibraryDialog::onSetActive);
    connect(btnRename, &QPushButton::clicked, this, &CalibrationLibraryDialog::onRenameTemplate);
    connect(btnRenameCam, &QPushButton::clicked, this, &CalibrationLibraryDialog::onRenameCamera);
    connect(btnDelCam, &QPushButton::clicked, this, &CalibrationLibraryDialog::onDeleteCamera);
    connect(btnDelTpl, &QPushButton::clicked, this, &CalibrationLibraryDialog::onDeleteTemplate);

    view3d_ = new TriangulationView;
    v->addWidget(view3d_, 1);

    status_ = new QLabel;
    v->addWidget(status_);

    connect(tree_, &QTreeWidget::itemSelectionChanged, this, &CalibrationLibraryDialog::onSelectionChanged);

    refreshTree();
}

CalibrationLibraryDialog::~CalibrationLibraryDialog() {
    // QUndoStack（undo_）是 this 的子对象。Qt 按 C++ 标准顺序析构：先跑完
    // ~CalibrationLibraryDialog()/~QDialog()/~QWidget() 这些派生类析构函数
    // 体，最后才在 ~QObject() 里自动删除子对象（包括 undo_）——这时候
    // this 已经在"析构进行时"，动态类型已经褪回了更基类的状态。
    //
    // 如果 undo_ 析构时栈里还有命令、当前 index 不是 0，QUndoStack 内部
    // 清空命令栈会重新触发一次 indexChanged 信号——这个信号连到了
    // this->refreshTree()，Qt 新式 connect 语法的类型安全检查这时候会
    // 发现"目标对象已经不是完整类型了"，直接断言失败、整个进程崩掉：
    //   "Called object is not of the correct type (class destructor may
    //    have already run)"
    // 这是 QUndoStack 的经典坑：子对象析构时发信号回调到"正在析构的父
    // 对象"身上。修法是在这里——this 还是完整、正确类型的最早时机——
    // 主动断开这条连接，undo_ 后面被 Qt 自动删除时，指向 this 的连接
    // 已经不在了，不会再有回调发生。
    disconnect(undo_, nullptr, this, nullptr);
}

void CalibrationLibraryDialog::refreshTree() {
    tree_->clear();
    for (const auto& t : lib_->templates()) {
        const int n = lib_->cameraCount(t.id);
        const bool active = (t.id == lib_->activeTemplateId());
        auto* tplItem = new QTreeWidgetItem(tree_);
        tplItem->setText(0, QString("%1  （%2台相机）%3")
            .arg(t.displayName).arg(n).arg(active ? QStringLiteral("  ★当前使用") : QString()));
        tplItem->setData(0, Qt::UserRole, t.id);
        tplItem->setData(0, Qt::UserRole + 1, QString());   // 空表示这是模板行，非相机行

        for (const auto& cam : lib_->templateCameras(t.id)) {
            auto* camItem = new QTreeWidgetItem(tplItem);
            camItem->setText(0, QString("%1  %2")
                .arg(cam.isCalibrated() ? QStringLiteral("●") : QStringLiteral("○"), shortLabel(cam)));
            camItem->setData(0, Qt::UserRole, t.id);
            camItem->setData(0, Qt::UserRole + 1, cam.deviceKey);
        }
    }
    tree_->expandAll();
    status_->setText(QStringLiteral("共 %1 个模板").arg(lib_->templates().size()));
}

QString CalibrationLibraryDialog::selectedTemplateId() const {
    auto items = tree_->selectedItems();
    if (items.isEmpty()) return {};
    return items.first()->data(0, Qt::UserRole).toString();
}

QString CalibrationLibraryDialog::selectedCameraKey() const {
    auto items = tree_->selectedItems();
    if (items.isEmpty()) return {};
    return items.first()->data(0, Qt::UserRole + 1).toString();
}

void CalibrationLibraryDialog::onSelectionChanged() {
    const QString tplId = selectedTemplateId();
    if (tplId.isEmpty()) return;

    QVector<TriangulationView::CamPose> poses;
    for (const auto& cam : lib_->templateCameras(tplId))
        poses << TriangulationView::CamPose{ shortLabel(cam), cam.extr.R, cam.extr.t };
    view3d_->setCameras(poses);

    const auto info = lib_->templateInfo(tplId);
    status_->setText(QStringLiteral("模板「%1」：%2 台相机").arg(info.displayName).arg(poses.size()));
}

void CalibrationLibraryDialog::onRenameTemplate() {
    const QString tplId = selectedTemplateId();
    if (tplId.isEmpty()) { status_->setText(QStringLiteral("先选中一个模板")); return; }
    const auto info = lib_->templateInfo(tplId);
    bool ok = false;
    const QString name = QInputDialog::getText(this, QStringLiteral("重命名模板"),
        QStringLiteral("新名字："), QLineEdit::Normal, info.displayName, &ok);
    if (!ok || name.trimmed().isEmpty()) return;
    lib_->renameTemplate(tplId, name.trimmed());
    refreshTree();
}

void CalibrationLibraryDialog::onRenameCamera() {
    const QString tplId = selectedTemplateId();
    const QString key = selectedCameraKey();
    if (tplId.isEmpty() || key.isEmpty()) {
        status_->setText(QStringLiteral("请先选中要重命名的相机（模板下面的那一行，不是模板本身）"));
        return;
    }
    CameraCalibration cur;
    bool found = false;
    for (const auto& c : lib_->templateCameras(tplId))
        if (c.deviceKey == key) { cur = c; found = true; break; }
    if (!found) return;

    bool ok = false;
    const QString alias = QInputDialog::getText(this, QStringLiteral("重命名相机"),
        QStringLiteral("给这台相机起个好记的名字（比如“左相机”），留空则显示自动短标签：\n%1").arg(key),
        QLineEdit::Normal, cur.alias, &ok);
    if (!ok) return;
    lib_->renameCamera(tplId, key, alias.trimmed());
    refreshTree();
}

void CalibrationLibraryDialog::onSetActive() {
    const QString tplId = selectedTemplateId();
    if (tplId.isEmpty()) { status_->setText(QStringLiteral("先选中一个模板")); return; }
    const int n = lib_->cameraCount(tplId);
    if (n < 2) {
        QMessageBox::warning(this, QStringLiteral("无法使用"),
            QStringLiteral("这个模板只有 %1 台相机的标定，三角化至少需要 2 台，无法设为当前使用。")
                .arg(n));
        return;
    }
    lib_->setActiveTemplateId(tplId);
    refreshTree();
}

void CalibrationLibraryDialog::onDeleteCamera() {
    const QString tplId = selectedTemplateId();
    const QString key = selectedCameraKey();
    if (tplId.isEmpty() || key.isEmpty()) {
        status_->setText(QStringLiteral("请先选中要删除的相机（模板下面的那一行，不是模板本身）"));
        return;
    }
    // 先读一份快照（不做真正删除），交给撤销命令；push() 触发 redo() 才是
    // 真正执行删除。
    CameraCalibration removed;
    bool found = false;
    for (const auto& c : lib_->templateCameras(tplId))
        if (c.deviceKey == key) { removed = c; found = true; break; }
    if (!found) return;

    undo_->push(new DeleteCameraCommand(lib_, tplId, removed));
    refreshTree();

    const int remain = lib_->cameraCount(tplId);
    if (remain < 2)
        status_->setText(QStringLiteral("已删除，该模板现在只剩 %1 台相机，不足以三角化，暂时无法设为当前使用。")
            .arg(remain));
}

void CalibrationLibraryDialog::onDeleteTemplate() {
    const QString tplId = selectedTemplateId();
    if (tplId.isEmpty()) { status_->setText(QStringLiteral("先选中一个模板")); return; }

    const auto info = lib_->templateInfo(tplId);
    if (QMessageBox::question(this, QStringLiteral("删除模板"),
            QStringLiteral("确定删除整个模板「%1」吗？（可以 Ctrl+Z 撤销）").arg(info.displayName))
        != QMessageBox::Yes) return;

    // 同样先读快照再 push，redo() 才真正删除。
    const auto cams = lib_->templateCameras(tplId);
    const bool wasActive = (tplId == lib_->activeTemplateId());
    undo_->push(new DeleteTemplateCommand(lib_, info, cams, wasActive));
    refreshTree();
}

} // namespace mocap
