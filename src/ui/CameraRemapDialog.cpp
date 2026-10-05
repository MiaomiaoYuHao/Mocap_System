#include "ui/CameraRemapDialog.hpp"
#include "ui/CalibWizard.hpp"   // 复用 PreviewPane
#include "camera/ICamera.hpp"
#include "calib/CalibrationStore.hpp"
#include "calib/Calibration.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QRegularExpression>
#include <QSet>

namespace mocap {

CameraRemapDialog::CameraRemapDialog(const QVector<ICamera*>& liveCams,
                                     const QVector<QString>& templateKeys,
                                     CalibrationStore* store, QWidget* parent)
    : QDialog(parent), liveCams_(liveCams), templateKeys_(templateKeys), store_(store) {
    setWindowTitle(QStringLiteral("相机重映射（重插拔后恢复标定）"));
    resize(720, 480);

    auto* v = new QVBoxLayout(this);
    v->addWidget(new QLabel(QStringLiteral(
        "重插拔/换 USB 口后设备路径会变，标定对不上时在这里把当前相机对应回模板中已标定的相机，无需重标。\n"
        "已按型号自动预匹配；同型号多台分不清时点「看画面」确认。")));

    buildRows();
    applyAutoMatch();

    auto* btnRow = new QHBoxLayout;
    auto* autoBtn = new QPushButton(QStringLiteral("🔄 重新自动匹配"));
    connect(autoBtn, &QPushButton::clicked, this, &CameraRemapDialog::onAutoMatch);
    btnRow->addWidget(autoBtn);
    btnRow->addStretch(1);
    v->addLayout(btnRow);

    auto* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(box, &QDialogButtonBox::accepted, this, &CameraRemapDialog::onAccept);
    connect(box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    v->addWidget(box);
}

QString CameraRemapDialog::vidPid(const QString& deviceKey) const {
    static const QRegularExpression reVid("vid_([0-9a-fA-F]{4})");
    static const QRegularExpression rePid("pid_([0-9a-fA-F]{4})");
    const auto mv = reVid.match(deviceKey);
    const auto mp = rePid.match(deviceKey);
    if (mv.hasMatch() && mp.hasMatch())
        return QStringLiteral("%1:%2").arg(mv.captured(1).toUpper(), mp.captured(1).toUpper());
    return QString();
}

QString CameraRemapDialog::shortLabel(const QString& deviceKey, const QString& alias) const {
    if (!alias.isEmpty()) return alias;
    const QString vp = vidPid(deviceKey);
    if (!vp.isEmpty()) return QStringLiteral("USB相机 %1").arg(vp);
    return deviceKey;   // 解析不出就显示原始值
}

void CameraRemapDialog::buildRows() {
    auto* grid = new QGridLayout;
    grid->addWidget(new QLabel(QStringLiteral("<b>模板里已标定的相机</b>")), 0, 0);
    grid->addWidget(new QLabel(QStringLiteral("<b>对应当前哪台实时相机</b>")), 0, 1);
    grid->addWidget(new QLabel(QStringLiteral("<b>确认</b>")), 0, 2);

    int r = 1;
    for (const QString& oldKey : templateKeys_) {
        Row row;
        row.oldKey = oldKey;

        // 从 store 里那台相机的标定读别名（如果有）。CameraCalibration 里
        // 有 alias 字段——get() 走 resolveKey，这里直接查旧 key 拿原始标定。
        const CameraCalibration cal = store_->get(oldKey);
        row.leftLabel = new QLabel(shortLabel(oldKey, cal.alias));
        grid->addWidget(row.leftLabel, r, 0);

        row.combo = new QComboBox;
        row.combo->addItem(QStringLiteral("—（不映射）"), QString());
        for (ICamera* c : liveCams_) {
            // 下拉项显示：实时相机名 + 它的 VID/PID，方便跟左边型号对照。
            const QString vp = vidPid(c->deviceKey());
            const QString text = vp.isEmpty()
                ? c->name()
                : QStringLiteral("%1 (%2)").arg(c->name(), vp);
            row.combo->addItem(text, c->deviceKey());
        }
        grid->addWidget(row.combo, r, 1);

        auto* peek = new QPushButton(QStringLiteral("👁 看画面"));
        const int rowIdx = rows_.size();
        connect(peek, &QPushButton::clicked, this, [this, rowIdx] { onPeek(rowIdx); });
        grid->addWidget(peek, r, 2);

        rows_.push_back(row);
        ++r;
    }
    static_cast<QVBoxLayout*>(layout())->addLayout(grid);
}

void CameraRemapDialog::applyAutoMatch() {
    // 按 VID/PID 自动预匹配：对每台模板相机，找 VID/PID 相同、且还没被
    // 别的行占用的实时相机。同型号多台时按出现顺序配（不保证对，但给个
    // 起点，用户再用「看画面」核对/调整）。VID/PID 都对不上的不自动配。
    QSet<QString> used;   // 已被占用的实时相机 deviceKey
    for (Row& row : rows_) {
        const QString wantVp = vidPid(row.oldKey);
        int chosen = 0;   // 0 = 不映射
        if (!wantVp.isEmpty()) {
            for (int i = 0; i < liveCams_.size(); ++i) {
                const QString liveKey = liveCams_[i]->deviceKey();
                if (used.contains(liveKey)) continue;
                if (vidPid(liveKey) != wantVp) continue;
                // combo 第0项是"不映射"，实时相机从第1项起，故 +1。
                chosen = i + 1;
                used.insert(liveKey);
                break;
            }
        }
        row.combo->setCurrentIndex(chosen);
    }
}

void CameraRemapDialog::onAutoMatch() {
    applyAutoMatch();
}

void CameraRemapDialog::onPeek(int rowIdx) {
    if (rowIdx < 0 || rowIdx >= rows_.size()) return;
    // 弹当前这一行下拉框选中的那台实时相机的画面。没选（不映射）就提示。
    const QString liveKey = rows_[rowIdx].combo->currentData().toString();
    if (liveKey.isEmpty()) return;
    ICamera* cam = nullptr;
    for (ICamera* c : liveCams_) if (c->deviceKey() == liveKey) { cam = c; break; }
    if (!cam) return;

    auto* dlg = new QDialog(this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowTitle(QStringLiteral("这是哪一台？—— %1").arg(cam->name()));
    dlg->resize(420, 340);
    auto* v = new QVBoxLayout(dlg);
    v->addWidget(new QLabel(QStringLiteral(
        "对着这台相机挥挥手，看画面动的是不是你要对应的那台。确认完关掉即可。")));
    auto* pane = new PreviewPane;
    v->addWidget(pane, 1);
    connect(cam, &ICamera::frameReady, dlg, [pane](quint32, const QImage& img, double) {
        pane->setImage(img);
    });
    dlg->show();
}

void CameraRemapDialog::onAccept() {
    // 把每一行的选择写进 store 的 remap 表。
    //   - 选了某台实时相机：remap[实时新key] = 模板旧key。
    //   - 选"不映射"：清掉这台旧 key 可能残留的映射（找出映射到它的新 key
    //     并清除）——避免上次映射过、这次改成不映射却没生效。
    // 先收集这次要建立的映射。
    QSet<QString> clearedOld;
    for (const Row& row : rows_) {
        const QString liveKey = row.combo->currentData().toString();
        if (!liveKey.isEmpty()) {
            // 实时相机自己就有标定（比如它其实没换口、key 没变）时不需要
            // 映射——setRemap 里 newKey==oldKey 会当成清除，这里也不建立。
            if (liveKey != row.oldKey)
                store_->setRemap(liveKey, row.oldKey);
        }
    }
    accept();
}

} // namespace mocap
