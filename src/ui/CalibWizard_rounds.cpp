// ===========================================================================
// CalibWizard_rounds.cpp —— CalibWizard 的“标定板接力 / 求解”实现分片。
// 从 CalibWizard.cpp 拆出（原第 1611~2139 行）：detectBoardFolders ~ validateCurrentPage。
// 仍是 CalibWizard 的成员函数定义，与主文件共用 ui/CalibWizard.hpp 的类声明。
// ===========================================================================
#include "ui/CalibWizard.hpp"
#include "ui/CalibResultView.hpp"
#include "ui/CalibrationDialog.hpp"
#include "ui/Theme.hpp"
#include <QScrollArea>   // 第3步整页放进滚动区，见 pageBoardRounds()
#include <QFrame>        // QFrame::NoFrame，同上
#include <QPalette>
#include "camera/CameraManager.hpp"
#include "camera/ICamera.hpp"
#include "calib/CalibrationStore.hpp"
#include "calib/CalibSettingsStore.hpp"
#include "calib/RoundConnectivity.hpp"
#include "settings/ParamPresets.hpp"
#include "core/ProjectPaths.hpp"
#include "camera/WebcamCamera.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QTableWidget>
#include <QHeaderView>
#include <QFileDialog>
#include <QDateTime>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSet>
#include <QProcess>
#include <QTimer>
#include <QPainter>
#include <QCoreApplication>
#include <QMessageBox>
#include <QFile>
#include <QFileInfo>
#include <QLineF>
#include <QDialog>
#include <QPixmap>
#include <QGuiApplication>
#include <QThreadPool>
#include <QRunnable>
#include <memory>
#include <QDebug>
#include <QGridLayout>
#include <cmath>
#include <QListWidget>
#include <QAbstractItemView>
#include <QListView>
#include <QIcon>
#include <QRegularExpression>
#include <QUndoStack>
#include <QUndoCommand>
#include <QAction>
#include <QKeySequence>
#include <algorithm>
#include <functional>
#include <QPair>

namespace mocap {

namespace {
// 与 CalibWizard.cpp 主文件相同的匿名常量（两 TU 各自一份，值必须一致）。
constexpr int kWandCoRequired = 500;
} // namespace

// ---------- ③ 标定板接力（标定板方式）----------
// 对某一轮已存的帧跑 ChArUco 板检测，返回"真检到板子"的相机文件夹序号集合。
// 约定（需要 solve_calibration.py 配合支持，跟 --intrinsics-only 同一模式）：
//   python solve_calibration.py --detect-only <会话目录> <roundId>
// 脚本读 manifest.json 里的板参数，对 <会话目录>/rounds/<roundId>/camN.png 逐张
// 检测，往 stdout 打一行 JSON：{"0": true, "1": false, ...}（键=相机文件夹序号，
// 值=该张是否检到足够的板角点）。若 Python/脚本缺失或输出无法解析，把原因写进
// *err、返回空集合——调用方据此回退到"按勾选记录（未验证）"，不至于没装 Python
// 就用不了向导。
// 对某一轮已存的帧跑 ChArUco 板检测，返回 folder序号 -> 检到的角点数（0=没检到/
// 被挡住）的映射——角点数既用来判"算不算共视"（>0），也直接反映这一轮质量，
// 供 refreshRoundsUI 在轮次列表里提前显示。约定（需要 solve_calibration.py 配合
// 支持，跟 --intrinsics-only 同一模式）：
//   python solve_calibration.py --detect-only <会话目录> <roundId>
// 脚本读 manifest.json 里的板参数，对 <会话目录>/rounds/<roundId>/camN.png 逐张
// 检测，往 stdout 打一行 JSON：{"0": 37, "1": 0}（键=相机文件夹序号，值=角点数）。
// 若 Python/脚本缺失或输出无法解析，把原因写进 *err、返回空集合——调用方据此回退
// 到"按勾选记录（未验证）"，不至于没装 Python 就用不了向导。
QHash<int, int> CalibWizard::detectBoardFolders(const QString& roundId, QString* err) const {
    QHash<int, int> found;
    const QString script = QCoreApplication::applicationDirPath() + "/solve_calibration.py";
    if (!QFile::exists(script)) { if (err) *err = QStringLiteral("未找到求解脚本"); return found; }
    const QString py = pyExe();
    if (py.isEmpty()) { if (err) *err = QStringLiteral("未检测到 Python"); return found; }

    QProcess p;
    p.start(py, { script, QStringLiteral("--detect-only"), sessionDir_, roundId });
    if (!p.waitForStarted(3000) || !p.waitForFinished(15000)) {
        p.kill(); p.waitForFinished(1000);
        if (err) *err = QStringLiteral("板子检测进程超时/无法启动");
        return found;
    }
    if (p.exitStatus() != QProcess::NormalExit || p.exitCode() != 0) {
        if (err) *err = QStringLiteral("板子检测脚本返回错误");
        return found;
    }

    const QByteArray out = p.readAllStandardOutput();
    QJsonParseError je{};
    QJsonDocument doc = QJsonDocument::fromJson(out.trimmed(), &je);
    if (!doc.isObject()) {
        // 输出可能夹带日志，逐行从后往前找最后一段 { 开头能解析的 JSON。
        const QStringList lines = QString::fromUtf8(out).split('\n');
        for (auto it = lines.crbegin(); it != lines.crend(); ++it) {
            const QString t = it->trimmed();
            if (t.startsWith('{')) { doc = QJsonDocument::fromJson(t.toUtf8()); break; }
        }
    }
    if (!doc.isObject()) { if (err) *err = QStringLiteral("板子检测输出无法解析"); return found; }

    const QJsonObject o = doc.object();
    for (auto it = o.constBegin(); it != o.constEnd(); ++it)
        found[it.key().toInt()] = it.value().toInt();
    return found;
}

void CalibWizard::onCaptureRound() {
    QVector<quint32> chosen;
    for (int i = 0; i < roundCamList_->count(); ++i) {
        auto* item = roundCamList_->item(i);
        if (item->checkState() == Qt::Checked) chosen << item->data(Qt::UserRole).toUInt();
    }
    if (chosen.size() < kMinCamsPerRound) {
        QMessageBox::information(this, QStringLiteral("标定板轮次"),
                                 QStringLiteral("至少勾选 %1 台相机才算一轮——单台相机没法跟别的相机产生共视关系。")
                                     .arg(kMinCamsPerRound));
        return;
    }
    if (!ensureSession()) return;

    const QString roundId = QStringLiteral("round_%1").arg(nextRoundSeq_++, 4, 10, QChar('0'));
    const QString roundDir = sessionDir_ + "/rounds/" + roundId;
    QDir().mkpath(roundDir);

    QStringList missing;
    QVector<quint32> saved;
    for (quint32 camId : chosen) {
        const QImage img = lastFrame_.value(camId);
        if (img.isNull()) {
            for (ICamera* c : cams_) if (c->id() == camId) missing << c->name();
            continue;
        }
        const int idx = folderIdx_.value(camId);
        img.save(QString("%1/cam%2.png").arg(roundDir).arg(idx));
        saved << camId;
    }

    // 板子检测：只有"真检到 ChArUco 板"的相机才算这一轮看到了板子——否则拍到
    // 的只是杂乱背景也会被当成共视（就是"拍了无论什么都显示连通"那个问题）。
    // 把 saved 收紧到真看到板子的相机，再往下走原来的"够不够两台"判断。
    {
        QString detErr;
        QGuiApplication::setOverrideCursor(Qt::WaitCursor);
        const QHash<int, int> boardFolders = detectBoardFolders(roundId, &detErr);
        QGuiApplication::restoreOverrideCursor();
        if (detErr.isEmpty()) {
            QStringList noBoard;
            QVector<quint32> verified;
            QHash<quint32, int> quality;
            for (quint32 camId : saved) {
                const int idx = folderIdx_.value(camId);
                const int corners = boardFolders.value(idx, 0);
                if (cameraSeenBoard(corners)) {
                    verified << camId;
                    quality[camId] = corners;   // 供轮次列表显示质量，见 refreshRoundsUI
                } else {
                    // 没检到板：删掉这张孤儿帧——求解器是 glob 整个轮次目录的
                    // （load_rounds），留着会被重读一遍再丢弃，纯属浪费；删掉更干净。
                    QFile::remove(QString("%1/cam%2.png").arg(roundDir).arg(idx));
                    for (ICamera* c : cams_) if (c->id() == camId) noBoard << c->name();
                }
            }
            saved = verified;
            roundCamQuality_[roundId] = quality;
            if (!noBoard.isEmpty() && saved.size() >= kMinCamsPerRound)
                QMessageBox::information(this, QStringLiteral("标定板轮次"),
                                         QStringLiteral("这一轮里 %1 没检测到标定板（可能被挡住/太糊/不在画面里），"
                                                        "不计入共视。").arg(noBoard.join("、")));
        } else if (!detectUnavailWarned_) {
            // 检测不可用：退回按勾选记录，但明确告知连通性此时未经板子验证。
            // 只弹一次，免得每拍一轮都被打断。
            detectUnavailWarned_ = true;
            QMessageBox::warning(this, QStringLiteral("标定板轮次"),
                                 QStringLiteral("未能做板子检测（%1），本次向导后续轮次都按你的勾选记为共视——"
                                                "注意连通性此时未经板子验证。最终求解会真正检测，没拍到板的相机"
                                                "会在那步被丢弃。").arg(detErr));
        }
    }

    if (saved.size() < kMinCamsPerRound) {
        QDir(roundDir).removeRecursively();   // 可用的不够两台，这一轮作废，不留半成品
        roundCamQuality_.remove(roundId);     // 不留孤儿质量数据（这个 roundId 不会进 roundIds_）
        QMessageBox::warning(this, QStringLiteral("标定板轮次"),
                             QStringLiteral("这一轮可用的相机不足 %2 台（只有 %1 台既有画面又检测到标定板），"
                                            "已经作废——把板子摆到至少 %2 台都能清楚看到、没被遮挡的位置再拍。")
                                 .arg(saved.size()).arg(kMinCamsPerRound));
        return;
    }

    roundIds_.push_back(roundId);
    roundCams_[roundId] = saved;
    if (!missing.isEmpty())
        QMessageBox::information(this, QStringLiteral("标定板轮次"),
                                 QStringLiteral("这一轮已保存（%1 台），但 %2 没有画面、跳过了。")
                                     .arg(saved.size()).arg(missing.join("、")));
    writeManifest();
    refreshRoundsUI();
}

void CalibWizard::onSetAnchorRound() {
    auto* item = roundsList_->currentItem();
    if (!item) {
        QMessageBox::information(this, QStringLiteral("标定板轮次"),
                                 QStringLiteral("先在下面的列表里点选中一轮，再点这个按钮标记它为世界锚定轮。"));
        return;
    }
    anchorRoundId_ = item->data(Qt::UserRole).toString();
    writeManifest();
    refreshRoundsUI();
}

void CalibWizard::onDeleteRound() {
    auto* item = roundsList_->currentItem();
    if (!item) return;
    const QString rid = item->data(Qt::UserRole).toString();
    QDir(sessionDir_ + "/rounds/" + rid).removeRecursively();
    roundIds_.removeAll(rid);
    roundCams_.remove(rid);
    roundCamQuality_.remove(rid);
    if (anchorRoundId_ == rid) anchorRoundId_.clear();
    writeManifest();
    refreshRoundsUI();
}

void CalibWizard::refreshRoundsUI() {
    if (!roundsList_) return;   // 页面还没建好（比如挥球模式下这些控件全是空指针）

    // ---- 轮次列表 ----
    roundsList_->clear();
    for (const QString& rid : roundIds_) {
        // 名字后面带上角点数（质量反馈，不用等第⑤步求解才知道这一轮好不好）：
        // 角点数越多，这一轮的位姿解算通常越稳。没有质量数据（比如检测器
        // 当时不可用、按勾选记录的轮次）就只显示名字，不编造数字。
        const auto& quality = roundCamQuality_.value(rid);
        QStringList names;
        for (quint32 id : roundCams_.value(rid)) {
            for (ICamera* c : cams_) if (c->id() == id) {
                    names << (quality.contains(id)
                              ? QStringLiteral("%1(%2角点)").arg(c->name()).arg(quality.value(id))
                              : c->name());
                }
        }
        const bool isAnchor = (rid == anchorRoundId_);
        auto* item = new QListWidgetItem(QStringLiteral("%1%2：%3")
                                             .arg(isAnchor ? QStringLiteral("⚓ ") : QString(), rid, names.join("、")));
        item->setData(Qt::UserRole, rid);
        roundsList_->addItem(item);
    }

    // ---- 共视矩阵 + 连通性（跟 solve_calibration.py 用同一套并查集逻辑）----
    const int n = int(cams_.size());
    QVector<QVector<int>> counts(n, QVector<int>(n, 0));
    QVector<QVector<int>> roundIdxs;   // 每一轮参与的相机下标列表，喂给 buildCovisibilityEdges
    for (const QString& rid : roundIds_) {
        const auto& camIds = roundCams_.value(rid);
        QVector<int> idxs;
        for (quint32 id : camIds)
            for (int k = 0; k < n; ++k) if (cams_[k]->id() == id) idxs << k;
        roundIdxs.push_back(idxs);
        // 共视矩阵是数值统计（这两台一起入镜过几轮），跟"边表"是两回事——
        // 边表只关心连不连通，矩阵还要计数，所以这里仍单独跑一遍双重循环。
        for (int a = 0; a < idxs.size(); ++a)
            for (int b = a + 1; b < idxs.size(); ++b) {
                counts[idxs[a]][idxs[b]]++; counts[idxs[b]][idxs[a]]++;
            }
    }
    const QVector<QPair<int,int>> edgeList = buildCovisibilityEdges(roundIdxs);

    if (covisTable_) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j) {
                    auto* it = new QTableWidgetItem(QStringLiteral("—"));
                    it->setTextAlignment(Qt::AlignCenter);
                    covisTable_->setItem(i, j, it);
                    continue;
                }
                auto* it = new QTableWidgetItem(QString::number(counts[i][j]));
                it->setTextAlignment(Qt::AlignCenter);
                const QColor bg = counts[i][j] == 0 ? theme::badBannerBg()
                                  : counts[i][j] < 2   ? theme::warnBannerBg()
                                                     : theme::okBannerBg();
                it->setBackground(bg);
                covisTable_->setItem(i, j, it);
            }
        }
    }

    if (connectivityLabel_) {
        if (n < 2) {
            connectivityLabel_->setText(QStringLiteral("相机数不足 2 台，无法做外参标定。"));
        } else {
            const auto comp = unionFindComponents(n, edgeList);
            const auto mc = findMainComponent(comp);
            QStringList isolated;
            for (int i : mc.isolatedIndices) isolated << cams_[i]->name();

            if (isolated.isEmpty()) {
                connectivityLabel_->setText(QStringLiteral("✅ 全部 %1 台相机已连通，可以设置世界锚定轮、进入下一步").arg(n));
                connectivityLabel_->setStyleSheet(QStringLiteral(
                    "padding:6px 8px;border-radius:2px;border-left:2px solid %1;background:%2;color:%1;")
                    .arg(theme::hex(theme::okBannerFg()), theme::hex(theme::okBannerBg())));
            } else {
                connectivityLabel_->setText(QStringLiteral(
                                                "⚠ 尚未连通：%1 还没连上——把标定板挪到它们跟已连通相机的共视区，再拍几轮。")
                                                .arg(isolated.join("、")));
                connectivityLabel_->setStyleSheet(QStringLiteral(
                    "padding:6px 8px;border-radius:2px;border-left:2px solid %1;background:%2;color:%1;")
                    .arg(theme::hex(theme::badBannerFg()), theme::hex(theme::badBannerBg())));
            }
        }
    }
}

bool CalibWizard::roundsStepReady(QString* reason) const {
    const int n = int(cams_.size());
    if (n < 2) {
        if (reason) *reason = QStringLiteral("至少需要两台相机才能做外参标定。");
        return false;
    }
    QVector<QVector<int>> roundIdxs;
    for (const QString& rid : roundIds_) {
        const auto& camIds = roundCams_.value(rid);
        QVector<int> idxs;
        for (quint32 id : camIds)
            for (int k = 0; k < n; ++k) if (cams_[k]->id() == id) idxs << k;
        roundIdxs.push_back(idxs);
    }
    const QVector<QPair<int,int>> edgeList = buildCovisibilityEdges(roundIdxs);
    if (edgeList.isEmpty()) {
        if (reason) *reason = QStringLiteral("还一轮都没拍——先拍几轮标定板，把相机连起来。");
        return false;
    }
    const auto comp = unionFindComponents(n, edgeList);
    if (!allConnected(comp)) {
        if (reason) *reason = QStringLiteral("还没全部连通——看上面的连通性提示，把标定板挪到"
                                     "还没连上的相机跟已连通相机的共视区，再拍几轮。");
        return false;
    }
    if (anchorRoundId_.isEmpty()) {
        if (reason) *reason = QStringLiteral("还没标记世界锚定轮——在下面列表选中一轮，点"
                                     "「设为世界锚定轮」，那一轮的板子位置将定义最终的世界坐标系。");
        return false;
    }
    return true;
}

// ---------- 会话与清单 ----------
bool CalibWizard::ensureSession() {
    if (!sessionDir_.isEmpty()) return true;
    sessionDir_ = dirEdit_->text().trimmed();
    if (sessionDir_.isEmpty()) return false;
    QDir().mkpath(sessionDir_);
    writeManifest();
    return true;
}

void CalibWizard::writeManifest() {
    QJsonObject board;
    board["dict"] = bDict_->currentText();
    board["squaresX"] = bSx_->value();
    board["squaresY"] = bSy_->value();
    board["squareMM"] = bSq_->value();
    board["markerMM"] = bMk_->value();

    QJsonArray cams;
    for (ICamera* c : cams_) {
        QJsonObject o;
        o["deviceKey"] = c->deviceKey();
        o["name"] = c->name();
        o["folder"] = QString("cam%1").arg(folderIdx_.value(c->id()));
        const QSize s = frameSize_.value(c->id());
        o["width"] = s.width(); o["height"] = s.height();
        cams << o;
    }

    QJsonArray world;
    for (int r = 0; r < 4; ++r) {
        QJsonArray p;
        for (int c = 0; c < 3; ++c)
            p << (worldTable_ && worldTable_->item(r, c)
                      ? worldTable_->item(r, c)->text().toDouble() : 0.0);
        world << p;
    }

    QJsonObject root;
    root["board"] = board;
    root["cameras"] = cams;
    root["worldPoints"] = world;   // 板子模式不用这个字段，写个占位值不影响（求解端板子分支不读它）
    root["extrinsicsMode"] = boardExtrinsicsMode_ ? QStringLiteral("board") : QStringLiteral("wand");
    if (boardExtrinsicsMode_ && !anchorRoundId_.isEmpty())
        root["anchorRound"] = anchorRoundId_;
    root["created"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    QFile f(sessionDir_ + "/manifest.json");
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

// ---------- ⑤ 求解 ----------
QString CalibWizard::pyExe() const {
    for (const QString& cand : { QStringLiteral("python"), QStringLiteral("py") }) {
        QProcess p; p.start(cand, { "--version" });
        if (p.waitForFinished(2000) && p.exitCode() == 0) return cand;
    }
    return {};
}

void CalibWizard::onSolve() {
    writeManifest();
    if (wandRecording_) onToggleWand();   // 忘了停就替他停并落盘

    const QString script = QCoreApplication::applicationDirPath() + "/solve_calibration.py";
    if (!QFile::exists(script)) {
        log_->appendPlainText(QStringLiteral("[错误] 未找到求解脚本：%1").arg(script));
        return;
    }
    const QString py = pyExe();
    if (py.isEmpty()) {
        log_->appendPlainText(QStringLiteral(
                                  "[错误] 未检测到 Python。请安装 Python 3 并执行：\n"
                                  "    pip install opencv-contrib-python numpy\n"
                                  "然后手动运行：\n    python \"%1\" \"%2\"\n"
                                  "完成后回到这里点“查看结果”前，请先在“高级：手动录入”里导入 "
                                  "会话文件夹下生成的 calibration.json。").arg(script, sessionDir_));
        return;
    }

    solveBtn_->setEnabled(false);
    log_->appendPlainText(QStringLiteral("[运行] %1 %2 %3").arg(py, script, sessionDir_));
    proc_ = new QProcess(this);
    proc_->setProcessChannelMode(QProcess::MergedChannels);
    connect(proc_, &QProcess::readyRead, this, [this] {
        log_->appendPlainText(QString::fromLocal8Bit(proc_->readAll()).trimmed());
    });
    connect(proc_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, [this](int code, QProcess::ExitStatus) { onSolveFinished(code); });
    proc_->start(py, { script, sessionDir_ });
}

void CalibWizard::onSolveFinished(int exitCode) {
    solveBtn_->setEnabled(true);
    if (exitCode != 0) {
        log_->appendPlainText(QStringLiteral("[失败] 求解脚本退出码 %1，查看上方日志。").arg(exitCode));
        return;
    }
    // 导入结果。
    QFile f(sessionDir_ + "/calibration.json");
    if (!f.open(QIODevice::ReadOnly)) {
        log_->appendPlainText(QStringLiteral("[失败] 未生成 calibration.json"));
        return;
    }
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
    const QJsonArray arr = doc.object()["cameras"].toArray();

    // 先清掉"本次向导参与的相机"在标定库里的旧记录，再灌新结果。
    // 为什么不直接按新结果的 deviceKey 覆盖就完事：同一台物理相机如果中途
    // 换了 USB 口/重新插拔，deviceKey（Windows 设备路径）可能变化，旧 key
    // 那条记录不会被新 key 覆盖掉，就会以"幽灵相机"的形式永远残留在结果
    // 列表里。这里以"本次会话实际用的相机集合"为准，把这些相机的旧条目
    // （无论 key 变没变）统统先删掉——同时严格只删本次参与的，完全无关的
    // 相机（比如你这次没接、上次单独标好的另一台）标定保持不动，支持
    // "只补标其中几台"的用法。
    QSet<QString> sessionKeys;
    for (int i = 0; i < mgr_->count(); ++i)
        sessionKeys.insert(mgr_->at(i)->deviceKey());
    // 新结果里的 key 也一并纳入清理集合（防止极端情况下 mgr_ 与求解结果
    // 的 key 对不齐时漏删）。
    for (const auto& v : arr) {
        const QString k = v.toObject()["deviceKey"].toString();
        if (!k.isEmpty()) sessionKeys.insert(k);
    }
    for (const QString& k : store_->keys())
        if (sessionKeys.contains(k)) store_->remove(k);

    int n = 0;
    for (const auto& v : arr) {
        CameraCalibration c = CameraCalibration::fromJson(v.toObject());
        if (!c.deviceKey.isEmpty()) { store_->set(c); ++n; }
    }
    store_->save();
    viewBtn_->setEnabled(true);
    log_->appendPlainText(QStringLiteral("[完成] 已导入 %1 台相机的标定，点“查看结果”。").arg(n));

    // 板子模式：求解脚本会在 world_axes/ 下生成"锚定轮照片+画好的世界系
    // XYZ 轴"，自动弹一张出来让用户当场确认世界系原点/朝向符不符合预期
    // （红=X 绿=Y 蓝=Z 黄点=原点）。多台的话挑第一张显示，并提示其余的
    // 在文件夹里。找不到图就静默跳过（比如脚本是老版本、还没这个功能）。
    if (boardExtrinsicsMode_) {
        const QString axesDir = sessionDir_ + "/world_axes";
        QDir d(axesDir);
        const QStringList imgs = d.exists()
                                     ? d.entryList(QStringList() << "*.png", QDir::Files, QDir::Name) : QStringList();
        if (!imgs.isEmpty()) {
            const QString first = axesDir + "/" + imgs.first();
            log_->appendPlainText(QStringLiteral(
                                      "[世界系] 已生成世界坐标轴示意图（%1 张）：%2\n"
                                      "         红=X 绿=Y 蓝=Z 黄点=原点，确认朝向符合预期。")
                                      .arg(imgs.size()).arg(QDir::toNativeSeparators(axesDir)));

            auto* dlg = new QDialog(this);
            dlg->setAttribute(Qt::WA_DeleteOnClose);
            dlg->setWindowTitle(QStringLiteral("世界坐标系朝向确认（红=X 绿=Y 蓝=Z 黄点=原点）"));
            auto* lay = new QVBoxLayout(dlg);
            auto* note = new QLabel(QStringLiteral(
                                        "下图是「世界锚定轮」的照片，画上了最终世界坐标系的三条轴。"
                                        "确认原点位置和 X/Y/Z 朝向符合你的预期；不符合的话，"
                                        "回第③步把锚定轮换成板子摆放/朝向正确的那一轮，重新求解即可。"
                                        "%1").arg(imgs.size() > 1
                                                 ? QStringLiteral("\n（共 %1 台相机的视角，这里显示第一张，其余在 world_axes 文件夹）").arg(imgs.size())
                                                 : QString()));
            note->setWordWrap(true);
            lay->addWidget(note);
            auto* pic = new QLabel;
            QPixmap pm(first);
            if (!pm.isNull())
                pic->setPixmap(pm.scaled(720, 540, Qt::KeepAspectRatio, Qt::SmoothTransformation));
            pic->setAlignment(Qt::AlignCenter);
            lay->addWidget(pic, 1);
            auto* ok = new QPushButton(QStringLiteral("知道了"));
            connect(ok, &QPushButton::clicked, dlg, &QDialog::accept);
            lay->addWidget(ok, 0, Qt::AlignRight);
            dlg->resize(760, 640);
            dlg->show();
        }
    }
}

// ---------- 严格样本门槛：拦住"下一步"，直到量/质都达标 ----------
// 【这一版放弃了九宫格面积覆盖率门槛】覆盖率是"数据分布够不够均匀"的一个
// 代理指标，代理指标天生有失真风险——不管用质心判定单格、还是外接框判定
// 重叠格，本质上都是在用一个粗糙的图像梯度启发式去猜"标定精度大概会怎样"，
// 猜得再仔细也只是猜。真正决定精度的量其实近在眼前，不用猜：
// maybeRunIntrinsicsPreview() 早就在后台老老实实跑真正的ChArUco内参标定、
// 把重投影误差算出来了(lastReprojErrPx_)——直接拿这个当门槛，比任何"覆盖率"
// 代理指标都更贴近"标定精度到底够不够"这个真问题，而且不需要新写任何标定
// 逻辑，复用的是已经在跑、已经在界面上显示给用户看的同一份数据。

// 【严格选拔模式已移除】原来这里有一整套门槛：每机最少张数、重投影误差
// 必须低于 0.5px、九宫格覆盖度不够不让下一步。实际用下来它挡人多于帮人——
// 重投影误差预览算得慢，经常出现"明明拍够了却过不去"，用户只能反复点。
// 现在不设门槛，重投影误差仍然照常显示，判断交给用户。
bool CalibWizard::boardStepReady(QString* reason) const {
    Q_UNUSED(reason);
    return true;
}

bool CalibWizard::wandStepReady(QString* reason) const {
    if (wandCoDone_ < kWandCoRequired) {
        if (reason) *reason = QStringLiteral(
                          "共视有效样本只有 %1，至少需要 %2（不是任意一台相机自己的计数，"
                          "是所有相机同时看到球的样本数）——继续挥球一会儿。")
                          .arg(wandCoDone_).arg(kWandCoRequired);
        return false;
    }
    return true;
}

bool CalibWizard::validateCurrentPage() {
    if (currentId() == 1) {
        QString reason;
        if (!boardStepReady(&reason)) {
            QMessageBox::warning(this, QStringLiteral("还没拍够"), reason);
            return false;
        }
    } else if (!boardExtrinsicsMode_ && currentId() == wandPageId_) {
        QString reason;
        if (!wandStepReady(&reason)) {
            QMessageBox::warning(this, QStringLiteral("样本还不够"), reason);
            return false;
        }
    } else if (boardExtrinsicsMode_ && currentId() == boardRoundsPageId_) {
        QString reason;
        if (!roundsStepReady(&reason)) {
            QMessageBox::warning(this, QStringLiteral("还没准备好"), reason);
            return false;
        }
    }
    return QWizard::validateCurrentPage();
}

} // namespace mocap
