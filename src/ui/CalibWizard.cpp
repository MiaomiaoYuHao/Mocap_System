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
// 板参数/世界坐标表的持久化，不需要在 CalibWizard.hpp 里加成员——用一个
// 进程内单例，向导每次打开都读到上次的值，改了立刻落盘，重启也不丢。
// 这个跟下面的自动连拍状态不一样：这是真正意义上"进程全局"的设置
// （跟哪个 CalibWizard 实例无关），用单例是合适的；自动连拍状态是
// 某一次向导会话自己的东西，已经改成 CalibWizard 的正经成员了。
CalibSettingsStore& calibSettings() {
    static CalibSettingsStore store;
    return store;
}

// 简易清晰度评分：稀疏网格采样点的水平/垂直灰度梯度平方和，取均值。
// 移动模糊/失焦时梯度会明显变小；用稀疏网格保证计算开销恒定、不随分辨率
// 线性增长——这是自动连拍时每隔一小段时间就要跑一次的检查，不能太重。
// 阈值凭经验给了个起点，如果实测下来"太严格几乎不存图"或"太松存了很多
// 糊图"，找我调整，也可以做成面板上的滑块。这是无状态的纯函数，留在
// 自由函数就好，不用是成员。
// 活性检测（Layer 1）用：估个大概亮度，稀疏采样、别整幅遍历——每帧都要跑，
// 得便宜。只用来判"这台是不是黑屏"，不用来判"看没看到板子"（那个需要真正的
// ChArUco 检测，见 detectBoardFolders），精度要求很低。
// 启动时清理孤儿 .trash：正常关闭向导时 ~CalibWizard() 会把 .trash 清空
// （见析构函数注释），但如果上一次是程序中途崩溃/被强杀，析构函数根本没
// 机会跑，.trash 里的文件就会永久留在磁盘上，此前只能靠手动去
// cam*/intrinsics/.trash/ 翻。这里在每次打开向导时顺手扫一遍默认会话根
// 目录，把所有历史会话里残留的 .trash 目录清掉，不用再手动清理。
//
// 覆盖范围的诚实说明：只扫 sessionsRoot（也就是第①步"会话保存到"那个
// 输入框的默认值，<项目目录>/calib_sessions）——如果用户当
// 时点了"选择…"把会话存到了别的自定义路径，这次扫描覆盖不到那个路径。
// 要做到"不管存哪都能扫到"得维护一份"历史用过的所有路径"的持久化清单，
// 复杂度不成比例；默认路径覆盖绝大多数真实使用场景已经足够。
int sweepOrphanTrash(const QString& sessionsRoot) {
    int cleaned = 0;
    QDir root(sessionsRoot);
    if (!root.exists()) return 0;   // 第一次用，还没产生过任何会话，正常

    const QStringList sessions = root.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QString& sessionName : sessions) {
        QDir sessionDir(root.filePath(sessionName));
        const QStringList camDirs = sessionDir.entryList(
            QStringList() << "cam*", QDir::Dirs | QDir::NoDotAndDotDot);
        for (const QString& camName : camDirs) {
            const QString trashDir = sessionDir.filePath(camName) + "/intrinsics/.trash";
            if (QDir(trashDir).exists() && QDir(trashDir).removeRecursively())
                ++cleaned;
        }
    }
    return cleaned;
}

double sampleLuma(const QImage& img) {
    if (img.isNull() || img.width() < 4 || img.height() < 4) return 0.0;
    const int step = qMax(1, img.width() / 32);
    quint64 sum = 0; int n = 0;
    for (int y = 0; y < img.height(); y += step)
        for (int x = 0; x < img.width(); x += step) {
            sum += qGray(img.pixel(x, y)); ++n;
        }
    return n ? double(sum) / n : 0.0;
}

double roughSharpness(const QImage& img) {
    if (img.width() < 20 || img.height() < 20) return 0.0;
    const QImage g = img.format() == QImage::Format_Grayscale8
                         ? img : img.convertToFormat(QImage::Format_Grayscale8);
    const int stepX = std::max(1, g.width() / 60);
    const int stepY = std::max(1, g.height() / 45);
    double sum = 0; int n = 0;
    for (int y = stepY; y < g.height() - stepY; y += stepY) {
        const uchar* row   = g.constScanLine(y);
        const uchar* rowUp = g.constScanLine(y - stepY);
        const uchar* rowDn = g.constScanLine(y + stepY);
        for (int x = stepX; x < g.width() - stepX; x += stepX) {
            const int gx = int(row[x + stepX]) - int(row[x - stepX]);
            const int gy = int(rowDn[x]) - int(rowUp[x]);
            sum += double(gx * gx + gy * gy);
            ++n;
        }
    }
    return n > 0 ? sum / n : 0.0;
}
constexpr double kSharpnessThreshold = 60.0;   // 凭经验起点，太严/太松都可以调
// 每隔多久尝试自动存一张。250ms = 4 张/秒。
// 【为什么可以翻倍】这个节拍原来是 500ms，配合"严格选拔"的九宫格限量才有意义——
// 存得慢一点、每张都挑过。选拔机制去掉之后没有再压节奏的理由：
// 判糊的 roughSharpness 是逐帧算的，节拍快了只是尝试次数多，糊的照样被跳过。
// 用户举着板子走一遍的时间直接减半。
constexpr qint64 kAutoShotIntervalMs = 250;

// ---------------------------------------------------------------------
// 覆盖度粗定位：找板子大概落在画面哪个区域，用于 3x3 网格覆盖度统计和
// 自动连拍的"选拔机制"。不追求精确分割/角点级定位（那是 Python 端
// ChArUco 检测的活），只要求"板子大致在画面哪个九宫格"这个粗粒度信息——
// 复用跟 roughSharpness 完全一样的稀疏网格梯度采样，取梯度明显偏高的点
// （棋盘格局部对比度远高于均匀背景）的加权质心。稀疏采样下这个近似足够
// 稳定，且不引入任何新依赖。
constexpr double kFootprintGradThreshold = 900.0;   // 经验值，明显高于噪声水平
constexpr int    kCoverageGridN = 3;                // 3x3 覆盖网格
// kMinCoveredCells 已移除——覆盖率门槛被重投影误差门槛取代，见 boardStepReady()。
constexpr qint64 kIntrinsicsPreviewCooldownMs = 4000; // 内参预览节流：不是每拍一张都跑

struct BoardFootprint {
    bool found = false;
    QPointF center;      // 归一化质心——保留给"选拔机制"的单格去重/择优逻辑用，语义不变
    QRectF  bboxNorm;    // 归一化外接框——给覆盖度判定用，见下面 cellsOverlapping()
};

BoardFootprint computeBoardFootprint(const QImage& img) {
    BoardFootprint out;
    if (img.width() < 20 || img.height() < 20) return out;
    const QImage g = img.format() == QImage::Format_Grayscale8
                         ? img : img.convertToFormat(QImage::Format_Grayscale8);
    const int stepX = std::max(1, g.width() / 60);
    const int stepY = std::max(1, g.height() / 45);
    // 【这是"屏幕角落照到了但覆盖度不算数"的第一个病根】采样窗口以前从
    // stepY/stepX起、到height-stepY/width-stepX止，等于把最外一圈(将近
    // 一个步长厚，通常有10~20px)的像素直接排除在梯度检测范围外——用户
    // 真把板子顶到画面物理边角拍时，板子在图像最外沿的那部分内容，程序
    // 自己都没去看过。改成从1开始、到width-1/height-1结束，邻域访问
    // (y-stepY/y+stepY等)用clamp限制在合法范围内而不是整段跳过——内部
    // 区域的梯度计算距离(stepX/stepY)跟原来完全一样，kFootprintGradThreshold
    // 这个经验阈值不会因此失配；只有紧贴边缘的那几行/列，梯度邻域基线会
    // 比stepX/stepY略短(clamp到了边界)，但仍然能检测到高对比度内容，不再
    // 是结构性的"看不见"。
    double sumW = 0, sumX = 0, sumY = 0;
    int minX = g.width(), maxX = 0, minY = g.height(), maxY = 0;
    for (int y = 1; y < g.height() - 1; y += stepY) {
        const int yUp = std::max(0, y - stepY), yDn = std::min(g.height() - 1, y + stepY);
        const uchar* row   = g.constScanLine(y);
        const uchar* rowUp = g.constScanLine(yUp);
        const uchar* rowDn = g.constScanLine(yDn);
        for (int x = 1; x < g.width() - 1; x += stepX) {
            const int xL = std::max(0, x - stepX), xR = std::min(g.width() - 1, x + stepX);
            const int gx = int(row[xR]) - int(row[xL]);
            const int gy = int(rowDn[x]) - int(rowUp[x]);
            const double mag = double(gx * gx + gy * gy);
            if (mag > kFootprintGradThreshold) {
                sumW += mag; sumX += mag * x; sumY += mag * y;
                minX = std::min(minX, x); maxX = std::max(maxX, x);
                minY = std::min(minY, y); maxY = std::max(maxY, y);
            }
        }
    }
    if (sumW <= 0) return out;
    out.found = true;
    out.center = QPointF((sumX / sumW) / g.width(), (sumY / sumW) / g.height());
    out.bboxNorm = QRectF(QPointF(double(minX) / g.width(), double(minY) / g.height()),
                          QPointF(double(maxX) / g.width(), double(maxY) / g.height()));
    return out;
}

int cellIndexFor(QPointF norm) {
    const int cx = std::clamp(int(norm.x() * kCoverageGridN), 0, kCoverageGridN - 1);
    const int cy = std::clamp(int(norm.y() * kCoverageGridN), 0, kCoverageGridN - 1);
    return cy * kCoverageGridN + cx;
}

// 【这是第二个病根，也是主因】覆盖度以前判定"这一格算不算拍到了"，用的是
// "梯度响应加权质心落在哪一个格子"——板子斜着伸到画面角落拍摄时，可见
// 部分的质心大概率还落在中间偏那个方向的格子里，不是角落格子本身，角落
// 格子的桶因此永远是空的，不管你实际把板子怼到那个角落拍了多少张。改成
// 用外接框跟哪些格子有重叠——只要板子的检测区域碰到了某个格子的范围，
// 那个格子就算覆盖到了，这才是用户直觉里"这个角落拍过了"的真正含义。
QVector<int> cellsOverlapping(const QRectF& bboxNorm) {
    QVector<int> out;
    if (!bboxNorm.isValid() && bboxNorm.width() <= 0 && bboxNorm.height() <= 0) {
        // 极端退化(单点)：宽高为0时QRectF::isValid()可能为false，仍按单点处理
    }
    const int cx0 = std::clamp(int(bboxNorm.left()   * kCoverageGridN), 0, kCoverageGridN - 1);
    const int cx1 = std::clamp(int(bboxNorm.right()  * kCoverageGridN), 0, kCoverageGridN - 1);
    const int cy0 = std::clamp(int(bboxNorm.top()    * kCoverageGridN), 0, kCoverageGridN - 1);
    const int cy1 = std::clamp(int(bboxNorm.bottom() * kCoverageGridN), 0, kCoverageGridN - 1);
    for (int r = cy0; r <= cy1; ++r)
        for (int c = cx0; c <= cx1; ++c)
            out.push_back(r * kCoverageGridN + c);
    return out;
}

// 挥球共视样本：不是"每台相机各自记录了多少个点"，而是"所有相机在同一
// 个时间点都看到了球"——这才是三角化真正用得上的量。
//
// 第一版用固定网格分桶（ts_ns / 10ms），实测共视计数涨得极慢——根因是
// 分桶的硬边界效应：两台相机的时间戳哪怕只差 2ms（19ms vs 21ms），只要
// 恰好卡在桶边界两侧，就会被分进不同桶、永远配不上，而 solve_calibration.py
// 里 match_tracks() 用的两两双指针 + 容忍窗口（TOL_NS=12ms）完全没有这个
// 问题。这里改用同样的思路：跟踪每台相机"最近一次样本"的时间戳，每来一个
// 新样本就检查是否全部相机的最近样本互相都落在容忍窗口内——是连续滑动的
// 判断，不是离散网格，不存在边界丢判的问题。
constexpr qint64 kWandToleranceNs = 20'000'000;   // 20ms，比两两配对稍宽松（这里要求 N 台全部同时在窗口内，不是两两各自12ms）
constexpr int    kWandCoRequired = 500;

// 并查集连通分量——标定板接力页实时算连通性用。跟 solve_calibration.py
// 里 connected_components() 是同一个算法（简单并查集），保证"前端显示
// 连通"和"后端真的能求解"是一回事，不会出现前端绿灯、后端却报孤岛。
} // namespace

// 分支式向导的固定页号：两套外参页面都建好，实际走哪条由 nextId() 按当前
// boardExtrinsicsMode_ 路由——第①步的模式开关因此本次立即生效。两套页面
// id 不重叠；PAGE_BOARD 保持 =1，跟代码里 id==1（拍板页）的硬编码一致。
enum WizPage {
    PAGE_INTRO        = 0,
    PAGE_BOARD        = 1,   // ChArUco 内参拍摄（两模式共用）
    PAGE_BOARD_ROUNDS = 2,   // 标定板接力（仅接力模式走到）
    PAGE_WAND         = 3,   // 挥球（仅挥球模式走到）
    PAGE_FRAME        = 4,   // 坐标支架（仅挥球模式走到）
    PAGE_SOLVE        = 5    // 求解并完成（两模式共用，末页）
};

// ============================ PreviewPane ============================
PreviewPane::PreviewPane(QWidget* parent) : QWidget(parent) {
    setMinimumSize(360, 240);
}
void PreviewPane::setImage(const QImage& img) { img_ = img; update(); }
void PreviewPane::setBlobs(const QVector<QPointF>& pts) { blobs_ = pts; }

void PreviewPane::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.fillRect(rect(), QColor(0x14, 0x14, 0x14));   // 跟宫格预览同一档暗底
    if (img_.isNull()) {
        p.setPen(theme::textDim());
        p.drawText(rect(), Qt::AlignCenter, QStringLiteral("等待画面…"));
        return;
    }
    QImage s = img_.scaled(size(), Qt::KeepAspectRatio, Qt::FastTransformation);
    const int x = (width() - s.width()) / 2, y = (height() - s.height()) / 2;
    p.drawImage(x, y, s);
    if (!blobs_.isEmpty()) {
        const double sx = double(s.width()) / img_.width();
        const double sy = double(s.height()) / img_.height();
        QPen pen(theme::marker()); pen.setWidthF(1.4); p.setPen(pen);
        for (const QPointF& b : blobs_) {
            const QPointF c(x + b.x() * sx, y + b.y() * sy);
            p.drawEllipse(c, 6, 6);
            p.drawLine(c + QPointF(-9, 0), c + QPointF(-3, 0));
            p.drawLine(c + QPointF(3, 0),  c + QPointF(9, 0));
            p.drawLine(c + QPointF(0, -9), c + QPointF(0, -3));
            p.drawLine(c + QPointF(0, 3),  c + QPointF(0, 9));
        }
    }
}

// ============================ CalibWizard ============================
CalibWizard::CalibWizard(CameraManager* mgr, CalibrationStore* store,
                         ParamPresets* presets, QWidget* parent)
    : QWizard(parent), mgr_(mgr), store_(store), presets_(presets) {
    setWindowTitle(QStringLiteral("标定向导"));
    // 【为什么从 ModernStyle 换成 ClassicStyle】ModernStyle 的顶部标题带不是
    // 普通控件，是 QWizard 内部的 QWizardHeader 自己按【调色板】画出来的 ——
    // 全局样式表管不到它。整个程序转深色之后，这条带子会是唯一一块白底，
    // 一打开向导就非常刺眼，而且没法用 QSS 修。
    //
    // ClassicStyle 压根不建那个 header 控件，标题/副标题就是页面里的普通
    // QLabel，样式表全覆盖得到。代价是少了那条装饰带 —— 但本来我们也不想
    // 要装饰带，直角朴素才是这一版的方向。
    setWizardStyle(QWizard::ClassicStyle);
    // 兜底：ClassicStyle 下 Qt 仍会用调色板画分隔标尺之类的小装饰。
    // 只改这一个窗口的调色板，不影响别处。
    {
        QPalette wp = palette();
        wp.setColor(QPalette::Window,     theme::editorBg());
        wp.setColor(QPalette::Base,       theme::editorBg());
        wp.setColor(QPalette::WindowText, theme::text());
        wp.setColor(QPalette::Text,       theme::text());
        wp.setColor(QPalette::Mid,        theme::textDim());
        wp.setColor(QPalette::Dark,       theme::chrome());
        wp.setColor(QPalette::Light,      theme::lineHard());
        setPalette(wp);
    }
    resize(1060, 720);   // 深色 + 直角之后信息密度更高，给向导更舒展的默认尺寸

    // 清理上次异常退出（崩溃/被强杀）遗留的孤儿 .trash——正常关闭时
    // ~CalibWizard() 会做这件事，但那次要是没走到正常关闭，就得靠这里
    // 兜底。只在打开向导时扫一次，不影响运行时性能。
    {
        const QString sessionsRoot = projectRootDir() + QStringLiteral("/calib_sessions");
        const int cleaned = sweepOrphanTrash(sessionsRoot);
        if (cleaned > 0)
            qInfo().noquote() << QStringLiteral("[标定向导] 清理了 %1 个上次异常退出遗留的临时文件夹")
                                     .arg(cleaned);
    }

    // 自动连拍状态现在是正经成员了，默认值在头文件里已经是 false/空，
    // 这里不用再手动清——每次 new CalibWizard 都是全新对象，天然干净。

    // 参与标定：仅真实相机（虚拟测试相机没有物理意义）。
    for (int i = 0; i < mgr_->count(); ++i) {
        ICamera* c = mgr_->at(i);
        if (c->deviceKey() == QStringLiteral("virt")) continue;
        cams_ << c;
        keyOf_[c->id()] = c->deviceKey();
        folderIdx_[c->id()] = int(cams_.size()) - 1;
        prevDetect_[c->id()] = c->detectEnabled();
        connect(c, &ICamera::frameReady, this, &CalibWizard::onFrame);
        connect(c, &ICamera::blobsReady, this, &CalibWizard::onBlobs);
    }

    // 外参求解方式：读上次的设置作为初值。以前这里按它决定加哪些页、构造完
    // 就定死，导致第①步的开关只能"下次生效"。现在改成分支式向导——见下。
    boardExtrinsicsMode_ = calibSettings().useBoardExtrinsics();

    // 两套外参页面都建好，实际走哪条由 nextId() 按当前 boardExtrinsicsMode_
    // 决定，所以第①步的模式开关本次就能切（勾一下立即改变第③步走哪个页面），
    // 不再是"下次重开才生效"。页号用固定枚举、两套 id 不重叠；凡是判断"进到
    // 某步该做什么"的地方仍旧用 boardRoundsPageId_/wandPageId_/framePageId_
    // 这几个成员配合模式判断，所以那些逻辑一行都不用改。
    setPage(PAGE_INTRO,        pageIntro());
    setPage(PAGE_BOARD,        pageBoard());
    setPage(PAGE_BOARD_ROUNDS, pageBoardRounds());
    setPage(PAGE_WAND,         pageWand());
    setPage(PAGE_FRAME,        pageFrame());
    setPage(PAGE_SOLVE,        pageSolve());
    boardRoundsPageId_ = PAGE_BOARD_ROUNDS;
    wandPageId_        = PAGE_WAND;
    framePageId_       = PAGE_FRAME;
    solvePageId_       = PAGE_SOLVE;
    setStartId(PAGE_INTRO);
    connect(this, &QWizard::currentIdChanged, this, &CalibWizard::enterPage);

    uiTimer_ = new QTimer(this);
    uiTimer_->setInterval(250);
    connect(uiTimer_, &QTimer::timeout, this, [this] {
        refreshActivityStatuses();   // Layer 1 活性状态条（接力页/挥球页网格标题）
        // 低频刷新计数标签，避免每帧 setText。
        if (wandCounts_) {
            QStringList parts;
            for (ICamera* c : cams_)
                parts << QString("%1: %2").arg(c->name()).arg(wandN_.value(c->id(), 0));
            const QString ready = wandCoDone_ >= kWandCoRequired
                                      ? QStringLiteral("（已达标，可以停了）") : QString();
            wandCounts_->setText(QStringLiteral("各相机已记录样本  ") + parts.join("   ")
                                 + QStringLiteral("\n共视有效样本（三角化真正用得上的那部分）：%1 / %2 %3")
                                       .arg(wandCoDone_).arg(kWandCoRequired).arg(ready));
        }
    });
    uiTimer_->start();
}

CalibWizard::~CalibWizard() {
    // 恢复进向导前的检测开关。
    for (ICamera* c : cams_)
        c->setDetectEnabled(prevDetect_.value(c->id(), false));

    // 审查照片对话框里"删除"其实是把文件挪进 cam*/intrinsics/.trash/，
    // 目的是配合那个对话框自己的 QUndoStack 支持 Ctrl+Z 撤销——但那个
    // QUndoStack 是挂在对话框（QDialog）身上的，对话框一关就跟着销毁，
    // 撤销历史也就没了。如果向导本体也在这之后关闭，.trash 里还没被
    // 撤销复原的文件就再没有任何 UI 路径能找回来了（只能去文件管理器
    // 手动翻），放着不管就是纯粹的磁盘空间泄漏，日积月累会越攒越多。
    // 这里在向导关闭时统一清掉——语义上等价于"删除操作到向导关闭这一刻
    // 才真正、不可撤销地生效"，跟"关掉审查窗口前必须先决定好"这件事
    // 解耦：只要向导还开着，用户随时能重新打开审查窗口……不过要注意，
    // 重新打开时旧的撤销历史确实已经丢了（每次打开审查窗口都是全新的
    // QUndoStack），这是当前设计的已知边界，不是这里能修的——真要做到
    // "整个向导会话内跨多次打开审查窗口都能撤销"，撤销栈需要提升成
    // CalibWizard 的成员而不是审查对话框的局部变量，是更大的改动，
    // 目前没必要为了这个边界情况做这个复杂度。
    if (!sessionDir_.isEmpty()) {
        for (ICamera* c : cams_) {
            const QString trashDir = QString("%1/cam%2/intrinsics/.trash")
            .arg(sessionDir_).arg(folderIdx_.value(c->id()));
            if (QDir(trashDir).exists()) QDir(trashDir).removeRecursively();
        }
    }
}

// ---------- ① 准备 ----------
QWizardPage* CalibWizard::pageIntro() {
    auto* pg = new QWizardPage;
    pg->setTitle(QStringLiteral("第 1 步 · 准备"));
    pg->setSubTitle(QStringLiteral(
        "本向导带你完成全部相机的内参（镜头畸变）与外参（相对位姿）标定。"
        "全程只需按提示拍照/挥球，最后一键求解。"));
    auto* v = new QVBoxLayout(pg);

    auto* info = new QLabel(QStringLiteral(
        "准备：1) 打印的 ChArUco 标定板；2) 外参用具（接力方式用同一块板，挥球方式需 1 颗反光球 + 四球坐标支架）。\n"
        "参与标定的相机：%1 台（虚拟相机不参与）"));
    info->setText(info->text().arg(cams_.size()));
    info->setWordWrap(true);
    v->addWidget(info);

    // 外参求解方式。现在是分支式向导（见构造函数），这个开关本次立即生效——
    // 勾选/取消当场改变第③步走「标定板接力」还是「挥球」，同时记住作为下次默认。
    auto* modeBox = new QCheckBox(QStringLiteral("用「标定板接力」求外参（推荐，更稳）"));
    modeBox->setChecked(boardExtrinsicsMode_);
    auto* modeNote = new QLabel;
    modeNote->setStyleSheet(QStringLiteral("color:%1;").arg(theme::hex(theme::textDim())));
    auto updateModeNote = [this, modeNote] {
        modeNote->setText(QStringLiteral("本次第③步：%1")
                              .arg(boardExtrinsicsMode_
                                       ? QStringLiteral("标定板接力（把板子摆到两两共视区，逐轮接力连通）")
                                       : QStringLiteral("挥球（手持反光球在共视区挥动积累样本）")));
    };
    updateModeNote();
    connect(modeBox, &QCheckBox::toggled, this, [this, updateModeNote](bool on) {
        boardExtrinsicsMode_ = on;                  // 本次立即生效：nextId() 据此分叉
        calibSettings().setUseBoardExtrinsics(on);  // 同时记住作为下次默认
        updateModeNote();
    });
    v->addWidget(modeBox);
    v->addWidget(modeNote);

    auto* form = new QFormLayout;
    // projectRootDir() 运行时定位项目根目录（见 core/ProjectPaths.hpp）：
    // 跟 build 输出目录无关，删 build 重新构建不会影响这个路径；也不会像
    // 编译期焊死的 PROJECT_SOURCE_DIR 那样，在项目文件夹改名/复制成新版本
    // 之后还指着旧文件夹。
    dirEdit_ = new QLineEdit(
        projectRootDir() + QStringLiteral("/calib_sessions/mocap_calib_")
        + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    auto* browse = new QPushButton(QStringLiteral("选择…"));
    connect(browse, &QPushButton::clicked, this, [this] {
        const QString d = QFileDialog::getExistingDirectory(this, QStringLiteral("选择会话文件夹"));
        if (!d.isEmpty()) dirEdit_->setText(d);
    });
    auto* dirRow = new QHBoxLayout;
    dirRow->addWidget(dirEdit_, 1); dirRow->addWidget(browse);
    form->addRow(QStringLiteral("会话保存到"), dirRow);

    bDict_ = new QComboBox;
    bDict_->addItems({"DICT_4X4_100", "DICT_5X5_100", "DICT_6X6_100"});
    bSx_ = new QSpinBox; bSx_->setRange(3, 20);
    bSy_ = new QSpinBox; bSy_->setRange(3, 20);
    bSq_ = new QDoubleSpinBox; bSq_->setRange(5, 200); bSq_->setSuffix(" mm");
    bMk_ = new QDoubleSpinBox; bMk_->setRange(3, 190); bMk_->setSuffix(" mm");

    // 板参数最容易被反复重填——换板子、或者像排查 calib.io 布局问题那样
    // 来回改字典/边长测试，都不该每次开向导就被打回硬编码默认值。这里读
    // 上次保存的值；如果历史记录里的字典不在下拉框选项里（比如换了新
    // 字典种类），现场补一项，不会因为找不到就静默丢失。
    {
        const BoardParams saved = calibSettings().board();
        int idx = bDict_->findText(saved.dict);
        if (idx < 0) { bDict_->addItem(saved.dict); idx = bDict_->count() - 1; }
        bDict_->setCurrentIndex(idx);
        bSx_->setValue(saved.squaresX);
        bSy_->setValue(saved.squaresY);
        bSq_->setValue(saved.squareMM);
        bMk_->setValue(saved.markerMM);
    }
    auto persistBoard = [this] {
        BoardParams b;
        b.dict = bDict_->currentText();
        b.squaresX = bSx_->value();
        b.squaresY = bSy_->value();
        b.squareMM = bSq_->value();
        b.markerMM = bMk_->value();
        calibSettings().setBoard(b);
    };
    connect(bDict_, &QComboBox::currentTextChanged, this, [persistBoard](const QString&) { persistBoard(); });
    connect(bSx_, &QSpinBox::valueChanged, this, [persistBoard](int) { persistBoard(); });
    connect(bSy_, &QSpinBox::valueChanged, this, [persistBoard](int) { persistBoard(); });
    connect(bSq_, &QDoubleSpinBox::valueChanged, this, [persistBoard](double) { persistBoard(); });
    connect(bMk_, &QDoubleSpinBox::valueChanged, this, [persistBoard](double) { persistBoard(); });

    form->addRow(QStringLiteral("ChArUco 字典"), bDict_);
    form->addRow(QStringLiteral("格子数（横×纵）"), [&] {
        auto* h = new QHBoxLayout; h->addWidget(bSx_); h->addWidget(new QLabel("×")); h->addWidget(bSy_); h->addStretch(1); return h; }());
    form->addRow(QStringLiteral("方格边长"), bSq_);
    form->addRow(QStringLiteral("码边长"),   bMk_);
    v->addLayout(form);

    auto* adv = new QHBoxLayout;
    auto* manual = new QPushButton(QStringLiteral("高级：手动录入/导入…"));
    connect(manual, &QPushButton::clicked, this, [this] {
        CalibrationDialog dlg(mgr_, store_, this); dlg.exec();
    });
    // 这里原来还有个"查看已有标定结果"按钮，查的是本次向导会话自己的
    // store_——新会话还没求解完之前这里面永远是空的，用户截图反馈的
    // "点了显示为空"就是这个原因。现在已经有独立的"标定模板库"承担
    // "查看/管理已有标定结果"这件事（能看到全部历史模板，不止这次会话），
    // 这个按钮的功能完全是重复且更差的，直接去掉，不留一个总是空的入口
    // 制造困惑。
    adv->addWidget(manual); adv->addStretch(1);
    v->addLayout(adv);
    v->addStretch(1);
    return pg;
}

// ---------- ② 拍标定板 ----------
QWizardPage* CalibWizard::pageBoard() {
    auto* pg = new QWizardPage;
    pg->setTitle(QStringLiteral("第 2 步 · 拍标定板（内参）"));
    pg->setSubTitle(QStringLiteral(
        "每台相机拍 15 张以上：不同距离、不同倾斜角，覆盖画面四角。拍完一台在下拉框切下一台。\n"
        "自动连拍：举板缓慢移动，按节奏自动存图，糊图自动跳过。\n"
        "严格选拔模式：按九宫格限额择优，覆盖不足不能进入下一步（更均匀、更慢）。"));
    auto* v = new QVBoxLayout(pg);

    auto* row = new QHBoxLayout;
    row->addWidget(new QLabel(QStringLiteral("当前相机")));
    camCombo_ = new QComboBox;
    for (ICamera* c : cams_) camCombo_->addItem(c->name(), c->id());
    row->addWidget(camCombo_, 1);
    auto* shot = new QPushButton(QStringLiteral("📷 拍一张"));
    shot->setDefault(true);
    connect(shot, &QPushButton::clicked, this, &CalibWizard::onCaptureBoard);
    row->addWidget(shot);

    auto* autoBtn = new QPushButton(QStringLiteral("🎥 自动连拍"));
    autoBtn->setCheckable(true);
    autoBtn->setToolTip(QStringLiteral(
        "开启后举着标定板缓慢移动/换角度，程序每隔约0.5秒自动存一张，"
        "画面太糊（移动中失焦）的帧会自动跳过。别拿着板子快速晃，"
        "给每个姿态留半秒左右停顿，成功率更高。"));
    connect(autoBtn, &QPushButton::toggled, this, [this](bool on) {
        boardAutoOn_ = on;
        if (on) lastAutoAttemptMs_.clear();   // 重新开始计时，避免用旧时间戳导致立刻猛存一张
    });
    boardAutoBtn_ = autoBtn;
    row->addWidget(autoBtn);

    reviewBtn_ = new QPushButton(QStringLiteral("📋 审查/剔除照片…"));
    reviewBtn_->setToolTip(QStringLiteral(
        "缩略图查看这台相机已拍的照片，勾掉板子被挡/明显糊/过曝的几张直接删除，"
        "不用去翻文件系统手动找。"));
    connect(reviewBtn_, &QPushButton::clicked, this, &CalibWizard::onReviewBoardShots);
    row->addWidget(reviewBtn_);
    v->addLayout(row);



    boardPreview_ = new PreviewPane;
    v->addWidget(boardPreview_, 1);
    boardCounts_ = new QLabel;
    // 【必须开换行 —— 这一页唯一漏掉的一个】七台相机的张数汇总拼成一行大约
    // 一百多个字符，不换行就横着长出窗口右边，而这个 QLabel 外面没有
    // QScrollArea，超出去的部分【既看不到也滚不到】。同一页的覆盖度/重投影/
    // 自动拍照三个标签都设了 wordWrap，只有它漏了。
    boardCounts_->setWordWrap(true);
    // 【显式声明纯文本】默认 Qt::AutoText 会猜内容是不是富文本。这条汇总里
    // 拼着相机名，而相机名是驱动上报的 iProduct —— 含 '<' 或 '&' 的话会被
    // 当成 HTML 解析，轻则名字缺一截，重则我们靠 '\n' 分的行被折叠成一行，
    // 又回到"横着长出去"。
    boardCounts_->setTextFormat(Qt::PlainText);
    // 等宽 + 对齐分隔，七台相机扫读时数字能对上列，不用逐个找冒号。
    boardCounts_->setStyleSheet(theme::monoCss(12.0));
    v->addWidget(boardCounts_);

    // 覆盖度反馈（点3）：3x3 粗网格，哪块区域还没拍过一目了然，不用凭感觉
    // "我觉得应该拍得差不多了"。
    boardCoverageLabel_ = new QLabel;
    boardCoverageLabel_->setStyleSheet(theme::monoCss(12.0));
    boardCoverageLabel_->setWordWrap(true);
    v->addWidget(boardCoverageLabel_);

    // 当场内参预览（点2）：攒够几张后在后台跑一次纯内参标定，把重投影
    // 误差显示出来，不用等第⑤步求解完才发现某台相机不达标、前面全白走。
    boardReprojLabel_ = new QLabel(QStringLiteral("重投影误差：尚未计算（存够 8 张以上会自动跑一次预览）"));
    boardReprojLabel_->setWordWrap(true);
    v->addWidget(boardReprojLabel_);

    boardAutoStatus_ = new QLabel;
    boardAutoStatus_->setWordWrap(true);
    v->addWidget(boardAutoStatus_);

    connect(camCombo_, &QComboBox::currentIndexChanged, this, [this](int) {
        if (!camCombo_) return;
        updateBoardCoverageLabel(camCombo_->currentData().toUInt());
        refreshReprojLabel();
    });
    // 首次构建时上面的 currentIndexChanged 不会补触发（信号是加完全部
    // items 之后才连接的），这里手动刷新一次，避免刚打开向导时两块标签
    // 显示的是空白/默认文案而非跟当前选中相机对应的真实状态。
    if (camCombo_->count() > 0) {
        updateBoardCoverageLabel(camCombo_->currentData().toUInt());
        refreshReprojLabel();
    }
    return pg;
}

// ---------- ③ 挥反光球 ----------
QWizardPage* CalibWizard::pageWand() {
    auto* pg = new QWizardPage;
    pg->setTitle(QStringLiteral("第 3 步 · 挥动单颗反光球（外参）"));
    pg->setSubTitle(QStringLiteral(
        "手持一颗反光球，在所有相机共视区内缓慢画 8 字，覆盖整个工作区。\n"
        "看下方「共视有效样本」，数够 500 即可停。场景里此时只能有这一颗球。"));
    auto* v = new QVBoxLayout(pg);

    // 网格预览：所有参与相机同时显示，跟接力页对齐——挥球时同样容易出现
    // "某台其实没画面/被挡住"，一眼看清比只信任下面的数字计数更直观。
    v->addWidget(new QLabel(QStringLiteral(
        "各相机预览（所有参与相机同时显示）：")));
    auto* grid = new QGridLayout;
    // 列数规则跟接力页保持一致：<=3 台一行排开，4 台 2x2，更多每行 3 个。
    // 用 ceil(sqrt(n)) 的话 3 台会排成 2 列、第三台独占一行、右边空一半。
    const int nCam = int(cams_.size());
    const int cols = (nCam <= 3) ? qMax(1, nCam) : (nCam == 4 ? 2 : 3);
    int gr = 0, gc = 0;
    for (ICamera* c : cams_) {
        auto* title = new QLabel(c->name());
        title->setAlignment(Qt::AlignCenter);
        auto* pane = new PreviewPane;
        pane->setMinimumSize(200, 120);
        wandPanes_[c->id()] = pane;
        wandPaneTitle_[c->id()] = title;
        auto* cell = new QVBoxLayout;
        cell->setContentsMargins(0, 0, 0, 0);
        cell->addWidget(title);
        cell->addWidget(pane, 1);
        auto* wrap = new QWidget;
        wrap->setLayout(cell);
        grid->addWidget(wrap, gr, gc);
        if (++gc >= cols) { gc = 0; ++gr; }
    }
    v->addLayout(grid, 1);

    wandBtn_ = new QPushButton(QStringLiteral("▶ 开始记录"));
    connect(wandBtn_, &QPushButton::clicked, this, &CalibWizard::onToggleWand);
    v->addWidget(wandBtn_, 0, Qt::AlignLeft);
    wandCounts_ = new QLabel(QStringLiteral("尚未开始"));
    wandCounts_->setWordWrap(true);
    v->addWidget(wandCounts_);
    return pg;
}

// ---------- ④ 拍坐标支架 ----------
QWizardPage* CalibWizard::pageFrame() {
    auto* pg = new QWizardPage;
    pg->setTitle(QStringLiteral("第 4 步 · 捕获坐标支架（世界系对齐）"));
    pg->setSubTitle(QStringLiteral(
        "把原点+XYZ 四球支架静止摆放在所有相机都能看到的位置，"
        "在下表核对/填写四颗球心的世界坐标（毫米），然后点捕获。"
        "此时场景里只能有这 4 颗球。"));
    auto* v = new QVBoxLayout(pg);

    worldTable_ = new QTableWidget(4, 3);
    worldTable_->setHorizontalHeaderLabels({"X (mm)", "Y (mm)", "Z (mm)"});
    worldTable_->setVerticalHeaderLabels({QStringLiteral("原点"), QStringLiteral("X 轴点"),
                                          QStringLiteral("Y 轴点"), QStringLiteral("Z 轴点")});
    // 支架是固定的物理硬件，量一次就不会变，没理由每次开向导都重敲一遍——
    // 读上次保存的坐标；第一次用（还没存过）就用 defs 这组常见默认值占位。
    const auto saved = calibSettings().world();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 3; ++c)
            worldTable_->setItem(r, c, new QTableWidgetItem(QString::number(saved[r][c])));
    worldTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    worldTable_->setFixedHeight(170);
    v->addWidget(worldTable_);
    connect(worldTable_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) {
        std::array<std::array<double, 3>, 4> w{};
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 3; ++c) {
                const auto* it = worldTable_->item(r, c);
                w[r][c] = it ? it->text().toDouble() : 0.0;
            }
        calibSettings().setWorld(w);
    });

    auto* cap = new QPushButton(QStringLiteral("📐 捕获支架（约 1 秒，保持静止）"));
    connect(cap, &QPushButton::clicked, this, &CalibWizard::onCaptureFrame);
    v->addWidget(cap, 0, Qt::AlignLeft);
    frameStatus_ = new QLabel;
    frameStatus_->setWordWrap(true);
    v->addWidget(frameStatus_);
    v->addStretch(1);
    return pg;
}

// ---------- ③ 标定板接力（标定板方式，替代挥球+支架两步）----------
QWizardPage* CalibWizard::pageBoardRounds() {
    auto* pg = new QWizardPage;
    pg->setTitle(QStringLiteral("第 3 步 · 标定板接力（外参 + 世界系）"));
    // 【副标题只留一句】原来这里写了四行：接力规则、共视矩阵会提示、
    // 世界锚定的含义、板子的坐标轴约定。四行小字顶在页面最上方，实际
    // 没人会在操作前逐字读完，却把整页向下挤了近百像素——而这一页本来
    // 就已经长到要滚动了。
    // 细节挪到该出现的地方：接力规则和"该往哪挪"由下面的连通性提示实时
    // 给出（它本来就在做这件事），世界锚定和坐标轴约定挪到「设为世界锚定轮」
    // 按钮的 tooltip 上——那才是要用到这个知识的时刻。
    pg->setSubTitle(QStringLiteral(
        "把标定板摆到任意两台（或更多）相机的共视区，勾选谁看到了、点拍摄；"
        "挪个位置再拍一轮。不要求所有相机同时看到，只要接力连通即可。"));

    // 【整页放进滚动区】这一页的内容是固定的一长串：预览网格 + 勾选列表 +
    // 连通性提示 + 共视矩阵 + 已拍轮次列表 + 两排按钮。相机一多、或者屏幕
    // 一小，底下的「设为世界锚定轮」就会被挤出可视区——而那一步是必做的，
    // 不做完根本没法求解。
    // 之前的表现是各控件被压扁（列表只露两行、矩阵只露一行），
    // 而不是出现滚动条，等于把"内容放不下"这件事藏了起来。
    auto* pageLay = new QVBoxLayout(pg);
    pageLay->setContentsMargins(0, 0, 0, 0);
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto* inner = new QWidget;
    scroll->setWidget(inner);
    pageLay->addWidget(scroll);
    auto* v = new QVBoxLayout(inner);
    v->setContentsMargins(0, 0, 6, 0);   // 右边留一点，别贴着滚动条

    // 网格预览：所有参与相机的小画面同时显示。每台相机的帧本来就都投递到
    // onFrame 并存进 lastFrame_，这里只是把它们都画出来，不额外增加采集开销。
    v->addWidget(new QLabel(QStringLiteral(
        "各相机预览（所有参与相机同时显示，一眼看清板子在谁的画面里）：")));
    auto* grid = new QGridLayout;
    // 【列数不能用 ceil(sqrt(n))】3 台相机时 ceil(sqrt(3))=2，排成 2 列，
    // 第三台独占一行、右边空着一半（见截图）。
    // 相机预览是横向的（16:9），横着铺比方阵好看也省高度：
    // <=3 台就一行排开，4 台 2×2，更多才每行 3 个。
    const int n = int(cams_.size());
    const int cols = (n <= 3) ? qMax(1, n) : (n == 4 ? 2 : 3);
    int gr = 0, gc = 0;
    for (ICamera* c : cams_) {
        auto* title = new QLabel(c->name());
        title->setAlignment(Qt::AlignCenter);
        auto* pane = new PreviewPane;
        // 【按 16:9 给最小尺寸】原来是 200x120（5:3），而相机画面是 16:9，
        // 塞进去左右必然留黑边。给一个跟画面同比例的下限，黑边就没了。
        pane->setMinimumSize(192, 108);
        roundPanes_[c->id()] = pane;
        roundPaneTitle_[c->id()] = title;
        auto* cell = new QVBoxLayout;
        cell->setContentsMargins(0, 0, 0, 0);
        cell->setSpacing(2);
        cell->addWidget(title);
        cell->addWidget(pane, 1);
        auto* wrap = new QWidget;
        wrap->setLayout(cell);
        grid->addWidget(wrap, gr, gc);
        if (++gc >= cols) { gc = 0; ++gr; }
    }
    // 每列等宽，否则最后一行不满时前面几列会被拉宽、大小不一致
    for (int i = 0; i < cols; ++i) grid->setColumnStretch(i, 1);
    v->addLayout(grid, 1);

    v->addWidget(new QLabel(QStringLiteral("这一轮谁看到了标定板（勾选，至少 2 台）：")));
    roundCamList_ = new QListWidget;
    for (ICamera* c : cams_) {
        auto* item = new QListWidgetItem(c->name(), roundCamList_);
        item->setData(Qt::UserRole, c->id());
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
    }
    // 【按相机台数算高度，不写死】原来固定 110px，三台相机就只露得出两台，
    // 第三台要滚动才看得见 —— 而这一步恰恰要求"勾选这一轮谁看到了"，
    // 漏掉看不见的那台是最容易犯的错，界面不该把它藏起来。
    // 上限 6 行：再多就真该滚动了，否则这一页会被列表撑得很长。
    {
        const int rows = qBound(2, int(cams_.size()), 6);
        const int rowH = roundCamList_->sizeHintForRow(0) > 0
                             ? roundCamList_->sizeHintForRow(0) : 22;
        roundCamList_->setMaximumHeight(rows * rowH + 8);
        roundCamList_->setMinimumHeight(rows * rowH + 8);
    }
    v->addWidget(roundCamList_);

    // ---- 全选 / 全不选 ----
    // 【为什么这一步特别需要】接力标定的常见做法就是"把板子摆到能被尽量多
    // 相机同时看到的位置，全勾一遍拍一轮"—— 那一轮能一次性把好几条共视边
    // 连起来，比两两凑快得多。没有全选的话每一轮都要逐个点。
    {
        auto* selRow = new QHBoxLayout;
        auto* btnAll  = new QPushButton(QStringLiteral("全选"));
        auto* btnNone = new QPushButton(QStringLiteral("全不选"));
        btnAll->setToolTip(QStringLiteral(
            "勾选全部相机。\n"
            "【只在板子确实出现在所有画面里时才用】上面的预览就是给这个用的——\n"
            "勾了没看到板子的相机，那一轮会把错误的观测算进外参里，\n"
            "而这种错误在最后求解时才暴露，很难回溯到是哪一轮引入的。"));
        btnNone->setToolTip(QStringLiteral("取消勾选全部相机，重新挑。"));
        connect(btnAll, &QPushButton::clicked, this, [this] {
            for (int i = 0; i < roundCamList_->count(); ++i)
                roundCamList_->item(i)->setCheckState(Qt::Checked);
        });
        connect(btnNone, &QPushButton::clicked, this, [this] {
            for (int i = 0; i < roundCamList_->count(); ++i)
                roundCamList_->item(i)->setCheckState(Qt::Unchecked);
        });
        selRow->addWidget(btnAll);
        selRow->addWidget(btnNone);
        selRow->addSpacing(16);

        // 【拍摄按钮跟全选放同一行】它们是同一个操作序列的三步
        // （挑相机 → 拍这一轮），拆成两行既白白多占一行高度，
        // 又让"勾完之后该点哪"变得不明显。
        // 拍摄是主操作，放最右边并做成默认按钮（回车即可触发）。
        captureRoundBtn_ = new QPushButton(QStringLiteral("📷 拍摄这一轮"));
        captureRoundBtn_->setToolTip(QStringLiteral(
            "对勾选的每台相机各存一张当前画面，作为这一轮的板子观测。\n"
            "拍摄前确认板子在这些相机的画面里都清楚可见、没被遮挡。"));
        captureRoundBtn_->setDefault(true);
        connect(captureRoundBtn_, &QPushButton::clicked, this, &CalibWizard::onCaptureRound);
        selRow->addWidget(captureRoundBtn_);
        selRow->addStretch(1);
        v->addLayout(selRow);
    }

    connectivityLabel_ = new QLabel;
    connectivityLabel_->setWordWrap(true);
    v->addWidget(connectivityLabel_);

    v->addWidget(new QLabel(QStringLiteral("共视矩阵（格子数字=这两台相机一起入镜过几轮）：")));
    covisTable_ = new QTableWidget(int(cams_.size()), int(cams_.size()));
    {
        QStringList headers;
        for (ICamera* c : cams_) headers << c->name();
        covisTable_->setHorizontalHeaderLabels(headers);
        covisTable_->setVerticalHeaderLabels(headers);
    }
    covisTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    covisTable_->setSelectionMode(QAbstractItemView::NoSelection);
    covisTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    covisTable_->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    // 【高度按行数算，不写死 160】原来固定 160px，三台相机的 3×3 矩阵
    // 只显示得出第一行（见截图）—— 而这张表的全部意义就是"一眼看出哪两台
    // 还没连上"，只露一行等于没有。
    // 行高用表头的高度做参考，再加表头本身和一点边距。
    {
        // 名字避开外层的 n（相机台数），否则 -Wshadow 会报遮蔽
        const int nRow = int(cams_.size());
        const int rowH = covisTable_->verticalHeader()->defaultSectionSize() > 0
                             ? covisTable_->verticalHeader()->defaultSectionSize() : 24;
        const int hdrH = covisTable_->horizontalHeader()->height() > 0
                             ? covisTable_->horizontalHeader()->height() : 26;
        const int h = hdrH + nRow * rowH + 6;
        covisTable_->setMinimumHeight(h);
        covisTable_->setMaximumHeight(h + rowH);   // 留一行余量，避免出滚动条
    }
    // 【列宽用 Stretch 铺满】原来是 ResizeToContents，格子里只有一个数字，
    // 于是三列挤在左边、右边一大片空白（截图里那样），矩阵的对称结构完全
    // 看不出来。铺满之后才像一张矩阵。
    v->addWidget(covisTable_);

    v->addWidget(new QLabel(QStringLiteral("已拍的轮次（⚓=世界锚定轮）：")));
    roundsList_ = new QListWidget;
    roundsList_->setMaximumHeight(130);
    v->addWidget(roundsList_);

    auto* btnRow = new QHBoxLayout;
    setAnchorBtn_ = new QPushButton(QStringLiteral("⚓ 设为世界锚定轮"));
    connect(setAnchorBtn_, &QPushButton::clicked, this, &CalibWizard::onSetAnchorRound);
    deleteRoundBtn_ = new QPushButton(QStringLiteral("删除选中轮次"));
    connect(deleteRoundBtn_, &QPushButton::clicked, this, &CalibWizard::onDeleteRound);
    btnRow->addWidget(setAnchorBtn_);
    btnRow->addWidget(deleteRoundBtn_);
    btnRow->addStretch(1);
    v->addLayout(btnRow);

    refreshRoundsUI();
    return pg;
}


QWizardPage* CalibWizard::pageSolve() {
    auto* pg = new QWizardPage;
    pg->setTitle(QStringLiteral("第 5 步 · 求解并完成"));
    pg->setSubTitle(QStringLiteral(
        "点击求解：自动运行标定求解脚本（需要本机装有 Python 与 "
        "opencv-contrib-python，见 README，一次安装终身使用）。"
        "成功后结果自动导入标定库。"));
    auto* v = new QVBoxLayout(pg);
    auto* row = new QHBoxLayout;
    solveBtn_ = new QPushButton(QStringLiteral("⚙ 运行求解"));
    connect(solveBtn_, &QPushButton::clicked, this, &CalibWizard::onSolve);
    viewBtn_ = new QPushButton(QStringLiteral("查看结果"));
    viewBtn_->setEnabled(false);
    connect(viewBtn_, &QPushButton::clicked, this, [this] {
        CalibResultView dlg(store_, this); dlg.exec();
    });
    auto* openDir = new QPushButton(QStringLiteral("打开会话文件夹"));
    connect(openDir, &QPushButton::clicked, this, [this] {
        QProcess::startDetached("explorer", { QDir::toNativeSeparators(sessionDir_) });
    });
    row->addWidget(solveBtn_); row->addWidget(viewBtn_); row->addWidget(openDir); row->addStretch(1);
    v->addLayout(row);
    log_ = new QPlainTextEdit;
    log_->setReadOnly(true);
    log_->setStyleSheet(theme::monoCss(12.0));
    v->addWidget(log_, 1);
    return pg;
}

// ---------- 页面切换：管理检测开关与预览 ----------
void CalibWizard::enterPage(int id) {
    // 页号含义随外参方式而不同，见 boardRoundsPageId_/wandPageId_/
    // framePageId_/solvePageId_ 这几个成员——只有 id==1（拍板）在两种模式
    // 下含义一样，其余一律要经过这几个成员判断，不能硬编码数字。
    if (id == 1) ensureSession();

    // 按步骤自动切换参数模板：拍板、板子轮次都是在拍"标定板"，用「标定板」
    // 参数；挥球、支架是在拍"反光点"，用「追踪」参数。没保存过对应模板
    // 就跳过（不打断流程），完全不用手动切。
    if (presets_) {
        QList<WebcamCamera*> webs;
        for (ICamera* c : cams_)
            if (auto* w = qobject_cast<WebcamCamera*>(c)) webs << w;
        const bool wantsBoardPreset = (id == 1) ||
                                      (boardExtrinsicsMode_ && id == boardRoundsPageId_);
        const bool wantsTrackPreset = !boardExtrinsicsMode_ &&
                                      (id == wandPageId_ || id == framePageId_);
        if (wantsBoardPreset && presets_->has(ParamPresets::Board))
            presets_->applyAll(ParamPresets::Board, webs);
        else if (wantsTrackPreset && presets_->has(ParamPresets::Track))
            presets_->applyAll(ParamPresets::Track, webs);
    }

    // 检测开关（反光点识别）：只有挥球/支架页需要；拍板、板子轮次都只是
    // 普通预览+手动/自动拍照，不涉及反光点检测。
    const bool needDetect = !boardExtrinsicsMode_ && (id == wandPageId_ || id == framePageId_);
    setAllDetect(needDetect);

    if (id == 1 && boardCounts_) {
        refreshBoardCountsLabel();
        if (camCombo_) {
            updateBoardCoverageLabel(camCombo_->currentData().toUInt());
            refreshReprojLabel();
        }
    }

    if (boardExtrinsicsMode_ && id == boardRoundsPageId_)
        refreshRoundsUI();
}

void CalibWizard::setAllDetect(bool on) {
    for (ICamera* c : cams_) c->setDetectEnabled(on);
}

// 分支路由：第①/②步两模式共用，走到第②步（拍板）之后按当前 boardExtrinsicsMode_
// 分叉——标定板接力走 BOARD_ROUNDS→SOLVE，挥球走 WAND→FRAME→SOLVE。因为读的是
// 实时的 boardExtrinsicsMode_，第①步的开关改了本次立即生效。SOLVE 返回 -1 →
// QWizard 在该页显示「完成」而不是「下一步」。
int CalibWizard::nextId() const {
    switch (currentId()) {
    case PAGE_INTRO:        return PAGE_BOARD;
    case PAGE_BOARD:        return boardExtrinsicsMode_ ? PAGE_BOARD_ROUNDS : PAGE_WAND;
    case PAGE_BOARD_ROUNDS: return PAGE_SOLVE;
    case PAGE_WAND:         return PAGE_FRAME;
    case PAGE_FRAME:        return PAGE_SOLVE;
    case PAGE_SOLVE:        return -1;
    default:                return -1;
    }
}

// ---------- 帧/检测数据路由 ----------
void CalibWizard::onFrame(quint32 camId, const QImage& img, double) {
    lastFrame_[camId] = img;
    if (!frameSize_.contains(camId)) frameSize_[camId] = img.size();

    // Layer 1 活性检测：不管当前在哪一页，只要这台相机在跑就记一下——判"在线/
    // 掉线/黑屏"用得上，采样便宜（sampleLuma 稀疏采样），跟拍照/预览逻辑无关。
    lastFrameMs_[camId] = QDateTime::currentMSecsSinceEpoch();
    lastLuma_[camId] = sampleLuma(img);

    if (currentId() == 1 && camCombo_ &&
        camCombo_->currentData().toUInt() == camId && boardPreview_)
        boardPreview_->setImage(img);

    if (boardExtrinsicsMode_ && currentId() == boardRoundsPageId_)
        if (auto* p = roundPanes_.value(camId)) p->setImage(img);

    if (!boardExtrinsicsMode_ && currentId() == wandPageId_)
        if (auto* p = wandPanes_.value(camId)) p->setImage(img);

    // 自动连拍：举着标定板走位，程序按节奏自动存图，不用手动一张张点。
    // 只在第②步、且是当前下拉框选中的那台相机上生效——别的相机的帧路过
    // 这里不动它。
    if (currentId() == 1 && boardAutoOn_ && camCombo_ &&
        camCombo_->currentData().toUInt() == camId) {
        const qint64 now = QDateTime::currentMSecsSinceEpoch();
        const qint64 last = lastAutoAttemptMs_.value(camId, 0);
        if (now - last >= kAutoShotIntervalMs) {
            lastAutoAttemptMs_[camId] = now;   // 不管这次存不存，节奏都往前推
            const double sharp = roughSharpness(img);
            if (sharp >= kSharpnessThreshold) {
                const bool saved = saveBoardShot(camId, img, sharp, /*fromAuto=*/true);
                if (saved) {
                    // 不调 enterPage(1)：那个函数每次都会重新下发一遍参数模板到
                    // 硬件，自动连拍每半秒调一次的话就是每半秒重发一次硬件参数，
                    // 没必要还可能引入画面轻微卡顿。这里只更新计数/覆盖度文字。
                    refreshBoardCountsLabel();
                    if (boardAutoStatus_)
                        boardAutoStatus_->setText(QStringLiteral(
                                                      "自动连拍中… 刚存了一张（清晰度分数 %1）").arg(sharp, 0, 'f', 0));
                } else if (boardAutoStatus_) {
                    // 选拔机制拦下了这一帧：这个位置已经攒够了足够清晰的样本，
                    // 新的这张也没明显更好，不值得再占一张——不是失败，是"够了"。
                    boardAutoStatus_->setText(QStringLiteral(
                                                  "自动连拍中… 这个位置样本已经选拔够了（清晰度 %1 没有明显超过已存的），"
                                                  "挪到画面里还没覆盖的区域，看下面的覆盖度提示").arg(sharp, 0, 'f', 0));
                }
            } else if (boardAutoStatus_) {
                boardAutoStatus_->setText(QStringLiteral(
                                              "自动连拍中… 这一帧太糊跳过了（分数 %1 / 门槛 %2），"
                                              "停顿一下让画面稳定再继续移动")
                                              .arg(sharp, 0, 'f', 0).arg(kSharpnessThreshold, 0, 'f', 0));
            }
        }
    }
}

void CalibWizard::refreshActivityStatuses() {
    // 只在接力页或挥球页生效，且只刷"当前正显示"的那一套标题，避免另一页
    // 没在看的控件也被无谓地 setText/setStyleSheet。
    const bool onRounds = boardExtrinsicsMode_ && currentId() == boardRoundsPageId_;
    const bool onWand    = !boardExtrinsicsMode_ && currentId() == wandPageId_;
    if (!onRounds && !onWand) return;

    QHash<quint32, QLabel*>& titles = onRounds ? roundPaneTitle_ : wandPaneTitle_;

    // "高亮"集合含义随页面而不同：接力页=这一轮勾选参与的相机；挥球页=正在
    // 录制时全员高亮（挥球没有"这一轮谁参与"的概念，是不是在录制是全局的）。
    QSet<quint32> highlighted;
    if (onRounds && roundCamList_) {
        for (int i = 0; i < roundCamList_->count(); ++i)
            if (roundCamList_->item(i)->checkState() == Qt::Checked)
                highlighted.insert(roundCamList_->item(i)->data(Qt::UserRole).toUInt());
    } else if (onWand && wandRecording_) {
        for (ICamera* c : cams_) highlighted.insert(c->id());
    }

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    constexpr qint64 kStaleMs = 600;        // 超过这么久没收到新帧，判"掉线"
    constexpr double kBlackLuma = 6.0;      // 稀疏采样的平均亮度低于这个值，判"近乎全黑"

    for (auto it = titles.constBegin(); it != titles.constEnd(); ++it) {
        QLabel* title = it.value();
        if (!title) continue;
        const quint32 id = it.key();
        QString name;
        for (ICamera* c : cams_) if (c->id() == id) name = c->name();
        const qint64 last = lastFrameMs_.value(id, -1);
        const double luma = lastLuma_.value(id, 0.0);

        QString tag, col;
        if (last < 0 || now - last > kStaleMs) {
            tag = QStringLiteral("⛔ 无画面"); col = theme::hex(theme::badBannerBg());
        } else if (luma < kBlackLuma) {
            tag = QStringLiteral("⚫ 接近全黑"); col = theme::hex(theme::badBannerBg());
        } else if (highlighted.contains(id)) {
            tag = onRounds ? QStringLiteral("✅ 本轮") : QStringLiteral("🔴 录制中");
            col = theme::hex(theme::okBannerBg());
        } else {
            tag = onRounds ? QStringLiteral("○ 在跑") : QStringLiteral("○ 待命");
            col = theme::hex(theme::inputBg());
        }
        title->setText(QStringLiteral("%1  %2").arg(name, tag));
        title->setStyleSheet(QStringLiteral(
                                 "padding:3px 6px;border-radius:2px;background:%1;color:%2;")
                                 .arg(col, theme::hex(theme::text())));
    }
}

void CalibWizard::onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns) {
    if (wandRecording_ && pts.size() == 1) {
        // deviceKey() 在 Windows 下形如 "web:\\?\usb#vid_xxxx&..."，天然带
        // 反斜杠——反斜杠是 JSON 转义字符的开头，原来手拼字符串没做任何
        // 转义就直接塞进去，路径里一旦出现"\u"这种片段就会被解析成非法的
        // \uXXXX 转义，导致 wand_blobs.jsonl 整体解析失败。
        // 这里只对 cam 这个字符串字段借 QJsonDocument 做正确转义；ts/x/y
        // 仍然手动拼数字文本——不能把 ts_ns 这种纳秒级 qint64 也交给
        // QJsonValue，它内部数字统一存 double，精度只有约15~17位有效数字，
        // 纳秒级时间戳会被悄悄截断精度，比原来的解析失败更隐蔽。
        const QString camEscaped = QString::fromUtf8(
            QJsonDocument(QJsonArray{ keyOf_.value(camId) }).toJson(QJsonDocument::Compact));
        // 上面序列化出的是形如 ["web:\\?\\usb#..."] 的数组，掐头去尾拿到
        // 转义好的带引号字符串本体。
        const QString camJsonStr = camEscaped.mid(1, camEscaped.size() - 2);

        wandLines_ << QStringLiteral("{\"cam\":%1,\"ts\":%2,\"x\":%3,\"y\":%4}")
                          .arg(camJsonStr).arg(ts_ns)
                          .arg(pts[0].x(), 0, 'f', 3).arg(pts[0].y(), 0, 'f', 3);
        wandN_[camId]++;
        registerWandSample(camId, ts_ns);
    }
    if (frameCollecting_ && pts.size() == 4)
        frameSamples_[camId].push_back(pts);
}

// ---------- ②-选拔机制：拍一张 / 自动连拍 共用的落盘逻辑 ----------
bool CalibWizard::saveBoardShot(quint32 camId, const QImage& img, double sharpness, bool fromAuto) {
    const BoardFootprint fp = computeBoardFootprint(img);
    const int cell = fp.found ? cellIndexFor(fp.center) : -1;

    // 选拔机制（严格样本门槛的另一面：不是数量越多越好，是"信息量够不够"）：
    // 只对自动连拍生效——手动点"拍一张"是用户的明确意图，永远照存，最多
    // 在界面上提示一句"这里已经拍够了"，不拦用户的手。自动连拍没有这层
    // 人的判断力，容易在同一姿态附近来回晃、刷出一堆信息量接近的照片
    // （"张数涨了，有效信息没涨"），所以按 3x3 格子做"数量上限 + 质量
    // 择优替换"：格子还没满就直接存；满了的话，新样本要明显更清晰（这里
    // 定为超过10%）才把格子里最糊的那张换掉，否则整帧跳过，不占存储也
    // 不占后续求解的计算量。这里用的还是质心归属的单格(cell)——去重/择优
    // 本来就该有个唯一的桶，跟下面"这一张实际覆盖了哪些格子"的覆盖度判定
    // 是两件事，不能混用同一份数据。
    const int idx = folderIdx_.value(camId);
    QDir().mkpath(QString("%1/cam%2/intrinsics").arg(sessionDir_).arg(idx));
    // 用单调递增的序号命名文件，跟"当前保留了几张"（shotCount_）解耦——
    // 选拔机制会删旧存新，如果复用 shotCount_ 当文件序号，删除后重新计数
    // 可能跟磁盘上还没来得及清理的旧文件撞名。序号只增不减，永不复用，
    // 彻底避免这类文件名碰撞。
    const int seq = nextShotSeq_.value(camId, 0) + 1;
    nextShotSeq_[camId] = seq;
    const QString path = QString("%1/cam%2/intrinsics/img_%3.png")
                             .arg(sessionDir_).arg(idx).arg(seq, 4, 10, QChar('0'));
    // PNG 编码 + 落盘挪到后台线程：这是自动连拍这条链里唯一在 GUI 线程上重的
    // 同步操作（每 0.5s 一次，整幅编码+写盘几十~上百毫秒）。留在主线程会在这段
    // 空档里把相机 worker 高帧率排队投来的 frameReady 越积越多、主线程再也追不上
    // ——表现就是"自动连拍卡死无响应"、内存还涨。QImage/QString 都是隐式共享(COW)，
    // 按值拷进后台任务安全；文件名用只增不复用的序号（见上），跟选拔机制删旧文件
    // 永不撞路径，所以多线程乱序也无所谓，不必串行化。
    QThreadPool::globalInstance()->start(QRunnable::create([img, path]{ img.save(path); }));
    shotCount_[camId] = shotCount_.value(camId, 0) + 1;

    const QVector<int> shotCells = fp.found ? cellsOverlapping(fp.bboxNorm) : QVector<int>{};
    if (cell >= 0) boardCells_[camId][cell].push_back({path, sharpness, shotCells});
    // 覆盖度用外接框跟哪些格子有重叠——这才是"这个角落拍到了没有"的真正
    // 含义，见 computeBoardFootprint/cellsOverlapping 顶部说明。从当前实际
    // 保留的全部照片重新推算，不是简单地把这一张的格子标记一下就完事——
    // 上面 eviction 分支可能刚删掉一张，那张也许是唯一覆盖某个格子的照片。
    recomputeCoveredCells(camId);
    updateBoardCoverageLabel(camId);
    maybeRunIntrinsicsPreview(camId);
    return true;
}

void CalibWizard::recomputeCoveredCells(quint32 camId) {
    std::array<bool, 9> covered{};
    const auto& cellsArr = boardCells_[camId];
    for (int i = 0; i < 9; ++i)
        for (const auto& shot : cellsArr[size_t(i)])
            for (int c : shot.coveredCells) covered[size_t(c)] = true;
    coveredCells_[camId] = covered;
}

void CalibWizard::onCaptureBoard() {
    if (!camCombo_) return;
    const quint32 id = camCombo_->currentData().toUInt();
    const QImage img = lastFrame_.value(id);
    if (img.isNull()) return;
    const double sharp = roughSharpness(img);
    saveBoardShot(id, img, sharp, /*fromAuto=*/false);
    enterPage(1);   // 刷新计数（手动点击不频繁，这里调用没问题，不像自动连拍那样密集）
}

// ---------- ②-覆盖度 / 内参预览 ----------
// 底部那条「每台相机拍了几张」的汇总。
//
// 【为什么单拎出来】原来这三行在 onPageChanged / 自动连拍回调 / 拍照回调里
// 各抄了一份，改格式要改三处，漏一处就出现"同一条信息两种样子"。
//
// 显示上做了两件事：
//   · 当前正在拍的那台前面加 "▶"，不用跟上面的下拉框来回对；
//   · 0 张的显式标出来。拍标定板最常见的翻车就是"以为都拍了、其实漏了一台"，
//     而挤在一行里的 0 和 15 扫过去看不出差别。
void CalibWizard::refreshBoardCountsLabel() {
    if (!boardCounts_) return;
    const quint32 curId = camCombo_ ? camCombo_->currentData().toUInt() : quint32(-1);
    QStringList parts;
    for (ICamera* c : cams_) {
        const int n = shotCount_.value(c->id(), 0);
        // 【一次性三参替换，不要链式 .arg()】链式是逐个替换的：如果相机名
        // 里恰好含有 "%1"（名字来自驱动上报的 iProduct，我们说了不算），
        // 下一个 .arg() 会把它当占位符填掉。三参版本是同时替换，不会互相吃。
        parts << QStringLiteral("%1%2   %3")
                     .arg(c->id() == curId ? QString::fromUtf8("\u25b6 ")   // ▶
                                           : QStringLiteral("   "),
                          c->name(),
                          n == 0 ? QStringLiteral("0 张（未拍）")
                                 : QStringLiteral("%1 张").arg(n));
    }
    // 【一台一行，不靠 wordWrap 折】折点落在哪儿取决于窗口宽度，
    // 一台相机的名字和张数可能被拆到两行去，正是原来"不易看"的一部分。
    // wordWrap 仍然要开着：单台相机别名很长时那一行还得能折。
    boardCounts_->setText(parts.join(QStringLiteral("\n")));
}

void CalibWizard::updateBoardCoverageLabel(quint32 camId) {
    if (!boardCoverageLabel_ || !camCombo_) return;
    if (camCombo_->currentData().toUInt() != camId) return;   // 只展示当前选中相机的
    const auto& covered9 = coveredCells_[camId];
    int covered = 0;
    QString grid;
    for (int r = 0; r < kCoverageGridN; ++r) {
        for (int c = 0; c < kCoverageGridN; ++c) {
            const int idx = r * kCoverageGridN + c;
            const bool has = covered9[size_t(idx)];
            if (has) ++covered;
            grid += has ? QStringLiteral("■") : QStringLiteral("□");
        }
        grid += "\n";
    }
    // 覆盖度九宫格纯供参考，帮你直观看出画面哪些区域还没怎么拍过——不再是
    // 拦「下一步」的门槛(那个门槛换成了真正决定精度的重投影误差，见
    // boardStepReady)。留着这个格子图是因为它依然是个有用的可视化提示：
    // 重投影误差不达标时，"往画面边缘/角落和不同倾斜角度补拍"这个建议
    // 具体该往哪补，格子图能一眼看出来，比死记"要往边缘拍"这句话直观。
    const QString hint = QStringLiteral("覆盖 %1/9 格（仅供参考，实际是否可以「下一步」看下面的重投影误差）。")
                             .arg(covered);
    boardCoverageLabel_->setText(QStringLiteral("覆盖度（9 宫格，方框=左上到右下）：\n") + grid + hint);
}

void CalibWizard::refreshReprojLabel() {
    if (!boardReprojLabel_ || !camCombo_) return;
    const quint32 camId = camCombo_->currentData().toUInt();
    if (!lastReprojErrPx_.contains(camId)) {
        boardReprojLabel_->setText(QStringLiteral("重投影误差：尚未计算（存够 8 张以上会自动跑一次预览）"));
        return;
    }
    const double e = lastReprojErrPx_.value(camId);
    if (e < 0) {
        boardReprojLabel_->setText(QStringLiteral(
            "重投影误差：预览计算失败或求解脚本暂不支持内参预览模式（不影响正常五步流程，"
            "第⑤步的正式求解不受影响）。"));
        return;
    }
    const QString grade = e <= 0.5 ? QStringLiteral("很好")
                          : e <= 1.0 ? QStringLiteral("可用")
                                     : QStringLiteral("偏高，建议继续补拍覆盖度提示里还薄弱的区域");
    boardReprojLabel_->setText(QStringLiteral("重投影误差（按当前已存 %1 张估算）：%2 px —— %3")
                                   .arg(shotCount_.value(camId, 0)).arg(e, 0, 'f', 3).arg(grade));
}

void CalibWizard::maybeRunIntrinsicsPreview(quint32 camId) {
    if (shotCount_.value(camId, 0) < 8) return;   // 太少张数算出来的误差没有参考意义
    if (intrinsicsProc_) return;                   // 上一次预览还没跑完，不重叠发起
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (now - lastIntrinsicsRunMs_.value(camId, 0) < kIntrinsicsPreviewCooldownMs) return;
    lastIntrinsicsRunMs_[camId] = now;

    const QString script = QCoreApplication::applicationDirPath() + "/solve_calibration.py";
    if (!QFile::exists(script)) return;   // 没有脚本就静默跳过，不打断拍摄流程
    const QString py = pyExe();
    if (py.isEmpty()) return;

    if (!ensureSession()) return;
    writeManifest();   // 求解脚本读板参数需要这份 manifest.json

    const int idx = folderIdx_.value(camId);
    intrinsicsProc_ = new QProcess(this);
    intrinsicsProc_->setProcessChannelMode(QProcess::MergedChannels);
    // 约定（需要 solve_calibration.py 配合支持，见随附说明）：
    //   python solve_calibration.py --intrinsics-only <会话目录> camN
    // 只对这一台相机跑 ChArUco 内参标定，不做外参/世界对齐，完成后往
    // stdout 打一行 "REPROJ_ERR <数值>"。脚本还没加这个模式时，下面的
    // 正则解析不到数字，界面老实显示"预览计算失败"，完全不影响第⑤步
    // 的正式五步求解流程。
    const quint32 capturedCamId = camId;
    connect(intrinsicsProc_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this, [this, capturedCamId](int code, QProcess::ExitStatus) {
                onIntrinsicsPreviewFinished(capturedCamId, code);
            });
    intrinsicsProc_->start(py, { script, "--intrinsics-only", sessionDir_, QString("cam%1").arg(idx) });
}

void CalibWizard::onIntrinsicsPreviewFinished(quint32 camId, int exitCode) {
    QProcess* proc = intrinsicsProc_;
    intrinsicsProc_ = nullptr;
    if (!proc) return;
    double err = -1.0;
    if (exitCode == 0) {
        const QString out = QString::fromLocal8Bit(proc->readAllStandardOutput());
        static const QRegularExpression re("REPROJ_ERR\\s+([0-9.]+)");
        const auto m = re.match(out);
        if (m.hasMatch()) err = m.captured(1).toDouble();
    }
    proc->deleteLater();
    if (err >= 0) lastReprojErrPx_[camId] = err;
    else if (!lastReprojErrPx_.contains(camId)) lastReprojErrPx_[camId] = -1;
    refreshReprojLabel();
}

// ---------- ②-点5：审查/剔除已拍照片 ----------
namespace {
// 审查照片对话框的撤销命令：redo() 把文件挪进 .trash（"删除"），undo() 挪
// 回原位（"恢复"）。跟 CalibrationLibraryDialog.cpp 里的 DeleteCameraCommand/
// DeleteTemplateCommand 是同一个模式——push() 之前只读快照不做真正删除，
// push() 触发 redo() 才真正执行。
class DeleteBoardShotCommand : public QUndoCommand {
public:
    DeleteBoardShotCommand(CalibWizard* wiz, quint32 camId, int cell, QString path, double sharpness,
                           QVector<int> coveredCells)
        : QUndoCommand(QStringLiteral("删除照片：%1").arg(QFileInfo(path).fileName())),
        wiz_(wiz), camId_(camId), cell_(cell), path_(std::move(path)), sharpness_(sharpness),
        coveredCells_(std::move(coveredCells)) {}
    void redo() override { wiz_->reviewTrashShot(camId_, cell_, path_); }
    void undo() override { wiz_->reviewRestoreShot(camId_, cell_, path_, sharpness_, coveredCells_); }
private:
    CalibWizard* wiz_;
    quint32 camId_;
    int cell_;
    QString path_;
    double sharpness_;
    QVector<int> coveredCells_;
};
} // namespace

void CalibWizard::onReviewBoardShots() {
    if (!camCombo_) return;
    const quint32 camId = camCombo_->currentData().toUInt();

    auto* dlg = new QDialog(this);
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setWindowTitle(QStringLiteral("审查已拍的标定板照片 —— %1").arg(camCombo_->currentText()));
    dlg->resize(760, 560);
    auto* v = new QVBoxLayout(dlg);
    v->addWidget(new QLabel(QStringLiteral(
        "勾选要删除的照片（比如板子被挡、明显糊、反光过曝），点下面的按钮批量删除，"
        "删错了 Ctrl+Z 撤销（跟标定模板库那边同一套逻辑）。删除后张数与覆盖度统计会自动更新。")));

    auto* list = new QListWidget;
    list->setViewMode(QListView::IconMode);
    list->setIconSize(QSize(140, 105));
    list->setResizeMode(QListView::Adjust);
    list->setSpacing(8);
    v->addWidget(list, 1);

    auto* undo = new QUndoStack(dlg);
    QAction* undoAction = undo->createUndoAction(dlg, QStringLiteral("撤销"));
    undoAction->setShortcut(QKeySequence::Undo);   // Ctrl+Z
    dlg->addAction(undoAction);

    // 缩略图缓存：撤销栈每变化一次（删除/撤销/重做）都会整个重建 list，原来
    // 每次都对"当前还在的每一张"重新 QIcon(path)——而 QIcon 在渲染时要从磁盘
    // 解码整张原始分辨率的 PNG 再缩到图标尺寸，纯 CPU 活、同步跑在主线程。
    // 张数一多（松散模式下九宫格没有硬上限，很容易几十张），每次点删除/撤销
    // 都要把所有还留着的照片重新解码一遍——这就是点删除/撤销会卡一下的原因。
    // 用 shared_ptr 包一层缓存：rebuildList 这个 lambda 后面会被
    // connect(undo,...) 里另一个 lambda 按值拷贝一份，两份拷贝必须共享
    // 同一份缓存数据（而不是各自复制一份 QHash），shared_ptr 保证这点。
    // 缓存 key 是文件路径——同一张照片撤销/恢复时路径不变（reviewRestoreShot
    // 是原样移回同一个 path），所以缓存在整个对话框生命周期内一直有效，
    // 不需要失效逻辑；对话框关闭后连同 shared_ptr 一起释放。
    auto thumbCache = std::make_shared<QHash<QString, QIcon>>();
    auto rebuildList = [this, list, camId, thumbCache] {
        list->clear();
        for (int cell = 0; cell < kCoverageGridN * kCoverageGridN; ++cell) {
            for (const auto& shot : boardCells_[camId][cell]) {
                auto cacheIt = thumbCache->find(shot.path);
                if (cacheIt == thumbCache->end()) {
                    const QPixmap pm = QPixmap(shot.path)
                    .scaled(140, 105, Qt::KeepAspectRatio, Qt::SmoothTransformation);
                    cacheIt = thumbCache->insert(shot.path, QIcon(pm));
                }
                auto* li = new QListWidgetItem(cacheIt.value(), QFileInfo(shot.path).fileName());
                li->setFlags(li->flags() | Qt::ItemIsUserCheckable);
                li->setCheckState(Qt::Unchecked);
                li->setData(Qt::UserRole, shot.path);
                li->setData(Qt::UserRole + 1, cell);
                list->addItem(li);
            }
        }
    };
    rebuildList();

    // 关键点跟 CalibrationLibraryDialog 那边一样：撤销/重做发生后，磁盘和
    // boardCells_ 已经改回去了，但列表控件不会自己知道要刷新——统一在
    // indexChanged 里重建整个列表（而不是在每个操作点各自记得刷新），
    // push/undo/redo 任何一种情况都会触发。
    connect(undo, &QUndoStack::indexChanged, dlg, [this, rebuildList, camId] {
        rebuildList();
        updateBoardCoverageLabel(camId);
        refreshBoardCountsLabel();
    });

    // 跟 CalibrationLibraryDialog.cpp 里那个"class destructor may have
    // already run"崩溃是同一个坑：undo 是 dlg 的子对象，如果 dlg 关闭时
    // 撤销栈里还有命令、index 不是 0，QUndoStack 自己析构时清空命令栈会
    // 重新触发一次 indexChanged，回调到上面那个连到 dlg 的 lambda——但
    // 这时候 dlg 已经在被 Qt 自动删子对象的阶段，动态类型已经不完整了，
    // 直接断言崩溃、把整个进程带走。dlg 是局部对话框，没有自己的类可以
    // 挂析构函数，改用 QDialog::finished 信号：不管是点"关闭"按钮
    // （accept）、按 Esc、还是点窗口的×（reject），都会同步触发这个信号，
    // 而且是在 WA_DeleteOnClose 真正把 dlg 排队销毁之前——这时候 dlg 还是
    // 完整、正确的类型，在这里主动断开连接是安全的。
    connect(dlg, &QDialog::finished, dlg, [undo, dlg](int) {
        disconnect(undo, nullptr, dlg, nullptr);
    });

    auto* btnRow = new QHBoxLayout;
    auto* delBtn = new QPushButton(QStringLiteral("删除勾选的照片"));
    auto* undoBtn = new QPushButton(QStringLiteral("撤销 (Ctrl+Z)"));
    auto* closeBtn = new QPushButton(QStringLiteral("关闭"));
    connect(undoBtn, &QPushButton::clicked, undo, &QUndoStack::undo);
    btnRow->addWidget(delBtn); btnRow->addWidget(undoBtn); btnRow->addStretch(1); btnRow->addWidget(closeBtn);
    v->addLayout(btnRow);
    connect(closeBtn, &QPushButton::clicked, dlg, &QDialog::accept);

    connect(delBtn, &QPushButton::clicked, this, [this, list, camId, undo] {
        QVector<QListWidgetItem*> checked;
        for (int i = 0; i < list->count(); ++i)
            if (list->item(i)->checkState() == Qt::Checked) checked << list->item(i);
        if (checked.isEmpty()) return;

        // 勾选的这一批打包成一个撤销单元——按一次 Ctrl+Z 就把这一批全部
        // 恢复，不用挨张撤销。
        undo->beginMacro(QStringLiteral("删除 %1 张照片").arg(checked.size()));
        for (auto* li : checked) {
            const QString path = li->data(Qt::UserRole).toString();
            const int cell = li->data(Qt::UserRole + 1).toInt();
            double sharpness = 0.0;
            QVector<int> coveredCells;
            for (const auto& shot : boardCells_[camId][cell])
                if (shot.path == path) { sharpness = shot.sharpness; coveredCells = shot.coveredCells; break; }
            undo->push(new DeleteBoardShotCommand(this, camId, cell, path, sharpness, coveredCells));
        }
        undo->endMacro();
        // push()/endMacro() 会触发上面连的 indexChanged，列表和计数已经在
        // 里面刷新过了，这里不用再手动做一遍。
    });

    dlg->exec();
}

// 把照片从 boardCells_ 摘除、文件移到同目录下的 .trash 子文件夹——用"移动"
// 代替"直接删除"，是为了配合上面的 Ctrl+Z：只要文件还在磁盘上（哪怕换了
// 个地方），undo 时原样移回来就行，不需要把图片字节缓存进内存假装"能
// 恢复"，也不需要真的永久销毁数据。
// 之所以是 CalibWizard 的公开方法而不是私有：DeleteBoardShotCommand（在
// 本文件的匿名命名空间里，不是 CalibWizard 的成员/友元）需要在 redo/undo
// 里调用它——跟 CalibrationLibraryDialog.cpp 里撤销命令调用
// CalibrationLibrary 公开方法的做法是同一个模式。
void CalibWizard::reviewTrashShot(quint32 camId, int cell, const QString& path) {
    auto& bucket = boardCells_[camId][cell];
    for (int k = 0; k < bucket.size(); ++k) {
        if (bucket[k].path != path) continue;
        const QString trashDir = QFileInfo(path).absolutePath() + "/.trash";
        QDir().mkpath(trashDir);
        const QString trashPath = trashDir + "/" + QFileInfo(path).fileName();
        if (QFile::rename(path, trashPath)) {
            bucket.removeAt(k);
            shotCount_[camId] = std::max(0, shotCount_.value(camId, 0) - 1);
            recomputeCoveredCells(camId);   // 删掉的可能是唯一覆盖某个格子的照片，覆盖度要诚实地退回去
            updateBoardCoverageLabel(camId);
        }
        return;
    }
}

void CalibWizard::reviewRestoreShot(quint32 camId, int cell, const QString& path, double sharpness,
                                    const QVector<int>& coveredCells) {
    const QString trashPath = QFileInfo(path).absolutePath() + "/.trash/" + QFileInfo(path).fileName();
    if (!QFile::exists(trashPath)) return;   // 只有真被 reviewTrashShot() 挪走过的路径才会走到这
    if (QFile::rename(trashPath, path)) {
        boardCells_[camId][cell].push_back({path, sharpness, coveredCells});
        shotCount_[camId] = shotCount_.value(camId, 0) + 1;
        recomputeCoveredCells(camId);
        updateBoardCoverageLabel(camId);
    }
}



// ---------- ③ 挥球 ----------
void CalibWizard::onToggleWand() {
    wandRecording_ = !wandRecording_;
    if (wandRecording_) {
        wandLines_.clear(); wandN_.clear();
        wandLastTs_.clear(); wandCoDone_ = 0;
        wandBtn_->setText(QStringLiteral("⏸ 停止记录"));
    } else {
        wandBtn_->setText(QStringLiteral("▶ 开始记录"));
        flushWand();
    }
}

void CalibWizard::flushWand() {
    QFile f(sessionDir_ + "/wand_blobs.jsonl");
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        for (const QString& l : wandLines_) { f.write(l.toUtf8()); f.write("\n"); }
    }
}

// 共视样本计数：跟踪每台相机"最近一次样本"的时间戳；每来一个新样本，
// 检查是否全部参与标定的相机都已经有过样本、且这些最近样本互相之间的
// 时间跨度落在容忍窗口内——是就算一份共视样本，然后清空重新累积（避免
// 同一批样本被反复计数）。比固定网格分桶更稳，不受桶边界效应影响。
void CalibWizard::registerWandSample(quint32 camId, qint64 ts_ns) {
    wandLastTs_[camId] = ts_ns;
    if (cams_.isEmpty() || wandLastTs_.size() < cams_.size()) return;   // 还没集齐全部相机的至少一个样本

    // QHash 不像 QMap 那样有 first()（无序容器，"第一个"没意义），用
    // constBegin() 迭代器取初值。
    auto it = wandLastTs_.constBegin();
    qint64 minTs = it.value(), maxTs = it.value();
    for (; it != wandLastTs_.constEnd(); ++it) {
        minTs = std::min(minTs, it.value());
        maxTs = std::max(maxTs, it.value());
    }

    if (maxTs - minTs <= kWandToleranceNs) {
        ++wandCoDone_;
        wandLastTs_.clear();   // 消费掉这一批，下一份共视样本重新累积
    }
    // 没对齐就什么都不做——继续等其它相机的下一帧追上来，下次新样本到达
    // 时会用最新的时间戳重新判断；每台相机的记录会被后续新样本自然覆盖，
    // 不需要显式的"过期丢弃"逻辑，也不会无限增长。
}

// ---------- ④ 支架捕获 ----------
void CalibWizard::onCaptureFrame() {
    frameSamples_.clear();
    frameCollecting_ = true;
    frameStatus_->setText(QStringLiteral("正在采样…保持支架与相机不动"));
    QTimer::singleShot(900, this, [this] {
        frameCollecting_ = false;
        frameSnap_.clear();
        QStringList ok, miss;
        for (ICamera* c : cams_) {
            const auto samples = frameSamples_.value(c->id());
            if (samples.size() < 5) { miss << c->name(); continue; }
            // 以第一帧为参考，把每帧的 4 点按最近邻对齐到参考顺序后求平均，
            // 消除逐帧输出顺序不稳带来的抖动。
            const QVector<QPointF> ref = samples.first();
            QVector<QPointF> acc(4, QPointF(0, 0));
            for (const auto& s : samples) {
                QVector<bool> used(4, false);
                for (int r = 0; r < 4; ++r) {
                    int best = -1; double bd = 1e18;
                    for (int k = 0; k < 4; ++k) {
                        if (used[k]) continue;
                        const double d = QLineF(ref[r], s[k]).length();
                        if (d < bd) { bd = d; best = k; }
                    }
                    used[best] = true;
                    acc[r] += s[best];
                }
            }
            QVector<QPointF> avg;
            for (const QPointF& a : acc) avg << a / double(samples.size());
            frameSnap_[c->id()] = avg;
            ok << c->name();
        }
        writeFrameSnapshot();
        frameStatus_->setText(QStringLiteral("已捕获：%1%2")
                                  .arg(ok.join("、"))
                                  .arg(miss.isEmpty() ? QString()
                                                      : QStringLiteral("   ⚠ 未捕到 4 球：%1（检查视野/阈值后重拍）").arg(miss.join("、"))));
    });
}

void CalibWizard::writeFrameSnapshot() {
    QJsonObject root;
    for (auto it = frameSnap_.constBegin(); it != frameSnap_.constEnd(); ++it) {
        QJsonArray pts;
        for (const QPointF& p : it.value()) {
            QJsonArray xy; xy << p.x() << p.y(); pts << xy;
        }
        root[keyOf_.value(it.key())] = pts;
    }
    QFile f(sessionDir_ + "/frame_snapshot.json");
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    writeManifest();   // 世界坐标可能被用户改过，一并更新
}


// 【拆文件】标定板接力/求解相关成员已移到 CalibWizard_rounds.cpp（原 1611 行起）。
} // namespace mocap
