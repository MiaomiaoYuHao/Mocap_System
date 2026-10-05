#pragma once
// ---------------------------------------------------------------------------
// 傻瓜式标定向导：
//   ① 准备        —— 选会话保存位置、填 ChArUco 板参数、选外参求解方式
//   ② 拍标定板    —— 逐台相机对着板子拍 15+ 张（不同角度/距离/覆盖四角）
//   ③ 外参采集，二选一（第①步选的）：
//        · 标定板接力（推荐）—— 把板子摆到任意两台相机的共视区拍一轮，
//          挪到下一对共视区再拍一轮，不要求所有相机同时看到，只要求
//          "接力连通"；其中一轮标记为"世界锚定"，直接定义世界坐标系。
//        · 挥反光球（原方式）—— 手持单颗反光球在相机共视区挥动积累样本，
//          配合第④步的坐标支架做世界对齐。
//   ④ 拍坐标支架（仅挥球方式需要）——摆好原点+XYZ 四球支架，一键捕获
//   ⑤ 求解并完成  —— 自动调用求解脚本算出全部内外参，导入标定库，查看结果
//
// 两条外参路径完全独立、并存，不是谁取代谁——标定板接力数学上更稳（一帧
// 静止拍摄就是完整6自由度解，不存在单球本质矩阵那类退化风险，实测验证
// 见 solve_calibration.py 里 solve_board_extrinsics() 的说明），挥球方式
// 继续保留作为兜底/习惯用法。
//
// 设计取向：所有采集数据落盘到一个"会话文件夹"（图片 + 观测 JSON），求解由
// 随程序附带的 solve_calibration.py（OpenCV 参考实现）完成——标定数学
// （ChArUco 角点、Zhang 内参、PnP/本质矩阵、位姿图优化）用行业标准库最可靠。
// 机器上装好 Python + opencv-contrib-python 后，向导内一键求解、自动导入。
//
// 外参方式的初值在向导构造时从 CalibSettingsStore 读一次，但向导是分支式的：
// 构造时两套页面（标定板接力 / 挥球）都建好，第③步实际走哪条由 nextId() 按
// 当前开关值路由。所以第①步的开关本次就生效——勾选/取消当场改变第③步走哪个
// 页面，同时记住作为下次默认。（早期版本受限于"页面构造时一次性加好"只能
// 下次生效，现已改为分支路由。）
// ---------------------------------------------------------------------------
#include <QWizard>
#include <QHash>
#include <QSet>
#include <QVector>
#include <QPointF>
#include <QImage>
#include <QElapsedTimer>
#include <array>

class QComboBox;
class QLabel;
class QLineEdit;
class QSpinBox;
class QDoubleSpinBox;
class QPushButton;
class QPlainTextEdit;
class QTableWidget;
class QProcess;
class QTimer;
class QListWidget;
class QListWidgetItem;
class QCheckBox;

namespace mocap {

    class CameraManager;
    class CalibrationStore;
    class ParamPresets;
    class ICamera;

    // 小型预览面板：画最新帧 + 检测十字丝（向导内用，无交互）。
    class PreviewPane : public QWidget {
        Q_OBJECT
    public:
        explicit PreviewPane(QWidget* parent = nullptr);
        void setImage(const QImage& img);
        void setBlobs(const QVector<QPointF>& pts);
    protected:
        void paintEvent(QPaintEvent*) override;
    private:
        QImage img_;
        QVector<QPointF> blobs_;
    };

    class CalibWizard : public QWizard {
        Q_OBJECT
    public:
        CalibWizard(CameraManager* mgr, CalibrationStore* store,
            ParamPresets* presets = nullptr, QWidget* parent = nullptr);
        ~CalibWizard() override;

        // ② 审查照片对话框的撤销命令（CalibWizard.cpp 里的
        // DeleteBoardShotCommand）调用这两个——"删除"实为把文件挪进同目录
        // 下的 .trash 子文件夹，"撤销"就是挪回来，不需要真的销毁数据。
        // 公开是因为调用方是本文件匿名命名空间里的类，不是成员/友元，
        // 跟 CalibrationLibrary 那边撤销命令调用公开方法是同一个模式。
        void reviewTrashShot(quint32 camId, int cell, const QString& path);
        void reviewRestoreShot(quint32 camId, int cell, const QString& path, double sharpness,
                               const QVector<int>& coveredCells);

    private slots:
        void onFrame(quint32 camId, const QImage& img, double fps);
        void onBlobs(quint32 camId, const QVector<QPointF>& pts, qint64 ts_ns);
        void onCaptureBoard();      // ② 拍一张标定板
        void onReviewBoardShots();  // ② 缩略图审查/剔除已拍照片
        void onToggleWand();        // ③ 开始/停止挥球记录（挥球方式）
        void onCaptureFrame();      // ④ 捕获坐标支架（挥球方式）
        void onCaptureRound();      // ③ 拍摄一轮标定板接力（标定板方式）
        void onSetAnchorRound();    // ③ 把选中的轮次标记为世界锚定轮（标定板方式）
        void onDeleteRound();       // ③ 删除选中的轮次（标定板方式）
        void onSolve();             // ⑤ 运行求解脚本
        void onSolveFinished(int exitCode);
        void onIntrinsicsPreviewFinished(quint32 camId, int exitCode);   // ② 当场内参预览完成

    protected:
        // 严格样本门槛：第②步（张数+覆盖度）、第③步（共视样本数）不达标
        // 就拦住"下一步"，避免"人力都投入了，结果卡在最后一步求解才发现
        // 某台相机不达标，前面全部白走"。
        bool validateCurrentPage() override;

        // 分支式向导的路由：按当前 boardExtrinsicsMode_ 决定第②步之后走
        // 「标定板接力」还是「挥球」两条页面链——让第①步的模式开关本次即生效。
        int nextId() const override;

    private:
        // —— 数据 ——
        CameraManager* mgr_;
        CalibrationStore* store_;
        ParamPresets* presets_ = nullptr;
        QList<ICamera*>   cams_;            // 参与标定的真实相机
        QHash<quint32, QString> keyOf_;     // camId -> deviceKey
        QHash<quint32, int>     folderIdx_; // camId -> camN 文件夹序号
        QHash<quint32, bool>    prevDetect_;// 进向导前的检测开关，退出时恢复
        QString sessionDir_;

        // 外参求解方式：构造时从 CalibSettingsStore 读一次、之后整个向导
        // 生命周期不变（QWizard 页面是构造时一次性加好的）。true=标定板
        // 接力，false=挥球（原方式）。boardRoundsPageId_/wandPageId_/
        // framePageId_/solvePageId_ 是这次向导实际的页号——两种模式下页面
        // 数量不一样，凡是"进入某一步该做什么"的判断都要用这些成员，不能
        // 硬编码数字，否则两种模式会互相踩。
        bool boardExtrinsicsMode_ = false;
        int  boardRoundsPageId_ = -1;   // 仅标定板模式有效
        int  wandPageId_ = -1;          // 仅挥球模式有效
        int  framePageId_ = -1;         // 仅挥球模式有效
        int  solvePageId_ = -1;         // 两种模式都有效

        // ③ 标定板接力（标定板模式）
        QComboBox* roundPreviewCombo_ = nullptr;
        PreviewPane* roundPreview_ = nullptr;
        QHash<quint32, PreviewPane*> roundPanes_;      // 网格预览：每台相机一个小画面（camId -> pane）
        QHash<quint32, QLabel*>      roundPaneTitle_;  // 对应的标题/状态条（camId -> label）
        QListWidget* roundCamList_ = nullptr;     // 这一轮谁参与（勾选）
        QPushButton* captureRoundBtn_ = nullptr;
        QListWidget* roundsList_ = nullptr;       // 已拍的轮次
        QPushButton* setAnchorBtn_ = nullptr;
        QPushButton* deleteRoundBtn_ = nullptr;
        QTableWidget* covisTable_ = nullptr;      // 共视矩阵 N×N
        QLabel* connectivityLabel_ = nullptr;
        QVector<QString> roundIds_;                // 已拍轮次 id，按拍摄顺序
        QHash<QString, QVector<quint32>> roundCams_; // roundId -> 参与的 camId 列表
        // roundId -> (camId -> 检测到的 ChArUco 角点数)。只存"真看到板子"的相机
        // （角点数>0），求解阶段前就能看出这一轮质量好不好，不用等到第⑤步。
        QHash<QString, QHash<quint32, int>> roundCamQuality_;
        QString anchorRoundId_;                     // 已标记的世界锚定轮，空=未标记
        int nextRoundSeq_ = 0;                      // 轮次文件夹命名用的单调递增序号
        bool detectUnavailWarned_ = false;          // 板子检测不可用的提示：本次向导只弹一次

        // ② 内参采集
        QComboBox* camCombo_ = nullptr;
        PreviewPane* boardPreview_ = nullptr;
        QLabel* boardCounts_ = nullptr;
        QHash<quint32, QImage> lastFrame_;
        QHash<quint32, int>    shotCount_;        // 当前保留（未被选拔机制淘汰）的张数
        QHash<quint32, int>    nextShotSeq_;       // 文件名用的单调递增序号，只增不减，避免删旧存新时文件名碰撞
        QHash<quint32, QSize>  frameSize_;
        // 自动连拍（举着标定板移动，程序按节奏自动存图，太糊的帧自动跳过）。
        QPushButton* boardAutoBtn_ = nullptr;
        QLabel* boardAutoStatus_ = nullptr;
        bool         boardAutoOn_ = false;
        QHash<quint32, qint64> lastAutoAttemptMs_;

        // ② 覆盖度反馈 + 选拔机制：camId -> 3x3 九宫格，每格保留清晰度最高
        // （历史：这里曾有"严格选拔模式"的每格限量，已移除）
        // （快速模式，等价于最早的"只挡糊图，不限制数量/位置"行为）——
        // 选拔+覆盖度硬门槛实测会让拍板明显变慢，改成用户自己勾选的可选
        // 项，两种模式都保留。
        struct BoardShot { QString path; double sharpness = 0.0; QVector<int> coveredCells; };
        QHash<quint32, std::array<QVector<BoardShot>, 9>> boardCells_;
        // camId -> 9宫格覆盖标记，由 recomputeCoveredCells() 从 boardCells_
        // 当前实际保留的照片重新推算(不是只增不减的旗标)——每张照片记着自己
        // 外接框覆盖到的格子列表(BoardShot::coveredCells)，一格只要被"当前
        // 还在的"任意一张照片覆盖到就算数；照片被审查删光了，对应格子会
        // 诚实地退回未覆盖，不会出现"数据其实没了但门槛还显示达标"的假象。
        QHash<quint32, std::array<bool, 9>> coveredCells_;
        void recomputeCoveredCells(quint32 camId);
        QLabel* boardCoverageLabel_ = nullptr;
        QPushButton* reviewBtn_ = nullptr;


        // ② 当场内参预览：攒够几张后台跑一次纯内参标定，把重投影误差显示
        // 出来（依赖 solve_calibration.py 支持 --intrinsics-only 模式，见
        // CalibWizard.cpp 里 maybeRunIntrinsicsPreview() 的注释）。
        QLabel* boardReprojLabel_ = nullptr;
        QProcess* intrinsicsProc_ = nullptr;
        QHash<quint32, double> lastReprojErrPx_;      // -1 = 算过但失败；不存在 = 还没算过
        QHash<quint32, qint64> lastIntrinsicsRunMs_;  // 节流用

        // ③ 挥球
        QPushButton* wandBtn_ = nullptr;
        QLabel* wandCounts_ = nullptr;
        QHash<quint32, PreviewPane*> wandPanes_;      // 网格预览：跟接力页对齐，两模式都能看清各台画面
        QHash<quint32, QLabel*>      wandPaneTitle_;
        bool         wandRecording_ = false;
        QStringList  wandLines_;                 // jsonl 行缓冲
        QHash<quint32, int> wandN_;
        // 共视有效样本：所有相机同时看到球的样本数（三角化真正用得上的
        // 量），跟 wandN_（各相机各自独立计数）是两回事。wandLastTs_ 记录
        // 每台相机"最近一次样本"的时间戳，用滑动容忍窗口判断共视，见
        // .cpp 里 registerWandSample() 的注释。
        QHash<quint32, qint64> wandLastTs_;
        int wandCoDone_ = 0;
        QTimer* uiTimer_ = nullptr;         // 低频刷新计数标签 + 活性状态条

        // Layer 1 活性检测（两模式通用）：camId -> 最近一次收到帧的时刻/大致
        // 亮度。只用来判"这台还在不在出帧、是不是黑屏"，防的是"勾了却没画面"
        // 这种低级错误；防不了遮挡，那是 detectBoardFolders 的职责。
        QHash<quint32, qint64> lastFrameMs_;
        QHash<quint32, double> lastLuma_;

        // ④ 支架
        QTableWidget* worldTable_ = nullptr;
        QLabel* frameStatus_ = nullptr;
        bool          frameCollecting_ = false;
        QHash<quint32, QVector<QVector<QPointF>>> frameSamples_;
        QHash<quint32, QVector<QPointF>>          frameSnap_;

        // ⑤ 求解
        QPlainTextEdit* log_ = nullptr;
        QPushButton* solveBtn_ = nullptr;
        QPushButton* viewBtn_ = nullptr;
        QProcess* proc_ = nullptr;

        // ① 准备页控件
        QLineEdit* dirEdit_ = nullptr;
        QSpinBox* bSx_ = nullptr, * bSy_ = nullptr;
        QDoubleSpinBox* bSq_ = nullptr, * bMk_ = nullptr;
        QComboBox* bDict_ = nullptr;

        // —— 页构建 ——
        QWizardPage* pageIntro();
        QWizardPage* pageBoard();
        QWizardPage* pageWand();
        QWizardPage* pageFrame();
        QWizardPage* pageBoardRounds();
        QWizardPage* pageSolve();

        void enterPage(int id);
        void setAllDetect(bool on);
        bool ensureSession();               // 建目录 + 写 manifest.json
        void writeManifest();
        void flushWand();
        void writeFrameSnapshot();
        // ②手动/自动连拍共用的落盘逻辑；fromAuto=true 时走"选拔机制"（见
        // .cpp）。返回是否真的存了盘——选拔机制跳过时返回 false。
        bool saveBoardShot(quint32 camId, const QImage& img, double sharpness, bool fromAuto);
        QString pyExe() const;

        // ② 覆盖度 / 内参预览 / 审查照片
        // 底部「每台相机拍了几张」汇总条。三个调用点（翻页/自动连拍/拍照）
        // 共用，别再各抄一份 —— 抄三份的后果就是改格式漏一处。
        void refreshBoardCountsLabel();
        void updateBoardCoverageLabel(quint32 camId);
        void refreshReprojLabel();
        void maybeRunIntrinsicsPreview(quint32 camId);

        // ③ 共视样本计数
        void registerWandSample(quint32 camId, qint64 ts_ns);

        // 严格样本门槛的判定，供 validateCurrentPage() 调用
        bool boardStepReady(QString* reason) const;
        bool wandStepReady(QString* reason) const;

        // ③ 标定板接力（标定板模式）
        void refreshRoundsUI();             // 重建轮次列表 + 共视矩阵 + 连通性判断
        bool roundsStepReady(QString* reason) const;   // 连通+已标记锚定轮 才算达标
        // 拍完一轮后，跑 ChArUco 检测拿到每台相机检出的角点数（folder序号 -> 角点数，
        // 0=没检到/被挡住）。只有角点数>0 的相机才计入共视，同时这个数也直接反映
        // 这一轮的质量，供 refreshRoundsUI 在轮次列表里显示，不用等第⑤步求解。
        QHash<int, int> detectBoardFolders(const QString& roundId, QString* err) const;

        // Layer 1 活性状态条：按当前页（接力/挥球）刷新对应网格预览标题的
        // 文字和底色（在线/掉线/黑屏/已勾选/录制中）。uiTimer_ 250ms 调一次。
        void refreshActivityStatuses();
    };

} // namespace mocap