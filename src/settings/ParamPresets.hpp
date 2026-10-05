#pragma once
// ---------------------------------------------------------------------------
// 相机硬件参数模板（快照式）。三套：
//   Board —— “标定板”参数：默认状态即可拍清 ChArUco 黑白格
//   Track —— “追踪”参数：调到背景黑、反光球白
//   Boot  —— “开机默认”：菜单里「存为开机默认」一键快照，开机自动套回去。
//            前两套是【手动切换】的工况模板，这一套是【开机自动应用】的，
//            用户不会去点它，所以别把它放进「应用 ▾」下拉里。
// 用法：先用参数对话框把画面调满意 -> “存为模板”一键快照全部相机的
// DirectShow 属性（值 + 自动标志）；之后一键应用，全部相机立即生效。
// 按 deviceKey 保存，落盘 JSON，重启不丢。标定向导按步骤自动切换模板。
// ---------------------------------------------------------------------------
#include <QString>
#include <QHash>
#include <QVector>
#include <QList>

namespace mocap {

class WebcamCamera;

class ParamPresets {
public:
    // 【加新槽位就得同步三处】enum、sets_ 的维度、.cpp 里的 kNames[]。
    // 漏掉任何一处都是越界或静默读不到，编译器不会提醒。
    enum Which { Board = 0, Track = 1, Boot = 2, WhichCount = 3 };

    ParamPresets();   // 自动从用户配置目录载入 param_presets.json

    // 把这些相机“当前”的全部硬件参数快照进模板 w。返回成功保存的相机数。
    int captureAll(Which w, const QList<WebcamCamera*>& cams);

    // 把模板 w 应用到这些相机（逐属性下发驱动，立即生效）。
    // 返回成功应用的相机数；没有保存记录的相机跳过。
    int applyAll(Which w, const QList<WebcamCamera*>& cams) const;

    // 单台相机版。【开机恢复要用这个，不能用 applyAll】开机时每台相机的
    // 采集引擎是各自异步起来的，谁先起来谁先套 —— 拿整份列表去 applyAll
    // 会对还没起来的相机白推一遍（那时推下去的值会被它自己的建图冲掉）。
    // 见 MainWindow::hookCamera() 里接 captureEngineStarted 的那段。
    bool applyTo(Which w, WebcamCamera* cam) const;

    // 这台相机在模板 w 里有没有记录。has() 是"整套模板非空"，
    // 这个是"具体这一台有没有"——七台相机里只存过三台时两者不是一回事。
    bool hasFor(Which w, const QString& deviceKey) const;

    bool has(Which w) const { return !sets_[w].isEmpty(); }
    bool save() const;
    QString filePath() const { return path_; }

private:
    struct P { long id; bool cc; long v; bool a; };   // 属性id/是否CameraControl/值/自动
    QHash<QString, QVector<P>> sets_[WhichCount];     // deviceKey -> 属性列表
    QString path_;
    void load();
};

} // namespace mocap
