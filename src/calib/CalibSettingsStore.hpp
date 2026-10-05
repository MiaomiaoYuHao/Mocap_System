#pragma once
// ---------------------------------------------------------------------------
// 标定向导里最容易被反复重填的两块设置：
//   - ChArUco 板参数（字典/格子数/方格边长/码边长）——换一次标定板、或者像
//     这次你们排查 calib.io 板子问题时，来回改字典/边长测试，每次开向导
//     都要重填一遍。
//   - 世界坐标支架四点表——你的支架是固定的物理硬件，量一次就不会变，
//     没理由每次开向导都要重新敲一遍数字。
// 跟 CalibrationStore / ParamPresets 是同一套模式：落盘 JSON，存到用户配置
// 目录，重启不丢。不含"会话保存目录"这种本来就该每次换新路径的字段。
// ---------------------------------------------------------------------------
#include <QString>
#include <array>

namespace mocap {

struct BoardParams {
    QString dict = "DICT_5X5_100";
    int squaresX = 11, squaresY = 8;
    double squareMM = 30.0, markerMM = 22.0;
};

class CalibSettingsStore {
public:
    explicit CalibSettingsStore(const QString& path = QString());

    BoardParams board() const { return board_; }
    void setBoard(const BoardParams& b) { board_ = b; save(); }

    // 4 行 x 3 列（原点/X轴点/Y轴点/Z轴点 各自的 X/Y/Z，单位 mm）。
    std::array<std::array<double, 3>, 4> world() const { return world_; }
    void setWorld(const std::array<std::array<double, 3>, 4>& w) { world_ = w; save(); }

    // 外参求解方式：true=标定板接力（推荐，更稳），false=挥反光球（原方式）。
    // 这个也是"填一次记住重启不丢"的偏好，跟板参数/世界坐标同一套持久化。
    bool useBoardExtrinsics() const { return useBoardExtrinsics_; }
    void setUseBoardExtrinsics(bool on) { useBoardExtrinsics_ = on; save(); }

    bool load();
    bool save() const;

private:
    QString path_;
    BoardParams board_;
    std::array<std::array<double, 3>, 4> world_ = {{ {0,0,0}, {100,0,0}, {0,100,0}, {0,0,100} }};
    bool useBoardExtrinsics_ = false;   // 默认挥球方式（向后兼容，老用户习惯不变）
};

} // namespace mocap
