#pragma once
// ---------------------------------------------------------------------------
// 手部模板持久化：手背5点模板 + 5根手指结构参数(anchor+lengths)，落盘为
// JSON。跟 calib/CalibrationStore.hpp 是同一个模式(deviceKey按相机分标定，
// 这里按"手"分模板，目前先做单模板——一个应用同时只認一套手部模板，够
// 单人多次标定复用；多人/多套glove的场景需要再加一层"选择哪个模板"的
// key索引，跟 CalibrationStore 加 remap 是同一个扩展方向，先不做)。
//
// 【这是修复的那个真实缺口】之前 HandCalibration.hpp/HandSelfCalibration.hpp
// 标定出来的结果，因为没有地方存、也没有地方读，等于白标定——
// HandColdStart.hpp 之前一直硬编码用 HandModel.hpp 的占位值。现在
// HandTrackingWorker 从这里读模板，HandColdStart 用读到的模板做匹配，
// HandModelAdapter 用读到的手指结构参数做FK——标定结果才真正生效。
// ---------------------------------------------------------------------------
#include "hand/HandPose.hpp"     // HandVec3
#include "hand/HandModel.hpp"    // FingerParam
#include <QString>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QStandardPaths>
#include <QDir>
#include <array>

namespace mocap {

struct HandTemplateData {
    std::array<HandVec3, 5> backMarkers{};        // 手背5点模板(手腕局部系)
    bool backCalibrated = false;

    std::array<FingerParam, 5> fingerParams{};    // [0]拇指 [1]食 [2]中 [3]无名 [4]小指
    std::array<bool, 5> fingerCalibrated{false,false,false,false,false};

    // ---- 拇指的两个逐人常数 ----
    // 【为什么必须存盘】这两个都是解剖常数，一个人一辈子不变，但原来一个都没存，
    // 每次开程序都要重新用束调整解一遍 —— 而束调整能不能解出来取决于这一轮
    // 用户有没有把拇指摆够，所以同一只手在不同会话里会拿到不同的值。
    // thumbAxialK 一直是这个状况（老问题），thumbPronation0 更不能重蹈覆辙：
    // 它量级 ~80°，解错一次拇指就整个外翻。
    double thumbAxialK = 0.576;              // 轴向旋前【耦合】系数
    double thumbPronation0 = 0.0;            // 第一掌骨【常数】解剖旋前(rad)
    bool   thumbPronationCalibrated = false; // false = 还没标出来，用 0(等价旧行为)

    // 全部标定完成(手背+5根手指)才算"完整"——UI应该用这个字段决定是否
    // 提示"标定还没做完，当前在用占位值/部分占位值"。
    bool isComplete() const {
        if (!backCalibrated) return false;
        for (bool b : fingerCalibrated) if (!b) return false;
        return true;
    }
};

class HandTemplateStore {
public:
    explicit HandTemplateStore(const QString& path = QString()) {
        if (!path.isEmpty()) {
            path_ = path;
        } else {
            QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
            if (dir.isEmpty()) dir = QDir::homePath();
            QDir().mkpath(dir);
            path_ = dir + "/hand_template.json";
        }
        load();
    }

    const HandTemplateData& data() const { return data_; }

    void setBackTemplate(const std::array<HandVec3,5>& templ) {
        data_.backMarkers = templ;
        data_.backCalibrated = true;
        save();
    }

    // fingerIdx: 0=拇指 1=食 2=中 3=无名 4=小指(跟 HandModel.hpp 的手指
    // 编号约定一致，见 handFK 里 fingerParam(1+f) 那个偏移)。
    void setFingerParam(int fingerIdx, const FingerParam& p) {
        if (fingerIdx < 0 || fingerIdx >= 5) return;
        data_.fingerParams[size_t(fingerIdx)] = p;
        data_.fingerCalibrated[size_t(fingerIdx)] = true;
        save();
    }

    // 重置成占位值(比如标定坏了想重来，或者临时切回默认值验证问题是不是
    // 标定引入的)。不删文件，只是把内存态和落盘态都清空重存。
    void resetToPlaceholders() {
        data_ = HandTemplateData{};
        data_.backMarkers = handBackMarkers();
        for (int i=0;i<5;++i) data_.fingerParams[size_t(i)] = fingerParam(i);
        // 注意：backCalibrated/fingerCalibrated 保持false——占位值终究是
        // 占位值，不应该被标记成"已标定"，即使数值上跟占位表一样。
        save();
    }

    bool load() {
        QFile f(path_);
        if (!f.open(QIODevice::ReadOnly)) { seedPlaceholdersIfEmpty(); return false; }
        const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
        if (!doc.isObject()) { seedPlaceholdersIfEmpty(); return false; }
        const QJsonObject root = doc.object();

        data_.backCalibrated = root["backCalibrated"].toBool(false);
        // 【老文件没有这几个键】toDouble/toBool 的默认值就是"没标定过"的语义，
        // 所以旧的 hand_template.json 直接读也不会出问题。
        data_.thumbAxialK = root["thumbAxialK"].toDouble(0.576);
        data_.thumbPronation0 = root["thumbPronation0"].toDouble(0.0);
        data_.thumbPronationCalibrated = root["thumbPronationCalibrated"].toBool(false);
        const QJsonArray backArr = root["backMarkers"].toArray();
        for (int i=0;i<5 && i<backArr.size();++i) {
            const QJsonArray p = backArr[i].toArray();
            for (int d=0; d<3 && d<p.size(); ++d) data_.backMarkers[size_t(i)][size_t(d)] = p[d].toDouble();
        }

        const QJsonArray fingersArr = root["fingers"].toArray();
        for (int i=0;i<5 && i<fingersArr.size();++i) {
            const QJsonObject fo = fingersArr[i].toObject();
            data_.fingerCalibrated[size_t(i)] = fo["calibrated"].toBool(false);
            const QJsonArray anchorArr = fo["anchor"].toArray();
            const QJsonArray lengthsArr = fo["lengths"].toArray();
            for (int d=0; d<3 && d<anchorArr.size(); ++d) data_.fingerParams[size_t(i)].anchor[size_t(d)] = anchorArr[d].toDouble();
            for (int d=0; d<3 && d<lengthsArr.size(); ++d) data_.fingerParams[size_t(i)].lengths[size_t(d)] = lengthsArr[d].toDouble();
            // 旧版本存的模板文件没有这个字段——用 FingerParam 的默认成员初始值
            // (0.7，见 HandModel.hpp)当回退，不报错、不当成加载失败。
            data_.fingerParams[size_t(i)].dipCoupling = fo.contains("dipCoupling")
                ? fo["dipCoupling"].toDouble() : FingerParam{}.dipCoupling;
        }

        // 没标定过的槽位(手背或某根手指)用占位值填，保证 data_ 任何时候
        // 都有一份可用的(哪怕不准的)几何数据，调用方不用另外判空。
        if (!data_.backCalibrated) data_.backMarkers = handBackMarkers();
        for (int i=0;i<5;++i) if (!data_.fingerCalibrated[size_t(i)]) data_.fingerParams[size_t(i)] = fingerParam(i);
        return true;
    }

    bool save() const {
        QJsonObject root;
        // v2 = 多了 thumbAxialK / thumbPronation0 / thumbPronationCalibrated。
        // 【故意不做版本闸】新增键读不到时 toDouble/toBool 的默认值就是"没标过"，
        // 旧文件直接读没问题；反过来旧程序读新文件也只是忽略这三个键。
        // 加个硬版本检查只会让两边互相读不了，没有任何好处。
        root["version"] = 2;
        root["backCalibrated"] = data_.backCalibrated;
        QJsonArray backArr;
        for (const auto& p : data_.backMarkers) {
            QJsonArray pa; pa.append(p[0]); pa.append(p[1]); pa.append(p[2]);
            backArr.append(pa);
        }
        root["backMarkers"] = backArr;
        root["thumbAxialK"] = data_.thumbAxialK;
        root["thumbPronation0"] = data_.thumbPronation0;
        root["thumbPronationCalibrated"] = data_.thumbPronationCalibrated;

        QJsonArray fingersArr;
        for (int i=0;i<5;++i) {
            QJsonObject fo;
            fo["calibrated"] = data_.fingerCalibrated[size_t(i)];
            QJsonArray anchorArr, lengthsArr;
            for (double v : data_.fingerParams[size_t(i)].anchor) anchorArr.append(v);
            for (double v : data_.fingerParams[size_t(i)].lengths) lengthsArr.append(v);
            fo["anchor"] = anchorArr; fo["lengths"] = lengthsArr;
            fo["dipCoupling"] = data_.fingerParams[size_t(i)].dipCoupling;
            fingersArr.append(fo);
        }
        root["fingers"] = fingersArr;

        QFile f(path_);
        if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
        return true;
    }

    QString filePath() const { return path_; }

private:
    void seedPlaceholdersIfEmpty() {
        data_.backMarkers = handBackMarkers();
        for (int i=0;i<5;++i) data_.fingerParams[size_t(i)] = fingerParam(i);
    }

    QString path_;
    HandTemplateData data_;
};

} // namespace mocap
