#include "ui/CamParamDialog.hpp"
#include "camera/ICamera.hpp"
#include "camera/WebcamCamera.hpp"
#include "settings/AppSettings.hpp"
#include <QVBoxLayout>
#include <QGridLayout>
#include <QTabWidget>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QLocale>
#include <QTimer>
#include <QPointer>
#include <algorithm>
#include <cmath>

namespace mocap {

// 数值框统一走 C locale：部分 Windows 区域/数字格式设置下，QSpinBox 若跟随
// 系统 locale 会用非拉丁数字符号绘制数值，导致显示花掉。这里强制用阿拉伯数字，
// 并给足宽度，避免数值与上下调节箭头重叠挤压。
static void fixSpinBoxDisplay(QSpinBox* b) {
    b->setLocale(QLocale::c());
    b->setMinimumWidth(88);
    b->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    b->setButtonSymbols(QAbstractSpinBox::UpDownArrows);
}

CamParamDialog::CamParamDialog(ICamera* cam, QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(cam->name() + QString::fromUtf8(" \u2014 \u5c5e\u6027"));  // — 属性
    auto* lay = new QVBoxLayout(this);
    auto* tabs = new QTabWidget;
    lay->addWidget(tabs, 1);

    web_ = qobject_cast<WebcamCamera*>(cam);
    if (web_) {
        // 红外相机常常型号一致、驱动上报的友好名字完全相同（比如都叫
        // "USB CAMERA"）。DShowControl 优先按设备路径精确匹配，但如果 Qt
        // （Media Foundation）给的设备id和 DirectShow 的 DevicePath 格式对
        // 不上，会退化到按名字匹配——这时必须告诉它"这是同名设备里的第几
        // 个"，否则永远只会连到枚举到的第一个同名相机，改哪个预览框的参数
        // 都改到同一台上。这个数第几的逻辑现在统一放在 WebcamCamera 里
        // （ParamPresets.cpp 应用模板时也调用同一个函数），避免两处各写一份、
        // 以后改枚举方式时漏改其中一处。
        ctrl_ = new DShowControl(web_->device().id(), web_->device().description(),
                                 web_->nameOccurrenceIndex());
        props_ = ctrl_->properties();
        tabs->addTab(buildPropTab(false),
                     QString::fromUtf8("\u89c6\u9891 Proc Amp"));            // 视频 Proc Amp
        tabs->addTab(buildPropTab(true),
                     QString::fromUtf8("\u7167\u76f8\u673a\u63a7\u5236"));   // 照相机控制
        tabs->addTab(buildFormatTab(),
                     QString::fromUtf8("\u683c\u5f0f\uff08\u5206\u8fa8\u7387/\u5e27\u7387\uff09"));
        // 格式（分辨率/帧率）
    } else {
        auto* l = new QLabel(QString::fromUtf8(
            "\u8be5\u76f8\u673a\u4e3a\u865a\u62df\u6d4b\u8bd5\u6e90\uff0c"
            "\u6ca1\u6709\u786c\u4ef6\u53c2\u6570\u3002\n"
            "\u63a5\u5165\u771f\u5b9e\u8bbe\u5907\u540e\u6b64\u5904\u663e\u793a "
            "AMCap \u540c\u6b3e\u53c2\u6570\u9762\u677f\u3002"));
        // 该相机为虚拟测试源，没有硬件参数。接入真实设备后此处显示 AMCap 同款参数面板。
        l->setAlignment(Qt::AlignCenter);
        tabs->addTab(l, QString::fromUtf8("\u53c2\u6570"));                  // 参数
    }

    auto* box = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    lay->addWidget(box);
    resize(560, 520);
}

CamParamDialog::~CamParamDialog() { delete ctrl_; }

QWidget* CamParamDialog::buildPropTab(bool cameraControl) {
    auto* page = new QWidget;
    auto* g = new QGridLayout(page);
    g->setColumnStretch(1, 1);
    g->addWidget(new QLabel(QString::fromUtf8("\u81ea\u52a8")), 0, 3);       // 自动

    int row = 1;
    bool any = false;
    for (int i = 0; i < props_.size(); ++i) {
        const DShowProp& p = props_[i];
        if (p.isCameraControl != cameraControl) continue;
        any = true;

        auto* name = new QLabel(p.name);
        auto* s = new QSlider(Qt::Horizontal);
        auto* b = new QSpinBox;
        auto* a = new QCheckBox;
        fixSpinBoxDisplay(b);

        if (p.supported) {
            s->setRange(int(p.min), int(p.max));
            s->setSingleStep(int(p.step > 0 ? p.step : 1));
            s->setPageStep(int((p.max - p.min) / 10 > 0 ? (p.max - p.min) / 10 : 1));
            s->setValue(int(p.value));
            b->setRange(int(p.min), int(p.max));
            b->setValue(int(p.value));
            a->setEnabled(p.autoSupported);
            a->setChecked(p.isAuto);
            const bool manual = !p.isAuto && p.manualSupported;
            s->setEnabled(manual);
            b->setEnabled(manual);
        } else {
            name->setEnabled(false);
            s->setEnabled(false);
            b->setEnabled(false);
            a->setEnabled(false);
        }

        g->addWidget(name, row, 0);
        g->addWidget(s,    row, 1);
        g->addWidget(b,    row, 2);
        g->addWidget(a,    row, 3);

        Row r; r.propIndex = i; r.slider = s; r.box = b; r.autoC = a;
        rows_.push_back(r);

        if (p.supported) {
            connect(s, &QSlider::valueChanged, this, [this, r](int v) {
                r.box->blockSignals(true); r.box->setValue(v); r.box->blockSignals(false);
                props_[r.propIndex].value = v;
                applyRow(r);
            });
            connect(b, &QSpinBox::valueChanged, this, [this, r](int v) {
                r.slider->blockSignals(true); r.slider->setValue(v); r.slider->blockSignals(false);
                props_[r.propIndex].value = v;
                applyRow(r);
            });
            connect(a, &QCheckBox::toggled, this, [this, r](bool on) {
                DShowProp& p2 = props_[r.propIndex];
                p2.isAuto = on;
                const bool manual = !on && p2.manualSupported;
                r.slider->setEnabled(manual);
                r.box->setEnabled(manual);
                applyRow(r);
            });
        }
        ++row;
    }

    auto* def = new QPushButton(QString::fromUtf8("\u9ed8\u8ba4\u503c"));    // 默认值
    connect(def, &QPushButton::clicked, this, [this, cameraControl] {
        for (const Row& r : rows_) {
            DShowProp& p = props_[r.propIndex];
            if (p.isCameraControl != cameraControl || !p.supported) continue;
            p.value = p.def;
            p.isAuto = p.autoSupported;   // 有自动能力的恢复为自动（AMCap 行为）
            setRowUi(r);
            applyRow(r);
        }
    });
    g->addWidget(def, row, 0, 1, 2, Qt::AlignLeft);
    ++row;

    if (!any) {
        const QString msg = (ctrl_ && ctrl_->valid())
        ? QString::fromUtf8("\u8be5\u8bbe\u5907\u4e0d\u652f\u6301\u6b64\u7c7b\u5c5e\u6027")
        // 该设备不支持此类属性
        : QString::fromUtf8("\u672a\u80fd\u8fde\u63a5\u8bbe\u5907\u5c5e\u6027\u63a5\u53e3"
                            "\uff08DirectShow\uff09");
        // 未能连接设备属性接口（DirectShow）
        g->addWidget(new QLabel(msg), row, 0, 1, 4);
    }
    g->setRowStretch(row + 1, 1);
    return page;
}

QWidget* CamParamDialog::buildFormatTab() {
    auto* page = new QWidget;
    auto* v = new QVBoxLayout(page);
    v->addWidget(new QLabel(QString::fromUtf8(
        "\u76f8\u5f53\u4e8e AMCap \u7684 Video Capture Pin\uff1a"
        "\u5206\u8fa8\u7387\u4e0e\u5e27\u7387\u5747\u76f4\u63a5\u8bfb\u53d6\u9a71\u52a8"
        "\u5b9e\u9645\u4e0a\u62a5\u7684\u80fd\u529b\uff08\u4e0d\u786c\u7f16\u7801\u4efb\u4f55"
        "\u9ed8\u8ba4\u503c\uff09\u3002")));
    // 相当于 AMCap 的 Video Capture Pin：分辨率与帧率均直接读取驱动实际上报的能力（不硬编码任何默认值）。

    allFormats_ = web_->formats();

    auto* form = new QGridLayout;
    form->addWidget(new QLabel(QString::fromUtf8("\u5206\u8fa8\u7387")), 0, 0); // 分辨率
    resCombo_ = new QComboBox;
    form->addWidget(resCombo_, 0, 1);
    form->addWidget(new QLabel(QString::fromUtf8("\u5e27\u7387")), 1, 0);      // 帧率
    fpsCombo_ = new QComboBox;
    form->addWidget(fpsCombo_, 1, 1);
    v->addLayout(form);

    // —— 去重收集分辨率列表，按面积从大到小排（不臆断哪个是“默认”）——
    QList<QSize> resList;
    for (const QCameraFormat& f : allFormats_) {
        if (!resList.contains(f.resolution()))
            resList.push_back(f.resolution());
    }
    std::sort(resList.begin(), resList.end(), [](const QSize& a, const QSize& b) {
        return a.width() * a.height() > b.width() * b.height();
    });

    const QCameraFormat cur = web_->currentFormat();
    int curResIdx = -1;
    for (int i = 0; i < resList.size(); ++i) {
        const QSize& r = resList[i];
        resCombo_->addItem(QString("%1 \u00d7 %2").arg(r.width()).arg(r.height()), r);
        if (cur.resolution() == r) curResIdx = i;
    }
    if (curResIdx < 0 && !resList.isEmpty()) curResIdx = 0;
    if (curResIdx >= 0) resCombo_->setCurrentIndex(curResIdx);

    connect(resCombo_, &QComboBox::currentIndexChanged, this, [this](int) {
        populateFpsForCurrentRes();
    });
    populateFpsForCurrentRes();   // 首次填充帧率列表（含当前分辨率下的最高帧率，比如你相机的 120）

    auto* apply = new QPushButton(QString::fromUtf8("\u5e94\u7528\u683c\u5f0f"));  // 应用格式
    connect(apply, &QPushButton::clicked, this, [this] { applyCurrentFormatSelection(); });
    v->addWidget(apply, 0, Qt::AlignLeft);

    // 实际生效格式回读——见头文件 formatReadbackLabel_ 注释：下拉框显示的
    // 是"你选了什么"，这个标签显示的是"驱动实际给了什么"，两者不一定
    // 相同(尤其未压缩格式在带宽不够时，标称帧率可能是驱动乐观上报的)。
    formatReadbackLabel_ = new QLabel(QString::fromUtf8("\u5b9e\u9645\u751f\u6548\u683c\u5f0f\uff1a--"));
    formatReadbackLabel_->setWordWrap(true);
    v->addWidget(formatReadbackLabel_);

    // 帧率可兑现性提示（平时是空的，不占视觉）
    rateHintLabel_ = new QLabel;
    rateHintLabel_->setWordWrap(true);
    rateHintLabel_->setTextFormat(Qt::PlainText);   // 内容里有 %、「」，别让它去猜富文本
    v->addWidget(rateHintLabel_);

    // ---- 调试区：逐条查看/强制指定驱动原始上报的格式 ----
    v->addWidget(new QLabel(QString::fromUtf8(
        "\u2014\u2014 \u8c03\u8bd5\uff1a\u4e0b\u9762\u628a\u5f53\u524d\u5206\u8fa8\u7387\u4e0b\u9a71\u52a8\u4e0a\u62a5\u7684"
        "\u6bcf\u4e00\u6761\u539f\u59cb\u683c\u5f0f\u90fd\u5217\u51fa\u6765(\u4e0d\u53bb\u91cd\u3001\u4e0d\u731c\u6d4b)\uff0c"
        "\u9009\u54ea\u6761\u5c31\u5f3a\u5236\u5e94\u7528\u54ea\u6761\uff0c\u65b9\u4fbf\u5bf9\u6bd4\u538b\u7f29/\u672a\u538b\u7f29"
        "\u5b9e\u9645\u4ea4\u4ed8\u5e27\u7387\u7684\u5dee\u5f02\u3002")));

    auto* exactForm = new QGridLayout;
    exactForm->addWidget(new QLabel(QString::fromUtf8("\u5177\u4f53\u683c\u5f0f")), 0, 0);
    exactFormatCombo_ = new QComboBox;
    exactForm->addWidget(exactFormatCombo_, 0, 1);
    v->addLayout(exactForm);

    auto* exactApply = new QPushButton(QString::fromUtf8(
        "\u5f3a\u5236\u5e94\u7528\u6240\u9009\u5177\u4f53\u683c\u5f0f"));
    connect(exactApply, &QPushButton::clicked, this, [this] { applyExactFormatSelection(); });
    v->addWidget(exactApply, 0, Qt::AlignLeft);

    auto* dumpBtn = new QPushButton(QString::fromUtf8(
        "\u6253\u5370\u5168\u90e8\u683c\u5f0f\u5230\u65e5\u5fd7(\u6240\u6709\u5206\u8fa8\u7387)"));
    connect(dumpBtn, &QPushButton::clicked, this, [this] { dumpAllFormatsToLog(); });
    v->addWidget(dumpBtn, 0, Qt::AlignLeft);

    connect(resCombo_, &QComboBox::currentIndexChanged, this, [this](int) {
        populateExactFormatsForCurrentRes();
    });
    populateExactFormatsForCurrentRes();

    v->addStretch(1);
    return page;
}

void CamParamDialog::populateFpsForCurrentRes() {
    if (!fpsCombo_ || !resCombo_) return;
    const QSize res = resCombo_->currentData().toSize();

    // 同一分辨率下，驱动可能给出多档离散帧率（min==max 各一条），也可能给
    // 一条连续区间；两种都覆盖：离散档直接列出，连续区间额外列出上限值
    // （= 你相机报的 120），并把整数帧率去重、从高到低排，方便一眼选到上限。
    QVector<double> fpsList;
    for (const QCameraFormat& f : allFormats_) {
        if (f.resolution() != res) continue;
        const double lo = f.minFrameRate(), hi = f.maxFrameRate();
        if (hi > 0) fpsList.push_back(hi);
        if (lo > 0 && !qFuzzyCompare(lo + 1.0, hi + 1.0)) fpsList.push_back(lo);
    }
    std::sort(fpsList.begin(), fpsList.end(), std::greater<double>());
    fpsList.erase(std::unique(fpsList.begin(), fpsList.end(),
                              [](double a, double b) { return std::abs(a - b) < 0.5; }),
                  fpsList.end());

    fpsCombo_->blockSignals(true);
    fpsCombo_->clear();
    const QCameraFormat cur = web_->currentFormat();
    int curIdx = -1;
    for (int i = 0; i < fpsList.size(); ++i) {
        fpsCombo_->addItem(QString("%1 fps").arg(fpsList[i], 0, 'f', 0), fpsList[i]);
        if (cur.resolution() == res && qFuzzyCompare(cur.maxFrameRate() + 1.0f, float(fpsList[i]) + 1.0f))
            curIdx = i;
    }
    if (curIdx < 0 && !fpsList.isEmpty()) curIdx = 0;   // 默认选该分辨率下的最高帧率
    if (curIdx >= 0) fpsCombo_->setCurrentIndex(curIdx);
    fpsCombo_->blockSignals(false);
}

void CamParamDialog::applyCurrentFormatSelection() {
    if (!resCombo_ || !fpsCombo_) return;
    const QSize res = resCombo_->currentData().toSize();
    const double wantFps = fpsCombo_->currentData().toDouble();

    // 【改用共享函数】以前这里是自己写的一个裸循环，只看帧率数字最接近，
    // 完全没有"帧率打平时优先选压缩格式"这层判断——跟 pickBestFormat()
    // (启动时自动选格式用的那份逻辑)不一致。如果驱动在同一分辨率下给出
    // 两条"标称都是120fps"的格式，一条压缩(真能到120)一条未压缩(标称
    // 乐观，实际USB带宽扛不住只能交付60多)，这里以前可能选中未压缩那条，
    // 表现就是"下拉框选了120，实际却只有60多"。见 WebcamCamera.hpp::
    // pickFormatMatching 注释。
    const QCameraFormat best = pickFormatMatching(allFormats_, res, wantFps);
    if (best.isNull()) return;

    // 【下拉框里的帧率不一定是"一档格式"】populateFpsForCurrentRes() 会把每条
    // 格式的 maxFrameRate 和 minFrameRate 都列出来：一条 [15,30] 的连续区间
    // 会同时贡献 "30 fps" 和 "15 fps"；有些驱动则会给出 [15,15]/[30,30] 两条
    // 离散格式。无论哪种，用户点"应用格式"时都已经明确指定了 wantFps，
    // 这里必须【始终把 wantFps 传下去】，不能再按"它是不是标称上限"决定是否
    // 忽略。否则离散的 [15,15] 会被当成 nominal、targetFps 传 0，随后 NV12
    // 这类未压缩格式按老规则走 Qt/Media Foundation —— Qt 仍然可能跑 30，
    // 用户看到的就是"选 15 无效"。
    //
    // targetFps 一旦 > 0，WebcamCamera::startWithFormat 就会走 DirectShow
    // （显式帧率需要 AvgTimePerFrame 才能真正兑现），并在驱动量化时由采集层
    // 抽帧兜底。0 只保留给"自动启动/具体格式调试"这两条非用户选帧率的路径。
    const double targetFps = wantFps;
    web_->setFormat(best, targetFps);
    AppSettings().saveCameraFormat(web_->deviceKey(), best.resolution(),
                                   int(best.pixelFormat()), best.maxFrameRate(), targetFps);
    noteRateFeasibility(best, wantFps);
    scheduleFormatReadback();
}

// 选中的帧率到底能不能兑现，当场说清楚。
//
// 【为什么必须说】区间内指定帧率只有 DirectShow 引擎做得到（IAMStreamConfig
// 设 AvgTimePerFrame）。而 startWithFormat() 只在【压缩格式】上才尝试
// DirectShow，未压缩格式一律走 Qt/Media Foundation —— 那条路上 QCameraFormat
// 表达不了"区间内取某个值"，请求会被无声丢弃、按标称上限跑。
// 静默跑成另一个帧率是这次这个 bug 最难查的部分，所以宁可话多也要写出来。
void CamParamDialog::noteRateFeasibility(const QCameraFormat& f, double wantFps) {
    if (!rateHintLabel_) return;
    if (formatHitsExactly(f, wantFps)) {   // 标称档，两条引擎都能跑到
        rateHintLabel_->clear();
        return;
    }
    // 走到这里说明目标帧率落在格式区间内部，不是标称档。这种请求只有
    // DirectShow 引擎能兑现（IAMStreamConfig 设 AvgTimePerFrame），
    // 而 WebcamCamera::startWithFormat 现在【只要指定了帧率就会去试
    // DirectShow】，不再限制像素格式 —— 所以压缩与否都有机会成功。
    // 真正的不确定性在驱动那一侧：UVC 驱动通常只支持一组离散帧间隔，
    // 会把请求量化到最近的一档，也可能干脆忽略。日志里有回读值。
    rateHintLabel_->setText(QStringLiteral(
        "%1 fps 不是该格式的标称档（标称 %2 fps），已作为帧间隔下发给引擎。"
        "以预览左上角的实测帧率为准。")
        .arg(wantFps, 0, 'f', 0).arg(f.maxFrameRate(), 0, 'f', 0));
}

void CamParamDialog::populateExactFormatsForCurrentRes() {
    if (!exactFormatCombo_ || !resCombo_) return;
    const QSize res = resCombo_->currentData().toSize();

    // 按压缩优先、帧率从高到低排——方便一眼看到"这个分辨率下最值得试的
    // 几条"，但不去重、不合并，驱动上报几条就列几条，包括帧率数字完全
    // 相同但像素格式不同的那些"看起来重复"的条目——这些恰恰是这个调试
    // 工具存在的意义。
    QList<int> idx;
    for (int i = 0; i < allFormats_.size(); ++i)
        if (allFormats_[i].resolution() == res) idx.push_back(i);
    std::sort(idx.begin(), idx.end(), [this](int a, int b) {
        const QCameraFormat& fa = allFormats_[a]; const QCameraFormat& fb = allFormats_[b];
        const bool ca = isCompressedFormat(fa.pixelFormat()), cb = isCompressedFormat(fb.pixelFormat());
        if (ca != cb) return ca; // 压缩的排前面
        return fa.maxFrameRate() > fb.maxFrameRate();
    });

    exactFormatCombo_->blockSignals(true);
    exactFormatCombo_->clear();
    for (int i : idx) {
        const QCameraFormat& f = allFormats_[i];
        const bool comp = isCompressedFormat(f.pixelFormat());
        const QString label = QString("%1~%2 fps \u00b7 %3 (%4)")
            .arg(f.minFrameRate(), 0, 'f', 0).arg(f.maxFrameRate(), 0, 'f', 0)
            .arg(QVideoFrameFormat::pixelFormatToString(f.pixelFormat()))
            .arg(comp ? QString::fromUtf8("\u538b\u7f29") : QString::fromUtf8("\u672a\u538b\u7f29"));
        exactFormatCombo_->addItem(label, i);
    }
    exactFormatCombo_->blockSignals(false);
}

void CamParamDialog::applyExactFormatSelection() {
    if (!exactFormatCombo_) return;
    const int idx = exactFormatCombo_->currentData().toInt();
    if (idx < 0 || idx >= allFormats_.size()) return;
    const QCameraFormat& f = allFormats_[idx];
    // 【这里是"强制应用所选具体格式"，按格式自己的标称档跑】用户点的是列表里
    // 一条具体格式，不是一个帧率数字，所以不存在"区间内指定"，targetFps 传 0。
    web_->setFormat(f, 0.0);
    AppSettings().saveCameraFormat(web_->deviceKey(), f.resolution(),
                                   int(f.pixelFormat()), f.maxFrameRate(), 0.0);
    if (rateHintLabel_) rateHintLabel_->clear();
    scheduleFormatReadback();
}

void CamParamDialog::dumpAllFormatsToLog() {
    qWarning().noquote() << QStringLiteral("[CamParamDialog] ---- 驱动上报的全部格式(共%1条) ----").arg(allFormats_.size());
    for (const QCameraFormat& f : allFormats_) {
        qWarning().noquote() << QStringLiteral("  %1x%2  %3~%4fps  %5  compressed=%6")
            .arg(f.resolution().width()).arg(f.resolution().height())
            .arg(f.minFrameRate(), 0, 'f', 0).arg(f.maxFrameRate(), 0, 'f', 0)
            .arg(QVideoFrameFormat::pixelFormatToString(f.pixelFormat()))
            .arg(isCompressedFormat(f.pixelFormat()) ? "yes" : "no");
    }
}

void CamParamDialog::scheduleFormatReadback() {
    if (formatReadbackLabel_) formatReadbackLabel_->setText(
        QString::fromUtf8("\u5b9e\u9645\u751f\u6548\u683c\u5f0f\uff1a\u5e94\u7528\u4e2d\u2026"));

    // setFormat() 内部用 Qt::QueuedConnection 把 applyFormat() 投到相机
    // 线程，不是同步生效——这里立刻回读 currentFormat() 大概率还是旧值。
    // 【整合DirectShow采集引擎之后】压缩格式每次应用都要重新走一遍完整的
    // DirectShow建图流程(枚举设备/绑定moniker/协商格式/连线/Run())，这比
    // 原来"切一下QCamera的格式"重得多，尤其第一次跑的时候——单次300ms
    // 有时候不够，读到的时候旧引擎已经拆掉、新引擎还没来得及标记自己
    // 就绪，会被误判成"读取失败"，但实际上引擎马上就成功了(预览画面/
    // 帧率读数很快就会恢复正常，只是这个诊断标签抢跑了一步)。
    // 改成失败后按固定间隔重试几次，给建图流程留够时间，比一次性延迟
    // 300ms更宽容，也不需要精确判断"到底哪一刻算真正切换完成"。
    tryReadbackFormat(0);
}

void CamParamDialog::tryReadbackFormat(int attempt) {
    static constexpr int kMaxAttempts = 6;      // 最多试6次
    static constexpr int kIntervalMs = 300;     // 每次间隔300ms，总共最长约1.8s

    QPointer<CamParamDialog> guard(this);
    QTimer::singleShot(kIntervalMs, this, [guard, attempt]() {
        if (!guard || !guard->web_ || !guard->formatReadbackLabel_) return;
        const QCameraFormat applied = guard->web_->currentFormat();
        if (applied.isNull()) {
            if (attempt + 1 < kMaxAttempts) {
                guard->tryReadbackFormat(attempt + 1);   // 还没就绪，再等一轮
                return;
            }
            // 真的等了将近2秒还是空——这时候才值得怀疑是不是真出问题了，
            // 不再是"建图慢"能解释的范围。
            guard->formatReadbackLabel_->setText(
                QString::fromUtf8("\u5b9e\u9645\u751f\u6548\u683c\u5f0f\uff1a\u8bfb\u53d6\u5931\u8d25(\u7b49\u5f85\u8f83\u4e45\u4ecd\u65e0\u7ed3\u679c\uff0c\u76f8\u673a\u53ef\u80fd\u771f\u7684\u6ca1\u542f\u52a8\u6210\u529f)"));
            return;
        }
        const bool comp = isCompressedFormat(applied.pixelFormat());
        guard->formatReadbackLabel_->setText(QString::fromUtf8(
            "\u5b9e\u9645\u751f\u6548\u683c\u5f0f\uff1a%1\u00d7%2 @ %3fps (%4)")
            .arg(applied.resolution().width()).arg(applied.resolution().height())
            .arg(applied.maxFrameRate(), 0, 'f', 0)
            .arg(comp ? QString::fromUtf8("\u538b\u7f29")
                      : QString::fromUtf8("\u672a\u538b\u7f29\uff0c\u5e26\u5bbd\u4e0d\u591f\u65f6\u5b9e\u9645\u5e27\u7387\u53ef\u80fd\u4f4e\u4e8e\u6807\u79f0\u503c")));
    });
}

void CamParamDialog::applyRow(const Row& r) {
    if (!ctrl_) return;
    const DShowProp& p = props_[r.propIndex];
    ctrl_->set(p.id, p.isCameraControl, p.value, p.isAuto);

    // 【持久化】只存"曝光"这一项——按名字匹配，不依赖具体的id数值(那是
    // DirectShow内部枚举值，跟"这一行显示的是不是曝光"没有必然对应关系，
    // 按UI上显示的中文名匹配更直接、也更不容易在这里犯"猜数字"的错误)。
    // 存了之后，CameraManager::addWebcam() 开机会自动读回来重新套用，
    // 不用每次开程序都手动调一遍。
    if (web_ && p.name == QString::fromUtf8("\u66dd\u5149")) {
        AppSettings().saveCameraExposure(web_->deviceKey(), p.isAuto, p.value);
    }
}

void CamParamDialog::setRowUi(const Row& r) {
    const DShowProp& p = props_[r.propIndex];
    r.slider->blockSignals(true);
    r.box->blockSignals(true);
    r.autoC->blockSignals(true);
    r.slider->setValue(int(p.value));
    r.box->setValue(int(p.value));
    r.autoC->setChecked(p.isAuto);
    const bool manual = !p.isAuto && p.manualSupported;
    r.slider->setEnabled(manual);
    r.box->setEnabled(manual);
    r.slider->blockSignals(false);
    r.box->blockSignals(false);
    r.autoC->blockSignals(false);
}

} // namespace mocap