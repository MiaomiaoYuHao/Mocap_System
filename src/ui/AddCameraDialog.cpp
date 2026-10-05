#include "ui/AddCameraDialog.hpp"
#include "camera/DShowControl.hpp"     // enumerateVideoDevices()：只读属性包的轻量枚举
#include "camera/CameraLabel.hpp"      // makeCameraLabel()：把七行 "USB CAMERA" 变成可区分的
#include "settings/AppSettings.hpp"     // cameraNumber()：跟相机对象共用的那张编号表
#include <QMediaDevices>
#include <QHash>
#include <QListWidget>
#include <QListWidgetItem>
#include <QAbstractItemView>   // MultiSelection（点一下选中、再点一下取消）
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QLabel>
#include <QTimer>

namespace mocap {

AddCameraDialog::AddCameraDialog(const QSet<QString>& usedKeys, QWidget* parent)
    : QDialog(parent), used_(usedKeys) {
    setWindowTitle(QString::fromUtf8("添加相机"));
    resize(460, 360);

    auto* lay = new QVBoxLayout(this);
    lay->addWidget(new QLabel(QString::fromUtf8(
        "选择要接入的视频设备（列表每秒自动刷新，插拔即时可见）：\n"
        "可批量选中 —— 点一下选中，再点一下取消；选好后一次确定全部加入。")));

    list_ = new QListWidget;
    // MultiSelection = 点一下选中、再点一下取消，不需要按 Ctrl/Shift。
    // 【为什么不用 ExtendedSelection】那个要靠 Ctrl 点才能多选、Shift 点才能选
    // 连续区间，对现场接线的人来说是多余的负担 —— 反光球工位上没人愿意记修饰键。
    // 已添加的设备在 refresh() 里被摘掉 ItemIsEnabled，点不中也带不进批量结果。
    list_->setSelectionMode(QAbstractItemView::MultiSelection);
    lay->addWidget(list_, 1);

    auto* btnRow = new QHBoxLayout;
    auto* refreshBtn = new QPushButton(QString::fromUtf8("刷新"));
    btnRow->addWidget(refreshBtn);
    btnRow->addStretch(1);
    auto* box = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    btnRow->addWidget(box);
    lay->addLayout(btnRow);

    connect(refreshBtn, &QPushButton::clicked, this, &AddCameraDialog::refresh);
    connect(box, &QDialogButtonBox::accepted, this, &AddCameraDialog::onAccept);
    connect(box, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(list_, &QListWidget::itemDoubleClicked, this,
            [this](QListWidgetItem*) { onAccept(); });

    // 【轮询而不是监听设备到达】枚举只读属性包、不打开设备，一次几百微秒，
    // 每秒一次的开销可以忽略。用轮询是有意的：不需要注册任何设备通知，
    // 也就不会掺进系统的 PnP 通知链里。
    poll_ = new QTimer(this);
    poll_->setInterval(1000);
    connect(poll_, &QTimer::timeout, this, &AddCameraDialog::refresh);
    poll_->start();

    refresh();
}

// 这个设备是不是已经被添加过了。
//
// 【为什么不能直接比字符串】used_ 里的 key 是 "web:" + QCameraDevice::id()
// （Qt 的 id），而我们枚举出来的是 DirectShow 的 DevicePath。两者都是内核
// 符号链接，但【不保证逐字节相同】—— Qt 走 Media Foundation 时编码方式
// 可能不同，大小写和前后缀也不一定一致。onAccept() 里正是因为这个才写了
// "互相包含 + 退回友好名" 的匹配。
//
// 直接比的后果：已添加的相机不会被置灰，用户会重复添加同一台。
bool AddCameraDialog::isUsed(const DevEntry& d) const {
    const QString path = QString::fromLatin1(d.id).toLower();
    for (const QString& k : used_) {
        if (!k.startsWith(QStringLiteral("web:"))) continue;
        const QString kid = k.mid(4).toLower();
        if (kid.isEmpty() || path.isEmpty()) continue;
        if (kid == path || kid.contains(path) || path.contains(kid)) return true;
    }
    return false;
}

void AddCameraDialog::refresh() {
    // 【看不见就别扫】对话框被遮住/最小化/正在关闭时继续每秒枚举一次
    // 纯属白烧，而且枚举要碰 DirectShow 子系统，跟正在跑的相机是同一套东西。
    // 手动点"刷新"时 isVisible() 必然为真，不受影响。
    if (!isVisible() && list_->count() > 0) return;

    // 只读属性包的枚举：不打开任何设备。见头文件说明。
    const QVector<DShowDevice> found = enumerateVideoDevices();

    // 先算指纹。列表没变就不要重建 —— 每秒重建会把用户正在做的选择清掉。
    QString sig;
    for (const DShowDevice& d : found) {
        sig += QString::fromLatin1(d.devicePath);
        sig += QLatin1Char('|');
        sig += d.friendlyName;
        sig += QLatin1Char('\n');
    }
    if (sig == sig_ && list_->count() > 0) return;
    sig_ = sig;

    // 【按设备 id 记住选中项，不能用行号】刷新的原因恰恰是列表变了 ——
    // 插一个相机导致重排时，同一个行号会指向另一台设备。用户选好了、
    // 一秒后列表刷新、选中项静默换成别的，然后点确定加错相机。
    // "虚拟测试相机"那一项没有 id，单独用一个标记记住。
    // 【多选也要保住】原来只记 currentItem()，改成多选后如果还只记一项，
    // 每秒刷新会把用户已经点好的那几台打回只剩一台。
    QList<QByteArray> prevSelIds;
    bool prevSelVirtual = false;
    for (QListWidgetItem* it : list_->selectedItems()) {
        const int i = it->data(Qt::UserRole).toInt();
        if (i < 0) prevSelVirtual = true;
        else if (i < devs_.size()) prevSelIds << devs_[i].id;
    }
    list_->clear();
    devs_.clear();
    devs_.reserve(found.size());
    {
        // 同名设备编号：三台一样的相机 description 相同，靠这个区分第几台。
        QHash<QString, int> seen;
        for (const DShowDevice& d : found) {
            const int occ = seen.value(d.friendlyName, 0);
            seen[d.friendlyName] = occ + 1;
            devs_.push_back({d.devicePath, d.friendlyName, occ});
        }
    }

    // ---- 显示名：跟相机对象的 name() 用【同一条规则、同一张编号表】 ----
    // 【原来为什么对不上】这里传的是 d.occ（"同名设备里的第几个"，DShow 枚举序），
    // 而 WebcamCamera 构造时压根没传编号，于是同一台相机在这里叫
    // "USB CAMERA #3 [Port_#0001.Hub_#0005]"、加完退出去（侧边栏、宫格、
    // 标定向导、同步监视器…… 全部取 ICamera::name()）却叫
    // "USB CAMERA [Port_#0001.Hub_#0005]"，用户对不上号。
    //
    // 【为什么不能靠现算的序号对齐】d.occ 数的是设备枚举序，插拔一次就重排；
    // 而且这个对话框走 DirectShow 枚举、相机对象走 Qt 枚举，两份顺序不保证
    // 一致。两边各算各的，早晚还是分叉。
    //
    // 现在两边都问 AppSettings::cameraNumber(devicePath) 要号，它拿
    // devicePathToInstanceId() 归一化后的【整条实例 ID】查一张持久化的表 ——
    // DShow 的 DevicePath 和 Qt 的 QCameraDevice::id() 会归一到同一个串，
    // 所以必然是同一个号，而且重启不变、拖拽换位不变。
    //
    // 【QSettings 只建一次】cameraNumber() 会读注册表，devs_ 有七台，
    // 在循环里 new 七个 QSettings 是白烧 —— 这个函数还是 1Hz 轮询调用的。
    AppSettings settings;
    for (int i = 0; i < devs_.size(); ++i) {
        const DevEntry& d = devs_[i];
        // 【显示归显示，持久化仍然只认 DevicePath】下面 onAccept() 的三梯匹配
        // 和 used_ 的判重用的都是 d.id，一个字都没改。
        const QString text = makeCameraLabel(d.desc, d.id, settings.cameraNumber(d.id)).display;
        auto* item = new QListWidgetItem(text, list_);
        item->setData(Qt::UserRole, i);
        if (isUsed(d)) {
            item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
            item->setText(text + QString::fromUtf8("（已添加）"));
        }
    }

    auto* virt = new QListWidgetItem(QString::fromUtf8("虚拟测试相机"), list_);
    virt->setData(Qt::UserRole, -1);

    // 按 id 找回原来选中的那一项；找不到（设备被拔了）就不选，
    // 让用户自己重新挑 —— 总比默默选中一台别的强。
    if (prevSelVirtual) {
        list_->setCurrentRow(list_->count() - 1);      // 虚拟相机固定在最后
    }
    if (!prevSelIds.isEmpty()) {
        QListWidgetItem* firstRestored = nullptr;
        for (int i = 0; i < list_->count(); ++i) {
            QListWidgetItem* it = list_->item(i);
            const int di = it->data(Qt::UserRole).toInt();
            if (di < 0 || di >= devs_.size()) continue;
            if (!prevSelIds.contains(devs_[di].id)) continue;
            if (!(it->flags() & Qt::ItemIsEnabled)) continue;   // 刚被加走了就不再选
            it->setSelected(true);
            if (!firstRestored) firstRestored = it;
        }
        // 让 currentItem 也落回选中集合里的第一项：键盘导航/QDialog 的默认按钮
        // 判定都看 currentItem，只 setSelected 不设 current 会让它们失同步。
        if (firstRestored) list_->setCurrentItem(firstRestored);
    }
}

// 把一个下拉项解析成 WebcamCamera 能用的 QCameraDevice。
// 【三梯匹配】内联在 onAccept() 里原来只有一份，批量添加要对每台都跑一遍，
// 抽出来共用，避免三份复制走样。
bool AddCameraDialog::resolveDevice(const DevEntry& de, QCameraDevice* out) const {
    // 【只有这一刻才碰 QMediaDevices】下游的 WebcamCamera 需要
    // QCameraDevice（选格式、以及 Qt 兜底采集那条路），绕不开。
    // 但这是用户点确定的一刻，不会跟设备到达并发。
    const QString wantPath = QString::fromLatin1(de.id).toLower();
    const QString wantName = de.desc;
    const int wantOcc = de.occ;
    const QList<QCameraDevice> live = QMediaDevices::videoInputs();

    // 【第一梯：路径严格相等】最可靠，但两套枚举的编码不一定一致。
    for (const QCameraDevice& d : live) {
        if (!wantPath.isEmpty() && QString::fromLatin1(d.id()).toLower() == wantPath) {
            *out = d; return true;
        }
    }
    // 【第二梯：路径互相包含】前后缀差异（\\?\ 前缀、#{guid} 后缀）时还能对上。
    // 但要求【唯一命中】—— 多台同型号相机的路径共享很长的前缀，
    // 命中多个就说明这个判据分辨不了，必须让位给下一梯。
    if (!wantPath.isEmpty()) {
        int hit = -1, n = 0;
        for (int i = 0; i < live.size(); ++i) {
            const QString p = QString::fromLatin1(live[i].id()).toLower();
            if (p.isEmpty()) continue;
            if (p.contains(wantPath) || wantPath.contains(p)) { hit = i; ++n; }
        }
        if (n == 1) { *out = live[hit]; return true; }
    }
    // 【第三梯：同名取第 N 个】三台一样的相机走到这里。
    // 【不能只按名字取第一个】那样第二、三台都会拿到第一台，
    // 而它已经被占用 —— 表现就是"选了相机显示无信号，只有第一个能用"。
    // 两套枚举都源自同一套设备枚举，同名设备的相对顺序一致。
    {
        int occ = 0;
        for (const QCameraDevice& d : live) {
            if (d.description() != wantName) continue;
            if (occ == wantOcc) { *out = d; return true; }
            ++occ;
        }
    }
    // 【兜底：序号超出范围】中途拔插会让数量对不上。退回第一个同名的，
    // 好过完全加不上 —— 但这种情况下可能连到错的物理相机。
    // 跟 DShowControl::findMoniker 的兜底策略保持一致。
    for (const QCameraDevice& d : live)
        if (d.description() == wantName) { *out = d; return true; }
    return false;
}

void AddCameraDialog::onAccept() {
    // 【批量】把当前选中的全部条目都收进来。选中项里可能混着"虚拟测试相机"
    // （列表最后一项），它没有 QCameraDevice，单独用 isVirtual 标记。
    const QList<QListWidgetItem*> picked = list_->selectedItems();
    if (picked.isEmpty()) return;

    // 选定之后就别再刷了，免得回调里列表又被换掉。
    if (poll_) poll_->stop();

    QList<Selection> out;
    bool aborted = false;
    for (QListWidgetItem* item : picked) {
        if (!(item->flags() & Qt::ItemIsEnabled)) continue;
        const int idx = item->data(Qt::UserRole).toInt();

        if (idx < 0) {                       // 虚拟测试相机
            Selection sel;
            sel.isVirtual = true;
            out << sel;
            continue;
        }
        if (idx >= devs_.size()) continue;

        QCameraDevice dev;
        if (!resolveDevice(devs_[idx], &dev)) {
            // 列出到点确定之间设备被拔了，或者两套枚举对不上。
            // 【整批中止，不做"能加的先加上"】用户挑的是一个要一起用的组合
            // （比如 6 台里少 2 台，剩下 4 台的共视/标定关系整个对不上），
            // 少加几台比让他清掉重来一遍更糟。
            aborted = true;
            break;
        }
        Selection sel;
        sel.device = dev;
        out << sel;
    }

    if (aborted || out.isEmpty()) {
        // 刷新让用户看到当前实际状态，而不是拿空设备去 addWebcam。
        if (poll_) poll_->start();
        sig_.clear();
        refresh();
        return;
    }

    selection_ = out;
    accept();
}

} // namespace mocap
