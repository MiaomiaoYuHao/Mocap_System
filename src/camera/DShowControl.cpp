#include "camera/DShowControl.hpp"

#ifdef _WIN32
// -------------------------- Windows 实现 --------------------------
#include <windows.h>
#include <dshow.h>

namespace mocap {

namespace {
// 属性清单（顺序即 UI 显示顺序，与 AMCap 两个页签一致）。
struct Item { long id; const char* name; bool camCtrl; };
const Item kItems[] = {
    { VideoProcAmp_Brightness,            "\u4eae\u5ea6",       false }, // 亮度
    { VideoProcAmp_Contrast,              "\u5bf9\u6bd4\u5ea6", false }, // 对比度
    { VideoProcAmp_Hue,                   "\u8272\u8c03",       false }, // 色调
    { VideoProcAmp_Saturation,            "\u9971\u548c\u5ea6", false }, // 饱和度
    { VideoProcAmp_Sharpness,             "\u6e05\u6670\u5ea6", false }, // 清晰度
    { VideoProcAmp_Gamma,                 "\u4f3d\u739b",       false }, // 伽玛
    { VideoProcAmp_ColorEnable,           "\u542f\u7528\u989c\u8272", false }, // 启用颜色
    { VideoProcAmp_WhiteBalance,          "\u767d\u5e73\u8861", false }, // 白平衡
    { VideoProcAmp_BacklightCompensation, "\u9006\u5149\u5bf9\u6bd4", false }, // 逆光对比
    { VideoProcAmp_Gain,                  "\u589e\u76ca",       false }, // 增益
    { CameraControl_Pan,                  "\u5168\u666f",       true  }, // 全景
    { CameraControl_Tilt,                 "\u503e\u659c",       true  }, // 倾斜
    { CameraControl_Roll,                 "\u6eda\u52a8",       true  }, // 滚动
    { CameraControl_Zoom,                 "\u7f29\u653e",       true  }, // 缩放
    { CameraControl_Exposure,             "\u66dd\u5149",       true  }, // 曝光
    { CameraControl_Iris,                 "\u5149\u5708",       true  }, // 光圈
    { CameraControl_Focus,                "\u7126\u70b9",       true  }, // 焦点
};
} // namespace

// ---------------------------------------------------------------------------
// 枚举全部视频输入设备。【只读属性包，不打开任何设备】
//
// 跟 Qt 的 QMediaDevices::videoInputs() 的关键区别：Qt 会把每个 moniker 绑成
// IBaseFilter 再查 IAMStreamConfig 拿支持的格式列表 —— 等于逐个打开摄像头。
// 有相机正在独占推流时那会阻塞，而且很慢。
//
// 这里只做 BindToStorage -> IPropertyBag，读 DevicePath 和 FriendlyName 两个
// 字符串。不碰设备本身，所以可以安全地定时反复调用（比如每秒刷一次列表）。
// ---------------------------------------------------------------------------
QVector<DShowDevice> enumerateVideoDevices() {
    QVector<DShowDevice> out;

    // 【套间模型必须跟本文件其余部分一致：STA】
    // DShowControl 的构造函数用的是 COINIT_APARTMENTTHREADED。同一个线程上
    // 两种模型冲突：先被谁初始化，另一个就拿到 RPC_E_CHANGED_MODE。
    //
    // 【绝对不要 CoUninitialize】这个函数是被 GUI 线程【每秒轮询】调用的。
    // 如果这次调用恰好是该线程上第一次成功初始化 COM，配对的 CoUninitialize
    // 就会把引用计数归零、【拆掉整个套间】—— 而此时正在预览的相机的
    // DirectShow 对象还活着，于是画面卡死。真机症状：添加一个相机之后，
    // 上一个相机的画面就停住。
    //
    // 正确做法是只保证"已经初始化过"，然后什么都不做 —— 线程退出时由
    // Qt/系统统一收尾。多调一次 CoInitializeEx 只是加引用计数，无害。
    CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);

    ICreateDevEnum* devEnum = nullptr;
    if (SUCCEEDED(CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                   IID_ICreateDevEnum, reinterpret_cast<void**>(&devEnum)))) {
        IEnumMoniker* en = nullptr;
        if (devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &en, 0) == S_OK) {
            IMoniker* mon = nullptr;
            ULONG fetched = 0;
            while (en->Next(1, &mon, &fetched) == S_OK) {
                IPropertyBag* bag = nullptr;
                DShowDevice d;
                if (SUCCEEDED(mon->BindToStorage(nullptr, nullptr, IID_IPropertyBag,
                                                 reinterpret_cast<void**>(&bag)))) {
                    VARIANT v; VariantInit(&v);
                    if (SUCCEEDED(bag->Read(L"DevicePath", &v, nullptr)) && v.vt == VT_BSTR)
                        d.devicePath = QString::fromWCharArray(v.bstrVal).toLatin1();
                    VariantClear(&v);
                    if (SUCCEEDED(bag->Read(L"FriendlyName", &v, nullptr)) && v.vt == VT_BSTR)
                        d.friendlyName = QString::fromWCharArray(v.bstrVal);
                    VariantClear(&v);
                    bag->Release();
                }
                mon->Release();
                if (!d.friendlyName.isEmpty() || !d.devicePath.isEmpty())
                    out.push_back(d);
            }
            en->Release();
        }
        devEnum->Release();
    }

    // 【故意不 CoUninitialize】见上面的说明。
    return out;
}

bool DShowControl::findMoniker(const QByteArray& qtDeviceId, const QString& friendlyName,
                               int nameOccurrenceIndex, void** outMonikerIUnknown) {
    if (outMonikerIUnknown) *outMonikerIUnknown = nullptr;

    ICreateDevEnum* devEnum = nullptr;
    if (FAILED(CoCreateInstance(CLSID_SystemDeviceEnum, nullptr, CLSCTX_INPROC_SERVER,
                                IID_ICreateDevEnum, reinterpret_cast<void**>(&devEnum))))
        return false;

    IEnumMoniker* en = nullptr;
    if (devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &en, 0) != S_OK) {
        devEnum->Release();
        return false;   // 无视频设备
    }

    // 第一遍：先把所有候选设备的 path/name/moniker 收集齐，不提前绑定、
    // 不提前判断——这样才能在“路径匹配全部失败”时，退回按名字数第几个，
    // 而不是像以前那样一边扫一边比对，第一个同名的就直接抢先绑定掉。
    struct Cand { QString path, name; IMoniker* mon; };
    QVector<Cand> cands;
    IMoniker* mon = nullptr;
    ULONG fetched = 0;
    while (en->Next(1, &mon, &fetched) == S_OK) {
        IPropertyBag* bag = nullptr;
        QString path, name;
        if (SUCCEEDED(mon->BindToStorage(nullptr, nullptr, IID_IPropertyBag,
                                         reinterpret_cast<void**>(&bag)))) {
            VARIANT v; VariantInit(&v);
            if (SUCCEEDED(bag->Read(L"DevicePath", &v, nullptr)) && v.vt == VT_BSTR)
                path = QString::fromWCharArray(v.bstrVal);
            VariantClear(&v);
            if (SUCCEEDED(bag->Read(L"FriendlyName", &v, nullptr)) && v.vt == VT_BSTR)
                name = QString::fromWCharArray(v.bstrVal);
            VariantClear(&v);
            bag->Release();
        }
        cands.push_back({path, name, mon});   // 持有 moniker，稍后统一 Release
    }
    en->Release();
    devEnum->Release();

    // 第二遍：先试路径精确/包含匹配（Qt 设备id 与 DShow DevicePath 都是内核
    // 符号链接，格式一致时这条最可靠、天然唯一，不受同名影响）。
    const QString want = QString::fromLatin1(qtDeviceId).toLower();
    int chosen = -1;
    if (!want.isEmpty()) {
        for (int i = 0; i < cands.size(); ++i) {
            const QString p = cands[i].path.toLower();
            if (!p.isEmpty() && (p == want || p.contains(want) || want.contains(p))) {
                chosen = i;
                break;
            }
        }
    }
    // 路径匹配失败（比如 Qt 走 Media Foundation 时设备id格式与 DShow 的
    // DevicePath 压根不是同一套编码，两者永远对不上）——退化到按友好名字
    // 匹配，但取“同名设备里的第 nameOccurrenceIndex 个”，而不是第一个，
    // 这样四台友好名一样的红外相机才能一一对应到各自的物理设备。
    if (chosen < 0 && !friendlyName.isEmpty()) {
        int occurrence = 0;
        for (int i = 0; i < cands.size(); ++i) {
            if (cands[i].name == friendlyName) {
                if (occurrence == nameOccurrenceIndex) { chosen = i; break; }
                ++occurrence;
            }
        }
        // 兜底：序号超出实际同名设备数量（比如中途拔插导致数量对不上），
        // 退回第一个同名的——好过完全连不上，但这种情况下参数面板可能
        // 仍连到错的物理相机，只是没有更可靠的信息可用了。
        if (chosen < 0) {
            for (int i = 0; i < cands.size(); ++i)
                if (cands[i].name == friendlyName) { chosen = i; break; }
        }
    }

    bool ok = false;
    for (int i = 0; i < cands.size(); ++i) {
        if (i == chosen) {
            if (outMonikerIUnknown) { cands[i].mon->AddRef(); *outMonikerIUnknown = cands[i].mon; ok = true; }
        }
        cands[i].mon->Release();
    }
    return ok;
}

DShowControl::DShowControl(const QByteArray& qtDeviceId, const QString& friendlyName,
                           int nameOccurrenceIndex) {
    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    comInit_ = SUCCEEDED(hr);   // S_OK 或 S_FALSE(已初始化) 都需配对 Uninit

    void* monV = nullptr;
    if (!findMoniker(qtDeviceId, friendlyName, nameOccurrenceIndex, &monV) || !monV)
        return;
    IMoniker* mon = static_cast<IMoniker*>(monV);

    IBaseFilter* f = nullptr;
    if (SUCCEEDED(mon->BindToObject(nullptr, nullptr, IID_IBaseFilter,
                                    reinterpret_cast<void**>(&f))))
        filter_ = f;
    mon->Release();

    if (filter_) {
        IBaseFilter* bf = static_cast<IBaseFilter*>(filter_);
        IAMVideoProcAmp* amp = nullptr;
        IAMCameraControl* cc = nullptr;
        if (SUCCEEDED(bf->QueryInterface(IID_IAMVideoProcAmp, reinterpret_cast<void**>(&amp))))
            amp_ = amp;
        if (SUCCEEDED(bf->QueryInterface(IID_IAMCameraControl, reinterpret_cast<void**>(&cc))))
            cam_ = cc;
    }
}

DShowControl::~DShowControl() {
    if (amp_)    static_cast<IAMVideoProcAmp*>(amp_)->Release();
    if (cam_)    static_cast<IAMCameraControl*>(cam_)->Release();
    if (filter_) static_cast<IBaseFilter*>(filter_)->Release();
    if (comInit_) CoUninitialize();
}

bool DShowControl::valid() const { return filter_ != nullptr; }

QVector<DShowProp> DShowControl::properties() {
    QVector<DShowProp> out;
    IAMVideoProcAmp*  amp = static_cast<IAMVideoProcAmp*>(amp_);
    IAMCameraControl* cc  = static_cast<IAMCameraControl*>(cam_);

    for (const Item& it : kItems) {
        DShowProp p;
        p.id = it.id;
        p.isCameraControl = it.camCtrl;
        p.name = QString::fromUtf8(it.name);

        long mn = 0, mx = 0, st = 1, df = 0, caps = 0;
        HRESULT hr = E_FAIL;
        if (it.camCtrl && cc)      hr = cc->GetRange(it.id, &mn, &mx, &st, &df, &caps);
        else if (!it.camCtrl && amp) hr = amp->GetRange(it.id, &mn, &mx, &st, &df, &caps);

        if (SUCCEEDED(hr)) {
            p.supported = true;
            p.min = mn; p.max = mx; p.step = (st > 0 ? st : 1); p.def = df;
            if (it.camCtrl) {
                p.autoSupported   = (caps & CameraControl_Flags_Auto)   != 0;
                p.manualSupported = (caps & CameraControl_Flags_Manual) != 0;
            } else {
                p.autoSupported   = (caps & VideoProcAmp_Flags_Auto)    != 0;
                p.manualSupported = (caps & VideoProcAmp_Flags_Manual)  != 0;
            }
            long v = df, fl = 0;
            HRESULT hg = it.camCtrl ? cc->Get(it.id, &v, &fl)
                                    : amp->Get(it.id, &v, &fl);
            if (SUCCEEDED(hg)) {
                p.value = v;
                p.isAuto = it.camCtrl ? ((fl & CameraControl_Flags_Auto) != 0)
                                      : ((fl & VideoProcAmp_Flags_Auto)  != 0);
            } else {
                p.value = df;
            }
        }
        out.push_back(p);
    }
    return out;
}

bool DShowControl::set(long propId, bool isCameraControl, long value, bool isAuto) {
    if (isCameraControl) {
        IAMCameraControl* cc = static_cast<IAMCameraControl*>(cam_);
        if (!cc) return false;
        long fl = isAuto ? CameraControl_Flags_Auto : CameraControl_Flags_Manual;
        return SUCCEEDED(cc->Set(propId, value, fl));
    }
    IAMVideoProcAmp* amp = static_cast<IAMVideoProcAmp*>(amp_);
    if (!amp) return false;
    long fl = isAuto ? VideoProcAmp_Flags_Auto : VideoProcAmp_Flags_Manual;
    return SUCCEEDED(amp->Set(propId, value, fl));
}

} // namespace mocap

#else
// -------------------------- 非 Windows 桩 --------------------------
namespace mocap {
DShowControl::DShowControl(const QByteArray&, const QString&, int) {}
DShowControl::~DShowControl() {}
bool DShowControl::valid() const { return false; }
QVector<DShowProp> DShowControl::properties() { return {}; }
bool DShowControl::set(long, bool, long, bool) { return false; }
bool DShowControl::findMoniker(const QByteArray&, const QString&, int, void** outMonikerIUnknown) {
    if (outMonikerIUnknown) *outMonikerIUnknown = nullptr;
    return false;
}
} // namespace mocap
#endif