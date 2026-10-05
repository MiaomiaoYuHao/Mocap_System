// ===========================================================================
// OrtRuntimeLoader.hpp
//
// onnxruntime 的手动加载 + Ort::InitApi，从原 OnnxSkeletonInferenceBackend.hpp
// 里抽出来的（那个文件已随上一代契约一起删除）。
//
// 【为什么必须手动加载，不能直接链接】
// CMakeLists.txt:169-256 里只从 onnxruntime::onnxruntime 取了
// INTERFACE_INCLUDE_DIRECTORIES，【故意没有】target_link_libraries。原因写在
// 那段注释里：静态链接 .lib 会让 PE 导入表写死裸文件名 "onnxruntime.dll"，
// 而 Windows 的 Known DLLs 里有同名项，加载器会在程序启动时就把它重定向到
// 系统那份（给 WinML 用的），跟 exe 旁边这份版本对不上。所以构建期把 dll
// 改名成 onnxruntime_mocap.dll 拷到 exe 旁，运行时用完整路径显式加载。
//
// 直接写 Ort::Env / Ort::Session 而不做这一步的后果：
//   1. 链接失败（没有导入库）
//   2. 就算链上了，onnxruntime_cxx_api.h 内部 Global<void>::api_ 会在静态
//      初始化阶段调用 OrtGetApiBase()，又绕回被重定向的老路
// 所以 ORT_API_MANUAL_INIT 必须在 include 之前定义，且必须先调用
// ensureOrtApiLoaded() 再构造任何 Ort:: 对象。
//
// 【已知缺口】原 OnnxSkeletonInferenceBackend.hpp 里的
// listDirectPeDependencies()（解析 PE 导入表、逐个探测缺哪个依赖）没有
// 一起搬过来——那是纯机械移动，建议你把它也挪进本文件的 detail 命名空间，
// 然后让 ensureOrtApiLoaded 在失败分支里调用，诊断信息就不会丢。
// 现在的失败分支只给错误码和 dll 路径。
// ===========================================================================
#pragma once

#include <string>

#ifdef HAVE_ONNXRUNTIME

// 必须在 include onnxruntime_cxx_api.h 之前定义
#ifndef ORT_API_MANUAL_INIT
#define ORT_API_MANUAL_INIT
#endif
#include <onnxruntime_cxx_api.h>

#ifdef _WIN32
#include <windows.h>
#endif

namespace mocap {
namespace ort {

// 进程内只做一次，结果缓存。线程安全由调用方保证（现在只在 worker 线程调）。
// 返回 false 时 err 里是可以直接显示给用户的中文原因。
inline bool ensureOrtApiLoaded(std::string& err) {
    static bool inited = false;
    static bool ok = false;
    static std::string cachedErr;
    if (inited) { err = cachedErr; return ok; }
    inited = true;

#ifdef _WIN32
    wchar_t exePath[MAX_PATH];
    GetModuleFileNameW(nullptr, exePath, MAX_PATH);
    std::wstring dir(exePath);
    const auto slash = dir.find_last_of(L"\\/");
    dir = (slash == std::wstring::npos) ? L"." : dir.substr(0, slash);
    const std::wstring dllPath = dir + L"\\onnxruntime_mocap.dll";

    // SetDllDirectoryW 作用于整条依赖解析链（LOAD_WITH_ALTERED_SEARCH_PATH
    // 只管第一层），加载完立刻恢复，不长期改动进程级搜索行为。
    SetDllDirectoryW(dir.c_str());
    HMODULE mod = LoadLibraryExW(dllPath.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
    const DWORD loadErrCode = mod ? 0 : GetLastError();
    SetDllDirectoryW(nullptr);

    if (!mod) {
        cachedErr = "onnxruntime_mocap.dll 加载失败(错误码"
                  + std::to_string(loadErrCode)
                  + ")。确认它已由 CMake 的 POST_BUILD 复制到可执行文件目录，"
                    "且 abseil_dll.dll / libprotobuf.dll / re2.dll 等依赖来自同一次 "
                    "vcpkg 安装。若错误码是 126，多半是依赖缺失或版本不一致。";
        err = cachedErr;
        return false;
    }

    using OrtGetApiBaseFn = const OrtApiBase*(ORT_API_CALL*)(void);
    auto getApiBase = reinterpret_cast<OrtGetApiBaseFn>(GetProcAddress(mod, "OrtGetApiBase"));
    if (!getApiBase) {
        cachedErr = "onnxruntime_mocap.dll 里找不到 OrtGetApiBase 导出函数，"
                    "文件可能不是有效的 onnxruntime 动态库";
        err = cachedErr;
        return false;
    }
    const OrtApiBase* base = getApiBase();
    const OrtApi* api = base ? base->GetApi(ORT_API_VERSION) : nullptr;
    if (!api) {
        cachedErr = "onnxruntime_mocap.dll 的 API 版本跟编译时的头文件不匹配，"
                    "确认 exe 旁边这份 dll 跟 vcpkg 里的版本一致";
        err = cachedErr;
        return false;
    }
    Ort::InitApi(api);
#else
    // 非 Windows 没有 Known DLLs 那套重定向，正常隐式链接即可；
    // 但 ORT_API_MANUAL_INIT 已经定义，所以仍要显式初始化一次。
    Ort::InitApi(OrtGetApiBase()->GetApi(ORT_API_VERSION));
#endif

    ok = true;
    err.clear();
    return true;
}

} // namespace ort
} // namespace mocap

#endif // HAVE_ONNXRUNTIME
