#pragma once
// ---------------------------------------------------------------------------
// 项目根目录定位 —— "复制改名无数次都还能自己找到"。
//
// 【为什么不能只用 PROJECT_SOURCE_DIR】那个宏是 CMake 在【配置那一刻】把
// CMAKE_SOURCE_DIR 的【绝对路径】焊进二进制里的。只要项目文件夹被改名、
// 挪位置、或者整份复制成新版本（"Mocap_System_v3" 这种），焊进去的那个
// 路径就指向旧文件夹了：程序还在老地方读写标定模板，用户在新文件夹里
// 怎么也看不到自己刚标好的东西——而且不会报错，只会"莫名其妙没有"，
// 是最难查的那种问题。重新跑一次 CMake 配置能修，但"必须记得重新配置"
// 本身就是个隐形陷阱。
//
// 【做法】改成运行时定位，按可靠性从高到低依次尝试：
//   ① 从可执行文件所在目录逐级向上找"项目根长什么样"的特征（同时存在
//      CMakeLists.txt 和 src/ 目录）。build 目录通常就在项目里（
//      <项目>/build/mocap_host.exe），向上两三级必然撞上项目根。这一步
//      完全基于"我现在实际在哪"，所以复制改名多少次都自动跟着走。
//   ② ①失败（比如 build 目录被放到项目外面，或者只拷了 exe 出去单独跑）
//      时，退回编译期焊入的 PROJECT_SOURCE_DIR，但会先确认那个路径当前
//      真的存在——存在才用，避免指向一个早就被删/改名的旧目录。
//   ③ 都不行就退回系统用户配置目录（AppConfigLocation）。这是最后兜底，
//      保证程序在任何情况下都有地方存数据、不至于起不来。
//
// 特征判定用"CMakeLists.txt + src/ 同时存在"而不是只看 CMakeLists.txt：
// 单看一个文件太容易误命中（上层目录里随便躺一个 CMakeLists.txt 就会
// 认错），两个条件一起要求基本不会误判。
// ---------------------------------------------------------------------------
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QString>
#include <QStandardPaths>

namespace mocap {

inline QString projectRootDir() {
    // 结果缓存：rootDir() 这类函数会被频繁调用（每次拼子路径都会调一次），
    // 没必要每次都去敲磁盘。注意只在 QCoreApplication 已经建好之后才缓存——
    // 否则 applicationDirPath() 返回空、①必然失败，那个错误结果会被永久
    // 焊死在缓存里。
    static QString cached;
    if (!cached.isEmpty()) return cached;

    const auto looksLikeProjectRoot = [](const QDir& d) {
        return QFile::exists(d.filePath(QStringLiteral("CMakeLists.txt")))
            && QDir(d.filePath(QStringLiteral("src"))).exists();
    };

    QString result;

    // ① 从 exe 位置向上找。8 级足够覆盖任何正常的 build 目录嵌套深度，
    //    给个上限纯粹是防御——万一 cdUp() 在某些路径上不按预期收敛，
    //    也不至于把这里变成死循环。
    if (QCoreApplication::instance()) {
        QDir d(QCoreApplication::applicationDirPath());
        for (int i = 0; i < 8; ++i) {
            if (looksLikeProjectRoot(d)) { result = d.absolutePath(); break; }
            if (!d.cdUp()) break;
        }
    }

    // ② 退回编译期焊入的源码目录（存在才用）。
#ifdef PROJECT_SOURCE_DIR
    if (result.isEmpty()) {
        const QDir baked(QStringLiteral(PROJECT_SOURCE_DIR));
        if (baked.exists()) result = baked.absolutePath();
    }
#endif

    // ③ 最后兜底：系统用户配置目录。
    if (result.isEmpty()) {
        result = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
        if (result.isEmpty()) result = QDir::homePath();
    }

    if (QCoreApplication::instance()) cached = result;
    return result;
}

} // namespace mocap
