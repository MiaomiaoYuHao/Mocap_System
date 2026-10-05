cmake_minimum_required(VERSION 3.20)
project(mocap_host LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 注意：需要在 Qt 安装器里勾选 "Qt Multimedia" 组件，否则这里会报
# "Could not find a package configuration file provided by Qt6Multimedia"。
find_package(Qt6 REQUIRED COMPONENTS Widgets Multimedia Network Concurrent)

include_directories(${CMAKE_SOURCE_DIR}/src)

qt_add_executable(mocap_host
    src/main.cpp
    src/ui/MainWindow.cpp
    src/ui/CameraView.cpp
    src/ui/CameraGrid.cpp
    src/ui/AddCameraDialog.cpp
    src/ui/CamParamDialog.cpp
    src/ui/CalibrationDialog.cpp
    src/ui/CalibWizard.cpp
    src/ui/CalibWizard_rounds.cpp   # 【拆文件】CalibWizard.cpp 的标定板接力/求解分片
    src/ui/CalibResultView.cpp
    src/ui/SyncMonitor.cpp
    src/ui/DetectParamsPanel.cpp
    src/ui/TriangulationView.cpp
    src/ui/TriangulationDebugDialog.cpp
    src/ui/PointCloudTestDialog.cpp
    src/ui/PointCloudTestDialog_widgets.hpp
    src/ui/PointCloudTestDialog_panel.cpp     # 【拆文件】buildParamSections（原构造 632~2379 行）
    src/ui/PointCloudTestDialog_pipeline.cpp   # 【拆文件】析构/骨架结果/rebuild/录制文件头
    src/ui/PointCloudTestDialog_capture.cpp   # 【拆文件】录制目录/采集/聚类 tryCluster
    src/ui/PointCloudTestDialog_uievents.cpp  # 【拆文件】面板控件操作的全量事件记录
    # 【新增】输出数据实时打印面板（点云面板右侧那块）。有 Q_OBJECT，
    # AUTOMOC 会自动扫同名 .hpp，不需要额外写 moc 规则。
    src/ui/HandOutputMonitor.cpp
    src/ui/WandPrecisionDialog.cpp
    src/ui/HandPoseDebugDialog.cpp
    src/ui/HandTrackingParamsPanel.cpp
    src/ui/HandCalibrationWizard.cpp
    src/camera/ICamera.cpp
    src/camera/VirtualCamera.cpp
    src/camera/WebcamCamera.cpp
    src/camera/DShowControl.cpp
    src/camera/DShowCapture.cpp
    src/camera/JpegDecoder.cpp
    src/camera/CameraManager.cpp
    # 【新增】相机显示名。六台同型号相机的 FriendlyName 来自 USB 描述符的
    # iProduct，厂商没写序列号，六个一字不差 —— 界面上六行全是 "USB Camera"。
    # 这个文件从 DevicePath 派生出稳定的短标识（无序列号时由父 Hub+端口生成，
    # 换口才会变），Windows 下再查一次 USB 物理位置 Port_#/Hub_#。
    # 字符串那半边零平台依赖；queryUsbLocation() 整体裹在 #ifdef Q_OS_WIN 里，
    # 非 Windows 是头文件里返回空的 inline stub，所以 .cpp 在哪都能编。
    src/camera/CameraLabel.cpp
    src/camera/CameraLabel.hpp
    src/detect/CentroidDetector.cpp
    src/detect/DetectWorker.cpp
    src/calib/Calibration.cpp
    src/calib/CalibrationStore.cpp
    src/calib/CalibSettingsStore.cpp
    src/calib/CalibrationLibrary.cpp
    src/calib/RoundConnectivity.cpp
    src/ui/CalibrationLibraryDialog.cpp
    src/ui/CameraRemapDialog.cpp
    src/ui/DShowCaptureTestDialog.cpp
    src/reconstruct/Triangulation.cpp
    src/reconstruct/Triangulator.cpp
    src/reconstruct/OneEuroFilter.hpp
    src/reconstruct/TrackFrameDebug.hpp
    src/estimate/HandTrackingWorker.cpp
    src/estimate/SmootherWorker.cpp
    src/estimate/SkeletonAssocWorker.hpp
    # hm20 骨架关联的纯头文件：都没有 Q_OBJECT，AUTOMOC 不需要扫，列在这里
    # 只是为了在 IDE 的项目树里可见；删掉也不影响编译。
    # 【拆文件】HandSkeletonAssociator.hpp 已拆为 Hm20AssocBase.hpp / Hm20AssocContract.hpp
    # 两个独立头，加 Hm20Assoc_*.ipp 八个类体分片（include 进类内）。
    # 分片是【按职责】切的，不是按行号——依据是 92 个数据成员里有 80 个只被
    # 1~3 个函数碰，也就是说这个类是几台互不相干的状态机挤在一起。
    # 导航表在 HandSkeletonAssociator.hpp 顶部，改之前先看那张表，别整包读。
    # Hm20AutoCalib.hpp 拆出 Hm20AutoCalibTypes.hpp。
    src/estimate/Hm20OnnxBackend.hpp
    src/estimate/OrtRuntimeLoader.hpp
    src/estimate/Hm20TemplateAdapter.hpp
    src/estimate/Hm20IkRefiner.hpp
    src/estimate/Hm20AutoCalib.hpp
    src/estimate/Hm20AutoCalibTypes.hpp
    src/estimate/HandSkeletonAssociator.hpp
    src/estimate/Hm20AssocBase.hpp
    src/estimate/Hm20AssocContract.hpp
    src/estimate/Hm20Assoc_types.ipp
    src/estimate/Hm20Assoc_api.ipp
    src/estimate/Hm20Assoc_process.ipp
    src/estimate/Hm20Assoc_chain.ipp
    src/estimate/Hm20Assoc_dorsum.ipp
    src/estimate/Hm20Assoc_thumb.ipp
    src/estimate/Hm20Assoc_segrot.ipp
    src/estimate/Hm20Assoc_geom.ipp
    src/estimate/Hm20PoseFilter.hpp
    src/estimate/DorsumRigidSolver.hpp
    src/estimate/Hm20SegRot.hpp
    src/estimate/ThumbPronationCalib.hpp
    src/record/PointCloudRecorder.hpp
    src/estimate/Hm20JointAngles.hpp
    src/estimate/RomCalibration.hpp
    src/net/UdpSender.cpp
    # 【新增】M3DS/M3DQ 字节布局的唯一定义。UdpSender 和输出监视面板共用它，
    # 保证"面板上打印的字节"就是"网线上跑的字节"。纯头文件，列在这里只为
    # IDE 项目树可见。
    src/net/HandPacket.hpp
    src/settings/AppSettings.cpp
    src/settings/ParamPresets.cpp
)

find_package(Threads REQUIRED)
target_link_libraries(mocap_host PRIVATE Qt6::Widgets Qt6::Multimedia Qt6::Network Qt6::Concurrent Threads::Threads)

# ---- libjpeg-turbo 灰度直解（默认开）----------------------------------------
# 开启后，MJPG 帧在解码源头就直接解成单通道灰度(TJPF_GRAY)，跳过色度平面上
# 采样和 YCbCr→RGB 矩阵——对“红外滤光片 + 黑底白球”画面零信息损失，且解码更快、
# 下游全链路 1/3 数据量。4 路×120fps 的解码密度下这是收益最高的单项优化(更不易
# 丢帧、主机发热腰斩)。运行时还要在 UI 里打开“算法灰度”开关才生效——这里只是把
# 库编进来、让灰度直解路径可用(HAVE_TURBOJPEG)；没编进来时状态栏那个“灰度直解”
# 统计永远不会出现、只会显示“彩色解码”，因为走的是 QImage::fromData 完整解码。
#
# 【默认开 + 缺库不崩】以前默认关、且 -DUSE_TURBOJPEG=ON 找不到库会 FATAL_ERROR。
# 现在默认就 ON，找不到库不再让整个构建崩掉，而是打一条醒目 WARNING 后优雅回退
# 到 Qt 软解(功能不变、只是不省 CPU、也不会有“灰度直解”统计)。这样装了库的机器
# 开箱即用、看得到灰度直解；没装库的机器照样编得过。
# 装库：vcpkg install libjpeg-turbo(配置时带上 vcpkg 工具链文件)，或系统包管理器
# 装 libturbojpeg-dev/libturbojpeg0-dev。
# 想彻底不依赖 turbo：配置时加 -DUSE_TURBOJPEG=OFF。
option(USE_TURBOJPEG "Enable libjpeg-turbo grayscale-direct MJPG decoding" ON)

# 【vcpkg 自动发现】不带 -DCMAKE_TOOLCHAIN_FILE 也尽量能找到 vcpkg 装的库。
#
# 为什么需要这个：CMAKE_TOOLCHAIN_FILE 和 find_package 的结果都存在
# CMakeCache.txt 里，而缓存记的是绝对路径。一旦项目文件夹被改名/复制成新
# 版本，缓存就作废并被重建；重建时如果配置命令里没再带工具链参数，就会
# 突然"以前能找到 libjpeg-turbo、现在找不到了"——参数只在命令行传过一次、
# 没有跟着项目走，这才是根因。把发现逻辑写进 CMakeLists.txt，它就跟项目
# 一起被复制，复制改名多少次都还在。
#
# 做法是往 CMAKE_PREFIX_PATH 里追加 vcpkg 的 installed/<triplet> 目录，而不是
# 去设 CMAKE_TOOLCHAIN_FILE（那个必须在 project() 之前设置，这里已经晚了）。
# 对 find_package(<pkg> CONFIG) 来说两者效果一样。triplet 名字因平台而异
# （x64-windows / x64-mingw-dynamic / x64-linux…），所以直接把 installed/ 下
# 所有子目录都加进去，让 CMake 自己挑得到的那个，不去猜。
if (USE_TURBOJPEG AND NOT CMAKE_TOOLCHAIN_FILE)
    set(_vcpkg_candidates "")
    if (DEFINED ENV{VCPKG_ROOT})
        list(APPEND _vcpkg_candidates "$ENV{VCPKG_ROOT}")
    endif()
    # 常见安装位置兜底——没设 VCPKG_ROOT 环境变量的机器也有机会命中。
    list(APPEND _vcpkg_candidates
        "C:/vcpkg" "C:/dev/vcpkg" "C:/src/vcpkg"
        "$ENV{USERPROFILE}/vcpkg" "$ENV{HOME}/vcpkg")
    foreach(_vr ${_vcpkg_candidates})
        file(TO_CMAKE_PATH "${_vr}" _vr)
        if (IS_DIRECTORY "${_vr}/installed")
            file(GLOB _vcpkg_trees "${_vr}/installed/*")
            foreach(_tree ${_vcpkg_trees})
                if (IS_DIRECTORY "${_tree}")
                    list(APPEND CMAKE_PREFIX_PATH "${_tree}")
                endif()
            endforeach()
            message(STATUS "vcpkg: 已自动加入搜索路径 ${_vr}/installed/*")
            break()   # 命中一个就够，不叠加多份 vcpkg
        endif()
    endforeach()
endif()

if (USE_TURBOJPEG)
    # vcpkg 装 libjpeg-turbo 后导出的 config。按官方推荐用法：哪个 target 存在
    # 用哪个(动态 turbojpeg 优先，否则 turbojpeg-static)——见 vcpkg 安装时打印的
    # “libjpeg-turbo provides CMake targets for the TurboJPEG C API”那段。
    find_package(libjpeg-turbo CONFIG QUIET)
    if (TARGET libjpeg-turbo::turbojpeg OR TARGET libjpeg-turbo::turbojpeg-static)
        target_link_libraries(mocap_host PRIVATE
            $<IF:$<TARGET_EXISTS:libjpeg-turbo::turbojpeg>,libjpeg-turbo::turbojpeg,libjpeg-turbo::turbojpeg-static>)
        target_compile_definitions(mocap_host PRIVATE HAVE_TURBOJPEG)
        message(STATUS "libjpeg-turbo: 已启用灰度直解 (vcpkg config target)")
    else()
        # 退回手动找头/库(不走 vcpkg、系统装 libturbojpeg-dev 或官方安装包的情形)
        find_path(TURBOJPEG_INCLUDE_DIR turbojpeg.h)
        find_library(TURBOJPEG_LIBRARY NAMES turbojpeg)
        if (TURBOJPEG_INCLUDE_DIR AND TURBOJPEG_LIBRARY)
            target_include_directories(mocap_host PRIVATE ${TURBOJPEG_INCLUDE_DIR})
            target_link_libraries(mocap_host PRIVATE ${TURBOJPEG_LIBRARY})
            target_compile_definitions(mocap_host PRIVATE HAVE_TURBOJPEG)
            message(STATUS "libjpeg-turbo: 已启用灰度直解 (${TURBOJPEG_LIBRARY})")
        else()
            # 缺库不再崩构建：打一条醒目告警后回退到 Qt 软解。不定义 HAVE_TURBOJPEG，
            # 于是 JpegDecoder.cpp 编的是 #else 那份纯 Qt 实现，灰度靠 convertToFormat
            # 事后转——功能一致、只是不省 CPU，且状态栏只会显示“彩色解码”。
            message(WARNING
                "USE_TURBOJPEG=ON 但没找到 libjpeg-turbo，已【回退到 Qt 软解】"
                "(功能不变、不省 CPU，状态栏不会出现“灰度直解”统计)。想用灰度直解请 "
                "vcpkg install libjpeg-turbo 并带上 vcpkg 工具链文件 "
                "(-DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake)，"
                "或系统装 libturbojpeg-dev；想彻底关掉这条告警配置时加 -DUSE_TURBOJPEG=OFF。")
        endif()
    endif()
else()
    message(STATUS "libjpeg-turbo: 未启用 (USE_TURBOJPEG=OFF)，MJPG 走 Qt 软解，"
                   "灰度靠解码后转换、状态栏只显示“彩色解码”。")
endif()

# ---- onnxruntime（手部骨骼关联，点云测试界面的AI叠加层，可选）------------
# 跟上面 libjpeg-turbo 同一个模式：CONFIG 优先，找不到不 FATAL_ERROR，只是
# 功能性关闭(骨架不显示，点云本身完全不受影响)。USE_ONNXRUNTIME=OFF 或没
# 装库时，estimate/Hm20OnnxBackend.hpp 编译出的是桩实现
# (ready()恒为false)，PointCloudTestDialog 正常运行，只是"叠加显示骨架"
# 那个勾选框勾了也没效果，工具提示里会显示原因。
# 装库：vcpkg install onnxruntime，或系统包管理器/官方预编译包，把
# lib/cmake/onnxruntime 路径加进 CMAKE_PREFIX_PATH。
option(USE_ONNXRUNTIME "Enable ONNX Runtime for AI hand skeleton association" ON)

if (USE_ONNXRUNTIME)
    find_package(onnxruntime CONFIG QUIET)
    if (TARGET onnxruntime::onnxruntime)
        # 【重要，别改回旧写法】故意不 target_link_libraries 链接
        # onnxruntime::onnxruntime 这个导入库——静态链接.lib会让PE导入表里
        # 写死裸文件名"onnxruntime.dll"，程序一启动，Windows加载器就已经
        # 按系统的"Known DLLs"名单强制从C:\Windows\System32加载了(较新
        # Windows把onnxruntime.dll也塞进了这个名单，给WinML用)，跟你exe
        # 旁边放的版本(即使更新)完全无关，来不及在main()里补救。
        # 只拿头文件目录，不链接导入库；真正的加载在
        # OrtRuntimeLoader.hpp 里用LoadLibraryExW+完整
        # 路径手动做(完整路径显式加载不受Known DLLs的裸名重定向影响)。
        get_target_property(_ort_inc_dirs onnxruntime::onnxruntime INTERFACE_INCLUDE_DIRECTORIES)
        target_include_directories(mocap_host PRIVATE ${_ort_inc_dirs})
        target_compile_definitions(mocap_host PRIVATE HAVE_ONNXRUNTIME)
        message(STATUS "onnxruntime: 已启用骨骼关联推理(动态加载，绕开Known DLLs)")

        # 把真正要用的dll从vcpkg装的地方复制到exe旁边，故意改名成
        # onnxruntime_mocap.dll——Known DLLs是按精确文件名匹配的，改名后
        # 系统不会对它做任何重定向，代码里就按这个改名后的文件名去加载。
        #
        # 【为什么不直接用 $<TARGET_FILE:onnxruntime::onnxruntime>】那个
        # 生成表达式只有当导入目标被声明成 SHARED 时才解析成 .dll；vcpkg
        # 生成的配置经常把它声明成 UNKNOWN，那样解析出来的是【.lib导入库】，
        # 复制过去改名成.dll也加载不了，而且要等到运行时 LoadLibraryExW
        # 失败才发现。这里改成在【配置阶段】就把真实dll找出来并打印路径，
        # 有问题当场就能看见，不用等运行。
        # 【必须用 Release 版的 onnxruntime，不能用 Debug 版】
        # vcpkg 会同时装 release(installed/<triplet>/bin) 和
        # debug(installed/<triplet>/debug/bin) 两份。Debug 版依赖的是
        # 【调试版MSVC运行时】ucrtbased.dll / VCRUNTIME140D.dll /
        # MSVCP140D.dll 以及 libprotobufd.dll 这类带d后缀的调试库——
        # 调试版运行时只随 Visual Studio 安装、微软【禁止再分发】，
        # 装"VC++可再发行程序包"也不会有，所以 Debug 版 onnxruntime 在
        # 没装VS的机器上必然 LoadLibrary 失败(错误码126，实测踩过)。
        # Release 版依赖的是 MSVCP140.dll / VCRUNTIME140.dll / ucrtbase.dll
        # 加 abseil_dll/libprotobuf/re2，都是可再发行的正常依赖。
        # 本工程即使自身是 Debug 构建，也照样用 Release 版 onnxruntime：
        # 跨DLL边界是纯C API(OrtApi函数指针表)，不传STL对象、不跨边界
        # new/delete，不存在CRT混用的风险。
        set(_ort_dll "")

        # ①先在 vcpkg 的 release bin 目录里找(明确排除 debug/bin)
        set(_ort_release_hints "")
        foreach(_t ${_vcpkg_trees})
            list(APPEND _ort_release_hints "${_t}/bin")
        endforeach()
        # 从导入库位置反推 <root>/bin 也算一条 release 线索
        get_target_property(_ort_implib onnxruntime::onnxruntime IMPORTED_IMPLIB_RELEASE)
        if (NOT _ort_implib)
            get_target_property(_ort_implib onnxruntime::onnxruntime IMPORTED_IMPLIB)
        endif()
        if (_ort_implib)
            get_filename_component(_ort_libdir "${_ort_implib}" DIRECTORY)
            get_filename_component(_ort_root "${_ort_libdir}" DIRECTORY)
            # 如果这个路径本身就在 debug 树下，退一级到 release 根
            if (_ort_root MATCHES "/debug$")
                get_filename_component(_ort_root "${_ort_root}" DIRECTORY)
            endif()
            list(APPEND _ort_release_hints "${_ort_root}/bin")
        endif()
        unset(_ort_dll_found CACHE)   # 不吃上一次configure的缓存结果
        find_file(_ort_dll_found NAMES onnxruntime.dll HINTS ${_ort_release_hints} NO_DEFAULT_PATH)
        if (_ort_dll_found)
            set(_ort_dll "${_ort_dll_found}")
        endif()

        # ②release 找不到才回退问导入目标的 IMPORTED_LOCATION_RELEASE
        if (NOT _ort_dll)
            get_target_property(_loc onnxruntime::onnxruntime IMPORTED_LOCATION_RELEASE)
            if (_loc AND _loc MATCHES "\\.(dll|DLL)$")
                set(_ort_dll "${_loc}")
            endif()
        endif()

        if (_ort_dll AND _ort_dll MATCHES "/debug/")
            message(WARNING
                "定位到的 onnxruntime.dll 位于 debug 目录(${_ort_dll})。Debug版依赖"
                "调试版MSVC运行时(ucrtbased.dll等)，那套东西只随Visual Studio安装、"
                "不可再分发，运行时会加载失败(错误码126)。请确认 vcpkg 已安装 release 版："
                "vcpkg install onnxruntime:x64-windows")
        endif()

        if (_ort_dll)
            message(STATUS "onnxruntime 运行库(release): ${_ort_dll} -> onnxruntime_mocap.dll")
            add_custom_command(TARGET mocap_host POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_ort_dll}"
                    $<TARGET_FILE_DIR:mocap_host>/onnxruntime_mocap.dll
                COMMENT "复制onnxruntime运行库(release版，改名避开Windows Known DLLs重定向)")

            # 【重要，别漏】onnxruntime 自己还依赖 abseil_dll.dll /
            # libprotobuf.dll / libprotobuf-lite.dll / re2.dll 这几个vcpkg库。
            # 正常情况下 vcpkg 工具链有个"applocal deployment"机制——只要
            # target_link_libraries 链接了某个vcpkg库，就会自动把它的运行时
            # 依赖复制到exe旁边。但上面为了绕开Windows Known DLLs对
            # "onnxruntime.dll"这个文件名的强制重定向，故意没有
            # target_link_libraries链接onnxruntime::onnxruntime(只拿了头文件
            # 目录)，副作用是vcpkg也就不知道这个库需要哪些依赖了，这几个
            # 传递依赖不会再被自动复制——必须在这里手动补上，否则表现就是
            # "onnxruntime_mocap.dll加载失败(错误码126)，缺少xxx"，而xxx其实
            # 从来没被复制到exe旁边过(不是"丢了"，是"压根没来过")。
            get_filename_component(_ort_bindir "${_ort_dll}" DIRECTORY)

            # 逐个定位依赖，并把结果【直接打印在cmake配置输出里】——不需要
            # 手动去build目录翻文件确认。找的时候不只看onnxruntime.dll所在
            # 目录，还会扫所有vcpkg安装树的bin(不同库可能装在不同triplet下)。
            set(_ort_dep_search "${_ort_bindir}")
            foreach(_t ${_vcpkg_trees})
                list(APPEND _ort_dep_search "${_t}/bin")
            endforeach()

            set(_ort_deps_found "")
            set(_ort_deps_missing "")
            foreach(_dep_dll abseil_dll.dll libprotobuf.dll libprotobuf-lite.dll re2.dll
                             onnxruntime_providers_shared.dll)
                unset(_dep_path CACHE)
                find_file(_dep_path NAMES ${_dep_dll} HINTS ${_ort_dep_search} NO_DEFAULT_PATH)
                if (_dep_path)
                    list(APPEND _ort_deps_found "${_dep_dll}")
                    add_custom_command(TARGET mocap_host POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            "${_dep_path}"
                            $<TARGET_FILE_DIR:mocap_host>/${_dep_dll}
                        COMMENT "  [onnxruntime依赖] 复制 ${_dep_dll}")
                else()
                    list(APPEND _ort_deps_missing "${_dep_dll}")
                endif()
            endforeach()

            message(STATUS "================ 骨架AI(onnxruntime) 依赖检查 ================")
            message(STATUS "  主运行库 : ${_ort_dll}")
            message(STATUS "             -> 将复制为 onnxruntime_mocap.dll")
            if (_ort_deps_found)
                message(STATUS "  已定位依赖(构建时自动复制到exe旁边):")
                foreach(_d ${_ort_deps_found})
                    message(STATUS "      [OK] ${_d}")
                endforeach()
            endif()
            if (_ort_deps_missing)
                message(STATUS "  未找到(多数情况无害——这几个可能本版本不需要，"
                               "或已被其它库的部署机制复制过):")
                foreach(_d ${_ort_deps_missing})
                    message(STATUS "      [--] ${_d}")
                endforeach()
            endif()
            message(STATUS "  提示：以上复制发生在【编译(build)】阶段，只配置(configure)不会产生文件。")
            message(STATUS "        编译输出里会出现若干行 \"[onnxruntime依赖] 复制 xxx.dll\"，")
            message(STATUS "        看到那些行就说明确实复制了。")
            message(STATUS "==============================================================")

            # 编译完成后再自检一次：直接在【编译输出】里打印每个文件到底
            # 在不在exe旁边，用户不需要自己去目录里翻。用cmake -E脚本模式
            # 实现，跨平台且不依赖shell语法。
            #
            # 【为什么这里只置一个标志，而不是当场 add_custom_command】
            # 同一个 target 上的 POST_BUILD 命令【按注册顺序执行】。自检必须是
            # 最后一条，否则它会先于"复制 .onnx 模型"那条跑掉，看到的是【上一次】
            # 构建留下的目录状态 —— 首次构建永远报"[缺失] 模型"，改完文件名也照报，
            # 得再编译一遍才变 OK。真正的注册挪到本文件末尾模型复制之后。
            set(_verify_skeleton_ai TRUE)
        else()
            message(WARNING
                "找到了 onnxruntime 的头文件/导入库，但【没能定位到 release 版的 onnxruntime.dll】，"
                "无法复制到可执行文件旁边，骨架AI功能将不可用。"
                "请手动把 vcpkg 的 installed/x64-windows/bin/onnxruntime.dll(注意：不是 debug/bin 下那个) "
                "复制到可执行文件目录并改名为 onnxruntime_mocap.dll。")
        endif()
    else()
        message(WARNING
            "USE_ONNXRUNTIME=ON 但没找到 onnxruntime，骨骼关联功能将【不可用】"
            "(点云测试窗口照常显示点云，勾选“叠加显示骨架”不会有效果)。"
            "vcpkg install onnxruntime，或配置时加 -DUSE_ONNXRUNTIME=OFF 关闭此告警。")
    endif()
endif()

# 把源码根目录的绝对路径焊进程序里，跟 build 目录（会被删/重建）无关。
# 标定向导默认把会话存到 <项目目录>/calib_sessions/ 下，用这个宏而不是
# QCoreApplication::applicationDirPath()（那个是 build 输出目录，删 build
# 重来时之前存的标定数据会跟着一起丢，风险太大）。
target_compile_definitions(mocap_host PRIVATE
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

if (WIN32)
    # DirectShow COM（IAMVideoProcAmp / IAMCameraControl）所需系统库
    # cfgmgr32：CameraLabel.cpp 用 CM_Locate_DevNodeW / CM_Get_DevNode_Registry_PropertyW
    # 查 USB 端口位置。【MinGW 必须显式链】—— MSVC 那边文件里有 #pragma comment
    # 能兜住，MinGW 不认那个 pragma，漏了就是链接期 undefined reference。
    target_link_libraries(mocap_host PRIVATE ole32 oleaut32 strmiids uuid cfgmgr32)
    set_target_properties(mocap_host PROPERTIES WIN32_EXECUTABLE ON)
endif()

# ---- 纯逻辑单元测试（不依赖 Qt GUI 运行）----
enable_testing()
add_executable(test_gridlayout tests/test_gridlayout.cpp)
target_include_directories(test_gridlayout PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME gridlayout COMMAND test_gridlayout)

# 标定板接力（第③步）的连通性数学核心——并查集/主分量/角点门槛/边表构建，
# 见 src/calib/RoundConnectivity.hpp 顶部注释。只用 Qt::Core 容器
# （QVector/QPair/QHash），不摸 GUI/相机/文件系统，可以脱离 Qt GUI 单独
# 编译测试，跟 test_triangulation 需要 Qt6::Core 是同一个道理（拿 Qt 的
# include 路径，不代表真的调用了 Qt 运行时功能）。
add_executable(test_roundconnectivity tests/test_round_connectivity.cpp src/calib/RoundConnectivity.cpp)
target_include_directories(test_roundconnectivity PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_roundconnectivity PRIVATE Qt6::Core)
add_test(NAME roundconnectivity COMMAND test_roundconnectivity)

# --detect-only 端到端测试（真实调用 solve_calibration.py，用 OpenCV 自己生成
# 的标准 ChArUco 板图像验证角点检测）。需要机器上装了 opencv-contrib-python；
# 没装的话脚本自己打印 SKIP、退出码 0，不会让 ctest 因为环境缺失而报失败，
# 见 tests/test_detect_only.py 顶部注释。
find_package(Python3 COMPONENTS Interpreter QUIET)
if (Python3_Interpreter_FOUND)
    add_test(NAME detect_only
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_detect_only.py)
endif()

add_executable(test_centroid tests/test_centroid.cpp src/detect/CentroidDetector.cpp)
target_include_directories(test_centroid PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME centroid COMMAND test_centroid)

# CentroidDetector 新加的轮廓收集能力(collectContours 参数)——零 Qt 依赖，
# 同 test_centroid 一样能脱离 Qt GUI 独立测。
add_executable(test_centroid_contours tests/test_centroid_contours.cpp src/detect/CentroidDetector.cpp)
target_include_directories(test_centroid_contours PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME centroidcontours COMMAND test_centroid_contours)

# BlobObservationAdapter 把轮廓点接进3a~3d流水线、再转换回归一化坐标——
# 这个需要 calib/Calibration.hpp + reconstruct/Triangulation.hpp，两者都
# 拉 Qt(QString/QJsonObject)，所以要链 Qt6::Core 才能编译(跟 test_triangulation
# 同样的道理：不调用 Qt 运行时功能，但编译期要能找到这两个头文件)。
add_executable(test_blob_observation_adapter tests/test_blob_observation_adapter.cpp src/reconstruct/Triangulation.cpp)
target_include_directories(test_blob_observation_adapter PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_blob_observation_adapter PRIVATE Qt6::Core)
add_test(NAME blobobservationadapter COMMAND test_blob_observation_adapter)

# 遮挡感知检测层 · 3a：已知半径约束的圆拟合（ConstrainedCircleFit.hpp）。
# 纯数学、零 Qt 依赖，是 3b（半遮挡圆弧）、3c（花生双圆分离）共同的地基。
# ---- ROM 标定引擎 v2（纯数学，零 Qt，可单独跑）----
# 复现真机上报的三个症状并验证修复：覆盖度上不去、方向反了、拇MCP 恒零。
add_executable(test_rom_calibration tests/test_rom_calibration.cpp)
target_include_directories(test_rom_calibration PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME romcalibration COMMAND test_rom_calibration)

# ---- 关联器类体分片的装配检查 ----
# 【它只编不跑】把八个 Hm20Assoc_*.ipp 按真实顺序装进类里编一遍，约 0.5 秒。
# 改 .ipp 之后不用等整个工程（连 Qt 几分钟）就能知道有没有写坏。
# 手动跑：g++ -std=c++17 -I src -fsyntax-only tests/test_assoc_fragments.cpp
add_executable(test_assoc_fragments tests/test_assoc_fragments.cpp)
target_include_directories(test_assoc_fragments PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME assocfragments COMMAND test_assoc_fragments)

# ---- 相机显示名：DevicePath -> 稳定短标识 ----
# 【只测字符串那半边】queryUsbLocation() 要碰 Windows 配置管理器，非 Windows
# 下是返回空的 inline stub，所以这条测试在 Linux/CI 上也能跑，验的是变换规则
# 本身。样本是真机那六条 DevicePath，判据是"六台派生出六个不同的 tag" ——
# 撞了就说明规则白写，而这正是加这套东西要解决的问题。
add_executable(test_camera_label tests/test_camera_label.cpp src/camera/CameraLabel.cpp)
target_include_directories(test_camera_label PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_camera_label PRIVATE Qt6::Core)
if (WIN32)
    target_link_libraries(test_camera_label PRIVATE cfgmgr32)
endif()
add_test(NAME cameralabel COMMAND test_camera_label)

add_executable(test_constrained_circle_fit tests/test_constrained_circle_fit.cpp)
target_include_directories(test_constrained_circle_fit PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME constrainedcirclefit COMMAND test_constrained_circle_fit)

# 遮挡感知检测层 · 3b：单球半遮挡的圆弧拟合 + 弧长门控（ArcCircleFit.hpp），
# 建立在 3a 的 ConstrainedCircleFit.hpp 之上。同样零 Qt 依赖。
add_executable(test_arc_circle_fit tests/test_arc_circle_fit.cpp)
target_include_directories(test_arc_circle_fit PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME arccirclefit COMMAND test_arc_circle_fit)

# 遮挡感知检测层 · 3c：侧向花生双圆联合拟合 + 1球/2球模型选择（TwoCircleFit.hpp），
# 建立在 3a 的 ConstrainedCircleFit.hpp 之上。同样零 Qt 依赖。
add_executable(test_two_circle_fit tests/test_two_circle_fit.cpp)
target_include_directories(test_two_circle_fit PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME twocirclefit COMMAND test_two_circle_fit)

# 遮挡感知检测层 · 3d（收尾）：检测层输出契约（DetectionOutput.hpp），把
# 3a/3b/3c 三条拟合路径统一成 (mu, Sigma, confidence)，是 §3 对外的唯一
# 入口，IEKF 只需要 include 这一个头。同样零 Qt 依赖。至此遮挡感知检测层
# （3a->3b->3c->3d）闭环。
add_executable(test_detection_output tests/test_detection_output.cpp)
target_include_directories(test_detection_output PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME detectionoutput COMMAND test_detection_output)

# 状态估计层：IEKF（PointIEKF.hpp），接住 detect/DetectionOutput.hpp 的
# (mu, Sigma, confidence) 观测，跨相机、跨帧融合成 6 维(位置+速度)状态
# 估计。同样零 Qt 依赖。
add_executable(test_point_iekf tests/test_point_iekf.cpp)
target_include_directories(test_point_iekf PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME pointiekf COMMAND test_point_iekf)

# 状态估计层：手部位姿 IEKF（HandStateIEKF.hpp）。把 PointIEKF 的"每点独立
# 滤波"升级成"手腕刚体位姿 + 16维关节角"一体状态，FK 通过回调注入（不复制
# HandModel.hpp 的实现），marker 身份对应假设由调用方解决（不在本层范围）。
# 同样零 Qt 依赖。
add_executable(test_hand_state_iekf tests/test_hand_state_iekf.cpp)
target_include_directories(test_hand_state_iekf PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handstateiekf COMMAND test_hand_state_iekf)

# 状态估计层（收尾）：Marker 数据关联 + 一体化 pipeline（MarkerAssociator.hpp）。
# 填上 HandStateIEKF 留出的"哪个检测对应第几号 marker"缺口——用当前状态+FK
# 把 marker 投影到各相机，贪心门控最近邻认领这一帧的检测；HandTrackingPipeline
# 把"冷启动/用先验关联 -> 关联 -> IEKF更新"串成每帧闭环。冷启动(Kabsch模板
# 匹配)通过回调注入，不绑死实现。同样零 Qt 依赖。至此检测->关联->手部状态
# 估计的完整闭环打通。
add_executable(test_marker_associator tests/test_marker_associator.cpp)
target_include_directories(test_marker_associator PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME markerassociator COMMAND test_marker_associator)

# 三角化数学核心是纯逻辑（无 Qt 依赖），同 test_centroid 一样可以脱离 Qt
# GUI 单独跑；只链 Triangulation.cpp 这一个源文件，不需要 Triangulator.cpp
# （那个是 Qt 包装，依赖 ICamera/CalibrationStore，不在这个离线数学测试
# 的职责范围内）。
# 三角化的数学算法本身不调用任何 Qt 运行时功能，但它引用了 calib/Calibration.hpp
# 里的 CameraIntrinsics/CameraExtrinsics 结构体——那个头文件顶部为了给
# CameraCalibration 做 JSON 序列化，无条件 include 了 <QString>/<QJsonObject>，
# 所以即使这里一次都不调用 Qt 的函数，编译期也需要能找到这两个头文件，
# 必须链接 Qt6::Core 才能拿到 Qt 的 include 路径（这一点跟 test_centroid
# 不一样——CentroidDetector.hpp 是真正零 Qt 依赖，所以那个目标不需要这行）。
add_executable(test_triangulation tests/test_triangulation.cpp src/reconstruct/Triangulation.cpp)
target_include_directories(test_triangulation PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_triangulation PRIVATE Qt6::Core)
add_test(NAME triangulation COMMAND test_triangulation)

# 手部动捕 · 阶段一：FK 运动学模型 + 手背刚体位姿求解（src/hand/ 下）。
# 这两个测试连 Qt6::Core 都不需要——两个头文件完全没 include 任何 Qt 头，
# 比三角化测试还干净。include 目录指向 src（不是 src/hand），因为测试文件
# 里用的是 "hand/HandModel.hpp" 这种带目录前缀的写法，跟项目里
# "camera/ICamera.hpp" 的习惯保持一致。
# UDP 手部输出包(M3DS/M3DQ)的字节布局黄金测试。打包实现从 UdpSender.cpp 搬到
# net/HandPacket.hpp 之后，用"重构前那份手写代码"当参照物逐字节比对——动的是
# 已经有 Unity 端在用的线上格式，错一个字节下游全部错位。
# 【需要 Qt6::Gui】QVector3D 在 QtGui 里，不是 QtCore。
# 关节角输出手性：直接调用生产代码 solveJointAngles()，验证"物理左手 + 系统
# 右手档"下屈曲类各维反号、外展不变，以及 mirrorOut 打开后能还原成右手基准。
# 纯 std，不需要 Qt / onnxruntime。
# 同一结论的第二条独立路径：不 include 工程任何头文件，把提取式独立重写一遍
# 验算。单独看它不能证明生产代码的行为(那是上面那个的活)，但两条互不依赖的
# 路给出同一结论，比任何一条单独成立都更硬。零依赖，可单独 g++ 编。
# 量化证明"手背编号循环移位，Kabsch 残差查不出来"——4 种移位有 3 种能骗过
# Validating 的 RMSE 自检，而腕部姿态已经转掉 72~144°。零依赖。
add_executable(test_dorsum_phase tests/test_dorsum_phase.cpp)
add_test(NAME dorsumphase COMMAND test_dorsum_phase)

# 手背刚体求解器的蒙特卡洛评测。零依赖(不需要 Qt / onnxruntime)。
# 只看"错标"那一列——它是唯一会让腕部系整个翻 180°、全手连线一起错的错误类型。
# 布局A(近似正五边形，= 现状) vs 布局B(极角明显不均匀)，量的是贴点布局本身值多少。
add_executable(test_dorsum_rigid_mc tests/test_dorsum_rigid_mc.cpp)
target_include_directories(test_dorsum_rigid_mc PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME dorsumrigidmc COMMAND test_dorsum_rigid_mc)

add_executable(test_joint_mirror_sign tests/test_joint_mirror_sign.cpp)
add_test(NAME jointmirrorsign COMMAND test_joint_mirror_sign)

add_executable(test_joint_handedness tests/test_joint_handedness.cpp)
target_include_directories(test_joint_handedness PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME jointhandedness COMMAND test_joint_handedness)

add_executable(test_hand_packet tests/test_hand_packet.cpp)
target_include_directories(test_hand_packet PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_hand_packet PRIVATE Qt6::Core Qt6::Gui)
add_test(NAME handpacket COMMAND test_hand_packet)

add_executable(test_handmodel tests/test_hand_model.cpp)
target_include_directories(test_handmodel PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handmodel COMMAND test_handmodel)

add_executable(test_handpose tests/test_hand_pose.cpp)
target_include_directories(test_handpose PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handpose COMMAND test_handpose)

# 手背模板自动标定（HandCalibration.hpp）：从任意(非退化)真实marker贴法
# 自动推出手腕局部系模板，替代 HandModel.hpp 里写死的占位坐标。零 Qt 依赖。
add_executable(test_hand_calibration tests/test_hand_calibration.cpp)
target_include_directories(test_hand_calibration PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handcalibration COMMAND test_hand_calibration)

# 手指结构自标定(方案3：自然摆动，不额外贴marker)——HandSelfCalibration.hpp。
# 交替优化(逐帧IK <-> 跨帧结构) + 多起点全局重启防局部极小值。零 Qt 依赖。
add_executable(test_hand_self_calibration tests/test_hand_self_calibration.cpp)
target_include_directories(test_hand_self_calibration PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handselfcalibration COMMAND test_hand_self_calibration)

# HandColdStart 重写(运行时模板参数 + 诊断信息)后的回归测试。零 Qt 依赖。
add_executable(test_hand_cold_start tests/test_hand_cold_start.cpp)
target_include_directories(test_hand_cold_start PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handcoldstart COMMAND test_hand_cold_start)

# HandColdStart 退化匹配(5->4->3点优先级搜索)——手背marker被遮挡1~2颗时
# 也能定位，不再要求全部5点都识别到才算数。零 Qt 依赖。
add_executable(test_hand_cold_start_degraded tests/test_hand_cold_start_degraded.cpp)
target_include_directories(test_hand_cold_start_degraded PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handcoldstartdegraded COMMAND test_hand_cold_start_degraded)

# MarkerAssociator 自适应马氏距离门控——用IEKF自身的状态协方差(经投影
# 雅可比传播到2D)替代固定欧氏半径做关联门控，协方差大时自动放宽(冷启动
# 刚完成/快速运动后)、协方差小时自动收紧(稳定跟踪中)，保留固定欧氏半径
# 作为硬性兜底上限。零 Qt 依赖。
add_executable(test_marker_associator_mahalanobis tests/test_marker_associator_mahalanobis.cpp)
target_include_directories(test_marker_associator_mahalanobis PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME markerassociatormahalanobis COMMAND test_marker_associator_mahalanobis)

# 全自动标定链路(高级选项，HandCalibrationWizard的"③全自动标定"tab用) ——
# 之前这五个模块只有单元测试、从没接进CMakeLists，是真正意义上的死代码。
# 现在 AutoCalibPage 真正调用 runHandAutoCalibPipeline() 了，补上这几个
# 一直缺失的测试target。全部零 Qt 依赖。
add_executable(test_hand_rigid_autocalib tests/test_hand_rigid_autocalib.cpp)
target_include_directories(test_hand_rigid_autocalib PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handrigidautocalib COMMAND test_hand_rigid_autocalib)

add_executable(test_finger_chain_segmentation tests/test_finger_chain_segmentation.cpp)
target_include_directories(test_finger_chain_segmentation PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME fingerchainsegmentation COMMAND test_finger_chain_segmentation)

add_executable(test_finger_kinematic_calib tests/test_finger_kinematic_calib.cpp)
target_include_directories(test_finger_kinematic_calib PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME fingerkinematiccalib COMMAND test_finger_kinematic_calib)

add_executable(test_hand_axis_alignment tests/test_hand_axis_alignment.cpp)
target_include_directories(test_hand_axis_alignment PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handaxisalignment COMMAND test_hand_axis_alignment)

# 标定杆精度统计核心(WandPrecisionStats.hpp)——测试文件一直存在且能通过，
# 但从没注册成 target，等于白写。零 Qt 依赖。
add_executable(test_wand_precision_stats tests/test_wand_precision_stats.cpp)
target_include_directories(test_wand_precision_stats PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME wandprecisionstats COMMAND test_wand_precision_stats)

# HandModelAdapter.hpp——同上，测试写了但没注册。零 Qt 依赖。
add_executable(test_hand_model_adapter tests/test_hand_model_adapter.cpp)
target_include_directories(test_hand_model_adapter PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME handmodeladapter COMMAND test_hand_model_adapter)

# 静态姿势手指标定(calibrateFingerFromStaticPose)——闭式解+静态初值加速
# 动态精修的效果验证。零 Qt 依赖。
add_executable(test_static_finger_calib tests/test_static_finger_calib.cpp)
target_include_directories(test_static_finger_calib PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME staticfingercalib COMMAND test_static_finger_calib)

# 三角化LM精修 + IRLS鲁棒估计（TriangulationRefine.hpp）——DLT给的是代数
# 最优解，不是直接最小化真实重投影误差的解；这里在DLT解基础上用LM迭代
# 精修，同时用IRLS对坏视角(检测噪声大/局部遮挡)自动软性降权。已经接进
# MultiViewCluster.hpp::clusterMultiView(新增useLmRefine参数，默认开启)。
# 零 Qt 依赖。
add_executable(test_triangulation_refine tests/test_triangulation_refine.cpp)
target_include_directories(test_triangulation_refine PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME triangulationrefine COMMAND test_triangulation_refine)

# 固定延迟窗口平滑器(FixedLagSmoother.hpp)——完全独立于实时HandStateIEKF
# 的一条精修流，窗口内联合优化多帧、延迟约等于窗口帧数换取更高精度，
# 尤其对手指这种单帧可观测性弱的自由度收益明显。零 Qt 依赖。
# 【已接入】HandTrackingWorker 通过 SmootherWorker 在独立线程跑它，结果由
# handPoseSmoothedReady 信号发出。注意目前**没有任何消费者**在接这个信号
# (UI没显示、下游没转发)，开关打开纯属空转——要用它必须先接一个消费者。
add_executable(test_fixed_lag_smoother tests/test_fixed_lag_smoother.cpp)
target_include_directories(test_fixed_lag_smoother PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME fixedlagsmoother COMMAND test_fixed_lag_smoother)

# 深度反推半径（RadiusRefine.hpp）：HandTrackingWorker 两遍流程的数学核心——
# 第一遍radius-free质心匹配拿粗略深度，第二遍每个blob按自己的真实深度反推
# 期望像素半径，替代原来"固定标称工作距离"这个在拍摄距离跨度大时两头都
# 系统性偏差很大的假设。零 Qt 依赖，可独立编译验证。
add_executable(test_radius_refine tests/test_radius_refine.cpp)
target_include_directories(test_radius_refine PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME radiusrefine COMMAND test_radius_refine)

# 关联门控半径自动推算（GateRadiusEstimator.hpp）：从已标定的手部模板几何
# (伸直/半屈/攥拳三种代表性姿态下遍历全部marker两两距离)直接推算出跟这只
# 手匹配的门控半径，不需要额外采集数据。零 Qt 依赖。
add_executable(test_gate_radius_estimator tests/test_gate_radius_estimator.cpp)
target_include_directories(test_gate_radius_estimator PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME gateradiusestimator COMMAND test_gate_radius_estimator)

# 多点动捕 · 对应性求解第一层：极线几何（本质矩阵+Sampson距离）+ 两视图
# 匹配（极线剪枝+匈牙利全局最优指派）。跟手部模型那两个一样零 Qt 依赖，
# 不需要 Qt6::Core。这两个文件放在 src/reconstruct/（不是 src/hand/），
# 因为它们是"从2D观测重建3D"这条链上的数学，跟 Triangulation.hpp 是
# 同一个关注点，不是手部骨架运动学。
add_executable(test_epipolar tests/test_epipolar.cpp)
target_include_directories(test_epipolar PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME epipolar COMMAND test_epipolar)

add_executable(test_twoviewmatcher tests/test_twoview_matcher.cpp)
target_include_directories(test_twoviewmatcher PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME twoviewmatcher COMMAND test_twoviewmatcher)

# 多点动捕 · 对应性求解第二层：多视图仲裁（MultiViewCluster.hpp），建立在
# Epipolar.hpp + TwoViewMatcher.hpp 之上，用第三台（及以上）相机验证/纠正
# 两视图候选，剔除"幽灵"配对。同样零 Qt 依赖。
add_executable(test_multiview_cluster tests/test_multiview_cluster.cpp)
target_include_directories(test_multiview_cluster PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME multiviewcluster COMMAND test_multiview_cluster)

# 多点动捕 · 对应性求解第三层（收尾）：时间连续性（TemporalTracker.hpp）。
# 建立在 MultiViewCluster.hpp 之上：跨帧维持稳定 ID、恒速模型预测、遮挡后在
# 丢失帧数窗口内重新捡回同一个点。同样零 Qt 依赖。至此"多点对应性求解"
# 三层（极线两视图 -> 多视图仲裁 -> 时间连续性）闭环。
add_executable(test_temporal_tracker tests/test_temporal_tracker.cpp)
target_include_directories(test_temporal_tracker PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME temporaltracker COMMAND test_temporal_tracker)

# IekfPointTracker.hpp——TemporalTracker的可选卡尔曼追踪后端，直接吃每
# 相机2D观测(不是三角化完的3D点)做贝叶斯融合，见该文件头部完整说明。
# 跟test_temporal_tracker覆盖同一套场景(id稳定性/遮挡记忆/确认门槛)，
# 方便横向对比两个后端。
add_executable(test_iekf_point_tracker tests/test_iekf_point_tracker.cpp)
target_include_directories(test_iekf_point_tracker PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME iekfpointtracker COMMAND test_iekf_point_tracker)

# 骨骼关联(HandSkeletonAssociator.hpp)——手掌位姿复用HandColdStart(确定性
# 数学)，手指15点归属+遮挡点预测走AI(ONNX)。测试本身不需要真的加载onnx
# 模型/不需要onnxruntime——用一个返回固定/可控输出的 ISkeletonInferenceBackend
# 测试实现，只验证"手掌匹配->局部系转换->候选点归类->朝向计算"这条胶水逻辑
# 本身对不对，AI模型的分类/回归准确率是另一件事(靠 tools/skeleton_assoc/
# run_pipeline.py 训练时的 val acc/reg_err 衡量，不是这个单测的职责)。跟
# test_triangulation 一样，链路上有 CamPose，需要 Qt6::Core 才能找到头文件。
# 单测已随本次迁移一起重写到 hm20 接口(IHm20InferenceBackend / logAssign /
# markers / segQuat)，其中 scenario4/5/6 是三处修复的回归测试：近节中节四元数
# 不得恒等、boneFrame 不得在退化姿态突变、手背点<3 不得退化到全20点 Kabsch。
# 改回旧实现这三条会立刻变红。不需要 onnxruntime，用假后端直接给指派矩阵。
option(BUILD_SKELETON_ASSOC_TEST "构建骨架关联单元测试" ON)
if (BUILD_SKELETON_ASSOC_TEST)
    add_executable(test_hand_skeleton_associator tests/test_hand_skeleton_associator.cpp)
    target_include_directories(test_hand_skeleton_associator PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(test_hand_skeleton_associator PRIVATE Qt6::Core)

    # 【新增】在线自标定 + 时序滤波。不依赖 onnxruntime（用可控假后端），
    # 所以 USE_ONNXRUNTIME=OFF 也能跑。
    add_executable(test_hm20_autocalib_filter tests/test_hm20_autocalib_filter.cpp)
    target_include_directories(test_hm20_autocalib_filter PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(test_hm20_autocalib_filter PRIVATE Qt6::Core)
    add_test(NAME handskeletonassociator COMMAND test_hand_skeleton_associator)
    add_test(NAME hm20autocalibfilter COMMAND test_hm20_autocalib_filter)
endif()

# 求解脚本随构建拷到可执行文件旁，标定向导第5步直接调用。
add_custom_command(TARGET mocap_host POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/tools/solve_calibration.py
        $<TARGET_FILE_DIR:mocap_host>/solve_calibration.py)

# 骨骼关联模型同理拷到可执行文件旁——后端用
# QCoreApplication::applicationDirPath()+"/<模型名>.onnx" 这个固定相对位置
# 去加载，不用每次改动都手动复制。tools/skeleton_assoc/run_pipeline.py 训练
# 产出的文件就落在这个源码路径下，构建时自动同步一份到可执行文件旁；
# 模型文件不存在时会跳过并留一条 STATUS(不是ERROR)——还没训练过模型时项目
# 应该照常能编译，只是骨架功能不可用，跟没装onnxruntime时的降级路径一致。
# 【当前使用】hm20 模型(6输入/21类)——Hm20OnnxBackend 用
# QCoreApplication::applicationDirPath() 下按候选顺序自动挑一个加载
# （v7 > v6_sk10 > v6），所以下面把存在的都拷过去。
#
# 【重要——改模型文件名或首次放入模型后必须重跑 cmake configure】
# if(EXISTS) 是 configure 期求值的，不是 build 期。把 .onnx 拷进源码树之后
# 若不重新 configure，下面这条 POST_BUILD 规则根本不会生成，症状是"源码树里
# 明明有模型，运行时却报找不到"。旧模型踩过这个坑。
# 【拷贝所有存在的候选，而不是只拷一个】
# C++ 侧(PointCloudTestDialog.cpp)现在是【自动回退】选模型：
#     QSettings 指定 > hm20_v7.onnx > hm20_v6_sk10.onnx > hm20_v6.onnx
# 如果这里只拷 v7，而源码树里只有 v6，就会出现"CMake 说缺文件、程序其实
# 能跑(或反过来)"的错位。两边的候选列表必须一致。
#
# 【为什么保留旧模型也拷】v7 加了姿态/手性头，但那几个头缺失时后端会把
# has* 置 false、上层退回几何路径 —— 也就是说 v6 仍然完全可用。
# 想 A/B 对比或临时回退时，把 v7 改个名就行，不用重编。
set(_hm20_models hm20_v7.onnx hm20_v6_sk10.onnx hm20_v6.onnx)
set(_hm20_found "")
foreach(_m ${_hm20_models})
    if (EXISTS ${CMAKE_SOURCE_DIR}/tools/skeleton_assoc/${_m})
        add_custom_command(TARGET mocap_host POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/tools/skeleton_assoc/${_m}
                $<TARGET_FILE_DIR:mocap_host>/${_m})
        list(APPEND _hm20_found ${_m})
    endif()
endforeach()

# ---- INT8 量化模型（可选的"高速模式"）-------------------------------------
# 【为什么单独一段、不并进上面的候选列表】
# 上面那个 _hm20_models 是【运行时优先加载哪个】的优先级链。把 _int8 塞进去，
# 它就会被当成主模型自动加载 —— 而 INT8 是要用户主动勾选才启用的，
# 默认必须跑原模型。两件事不能混。
#
# C++ 侧的约定（PointCloudTestDialog.cpp）：主模型 xxx.onnx -> 高速版
# xxx_int8.onnx，放同一目录。找不到时"高速模式"勾选框会灰掉并改名为
# "高速模式(未找到INT8模型)"，不静默失败。
#
# 【不拷的后果】源码树里有、可执行文件旁边没有 —— 勾永远是灰的，
# 而且看不出为什么。这正是 onnx 文件当初踩过的那个坑。
set(_hm20_fast "")
foreach(_m ${_hm20_models})
    string(REPLACE ".onnx" "_int8.onnx" _mi ${_m})
    if (EXISTS ${CMAKE_SOURCE_DIR}/tools/skeleton_assoc/${_mi})
        add_custom_command(TARGET mocap_host POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_SOURCE_DIR}/tools/skeleton_assoc/${_mi}
                $<TARGET_FILE_DIR:mocap_host>/${_mi})
        list(APPEND _hm20_fast ${_mi})
    endif()
endforeach()
if (_hm20_fast)
    message(STATUS "骨骼关联模型(高速模式/INT8): 将复制 ${_hm20_fast}")
else()
    message(STATUS "骨骼关联模型(高速模式/INT8): 没有 —— 界面上的\"高速模式\"会灰掉。"
                   "用 tools/quantize_hm20.py 生成 xxx_int8.onnx 放进 "
                   "tools/skeleton_assoc/ 再【重跑 cmake configure】。")
endif()
if (_hm20_found)
    list(GET _hm20_found 0 _hm20_active)
    message(STATUS "骨骼关联模型: 将复制 ${_hm20_found}；运行时优先加载 ${_hm20_active}")
else()
    message(STATUS "骨骼关联模型 tools/skeleton_assoc/ 下一个都没有"
                   "（找过 ${_hm20_models}）——本次构建照常进行，"
                   "点云测试界面的骨架叠加功能暂不可用。"
                   "放入模型后【必须重跑 cmake configure】，否则拷贝规则不会生成。")
endif()

# 上一代的 skeleton_assoc.onnx(4输入/16类)与配套后端已经删除，不再复制。
# cmake/verify_skeleton_ai_files.cmake 里的 _required 也已同步改成 hm20 的文件名——
# 那两处必须一致，否则编译输出里会莫名其妙报"缺失"一个根本不用的文件。

# ---------------------------------------------------------------------------
# 骨架AI 文件自检 —— 【必须是本文件里最后一条 POST_BUILD】
# ---------------------------------------------------------------------------
# 同一 target 的 POST_BUILD 命令按注册顺序执行。这条自检要看的正是前面那几条
# 命令(onnxruntime_mocap.dll、各依赖DLL、hm20_v7.onnx)的产物，所以只能排
# 在它们【之后】注册。原先它写在上面 onnxruntime 那一段里(模型复制之前)，导致
# 首次构建时它检查的是上一次构建留下的目录 —— 模型明明这次会被复制，自检却先
# 一步报"[缺失]"，白白误导人查半天。
#
# 往这个文件里加新的 POST_BUILD 命令时，加在这一段【上面】。
if (_verify_skeleton_ai)
    add_custom_command(TARGET mocap_host POST_BUILD
        COMMAND ${CMAKE_COMMAND}
            -DTARGET_DIR=$<TARGET_FILE_DIR:mocap_host>
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/verify_skeleton_ai_files.cmake"
        COMMENT "自检：骨架AI所需文件是否齐备")
endif()

