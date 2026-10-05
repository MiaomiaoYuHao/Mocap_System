# 骨架AI运行所需文件的编译后自检。
#
# 由 CMakeLists.txt 里的 add_custom_command(TARGET mocap_host POST_BUILD ...)
# 以脚本模式(cmake -P)调用，直接把结果打印在【编译输出】里——这样不需要
# 手动进 build 目录一个个翻文件确认到底复制成功没有。
#
# 入参：TARGET_DIR = 可执行文件所在目录(由生成表达式 $<TARGET_FILE_DIR:...> 传入)

if (NOT DEFINED TARGET_DIR)
    message(FATAL_ERROR "verify_skeleton_ai_files.cmake 需要 -DTARGET_DIR=<exe目录>")
endif()

# 必需项：缺任何一个，骨架AI都跑不起来
#
# 【模型改成"候选里至少有一个"，不再死盯某个文件名】
# C++ 侧(PointCloudTestDialog.cpp)是自动回退选模型：
#     QSettings 指定 > hm20_v7.onnx > hm20_v6_sk10.onnx > hm20_v6.onnx
# 以前这里写死一个名字，结果是：换模型要同时改三处(这里、CMakeLists、
# 那个 cpp)，漏改一处就会在编译输出里报"[缺失] xxx.onnx"虚惊一场——
# 上一代 skeleton_assoc.onnx 就是这么白查了半天。
# 现在只判"这几个里有没有"，跟运行时的实际行为一致。
set(_required
    onnxruntime_mocap.dll      # onnxruntime运行库(改名版，避开Windows Known DLLs重定向)
)
set(_model_candidates
    hm20_v7.onnx               # 姿态版：6输入/10输出(多了姿态·手性·置信度·分段朝向)
    hm20_v6_sk10.onnx          # 上一版：6输入/6输出，仍然可用(新头缺失时退回几何路径)
    hm20_v6.onnx
)
# onnxruntime 的传递依赖：不同 vcpkg 版本需要的不完全一样，缺了不一定是错，
# 但如果运行时报"缺少依赖"，对照这里就能立刻知道是哪个没到位。
set(_deps
    abseil_dll.dll
    libprotobuf.dll
    libprotobuf-lite.dll
    re2.dll
    onnxruntime_providers_shared.dll
)

set(_missing_required "")

message(STATUS "---------- 骨架AI 文件自检 (${TARGET_DIR}) ----------")
foreach(_f ${_required})
    if (EXISTS "${TARGET_DIR}/${_f}")
        file(SIZE "${TARGET_DIR}/${_f}" _sz)
        math(EXPR _szkb "${_sz} / 1024")
        message(STATUS "  [OK]   ${_f}  (${_szkb} KB)")
    else()
        message(STATUS "  [缺失] ${_f}   <-- 骨架AI将不可用")
        list(APPEND _missing_required "${_f}")
    endif()
endforeach()

# 模型：候选里有一个就算过，并指出运行时实际会加载哪个
set(_model_found "")
foreach(_m ${_model_candidates})
    if (EXISTS "${TARGET_DIR}/${_m}")
        file(SIZE "${TARGET_DIR}/${_m}" _sz)
        math(EXPR _szkb "${_sz} / 1024")
        if (NOT _model_found)
            message(STATUS "  [OK]   ${_m}  (${_szkb} KB)  <-- 运行时加载这个")
            set(_model_found ${_m})
        else()
            message(STATUS "  [有]   ${_m}  (${_szkb} KB)  (备用，优先级更低)")
        endif()
    endif()
endforeach()
if (NOT _model_found)
    message(STATUS "  [缺失] 模型文件   <-- 骨架AI将不可用")
    message(STATUS "         找过: ${_model_candidates}")
    list(APPEND _missing_required "hm20 模型(${_model_candidates} 之一)")
endif()

# INT8 量化模型：【可选】，只影响界面上的"高速模式"能不能勾。
#
# 【为什么不放进 _model_candidates】那个列表是"运行时优先加载哪个"的优先级链，
# 把 INT8 放进去会让它被当成主模型自动加载 —— 而它必须是用户主动勾选才启用。
#
# 【为什么仍然要在这里报一行】不报的话它就完全不可见：源码树里有、
# exe 旁边没有的时候，界面上那个勾是灰的，而你从编译输出看不出任何线索。
# 报一行就能立刻知道是"没生成"还是"没拷过来"。
if (_model_found)
    string(REPLACE ".onnx" "_int8.onnx" _int8_name ${_model_found})
    if (EXISTS "${TARGET_DIR}/${_int8_name}")
        file(SIZE "${TARGET_DIR}/${_int8_name}" _sz)
        math(EXPR _szkb "${_sz} / 1024")
        message(STATUS "  [OK]   ${_int8_name}  (${_szkb} KB)  <-- 高速模式可用")
    else()
        message(STATUS "  [无]   ${_int8_name}  (可选：高速模式会灰掉；"
                       "用 tools/quantize_hm20.py 生成后重跑 cmake configure)")
    endif()
endif()

foreach(_f ${_deps})
    if (EXISTS "${TARGET_DIR}/${_f}")
        message(STATUS "  [OK]   ${_f}")
    else()
        message(STATUS "  [无]   ${_f}  (本版本可能不需要；若运行时报缺此依赖再处理)")
    endif()
endforeach()

if (_missing_required)
    message(WARNING
        "骨架AI必需文件缺失: ${_missing_required}。"
        "程序仍可正常编译运行，但点云窗口里勾选\"叠加显示骨架\"不会有效果。"
        "onnxruntime_mocap.dll 应由构建自动从 vcpkg 复制；"
        "hm20_v7.onnx 需要先在 tools/skeleton_assoc/ 下训练/导出出来，"
        "且【模型放进源码树后要重新执行一次 cmake 配置】"
        "(复制命令是配置阶段按 if(EXISTS) 决定要不要生成的，build 期不重新判断)。")
else()
    message(STATUS "  => 骨架AI必需文件齐备")
endif()
message(STATUS "------------------------------------------------------")
