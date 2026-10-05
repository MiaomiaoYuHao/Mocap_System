// ===========================================================================
// Hm20Assoc_types.ipp —— Hm20SkeletonAssociator 类体分片：类型与配置别名
//
// 嵌套类型必须最先出现——它们被后面的函数当返回类型用，
// 而类内名字要先声明后使用。改这里等于改公开契约，动之前先看调用方。
//
// 【本文件被 #include 在 class 体内部】内容即类内定义（隐式 inline）。
// 不要加 include guard、不要加 namespace。开头的 access 说明符是显式写死的，
// 不依赖上一个分片以什么结尾——调换 include 顺序不会改变可见性。
// 独立语法检查：见 tests/test_assoc_fragments.cpp
// ===========================================================================
public:
    using Config = Hm20Config;

    struct NetSegCalibReport {
        bool ok = false;
        double cSpreadDeg = -1.0;
        std::array<double, 3> kSpreadDeg{{-1, -1, -1}};
        std::array<double, 3> kOffsetDeg{{-1, -1, -1}};   // K 相对单位阵的转角
        int cSamples = 0, kSamples = 0;
        const char* why = "还没跑";
    };
    // 让离线程序能直接驱动 smoothPredicted —— 【必须验真正跑的那份代码】，
    // 而不是在测试里复刻一遍逻辑。复刻版和真版走偏过一次（第一版顺序放错），
    // 那种情况下测试全绿而真机没变。
    struct TestHook_ContState {
        std::array<Vec3, kNumMarkers> s{}, off{};
        std::array<bool, kNumMarkers> has{}, seen0{};
    };
