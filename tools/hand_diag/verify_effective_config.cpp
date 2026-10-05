// verify_effective_config.cpp —— 把【实际生效的】配置值打出来
//
//   g++ -std=c++20 -O2 -I src tools/hm20_diag/verify_effective_config.cpp -o vc && ./vc
//
// 【为什么需要这个】改完配置默认值之后，"改了没生效"有三种可能：
//   ① 编辑脚本中途失败，改动根本没落盘
//   ② 落盘了，但有别的地方 setConfig 覆盖回去
//   ③ 落盘也没被覆盖，但那个字段没人引用
// 读源码只能排除①。这个程序直接构造求解器、把 config() 打出来，②③一起排除。
//
// 期望输出（当前设定，按 25fps 换算）：
//   relockAfterBad          6   0.24s
//   revertAfterBad         25   1.00s
//   relockAfterImplausible  6   0.24s
//   repairMinFrames        18   0.72s
//   refineEnabled        true
//   refineAlpha         0.050   0.80s

#include "estimate/HandSkeletonAssociator.hpp"
#include <cstdio>
using namespace mocap::hm20;
struct FB : IHm20InferenceBackend { InferenceOutput run(const InferenceInput&) override { return {}; } };
int main(){
    // 完全按关联器实际用的方式构造
    DorsumRigidSolver S;
    const auto c = S.config();
    const double FPS = 25.0;
    std::printf("%-28s%8s%12s\n","配置项","值","换算成秒");
    std::printf("%-28s%8d%10.2fs\n","relockAfterBad(松锁)",c.relockAfterBad,c.relockAfterBad/FPS);
    std::printf("%-28s%8d%10.2fs\n","revertAfterBad(退回模板)",c.revertAfterBad,c.revertAfterBad/FPS);
    std::printf("%-28s%8d%10.2fs\n","relockAfterImplausible",c.relockAfterImplausible,c.relockAfterImplausible/FPS);
    std::printf("%-28s%8d%10.2fs\n","repairMinFrames(重捕观察)",c.repairMinFrames,c.repairMinFrames/FPS);
    std::printf("%-28s%8s%10s\n","refineEnabled(持续微调)",c.refineEnabled?"true":"false","-");
    std::printf("%-28s%8.3f%10.2fs\n","refineAlpha",c.refineAlpha,1.0/(c.refineAlpha*FPS));
    std::printf("%-28s%8.1f%10s\n","gateWithHistoryMm",c.gateWithHistoryMm,"-");
    std::printf("%-28s%8d%10s\n","maxSeeds",c.maxSeeds,"-");
    std::printf("\n=== 关联器有没有覆盖这些默认值 ===\n");
    auto be=std::make_shared<FB>();
    Hm20SkeletonAssociator a(be,{},false);
    std::array<Vec3,kNumMarkers> mm{};
    double rad[5]={28.2,35.2,21.8,23.7,34.4},ang[5]={0,74,145,212,289};
    for(int i=0;i<5;++i){double t=ang[i]*3.14159265358979/180.0;
        mm[size_t(i)]={rad[i]*std::cos(t),rad[i]*std::sin(t),0.0};}
    a.setTemplateMm(mm);
    std::printf("  setTemplateMm 之后 selfAmbiguity=%.3f (说明模板确实喂进去了)\n",
        a.dorsumSelfAmbiguityMm());
    std::printf("\n=== Hm20Config 里的相关开关 ===\n");
    Hm20Config h;
    std::printf("  chainContinue     = %s\n", h.chainContinue?"true":"false");
    std::printf("  dorsumRigidSolve  = %s\n", h.dorsumRigidSolve?"true":"false");
    std::printf("  dorsumTracklet    = %s\n", h.dorsumTracklet?"true":"false");
    return 0;
}
