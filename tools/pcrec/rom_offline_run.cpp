#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include "estimate/RomCalibration.hpp"
using namespace mocap::hm20::rom;

static std::vector<FrameObs> rd(const char* p) {
    FILE* f = fopen(p, "rb");
    if (!f) return {};
    char magic[4] = {};
    int32_t n = 0;
    if (fread(magic, 1, 4, f) != 4 || fread(&n, 4, 1, f) != 1) { fclose(f); return {}; }
    if (magic[0] != 'O' || magic[1] != 'B' || magic[2] != 'S' || magic[3] != '2') {
        fprintf(stderr, "unsupported obs format: %s\n", p);
        fclose(f); return {};
    }
    std::vector<FrameObs> v;
    v.resize(size_t(n));
    for (int i = 0; i < n; ++i) {
        FrameObs& o = v[size_t(i)];
        if (fread(o.q.data(), 8, 16, f) != 16 ||
            fread(o.state.data(), 1, 16, f) != 16 ||
            fread(o.curl.data(), 8, 5, f) != 5 ||
            fread(o.curlValid.data(), 1, 5, f) != 5) { v.resize(size_t(i)); break; }
        // 文件里 signRef 是【整块候选先写完，再写所有 valid】。
        for (int c = 0; c < kSignRefs; ++c) {
            if (fread(o.signRef[size_t(c)].data(), 8, 16, f) != 16) {
                v.resize(size_t(i)); fclose(f); return v;
            }
        }
        for (int c = 0; c < kSignRefs; ++c) {
            if (fread(o.signRefValid[size_t(c)].data(), 1, 16, f) != 16) {
                v.resize(size_t(i)); fclose(f); return v;
            }
        }
        if (fread(&o.frameTsNs, 8, 1, f) != 1) { v.resize(size_t(i)); break; }
    }
    fclose(f);
    return v;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <mode> <obs> [obs...]\n", argv[0]);
        return 2;
    }
    Config c;
    Calibrator cal;
    if (std::string(argv[1]) == "nogate") c.acceptDegenerate = true;
    cal.configure(c);
    for (int i = 2; i < argc; ++i) {
        cal.beginPass(i > 2);           // 第二段起走【补标】
        for (auto& o : rd(argv[i])) cal.observe(o);
        const auto& r = cal.finish();
        printf("  第%d段之后: 覆盖 %.0f%%  达标 %d/11  方向纠正 %d  ready=%s\n",
               i - 1, r.coverageFlex * 100, r.nOkFlex, r.nSignFlipped, r.ready ? "是" : "否");
    }
    printf("\n%s", cal.humanReport().c_str());
    printf("\nSIGNS");
    for (int i = 0; i < kDof; ++i) printf(" %d", cal.sign(i));
    printf("\nLO");
    for (int i = 0; i < kDof; ++i) printf(" %.9g", cal.lo(i));
    printf("\nHI");
    for (int i = 0; i < kDof; ++i) printf(" %.9g", cal.hi(i));
    printf("\nSTATUS");
    for (int i = 0; i < kDof; ++i) printf(" %d", int(cal.status(i)));
    printf("\n");
    return 0;
}
