// ===========================================================================
// pcrec_replay.cpp —— 离线复算 / 参数扫描
//
//   g++ -std=c++20 -O2 -I src tools/hm20_diag/pcrec_replay.cpp -o replay
//   ./replay capture.pcrec                          # 对拍：离线 vs 在线
//   ./replay capture.pcrec --set thumbPronationRad=0
//   ./replay capture.pcrec --sweep dorsumGeoMinMarginMm=0.5,1,2,4
//
// 【核心设计：不需要 onnxruntime，也不需要 .onnx 文件】
// 因为 .pcrec 里录了 NetRaw（网络的原始输出），复算时把它当作"后端"回放即可。
// 这带来三个好处，每一个都直接影响调参效率：
//   ① 快 —— 没有推理，一秒能跑几千帧，参数扫描才有可能做
//   ② 准 —— 网络输出被【钉死】，观察到的差异 100% 来自后处理参数，
//           不会混进推理的非确定性（不同 provider/驱动结果会有微小差异）
//   ③ 轻 —— 单文件，任何有 g++ 的机器都能跑，不用配 ORT
//
// 【先对拍，再扫参】不改任何参数先跑一遍，确认离线结果跟在线【逐帧一致】。
// 对不上就先修对拍，别急着扫参 —— 基线错了，扫出来的最优值也是错的。
// 这就是为什么必须录 AssocInput：网络的指派对输入顺序敏感，而 Points3D 记的
// 是追踪器输出，中间还隔着筛选和 maxCandidates 截断，顺序对不上就永远对不拍。
// ===========================================================================

#include "estimate/HandSkeletonAssociator.hpp"
#include "record/PointCloudRecorder.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace mocap;
using namespace mocap::hm20;

// ---------------------------------------------------------------------------
// 极简 .pcrec 读取。只取复算需要的四种块，其余按长度跳过。
// ---------------------------------------------------------------------------
struct Frame {
    int64_t ts = 0;
    std::vector<std::pair<int, Vec3>> cand;
    int candBeforeCap = 0;
    pcrec::NetRawRec net{};
    std::vector<pcrec::NetTopK> topk;
    pcrec::SkeletonRec skel{};
    pcrec::Hm20DiagRec diag{};
    bool hasNet = false, hasSkel = false, hasDiag = false, hasCand = false;
};

struct Rec {
    std::string header;
    std::vector<Frame> frames;
};

static bool loadRec(const char* path, Rec& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { std::fprintf(stderr, "打不开 %s\n", path); return false; }
    std::vector<char> raw((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    if (raw.size() < 16 || std::memcmp(raw.data(), "MCPCREC\0", 8) != 0) {
        std::fprintf(stderr, "不是 .pcrec 文件\n"); return false;
    }
    uint32_t ver = 0, hlen = 0;
    std::memcpy(&ver, raw.data() + 8, 4);
    std::memcpy(&hlen, raw.data() + 12, 4);
    out.header.assign(raw.data() + 16, hlen);
    size_t off = 16 + hlen;

    // 按时间戳归帧：同一 tsNs 的 AssocInput / NetRaw / Skeleton 属于同一帧
    std::map<int64_t, Frame> byTs;
    while (off + 5 <= raw.size()) {
        const uint8_t ct = uint8_t(raw[off]);
        uint32_t clen = 0;
        std::memcpy(&clen, raw.data() + off + 1, 4);
        const size_t body = off + 5;
        if (body + clen > raw.size()) break;          // 截断：读到最后一个完整块
        const char* p = raw.data() + body;
        off = body + clen;

        if (ct == uint8_t(pcrec::ChunkType::AssocInput)) {
            pcrec::AssocInputRec h{};
            std::memcpy(&h, p, sizeof(h));
            Frame& fr = byTs[h.tsNs];
            fr.ts = h.tsNs; fr.candBeforeCap = h.nBeforeCap; fr.hasCand = true;
            const char* q = p + sizeof(h);
            for (int i = 0; i < int(h.n); ++i) {
                float xyz[3]; int32_t id;
                std::memcpy(xyz, q, 12); std::memcpy(&id, q + 12, 4); q += 16;
                fr.cand.push_back({int(id), Vec3{double(xyz[0]), double(xyz[1]), double(xyz[2])}});
            }
        } else if (ct == uint8_t(pcrec::ChunkType::NetRaw)) {
            pcrec::NetRawRec h{};
            std::memcpy(&h, p, sizeof(h));
            Frame& fr = byTs[h.tsNs];
            fr.ts = h.tsNs; fr.net = h; fr.hasNet = true;
            fr.topk.resize(size_t(h.nCand));
            if (h.nCand)
                std::memcpy(fr.topk.data(), p + sizeof(h),
                            size_t(h.nCand) * sizeof(pcrec::NetTopK));
        } else if (ct == uint8_t(pcrec::ChunkType::Skeleton)) {
            int64_t ts; std::memcpy(&ts, p, 8);
            Frame& fr = byTs[ts];
            fr.ts = ts; fr.hasSkel = true;
            std::memcpy(&fr.skel, p + 8, sizeof(pcrec::SkeletonRec));
        } else if (ct == uint8_t(pcrec::ChunkType::Hm20Diag)) {
            pcrec::Hm20DiagRec h{};
            std::memcpy(&h, p, sizeof(h));
            Frame& fr = byTs[h.frameTsNs];
            fr.ts = h.frameTsNs; fr.diag = h; fr.hasDiag = true;
        }
    }
    for (auto& kv : byTs) out.frames.push_back(std::move(kv.second));
    std::printf("读入 %s  v%u  帧 %zu\n", path, ver, out.frames.size());
    return true;
}

// ---------------------------------------------------------------------------
// 回放后端：把录下来的网络输出原样吐回去。
// 【一个字都不加工】——一旦在这里"顺手补一下"，复算的就不再是当时那条链路。
// ---------------------------------------------------------------------------
class ReplayBackend : public IHm20InferenceBackend {
public:
    const Frame* fr = nullptr;
    InferenceOutput run(const InferenceInput&) override {
        InferenceOutput o;
        if (!fr || !fr->hasNet) return o;
        const int n = int(fr->net.nCand);
        o.rows = n + 1;
        o.cols = kNumClasses;
        // top-3 之外的类别给一个很小的 log 概率：关联器只看 argmax 和阈值，
        // 补全整张表没有意义，但必须保证 argmax 落在 top1 上。
        o.logAssign.assign(size_t(o.rows) * size_t(o.cols), -30.0f);
        for (int i = 0; i < n && i < int(fr->topk.size()); ++i)
            for (int k = 0; k < 3; ++k) {
                const int lab = fr->topk[size_t(i)].label[k];
                if (lab < 0 || lab >= kNumClasses) continue;
                const float pr = fr->topk[size_t(i)].prob[k];
                o.logAssign[size_t(i) * size_t(o.cols) + size_t(lab)] =
                    (pr > 1e-9f) ? std::log(pr) : -30.0f;
            }
        for (int m = 0; m < 20; ++m)
            o.pos[size_t(m)] = Vec3{double(fr->net.pos[m * 3 + 0]),
                                    double(fr->net.pos[m * 3 + 1]),
                                    double(fr->net.pos[m * 3 + 2])};
        o.center = Vec3{double(fr->net.center[0]), double(fr->net.center[1]),
                        double(fr->net.center[2])};
        o.scale = double(fr->net.scale);
        o.hasCenterScale = true;
        for (int m = 0; m < 20; ++m) {
            o.missLogit[size_t(m)] = double(fr->net.missLogit[m]);
            o.jointAng[size_t(m)]  = double(fr->net.jointAng[m]);
        }
        o.hasJointAng = true;
        o.handLogit = double(fr->net.handLogit);
        o.hasHandLogit = true;
        for (int i = 0; i < 5; ++i) o.poseConf[size_t(i)] = double(fr->net.poseConf[i]);
        o.hasPoseConf = fr->net.hasPose != 0;
        if (fr->net.hasSegR) {
            for (int g = 0; g < 16; ++g) {
                Mat3 R{};
                for (int c = 0; c < 6; ++c) {
                    const int r0 = c % 3, c0 = c / 3;
                    R[size_t(r0 * 3 + c0)] = double(fr->net.segRot6d[g * 6 + c]);
                }
                // 第三列 = 前两列叉积，保证是真旋转
                R[2] = R[3] * R[7] - R[6] * R[4];
                R[5] = R[6] * R[1] - R[0] * R[7];
                R[8] = R[0] * R[4] - R[3] * R[1];
                o.segR[size_t(g)] = R;
            }
            o.hasSegR = true;
        }
        o.ok = true;
        return o;
    }
};

// ---------------------------------------------------------------------------
static bool applyOverride(Hm20Config& c, const std::string& k, double v) {
    if (k == "thumbPronationRad")     { c.thumbPronationRad = v; return true; }
    if (k == "thumbRollOffsetRad")    { c.thumbRollOffsetRad = v; return true; }
    if (k == "dorsumGeoMinMarginMm")  { c.dorsumGeoMinMarginMm = v; return true; }
    if (k == "dorsumRigidSolve")      { c.dorsumRigidSolve = v != 0; return true; }
    if (k == "dorsumGeoRelabel")      { c.dorsumGeoRelabel = v != 0; return true; }
    if (k == "dorsumTracklet")        { c.dorsumTracklet = v != 0; return true; }
    if (k == "minAssignProbDorsum")   { c.minAssignProbDorsum = v; return true; }
    if (k == "minAssignProbFinger")   { c.minAssignProbFinger = v; return true; }
    if (k == "ikMaxRmseMm")           { c.ikMaxRmseMm = v; return true; }
    if (k == "midDirMode")            { c.midDirMode = int(v); return true; }
    if (k == "chainContinue")         { c.chainContinue = v != 0; return true; }
    if (k == "occludedIkOnly")        { c.occludedIkOnly = v != 0; return true; }
    if (k == "thumbSegFromNet")       { c.thumbSegFromNet = v != 0; return true; }
    if (k == "minAssignProbFinger")   { c.minAssignProbFinger = v; return true; }
    if (k == "minAssignProbDorsum")   { c.minAssignProbDorsum = v; return true; }
    if (k == "useTemplate")           { c.useTemplate = v != 0; return true; }
    if (k == "usePrevFrame")          { c.usePrevFrame = v != 0; return true; }
    if (k == "ikOverrideQuat")        { c.ikOverrideQuat = v != 0; return true; }
    if (k == "ikFillOccluded")        { c.ikFillOccluded = v != 0; return true; }
    if (k == "occludedJumpGateMm")    { c.occludedJumpGateMm = v; return true; }
    if (k == "chainRoll")             { c.chainRoll = v != 0; return true; }
    if (k == "geoHandMinBendDeg")     { c.geoHandMinBendDeg = v; return true; }
    if (k == "geoHandLockThresh")     { c.geoHandLockThresh = v; return true; }
    if (k == "occStaleRad0")          { c.occStaleRad0 = v; return true; }
    if (k == "occStaleRad1")          { c.occStaleRad1 = v; return true; }
    if (k == "caseBMaxFrames")        { c.caseBMaxFrames = int(v); return true; }
    if (k == "planeLearnMinDeg")      { c.planeLearnMinDeg = v; return true; }
    if (k == "occCouple")             { c.occCouple = v != 0; return true; }
    if (k == "occCoupleGain")         { c.occCoupleGain = v; return true; }
    if (k == "occCoupleMaxRad")       { c.occCoupleMaxRad = v; return true; }
    if (k == "handVoteRel")           { c.handVoteRel = v; return true; }
    if (k == "handConfFloor")         { c.handConfFloor = v; return true; }
    if (k == "predictSmoothAlpha")    { c.predictSmoothAlpha = v; return true; }
    if (k == "recoverBlendDecay")     { c.recoverBlendDecay = v; return true; }
    if (k == "recoverMaxOffsetMm")    { c.recoverMaxOffsetMm = v; return true; }
    if (k == "dorsumTrackGateMm")     { c.dorsumTrackGateMm = v; return true; }
    if (k == "dorsumTrackWarm")       { c.dorsumTrackWarm = int(v); return true; }
    if (k == "occUseNetDir")          { c.occUseNetDir = v != 0; return true; }
    if (k == "occNetDirAlpha")        { c.occNetDirAlpha = v; return true; }
    return false;
}

struct Score {
    int frames = 0, matchLabels = 0, totalLabels = 0;
    int dorsumOk = 0, wristOk = 0;
    double posErrSum = 0; int posErrN = 0;
    int reasonMatch = 0;
    double pairMed = 0, pairP90 = 0, pairN = 0;
    double ghostMean = 0;
};

static Score runOnce(Rec& rec, const std::array<Vec3, kNumMarkers>& tmplMm,
                     const Hm20Config& cfg, bool isRight) {
    auto be = std::make_shared<ReplayBackend>();
    // 归一化模板（61 维）从毫米模板现算：质心归零、平均半径归一，
    // 跟 packNormalized 同一套约定。tmpl[60] 是手性位。
    std::array<float, 61> t61{};
    {
        Vec3 c{{0,0,0}};
        for (int m = 0; m < kNumMarkers; ++m)
            for (int d = 0; d < 3; ++d) c[size_t(d)] += tmplMm[size_t(m)][size_t(d)] / kNumMarkers;
        double sc = 0;
        for (int m = 0; m < kNumMarkers; ++m) {
            double e = 0;
            for (int d = 0; d < 3; ++d) {
                const double q = tmplMm[size_t(m)][size_t(d)] - c[size_t(d)];
                e += q * q;
            }
            sc += std::sqrt(e) / kNumMarkers;
        }
        if (sc < 1e-9) sc = 1.0;
        for (int m = 0; m < kNumMarkers; ++m)
            for (int d = 0; d < 3; ++d)
                t61[size_t(m * 3 + d)] =
                    float((tmplMm[size_t(m)][size_t(d)] - c[size_t(d)]) / sc);
        t61[60] = isRight ? 1.0f : 0.0f;
    }
    Hm20SkeletonAssociator assoc(be, t61, true, cfg);
    assoc.setTemplateMm(tmplMm);
    Score s;
    std::array<std::vector<Vec3>, kNumMarkers> mp;   // 每 marker 被观测到的位置
    for (Frame& f : rec.frames) {
        if (!f.hasCand || !f.hasNet) continue;
        be->fr = &f;
        const SkeletonFrameResult r = assoc.process(f.cand);
        ++s.frames;
        if (f.hasSkel) {
            if (r.wristPoseValid == (f.skel.wristPoseValid != 0)) ++s.wristOk;
            for (int m = 0; m < 20; ++m) {
                ++s.totalLabels;
                if (r.markers[size_t(m)].observed == (f.skel.observed[m] != 0))
                    ++s.matchLabels;
                if (f.skel.observed[m] && r.markers[size_t(m)].observed) {
                    double e = 0;
                    for (int c = 0; c < 3; ++c) {
                        const double d = r.markers[size_t(m)].posWorld[size_t(c)]
                                       - double(f.skel.pos[m * 3 + c]);
                        e += d * d;
                    }
                    s.posErrSum += std::sqrt(e); ++s.posErrN;
                }
            }
        }
        if (f.hasDiag && r.dorsumSolveReason == int(f.diag.dorsumReason)) ++s.reasonMatch;
        if (r.dorsumSolveReason == 0) ++s.dorsumOk;
        s.ghostMean += double(r.numGhost);
        for (int m = 0; m < 20; ++m)
            if (r.markers[size_t(m)].observed) mp[size_t(m)].push_back(r.markers[size_t(m)].posWorld);
    }
    if (s.frames) s.ghostMean /= double(s.frames);
    // 刚体不变量: 两两 marker 距离的跨帧标准差
    std::vector<double> stds;
    for (int a = 0; a < 20; ++a) for (int b = a+1; b < 20; ++b) {
        size_t n = std::min(mp[size_t(a)].size(), mp[size_t(b)].size());
        if (n < 30) continue;
        std::vector<double> d(n);
        for (size_t q = 0; q < n; ++q) {
            double dx=mp[size_t(a)][q][0]-mp[size_t(b)][q][0];
            double dy=mp[size_t(a)][q][1]-mp[size_t(b)][q][1];
            double dz=mp[size_t(a)][q][2]-mp[size_t(b)][q][2];
            d[q]=std::sqrt(dx*dx+dy*dy+dz*dz);
        }
        double mean=0; for(double v:d) mean+=v; mean/=double(n);
        double var=0; for(double v:d){double e=v-mean; var+=e*e;} var/=double(n);
        stds.push_back(std::sqrt(var));
    }
    if (!stds.empty()) {
        std::sort(stds.begin(), stds.end());
        s.pairMed = stds[stds.size()/2];
        s.pairP90 = stds[std::min<size_t>(stds.size()-1, stds.size()*90/100)];
        s.pairN = double(stds.size());
    }
    return s;
}

static void printScore(const char* tag, const Score& s) {
    std::printf("%-34s 帧%4d  标签一致 %6.2f%%  位置差中位 %6.3fmm  手背解出 %5.1f%%  判据一致 %5.1f%%  刚体Med %5.2fmm P90 %5.2fmm(n=%3.0f)  鬼点 %.2f\n",
        tag, s.frames,
        s.totalLabels ? 100.0 * s.matchLabels / s.totalLabels : 0.0,
        s.posErrN ? s.posErrSum / s.posErrN : 0.0,
        s.frames ? 100.0 * s.dorsumOk / s.frames : 0.0,
        s.frames ? 100.0 * s.reasonMatch / s.frames : 0.0,
        s.pairMed, s.pairP90, s.pairN, s.ghostMean);
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 2) {
        std::puts("用法: replay <file.pcrec> [--set k=v ...] [--sweep k=v1,v2,...]");
        return 1;
    }
    Rec rec;
    if (!loadRec(argv[1], rec)) return 1;

    int usable = 0;
    for (const Frame& f : rec.frames) if (f.hasCand && f.hasNet) ++usable;
    std::printf("可复算帧 %d / %zu\n", usable, rec.frames.size());
    if (usable == 0) {
        std::puts("\n【这个文件没法复算】缺 AssocInput 和/或 NetRaw 块。");
        std::puts("这两样是复算的最小闭包：没有'送进网络的候选点及顺序'，指派结果");
        std::puts("对不上在线；没有网络原始输出，就分不开'模型判错'和'后处理搞砸'。");
        std::puts("-> 用带这两个块的版本重录一段。录制期间会自动打开。");
        return 2;
    }

    // 模板：从文件头的 template.backMarkers 取。这里只做最粗的 JSON 抠取，
    // 因为完整 JSON 解析不值得为一个字段引入依赖。
    std::array<Vec3, kNumMarkers> tmplMm{};
    {
        const std::string key = "\"backMarkers\"";
        size_t p = rec.header.find(key);
        int n = 0;
        while (p != std::string::npos && n < 15) {
            p = rec.header.find_first_of("-0123456789", p + 1);
            if (p == std::string::npos) break;
            tmplMm[size_t(n / 3)][size_t(n % 3)] = std::strtod(rec.header.c_str() + p, nullptr);
            ++n;
            p = rec.header.find_first_not_of("-0123456789.eE", p);
        }
        if (n < 15) std::puts("警告：文件头里没找全 backMarkers，模板可能不对");
    }

    // 手性 / 拇指 roll 从 header 粗抠(跟 backMarkers 同一套做法)。
    bool isRight = true;
    {
        const std::string key = "\"isRight\"";
        size_t p = rec.header.find(key);
        if (p != std::string::npos) {
            size_t tf = rec.header.find("true", p);
            size_t ff = rec.header.find("false", p);
            if (ff != std::string::npos && (tf == std::string::npos || ff < tf)) isRight = false;
        }
    }
    double thumbRollOffsetRad = 0.0;
    {
        const std::string key = "\"thumbRollOffsetRad\"";
        size_t p = rec.header.find(key);
        if (p != std::string::npos) {
            p = rec.header.find_first_of("-0123456789", p + 1);
            if (p != std::string::npos) thumbRollOffsetRad = std::strtod(rec.header.c_str() + p, nullptr);
        }
    }

    Hm20Config base;
    // 在线面板实际用的阈值(不是结构体默认 0.55)。
    base.minAssignProbFinger = 0.35;
    base.minAssignProbDorsum = 0.30;
    base.thumbRollOffsetRad = thumbRollOffsetRad;
    std::string sweepKey;
    std::vector<double> sweepVals;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--set" && i + 1 < argc) {
            const std::string kv = argv[++i];
            const size_t e = kv.find('=');
            if (e == std::string::npos) continue;
            const std::string k = kv.substr(0, e);
            if (!applyOverride(base, k, std::strtod(kv.c_str() + e + 1, nullptr)))
                std::printf("未知参数 %s（忽略）\n", k.c_str());
        } else if (a == "--sweep" && i + 1 < argc) {
            const std::string kv = argv[++i];
            const size_t e = kv.find('=');
            if (e == std::string::npos) continue;
            sweepKey = kv.substr(0, e);
            const char* q = kv.c_str() + e + 1;
            while (*q) { sweepVals.push_back(std::strtod(q, const_cast<char**>(&q)));
                         if (*q == ',') ++q; else break; }
        }
    }

    std::puts("\n【先看基线对拍】不改参数复算一遍，跟在线结果比：");
    std::puts("标签一致 <99% 说明复算跟在线对不上，先修这个，别急着扫参 ——");
    std::puts("基线错了，扫出来的最优值也是错的。\n");
    printScore("基线（不改参数）", runOnce(rec, tmplMm, base, isRight));

    if (!sweepKey.empty()) {
        std::printf("\n【参数扫描】%s\n", sweepKey.c_str());
        for (const double v : sweepVals) {
            Hm20Config c = base;
            if (!applyOverride(c, sweepKey, v)) {
                std::printf("未知参数 %s\n", sweepKey.c_str()); break;
            }
            char tag[80];
            std::snprintf(tag, sizeof(tag), "  %s=%g", sweepKey.c_str(), v);
            printScore(tag, runOnce(rec, tmplMm, c, isRight));
        }
        std::puts("\n注意：这里的\"标签一致/位置差\"是【跟在线结果比】，不是跟真值比。");
        std::puts("扫参时你要看的是【手背解出率】这类绝对指标，");
        std::puts("以及在 rigid_motion 段上用 pcrec_report.py 算的刚体不变量。");
    }
    return 0;
}
