// ===========================================================================
// Hm20OnnxBackend.hpp
//
// IHm20InferenceBackend 的 onnxruntime 实现，对应 HandSkeletonAssociator.hpp
// 里声明的 hm20 契约（6 输入，21 类）。
//
// 【本后端实际消费的是 2 个输出，但模型里可以有更多】
// kOnnxOutputNames() 只请求 log_assign / pos 两个；实际导出的 hm20_v6 /
// hm20_v6_sk10 是 6 个输出（另有 center / scale / ghost_logit / miss_logit
// 这几个训练用的辅助头）。onnxruntime 允许只请求输出子集，多余的头不参与
// 计算图之外的任何事，所以【不要】拿输出个数当契约校验条件，见
// verifyContract() 里的说明。
//
// 【为什么需要这个文件】
// 树里原有的 OnnxSkeletonInferenceBackend.hpp（已删除）实现的是【上一代】契约：
//     输入  points / anchor_dists(1,N,5) / mask / tmpl(1,46)
//     输出  logits(1,N,16) / reg(1,15,3)
//     且 points 必须先转到腕部局部系（in.pointsWristLocal）
// 而 hm20 的 v6/v7 模型是：
//     输入  points / mask / tmpl(1,61) / tmpl_valid(1,20) / prev / prev_mask
//     输出  log_assign(1,N+1,21) / pos(1,20,3) （+ 若干不使用的辅助头）
//     points 是旋转/平移/尺度不变量，【不需要】预先转腕部系
// 两者输入个数、张量维度、类数、坐标系全部不同，旧后端加载 v6 会在
// Session::Run 直接抛"输入个数不匹配"。所以不是改几行能兼容的，得单独一个。
//
// 【接线现状】PointCloudTestDialog.cpp 里已经加载 hm20_v6_sk10.onnx、
// 构造 mocap::hm20::Hm20OnnxBackend（经 SkeletonAssocWorker，在 worker 线程里
// 建会话）。文件末尾那段接线示例保留作参考，不是待办。
// ===========================================================================
#pragma once

#include "estimate/Hm20AssocContract.hpp"   // 【只要类型，不要那个类】原来这里 include 的是
                                            // HandSkeletonAssociator.hpp（闭包 5776 行），但本文件
                                            // 一次都没提到 Hm20SkeletonAssociator，要的只是
                                            // SkeletonFrameResult / Hm20Config / IHm20InferenceBackend
                                            // 这些类型 —— 它们在 contract 里（闭包 1557 行）。
                                            // estimate 目录下有 7 个头都犯了同一个错，每个多吃 4219 行，
                                            // 而这些头又被 UI 层层包含，代价是乘出来的。
#include "estimate/Hm20SegRot.hpp"   // rot6dToMat
#include "estimate/OrtRuntimeLoader.hpp"   // 必须在任何 Ort:: 之前，它定义 ORT_API_MANUAL_INIT

#include <algorithm>   // std::find —— verifyContract() 里按名字查输出用
#include <array>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace mocap {
namespace hm20 {

// HAVE_ONNXRUNTIME 由 CMakeLists.txt:181 在找到 onnxruntime 时定义。
// 没定义时本文件末尾提供桩实现，保证 USE_ONNXRUNTIME=OFF 也能编译。

// ---------------------------------------------------------------------------
// 模板打包：61 维 = 中立位 20 点归一化坐标(60) + 左右手标志(1)
//
// 【单位】送进 ONNX 的 tmpl 是【归一化】的，不是毫米。Kabsch 那边需要的才是
// 毫米，所以两份分开存（markersMm / packNormalized），别共用一个数组。
//
// 【归一化定义必须逐字对齐训练侧】唯一权威定义是语料生成器 synth.py 的
// template_feature()，tools/skeleton_assoc/hm20_sim.py:122 里有一份照抄版：
//     c  = 20 点质心
//     sc = 20 点到质心距离的【平均值】
//     f  = ((P - c) / sc).flatten()  ++  [hand_sign]
// 也就是【先减质心，再除以平均半径】。
//
// 【2026-08 修复——这里原来是错的，而且是"不崩不报只掉精度"的那种错】
// 原实现是 markersMm[m] / handScale()，其中
//     handScale = |中指近节 marker| + 中指三节 marker 间距之和
// 两处都不对：①【没有减质心】，于是整份模板带着一个巨大的平移分量；
// ②尺度定义完全不同，实测 handScale≈150mm 而训练侧的平均半径≈38mm，差约 4 倍。
// 拿 hand_rig.py 造的受试者实测，两种定义出来的 61 维向量逐元素 RMS 差 0.60，
// 而向量本身量级就是 O(1) —— 等于送了一份网络没见过的条件向量进去。
//
// 后果【只砸手背，不砸手指】，所以很容易误判成"模型不行"：
//   手指靠链式几何本身就能认（点特征是旋转/平移/尺度不变量），不依赖模板；
//   而手背 5 点近似五重对称，相位（谁是 0 号）【只能】靠模板打破。
// 实测（60 个随机受试者，0.5mm 噪声，无遮挡无 prev）：
//   无模板            手背 43.7%  top1 概率中位 0.496
//   错模板(修复前)    手背 73.3%  top1 概率中位 0.472   ← 顺带把手指从 72.6% 拖到 66.3%
//   正确模板(修复后)  手背 97.3%  top1 概率中位 0.998
//
// 顺带：hand_sign 训练侧是 +1(右)/-1(左)，原来写的是 1/0。实测这一位对手背
// 准确率影响很小（几何本身已经带了手性），但既然是照抄契约，就照实写 ±1。
// ---------------------------------------------------------------------------
struct Hm20Template {
    std::array<Vec3, kNumMarkers> markersMm{};   // 中立位 20 点，毫米，Kabsch 用
    std::array<Vec3, 5> anchorsMm{};             // 5 个指根关节，毫米，解近节骨朝向用
    bool anchorsValid = false;
    bool isRight = true;
    bool valid = false;

    // 训练侧 template_feature 的尺度：20 点到质心距离的平均值（不是最大值、
    // 不是 RMS、不是某两点的距离——换任何一个都会让网络收到分布外的输入）。
    double templateScaleMm() const {
        Vec3 c{0.0, 0.0, 0.0};
        for (int m = 0; m < kNumMarkers; ++m)
            for (int d = 0; d < 3; ++d) c[size_t(d)] += markersMm[size_t(m)][size_t(d)];
        for (int d = 0; d < 3; ++d) c[size_t(d)] /= double(kNumMarkers);
        double s = 0.0;
        for (int m = 0; m < kNumMarkers; ++m) {
            const double dx = markersMm[size_t(m)][0] - c[0];
            const double dy = markersMm[size_t(m)][1] - c[1];
            const double dz = markersMm[size_t(m)][2] - c[2];
            s += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        return s / double(kNumMarkers);
    }

    std::array<float, 61> packNormalized() const {
        std::array<float, 61> out{};
        Vec3 c{0.0, 0.0, 0.0};
        for (int m = 0; m < kNumMarkers; ++m)
            for (int d = 0; d < 3; ++d) c[size_t(d)] += markersMm[size_t(m)][size_t(d)];
        for (int d = 0; d < 3; ++d) c[size_t(d)] /= double(kNumMarkers);

        const double sc = std::max(templateScaleMm(), 1e-6);
        int k = 0;
        for (int m = 0; m < kNumMarkers; ++m)
            for (int d = 0; d < 3; ++d)
                out[size_t(k++)] = float((markersMm[size_t(m)][size_t(d)] - c[size_t(d)]) / sc);
        out[60] = isRight ? 1.0f : -1.0f;   // 训练侧 hand_sign 是 ±1，不是 1/0
        return out;
    }

    // tmpl_valid 是【逐点】的 (1,20)，不是一个标量开关。
    // 手背 5 点：标定把"0号=极角最小的点"这个任意但自洽的相位锁死了，有效 -> 1.0
    // 手指 15 点：手指标定原理上做不到（也只值 1.0 点），照实发 0.0
    std::array<float, kNumMarkers> packValidMask() const {
        std::array<float, kNumMarkers> v{};
        for (int m = 0; m < kNumMarkers; ++m)
            v[size_t(m)] = (valid && m < 5) ? 1.0f : 0.0f;
        return v;
    }
};

#ifdef HAVE_ONNXRUNTIME

// ---------------------------------------------------------------------------
class Hm20OnnxBackend : public IHm20InferenceBackend {
public:
    // modelPath: hm20_v6_sk10.onnx 的绝对路径
    // 构造不抛异常；加载失败时 ready()==false、lastError() 给原因，
    // run() 返回 ok=false，上层 Hm20SkeletonAssociator 会安全地跳过这一帧。
    explicit Hm20OnnxBackend(const std::string& modelPath, int intraOpThreads = 1) {
        // 【顺序不能变】必须先手动加载 dll 并 Ort::InitApi，才能构造 Ort::Env。
        // 本工程故意不隐式链接 onnxruntime，原因见 OrtRuntimeLoader.hpp。
        if (!mocap::ort::ensureOrtApiLoaded(err_)) return;
        try {
            env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "hm20");
            Ort::SessionOptions so;
            so.SetIntraOpNumThreads(intraOpThreads);
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
#ifdef _WIN32
            std::wstring wp(modelPath.begin(), modelPath.end());
            session_ = std::make_unique<Ort::Session>(*env_, wp.c_str(), so);
#else
            session_ = std::make_unique<Ort::Session>(*env_, modelPath.c_str(), so);
#endif
            if (!verifyContract()) return;      // verifyContract 里已填 err_
            ready_ = true;
        } catch (const Ort::Exception& e) {
            err_ = std::string("onnxruntime 加载失败: ") + e.what()
                 + "。路径: " + modelPath;
        } catch (const std::exception& e) {
            err_ = std::string("加载失败: ") + e.what();
        }
    }

    bool ready() const { return ready_; }
    const std::string& lastError() const { return err_; }
    const std::string& modelIoDump() const { return ioDump_; }

    void setTemplate(const Hm20Template& t) { tmpl_ = t; }

    InferenceOutput run(const InferenceInput& in) override {
        InferenceOutput out;
        if (!ready_) return out;

        const int nReal = int(in.points.size());
        if (nReal <= 0 || nReal > kMaxPoints) return out;

        // ---- 定长 padding 到 kMaxPoints，用 mask 标注有效 ----
        // 训练侧 net.py 是按 MAX_POINTS 定长喂的；即便导出时把 N 设成了动态轴，
        // 定长也一定能跑。反过来不成立，所以这里统一走定长。
        const int N = kMaxPoints;
        std::vector<float> pts(size_t(N) * 3, 0.0f);
        std::vector<float> mask(size_t(N), 0.0f);
        for (int i = 0; i < nReal; ++i) {
            pts[size_t(i) * 3 + 0] = float(in.points[size_t(i)][0]);
            pts[size_t(i) * 3 + 1] = float(in.points[size_t(i)][1]);
            pts[size_t(i) * 3 + 2] = float(in.points[size_t(i)][2]);
            mask[size_t(i)] = 1.0f;
        }

        std::array<float, 61> tmplBuf = tmpl_.valid ? tmpl_.packNormalized()
                                                    : std::array<float, 61>{};
        std::array<float, kNumMarkers> tmplValidBuf = tmpl_.packValidMask();
        // in.tmplValid 是调用方的总开关（Hm20Config::useTemplate）。关掉时
        // 逐点有效位整体清零，网络训练时按概率丢过模板，这条路径是学过的。
        if (!in.tmplValid) tmplValidBuf.fill(0.0f);

        std::vector<float> prev(size_t(kNumMarkers) * 3, 0.0f);
        std::vector<float> prevMask(size_t(kNumMarkers), 0.0f);
        for (int m = 0; m < kNumMarkers; ++m) {
            prev[size_t(m) * 3 + 0] = float(in.prev[size_t(m)][0]);
            prev[size_t(m) * 3 + 1] = float(in.prev[size_t(m)][1]);
            prev[size_t(m) * 3 + 2] = float(in.prev[size_t(m)][2]);
            prevMask[size_t(m)] = in.prevMask[size_t(m)] ? 1.0f : 0.0f;
        }

        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        const std::array<int64_t, 3> sPts{1, N, 3};
        const std::array<int64_t, 2> sMask{1, N};
        const std::array<int64_t, 2> sTmpl{1, 61};
        const std::array<int64_t, 2> sTmplValid{1, kNumMarkers};
        const std::array<int64_t, 3> sPrev{1, kNumMarkers, 3};
        const std::array<int64_t, 2> sPrevMask{1, kNumMarkers};

        std::vector<Ort::Value> inputs;
        inputs.reserve(6);
        inputs.push_back(Ort::Value::CreateTensor<float>(mem, pts.data(), pts.size(),
                                                         sPts.data(), sPts.size()));
        inputs.push_back(Ort::Value::CreateTensor<float>(mem, mask.data(), mask.size(),
                                                         sMask.data(), sMask.size()));
        inputs.push_back(Ort::Value::CreateTensor<float>(mem, tmplBuf.data(), tmplBuf.size(),
                                                         sTmpl.data(), sTmpl.size()));
        inputs.push_back(Ort::Value::CreateTensor<float>(mem, tmplValidBuf.data(), tmplValidBuf.size(),
                                                         sTmplValid.data(), sTmplValid.size()));
        inputs.push_back(Ort::Value::CreateTensor<float>(mem, prev.data(), prev.size(),
                                                         sPrev.data(), sPrev.size()));
        inputs.push_back(Ort::Value::CreateTensor<float>(mem, prevMask.data(), prevMask.size(),
                                                         sPrevMask.data(), sPrevMask.size()));

        try {
            const auto& inNames = kOnnxInputNames();
            const auto& outNames = kOnnxOutputNames();
            auto res = session_->Run(Ort::RunOptions{nullptr},
                                     inNames.data(), inputs.data(), inputs.size(),
                                     outNames.data(), outNames.size());

            // ---- log_assign (1, N+1, 21) ----
            auto laShape = res[0].GetTensorTypeAndShapeInfo().GetShape();
            if (laShape.size() != 3 || laShape[2] != kNumClasses) {
                err_ = "log_assign 维度不符合契约";
                return out;
            }
            const int rows = int(laShape[1]);
            const float* la = res[0].GetTensorData<float>();
            out.rows = rows;
            out.cols = kNumClasses;
            out.logAssign.assign(la, la + size_t(rows) * size_t(kNumClasses));

            // ---- pos (1, 20, 3) ----
            auto pShape = res[1].GetTensorTypeAndShapeInfo().GetShape();
            if (pShape.size() != 3 || pShape[1] != kNumMarkers || pShape[2] != 3) {
                err_ = "pos 维度不符合契约";
                return out;
            }
            const float* pp = res[1].GetTensorData<float>();
            for (int m = 0; m < kNumMarkers; ++m)
                out.pos[size_t(m)] = {double(pp[m * 3 + 0]),
                                      double(pp[m * 3 + 1]),
                                      double(pp[m * 3 + 2])};
            // ---- v7 姿态版的三个新头（v6 模型没有，跳过即可）----
            // 【为什么要逐个判在不在】树里同时存在 v6 和 v7 两代模型，而
            // verifyContract 只保证 log_assign/pos 必须有。新头缺失时对应的
            // has* 保持 false，上层退回几何路径 —— 换回旧模型不该让程序崩。
            // 【为什么按名字找而不是按下标】Run() 请求的输出里，缺的那些
            // onnxruntime 会返回空 Value，下标会错位。按名字定位最稳。
            const auto& on = kOnnxOutputNames();
            auto slot = [&on](const char* want) -> int {
                for (size_t i = 0; i < on.size(); ++i)
                    if (std::string(on[i]) == want) return int(i);
                return -1;
            };
            auto valid = [&res](int i) {
                return i >= 0 && size_t(i) < res.size() && res[size_t(i)].IsTensor();
            };

            const int iJa = slot("joint_ang");
            if (valid(iJa) && haveOut_.joint) {
                auto sh = res[size_t(iJa)].GetTensorTypeAndShapeInfo().GetShape();
                // 训练侧是 5指×4维=20。这里【不接受 16】——如果哪天模型改成
                // 16 维，映射关系就变了，宁可不用也不要按错的语义解读。
                if (sh.size() == 2 && sh[1] == 20) {
                    const float* ja = res[size_t(iJa)].GetTensorData<float>();
                    for (int k = 0; k < 20; ++k) out.jointAng[size_t(k)] = double(ja[k]);
                    out.hasJointAng = true;
                }
            }
            const int iHl = slot("hand_logit");
            if (valid(iHl) && haveOut_.hand) {
                const float* hl = res[size_t(iHl)].GetTensorData<float>();
                out.handLogit = double(hl[0]);
                out.hasHandLogit = true;
            }
            const int iPc = slot("pose_conf");
            if (valid(iPc) && haveOut_.conf) {
                auto sh = res[size_t(iPc)].GetTensorTypeAndShapeInfo().GetShape();
                if (sh.size() == 2 && sh[1] == 5) {
                    const float* pc = res[size_t(iPc)].GetTensorData<float>();
                    for (int k = 0; k < 5; ++k) out.poseConf[size_t(k)] = double(pc[k]);
                    out.hasPoseConf = true;
                }
            }
            // seg_rot6d (1,16,6) —— 唯一一条【不经过 marker 差分】的姿态来源。
            // 拇指远节被遮挡时，它比"两个球做差"稳得多，因为它不需要那两个球。
            const int iCt = slot("center");
            const int iSc = slot("scale");
            if (valid(iCt) && valid(iSc)) {
                const float* ct = res[size_t(iCt)].GetTensorData<float>();
                const float* sc = res[size_t(iSc)].GetTensorData<float>();
                out.center = Vec3{double(ct[0]), double(ct[1]), double(ct[2])};
                out.scale  = double(sc[0]);
                out.hasCenterScale = true;
            }
            const int iMl = slot("miss_logit");
            if (valid(iMl)) {
                auto sh = res[size_t(iMl)].GetTensorTypeAndShapeInfo().GetShape();
                if (sh.size() == 2 && sh[1] == kNumMarkers) {
                    const float* ml = res[size_t(iMl)].GetTensorData<float>();
                    for (int m = 0; m < kNumMarkers; ++m) out.missLogit[size_t(m)] = double(ml[m]);
                }
            }
            const int iSr = slot("seg_rot6d");
            if (valid(iSr) && haveOut_.seg) {
                auto sh = res[size_t(iSr)].GetTensorTypeAndShapeInfo().GetShape();
                if (sh.size() == 3 && sh[1] == kNumSegments && sh[2] == 6) {
                    const float* sr = res[size_t(iSr)].GetTensorData<float>();
                    for (int s = 0; s < kNumSegments; ++s)
                        out.segR[size_t(s)] = rot6dToMat(&sr[size_t(s) * 6]);
                    out.hasSegR = true;
                }
            }

            out.ok = true;
        } catch (const Ort::Exception& e) {
            err_ = std::string("推理失败: ") + e.what();
        }
        return out;
    }

private:
    // 加载后立刻把模型真实的 I/O 和契约对一遍。
    // 这一步很重要：树里同时存在两代模型，路径配错时报"输入个数 4 != 6"
    // 远比在 Run 里抛一个 onnxruntime 的原始异常好定位。
    bool verifyContract() {
        Ort::AllocatorWithDefaultOptions alloc;
        std::ostringstream os;
        const size_t nIn = session_->GetInputCount();
        const size_t nOut = session_->GetOutputCount();
        os << "模型 I/O: " << nIn << " 输入 / " << nOut << " 输出\n";

        std::vector<std::string> gotIn, gotOut;
        for (size_t i = 0; i < nIn; ++i) {
            auto nm = session_->GetInputNameAllocated(i, alloc);
            auto sh = session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            gotIn.emplace_back(nm.get());
            os << "  in[" << i << "] " << nm.get() << " (";
            for (size_t k = 0; k < sh.size(); ++k) os << (k ? "," : "") << sh[k];
            os << ")\n";
        }
        for (size_t i = 0; i < nOut; ++i) {
            auto nm = session_->GetOutputNameAllocated(i, alloc);
            auto sh = session_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            gotOut.emplace_back(nm.get());
            os << "  out[" << i << "] " << nm.get() << " (";
            for (size_t k = 0; k < sh.size(); ++k) os << (k ? "," : "") << sh[k];
            os << ")\n";
        }
        ioDump_ = os.str();

        // 【2026-08 修复——不要改回按输出【个数】判断】
        // 原来这里写的是 if (nIn != 6 || nOut != 2)，把训练侧正常导出的
        // hm20_v6 / hm20_v6_sk10 直接判死了：那两个模型都是 6 输入 /【6】输出
        //     log_assign, pos, center, scale, ghost_logit, miss_logit
        // 后 4 个是训练时的辅助头，导出时一并写进了 graph。症状是 ready_ 永远
        // false -> SkeletonAssocWorker 走 backendReady_=false 分支 -> 界面显示
        // "骨骼AI：不可用（期望 6 输入/2 输出，实际 6/6）"，但模型本身完全没问题。
        //
        // 多出来的输出是无害的：Run() 传的是 kOnnxOutputNames()={log_assign,pos}，
        // onnxruntime 支持只请求输出子集，返回顺序 = 请求顺序，所以 res[0]/res[1]
        // 仍然是 log_assign/pos，run() 里那两处形状校验照常成立
        // (实测 sk10：log_assign (1,27,21)、pos (1,20,3))。
        //
        // 所以判定条件改成【按名字查所需的输出在不在】，不再数个数——数个数
        // 正是这次踩坑的根因：训练侧多导出一个头，运行侧就整个瘫掉。
        auto hasOutput = [&gotOut](const char* n) {
            return std::find(gotOut.begin(), gotOut.end(), std::string(n)) != gotOut.end();
        };
        if (nIn != 6 || !hasOutput("log_assign") || !hasOutput("pos")) {
            err_ = "这个模型不是 hm20 契约（需要 6 输入 + log_assign/pos 两个输出，"
                   "实际 " + std::to_string(nIn) + " 输入/" + std::to_string(nOut)
                 + " 输出）。若它是 4 输入且带 anchor_dists，那是上一代模型，"
                   "是上一代 skeleton_assoc.onnx，不是 hm20 模型。\n" + ioDump_;
            return false;
        }
        // ---- 记录 v7 的新头在不在，只查一次 ----
        // 【为什么不当成必需】树里同时存在 v6 和 v7。v6 没有这三个头，
        // 但它的 log_assign/pos 完全可用 —— 把它判死是这个文件注释里
        // 已经写明的历史教训(曾经按输出个数校验，把正常模型判成"不可用")。
        haveOut_.joint = hasOutput("joint_ang");
        haveOut_.hand  = hasOutput("hand_logit");
        haveOut_.conf  = hasOutput("pose_conf");
        haveOut_.seg   = hasOutput("seg_rot6d");
        {
            std::ostringstream ex;
            ex << "姿态头: joint_ang " << (haveOut_.joint ? "有" : "无")
               << " / hand_logit " << (haveOut_.hand ? "有" : "无")
               << " / pose_conf " << (haveOut_.conf ? "有" : "无")
               << " / seg_rot6d " << (haveOut_.seg ? "有" : "无");
            if (!haveOut_.joint)
                ex << "  <- 这是 v6 模型，姿态走原来的几何路径";
            ioDump_ += ex.str() + "\n";
        }

        const auto& want = kOnnxInputNames();
        for (size_t i = 0; i < 6; ++i) {
            if (gotIn[i] != want[i]) {
                err_ = "输入名不匹配：期望 " + std::string(want[i])
                     + "，实际 " + gotIn[i] + "\n" + ioDump_;
                return false;
            }
        }
        return true;
    }

    // v7 新头的存在性，verifyContract 时查一次，run() 里直接用
    struct { bool joint = false, hand = false, conf = false, seg = false; } haveOut_;

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    Hm20Template tmpl_;
    bool ready_ = false;
    std::string err_;
    std::string ioDump_;
};

#else  // !HAVE_ONNXRUNTIME —— 桩实现

class Hm20OnnxBackend : public IHm20InferenceBackend {
public:
    explicit Hm20OnnxBackend(const std::string&, int = 1) {}
    bool ready() const { return false; }
    const std::string& lastError() const {
        static std::string s = "本次构建未启用 onnxruntime(USE_ONNXRUNTIME=OFF 或未找到库)，"
                               "骨架关联功能不可用。";
        return s;
    }
    const std::string& modelIoDump() const { static std::string s; return s; }
    void setTemplate(const Hm20Template&) {}
    InferenceOutput run(const InferenceInput&) override { return InferenceOutput{}; }
};

#endif // HAVE_ONNXRUNTIME

} // namespace hm20
} // namespace mocap

// ===========================================================================
// 接到点云测试界面（【已完成】，以下留作参考，不是待办）
// ===========================================================================
// PointCloudTestDialog.cpp 里现在已经是 hm20_v6_sk10.onnx + SkeletonAssocWorker
// 的写法（会话建在 worker 线程，GUI 线程不碰 onnxruntime）。等价的直连写法：
//
//   auto backend = std::make_shared<mocap::hm20::Hm20OnnxBackend>(
//       (QCoreApplication::applicationDirPath()
//        + QStringLiteral("/hm20_v6_sk10.onnx")).toStdString());
//   if (!backend->ready()) {
//       qWarning().noquote() << QString::fromStdString(backend->lastError());
//       // 界面上把 modelIoDump() 显示出来，比只说"加载失败"有用得多
//   }
//   mocap::hm20::Hm20Template t;
//   t.markersMm = <标定得到的中立位 20 点，毫米>;
//   t.isRight   = <左右手>;
//   t.valid     = <手背相位是否已标定>;
//   backend->setTemplate(t);
//
//   mocap::hm20::Hm20Config cfg;
//   cfg.minAssignProbFinger = 0.55;   // 手指阈值（阈值扫描的推荐值）
//   cfg.minAssignProbDorsum = 0.30;   // 手背阈值——必须比手指低，见下
//   cfg.useTemplate   = true;
//   cfg.usePrevFrame  = true;    // 120fps 下信息量最大的输入
//   assoc_ = std::make_unique<mocap::hm20::Hm20SkeletonAssociator>(
//                backend, t.packNormalized(), t.valid, cfg);
//
// 【手背 0.30 已经拆开了】Hm20Config 现在是 minAssignProbFinger /
// minAssignProbDorsum 两个字段，process() 里按 j<5 分流。原来共用一个 0.55
// 的时候，七成以上手背点被判进 dustbin —— 症状是"手指连得很好、手背基本
// 不认"，顺带腕部位姿也解不出来。
//
// 【IHm20IkRefiner 还没接】不接的话，被遮挡的点直接用网络 pos 头的预测，
// 手指段四元数的中位偏差按标定过的手模型实测约 11°（改完配对之后），这部分
// 只有 IK 能消。见 setIkRefiner()。
// ===========================================================================
