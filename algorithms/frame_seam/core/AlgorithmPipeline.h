#pragma once

#include "SeamDetector.h"
#include <functional>
#include <exception>

namespace weld {

// Host-neutral execution bridge. The original numerical routines remain independent of Qt UI.
struct ExecutionCancelled : std::exception {
  const char *what() const noexcept override { return "Execution cancelled"; }
};
struct ExecutionHooks {
  std::function<bool()> cancelled;
  std::function<void(const DebugStep &)> onStep;
  bool retainTrace = true;
  void check() const { if (cancelled && cancelled()) throw ExecutionCancelled(); }
};

enum class AlgorithmKind { Custom, Reference, FrameSurface };

struct ScanFrameRange {
  std::size_t begin = 0, count = 0;
  int sdkSequence = 0;
};
enum class StepStatus { Completed, NotImplemented, Failed };

struct StepResult {
  StepStatus status = StepStatus::Completed;
  std::string message;
};

// 每次运行新建一个上下文。这里可增加你需要的截面、法向、候选等中间数据。
// 算法层不依赖 Qt、窗口或文件路径；输入点云和历史只读。
struct AlgorithmContext {
  CloudT::ConstPtr input;
  DetectorConfig config;
  const SeamHistory *leftHistory = nullptr, *rightHistory = nullptr;
  CloudT::Ptr workingCloud{new CloudT};
  CloudT::Ptr features{new CloudT};
  SeamResult result;
  DebugTrace *debug = nullptr;
  ExecutionHooks *hooks = nullptr;
  void checkCancelled() const { if (hooks) hooks->check(); }
  std::map<std::string, double> metrics; // 每个步骤开始前清空。

  // 在任意内部循环调用，可记录某个截面/候选/拟合结果。
  // layers 中的点保持原始 XYZ 坐标；文件名由框架生成。
  void snapshot(const std::string &title, const std::string &message,
                const std::vector<DebugCloudStage> &layers) const;
};

struct AlgorithmStep {
  std::string title;
  std::function<StepResult(AlgorithmContext &)> process;
};

// 可独立运行任意步骤列表，供自定义算法和测试复用。
SeamResult executeSteps(AlgorithmContext &context,
                        const std::vector<AlgorithmStep> &steps);

// GUI 和 CLI 唯一的算法入口；Reference 直接调用原 SeamDetector。
SeamResult runAlgorithm(AlgorithmKind kind, const CloudT::ConstPtr &input,
                        const DetectorConfig &config,
                        const SeamHistory *leftHistory = nullptr,
                        const SeamHistory *rightHistory = nullptr,
                        DebugTrace *debug = nullptr,
                        const std::vector<ScanFrameRange> *frames = nullptr);

} // namespace weld
