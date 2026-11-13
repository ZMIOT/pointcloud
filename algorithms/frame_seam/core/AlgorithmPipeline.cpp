#include "AlgorithmPipeline.h"
#include <chrono>
#include <cmath>
#include <exception>

namespace weld {
namespace {
void record(const AlgorithmContext &context, const std::string &title, const std::string &status,
            const std::string &message, const std::vector<DebugCloudStage> &layers,
            const std::map<std::string, double> &metrics = {}, double elapsedMs = 0) {
  context.checkCancelled();
  auto trace = context.debug;
  if (!trace && !context.hooks) return;
  DebugStep step;
  step.title = title; step.status = status; step.message = message;
  step.metrics = metrics; step.elapsedMs = elapsedMs;
  for (const auto &layer : layers) {
    step.layers.push_back(layer);
  }
  if (context.hooks && context.hooks->onStep) context.hooks->onStep(step);
  if (trace) {
    if (!context.hooks || context.hooks->retainTrace) {
      for (auto &layer : step.layers) if (layer.cloud) layer.cloud.reset(new CloudT(*layer.cloud));
    } else trace->steps.clear(); // Host owns immutable snapshots and their memory budget.
    trace->steps.push_back(std::move(step));
  }
}
std::vector<DebugCloudStage> outputs(const AlgorithmContext &ctx) {
  return {{"处理点云", ctx.workingCloud}, {"特征点", ctx.features},
          {"左轨迹", ctx.result.leftSeam}, {"右轨迹", ctx.result.rightSeam}};
}
std::string validate(const SeamResult &r) {
  if (!r.ok()) return "轨迹输出不足：左右各需至少 3 个点。";
  for (const auto &cloud : {r.leftSeam, r.rightSeam})
    for (const auto &p : *cloud)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        return "轨迹包含 NaN 或无穷坐标。";
  if ((!r.leftPointConfidence.empty() && r.leftPointConfidence.size() != r.leftSeam->size()) ||
      (!r.rightPointConfidence.empty() && r.rightPointConfidence.size() != r.rightSeam->size()))
    return "逐点置信度数量必须与相应轨迹点数一致，或留空。";
  for (const auto *values : {&r.leftPointConfidence, &r.rightPointConfidence})
    for (float value : *values)
      if (!std::isfinite(value) || value < 0 || value > 1) return "置信度必须位于 [0,1]。";
  for (float value : {r.confidence, r.leftConfidence, r.rightConfidence})
    if (!std::isfinite(value) || value < 0 || value > 1) return "总体置信度必须位于 [0,1]。";
  return {};
}
void clearTrajectories(SeamResult &result) {
  // 出错或未实现时，避免把中间轨迹当作成功结果导出；调试快照仍保留。
  auto cloud = result.filteredCloud;
  auto summary = result.summary;
  result = SeamResult(); result.filteredCloud = cloud; result.summary = summary;
}
}

void AlgorithmContext::snapshot(const std::string &title, const std::string &message,
                                const std::vector<DebugCloudStage> &layers) const {
  record(*this, title, "completed", message, layers, metrics);
}

SeamResult executeSteps(AlgorithmContext &ctx, const std::vector<AlgorithmStep> &steps) {
  ctx.result = SeamResult();
  ctx.workingCloud.reset(new CloudT);
  ctx.features.reset(new CloudT);
  ctx.metrics.clear();
  if (ctx.debug) {
    *ctx.debug = DebugTrace(); ctx.debug->algorithm = "custom";
    ctx.debug->config = ctx.config;
    ctx.debug->leftHistoryPasses = ctx.leftHistory ? ctx.leftHistory->size() : 0;
    ctx.debug->rightHistoryPasses = ctx.rightHistory ? ctx.rightHistory->size() : 0;
  }
  const bool validInput = ctx.input && !ctx.input->empty();
  record(ctx, "00 · 输入点云", validInput ? "completed" : "failed",
         validInput ? "原始 XYZ 点云，保持输入坐标和单位。" : "输入点云为空。",
         {{"输入点云", ctx.input}});
  bool stopped = !validInput;
  if (validInput) *ctx.workingCloud = *ctx.input;
  else ctx.result.summary = "输入点云为空。";
  for (const auto &step : steps) {
    ctx.checkCancelled();
    if (stopped) {
      record(ctx, step.title, "skipped", "前一步未完成，本步骤未执行。", {});
      continue;
    }
    ctx.metrics.clear();
    const auto start = std::chrono::steady_clock::now();
    StepResult outcome;
    try { outcome = step.process(ctx); }
    catch (const ExecutionCancelled &) { throw; }
    catch (const std::exception &e) { outcome = {StepStatus::Failed, e.what()}; }
    catch (...) { outcome = {StepStatus::Failed, "算法抛出未知异常。"}; }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const char *status = outcome.status == StepStatus::Completed ? "completed" :
                         outcome.status == StepStatus::NotImplemented ? "not_implemented" : "failed";
    record(ctx, step.title, status, outcome.message, outputs(ctx), ctx.metrics, ms);
    stopped = outcome.status != StepStatus::Completed;
    if (stopped) ctx.result.summary = step.title + "：" + outcome.message;
  }
  ctx.result.filteredCloud = ctx.workingCloud ? ctx.workingCloud : CloudT::Ptr(new CloudT);
  if (!stopped) {
    const auto error = validate(ctx.result);
    stopped = !error.empty();
    record(ctx, "05 · 输出检查", stopped ? "failed" : "completed",
           stopped ? error : "左右轨迹通过点数、有限坐标及置信度检查。", outputs(ctx));
    if (stopped) ctx.result.summary = error;
    else if (ctx.result.summary.empty()) ctx.result.summary = "自定义算法运行完成。";
  } else {
    record(ctx, "05 · 输出检查", "skipped", "处理流程未完成，未生成可导出结果。", {});
  }
  if (stopped) clearTrajectories(ctx.result);
  if (ctx.debug) ctx.debug->pathComplete = !stopped;
  return ctx.result;
}

} // namespace weld
