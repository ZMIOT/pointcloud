#pragma once
#include "AlgorithmPipeline.h"

namespace weld {
// 方案3：只接受经过验证的原始帧范围，不根据点间距离猜帧。
SeamResult detectFrameSeams(const CloudT::ConstPtr &input,
                            const DetectorConfig &config,
                            const std::vector<ScanFrameRange> &frames,
                            DebugTrace *debug = nullptr,
                            ExecutionHooks *hooks = nullptr);
} // namespace weld
