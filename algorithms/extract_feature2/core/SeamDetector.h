#pragma once

#include "SeamDebug.h"
#include "SeamHistory.h"
#include "SeamTypes.h"

namespace weld {

class SeamDetector {
public:
  SeamResult detect(const CloudT::ConstPtr &input, const DetectorConfig &config,
                    const SeamHistory *leftHistory = nullptr,
                    const SeamHistory *rightHistory = nullptr,
                    DebugTrace *debug = nullptr,
                    const std::vector<ScanFrameRange> *frames = nullptr,
                    const std::function<bool()> &cancelled = {}) const;

private:
  static CloudT::Ptr preprocess(const CloudT::ConstPtr &input,
                                const DetectorConfig &config,
                                DebugTrace *debug);
};

} // namespace weld
