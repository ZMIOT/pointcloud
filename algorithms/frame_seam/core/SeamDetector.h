#pragma once

#include "SeamHistory.h"
#include "SeamDebug.h"
#include "SeamTypes.h"

namespace weld {

class SeamDetector {
public:
    SeamResult detect(const CloudT::ConstPtr& input,
                      const DetectorConfig& config,
                      const SeamHistory* leftHistory = nullptr,
                      const SeamHistory* rightHistory = nullptr,
                      DebugTrace* debug = nullptr) const;

private:
    static CloudT::Ptr preprocess(const CloudT::ConstPtr& input,
                                  const DetectorConfig& config,
                                  DebugTrace* debug);
};

} // namespace weld
