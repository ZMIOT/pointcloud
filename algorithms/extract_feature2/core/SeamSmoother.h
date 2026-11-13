#pragma once

#include "SeamTypes.h"

namespace weld {

// Applies the optional final robust constant-slope Kalman filter and
// fixed-interval RTS smoother.  Legacy mode is an intentional no-op.
void applySeamPostprocess(SeamResult &result, const DetectorConfig &config);

} // namespace weld

