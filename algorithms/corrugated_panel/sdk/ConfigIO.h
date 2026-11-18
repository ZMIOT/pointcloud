#pragma once

#include "PointCloudWeldSdk.h"

#include <string>

namespace pcws {

bool loadOptionsFile(const std::string &path, PcwsOptionsV1 &options,
                     std::string &error);

} // namespace pcws
