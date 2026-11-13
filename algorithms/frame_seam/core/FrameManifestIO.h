#pragma once
#include "AlgorithmPipeline.h"
#include <QString>
namespace weld {
QString findFrameManifest(const QString &inputPath);
bool loadFrameManifest(const QString &path, const CloudT::ConstPtr &input,
                       std::vector<ScanFrameRange> *frames, QString *error,
                       std::function<bool()> cancelled = {});
} // namespace weld
