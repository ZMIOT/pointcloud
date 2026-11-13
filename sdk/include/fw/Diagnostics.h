#pragma once
#include "Algorithm.h"

namespace fw {
// Optional protocol carried by parameterSchema().diagnostics and result.metrics.diagnostics.
// These functions never call plugin code and do not change the v1 plugin ABI.
class FW_API Diagnostics {
public:
    static constexpr int Version = 1;
    static constexpr int MaxProfileBytes = 64 * 1024;
    static constexpr int MaxReportBytes = 256 * 1024;
    static QStringList validateProfile(const QJsonObject& profile, const QJsonObject& parameterSchema,
                                       const AlgorithmDescriptor& descriptor);
    static QStringList validateReport(const QJsonObject& report, const QJsonObject& profile,
                                      const AlgorithmDescriptor& descriptor, const AlgorithmInput& input,
                                      const ParameterSet& parameters, const AlgorithmResult& result);
};
}
