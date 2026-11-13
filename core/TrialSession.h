#pragma once
#include "Runtime.h"

namespace fw {
struct TrialVariant {
    QString parameterId;
    ParameterSet values;
};
// Pure host-side planning/comparison. It never executes an algorithm or accepts an output.
class FW_CORE_API TrialSession {
public:
    static QVector<TrialVariant> plan(const QJsonObject& schema, const QJsonObject& profile,
                                      const QJsonObject& baselineReport, const ParameterSet& baseline,
                                      int maxVariants=4);
    static QJsonObject compare(const QJsonObject& baselineReport, const QJsonObject& trialReport);
    // A UI guard, not a claim of accuracy. The user still explicitly chooses adoption.
    static bool oneClickAdoptionAllowed(const QJsonObject& profile, const QJsonObject& comparison,
                                        const QString& parameterId);
};
}
