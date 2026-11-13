#pragma once
#include "Data.h"
#include <QJsonArray>
#include <QStringList>
namespace fw {
class FW_API Parameters {
public:
    static QJsonArray definitions(const QJsonObject& schema);
    static QStringList validateSchema(const QJsonObject& schema);
    static QStringList validate(const QJsonObject& schema, const ParameterSet& values);
    static ParameterSet defaults(const QJsonObject& schema);
    static ParameterSet merge(const QJsonObject& schema, const ParameterSet& values, QStringList* warnings=nullptr);
    static bool condition(const QJsonValue& rule, const ParameterSet& values);
};
}
