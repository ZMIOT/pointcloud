#pragma once
#include "Data.h"
#include <QtPlugin>
namespace fw {
enum class DebugAction { Continue, Stop };
class IDebugContext {
public:
    virtual ~IDebugContext() = default;
    virtual bool enabled() const = 0;
    virtual DebugAction checkpoint(const DebugFrame&) = 0;
};
class IExecutionContext {
public:
    virtual ~IExecutionContext() = default;
    virtual IDebugContext& debug() = 0;
    virtual bool isCancelled() const = 0;
    virtual void log(const QString&) = 0;
    virtual void progress(int percent) = 0;
};
struct AlgorithmDescriptor { QString id, name, version; QStringList requiredFields; };
class IAlgorithm {
public:
    virtual ~IAlgorithm() = default;
    virtual QString validate(const AlgorithmInput&, const ParameterSet&) const = 0;
    virtual AlgorithmResult execute(const AlgorithmInput&, const ParameterSet&, IExecutionContext&) = 0;
};
class IAlgorithmPlugin {
public:
    virtual ~IAlgorithmPlugin() = default;
    virtual AlgorithmDescriptor descriptor() const = 0;
    virtual QJsonObject parameterSchema() const = 0;
    // Factory methods run on the worker. Do not create QObject children of the plugin.
    virtual IAlgorithm* create() = 0;
    virtual void destroy(IAlgorithm* instance) = 0;
    virtual bool migrateParameters(int, const ParameterSet&, ParameterSet&, QString& error) const {
        error=QStringLiteral("此算法未提供参数迁移，请加载默认值。"); return false;
    }
};
}
#define FW_ALGORITHM_IID "org.pointcloud.framework.IAlgorithmPlugin/1.0"
Q_DECLARE_INTERFACE(fw::IAlgorithmPlugin, FW_ALGORITHM_IID)
