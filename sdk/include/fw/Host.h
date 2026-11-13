#pragma once
#include "Algorithm.h"
#include <QObject>
namespace fw {
class IParameterUiPlugin;
class IVisualizationPlugin;
class FW_API Host : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;
    virtual QVector<AlgorithmDescriptor> algorithms() const = 0;
    virtual QJsonObject schema(const QString&) const = 0;
    virtual IParameterUiPlugin* parametersUi() const = 0;
    virtual IVisualizationPlugin* visualization() const = 0;
    virtual QStringList diagnostics() const = 0;
    virtual bool busy() const = 0;
    virtual bool start(const QString&, const AlgorithmInput&, const ParameterSet&, bool debug, QString& error) = 0;
    virtual void next() = 0;
    virtual void continueRun() = 0;
    virtual void singleStep() = 0;
    virtual void stop() = 0;
    virtual void presented(quint64 sequence) = 0;
    virtual void setDebugLimits(int windows, quint64 bytes) = 0;
    virtual FramePtr history(quint64 sequence) const = 0;
    virtual bool loadPreset(const QString& algorithm, const QString& file, ParameterSet&, QStringList& messages) = 0;
    virtual bool savePreset(const QString& algorithm, const QString& file, const ParameterSet&, QString& error) = 0;
    virtual CloudData readCloud(const QString&, const QString& unit, QString& error) = 0;
    virtual bool writeCloud(const QString&, const CloudData&, QString& error) = 0;
signals:
    void logMessage(const QString&);
    void progressChanged(int);
    void runningChanged(bool);
    void pausedChanged(bool);
    void debugReady(fw::FramePtr);
    void debugEvicted(quint64 sequence);
    void finished(fw::AlgorithmResult);
};
}
