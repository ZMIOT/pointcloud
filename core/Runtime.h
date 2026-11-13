#pragma once
#include <fw/UiInterfaces.h>
#include "DebugController.h"
#include <QMap>
#include <QPluginLoader>
#include <deque>
#include <thread>
#include <vector>
#ifdef FW_CORE_BUILD
#define FW_CORE_API Q_DECL_EXPORT
#else
#define FW_CORE_API Q_DECL_IMPORT
#endif
namespace fw {
class FW_CORE_API Runtime final : public Host {
    Q_OBJECT
public:
    Runtime();
    ~Runtime() override;
    bool initialize(const QString& pluginRoot, QString& error);
    QWidget* createWindow();
    void shutdown();
    QVector<AlgorithmDescriptor> algorithms() const override;
    QJsonObject schema(const QString&) const override;
    IParameterUiPlugin* parametersUi() const override { return parameters_; }
    IVisualizationPlugin* visualization() const override { return visualization_; }
    QStringList diagnostics() const override { return diagnostics_; }
    bool busy() const override { return busy_; }
    bool start(const QString&,const AlgorithmInput&,const ParameterSet&,bool,QString&) override;
    // Internal host operation. Trial output is delivered only through trialFinished.
    bool startTrial(const QString&,const AlgorithmInput&,const ParameterSet&,QString&);
    void next() override { debug_.next(); }
    void continueRun() override { debug_.resume(); }
    void singleStep() override { debug_.singleStep(); }
    void stop() override { debug_.stop(); }
    void presented(quint64 sequence) override { debug_.presented(sequence); }
    void setDebugLimits(int,quint64) override;
    FramePtr history(quint64) const override;
    bool loadPreset(const QString&,const QString&,ParameterSet&,QStringList&) override;
    bool savePreset(const QString&,const QString&,const ParameterSet&,QString&) override;
    CloudData readCloud(const QString&,const QString&,QString&) override;
    bool writeCloud(const QString&,const CloudData&,QString&) override;
signals:
    void trialFinished(fw::AlgorithmResult);
private:
    struct Entry { IAlgorithmPlugin* plugin; AlgorithmDescriptor descriptor; QJsonObject schema; };
    QMap<QString,Entry> entries_;
    std::vector<std::unique_ptr<QPluginLoader>> loaders_;
    IParameterUiPlugin* parameters_=nullptr;
    IVisualizationPlugin* visualization_=nullptr;
    IMainUiPlugin* main_=nullptr;
    QStringList diagnostics_;
    DebugController debug_;
    std::thread worker_;
    bool busy_=false, closing_=false;
    quint64 run_=0, historyBytes_=0, budget_=512ull*1024*1024;
    int windowLimit_=8;
    std::deque<FramePtr> frames_;
    void acceptFrame(FramePtr);
    bool startImpl(const QString&,const AlgorithmInput&,const ParameterSet&,bool debugging,bool trial,QString&);
};
}
