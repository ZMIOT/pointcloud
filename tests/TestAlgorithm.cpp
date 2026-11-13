#include <fw/Algorithm.h>
#include <QJsonDocument>
#include <stdexcept>
#include <QThread>
class StressAlgorithm final : public fw::IAlgorithm {
public:
    QString validate(const fw::AlgorithmInput&,const fw::ParameterSet&) const override { return {}; }
    fw::AlgorithmResult execute(const fw::AlgorithmInput& input,const fw::ParameterSet& p,fw::IExecutionContext& context) override {
        if(p["fail"].toBool()) throw std::runtime_error("Deliberate test exception");
        for(int i=0;i<p["count"].toInt();++i) {
            if(context.isCancelled()) return fw::AlgorithmResult::cancelled();
            if(p["slow"].toBool()) QThread::msleep(2);
            fw::DebugFrame frame; frame.stepId="iteration"; frame.title=QString::number(i); frame.layers={fw::DebugLayer::result("result",input.clouds.front())};
            if(context.debug().checkpoint(frame)==fw::DebugAction::Stop) return fw::AlgorithmResult::cancelled();
        }
        fw::AlgorithmResult result; result.outputs=input.clouds; return result;
    }
};
class TestPlugin final : public QObject,public fw::IAlgorithmPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_ALGORITHM_IID FILE "metadata.json")
    Q_INTERFACES(fw::IAlgorithmPlugin)
public:
    fw::AlgorithmDescriptor descriptor() const override { return {"tests.stress","Stress fixture","1.0.0",{"x","y","z"}}; }
    QJsonObject parameterSchema() const override {
        return QJsonDocument::fromJson(R"({"schemaVersion":1,"parameterVersion":1,"algorithmId":"tests.stress","groups":[{"id":"test","parameters":[{"id":"count","type":"int","default":100,"min":1,"max":1000},{"id":"fail","type":"bool","default":false},{"id":"slow","type":"bool","default":false}]}]})").object();
    }
    fw::IAlgorithm* create() override { return new StressAlgorithm; }
    void destroy(fw::IAlgorithm* p) override { delete p; }
};
#include "TestAlgorithm.moc"
