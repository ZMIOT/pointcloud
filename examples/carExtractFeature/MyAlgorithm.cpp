#include <fw/Algorithm.h>
#include <QFile>
#include <QJsonDocument>

class MyAlgorithm final : public fw::IAlgorithm {
public:
    QString validate(const fw::AlgorithmInput& input,const fw::ParameterSet&) const override {
        return input.clouds.isEmpty()?QStringLiteral("需要输入点云"):QString();
    }
    fw::AlgorithmResult execute(const fw::AlgorithmInput& input,const fw::ParameterSet& parameters,fw::IExecutionContext& context) override {
        if(context.isCancelled()) return fw::AlgorithmResult::cancelled();
        auto output=input.clouds.front(); output.name=parameters["name"].toString();
        // Replace this identity operation with your PCL / OpenCV algorithm.
        if(context.debug().enabled()) {
            fw::DebugFrame frame; frame.stepId="result"; frame.title=output.name;
            frame.layers={fw::DebugLayer::result(output.name,output)};
            if(context.debug().checkpoint(frame)==fw::DebugAction::Stop) return fw::AlgorithmResult::cancelled();
        }
        fw::AlgorithmResult result; result.outputs={output}; result.layers={fw::DebugLayer::result(output.name,output)}; return result;
    }
};
class MyAlgorithmPlugin final : public QObject,public fw::IAlgorithmPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_ALGORITHM_IID FILE "metadata.json")
    Q_INTERFACES(fw::IAlgorithmPlugin)
public:
    fw::AlgorithmDescriptor descriptor() const override { return {"custom.identity",QStringLiteral("我的算法"),"1.0.0",{"x","y","z"}}; }
    QJsonObject parameterSchema() const override { QFile f(":/custom.identity/parameters.json"); f.open(QIODevice::ReadOnly); return QJsonDocument::fromJson(f.readAll()).object(); }
    fw::IAlgorithm* create() override { return new MyAlgorithm; }
    void destroy(fw::IAlgorithm* p) override { delete p; }
};
#include "MyAlgorithm.moc"
