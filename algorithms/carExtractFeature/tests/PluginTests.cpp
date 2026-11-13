#include "Synthetic.h"
#include <algorithm>
#include <fw/Algorithm.h>
#include <fw/Parameters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <QCoreApplication>
#include <QPluginLoader>
#include <QJsonDocument>
#include <QFile>
#include <QDir>
#include <iostream>
struct Context:fw::IExecutionContext,fw::IDebugContext {
    bool debugging=false,cancelled=false;int stages=0,stopAt=0;
    fw::IDebugContext& debug() override {return *this;}
    bool enabled() const override {return debugging;}
    bool isCancelled() const override {return cancelled;}
    void log(const QString&) override {} void progress(int) override {}
    fw::DebugAction checkpoint(const fw::DebugFrame&) override {++stages;return stopAt&&stages>=stopAt?fw::DebugAction::Stop:fw::DebugAction::Continue;}
};
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        QPluginLoader loader(QString::fromUtf8(PLUGIN_PATH));auto plugin=qobject_cast<fw::IAlgorithmPlugin*>(loader.instance());require(plugin,loader.errorString().toStdString());
        require(plugin->descriptor().id=="custom.carExtractFeature","Wrong ID");const auto schema=plugin->parameterSchema();
        require(fw::Parameters::validateSchema(schema).isEmpty(),"Invalid schema/resource");auto parameters=fw::Parameters::defaults(schema);
        QFile preset(QString::fromUtf8(PRESET_PATH));require(preset.open(QIODevice::ReadOnly),"Cannot open example preset");
        const auto saved=QJsonDocument::fromJson(preset.readAll()).object();
        require(saved["algorithmId"].toString()==plugin->descriptor().id&&
                saved["parameterVersion"]==schema["parameterVersion"]&&saved["values"].isObject(),"Example preset ID or format mismatch");
        const auto presetValues=saved["values"].toObject();
        require(fw::Parameters::validate(schema,presetValues).isEmpty()&&presetValues==parameters,"Example preset differs from valid defaults");
        pcl::PointCloud<pcl::PointXYZ> cloud;for(const auto& p:synthetic()) cloud.push_back(pcl::PointXYZ(float(p.xyz.x()),float(p.xyz.y()),float(p.xyz.z())));
        auto data=fw::DebugLayer::result("input",cloud).data;data.coordinateFrame="test-world";data.unit="mm";
        if(argc==3) {pcl::PCLPointCloud2::Ptr real(new pcl::PCLPointCloud2);require(pcl::io::loadPCDFile(argv[1],*real)==0,"Cannot read real sample");data.cloud=real;data.unit="unknown";}
        fw::AlgorithmInput input{{data},{}};auto algorithm=plugin->create();struct Guard {fw::IAlgorithmPlugin* p;fw::IAlgorithm* a;~Guard(){p->destroy(a);}} guard{plugin,algorithm};
        require(algorithm->validate(input,parameters).isEmpty(),"Default validation failed");Context normal;auto result=algorithm->execute(input,parameters,normal);
        require(result.status==fw::ResultStatus::Success,result.message.toStdString());require(result.outputs.size()==2,"Missing outputs");require(normal.stages==0,"Debug disabled still snapshots");
        require(result.metrics.value("algorithm_elapsed_ms").isDouble()&&result.metrics.value("algorithm_elapsed_ms").toDouble()>=0,"Missing algorithm timing");
        const auto diagnostic=result.metrics.value("diagnostics").toObject();
        require(diagnostic.value("protocolVersion")==1&&diagnostic.value("algorithmId")==plugin->descriptor().id,"Diagnostic report missing or wrong version");
        require(diagnostic.value("input").toObject().value("coordinateFrame")==data.coordinateFrame&&diagnostic.value("parameters").toObject()==parameters,"Diagnostic input/parameter snapshot mismatch");
        require(diagnostic.value("outputs").toArray().size()==2,"Diagnostic output identities missing");
        for(const auto& value:diagnostic.value("issues").toArray()) {
            const auto issue=value.toObject();
            require(!issue.value("code").toString().isEmpty()&&!issue.value("stage").toString().isEmpty()&&issue.value("region").isObject()&&issue.value("evidence").isObject(),"Incomplete diagnostic issue");
        }
        require(result.outputs[1].name==QStringLiteral("最终轨迹点"),"Final trajectory point output is not labeled");
        const auto pointLayer=std::find_if(result.layers.cbegin(),result.layers.cend(),[](const fw::DebugLayer& layer) {return layer.name==QStringLiteral("最终轨迹点");});
        require(pointLayer!=result.layers.cend()&&pointLayer->data.cloud&&pointLayer->data.cloud->data==result.outputs[1].cloud->data,"Displayed green points differ from the exported trajectory");
        require(pointLayer->style.color[1]>pointLayer->style.color[0]&&pointLayer->style.color[1]>pointLayer->style.color[2],"Final trajectory points are not green");
        Context debug;debug.debugging=true;auto stepped=algorithm->execute(input,parameters,debug);
        require(stepped.status==result.status&&debug.stages==6,"Debug stages failed");
        for(int i=0;i<2;++i) {
            require(stepped.outputs[i].cloud->data==result.outputs[i].cloud->data,"Debug changes result");
            require(result.outputs[i].unit==data.unit&&result.outputs[i].coordinateFrame==data.coordinateFrame,"Lost frame/unit");
            require(fw::hasFields(result.outputs[i],{"source_index","section_id","segment_id","confidence","interpolated"}),"Lost metadata");
        }
        Context stop;stop.debugging=true;stop.stopAt=3;require(algorithm->execute(input,parameters,stop).status==fw::ResultStatus::Cancelled,"Debug stop failed");
        Context early;early.cancelled=true;require(algorithm->execute(input,parameters,early).status==fw::ResultStatus::Cancelled,"Early cancellation failed");
        require(!algorithm->validate({},parameters).isEmpty(),"Empty input accepted");auto bad=parameters;bad["neighborhood_scale"]=0.;require(!algorithm->validate(input,bad).isEmpty(),"Bad scale accepted");
        bad=parameters;bad["manual_scan_normal"]=true;bad["scan_nx"]=0.;bad["scan_ny"]=0.;bad["scan_nz"]=0.;require(!algorithm->validate(input,bad).isEmpty(),"Zero normal accepted");
        if(argc==3) {
            QDir().mkpath(argv[2]);const QDir dir(argv[2]);pcl::PCDWriter writer;
            for(int i=0;i<2;++i) require(writer.writeBinary(dir.filePath(i?"trajectory.pcd":"measured.pcd").toStdString(),*result.outputs[i].cloud)==0,"Write PCD failed");
            QFile file(dir.filePath("plugin-report.json"));require(file.open(QIODevice::WriteOnly),"Write report failed");
            auto report=result.metrics;report["debug_matches_normal"]=true;report["debug_stages"]=debug.stages;file.write(QJsonDocument(report).toJson());
        }
        std::cout<<QJsonDocument(result.metrics).toJson(QJsonDocument::Compact).constData()<<"\nPlugin tests passed\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}return 0;
}
