#include <fw/Algorithm.h>
#include <fw/Parameters.h>
#include "FrameSeamAlgorithm.h"
#include "FrameManifestIO.h"
#include "PointCloudIO.h"
#include <QCoreApplication>
#include <QPluginLoader>
#include <QTemporaryDir>
#include <QJsonDocument>
#include <QFile>
#include <QDir>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cstring>

void require(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
struct Context : fw::IExecutionContext,fw::IDebugContext {
    bool debugging=false, cancelled=false; int checkpoints=0,stopAt=0;
    fw::FramePtr first;
    fw::IDebugContext& debug() override {return *this;}
    bool enabled() const override {return debugging;}
    bool isCancelled() const override {return cancelled;}
    void log(const QString&) override {}
    void progress(int) override {}
    fw::DebugAction checkpoint(const fw::DebugFrame& f) override {
        ++checkpoints;
        if(!first) first=fw::snapshot(f,1024*1024);
        if(stopAt&&checkpoints>=stopAt) {cancelled=true; return fw::DebugAction::Stop;}
        return fw::DebugAction::Continue;
    }
};
void writeJson(const QString& path,const QJsonObject& object) {
    QFile file(path); require(file.open(QIODevice::WriteOnly),"Cannot write JSON"); file.write(QJsonDocument(object).toJson());
}
fw::AlgorithmInput inputFrom(const weld::CloudT& cloud) {
    auto data=fw::DebugLayer::result(QStringLiteral("原始扫描"),cloud).data; data.unit="mm"; data.id="scan";
    return {{data},{}};
}
void compare(const fw::AlgorithmResult& a,const fw::AlgorithmResult& b) {
    require(a.status==fw::ResultStatus::Success&&b.status==a.status,"Debug execution failed");
    require(a.outputs.size()==2&&b.outputs.size()==2,"Expected independent left/right outputs");
    for(int side=0;side<2;++side) {
        auto x=fw::toCloud<weld::PointT>(a.outputs[side]),y=fw::toCloud<weld::PointT>(b.outputs[side]);
        require(x->size()==y->size(),"Debug changes point count");
        for(std::size_t i=0;i<x->size();++i) require((weld::toVector((*x)[i])-weld::toVector((*y)[i])).norm()==0,"Debug changes coordinates");
        require(fw::hasFields(a.outputs[side],{"confidence","segment_id"}),"Lost confidence/segments");
        require(a.outputs[side].unit=="mm","Lost unit metadata");
    }
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        QPluginLoader loader(QString::fromUtf8(PLUGIN_PATH)); auto plugin=qobject_cast<fw::IAlgorithmPlugin*>(loader.instance());
        require(plugin!=nullptr,qPrintable(loader.errorString()));
        const auto schema=plugin->parameterSchema(); const auto errors=fw::Parameters::validateSchema(schema);
        require(errors.isEmpty(),qPrintable(errors.join('\n')));
        QTemporaryDir temporary; require(temporary.isValid(),"Cannot create fixtures");
        weld::CloudT::Ptr cloud(new weld::CloudT); std::vector<weld::ScanFrameRange> ranges;
        QString manifest=temporary.filePath("manifest.json");
        const bool real=argc==5&&QString::fromLocal8Bit(argv[1])=="--real";
        if(real) {
            std::string error; cloud=weld::PointCloudIO::load(argv[2],&error); require(cloud&&!cloud->empty(),error.c_str()); manifest=QString::fromLocal8Bit(argv[3]);
        } else {
            QJsonArray frames;
            for(int f=0;f<40;++f) {
                const auto begin=cloud->size(); weld::CloudT frame;
                for(int j=0;j<=466;++j) {
                    float y=-35.f+.15f*j,a=std::abs(y);
                    float z=a<12?20.f+1.2f*std::cos(y/5.f):a<22?2.f*(22-a):0.f;
                    if(f==14||f==15) z=0;
                    frame.push_back(weld::PointT(.7f*f,y,z+.3f*f+.03f*std::sin(j*1.7f+f)));
                }
                *cloud+=frame; const QString name=QString("frame_%1.pcd").arg(f);
                require(pcl::io::savePCDFileBinary(temporary.filePath(name).toLocal8Bit().constData(),frame)==0,"Write fixture failed");
                frames.append(QJsonObject{{"file",name},{"pointCount",int(frame.size())},{"startIndex",int(begin+1)},{"endIndex",int(cloud->size())},{"sdkSequence",f+1}});
            }
            writeJson(manifest,{{"inputPointCount",int(cloud->size())},{"frames",frames}});
        }
        auto p=fw::Parameters::defaults(schema); p["manifest"]=manifest;
        require(fw::Parameters::validate(schema,p).isEmpty(),"Invalid mapped defaults");
        auto input=inputFrom(*cloud); auto algorithm=plugin->create();
        struct Guard {fw::IAlgorithmPlugin* p; fw::IAlgorithm* a; ~Guard(){p->destroy(a);}} guard{plugin,algorithm};
        require(algorithm->validate(input,p).isEmpty(),"Valid scan rejected");
        Context normal; auto result=algorithm->execute(input,p,normal);
        require(result.status==fw::ResultStatus::Success,qPrintable(result.message)); require(normal.checkpoints==0,"Debug-off emitted snapshots");
        Context stepped; stepped.debugging=true; auto debugged=algorithm->execute(input,p,stepped); compare(result,debugged);
        require(stepped.checkpoints==6,"Expected input + four stages + output checkpoints");
        if(real) {
            const QString out=QString::fromLocal8Bit(argv[4]); require(QDir().mkpath(out),"Cannot create output directory");
            require(pcl::io::savePCDFileBinary(QDir(out).filePath("original_scan.pcd").toLocal8Bit().constData(),*cloud)==0,"Cannot save converted input");
            pcl::PCDWriter writer;
            for(int i=0;i<2;++i) require(writer.writeBinary(QDir(out).filePath(i?"right.pcd":"left.pcd").toLocal8Bit().toStdString(),*result.outputs[i].cloud)==0,"Cannot save plugin output");
            writeJson(QDir(out).filePath("parameters.preset.json"),{{"algorithmId","weld.frame_seam"},{"parameterVersion",1},{"values",p}});
            auto report=result.metrics; report["debugCheckpoints"]=stepped.checkpoints; report["debugMatchesNormal"]=true; report["inputPoints"]=int(cloud->size());
            writeJson(QDir(out).filePath("report.json"),report);
        } else {
            Context detail; detail.debugging=true; p["debug_detail"]="frames";
            compare(result,algorithm->execute(input,p,detail)); require(detail.checkpoints==46,"Per-frame bridge lost steps");
            Context stopped; stopped.debugging=true; stopped.stopAt=2;
            require(algorithm->execute(input,p,stopped).status==fw::ResultStatus::Cancelled,"Checkpoint stop not propagated");
            Context preCancelled; preCancelled.cancelled=true;
            require(algorithm->execute(input,p,preCancelled).status==fw::ResultStatus::Cancelled,"Initial cancel ignored");
            auto mismatch=inputFrom(*cloud); auto changed=fw::toCloud<weld::PointT>(mismatch.clouds[0]); (*changed)[0].z+=1;
            mismatch=inputFrom(*changed); Context invalid;
            require(algorithm->execute(mismatch,p,invalid).status==fw::ResultStatus::Failed,"Coordinate mismatch accepted");
            fw::AlgorithmInput empty; require(!algorithm->validate(empty,p).isEmpty(),"Empty input accepted");
            auto broken=p; broken["manifest"]=temporary.filePath("missing.json"); require(!algorithm->validate(input,broken).isEmpty(),"Missing manifest accepted");
        }
        std::cout<<"Plugin tests passed: "<<result.metrics["leftPoints"].toInt()<<" left, "<<result.metrics["rightPoints"].toInt()<<" right, "<<stepped.checkpoints<<" checkpoints.\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n'; return 1;}
}
