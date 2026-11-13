#include <fw/Algorithm.h>
#include <fw/Parameters.h>
#include <fw/Diagnostics.h>
#include "SeamDetector.h"
#include "PointCloudIO.h"
#include "../Configuration.h"
#include "../CloudFrames.h"
#include <QCoreApplication>
#include <QPluginLoader>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QDir>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <iostream>

void require(bool ok,const char* message) {if(!ok) throw std::runtime_error(message);}
struct Context:fw::IExecutionContext,fw::IDebugContext {
    bool debugging=false,cancelled=false; int checkpoints=0,stopAt=0,checks=0,cancelAfter=0;
    fw::FramePtr first;
    fw::IDebugContext& debug() override {return *this;}
    bool enabled() const override {return debugging;}
    bool isCancelled() const override {auto self=const_cast<Context*>(this); return cancelled||(cancelAfter&&++self->checks>=cancelAfter);}
    void log(const QString&) override {}
    void progress(int) override {}
    fw::DebugAction checkpoint(const fw::DebugFrame& frame) override {
        ++checkpoints; if(!first) first=fw::snapshot(frame,16*1024*1024);
        if(stopAt&&checkpoints>=stopAt) {cancelled=true;return fw::DebugAction::Stop;}
        return fw::DebugAction::Continue;
    }
};
void same(const weld::CloudT& a,const weld::CloudT& b) {
    require(a.size()==b.size(),"Point counts differ");
    for(std::size_t i=0;i<a.size();++i) require((weld::toVector(a[i])-weld::toVector(b[i])).norm()==0,"XYZ differ");
}
void compare(const fw::AlgorithmResult& a,const fw::AlgorithmResult& b) {
    require(a.status==fw::ResultStatus::Success&&b.status==a.status,"Debug failed");
    for(int side=0;side<2;++side) same(*fw::toCloud<weld::PointT>(a.outputs[side]),*fw::toCloud<weld::PointT>(b.outputs[side]));
    auto aDiagnostics=a.metrics.value("diagnostics").toObject();aDiagnostics.remove("parameters");
    auto bDiagnostics=b.metrics.value("diagnostics").toObject();bDiagnostics.remove("parameters");
    require(QJsonDocument(aDiagnostics).toJson(QJsonDocument::Compact)==
            QJsonDocument(bDiagnostics).toJson(QJsonDocument::Compact),
            "Debug changed diagnostics");
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        const QString pluginPath=qEnvironmentVariable("EXTRACT2_PLUGIN_PATH",QString::fromUtf8(PLUGIN_PATH));
        QPluginLoader loader(pluginPath); auto plugin=qobject_cast<fw::IAlgorithmPlugin*>(loader.instance());
        require(plugin,qPrintable(loader.errorString())); const auto schema=plugin->parameterSchema();
        auto errors=fw::Parameters::validateSchema(schema); require(errors.isEmpty(),qPrintable(errors.join('\n')));
        errors=fw::Diagnostics::validateProfile(schema.value("diagnostics").toObject(),schema,plugin->descriptor());
        require(errors.isEmpty(),qPrintable(errors.join('\n')));
        auto p=fw::Parameters::defaults(schema);
        require(schema.value("parameterVersion").toInt()==8,"Unexpected parameter version");
        require(p.value("processingMode").toString()=="protected_seams","Default must run protected seams");
        p["processingMode"]="legacy_seams";
        auto old=p;old.remove("interiorOpeningDirection");fw::ParameterSet migrated;QString migrationError;
        require(plugin->migrateParameters(3,old,migrated,migrationError),qPrintable(migrationError));
        require(migrated.value("interiorOpeningDirection").toString()=="auto","V3 migration lost interior default");
        require(migrated.value("useHullEnvelopeCrop").toBool(),"V3 migration lost hull crop default");
        auto v4=p;for(const auto& key:{"useHullEnvelopeCrop","hullEnvelopeGridSize","hullEnvelopeMinCellSupport","hullEnvelopeMinThickness","hullEnvelopeMargin","hullEnvelopeSmoothingRadius"}) v4.remove(key);
        require(plugin->migrateParameters(4,v4,migrated,migrationError),qPrintable(migrationError));
        require(migrated.value("hullEnvelopeGridSize").toDouble()==2.0,"V4 migration lost hull grid default");
        require(!plugin->migrateParameters(2,old,migrated,migrationError),"Unsupported migration accepted");
        const QString mode=argc==4?QString::fromLocal8Bit(argv[1]):QString();
        const bool dataset=mode=="--dataset";
        const bool real=argc==4&&(mode=="--real"||dataset);
        weld::CloudT::Ptr cloud(new weld::CloudT); std::vector<weld::ScanFrameRange> frames;
        if(real) {std::string e;cloud=weld::PointCloudIO::load(argv[2],&e,&frames);require(cloud&&!cloud->empty(),e.c_str());}
        else {
            for(int f=0;f<24;++f) {
                const auto begin=cloud->size();
                for(int j=0;j<=466;++j) {
                    const float y=-35.f+.15f*j,a=std::abs(y);
                    const float z=a<12?20.f+.2f*std::cos(y/5.f):a<22?2.f*(22-a):0.f;
                    cloud->push_back(weld::PointT(2.f*f,y,z+.04f*f));
                }
                frames.push_back({begin,cloud->size()-begin,f});
            }
        }
        auto data=fw::DebugLayer::result("scan",*cloud).data; data.unit="mm";
        if(real&&!frames.empty()) {
            auto framed=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*data.cloud));
            std::vector<int> ids(cloud->size());
            for(const auto& f:frames) std::fill(ids.begin()+f.begin,ids.begin()+f.begin+f.count,f.frameIndex);
            writeFrameIds(*framed,ids); data.cloud=framed;
        }
        fw::AlgorithmInput input{{data},{}}; auto algorithm=plugin->create();
        struct Guard {fw::IAlgorithmPlugin* p;fw::IAlgorithm* a;~Guard(){p->destroy(a);}} guard{plugin,algorithm};
        Context normal; auto result=algorithm->execute(input,p,normal);
        require(result.status==fw::ResultStatus::Success,qPrintable(result.message));require(!normal.checkpoints,"Debug-off checkpoints");
        errors=fw::Diagnostics::validateReport(result.metrics.value("diagnostics").toObject(),
            schema.value("diagnostics").toObject(),plugin->descriptor(),input,p,result);
        require(errors.isEmpty(),qPrintable(errors.join('\n')));
        for(const auto& output:result.outputs) {require(output.unit=="mm","Lost unit");require(fw::hasFields(output,{"confidence","segment_id","rts_shift","innovation_nis","position_sigma","quality_flags","wall_rms","floor_rms"}),"Lost result fields");}
        Context stepped;fw::AlgorithmResult robustResult;QJsonArray stableReports;
        if(!dataset) {
            auto direct=weld::SeamDetector().detect(cloud,configuration(p),nullptr,nullptr,nullptr,real&&!frames.empty()?&frames:nullptr);
            same(*direct.leftSeam,*fw::toCloud<weld::PointT>(result.outputs[0]));same(*direct.rightSeam,*fw::toCloud<weld::PointT>(result.outputs[1]));
            stepped.debugging=true;compare(result,algorithm->execute(input,p,stepped)); require(stepped.checkpoints>=8,"Missing stages");
            auto robust=p;robust["postprocessMode"]="robust_rts";Context robustContext;
            robustResult=algorithm->execute(input,robust,robustContext);
            require(robustResult.status==fw::ResultStatus::Success,qPrintable(robustResult.message));
            require(robustResult.metrics.value("postprocessMode").toString()=="robust_rts","RTS mode not reported");
            errors=fw::Diagnostics::validateReport(robustResult.metrics.value("diagnostics").toObject(),
                schema.value("diagnostics").toObject(),plugin->descriptor(),input,robust,robustResult);
            require(errors.isEmpty(),qPrintable(errors.join('\n')));
        }
        if(!dataset) for(double bin:{0.8,1.2,1.6}) {
            auto stable=p;stable["profileSamplingMode"]="stable_grid";
            stable["profileBinSize"]=bin;stable["profileReferenceScale"]=1.2;
            Context stableContext;auto stableResult=algorithm->execute(input,stable,stableContext);
            require(stableResult.status==fw::ResultStatus::Success,qPrintable(stableResult.message));
            require(stableResult.metrics.value("profileSamplingMode").toString()=="stable_grid","Stable profile mode not reported");
            errors=fw::Diagnostics::validateReport(stableResult.metrics.value("diagnostics").toObject(),
                schema.value("diagnostics").toObject(),plugin->descriptor(),input,stable,stableResult);
            require(errors.isEmpty(),qPrintable(errors.join('\n')));
            auto stableReport=stableResult.metrics;stableReport["profileBinSize"]=bin;
            stableReports.append(stableReport);
        }
        if(!dataset) {
            auto fallback=p;fallback["hullEnvelopeMinThickness"]=1000.0;
            Context fallbackContext;auto fallbackResult=algorithm->execute(input,fallback,fallbackContext);
            require(fallbackResult.status==fw::ResultStatus::Success,qPrintable(fallbackResult.message));
            require(!fallbackResult.metrics.value("hullEnvelopeApplied").toBool(),"Unavailable hull envelope was reported as applied");
            const auto fallbackReport=fallbackResult.metrics.value("diagnostics").toObject();
            errors=fw::Diagnostics::validateReport(fallbackReport,schema.value("diagnostics").toObject(),
                plugin->descriptor(),input,fallback,fallbackResult);
            require(errors.isEmpty(),qPrintable(errors.join('\n')));
            bool foundHullIssue=false;
            for(const auto& issue:fallbackReport.value("issues").toArray())
                foundHullIssue|=issue.toObject().value("code").toString()=="hull_envelope_unavailable";
            require(foundHullIssue,"Hull-envelope fallback diagnostic missing");
        }
        if(!real) {
            auto frameWindowConfig=configuration(p);frameWindowConfig.voxelSize=0;frameWindowConfig.outlierMeanK=0;
            weld::DebugTrace frameWindowTrace;
            const auto windowed=weld::SeamDetector().detect(cloud,frameWindowConfig,nullptr,nullptr,&frameWindowTrace,&frames);
            require(windowed.ok(),windowed.summary.c_str());require(frameWindowTrace.slices.size()==frames.size(),"Frame windows lost target frames");
            require(frameWindowTrace.slices.front().raw.size()==3*std::size_t(467),"First frame did not use 3-frame window");
            require(frameWindowTrace.slices[1].raw.size()==4*std::size_t(467),"Second frame did not use 4-frame window");
            require(frameWindowTrace.slices[2].raw.size()==5*std::size_t(467),"Middle frame did not use 5-frame window");
            require(frameWindowTrace.slices.back().raw.size()==3*std::size_t(467),"Last frame did not use 3-frame window");
            // 整船纵向—竖向投影的主体具有连续厚度；同样密集但只有一格
            // 厚的外部扫描拖影不应扩张到底部包络内，也不能改变后续轨迹。
            auto hullConfig=configuration(p);hullConfig.voxelSize=0;hullConfig.outlierMeanK=0;
            const auto cleanHull=weld::SeamDetector().detect(cloud,hullConfig);
            weld::CloudT::Ptr hullWithNoise(new weld::CloudT(*cloud));
            for(int f=0;f<24;++f)
                for(int j=0;j<50;++j)
                    hullWithNoise->push_back(weld::PointT(2.f*f,-25.f+.1f*j,-25.f));
            const auto croppedHull=weld::SeamDetector().detect(hullWithNoise,hullConfig);
            require(cleanHull.ok()&&croppedHull.ok(),"Hull-envelope regression did not produce seams");
            require(croppedHull.hullEnvelopeApplied,"Hull envelope was not applied");
            require(croppedHull.hullEnvelopeInputPoints>croppedHull.hullEnvelopeRetainedPoints,
                    "Hull envelope did not remove exterior scan trails");
            for(const auto& point:*croppedHull.filteredCloud)
                require(point.z>-10.f,"Exterior scan trail survived hull envelope");
            same(*cleanHull.leftSeam,*croppedHull.leftSeam);
            same(*cleanHull.rightSeam,*croppedHull.rightSeam);
            // 两组凹向相反的三面结构同时存在时，明确的内部方向必须在路径
            // 优化前排除另一组；这验证的是筛选语义，不把自动选择当作真值。
            weld::CloudT::Ptr mixed(new weld::CloudT(*cloud));
            for(int f=0;f<24;++f) for(int j=0;j<=466;++j) {
                const float local=-35.f+.15f*j,a=std::abs(local),y=100.f+local;
                const float z=a<12?-20.f-.2f*std::cos(local/5.f):a<22?-2.f*(22-a):0.f;
                mixed->push_back(weld::PointT(2.f*f,y,z+.04f*f));
            }
            auto centerOf=[&](const weld::SeamResult& value) {
                double sum=0;std::size_t count=0;
                for(const auto& point:*value.leftSeam) {sum+=point.y;++count;}
                for(const auto& point:*value.rightSeam) {sum+=point.y;++count;}
                return count?sum/double(count):0.0;
            };
            auto insideConfig=configuration(p);insideConfig.useHullEnvelopeCrop=false;
            insideConfig.interiorOpeningMode=weld::InteriorOpeningMode::NegativeV;
            const auto inside=weld::SeamDetector().detect(mixed,insideConfig);
            require(inside.ok(),inside.summary.c_str());require(inside.interiorOpeningDirection==-1,"Negative-V interior not selected");
            require(std::abs(centerOf(inside))<30.0,"Exterior structure contaminated Negative-V result");
            auto outsideConfig=insideConfig;outsideConfig.interiorOpeningMode=weld::InteriorOpeningMode::PositiveV;
            const auto outside=weld::SeamDetector().detect(mixed,outsideConfig);
            require(outside.ok(),outside.summary.c_str());require(outside.interiorOpeningDirection==1,"Positive-V structure not selected");
            require(centerOf(outside)>70.0,"Interior structure contaminated Positive-V result");

            // 沿纵向加入局部高斯凸起。固定 6 mm 平面会跨越明显曲率；自适应
            // 窗口应在至少一个凸起站位缩小，同时仍通过实测底面交点约束。
            weld::CloudT::Ptr curved(new weld::CloudT);
            for(int f=0;f<40;++f) {
                const float x=float(f),bump=1.8f*std::exp(-.5f*std::pow((x-20.f)/2.f,2.f));
                for(int j=0;j<=466;++j) {
                    const float y=-35.f+.15f*j,a=std::abs(y);
                    const float z=a<12?20.f+.2f*std::cos(y/5.f):a<22?2.f*(22-a):0.f;
                    curved->push_back(weld::PointT(x,y,z+bump));
                }
            }
            auto curvedConfig=insideConfig;
            curvedConfig.floorPatchMaxRms=0.15f;
            const auto curvedResult=weld::SeamDetector().detect(curved,curvedConfig);
            require(curvedResult.ok(),curvedResult.summary.c_str());
            bool adaptiveAccepted=false;
            for(const auto* side:{&curvedResult.leftPointQuality,&curvedResult.rightPointQuality})
                for(const auto& quality:*side)
                    adaptiveAccepted|=quality.floorAccepted&&quality.floorPatchSizeUsed>0&&
                        quality.floorPatchSizeUsed<curvedConfig.floorPatchSize;
            require(adaptiveAccepted,"Curved floor never used an accepted adaptive patch");
        }
        if(!dataset) {
            Context stopped;stopped.debugging=true;stopped.stopAt=2;
            require(algorithm->execute(input,p,stopped).status==fw::ResultStatus::Cancelled,"Debug stop failed");
            Context early;early.cancelled=true;require(algorithm->execute(input,p,early).status==fw::ResultStatus::Cancelled,"Initial stop failed");
            Context during;during.cancelAfter=8;require(algorithm->execute(input,p,during).status==fw::ResultStatus::Cancelled,"Normal-run cancellation failed");
        }
        require(!algorithm->validate({},p).isEmpty(),"Empty accepted");auto bad=p;bad["sliceStep"]=0;
        require(!algorithm->validate(input,bad).isEmpty(),"Invalid parameter accepted");
        if(!real) {
            Context detail;detail.debugging=true;auto all=p;all["debug_detail"]="all";compare(result,algorithm->execute(input,all,detail));
            require(detail.checkpoints>stepped.checkpoints,"Missing detailed steps");
            auto framed=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*data.cloud));std::vector<int> ids(cloud->size());
            for(const auto& f:frames) std::fill(ids.begin()+f.begin,ids.begin()+f.begin+f.count,f.frameIndex);
            writeFrameIds(*framed,ids);input.clouds[0].cloud=framed;
            Context frameContext;auto frameResult=algorithm->execute(input,p,frameContext);
            require(frameResult.status==fw::ResultStatus::Success,qPrintable(frameResult.message));
            auto expected=weld::SeamDetector().detect(cloud,configuration(p),nullptr,nullptr,nullptr,&frames);
            same(*expected.leftSeam,*fw::toCloud<weld::PointT>(frameResult.outputs[0]));same(*expected.rightSeam,*fw::toCloud<weld::PointT>(frameResult.outputs[1]));
            require(fw::hasFields(frameResult.outputs[0],{"frame_index"}),"Lost frame IDs");
            auto reread=readFrames(*framed); require(reread.size()==frames.size(),"Frame roundtrip failed");
            int badId=-1;std::memcpy(framed->data.data()+framed->fields.back().offset,&badId,4);
            require(!algorithm->validate(input,p).isEmpty(),"Invalid frame accepted");
            auto snapshotBefore=stepped.first->layers.front().data.cloud->data;
            cloud->clear();require(stepped.first->layers.front().data.cloud->data==snapshotBefore,"Snapshot mutated");
        } else {
            const QString dir=QString::fromLocal8Bit(argv[3]);QDir().mkpath(dir);
            pcl::PCDWriter writer;
            for(int i=0;i<2;++i) require(writer.writeBinary(QDir(dir).filePath(i?"right.pcd":"left.pcd").toLocal8Bit().toStdString(),*result.outputs[i].cloud)==0,"Write result failed");
            if(!dataset) for(int i=0;i<2;++i) require(writer.writeBinary(QDir(dir).filePath(i?"robust_right.pcd":"robust_left.pcd").toLocal8Bit().toStdString(),*robustResult.outputs[i].cloud)==0,"Write robust result failed");
            auto report=result.metrics;report["inputPoints"]=double(cloud->size());report["inputFrames"]=double(frames.size());
            if(!dataset) {report["checkpoints"]=stepped.checkpoints;report["debugMatchesNormal"]=true;report["directMatchesPlugin"]=true;report["robust"]=robustResult.metrics;report["stableGrid"]=stableReports;}
            QFile f(QDir(dir).filePath("report.json"));require(f.open(QIODevice::WriteOnly),"Report write failed");f.write(QJsonDocument(report).toJson());
        }
        std::cout<<"Passed: "<<result.metrics["leftPoints"].toInt()<<" left, "<<result.metrics["rightPoints"].toInt()<<" right; "<<stepped.checkpoints<<" stages.\n";
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
