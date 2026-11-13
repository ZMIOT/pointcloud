#include "HullSurfaces.h"
#include "PointCloudIO.h"
#include "../CloudFrames.h"
#include <fw/Diagnostics.h>
#include <fw/Parameters.h>
#include <QCoreApplication>
#include <QPluginLoader>
#include <QFile>
#include <QJsonDocument>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cstring>
#include <set>
#include <random>

void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
struct Context:fw::IExecutionContext,fw::IDebugContext {
  bool debugOn=false,stop=false;int frames=0;
  fw::IDebugContext& debug()override{return *this;}
  bool enabled()const override{return debugOn;}
  bool isCancelled()const override{return stop;}
  void log(const QString&)override{}
  void progress(int)override{}
  fw::DebugAction checkpoint(const fw::DebugFrame&)override{++frames;return fw::DebugAction::Continue;}
};

int main(int argc,char **argv) {
  QCoreApplication app(argc,argv);
  try {
    const bool real=argc==3;
    require(real||argc==1,"Usage: SurfaceTests [input output.pcd]");
    weld::CloudT::Ptr input(new weld::CloudT),roughForPlugin;std::size_t truthCount=0;
    if(real){std::string error;input=weld::PointCloudIO::load(argv[1],&error);require(bool(input),error.c_str());}
    else {
      // A rounded U section: two walls and a floor are one known body.
      for(float x=0;x<=100;x+=.5f) {
        for(float y=-25;y<=25;y+=.5f)input->push_back({x,y,0});
        for(int side:{-1,1}) {
          for(float angle=0;angle<=1.570797f;angle+=.05f)
            input->push_back({x,side*(25+5*std::sin(angle)),5-5*std::cos(angle)});
          for(float z=5;z<=30;z+=.5f)input->push_back({x,side*30.f,z});
        }
      }
      truthCount=input->size();
      // Smaller disjoint plane, oversampled so raw point count cannot rank it.
      for(int repeat=0;repeat<100;++repeat)for(int x=0;x<=20;++x)for(int y=0;y<=20;++y)
        input->push_back({float(x),float(y),-20.f});
    }
    weld::DetectorConfig config;
    auto result=weld::extractHullSurfaces(input,config);
    require(result.ok(),result.error.c_str());
    std::cout<<"input="<<input->size()<<" samples="<<result.analysisPoints<<" kept="<<result.cloud->size()<<" regions=";
    for(std::size_t i=0;i<std::min<std::size_t>(10,result.regionSizes.size());++i)std::cout<<result.regionSizes[i]<<',';
    std::cout<<" patches="<<result.referencePatches<<" recovered="<<result.recoveredIndices.size()<<" nearSurfaceRejected="<<result.rejectedNearSurfacePoints<<std::endl;
    std::set<std::size_t> unique;
    for(std::size_t i=0;i<result.sourceIndices.size();++i) {
      const auto index=result.sourceIndices[i];require(unique.insert(index).second,"Duplicated source point");
      require((weld::toVector((*input)[index])-weld::toVector((*result.cloud)[i])).norm()==0,"Modified raw XYZ");
      if(!real)require(index<truthCount,"Retained smaller interference component");
    }
    if(!real) {
      require(result.sourceIndices.size()>.95*truthCount,"Synthetic body lost more than 5 percent of measured points");
      auto bad=config;bad.hullSurfaceVoxelSize=0;require(!weld::extractHullSurfaces(input,bad).ok(),"Accepted zero grid");
      require(!weld::extractHullSurfaces({},config).ok(),"Accepted null cloud");
      bool cancelled=false;try{weld::extractHullSurfaces(input,config,[]{return true;});}catch(const weld::ExecutionCancelled&){cancelled=true;}
      require(cancelled,"Core cancellation ignored");
      auto shuffled=weld::CloudT::Ptr(new weld::CloudT(*input));std::mt19937 random(12);std::shuffle(shuffled->begin(),shuffled->end(),random);
      auto again=weld::extractHullSurfaces(shuffled,config);require(again.ok(),"Reordered cloud failed");
      require(std::abs(double(again.cloud->size())-double(result.cloud->size()))<.01*truthCount,"Input order changed body selection");
      for(const auto& point:*again.cloud)require(point.z>-10,"Reordered cloud selected interference");
      // Raw floor measurements remain present, but a volumetric scan trail
      // destroys local normal support across a long stripe. Recover the floor
      // from neighbouring bounded charts, never by synthesising fill points.
      auto disturbed=weld::CloudT::Ptr(new weld::CloudT(*input));
      for(float x=30;x<=70;x+=1)for(float y=-4;y<=4;y+=1)for(float z=1;z<=7;z+=1)
        disturbed->push_back({x,y,z});
      auto protectedFloor=weld::extractHullSurfaces(disturbed,config);require(protectedFloor.ok(),"Disturbed floor failed");
      std::set<std::size_t> kept(protectedFloor.sourceIndices.begin(),protectedFloor.sourceIndices.end());
      int stripeTotal=0,stripeKept=0;
      for(std::size_t i=0;i<truthCount;++i){const auto& point=(*input)[i];if(point.x>=35&&point.x<=65&&std::abs(point.y)<=3&&point.z==0){++stripeTotal;stripeKept+=kept.count(i)!=0;}}
      std::cout<<"labelled floor stripe="<<stripeKept<<'/'<<stripeTotal<<" recovered="<<protectedFloor.recoveredIndices.size()<<std::endl;
      require(stripeKept>.95*stripeTotal,"Lost measured floor stripe under noisy normal estimates");
      require(!protectedFloor.recoveredIndices.empty(),"Recovery regression did not exercise patch support");
      // A measured weld-toe deformation is not an outlier just because it
      // exceeds the reference-surface fit tolerance. Strict cleanup must be a
      // subset; preserve original coordinates, never project/fill the strip.
      auto rough=weld::CloudT::Ptr(new weld::CloudT(*input));std::vector<std::size_t> toe;
      roughForPlugin=rough;
      for(std::size_t i=0;i<truthCount;++i){auto& point=(*rough)[i];
        if(point.x>=20&&point.x<=80&&point.y>=21&&point.y<=25&&point.z==0){
          point.z-=1.5f*std::sin((point.y-21.f)*3.14159265f/4.f)*std::cos(2.f*point.x);toe.push_back(i);}}
      auto strictConfig=config;strictConfig.hullPreservationMargin=0;
      auto strict=weld::extractHullSurfaces(rough,strictConfig),preserved=weld::extractHullSurfaces(rough,config);
      require(strict.ok()&&preserved.ok(),"Rough measured junction failed");
      std::set<std::size_t> strictIds(strict.sourceIndices.begin(),strict.sourceIndices.end()),preservedIds(preserved.sourceIndices.begin(),preserved.sourceIndices.end());
      int strictToe=0,keptToe=0;for(auto i:toe){strictToe+=strictIds.count(i)!=0;keptToe+=preservedIds.count(i)!=0;}
      std::cout<<"measured rough toe strict/protected/total="<<strictToe<<'/'<<keptToe<<'/'<<toe.size()<<std::endl;
      require(keptToe>.98*toe.size()&&keptToe>strictToe,"Preservation band did not protect the measured rough junction");
      for(auto i:strict.sourceIndices)require(preservedIds.count(i)!=0,"Protection removed strict baseline points");
      require(!preserved.protectedUncertainIndices.empty(),"Missing uncertainty accounting");
      for(auto i:preserved.protectedUncertainIndices)require(preservedIds.count(i)&&!strictIds.count(i),"Uncertain point classification inconsistent");
      // Actual input holes must remain holes (no output point may be generated).
      auto missing=weld::CloudT::Ptr(new weld::CloudT);
      for(const auto& point:*input)if(!(point.z==0&&point.x>40&&point.x<50&&std::abs(point.y)<3))missing->push_back(point);
      auto missingResult=weld::extractHullSurfaces(missing,config);require(missingResult.ok(),"Missing-input case failed");
      for(const auto& point:*missingResult.cloud)require(!(point.z==0&&point.x>40&&point.x<50&&std::abs(point.y)<3),"Invented points inside an input hole");
      auto parallel=weld::CloudT::Ptr(new weld::CloudT(*input));
      for(auto& point:*parallel)if(point.z==-20)point.z=-6;
      auto parallelResult=weld::extractHullSurfaces(parallel,config);require(parallelResult.ok(),"Nearby parallel plane case failed");
      for(auto index:parallelResult.sourceIndices)require(index<truthCount,"Recovered a separate parallel interference plane");
    }
    QPluginLoader loader(qEnvironmentVariable("EXTRACT2_PLUGIN_PATH",QString::fromUtf8(PLUGIN_PATH)));
    auto plugin=qobject_cast<fw::IAlgorithmPlugin*>(loader.instance());require(plugin,qPrintable(loader.errorString()));
    const auto schema=plugin->parameterSchema();auto parameters=fw::Parameters::defaults(schema);
    parameters["processingMode"]="surfaces_only";
    auto errors=fw::Diagnostics::validateProfile(schema["diagnostics"].toObject(),schema,plugin->descriptor());require(errors.isEmpty(),qPrintable(errors.join('\n')));
    auto data=fw::DebugLayer::result("surface input",*input).data;data.unit="mm";data.coordinateFrame="test_frame";
    auto framed=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*data.cloud));
    std::vector<int> frameIds(input->size());for(std::size_t i=0;i<frameIds.size();++i)frameIds[i]=int(i/100);
    writeFrameIds(*framed,frameIds);data.cloud=framed;
    fw::AlgorithmInput request{{data},{}};auto algorithm=plugin->create();
    struct Guard{fw::IAlgorithmPlugin* p;fw::IAlgorithm* a;~Guard(){p->destroy(a);}}guard{plugin,algorithm};
    Context context;context.debugOn=!real;auto output=algorithm->execute(request,parameters,context);
    require(output.status==fw::ResultStatus::Success,qPrintable(output.message));
    require(output.outputs.size()==1&&output.outputs[0].id=="hull_body","Must output only largest body");
    require(output.layers.size()==1,"Final display must not contain rejected input or seams");
    require(real||context.frames==1,"Unexpected downstream checkpoints");
    errors=fw::Diagnostics::validateReport(output.metrics["diagnostics"].toObject(),schema["diagnostics"].toObject(),plugin->descriptor(),request,parameters,output);
    require(errors.isEmpty(),qPrintable(errors.join('\n')));
    const auto& actual=*output.outputs[0].cloud;
    require(actual.width==result.sourceIndices.size()&&actual.point_step==framed->point_step,"Lost measured points or fields");
    for(std::size_t i=0;i<result.sourceIndices.size();++i)
      require(std::memcmp(actual.data.data()+i*actual.point_step,framed->data.data()+result.sourceIndices[i]*framed->point_step,actual.point_step)==0,"Modified input record/custom field");
    Context stopped;stopped.stop=true;require(algorithm->execute(request,parameters,stopped).status==fw::ResultStatus::Cancelled,"Plugin cancellation ignored");
    auto malformed=request;auto truncated=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*data.cloud));truncated->data.resize(1);malformed.clouds[0].cloud=truncated;
    require(!algorithm->validate(malformed,parameters).isEmpty(),"Truncated input accepted");
    auto v5=parameters;for(const auto& key:{"processingMode","hullSurfaceVoxelSize","hullSurfaceCurvature","hullSurfaceDistance"})v5.remove(key);
    fw::ParameterSet migrated;QString migrationError;require(plugin->migrateParameters(5,v5,migrated,migrationError),"V5 migration failed");
    require(migrated==parameters,"Migration must select first stage and complete parameters");
    if(roughForPlugin) {
      auto roughData=fw::DebugLayer::result("measured rough junction",*roughForPlugin).data;roughData.unit="mm";
      fw::AlgorithmInput roughRequest{{roughData},{}};Context normal,debug;debug.debugOn=true;
      const auto a=algorithm->execute(roughRequest,parameters,normal),b=algorithm->execute(roughRequest,parameters,debug);
      require(a.status==fw::ResultStatus::Success&&b.status==fw::ResultStatus::Success,"Protected rough plugin run failed");
      require(a.metrics["protectedUncertainPoints"].toDouble()>0,"Plugin did not report uncertain protection");
      require(a.metrics==b.metrics&&a.outputs[0].cloud->data==b.outputs[0].cloud->data,"Debug changed protected points/diagnostic facts");
      errors=fw::Diagnostics::validateReport(a.metrics["diagnostics"].toObject(),schema["diagnostics"].toObject(),plugin->descriptor(),roughRequest,parameters,a);
      require(errors.isEmpty(),qPrintable(errors.join('\n')));
    }
    if(real) {
      QFile report(QString::fromLocal8Bit(argv[2])+".report.json");
      require(report.open(QIODevice::WriteOnly),"Could not save diagnostic report");
      require(report.write(QJsonDocument(output.metrics).toJson())>0,"Could not write diagnostic report");
      require(pcl::io::savePCDFileBinary(argv[2],*result.cloud)==0,"Save failed");
      weld::CloudT recovered,rejected;
      for(auto index:result.recoveredIndices)recovered.push_back((*input)[index]);
      std::size_t keptIndex=0;
      for(std::size_t i=0;i<input->size();++i) {if(keptIndex<result.sourceIndices.size()&&result.sourceIndices[keptIndex]==i)++keptIndex;else rejected.push_back((*input)[i]);}
      if(!recovered.empty())pcl::io::savePCDFileBinary(std::string(argv[2])+".recovered.pcd",recovered);
      if(!rejected.empty())pcl::io::savePCDFileBinary(std::string(argv[2])+".rejected.pcd",rejected);
      for(int side=0;side<3;++side){weld::CloudT surface;for(std::size_t i=0;i<result.cloud->size();++i)if(result.surfaceIds[i]==side)surface.push_back((*result.cloud)[i]);if(!surface.empty())pcl::io::savePCDFileBinary(std::string(argv[2])+"."+std::to_string(side)+".pcd",surface);}
    }
    std::cout<<"PASS: largest body, raw fields, one-stage output, diagnostics, cancellation"<<std::endl;return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<std::endl;return 1;}
}
