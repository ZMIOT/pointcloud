#include <fw/Algorithm.h>
#include <fw/Parameters.h>
#include <fw/Diagnostics.h>
#include "SeamDetector.h"
#include "PointCloudIO.h"
#include "../Configuration.h"
#include "../CloudFrames.h"
#include <QCoreApplication>
#include <QPluginLoader>
#include <QJsonDocument>
#include <QJsonArray>
#include <QFile>
#include <QDir>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <set>

void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
struct Context:fw::IExecutionContext,fw::IDebugContext {
  bool debugOn=false,stop=false;int checkpoints=0;
  fw::IDebugContext& debug()override{return *this;}
  bool enabled()const override{return debugOn;}
  bool isCancelled()const override{return stop;}
  void log(const QString& s)override{std::cout<<s.toStdString()<<std::endl;}
  void progress(int)override{}
  fw::DebugAction checkpoint(const fw::DebugFrame&)override{++checkpoints;return stop?fw::DebugAction::Stop:fw::DebugAction::Continue;}
};
int main(int argc,char** argv){
 QCoreApplication app(argc,argv);
 try{
  QPluginLoader loader(qEnvironmentVariable("EXTRACT2_PLUGIN_PATH",QString::fromUtf8(PLUGIN_PATH)));
  auto plugin=qobject_cast<fw::IAlgorithmPlugin*>(loader.instance());require(plugin,qPrintable(loader.errorString()));
  const auto schema=plugin->parameterSchema();auto p=fw::Parameters::defaults(schema);
  require(p["processingMode"]=="protected_seams","Full mode must be default");
  auto errors=fw::Diagnostics::validateProfile(schema["diagnostics"].toObject(),schema,plugin->descriptor());require(errors.isEmpty(),qPrintable(errors.join('\n')));
  auto old=p;old["processingMode"]="surfaces_only";const auto expectedMigrated=old;
  old.remove("protectedProfileBinSize");old.remove("protectedMinSeamSeparation");fw::ParameterSet migrated;QString error;
  require(plugin->migrateParameters(6,old,migrated,error)&&migrated==expectedMigrated,"Must preserve explicit V6 surface-only preset and supply new defaults");
  auto v7=p;v7.remove("hullPreservationMargin");
  require(plugin->migrateParameters(7,v7,migrated,error)&&migrated==p,"V7 migration must add protection margin without changing mode");
  const bool real=argc==3;
  weld::CloudT::Ptr cloud(new weld::CloudT);std::vector<weld::ScanFrameRange> frames;
  if(real){std::string e;cloud=weld::PointCloudIO::load(argv[1],&e,&frames);require(cloud&&!cloud->empty(),e.c_str());}
  else for(int f=0;f<161;++f){
   auto begin=cloud->size();
   for(int j=0;j<=280;++j){float y=-35.f+.25f*j;cloud->push_back({.5f*f,y,2.f*std::max(0.f,std::abs(y)-20.f)});}
   frames.push_back({begin,cloud->size()-begin,100+f});
  }
  auto inputData=fw::DebugLayer::result("full input",*cloud).data;inputData.unit="mm";inputData.coordinateFrame="scan";
  if(!frames.empty()){
   auto raw=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*inputData.cloud));std::vector<int> ids(cloud->size());
   for(const auto& f:frames)std::fill(ids.begin()+f.begin,ids.begin()+f.begin+f.count,f.frameIndex);
   writeFrameIds(*raw,ids);inputData.cloud=raw;
  }
  fw::AlgorithmInput input{{inputData},{}};auto algorithm=plugin->create();
  struct Guard{fw::IAlgorithmPlugin* p;fw::IAlgorithm* a;~Guard(){p->destroy(a);}}guard{plugin,algorithm};
  Context context;auto result=algorithm->execute(input,p,context);
  require(result.status==fw::ResultStatus::Success,qPrintable(result.message));
  require(result.outputs.size()==3&&result.outputs[0].id=="left_seam"&&result.outputs[1].id=="right_seam"&&result.outputs[2].id=="hull_body","Wrong output IDs");
  errors=fw::Diagnostics::validateReport(result.metrics["diagnostics"].toObject(),schema["diagnostics"].toObject(),plugin->descriptor(),input,p,result);
  require(errors.isEmpty(),qPrintable(errors.join('\n')));
  require(!result.metrics["secondaryFilteringApplied"].toBool()&&!result.metrics["hullEnvelopeApplied"].toBool(),"Protected floor filtered twice");
  for(int side=0;side<2;++side){
   const auto& out=result.outputs[side];require(fw::hasFields(out,{"quality_flags","confidence","segment_id","wall_rms","floor_rms"}),"Missing seam evidence");
   if(!frames.empty())require(fw::hasFields(out,{"frame_index"}),"Lost frame IDs");
   const auto points=fw::toCloud<weld::PointT>(out);
   for(const auto& point:*points)require(std::isfinite(point.x)&&std::isfinite(point.y)&&std::isfinite(point.z),"Nonfinite output");
   if(!real){
    require(points->size()>150,"Lost synthetic seam length");
    for(const auto& point:*points)require(std::abs(point.y-(side?20.f:-20.f))<1&&std::abs(point.z)<1,"Synthetic ground-truth seam error > 1 mm");
   }
  }
  if(!real){
   auto surfaces=p;surfaces["processingMode"]="surfaces_only";Context c;
   const auto first=algorithm->execute(input,surfaces,c);require(first.status==fw::ResultStatus::Success,"Surface comparison failed");
   require(first.outputs[0].cloud->data==result.outputs[2].cloud->data,"Full route changed protected original records");
   Context debug;debug.debugOn=true;auto stepped=algorithm->execute(input,p,debug);
   require(stepped.status==fw::ResultStatus::Success&&debug.checkpoints>8,"Missing full checkpoints");
   for(int i=0;i<3;++i)require(stepped.outputs[i].cloud->data==result.outputs[i].cloud->data,"Debug changed full result");
   require(stepped.metrics==result.metrics,"Debug changed diagnostic facts");
   Context stopped;stopped.stop=true;require(algorithm->execute(input,p,stopped).status==fw::ResultStatus::Cancelled,"Cancellation ignored");
   // A gap with no three-surface support: keep both supported runs. Neither
   // regularization nor interpolation may move endpoints across this gap.
   weld::CloudT::Ptr gap(new weld::CloudT);std::vector<weld::ScanFrameRange> ranges;
   for(const auto& f:frames){if(f.frameIndex>160&&f.frameIndex<180)continue;auto begin=gap->size();
    gap->insert(gap->end(),cloud->begin()+f.begin,cloud->begin()+f.begin+f.count);ranges.push_back({begin,f.count,f.frameIndex});}
   auto config=configuration(p);config.protectedHullInput=true;
   const auto split=weld::SeamDetector().detect(gap,config,nullptr,nullptr,nullptr,&ranges);
   require(split.ok()&&split.leftSegmentStarts.size()==2&&split.rightSegmentStarts.size()==2,"Must preserve two reliable runs");
   for(int id:split.leftFrameIndices)require(id<=160||id>=180,"Invented absent frame in long gap");
   const auto spatial=weld::SeamDetector().detect(gap,config);
   require(spatial.ok()&&spatial.leftSegmentStarts.size()==2&&spatial.rightSegmentStarts.size()==2,"Spatial sections must retain both supported runs");
   // Curved scan trajectory: every original frame has a different plane.
   // Ground-truth seams are known in world XYZ, not merely in flattened YZ.
   weld::CloudT::Ptr curved(new weld::CloudT);std::vector<weld::ScanFrameRange> curvedFrames;
   std::vector<Eigen::Vector3f> centers;
   for(int f=0;f<100;++f){
    const float angle=-.35f+.014f*f;
    const Eigen::Vector3f center(60.f*std::sin(angle),0,60.f*(1-std::cos(angle)));
    const Eigen::Vector3f vertical(-std::sin(angle),0,std::cos(angle));centers.push_back(center);
    const auto begin=curved->size();
    for(int j=0;j<=280;++j){const float y=-35.f+.25f*j,z=2.f*std::max(0.f,std::abs(y)-20.f);
     curved->push_back(weld::toPoint(center+Eigen::Vector3f(0,y,0)+vertical*z));}
    curvedFrames.push_back({begin,curved->size()-begin,f});
   }
   const auto curvedResult=weld::SeamDetector().detect(curved,config,nullptr,nullptr,nullptr,&curvedFrames);
   require(curvedResult.ok()&&curvedResult.leftSeam->size()>95,"Curved original-frame coverage lost");
   for(int side=0;side<2;++side){const auto& points=side?*curvedResult.rightSeam:*curvedResult.leftSeam;
    const auto& ids=side?curvedResult.rightFrameIndices:curvedResult.leftFrameIndices;
    for(std::size_t i=0;i<points.size();++i)
     require((weld::toVector(points[i])-centers[ids[i]]-Eigen::Vector3f(0,side?20.f:-20.f,0)).norm()<1.f,"Tilted-frame inverse transform or seam error > 1 mm");
   }
  }else{
   QDir dir(QString::fromLocal8Bit(argv[2]));require(dir.mkpath("."),"Output folder failed");pcl::PCDWriter writer;
   for(const auto& out:result.outputs)require(writer.writeBinary(dir.filePath(out.id+".pcd").toStdString(),*out.cloud)==0,"PCD write failed");
   QFile file(dir.filePath("report.json"));require(file.open(QIODevice::WriteOnly),"Report write failed");
   auto report=result.metrics;report["message"]=result.message;report["originalFrameCount"]=double(frames.size());file.write(QJsonDocument(report).toJson());
  }
  std::cout<<result.message.toStdString()<<"\nFull seam tests passed.\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
