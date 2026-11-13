// Host trial-channel regression: no createWindow(), no formal run/list writes.
#include "Runtime.h"
#include <fw/Parameters.h>
#include <fw/Diagnostics.h>
#include <QCoreApplication>
#include <QTimer>
#include <QJsonDocument>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <iostream>
int main(int argc,char** argv){
 QCoreApplication app(argc,argv);if(argc!=4)return 2;
 fw::Runtime runtime;QString error;const QString id="weld.extract_feature2";
 if(!runtime.initialize(QDir(argv[1]).filePath("plugins"),error)){std::cerr<<error.toStdString();return 3;}
 auto cloud=runtime.readCloud(QString::fromLocal8Bit(argv[2]),"mm",error);
 if(!error.isEmpty()){std::cerr<<error.toStdString();return 4;}
 fw::AlgorithmInput input{{cloud},{}};const auto schema=runtime.schema(id);auto p=fw::Parameters::defaults(schema);
 bool formalSignal=false;QObject::connect(&runtime,&fw::Host::finished,&app,[&](fw::AlgorithmResult){formalSignal=true;});
 QObject::connect(&runtime,&fw::Runtime::trialFinished,&app,[&](fw::AlgorithmResult result){
  QStringList errors;fw::AlgorithmDescriptor descriptor;
  for(const auto& d:runtime.algorithms())if(d.id==id)descriptor=d;
  errors=fw::Diagnostics::validateReport(result.metrics["diagnostics"].toObject(),schema["diagnostics"].toObject(),descriptor,input,p,result);
  bool saved=true;const QDir outputDirectory=QFileInfo(QString::fromLocal8Bit(argv[3])).absoluteDir();
  for(const auto& out:result.outputs)if(!runtime.writeCloud(outputDirectory.filePath(out.id+".pcd"),out,error)){saved=false;std::cerr<<error.toStdString();}
  const bool success=result.status==fw::ResultStatus::Success&&result.outputs.size()==3&&errors.isEmpty()&&!formalSignal&&saved;
  QJsonObject report=result.metrics;report["trialChannelSuccess"]=success;report["formalSignalEmitted"]=formalSignal;report["message"]=result.message;
  QFile file(QString::fromLocal8Bit(argv[3]));if(!file.open(QIODevice::WriteOnly)){app.exit(6);return;}file.write(QJsonDocument(report).toJson());
  std::cout<<result.message.toStdString()<<'\n'<<errors.join('\n').toStdString()<<std::endl;app.exit(success?0:5);
 });
 QTimer::singleShot(0,&app,[&]{if(!runtime.startTrial(id,input,p,error)){std::cerr<<error.toStdString();app.exit(7);}});
 QTimer::singleShot(600000,&app,[&]{runtime.stop();app.exit(8);});
 int code=app.exec();runtime.shutdown();return code;
}
