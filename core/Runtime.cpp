#include "Runtime.h"
#include <fw/Parameters.h>
#include <fw/Diagnostics.h>
#include <fw/BuildInfo.h>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QRegExp>
#include <QSaveFile>
#include <QSettings>
#include <QTextStream>
#include <QUuid>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace {
pcl::PCLPointCloud2::Ptr loadTextCloud(const QString& file, QString& error) {
    QFile input(file);
    if(!input.open(QIODevice::ReadOnly|QIODevice::Text)) {
        error=QStringLiteral("无法打开 TXT 点云：%1").arg(input.errorString());
        return {};
    }

    pcl::PointCloud<pcl::PointXYZ> points;
    QTextStream stream(&input);
    stream.setCodec("UTF-8");
    qsizetype lineNumber=0;
    bool firstContentLine=true;
    int xyz[3]={0,1,2};
    int frameColumn=-1;
    std::vector<std::int32_t> frameIds;
    while(!stream.atEnd()) {
        QString line=stream.readLine(); ++lineNumber;
        if(lineNumber==1 && !line.isEmpty() && line.front()==QChar(0xfeff)) line.remove(0,1);
        const auto hash=line.indexOf('#'); if(hash>=0) line.truncate(hash);
        const auto slash=line.indexOf(QStringLiteral("//")); if(slash>=0) line.truncate(slash);
        line=line.trimmed();
        if(line.isEmpty()) continue;

        line.replace(',', ' ');
        const auto columns=line.split(QRegExp(QStringLiteral("\\s+")),Qt::SkipEmptyParts);
        if(firstContentLine) {
            firstContentLine=false;
            QStringList lower; lower.reserve(columns.size());
            for(const auto& column:columns) lower.push_back(column.toLower());
            xyz[0]=lower.indexOf(QStringLiteral("x")); xyz[1]=lower.indexOf(QStringLiteral("y")); xyz[2]=lower.indexOf(QStringLiteral("z"));
            frameColumn=lower.indexOf(QStringLiteral("frameindex"));
            if(frameColumn<0) frameColumn=lower.indexOf(QStringLiteral("frame_index"));
            // 当前采集软件导出 index x y z target，其中 target 表示帧号。
            // 只在有明确 XYZ 表头时采用该兼容别名，避免把无表头第五列误判。
            if(frameColumn<0&&xyz[0]>=0&&xyz[1]>=0&&xyz[2]>=0)
                frameColumn=lower.indexOf(QStringLiteral("target"));
            if(xyz[0]>=0&&xyz[1]>=0&&xyz[2]>=0) continue;
            xyz[0]=0; xyz[1]=1; xyz[2]=2;
        }
        if(columns.size()<=std::max({xyz[0],xyz[1],xyz[2]})) {
            error=QStringLiteral("TXT 点云第 %1 行至少需要三列 X Y Z").arg(lineNumber);
            return {};
        }

        bool xOk=false,yOk=false,zOk=false;
        const double x=columns[xyz[0]].toDouble(&xOk), y=columns[xyz[1]].toDouble(&yOk), z=columns[xyz[2]].toDouble(&zOk);
        pcl::PointXYZ point; point.x=static_cast<float>(x); point.y=static_cast<float>(y); point.z=static_cast<float>(z);
        if(!xOk||!yOk||!zOk||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)
           ||!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z)) {
            error=QStringLiteral("TXT 点云第 %1 行包含无效的 XYZ 数值").arg(lineNumber);
            return {};
        }
        if(frameColumn>=0) {
            if(columns.size()<=frameColumn) {
                error=QStringLiteral("TXT 点云第 %1 行缺少帧号").arg(lineNumber);
                return {};
            }
            bool frameOk=false; const qlonglong frame=columns[frameColumn].toLongLong(&frameOk);
            if(!frameOk||frame<0||frame>std::numeric_limits<std::int32_t>::max()
               ||(!frameIds.empty()&&frame<frameIds.back())) {
                error=QStringLiteral("TXT 点云第 %1 行的帧号必须是非负且非递减的 int32").arg(lineNumber);
                return {};
            }
            frameIds.push_back(static_cast<std::int32_t>(frame));
        }
        points.push_back(point);
    }
    if(stream.status()!=QTextStream::Ok) {
        error=QStringLiteral("读取 TXT 点云失败：%1").arg(input.errorString());
        return {};
    }
    if(points.empty()) {
        error=QStringLiteral("TXT 文件中没有有效的 XYZ 点");
        return {};
    }
    points.width=static_cast<std::uint32_t>(points.size()); points.height=1; points.is_dense=true;
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(points,*cloud);
    if(!frameIds.empty()) {
        const auto oldStep=cloud->point_step,oldRow=cloud->row_step;
        auto old=std::move(cloud->data);
        pcl::PCLPointField field; field.name="frame_index"; field.offset=oldStep;
        field.datatype=pcl::PCLPointField::INT32; field.count=1; cloud->fields.push_back(field);
        cloud->point_step+=4; cloud->row_step=cloud->point_step*cloud->width;
        cloud->data.resize(std::size_t(cloud->row_step)*cloud->height);
        for(std::size_t i=0;i<frameIds.size();++i) {
            auto dst=cloud->data.data()+i*cloud->point_step;
            std::memcpy(dst,old.data()+(i/cloud->width)*oldRow+(i%cloud->width)*oldStep,oldStep);
            std::memcpy(dst+oldStep,&frameIds[i],4);
        }
    }
    return cloud;
}
}

namespace fw {
Runtime::Runtime() {
    qRegisterMetaType<FramePtr>(); qRegisterMetaType<AlgorithmResult>();
    QSettings settings;
    setDebugLimits(settings.value("debug/windows",8).toInt(),settings.value("debug/bytes",qulonglong(budget_)).toULongLong());
    debug_.onFrame=[this](FramePtr frame) { QMetaObject::invokeMethod(this,[this,frame] { acceptFrame(frame); },Qt::QueuedConnection); };
    debug_.onPause=[this](bool p) { QMetaObject::invokeMethod(this,[this,p] { emit pausedChanged(p); },Qt::QueuedConnection); };
    debug_.onLog=[this](const QString& s) { emit logMessage(s); };
    debug_.onProgress=[this](int p) { emit progressChanged(std::clamp(p,0,100)); };
}
Runtime::~Runtime() { shutdown(); }
void Runtime::shutdown() {
    closing_=true; debug_.stop(); if(worker_.joinable()) worker_.join();
    frames_.clear(); historyBytes_=0; busy_=false;
}
bool Runtime::initialize(const QString& root,QString& error) {
    struct Candidate { QString path; QJsonObject meta; };
    QVector<Candidate> candidates; QMap<QString,int> pluginCounts,algorithmCounts;
    QDirIterator it(root,QDir::Files,QDirIterator::Subdirectories);
    while(it.hasNext()) {
        auto path=it.next(); if(!QLibrary::isLibrary(path)) continue;
        QPluginLoader loader(path); const auto m=loader.metaData()["MetaData"].toObject();
        if(m["sdkVersion"]!=1||m["buildId"].toString()!=QStringLiteral(FW_BUILD_ID)||m["pluginId"].toString().isEmpty()) {
            diagnostics_<<QFileInfo(path).fileName()+QStringLiteral("：插件元数据或构建版本不兼容"); continue;
        }
        if(m["type"]!="ui"&&m["type"]!="algorithm") { diagnostics_<<path+QStringLiteral("：未知插件类型"); continue; }
        candidates.push_back({path,m}); pluginCounts[m["pluginId"].toString()]++;
        if(m["type"]=="algorithm") algorithmCounts[m["algorithmId"].toString()]++;
    }
    for(const auto& c:candidates) {
        const auto id=c.meta["pluginId"].toString(), aid=c.meta["algorithmId"].toString();
        if(pluginCounts[id]>1||(c.meta["type"]=="algorithm"&&algorithmCounts[aid]>1)) {
            diagnostics_<<id+QStringLiteral("：ID 冲突，所有冲突副本均被禁用"); continue;
        }
        auto loader=std::make_unique<QPluginLoader>(c.path);
        loader->setLoadHints(QLibrary::PreventUnloadHint);
        QObject* instance=loader->instance();
        if(!instance) { diagnostics_<<id+": "+loader->errorString(); continue; }
        try {
            if(c.meta["type"]=="algorithm") {
                auto plugin=qobject_cast<IAlgorithmPlugin*>(instance);
                if(!plugin) throw std::runtime_error("Algorithm interface mismatch");
                auto descriptor=plugin->descriptor(); auto definition=plugin->parameterSchema();
                QFile overrideFile(QFileInfo(c.path).dir().filePath(QFileInfo(c.path).completeBaseName()+".parameters.json"));
                if(overrideFile.exists()) {
                    if(!overrideFile.open(QIODevice::ReadOnly)) throw std::runtime_error("Cannot read parameter override");
                    QJsonParseError parse; const auto doc=QJsonDocument::fromJson(overrideFile.readAll(),&parse);
                    if(parse.error!=QJsonParseError::NoError||!doc.isObject()) throw std::runtime_error("Invalid parameter override JSON");
                    definition=doc.object();
                }
                auto problems=Parameters::validateSchema(definition);
                if(descriptor.id!=aid||definition["algorithmId"].toString()!=aid||definition["parameterVersion"]!=c.meta["parameterVersion"]||descriptor.version!=c.meta["version"].toString()) problems<<QStringLiteral("元数据与算法契约不一致");
                if(!problems.isEmpty()) throw std::runtime_error(problems.join("; ").toStdString());
                if(definition.contains("diagnostics")) {
                    const auto profileErrors=Diagnostics::validateProfile(definition["diagnostics"].toObject(),definition,descriptor);
                    if(!profileErrors.isEmpty()) {
                        diagnostics_<<aid+QStringLiteral("：诊断档案无效，专有诊断已停用：")+profileErrors.join("; ");
                        definition.remove("diagnostics");
                    }
                }
                entries_.insert(aid,{plugin,descriptor,definition});
            } else if(id=="framework.parameters") parameters_=qobject_cast<IParameterUiPlugin*>(instance);
            else if(id=="framework.visualization") visualization_=qobject_cast<IVisualizationPlugin*>(instance);
            else if(id=="framework.main") main_=qobject_cast<IMainUiPlugin*>(instance);
            else diagnostics_<<id+QStringLiteral("：未知界面模块");
        } catch(const std::exception& e) { diagnostics_<<id+": "+QString::fromUtf8(e.what()); }
        catch(...) { diagnostics_<<id+QStringLiteral("：插件初始化异常"); }
        loaders_.push_back(std::move(loader));
    }
    if(!parameters_||!visualization_||!main_) { error=QStringLiteral("缺少必要的界面模块。\n")+diagnostics_.join('\n'); return false; }
    return true;
}
QWidget* Runtime::createWindow() { return main_->createWindow(*this); }
QVector<AlgorithmDescriptor> Runtime::algorithms() const { QVector<AlgorithmDescriptor> list; for(const auto& e:entries_) list<<e.descriptor; return list; }
QJsonObject Runtime::schema(const QString& id) const { return entries_.value(id).schema; }
bool Runtime::start(const QString& id,const AlgorithmInput& input,const ParameterSet& values,bool debugging,QString& error) {
    return startImpl(id,input,values,debugging,false,error);
}
bool Runtime::startTrial(const QString& id,const AlgorithmInput& input,const ParameterSet& values,QString& error) {
    return startImpl(id,input,values,false,true,error);
}
bool Runtime::startImpl(const QString& id,const AlgorithmInput& input,const ParameterSet& values,bool debugging,bool trial,QString& error) {
    if(busy_||closing_) { error=QStringLiteral("算法正在运行或程序正在关闭"); return false; }
    if(!entries_.contains(id)) { error=QStringLiteral("算法不存在"); return false; }
    const auto entry=entries_[id]; const auto params=Parameters::merge(entry.schema,values);
    auto issues=Parameters::validate(entry.schema,params);
    if(input.clouds.isEmpty()||!input.clouds.front().cloud||input.clouds.front().cloud->data.empty()) issues<<QStringLiteral("请先加载非空点云");
    else if(!hasFields(input.clouds.front(),entry.descriptor.requiredFields)) issues<<QStringLiteral("输入缺少字段：")+entry.descriptor.requiredFields.join(',');
    if(!issues.isEmpty()) { error=issues.join('\n'); return false; }
    if(worker_.joinable()) worker_.join();
    for(const auto& frame:frames_) emit debugEvicted(frame->sequence);
    frames_.clear(); historyBytes_=0; ++run_; busy_=true;
    debug_.reset(debugging,run_,budget_); emit runningChanged(true); emit progressChanged(0);
    worker_=std::thread([this,entry,input,params,trial] {
        AlgorithmResult result; IAlgorithm* algorithm=nullptr;
        try {
            algorithm=entry.plugin->create();
            if(!algorithm) throw std::runtime_error("Algorithm factory returned null");
            const auto validation=algorithm->validate(input,params);
            if(!validation.isEmpty()) result=AlgorithmResult::failed(validation);
            else result=freezeResult(algorithm->execute(input,params,debug_));
        } catch(const std::exception& e) { result=AlgorithmResult::failed(QString::fromUtf8(e.what())); }
        catch(...) { result=AlgorithmResult::failed(QStringLiteral("算法发生未知异常")); }
        if(algorithm) { try { entry.plugin->destroy(algorithm); } catch(...) { result=AlgorithmResult::failed(QStringLiteral("算法销毁异常")); } }
        if(debug_.isCancelled()) result=AlgorithmResult::cancelled();
        QMetaObject::invokeMethod(this,[this,result,input,params,entry,trial]() mutable {
            if(closing_) return;
            if(worker_.joinable()) worker_.join(); busy_=false;
            if(result.metrics.contains("diagnostics")) {
                QStringList errors;
                if(!entry.schema.contains("diagnostics")) errors<<QStringLiteral("算法未声明诊断档案");
                else errors=Diagnostics::validateReport(result.metrics["diagnostics"].toObject(),entry.schema["diagnostics"].toObject(),entry.descriptor,input,params,result);
                if(!errors.isEmpty()) {
                    result.metrics.remove("diagnostics");
                    result.metrics["diagnostics_validation"] = errors.join("; ");
                    emit logMessage(entry.descriptor.id+QStringLiteral("：诊断报告被拒绝：")+errors.join("; "));
                }
            }
            emit pausedChanged(false); emit runningChanged(false);
            if(trial) emit trialFinished(result); else emit finished(result);
        },Qt::QueuedConnection);
    });
    return true;
}
void Runtime::setDebugLimits(int windows,quint64 bytes) {
    windowLimit_=std::clamp(windows,1,64); budget_=std::clamp(bytes,quint64(1024*1024),quint64(8ull*1024*1024*1024));
    QSettings settings; settings.setValue("debug/windows",windowLimit_); settings.setValue("debug/bytes",qulonglong(budget_));
}
void Runtime::acceptFrame(FramePtr frame) {
    if(closing_||debug_.isCancelled()) { debug_.presented(frame->sequence); return; }
    const auto bytes=frameBytes(*frame);
    while(!frames_.empty()&&(int(frames_.size())>=windowLimit_||historyBytes_+bytes>budget_)) {
        const auto oldest=frames_.front(); frames_.pop_front(); historyBytes_-=frameBytes(*oldest); emit debugEvicted(oldest->sequence);
    }
    frames_.push_back(frame); historyBytes_+=bytes; emit debugReady(frame);
}
FramePtr Runtime::history(quint64 seq) const { for(const auto& frame:frames_) if(frame->sequence==seq) return frame; return {}; }
bool Runtime::loadPreset(const QString& id,const QString& file,ParameterSet& values,QStringList& messages) {
    if(!entries_.contains(id)) { messages<<QStringLiteral("算法不存在"); return false; }
    QFile f(file); if(!f.open(QIODevice::ReadOnly)) { messages<<f.errorString(); return false; }
    QJsonParseError parse; const auto doc=QJsonDocument::fromJson(f.readAll(),&parse); const auto o=doc.object();
    if(parse.error!=QJsonParseError::NoError||!doc.isObject()||o["algorithmId"].toString()!=id||!o["values"].isObject()||o["parameterVersion"].toInt()<1) { messages<<QStringLiteral("无效预设或算法 ID 不匹配"); return false; }
    auto candidate=o["values"].toObject(); const auto entry=entries_[id];
    if(o["parameterVersion"]!=entry.schema["parameterVersion"]) {
        QString error; ParameterSet migrated;
        try { if(!entry.plugin->migrateParameters(o["parameterVersion"].toInt(),candidate,migrated,error)) { messages<<error; return false; } }
        catch(...) { messages<<QStringLiteral("参数迁移异常"); return false; }
        candidate=migrated;
    }
    candidate=Parameters::merge(entry.schema,candidate,&messages);
    const auto errors=Parameters::validate(entry.schema,candidate); if(!errors.isEmpty()) { messages<<errors; return false; }
    values=candidate; return true;
}
bool Runtime::savePreset(const QString& id,const QString& file,const ParameterSet& values,QString& error) {
    if(!entries_.contains(id)) { error=QStringLiteral("算法不存在"); return false; }
    const auto errors=Parameters::validate(schema(id),values); if(!errors.isEmpty()) { error=errors.join('\n'); return false; }
    QSaveFile f(file); if(!f.open(QIODevice::WriteOnly)) { error=f.errorString(); return false; }
    const auto bytes=QJsonDocument(QJsonObject{{"algorithmId",id},{"parameterVersion",schema(id)["parameterVersion"]},{"values",values}}).toJson();
    if(f.write(bytes)!=bytes.size()||!f.commit()) { error=f.errorString(); return false; } return true;
}
CloudData Runtime::readCloud(const QString& file,const QString& unit,QString& error) {
    error.clear();
    CloudData data; data.id=QUuid::createUuid().toString(QUuid::WithoutBraces); data.name=QFileInfo(file).fileName(); data.unit=unit;
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    try {
        const auto path=file.toLocal8Bit().toStdString(); const auto ext=QFileInfo(file).suffix().toLower();
        if(ext=="txt") cloud=loadTextCloud(file,error);
        else {
            const int status=ext=="pcd"?pcl::io::loadPCDFile(path,*cloud):ext=="ply"?pcl::io::loadPLYFile(path,*cloud):-1;
            if(status<0||cloud->data.empty()) throw std::runtime_error("Cannot read cloud, empty cloud, or unsupported extension");
        }
        if(!cloud||cloud->data.empty()) {
            if(error.isEmpty()) error=QStringLiteral("无法读取点云、点云为空或文件格式不受支持");
            return data;
        }
        data.cloud=cloud;
    } catch(const std::exception& e) { error=QString::fromUtf8(e.what()); }
    return data;
}
bool Runtime::writeCloud(const QString& file,const CloudData& data,QString& error) {
    if(!data.cloud) { error=QStringLiteral("没有可保存的点云"); return false; }
    try {
        const auto path=file.toLocal8Bit().toStdString(); const auto ext=QFileInfo(file).suffix().toLower();
        int result=-1;
        if(ext=="pcd") { pcl::PCDWriter writer; result=writer.writeBinary(path,*data.cloud); }
        else if(ext=="ply") { pcl::PLYWriter writer; result=writer.write(path,*data.cloud,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),true,false); }
        if(result<0) { error=QStringLiteral("保存失败；只支持 PCD/PLY"); return false; }
    } catch(const std::exception& e) { error=QString::fromUtf8(e.what()); return false; }
    return true;
}
}
