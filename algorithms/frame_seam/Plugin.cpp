#include <fw/Algorithm.h>
#include "FrameSeamAlgorithm.h"
#include "FrameManifestIO.h"
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <cmath>
#include <cstring>

namespace {
constexpr auto AlgorithmId="weld.frame_seam";
QString text(const std::string& s) {return QString::fromUtf8(s.data(),int(s.size()));}
weld::DetectorConfig configuration(const fw::ParameterSet& p) {
    weld::DetectorConfig c;
#define FLOAT_PARAMETER(field) c.field=static_cast<float>(p[#field].toDouble())
#define INT_PARAMETER(field) c.field=p[#field].toInt()
    FLOAT_PARAMETER(frameBinSize); FLOAT_PARAMETER(frameCornerWindow);
    FLOAT_PARAMETER(frameMinTurnDegrees); FLOAT_PARAMETER(frameMinDepth);
    FLOAT_PARAMETER(frameMaxProfileGap); INT_PARAMETER(frameSurfaceNeighbors);
    FLOAT_PARAMETER(frameSurfaceRadius); FLOAT_PARAMETER(frameSurfaceMaxRms);
    FLOAT_PARAMETER(frameMaxCorrection); FLOAT_PARAMETER(frameMaxPointDistance);
    FLOAT_PARAMETER(frameMaxTrackJump); INT_PARAMETER(frameMaxMissing);
    FLOAT_PARAMETER(minSeamSeparation);
#undef FLOAT_PARAMETER
#undef INT_PARAMETER
    return c;
}
fw::CloudData outputCloud(const weld::CloudT& points,const std::vector<float>& confidence,
                          const std::vector<std::size_t>& starts,const fw::CloudData& input,
                          const QString& id,const QString& name) {
    auto layer=fw::DebugLayer::result(name,points);
    auto cloud=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*layer.data.cloud));
    const auto oldStep=cloud->point_step;
    auto old=std::move(cloud->data);
    cloud->point_step=oldStep+8; cloud->row_step=cloud->point_step*cloud->width;
    cloud->data.resize(std::size_t(cloud->row_step)*cloud->height);
    pcl::PCLPointField field; field.name="confidence"; field.offset=oldStep; field.datatype=pcl::PCLPointField::FLOAT32; field.count=1; cloud->fields.push_back(field);
    field.name="segment_id"; field.offset=oldStep+4; field.datatype=pcl::PCLPointField::UINT32; cloud->fields.push_back(field);
    std::uint32_t segment=0;
    for(std::size_t i=0;i<points.size();++i) {
        while(segment+1<starts.size()&&i>=starts[segment+1]) ++segment;
        auto dst=cloud->data.data()+i*cloud->point_step;
        std::memcpy(dst,old.data()+i*oldStep,oldStep);
        const float score=i<confidence.size()?confidence[i]:0;
        std::memcpy(dst+oldStep,&score,4); std::memcpy(dst+oldStep+4,&segment,4);
    }
    fw::CloudData result=input; result.id=id; result.name=name; result.cloud=cloud; return result;
}
void addSegments(QVector<fw::DebugLayer>& layers,const weld::CloudT& cloud,
                 const std::vector<std::size_t>& starts,const QString& name,
                 const std::array<double,3>& color) {
    for(std::size_t s=0;s<starts.size();++s) {
        const auto end=s+1<starts.size()?starts[s+1]:cloud.size();
        if(end-starts[s]<2) continue;
        fw::DebugLayer line; line.name=name+QStringLiteral(" · 段 %1").arg(s+1);
        line.geometry.kind=fw::GeometryKind::Polyline; line.style.color=color; line.style.lineWidth=4;
        for(auto i=starts[s];i<end;++i) line.geometry.vertices.push_back({cloud[i].x,cloud[i].y,cloud[i].z});
        layers.push_back(std::move(line));
    }
}
class FrameAlgorithm final : public fw::IAlgorithm {
public:
    QString validate(const fw::AlgorithmInput& input,const fw::ParameterSet& p) const override {
        if(input.clouds.size()!=1||!input.clouds[0].cloud||!fw::hasFields(input.clouds[0],{"x","y","z"}))
            return QStringLiteral("需要一份包含 XYZ 的完整原始扫描点云。");
        for(const auto& field:input.clouds[0].cloud->fields)
            if((field.name=="x"||field.name=="y"||field.name=="z")&&field.datatype!=pcl::PCLPointField::FLOAT32)
                return QStringLiteral("当前算法要求 XYZ 为 float32。");
        if(!QFileInfo(p["manifest"].toString()).isFile()) return QStringLiteral("请选择与输入点云匹配的帧索引 manifest.json。");
        return {};
    }
    fw::AlgorithmResult execute(const fw::AlgorithmInput& input,const fw::ParameterSet& p,fw::IExecutionContext& context) override {
        const auto invalid=validate(input,p); if(!invalid.isEmpty()) return fw::AlgorithmResult::failed(invalid);
        if(context.isCancelled()) return fw::AlgorithmResult::cancelled();
        const auto& source=input.clouds.front(); auto cloud=fw::toCloud<weld::PointT>(source);
        context.log(QStringLiteral("正在逐点核对原始帧索引；长度参数沿用输入坐标单位。"));
        std::vector<weld::ScanFrameRange> ranges; QString error;
        if(!weld::loadFrameManifest(p["manifest"].toString(),cloud,&ranges,&error,[&] {return context.isCancelled();}))
            return context.isCancelled()?fw::AlgorithmResult::cancelled():fw::AlgorithmResult::failed(error);
        context.progress(10);
        const bool debug=context.debug().enabled(); const bool eachFrame=p["debug_detail"].toString()=="frames";
        weld::DebugTrace trace; weld::ExecutionHooks hooks; hooks.retainTrace=false;
        hooks.cancelled=[&] {return context.isCancelled();};
        int stageIndex=0;
        hooks.onStep=[&](const weld::DebugStep& step) {
            const bool frameStep=step.metrics.count("sdk_sequence")!=0;
            if(!frameStep) {context.log(text(step.title)+QStringLiteral("：")+text(step.message)); context.progress(std::min(95,10+stageIndex++*15));}
            if(!debug||(frameStep&&!eachFrame)) return;
            fw::DebugFrame frame; frame.stepId=frameStep?"scan_frame":QString("stage_%1").arg(stageIndex);
            frame.title=text(step.title); frame.message=text(step.message);
            for(const auto& item:step.metrics) if(std::isfinite(item.second)) frame.metrics[text(item.first)]=item.second;
            frame.metrics["elapsedMs"]=step.elapsedMs;
            for(const auto& layer:step.layers) {
                if(!layer.cloud||layer.cloud->empty()) continue;
                auto converted=fw::DebugLayer::result(text(layer.name),*layer.cloud);
                converted.data.unit=source.unit; converted.data.coordinateFrame=source.coordinateFrame;
                if(layer.name=="输入点云"||layer.name=="处理点云"||layer.name=="原始帧") converted=fw::DebugLayer::background(converted.name,converted.data);
                else if(layer.name=="左轨迹") converted.style.color={1,0.8,0.15};
                frame.layers.push_back(std::move(converted));
            }
            if(context.debug().checkpoint(frame)==fw::DebugAction::Stop) throw weld::ExecutionCancelled();
        };
        weld::SeamResult seams;
        try {seams=weld::detectFrameSeams(cloud,configuration(p),ranges,debug?&trace:nullptr,&hooks);}
        catch(const weld::ExecutionCancelled&) {return fw::AlgorithmResult::cancelled();}
        if(context.isCancelled()) return fw::AlgorithmResult::cancelled();
        if(!seams.ok()) return fw::AlgorithmResult::failed(text(seams.summary));
        const auto prefix=p["name"].toString();
        auto left=outputCloud(*seams.leftSeam,seams.leftPointConfidence,seams.leftSegmentStarts,source,"left_seam",prefix+QStringLiteral(" · 左"));
        auto right=outputCloud(*seams.rightSeam,seams.rightPointConfidence,seams.rightSegmentStarts,source,"right_seam",prefix+QStringLiteral(" · 右"));
        auto l=fw::DebugLayer::result(left.name,left); l.style.color={1,0.8,0.15};
        auto r=fw::DebugLayer::result(right.name,right); r.style.color={0.2,1,0.55};
        fw::AlgorithmResult result; result.outputs={left,right}; result.layers={fw::DebugLayer::background(source.name,source),l,r};
        addSegments(result.layers,*seams.leftSeam,seams.leftSegmentStarts,left.name,l.style.color);
        addSegments(result.layers,*seams.rightSeam,seams.rightSegmentStarts,right.name,r.style.color);
        result.message=text(seams.summary);
        result.metrics={{"frameCount",int(ranges.size())},{"leftPoints",int(seams.leftSeam->size())},{"rightPoints",int(seams.rightSeam->size())},
            {"leftConfidence",seams.leftConfidence},{"rightConfidence",seams.rightConfidence},
            {"leftSegments",int(seams.leftSegmentStarts.size())},{"rightSegments",int(seams.rightSegmentStarts.size())}};
        context.progress(100); return result;
    }
};
class FramePlugin final : public QObject,public fw::IAlgorithmPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_ALGORITHM_IID FILE "metadata.json")
    Q_INTERFACES(fw::IAlgorithmPlugin)
public:
    fw::AlgorithmDescriptor descriptor() const override {return {AlgorithmId,QStringLiteral("焊缝识别：单帧轮廓 + 邻帧局部面"),"1.0.0",{"x","y","z"}};}
    QJsonObject parameterSchema() const override {QFile file(":/weld.frame_seam/parameters.json"); if(!file.open(QIODevice::ReadOnly)) return {}; return QJsonDocument::fromJson(file.readAll()).object();}
    fw::IAlgorithm* create() override {return new FrameAlgorithm;}
    void destroy(fw::IAlgorithm* algorithm) override {delete algorithm;}
};
}
#include "Plugin.moc"
