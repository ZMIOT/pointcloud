#include "Configuration.h"
#include "CloudFrames.h"
#include "SeamDetector.h"
#include "HullSurfaces.h"
#include <fw/Parameters.h>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QSet>
#include <algorithm>
#include <cmath>
#include <cstring>
namespace {
constexpr auto AlgorithmId="weld.extract_feature2";
QString text(const std::string& s) {return QString::fromUtf8(s.data(),int(s.size()));}
QJsonObject schema() {QFile f(":/weld.extract_feature2/parameters.json"); if(!f.open(QIODevice::ReadOnly)) return {}; return QJsonDocument::fromJson(f.readAll()).object();}
fw::AlgorithmResult surfaceResult(const fw::CloudData& source,const weld::HullSurfacesResult& surfaces,
                                  const fw::ParameterSet& p,fw::IExecutionContext& context,bool complete=true) {
    fw::AlgorithmResult result;
    const std::array<QString,1> ids{"hull_body"};
    const std::array<QString,1> names{QStringLiteral("主体 · 底面与交界保护")};
    const std::array<std::array<double,3>,1> colors{{{.2,.75,1}}};
    QJsonArray outputs;
    for(int side=0;side<1;++side) {
        if(context.isCancelled())return fw::AlgorithmResult::cancelled();
        auto cloud=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*source.cloud));
        cloud->data.clear();cloud->height=1;cloud->width=0;cloud->is_dense=true;
        for(std::size_t i=0;i<surfaces.sourceIndices.size();++i) {
            if((i&4095u)==0&&context.isCancelled())return fw::AlgorithmResult::cancelled();
            const auto index=surfaces.sourceIndices[i];
            const auto offset=(index/source.cloud->width)*source.cloud->row_step+(index%source.cloud->width)*source.cloud->point_step;
            const auto begin=source.cloud->data.begin()+offset;
            cloud->data.insert(cloud->data.end(),begin,begin+source.cloud->point_step);++cloud->width;
        }
        cloud->row_step=cloud->width*cloud->point_step;
        if(!cloud->width)return fw::AlgorithmResult::failed(QStringLiteral("主体为空；未回退到未经分割的点云。"));
        fw::CloudData output=source;output.id=ids[side];output.name=p["name"].toString()+" · "+names[side];output.cloud=cloud;
        result.outputs.push_back(output);
        auto layer=fw::DebugLayer::result(output.name,output);layer.style.color=colors[side];layer.style.pointSize=2;
        result.layers.push_back(layer);
        outputs.append(QJsonObject{{"id",ids[side]},{"cloudIndex",side},{"statistics",QJsonObject{{"sampleCount",double(cloud->width)},{"interpolatedCount",0}}}});
    }
    const double count=double(quint64(source.cloud->width)*source.cloud->height);
    result.metrics={{"processingMode","surfaces_only"},{"inputPoints",count},{"retainedPoints",double(surfaces.sourceIndices.size())},
        {"removedPoints",count-double(surfaces.sourceIndices.size())},{"analysisPoints",double(surfaces.analysisPoints)},
        {"referencePatches",double(surfaces.referencePatches)},{"recoveredMeasuredPoints",double(surfaces.recoveredIndices.size())},
        {"protectedUncertainPoints",double(surfaces.protectedUncertainIndices.size())},
        {"rejectedNearSurfacePoints",double(surfaces.rejectedNearSurfacePoints)}};
    result.metrics["diagnostics"]=QJsonObject{{"protocolVersion",1},{"algorithmId",AlgorithmId},{"algorithmVersion","1.9.0"},
        {"input",QJsonObject{{"cloudId",source.id},{"pointCount",count},{"coordinateFrame",source.coordinateFrame},{"unit",source.unit}}},
        {"parameters",p},{"outputs",outputs},{"issues",QJsonArray{QJsonObject{
          {"code","hull_completeness_unverified"},{"stage","hull_surfaces"},{"severity","info"},
          {"origin","undetermined"},{"visibility","unknown"},
          {"scope",QJsonObject{{"targetId","hull_main_body"},{"outputRole","segmented"}}},
          {"region",QJsonObject{{"kind","whole"}}},
          {"summary",QStringLiteral("最大块用于定位主体，局部有界曲面用于恢复底面与交界处的原始点；恢复数量不代表准确性，三面完整性仍需对照原始点复核。")},
          {"evidence",QJsonObject{{"support",double(surfaces.sourceIndices.size())},{"supportUnit","count"},{"basis","retained_measured_points_not_accuracy"}}}
        }}}};
    if(surfaces.rejectedNearSurfacePoints) {
        auto report=result.metrics["diagnostics"].toObject();auto issues=report["issues"].toArray();
        issues.append(QJsonObject{{"code","hull_near_surface_rejected"},{"stage","hull_surfaces"},{"severity","info"},
          {"origin","undetermined"},{"visibility","observable"},
          {"scope",QJsonObject{{"targetId","hull_main_body"},{"outputRole","segmented"}}},
          {"region",QJsonObject{{"kind","whole"}}},
          {"summary",QStringLiteral("有原始点位于局部曲面支持范围附近但未通过距离判据；可能是干扰或真实局部形变，请在调试图中复核，不能视为输入缺测。")},
          {"evidence",QJsonObject{{"support",double(surfaces.rejectedNearSurfacePoints)},{"supportUnit","count"},{"basis","rejected_inside_patch_footprint_within_three_distance_tolerances"}}}});
        report["issues"]=issues;result.metrics["diagnostics"]=report;
    }
    if(!surfaces.protectedUncertainIndices.empty()) {
        auto report=result.metrics["diagnostics"].toObject();auto issues=report["issues"].toArray();
        issues.append(QJsonObject{{"code","hull_uncertain_points_preserved"},{"stage","hull_surfaces"},{"severity","info"},
          {"origin","undetermined"},{"visibility","observable"},
          {"scope",QJsonObject{{"targetId","hull_main_body"},{"outputRole","segmented"}}},{"region",QJsonObject{{"kind","whole"}}},
          {"summary",QStringLiteral("主体有界范围内的部分原始点未通过严格参考曲面容差，但在清理保护带内予以保留；可能包含粗糙底面、焊趾、真实形变或贴面噪声。数量不是准确性指标，后续拟合不因此放宽验收条件。")},
          {"evidence",QJsonObject{{"support",double(surfaces.protectedUncertainIndices.size())},{"supportUnit","count"},{"basis","measured_points_retained_by_bounded_preservation_band"}}}});
        report["issues"]=issues;result.metrics["diagnostics"]=report;
    }
    result.message=QStringLiteral("第一步：保留主体原始点 %1 个，保护带额外保留 %2 个待判定点；不补点、不平滑。清理余量与参考面拟合容差分离，底面和交界仍需复核。")
        .arg(surfaces.sourceIndices.size()).arg(surfaces.protectedUncertainIndices.size());
    if(context.debug().enabled()) {
        fw::DebugFrame frame;frame.stepId="hull_surfaces";frame.title=QStringLiteral("第一步 · 保留与剔除对照");
        frame.message=result.message;frame.layers=result.layers;frame.metrics=result.metrics;
        const auto raw=fw::toCloud<weld::PointT>(source);weld::CloudT recovered,rejected,uncertain;
        for(auto index:surfaces.recoveredIndices)recovered.push_back((*raw)[index]);
        for(auto index:surfaces.protectedUncertainIndices)uncertain.push_back((*raw)[index]);
        std::size_t kept=0;
        for(std::size_t i=0;i<raw->size();++i) {
            if((i&4095u)==0&&context.isCancelled())return fw::AlgorithmResult::cancelled();
            if(kept<surfaces.sourceIndices.size()&&surfaces.sourceIndices[kept]==i)++kept;else rejected.push_back((*raw)[i]);
        }
        auto comparisonLayer=[&](const QString& name,const weld::CloudT& points,const std::array<double,3>& color) {
            if(points.empty())return;auto layer=fw::DebugLayer::result(name,points);
            layer.data.unit=source.unit;layer.data.coordinateFrame=source.coordinateFrame;layer.style.color=color;frame.layers.push_back(std::move(layer));
        };
        comparisonLayer(QStringLiteral("恢复的原始实测点（非插补）"),recovered,{.2,1,.3});
        comparisonLayer(QStringLiteral("保护带原始点（待判定，非拟合内点）"),uncertain,{1,.65,.1});
        comparisonLayer(QStringLiteral("剔除的原始点（待复核）"),rejected,{1,.25,.2});
        if(context.debug().checkpoint(frame)==fw::DebugAction::Stop)return fw::AlgorithmResult::cancelled();
    }
    if(context.isCancelled())return fw::AlgorithmResult::cancelled();
    context.progress(complete?100:35);return result;
}
fw::CloudData outputCloud(const weld::CloudT& points,const std::vector<float>& confidence,
                          const std::vector<weld::SeamPointQuality>& quality,
                          const std::vector<std::size_t>& starts,const fw::CloudData& input,
                          const QString& id,const QString& name) {
    auto layer=fw::DebugLayer::result(name,points);
    auto cloud=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*layer.data.cloud));
    const auto oldStep=cloud->point_step;
    auto old=std::move(cloud->data);
    cloud->point_step=oldStep+32; cloud->row_step=cloud->point_step*cloud->width;
    cloud->data.resize(std::size_t(cloud->row_step)*cloud->height);
    pcl::PCLPointField field; field.name="confidence"; field.offset=oldStep; field.datatype=pcl::PCLPointField::FLOAT32; field.count=1; cloud->fields.push_back(field);
    field.name="segment_id"; field.offset=oldStep+4; field.datatype=pcl::PCLPointField::UINT32; cloud->fields.push_back(field);
    field.datatype=pcl::PCLPointField::FLOAT32;
    field.name="rts_shift"; field.offset=oldStep+8; cloud->fields.push_back(field);
    field.name="innovation_nis"; field.offset=oldStep+12; cloud->fields.push_back(field);
    field.name="position_sigma"; field.offset=oldStep+16; cloud->fields.push_back(field);
    field.name="quality_flags"; field.offset=oldStep+20; field.datatype=pcl::PCLPointField::UINT32; cloud->fields.push_back(field);
    field.name="wall_rms"; field.offset=oldStep+24; field.datatype=pcl::PCLPointField::FLOAT32; cloud->fields.push_back(field);
    field.name="floor_rms"; field.offset=oldStep+28; cloud->fields.push_back(field);
    std::uint32_t segment=0;
    for(std::size_t i=0;i<points.size();++i) {
        while(segment+1<starts.size()&&i>=starts[segment+1]) ++segment;
        auto dst=cloud->data.data()+i*cloud->point_step;
        std::memcpy(dst,old.data()+i*oldStep,oldStep);
        const float score=i<confidence.size()?confidence[i]:0;
        std::memcpy(dst+oldStep,&score,4); std::memcpy(dst+oldStep+4,&segment,4);
        const weld::SeamPointQuality q=i<quality.size()?quality[i]:weld::SeamPointQuality{};
        std::memcpy(dst+oldStep+8,&q.rtsCorrection,4);
        std::memcpy(dst+oldStep+12,&q.innovationNis,4);
        std::memcpy(dst+oldStep+16,&q.positionSigma,4);
        std::memcpy(dst+oldStep+20,&q.flags,4);
        std::memcpy(dst+oldStep+24,&q.wallRms,4);
        std::memcpy(dst+oldStep+28,&q.floorRms,4);
    }
    fw::CloudData result=input; result.id=id; result.name=name; result.cloud=cloud; return result;
}
QJsonObject diagnosticReport(const weld::SeamResult& seams,const fw::CloudData& input,
                             const fw::ParameterSet& parameters) {
    QJsonArray issues;
    auto append=[&](const char* code,const char* stage,const char* severity,const char* target,
                    const QString& summary,int support,const char* basis) {
        if(!support) return;
        issues.append(QJsonObject{{"code",code},{"stage",stage},{"severity",severity},
            {"origin","undetermined"},{"visibility","observable"},
            {"scope",QJsonObject{{"targetId",target},{"outputRole","fitted"}}},
            {"region",QJsonObject{{"kind","whole"}}},{"summary",summary},
            {"evidence",QJsonObject{{"support",support},{"supportUnit","count"},{"basis",basis}}}});
    };
    auto addSide=[&](const std::vector<weld::SeamPointQuality>& quality,const char* target) {
        int weak=0,outliers=0,limited=0;
        for(const auto& q:quality) {
            weak+=(q.flags&(weld::QualityWallUnavailable|weld::QualityFloorUnavailable))!=0;
            outliers+=(q.flags&weld::QualityInnovationOutlier)!=0;
            limited+=(q.flags&(weld::QualityCorrectionLimited|weld::QualityGeometryLimited|
                               weld::QualityPairConstraintLimited))!=0;
        }
        append("geometry_evidence_weak","geometry","info",target,
               QStringLiteral("部分站位未同时获得全部局部几何约束；这不等同于定位错误。"),weak,"quality_flags");
        append("rts_innovation_outlier","smoothing","warning",target,
               QStringLiteral("部分站位与局部常斜率模型不一致，可能是噪声、错选或真实形变，需结合原始点云复核。"),outliers,"innovation_nis");
        append("rts_correction_limited","smoothing","info",target,
               QStringLiteral("部分 RTS 修正受到最大位移、局部几何或左右间距约束。"),limited,"quality_flags");
    };
    addSide(seams.leftPointQuality,"left_weld_seam");
    addSide(seams.rightPointQuality,"right_weld_seam");
    if(seams.hullEnvelopeRequested&&!seams.hullEnvelopeApplied) {
        const auto detail=QString::fromStdString(seams.hullEnvelopeStatus);
        for(const char* target:{"left_weld_seam","right_weld_seam"})
            append("hull_envelope_unavailable","hull_crop","warning",target,
                   QStringLiteral("当前输入未形成可靠的整船外壁闭合包络，已保留常规预处理点云继续运行（状态：%1）；这不等同于输入没有船体外壁。").arg(detail),
                   1,"hull_envelope_status");
    }
    if(seams.interiorOrientationAmbiguous) {
        const int support=int(std::min(seams.positiveInteriorStations,seams.negativeInteriorStations));
        for(const char* target:{"left_weld_seam","right_weld_seam"})
            append("interior_orientation_ambiguous","candidate","warning",target,
                   QStringLiteral("自动模式下两个开口方向的连续三面结构证据接近；当前选择仅为算法内部判定，请结合坐标方向或原始点云确认。"),
                   support,"opposite_opening_station_coverage");
    }
    const int axis=QStringLiteral("xyz").indexOf(parameters.value("longitudinalAxis").toString());
    const float jumpThreshold=std::max(2.0f,0.4f*float(parameters.value("maxTransitionDistance").toDouble()));
    auto addJumps=[&](const weld::CloudT& cloud,const std::vector<std::size_t>& starts,const char* target) {
        int count=0;float maximum=0;
        for(std::size_t i=1;i<cloud.size();++i) {
            if(std::binary_search(starts.begin(),starts.end(),i))continue;
            Eigen::Vector3f delta=weld::toVector(cloud[i])-weld::toVector(cloud[i-1]);
            if(axis>=0) delta[axis]=0;
            const float distance=delta.norm();maximum=std::max(maximum,distance);
            count+=distance>jumpThreshold;
        }
        if(count) append("trajectory_jump_observed","selection","warning",target,
            QStringLiteral("相邻输出帧有 %1 处横向位移超过 %2，最大值为 %3；可能是候选切换、噪声或真实突变，需结合原始点云复核。")
                .arg(count).arg(jumpThreshold,0,'f',2).arg(maximum,0,'f',2),
            count,"adjacent_transverse_displacement");
    };
    addJumps(*seams.leftSeam,seams.leftSegmentStarts,"left_weld_seam");
    addJumps(*seams.rightSeam,seams.rightSegmentStarts,"right_weld_seam");
    if(seams.frameBased&&seams.inputFrameCount) {
        auto addCoverage=[&](std::size_t outputCount,const char* target) {
            const double ratio=double(outputCount)/double(seams.inputFrameCount);
            if(ratio<1.0) append("limited_frame_coverage","candidate","info",target,
                QStringLiteral("焊缝输出覆盖输入帧的 %1%；未输出范围包含弱特征或未通过判据的帧，不等同于输入缺测，也不能证明当前定位正确。")
                    .arg(100.0*ratio,0,'f',1),
                int(seams.inputFrameCount-outputCount),"unselected_input_frames");
        };
        addCoverage(seams.leftSeam->size(),"left_weld_seam");
        addCoverage(seams.rightSeam->size(),"right_weld_seam");
    }
    auto output=[&](const char* id,const weld::CloudT& cloud,
                    const std::vector<std::size_t>& starts,
                    const std::vector<weld::SeamPointQuality>& quality,int index) {
        int inserted=0; for(const auto& q:quality) inserted+=(q.flags&weld::QualityInterpolated)!=0;
        return QJsonObject{{"id",id},{"cloudIndex",index},{"statistics",QJsonObject{
            {"sampleCount",double(cloud.size())},{"segmentCount",cloud.empty()?0:std::max(1,int(starts.size()))},
            {"interpolatedCount",inserted}}}};
    };
    QJsonArray outputs;
    outputs.append(output("left_seam",*seams.leftSeam,seams.leftSegmentStarts,seams.leftPointQuality,0));
    outputs.append(output("right_seam",*seams.rightSeam,seams.rightSegmentStarts,seams.rightPointQuality,1));
    return QJsonObject{{"protocolVersion",1},{"algorithmId",AlgorithmId},{"algorithmVersion","1.9.0"},
        {"input",QJsonObject{{"cloudId",input.id},{"pointCount",double(input.cloud?quint64(input.cloud->width)*input.cloud->height:0)},
                              {"coordinateFrame",input.coordinateFrame},{"unit",input.unit}}},
        {"parameters",parameters},{"outputs",outputs},{"issues",issues}};
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
class ExtractAlgorithm final : public fw::IAlgorithm {
public:
    QString validate(const fw::AlgorithmInput& input,const fw::ParameterSet& p) const override {
        if(input.clouds.size()!=1||!input.clouds[0].cloud||!fw::hasFields(input.clouds[0],{"x","y","z"})||input.clouds[0].cloud->data.empty())
            return QStringLiteral("请选择一份非空的 XYZ 原始点云。");
        const auto errors=fw::Parameters::validate(schema(),p);
        if(!errors.isEmpty()) return errors.join('\n');
        for(const auto& f:input.clouds[0].cloud->fields)
            if((f.name=="x"||f.name=="y"||f.name=="z")&&(f.datatype!=pcl::PCLPointField::FLOAT32||f.count!=1))
                return QStringLiteral("XYZ 字段需要为 float32。");
        const auto& raw=*input.clouds[0].cloud;
        if(raw.is_bigendian||!raw.width||!raw.height||!raw.point_step||
           quint64(raw.row_step)<quint64(raw.width)*raw.point_step||
           quint64(raw.data.size())<quint64(raw.height-1)*raw.row_step+quint64(raw.width)*raw.point_step)
            return QStringLiteral("点云存储布局无效或不支持大端数据。");
        for(const auto& f:raw.fields)if((f.name=="x"||f.name=="y"||f.name=="z")&&quint64(f.offset)+4>raw.point_step)
            return QStringLiteral("XYZ 字段超出点记录范围。");
        if(p.value("processingMode").toString()!="surfaces_only")
            try {readFrames(raw);} catch(const std::exception& e) {return text(e.what());}
        return {};
    }
    fw::AlgorithmResult execute(const fw::AlgorithmInput& input,const fw::ParameterSet& p,fw::IExecutionContext& context) override {
        if(context.isCancelled()) return fw::AlgorithmResult::cancelled();
        const auto invalid=validate(input,p); if(!invalid.isEmpty()) return fw::AlgorithmResult::failed(invalid);
        const auto& source=input.clouds.front(); auto cloud=fw::toCloud<weld::PointT>(source);
        const bool protectedSeams=p.value("processingMode").toString()=="protected_seams";
        fw::AlgorithmResult body;
        if(p.value("processingMode").toString()=="surfaces_only"||protectedSeams) {
            context.log(protectedSeams?QStringLiteral("ExtractFeature2：主体与底面保护后识别两条焊缝；不再二次滤除底面原始点。"):
                QStringLiteral("ExtractFeature2：仅执行第一步最大主体保留；原始坐标及附加字段保持不变。"));context.progress(5);
            weld::HullSurfacesResult surfaces;
            try {surfaces=weld::extractHullSurfaces(cloud,configuration(p),[&]{return context.isCancelled();});}
            catch(const weld::ExecutionCancelled&){return fw::AlgorithmResult::cancelled();}
            if(context.isCancelled())return fw::AlgorithmResult::cancelled();
            if(!surfaces.ok())return fw::AlgorithmResult::failed(QStringLiteral("三面分割失败：")+text(surfaces.error));
            body=surfaceResult(source,surfaces,p,context,!protectedSeams);
            if(!protectedSeams||body.status!=fw::ResultStatus::Success)return body;
            cloud=surfaces.cloud;
        }
        auto frames=readFrames(protectedSeams?*body.outputs.front().cloud:*source.cloud); auto config=configuration(p);
        config.protectedHullInput=protectedSeams;
        if(protectedSeams) {
            config.useHullEnvelopeCrop=false;
            config.profileBinSize=float(p["protectedProfileBinSize"].toDouble());
            config.minSeamSeparation=float(p["protectedMinSeamSeparation"].toDouble());
        }
        config.debugFlag=context.debug().enabled();
        context.log(frames.empty()?QStringLiteral("ExtractFeature2：按空间截面处理，无需 manifest.json。"):
            QStringLiteral("ExtractFeature2：使用 frame_index 中的 %1 个原始帧。").arg(frames.size()));
        context.progress(protectedSeams?35:5);
        weld::DebugTrace trace; trace.retainDetails=false; QSet<QString> seen;
        const bool allSteps=p["debug_detail"].toString()=="all";
        trace.onStep=[&](const weld::DebugStep& step) {
            if(context.isCancelled()) throw weld::ExecutionCancelled();
            const auto title=text(step.title),id=title.left(3);
            if(!allSteps&&seen.contains(id)) return true;
            seen.insert(id);
            fw::DebugFrame f; f.stepId=id; f.title=title; f.message=text(step.message);
            for(const auto& m:step.metrics) if(std::isfinite(m.second)) f.metrics[text(m.first)]=m.second;
            for(const auto& l:step.layers) {
                if(!l.cloud||l.cloud->empty()) continue;
                auto layer=fw::DebugLayer::result(text(l.name),*l.cloud);
                layer.data.unit=source.unit; layer.data.coordinateFrame=source.coordinateFrame;
                if(l.name=="原始点云" || (l.name=="处理后点云"&&step.layers.size()>1)) layer=fw::DebugLayer::background(layer.name,layer.data);
                else if(layer.name.contains(QStringLiteral("左"))) layer.style.color={1,.8,.15};
                f.layers.push_back(std::move(layer));
            }
            context.progress(protectedSeams?std::min(95,35+id.mid(1).toInt()*3):std::min(95,5+id.mid(1).toInt()*5));
            if(context.debug().checkpoint(f)==fw::DebugAction::Stop) throw weld::ExecutionCancelled();
            return true;
        };
        weld::SeamResult seams;
        try {seams=weld::SeamDetector().detect(cloud,config,nullptr,nullptr,config.debugFlag?&trace:nullptr,
            frames.empty()?nullptr:&frames,[&] {return context.isCancelled();});}
        catch(const weld::ExecutionCancelled&) {return fw::AlgorithmResult::cancelled();}
        if(context.isCancelled()) return fw::AlgorithmResult::cancelled();
        if(!seams.ok()) return fw::AlgorithmResult::failed(text(seams.summary));
        if(protectedSeams&&seams.frameBased)seams.inputFrameCount=readFrames(*source.cloud).size();
        const auto prefix=p["name"].toString();
        auto left=outputCloud(*seams.leftSeam,seams.leftPointConfidence,seams.leftPointQuality,seams.leftSegmentStarts,source,"left_seam",prefix+QStringLiteral(" · 左"));
        auto right=outputCloud(*seams.rightSeam,seams.rightPointConfidence,seams.rightPointQuality,seams.rightSegmentStarts,source,"right_seam",prefix+QStringLiteral(" · 右"));
        if(seams.frameBased) {
            auto l=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*left.cloud)); writeFrameIds(*l,seams.leftFrameIndices); left.cloud=l;
            auto r=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*right.cloud)); writeFrameIds(*r,seams.rightFrameIndices); right.cloud=r;
        }
        auto l=fw::DebugLayer::result(left.name,left); l.style.color={1,.8,.15};
        auto r=fw::DebugLayer::result(right.name,right); r.style.color={.2,1,.55};
        auto hull=fw::DebugLayer::result(QStringLiteral("整船外壁包络裁剪结果"),*seams.filteredCloud);
        hull.data.unit=source.unit; hull.data.coordinateFrame=source.coordinateFrame;
        hull.style.color={.2,.75,1}; hull.style.pointSize=2;
        fw::AlgorithmResult result; result.outputs={left,right}; result.layers={fw::DebugLayer::background(source.name,source),hull,l,r};
        addSegments(result.layers,*seams.leftSeam,seams.leftSegmentStarts,left.name,l.style.color);
        addSegments(result.layers,*seams.rightSeam,seams.rightSegmentStarts,right.name,r.style.color);
        result.message=text(seams.summary);
        result.metrics={{"frameBased",seams.frameBased},{"frameCount",int(seams.inputFrameCount)},
            {"leftPoints",int(seams.leftSeam->size())},{"rightPoints",int(seams.rightSeam->size())},
            {"leftConfidence",seams.leftConfidence},{"rightConfidence",seams.rightConfidence}};
        result.metrics["hullEnvelopeRequested"]=seams.hullEnvelopeRequested;
        result.metrics["hullEnvelopeApplied"]=seams.hullEnvelopeApplied;
        result.metrics["hullEnvelopeStatus"]=QString::fromStdString(seams.hullEnvelopeStatus);
        result.metrics["hullEnvelopeInputPoints"]=double(seams.hullEnvelopeInputPoints);
        result.metrics["hullEnvelopeRetainedPoints"]=double(seams.hullEnvelopeRetainedPoints);
        result.metrics["hullEnvelopeRemovedPoints"]=double(seams.hullEnvelopeInputPoints-seams.hullEnvelopeRetainedPoints);
        result.metrics["hullEnvelopeColumns"]=double(seams.hullEnvelopeColumns);
        if(seams.frameBased) result.metrics["frameWindowMode"]="neighbor_3_5";
        result.metrics["postprocessMode"]=p["postprocessMode"];
        result.metrics["profileSamplingMode"]=p["profileSamplingMode"];
        result.metrics["profileReferenceScale"]=p["profileReferenceScale"];
        result.metrics["interiorOpeningDirection"]=seams.interiorOpeningDirection;
        result.metrics["positiveInteriorStations"]=double(seams.positiveInteriorStations);
        result.metrics["negativeInteriorStations"]=double(seams.negativeInteriorStations);
        result.metrics["interiorOrientationAmbiguous"]=seams.interiorOrientationAmbiguous;
        result.metrics["leftRtsMeanCorrection"]=seams.leftSmoothing.meanCorrection;
        result.metrics["rightRtsMeanCorrection"]=seams.rightSmoothing.meanCorrection;
        result.metrics["leftRtsMaxCorrection"]=seams.leftSmoothing.maxCorrection;
        result.metrics["rightRtsMaxCorrection"]=seams.rightSmoothing.maxCorrection;
        result.metrics["leftRtsOutliers"]=double(seams.leftSmoothing.innovationOutliers);
        result.metrics["rightRtsOutliers"]=double(seams.rightSmoothing.innovationOutliers);
        result.metrics["diagnostics"]=diagnosticReport(seams,source,p);
        result.metrics["processingMode"]=p["processingMode"];
        result.metrics["leftSegments"]=int(seams.leftSegmentStarts.size());
        result.metrics["rightSegments"]=int(seams.rightSegmentStarts.size());
        if(protectedSeams) {
            // Show only the protected measured body, not the rejected raw scan
            // or its flattened/reprojected working copy. Preserve all fields.
            result.outputs.push_back(body.outputs.front());
            result.layers.removeFirst();result.layers.removeFirst();
            result.layers.prepend(body.layers.front());
            auto report=result.metrics["diagnostics"].toObject();
            auto outputs=report["outputs"].toArray();auto issues=report["issues"].toArray();
            const auto bodyReport=body.metrics["diagnostics"].toObject();
            auto bodyOutput=bodyReport["outputs"].toArray().first().toObject();bodyOutput["cloudIndex"]=2;outputs.append(bodyOutput);
            for(const auto& issue:bodyReport["issues"].toArray())issues.append(issue);
            for(const char* target:{"left_weld_seam","right_weld_seam"})
                issues.append(QJsonObject{{"code","seam_position_unverified"},{"stage","geometry"},{"severity","info"},
                  {"origin","undetermined"},{"visibility","unknown"},
                  {"scope",QJsonObject{{"targetId",target},{"outputRole","fitted"}}},{"region",QJsonObject{{"kind","whole"}}},
                  {"summary",QStringLiteral("当前交线由受保护实测点的局部面模型识别；未提供人工标注真值，帧覆盖和内部置信度不能证明全长定位正确。")},
                  {"evidence",QJsonObject{{"support",1},{"supportUnit","count"},{"basis","fitted_output_without_external_ground_truth"}}}});
            report["outputs"]=outputs;report["issues"]=issues;result.metrics["diagnostics"]=report;
            for(auto it=body.metrics.begin();it!=body.metrics.end();++it)
                if(it.key()!="diagnostics"&&it.key()!="processingMode")result.metrics[it.key()]=it.value();
            result.metrics["secondaryFilteringApplied"]=false;
            result.metrics["protectedFrameCount"]=int(frames.size());
            result.metrics["frameCoordinateMode"]=frames.empty()?"spatial_sections":"orthonormal_scan_plane";
            result.metrics["effectiveProfileBinSize"]=config.profileBinSize;
            result.metrics["effectiveMinSeamSeparation"]=config.minSeamSeparation;
            result.message=QStringLiteral("主体保护后识别：左 %1 点 / %2 段，右 %3 点 / %4 段。长缺口不连线；短插补点在 quality_flags 中标记。位置与全长覆盖仍需对照实测点复核。")
                .arg(seams.leftSeam->size()).arg(seams.leftSegmentStarts.size()).arg(seams.rightSeam->size()).arg(seams.rightSegmentStarts.size());
        }
        context.progress(100); return result;
    }
};
class ExtractPlugin final : public QObject,public fw::IAlgorithmPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_ALGORITHM_IID FILE "metadata.json")
    Q_INTERFACES(fw::IAlgorithmPlugin)
public:
    fw::AlgorithmDescriptor descriptor() const override {return {AlgorithmId,QStringLiteral("ExtractFeature2：三面保留与焊缝提取"),"1.9.0",{"x","y","z"}};}
    QJsonObject parameterSchema() const override {return schema();}
    fw::IAlgorithm* create() override {return new ExtractAlgorithm;}
    void destroy(fw::IAlgorithm* a) override {delete a;}
    bool migrateParameters(int fromVersion,const fw::ParameterSet& values,
                           fw::ParameterSet& migrated,QString& error) const override {
        if(fromVersion!=3&&fromVersion!=4&&fromVersion!=5&&fromVersion!=6&&fromVersion!=7) {error=QStringLiteral("只支持从 ExtractFeature2 参数版本 3 至 7 迁移。");return false;}
        migrated=values;
        if(!migrated.contains("hullPreservationMargin"))migrated["hullPreservationMargin"]=3.0;
        if(!migrated.contains("protectedProfileBinSize"))migrated["protectedProfileBinSize"]=0.6;
        if(!migrated.contains("protectedMinSeamSeparation"))migrated["protectedMinSeamSeparation"]=6.0;
        if(!migrated.contains("processingMode"))migrated["processingMode"]="surfaces_only";
        if(!migrated.contains("hullSurfaceVoxelSize"))migrated["hullSurfaceVoxelSize"]=1.0;
        if(!migrated.contains("hullSurfaceCurvature"))migrated["hullSurfaceCurvature"]=0.035;
        if(!migrated.contains("hullSurfaceDistance"))migrated["hullSurfaceDistance"]=0.6;
        if(!migrated.contains("interiorOpeningDirection")) migrated["interiorOpeningDirection"]="auto";
        if(!migrated.contains("useHullEnvelopeCrop")) migrated["useHullEnvelopeCrop"]=true;
        if(!migrated.contains("hullEnvelopeGridSize")) migrated["hullEnvelopeGridSize"]=2.0;
        if(!migrated.contains("hullEnvelopeMinCellSupport")) migrated["hullEnvelopeMinCellSupport"]=8;
        if(!migrated.contains("hullEnvelopeMinThickness")) migrated["hullEnvelopeMinThickness"]=8.0;
        if(!migrated.contains("hullEnvelopeMargin")) migrated["hullEnvelopeMargin"]=4.0;
        if(!migrated.contains("hullEnvelopeSmoothingRadius")) migrated["hullEnvelopeSmoothingRadius"]=4;
        return true;
    }
};
}
#include "Plugin.moc"
