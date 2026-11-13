#include <fw/Diagnostics.h>
#include <fw/Parameters.h>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSet>
#include <QMap>
#include <cmath>

namespace fw {
namespace {
bool finite(const QJsonValue& v) { return v.isDouble() && std::isfinite(v.toDouble()); }
bool positiveInteger(const QJsonValue& v) { return finite(v) && v.toDouble() >= 0 && std::floor(v.toDouble()) == v.toDouble(); }
bool token(const QJsonValue& v) { return v.isString() && !v.toString().trimmed().isEmpty() && v.toString().size() <= 128; }
bool oneOf(const QJsonValue& v, const QSet<QString>& allowed) { return token(v) && allowed.contains(v.toString()); }
void requiredToken(QStringList& errors, const QJsonObject& object, const QString& key, const QString& path) {
    if(!token(object.value(key))) errors << path + "." + key + QStringLiteral(" 必须为非空文本");
}
const QSet<QString> geometries={"points","polyline","surface","region","measurement"};
const QSet<QString> severities={"info","warning","critical"};
const QSet<QString> visibilities={"observable","missing_input","not_applicable","unknown"};
const QSet<QString> origins={"undetermined","input_missing","acquisition_noise_suspected","physical_defect_suspected","physical_defect_confirmed","algorithm_error_confirmed"};
const QSet<QString> regionKinds={"whole","section_range","bbox","point_index"};
const QSet<QString> roles={"measured","fitted","segmented","measurement","auxiliary"};
void validateRegion(QStringList& errors, const QJsonObject& region, const QString& path, quint64 pointCount) {
    const auto kind=region.value("kind");
    if(!oneOf(kind,regionKinds)) { errors<<path+QStringLiteral(".kind 不支持"); return; }
    if(kind=="point_index") {
        if(!positiveInteger(region.value("index"))||region.value("index").toDouble()>=double(pointCount)) errors<<path+QStringLiteral(".index 超出输入范围");
    } else if(kind=="section_range") {
        if(!positiveInteger(region.value("first"))||!positiveInteger(region.value("last"))||region.value("first").toDouble()>region.value("last").toDouble()) errors<<path+QStringLiteral(" 截面范围无效");
        const auto p=region.value("representative").toArray();
        if(p.size()!=3||!finite(p.at(0))||!finite(p.at(1))||!finite(p.at(2))) errors<<path+QStringLiteral(".representative 需要有限的 XYZ 坐标");
        if(region.contains("sourceIndex")&&(!positiveInteger(region.value("sourceIndex"))||region.value("sourceIndex").toDouble()>=double(pointCount))) errors<<path+QStringLiteral(".sourceIndex 超出输入范围");
    } else if(kind=="bbox") {
        const auto a=region.value("min").toArray(), b=region.value("max").toArray();
        if(a.size()!=3||b.size()!=3) errors<<path+QStringLiteral(" 包围盒需要 min/max XYZ");
        else for(int i=0;i<3;++i) if(!finite(a.at(i))||!finite(b.at(i))||a.at(i).toDouble()>b.at(i).toDouble()) errors<<path+QStringLiteral(" 包围盒坐标无效");
    }
}
}
QStringList Diagnostics::validateProfile(const QJsonObject& p, const QJsonObject& schema, const AlgorithmDescriptor& descriptor) {
    QStringList errors;
    if(p.value("protocolVersion")!=Version) errors<<QStringLiteral("diagnostics.protocolVersion 仅支持 1");
    if(p.value("algorithmId")!=descriptor.id||p.value("algorithmVersion")!=descriptor.version) errors<<QStringLiteral("diagnostics 算法 ID/版本不匹配");
    if(QJsonDocument(p).toJson(QJsonDocument::Compact).size()>MaxProfileBytes) errors<<QStringLiteral("diagnostics 档案超过 64 KiB");
    if(!p.value("orderedInput").isBool()) errors<<QStringLiteral("diagnostics.orderedInput 必须为布尔值");
    if(!p.value("stages").isArray()||p.value("stages").toArray().isEmpty()) errors<<QStringLiteral("diagnostics.stages 不能为空");
    QSet<QString> stages, codes, outputs, parameters;
    for(const auto& v:p.value("stages").toArray()) {
        const auto o=v.toObject(); const auto id=o.value("id").toString();
        if(!token(o.value("id"))||stages.contains(id)||!token(o.value("label"))) errors<<QStringLiteral("diagnostics.stages 的 ID/标签无效或重复");
        stages.insert(id);
    }
    if(!p.value("outputs").isArray()||p.value("outputs").toArray().isEmpty()) errors<<QStringLiteral("diagnostics.outputs 不能为空");
    for(const auto& v:p.value("outputs").toArray()) {
        const auto o=v.toObject(); const auto id=o.value("id").toString();
        if(!token(o.value("id"))||outputs.contains(id)||!token(o.value("targetId"))||!oneOf(o.value("role"),roles)||!oneOf(o.value("geometry"),geometries)||
           (o.contains("targetLabel")&&!token(o.value("targetLabel")))) errors<<QStringLiteral("diagnostics.outputs 的身份/角色/几何类型无效或重复");
        outputs.insert(id);
    }
    if(!p.value("codes").isArray()) errors<<QStringLiteral("diagnostics.codes 必须为数组");
    for(const auto& v:p.value("codes").toArray()) {
        const auto o=v.toObject(); const auto id=o.value("id").toString();
        if(!token(o.value("id"))||codes.contains(id)||!stages.contains(o.value("stage").toString())||!oneOf(o.value("defaultSeverity"),severities)||!token(o.value("label"))) errors<<QStringLiteral("diagnostics.codes 的代码/阶段/严重程度无效或重复");
        codes.insert(id);
    }
    QMap<QString,QString> schemaTypes; for(const auto& v:Parameters::definitions(schema)) {const auto o=v.toObject(); schemaTypes[o.value("id").toString()]=o.value("type").toString();}
    if(!p.value("trialParameters").isArray()) errors<<QStringLiteral("diagnostics.trialParameters 必须为数组");
    for(const auto& v:p.value("trialParameters").toArray()) {
        const auto o=v.toObject(); const auto id=o.value("id").toString();
        if(!schemaTypes.contains(id)||parameters.contains(id)||!finite(o.value("step"))||o.value("step").toDouble()<=0||!o.value("maySuppressPhysicalDefect").isBool()||!token(o.value("expectedChange"))||!token(o.value("risk"))||!token(o.value("cannotFix"))||
           (schemaTypes.value(id)!="double"&&schemaTypes.value(id)!="int")||
           (schemaTypes.value(id)=="int"&&std::floor(o.value("step").toDouble())!=o.value("step").toDouble())) errors<<QStringLiteral("diagnostics.trialParameters 声明无效或重复：")+id;
        for(const auto& stage:o.value("stages").toArray()) if(!stages.contains(stage.toString())) errors<<QStringLiteral("试参参数引用未知阶段：")+id;
        if(!o.value("stages").isArray()||o.value("stages").toArray().isEmpty()) errors<<QStringLiteral("试参参数缺少作用阶段：")+id;
        parameters.insert(id);
    }
    errors.removeDuplicates(); return errors;
}
QStringList Diagnostics::validateReport(const QJsonObject& r, const QJsonObject& p, const AlgorithmDescriptor& d, const AlgorithmInput& input, const ParameterSet& params, const AlgorithmResult& result) {
    QStringList errors;
    const int outputCount=result.outputs.size();
    if(r.value("protocolVersion")!=Version) errors<<QStringLiteral("report.protocolVersion 仅支持 1");
    if(r.value("algorithmId")!=d.id||r.value("algorithmVersion")!=d.version) errors<<QStringLiteral("report 算法 ID/版本不匹配");
    if(QJsonDocument(r).toJson(QJsonDocument::Compact).size()>MaxReportBytes) errors<<QStringLiteral("report 超过 256 KiB");
    const auto in=r.value("input").toObject();
    if(input.clouds.isEmpty()) errors<<QStringLiteral("report 无对应输入");
    else {
        const auto& cloud=input.clouds.front();
        const quint64 count=cloud.cloud?quint64(cloud.cloud->width)*cloud.cloud->height:0;
        if(in.value("cloudId")!=cloud.id||!positiveInteger(in.value("pointCount"))||in.value("pointCount").toDouble()!=double(count)||in.value("coordinateFrame")!=cloud.coordinateFrame||in.value("unit")!=cloud.unit) errors<<QStringLiteral("report 输入身份/点数/坐标系/单位不匹配");
    }
    if(!r.value("parameters").isObject()||r.value("parameters").toObject()!=params) errors<<QStringLiteral("report 参数快照不匹配");
    QSet<QString> stages,codes,outputIds,outputScopes;
    QMap<QString,QString> codeStages;
    for(const auto& v:p.value("stages").toArray()) stages.insert(v.toObject().value("id").toString());
    for(const auto& v:p.value("codes").toArray()) {const auto o=v.toObject(); codes.insert(o.value("id").toString()); codeStages[o.value("id").toString()]=o.value("stage").toString();}
    for(const auto& v:p.value("outputs").toArray()) {const auto o=v.toObject(); outputIds.insert(o.value("id").toString()); outputScopes.insert(o.value("targetId").toString()+"|"+o.value("role").toString());}
    if(!r.value("outputs").isArray()) errors<<QStringLiteral("report.outputs 必须为数组");
    QSet<QString> seen,seenIndices;
    for(const auto& v:r.value("outputs").toArray()) {
        const auto o=v.toObject(); const auto id=o.value("id").toString();
        if(!outputIds.contains(id)||seen.contains(id)||!positiveInteger(o.value("cloudIndex"))||o.value("cloudIndex").toInt()>=outputCount||seenIndices.contains(QString::number(o.value("cloudIndex").toInt()))) errors<<QStringLiteral("report 输出身份或索引无效：")+id;
        seen.insert(id); seenIndices.insert(QString::number(o.value("cloudIndex").toInt()));
        if(o.contains("statistics")) {
            const auto stats=o.value("statistics").toObject();
            for(const auto& field:{"sampleCount","segmentCount","interpolatedCount"})
                if(stats.contains(field)&&!positiveInteger(stats.value(field))) errors<<QStringLiteral("report 输出统计无效：")+id+"."+field;
            if(stats.contains("sampleCount")&&stats.contains("interpolatedCount")&&
               stats.value("interpolatedCount").toDouble()>stats.value("sampleCount").toDouble()) errors<<QStringLiteral("report 插补数量超过样本数量：")+id;
            if(positiveInteger(o.value("cloudIndex"))&&o.value("cloudIndex").toInt()<outputCount&&stats.contains("sampleCount")) {
                const auto& cloud=result.outputs.at(o.value("cloudIndex").toInt()).cloud;
                if(!cloud||stats.value("sampleCount").toDouble()!=double(quint64(cloud->width)*cloud->height)) errors<<QStringLiteral("report 样本数量与实际输出不符：")+id;
            }
        }
    }
    if(outputCount>0&&seen.size()!=outputCount) errors<<QStringLiteral("report 未给每个输出建立身份映射");
    if(!r.value("issues").isArray()||r.value("issues").toArray().size()>128) errors<<QStringLiteral("report.issues 必须为至多 128 项数组");
    const quint64 count=input.clouds.isEmpty()||!input.clouds.front().cloud?0:quint64(input.clouds.front().cloud->width)*input.clouds.front().cloud->height;
    int i=0;
    for(const auto& v:r.value("issues").toArray()) {
        const auto o=v.toObject(); const auto path=QStringLiteral("issues[%1]").arg(i++);
        if(!codes.contains(o.value("code").toString())) errors<<path+QStringLiteral(" 未知代码");
        if(!stages.contains(o.value("stage").toString())) errors<<path+QStringLiteral(" 未知阶段");
        if(codes.contains(o.value("code").toString())&&codeStages.value(o.value("code").toString())!=o.value("stage").toString()) errors<<path+QStringLiteral(" 代码与阶段不匹配");
        if(!oneOf(o.value("severity"),severities)) errors<<path+QStringLiteral(" 严重程度无效");
        if(!oneOf(o.value("visibility"),visibilities)) errors<<path+QStringLiteral(" 可观测性无效");
        if(!oneOf(o.value("origin"),origins)) errors<<path+QStringLiteral(" 异常来源状态无效");
        requiredToken(errors,o,"summary",path);
        if(!o.value("region").isObject()) errors<<path+QStringLiteral(" 缺少区域"); else validateRegion(errors,o.value("region").toObject(),path+".region",count);
        const auto scope=o.value("scope").toObject();
        if(!token(scope.value("targetId"))||!token(scope.value("outputRole"))) errors<<path+QStringLiteral(" 缺少目标身份/输出角色");
        else if(!outputScopes.contains(scope.value("targetId").toString()+"|"+scope.value("outputRole").toString())) errors<<path+QStringLiteral(" 目标身份/输出角色未声明");
        const auto evidence=o.value("evidence").toObject();
        if(!positiveInteger(evidence.value("support"))||!token(evidence.value("supportUnit"))||!token(evidence.value("basis"))) errors<<path+QStringLiteral(" 证据支持量/口径无效");
        if(evidence.contains("observed")&&!finite(evidence.value("observed"))) errors<<path+QStringLiteral(" observed 非有限数值");
        if(evidence.contains("limit")&&!finite(evidence.value("limit"))) errors<<path+QStringLiteral(" limit 非有限数值");
        if((evidence.contains("observed")||evidence.contains("limit"))&&!token(evidence.value("unit"))) errors<<path+QStringLiteral(" 数值证据缺少单位");
    }
    errors.removeDuplicates(); return errors;
}
}
