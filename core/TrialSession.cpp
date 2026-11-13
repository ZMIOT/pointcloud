#include "Runtime.h"
#include "TrialSession.h"
#include <fw/Parameters.h>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSet>
#include <cmath>
#include <algorithm>

namespace fw {
QVector<TrialVariant> TrialSession::plan(const QJsonObject& schema, const QJsonObject& profile,
                                        const QJsonObject& report, const ParameterSet& baseline,
                                        int maxVariants) {
    QVector<TrialVariant> result;
    if(maxVariants<=0||report.value("protocolVersion")!=1) return result;
    maxVariants=std::min(maxVariants,4);
    QSet<QString> relevant;
    for(const auto& v:report.value("issues").toArray()) {
        const auto issue=v.toObject();
        if((issue.value("visibility")=="observable"||issue.value("visibility")=="unknown")&&
           (issue.value("origin")=="undetermined"||issue.value("origin")=="algorithm_error_confirmed"))
            if(issue.value("severity")!="info") relevant.insert(issue.value("stage").toString());
    }
    QMap<QString,QJsonObject> definitions;
    for(const auto& v:Parameters::definitions(schema)) { const auto p=v.toObject(); definitions[p.value("id").toString()]=p; }
    for(const auto& v:profile.value("trialParameters").toArray()) {
        const auto rule=v.toObject(); const auto id=rule.value("id").toString();
        bool applies=false; for(const auto& stage:rule.value("stages").toArray()) if(relevant.contains(stage.toString())) applies=true;
        if(!applies) continue;
        const auto definition=definitions.value(id); const auto type=definition.value("type").toString();
        if(type!="double"&&type!="int") continue;
        const auto current=baseline.value(id);
        if(!current.isDouble()) continue;
        const double step=rule.value("step").toDouble();
        if(!std::isfinite(step)||step<=0) continue;
        for(int sign:{-1,1}) {
            auto candidate=baseline;
            double next=current.toDouble()+sign*step;
            if(type=="int") next=std::round(next);
            if(next==current.toDouble()) continue;
            candidate[id]=next;
            if(Parameters::validate(schema,candidate).isEmpty()) result.push_back({id,candidate});
            if(result.size()>=maxVariants) return result;
        }
    }
    return result;
}
namespace {
QString key(const QJsonObject& issue) {
    const auto scope=issue.value("scope").toObject(), region=issue.value("region").toObject();
    // sourceIndex is stable for the same input. A recovered section number alone is not.
    QString location=region.value("kind").toString();
    if(region.contains("sourceIndex")) location+=":"+QString::number(region.value("sourceIndex").toInt());
    else if(location=="point_index") location+=":"+QString::number(region.value("index").toInt());
    else if(location=="section_range") location+=":"+QString::fromUtf8(QJsonDocument(QJsonObject{{"representative",region.value("representative")}}).toJson(QJsonDocument::Compact));
    else if(location=="bbox") location+=":"+QString::fromUtf8(QJsonDocument(region).toJson(QJsonDocument::Compact));
    return scope.value("targetId").toString()+"|"+scope.value("outputRole").toString()+"|"+
           issue.value("code").toString()+"|"+location;
}
}
QJsonObject TrialSession::compare(const QJsonObject& baseline, const QJsonObject& trial) {
    QJsonObject result{{"comparable",false}};
    if(baseline.value("protocolVersion")!=trial.value("protocolVersion")||
       baseline.value("algorithmId")!=trial.value("algorithmId")||
       baseline.value("algorithmVersion")!=trial.value("algorithmVersion")||
       baseline.value("input")!=trial.value("input")) {
        result["reason"]=QStringLiteral("输入或算法身份不一致"); return result;
    }
    QMap<QString,int> before,after; QMap<QString,QJsonObject> sample;
    for(const auto& v:baseline.value("issues").toArray()) { const auto o=v.toObject(); ++before[key(o)]; sample[key(o)]=o; }
    for(const auto& v:trial.value("issues").toArray()) { const auto o=v.toObject(); ++after[key(o)]; sample[key(o)]=o; }
    QJsonArray resolved,added;
    int unchanged=0;
    for(auto it=sample.begin();it!=sample.end();++it) {
        const auto b=before.value(it.key()),a=after.value(it.key());
        unchanged+=std::min(a,b);
        for(int i=0;i<b-a;++i) resolved.append(it.value());
        for(int i=0;i<a-b;++i) added.append(it.value());
    }
    result["comparable"]=true;
    QMap<QString,QJsonObject> baselineOutputs,trialOutputs;
    for(const auto& v:baseline.value("outputs").toArray()) {const auto o=v.toObject(); baselineOutputs[o.value("id").toString()]=o;}
    for(const auto& v:trial.value("outputs").toArray()) {const auto o=v.toObject(); trialOutputs[o.value("id").toString()]=o;}
    QJsonArray lostOutputs,newOutputs,outputChanges;
    for(auto it=baselineOutputs.begin();it!=baselineOutputs.end();++it) {
        if(!trialOutputs.contains(it.key())) {lostOutputs.append(it.key()); continue;}
        const auto beforeStats=it.value().value("statistics").toObject(),afterStats=trialOutputs.value(it.key()).value("statistics").toObject();
        if(beforeStats!=afterStats) outputChanges.append(QJsonObject{{"id",it.key()},{"before",beforeStats},{"after",afterStats}});
    }
    for(auto it=trialOutputs.begin();it!=trialOutputs.end();++it) if(!baselineOutputs.contains(it.key())) newOutputs.append(it.key());
    result["lostOutputs"]=lostOutputs; result["newOutputs"]=newOutputs; result["outputStatisticChanges"]=outputChanges;
    result["disappearedIssues"]=resolved;
    result["newIssues"]=added;
    result["unchangedCount"]=unchanged;
    result["interpretation"]=QStringLiteral("仅比较同输入、同目标和角色的问题条目及输出统计。条目消失、段数或点数变化不证明定位正确；真实物理缺陷可能被平滑抹掉。");
    result["physicalDefectRequiresReview"]=std::any_of(resolved.begin(),resolved.end(),[](const QJsonValue& v) {return v.toObject().value("origin").toString().startsWith("physical_defect");}) ||
        std::any_of(added.begin(),added.end(),[](const QJsonValue& v) {return v.toObject().value("origin").toString().startsWith("physical_defect");});
    const auto unresolved=[](const QJsonValue& v) {const auto origin=v.toObject().value("origin").toString(); return origin=="undetermined"||origin.startsWith("physical_defect");};
    const auto baselineIssues=baseline.value("issues").toArray(), trialIssues=trial.value("issues").toArray();
    const bool uncertainBaseline=std::any_of(baselineIssues.begin(),baselineIssues.end(),unresolved);
    const bool disappearingUncertain=std::any_of(resolved.begin(),resolved.end(),unresolved);
    const bool anyPhysical=std::any_of(baselineIssues.begin(),baselineIssues.end(),[](const QJsonValue& v) {return v.toObject().value("origin").toString().startsWith("physical_defect");}) ||
        std::any_of(trialIssues.begin(),trialIssues.end(),[](const QJsonValue& v) {return v.toObject().value("origin").toString().startsWith("physical_defect");});
    result["reviewRequired"]=disappearingUncertain||anyPhysical||!lostOutputs.isEmpty()||(uncertainBaseline&&!outputChanges.isEmpty());
    return result;
}
bool TrialSession::oneClickAdoptionAllowed(const QJsonObject& profile, const QJsonObject& comparison,
                                           const QString& parameterId) {
    if(!comparison.value("comparable").toBool()||comparison.value("reviewRequired").toBool()||
       !comparison.value("lostOutputs").toArray().isEmpty()||!comparison.value("newIssues").toArray().isEmpty()) return false;
    for(const auto& v:profile.value("trialParameters").toArray()) {
        const auto rule=v.toObject();
        if(rule.value("id")==parameterId) return rule.value("maySuppressPhysicalDefect").isBool()&&
            !rule.value("maySuppressPhysicalDefect").toBool();
    }
    return false;
}
}
