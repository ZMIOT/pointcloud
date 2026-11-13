#include <fw/Parameters.h>
#include <QFileInfo>
#include <QRegularExpression>
#include <QSet>
#include <QMap>
#include <cmath>
#include <functional>
#include <limits>

namespace fw {
namespace {
const QSet<QString> types={"bool","int","double","string","enum","range","file","directory","vector3","color"};
bool number(QJsonValue value, bool integer=false) {
    return value.isDouble() && std::isfinite(value.toDouble()) && (!integer || value.toDouble()==std::floor(value.toDouble()));
}
QString checkValue(const QJsonObject& p, const QJsonValue& v, bool checkPath=true) {
    const auto type=p["type"].toString();
    auto numeric=[&](const QJsonValue& n, bool integer) -> QString {
        if(!number(n,integer)) return QStringLiteral("需要有限%1数值").arg(integer?QStringLiteral("整数"):QStringLiteral("浮点"));
        if(p.contains("min")&&n.toDouble()<p["min"].toDouble()) return QStringLiteral("小于允许的最小值");
        if(p.contains("max")&&n.toDouble()>p["max"].toDouble()) return QStringLiteral("大于允许的最大值");
        if(integer&&(n.toDouble()<std::numeric_limits<int>::min()||n.toDouble()>std::numeric_limits<int>::max())) return QStringLiteral("超出 32 位整数范围");
        if(!integer&&p.contains("decimals")) {
            const double scale=std::pow(10.0,p["decimals"].toInt());
            const double rounded=std::round(n.toDouble()*scale)/scale;
            if(std::isfinite(rounded)&&std::abs(rounded-n.toDouble())>std::max(1e-14,std::abs(n.toDouble())*1e-12)) return QStringLiteral("超过允许的小数精度");
        }
        return {};
    };
    if(type=="bool") return v.isBool()?QString():QStringLiteral("需要布尔值");
    if(type=="int"||type=="double") return numeric(v,type=="int");
    if(type=="range") {
        if(!v.isObject()) return QStringLiteral("范围必须包含 lower/upper");
        const auto o=v.toObject();
        auto a=numeric(o["lower"],p["valueType"]=="int"), b=numeric(o["upper"],p["valueType"]=="int");
        if(!a.isEmpty()) return a; if(!b.isEmpty()) return b;
        if(o["lower"].toDouble()>o["upper"].toDouble()) return QStringLiteral("下限不能大于上限");
        return {};
    }
    if(type=="vector3") {
        if(!v.isArray()||v.toArray().size()!=3) return QStringLiteral("需要三个坐标 [x,y,z]");
        for(auto n:v.toArray()) { auto e=numeric(n,false); if(!e.isEmpty()) return e; } return {};
    }
    if(type=="color") {
        if(!v.isArray()||v.toArray().size()!=4) return QStringLiteral("需要 RGBA 四个分量");
        for(auto n:v.toArray()) if(!number(n,true)||n.toInt()<0||n.toInt()>255) return QStringLiteral("颜色分量范围为 0..255");
        return {};
    }
    if(type=="enum") {
        for(auto option:p["options"].toArray()) if(option.toObject()["value"]==v) return {};
        return QStringLiteral("不在可选值中");
    }
    if(type=="string"||type=="file"||type=="directory") {
        if(!v.isString()) return QStringLiteral("需要文本值");
        const auto s=v.toString();
        if(p["required"].toBool()&&s.trimmed().isEmpty()) return QStringLiteral("不能为空");
        if(p.contains("maxLength")&&s.size()>p["maxLength"].toInt()) return QStringLiteral("文本过长");
        if(p.contains("pattern")&&!QRegularExpression(QRegularExpression::anchoredPattern(p["pattern"].toString())).match(s).hasMatch()) return QStringLiteral("文本格式不匹配");
        if(checkPath&&p["mustExist"].toBool()&&!s.isEmpty()) {
            QFileInfo f(s);
            if(type=="file"&&!f.isFile()) return QStringLiteral("文件不存在");
            if(type=="directory"&&!f.isDir()) return QStringLiteral("目录不存在");
        }
        return {};
    }
    return QStringLiteral("未知参数类型");
}
void refs(const QJsonValue& value, QStringList& found, QStringList& errors) {
    if(value.isUndefined()) return;
    if(!value.isObject()) { errors<<QStringLiteral("联动规则必须为对象"); return; }
    const auto rule=value.toObject();
    if(rule.contains("all")||rule.contains("any")) {
        const auto key=rule.contains("all")?QStringLiteral("all"):QStringLiteral("any");
        if(rule.size()!=1||!rule[key].isArray()||rule[key].toArray().isEmpty()) errors<<QStringLiteral("all/any 必须包含非空规则数组");
        for(auto item:rule[key].toArray()) refs(item,found,errors);
    } else {
        static const QSet<QString> ops={"eq","ne","lt","le","gt","ge"};
        if(rule["parameter"].toString().isEmpty()||!ops.contains(rule["operator"].toString())||!rule.contains("value")) errors<<QStringLiteral("联动规则缺少参数、运算符或值");
        if(rule["operator"]!="eq"&&rule["operator"]!="ne"&&!number(rule["value"])) errors<<QStringLiteral("大小比较需要数值条件");
        found<<rule["parameter"].toString();
    }
}
}
QJsonArray Parameters::definitions(const QJsonObject& s) {
    QJsonArray result;
    for(auto g:s["groups"].toArray()) for(auto p:g.toObject()["parameters"].toArray()) result.append(p);
    return result;
}
ParameterSet Parameters::defaults(const QJsonObject& s) {
    ParameterSet result;
    for(auto v:definitions(s)) { const auto p=v.toObject(); result[p["id"].toString()]=p["default"]; }
    return result;
}
QStringList Parameters::validateSchema(const QJsonObject& s) {
    QStringList errors;
    if(s["schemaVersion"]!=1) errors<<QStringLiteral("仅支持 schemaVersion=1");
    if(s["algorithmId"].toString().isEmpty()) errors<<QStringLiteral("缺少 algorithmId");
    if(!number(s["parameterVersion"],true)||s["parameterVersion"].toInt()<1) errors<<QStringLiteral("parameterVersion 必须为正整数");
    if(!s["groups"].isArray()) errors<<QStringLiteral("groups 必须为数组");
    QSet<QString> ids, groups; QMap<QString,QStringList> graph;
    for(auto value:s["groups"].toArray()) {
        auto group=value.toObject(); auto id=group["id"].toString();
        if(id.isEmpty()||groups.contains(id)) errors<<QStringLiteral("分组 ID 为空或重复：")+id;
        groups.insert(id);
        if(!group["parameters"].isArray()) errors<<QStringLiteral("parameters 必须为数组：")+id;
    }
    for(auto value:definitions(s)) {
        const auto p=value.toObject(); const auto id=p["id"].toString(), type=p["type"].toString();
        if(id.isEmpty()||ids.contains(id)) errors<<QStringLiteral("参数 ID 为空或重复：")+id;
        ids.insert(id);
        if(!types.contains(type)) errors<<id+QStringLiteral("：不支持的类型 ")+type;
        for(const auto& key:{"min","max","step"}) if(p.contains(key)&&!number(p[key])) errors<<id+":"+key+QStringLiteral(" 必须是有限数值");
        if(type=="int"||(type=="range"&&p["valueType"]=="int")) {
            for(const auto& key:{"min","max","step"}) if(p.contains(key)&&(!number(p[key],true)||p[key].toDouble()<std::numeric_limits<int>::min()||p[key].toDouble()>std::numeric_limits<int>::max())) errors<<id+":"+key+QStringLiteral(" 必须是 32 位整数");
        }
        for(const auto& key:{"required","advanced","multiline","mustExist"}) if(p.contains(key)&&!p[key].isBool()) errors<<id+":"+key+QStringLiteral(" 必须是布尔值");
        if(p.contains("min")&&p.contains("max")&&p["min"].toDouble()>p["max"].toDouble()) errors<<id+QStringLiteral("：min 大于 max");
        if(p.contains("step")&&p["step"].toDouble()<=0) errors<<id+QStringLiteral("：step 必须大于零");
        if(p.contains("decimals")&&(!number(p["decimals"],true)||p["decimals"].toInt()<0||p["decimals"].toInt()>15)) errors<<id+QStringLiteral("：decimals 范围为 0..15");
        if(type=="range"&&p["valueType"]!="int"&&p["valueType"]!="double") errors<<id+QStringLiteral("：range 必须指定 int/double valueType");
        if(p.contains("maxLength")&&(!number(p["maxLength"],true)||p["maxLength"].toInt()<0)) errors<<id+QStringLiteral("：maxLength 必须是非负整数");
        if(p.contains("pattern")&&(!p["pattern"].isString()||!QRegularExpression(p["pattern"].toString()).isValid())) errors<<id+QStringLiteral("：正则表达式无效");
        if(type=="enum") {
            QJsonArray values;
            if(p["options"].toArray().isEmpty()) errors<<id+QStringLiteral("：选项不能为空");
            for(auto item:p["options"].toArray()) {
                auto o=item.toObject(); auto v=o["value"];
                if(!o["label"].isString()||(!v.isString()&&!v.isDouble()&&!v.isBool())||values.contains(v)) errors<<id+QStringLiteral("：选项无效或重复");
                values.append(v);
            }
        }
        const auto e=checkValue(p,p["default"],false);
        if(!e.isEmpty()) errors<<id+QStringLiteral(" 默认值：")+e;
        refs(p["visibleWhen"],graph[id],errors); refs(p["enabledWhen"],graph[id],errors);
    }
    for(auto it=graph.begin();it!=graph.end();++it) for(auto ref:it.value()) if(!ids.contains(ref)) errors<<it.key()+QStringLiteral(" 引用了不存在的参数：")+ref;
    QMap<QString,int> states;
    std::function<void(const QString&)> visit=[&](const QString& id) {
        if(states.value(id)==1) { errors<<QStringLiteral("参数联动存在循环：")+id; return; }
        if(states.value(id)==2) return;
        states[id]=1; for(auto ref:graph.value(id)) if(ids.contains(ref)) visit(ref); states[id]=2;
    };
    for(auto id:ids) visit(id);
    errors.removeDuplicates(); return errors;
}
QStringList Parameters::validate(const QJsonObject& s,const ParameterSet& values) {
    QStringList errors;
    for(auto value:definitions(s)) {
        const auto p=value.toObject(); const auto id=p["id"].toString();
        const auto e=checkValue(p,values[id]); if(!e.isEmpty()) errors<<p["label"].toString(id)+" ("+id+"): "+e;
    }
    return errors;
}
ParameterSet Parameters::merge(const QJsonObject& s,const ParameterSet& input,QStringList* warnings) {
    auto output=defaults(s);
    for(auto it=input.begin();it!=input.end();++it) {
        if(output.contains(it.key())) output[it.key()]=it.value();
        else if(warnings) *warnings<<QStringLiteral("忽略未知参数：")+it.key();
    }
    return output;
}
bool Parameters::condition(const QJsonValue& value,const ParameterSet& values) {
    if(value.isUndefined()) return true;
    const auto r=value.toObject();
    if(r.contains("all")) { for(auto child:r["all"].toArray()) if(!condition(child,values)) return false; return true; }
    if(r.contains("any")) { for(auto child:r["any"].toArray()) if(condition(child,values)) return true; return false; }
    const auto a=values[r["parameter"].toString()], b=r["value"]; const auto op=r["operator"].toString();
    if(op=="eq") return a==b; if(op=="ne") return a!=b;
    if(!a.isDouble()||!b.isDouble()) return false;
    if(op=="lt") return a.toDouble()<b.toDouble(); if(op=="le") return a.toDouble()<=b.toDouble();
    if(op=="gt") return a.toDouble()>b.toDouble(); if(op=="ge") return a.toDouble()>=b.toDouble();
    return false;
}
}
