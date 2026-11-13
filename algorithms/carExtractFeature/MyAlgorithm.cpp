#include "core/LapEdge.h"
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <algorithm>
#include <cstring>
#include <fw/Algorithm.h>
#include <fw/Parameters.h>
#include <limits>
#include <map>
#include <set>
#include <pcl/point_types.h>

namespace {
QJsonObject schema() {
    QFile file(":/custom.carExtractFeature/parameters.json");
    if (!file.open(QIODevice::ReadOnly))
        return {};
    return QJsonDocument::fromJson(file.readAll()).object();
}
lap::Config configuration(const fw::ParameterSet &p) {
    lap::Config c;
    const QString up = p["up_axis"].toString("+z");
    c.up = lap::Vec::Zero();
    const int axis = up.endsWith('x') ? 0 : up.endsWith('y') ? 1 : 2;
    c.up[axis] = up.startsWith('-') ? -1 : 1;
    c.neighborhoodScale = p["neighborhood_scale"].toDouble(1);
    c.smoothing = p["smoothing"].toDouble(1);
    c.maxMissing = p["max_missing"].toInt(3);
    if (p["manual_scan_normal"].toBool())
        c.scanNormal =
            lap::Vec(p["scan_nx"].toDouble(), p["scan_ny"].toDouble(1), p["scan_nz"].toDouble());
    return c;
}
fw::CloudData outputCloud(const fw::CloudData &input, const std::vector<lap::Sample> &samples,
                          const QString &suffix) {
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    const std::vector<std::pair<std::string, std::uint8_t>> fields = {
        {"x", 7},          {"y", 7},          {"z", 7},          {"source_index", 6},
        {"section_id", 6}, {"segment_id", 6}, {"confidence", 7}, {"interpolated", 6}};
    for (const auto &f : fields) {
        pcl::PCLPointField field;
        field.name = f.first;
        field.offset = std::uint32_t(cloud->fields.size() * 4);
        field.count = 1;
        field.datatype = f.second;
        cloud->fields.push_back(field);
    }
    cloud->height = 1;
    cloud->width = std::uint32_t(samples.size());
    cloud->point_step = 32;
    cloud->row_step = 32 * cloud->width;
    cloud->is_dense = true;
    cloud->is_bigendian = false;
    cloud->data.resize(cloud->row_step);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto &s = samples[i];
        auto data = cloud->data.data() + i * 32;
        const float xyz[] = {float(s.xyz.x()), float(s.xyz.y()), float(s.xyz.z())};
        std::memcpy(data, xyz, 12);
        std::memcpy(data + 12, &s.source, 4);
        std::memcpy(data + 16, &s.section, 4);
        std::memcpy(data + 20, &s.segment, 4);
        std::memcpy(data + 24, &s.confidence, 4);
        const std::uint32_t flag = s.interpolated ? 1 : 0;
        std::memcpy(data + 28, &flag, 4);
    }
    auto output = input;
    output.id = input.id + suffix;
    output.name = suffix;
    output.cloud = cloud;
    return output;
}
QJsonObject metrics(const lap::Result &r) {
    QJsonObject reasons;
    int eligible = 0, inserted = 0;
    for (const auto &c : r.candidates) {
        const QString reason = lap::reasonName(c.reason);
        reasons[reason] = reasons[reason].toInt() + 1;
        if (c.eligible)
            ++eligible;
    }
    for (const auto &p : r.trajectory)
        if (p.interpolated)
            ++inserted;
    return {
        {"input_points", int(r.input.size())},
        {"sections", int(r.sections.size())},
        {"point_spacing", r.pointSpacing},
        {"section_spacing", r.sectionSpacing},
        {"scan_normal", QJsonArray{r.scanNormal.x(), r.scanNormal.y(), r.scanNormal.z()}},
        {"candidates", int(r.candidates.size())},
        {"surface_validated", eligible},
        {"isolated_points", int(r.isolated.size())},
        {"measured_points", int(r.measured.size())},
        {"trajectory_points", int(r.trajectory.size())},
        {"interpolated_points", inserted},
        {"measured_segments", r.measured.empty() ? 0 : int(r.measured.back().segment) + 1},
        {"trajectory_segments", r.trajectory.empty() ? 0 : int(r.trajectory.back().segment) + 1},
        {"candidate_reasons", reasons}};
}
QJsonObject diagnosticReport(const lap::Result &r, const fw::CloudData &input,
                             const fw::ParameterSet &parameters, bool hasOutputs) {
    QJsonArray issues;
    auto issue = [&](const char *code, const char *stage, const char *severity,
                     const QJsonObject &region, const char *visibility, const QString &summary,
                     int support, const char *basis, const char *role = "measured") {
        issues.append(QJsonObject{{"code", code}, {"stage", stage}, {"severity", severity},
                                  {"region", region}, {"visibility", visibility},
                                  {"origin", "undetermined"},
                                  {"scope", QJsonObject{{"targetId", "upper_lap_edge"},
                                                        {"outputRole", role}}},
                                  {"summary", summary},
                                  {"evidence", QJsonObject{{"support", support},
                                                           {"supportUnit", "count"},
                                                           {"basis", basis}}}});
    };
    const QJsonObject whole{{"kind", "whole"}};
    if(r.sections.empty())
        issue("no_sections", "scan_recovery", "critical", whole, "unknown",
              QStringLiteral("未恢复扫描截面；请检查输入点序和扫描方向。"), 0, "recovered_sections");
    else if(r.candidates.empty())
        issue("no_candidates", "candidate", "warning", whole, "unknown",
              QStringLiteral("有扫描截面，但没有候选点；无法据此断定工件存在漏检。"),
              int(r.sections.size()), "recovered_sections");
    // Merge adjacent recovered sections so a long anomaly becomes one regional event.
    std::map<int, std::pair<int, int>> weakSections, surfaceSections; // section -> (count, representative input index)
    for(const auto &c : r.candidates) {
        const bool isWeak = c.reason == lap::Reason::WeakStep;
        const bool isSurface = c.reason == lap::Reason::NoSurface ||
                               c.reason == lap::Reason::SurfaceMismatch ||
                               c.reason == lap::Reason::ShortSupport ||
                               c.reason == lap::Reason::NoLayerSeparation;
        if(c.section < 0 || c.point < 0 || c.point >= int(r.input.size())) continue;
        auto *sections=isWeak?&weakSections:isSurface?&surfaceSections:nullptr;
        if(sections) {
            auto &summary=(*sections)[c.section];
            if(summary.first==0) summary.second=c.point;
            ++summary.first;
        }
    }
    int listed=0;
    std::set<int> measuredSections;
    for(const auto &sample:r.measured) measuredSections.insert(int(sample.section));
    auto appendRegions=[&](const std::map<int,std::pair<int,int>> &sections,
                           const char *code,const char *stage,const QString &summary) {
        int omitted=0, first=-1, last=-1, count=0, representative=-1;
        auto flush=[&] {
            if(first<0) return;
            if(listed>=16) {omitted+=count; return;}
            const auto &point=r.input[representative]; const auto &p=point.xyz;
            const QJsonObject region{{"kind","section_range"},{"first",first},{"last",last},
                {"sourceIndex",double(point.source)},
                {"representative",QJsonArray{p.x(),p.y(),p.z()}}};
            bool hasUnselectedSection=false;
            for(int section=first;section<=last;++section) if(!measuredSections.count(section)) {hasUnselectedSection=true; break;}
            // A rejected candidate in a section that already has a measured edge is
            // normal competition, not an automatic alarm.
            issue(code,stage,hasUnselectedSection?"warning":"info",region,"unknown",summary,
                  count,"rejected_candidates");
            ++listed;
        };
        for(const auto &entry:sections) {
            if(first>=0 && entry.first>last+1) {flush(); first=-1; count=0;}
            if(first<0) {first=entry.first; representative=entry.second.second;}
            last=entry.first; count+=entry.second.first;
        }
        flush();
        if(omitted) issue(code,stage,"info",whole,"unknown",
                          QStringLiteral("还有未逐段列出的候选拒绝区域。"),
                          omitted,"unlisted_rejected_candidates");
    };
    appendRegions(weakSections,"weak_step","candidate",QStringLiteral("这段扫描截面的候选台阶证据不足。"));
    appendRegions(surfaceSections,"surface_rejected","surface",QStringLiteral("这段扫描截面的候选未通过邻截面曲面验证。"));
    if(!r.measured.empty() && r.measured.back().segment > 0)
        issue("trajectory_fragmented", "selection", "info", whole, "observable",
              QStringLiteral("实测边缘包含多个连续段；断裂可能是真实缺口或证据不足。"),
              int(r.measured.back().segment) + 1, "measured_segments");
    const int inserted = int(std::count_if(r.trajectory.begin(), r.trajectory.end(),
                                          [](const lap::Sample &s) { return s.interpolated; }));
    if(inserted > 0)
        issue("interpolated", "smoothing", "info", whole, "observable",
              QStringLiteral("拟合轨迹含插补点，不能把插补点当作实测点。"),
              inserted, "interpolated_samples", "fitted");
    QJsonArray outputs;
    if(hasOutputs) {
        outputs.append(QJsonObject{{"id", "upper_measured"}, {"cloudIndex", 0},
            {"statistics",QJsonObject{{"sampleCount",double(r.measured.size())},
                                      {"segmentCount",r.measured.empty()?0:int(r.measured.back().segment)+1},
                                      {"interpolatedCount",0}}}});
        outputs.append(QJsonObject{{"id", "upper_trajectory"}, {"cloudIndex", 1},
            {"statistics",QJsonObject{{"sampleCount",double(r.trajectory.size())},
                                      {"segmentCount",r.trajectory.empty()?0:int(r.trajectory.back().segment)+1},
                                      {"interpolatedCount",inserted}}}});
    }
    return QJsonObject{{"protocolVersion", 1}, {"algorithmId", "custom.carExtractFeature"},
                       {"algorithmVersion", "1.0.0"},
                       {"input", QJsonObject{{"cloudId", input.id},
                                             {"pointCount", double(input.cloud ?
                                                quint64(input.cloud->width) * input.cloud->height : 0)},
                                             {"coordinateFrame", input.coordinateFrame},
                                             {"unit", input.unit}}},
                       {"parameters", parameters}, {"outputs", outputs}, {"issues", issues}};
}
fw::DebugLayer pointLayer(const lap::Result &r, const std::vector<int> &ids, const QString &title,
                          std::array<double, 3> color) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int id : ids) {
        const auto &p = r.input[id].xyz;
        cloud.push_back(pcl::PointXYZ(float(p.x()), float(p.y()), float(p.z())));
    }
    auto layer = fw::DebugLayer::result(title, cloud);
    layer.style.color = color;
    layer.style.pointSize = 5;
    return layer;
}
void addPolylines(QVector<fw::DebugLayer> &layers, const std::vector<lap::Sample> &samples) {
    std::size_t start = 0;
    while (start < samples.size()) {
        std::size_t end = start + 1;
        while (end < samples.size() && samples[end].segment == samples[start].segment)
            ++end;
        if (end - start > 1) {
            fw::DebugLayer line;
            line.name = QStringLiteral("拟合轨迹 · 段 %1").arg(samples[start].segment);
            line.geometry.kind = fw::GeometryKind::Polyline;
            line.style.color = {1, 0.12, 0.08};
            line.style.lineWidth = 3;
            for (std::size_t i = start; i < end; ++i) {
                const auto &p = samples[i].xyz;
                line.geometry.vertices.push_back({p.x(), p.y(), p.z()});
            }
            layers.push_back(line);
        }
        start = end;
    }
}
fw::DebugLayer finalTrajectoryPoints(const std::vector<lap::Sample> &samples) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(samples.size());
    for (const auto &sample : samples)
        cloud.push_back(
            pcl::PointXYZ(float(sample.xyz.x()), float(sample.xyz.y()), float(sample.xyz.z())));
    auto layer = fw::DebugLayer::result(QStringLiteral("最终轨迹点"), cloud);
    layer.style.color = {0.12, 1.0, 0.25};
    layer.style.pointSize = 5;
    return layer;
}
void addSurfacePreviews(QVector<fw::DebugLayer> &layers, const lap::Result &r) {
    int lastSection = -100000;
    const int stride = std::max(1, int(r.sections.size()) / 8);
    for (const auto &c : r.candidates) {
        if (!c.eligible || c.section - lastSection < stride)
            continue;
        lastSection = c.section;
        for (int sheet = 0; sheet < 2; ++sheet) {
            const auto &surface = sheet ? c.lower : c.upper;
            const lap::Vec origin = r.local[sheet ? c.other : c.point];
            const double side = sheet ? -c.side : c.side, d = r.sections[c.section].spacing;
            for (int row = -1; row <= 1; ++row) {
                fw::DebugLayer patch;
                patch.name = QStringLiteral("截面 %1 · %2局部曲面")
                                 .arg(c.section)
                                 .arg(sheet ? QStringLiteral("下层") : QStringLiteral("上层"));
                patch.role = fw::LayerRole::Auxiliary;
                patch.geometry.kind = fw::GeometryKind::Polyline;
                patch.style.color =
                    sheet ? std::array<double, 3>{0.2, 0.6, 1} : std::array<double, 3>{1, 0.8, 0.1};
                const double s = origin.y() + row * r.sectionSpacing;
                for (int k = 0; k <= 8; ++k) {
                    const double x = origin.x() + side * 0.5 * k * d, z = surface.height(x, s);
                    const lap::Vec xyz = r.transverse * x + r.scanNormal * s + r.vertical * z;
                    patch.geometry.vertices.push_back({xyz.x(), xyz.y(), xyz.z()});
                }
                layers.push_back(patch);
            }
        }
    }
}

// 调试视图只读取核心结果，不参与选点。每个阶段保留原始点云作背景。
fw::DebugFrame makeDebugFrame(int index, const lap::Result &r, const fw::CloudData &inputCloud) {
    const QStringList titles = {QStringLiteral("恢复扫描截面"), QStringLiteral("疑似孤立杂点"),
                                QStringLiteral("单截面候选"),   QStringLiteral("邻截面曲面验证"),
                                QStringLiteral("全局轨迹选点"), QStringLiteral("受约束平滑轨迹")};

    fw::DebugFrame frame;
    frame.stepId = QString("lap_%1").arg(index);
    frame.title = titles[index];
    frame.metrics = metrics(r);
    frame.message = QStringLiteral(
        "候选拒绝原因见 metrics.candidate_reasons；截面编号为几何恢复编号，不是采集帧号。");

    auto background = fw::DebugLayer::background(QStringLiteral("原始点云"), inputCloud);
    background.style.opacity = 0.3;
    frame.layers.push_back(background);

    std::vector<int> ids;
    if (index == 0) {
        for (std::size_t f = 0; f < r.sections.size();
             f += std::max<std::size_t>(1, r.sections.size() / 20))
            ids.insert(ids.end(), r.sections[f].points.begin(), r.sections[f].points.end());
    }
    if (index == 1)
        ids = r.isolated;
    if (index == 2)
        for (const auto &c : r.candidates)
            if (c.reason == lap::Reason::Pending)
                ids.push_back(c.point);
    if (index == 3) {
        std::map<lap::Reason, std::vector<int>> rejected;
        for (const auto &c : r.candidates)
            if (c.eligible)
                ids.push_back(c.point);
            else
                rejected[c.reason].push_back(c.point);
        for (const auto &group : rejected)
            frame.layers.push_back(pointLayer(r, group.second,
                                              QString::fromLatin1(lap::reasonName(group.first)),
                                              {0.8, 0.3, 0.8}));
        addSurfacePreviews(frame.layers, r);
    }
    if (index >= 4)
        for (int ci : r.selected)
            ids.push_back(r.candidates[ci].point);
    frame.layers.push_back(pointLayer(r, ids, titles[index], {1, 0.65, 0.05}));
    if (index == 5) {
        addPolylines(frame.layers, r.trajectory);
        frame.layers.push_back(finalTrajectoryPoints(r.trajectory));
    }
    return frame;
}
} // namespace
class MyAlgorithm final : public fw::IAlgorithm {
  public:
    QString validate(const fw::AlgorithmInput &input,
                     const fw::ParameterSet &values) const override {
        if (input.clouds.size() != 1 || !input.clouds.front().cloud)
            return QStringLiteral("请选择一份保留扫描点序的原始点云。");
        if (!fw::hasFields(input.clouds.front(), {"x", "y", "z"}))
            return QStringLiteral("点云必须包含 XYZ。");
        const auto p = fw::Parameters::merge(schema(), values);
        const auto errors = fw::Parameters::validate(schema(), p);
        if (!errors.isEmpty())
            return errors.join('\n');
        if (p["manual_scan_normal"].toBool() && configuration(p).scanNormal.norm() < 1e-8)
            return QStringLiteral("手动扫描法向量不能为零。");
        return QString::fromStdString(lap::validateConfig(configuration(p)));
    }
    fw::AlgorithmResult execute(const fw::AlgorithmInput &input, const fw::ParameterSet &values,
                                fw::IExecutionContext &context) override {
        // 入口步骤 1：校验参数，把宿主点云转换为带原始索引的核心输入。
        if (context.isCancelled())
            return fw::AlgorithmResult::cancelled();
        const auto error = validate(input, values);
        if (!error.isEmpty())
            return fw::AlgorithmResult::failed(error);
        const auto parameters = fw::Parameters::merge(schema(), values);
        const auto &inputCloud = input.clouds.front();
        const auto cloud = fw::toCloud<pcl::PointXYZ>(inputCloud);
        if (cloud->size() > std::numeric_limits<std::uint32_t>::max())
            return fw::AlgorithmResult::failed(QStringLiteral("点数超过 source_index 容量。"));
        std::vector<lap::Point> points;
        points.reserve(cloud->size());
        for (std::size_t i = 0; i < cloud->size(); ++i) {
            const auto &p = (*cloud)[i];
            points.push_back({lap::Vec(p.x, p.y, p.z), std::uint32_t(i)});
        }
        lap::Hooks hooks;
        qint64 debugElapsedMs = 0;
        // 入口步骤 2：绑定取消、进度和可选调试回调；调试回调只显示快照。
        hooks.cancelled = [&] { return context.isCancelled(); };
        hooks.progress = [&](int v) { context.progress(v); };
        if (context.debug().enabled())
            hooks.stage = [&](int index, const lap::Result &r) {
                QElapsedTimer checkpointTimer;
                checkpointTimer.start();
                auto frame = makeDebugFrame(index, r, inputCloud);
                const bool keepRunning = context.debug().checkpoint(frame) != fw::DebugAction::Stop;
                debugElapsedMs += checkpointTimer.elapsed();
                return keepRunning;
            };
        // 入口步骤 3：执行与界面无关的识别核心。
        QElapsedTimer algorithmTimer;
        algorithmTimer.start();
        const auto result = lap::detect(points, configuration(parameters), hooks);
        const auto algorithmElapsedMs =
            std::max<qint64>(0, algorithmTimer.elapsed() - debugElapsedMs);
        if (result.cancelled)
            return fw::AlgorithmResult::cancelled();
        if (!result.ok) {
            auto failure = fw::AlgorithmResult::failed(QString::fromStdString(result.message));
            failure.metrics = metrics(result);
            failure.metrics["algorithm_elapsed_ms"] = double(algorithmElapsedMs);
            failure.metrics["diagnostics"] = diagnosticReport(result, inputCloud, parameters, false);
            return failure;
        }
        // 入口步骤 4：分别封装原始实测点、最终轨迹点；红线仅辅助观察点的连接关系。
        fw::AlgorithmResult output;
        output.message =
            QStringLiteral("已提取上层搭接边缘；质量评分不是正确概率。长缺口按段断开。");
        output.metrics = metrics(result);
        output.metrics["algorithm_elapsed_ms"] = double(algorithmElapsedMs);
        output.metrics["diagnostics"] = diagnosticReport(result, inputCloud, parameters, true);
        auto measured = outputCloud(inputCloud, result.measured, QStringLiteral("上层实测边界"));
        auto trajectory = outputCloud(inputCloud, result.trajectory, QStringLiteral("最终轨迹点"));
        output.outputs = {measured, trajectory};
        auto raw = fw::DebugLayer::result(measured.name, measured);
        raw.style.color = {1, 0.75, 0.05};
        raw.style.pointSize = 4;
        auto background = fw::DebugLayer::background(QStringLiteral("原始点云"), inputCloud);
        background.style.color = {0.25, 0.65, 0.75};
        background.style.opacity = 0.45;
        background.style.pointSize = 2;
        auto fit = fw::DebugLayer::result(trajectory.name, trajectory);
        fit.style.color = {0.12, 1.0, 0.25};
        fit.style.pointSize = 5;
        output.layers = {background, raw};
        addPolylines(output.layers, result.trajectory);
        output.layers.push_back(fit);
        context.log(
            QString::fromUtf8(QJsonDocument(output.metrics).toJson(QJsonDocument::Compact)));
        return output;
    }
};
class MyAlgorithmPlugin final : public QObject, public fw::IAlgorithmPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_ALGORITHM_IID FILE "metadata.json")
    Q_INTERFACES(fw::IAlgorithmPlugin)
  public:
    fw::AlgorithmDescriptor descriptor() const override {
        return {"custom.carExtractFeature",
                QStringLiteral("汽车特征提取：上层搭接边缘"),
                "1.0.0",
                {"x", "y", "z"}};
    }
    QJsonObject parameterSchema() const override { return schema(); }
    fw::IAlgorithm *create() override { return new MyAlgorithm; }
    void destroy(fw::IAlgorithm *p) override { delete p; }
};
#include "MyAlgorithm.moc"
