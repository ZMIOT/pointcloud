#include "core/CorrugatedPanel.h"

#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <fw/Algorithm.h>
#include <fw/Parameters.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cstring>
#include <limits>

namespace {

QJsonObject schema() {
    QFile file(":/custom.corrugatedPanelFeature/parameters.json");
    if (!file.open(QIODevice::ReadOnly))
        return {};
    return QJsonDocument::fromJson(file.readAll()).object();
}

bwb::Config configuration(const fw::ParameterSet &parameters) {
    bwb::Config config;
    config.sliceOffsets = {parameters["offset_1"].toDouble(10.0),
                           parameters["offset_2"].toDouble(20.0),
                           parameters["offset_3"].toDouble(30.0)};
    config.sliceThickness = parameters["slice_thickness"].toDouble(3.0);
    config.planeDistance = parameters["plane_distance"].toDouble(0.6);
    config.referenceSegmentLength =
        parameters["reference_segment_length"].toDouble(500.0);
    config.maxAdjacentNormalAngleDegrees =
        parameters["max_adjacent_normal_angle"].toDouble(15.0);
    config.planeIterations = parameters["plane_iterations"].toInt(600);
    config.planeCandidates = parameters["plane_candidates"].toInt(6);
    config.maxRansacPoints = parameters["max_ransac_points"].toInt(60000);
    config.minPlanePoints = parameters["min_plane_points"].toInt(200);
    config.denoiseRadius = parameters["denoise_radius"].toDouble(2.5);
    config.minNeighbors = parameters["min_neighbors"].toInt(3);
    config.bendDeviation = parameters["bend_deviation"].toDouble(2.0);
    config.trajectorySpacing = parameters["trajectory_spacing"].toDouble(2.0);
    config.lineFitTolerance = parameters["line_fit_tolerance"].toDouble(3.0);
    config.whitePointSpacing = parameters["white_point_spacing"].toDouble(10.0);
    config.maxSurfaceExtrapolation = parameters["max_surface_extrapolation"].toDouble(40.0);
    config.surfaceFitTolerance = parameters["surface_fit_tolerance"].toDouble(0.6);
    config.minSurfacePoints = parameters["min_surface_points"].toInt(20);
    config.minIntersectionAngleDegrees = parameters["min_intersection_angle"].toDouble(10.0);
    config.allowProjectionFallback = parameters["allow_projection_fallback"].toBool(true);
    config.threads = parameters["threads"].toInt(0);
    return config;
}

fw::CloudData indexedCloud(const fw::CloudData &input, const std::vector<bwb::Point> &points,
                           const std::vector<int> &indices, const QString &id,
                           const QString &name) {
    std::vector<int> sourceIndices;
    sourceIndices.reserve(indices.size());
    for (int index : indices)
        sourceIndices.push_back(int(points[std::size_t(index)].source));
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::copyPointCloud(*input.cloud, sourceIndices, *cloud);
    fw::CloudData output = input;
    output.id = input.id + id;
    output.name = name;
    output.cloud = cloud;
    return output;
}

fw::CloudData generatedCloud(const fw::CloudData &input,
                             const std::vector<bwb::ProjectedPoint> &points,
                             const QString &id, const QString &name) {
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    const std::pair<const char *, std::uint8_t> definitions[] = {
        {"x", pcl::PCLPointField::FLOAT32}, {"y", pcl::PCLPointField::FLOAT32},
        {"z", pcl::PCLPointField::FLOAT32}, {"source_index", pcl::PCLPointField::UINT32},
        {"is_corner", pcl::PCLPointField::UINT32},
        {"reference_index", pcl::PCLPointField::UINT32},
        {"is_extrapolated", pcl::PCLPointField::UINT32},
        {"extrapolation_distance", pcl::PCLPointField::FLOAT32},
        {"surface_rms", pcl::PCLPointField::FLOAT32},
        {"support_count", pcl::PCLPointField::UINT32},
        {"surface_degree", pcl::PCLPointField::UINT32},
        {"is_projection_fallback", pcl::PCLPointField::UINT32}};
    for (std::size_t i = 0; i < 12; ++i) {
        pcl::PCLPointField field;
        field.name = definitions[i].first;
        field.offset = std::uint32_t(i * 4);
        field.datatype = definitions[i].second;
        field.count = 1;
        cloud->fields.push_back(field);
    }
    cloud->height = 1;
    cloud->width = std::uint32_t(points.size());
    cloud->point_step = 48;
    cloud->row_step = cloud->point_step * cloud->width;
    cloud->is_dense = true;
    cloud->is_bigendian = false;
    cloud->data.resize(cloud->row_step);
    for (std::size_t i = 0; i < points.size(); ++i) {
        auto *destination = cloud->data.data() + i * cloud->point_step;
        const float xyz[] = {float(points[i].xyz.x()), float(points[i].xyz.y()),
                             float(points[i].xyz.z())};
        std::memcpy(destination, xyz, sizeof(xyz));
        std::memcpy(destination + 12, &points[i].source, sizeof(points[i].source));
        const std::uint32_t corner = points[i].corner ? 1u : 0u;
        std::memcpy(destination + 16, &corner, sizeof(corner));
        std::memcpy(destination + 20, &points[i].referenceIndex, 4);
        const std::uint32_t extrapolated = points[i].extrapolated ? 1u : 0u;
        const float distance = float(points[i].extrapolationDistance);
        const float rms = float(points[i].surfaceRms);
        const std::uint32_t degree = std::uint32_t(points[i].surfaceDegree);
        const std::uint32_t projectionFallback = points[i].projectionFallback ? 1u : 0u;
        std::memcpy(destination + 24, &extrapolated, 4);
        std::memcpy(destination + 28, &distance, 4);
        std::memcpy(destination + 32, &rms, 4);
        std::memcpy(destination + 36, &points[i].supportCount, 4);
        std::memcpy(destination + 40, &degree, 4);
        std::memcpy(destination + 44, &projectionFallback, 4);
    }
    fw::CloudData output = input;
    output.id = input.id + id;
    output.name = name;
    output.cloud = cloud;
    return output;
}

QJsonObject resultMetrics(const bwb::Result &result, qint64 elapsedMs) {
    QJsonObject failureCounts;
    QJsonArray failedStations;
    int validStations = 0, intersectionValidStations = 0, quadraticStations = 0;
    int projectionFallbackStations = 0, attemptedFailures = 0;
    int localPatchStations = 0;
    int suppressedLengthSlopeStations = 0;
    double maxPatchSpread = 0;
    double minSeedLongitudinalSpan = std::numeric_limits<double>::infinity();
    double maxRms = 0, maxExtrapolation = 0, maxBoundaryGap = 0;
    for (const auto &station : result.intersectionStations) {
        if (station.localPatchExtension) {
            ++localPatchStations;
            maxPatchSpread = std::max(maxPatchSpread, station.localPatchPredictionSpread);
        }
        suppressedLengthSlopeStations += station.lengthSlopeIdentifiable ? 0 : 1;
        minSeedLongitudinalSpan = std::min(minSeedLongitudinalSpan,
                                           station.seedLongitudinalSpan);
        maxBoundaryGap = std::max(maxBoundaryGap, station.boundaryIntersectionGap);
        if (station.valid) {
            ++validStations;
            projectionFallbackStations += station.point.projectionFallback ? 1 : 0;
        }
        const std::string attemptFailure = !station.intersectionFailure.empty()
            ? station.intersectionFailure : (!station.valid ? station.failure : std::string());
        if (attemptFailure.empty()) {
            ++intersectionValidStations;
            quadraticStations += station.point.surfaceDegree == 2 ? 1 : 0;
            maxRms = std::max(maxRms, station.point.surfaceRms);
            maxExtrapolation = std::max(maxExtrapolation, station.attemptedExtrapolation);
        } else {
            ++attemptedFailures;
            const QString reason = QString::fromStdString(attemptFailure);
            failureCounts[reason] = failureCounts.value(reason).toInt() + 1;
            if (failedStations.size() < 128)
                failedStations.append(QJsonObject{{"longitudinal", station.longitudinal},
                    {"reference_index", int(station.point.referenceIndex)}, {"reason", reason},
                    {"surface_rms", station.point.surfaceRms},
                    {"support_count", int(station.point.supportCount)},
                    {"support_min_height", station.supportMinHeight},
                    {"support_max_height", station.supportMaxHeight},
                    {"extrapolation_distance", station.attemptedExtrapolation}});
        }
    }
    QJsonArray coefficients{result.reference.normal.x(), result.reference.normal.y(),
                            result.reference.normal.z(), result.reference.d};
    QJsonArray slices;
    for (const auto &slice : result.slices)
        slices.append(QJsonObject{{"side", slice.side},
                                  {"offset", slice.offset},
                                  {"raw_points", int(slice.raw.size())},
                                  {"filtered_points", int(slice.filtered.size())},
                                  {"length", slice.length},
                                  {"completeness", slice.completeness},
                                  {"deviation", slice.deviation},
                                  {"linearity_ratio", slice.linearityRatio},
                                  {"bent", slice.bent},
                                  {"score", slice.score}});
    QJsonArray referenceSegments;
    for (std::size_t i = 0; i < result.referenceSegments.size(); ++i) {
        const auto &segment = result.referenceSegments[i];
        referenceSegments.append(
            QJsonObject{{"id", QStringLiteral("A%1").arg(i + 1)},
                        {"longitudinal_min", segment.longitudinalMin},
                        {"longitudinal_max", segment.longitudinalMax},
                        {"point_count", int(segment.indices.size())},
                        {"locally_fitted", segment.locallyFitted},
                        {"plane_rms", segment.planeRms},
                        {"normal_change_from_previous_degrees",
                         segment.normalChangeFromPreviousDegrees},
                        {"boundary_gap_measured", segment.boundaryGapMeasured},
                        {"boundary_gap_from_previous",
                         segment.boundaryGapMeasured
                             ? QJsonValue(segment.boundaryGapFromPrevious)
                             : QJsonValue(QJsonValue::Null)},
                        {"local_origin", QJsonArray{segment.plane.origin.x(),
                                                     segment.plane.origin.y(),
                                                     segment.plane.origin.z()}},
                        {"local_x", QJsonArray{segment.localLongitudinal.x(),
                                                segment.localLongitudinal.y(),
                                                segment.localLongitudinal.z()}},
                        {"local_y", QJsonArray{segment.localTransverse.x(),
                                                segment.localTransverse.y(),
                                                segment.localTransverse.z()}},
                        {"local_z", QJsonArray{segment.plane.normal.x(),
                                                segment.plane.normal.y(),
                                                segment.plane.normal.z()}},
                        {"plane", QJsonArray{segment.plane.normal.x(),
                                             segment.plane.normal.y(),
                                             segment.plane.normal.z(),
                                             segment.plane.d}}});
    }
    return {{"algorithm_elapsed_ms", double(elapsedMs)},
            {"surface_intersection", QJsonObject{
                {"method", result.usedProjectionFallback
                    ? "whole_curve_B_to_A_projection_fallback"
                    : "off_edge_body_surface_extension"},
                {"point_provenance", result.usedProjectionFallback
                    ? "projected_fallback_not_surface_intersection"
                    : "nominal_body_extension_not_measured_edge"},
                {"support_region", "offset_slice_body_band_excluding_edge"},
                {"body_band_min", result.bodyBandMin},
                {"body_band_max", result.bodyBandMax},
                {"used_projection_fallback", result.usedProjectionFallback},
                {"stations", int(result.intersectionStations.size())},
                {"valid_stations", validStations},
                {"intersection_valid_stations", intersectionValidStations},
                {"projection_fallback_stations", projectionFallbackStations},
                {"quadratic_stations", quadraticStations},
                {"local_patch_validated_stations", localPatchStations},
                {"max_local_patch_prediction_spread", maxPatchSpread},
                {"length_slope_suppressed_stations", suppressedLengthSlopeStations},
                {"min_seed_longitudinal_span", std::isfinite(minSeedLongitudinalSpan)
                    ? minSeedLongitudinalSpan : 0.0},
                {"max_valid_intersection_rms", maxRms},
                {"max_valid_intersection_extrapolation", maxExtrapolation},
                {"max_boundary_intersection_gap", maxBoundaryGap},
                {"failure_counts", failureCounts}, {"failed_stations", failedStations},
                {"failed_stations_truncated", attemptedFailures > 128}}},
            {"coordinate_normalization", "local_right_handed_orthonormal_frames"},
            {"reference_frame",
             QJsonObject{{"origin", QJsonArray{result.referenceOrigin.x(),
                                                result.referenceOrigin.y(),
                                                result.referenceOrigin.z()}},
                         {"local_x", QJsonArray{result.referenceLongitudinal.x(),
                                                 result.referenceLongitudinal.y(),
                                                 result.referenceLongitudinal.z()}},
                         {"local_y", QJsonArray{result.referenceTransverse.x(),
                                                 result.referenceTransverse.y(),
                                                 result.referenceTransverse.z()}},
                         {"local_z", QJsonArray{result.reference.normal.x(),
                                                 result.reference.normal.y(),
                                                 result.reference.normal.z()}}}},
            {"reference_plane", coefficients},
            {"reference_plane_count", int(result.referenceSegments.size())},
            {"reference_segments", referenceSegments},
            {"reference_points", int(result.referenceIndices.size())},
            {"slices", slices},
            {"selected_slice", result.selectedSlice},
            {"selected_points", int(result.selectedIndices.size())},
            {"projected_points", int(result.projected.size())},
            {"trajectory_points", int(result.trajectory.size())},
            {"fitted_line_points", int(result.fittedLine.size())},
            {"white_points", int(result.whitePoints.size())},
            {"white_corner_points", int(std::count_if(
                result.whitePoints.begin(), result.whitePoints.end(),
                [](const bwb::ProjectedPoint &point) { return point.corner; }))}};
}

fw::DebugLayer styled(const QString &name, const fw::CloudData &cloud,
                      std::array<double, 3> color, double size, double opacity = 1.0) {
    auto layer = fw::DebugLayer::result(name, cloud);
    layer.style.color = color;
    layer.style.pointSize = size;
    layer.style.opacity = opacity;
    return layer;
}

void addPolyline(QVector<fw::DebugLayer> &layers,
                 const std::vector<bwb::ProjectedPoint> &points,
                 const QString &name, std::array<double, 3> color, double width) {
    if (points.size() < 2)
        return;
    fw::DebugLayer line;
    line.name = name;
    line.geometry.kind = fw::GeometryKind::Polyline;
    line.style.color = color;
    line.style.lineWidth = width;
    for (const auto &point : points)
        line.geometry.vertices.push_back({point.xyz.x(), point.xyz.y(), point.xyz.z()});
    layers.push_back(line);
}

void addReferenceLayers(QVector<fw::DebugLayer> &layers,
                        const fw::CloudData &sourceCloud,
                        const std::vector<bwb::Point> &points,
                        const std::vector<bwb::ReferenceSegment> &segments,
                        const QString &idPrefix) {
    const std::array<std::array<double, 3>, 5> colors{{
        {1.0, 0.20, 0.15}, {1.0, 0.48, 0.10}, {0.95, 0.75, 0.10},
        {0.75, 0.25, 0.95}, {0.95, 0.25, 0.60}}};
    for (std::size_t i = 0; i < segments.size(); ++i) {
        const QString name = QStringLiteral("A%1 局部基准面").arg(i + 1);
        const auto cloud = indexedCloud(
            sourceCloud, points, segments[i].indices,
            idPrefix + QStringLiteral("_%1").arg(i + 1), name);
        layers.push_back(styled(name, cloud, colors[i % colors.size()], 3, 0.8));
    }
}

} // namespace

class CorrugatedPanelAlgorithm final : public fw::IAlgorithm {
  public:
    QString validate(const fw::AlgorithmInput &input,
                     const fw::ParameterSet &values) const override {
        if (input.clouds.size() != 1 || !input.clouds.front().cloud)
            return QStringLiteral("请选择一份波纹板 XYZ 点云。");
        if (!fw::hasFields(input.clouds.front(), {"x", "y", "z"}))
            return QStringLiteral("输入点云必须包含 XYZ 字段。");
        const auto parameters = fw::Parameters::merge(schema(), values);
        const auto errors = fw::Parameters::validate(schema(), parameters);
        if (!errors.isEmpty())
            return errors.join('\n');
        return QString::fromStdString(bwb::validateConfig(configuration(parameters)));
    }

    fw::AlgorithmResult execute(const fw::AlgorithmInput &input,
                                const fw::ParameterSet &values,
                                fw::IExecutionContext &context) override {
        const QString validation = validate(input, values);
        if (!validation.isEmpty())
            return fw::AlgorithmResult::failed(validation);
        const auto parameters = fw::Parameters::merge(schema(), values);
        const auto &sourceCloud = input.clouds.front();
        const auto typed = fw::toCloud<pcl::PointXYZ>(sourceCloud);
        if (typed->size() > std::numeric_limits<std::uint32_t>::max())
            return fw::AlgorithmResult::failed(QStringLiteral("输入点数超过索引容量。"));
        std::vector<bwb::Point> points;
        points.reserve(typed->size());
        for (std::size_t i = 0; i < typed->size(); ++i) {
            if ((i & 8191u) == 0 && context.isCancelled())
                return fw::AlgorithmResult::cancelled();
            const auto &point = typed->points[i];
            points.push_back({{point.x, point.y, point.z}, std::uint32_t(i)});
        }

        bwb::Hooks hooks;
        hooks.cancelled = [&] { return context.isCancelled(); };
        hooks.progress = [&](int value) { context.progress(value); };
        if (context.debug().enabled())
            hooks.stage = [&](int stageIndex, const bwb::Result &partial) {
                fw::DebugFrame frame;
                const QStringList titles = {QStringLiteral("提取局部基准面 A1…An"),
                                            QStringLiteral("双侧切片评估"),
                                            QStringLiteral("选择候选点云 B"),
                                            QStringLiteral("构建特征曲线 C（求交优先、整线投影回退）"),
                                            QStringLiteral("直线拟合与白点投影")};
                frame.stepId = QStringLiteral("corrugated_%1").arg(stageIndex);
                frame.title = titles[stageIndex];
                if (stageIndex == 3) frame.metrics = resultMetrics(partial, 0);
                auto background = fw::DebugLayer::background(QStringLiteral("原始点云"),
                                                              sourceCloud);
                background.style.opacity = 0.25;
                frame.layers.push_back(background);
                if (!partial.referenceSegments.empty()) {
                    addReferenceLayers(frame.layers, sourceCloud, points,
                                       partial.referenceSegments,
                                       QStringLiteral("_debug_a"));
                } else if (!partial.referenceIndices.empty()) {
                    const auto cloud = indexedCloud(sourceCloud, points, partial.referenceIndices,
                                                    "_debug_a", QStringLiteral("A 面"));
                    frame.layers.push_back(styled(QStringLiteral("A 面"), cloud,
                                                  {1.0, 0.25, 0.15}, 3));
                }
                if (!partial.selectedIndices.empty()) {
                    const auto cloud = indexedCloud(sourceCloud, points, partial.selectedIndices,
                                                    "_debug_b", QStringLiteral("B 点云"));
                    frame.layers.push_back(styled(QStringLiteral("B 点云"), cloud,
                                                  {1.0, 0.75, 0.05}, 4));
                }
                if (!partial.projected.empty()) {
                    const auto cloud = generatedCloud(sourceCloud, partial.projected, "_debug_c",
                                                      QStringLiteral("C 特征曲线候选点"));
                    frame.layers.push_back(styled(QStringLiteral("C 特征曲线候选点"), cloud,
                                                  {0.2, 0.7, 1.0}, 4));
                }
                addPolyline(frame.layers, partial.trajectory,
                            QStringLiteral("特征曲线（求交/投影）"),
                            {0.1, 0.95, 0.35}, 2);
                addPolyline(frame.layers, partial.fittedLine,
                            QStringLiteral("拟合直线"),
                            {1.0, 0.12, 0.08}, 4);
                if (!partial.whitePoints.empty()) {
                    const auto cloud = generatedCloud(sourceCloud, partial.whitePoints,
                                                      "_debug_white",
                                                      QStringLiteral("白色均匀点"));
                    frame.layers.push_back(styled(QStringLiteral("白色均匀点"), cloud,
                                                  {1.0, 1.0, 1.0}, 8));
                }
                return context.debug().checkpoint(frame) != fw::DebugAction::Stop;
            };

        QElapsedTimer timer;
        timer.start();
        const auto extracted = bwb::extract(points, configuration(parameters), hooks);
        const qint64 elapsed = timer.elapsed();
        if (extracted.cancelled)
            return fw::AlgorithmResult::cancelled();
        if (!extracted.ok) {
            auto failure = fw::AlgorithmResult::failed(
                QString::fromStdString(extracted.message));
            failure.metrics = resultMetrics(extracted, elapsed);
            return failure;
        }

        auto plane = indexedCloud(sourceCloud, points, extracted.referenceIndices, "_a",
                                  QStringLiteral("A 基准面"));
        auto selected = indexedCloud(sourceCloud, points, extracted.selectedIndices, "_b",
                                     QStringLiteral("B 最佳切片"));
        auto projected = generatedCloud(sourceCloud, extracted.projected, "_c",
                                        QStringLiteral("C 特征曲线候选点"));
        auto measuredCurve = generatedCloud(sourceCloud, extracted.trajectory,
                                             "_measured_curve",
                                             QStringLiteral("特征曲线（求交/投影）"));
        auto fittedLine = generatedCloud(sourceCloud, extracted.fittedLine, "_fitted_line",
                                         QStringLiteral("拟合直线控制点"));
        auto whitePoints = generatedCloud(sourceCloud, extracted.whitePoints, "_white_points",
                                          QStringLiteral("白色均匀点"));
        fw::AlgorithmResult output;
        output.outputs = {plane, selected, projected, measuredCurve, fittedLine, whitePoints};
        output.message = extracted.usedProjectionFallback
            ? QStringLiteral("表面求交未覆盖全部站位，已将整条 B 特征曲线投影到对应 A 面；输出明确标记为投影回退，不代表 S 与 A 的交线。")
            : QStringLiteral("已使用远离边缘的板材主体拟合并延伸到基准面；边缘区域未参与特征曲线拟合。白点由该曲线规则化生成。");
        output.metrics = resultMetrics(extracted, elapsed);
        auto background = fw::DebugLayer::background(QStringLiteral("原始点云"), sourceCloud);
        background.style.opacity = 0.2;
        auto aLayer = styled(plane.name, plane, {1.0, 0.2, 0.15}, 2, 0.75);
        if (extracted.referenceSegments.size() > 1)
            aLayer.style.visible = false;
        auto bLayer = styled(selected.name, selected, {1.0, 0.75, 0.05}, 3);
        auto cLayer = styled(projected.name, projected, {0.2, 0.7, 1.0}, 3, 0.55);
        cLayer.style.visible = false;
        output.layers = {background, aLayer, bLayer, cLayer};
        if (extracted.referenceSegments.size() > 1)
            addReferenceLayers(output.layers, sourceCloud, points,
                               extracted.referenceSegments, QStringLiteral("_a_segment"));
        addPolyline(output.layers, extracted.trajectory,
                    QStringLiteral("特征曲线（求交/投影）"),
                    {0.1, 0.95, 0.35}, 2);
        addPolyline(output.layers, extracted.fittedLine,
                    QStringLiteral("拟合直线"),
                    {1.0, 0.12, 0.08}, 4);
        auto measuredControlLayer = styled(measuredCurve.name, measuredCurve,
                                           {0.1, 0.95, 0.35}, 2);
        measuredControlLayer.style.visible = false;
        output.layers.push_back(measuredControlLayer);
        auto fitControlLayer = styled(fittedLine.name, fittedLine, {1.0, 0.12, 0.08}, 3);
        fitControlLayer.style.visible = false;
        output.layers.push_back(fitControlLayer);
        output.layers.push_back(styled(whitePoints.name, whitePoints, {1.0, 1.0, 1.0}, 8));
        context.log(QString::fromUtf8(
            QJsonDocument(output.metrics).toJson(QJsonDocument::Compact)));
        return output;
    }
};

class CorrugatedPanelPlugin final : public QObject, public fw::IAlgorithmPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_ALGORITHM_IID FILE "metadata.json")
    Q_INTERFACES(fw::IAlgorithmPlugin)

  public:
    fw::AlgorithmDescriptor descriptor() const override {
        return {"custom.corrugatedPanelFeature",
                QStringLiteral("波纹板特征提取：竖直放置"),
                "1.7.3",
                {"x", "y", "z"}};
    }
    QJsonObject parameterSchema() const override { return schema(); }
    bool migrateParameters(int fromVersion, const fw::ParameterSet &old,
                           fw::ParameterSet &updated, QString &error) const override {
        if (fromVersion < 1 || fromVersion > 4) {
            error = QStringLiteral("只支持从参数版本 1、2、3 或 4 迁移。");
            return false;
        }
        updated = fw::Parameters::merge(schema(), old);
        error = fw::Parameters::validate(schema(), updated).join('\n');
        return error.isEmpty();
    }
    fw::IAlgorithm *create() override { return new CorrugatedPanelAlgorithm; }
    void destroy(fw::IAlgorithm *algorithm) override { delete algorithm; }
};

#include "MyAlgorithm.moc"
