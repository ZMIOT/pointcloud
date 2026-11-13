#include <QCoreApplication>
#include <QPluginLoader>
#include <fw/Algorithm.h>
#include <fw/Parameters.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {

void require(bool condition, const std::string &message) {
    if (!condition)
        throw std::runtime_error(message);
}

pcl::PointCloud<pcl::PointXYZ> synthetic() {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::mt19937 random(20261112u);
    std::normal_distribution<float> noise(0.0f, 0.04f);
    auto append = [&](float x, float y, float z) {
        cloud.push_back({x + noise(random), y + noise(random), z + noise(random)});
    };
    for (int x = -100; x <= 100; x += 2)
        for (int y = -70; y <= 70; y += 2)
            append(float(x), float(y), 0.0f);
    for (int x = -100; x <= 100; ++x) {
        const float wave = 22.0f * std::sin(float(x) / 18.0f);
        for (int distance = 1; distance <= 35; ++distance) {
            append(float(x), wave, float(distance));
            append(float(x), -48.0f, -float(distance));
        }
    }
    return cloud;
}

struct Context final : fw::IExecutionContext, fw::IDebugContext {
    bool debugging = false;
    bool cancelled = false;
    int checkpoints = 0;
    int stopAt = 0;
    fw::IDebugContext &debug() override { return *this; }
    bool enabled() const override { return debugging; }
    bool isCancelled() const override { return cancelled; }
    void log(const QString &) override {}
    void progress(int) override {}
    fw::DebugAction checkpoint(const fw::DebugFrame &) override {
        ++checkpoints;
        return stopAt > 0 && checkpoints >= stopAt ? fw::DebugAction::Stop
                                                    : fw::DebugAction::Continue;
    }
};

} // namespace

int main(int argc, char **argv) {
    QCoreApplication application(argc, argv);
    try {
        QPluginLoader loader(QString::fromUtf8(PLUGIN_PATH));
        auto *plugin = qobject_cast<fw::IAlgorithmPlugin *>(loader.instance());
        require(plugin != nullptr, loader.errorString().toStdString());
        require(plugin->descriptor().id == "custom.corrugatedPanelFeature", "wrong plugin ID");
        const auto schema = plugin->parameterSchema();
        require(fw::Parameters::validateSchema(schema).isEmpty(), "invalid parameter schema");
        const auto parameters = fw::Parameters::defaults(schema);
        require(schema.value("parameterVersion").toInt() == 5 &&
                    plugin->descriptor().version == "1.7.3", "plugin version mismatch");
        fw::ParameterSet old{{"trajectory_spacing", 3.0}}, migrated;
        QString migrationError;
        require(plugin->migrateParameters(3, old, migrated, migrationError) &&
                    migrated.value("trajectory_spacing").toDouble() == 3.0 &&
                    migrated.value("max_surface_extrapolation").toDouble() == 40.0 &&
                    migrated.value("allow_projection_fallback").toBool(),
                "parameter migration must preserve old settings and add defaults");
        old["max_surface_extrapolation"] = 5.0;
        require(plugin->migrateParameters(4, old, migrated, migrationError) &&
                    migrated.value("max_surface_extrapolation").toDouble() == 5.0,
                "explicit user extension limit must survive migration");
        require(!plugin->migrateParameters(99, old, migrated, migrationError),
                "unknown parameter version must not migrate");
        require(fw::Parameters::validate(schema, parameters).isEmpty(),
                "default parameters are invalid");

        auto data = fw::DebugLayer::result("synthetic", synthetic()).data;
        data.id = "synthetic";
        data.coordinateFrame = "test-world";
        data.unit = "mm";
        fw::AlgorithmInput input{{data}, {}};
        fw::IAlgorithm *algorithm = plugin->create();
        struct Guard {
            fw::IAlgorithmPlugin *plugin;
            fw::IAlgorithm *algorithm;
            ~Guard() { plugin->destroy(algorithm); }
        } guard{plugin, algorithm};
        require(algorithm->validate(input, parameters).isEmpty(), "default validation failed");

        Context normal;
        const auto result = algorithm->execute(input, parameters, normal);
        require(result.status == fw::ResultStatus::Success, result.message.toStdString());
        require(result.metrics.value("reference_plane_count").toInt() >= 1,
                "local reference-plane metrics are missing");
        require(result.metrics.value("coordinate_normalization").toString() ==
                    "local_right_handed_orthonormal_frames",
                "coordinate-normalization mode is missing");
        const auto referenceSegments = result.metrics.value("reference_segments").toArray();
        require(!referenceSegments.isEmpty() &&
                    referenceSegments.at(0).toObject().contains("boundary_gap_measured") &&
                    !referenceSegments.at(0).toObject().value("boundary_gap_measured").toBool() &&
                    referenceSegments.at(0).toObject().value("boundary_gap_from_previous").isNull(),
                "the first segment must report boundary gap as unmeasured/null");
        require(!referenceSegments.isEmpty() &&
                    referenceSegments.at(0).toObject().value("local_x").toArray().size() == 3 &&
                    referenceSegments.at(0).toObject().value("local_y").toArray().size() == 3 &&
                    referenceSegments.at(0).toObject().value("local_z").toArray().size() == 3,
                "local reference frames are missing from metrics");
        require(result.outputs.size() == 6,
                "A/B/C/measured-curve/fitted-line/white-point outputs are required");
        require(result.outputs[0].name == QStringLiteral("A 基准面"), "A output missing");
        require(result.outputs[1].name == QStringLiteral("B 最佳切片"), "B output missing");
        require(result.outputs[2].name == QStringLiteral("C 特征曲线候选点"), "C output missing");
        require(result.outputs[3].name == QStringLiteral("特征曲线（求交/投影）"),
                "measured curve output missing");
        require(result.outputs[4].name == QStringLiteral("拟合直线控制点"),
                "fitted line output missing");
        require(result.outputs[5].name == QStringLiteral("白色均匀点"),
                "white point output missing");
        require(fw::hasFields(result.outputs[2], {"x", "y", "z", "source_index"}),
                "C metadata missing");
        for (int i = 2; i < 6; ++i)
            require(fw::hasFields(result.outputs[i], {"reference_index", "is_extrapolated",
                        "extrapolation_distance", "surface_rms", "support_count", "surface_degree",
                        "is_projection_fallback"}),
                    "surface provenance missing");
        const auto intersection = result.metrics.value("surface_intersection").toObject();
        require(intersection.value("support_region").toString() ==
                    "offset_slice_body_band_excluding_edge" &&
                    intersection.value("body_band_min").toDouble() == 8.5 &&
                    intersection.value("body_band_max").toDouble() == 31.5,
                "plugin must expose the edge-excluding body support policy");
        require(intersection.value("stations").toInt() > 0 &&
                    intersection.value("stations") == intersection.value("valid_stations"),
                "intersection station metrics missing");
        require(fw::hasFields(result.outputs[3], {"x", "y", "z", "source_index"}),
                "measured curve metadata missing");
        require(fw::hasFields(result.outputs[4], {"x", "y", "z", "source_index"}),
                "fitted line metadata missing");
        require(fw::hasFields(result.outputs[5],
                              {"x", "y", "z", "source_index", "is_corner"}),
                "white point metadata missing");
        require(result.outputs[3].cloud->data != result.outputs[4].cloud->data,
                "measured curve and fitted line must remain separate outputs");
        const auto measuredLayer = std::find_if(
            result.layers.cbegin(), result.layers.cend(), [](const fw::DebugLayer &layer) {
                return layer.name == QStringLiteral("特征曲线（求交/投影）") &&
                       layer.geometry.kind == fw::GeometryKind::Polyline;
            });
        const auto fittedLayer = std::find_if(
            result.layers.cbegin(), result.layers.cend(), [](const fw::DebugLayer &layer) {
                return layer.name == QStringLiteral("拟合直线") &&
                       layer.geometry.kind == fw::GeometryKind::Polyline;
            });
        require(measuredLayer != result.layers.cend() && fittedLayer != result.layers.cend(),
                "both measured and fitted line layers must remain visible");
        require(measuredLayer->geometry.vertices.size() > fittedLayer->geometry.vertices.size(),
                "measured curve must retain more detail than the fitted line");
        require(measuredLayer->style.color != fittedLayer->style.color,
                "measured and fitted lines must use distinguishable colors");
        const auto whiteLayer = std::find_if(
            result.layers.cbegin(), result.layers.cend(), [](const fw::DebugLayer &layer) {
                return layer.name == QStringLiteral("白色均匀点");
            });
        require(whiteLayer != result.layers.cend() &&
                    whiteLayer->style.color == std::array<double, 3>{1.0, 1.0, 1.0},
                "white points are not displayed in white");
        for (const auto &output : result.outputs)
            require(output.coordinateFrame == data.coordinateFrame && output.unit == data.unit,
                    "coordinate frame or unit was lost");
        require(normal.checkpoints == 0, "debug-disabled execution emitted checkpoints");
        auto noExtrapolation = parameters;
        noExtrapolation["max_surface_extrapolation"] = 0.0;
        noExtrapolation["allow_projection_fallback"] = false;
        Context failureContext;
        const auto unsupported = algorithm->execute(input, noExtrapolation, failureContext);
        require(unsupported.status == fw::ResultStatus::Failed && unsupported.outputs.empty() &&
                    unsupported.metrics.value("surface_intersection").toObject()
                        .value("failure_counts").toObject().contains("surface_extrapolation_limit"),
                "unsupported recovery must expose failure evidence without formal trajectory outputs");
        auto fallbackParameters = parameters;
        fallbackParameters["max_surface_extrapolation"] = 0.0;
        Context fallbackContext;
        const auto fallbackResult = algorithm->execute(input, fallbackParameters, fallbackContext);
        require(fallbackResult.status == fw::ResultStatus::Success &&
                    fallbackResult.metrics.value("surface_intersection").toObject()
                        .value("used_projection_fallback").toBool(),
                "enabled fallback must switch the entire curve to projection mode");

        // 同一真实样本还需经过插件的 float 点云转换，避免仅验证 double 核心。
        for (const char *sampleVariable : {"BWB_SAMPLE_6_PATH", "BWB_SAMPLE_52_PATH",
                                           "BWB_SAMPLE_1_PATH", "BWB_SAMPLE_3_PATH",
                                           "BWB_SAMPLE_8_PATH"}) {
            const char *samplePath = std::getenv(sampleVariable);
            if (!samplePath || !*samplePath)
                continue;
            const bool sample52 = std::string(sampleVariable) == "BWB_SAMPLE_52_PATH";
            const bool sample1 = std::string(sampleVariable) == "BWB_SAMPLE_1_PATH";
            const bool sample3 = std::string(sampleVariable) == "BWB_SAMPLE_3_PATH";
            const bool sample8 = std::string(sampleVariable) == "BWB_SAMPLE_8_PATH";
            std::ifstream file(std::filesystem::u8path(samplePath));
            require(bool(file), "cannot open sample6 for plugin regression");
            std::string header;
            std::getline(file, header);
            pcl::PointCloud<pcl::PointXYZ> realCloud;
            std::uint64_t index = 0;
            double x = 0, y = 0, z = 0;
            while (file >> index >> x >> y >> z)
                realCloud.push_back({float(x), float(y), float(z)});
            require(file.eof() && realCloud.size() ==
                        (sample1 ? 1448522u : (sample52 ? 831069u :
                            (sample3 ? 1100339u : (sample8 ? 593461u : 1309174u)))),
                    "wrong sample6 fixture for plugin regression");
            const char *sampleName = sample1 ? "1.txt" : (sample52 ? "52.txt" :
                (sample3 ? "3.txt" : (sample8 ? "8.txt" : "6.txt")));
            auto realData = fw::DebugLayer::result(sampleName, realCloud).data;
            realData.id = sample1 ? "sample1-regression" :
                (sample52 ? "sample52-regression" : (sample3 ? "sample3-regression" :
                    (sample8 ? "sample8-regression" : "sample6-regression")));
            realData.coordinateFrame = "world";
            realData.unit = "mm";
            Context realContext;
            const auto realResult = algorithm->execute({{realData}, {}}, parameters, realContext);
            require(realResult.status == fw::ResultStatus::Success,
                    realResult.message.toStdString());
            const auto segments = realResult.metrics.value("reference_segments").toArray();
            if (sample8) {
                const auto surface = realResult.metrics.value("surface_intersection").toObject();
                require(segments.size() == 2 && surface.value("used_projection_fallback").toBool() &&
                            surface.value("valid_stations").toInt() == 384 &&
                            surface.value("length_slope_suppressed_stations").toInt() == 8,
                        "plugin sample8 lost narrow-window spike suppression");
                std::cout << "8.txt plugin regression: narrow windows suppressed=8\n";
                continue;
            }
            if (sample1) {
                const auto surface = realResult.metrics.value("surface_intersection").toObject();
                require(segments.size() == 1 && !surface.value("used_projection_fallback").toBool() &&
                            surface.value("intersection_valid_stations").toInt() == 194,
                        "plugin sample1 did not recover from body evidence");
                std::cout << "1.txt plugin regression: body-extension stations="
                          << surface.value("intersection_valid_stations").toInt() << '\n';
                continue;
            }
            require(segments.size() == 2, "plugin sample must have two A planes");
            if (sample52 || sample3) {
                const auto surface = realResult.metrics.value("surface_intersection").toObject();
                require(!surface.value("used_projection_fallback").toBool() &&
                            surface.value("intersection_valid_stations").toInt() == (sample52 ? 385 : 383) &&
                            surface.value("local_patch_validated_stations").toInt() == (sample52 ? 63 : 1),
                        "float path lost stable local-face extension or reverted to projection");
            }
            for (const auto &entry : segments) {
                const auto segment = entry.toObject();
                require(segment.value("point_count").toInt() > 78000 &&
                            segment.value("plane_rms").toDouble(1.0) < 0.25,
                        "plugin conversion caused sample6 A support/residual regression");
            }
            const auto second = segments.at(1).toObject();
            require(second.value("normal_change_from_previous_degrees").toDouble(90.0) < 1.0 &&
                        second.value("boundary_gap_measured").toBool() &&
                        second.value("boundary_gap_from_previous").toDouble(10.0) < 0.6,
                    "plugin conversion caused sample6 A continuity regression");
            std::cout << sampleName << " plugin regression: A1="
                      << segments.at(0).toObject().value("point_count").toInt()
                      << " A2=" << second.value("point_count").toInt()
                      << " gap=" << second.value("boundary_gap_from_previous").toDouble()
                      << '\n';
        }

        Context debug;
        debug.debugging = true;
        const auto stepped = algorithm->execute(input, parameters, debug);
        require(stepped.status == fw::ResultStatus::Success && debug.checkpoints == 5,
                "five debug stages are required");
        for (int i = 0; i < result.outputs.size(); ++i)
            require(result.outputs[i].cloud->data == stepped.outputs[i].cloud->data,
                    "debugging changed output data");

        Context stopped;
        stopped.debugging = true;
        stopped.stopAt = 2;
        require(algorithm->execute(input, parameters, stopped).status ==
                    fw::ResultStatus::Cancelled,
                "debug stop was ignored");
        Context cancelled;
        cancelled.cancelled = true;
        require(algorithm->execute(input, parameters, cancelled).status ==
                    fw::ResultStatus::Cancelled,
                "early cancellation was ignored");
        auto bad = parameters;
        bad["slice_thickness"] = 0.0;
        require(!algorithm->validate(input, bad).isEmpty(), "invalid thickness was accepted");
        bad = parameters;
        bad["max_adjacent_normal_angle"] = 90.0;
        require(!algorithm->validate(input, bad).isEmpty(),
                "invalid adjacent-normal angle was accepted");
        require(!algorithm->validate({}, parameters).isEmpty(), "empty input was accepted");
        std::cout << "CorrugatedPanel plugin tests passed\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
