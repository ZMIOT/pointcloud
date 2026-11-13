#include "CorrugatedPanel.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {

bool readTextCloud(const std::filesystem::path &path, std::vector<bwb::Point> &points,
                   std::string &error) {
    std::ifstream input(path);
    if (!input) {
        error = "无法打开输入文件";
        return false;
    }
    std::string line;
    std::getline(input, line); // index x y z
    std::uint64_t ignoredIndex = 0;
    double x = 0, y = 0, z = 0;
    while (input >> ignoredIndex >> x >> y >> z) {
        if (points.size() >= std::numeric_limits<std::uint32_t>::max()) {
            error = "点数超过 uint32 索引容量";
            return false;
        }
        points.push_back({{x, y, z}, std::uint32_t(points.size())});
    }
    if (points.empty()) {
        error = "文件没有可解析的 index x y z 数据";
        return false;
    }
    return true;
}

void writePoints(const std::filesystem::path &path,
                 const std::vector<bwb::ProjectedPoint> &points) {
    std::ofstream output(path);
    output << "source_index x y z is_corner reference_index is_extrapolated extrapolation_distance surface_rms support_count surface_degree is_projection_fallback is_line_fitted line_intersection_gap\n" << std::setprecision(12);
    for (const auto &point : points)
        output << point.source << ' ' << point.xyz.x() << ' ' << point.xyz.y() << ' '
               << point.xyz.z() << ' ' << (point.corner ? 1 : 0) << ' '
               << point.referenceIndex << ' ' << point.extrapolated << ' '
               << point.extrapolationDistance << ' ' << point.surfaceRms << ' '
               << point.supportCount << ' ' << point.surfaceDegree << ' '
               << point.projectionFallback << ' ' << point.lineFitted << ' '
               << point.lineIntersectionGap << '\n';
}

void writeIndexed(const std::filesystem::path &path, const std::vector<bwb::Point> &points,
                  const std::vector<int> &indices) {
    std::ofstream output(path);
    output << "source_index x y z\n" << std::setprecision(12);
    for (int index : indices) {
        const auto &point = points[std::size_t(index)];
        output << point.source << ' ' << point.xyz.x() << ' ' << point.xyz.y() << ' '
               << point.xyz.z() << '\n';
    }
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || argc > 5) {
        std::cerr << "用法: CorrugatedPanelInspect <index-x-y-z.txt> [输出目录] "
                     "[坡段拟合容差] [局部基准面最大长度]\n";
        return 2;
    }
    std::vector<bwb::Point> points;
    std::string error;
    if (!readTextCloud(std::filesystem::u8path(argv[1]), points, error)) {
        std::cerr << error << '\n';
        return 2;
    }
    bwb::Hooks hooks;
    hooks.referenceCandidate = [](const bwb::Plane &plane, std::size_t support,
                                   std::size_t positive, std::size_t negative, double score,
                                   double rms, double peakSupport) {
        std::cerr << "seed_candidate normal=" << plane.normal.transpose() << " d=" << plane.d
                  << " support=" << support << " positive=" << positive
                  << " negative=" << negative << " score=" << score
                  << " rms=" << rms << " peak_support=" << peakSupport << '\n';
    };
    hooks.progress = [](int value) {
        if (value == 5 || value == 35 || value == 65 || value == 78 || value == 88 ||
            value == 100)
            std::cerr << "progress=" << value << "%\n";
    };
    bwb::Config config;
    if (argc >= 4) {
        try {
            config.lineFitTolerance = std::stod(argv[3]);
        } catch (...) {
            std::cerr << "坡段拟合容差必须是有效数字\n";
            return 2;
        }
    }
    if (argc >= 5) {
        try {
            config.referenceSegmentLength = std::stod(argv[4]);
        } catch (...) {
            std::cerr << "局部基准面最大长度必须是有效数字\n";
            return 2;
        }
    }
    const auto result = bwb::extract(points, config, hooks);
    std::cout << "input_points=" << points.size() << '\n'
              << "ok=" << (result.ok ? "true" : "false") << '\n'
              << "message=" << result.message << '\n'
              << "plane=" << result.reference.normal.x() << ' '
              << result.reference.normal.y() << ' ' << result.reference.normal.z() << ' '
              << result.reference.d << '\n'
              << "a_points=" << result.referenceIndices.size() << '\n'
              << "reference_origin=" << result.referenceOrigin.transpose() << '\n'
              << "reference_longitudinal=" << result.referenceLongitudinal.transpose() << '\n'
              << "a_plane_count=" << result.referenceSegments.size() << '\n'
              << "selected_slice=" << result.selectedSlice << '\n'
              << "b_points=" << result.selectedIndices.size() << '\n'
              << "c_points=" << result.projected.size() << '\n'
              << "curve_construction="
              << (result.usedProjectionFallback ? "projection_fallback" : "off_edge_body_extension") << '\n'
              << "body_support_band=" << result.bodyBandMin << ':' << result.bodyBandMax << '\n'
              << "trajectory_points=" << result.trajectory.size() << '\n'
              << "fitted_line_points=" << result.fittedLine.size() << '\n'
              << "white_points=" << result.whitePoints.size() << '\n';
    std::size_t cornerCount = 0;
    for (const auto &point : result.whitePoints)
        cornerCount += point.corner ? 1u : 0u;
    std::cout << "white_corner_points=" << cornerCount << '\n';
    double maxJunctionGap = 0;
    for (const auto &p : result.fittedLine) maxJunctionGap = std::max(maxJunctionGap,p.lineIntersectionGap);
    std::cout << "max_line_junction_gap=" << maxJunctionGap << '\n';
    for (std::size_t i = 0; i < result.lineModels.size(); ++i) {
        const auto &line = result.lineModels[i];
        std::cout << "line[" << i << "] support=" << line.support << '/' << line.last-line.first+1
                  << " rms=" << line.rms << " origin=" << line.origin.transpose()
                  << " direction=" << line.direction.transpose() << '\n';
    }
    std::map<std::string, int> failures;
    double maxExtrapolation = 0, maxRms = 0;
    int curved = 0, intersectionFailures = 0;
    for (const auto &station : result.intersectionStations) {
        const auto reason = !station.intersectionFailure.empty()
            ? station.intersectionFailure : (!station.valid ? station.failure : std::string());
        if (!reason.empty()) { ++failures[reason]; ++intersectionFailures; }
        else {
            maxExtrapolation = std::max(maxExtrapolation, station.attemptedExtrapolation);
            maxRms = std::max(maxRms, station.point.surfaceRms);
            curved += station.point.surfaceDegree == 2 ? 1 : 0;
        }
    }
    std::cout << "intersection_stations=" << result.intersectionStations.size()
              << " quadratic=" << curved << " max_valid_extrapolation=" << maxExtrapolation
              << " max_valid_rms=" << maxRms
              << " primary_intersection_failures=" << intersectionFailures << '\n';
    for (const auto &entry : failures)
        std::cout << "intersection_failure=" << entry.first << " count=" << entry.second << '\n';
    for (std::size_t i = 0; i < result.referenceSegments.size(); ++i) {
        const auto &segment = result.referenceSegments[i];
        std::cout << "A" << i + 1 << " range=" << segment.longitudinalMin << ':'
                  << segment.longitudinalMax << " points=" << segment.indices.size()
                  << " plane=" << segment.plane.normal.transpose() << ' '
                  << segment.plane.d << " local_x="
                  << segment.localLongitudinal.transpose() << " local_y="
                  << segment.localTransverse.transpose() << " rms=" << segment.planeRms
                  << " normal_change_deg="
                  << segment.normalChangeFromPreviousDegrees << '\n';
        if (segment.boundaryGapMeasured)
            std::cout << "boundary_gap_A" << i << "_A" << i + 1 << '='
                      << segment.boundaryGapFromPrevious << '\n';
    }
    for (std::size_t i = 0; i < result.slices.size(); ++i) {
        const auto &slice = result.slices[i];
        std::cout << "slice[" << i << "] side=" << slice.side
                  << " offset=" << slice.offset << " raw=" << slice.raw.size()
                  << " filtered=" << slice.filtered.size() << " length=" << slice.length
                  << " completeness=" << slice.completeness
                  << " deviation=" << slice.deviation << " bent=" << slice.bent << '\n';
    }
    if (argc >= 3 && std::string(argv[2]) != "-") {
        const auto directory = std::filesystem::u8path(argv[2]);
        std::filesystem::create_directories(directory);
        writeIndexed(directory / "a_plane.xyz", points, result.referenceIndices);
        writeIndexed(directory / "b_slice.xyz", points, result.selectedIndices);
        writePoints(directory / "c_projected.xyz", result.projected);
        writePoints(directory / "c_intersection.xyz", result.projected);
        writePoints(directory / "c_feature_curve.xyz", result.projected);
        std::ofstream stations(directory / "intersection_stations.tsv");
        stations << "longitudinal\tvalid\treference_index\tsupport\tdegree\trms\textrapolation\tboundary_intersection_gap\tis_projection_fallback\tintersection_failure\tfinal_failure\tsupport_min_height\tsupport_max_height\tattempted_extrapolation\tlocal_patch_extension\tlocal_patch_prediction_spread\tbroad_model_failure\tseed_longitudinal_span\tlength_slope_identifiable\tx\ty\tz\tdx_dh\tdy_dh\tdz_dh\n";
        for (const auto &s : result.intersectionStations)
            stations << s.longitudinal << '\t' << s.valid << '\t' << s.point.referenceIndex
                     << '\t' << s.point.supportCount << '\t' << s.point.surfaceDegree << '\t'
                     << s.point.surfaceRms << '\t' << s.point.extrapolationDistance << '\t'
                     << s.boundaryIntersectionGap << '\t' << s.point.projectionFallback << '\t'
                     << s.intersectionFailure << '\t' << s.failure << '\t'
                     << s.supportMinHeight << '\t' << s.supportMaxHeight << '\t'
                     << s.attemptedExtrapolation << '\t' << s.localPatchExtension << '\t'
                     << s.localPatchPredictionSpread << '\t' << s.broadModelFailure << '\t'
                     << s.seedLongitudinalSpan << '\t' << s.lengthSlopeIdentifiable << '\t'
                     << std::setprecision(12) << s.point.xyz.x() << '\t' << s.point.xyz.y() << '\t'
                     << s.point.xyz.z() << '\t' << s.surfaceHeightDirection.x() << '\t'
                     << s.surfaceHeightDirection.y() << '\t' << s.surfaceHeightDirection.z() << '\n';
        writePoints(directory / "trajectory.xyz", result.trajectory);
        writePoints(directory / "measured_curve.xyz", result.trajectory);
        writePoints(directory / "fitted_line.xyz", result.fittedLine);
        writePoints(directory / "white_points.xyz", result.whitePoints);
    }
    return result.ok ? 0 : 1;
}
