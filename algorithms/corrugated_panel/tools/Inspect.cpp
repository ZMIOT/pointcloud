#include "CorrugatedPanel.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
    output << "source_index x y z is_corner\n" << std::setprecision(12);
    for (const auto &point : points)
        output << point.source << ' ' << point.xyz.x() << ' ' << point.xyz.y() << ' '
               << point.xyz.z() << ' ' << (point.corner ? 1 : 0) << '\n';
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
                     "[坡段合并容差] [局部基准面最大长度]\n";
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
            std::cerr << "坡段合并容差必须是有效数字\n";
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
              << "a_plane_count=" << result.referenceSegments.size() << '\n'
              << "selected_slice=" << result.selectedSlice << '\n'
              << "b_points=" << result.selectedIndices.size() << '\n'
              << "c_points=" << result.projected.size() << '\n'
              << "trajectory_points=" << result.trajectory.size() << '\n'
              << "fitted_line_points=" << result.fittedLine.size() << '\n'
              << "white_points=" << result.whitePoints.size() << '\n';
    std::size_t cornerCount = 0;
    for (const auto &point : result.whitePoints)
        cornerCount += point.corner ? 1u : 0u;
    std::cout << "white_corner_points=" << cornerCount << '\n';
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
        writePoints(directory / "trajectory.xyz", result.trajectory);
        writePoints(directory / "measured_curve.xyz", result.trajectory);
        writePoints(directory / "fitted_line.xyz", result.fittedLine);
        writePoints(directory / "white_points.xyz", result.whitePoints);
    }
    return result.ok ? 0 : 1;
}
