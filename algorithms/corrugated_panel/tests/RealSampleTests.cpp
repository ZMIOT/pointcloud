#include "CorrugatedPanel.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}
}

// 可选真实数据回归，不将用户的 54 MB 点云复制进源码。
// 这些阈值防止已观测到的局部错面回退，不是带人工标注的精度验收。
int main(int argc, char **argv) {
    const bool sample52 = argc > 1 && std::string(argv[1]) == "52";
    const bool sample121 = argc > 1 && std::string(argv[1]) == "121";
    const char *variable = sample121 ? "BWB_SAMPLE_121_PATH" :
        (sample52 ? "BWB_SAMPLE_52_PATH" : "BWB_SAMPLE_6_PATH");
    const char *path = std::getenv(variable);
    if (!path || !*path) {
        std::cout << "SKIP: set " << variable << " to the original sample\n";
        return 77;
    }
    try {
        std::ifstream input(std::filesystem::u8path(path));
        require(bool(input), "cannot open the specified real-sample fixture");
        std::string header;
        std::getline(input, header);
        std::vector<bwb::Point> points;
        std::uint64_t index = 0;
        double x = 0, y = 0, z = 0;
        while (input >> index >> x >> y >> z)
            points.push_back({bwb::Vec3(x, y, z), std::uint32_t(points.size())});
        require(input.eof() && points.size() == (sample121 ? 4238245u : (sample52 ? 831069u : 1309174u)),
                "unexpected real-sample point count");
        const bwb::Config config;
        const auto result = bwb::extract(points, config);
        if (sample121) {
            // This is a safety regression, not an assertion that 121 is recovered.
            // Allow future valid recovery, but never accept unsafe gap bridging.
            require(result.referenceSegments.size() == 6, "121 local A count regressed");
            for (const auto &a : result.referenceSegments)
                require(a.indices.size() > 150000 && a.planeRms < 0.25,
                        "121 local A support/residual regressed");
            require(result.intersectionStations.size() > 1000 && !result.projected.empty(),
                    "121 surface evidence missing");
            if (!result.ok) {
                require(result.trajectory.empty() && result.fittedLine.empty() && result.whitePoints.empty(),
                        "failed 121 recovery generated a complete trajectory");
                require(std::any_of(result.intersectionStations.begin(), result.intersectionStations.end(),
                            [](const bwb::IntersectionStation &s) { return !s.valid && !s.failure.empty(); }),
                        "121 failure has no located evidence");
                std::cout << "121.txt NOT fully recovered; safety rejection verified: " << result.message << '\n';
            } else {
                require(std::all_of(result.intersectionStations.begin(), result.intersectionStations.end(),
                            [&](const bwb::IntersectionStation &s) {
                                return s.valid && s.point.surfaceRms <= config.surfaceFitTolerance &&
                                    s.point.extrapolationDistance <= config.maxSurfaceExtrapolation;
                            }), "121 recovery bypassed quality gates");
            }
            return 0;
        }
        require(result.ok, result.message.c_str());
        require(result.referenceSegments.size() == 2, "sample must produce A1 and A2");
        for (const auto &segment : result.referenceSegments) {
            require(segment.locallyFitted, "A plane must be fitted from local points");
            require(segment.indices.size() > points.size() * (sample52 ? 0.08 : 0.06),
                    "a local A plane lost most of its measured support");
            require(segment.planeRms < 0.25, "local A plane residual regressed");
            // 单靠点数不能证明覆盖整段。52 的端部 A 面包含缺段，覆盖门槛为
            // 16/20；6 的门槛仍是 18/20，分别防止两个已知样本退化。
            bool covered[20] = {};
            for (int pointIndex : segment.indices) {
                const double coordinate = result.referenceLongitudinal.dot(
                    points[std::size_t(pointIndex)].xyz - result.referenceOrigin);
                const double ratio = (coordinate - segment.longitudinalMin) /
                    (segment.longitudinalMax - segment.longitudinalMin);
                covered[std::clamp(int(ratio * 20), 0, 19)] = true;
            }
            const auto coverage = std::count(std::begin(covered), std::end(covered), true);
            std::cout << "local_support_coverage=" << coverage << "/20\n";
            require(coverage >= (sample52 ? 16 : 18),
                    "A support is concentrated in only part of a segment");
        }
        const auto &second = result.referenceSegments[1];
        require(second.normalChangeFromPreviousDegrees < 1.0,
                "sample adjacent normal change regressed");
        require(second.boundaryGapMeasured && second.boundaryGapFromPrevious < 0.6,
                "sample boundary gap regressed or was not measured");
        require(!result.projected.empty() && !result.fittedLine.empty(),
                "sample projection/trajectory output is missing");
        if (sample52) {
            // 固定世界坐标的该样本：错误种子朝向 X，实际 A 面候选朝向 Z。
            // 同时要求后续得到完整弯折侧和另一侧直线证据，不能只凭点数通过。
            require(std::abs(result.reference.normal.z()) > 0.98,
                    "52.txt selected the broad outer surface instead of the A face");
            const auto &selected = result.slices[std::size_t(result.selectedSlice)];
            require(selected.completeness > 0.9 && selected.length > 700.0 && selected.bent,
                    "52.txt corrugated side is incomplete");
            bool oppositeStraight = false;
            for (const auto &slice : result.slices)
                if (slice.side != selected.side && !slice.bent && slice.length > 650.0)
                    oppositeStraight = true;
            require(oppositeStraight, "52.txt has no supported opposite straight side");
        }
        for (const auto &point : result.projected) {
            require(point.referenceIndex < result.referenceSegments.size(), "invalid intersection owner");
            const auto &plane = result.referenceSegments[point.referenceIndex].plane;
            require(point.supportCount >= 20 && point.extrapolationDistance <= 5.0 &&
                        point.surfaceRms <= 0.6, "unsupported surface intersection");
            require(std::abs(plane.normal.dot(point.xyz) + plane.d) < 1e-7,
                    "projected point does not lie on its owning A plane");
        }
        std::cout << (sample52 ? "52.txt" : "6.txt") << " regression passed: support="
                  << result.referenceSegments[0].indices.size() << ','
                  << second.indices.size() << " angle="
                  << second.normalChangeFromPreviousDegrees << " boundary_gap="
                  << second.boundaryGapFromPrevious << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
