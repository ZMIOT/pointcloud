#include "CorrugatedPanel.h"
#include <Eigen/Geometry>

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
    const bool sample1 = argc > 1 && std::string(argv[1]) == "1";
    const bool sample3 = argc > 1 && std::string(argv[1]) == "3";
    const char *variable = sample121 ? "BWB_SAMPLE_121_PATH" :
        (sample52 ? "BWB_SAMPLE_52_PATH" :
            (sample1 ? "BWB_SAMPLE_1_PATH" : (sample3 ? "BWB_SAMPLE_3_PATH" : "BWB_SAMPLE_6_PATH")));
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
        require(input.eof() && points.size() == (sample121 ? 4238245u :
                    (sample1 ? 1448522u : (sample52 ? 831069u : (sample3 ? 1100339u : 1309174u)))),
                "unexpected real-sample point count");
        bwb::Config config;
        if (sample52) config.allowProjectionFallback = false;
        const auto result = bwb::extract(points, config);
        require(result.bodyBandMin == 8.5 && result.bodyBandMax == 31.5,
                "sample must use the off-edge body support band");
        for (const auto &station : result.intersectionStations)
            if (station.supportMinHeight > 0)
                require(station.supportMinHeight >= 8.5-1e-7 &&
                            station.supportMaxHeight <= 31.5+1e-7,
                        "edge support leaked into a real-sample body fit");
        if (sample121) {
            require(result.referenceSegments.size() == 6, "121 local A count regressed");
            for (const auto &a : result.referenceSegments)
                require(a.indices.size() > 150000 && a.planeRms < 0.25,
                        "121 local A support/residual regressed");
            require(result.ok && result.usedProjectionFallback &&
                        result.intersectionStations.size() > 1000 && !result.projected.empty(),
                    "121 surface evidence missing");
            require(std::any_of(result.intersectionStations.begin(), result.intersectionStations.end(),
                        [](const bwb::IntersectionStation &s) { return !s.intersectionFailure.empty(); }),
                    "121 fallback lost the primary intersection failure evidence");
            require(std::all_of(result.projected.begin(), result.projected.end(),
                        [](const bwb::ProjectedPoint &p) { return p.projectionFallback; }),
                    "121 mixed intersection and projection points");
            std::cout << "121.txt whole-curve projection fallback verified\n";
            return 0;
        }
        require(result.ok, result.message.c_str());
        if (sample3) {
            require(result.referenceSegments.size() == 2 && !result.usedProjectionFallback &&
                        result.projected.size() == 383, "3.txt feature coverage or fallback changed");
            require(result.fittedLine.size() == 18, "3.txt gained a spurious slope segment");
            require((result.fittedLine.front().xyz-result.trajectory.front().xyz).norm() < 1e-10 &&
                        (result.fittedLine.back().xyz-result.trajectory.back().xyz).norm() < 1e-10,
                    "3.txt endpoint was relocated by fitting");
            for (const auto &corner : result.fittedLine)
                require(std::any_of(result.trajectory.begin(), result.trajectory.end(),
                        [&](const bwb::ProjectedPoint &p) { return (p.xyz-corner.xyz).norm() < 1e-10; }),
                        "3.txt slope transition left the recovered curve");
            for (const auto &p : result.projected) {
                require(!p.projectionFallback && p.referenceIndex < result.referenceSegments.size(),
                        "3.txt mixed curve modes or lost A ownership");
                const auto &plane = result.referenceSegments[p.referenceIndex].plane;
                require(std::abs(plane.normal.dot(p.xyz)+plane.d) < 1e-7,
                        "3.txt feature point is not on A");
                const auto &source = points[p.source].xyz;
                require(std::abs(plane.normal.dot(source)+plane.d) >= 8.5-1e-7,
                        "3.txt trajectory source is in the excluded edge region");
            }
            require(std::count_if(result.intersectionStations.begin(),result.intersectionStations.end(),
                        [](const bwb::IntersectionStation &s) { return s.localPatchExtension; }) == 1,
                    "3.txt must retain local patch recovery evidence");
            std::cout << "3.txt body-face extension passed: 383 stations, 18 fitted corners\n";
            return 0;
        }
        if (sample1) {
            require(result.referenceSegments.size() == 1, "1.txt local A count regressed");
            require(result.referenceSegments[0].indices.size() > 250000 &&
                        result.referenceSegments[0].planeRms < 0.25,
                    "1.txt local A support/residual regressed");
            require(!result.usedProjectionFallback && result.intersectionStations.size() == 194,
                    "1.txt must recover from body evidence without the welded edge");
            const auto failedAttempts = std::count_if(
                result.intersectionStations.begin(), result.intersectionStations.end(),
                [](const bwb::IntersectionStation &s) { return !s.intersectionFailure.empty(); });
            require(failedAttempts == 0 &&
                        std::all_of(result.projected.begin(), result.projected.end(),
                            [](const bwb::ProjectedPoint &p) { return !p.projectionFallback; }),
                    "1.txt body recovery unexpectedly failed or mixed modes");
            for (const auto &point : result.projected) {
                const auto &plane = result.referenceSegments[point.referenceIndex].plane;
                require(std::abs(plane.normal.dot(point.xyz) + plane.d) < 1e-7,
                        "1.txt fallback point does not lie on A");
            }
            std::cout << "1.txt regression passed: stations=" << result.projected.size()
                      << " failed_intersection_attempts=" << failedAttempts << '\n';
            return 0;
        }
        require(result.usedProjectionFallback == !sample52,
                "52 must extend its face; 6 must retain fallback for unsupported stations");
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
            require(result.projected.size() == 385 &&
                        std::count_if(result.intersectionStations.begin(),result.intersectionStations.end(),
                            [](const bwb::IntersectionStation &s) { return s.localPatchExtension; }) == 63,
                    "52.txt lost validated local face extensions");
            for (const auto &s : result.intersectionStations) {
                require(s.valid && s.failure.empty() && s.intersectionFailure.empty() &&
                            s.localPatchPredictionSpread <= config.surfaceFitTolerance,
                        "52.txt has an unsupported intersection");
                const double shift = (s.point.xyz-s.projectionCandidate.xyz).norm();
                require(shift > 3.5 && shift < 5.6,
                        "52.txt reverted to normal projection or changed extension direction");
                // Independent measured-B check: the recovered local plane must
                // still contain the slice points, not merely land somewhere on A.
                const auto normal = s.longitudinalTangent.cross(s.surfaceHeightDirection).normalized().eval();
                std::vector<double> residuals;
                for (int index : result.selectedIndices) {
                    const auto delta = (points[std::size_t(index)].xyz-s.point.xyz).eval();
                    if (std::abs(result.referenceLongitudinal.dot(delta)) <= 1.0)
                        residuals.push_back(std::abs(normal.dot(delta)));
                }
                require(!residuals.empty(), "52.txt lacks B support for recovered face");
                std::sort(residuals.begin(),residuals.end());
                require(residuals[residuals.size()/2] < config.surfaceFitTolerance,
                        "52.txt extension direction does not agree with measured B face");
            }
        }
        for (const auto &point : result.projected) {
            require(point.referenceIndex < result.referenceSegments.size(), "invalid intersection owner");
            const auto &plane = result.referenceSegments[point.referenceIndex].plane;
            require(point.projectionFallback == !sample52, "sample mixed body-extension and projection modes");
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
