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
    const bool sample118 = argc > 1 && std::string(argv[1]) == "118";
    const bool sample52 = argc > 1 && std::string(argv[1]) == "52";
    const bool sample121 = argc > 1 && std::string(argv[1]) == "121";
    const bool sample1 = argc > 1 && std::string(argv[1]) == "1";
    const bool sample3 = argc > 1 && std::string(argv[1]) == "3";
    const bool sample8 = argc > 1 && std::string(argv[1]) == "8";
    const char *variable = sample118 ? "BWB_SAMPLE_118_PATH" : sample121 ? "BWB_SAMPLE_121_PATH" :
        (sample52 ? "BWB_SAMPLE_52_PATH" :
            (sample1 ? "BWB_SAMPLE_1_PATH" : (sample3 ? "BWB_SAMPLE_3_PATH" :
                (sample8 ? "BWB_SAMPLE_8_PATH" : "BWB_SAMPLE_6_PATH"))));
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
        require(input.eof() && points.size() == (sample118 ? 5779410u : sample121 ? 4238245u :
                    (sample1 ? 1448522u : (sample52 ? 831069u :
                        (sample3 ? 1100339u : (sample8 ? 593461u : 1309174u))))),
                "unexpected real-sample point count");
        bwb::Config config;
        if (sample52) config.allowProjectionFallback = false;
        const auto result = bwb::extract(points, config);
        if (sample118) {
            require(result.ok && result.prefilterApplied && !result.usedProjectionFallback,
                    "118 prefilter failed to recover the off-edge body curve");
            require(result.referenceSegments.size()==6 && result.referenceIndices.size()==1351011 &&
                        result.retainedIndices.size()+result.rejectedIndices.size()==points.size(),
                    "118 reference evidence or filter partition changed");
            require(result.rejectedIndices.size()>100000 && result.retainedIndices.size()>points.size()*0.75,
                    "118 filter removed no interference or erased most of the panel");
            for (int i:result.referenceIndices) require(result.surfaceMask[std::size_t(i)],"118 A surface was removed");
            for (const auto &slice:result.slices) for (int i:slice.raw)
                require(result.surfaceMask[std::size_t(i)],"118 removed point entered slice");
            require(result.projected.size()==1259 && result.fittedLine.size()==38 &&
                        result.intersectionStations.size()==result.projected.size(),
                    "118 curve lost length coverage or folds");
            for (const auto &station:result.intersectionStations)
                require(station.valid && station.failure.empty() && station.intersectionFailure.empty() &&
                            result.surfaceMask[station.point.source],"118 unsupported station or rejected provenance");
            const auto &selected=result.slices[std::size_t(result.selectedSlice)];
            require(selected.bent && selected.completeness>0.95 && selected.length>2450,
                    "118 corrugated side is incomplete");
            require(std::any_of(result.slices.begin(),result.slices.end(),[&](const bwb::SliceEvaluation &s) {
                return s.side!=selected.side && !s.bent && s.completeness>0.95 && s.length>2450;
            }),"118 straight platform side remains confused with corrugation");
            std::cout << "118 prefilter regression passed: retained=" << result.retainedIndices.size()
                      << " removed=" << result.rejectedIndices.size() << " stations=" << result.projected.size() << '\n';
            return 0;
        }
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
            require((result.fittedLine.front().xyz-result.trajectory.front().xyz).norm() < config.lineFitTolerance &&
                        (result.fittedLine.back().xyz-result.trajectory.back().xyz).norm() < config.lineFitTolerance,
                    "3.txt endpoint projection moved beyond the line-fit tolerance");
            for (const auto &corner : result.fittedLine)
                require(corner.lineFitted && corner.lineIntersectionGap <= config.surfaceFitTolerance,
                        "3.txt synthetic junction lost its fitting evidence");
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
        if (sample8) {
            require(result.referenceSegments.size() == 2 && result.usedProjectionFallback &&
                        result.projected.size() == 384,
                    "8.txt feature coverage or fallback changed");
            require(result.fittedLine.size() == 18,
                    "8.txt narrow scan-line windows created spurious slope segments");
            require(result.lineModels.size() == 17, "8.txt must fit one line per phase");
            double squaredResidual = 0;
            for (const auto &p : result.trajectory) {
                double nearest = 1e30;
                for (std::size_t j = 1; j < result.fittedLine.size(); ++j) {
                    const bwb::Vec3 start = result.fittedLine[j-1].xyz;
                    const bwb::Vec3 delta = result.fittedLine[j].xyz-start;
                    const double t = std::clamp((p.xyz-start).dot(delta)/delta.squaredNorm(),0.0,1.0);
                    nearest = std::min(nearest,(p.xyz-start-t*delta).squaredNorm());
                }
                squaredResidual += nearest;
            }
            // Internal model-consistency guard, NOT a physical accuracy assertion.
            require(std::sqrt(squaredResidual/result.trajectory.size()) < 0.75,
                    "8.txt reverted to chords pinned to recovered samples");
            for (const auto &line : result.lineModels)
                require(line.support*2 > line.last-line.first+1,
                        "8.txt fitted a line without majority support");
            const auto suppressed = std::count_if(
                result.intersectionStations.begin(), result.intersectionStations.end(),
                [](const bwb::IntersectionStation &s) { return !s.lengthSlopeIdentifiable; });
            require(suppressed == 8,
                    "8.txt did not preserve the known narrow scan-line evidence");
            double maximumStep = 0;
            for (std::size_t i=1; i<result.projected.size(); ++i)
                maximumStep = std::max(maximumStep,
                    (result.projected[i].xyz-result.projected[i-1].xyz).norm());
            require(maximumStep < 5.0,
                    "8.txt projection candidates contain an isolated jump");
            std::cout << "8.txt narrow-window regression passed: suppressed=" << suppressed
                      << " max_step=" << maximumStep << '\n';
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
                            [](const bwb::IntersectionStation &s) { return s.localPatchExtension; }) > 0,
                    "52.txt lost validated local face extensions");
            // Prefiltering can let a previously rejected broad model pass, so local
            // fallback counts need not remain 63. Keep that baseline regression AND
            // constrain geometry, coverage and curve mode with filtering enabled.
            auto baselineConfig=config;baselineConfig.prefilterEnabled=false;
            const auto baseline=bwb::extract(points,baselineConfig);
            require(baseline.ok && !baseline.usedProjectionFallback &&
                        baseline.projected.size()==result.projected.size() &&
                        std::count_if(baseline.intersectionStations.begin(),baseline.intersectionStations.end(),
                            [](const bwb::IntersectionStation &s) { return s.localPatchExtension; })==63,
                    "52.txt unfiltered baseline changed");
            double maxFilterShift=0;
            for (std::size_t i=0;i<baseline.projected.size();++i)
                maxFilterShift=std::max(maxFilterShift,(baseline.projected[i].xyz-result.projected[i].xyz).norm());
            require(maxFilterShift<0.1 && baseline.fittedLine.size()==result.fittedLine.size(),
                    "52.txt prefilter moved the curve or lost fold coverage");
            std::cout << "52 prefilter max_curve_shift=" << maxFilterShift << '\n';
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
