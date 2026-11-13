#include "CorrugatedPanel.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

using bwb::Point;
using bwb::Vec3;

void require(bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

std::vector<Point> cloud(bool positiveBent, bool negativeBent) {
    std::vector<Point> points;
    std::mt19937 random(20261112u);
    std::normal_distribution<double> noise(0.0, 0.04);
    auto append = [&](double x, double y, double z) {
        points.push_back({Vec3(x + noise(random), y + noise(random), z + noise(random)),
                          std::uint32_t(points.size())});
    };
    // A 面：z=0，横跨工件；两侧幕面保证它确实把点云分成两边。
    for (int x = -100; x <= 100; x += 2)
        for (int y = -70; y <= 70; y += 2)
            append(x, y, 0.0);
    auto curtain = [&](int side, bool bent) {
        for (int x = -100; x <= 100; ++x) {
            const double y = bent ? 22.0 * std::sin(double(x) / 18.0)
                                  : (side > 0 ? 48.0 : -48.0);
            for (int distance = 5; distance <= 35; ++distance)
                append(double(x), y, side * double(distance));
        }
    };
    curtain(1, positiveBent);
    curtain(-1, negativeBent);
    // 稀疏离群点。
    for (int i = 0; i < 80; ++i)
        append(-100.0 + 2.5 * i, 80.0, 10.0 + (i % 3) * 10.0);
    return points;
}

std::vector<Point> longCurvedCloud() {
    std::vector<Point> points;
    std::mt19937 random(20261113u);
    std::normal_distribution<double> noise(0.0, 0.02);
    auto append = [&](const Vec3 &point) {
        points.push_back({point + Vec3(noise(random), noise(random), noise(random)),
                          std::uint32_t(points.size())});
    };
    constexpr double curvature = 1.0e-5;
    for (int x = -600; x <= 600; x += 4) {
        const double z = curvature * double(x) * double(x);
        for (int y = -70; y <= 70; y += 4)
            append({double(x), double(y), z});
        const Vec3 normal = Vec3(-2.0 * curvature * x, 0.0, 1.0).normalized();
        for (int distance = 5; distance <= 35; ++distance) {
            append(Vec3(double(x), 24.0 * std::sin(double(x) / 45.0), z) +
                   double(distance) * normal);
            append(Vec3(double(x), -48.0, z) - double(distance) * normal);
        }
    }
    return points;
}

std::vector<Point> piecewiseSlopeCloud() {
    std::vector<Point> points;
    std::mt19937 random(20261114u);
    std::normal_distribution<double> noise(0.0, 0.04);
    auto append = [&](double x, double y, double z) {
        points.push_back({Vec3(x + noise(random), y + noise(random), z + noise(random)),
                          std::uint32_t(points.size())});
    };
    for (int x = -120; x <= 120; x += 2)
        for (int y = -70; y <= 70; y += 2)
            append(x, y, 0.0);
    const auto trajectory = [](double x) {
        if (x < -80.0)
            return 0.0;
        if (x < -20.0)
            return (x + 80.0) * 0.5;
        if (x < 20.0)
            return 30.0;
        if (x < 80.0)
            return 30.0 - (x - 20.0) * 0.5;
        return 0.0;
    };
    for (int x = -120; x <= 120; ++x)
        for (int distance = 5; distance <= 35; ++distance) {
            append(x, trajectory(double(x)), distance);
            append(x, -50.0, -distance);
        }
    return points;
}

std::vector<Point> localDistractorCloud() {
    std::vector<Point> points;
    std::mt19937 random(20261115u);
    std::normal_distribution<double> noise(0.0, 0.03);
    auto append = [&](double x, double y, double z) {
        points.push_back({Vec3(x + noise(random), y + noise(random), z + noise(random)),
                          std::uint32_t(points.size())});
    };
    for (int x = -300; x <= 300; x += 3)
        for (int y = -72; y <= 72; y += 3)
            append(double(x), double(y), 0.0);
    for (int x = -300; x <= 300; x += 2) {
        const double wave = 22.0 * std::sin(double(x) / 22.0);
        for (int distance = 5; distance <= 35; ++distance) {
            append(double(x), wave, double(distance));
            append(double(x), -50.0, -double(distance));
        }
    }
    // 只在中间长度区段放置一个更密的斜平面，模拟 121.txt 中会吸引局部
    // RANSAC 的非 A 表面。它不能替代贯穿工件并把点云分到两侧的 z=0 A 面。
    for (int x = -105; x <= 105; x += 2)
        for (int y = -68; y <= 68; y += 2)
            append(double(x), double(y), 24.0 + 0.18 * double(x));
    return points;
}

std::vector<Point> leadingEdgeDistractorCloud() {
    std::vector<Point> points;
    std::mt19937 random(20261116u);
    std::normal_distribution<double> noise(0.0, 0.03);
    auto append = [&](double x, double y, double z) {
        points.push_back({Vec3(x + noise(random), y + noise(random), z + noise(random)),
                          std::uint32_t(points.size())});
    };
    for (int x = -300; x <= 300; x += 3)
        for (int y = -72; y <= 72; y += 3)
            append(double(x), double(y), 0.0);
    for (int x = -300; x <= 300; x += 2) {
        const double wave = 22.0 * std::sin(double(x) / 22.0);
        for (int distance = 5; distance <= 35; ++distance) {
            append(double(x), wave, double(distance));
            append(double(x), -50.0, -double(distance));
        }
    }
    // 在首段增加一个比 A 面更密、但法向偏离约 35 度的平面。端部只有一个
    // 邻段，不能依赖“异常段夹在两个正常段之间”的事后修复。
    for (int x = -300; x <= -5; x += 2)
        for (int y = -68; y <= 68; y += 2)
            append(double(x), double(y), 35.0 + 0.70 * double(x + 150));
    return points;
}

struct RigidTransform {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Vec3 translation = Vec3::Zero();

    Vec3 apply(const Vec3 &point) const { return rotation * point + translation; }
    Vec3 inverse(const Vec3 &point) const {
        return rotation.transpose() * (point - translation);
    }
};

RigidTransform rotation(double radians, const Vec3 &axis,
                        const Vec3 &translation = Vec3::Zero()) {
    return {Eigen::AngleAxisd(radians, axis.normalized()).toRotationMatrix(),
            translation};
}

std::vector<Point> transformed(const std::vector<Point> &points,
                               const RigidTransform &transform) {
    auto result = points;
    for (auto &point : result)
        point.xyz = transform.apply(point.xyz);
    return result;
}

bwb::Config orientationConfig() {
    bwb::Config config;
    // 避免把高斯噪声的极端尾点恰好放在内点阈值上；测试目标是刚性变换等价性。
    config.planeDistance = 0.25;
    config.minPlanePoints = 1000;
    config.denoiseRadius = 1.8;
    config.minNeighbors = 2;
    config.bendDeviation = 2.0;
    config.trajectorySpacing = 2.0;
    config.lineFitTolerance = 1.0;
    config.whitePointSpacing = 10.0;
    config.planeIterations = 250;
    config.maxRansacPoints = 30000;
    return config;
}

double positionTolerance(const std::vector<Point> &points) {
    Vec3 low = points.front().xyz;
    Vec3 high = low;
    for (const auto &point : points) {
        low = low.cwiseMin(point.xyz);
        high = high.cwiseMax(point.xyz);
    }
    return std::max(1e-4, (high - low).norm() * 1e-6);
}

std::vector<std::uint32_t> sources(const std::vector<Point> &points,
                                   const std::vector<int> &indices) {
    std::vector<std::uint32_t> result;
    result.reserve(indices.size());
    for (int index : indices)
        result.push_back(points[std::size_t(index)].source);
    std::sort(result.begin(), result.end());
    return result;
}

double directedGeometryError(const std::vector<bwb::ProjectedPoint> &expected,
                             const std::vector<bwb::ProjectedPoint> &actual,
                             const RigidTransform &transform) {
    double maximum = 0.0;
    for (const auto &point : expected) {
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto &candidate : actual)
            nearest = std::min(nearest,
                               (point.xyz - transform.inverse(candidate.xyz)).norm());
        maximum = std::max(maximum, nearest);
    }
    return maximum;
}

double geometryError(const std::vector<bwb::ProjectedPoint> &expected,
                     const std::vector<bwb::ProjectedPoint> &actual,
                     const RigidTransform &transform) {
    if (expected.empty() || actual.empty())
        return std::numeric_limits<double>::infinity();
    const double forward = directedGeometryError(expected, actual, transform);
    std::vector<bwb::ProjectedPoint> inverseActual = actual;
    for (auto &point : inverseActual)
        point.xyz = transform.inverse(point.xyz);
    const RigidTransform identity;
    return std::max(forward, directedGeometryError(inverseActual, expected, identity));
}

void requireRightHandedFrames(const bwb::Result &result) {
    for (const auto &segment : result.referenceSegments) {
        const Vec3 &x = segment.localLongitudinal;
        const Vec3 &y = segment.localTransverse;
        const Vec3 &z = segment.plane.normal;
        require(std::abs(x.norm() - 1.0) < 1e-8 &&
                    std::abs(y.norm() - 1.0) < 1e-8 &&
                    std::abs(z.norm() - 1.0) < 1e-8,
                "local frame axes must have unit length");
        require(std::abs(x.dot(y)) < 1e-8 && std::abs(x.dot(z)) < 1e-8 &&
                    std::abs(y.dot(z)) < 1e-8,
                "local frame axes must be orthogonal");
        require(x.cross(y).dot(z) > 1.0 - 1e-8,
                "local frame must be right handed");
    }
}

void requireEquivalent(const std::vector<Point> &original,
                       const bwb::Result &expected, const RigidTransform &transform,
                       const bwb::Config &config, const std::string &label) {
    const auto moved = transformed(original, transform);
    const auto actual = bwb::extract(moved, config);
    require(actual.ok, label + ": " + actual.message);
    requireRightHandedFrames(actual);
    require(actual.referenceSegments.size() == expected.referenceSegments.size(),
            label + ": local reference-plane count changed");
    const auto actualReferenceSources = sources(moved, actual.referenceIndices);
    const auto expectedReferenceSources = sources(original, expected.referenceIndices);
    require(actualReferenceSources == expectedReferenceSources,
            label + ": A support points changed (expected " +
                std::to_string(expectedReferenceSources.size()) + ", actual " +
                std::to_string(actualReferenceSources.size()) + ")");
    require(sources(moved, actual.selectedIndices) ==
                sources(original, expected.selectedIndices),
            label + ": B selected points changed");
    require(actual.selectedSlice >= 0 && expected.selectedSlice >= 0 &&
                std::abs(actual.slices[std::size_t(actual.selectedSlice)].offset -
                         expected.slices[std::size_t(expected.selectedSlice)].offset) < 1e-12,
            label + ": selected absolute slice offset changed");
    require(actual.projected.size() == expected.projected.size(),
            label + ": C point count changed");
    require(actual.fittedLine.size() == expected.fittedLine.size(),
            label + ": fitted corner count changed");
    require(actual.whitePoints.size() == expected.whitePoints.size(),
            label + ": white point count changed");
    const double tolerance = positionTolerance(original);
    std::vector<double> expectedGaps, actualGaps;
    for (const auto &segment : expected.referenceSegments)
        if (segment.boundaryGapMeasured)
            expectedGaps.push_back(segment.boundaryGapFromPrevious);
    for (const auto &segment : actual.referenceSegments)
        if (segment.boundaryGapMeasured)
            actualGaps.push_back(segment.boundaryGapFromPrevious);
    std::sort(expectedGaps.begin(), expectedGaps.end());
    std::sort(actualGaps.begin(), actualGaps.end());
    require(expectedGaps.size() == actualGaps.size(),
            label + ": boundary-gap measurement availability changed");
    for (std::size_t i = 0; i < expectedGaps.size(); ++i)
        require(std::abs(expectedGaps[i] - actualGaps[i]) <= tolerance,
                label + ": boundary gap changed after rigid transform");
    const double projectedError =
        geometryError(expected.projected, actual.projected, transform);
    require(projectedError <= tolerance,
            label + ": projected C geometry changed after inverse transform");
    const double trajectoryError =
        geometryError(expected.trajectory, actual.trajectory, transform);
    require(trajectoryError <= tolerance,
            label + ": measured trajectory changed after inverse transform (expected count " +
                std::to_string(expected.trajectory.size()) + ", actual count " +
                std::to_string(actual.trajectory.size()) + ", error " +
                std::to_string(trajectoryError) + ")");
    const double fittedError =
        geometryError(expected.fittedLine, actual.fittedLine, transform);
    require(fittedError <= tolerance,
            label + ": fitted corners changed after inverse transform (error " +
                std::to_string(fittedError) + ")");
    const double whiteError =
        geometryError(expected.whitePoints, actual.whitePoints, transform);
    require(whiteError <= tolerance,
            label + ": white points changed after inverse transform");
    const double maximumError =
        std::max(std::max(projectedError, trajectoryError),
                 std::max(fittedError, whiteError));
    std::cout << "orientation_equivalence=" << label
              << " max_error=" << maximumError
              << " tolerance=" << tolerance << '\n';
}

void corrugatedSideIsSelected() {
    bwb::Config config;
    config.planeDistance = 0.2;
    config.minPlanePoints = 1000;
    config.denoiseRadius = 1.8;
    config.minNeighbors = 2;
    config.bendDeviation = 2.0;
    config.trajectorySpacing = 2.0;
    config.lineFitTolerance = 1.0;
    config.whitePointSpacing = 10.0;
    config.planeIterations = 250;
    config.maxRansacPoints = 30000;
    const auto result = bwb::extract(cloud(true, false), config);
    require(result.ok, result.message);
    require(result.selectedSlice >= 0, "a slice must be selected");
    double minY = result.trajectory.front().xyz.y(), maxY = minY;
    for (const auto &point : result.trajectory) {
        minY = std::min(minY, point.xyz.y());
        maxY = std::max(maxY, point.xyz.y());
    }
    require(maxY - minY > 30.0, "the corrugated curtain must be selected");
    require(result.referenceIndices.size() > 5000, "A plane must contain the base grid");
    require(result.projected.size() > 100, "B must project to C");
    require(result.trajectory.size() > 80, "uniform trajectory must be populated");
    require(result.fittedLine.size() >= 2 &&
                result.fittedLine.size() < result.trajectory.size(),
            "piecewise straight-line fitting must simplify the center trajectory");
    const bool fittedAwayFromSamples = std::any_of(
        result.fittedLine.begin(), result.fittedLine.end(),
        [&](const bwb::ProjectedPoint &fitted) {
            return std::none_of(
                result.trajectory.begin(), result.trajectory.end(),
                [&](const bwb::ProjectedPoint &measured) {
                    return (fitted.xyz - measured.xyz).norm() < 1e-8;
                });
        });
    require(fittedAwayFromSamples,
            "fitted line must be orthogonally refitted instead of reusing RDP samples");
    require(result.whitePoints.size() > 10,
            "white points must be distributed along the fitted line");
    for (const auto &point : result.trajectory)
        require(std::abs(result.reference.normal.dot(point.xyz) + result.reference.d) < 1e-7,
                "trajectory must lie on A");
    for (std::size_t i = 1; i < result.trajectory.size(); ++i) {
        const double distance =
            (result.trajectory[i].xyz - result.trajectory[i - 1].xyz).norm();
        // 采样间隔沿分箱中心折线计算；跨折点的欧氏弦长允许略短于弧长。
        require(distance > config.trajectorySpacing * 0.80 &&
                    distance <= config.trajectorySpacing * 1.01,
                "trajectory must use near-uniform arc-length spacing");
    }
    double maximumWhiteSpacing = 0.0;
    std::size_t cornerCount = 0;
    for (std::size_t i = 0; i < result.whitePoints.size(); ++i) {
        require(std::abs(result.reference.normal.dot(result.whitePoints[i].xyz) +
                         result.reference.d) < 1e-7,
                "white points must lie on A");
        cornerCount += result.whitePoints[i].corner ? 1u : 0u;
        double lineResidual = std::numeric_limits<double>::infinity();
        for (std::size_t segment = 1; segment < result.fittedLine.size(); ++segment) {
            const auto start = result.fittedLine[segment - 1].xyz;
            const auto delta = result.fittedLine[segment].xyz - start;
            const double lengthSquared = delta.squaredNorm();
            if (lengthSquared <= 1e-12)
                continue;
            const double ratio = std::max(0.0, std::min(
                1.0, (result.whitePoints[i].xyz - start).dot(delta) / lengthSquared));
            lineResidual = std::min(
                lineResidual,
                (result.whitePoints[i].xyz - (start + ratio * delta)).norm());
        }
        require(lineResidual < 1e-8,
                "every white point must be projected exactly onto a fitted segment");
        if (!result.whitePoints[i].corner) {
            bool projectedFromMeasured = false;
            for (const auto &measured : result.trajectory) {
                if (measured.source != result.whitePoints[i].source)
                    continue;
                for (std::size_t segment = 1; segment < result.fittedLine.size(); ++segment) {
                    const auto start = result.fittedLine[segment - 1].xyz;
                    const auto delta = result.fittedLine[segment].xyz - start;
                    const double lengthSquared = delta.squaredNorm();
                    if (lengthSquared <= 1e-12)
                        continue;
                    const auto projected = start + delta *
                        ((measured.xyz - start).dot(delta) / lengthSquared);
                    if ((projected - result.whitePoints[i].xyz).norm() < 1e-8) {
                        projectedFromMeasured = true;
                        break;
                    }
                }
                if (projectedFromMeasured)
                    break;
            }
            require(projectedFromMeasured,
                    "white points must be projections of measured trajectory points");
        }
        if (i > 0) {
            const double distance =
                (result.whitePoints[i].xyz - result.whitePoints[i - 1].xyz).norm();
            maximumWhiteSpacing = std::max(maximumWhiteSpacing, distance);
        }
    }
    require(maximumWhiteSpacing < config.whitePointSpacing * 1.50,
            "each fitted segment must be nearly equally spaced");
    require(cornerCount == result.fittedLine.size(),
            "every fitted-line control point must be marked as a white corner point");
    for (const auto &corner : result.fittedLine) {
        const auto found = std::find_if(
            result.whitePoints.begin(), result.whitePoints.end(),
            [&](const bwb::ProjectedPoint &point) {
                return point.corner && (point.xyz - corner.xyz).norm() < 1e-10;
            });
        require(found != result.whitePoints.end(),
                "every turn must appear exactly in the white point sequence");
    }
}

void fallbackToNegativeSide() {
    bwb::Config config;
    config.planeDistance = 0.2;
    config.minPlanePoints = 1000;
    config.denoiseRadius = 1.8;
    config.minNeighbors = 2;
    config.bendDeviation = 2.0;
    config.planeIterations = 250;
    const auto result = bwb::extract(cloud(false, true), config);
    require(result.ok, result.message);
    double minY = result.trajectory.front().xyz.y(), maxY = minY;
    for (const auto &point : result.trajectory) {
        minY = std::min(minY, point.xyz.y());
        maxY = std::max(maxY, point.xyz.y());
    }
    require(maxY - minY > 30.0,
            "the straight side must be skipped in favor of the corrugated side");
}

void twoStraightSidesFail() {
    bwb::Config config;
    config.planeDistance = 0.2;
    config.minPlanePoints = 1000;
    config.denoiseRadius = 1.8;
    config.minNeighbors = 2;
    config.bendDeviation = 2.0;
    config.planeIterations = 250;
    const auto result = bwb::extract(cloud(false, false), config);
    require(!result.ok, "two straight sides must fail");
    require(result.message.find("均为长直") != std::string::npos,
            "failure must explain the straight-slice condition");
}

void cancellationIsObserved() {
    bool cancel = true;
    bwb::Hooks hooks;
    hooks.cancelled = [&] { return cancel; };
    const auto result = bwb::extract(cloud(true, false), {}, hooks);
    require(result.cancelled, "pre-cancelled runs must stop");
}

void longBentReferenceUsesLocalPlanes() {
    bwb::Config config;
    config.planeDistance = 0.25;
    config.referenceSegmentLength = 200.0;
    config.minPlanePoints = 500;
    config.denoiseRadius = 5.0;
    config.minNeighbors = 2;
    config.bendDeviation = 2.0;
    config.trajectorySpacing = 4.0;
    config.lineFitTolerance = 1.0;
    config.whitePointSpacing = 12.0;
    config.planeIterations = 350;
    config.maxRansacPoints = 40000;
    const auto result = bwb::extract(longCurvedCloud(), config);
    require(result.ok, result.message);
    requireRightHandedFrames(result);
    require(result.referenceSegments.size() >= 5,
            "a long panel must create several local A planes");
    require(std::all_of(result.referenceSegments.begin(), result.referenceSegments.end(),
                        [](const bwb::ReferenceSegment &segment) {
                            return segment.locallyFitted && !segment.indices.empty();
                        }),
            "every local A plane must be independently supported by local points");
    const auto &first = result.referenceSegments.front().plane;
    const auto &last = result.referenceSegments.back().plane;
    require(std::abs(first.d - last.d) > 0.5 ||
                (first.normal - last.normal).norm() > 0.005,
            "A1 and An must be allowed to differ");
    for (const auto &point : result.projected) {
        const double coordinate = result.referenceLongitudinal.dot(
            point.xyz - result.referenceOrigin);
        auto found = std::lower_bound(
            result.referenceSegments.begin(), result.referenceSegments.end(), coordinate,
            [](const bwb::ReferenceSegment &segment, double value) {
                return segment.longitudinalMax < value;
            });
        if (found == result.referenceSegments.end())
            found = result.referenceSegments.end() - 1;
        require(std::abs(found->plane.normal.dot(point.xyz) + found->plane.d) < 1e-7,
                "each C point must lie on its corresponding local A plane");
    }
}

void rigidOrientationChangesAreEquivalent() {
    constexpr double pi = 3.14159265358979323846;
    const auto original = cloud(true, false);
    const auto config = orientationConfig();
    const auto expected = bwb::extract(original, config);
    require(expected.ok, expected.message);
    requireRightHandedFrames(expected);

    requireEquivalent(original, expected,
                      rotation(0.5 * pi, Vec3::UnitY()), config,
                      "xoy to yoz rotation");
    requireEquivalent(original, expected,
                      rotation(-0.5 * pi, Vec3::UnitX()), config,
                      "xoy to xoz rotation");

    RigidTransform arbitrary;
    arbitrary.rotation =
        Eigen::AngleAxisd(0.73, Vec3(1.0, 2.0, -0.5).normalized()).toRotationMatrix() *
        Eigen::AngleAxisd(-0.41, Vec3(-0.3, 0.7, 1.0).normalized()).toRotationMatrix();
    require(arbitrary.rotation.determinant() > 1.0 - 1e-12,
            "arbitrary transform must be a proper rotation");
    requireEquivalent(original, expected, arbitrary, config,
                      "arbitrary three-axis rotation");

    arbitrary.translation = Vec3(100000.0, -250000.0, 50000.0);
    requireEquivalent(original, expected, arbitrary, config,
                      "arbitrary rotation with large translation");

    // 镜像输入会迫使平面法向的符号约定改变；输出几何仍应一致，仅侧标签可互换。
    RigidTransform mirrored;
    mirrored.rotation.diagonal() << 1.0, 1.0, -1.0;
    require(mirrored.rotation.determinant() < 0.0,
            "normal-sign test must reverse handedness of the input coordinates");
    requireEquivalent(original, expected, mirrored, config,
                      "reference-normal sign ambiguity");
}

void rotatedLongLocalPlanesAreEquivalent() {
    auto config = orientationConfig();
    config.planeDistance = 0.25;
    config.referenceSegmentLength = 200.0;
    config.minPlanePoints = 500;
    config.denoiseRadius = 5.0;
    config.trajectorySpacing = 4.0;
    config.whitePointSpacing = 12.0;
    config.planeIterations = 350;
    config.maxRansacPoints = 40000;
    const auto original = longCurvedCloud();
    const auto expected = bwb::extract(original, config);
    require(expected.ok, expected.message);
    RigidTransform arbitrary =
        rotation(0.91, Vec3(0.4, -1.0, 0.8), Vec3(-3000.0, 7000.0, 1200.0));
    requireEquivalent(original, expected, arbitrary, config,
                      "rotated long workpiece with local A planes");
}

void abruptLocalPlaneChangeIsRejected() {
    auto config = orientationConfig();
    config.planeDistance = 0.25;
    config.referenceSegmentLength = 200.0;
    config.minPlanePoints = 500;
    config.denoiseRadius = 5.0;
    config.trajectorySpacing = 4.0;
    config.planeIterations = 350;
    config.maxRansacPoints = 40000;
    config.maxAdjacentNormalAngleDegrees = 0.01;
    const auto result = bwb::extract(longCurvedCloud(), config);
    require(!result.ok && result.message.find("法向变化过大") != std::string::npos,
            "an over-limit adjacent local-plane angle must fail explicitly");
    require(!result.referenceSegments.empty(),
            "failed continuity check must retain local-plane evidence");
}

void localDenseDistractorDoesNotReplaceReference() {
    auto config = orientationConfig();
    config.referenceSegmentLength = 180.0;
    config.minPlanePoints = 700;
    config.planeIterations = 350;
    config.maxRansacPoints = 40000;
    const auto result = bwb::extract(localDistractorCloud(), config);
    require(result.ok, result.message);
    require(result.referenceSegments.size() >= 3,
            "distractor regression requires several local A planes");
    for (std::size_t index = 0; index < result.referenceSegments.size(); ++index) {
        const auto &segment = result.referenceSegments[index];
        require(std::abs(segment.plane.normal.dot(Vec3::UnitZ())) > 0.995,
                "a dense local distractor replaced A" + std::to_string(index + 1) +
                    " (normal=" + std::to_string(segment.plane.normal.x()) + "," +
                    std::to_string(segment.plane.normal.y()) + "," +
                    std::to_string(segment.plane.normal.z()) + ")");
        require(segment.planeRms < config.planeDistance,
                "the recovered local A plane has excessive residual");
    }
}

void leadingEdgeDistractorDoesNotReplaceReference() {
    auto config = orientationConfig();
    config.referenceSegmentLength = 310.0;
    config.minPlanePoints = 700;
    config.planeIterations = 350;
    config.maxRansacPoints = 40000;
    const auto result = bwb::extract(leadingEdgeDistractorCloud(), config);
    require(result.ok, result.message);
    require(result.referenceSegments.size() == 2,
            "leading-edge distractor regression requires two local A planes");
    for (std::size_t index = 0; index < result.referenceSegments.size(); ++index)
        require(std::abs(result.referenceSegments[index].plane.normal.dot(Vec3::UnitZ())) >
                    0.995,
                "a leading-edge distractor replaced A" + std::to_string(index + 1));
}

void diffuseBandDoesNotReplaceReference() {
    auto points = cloud(true, false);
    // 两侧均使用弯曲幕面，使 z=0 成为唯一完整平面，隔离厚带评分问题。
    for (auto &point : points)
        if (point.xyz.z() < -1.0)
            point.xyz.y() += 12.0 * std::sin(point.xyz.x() / 13.0);
    std::mt19937 random(20261117u);
    std::uniform_real_distribution<double> x(-100.0, 100.0);
    std::uniform_real_distribution<double> y(-65.0, 65.0);
    std::uniform_real_distribution<double> z(5.0, 15.0);
    for (int i = 0; i < 60000; ++i) {
        const Vec3 point(x(random), y(random), z(random));
        points.push_back({point, std::uint32_t(points.size())});
    }
    auto config = orientationConfig();
    bwb::Hooks hooks;
    // 本测试只验证基准面；体噪声内的切片没有可定义的目标轨迹。
    double legacyScore = -1.0, legacyOffset = 0.0;
    hooks.referenceCandidate = [&](const bwb::Plane &p, std::size_t support,
                                     std::size_t positive, std::size_t negative,
                                     double, double, double) {
        const double balance = positive + negative == 0 ? 0.0 :
            2.0 * double(std::min(positive, negative)) / double(positive + negative);
        const double score = double(support) * (0.10 + balance * balance);
        if (balance >= 0.04 && score > legacyScore) {
            legacyScore = score;
            legacyOffset = std::abs(p.d);
        }
    };
    hooks.stage = [](int, const bwb::Result &) { return false; };
    const auto result = bwb::extract(points, config, hooks);
    require(result.cancelled, "reference-only regression must reach the A checkpoint");
    require(std::abs(result.reference.normal.z()) > 0.995 &&
                std::abs(result.reference.d) < config.planeDistance,
            "a balanced diffuse band replaced the physical z=0 A surface");
    require(legacyOffset > 1.0,
            "diffuse fixture must expose the former count/balance-only misselection");
    hooks.referenceCandidate = {};
    const auto transform = rotation(0.83, Vec3(0.4, -0.7, 1.0), Vec3(2000, -400, 900));
    const auto moved = bwb::extract(transformed(points, transform), config, hooks);
    require(moved.cancelled &&
                std::abs(moved.reference.normal.dot(transform.rotation * Vec3::UnitZ())) >
                    0.995 &&
                std::abs(moved.reference.normal.dot(transform.translation) + moved.reference.d) <
                    config.planeDistance,
            "surface concentration selection changed after a rigid transform");
}

void slopePhasesUseOneLineEach() {
    bwb::Config config;
    config.planeDistance = 0.2;
    config.minPlanePoints = 1000;
    config.denoiseRadius = 1.8;
    config.minNeighbors = 2;
    config.bendDeviation = 2.0;
    config.trajectorySpacing = 2.0;
    config.lineFitTolerance = 3.0;
    config.whitePointSpacing = 10.0;
    config.planeIterations = 250;
    const auto result = bwb::extract(piecewiseSlopeCloud(), config);
    require(result.ok, result.message);
    require(result.fittedLine.size() == 6,
            "horizontal/up/horizontal/down/horizontal must produce exactly five lines");
    require(std::count_if(result.whitePoints.begin(), result.whitePoints.end(),
                          [](const bwb::ProjectedPoint &point) {
                              return point.corner;
                          }) == 6,
            "all five phase transitions and endpoints must remain in white points");
}

} // namespace

int main() {
    corrugatedSideIsSelected();
    fallbackToNegativeSide();
    twoStraightSidesFail();
    cancellationIsObserved();
    longBentReferenceUsesLocalPlanes();
    rigidOrientationChangesAreEquivalent();
    rotatedLongLocalPlanesAreEquivalent();
    abruptLocalPlaneChangeIsRejected();
    localDenseDistractorDoesNotReplaceReference();
    leadingEdgeDistractorDoesNotReplaceReference();
    diffuseBandDoesNotReplaceReference();
    slopePhasesUseOneLineEach();
    std::cout << "CorrugatedPanelCore tests passed\n";
    return 0;
}
