#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace bwb {

using Vec3 = Eigen::Vector3d;

struct Point {
    Vec3 xyz = Vec3::Zero();
    std::uint32_t source = 0;
};

struct Config {
    std::array<double, 3> sliceOffsets{{10.0, 20.0, 30.0}};
    double sliceThickness = 3.0;
    double planeDistance = 0.6;
    double referenceSegmentLength = 500.0;
    double maxAdjacentNormalAngleDegrees = 15.0;
    int planeIterations = 600;
    int planeCandidates = 6;
    int maxRansacPoints = 60000;
    int minPlanePoints = 200;
    double denoiseRadius = 2.5;
    int minNeighbors = 3;
    double bendDeviation = 2.0;
    double trajectorySpacing = 2.0;
    double lineFitTolerance = 3.0;
    double whitePointSpacing = 10.0;
    double maxSurfaceExtrapolation = 40.0; // Intentional extension from the off-edge body band.
    double surfaceFitTolerance = 0.6;
    int minSurfacePoints = 20;
    double minIntersectionAngleDegrees = 10.0;
    bool allowProjectionFallback = true;
    int threads = 0; // 0: OpenMP runtime default
};

struct Plane {
    Vec3 normal = Vec3::UnitZ();
    Vec3 origin = Vec3::Zero();
    double d = 0.0; // normal.dot(x) + d = 0
};

struct SliceEvaluation {
    int side = 1;
    double offset = 0.0;
    std::vector<int> raw;
    std::vector<int> filtered;
    double length = 0.0;
    double completeness = 0.0;
    double deviation = 0.0;
    double linearityRatio = 0.0;
    double score = 0.0;
    bool bent = false;
};

struct ReferenceSegment {
    Plane plane;
    Vec3 localLongitudinal = Vec3::UnitX();
    Vec3 localTransverse = Vec3::UnitY();
    double longitudinalMin = 0.0;
    double longitudinalMax = 0.0;
    double planeRms = 0.0;
    double normalChangeFromPreviousDegrees = 0.0;
    // 在实际边界点的横向 5%/50%/95% 位置，沿种子法向测得的最大相邻面间距。
    double boundaryGapFromPrevious = 0.0;
    bool boundaryGapMeasured = false;
    bool locallyFitted = false;
    std::vector<int> indices;
};

struct ProjectedPoint {
    Vec3 xyz = Vec3::Zero();
    std::uint32_t source = 0;
    bool corner = false;
    std::uint32_t referenceIndex = 0;
    bool extrapolated = false;
    double extrapolationDistance = 0.0;
    double surfaceRms = 0.0;
    std::uint32_t supportCount = 0;
    int surfaceDegree = 0;
    bool projectionFallback = false;
};

struct IntersectionStation {
    double longitudinal = 0.0;
    ProjectedPoint point;
    ProjectedPoint projectionCandidate;
    Vec3 longitudinalTangent = Vec3::Zero();
    Vec3 surfaceHeightDirection = Vec3::Zero(); // dX/dh at A for the attempted surface model.
    Vec3 projectionTangent = Vec3::Zero();
    double boundaryIntersectionGap = 0.0;
    double supportMinHeight = 0.0;
    double supportMaxHeight = 0.0;
    double attemptedExtrapolation = 0.0;
    bool localPatchExtension = false; // Validated local face direction, not projection.
    double localPatchPredictionSpread = 0.0;
    std::string broadModelFailure; // Preserved even when the local patch succeeds.
    double seedLongitudinalSpan = 0.0;
    bool lengthSlopeIdentifiable = false;
    bool hasProjectionCandidate = false;
    bool valid = false;
    std::string failure;
    std::string intersectionFailure;
};

struct Result {
    bool ok = false;
    bool cancelled = false;
    std::string message;
    Plane reference;
    Vec3 referenceOrigin = Vec3::Zero();
    Vec3 referenceLongitudinal = Vec3::UnitX();
    Vec3 referenceTransverse = Vec3::UnitY();
    std::vector<ReferenceSegment> referenceSegments;
    std::vector<int> referenceIndices;
    std::vector<SliceEvaluation> slices;
    int selectedSlice = -1;
    std::vector<int> selectedIndices;
    std::vector<ProjectedPoint> projected; // Legacy member name; mode-uniform feature-curve candidates.
    std::vector<IntersectionStation> intersectionStations;
    bool usedProjectionFallback = false; // true 时整条曲线均为 B 到 A 的投影，不与求交点混用
    double bodyBandMin = 0.0;
    double bodyBandMax = 0.0;
    std::vector<ProjectedPoint> trajectory; // 求交恢复或全局投影回退曲线，不是直接实测点
    std::vector<ProjectedPoint> fittedLine; // 恢复曲线上的坡段起止控制点，一坡一段
    std::vector<ProjectedPoint> whitePoints; // 恢复曲线采样后投影到整段拟合直线
};

struct Hooks {
    std::function<bool()> cancelled;
    std::function<void(int)> progress;
    // 0 reference plane, 1 slices, 2 selected B, 3 intersection C, 4 fitted line/white points.
    std::function<bool(int, const Result &)> stage;
    std::function<void(const Plane &, std::size_t, std::size_t, std::size_t, double,
                       double, double)>
        referenceCandidate;
};

std::string validateConfig(const Config &config);
Result extract(const std::vector<Point> &points, const Config &config = {},
               const Hooks &hooks = {});

} // namespace bwb
