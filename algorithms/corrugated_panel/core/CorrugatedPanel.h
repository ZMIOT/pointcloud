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
    std::vector<ProjectedPoint> projected;
    std::vector<ProjectedPoint> trajectory; // C 的稠密中心轨迹
    std::vector<ProjectedPoint> fittedLine; // RDP 分段后正交最小二乘拟合的直线角点
    std::vector<ProjectedPoint> whitePoints; // 真实轨迹点采样后投影到整段拟合直线
};

struct Hooks {
    std::function<bool()> cancelled;
    std::function<void(int)> progress;
    // 0 reference plane, 1 slices, 2 selected B, 3 projected C, 4 fitted line/white points.
    std::function<bool(int, const Result &)> stage;
    std::function<void(const Plane &, std::size_t, std::size_t, std::size_t, double,
                       double, double)>
        referenceCandidate;
};

std::string validateConfig(const Config &config);
Result extract(const std::vector<Point> &points, const Config &config = {},
               const Hooks &hooks = {});

} // namespace bwb
