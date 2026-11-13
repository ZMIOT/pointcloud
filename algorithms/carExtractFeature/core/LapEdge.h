#pragma once
#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace lap {
using Vec = Eigen::Vector3d;
// 所有距离和阈值均使用输入点云的坐标单位；0 法向表示从扫描点序自动估计。
struct Config {
    Vec up = Vec::UnitZ();
    Vec scanNormal = Vec::Zero(); // zero: estimate from ordered scan samples
    double neighborhoodScale = 1.0;
    double smoothing = 1.0;
    int maxMissing = 3;
    double evidenceScale = 1.0; // regression-test control, not a physical distance
};
struct Point {
    Vec xyz;
    std::uint32_t source = 0;
};
struct Section {
    double s = 0, spacing = 0, noise = 0;
    std::vector<int> rawPoints; // 按横向排序的完整原始点索引，用于最终端点定位
    std::vector<int> points;    // 分析代表点索引；稀疏截面与 rawPoints 相同
};
struct Surface {
    // z = c0+c1*x+c2*s+c3*x*x+c4*x*s+c5*s*s, in centered/scaled coordinates.
    std::array<double, 6> c{};
    double x0 = 0, s0 = 0, dx = 1, ds = 1, rms = 0;
    int degree = 1, supportSections = 0;
    bool valid = false;
    double height(double x, double s) const;
};
enum class Reason {
    Pending,
    Accepted,
    LowerSide,
    WeakStep,
    ShortSupport,
    Isolated,
    NoSurface,
    SurfaceMismatch,
    NoLayerSeparation,
    EndpointMismatch,
    DetachedFragment,
    NotSelected
};
const char *reasonName(Reason reason);
// 原始点回查的诊断结果；数值保持稳定，便于分析已有 candidates.csv。
enum class RawDecision : int {
    None = 0,
    ConnectedTip = 1,
    DetachedToMain = 2,
    RejectedFragment = 3,
    ShortLocalTip = 4,
    SurfaceFallback = 5,
    SparseInput = 6,
    RepeatableShortTip = 7
};
struct Candidate {
    // point 是上层端点，other 是断点另一侧；side=-1 表示上层内部在横向左侧。
    int section = 0, point = 0, other = 0, side = -1;
    double score = 0, profileScore = 0, step = 0, gap = 0;
    double upperSlope = 0, lowerSlope = 0;
    int upperCount = 0, lowerCount = 0, radius = 0, endpointSupport = 0;
    int upperSurfaceSections = 0, lowerSurfaceSections = 0;
    double surfaceNoise = 0, surfaceResidual = 0;
    int rawCoarsePoint = -1, rawRunCount = 0;
    RawDecision rawDecision = RawDecision::None;
    double rawRunSpan = 0;
    bool eligible = false;
    Reason reason = Reason::Pending;
    Surface upper, lower;
};
struct Sample {
    Vec xyz; // 实测点保持原始 XYZ；轨迹点可以是平滑或插补后的 XYZ
    std::uint32_t source = 0, section = 0, segment = 0;
    float confidence = 0;
    bool interpolated = false;
};
struct Result {
    bool ok = false, cancelled = false;
    std::string message;
    Vec scanNormal = Vec::Zero(), transverse = Vec::Zero(), vertical = Vec::Zero();
    double sectionSpacing = 0, pointSpacing = 0;
    std::vector<Point> input;
    std::vector<Vec> local; // (横向 x, 截面位置 y, 向上 z)，与 input 索引一一对应
    std::vector<Section> sections;
    std::vector<int> isolated, selected; // selected 存储 candidates 的索引
    std::vector<Candidate> candidates;
    std::vector<Sample> measured, trajectory; // 分别为原始实测点和拟合轨迹
};
struct Hooks {
    std::function<bool()> cancelled;
    std::function<bool(int, const Result &)> stage; // stages 0..5, false requests stop
    std::function<void(int)> progress;
};
std::string validateConfig(const Config &config);
Result detect(const std::vector<Point> &points, const Config &config = {}, const Hooks &hooks = {});
} // namespace lap
