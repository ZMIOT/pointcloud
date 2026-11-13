#include "CorrugatedPanel.h"
#include "SurfaceIntersection.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>

#ifdef BWB_HAS_OPENMP
#include <omp.h>
#endif

namespace bwb {
namespace {

constexpr double kEpsilon = 1e-12;
constexpr double kRadiansToDegrees = 57.2957795130823208768;

std::uint64_t mix(std::uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
}

bool finite(const Vec3 &p) { return p.allFinite(); }

bool stopped(const Hooks &hooks) { return hooks.cancelled && hooks.cancelled(); }

void reportProgress(const Hooks &hooks, int value) {
    if (hooks.progress)
        hooks.progress(std::max(0, std::min(100, value)));
}

bool stage(const Hooks &hooks, int index, const Result &result) {
    return !hooks.stage || hooks.stage(index, result);
}

template <typename T> T quantile(std::vector<T> values, double q) {
    if (values.empty())
        return T{};
    q = std::max(0.0, std::min(1.0, q));
    // 线性插值分位数关于坐标取反保持对称；向下取整会让 q 与 1-q 在偶数样本上
    // 选择不互补的秩，导致同一工件旋转后区段边界偶尔多/少一个点。
    std::sort(values.begin(), values.end());
    const double position = q * double(values.size() - 1);
    const std::size_t lower = std::size_t(std::floor(position));
    const std::size_t upper = std::size_t(std::ceil(position));
    const double fraction = position - double(lower);
    return T(values[lower] * (1.0 - fraction) + values[upper] * fraction);
}

Plane fitPlane(const std::vector<Point> &points, const std::vector<int> &indices,
               const Plane &fallback) {
    if (indices.size() < 3)
        return fallback;
    Vec3 mean = Vec3::Zero();
    for (int index : indices)
        mean += points[std::size_t(index)].xyz;
    mean /= double(indices.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int index : indices) {
        const Vec3 delta = points[std::size_t(index)].xyz - mean;
        covariance.noalias() += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return fallback;
    Plane plane;
    plane.normal = solver.eigenvectors().col(0).normalized();
    if (plane.normal.dot(fallback.normal) < 0)
        plane.normal = -plane.normal;
    plane.origin = mean;
    plane.d = -plane.normal.dot(mean);
    return plane;
}

std::vector<int> inliers(const std::vector<Point> &points, const Plane &plane,
                         double threshold) {
    std::vector<int> result;
    result.reserve(points.size() / 8);
    for (std::size_t i = 0; i < points.size(); ++i)
        if (finite(points[i].xyz) &&
            std::abs(plane.normal.dot(points[i].xyz) + plane.d) <= threshold)
            result.push_back(int(i));
    return result;
}

struct PlaneCandidate {
    Plane plane;
    std::vector<int> fullInliers;
    double score = -1.0;
};

PlaneCandidate detectReference(const std::vector<Point> &points, const Config &config,
                               const Hooks &hooks) {
    std::vector<int> sample;
    const std::size_t stride =
        std::max<std::size_t>(1, points.size() / std::size_t(config.maxRansacPoints));
    sample.reserve(std::min<std::size_t>(points.size(), config.maxRansacPoints));
    for (std::size_t i = 0; i < points.size(); i += stride)
        if (finite(points[i].xyz))
            sample.push_back(int(i));

    std::vector<int> active = sample;
    PlaneCandidate best;
    PlaneCandidate largest;
    for (int candidate = 0; candidate < config.planeCandidates && active.size() >= 3;
         ++candidate) {
        if (stopped(hooks))
            return {};
        Plane ransacBest;
        int bestCount = 0;
        int bestIteration = std::numeric_limits<int>::max();
        const int iterations = config.planeIterations;
#ifdef BWB_HAS_OPENMP
#pragma omp parallel
#endif
        {
            int localCount = 0;
            int localIteration = std::numeric_limits<int>::max();
            Plane localPlane;
#ifdef BWB_HAS_OPENMP
#pragma omp for schedule(static)
#endif
            for (int iteration = 0; iteration < iterations; ++iteration) {
                // 每次迭代只由 candidate/iteration 决定采样，与线程调度无关。
                const std::uint64_t seed =
                    std::uint64_t(candidate + 1) * 0xB0B2026ULL + std::uint64_t(iteration);
                const auto pick = [&](std::uint64_t salt) {
                    return std::size_t(mix(seed + salt) % active.size());
                };
                const Vec3 a = points[std::size_t(active[pick(0)])].xyz;
                const Vec3 b = points[std::size_t(active[pick(1)])].xyz;
                const Vec3 c = points[std::size_t(active[pick(2)])].xyz;
                Vec3 normal = (b - a).cross(c - a);
                const double norm = normal.norm();
                if (norm <= kEpsilon)
                    continue;
                normal /= norm;
                const double d = -normal.dot(a);
                int count = 0;
                for (int index : active)
                    if (std::abs(normal.dot(points[std::size_t(index)].xyz) + d) <=
                        config.planeDistance)
                        ++count;
                if (count > localCount || (count == localCount && iteration < localIteration)) {
                    localCount = count;
                    localIteration = iteration;
                    localPlane.normal = normal;
                    localPlane.origin = a;
                    localPlane.d = d;
                }
            }
#ifdef BWB_HAS_OPENMP
#pragma omp critical
#endif
            {
                if (localCount > bestCount ||
                    (localCount == bestCount && localIteration < bestIteration)) {
                    bestCount = localCount;
                    bestIteration = localIteration;
                    ransacBest = localPlane;
                }
            }
        }
        if (bestCount < 3)
            break;
        std::vector<int> sampleInliers;
        sampleInliers.reserve(std::size_t(bestCount));
        for (int index : active)
            if (std::abs(ransacBest.normal.dot(points[std::size_t(index)].xyz) +
                         ransacBest.d) <= config.planeDistance)
                sampleInliers.push_back(index);
        Plane refined = fitPlane(points, sampleInliers, ransacBest);
        auto full = inliers(points, refined, config.planeDistance);
        refined = fitPlane(points, full, refined);
        full = inliers(points, refined, config.planeDistance);

        std::size_t positive = 0, negative = 0, shoulder = 0;
        double squaredError = 0.0;
        const double exclusion = config.planeDistance * 1.5;
#ifdef BWB_HAS_OPENMP
#pragma omp parallel for reduction(+ : positive, negative, shoulder, squaredError) schedule(static)
#endif
        for (long long i = 0; i < static_cast<long long>(points.size()); ++i) {
            if (!finite(points[std::size_t(i)].xyz))
                continue;
            const double distance =
                refined.normal.dot(points[std::size_t(i)].xyz) + refined.d;
            positive += distance > exclusion ? 1u : 0u;
            negative += distance < -exclusion ? 1u : 0u;
            const double absoluteDistance = std::abs(distance);
            if (absoluteDistance <= config.planeDistance)
                squaredError += distance * distance;
            else if (absoluteDistance <= config.planeDistance * 3.0)
                ++shoulder;
        }
        const double balance = positive + negative == 0
                                   ? 0.0
                                   : 2.0 * double(std::min(positive, negative)) /
                                         double(positive + negative);
        PlaneCandidate evaluated;
        evaluated.plane = refined;
        evaluated.fullInliers = std::move(full);
        // 核心带宽 2t，外侧两肩总带宽 4t。扣除按带宽折算的背景点数，避免
        // 将斜穿曲面/体点云、仅因正负侧均衡而得分很高的平面误当真实 A 面。
        const double peakSupport = std::max(0.0,
            double(evaluated.fullInliers.size()) - 0.5 * double(shoulder));
        evaluated.score = peakSupport * (0.10 + balance * balance);
        if (hooks.referenceCandidate)
            hooks.referenceCandidate(refined, evaluated.fullInliers.size(), positive,
                                     negative, evaluated.score,
                                     std::sqrt(squaredError /
                                         std::max(1.0, double(evaluated.fullInliers.size()))),
                                     peakSupport);
        if (evaluated.fullInliers.size() > largest.fullInliers.size())
            largest = evaluated;
        if (balance >= 0.04 && evaluated.score > best.score)
            best = evaluated;

        std::vector<int> remaining;
        remaining.reserve(active.size() - sampleInliers.size());
        for (int index : active)
            if (std::abs(refined.normal.dot(points[std::size_t(index)].xyz) + refined.d) >
                config.planeDistance * 1.5)
                remaining.push_back(index);
        active.swap(remaining);
    }
    return best.score >= 0 ? best : largest;
}

std::pair<Vec3, Vec3> planeBasis(const Vec3 &normal) {
    const Vec3 helper = std::abs(normal.z()) < 0.85 ? Vec3::UnitZ() : Vec3::UnitY();
    const Vec3 u = normal.cross(helper).normalized();
    return {u, normal.cross(u).normalized()};
}

double normalAngleDegrees(const Vec3 &left, const Vec3 &right) {
    const double cosine = std::max(-1.0, std::min(1.0, left.dot(right)));
    return std::acos(cosine) * kRadiansToDegrees;
}

struct ReferenceModel {
    Vec3 origin = Vec3::Zero();
    Vec3 longitudinal = Vec3::UnitX();
    Vec3 transverse = Vec3::UnitY();
    Vec3 depth = Vec3::UnitZ();
    std::vector<ReferenceSegment> segments;
};

double longitudinalCoordinate(const ReferenceModel &model, const Vec3 &point) {
    return model.longitudinal.dot(point - model.origin);
}

std::size_t referenceSegmentIndex(const ReferenceModel &model, const Vec3 &point) {
    const double coordinate = longitudinalCoordinate(model, point);
    if (model.segments.size() <= 1)
        return 0;
    auto found = std::lower_bound(
        model.segments.begin(), model.segments.end(), coordinate,
        [](const ReferenceSegment &segment, double value) {
            return segment.longitudinalMax < value;
        });
    if (found == model.segments.end())
        return model.segments.size() - 1;
    return std::size_t(found - model.segments.begin());
}

const Plane &localPlane(const ReferenceModel &model, const Vec3 &point) {
    return model.segments[referenceSegmentIndex(model, point)].plane;
}

double signedReferenceDistance(const ReferenceModel &model, const Vec3 &point) {
    const auto &plane = localPlane(model, point);
    return plane.normal.dot(point) + plane.d;
}

Eigen::Vector2d flattenReference(const ReferenceModel &model, const Vec3 &point) {
    const Vec3 delta = point - model.origin;
    return {model.longitudinal.dot(delta), model.transverse.dot(delta)};
}

std::vector<int> domainInliers(const std::vector<Point> &points,
                               const std::vector<int> &domain, const Plane &plane,
                               double threshold) {
    std::vector<int> result;
    result.reserve(domain.size() / 4);
    for (int index : domain) {
        const auto &point = points[std::size_t(index)].xyz;
        if (finite(point) && std::abs(plane.normal.dot(point) + plane.d) <= threshold)
            result.push_back(index);
    }
    return result;
}

PlaneCandidate detectLocalReference(const std::vector<Point> &points,
                                    const std::vector<int> &domain,
                                    const Config &config, const Plane &expected,
                                    int segmentIndex) {
    PlaneCandidate best;
    if (domain.size() < 3)
        return best;
    const std::size_t maximumSample = std::max<std::size_t>(
        500, std::size_t(config.maxRansacPoints) /
                 std::max<std::size_t>(1, std::size_t(std::ceil(
                     double(points.size()) / double(domain.size())))));
    const std::size_t stride = std::max<std::size_t>(1, domain.size() / maximumSample);
    std::vector<int> sample;
    sample.reserve(std::min(domain.size(), maximumSample));
    for (std::size_t i = 0; i < domain.size(); i += stride)
        sample.push_back(domain[i]);

    const int iterations = std::max(100, config.planeIterations / 2);
    // expected 来自全局 A 面种子或已经建立的相邻局部 A 面。超过连续性上限的
    // 候选即使内点更多，也不能作为下一段 A 面；否则位于工件首尾的干扰平面
    // 没有两侧邻段可供事后识别，会直接形成错误的 A1 或 An。搜索角至少保留
    // 5 度，使非常严格的质量阈值仍能在后续连续性检查中报告真实变化。
    const double maximumSearchAngleDegrees =
        std::max(5.0, config.maxAdjacentNormalAngleDegrees);
    const double minimumAlignment =
        std::cos(maximumSearchAngleDegrees / kRadiansToDegrees);
    double bestScore = 0.0;
    int bestIteration = std::numeric_limits<int>::max();
    Plane ransacBest = expected;
#ifdef BWB_HAS_OPENMP
#pragma omp parallel
#endif
    {
        double localScore = 0.0;
        int localIteration = std::numeric_limits<int>::max();
        Plane localBest = expected;
#ifdef BWB_HAS_OPENMP
#pragma omp for schedule(static)
#endif
        for (int iteration = 0; iteration < iterations; ++iteration) {
            const std::uint64_t seed =
                std::uint64_t(segmentIndex + 1) * 0x51A7E2026ULL +
                std::uint64_t(iteration);
            const auto pick = [&](std::uint64_t salt) {
                return std::size_t(mix(seed + salt) % sample.size());
            };
            const Vec3 a = points[std::size_t(sample[pick(0)])].xyz;
            const Vec3 b = points[std::size_t(sample[pick(1)])].xyz;
            const Vec3 c = points[std::size_t(sample[pick(2)])].xyz;
            Vec3 normal = (b - a).cross(c - a);
            const double norm = normal.norm();
            if (norm <= kEpsilon)
                continue;
            normal /= norm;
            double alignment = normal.dot(expected.normal);
            if (alignment < 0.0) {
                normal = -normal;
                alignment = -alignment;
            }
            if (alignment < minimumAlignment)
                continue;
            const double d = -normal.dot(a);
            int count = 0;
            int positive = 0, negative = 0;
            const double exclusion = config.planeDistance * 1.5;
            for (int index : sample) {
                const double distance =
                    normal.dot(points[std::size_t(index)].xyz) + d;
                if (std::abs(distance) <= config.planeDistance)
                    ++count;
                positive += distance > exclusion ? 1 : 0;
                negative += distance < -exclusion ? 1 : 0;
            }
            const double balance = positive + negative == 0
                                       ? 0.0
                                       : 2.0 * double(std::min(positive, negative)) /
                                             double(positive + negative);
            // 局部 A 面也必须延续全局 A 面“把工件分到两侧”的语义。
            // 仅按内点数会把 121.txt 前部更密集的外表面误选为 A1/A2。
            const double score = double(count) * alignment * alignment *
                                 (0.10 + balance * balance);
            if (score > localScore ||
                (score == localScore && iteration < localIteration)) {
                localScore = score;
                localIteration = iteration;
                localBest = {normal, a, d};
            }
        }
#ifdef BWB_HAS_OPENMP
#pragma omp critical
#endif
        {
            if (localScore > bestScore ||
                (localScore == bestScore && localIteration < bestIteration)) {
                bestScore = localScore;
                bestIteration = localIteration;
                ransacBest = localBest;
            }
        }
    }
    // 随机三点不保证命中已知 A 面，尤其是端部宽度较小或采样顺序偏置时。
    // 将传播来的 A 面作为显式候选，在本段实测点上收敛，再与随机候选使用
    // 同一全量点评分比较。expected 只是初值，输出仍是本段独立拟合的平面。
    auto evaluate = [&](Plane candidate, double initialThreshold) {
        for (int refinement = 0; refinement < 5; ++refinement) {
            const auto support = domainInliers(points, domain, candidate,
                refinement == 0 ? initialThreshold : config.planeDistance);
            if (support.size() < 3)
                return;
            candidate = fitPlane(points, support, candidate);
        }
        const double alignment = candidate.normal.dot(expected.normal);
        if (alignment < minimumAlignment)
            return;
        auto full = domainInliers(points, domain, candidate, config.planeDistance);
        std::size_t positive = 0, negative = 0, shoulder = 0;
        for (int index : domain) {
            const double distance = candidate.normal.dot(points[std::size_t(index)].xyz) +
                                    candidate.d;
            positive += distance > config.planeDistance * 1.5 ? 1u : 0u;
            negative += distance < -config.planeDistance * 1.5 ? 1u : 0u;
            if (std::abs(distance) > config.planeDistance &&
                std::abs(distance) <= config.planeDistance * 3.0)
                ++shoulder;
        }
        const double balance = positive + negative == 0 ? 0.0 :
            2.0 * double(std::min(positive, negative)) / double(positive + negative);
        const double peakSupport = std::max(0.0, double(full.size()) - 0.5 * double(shoulder));
        const double score = peakSupport * alignment * alignment *
                             (0.10 + balance * balance);
        if (score > best.score) {
            best.plane = candidate;
            best.fullInliers = std::move(full);
            best.score = score;
        }
    };
    if (bestScore > 0.0)
        evaluate(ransacBest, config.planeDistance);
    evaluate(expected, config.planeDistance * 2.0);
    return best;
}

ReferenceModel buildReferenceModel(const std::vector<Point> &points,
                                   const PlaneCandidate &seed,
                                   const Config &config, const Hooks &hooks) {
    ReferenceModel model;
    model.origin = seed.plane.origin;
    model.depth = seed.plane.normal;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    Vec3 mean = Vec3::Zero();
    for (int index : seed.fullInliers)
        mean += points[std::size_t(index)].xyz;
    if (!seed.fullInliers.empty())
        mean /= double(seed.fullInliers.size());
    for (int index : seed.fullInliers) {
        const Vec3 delta = points[std::size_t(index)].xyz - mean;
        covariance.noalias() += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    Vec3 axis = solver.info() == Eigen::Success ? solver.eigenvectors().col(2)
                                                : planeBasis(seed.plane.normal).first;
    axis -= seed.plane.normal * seed.plane.normal.dot(axis);
    if (axis.squaredNorm() <= kEpsilon)
        axis = planeBasis(seed.plane.normal).first;
    model.longitudinal = axis.normalized();
    Eigen::Index dominant = 0;
    model.longitudinal.cwiseAbs().maxCoeff(&dominant);
    if (model.longitudinal[dominant] < 0.0)
        model.longitudinal = -model.longitudinal;
    model.transverse = model.depth.cross(model.longitudinal).normalized();

    std::vector<double> coordinates;
    coordinates.reserve(points.size());
    for (const auto &point : points)
        if (finite(point.xyz))
            coordinates.push_back(longitudinalCoordinate(model, point.xyz));
    if (coordinates.empty())
        return model;
    const double low = quantile(coordinates, 0.005);
    const double high = quantile(coordinates, 0.995);
    const double totalLength = std::max(kEpsilon, high - low);
    const int count = std::max(1, std::min(
        128, int(std::ceil(totalLength / config.referenceSegmentLength))));
    const double width = totalLength / double(count);
    const int seedIndex = std::max(0, std::min(
        count - 1, int(std::floor((longitudinalCoordinate(model, seed.plane.origin) - low) /
                                  width))));
    model.segments.resize(std::size_t(count));
    std::vector<unsigned char> ready(std::size_t(count), 0);
    const double overlap = std::min(width * 0.15, config.referenceSegmentLength * 0.1);

    auto fitSegment = [&](int index, const Plane &expected) {
        ReferenceSegment segment;
        segment.longitudinalMin = low + width * index;
        segment.longitudinalMax = index + 1 == count ? high : low + width * (index + 1);
        std::vector<int> domain;
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (!finite(points[i].xyz))
                continue;
            const double coordinate = longitudinalCoordinate(model, points[i].xyz);
            if (coordinate >= segment.longitudinalMin - overlap &&
                coordinate <= segment.longitudinalMax + overlap)
                domain.push_back(int(i));
        }
        const auto detected = detectLocalReference(points, domain, config, expected, index);
        const std::size_t localMinimum = std::size_t(std::max(
            20, std::min(config.minPlanePoints, int(domain.size() / 20))));
        segment.locallyFitted = detected.fullInliers.size() >= localMinimum;
        segment.plane = segment.locallyFitted ? detected.plane : expected;
        segment.localLongitudinal =
            model.longitudinal - segment.plane.normal *
                                     segment.plane.normal.dot(model.longitudinal);
        if (segment.localLongitudinal.squaredNorm() <= kEpsilon)
            segment.localLongitudinal = planeBasis(segment.plane.normal).first;
        segment.localLongitudinal.normalize();
        if (segment.localLongitudinal.dot(model.longitudinal) < 0.0)
            segment.localLongitudinal = -segment.localLongitudinal;
        segment.localTransverse =
            segment.plane.normal.cross(segment.localLongitudinal).normalized();
        for (int pointIndex : domain) {
            const auto &point = points[std::size_t(pointIndex)].xyz;
            const double coordinate = longitudinalCoordinate(model, point);
            const bool inCore = coordinate >= segment.longitudinalMin &&
                                (index + 1 == count
                                     ? coordinate <= segment.longitudinalMax
                                     : coordinate < segment.longitudinalMax);
            if (inCore && std::abs(segment.plane.normal.dot(point) + segment.plane.d) <=
                              config.planeDistance)
                segment.indices.push_back(pointIndex);
        }
        if (!segment.indices.empty()) {
            double squaredError = 0.0;
            for (int pointIndex : segment.indices) {
                const double distance = segment.plane.normal.dot(
                                            points[std::size_t(pointIndex)].xyz) +
                                        segment.plane.d;
                squaredError += distance * distance;
            }
            segment.planeRms =
                std::sqrt(squaredError / double(segment.indices.size()));
        }
        model.segments[std::size_t(index)] = std::move(segment);
        ready[std::size_t(index)] = 1;
    };

    fitSegment(seedIndex, seed.plane);
    for (int index = seedIndex + 1; index < count && !stopped(hooks); ++index)
        fitSegment(index, model.segments[std::size_t(index - 1)].plane);
    for (int index = seedIndex - 1; index >= 0 && !stopped(hooks); --index)
        fitSegment(index, model.segments[std::size_t(index + 1)].plane);
    for (int index = 0; index < count; ++index)
        if (!ready[std::size_t(index)])
            fitSegment(index, seed.plane);

    auto guidedRefit = [&](int index, const Plane &predicted) {
        auto &current = model.segments[std::size_t(index)];
        std::vector<int> domain;
        for (std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex) {
            if (!finite(points[pointIndex].xyz))
                continue;
            const double coordinate =
                longitudinalCoordinate(model, points[pointIndex].xyz);
            if (coordinate >= current.longitudinalMin - overlap &&
                coordinate <= current.longitudinalMax + overlap)
                domain.push_back(int(pointIndex));
        }
        const std::size_t localMinimum = std::size_t(std::max(
            20, std::min(config.minPlanePoints, int(domain.size() / 20))));
        auto support = domainInliers(points, domain, predicted,
                                     config.planeDistance * 2.0);
        if (support.size() < localMinimum)
            return false;
        Plane refined = fitPlane(points, support, predicted);
        if (refined.normal.dot(predicted.normal) < 0.0) {
            refined.normal = -refined.normal;
            refined.d = -refined.d;
        }
        support = domainInliers(points, domain, refined, config.planeDistance);
        if (support.size() < localMinimum)
            return false;
        refined = fitPlane(points, support, refined);
        if (refined.normal.dot(predicted.normal) < 0.0) {
            refined.normal = -refined.normal;
            refined.d = -refined.d;
        }

        ReferenceSegment candidate = current;
        candidate.plane = refined;
        candidate.locallyFitted = true;
        candidate.indices.clear();
        candidate.localLongitudinal =
            model.longitudinal - refined.normal * refined.normal.dot(model.longitudinal);
        if (candidate.localLongitudinal.squaredNorm() <= kEpsilon)
            return false;
        candidate.localLongitudinal.normalize();
        if (candidate.localLongitudinal.dot(model.longitudinal) < 0.0)
            candidate.localLongitudinal = -candidate.localLongitudinal;
        candidate.localTransverse =
            refined.normal.cross(candidate.localLongitudinal).normalized();
        double squaredError = 0.0;
        for (int pointIndex : domain) {
            const Vec3 &point = points[std::size_t(pointIndex)].xyz;
            const double coordinate = longitudinalCoordinate(model, point);
            const bool inCore = coordinate >= candidate.longitudinalMin &&
                                (index + 1 == count
                                     ? coordinate <= candidate.longitudinalMax
                                     : coordinate < candidate.longitudinalMax);
            const double distance = refined.normal.dot(point) + refined.d;
            if (inCore && std::abs(distance) <= config.planeDistance) {
                candidate.indices.push_back(pointIndex);
                squaredError += distance * distance;
            }
        }
        if (candidate.indices.size() < localMinimum)
            return false;
        candidate.planeRms =
            std::sqrt(squaredError / double(candidate.indices.size()));

        const double oldPredictionError =
            normalAngleDegrees(current.plane.normal, predicted.normal);
        const double newPredictionError =
            normalAngleDegrees(candidate.plane.normal, predicted.normal);
        if (newPredictionError + 0.25 >= oldPredictionError)
            return false;
        current = std::move(candidate);
        return true;
    };

    // 识别“跳到另一平面若干段后又跳回来”的连续错面块。单点异常检查无法
    // 发现两个相邻区段同时选到同一个干扰平面，而这种模式在真实 121.txt 和
    // 局部密集平面合成回归中都会出现。
    const double jumpAngle =
        std::min(5.0, config.maxAdjacentNormalAngleDegrees / 3.0);
    std::vector<int> jumps;
    for (int index = 1; index < count; ++index)
        if (normalAngleDegrees(
                model.segments[std::size_t(index - 1)].plane.normal,
                model.segments[std::size_t(index)].plane.normal) > jumpAngle)
            jumps.push_back(index);
    for (std::size_t jump = 1; jump < jumps.size(); ++jump) {
        const int first = jumps[jump - 1];
        const int afterLast = jumps[jump];
        const auto &left = model.segments[std::size_t(first - 1)];
        const auto &right = model.segments[std::size_t(afterLast)];
        if (normalAngleDegrees(left.plane.normal, right.plane.normal) > jumpAngle)
            continue;
        for (int index = first; index < afterLast; ++index) {
            const double ratio = double(index - first + 1) /
                                 double(afterLast - first + 1);
            Plane predicted;
            predicted.normal =
                ((1.0 - ratio) * left.plane.normal + ratio * right.plane.normal)
                    .normalized();
            predicted.origin =
                (1.0 - ratio) * left.plane.origin + ratio * right.plane.origin;
            predicted.d = -predicted.normal.dot(predicted.origin);
            guidedRefit(index, predicted);
        }
    }

    // 局部点数最多的平面不一定是 A 面。若一个内段同时偏离左右邻段，而左右
    // 邻段彼此一致，并且该段支持量或 RMS 也异常，则用两侧预测面引导一次独立
    // 局部重拟合。重拟合仍只使用本长度区段附近的实测点，不复制邻段平面。
    for (int index = 1; index + 1 < count; ++index) {
        const auto &left = model.segments[std::size_t(index - 1)];
        const auto &middle = model.segments[std::size_t(index)];
        const auto &right = model.segments[std::size_t(index + 1)];
        const double neighborAngle =
            normalAngleDegrees(left.plane.normal, right.plane.normal);
        const double middleAngle = std::max(
            normalAngleDegrees(left.plane.normal, middle.plane.normal),
            normalAngleDegrees(middle.plane.normal, right.plane.normal));
        const bool isolatedDirection =
            neighborAngle <= std::min(5.0, config.maxAdjacentNormalAngleDegrees / 3.0) &&
            middleAngle > std::max(2.0, neighborAngle * 3.0 + 0.5);
        const bool weakSupport = middle.indices.size() * 4 <
                                 std::min(left.indices.size(), right.indices.size()) * 3;
        const bool highResidual =
            middle.planeRms > std::max(left.planeRms, right.planeRms) * 1.5;
        if (!isolatedDirection || (!weakSupport && !highResidual))
            continue;
        Plane predicted;
        predicted.normal = (left.plane.normal + right.plane.normal).normalized();
        predicted.origin = 0.5 * (left.plane.origin + right.plane.origin);
        predicted.d = -predicted.normal.dot(predicted.origin);
        guidedRefit(index, predicted);
    }
    for (std::size_t index = 1; index < model.segments.size(); ++index) {
        auto &segment = model.segments[index];
        segment.normalChangeFromPreviousDegrees =
            normalAngleDegrees(model.segments[index - 1].plane.normal,
                               segment.plane.normal);
        const auto &previous = model.segments[index - 1].plane;
        const double leftDenominator = previous.normal.dot(model.depth);
        const double rightDenominator = segment.plane.normal.dot(model.depth);
        if (std::abs(leftDenominator) <= 1e-8 ||
            std::abs(rightDenominator) <= 1e-8)
            continue;
        std::vector<double> transverse;
        const double band = std::max(config.planeDistance * 3.0, width * 0.02);
        for (const auto &point : points) {
            if (finite(point.xyz) &&
                std::abs(longitudinalCoordinate(model, point.xyz) -
                         segment.longitudinalMin) <= band)
                transverse.push_back(model.transverse.dot(point.xyz - model.origin));
        }
        if (transverse.empty())
            continue;
        for (double q : {0.05, 0.50, 0.95}) {
            const Vec3 anchor = model.origin +
                model.longitudinal * segment.longitudinalMin +
                model.transverse * quantile(transverse, q);
            const double left = -(previous.normal.dot(anchor) + previous.d) /
                                leftDenominator;
            const double right = -(segment.plane.normal.dot(anchor) + segment.plane.d) /
                                 rightDenominator;
            segment.boundaryGapFromPrevious =
                std::max(segment.boundaryGapFromPrevious, std::abs(left - right));
        }
        segment.boundaryGapMeasured = true;
    }
    return model;
}

struct Cell {
    int x = 0, y = 0;
    bool operator==(const Cell &other) const { return x == other.x && y == other.y; }
};

struct CellHash {
    std::size_t operator()(const Cell &value) const noexcept {
        return (std::size_t(std::uint32_t(value.x)) << 32) ^
               std::size_t(std::uint32_t(value.y) * 0x9e3779b9u);
    }
};

SliceEvaluation evaluateSlice(const std::vector<Point> &points,
                              const ReferenceModel &reference,
                              const Config &config, int side, double offset) {
    SliceEvaluation result;
    result.side = side;
    result.offset = offset;
    const double half = config.sliceThickness * 0.5;
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (!finite(points[i].xyz))
            continue;
        const double signedDistance = signedReferenceDistance(reference, points[i].xyz);
        if (std::abs(double(side) * signedDistance - offset) <= half)
            result.raw.push_back(int(i));
    }
    if (result.raw.size() < std::size_t(std::max(8, config.minNeighbors + 2)))
        return result;

    const double cellSize = std::max(config.denoiseRadius, kEpsilon);
    std::vector<Eigen::Vector2d> xy(result.raw.size());
    std::vector<Cell> cells(result.raw.size());
    std::unordered_map<Cell, std::vector<int>, CellHash> grid;
    grid.reserve(result.raw.size() * 2);
    for (std::size_t i = 0; i < result.raw.size(); ++i) {
        xy[i] = flattenReference(reference,
                                 points[std::size_t(result.raw[i])].xyz);
        cells[i] = {int(std::floor(xy[i].x() / cellSize)),
                    int(std::floor(xy[i].y() / cellSize))};
        grid[cells[i]].push_back(int(i));
    }
    std::vector<unsigned char> keep(result.raw.size(), 0);
    const double radiusSquared = config.denoiseRadius * config.denoiseRadius;
#ifdef BWB_HAS_OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (long long i = 0; i < static_cast<long long>(result.raw.size()); ++i) {
        int neighbors = 0;
        for (int dx = -1; dx <= 1 && neighbors < config.minNeighbors; ++dx)
            for (int dy = -1; dy <= 1 && neighbors < config.minNeighbors; ++dy) {
                const auto found = grid.find({cells[std::size_t(i)].x + dx,
                                              cells[std::size_t(i)].y + dy});
                if (found == grid.end())
                    continue;
                for (int other : found->second) {
                    if (other != i &&
                        (xy[std::size_t(other)] - xy[std::size_t(i)]).squaredNorm() <=
                            radiusSquared)
                        ++neighbors;
                    if (neighbors >= config.minNeighbors)
                        break;
                }
            }
        keep[std::size_t(i)] = neighbors >= config.minNeighbors ? 1 : 0;
    }
    std::vector<Eigen::Vector2d> filtered2d;
    filtered2d.reserve(result.raw.size());
    for (std::size_t i = 0; i < result.raw.size(); ++i)
        if (keep[i]) {
            result.filtered.push_back(result.raw[i]);
            filtered2d.push_back(xy[i]);
        }
    if (filtered2d.size() < 8)
        return result;

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto &point : filtered2d)
        mean += point;
    mean /= double(filtered2d.size());
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto &point : filtered2d) {
        const auto delta = point - mean;
        covariance.noalias() += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return result;
    const Eigen::Vector2d axis = solver.eigenvectors().col(1).normalized();
    const Eigen::Vector2d cross(-axis.y(), axis.x());
    std::vector<double> longitudinal, residual;
    longitudinal.reserve(filtered2d.size());
    residual.reserve(filtered2d.size());
    for (const auto &point : filtered2d) {
        const auto delta = point - mean;
        longitudinal.push_back(axis.dot(delta));
        residual.push_back(std::abs(cross.dot(delta)));
    }
    const double low = quantile(longitudinal, 0.01);
    const double high = quantile(longitudinal, 0.99);
    result.length = std::max(0.0, high - low);
    result.deviation = quantile(residual, 0.90);
    const auto eigenvalues = solver.eigenvalues();
    result.linearityRatio = eigenvalues.sum() <= kEpsilon
                                ? 0.0
                                : eigenvalues[0] / eigenvalues.sum();
    constexpr int bins = 100;
    std::array<unsigned char, bins> occupied{};
    if (result.length > kEpsilon)
        for (double value : longitudinal) {
            const int bin = std::max(0, std::min(bins - 1,
                int(std::floor((value - low) / result.length * bins))));
            if (value >= low && value <= high)
                occupied[std::size_t(bin)] = 1;
        }
    const int occupiedCount = std::accumulate(occupied.begin(), occupied.end(), 0);
    int longestRun = 0, run = 0;
    for (unsigned char value : occupied) {
        run = value ? run + 1 : 0;
        longestRun = std::max(longestRun, run);
    }
    result.completeness = 0.5 * double(occupiedCount) / bins +
                          0.5 * double(longestRun) / bins;
    result.bent = result.deviation >= config.bendDeviation &&
                  result.linearityRatio >= 0.0003;
    result.score = result.length * (0.25 + 0.75 * result.completeness) *
                   std::log1p(double(result.filtered.size()));
    return result;
}

double pointSegmentDistance(const Vec3 &point, const Vec3 &start, const Vec3 &end) {
    const Vec3 segment = end - start;
    const double lengthSquared = segment.squaredNorm();
    if (lengthSquared <= kEpsilon)
        return (point - start).norm();
    const double ratio = std::max(0.0, std::min(1.0,
        (point - start).dot(segment) / lengthSquared));
    return (point - (start + ratio * segment)).norm();
}

std::vector<std::size_t> simplifyPolyline(const std::vector<ProjectedPoint> &points,
                                          double tolerance) {
    if (points.size() <= 2) {
        std::vector<std::size_t> indices(points.size());
        std::iota(indices.begin(), indices.end(), std::size_t(0));
        return indices;
    }
    std::vector<unsigned char> keep(points.size(), 0);
    keep.front() = keep.back() = 1;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, points.size() - 1}};
    while (!stack.empty()) {
        const auto range = stack.back();
        stack.pop_back();
        double maximum = -1.0;
        std::size_t split = range.first;
        for (std::size_t i = range.first + 1; i < range.second; ++i) {
            const double distance = pointSegmentDistance(
                points[i].xyz, points[range.first].xyz, points[range.second].xyz);
            if (distance > maximum) {
                maximum = distance;
                split = i;
            }
        }
        if (maximum > tolerance && split > range.first && split < range.second) {
            keep[split] = 1;
            stack.push_back({range.first, split});
            stack.push_back({split, range.second});
        }
    }
    std::vector<std::size_t> indices;
    indices.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
        if (keep[i])
            indices.push_back(i);
    return indices;
}

std::vector<std::size_t> slopePhaseAnchors(
    const std::vector<ProjectedPoint> &points, double tolerance) {
    if (points.size() < 6)
        return simplifyPolyline(points, tolerance);

    Vec3 mean = Vec3::Zero();
    for (const auto &point : points)
        mean += point.xyz;
    mean /= double(points.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &point : points) {
        const Vec3 delta = point.xyz - mean;
        covariance.noalias() += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return simplifyPolyline(points, tolerance);
    Vec3 longitudinal = solver.eigenvectors().col(2).normalized();
    if (longitudinal.dot(points.back().xyz - points.front().xyz) < 0.0)
        longitudinal = -longitudinal;
    Vec3 transverse = solver.eigenvectors().col(1).normalized();

    std::vector<double> s(points.size()), t(points.size()), spacing;
    spacing.reserve(points.size() - 1);
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Vec3 delta = points[i].xyz - mean;
        s[i] = longitudinal.dot(delta);
        t[i] = transverse.dot(delta);
        if (i > 0)
            spacing.push_back((points[i].xyz - points[i - 1].xyz).norm());
    }
    const double medianSpacing = std::max(quantile(spacing, 0.5), 1e-6);
    const int radius = std::max(2, std::min(
        8, int(std::ceil(tolerance / medianSpacing)) + 1));
    // 容差越大，越倾向于把小幅斜率视为同一水平段；真实明显坡段仍按正负号分开。
    const double horizontalSlope = std::max(
        0.04, std::min(0.18, tolerance / (medianSpacing * 10.0)));
    std::vector<int> states(points.size(), 0);
    for (std::size_t i = 0; i < points.size(); ++i) {
        const std::size_t first = i > std::size_t(radius) ? i - radius : 0;
        const std::size_t last = std::min(points.size() - 1, i + std::size_t(radius));
        const double ds = s[last] - s[first];
        const double slope = std::abs(ds) <= 1e-9
                                 ? std::copysign(std::numeric_limits<double>::infinity(),
                                                 t[last] - t[first])
                                 : (t[last] - t[first]) / ds;
        states[i] = slope > horizontalSlope ? 1
                   : slope < -horizontalSlope ? -1
                                              : 0;
    }

    // 三点多数表决抑制单点斜率翻转。
    auto filteredStates = states;
    for (std::size_t i = 1; i + 1 < states.size(); ++i) {
        const int sum = states[i - 1] + states[i] + states[i + 1];
        filteredStates[i] = sum > 0 ? 1 : sum < 0 ? -1 : 0;
    }
    states.swap(filteredStates);

    struct Run { std::size_t first = 0, last = 0; int state = 0; };
    const auto runsOf = [&](const std::vector<int> &values) {
        std::vector<Run> runs;
        std::size_t first = 0;
        for (std::size_t i = 1; i <= values.size(); ++i) {
            if (i == values.size() || values[i] != values[first]) {
                runs.push_back({first, i - 1, values[first]});
                first = i;
            }
        }
        return runs;
    };
    const std::size_t minimumRun = std::size_t(std::max(4, radius * 2));
    for (int pass = 0; pass < 8; ++pass) {
        const auto runs = runsOf(states);
        bool changed = false;
        for (std::size_t i = 0; i < runs.size(); ++i) {
            const std::size_t length = runs[i].last - runs[i].first + 1;
            if (length >= minimumRun || runs.size() == 1)
                continue;
            int replacement = runs[i].state;
            if (i > 0 && i + 1 < runs.size() &&
                runs[i - 1].state == runs[i + 1].state) {
                replacement = runs[i - 1].state;
            } else if (i == 0) {
                replacement = runs[i + 1].state;
            } else if (i + 1 == runs.size()) {
                replacement = runs[i - 1].state;
            } else {
                const std::size_t leftLength =
                    runs[i - 1].last - runs[i - 1].first + 1;
                const std::size_t rightLength =
                    runs[i + 1].last - runs[i + 1].first + 1;
                replacement = leftLength >= rightLength ? runs[i - 1].state
                                                        : runs[i + 1].state;
            }
            std::fill(states.begin() + std::ptrdiff_t(runs[i].first),
                      states.begin() + std::ptrdiff_t(runs[i].last + 1), replacement);
            changed = true;
            break;
        }
        if (!changed)
            break;
    }

    const auto runs = runsOf(states);
    std::vector<std::size_t> anchors{0};
    for (std::size_t i = 1; i < runs.size(); ++i) {
        const std::size_t right = runs[i].first;
        const std::size_t left = right - 1;
        // 状态转换位于相邻两个采样点之间。长度轴反向时“右侧第一个点”会换成
        // 原来的左点，因此用稳定来源索引选择同一个物理边界点。
        anchors.push_back(points[left].source <= points[right].source ? left : right);
    }
    anchors.push_back(points.size() - 1);
    anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
    return anchors.size() >= 2 ? anchors : simplifyPolyline(points, tolerance);
}

struct FittedPolyline {
    std::vector<std::size_t> anchors;
    std::vector<FittedLineModel> segments;
    std::vector<ProjectedPoint> corners;
    std::string failure;
};

bool consensusLine(const std::vector<ProjectedPoint> &points, std::size_t first,
                   std::size_t last, double tolerance, FittedLineModel &line) {
    line.first = first;
    line.last = last;
    if (last <= first) return false;
    // Deterministic bounded pair search, ranked by support count, then inlier SSE.
    // Source order makes candidate selection independent of traversal direction.
    std::vector<std::size_t> candidates;
    for (std::size_t i = first; i <= last; ++i) candidates.push_back(i);
    std::sort(candidates.begin(), candidates.end(), [&](std::size_t a, std::size_t b) {
        return points[a].source < points[b].source;
    });
    if (candidates.size() > 64) {
        std::vector<std::size_t> sampled;
        for (std::size_t i = 0; i < 64; ++i)
            sampled.push_back(candidates[i*(candidates.size()-1)/63]);
        candidates.swap(sampled);
    }
    std::vector<std::size_t> bestInliers;
    double bestError = std::numeric_limits<double>::infinity();
    const auto classify = [&](const Vec3 &origin, const Vec3 &direction, double &error) {
        std::vector<std::size_t> inliers;
        error = 0.0;
        for (std::size_t i = first; i <= last; ++i) {
            const Vec3 d = points[i].xyz-origin;
            const double residual = (d-direction*direction.dot(d)).squaredNorm();
            if (residual <= tolerance*tolerance) {
                inliers.push_back(i);
                error += residual;
            }
        }
        return inliers;
    };
    for (std::size_t a = 0; a < candidates.size(); ++a) {
        for (std::size_t b = a+1; b < candidates.size(); ++b) {
            const Vec3 origin = points[candidates[a]].xyz;
            const Vec3 delta = points[candidates[b]].xyz-origin;
            if (delta.squaredNorm() < 1e-12) continue;
            double error;
            auto inliers = classify(origin, delta.normalized(), error);
            if (inliers.size() > bestInliers.size() ||
                (inliers.size() == bestInliers.size() && error < bestError-1e-9)) {
                bestInliers = std::move(inliers);
                bestError = error;
            }
        }
    }
    if (bestInliers.size() < 2 || bestInliers.size()*2 <= last-first+1) return false;
    // Orthogonal least squares over all consensus support, never just two endpoints.
    for (int pass = 0; pass < 5; ++pass) {
        const Vec3 center = points[first].xyz;
        Vec3 mean = Vec3::Zero();
        for (auto i : bestInliers) mean += points[i].xyz-center;
        mean /= double(bestInliers.size());
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        for (auto i : bestInliers) {
            const Vec3 d = points[i].xyz-center-mean;
            covariance.noalias() += d*d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
        if (solver.info() != Eigen::Success || solver.eigenvalues()[2] < 1e-12) return false;
        line.origin = center+mean;
        line.direction = solver.eigenvectors().col(2).normalized();
        if (line.direction.dot(points[last].xyz-points[first].xyz) < 0)
            line.direction = -line.direction;
        double error;
        auto inliers = classify(line.origin, line.direction, error);
        line.support = inliers.size();
        line.rms = std::sqrt(error / std::max(std::size_t(1), inliers.size()));
        if (inliers == bestInliers || pass == 4) break;
        if (inliers.size() < 2 || inliers.size()*2 <= last-first+1) return false;
        bestInliers = std::move(inliers);
    }
    return line.support >= 2 && line.support*2 > last-first+1;
}

void refineSlopeTransitions(const std::vector<ProjectedPoint> &points,
                            std::vector<std::size_t> &anchors) {
    if (anchors.size() < 3) return;
    // Keep the phase count fixed. Only locate each transition more precisely;
    // a bent continuous uphill/downhill phase must never create extra segments.
    const auto coarse = anchors;
    for (std::size_t phase = 1; phase + 1 < coarse.size(); ++phase) {
        // Midpoint windows can exclude the real turn on a short terminal slope.
        // Fit across both adjacent phases, but keep the resulting boundary in
        // its own non-overlapping coarse-phase cell.
        const std::size_t first = coarse[phase-1];
        const std::size_t last = coarse[phase+1];
        if (last <= first + 6) continue;
        // Local centering avoids loss of precision after large world translations.
        std::vector<Vec3> sums(last-first+2, Vec3::Zero());
        std::vector<Eigen::Matrix3d> products(last-first+2, Eigen::Matrix3d::Zero());
        for (std::size_t i = first; i <= last; ++i) {
            const Vec3 p = points[i].xyz - points[first].xyz;
            const auto j = i-first+1;
            sums[j] = sums[j-1] + p;
            products[j] = products[j-1] + p*p.transpose();
        }
        const auto lineError = [&](std::size_t lo, std::size_t hi) {
            const Vec3 sum = sums[hi-first+1] - sums[lo-first];
            const Eigen::Matrix3d scatter = products[hi-first+1] - products[lo-first] -
                sum*sum.transpose()/double(hi-lo+1);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter, Eigen::EigenvaluesOnly);
            if (solver.info() != Eigen::Success) return std::numeric_limits<double>::infinity();
            return std::max(0.0, solver.eigenvalues()[0] + solver.eigenvalues()[1]);
        };
        double bestError = std::numeric_limits<double>::infinity();
        std::size_t best = coarse[phase];
        const std::size_t searchFirst = phase == 1 ? first+3 :
            std::max(first+3, (first+coarse[phase])/2+1);
        const std::size_t searchLast = phase+2 == coarse.size() ? last-3 :
            std::min(last-3, (coarse[phase]+last)/2);
        for (std::size_t split = searchFirst; split <= searchLast; ++split) {
            const double error = lineError(first, split) + lineError(split, last);
            if (error < bestError-1e-9 ||
                (std::abs(error-bestError) <= 1e-9 && points[split].source < points[best].source)) {
                bestError = error;
                best = split;
            }
        }
        anchors[phase] = best;
    }
}

FittedPolyline fitPolyline(const std::vector<ProjectedPoint> &points,
                           double tolerance, double junctionTolerance) {
    FittedPolyline fitted;
    if (points.size() < 2)
        return fitted;
    fitted.anchors = slopePhaseAnchors(points, tolerance);
    if (fitted.anchors.size() < 2)
        return fitted;
    refineSlopeTransitions(points, fitted.anchors);

    fitted.segments.reserve(fitted.anchors.size() - 1);
    fitted.corners.reserve(fitted.anchors.size());
    for (std::size_t i = 1; i < fitted.anchors.size(); ++i) {
        FittedLineModel line;
        if (!consensusLine(points, fitted.anchors[i-1], fitted.anchors[i], tolerance, line)) {
            fitted.failure = "坡段直线拟合失败：容差内缺少多数有效点，不能仅由端点强行连线。";
            return fitted;
        }
        fitted.segments.push_back(line);
    }
    const auto project = [](const Vec3 &p, const FittedLineModel &line) -> Vec3 {
        return line.origin+line.direction*line.direction.dot(p-line.origin);
    };
    for (std::size_t i = 0; i < fitted.anchors.size(); ++i) {
        auto point = points[fitted.anchors[i]];
        point.corner = true;
        point.lineFitted = true;
        if (i == 0) point.xyz = project(point.xyz, fitted.segments.front());
        else if (i+1 == fitted.anchors.size()) point.xyz = project(point.xyz, fitted.segments.back());
        else {
            const auto &left = fitted.segments[i-1];
            const auto &right = fitted.segments[i];
            const double cosine = left.direction.dot(right.direction);
            const double denominator = 1.0-cosine*cosine;
            // Below 3 degrees, a small fit error can send the intersection arbitrarily far away.
            if (denominator < 0.0027390523158633) {
                fitted.failure = "相邻坡段直线近乎平行（锐角小于3度），无法稳定求拐点。";
                return fitted;
            }
            const Vec3 delta = right.origin-left.origin;
            const double a = (left.direction.dot(delta)-cosine*right.direction.dot(delta))/denominator;
            const double b = (cosine*left.direction.dot(delta)-right.direction.dot(delta))/denominator;
            const Vec3 p = left.origin+a*left.direction;
            const Vec3 q = right.origin+b*right.direction;
            point.lineIntersectionGap = (p-q).norm();
            if (point.lineIntersectionGap > junctionTolerance) {
                fitted.failure = "相邻坡段三维拟合直线不相交，最近距离超过连接容差。";
                return fitted;
            }
            // Exact intersection for coplanar lines; explicitly measured approximation otherwise.
            const Vec3 junction = (p+q)*0.5;
            const double leftSpan = (points[fitted.anchors[i]].xyz-points[fitted.anchors[i-1]].xyz).norm();
            const double rightSpan = (points[fitted.anchors[i+1]].xyz-points[fitted.anchors[i]].xyz).norm();
            if ((junction-point.xyz).norm() > std::max(4.0*tolerance, 0.5*std::min(leftSpan,rightSpan))) {
                fitted.failure = "相邻坡段交点远离局部转换区域，拒绝无限延长形成跳点。";
                return fitted;
            }
            point.xyz = junction;
        }
        fitted.corners.push_back(point);
    }
    for (std::size_t i = 0; i < fitted.segments.size(); ++i) {
        if ((fitted.corners[i+1].xyz-fitted.corners[i].xyz).dot(fitted.segments[i].direction) <= 1e-6) {
            fitted.failure = "相邻交点顺序倒置或坡段退化，不能生成白点轨迹。";
            return fitted;
        }
    }
    return fitted;
}

std::vector<ProjectedPoint> projectMeasuredPoints(
    const std::vector<ProjectedPoint> &measured, const FittedPolyline &line,
    double requestedSpacing) {
    if (line.corners.size() < 2 || line.anchors.size() != line.corners.size())
        return {};
    std::vector<ProjectedPoint> result;
    auto first = line.corners.front();
    first.corner = true;
    result.push_back(first);
    for (std::size_t segment = 0; segment < line.segments.size(); ++segment) {
        const Vec3 start = line.corners[segment].xyz;
        const Vec3 delta = line.corners[segment + 1].xyz - start;
        const double length = delta.norm();
        if (length <= kEpsilon)
            continue;
        const Vec3 direction = delta / length;
        const int intervals = std::max(1, int(std::lround(length / requestedSpacing)));
        std::size_t lastSelected = line.anchors[segment];
        for (int interval = 1; interval < intervals; ++interval) {
            const double target = length * double(interval) / intervals;
            std::size_t best = measured.size();
            double bestError = std::numeric_limits<double>::infinity();
            for (std::size_t i = lastSelected + 1; i < line.anchors[segment + 1]; ++i) {
                const Vec3 projected = start +
                    direction * direction.dot(measured[i].xyz - start);
                const double parameter = direction.dot(projected - start);
                if (parameter <= 0.0 || parameter >= length)
                    continue;
                const double error = std::abs(parameter - target);
                if (error < bestError) {
                    best = i;
                    bestError = error;
                }
            }
            if (best == measured.size())
                continue;
            const Vec3 projected = start +
                direction * direction.dot(measured[best].xyz - start);
            auto point = measured[best];
            point.xyz = projected;
            point.corner = false;
            point.lineFitted = true;
            point.lineIntersectionGap = 0.0;
            result.push_back(point);
            lastSelected = best;
        }
        // 合成交点必须原样进入白点序列。
        result.push_back(line.corners[segment + 1]);
    }
    return result;
}

} // namespace

bool fitTrajectoryLines(const std::vector<ProjectedPoint> &trajectory,
                        const Config &config, Result &result) {
    result.fittedLine.clear();
    result.whitePoints.clear();
    result.lineModels.clear();
    if (trajectory.size() < 2 || !std::isfinite(config.lineFitTolerance) ||
        config.lineFitTolerance <= 0 || !std::isfinite(config.whitePointSpacing) ||
        config.whitePointSpacing <= 0 || !std::isfinite(config.surfaceFitTolerance) ||
        config.surfaceFitTolerance <= 0 ||
        std::any_of(trajectory.begin(), trajectory.end(), [](const ProjectedPoint &p) { return !p.xyz.allFinite(); })) {
        result.message = "直线拟合输入或参数无效。";
        return false;
    }
    const auto fitted = fitPolyline(trajectory, config.lineFitTolerance,
                                   std::min(config.lineFitTolerance, config.surfaceFitTolerance));
    result.lineModels = fitted.segments;
    if (!fitted.failure.empty()) {
        result.message = fitted.failure;
        return false;
    }
    result.fittedLine = fitted.corners;
    result.whitePoints = projectMeasuredPoints(trajectory, fitted, config.whitePointSpacing);
    if (result.fittedLine.size() < 2 || result.whitePoints.size() < 2) {
        result.message = "中心轨迹已生成，但无法拟合折线并平均分配白色点。";
        return false;
    }
    return true;
}

std::string validateConfig(const Config &config) {
    if (!std::isfinite(config.maxSurfaceExtrapolation) || config.maxSurfaceExtrapolation < 0.0 ||
        !std::isfinite(config.surfaceFitTolerance) || config.surfaceFitTolerance <= 0.0 ||
        config.minSurfacePoints < 6 || !std::isfinite(config.minIntersectionAngleDegrees) ||
        config.minIntersectionAngleDegrees <= 0.0 || config.minIntersectionAngleDegrees >= 90.0)
        return "表面求交参数无效：外推长度须非负，拟合容差须为正，支持点不少于6，交会角须在0至90度之间。";
    for (double offset : config.sliceOffsets)
        if (!std::isfinite(offset) || offset <= 0)
            return "切片偏移必须为正数。";
    if (!std::isfinite(config.sliceThickness) || config.sliceThickness <= 0)
        return "切片厚度必须为正数。";
    if (!std::isfinite(config.planeDistance) || config.planeDistance <= 0 ||
        !std::isfinite(config.referenceSegmentLength) ||
        config.referenceSegmentLength <= 0)
        return "平面距离阈值和局部基准面长度必须为正数。";
    if (!std::isfinite(config.maxAdjacentNormalAngleDegrees) ||
        config.maxAdjacentNormalAngleDegrees <= 0.0 ||
        config.maxAdjacentNormalAngleDegrees >= 90.0)
        return "相邻局部基准面最大法向变化角必须在 0 到 90 度之间。";
    if (config.planeIterations < 20 || config.planeCandidates < 1 ||
        config.maxRansacPoints < 100 || config.minPlanePoints < 3)
        return "平面提取参数超出有效范围。";
    if (!std::isfinite(config.denoiseRadius) || config.denoiseRadius <= 0 ||
        config.minNeighbors < 1)
        return "去噪半径和邻点数必须为正数。";
    if (!std::isfinite(config.bendDeviation) || config.bendDeviation <= 0 ||
        !std::isfinite(config.trajectorySpacing) || config.trajectorySpacing <= 0 ||
        !std::isfinite(config.lineFitTolerance) || config.lineFitTolerance <= 0 ||
        !std::isfinite(config.whitePointSpacing) || config.whitePointSpacing <= 0)
        return "弯折阈值、拟合容差和轨迹点间距必须为正数。";
    return {};
}

Result extract(const std::vector<Point> &points, const Config &config, const Hooks &hooks) {
    Result result;
    const auto configError = validateConfig(config);
    if (!configError.empty()) {
        result.message = configError;
        return result;
    }
    if (points.size() < std::size_t(config.minPlanePoints)) {
        result.message = "有效点数不足，无法提取基准面。";
        return result;
    }
#ifdef BWB_HAS_OPENMP
    if (config.threads > 0)
        omp_set_num_threads(config.threads);
#endif
    if (stopped(hooks)) {
        result.cancelled = true;
        return result;
    }
    reportProgress(hooks, 5);
    auto reference = detectReference(points, config, hooks);
    if (stopped(hooks)) {
        result.cancelled = true;
        return result;
    }
    if (reference.fullInliers.size() < std::size_t(config.minPlanePoints)) {
        result.message = "没有找到能把点云分成两侧的稳定基准面 A。";
        return result;
    }
    const auto referenceModel = buildReferenceModel(points, reference, config, hooks);
    if (stopped(hooks)) {
        result.cancelled = true;
        return result;
    }
    if (referenceModel.segments.empty()) {
        result.message = "已找到基准面种子，但无法沿长度方向建立局部基准面。";
        return result;
    }
    result.reference = reference.plane;
    result.referenceOrigin = referenceModel.origin;
    result.referenceLongitudinal = referenceModel.longitudinal;
    result.referenceTransverse = referenceModel.transverse;
    result.referenceSegments = referenceModel.segments;
    for (const auto &segment : result.referenceSegments)
        result.referenceIndices.insert(result.referenceIndices.end(),
                                       segment.indices.begin(), segment.indices.end());
    if (std::any_of(referenceModel.segments.begin(), referenceModel.segments.end(),
                    [](const ReferenceSegment &segment) {
                        return !segment.locallyFitted;
                    })) {
        result.message = "至少一个长度区段缺少足够点数，无法独立建立局部基准面 Aᵢ。";
        return result;
    }
    const auto invalidFrame = std::find_if(
        result.referenceSegments.begin(), result.referenceSegments.end(),
        [](const ReferenceSegment &segment) {
            const Vec3 &x = segment.localLongitudinal;
            const Vec3 &y = segment.localTransverse;
            const Vec3 &z = segment.plane.normal;
            return !finite(x) || !finite(y) || !finite(z) ||
                   std::abs(x.norm() - 1.0) > 1e-8 ||
                   std::abs(y.norm() - 1.0) > 1e-8 ||
                   std::abs(z.norm() - 1.0) > 1e-8 ||
                   std::abs(x.dot(y)) > 1e-8 || std::abs(x.dot(z)) > 1e-8 ||
                   std::abs(y.dot(z)) > 1e-8 || x.cross(y).dot(z) < 1.0 - 1e-8;
        });
    if (invalidFrame != result.referenceSegments.end()) {
        result.message = "局部基准面的长度轴、横向轴和法向无法组成有效右手正交坐标系。";
        return result;
    }
    const auto abrupt = std::find_if(
        result.referenceSegments.begin() + 1, result.referenceSegments.end(),
        [&](const ReferenceSegment &segment) {
            return segment.normalChangeFromPreviousDegrees >
                   config.maxAdjacentNormalAngleDegrees;
        });
    if (abrupt != result.referenceSegments.end()) {
        result.message = "相邻局部基准面法向变化过大，不能用平面分段静默展平该工件。";
        return result;
    }
    if (result.referenceIndices.size() < std::size_t(config.minPlanePoints)) {
        result.message = "局部基准面 A1…An 的有效内点总数不足。";
        return result;
    }
    reportProgress(hooks, 35);
    if (!stage(hooks, 0, result)) {
        result.cancelled = true;
        return result;
    }

    result.slices.resize(6);
#ifdef BWB_HAS_OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
    for (int task = 0; task < 6; ++task) {
        const int side = task < 3 ? 1 : -1;
        const int offsetIndex = task % 3;
        result.slices[std::size_t(task)] = evaluateSlice(
            points, referenceModel, config, side,
            config.sliceOffsets[std::size_t(offsetIndex)]);
    }
    if (stopped(hooks)) {
        result.cancelled = true;
        return result;
    }
    reportProgress(hooks, 65);
    if (!stage(hooks, 1, result)) {
        result.cancelled = true;
        return result;
    }

    // 第 2/3 步：先尝试法向正侧；全为直线时自动转到负侧。两侧都弯折则取证据更完整者。
    int bestPositive = -1, bestNegative = -1;
    for (int i = 0; i < 6; ++i) {
        if (!result.slices[std::size_t(i)].bent)
            continue;
        int &best = i < 3 ? bestPositive : bestNegative;
        if (best < 0 || result.slices[std::size_t(i)].score >
                            result.slices[std::size_t(best)].score)
            best = i;
    }
    if (bestPositive >= 0 && bestNegative >= 0)
        result.selectedSlice = result.slices[std::size_t(bestPositive)].score >=
                                       result.slices[std::size_t(bestNegative)].score
                                   ? bestPositive
                                   : bestNegative;
    else
        result.selectedSlice = bestPositive >= 0 ? bestPositive : bestNegative;
    if (result.selectedSlice < 0) {
        result.message = "基准面两侧的 10/20/30 mm 切片均为长直形态，未找到波纹轨迹。";
        return result;
    }
    result.selectedIndices =
        result.slices[std::size_t(result.selectedSlice)].filtered;
    if (!stage(hooks, 2, result)) {
        result.cancelled = true;
        return result;
    }
    reportProgress(hooks, 78);

    const bool intersected = recoverSurfaceIntersection(points, config, hooks, result);
    if (result.cancelled) return result;
    if (!stage(hooks, 3, result)) {
        result.cancelled = true;
        return result;
    }
    reportProgress(hooks, 88);
    if (!intersected) return result;
    result.trajectory = resampleIntersection(result.projected, config.trajectorySpacing);
    if (result.trajectory.size() < 2) {
        result.message = "特征曲线候选已生成，但无法生成连续均匀轨迹。";
        return result;
    }
    if (!fitTrajectoryLines(result.trajectory, config, result)) return result;
    result.ok = true;
    result.message = result.usedProjectionFallback
        ? "局部表面无法在全部站位稳定求交，已按启用的回退策略将整条 B 特征曲线投影到对应基准面；该结果不是表面交线。"
        : "已由远离边缘的板材主体模型延伸到基准面，完成整段直线拟合和白点采样；边缘区域未参与特征曲线拟合。";
    if (!stage(hooks, 4, result)) {
        result.ok = false;
        result.cancelled = true;
        return result;
    }
    reportProgress(hooks, 100);
    return result;
}

} // namespace bwb
