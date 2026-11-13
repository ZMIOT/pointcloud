#include "SeamSmoother.h"

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

namespace weld {
namespace {

using State = Eigen::Vector2f;
using Covariance = Eigen::Matrix2f;

float clamp01(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

std::array<int, 2> localTransverseAxes(Axis axis) {
  switch (axis) {
  case Axis::X:
    return {{1, 2}};
  case Axis::Y:
    return {{0, 2}};
  case Axis::Z:
    return {{0, 1}};
  }
  return {{1, 2}};
}

std::vector<std::size_t> normalizedStarts(const std::vector<std::size_t> &starts,
                                          std::size_t count) {
  std::vector<std::size_t> result;
  if (count == 0)
    return result;
  result.push_back(0);
  for (std::size_t start : starts)
    if (start > 0 && start < count && start > result.back())
      result.push_back(start);
  return result;
}

float observationVariance(const SeamPointQuality &quality,
                          const DetectorConfig &config) {
  const float confidence = std::max(0.20f, quality.sourceConfidence);
  float deviation = config.rtsMeasurementStddev / confidence;
  if (quality.flags & QualityInterpolated)
    deviation *= 3.0f;
  if (!quality.wallAccepted)
    deviation *= 1.35f;
  if (config.useLocalFloorConstraint && !quality.floorAccepted)
    deviation *= 1.20f;
  return std::max(1.0e-6f, deviation * deviation);
}

void smoothCoordinate(const CloudT &points,
                      const std::vector<SeamPointQuality> &quality,
                      const std::vector<std::size_t> &segmentStarts, int axis,
                      int stationAxis, const DetectorConfig &config,
                      std::vector<float> &smoothed, std::vector<float> &nis,
                      std::vector<float> &sigma) {
  const std::size_t count = points.size();
  smoothed.resize(count);
  nis.assign(count, 0.0f);
  sigma.assign(count, 0.0f);
  for (std::size_t i = 0; i < count; ++i)
    smoothed[i] = points[i].data[axis];

  const auto starts = normalizedStarts(segmentStarts, count);
  for (std::size_t segment = 0; segment < starts.size(); ++segment) {
    const std::size_t begin = starts[segment];
    const std::size_t end = segment + 1 < starts.size() ? starts[segment + 1]
                                                        : count;
    if (end - begin < 3)
      continue;

    const std::size_t length = end - begin;
    std::vector<State> predicted(length), filtered(length), result(length);
    std::vector<Covariance> predictedCovariance(length),
        filteredCovariance(length), resultCovariance(length), transition(length);

    const float firstGap = std::max(
        1.0e-4f, points[begin + 1].data[stationAxis] -
                       points[begin].data[stationAxis]);
    filtered[0] << points[begin].data[axis],
        (points[begin + 1].data[axis] - points[begin].data[axis]) / firstGap;
    filteredCovariance[0] = Covariance::Zero();
    filteredCovariance[0](0, 0) = observationVariance(quality[begin], config);
    filteredCovariance[0](1, 1) =
        4.0f * filteredCovariance[0](0, 0) / (firstGap * firstGap);
    predicted[0] = filtered[0];
    predictedCovariance[0] = filteredCovariance[0];
    transition[0] = Covariance::Identity();

    for (std::size_t local = 1; local < length; ++local) {
      const std::size_t i = begin + local;
      const float ds = std::max(1.0e-4f, points[i].data[stationAxis] -
                                             points[i - 1].data[stationAxis]);
      Covariance f;
      f << 1.0f, ds, 0.0f, 1.0f;
      transition[local] = f;
      const float ds2 = ds * ds;
      Covariance q;
      q << 0.25f * ds2 * ds2, 0.5f * ds2 * ds,
          0.5f * ds2 * ds, ds2;
      q *= std::max(1.0e-8f, config.rtsProcessNoise);

      predicted[local] = f * filtered[local - 1];
      predictedCovariance[local] =
          f * filteredCovariance[local - 1] * f.transpose() + q;

      float variance = observationVariance(quality[i], config);
      const float innovation = points[i].data[axis] - predicted[local][0];
      float innovationVariance = predictedCovariance[local](0, 0) + variance;
      const float rawNis = innovation * innovation /
                           std::max(1.0e-8f, innovationVariance);
      nis[i] = rawNis;
      if (rawNis > config.rtsInnovationGate) {
        const float inflation = std::min(
            100.0f, rawNis / std::max(1.0e-4f, config.rtsInnovationGate));
        variance *= inflation;
        innovationVariance = predictedCovariance[local](0, 0) + variance;
      }

      const State gain = predictedCovariance[local].col(0) /
                         std::max(1.0e-8f, innovationVariance);
      filtered[local] = predicted[local] + gain * innovation;
      Covariance correction = Covariance::Identity();
      correction.col(0) -= gain;
      filteredCovariance[local] =
          correction * predictedCovariance[local] * correction.transpose() +
          variance * gain * gain.transpose();
    }

    result.back() = filtered.back();
    resultCovariance.back() = filteredCovariance.back();
    for (std::size_t local = length - 1; local-- > 0;) {
      const Covariance inverse =
          predictedCovariance[local + 1].ldlt().solve(Covariance::Identity());
      const Covariance gain = filteredCovariance[local] *
                              transition[local + 1].transpose() * inverse;
      result[local] = filtered[local] +
                      gain * (result[local + 1] - predicted[local + 1]);
      resultCovariance[local] =
          filteredCovariance[local] +
          gain * (resultCovariance[local + 1] -
                  predictedCovariance[local + 1]) *
              gain.transpose();
    }
    for (std::size_t local = 0; local < length; ++local) {
      const std::size_t i = begin + local;
      if (result[local].allFinite() && resultCovariance[local].allFinite()) {
        smoothed[i] = result[local][0];
        sigma[i] = std::sqrt(std::max(0.0f, resultCovariance[local](0, 0)));
      }
    }
  }
}

bool satisfiesGeometry(const Eigen::Vector3f &point,
                       const SeamPointQuality &quality,
                       const DetectorConfig &config) {
  const auto axes = localTransverseAxes(config.longitudinalAxis);
  const float tolerance = std::max(1.0e-4f, config.rtsGeometryTolerance);
  if (quality.wallAccepted) {
    const Eigen::Vector2f uv(point[axes[0]], point[axes[1]]);
    if (std::fabs(quality.wallNormal.dot(uv) - quality.wallOffset) > tolerance)
      return false;
  }
  if (quality.floorAccepted &&
      std::fabs(quality.floorNormal.dot(point) - quality.floorOffset) >
          tolerance)
    return false;
  return true;
}

Eigen::Vector3f projectSingleConstraint(Eigen::Vector3f point,
                                        const SeamPointQuality &quality,
                                        const DetectorConfig &config) {
  const auto axes = localTransverseAxes(config.longitudinalAxis);
  if (quality.wallAccepted) {
    Eigen::Vector2f uv(point[axes[0]], point[axes[1]]);
    const float squaredNorm = quality.wallNormal.squaredNorm();
    if (squaredNorm > 1.0e-8f)
      uv -= quality.wallNormal *
            ((quality.wallNormal.dot(uv) - quality.wallOffset) / squaredNorm);
    point[axes[0]] = uv.x();
    point[axes[1]] = uv.y();
  } else if (quality.floorAccepted) {
    Eigen::Vector2f normal(quality.floorNormal[axes[0]],
                           quality.floorNormal[axes[1]]);
    const float squaredNorm = normal.squaredNorm();
    if (squaredNorm > 1.0e-8f) {
      const int stationAxis = static_cast<int>(config.longitudinalAxis);
      Eigen::Vector2f uv(point[axes[0]], point[axes[1]]);
      const float offset = quality.floorOffset -
                           quality.floorNormal[stationAxis] * point[stationAxis];
      uv -= normal * ((normal.dot(uv) - offset) / squaredNorm);
      point[axes[0]] = uv.x();
      point[axes[1]] = uv.y();
    }
  }
  return point;
}

void smoothSide(CloudT &points, std::vector<float> &confidence,
                std::vector<SeamPointQuality> &quality,
                const std::vector<std::size_t> &segmentStarts,
                const DetectorConfig &config) {
  if (points.size() < 3 || quality.size() != points.size())
    return;
  const int stationAxis = static_cast<int>(config.longitudinalAxis);
  const auto axes = localTransverseAxes(config.longitudinalAxis);
  std::array<std::vector<float>, 2> values, nis, sigma;
  for (int component = 0; component < 2; ++component)
    smoothCoordinate(points, quality, segmentStarts, axes[component], stationAxis,
                     config, values[component], nis[component], sigma[component]);

  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3f original = toVector(points[i]);
    Eigen::Vector3f target = original;
    target[axes[0]] = values[0][i];
    target[axes[1]] = values[1][i];
    quality[i].innovationNis = std::max(nis[0][i], nis[1][i]);
    quality[i].positionSigma = std::hypot(sigma[0][i], sigma[1][i]);
    if (quality[i].innovationNis > config.rtsInnovationGate)
      quality[i].flags |= QualityInnovationOutlier;

    Eigen::Vector3f delta = target - original;
    delta[stationAxis] = 0.0f;
    const float requested = delta.norm();
    if (requested > config.rtsMaxCorrection && requested > 1.0e-8f) {
      delta *= config.rtsMaxCorrection / requested;
      quality[i].flags |= QualityCorrectionLimited;
    }
    target = projectSingleConstraint(original + delta, quality[i], config);

    if (!satisfiesGeometry(target, quality[i], config)) {
      const Eigen::Vector3f projected = target;
      float scale = 1.0f;
      for (int pass = 0; pass < 14; ++pass) {
        target = projectSingleConstraint(
            original + scale * (projected - original), quality[i], config);
        if (satisfiesGeometry(target, quality[i], config))
          break;
        scale *= 0.5f;
      }
      quality[i].flags |= QualityGeometryLimited;
      if (!satisfiesGeometry(target, quality[i], config))
        target = original;
    }
    points[i] = toPoint(target);
    quality[i].rtsCorrection = (target - original).norm();

    float evidence = 1.0f;
    if (quality[i].wallRms >= 0.0f)
      evidence = std::min(evidence, std::exp(-quality[i].wallRms /
                                             std::max(0.1f, config.surfaceRmsScale)));
    if (quality[i].floorRms >= 0.0f)
      evidence = std::min(evidence, std::exp(-quality[i].floorRms /
                                             std::max(0.1f, config.floorPatchMaxRms)));
    if (quality[i].innovationNis > config.rtsInnovationGate)
      evidence *= std::sqrt(config.rtsInnovationGate /
                            quality[i].innovationNis);
    if (i < confidence.size())
      confidence[i] = clamp01(confidence[i] * (0.5f + 0.5f * evidence));
  }
}

void enforcePairSeparation(SeamResult &result, const CloudT &leftBefore,
                           const CloudT &rightBefore,
                           const DetectorConfig &config) {
  const int sideAxis = localTransverseAxes(config.longitudinalAxis)[0];
  const std::size_t count = std::min(result.leftSeam->size(),
                                     result.rightSeam->size());
  for (std::size_t i = 0; i < count; ++i) {
    const float oldWidth = rightBefore[i].data[sideAxis] -
                           leftBefore[i].data[sideAxis];
    const float newWidth = (*result.rightSeam)[i].data[sideAxis] -
                           (*result.leftSeam)[i].data[sideAxis];
    if (newWidth >= config.minSeamSeparation)
      continue;
    float scale = 0.0f;
    if (oldWidth > config.minSeamSeparation && oldWidth > newWidth)
      scale = clamp01((oldWidth - config.minSeamSeparation) /
                      (oldWidth - newWidth));
    (*result.leftSeam)[i].data[sideAxis] =
        leftBefore[i].data[sideAxis] +
        scale * ((*result.leftSeam)[i].data[sideAxis] -
                 leftBefore[i].data[sideAxis]);
    (*result.rightSeam)[i].data[sideAxis] =
        rightBefore[i].data[sideAxis] +
        scale * ((*result.rightSeam)[i].data[sideAxis] -
                 rightBefore[i].data[sideAxis]);
    result.leftPointQuality[i].flags |= QualityPairConstraintLimited;
    result.rightPointQuality[i].flags |= QualityPairConstraintLimited;
  }
}

SeamSmoothingSummary summarize(const CloudT &before, const CloudT &after,
                               std::vector<SeamPointQuality> &quality) {
  SeamSmoothingSummary summary;
  summary.applied = true;
  float total = 0.0f;
  const std::size_t count = std::min(before.size(), after.size());
  for (std::size_t i = 0; i < count; ++i) {
    quality[i].rtsCorrection = (toVector(after[i]) - toVector(before[i])).norm();
    total += quality[i].rtsCorrection;
    summary.maxCorrection = std::max(summary.maxCorrection,
                                     quality[i].rtsCorrection);
    summary.innovationOutliers +=
        (quality[i].flags & QualityInnovationOutlier) != 0;
    summary.correctionLimited +=
        (quality[i].flags & QualityCorrectionLimited) != 0;
    summary.geometryLimited +=
        (quality[i].flags & QualityGeometryLimited) != 0;
    summary.pairConstraintLimited +=
        (quality[i].flags & QualityPairConstraintLimited) != 0;
  }
  summary.meanCorrection = count ? total / static_cast<float>(count) : 0.0f;
  return summary;
}

} // namespace

void applySeamPostprocess(SeamResult &result, const DetectorConfig &config) {
  if (config.postprocessMode != SeamPostprocessMode::RobustRts ||
      !result.leftSeam || !result.rightSeam)
    return;
  const CloudT leftBefore(*result.leftSeam), rightBefore(*result.rightSeam);
  smoothSide(*result.leftSeam, result.leftPointConfidence,
             result.leftPointQuality, result.leftSegmentStarts, config);
  smoothSide(*result.rightSeam, result.rightPointConfidence,
             result.rightPointQuality, result.rightSegmentStarts, config);
  if (leftBefore.size() == result.leftSeam->size() &&
      rightBefore.size() == result.rightSeam->size() &&
      result.leftPointQuality.size() == result.leftSeam->size() &&
      result.rightPointQuality.size() == result.rightSeam->size())
    enforcePairSeparation(result, leftBefore, rightBefore, config);
  result.leftSmoothing =
      summarize(leftBefore, *result.leftSeam, result.leftPointQuality);
  result.rightSmoothing =
      summarize(rightBefore, *result.rightSeam, result.rightPointQuality);
}

} // namespace weld

