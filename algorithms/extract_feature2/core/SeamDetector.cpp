/**
 * 方案一算法阅读入口
 *
 * 目标：从槽形点云中输出左右两条焊缝，并使可靠点同时位于局部底面和侧壁。
 * 坐标：A 为 longitudinalAxis；U/V 是 transverseAxes(A) 返回的两个截面轴。
 *       L 是 U 较小的一侧，R 是 U 较大的一侧，与屏幕左右、全局上下无关。
 * 数据：filteredCloud 用于搜索结构；局部侧壁/底面精修重新读取原始 input。
 *       不能把降采样中值点当作最后的实测平面支持，否则会再次产生偏移。
 *
 * 主流程索引（在编辑器中搜索对应 S 编号；文档使用相同编号）：
 * S00 输入检查 -> S01 有限点 -> S02 离群滤波 -> S03 体素降采样
 * -> S04 整船外壁包络裁剪 -> S05 纵向切片 -> S06 截面分箱
 * -> S07 三面结构候选 -> S08 局部折角 -> S09 保留主体
 * -> S10 成对路径优化 -> S11 内部短缺口插补 -> S12 曲线正则
 * -> S13 侧壁局部拟合 -> S14 高残差侧壁重拟合 -> S15 贴壁投影
 * -> S16 正方形底面拟合 -> S17 三约束交点 -> S18 输出。
 *
 * 关键约束：纵向站位固定；左右成对选取；没有可靠支撑就保留上一阶段结果；
 *           最终求交后不再自由平滑，否则点会离开两个拟合面。
 * 阅读入口：detect -> collectFrameSlices/collectSlices -> retainMainSegment -> optimizePairedPaths
 * -> interpolateInternalGaps -> fillSideResult。单侧处理在 fillSideResult
 * 中展开。 debugFlag 开启时，界面/CLI 经 detectWithDebugViewer 在 GUI
 * 线程逐步显示 VTK。 算法只发布 DebugStep；窗口逻辑位于 SeamDebugViewer.cpp。
 */

#include "SeamDetector.h"
#include "SeamSmoother.h"

#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#ifndef WELD_NO_PCL_FILTERS
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#endif

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>
#include <utility>
#include <vector>

namespace weld {
namespace {

thread_local const std::function<bool()> *activeCancellation = nullptr;
void checkCancellation() {
  if (activeCancellation && *activeCancellation && (*activeCancellation)()) throw ExecutionCancelled();
}
struct CancellationScope {
  const std::function<bool()> *previous;
  explicit CancellationScope(const std::function<bool()> &callback):previous(activeCancellation) { activeCancellation=&callback; }
  ~CancellationScope() {activeCancellation=previous;}
};
constexpr float kPi = 3.14159265358979323846f;

struct ProfilePoint {
  PointT point;
  float u = 0.0f;
  float v = 0.0f;
  // 当前横向分箱内包含的原始点数。大面的扫描点通常比边缘噪声密集，
  // 拟合时使用这个权重可以避免稀疏的外轮廓把侧面方向拉偏。
  float support = 1.0f;
  // 分箱只用于建立稳定的横向顺序；均值和箱内离差保留原始二维分布，
  // 因此同一个 Y 箱内沿 Z 展开的近竖直侧面不会被压扁成一个点。
  Eigen::Vector2f fitMean = Eigen::Vector2f::Zero();
  Eigen::Matrix2f fitScatter = Eigen::Matrix2f::Zero();
};

struct Candidate {
  PointT point;
  Eigen::Vector2f wallNormal = Eigen::Vector2f::Zero();
  float wallOffset = 0, wallRms = 0;
  bool hasWall = false;
  Eigen::Vector2f floorDirection = Eigen::Vector2f::UnitX();
  Eigen::Vector3f localOffset = Eigen::Vector3f::Zero();
  float feature = 0.0f;
  bool fromRegionEdges = false;
  bool fromLocalCorner = false;
  std::array<float, 2> historyDistance{
      {std::numeric_limits<float>::infinity(),
       std::numeric_limits<float>::infinity()}};
  std::array<bool, 2> historyAvailable{{false, false}};
  std::array<float, 2> nodeCost{{0.0f, 0.0f}};
};

struct SurfacePair {
  Candidate left;
  Candidate right;
  int interiorOpeningDirection = 0;
  float geometryCost = std::numeric_limits<float>::infinity();
  std::shared_ptr<DebugPair> debug;
};

struct Slice {
  float station = 0.0f;
  int frameIndex = -1;
  std::size_t debugIndex = 0;
  bool hasGeometry = false;
  std::vector<SurfacePair> pairs;
};

// 二维剖面中的直线使用“单位方向 + 法向距离”表示。与 v=k*u+b 相比，
// 这种正交拟合即使面对近乎竖直的侧面也不会出现无穷大斜率或巨大的伪残差。
struct Line2D {
  Eigen::Vector2f center = Eigen::Vector2f::Zero();
  Eigen::Vector2f direction = Eigen::Vector2f::UnitX();
  Eigen::Vector2f normal = Eigen::Vector2f::UnitY();
  float offset = 0.0f;
  float rms = std::numeric_limits<float>::infinity();
  float support = 0.0f;
  bool valid = false;
};

// profile 中的半开区间 [begin, end)。区域生长只沿已经按横向坐标排序的
// 剖面相邻点进行，因而不会把三维点云中另一个纵向站位的近邻误接进来。
struct ProfileRegion {
  int begin = 0;
  int end = 0;
  Line2D seedLine;
  float quality = 0.0f;
  bool valid = false;
};

bool stepDebugEnabled(const DebugTrace *debug) {
  checkCancellation();
  return debug && debug->config.debugFlag && bool(debug->onStep);
}

void publishDebugStep(DebugTrace *debug, const std::string &title,
                      std::vector<DebugCloudStage> layers,
                      const std::string &message = {},
                      const std::map<std::string, double> &metrics = {}) {
  if (!stepDebugEnabled(debug))
    return;
  DebugStep step;
  step.title = title;
  step.status = "completed";
  step.message = message;
  step.metrics = metrics;
  step.layers = std::move(layers);
  if (!debug->onStep(step))
    debug->onStep = {};
}

CloudT::Ptr pointsCloud(const std::vector<PointT> &points) {
  CloudT::Ptr cloud(new CloudT);
  cloud->reserve(points.size());
  for (const auto &point : points)
    cloud->push_back(point);
  return cloud;
}

CloudT::Ptr candidateCloud(const std::vector<Candidate> &path) {
  CloudT::Ptr cloud(new CloudT);
  cloud->reserve(path.size());
  for (const auto &candidate : path)
    cloud->push_back(candidate.point);
  return cloud;
}

void publishPathStep(DebugTrace *debug, const std::string &title,
                     const std::vector<Candidate> &left,
                     const std::vector<Candidate> &right) {
  if (stepDebugEnabled(debug))
    publishDebugStep(
        debug, title,
        {{"左焊缝", candidateCloud(left)}, {"右焊缝", candidateCloud(right)}});
}

void publishProfileStep(DebugTrace *debug,
                        const std::vector<ProfilePoint> &profile,
                        float station, const DetectorConfig &config) {
  if (!stepDebugEnabled(debug))
    return;
  CloudT::Ptr cloud(new CloudT);
  double supportSum = 0.0;
  int lowSupportBins = 0;
  for (const auto &point : profile) {
    cloud->push_back(point.point);
    supportSum += point.support;
    lowSupportBins += point.support < 3.0f;
  }
  const double geometryScale =
      config.profileSamplingMode == ProfileSamplingMode::StableGrid
          ? config.profileReferenceScale
          : config.profileBinSize;
  publishDebugStep(debug, "S06 截面分箱 · " + std::to_string(station),
                   {{"分箱轮廓", cloud}}, {},
                   {{"profile_bins", static_cast<double>(profile.size())},
                    {"mean_bin_support",
                     profile.empty() ? 0.0
                                     : supportSum / double(profile.size())},
                    {"low_support_bins", double(lowSupportBins)},
                    {"profile_bin_size", config.profileBinSize},
                    {"profile_geometry_scale", geometryScale}});
}

void publishPairStep(DebugTrace *debug, const std::vector<SurfacePair> &pairs,
                     float station) {
  if (!stepDebugEnabled(debug))
    return;
  std::vector<Candidate> left, right, corners;
  for (const auto &pair : pairs) {
    left.push_back(pair.left);
    right.push_back(pair.right);
    if (pair.left.fromLocalCorner)
      corners.push_back(pair.left);
    if (pair.right.fromLocalCorner)
      corners.push_back(pair.right);
  }
  publishPathStep(debug, "S07 三面结构候选 · " + std::to_string(station), left,
                  right);
  if (stepDebugEnabled(debug))
    publishDebugStep(debug, "S08 局部折角 · " + std::to_string(station),
                     {{"局部折角候选", candidateCloud(corners)}});
}

void publishRetainedSlices(DebugTrace *debug,
                           const std::vector<Slice> &slices) {
  if (!stepDebugEnabled(debug))
    return;
  std::vector<Candidate> left, right;
  for (const auto &slice : slices)
    for (const auto &pair : slice.pairs) {
      left.push_back(pair.left);
      right.push_back(pair.right);
    }
  publishPathStep(debug, "S09 保留主体连续段", left, right);
}

std::string sideStepTitle(const char *stage, int side, const PointT &point,
                          Axis axis) {
  return std::string(stage) + (side == 0 ? " · L · " : " · R · ") +
         std::to_string(coordinate(point, axis));
}

float clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }

float median(std::vector<float> values) {
  if (values.empty()) {
    return 0.0f;
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  float result = values[middle];
  if (values.size() % 2 == 0) {
    const auto lower =
        std::max_element(values.begin(), values.begin() + middle);
    result = 0.5f * (result + *lower);
  }
  return result;
}

std::array<int, 2> transverseAxes(Axis longitudinal) {
  switch (longitudinal) {
  case Axis::X:
    return {{1, 2}};
  case Axis::Y:
    return {{0, 2}};
  case Axis::Z:
    return {{0, 1}};
  }
  return {{1, 2}};
}

float component(const PointT &point, int axis) {
  if (axis == 0)
    return point.x;
  if (axis == 1)
    return point.y;
  return point.z;
}

float transverseDistance(const PointT &lhs, const PointT &rhs,
                         Axis longitudinal) {
  Eigen::Vector3f difference = toVector(lhs) - toVector(rhs);
  difference[static_cast<int>(longitudinal)] = 0.0f;
  return difference.norm();
}

float transverseDistance(const PointT &lhs, const Eigen::Vector3f &rhs,
                         Axis longitudinal) {
  Eigen::Vector3f difference = toVector(lhs) - rhs;
  difference[static_cast<int>(longitudinal)] = 0.0f;
  return difference.norm();
}

float transverseOffsetDistance(const Candidate &lhs, const Candidate &rhs,
                               Axis longitudinal) {
  Eigen::Vector3f difference = lhs.localOffset - rhs.localOffset;
  difference[static_cast<int>(longitudinal)] = 0.0f;
  return difference.norm();
}

bool stableProfileSampling(const DetectorConfig &config) {
  return config.profileSamplingMode == ProfileSamplingMode::StableGrid;
}

float profileGeometryScale(const DetectorConfig &config) {
  return std::max(
      0.05f, stableProfileSampling(config) ? config.profileReferenceScale
                                           : config.profileBinSize);
}

int scaledProfileCount(const DetectorConfig &config, int configuredCount,
                       int legacyMinimum, int stableMinimum = 3) {
  const int legacyCount = std::max(legacyMinimum, configuredCount);
  if (!stableProfileSampling(config))
    return legacyCount;
  const float physicalSpan =
      static_cast<float>(legacyCount) * profileGeometryScale(config);
  return std::max(stableMinimum,
                  static_cast<int>(std::lround(
                      physicalSpan / std::max(0.05f, config.profileBinSize))));
}

float maximumProfileGap(const DetectorConfig &config) {
  return std::max(0.2f, 3.0f * profileGeometryScale(config));
}

// S07 辅助：按每个分箱的原始支持点数、均值和箱内散布重建协方差。
// 最小特征值对应直线法向，sqrt(最小特征值) 为正交 RMS。
// 不采用普通 y=kx+b 拟合，因为侧壁可能接近竖直，斜率会发散。
Line2D fitOrthogonalLine(const std::vector<ProfilePoint> &profile, int begin,
                         int end, bool contourOnly = false) {
  Line2D line;
  if (begin < 0 || end > static_cast<int>(profile.size()) || end - begin < 3) {
    return line;
  }

  // 第一步：用每个横向分箱的原始点数加权求中心和协方差。
  for (int i = begin; i < end; ++i) {
    const ProfilePoint &point = profile[static_cast<std::size_t>(i)];
    const float weight = contourOnly ? 1.0f : std::max(1.0f, point.support);
    const Eigen::Vector2f mean =
        contourOnly ? Eigen::Vector2f(point.u, point.v) : point.fitMean;
    line.center += weight * mean;
    line.support += weight;
  }
  if (line.support < 3.0f) {
    return line;
  }
  line.center /= line.support;

  Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
  for (int i = begin; i < end; ++i) {
    const ProfilePoint &point = profile[static_cast<std::size_t>(i)];
    const float weight = contourOnly ? 1.0f : std::max(1.0f, point.support);
    const Eigen::Vector2f mean =
        contourOnly ? Eigen::Vector2f(point.u, point.v) : point.fitMean;
    const Eigen::Vector2f difference = mean - line.center;
    if (contourOnly) {
      covariance += weight * difference * difference.transpose();
    } else {
      covariance +=
          point.fitScatter + weight * difference * difference.transpose();
    }
  }
  covariance /= line.support;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
  if (solver.info() != Eigen::Success || solver.eigenvalues().y() < 1.0e-5f) {
    return line;
  }
  line.direction = solver.eigenvectors().col(1).normalized();
  // 统一让方向朝横向坐标增大的方向，后面才能判断左右侧面是否向相反方向弯折。
  if (line.direction.x() < 0.0f) {
    line.direction = -line.direction;
  }
  line.normal = Eigen::Vector2f(-line.direction.y(), line.direction.x());
  line.offset = line.normal.dot(line.center);

  line.rms =
      std::sqrt(std::max(0.0f, line.normal.dot(covariance * line.normal)));
  line.valid = std::isfinite(line.rms);
  return line;
}

float acuteAngle(const Line2D &first, const Line2D &second) {
  const float cosine = std::max(
      0.0f, std::min(1.0f, std::fabs(first.direction.dot(second.direction))));
  return std::acos(cosine);
}

float cross2D(const Eigen::Vector2f &first, const Eigen::Vector2f &second) {
  return first.x() * second.y() - first.y() * second.x();
}

bool intersectLines(const Line2D &first, const Line2D &second,
                    Eigen::Vector2f *intersection) {
  if (!intersection || !first.valid || !second.valid) {
    return false;
  }
  Eigen::Matrix2f normals;
  normals.row(0) = first.normal.transpose();
  normals.row(1) = second.normal.transpose();
  if (std::fabs(normals.determinant()) < 1.0e-3f) {
    return false;
  }
  *intersection =
      normals.inverse() * Eigen::Vector2f(first.offset, second.offset);
  return intersection->allFinite();
}

// S06 截面轮廓：输入当前厚截面的原始采样，输出按 U 递增排列的分箱。
// 代表点取各坐标中位数供定位；另外保留 support/fitMean/fitScatter 供面拟合。
// 当箱内纵向跨度足够时扣除局部纵向线性趋势，减轻厚截面在弯曲区的混叠。
// 断点观察：points.size()、result.size()、每箱 support 和 fitScatter。
std::vector<ProfilePoint> makeProfile(const std::vector<PointT> &points,
                                      Axis longitudinal,
                                      const DetectorConfig &config,
                                      float station = 0.0f,
                                      float detrendMinimumSpan = 0.0f) {
  std::vector<ProfilePoint> result;
  if (points.size() < 5) {
    return result;
  }

  const std::array<int, 2> axes = transverseAxes(longitudinal);

  // 坐标系固定时，整条轨迹必须使用同一剖面定义。之前逐片选择变化范围较大
  // 的坐标，船体高度范围超过宽度时会从 Y 突然切换到 Z，产生分叉和跨面。
  // 当前 X 向扫描因此始终按 Y 排序并拟合 Z；其它纵轴也使用固定的剩余轴序。
  const int uAxis = axes[0];
  const int vAxis = axes[1];

  std::vector<PointT> sorted = points;
  std::sort(sorted.begin(), sorted.end(),
            [uAxis](const PointT &lhs, const PointT &rhs) {
              return component(lhs, uAxis) < component(rhs, uAxis);
            });

  const float safeBinSize = std::max(0.05f, config.profileBinSize);
  std::size_t begin = 0;
  while (begin < sorted.size()) {
    const float binStart = component(sorted[begin], uAxis);
    std::size_t end = begin + 1;
    if (stableProfileSampling(config)) {
      const auto binIndex = static_cast<long long>(
          std::floor((binStart - config.profileGridOrigin) / safeBinSize));
      while (end < sorted.size()) {
        const float coordinateU = component(sorted[end], uAxis);
        const auto nextIndex = static_cast<long long>(std::floor(
            (coordinateU - config.profileGridOrigin) / safeBinSize));
        if (nextIndex != binIndex)
          break;
        ++end;
      }
    } else {
      // Original rolling binning remains the compatibility path.
      while (end < sorted.size() &&
             component(sorted[end], uAxis) - binStart < safeBinSize) {
        ++end;
      }
    }

    std::vector<PointT> corrected(sorted.begin() + begin, sorted.begin() + end);
    // 同一横向箱里的点来自截面厚度内的多个纵向位置。分别估计每个箱的
    // Theil-Sen 斜率，可去掉船体纵向陡坡，又不会受截面覆盖范围变化影响。
    if (detrendMinimumSpan > 0.0f && corrected.size() >= 3) {
      const int longitudinalAxis = static_cast<int>(longitudinal);
      std::vector<float> slopes;
      for (std::size_t i = 0; i < corrected.size(); ++i) {
        for (std::size_t j = i + 1; j < corrected.size(); ++j) {
          const float delta = corrected[j].data[longitudinalAxis] -
                              corrected[i].data[longitudinalAxis];
          if (std::fabs(delta) >= detrendMinimumSpan) {
            slopes.push_back(
                (corrected[j].data[vAxis] - corrected[i].data[vAxis]) / delta);
          }
        }
      }
      if (!slopes.empty()) {
        const float slope = median(std::move(slopes));
        if (std::isfinite(slope) && std::fabs(slope) < 100.0f) {
          for (PointT &point : corrected) {
            point.data[vAxis] -=
                slope * (point.data[longitudinalAxis] - station);
          }
        }
      }
    }

    std::vector<float> xs;
    std::vector<float> ys;
    std::vector<float> zs;
    xs.reserve(end - begin);
    ys.reserve(end - begin);
    zs.reserve(end - begin);
    for (const PointT &point : corrected) {
      xs.push_back(point.x);
      ys.push_back(point.y);
      zs.push_back(point.z);
    }
    ProfilePoint sample;
    sample.point = PointT(median(xs), median(ys), median(zs));
    sample.u = component(sample.point, uAxis);
    sample.v = component(sample.point, vAxis);
    sample.support = static_cast<float>(end - begin);
    for (const PointT &point : corrected) {
      sample.fitMean +=
          Eigen::Vector2f(component(point, uAxis), component(point, vAxis));
    }
    sample.fitMean /= sample.support;
    for (const PointT &point : corrected) {
      const Eigen::Vector2f difference(
          component(point, uAxis) - sample.fitMean.x(),
          component(point, vAxis) - sample.fitMean.y());
      sample.fitScatter += difference * difference.transpose();
    }
    result.push_back(sample);
    begin = end;
  }
  return result;
}

// S07 辅助：从交界向底面内侧逐步扩大拟合窗口。
// 法向漂移过大时停止扩大，防止把局部弯曲底面压成跨槽统一平面。
Line2D fitAdaptiveBoundaryLine(const std::vector<ProfilePoint> &profile,
                               int anchor, int direction, int minimumWindow,
                               int maximumWindow,
                               const DetectorConfig &config) {
  Line2D selected;
  Line2D previous;
  const float maximumNormalChange =
      config.maxLocalFitNormalChangeDegrees * kPi / 180.0f;
  const float maximumRms = std::max(0.25f, config.surfaceRmsScale);
  for (int window = std::max(3, minimumWindow); window <= maximumWindow;
       ++window) {
    const int begin = direction > 0 ? anchor : anchor - window;
    const int end = direction > 0 ? anchor + window : anchor;
    const Line2D current = fitOrthogonalLine(profile, begin, end);
    if (!current.valid || current.rms > maximumRms) {
      continue;
    }
    if (previous.valid && acuteAngle(previous, current) > maximumNormalChange) {
      break;
    }
    selected = current;
    previous = current;
  }
  return selected;
}

float pointToLineDistance(const ProfilePoint &point, const Line2D &line) {
  if (!line.valid) {
    return std::numeric_limits<float>::infinity();
  }
  return std::fabs(line.normal.dot(point.fitMean) - line.offset);
}

// S08 辅助：从可靠面种子沿轮廓生长，逐点检查切线变化、到面距离和总法向漂移。
// 底面与侧壁分别生长；面内连续起伏可以保留，发生真实折角时停止。
ProfileRegion growProfileRegion(const std::vector<ProfilePoint> &profile,
                                int seedBegin, int seedEnd, int minimumIndex,
                                int maximumIndex, const DetectorConfig &config,
                                bool contourOnly = false) {
  ProfileRegion region;
  const int pointCount = static_cast<int>(profile.size());
  minimumIndex = std::max(0, minimumIndex);
  maximumIndex = std::min(pointCount, maximumIndex);
  seedBegin = std::max(minimumIndex, seedBegin);
  seedEnd = std::min(maximumIndex, seedEnd);
  if (seedEnd - seedBegin < 3) {
    return region;
  }

  region.begin = seedBegin;
  region.end = seedEnd;
  region.seedLine = fitOrthogonalLine(profile, seedBegin, seedEnd, contourOnly);
  if (!region.seedLine.valid) {
    return region;
  }

  // 轮廓中值仍有离散噪声。底面用较宽的局部窗口估计趋势，避免四个点的
  // 微小起伏使生长反复停止；真实侧面的方向突变仍由步进角和总漂移限制。
  const int window =
      scaledProfileCount(config, config.regionGrowingWindow, 3) *
      (contourOnly ? 2 : 1);
  const float maximumStepAngle =
      config.regionGrowingSmoothnessDegrees * kPi / 180.0f;
  const float maximumDrift =
      config.regionGrowingMaxNormalDriftDegrees * kPi / 180.0f;
  const float maximumDistance = std::max(0.1f, config.regionGrowingDistance);
  const float maximumGap = maximumProfileGap(config);
  float acceptedQuality = 0.0f;
  int acceptedCount = 0;

  auto acceptAt = [&](int index, int direction) {
    int localBegin = 0;
    int localEnd = 0;
    if (direction < 0) {
      localBegin = region.begin;
      localEnd = std::min(region.end, region.begin + window);
    } else {
      localBegin = std::max(region.begin, region.end - window);
      localEnd = region.end;
    }
    const Line2D local =
        fitOrthogonalLine(profile, localBegin, localEnd, contourOnly);
    if (!local.valid) {
      return false;
    }

    const int expandedBegin = direction < 0 ? index : localBegin;
    const int expandedEnd = direction < 0 ? localEnd : index + 1;
    const Line2D expanded =
        fitOrthogonalLine(profile, expandedBegin, expandedEnd, contourOnly);
    if (!expanded.valid || acuteAngle(local, expanded) > maximumStepAngle ||
        acuteAngle(region.seedLine, expanded) > maximumDrift) {
      return false;
    }

    const int adjacent = direction < 0 ? index + 1 : index - 1;
    if (std::fabs(profile[static_cast<std::size_t>(index)].u -
                  profile[static_cast<std::size_t>(adjacent)].u) > maximumGap) {
      return false;
    }
    const auto &sample = profile[static_cast<std::size_t>(index)];
    const float distance =
        contourOnly
            ? std::fabs(local.normal.dot(Eigen::Vector2f(sample.u, sample.v)) -
                        local.offset)
            : pointToLineDistance(sample, local);
    const float distanceTolerance =
        contourOnly ? std::max(maximumDistance, std::min(2.0f * maximumDistance,
                                                         2.5f * local.rms))
                    : maximumDistance;
    if (distance > distanceTolerance) {
      return false;
    }
    acceptedQuality += std::exp(-distance / distanceTolerance) *
                       std::exp(-acuteAngle(local, expanded) /
                                std::max(1.0e-3f, maximumStepAngle));
    ++acceptedCount;
    return true;
  };

  while (region.begin > minimumIndex && acceptAt(region.begin - 1, -1)) {
    --region.begin;
  }
  while (region.end < maximumIndex && acceptAt(region.end, +1)) {
    ++region.end;
  }

  region.quality =
      acceptedCount > 0
          ? clamp01(acceptedQuality / static_cast<float>(acceptedCount))
          : 0.5f;
  region.valid = region.end - region.begin >= 3;
  return region;
}

struct RegionEdgePair {
  Eigen::Vector2f sidePoint = Eigen::Vector2f::Zero();
  Eigen::Vector2f groundPoint = Eigen::Vector2f::Zero();
  float distance = std::numeric_limits<float>::infinity();
  bool valid = false;
};

// 对区域已经相交的圆角，取两个生长前沿之间的中间位置；存在缺口时，
// 则在两个前沿的小邻域内寻找真正的二维最近点。搜索始终局限在同一截面。
RegionEdgePair matchRegionEdges(const std::vector<ProfilePoint> &profile,
                                const ProfileRegion &side,
                                const ProfileRegion &ground, bool left,
                                const DetectorConfig &config) {
  RegionEdgePair result;
  if (!side.valid || !ground.valid) {
    return result;
  }

  const int sideFront = left ? side.end - 1 : side.begin;
  const int groundFront = left ? ground.begin : ground.end - 1;
  const bool overlaps =
      left ? sideFront >= groundFront : groundFront >= sideFront;
  if (overlaps) {
    const int middle = (sideFront + groundFront) / 2;
    const ProfilePoint &point = profile[static_cast<std::size_t>(middle)];
    result.sidePoint = Eigen::Vector2f(point.u, point.v);
    result.groundPoint = result.sidePoint;
    result.distance = 0.0f;
    result.valid = true;
    return result;
  }

  const int edgeSamples =
      scaledProfileCount(config, config.regionGrowingWindow, 2, 2);
  const int sideBegin =
      left ? std::max(side.begin, side.end - edgeSamples) : side.begin;
  const int sideEnd =
      left ? side.end : std::min(side.end, side.begin + edgeSamples);
  const int groundBegin =
      left ? ground.begin : std::max(ground.begin, ground.end - edgeSamples);
  const int groundEnd =
      left ? std::min(ground.end, ground.begin + edgeSamples) : ground.end;
  for (int sideIndex = sideBegin; sideIndex < sideEnd; ++sideIndex) {
    const Eigen::Vector2f sidePoint(
        profile[static_cast<std::size_t>(sideIndex)].u,
        profile[static_cast<std::size_t>(sideIndex)].v);
    for (int groundIndex = groundBegin; groundIndex < groundEnd;
         ++groundIndex) {
      const Eigen::Vector2f groundPoint(
          profile[static_cast<std::size_t>(groundIndex)].u,
          profile[static_cast<std::size_t>(groundIndex)].v);
      const float distance = (sidePoint - groundPoint).norm();
      if (distance < result.distance) {
        result.sidePoint = sidePoint;
        result.groundPoint = groundPoint;
        result.distance = distance;
      }
    }
  }
  result.valid = std::isfinite(result.distance) &&
                 result.distance <= config.regionGrowingMaxEdgeGap;
  return result;
}

Candidate candidateFromRegionEdges(const std::vector<ProfilePoint> &profile,
                                   const RegionEdgePair &edges,
                                   Axis longitudinal, float feature) {
  Candidate candidate;
  if (!edges.valid || profile.empty()) {
    return candidate;
  }
  const std::array<int, 2> axes = transverseAxes(longitudinal);
  const Eigen::Vector2f midpoint = 0.5f * (edges.sidePoint + edges.groundPoint);
  const auto nearest = std::min_element(
      profile.begin(), profile.end(),
      [&midpoint](const ProfilePoint &lhs, const ProfilePoint &rhs) {
        return (Eigen::Vector2f(lhs.u, lhs.v) - midpoint).squaredNorm() <
               (Eigen::Vector2f(rhs.u, rhs.v) - midpoint).squaredNorm();
      });
  candidate.point = nearest->point;
  candidate.point.data[axes[0]] = midpoint.x();
  candidate.point.data[axes[1]] = midpoint.y();
  candidate.feature = clamp01(feature);
  candidate.fromRegionEdges = true;
  return candidate;
}

Candidate candidateFromIntersection(const std::vector<ProfilePoint> &profile,
                                    const Eigen::Vector2f &intersection,
                                    Axis longitudinal, float feature,
                                    float profileBinSize) {
  Candidate candidate;
  if (profile.empty()) {
    return candidate;
  }
  const std::array<int, 2> axes = transverseAxes(longitudinal);
  std::vector<std::size_t> nearestIndices(profile.size());
  std::iota(nearestIndices.begin(), nearestIndices.end(), 0);
  std::sort(
      nearestIndices.begin(), nearestIndices.end(),
      [&profile, &intersection](std::size_t lhs, std::size_t rhs) {
        return (Eigen::Vector2f(profile[lhs].u, profile[lhs].v) - intersection)
                   .squaredNorm() <
               (Eigen::Vector2f(profile[rhs].u, profile[rhs].v) - intersection)
                   .squaredNorm();
      });
  const ProfilePoint &nearest = profile[nearestIndices.front()];

  candidate.point = nearest.point;
  // 单个最近点会随采样密度和毛刺在相邻箱之间跳动。取交点附近最多五个
  // 样本的坐标中值作为弱观测，理论交点仍保持主导。
  std::vector<float> nearbyU;
  std::vector<float> nearbyV;
  const float refineRadius = std::max(1.0f, 2.5f * profileBinSize);
  for (std::size_t index : nearestIndices) {
    const ProfilePoint &sample = profile[index];
    if ((Eigen::Vector2f(sample.u, sample.v) - intersection).norm() >
        refineRadius) {
      break;
    }
    nearbyU.push_back(sample.u);
    nearbyV.push_back(sample.v);
    if (nearbyU.size() >= 5) {
      break;
    }
  }
  const Eigen::Vector2f measured(
      nearbyU.empty() ? nearest.u : median(std::move(nearbyU)),
      nearbyV.empty() ? nearest.v : median(std::move(nearbyV)));
  const Eigen::Vector2f refined = 0.90f * intersection + 0.10f * measured;
  candidate.point.data[axes[0]] = refined.x();
  candidate.point.data[axes[1]] = refined.y();
  candidate.feature = clamp01(feature);
  return candidate;
}

Candidate nearestHistoryCandidate(const std::vector<ProfilePoint> &profile,
                                  const HistoryPrediction &prediction,
                                  const DetectorConfig &config) {
  Candidate candidate;
  if (!prediction.valid || profile.empty()) {
    return candidate;
  }
  const auto nearest = std::min_element(
      profile.begin(), profile.end(),
      [&prediction, &config](const ProfilePoint &lhs, const ProfilePoint &rhs) {
        return transverseDistance(lhs.point, prediction.point,
                                  config.longitudinalAxis) <
               transverseDistance(rhs.point, prediction.point,
                                  config.longitudinalAxis);
      });
  candidate.point = nearest->point;
  candidate.feature = 0.05f;
  return candidate;
}

struct LocalContourCorner {
  int index = -1;
  float angle = 0.0f;
  float quality = 0.0f;
};

// 区域前沿之间有缺口或重叠时，中点未必是板面交界。只在已经通过三面
// 结构确认的候选附近，寻找“稳定侧面 -> 局部底面轮廓”的实测转折。
// 外侧短窗口必须与原始分布拟合的侧面同向，内侧短窗口必须明显转向；
// 代表点还必须贴近侧面且接近原候选，避免选到槽内起伏或外侧肩部。
LocalContourCorner
findLocalContourCorner(const std::vector<ProfilePoint> &profile,
                       const Line2D &sideSurface, const Line2D &broadBottom,
                       const Eigen::Vector2f &reference, int boundary,
                       bool left, const DetectorConfig &config) {
  LocalContourCorner best;
  if (acuteAngle(sideSurface, broadBottom) <
      config.minContourCornerAngleDegrees * kPi / 180.0f)
    return best;
  const int window = scaledProfileCount(
      config, std::min(5, config.localAngleWindow), 3);
  const int n = static_cast<int>(profile.size());
  const int radius =
      scaledProfileCount(config, config.fitHalfWindow, 5) +
      scaledProfileCount(config, config.fitGuardPoints, 1, 1);
  const float maxDistance = std::max(0.5f, config.regionGrowingDistance);
  const float minTurn = std::max(config.minLocalSurfaceAngleDegrees,
                                 config.minSurfaceAngleDegrees) *
                        kPi / 180.0f;
  const float maxSideAngle =
      std::max(12.0f, config.regionGrowingSmoothnessDegrees) * kPi / 180.0f;
  for (int k = std::max(window - 1, boundary - radius);
       k <= std::min(n - window, boundary + radius); ++k) {
    const auto &sample = profile[static_cast<std::size_t>(k)];
    const Eigen::Vector2f point(sample.u, sample.v);
    const float sideDistance =
        std::fabs(sideSurface.normal.dot(point) - sideSurface.offset);
    if (sideDistance > maxDistance ||
        (point - reference).norm() > config.surfaceIntersectionTolerance)
      continue;
    // 不把没有采样的孔洞当成观测到的折角。
    bool gap = false;
    for (int j = k - window + 2; j < k + window; ++j) {
      if (profile[j].u - profile[j - 1].u > maximumProfileGap(config)) {
        gap = true;
        break;
      }
    }
    if (gap)
      continue;
    const Line2D before =
        fitOrthogonalLine(profile, k - window + 1, k + 1, true);
    const Line2D after = fitOrthogonalLine(profile, k, k + window, true);
    if (!before.valid || !after.valid)
      continue;
    const Line2D &outer = left ? before : after;
    const Line2D &inner = left ? after : before;
    const float sideAngle = acuteAngle(outer, sideSurface);
    const float turn = acuteAngle(outer, inner);
    if (sideAngle > maxSideAngle || turn < minTurn || outer.rms > maxDistance ||
        inner.rms > maxDistance)
      continue;
    const float quality =
        clamp01(turn / (0.5f * kPi)) * std::exp(-sideAngle / maxSideAngle) *
        std::exp(-sideDistance / maxDistance) *
        std::exp(-(outer.rms + inner.rms) / (2.0f * maxDistance));
    if (quality > best.quality) {
      best.index = k;
      best.angle = turn * 180.0f / kPi;
      best.quality = quality;
    }
  }
  return best;
}

// 跨槽底拟合只作为稳定的结构基准；左右交界附近再分别做多尺度局部底面
// 拟合。局部结果与基准方向、交点都一致时才有限修正左右交点，从而兼顾
// 焊接翘曲和缺口/焊道附近的稳定性。
// S07 三面结构候选：枚举左/右边界，拟合左侧壁、跨槽底面和右侧壁。
// 依次检查有效拟合、底面斜率、两侧交角、局部法向变化、转向和左右间距。
// 再进入 S08 的区域边缘/轮廓折角定位；有限数量候选按质量排序去重。
// 调试重点：哪一个 continue 门槛丢弃了候选，而不是先放大所有阈值。
struct SurfaceProposal {
  SurfacePair pair;
  float cost = std::numeric_limits<float>::infinity();
};

// 一组边界候选的局部状态，只在本次评估中使用，不跨截面共享。
// 各阶段依次完成结构验证、交点定位、区域生长和局部折角修正。
class SurfacePairBuilder {
public:
  SurfacePairBuilder(const std::vector<ProfilePoint> &profile,
                     const DetectorConfig &config, int leftBoundary,
                     int rightBoundary, DebugSlice *debug)
      : profile(profile), config(config), leftBoundary(leftBoundary),
        rightBoundary(rightBoundary), debug(debug),
        sideWindow(scaledProfileCount(config, config.fitHalfWindow, 5)),
        guard(scaledProfileCount(config, config.fitGuardPoints, 1, 1)),
        localWindow(std::max(
            3, std::min(sideWindow,
                        scaledProfileCount(config, config.localAngleWindow,
                                           3)))),
        pointCount(static_cast<int>(profile.size())) {}

  bool build(SurfaceProposal &result) {
    if (!fitSurfaces() || !validateLocalAngles() || !locateIntersections())
      return false;
    scoreGeometry();
    recordSurfaceFits();
    growRegionCandidates();
    refineContourCandidates();
    attachSurfaceConstraints();
    finishProposal();
    result = std::move(proposal);
    return true;
  }

private:
  void countGate(const char *gate) {
    if (debug)
      ++debug->gates[gate];
  }
  bool fitSurfaces() {
    // 侧面使用交接点外侧的一段；底面从每个交界向槽内分别做多尺度
    // 局部拟合。guard 区域故意不参加拟合，因为这里通常是焊道冠部。
    leftSurface = fitOrthogonalLine(profile, leftBoundary - guard - sideWindow,
                                    leftBoundary - guard);
    broadBottom =
        fitOrthogonalLine(profile, leftBoundary + guard, rightBoundary - guard);
    rightSurface = fitOrthogonalLine(profile, rightBoundary + guard,
                                     rightBoundary + guard + sideWindow);
    const int availableBottomPoints = rightBoundary - leftBoundary - 2 * guard;
    const int maximumBottomWindow = std::min(
        availableBottomPoints / 2, std::max(localWindow, 2 * sideWindow));
    if (!leftSurface.valid || !broadBottom.valid || !rightSurface.valid ||
        maximumBottomWindow < localWindow) {
      countGate("invalid_fit");
      return false;
    }
    leftBottom =
        fitAdaptiveBoundaryLine(profile, leftBoundary + guard, +1, localWindow,
                                maximumBottomWindow, config);
    rightBottom =
        fitAdaptiveBoundaryLine(profile, rightBoundary - guard, -1, localWindow,
                                maximumBottomWindow, config);
    broadBottomSlope = std::fabs(broadBottom.direction.y()) /
                       std::max(1.0e-3f, std::fabs(broadBottom.direction.x()));
    const float maximumBottomAngleDifference =
        config.maxLocalBottomAngleDifferenceDegrees * kPi / 180.0f;
    leftLocalUsable = leftBottom.valid && acuteAngle(leftBottom, broadBottom) <=
                                              maximumBottomAngleDifference;
    rightLocalUsable =
        rightBottom.valid &&
        acuteAngle(rightBottom, broadBottom) <= maximumBottomAngleDifference;
    leftAngle = acuteAngle(leftSurface, broadBottom);
    rightAngle = acuteAngle(rightSurface, broadBottom);
    if (broadBottomSlope > config.maxBottomSlope ||
        leftAngle < config.minSurfaceAngleDegrees * kPi / 180.0f ||
        rightAngle < config.minSurfaceAngleDegrees * kPi / 180.0f) {
      countGate("slope_or_broad_angle");
      return false;
    }

    return true;
  }

  bool validateLocalAngles() {
    // 仅比较远处大面的方向会把平滑弧面两端的累计转角误判为折角。
    // 因此在每个理论交界的两侧各取一个短窗口：真正的板面交接会在很短
    // 距离内改变法向，而同一弧面的切向只会缓慢连续变化。
    const Line2D leftLocalOuter = fitOrthogonalLine(
        profile, leftBoundary - guard - localWindow, leftBoundary - guard);
    const Line2D leftLocalInner = fitOrthogonalLine(
        profile, leftBoundary + guard, leftBoundary + guard + localWindow);
    const Line2D rightLocalInner = fitOrthogonalLine(
        profile, rightBoundary - guard - localWindow, rightBoundary - guard);
    const Line2D rightLocalOuter = fitOrthogonalLine(
        profile, rightBoundary + guard, rightBoundary + guard + localWindow);
    if (!leftLocalOuter.valid || !leftLocalInner.valid ||
        !rightLocalInner.valid || !rightLocalOuter.valid) {
      countGate("invalid_local_fit");
      return false;
    }
    leftLocalAngle = acuteAngle(leftLocalOuter, leftLocalInner);
    rightLocalAngle = acuteAngle(rightLocalInner, rightLocalOuter);
    // 平均交角负责保留一侧轻微遮挡的截面，同时每一侧还必须达到一半
    // 下限，防止一侧噪声替另一侧完全平滑的弧面“代为通过”。
    if (std::min(leftLocalAngle, rightLocalAngle) <
            0.5f * (config.minLocalSurfaceAngleDegrees * kPi / 180.0f) ||
        0.5f * (leftLocalAngle + rightLocalAngle) <
            (config.minLocalSurfaceAngleDegrees * kPi / 180.0f)) {
      countGate("local_angle");
      return false;
    }

    // U 形结构的两个侧面必须位于底面方向的两侧。这个符号约束会排除
    // 同一块侧面上的两个纹理折角，但不依赖 U 形朝上还是朝下。
    leftTurn = cross2D(broadBottom.direction, leftSurface.direction);
    const float rightTurn =
        cross2D(broadBottom.direction, rightSurface.direction);
    if (leftTurn * rightTurn >= 0.0f) {
      countGate("turn_direction");
      return false;
    }

    return true;
  }

  bool locateIntersections() {
    if (!intersectLines(leftSurface, broadBottom, &leftIntersection) ||
        !intersectLines(rightSurface, broadBottom, &rightIntersection)) {
      countGate("parallel_lines");
      return false;
    }
    // 跨槽底拟合提供稳定基准；只有局部底面与基准方向一致、且算出的
    // 交点没有远离基准时，才允许它做有限修正。这样既能跟随缓慢翘曲，
    // 又不会因焊道或缺口让交点突然跳到另一处。
    auto refineWithLocalBottom = [&](const Line2D &sideSurface,
                                     const Line2D &localBottom,
                                     bool localUsable,
                                     Eigen::Vector2f *intersection) {
      if (!localUsable || !intersection) {
        return;
      }
      Eigen::Vector2f localIntersection;
      if (!intersectLines(sideSurface, localBottom, &localIntersection)) {
        return;
      }
      const float difference = (localIntersection - *intersection).norm();
      if (difference > config.surfaceIntersectionTolerance) {
        return;
      }
      const float weight =
          0.15f * std::exp(-difference /
                           std::max(1.0f, config.surfaceIntersectionTolerance));
      *intersection =
          (1.0f - weight) * *intersection + weight * localIntersection;
    };
    refineWithLocalBottom(leftSurface, leftBottom, leftLocalUsable,
                          &leftIntersection);
    refineWithLocalBottom(rightSurface, rightBottom, rightLocalUsable,
                          &rightIntersection);
    leftBoundaryError =
        std::fabs(leftIntersection.x() -
                  profile[static_cast<std::size_t>(leftBoundary)].u);
    rightBoundaryError =
        std::fabs(rightIntersection.x() -
                  profile[static_cast<std::size_t>(rightBoundary)].u);
    if (leftBoundaryError > config.surfaceIntersectionTolerance ||
        rightBoundaryError > config.surfaceIntersectionTolerance ||
        rightIntersection.x() - leftIntersection.x() <
            config.minSeamSeparation) {
      countGate("intersection_or_separation");
      return false;
    }

    // 不比较绝对 Z。基准底面是当前站位的局部切线，点到切线的正交
    // 距离会随弧面姿态一起变化。
    leftClearance = std::fabs(broadBottom.normal.dot(leftSurface.center) -
                              broadBottom.offset);
    rightClearance = std::fabs(broadBottom.normal.dot(rightSurface.center) -
                               broadBottom.offset);
    if (leftClearance < config.minSurfaceClearance ||
        rightClearance < config.minSurfaceClearance) {
      countGate("clearance");
      return false;
    }

    return true;
  }

  void scoreGeometry() {
    residualSum = leftSurface.rms + broadBottom.rms + rightSurface.rms;
    const float angleQuality =
        clamp01(std::min(leftAngle, rightAngle) / (0.5f * kPi));
    const float residualQuality =
        std::exp(-residualSum / std::max(0.2f, 3.0f * config.surfaceRmsScale));
    const float bottomQuality =
        std::exp(-broadBottomSlope / std::max(0.1f, config.maxBottomSlope));
    const float boundaryQuality =
        std::exp(-(leftBoundaryError + rightBoundaryError) /
                 std::max(1.0f, 2.0f * config.surfaceIntersectionTolerance));
    feature = clamp01(0.40f * angleQuality + 0.35f * residualQuality +
                      0.15f * bottomQuality + 0.10f * boundaryQuality);
  }

  void recordSurfaceFits() {
    if (debug) {
      proposal.pair.debug = std::make_shared<DebugPair>();
      DebugPair &d = *proposal.pair.debug;
      auto line = [](const Line2D &l, int begin, int end) {
        DebugLine value;
        value.u = l.center.x();
        value.v = l.center.y();
        value.du = l.direction.x();
        value.dv = l.direction.y();
        value.rms = l.rms;
        value.begin = begin;
        value.end = end;
        return value;
      };
      d.lines = {
          {line(leftSurface, leftBoundary - guard - sideWindow,
                leftBoundary - guard),
           line(broadBottom, leftBoundary + guard, rightBoundary - guard),
           line(rightSurface, rightBoundary + guard,
                rightBoundary + guard + sideWindow)}};
      d.boundaries = {{leftBoundary, rightBoundary}};
      d.angles = {{leftAngle * 180 / kPi, rightAngle * 180 / kPi}};
      d.localAngles = {
          {leftLocalAngle * 180 / kPi, rightLocalAngle * 180 / kPi}};
      d.clearance = {{leftClearance, rightClearance}};
    }
  }

  void growRegionCandidates() {
    if (config.useRegionGrowingEdges) {
      // 底面从候选槽底中央的稳定小段向两侧生长；侧面则从原有大面
      // 拟合段向交界处生长。允许范围略越过原理论边界，使边界位置由
      // 局部法向和距离决定，而不是被枚举下标预先固定。
      const int groundBegin = leftBoundary + guard;
      const int groundEnd = rightBoundary - guard;
      const int seedWidth = std::max(
          3, std::min(scaledProfileCount(config, config.regionGrowingWindow,
                                         3) *
                          (config.useContourRegionGrowing ? 2 : 1),
                      std::max(3, groundEnd - groundBegin)));
      const int groundMiddle = (groundBegin + groundEnd) / 2;
      int groundSeedBegin = groundMiddle - seedWidth / 2;
      int groundSeedEnd = groundSeedBegin + seedWidth;
      if (groundSeedBegin < groundBegin) {
        groundSeedBegin = groundBegin;
        groundSeedEnd = groundSeedBegin + seedWidth;
      }
      if (groundSeedEnd > groundEnd) {
        groundSeedEnd = groundEnd;
        groundSeedBegin = groundSeedEnd - seedWidth;
      }

      const int leftSeedBegin = leftBoundary - guard - sideWindow;
      const int leftSeedEnd = leftBoundary - guard;
      const int rightSeedBegin = rightBoundary + guard;
      const int rightSeedEnd = rightBoundary + guard + sideWindow;
      const ProfileRegion groundRegion = growProfileRegion(
          profile, groundSeedBegin, groundSeedEnd,
          std::max(0, leftBoundary - sideWindow),
          std::min(pointCount, rightBoundary + sideWindow + 1), config,
          config.useContourRegionGrowing);
      const ProfileRegion leftRegion = growProfileRegion(
          profile, leftSeedBegin, leftSeedEnd, leftSeedBegin,
          std::min(pointCount, leftBoundary + guard + localWindow), config);
      const ProfileRegion rightRegion =
          growProfileRegion(profile, rightSeedBegin, rightSeedEnd,
                            std::max(0, rightBoundary - guard - localWindow),
                            rightSeedEnd, config);
      const RegionEdgePair leftEdges =
          matchRegionEdges(profile, leftRegion, groundRegion, true, config);
      const RegionEdgePair rightEdges =
          matchRegionEdges(profile, rightRegion, groundRegion, false, config);
      if (debug) {
        DebugPair &d = *proposal.pair.debug;
        d.regionAttempted = true;
        d.seeds = {{leftSeedBegin, leftSeedEnd, groundSeedBegin, groundSeedEnd,
                    rightSeedBegin, rightSeedEnd}};
        d.regions = {{leftRegion.begin, leftRegion.end, groundRegion.begin,
                      groundRegion.end, rightRegion.begin, rightRegion.end}};
        d.edges = {{leftEdges.sidePoint.x(), leftEdges.sidePoint.y(),
                    leftEdges.groundPoint.x(), leftEdges.groundPoint.y(),
                    rightEdges.sidePoint.x(), rightEdges.sidePoint.y(),
                    rightEdges.groundPoint.x(), rightEdges.groundPoint.y()}};
        d.edgeGaps = {{leftEdges.distance, rightEdges.distance}};
      }
      const bool allowIndependentEdges = config.useContourRegionGrowing;
      if ((leftEdges.valid && rightEdges.valid) ||
          (allowIndependentEdges && (leftEdges.valid || rightEdges.valid))) {
        const float grownLeftU =
            leftEdges.valid
                ? 0.5f * (leftEdges.sidePoint.x() + leftEdges.groundPoint.x())
                : leftIntersection.x();
        const float grownRightU =
            rightEdges.valid
                ? 0.5f * (rightEdges.sidePoint.x() + rightEdges.groundPoint.x())
                : rightIntersection.x();
        if (grownRightU - grownLeftU >= config.minSeamSeparation) {
          const float regionQuality =
              (groundRegion.quality + leftRegion.quality +
               rightRegion.quality) /
              3.0f;
          const float gapSum = (leftEdges.valid ? leftEdges.distance : 0.0f) +
                               (rightEdges.valid ? rightEdges.distance : 0.0f);
          const int edgeCount = static_cast<int>(leftEdges.valid) +
                                static_cast<int>(rightEdges.valid);
          const float gapQuality = std::exp(
              -gapSum / std::max(0.5f, static_cast<float>(edgeCount) *
                                           config.regionGrowingMaxEdgeGap));
          const float grownFeature = clamp01(
              0.70f * feature + 0.20f * regionQuality + 0.10f * gapQuality);
          // 配对路径仍联合约束左右间距，但可靠的一侧不应被另一侧的孔洞
          // 或生长失败牵连。只对失败的一侧回退到理论交点。
          proposal.pair.left =
              leftEdges.valid
                  ? candidateFromRegionEdges(profile, leftEdges,
                                             config.longitudinalAxis,
                                             grownFeature)
                  : candidateFromIntersection(profile, leftIntersection,
                                              config.longitudinalAxis, feature,
                                              profileGeometryScale(config));
          proposal.pair.right =
              rightEdges.valid
                  ? candidateFromRegionEdges(profile, rightEdges,
                                             config.longitudinalAxis,
                                             grownFeature)
                  : candidateFromIntersection(profile, rightIntersection,
                                              config.longitudinalAxis, feature,
                                              profileGeometryScale(config));
          regionCost = 0.15f * gapSum - 0.35f * (regionQuality + gapQuality) +
                       0.25f * static_cast<float>(2 - edgeCount);
          const int uAxis = transverseAxes(config.longitudinalAxis)[0];
          usedRegionEdges = component(proposal.pair.right.point, uAxis) -
                                component(proposal.pair.left.point, uAxis) >=
                            config.minSeamSeparation;
          usedLeftEdges = usedRegionEdges && leftEdges.valid;
          usedRightEdges = usedRegionEdges && rightEdges.valid;
          if (!usedRegionEdges)
            regionCost = 0.0f;
        }
      }
    }
    // 区域生长在遮挡、孔洞或极稀疏截面中无法形成可靠边缘时，保留
    // 原理论交点作为降级路径，避免整段轨迹因少数坏截面中断。
    if (!usedRegionEdges) {
      proposal.pair.left = candidateFromIntersection(
          profile, leftIntersection, config.longitudinalAxis, feature,
          profileGeometryScale(config));
      proposal.pair.right = candidateFromIntersection(
          profile, rightIntersection, config.longitudinalAxis, feature,
          profileGeometryScale(config));
      regionCost += config.useRegionGrowingEdges ? 0.5f : 0.0f;
    }
  }

  void refineContourCandidates() {
    if (config.useRegionGrowingEdges && config.useContourRegionGrowing) {
      const auto leftCorner =
          findLocalContourCorner(profile, leftSurface, broadBottom,
                                 leftIntersection, leftBoundary, true, config);
      const auto rightCorner = findLocalContourCorner(
          profile, rightSurface, broadBottom, rightIntersection, rightBoundary,
          false, config);
      Candidate localLeft = proposal.pair.left;
      Candidate localRight = proposal.pair.right;
      auto useCorner = [&](const LocalContourCorner &corner,
                           Candidate *candidate) {
        if (corner.index < 0)
          return;
        candidate->point =
            profile[static_cast<std::size_t>(corner.index)].point;
        candidate->feature = clamp01(0.85f * feature + 0.15f * corner.quality);
        candidate->fromRegionEdges = false;
        candidate->fromLocalCorner = true;
      };
      useCorner(leftCorner, &localLeft);
      useCorner(rightCorner, &localRight);
      const int uAxis = transverseAxes(config.longitudinalAxis)[0];
      if (component(localRight.point, uAxis) -
              component(localLeft.point, uAxis) >=
          config.minSeamSeparation) {
        proposal.pair.left = localLeft;
        proposal.pair.right = localRight;
        usedLocalCorner = leftCorner.index >= 0 || rightCorner.index >= 0;
        cornerIndices = {{leftCorner.index, rightCorner.index}};
        if (leftCorner.index >= 0)
          usedLeftEdges = false;
        if (rightCorner.index >= 0)
          usedRightEdges = false;
        if (debug) {
          proposal.pair.debug->localCornerIndices = {
              {leftCorner.index, rightCorner.index}};
          proposal.pair.debug->localCornerAngles = {
              {leftCorner.angle, rightCorner.angle}};
        }
      }
    }
  }

  void attachSurfaceConstraints() {
    // 将侧壁约束与候选一起传递到路径末端，不能在平滑后丢掉几何归属。
    auto attachWall = [&](Candidate &candidate, const Line2D &surface,
                          int corner, bool left) {
      if (!surface.valid)
        return;
      Line2D wall = surface;
      const int w = scaledProfileCount(
          config, std::min(5, config.localAngleWindow), 3);
      if (corner >= w && corner + w < pointCount) {
        const auto local =
            left ? fitOrthogonalLine(profile, corner - w, corner, true)
                 : fitOrthogonalLine(profile, corner + 1, corner + w + 1, true);
        if (local.valid && local.rms <= config.regionGrowingDistance &&
            acuteAngle(local, surface) <= 12.f * kPi / 180.f)
          wall = local;
      }
      if (wall.rms > config.regionGrowingDistance)
        return;
      candidate.wallNormal = wall.normal;
      candidate.wallOffset = wall.offset;
      candidate.wallRms = wall.rms;
      candidate.hasWall = true;
    };
    attachWall(proposal.pair.left, leftSurface, cornerIndices[0], true);
    attachWall(proposal.pair.right, rightSurface, cornerIndices[1], false);
    proposal.pair.left.floorDirection = proposal.pair.right.floorDirection =
        broadBottom.direction;
  }

  void finishProposal() {
    // 左右独立残差和底面方向一致性负责候选排序；全局优化随后再根据
    // 整条轨迹的连续性选择，不会让一侧的低质量观测污染另一侧置信度。
    proposal.cost =
        1.2f * residualSum + 0.08f * (leftBoundaryError + rightBoundaryError) -
        0.8f * (leftAngle + rightAngle) -
        0.02f * std::log1p(leftSurface.support * rightSurface.support) +
        regionCost;
    // broadBottom 始终朝 +U；左侧面向下转表示槽口朝 +V，向上转表示槽口朝 -V。
    proposal.pair.interiorOpeningDirection = leftTurn < 0.0f ? +1 : -1;
    proposal.pair.geometryCost = proposal.cost;
    countGate("accepted");
    if (debug) {
      proposal.pair.debug->proposalCost = proposal.cost;
      proposal.pair.debug->source =
          usedLocalCorner
              ? "local_contour"
              : (usedLeftEdges && usedRightEdges
                     ? "region_edges"
                     : (usedRegionEdges ? "mixed_edges" : "intersection"));
      proposal.pair.debug->regionUsed = {{usedLeftEdges, usedRightEdges}};
    }
  }

  const std::vector<ProfilePoint> &profile;
  const DetectorConfig &config;
  const int leftBoundary, rightBoundary;
  DebugSlice *debug;
  const int sideWindow, guard, localWindow, pointCount;
  Line2D leftSurface, broadBottom, rightSurface, leftBottom, rightBottom;
  bool leftLocalUsable = false, rightLocalUsable = false;
  float broadBottomSlope = 0, leftAngle = 0, rightAngle = 0;
  float leftLocalAngle = 0, rightLocalAngle = 0;
  float leftTurn = 0;
  Eigen::Vector2f leftIntersection, rightIntersection;
  float leftBoundaryError = 0, rightBoundaryError = 0;
  float leftClearance = 0, rightClearance = 0, residualSum = 0, feature = 0;
  float regionCost = 0;
  bool usedRegionEdges = false, usedLeftEdges = false, usedRightEdges = false;
  bool usedLocalCorner = false;
  std::array<int, 2> cornerIndices{{-1, -1}};
  SurfaceProposal proposal;
};

std::vector<SurfacePair>
findSurfacePairs(const std::vector<ProfilePoint> &profile,
                 const DetectorConfig &config,
                 const std::array<HistoryPrediction, 2> &predictions,
                 bool *hasGeometry, DebugSlice *debug) {
  std::vector<SurfaceProposal> proposals;
  const int sideWindow =
      scaledProfileCount(config, config.fitHalfWindow, 5);
  const int guard =
      scaledProfileCount(config, config.fitGuardPoints, 1, 1);
  const int pointCount = static_cast<int>(profile.size());
  const int minimumBoundaryGap =
      std::max(8, static_cast<int>(0.70f * config.minSeamSeparation /
                                   std::max(0.05f, config.profileBinSize)));
  const bool enoughForSurfaceModel =
      pointCount >= 2 * (sideWindow + guard) + minimumBoundaryGap;
  if (!enoughForSurfaceModel && debug)
    ++debug->gates["profile_insufficient"];
  if (enoughForSurfaceModel) {
    for (int leftBoundary = sideWindow + guard;
         leftBoundary + minimumBoundaryGap + sideWindow + guard < pointCount;
         ++leftBoundary) {
      for (int rightBoundary = leftBoundary + minimumBoundaryGap;
           rightBoundary + guard + sideWindow <= pointCount; ++rightBoundary) {
        if (debug)
          ++debug->gates["enumerated"];
        SurfaceProposal proposal;
        if (SurfacePairBuilder(profile, config, leftBoundary, rightBoundary,
                               debug)
                .build(proposal) &&
            (config.interiorOpeningMode == InteriorOpeningMode::Auto ||
             proposal.pair.interiorOpeningDirection ==
                 static_cast<int>(config.interiorOpeningMode)))
          proposals.push_back(std::move(proposal));
      }
    }
  }
  std::sort(proposals.begin(), proposals.end(),
            [](const SurfaceProposal &lhs, const SurfaceProposal &rhs) {
              return lhs.cost < rhs.cost;
            });

  std::vector<SurfacePair> result;
  const int maximumPairs = std::max(2, config.maxCandidatesPerSlice / 2);
  int positivePairs = 0, negativePairs = 0;
  for (const SurfaceProposal &proposal : proposals) {
    if ((proposal.pair.interiorOpeningDirection > 0 &&
         positivePairs >= maximumPairs) ||
        (proposal.pair.interiorOpeningDirection < 0 &&
         negativePairs >= maximumPairs))
      continue;
    bool duplicate = false;
    for (const SurfacePair &existing : result) {
      if (transverseDistance(proposal.pair.left.point, existing.left.point,
                             config.longitudinalAxis) <
              config.minCandidateSpacing &&
          transverseDistance(proposal.pair.right.point, existing.right.point,
                             config.longitudinalAxis) <
              config.minCandidateSpacing) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      result.push_back(proposal.pair);
      if (proposal.pair.interiorOpeningDirection > 0) ++positivePairs;
      else ++negativePairs;
    }
    if ((config.interiorOpeningMode != InteriorOpeningMode::Auto &&
         static_cast<int>(result.size()) >= maximumPairs) ||
        (config.interiorOpeningMode == InteriorOpeningMode::Auto &&
         positivePairs >= maximumPairs && negativePairs >= maximumPairs)) {
      break;
    }
  }

  if (hasGeometry) {
    *hasGeometry = !result.empty();
  }

  // 焊到高层后，三面交角可能已经被焊道完全填圆。此时加入一对历史投影
  // 作为低特征候选，后续全局优化会在“当前三面模型”和“历史趋势”之间选择。
  if (predictions[0].valid && predictions[1].valid) {
    SurfacePair historyPair;
    historyPair.left = nearestHistoryCandidate(profile, predictions[0], config);
    historyPair.right =
        nearestHistoryCandidate(profile, predictions[1], config);
    const int sideAxis = transverseAxes(config.longitudinalAxis)[0];
    if (component(historyPair.right.point, sideAxis) -
            component(historyPair.left.point, sideAxis) >=
        config.minSeamSeparation) {
      bool duplicate = false;
      for (const SurfacePair &existing : result) {
        if (transverseDistance(historyPair.left.point, existing.left.point,
                               config.longitudinalAxis) <
                config.minCandidateSpacing &&
            transverseDistance(historyPair.right.point, existing.right.point,
                               config.longitudinalAxis) <
                config.minCandidateSpacing) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        if (debug) {
          historyPair.debug = std::make_shared<DebugPair>();
          historyPair.debug->source = "history";
        }
        result.push_back(historyPair);
      }
    }
  }
  return result;
}

struct InteriorSelection {
  int direction = 0;
  std::size_t positiveStations = 0, negativeStations = 0;
  bool ambiguous = false;
};

InteriorSelection selectInteriorCandidates(std::vector<Slice> &slices,
                                           const DetectorConfig &config) {
  InteriorSelection selection;
  double cost[2] = {0.0, 0.0};
  std::size_t costCount[2] = {0, 0};
  for (const auto &slice : slices) {
    for (int sign : {-1, +1}) {
      float best = std::numeric_limits<float>::infinity();
      for (const auto &pair : slice.pairs)
        if (pair.interiorOpeningDirection == sign)
          best = std::min(best, pair.geometryCost);
      if (!std::isfinite(best)) continue;
      if (sign > 0) ++selection.positiveStations;
      else ++selection.negativeStations;
      const int index = sign > 0 ? 1 : 0;
      cost[index] += best; ++costCount[index];
    }
  }

  if (config.interiorOpeningMode != InteriorOpeningMode::Auto) {
    selection.direction = static_cast<int>(config.interiorOpeningMode);
  } else {
    const auto positive = selection.positiveStations;
    const auto negative = selection.negativeStations;
    const double positiveMean = costCount[1] ? cost[1] / double(costCount[1])
                                             : std::numeric_limits<double>::infinity();
    const double negativeMean = costCount[0] ? cost[0] / double(costCount[0])
                                             : std::numeric_limits<double>::infinity();
    if (positive == 0 && negative == 0) return selection;
    if (positive != negative) selection.direction = positive > negative ? +1 : -1;
    else selection.direction = positiveMean <= negativeMean ? +1 : -1;
    const std::size_t major = std::max(positive, negative);
    const std::size_t minor = std::min(positive, negative);
    const double scale = std::max(1.0, std::abs(positiveMean) + std::abs(negativeMean));
    selection.ambiguous = major > 0 && minor * 5 >= major * 4 &&
                          std::isfinite(positiveMean) && std::isfinite(negativeMean) &&
                          std::abs(positiveMean - negativeMean) <= 0.15 * scale;
  }

  for (auto &slice : slices)
    slice.pairs.erase(
        std::remove_if(slice.pairs.begin(), slice.pairs.end(), [&](const SurfacePair &pair) {
          return pair.interiorOpeningDirection != 0 &&
                 pair.interiorOpeningDirection != selection.direction;
        }),
        slice.pairs.end());
  slices.erase(std::remove_if(slices.begin(), slices.end(),
                              [](const Slice &slice) { return slice.pairs.empty(); }),
               slices.end());
  return selection;
}

bool validateDetectionInput(const CloudT::ConstPtr &input,
                            const DetectorConfig &config, std::string &error) {
  // S00 输入与参数检查。失败通过 error 返回，不产生虚构轨迹。
  if (!input || input->size() < 30) {
    error = "Point cloud has fewer than 30 valid points.";
    return false;
  }
  if (config.sliceStep <= 0.0f || config.sliceThickness <= 0.0f ||
      config.profileBinSize <= 0.0f ||
      (stableProfileSampling(config) &&
       (!std::isfinite(config.profileReferenceScale) ||
        config.profileReferenceScale <= 0.0f))) {
    error = "Slice and profile dimensions must be positive.";
    return false;
  }
  if (!std::isfinite(config.hullEnvelopeGridSize) ||
      config.hullEnvelopeGridSize <= 0.0f ||
      config.hullEnvelopeMinCellSupport < 1 ||
      !std::isfinite(config.hullEnvelopeMinThickness) ||
      config.hullEnvelopeMinThickness <= 0.0f ||
      !std::isfinite(config.hullEnvelopeMargin) ||
      config.hullEnvelopeMargin < 0.0f ||
      config.hullEnvelopeSmoothingRadius < 0) {
    error = "Hull-envelope crop parameters are invalid.";
    return false;
  }
  if (!std::isfinite(config.floorPatchSize) || config.floorPatchSize <= 0 ||
      !std::isfinite(config.floorPatchGuard) || config.floorPatchGuard < 0 ||
      !std::isfinite(config.floorPatchMaxRms) || config.floorPatchMaxRms <= 0 ||
      !std::isfinite(config.floorPatchMaxCorrection) ||
      config.floorPatchMaxCorrection < 0) {
    error = "Local floor patch parameters are invalid.";
    return false;
  }
  if (config.useRegionGrowingEdges &&
      (config.regionGrowingWindow < 3 ||
       config.regionGrowingSmoothnessDegrees <= 0.0f ||
       config.regionGrowingMaxNormalDriftDegrees <= 0.0f ||
       config.regionGrowingDistance <= 0.0f ||
       config.regionGrowingMaxEdgeGap <= 0.0f)) {
    error = "Region-growing parameters must be positive.";
    return false;
  }
  return true;
}

struct StationRange {
  float minimum;
  float maximum;
};

StationRange findStationRange(const CloudT &cloud, Axis axis) {
  float minimum = std::numeric_limits<float>::max();
  float maximum = std::numeric_limits<float>::lowest();
  for (const PointT &point : cloud) {
    const float station = coordinate(point, axis);
    minimum = std::min(minimum, station);
    maximum = std::max(maximum, station);
  }
  return {minimum, maximum};
}

void setComponent(PointT &point, int axis, float value) {
  if (axis == 0)
    point.x = value;
  else if (axis == 1)
    point.y = value;
  else
    point.z = value;
}

struct HullEnvelopeColumn {
  int low = -1;
  int high = -1;
  int length = 0;
};

std::vector<int> medianSmooth(const std::vector<int> &values, int radius) {
  if (radius <= 0 || values.size() < 3)
    return values;
  std::vector<int> result(values.size());
  std::vector<int> window;
  window.reserve(static_cast<std::size_t>(2 * radius + 1));
  for (std::size_t i = 0; i < values.size(); ++i) {
    const std::size_t begin = i > static_cast<std::size_t>(radius)
                                  ? i - static_cast<std::size_t>(radius)
                                  : 0;
    const std::size_t end =
        std::min(values.size(), i + static_cast<std::size_t>(radius) + 1);
    window.assign(values.begin() + static_cast<std::ptrdiff_t>(begin),
                  values.begin() + static_cast<std::ptrdiff_t>(end));
    const auto middle = window.begin() +
                        static_cast<std::ptrdiff_t>(window.size() / 2);
    std::nth_element(window.begin(), middle, window.end());
    result[i] = *middle;
  }
  return result;
}

CloudT::Ptr cropToHullEnvelope(const CloudT::ConstPtr &input,
                               const DetectorConfig &config,
                               DebugTrace *debug,
                               std::vector<ScanFrameRange> *frames,
                               SeamResult &result) {
  result.hullEnvelopeRequested = config.useHullEnvelopeCrop;
  result.hullEnvelopeInputPoints = input ? input->size() : 0;
  result.hullEnvelopeRetainedPoints = result.hullEnvelopeInputPoints;
  result.hullEnvelopeColumns = 0;
  result.hullEnvelopeApplied = false;

  if (!input || input->empty()) {
    result.hullEnvelopeStatus = "empty_input";
    return CloudT::Ptr(new CloudT);
  }
  if (!config.useHullEnvelopeCrop) {
    result.hullEnvelopeStatus = "disabled";
    if (debug)
      publishDebugStep(debug, "S04 整船外壁包络裁剪（已关闭）",
                       {{"保留点云", input}});
    return CloudT::Ptr(new CloudT(*input));
  }

  const int longitudinal = static_cast<int>(config.longitudinalAxis);
  const auto transverse = transverseAxes(config.longitudinalAxis);
  const int vertical = transverse[1];
  const int lateral = transverse[0];
  float minimumA = std::numeric_limits<float>::max();
  float maximumA = std::numeric_limits<float>::lowest();
  float minimumV = std::numeric_limits<float>::max();
  float maximumV = std::numeric_limits<float>::lowest();
  float minimumU = std::numeric_limits<float>::max();
  float maximumU = std::numeric_limits<float>::lowest();
  for (const auto &point : *input) {
    checkCancellation();
    minimumA = std::min(minimumA, component(point, longitudinal));
    maximumA = std::max(maximumA, component(point, longitudinal));
    minimumV = std::min(minimumV, component(point, vertical));
    maximumV = std::max(maximumV, component(point, vertical));
    minimumU = std::min(minimumU, component(point, lateral));
    maximumU = std::max(maximumU, component(point, lateral));
  }
  const float spanA = maximumA - minimumA;
  const float spanV = maximumV - minimumV;
  float cellSize = std::max(1.0e-4f, config.hullEnvelopeGridSize);
  constexpr std::size_t maximumCells = 4u * 1024u * 1024u;
  auto dimensions = [&](float size) {
    return std::array<std::size_t, 2>{
        static_cast<std::size_t>(std::floor(spanA / size)) + 1,
        static_cast<std::size_t>(std::floor(spanV / size)) + 1};
  };
  auto dims = dimensions(cellSize);
  if (dims[0] == 0 || dims[1] == 0 || spanA <= 0.0f || spanV <= 0.0f) {
    result.hullEnvelopeStatus = "degenerate_projection";
    if (debug)
      publishDebugStep(debug, "S04 整船外壁包络裁剪（未应用）",
                       {{"保留点云", input}},
                       "纵向—竖向投影退化，保留预处理点云。 ");
    return CloudT::Ptr(new CloudT(*input));
  }
  if (dims[0] > maximumCells / dims[1]) {
    cellSize = std::max(
        cellSize,
        std::sqrt((spanA * spanV) / static_cast<float>(maximumCells)));
    dims = dimensions(cellSize);
  }
  const std::size_t columns = dims[0];
  const std::size_t rows = dims[1];
  if (columns < 3 || rows < 3 || columns > maximumCells / rows) {
    result.hullEnvelopeStatus = "projection_grid_unavailable";
    if (debug)
      publishDebugStep(debug, "S04 整船外壁包络裁剪（未应用）",
                       {{"保留点云", input}},
                       "投影网格不足以形成外壁包络，保留预处理点云。 ");
    return CloudT::Ptr(new CloudT(*input));
  }

  std::vector<std::uint32_t> support(columns * rows, 0);
  auto cellIndex = [&](const PointT &point) {
    const auto a = std::min(
        columns - 1,
        static_cast<std::size_t>(std::max(
            0.0f, std::floor((component(point, longitudinal) - minimumA) /
                              cellSize))));
    const auto v = std::min(
        rows - 1,
        static_cast<std::size_t>(std::max(
            0.0f, std::floor((component(point, vertical) - minimumV) /
                              cellSize))));
    return std::array<std::size_t, 2>{a, v};
  };
  for (const auto &point : *input) {
    checkCancellation();
    const auto cell = cellIndex(point);
    auto &count = support[cell[0] * rows + cell[1]];
    if (count != std::numeric_limits<std::uint32_t>::max())
      ++count;
  }

  const std::uint32_t minimumSupport = static_cast<std::uint32_t>(
      std::max(1, config.hullEnvelopeMinCellSupport));
  std::vector<unsigned char> occupied(columns * rows, 0);
  for (std::size_t a = 0; a < columns; ++a) {
    for (std::size_t v = 0; v < rows; ++v) {
      const auto index = a * rows + v;
      const bool direct = support[index] >= minimumSupport;
      const bool oneCellGap =
          v > 0 && v + 1 < rows &&
          support[index - 1] >= minimumSupport &&
          support[index + 1] >= minimumSupport;
      occupied[index] = static_cast<unsigned char>(direct || oneCellGap);
    }
  }

  // 同一纵向列里的外壁与底部可能因遮挡形成多个竖向小段，不能只取
  // 最长小段，否则会把真实外壁的一部分裁掉。先在二维投影上找最大的
  // 8 邻域连通体，再以该连通体每列的最小/最大 V 作为闭合包络。
  std::vector<int> labels(columns * rows, -1);
  std::vector<std::size_t> stack;
  std::size_t largestSize = 0;
  int largestLabel = -1;
  int nextLabel = 0;
  for (std::size_t seed = 0; seed < occupied.size(); ++seed) {
    if (!occupied[seed] || labels[seed] >= 0)
      continue;
    stack.clear();
    stack.push_back(seed);
    labels[seed] = nextLabel;
    std::size_t componentSize = 0;
    while (!stack.empty()) {
      const auto index = stack.back();
      stack.pop_back();
      ++componentSize;
      const std::size_t a = index / rows;
      const std::size_t v = index % rows;
      for (int da = -1; da <= 1; ++da) {
        for (int dv = -1; dv <= 1; ++dv) {
          if ((da == 0 && dv == 0) ||
              (da < 0 && a == 0) || (da > 0 && a + 1 >= columns) ||
              (dv < 0 && v == 0) || (dv > 0 && v + 1 >= rows))
            continue;
          const auto neighbor =
              static_cast<std::size_t>(static_cast<std::ptrdiff_t>(index) +
                                       da * static_cast<std::ptrdiff_t>(rows) +
                                       dv);
          if (occupied[neighbor] && labels[neighbor] < 0) {
            labels[neighbor] = nextLabel;
            stack.push_back(neighbor);
          }
        }
      }
    }
    if (componentSize > largestSize) {
      largestSize = componentSize;
      largestLabel = nextLabel;
    }
    ++nextLabel;
    if ((static_cast<std::size_t>(nextLabel) & 31u) == 0)
      checkCancellation();
  }

  std::vector<HullEnvelopeColumn> envelope(columns);
  if (largestLabel >= 0) {
    for (std::size_t a = 0; a < columns; ++a) {
      for (std::size_t v = 0; v < rows; ++v) {
        if (labels[a * rows + v] != largestLabel)
          continue;
        if (envelope[a].low < 0)
          envelope[a].low = static_cast<int>(v);
        envelope[a].high = static_cast<int>(v);
      }
      if (envelope[a].low >= 0)
        envelope[a].length = envelope[a].high - envelope[a].low + 1;
    }
  }

  const int minimumThicknessBins = std::max(
      2, static_cast<int>(std::ceil(config.hullEnvelopeMinThickness /
                                    cellSize)));
  std::vector<unsigned char> substantial(columns, 0);
  for (std::size_t a = 0; a < columns; ++a)
    substantial[a] =
        static_cast<unsigned char>(envelope[a].length >= minimumThicknessBins);
  // 填补最多两列的局部投影缺口，但不跨越长距离缺测连接两个物体。
  for (std::size_t begin = 0; begin < columns;) {
    if (substantial[begin]) {
      ++begin;
      continue;
    }
    std::size_t end = begin;
    while (end < columns && !substantial[end])
      ++end;
    if (begin > 0 && end < columns && end - begin <= 2)
      std::fill(substantial.begin() + static_cast<std::ptrdiff_t>(begin),
                substantial.begin() + static_cast<std::ptrdiff_t>(end), 1);
    begin = end;
  }

  std::size_t mainBegin = 0, mainEnd = 0;
  for (std::size_t begin = 0; begin < columns;) {
    while (begin < columns && !substantial[begin])
      ++begin;
    std::size_t end = begin;
    while (end < columns && substantial[end])
      ++end;
    if (end - begin > mainEnd - mainBegin) {
      mainBegin = begin;
      mainEnd = end;
    }
    begin = end + (end == begin && end < columns ? 1 : 0);
  }
  const float mainSpan = static_cast<float>(mainEnd - mainBegin) * cellSize;
  const float minimumMainSpan = std::max(6.0f * cellSize, 0.2f * spanA);
  if (mainEnd <= mainBegin || mainSpan < minimumMainSpan) {
    result.hullEnvelopeStatus = "continuous_body_not_found";
    if (debug)
      publishDebugStep(debug, "S04 整船外壁包络裁剪（未应用）",
                       {{"保留点云", input}},
                       "没有找到同时满足投影厚度和纵向连续性的主体，保留预处理点云。",
                       {{"grid_size", cellSize},
                        {"minimum_cell_support", double(minimumSupport)}});
    return CloudT::Ptr(new CloudT(*input));
  }

  // 两列以内的缺口用相邻列包络插值；随后分别对上外壁与底部外壁做
  // 中值平滑，避免细长噪声与船体交叉时把单个尖刺带入裁剪边界。
  for (std::size_t a = mainBegin; a < mainEnd; ++a) {
    if (envelope[a].length > 0)
      continue;
    std::size_t left = a, right = a;
    while (left > mainBegin && envelope[left].length == 0)
      --left;
    while (right + 1 < mainEnd && envelope[right].length == 0)
      ++right;
    if (envelope[left].length > 0 && envelope[right].length > 0) {
      envelope[a].low = (envelope[left].low + envelope[right].low) / 2;
      envelope[a].high = (envelope[left].high + envelope[right].high) / 2;
      envelope[a].length = envelope[a].high - envelope[a].low + 1;
    }
  }
  std::vector<int> lower, upper;
  lower.reserve(mainEnd - mainBegin);
  upper.reserve(mainEnd - mainBegin);
  for (std::size_t a = mainBegin; a < mainEnd; ++a) {
    lower.push_back(envelope[a].low);
    upper.push_back(envelope[a].high);
  }
  lower = medianSmooth(lower, config.hullEnvelopeSmoothingRadius);
  upper = medianSmooth(upper, config.hullEnvelopeSmoothingRadius);
  const int marginBins = std::max(
      0, static_cast<int>(std::ceil(config.hullEnvelopeMargin / cellSize)));

  CloudT::Ptr cropped(new CloudT);
  cropped->reserve(input->size());
  std::vector<std::size_t> keptPerFrame(frames ? frames->size() : 0, 0);
  std::size_t frameCursor = 0;
  for (std::size_t i = 0; i < input->size(); ++i) {
    checkCancellation();
    while (frames && frameCursor + 1 < frames->size() &&
           i >= (*frames)[frameCursor].begin + (*frames)[frameCursor].count)
      ++frameCursor;
    const auto cell = cellIndex((*input)[i]);
    bool keep = false;
    if (cell[0] >= mainBegin && cell[0] < mainEnd) {
      const auto local = cell[0] - mainBegin;
      keep = static_cast<int>(cell[1]) >= lower[local] - marginBins &&
             static_cast<int>(cell[1]) <= upper[local] + marginBins;
    }
    if (keep) {
      cropped->push_back((*input)[i]);
      if (frames && frameCursor < keptPerFrame.size())
        ++keptPerFrame[frameCursor];
    }
  }
  const std::size_t minimumRetained =
      std::max<std::size_t>(30, input->size() / 5);
  if (cropped->size() < minimumRetained) {
    result.hullEnvelopeStatus = "retained_support_too_small";
    if (debug)
      publishDebugStep(debug, "S04 整船外壁包络裁剪（未应用）",
                       {{"保留点云", input}},
                       "候选包络保留的支持量过少，已回退为预处理点云。",
                       {{"candidate_retained_points", double(cropped->size())},
                        {"input_points", double(input->size())}});
    return CloudT::Ptr(new CloudT(*input));
  }

  cropped->width = static_cast<std::uint32_t>(cropped->size());
  cropped->height = 1;
  cropped->is_dense = true;
  if (frames) {
    std::vector<ScanFrameRange> remapped;
    remapped.reserve(frames->size());
    std::size_t offset = 0;
    for (std::size_t i = 0; i < frames->size(); ++i) {
      if (keptPerFrame[i] == 0)
        continue;
      remapped.push_back({offset, keptPerFrame[i], (*frames)[i].frameIndex});
      offset += keptPerFrame[i];
    }
    *frames = std::move(remapped);
  }

  result.hullEnvelopeApplied = true;
  result.hullEnvelopeRetainedPoints = cropped->size();
  result.hullEnvelopeColumns = mainEnd - mainBegin;
  result.hullEnvelopeStatus = "applied";
  if (debug) {
    CloudT::Ptr bottom(new CloudT), top(new CloudT);
    bottom->reserve(mainEnd - mainBegin);
    top->reserve(mainEnd - mainBegin);
    const float lateralCenter = 0.5f * (minimumU + maximumU);
    for (std::size_t a = mainBegin; a < mainEnd; ++a) {
      PointT low, high;
      const auto local = a - mainBegin;
      setComponent(low, longitudinal,
                   minimumA + (static_cast<float>(a) + 0.5f) * cellSize);
      setComponent(high, longitudinal,
                   minimumA + (static_cast<float>(a) + 0.5f) * cellSize);
      setComponent(low, lateral, lateralCenter);
      setComponent(high, lateral, lateralCenter);
      setComponent(low, vertical,
                   minimumV + (static_cast<float>(lower[local]) + 0.5f) *
                                  cellSize);
      setComponent(high, vertical,
                   minimumV + (static_cast<float>(upper[local]) + 0.5f) *
                                  cellSize);
      bottom->push_back(low);
      top->push_back(high);
    }
    publishDebugStep(
        debug, "S04 整船外壁包络裁剪",
        {{"包络内点云", cropped}, {"底部外壁", bottom}, {"上部外壁", top}},
        "沿整船上外壁与底部外壁形成的闭合包络裁剪；包络是当前输入的统计边界，不代表人工真值。",
        {{"input_points", double(input->size())},
         {"retained_points", double(cropped->size())},
         {"removed_points", double(input->size() - cropped->size())},
         {"envelope_columns", double(mainEnd - mainBegin)},
         {"grid_size", cellSize},
         {"minimum_cell_support", double(minimumSupport)}});
  }
  return cropped;
}

float findProfileGridOrigin(const CloudT &cloud, Axis longitudinal) {
  const int uAxis = transverseAxes(longitudinal)[0];
  float minimum = std::numeric_limits<float>::max();
  for (const PointT &point : cloud)
    if (pcl::isFinite(point))
      minimum = std::min(minimum, component(point, uAxis));
  return std::isfinite(minimum) ? minimum : 0.0f;
}

struct FrameProjection {
  bool orthonormal = false;
  Eigen::Vector3f across = Eigen::Vector3f::UnitY(), vertical = Eigen::Vector3f::UnitZ();
  int frameIndex = -1;
  float station = 0.0f;
  float fallbackLongitudinal = 0.0f;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::UnitX();
};

CloudT::Ptr preprocessOneFrame(const CloudT::Ptr &frame,
                               const DetectorConfig &config) {
  if (config.protectedHullInput) return frame;
#ifdef WELD_NO_PCL_FILTERS
  (void)config;
  return frame;
#else
  CloudT::Ptr denoised(new CloudT);
  if (config.outlierMeanK >= 3 &&
      frame->size() > static_cast<std::size_t>(config.outlierMeanK)) {
    pcl::StatisticalOutlierRemoval<PointT> filter;
    filter.setInputCloud(frame);
    filter.setMeanK(config.outlierMeanK);
    filter.setStddevMulThresh(config.outlierStddev);
    filter.filter(*denoised);
  } else {
    *denoised = *frame;
  }
  if (config.voxelSize <= 0.0f)
    return denoised;
  CloudT::Ptr downsampled(new CloudT);
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(denoised);
  voxel.setLeafSize(config.voxelSize, config.voxelSize, config.voxelSize);
  voxel.filter(*downsampled);
  return downsampled;
#endif
}

bool prepareFrameSections(const CloudT::ConstPtr &input,
                          const std::vector<ScanFrameRange> &frames,
                          const DetectorConfig &config,
                          CloudT::Ptr &flattened,
                          std::vector<ScanFrameRange> &processedFrames,
                          std::vector<FrameProjection> &projections,
                          float &frameStep, std::string &error) {
  if (!input || frames.empty()) {
    error = "No original frames were supplied.";
    return false;
  }
  std::size_t expected = 0;
  const Axis axis = config.longitudinalAxis;
  int previousId = -1;
  std::vector<float> rawStations;
  rawStations.reserve(frames.size());
  for (const auto &frame : frames) {
    checkCancellation();
    if (frame.begin != expected || frame.count == 0 ||
        frame.count > input->size() - expected ||
        frame.frameIndex <= previousId) {
      error = "Original frame ranges are incomplete or unordered.";
      return false;
    }
    std::vector<float> values;
    values.reserve(frame.count);
    for (std::size_t i = frame.begin; i < frame.begin + frame.count; ++i) {
      if (!pcl::isFinite((*input)[i])) {
        error = "An original frame contains a non-finite point.";
        return false;
      }
      values.push_back(coordinate((*input)[i], axis));
    }
    rawStations.push_back(median(std::move(values)));
    expected += frame.count;
    previousId = frame.frameIndex;
  }
  if (expected != input->size()) {
    error = "Original frames do not cover the complete point cloud.";
    return false;
  }
  std::vector<float> pitches;
  for (std::size_t i = 1; i < frames.size(); ++i) {
    const int idGap = frames[i].frameIndex - frames[i - 1].frameIndex;
    const float pitch =
        std::fabs(rawStations[i] - rawStations[i - 1]) / float(idGap);
    if (std::isfinite(pitch) && pitch > 1.0e-4f)
      pitches.push_back(pitch);
  }
  frameStep = pitches.empty() ? 1.0f : median(std::move(pitches));
  flattened.reset(new CloudT);
  flattened->reserve(input->size());
  processedFrames.reserve(frames.size());
  projections.reserve(frames.size());
  const int a = static_cast<int>(axis);
  const int firstId = frames.front().frameIndex;
  for (std::size_t f = 0; f < frames.size(); ++f) {
    const auto &range = frames[f];
    FrameProjection projection;
    projection.frameIndex = range.frameIndex;
    projection.station = rawStations.front() +
                         frameStep * float(range.frameIndex - firstId);
    projection.fallbackLongitudinal = rawStations[f];
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (std::size_t i = range.begin; i < range.begin + range.count; ++i)
      center += toVector((*input)[i]).cast<double>();
    center /= double(range.count);
    projection.center = center.cast<float>();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    CloudT::Ptr rawFrame(new CloudT);
    rawFrame->reserve(range.count);
    for (std::size_t i = range.begin; i < range.begin + range.count; ++i) {
      const auto delta =
          (toVector((*input)[i]).cast<double>() - center).eval();
      covariance += delta * delta.transpose();
      rawFrame->push_back((*input)[i]);
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
        covariance / double(range.count));
    if (solver.info() == Eigen::Success)
      projection.normal = solver.eigenvectors().col(0).cast<float>();
    if (config.protectedHullInput) {
      const auto axes = transverseAxes(axis);
      const auto reference = projections.empty() ? Eigen::Vector3f::Unit(a).eval() : projections.back().normal;
      if (projection.normal.dot(reference) < 0) projection.normal = -projection.normal;
      projection.across = Eigen::Vector3f::Unit(axes[0]);
      projection.across -= projection.normal * projection.normal.dot(projection.across);
      if (projection.across.norm() < .1f) { error = "Scan plane cannot resolve transverse axis."; return false; }
      projection.across.normalize();
      projection.vertical = projection.normal.cross(projection.across).normalized();
      if (a == 1) projection.vertical = -projection.vertical;
      projection.orthonormal = true;
    }
    CloudT::Ptr cleaned = preprocessOneFrame(rawFrame, config);
    if (!cleaned || cleaned->empty()) {
      error = "A frame became empty during preprocessing.";
      return false;
    }
    ScanFrameRange processed;
    processed.begin = flattened->size();
    processed.count = cleaned->size();
    processed.frameIndex = range.frameIndex;
    for (PointT point : *cleaned) {
      if (projection.orthonormal) {
        const auto axes = transverseAxes(axis);
        const Eigen::Vector3f delta = toVector(point) - projection.center;
        point.data[axes[0]] = delta.dot(projection.across) + projection.center[axes[0]];
        point.data[axes[1]] = delta.dot(projection.vertical);
      }
      point.data[a] = projection.station;
      flattened->push_back(point);
    }
    processedFrames.push_back(processed);
    projections.push_back(projection);
  }
  flattened->width = static_cast<std::uint32_t>(flattened->size());
  flattened->height = 1;
  flattened->is_dense = true;
  return true;
}

void restoreOriginalFrameGeometry(SeamResult &result,
                                  const std::vector<int> &frameIds,
                                  const std::vector<FrameProjection> &planes,
                                  Axis axis) {
  result.leftFrameIndices.clear();
  result.rightFrameIndices.clear();
  const int a = static_cast<int>(axis);
  auto restore = [&](CloudT &cloud, std::vector<int> &ids) {
    const std::size_t count = std::min(cloud.size(), frameIds.size());
    ids.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const int frameId = frameIds[i];
      ids.push_back(frameId);
      const auto found = std::find_if(
          planes.begin(), planes.end(),
          [frameId](const FrameProjection &p) { return p.frameIndex == frameId; });
      if (found == planes.end())
        continue;
      if (found->orthonormal) {
        const auto axes = transverseAxes(axis);
        cloud[i] = toPoint(found->center + found->across * (cloud[i].data[axes[0]] - found->center[axes[0]]) + found->vertical * cloud[i].data[axes[1]]);
        continue;
      }
      float rhs = found->normal.dot(found->center);
      for (int component = 0; component < 3; ++component)
        if (component != a)
          rhs -= found->normal[component] * cloud[i].data[component];
      cloud[i].data[a] =
          std::fabs(found->normal[a]) > 1.0e-4f
              ? rhs / found->normal[a]
              : found->fallbackLongitudinal;
    }
  };
  restore(*result.leftSeam, result.leftFrameIndices);
  restore(*result.rightSeam, result.rightFrameIndices);
}

void restoreFrameCloudGeometry(CloudT &cloud,
                               const std::vector<ScanFrameRange> &frames,
                               const std::vector<FrameProjection> &planes,
                               Axis axis) {
  const int a = static_cast<int>(axis);
  for (const auto &frame : frames) {
    const auto found = std::find_if(
        planes.begin(), planes.end(), [&](const FrameProjection &projection) {
          return projection.frameIndex == frame.frameIndex;
        });
    if (found == planes.end())
      continue;
    const std::size_t end =
        std::min(cloud.size(), frame.begin + frame.count);
    for (std::size_t i = frame.begin; i < end; ++i) {
      if (found->orthonormal) {
        const auto axes = transverseAxes(axis);
        cloud[i] = toPoint(found->center + found->across * (cloud[i].data[axes[0]] - found->center[axes[0]]) + found->vertical * cloud[i].data[axes[1]]);
        continue;
      }
      float rhs = found->normal.dot(found->center);
      for (int componentIndex = 0; componentIndex < 3; ++componentIndex)
        if (componentIndex != a)
          rhs -= found->normal[componentIndex] * cloud[i].data[componentIndex];
      cloud[i].data[a] =
          std::fabs(found->normal[a]) > 1.0e-4f
              ? rhs / found->normal[a]
              : found->fallbackLongitudinal;
    }
  }
}

void scoreSurfacePairs(std::vector<SurfacePair> &pairs,
                       const std::array<HistoryPrediction, 2> &predictions,
                       const DetectorConfig &config) {
  for (SurfacePair &pair : pairs) {
    Candidate *sideCandidates[2] = {&pair.left, &pair.right};
    for (int side = 0; side < 2; ++side) {
      Candidate &candidate = *sideCandidates[side];
      // 连续性使用真实横截面坐标，不减去每片的中位中心；扫描覆盖
      // 范围变化时，中位中心本身会跳动并制造假的连续轨迹。
      candidate.localOffset = toVector(candidate.point);
      candidate.localOffset[static_cast<int>(config.longitudinalAxis)] = 0.0f;
      const HistoryPrediction &prediction =
          predictions[static_cast<std::size_t>(side)];
      candidate.historyAvailable[static_cast<std::size_t>(side)] =
          prediction.valid;
      if (prediction.valid) {
        candidate.historyDistance[static_cast<std::size_t>(side)] =
            transverseDistance(candidate.point, prediction.point,
                               config.longitudinalAxis);
      }

      // 三面交线是主约束；当交角变弱时，再提高对应一侧历史的权重。
      const float adaptiveHistoryWeight =
          config.historyWeight * prediction.stability *
          (1.0f + config.weakFeatureHistoryBoost * (1.0f - candidate.feature));
      const float historyCost =
          prediction.valid
              ? adaptiveHistoryWeight *
                    std::pow(candidate.historyDistance[static_cast<std::size_t>(
                                 side)] /
                                 std::max(1.0f, config.historySearchRadius),
                             2.0f)
              : 0.0f;
      candidate.nodeCost[static_cast<std::size_t>(side)] =
          -config.featureWeight * candidate.feature + historyCost;
    }
  }
}

void appendSlice(std::vector<Slice> &slices, const std::vector<PointT> &points,
                 float station, int frameIndex, const DetectorConfig &config,
                 const SeamHistory *leftHistory,
                 const SeamHistory *rightHistory, DebugTrace *debug) {
  const int sideAxis = transverseAxes(config.longitudinalAxis)[0];
  const int requiredProfilePoints =
      2 * (scaledProfileCount(config, config.fitHalfWindow, 5) +
           scaledProfileCount(config, config.fitGuardPoints, 1, 1)) +
      8;
  DebugSlice *debugSlice = nullptr;
  if (debug) {
    debug->slices.emplace_back();
    debugSlice = &debug->slices.back();
    debugSlice->station = station;
    debugSlice->raw = points;
  }
  if (points.size() < static_cast<std::size_t>(requiredProfilePoints))
    return;

  const std::vector<ProfilePoint> profile =
      makeProfile(points, config.longitudinalAxis, config, station,
                  config.longitudinalDetrendMinSpan);
  publishProfileStep(debug, profile, station, config);
  const std::array<HistoryPrediction, 2> predictions{
      {leftHistory ? leftHistory->predict(station, config.longitudinalAxis,
                                          config.maxHistoryExtrapolation,
                                          sideAxis, +1)
                   : HistoryPrediction(),
       rightHistory ? rightHistory->predict(station, config.longitudinalAxis,
                                            config.maxHistoryExtrapolation,
                                            sideAxis, -1)
                    : HistoryPrediction()}};
  if (debugSlice) {
    debugSlice->predictions = predictions;
    for (const auto &p : profile)
      debugSlice->profile.push_back(
          {p.u,
           p.v,
           p.support,
           {{p.fitMean.x(), p.fitMean.y()}},
           {{p.fitScatter(0, 0), p.fitScatter(0, 1), p.fitScatter(1, 1)}}});
    debugSlice->status = "no_candidates";
  }
  bool hasGeometry = false;
  std::vector<SurfacePair> pairs =
      findSurfacePairs(profile, config, predictions, &hasGeometry, debugSlice);
  publishPairStep(debug, pairs, station);
  if (pairs.empty())
    return;

  scoreSurfacePairs(pairs, predictions, config);
  Slice slice;
  if (debugSlice) {
    slice.debugIndex = debug->slices.size() - 1;
    debugSlice->hasGeometry = hasGeometry;
    debugSlice->status = "outside_main_segment";
    for (const auto &pair : pairs) {
      DebugPair d = pair.debug ? *pair.debug : DebugPair();
      d.points = {{pair.left.point, pair.right.point}};
      d.feature = {{pair.left.feature, pair.right.feature}};
      d.nodeCost = {{pair.left.nodeCost[0], pair.right.nodeCost[1]}};
      d.historyDistance =
          {{pair.left.historyDistance[0], pair.right.historyDistance[1]}};
      debugSlice->pairs.push_back(std::move(d));
    }
  }
  slice.station = station;
  slice.frameIndex = frameIndex;
  slice.hasGeometry = hasGeometry;
  slice.pairs = std::move(pairs);
  slices.push_back(std::move(slice));
}

std::vector<Slice> collectSlices(const CloudT &cloud, const StationRange &range,
                                 const DetectorConfig &config,
                                 const SeamHistory *leftHistory,
                                 const SeamHistory *rightHistory,
                                 DebugTrace *debug) {
  // S05 从 min(A)+厚度/2 开始逐站位收集 abs(A-station)<=厚度/2 的点。
  // 这是一组几何截面，不是传感器原始帧；步长小于厚度时截面允许重叠。
  std::vector<Slice> slices;
  const float minimum = range.minimum, maximum = range.maximum;
  const float halfThickness = 0.5f * config.sliceThickness;
  for (float station = minimum + halfThickness;
       station <= maximum - halfThickness; station += config.sliceStep) {
    checkCancellation();
    std::vector<PointT> points;
    for (const PointT &point : cloud) {
      if (std::fabs(coordinate(point, config.longitudinalAxis) - station) <=
          halfThickness) {
        points.push_back(point);
      }
    }
    if (stepDebugEnabled(debug))
      publishDebugStep(debug, "S05 纵向切片 · " + std::to_string(station),
                       {{"截面原始点", pointsCloud(points)}});
    appendSlice(slices, points, station, -1, config, leftHistory,
                rightHistory, debug);
  }
  return slices;
}

std::vector<Slice>
collectFrameSlices(const CloudT &cloud,
                   const std::vector<ScanFrameRange> &frames,
                   const DetectorConfig &config,
                   const SeamHistory *leftHistory,
                   const SeamHistory *rightHistory, DebugTrace *debug) {
  std::vector<Slice> slices;
  for (std::size_t center = 0; center < frames.size(); ++center) {
    checkCancellation();
    // 原始帧直接定义纵向站位，不再按纵向坐标重新分箱。为降低单帧缺测和
    // 扫描噪声的影响，每个目标帧联合相邻帧构造局部截面：中部为 5 帧，
    // 首尾自然缩为 3/4 帧。输出站位和 frame_index 仍属于目标帧。
    const std::size_t beginFrame = center > 2 ? center - 2 : 0;
    const std::size_t endFrame = std::min(frames.size(), center + 3);
    std::size_t pointCount = 0;
    for (std::size_t i = beginFrame; i < endFrame; ++i)
      if (!config.protectedHullInput || std::abs(frames[i].frameIndex-frames[center].frameIndex)<=2)
      pointCount += frames[i].count;
    std::vector<PointT> points;
    points.reserve(pointCount);
    for (std::size_t i = beginFrame; i < endFrame; ++i)
      if (!config.protectedHullInput || std::abs(frames[i].frameIndex-frames[center].frameIndex)<=2)
      points.insert(points.end(), cloud.begin() + frames[i].begin,
                    cloud.begin() + frames[i].begin + frames[i].count);
    const auto &frame = frames[center];
    const float station =
        coordinate(cloud[frame.begin], config.longitudinalAxis);
    if (stepDebugEnabled(debug))
      publishDebugStep(debug,
                       "S05 原始帧截面 · " + std::to_string(frame.frameIndex),
                       {{"相邻 3-5 帧联合截面", pointsCloud(points)}});
    appendSlice(slices, points, station, frame.frameIndex, config, leftHistory,
                rightHistory, debug);
  }
  return slices;
}

std::size_t retainMainSegment(std::vector<Slice> &slices,
                              const DetectorConfig &config,
                              bool hasBothHistories, DebugTrace *debug) {
  // 三面模型偶尔会在光滑上表面产生一两个孤立候选。焊缝应沿纵向形成
  // 连续主体，因此按“当前几何锚点”分段，只保留锚点最多的连续段。历史可
  // 在末端提供少量只参与优化的前视点，但最终输出仍严格截在几何锚点内。
  // S09 根据有效几何锚点选取主体连续段，避免把首尾圆滑上表面误当成焊缝。
  // 输出范围由可靠结构决定；本独立版无历史输入，不使用历史前视。
  std::size_t outputSliceCount = 0;
  if (!slices.empty()) {
    const float maximumSegmentGap =
        static_cast<float>(std::max(1, config.maxInterpolatedGapStations + 1)) *
        config.sliceStep;
    std::vector<std::size_t> anchors;
    for (std::size_t i = 0; i < slices.size(); ++i) {
      if (slices[i].hasGeometry) {
        anchors.push_back(i);
      }
    }

    std::size_t bestBegin = 0;
    std::size_t bestEnd = slices.size();
    if (!anchors.empty()) {
      std::size_t bestAnchorBegin = 0;
      std::size_t bestAnchorEnd = 1;
      std::size_t anchorBegin = 0;
      for (std::size_t i = 1; i <= anchors.size(); ++i) {
        const bool segmentEnds =
            i == anchors.size() ||
            slices[anchors[i]].station - slices[anchors[i - 1]].station >
                maximumSegmentGap + 0.25f * config.sliceStep;
        if (!segmentEnds) {
          continue;
        }
        if (i - anchorBegin > bestAnchorEnd - bestAnchorBegin) {
          bestAnchorBegin = anchorBegin;
          bestAnchorEnd = i;
        }
        anchorBegin = i;
      }
      bestBegin = anchors[bestAnchorBegin];
      bestEnd = anchors[bestAnchorEnd - 1] + 1;
    }
    std::size_t retainedEnd = bestEnd;
    if (!anchors.empty() && hasBothHistories) {
      retainedEnd =
          std::min(slices.size(),
                   bestEnd + static_cast<std::size_t>(
                                 std::max(0, config.historyLookaheadStations)));
      outputSliceCount = bestEnd - bestBegin;
    }
    if (bestBegin != 0 || retainedEnd != slices.size()) {
      std::vector<Slice> continuous;
      continuous.reserve(retainedEnd - bestBegin);
      for (std::size_t i = bestBegin; i < retainedEnd; ++i) {
        continuous.push_back(std::move(slices[i]));
      }
      slices = std::move(continuous);
    }
  }

  if (debug) {
    for (std::size_t i = 0; i < slices.size(); ++i) {
      auto &d = debug->slices[slices[i].debugIndex];
      d.retained = true;
      d.lookahead = outputSliceCount > 0 && i >= outputSliceCount;
      d.status = d.lookahead ? "lookahead_only" : "retained";
    }
  }
  publishRetainedSlices(debug, slices);
  return outputSliceCount;
}

float nodeCost(const SurfacePair &pair) {
  return pair.left.nodeCost[0] + pair.right.nodeCost[1];
}

float pairTransitionCost(const SurfacePair &previousPair,
                         const SurfacePair &currentPair, float stationGap,
                         const DetectorConfig &config) {
  const int sideAxis = transverseAxes(config.longitudinalAxis)[0];
  const float allowedJump = config.maxTransitionDistance *
                            std::max(1.0f, stationGap / config.sliceStep);
  const float leftDistance = transverseOffsetDistance(
      currentPair.left, previousPair.left, config.longitudinalAxis);
  const float rightDistance = transverseOffsetDistance(
      currentPair.right, previousPair.right, config.longitudinalAxis);
  const float leftNormalized = leftDistance / std::max(1.0f, allowedJump);
  const float rightNormalized = rightDistance / std::max(1.0f, allowedJump);
  float transition =
      config.smoothnessWeight *
      (leftNormalized * leftNormalized + rightNormalized * rightNormalized);
  if (leftDistance > allowedJump) {
    transition += 20.0f * (leftDistance - allowedJump) / allowedJump;
  }
  if (rightDistance > allowedJump) {
    transition += 20.0f * (rightDistance - allowedJump) / allowedJump;
  }
  const float currentWidth = component(currentPair.right.point, sideAxis) -
                             component(currentPair.left.point, sideAxis);
  const float previousWidth = component(previousPair.right.point, sideAxis) -
                              component(previousPair.left.point, sideAxis);
  const float widthChange =
      (currentWidth - previousWidth) / std::max(1.0f, allowedJump);
  return transition +
         0.75f * config.smoothnessWeight * widthChange * widthChange;
}

float tangentPredictionCost(const SurfacePair &beforePrevious,
                            const SurfacePair &previous,
                            const SurfacePair &current, float previousGap,
                            float currentGap, const DetectorConfig &config) {
  const float ratio = currentGap / std::max(1.0e-4f, previousGap);
  auto predictionError = [&](const Candidate &first, const Candidate &second,
                             const Candidate &third) {
    Eigen::Vector3f predicted =
        second.localOffset + (second.localOffset - first.localOffset) * ratio;
    Eigen::Vector3f difference = third.localOffset - predicted;
    difference[static_cast<int>(config.longitudinalAxis)] = 0.0f;
    return difference.norm();
  };
  const float leftError =
      predictionError(beforePrevious.left, previous.left, current.left);
  const float rightError =
      predictionError(beforePrevious.right, previous.right, current.right);
  const float allowedJump = config.maxTransitionDistance *
                            std::max(1.0f, currentGap / config.sliceStep);
  // 预测误差尺度小于单步最大位移：斜率可以很大，但斜率不应突然变化。
  const float curvatureScale = std::max(1.0f, 0.35f * allowedJump);
  const float leftNormalized = leftError / curvatureScale;
  const float rightNormalized = rightError / curvatureScale;
  float cost = config.curvatureWeight * (leftNormalized * leftNormalized +
                                         rightNormalized * rightNormalized);
  if (leftError > allowedJump) {
    cost += 30.0f * (leftError - allowedJump) / std::max(1.0f, allowedJump);
  }
  if (rightError > allowedJump) {
    cost += 30.0f * (rightError - allowedJump) / std::max(1.0f, allowedJump);
  }
  return cost;
}

bool optimizePairedPaths(const std::vector<Slice> &slices,
                         const DetectorConfig &config, DebugTrace *debug,
                         std::vector<Candidate> &leftPath,
                         std::vector<Candidate> &rightPath,
                         std::string &error) {
  // 状态保存连续两个截面的候选编号。这样不仅能限制相邻距离，还能用
  // 前两点的切线预测当前点；连续陡坡可以通过，突然换到另一块面会产生
  // 很大的预测残差。左右交点仍必须来自同一套截面结构候选。
  // S10 左右成对的二阶动态规划。
  // 状态=(前一截面候选编号, 当前截面候选编号)，从更早一片转移。
  // 总代价=特征节点代价+左右位移/宽度变化代价+前两点切线预测误差。
  // stateParents 记录最优前驱；回溯 selected 后得到 leftPath/rightPath。
  // 成对优化防止左右焊缝来自互不一致的截面结构。
  std::vector<std::vector<int>> stateParents(slices.size());
  const std::size_t firstCount = slices[0].pairs.size();
  const std::size_t secondCount = slices[1].pairs.size();
  std::vector<float> stateCosts(firstCount * secondCount,
                                std::numeric_limits<float>::infinity());
  stateParents[1].assign(firstCount * secondCount, -1);
  const float firstGap = slices[1].station - slices[0].station;
  for (std::size_t first = 0; first < firstCount; ++first) {
    for (std::size_t second = 0; second < secondCount; ++second) {
      stateCosts[first * secondCount + second] =
          nodeCost(slices[0].pairs[first]) +
          pairTransitionCost(slices[0].pairs[first], slices[1].pairs[second],
                             firstGap, config) +
          nodeCost(slices[1].pairs[second]);
    }
  }

  for (std::size_t i = 2; i < slices.size(); ++i) {
    checkCancellation();
    const std::size_t beforeCount = slices[i - 2].pairs.size();
    const std::size_t previousCount = slices[i - 1].pairs.size();
    const std::size_t currentCount = slices[i].pairs.size();
    std::vector<float> nextCosts(previousCount * currentCount,
                                 std::numeric_limits<float>::infinity());
    stateParents[i].assign(previousCount * currentCount, -1);
    const float previousGap = slices[i - 1].station - slices[i - 2].station;
    const float currentGap = slices[i].station - slices[i - 1].station;
    for (std::size_t previous = 0; previous < previousCount; ++previous) {
      for (std::size_t current = 0; current < currentCount; ++current) {
        const float base =
            pairTransitionCost(slices[i - 1].pairs[previous],
                               slices[i].pairs[current], currentGap, config) +
            nodeCost(slices[i].pairs[current]);
        const std::size_t state = previous * currentCount + current;
        for (std::size_t before = 0; before < beforeCount; ++before) {
          const float preceding = stateCosts[before * previousCount + previous];
          if (!std::isfinite(preceding)) {
            continue;
          }
          const float total =
              preceding + base +
              tangentPredictionCost(
                  slices[i - 2].pairs[before], slices[i - 1].pairs[previous],
                  slices[i].pairs[current], previousGap, currentGap, config);
          if (total < nextCosts[state]) {
            nextCosts[state] = total;
            stateParents[i][state] = static_cast<int>(before);
          }
        }
      }
    }
    stateCosts = std::move(nextCosts);
  }

  const auto last = std::min_element(stateCosts.begin(), stateCosts.end());
  if (last == stateCosts.end() || !std::isfinite(*last)) {
    error = "No continuous left/right seam pair could be formed.";
    return false;
  }

  leftPath.resize(slices.size());
  if (debug)
    debug->optimalCost = *last;
  rightPath.resize(slices.size());
  std::vector<int> selected(slices.size(), -1);
  const std::size_t lastCurrentCount = slices.back().pairs.size();
  const std::size_t lastState =
      static_cast<std::size_t>(std::distance(stateCosts.begin(), last));
  selected[slices.size() - 2] = static_cast<int>(lastState / lastCurrentCount);
  selected[slices.size() - 1] = static_cast<int>(lastState % lastCurrentCount);
  for (std::size_t i = slices.size() - 1; i >= 2; --i) {
    const std::size_t currentCount = slices[i].pairs.size();
    const std::size_t state =
        static_cast<std::size_t>(selected[i - 1]) * currentCount +
        static_cast<std::size_t>(selected[i]);
    selected[i - 2] = stateParents[i][state];
  }
  for (std::size_t i = 0; i < slices.size(); ++i) {
    if (selected[i] < 0 ||
        selected[i] >= static_cast<int>(slices[i].pairs.size())) {
      error = "Internal paired-path backtracking failed.";
      return false;
    }
    const SurfacePair &pair =
        slices[i].pairs[static_cast<std::size_t>(selected[i])];
    if (debug) {
      auto &d = debug->slices[slices[i].debugIndex];
      d.selected = selected[i];
      d.selectedNodeCost = nodeCost(pair);
      if (i > 0)
        d.transitionCost = pairTransitionCost(
            slices[i - 1].pairs[selected[i - 1]], pair,
            slices[i].station - slices[i - 1].station, config);
      if (i > 1)
        d.tangentCost = tangentPredictionCost(
            slices[i - 2].pairs[selected[i - 2]],
            slices[i - 1].pairs[selected[i - 1]], pair,
            slices[i - 1].station - slices[i - 2].station,
            slices[i].station - slices[i - 1].station, config);
    }
    leftPath[i] = pair.left;
    rightPath[i] = pair.right;
    leftPath[i].point.data[static_cast<int>(config.longitudinalAxis)] =
        slices[i].station;
    rightPath[i].point.data[static_cast<int>(config.longitudinalAxis)] =
        slices[i].station;
  }
  return true;
}

float predictTransverseCoordinate(const std::vector<Candidate> &path,
                                  float station, const DetectorConfig &config) {
  const int sideAxis = transverseAxes(config.longitudinalAxis)[0];
  const int axis = static_cast<int>(config.longitudinalAxis);
  const auto upper =
      std::lower_bound(path.begin(), path.end(), station,
                       [&](const Candidate &candidate, float value) {
                         return candidate.point.data[axis] < value;
                       });
  if (upper == path.begin()) {
    return component(path.front().point, sideAxis);
  }
  if (upper == path.end()) {
    return component(path.back().point, sideAxis);
  }
  const Candidate &after = *upper;
  const Candidate &before = *(upper - 1);
  const float beforeStation = before.point.data[axis];
  const float afterStation = after.point.data[axis];
  const float ratio = (station - beforeStation) /
                      std::max(1.0e-4f, afterStation - beforeStation);
  return component(before.point, sideAxis) +
         ratio * (component(after.point, sideAxis) -
                  component(before.point, sideAxis));
}

Candidate continuityCandidate(const std::vector<ProfilePoint> &profile,
                              float station, float predictedTransverse,
                              const DetectorConfig &config) {
  const int sideAxis = transverseAxes(config.longitudinalAxis)[0];
  Candidate candidate;
  const auto nearest = std::min_element(
      profile.begin(), profile.end(),
      [predictedTransverse](const ProfilePoint &lhs, const ProfilePoint &rhs) {
        return std::fabs(lhs.u - predictedTransverse) <
               std::fabs(rhs.u - predictedTransverse);
      });
  candidate.point = nearest->point;
  candidate.point.data[static_cast<int>(config.longitudinalAxis)] = station;
  // 横向保持连续模型的位置，高度取本次点云中该横向附近的实测值。
  candidate.point.data[sideAxis] = predictedTransverse;
  candidate.feature = 0.08f;
  return candidate;
}

void interpolateInternalGaps(const CloudT &cloud, const StationRange &range,
                             const std::vector<Slice> &slices,
                             const DetectorConfig &config,
                             std::vector<Candidate> &leftPath,
                             std::vector<Candidate> &rightPath,
                             DebugTrace *debug,
                             const std::vector<ScanFrameRange> *frames = nullptr,
                             std::vector<int> *outputFrameIds = nullptr) {
  // S11 只在前后可靠锚点之间填补少量内部缺口。
  // U 由前后路径插值，V 取本次截面附近实测高度；首尾和长缺口不补。
  // 默认最多补 3 个站位，并继续保持左右最小间距。
  const float minimum = range.minimum, maximum = range.maximum;
  const float halfThickness = .5f * config.sliceThickness;
  if (frames) {
    std::vector<Candidate> completeLeft, completeRight;
    std::size_t anchor = 0;
    if (outputFrameIds)
      outputFrameIds->clear();
    for (const auto &frame : *frames) {
      const float station =
          coordinate(cloud[frame.begin], config.longitudinalAxis);
      if (anchor < slices.size() &&
          std::fabs(slices[anchor].station - station) <=
              .25f * config.sliceStep) {
        completeLeft.push_back(leftPath[anchor]);
        completeRight.push_back(rightPath[anchor]);
        if (outputFrameIds)
          outputFrameIds->push_back(frame.frameIndex);
        ++anchor;
        continue;
      }
      if (anchor == 0 || anchor >= slices.size() ||
          slices[anchor].station - slices[anchor - 1].station >
              float(config.maxInterpolatedGapStations + 1) *
                  config.sliceStep)
        continue;
      std::vector<PointT> points(cloud.begin() + frame.begin,
                                 cloud.begin() + frame.begin + frame.count);
      const auto profile =
          makeProfile(points, config.longitudinalAxis, config, station,
                      config.longitudinalDetrendMinSpan);
      if (profile.empty())
        continue;
      float predictedLeft =
          predictTransverseCoordinate(leftPath, station, config);
      float predictedRight =
          predictTransverseCoordinate(rightPath, station, config);
      if (predictedRight - predictedLeft < config.minSeamSeparation) {
        const float center = .5f * (predictedLeft + predictedRight);
        predictedLeft = center - .5f * config.minSeamSeparation;
        predictedRight = center + .5f * config.minSeamSeparation;
      }
      completeLeft.push_back(
          continuityCandidate(profile, station, predictedLeft, config));
      completeRight.push_back(
          continuityCandidate(profile, station, predictedRight, config));
      if (outputFrameIds)
        outputFrameIds->push_back(frame.frameIndex);
    }
    leftPath = std::move(completeLeft);
    rightPath = std::move(completeRight);
    if (debug) {
      for (const auto &c : leftPath)
        debug->interpolatedPath[0].push_back(c.point);
      for (const auto &c : rightPath)
        debug->interpolatedPath[1].push_back(c.point);
      debug->pathComplete = true;
      publishPathStep(debug, "S11 内部短缺口插补", leftPath, rightPath);
    }
    return;
  }
  // 只填补两个可靠锚点之间的少量漏检站位。首尾贴着弧形上表面的区段
  // 保持为空，不再为了视觉连续而生成实际上并不存在的焊缝。
  if (!leftPath.empty() && !rightPath.empty()) {
    std::vector<Candidate> completeLeft;
    std::vector<Candidate> completeRight;
    std::size_t anchor = 0;
    for (float station = minimum + halfThickness;
         station <= maximum - halfThickness + 0.25f * config.sliceStep;
         station += config.sliceStep) {
      if (anchor < slices.size() &&
          std::fabs(slices[anchor].station - station) <=
              0.25f * config.sliceStep) {
        completeLeft.push_back(leftPath[anchor]);
        completeRight.push_back(rightPath[anchor]);
        ++anchor;
        continue;
      }

      // 没有前后两个三面交线锚点时属于首尾弱特征区，直接不输出。内部
      // 缺口过长也不跨越，防止轨迹再次沿着弧形上表面延伸。
      if (anchor == 0 || anchor >= slices.size() ||
          slices[anchor].station - slices[anchor - 1].station >
              static_cast<float>(config.maxInterpolatedGapStations + 1) *
                  config.sliceStep) {
        continue;
      }

      std::vector<PointT> stationPoints;
      for (const PointT &point : cloud) {
        if (std::fabs(coordinate(point, config.longitudinalAxis) - station) <=
            halfThickness) {
          stationPoints.push_back(point);
        }
      }
      const std::vector<ProfilePoint> profile = makeProfile(
          stationPoints, config.longitudinalAxis, config, station,
          config.longitudinalDetrendMinSpan);
      if (profile.empty()) {
        continue;
      }
      float predictedLeft =
          predictTransverseCoordinate(leftPath, station, config);
      float predictedRight =
          predictTransverseCoordinate(rightPath, station, config);
      if (predictedRight - predictedLeft < config.minSeamSeparation) {
        // 短缺口插值仍保持左右最小间距，避免两条焊缝在缺口中合并。
        const float center = 0.5f * (predictedLeft + predictedRight);
        predictedLeft = center - 0.5f * config.minSeamSeparation;
        predictedRight = center + 0.5f * config.minSeamSeparation;
      }
      completeLeft.push_back(
          continuityCandidate(profile, station, predictedLeft, config));
      completeRight.push_back(
          continuityCandidate(profile, station, predictedRight, config));
    }
    leftPath = std::move(completeLeft);
    rightPath = std::move(completeRight);
  }

  if (debug) {
    for (const auto &c : leftPath)
      debug->interpolatedPath[0].push_back(c.point);
    for (const auto &c : rightPath)
      debug->interpolatedPath[1].push_back(c.point);
    debug->pathComplete = true;
    publishPathStep(debug, "S11 内部短缺口插补", leftPath, rightPath);
  }
}

std::vector<Candidate> regularizePath(const std::vector<Candidate> &path,
                                      const DetectorConfig &config) {
  // S12 平滑定位：分别对 U/V 解加权最小二乘，A 保持不变。
  // 目标=sum(w_i*(q_i-p_i)^2)+lambda*sum(两段斜率不一致的残差^2)。
  // 低置信候选更服从邻点连续性；这是贴面前的平滑，不能放到最终求交后。
  std::vector<Candidate> regularizedPath = path;
  // 对每个横向坐标解一个置信度加权的平滑最小二乘问题：
  //   数据残差 + lambda * 相邻两段斜率差。
  // 二阶项不会把连续陡坡当成跳变，端点也不会像缩短窗口均值那样突然变平。
  if (regularizedPath.size() >= 3 && config.curveRegularizationWeight > 0.0f) {
    const int longitudinalAxis = static_cast<int>(config.longitudinalAxis);
    const int pointCount = static_cast<int>(regularizedPath.size());
    for (int axis = 0; axis < 3; ++axis) {
      if (axis == longitudinalAxis) {
        continue;
      }
      Eigen::MatrixXf system = Eigen::MatrixXf::Zero(pointCount, pointCount);
      Eigen::VectorXf rhs = Eigen::VectorXf::Zero(pointCount);
      for (int i = 0; i < pointCount; ++i) {
        const Candidate &candidate =
            regularizedPath[static_cast<std::size_t>(i)];
        const float dataWeight =
            std::max(0.05f, 4.0f * candidate.feature * candidate.feature);
        system(i, i) += dataWeight;
        rhs(i) += dataWeight * candidate.point.data[axis];
      }
      for (int i = 1; i + 1 < pointCount; ++i) {
        const float previousStation =
            regularizedPath[static_cast<std::size_t>(i - 1)]
                .point.data[longitudinalAxis];
        const float currentStation =
            regularizedPath[static_cast<std::size_t>(i)]
                .point.data[longitudinalAxis];
        const float nextStation =
            regularizedPath[static_cast<std::size_t>(i + 1)]
                .point.data[longitudinalAxis];
        const float previousGap =
            std::max(1.0e-4f, currentStation - previousStation);
        const float nextGap = std::max(1.0e-4f, nextStation - currentStation);
        const float totalGap = previousGap + nextGap;
        Eigen::Vector3f coefficients;
        coefficients << -nextGap / totalGap, 1.0f, -previousGap / totalGap;
        const int indices[3] = {i - 1, i, i + 1};
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column) {
            system(indices[row], indices[column]) +=
                config.curveRegularizationWeight * coefficients[row] *
                coefficients[column];
          }
        }
      }
      const Eigen::VectorXf solution = system.ldlt().solve(rhs);
      if (!solution.allFinite()) {
        continue;
      }
      for (int i = 0; i < pointCount; ++i) {
        regularizedPath[static_cast<std::size_t>(i)].point.data[axis] =
            solution[i];
      }
    }
  }
  return regularizedPath;
}

struct WallEvidence {
  std::vector<PointT> patch, inliers;
  float beforeRms = -1;
  std::string method = "profile_fallback";
};

bool isSteepWallTrack(const std::vector<Candidate> &path) {
  // “贴垂直侧壁”按整段可靠侧壁的中值方向启用，避免单片角度在 45°
  // 附近摆动时反复切换约束。浅夹角焊道继续采用原有定位流程。
  std::vector<float> wallSteepness;
  for (const auto &c : path)
    if (c.hasWall)
      wallSteepness.push_back(std::abs(c.wallNormal.x()));
  return wallSteepness.size() >= 3 && median(wallSteepness) >= std::sqrt(.5f);
}

void fitLocalWallModel(Candidate &candidate,
                       const std::vector<Candidate> &regularizedPath,
                       std::size_t i, int side, const CloudT::ConstPtr &input,
                       const DetectorConfig &config, WallEvidence &evidence,
                       DebugTrace *debug) {
  // 用相邻站位的原始侧壁小片拟合三维局部面，避免逐片直线噪声导致贴壁后抖动。
  // 只收集候选外侧、靠近既有侧壁的点，不把槽内底面加入此拟合。
  if (candidate.hasWall) {
    const auto axes = transverseAxes(config.longitudinalAxis);
    const float profileScale = profileGeometryScale(config);
    const int a = int(config.longitudinalAxis);
    const auto anchor = toVector(candidate.point);
    Eigen::Vector3f trend = Eigen::Vector3f::Zero();
    if (regularizedPath.size() > 1) {
      const auto
          first = toVector(regularizedPath[i ? i - 1 : i].point),
          last = toVector(
              regularizedPath[std::min(i + 1, regularizedPath.size() - 1)]
                  .point);
      if (last[a] > first[a])
        trend = (last - first) / (last[a] - first[a]);
    }
    std::vector<Eigen::Vector3d> patch;
    for (const auto &sample : *input) {
      if (!pcl::isFinite(sample))
        continue;
      const Eigen::Vector3f delta = toVector(sample) - anchor;
      if (std::abs(delta[a]) >
          std::max(config.sliceThickness, 2 * config.sliceStep))
        continue;
      const auto d = (delta - trend * delta[a]).eval();
      const float du = d[axes[0]], dv = d[axes[1]];
      if ((side == 0 ? du >= -.5f * profileScale
                     : du <= .5f * profileScale) ||
          std::abs(du) > 6 * profileScale ||
          std::abs(dv) > 12 * profileScale)
        continue;
      const Eigen::Vector2f uv(sample.data[axes[0]] - trend[axes[0]] * delta[a],
                               sample.data[axes[1]] -
                                   trend[axes[1]] * delta[a]);
      if (std::abs(candidate.wallNormal.dot(uv) - candidate.wallOffset) >
          std::max(.5f, profileScale))
        continue;
      patch.push_back(toVector(sample).cast<double>());
    }
    if (debug)
      for (const auto &p : patch)
        evidence.patch.push_back(toPoint(p.cast<float>()));
    for (int pass = 0; pass < 2 && patch.size() >= 20; ++pass) {
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      for (const auto &p : patch)
        center += p;
      center /= double(patch.size());
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (const auto &p : patch) {
        const auto d = (p - center).eval();
        cov += d * d.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov /
                                                         double(patch.size()));
      if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < .01)
        break;
      const Eigen::Vector3d n = eig.eigenvectors().col(0);
      const float rms = float(std::sqrt(std::max(0., eig.eigenvalues()[0])));
      if (pass == 0) {
        std::vector<Eigen::Vector3d> kept;
        for (const auto &p : patch)
          if (std::abs(n.dot(p - center)) <= std::max(.15f, 2.5f * rms))
            kept.push_back(p);
        patch.swap(kept);
        continue;
      }
      Eigen::Vector2f normal{float(n[axes[0]]), float(n[axes[1]])};
      const float length = normal.norm();
      if (length < .1f || rms > std::min(.6f, config.regionGrowingDistance))
        break;
      normal /= length;
      if (std::abs(normal.dot(candidate.wallNormal)) <
          std::cos(12.f * kPi / 180.f))
        break;
      candidate.wallNormal = normal;
      candidate.wallOffset = float((n.dot(center) - n[a] * anchor[a]) / length);
      candidate.wallRms = rms;
      evidence.method = "local_plane";
      if (debug)
        for (const auto &p : patch)
          evidence.inliers.push_back(toPoint(p.cast<float>()));
    }
  }
}

void refitWallModel(Candidate &candidate,
                    const std::vector<Candidate> &regularizedPath,
                    const std::vector<Candidate> &wallReferences, std::size_t i,
                    int side, const CloudT::ConstPtr &input,
                    const DetectorConfig &config, WallEvidence &evidence,
                    DebugTrace *debug) {
  // S14 高残差侧壁重拟合，解决底面/外侧平板混入造成的侧壁偏斜。
  // 只处理 RMS>min(0.25,regionGrowingDistance) 的点；可靠段保持原模型。
  // 缩小纵向窗口、放宽旧面距离门槛，以固定种子采样 240 个三点平面。
  // 用可靠邻站位方向约束选平面，再对距离内点做两次 PCA 精修。
  // 至少 20 个内点且面内支撑不退化才采用；否则仍使用 S13 的模型。
  // 断点观察：reference、patch、best、rms、wallEvidence[i].method。
  const float reliableRms = std::min(.25f, config.regionGrowingDistance);
  if (!candidate.hasWall || candidate.wallRms <= reliableRms)
    return;
  const auto axes = transverseAxes(config.longitudinalAxis);
  const float profileScale = profileGeometryScale(config);
  const int a = int(config.longitudinalAxis);
  const Eigen::Vector3f anchor = toVector(candidate.point);
  std::vector<float> angles;
  for (std::size_t j = i > 12 ? i - 12 : 0;
       j < std::min(i + 13, wallReferences.size()); ++j) {
    const auto &ref = wallReferences[j];
    if (!ref.hasWall || ref.wallRms > reliableRms ||
        std::abs(coordinate(ref.point, config.longitudinalAxis) - anchor[a]) >
            12 * config.sliceStep)
      continue;
    Eigen::Vector2f n = ref.wallNormal;
    if (n.x() < 0)
      n = -n;
    angles.push_back(std::atan2(n.y(), n.x()));
  }
  Eigen::Vector2f reference = candidate.wallNormal;
  const float maxAngle = angles.size() >= 3 ? 12.f : 35.f;
  if (angles.size() >= 3) {
    const float angle = median(angles);
    reference = {std::cos(angle), std::sin(angle)};
  }
  const float orientationLimit = std::cos(maxAngle * kPi / 180.f);
  Eigen::Vector3f trend = Eigen::Vector3f::Zero();
  if (regularizedPath.size() > 1) {
    const Eigen::Vector3f delta =
        toVector(regularizedPath[std::min(i + 1, regularizedPath.size() - 1)]
                     .point) -
        toVector(regularizedPath[i ? i - 1 : i].point);
    if (delta[a] > 0)
      trend = delta / delta[a];
  }
  // 缩小纵向窗口，放宽旧平面的距离门槛，让实测侧壁能纠正旧方向。
  std::vector<Eigen::Vector3d> patch;
  for (const auto &sample : *input) {
    if (!pcl::isFinite(sample))
      continue;
    const Eigen::Vector3f delta = toVector(sample) - anchor;
    if (std::abs(delta[a]) >
        std::max(config.sliceStep, .5f * config.sliceThickness))
      continue;
    const Eigen::Vector3f d = delta - trend * delta[a];
    if (d[axes[0]] * (side == 0 ? 1.f : -1.f) >= -.5f * profileScale ||
        std::abs(d[axes[0]]) > 6 * profileScale ||
        std::abs(d[axes[1]]) > 12 * profileScale)
      continue;
    const Eigen::Vector2f uv(anchor[axes[0]] + d[axes[0]],
                             anchor[axes[1]] + d[axes[1]]);
    if (std::abs(candidate.wallNormal.dot(uv) - candidate.wallOffset) >
        3 * profileScale)
      continue;
    patch.push_back(delta.cast<double>());
  }
  if (patch.size() < 20)
    return;
  const double tolerance = std::min(
      double(reliableRms), std::max(.08, double(profileScale) * .15));
  auto oriented = [&](const Eigen::Vector3d &n) {
    Eigen::Vector2f uv{float(n[axes[0]]), float(n[axes[1]])};
    if (uv.norm() < .1f)
      return false;
    uv.normalize();
    return std::abs(uv.x()) >= std::sqrt(.5f) &&
           std::abs(uv.dot(reference)) >= orientationLimit;
  };
  std::mt19937 random(47);
  std::uniform_int_distribution<std::size_t> pick(0, patch.size() - 1);
  std::vector<Eigen::Vector3d> best;
  for (int trial = 0; trial < 240; ++trial) {
    const Eigen::Vector3d p = patch[pick(random)], q = patch[pick(random)],
                          r = patch[pick(random)];
    Eigen::Vector3d n = (q - p).cross(r - p);
    if (n.norm() < 1e-6)
      continue;
    n.normalize();
    if (!oriented(n))
      continue;
    std::vector<Eigen::Vector3d> inliers;
    for (const auto &s : patch)
      if (std::abs(n.dot(s - p)) <= tolerance)
        inliers.push_back(s);
    if (inliers.size() > best.size())
      best.swap(inliers);
  }
  for (int pass = 0; pass < 2 && best.size() >= 20; ++pass) {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto &p : best)
      center += p;
    center /= double(best.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto &p : best) {
      const Eigen::Vector3d delta = p - center;
      covariance += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance /
                                                       double(best.size()));
    if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < .01)
      break;
    const Eigen::Vector3d normal = eig.eigenvectors().col(0);
    const float rms = float(std::sqrt(std::max(0., eig.eigenvalues()[0])));
    if (!oriented(normal) || rms > reliableRms)
      break;
    if (pass == 0) {
      best.clear();
      for (const auto &p : patch)
        if (std::abs(normal.dot(p - center)) <= tolerance)
          best.push_back(p);
      continue;
    }
    Eigen::Vector2f uv{float(normal[axes[0]]), float(normal[axes[1]])};
    const float length = uv.norm();
    uv /= length;
    candidate.wallNormal = uv;
    candidate.wallOffset =
        float(normal.dot(center) / length) +
        uv.dot(Eigen::Vector2f(anchor[axes[0]], anchor[axes[1]]));
    candidate.wallRms = rms;
    evidence.method = "robust_neighborhood";
    if (debug) {
      evidence.patch.clear();
      evidence.inliers.clear();
      for (const auto &p : patch)
        evidence.patch.push_back(
            toPoint((p + anchor.cast<double>()).cast<float>()));
      for (const auto &p : best)
        evidence.inliers.push_back(
            toPoint((p + anchor.cast<double>()).cast<float>()));
    }
  }
}

std::vector<Candidate>
fitWallModels(const std::vector<Candidate> &path,
              const std::vector<Candidate> &regularizedPath, int side,
              const CloudT::ConstPtr &input, const DetectorConfig &config,
              std::vector<WallEvidence> &wallEvidence, DebugTrace *debug,
              const std::function<CloudT::ConstPtr(std::size_t)>& neighborhood) {
  const bool steepWallTrack = isSteepWallTrack(path);
  auto wallModels = regularizedPath;
  wallEvidence.resize(regularizedPath.size());
  // S13 先完成全部局部侧壁拟合；S14 再用这份固定快照辅助高残差重拟合。
  for (std::size_t i = 0; i < wallModels.size(); ++i) {
    checkCancellation();
    wallModels[i].hasWall = wallModels[i].hasWall && steepWallTrack;
    fitLocalWallModel(wallModels[i], regularizedPath, i, side, neighborhood(i), config,
                      wallEvidence[i], debug);
    wallEvidence[i].beforeRms = wallModels[i].wallRms;
    if (stepDebugEnabled(debug))
      publishDebugStep(debug,
                       sideStepTitle("S13 局部侧壁拟合", side,
                                     wallModels[i].point,
                                     config.longitudinalAxis),
                       {{"侧壁取点", pointsCloud(wallEvidence[i].patch)},
                        {"侧壁内点", pointsCloud(wallEvidence[i].inliers)}},
                       wallEvidence[i].method);
  }
  const auto wallReferences = wallModels;
  for (std::size_t i = 0; i < wallModels.size(); ++i) {
    checkCancellation();
    refitWallModel(wallModels[i], regularizedPath, wallReferences, i, side,
                   neighborhood(i), config, wallEvidence[i], debug);
    if (stepDebugEnabled(debug))
      publishDebugStep(debug,
                       sideStepTitle("S14 高残差侧壁重拟合", side,
                                     wallModels[i].point,
                                     config.longitudinalAxis),
                       {{"侧壁取点", pointsCloud(wallEvidence[i].patch)},
                        {"侧壁内点", pointsCloud(wallEvidence[i].inliers)}},
                       wallEvidence[i].method);
  }
  return wallModels;
}

DebugWallPoint projectOntoWall(Candidate &candidate, int side,
                               const CloudT::ConstPtr &input,
                               const DetectorConfig &config,
                               WallEvidence &evidence, DebugTrace *debug) {
  const float halfThickness = .5f * config.sliceThickness;
  // S15 将正则后的候选投影到当前站位的侧壁线：p'=p-n*(n·p-offset)。
  // 位移、侧壁 RMS、到当前原始截面最近点距离都通过才采用。
  // beforeFloor 是贴壁结果，也是下一阶段失败时保留的位置。
  DebugWallPoint wall;
  wall.side = side;
  wall.before = candidate.point;
  wall.after = candidate.point;
  wall.wallFitMethod = evidence.method;
  wall.wallRmsBeforeRefit = evidence.beforeRms;
  if (debug) {
    wall.wallPatch = std::move(evidence.patch);
    wall.wallInliers = std::move(evidence.inliers);
  }
  wall.normal = candidate.wallNormal;
  wall.offset = candidate.wallOffset;
  wall.rms = candidate.wallRms;
  wall.status = candidate.hasWall ? "wall_fit_rejected" : "no_reliable_wall";
  if (debug)
    debug->beforeWallPath[side].push_back(candidate.point);
  if (candidate.hasWall &&
      candidate.wallRms <= std::min(.6f, config.regionGrowingDistance)) {
    const auto axes = transverseAxes(config.longitudinalAxis);
    Eigen::Vector2f p(candidate.point.data[axes[0]],
                      candidate.point.data[axes[1]]);
    const float signedDistance =
        candidate.wallNormal.dot(p) - candidate.wallOffset;
    wall.distanceBefore = wall.distanceAfter = std::abs(signedDistance);
    const Eigen::Vector2f projected = p - signedDistance * candidate.wallNormal;
    const float limit = std::min(config.surfaceIntersectionTolerance,
                                 3.f * profileGeometryScale(config));
    wall.status = "displacement_rejected";
    if (projected.allFinite() && std::abs(signedDistance) <= limit) {
      float nearest = std::numeric_limits<float>::infinity();
      for (const auto &sample : *input) {
        if (!pcl::isFinite(sample) ||
            std::abs(coordinate(sample, config.longitudinalAxis) -
                     coordinate(candidate.point, config.longitudinalAxis)) >
                halfThickness)
          continue;
        nearest = std::min(nearest, (Eigen::Vector2f(sample.data[axes[0]],
                                                     sample.data[axes[1]]) -
                                     projected)
                                        .norm());
      }
      wall.supportDistance = nearest;
      wall.status = "support_rejected";
      if (nearest <= std::max(.5f, profileGeometryScale(config))) {
        candidate.point.data[axes[0]] = projected.x();
        candidate.point.data[axes[1]] = projected.y();
        wall.after = candidate.point;
        wall.distanceAfter = std::abs(candidate.wallNormal.dot(projected) -
                                      candidate.wallOffset);
        wall.status = "projected";
      }
    }
  }
  if (stepDebugEnabled(debug))
    publishDebugStep(debug,
                     sideStepTitle("S15 贴壁投影", side, candidate.point,
                                   config.longitudinalAxis),
                     {{"侧壁取点", pointsCloud(wall.wallPatch)},
                      {"修正前", pointsCloud({wall.before})},
                      {"贴壁结果", pointsCloud({wall.after})}},
                     wall.status);
  return wall;
}

void intersectWallAndFloor(Candidate &candidate, DebugWallPoint &wall,
                           const CloudT::ConstPtr &input,
                           const DetectorConfig &config) {
  const auto axes = transverseAxes(config.longitudinalAxis);
  const int a = int(config.longitudinalAxis);
  const auto anchor = toVector(candidate.point);
  const float halfThickness = .5f * config.sliceThickness;
  const Eigen::Vector2f n(wall.floorNormal[axes[0]], wall.floorNormal[axes[1]]);
  const float determinant =
      candidate.wallNormal.x() * n.y() - candidate.wallNormal.y() * n.x();
  wall.floorStatus = "floor_wall_parallel";
  if (std::abs(determinant) < .2f * n.norm())
    return;
  // S17 固定 A，联立侧壁线与底面在此截面内的直线，解 2×2 方程。
  // wallNu*U+wallNv*V=wallOffset
  // floorNu*U+floorNv*V=floorOffset-floorNa*A
  // 行列式过小表示近乎平行，拒绝；位移过大或没有原始点支持也拒绝。
  // 成功点同时落在两个面上，到侧壁面距离为零；不再做自由平滑。
  const float rhs = wall.floorOffset - wall.floorNormal[a] * anchor[a];
  Eigen::Vector3f point = anchor;
  point[axes[0]] =
      (candidate.wallOffset * n.y() - candidate.wallNormal.y() * rhs) /
      determinant;
  point[axes[1]] =
      (candidate.wallNormal.x() * rhs - candidate.wallOffset * n.x()) /
      determinant;
  wall.floorShift = (point - anchor).norm();
  wall.floorStatus = "floor_shift_rejected";
  if (!point.allFinite() || wall.floorShift > config.floorPatchMaxCorrection)
    return;
  float nearest = std::numeric_limits<float>::infinity();
  for (const auto &sample : *input) {
    if (!pcl::isFinite(sample) ||
        std::abs(sample.data[a] - anchor[a]) > halfThickness)
      continue;
    const auto delta = (toVector(sample) - point).eval();
    nearest = std::min(nearest, std::hypot(delta[axes[0]], delta[axes[1]]));
  }
  wall.floorStatus = "floor_intersection_unsupported";
  if (nearest > std::max(.5f, profileGeometryScale(config)))
    return;
  candidate.point = toPoint(point);
  wall.after = candidate.point;
  wall.supportDistance = nearest;
  wall.distanceAfter = std::abs(candidate.wallNormal.dot(Eigen::Vector2f(
                                    point[axes[0]], point[axes[1]])) -
                                candidate.wallOffset);
  wall.floorResidual = std::abs(wall.floorNormal.dot(point) - wall.floorOffset);
  wall.floorStatus = "floor_wall_intersection";
  wall.status = "projected";
}

void constrainToLocalFloor(Candidate &candidate,
                           const std::vector<Candidate> &regularizedPath,
                           std::size_t i, int side,
                           const CloudT::ConstPtr &input,
                           const DetectorConfig &config, DebugWallPoint &wall,
                           DebugTrace *debug) {
  // S16 局部底面：以轨迹切线 along 和向槽内 across 建立正方形取点区域。
  // 边长 floorPatchSize，靠壁边向内退让 floorPatchGuard；仅采集原始 input。
  // 两次 PCA 剔除离群点，至少 12 个内点，第二特征值>=0.1，RMS 不超上限。
  // floorNormal·p=floorOffset 是局部拟合面，不强制底面平坦或全局水平。
  // 断点观察：patchCenter、patch.size()、floorRms、floorStatus。
  wall.beforeFloor = candidate.point;
  wall.floorStatus =
      config.useLocalFloorConstraint ? "wall_unreliable" : "disabled";
  if (config.useLocalFloorConstraint && candidate.hasWall &&
      candidate.wallRms <= std::min(.6f, config.regionGrowingDistance)) {
    const auto axes = transverseAxes(config.longitudinalAxis);
    const int a = int(config.longitudinalAxis);
    const auto anchor = toVector(candidate.point);
    const float size = config.floorPatchSize, half = .5f * size;
    Eigen::Vector3f across = Eigen::Vector3f::Zero();
    across[axes[0]] = candidate.floorDirection.x();
    across[axes[1]] = candidate.floorDirection.y();
    if (across[axes[0]] * (side == 0 ? 1.f : -1.f) < 0)
      across = -across;
    Eigen::Vector3f along = Eigen::Vector3f::Zero();
    along[a] = 1;
    if (regularizedPath.size() > 1) {
      const auto
          first = toVector(regularizedPath[i ? i - 1 : i].point),
          last = toVector(
              regularizedPath[std::min(i + 1, regularizedPath.size() - 1)]
                  .point);
      if ((last - first).norm() > 1e-4f)
        along = (last - first).normalized();
    }
    along -= across * along.dot(across);
    along.normalize();
    const Eigen::Vector3f initialNormal = along.cross(across).normalized();
    const Eigen::Vector3f patchCenter =
        anchor + across * (half + config.floorPatchGuard);
    struct FloorSample {
      Eigen::Vector3d point;
      PointT raw;
      float alongOffset = 0, acrossOffset = 0;
    };
    std::vector<FloorSample> available;
    for (const auto &sample : *input) {
      if (!pcl::isFinite(sample))
        continue;
      const Eigen::Vector3f delta = toVector(sample) - patchCenter;
      if (std::abs(delta.dot(along)) > half ||
          std::abs(delta.dot(across)) > half ||
          std::abs(delta.dot(initialNormal)) > std::max(2.f, .5f * size))
        continue;
      available.push_back({toVector(sample).cast<double>(), sample,
                           delta.dot(along), delta.dot(across)});
      if (debug)
        wall.floorPatch.push_back(sample);
    }
    if (stepDebugEnabled(debug))
      publishDebugStep(debug,
                       sideStepTitle("S16 正方形底面取点", side,
                                     candidate.point, config.longitudinalAxis),
                       {{"底面取点", pointsCloud(wall.floorPatch)},
                        {"当前焊缝点", pointsCloud({candidate.point})}});
    wall.floorStatus = "insufficient_floor_points";
    // 曲面或局部凸起不应用放宽 RMS 来强行接受。先尝试原窗口；若它把
    // 曲率混进单个平面，则缩小到 75%/50%，仍要求至少 12 点和全部交点约束。
    for (const float scale : {1.0f, 0.75f, 0.5f}) {
      const float usedHalf = half * scale;
      std::vector<Eigen::Vector3d> patch;
      std::vector<PointT> rawPatch;
      for (const auto &sample : available) {
        if (std::abs(sample.alongOffset) <= usedHalf &&
            std::abs(sample.acrossOffset) <= usedHalf) {
          patch.push_back(sample.point);
          rawPatch.push_back(sample.raw);
        }
      }
      if (patch.size() < 12)
        continue;

      bool fitted = false;
      Eigen::Vector3d fittedCenter = Eigen::Vector3d::Zero();
      Eigen::Vector3d fittedNormal = Eigen::Vector3d::Zero();
      float fittedRms = -1.0f;
      for (int pass = 0; pass < 2 && patch.size() >= 12; ++pass) {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto &p : patch) center += p;
        center /= double(patch.size());
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto &p : patch) {
          const auto d = (p - center).eval();
          cov += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(
            cov / double(patch.size()));
        wall.floorStatus = "floor_degenerate";
        if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < .1)
          break;
        const Eigen::Vector3d normal = eig.eigenvectors().col(0);
        const float rms =
            float(std::sqrt(std::max(0., eig.eigenvalues()[0])));
        if (pass == 0) {
          std::vector<Eigen::Vector3d> kept;
          for (const auto &p : patch)
            if (std::abs(normal.dot(p - center)) <=
                std::max(.15f, 2.5f * rms))
              kept.push_back(p);
          patch.swap(kept);
        } else {
          fitted = true;
          fittedCenter = center;
          fittedNormal = normal;
          fittedRms = rms;
        }
      }
      if (!fitted)
        continue;

      wall.floorNormal = fittedNormal.cast<float>();
      wall.floorOffset = float(fittedNormal.dot(fittedCenter));
      wall.floorRms = fittedRms;
      wall.floorInliers = int(patch.size());
      wall.floorPatchSizeUsed = size * scale;
      wall.floorCorners.clear();
      if (debug) wall.floorPatch = rawPatch;
      for (const auto &q : std::array<Eigen::Vector2f, 4>{
               {{-usedHalf, -usedHalf}, {usedHalf, -usedHalf},
                {usedHalf, usedHalf}, {-usedHalf, usedHalf}}}) {
        Eigen::Vector3f p = patchCenter + along * q.x() + across * q.y();
        p -= wall.floorNormal * (wall.floorNormal.dot(p) - wall.floorOffset);
        wall.floorCorners.push_back(toPoint(p));
      }
      if (stepDebugEnabled(debug))
        publishDebugStep(debug,
                         sideStepTitle("S16 局部底面拟合", side,
                                       candidate.point,
                                       config.longitudinalAxis),
                         {{"底面取点", pointsCloud(wall.floorPatch)},
                          {"拟合面四角", pointsCloud(wall.floorCorners)}},
                         {}, {{"patch_size_used", wall.floorPatchSizeUsed},
                              {"floor_rms", wall.floorRms}});
      wall.floorStatus = "floor_fit_rejected";
      if (fittedRms > config.floorPatchMaxRms)
        continue;
      intersectWallAndFloor(candidate, wall, input, config);
      // 窗口缩小只解决“曲率使单平面 RMS 超限”。若当前尺度的平面已经
      // 通过 RMS，但交点位移或实测支持失败，不再缩窗绕过这些独立门槛。
      break;
    }
  }
}

void recordWallRefinement(const Candidate &candidate, int side,
                          const DetectorConfig &config,
                          const DebugWallPoint &wall, DebugTrace *debug) {
  if (debug) {
    DebugStep step;
    step.title =
        std::string("局部底面 ") + (side == 0 ? "L" : "R") + " · " +
        std::to_string(coordinate(candidate.point, config.longitudinalAxis));
    step.status = "completed";
    step.message = wall.floorStatus;
    step.metrics = {{"wall_rms_before_refit", wall.wallRmsBeforeRefit},
                    {"wall_rms", wall.rms},
                    {"wall_patch_points", double(wall.wallPatch.size())},
                    {"wall_inliers", double(wall.wallInliers.size())},
                    {"patch_size", config.floorPatchSize},
                    {"patch_size_used", wall.floorPatchSizeUsed},
                    {"patch_points", double(wall.floorPatch.size())},
                    {"inliers", double(wall.floorInliers)},
                    {"floor_rms", wall.floorRms},
                    {"floor_shift", wall.floorShift},
                    {"floor_residual", wall.floorResidual},
                    {"wall_residual", wall.distanceAfter}};
    CloudT::Ptr patch(new CloudT), plane(new CloudT), before(new CloudT),
        after(new CloudT);
    for (const auto &p : wall.floorPatch)
      patch->push_back(p);
    if (wall.floorCorners.size() == 4)
      for (int row = 0; row <= 10; ++row)
        for (int col = 0; col <= 10; ++col) {
          const Eigen::Vector3f p = toVector(wall.floorCorners[0]) +
                                    (toVector(wall.floorCorners[1]) -
                                     toVector(wall.floorCorners[0])) *
                                        (row / 10.f) +
                                    (toVector(wall.floorCorners[3]) -
                                     toVector(wall.floorCorners[0])) *
                                        (col / 10.f);
          plane->push_back(toPoint(p));
        }
    before->push_back(wall.beforeFloor);
    after->push_back(wall.after);
    CloudT::Ptr wallPatch(new CloudT), wallInliers(new CloudT);
    for (const auto &p : wall.wallPatch)
      wallPatch->push_back(p);
    for (const auto &p : wall.wallInliers)
      wallInliers->push_back(p);
    step.layers = {{"侧壁取点", wallPatch},   {"侧壁内点", wallInliers},
                   {"正方形底面取点", patch}, {"拟合底面", plane},
                   {"修正前", before},        {"最终点", after}};
    if (stepDebugEnabled(debug))
      publishDebugStep(debug,
                       sideStepTitle("S17 底面/侧壁/站位求交", side,
                                     candidate.point, config.longitudinalAxis),
                       step.layers, wall.floorStatus);
    if (debug->retainDetails) {
      debug->steps.push_back(std::move(step));
      debug->wallPoints.push_back(wall);
    }
  }
}

float candidateConfidence(const Candidate &sourceCandidate, int side,
                          const DetectorConfig &config,
                          std::size_t &historyDominated) {
  float pointScore = sourceCandidate.feature;
  if (sourceCandidate.historyAvailable[static_cast<std::size_t>(side)]) {
    const float historyAgreement = std::exp(
        -sourceCandidate.historyDistance[static_cast<std::size_t>(side)] /
        std::max(1.0f, config.historySearchRadius));
    pointScore = 0.65f * sourceCandidate.feature + 0.35f * historyAgreement;
    if (sourceCandidate.feature < 0.35f && historyAgreement > 0.55f) {
      ++historyDominated;
    }
  }
  return clamp01(pointScore);
}

void fillSideResult(const std::vector<Candidate> &path, int side,
                    const CloudT::ConstPtr &input, const DetectorConfig &config,
                    DebugTrace *debug, CloudT::Ptr seam,
                    std::vector<float> *pointConfidence,
                    std::vector<SeamPointQuality> *pointQuality,
                    float *confidence, float *historyUsageRatio) {
  const auto regularizedPath = regularizePath(path, config);
  // Exact longitudinal range index, not a filter: keep original sample order
  // and every point that can satisfy any of the local wall/floor predicates.
  const int a = int(config.longitudinalAxis);
  std::vector<std::size_t> order;
  if (config.protectedHullInput) {
    order.resize(input->size());std::iota(order.begin(),order.end(),0);
    std::stable_sort(order.begin(),order.end(),[&](std::size_t l,std::size_t r){return (*input)[l].data[a]<(*input)[r].data[a];});
  }
  auto neighborhood = [&](std::size_t index)->CloudT::ConstPtr {
    if (!config.protectedHullInput) return input;
    const float half=.5f*config.floorPatchSize;
    const float radius=std::max({config.sliceThickness,2*config.sliceStep,half+std::max(2.f,half)})+.001f;
      checkCancellation();const float station=regularizedPath[index].point.data[a];
      auto begin=std::lower_bound(order.begin(),order.end(),station-radius,[&](std::size_t i,float s){return (*input)[i].data[a]<s;});
      auto end=std::upper_bound(begin,order.end(),station+radius,[&](float s,std::size_t i){return s<(*input)[i].data[a];});
      std::vector<std::size_t> selected(begin,end);if(!std::is_sorted(selected.begin(),selected.end()))std::sort(selected.begin(),selected.end());
      CloudT::Ptr local(new CloudT);local->reserve(selected.size());for(auto i:selected)local->push_back((*input)[i]);
      return local;
  };
  if (stepDebugEnabled(debug))
    publishDebugStep(debug,
                     std::string("S12 曲线正则 · ") + (side == 0 ? "L" : "R"),
                     {{"原候选", candidateCloud(path)},
                      {"平滑路径", candidateCloud(regularizedPath)}});
  seam->reserve(regularizedPath.size());
  pointConfidence->reserve(path.size());
  pointQuality->reserve(path.size());
  std::vector<WallEvidence> wallEvidence;
  const auto wallModels = fitWallModels(path, regularizedPath, side, input,
                                        config, wallEvidence, debug, neighborhood);
  std::size_t historyDominated = 0;
  for (std::size_t i = 0; i < wallModels.size(); ++i) {
    checkCancellation();
    Candidate candidate = wallModels[i];
    const auto local=neighborhood(i);
    auto wall =
        projectOntoWall(candidate, side, local, config, wallEvidence[i], debug);
    constrainToLocalFloor(candidate, regularizedPath, i, side, local, config,
                          wall, debug);
    recordWallRefinement(candidate, side, config, wall, debug);
    seam->push_back(candidate.point);
    const float sourceConfidence =
        candidateConfidence(path[i], side, config, historyDominated);
    pointConfidence->push_back(sourceConfidence);
    SeamPointQuality quality;
    quality.sourceConfidence = sourceConfidence;
    quality.wallRms = wall.rms;
    quality.floorRms = wall.floorRms;
    quality.floorPatchSizeUsed = wall.floorPatchSizeUsed;
    quality.supportDistance = wall.supportDistance;
    quality.geometryCorrection =
        (toVector(wall.after) - toVector(wall.before)).norm();
    quality.wallAccepted = wall.status == "projected";
    quality.wallNormal = wall.normal;
    quality.wallOffset = wall.offset;
    quality.floorAccepted = wall.floorStatus == "floor_wall_intersection";
    quality.floorNormal = wall.floorNormal;
    quality.floorOffset = wall.floorOffset;
    if (path[i].feature == 0.08f)
      quality.flags |= QualityInterpolated;
    if (!quality.wallAccepted)
      quality.flags |= QualityWallUnavailable;
    if (config.useLocalFloorConstraint && !quality.floorAccepted)
      quality.flags |= QualityFloorUnavailable;
    pointQuality->push_back(quality);
  }
  seam->width = static_cast<std::uint32_t>(seam->size());
  seam->height = 1;
  seam->is_dense = true;
  *confidence =
      std::accumulate(pointConfidence->begin(), pointConfidence->end(), 0.0f) /
      static_cast<float>(pointConfidence->size());
  *historyUsageRatio = static_cast<float>(historyDominated) /
                       static_cast<float>(regularizedPath.size());
}

void summarizeDetection(SeamResult &result,
                        const std::vector<Candidate> &leftPath,
                        const std::vector<Candidate> &rightPath) {
  result.confidence = 0.5f * (result.leftConfidence + result.rightConfidence);

  std::size_t regionEdgeStations = 0;
  std::size_t localContourStations = 0;
  for (std::size_t i = 0; i < leftPath.size() && i < rightPath.size(); ++i) {
    if (leftPath[i].fromRegionEdges && rightPath[i].fromRegionEdges) {
      ++regionEdgeStations;
    }
    if (leftPath[i].fromLocalCorner || rightPath[i].fromLocalCorner)
      ++localContourStations;
  }
  std::ostringstream message;
  message << "Detected left=" << result.leftSeam->size()
          << " and right=" << result.rightSeam->size()
          << " seam stations; confidence=" << result.confidence
           << "; region-edge stations=" << regionEdgeStations
           << "; local-contour stations=" << localContourStations;
  if (result.hullEnvelopeRequested) {
    if (result.hullEnvelopeApplied)
      message << "; hull-envelope retained="
              << result.hullEnvelopeRetainedPoints << "/"
              << result.hullEnvelopeInputPoints;
    else
      message << "; hull-envelope fallback=" << result.hullEnvelopeStatus;
  }
  if (result.leftSmoothing.applied || result.rightSmoothing.applied) {
    message << "; robust-rts mean correction L/R="
            << result.leftSmoothing.meanCorrection << "/"
            << result.rightSmoothing.meanCorrection
            << "; innovation outliers L/R="
            << result.leftSmoothing.innovationOutliers << "/"
            << result.rightSmoothing.innovationOutliers;
  }
  result.summary = message.str();
}

} // namespace

CloudT::Ptr SeamDetector::preprocess(const CloudT::ConstPtr &input,
                                     const DetectorConfig &config,
                                     DebugTrace *debug) {
  if (config.protectedHullInput) return CloudT::Ptr(new CloudT(*input));
  // S01 去除 NaN/Inf。输出 finite
  // 不改变有效点坐标；先检查这里是否丢掉大量输入。
  CloudT::Ptr finite(new CloudT);
  finite->reserve(input->size());
  for (const PointT &point : *input) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }
  finite->width = static_cast<std::uint32_t>(finite->size());
  finite->height = 1;
  finite->is_dense = true;
  if (debug) {
    debug->clouds.push_back({"01_finite", finite});
    publishDebugStep(debug, "S01 去除无效点", {{"处理后点云", finite}});
  }
  if (finite->empty()) {
    return finite;
  }

#ifdef WELD_NO_PCL_FILTERS
  // 仅供没有 ABI 兼容 PCL 库时运行合成单元测试；正式 CMake 构建不会定义它。
  (void)config;
  if (debug) {
    debug->clouds.push_back({"02_sor_bypassed_test_build", finite});
    debug->clouds.push_back({"03_voxel_bypassed_test_build", finite});
  }
  return finite;
#else
  // S02 SOR：每点与近邻的平均距离超过统计门槛时视为离群。
  // 默认 K=12、标准差倍数=2；有限点不足时原样通过。
  CloudT::Ptr denoised(new CloudT);
  if (config.outlierMeanK >= 3 &&
      finite->size() > static_cast<std::size_t>(config.outlierMeanK)) {
    pcl::StatisticalOutlierRemoval<PointT> filter;
    filter.setInputCloud(finite);
    filter.setMeanK(config.outlierMeanK);
    filter.setStddevMulThresh(config.outlierStddev);
    filter.filter(*denoised);
  } else {
    *denoised = *finite;
  }

  if (debug) {
    debug->clouds.push_back({"02_sor", denoised});
    publishDebugStep(debug, "S02 离群滤波", {{"处理后点云", denoised}});
  }
  if (config.voxelSize <= 0.0f) {
    if (debug) {
      debug->clouds.push_back({"03_voxel_disabled", denoised});
      publishDebugStep(debug, "S03 体素降采样（已关闭）",
                       {{"保留点云", denoised}});
    }
    return denoised;
  }
  // S03 体素降采样：降低截面候选搜索成本；voxelSize<=0 时跳过。
  // 注意：后面的 S13/S14/S16/S17 实测支持检查使用当前裁剪结果。
  CloudT::Ptr downsampled(new CloudT);
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(denoised);
  voxel.setLeafSize(config.voxelSize, config.voxelSize, config.voxelSize);
  voxel.filter(*downsampled);
  if (debug) {
    debug->clouds.push_back({"03_voxel", downsampled});
    publishDebugStep(debug, "S03 体素降采样", {{"处理后点云", downsampled}});
  }
  return downsampled;
#endif
}

SeamResult SeamDetector::detect(const CloudT::ConstPtr &input,
                                const DetectorConfig &config,
                                const SeamHistory *leftHistory,
                                const SeamHistory *rightHistory,
                                DebugTrace *debug, const std::vector<ScanFrameRange> *frames,
                                const std::function<bool()> &cancelled) const {
  CancellationScope cancellationScope(cancelled);
  checkCancellation();
  SeamResult result;
  const bool frameBased = frames && !frames->empty();
  result.frameBased = frameBased;
  result.inputFrameCount = frameBased ? frames->size() : 0;
  DetectorConfig workingConfig = config;
  if (config.protectedHullInput) workingConfig.useHullEnvelopeCrop = false;
  if (debug) {
    auto onStep = std::move(debug->onStep);
    const bool retainDetails = debug->retainDetails;
    *debug = DebugTrace();
    debug->onStep = std::move(onStep);
    debug->retainDetails = retainDetails;
    debug->config = config;
    debug->algorithm = config.protectedHullInput ? "protected_all_segments" : (frameBased ? "scheme1_original_frames" : "reference");
    debug->leftHistoryPasses = leftHistory ? leftHistory->size() : 0;
    debug->rightHistoryPasses = rightHistory ? rightHistory->size() : 0;
    debug->clouds.push_back({"00_input", input});
    publishDebugStep(debug, "S00 输入点云", {{"原始点云", input}});
  }

  // S00-S03 校验与预处理。
  if (!validateDetectionInput(input, config, result.summary))
    return result;
  CloudT::ConstPtr workingInput = input;
  std::vector<FrameProjection> frameProjections;
  std::vector<ScanFrameRange> processedFrames;
  if (frameBased) {
    CloudT::Ptr flattened;
    float frameStep = 1.0f;
    if (!prepareFrameSections(input, *frames, config, flattened,
                              processedFrames, frameProjections, frameStep,
                              result.summary))
      return result;
    // 帧号只替代截面生成。内部用原始帧间距估计连续性尺度，并把每帧
    // 暂时展平为一个剖面；候选、配对、路径和局部面修正仍走同一套流程。
    workingConfig.sliceStep = frameStep;
    workingConfig.sliceThickness = std::max(frameStep, 5.0f * frameStep);
    workingInput = flattened;
    result.filteredCloud = flattened;
    if (debug) {
      debug->clouds.push_back({"01_original_frames", input});
      publishDebugStep(debug, "S01 保留原始帧点",
                       {{"原始帧点云", input}},
                       "帧号定义截面，预处理不会混合不同帧。");
      publishDebugStep(debug, config.protectedHullInput ? "S02 原始帧正交截面坐标" : "S02 截面来源：原始帧",
                       {{"逐帧预处理点云", flattened}});
      publishDebugStep(debug, config.protectedHullInput ? "S03 跳过二次滤波（主体已保护）" : "S03 逐帧体素降采样",
                       {{"处理后点云", flattened}});
    }
  } else {
    result.filteredCloud = preprocess(input, workingConfig, debug);
    workingInput = result.filteredCloud;
  }
  result.filteredCloud = cropToHullEnvelope(
      workingInput, workingConfig, debug,
      frameBased ? &processedFrames : nullptr, result);
  workingInput = result.filteredCloud;
  if (!result.filteredCloud || result.filteredCloud->size() < 30) {
    result.summary = "Too few points remain after hull-envelope cropping.";
    return result;
  }
  if (stableProfileSampling(workingConfig))
    workingConfig.profileGridOrigin =
        findProfileGridOrigin(*workingInput, workingConfig.longitudinalAxis);

  // S05-S09 生成截面候选并保留主体连续段。
  const auto range =
      findStationRange(*workingInput, workingConfig.longitudinalAxis);
  auto slices = frameBased
                    ? collectFrameSlices(*workingInput, processedFrames,
                                         workingConfig, leftHistory,
                                         rightHistory, debug)
                    : collectSlices(*workingInput, range, workingConfig,
                                    leftHistory, rightHistory, debug);
  const auto interior = selectInteriorCandidates(slices, workingConfig);
  result.interiorOpeningDirection = interior.direction;
  result.positiveInteriorStations = interior.positiveStations;
  result.negativeInteriorStations = interior.negativeStations;
  result.interiorOrientationAmbiguous = interior.ambiguous;
  const auto outputSliceCount = config.protectedHullInput ? 0 :
      retainMainSegment(slices, workingConfig, leftHistory && rightHistory, debug);
  if (slices.size() < 3) {
    result.summary = "Fewer than three usable cross-sections were found.";
    return result;
  }

  // The protected hull has already selected the main physical body. Do not
  // confuse that with selecting only the longest *seam* segment. Split long
  // gaps before optimization AND local regularization, retaining supported
  // segments on both sides without extrapolating across the gap.
  std::vector<std::vector<Slice>> groups;
  if (config.protectedHullInput) {
    std::vector<Slice> group;
    const float gap = (workingConfig.maxInterpolatedGapStations + 1.25f) * workingConfig.sliceStep;
    for (const auto& slice : slices) {
      if (!slice.hasGeometry) continue;
      if (!group.empty() && slice.station - group.back().station > gap) {
        if (group.size() >= 3) groups.push_back(std::move(group));
        group.clear();
      }
      group.push_back(slice);
    }
    if (group.size() >= 3) groups.push_back(std::move(group));
    if (groups.empty()) { result.summary = "No supported three-section seam segment."; return result; }
    if (debug) {
      for (const auto& group : groups) for (const auto& slice : group) {
        debug->slices[slice.debugIndex].retained = true;
        debug->slices[slice.debugIndex].status = "retained";
      }
      publishRetainedSlices(debug, slices);
    }
  } else groups.push_back(slices);
  std::vector<Candidate> allLeftPath, allRightPath;
  std::vector<int> allFrameIds;
  result.leftSeam.reset(new CloudT); result.rightSeam.reset(new CloudT);
  for (auto& group : groups) {
  checkCancellation();
  const auto& slices = group;
  // S10 成对优化左右轨迹，随后移除只参与优化的历史前视站位。
  std::vector<Candidate> leftPath, rightPath;
  if (!optimizePairedPaths(slices, workingConfig, debug, leftPath, rightPath,
                           result.summary))
    return result;
  // 前视站位已经完成了对末端路径的约束，现在立即裁掉，保证保存和显示中
  // 不会出现任何超出当前可靠几何范围的历史外推点。
  if (outputSliceCount > 0 && outputSliceCount < slices.size()) {
    group.resize(outputSliceCount);
    leftPath.resize(outputSliceCount);
    rightPath.resize(outputSliceCount);
  }

  if (debug) {
    for (const auto &c : leftPath)
      debug->selectedPath[0].push_back(c.point);
    for (const auto &c : rightPath)
      debug->selectedPath[1].push_back(c.point);
  }

  publishPathStep(debug, "S10 成对路径优化", leftPath, rightPath);

  // S11 插补内部短缺口；S12-S17 平滑、贴壁及局部底面求交。
  std::vector<int> outputFrameIds;
  interpolateInternalGaps(*workingInput, range, slices, workingConfig, leftPath,
                          rightPath, debug,
                          frameBased ? &processedFrames : nullptr,
                          frameBased ? &outputFrameIds : nullptr);
  std::vector<std::size_t> cuts{0};
  if (config.protectedHullInput)
    for (std::size_t i=1;i<leftPath.size();++i)
      if (coordinate(leftPath[i].point,config.longitudinalAxis)-coordinate(leftPath[i-1].point,config.longitudinalAxis)>1.25f*workingConfig.sliceStep)
        cuts.push_back(i);
  cuts.push_back(leftPath.size());
  for (std::size_t piece=0;piece+1<cuts.size();++piece) {
  const std::vector<Candidate> leftPiece(leftPath.begin()+cuts[piece],leftPath.begin()+cuts[piece+1]);
  const std::vector<Candidate> rightPiece(rightPath.begin()+cuts[piece],rightPath.begin()+cuts[piece+1]);
  SeamResult part;
  fillSideResult(leftPiece, 0, workingInput, workingConfig, debug,
                  part.leftSeam,
                  &part.leftPointConfidence, &part.leftPointQuality,
                  &part.leftConfidence,
                  &part.leftHistoryUsageRatio);
  fillSideResult(rightPiece, 1, workingInput, workingConfig, debug,
                  part.rightSeam,
                  &part.rightPointConfidence, &part.rightPointQuality,
                  &part.rightConfidence,
                  &part.rightHistoryUsageRatio);
  result.leftSegmentStarts.push_back(result.leftSeam->size());
  result.rightSegmentStarts.push_back(result.rightSeam->size());
  *result.leftSeam += *part.leftSeam; *result.rightSeam += *part.rightSeam;
  result.leftPointConfidence.insert(result.leftPointConfidence.end(), part.leftPointConfidence.begin(), part.leftPointConfidence.end());
  result.rightPointConfidence.insert(result.rightPointConfidence.end(), part.rightPointConfidence.begin(), part.rightPointConfidence.end());
  result.leftPointQuality.insert(result.leftPointQuality.end(), part.leftPointQuality.begin(), part.leftPointQuality.end());
  result.rightPointQuality.insert(result.rightPointQuality.end(), part.rightPointQuality.begin(), part.rightPointQuality.end());
  result.leftHistoryUsageRatio = part.leftHistoryUsageRatio;
  result.rightHistoryUsageRatio = part.rightHistoryUsageRatio;
  result.leftConfidence = part.leftConfidence; result.rightConfidence = part.rightConfidence;
  }
  allLeftPath.insert(allLeftPath.end(), leftPath.begin(), leftPath.end());
  allRightPath.insert(allRightPath.end(), rightPath.begin(), rightPath.end());
  allFrameIds.insert(allFrameIds.end(), outputFrameIds.begin(), outputFrameIds.end());
  }
  applySeamPostprocess(result, workingConfig);
  {
    result.leftConfidence =
        result.leftPointConfidence.empty()
            ? 0.0f
            : std::accumulate(result.leftPointConfidence.begin(),
                              result.leftPointConfidence.end(), 0.0f) /
                  static_cast<float>(result.leftPointConfidence.size());
    result.rightConfidence =
        result.rightPointConfidence.empty()
            ? 0.0f
            : std::accumulate(result.rightPointConfidence.begin(),
                              result.rightPointConfidence.end(), 0.0f) /
                  static_cast<float>(result.rightPointConfidence.size());
  }
  if (frameBased) {
    restoreOriginalFrameGeometry(result, allFrameIds, frameProjections,
                                 config.longitudinalAxis);
    restoreFrameCloudGeometry(*result.filteredCloud, processedFrames,
                              frameProjections, config.longitudinalAxis);
  }
  // S18 汇总左右结果。
  summarizeDetection(result, allLeftPath, allRightPath);
  if (frameBased)
    result.summary += " Original-frame input: " +
                      std::to_string(result.inputFrameCount) +
                      " frames; each target frame used a 3-5 frame local "
                      "window without longitudinal spatial binning.";
  publishDebugStep(debug, "S18 最终焊缝",
                   {{"处理后点云", result.filteredCloud},
                    {"左焊缝", result.leftSeam},
                    {"右焊缝", result.rightSeam}},
                   result.summary);
  return result;
}

} // namespace weld
