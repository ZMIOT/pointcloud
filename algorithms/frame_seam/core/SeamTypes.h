#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace weld {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

enum class Axis { X = 0, Y = 1, Z = 2 };

struct DetectorConfig {
  // 方案1：在候选内侧取局部底面正方形，以底面、侧壁、当前站位共同定位。
  float floorPatchSize = 6.0f;
  bool useLocalFloorConstraint = true;
  float floorPatchGuard = 0.6f;
  float floorPatchMaxRms = 0.4f;
  float floorPatchMaxCorrection = 5.0f;
  // 方案3：原始帧轮廓与邻帧局部面。与方案1/2处理逻辑独立。
  float frameBinSize = 0.25f;
  float frameCornerWindow = 2.5f;
  float frameMinTurnDegrees = 25.0f;
  float frameMinDepth = 3.0f;
  float frameMaxProfileGap = 1.5f;
  int frameSurfaceNeighbors = 3;
  float frameSurfaceRadius = 4.0f;
  float frameSurfaceMaxRms = 0.6f;
  float frameMaxCorrection = 1.2f;
  float frameMaxPointDistance = 0.8f;
  float frameMaxTrackJump = 3.0f;
  int frameMaxMissing = 3;
  Axis longitudinalAxis = Axis::X;

  // 所有长度参数都使用点云自身单位；当前样例数据按毫米设置默认值。
  float voxelSize = 0.8f;
  int outlierMeanK = 12;
  float outlierStddev = 2.0f;
  float sliceStep = 2.0f;
  float sliceThickness = 3.5f;
  float profileBinSize = 1.2f;
  int fitHalfWindow = 5;
  // 拟合三块结构面时跳过交接点附近的焊道冠部，避免焊道余高拉偏板面。
  int fitGuardPoints = 1;
  // 底面相对剖面横向的最大斜率，以及侧面与底面的最小夹角。
  // 二者共同排除底面纹理、扫描边缘等不符合“侧面-底面”结构的折角。
  float maxBottomSlope = 0.9f;
  float minSurfaceAngleDegrees = 20.0f;
  // 大窗口确认三块大面的总体方向，小窗口再确认交界处确实发生了局部法向
  // 突变。后者用于排除把一整块平滑弧面误拆成三条直线的情况。
  float minLocalSurfaceAngleDegrees = 8.0f;
  int localAngleWindow = 4;
  // 底面在每个交界处分别做多尺度局部拟合。窗口逐步扩大时，如果法向
  // 变化超过该角度，就停止扩大，避免把焊接变形或另一处折角混入切平面。
  float maxLocalFitNormalChangeDegrees = 6.0f;
  // 左右两个局部底面允许有缓慢翘曲，但不能指向完全不同的结构面。
  float maxLocalBottomAngleDifferenceDegrees = 18.0f;
  // 在每个二维剖面内从底面中央、左右侧面稳定段分别开始区域生长。
  // 相邻局部切线变化小且点到当前切线足够近时归入同一区域；这样平滑
  // 弯曲的底面不会被强制压成直线，同时会在侧面法向突变处停止。
  bool useRegionGrowingEdges = true;
  // 底面生长跟踪分箱中值形成的轮廓切线。厚截面内的纵向高度变化会
  // 增大箱内竖向离差，不能把这种厚度当成底面在剖面内的法向变化。
  // 侧面和三面结构确认仍使用原始分布的加权 PCA。
  bool useContourRegionGrowing = true;
  // 只有侧面与底面方向充分分离时才以局部轮廓折点校正位置；浅夹角
  // 焊道冠部可能产生多个焊趾，继续使用区域配对，避免追到冠部另一侧。
  float minContourCornerAngleDegrees = 50.0f;
  int regionGrowingWindow = 4;
  float regionGrowingSmoothnessDegrees = 10.0f;
  float regionGrowingMaxNormalDriftDegrees = 35.0f;
  // 基础点线距离。底面中值轮廓有噪声时按局部 RMS 自适应，最多放宽到
  // 两倍；步进角和相对种子的法向漂移仍作为独立限制。
  float regionGrowingDistance = 2.0f;
  // 底面边缘与对应侧面边缘之间允许的最大缺口。候选位置取两个边缘
  // 最近点的中点，不再主要依赖两条无限延长直线的理论交点。
  float regionGrowingMaxEdgeGap = 8.0f;
  // 拟合面理论交点与分段边界允许的最大距离。
  float surfaceIntersectionTolerance = 10.0f;
  // 侧面样本到当前截面局部上表面切线的最小正交距离。它衡量侧面是否
  // 真正从弧形上表面分离，不能用全局 Z 高度代替。
  float minSurfaceClearance = 2.0f;
  // 三个面的正交拟合残差转成置信度时使用的毫米尺度。
  float surfaceRmsScale = 2.5f;
  int maxCandidatesPerSlice = 7;
  float minCandidateSpacing = 4.0f;
  // 两侧板面拟合线的理论交点只有落在此宽度内才可信，防止近似平行的
  // 拟合线在远处相交后把候选点拉离真实焊缝。
  float maxBeadWidth = 18.0f;
  // 同一截面内左右焊缝在横向坐标上的最小间距。配对约束可防止把同一
  // 焊缝附近的两个焊趾误识别为左右两道焊缝。
  float minSeamSeparation = 16.0f;
  float maxTransitionDistance = 12.0f;
  float smoothnessWeight = 2.0f;
  // 相比只限制相邻点距离，切线预测残差可以允许连续陡坡，同时抑制候选
  // 在相邻截面间切换到另一块表面。
  float curvatureWeight = 4.0f;
  float featureWeight = 4.0f;

  // 每个横向分箱在厚截面内独立估计纵向高度斜率，再投影回当前站位。
  // 真实点云的扫描覆盖变化可能破坏该估计，因此默认关闭；该值大于 0 时
  // 表示参与斜率计算的最小纵向点间距。
  float longitudinalDetrendMinSpan = 0.0f;

  // 历史项是软约束。特征越弱，它在总分中的占比越高。
  float historyWeight = 2.5f;
  float weakFeatureHistoryBoost = 3.0f;
  float historySearchRadius = 18.0f;
  float maxHistoryExtrapolation = 8.0f;
  std::size_t maxHistoryPasses = 5;
  // 历史可在可靠几何段末端之后提供少量“只参与优化、不输出”的前视站位，
  // 用来稳定最后几个已被焊道填圆的交点。
  int historyLookaheadStations = 8;

  // 只插补少量内部漏检站位；首尾或长距离贴近上表面的区段保持不输出。
  int maxInterpolatedGapStations = 3;
  // 输出曲线的二阶正则权重。它使用候选置信度作为数据权重，低置信度
  // 缺口主要服从前后切线趋势，不再使用会压平末端斜率的移动平均。
  float curveRegularizationWeight = 12.0f;
};

struct HistoryPrediction {
  bool valid = false;
  Eigen::Vector3f point = Eigen::Vector3f::Zero();
  // 0 表示历史轨迹相互矛盾，1 表示历次轨迹十分稳定。
  float stability = 0.0f;
};

struct SeamResult {
  CloudT::Ptr filteredCloud{new CloudT};
  CloudT::Ptr leftSeam{new CloudT};
  CloudT::Ptr rightSeam{new CloudT};
  std::vector<float> leftPointConfidence;
  std::vector<float> rightPointConfidence;
  // 独立轨迹段的起始点位置；绘图不得跨越段边界连线。
  std::vector<std::size_t> leftSegmentStarts, rightSegmentStarts;
  float leftConfidence = 0.0f;
  float rightConfidence = 0.0f;
  float confidence = 0.0f;
  float leftHistoryUsageRatio = 0.0f;
  float rightHistoryUsageRatio = 0.0f;
  std::string summary;

  bool ok() const {
    return leftSeam && rightSeam && leftSeam->size() >= 3 &&
           rightSeam->size() >= 3;
  }
};

inline float coordinate(const PointT &point, Axis axis) {
  switch (axis) {
  case Axis::X:
    return point.x;
  case Axis::Y:
    return point.y;
  case Axis::Z:
    return point.z;
  }
  return point.x;
}

inline float coordinate(const Eigen::Vector3f &point, Axis axis) {
  return point[static_cast<int>(axis)];
}

inline Eigen::Vector3f toVector(const PointT &point) {
  return Eigen::Vector3f(point.x, point.y, point.z);
}

inline PointT toPoint(const Eigen::Vector3f &point) {
  return PointT(point.x(), point.y(), point.z());
}

} // namespace weld
