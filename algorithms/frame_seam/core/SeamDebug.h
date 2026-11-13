#pragma once

#include "SeamTypes.h"
#include <array>
#include <map>

namespace weld {

// 仅在 detect(..., &trace) 时收集中间结果；不改变候选与轨迹的计算。
// 二维数据均使用固定剖面轴 (u,v)，三维数据保持原始 XYZ。
struct DebugProfilePoint {
  float u, v, support;
  std::array<float, 2> mean;
  std::array<float, 3> scatter; // xx, xy, yy：箱内离差和（未除以点数）
};

struct DebugLine {
  float u = 0, v = 0, du = 0, dv = 0, rms = 0;
  int begin = 0, end = 0; // 拟合窗口，半开区间
};

struct DebugPair {
  std::array<PointT, 2> points;
  std::array<float, 2> feature{{0, 0}}, nodeCost{{0, 0}};
  std::array<float, 2> historyDistance{{-1, -1}};
  std::string source = "intersection";
  float proposalCost = 0;
  std::array<DebugLine, 3> lines; // 左侧、跨槽底面、右侧
  std::array<int, 2> boundaries{{-1, -1}};
  std::array<int, 6> regions{{0, 0, 0, 0, 0, 0}}; // 左、底、右 begin/end
  std::array<int, 6> seeds{{0, 0, 0, 0, 0, 0}};
  std::array<float, 8> edges{{0, 0, 0, 0, 0, 0, 0, 0}};
  std::array<float, 2> edgeGaps{{-1, -1}}, angles{{0, 0}}, localAngles{{0, 0}};
  std::array<float, 2> clearance{{0, 0}};
  bool regionAttempted = false;
  std::array<bool, 2> regionUsed{{false, false}};
  std::array<int, 2> localCornerIndices{{-1, -1}};
  std::array<float, 2> localCornerAngles{{0, 0}};
};

struct DebugSlice {
  float station = 0;
  std::vector<PointT> raw;
  std::vector<DebugProfilePoint> profile;
  std::array<HistoryPrediction, 2> predictions;
  std::vector<DebugPair> pairs;
  // 每个枚举模型只计入最先失败的门槛；accepted 是通过全部门槛的数量。
  std::map<std::string, std::size_t> gates;
  std::string status = "raw_points_insufficient";
  bool hasGeometry = false, retained = false, lookahead = false;
  int selected = -1;
  float selectedNodeCost = 0, transitionCost = 0, tangentCost = 0;
};

struct DebugCloudStage {
  std::string name;
  CloudT::ConstPtr cloud;
};

// 通用算法步骤，不依赖参考算法的截面、PCA 或路径规划实现。
struct DebugStep {
  bool hasFrameView = false;
  Eigen::Vector3f viewOrigin = Eigen::Vector3f::Zero();
  Eigen::Vector3f viewU = Eigen::Vector3f::UnitY(), viewV = Eigen::Vector3f::UnitZ();
  std::string title, status, message;
  double elapsedMs = 0;
  std::map<std::string, double> metrics;
  std::vector<DebugCloudStage> layers; // 发布时深拷贝，后续修改不会污染已记录步骤。
};

struct DebugWallPoint {
  int side=0;
  PointT before, after;
  PointT beforeFloor;
  Eigen::Vector2f normal=Eigen::Vector2f::Zero();
  float offset=0, rms=0, distanceBefore=0, distanceAfter=0, supportDistance=-1;
  std::string status;
  std::vector<PointT> wallPatch, wallInliers;
  std::string wallFitMethod;
  float wallRmsBeforeRefit=-1;
  std::vector<PointT> floorPatch, floorCorners;
  Eigen::Vector3f floorNormal=Eigen::Vector3f::Zero();
  float floorOffset=0, floorRms=-1, floorResidual=-1, floorShift=0;
  int floorInliers=0;
  std::string floorStatus="not_attempted";
};

struct DebugTrace {
  std::string algorithm = "reference";
  std::vector<DebugStep> steps;
  DetectorConfig config;
  std::size_t leftHistoryPasses = 0, rightHistoryPasses = 0;
  std::vector<DebugCloudStage> clouds;
  std::vector<DebugSlice> slices;
  std::array<std::vector<PointT>, 2> selectedPath, interpolatedPath;
  std::array<std::vector<PointT>, 2> beforeWallPath;
  std::vector<DebugWallPoint> wallPoints;
  float optimalCost = 0;
  bool pathComplete = false;
};

} // namespace weld
