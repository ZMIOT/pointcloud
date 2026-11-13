#pragma once

#include "SeamTypes.h"

#include <string>
#include <vector>

namespace weld {

class PointCloudIO {
public:
  // TXT/XYZ 有 x/y/z 表头时按列名读取（支持 index x y z target，
  // 其中 target 是采集帧号，也兼容 frameIndex/frame_index）；
  // 无表头时仍取前三列 XYZ。保持有效点的原始行顺序，不排序、不重采样。
  static CloudT::Ptr load(const std::string &path, std::string *error = nullptr,
                          std::vector<ScanFrameRange> *frames = nullptr);

  static bool saveFrameSeamCsv(const std::string &path,
                               const SeamResult &result,
                               std::string *error = nullptr);

  // 将单条焊缝作为独立点云保存。PCD 使用二进制格式，PLY/XYZ 也可用于
  // 与其它点云软件交换；左右焊缝由调用方分别调用本函数。
  static bool savePointCloud(const std::string &path, const CloudT &cloud,
                             std::string *error = nullptr);

  static bool saveSeamCsv(const std::string &path, const CloudT &seam,
                          const std::vector<float> &confidence, Axis axis,
                          std::string *error = nullptr);

  // 左右焊缝保存在同一文件中，side 列明确区分 L/R，历史导入时不会混线。
  static bool saveDualSeamCsv(const std::string &path, const CloudT &leftSeam,
                              const std::vector<float> &leftConfidence,
                              const CloudT &rightSeam,
                              const std::vector<float> &rightConfidence,
                              Axis axis, std::string *error = nullptr);

  static bool loadDualSeamCsv(const std::string &path, CloudT::Ptr *leftSeam,
                              CloudT::Ptr *rightSeam,
                              std::string *error = nullptr);

  static CloudT::Ptr loadSeamCsv(const std::string &path,
                                 std::string *error = nullptr);

private:
  static CloudT::Ptr loadText(const std::string &path, std::string *error,
                              std::vector<ScanFrameRange> *frames);
};

} // namespace weld
