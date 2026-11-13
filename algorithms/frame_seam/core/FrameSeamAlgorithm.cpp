#include "FrameSeamAlgorithm.h"
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>

namespace weld {
namespace {
constexpr float pi = 3.14159265358979323846f;
float degrees(float r) { return r * 180.0f / pi; }
float median(std::vector<float> v) {
  if (v.empty())
    return 0;
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}
struct Bin {
  float u = 0, v = 0, h = 0;
  PointT point;
  std::size_t source = 0;
};
struct Candidate {
  PointT raw, corrected;
  std::size_t source = 0;
  float u = 0, h = 0, turn = 0, score = 0;
  float surfaceRms = -1, correction = 0;
  int supportFrames = 0;
  std::string surfaceStatus = "insufficient_support";
  float cost = 0, window = 0, bottomAngle = 0;
  int previous = -1;
};
struct Pair {
  int left = 0, right = 0;
  float score = 0;
};
struct Frame {
  ScanFrameRange range;
  Eigen::Vector3f center = Eigen::Vector3f::Zero(),
                  normal = Eigen::Vector3f::UnitX();
  Eigen::Vector3f u = Eigen::Vector3f::UnitY(), v = Eigen::Vector3f::UnitZ();
  std::vector<Bin> bins;
  std::array<std::vector<Candidate>, 2> candidates;
  std::vector<Pair> pairs;
  float depth = 0, sign = 1, planeRms = 0;
  std::array<int, 2> selected{{-1, -1}}, segment{{-1, -1}};
  std::array<PointT, 2> output;
  std::array<float, 2> smoothing{{0, 0}};
  std::string status = "no_corner_pair";
};
struct State {
  std::vector<Frame> frames;
};
struct Line {
  bool valid = false;
  float angle = 0, rms = 0;
};
Line fitLine(const std::vector<Bin> &bins, float lo, float hi, float maxGap) {
  std::vector<Eigen::Vector2f> points;
  float last = 0;
  for (const auto &b : bins)
    if (b.u >= lo && b.u <= hi) {
      if (!points.empty() && b.u - last > maxGap)
        return {};
      points.emplace_back(b.u, b.h);
      last = b.u;
    }
  if (points.size() < 4)
    return {};
  Eigen::Vector2f center = Eigen::Vector2f::Zero();
  for (const auto &p : points)
    center += p;
  center /= float(points.size());
  Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
  for (const auto &p : points) {
    auto d = (p - center).eval();
    covariance += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covariance /
                                                     float(points.size()));
  if (eig.info() != Eigen::Success)
    return {};
  Eigen::Vector2f direction = eig.eigenvectors().col(1);
  if (direction.x() < 0)
    direction = -direction;
  return {true, degrees(std::atan2(direction.y(), direction.x())),
          std::sqrt(std::max(0.f, eig.eigenvalues()[0]))};
}
void prepareFrame(Frame &f, const CloudT &cloud, const DetectorConfig &config) {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (std::size_t i = f.range.begin; i < f.range.begin + f.range.count; ++i)
    center += toVector(cloud[i]).cast<double>();
  center /= double(f.range.count);
  f.center = center.cast<float>();
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t i = f.range.begin; i < f.range.begin + f.range.count; ++i) {
    auto d = (toVector(cloud[i]).cast<double>() - center).eval();
    covariance += d * d.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance /
                                                     double(f.range.count));
  if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < 0.01f) {
    f.status = "degenerate_frame";
    return;
  }
  f.normal = eig.eigenvectors().col(0).cast<float>();
  f.planeRms = float(std::sqrt(std::max(0., eig.eigenvalues()[0])));
  if (f.planeRms > 0.1f) {
    f.status = "non_planar_frame";
    return;
  }
  f.u = Eigen::Vector3f::UnitY() - f.normal * f.normal.y();
  if (f.u.norm() < .1f) {
    f.status = "invalid_scan_axis";
    return;
  }
  f.u.normalize();
  f.v = f.normal.cross(f.u).normalized();
  if (f.v.z() < 0)
    f.v = -f.v;
  std::vector<Bin> raw;
  for (std::size_t i = f.range.begin; i < f.range.begin + f.range.count; ++i) {
    auto d = (toVector(cloud[i]) - f.center).eval();
    Bin b;
    b.u = d.dot(f.u);
    b.v = d.dot(f.v);
    b.point = cloud[i];
    b.source = i;
    raw.push_back(b);
  }
  std::sort(raw.begin(), raw.end(),
            [](const Bin &a, const Bin &b) { return a.u < b.u; });
  // 用中值选择一个实测点作分箱代表；候选始终能追溯到原始 index。
  for (std::size_t start = 0; start < raw.size();) {
    std::size_t end = start + 1;
    while (end < raw.size() && raw[end].u - raw[start].u < config.frameBinSize)
      ++end;
    std::vector<Bin> group(raw.begin() + start, raw.begin() + end);
    std::sort(group.begin(), group.end(),
              [](const Bin &a, const Bin &b) { return a.v < b.v; });
    f.bins.push_back(group[group.size() / 2]);
    start = end;
  }
  if (f.bins.size() < 20) {
    f.status = "insufficient_profile";
    return;
  }
  const float minU = f.bins.front().u, maxU = f.bins.back().u,
              span = maxU - minU;
  std::vector<float> left, right, middle;
  for (const auto &b : f.bins) {
    if (b.u < minU + .1f * span)
      left.push_back(b.v);
    if (b.u > maxU - .1f * span)
      right.push_back(b.v);
  }
  const float vl = median(left), vr = median(right);
  for (auto &b : f.bins) {
    const float baseline =
        vl + (vr - vl) * (b.u - (minU + .05f * span)) / (.9f * span);
    b.h = b.v - baseline;
    if (std::abs(b.u - (minU + maxU) * .5f) < .1f * span)
      middle.push_back(b.h);
  }
  const float depth = median(middle);
  f.sign = depth >= 0 ? 1.f : -1.f;
  f.depth = std::abs(depth);
  for (auto &b : f.bins)
    b.h *= f.sign;
  if (f.depth < config.frameMinDepth) {
    f.status = "insufficient_depth";
    return;
  }
}
void extractCandidates(Frame &f, const DetectorConfig &config) {
  if (f.bins.size() < 20 || f.depth < config.frameMinDepth)
    return;
  // 切线相对局部底面比较，不把两端不等高/一侧扫出视野造成的基线斜率当成壁角。
  const float w = config.frameCornerWindow;
  std::vector<float> directions;
  for (std::size_t i = 0; i < f.bins.size(); i += 4) {
    const auto line = fitLine(f.bins, f.bins[i].u - w, f.bins[i].u + w,
                              config.frameMaxProfileGap);
    if (line.valid && line.rms < .3f)
      directions.push_back(line.angle);
  }
  const float bottomDirection = median(directions);
  std::array<std::vector<Candidate>, 2> narrowFallback;
  for (const auto &b : f.bins) {
    if (b.h < f.depth * .35f)
      continue;
    for (int side = 0; side < 2; ++side) {
      Candidate best;
      bool found = false;
      for (float scale : {1.f, 2.f, 4.f, .5f}) {
        const float width = w * scale;
        const auto l =
            fitLine(f.bins, b.u - width, b.u - .15f, config.frameMaxProfileGap);
        const auto r =
            fitLine(f.bins, b.u + .15f, b.u + width, config.frameMaxProfileGap);
        if (!l.valid || !r.valid || l.rms > .65f || r.rms > .65f)
          continue;
        const auto &floor = side == 0 ? r : l;
        // 底面再向内延伸一个窗口，确认这里已进入底面而不是停在圆弧中部。
        const float innerWidth = std::max(w, width);
        const auto inner =
            side == 0 ? fitLine(f.bins, b.u + innerWidth, b.u + 2 * innerWidth,
                                config.frameMaxProfileGap)
                      : fitLine(f.bins, b.u - 2 * innerWidth, b.u - innerWidth,
                                config.frameMaxProfileGap);
        const bool sharp = scale <= 1 && l.angle - r.angle >= 60.f;
        const float sharpLimit = scale < 1 ? 100.f : 55.f;
        if (!inner.valid ||
            std::abs(floor.angle - bottomDirection) >
                (sharp ? sharpLimit : 25.f) ||
            std::abs(inner.angle - bottomDirection) > 35.f)
          continue;
        const float turn = l.angle - r.angle;
        const float bottomChange = std::abs(floor.angle - inner.angle);
        if (turn < config.frameMinTurnDegrees ||
            bottomChange > (sharp        ? sharpLimit
                            : scale == 1 ? 30.f
                                         : 12.f))
          continue;
        float wallReach = 0;
        for (const auto &outer : f.bins) {
          const float delta = outer.u - b.u;
          if ((side == 0 ? delta < 0 : delta > 0) &&
              std::abs(delta) <= 1.8f * std::max(w, width))
            wallReach = std::max(
                wallReach,
                b.h - outer.h + delta * std::tan(bottomDirection * pi / 180.f));
        }
        if (wallReach <
            std::min(6.f, std::max(config.frameMinDepth, .2f * f.depth)))
          continue;
        const auto broad = side == 0
                               ? fitLine(f.bins, b.u - 1.8f * width, b.u - .3f,
                                         config.frameMaxProfileGap)
                               : fitLine(f.bins, b.u + .3f, b.u + 1.8f * width,
                                         config.frameMaxProfileGap);
        if (!broad.valid)
          continue;
        const float wallTurn =
            side == 0 ? broad.angle - inner.angle : inner.angle - broad.angle;
        if (wallTurn < 30.f)
          continue;
        // 平滑整弧没有稳定底段；宽窗口候选必须有低残差、方向稳定的内侧支持。
        if (scale > 1 && (inner.rms > .25f || bottomChange > 8.f))
          continue;
        Candidate c;
        c.raw = c.corrected = b.point;
        c.source = b.source;
        c.u = b.u;
        c.h = b.h;
        c.turn = turn;
        c.window = width;
        c.bottomAngle = bottomChange;
        c.score = .45f * std::min(1.f, turn / 70.f) +
                  .3f * std::min(1.f, b.h / std::max(1.f, f.depth)) +
                  .25f * (1.f - std::min(1.f, bottomChange / 20.f)) -
                  .15f * (l.rms + r.rms);
        if (scale < 1) {
          narrowFallback[side].push_back(c);
          continue;
        }
        if (!found || c.score > best.score) {
          best = c;
          found = true;
        }
      }
      if (found)
        f.candidates[side].push_back(best);
    }
  }
  for (int side = 0; side < 2; ++side)
    if (f.candidates[side].empty())
      f.candidates[side] = std::move(narrowFallback[side]);
  for (auto &side : f.candidates) {
    std::sort(side.begin(), side.end(),
              [](const Candidate &a, const Candidate &b) {
                return a.score > b.score;
              });
    std::vector<Candidate> selected;
    for (const auto &c : side) {
      bool near = false;
      for (const auto &v : selected)
        if (std::abs(c.u - v.u) < 1.f)
          near = true;
      if (!near && selected.size() < 8)
        selected.push_back(c);
    }
    side = selected;
  }
}
struct Plane {
  bool valid = false;
  Eigen::Vector3f center = Eigen::Vector3f::Zero(),
                  normal = Eigen::Vector3f::UnitX();
  float rms = 0;
};
Plane fitPlane(std::vector<Eigen::Vector3f> points) {
  Plane p;
  if (points.size() < 20)
    return p;
  for (int pass = 0; pass < 2; ++pass) {
    p.center.setZero();
    for (const auto &v : points)
      p.center += v;
    p.center /= float(points.size());
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto &v : points) {
      auto d = (v - p.center).eval();
      covariance += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance /
                                                       float(points.size()));
    if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < .01f)
      return p;
    p.normal = eig.eigenvectors().col(0);
    p.rms = std::sqrt(std::max(0.f, eig.eigenvalues()[0]));
    if (!pass) {
      std::vector<Eigen::Vector3f> retained;
      for (const auto &v : points)
        if (std::abs((v - p.center).dot(p.normal)) <
            std::max(.15f, 2.5f * p.rms))
          retained.push_back(v);
      if (retained.size() < 20)
        return p;
      points.swap(retained);
    }
  }
  p.valid = true;
  return p;
}
void refine(State &s, std::size_t frameIndex, const CloudT &cloud,
            const DetectorConfig &cfg) {
  auto &f = s.frames[frameIndex];
  for (int side = 0; side < 2; ++side)
    for (auto &c : f.candidates[side]) {
      std::vector<Eigen::Vector3f> wall, bottom;
      int support = 0;
      const int lo = std::max(0, int(frameIndex) - cfg.frameSurfaceNeighbors),
                hi = std::min(int(s.frames.size()) - 1,
                              int(frameIndex) + cfg.frameSurfaceNeighbors);
      for (int j = lo; j <= hi; ++j) {
        const auto &other = s.frames[j];
        if (other.bins.empty())
          continue;
        bool usedWall = false, usedBottom = false;
        for (const auto &b : other.bins) {
          const auto p = toVector(b.point);
          const float delta = (p - f.center).dot(f.u) - c.u;
          if (std::abs(delta) < .5f || std::abs(delta) > cfg.frameSurfaceRadius)
            continue;
          if ((p - toVector(c.raw)).norm() > cfg.frameSurfaceRadius * 5)
            continue;
          const bool isWall = side == 0 ? delta < 0 : delta > 0;
          if (isWall) {
            wall.push_back(p);
            usedWall = true;
          } else {
            bottom.push_back(p);
            usedBottom = true;
          }
        }
        if (usedWall && usedBottom)
          ++support;
      }
      c.supportFrames = support;
      if (support < 3)
        continue;
      const auto a = fitPlane(wall), b = fitPlane(bottom);
      if (!a.valid || !b.valid)
        continue;
      c.surfaceRms = std::max(a.rms, b.rms);
      if (c.surfaceRms > cfg.frameSurfaceMaxRms) {
        c.surfaceStatus = "surface_residual_rejected";
        continue;
      }
      Eigen::Matrix3f matrix;
      matrix.row(0) = a.normal.transpose();
      matrix.row(1) = b.normal.transpose();
      matrix.row(2) = f.normal.transpose();
      if (std::abs(matrix.determinant()) < .15f) {
        c.surfaceStatus = "surface_angle_rejected";
        continue;
      }
      Eigen::Vector3f rhs(a.normal.dot(a.center), b.normal.dot(b.center),
                          f.normal.dot(f.center));
      const Eigen::Vector3f intersection = matrix.fullPivLu().solve(rhs);
      const float shift = (intersection - toVector(c.raw)).norm();
      if (!intersection.allFinite() || shift > cfg.frameMaxCorrection) {
        c.surfaceStatus = "correction_limit_rejected";
        continue;
      }
      float nearest = std::numeric_limits<float>::max();
      for (std::size_t i = f.range.begin; i < f.range.begin + f.range.count;
           ++i)
        nearest = std::min(nearest, (toVector(cloud[i]) - intersection).norm());
      if (nearest > cfg.frameMaxPointDistance) {
        c.surfaceStatus = "unsupported_position_rejected";
        continue;
      }
      c.corrected = toPoint(intersection);
      c.correction = shift;
      c.surfaceStatus = "surface_corrected";
      c.score = std::min(
          1.f, c.score + .15f * (1.f - c.surfaceRms / cfg.frameSurfaceMaxRms));
    }
}
void makePairs(Frame &f, const DetectorConfig &cfg) {
  for (int l = 0; l < int(f.candidates[0].size()); ++l)
    for (int r = 0; r < int(f.candidates[1].size()); ++r) {
      const auto &a = f.candidates[0][l];
      const auto &b = f.candidates[1][r];
      if ((toVector(b.corrected) - toVector(a.corrected)).dot(f.u) <
          cfg.minSeamSeparation)
        continue;
      Pair p;
      p.left = l;
      p.right = r;
      p.score = (a.score + b.score) * .5f;
      f.pairs.push_back(p);
    }
  if (!f.pairs.empty())
    f.status = "candidates_ready";
}
float transition(const Frame &a, const Candidate &ca, const Frame &b,
                 const Candidate &cb, const DetectorConfig &cfg, int gap) {
  // 沿扫描平面法向的采集推进不计入跳跃；不减点云质心，避免可见范围变化拉动轨迹。
  const Eigen::Vector3f delta = toVector(cb.corrected) - toVector(ca.corrected);
  const float du = delta.dot(b.u), dv = delta.dot(b.v),
              distance = std::hypot(du, dv);
  if (distance > cfg.frameMaxTrackJump * gap)
    return std::numeric_limits<float>::infinity();
  return (du * du + dv * dv) / float(gap);
}
void selectPath(State &s, const DetectorConfig &cfg) {
  for (int side = 0; side < 2; ++side) {
    std::vector<int> current;
    int segment = 0, previous = -1;
    auto finish = [&]() {
      if (current.size() >= 3) {
        const auto &last = s.frames[current.back()].candidates[side];
        int chosen =
            int(std::min_element(last.begin(), last.end(),
                                 [](const Candidate &a, const Candidate &b) {
                                   return a.cost < b.cost;
                                 }) -
                last.begin());
        for (auto it = current.rbegin(); it != current.rend(); ++it) {
          auto &f = s.frames[*it];
          f.selected[side] = chosen;
          f.segment[side] = segment;
          f.output[side] = f.candidates[side][chosen].corrected;
          chosen = f.candidates[side][chosen].previous;
        }
        ++segment;
      }
      current.clear();
    };
    for (int i = 0; i < int(s.frames.size()); ++i) {
      auto &f = s.frames[i];
      if (f.candidates[side].empty())
        continue;
      if (previous >= 0 && i - previous > cfg.frameMaxMissing + 1) {
        finish();
        previous = -1;
      }
      bool any = false;
      for (auto &c : f.candidates[side]) {
        c.cost = std::numeric_limits<float>::infinity();
        c.previous = -1;
        if (previous < 0) {
          c.cost = -4 * c.score;
          any = true;
          continue;
        }
        const auto &prev = s.frames[previous];
        for (int j = 0; j < int(prev.candidates[side].size()); ++j) {
          const auto &pc = prev.candidates[side][j];
          const float cost = pc.cost +
                             transition(prev, pc, f, c, cfg, i - previous) -
                             4 * c.score;
          if (cost < c.cost) {
            c.cost = cost;
            c.previous = j;
            any = true;
          }
        }
      }
      if (!any) {
        finish();
        for (auto &c : f.candidates[side]) {
          c.cost = -4 * c.score;
          c.previous = -1;
        }
      }
      current.push_back(i);
      previous = i;
    }
    finish();
  }
}
void smoothPath(State &s, const CloudT &cloud, const DetectorConfig &cfg) {
  // 稳健局部二次回归只调整已观测点，逐点投影回真实扫描平面，不插补缺帧。
  for (int side = 0; side < 2; ++side)
    for (int i = 0; i < int(s.frames.size()); ++i) {
      auto &f = s.frames[i];
      if (f.selected[side] < 0)
        continue;
      std::vector<int> ids;
      for (int j = std::max(0, i - 5);
           j <= std::min(int(s.frames.size()) - 1, i + 5); ++j)
        if (s.frames[j].selected[side] >= 0 &&
            s.frames[j].segment[side] == f.segment[side])
          ids.push_back(j);
      if (ids.size() < 5)
        continue;
      const auto origin =
          toVector(f.candidates[side][f.selected[side]].corrected);
      std::vector<float> weights(ids.size(), 1.f);
      Eigen::Matrix3f coefficients = Eigen::Matrix3f::Zero();
      for (int pass = 0; pass < 3; ++pass) {
        Eigen::Matrix3f lhs = Eigen::Matrix3f::Zero(),
                        rhs = Eigen::Matrix3f::Zero();
        for (std::size_t k = 0; k < ids.size(); ++k) {
          const int j = ids[k];
          const auto &other = s.frames[j];
          const float t = float(j - i) / 5;
          const Eigen::Vector3f basis(1, t, t * t);
          const Eigen::Vector3f point =
              toVector(other.candidates[side][other.selected[side]].corrected) -
              origin;
          lhs += weights[k] * basis * basis.transpose();
          rhs += weights[k] * basis * point.transpose();
        }
        if (std::abs(lhs.determinant()) < 1e-6f)
          break;
        coefficients = lhs.fullPivLu().solve(rhs);
        for (std::size_t k = 0; k < ids.size(); ++k) {
          const int j = ids[k];
          const auto &other = s.frames[j];
          const float t = float(j - i) / 5;
          const Eigen::Vector3f p =
              toVector(other.candidates[side][other.selected[side]].corrected) -
              origin;
          const float residual =
              (coefficients.transpose() * Eigen::Vector3f(1, t, t * t) - p)
                  .norm();
          weights[k] = std::min(1.f, .35f / std::max(.001f, residual));
        }
      }
      Eigen::Vector3f shift = coefficients.row(0).transpose();
      shift -= f.normal * shift.dot(f.normal);
      if (shift.norm() > cfg.frameMaxCorrection)
        shift *= cfg.frameMaxCorrection / shift.norm();
      const Eigen::Vector3f point = origin + shift;
      float nearest = std::numeric_limits<float>::max();
      for (std::size_t j = f.range.begin; j < f.range.begin + f.range.count;
           ++j)
        nearest = std::min(nearest, (toVector(cloud[j]) - point).norm());
      if (nearest <= cfg.frameMaxPointDistance) {
        f.output[side] = toPoint(point);
        f.smoothing[side] = shift.norm();
      }
    }
}
} // namespace

SeamResult detectFrameSeams(const CloudT::ConstPtr &input,
                            const DetectorConfig &config,
                            const std::vector<ScanFrameRange> &ranges,
                            DebugTrace *debug, ExecutionHooks *hooks) {
  auto state = std::make_shared<State>();
  AlgorithmContext ctx;
  ctx.input = input;
  ctx.config = config;
  ctx.debug = debug;
  ctx.hooks = hooks;
  auto result = executeSteps(
      ctx,
      {{"01 · 原始帧校验与展开",
        [&](AlgorithmContext &c) {
          if (ranges.empty())
            return StepResult{StepStatus::Failed,
                              "方案3需要已核对的帧索引 manifest.json。"};
          if (config.frameBinSize <= 0 || config.frameCornerWindow <= .3f ||
              config.frameSurfaceRadius <= .5f ||
              config.frameSurfaceMaxRms <= 0 || config.frameMaxCorrection < 0 ||
              config.frameMaxProfileGap <= 0 ||
              config.frameSurfaceNeighbors < 0 ||
              config.frameMaxTrackJump <= 0 || config.frameMaxMissing < 0)
            return StepResult{StepStatus::Failed, "方案3参数范围无效。"};
          std::size_t offset = 0;
          for (const auto &r : ranges) {
            c.checkCancelled();
            if (r.begin != offset || r.count < 20 ||
                r.begin + r.count > input->size())
              return StepResult{StepStatus::Failed, "原始帧范围无效。"};
            offset += r.count;
          }
          if (offset != input->size())
            return StepResult{StepStatus::Failed, "原始帧未覆盖输入。"};
          for (const auto &r : ranges) {
            c.checkCancelled();
            Frame f;
            f.range = r;
            prepareFrame(f, *input, config);
            state->frames.push_back(std::move(f));
          }
          c.metrics["frame_count"] = double(ranges.size());
          return StepResult{StepStatus::Completed,
                            "按采集帧展开轮廓并计算槽深；预处理不混合不同帧。"};
        }},
       {"02 · 单帧候选",
        [&](AlgorithmContext &c) {
          int count = 0, paired = 0;
          for (auto &f : state->frames) {
            c.checkCancelled();
            extractCandidates(f, config);
            if (!f.candidates[0].empty() && !f.candidates[1].empty())
              ++paired;
            for (const auto &side : f.candidates)
              for (const auto &p : side) {
                c.features->push_back(p.raw);
                ++count;
              }
          }
          c.metrics["candidate_count"] = count;
          c.metrics["frames_with_two_sides"] = paired;
          return StepResult{StepStatus::Completed,
                            "以原始实测点为候选；检查局部转折、侧壁走向和槽深，"
                            "保留多个候选供联合筛选。"};
        }},
       {"03 · 邻帧局部面验证与修正",
        [&](AlgorithmContext &c) {
          c.features->clear();
          int corrected = 0;
          for (std::size_t i = 0; i < state->frames.size(); ++i) {
            c.checkCancelled();
            refine(*state, i, *input, config);
            makePairs(state->frames[i], config);
            for (const auto &side : state->frames[i].candidates)
              for (const auto &p : side) {
                c.features->push_back(p.corrected);
                if (p.surfaceStatus == "surface_corrected")
                  ++corrected;
              }
          }
          c.metrics["surface_corrected_candidates"] = corrected;
          return StepResult{StepStatus::Completed,
                            "分别拟合邻帧局部侧面和底面；三平面交点约束在当前扫"
                            "描帧内，经残差、位移和实测支持检查后采用。"};
        }},
       {"04 · 独立跟踪、受限平滑与逐帧记录", [&](AlgorithmContext &c) {
          c.checkCancelled();
          selectPath(*state, config);
          c.checkCancelled();
          smoothPath(*state, *input, config);
          c.checkCancelled();
          for (auto &f : state->frames)
            if (f.selected[0] >= 0 && f.selected[1] >= 0 &&
                (toVector(f.output[1]) - toVector(f.output[0])).dot(f.u) <
                    config.minSeamSeparation) {
              const int weaker = f.candidates[0][f.selected[0]].score <
                                         f.candidates[1][f.selected[1]].score
                                     ? 0
                                     : 1;
              f.selected[weaker] = -1;
            }
          std::array<int, 2> selected{{0, 0}}, lastSegment{{-1, -1}},
              lastFrame{{-100, -100}};
          double shift = 0;
          for (std::size_t i = 0; i < state->frames.size(); ++i) {
            c.checkCancelled();
            auto &f = state->frames[i];
            CloudT::Ptr raw(new CloudT), candidates(new CloudT),
                refined(new CloudT), chosen(new CloudT), tracked(new CloudT);
            if (debug)
              for (std::size_t j = f.range.begin;
                   j < f.range.begin + f.range.count; ++j)
                raw->push_back((*input)[j]);
            c.metrics.clear();
            c.metrics["frame"] = double(i);
            c.metrics["sdk_sequence"] = f.range.sdkSequence;
            c.metrics["depth"] = f.depth;
            c.metrics["pair_count"] = double(f.pairs.size());
            c.metrics["left_candidates"] = double(f.candidates[0].size());
            c.metrics["right_candidates"] = double(f.candidates[1].size());
            for (int side = 0; side < 2; ++side)
              for (std::size_t k = 0; k < f.candidates[side].size(); ++k) {
                const auto &p = f.candidates[side][k];
                candidates->push_back(p.raw);
                refined->push_back(p.corrected);
                const auto prefix = std::string(side == 0 ? "L" : "R") +
                                    std::to_string(k) + "_";
                c.metrics[prefix + "source_index"] = double(p.source + 1);
                c.metrics[prefix + "turn_deg"] = p.turn;
                c.metrics[prefix + "window"] = p.window;
                c.metrics[prefix + "bottom_angle_change"] = p.bottomAngle;
                c.metrics[prefix + "score"] = p.score;
                c.metrics[prefix + "surface_rms"] = p.surfaceRms;
                c.metrics[prefix + "correction"] = p.correction;
                c.metrics[prefix + "support_frames"] = p.supportFrames;
                c.metrics[prefix + p.surfaceStatus] = 1;
              }
            for (int side = 0; side < 2; ++side)
              if (f.selected[side] >= 0) {
                const auto &p = f.candidates[side][f.selected[side]];
                auto &points =
                    side == 0 ? c.result.leftSeam : c.result.rightSeam;
                auto &confidence = side == 0 ? c.result.leftPointConfidence
                                             : c.result.rightPointConfidence;
                auto &segments = side == 0 ? c.result.leftSegmentStarts
                                           : c.result.rightSegmentStarts;
                if (f.segment[side] != lastSegment[side] ||
                    int(i) - lastFrame[side] > config.frameMaxMissing + 1) {
                  segments.push_back(points->size());
                  lastSegment[side] = f.segment[side];
                }
                points->push_back(f.output[side]);
                tracked->push_back(p.corrected);
                lastFrame[side] = int(i);
                confidence.push_back(p.score);
                chosen->push_back(f.output[side]);
                ++selected[side];
                shift = std::max(shift, double(p.correction));
                const std::string prefix = side == 0 ? "left" : "right";
                c.metrics["selected_" + prefix] = f.selected[side];
                c.metrics[prefix + "_segment"] = double(segments.size() - 1);
                c.metrics[prefix + "_smoothing"] = f.smoothing[side];
                c.metrics[prefix + "_total_displacement"] =
                    (toVector(f.output[side]) - toVector(p.raw)).norm();
                c.metrics[prefix + "_x"] = f.output[side].x;
                c.metrics[prefix + "_y"] = f.output[side].y;
                c.metrics[prefix + "_z"] = f.output[side].z;
              }
            if (f.selected[0] >= 0 && f.selected[1] >= 0)
              f.status = "selected";
            else if (f.selected[0] >= 0)
              f.status = "selected_left_only";
            else if (f.selected[1] >= 0)
              f.status = "selected_right_only";
            else if (!f.candidates[0].empty() || !f.candidates[1].empty())
              f.status = "short_or_disconnected_candidates";
            c.snapshot("帧 " + std::to_string(i) + " · SDK " +
                           std::to_string(f.range.sdkSequence),
                       f.status,
                       {{"原始帧", raw},
                        {"单帧候选", candidates},
                        {"面修正候选", refined},
                        {"最终选中（平滑后）", chosen},
                        {"连续性选中（平滑前）", tracked}});
            if (debug) {
              auto &step = debug->steps.back();
              step.hasFrameView = true;
              step.viewOrigin = f.center;
              step.viewU = f.u;
              step.viewV = -f.sign * f.v;
            }
          }
          c.metrics.clear();
          c.metrics["selected_left_frames"] = selected[0];
          c.metrics["selected_right_frames"] = selected[1];
          c.metrics["max_selected_correction"] = shift;
          if (selected[0] < 3 || selected[1] < 3)
            return StepResult{
                StepStatus::Failed,
                "未找到至少3帧连续可靠的左右候选；请查看逐帧原因。"};
          c.result.leftConfidence =
              std::accumulate(c.result.leftPointConfidence.begin(),
                              c.result.leftPointConfidence.end(), 0.f) /
              selected[0];
          c.result.rightConfidence =
              std::accumulate(c.result.rightPointConfidence.begin(),
                              c.result.rightPointConfidence.end(), 0.f) /
              selected[1];
          c.result.confidence =
              (c.result.leftConfidence + c.result.rightConfidence) * .5f;
          c.result.summary = "方案3：" + std::to_string(ranges.size()) +
                             " 原始帧，输出 " + std::to_string(selected[0]) +
                             " 左 / " + std::to_string(selected[1]) + " 右" +
                             " 点；左右独立跟踪，不补造缺失侧。";
          return StepResult{
              StepStatus::Completed,
              "左右独立保留可靠段；稳健回归抑制跳点，修正受扫描平面、"
              "位移及实测支持限制。缺帧不补点，分段不跨接。"};
        }}});
  if (debug)
    debug->algorithm = "frame_surface";
  return result;
}
} // namespace weld
