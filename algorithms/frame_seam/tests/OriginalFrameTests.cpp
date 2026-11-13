#include "FrameSeamAlgorithm.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
int main() {
  try {
    using namespace weld;
    DetectorConfig config;
    auto run = [&](float sign, bool smooth, bool partial = false,
                   bool raisedToe = false) {
      CloudT::Ptr cloud(new CloudT);
      std::vector<ScanFrameRange> frames;
      for (int f = 0; f < 40; ++f) {
        const auto begin = cloud->size();
        for (int j = 0; j <= 466; ++j) {
          const float y = -35.f + .15f * j, a = std::abs(y);
          if (partial && f >= 27 && y > 8)
            continue;
          float z = smooth ? 20.f * (1.f - y * y / (35.f * 35.f))
                           : (a < 12   ? 20.f + 1.2f * std::cos(y / 5.f)
                              : a < 22 ? 2.f * (22 - a)
                                       : 0.f);
          if (raisedToe && a < 12)
            // 此夹具的槽深为正；向开口凸起需要减小槽深。
            z -= 3.f * std::exp(-std::pow((a - 10.7f) / .7f, 2));
          if (!smooth &&
              (f == 14 || f == 15 || (partial && f >= 16 && f <= 21)))
            z = 0; // 两帧真实缺特征，不允许伪造输出。
          z = sign * z + .3f * f + .03f * std::sin(j * 1.7f + f);
          cloud->push_back(PointT(.7f * f, y, z));
        }
        frames.push_back({begin, cloud->size() - begin, f + 1});
      }
      DebugTrace trace;
      const auto result = detectFrameSeams(cloud, config, frames, &trace);
      if (smooth) {
        require(!result.ok(), "Smooth curved profile must not become a seam");
        return;
      }
      require(result.ok() && result.leftSeam->size() >= (partial ? 25 : 30),
              "Uneven bottom must retain continuous seams through short "
              "missing frames");
      float total = 0;
      for (int side = 0; side < 2; ++side) {
        const auto points = side ? result.rightSeam : result.leftSeam;
        for (const auto &p : *points) {
          total += std::abs(std::abs(p.y) - 12.f);
          const int frame = int(std::round(p.x / .7f));
          require(frame != 14 && frame != 15,
                  "Missing frames must not have fabricated output");
          if (partial) {
            require(frame < 14 || frame > 21,
                    "Long gaps must not be fabricated");
            require(side == 0 || frame < 27,
                    "Unobserved right side must not be fabricated");
          }
          require(std::abs(p.x - frame * .7f) < 1e-4f,
                  "Surface correction must stay in acquisition frame plane");
        }
      }
      total /= float(result.leftSeam->size() + result.rightSeam->size());
      std::cout << "raisedToe=" << raisedToe << " mean corner error=" << total
                << '\n';
      require(total < 1.5f,
              "Detected corners should remain near wall/bottom junctions");
      if (partial) {
        require(result.leftSeam->back().x > 26,
                "Visible left seam must continue when right side is absent");
        require(result.leftSegmentStarts.size() >= 2,
                "Long gap must preserve separate segments");
        require(result.leftSeam->front().x < 1,
                "Earlier reliable segment must not be discarded");
      }
      require(trace.algorithm == "frame_surface" && trace.pathComplete,
              "Scheme3 debug identity/status missing");
      for (const auto &step : trace.steps)
        for (const auto &metric : step.metrics)
          if (metric.first == "max_selected_correction")
            require(metric.second <= config.frameMaxCorrection + 1e-5,
                    "Surface displacement exceeded limit");
      std::cout << "sign=" << sign << " points=" << result.leftSeam->size()
                << " mean error=" << total << '\n';
    };
    run(1, false);
    run(-1, false);
    run(1, true);
    run(1, false, true);
    run(1, false, false, true);
    CloudT::Ptr tiny(new CloudT);
    tiny->push_back(PointT());
    require(!detectFrameSeams(tiny, config, {}).ok(),
            "Manifest-free input must fail closed");
    std::cout << "Frame algorithm tests passed.\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
