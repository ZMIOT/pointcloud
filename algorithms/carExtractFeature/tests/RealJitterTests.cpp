#include "LapEdge.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdexcept>
#include <vector>

namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}

// 仅统计同一轨迹段内三个相邻扫描截面的二阶差分；连续坡度不会被当成尖跳。
double largestSingleSectionJump(const std::vector<lap::Sample> &trajectory) {
    double largest = 0;
    for (std::size_t i = 1; i + 1 < trajectory.size(); ++i) {
        const auto &prev = trajectory[i - 1];
        const auto &center = trajectory[i];
        const auto &next = trajectory[i + 1];
        if (prev.segment != center.segment || center.segment != next.segment ||
            prev.section + 1 != center.section || center.section + 1 != next.section)
            continue;
        largest = std::max(largest, (next.xyz - 2 * center.xyz + prev.xyz).norm());
    }
    return largest;
}
} // namespace

int main(int argc, char **argv) {
    try {
        require(argc == 2, "Expected the 3.pcd path");
        pcl::PointCloud<pcl::PointXYZ> cloud;
        require(pcl::io::loadPCDFile(argv[1], cloud) == 0, "Cannot load 3.pcd");
        std::vector<lap::Point> input;
        input.reserve(cloud.size());
        for (std::size_t i = 0; i < cloud.size(); ++i)
            input.push_back({lap::Vec(cloud[i].x, cloud[i].y, cloud[i].z), std::uint32_t(i)});

        // 对比关闭平滑的同一份输入，确认改善来自轨迹拟合而不是删掉检测点。
        lap::Config rawConfig;
        rawConfig.smoothing = 0;
        const auto raw = lap::detect(input, rawConfig);
        const auto smooth = lap::detect(input);
        require(raw.ok && smooth.ok, "3.pcd edge extraction failed");
        require(smooth.sections.size() == 395 && smooth.measured.size() == 391,
                "3.pcd scan recovery or measured edge changed");
        require(raw.measured.size() == smooth.measured.size(),
                "Smoothing changed the measured edge count");
        for (std::size_t i = 0; i < smooth.measured.size(); ++i) {
            const auto &p = smooth.measured[i];
            require((p.xyz - input[p.source].xyz).norm() == 0,
                    "Measured edge moved away from its source point");
            require(p.source == raw.measured[i].source,
                    "Smoothing changed a measured source index");
        }
        const double before = largestSingleSectionJump(raw.trajectory);
        const double after = largestSingleSectionJump(smooth.trajectory);
        require(before > 0.6, "3.pcd no longer exercises the single-section jitter case");
        // 真实波纹有连续坡度，但不应出现短窗口追逐噪声形成的尖锐弯折。
        require(after < 0.3 * smooth.pointSpacing,
                "Fitted trajectory retains an excessive single-section bend");
        int checkedBreaks = 0;
        for (std::size_t i = 1; i < smooth.trajectory.size(); ++i) {
            const auto &left = smooth.trajectory[i - 1];
            const auto &right = smooth.trajectory[i];
            if (left.segment == right.segment || left.interpolated || right.interpolated)
                continue;
            for (const auto &endpoint : {left, right}) {
                const auto measured = std::find_if(
                    smooth.measured.begin(), smooth.measured.end(),
                    [&](const lap::Sample &sample) { return sample.section == endpoint.section; });
                require(
                    measured != smooth.measured.end() &&
                        (endpoint.xyz - measured->xyz).norm() < 1e-6,
                    "A disconnected trajectory endpoint was pulled away from its measured point");
            }
            ++checkedBreaks;
        }
        require(checkedBreaks > 0, "3.pcd no longer exercises a disconnected trajectory");
        std::cout << "3.pcd maximum single-section jump: " << before << " -> " << after << '\n';
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
