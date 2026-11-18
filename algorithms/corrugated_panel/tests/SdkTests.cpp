#include "PointCloudWeldSdk.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string &message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

std::vector<PcwsFramePointV1> cloud() {
    std::vector<PcwsFramePointV1> points;
    std::mt19937 random(20261113u);
    std::normal_distribution<double> noise(0.0, 0.04);
    auto append = [&](double x, double y, double z) {
        PcwsFramePointV1 point{};
        point.frame_number = static_cast<std::uint64_t>(std::lround(x + 100.0));
        point.position_mm = {x + noise(random), y + noise(random), z + noise(random)};
        points.push_back(point);
    };
    for (int x = -100; x <= 100; x += 2)
        for (int y = -70; y <= 70; y += 2)
            append(double(x), double(y), 0.0);
    for (int x = -100; x <= 100; ++x) {
        const double wave = 22.0 * std::sin(double(x) / 18.0);
        for (int distance = 1; distance <= 35; ++distance) {
            append(double(x), wave, double(distance));
            append(double(x), -48.0, -double(distance));
        }
    }
    for (int i = 0; i < 80; ++i)
        append(-100.0 + 2.5 * i, 80.0, 10.0 + (i % 3) * 10.0);
    PcwsFramePointV1 invalid{};
    invalid.frame_number = 999;
    invalid.position_mm.x = std::numeric_limits<double>::quiet_NaN();
    points.push_back(invalid);
    return points;
}

bool finite(const PcwsPoint3d &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

double distance(const PcwsPoint3d &left, const PcwsPoint3d &right) {
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int32_t PCWS_CALL cancel(void *) { return 1; }

void invalidArgumentsAreRejected() {
    PcwsResultV1 result{};
    require(PcwsProcessV1(nullptr, &result) == PCWS_INVALID_ARGUMENT,
            "null request must fail");
    require(result.sdkbase == nullptr && result.sdkclass == nullptr,
            "failed request must not return arrays");
    require(result.error_utf8[0] != '\0', "failed request must explain the error");
    PcwsReleaseResultV1(&result);
}

void cancellationIsObserved(const std::vector<PcwsFramePointV1> &points) {
    PcwsRequestV1 request{};
    request.cloud_mm = points.data();
    request.cloud_count = points.size();
    request.should_cancel = cancel;
    PcwsResultV1 result{};
    require(PcwsProcessV1(&request, &result) == PCWS_CANCELED,
            "pre-cancelled request must stop");
    require(result.sdkbase == nullptr && result.sdkclass == nullptr,
            "cancelled request must not return partial arrays");
    PcwsReleaseResultV1(&result);
}

PcwsOptionsV1 optionsAreLoadedFromJson() {
    PcwsOptionsV1 options{};
    char error[256]{};
    const PcwsStatus status =
        PcwsLoadOptionsV1(PCWS_PARAMETERS_PATH, &options, error);
    require(status == PCWS_OK,
            std::string("parameter JSON failed: ") + error);
    require(options.struct_size == sizeof(PcwsOptionsV1),
            "loaded options must be a complete V1 snapshot");
    require(options.prefilter_window_mm == 12.0 &&
                options.prefilter_distance_mm == 0.8 &&
                options.prefilter_min_support == 12,
            "surface prefilter values were not loaded");
    require(options.white_point_spacing_mm == 10.0 &&
                options.max_surface_extrapolation_mm == 40.0,
            "trajectory values were not loaded");
    require((options.flags & PCWS_OPTION_DISABLE_SURFACE_PREFILTER) == 0,
            "packaged configuration must enable the surface prefilter");

    PcwsOptionsV1 missing{};
    missing.struct_size = 123;
    char missingError[256]{};
    const std::string missingPath =
        std::string(PCWS_PARAMETERS_PATH) + ".does-not-exist";
    require(PcwsLoadOptionsV1(missingPath.c_str(), &missing, missingError) ==
                PCWS_INVALID_ARGUMENT,
            "missing parameter JSON must fail");
    require(missing.struct_size == 0 && missingError[0] != '\0',
            "failed JSON load must clear options and explain the error");
    return options;
}

void frameCloudExportsCurrentResults(
    const std::vector<PcwsFramePointV1> &points, PcwsOptionsV1 options) {
    options.line_fit_tolerance_mm = 1.0;
    PcwsRequestV1 request{};
    request.cloud_mm = points.data();
    request.cloud_count = points.size();
    request.options = &options;
    PcwsResultV1 forward{};
    const PcwsStatus status = PcwsProcessV1(&request, &forward);
    require(status == PCWS_OK,
            std::string("frame cloud failed: ") + forward.error_utf8);
    require(forward.sdkbase_count > 90,
            "sdkbase must expose the dense C feature candidates");
    require(forward.sdkclass_count > 10,
            "sdkclass must expose the white uniform points");
    require(forward.sdkbase[0].x <
                forward.sdkbase[forward.sdkbase_count - 1].x,
            "zero scan direction must follow increasing frame numbers");
    for (std::uint64_t i = 0; i < forward.sdkbase_count; ++i)
        require(finite(forward.sdkbase[i]), "sdkbase contains a non-finite point");
    for (std::uint64_t i = 0; i < forward.sdkclass_count; ++i) {
        const auto &point = forward.sdkclass[i];
        require(finite(point.position_mm), "sdkclass contains a non-finite point");
        require(point.base_index < forward.sdkbase_count,
                "sdkclass base_index is out of range");
        if (i > 0)
            require(point.base_index >= forward.sdkclass[i - 1].base_index,
                    "sdkclass base_index must be monotonic");
    }
    require(forward.sdkclass[0].point_type == PCWS_POINT_START,
            "first white point must be start");
    require(forward.sdkclass[forward.sdkclass_count - 1].point_type ==
                PCWS_POINT_END,
            "last white point must be end");
    require(forward.sdkclass[forward.sdkclass_count - 1].segment_kind_after ==
                PCWS_SEGMENT_NONE,
            "last white point cannot own a following segment");

    request.scan_direction = {-1.0, 0.0, 0.0};
    PcwsResultV1 reverse{};
    const PcwsStatus reverseStatus = PcwsProcessV1(&request, &reverse);
    require(reverseStatus == PCWS_OK,
            std::string("reverse request failed: ") + reverse.error_utf8);
    require(reverse.sdkbase_count == forward.sdkbase_count &&
                reverse.sdkclass_count == forward.sdkclass_count,
            "direction reversal changed output counts");
    for (std::uint64_t i = 0; i < forward.sdkbase_count; ++i)
        require(distance(forward.sdkbase[i],
                         reverse.sdkbase[reverse.sdkbase_count - 1 - i]) < 1e-9,
                "scan direction must reverse sdkbase without changing geometry");
    for (std::uint64_t i = 0; i < forward.sdkclass_count; ++i)
        require(distance(forward.sdkclass[i].position_mm,
                         reverse.sdkclass[reverse.sdkclass_count - 1 - i].position_mm) <
                    1e-9,
                "scan direction must reverse sdkclass without changing geometry");

    PcwsReleaseResultV1(&forward);
    PcwsReleaseResultV1(&reverse);
    require(forward.sdkbase == nullptr && forward.sdkclass == nullptr &&
                forward.sdkbase_count == 0 && forward.sdkclass_count == 0,
            "release must clear the result");
    PcwsReleaseResultV1(&forward);
}

} // namespace

int main() {
    static_assert(sizeof(PcwsFramePointV1) == 32, "input point ABI changed");
    static_assert(sizeof(PcwsOptionsV1) == 176, "options ABI changed");
    static_assert(sizeof(PcwsResultV1) == 288, "result ABI changed");
    invalidArgumentsAreRejected();
    const PcwsOptionsV1 options = optionsAreLoadedFromJson();
    const auto points = cloud();
    cancellationIsObserved(points);
    frameCloudExportsCurrentResults(points, options);
    std::cout << "PointCloudWeldSdk tests passed\n";
    return 0;
}
