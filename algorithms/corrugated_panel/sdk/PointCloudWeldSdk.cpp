#include "PointCloudWeldSdk.h"

#include "ConfigIO.h"
#include "CorrugatedPanel.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t kKnownFlags =
    PCWS_OPTION_DISABLE_SURFACE_PREFILTER |
    PCWS_OPTION_DISABLE_PROJECTION_FALLBACK;

bool finite(const PcwsPoint3d &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

bwb::Vec3 vector(const PcwsPoint3d &point) {
    return {point.x, point.y, point.z};
}

PcwsPoint3d point(const bwb::Vec3 &value) {
    return {value.x(), value.y(), value.z()};
}

void setErrorBuffer(char *buffer, std::size_t capacity,
                    const std::string &message) {
    if (!buffer || capacity == 0)
        return;
    std::memset(buffer, 0, capacity);
    const std::size_t length = std::min(message.size(), capacity - 1);
    std::memcpy(buffer, message.data(), length);
}

void setError(PcwsResultV1 *result, const std::string &message) {
    if (result)
        setErrorBuffer(result->error_utf8, sizeof(result->error_utf8), message);
}

bool empty(const PcwsResultV1 &result) {
    return !result.sdkbase && result.sdkbase_count == 0 && !result.sdkclass &&
           result.sdkclass_count == 0;
}

bool applyOptions(const PcwsOptionsV1 *options, bwb::Config &config,
                  std::string &error) {
    if (!options)
        return true;
    if ((options->flags & ~kKnownFlags) != 0) {
        error = "PcwsOptionsV1 包含未知 flags。";
        return false;
    }
    if (options->struct_size != 0 &&
        options->struct_size < sizeof(PcwsOptionsV1)) {
        error = "PcwsOptionsV1.struct_size 小于当前 V1 结构尺寸。";
        return false;
    }
    config.prefilterEnabled =
        (options->flags & PCWS_OPTION_DISABLE_SURFACE_PREFILTER) == 0;
    config.allowProjectionFallback =
        (options->flags & PCWS_OPTION_DISABLE_PROJECTION_FALLBACK) == 0;

    const bool complete = options->struct_size >= sizeof(PcwsOptionsV1);
#define PCWS_APPLY_DOUBLE(field, target)                                         \
    do {                                                                         \
        if (complete || options->field != 0.0)                                   \
            config.target = options->field;                                      \
    } while (false)
#define PCWS_APPLY_INT(field, target)                                            \
    do {                                                                         \
        if (complete || options->field != 0)                                     \
            config.target = options->field;                                      \
    } while (false)
    if (complete || options->slice_offset_1_mm != 0.0)
        config.sliceOffsets[0] = options->slice_offset_1_mm;
    if (complete || options->slice_offset_2_mm != 0.0)
        config.sliceOffsets[1] = options->slice_offset_2_mm;
    if (complete || options->slice_offset_3_mm != 0.0)
        config.sliceOffsets[2] = options->slice_offset_3_mm;
    PCWS_APPLY_DOUBLE(slice_thickness_mm, sliceThickness);
    PCWS_APPLY_DOUBLE(plane_distance_mm, planeDistance);
    PCWS_APPLY_DOUBLE(reference_segment_length_mm, referenceSegmentLength);
    PCWS_APPLY_DOUBLE(max_adjacent_normal_angle_degrees,
                      maxAdjacentNormalAngleDegrees);
    PCWS_APPLY_DOUBLE(prefilter_window_mm, prefilterWindow);
    PCWS_APPLY_DOUBLE(prefilter_distance_mm, prefilterDistance);
    PCWS_APPLY_DOUBLE(denoise_radius_mm, denoiseRadius);
    PCWS_APPLY_DOUBLE(bend_deviation_mm, bendDeviation);
    PCWS_APPLY_DOUBLE(trajectory_spacing_mm, trajectorySpacing);
    PCWS_APPLY_DOUBLE(white_point_spacing_mm, whitePointSpacing);
    PCWS_APPLY_DOUBLE(line_fit_tolerance_mm, lineFitTolerance);
    PCWS_APPLY_DOUBLE(max_surface_extrapolation_mm, maxSurfaceExtrapolation);
    PCWS_APPLY_DOUBLE(surface_fit_tolerance_mm, surfaceFitTolerance);
    PCWS_APPLY_DOUBLE(min_intersection_angle_degrees,
                      minIntersectionAngleDegrees);
    PCWS_APPLY_INT(plane_iterations, planeIterations);
    PCWS_APPLY_INT(plane_candidates, planeCandidates);
    PCWS_APPLY_INT(max_ransac_points, maxRansacPoints);
    PCWS_APPLY_INT(min_plane_points, minPlanePoints);
    PCWS_APPLY_INT(prefilter_min_support, prefilterMinSupport);
    PCWS_APPLY_INT(min_neighbors, minNeighbors);
    PCWS_APPLY_INT(min_surface_points, minSurfacePoints);
    PCWS_APPLY_INT(threads, threads);
#undef PCWS_APPLY_INT
#undef PCWS_APPLY_DOUBLE
    if (config.threads < 0) {
        error = "threads 不能为负数。";
        return false;
    }
    return true;
}

bool direction(const PcwsPoint3d &input, bool required, bwb::Vec3 &output,
               std::string &error, const char *name) {
    if (!finite(input)) {
        error = std::string(name) + " 含 NaN 或 Inf。";
        return false;
    }
    output = vector(input);
    const double length = output.norm();
    if (length <= 1e-12) {
        if (required) {
            error = std::string(name) + " 不能为空方向。";
            return false;
        }
        output.setZero();
        return true;
    }
    output /= length;
    return true;
}

std::uint64_t frameFor(const bwb::ProjectedPoint &candidate,
                       const std::vector<std::uint64_t> &frames) {
    return candidate.source < frames.size() ? frames[candidate.source] : 0;
}

bool shouldReverse(const std::vector<std::uint64_t> &frames,
                   const std::vector<bwb::ProjectedPoint> &base,
                   const bwb::Vec3 &scanDirection) {
    if (base.size() < 2)
        return false;
    if (scanDirection.squaredNorm() > 0.0)
        return (base.back().xyz - base.front().xyz).dot(scanDirection) < 0.0;
    const std::size_t sampleCount = std::min<std::size_t>(8, base.size());
    long double first = 0.0, last = 0.0;
    for (std::size_t i = 0; i < sampleCount; ++i) {
        first += frameFor(base[i], frames);
        last += frameFor(base[base.size() - 1 - i], frames);
    }
    return first > last;
}

bwb::Vec3 automaticProfileDirection(const bwb::Result &result) {
    bwb::Vec3 profile = result.referenceTransverse;
    if (!profile.allFinite() || profile.norm() <= 1e-12)
        profile = bwb::Vec3::UnitY();
    profile.normalize();
    Eigen::Index component = 0;
    profile.cwiseAbs().maxCoeff(&component);
    if (profile[component] < 0.0)
        profile = -profile;
    return profile;
}

std::uint32_t pointType(const std::vector<bwb::ProjectedPoint> &classified,
                        std::size_t index, const bwb::Vec3 &profile) {
    if (index == 0)
        return PCWS_POINT_START;
    if (index + 1 == classified.size())
        return PCWS_POINT_END;
    if (!classified[index].corner)
        return PCWS_POINT_NORMAL;
    const bwb::Vec3 &before = classified[index - 1].xyz;
    const bwb::Vec3 &current = classified[index].xyz;
    const bwb::Vec3 &after = classified[index + 1].xyz;
    const double left = (current - before).norm();
    const double right = (after - current).norm();
    if (left <= 1e-12 || right <= 1e-12)
        return PCWS_POINT_NORMAL;
    const bwb::Vec3 chord = before + (after - before) * (left / (left + right));
    const double signedBulge = profile.dot(current - chord);
    if (std::abs(signedBulge) <= 1e-8)
        return PCWS_POINT_NORMAL;
    return signedBulge > 0.0 ? PCWS_POINT_INNER_CORNER
                             : PCWS_POINT_OUTER_CORNER;
}

std::uint32_t segmentKind(const std::vector<bwb::ProjectedPoint> &classified,
                          std::size_t index, const bwb::Vec3 &profile,
                          double minimumHeight, double maximumHeight) {
    if (index + 1 >= classified.size())
        return PCWS_SEGMENT_NONE;
    const bwb::Vec3 delta = classified[index + 1].xyz - classified[index].xyz;
    const double height = profile.dot(delta);
    const double tolerance = std::max(1e-6, delta.norm() * 0.05);
    if (height > tolerance)
        return PCWS_SEGMENT_RISING_EDGE;
    if (height < -tolerance)
        return PCWS_SEGMENT_FALLING_EDGE;
    const double level = profile.dot(
        (classified[index].xyz + classified[index + 1].xyz) * 0.5);
    return level >= (minimumHeight + maximumHeight) * 0.5
               ? PCWS_SEGMENT_HIGH_PLATFORM
               : PCWS_SEGMENT_LOW_PLATFORM;
}

std::vector<std::uint64_t> baseIndices(
    const std::vector<bwb::ProjectedPoint> &base,
    const std::vector<bwb::ProjectedPoint> &classified) {
    std::vector<std::uint64_t> result(classified.size(), 0);
    if (base.empty() || classified.empty())
        return result;
    std::size_t lower = 0;
    for (std::size_t i = 0; i < classified.size(); ++i) {
        if (i + 1 == classified.size()) {
            result[i] = std::uint64_t(base.size() - 1);
            break;
        }
        double bestDistance = std::numeric_limits<double>::infinity();
        std::size_t best = lower;
        for (std::size_t j = lower; j < base.size(); ++j) {
            const double distance =
                (classified[i].xyz - base[j].xyz).squaredNorm();
            if (distance < bestDistance) {
                bestDistance = distance;
                best = j;
            }
        }
        result[i] = std::uint64_t(best);
        lower = best;
    }
    result.front() = 0;
    result.back() = std::uint64_t(base.size() - 1);
    return result;
}

PcwsStatus fail(PcwsResultV1 *result, PcwsStatus status,
                const std::string &message) {
    setError(result, message);
    return status;
}

} // namespace

extern "C" PcwsStatus PCWS_CALL PcwsLoadOptionsV1(
    const char *parametersJsonPathUtf8, PcwsOptionsV1 *outOptions,
    char errorUtf8[256]) {
    setErrorBuffer(errorUtf8, 256, {});
    if (!outOptions) {
        setErrorBuffer(errorUtf8, 256, "out_options 不能为空。");
        return PCWS_INVALID_ARGUMENT;
    }
    *outOptions = {};
    if (!parametersJsonPathUtf8 || parametersJsonPathUtf8[0] == '\0') {
        setErrorBuffer(errorUtf8, 256, "参数 JSON 路径不能为空。");
        return PCWS_INVALID_ARGUMENT;
    }
    try {
        PcwsOptionsV1 loaded{};
        std::string error;
        if (!pcws::loadOptionsFile(parametersJsonPathUtf8, loaded, error)) {
            setErrorBuffer(errorUtf8, 256, error);
            return PCWS_INVALID_ARGUMENT;
        }
        bwb::Config config;
        if (!applyOptions(&loaded, config, error)) {
            setErrorBuffer(errorUtf8, 256, error);
            return PCWS_INVALID_ARGUMENT;
        }
        error = bwb::validateConfig(config);
        if (!error.empty()) {
            setErrorBuffer(errorUtf8, 256, error);
            return PCWS_INVALID_ARGUMENT;
        }
        *outOptions = loaded;
        return PCWS_OK;
    } catch (const std::bad_alloc &) {
        setErrorBuffer(errorUtf8, 256, "读取参数时内存分配失败。");
        return PCWS_RESOURCE_EXHAUSTED;
    } catch (const std::exception &exception) {
        setErrorBuffer(errorUtf8, 256, exception.what());
        return PCWS_INTERNAL_ERROR;
    } catch (...) {
        setErrorBuffer(errorUtf8, 256, "读取参数时发生未知内部错误。");
        return PCWS_INTERNAL_ERROR;
    }
}

extern "C" PcwsStatus PCWS_CALL PcwsProcessV1(
    const PcwsRequestV1 *request, PcwsResultV1 *outResult) {
    if (!outResult)
        return PCWS_INVALID_ARGUMENT;
    if (!empty(*outResult))
        return fail(outResult, PCWS_INVALID_ARGUMENT,
                    "out_result 必须先清零或先调用 PcwsReleaseResultV1。");
    *outResult = {};
    if (!request || !request->cloud_mm || request->cloud_count < 2)
        return fail(outResult, PCWS_INVALID_ARGUMENT,
                    "request、cloud_mm 不能为空，cloud_count 至少为 2。");
    if (request->cloud_count > std::numeric_limits<std::uint32_t>::max())
        return fail(outResult, PCWS_RESOURCE_EXHAUSTED,
                    "V1 单次调用的点数不能超过 UINT32_MAX。");

    try {
        bwb::Config config;
        std::string error;
        if (!applyOptions(request->options, config, error))
            return fail(outResult, PCWS_INVALID_ARGUMENT, error);
        const std::string configError = bwb::validateConfig(config);
        if (!configError.empty())
            return fail(outResult, PCWS_INVALID_ARGUMENT, configError);

        bwb::Vec3 scanDirection, requestedProfile;
        if (!direction(request->scan_direction, false, scanDirection, error,
                       "scan_direction") ||
            !direction(request->profile_direction, false, requestedProfile,
                       error, "profile_direction"))
            return fail(outResult, PCWS_INVALID_ARGUMENT, error);
        if (scanDirection.squaredNorm() > 0.0 &&
            requestedProfile.squaredNorm() > 0.0 &&
            std::abs(scanDirection.dot(requestedProfile)) > 0.999)
            return fail(outResult, PCWS_INVALID_ARGUMENT,
                        "profile_direction 不能与 scan_direction 平行。");

        std::vector<bwb::Point> points;
        std::vector<std::uint64_t> frames(
            std::size_t(request->cloud_count), 0);
        points.reserve(std::size_t(request->cloud_count));
        for (std::uint64_t i = 0; i < request->cloud_count; ++i) {
            const auto &input = request->cloud_mm[i];
            frames[std::size_t(i)] = input.frame_number;
            if (!finite(input.position_mm))
                continue;
            points.push_back(
                {vector(input.position_mm), static_cast<std::uint32_t>(i)});
        }
        if (points.size() < 2)
            return fail(outResult, PCWS_INVALID_ARGUMENT,
                        "有限 XYZ 点少于 2 个。");
        if (request->should_cancel &&
            request->should_cancel(request->cancel_user_data) != 0)
            return fail(outResult, PCWS_CANCELED, "调用已取消。");

        bwb::Hooks hooks;
        if (request->should_cancel) {
            hooks.cancelled = [request] {
                return request->should_cancel(request->cancel_user_data) != 0;
            };
        }
        bwb::Result extracted = bwb::extract(points, config, hooks);
        if (extracted.cancelled)
            return fail(outResult, PCWS_CANCELED, "调用已取消。");
        if (!extracted.ok || extracted.projected.size() < 2) {
            return fail(outResult, PCWS_NO_SEAM,
                        extracted.message.empty()
                            ? "无法建立有效的 C 特征候选曲线。"
                            : extracted.message);
        }
        if (extracted.whitePoints.size() < 2)
            return fail(outResult, PCWS_CLASSIFICATION_FAILED,
                        extracted.message.empty()
                            ? "C 特征候选已建立，但未生成白色均匀点。"
                            : extracted.message);

        std::vector<bwb::ProjectedPoint> base = extracted.projected;
        std::vector<bwb::ProjectedPoint> classified = extracted.whitePoints;
        if (shouldReverse(frames, base, scanDirection)) {
            std::reverse(base.begin(), base.end());
            std::reverse(classified.begin(), classified.end());
        }
        bwb::Vec3 profile = requestedProfile.squaredNorm() > 0.0
                                ? requestedProfile
                                : automaticProfileDirection(extracted);
        if (scanDirection.squaredNorm() > 0.0) {
            profile -= scanDirection * profile.dot(scanDirection);
            if (profile.norm() <= 1e-12)
                return fail(outResult, PCWS_INVALID_ARGUMENT,
                            "profile_direction 在垂直扫描方向上的分量为零。");
            profile.normalize();
        }

        double minimumHeight = std::numeric_limits<double>::infinity();
        double maximumHeight = -std::numeric_limits<double>::infinity();
        for (const auto &candidate : classified) {
            const double height = profile.dot(candidate.xyz);
            minimumHeight = std::min(minimumHeight, height);
            maximumHeight = std::max(maximumHeight, height);
        }
        const auto indices = baseIndices(base, classified);
        std::unique_ptr<PcwsPoint3d[]> sdkbase(
            new PcwsPoint3d[base.size()]);
        std::unique_ptr<PcwsKeyPointV1[]> sdkclass(
            new PcwsKeyPointV1[classified.size()]);
        for (std::size_t i = 0; i < base.size(); ++i)
            sdkbase[i] = point(base[i].xyz);
        for (std::size_t i = 0; i < classified.size(); ++i) {
            sdkclass[i].position_mm = point(classified[i].xyz);
            sdkclass[i].base_index = indices[i];
            sdkclass[i].point_type = pointType(classified, i, profile);
            sdkclass[i].segment_kind_after =
                segmentKind(classified, i, profile, minimumHeight, maximumHeight);
        }

        outResult->sdkbase = sdkbase.release();
        outResult->sdkbase_count = std::uint64_t(base.size());
        outResult->sdkclass = sdkclass.release();
        outResult->sdkclass_count = std::uint64_t(classified.size());
        outResult->error_utf8[0] = '\0';
        return PCWS_OK;
    } catch (const std::bad_alloc &) {
        return fail(outResult, PCWS_RESOURCE_EXHAUSTED, "SDK 内存分配失败。");
    } catch (const std::exception &exception) {
        return fail(outResult, PCWS_INTERNAL_ERROR, exception.what());
    } catch (...) {
        return fail(outResult, PCWS_INTERNAL_ERROR, "SDK 发生未知内部错误。");
    }
}

extern "C" void PCWS_CALL PcwsReleaseResultV1(PcwsResultV1 *result) {
    if (!result)
        return;
    delete[] result->sdkbase;
    delete[] result->sdkclass;
    *result = {};
}
