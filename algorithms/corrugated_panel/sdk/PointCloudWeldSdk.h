#ifndef POINT_CLOUD_WELD_SDK_H
#define POINT_CLOUD_WELD_SDK_H

#include <stddef.h>
#include <stdint.h>

/*
 * 波纹板特征提取 SDK V1。
 *
 * 输入为带帧号的 XYZ 点云；输出 sdkbase 为算法的 C 特征候选点，sdkclass
 * 为拟合折线上的白色均匀点。SDK 不读写点云、配置、结果或调试文件。
 * Windows 交付物为 x64 Release DLL、配套 .lib 和本头文件。
 */

#if defined(_WIN32)
#if defined(PCWS_SDK_EXPORTS)
#define PCWS_API __declspec(dllexport)
#else
#define PCWS_API __declspec(dllimport)
#endif
#define PCWS_CALL __cdecl
#else
#define PCWS_API
#define PCWS_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 8)

typedef struct PcwsPoint3d {
    double x;
    double y;
    double z;
} PcwsPoint3d;

/* 同一帧可包含多个点；frame_number 仅用于确定扫描先后，不要求连续。 */
typedef struct PcwsFramePointV1 {
    uint64_t frame_number;
    PcwsPoint3d position_mm;
} PcwsFramePointV1;

typedef int32_t PcwsStatus;
enum {
    PCWS_OK = 0,
    PCWS_INVALID_ARGUMENT = 1,
    PCWS_NO_SEAM = 2,
    PCWS_INCOMPLETE_BASE = 3,
    PCWS_CLASSIFICATION_FAILED = 4,
    PCWS_CANCELED = 5,
    PCWS_RESOURCE_EXHAUSTED = 6,
    PCWS_INTERNAL_ERROR = 7
};

enum {
    PCWS_POINT_START = 1,
    PCWS_POINT_END = 2,
    PCWS_POINT_INNER_CORNER = 3,
    PCWS_POINT_OUTER_CORNER = 4,
    PCWS_POINT_NORMAL = 5
};

enum {
    PCWS_SEGMENT_NONE = 0,
    PCWS_SEGMENT_LOW_PLATFORM = 1,
    PCWS_SEGMENT_RISING_EDGE = 2,
    PCWS_SEGMENT_HIGH_PLATFORM = 3,
    PCWS_SEGMENT_FALLING_EDGE = 4,
    PCWS_SEGMENT_LAP_STEP = 5
};

typedef struct PcwsKeyPointV1 {
    PcwsPoint3d position_mm;
    uint64_t base_index;         /* 在本次 sdkbase 中从 0 起算的最近顺序索引。 */
    uint32_t point_type;         /* PCWS_POINT_* */
    uint32_t segment_kind_after; /* PCWS_SEGMENT_* */
} PcwsKeyPointV1;

enum {
    PCWS_OPTION_DISABLE_SURFACE_PREFILTER = 1u << 0,
    PCWS_OPTION_DISABLE_PROJECTION_FALLBACK = 1u << 1
};

/*
 * NULL 或全零时使用算法内置缺省值。PcwsLoadOptionsV1 会填充完整快照并将
 * struct_size 设为 sizeof(PcwsOptionsV1)。手工只覆盖少量参数时保持
 * struct_size=0，非零数值覆盖对应缺省值；手工提供完整快照时填写结构尺寸。
 */
typedef struct PcwsOptionsV1 {
    uint32_t struct_size;
    uint32_t flags;
    double slice_offset_1_mm;
    double slice_offset_2_mm;
    double slice_offset_3_mm;
    double slice_thickness_mm;
    double plane_distance_mm;
    double reference_segment_length_mm;
    double max_adjacent_normal_angle_degrees;
    double prefilter_window_mm;
    double prefilter_distance_mm;
    double denoise_radius_mm;
    double bend_deviation_mm;
    double trajectory_spacing_mm;
    double white_point_spacing_mm;
    double line_fit_tolerance_mm;
    double max_surface_extrapolation_mm;
    double surface_fit_tolerance_mm;
    double min_intersection_angle_degrees;
    int32_t plane_iterations;
    int32_t plane_candidates;
    int32_t max_ransac_points;
    int32_t min_plane_points;
    int32_t prefilter_min_support;
    int32_t min_neighbors;
    int32_t min_surface_points;
    int32_t threads;
} PcwsOptionsV1;

typedef int32_t(PCWS_CALL *PcwsCancelFn)(void *user_data);

typedef struct PcwsRequestV1 {
    const PcwsFramePointV1 *cloud_mm;
    uint64_t cloud_count;
    /* 可为全零。非零时用于规定 sdkbase/sdkclass 的首尾方向。 */
    PcwsPoint3d scan_direction;
    /* 可为全零。非零时用于内外角及平台高低分类；全零时由点云自动确定。 */
    PcwsPoint3d profile_direction;
    const PcwsOptionsV1 *options;
    PcwsCancelFn should_cancel;
    void *cancel_user_data;
} PcwsRequestV1;

typedef struct PcwsResultV1 {
    const PcwsPoint3d *sdkbase;      /* C 特征候选点。 */
    uint64_t sdkbase_count;
    const PcwsKeyPointV1 *sdkclass;  /* 白色均匀点及其顺序属性。 */
    uint64_t sdkclass_count;
    char error_utf8[256];
} PcwsResultV1;

#pragma pack(pop)

/*
 * 调用约定：
 * - 同步调用；输入只读且仅在调用期间有效。NaN/Inf 点会被忽略。
 * - cloud_mm 至少需要两个有限点，实际无法建立完整特征曲线时返回 PCWS_NO_SEAM。
 * - scan_direction 为零时，优先按输出点关联的 frame_number 递增排列。
 * - 成功时同时返回 sdkbase 和 sdkclass；失败时两组数组均为空。
 * - 数组由 DLL 分配，必须由同一 DLL 的 PcwsReleaseResultV1 释放。
 * - 允许独立请求并发；每次调用必须使用独立且已清零的 PcwsResultV1。
 */
PCWS_API PcwsStatus PCWS_CALL PcwsProcessV1(const PcwsRequestV1 *request,
                                            PcwsResultV1 *out_result);

/*
 * 显式读取 UTF-8 JSON 参数文件。成功时 out_options 可直接赋给 request.options；
 * 失败时 out_options 清零并在 error_utf8 写入原因。SDK 不自动搜索文件。
 */
PCWS_API PcwsStatus PCWS_CALL PcwsLoadOptionsV1(
    const char *parameters_json_path_utf8,
    PcwsOptionsV1 *out_options,
    char error_utf8[256]);

PCWS_API void PCWS_CALL PcwsReleaseResultV1(PcwsResultV1 *result);

#ifdef __cplusplus
} /* extern "C" */
#endif

#if defined(_M_X64) || defined(__x86_64__)
#if defined(__cplusplus)
#define PCWS_STATIC_ASSERT static_assert
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define PCWS_STATIC_ASSERT _Static_assert
#endif
#ifdef PCWS_STATIC_ASSERT
PCWS_STATIC_ASSERT(sizeof(PcwsPoint3d) == 24, "PcwsPoint3d ABI");
PCWS_STATIC_ASSERT(sizeof(PcwsFramePointV1) == 32 &&
                       offsetof(PcwsFramePointV1, position_mm) == 8,
                   "PcwsFramePointV1 ABI");
PCWS_STATIC_ASSERT(sizeof(PcwsKeyPointV1) == 40 &&
                       offsetof(PcwsKeyPointV1, base_index) == 24,
                   "PcwsKeyPointV1 ABI");
PCWS_STATIC_ASSERT(sizeof(PcwsOptionsV1) == 176, "PcwsOptionsV1 ABI");
PCWS_STATIC_ASSERT(sizeof(PcwsRequestV1) == 88 &&
                       offsetof(PcwsRequestV1, options) == 64,
                   "PcwsRequestV1 ABI");
PCWS_STATIC_ASSERT(sizeof(PcwsResultV1) == 288 &&
                       offsetof(PcwsResultV1, error_utf8) == 32,
                   "PcwsResultV1 ABI");
#undef PCWS_STATIC_ASSERT
#endif
#endif

#endif /* POINT_CLOUD_WELD_SDK_H */
