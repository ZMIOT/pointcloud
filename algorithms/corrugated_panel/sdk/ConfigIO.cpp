#include "ConfigIO.h"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string_view>

namespace {

struct JsonValue {
    enum class Type { Number, Boolean } type = Type::Number;
    double number = 0.0;
    bool boolean = false;
};

class FlatJsonParser {
public:
    explicit FlatJsonParser(std::string_view text) : text_(text) {}

    bool parse(std::map<std::string, JsonValue> &values, std::string &error) {
        skipSpace();
        if (!consume('{'))
            return fail(error, "参数 JSON 根必须是对象。");
        skipSpace();
        if (consume('}'))
            return finish(error);
        for (;;) {
            std::string key;
            if (!parseString(key, error))
                return false;
            skipSpace();
            if (!consume(':'))
                return fail(error, "参数名后缺少冒号。");
            skipSpace();
            JsonValue value;
            if (!parseValue(value, error))
                return false;
            if (!values.emplace(key, value).second)
                return fail(error, "参数 JSON 存在重复字段：" + key);
            skipSpace();
            if (consume('}'))
                return finish(error);
            if (!consume(','))
                return fail(error, "参数字段后缺少逗号或右花括号。");
            skipSpace();
        }
    }

private:
    void skipSpace() {
        while (position_ < text_.size() &&
               (text_[position_] == ' ' || text_[position_] == '\t' ||
                text_[position_] == '\r' || text_[position_] == '\n'))
            ++position_;
    }

    bool consume(char expected) {
        if (position_ >= text_.size() || text_[position_] != expected)
            return false;
        ++position_;
        return true;
    }

    bool parseString(std::string &value, std::string &error) {
        if (!consume('"'))
            return fail(error, "参数名必须是 JSON 字符串。");
        while (position_ < text_.size()) {
            const char character = text_[position_++];
            if (character == '"')
                return true;
            if (static_cast<unsigned char>(character) < 0x20)
                return fail(error, "JSON 字符串包含控制字符。");
            if (character != '\\') {
                value.push_back(character);
                continue;
            }
            if (position_ >= text_.size())
                return fail(error, "JSON 字符串转义不完整。");
            const char escaped = text_[position_++];
            switch (escaped) {
            case '"': value.push_back('"'); break;
            case '\\': value.push_back('\\'); break;
            case '/': value.push_back('/'); break;
            case 'b': value.push_back('\b'); break;
            case 'f': value.push_back('\f'); break;
            case 'n': value.push_back('\n'); break;
            case 'r': value.push_back('\r'); break;
            case 't': value.push_back('\t'); break;
            default:
                return fail(error, "参数名包含不支持的 JSON 转义。");
            }
        }
        return fail(error, "JSON 字符串未结束。");
    }

    bool parseValue(JsonValue &value, std::string &error) {
        if (text_.substr(position_, 4) == "true") {
            position_ += 4;
            value.type = JsonValue::Type::Boolean;
            value.boolean = true;
            return true;
        }
        if (text_.substr(position_, 5) == "false") {
            position_ += 5;
            value.type = JsonValue::Type::Boolean;
            value.boolean = false;
            return true;
        }
        const std::size_t begin = position_;
        if (position_ < text_.size() && text_[position_] == '-')
            ++position_;
        const std::size_t integerBegin = position_;
        while (position_ < text_.size() && text_[position_] >= '0' &&
               text_[position_] <= '9')
            ++position_;
        if (position_ == integerBegin)
            return fail(error, "参数值必须是数字或布尔值。");
        if (position_ < text_.size() && text_[position_] == '.') {
            ++position_;
            const std::size_t fractionBegin = position_;
            while (position_ < text_.size() && text_[position_] >= '0' &&
                   text_[position_] <= '9')
                ++position_;
            if (position_ == fractionBegin)
                return fail(error, "JSON 小数点后缺少数字。");
        }
        if (position_ < text_.size() &&
            (text_[position_] == 'e' || text_[position_] == 'E')) {
            ++position_;
            if (position_ < text_.size() &&
                (text_[position_] == '+' || text_[position_] == '-'))
                ++position_;
            const std::size_t exponentBegin = position_;
            while (position_ < text_.size() && text_[position_] >= '0' &&
                   text_[position_] <= '9')
                ++position_;
            if (position_ == exponentBegin)
                return fail(error, "JSON 指数后缺少数字。");
        }
        const auto token = text_.substr(begin, position_ - begin);
        double number = 0.0;
        const auto converted =
            std::from_chars(token.data(), token.data() + token.size(), number);
        if (converted.ec != std::errc{} || converted.ptr != token.data() + token.size() ||
            !std::isfinite(number))
            return fail(error, "参数数字无效或超出范围。");
        value.type = JsonValue::Type::Number;
        value.number = number;
        return true;
    }

    bool finish(std::string &error) {
        skipSpace();
        if (position_ != text_.size())
            return fail(error, "参数 JSON 根对象后存在多余内容。");
        return true;
    }

    bool fail(std::string &error, const std::string &message) const {
        error = message + "（字节位置 " + std::to_string(position_) + "）";
        return false;
    }

    std::string_view text_;
    std::size_t position_ = 0;
};

bool number(const std::map<std::string, JsonValue> &values,
            const char *key, double &destination, std::string &error) {
    const auto found = values.find(key);
    if (found == values.end())
        return true;
    if (found->second.type != JsonValue::Type::Number) {
        error = std::string(key) + " 必须是数字。";
        return false;
    }
    destination = found->second.number;
    return true;
}

bool integer(const std::map<std::string, JsonValue> &values,
             const char *key, std::int32_t &destination, std::string &error) {
    const auto found = values.find(key);
    if (found == values.end())
        return true;
    if (found->second.type != JsonValue::Type::Number ||
        std::floor(found->second.number) != found->second.number ||
        found->second.number < std::numeric_limits<std::int32_t>::min() ||
        found->second.number > std::numeric_limits<std::int32_t>::max()) {
        error = std::string(key) + " 必须是 32 位整数。";
        return false;
    }
    destination = static_cast<std::int32_t>(found->second.number);
    return true;
}

bool boolean(const std::map<std::string, JsonValue> &values,
             const char *key, bool &destination, std::string &error) {
    const auto found = values.find(key);
    if (found == values.end())
        return true;
    if (found->second.type != JsonValue::Type::Boolean) {
        error = std::string(key) + " 必须是 true 或 false。";
        return false;
    }
    destination = found->second.boolean;
    return true;
}

} // namespace

namespace pcws {

bool loadOptionsFile(const std::string &path, PcwsOptionsV1 &options,
                     std::string &error) {
    options = {};
    if (path.empty()) {
        error = "参数 JSON 路径不能为空。";
        return false;
    }
    std::ifstream file(std::filesystem::u8path(path), std::ios::binary);
    if (!file) {
        error = "无法打开参数 JSON：" + path;
        return false;
    }
    file.seekg(0, std::ios::end);
    const std::streamoff length = file.tellg();
    if (length <= 0 || length > 1024 * 1024) {
        error = "参数 JSON 为空、不可读取或超过 1 MiB。";
        return false;
    }
    file.seekg(0, std::ios::beg);
    std::string contents(static_cast<std::size_t>(length), '\0');
    if (!file.read(contents.data(), static_cast<std::streamsize>(length))) {
        error = "读取参数 JSON 失败：" + path;
        return false;
    }

    std::map<std::string, JsonValue> values;
    FlatJsonParser parser(contents);
    if (!parser.parse(values, error))
        return false;
    static const char *known[] = {
        "schema_version", "prefilter_enabled", "prefilter_window_mm",
        "prefilter_distance_mm", "prefilter_min_support", "slice_offset_1_mm",
        "slice_offset_2_mm", "slice_offset_3_mm", "slice_thickness_mm",
        "plane_distance_mm", "reference_segment_length_mm",
        "max_adjacent_normal_angle_degrees", "plane_iterations",
        "plane_candidates", "max_ransac_points", "min_plane_points",
        "denoise_radius_mm", "min_neighbors", "bend_deviation_mm",
        "trajectory_spacing_mm", "line_fit_tolerance_mm",
        "white_point_spacing_mm", "max_surface_extrapolation_mm",
        "surface_fit_tolerance_mm", "min_surface_points",
        "min_intersection_angle_degrees", "allow_projection_fallback", "threads"};
    for (const auto &entry : values) {
        bool recognized = false;
        for (const char *key : known)
            recognized = recognized || entry.first == key;
        if (!recognized) {
            error = "未知参数字段：" + entry.first;
            return false;
        }
    }
    const auto schema = values.find("schema_version");
    if (schema != values.end() &&
        (schema->second.type != JsonValue::Type::Number ||
         schema->second.number != 1.0)) {
        error = "schema_version 必须为 1。";
        return false;
    }

    options.struct_size = sizeof(PcwsOptionsV1);
    options.slice_offset_1_mm = 10.0;
    options.slice_offset_2_mm = 20.0;
    options.slice_offset_3_mm = 30.0;
    options.slice_thickness_mm = 3.0;
    options.plane_distance_mm = 0.6;
    options.reference_segment_length_mm = 500.0;
    options.max_adjacent_normal_angle_degrees = 15.0;
    options.prefilter_window_mm = 12.0;
    options.prefilter_distance_mm = 0.8;
    options.denoise_radius_mm = 2.5;
    options.bend_deviation_mm = 2.0;
    options.trajectory_spacing_mm = 2.0;
    options.white_point_spacing_mm = 10.0;
    options.line_fit_tolerance_mm = 3.0;
    options.max_surface_extrapolation_mm = 40.0;
    options.surface_fit_tolerance_mm = 0.6;
    options.min_intersection_angle_degrees = 10.0;
    options.plane_iterations = 600;
    options.plane_candidates = 6;
    options.max_ransac_points = 60000;
    options.min_plane_points = 200;
    options.prefilter_min_support = 12;
    options.min_neighbors = 3;
    options.min_surface_points = 20;
    options.threads = 0;
    bool prefilter = true, projectionFallback = true;
    if (!boolean(values, "prefilter_enabled", prefilter, error) ||
        !boolean(values, "allow_projection_fallback", projectionFallback, error) ||
        !number(values, "slice_offset_1_mm", options.slice_offset_1_mm, error) ||
        !number(values, "slice_offset_2_mm", options.slice_offset_2_mm, error) ||
        !number(values, "slice_offset_3_mm", options.slice_offset_3_mm, error) ||
        !number(values, "slice_thickness_mm", options.slice_thickness_mm, error) ||
        !number(values, "plane_distance_mm", options.plane_distance_mm, error) ||
        !number(values, "reference_segment_length_mm",
                options.reference_segment_length_mm, error) ||
        !number(values, "max_adjacent_normal_angle_degrees",
                options.max_adjacent_normal_angle_degrees, error) ||
        !number(values, "prefilter_window_mm", options.prefilter_window_mm, error) ||
        !number(values, "prefilter_distance_mm", options.prefilter_distance_mm, error) ||
        !number(values, "denoise_radius_mm", options.denoise_radius_mm, error) ||
        !number(values, "bend_deviation_mm", options.bend_deviation_mm, error) ||
        !number(values, "trajectory_spacing_mm", options.trajectory_spacing_mm, error) ||
        !number(values, "white_point_spacing_mm", options.white_point_spacing_mm, error) ||
        !number(values, "line_fit_tolerance_mm", options.line_fit_tolerance_mm, error) ||
        !number(values, "max_surface_extrapolation_mm",
                options.max_surface_extrapolation_mm, error) ||
        !number(values, "surface_fit_tolerance_mm",
                options.surface_fit_tolerance_mm, error) ||
        !number(values, "min_intersection_angle_degrees",
                options.min_intersection_angle_degrees, error) ||
        !integer(values, "plane_iterations", options.plane_iterations, error) ||
        !integer(values, "plane_candidates", options.plane_candidates, error) ||
        !integer(values, "max_ransac_points", options.max_ransac_points, error) ||
        !integer(values, "min_plane_points", options.min_plane_points, error) ||
        !integer(values, "prefilter_min_support", options.prefilter_min_support, error) ||
        !integer(values, "min_neighbors", options.min_neighbors, error) ||
        !integer(values, "min_surface_points", options.min_surface_points, error) ||
        !integer(values, "threads", options.threads, error)) {
        options = {};
        return false;
    }
    if (!prefilter)
        options.flags |= PCWS_OPTION_DISABLE_SURFACE_PREFILTER;
    if (!projectionFallback)
        options.flags |= PCWS_OPTION_DISABLE_PROJECTION_FALLBACK;
    error.clear();
    return true;
}

} // namespace pcws
