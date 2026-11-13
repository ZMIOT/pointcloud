#include "SurfaceIntersection.h"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <numeric>

namespace bwb {
namespace {
constexpr double pi = 3.14159265358979323846;
std::string failureLabel(const std::string &code) {
    if (code == "surface_fit_residual") return "表面拟合残差超限";
    if (code == "surface_extrapolation_limit") return "沿表面的外推弧长超限";
    if (code == "near_tangent_intersection") return "表面与基准面接近相切";
    if (code == "ambiguous_surface_branch") return "存在无法区分的表面分支";
    if (code == "missing_B_seed") return "本长度站位缺少 B 种子";
    if (code == "insufficient_surface_support") return "连续表面支持点不足";
    if (code == "insufficient_height_span") return "表面支持数量或高度跨度不足";
    return "无法建立有效截面坐标系";
}
double median(std::vector<double> values) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const auto n = values.size();
    return (values[(n - 1) / 2] + values[n / 2]) * 0.5;
}
struct Sample { double h, t, s; std::uint32_t source; int layer; };
struct Model {
    Eigen::Vector4d coefficients = Eigen::Vector4d::Zero();
    double center = 0.0, scale = 1.0, sScale = 1.0, rms = 0.0;
    int degree = 1;
    bool valid = false;
    Eigen::Vector4d basis(const Sample &p) const {
        const double z = (p.h - center) / scale;
        return {1.0, z, p.s / sScale, degree == 2 ? z * z : 0.0};
    }
    double value(double h, double s = 0.0) const {
        return coefficients.dot(basis({h, 0.0, s, 0, 0}));
    }
    double slope(double h) const {
        return (coefficients[1] + 2.0 * coefficients[3] * (h - center) / scale) / scale;
    }
};

Model fit(const std::vector<Sample> &samples, int degree, double tolerance,
          double halfWidth, int heldParity = -1) {
    Model model;
    model.degree = degree;
    model.sScale = halfWidth;
    std::vector<Sample> training;
    std::map<int, int> layers;
    for (const auto &p : samples)
        if (heldParity < 0 || std::abs(p.layer % 2) != heldParity) {
            training.push_back(p);
            ++layers[p.layer];
        }
    if (training.size() < 6 || layers.size() < std::size_t(degree + 1)) return model;
    double low = training[0].h, high = low;
    for (const auto &p : training) { low = std::min(low, p.h); high = std::max(high, p.h); }
    if (high - low < 1e-6) return model;
    model.center = (low + high) * 0.5;
    model.scale = std::max(1.0, high - low);
    for (int iteration = 0; iteration < 6; ++iteration) {
        Eigen::Matrix4d gram = Eigen::Matrix4d::Identity() * 1e-10;
        Eigen::Vector4d rhs = Eigen::Vector4d::Zero();
        for (const auto &p : training) {
            const auto row = model.basis(p);
            const double error = std::abs(p.t - row.dot(model.coefficients));
            const double robust = iteration == 0 || error <= tolerance ? 1.0 : tolerance / error;
            const double weight = robust / double(layers[p.layer]);
            gram.noalias() += weight * row * row.transpose();
            rhs.noalias() += weight * row * p.t;
        }
        model.coefficients = gram.ldlt().solve(rhs);
        if (!model.coefficients.allFinite()) return model;
    }
    double sum = 0.0, weight = 0.0;
    for (const auto &p : training) {
        const double residual = p.t - model.value(p.h, p.s);
        const double w = 1.0 / double(layers[p.layer]);
        sum += w * residual * residual;
        weight += w;
    }
    model.rms = std::sqrt(sum / weight);
    model.valid = std::isfinite(model.rms);
    return model;
}

double validationError(const std::vector<Sample> &samples, int degree,
                       double tolerance, double halfWidth) {
    double sum = 0.0;
    std::size_t count = 0;
    for (int parity = 0; parity < 2; ++parity) {
        const auto model = fit(samples, degree, tolerance, halfWidth, parity);
        if (!model.valid) return std::numeric_limits<double>::infinity();
        std::map<int, std::vector<double>> residuals;
        for (const auto &p : samples)
            if (std::abs(p.layer % 2) == parity)
                residuals[p.layer].push_back(std::abs(p.t - model.value(p.h, p.s)));
        for (const auto &layer : residuals) { const double r = median(layer.second); sum += r*r; ++count; }
    }
    return count ? std::sqrt(sum / double(count)) : std::numeric_limits<double>::infinity();
}

double arcLength(const Model &model, double low, double high, double depthFactor) {
    double length = 0.0;
    for (int i = 0; i < 64; ++i) {
        const double h = low + (high - low) * (i + 0.5) / 64.0;
        length += std::hypot(1.0 / depthFactor, model.slope(h));
    }
    return length * (high - low) / 64.0;
}
}

bool recoverSurfaceIntersection(const std::vector<Point> &points, const Config &config,
                                const Hooks &hooks, Result &result) {
    result.projected.clear();
    result.intersectionStations.clear();
    result.usedProjectionFallback = false;
    result.trajectory.clear();
    result.fittedLine.clear();
    result.whitePoints.clear();
    if (result.selectedIndices.empty() || result.referenceSegments.empty() ||
        result.selectedSlice < 0 || std::size_t(result.selectedSlice) >= result.slices.size()) {
        result.message = "表面求交缺少局部基准面或 B 种子。";
        return false;
    }
    const Vec3 &l = result.referenceLongitudinal;
    const Vec3 &origin = result.referenceOrigin;
    auto coordinate = [&](const Vec3 &p) { return l.dot(p - origin); };
    const int side = result.slices[std::size_t(result.selectedSlice)].side;
    const double spacing = config.trajectorySpacing;
    const double heightStep = std::max(config.sliceThickness * 2.0 / 3.0, config.planeDistance * 2.0);
    // The offset slices deliberately avoid the welded edge. Never walk back
    // into that edge to fit the surface: extend the unwelded body model instead.
    const auto offsets = std::minmax_element(config.sliceOffsets.begin(), config.sliceOffsets.end());
    const double floor = std::max(*offsets.first - config.sliceThickness * 0.5,
                                  config.planeDistance * 1.5);
    const double ceiling = *offsets.second + config.sliceThickness * 0.5;
    result.bodyBandMin = floor;
    result.bodyBandMax = ceiling;
    const double maxSlope = 1.0 / std::tan(config.minIntersectionAngleDegrees * pi / 180.0);
    std::vector<std::pair<double, int>> ordered, seeds;
    ordered.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
        if (points[i].xyz.allFinite()) ordered.emplace_back(coordinate(points[i].xyz), int(i));
    for (int i : result.selectedIndices) seeds.emplace_back(coordinate(points[std::size_t(i)].xyz), i);
    std::sort(ordered.begin(), ordered.end());
    std::sort(seeds.begin(), seeds.end());
    const auto percentile = [&](double q) {
        const double index = q * double(seeds.size() - 1);
        const auto lo = std::size_t(std::floor(index)), hi = std::size_t(std::ceil(index));
        return seeds[lo].first * (1.0 - (index - lo)) + seeds[hi].first * (index - lo);
    };
    const double low = percentile(0.005), high = percentile(0.995);
    const double requestedIntervals = (high-low)/spacing;
    if (!std::isfinite(requestedIntervals) || requestedIntervals > 100000) {
        result.message = "表面求交站位数量过多或坐标范围异常，请检查单位和轨迹间距。";
        return false;
    }
    const int intervals = std::max(1, int(std::lround(requestedIntervals)));
    auto beginAt = [](const auto &items, double s) {
        return std::lower_bound(items.begin(), items.end(), std::make_pair(s, -1));
    };
    for (int stationIndex = 0; stationIndex <= intervals; ++stationIndex) {
        if (hooks.cancelled && hooks.cancelled()) { result.cancelled = true; return false; }
        IntersectionStation station;
        station.longitudinal = low + (high - low) * double(stationIndex) / intervals;
        const double s = station.longitudinal;
        const auto localSeedBegin = beginAt(seeds, s-spacing);
        const auto localSeedEnd = beginAt(seeds, s+spacing);
        // Widen only genuinely sparse sections, not dense corners. A broad
        // window across a bend would mix different longitudinal tangents.
        const double halfWidth = std::distance(localSeedBegin, localSeedEnd) < config.minSurfacePoints/2
            ? spacing*1.5 : spacing;
        const auto found = std::lower_bound(result.referenceSegments.begin(), result.referenceSegments.end(), s,
            [](const ReferenceSegment &segment, double value) { return segment.longitudinalMax < value; });
        const std::size_t ai = found == result.referenceSegments.end() ? result.referenceSegments.size() - 1 :
            std::size_t(found - result.referenceSegments.begin());
        station.point.referenceIndex = std::uint32_t(ai);
        const auto &plane = result.referenceSegments[ai].plane;
        const Vec3 u = plane.normal.cross(l).normalized();
        const Vec3 v = l.cross(u).normalized();
        const double depthFactor = plane.normal.dot(v);
        Vec3 anchor = origin + l * s;
        if (!u.allFinite() || depthFactor < 1e-8) {
            station.failure = "invalid_section_frame";
            result.intersectionStations.push_back(station); continue;
        }
        anchor -= v * ((plane.normal.dot(anchor) + plane.d) / depthFactor);
        std::vector<double> seedTs, seedHs, seedSs;
        std::vector<std::uint32_t> seedSources;
        for (auto it = beginAt(seeds, s - halfWidth); it != seeds.end() && it->first <= s + halfWidth; ++it) {
            const auto &p = points[std::size_t(it->second)].xyz;
            const double h = side * (plane.normal.dot(p) + plane.d);
            if (h < floor - 1e-8 || h > ceiling + 1e-8) continue;
            seedTs.push_back(u.dot(p - anchor));
            seedHs.push_back(h);
            seedSs.push_back(it->first-s);
            seedSources.push_back(points[std::size_t(it->second)].source);
        }
        if (seedTs.empty()) {
            station.failure = "missing_B_seed";
            result.intersectionStations.push_back(station); continue;
        }
        const double meanS = std::accumulate(seedSs.begin(), seedSs.end(), 0.0)/seedSs.size();
        const double meanT = std::accumulate(seedTs.begin(), seedTs.end(), 0.0)/seedTs.size();
        double covarianceST = 0, varianceS = 0;
        for (std::size_t i = 0; i < seedSs.size(); ++i) {
            covarianceST += (seedSs[i]-meanS)*(seedTs[i]-meanT);
            varianceS += (seedSs[i]-meanS)*(seedSs[i]-meanS);
        }
        const double lengthSlope = varianceS > 1e-8 ? covarianceST/varianceS : 0;
        for (std::size_t i = 0; i < seedTs.size(); ++i) seedTs[i] -= lengthSlope*seedSs[i];
        const double seedT = median(seedTs), seedH = median(seedHs);
        std::size_t seedRepresentative = 0;
        for (std::size_t i = 1; i < seedTs.size(); ++i)
            if (std::hypot(seedSs[i], seedTs[i]-seedT) <
                std::hypot(seedSs[seedRepresentative], seedTs[seedRepresentative]-seedT))
                seedRepresentative = i;
        station.projectionCandidate.xyz = anchor + u*seedT;
        station.projectionCandidate.source = seedSources[seedRepresentative];
        station.projectionCandidate.referenceIndex = std::uint32_t(ai);
        station.projectionCandidate.supportCount = std::uint32_t(seedTs.size());
        station.projectionCandidate.projectionFallback = true;
        station.projectionTangent = l-v*(plane.normal.dot(l)/depthFactor) + u*lengthSlope;
        station.hasProjectionCandidate = true;
        const auto transverseAtStation = [&](const Sample &p) { return p.t-lengthSlope*p.s; };
        std::map<int, std::vector<Sample>> layers;
        for (auto it = beginAt(ordered, s - halfWidth); it != ordered.end() && it->first <= s + halfWidth; ++it) {
            const auto &p = points[std::size_t(it->second)];
            const double h = side * (plane.normal.dot(p.xyz) + plane.d);
            if (h < floor - 1e-8 || h > ceiling + 1e-8) continue;
            const int layer = int(std::floor(h / heightStep));
            layers[layer].push_back({h, u.dot(p.xyz - anchor), it->first - s, p.source, layer});
        }
        for (auto &entry : layers)
            std::sort(entry.second.begin(), entry.second.end(), [&](const Sample &a, const Sample &b) {
                return transverseAtStation(a) < transverseAtStation(b);
            });
        std::vector<Sample> support;
        bool ambiguous = false;
        auto choose = [&](int layer, double prediction) -> std::vector<Sample> {
            const auto it = layers.find(layer);
            if (it == layers.end()) return {};
            const auto &raw = it->second;
            struct Cluster { std::size_t first, last; double distance; };
            std::vector<Cluster> clusters;
            const double split = std::max(config.surfaceFitTolerance * 2.0, config.denoiseRadius);
            for (std::size_t first = 0; first < raw.size();) {
                std::size_t last = first + 1;
                while (last < raw.size() && transverseAtStation(raw[last]) - transverseAtStation(raw[last-1]) <= split) ++last;
                // Sparse scan lines can contribute a single point per height
                // layer; the full patch still must pass support/span/residual gates.
                if (last > first) {
                    const double center = (transverseAtStation(raw[(first + last - 1) / 2]) +
                                           transverseAtStation(raw[(first + last) / 2])) * 0.5;
                    clusters.push_back({first, last, std::abs(center - prediction)});
                }
                first = last;
            }
            std::sort(clusters.begin(), clusters.end(), [](const Cluster &a, const Cluster &b) { return a.distance < b.distance; });
            if (clusters.empty() || clusters[0].distance > heightStep * maxSlope + split) return {};
            if (clusters.size() > 1 && clusters[1].distance - clusters[0].distance <= config.surfaceFitTolerance) {
                ambiguous = true; return {};
            }
            return {raw.begin() + clusters[0].first, raw.begin() + clusters[0].last};
        };
        const int seedLayer = int(std::floor(seedH / heightStep));
        auto central = choose(seedLayer, seedT);
        support.insert(support.end(), central.begin(), central.end());
        for (int direction : {-1, 1}) {
            double previousH = seedH, previousT = seedT, slope = 0.0;
            for (int step = 1; step <= int(std::ceil((ceiling - floor) / heightStep)) + 2; ++step) {
                const int layer = seedLayer + direction * step;
                if (layer < 0) break;
                const double targetH = (layer + 0.5) * heightStep;
                auto chosen = choose(layer, previousT + slope * (targetH - previousH));
                if (chosen.empty()) break;
                std::vector<double> hs, ts;
                for (const auto &p : chosen) { hs.push_back(p.h); ts.push_back(transverseAtStation(p)); }
                const double h = median(hs), t = median(ts);
                if (std::abs(h - previousH) > 1e-6)
                    slope = std::clamp((t - previousT) / (h - previousH), -maxSlope, maxSlope);
                previousH = h; previousT = t;
                support.insert(support.end(), chosen.begin(), chosen.end());
            }
        }
        if (ambiguous || central.empty() || support.size() < std::size_t(config.minSurfacePoints)) {
            station.failure = ambiguous ? "ambiguous_surface_branch" : "insufficient_surface_support";
            result.intersectionStations.push_back(station); continue;
        }
        const double minH = std::min_element(support.begin(), support.end(), [](const Sample &a, const Sample &b) { return a.h < b.h; })->h;
        std::map<int, int> occupied;
        double maxH = minH;
        for (const auto &p : support) { ++occupied[p.layer]; maxH = std::max(maxH, p.h); }
        station.supportMinHeight = minH;
        station.supportMaxHeight = maxH;
        station.point.supportCount = std::uint32_t(support.size());
        if (occupied.size() < 3 || support.size() < std::size_t(config.minSurfacePoints)) {
            station.failure = "insufficient_height_span";
            result.intersectionStations.push_back(station); continue;
        }
        Model model = fit(support, 1, config.surfaceFitTolerance, halfWidth);
        if (occupied.size() >= 6) {
            const auto curved = fit(support, 2, config.surfaceFitTolerance, halfWidth);
            const double linearCV = validationError(support, 1, config.surfaceFitTolerance, halfWidth);
            const double curvedCV = validationError(support, 2, config.surfaceFitTolerance, halfWidth);
            if (curved.valid && curvedCV < 0.75 * linearCV && curved.rms < model.rms)
                model = curved;
        }
        station.point.surfaceRms = model.rms;
        station.point.surfaceDegree = model.degree;
        if (!model.valid || model.rms > config.surfaceFitTolerance) station.failure = "surface_fit_residual";
        const double angle = std::atan2(1.0, std::abs(model.slope(0.0)) * depthFactor) * 180.0 / pi;
        if (station.failure.empty() && angle < config.minIntersectionAngleDegrees) station.failure = "near_tangent_intersection";
        const double extrapolation = arcLength(model, 0.0, minH, depthFactor);
        const double observedSpan = arcLength(model, minH, maxH, depthFactor);
        station.point.extrapolated = minH > 1e-8;
        station.point.extrapolationDistance = extrapolation;
        station.attemptedExtrapolation = extrapolation;
        if (station.failure.empty() && (extrapolation > config.maxSurfaceExtrapolation + 1e-8 ||
                                       extrapolation > observedSpan + 1e-8)) station.failure = "surface_extrapolation_limit";
        station.point.xyz = anchor + u * model.value(0.0);
        station.longitudinalTangent = l-v*(plane.normal.dot(l)/depthFactor) +
            u*(model.coefficients[2]/model.sScale);
        const auto representative = std::min_element(support.begin(), support.end(), [](const Sample &a, const Sample &b) {
            const double da = std::hypot(a.h, a.s), db = std::hypot(b.h, b.s);
            return da == db ? a.source < b.source : da < db;
        });
        station.point.source = representative->source;
        station.valid = station.failure.empty();
        result.intersectionStations.push_back(station);
        if (station.valid) result.projected.push_back(station.point);
    }
    // Intersection and projection are different geometric meanings. If any station
    // cannot be intersected, either reject the run or switch every station to the
    // same projection mode; never mix both modes in one trajectory.
    const auto firstFailure = std::find_if(result.intersectionStations.begin(),
        result.intersectionStations.end(), [](const IntersectionStation &station) {
            return !station.valid;
        });
    if (firstFailure != result.intersectionStations.end()) {
        const bool projectionAvailable = std::all_of(result.intersectionStations.begin(),
            result.intersectionStations.end(), [](const IntersectionStation &station) {
                return station.hasProjectionCandidate;
            });
        if (config.allowProjectionFallback && projectionAvailable) {
            result.usedProjectionFallback = true;
            result.projected.clear();
            result.projected.reserve(result.intersectionStations.size());
            for (auto &station : result.intersectionStations) {
                if (!station.valid) station.intersectionFailure = station.failure;
                station.point.xyz = station.projectionCandidate.xyz;
                station.point.source = station.projectionCandidate.source;
                station.point.referenceIndex = station.projectionCandidate.referenceIndex;
                station.point.projectionFallback = true;
                station.point.extrapolated = false;
                station.point.extrapolationDistance = 0.0;
                station.longitudinalTangent = station.projectionTangent;
                station.valid = true;
                station.failure.clear();
                result.projected.push_back(station.point);
            }
        } else {
            result.message = "表面求交失败，长度坐标 " + std::to_string(firstFailure->longitudinal) +
                "：" + failureLabel(firstFailure->failure) + "（" + firstFailure->failure +
                "），支持点=" + std::to_string(firstFailure->point.supportCount) + "，残差=" +
                std::to_string(firstFailure->point.surfaceRms) + "，外推弧长=" +
                std::to_string(firstFailure->point.extrapolationDistance) +
                (config.allowProjectionFallback
                    ? "。该站位也没有完整 B 投影证据，未生成轨迹。"
                    : "。投影回退已关闭，未生成轨迹。保留检查点供复核。");
            return false;
        }
    }
    for (std::size_t i = 1; i < result.projected.size(); ++i)
        if (result.projected[i-1].referenceIndex != result.projected[i].referenceIndex) {
            const auto ai = result.projected[i].referenceIndex;
            const auto &segment = result.referenceSegments[ai];
            const auto &left = result.intersectionStations[i-1];
            auto &right = result.intersectionStations[i];
            const double boundary = segment.longitudinalMin;
            right.boundaryIntersectionGap =
                ((left.point.xyz + left.longitudinalTangent*(boundary-left.longitudinal)) -
                 (right.point.xyz + right.longitudinalTangent*(boundary-right.longitudinal))).norm();
            if (!segment.boundaryGapMeasured || segment.boundaryGapFromPrevious > config.surfaceFitTolerance * 2.0 ||
                right.boundaryIntersectionGap > config.surfaceFitTolerance * 2.0) {
                right.valid = false;
                right.failure = "reference_boundary_intersection_gap";
                result.projected.erase(result.projected.begin()+i);
                result.message = "相邻 A 面边界不连续，特征曲线不能跨区段强行连接。";
                return false;
            }
        }
    return result.projected.size() >= 2;
}

std::vector<ProjectedPoint> resampleIntersection(const std::vector<ProjectedPoint> &points, double spacing) {
    if (points.size() < 2) return {};
    std::vector<double> cumulative(points.size(), 0.0);
    for (std::size_t i = 1; i < points.size(); ++i)
        cumulative[i] = cumulative[i-1] + (points[i].xyz - points[i-1].xyz).norm();
    if (cumulative.back() <= 1e-9) return {};
    const int count = std::max(1, int(std::lround(cumulative.back() / spacing)));
    std::vector<ProjectedPoint> result;
    std::size_t segment = 1;
    for (int i = 0; i <= count; ++i) {
        const double target = cumulative.back() * i / count;
        while (segment + 1 < points.size() && cumulative[segment] < target) ++segment;
        const auto &a = points[segment-1], &b = points[segment];
        const double length = cumulative[segment] - cumulative[segment-1];
        const double ratio = length > 1e-9 ? (target - cumulative[segment-1]) / length : 0.0;
        if (a.referenceIndex != b.referenceIndex) {
            // Preserve both boundary samples once, never walk backwards when
            // multiple arc-length samples fall within the same boundary edge.
            if (result.empty() || (result.back().xyz-b.xyz).norm() > 1e-8) {
                if (result.empty() || (result.back().xyz-a.xyz).norm() > 1e-8) result.push_back(a);
                if ((result.back().xyz-b.xyz).norm() > 1e-8) result.push_back(b);
            }
            continue;
        }
        auto p = ratio < 0.5 ? a : b;
        p.xyz = a.xyz * (1.0-ratio) + b.xyz * ratio;
        p.extrapolated = a.extrapolated || b.extrapolated;
        p.extrapolationDistance = std::max(a.extrapolationDistance,b.extrapolationDistance);
        p.surfaceRms = std::max(a.surfaceRms,b.surfaceRms);
        result.push_back(p);
    }
    return result;
}
}
