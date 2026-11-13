#include "LapEdge.h"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>

namespace lap {
namespace {
// 核心计算统一使用局部坐标：(横向 x，扫描截面位置 y，向上 z)。
// 对外输出时仍使用输入点的原始 XYZ；local 只用于几何判断。
struct Cancelled {};
struct ScanSpreadFailure : std::runtime_error {
    ScanSpreadFailure()
        : std::runtime_error("Excessive within-section spread; scan recovery is unreliable.") {}
};
double median(std::vector<double> a) {
    if (a.empty())
        return 0;
    const auto m = a.begin() + a.size() / 2;
    std::nth_element(a.begin(), m, a.end());
    const double v = *m;
    return a.size() % 2 ? v : 0.5 * (v + *std::max_element(a.begin(), m));
}
double quantile(std::vector<double> a, double q) {
    if (a.empty())
        return 0;
    const auto k = std::size_t(q * (a.size() - 1));
    std::nth_element(a.begin(), a.begin() + k, a.end());
    return a[k];
}
double mad(const std::vector<double> &a) {
    const double m = median(a);
    std::vector<double> v;
    v.reserve(a.size());
    for (double x : a)
        v.push_back(std::abs(x - m));
    return 1.4826 * median(v);
}
double huber(double a) {
    a = std::abs(a);
    return a < 1 ? 0.5 * a * a : a - 0.5;
}
double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
void check(const Hooks &h) {
    if (h.cancelled && h.cancelled())
        throw Cancelled{};
}
void stage(int i, const Result &r, const Hooks &h) {
    check(h);
    if (h.progress)
        h.progress(10 + i * 16);
    if (h.stage && !h.stage(i, r))
        throw Cancelled{};
}
struct Line {
    double slope = 0, intercept = 0, noise = 0;
    int count = 0;
};
Line lineFit(const std::vector<Vec> &q, const std::vector<int> &ids, double fallback, double d) {
    Line out;
    out.count = int(ids.size());
    out.slope = fallback;
    if (ids.empty())
        return out;
    std::vector<double> slopes;
    for (std::size_t i = 0; i < ids.size(); ++i)
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const Vec diff = q[ids[j]] - q[ids[i]];
            if (std::abs(diff.x()) > 0.4 * d)
                slopes.push_back(diff.z() / diff.x());
        }
    if (ids.size() >= 3 && !slopes.empty())
        out.slope = median(slopes);
    std::vector<double> offsets;
    for (int id : ids)
        offsets.push_back(q[id].z() - out.slope * q[id].x());
    out.intercept = median(offsets);
    out.noise = mad(offsets) / std::sqrt(1 + out.slope * out.slope);
    return out;
}
double at(const Line &a, double x) { return a.slope * x + a.intercept; }

// 相邻点拟合在密集扫描中可能被量化噪声带偏。只有首次截面恢复失败时，
// 才利用扫描线末尾到下一线开头的显著空间回跳重新估计平面法向。
Vec normalFromScanResets(const std::vector<Point> &input, const Vec &forward, const Hooks &h) {
    std::vector<double> distances;
    distances.reserve(input.size() - 1);
    for (std::size_t i = 1; i < input.size(); ++i)
        distances.push_back((input[i].xyz - input[i - 1].xyz).norm());

    const double typical = median(distances);
    const double resetGate = std::max(15 * typical, 0.5 * quantile(distances, 0.9995));
    std::vector<std::size_t> boundaries{0};
    for (std::size_t i = 0; i < distances.size(); ++i)
        if (distances[i] > resetGate)
            boundaries.push_back(i + 1);
    boundaries.push_back(input.size());
    if (boundaries.size() < 12)
        return Vec::Zero();

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    std::size_t supported = 0;
    for (std::size_t section = 1; section < boundaries.size(); ++section) {
        check(h);
        const auto begin = boundaries[section - 1], end = boundaries[section];
        if (end - begin < 30)
            continue;
        Vec center = Vec::Zero();
        for (auto i = begin; i < end; ++i)
            center += input[i].xyz;
        center /= double(end - begin);
        for (auto i = begin; i < end; ++i) {
            const Vec delta = input[i].xyz - center;
            covariance.noalias() += delta * delta.transpose();
        }
        supported += end - begin;
    }
    if (supported < input.size() * 0.7)
        return Vec::Zero();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.eigenvalues()[1] <= eigen.eigenvalues()[2] * 1e-10)
        return Vec::Zero();
    Vec normal = eigen.eigenvectors().col(0);
    if (normal.dot(forward) < 0)
        normal = -normal;
    return normal;
}

void recover(Result &r, const Config &cfg, const Hooks &h) {
    // 1.1 用整体主方向和用户指定的“上方”建立临时坐标基。
    const std::size_t count = r.input.size();
    if (count < 30)
        throw std::runtime_error("Too few points to recover scan sections (need at least 30).");
    Vec mean = Vec::Zero();
    for (const auto &p : r.input) {
        if (!p.xyz.allFinite())
            throw std::runtime_error("Input contains non-finite XYZ.");
        mean += p.xyz;
    }
    mean /= double(count);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto &p : r.input) {
        const Vec d = p.xyz - mean;
        cov.noalias() += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(cov);
    Vec longitudinal = eigen.eigenvectors().col(2);
    if (longitudinal.dot(r.input.back().xyz - r.input.front().xyz) < 0)
        longitudinal = -longitudinal;
    Vec up = cfg.up.normalized();
    Vec vertical = up - longitudinal * up.dot(longitudinal);
    if (vertical.norm() < 0.1)
        throw std::runtime_error(
            "Up direction is parallel to scan travel; specify a valid up direction.");
    vertical.normalize();
    const Vec horizontal = longitudinal.cross(vertical).normalized();
    std::vector<Vec> deltas;
    std::vector<double> distances;
    for (std::size_t i = 1; i < count; ++i) {
        Vec d = r.input[i].xyz - r.input[i - 1].xyz;
        deltas.push_back(d);
        distances.push_back(d.norm());
    }
    const double typical = median(distances);
    if (!(typical > 0))
        throw std::runtime_error("Degenerate or duplicated input.");
    Vec normal = cfg.scanNormal;
    if (normal.norm() == 0) {
        // 1.2 从原始点序中挑选截面内相邻点，用鲁棒拟合估计扫描平面法向。
        // 大跳步属于截面切换，不应参与平面拟合。
        const double limit = quantile(distances, 0.65) * 1.2;
        std::vector<Vec> fit;
        for (const auto &d : deltas)
            if (d.norm() > 0.15 * typical && d.norm() < limit &&
                std::abs(d.dot(longitudinal)) < 0.4 * d.norm())
                fit.emplace_back(d.dot(horizontal), d.dot(vertical), d.dot(longitudinal));
        if (fit.size() < 20)
            throw std::runtime_error("Cannot estimate scan plane from ordered neighboring points.");
        Eigen::Vector2d coefficients = Eigen::Vector2d::Zero();
        for (int iteration = 0; iteration < 8; ++iteration) {
            check(h);
            std::vector<double> residual;
            for (const auto &d : fit)
                residual.push_back(d.z() - coefficients.dot(d.head<2>()));
            const double sigma = std::max(mad(residual), typical * 1e-7);
            Eigen::Matrix2d a = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            for (std::size_t i = 0; i < fit.size(); ++i) {
                const double w =
                    iteration ? std::min(1., 1.5 * sigma / std::max(std::abs(residual[i]), 1e-20))
                              : 1.;
                const Eigen::Vector2d v = fit[i].head<2>();
                a.noalias() += w * v * v.transpose();
                b += w * v * fit[i].z();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> e(a);
            if (e.eigenvalues()[0] < e.eigenvalues()[1] * 1e-8) {
                // Perfectly straight synthetic profiles cannot determine plane tilt: PCA travel is
                // the identifiable default.
                coefficients.setZero();
                break;
            }
            coefficients = a.ldlt().solve(b);
        }
        normal = (longitudinal - coefficients.x() * horizontal - coefficients.y() * vertical)
                     .normalized();
    } else
        normal.normalize();
    if (normal.dot(longitudinal) < 0)
        normal = -normal;
    r.scanNormal = normal;
    vertical = up - normal * up.dot(normal);
    if (vertical.norm() < 0.1)
        throw std::runtime_error("Up direction is parallel to scan normal.");
    r.vertical = vertical.normalized();
    r.transverse = normal.cross(r.vertical).normalized();
    r.local.reserve(count);
    for (const auto &p : r.input)
        r.local.emplace_back(p.xyz.dot(r.transverse), p.xyz.dot(normal), p.xyz.dot(r.vertical));
    // 1.3 沿点序检测扫描位置的跳变，恢复每个截面的点索引。
    std::vector<double> ds;
    for (std::size_t i = 1; i < count; ++i)
        ds.push_back(std::abs(r.local[i].y() - r.local[i - 1].y()));
    const double noise = std::max(quantile(ds, 0.75), typical * 1e-7);
    std::vector<double> separatedSteps;
    for (double step : ds)
        if (step > 20 * noise)
            separatedSteps.push_back(step);
    if (separatedSteps.size() < 5 || separatedSteps.size() > count / 3)
        throw std::runtime_error("Cannot distinguish within-scan noise from between-scan steps.");
    const double jump = median(separatedSteps);
    if (jump < noise * 15)
        throw std::runtime_error(
            "Scan sections are not separable; preserve scan order or supply the scan normal.");
    const double threshold = std::sqrt(noise * jump);
    r.sections.emplace_back();
    for (std::size_t i = 0; i < count; ++i) {
        if (i && std::abs(r.local[i].y() - r.local[i - 1].y()) > threshold)
            r.sections.emplace_back();
        r.sections.back().points.push_back(int(i));
    }
    if (r.sections.size() < 6 || r.sections.size() > count / 3)
        throw std::runtime_error(
            "Too few reliable scan sections, or input point order is scrambled.");
    std::vector<double> stationSteps, allSpacing;
    std::size_t supported = 0;
    for (auto &f : r.sections) {
        // 1.4 校验截面，并同时保留“完整原始点”和“分析代表点”。
        // 密集扫描只压缩用于候选/曲面计算的点；最终实测边缘会回查原始点。
        std::vector<double> stations;
        for (int id : f.points)
            stations.push_back(r.local[id].y());
        f.s = median(stations);
        std::size_t outliers = 0;
        for (double s : stations)
            if (std::abs(s - f.s) > threshold * 0.5 &&
                ++outliers > std::max<std::size_t>(1, stations.size() / 200))
                throw ScanSpreadFailure{};
        if (f.points.size() >= 4)
            supported += f.points.size();
        std::sort(f.points.begin(), f.points.end(),
                  [&](int a, int b) { return r.local[a].x() < r.local[b].x(); });
        f.rawPoints = f.points;
        // Keep the geometric analysis scale comparable across dense and sparse
        // scans. The representatives are original points, never invented XYZ.
        // rawPoints remains available for final edge localization.
        if (f.points.size() > 96) {
            const double minX = r.local[f.points.front()].x(), maxX = r.local[f.points.back()].x();
            const double bin = std::max((maxX - minX) / 64., typical * 4);
            std::vector<int> representatives;
            for (std::size_t begin = 0; begin < f.points.size();) {
                std::size_t end = begin + 1;
                const auto binId =
                    std::int64_t(std::floor((r.local[f.points[begin]].x() - minX) / bin));
                while (end < f.points.size() &&
                       std::int64_t(std::floor((r.local[f.points[end]].x() - minX) / bin)) == binId)
                    ++end;
                std::vector<double> heights;
                heights.reserve(end - begin);
                for (std::size_t k = begin; k < end; ++k)
                    heights.push_back(r.local[f.points[k]].z());
                const double z = median(heights), x = 0.5 * (r.local[f.points[begin]].x() +
                                                             r.local[f.points[end - 1]].x());
                int chosen = f.points[begin];
                double best = std::numeric_limits<double>::infinity();
                for (std::size_t k = begin; k < end; ++k) {
                    const int id = f.points[k];
                    const double distance =
                        std::abs(r.local[id].z() - z) + 0.5 * std::abs(r.local[id].x() - x);
                    if (distance < best) {
                        best = distance;
                        chosen = id;
                    }
                }
                representatives.push_back(chosen);
                begin = end;
            }
            f.points = std::move(representatives);
        }
        std::vector<double> spacing;
        for (std::size_t i = 1; i < f.points.size(); ++i) {
            const double d = (r.local[f.points[i]] - r.local[f.points[i - 1]]).norm();
            if (d > typical * 1e-6)
                spacing.push_back(d);
        }
        f.spacing = quantile(spacing, 0.35);
        if (f.spacing > 0)
            allSpacing.push_back(f.spacing);
    }
    if (supported < count * 0.8)
        throw std::runtime_error("Most points have insufficient section support.");
    for (std::size_t i = 1; i < r.sections.size(); ++i) {
        const double d = r.sections[i].s - r.sections[i - 1].s;
        if (d <= threshold)
            throw std::runtime_error(
                "Scan stations are not monotonic; input must retain ordered scans.");
        stationSteps.push_back(d);
    }
    r.sectionSpacing = median(stationSteps);
    r.pointSpacing = median(allSpacing);
    // 各截面使用自己的点距，但限制在整体中位数附近，避免局部缺口放大阈值。
    if (!(r.pointSpacing > 0) || r.sectionSpacing < threshold * 3)
        throw std::runtime_error("Insufficient separation between scan planes.");
    for (auto &f : r.sections)
        f.spacing = clamp(f.spacing > 0 ? f.spacing : r.pointSpacing, 0.5 * r.pointSpacing,
                          2 * r.pointSpacing);
}

bool neighborPointSupport(const Result &r, int section, int point, double radius) {
    int supported = 0;
    const Vec p = r.local[point];
    for (int j = std::max(0, section - 2); j <= std::min(int(r.sections.size()) - 1, section + 2);
         ++j) {
        if (j == section)
            continue;
        if (std::abs(r.sections[j].s - p.y()) > 3 * r.sectionSpacing)
            continue;
        for (int id : r.sections[j].points) {
            const Vec delta = r.local[id] - p;
            if (std::hypot(delta.x(), delta.z()) < radius) {
                ++supported;
                break;
            }
        }
    }
    return supported >= 2;
}

void profileCandidates(Result &r, const Config &cfg, const Hooks &h) {
    // 2.1 双侧点距都大、且前后截面也无对应点，才认定为孤立点。
    std::set<int> isolated;
    for (int fi = 0; fi < int(r.sections.size()); ++fi) {
        check(h);
        auto &f = r.sections[fi];
        const auto &ids = f.points;
        const double d = f.spacing;
        for (int i = 1; i + 1 < int(ids.size()); ++i)
            if ((r.local[ids[i]] - r.local[ids[i - 1]]).norm() > 2.5 * d &&
                (r.local[ids[i]] - r.local[ids[i + 1]]).norm() > 2.5 * d &&
                !neighborPointSupport(r, fi, ids[i], 1.5 * d))
                isolated.insert(ids[i]);
    }
    r.isolated.assign(isolated.begin(), isolated.end());
    stage(1, r, h);
    // 2.2 对每个截面枚举断点，分别拟合断点两侧的短轮廓。
    // 两侧都先保留，直到有证据确认哪一侧属于上层板。
    for (int fi = 0; fi < int(r.sections.size()); ++fi) {
        check(h);
        auto &f = r.sections[fi];
        std::vector<int> ids;
        for (int id : f.points)
            if (!isolated.count(id))
                ids.push_back(id);
        if (ids.size() < 5)
            continue;
        const double d = f.spacing;
        std::vector<double> slopes;
        for (std::size_t j = 1; j < ids.size(); ++j) {
            const Vec delta = r.local[ids[j]] - r.local[ids[j - 1]];
            if (delta.norm() < 2 * d && std::abs(delta.x()) > 0.3 * d)
                slopes.push_back(delta.z() / delta.x());
        }
        const double slope = median(slopes);
        std::vector<double> errors;
        for (std::size_t j = 1; j < ids.size(); ++j) {
            const Vec delta = r.local[ids[j]] - r.local[ids[j - 1]];
            if (delta.norm() < 2 * d)
                errors.push_back((delta.z() - slope * delta.x()) / std::sqrt(1 + slope * slope));
        }
        f.noise = std::max(mad(errors) / std::sqrt(2.), 0.025 * d);
        std::vector<Candidate> proposals;
        for (int split = 2; split <= int(ids.size()) - 2; ++split) {
            std::vector<int> left, right;
            for (int j = std::max(0, split - 6); j < split; ++j)
                left.push_back(ids[j]);
            for (int j = split; j < std::min(int(ids.size()), split + 6); ++j)
                right.push_back(ids[j]);
            const Line l = lineFit(r.local, left, slope, d), rr = lineFit(r.local, right, slope, d);
            const Vec a = r.local[ids[split - 1]], b = r.local[ids[split]];
            const double mid = 0.5 * (a.x() + b.x());
            const double signedStep = (at(l, mid) - at(rr, mid)) / std::sqrt(1 + slope * slope);
            const double noise = std::max({f.noise, l.noise, rr.noise});
            const double minStep = cfg.evidenceScale * std::max(3 * noise, 0.25 * d);
            if (std::abs(signedStep) < minStep)
                continue;
            for (int side : {-1, 1}) {
                Candidate c;
                c.section = fi;
                c.point = side < 0 ? ids[split - 1] : ids[split];
                c.other = side < 0 ? ids[split] : ids[split - 1];
                c.side = side;
                c.step = -side * signedStep;
                c.gap = (a - b).norm();
                c.upperSlope = side < 0 ? l.slope : rr.slope;
                c.lowerSlope = side < 0 ? rr.slope : l.slope;
                c.upperCount = side < 0 ? l.count : rr.count;
                c.lowerCount = side < 0 ? rr.count : l.count;
                const Line &upper = side < 0 ? l : rr;
                const Vec p = r.local[c.point];
                if (c.step < 0) {
                    c.reason = Reason::LowerSide;
                    r.candidates.push_back(c);
                    continue;
                }
                // Anchor to the last point consistent with the interior board,
                // rather than a spur immediately before the layer gap.
                const double supportGate = std::max(3 * noise, 0.4 * d);
                const int first = side < 0 ? std::max(0, split - 6) : split;
                const int last = side < 0 ? split : std::min(int(ids.size()), split + 6);
                int anchor = -1;
                for (int j = side < 0 ? last - 1 : first; side < 0 ? j >= first : j < last;
                     j += side) {
                    const Vec q = r.local[ids[j]];
                    if (std::abs(q.z() - at(upper, q.x())) /
                            std::sqrt(1 + upper.slope * upper.slope) <=
                        supportGate) {
                        anchor = ids[j];
                        break;
                    }
                }
                if (anchor < 0 || (r.local[anchor] - p).norm() > 3 * d) {
                    c.reason = Reason::Isolated;
                    r.candidates.push_back(c);
                    continue;
                }
                c.point = anchor;
                c.gap = (r.local[c.point] - r.local[c.other]).norm();
                const Vec endpoint = r.local[c.point];
                const double endpointError = std::abs(endpoint.z() - at(upper, endpoint.x())) /
                                             std::sqrt(1 + upper.slope * upper.slope);
                const double endpointGate = std::max(3 * noise, 0.4 * d);
                if (endpointError > endpointGate &&
                    (endpointError > 2 * d || !neighborPointSupport(r, fi, c.point, 1.5 * d))) {
                    c.reason = Reason::Isolated;
                    r.candidates.push_back(c);
                    continue;
                }
                c.profileScore =
                    (1 - std::exp(-c.step / (minStep + noise))) * std::exp(-noise / (0.4 * d));
                c.profileScore *=
                    0.7 + 0.3 * std::min(1., double(std::min(c.upperCount, c.lowerCount)) / 4.);
                if (endpointError > endpointGate)
                    c.profileScore *= std::exp(-(endpointError - endpointGate) / d);
                c.score = c.profileScore;
                proposals.push_back(c);
            }
        }
        std::sort(proposals.begin(), proposals.end(),
                  [](const Candidate &a, const Candidate &b) { return a.score > b.score; });
        // 2.3 每截面最多保留五个不同端点，避免噪声候选拖慢后续曲面验证。
        std::set<int> kept;
        for (auto &c : proposals) {
            if (kept.count(c.point))
                continue;
            if (kept.size() >= 5) {
                c.reason = Reason::WeakStep;
                r.candidates.push_back(c);
                continue;
            }
            kept.insert(c.point);
            r.candidates.push_back(c);
        }
    }
}

Surface fitSurface(const std::vector<Vec> &points, double x0, double s0, double dx, double ds,
                   int sectionCount, double bandwidth) {
    // 邻截面板面拟合：先试平面；数据充分且二次曲面显著降低残差时再升级。
    // 迭代权重抑制离群点，坐标归一化避免 x 与扫描位置的尺度差影响求解。
    Surface best;
    best.x0 = x0;
    best.s0 = s0;
    best.dx = dx;
    best.ds = ds;
    best.supportSections = sectionCount;
    if (points.size() < 6 || sectionCount < 2)
        return best;
    for (int degree : {1, 2}) {
        if (degree == 2 && (points.size() < 18 || sectionCount < 4))
            break;
        const int cols = degree == 1 ? 3 : 6, rows = int(points.size());
        Eigen::MatrixXd a(rows, cols);
        Eigen::VectorXd z(rows), w(rows), spatial(rows), coef;
        for (int i = 0; i < rows; ++i) {
            const double x = (points[i].x() - x0) / dx, s = (points[i].y() - s0) / ds;
            a(i, 0) = 1;
            a(i, 1) = x;
            a(i, 2) = s;
            if (degree == 2) {
                a(i, 3) = x * x;
                a(i, 4) = x * s;
                a(i, 5) = s * s;
            }
            z[i] = points[i].z();
            spatial[i] = std::exp(-0.5 * (x * x + s * s) / (bandwidth * bandwidth));
            w[i] = spatial[i];
        }
        bool valid = true;
        for (int iteration = 0; iteration < 5; ++iteration) {
            Eigen::MatrixXd weighted = a;
            Eigen::VectorXd rhs = z;
            for (int i = 0; i < rows; ++i) {
                const double sw = std::sqrt(w[i]);
                weighted.row(i) *= sw;
                rhs[i] *= sw;
            }
            Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(weighted);
            if (qr.rank() < cols) {
                valid = false;
                break;
            }
            coef = qr.solve(rhs);
            std::vector<double> residual;
            for (int i = 0; i < rows; ++i)
                residual.push_back(z[i] - a.row(i).dot(coef));
            const double sigma = std::max(mad(residual), dx * 0.01);
            for (int i = 0; i < rows; ++i)
                w[i] =
                    spatial[i] * std::min(1., 1.5 * sigma / std::max(std::abs(residual[i]), 1e-20));
        }
        if (!valid)
            continue;
        double error = 0, weight = 0;
        for (int i = 0; i < rows; ++i) {
            const double e = z[i] - a.row(i).dot(coef);
            error += w[i] * e * e;
            weight += w[i];
        }
        const double rms = std::sqrt(error / std::max(1e-12, weight)) *
                           std::sqrt(double(rows) / std::max(1, rows - cols));
        if (degree == 2 && best.valid && rms > best.rms * 0.7)
            break;
        best.valid = true;
        best.degree = degree;
        best.rms = rms;
        best.c.fill(0);
        for (int k = 0; k < cols; ++k)
            best.c[k] = coef[k];
    }
    return best;
}

void validateSurfaces(Result &r, const Config &cfg, const Hooks &h) {
    // 3.1 先按截面索引候选，以便从前后截面找到同方向的断点。
    std::vector<std::vector<int>> bySection(r.sections.size());
    for (int i = 0; i < int(r.candidates.size()); ++i)
        if (r.candidates[i].reason == Reason::Pending)
            bySection[r.candidates[i].section].push_back(i);
    for (auto &c : r.candidates) {
        if (c.reason != Reason::Pending)
            continue;
        check(h);
        const auto &section = r.sections[c.section];
        const double d = section.spacing;
        const Vec p = r.local[c.point], other = r.local[c.other];
        c.reason = Reason::NoSurface;
        for (int radius : {2, 4, 7}) {
            // 3.2 从小到大尝试三个窗口；上下板分别收点，当前截面不参与，
            // 避免候选点用自己的邻域“证明自己”。
            std::vector<Vec> upperPoints, lowerPoints;
            int upperFrames = 0, lowerFrames = 0;
            for (int j = std::max(0, c.section - radius);
                 j <= std::min(int(r.sections.size()) - 1, c.section + radius); ++j) {
                if (j == c.section ||
                    std::abs(r.sections[j].s - section.s) > (radius + 0.5) * r.sectionSpacing)
                    continue;
                // A neighboring profile supplies its own split; never let a radius neighborhood mix
                // the two sheets.
                const Candidate *match = nullptr;
                double best = std::numeric_limits<double>::infinity();
                for (int ci : bySection[j]) {
                    const auto &neighbor = r.candidates[ci];
                    if (neighbor.side != c.side)
                        continue;
                    const double offset = std::abs(r.local[neighbor.point].x() - p.x());
                    if (offset < best && offset < 6 * d) {
                        best = offset;
                        match = &neighbor;
                    }
                }
                if (!match)
                    continue;
                const Vec edge = r.local[match->point], bottom = r.local[match->other];
                const double gate = std::max(0.35 * d, 3 * r.sections[j].noise);
                int nu = 0, nl = 0;
                for (int id : r.sections[j].points) {
                    if (id == match->point || id == match->other)
                        continue;
                    const Vec q = r.local[id];
                    const double du = (q.x() - edge.x()) * c.side,
                                 dl = (q.x() - bottom.x()) * (-c.side);
                    if (du > 0.2 * d && du < 8 * d &&
                        std::abs(q.z() - edge.z() - match->upperSlope * (q.x() - edge.x())) <
                            gate) {
                        upperPoints.push_back(q);
                        ++nu;
                    }
                    if (dl > 0.2 * d && dl < 8 * d &&
                        std::abs(q.z() - bottom.z() - match->lowerSlope * (q.x() - bottom.x())) <
                            gate) {
                        lowerPoints.push_back(q);
                        ++nl;
                    }
                }
                if (nu >= 2)
                    ++upperFrames;
                if (nl >= 2)
                    ++lowerFrames;
            }
            c.upperSurfaceSections = std::max(c.upperSurfaceSections, upperFrames);
            c.lowerSurfaceSections = std::max(c.lowerSurfaceSections, lowerFrames);
            Surface upper = fitSurface(upperPoints, p.x(), p.y(), 4 * d, radius * r.sectionSpacing,
                                       upperFrames, cfg.neighborhoodScale);
            Surface lower =
                fitSurface(lowerPoints, other.x(), p.y(), 4 * d, radius * r.sectionSpacing,
                           lowerFrames, cfg.neighborhoodScale);
            if (!upper.valid || !lower.valid)
                continue;
            // 3.3 分别检查噪声、候选到上层板面的残差、上下板分离度。
            // 曲面证据不足的候选不能交给轨迹平滑阶段补救。
            const double slope = upper.c[1] / upper.dx;
            const double normalizer = std::sqrt(1 + slope * slope);
            const double noise =
                std::max(section.noise, std::max(upper.rms, lower.rms) / normalizer);
            c.surfaceNoise = noise;
            if (noise > 0.4 * d) {
                c.reason = Reason::SurfaceMismatch;
                continue;
            }
            const double residual = std::abs(p.z() - upper.height(p.x(), p.y())) / normalizer;
            c.surfaceResidual = residual;
            if (residual > std::max(3 * noise, 0.35 * d)) {
                c.reason = Reason::SurfaceMismatch;
                continue;
            }
            const double midpoint = (p.x() + other.x()) * 0.5;
            const double separation =
                (upper.height(midpoint, p.y()) - lower.height(midpoint, p.y())) / normalizer;
            if (separation < cfg.evidenceScale * std::max(3 * noise, 0.25 * d)) {
                c.reason = Reason::NoLayerSeparation;
                continue;
            }
            c.upper = upper;
            c.lower = lower;
            c.radius = radius;
            c.eligible = true;
            c.reason = Reason::Accepted;
            const double support = std::min(1., double(std::min(upperFrames, lowerFrames)) / 4.);
            c.score =
                clamp(0.6 * c.profileScore + 0.25 * std::exp(-residual / std::max(0.2 * d, noise)) +
                          0.15 * support,
                      0., 1.);
            break;
        }
    }
}

// Surface membership alone also accepts the last remaining interior point after
// edge occlusion. Independently check the neighboring, surface-validated endpoint
// envelope before tracking, so tracking cannot turn an interior dropout into a seam.
void validateEndpoints(Result &r, const Hooks &h) {
    // 3.4 已通过曲面验证的候选还需符合邻截面端点趋势。
    // 这一步在全局选点前执行，防止轨迹把板内缺测点误连成边缘。
    std::vector<std::array<int, 2>> envelope(r.sections.size(), std::array<int, 2>{-1, -1});
    for (int i = 0; i < int(r.candidates.size()); ++i) {
        const auto &c = r.candidates[i];
        if (!c.eligible)
            continue;
        int &selected = envelope[c.section][c.side > 0 ? 1 : 0];
        if (selected < 0 ||
            c.side * r.local[c.point].x() < c.side * r.local[r.candidates[selected].point].x())
            selected = i;
    }
    for (auto &c : r.candidates) {
        if (!c.eligible)
            continue;
        check(h);
        const Vec p = r.local[c.point];
        const double d = r.sections[c.section].spacing;
        std::vector<Eigen::Vector2d> edges;
        for (int j = std::max(0, c.section - 7);
             j <= std::min(int(r.sections.size()) - 1, c.section + 7); ++j) {
            const int k = envelope[j][c.side > 0 ? 1 : 0];
            if (j == c.section || k < 0 ||
                std::abs(r.sections[j].s - p.y()) > 7.5 * r.sectionSpacing)
                continue;
            const Vec q = r.local[r.candidates[k].point];
            edges.emplace_back(q.y() - p.y(), q.x());
        }
        if (edges.size() < 4)
            continue;
        std::vector<double> slopes;
        for (std::size_t a = 0; a < edges.size(); ++a)
            for (std::size_t b = a + 1; b < edges.size(); ++b)
                if (std::abs(edges[a].x() - edges[b].x()) > r.sectionSpacing * 0.5)
                    slopes.push_back((edges[a].y() - edges[b].y()) / (edges[a].x() - edges[b].x()));
        const double slope = median(slopes);
        std::vector<double> centers;
        for (const auto &q : edges)
            centers.push_back(q.y() - slope * q.x());
        const double prediction = median(centers), sigma = std::max(mad(centers), 0.5 * d);
        for (double center : centers)
            if (std::abs(center - prediction) <= std::max(d, 2 * sigma))
                ++c.endpointSupport;
        const double residual = std::abs(p.x() - prediction);
        if (residual > std::max(2 * d, 3 * sigma)) {
            c.eligible = false;
            c.reason = Reason::EndpointMismatch;
            continue;
        }
        c.score *= 0.75 + 0.25 * std::exp(-0.5 * residual * residual / (sigma * sigma));
    }
}

void track(Result &r, const Config &cfg, const Hooks &h) {
    // 4.1 动态规划状态保存前两个已选候选，用位移和趋势共同约束轨迹。
    // 每个截面还允许“缺失”，曲面支持不足时不强行补候选。
    struct Node {
        int candidate = -1, previous = -1;
        bool start = false;
    };
    struct State {
        int previous = -1, last = -1, node = -1;
        double cost = 0;
        bool start = false;
    };
    std::vector<std::vector<int>> bySection(r.sections.size());
    for (int i = 0; i < int(r.candidates.size()); ++i)
        if (r.candidates[i].eligible)
            bySection[r.candidates[i].section].push_back(i);
    std::vector<Node> nodes;
    std::vector<State> states(1);
    for (int fi = 0; fi < int(r.sections.size()); ++fi) {
        check(h);
        std::map<std::pair<int, int>, State> next;
        auto put = [&](const State &s) {
            auto key = std::make_pair(s.previous, s.last);
            auto i = next.find(key);
            if (i == next.end() || s.cost < i->second.cost)
                next[key] = s;
        };
        for (auto state : states) {
            if (state.last >= 0) {
                const auto &last = r.candidates[state.last];
                if (fi - last.section > cfg.maxMissing + 1 ||
                    r.sections[fi].s - r.sections[last.section].s >
                        (cfg.maxMissing + 1.5) * r.sectionSpacing)
                    state.previous = state.last = -1;
            }
            State missing = state;
            missing.cost += 0.35;
            missing.start = false;
            put(missing);
            for (int ci : bySection[fi]) {
                const auto &c = r.candidates[ci];
                const Vec q = r.local[c.point];
                double transition = 0;
                if (state.last >= 0) {
                    const auto &last = r.candidates[state.last];
                    const Vec p = r.local[last.point];
                    if (last.side != c.side)
                        continue;
                    const double dt = q.y() - p.y();
                    const double scale = r.pointSpacing * (1 + 0.3 * dt / r.sectionSpacing);
                    transition = 0.15 * huber(std::hypot(q.x() - p.x(), q.z() - p.z()) / scale);
                    if (state.previous >= 0) {
                        const Vec before = r.local[r.candidates[state.previous].point];
                        const Vec prediction = p + (p - before) * (dt / (p.y() - before.y()));
                        transition +=
                            0.7 * huber(std::hypot(q.x() - prediction.x(), q.z() - prediction.z()) /
                                        (r.pointSpacing * (1 + 0.15 * dt / r.sectionSpacing)));
                    }
                }
                State selected;
                selected.previous = state.last;
                selected.last = ci;
                selected.node = state.node;
                selected.cost = state.cost + transition - 1.8 * c.score;
                selected.start = state.last < 0;
                put(selected);
            }
        }
        states.clear();
        for (auto &entry : next) {
            State state = entry.second;
            if (state.last >= 0 && r.candidates[state.last].section == fi) {
                nodes.push_back({state.last, state.node, state.start});
                state.node = int(nodes.size()) - 1;
            }
            states.push_back(state);
        }
    }
    const auto best =
        std::min_element(states.begin(), states.end(),
                         [](const State &a, const State &b) { return a.cost < b.cost; });
    std::vector<Node> path;
    for (int node = best->node; node >= 0; node = nodes[node].previous)
        path.push_back(nodes[node]);
    std::reverse(path.begin(), path.end());
    // 4.2 回溯最优路径；不足三个实测点的独立小段不作为最终轨迹。
    std::uint32_t segment = 0;
    for (std::size_t begin = 0; begin < path.size();) {
        std::size_t end = begin + 1;
        while (end < path.size() && !path[end].start)
            ++end;
        if (end - begin < 3) {
            for (std::size_t i = begin; i < end; ++i)
                r.candidates[path[i].candidate].reason = Reason::ShortSupport;
        } else {
            if (!r.measured.empty())
                ++segment;
            for (std::size_t i = begin; i < end; ++i) {
                const auto &node = path[i];
                const auto &c = r.candidates[node.candidate];
                r.selected.push_back(node.candidate);
                r.measured.push_back({r.input[c.point].xyz, r.input[c.point].source,
                                      std::uint32_t(c.section), segment, float(c.score), false});
            }
        }
        begin = end;
    }
    std::set<int> selected(r.selected.begin(), r.selected.end());
    for (int i = 0; i < int(r.candidates.size()); ++i)
        if (r.candidates[i].eligible && !selected.count(i) &&
            r.candidates[i].reason != Reason::ShortSupport)
            r.candidates[i].reason = Reason::NotSelected;
}

// A short run before the layer gap can be the actual visible edge. Keep its
// gap-facing tip only when the same upper-sheet fragment persists in nearby
// sections; a one-section cluster still falls back to the long interior run.
bool neighboringRawFragment(const Result &r, const Candidate &c, int tip, double d,
                            double tolerance, const Hooks &h) {
    // 同一短段要在邻近截面重复出现，且点同时接近上层曲面、远离下层曲面。
    // 仅凭本截面的几个点无法区分真实短段与小团杂点。
    const Vec endpoint = r.local[tip];
    int supported = 0;
    const double slope = c.upper.c[1] / c.upper.dx, norm = std::sqrt(1 + slope * slope);
    for (int offset : {-3, -2, -1, 1, 2, 3}) {
        const int fi = c.section + offset;
        if (fi < 0 || fi >= int(r.sections.size()))
            continue;
        check(h);
        if (std::abs(r.sections[fi].s - endpoint.y()) > 3.5 * r.sectionSpacing)
            continue;
        int close = 0;
        for (int id : r.sections[fi].rawPoints) {
            const Vec q = r.local[id];
            if (std::abs(q.x() - endpoint.x()) > 0.9 * d)
                continue;
            if (std::abs(q.z() - c.upper.height(q.x(), q.y())) / norm > tolerance)
                continue;
            const double separation =
                (c.upper.height(q.x(), q.y()) - c.lower.height(q.x(), q.y())) / norm;
            if (separation < 0.25 * d ||
                (q.z() - c.lower.height(q.x(), q.y())) / norm < 0.5 * separation)
                continue;
            if (++close >= 3)
                break;
        }
        if (close >= 3 && ++supported >= 2)
            return true;
    }
    return false;
}

// Refine the selected edge to an original point while retaining the gap-facing
// endpoint of a short, repeatable upper-sheet fragment.
void refineConnectedRawEdges(Result &r, const Hooks &h) {
    std::vector<int> retained;
    std::vector<Sample> measured;
    retained.reserve(r.selected.size());
    measured.reserve(r.measured.size());
    for (std::size_t k = 0; k < r.selected.size(); ++k) {
        check(h);
        auto &c = r.candidates[r.selected[k]];
        c.rawCoarsePoint = c.point;
        const auto &section = r.sections[c.section];
        const auto &ids = section.rawPoints;
        if (ids.size() == section.points.size()) {
            c.rawDecision = RawDecision::SparseInput;
            retained.push_back(r.selected[k]);
            measured.push_back(r.measured[k]);
            continue;
        }
        const Vec center = r.local[c.point];
        const double d = section.spacing;
        const double slope = c.upper.c[1] / c.upper.dx, norm = std::sqrt(1 + slope * slope);
        const double tolerance = std::max(5 * c.upper.rms, 0.9 * d);
        // 5.1 在粗候选附近回查原始点；局部点距决定连通阈值。
        std::vector<int> nearby;
        std::vector<double> steps;
        for (int id : ids) {
            const Vec q = r.local[id];
            const double inward = (q.x() - center.x()) * c.side;
            if (inward >= -2 * d && inward <= 8 * d) {
                if (!nearby.empty())
                    steps.push_back((q - r.local[nearby.back()]).norm());
                nearby.push_back(id);
            }
        }
        if (nearby.size() < 4 || steps.empty()) {
            retained.push_back(r.selected[k]);
            measured.push_back(r.measured[k]);
            continue;
        }
        const double normalSpacing = quantile(steps, 0.35);
        const double connect = std::min(1.3 * d, std::max(5 * normalSpacing, 0.3 * d));
        // 5.2 只让属于上层板的原始点进入连通分量，避免跨缝连接下层板。
        std::vector<int> members;
        for (int id : nearby) {
            const Vec q = r.local[id];
            if (std::abs(q.z() - c.upper.height(q.x(), q.y())) / norm > tolerance)
                continue;
            const double separation =
                (c.upper.height(q.x(), q.y()) - c.lower.height(q.x(), q.y())) / norm;
            if (separation < 0.25 * d ||
                (q.z() - c.lower.height(q.x(), q.y())) / norm < 0.5 * separation)
                continue;
            members.push_back(id);
        }
        std::vector<int> parent(members.size());
        std::iota(parent.begin(), parent.end(), 0);
        // 横向已排序，仅检查距离可能小于 connect 的点对。
        auto root = [&](int id) {
            while (parent[id] != id) {
                parent[id] = parent[parent[id]];
                id = parent[id];
            }
            return id;
        };
        for (int i = 0; i < int(members.size()); ++i) {
            const Vec a = r.local[members[i]];
            for (int j = i + 1; j < int(members.size()); ++j) {
                const Vec b = r.local[members[j]];
                if (b.x() - a.x() > connect)
                    break;
                if (std::hypot(b.x() - a.x(), b.z() - a.z()) <= connect)
                    parent[root(j)] = root(i);
            }
        }
        struct Component {
            int count = 0, tip = -1;
            double minX = std::numeric_limits<double>::infinity(),
                   maxX = -std::numeric_limits<double>::infinity();
        };
        std::map<int, Component> components;
        int seed = -1;
        double seedDistance = connect * 1.5;
        for (int i = 0; i < int(members.size()); ++i) {
            const Vec q = r.local[members[i]];
            auto &component = components[root(i)];
            ++component.count;
            component.minX = std::min(component.minX, q.x());
            component.maxX = std::max(component.maxX, q.x());
            if (component.tip < 0 || c.side * (q.x() - r.local[component.tip].x()) < 0)
                component.tip = members[i];
            const double distance = std::hypot(q.x() - center.x(), q.z() - center.z());
            if (distance < seedDistance) {
                seed = i;
                seedDistance = distance;
            }
        }
        int best = -1;
        bool detached = false;
        if (seed >= 0) {
            // 5.3 优先取当前上层分量朝缝隙的端点。短分量需邻截面重复支持；
            // 否则回退到连续长板面，或在证据不足时拒绝该截面。
            const auto &current = components[root(seed)];
            const double span = current.maxX - current.minX;
            c.rawRunCount = current.count;
            c.rawRunSpan = span;
            if (span >= 1.5 * d && current.count >= 6) {
                if (std::abs(r.local[current.tip].x() - center.x()) <= 2 * d) {
                    best = current.tip;
                    c.rawDecision = RawDecision::ConnectedTip;
                }
            } else {
                const bool repeatable = current.count >= 4 && span >= 0.3 * d &&
                                        std::abs(r.local[current.tip].x() - center.x()) <= d &&
                                        neighboringRawFragment(r, c, current.tip, d, tolerance, h);
                if (repeatable) {
                    best = current.tip;
                    c.rawDecision = RawDecision::RepeatableShortTip;
                }
                double nearest = std::numeric_limits<double>::infinity();
                for (const auto &entry : components) {
                    if (best >= 0)
                        break;
                    const auto &run = entry.second;
                    const double runSpan = run.maxX - run.minX;
                    const double inward = (r.local[run.tip].x() - center.x()) * c.side;
                    if (run.count >= 6 && runSpan >= 1.5 * d && inward > connect &&
                        inward < 4 * d && inward < nearest) {
                        best = run.tip;
                        nearest = inward;
                        c.rawDecision = RawDecision::DetachedToMain;
                    }
                }
                if (best < 0 && (span < 0.6 * d || current.count < 4)) {
                    detached = true;
                    c.rawDecision = RawDecision::RejectedFragment;
                } else if (best < 0 && std::abs(r.local[current.tip].x() - center.x()) <= d) {
                    best = current.tip;
                    c.rawDecision = RawDecision::ShortLocalTip;
                }
            }
        }
        if (detached) {
            c.eligible = false;
            c.reason = Reason::DetachedFragment;
            continue;
        }
        if (best < 0 && seed < 0) {
            c.rawDecision = RawDecision::SurfaceFallback;
            // Keep the previous surface-based refinement when a long run cannot
            // be established, rather than moving an endpoint deep inside a sheet.
            const double oldTolerance = std::max(3 * c.upper.rms, 0.15 * d);
            auto supported = [&](int id) {
                const Vec q = r.local[id];
                if (std::abs(q.z() - c.upper.height(q.x(), q.y())) / norm > oldTolerance)
                    return false;
                const double separation =
                    (c.upper.height(q.x(), q.y()) - c.lower.height(q.x(), q.y())) / norm;
                if (separation < 0.25 * d ||
                    (q.z() - c.lower.height(q.x(), q.y())) / norm < 0.5 * separation)
                    return false;
                const auto at =
                    std::lower_bound(ids.begin(), ids.end(), q.x(),
                                     [&](int a, double x) { return r.local[a].x() < x; });
                const int pos = int(at - ids.begin());
                int interior = 0;
                for (int j = std::max(0, pos - 12); j < std::min(int(ids.size()), pos + 13); ++j) {
                    if (ids[j] == id)
                        continue;
                    const Vec other = r.local[ids[j]];
                    const double inward = (other.x() - q.x()) * c.side;
                    if (inward > 0 && inward <= 0.3 * d &&
                        std::abs(other.z() - c.upper.height(other.x(), other.y())) / norm <
                            oldTolerance)
                        ++interior;
                }
                return interior >= 2;
            };
            double reach = -std::numeric_limits<double>::infinity();
            best = c.point;
            for (int id : ids) {
                const double towardGap = (r.local[id].x() - center.x()) * (-c.side);
                if (towardGap < -.25 * d || towardGap > 2 * d || towardGap <= reach)
                    continue;
                if (supported(id)) {
                    best = id;
                    reach = towardGap;
                }
            }
        }
        if (best < 0)
            best = c.point;
        // 5.4 实测输出仍绑定原始点的 XYZ 和 source_index。
        c.point = best;
        auto sample = r.measured[k];
        sample.xyz = r.input[best].xyz;
        sample.source = r.input[best].source;
        retained.push_back(r.selected[k]);
        measured.push_back(sample);
    }
    r.selected = std::move(retained);
    r.measured = std::move(measured);
}

bool supportedPosition(const Result &r, const Candidate &c, const Vec &q, double maxLateral) {
    const Vec original = r.local[c.point];
    const double d = r.sections[c.section].spacing;
    const double slope = c.upper.c[1] / c.upper.dx, norm = std::sqrt(1 + slope * slope);
    const double noise = std::max(r.sections[c.section].noise, c.upper.rms / norm);
    if (std::abs(q.x() - original.x()) > maxLateral * d)
        return false;
    if (std::abs(q.z() - c.upper.height(q.x(), q.y())) / norm > std::max(3 * noise, 0.25 * d))
        return false;
    const double separation = (c.upper.height(q.x(), q.y()) - c.lower.height(q.x(), q.y())) / norm;
    return separation > std::max(2 * noise, 0.2 * d) &&
           (q.z() - c.lower.height(q.x(), q.y())) / norm > 0.45 * separation;
}
Vec global(const Result &r, const Vec &q) {
    return r.transverse * q.x() + r.scanNormal * q.y() + r.vertical * q.z();
}
void smooth(Result &r, const Config &cfg, const Hooks &h) {
    // 6.1 实测点保持原始 XYZ；拟合轨迹单独沿扫描方向去除单帧尖跳。
    // 横向与高度一起按前后截面拟合；上层曲面只作几何约束，避免逐截面
    // 曲面模型自身的微小变化被直接写进轨迹而产生新的高度尖跳。
    std::vector<Sample> fitted = r.measured;
    // 先找出连续实测截面。即使短缺口后续可能插补，也不能在决定是否
    // 连通之前用缺口两侧的点共同拟合，否则断段端点会被另一侧拉偏。
    std::vector<int> smoothingRun(fitted.size(), 0);
    for (int i = 1; i < int(fitted.size()); ++i) {
        smoothingRun[i] = smoothingRun[i - 1];
        if (r.measured[i].segment != r.measured[i - 1].segment ||
            r.measured[i].section != r.measured[i - 1].section + 1)
            ++smoothingRun[i];
    }
    std::vector<int> runBeginIndex(fitted.size()), runEndIndex(fitted.size());
    for (int i = 0; i < int(fitted.size()); ++i)
        runBeginIndex[i] = i && smoothingRun[i] == smoothingRun[i - 1]
                               ? runBeginIndex[i - 1]
                               : i;
    for (int i = int(fitted.size()) - 1; i >= 0; --i)
        runEndIndex[i] = i + 1 < int(fitted.size()) &&
                                 smoothingRun[i] == smoothingRun[i + 1]
                             ? runEndIndex[i + 1]
                             : i;
    for (int i = 0; i < int(fitted.size()); ++i) {
        check(h);
        if (cfg.smoothing == 0)
            break;
        const auto &c = r.candidates[r.selected[i]];
        // 确认属于独立短段的端点不跨断口移动。
        if (c.rawDecision == RawDecision::DetachedToMain ||
            c.rawDecision == RawDecision::RepeatableShortTip)
            continue;
        const Vec original = r.local[c.point];
        // 6.1.1 取连续段内对称的较长窗口。旧的 ±7 截面二次拟合会跟随
        // 若干连续噪声形成短促弯折；对称窗口也避免在断段端点外推。
        const int radius = std::min({18, i - runBeginIndex[i], runEndIndex[i] - i});
        if (radius < 3)
            continue;
        std::vector<int> neighbors;
        neighbors.reserve(2 * radius);
        for (int j = i - radius; j <= i + radius; ++j)
            if (j != i)
                neighbors.push_back(j);
        Eigen::MatrixXd a(neighbors.size(), 3), positions(neighbors.size(), 2);
        Eigen::VectorXd baseWeights(neighbors.size());
        for (int j = 0; j < int(neighbors.size()); ++j) {
            const int k = neighbors[j];
            const Vec q = r.local[r.candidates[r.selected[k]].point];
            const double t = (q.y() - original.y()) / r.sectionSpacing;
            a.row(j) << 1, t, t * t;
            positions.row(j) << q.x(), q.z();
            const double normalizedDistance =
                t / (0.7 * radius * cfg.neighborhoodScale);
            baseWeights[j] = r.measured[k].confidence /
                             (1 + normalizedDistance * normalizedDistance);
        }
        // 6.1.2 同时拟合直线与二次趋势。只有二次项确实解释了持续弯曲，
        // 才让轨迹转弯；单个或少量离群点用鲁棒权重降权。
        auto fitTrend = [&](int degree, Eigen::Vector2d &center, double &error) {
            const Eigen::MatrixXd design = a.leftCols(degree + 1);
            Eigen::VectorXd weights = baseWeights;
            Eigen::MatrixXd coefficients;
            bool success = false;
            for (int iteration = 0; iteration < 4; ++iteration) {
                Eigen::MatrixXd wa = design, wp = positions;
                for (int j = 0; j < design.rows(); ++j) {
                    const double w = std::sqrt(weights[j]);
                    wa.row(j) *= w;
                    wp.row(j) *= w;
                }
                Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(wa);
                if (qr.rank() < degree + 1)
                    return false;
                coefficients = qr.solve(wp);
                success = coefficients.allFinite();
                if (!success)
                    return false;
                std::vector<double> residuals;
                residuals.reserve(design.rows());
                for (int j = 0; j < design.rows(); ++j)
                    residuals.push_back((positions.row(j) - design.row(j) * coefficients).norm());
                error = median(residuals);
                const double noise = std::max(mad(residuals), 0.03 * r.pointSpacing);
                for (int j = 0; j < design.rows(); ++j)
                    weights[j] = baseWeights[j] *
                                 std::min(1., 1.5 * noise / std::max(residuals[j], 1e-20));
            }
            center = coefficients.row(0).transpose();
            return success;
        };
        Eigen::Vector2d lineCenter, curveCenter;
        double lineError = 0, curveError = 0;
        if (!fitTrend(1, lineCenter, lineError) ||
            !fitTrend(2, curveCenter, curveError))
            continue;
        const Eigen::Vector2d target =
            lineError - curveError > 0.035 * r.pointSpacing ? curveCenter : lineCenter;
        const double deltaX = target.x() - original.x();
        const double deltaZ = target.y() - original.z();
        if (std::hypot(deltaX, deltaZ) < 0.02 * r.pointSpacing)
            continue;
        // 不让当前跳点参与拟合。若它显著偏离两侧趋势，就给模型更高权重；
        // 其余位置保留适量实测约束，避免抹平连续转折。
        const double alpha =
            std::hypot(deltaX, deltaZ) > 0.2 * r.pointSpacing
                ? 1.
                : cfg.smoothing / (cfg.smoothing + 0.2 + 0.25 * r.measured[i].confidence);
        for (double fraction : {alpha, alpha * 0.5, alpha * 0.25, alpha * 0.125}) {
            const double lateral =
                clamp(deltaX * fraction, -0.6 * r.pointSpacing, 0.6 * r.pointSpacing);
            const double xFit = original.x() + lateral;
            const double slope = c.upper.c[1] / c.upper.dx;
            const double normalLength = std::sqrt(1 + slope * slope);
            const double noise = std::max(r.sections[c.section].noise, c.upper.rms / normalLength);
            const double tolerance =
                normalLength * std::max(3 * noise, 0.25 * r.sections[c.section].spacing);
            const double surfaceZ = c.upper.height(xFit, original.y());
            const double zFit = clamp(original.z() + deltaZ * fraction, surfaceZ - 0.6 * tolerance,
                                      surfaceZ + 0.6 * tolerance);
            const Vec q(xFit, original.y(), zFit);
            if ((q - original).norm() <= 0.75 * r.pointSpacing && supportedPosition(r, c, q, 0.6)) {
                fitted[i].xyz = global(r, q);
                break;
            }
        }
    }
    std::uint32_t segment = 0;
    // 6.2 只在短缺口两侧都有独立端点和曲面支持时插补；
    // 验证失败或超过最大缺失数时断段，不在首尾外推。
    for (int i = 0; i < int(fitted.size()); ++i) {
        check(h);
        if (i) {
            const auto &prev = fitted[i - 1];
            const auto &current = fitted[i];
            const int missing = int(current.section) - int(prev.section) - 1;
            bool continuous = prev.segment == current.segment && missing <= cfg.maxMissing;
            std::vector<Sample> bridge;
            if (continuous && missing > 0) {
                const auto &ca = r.candidates[r.selected[i - 1]];
                const auto &cb = r.candidates[r.selected[i]];
                const auto toLocal = [&](const Vec &p) {
                    return Vec(p.dot(r.transverse), p.dot(r.scanNormal), p.dot(r.vertical));
                };
                const Vec a = toLocal(prev.xyz), b = toLocal(current.xyz);
                // Reliability is based on independent endpoint support, not an
                // arbitrary cutoff on the uncalibrated display quality score.
                if (ca.endpointSupport < 2 || cb.endpointSupport < 2)
                    continuous = false;
                for (int k = 1; continuous && k <= missing; ++k) {
                    const int fi = int(prev.section) + k;
                    const double t = (r.sections[fi].s - a.y()) / (b.y() - a.y());
                    const Vec q = a + t * (b - a);
                    if (!supportedPosition(r, ca, q, 3) || !supportedPosition(r, cb, q, 3)) {
                        continuous = false;
                        break;
                    }
                    bridge.push_back({global(r, q), std::numeric_limits<std::uint32_t>::max(),
                                      std::uint32_t(fi), segment,
                                      0.5f * std::min(prev.confidence, current.confidence), true});
                }
            }
            if (!continuous)
                ++segment;
            else
                r.trajectory.insert(r.trajectory.end(), bridge.begin(), bridge.end());
        }
        auto sample = fitted[i];
        sample.segment = segment;
        r.trajectory.push_back(sample);
    }
}
} // namespace

double Surface::height(double x, double s) const {
    x = (x - x0) / dx;
    s = (s - s0) / ds;
    return c[0] + c[1] * x + c[2] * s + c[3] * x * x + c[4] * x * s + c[5] * s * s;
}
const char *reasonName(Reason r) {
    switch (r) {
    case Reason::Pending:
        return "pending";
    case Reason::Accepted:
        return "accepted";
    case Reason::LowerSide:
        return "lower_side";
    case Reason::WeakStep:
        return "weak_or_ranked_out";
    case Reason::ShortSupport:
        return "short_support";
    case Reason::Isolated:
        return "isolated_endpoint";
    case Reason::NoSurface:
        return "insufficient_neighbor_surfaces";
    case Reason::SurfaceMismatch:
        return "surface_mismatch";
    case Reason::NoLayerSeparation:
        return "no_layer_separation";
    case Reason::NotSelected:
        return "not_selected_by_track";
    case Reason::EndpointMismatch:
        return "inconsistent_neighbor_endpoint";
    case Reason::DetachedFragment:
        return "detached_short_fragment";
    }
    return "unknown";
}
std::string validateConfig(const Config &c) {
    if (!c.up.allFinite() || c.up.norm() < 1e-8)
        return "Up direction must be finite and nonzero.";
    if (!c.scanNormal.allFinite())
        return "Scan normal must be finite.";
    if (!std::isfinite(c.neighborhoodScale) || c.neighborhoodScale < 0.5 || c.neighborhoodScale > 3)
        return "Neighborhood scale must be in [0.5, 3].";
    if (!std::isfinite(c.smoothing) || c.smoothing < 0 || c.smoothing > 4)
        return "Smoothing must be in [0, 4].";
    if (c.maxMissing < 0 || c.maxMissing > 3)
        return "Maximum missing sections must be in [0, 3].";
    if (!std::isfinite(c.evidenceScale) || c.evidenceScale < 0.5 || c.evidenceScale > 2)
        return "Evidence scale must be in [0.5, 2].";
    return {};
}
Result detect(const std::vector<Point> &points, const Config &cfg, const Hooks &h) {
    Result r;
    try {
        check(h);
        const auto error = validateConfig(cfg);
        if (!error.empty())
            throw std::runtime_error(error);
        r.input = points;

        // 步骤 1：恢复有序扫描截面，估计点距与扫描间距。
        try {
            recover(r, cfg, h);
        } catch (const ScanSpreadFailure &) {
            if (cfg.scanNormal.norm() != 0)
                throw; // 手动指定法向时不覆盖用户选择。
            const Vec retryNormal = normalFromScanResets(r.input, r.scanNormal, h);
            if (retryNormal.isZero())
                throw;
            // recover 失败时 r 只含部分截面；重试前必须清空派生状态。
            r.local.clear();
            r.sections.clear();
            Config retry = cfg;
            retry.scanNormal = retryNormal;
            recover(r, retry, h);
        }
        stage(0, r, h);

        // 步骤 2：单截面生成候选（疑似孤立点作为单独调试阶段输出）。
        profileCandidates(r, cfg, h);
        stage(2, r, h);

        // 步骤 3：用前后截面的上下板曲面和端点趋势验证候选。
        validateSurfaces(r, cfg, h);
        validateEndpoints(r, h);
        stage(3, r, h);

        // 步骤 4：选整段路径；步骤 5：回到原始密集点定位真实端点。
        track(r, cfg, h);
        refineConnectedRawEdges(r, h);
        stage(4, r, h);

        // 步骤 6：受约束平滑，可靠短缺口插补，其余位置断段。
        smooth(r, cfg, h);
        stage(5, r, h);
        if (r.measured.size() < 3)
            throw std::runtime_error(
                "No reliable upper lap edge: fewer than three surface-validated points.");
        r.ok = true;
        r.message = "Upper lap edge extracted; quality scores are not calibrated probabilities.";
        if (h.progress)
            h.progress(100);
    } catch (const Cancelled &) {
        r.cancelled = true;
        r.message = "Cancelled";
    } catch (const std::exception &e) {
        r.message = e.what();
    }
    return r;
}
} // namespace lap
