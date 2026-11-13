#include "SurfaceFilter.h"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace bwb {
namespace {
std::uint64_t hash(std::uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

struct Patch {
    Vec3 origin = Vec3::Zero(), normal = Vec3::UnitZ(), across = Vec3::UnitY();
    double low = 0, high = 0;
};

// Require area support in both directions. A dense single scan line is not a sheet.
bool refine(const std::vector<Point> &points, const std::vector<int> &sample,
            const Vec3 &longitudinal, double distance, int minimum, Patch &patch) {
    std::vector<int> support;
    for (int i : sample)
        if (std::abs(patch.normal.dot(points[std::size_t(i)].xyz-patch.origin)) <= distance)
            support.push_back(i);
    if (support.size() < std::size_t(minimum)) return false;
    Vec3 mean = Vec3::Zero();
    for (int i : support) mean += points[std::size_t(i)].xyz;
    mean /= double(support.size());
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int i : support) {
        const Vec3 d = points[std::size_t(i)].xyz-mean;
        covariance.noalias() += d*d.transpose();
    }
    covariance /= double(support.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues()[1] < distance*distance ||
        eigen.eigenvalues()[0] > distance*distance) return false;
    patch.normal = eigen.eigenvectors().col(0);
    patch.origin = mean;
    patch.across = patch.normal.cross(longitudinal);
    if (patch.across.norm() < 0.15) return false;
    patch.across.normalize();
    patch.low = std::numeric_limits<double>::infinity();
    patch.high = -patch.low;
    std::vector<double> lengths, widths;
    for (int i : support) {
        const Vec3 delta=points[std::size_t(i)].xyz-mean;
        const double t = patch.across.dot(delta);
        widths.push_back(t); lengths.push_back(longitudinal.dot(delta));
        patch.low = std::min(patch.low,t);
        patch.high = std::max(patch.high,t);
    }
    // A dense stray line plus a handful of remote points can fool covariance.
    // Require the central 80% of support to span both in-plane dimensions.
    const auto centralSpan=[](std::vector<double> values) {
        std::sort(values.begin(),values.end());
        return values[std::size_t(0.9*(values.size()-1))]-values[std::size_t(0.1*(values.size()-1))];
    };
    if (centralSpan(widths)<=2*distance || centralSpan(lengths)<=2*distance) return false;
    std::sort(widths.begin(),widths.end());
    const std::size_t first=std::size_t(0.1*(widths.size()-1));
    const std::size_t last=std::size_t(0.9*(widths.size()-1));
    for (std::size_t i=first+1;i<=last;++i)
        if (widths[i]-widths[i-1]>std::max(4*distance,0.25*(widths[last]-widths[first])))
            return false; // A stray line and remote specks do not form a connected sheet.
    return true;
}

bool fitSheet(const std::vector<Point> &points, const std::vector<int> &sample,
              const Vec3 &longitudinal, const Config &config, Patch &best) {
    if (sample.size() < std::size_t(config.prefilterMinSupport)) return false;
    const double tolerance = config.prefilterDistance;
    double bestScore = 0;
    int bestCore = 0;
    for (int iteration = 0; iteration < 160; ++iteration) {
        const auto pick = [&](int salt) -> const Vec3 & {
            return points[std::size_t(sample[hash(std::uint64_t(iteration)*3+salt)%sample.size()])].xyz;
        };
        const Vec3 a = pick(0), b = pick(1), c = pick(2);
        Vec3 n = (b-a).cross(c-a);
        if (n.norm() < 1e-9) continue;
        n.normalize();
        if (std::abs(n.dot(longitudinal)) > 0.99) continue;
        int core = 0, shoulder = 0;
        for (int i : sample) {
            const double d = std::abs(n.dot(points[std::size_t(i)].xyz-a));
            if (d <= tolerance) ++core;
            else if (d <= 3*tolerance) ++shoulder;
        }
        // Equal-width density comparison rejects arbitrary planes through volume noise.
        const double score = core-0.5*shoulder;
        if (core >= config.prefilterMinSupport && score >= 0.60*core && score > bestScore) {
            Patch candidate;
            candidate.origin = a;
            candidate.normal = n;
            if (!refine(points,sample,longitudinal,tolerance,config.prefilterMinSupport,candidate))
                continue;
            best = candidate;
            bestScore = score;
            bestCore = core;
        }
    }
    if (bestCore == 0) return false;
    return refine(points,sample,longitudinal,tolerance,config.prefilterMinSupport,best);
}
}

bool filterPanelSurfaces(const std::vector<Point> &points, const Config &config,
                         const Hooks &hooks, Result &result) {
    result.prefilterApplied = config.prefilterEnabled;
    result.surfaceMask.assign(points.size(),0);
    result.retainedIndices.clear();
    result.rejectedIndices.clear();
    result.prefilterPatches = 0;
    result.prefilterUnsupportedWindows = 0;
    const auto cancelled = [&] {
        if (hooks.cancelled && hooks.cancelled()) { result.cancelled = true; return true; }
        return false;
    };
    if (cancelled()) return false;
    if (!config.prefilterEnabled) {
        for (std::size_t i=0; i<points.size(); ++i) {
            if ((i & 8191u)==0 && cancelled()) return false;
            result.surfaceMask[i] = points[i].xyz.allFinite() ? 1 : 0;
        }
    } else {
        if (result.referenceSegments.empty()) {
            result.message = "前置表面滤波缺少局部基准面。";
            return false;
        }
        const auto &axis = result.referenceLongitudinal;
        const auto &origin = result.referenceOrigin;
        const double width = config.prefilterWindow;
        // Four overlapping fits cover each interior point, including scan-line and
        // fold boundaries where one centered window can have degenerate support.
        const double step = width*0.25;
        const double boundaryEpsilon=1e-8*std::max(1.0,width);
        const double low = result.referenceSegments.front().longitudinalMin-width;
        const double high = result.referenceSegments.back().longitudinalMax+width;
        const double count = std::ceil((high-low)/step)+1;
        if (!std::isfinite(count) || count > 100000 || count < 1) {
            result.message = "前置表面滤波窗口数量异常，请检查坐标单位与窗口长度。";
            return false;
        }
        std::vector<std::pair<double,int>> ordered;
        ordered.reserve(points.size());
        for (std::size_t i=0; i<points.size(); ++i) {
            if ((i & 8191u)==0 && cancelled()) return false;
            if (!points[i].xyz.allFinite()) continue;
            const double s = axis.dot(points[i].xyz-origin);
            if (s >= low-boundaryEpsilon && s <= high+boundaryEpsilon) ordered.emplace_back(s,int(i));
        }
        std::sort(ordered.begin(),ordered.end());
        // Existing independently fitted A support is retained; no infinite-plane exemption.
        for (int i : result.referenceIndices) result.surfaceMask[std::size_t(i)] = 1;
        for (int window=0; window<int(count); ++window) {
            if (cancelled()) return false;
            const double center = low+step*window;
            const auto begin = std::lower_bound(ordered.begin(),ordered.end(),
                std::make_pair(center-width*0.5-boundaryEpsilon,-1));
            const auto end = std::upper_bound(ordered.begin(),ordered.end(),
                std::make_pair(center+width*0.5+boundaryEpsilon,std::numeric_limits<int>::max()));
            if (begin==end) continue;
            auto ai = std::lower_bound(result.referenceSegments.begin(),result.referenceSegments.end(),center,
                [](const ReferenceSegment &a,double s) { return a.longitudinalMax < s; });
            if (ai==result.referenceSegments.end()) ai=std::prev(result.referenceSegments.end());
            Patch base;
            base.normal = ai->plane.normal;
            base.origin = ai->plane.origin;
            base.across = base.normal.cross(axis).normalized();
            base.low = std::numeric_limits<double>::infinity(); base.high = -base.low;
            std::vector<int> remaining;
            for (auto it=begin;it!=end;++it) {
                const auto &p=points[std::size_t(it->second)].xyz;
                if (std::abs(base.normal.dot(p-base.origin)) <= config.planeDistance) {
                    const double t=base.across.dot(p-base.origin);
                    base.low=std::min(base.low,t);base.high=std::max(base.high,t);
                } else remaining.push_back(it->second);
            }
            std::vector<Patch> patches;
            if (base.low<=base.high) patches.push_back(base);
            // Two principal non-reference sheets: platform and corrugation. Overlapping
            // windows let either side of a genuine corner support its own local plane.
            for (int surface=0;surface<2;++surface) {
                if (cancelled()) return false;
                // Input index order makes sampling independent of rounding in equal
                // longitudinal coordinates after a rigid transform.
                std::sort(remaining.begin(),remaining.end());
                std::vector<int> sample;
                const std::size_t sampleSize=std::min<std::size_t>(1024,remaining.size());
                for (std::size_t j=0;j<sampleSize;++j)
                    sample.push_back(remaining[j*remaining.size()/sampleSize]);
                Patch patch;
                if (!fitSheet(points,sample,axis,config,patch)) break;
                patches.push_back(patch);
                // Refine finite footprint using full inlier support, not the capped sample.
                auto &stored=patches.back();
                stored.low=std::numeric_limits<double>::infinity();stored.high=-stored.low;
                std::vector<int> next;
                next.reserve(remaining.size());
                for (int i:remaining) {
                    const Vec3 d=points[std::size_t(i)].xyz-stored.origin;
                    if (std::abs(stored.normal.dot(d)) <= config.prefilterDistance) {
                        const double t=stored.across.dot(d);
                        stored.low=std::min(stored.low,t);stored.high=std::max(stored.high,t);
                    } else next.push_back(i);
                }
                remaining.swap(next);
            }
            result.prefilterPatches+=patches.size();
            if (patches.size()<=1) ++result.prefilterUnsupportedWindows;
            for (auto it=begin;it!=end;++it) {
                const std::size_t i=std::size_t(it->second);
                for (const auto &patch:patches) {
                    const Vec3 d=points[i].xyz-patch.origin;
                    const double t=patch.across.dot(d);
                    if (std::abs(patch.normal.dot(d))<=config.prefilterDistance &&
                        t>=patch.low-config.prefilterDistance && t<=patch.high+config.prefilterDistance) {
                        result.surfaceMask[i]=1;break;
                    }
                }
            }
            if (hooks.progress && window%16==0) hooks.progress(36+8*window/int(count));
        }
    }
    for (std::size_t i=0;i<points.size();++i) {
        if ((i & 8191u)==0 && cancelled()) return false;
        (result.surfaceMask[i] ? result.retainedIndices : result.rejectedIndices).push_back(int(i));
    }
    if (result.retainedIndices.size()<std::size_t(config.minPlanePoints)) {
        result.message="前置滤波后有效表面点数不足，停止轨迹提取。";
        return false;
    }
    return true;
}
}
