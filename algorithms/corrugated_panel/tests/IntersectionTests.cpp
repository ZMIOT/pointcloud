#include "SurfaceIntersection.h"
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value, const std::string &message) {
    if (!value) throw std::runtime_error(message);
}
struct Fixture {
    std::vector<bwb::Point> points;
    bwb::Result result;
};
Fixture fixture(double slope, double curvature, double start = 1.0, bool gap = false) {
    Fixture f;
    f.result.referenceLongitudinal = bwb::Vec3::UnitX();
    f.result.referenceTransverse = bwb::Vec3::UnitY();
    f.result.referenceOrigin = bwb::Vec3::Zero();
    bwb::ReferenceSegment a;
    a.plane.normal = bwb::Vec3::UnitZ();
    a.plane.d = 0;
    a.longitudinalMin = -50;
    a.longitudinalMax = 50;
    f.result.referenceSegments.push_back(a);
    f.result.slices.resize(1);
    f.result.slices[0].side = 1;
    f.result.selectedSlice = 0;
    // Analytic surface: y = 0.1*x + slope*z + curvature*z*z.
    // Its true intersection with z=0 is y=0.1*x, not y at the B height.
    for (int ix = -100; ix <= 100; ++ix) {
        const double x = ix * 0.5;
        if (gap && std::abs(x) < 4) continue;
        for (double h = start; h <= 35.0; h += 0.25) {
            const auto index = std::uint32_t(f.points.size());
            f.points.push_back({{x, 0.1*x + slope*h + curvature*h*h, h}, index});
            if (std::abs(h-10) <= 1.5) f.result.selectedIndices.push_back(int(index));
        }
    }
    return f;
}
void analytic(double slope, double curvature, const Eigen::Matrix3d &r,
              const bwb::Vec3 &translation, const std::string &label) {
    auto f = fixture(slope, curvature);
    for (auto &p : f.points) p.xyz = r*p.xyz + translation;
    auto &a = f.result.referenceSegments[0].plane;
    a.normal = r*bwb::Vec3::UnitZ();
    a.d = -a.normal.dot(translation);
    f.result.referenceLongitudinal = r*bwb::Vec3::UnitX();
    f.result.referenceOrigin = translation;
    require(bwb::recoverSurfaceIntersection(f.points, {}, {}, f.result), label+": "+f.result.message);
    double error = 0;
    for (const auto &p : f.result.projected) {
        const auto original = (r.transpose()*(p.xyz-translation)).eval();
        error = std::max(error, std::hypot(original.y()-0.1*original.x(), original.z()));
        require(!p.projectionFallback && p.supportCount >= 20 && p.extrapolationDistance <= 40,
                "missing body-extension provenance");
        if (curvature != 0) require(p.surfaceDegree == 2, "quadratic surface was not selected");
    }
    require(error < 1e-5, label+": intersection error="+std::to_string(error));
    std::cout << label << " max_analytic_error=" << error << '\n';
}
void rejected(Fixture f, const std::string &reason, bwb::Config config = {}) {
    config.allowProjectionFallback = false;
    require(!bwb::recoverSurfaceIntersection(f.points, config, {}, f.result), "unsafe case accepted: "+reason);
    require(f.result.message.find(reason) != std::string::npos, f.result.message);
    require(f.result.trajectory.empty(), "failed surface must not produce trajectory");
}

void weldedEdgeInvariant(const Eigen::Matrix3d &rotation, const bwb::Vec3 &translation) {
    auto clean = fixture(0.4, 0.02);
    auto welded = clean;
    // A bead distorts only the excluded edge strip. Leave the body and B identical.
    for (auto &p : welded.points)
        if (p.xyz.z() < 8.5 && std::abs(p.xyz.x()) < 12.0)
            p.xyz.y() += 12.0 * (1.0 - p.xyz.z()/8.5);
    for (auto *f : {&clean, &welded}) {
        for (auto &p : f->points) p.xyz = rotation*p.xyz + translation;
        f->result.referenceOrigin = translation;
        f->result.referenceLongitudinal = rotation*bwb::Vec3::UnitX();
        auto &a = f->result.referenceSegments[0].plane;
        a.normal = rotation*bwb::Vec3::UnitZ();
        a.d = -a.normal.dot(translation);
        require(bwb::recoverSurfaceIntersection(f->points, {}, {}, f->result), f->result.message);
        require(!f->result.usedProjectionFallback, "weld test must exercise body extension");
        for (const auto &station : f->result.intersectionStations) {
            require(station.supportMinHeight >= 8.5-1e-7 &&
                        station.supportMaxHeight <= 31.5+1e-7,
                    "edge points leaked into the fit support");
            const auto source = rotation.transpose() *
                (f->points[station.point.source].xyz-translation);
            require(source.z() >= 8.5-1e-7, "representative support came from the welded edge");
        }
    }
    require(clean.result.projected.size() == welded.result.projected.size(),
            "edge bead changed curve coverage");
    for (std::size_t i = 0; i < clean.result.projected.size(); ++i)
        require((clean.result.projected[i].xyz-welded.result.projected[i].xyz).norm() < 1e-7,
                "edge bead changed nominal body trajectory");
    std::cout << "excluded weld invariance passed\n";
}
}
int main() {
    try {
        const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
        const bwb::Vec3 zero = bwb::Vec3::Zero();
        analytic(0, 0, identity, zero, "perpendicular surface");
        analytic(0.4, 0, identity, zero, "inclined surface (old projection bias 4 mm at B=10)");
        analytic(0.2, 0.03, identity, zero, "curved surface");
        for (const auto &axis : {bwb::Vec3::UnitX().eval(), bwb::Vec3::UnitY().eval(), bwb::Vec3(1,2,-3)}) {
            const auto r = Eigen::AngleAxisd(1.5707963267948966, axis.normalized()).toRotationMatrix();
            analytic(0.4, 0.02, r, {100000,-250000,50000}, "rotated/translated curved surface");
        }
        weldedEdgeInvariant(identity, zero);
        weldedEdgeInvariant(Eigen::AngleAxisd(1.1, bwb::Vec3(1,2,-3).normalized())
                                .toRotationMatrix(), {100000,-250000,50000});
        auto strictLimit = bwb::Config{};
        strictLimit.maxSurfaceExtrapolation = 8.0;
        rejected(fixture(0,0,6), "surface_extrapolation_limit", strictLimit);
        strictLimit.maxSurfaceExtrapolation = 10.0;
        rejected(fixture(1,0,4), "surface_extrapolation_limit", strictLimit); // arc > 10 at height 8.5
        auto fallback = fixture(0.4,0,6);
        strictLimit.maxSurfaceExtrapolation = 5.0;
        require(bwb::recoverSurfaceIntersection(fallback.points, strictLimit, {}, fallback.result),
                "whole-curve projection fallback must recover unsupported intersection stations");
        require(fallback.result.usedProjectionFallback &&
                    std::all_of(fallback.result.projected.begin(), fallback.result.projected.end(),
                        [](const bwb::ProjectedPoint &p) { return p.projectionFallback; }),
                "fallback must switch every station to one projection mode");
        require(std::any_of(fallback.result.intersectionStations.begin(),
                            fallback.result.intersectionStations.end(),
                        [](const bwb::IntersectionStation &s) {
                            return s.intersectionFailure == "surface_extrapolation_limit";
                        }), "fallback lost the primary intersection failure evidence");
        for (const auto &p : fallback.result.projected) {
            require(std::abs(p.xyz.z()) < 1e-9, "fallback point is not on A");
            require(std::abs(p.xyz.y() - 0.1*p.xyz.x()) > 2.0,
                    "projection fallback was mislabeled as the analytic intersection");
        }
        auto onLimit = fixture(0,0,5);
        strictLimit.maxSurfaceExtrapolation = 8.5;
        require(bwb::recoverSurfaceIntersection(onLimit.points, strictLimit, {}, onLimit.result),
                "exactly 8.5 mm supported body extension must be accepted");
        require(!onLimit.result.usedProjectionFallback,
                "a supported intersection must not use projection fallback");
        rejected(fixture(0,0,1,true), "missing_B_seed");
        auto missingSeed = fixture(0,0,1,true);
        require(!bwb::recoverSurfaceIntersection(missingSeed.points, {}, {}, missingSeed.result),
                "projection fallback must not bridge an empty B section");
        auto tangentLimit = bwb::Config{};
        tangentLimit.minIntersectionAngleDegrees = 60.0;
        rejected(fixture(1,0), "near_tangent_intersection", tangentLimit);
        auto noExtrapolation = bwb::Config{};
        noExtrapolation.maxSurfaceExtrapolation = 0;
        rejected(fixture(0,0), "surface_extrapolation_limit", noExtrapolation);
        auto ambiguous = fixture(0,0);
        auto second = ambiguous.points;
        for (auto &p : ambiguous.points) p.xyz.y() -= 3;
        for (auto p : second) {
            p.xyz.y() += 3;
            p.source = std::uint32_t(ambiguous.points.size());
            if (std::abs(p.xyz.z()-10) <= 1.5) ambiguous.result.selectedIndices.push_back(int(p.source));
            ambiguous.points.push_back(p);
        }
        rejected(std::move(ambiguous), "ambiguous_surface_branch");
        auto boundary = fixture(0.4,0);
        boundary.result.referenceSegments[0].longitudinalMax = 0;
        auto a2 = boundary.result.referenceSegments[0];
        a2.longitudinalMin = 0;
        a2.longitudinalMax = 50;
        a2.boundaryGapMeasured = true;
        boundary.result.referenceSegments.push_back(a2);
        auto invalidBoundary = boundary;
        invalidBoundary.result.referenceSegments[1].boundaryGapFromPrevious = 2;
        require(!bwb::recoverSurfaceIntersection(invalidBoundary.points, {}, {}, invalidBoundary.result),
                "discontinuous A boundary must fail");
        require(bwb::recoverSurfaceIntersection(boundary.points, {}, {}, boundary.result),
                "continuous A boundary must succeed");
        const auto uniform = bwb::resampleIntersection(boundary.result.projected, 0.25);
        for (std::size_t i = 1; i < uniform.size(); ++i)
            require(uniform[i].xyz.x() > uniform[i-1].xyz.x(), "boundary resampling duplicates or reverses points");
        auto cancelled = fixture(0,0);
        int checks = 0;
        bwb::Hooks hooks;
        hooks.cancelled = [&] { return ++checks > 3; };
        require(!bwb::recoverSurfaceIntersection(cancelled.points, {}, hooks, cancelled.result) &&
                    cancelled.result.cancelled && cancelled.result.trajectory.empty(),
                "surface recovery must observe cancellation");
        std::cout << "Surface intersection tests passed\n";
    } catch (const std::exception &e) {
        std::cerr << "FAILED: " << e.what() << '\n';
        return 1;
    }
}
