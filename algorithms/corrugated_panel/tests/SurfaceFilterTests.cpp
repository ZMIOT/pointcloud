#include "SurfaceFilter.h"
#include "SurfaceIntersection.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace {
void require(bool value,const char *message) { if (!value) throw std::runtime_error(message); }
struct Fixture {
    std::vector<bwb::Point> points;
    bwb::Result reference;
    std::size_t baseEnd=0, waveEnd=0, cleanEnd=0;
};
Fixture fixture() {
    Fixture f;
    auto add=[&](double x,double y,double z) {
        // Source IDs deliberately differ from vector positions.
        f.points.push_back({bwb::Vec3(x,y,z),std::uint32_t(500+2*f.points.size())});
    };
    for (int x=-100;x<=100;++x) for (int y=-60;y<=60;y+=2) add(x,y,0);
    f.baseEnd=f.points.size();
    for (int x=-100;x<=100;++x) for (int z=1;z<=35;++z) {
        const double y=x<-40 ? 0 : x<0 ? (x+40)*0.5 : x<40 ? 20-x*0.5 : 0;
        add(x,y,z);
    }
    f.waveEnd=f.points.size();
    for (int x=-100;x<=100;++x) for (int z=-35;z<0;++z) add(x,-48,z);
    f.cleanEnd=f.points.size();
    std::mt19937 rng(118);
    std::uniform_real_distribution<double> x(-100,100),y(65,115),z(-40,40);
    for (int i=0;i<8000;++i) add(x(rng),y(rng),z(rng));
    // Dense collinear interference, incapable of providing area support.
    for (int i=0;i<3000;++i) add(-30+i*0.02,140,20);
    add(800,20,0); // coplanar but beyond the finite longitudinal panel domain
    add(std::numeric_limits<double>::quiet_NaN(),0,0);
    auto &r=f.reference;
    r.reference.normal=bwb::Vec3::UnitZ();
    r.referenceLongitudinal=bwb::Vec3::UnitX();
    r.referenceTransverse=bwb::Vec3::UnitY();
    bwb::ReferenceSegment a;
    a.longitudinalMin=-100;a.longitudinalMax=100;a.locallyFitted=true;
    for (std::size_t i=0;i<f.baseEnd;++i) a.indices.push_back(int(i));
    r.referenceSegments.push_back(a);r.referenceIndices=a.indices;
    return f;
}
void preservationAndRemoval() {
    const auto f=fixture();
    auto result=f.reference;
    bwb::Config config;
    require(bwb::filterPanelSurfaces(f.points,config,{},result),"surface filter failed");
    require(result.retainedIndices.size()+result.rejectedIndices.size()==f.points.size(),"partition incomplete");
    auto fraction=[&](std::size_t lo,std::size_t hi) {
        return double(std::count(result.surfaceMask.begin()+lo,result.surfaceMask.begin()+hi,1))/double(hi-lo);
    };
    std::cout << "retention base=" << fraction(0,f.baseEnd) << " wave=" << fraction(f.baseEnd,f.waveEnd)
              << " platform=" << fraction(f.waveEnd,f.cleanEnd)
              << " rejected_noise=" << 1-fraction(f.cleanEnd,f.points.size()) << '\n';
    require(fraction(0,f.baseEnd)==1,"reference surface eroded");
    require(fraction(f.baseEnd,f.waveEnd)>0.995,"wave/folds eroded");
    require(fraction(f.waveEnd,f.cleanEnd)>0.995,"platform eroded");
    require(fraction(f.cleanEnd,f.points.size())<0.08,"off-surface interference survived");
    require(!result.surfaceMask[f.points.size()-2] && !result.surfaceMask.back(),"invalid/far coplanar point survived");
    std::vector<unsigned char> seen(f.points.size(),0);
    for (int i:result.retainedIndices) { require(result.surfaceMask[std::size_t(i)] && !seen[std::size_t(i)],"invalid kept index");seen[std::size_t(i)]=1; }
    for (int i:result.rejectedIndices) { require(!result.surfaceMask[std::size_t(i)] && !seen[std::size_t(i)],"invalid removed index");seen[std::size_t(i)]=1; }

    auto moved=f;
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(0.93,bwb::Vec3(0.4,-0.7,1).normalized()).toRotationMatrix();
    const bwb::Vec3 translation(2300,-1700,900);
    for (auto &p:moved.points) p.xyz=rotation*p.xyz+translation;
    auto &r=moved.reference;
    r.reference.normal=rotation*bwb::Vec3::UnitZ();
    r.referenceOrigin=translation;r.referenceLongitudinal=rotation*bwb::Vec3::UnitX();
    r.referenceTransverse=rotation*bwb::Vec3::UnitY();
    for (auto &a:r.referenceSegments) {
        a.plane.origin=translation;a.plane.normal=rotation*bwb::Vec3::UnitZ();
        a.plane.d=-a.plane.normal.dot(translation);
    }
    require(bwb::filterPanelSurfaces(moved.points,config,{},r),"rotated filtering failed");
    require(r.surfaceMask==result.surfaceMask,"filter changed after rigid transform");
    for (auto &a:r.referenceSegments) { a.plane.normal=-a.plane.normal;a.plane.d=-a.plane.d; }
    require(bwb::filterPanelSurfaces(moved.points,config,{},r) && r.surfaceMask==result.surfaceMask,"normal sign changed filtering");
    config.prefilterEnabled=false;
    require(bwb::filterPanelSurfaces(f.points,config,{},r) && r.rejectedIndices.size()==1 && !r.prefilterApplied,
            "disabled mode must pass through finite input");
    int calls=0;
    bwb::Hooks hooks;hooks.cancelled=[&] { return ++calls>3; };
    config.prefilterEnabled=true;r=f.reference;
    require(!bwb::filterPanelSurfaces(f.points,config,hooks,r) && r.cancelled,"filter cancellation ignored");
    config.prefilterWindow=0;
    require(!bwb::validateConfig(config).empty(),"invalid window accepted");
}

void pipelineOrderAndExclusion() {
    auto f=fixture();
    // Use the real public pipeline and verify every downstream input still names an original point.
    bwb::Config config;config.planeIterations=250;config.threads=2;
    std::vector<int> order;
    bwb::Hooks hooks;
    hooks.stage=[&](int stage,const bwb::Result &r) {
        order.push_back(stage);
        if (stage==5) require(r.slices.empty() && r.projected.empty() && !r.retainedIndices.empty(),"filter executed after trajectory work");
        if (stage==1) for (const auto &s:r.slices) for (int i:s.raw)
            require(r.surfaceMask[std::size_t(i)],"rejected point entered slice");
        return stage!=2; // Stop after B so trajectory-specific choices cannot hide a filter regression.
    };
    const auto r=bwb::extract(f.points,config,hooks);
    require(r.cancelled && order==std::vector<int>({0,5,1,2}),"incorrect pipeline filter stage order");
    for (int i:r.selectedIndices) require(r.surfaceMask[std::size_t(i)],"rejected point entered B");
}

void intersectionUsesOnlyRetainedSupport() {
    const auto f=fixture();
    auto clean=f.points;clean.resize(f.cleanEnd);
    auto baseline=f.reference,masked=f.reference;
    baseline.slices.resize(1);baseline.slices[0].side=1;baseline.slices[0].offset=10;
    baseline.selectedSlice=0;
    for (std::size_t i=f.baseEnd;i<f.waveEnd;++i)
        if (clean[i].xyz.z()>=9 && clean[i].xyz.z()<=11) baseline.selectedIndices.push_back(int(i));
    masked=baseline;masked.surfaceMask.assign(f.points.size(),0);
    std::fill(masked.surfaceMask.begin(),masked.surfaceMask.begin()+f.cleanEnd,1);
    bwb::Config config;
    require(bwb::recoverSurfaceIntersection(clean,config,{},baseline),"clean intersection fixture failed");
    require(bwb::recoverSurfaceIntersection(f.points,config,{},masked),"masked intersection failed");
    require(baseline.projected.size()==masked.projected.size(),"mask changed coverage");
    for (std::size_t i=0;i<baseline.projected.size();++i)
        require((baseline.projected[i].xyz-masked.projected[i].xyz).norm()<1e-10 &&
                    baseline.projected[i].supportCount==masked.projected[i].supportCount,
                "rejected points leaked into surface intersection support");
}
}
int main() {
    try { preservationAndRemoval();pipelineOrderAndExclusion();intersectionUsesOnlyRetainedSupport();
          std::cout << "surface filter tests passed\n";return 0; }
    catch (const std::exception &e) { std::cerr << e.what() << '\n';return 1; }
}
