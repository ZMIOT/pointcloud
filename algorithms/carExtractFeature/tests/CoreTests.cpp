#include "Synthetic.h"
#include <iostream>
#include <random>
#include <limits>
#include <Eigen/Geometry>
void goodEdge(const lap::Result& r,std::size_t minCount) {
    require(r.ok,r.message);require(r.sections.size()==140,"Wrong recovered section count");
    require(r.measured.size()>=minCount,"Too many missing edge points: "+std::to_string(r.measured.size()));
    for(const auto& p:r.measured) {
        require((p.xyz-r.input[p.source].xyz).norm()==0,"Measured XYZ is not original");
        const double s=p.section*0.5;
        require(std::abs(p.xyz.x()-edgeX(s))<0.65,"Wrong sheet/edge selected at "+std::to_string(p.section));
        require(std::abs(p.xyz.z()-baseZ(p.xyz.x(),s)-2)<0.2,"Noise or lower sheet selected");
    }
}
int main() {
    try {
        auto clean=lap::detect(synthetic());goodEdge(clean,130);
        std::vector<lap::Point> dense;
        for(int f=0;f<140;++f) {
            const double s=f*0.5;
            for(int j=-500;j<=500;++j) {
                if(j==0) continue;
                const double x=edgeX(s)+j*0.03;
                const double z=baseZ(x,s)+(j<0?2.:0.)+0.008*std::sin(f*1.3+j*2.1);
                dense.push_back({lap::Vec(x,s-0.06845*x+0.05279*z,z),std::uint32_t(dense.size())});
            }
        }
        auto highResolution=lap::detect(dense);
        require(highResolution.ok,highResolution.message);
        require(highResolution.sections.size()==140&&highResolution.measured.size()>=125,"Dense scans lost the edge");
        for(const auto& p:highResolution.measured) {
            const double s=p.section*0.5;
            require(std::abs(p.xyz.x()-edgeX(s))<0.18,"Dense edge was not refined to an original endpoint");
            require((p.xyz-dense[p.source].xyz).norm()==0,"Dense source XYZ changed");
        }
        std::vector<lap::Point> spurred;
        for(int f=0;f<140;++f) for(int j=0;j<1000;++j) {
            if(j==500&&f>=55&&f<=85) {
                const double s=f*0.5,x=edgeX(s)+0.005,z=baseZ(x,s)+3.5;
                spurred.push_back({lap::Vec(x,s-0.06845*x+0.05279*z,z),std::uint32_t(spurred.size())});
            }
            spurred.push_back({dense[f*1000+j].xyz,std::uint32_t(spurred.size())});
        }
        const auto spurResult=lap::detect(spurred);
        require(spurResult.ok,spurResult.message);
        int supportedSpurSections=0;
        for(const auto& p:spurResult.measured) if(p.section>=55&&p.section<=85) {
            ++supportedSpurSections;
            const double s=p.section*0.5;
            require(std::abs(p.xyz.x()-edgeX(s))<0.18,"Persistent spur moved the dense upper edge");
            require(std::abs(p.xyz.z()-baseZ(p.xyz.x(),s)-2)<0.2,"Persistent spur was selected as the edge");
        }
        require(supportedSpurSections>=27,"Persistent spurs created a long edge gap");
        std::vector<lap::Point> detached,transient;
        for(int f=0;f<140;++f) {
            const double s=f*0.5;
            for(int j=-500;j<=500;++j) {
                if(j==0) {
                    if(f>=55&&f<=85) for(int k=0;k<5;++k) {
                        const double x=edgeX(s)+0.85+0.05*k;
                        const double z=baseZ(x,s)+2.+0.006*std::sin(f+k);
                        const lap::Vec xyz(x,s-0.06845*x+0.05279*z,z);
                        detached.push_back({xyz,std::uint32_t(detached.size())});
                        if(f==70) transient.push_back({xyz,std::uint32_t(transient.size())});
                    }
                    continue;
                }
                const double x=edgeX(s)+(j<0?j*0.03:1.6+j*0.03);
                const double z=baseZ(x,s)+(j<0?2.:0.)+0.008*std::sin(f*1.3+j*2.1);
                const lap::Vec xyz(x,s-0.06845*x+0.05279*z,z);
                detached.push_back({xyz,std::uint32_t(detached.size())});
                transient.push_back({xyz,std::uint32_t(transient.size())});
            }
        }
        const auto detachedResult=lap::detect(detached);
        require(detachedResult.ok,detachedResult.message);
        int detachedCoverage=0;
        for(const auto& p:detachedResult.measured) if(p.section>=55&&p.section<=85) {
            ++detachedCoverage;
            const double s=p.section*0.5;
            require(std::abs(p.xyz.x()-(edgeX(s)+1.05))<0.25,"Repeatable fragment tip was not selected");
        }
        require(detachedCoverage>=27,"Short floating piece caused a long trajectory gap");
        for(const auto& p:detachedResult.trajectory) if(!p.interpolated&&p.section>=55&&p.section<=85) {
            const double s=p.section*0.5;
            require(std::abs(p.xyz.x()-(edgeX(s)+1.05))<0.25,"Smoothed trajectory left the repeatable fragment tip");
        }
        const auto transientResult=lap::detect(transient);
        require(transientResult.ok,transientResult.message);
        for(const auto& p:transientResult.measured) if(p.section==70)
            require(std::abs(p.xyz.x()-edgeX(35.))<0.25,"One-section fragment displaced the upper edge");
        auto dirty=lap::detect(synthetic(true));goodEdge(dirty,110);
        auto gaps=lap::detect(synthetic(false,true));goodEdge(gaps,115);
        bool bridge=false,segmented=false;
        for(const auto& p:gaps.trajectory) {
            require(!(p.section>=75&&p.section<=84),"Long occlusion was interpolated");
            if(p.interpolated&&(p.section==40||p.section==41)) bridge=true;
            if(p.segment>0) segmented=true;
        }
        require(bridge,"Short supported gap was not interpolated");require(segmented,"Long gap did not split trajectory");
        lap::Config noGap;noGap.maxMissing=0;auto noBridge=lap::detect(synthetic(false,true),noGap);
        for(const auto& p:noBridge.trajectory) require(!p.interpolated,"maxMissing=0 generated points");
        require(!lap::detect(synthetic(false,false,false)).ok,"Smooth single surface produced an edge");
        int stages=0;lap::Hooks debug;debug.stage=[&](int i,const lap::Result&){require(i==stages,"Stage order");++stages;return true;};
        auto stepped=lap::detect(synthetic(),{},debug);require(stages==6,"Missing debug stages");
        require(stepped.measured.size()==clean.measured.size(),"Debug changes output");
        for(std::size_t i=0;i<clean.trajectory.size();++i) require((clean.trajectory[i].xyz-stepped.trajectory[i].xyz).norm()==0,"Debug changes trajectory");
        lap::Hooks stop;stop.stage=[](int i,const lap::Result&){return i<2;};require(lap::detect(synthetic(),{},stop).cancelled,"Checkpoint cancellation failed");
        int checks=0;stop.stage={};stop.cancelled=[&]{return ++checks>20;};require(lap::detect(synthetic(),{},stop).cancelled,"Mid-run cancellation failed");
        for(double factor:{0.8,1.,1.2,1.5}) for(int which=0;which<3;++which) {
            lap::Config cfg;if(which==0) cfg.neighborhoodScale=factor;if(which==1) cfg.smoothing=factor;if(which==2) cfg.evidenceScale=factor;
            goodEdge(lap::detect(synthetic(true),cfg),105);
        }
        auto reverse=synthetic();std::reverse(reverse.begin(),reverse.end());auto reversed=lap::detect(reverse);
        require(reversed.ok,reversed.message);require(reversed.measured.size()>=130,"Reverse scan order failed");
        auto scaled=synthetic();for(auto& p:scaled) p.xyz*=0.001;auto metric=lap::detect(scaled);
        require(metric.ok&&metric.measured.size()==clean.measured.size(),"Unit-scale invariance failed");
        const Eigen::Matrix3d rotation=Eigen::AngleAxisd(0.65,lap::Vec(1,2,3).normalized()).toRotationMatrix();
        auto rotated=synthetic();for(auto& p:rotated) p.xyz=rotation*p.xyz;
        lap::Config rotatedConfig;rotatedConfig.up=rotation*lap::Vec::UnitZ();auto rotatedResult=lap::detect(rotated,rotatedConfig);
        require(rotatedResult.ok&&rotatedResult.measured.size()>=130,"Rotated coordinate frame failed");
        auto steep=synthetic();
        for(auto& p:steep) {
            const double s=p.xyz.y()+0.06845*p.xyz.x()-0.05279*p.xyz.z();
            const double ramp=1.5*std::max(0.,std::min(6.,s-23.));p.xyz.z()+=ramp;p.xyz.y()+=0.05279*ramp;
        }
        auto steepResult=lap::detect(steep);require(steepResult.ok&&steepResult.measured.size()>=125,"Continuous steep ramp rejected");
        for(const auto& p:steepResult.trajectory) if(!p.interpolated)
            require((p.xyz-steep[p.source].xyz).norm()<=0.76*steepResult.pointSpacing,"Smoothing flattened a real ramp");
        lap::Config manual;manual.scanNormal=lap::Vec(0.06845,1,-0.05279);
        goodEdge(lap::detect(synthetic(),manual),130);
        manual.scanNormal=lap::Vec::UnitX();require(!lap::detect(synthetic(),manual).ok,"Wrong scan plane accepted");
        auto shuffled=synthetic();std::mt19937 rng(7);std::shuffle(shuffled.begin(),shuffled.end(),rng);
        require(!lap::detect(shuffled).ok,"Scrambled scan order accepted");
        require(!lap::detect({}).ok,"Empty accepted");auto bad=synthetic();bad[1].xyz.x()=std::numeric_limits<double>::quiet_NaN();require(!lap::detect(bad).ok,"NaN accepted");
        lap::Config invalid;invalid.up.setZero();require(!lap::detect(synthetic(),invalid).ok,"Zero up accepted");
        std::cout<<"Core tests passed: clean="<<clean.measured.size()<<", noisy="<<dirty.measured.size()<<", gaps="<<gaps.measured.size()<<"\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}return 0;
}
