#pragma once
#include "LapEdge.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
inline void require(bool ok,const std::string& message) {if(!ok) throw std::runtime_error(message);}
inline double edgeX(double s) {return 1.5*std::sin(s/12.);}
inline double baseZ(double x,double s) {return 0.55*x+1.2*std::sin(s/9.);}
inline std::vector<lap::Point> synthetic(bool noise=false,bool missing=false,bool step=true) {
    std::vector<lap::Point> points;
    for(int f=0;f<140;++f) {
        const double s=f*0.5;std::vector<lap::Vec> frame;
        for(int j=-20;j<=20;++j) {
            if(j==1) continue;
            if(missing&&((f>=40&&f<=41)||(f>=75&&f<=84))&&j<=0) continue;
            const double x=edgeX(s)+j*0.6;
            const double z=baseZ(x,s)+(j<=0&&step?2.:0.)+0.012*std::sin(f*1.3+j*2.1);
            frame.emplace_back(x,s-0.06845*x+0.05279*z,z);
        }
        if(noise&&f%9==0) {
            const double x=edgeX(s)+0.2,z=baseZ(x,s)+5.;frame.emplace_back(x,s-0.06845*x+0.05279*z,z);
        }
        if(noise&&f>=55&&f<=58) for(int k=0;k<3;++k) {
            const double x=edgeX(s)+0.1*k,z=baseZ(x,s)+4.5;frame.emplace_back(x,s-0.06845*x+0.05279*z,z);
        }
        std::sort(frame.begin(),frame.end(),[](const lap::Vec& a,const lap::Vec& b){return a.x()<b.x();});
        for(const auto& p:frame) points.push_back({p,std::uint32_t(points.size())});
    }
    return points;
}
