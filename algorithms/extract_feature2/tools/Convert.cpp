#include "PointCloudIO.h"
#include "../CloudFrames.h"
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <iostream>
int main(int argc,char** argv) {
    if(argc!=3) {std::cerr<<"Usage: ExtractFeature2Convert input.txt output.pcd\n";return 2;}
    std::string error; std::vector<weld::ScanFrameRange> frames;
    auto cloud=weld::PointCloudIO::load(argv[1],&error,&frames);
    if(!cloud||cloud->empty()) {std::cerr<<error;return 1;}
    pcl::PCLPointCloud2 output; pcl::toPCLPointCloud2(*cloud,output);
    if(!frames.empty()) {
        std::vector<int> ids(cloud->size());
        for(const auto& f:frames) std::fill(ids.begin()+f.begin,ids.begin()+f.begin+f.count,f.frameIndex);
        writeFrameIds(output,ids);
    }
    const int code=pcl::PCDWriter().writeBinary(argv[2],output);
    std::cout<<cloud->size()<<" points, "<<frames.size()<<" frames.\n";return code?1:0;
}
