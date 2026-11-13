#pragma once
#include "SeamTypes.h"
#include <pcl/PCLPointCloud2.h>
#include <cstring>
#include <stdexcept>
#include <limits>

// The workbench keeps this field unchanged on PCD import/export.
inline std::vector<weld::ScanFrameRange> readFrames(const pcl::PCLPointCloud2& cloud) {
    const pcl::PCLPointField* field=nullptr;
    for(const auto& f:cloud.fields) if(f.name=="frame_index"||f.name=="frameIndex") {
        if(field) throw std::runtime_error("Ambiguous frame fields");
        field=&f;
    }
    if(!field) return {};
    if(cloud.is_bigendian||field->count!=1||field->datatype!=pcl::PCLPointField::INT32||field->offset+4>cloud.point_step)
        throw std::runtime_error("frame_index must be a little-endian int32 field");
    std::vector<weld::ScanFrameRange> frames;
    for(std::size_t i=0;i<std::size_t(cloud.width)*cloud.height;++i) {
        const auto offset=(i/cloud.width)*cloud.row_step+(i%cloud.width)*cloud.point_step+field->offset;
        if(offset+4>cloud.data.size()) throw std::runtime_error("Truncated frame field");
        std::int32_t id; std::memcpy(&id,cloud.data.data()+offset,4);
        if(id<0||(!frames.empty()&&id<frames.back().frameIndex)) throw std::runtime_error("Frame IDs must be nonnegative, contiguous per frame, and increasing");
        if(frames.empty()||id!=frames.back().frameIndex) frames.push_back({i,0,id});
        ++frames.back().count;
    }
    return frames;
}
inline void writeFrameIds(pcl::PCLPointCloud2& cloud,const std::vector<int>& ids) {
    if(ids.size()!=std::size_t(cloud.width)*cloud.height) throw std::runtime_error("Frame count does not match cloud");
    const auto oldStep=cloud.point_step,oldRow=cloud.row_step;
    auto old=std::move(cloud.data);
    pcl::PCLPointField field; field.name="frame_index"; field.offset=oldStep; field.datatype=pcl::PCLPointField::INT32; field.count=1;
    cloud.fields.push_back(field); cloud.point_step+=4; cloud.row_step=cloud.point_step*cloud.width;
    cloud.data.resize(std::size_t(cloud.row_step)*cloud.height);
    for(std::size_t i=0;i<ids.size();++i) {
        auto dst=cloud.data.data()+i*cloud.point_step;
        std::memcpy(dst,old.data()+(i/cloud.width)*oldRow+(i%cloud.width)*oldStep,oldStep);
        const std::int32_t id=ids[i]; std::memcpy(dst+oldStep,&id,4);
    }
}
