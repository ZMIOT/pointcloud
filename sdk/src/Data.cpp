#include <fw/Data.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fw {
DebugLayer DebugLayer::background(const QString& name, const CloudData& data) {
    DebugLayer layer; layer.name=name; layer.data=data; layer.role=LayerRole::Background;
    layer.style.color={0.55,0.58,0.62}; layer.style.pointSize=1; layer.style.opacity=0.25;
    return layer;
}
DebugLayer DebugLayer::result(const QString& name, const CloudData& data) {
    DebugLayer layer; layer.name=name; layer.data=data; return layer;
}
bool hasFields(const CloudData& data, const QStringList& required) {
    if(!data.cloud) return false;
    for(const auto& field: required) {
        bool found=false;
        for(const auto& f: data.cloud->fields) if(QString::fromStdString(f.name)==field) found=true;
        if(!found) return false;
    }
    return true;
}
quint64 frameBytes(const DebugFrame& frame) {
    quint64 bytes=0;
    for(const auto& layer: frame.layers) {
        if(layer.data.cloud) bytes+=layer.data.cloud->data.size();
        bytes+=layer.image.total()*layer.image.elemSize();
        bytes+=quint64(layer.geometry.vertices.size())*sizeof(Vec3);
    }
    return bytes;
}
FramePtr snapshot(const DebugFrame& source, quint64 budget) {
    auto copy=std::make_shared<DebugFrame>(source);
    const quint64 total=frameBytes(source);
    const double ratio=total>budget ? double(budget)/double(total) : 1.0;
    copy->sampled=ratio<1.0;
    for(auto& layer: copy->layers) {
        if(layer.data.cloud) {
            const auto& src=*layer.data.cloud;
            pcl::PCLPointCloud2::Ptr dst(new pcl::PCLPointCloud2);
            if(ratio>=1) *dst=src;
            else {
                // Copy metadata only: never allocate a full cloud just to sample it.
                dst->header=src.header; dst->fields=src.fields;
                dst->is_bigendian=src.is_bigendian; dst->is_dense=src.is_dense; dst->point_step=src.point_step;
                const quint64 count=quint64(src.width)*src.height;
                const quint64 keep=std::min(count, quint64(std::floor(count*ratio)));
                dst->width=static_cast<std::uint32_t>(keep); dst->height=1;
                dst->row_step=dst->width*dst->point_step; dst->data.resize(dst->row_step);
                for(quint64 i=0;i<keep;++i) {
                    const quint64 index=i*count/keep;
                    const quint64 offset=(index/src.width)*src.row_step+(index%src.width)*src.point_step;
                    if(offset+src.point_step>src.data.size()) throw std::runtime_error("Invalid cloud buffer");
                    std::memcpy(dst->data.data()+i*src.point_step, src.data.data()+offset, src.point_step);
                }
            }
            layer.data.cloud=dst;
        }
        if(!layer.image.empty()) {
            if(ratio>=1) layer.image=layer.image.clone();
            else {
                const int w=int(layer.image.cols*std::sqrt(ratio)), h=int(layer.image.rows*std::sqrt(ratio));
                if(w<1||h<1) layer.image.release();
                else { cv::Mat image; cv::resize(layer.image,image,cv::Size(w,h),0,0,cv::INTER_AREA); layer.image=image; }
            }
        }
        if(ratio<1 && !layer.geometry.vertices.isEmpty()) {
            // Dropping complete primitives preserves their meaning better than changing vertices.
            layer.geometry.vertices.clear();
        }
    }
    if(copy->sampled) copy->message+=QStringLiteral("\n显示快照已采样（几何辅助图元可能省略），计算结果未改变。");
    return copy;
}
AlgorithmResult freezeResult(const AlgorithmResult& source) {
    auto result=source;
    for(auto& out: result.outputs) if(out.cloud) out.cloud=pcl::PCLPointCloud2::ConstPtr(new pcl::PCLPointCloud2(*out.cloud));
    DebugFrame frame; frame.layers=source.layers;
    result.layers=snapshot(frame,frameBytes(frame))->layers;
    return result;
}
}
