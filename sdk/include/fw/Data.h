#pragma once
#include "Export.h"
#include <QJsonObject>
#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QVector>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <opencv2/core.hpp>
#include <array>
#include <memory>

namespace fw {
using ParameterSet = QJsonObject;
struct CloudData {
    QString id, name, coordinateFrame = "world", unit = "unknown";
    pcl::PCLPointCloud2::ConstPtr cloud;
};
struct Vec3 { double x=0, y=0, z=0; };
enum class GeometryKind { Points, Line, Polyline, Plane, Box };
struct Geometry { GeometryKind kind = GeometryKind::Points; QVector<Vec3> vertices; };
enum class LayerRole { Background, Result, Auxiliary };
struct LayerStyle {
    std::array<double, 3> color {0.2, 0.95, 0.45};
    double opacity=1, pointSize=5, lineWidth=3;
    bool visible=true;
};
struct FW_API DebugLayer {
    QString name;
    LayerRole role = LayerRole::Result;
    CloudData data;
    cv::Mat image;
    Geometry geometry;
    LayerStyle style;
    static DebugLayer background(const QString&, const CloudData&);
    static DebugLayer result(const QString&, const CloudData&);
    template<class PointT> static DebugLayer result(const QString& name, const pcl::PointCloud<PointT>& cloud) {
        pcl::PCLPointCloud2::Ptr converted(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(cloud, *converted);
        CloudData data; data.name=name; data.cloud=converted;
        return result(name, data);
    }
};
struct DebugFrame {
    QString stepId, title, message;
    QVector<DebugLayer> layers;
    QJsonObject metrics;
    quint64 runId=0, sequence=0;
    bool sampled=false;
};
using FramePtr = std::shared_ptr<const DebugFrame>;
struct AlgorithmInput { QVector<CloudData> clouds; QVector<cv::Mat> images; };
enum class ResultStatus { Success, Cancelled, Failed };
struct AlgorithmResult {
    ResultStatus status=ResultStatus::Success;
    QString message;
    QVector<CloudData> outputs;
    QVector<DebugLayer> layers;
    QJsonObject metrics;
    static AlgorithmResult cancelled() { AlgorithmResult r; r.status=ResultStatus::Cancelled; return r; }
    static AlgorithmResult failed(const QString& s) { AlgorithmResult r; r.status=ResultStatus::Failed; r.message=s; return r; }
};
template<class PointT> typename pcl::PointCloud<PointT>::Ptr toCloud(const CloudData& data) {
    typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>);
    if(data.cloud) pcl::fromPCLPointCloud2(*data.cloud, *result);
    return result;
}
FW_API bool hasFields(const CloudData&, const QStringList&);
FW_API FramePtr snapshot(const DebugFrame&, quint64 maxBytes);
FW_API quint64 frameBytes(const DebugFrame&);
FW_API AlgorithmResult freezeResult(const AlgorithmResult&);
}
Q_DECLARE_METATYPE(fw::FramePtr)
Q_DECLARE_METATYPE(fw::AlgorithmResult)
