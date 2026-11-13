#include <fw/Algorithm.h>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QElapsedTimer>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>

namespace {
using namespace fw;
class Sample final : public IAlgorithm {
public:
    QString validate(const AlgorithmInput& input,const ParameterSet&) const override {
        if(input.clouds.isEmpty()||!hasFields(input.clouds.front(),{"x","y","z"})) return QStringLiteral("需要包含 XYZ 字段的点云");
        for(const auto& field:input.clouds.front().cloud->fields) {
            if((field.name=="x"||field.name=="y"||field.name=="z")&&field.datatype!=pcl::PCLPointField::FLOAT32) return QStringLiteral("此示例需要 float32 XYZ 字段；原始其他字段会保留");
        }
        return {};
    }
    AlgorithmResult execute(const AlgorithmInput& input,const ParameterSet& p,IExecutionContext& context) override {
        QElapsedTimer timer; timer.start(); const auto original=input.clouds.front();
        auto typed=toCloud<pcl::PointXYZ>(original); std::vector<int> indices; indices.reserve(typed->size());
        const auto limits=p["limits"].toObject(); const auto axis=p["axis"].toString();
        for(std::size_t i=0;i<typed->size();++i) {
            if((i%4096)==0&&context.isCancelled()) return AlgorithmResult::cancelled();
            const auto& v=typed->points[i]; const double coordinate=axis=="x"?v.x:axis=="y"?v.y:v.z;
            if(std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z)&&coordinate>=limits["lower"].toDouble()&&coordinate<=limits["upper"].toDouble()) indices.push_back(int(i));
        }
        pcl::PCLPointCloud2::Ptr clipped(new pcl::PCLPointCloud2);
        pcl::copyPointCloud(*original.cloud,indices,*clipped); clipped->is_dense=true;
        if(clipped->width*clipped->height<3) return AlgorithmResult::failed(QStringLiteral("裁剪后有效点少于 3 个"));
        pcl::PCLPointCloud2::Ptr downsampled(new pcl::PCLPointCloud2);
        if(p["enable_downsample"].toBool()) {
            pcl::VoxelGrid<pcl::PCLPointCloud2> filter; filter.setInputCloud(clipped);
            const float leaf=float(p["leaf_size"].toDouble()); filter.setLeafSize(leaf,leaf,leaf); filter.setDownsampleAllData(true); filter.filter(*downsampled);
        } else *downsampled=*clipped;
        CloudData stage=original; stage.cloud=downsampled; stage.name=QStringLiteral("降采样结果"); stage.id="downsample";
        if(!checkpoint(context,"downsample",stage.name,original,stage,timer.elapsed())) return AlgorithmResult::cancelled();
        context.progress(30);
        if(context.isCancelled()) return AlgorithmResult::cancelled();
        auto cloud=toCloud<pcl::PointXYZ>(stage);
        if(cloud->size()<3) return AlgorithmResult::failed(QStringLiteral("降采样后有效点少于 3 个"));
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; sor.setInputCloud(cloud); sor.setMeanK(std::min(p["mean_k"].toInt(),int(cloud->size())-1)); sor.setStddevMulThresh(p["stddev"].toDouble());
        std::vector<int> retained; sor.filter(retained);
        pcl::PCLPointCloud2::Ptr filtered(new pcl::PCLPointCloud2); pcl::copyPointCloud(*downsampled,retained,*filtered);
        stage.cloud=filtered; stage.name=QStringLiteral("离群过滤结果"); stage.id="filtered";
        if(!checkpoint(context,"outliers",stage.name,original,stage,timer.elapsed())) return AlgorithmResult::cancelled();
        context.progress(65); if(context.isCancelled()) return AlgorithmResult::cancelled();
        cloud=toCloud<pcl::PointXYZ>(stage);
        if(cloud->size()<3) return AlgorithmResult::failed(QStringLiteral("过滤后有效点少于 3 个"));
        pcl::SACSegmentation<pcl::PointXYZ> segment(false); // deterministic seed
        segment.setOptimizeCoefficients(true); segment.setModelType(pcl::SACMODEL_PLANE); segment.setMethodType(pcl::SAC_RANSAC);
        segment.setDistanceThreshold(p["distance"].toDouble()); segment.setMaxIterations(p["max_iterations"].toInt()); segment.setInputCloud(cloud);
        pcl::PointIndices inliers; pcl::ModelCoefficients coefficients; segment.segment(inliers,coefficients);
        if(context.isCancelled()) return AlgorithmResult::cancelled();
        if(inliers.indices.empty()) return AlgorithmResult::failed(QStringLiteral("没有找到平面，请调整参数"));
        pcl::PCLPointCloud2::Ptr plane(new pcl::PCLPointCloud2); pcl::copyPointCloud(*filtered,inliers.indices,*plane);
        CloudData output=original; output.cloud=plane; output.name=p["name"].toString(); output.id="plane";
        auto highlighted=DebugLayer::result(output.name,output); const auto color=p["result_color"].toArray();
        for(int i=0;i<3;++i) highlighted.style.color[i]=color[i].toDouble()/255; highlighted.style.opacity=color[3].toDouble()/255;
        QVector<DebugLayer> layers={DebugLayer::background(original.name,original),highlighted};
        if(context.debug().enabled()) {
            DebugFrame frame; frame.stepId="plane"; frame.title=QStringLiteral("平面分割结果"); frame.layers=layers; frame.metrics={{"planePoints",double(inliers.indices.size())}};
            if(context.debug().checkpoint(frame)==DebugAction::Stop) return AlgorithmResult::cancelled();
        }
        AlgorithmResult result; result.outputs={output}; result.layers=layers;
        QJsonArray model; for(float c:coefficients.values) model.append(c);
        result.metrics={{"planePoints",double(inliers.indices.size())},{"elapsedIncludingDebugMs",double(timer.elapsed())},{"planeCoefficients",model}};
        context.progress(100); context.log(QStringLiteral("平面提取完成，保留 %1 点").arg(inliers.indices.size())); return result;
    }
private:
    static bool checkpoint(IExecutionContext& context,const QString& id,const QString& title,const CloudData& input,const CloudData& output,qint64 elapsed) {
        if(context.isCancelled()) return false;
        if(!context.debug().enabled()) return true;
        DebugFrame frame; frame.stepId=id; frame.title=title;
        frame.layers={DebugLayer::background(QStringLiteral("原始点云"),input),DebugLayer::result(title,output)};
        frame.metrics={{"pointCount",double(output.cloud->width)*output.cloud->height},{"elapsedIncludingDebugMs",double(elapsed)}};
        return context.debug().checkpoint(frame)==DebugAction::Continue;
    }
};
class SamplePlugin final : public QObject,public IAlgorithmPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_ALGORITHM_IID FILE "metadata.json")
    Q_INTERFACES(fw::IAlgorithmPlugin)
public:
    AlgorithmDescriptor descriptor() const override { return {"cloud.sample_pipeline",QStringLiteral("示例：降采样 → 离群过滤 → 平面分割"),"1.0.0",{"x","y","z"}}; }
    QJsonObject parameterSchema() const override { QFile file(":/sample/parameters.json"); if(!file.open(QIODevice::ReadOnly)) return {}; return QJsonDocument::fromJson(file.readAll()).object(); }
    IAlgorithm* create() override { return new Sample; }
    void destroy(IAlgorithm* algorithm) override { delete algorithm; }
};
}
#include "SampleAlgorithm.moc"
