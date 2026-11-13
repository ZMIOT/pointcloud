#include <fw/UiInterfaces.h>
#include <QVTKOpenGLNativeWidget.h>
#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QThread>
#include <QApplication>
#include <QImage>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkVersion.h>
#include <opencv2/imgproc.hpp>
#include <QTabBar>
#include <QScreen>
#include <QShortcut>
#include <QLocale>
#include "Theme.h"

namespace {
using namespace fw;
class SceneInteractorStyle final : public pcl::visualization::PCLVisualizerInteractorStyle {
public:
    static SceneInteractorStyle* New() { return new SceneInteractorStyle; }
    vtkTypeMacro(SceneInteractorStyle,pcl::visualization::PCLVisualizerInteractorStyle);
    // Reuse PCL's pan gesture, including VTK's focus capture and release.
    void OnRightButtonDown() override { Superclass::OnMiddleButtonDown(); }
    void OnRightButtonUp() override { Superclass::OnMiddleButtonUp(); }
};

class SceneView final : public QWidget, public ISceneView {
public:
    explicit SceneView(QWidget* parent):QWidget(parent) {
        Q_ASSERT(QThread::currentThread()==qApp->thread());
        auto layout=new QVBoxLayout(this); layout->setContentsMargins(0,0,0,0);
        tabs_=new QTabWidget; layout->addWidget(tabs_);
        vtk_=new QVTKOpenGLNativeWidget; tabs_->addTab(vtk_,QStringLiteral("点云 / 几何"));
        tabs_->tabBar()->hide(); vtk_->setAccessibleName(QStringLiteral("三维点云视图"));
        vtk_->setToolTip(QStringLiteral("左键拖动旋转 · 右键拖动平移 · 滚轮缩放"));
        renderer_=vtkSmartPointer<vtkRenderer>::New(); window_=vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        window_->SetMultiSamples(0); window_->AddRenderer(renderer_);
#if VTK_MAJOR_VERSION >= 9
        vtk_->setRenderWindow(window_);
#else
        vtk_->SetRenderWindow(window_);
#endif
        auto style=vtkSmartPointer<SceneInteractorStyle>::New();
        int argc=1; char name[]="workbench"; char* argv[]={name,nullptr};
        viewer_.reset(new pcl::visualization::PCLVisualizer(argc,argv,renderer_,window_,"workbench",style,false));
        viewer_->setShowFPS(false);
        viewer_->setupInteractor(window_->GetInteractor(),window_);
        viewer_->setBackgroundColor(0.055,0.065,0.085); viewer_->addCoordinateSystem(0.1); viewer_->initCameraParameters();
        status_=new QLabel; status_->setWordWrap(true); status_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred); status_->setMargin(6); workbench::setRole(status_,"muted"); layout->addWidget(status_);
    }
    QWidget* widget() override { return this; }
    void setLayers(const QVector<DebugLayer>& layers) override {
        Q_ASSERT(QThread::currentThread()==qApp->thread());
        viewer_->removeAllPointClouds(); viewer_->removeAllShapes();
        for(auto actor:actors_) renderer_->RemoveActor(actor); actors_.clear();
        while(tabs_->count()>1) { auto page=tabs_->widget(1); tabs_->removeTab(1); delete page; }
        QStringList legends; int index=0;
        for(const auto& layer:layers) {
            const auto id=std::string("layer_")+std::to_string(index++); if(!layer.style.visible) continue;
            const auto& style=layer.style;
            if(layer.data.cloud&&hasFields(layer.data,{"x","y","z"})) {
                auto cloud=toCloud<pcl::PointXYZ>(layer.data);
                if(!cloud->empty()) {
                    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud,int(style.color[0]*255),int(style.color[1]*255),int(style.color[2]*255));
                    viewer_->addPointCloud<pcl::PointXYZ>(cloud,color,id);
                    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,style.pointSize,id);
                    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,style.opacity,id);
                }
                legends<<layer.name+QStringLiteral("：%1 点 [%2]").arg(QLocale().toString(qulonglong(cloud->size()))).arg(layer.data.unit=="unknown"?QStringLiteral("坐标单位"):layer.data.unit);
            }
            if(!layer.geometry.vertices.isEmpty()) addGeometry(layer);
            if(!layer.image.empty()) addImage(layer);
        }
        tabs_->tabBar()->setVisible(tabs_->count()>1);
        status_->setText(legends.join("   ·   ")); status_->setVisible(!legends.isEmpty()); window_->Render(); vtk_->update();
    }
    void fit() override { viewer_->resetCamera(); window_->Render(); vtk_->update(); }
private:
    QTabWidget* tabs_; QVTKOpenGLNativeWidget* vtk_; QLabel* status_;
    vtkSmartPointer<vtkRenderer> renderer_; vtkSmartPointer<vtkGenericOpenGLRenderWindow> window_;
    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer_;
    QVector<vtkSmartPointer<vtkActor>> actors_;
    void addGeometry(const DebugLayer& layer) {
        const auto& g=layer.geometry; auto vertices=g.vertices;
        if(g.kind==GeometryKind::Box&&vertices.size()==2) {
            const auto a=vertices[0],b=vertices[1]; vertices.clear();
            for(int i=0;i<8;++i) vertices.push_back({(i&1)?b.x:a.x,(i&2)?b.y:a.y,(i&4)?b.z:a.z});
        }
        auto points=vtkSmartPointer<vtkPoints>::New(); for(const auto& v:vertices) points->InsertNextPoint(v.x,v.y,v.z);
        auto cells=vtkSmartPointer<vtkCellArray>::New(); auto poly=vtkSmartPointer<vtkPolyData>::New(); poly->SetPoints(points);
        if(g.kind==GeometryKind::Points) {
            for(vtkIdType i=0;i<vertices.size();++i) cells->InsertNextCell(1,&i); poly->SetVerts(cells);
        } else if(g.kind==GeometryKind::Box&&vertices.size()==8) {
            for(vtkIdType i=0;i<8;++i) for(int bit:{1,2,4}) if(!(i&bit)) { vtkIdType edge[2]={i,i|bit}; cells->InsertNextCell(2,edge); }
            poly->SetLines(cells);
        } else {
            cells->InsertNextCell(vertices.size()); for(vtkIdType i=0;i<vertices.size();++i) cells->InsertCellPoint(i);
            if(g.kind==GeometryKind::Plane) poly->SetPolys(cells); else poly->SetLines(cells);
        }
        auto mapper=vtkSmartPointer<vtkPolyDataMapper>::New(); mapper->SetInputData(poly);
        auto actor=vtkSmartPointer<vtkActor>::New(); actor->SetMapper(mapper);
        actor->GetProperty()->SetColor(layer.style.color[0],layer.style.color[1],layer.style.color[2]); actor->GetProperty()->SetOpacity(g.kind==GeometryKind::Plane?std::min(0.4,layer.style.opacity):layer.style.opacity);
        actor->GetProperty()->SetLineWidth(layer.style.lineWidth); actor->GetProperty()->SetPointSize(layer.style.pointSize);
        if(g.kind==GeometryKind::Plane) { actor->GetProperty()->EdgeVisibilityOn(); actor->GetProperty()->SetEdgeColor(layer.style.color[0],layer.style.color[1],layer.style.color[2]); }
        renderer_->AddActor(actor); actors_<<actor;
    }
    void addImage(const DebugLayer& layer) {
        cv::Mat bytes,rgb;
        if(layer.image.depth()!=CV_8U) cv::normalize(layer.image,bytes,0,255,cv::NORM_MINMAX,CV_8U); else bytes=layer.image;
        if(bytes.channels()==1) cv::cvtColor(bytes,rgb,cv::COLOR_GRAY2RGB);
        else if(bytes.channels()==3) cv::cvtColor(bytes,rgb,cv::COLOR_BGR2RGB);
        else if(bytes.channels()==4) cv::cvtColor(bytes,rgb,cv::COLOR_BGRA2RGBA); else return;
        QImage image(rgb.data,rgb.cols,rgb.rows,int(rgb.step),rgb.channels()==4?QImage::Format_RGBA8888:QImage::Format_RGB888);
        auto label=new QLabel; label->setPixmap(QPixmap::fromImage(image.copy())); label->setAlignment(Qt::AlignCenter);
        auto scroll=new QScrollArea; scroll->setWidget(label); tabs_->addTab(scroll,layer.name);
    }
};
class VisualizationPlugin final : public QObject, public IVisualizationPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_VISUALIZATION_IID FILE "metadata.json")
    Q_INTERFACES(fw::IVisualizationPlugin)
public:
    ISceneView* createView(QWidget* parent) override { return new SceneView(parent); }
    void destroyView(ISceneView* view) override { delete view; }
    QWidget* showDebug(FramePtr frame,const DebugControls& controls,QWidget* parent) override {
        auto dialog=new QDialog(parent); dialog->setObjectName("debugWindow"); dialog->setAttribute(Qt::WA_DeleteOnClose);
        const auto available=QApplication::primaryScreen()->availableGeometry().size();
        dialog->resize(std::min(980,available.width()-40),std::min(740,available.height()-50));
        dialog->setWindowTitle(QStringLiteral("运行 %1 · 步骤 %2 · %3").arg(frame->runId).arg(frame->sequence).arg(frame->title));
        auto layout=new QVBoxLayout(dialog); layout->setContentsMargins(20,16,20,16); layout->setSpacing(12);
        auto context=new QLabel(QStringLiteral("运行 %1  /  步骤 %2").arg(frame->runId).arg(frame->sequence)); workbench::setRole(context,"muted"); layout->addWidget(context);
        auto title=new QLabel(frame->title); title->setWordWrap(true); workbench::setRole(title,"title"); layout->addWidget(title);
        if(!frame->message.isEmpty()) { auto message=new QLabel(frame->message); message->setWordWrap(true); layout->addWidget(message); }
        auto view=new SceneView(dialog); layout->addWidget(view,1); view->setLayers(frame->layers);
        QStringList metrics; for(auto it=frame->metrics.begin();it!=frame->metrics.end();++it) metrics<<it.key()+": "+it.value().toVariant().toString();
        if(!metrics.isEmpty()) { auto label=new QLabel(metrics.join("   ·   ")); label->setWordWrap(true); workbench::setRole(label,"muted"); layout->addWidget(label); }
        auto buttons=new QHBoxLayout; layout->addLayout(buttons);
        auto add=[&](const QString& text,std::function<void()> action,const QKeySequence& key) { auto button=new QPushButton(text); button->setAutoDefault(false); buttons->addWidget(button); QObject::connect(button,&QPushButton::clicked,dialog,[action] { if(action) action(); }); auto shortcut=new QShortcut(key,dialog); QObject::connect(shortcut,&QShortcut::activated,button,&QPushButton::click); return button; };
        auto fit=new QPushButton(QStringLiteral("适应视图")); buttons->addWidget(fit); connect(fit,&QPushButton::clicked,dialog,[view] { view->fit(); });
        fit->setAutoDefault(false); buttons->addStretch();
        add(QStringLiteral("停止"),controls.stop,QKeySequence(Qt::SHIFT|Qt::Key_F5));
        add(QStringLiteral("继续运行 · F6"),controls.resume,QKeySequence(Qt::Key_F6));
        auto next=add(QStringLiteral("下一步 · F10"),controls.next,QKeySequence(Qt::Key_F10)); workbench::setRole(next,"primary");
        dialog->show(); view->fit(); return dialog;
    }
};
}
#include "Visualization.moc"
