#include "Runtime.h"
#include <fw/Parameters.h>
#include <QApplication>
#include <QMessageBox>
#include <QPointer>
#include <QSurfaceFormat>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QFont>
#include <QCommandLineParser>
#include <QScreen>
#include <QWindow>
#include <pcl/point_types.h>
#include "Theme.h"

int main(int argc,char** argv) {
#if QT_VERSION >= QT_VERSION_CHECK(5,14,0)
    // Qt 5 otherwise rounds Windows 150% scaling to 200%.
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
#endif
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc,argv); app.setOrganizationName("PointCloudFramework"); app.setApplicationName("Workbench");
    QCommandLineParser parser; parser.addHelpOption();
    parser.addOption({"smoke-test","Run an automated rendering smoke test"});
    parser.addOption({"plugin-root","Plugin search directory","directory",QCoreApplication::applicationDirPath()+"/plugins"});
    parser.addOption({"algorithm","Algorithm used by the smoke test","id","cloud.sample_pipeline"});
    parser.addOption({"expected-checkpoints","Expected smoke test checkpoint count","count","3"});
    parser.addOption({"theme","Override panel theme (light or dark)","theme"});
    parser.addOption({"font-scale","UI font scale for accessibility verification","scale","1"});
    parser.addOption({"window-size","Initial logical window size, e.g. 1280x800","size"});
    parser.addOption({"smoke-name","Synthetic input name for layout regression checks","name","Synthetic plane"});
    parser.process(app);
    bool validScale=false; const double fontScale=parser.value("font-scale").toDouble(&validScale);
    if(!validScale||fontScale<1||fontScale>2) parser.showHelp(1);
    workbench::initializeAppearance(app,fontScale);
    if(parser.isSet("theme")) {
        if(parser.value("theme")!="dark"&&parser.value("theme")!="light") parser.showHelp(1);
        workbench::applyTheme(app,parser.value("theme")=="dark");
    }
    fw::Runtime runtime; QString error;
    const bool smoke=parser.isSet("smoke-test");
    const auto algorithm=parser.value("algorithm"); const int expected=parser.value("expected-checkpoints").toInt();
    if(!runtime.initialize(parser.value("plugin-root"),error)) {
        if(smoke) { qCritical("%s",qPrintable(error)); return 2; }
        QMessageBox::critical(nullptr,QStringLiteral("启动失败"),error); return 1;
    }
    QPointer<QWidget> window=runtime.createWindow(); window->show();
    if(parser.isSet("window-size")) {
        const auto parts=parser.value("window-size").split('x');
        if(parts.size()==2&&parts[0].toInt()>0&&parts[1].toInt()>0) window->resize(parts[0].toInt(),parts[1].toInt());
    }
    if(smoke) {
        auto count=std::make_shared<int>(0);
        auto debugFits=std::make_shared<bool>(true);
        QTimer::singleShot(100,&app,[window] { if(window) window->grab().save(QCoreApplication::applicationDirPath()+"/smoke-empty.png"); });
        QObject::connect(&runtime,&fw::Host::debugReady,&app,[count](fw::FramePtr) { ++*count; });
        QObject::connect(&runtime,&fw::Host::pausedChanged,&app,[&runtime,debugFits](bool paused) {
            if(paused) QTimer::singleShot(80,&runtime,[&runtime,debugFits] {
                for(auto widget:QApplication::topLevelWidgets()) if(widget->objectName()=="debugWindow") {
                    widget->grab().save(QCoreApplication::applicationDirPath()+"/smoke-debug.png");
                    if(widget->windowHandle()&&!widget->windowHandle()->screen()->availableGeometry().contains(widget->frameGeometry())) *debugFits=false;
                }
                runtime.next();
            });
        });
        QObject::connect(&runtime,&fw::Host::finished,&app,[&app,window,count,debugFits,algorithm,expected](fw::AlgorithmResult result) {
            QTimer::singleShot(300,&app,[&app,window,count,debugFits,result,algorithm,expected] {
                if(window) window->grab().save(QCoreApplication::applicationDirPath()+"/smoke-main.png");
                QJsonObject report{{"algorithm",algorithm},{"checkpoints",*count},{"success",result.status==fw::ResultStatus::Success},{"outputs",result.outputs.size()},{"message",result.message}};
                report["theme"]=app.property("workbench.dark").toBool()?"dark":"light";
                report["debugFitsScreen"]=*debugFits;
                report["fontFamily"]=app.font().family(); report["fontPoints"]=app.font().pointSizeF();
                bool fitsScreen=false;
                bool controlsFit=false;
                if(window) {
                    report["logicalWidth"]=window->width(); report["logicalHeight"]=window->height(); report["devicePixelRatio"]=window->devicePixelRatioF();
                    auto screen=window->windowHandle()->screen(); const auto available=screen->availableGeometry();
                    report["availableWidth"]=available.width(); report["availableHeight"]=available.height();
                    report["logicalDpi"]=screen->logicalDotsPerInch();
                    fitsScreen=available.contains(window->frameGeometry()); report["fitsScreen"]=fitsScreen;
                    auto run=window->findChild<QWidget*>("runButton");
                    controlsFit=run&&run->isVisible()&&window->rect().contains(QRect(run->mapTo(window,QPoint()),run->size()));
                    report["runButtonFits"]=controlsFit;
                }
                QFile f(QCoreApplication::applicationDirPath()+"/smoke-report.json"); if(f.open(QIODevice::WriteOnly)) f.write(QJsonDocument(report).toJson());
                app.exit(result.status==fw::ResultStatus::Success&&*count==expected&&!result.outputs.isEmpty()&&fitsScreen&&*debugFits&&controlsFit?0:3);
            });
        });
        const auto inputName=parser.value("smoke-name");
        QTimer::singleShot(200,&app,[&runtime,&app,algorithm,inputName] {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            for(int x=-30;x<=30;++x) for(int y=-30;y<=30;++y) cloud.push_back(pcl::PointXYZ(x*0.025f,y*0.025f,float((x*y)%3)*0.0001f));
            auto layer=fw::DebugLayer::result(inputName,cloud); layer.data.unit="m"; layer.data.name=inputName;
            fw::AlgorithmInput input; input.clouds={layer.data}; QString e;
            if(!runtime.start(algorithm,input,fw::Parameters::defaults(runtime.schema(algorithm)),true,e)) { qCritical("%s",qPrintable(e)); app.exit(4); }
        });
        QTimer::singleShot(45000,&app,[&app,&runtime] { runtime.stop(); app.exit(5); });
    }
    const int result=app.exec(); runtime.shutdown(); if(window) delete window.data(); return result;
}
