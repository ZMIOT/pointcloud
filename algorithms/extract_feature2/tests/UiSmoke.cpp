#include "Runtime.h"
#include "Theme.h"
#include <QVTKOpenGLNativeWidget.h>
#include <QSurfaceFormat>
#include <QApplication>
#include <QComboBox>
#include <QCheckBox>
#include <QLineEdit>
#include <QJsonDocument>
#include <QFile>
#include <QDir>
#include <QTimer>
#include <QPointer>
#include <QWindow>
#include <QScreen>

int main(int argc,char** argv) {
    if(argc!=5) return 2;
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#if QT_VERSION >= QT_VERSION_CHECK(5,14,0)
    QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);
#endif
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc,argv); app.setOrganizationName("PointCloudFrameworkTest"); app.setApplicationName("ExtractFeature2UiSmoke");
    workbench::initializeAppearance(app); workbench::applyTheme(app,false);
    const QString host=QString::fromLocal8Bit(argv[1]),inputFile=QString::fromLocal8Bit(argv[2]),preset=QString::fromLocal8Bit(argv[3]),output=QString::fromLocal8Bit(argv[4]);
    if(!QDir().mkpath(output)) return 2;
    fw::Runtime runtime; QString error;
    if(!runtime.initialize(QDir(host).filePath("plugins"),error)) {qCritical("%s",qPrintable(error)); return 3;}
    fw::ParameterSet parameters; QStringList messages;
    if(!runtime.loadPreset("weld.extract_feature2",preset,parameters,messages)) {qCritical("%s",qPrintable(messages.join('\n'))); return 4;}
    auto cloud=runtime.readCloud(inputFile,"mm",error); if(!error.isEmpty()) return 5;
    QPointer<QWidget> window=runtime.createWindow(); window->show();
    auto selector=window->findChild<QComboBox*>("algorithmSelector");
    if(!selector||selector->findData("weld.extract_feature2")<0) return 6;
    selector->setCurrentIndex(selector->findData("weld.extract_feature2"));
    window->findChild<QCheckBox*>("advancedMode")->setChecked(true);
    window->findChild<QCheckBox*>("enableDebug")->setChecked(true);
    int checkpoints=0; bool debugFits=true;
    QObject::connect(&runtime,&fw::Host::debugReady,&app,[&](fw::FramePtr) {++checkpoints;});
    QObject::connect(&runtime,&fw::Host::pausedChanged,&app,[&](bool paused) {
        if(paused) QTimer::singleShot(20,&app,[&] {
            for(auto widget:QApplication::topLevelWidgets()) if(widget->objectName()=="debugWindow") {
                widget->grab().save(QDir(output).filePath("debug.png"));
                debugFits&=widget->windowHandle()->screen()->availableGeometry().contains(widget->frameGeometry());
            }
            runtime.next();
        });
    });
    QObject::connect(&runtime,&fw::Host::finished,&app,[&](fw::AlgorithmResult result) {
        QTimer::singleShot(200,&app,[&,result] {
            window->grab().save(QDir(output).filePath("main.png"));
            auto report=result.metrics; report["checkpoints"]=checkpoints; report["outputs"]=result.outputs.size(); report["message"]=result.message;
            const bool fits=window->windowHandle()->screen()->availableGeometry().contains(window->frameGeometry());
            const bool success=result.status==fw::ResultStatus::Success&&checkpoints>=8&&result.outputs.size()==2&&fits&&debugFits;
            report["success"]=success; report["windowFits"]=fits; report["debugFits"]=debugFits;
            QFile file(QDir(output).filePath("report.json")); if(file.open(QIODevice::WriteOnly)) file.write(QJsonDocument(report).toJson());
            app.exit(success?0:7);
        });
    });
    QTimer::singleShot(200,&app,[&] {fw::AlgorithmInput input; input.clouds={cloud}; if(!runtime.start("weld.extract_feature2",input,parameters,true,error)) {qCritical("%s",qPrintable(error)); app.exit(8);}});
    QTimer::singleShot(600000,&app,[&] {runtime.stop(); app.exit(9);});
    int code=app.exec(); runtime.shutdown(); if(window) delete window.data(); return code;
}
