#include "Runtime.h"
#include "DebugController.h"
#include "TrialSession.h"
#include <fw/Parameters.h>
#include <fw/Diagnostics.h>
#include <QtTest>
#include <QFile>
#include <QJsonDocument>
#include <QTemporaryDir>
#include <QDirIterator>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QListWidget>
#include <QTimer>
#include <QAction>
#include <QPushButton>
#include <QLabel>
#include <QToolButton>
#include <QTabWidget>
#include <pcl/point_types.h>
#include <cstring>
#include <future>

using namespace fw;
class FrameworkTests final : public QObject {
    Q_OBJECT
    QJsonObject schema_;
    CloudData cloud() {
        pcl::PointCloud<pcl::PointXYZ> points;
        for(int x=-20;x<=20;++x) for(int y=-20;y<=20;++y) points.push_back(pcl::PointXYZ(float(x)*0.03f,float(y)*0.03f,0.f));
        return DebugLayer::result("test",points).data;
    }
private slots:
    void initTestCase() {
        QFile f(SAMPLE_SCHEMA); QVERIFY(f.open(QIODevice::ReadOnly)); schema_=QJsonDocument::fromJson(f.readAll()).object();
        QVERIFY2(Parameters::validateSchema(schema_).isEmpty(),qPrintable(Parameters::validateSchema(schema_).join('\n')));
    }
    void parametersAndPresets() {
        auto values=Parameters::defaults(schema_); QVERIFY(Parameters::validate(schema_,values).isEmpty());
        values["limits"]=QJsonObject{{"lower",2},{"upper",1}}; QVERIFY(!Parameters::validate(schema_,values).isEmpty());
        values=Parameters::defaults(schema_); values["axis"]="invalid"; QVERIFY(!Parameters::validate(schema_,values).isEmpty());
        values["axis"]="z"; values["mean_k"]=2.5; QVERIFY(!Parameters::validate(schema_,values).isEmpty());
        QStringList warnings; auto merged=Parameters::merge(schema_,{{"name","preset"},{"unknown",1}},&warnings);
        QCOMPARE(merged["name"].toString(),QString("preset")); QVERIFY(merged.contains("leaf_size")); QCOMPARE(warnings.size(),1);
        auto groups=schema_["groups"].toArray(); auto g=groups[0].toObject(); auto parameters=g["parameters"].toArray();
        auto a=parameters[0].toObject(), b=parameters[1].toObject();
        a["enabledWhen"]=QJsonObject{{"parameter","axis"},{"operator","eq"},{"value","z"}};
        b["visibleWhen"]=QJsonObject{{"parameter","name"},{"operator","eq"},{"value","x"}};
        parameters[0]=a; parameters[1]=b; g["parameters"]=parameters; groups[0]=g; auto cyclic=schema_; cyclic["groups"]=groups;
        QVERIFY(!Parameters::validateSchema(cyclic).isEmpty());
        a["enabledWhen"]=QJsonObject{{"parameter","missing"},{"operator","eq"},{"value",1}}; parameters[0]=a; g["parameters"]=parameters; groups[0]=g; cyclic["groups"]=groups;
        QVERIFY(!Parameters::validateSchema(cyclic).isEmpty());
        QVERIFY(Parameters::condition(QJsonObject{{"all",QJsonArray{QJsonObject{{"parameter","axis"},{"operator","eq"},{"value","z"}},QJsonObject{{"parameter","mean_k"},{"operator","ge"},{"value",10}}}}},Parameters::defaults(schema_)));
    }
    void snapshotIsolationAndBudget() {
        DebugFrame frame; auto data=cloud(); frame.layers={DebugLayer::result("test",data)};
        auto frozen=snapshot(frame,512ull*1024*1024); const auto before=frozen->layers[0].data.cloud->data;
        auto mutableCloud=pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(*data.cloud)); frame.layers[0].data.cloud=mutableCloud;
        frozen=snapshot(frame,512ull*1024*1024); mutableCloud->data[0]^=0xff;
        QVERIFY(frozen->layers[0].data.cloud->data==before);
        auto small=snapshot(frame,1024); QVERIFY(small->sampled); QVERIFY(frameBytes(*small)<=1024); QVERIFY(frameBytes(frame)>1024);
        DebugLayer image; image.image=cv::Mat(100,100,CV_8UC3,cv::Scalar(7,8,9)); frame.layers={image}; frozen=snapshot(frame,40000); image.image.setTo(cv::Scalar(0));
        QCOMPARE(int(frozen->layers[0].image.at<cv::Vec3b>(0,0)[0]),7);
    }
    void allParameterWidgets() {
        auto schema=QJsonDocument::fromJson(R"({"schemaVersion":1,"parameterVersion":1,"algorithmId":"test.widgets","groups":[{"id":"g","parameters":[
          {"id":"bool","type":"bool","default":true},
          {"id":"int","type":"int","default":12},
          {"id":"double","type":"double","default":0.25,"decimals":3},
          {"id":"text","type":"string","default":"text"},
          {"id":"multiline","type":"string","default":"two\nlines","multiline":true},
          {"id":"enum","type":"enum","default":2,"options":[{"value":1,"label":"one"},{"value":2,"label":"two"}]},
          {"id":"range","type":"range","valueType":"int","default":{"lower":1,"upper":4}},
          {"id":"file","type":"file","default":""},
          {"id":"directory","type":"directory","default":""},
          {"id":"vector","type":"vector3","default":[1.25,2.5,3.75]},
          {"id":"color","type":"color","default":[1,2,3,4]}
        ]}]})").object();
        QVERIFY2(Parameters::validateSchema(schema).isEmpty(),qPrintable(Parameters::validateSchema(schema).join('\n')));
        Runtime runtime; QString error; QVERIFY(runtime.initialize(QCoreApplication::applicationDirPath()+"/plugins",error));
        auto editor=runtime.parametersUi()->createEditor(schema,nullptr); QVERIFY(editor); QCOMPARE(editor->values(),Parameters::defaults(schema));
        QVERIFY(editor->errors().isEmpty()); auto changed=editor->values(); changed["vector"]=QJsonArray{4.0,5.0,6.0}; changed["color"]=QJsonArray{12,34,56,128}; editor->setValues(changed); QCOMPARE(editor->values(),changed);
        auto numeric=editor->widget()->findChild<QDoubleSpinBox*>("double"); QVERIFY(numeric);
        const auto before=numeric->value();
        QWheelEvent wheel(QPointF(10,10),QPointF(numeric->mapToGlobal(QPoint(10,10))),QPoint(),QPoint(0,120),Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
        QApplication::sendEvent(numeric,&wheel); QCOMPARE(numeric->value(),before);
        QTest::keyClick(numeric,Qt::Key_Up); QVERIFY(numeric->value()>before);
        QCOMPARE(numeric->accessibleName(),QString("double"));
        runtime.parametersUi()->destroyEditor(editor);
    }
    void debugControls() {
        DebugController debug; debug.reset(true,1,1024*1024); DebugFrame frame; frame.stepId="step";
        int count=0;
        // Deliberately acknowledge and click Next before checkpoint starts waiting.
        debug.onFrame=[&](FramePtr f) { ++count; debug.next(); debug.presented(f->sequence); };
        auto worker=std::async(std::launch::async,[&] { for(int i=0;i<100;++i) if(debug.checkpoint(frame)!=DebugAction::Continue) return false; return true; });
        const bool ready=worker.wait_for(std::chrono::seconds(3))==std::future_status::ready;
        if(!ready) debug.stop(); QVERIFY(ready); QVERIFY(worker.get()); QCOMPARE(count,100);
        debug.reset(true,2,1024); debug.onFrame=[&](FramePtr) { debug.stop(); };
        QCOMPARE(debug.checkpoint(frame),DebugAction::Stop);
        debug.reset(false,3,1024); count=0; debug.onFrame=[&](FramePtr) {++count;};
        QCOMPARE(debug.checkpoint(frame),DebugAction::Continue); QCOMPARE(count,0);
        debug.reset(true,4,1024); debug.resume(); debug.onFrame=[&](FramePtr f) {debug.presented(f->sequence);};
        QCOMPARE(debug.checkpoint(frame),DebugAction::Continue);
    }
    void runtimeAndParameterEditor() {
        Runtime runtime; QString error;
        QVERIFY2(runtime.initialize(QCoreApplication::applicationDirPath()+"/plugins",error),qPrintable(error));
        QVERIFY(!runtime.algorithms().isEmpty());
        auto editor=runtime.parametersUi()->createEditor(schema_,nullptr); QVERIFY(editor);
        QCOMPARE(editor->values(),Parameters::defaults(schema_));
        auto enabled=editor->widget()->findChild<QCheckBox*>("enable_downsample"); QVERIFY(enabled);
        QVERIFY(!enabled->parentWidget()->isHidden()); QVERIFY(enabled->isEnabled());
        auto leaf=editor->widget()->findChild<QDoubleSpinBox*>("leaf_size"); QVERIFY(leaf); enabled->setChecked(false); QVERIFY(!leaf->isEnabled());
        runtime.parametersUi()->destroyEditor(editor);
        QTemporaryDir dir; QVERIFY(dir.isValid()); const auto path=dir.filePath("preset.json");
        auto values=Parameters::defaults(schema_); QVERIFY(runtime.savePreset("cloud.sample_pipeline",path,values,error));
        ParameterSet loaded; QStringList messages; QVERIFY(runtime.loadPreset("cloud.sample_pipeline",path,loaded,messages)); QCOMPARE(values,loaded);
        QFile file(path); QVERIFY(file.open(QIODevice::WriteOnly)); file.write("{\"algorithmId\":\"cloud.sample_pipeline\",\"parameterVersion\":99,\"values\":{}}"); file.close();
        QVERIFY(!runtime.loadPreset("cloud.sample_pipeline",path,loaded,messages));
        AlgorithmInput input; input.clouds={cloud()}; QSignalSpy finished(&runtime,&Host::finished);
        QVERIFY(runtime.start("cloud.sample_pipeline",input,values,false,error)); QVERIFY(!runtime.start("cloud.sample_pipeline",input,values,false,error));
        QVERIFY(finished.wait(15000)); const auto result=qvariant_cast<AlgorithmResult>(finished.at(0).at(0)); QCOMPARE(result.status,ResultStatus::Success); QCOMPARE(result.outputs.size(),1);
        const auto output=result.outputs.front(); QVERIFY(runtime.writeCloud(dir.filePath("result.pcd"),output,error));
        const auto read=runtime.readCloud(dir.filePath("result.pcd"),"m",error); QVERIFY(read.cloud); QCOMPARE(read.cloud->width,output.cloud->width);
        // Debug mode must preserve the numerical result, and evict history in order.
        runtime.setDebugLimits(1,1024*1024); QSignalSpy evicted(&runtime,&Host::debugEvicted); int debugFrames=0; quint64 last=0;
        connect(&runtime,&Host::debugReady,&runtime,[&](FramePtr f) {++debugFrames; last=f->sequence; runtime.presented(f->sequence); runtime.next();});
        finished.clear(); QVERIFY(runtime.start("cloud.sample_pipeline",input,values,true,error)); QVERIFY(finished.wait(15000));
        auto debugResult=qvariant_cast<AlgorithmResult>(finished.at(0).at(0)); QCOMPARE(debugResult.status,ResultStatus::Success);
        QVERIFY(debugResult.outputs.front().cloud->data==result.outputs.front().cloud->data); QCOMPARE(debugFrames,3); QCOMPARE(evicted.count(),2); QVERIFY(runtime.history(last)); QVERIFY(!runtime.history(last-1));
    }
    void textCloudImport() {
        QTemporaryDir dir; QVERIFY(dir.isValid());
        const auto path=dir.filePath("points.txt");
        QFile file(path); QVERIFY(file.open(QIODevice::WriteOnly));
        file.write("\xEF\xBB\xBFx,y,z,intensity\n# ordered input\n1,2,3,10\n4.5\t5.5 6.5 20 // extra columns are ignored\n"); file.close();

        Runtime runtime; QString error=QStringLiteral("stale error");
        const auto data=runtime.readCloud(path,"mm",error);
        QVERIFY2(data.cloud,qPrintable(error)); QVERIFY(error.isEmpty()); QCOMPARE(data.name,QStringLiteral("points.txt")); QCOMPARE(data.unit,QStringLiteral("mm"));
        pcl::PointCloud<pcl::PointXYZ> points; pcl::fromPCLPointCloud2(*data.cloud,points);
        QCOMPARE(points.size(),std::size_t(2)); QCOMPARE(points[0].x,1.0f); QCOMPARE(points[0].y,2.0f); QCOMPARE(points[0].z,3.0f);
        QCOMPARE(points[1].x,4.5f); QCOMPARE(points[1].y,5.5f); QCOMPARE(points[1].z,6.5f);

        const auto framedPath=dir.filePath("framed.txt");
        QFile framed(framedPath); QVERIFY(framed.open(QIODevice::WriteOnly));
        framed.write("index x y z target\n1 1 2 3 7\n2 4 5 6 7\n3 7 8 9 8\n"); framed.close();
        const auto framedData=runtime.readCloud(framedPath,"mm",error);
        QVERIFY2(framedData.cloud,qPrintable(error));
        const auto field=std::find_if(framedData.cloud->fields.begin(),framedData.cloud->fields.end(),
            [](const pcl::PCLPointField& value) {return value.name=="frame_index";});
        QVERIFY(field!=framedData.cloud->fields.end()); QCOMPARE(field->datatype,std::uint8_t(pcl::PCLPointField::INT32));
        std::int32_t frameId=0; std::memcpy(&frameId,framedData.cloud->data.data()+field->offset,4); QCOMPARE(frameId,std::int32_t(7));
        std::memcpy(&frameId,framedData.cloud->data.data()+2*framedData.cloud->point_step+field->offset,4); QCOMPARE(frameId,std::int32_t(8));

        const auto invalidPath=dir.filePath("invalid.txt");
        QFile invalid(invalidPath); QVERIFY(invalid.open(QIODevice::WriteOnly)); invalid.write("1 2 3\n4 bad 6\n"); invalid.close();
        const auto rejected=runtime.readCloud(invalidPath,"unknown",error);
        QVERIFY(!rejected.cloud); QVERIFY(error.contains(QStringLiteral("第 2 行")));

        const auto invalidFramePath=dir.filePath("invalid-frame.txt");
        QFile invalidFrame(invalidFramePath); QVERIFY(invalidFrame.open(QIODevice::WriteOnly));
        invalidFrame.write("index x y z target\n1 1 2 3 2\n2 4 5 6 1\n"); invalidFrame.close();
        const auto rejectedFrame=runtime.readCloud(invalidFramePath,"mm",error);
        QVERIFY(!rejectedFrame.cloud); QVERIFY(error.contains(QStringLiteral("帧号")));
    }
    void diagnosticProtocolAndTrialPlanning() {
        for(const char* path:{PROFILE_JSON_SCHEMA,REPORT_JSON_SCHEMA}) {QFile file(QString::fromUtf8(path)); QVERIFY(file.open(QIODevice::ReadOnly)); QJsonParseError parse; auto doc=QJsonDocument::fromJson(file.readAll(),&parse); QCOMPARE(parse.error,QJsonParseError::NoError); QVERIFY(doc.isObject());}
        QFile carFile(CAR_SCHEMA); QVERIFY(carFile.open(QIODevice::ReadOnly)); const auto carSchema=QJsonDocument::fromJson(carFile.readAll()).object();
        QVERIFY2(Parameters::validateSchema(carSchema).isEmpty(),qPrintable(Parameters::validateSchema(carSchema).join(';')));
        const AlgorithmDescriptor carDescriptor{"custom.carExtractFeature","CAR","1.0.0",{"x","y","z"}};
        QVERIFY2(Diagnostics::validateProfile(carSchema.value("diagnostics").toObject(),carSchema,carDescriptor).isEmpty(),qPrintable(Diagnostics::validateProfile(carSchema.value("diagnostics").toObject(),carSchema,carDescriptor).join(';')));
        const auto schema=QJsonDocument::fromJson(R"({"schemaVersion":1,"parameterVersion":1,"algorithmId":"tests.diagnostic","groups":[{"id":"g","parameters":[{"id":"threshold","type":"double","default":1.0,"min":0.5,"max":2.0,"step":0.1,"decimals":2}]}]})").object();
        const AlgorithmDescriptor descriptor{"tests.diagnostic","fixture","1.0",{"x","y","z"}};
        const auto profile=QJsonDocument::fromJson(R"({"protocolVersion":1,"algorithmId":"tests.diagnostic","algorithmVersion":"1.0","orderedInput":false,"stages":[{"id":"detect","label":"detect"}],"outputs":[{"id":"left","targetId":"left","role":"measured","geometry":"points"}],"codes":[{"id":"weak","stage":"detect","defaultSeverity":"warning","label":"weak"}],"trialParameters":[{"id":"threshold","stages":["detect"],"step":0.1,"maySuppressPhysicalDefect":false,"expectedChange":"changed","risk":"false positive","cannotFix":"missing data"}]})").object();
        QVERIFY2(Diagnostics::validateProfile(profile,schema,descriptor).isEmpty(),qPrintable(Diagnostics::validateProfile(profile,schema,descriptor).join(';')));
        auto invalidProfile=profile; invalidProfile["protocolVersion"]=2; QVERIFY(!Diagnostics::validateProfile(invalidProfile,schema,descriptor).isEmpty());
        AlgorithmInput input; input.clouds={cloud()}; const auto params=Parameters::defaults(schema);
        const auto count=double(input.clouds.front().cloud->width)*input.clouds.front().cloud->height;
        const QJsonObject issue{{"code","weak"},{"stage","detect"},{"severity","warning"},{"visibility","unknown"},{"origin","undetermined"},
            {"region",QJsonObject{{"kind","section_range"},{"first",1},{"last",1},{"sourceIndex",0},{"representative",QJsonArray{0,0,0}}}},
            {"scope",QJsonObject{{"targetId","left"},{"outputRole","measured"}}},{"summary","weak evidence"},
            {"evidence",QJsonObject{{"support",1},{"supportUnit","candidates"},{"basis","rejected_candidates"}}}};
        const QJsonObject report{{"protocolVersion",1},{"algorithmId",descriptor.id},{"algorithmVersion",descriptor.version},
            {"input",QJsonObject{{"cloudId",input.clouds.front().id},{"pointCount",count},{"coordinateFrame",input.clouds.front().coordinateFrame},{"unit",input.clouds.front().unit}}},
            {"parameters",params},{"outputs",QJsonArray{QJsonObject{{"id","left"},{"cloudIndex",0}}}},{"issues",QJsonArray{issue}}};
        AlgorithmResult validationResult; validationResult.outputs={input.clouds.front()};
        QVERIFY2(Diagnostics::validateReport(report,profile,descriptor,input,params,validationResult).isEmpty(),qPrintable(Diagnostics::validateReport(report,profile,descriptor,input,params,validationResult).join(';')));
        auto bad=report; bad["protocolVersion"]=2; QVERIFY(!Diagnostics::validateReport(bad,profile,descriptor,input,params,validationResult).isEmpty());
        bad=report; auto issues=bad["issues"].toArray(); auto changed=issues[0].toObject(); changed["code"]="unknown"; issues[0]=changed; bad["issues"]=issues; QVERIFY(!Diagnostics::validateReport(bad,profile,descriptor,input,params,validationResult).isEmpty());
        bad=report; issues=bad["issues"].toArray(); changed=issues[0].toObject(); auto evidence=changed["evidence"].toObject(); evidence["observed"]=QJsonValue(); changed["evidence"]=evidence; issues[0]=changed; bad["issues"]=issues; QVERIFY(!Diagnostics::validateReport(bad,profile,descriptor,input,params,validationResult).isEmpty());
        bad=report; bad.remove("parameters"); QVERIFY(!Diagnostics::validateReport(bad,profile,descriptor,input,params,validationResult).isEmpty());
        bad=report; issues=bad.value("issues").toArray(); changed=issues[0].toObject(); auto scope=changed.value("scope").toObject(); scope["targetId"]="right"; changed["scope"]=scope; issues[0]=changed; bad["issues"]=issues;
        QVERIFY(!Diagnostics::validateReport(bad,profile,descriptor,input,params,validationResult).isEmpty());
        const auto variants=TrialSession::plan(schema,profile,report,params); QCOMPARE(variants.size(),2); QCOMPARE(variants[0].values.value("threshold").toDouble(),0.9);
        auto noData=report; issues=noData["issues"].toArray(); changed=issues[0].toObject(); changed["visibility"]="missing_input"; issues[0]=changed; noData["issues"]=issues;
        QVERIFY(TrialSession::plan(schema,profile,noData,params).isEmpty());
        changed["visibility"]="observable"; changed["origin"]="physical_defect_suspected"; issues[0]=changed; noData["issues"]=issues;
        QVERIFY(TrialSession::plan(schema,profile,noData,params).isEmpty());
        auto trial=report; trial["issues"]=QJsonArray{}; const auto comparison=TrialSession::compare(report,trial);
        QVERIFY(comparison.value("comparable").toBool()); QCOMPARE(comparison.value("disappearedIssues").toArray().size(),1);
        QVERIFY(comparison.value("reviewRequired").toBool());
        QVERIFY(!TrialSession::oneClickAdoptionAllowed(profile,comparison,"threshold"));
        QVERIFY(TrialSession::oneClickAdoptionAllowed(profile,TrialSession::compare(report,report),"threshold"));
        auto riskProfile=profile; auto rules=riskProfile.value("trialParameters").toArray(); auto riskRule=rules[0].toObject(); riskRule["maySuppressPhysicalDefect"]=true; rules[0]=riskRule; riskProfile["trialParameters"]=rules;
        QVERIFY(!TrialSession::oneClickAdoptionAllowed(riskProfile,TrialSession::compare(report,report),"threshold"));
        auto outputs=trial.value("outputs").toArray(); auto changedOutput=outputs[0].toObject(); changedOutput["statistics"]=QJsonObject{{"sampleCount",2},{"segmentCount",2},{"interpolatedCount",0}}; outputs[0]=changedOutput; trial["outputs"]=outputs;
        QCOMPARE(TrialSession::compare(report,trial).value("outputStatisticChanges").toArray().size(),1);
        auto physical=report; issues=physical.value("issues").toArray(); changed=issues[0].toObject(); changed["origin"]="physical_defect_suspected"; issues[0]=changed; physical["issues"]=issues;
        QVERIFY(TrialSession::compare(physical,trial).value("physicalDefectRequiresReview").toBool());
        auto differentTarget=report; issues=differentTarget.value("issues").toArray(); changed=issues[0].toObject(); scope=changed.value("scope").toObject(); scope["targetId"]="right"; changed["scope"]=scope; issues[0]=changed; differentTarget["issues"]=issues;
        const auto identityComparison=TrialSession::compare(report,differentTarget);
        QCOMPARE(identityComparison.value("disappearedIssues").toArray().size(),1);
        QCOMPARE(identityComparison.value("newIssues").toArray().size(),1);
        trial["input"]=QJsonObject{{"cloudId","different"}}; QVERIFY(!TrialSession::compare(report,trial).value("comparable").toBool());
    }
    void trialIsolationCancellationAndDiagnosticDisplay() {
        QDir().mkpath(TEST_ARTIFACTS_PATH);
        QTemporaryDir dir(QStringLiteral(TEST_ARTIFACTS_PATH)+"/trial-XXXXXX"); dir.setAutoRemove(false);
        QDirIterator files(QCoreApplication::applicationDirPath()+"/plugins/ui",QDir::Files);
        while(files.hasNext()) {auto path=files.next(); if(QLibrary::isLibrary(path)) QVERIFY(QFile::copy(path,dir.filePath(QFileInfo(path).fileName())));}
        QVERIFY(QFile::copy(TEST_PLUGIN_PATH,dir.filePath("TestAlgorithm.dll")));
        Runtime runtime; QString error; QVERIFY2(runtime.initialize(dir.path(),error),qPrintable(error));
        QVERIFY(!runtime.schema("tests.stress").contains("diagnostics")); // Legacy plugin stays usable.
        auto window=runtime.createWindow(); QVERIFY(window);
        auto list=window->findChild<QListWidget*>("diagnosticsList"); QVERIFY(list);
        auto banner=window->findChild<QLabel*>("diagnosticBanner"); QVERIFY(banner);
        auto details=window->findChild<QToolButton*>("diagnosticDetailsButton"); QVERIFY(details);
        auto advanced=window->findChild<QCheckBox*>("advancedMode"); QVERIFY(advanced); QVERIFY(!advanced->isChecked());
        auto debugControls=window->findChild<QWidget*>("debugControls"); QVERIFY(debugControls); QVERIFY(debugControls->isHidden());
        auto debugControlsAction=window->findChild<QAction*>("debugControlsAction"); QVERIFY(debugControlsAction); QVERIFY(!debugControlsAction->isVisible());
        auto noticeOption=window->findChild<QCheckBox*>("enableDiagnosticBanner"); QVERIFY(noticeOption); QVERIFY(!noticeOption->isChecked()); QVERIFY(noticeOption->isHidden());
        QCOMPARE(banner->property("state").toString(),QStringLiteral("idle")); QVERIFY(banner->toolTip().contains(QStringLiteral("拖动旋转"))); QVERIFY(details->isHidden());
        auto datasets=window->findChild<QListWidget*>("datasets"); QVERIFY(datasets); QCOMPARE(datasets->count(),0);
        AlgorithmInput input; input.clouds={cloud()}; auto values=Parameters::defaults(runtime.schema("tests.stress")); values["count"]=1;
        QSignalSpy normal(&runtime,&Host::finished), trial(&runtime,&Runtime::trialFinished);
        QVERIFY(runtime.start("tests.stress",input,values,false,error)); QVERIFY(normal.wait(10000));
        QCOMPARE(datasets->count(),1);
        QCOMPARE(list->count(),1); QVERIFY(list->item(0)->text().contains(QStringLiteral("未提供结构化诊断")));
        QVERIFY(debugControls->isHidden()); QVERIFY(!debugControlsAction->isVisible());
        QCOMPARE(banner->property("state").toString(),QStringLiteral("idle")); QVERIFY(details->isHidden());
        advanced->setChecked(true);
        QVERIFY(!debugControls->isHidden()); QVERIFY(debugControlsAction->isVisible());
        QCOMPARE(banner->property("state").toString(),QStringLiteral("idle")); QVERIFY(noticeOption->isEnabled()); QVERIFY(!noticeOption->isHidden());
        noticeOption->setChecked(true); QCOMPARE(banner->property("state").toString(),QStringLiteral("idle"));
        auto runAction=window->findChild<QAction*>("runAction"); QVERIFY(runAction); QVERIFY(runAction->isEnabled());
        normal.clear(); runAction->trigger(); QVERIFY(normal.wait(10000));
        QCOMPARE(banner->property("state").toString(),QStringLiteral("info")); QVERIFY(banner->toolTip().contains(QStringLiteral("未提供结构化诊断")));
        const int formalCount=datasets->count();
        normal.clear(); QVERIFY(runtime.startTrial("tests.stress",input,values,error)); QVERIFY(trial.wait(10000)); QCOMPARE(normal.count(),0);
        QCOMPARE(datasets->count(),formalCount); // Trial completion cannot append a formal output.
        QCOMPARE(qvariant_cast<AlgorithmResult>(trial.at(0).at(0)).status,ResultStatus::Success);
        trial.clear(); values["count"]=1000; values["slow"]=true;
        QVERIFY(runtime.startTrial("tests.stress",input,values,error)); QTimer::singleShot(10,&runtime,&Runtime::stop);
        QVERIFY(trial.wait(10000)); QCOMPARE(normal.count(),0);
        QCOMPARE(qvariant_cast<AlgorithmResult>(trial.at(0).at(0)).status,ResultStatus::Cancelled);
        AlgorithmResult displayFixture; displayFixture.metrics["diagnostics"]=QJsonObject{{"issues",QJsonArray{QJsonObject{{"severity","warning"},{"stage","detect"},{"origin","physical_defect_suspected"},{"scope",QJsonObject{{"targetId","left"}}},{"region",QJsonObject{{"kind","whole"}}},{"summary","needs review"}}}}};
        runtime.finished(displayFixture); QCOMPARE(list->count(),1); QVERIFY(list->item(0)->text().contains(QStringLiteral("疑似产品瑕疵")));
        QCOMPARE(banner->property("state").toString(),QStringLiteral("warning")); QVERIFY(banner->toolTip().contains(QStringLiteral("needs review")));
        auto issues=displayFixture.metrics.value("diagnostics").toObject().value("issues").toArray();
        auto critical=issues.at(0).toObject(); critical["severity"]="critical"; critical["summary"]="critical issue"; issues.append(critical);
        auto info=critical; info["severity"]="info"; info["summary"]="ordinary hint"; issues.append(info);
        displayFixture.metrics["diagnostics"]=QJsonObject{{"issues",issues},{"parameters",Parameters::defaults(runtime.schema("tests.stress"))}}; runtime.finished(displayFixture);
        QCOMPARE(banner->property("state").toString(),QStringLiteral("critical")); QVERIFY(banner->toolTip().contains(QStringLiteral("critical issue")));
        QVERIFY(!banner->toolTip().contains(QStringLiteral("ordinary hint"))); QVERIFY(details->text().contains(QStringLiteral("3")));
        details->click(); auto tabs=window->findChild<QTabWidget*>("executionTabs"); QVERIFY(tabs); QCOMPARE(tabs->currentIndex(),2);
        advanced->setChecked(false); QCOMPARE(banner->property("state").toString(),QStringLiteral("idle")); QVERIFY(details->isHidden()); QVERIFY(debugControls->isHidden()); QVERIFY(!debugControlsAction->isVisible());
        advanced->setChecked(true); QCOMPARE(banner->property("state").toString(),QStringLiteral("idle")); QVERIFY(details->isHidden());
        normal.clear(); runAction->trigger(); QVERIFY(normal.wait(10000)); runtime.finished(displayFixture);
        QCOMPARE(banner->property("state").toString(),QStringLiteral("critical")); QVERIFY(banner->toolTip().contains(QStringLiteral("critical issue")));
        auto countEditor=window->findChild<QSpinBox*>("count"); QVERIFY(countEditor); countEditor->setValue(countEditor->value()+1);
        runtime.finished(displayFixture); QCOMPARE(banner->property("state").toString(),QStringLiteral("warning")); QVERIFY(banner->toolTip().contains(QStringLiteral("参数已修改")));
        QVERIFY(details->isHidden()); QCOMPARE(list->count(),3); // The detailed diagnosis remains available.
        AlgorithmResult invalid; invalid.metrics["diagnostics_validation"]="invalid report"; runtime.finished(invalid);
        QCOMPARE(banner->property("state").toString(),QStringLiteral("critical")); QVERIFY(banner->toolTip().contains(QStringLiteral("invalid report")));
        AlgorithmResult cancelled; cancelled.status=ResultStatus::Cancelled; runtime.finished(cancelled);
        QCOMPARE(banner->property("state").toString(),QStringLiteral("info")); QVERIFY(banner->toolTip().contains(QStringLiteral("已停止")));
        QVERIFY(!banner->toolTip().contains(QStringLiteral("critical issue")));
        advanced->setChecked(false); runtime.finished(displayFixture); datasets->setCurrentRow(-1); advanced->setChecked(true);
        QCOMPARE(banner->property("state").toString(),QStringLiteral("idle")); QVERIFY(banner->toolTip().contains(QStringLiteral("拖动旋转")));
        QVERIFY(details->isHidden()); QCOMPARE(list->count(),0);
        delete window;
    }
    void carRuntimeDiagnosticIntegration() {
        const auto pluginPath=qEnvironmentVariable("CAR_PLUGIN_PATH"),samplePath=qEnvironmentVariable("CAR_SAMPLE_PATH");
        if(pluginPath.isEmpty()||samplePath.isEmpty()) QSKIP("Set CAR_PLUGIN_PATH and CAR_SAMPLE_PATH for the external CAR integration test");
        QTemporaryDir dir; QVERIFY(dir.isValid());
        QDirIterator files(QCoreApplication::applicationDirPath()+"/plugins/ui",QDir::Files);
        while(files.hasNext()) {const auto path=files.next(); if(QLibrary::isLibrary(path)) QVERIFY(QFile::copy(path,dir.filePath(QFileInfo(path).fileName())));}
        QVERIFY(QFile::copy(pluginPath,dir.filePath("CarExtractFeatureAlgorithm.dll")));
        Runtime runtime; QString error; QVERIFY2(runtime.initialize(dir.path(),error),qPrintable(error));
        const auto profile=runtime.schema("custom.carExtractFeature").value("diagnostics").toObject();
        QVERIFY2(!profile.isEmpty(),qPrintable(runtime.diagnostics().join(';')));
        auto inputCloud=runtime.readCloud(samplePath,"unknown",error); QVERIFY2(inputCloud.cloud,qPrintable(error));
        AlgorithmInput input; input.clouds={inputCloud};
        auto values=Parameters::defaults(runtime.schema("custom.carExtractFeature"));
        auto window=runtime.createWindow(); QVERIFY(window);
        auto list=window->findChild<QListWidget*>("diagnosticsList"); QVERIFY(list);
        QSignalSpy finished(&runtime,&Host::finished);
        QVERIFY2(runtime.start("custom.carExtractFeature",input,values,false,error),qPrintable(error));
        QVERIFY(finished.wait(60000));
        const auto result=qvariant_cast<AlgorithmResult>(finished.at(0).at(0));
        QCOMPARE(result.status,ResultStatus::Success);
        QVERIFY2(!result.metrics.contains("diagnostics_validation"),qPrintable(result.metrics.value("diagnostics_validation").toString()));
        const auto report=result.metrics.value("diagnostics").toObject(); QVERIFY(!report.isEmpty());
        QCOMPARE(report.value("outputs").toArray().size(),2);
        QCOMPARE(list->count(),report.value("issues").toArray().isEmpty()?1:report.value("issues").toArray().size());
        for(int i=0;i<list->count();++i) if(list->item(i)->text().contains(QStringLiteral("截面"))) {
            list->setCurrentRow(i); auto layers=window->findChild<QListWidget*>("layers"); QVERIFY(layers);
            bool highlighted=false; for(int j=0;j<layers->count();++j) if(layers->item(j)->text()==QStringLiteral("诊断位置")) highlighted=true;
            QVERIFY(highlighted); break;
        }
        auto datasets=window->findChild<QListWidget*>("datasets"); QVERIFY(datasets);
        AlgorithmResult importFixture; importFixture.outputs={inputCloud}; runtime.finished(importFixture);
        const int formalCount=datasets->count(); QCOMPARE(formalCount,3); datasets->setCurrentRow(formalCount-1);
        auto checkAction=window->findChild<QAction*>("autoCheckAction"); QVERIFY(checkAction);
        QSignalSpy trials(&runtime,&Runtime::trialFinished);
        checkAction->trigger(); QTRY_VERIFY_WITH_TIMEOUT(trials.count()>=1,30000);
        QCOMPARE(datasets->count(),formalCount); // UI auto-check baseline is never a formal output.
        const auto expected=TrialSession::plan(runtime.schema("custom.carExtractFeature"),profile,report,values);
        if(!expected.isEmpty()) {
            QTRY_VERIFY_WITH_TIMEOUT(trials.count()>=expected.size()+1,30000);
            auto trialList=window->findChild<QListWidget*>("trialList"); QVERIFY(trialList); QCOMPARE(trialList->count(),expected.size());
            auto adopt=window->findChild<QPushButton*>("adoptTrialButton"); QVERIFY(adopt);
            trialList->setCurrentRow(0); QVERIFY(!adopt->isEnabled()); // CAR trial parameters may suppress real defects.
        }
        if(runtime.busy()) {runtime.stop(); QTRY_VERIFY_WITH_TIMEOUT(!runtime.busy(),30000);}
        QCOMPARE(datasets->count(),formalCount);
        delete window;
    }
    void duplicatePlugins() {
        QDir().mkpath(TEST_ARTIFACTS_PATH);
        QTemporaryDir dir(QStringLiteral(TEST_ARTIFACTS_PATH)+"/duplicates-XXXXXX"); dir.setAutoRemove(false);
        const auto plugins=QCoreApplication::applicationDirPath()+"/plugins";
        QDirIterator files(plugins,QDir::Files,QDirIterator::Subdirectories); QString algorithm;
        while(files.hasNext()) { auto path=files.next(); if(!QLibrary::isLibrary(path)) continue; auto target=dir.filePath(QFileInfo(path).fileName()); QVERIFY(QFile::copy(path,target)); if(path.contains("SampleAlgorithm")) algorithm=target; }
        QVERIFY(!algorithm.isEmpty()); QVERIFY(QFile::copy(algorithm,dir.filePath("duplicate.dll")));
        Runtime runtime; QString error; QVERIFY2(runtime.initialize(dir.path(),error),qPrintable(error));
        bool duplicatedSampleLoaded=false;
        for(const auto& descriptor:runtime.algorithms())
            if(descriptor.id=="cloud.sample_pipeline") duplicatedSampleLoaded=true;
        QVERIFY(!duplicatedSampleLoaded); // 冲突的两个 Sample 副本都必须禁用；其他算法仍可加载。
        QVERIFY(!runtime.algorithms().isEmpty());
        QVERIFY(runtime.diagnostics().join(' ').contains(QStringLiteral("冲突")));
    }
    void stressHistoryCancellationAndFailure() {
        QDir().mkpath(TEST_ARTIFACTS_PATH);
        QTemporaryDir dir(QStringLiteral(TEST_ARTIFACTS_PATH)+"/stress-XXXXXX"); dir.setAutoRemove(false);
        QDirIterator files(QCoreApplication::applicationDirPath()+"/plugins/ui",QDir::Files);
        while(files.hasNext()) { auto path=files.next(); if(QLibrary::isLibrary(path)) QVERIFY(QFile::copy(path,dir.filePath(QFileInfo(path).fileName()))); }
        QVERIFY(QFile::copy(TEST_PLUGIN_PATH,dir.filePath("TestAlgorithm.dll")));
        Runtime runtime; QString error; QVERIFY2(runtime.initialize(dir.path(),error),qPrintable(error)); runtime.setDebugLimits(8,1024*1024);
        AlgorithmInput input; input.clouds={cloud()}; const auto values=Parameters::defaults(runtime.schema("tests.stress"));
        int frames=0; quint64 first=0,last=0; bool cancel=false;
        connect(&runtime,&Host::debugReady,&runtime,[&](FramePtr f) {
            ++frames; if(!first) first=f->sequence; last=f->sequence;
            if(cancel) runtime.stop(); else { runtime.presented(f->sequence); runtime.next(); }
        });
        QSignalSpy evicted(&runtime,&Host::debugEvicted), finished(&runtime,&Host::finished);
        QVERIFY(runtime.start("tests.stress",input,values,true,error)); QVERIFY(finished.wait(15000));
        QCOMPARE(frames,100); QCOMPARE(evicted.count(),92); QVERIFY(!runtime.history(first)); QVERIFY(runtime.history(last));
        QCOMPARE(qvariant_cast<AlgorithmResult>(finished.at(0).at(0)).status,ResultStatus::Success);
        cancel=true; finished.clear(); QVERIFY(runtime.start("tests.stress",input,values,true,error)); QVERIFY(finished.wait(5000));
        QCOMPARE(qvariant_cast<AlgorithmResult>(finished.at(0).at(0)).status,ResultStatus::Cancelled);
        auto failure=values; failure["fail"]=true; finished.clear(); QVERIFY(runtime.start("tests.stress",input,failure,false,error)); QVERIFY(finished.wait(5000));
        QCOMPARE(qvariant_cast<AlgorithmResult>(finished.at(0).at(0)).status,ResultStatus::Failed);
        AlgorithmInput empty; QVERIFY(!runtime.start("tests.stress",empty,values,false,error));
    }
};
QTEST_MAIN(FrameworkTests)
#include "FrameworkTests.moc"
