#include <fw/UiInterfaces.h>
#include <fw/Parameters.h>
#include "Runtime.h"
#include "TrialSession.h"
#include <QMainWindow>
#include <QCheckBox>
#include <QCloseEvent>
#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEvent>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QStatusBar>
#include <QTabWidget>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QApplication>
#include <QMenuBar>
#include <QMenu>
#include <QScreen>
#include <QSplitter>
#include <QStackedWidget>
#include <QToolButton>
#include <QStyle>
#include <QSignalBlocker>
#include <QLocale>
#include <QResizeEvent>
#include <QJsonDocument>
#include "Theme.h"
#include <algorithm>

namespace {
using namespace fw;
class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(Host& host):host_(host) {
        setWindowTitle(QStringLiteral("点云工作台")); setObjectName("mainWindow"); setAttribute(Qt::WA_DeleteOnClose);
        const auto available=QApplication::primaryScreen()->availableGeometry().size();
        setMinimumSize(std::min(860,available.width()-40),std::min(520,available.height()-50));
        resize(std::min(1440,available.width()-40),std::min(940,available.height()-50));
        setCorner(Qt::BottomLeftCorner,Qt::LeftDockWidgetArea); setCorner(Qt::BottomRightCorner,Qt::RightDockWidgetArea);
        setDockNestingEnabled(true);
        auto fileMenu=menuBar()->addMenu(QStringLiteral("文件(&F)"));
        viewMenu_=menuBar()->addMenu(QStringLiteral("视图(&V)"));
        auto executionMenu=menuBar()->addMenu(QStringLiteral("算法(&A)"));
        auto brand=new QLabel(QStringLiteral("◈  点云工作台")); brand->setProperty("role","accent"); brand->setContentsMargins(16,0,18,0);
        menuBar()->setCornerWidget(brand,Qt::TopLeftCorner);
        auto engine=new QLabel(QStringLiteral("<span style='color:#34D399'>●</span>  引擎就绪")); engine->setProperty("role","muted"); engine->setContentsMargins(12,0,16,0);
        menuBar()->setCornerWidget(engine,Qt::TopRightCorner);
        auto action=[this](const QString& text,QStyle::StandardPixmap icon,const QKeySequence& key,std::function<void()> callback) {
            auto result=new QAction(style()->standardIcon(icon),text,this); result->setShortcut(key); addAction(result);
            connect(result,&QAction::triggered,this,[callback] {callback();}); return result;
        };
        open_=action(QStringLiteral("导入点云"),QStyle::SP_DialogOpenButton,QKeySequence::Open,[this] {open();}); open_->setObjectName("openAction");
        save_=action(QStringLiteral("导出选中点云"),QStyle::SP_DialogSaveButton,QKeySequence("Ctrl+Shift+S"),[this] {save();});
        auto fit=action(QStringLiteral("适应视图"),QStyle::SP_TitleBarMaxButton,QKeySequence("Ctrl+0"),[this] {scene_->fit();});
        fit->setIcon(QIcon());
        run_=action(QStringLiteral("运行算法"),QStyle::SP_MediaPlay,QKeySequence(Qt::Key_F5),[this] {run();}); run_->setObjectName("runAction");
        autoCheck_=action(QStringLiteral("自动检查"),QStyle::SP_FileDialogDetailedView,QKeySequence("Ctrl+Shift+D"),[this] {autoCheck();}); autoCheck_->setObjectName("autoCheckAction");
        stop_=action(QStringLiteral("停止"),QStyle::SP_MediaStop,QKeySequence("Shift+F5"),[this] {trialCancelled_=true; host_.stop(); if(trialActive_&&!baselineReport_.isEmpty()) refreshDiagnosticNotice(QStringLiteral("正在停止试跑 · 显示基线诊断")); else setSceneNotice(QStringLiteral("正在停止 · 等待算法退出"),"info");});
        next_=action(QStringLiteral("下一步"),QStyle::SP_MediaSkipForward,QKeySequence(Qt::Key_F10),[this] {host_.next();});
        resume_=action(QStringLiteral("继续运行"),QStyle::SP_MediaPlay,QKeySequence(Qt::Key_F6),[this] {host_.continueRun();});
        step_=action(QStringLiteral("恢复单步"),QStyle::SP_MediaPause,{},[this] {host_.singleStep();});
        fileMenu->addActions({open_,save_}); fileMenu->addSeparator(); fileMenu->addAction(QStringLiteral("退出"),this,&QWidget::close,QKeySequence::Quit);
        executionMenu->addActions({run_,autoCheck_,stop_,next_,resume_,step_});
        auto bar=addToolBar(QStringLiteral("文件与视图")); bar->setObjectName("mainToolbar"); bar->setMovable(false); bar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        bar->setIconSize(QSize(16,16));
        bar->addActions({open_,save_}); bar->addSeparator(); bar->addAction(fit); bar->addAction(autoCheck_);
        if(auto button=qobject_cast<QToolButton*>(bar->widgetForAction(open_))) button->setProperty("role","primary");
        // 单步操作与文件操作共用一行，避免高级模式额外占掉一整条工具栏。
        debugGroup_=new QWidget; debugGroup_->setObjectName("debugControls");
        auto debugLayout=new QHBoxLayout(debugGroup_); debugLayout->setContentsMargins(8,0,0,0); debugLayout->setSpacing(5);
        debugging_=new QCheckBox(QStringLiteral("启用单步调试")); debugging_->setObjectName("enableDebug"); debugLayout->addWidget(debugging_);
        diagnosticNoticeOption_=new QCheckBox(QStringLiteral("顶部彩色诊断提示")); diagnosticNoticeOption_->setObjectName("enableDiagnosticBanner"); diagnosticNoticeOption_->setChecked(false);
        diagnosticNoticeOption_->setToolTip(QStringLiteral("勾选后在下一次运行算法或自动检查时，在点云视图顶部显示诊断提示")); debugLayout->addWidget(diagnosticNoticeOption_); diagnosticNoticeOption_->hide();
        for(auto debugAction:{next_,resume_,step_}) {
            auto button=new QToolButton; button->setDefaultAction(debugAction); button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon); button->setIconSize(QSize(14,14));
            debugLayout->addWidget(button);
        }
        auto resources=new QToolButton; resources->setText(QStringLiteral("资源设置…"));
        connect(resources,&QToolButton::clicked,this,[this] {debugSettings();}); debugLayout->addWidget(resources);
        debugGroupAction_=bar->addWidget(debugGroup_); debugGroupAction_->setObjectName("debugControlsAction"); debugGroupAction_->setVisible(false); debugGroup_->hide();
        auto spacer=new QWidget; spacer->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred); bar->addWidget(spacer);
        advanced_=new QCheckBox(QStringLiteral("高级模式")); advanced_->setObjectName("advancedMode"); advanced_->setChecked(false); bar->addWidget(advanced_);

        auto center=new QWidget; center->setProperty("role","viewport");
        auto centerLayout=new QGridLayout(center); centerLayout->setContentsMargins(0,0,0,0); centerLayout->setSpacing(0);
        sceneStack_=new QStackedWidget; sceneStack_->setObjectName("sceneStack"); centerLayout->addWidget(sceneStack_,0,0);
        // 画布信息贴合顶部边缘；细栏中的诊断入口可打开完整问题清单。
        auto viewHeader=new QWidget(center); viewHeader->setProperty("role","viewHeader");
        viewHeader->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
        auto headerLayout=new QHBoxLayout(viewHeader); headerLayout->setContentsMargins(12,5,12,5); headerLayout->setSpacing(6);
        auto heading=new QLabel(QStringLiteral("点云视图")); heading->setProperty("role","viewTitle"); headerLayout->addWidget(heading);
        auto divider=new QLabel(QStringLiteral("│")); divider->setProperty("role","muted"); headerLayout->addWidget(divider);
        sceneHint_=new QLabel(QStringLiteral("左键拖动旋转 · 右键拖动平移 · 滚轮缩放")); sceneHint_->setObjectName("diagnosticBanner"); sceneHint_->setProperty("role","diagnosticNotice"); sceneHint_->setProperty("state","idle");
        sceneHint_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred); sceneHint_->setMinimumWidth(0); sceneHint_->installEventFilter(this); headerLayout->addWidget(sceneHint_,1);
        diagnosticDetails_=new QToolButton; diagnosticDetails_->setObjectName("diagnosticDetailsButton"); diagnosticDetails_->setProperty("role","diagnosticDetails");
        diagnosticDetails_->setText(QStringLiteral("查看诊断")); diagnosticDetails_->setToolTip(QStringLiteral("打开执行记录中的诊断问题清单")); diagnosticDetails_->hide(); headerLayout->addWidget(diagnosticDetails_);
        centerLayout->addWidget(viewHeader,0,0,Qt::AlignTop);
        auto empty=new QWidget; empty->setProperty("role","viewport"); auto emptyLayout=new QVBoxLayout(empty);
        emptyLayout->setContentsMargins(24,24,24,24); emptyLayout->addStretch();
        auto emptyCard=new QWidget; emptyCard->setProperty("role","emptyCard"); emptyCard->setMaximumWidth(430); emptyCard->setMinimumHeight(330);
        auto cardLayout=new QVBoxLayout(emptyCard); cardLayout->setContentsMargins(32,28,32,28); cardLayout->setSpacing(14);
        auto emptyIcon=new QLabel; emptyIcon->setProperty("role","emptyIcon"); emptyIcon->setPixmap(QIcon(QStringLiteral(":/workbench/cloud-upload.svg")).pixmap(36,36));
        emptyIcon->setAlignment(Qt::AlignCenter); emptyIcon->setFixedSize(80,80); cardLayout->addWidget(emptyIcon,0,Qt::AlignHCenter); cardLayout->addSpacing(12);
        auto emptyTitle=new QLabel(QStringLiteral("从一份点云开始")); emptyTitle->setProperty("role","title"); emptyTitle->setAlignment(Qt::AlignCenter); cardLayout->addWidget(emptyTitle);
        auto emptyText=new QLabel(QStringLiteral("导入 PCD / PLY / TXT，选择算法并设置参数。\n结果会与原始点云叠加显示。")); emptyText->setProperty("role","muted"); emptyText->setWordWrap(true); emptyText->setAlignment(Qt::AlignCenter); cardLayout->addWidget(emptyText);
        auto import=new QPushButton(style()->standardIcon(QStyle::SP_DialogOpenButton),QStringLiteral("导入点云")); import->setProperty("role","primary");
        connect(import,&QPushButton::clicked,this,[this] {open();}); cardLayout->addWidget(import,0,Qt::AlignHCenter);
        emptyLayout->addWidget(emptyCard,0,Qt::AlignHCenter); emptyLayout->addStretch();
        sceneStack_->addWidget(empty); scene_=host.visualization()->createView(this); sceneStack_->addWidget(scene_->widget()); setCentralWidget(center);

        auto left=new QWidget; left->setProperty("role","surface"); auto leftLayout=new QVBoxLayout(left); leftLayout->setContentsMargins(12,10,12,10); leftLayout->setSpacing(7);
        auto inputRow=new QHBoxLayout; auto inputHeader=new QLabel(QStringLiteral("输入与结果")); inputHeader->setProperty("compactHide",true); inputHeader->setProperty("role","muted"); inputRow->addWidget(inputHeader);
        inputRow->addStretch(); selectionBadge_=new QLabel(QStringLiteral("0 选中")); selectionBadge_->setProperty("role","muted"); inputRow->addWidget(selectionBadge_); leftLayout->addLayout(inputRow);
        auto inputHelp=new QLabel(QStringLiteral("选中点云，作为下一次运行的输入。")); inputHelp->setWordWrap(true); inputHelp->setProperty("role","muted"); leftLayout->addWidget(inputHelp);
        inputHelp->setProperty("compactHide",true);
        datasetsStack_=new QStackedWidget; datasetsStack_->setObjectName("datasetsStack"); datasetsStack_->setProperty("role","emptyWell");
        auto noDataset=new QLabel(QStringLiteral("◇\n尚未选择输入")); noDataset->setProperty("role","muted"); noDataset->setAlignment(Qt::AlignCenter); datasetsStack_->addWidget(noDataset);
        datasets_=new QListWidget; datasets_->setTextElideMode(Qt::ElideMiddle); datasets_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); datasets_->setObjectName("datasets"); datasets_->setAccessibleName(QStringLiteral("输入与结果点云")); datasetsStack_->addWidget(datasets_); datasetsStack_->setMaximumHeight(120); leftLayout->addWidget(datasetsStack_);
        inputInfo_=new QLabel(QStringLiteral("尚未选择输入")); inputInfo_->setWordWrap(true); inputInfo_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred); inputInfo_->setProperty("role","muted"); leftLayout->addWidget(inputInfo_);
        inputInfo_->setProperty("compactHide",true); datasets_->setMinimumHeight(36);
        auto layerHeader=new QLabel(QStringLiteral("显示图层")); layerHeader->setProperty("role","muted"); leftLayout->addWidget(layerHeader);
        layersStack_=new QStackedWidget; layersStack_->setObjectName("layersStack"); layersStack_->setProperty("role","emptyWell");
        auto noLayers=new QLabel(QStringLiteral("暂无活动图层")); noLayers->setProperty("role","muted"); noLayers->setAlignment(Qt::AlignCenter); layersStack_->addWidget(noLayers);
        layers_=new QListWidget; layers_->setTextElideMode(Qt::ElideMiddle); layers_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); layers_->setObjectName("layers"); layers_->setAccessibleName(QStringLiteral("点云图层，空格切换显隐")); layersStack_->addWidget(layers_); layersStack_->setMaximumHeight(150); leftLayout->addWidget(layersStack_);
        layers_->setMinimumHeight(36);
        auto layerStyle=new QPushButton(QStringLiteral("图层样式…")); connect(layerStyle,&QPushButton::clicked,this,[this] {editStyle(layers_->currentRow());}); leftLayout->addWidget(layerStyle); leftLayout->addStretch();
        connect(layers_,&QListWidget::currentRowChanged,layerStyle,[layerStyle](int row) {layerStyle->setEnabled(row>=0);}); layerStyle->setEnabled(false);
        leftDock_=dock(QStringLiteral("数据与图层"),left,Qt::LeftDockWidgetArea); leftDock_->setMinimumWidth(170);

        auto parameters=new QWidget; parameters->setProperty("role","surface"); auto outer=new QVBoxLayout(parameters); outer->setContentsMargins(12,10,12,10); outer->setSpacing(7);
        auto algorithmLabel=new QLabel(QStringLiteral("处理算法")); algorithmLabel->setProperty("role","muted"); outer->addWidget(algorithmLabel);
        algorithmLabel->setProperty("compactHide",true);
        algorithms_=new QComboBox; algorithms_->setObjectName("algorithmSelector"); algorithms_->setAccessibleName(QStringLiteral("处理算法")); algorithms_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon); algorithms_->setMinimumContentsLength(12);
        for(const auto& a:host.algorithms()) algorithms_->addItem(a.name,a.id);
        outer->addWidget(algorithms_);
        auto presetRow=new QHBoxLayout; auto parameterTitle=new QLabel(QStringLiteral("参数配置")); parameterTitle->setProperty("role","muted"); presetRow->addWidget(parameterTitle); presetRow->addStretch();
        auto preset=new QToolButton; preset->setText(QStringLiteral("参数预设")); preset->setPopupMode(QToolButton::InstantPopup); auto presets=new QMenu(preset); preset->setMenu(presets);
        presets->addAction(QStringLiteral("恢复默认值"),this,[this] {if(editor_) editor_->setValues(Parameters::defaults(host_.schema(algorithmId())));});
        presets->addAction(QStringLiteral("加载预设…"),this,[this] {if(editor_) loadPreset();}); presets->addAction(QStringLiteral("保存预设…"),this,[this] {if(editor_) savePreset();}); presetRow->addWidget(preset); outer->addLayout(presetRow);
        parameterLayout_=new QVBoxLayout; parameterLayout_->setContentsMargins(0,0,0,0); outer->addLayout(parameterLayout_,1);
        auto runRow=new QHBoxLayout; auto runButton=new QToolButton; runButton->setObjectName("runButton"); runButton->setDefaultAction(run_); runButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon); runButton->setProperty("role","primary"); runButton->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
        auto stopButton=new QToolButton; stopButton->setDefaultAction(stop_); stopButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon); runRow->addWidget(runButton,1); runRow->addWidget(stopButton); outer->addLayout(runRow);
        parameterDock_=dock(QStringLiteral("算法与参数"),parameters,Qt::RightDockWidgetArea); parameterDock_->setMinimumWidth(300);

        executionTabs_=new QTabWidget; executionTabs_->setObjectName("executionTabs"); steps_=new QListWidget; steps_->setAccessibleName(QStringLiteral("调试历史，回车查看快照")); logs_=new QPlainTextEdit; logs_->setReadOnly(true); logs_->setMaximumBlockCount(3000); logs_->setAccessibleName(QStringLiteral("运行日志"));
        executionTabs_->addTab(steps_,QStringLiteral("调试步骤")); executionTabs_->addTab(logs_,QStringLiteral("日志与统计"));
        auto diagnosticsPanel=new QWidget; auto diagnosticsLayout=new QVBoxLayout(diagnosticsPanel); diagnosticsLayout->setContentsMargins(5,5,5,5);
        diagnosisSummary_=new QLabel(QStringLiteral("运行后显示结构化诊断")); diagnosisSummary_->setObjectName("diagnosisSummary"); diagnosticsLayout->addWidget(diagnosisSummary_);
        diagnosticsList_=new QListWidget; diagnosticsList_->setObjectName("diagnosticsList"); diagnosticsList_->setAccessibleName(QStringLiteral("自动诊断问题清单")); diagnosticsLayout->addWidget(diagnosticsList_);
        executionTabs_->addTab(diagnosticsPanel,QStringLiteral("诊断问题"));
        auto trialPanel=new QWidget; auto trialLayout=new QVBoxLayout(trialPanel); trialLayout->setContentsMargins(5,5,5,5);
        trialList_=new QListWidget; trialList_->setObjectName("trialList"); trialLayout->addWidget(trialList_);
        auto trialButtons=new QHBoxLayout; adoptTrial_=new QPushButton(QStringLiteral("采用所选试参")); restoreTrial_=new QPushButton(QStringLiteral("恢复试参前参数"));
        adoptTrial_->setObjectName("adoptTrialButton"); restoreTrial_->setObjectName("restoreTrialButton");
        trialButtons->addWidget(adoptTrial_); trialButtons->addWidget(restoreTrial_); trialButtons->addStretch(); trialLayout->addLayout(trialButtons);
        executionTabs_->addTab(trialPanel,QStringLiteral("试参对比")); executionDock_=dock(QStringLiteral("执行记录"),executionTabs_,Qt::BottomDockWidgetArea);
        connect(diagnosticDetails_,&QToolButton::clicked,this,[this] {executionDock_->show(); executionDock_->raise(); executionTabs_->setCurrentIndex(2);});
        adoptTrial_->setEnabled(false); restoreTrial_->setEnabled(false);
        connect(trialList_,&QListWidget::currentRowChanged,this,[this](int row) {adoptTrial_->setEnabled(!host_.busy()&&row>=0&&row<trialVariants_.size()&&trialList_->item(row)->data(Qt::UserRole).toBool());});
        connect(adoptTrial_,&QPushButton::clicked,this,[this] {const int row=trialList_->currentRow(); if(editor_&&row>=0&&row<trialVariants_.size()&&trialList_->item(row)->data(Qt::UserRole).toBool()) {editor_->setValues(trialVariants_[row].values); resetDiagnostics(); setSceneNotice(QStringLiteral("已采用试参参数 · 请重新运行诊断确认"),"info"); setStatus(QStringLiteral("已采用试参参数；结果仍需人工确认"));}});
        connect(restoreTrial_,&QPushButton::clicked,this,[this] {if(editor_) {editor_->setValues(trialBaseline_); setStatus(QStringLiteral("已恢复试参前参数")); if(baselineReport_.isEmpty()) setSceneNotice(QStringLiteral("已恢复试参前参数 · 请重新检查"),"info"); else {AlgorithmResult restored; restored.metrics["diagnostics"]=baselineReport_; renderDiagnostics(restored); refreshDiagnosticNotice(QStringLiteral("已恢复参数 · 显示试参前基线诊断"));}}});
        connect(diagnosticsList_,&QListWidget::currentRowChanged,this,[this](int row) {if(row>=0&&row<currentIssues_.size()) highlightDiagnostic(currentIssues_.at(row).toObject());});
        timing_=new QLabel; timing_->setObjectName("timingLabel"); timing_->setAccessibleName(QStringLiteral("本次运行耗时")); statusBar()->addPermanentWidget(timing_); timing_->hide();
        progress_=new QProgressBar; progress_->setTextVisible(false); progress_->setMaximumWidth(140); progress_->setAccessibleName(QStringLiteral("算法运行进度")); statusBar()->addPermanentWidget(progress_);
        percent_=new QLabel("0%"); statusBar()->addPermanentWidget(percent_); connect(progress_,&QProgressBar::valueChanged,percent_,[this](int value) {percent_->setText(QLocale().toString(value)+"%");});
        status_=new QLabel(QStringLiteral("就绪")); status_->setProperty("role","status"); statusBar()->addWidget(status_,1);
        viewMenu_->addSeparator(); auto dark=viewMenu_->addAction(QStringLiteral("深色主题")); dark->setCheckable(true); dark->setChecked(qApp->property("workbench.dark").toBool());
        connect(dark,&QAction::toggled,this,[](bool enabled) {QSettings().setValue("appearance/dark",enabled); workbench::applyTheme(*qApp,enabled);});
        viewMenu_->addAction(QStringLiteral("恢复默认布局"),this,[this] {resetLayout();});
        connect(algorithms_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this] { selectAlgorithm(); });
        connect(advanced_,&QCheckBox::toggled,this,[this](bool enabled) {debugGroupAction_->setVisible(enabled); debugGroup_->setVisible(enabled); debugging_->setEnabled(enabled&&!host_.busy()); diagnosticNoticeOption_->setVisible(enabled); diagnosticNoticeOption_->setEnabled(enabled&&!host_.busy()); if(!enabled) {debugging_->setChecked(false); diagnosticNoticeEnabledForRun_=false;} if(editor_) editor_->setAdvanced(enabled); refreshVisibleSceneNotice();});
        connect(diagnosticNoticeOption_,&QCheckBox::toggled,this,[this](bool checked) {if(!checked) diagnosticNoticeEnabledForRun_=false; refreshVisibleSceneNotice();});
        connect(datasets_,&QListWidget::currentRowChanged,this,[this](int row) {
            if(!host_.busy()) {resetDiagnostics(); invalidateTrialOptions();}
            selectionBadge_->setText(row>=0?QStringLiteral("1 选中"):QStringLiteral("0 选中"));
            if(row>=0&&row<data_.size()) {
                const auto& data=data_[row]; const auto points=data.cloud?qulonglong(data.cloud->width)*data.cloud->height:0;
                inputInfo_->setText(QStringLiteral("%1\n%2 点 · %3").arg(data.name,QLocale().toString(points),data.unit=="unknown"?QStringLiteral("坐标单位"):data.unit));
                if(!host_.busy()) {auto layer=DebugLayer::background(data.name,data); layer.style.opacity=0.8; showLayers({layer});}
            }
            run_->setEnabled(!host_.busy()&&editor_&&row>=0); autoCheck_->setEnabled(!host_.busy()&&editor_&&row>=0); save_->setEnabled(row>=0);
        });
        connect(layers_,&QListWidget::itemChanged,this,[this](QListWidgetItem* item) { int row=layers_->row(item); if(row>=0&&row<display_.size()) { display_[row].style.visible=item->checkState()==Qt::Checked; scene_->setLayers(display_); } });
        connect(layers_,&QListWidget::itemDoubleClicked,this,[this](QListWidgetItem* item) { editStyle(layers_->row(item)); });
        connect(steps_,&QListWidget::itemActivated,this,[this](QListWidgetItem* item) {
            const auto seq=item->data(Qt::UserRole).toULongLong(); const auto frame=host_.history(seq);
            if(frame) displayFrame(frame); else status_->setText(QStringLiteral("该步骤快照已释放，名称与统计仍可查看"));
        });
        connect(&host,&Host::logMessage,this,[this](const QString& text) { logs_->appendPlainText(text); });
        connect(&host,&Host::progressChanged,progress_,&QProgressBar::setValue);
        connect(&host,&Host::runningChanged,this,[this](bool running) { updateRunning(running); });
        connect(&host,&Host::pausedChanged,this,[this](bool paused) {next_->setEnabled(paused); setStatus(paused?QStringLiteral("已暂停 · 按 F10 下一步，F6 继续"):host_.busy()?QStringLiteral("算法运行中…"):QStringLiteral("就绪"));});
        connect(&host,&Host::debugReady,this,[this](FramePtr frame) {
            runtimeDebug_=true; resume_->setEnabled(true); step_->setEnabled(true);
            auto item=new QListWidgetItem(QStringLiteral("%1 / %2  %3").arg(frame->runId).arg(frame->sequence).arg(frame->title),steps_);
            item->setData(Qt::UserRole,qulonglong(frame->sequence));
            QStringList details{frame->message}; for(auto it=frame->metrics.begin();it!=frame->metrics.end();++it) details<<it.key()+": "+it.value().toVariant().toString();
            item->setToolTip(details.join('\n')); steps_->setCurrentItem(item); displayFrame(frame);
            QTimer::singleShot(0,this,[this,seq=frame->sequence] { host_.presented(seq); });
        });
        connect(&host,&Host::debugEvicted,this,[this](quint64 seq) {
            if(windows_.value(seq)) delete windows_[seq].data(); windows_.remove(seq);
            for(int i=0;i<steps_->count();++i) if(steps_->item(i)->data(Qt::UserRole).toULongLong()==seq) steps_->item(i)->setText(steps_->item(i)->text()+QStringLiteral(" [快照已释放]"));
        });
        connect(&host,&Host::finished,this,[this](AlgorithmResult result) {
            if(result.status==ResultStatus::Success) {
                for(const auto& output:result.outputs) {data_<<output; auto item=new QListWidgetItem(output.name,datasets_); item->setToolTip(output.name);}
                datasetsStack_->setCurrentIndex(data_.isEmpty()?0:1);
                if(datasets_->currentRow()<0&&!data_.isEmpty()) datasets_->setCurrentRow(data_.size()-1);
                if(!result.layers.isEmpty()) showLayers(result.layers);
                else { QVector<DebugLayer> layers; for(const auto& output:result.outputs) layers<<DebugLayer::result(output.name,output); showLayers(layers); }
                progress_->setValue(100); setStatus(QStringLiteral("处理完成 · %1 个输出").arg(result.outputs.size()),"success");
            } else setStatus(result.status==ResultStatus::Cancelled?QStringLiteral("已停止"):QStringLiteral("处理失败：")+result.message,result.status==ResultStatus::Failed?"error":"idle");
            if(result.status==ResultStatus::Success) renderDiagnostics(result);
            else {
                resetDiagnostics(); diagnosisSummary_->setText(QStringLiteral("本次运行未形成有效诊断"));
                if(result.status==ResultStatus::Cancelled) setSceneNotice(QStringLiteral("本次运行已停止 · 未形成有效诊断"),"info");
                else setSceneNotice(QStringLiteral("处理失败：")+result.message,"critical");
            }
            // 总耗时到结果图层设置完成为止；算法耗时由插件记录纯识别核心。
            if(runTimer_.isValid()) {
                QString text=QStringLiteral("整体 %1").arg(formatDuration(runTimer_.elapsed()));
                const auto algorithmMs=result.metrics.value("algorithm_elapsed_ms");
                if(algorithmMs.isDouble()&&algorithmMs.toDouble()>=0)
                    text+=QStringLiteral(" · 算法 %1").arg(formatDuration(qRound64(algorithmMs.toDouble())));
                timing_->setText(text); timing_->show(); runTimer_.invalidate();
            }
            logs_->appendPlainText(status_->text()+" "+result.message);
            for(auto it=result.metrics.begin();it!=result.metrics.end();++it) logs_->appendPlainText(it.key()+": "+it.value().toVariant().toString());
            if(closePending_) QTimer::singleShot(0,this,[this] { close(); });
        });
        if(auto runtime=dynamic_cast<Runtime*>(&host)) connect(runtime,&Runtime::trialFinished,this,[this](AlgorithmResult result) {onTrialFinished(result);});
        for(const auto& message:host.diagnostics()) logs_->appendPlainText(message);
        selectAlgorithm(); updateRunning(false);
        resetLayout();
    }
    ~MainWindow() override {
        if(editor_) host_.parametersUi()->destroyEditor(editor_);
        host_.visualization()->destroyView(scene_);
    }
protected:
    void resizeEvent(QResizeEvent* event) override {
        QMainWindow::resizeEvent(event);
        for(auto label:findChildren<QLabel*>()) if(label->property("compactHide").toBool()) label->setVisible(height()>=600);
    }
    bool eventFilter(QObject* watched,QEvent* event) override {
        if(watched==sceneHint_&&event->type()==QEvent::Resize) elideSceneNotice();
        return QMainWindow::eventFilter(watched,event);
    }
    void closeEvent(QCloseEvent* event) override {
        if(host_.busy()) { closePending_=true; trialCancelled_=true; host_.stop(); status_->setText(QStringLiteral("正在等待算法安全停止…")); event->ignore(); }
        else QMainWindow::closeEvent(event);
    }
private:
    Host& host_; ISceneView* scene_=nullptr; IParameterEditor* editor_=nullptr;
    QComboBox* algorithms_; QCheckBox *advanced_,*debugging_,*diagnosticNoticeOption_; QVBoxLayout* parameterLayout_;
    QListWidget *datasets_,*layers_,*steps_,*diagnosticsList_,*trialList_; QPlainTextEdit* logs_; QProgressBar* progress_; QLabel *status_,*timing_,*inputInfo_,*sceneHint_,*selectionBadge_,*percent_,*diagnosisSummary_;
    QPushButton *adoptTrial_,*restoreTrial_; QToolButton* diagnosticDetails_; QTabWidget* executionTabs_;
    QAction *open_,*save_,*run_,*autoCheck_,*stop_,*next_,*resume_,*step_,*debugGroupAction_;
    QMenu* viewMenu_; QWidget* debugGroup_; QStackedWidget *sceneStack_,*datasetsStack_,*layersStack_; QDockWidget *leftDock_,*parameterDock_,*executionDock_;
    QVector<CloudData> data_; QVector<DebugLayer> display_; QMap<quint64,QPointer<QWidget>> windows_; bool closePending_=false,runtimeDebug_=false;
    QElapsedTimer runTimer_;
    AlgorithmInput trialInput_; ParameterSet trialBaseline_; QString trialAlgorithm_;
    QVector<TrialVariant> trialVariants_; QJsonObject baselineReport_; QJsonArray currentIssues_;
    QString currentDiagnosticsInputId_; QJsonObject currentDiagnosticsParameters_;
    QString sceneNoticeFullText_,visibleSceneNoticeText_,sceneNoticeState_=QStringLiteral("idle"); int sceneNoticeDetails_=0;
    bool diagnosticNoticeEnabledForRun_=false;
    QElapsedTimer trialTimer_; bool trialActive_=false,trialCancelled_=false; int trialStep_=-1,trialGeneration_=0;
    QDockWidget* dock(const QString& title,QWidget* widget,Qt::DockWidgetArea area) {
        auto d=new QDockWidget(title,this); d->setObjectName(title); d->setWidget(widget); addDockWidget(area,d); viewMenu_->addAction(d->toggleViewAction()); return d;
    }
    void setStatus(const QString& text,const char* state="idle") {status_->setText(text); status_->setProperty("state",state); workbench::repolish(status_);}
    void elideSceneNotice() {
        sceneHint_->setText(sceneHint_->fontMetrics().elidedText(visibleSceneNoticeText_,Qt::ElideRight,std::max(1,sceneHint_->width()-12)));
    }
    void refreshVisibleSceneNotice() {
        const bool showDiagnostics=advanced_->isChecked()&&diagnosticNoticeEnabledForRun_;
        const bool parametersChanged=showDiagnostics&&!currentDiagnosticsParameters_.isEmpty()&&editor_
            &&Parameters::merge(host_.schema(algorithmId()),editor_->values())!=currentDiagnosticsParameters_;
        visibleSceneNoticeText_=!showDiagnostics?QStringLiteral("左键拖动旋转 · 右键拖动平移 · 滚轮缩放")
            :parametersChanged?QStringLiteral("参数已修改 · 请重新运行诊断"):sceneNoticeFullText_;
        sceneHint_->setToolTip(visibleSceneNoticeText_); sceneHint_->setAccessibleName(visibleSceneNoticeText_);
        sceneHint_->setProperty("state",!showDiagnostics?QStringLiteral("idle"):parametersChanged?QStringLiteral("warning"):sceneNoticeState_);
        workbench::repolish(sceneHint_); elideSceneNotice();
        diagnosticDetails_->setText(sceneNoticeDetails_>1?QStringLiteral("查看诊断（%1）").arg(sceneNoticeDetails_):QStringLiteral("查看诊断"));
        diagnosticDetails_->setVisible(showDiagnostics&&!parametersChanged&&sceneNoticeDetails_>0);
    }
    void setSceneNotice(const QString& text,const char* state="idle",int details=0) {
        sceneNoticeFullText_=text; sceneNoticeState_=QString::fromLatin1(state); sceneNoticeDetails_=details;
        refreshVisibleSceneNotice();
    }
    void resetDiagnostics() {
        currentIssues_={}; currentDiagnosticsInputId_.clear(); currentDiagnosticsParameters_={}; diagnosticsList_->clear();
        diagnosisSummary_->setText(QStringLiteral("运行后显示结构化诊断"));
        setSceneNotice(QStringLiteral("左键拖动旋转 · 右键拖动平移 · 滚轮缩放"));
    }
    void invalidateTrialOptions() {
        trialVariants_.clear(); baselineReport_={}; trialList_->clear(); restoreTrial_->setEnabled(false); adoptTrial_->setEnabled(false);
    }
    void refreshDiagnosticNotice(const QString& context={}) {
        if(currentIssues_.isEmpty()) {
            setSceneNotice(context.isEmpty()?QStringLiteral("未报告问题 · 仍需人工复核"):context,"info",diagnosticsList_->count());
            return;
        }
        const auto issue=currentIssues_.at(0).toObject();
        const auto severity=issue.value("severity").toString();
        const auto label=severity=="critical"?QStringLiteral("严重"):severity=="warning"?QStringLiteral("警告"):QStringLiteral("提示");
        QString text=label+QStringLiteral("：")+issue.value("summary").toString();
        if(currentIssues_.size()>1) text+=QStringLiteral(" · 另有 %1 条").arg(currentIssues_.size()-1);
        if(!context.isEmpty()) text+=QStringLiteral(" · ")+context;
        setSceneNotice(text,severity=="critical"?"critical":severity=="warning"?"warning":"info",diagnosticsList_->count());
    }
    static QString formatDuration(qint64 ms) {
        if(ms<1000) return QStringLiteral("%1 ms").arg(QLocale().toString(ms));
        return QStringLiteral("%1 s").arg(QLocale().toString(ms/1000.0,'f',2));
    }
    void resetLayout() {
        for(auto d:{leftDock_,parameterDock_,executionDock_}) {d->setFloating(false); d->show();}
        addDockWidget(Qt::LeftDockWidgetArea,leftDock_); addDockWidget(Qt::RightDockWidgetArea,parameterDock_); addDockWidget(Qt::BottomDockWidgetArea,executionDock_);
        resizeDocks({leftDock_,parameterDock_},{qBound(200,width()/5,310),qBound(300,width()/4,390)},Qt::Horizontal);
        resizeDocks({executionDock_},{qBound(100,height()/6,155)},Qt::Vertical);
        // On laptop screens, keep the canvas usable; the View menu restores history.
        executionDock_->setVisible(height()>=760);
    }
    QString algorithmId() const { return algorithms_->currentData().toString(); }
    void selectAlgorithm() {
        resetDiagnostics(); invalidateTrialOptions();
        if(editor_) { host_.parametersUi()->destroyEditor(editor_); editor_=nullptr; }
        if(algorithmId().isEmpty()) return;
        algorithms_->setToolTip(algorithms_->currentText()); algorithms_->setAccessibleDescription(algorithms_->currentText());
        editor_=host_.parametersUi()->createEditor(host_.schema(algorithmId()),this);
        if(editor_) { editor_->widget()->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Ignored); parameterLayout_->addWidget(editor_->widget(),1); editor_->setAdvanced(advanced_->isChecked()); }
    }
    void updateRunning(bool running) {
        debugGroupAction_->setVisible(advanced_->isChecked());
        debugGroup_->setVisible(advanced_->isChecked());
        open_->setEnabled(!running); algorithms_->setEnabled(!running); datasets_->setEnabled(!running);
        run_->setEnabled(!running&&editor_&&datasets_->currentRow()>=0); autoCheck_->setEnabled(!running&&editor_&&datasets_->currentRow()>=0); save_->setEnabled(datasets_->currentRow()>=0); stop_->setEnabled(running); next_->setEnabled(false);
        adoptTrial_->setEnabled(!running&&trialList_->currentRow()>=0&&trialList_->currentRow()<trialVariants_.size()&&trialList_->currentItem()->data(Qt::UserRole).toBool());
        if(running) runtimeDebug_=advanced_->isChecked()&&debugging_->isChecked();
        resume_->setEnabled(running&&runtimeDebug_); step_->setEnabled(running&&runtimeDebug_); advanced_->setEnabled(!running); debugging_->setEnabled(!running&&advanced_->isChecked()); diagnosticNoticeOption_->setEnabled(!running&&advanced_->isChecked());
        progress_->setVisible(running); percent_->setVisible(running);
        setStatus(running?QStringLiteral("算法运行中…"):QStringLiteral("就绪"));
    }
    void showLayers(QVector<DebugLayer> layers) {
        display_=std::move(layers); layers_->blockSignals(true); layers_->clear();
        for(const auto& layer:display_) { auto item=new QListWidgetItem(layer.name,layers_); item->setToolTip(layer.name); item->setFlags(item->flags()|Qt::ItemIsUserCheckable); item->setCheckState(layer.style.visible?Qt::Checked:Qt::Unchecked); }
        layers_->blockSignals(false); layersStack_->setCurrentIndex(display_.isEmpty()?0:1); sceneStack_->setCurrentIndex(display_.isEmpty()?0:1); scene_->setLayers(display_); scene_->fit();
    }
    void open() {
        const auto file=QFileDialog::getOpenFileName(this,QStringLiteral("打开点云"),{},"Point cloud (*.pcd *.ply *.txt)"); if(file.isEmpty()) return;
        bool ok=false; const auto unit=QInputDialog::getItem(this,QStringLiteral("点云单位"),QStringLiteral("仅记录单位，不缩放坐标"),{"unknown","m","mm","cm"},0,false,&ok); if(!ok) return;
        QString error; auto cloud=host_.readCloud(file,unit,error); if(!error.isEmpty()) { QMessageBox::warning(this,QStringLiteral("加载失败"),error); return; }
        data_<<cloud; auto item=new QListWidgetItem(cloud.name,datasets_); item->setToolTip(cloud.name); datasetsStack_->setCurrentIndex(1); datasets_->setCurrentRow(data_.size()-1);
    }
    void save() {
        const int row=datasets_->currentRow(); if(row<0) return;
        auto file=QFileDialog::getSaveFileName(this,QStringLiteral("保存点云"),{},"PCD (*.pcd);;PLY (*.ply)"); if(file.isEmpty()) return;
        QString error; if(!host_.writeCloud(file,data_[row],error)) QMessageBox::warning(this,QStringLiteral("保存失败"),error);
    }
    void run() {
        if(!editor_||datasets_->currentRow()<0) { QMessageBox::information(this,QStringLiteral("运行"),QStringLiteral("请先打开点云并选择算法。")); return; }
        AlgorithmInput input; input.clouds={data_[datasets_->currentRow()]}; QString error;
        diagnosticNoticeEnabledForRun_=advanced_->isChecked()&&diagnosticNoticeOption_->isChecked();
        resetDiagnostics(); invalidateTrialOptions(); setSceneNotice(QStringLiteral("算法运行中 · 诊断待生成"),"info");
        timing_->hide(); runTimer_.start();
        if(!host_.start(algorithmId(),input,editor_->values(),advanced_->isChecked()&&debugging_->isChecked(),error)) {
            runTimer_.invalidate(); setSceneNotice(QStringLiteral("无法运行：")+error,"critical"); QMessageBox::warning(this,QStringLiteral("无法运行"),error);
        }
    }
    void renderDiagnostics(const AlgorithmResult& result) {
        diagnosticsList_->clear(); currentIssues_=result.metrics.value("diagnostics").toObject().value("issues").toArray();
        currentDiagnosticsInputId_=result.metrics.value("diagnostics").toObject().value("input").toObject().value("cloudId").toString();
        currentDiagnosticsParameters_=result.metrics.value("diagnostics").toObject().value("parameters").toObject();
        if(result.metrics.contains("diagnostics_validation")) {
            diagnosisSummary_->setText(QStringLiteral("诊断报告被主软件校验拒绝"));
            auto item=new QListWidgetItem(QStringLiteral("诊断报告无效：")+result.metrics.value("diagnostics_validation").toString(),diagnosticsList_); item->setForeground(QColor(220,90,90));
            setSceneNotice(item->text(),"critical",diagnosticsList_->count()); return;
        }
        if(!result.metrics.contains("diagnostics")) {diagnosisSummary_->setText(QStringLiteral("无结构化诊断")); new QListWidgetItem(QStringLiteral("此算法未提供结构化诊断；可查看运行日志。"),diagnosticsList_); setSceneNotice(QStringLiteral("此算法未提供结构化诊断 · 可查看运行日志"),"info",diagnosticsList_->count()); return;}
        if(currentIssues_.isEmpty()) {diagnosisSummary_->setText(QStringLiteral("未报告问题")); new QListWidgetItem(QStringLiteral("未报告问题；不代表已通过人工真值验收。"),diagnosticsList_); setSceneNotice(QStringLiteral("未报告问题 · 不代表已通过人工真值验收"),"info",diagnosticsList_->count()); return;}
        const auto profile=host_.schema(trialActive_?trialAlgorithm_:algorithmId()).value("diagnostics").toObject();
        QMap<QString,int> stageOrder; QMap<QString,QString> stageLabels,targetLabels; int stageIndex=0;
        for(const auto& v:profile.value("stages").toArray()) {const auto stage=v.toObject(); const auto id=stage.value("id").toString(); stageOrder[id]=stageIndex++; stageLabels[id]=stage.value("label").toString(id);}
        for(const auto& v:profile.value("outputs").toArray()) {const auto output=v.toObject(); const auto id=output.value("targetId").toString(); targetLabels[id]=output.value("targetLabel").toString(id);}
        QVector<QJsonObject> ordered; for(const auto& v:currentIssues_) ordered<<v.toObject();
        std::stable_sort(ordered.begin(),ordered.end(),[&](const QJsonObject& a,const QJsonObject& b) {
            const auto severity=[](const QJsonObject& o) {return o.value("severity")=="critical"?0:o.value("severity")=="warning"?1:2;};
            if(severity(a)!=severity(b)) return severity(a)<severity(b);
            return stageOrder.value(a.value("stage").toString(),999)<stageOrder.value(b.value("stage").toString(),999);
        });
        currentIssues_=QJsonArray{}; for(const auto& o:ordered) currentIssues_.append(o);
        int critical=0,warning=0,first=999; QString firstStage;
        for(const auto& o:ordered) {
            if(o.value("severity")=="critical") ++critical;
            if(o.value("severity")=="warning") ++warning;
            if(o.value("severity")!="info"&&stageOrder.value(o.value("stage").toString(),999)<first) {first=stageOrder.value(o.value("stage").toString(),999); firstStage=o.value("stage").toString();}
        }
        diagnosisSummary_->setText(QStringLiteral("严重 %1 · 警告 %2 · 最早问题阶段：%3（来源仍需核实）").arg(critical).arg(warning).arg(firstStage.isEmpty()?QStringLiteral("未报告"):stageLabels.value(firstStage,firstStage)));
        for(const auto& v:currentIssues_) {
            const auto o=v.toObject(); const auto scope=o.value("scope").toObject(); const auto region=o.value("region").toObject();
            const auto kind=region.value("kind").toString();
            const auto place=kind=="section_range"
                ?QStringLiteral("截面 %1–%2").arg(region.value("first").toInt()).arg(region.value("last").toInt())
                :kind=="whole"?QStringLiteral("整片点云")
                :kind=="point_index"?QStringLiteral("原始点 %1").arg(region.value("index").toInt())
                :kind=="bbox"?QStringLiteral("空间范围"):kind;
            const auto severity=o.value("severity").toString(), origin=o.value("origin").toString(), visibility=o.value("visibility").toString();
            const QMap<QString,QString> severityLabels{{"critical",QStringLiteral("严重")},{"warning",QStringLiteral("警告")},{"info",QStringLiteral("提示")}};
            const QMap<QString,QString> originLabels{{"undetermined",QStringLiteral("来源未判明")},{"input_missing",QStringLiteral("输入缺测")},{"acquisition_noise_suspected",QStringLiteral("疑似采集噪声")},{"physical_defect_suspected",QStringLiteral("疑似产品瑕疵")},{"physical_defect_confirmed",QStringLiteral("已确认产品瑕疵")},{"algorithm_error_confirmed",QStringLiteral("已确认算法错误")}};
            const QMap<QString,QString> visibilityLabels{{"observable",QStringLiteral("可观测")},{"missing_input",QStringLiteral("输入缺测")},{"not_applicable",QStringLiteral("不适用")},{"unknown",QStringLiteral("可观测性未判定")}};
            const auto target=scope.value("targetId").toString(),stage=o.value("stage").toString();
            auto item=new QListWidgetItem(QStringLiteral("[%1] %2 · %3 · %4 · %5（%6）：%7")
                .arg(severityLabels.value(severity,severity),targetLabels.value(target,target),stageLabels.value(stage,stage),place,
                     originLabels.value(origin,origin),visibilityLabels.value(visibility,visibility),o.value("summary").toString()),diagnosticsList_);
            item->setToolTip(QString::fromUtf8(QJsonDocument(o).toJson(QJsonDocument::Indented)));
            if(o.value("severity")=="critical") item->setForeground(QColor(220,75,75));
            else if(o.value("severity")=="warning") item->setForeground(QColor(230,165,40));
        }
        refreshDiagnosticNotice();
    }
    void highlightDiagnostic(const QJsonObject& issue) {
        const auto region=issue.value("region").toObject(); const auto kind=region.value("kind").toString();
        DebugLayer marker; marker.name=QStringLiteral("诊断位置"); marker.role=LayerRole::Auxiliary;
        marker.style.color={1.0,0.18,0.15}; marker.style.pointSize=15; marker.style.lineWidth=4;
        if(kind=="section_range") {
            const auto p=region.value("representative").toArray();
            if(p.size()==3) {marker.geometry.kind=GeometryKind::Points; marker.geometry.vertices.push_back({p.at(0).toDouble(),p.at(1).toDouble(),p.at(2).toDouble()});
                setStatus(issue.value("summary").toString()+QStringLiteral(" · 代表位置 (%1, %2, %3)").arg(p.at(0).toDouble()).arg(p.at(1).toDouble()).arg(p.at(2).toDouble()));}
        } else if(kind=="bbox") {
            const auto a=region.value("min").toArray(),b=region.value("max").toArray();
            if(a.size()==3&&b.size()==3) {marker.geometry.kind=GeometryKind::Box;
                marker.geometry.vertices={{a.at(0).toDouble(),a.at(1).toDouble(),a.at(2).toDouble()},
                                          {b.at(0).toDouble(),b.at(1).toDouble(),b.at(2).toDouble()}};}
            setStatus(issue.value("summary").toString());
        } else setStatus(issue.value("summary").toString());
        if(marker.geometry.vertices.isEmpty()) return;
        QVector<DebugLayer> layers;
        for(const auto& layer:display_) if(layer.name!=QStringLiteral("诊断位置")) layers<<layer;
        bool hasInput=false;
        for(const auto& layer:layers) if(layer.data.id==currentDiagnosticsInputId_) hasInput=true;
        if(!hasInput) for(const auto& data:data_) if(data.id==currentDiagnosticsInputId_) {
            auto background=DebugLayer::background(QStringLiteral("诊断原始输入"),data); background.style.opacity=0.25; layers.prepend(background); break;
        }
        layers<<marker; showLayers(layers);
    }
    void autoCheck() {
        auto runtime=dynamic_cast<Runtime*>(&host_);
        if(!runtime||!editor_||datasets_->currentRow()<0||host_.busy()) return;
        diagnosticNoticeEnabledForRun_=advanced_->isChecked()&&diagnosticNoticeOption_->isChecked();
        resetDiagnostics();
        trialAlgorithm_=algorithmId(); trialInput_.clouds={data_[datasets_->currentRow()]};
        trialBaseline_=Parameters::merge(host_.schema(trialAlgorithm_),editor_->values());
        trialVariants_.clear(); baselineReport_={}; trialList_->clear();
        trialActive_=true; trialCancelled_=false; trialStep_=-1; trialTimer_.start(); restoreTrial_->setEnabled(true);
        QString error;
        if(!runtime->startTrial(trialAlgorithm_,trialInput_,trialBaseline_,error)) {
            trialActive_=false; restoreTrial_->setEnabled(false); setSceneNotice(QStringLiteral("无法自动检查：")+error,"critical"); QMessageBox::warning(this,QStringLiteral("无法自动检查"),error); return;
        }
        setStatus(QStringLiteral("自动检查：运行基线…")); setSceneNotice(QStringLiteral("自动检查：运行基线诊断…"),"info"); armTrialTimeout();
    }
    void armTrialTimeout() {
        const int generation=++trialGeneration_;
        QTimer::singleShot(30000,this,[this,generation] {
            if(trialActive_&&generation==trialGeneration_&&host_.busy()) {
                trialCancelled_=true; host_.stop(); setStatus(QStringLiteral("单次试跑超时，正在取消"));
                if(baselineReport_.isEmpty()) setSceneNotice(QStringLiteral("单次试跑超时 · 正在取消"),"warning");
                else refreshDiagnosticNotice(QStringLiteral("试跑超时 · 正在取消，保留基线诊断"));
            }
        });
    }
    void launchTrial() {
        if(trialCancelled_||trialTimer_.elapsed()>120000||trialStep_>=trialVariants_.size()) {
            trialActive_=false; setStatus(trialCancelled_?QStringLiteral("试跑已取消"):QStringLiteral("试跑结束；内部证据待人工确认"));
            refreshDiagnosticNotice(trialCancelled_?QStringLiteral("试跑已取消 · 保留基线诊断"):QStringLiteral("试跑结束 · 结果待人工确认")); return;
        }
        auto runtime=dynamic_cast<Runtime*>(&host_); QString error;
        const auto& variant=trialVariants_.at(trialStep_);
        if(!runtime||!runtime->startTrial(trialAlgorithm_,trialInput_,variant.values,error)) {
            new QListWidgetItem(variant.parameterId+QStringLiteral("：无法启动：")+error,trialList_); ++trialStep_; launchTrial(); return;
        }
        const auto progress=QStringLiteral("自动检查：试跑 %1/%2…").arg(trialStep_+1).arg(trialVariants_.size());
        setStatus(progress); refreshDiagnosticNotice(progress+QStringLiteral(" · 显示基线诊断")); armTrialTimeout();
    }
    void onTrialFinished(const AlgorithmResult& result) {
        ++trialGeneration_;
        if(!trialActive_) return;
        if(trialCancelled_||result.status==ResultStatus::Cancelled) {trialActive_=false; setStatus(QStringLiteral("试跑已取消"));
            if(baselineReport_.isEmpty()) {resetDiagnostics(); setSceneNotice(QStringLiteral("试跑已取消 · 无有效基线诊断"),"info");}
            else refreshDiagnosticNotice(QStringLiteral("试跑已取消 · 保留基线诊断"));
            if(closePending_) QTimer::singleShot(0,this,[this] {close();}); return;}
        const auto report=result.metrics.value("diagnostics").toObject();
        if(trialStep_<0) {
            renderDiagnostics(result); baselineReport_=report;
            if(report.isEmpty()||result.status!=ResultStatus::Success) {trialActive_=false; setStatus(QStringLiteral("基线诊断结束；无法生成试参"));
                if(result.status==ResultStatus::Failed) setSceneNotice(QStringLiteral("基线诊断失败：")+result.message,"critical",diagnosticsList_->count());
                else if(!result.metrics.contains("diagnostics_validation")) setSceneNotice(QStringLiteral("基线未返回结构化诊断 · 无法生成试参"),"warning",diagnosticsList_->count());
                return;}
            trialVariants_=TrialSession::plan(host_.schema(trialAlgorithm_),host_.schema(trialAlgorithm_).value("diagnostics").toObject(),report,trialBaseline_);
            if(trialVariants_.isEmpty()) {trialActive_=false; setStatus(QStringLiteral("没有有依据的数值试参；请查看诊断问题")); refreshDiagnosticNotice(QStringLiteral("无有依据的数值试参")); return;}
            trialStep_=0; launchTrial(); return;
        }
        const auto& variant=trialVariants_.at(trialStep_);
        QString line=variant.parameterId+" = "+variant.values.value(variant.parameterId).toVariant().toString();
        QString detail,comparisonDetail; bool adoptable=false;
        QString effect; bool suppressionRisk=true; // Missing declaration is treated conservatively.
        const auto rules=host_.schema(trialAlgorithm_).value("diagnostics").toObject().value("trialParameters").toArray();
        for(const auto& v:rules) {const auto rule=v.toObject(); if(rule.value("id")==variant.parameterId) {
            suppressionRisk=rule.value("maySuppressPhysicalDefect").toBool(true);
            effect=QStringLiteral("预期变化：%1\n可能退化：%2\n不能解决：%3\n")
                .arg(rule.value("expectedChange").toString(),rule.value("risk").toString(),rule.value("cannotFix").toString()); break;
        }}
        if(result.status!=ResultStatus::Success||report.isEmpty()) detail=QStringLiteral("试跑失败或诊断报告无效");
        else {
            const auto comparison=TrialSession::compare(baselineReport_,report);
            adoptable=TrialSession::oneClickAdoptionAllowed(host_.schema(trialAlgorithm_).value("diagnostics").toObject(),comparison,variant.parameterId);
            detail=comparison.value("comparable").toBool()
                ?QStringLiteral("诊断条目消失 %1，新增 %2；不代表定位改善")
                    .arg(comparison.value("disappearedIssues").toArray().size()).arg(comparison.value("newIssues").toArray().size())
                :comparison.value("reason").toString();
            if(!comparison.value("lostOutputs").toArray().isEmpty()) detail+=QStringLiteral("；有输出身份丢失");
            if(comparison.value("physicalDefectRequiresReview").toBool()) detail+=QStringLiteral("；疑似真实缺陷需复核");
            if(comparison.value("reviewRequired").toBool()||suppressionRisk) detail+=QStringLiteral("；可能掩盖真实瑕疵，需人工复核后手动设置");
            comparisonDetail=QString::fromUtf8(QJsonDocument(comparison).toJson(QJsonDocument::Indented));
        }
        auto item=new QListWidgetItem(line+QStringLiteral(" · ")+detail,trialList_);
        item->setData(Qt::UserRole,adoptable);
        item->setToolTip(QStringLiteral("参数作用仅是试探假设。\n")+effect+comparisonDetail);
        ++trialStep_; launchTrial();
    }
    void loadPreset() {
        const auto file=QFileDialog::getOpenFileName(this,QStringLiteral("加载参数预设"),{},"JSON (*.json)"); if(file.isEmpty()) return;
        ParameterSet values; QStringList messages; const bool ok=host_.loadPreset(algorithmId(),file,values,messages);
        if(ok) editor_->setValues(values);
        if(!messages.isEmpty()) QMessageBox::information(this,ok?QStringLiteral("预设提示"):QStringLiteral("预设未加载"),messages.join('\n'));
    }
    void savePreset() {
        const auto file=QFileDialog::getSaveFileName(this,QStringLiteral("保存参数预设"),{},"JSON (*.json)"); if(file.isEmpty()) return;
        QString error; if(!host_.savePreset(algorithmId(),file,editor_->values(),error)) QMessageBox::warning(this,QStringLiteral("保存失败"),error);
    }
    void displayFrame(FramePtr frame) {
        auto old=windows_.value(frame->sequence); if(old) { old->show(); old->raise(); return; }
        try { windows_[frame->sequence]=host_.visualization()->showDebug(frame,{[this] {host_.next();},[this] {host_.continueRun();},[this] {host_.stop();}},this); }
        catch(const std::exception& e) { logs_->appendPlainText(QString::fromUtf8(e.what())); host_.stop(); }
        catch(...) { logs_->appendPlainText(QStringLiteral("调试窗口创建失败")); host_.stop(); }
    }
    void editStyle(int row) {
        if(row<0||row>=display_.size()) return;
        QDialog dialog(this); dialog.setWindowTitle(QStringLiteral("图层样式")); auto layout=new QFormLayout(&dialog); auto style=display_[row].style;
        auto size=new QDoubleSpinBox; size->setRange(1,30); size->setValue(style.pointSize); layout->addRow(QStringLiteral("点大小"),size);
        auto width=new QDoubleSpinBox; width->setRange(1,20); width->setValue(style.lineWidth); layout->addRow(QStringLiteral("线宽"),width);
        auto opacity=new QDoubleSpinBox; opacity->setRange(0,1); opacity->setSingleStep(0.05); opacity->setValue(style.opacity); layout->addRow(QStringLiteral("透明度"),opacity);
        auto color=new QPushButton(QStringLiteral("选择颜色")); layout->addRow(color);
        connect(color,&QPushButton::clicked,&dialog,[&] { auto c=QColorDialog::getColor(QColor::fromRgbF(style.color[0],style.color[1],style.color[2]),&dialog); if(c.isValid()) style.color={c.redF(),c.greenF(),c.blueF()}; });
        auto buttons=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel); layout->addRow(buttons);
        connect(buttons,&QDialogButtonBox::accepted,&dialog,&QDialog::accept); connect(buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
        if(dialog.exec()==QDialog::Accepted) { style.pointSize=size->value(); style.lineWidth=width->value(); style.opacity=opacity->value(); display_[row].style=style; scene_->setLayers(display_); }
    }
    void debugSettings() {
        if(host_.busy()) { status_->setText(QStringLiteral("运行结束后可修改调试资源限制")); return; }
        QSettings settings; QDialog dialog(this); dialog.setWindowTitle(QStringLiteral("调试资源限制")); auto form=new QFormLayout(&dialog);
        auto windows=new QSpinBox; windows->setRange(1,64); windows->setValue(settings.value("debug/windows",8).toInt()); form->addRow(QStringLiteral("保留步骤 / 窗口上限"),windows);
        auto memory=new QSpinBox; memory->setRange(1,8192); memory->setValue(int(settings.value("debug/bytes",qulonglong(512ull*1024*1024)).toULongLong()/1024/1024)); form->addRow(QStringLiteral("快照内存 MiB"),memory);
        auto buttons=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel); form->addRow(buttons); connect(buttons,&QDialogButtonBox::accepted,&dialog,&QDialog::accept); connect(buttons,&QDialogButtonBox::rejected,&dialog,&QDialog::reject);
        if(dialog.exec()==QDialog::Accepted) host_.setDebugLimits(windows->value(),quint64(memory->value())*1024*1024);
    }
};
class MainPlugin final : public QObject,public IMainUiPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_MAIN_IID FILE "metadata.json")
    Q_INTERFACES(fw::IMainUiPlugin)
public:
    QWidget* createWindow(Host& host) override { return new MainWindow(host); }
};
}
#include "MainUI.moc"
