#include <fw/UiInterfaces.h>
#include <fw/Parameters.h>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QMap>
#include <limits>
#include <QScrollBar>
#include <QGridLayout>
#include <QPainter>
#include <QPixmap>
#include <QKeyEvent>
#include "Theme.h"

namespace {
using namespace fw;
struct Binding {
    QWidget* control=nullptr;
    QWidget* row=nullptr;
    QJsonObject definition;
    std::function<QJsonValue()> read;
    std::function<void(QJsonValue)> write;
};
class Editor final : public QWidget, public IParameterEditor {
public:
    Editor(QJsonObject schema,QWidget* parent):QWidget(parent),schema_(std::move(schema)) {
        auto layout=new QVBoxLayout(this); layout->setContentsMargins(0,0,0,0);
        auto scroll=new QScrollArea; scroll->setObjectName("parameterScroll"); scroll->setWidgetResizable(true); scroll->setFrameShape(QFrame::NoFrame); layout->addWidget(scroll);
        auto body=new QWidget; body->setObjectName("parameterBody"); auto form=new QVBoxLayout(body); form->setContentsMargins(0,0,4,0); form->setSpacing(8); scroll->setWidget(body);
        registerFactories();
        for(auto value:schema_["groups"].toArray()) {
            auto group=value.toObject(); auto box=new QGroupBox(group["title"].toString());
            box->setObjectName("group_"+group["id"].toString());
            groups_.push_back({box,group["advanced"].toBool()}); auto rows=new QVBoxLayout(box); rows->setSpacing(7);
            for(auto parameter:group["parameters"].toArray()) {
                const auto p=parameter.toObject(); auto binding=factories_.value(p["type"].toString())(p);
                binding.definition=p; binding.control->setObjectName(p["id"].toString());
                binding.control->setToolTip(p["description"].toString());
                binding.control->setAccessibleName(p["label"].toString(p["id"].toString())); binding.control->setAccessibleDescription(p["description"].toString());
                auto row=new QWidget;
                const auto unit=p["unit"].toString();
                const auto label=p["label"].toString(p["id"].toString());
                auto rowLayout=new QVBoxLayout(row); rowLayout->setContentsMargins(0,0,0,0); rowLayout->setSpacing(3);
                const auto captionText=unit.isEmpty()?label:label+QStringLiteral("（%1）").arg(unit);
                if(auto check=qobject_cast<QCheckBox*>(binding.control)) {
                    check->setText(captionText); rowLayout->addWidget(check);
                } else {
                    auto caption=new QLabel(captionText); caption->setObjectName("label_"+p["id"].toString());
                    caption->setProperty("role","muted"); caption->setWordWrap(true); caption->setBuddy(binding.control);
                    caption->setToolTip(captionText+(p["description"].toString().isEmpty()?QString():"\n"+p["description"].toString()));
                    rowLayout->addWidget(caption);
                    binding.control->setSizePolicy(QSizePolicy::Expanding,binding.control->sizePolicy().verticalPolicy());
                    rowLayout->addWidget(binding.control);
                }
                binding.row=row; rows->addWidget(row); bindings_.insert(p["id"].toString(),binding);
                // Scrolling a long form must not silently alter a focused numeric value.
                auto focusControls=binding.control->findChildren<QWidget*>(); focusControls.prepend(binding.control);
                for(auto control:focusControls) if(qobject_cast<QAbstractSpinBox*>(control)||qobject_cast<QComboBox*>(control)) {control->setFocusPolicy(Qt::StrongFocus); control->installEventFilter(this);}
            }
            form->addWidget(box);
        }
        form->addStretch(); errors_=new QLabel; errors_->setObjectName("parameterErrors"); errors_->setAccessibleName(QStringLiteral("参数校验问题")); errors_->setWordWrap(true); errors_->setProperty("role","error"); layout->addWidget(errors_);
        setValues(Parameters::defaults(schema_));
    }
    QWidget* widget() override { return this; }
    ParameterSet values() const override { ParameterSet result; for(auto it=bindings_.begin();it!=bindings_.end();++it) result[it.key()]=it->read(); return result; }
    void setValues(const ParameterSet& input) override {
        updating_=true; const auto merged=Parameters::merge(schema_,input);
        for(auto it=bindings_.begin();it!=bindings_.end();++it) it->write(merged[it.key()]);
        updating_=false; refresh();
    }
    void setAdvanced(bool value) override { advanced_=value; refresh(); }
    QStringList errors() const override { return Parameters::validate(schema_,values()); }
protected:
    bool eventFilter(QObject* watched,QEvent* event) override {
        if(event->type()==QEvent::Wheel) {event->ignore(); return true;}
        return QWidget::eventFilter(watched,event);
    }
private:
    QJsonObject schema_; QMap<QString,Binding> bindings_;
    QMap<QString,std::function<Binding(const QJsonObject&)>> factories_;
    QVector<QPair<QGroupBox*,bool>> groups_; QLabel* errors_=nullptr;
    bool advanced_=false,updating_=false;
    void refresh() {
        if(updating_||!errors_) return;
        const auto current=values();
        for(const auto& group:groups_) group.first->setVisible(!group.second||advanced_);
        for(auto& b:bindings_) {
            b.row->setVisible((!b.definition.value("advanced").toBool()||advanced_)&&Parameters::condition(b.definition.value("visibleWhen"),current));
            b.control->setEnabled(Parameters::condition(b.definition.value("enabledWhen"),current));
        }
        const auto issues=errors(); errors_->setText(issues.join('\n')); errors_->setVisible(!issues.isEmpty());
    }
    QDoubleSpinBox* numeric(const QJsonObject& p,bool integer=false) {
        auto spin=new QDoubleSpinBox;
        spin->setMinimumWidth(fontMetrics().horizontalAdvance("-0.00")+34); spin->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
        spin->setDecimals(integer?0:p["decimals"].toInt(6));
        spin->setRange(p["min"].toDouble(integer?std::numeric_limits<int>::min():-1e100),p["max"].toDouble(integer?std::numeric_limits<int>::max():1e100));
        spin->setSingleStep(p["step"].toDouble(integer?1:0.01));
        connect(spin,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[this] { refresh(); }); return spin;
    }
    void registerFactories() {
        factories_["bool"]=[this](const QJsonObject& p) {
            auto w=new QCheckBox; connect(w,&QCheckBox::toggled,this,[this] { refresh(); });
            return Binding{w,nullptr,{},[w] { return QJsonValue(w->isChecked()); },[w](QJsonValue v) { w->setChecked(v.toBool()); }};
        };
        factories_["int"]=[this](const QJsonObject& p) {
            auto w=new QSpinBox; w->setRange(p["min"].toInt(std::numeric_limits<int>::min()),p["max"].toInt(std::numeric_limits<int>::max())); w->setSingleStep(p["step"].toInt(1));
            w->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed); w->setMinimumWidth(fontMetrics().horizontalAdvance("-0000")+34);
            connect(w,QOverload<int>::of(&QSpinBox::valueChanged),this,[this] { refresh(); });
            return Binding{w,nullptr,{},[w] { return QJsonValue(w->value()); },[w](QJsonValue v) { w->setValue(v.toInt()); }};
        };
        factories_["double"]=[this](const QJsonObject& p) {
            auto w=numeric(p); return Binding{w,nullptr,{},[w] { return QJsonValue(w->value()); },[w](QJsonValue v) { w->setValue(v.toDouble()); }};
        };
        factories_["string"]=[this](const QJsonObject& p) {
            if(p["multiline"].toBool()) {
                auto w=new QPlainTextEdit; w->setMaximumHeight(100); connect(w,&QPlainTextEdit::textChanged,this,[this] { refresh(); });
                return Binding{w,nullptr,{},[w] { return QJsonValue(w->toPlainText()); },[w](QJsonValue v) { w->setPlainText(v.toString()); }};
            }
            auto w=new QLineEdit; w->setMaxLength(p["maxLength"].toInt(32767));
            w->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
            connect(w,&QLineEdit::textChanged,this,[this] { refresh(); });
            return Binding{w,nullptr,{},[w] { return QJsonValue(w->text()); },[w](QJsonValue v) { w->setText(v.toString()); }};
        };
        factories_["enum"]=[this](const QJsonObject& p) {
            auto w=new QComboBox;
            w->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon); w->setMinimumContentsLength(4); w->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
            for(auto value:p["options"].toArray()) { auto o=value.toObject(); w->addItem(o["label"].toString(),o["value"].toVariant()); }
            connect(w,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this] { refresh(); });
            return Binding{w,nullptr,{},[w] { return QJsonValue::fromVariant(w->currentData()); },[w](QJsonValue v) { w->setCurrentIndex(w->findData(v.toVariant())); }};
        };
        factories_["range"]=[this](const QJsonObject& p) {
            auto w=new QWidget; auto row=new QHBoxLayout(w); row->setContentsMargins(0,0,0,0); row->setSpacing(6);
            auto a=numeric(p,p["valueType"]=="int"), b=numeric(p,p["valueType"]=="int");
            a->setAccessibleName(p["label"].toString()+QStringLiteral("，下限")); b->setAccessibleName(p["label"].toString()+QStringLiteral("，上限"));
            auto add=[&](QDoubleSpinBox* spin,const QString& label) {
                auto cell=new QWidget; auto column=new QVBoxLayout(cell); column->setContentsMargins(0,0,0,0); column->setSpacing(3);
                auto heading=new QLabel(label); heading->setProperty("role","muted"); heading->setBuddy(spin);
                spin->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed); column->addWidget(heading); column->addWidget(spin); row->addWidget(cell,1);
            };
            add(a,QStringLiteral("下限")); add(b,QStringLiteral("上限"));
            return Binding{w,nullptr,{},[a,b] { return QJsonValue(QJsonObject{{"lower",a->value()},{"upper",b->value()}}); },[a,b](QJsonValue v) { a->setValue(v.toObject()["lower"].toDouble()); b->setValue(v.toObject()["upper"].toDouble()); }};
        };
        factories_["vector3"]=[this](const QJsonObject& p) {
            auto w=new QWidget; auto row=new QFormLayout(w); row->setContentsMargins(0,0,0,0); row->setSpacing(6); row->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow); QVector<QDoubleSpinBox*> spins;
            for(auto axis:{"X","Y","Z"}) {auto spin=numeric(p); spin->setAccessibleName(p["label"].toString()+" "+axis); row->addRow(QString(axis),spin); spins<<spin;}
            return Binding{w,nullptr,{},[spins] { QJsonArray a; for(auto spin:spins) a.append(spin->value()); return QJsonValue(a); },[spins](QJsonValue v) { auto a=v.toArray(); for(int i=0;i<3;++i) spins[i]->setValue(a.at(i).toDouble()); }};
        };
        for(auto type:{"file","directory"}) factories_[type]=[this,type](const QJsonObject& p) {
            auto w=new QWidget; auto row=new QHBoxLayout(w); row->setContentsMargins(0,0,0,0); row->setSpacing(4); auto edit=new QLineEdit; auto button=new QPushButton(QStringLiteral("…"));
            edit->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed); button->setFixedWidth(fontMetrics().height()+22); button->setToolTip(QStringLiteral("浏览文件或目录"));
            edit->setAccessibleName(p["label"].toString()); button->setAccessibleName(QStringLiteral("浏览：")+p["label"].toString());
            row->addWidget(edit,1); row->addWidget(button); connect(edit,&QLineEdit::textChanged,this,[this] { refresh(); });
            connect(button,&QPushButton::clicked,this,[this,edit,p,type] {
                QString path;
                if(QString(type)=="directory") path=QFileDialog::getExistingDirectory(this,QStringLiteral("选择目录"),edit->text());
                else if(p["mustExist"].toBool()) path=QFileDialog::getOpenFileName(this,QStringLiteral("选择文件"),edit->text(),p["filter"].toString());
                else path=QFileDialog::getSaveFileName(this,QStringLiteral("选择文件"),edit->text(),p["filter"].toString());
                if(!path.isEmpty()) edit->setText(path);
            });
            return Binding{w,nullptr,{},[edit] { return QJsonValue(edit->text()); },[edit](QJsonValue v) { edit->setText(v.toString()); }};
        };
        factories_["color"]=[this](const QJsonObject&) {
            auto w=new QPushButton; auto color=std::make_shared<QColor>(Qt::green);
            w->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
            const auto update=[w,color] {w->setText(color->name(QColor::HexArgb)); QPixmap swatch(20,20); swatch.fill(*color); w->setIcon(QIcon(swatch));};
            connect(w,&QPushButton::clicked,this,[this,w,color,update] { auto selected=QColorDialog::getColor(*color,w,QStringLiteral("颜色"),QColorDialog::ShowAlphaChannel); if(selected.isValid()) { *color=selected; update(); refresh(); } });
            return Binding{w,nullptr,{},[color] { return QJsonValue(QJsonArray{color->red(),color->green(),color->blue(),color->alpha()}); },[color,update](QJsonValue v) { auto a=v.toArray(); *color=QColor(a.at(0).toInt(),a.at(1).toInt(),a.at(2).toInt(),a.at(3).toInt()); update(); }};
        };
    }
};
class ParameterPlugin final : public QObject, public IParameterUiPlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID FW_PARAMETERS_IID FILE "metadata.json")
    Q_INTERFACES(fw::IParameterUiPlugin)
public:
    IParameterEditor* createEditor(const QJsonObject& schema,QWidget* parent) override {
        if(!Parameters::validateSchema(schema).isEmpty()) return nullptr;
        return new Editor(schema,parent);
    }
    void destroyEditor(IParameterEditor* editor) override { delete editor; }
};
}
#include "ParameterUI.moc"
