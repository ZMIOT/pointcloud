#pragma once
#include "Host.h"
#include <functional>
class QWidget;
namespace fw {
class IParameterEditor {
public:
    virtual ~IParameterEditor() = default;
    virtual QWidget* widget() = 0;
    virtual ParameterSet values() const = 0;
    virtual void setValues(const ParameterSet&) = 0;
    virtual void setAdvanced(bool) = 0;
    virtual QStringList errors() const = 0;
};
class IParameterUiPlugin {
public:
    virtual ~IParameterUiPlugin() = default;
    virtual IParameterEditor* createEditor(const QJsonObject&, QWidget* parent) = 0;
    virtual void destroyEditor(IParameterEditor*) = 0;
};
class ISceneView {
public:
    virtual ~ISceneView() = default;
    virtual QWidget* widget() = 0;
    virtual void setLayers(const QVector<DebugLayer>&) = 0;
    virtual void fit() = 0;
};
struct DebugControls { std::function<void()> next, resume, stop; };
class IVisualizationPlugin {
public:
    virtual ~IVisualizationPlugin() = default;
    virtual ISceneView* createView(QWidget*) = 0;
    virtual void destroyView(ISceneView*) = 0;
    virtual QWidget* showDebug(FramePtr, const DebugControls&, QWidget* parent) = 0;
};
class IMainUiPlugin {
public:
    virtual ~IMainUiPlugin() = default;
    virtual QWidget* createWindow(Host&) = 0;
};
}
#define FW_PARAMETERS_IID "org.pointcloud.framework.IParameterUiPlugin/1.0"
#define FW_VISUALIZATION_IID "org.pointcloud.framework.IVisualizationPlugin/1.0"
#define FW_MAIN_IID "org.pointcloud.framework.IMainUiPlugin/1.0"
Q_DECLARE_INTERFACE(fw::IParameterUiPlugin, FW_PARAMETERS_IID)
Q_DECLARE_INTERFACE(fw::IVisualizationPlugin, FW_VISUALIZATION_IID)
Q_DECLARE_INTERFACE(fw::IMainUiPlugin, FW_MAIN_IID)
