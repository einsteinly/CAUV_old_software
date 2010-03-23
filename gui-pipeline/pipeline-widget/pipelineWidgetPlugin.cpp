#include "pipelineWidgetPlugin.h"
#include "pipelineWidget.h"

#include <QtPlugin>

Q_EXPORT_PLUGIN2(customwidgetplugin, PipelineWidgetPlugin)

PipelineWidgetPlugin::PipelineWidgetPlugin(QObject *parent)
    : QObject(parent), m_initialized(false){
}

void PipelineWidgetPlugin::initialize(QDesignerFormEditorInterface * /*core*/){
    if (m_initialized)
        return;
    
    // ...
    
    m_initialized = true;
}

QWidget *PipelineWidgetPlugin::createWidget(QWidget *parent){
    return new PipelineWidget(parent);
}

QString PipelineWidgetPlugin::name() const{
    return "PipelineWidget";
}

QString PipelineWidgetPlugin::group() const{
    return "CAUV Widgets";
}

QIcon PipelineWidgetPlugin::icon() const{
    return QIcon();
}

QString PipelineWidgetPlugin::toolTip() const{
    return "";
}

QString PipelineWidgetPlugin::whatsThis() const{
    return "";
}

bool PipelineWidgetPlugin::isContainer() const{
    return false;
}

bool PipelineWidgetPlugin::isInitialized() const{
    return m_initialized;
}

QString PipelineWidgetPlugin::domXml() const
{
    return
    "<widget class=\"PipelineWidget\" name=\"pipelineWidget\">\n"
    " <property name=\"geometry\">\n"
    "  <rect>\n"
    "   <x>0</x>\n"
    "   <y>0</y>\n"
    "   <width>200</width>\n"
    "   <height>200</height>\n"
    "  </rect>\n"
    " </property>\n"
    "</widget>\n";
}

QString PipelineWidgetPlugin::includeFile() const
{
    return "pipelineWidget.h";
}

