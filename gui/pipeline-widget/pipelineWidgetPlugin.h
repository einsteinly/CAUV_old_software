#ifndef __PIPELINE_WIDGET_PLUGIN_H__
#define __PIPELINE_WIDGET_PLUGIN_H__

#include <QDesignerCustomWidgetInterface>

class PipelineWidgetPlugin: public QObject, public QDesignerCustomWidgetInterface{
    Q_OBJECT
    Q_INTERFACES(QDesignerCustomWidgetInterface)
    
    public:
        PipelineWidgetPlugin(QObject *parent = 0);
        bool isContainer() const;
        bool isInitialized() const;
        QIcon icon() const;
        QString domXml() const;
        QString group() const;
        QString includeFile() const;
        QString name() const;
        QString toolTip() const;
        QString whatsThis() const;
        QWidget *createWidget(QWidget *parent);
        void initialize(QDesignerFormEditorInterface *core);
    
    private:
        bool m_initialized;
};


#endif // ndef __PIPELINE_WIDGET_PLUGIN_H__
