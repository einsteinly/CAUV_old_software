/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

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
