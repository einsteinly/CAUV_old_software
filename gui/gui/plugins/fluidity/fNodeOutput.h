/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_FNODE_OUTPUT_H__
#define __CAUV_GUI_FNODE_OUTPUT_H__

#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>
#include <QPainter>
#include <QLabel>
#include <QGraphicsProxyWidget>
#include <QGraphicsScene>

#include <liquid/arcSource.h>
#include <liquid/arc.h>
#include <liquid/label.h>
#include <liquid/proxyWidget.h>

#include <common/pipeline_model/edge_model.h>

#include "elements/style.h"

#include "fNode.h"

#include <debug/cauv_debug.h>

namespace cauv{
namespace gui{
namespace f{

class FNodeOutput: //public pipeline_model::OutputModel,
                   public QGraphicsWidget,
                   public liquid::ArcSourceDelegate {
    Q_OBJECT
    public:
        FNodeOutput(FNode &node, liquid::ArcStyle const& arc_style, const std::string& id)
            : 
              QGraphicsWidget(&node),
              m_source(NULL),
              m_text(NULL){
            QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(
                Qt::Horizontal, this
            );
            hlayout->setSpacing(0);
            hlayout->setContentsMargins(0,0,0,0);
            
            hlayout->addStretch(1);

            liquid::LiquidLabel* text_label = new liquid::LiquidLabel(QString::fromStdString(id));
            text_label->setFont(F_Node_Style().text.font);

            m_text = new liquid::ProxyWidget();
            m_text->setWidget(text_label);
            hlayout->addItem(m_text);
            hlayout->setAlignment(m_text, Qt::AlignBottom | Qt::AlignRight);
            
            hlayout->setItemSpacing(1, 4.0);

            m_source = new liquid::ArcSource(
                this, new liquid::Arc(arc_style)
            );
            m_source->setParentItem(this);
            m_source->setZValue(10);
            hlayout->addItem(m_source);
            hlayout->setAlignment(m_source, Qt::AlignBottom | Qt::AlignRight);            

            setLayout(hlayout);
            
            connect(&node, SIGNAL(xChanged()), this, SIGNAL(xChanged()));
            connect(&node, SIGNAL(yChanged()), this, SIGNAL(yChanged()));
        }
        virtual ~FNodeOutput(){}

        liquid::Arc* arc() const{
            return m_source->arc();
        }

    protected:
        liquid::ArcSource* m_source;
        QGraphicsProxyWidget* m_text;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_OUTPUT_H__

