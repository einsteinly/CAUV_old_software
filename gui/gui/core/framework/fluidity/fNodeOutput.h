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

#include <generated/types/LocalNodeOutput.h>

#include "elements/style.h"

#include "fNodeIO.h"
#include "fNode.h"

#include <debug/cauv_debug.h>

namespace cauv{
namespace gui{
namespace f{

class FNodeOutput: public QGraphicsWidget,
                   public FNodeIO,
                   public liquid::ArcSourceDelegate{
    Q_OBJECT
    public:
        FNodeOutput(FNode* node, liquid::ArcStyle const& arc_style, std::string const& id)
            : QGraphicsWidget(node),
              FNodeIO(node, id),
              m_source(NULL),
              m_text(NULL){
            QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(
                Qt::Horizontal, this
            );
            hlayout->setSpacing(0);
            hlayout->setContentsMargins(0,0,0,0);
            
            hlayout->addStretch(1);

            liquid::LiquidLabel* text_label = new liquid::LiquidLabel(QString::fromStdString(id));
            text_label->setFont(F_Node_Style.text.font);

            m_text = new QGraphicsProxyWidget();
            m_text->setWidget(text_label);
            hlayout->addItem(m_text);
            hlayout->setAlignment(m_text, Qt::AlignVCenter | Qt::AlignRight);
            
            hlayout->setItemSpacing(1, 4.0);

            m_source = new liquid::ArcSource(
                this, new liquid::Arc(arc_style)
            );
            m_source->setParentItem(this);
            m_source->setZValue(10);
            hlayout->addItem(m_source);
            hlayout->setAlignment(m_source, Qt::AlignVCenter | Qt::AlignRight);            

            setLayout(hlayout);
            
            connect(node, SIGNAL(xChanged()), this, SIGNAL(xChanged()));
            connect(node, SIGNAL(yChanged()), this, SIGNAL(yChanged()));
        }
        virtual ~FNodeOutput(){}

        virtual OutputType::e ioType() const = 0;

        virtual SubType subType() const{
            return -1;
        }

        liquid::Arc* arc() const{
            return m_source->arc();
        }

    protected Q_SLOTS:
        void test(){
            debug() << "TEST SLOT";
        }

    protected:
        liquid::ArcSource* m_source;
        QGraphicsProxyWidget* m_text;
};

class FNodeImageOutput: public FNodeOutput{
    public:
        FNodeImageOutput(LocalNodeOutput const& output, FNode* node)
            : FNodeOutput(node, Image_Arc_Style, output.output){
        }

        virtual OutputType::e ioType() const{
            return OutputType::Image;
        }

        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget=0){
            Q_UNUSED(option);
            Q_UNUSED(widget);
            painter->setPen(QPen(QColor(20,20,200,64)));
            painter->setBrush(Qt::NoBrush);
            painter->drawRect(boundingRect());
        }
};

class FNodeParamOutput: public FNodeOutput{
    public:
        FNodeParamOutput(LocalNodeOutput const& output, FNode* node)
            : FNodeOutput(node, Param_Arc_Style, output.output), m_subType(output.subType){
            
        }

        virtual OutputType::e ioType() const{
            return OutputType::Parameter;
        }
        virtual SubType subType() const{
            return m_subType;
        }

        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget=0){
            Q_UNUSED(option);
            Q_UNUSED(widget);
            painter->setPen(QPen(QColor(200,20,30,64)));
            painter->setBrush(Qt::NoBrush);
            painter->drawRect(boundingRect());
        }
    private:
        SubType m_subType;
};


} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_OUTPUT_H__

