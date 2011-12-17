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

#include <liquid/arcSource.h>
#include <liquid/arc.h>

#include <generated/types/LocalNodeOutput.h>

#include "elements/style.h"

#include "fNodeIO.h"
#include "fNode.h"

#include <debug/cauv_debug.h>

namespace cauv{
namespace gui{
namespace f{

// !!! TODO: note to self: this is far from the final class structure for node
// outputs, quickly hacked together while working on inputs... 
class FNodeOutput: public QGraphicsWidget,
                   public FNodeIO,
                   public liquid::ArcSourceDelegate{
    Q_OBJECT
    public:
        FNodeOutput(FNode* node)
            : QGraphicsWidget(),
              FNodeIO(node),
              m_source(NULL){
            QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(
                Qt::Horizontal, this
            );
            hlayout->setSpacing(0);
            hlayout->setContentsMargins(0,0,0,0);
            hlayout->addStretch(1);
            /*m_pending_source = new liquid::ArcSource(
                this, new liquid::Arc(Pending_Arc_Style)
            );
            m_pending_source->setParentItem(this);
            hlayout->addItem(m_pending_source);*/
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
        /*liquid::Arc* pendingArc() const{
            return m_pending_source->arc();
        }*/

        QGraphicsLinearLayout* layout(){
            return dynamic_cast<QGraphicsLinearLayout*>(QGraphicsWidget::layout());
        }

    protected Q_SLOTS:
        void test(){
            debug() << "TEST SLOT";
        }

    protected:
        liquid::ArcSource* m_source;
        //liquid::ArcSource* m_pending_source;
};

class FNodeImageOutput: public FNodeOutput{
    public:
        FNodeImageOutput(LocalNodeOutput const& output, FNode* node)
            : FNodeOutput(node){
            m_source = new liquid::ArcSource(
                this, new liquid::Arc(Image_Arc_Style)
            );
            m_source->setParentItem(this);
            layout()->addItem(m_source);
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
            : FNodeOutput(node), m_subType(output.subType){
            m_source = new liquid::ArcSource(
                this, new liquid::Arc(Param_Arc_Style)
            );
            m_source->setParentItem(this);
            layout()->addItem(m_source);
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

