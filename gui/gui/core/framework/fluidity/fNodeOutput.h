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

#include "elements/style.h"

#include "fNodeIO.h"

namespace cauv{
namespace gui{
namespace f{

// !!! TODO: note to self: this is far from the final class structure for node
// outputs, quickly hacked together while working on inputs... 
class FNodeOutput: public QGraphicsWidget,
                   public FNodeIO,
                   public liquid::ArcSourceDelegate{
    public:
        FNodeOutput(FNode* node)
            : QGraphicsWidget(),
              FNodeIO(node){
            QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(
                Qt::Horizontal, this
            );
            hlayout->setSpacing(0);
            hlayout->setContentsMargins(0,0,0,0);
            hlayout->addStretch(1);
            liquid::ArcSource *s = new liquid::ArcSource(
                this, new liquid::Arc(Image_Arc_Style)
            );
            s->setParentItem(this);
            hlayout->addItem(s);
            setLayout(hlayout);
        }
        virtual ~FNodeOutput(){}

        virtual OutputType::e ioType() const{
            return OutputType::Image;
        }

        virtual SubType subType() const{
            return -1;
        }

        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget=0){
            Q_UNUSED(option);
            Q_UNUSED(widget);
            painter->setPen(QPen(QColor(20,20,200,64)));
            painter->setBrush(Qt::NoBrush);
            painter->drawRect(boundingRect());
        }

    private:
        
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_OUTPUT_H__

