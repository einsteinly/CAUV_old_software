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

namespace cauv{
namespace gui{

class FNodeOutput: public QGraphicsWidget{
    public:
        FNodeOutput(void* sourceDelegate)
            : QGraphicsWidget(){
            QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(
                Qt::Horizontal, this
            );
            hlayout->setSpacing(0);
            hlayout->setContentsMargins(0,0,0,0);
            hlayout->addStretch(1);
            liquid::ArcSource *s = new liquid::ArcSource(
                sourceDelegate, new liquid::Arc(Image_Arc_Style)
            );
            s->setParentItem(this);
            hlayout->addItem(s);
            setLayout(hlayout);
        }
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget=0){
            Q_UNUSED(option);
            Q_UNUSED(widget);
            painter->setPen(QPen(QColor(20,20,200,64)));
            painter->setBrush(Qt::NoBrush);
            painter->drawRect(boundingRect());
        }
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_OUTPUT_H__

