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

#ifndef __CAUV_ELEMENT_MULTIARC_START_H__
#define __CAUV_ELEMENT_MULTIARC_START_H__

#include <QPen>

#include "fluidity/managedElement.h"

#include "multiArc.h"

namespace cauv{
namespace gui{

class MultiArcStart: public QGraphicsObject,
                     public ConnectableInterface,
                     public ManagedElement{
    Q_OBJECT
    public:
        MultiArcStart(ManagedElement const& m, liquid::ArcStyle const&);
        //~MultiArcStart(); // children deleted by ~QGraphicsItem

        MultiArc* arc() const;
        liquid::ArcStyle const& style() const;

   // QGraphicsItem required:
        virtual QRectF boundingRect() const;
        virtual void paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *opt,
                           QWidget *widget);

    // ConnectableInterface:
        virtual QGraphicsObject* asQGraphicsObject();
        virtual QPointF connectionPoint();

    Q_SIGNALS:
        void boundriesChanged();
        void disconnected();
    
    protected:
        MultiArc *m_arc;
        QGraphicsLineItem *m_front_line;
        QGraphicsLineItem *m_back_line;

        liquid::ArcStyle const& m_style;
};

} // namespace gui
} // namespace cauv


#endif // ndef __CAUV_ELEMENT_MULTIARC_START_H__
