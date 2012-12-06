/* Copyright 2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Steven Ogborne  steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __LIQUID_INTERACTION_AREA__
#define __LIQUID_INTERACTION_AREA__

#include <QGraphicsItem>

namespace liquid{

class InteractionArea: public QGraphicsItem{
    public:
        InteractionArea(QGraphicsItem* delegate, float add_border);

        void freezeDelegateShape();

        // these are forwarded to the delegate
        void mousePressEvent(QGraphicsSceneMouseEvent *e); 
        void mouseMoveEvent(QGraphicsSceneMouseEvent *e); 
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *e); 
        
        
        virtual QPainterPath shape() const;
        virtual QRectF boundingRect() const; 
        virtual void paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *opt,
                           QWidget *widget);
        virtual bool contains(QPointF const& point) const;

    private:
        QPainterPath m_outline;
        
        float m_add_border;
        QGraphicsItem* m_delegate;
};

} // namespace liquid

#endif // ndef __LIQUID_INTERACTION_AREA__

