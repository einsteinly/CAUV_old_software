/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

