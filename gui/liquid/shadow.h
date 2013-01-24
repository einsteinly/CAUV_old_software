/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */



#ifndef __LIQUID_SHADOW_H__
#define __LIQUID_SHADOW_H__

#include <QGraphicsObject>
#include <QBrush>

namespace liquid{

class Shadow: public QGraphicsObject{
    Q_OBJECT

    public:
        Shadow(QGraphicsItem* parent);

        virtual QRectF boundingRect() const;
        virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget=0);
    
    public Q_SLOTS:
        void setShape(QPainterPath shape);
        void setBrush(QBrush brush);
        
    private:
        QGraphicsPathItem *m_shape;
        bool m_blur_effect_applied;
};

} // namespace liquid

#endif // __LIQUID_SHADOW_H__
