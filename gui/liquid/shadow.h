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
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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
