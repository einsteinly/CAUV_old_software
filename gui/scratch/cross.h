#ifndef __CAUV_GUI_SCRATCH_CROSS_H__
#define __CAUV_GUI_SCRATCH_CROSS_H__

#include <QPainter>

#include "elements/arc.h"

namespace cauv{
namespace gui{

class Cross: public QGraphicsObject,
             public ConnectableInterface{
    Q_OBJECT

    public:
        Cross(qreal size)
         : QGraphicsObject(),
           ConnectableInterface(),
           m_size(size),
           m_line_a(new QGraphicsLineItem(2-size/2, size/2-2, size/2-2, 2-size/2, this)),
           m_line_b(new QGraphicsLineItem(2-size/2, 2-size/2, size/2-2, size/2-2, this)){
            QPen p(QColor(0,0,0,255));
            setPen(p);
            setFlag(ItemIsMovable);
            connect(this, SIGNAL(xChanged()), this, SIGNAL(boundriesChanged()));
            connect(this, SIGNAL(yChanged()), this, SIGNAL(boundriesChanged()));
        }

        void setPen(const QPen &pen){
            m_line_a->setPen(pen);
            m_line_b->setPen(pen);
        }
    
    // QGraphicsItem required:
        QRectF boundingRect() const{
            return QRectF(-m_size/2, -m_size/2, m_size, m_size);
        } 
        void paint(QPainter *painter,
                   const QStyleOptionGraphicsItem *opt,
                   QWidget *widget){
            Q_UNUSED(opt);
            Q_UNUSED(widget);
            painter->setBrush(QBrush(QColor(0,0,0,32)));
            painter->setPen(QPen(QColor(0,0,0,64)));
            painter->drawRoundedRect(1-m_size/2, 1-m_size/2, m_size-2, m_size-2, 2, 2);
        }
       

    // ConnectableInterface:
        virtual QGraphicsObject * asQGraphicsObject(){
            return this;
        }
    Q_SIGNALS:
        void boundriesChanged();
        void disconnected();
    
    protected:
        qreal m_size;
        QGraphicsLineItem *m_line_a;
        QGraphicsLineItem *m_line_b;
};

} // namespace gui
} // namespace cauv

#endif //ndef __CAUV_GUI_SCRATCH_CROSS_H__
