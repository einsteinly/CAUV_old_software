#ifndef __CAUV_ELEMENT_MULTIARC_START_H__
#define __CAUV_ELEMENT_MULTIARC_START_H__

#include <QPen>

#include "multiArc.h"

namespace cauv{
namespace gui{

/*
 *  o = origin (0,0)
 *  + = connection point
 *  L = Length
 *  T = Thickness
 *
 *      <----L---->
 *   -  o---------| -  -  -  -> x
 *  T|  | | || |||+ arc connects here
 *   -  |---------| - - - -
 *      |
 *      y
 *
 */
class MultiArcStart: public QGraphicsObject,
                     public ConnectableInterface{
    Q_OBJECT
    private:
        const static qreal Length;
        const static qreal Thickness;
        const static QColor Lead_In_Colour;

    public:
        MultiArcStart()
            : m_arc(), m_line(){
            m_arc = new MultiArc(this, NULL);
            m_line = new QGraphicsLineItem(0, Thickness/2, Length, Thickness/2, this);
            
            QLinearGradient line_gradient(QPointF(0,Thickness/2), connectionPoint());
            line_gradient.setColorAt(0, Lead_In_Colour);
            line_gradient.setColorAt(1, m_arc->startColour());
            QBrush brush(line_gradient);
            
            m_line->setPen(QPen(brush, Thickness));
            
            setFlag(ItemIsMovable);
            connect(this, SIGNAL(xChanged()), this, SIGNAL(boundriesChanged()));
            connect(this, SIGNAL(yChanged()), this, SIGNAL(boundriesChanged()));
        }

        ~MultiArcStart(){
            // children deleted by ~QGraphicsItem
        }

        MultiArc* arc(){
            // !!! do not delete 'this' while m_arc (a child of this) is being
            // used
            return m_arc;
        }

   // QGraphicsItem required:
        QRectF boundingRect() const{
            return QRectF(0, 0, Length, Thickness);
        }
        void paint(QPainter *painter,
                   const QStyleOptionGraphicsItem *opt,
                   QWidget *widget){
            Q_UNUSED(opt);
            Q_UNUSED(widget);
            Q_UNUSED(painter);
            // drawing is all done by children
        }
        

    // ConnectableInterface:
        virtual QGraphicsObject *asQGraphicsObject(){
            return this;
        }
        virtual QPointF connectionPoint(){
            return QPointF(Length, Thickness/2);
        }

    Q_SIGNALS:
        void boundriesChanged();
        void disconnected();
    
    protected:
        MultiArc *m_arc;
        QGraphicsLineItem *m_line;
};

} // namespace gui
} // namespace cauv


#endif // ndef __CAUV_ELEMENT_MULTIARC_START_H__
