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
        MultiArcStart();
        //~MultiArcStart(); // children deleted by ~QGraphicsItem

        MultiArc* arc();

   // QGraphicsItem required:
        QRectF boundingRect() const;
        void paint(QPainter *painter,
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
        QGraphicsLineItem *m_line;
};

} // namespace gui
} // namespace cauv


#endif // ndef __CAUV_ELEMENT_MULTIARC_START_H__
