#ifndef __CAUV_ELEMENT_MULTIARC_START_H__
#define __CAUV_ELEMENT_MULTIARC_START_H__

#include <QPen>

#include "multiArc.h"

namespace cauv{
namespace gui{

class MultiArcStart: public QGraphicsObject,
                     public ConnectableInterface{
    Q_OBJECT
    public:
        MultiArcStart(ArcStyle const&);
        //~MultiArcStart(); // children deleted by ~QGraphicsItem

        MultiArc* arc() const;
        ArcStyle const& style() const;

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

        ArcStyle const& m_style;
};

} // namespace gui
} // namespace cauv


#endif // ndef __CAUV_ELEMENT_MULTIARC_START_H__
