#ifndef __CAUV_ELEMENT_MULTIARC_END_H__
#define __CAUV_ELEMENT_MULTIARC_END_H__

#include <QPen>

#include "fluidité/managedElement.h"

#include "arc.h"
#include "style.h"

namespace cauv{
namespace gui{

class MultiArc;
struct ArcStyle;

class MultiArcEnd: public QGraphicsObject,
                   public ConnectableInterface,
                   public ManagedElement{
    Q_OBJECT
    /*private:
        const static qreal Length;
        const static qreal Base_Thickness;
        const static qreal Tip_Thickness;
        const static QColor Tip_Colour;
        const static QColor Pressed_Colour;*/

    public:
        // arc is used as parent
        // if ephemeral, only live as long as dragged
        MultiArcEnd(MultiArc* arc, bool ephemeral = false); 
    
   // reimplement protected members to handle events forwarded from m_arc when
   // this is an ephemeral end point:
        void mousePressEvent(QGraphicsSceneMouseEvent *event);
        void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

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
        
    protected Q_SLOTS:
        void removeFromScene();
    
    protected:
        void setFill(bool pressed);

        MultiArc *m_arc;
        QGraphicsPolygonItem *m_back_poly;
        QGraphicsPolygonItem *m_front_poly;
        bool m_ephemeral;
        
        ArcStyle const& m_style;
};

} // namespace gui
} // namespace cauv


#endif // ndef __CAUV_ELEMENT_MULTIARC_END_H__

