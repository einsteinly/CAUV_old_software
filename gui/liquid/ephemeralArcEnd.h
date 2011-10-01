#ifndef __LIQUID_EPHEMERAL_ARC_END__
#define __LIQUID_EPHEMERAL_ARC_END__

#include "arcSink.h"

namespace liquid{

struct ArcStyle;

class EphemeralArcEnd: public AbstractArcSink{
    Q_OBJECT
    public:
        EphemeralArcEnd(ArcStyle const& of_style);
        virtual ~EphemeralArcEnd();
    
        // these are forwarded from AbstractArcSink: the event object is
        // in the *parent* coordinate system
        void mousePressEvent(QGraphicsSceneMouseEvent *e); 
        void mouseMoveEvent(QGraphicsSceneMouseEvent *e); 
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *e); 

        QRectF boundingRect() const; 
        void paint(QPainter *painter,
                   const QStyleOptionGraphicsItem *opt,
                   QWidget *widget);
        
        virtual bool willAcceptConnection(void*){return false;}
        virtual void doPresentHighlight(qreal){}
        virtual ConnectionStatus doAcceptConnection(void*){return Rejected;}

    Q_SIGNALS:
        void disconnected(AbstractArcSink*);

    protected Q_SLOTS:
        void removeFromScene();

    protected:
        void setFill(bool pressed);
        
        QGraphicsPolygonItem *m_back_poly;
        QGraphicsPolygonItem *m_front_poly;
        
        ArcStyle const& m_style;
};

} // namespace liquid

#endif // ndef __LIQUID_EPHEMERAL_ARC_END__
