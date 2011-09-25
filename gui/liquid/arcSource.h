#ifndef __LIQUID_ARC_SOURCE_H__
#define __LIQUID_ARC_SOURCE_H__

#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>

namespace liquid {

struct ArcStyle;
class ConnectionSink;

class AbstractArcSource: public QGraphicsObject{
    Q_OBJECT
    public:
        AbstractArcSource(ArcStyle const& of_style, 
                          void* sourceDelegate,
                          Arc* arc);
        
        Arc* arc() const;
        ArcStyle const& style() const;
    
    public Q_SIGNALS:
        void geometryChanged();

    protected:
        // !!! not implementing the GraphicsItem required functions: this is an
        // abstract base

        // Handles creating new arcs by dragging:
        virtual void mousePressEvent(QGraphicsSceneMouseEvent *e);
        virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *e);
        virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *e);
        
        // used to create the temporary ConnectionSink* to which the arc is
        // connected during a drag operation: this may be overriden by derived
        // classes if they want to drastically change the style
        virtual QGraphicsItem* newArcEnd();

     protected:
        // ...style, currently highlighted scene item, connection source (which
        // may be this), ...
        ArcStyle const& m_style;  
        Arc *m_arc;
        void *m_sourceDelegate;
        ArcSink *m_ephemeral_sink;
};

class ArcSource: public AbstractArcSource,
                 public QGraphicsLayoutItem{
    public:
        ArcSource(ArcStyle const& of_style,
                  void* sourceDelegate);

    protected:
        virtual QRectF boundingRect() const;
        virtual void paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *opt,
                           QWidget *widget=0);

    private:
        QGraphicsLineItem *m_front_line;
        QGraphicsLineItem *m_back_line;
};

} // namespace liquid

#endif // __LIQUID_ARC_SOURCE_H__

