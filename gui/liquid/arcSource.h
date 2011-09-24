#ifndef __LIQUID_ARC_SOURCE_H__
#define __LIQUID_ARC_SOURCE_H__

#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>

namespace liquid {

struct ArcStyle;
class ConnectionSink;

class ArcSource: public QGraphicsItem{
    public:
        ArcSource(ArcStyle const& of_style, 
                  void* sourceDelegate);
        
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
        virtual ConnectionSink* newArcEnd();

     private:
        // ...style, currently highlighted scene item, connection source (which
        // may be this), ...
        ArcStyle const& m_style;
        void * m_sourceDelegate;
};

} // namespace liquid

#endif // __LIQUID_ARC_SOURCE_H__

