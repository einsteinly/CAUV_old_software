#ifndef __CAUV_ELEMENT_MULTIARC_H__
#define __CAUV_ELEMENT_MULTIARC_H__

#include <QObject>
#include <QGraphicsPathItem>

#include "arc.h"

namespace cauv{
namespace gui{

class MultiArcEnd;

class MultiArc : public QObject, public QGraphicsPathItem {
        Q_OBJECT
    public: 
        // From is used as parent. Magics are done so that coordinates are all
        // relative to parent for drawing.
        MultiArc(ConnectableInterface *from, ConnectableInterface *to=NULL);
        
        // so that start/end connectors can blend into the arc
        QColor startColour();
        QColor endColour();

        void addTo(ConnectableInterface *to);
        void removeTo(ConnectableInterface *to);
        
        void mousePressEvent(QGraphicsSceneMouseEvent *event);
        void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
        void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

    protected Q_SLOTS:
        void updateLayout();

    protected:
        QList<ConnectableInterface*> m_to;
        ConnectableInterface *m_from;
        MultiArcEnd* m_ephemeral_arc_end;
};

} // namespace gui
} // namespace cauv


#endif // ndef __CAUV_ELEMENT_MULTIARC_H__
