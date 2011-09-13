#ifndef __CAUV_ELEMENT_ARC_H__
#define __CAUV_ELEMENT_ARC_H__

#include <QObject>
#include <QGraphicsPathItem>

namespace cauv {
    namespace gui {

        struct ConnectableInterface {
            // return the item this arc is pointing to
            virtual QGraphicsObject * asQGraphicsObject() = 0;

            // point (in this's coordinates) that arcs should connect to
            virtual QPointF connectionPoint(){
                return QPointF(0,0);
            }

            // Q_SIGNALS:
            // must be implemented as signals
            virtual void boundriesChanged() = 0;
            virtual void disconnected() = 0;
        };

        class ConnectingArc : public QObject, public QGraphicsPathItem {
            Q_OBJECT
        public:
            ConnectingArc(ConnectableInterface * from, ConnectableInterface * to);

        protected Q_SLOTS:
            void updatePosition();

        protected:
            QGraphicsObject * m_to;
            QGraphicsObject * m_from;
        };

    } // namespace gui
} // namespace gui


#endif // __CAUV_ELEMENT_ARC_H__
