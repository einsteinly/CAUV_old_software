#ifndef ARC_H
#define ARC_H

#include <QObject>
#include <QGraphicsPathItem>

namespace cauv {
    namespace gui {

        class JoiningArc : public QObject, public QGraphicsPathItem {
            Q_OBJECT
        public:
            JoiningArc(QGraphicsObject * from, QGraphicsObject * to);

        protected Q_SLOTS:
            void updatePosition();

        protected:
            QGraphicsItem * m_to;
            QGraphicsItem * m_from;
        };

    } // namespace gui
} // namespace gui


#endif // ARC_H
