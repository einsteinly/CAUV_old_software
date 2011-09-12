#ifndef __CAUV_ELEMENT_MULTIARC_H__
#define __CAUV_ELEMENT_MULTIARC_H__

#include <QObject>
#include <QGraphicsPathItem>

#include "arc.h"

namespace cauv{
namespace gui{

class MultiArc : public QObject, public QGraphicsPathItem {
        Q_OBJECT
    public: 
        // from and to must be in the same coordinate system as this (ie, same
        // parent, or scene parent)
        MultiArc(ConnectableInterface *from, ConnectableInterface *to=NULL);

        void addTo(ConnectableInterface *to);
        //void removeTo(ConnectableInterface *to);

    protected Q_SLOTS:
        void updateLayout();

    protected:
        QList<QGraphicsObject*> m_to;
        QGraphicsObject * m_from;
};

} // namespace gui
} // namespace cauv


#endif // ndef __CAUV_ELEMENT_MULTIARC_H__
