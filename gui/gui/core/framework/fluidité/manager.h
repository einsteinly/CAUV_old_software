#ifndef __CAUV_F_MANAGER_H__
#define __CAUV_F_MANAGER_H__

#include <QObject>

#include <boost/enable_shared_from_this.hpp>

#include <common/cauv_node.h>
#include <generated/message_observers.h>

class QGraphicsScene;

namespace cauv{

// do we have a message types forward declaration header?
//class NodeOutput;
//class NodeInput;

namespace gui{

class Manager: public QObject,
               public BufferedMessageObserver,
               public boost::enable_shared_from_this<Manager>{
    Q_OBJECT
    public:
        Manager(QGraphicsScene *scene, CauvNode *node);

    public Q_SLOTS:
        // Update propagation:
        //
        // 1) GUI element emits arcRequested in response to drag & drop, or
        //    something
        //
        // 2) slot requestArc called, sends AddArc message, fills in pipeline
        //    name
        //
        // 3) onArcAddedMessage
        //
        //void requestArc(NodeOutput const& from, NodeInput const& to);

    protected:
    
    protected Q_SLOTS:

    protected:
        QGraphicsScene *m_scene;
        CauvNode       *m_node;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_MANAGER_H__

