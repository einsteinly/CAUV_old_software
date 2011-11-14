#ifndef __CAUV_F_MANAGER_H__
#define __CAUV_F_MANAGER_H__

#include <QObject>

#include <boost/enable_shared_from_this.hpp>
#include <boost/bimap.hpp>
#include <boost/shared_ptr.hpp>

#include <generated/message_observers.h>
#include <generated/types/NodeOutput.h>
#include <generated/types/NodeInput.h>
#include <generated/types/NodeType.h>

#include "fluidity/types.h"

class QGraphicsScene;

namespace cauv{

class CauvNode;

namespace gui{
namespace f{

class Manager: public QObject,
               public BufferedMessageObserver,
               public boost::enable_shared_from_this<Manager>{
    Q_OBJECT
    public:
        Manager(QGraphicsScene *scene, CauvNode *node, std::string const& pipeline_name);
        
        // a shared pointer to this must be held when this is called!
        void init();

        virtual void onGraphDescriptionMessage(GraphDescriptionMessage_ptr);
        virtual void onNodeParametersMessage(NodeParametersMessage_ptr);
        virtual void onNodeAddedMessage(NodeAddedMessage_ptr);
        virtual void onNodeRemovedMessage(NodeRemovedMessage_ptr);
        virtual void onArcAddedMessage(ArcAddedMessage_ptr);
        virtual void onArcRemovedMessage(ArcRemovedMessage_ptr);

    public Q_SLOTS:
        /* When an arc is added: 
         * 1) GUI element emits arcRequested in response to drag & drop, or
         *    something
         *
         * 2) slot requestArc called, sends AddArc message, fills in pipeline
         *    name
         *
         * 3) onArcAddedMessage
         *
         */
        void requestArc(NodeOutput from, NodeInput to);
        void requestRemoveArc(NodeOutput from, NodeInput to);
        void requestNode(NodeType::e const& type);
        void requestRemoveNode(node_id_t const& id);

    protected:
        void removeNode(node_id_t const& id);
        void addNode(NodeType::e const& type, node_id_t const& id);
        void addNode(NodeAddedMessage_ptr);
        void clearNodes();
    
    protected Q_SLOTS:


    private:
        template<typename message_T>
        bool _nameMatches(boost::shared_ptr<const message_T> msg);

    protected:
        QGraphicsScene *m_scene;
        CauvNode       *m_cauv_node;
        
        typedef boost::bimap<fnode_ptr, node_id_t> node_id_map_t;
        node_id_map_t m_nodes;
        typedef boost::bimap<imgnode_ptr, node_id_t> imgnode_id_map_t;
        imgnode_id_map_t m_imgnodes;

        std::string m_pipeline_name;

};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_MANAGER_H__

