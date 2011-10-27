#include "manager.h"

#include <common/cauv_node.h>
#include <debug/cauv_debug.h>

#include <generated/types/PipelineGroup.h>
#include <generated/types/Pl_GuiGroup.h>

#include "fNode.h"

using namespace cauv;
using namespace cauv::gui::f;

// - Templates Used Locally
template<typename message_T>
bool Manager::_nameMatches(boost::shared_ptr<const message_T> msg){
    if(msg->pipelineName() == m_pipeline_name)
        return true;
    return false;
}

// - General Public Implementation:
Manager::Manager(QGraphicsScene *scene, CauvNode *node, std::string const& pipeline_name)
    : QObject(),
      BufferedMessageObserver(),
      boost::enable_shared_from_this<Manager>(),
      m_scene(scene),
      m_cauv_node(node),
      m_nodes(),
      m_imgnodes(),
      m_pipeline_name(pipeline_name){
}

void Manager::init(){
    m_cauv_node->addMessageObserver(shared_from_this());
    m_cauv_node->joinGroup("pl_gui");
}

// - Message Observer Implementation:
void Manager::onGraphDescriptionMessage(GraphDescriptionMessage_ptr m){
    if(!_nameMatches(m)) return;

    debug(7) << BashColour::Green << "Manager::" << __func__ << *m;

    typedef std::map<node_id_t, NodeType::e> node_type_map_t;
    typedef std::map<node_id_t, FNode::msg_node_input_map_t> node_input_map_t;
    typedef std::map<node_id_t, FNode::msg_node_output_map_t> node_output_map_t;
    typedef std::map<node_id_t, FNode::msg_node_param_map_t> node_param_map_t;
    
    // remove nodes that shouldn't exist
    node_id_map_t::right_iterator i;
    bool inconsistency_detected = false;
    std::vector<node_id_t> nodes_for_removal;
    for(i = m_nodes.right.begin(); i != m_nodes.right.end(); i++){
        const node_type_map_t::const_iterator j = m->nodeTypes().find(i->first);
        if(j == m->nodeTypes().end())
            nodes_for_removal.push_back(i->first);
        else if(j->second != i->second->type())
            inconsistency_detected = true;
    }
    if(!inconsistency_detected){
        foreach(node_id_t id, nodes_for_removal)
            removeNode(id);
    
        // !!! TODO: remove arcs that shouldn't exist
        warning() << "removing dead arcs is not yet implemented";
    }

    if(inconsistency_detected)
        clearNodes();
    
    // destructive operations on nodes and arcs are complete, now make sure all
    // nodes exist with the right inputs and outputs:
    

    
}

void Manager::onArcAddedMessage(ArcAddedMessage_ptr m){
    if(!_nameMatches(m)) return;
}

// - Slot Implementations
void Manager::requestArc(NodeOutput from, NodeInput to){
}

void Manager::requestRemoveNode(node_id_t id){
}

// - General Protected Implementations
void Manager::removeNode(node_id_t id){
    node_id_map_t::right_iterator i = m_nodes.right.find(id);
    if(i != m_nodes.right.end()){
        i->second->close();
        m_nodes.right.erase(i);
    }else{
        error() << "no such node:" << id;
    }
}

void Manager::addNode(node_id_t id){
    node_id_map_t::right_const_iterator i = m_nodes.right.find(id);
    if(i == m_nodes.right.end()){
        m_nodes.insert(node_id_map_t::value_type(new FNode(*this, id), id));
    }else{
        error() << "node already exists:" << id;
    }
}

void Manager::clearNodes(){
    node_id_map_t::right_iterator i;
    for(i = m_nodes.right.begin(); i != m_nodes.right.end(); i++)
        i->second->close();
    m_nodes.clear();
}

