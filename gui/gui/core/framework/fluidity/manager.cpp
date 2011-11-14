/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "manager.h"

#include <common/cauv_node.h>
#include <debug/cauv_debug.h>

#include <generated/types/PipelineGroup.h>
#include <generated/types/Pl_GuiGroup.h>

#include "fNode.h"
#include "imgNode.h"

using namespace cauv;
using namespace cauv::gui::f;

// - Templates Used Locally
template<typename message_T>
bool Manager::_nameMatches(boost::shared_ptr<const message_T> msg){
    if(msg->pipelineName() == m_pipeline_name)
        return true;
    return false;
}

// - Static helper functions
static bool isImageNode(NodeType::e t){
    return t == NodeType::GuiOutput;
}

static bool isInvalid(NodeType::e t){
    return t == NodeType::Invalid;
}

static bool isInvalid(node_id_t const& id){
    return id == 0;
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
    node_type_map_t::const_iterator j;

    bool inconsistency_detected = false;
    std::vector<node_id_t> nodes_for_removal;
    for(i = m_nodes.right.begin(); i != m_nodes.right.end(); i++){
        j = m->nodeTypes().find(i->first);
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
    
    for(j = m->nodeTypes().begin(); j != m->nodeTypes().end(); j++){
        const node_id_t id = j->first;
        const NodeType::e type = j->second;
        i = m_nodes.right.find(id); 
        if(i == m_nodes.right.end()){
            addNode(type, id);
            i = m_nodes.right.find(j->first);
        }
        if(i == m_nodes.right.end())
            continue;
        fnode_ptr node = i->second;
        
        node_input_map_t::const_iterator inputs_it = m->nodeInputs().find(id);
        node_output_map_t::const_iterator outputs_it = m->nodeOutputs().find(id);
        node_param_map_t::const_iterator params_it = m->nodeParams().find(id);

        if(inputs_it == m->nodeInputs().end() ||
           outputs_it == m->nodeOutputs().end() ||
           params_it == m->nodeParams().end()){
            error() << "inconsistent graph description for node" << id;
            continue;
        }
        
        node->setInputs(inputs_it->second);
        node->setOutputs(outputs_it->second);
        node->setParams(params_it->second);
    }

    for(j = m->nodeTypes().begin(); j != m->nodeTypes().end(); j++){
        const node_id_t id = j->first;
        i = m_nodes.right.find(id);
        fnode_ptr node = i->second;
        
        node_input_map_t::const_iterator inputs_it = m->nodeInputs().find(id);
        node_output_map_t::const_iterator outputs_it = m->nodeOutputs().find(id);

        node->setInputLinks(inputs_it->second);
        node->setOutputLinks(outputs_it->second);
    }
}

void Manager::onNodeParametersMessage(NodeParametersMessage_ptr m){
    if(!_nameMatches(m)) return;
    node_id_map_t::right_iterator i = m_nodes.right.find(m->nodeId());
    if(i != m_nodes.right.end())
        i->second->setParams(m->params());
    else
        warning() << "unknown node" << m->nodeId();
}

void Manager::onNodeAddedMessage(NodeAddedMessage_ptr m){
    if(!_nameMatches(m)) return;
    addNode(m);
}

void Manager::onNodeRemovedMessage(NodeRemovedMessage_ptr m){
    if(!_nameMatches(m)) return;
    removeNode(m->nodeId());
}

void Manager::onArcAddedMessage(ArcAddedMessage_ptr m){
    if(!_nameMatches(m)) return;
}

void Manager::onArcRemovedMessage(ArcRemovedMessage_ptr m){
    if(!_nameMatches(m)) return;
}

// - Slot Implementations
void Manager::requestArc(NodeOutput from, NodeInput to){
}

void Manager::requestRemoveArc(NodeOutput from, NodeInput to){
}

void Manager::requestNode(NodeType::e const& type){
}

void Manager::requestRemoveNode(node_id_t const& id){
}

// - General Protected Implementations
void Manager::removeNode(node_id_t const& id){
    node_id_map_t::right_iterator i = m_nodes.right.find(id);
    if(i != m_nodes.right.end()){
        i->second->close();
        m_nodes.right.erase(i);
        m_imgnodes.right.erase(id);
    }else{
        error() << "no such node:" << id;
    }
}

void Manager::addNode(NodeType::e const& type, node_id_t const& id){
    if(isInvalid(type)){
        warning() << "ignoring invalid node type" << type;
    }else if(isInvalid(id)){
        warning() << "ignoring invalid node id" << id;
    }else if(m_nodes.right.find(id) != m_nodes.right.end()){
        error() << "node" << id << "already exists";
    }else if(isImageNode(type)){
        imgnode_ptr p = new ImgNode(*this, id);
        m_nodes.insert(node_id_map_t::value_type(p, id));
        m_imgnodes.insert(imgnode_id_map_t::value_type(p, id));
    }else{
        m_nodes.insert(node_id_map_t::value_type(new FNode(*this, id), id));
    }
}

void Manager::addNode(NodeAddedMessage_ptr m){
    const NodeType::e type = m->nodeType();
    const node_id_t id = m->nodeId();
    if(isInvalid(type)){
        warning() << "ignoring invalid node type" << type;
    }else if(isInvalid(id)){
        warning() << "ignoring invalid node id" << id;
    }else if(m_nodes.right.find(id) != m_nodes.right.end()){
        error() << "node" << id << "already exists";
    }else if(isImageNode(type)){
        imgnode_ptr p = new ImgNode(*this, m);
        m_nodes.insert(node_id_map_t::value_type(p, id));
        m_imgnodes.insert(imgnode_id_map_t::value_type(p, id));
    }else{
        m_nodes.insert(node_id_map_t::value_type(new FNode(*this, m), id));
    }
}

void Manager::clearNodes(){
    node_id_map_t::right_iterator i;
    for(i = m_nodes.right.begin(); i != m_nodes.right.end(); i++)
        i->second->close();
    m_nodes.clear();
}

