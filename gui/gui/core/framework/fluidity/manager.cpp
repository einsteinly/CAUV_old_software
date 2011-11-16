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

#include <QGraphicsScene>

#include <boost/make_shared.hpp>

#include <common/cauv_node.h>
#include <debug/cauv_debug.h>
#include <utility/bash_cout.h>

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
    qRegisterMetaType<GraphDescriptionMessage_ptr>("GraphDescriptionMessage_ptr");
    qRegisterMetaType<NodeParametersMessage_ptr>("NodeParametersMessage_ptr");
    qRegisterMetaType<NodeAddedMessage_ptr>("NodeAddedMessage_ptr");
    qRegisterMetaType<NodeRemovedMessage_ptr>("NodeRemovedMessage_ptr");
    qRegisterMetaType<ArcAddedMessage_ptr>("ArcAddedMessage_ptr");
    qRegisterMetaType<ArcRemovedMessage_ptr>("ArcRemovedMessage_ptr");
    connect(this, SIGNAL(receivedGraphDescription(GraphDescriptionMessage_ptr)),
            this, SLOT(onGraphDescription(GraphDescriptionMessage_ptr)));
    connect(this, SIGNAL(receivedNodeParameters(NodeParametersMessage_ptr)),
            this, SLOT(onNodeParameters(NodeParametersMessage_ptr)));
    connect(this, SIGNAL(receivedNodeAdded(NodeAddedMessage_ptr)),
            this, SLOT(onNodeAdded(NodeAddedMessage_ptr)));
    connect(this, SIGNAL(receivedNodeRemoved(NodeRemovedMessage_ptr)),
            this, SLOT(onNodeRemoved(NodeRemovedMessage_ptr)));
    connect(this, SIGNAL(receivedArcAdded(ArcAddedMessage_ptr)),
            this, SLOT(onArcAdded(ArcAddedMessage_ptr)));
    connect(this, SIGNAL(receivedArcRemoved(ArcRemovedMessage_ptr)),
            this, SLOT(onArcRemoved(ArcRemovedMessage_ptr)));
}

void Manager::init(){
    m_cauv_node->addMessageObserver(shared_from_this());
    m_cauv_node->joinGroup("pl_gui");
}

// - Message Observer Implementation: thunks
void Manager::onGraphDescriptionMessage(GraphDescriptionMessage_ptr m){
    Q_EMIT receivedGraphDescription(m);
}
void Manager::onNodeParametersMessage(NodeParametersMessage_ptr m){
    Q_EMIT receivedNodeParameters(m);
}
void Manager::onNodeAddedMessage(NodeAddedMessage_ptr m){
    Q_EMIT receivedNodeAdded(m);
}
void Manager::onNodeRemovedMessage(NodeRemovedMessage_ptr m){
    Q_EMIT receivedNodeRemoved(m);
}
void Manager::onArcAddedMessage(ArcAddedMessage_ptr m){
    Q_EMIT receivedArcAdded(m);
}
void Manager::onArcRemovedMessage(ArcRemovedMessage_ptr m){
    Q_EMIT receivedArcRemoved(m);
}

// - Message Handling Slots:
void Manager::onGraphDescription(GraphDescriptionMessage_ptr m){
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
    for(i = m_nodes.right().begin(); i != m_nodes.right().end(); i++){
        j = m->nodeTypes().find(i->right);
        if(j == m->nodeTypes().end())
            nodes_for_removal.push_back(i->right);
        else if(j->second != i->left->type())
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
        i = m_nodes.right().find(id); 
        if(i == m_nodes.right().end()){
            addNode(type, id);
            i = m_nodes.right().find(j->first);
        }
        if(i == m_nodes.right().end())
            continue;
        fnode_ptr node = i->left;
        
        node_input_map_t::const_iterator inputs_it = m->nodeInputs().find(id);
        node_output_map_t::const_iterator outputs_it = m->nodeOutputs().find(id);
        node_param_map_t::const_iterator params_it = m->nodeParams().find(id);

        if(inputs_it == m->nodeInputs().end() ||
           outputs_it == m->nodeOutputs().end() ||
           params_it == m->nodeParams().end()){
            error() << "inconsistent graph description for node" << id;
            continue;
        }
        
        node->setParams(params_it->second);        
        node->setInputs(inputs_it->second);
        node->setOutputs(outputs_it->second);
    }

    for(j = m->nodeTypes().begin(); j != m->nodeTypes().end(); j++){
        const node_id_t id = j->first;
        i = m_nodes.right().find(id);
        fnode_ptr node = i->left;
        
        node_input_map_t::const_iterator inputs_it = m->nodeInputs().find(id);
        node_output_map_t::const_iterator outputs_it = m->nodeOutputs().find(id);

        node->setInputLinks(inputs_it->second);
        node->setOutputLinks(outputs_it->second);
    }
}

void Manager::onNodeParameters(NodeParametersMessage_ptr m){
    if(!_nameMatches(m)) return;
    node_id_map_t::right_iterator i = m_nodes.right().find(m->nodeId());
    if(i != m_nodes.right().end())
        i->left->setParams(m->params());
    else
        warning() << "unknown node" << m->nodeId();
}

void Manager::onNodeAdded(NodeAddedMessage_ptr m){
    if(!_nameMatches(m)) return;
    addNode(m);
}

void Manager::onNodeRemoved(NodeRemovedMessage_ptr m){
    if(!_nameMatches(m)) return;
    removeNode(m->nodeId());
}

void Manager::onArcAdded(ArcAddedMessage_ptr m){
    if(!_nameMatches(m)) return;
    // !!!
}

void Manager::onArcRemoved(ArcRemovedMessage_ptr m){
    if(!_nameMatches(m)) return;
    // !!!
}

// - Slot Implementations
void Manager::requestArc(NodeOutput from, NodeInput to){
    debug() << BashColour::Brown << BashIntensity::Bold << "requestArc" << from << to;
    m_cauv_node->send(boost::make_shared<AddArcMessage>(m_pipeline_name, from, to));
}

void Manager::requestRemoveArc(NodeOutput from, NodeInput to){
    debug() << BashColour::Brown << "requestRemoveArc" << from << to;
    m_cauv_node->send(boost::make_shared<RemoveArcMessage>(m_pipeline_name, from, to));
}

void Manager::requestNode(NodeType::e const& type){
    debug() << BashColour::Brown << BashIntensity::Bold << "requestAddNode" << type;
    m_cauv_node->send(boost::make_shared<AddNodeMessage>(
        m_pipeline_name, type, std::vector<NodeInputArc>(), std::vector<NodeOutputArc>()
    ));
}

void Manager::requestRemoveNode(node_id_t const& id){
    debug() << BashColour::Brown << "requestRemoveNode" << id;
    m_cauv_node->send(boost::make_shared<RemoveNodeMessage>(m_pipeline_name, id));
}

// - General Protected Implementations
void Manager::removeNode(node_id_t const& id){
    debug() << BashColour::Red << "removeNode:" << id;
    node_id_map_t::right_iterator i = m_nodes.right().find(id);
    if(i != m_nodes.right().end()){
        i->left->fadeAndRemove();
        m_nodes.right().erase(i);
        m_imgnodes.right().erase(id);
    }else{
        error() << "no such node:" << id;
    }
}

void Manager::addNode(NodeType::e const& type, node_id_t const& id){
    debug() << BashColour::Green << "addNode:" << type << id;
    if(isInvalid(type)){
        warning() << "ignoring invalid node type" << type;
    }else if(isInvalid(id)){
        warning() << "ignoring invalid node id" << id;
    }else if(m_nodes.right().find(id) != m_nodes.right().end()){
        error() << "node" << id << "already exists";
    }else if(isImageNode(type)){
        imgnode_ptr p = new ImgNode(*this, id, type);
        m_nodes.insert(node_id_map_t::value_type(p, id));
        m_imgnodes.insert(imgnode_id_map_t::value_type(p, id));
        m_scene->addItem(p);
    }else{
        fnode_ptr p = new FNode(*this, id, type);
        m_nodes.insert(node_id_map_t::value_type(p, id));
        m_scene->addItem(p);
    }
}

void Manager::addNode(NodeAddedMessage_ptr m){
    const NodeType::e type = m->nodeType();
    const node_id_t id = m->nodeId();
    debug() << BashColour::Green << "addNode:" << type << id;
    if(isInvalid(type)){
        warning() << "ignoring invalid node type" << type;
    }else if(isInvalid(id)){
        warning() << "ignoring invalid node id" << id;
    }else if(m_nodes.right().find(id) != m_nodes.right().end()){
        error() << "node" << id << "already exists";
    }else if(isImageNode(type)){
        imgnode_ptr p = new ImgNode(*this, m);
        m_nodes.insert(node_id_map_t::value_type(p, id));
        m_imgnodes.insert(imgnode_id_map_t::value_type(p, id));
        m_scene->addItem(p);        
    }else{
        fnode_ptr p = new FNode(*this, m);
        m_nodes.insert(node_id_map_t::value_type(p, id));
        m_scene->addItem(p);
    }
}

void Manager::clearNodes(){
    debug() << BashColour::Red << "clearNodes";
    node_id_map_t::right_iterator i;
    for(i = m_nodes.right().begin(); i != m_nodes.right().end(); i++)
        i->left->fadeAndRemove();
    m_nodes.clear();
    m_imgnodes.clear();
}

