#include "imageProcessor.h"
#include "nodeFactory.h"
#include "nodes/inputNode.h"

#include <boost/make_shared.hpp>

#include <common/messages.h>

ImageProcessor::ImageProcessor(mb_ptr_t mb)
    : m_scheduler(), m_mailbox(mb){
    m_scheduler.start();
}

void ImageProcessor::onImageMessage(boost::shared_ptr<ImageMessage> m){
    std::set<input_node_ptr_t>::iterator i;
    debug() << __func__ << "notifying" << m_input_nodes.size() << "input nodes";
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onImageMessage(m);
}

void ImageProcessor::onAddNodeMessage(boost::shared_ptr<AddNodeMessage> m){
    node_id new_id = 0;
    try{
        node_ptr_t node = NodeFactoryRegister::create(m->nodeType(), m_scheduler);

        BOOST_FOREACH(NodeInputArc const& a, m->parents()){
            node->setInput(a.input, _lookupNode(a.src.node), a.src.output);
            _lookupNode(a.src.node)->setOutput(a.src.output, node, a.input);
        }
        BOOST_FOREACH(NodeOutputArc const& a, m->children()){
            node->setOutput(a.output, _lookupNode(a.dst.node), a.dst.input);
            _lookupNode(a.dst.node)->setInput(a.dst.input, node , a.output);
        }
        
        new_id = _newID(node);
        m_nodes[new_id] = node;

        if(node->isInputNode()){
            m_input_nodes.insert(boost::dynamic_pointer_cast<InputNode, Node>(node));
        }
        info() << "Node added, (type=" << m->nodeType() << " "
               << m->parents().size() << " parents, "
               << m->children().size() << " children)";
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
    // TODO: error message of some sort, or something
    sendMessage(boost::make_shared<NodeAddedMessage>(new_id));
}

void ImageProcessor::onRemoveNodeMessage(boost::shared_ptr<RemoveNodeMessage> m){
    try{
        node_ptr_t n = _lookupNode(m->nodeId());
        m_nodes.erase(m->nodeId());
       
        if(n->isInputNode()){
            m_input_nodes.erase(boost::dynamic_pointer_cast<InputNode, Node>(n));
        }
        /* since the graph is linked both ways we have to unlink the node from
         * it's neighbors _and_ unlink the neigbors from the node
         */
        BOOST_FOREACH(node_ptr_t p, n->parents())
            p->clearOutputs(n);
        BOOST_FOREACH(node_ptr_t p, n->children())
            p->clearInputs(n);
        n->clearOutputs();
        n->clearInputs();
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
    // TODO: error message of some sort, or something
}

void ImageProcessor::onSetNodeParameterMessage(boost::shared_ptr<SetNodeParameterMessage> m){
    try{
        node_ptr_t n = _lookupNode(m->nodeId());
        n->setParam(m);
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
    // TODO: error message of some sort, or something
}

void ImageProcessor::sendMessage(const boost::shared_ptr<Message> msg, service_t service_type) const{
    m_mailbox->sendMessage(msg, service_type);
}

ImageProcessor::~ImageProcessor(){
    m_scheduler.stopWait();
}


