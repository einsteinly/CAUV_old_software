#include "imageProcessor.h"
#include "nodeFactory.h"
#include "nodes/inputNode.h"

#include <boost/make_shared.hpp>

#include <common/messages.h>

// TODO: error() should send an error message of some sort on spread

ImageProcessor::ImageProcessor(mb_ptr_t mb)
    : m_scheduler(), m_mailbox(mb){
    m_scheduler.start();
}

void ImageProcessor::onImageMessage(ImageMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<input_node_ptr_t>::iterator i;
    debug() << __func__ << "notifying" << m_input_nodes.size() << "input nodes";
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onImageMessage(m);
}

void ImageProcessor::onAddNodeMessage(AddNodeMessage_ptr m){
    node_id new_id = 0;
    std::map<std::string, NodeParamValue> params;
    std::map<std::string, NodeOutput> inputs;
    std::map<std::string, std::vector<NodeInput> > outputs;
    try{
        node_ptr_t node = NodeFactoryRegister::create(m->nodeType(), m_scheduler, *this);

        BOOST_FOREACH(NodeInputArc const& a, m->parents()){
            node->setInput(a.input, lookup(a.src.node), a.src.output);
            lookup(a.src.node)->setOutput(a.src.output, node, a.input);
        }
        BOOST_FOREACH(NodeOutputArc const& a, m->children()){
            node->setOutput(a.output, lookup(a.dst.node), a.dst.input);
            lookup(a.dst.node)->setInput(a.dst.input, node , a.output);
        }
        
        new_id = _newID(node);
        lock_t l(m_nodes_lock);
        _addNode(node, new_id);
        if(node->isInputNode()){
            m_input_nodes.insert(boost::dynamic_pointer_cast<InputNode, Node>(node));
        }
        l.unlock();

        params = node->parameters();
        inputs = node->inputLinks();
        outputs = node->outputLinks();

        info() << "Node added, (type=" << m->nodeType() << " "
               << m->parents().size() << " parents, "
               << m->children().size() << " children)";
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
    sendMessage(boost::make_shared<NodeAddedMessage>(new_id, m->nodeType(), inputs, outputs));
    sendMessage(boost::make_shared<NodeParametersMessage>(new_id, params));
}

void ImageProcessor::onRemoveNodeMessage(RemoveNodeMessage_ptr m){
    try{
        std::vector<ArcRemovedMessage_ptr> arms;
        lock_t l(m_nodes_lock);
        node_ptr_t n = lookup(m->nodeId());
        
        // unlink the node first:        
        Node::msg_node_input_map_t il = n->inputLinks();
        Node::msg_node_output_map_t ol = n->outputLinks();
        NodeInput in;
        in.node = m->nodeId();
        for(Node::msg_node_input_map_t::const_iterator i = il.begin(); i != il.end(); i++){
            in.input = i->first;
            arms.push_back(boost::make_shared<ArcRemovedMessage>(i->second, in));
        }
        NodeOutput out;
        out.node = m->nodeId();
        for(Node::msg_node_output_map_t::const_iterator i = ol.begin(); i != ol.end(); i++){
            out.output = i->first;
            for(Node::msg_node_in_list_t::const_iterator j = i->second.begin(); j != i->second.end(); j++)
                arms.push_back(boost::make_shared<ArcRemovedMessage>(out, *j));
        }       
        
        _removeNode(m->nodeId()); 
        
        if(n->isInputNode()){
            m_input_nodes.erase(boost::dynamic_pointer_cast<InputNode, Node>(n));
        }
        l.unlock();

        /* since the graph is linked both ways we have to unlink the node from
         * it's neighbors _and_ unlink the neighbors from the node
         */
        BOOST_FOREACH(node_ptr_t p, n->parents())
            p->clearOutputs(n);
        BOOST_FOREACH(node_ptr_t p, n->children())
            p->clearInputs(n);
        n->clearOutputs();
        n->clearInputs();
        
        std::vector<ArcRemovedMessage_ptr>::const_iterator i;
        for(i = arms.begin(); i != arms.end(); i++)
            sendMessage(*i);
        sendMessage(boost::make_shared<NodeRemovedMessage>(m->nodeId()));
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::onSetNodeParameterMessage(SetNodeParameterMessage_ptr m){
    try{
        node_ptr_t n = lookup(m->nodeId());
        n->setParam(m);
        
        sendMessage(boost::make_shared<NodeParametersMessage>(m->nodeId(), n->parameters()));
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::onAddArcMessage(AddArcMessage_ptr m){
    try{
        node_ptr_t from = lookup(m->from().node);
        node_ptr_t to = lookup(m->to().node);
        output_id output = m->from().output;
        input_id input = m->to().input;

        if(!from) throw id_error(MakeString() << "invalid node:" << m->from().node);
        if(!to) throw id_error(MakeString() << "invalid node:" << m->to().node);        
        
        Node::msg_node_input_map_t il = to->inputLinks();
        
        // remove any existing arc to input first:
        Node::msg_node_input_map_t old_il = to->inputLinks();
        if(old_il[input].node){
            node_ptr_t old_from = lookup(old_il[input].node);
            if(old_from)
                old_from->clearOutput(old_il[input].output, to, input);
        }
        to->clearInput(input);

        from->setOutput(output, to, input);
        to->setInput(input, from, output);
        
        sendMessage(boost::make_shared<ArcRemovedMessage>(il[input], m->to()));
        sendMessage(boost::make_shared<ArcAddedMessage>(m->from(), m->to()));
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::onGraphRequestMessage(GraphRequestMessage_ptr){
    try{
        std::map<node_id, NodeType::e> node_types;
        std::map<node_id, std::map<input_id, NodeOutput> > node_inputs;
        std::map<node_id, std::map<output_id, std::vector<NodeInput> > > node_outputs;
        std::map<node_id, std::map<param_id, NodeParamValue> > node_parameters;
        
        lock_t l(m_nodes_lock);
        std::map<node_id, node_ptr_t>::const_iterator i;
        for(i = m_nodes.begin(); i != m_nodes.end(); i++){
            node_types[i->first] = i->second->type();
            node_inputs[i->first] = i->second->inputLinks();
            node_outputs[i->first] = i->second->outputLinks();
            node_parameters[i->first] = i->second->parameters();
        }

        sendMessage(boost::make_shared<GraphDescriptionMessage>(
            node_types, node_inputs, node_outputs, node_parameters
        ));
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::sendMessage(const boost::shared_ptr<const Message> msg, service_t service_type) const{
    m_mailbox->sendMessage(msg, service_type);
}

ImageProcessor::~ImageProcessor(){
    m_scheduler.stopWait();
}


node_ptr_t ImageProcessor::lookup(node_id const& id) const throw(id_error){
    lock_t l(m_nodes_lock);
    std::map<node_id, node_ptr_t>::const_iterator i = m_nodes.find(id);
    if(i != m_nodes.end())
        return i->second;
    else
        throw(id_error(std::string("Unknown node id: ") + to_string(id)));
}

node_id ImageProcessor::lookup(node_ptr_t const& p) const throw(){
    lock_t l(m_nodes_lock);
    std::map<node_ptr_t, node_id>::const_iterator i = m_nodes_rev.find(p);
    if(i != m_nodes_rev.end())
        return i->second;
    else
        return 0;
}

void ImageProcessor::_addNode(node_ptr_t const& p, node_id const& id) throw(){
    lock_t l(m_nodes_lock);
    m_nodes[id] = p;
    m_nodes_rev[p] = id;
}

void ImageProcessor::_addNode(node_ptr_t const& p) throw(){
    _addNode(p, _newID(p));
}

void ImageProcessor::_removeNode(node_id const& id) throw(id_error){
    std::map<node_id, node_ptr_t>::iterator i = m_nodes.find(id);
    if(i != m_nodes.end()){
        m_nodes_rev.erase(i->second);
        m_nodes.erase(i);
    }else{
        throw(id_error(std::string("Unknown node id: ") + to_string(id)));
    }
}

node_id ImageProcessor::_newID(node_ptr_t) const throw(){
    // Can probably do better than this...
    static node_id id = 1;
    return id++;
}

