#include "imageProcessor.h"
#include "nodeFactory.h"
#include "nodes/inputNode.h"

#include <boost/make_shared.hpp>

#include <utility/string.h>
#include <generated/messages.h>

using namespace cauv::imgproc;

ImageProcessor::ImageProcessor(mb_ptr_t mb)
    : m_nodes_lock(), m_nodes(), m_nodes_rev(), m_input_nodes(), m_scheduler(),
      m_name("---"), m_mailbox_lock(), m_mailbox(mb){
}

void ImageProcessor::start(std::string const& name){
    lock_t l(m_name_lock);
    m_name = name;
    m_scheduler.start();
}

void ImageProcessor::onImageMessage(ImageMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<input_node_ptr_t>::iterator i;    
    debug(5) << __func__ << "notifying" << m_input_nodes.size() << "input nodes";
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onImageMessage(m);
}

void ImageProcessor::onSonarDataMessage(SonarDataMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<input_node_ptr_t>::iterator i;        
    debug(5) << __func__ << "notifying" << m_input_nodes.size() << "input nodes";
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onSonarDataMessage(m);
}

void ImageProcessor::onAddNodeMessage(AddNodeMessage_ptr m){
    if(!_filterMatches(m))
        return;
    node_id new_id = 0;
    std::map<std::string, NodeParamValue> params;
    std::map<std::string, NodeOutput> inputs;
    std::map<std::string, std::vector<NodeInput> > outputs;
    try{
        node_ptr_t node = NodeFactoryRegister::create(m_scheduler, *this, m_name, m->nodeType());

        BOOST_FOREACH(NodeInputArc const& a, m->parents()){
            node->setInput(a.input, lookup(a.src.node), a.src.output);
            lookup(a.src.node)->setOutput(a.src.output, node, a.input);
        }
        BOOST_FOREACH(NodeOutputArc const& a, m->children()){
            node->setOutput(a.output, lookup(a.dst.node), a.dst.input);
            lookup(a.dst.node)->setInput(a.dst.input, node , a.output);
        }
        
        new_id = node->id();
        lock_t l(m_nodes_lock);
        _addNode(node, new_id);
        if(node->isInputNode()){
            debug(6) << "adding input node:" << *node;
            m_input_nodes.insert(boost::dynamic_pointer_cast<InputNode, Node>(node));
        }
        l.unlock();

        params = node->parameters();
        inputs = node->inputLinks();
        outputs = node->outputLinks();

        info() << "Node added: (type=" << m->nodeType() << " "
               << m->parents().size() << " parents, "
               << m->children().size() << " children)";
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
    sendMessage(boost::make_shared<NodeAddedMessage>(m_name, new_id, m->nodeType(), inputs, outputs, params));
}

void ImageProcessor::removeNode(node_id const& id){
    std::vector<ArcRemovedMessage_ptr> arms;
    lock_t l(m_nodes_lock);
    node_ptr_t n = lookup(id);
    
    // unlink the node first:        
    Node::msg_node_input_map_t il = n->inputLinks();
    Node::msg_node_output_map_t ol = n->outputLinks();
    NodeInput in;
    in.node = id;
    for(Node::msg_node_input_map_t::const_iterator i = il.begin(); i != il.end(); i++){
        in.input = i->first;
        arms.push_back(boost::make_shared<ArcRemovedMessage>(m_name, i->second, in));
    }
    NodeOutput out;
    out.node = id;
    for(Node::msg_node_output_map_t::const_iterator i = ol.begin(); i != ol.end(); i++){
        out.output = i->first;
        for(Node::msg_node_in_list_t::const_iterator j = i->second.begin(); j != i->second.end(); j++)
            arms.push_back(boost::make_shared<ArcRemovedMessage>(m_name, out, *j));
    }       
    
    _removeNode(id); 
    info() << "Node removed:" << id;
    
    if(n->isInputNode()){
        debug(6) << "removing input node:" << *n;
        m_input_nodes.erase(boost::dynamic_pointer_cast<InputNode, Node>(n));
    }
    l.unlock();

    /* since the graph is linked both ways we have to unlink the node from
     * it's neighbours _and_ unlink the neighbours from the node
     */
    BOOST_FOREACH(node_ptr_t p, n->parents())
        if(p) p->clearOutputs(n);
    BOOST_FOREACH(node_ptr_t p, n->children())
        if(p) p->clearInputs(n);
    n->clearOutputs();
    n->clearInputs();
    
    std::vector<ArcRemovedMessage_ptr>::const_iterator i;
    for(i = arms.begin(); i != arms.end(); i++)
        sendMessage(*i);
    sendMessage(boost::make_shared<NodeRemovedMessage>(m_name, id));
}

void ImageProcessor::onRemoveNodeMessage(RemoveNodeMessage_ptr m){
    if(!_filterMatches(m))
        return;
    try{
        removeNode(m->nodeId());
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::onSetNodeParameterMessage(SetNodeParameterMessage_ptr m){
    if(!_filterMatches(m))
        return;
    try{
        node_ptr_t n = lookup(m->nodeId());
        n->setParam(m);
        info() << "Node parameter set:"
               << m->nodeId() <<  m->paramId() << std::boolalpha << m->value();
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::onAddArcMessage(AddArcMessage_ptr m){
    if(!_filterMatches(m))
        return;
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
        Node::msg_node_input_map_t::const_iterator old_il_in = old_il.find(input);
        if(old_il_in != old_il.end() && old_il_in->second.node){
            node_ptr_t old_from = lookup(old_il_in->second.node);
            if(old_from)
                old_from->clearOutput(old_il_in->second.output, to, input);
        }else if(old_il_in == old_il.end()){
            error() << "badness: node" << m->to().node
                    << "has no input link record for input" << input;
        }
        to->clearInput(input);

        from->setOutput(output, to, input);
        to->setInput(input, from, output);

        info() << "Arc added:" << m->from() << "->" << m->to();
        
        sendMessage(boost::make_shared<ArcRemovedMessage>(m_name, il[input], m->to()));
        sendMessage(boost::make_shared<ArcAddedMessage>(m_name, m->from(), m->to()));
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::onRemoveArcMessage(RemoveArcMessage_ptr m){
    if(!_filterMatches(m))
        return;
    try{
        node_ptr_t from = lookup(m->from().node);
        node_ptr_t to = lookup(m->to().node);
        output_id output = m->from().output;
        input_id input = m->to().input;

        if(!from) throw id_error(MakeString() << "invalid node:" << m->from().node);
        if(!to) throw id_error(MakeString() << "invalid node:" << m->to().node);        
        
        Node::msg_node_input_map_t il = to->inputLinks();
        
        // remove any existing arc:
        Node::msg_node_input_map_t old_il = to->inputLinks();
        Node::msg_node_input_map_t::const_iterator old_il_in = old_il.find(input);
        if(old_il_in != old_il.end() && old_il_in->second.node){
            node_ptr_t old_from = lookup(old_il_in->second.node);
            if(old_from && old_from == from)
                old_from->clearOutput(old_il_in->second.output, to, input);
        }else if(old_il_in == old_il.end()){
            error() << "badness: node" << m->to().node
                    << "has no input link record for input" << input;
        }
        to->clearInput(input);

        info() << "Arc removed:" << m->from() << "->" << m->to();
        
        sendMessage(boost::make_shared<ArcRemovedMessage>(m_name, m->from(), m->to()));
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::onGraphRequestMessage(GraphRequestMessage_ptr m){
    if(!_filterMatches(m))
        return;
    try{
        std::map<node_id, NodeType::e> node_types;
        std::map<node_id, std::map<input_id, NodeOutput> > node_inputs;
        std::map<node_id, std::map<output_id, std::vector<NodeInput> > > node_outputs;
        std::map<node_id, std::map<input_id, NodeParamValue> > node_parameters;
        
        lock_t l(m_nodes_lock);
        std::map<node_id, node_ptr_t>::const_iterator i;
        for(i = m_nodes.begin(); i != m_nodes.end(); i++){
            node_types[i->first] = i->second->type();
            node_inputs[i->first] = i->second->inputLinks();
            node_outputs[i->first] = i->second->outputLinks();
            node_parameters[i->first] = i->second->parameters();
        }

        sendMessage(boost::make_shared<GraphDescriptionMessage>(
            m_name, node_types, node_inputs, node_outputs, node_parameters
        ));
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}


void ImageProcessor::onForceExecRequestMessage(ForceExecRequestMessage_ptr m){
    if(!_filterMatches(m))
        return;
    try{
        node_ptr_t n = lookup(m->nodeId());
        if(n){
            n->checkAddSched(Node::Force);
        }else{
            error() << __func__ << "invalid node id:" << m->nodeId();
        }
    }catch(std::exception& e){
        error() << __func__  << ":" << e.what();
    }
}

void ImageProcessor::onClearPipelineMessage(ClearPipelineMessage_ptr m){
    if(!_filterMatches(m))
        return;
    try{
        lock_t l(m_nodes_lock);    
        while(m_nodes.size())
            removeNode(m_nodes.rbegin()->first);
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}


void ImageProcessor::onPipelineDiscoveryRequestMessage(PipelineDiscoveryRequestMessage_ptr){
    info() << "Pipeline discovery request message recieved";
    sendMessage(boost::make_shared<PipelineDiscoveryResponseMessage>(m_name));
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
        throw id_error(std::string("Unknown node id: ") + toStr(id));
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
    _addNode(p, p->id());
}

void ImageProcessor::_removeNode(node_id const& id) throw(id_error){
    std::map<node_id, node_ptr_t>::iterator i = m_nodes.find(id);
    if(i != m_nodes.end()){
        m_nodes_rev.erase(i->second);
        m_nodes.erase(i);
    }else{
        throw id_error(std::string("Unknown node id: ") + toStr(id));
    }
}


