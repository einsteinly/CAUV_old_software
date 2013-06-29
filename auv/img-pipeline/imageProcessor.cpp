/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "imageProcessor.h"
#include "nodeFactory.h"
#include "nodes/inputNode.h"

#include <boost/make_shared.hpp>

#include <utility/string.h>
#include <map>
#include <list>

using namespace cauv::imgproc;

ImageProcessor::ImageProcessor(mb_ptr_t mb, boost::shared_ptr<Scheduler> scheduler)
    : m_nodes_lock(), m_nodes(), m_nodes_rev(), m_input_nodes(), m_scheduler(scheduler),
      m_name("---"), m_mailbox_lock(), m_mailbox(mb){

    m_mailbox->subMessage(AddNodeMessage());
    m_mailbox->subMessage(RemoveNodeMessage());
    m_mailbox->subMessage(SetNodeParameterMessage());
    m_mailbox->subMessage(AddArcMessage());
    m_mailbox->subMessage(RemoveArcMessage());
    m_mailbox->subMessage(SetPipelineMessage());
    m_mailbox->subMessage(GraphRequestMessage());
    m_mailbox->subMessage(ForceExecRequestMessage());
    m_mailbox->subMessage(ClearPipelineMessage());
    m_mailbox->subMessage(PipelineDiscoveryRequestMessage());

    m_mailbox->subMessage(GPSLocationMessage());
    m_mailbox->subMessage(TelemetryMessage());
}

void ImageProcessor::start(std::string const& name){
    lock_t l(m_name_lock);
    m_name = name;
}

void ImageProcessor::onLinesMessage(LinesMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<input_node_ptr_t>::iterator i;    
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onLinesMessage(m);
}

void ImageProcessor::onImageMessage(ImageMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<input_node_ptr_t>::iterator i;    
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onImageMessage(m);
}

void ImageProcessor::onSonarDataMessage(SonarDataMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<input_node_ptr_t>::iterator i;        
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onSonarDataMessage(m);
}

void ImageProcessor::onSonarImageMessage(SonarImageMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<input_node_ptr_t>::iterator i;        
    for(i = m_input_nodes.begin(); i != m_input_nodes.end(); i++)
        (*i)->onSonarImageMessage(m);
}


void ImageProcessor::onTelemetryMessage(TelemetryMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<node_ptr_t>::iterator i;        
    for(i = m_telem_req_nodes.begin(); i != m_telem_req_nodes.end(); i++)
        (*i)->onTelemetry(m);
    
}

void ImageProcessor::onGPSLocationMessage(GPSLocationMessage_ptr m){
    lock_t l(m_nodes_lock);
    std::set<node_ptr_t>::iterator i;        
    for(i = m_gps_req_nodes.begin(); i != m_gps_req_nodes.end(); i++)
        (*i)->onGPSLoc(m);
}

void ImageProcessor::onAddNodeMessage(AddNodeMessage_ptr m){
    if(!_filterMatches(m))
        return;
    node_id new_id = 0;
    Node::msg_node_param_map_t params;
    Node::msg_node_input_map_t inputs;
    Node::msg_node_output_map_t outputs;
    try{
        node_ptr_t node = NodeFactoryRegister::create(*m_scheduler, *this, m_name, m->nodeType());

        for (NodeInputArc const& a : m->parents()){
            node->setInput(a.input, lookup(a.src.node), a.src.output);
            lookup(a.src.node)->setOutput(a.src.output, node, a.input);
        }
        for (NodeOutputArc const& a : m->children()){
            node->setOutput(a.output, lookup(a.dst.node), a.dst.input);
            lookup(a.dst.node)->setInput(a.dst.input, node , a.output);
        }
        
        new_id = node->id();
        lock_t l(m_nodes_lock);
        _addNode(node, new_id);
        if(node->isInputNode()){
            m_input_nodes.insert(boost::dynamic_pointer_cast<InputNode, Node>(node));
        }
        l.unlock();

        params = node->parameters();
        inputs = node->inputLinks();
        outputs = node->outputLinks();

        debug(2) << "Node added: (id="
                 << new_id << " type=" << m->nodeType() << " "
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
    {
        NodeInput in;
        in.node = id;
        for (const auto& input_link : il){
            in.input = input_link.first.input;
            arms.push_back(boost::make_shared<ArcRemovedMessage>(m_name, input_link.second, in));
        }
    }
    {
        NodeOutput out;
        out.node = id;
        for (const auto& output_link : ol){
            out.output = output_link.first.output;
            for(auto const & in : output_link.second)
                arms.push_back(boost::make_shared<ArcRemovedMessage>(m_name, out, in));
        }
    }
    
    _removeNode(id); 
    debug(2) << "Node removed:" << id;
    
    if(n->isInputNode()){
        m_input_nodes.erase(boost::dynamic_pointer_cast<InputNode, Node>(n));
    }
    l.unlock();

    /* since the graph is linked both ways we have to unlink the node from
     * it's neighbours _and_ unlink the neighbours from the node
     */
    for (node_ptr_t p : n->parents())
        if(p) p->clearOutputs(n);
    for (node_ptr_t p : n->children())
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
        debug(2) << "Node parameter set:" << m->nodeId() <<  m->paramId()
                 << std::string((mkStr() << std::boolalpha << m->value()).lengthLimit(100));
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
        NodeOutput old_from;
        
        // remove any existing arc to input first:
        Node::msg_node_input_map_t old_il = to->inputLinks();
        Node::msg_node_input_map_t::const_iterator old_il_in;
        for(old_il_in = old_il.begin(); old_il_in != old_il.end(); old_il_in++)
            if(old_il_in->first.input == input){
                old_from = old_il_in->second;
                break;
            }
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

        debug(2) << "Arc added:" << m->from() << "->" << m->to();
        
        if(old_from.node)
            sendMessage(boost::make_shared<ArcRemovedMessage>(m_name, old_from, m->to()));
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
        NodeOutput old_from;        
        
        // remove any existing arc:
        Node::msg_node_input_map_t old_il = to->inputLinks();
        Node::msg_node_input_map_t::const_iterator old_il_in;
        for(old_il_in = old_il.begin(); old_il_in != old_il.end(); old_il_in++)
            if(old_il_in->first.input == input){
                old_from = old_il_in->second;
                break;
            }

        if(old_il_in != old_il.end() && old_il_in->second.node){
            node_ptr_t old_from = lookup(old_il_in->second.node);
            if(old_from && old_from == from)
                old_from->clearOutput(old_il_in->second.output, to, input);
        }else if(old_il_in == old_il.end()){
            error() << "badness: node" << m->to().node
                    << "has no input link record for input" << input;
        }
        to->clearInput(input);

        debug(2) << "Arc removed:" << m->from() << "->" << m->to();
        
        sendMessage(boost::make_shared<ArcRemovedMessage>(m_name, m->from(), m->to()));
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}

void ImageProcessor::onSetPipelineMessage(SetPipelineMessage_ptr m){
    if(!_filterMatches(m))
        return;
    lock_t l(m_nodes_lock);
    //translation between ids
    try{
        //clear the existing pipeline
        while(m_nodes.size()){
            node_id n = m_nodes.rbegin()->first;
            removeNode(n);
        }
    }
    catch(std::exception& e){
        error() << __func__ << ":" <<e.what();
    }
    std::map<node_id, node_id> old_id2new_id;
    //need to add all nodes first so that arcs can be added
    for(auto const & node_type : m->nodeTypes()){
        try{
            node_ptr_t node = NodeFactoryRegister::create(*m_scheduler, *this, m_name, node_type.second);
            if(node->isInputNode()){
                m_input_nodes.insert(boost::dynamic_pointer_cast<InputNode, Node>(node));
            }
            old_id2new_id[node_type.first] = node->id();
            _addNode(node, node->id());
        }
        catch(std::exception& e){
            error() << __func__ << ":" <<e.what();
        }
    }
    debug() << old_id2new_id;
    //params that dont need to be set because they are filled by arcs
    std::map<node_id, std::list<input_id> > params_with_arcs;
    //now fill in arcs
    for(auto const & node_connection : m->nodeConnections()){
        try{
            node_ptr_t to = lookup(old_id2new_id[node_connection.first]);
            if(!to) throw id_error(MakeString() << "invalid node:" << node_connection.first);
            std::list<input_id> node_params_set;
            
            for(auto const & connection : node_connection.second){
                try{
                    if(connection.second.node == 0){
                        continue;
                    }
                    node_ptr_t from = lookup(old_id2new_id[connection.second.node]);
                    if(!from) throw id_error(MakeString() << "invalid node:" << connection.second.node);
                    
                    output_id output = connection.second.output;
                    input_id input = connection.first.input;
                    
                    from->setOutput(output, to, input);
                    to->setInput(input, from, output);
                    
                    node_params_set.push_back(input);
                }
                catch(std::exception& e){
                    error() << __func__ << ":" <<e.what();
                }
            }
            params_with_arcs[node_connection.first] = node_params_set;
        }
        catch(std::exception& e){
            error() << __func__ << ":" <<e.what();
        }
    }
    for(auto const & node_param : m->nodeParams()){
        try{
            std::map<node_id, std::list<input_id> >::const_iterator exclude_params_it = params_with_arcs.find(node_param.first);
            if(exclude_params_it==params_with_arcs.end()){
                throw id_error(std::string("Unknown node id: ") + toStr(node_param.first));
            }
            std::list<input_id> exclude_params = exclude_params_it->second;
            node_ptr_t n = lookup(old_id2new_id[node_param.first]);
            for(auto const & param : node_param.second){
                std::list<input_id>::iterator ep_it;
                for(ep_it = exclude_params.begin(); ep_it != exclude_params.end(); ep_it++){
                    if(*ep_it == param.first.input){
                        break;
                    }
                }
                //found that param was set by inarc
                if(ep_it != exclude_params.end()){
                    continue;
                }
                try{
                    n->setParam(param.first.input, param.second);
                }catch(std::exception& e){
                    error() << __func__ << ":" << e.what();
                }
            }
        }catch(std::exception& e){
            error() << __func__ << ":" << e.what();
        }
    }
    try{
        std::map<node_id, NodeType::e> node_types;
        std::map<node_id, Node::msg_node_input_map_t > node_inputs;
        std::map<node_id, Node::msg_node_output_map_t > node_outputs;
        std::map<node_id, Node::msg_node_param_map_t > node_parameters;
        
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
    l.unlock();
}

void ImageProcessor::onGraphRequestMessage(GraphRequestMessage_ptr m){
    if(!_filterMatches(m))
        return;
    try{
        std::map<node_id, NodeType::e> node_types;
        std::map<node_id, Node::msg_node_input_map_t > node_inputs;
        std::map<node_id, Node::msg_node_output_map_t > node_outputs;
        std::map<node_id, Node::msg_node_param_map_t > node_parameters;
        
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
    if(!_filterMatchesPartial(m))
        return;
    try{
        lock_t l(m_nodes_lock);    
        while(m_nodes.size()){
            node_id n = m_nodes.rbegin()->first;
            removeNode(n);
        }
    }catch(std::exception& e){
        error() << __func__ << ":" << e.what();
    }
}


void ImageProcessor::onPipelineDiscoveryRequestMessage(PipelineDiscoveryRequestMessage_ptr){
    debug(2) << "Pipeline discovery request message received";
    sendMessage(boost::make_shared<PipelineDiscoveryResponseMessage>(m_name));
}

void ImageProcessor::sendMessage(const boost::shared_ptr<const Message> msg, MessageReliability reliability) const{
    m_mailbox->sendMessage(msg, reliability);
}

void ImageProcessor::subMessage(Message const& msg, node_id const&){
    m_mailbox->subMessage(msg);
}


ImageProcessor::~ImageProcessor(){
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
    if(p->requiresGPS())
        m_gps_req_nodes.insert(p);
    if(p->requiresTelemetry())
        m_telem_req_nodes.insert(p);
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


