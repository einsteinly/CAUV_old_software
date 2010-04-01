#include "node.h"


Node::Node(Scheduler& sched, ImageProcessor& pl, NodeType::e type)
    : m_priority(priority_slow), m_speed(slow),
      m_node_type(type), m_exec_queued(false), m_output_demanded(false),
      m_sched(sched), m_pl(pl){
}

NodeType::e const& Node::type() const{
    return m_node_type;
}

/* overload for the common case where we're connecting a node with one
 * output to a node with one input
 */
void Node::setInput(node_ptr_t n){
    lock_t l(m_parent_links_lock);
    
    if(m_parent_links.size() == 1){
        const output_id_set_t parent_outputs = n->outputs();
        if(parent_outputs.size() != 1){
            std::cerr << "Parent has " << parent_outputs.size() << " outputs!" << std::endl;
            throw link_error("setInput: specific parent output must be specified");
        }
        const in_link_map_t::iterator i = m_parent_links.begin();
        if(i->second.first){
            throw(link_error("old arc must be removed first"));
        }
        i->second = input_link_t(n, *parent_outputs.begin());
        // TODO: if (and only if) there is already an image available
        // from the parent, we should call newInput here:
        //newInput(m_parent_links.begin()->first); 
        // else: demand new output from the parent
        debug() << "node" << *this << "input set, demand new output on" <<  n;
        n->demandNewOutput();
    }else if(m_parent_links.size() > 1){
        throw link_error("setInput: specific input must be specified");
    }else{
        throw link_error("setInput: This node accepts no inputs");
    }
}

void Node::setInput(input_id const& i_id, node_ptr_t n, output_id const& o_id){
    lock_t l(m_parent_links_lock);
    const in_link_map_t::iterator i = m_parent_links.find(i_id);
    if(i == m_parent_links.end()){
        throw(id_error("setInput: Invalid input id" + to_string(i_id)));
    }else{
        if(i->second.first){
            throw(link_error("old arc must be removed first"));
        }
        i->second = input_link_t(n, o_id); 
        // TODO: if (and only if) there is already an image available
        // from the parent, we should call newInput here:
        //newInput(i->first);
        // else: demand new output from the parent
        debug() << "node" << *this << "input set, demand new output on" <<  n;
        n->demandNewOutput();
    }
}

void Node::clearInput(input_id const& i_id){ 
    lock_t l(m_parent_links_lock);
    const in_link_map_t::iterator i = m_parent_links.find(i_id);
    if(i == m_parent_links.end()){
        throw(id_error("clearInput: Invalid input id" + to_string(i_id)));
    }else{
        i->second = input_link_t();
    }
}

void Node::clearInputs(node_ptr_t parent){
    lock_t l(m_parent_links_lock);
    in_link_map_t::iterator i;
    for(i = m_parent_links.begin(); i != m_parent_links.end(); i++){
        if(i->second.first == parent)
            i->second = input_link_t();
    }
}

void Node::clearInputs(){
    lock_t l(m_parent_links_lock); 
    in_link_map_t::iterator i;
    for(i = m_parent_links.begin(); i != m_parent_links.end(); i++){
        i->second = input_link_t();
    }
}

Node::input_id_set_t Node::inputs() const{
    input_id_set_t r;
    lock_t n(m_parent_links_lock);
    in_link_map_t::const_iterator i;
    for(i = m_parent_links.begin(); i != m_parent_links.end(); i++)
        r.insert(i->first);
    return r;
}

Node::msg_node_input_map_t Node::inputLinks() const{
    lock_t l(m_parent_links_lock);
    msg_node_input_map_t r;
    in_link_map_t::const_iterator i;
    for(i = m_parent_links.begin(); i != m_parent_links.end(); i++){
        NodeOutput t;
        t.node = m_pl.lookup(i->second.first);
        t.output = i->second.second;
        r[i->first] = t;
    }
    return r;
}

std::set<node_ptr_t> Node::parents() const{
    lock_t l(m_parent_links_lock); 
    std::set<node_ptr_t> r;
    in_link_map_t::const_iterator i;
    for(i = m_parent_links.begin(); i != m_parent_links.end(); i++)
        r.insert(i->second.first);
    return r;
}


/*  overload for the common case where we're connecting a node with one
 *  output to a node with one input
 */ 
void Node::setOutput(node_ptr_t n){
    lock_t l(m_child_links_lock);

    if(m_child_links.size() == 1){
        const input_id_set_t child_inputs = n->inputs();
        if(child_inputs.size() != 1){
            throw(link_error("setOutput: specific child input must be specified"));
        }
        // An output can be connected to more than one input, so
        // m_child_links[output_id] is a list of output_link_t
        input_id child_in = *child_inputs.begin();
        debug() << BashColour::Green << "adding output link to child: " << n << child_in; 
        m_child_links.begin()->second.push_back(output_link_t(n, child_in));
    }else if(m_child_links.size() > 1){
        throw link_error("setOutput: specific output must be specified");
    }else{
        throw link_error("setOutput: this node has no outputs");
    }
}

void Node::setOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id){
    lock_t l(m_child_links_lock);
    const out_link_map_t::iterator i = m_child_links.find(o_id);
    if(i == m_child_links.end()){
        throw(id_error("setOutput: Invalid output id" + to_string(o_id)));
    }else{
        // An output can be connected to more than one input, so
        // m_child_links[output_id] is a list of output_link_t
        debug() << BashColour::Green << "adding output link to child: " << n << i_id;
        i->second.push_back(output_link_t(n, i_id));
    }
}

void Node::clearOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id){
    lock_t l(m_child_links_lock);
    const out_link_map_t::iterator i = m_child_links.find(o_id);
    if(i == m_child_links.end()){
        throw(id_error("clearOutput: Invalid output id" + to_string(o_id)));
    }else{
        // An output can be connected to more than one input, so
        // m_child_links[output_id] is a list of output_link_t
        output_link_list_t::iterator j = std::find(i->second.begin(), i->second.end(), output_link_t(n, i_id));
        if(j == i->second.end()){
            throw(id_error("clearOutput: Invalid node & input id: "
                           + to_string(m_pl.lookup(n)) + ", " 
                           + to_string(i_id)));
        }else{
            debug() << BashColour::Purple << "removing output link to child: " << j->first << j->second; 
            i->second.erase(j);
        }
    }
}

void Node::clearOutputs(node_ptr_t child){
    lock_t l(m_child_links_lock); 
    out_link_map_t::iterator i;
    output_link_list_t::iterator j, t;
    // NB: this only works because output_link_list_t is a std::list,
    // hence iterators aren't invalidated on removal of items
    for(i = m_child_links.begin(); i != m_child_links.end(); i++){
        j = i->second.begin();
        while(j != i->second.end()){
            if(j->first == child){
                t = j;
                j++;
                debug() << BashColour::Purple << "removing output link to child: " << j->first << j->second; 
                i->second.erase(t);
            }else{
                j++;
            }
        }
    }
}

void Node::clearOutputs(){
    lock_t l(m_child_links_lock);
    out_link_map_t::iterator i;
    for(i = m_child_links.begin(); i != m_child_links.end(); i++){
        debug() << BashColour::Purple << "removing output link to all children on:" << i->first;             
        i->second.clear();
    }
}

Node::output_id_set_t Node::outputs() const{
    output_id_set_t r;
    lock_t n(m_child_links_lock);
    out_link_map_t::const_iterator i;
    for(i = m_child_links.begin(); i != m_child_links.end(); i++)
        r.insert(i->first);
    return r;
}

Node::msg_node_output_map_t Node::outputLinks() const{
    lock_t l(m_child_links_lock);
    msg_node_output_map_t r;
    out_link_map_t::const_iterator i;
    for(i = m_child_links.begin(); i != m_child_links.end(); i++){
        msg_node_in_list_t input_list;
        output_link_list_t::const_iterator j;
        for(j = i->second.begin(); j != i->second.end(); j++){
            NodeInput t;
            t.node = m_pl.lookup(j->first);
            t.input = j->second;
            input_list.push_back(t);
        }
        r[i->first] = input_list;
    }
    return r;
}

std::set<node_ptr_t> Node::children() const{
    lock_t l(m_child_links_lock);
    std::set<node_ptr_t> r;
    out_link_map_t::const_iterator i;
    output_link_list_t::const_iterator j;
    for(i = m_child_links.begin(); i != m_child_links.end(); i++)
        for(j = i->second.begin(); j != i->second.end(); j++)
            r.insert(j->first);
    return r;
}

int Node::numChildren() const{
    int r = 0;
    lock_t l(m_child_links_lock);
    out_link_map_t::const_iterator i;
    for(i = m_child_links.begin(); i != m_child_links.end(); i++)
        r += i->second.size();
    return r;
}

void Node::exec(){
    // take copies of image_ptr s from parents before _demandNewParentInput()
    in_image_map_t inputs;
    out_image_map_t outputs;

    m_parent_links_lock.lock();
    for(in_link_map_t::const_iterator i = m_parent_links.begin(); i != m_parent_links.end(); i++){
        inputs[i->first] = i->second.first->getOutputImage(i->second.second);
        if(!inputs[i->first]){
            m_parent_links_lock.unlock();
            return;
        }
    }
    // Record that we've used all of our inputs
    m_new_inputs_lock.lock();
    BOOST_FOREACH(in_bool_map_t::value_type& v, m_new_inputs)
        v.second = false;
    m_new_inputs_lock.unlock();
    
    m_parent_links_lock.unlock();


    debug() << "exec: speed" << m_speed
            << "inputs" << inputs.size()
            << "outputs" << m_outputs.size()
            << "parent links" << m_parent_links.size()
            << "child links" << m_child_links.size();
    try{
        if(this->m_speed < medium){
            // if this is a fast node: request new image from parents before executing
            _demandNewParentInput();
            outputs = this->doWork(inputs);
        }else{
            // if this is a slow node, request new images from parents after executing
            outputs = this->doWork(inputs);
            _demandNewParentInput();
        }
    }catch(std::exception& e){
        error() << "Error executing node: " << *this << "\n\t" << e.what();
    }
    
    m_outputs_lock.lock();
    m_outputs = outputs;
    m_outputs_lock.unlock();
    
    
    m_output_demanded_lock.lock();
    m_output_demanded = false;
    m_output_demanded_lock.unlock();
    
    // for each of this node's outputs
    BOOST_FOREACH(out_link_map_t::value_type& v, m_child_links){
        // v is a std::pair<output_id, std::list<...> >
        if(!m_outputs[v.first] ||
           !m_outputs[v.first]->cvMat().size().width ||
           !m_outputs[v.first]->cvMat().size().height){
            debug() << "exec() did not fill output:" << v.first;
            debug() << v.second.size() << "children will not be prompted";
        }else{
            debug() << "Prompting" << v.second.size() << "children of new output:";
            // for each node connected to the output
            BOOST_FOREACH(output_link_t& link, v.second){
                // link is a std::pair<node_ptr, input_id>
                debug() << "prompting new input to child on:" << v.first;
                // notify the node that it has new input
                link.first->newInput(link.second);
            }
        }
    }
    
    m_exec_queued_lock.lock();
    m_exec_queued = false;
    m_exec_queued_lock.unlock();
    

    checkAddSched();
}

/* Keep a record of which inputs are new (have changed since they were
 * last used by this node)
 * Check to see if this node should add itself to the scheduler queue
 */
void Node::newInput(input_id const& a){
    lock_t l(m_new_inputs_lock);
    const std::map<input_id, bool>::iterator i = m_new_inputs.find(a);
    
    if(i == m_new_inputs.end()){
        error() << a << "invalid";
        error() << "valid inputs:";
        BOOST_FOREACH(in_bool_map_t::value_type const& v, m_new_inputs)
            error() << v.second;

        throw(id_error("newInput: Invalid input id: " + to_string(a)));
    }else{
        debug() << BashColour::Green << this << "notified of new input: " << a;
        i->second = true;
        m_valid_inputs[a] = true;
    }
    l.unlock();
    checkAddSched();
}

/* mark all inputs as new
 */
void Node::newInput(){
    lock_t m(m_new_inputs_lock); 
    debug() << BashColour::Green << this << "notified all inputs new";        
    std::map<input_id, bool>::iterator i;
    for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++)
        i->second = true;
    m.unlock();
    checkAddSched();
}

/* This is called by the children of this node in order to request new
 * output. It may be called at the start or end of the child's exec()
 */
void Node::demandNewOutput(/*output_id ?*/) throw(){
    lock_t l(m_output_demanded_lock);
    if(!m_output_demanded){
        m_output_demanded = true;
        lock_t m(m_new_inputs_lock);
        lock_t n(m_parent_links_lock);
        BOOST_FOREACH(in_bool_map_t::value_type& v, m_new_inputs){
            if(!v.second && m_parent_links[v.first].first)
                m_parent_links[v.first].first->demandNewOutput();
        }
        l.unlock();
        m.unlock();
        n.unlock();
        checkAddSched();
    }
}

/* Get the actual image data associated with an output
 */
Node::image_ptr_t Node::getOutputImage(output_id const& o_id) const throw(id_error){
    lock_t l(m_outputs_lock);
    const out_image_map_t::const_iterator i = m_outputs.find(o_id);
    if(i == m_outputs.end() || !i->second){
        debug() << __func__ << "non-existent output requested, returning NULL";
        return image_ptr_t();
    }else{
        return i->second;
    }
}


static NodeParamValue toNPV(param_value_t const& v){
    NodeParamValue r;
    try{
        r.intValue = boost::get<int>(v);
        r.type = ParamType::Int32;
    }catch(boost::bad_get&){}
    try{
        r.floatValue = boost::get<float>(v);
        r.type = ParamType::Float;
    }catch(boost::bad_get&){}
    try{
        r.stringValue = boost::get<std::string>(v);
        r.type = ParamType::String;
    }catch(boost::bad_get&){}
    return r;
}

/* return all parameter values
 */
std::map<param_id, NodeParamValue> Node::parameters() const{
    lock_t l(m_parameters_lock);
    std::map<param_id, NodeParamValue> r;
    param_value_map_t::const_iterator i;
    for(i = m_parameters.begin(); i != m_parameters.end(); i++)
        r[i->first] = toNPV(i->second);
    return r;
}

/* set a parameter based on a message
 */
void Node::setParam(boost::shared_ptr<const SetNodeParameterMessage>  m){
    param_value_t value;
    switch((ParamType::e)m->value().type){
        case ParamType::Int32:  value = m->value().intValue; break;
        case ParamType::Float:  value = m->value().floatValue; break;
        case ParamType::String: value = m->value().stringValue; break;
        default:
            // TODO: throw?
            error() << "Unknown parameter type:" << m->value().type;
    }
    setParam(m->paramId(), value);
}

void Node::registerOutputID(output_id const& o){
    lock_t l(m_child_links_lock);
    lock_t m(m_outputs_lock);
    
    m_child_links[o] = output_link_list_t();
    m_outputs[o] = image_ptr_t();
}

void Node::registerInputID(input_id const& i){
    lock_t l(m_new_inputs_lock);
    lock_t m(m_valid_inputs_lock);
    lock_t n(m_parent_links_lock);

    m_new_inputs[i] = false;
    m_valid_inputs[i] = false;
    m_parent_links[i] = input_link_t();
}

/* Check to see if all inputs are new and output is demanded; if so, 
 * add this node to the scheduler queue
 */
void Node::checkAddSched() throw(){
    std::map<input_id, bool>::const_iterator i;
    lock_t li(m_output_demanded_lock);
    lock_t lo(m_new_inputs_lock);
    lock_t lr(m_exec_queued_lock);
    
    if(!allowQueueExec()){
        debug() << "Cannot enqueue node" << this << ", allowQueueExec failed"; 
        return;
    }

    if(m_exec_queued){
        debug() << "Cannot enqueue node" << this << ", exec queued already"; 
        return;
    }

    if(!isOutputNode() && !m_output_demanded){
        debug() << "Cannot enqueue node" << this << ", no output demanded"; 
        return;
    }
    
    // ALL inputs must be new
    for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++){
        if(!i->second){
            debug() << "Cannot enqueue node" << this << ", input is old";
            return;
        }
    }

    debug() << "Queuing node:" << *this;

    // if all inputs are new, all inputs are valid
    
    // we rely on multiple-reader thread-safety of std::map here,
    // which is only true if we aren't creating new key-value pairs
    // using operator[] (which we aren't, and doing so would return a
    // NULL queue pointer anyway)
    m_exec_queued = true;
    m_sched.addJob(this, m_priority);
}

bool Node::_allInputsValid() const throw(){
    lock_t l(m_valid_inputs_lock);
    BOOST_FOREACH(in_bool_map_t::value_type const& v, m_valid_inputs)
        if(!v.second) return false;
    return true;
}

void Node::_demandNewParentInput() throw(){
    lock_t l(m_parent_links_lock);
    in_link_map_t::const_iterator i;
    debug() << "node" << this << "demanding new output from all parents";
    for(i = m_parent_links.begin(); i != m_parent_links.end(); i++){
        if(i->second.first)
            i->second.first->demandNewOutput();
    }
}


