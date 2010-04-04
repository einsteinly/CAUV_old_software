#include "node.h"


Node::Node(Scheduler& sched, ImageProcessor& pl, NodeType::e type)
    : m_priority(priority_slow), m_speed(slow),
      m_node_type(type), m_id(_newID()),
      m_exec_queued(false), m_exec_queued_lock(),
      m_parent_links(), m_parent_links_lock(),
      m_child_links(), m_child_links_lock(),
      m_outputs(), m_outputs_lock(),
      m_parameters(), m_parameters_lock(),
      m_new_inputs(), m_new_inputs_lock(),
      m_valid_inputs(), m_valid_inputs_lock(),
      m_output_demanded(false), m_output_demanded_lock(),
      m_allow_queue(true), m_allow_queue_lock(),
      m_sched(sched), m_pl(pl){
}

Node::~Node(){
    debug(3) << BashColour::Purple << "~Node" << *this << ", waiting for pending exec";
    clearAllowQueue();
    // wait for any last execution of the node to finish
    m_exec_queued_lock.lock();
    while(m_exec_queued){
        m_exec_queued_lock.unlock();
        clearAllowQueue();
        boost::this_thread::sleep(boost::posix_time::milliseconds(20));
        m_exec_queued_lock.lock();
    }
    m_exec_queued_lock.unlock();
    debug(3) << BashColour::Purple << "~Node" << *this << ", done";
}


NodeType::e const& Node::type() const{
    return m_node_type;
}

node_id const& Node::id() const{
    return m_id;
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
        output_id parent_output = *parent_outputs.begin();
        i->second = input_link_t(n, parent_output);
        if(n->getOutputImage(parent_output)){
            debug() << "node" << *this << "input set, output available from" << *n;
            setNewInput(m_parent_links.begin()->first);
        }else{
            debug() << "node" << *this << "input set, demand new output on" << *n;
            n->setNewOutputDemanded(parent_output);
        }
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
        debug(3) << BashColour::Green << "adding parent link on" << i_id << "->" << *n << o_id;
        i->second = input_link_t(n, o_id);
        if(n->getOutputImage(o_id)){
            debug() << "node" << *this << "input set, output available from" << *n;
            setNewInput(m_parent_links.begin()->first);
        }else{
            debug() << "node" << *this << "input set, demand new output on" << *n;
            n->setNewOutputDemanded(o_id);
        }
        n->setNewOutputDemanded(o_id);
    }
}

void Node::clearInput(input_id const& i_id){
    lock_t l(m_parent_links_lock);
    const in_link_map_t::iterator i = m_parent_links.find(i_id);
    if(i == m_parent_links.end()){
        throw(id_error("clearInput: Invalid input id" + to_string(i_id)));
    }else{
        debug(3) << BashColour::Purple << "removing parent link on " << i_id;
        i->second = input_link_t();
    }
}

void Node::clearInputs(node_ptr_t parent){
    lock_t l(m_parent_links_lock);
    in_link_map_t::iterator i;
    debug(3) << BashColour::Purple << "removing parent links to" << *parent;
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
        debug(3) << BashColour::Green << "adding output link to child: " << *n << child_in;
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
        debug(3) << BashColour::Green << "adding output link to child: " << *n << i_id;
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
            debug(3) << BashColour::Purple << "removing output link to child:" << j->first << j->second;
            i->second.erase(j);
        }
    }
}

template<typename T>
struct FirstIs{
    FirstIs(typename T::first_type const& comp): m_comp(comp){ }
    bool operator()(T const& v){ return v.first == m_comp; }
    typename T::first_type m_comp;
};
void Node::clearOutputs(node_ptr_t child){
    lock_t l(m_child_links_lock);
    out_link_map_t::iterator i;
    debug(3) << BashColour::Purple << "removing output links to child:" << *child;
    for(i = m_child_links.begin(); i != m_child_links.end(); i++)
        i->second.remove_if(FirstIs<output_link_t>(child));
}

void Node::clearOutputs(){
    lock_t l(m_child_links_lock);
    out_link_map_t::iterator i;
    for(i = m_child_links.begin(); i != m_child_links.end(); i++){
        debug(3) << BashColour::Purple << "removing output link to children:" << i->first << "->" << i->second;
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

// there must be a nicer way to do this...
#define CallOnDestruct(Type, member) \
struct _COD{ _COD(Type& m):m(m){} ~_COD(){m.member();} Type& m; }

void Node::exec(){
    CallOnDestruct(Node, clearExecQueued) cod(*this);
    // take copies of image_ptr s from parents before _demandNewParentInput()
    in_image_map_t inputs;
    out_image_map_t outputs;

    lock_t pl(m_parent_links_lock);
    for(in_link_map_t::const_iterator i = m_parent_links.begin(); i != m_parent_links.end(); i++){
        if(!i->second.first){
            warning() << "exec: no parent on:" << i->first;
            clearInputValid(i->first);
            return;
        }
        inputs[i->first] = i->second.first->getOutputImage(i->second.second);
        if(!inputs[i->first]){
            warning() << "exec: no output from: " << i->second << "to" << i->first;
            return;
        }
    }
    // Record that we've used all of our inputs
    clearNewInput();

    pl.unlock();

    debug() << "exec: id=" << m_id << "type=" << m_node_type
            << "speed=" << m_speed << ", " << inputs.size() << "inputs";
    
    int status = 0;
    if(allowQueue()) status |= NodeStatus::AllowQueue;
    _statusMessage(boost::make_shared<StatusMessage>(m_id, status | NodeStatus::Executing));
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
    _statusMessage(boost::make_shared<StatusMessage>(m_id, status));

    m_outputs_lock.lock();
    m_outputs = outputs;
    m_outputs_lock.unlock();

    if(!this->isOutputNode())
        clearNewOutputDemanded();

    lock_t cl(m_child_links_lock);
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
                link.first->setNewInput(link.second);
            }
        }
    }
    cl.unlock();
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
    NodeParamValue r = {0,0,0,"",0};
    try{
        r.intValue = boost::get<int>(v);
        r.type = ParamType::Int32;
        return r;
    }catch(boost::bad_get&){}
    try{
        r.floatValue = boost::get<float>(v);
        r.type = ParamType::Float;
        return r;
    }catch(boost::bad_get&){}
    try{
        r.stringValue = boost::get<std::string>(v);
        r.type = ParamType::String;
        return r;
    }catch(boost::bad_get&){}
    try{
        r.boolValue = boost::get<bool>(v);
        r.type = ParamType::Bool;
        return r;
    }catch(boost::bad_get&){}
    // TODO: throw?
    error() << "bad parameter value (" << v
            << "), is the NodeParamValue struct out of sync with param_value_t?";
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
    using std::string;
    switch((ParamType::e)m->value().type){
        case ParamType::Int32:  value = (int)   m->value().intValue;    break;
        case ParamType::Float:  value = (float) m->value().floatValue;  break;
        case ParamType::String: value = (string)m->value().stringValue; break;
        case ParamType::Bool:   value = (bool)  m->value().boolValue;   break;
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

    _statusMessage(boost::make_shared<OutputStatusMessage>(m_id, o, 0));
}

void Node::registerInputID(input_id const& i){
    lock_t l(m_new_inputs_lock);
    lock_t m(m_valid_inputs_lock);
    lock_t n(m_parent_links_lock);

    m_new_inputs[i] = false;
    m_valid_inputs[i] = false;
    m_parent_links[i] = input_link_t();

    _statusMessage(boost::make_shared<InputStatusMessage>(m_id, i, 0));
}

/* Check to see if all inputs are new and output is demanded; if so,
 * add this node to the scheduler queue
 */
void Node::checkAddSched() throw(){
    lock_t li(m_output_demanded_lock);
    lock_t lo(m_new_inputs_lock);
    lock_t lr(m_exec_queued_lock);
    lock_t la(m_allow_queue_lock);

    if(!allowQueue()){
        debug() << "Cannot enqueue node" << *this << ", allowQueue == false";
        return;
    }

    if(execQueued()){
        debug() << "Cannot enqueue node" << *this << ", exec queued already";
        return;
    }

    if(!newOutputDemanded()){
        debug() << "Cannot enqueue node" << *this << ", no output demanded";
        return;
    }

    // ALL inputs must be new
    if(!newInputAll()){
        debug() << "Cannot enqueue node" << *this << ", input is old";
        return;
    }

    if(!validInputAll()){
        debug() << "Cannot enqueue node" << *this << ", input is invalid";
        return;
    }

    debug() << "Queuing node:" << *this;
    setExecQueued();

    m_sched.addJob(this, m_priority);
}

void Node::sendMessage(boost::shared_ptr<Message const> m){
    m_pl.sendMessage(m);
}

/* Keep a record of which inputs are new (have changed since they were
 * last used by this node)
 * Check to see if this node should add itself to the scheduler queue
 */
void Node::setNewInput(input_id const& a){
    lock_t l(m_new_inputs_lock);
    lock_t m(m_valid_inputs_lock);
    const std::map<input_id, bool>::iterator i = m_new_inputs.find(a);


    debug() << *this << "input new" << a;
    if(i == m_new_inputs.end()){
        error() << a << "invalid";
        error() << "valid inputs:";
        BOOST_FOREACH(in_bool_map_t::value_type const& v, m_new_inputs)
        error() << v.second;

        throw(id_error("newInput: Invalid input id: " + to_string(a)));
    }else{
        debug() << *this << "notified of new input: " << a;
        i->second = true;
        m_valid_inputs[a] = true;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_id, a, NodeIOStatus::New | NodeIOStatus::Valid
        ));
    }
    m.unlock();
    l.unlock();
    checkAddSched();
}

/* mark all inputs as new
 */
void Node::setNewInput(){
    lock_t m(m_new_inputs_lock);
    lock_t n(m_valid_inputs_lock);
    debug() << *this << "all inputs new";
    std::map<input_id, bool>::iterator i;
    for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++){
        i->second = true;
        m_valid_inputs[i->first] = true;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_id, i->first, NodeIOStatus::New | NodeIOStatus::Valid
        ));
    }
    n.unlock();
    m.unlock();
    checkAddSched();
}

void Node::clearNewInput(){
    lock_t m(m_new_inputs_lock);
    lock_t n(m_valid_inputs_lock);
    debug() <<  *this << "all inputs old";
    std::map<input_id, bool>::iterator i;
    for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++){
        i->second = false;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_id, i->first, m_valid_inputs[i->first]? NodeIOStatus::Valid : 0
        ));
    }
}

bool Node::newInputAll() const{
    lock_t m(m_new_inputs_lock);
    in_bool_map_t::const_iterator i;
    for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++)
        if(!i->second)
            return false;
    return true;
}

void Node::setValidInput(input_id const& i){
    using namespace NodeIOStatus;
    lock_t l(m_valid_inputs_lock);
    if(!m_valid_inputs[i]){
        lock_t l(m_new_inputs_lock);
        m_valid_inputs[i] = true;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_id, i, m_new_inputs[i]? New | Valid : Valid
        ));
    }
    l.unlock();
    checkAddSched();
}

void Node::clearInputValid(input_id const& i){
    lock_t l(m_valid_inputs_lock);
    if(m_valid_inputs[i]){
        lock_t l(m_new_inputs_lock);
        m_valid_inputs[i] = false;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_id, i, m_new_inputs[i]? NodeIOStatus::New : 0
        ));
    }
}

bool Node::validInputAll() const{
    lock_t l(m_valid_inputs_lock);
    BOOST_FOREACH(in_bool_map_t::value_type const& v, m_valid_inputs)
        if(!v.second) return false;
    return true;
}

/* This is called by the children of this node in order to request new
 * output. It may be called at the start or end of the child's exec()
 */
void Node::setNewOutputDemanded(output_id const& o){
    lock_t l(m_output_demanded_lock);
    if(!m_output_demanded){
        m_output_demanded = true;

        _statusMessage(boost::make_shared<OutputStatusMessage>(
            m_id, o, NodeIOStatus::Demanded
        ));

        lock_t m(m_new_inputs_lock);
        lock_t n(m_parent_links_lock);
        BOOST_FOREACH(in_bool_map_t::value_type& v, m_new_inputs){
            if(!v.second && m_parent_links[v.first].first)
                m_parent_links[v.first].first->setNewOutputDemanded(
                    m_parent_links[v.first].second
                );
        }
        m.unlock();
        n.unlock();
    }
    l.unlock();
    checkAddSched();
}

void Node::clearNewOutputDemanded(){
    lock_t l(m_output_demanded_lock);
    lock_t m(m_outputs_lock);
    m_output_demanded = false;
    out_image_map_t::const_iterator i;
    for(i = m_outputs.begin(); i != m_outputs.end(); i++)
        _statusMessage(boost::make_shared<OutputStatusMessage>(m_id, i->first, 0));
}

bool Node::newOutputDemanded() const{
    if(this->isOutputNode())
        return true;
    lock_t l(m_output_demanded_lock);
    return m_output_demanded;
}

void Node::setAllowQueue(){
    lock_t l(m_allow_queue_lock);
    m_allow_queue = true;
    int status = NodeStatus::AllowQueue;
    if(execQueued()) status |= NodeStatus::ExecQueued;
    _statusMessage(boost::make_shared<StatusMessage>(m_id, status));
    l.unlock();
    checkAddSched();
}

void Node::clearAllowQueue(){
    lock_t l(m_allow_queue_lock);
    m_allow_queue = false;
    int status = 0;
    if(execQueued()) status |= NodeStatus::ExecQueued;
    _statusMessage(boost::make_shared<StatusMessage>(m_id, status));
}

bool Node::allowQueue() const{
    lock_t l(m_allow_queue_lock);
    return m_allow_queue;
}

void Node::setExecQueued(){
    lock_t l(m_exec_queued_lock);
    m_exec_queued = true;
    int status = NodeStatus::ExecQueued;
    if(allowQueue()) status |= NodeStatus::AllowQueue;
    _statusMessage(boost::make_shared<StatusMessage>(m_id, status));
}

void Node::clearExecQueued(){
    lock_t l(m_exec_queued_lock);
    m_exec_queued = false;
    int status = 0;
    if(allowQueue()) status |= NodeStatus::AllowQueue;
    _statusMessage(boost::make_shared<StatusMessage>(m_id, status));
    l.unlock();
    checkAddSched();
}

bool Node::execQueued() const{
    lock_t l(m_exec_queued_lock);
    return m_exec_queued;
}

void Node::_demandNewParentInput() throw(){
    lock_t l(m_parent_links_lock);
    in_link_map_t::const_iterator i;
    debug() << "node" << *this << "demanding new output from all parents";
    for(i = m_parent_links.begin(); i != m_parent_links.end(); i++){
        if(i->second.first)
            i->second.first->setNewOutputDemanded(i->second.second);
    }
}

void Node::_statusMessage(boost::shared_ptr<Message const> m){
    #ifndef NO_NODE_IO_STATUS
    m_pl.sendMessage(m);
    #endif
}

node_id Node::_newID() throw(){
    static node_id id = 1;
    if(id == ~node_id(0)){
        error() << "run out of node IDs, starting to recycle";
        id = 1;
    }
    return id++;
}

