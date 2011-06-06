#include "node.h"

#include "imageProcessor.h"
#include "pipelineTypes.h"
#include "nodeFactory.h"

using namespace cauv;
using namespace cauv::imgproc;

const char* Node::Image_In_Name = "image in";
const char* Node::Image_Out_Name = "image out (not copied)";
const char* Node::Image_Out_Copied_Name = "image out";


Node::Node(Scheduler& sched, ImageProcessor& pl, std::string const& pl_name, NodeType::e type)
    : m_priority(priority_slow),
      m_speed(slow),
      m_node_type(type),
      m_id(_newID()),
      m_inputs(),
      m_inputs_lock(),
      m_outputs(),
      m_outputs_lock(),
      m_output_demanded_on(),
      m_output_demanded_on_lock(),
      m_checking_sched_lock(),
      m_exec_queued(false),
      m_exec_queued_lock(),
      m_allow_queue(true),
      m_allow_queue_lock(),

      m_sched(sched),
      m_pl(pl),
      m_pl_name(pl_name){
}

Node::~Node(){
    debug() << "~Node" << *this;
}

void Node::stop(){
    debug(-3) << BashColour::Purple << "stop()" << *this << "check...";
    // check that the node is not executing: if it is then there should still
    // be a shared pointer hanging around, and this node should not be being
    // destroyed, so complain loudly!
    int err = 0;
    if(!m_inputs_lock.try_lock())       err |= 1;
    if(!m_outputs_lock.try_lock())      err |= 1 << 1;
    if(!m_output_demanded_on_lock.try_lock())   err |= 1 << 2;
    if(!m_checking_sched_lock.try_lock())       err |= 1 << 3;
    if(!m_exec_queued_lock.try_lock())  err |= 1 << 4;
    if(!m_allow_queue_lock.try_lock())  err |= 1 << 5;
    
    if(err)
        error() << this << "SUPER BADNESS: destruct with locked mutexes:" << err;

    debug(-3) << BashColour::Purple << "stop()" << *this << ", done";
}

NodeType::e const& Node::type() const{
    return m_node_type;
}

node_id const& Node::id() const{
    return m_id;
}

std::string const& Node::plName() const{
    return m_pl_name;
}

void Node::setInput(input_id const& i_id, node_ptr_t n, output_id const& o_id){
    unique_lock_t l(m_inputs_lock);
    const private_in_map_t::iterator i = m_inputs.find(i_id);
    if(i == m_inputs.end()){
        throw id_error("setInput: Invalid input id" + toStr(i_id));
    }else if(n->id() == id()){
        throw link_error("sorry, can't link nodes to themselves: blame shared mutexes being non-recursive");
    }else if(i->second.isParam() != !!(n->paramOutputs().count(o_id))){
        throw link_error("setInput: Parameter <==> Image mismatch");
    }else{
        if(i->second.target && i->second.target != input_link_t(n, o_id)){
            debug() << i->second << std::endl
                    << i->second.target << "!=" << input_link_t(n, o_id);
            throw link_error("old arc must be removed first");
        }
        debug(-3) << BashColour::Green << "adding parent link on" << i_id << "->" << *n << o_id;
        i->second.status = NodeInputStatus::Old;
        i->second.target = input_link_t(n, o_id);
        if(i->second.target){
            l.unlock();
            setNewInput(i_id);
        }else{
            n->setNewOutputDemanded(o_id);
        }
    }
}

void Node::clearInput(input_id const& i_id){
    unique_lock_t l(m_inputs_lock);
    const private_in_map_t::iterator i = m_inputs.find(i_id);
    if(i != m_inputs.end()){
        debug(-3) << BashColour::Purple << *this << "removing parent link on " << i_id;    
        i->second.target.clear();
    }else
        throw id_error("clearInput: Invalid input id" + toStr(i_id));
}

void Node::clearInputs(node_ptr_t parent){
    unique_lock_t l(m_inputs_lock);
    debug(-3) << BashColour::Purple << *this << "removing parent links to" << *parent;    
    foreach(private_in_map_t::value_type& v, m_inputs)
        if(v.second.target.node == parent)
            v.second.target.clear();
}

void Node::clearInputs(){
    unique_lock_t l(m_inputs_lock);
    debug(-3) << BashColour::Purple << *this << "removing all parent links";    
    foreach(private_in_map_t::value_type& v, m_inputs)
        v.second.target.clear();
}

Node::input_id_set_t Node::inputs() const{
    shared_lock_t l(m_inputs_lock);
    input_id_set_t r;
    foreach(private_in_map_t::value_type const& v, m_inputs)
        if(!v.second.isParam()) // parameters don't count!
            r.insert(v.first);
    return r;
}

Node::msg_node_input_map_t Node::inputLinks() const{
    shared_lock_t l(m_inputs_lock);
    msg_node_input_map_t r;
    foreach(private_in_map_t::value_type const& v, m_inputs){
        const input_id& id = v.first;
        const input_link_t& link = v.second.target;
        // parameters _do_ count
        NodeOutput t;
        t.node = m_pl.lookup(link.node);
        t.output = link.id;
        // setInput enforces links are the same type (Image/Parameter) at both
        // ends, so we can check the type of the output from the type of the
        // input
        if(v.second.isParam())
            t.type = OutputType::Parameter;
        else
            t.type = OutputType::Image;
        r[id] = t;
    }
    return r;
}

std::set<node_ptr_t> Node::parents() const{
    shared_lock_t l(m_inputs_lock);
    std::set<node_ptr_t> r;    
    foreach(private_in_map_t::value_type const& i, m_inputs)
        if(i.second.target)
            r.insert(i.second.target.node); // parameters _do_count
    return r;
}

void Node::setOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id){
    unique_lock_t l(m_outputs_lock);
    const private_out_map_t::iterator i = m_outputs.find(o_id);
    if(i == m_outputs.end()){
        throw id_error("setOutput: Invalid output id" + toStr(o_id));
    }else if(i->second.isParam() != n->parameters().count(i_id)){
        throw link_error("setInput: Parameter <==> Image mismatch");
    }else if(n->id() == id()){
        throw link_error("sorry, can't link nodes to themselves: blame shared mutexes being non-recursive");
    }else{
        debug(-3) << BashColour::Green << *this << "adding output link to child: " << *n << i_id;
        i->second.targets.push_back(output_link_t(n, i_id));
    }
}

void Node::clearOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id){
    unique_lock_t l(m_outputs_lock);
    const private_out_map_t::iterator i = m_outputs.find(o_id);
    if(i == m_outputs.end()){
        throw id_error("clearOutput: Invalid output id" + toStr(o_id));
    }else{
        output_link_list_t::iterator j = std::find(
            i->second.targets.begin(),
            i->second.targets.end(),
            output_link_t(n, i_id)
        );
        if(j == i->second.targets.end()){
            throw id_error("clearOutput: Invalid node & input id: "
                           + toStr(m_pl.lookup(n)) + ", " + toStr(i_id));
        }else{
            debug(-3) << BashColour::Purple << *this << "removing output link to child:" << j->node << j->id;
            i->second.targets.erase(j);
        }
    }
}

template<typename T>
struct NodeIs{
    NodeIs(typename T::node_t const& node): m_node(node){ }
    bool operator()(T const& v){ return v.node == m_node; }
    typename T::node_t m_node;
};
void Node::clearOutputs(node_ptr_t child){
    unique_lock_t l(m_outputs_lock);
    debug(-3) << BashColour::Purple << *this << "removing output links to child:" << *child;
    foreach(private_out_map_t::value_type& i, m_outputs)
        i.second.targets.remove_if(NodeIs<output_link_t>(child));
}

void Node::clearOutputs(){
    unique_lock_t l(m_outputs_lock);
    foreach(private_out_map_t::value_type& i, m_outputs){
        debug(-3) << BashColour::Purple << *this << "removing output links from" << i.first;
        i.second.targets.clear();
    }
}

Node::output_id_set_t Node::outputs(int type_index) const{
    output_id_set_t r;
    shared_lock_t n(m_outputs_lock);
    foreach(private_out_map_t::value_type const& i, m_outputs)
        if(i.second.value.which() == type_index)
            r.insert(i.first);
    return r;
}

Node::output_id_set_t Node::outputs() const{
    return outputs(OutType_Image);
}

Node::output_id_set_t Node::paramOutputs() const{
    return outputs(OutType_Parameter);
}

Node::msg_node_output_map_t Node::outputLinks() const{
    shared_lock_t l(m_outputs_lock);
    msg_node_output_map_t r;
    foreach(private_out_map_t::value_type const& i, m_outputs){
        msg_node_in_list_t input_list;
        foreach(output_link_list_t::value_type const& j, i.second.targets)
            input_list.push_back(NodeInput(m_pl.lookup(j.node), j.id));
        r[i.first] = input_list;
    }
    return r;
}

std::set<node_ptr_t> Node::children() const{
    shared_lock_t l(m_outputs_lock);
    std::set<node_ptr_t> r;
    foreach(private_out_map_t::value_type const& i, m_outputs)
        foreach(output_link_list_t::value_type const& j, i.second.targets)
            r.insert(j.node);
    return r;
}

int Node::numChildren() const{
    shared_lock_t l(m_outputs_lock);
    int r = 0;
    foreach(private_out_map_t::value_type const& i, m_outputs)
        r += i.second.targets.size();
    return r;
}

// enum status mangling:
static NodeStatus::e& operator|=(NodeStatus::e& l, NodeStatus::e const& r){
    return l = NodeStatus::e(unsigned(l) | unsigned(r));
}
static NodeStatus::e operator|(NodeStatus::e const& l, NodeStatus::e const& r){
    return NodeStatus::e(unsigned(l) | unsigned(r));
}

//static NodeIOStatus::e& operator|=(NodeIOStatus::e& l, NodeIOStatus::e const& r){
//    return l = NodeIOStatus::e(unsigned(l) | unsigned(r));
//}
static NodeIOStatus::e operator|(NodeIOStatus::e const& l, NodeIOStatus::e const& r){
    return NodeIOStatus::e(unsigned(l) | unsigned(r));
}

// there must be a nicer way to do this...
#define CallOnDestruct(Type, member) \
struct _COD{_COD(Type& m):m(m){}~_COD(){m.member();}Type& m;}
void Node::exec(){
    CallOnDestruct(Node, clearExecQueued) cod(*this);
    in_image_map_t inputs;
    out_map_t outputs;

    shared_lock_t il(m_inputs_lock);
    
    try{
        foreach(private_in_map_t::value_type const& v, m_inputs){
            if(!v.second.isParam()){
                if(!v.second){
                    warning() << "exec: no parent or valid input on: " << v.first;
                    clearValidInput(v.first);
                    throw bad_input_error();
                }
                inputs[v.first] = v.second.getImage();
                if(!inputs[v.first]){
                    warning() << "exec: no output from: " << v.second.target << "->" << v.first;
                    clearValidInput(v.first);
                    throw bad_input_error();
                }
            }
        }
    }catch(bad_input_error&){
        return;
    }

    // Record that we've used all of our inputs with the current parameters
    clearNewInput();

    debug(4) << "exec:" << *this << "speed=" << m_speed << ", " << inputs.size() << "inputs";
    
    NodeStatus::e status = NodeStatus::None;
    if(allowQueue()) status |= NodeStatus::AllowQueue;
    _statusMessage(boost::make_shared<StatusMessage>(m_pl_name, m_id, status | NodeStatus::Executing));
    try{
        if(m_speed == asynchronous){
            // doWork will arrange for demandNewParentInput to be called when
            // appropriate
            outputs = this->doWork(inputs);
        }else if(this->m_speed < medium){
            // if this is a fast node: request new image from parents before executing
            demandNewParentInput();
            outputs = this->doWork(inputs);
        }else{
            // if this is a slow node, request new images from parents after executing
            outputs = this->doWork(inputs);
            demandNewParentInput();
        }
    }catch(std::exception& e){
        error() << "Error executing node: " << *this << "\n\t" << e.what();
    }catch(...){
        error() << "Evil error executing node: " << *this << "\n\t";
    }
    _statusMessage(boost::make_shared<StatusMessage>(m_pl_name, m_id, status));


    unique_lock_t ol(m_outputs_lock);
    foreach(out_map_t::value_type& v, outputs){
        private_out_map_t::iterator i = m_outputs.find(v.first);
        if(i == m_outputs.end()){
            error() << "exec() produced output at an unknown id:" << v.first
                    << "(ignored)";
            continue;
        }
        if(i->second.value.which() == v.second.which()){
            clearNewOutputDemanded(v.first);
            i->second.value = v.second;
            if(i->second.targets.size()){
                debug(5) << "Prompting" << i->second.targets.size()
                         << "children of new output:";
                // for each node connected to the output
                foreach(output_link_t& link, i->second.targets){
                    // notify the node that it has new input
                    if(link.node){
                        debug(5) << "prompting new input to child on:" << v.first;
                        link.node->setNewInput(link.id);
                    }else{
                        error() << "cannot prompt NULL link about output" << v.first;
                    }
                }
            }else{
                debug(5) << "no children to prompt for output" << v.first;
            }
        }else{
            error() << "exec() produced output of the wrong type for id:"
                    << v.first << "(ignored)";
        }
    }
    // warn if no outputs were filled
    if(0 == outputs.size() && m_outputs.size())
        warning() << *this << "exec() produced no output when some was expected";
    /* this is commented out because it is perfectly valid for a node to not
     * always produce output on all outputs (e.g. SonarInputNode)
    // warn about any outputs that weren't filled
    //foreach(out_link_map_t::value_type& v, m_child_links)
    //    if(!outputs.count(v.first))
    //        warning() << "exec() did not fill output:" << v.first << "\n\t"
    //                  << v.second.size() << "children will not be prompted";
     */
    ol.unlock();
}

/* Get the actual image data associated with an output
 */
Node::image_ptr_t Node::getOutputImage(output_id const& o_id,
                                       bool suppress_null_warning) const throw(id_error){
    shared_lock_t l(m_outputs_lock);
    const private_out_map_t::const_iterator i = m_outputs.find(o_id);
    image_ptr_t r;
    if(i != m_outputs.end()){
        try{
            r = boost::get<image_ptr_t>(i->second.value);
        }catch(boost::bad_get&){
            throw id_error("requested output is not an image_ptr_t" + toStr(o_id));
        }
    }else{
        throw id_error("no such output" + toStr(o_id));
    }
    if(!r && !suppress_null_warning)
        warning() << m_id << "returning NULL image for" << o_id;
    return r;
}

NodeParamValue Node::getOutputParam(output_id const& o_id) const throw(id_error){
    shared_lock_t l(m_outputs_lock);
    const private_out_map_t::const_iterator i = m_outputs.find(o_id);
    NodeParamValue r;
    if(i != m_outputs.end()){
        try{
            r = boost::get<NodeParamValue>(i->second.value);
        }catch(boost::bad_get&){
            throw id_error("requested output is not an NodeParamValue" + toStr(o_id));
        }
    }else{
        throw id_error("no such output" + toStr(o_id));
    }
    return r;
}

/* return all parameter values (without querying connected parents)
 */
std::map<input_id, NodeParamValue> Node::parameters() const{
    shared_lock_t l(m_outputs_lock);
    std::map<input_id, NodeParamValue> r;
    foreach(private_in_map_t::value_type const& v, m_inputs)
        if(v.second.isParam())
            r[v.first] = v.second.param_value;
    return r;
}

/* set a parameter based on a message
 */
void Node::setParam(boost::shared_ptr<const SetNodeParameterMessage>  m){
    debug(3) << *this << "setParam" << m;
    setParam(m->paramId(), m->value());
}

void Node::registerInputID(input_id const& i, InputSchedType const& st){
    unique_lock_t l(m_inputs_lock);
    // Avoid need for default Input constructor
    m_inputs.insert(
        private_in_map_t::value_type(i, Input::makeImageInput(st))
    );
    _statusMessage(boost::make_shared<InputStatusMessage>(m_pl_name, m_id, i, NodeIOStatus::None));
}

/* Check to see whether this node should be added to the scheduler queue
 */
void Node::checkAddSched(SchedMode m){
    unique_lock_t l(m_checking_sched_lock);
    if(m != Force && !allowQueue()){
        debug(4) << __func__ << "Cannot enqueue node" << *this << ", allowQueue false";
        return;
    }
    if(execQueued()){
        debug(4) << __func__ << "Cannot enqueue node" << *this << ", exec queued already";
        return;
    }

    const bool out_demanded = newOutputDemanded();
    const bool any_new_in = anyInputsAreNew();
    if(m != Always && m != Force && out_demanded == false && any_new_in == false){
        debug(4) << __func__ << "Cannot enqueue node" << *this
                 << "new input/params=" << any_new_in << "output demand=" << out_demanded;
        return;
    }

    if(!validInputAll()){
        debug(4) << __func__ << "Cannot enqueue node" << *this << ", input is invalid";
        return;
    }

    if(m == AllNew && !allRequiredInputsAreNew()){
        debug(4) << __func__ << "Cannot enqueue node" << *this << ", some input is old";
        return;
    }

    if(m == AnyNew && !anyRequiredInputsAreNew()){
        debug(4) << __func__ << "Cannot enqueue node" << *this << ", all input is old";
        return;
    }

    debug(4) << __func__ << "Queuing node (" << m << "):" << *this;
    setExecQueued();
    m_sched.addJob(shared_from_this(), m_priority);
}

void Node::sendMessage(boost::shared_ptr<Message const> m, service_t p) const {
    m_pl.sendMessage(m, p);
}

/* Keep a record of which inputs are new (have changed since they were
 * last used by this node)
 * Check to see if this node should add itself to the scheduler queue
 */
void Node::setNewInput(input_id const& a){
    // new input implies it's probably valid.. if not we'll just find out the
    // next time we come to exec()
    setValidInput(a);

    unique_lock_t l(m_inputs_lock);
    const private_in_map_t::iterator i = m_inputs.find(a);
    debug(5) << *this << "setting input new" << a;
    if(i == m_inputs.end()){
        throw id_error("newInput: Invalid input id: " + toStr(a));
    }else{
        i->second.status = NodeInputStatus::New;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_pl_name, m_id, a, NodeIOStatus::New | NodeIOStatus::Valid
        ));
    }
    l.unlock();
    checkAddSched();
}

/* mark all inputs as new
 */
void Node::setNewInput(){
    unique_lock_t l(m_inputs_lock);
    debug(5) << *this << "setting all inputs new";
    foreach(private_in_map_t::value_type& i, m_inputs){
        i.second.status = NodeInputStatus::New;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_pl_name, m_id, i.first, NodeIOStatus::New | NodeIOStatus::Valid
        ));
    }
    l.unlock();
    checkAddSched();
}

void Node::clearNewInput(){
    unique_lock_t m(m_inputs_lock);
    debug(5) <<  *this << "setting all inputs old";
    foreach(private_in_map_t::value_type& i, m_inputs){
        if(i.second.status != NodeInputStatus::Invalid){
            i.second.status = NodeInputStatus::Old;
            _statusMessage(boost::make_shared<InputStatusMessage>(
                m_pl_name, m_id, i.first, NodeIOStatus::Valid
            ));
        }
    }
}

/* all includes none! (if none must be new) */
bool Node::allRequiredInputsAreNew() const{
    unique_lock_t m(m_inputs_lock);
    foreach(private_in_map_t::value_type const& i, m_inputs)
        if(i.second.sched_type == Must_Be_New && i.second.status != NodeInputStatus::New)
            return false;
    return true;
}

bool Node::anyRequiredInputsAreNew() const{
    unique_lock_t m(m_inputs_lock);
    foreach(private_in_map_t::value_type const& i, m_inputs)
        if(i.second.sched_type == Must_Be_New && i.second.status == NodeInputStatus::New)
            return true;
    return false;
}

bool Node::anyInputsAreNew() const{
    foreach(private_in_map_t::value_type const& i, m_inputs)
        if(i.second.status == NodeInputStatus::New)
            return true;
    return false;
}

void Node::setValidInput(input_id const& id){
    unique_lock_t l(m_inputs_lock);
    const private_in_map_t::iterator i = m_inputs.find(id);
    if(i == m_inputs.end()){
        throw id_error("clearValidInput: Invalid input id: " + toStr(id));
    }else if(i->second.status == NodeInputStatus::Invalid){
        i->second.status = NodeInputStatus::New;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_pl_name, m_id, id, NodeIOStatus::None
        ));
    }
    l.unlock();
    checkAddSched();
}

void Node::clearValidInput(input_id const& id){
    unique_lock_t l(m_inputs_lock);
    const private_in_map_t::iterator i = m_inputs.find(id);
    if(i == m_inputs.end()){
        throw id_error("clearValidInput: Invalid input id: " + toStr(id));
    }else if(i->second.status != NodeInputStatus::Invalid){
        i->second.status = NodeInputStatus::Invalid;
        _statusMessage(boost::make_shared<InputStatusMessage>(
            m_pl_name, m_id, id, NodeIOStatus::None
        ));
    }
}

bool Node::validInputAll() const{
    unique_lock_t l(m_inputs_lock);
    foreach(private_in_map_t::value_type const& i, m_inputs)
        if(!i.second)
            return false;
    return true;
}

/* This is called by the children of this node in order to request new
 * output. It may be called at the start or end of the child's exec()
 */
void Node::setNewOutputDemanded(output_id const& o){
    unique_lock_t l(m_output_demanded_on_lock);
    const bool output_demanded_already = !!m_output_demanded_on.size();
    m_output_demanded_on.insert(o);
    if(!output_demanded_already)
        _statusMessage(boost::make_shared<OutputStatusMessage>(
            m_pl_name, m_id, o, NodeIOStatus::Demanded
        ));
    l.unlock();
    checkAddSched();
}

void Node::clearNewOutputDemanded(output_id const& o){
    unique_lock_t l(m_output_demanded_on_lock);
    const bool output_demanded_before = !!m_output_demanded_on.size();    
    m_output_demanded_on.erase(o);
    if(output_demanded_before !=  !!m_output_demanded_on.size())
        _statusMessage(boost::make_shared<OutputStatusMessage>(
                m_pl_name, m_id, o, NodeIOStatus::e(0)
        ));
}

bool Node::newOutputDemanded() const{
    if(this->isOutputNode())
        return true;
    shared_lock_t l(m_output_demanded_on_lock);
    return m_output_demanded_on.size();
}

void Node::setAllowQueue(){
    unique_lock_t l(m_allow_queue_lock);
    m_allow_queue = true;
    NodeStatus::e status = NodeStatus::AllowQueue;
    if(execQueued()) status |= NodeStatus::ExecQueued;
    _statusMessage(boost::make_shared<StatusMessage>(m_pl_name, m_id, status));
    l.unlock();
    checkAddSched();
}

void Node::clearAllowQueue(){
    unique_lock_t l(m_allow_queue_lock);
    m_allow_queue = false;
    NodeStatus::e status = NodeStatus::e(0);
    if(execQueued()) status |= NodeStatus::ExecQueued;
    _statusMessage(boost::make_shared<StatusMessage>(m_pl_name, m_id, status));
}

bool Node::allowQueue() const{
    shared_lock_t l(m_allow_queue_lock);
    return m_allow_queue;
}

void Node::setExecQueued(){
    unique_lock_t l(m_exec_queued_lock);
    m_exec_queued = true;
    NodeStatus::e status = NodeStatus::ExecQueued;
    if(allowQueue()) status |= NodeStatus::AllowQueue;
    _statusMessage(boost::make_shared<StatusMessage>(m_pl_name, m_id, status));
}

void Node::clearExecQueued(){
    unique_lock_t l(m_exec_queued_lock);
    m_exec_queued = false;
    NodeStatus::e status = NodeStatus::e(0);
    if(allowQueue()) status |= NodeStatus::AllowQueue;
    _statusMessage(boost::make_shared<StatusMessage>(m_pl_name, m_id, status));
    l.unlock();
    checkAddSched();
}

bool Node::execQueued() const{
    shared_lock_t l(m_exec_queued_lock);
    return m_exec_queued;
}

void Node::demandNewParentInput() throw(){
    unique_lock_t l(m_inputs_lock);
    debug(5) << "node" << *this << "demanding new output from all parents";
    foreach(private_in_map_t::value_type& i, m_inputs)
        if(i.second.target)
            i.second.target.node->setNewOutputDemanded(i.second.target.id);
}

#ifndef NO_NODE_IO_STATUS
void Node::_statusMessage(boost::shared_ptr<Message const> m){
    m_pl.sendMessage(m);
}
#else
void Node::_statusMessage(boost::shared_ptr<Message const>){ }
#endif

node_id Node::_newID() throw(){
    static node_id id = 1;
    if(id == ~node_id(0)){
        error() << "run out of node IDs, starting to recycle";
        id = 1;
    }
    return id++;
}

