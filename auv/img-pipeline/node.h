#ifndef __NODE_H__
#define __NODE_H__

#include <map>
#include <list>
#include <set>
#include <stdexcept>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/variant.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <utility/string.h>
#include <common/cauv_utils.h>
#include <generated/messages_messages.h>
#include <common/image.h>
#include <debug/cauv_debug.h>

// TODO: remove this dependency
#include <ssrc/spread.h>

#include "pipelineTypes.h"

namespace cauv{
namespace imgproc{

class Node: public boost::enable_shared_from_this<Node>{
    public:
        // Public typedefs: used as return types
        typedef std::vector<NodeInput> msg_node_in_list_t;
        typedef std::map<output_id, msg_node_in_list_t> msg_node_output_map_t;
        typedef std::map<input_id, NodeOutput> msg_node_input_map_t;

        typedef std::map<param_id, std::string> param_tip_map_t;

        typedef std::set<output_id> output_id_set_t;
        typedef std::set<input_id> input_id_set_t;

    protected:
        // Protected typedefs: useful for derived nodes
        typedef boost::shared_ptr<Image> image_ptr_t;
        
        // NB: order here is important, don't change it!
        typedef boost::variant<image_ptr_t, param_value_t> output_t;

        typedef std::map<output_id, output_t> out_map_t;
        typedef std::map<input_id, image_ptr_t> in_image_map_t;

    private:
        // Private typedefs: only used internally
        typedef std::pair<node_ptr_t, input_id> output_link_t;
        typedef std::pair<node_ptr_t, output_id> input_link_t;

        typedef std::list<output_link_t> output_link_list_t;

        typedef std::map<param_id, param_value_t> param_value_map_t;
        typedef std::map<input_id, bool> in_bool_map_t;
        typedef std::map<param_id, bool> param_bool_map_t;
        typedef std::map<output_id, output_link_list_t> out_link_map_t;
        typedef std::map<input_id, input_link_t> in_link_map_t;
        
        /**** SHARED LOCKS ARE NOT RECURSIVE ****/
        typedef boost::shared_mutex mutex_t;
        typedef boost::shared_lock<mutex_t> shared_lock_t;
        typedef boost::upgrade_lock<mutex_t> unique_lock_t;
        
        typedef Spread::service service_t;
    
    public:
        // TODO: shouldn't be necessary to pass `type' here!
        Node(Scheduler& sched, ImageProcessor& pl, NodeType::e type);

        /* non-trivial construction MUST be done in init(): non-trivial means
         * anything that relies on calling any method of this base class
         */
        virtual void init() = 0;


        virtual ~Node();

        /* Destructors of derived types that have any member variables should
         * call this FIRST.
         * This function should ONLY be called from a destructor.
         */
        void stop();

        NodeType::e const& type() const;
        node_id const& id() const;
        
        /* overload for the common case where we're connecting a node with one
         * output to a node with one input
         */
        void setInput(node_ptr_t n);
        void setInput(input_id const& i_id, node_ptr_t n, output_id const& o_id); 

        void clearInput(input_id const& i_id);
        void clearInputs(node_ptr_t parent);
        void clearInputs();

        input_id_set_t inputs() const; 
        msg_node_input_map_t inputLinks() const;
        std::set<node_ptr_t> parents() const;

        /*  overload for the common case where we're connecting a node with one
         *  output to a node with one input
         */ 
        void setOutput(node_ptr_t n);
        void setOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id); 

        void clearOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id); 
        void clearOutputs(node_ptr_t child); 
        void clearOutputs();
        
        output_id_set_t outputs(int type_index) const;
        output_id_set_t outputs() const;
        output_id_set_t paramOutputs() const;

        // TODO NPA: include param children
        msg_node_output_map_t outputLinks() const;

        // TODO NPA: include param children
        std::set<node_ptr_t> children() const;

        int numChildren() const;
        
        void exec();
        
        
        /* Get the actual image data associated with an output
         */
        image_ptr_t getOutputImage(output_id const& o_id,
                                   bool suppress_null_warning = false) const throw(id_error);
        
        /* Get the parameter associated with a parameter output
         */
        param_value_t getOutputParam(output_id const& o_id) const throw(id_error);
        
        /* return all parameter values
         */
        std::map<param_id, NodeParamValue> parameters() const;
        
        /* set a parameter based on a message
         */
        void setParam(boost::shared_ptr<const SetNodeParameterMessage>  m);

        /* set a single parameter value
         */
        template<typename T>
        void setParam(param_id const& p, T const& v) throw(id_error){
            unique_lock_t l(m_parameters_lock);
            param_value_map_t::iterator i = m_parameters.find(p);
            if(i != m_parameters.end()){
                debug(2) << "param" << p << "set to" << v;
                i->second = v;
            }else{
                error e;
                e << m_parameters.size() << "valid parameters are:";
                for(i = m_parameters.begin(); i != m_parameters.end(); i++)
                    e << i->first << "( =" << i->second << ")";
                throw(id_error("setParam: Invalid parameter id: " + toStr(p)));
            }
            // provide notification that parameters have changed: principally
            // for asynchronous nodes
            // NB:
            // release parameters lock first to reduce potential for deadlocks:
            // this means that the parameter we call the function with may no
            // longer be present when it's body is executed: node
            // implementations should not rely on it being valid
            l.unlock();
            this->paramChanged(p);
            // check to see if the node should be re-scheduled
            setNewParamValue(p);
        }
        
        /* Derived types overload this for notification of changed parameters:
         * this notification is only useful for asynchronous nodes.
         */
        virtual void paramChanged(param_id const&){ }
        
        /* return a single parameter value: retrieves value from parent if the
         * parameter is linked to the output of a parent
         */
        template<typename T>
        T param(param_id const& p) const {
            unique_lock_t l(m_parameters_lock);
            const param_value_map_t::const_iterator i = m_parameters.find(p);
            if(i != m_parameters.end()){
                const in_link_map_t::const_iterator j = m_parent_links.find(p);
                node_ptr_t node;
                if(j != m_parent_links.end() && (node = j->second.first)){
                    output_id outparam = j->second.second;
                    assert(node->paramOutputs().count(outparam));
                    debug(4) << "returning linked parameter value for" << p
                            << "(linked to" << j->second.first
                            << j->second.second << ")";
                    // TODO: this will throw boost::bad_get if there is a
                    // param_value type mismatch between the output and the
                    // requested parameter type:
                    //  prevent this happening (somehow...)
                    l.unlock();
                    try{
                        return boost::get<T>(node->getOutputParam(outparam));
                    }catch(boost::bad_get& e){
                        warning() << "parameter output not available / bad get:"
                                  << j->second.first << j->second.second
                                  << ", non-linked value will be returned";

                    }
                }
                return boost::get<T>(i->second);
            }else{
                throw(id_error("param: Invalid parameter id: " + toStr(p)));
            }
        }
        
        /* input nodes need to be identified so that onImageMessage() can be
         * efficiently called on only input nodes
         */
        virtual bool isInputNode() const { return false; }
        
       /* output nodes ignore m_output_demanded: they always execute whenever
        * there is new input.
        */
        virtual bool isOutputNode() const { return false; }
        
        /* Check to see if all inputs are new and output is demanded; if so, 
         * add this node to the scheduler queue
         */
        enum SchedMode{AllNew, AnyNew, Always};
        void checkAddSched(SchedMode m = AllNew);

    protected:
        /* Derived classes override this to do whatever image processing it is
         * that they do.
         */
        virtual out_map_t doWork(in_image_map_t&) = 0;
                
        /* Priority of this node; this might change dynamically.
         * Used when this node is added to a scheduler queue
         */
        SchedulerPriority m_priority;
    
        /* How long does this node take to run?
         * used by exec() to decide when to request new input from parents.
         */
        enum Speed {slow=0, medium=5, fast=10};
        Speed m_speed;
        
        /* Derived node types should call these functions (probably from their
         * constructors) in order to register valid input output and parameter
         * ids.
         */
        template<typename T>
        void registerParamID(param_id const& p, T const& default_value,
                             std::string const& tip=""){
            unique_lock_t l(m_parameters_lock);
            m_parameters[p] = param_value_t(default_value);
            m_parameter_tips[p] = tip;
            m_new_paramvalues[p] = true;
            // parameters are also inputs...
            registerInputID(p);
        }
        /* Template type is used to determine whether this is an image or a
         * parameter output: supported types
         *      - image_ptr_t
         *      - param_value_t
         */
        template<typename T>
        void registerOutputID(output_id const& o){
            unique_lock_t l(m_child_links_lock);
            unique_lock_t m(m_outputs_lock);
            
            m_child_links[o] = output_link_list_t();
            m_outputs[o] = output_t(T());
            
            m.unlock();
            l.unlock();
            _statusMessage(boost::make_shared<OutputStatusMessage>(
                m_id, o, NodeIOStatus::e(0)
            ));
        }
        void registerInputID(input_id const& i);

        void sendMessage(boost::shared_ptr<Message const>, service_t p = SAFE_MESS);
        
        /* Keep a record of which inputs are new (have changed since they were
         * last used by this node)
         * Check to see if this node should add itself to the scheduler queue
         */
        void setNewInput(input_id const&);
        void setNewInput(); 
        void clearNewInput();
        bool newInputAll() const;
        bool newInput() const;

        void setValidInput(input_id const&);
        void clearValidInput(input_id const&);
        bool validInputAll() const;
        
        /* This is called by the children of this node in order to request new
         * output. It may be called at the start or end of the child's exec()
         */
        void setNewOutputDemanded(output_id const&);
        void clearNewOutputDemanded(output_id const&);
        bool newOutputDemanded() const;

        void setAllowQueue();
        void clearAllowQueue();
        bool allowQueue() const;

        void setExecQueued();
        void clearExecQueued();
        bool execQueued() const;

        void setNewParamValue(param_id const&);
        void clearNewParamValues();
        bool newParamValues() const;

    private:
        void _demandNewParentInput() throw();
        void _statusMessage(boost::shared_ptr<Message const>);
        static node_id _newID() throw();
        
        const NodeType::e m_node_type;
        const node_id m_id;
       
        /* maps an input_id (including parameters) to an output of another node
         */
        in_link_map_t   m_parent_links;
        mutable mutex_t m_parent_links_lock;
        
        /* maps an output_id to a list of inputs (including parameters) on
         * other nodes
         */
        out_link_map_t  m_child_links;
        mutable mutex_t m_child_links_lock;
        
        /* maps an output_id to an image or parameter output */
        out_map_t m_outputs;
        mutable mutex_t m_outputs_lock;
        
        /* parameters of the filters */
        param_value_map_t m_parameters;
        param_tip_map_t m_parameter_tips;
        mutable mutex_t m_parameters_lock;


        /** Variables that control when the node is scheduled:
         **/

        /* Prevent checkAddSched from being run concurrently:
         */
        mutable mutex_t m_checking_sched_lock;
        
        /* prevent nodes (esp. output nodes) from executing in more than one
         * thread at once
         */
        bool m_exec_queued;
        mutable mutex_t m_exec_queued_lock;

        /* Keep track of which of our inputs have been refreshed since this node
         * was last exec()d
         */
        in_bool_map_t m_new_inputs;
        mutable mutex_t m_new_inputs_lock;
        
        /* Which inputs have valid data associated with them:
         */
        in_bool_map_t m_valid_inputs;
        mutable mutex_t m_valid_inputs_lock;

        /* Which parameters have been changed since this node was last exec()d
         */
        param_bool_map_t m_new_paramvalues;
        mutable mutex_t m_new_paramvalues_lock;
        
        /* Has output been demanded of this node?
         */
        bool m_output_demanded;
        std::set<output_id> m_output_demanded_on;
        mutable mutex_t m_output_demanded_lock;

        /* generic stop: used by derived types
         */
        bool m_allow_queue;
        mutable mutex_t m_allow_queue_lock;


        /** Other Variables:
         **/
        
        /* The scheduler associated with this node:
         * This is used by checkAddSched(), which may decide that this node now
         * needs to be executed (this->exec()), so it adds this node to a
         * scheduler queue using:
         *		m_sched.addToQueue(this, m_priority);
         */
        Scheduler& m_sched;
        
        /* The pipeline manager associated with this node:
         * Used for sending messages, and node pointer -> node id lookups
         */
        ImageProcessor& m_pl;
};

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, Node const& n){
    os << "{Node " << &n
       << " type=" << n.type()
       << " id=" << n.id()
       << "}";
    return os;
}

} // namespace imgproc
} // namespace cauv

#endif // ndef __NODE_H__
