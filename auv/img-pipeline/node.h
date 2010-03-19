#ifndef __NODE_H__
#define __NODE_H__

#include <map>
#include <list>
#include <set>
#include <stdexcept>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/variant.hpp>
#include <boost/foreach.hpp>

#include <common/cauv_utils.h>
#include <common/messages.h>
#include <common/debug.h> 
#include <common/image.h> 
#include <common/bash_cout.h>

#include "imageProcessor.h"
#include "pipelineTypes.h"
#include "nodeFactory.h"

class Node{
        // Private typedefs
        typedef boost::shared_ptr<Node> node_ptr_t;

        typedef std::pair<node_ptr_t, input_id> output_link_t;
        typedef std::pair<node_ptr_t, output_id> input_link_t;

        typedef std::list<output_link_t> output_link_list_t;
        
        typedef std::map<output_id, output_link_list_t> out_link_map_t;
        typedef std::map<input_id, input_link_t> in_link_map_t;
        typedef std::map<input_id, bool> in_bool_map_t;

        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    protected:
        // Protected typedefs
        typedef boost::shared_ptr<Image> image_ptr_t;
        typedef std::map<output_id, image_ptr_t> out_image_map_t;
        typedef std::map<input_id, image_ptr_t> in_image_map_t;

    public:
        Node(Scheduler& sched)
            : m_priority(priority_slow), m_speed(slow),
              m_exec_queued(false), m_output_demanded(false), m_sched(sched){
        }
        virtual ~Node(){
        }
        
        void setInput(input_id const& i_id, node_ptr_t n, output_id const& o_id){
            lock_t l(m_parent_links_lock);
            const in_link_map_t::iterator i = m_parent_links.find(i_id);
            if(i == m_parent_links.end()){
                throw(id_error(std::string("setInput: Invalid input id") + to_string(i_id)));
            }else{
                i->second = input_link_t(n, o_id); 
                // TODO: if (and only if) there is already an image available
                // from the parent, we should call newInput here:
                //newInput(i->first);
                // else: demand new output from the parent
                debug() << "node" << this << "input set, demand new output on" <<  n;
                n->demandNewOutput();
            }
        }

        /* overload for the common case where we're connecting a node with one
         * output to a node with one input
         */
        void setInput(node_ptr_t n){
            lock_t l(m_parent_links_lock);
            
            if(m_parent_links.size() == 1){
                const std::vector<output_id> parent_outputs = n->getOutputIDs();
                if(parent_outputs.size() != 1){
                    std::cerr << "Parent has " << parent_outputs.size() << " outputs!" << std::endl;
                    throw link_error("setInput: specific parent output must be specified");
                }
                m_parent_links.begin()->second = input_link_t(n, parent_outputs[0]);
                // TODO: if (and only if) there is already an image available
                // from the parent, we should call newInput here:
                //newInput(m_parent_links.begin()->first); 
                // else: demand new output from the parent
                debug() << "node" << this << "input set, demand new output on" <<  n;
                n->demandNewOutput();
            }else if(m_parent_links.size() > 1){
                throw link_error("setInput: specific input must be specified");
            }else{
                throw link_error("setInput: This node accepts no inputs");
            }
        }

        void clearInput(input_id const& i_id){ 
            lock_t l(m_parent_links_lock);
            const in_link_map_t::iterator i = m_parent_links.find(i_id);
            if(i == m_parent_links.end()){
                throw(id_error(std::string("clearInput: Invalid input id") + to_string(i_id)));
            }else{
                i->second = input_link_t();
            }
        }

        void clearInputs(node_ptr_t parent){
            lock_t l(m_parent_links_lock);
            in_link_map_t::iterator i;
            for(i = m_parent_links.begin(); i != m_parent_links.end(); i++){
                if(i->second.first == parent)
                    i->second = input_link_t();
            }
        }
 
        void clearInputs(){
            lock_t l(m_parent_links_lock); 
            in_link_map_t::iterator i;
            for(i = m_parent_links.begin(); i != m_parent_links.end(); i++){
                i->second = input_link_t();
            }
        }

        std::set<node_ptr_t> parents() const{
            lock_t l(m_parent_links_lock); 
            std::set<node_ptr_t> r;
            in_link_map_t::const_iterator i;
            for(i = m_parent_links.begin(); i != m_parent_links.end(); i++)
                r.insert(i->second.first);
            return r;
        }

        void setOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id){
            lock_t l(m_child_links_lock);
            const out_link_map_t::iterator i = m_child_links.find(o_id);
            if(i == m_child_links.end()){
                throw(id_error(std::string("setOutput: Invalid output id") + to_string(o_id)));
            }else{
                // An output can be connected to more than one input, so
                // m_child_links[output_id] is a list of output_link_t
                debug() << BashColour::Green << "adding output link to child: " << n << i_id;
                i->second.push_back(output_link_t(n, i_id));
            }
        }

        /*  overload for the common case where we're connecting a node with one
         *  output to a node with one input
         */ 
        void setOutput(node_ptr_t n){
            lock_t l(m_child_links_lock);

            if(m_child_links.size() == 1){
                const std::vector<input_id> child_inputs = n->getInputIDs();
                if(child_inputs.size() != 1){
                    throw(link_error("setOutput: specific child input must be specified"));
                }
                // An output can be connected to more than one input, so
                // m_child_links[output_id] is a list of output_link_t
                debug() << BashColour::Green << "adding output link to child: " << n << child_inputs[0]; 
                m_child_links.begin()->second.push_back(output_link_t(n, child_inputs[0]));
            }else if(m_child_links.size() > 1){
                throw link_error("setOutput: specific output must be specified");
            }else{
                throw link_error("setOutput: this node has no outputs");
            }
        }

        void clearOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id){
            lock_t l(m_child_links_lock);
            const out_link_map_t::iterator i = m_child_links.find(o_id);
            if(i == m_child_links.end()){
                throw(id_error(std::string("clearOutput: Invalid output id") + to_string(o_id)));
            }else{
                // An output can be connected to more than one input, so
                // m_child_links[output_id] is a list of output_link_t
                output_link_list_t::iterator j = std::find(i->second.begin(), i->second.end(), output_link_t(n, i_id));
                if(j == i->second.end()){
                    throw(id_error("clearOutput: Invalid node & input id: (node ID lookup is TODO): " + to_string(i_id)));
                }else{
                    debug() << BashColour::Purple << "removing output link to child: " << j->first << j->second; 
                    i->second.erase(j);
                }
            }
        }

        void clearOutputs(node_ptr_t child){
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

        void clearOutputs(){
            lock_t l(m_child_links_lock);
            out_link_map_t::iterator i;
            for(i = m_child_links.begin(); i != m_child_links.end(); i++){
                debug() << BashColour::Purple << "removing output link to all children on:" << i->first;             
                i->second.clear();
            }
        }

        std::set<node_ptr_t> children() const{
            lock_t l(m_child_links_lock);
            std::set<node_ptr_t> r;
            out_link_map_t::const_iterator i;
            output_link_list_t::const_iterator j;
            for(i = m_child_links.begin(); i != m_child_links.end(); i++)
                for(j = i->second.begin(); j != i->second.end(); j++)
                    r.insert(j->first);
            return r;
        }

        int numChildren() const{
            int r = 0;
            lock_t l(m_child_links_lock);
            out_link_map_t::const_iterator i;
            for(i = m_child_links.begin(); i != m_child_links.end(); i++)
                r += i->second.size();
            return r;
        }
        
        void exec(){
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
            if(this->m_speed < medium){
                // if this is a fast node: request new image from parents before executing
                _demandNewParentInput();
                outputs = this->doWork(inputs);
            }else{
                // if this is a slow node, request new images from parents after executing
                outputs = this->doWork(inputs);
                _demandNewParentInput();
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
        void newInput(input_id const& a){
            lock_t l(m_new_inputs_lock);
            const std::map<input_id, bool>::iterator i = m_new_inputs.find(a);
            
            if(i == m_new_inputs.end()){
                error() << a << "invalid";
                error() << "valid inputs:";
                BOOST_FOREACH(in_bool_map_t::value_type const& v, m_new_inputs)
                    error() << v.second;

                throw(id_error(std::string("newInput: Invalid input id: ") + to_string(a)));
            }else{
                debug() << BashColour::Green << this << "notified of new input: " << a;
                i->second = true;
                m_valid_inputs[a] = true;
            }
            checkAddSched();
        }
    
        /* mark all inputs as new
         */
        void newInput(){
            debug() << BashColour::Green << this << "notified all inputs new";        
            std::map<input_id, bool>::iterator i;
            for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++)
                i->second = true;
            checkAddSched();
        }
        
        /* This is called by the children of this node in order to request new
         * output. It may be called at the start or end of the child's exec()
         */
        void demandNewOutput(/*output_id ?*/) throw(){
            lock_t l(m_output_demanded_lock);
            if(!m_output_demanded){
                m_output_demanded = true;
                lock_t m(m_new_inputs_lock);
                lock_t n(m_parent_links_lock);
                BOOST_FOREACH(in_bool_map_t::value_type& v, m_new_inputs){
                    if(!v.second)
                        m_parent_links[v.first].first->demandNewOutput();
                }
                checkAddSched();
            }
        }
        
        /* Get the actual image data associated with an output
         */
        image_ptr_t getOutputImage(output_id const& o_id) const throw(id_error){
            lock_t l(m_outputs_lock);
            const out_image_map_t::const_iterator i = m_outputs.find(o_id);
            if(i == m_outputs.end() || !i->second){
                debug() << __func__ << "non-existent output requested, returning NULL";
                return image_ptr_t();
            }else{
                return i->second;
            }
        }
        
        /* Return a list of valid outputs from this node
         */
        std::vector<output_id> getOutputIDs() const throw(){
            lock_t l(m_outputs_lock);
            std::vector<output_id> r;
            out_image_map_t::const_iterator i;
            for(i = m_outputs.begin(); i != m_outputs.end(); i++)
                r.push_back(i->first);
            return r;
        }
        
        /* Return a list of valid inputs to this node
         */
        std::vector<input_id> getInputIDs() const throw(){
            lock_t l(m_new_inputs_lock);
            std::vector<input_id> r;
            in_bool_map_t::const_iterator i;
            for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++)
                r.push_back(i->first);
            return r;
        }
        
        /* return all parameter values
         */
        std::map<param_id, param_value_t> getParamValues() const;
        
        /* set a parameter based on a message
         */
        void setParam(boost::shared_ptr<SetNodeParameterMessage>  m){
            param_value_t value;
            switch(m->paramType()){
                case pt_int32: value = m->intValue(); break;
                case pt_float: value = m->floatValue(); break;
                case pt_string: value = m->stringValue(); break;
                default:
                    // TODO: throw?
                    error() << "Unknown parameter type:" << m->paramType();
            }
            setParam(m->paramId(), value);
        }

        /* set a single parameter value
         */
        template<typename T>
        void setParam(param_id const& p, T const& v) throw(id_error){
            lock_t l(m_parameters_lock);
            std::map<param_id, param_value_t>::iterator i = m_parameters.find(p);
            if(i != m_parameters.end()){
                debug() << "param" << p << "set to" << v;
                i->second = v;
            }else{
                error e;
                e << m_parameters.size() << "valid parameters are:";
                for(i = m_parameters.begin(); i != m_parameters.end(); i++)
                    e << i->first << "( =" << i->second << ")";
                throw(id_error(std::string("setParam: Invalid parameter id: ") + to_string(p)));
            }
            // provide notification that parameters have changed: principally
            // for asynchronous nodes
            this->paramChanged<T>(p);
            // if all inputs are valid (but not necessarily still new), add
            // this node to the scheduler queue by pretending all input is new
            if(_allInputsValid())
                newInput();
        }

        template<typename T>
        void paramChanged(param_id const&){ }
        
        /* return a single parameter value
         */
        template<typename T>
        T param(param_id const& p) const throw(id_error){
            lock_t l(m_parameters_lock);
            const std::map<param_id, param_value_t>::const_iterator i = m_parameters.find(p);
            if(i != m_parameters.end()){
                return boost::get<T>(i->second);
            }else{
                throw(id_error(std::string("param: Invalid parameter id: ") + to_string(p)));
            }
        }
        
        /* input nodes need to be identified so that onImageMessage() can be
         * efficiently called on only input nodes
         */
        virtual bool isInputNode() throw() { return false; }
        
       /* output nodes ignore m_output_demanded: they always execute whenever
        * there is new input.
        */
        virtual bool isOutputNode() throw() { return false; }

    protected:
        /* Derived classes override this to do whatever image processing it is
         * that they do.
         */
        virtual out_image_map_t doWork(in_image_map_t&) = 0;
                
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
        void registerParamID(param_id const& p, T const& default_value){
            lock_t l(m_parameters_lock);
            m_parameters[p] = param_value_t(default_value);
        }
        void registerOutputID(output_id const& o){
            lock_t l(m_child_links_lock);
            lock_t m(m_outputs_lock);
            
            m_child_links[o] = output_link_list_t();
            m_outputs[o] = image_ptr_t();
        }
        void registerInputID(input_id const& i){
            lock_t l(m_new_inputs_lock);
            lock_t m(m_valid_inputs_lock);
            lock_t n(m_parent_links_lock);

            m_new_inputs[i] = false;
            m_valid_inputs[i] = false;
            m_parent_links[i] = input_link_t();
        }
        
        /* allow derived types to stop nodes from being queued for execution:
         * use with care, excessive use could stop the pipeline.
         */
        virtual bool allowQueueExec() throw(){
            return true;
        }

        /* Check to see if all inputs are new and output is demanded; if so, 
         * add this node to the scheduler queue
         */
        void checkAddSched() throw(){
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

            debug() << "queuing node" << this;

            // if all inputs are new, all inputs are valid
            
            // we rely on multiple-reader thread-safety of std::map here,
            // which is only true if we aren't creating new key-value pairs
            // using operator[] (which we aren't, and doing so would return a
            // NULL queue pointer anyway)
            m_exec_queued = true;
            m_sched.addJob(this, m_priority);
        }

    private:
        bool _allInputsValid() const throw(){
            lock_t l(m_valid_inputs_lock);
            BOOST_FOREACH(in_bool_map_t::value_type const& v, m_valid_inputs)
                if(!v.second) return false;
            return true;
        }

        void _demandNewParentInput() throw(){
            lock_t l(m_parent_links_lock);
            in_link_map_t::const_iterator i;
            debug() << "node" << this << "demanding new output from all parents";
            for(i = m_parent_links.begin(); i != m_parent_links.end(); i++)
                i->second.first->demandNewOutput();

            BOOST_FOREACH(in_link_map_t::value_type const& v, m_parent_links)
                v.second.first->demandNewOutput(); 
        }
            
        /* prevent nodes (esp. output nodes) from executing in more than one
         * thread at once
         */
        bool m_exec_queued;
        mutable boost::recursive_mutex m_exec_queued_lock;
        
        /* maps an input_id to an output of another node */
        in_link_map_t   m_parent_links;
        mutable boost::recursive_mutex m_parent_links_lock;
        
        /* maps an output_id to a list of inputs on other nodes */
        out_link_map_t  m_child_links;
        mutable boost::recursive_mutex m_child_links_lock;
        
        /* maps an output_id to an image */
        out_image_map_t m_outputs;
        mutable boost::recursive_mutex m_outputs_lock;
        
        /* parameters of the filters */
        std::map<param_id, param_value_t> m_parameters;
        mutable boost::recursive_mutex m_parameters_lock;
        
        /* Keep track of which of our inputs have been refreshed since this node
         * was last exec()d
         * Set by newInput(), checked by checkAddSched(), cleared by exec()
         */
        in_bool_map_t m_new_inputs;
        mutable boost::recursive_mutex m_new_inputs_lock;

        in_bool_map_t m_valid_inputs;
        mutable boost::recursive_mutex m_valid_inputs_lock;
        
        /* Has output been demanded of this node?
         * Set by demandNewOutput(), checked by checkAddSched(), cleared by exec()
         */
        bool m_output_demanded;
        mutable boost::recursive_mutex m_output_demanded_lock;
        
        /* The scheduler associated with this node:
         * This is used by newInput() and demandNewOutput(), each of which may
         * decide that this node now needs to be executed (this->exec()), so
         * they add this node to a scheduler queue using:
         *		_sched->addToQueue(this, _priority);
         */
        Scheduler& m_sched;
};

#endif // ndef __NODE_H__
