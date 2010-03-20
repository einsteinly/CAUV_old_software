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

#include <common/cauv_utils.h>
#include <common/messages.h>
#include <common/debug.h> 
#include <common/image.h> 

#include "imageProcessor.h"
#include "pipelineTypes.h"
#include "nodeFactory.h"

class Node{
    public:
        // Public typedefs: used as return types
        typedef std::pair<node_ptr_t, input_id> output_link_t;
        typedef std::pair<node_ptr_t, output_id> input_link_t;

        typedef std::list<output_link_t> output_link_list_t;

        typedef std::map<output_id, output_link_list_t> out_link_map_t;
        typedef std::map<input_id, input_link_t> in_link_map_t;
        typedef std::map<param_id, param_value_t> param_value_map_t;

        typedef std::set<output_id> output_id_set_t;
        typedef std::set<input_id> input_id_set_t;
        

    protected:
        // Protected typedefs: useful for derived nodes
        typedef boost::shared_ptr<Image> image_ptr_t;

        typedef std::map<output_id, image_ptr_t> out_image_map_t;
        typedef std::map<input_id, image_ptr_t> in_image_map_t;

    private:
        // Private typedefs: only used internally 
        typedef std::map<input_id, bool> in_bool_map_t;

        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        Node(Scheduler& sched);
        virtual ~Node(){ }
        
        /* overload for the common case where we're connecting a node with one
         * output to a node with one input
         */
        void setInput(node_ptr_t n);
        void setInput(input_id const& i_id, node_ptr_t n, output_id const& o_id); 

        void clearInput(input_id const& i_id);
        void clearInputs(node_ptr_t parent);
        void clearInputs();

        input_id_set_t inputs() const; 
        in_link_map_t inputLinks() const; 
        std::set<node_ptr_t> parents() const;

        /*  overload for the common case where we're connecting a node with one
         *  output to a node with one input
         */ 
        void setOutput(node_ptr_t n);
        void setOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id); 

        void clearOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id); 
        void clearOutputs(node_ptr_t child); 
        void clearOutputs();
        
        output_id_set_t outputs() const;
        out_link_map_t outputLinks() const;
        std::set<node_ptr_t> children() const;

        int numChildren() const;
        
        void exec();
        
        /* Keep a record of which inputs are new (have changed since they were
         * last used by this node)
         * Check to see if this node should add itself to the scheduler queue
         */
        void newInput(input_id const& a);
    
        /* mark all inputs as new
         */
        void newInput();
        
        /* This is called by the children of this node in order to request new
         * output. It may be called at the start or end of the child's exec()
         */
        void demandNewOutput(/*output_id ?*/) throw();
        
        /* Get the actual image data associated with an output
         */
        image_ptr_t getOutputImage(output_id const& o_id) const throw(id_error);
        
        /* return all parameter values
         */
        param_value_map_t parameters() const;
        
        /* set a parameter based on a message
         */
        void setParam(boost::shared_ptr<SetNodeParameterMessage>  m);

        /* set a single parameter value
         */
        template<typename T>
        void setParam(param_id const& p, T const& v) throw(id_error){
            lock_t l(m_parameters_lock);
            param_value_map_t::iterator i = m_parameters.find(p);
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
        
        /* Derived types overload this for notification of changed parameters:
         * this notification is only useful for asynchronous nodes.
         */
        template<typename T>
        void paramChanged(param_id const&){ }
        
        /* return a single parameter value
         */
        template<typename T>
        T param(param_id const& p) const throw(id_error){
            lock_t l(m_parameters_lock);
            const param_value_map_t::const_iterator i = m_parameters.find(p);
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
        void registerOutputID(output_id const& o);
        void registerInputID(input_id const& i);
        
        /* allow derived types to stop nodes from being queued for execution:
         * use with care, excessive use could stop the pipeline.
         */
        virtual bool allowQueueExec() throw(){
            return true;
        }

        /* Check to see if all inputs are new and output is demanded; if so, 
         * add this node to the scheduler queue
         */
        void checkAddSched() throw();

    private:
        bool _allInputsValid() const throw(); 
        void _demandNewParentInput() throw();
            
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
        param_value_map_t m_parameters;
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

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, Node const& n){
    os << "{Node links in=" << n.inputLinks()
       << " links out=" << n.outputLinks() << "}";
    return os;
}

#endif // ndef __NODE_H__
