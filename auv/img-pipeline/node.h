#ifndef __NODE_H__
#define __NODE_H__

#include <map>
#include <list>
#include <stdexcept>

//#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/variant.hpp>
#include <boost/foreach.hpp>

#include "image.h"
#include "pipeline_types.h"

class Node{
        // Private typedefs
        typedef boost::shared_ptr<Node> node_ptr_t;
        typedef std::pair<node_ptr_t, input_id> output_link_t;
        typedef std::list<output_link_t> output_link_list_t;
        
        //typedef boost::tuple<node_id, input_id> output_link_t;
        typedef std::map<output_id, output_link_list_t> out_link_map_t;

        typedef std::pair<node_ptr_t, output_id> input_link_t;
        
        //typedef boost::tuple<node_id, output_id> input_link_t;
        typedef std::map<input_id, input_link_t> in_link_map_t;
        
        typedef std::map<input_id, bool> in_bool_map_t;

    protected:
        // Protected typedefs
        typedef boost::shared_ptr<Image> image_ptr_t;
        typedef std::map<output_id, image_ptr_t> out_image_map_t;
        typedef std::map<input_id, image_ptr_t> in_image_map_t;

    public:
        Node(Scheduler& sched)
            : m_priority(low), m_speed(slow), /*m_lock(),*/ m_sched(sched){
        }
        
        void setInput(input_id const& i_id, node_ptr_t n, output_id const& o_id){
            boost::lock_guard<boost::recursive_mutex> l(m_parent_links_lock);
            const in_link_map_t::iterator i = m_parent_links.find(i_id);
            if(i == m_parent_links.end()){
                throw(id_error(std::string("setInput: Invalid input id") + std::string(i_id)));
            }else{
                i->second = input_link_t(n, o_id);
            }
        }

        /* overload for the common case where we're connecting a node with one
         * output to a node with one input
         */
        void setInput(node_ptr_t n){
            boost::lock_guard<boost::recursive_mutex> l(m_parent_links_lock);
            
            if(m_parent_links.size() == 1){
                const std::vector<output_id> parent_outputs = n->getOutputIDs();
                if(parent_outputs.size() != 1){
                    std::cerr << "Parent has " << parent_outputs.size() << " outputs!" << std::endl;
                    throw link_error("setInput: specific parent output must be specified");
                }
                m_parent_links.begin()->second = input_link_t(n, parent_outputs[0]);
            }else if(m_parent_links.size() > 1){
                throw link_error("setInput: specific input must be specified");
            }else{
                throw link_error("setInput: This node accepts no inputs");
            }
        }

        void clearInput(input_id const& i_id){ 
            boost::lock_guard<boost::recursive_mutex> l(m_parent_links_lock);
            const in_link_map_t::iterator i = m_parent_links.find(i_id);
            if(i == m_parent_links.end()){
                throw(id_error(std::string("clearInput: Invalid input id") + std::string(i_id)));
            }else{
                i->second = input_link_t();
            }
        }
  
        void setOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id){
            boost::lock_guard<boost::recursive_mutex> l(m_child_links_lock);
            const out_link_map_t::iterator i = m_child_links.find(o_id);
            if(i == m_child_links.end()){
                throw(id_error(std::string("setOutput: Invalid output id") + std::string(o_id)));
            }else{
                // An output can be connected to more than one input, so
                // m_child_links[output_id] is a list of output_link_t
                i->second.push_back(output_link_t(n, i_id));
            }
        }

        /*  overload for the common case where we're connecting a node with out
         *  output to a node with one input
         */ 
        void setOutput(node_ptr_t n){
            boost::lock_guard<boost::recursive_mutex> l(m_child_links_lock);

            if(m_child_links.size() == 1){
                const std::vector<input_id> child_inputs = n->getInputIDs();
                if(child_inputs.size() != 1){
                    throw(link_error("setOutput: specific child input must be specified"));
                }
                // An output can be connected to more than one input, so
                // m_child_links[output_id] is a list of output_link_t
                m_child_links.begin()->second.push_back(output_link_t(n, child_inputs[0]));
            }else if(m_child_links.size() > 1){
                throw link_error("setOutput: specific output must be specified");
            }else{
                throw link_error("setOutput: this node has no outputs");
            }
        }

        void clearOutput(output_id const& o_id, node_ptr_t n, input_id const& i_id){
            boost::lock_guard<boost::recursive_mutex> l(m_child_links_lock);
            const out_link_map_t::iterator i = m_child_links.find(o_id);
            if(i == m_child_links.end()){
                throw(id_error(std::string("clearOutput: Invalid output id") + std::string(o_id)));
            }else{
                // An output can be connected to more than one input, so
                // m_child_links[output_id] is a list of output_link_t
                output_link_list_t::iterator j = std::find(i->second.begin(), i->second.end(), output_link_t(n, i_id));
                if(j == i->second.end()){
                    throw(id_error("clearOutput: Invalid node & input id: (node ID lookup is TODO): " + std::string(i_id)));
                }else{
                    i->second.erase(j);
                }
            }
        }

        void exec(){
            // take copies of image_ptr s from parents before _demandNewParentInput()
            in_image_map_t inputs;
            out_image_map_t outputs;

            m_parent_links_lock.lock();
            for(in_link_map_t::const_iterator i = m_parent_links.begin(); i != m_parent_links.end(); i++)
                inputs[i->first] = i->second.first->getOutputImage(i->second.second);
            m_parent_links_lock.unlock();
            
            std::cerr << "exec: speed " << m_speed << ", " << inputs.size() << " inputs" << std::endl;
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
            
            
            // Record that we've used all of our inputs
            m_new_inputs_lock.lock();
            
            BOOST_FOREACH(in_bool_map_t::value_type& v, m_new_inputs)
                v.second = false;
            m_new_inputs_lock.unlock();
            
            
            // for each of this node's outputs
            BOOST_FOREACH(out_link_map_t::value_type& v, m_child_links){
                // v is a std::pair<output_id, std::list<...> >
                
                // for each node connected to the output
                BOOST_FOREACH(output_link_t& link, v.second){
                    // link is a std::pair<node_ptr, input_id>
                    
                    // notify the node that it has new input
                    link.first->newInput(link.second);
                }
            }
        }
        
        /* Keep a record of which inputs are new (have changed since they were
         * last used by this node)
         * Check to see if this node should add itself to the scheduler queue
         */
        void newInput(input_id const& a){
            boost::lock_guard<boost::recursive_mutex> l(m_new_inputs_lock);
            const std::map<input_id, bool>::iterator i = m_new_inputs.find(a);
            
            if(i == m_new_inputs.end()){
                std::cerr << "valid inputs:" << std::endl;
                BOOST_FOREACH(in_bool_map_t::value_type const& v, m_new_inputs)
                    std::cerr << v.second << std::endl;

                throw(id_error(std::string("newInput: Invalid input id: ") + std::string(a)));
            }else{
                i->second = true;
            }
        }
        
        /* This is called by the children of this node in order to request new
         * output. It may be called at the start or end of the child's exec()
         */
        void demandNewOutput(/*output_id ?*/){
            boost::lock_guard<boost::recursive_mutex> l(m_output_demanded_lock);
            m_output_demanded = true;
            
            _checkAddSched();
        }
        
        /* Get the actual image data associated with an output
         */
        image_ptr_t getOutputImage(output_id const& o_id){
            boost::lock_guard<boost::recursive_mutex> l(m_outputs_lock);
            const out_image_map_t::const_iterator i = m_outputs.find(o_id);
            if(i == m_outputs.end() || !i->second){
                // TODO: double check that this releases the lock_guard
                throw(id_error(std::string("getOutputImage: Invalid output id: ") + std::string(o_id)));
            }else{
                return i->second;
            }
        }
        
        /* Return a list of valid outputs from this node
         */
        std::vector<output_id> getOutputIDs(){
            boost::lock_guard<boost::recursive_mutex> l(m_outputs_lock);
            std::vector<output_id> r;
            out_image_map_t::const_iterator i;
            for(i = m_outputs.begin(); i != m_outputs.end(); i++)
                r.push_back(i->first);
            return r;
        }
        
        /* Return a list of valid inputs to this node
         */
        std::vector<input_id> getInputIDs(){
            boost::lock_guard<boost::recursive_mutex> l(m_new_inputs_lock);
            std::vector<input_id> r;
            in_bool_map_t::const_iterator i;
            for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++)
                r.push_back(i->first);
            return r;
        }
        
        /* return all parameter values
         */
        std::map<param_id, param_value_t> getParamValues();
        
        /* set a single parameter value
         */
        template<typename T>
        void setParam(param_id const& p, T const& v){
            boost::lock_guard<boost::recursive_mutex> l(m_parameters_lock);
            const std::map<param_id, param_value_t>::iterator i = m_parameters.find(p);
            if(i != m_parameters.end()){
                i->second = v;
            }else{
                throw(id_error(std::string("setParam: Invalid parameter id: ") + std::string(p)));
            }
        }
        
        /* return a single parameter value
         */
        template<typename T>
        T param(param_id const& p){
            boost::lock_guard<boost::recursive_mutex> l(m_parameters_lock);
            const std::map<param_id, param_value_t>::const_iterator i = m_parameters.find(p);
            if(i != m_parameters.end()){
                return boost::get<T>(i->second);
            }else{
                throw(id_error(std::string("param: Invalid parameter id: ") + std::string(p)));
            }
        }
        
    protected:
        /* Derived classes override this to do whatever image processing it is
         * that they do.
         */
        virtual out_image_map_t doWork(in_image_map_t&) = 0;
                
        /* Priority of this node; this might change dynamically.
         * Used when this node is added to a scheduler queue
         */
        Priority m_priority;
    
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
            boost::lock_guard<boost::recursive_mutex> l(m_parameters_lock);
            m_parameters[p] = param_value_t(default_value);
        }
        void registerOutputID(output_id const& o){
            m_child_links[o] = output_link_list_t();
            m_outputs[o] = image_ptr_t();
        }
        void registerInputID(input_id const& i){
            m_new_inputs[i] = false;
            m_parent_links[i] = input_link_t();
        }
        
    private:
        /* Check to see if all inputs are new and output is demanded; if so, 
         * add this node to the scheduler queue
         */
        void _checkAddSched(){
            std::map<input_id, bool>::const_iterator i;
            boost::lock_guard<boost::recursive_mutex> li(m_output_demanded_lock);
            boost::lock_guard<boost::recursive_mutex> lo(m_new_inputs_lock);
            
            if(!m_output_demanded)
                return;
            
            for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++)
                if(!i->second)
                    return;
            
            m_sched.addToQueue(this, m_priority);
        }

        void _demandNewParentInput(){
            boost::lock_guard<boost::recursive_mutex> l(m_parent_links_lock);
            in_link_map_t::const_iterator i;
            for(i = m_parent_links.begin(); i != m_parent_links.end(); i++)
                i->second.first->demandNewOutput();

            BOOST_FOREACH(in_link_map_t::value_type const& v, m_parent_links)
                v.second.first->demandNewOutput(); 
        }
        
        
        /* maps an input_id to an output of another node */
        in_link_map_t   m_parent_links;
        boost::recursive_mutex m_parent_links_lock;
        
        /* maps an output_id to a list of inputs on other nodes */
        out_link_map_t  m_child_links;
        boost::recursive_mutex m_child_links_lock;
        
        /* maps an output_id to an image */
        out_image_map_t m_outputs;
        boost::recursive_mutex m_outputs_lock;
        
        /* parameters of the filters */
        std::map<param_id, param_value_t> m_parameters;
        boost::recursive_mutex m_parameters_lock;
        
        /* Keep track of which of our inputs have been refreshed since this node
         * was last exec()d
         * Set by newInput(), checked by checkAddSched(), cleared by exec()
         */
        in_bool_map_t m_new_inputs;
        boost::recursive_mutex m_new_inputs_lock;
        
        /* Has output been demanded of this node?
         * Set by demandNewOutput(), checked by checkAddSched(), cleared by exec()
         */
        bool m_output_demanded;
        boost::recursive_mutex m_output_demanded_lock;
        
        //TODO: think about locking for adding/removing nodes
        /* When a child is deleted, it needs to lock it's parent, (and it's
         * children, but we aren't going to allow nodes that have children to be
         * deleted), since it's parent may try to call newInput()
         */
        //boost::recursive_mutex m_lock;
        
        /* The scheduler associated with this node:
         * This is used by newInput() and demandNewOutput(), each of which may
         * decide that this node now needs to be executed (this->exec()), so
         * they add this node to a scheduler queue using:
         *		_sched->addToQueue(this, _priority);
         */
        Scheduler& m_sched;
};


#endif // ndef __NODE_H__
