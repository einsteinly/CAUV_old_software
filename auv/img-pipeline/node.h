#ifndef __NODE_H__
#define __NODE_H__

#include <map>
#include <stdexcept>

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/variant.hpp>

#include "cv.h"

#include "pipeline_types.h"


class Node{
    // Private typedefs
    typedef std::map<output_id, boost::shared_ptr<image> > output_map_t;

    typedef boost::tuple<node_id, input_id> output_link_t;
    typedef std::map<output_id, output_link_t> out_link_map_t;

    typedef boost::tuple<node_id, output_id> input_link_t;
    typedef std::map<input_id, input_link_t> in_link_map_t;

    public:
        Node(Scheduler& sched)
            : m_lock(), m_sched(sched){
        }
        
        void setInput(input_id const& i_id, node_id const& n_id, output_id const& o_id){
            const in_link_map_t::iterator i = m_parent_links.find(i_id);
            if(i == m_parent_links.end()){
                throw(parameter_error("Invalid input id"));
            }else{
                i->second = input_link_t(n_id, o_id);
            }
        }
        
        void setOutput(output_id const& o_id, node_id const& n_id, input_id const& i_id){
            const out_link_map_t::iterator i = m_child_links.find(o_id);
            if(i == m_child_links.end()){
                throw(parameter_error("Invalid output id"));
            }else{
                i->second = output_link_t(n_id, i_id);
            }
        }

        void exec(){
            // TODO: should we lock here? can't getLock(), since that could cause huge delays on newInput / demandNewOutput calls
            //
            // if this is a fast node: request new image from parents now (demandNewOutput)
            // 
            //  this->doWork();
            //
            // if this is a slow node, request a new image from parents now
            //
            // for each child that takes our output, set a flag saying 'this output is new'
            // 	ie
            // 	for node in children
            //		node->getLock()
            // 		node->setNew(node->input[our_output]) // or similar
            //		node->releaseLock()
            // 
            // 
            // Record that we've used all of our inputs
            // for v in this->new_inputs
            //	v.second = false
            //
            // TODO: unlock? see top of function
        }
        
        /* Keep a record of which inputs are new (have changed since they were
         * last used by this node)
         * Check to see if this node should add itself to the scheduler queue
         */
        void newInput(input_id const& a){
            getLock();
            const std::map<input_id, bool>::iterator i = m_new_inputs.find(a);
            
            if(i == m_new_inputs.end()){
                throw(parameter_error("Invalid input id"));
            }else{
                i->second = true;
            }
            
            // keeps a record of which inputs are new
            releaseLock();
        }
        
        /* This is called by the children of this node in order to request new
         * output. It may be called at the start or end of the child's exec()
         */
        void demandNewOutput(/*output_id ?*/){
            getLock();
            
            m_output_demanded = true;
            checkAddSched();
            
            releaseLock();
        }
        
        /* Get the actual image data associated with an output
         */
        boost::shared_ptr<image> getOutput(output_id const& o_id) const{
            const output_map_t::const_iterator i = m_outputs.find(o_id);
            if(i == m_outputs.end()){
                throw(parameter_error("Invalid output id"));
            }else{
                return i->second;
            }
        }
        
        /* return all parameter values
         */
        //std::map<parameter_id, param_value> getParamValues() const;
        
        /* set a single parameter value
         */
        void setParamValue(param_id const& p, param_value_t const& v);
        
        /* return a single parameter value
         */
        param_value_t param(param_id const& p) const;
        
    protected:
        /* Derived classes override this to do whatever image processing it is
         * that they do.
         */
        virtual void doWork() = 0;
                
        /* Priority of this node; this might change dynamically.
         * Used when this node is added to a scheduler queue
         */
        Priority m_priority;
        
        /* Derived node types should call these functions (probably from their
         * constructors) in order to register valid input output and parameter
         * ids.
         */
        void registerParamID(param_id const& p){
            m_parameters[p] = param_value_t();
        }
        void registerOutputID(output_id const& o){
            m_child_links[o] = output_link_t();
        }
        void registerInputID(input_id const& i){
            m_parent_links[i] = input_link_t();
        }
        
    private:
        void getLock(){
            m_lock.lock();
        }
        void releaseLock(){
            m_lock.unlock();
        }
        
        /* Check to see if all inputs are new and output is demanded; if so, 
         * add this node to the scheduler queue
         */
        void checkAddSched(){
            std::map<input_id, bool>::const_iterator i;
            
            if(!m_output_demanded)
                return;
            
            for(i = m_new_inputs.begin(); i != m_new_inputs.end(); i++)
                if(!i->second)
                    return;
            
                m_sched.addToQueue(this, m_priority);
            }
        
        
        in_link_map_t	m_parent_links;
        out_link_map_t	m_child_links;
        output_map_t	m_outputs;
        
        
        /* parameters of the filters
         */
        std::map<param_id, param_value_t> m_parameters;
        
        /* Keep track of which of our inputs have been refreshed since this node
         * was last exec()d
         * Set by newInput(), checked by checkAddSched(), cleared by exec()
         */
        std::map<input_id, bool> m_new_inputs;
        
        /* Has output been demanded of this node?
         * Set by demandNewOutput(), checked by checkAddSched(), cleared by exec()
         */
        bool m_output_demanded;
        
        /* When a child is deleted, it needs to lock it's parent, (and it's
         * children, but we aren't going to allow nodes that have children to be
         * deleted), since it's parent may try to call newInput()
         */
        boost::mutex m_lock;
        
        /* The scheduler associated with this node:
         * This is used by newInput() and demandNewOutput(), each of which may
         * decide that this node now needs to be executed (this->exec()), so
         * they add this node to a scheduler queue using:
         *		_sched->addToQueue(this, _priority);
         */
        Scheduler& m_sched;
};


#endif // ndef __NODE_H__
