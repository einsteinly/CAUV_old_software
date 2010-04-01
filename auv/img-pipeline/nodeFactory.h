/**
 * nodeFactory.h
 *
 * Hopefully this file hides away all the messy details of registering and
 * creating nodes of different types:
 *
 * the NodeType enum is defined in messages.msg, add new types there!
 *
 * How to register your shiny new node type
 *
 * class SomeNode{
 *
 *      ...
 *     
 *     // Declare a static NodeFactoryRegister member;
 *     DECLARE_NFR;
 * };
 *
 * // Define the NodeFactoryRegisterMember using your NodeClassName and
 * // nt_node_identifier
 * DEFINE_NFR(SomeNode, nt_some);
 *
 **/
 
#ifndef __NODE_FACTORY_H__
#define __NODE_FACTORY_H__

#include <map>
#include <list>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <common/messages.h>

#include "pipelineTypes.h"


/* These make registering your new node type a little easier */
#define DECLARE_NFR \
const static NodeFactoryRegister s_nfr

#define DEFINE_NFR(NodeName, nt_ident) \
const NodeFactoryRegister NodeName::s_nfr = NodeFactoryRegister(nt_ident, creator_ptr_t(new Creator<NodeName>()))

/* need to know that nodes exist, but want to #include this file in node.h */
class Node;

struct CreatorBase{
    virtual boost::shared_ptr<Node> create(Scheduler&, ImageProcessor&, NodeType::e) const = 0;
};
typedef boost::shared_ptr<CreatorBase> creator_ptr_t;

template<typename T>
struct Creator: public CreatorBase{
    virtual boost::shared_ptr<Node> create(Scheduler& s, ImageProcessor& pl, NodeType::e t) const{
        return boost::make_shared<T>(boost::ref(s), boost::ref(pl), t);
    }
};

/* static magic */
class NodeFactoryRegister{
        typedef std::map<NodeType::e, creator_ptr_t> nt_creator_map_t;
    public:
        typedef std::list<NodeType::e> nt_list_t;
        typedef boost::shared_ptr<Node> node_ptr_t;
        typedef boost::recursive_mutex mutex_t;
        typedef boost::lock_guard<mutex_t> lock_t;

        NodeFactoryRegister(NodeType::e const& n, creator_ptr_t f){
            lock_t l(registerLock());
            nodeRegister()[n] = f;
        }
        
        static node_ptr_t create(NodeType::e const& n, Scheduler& s, ImageProcessor& pl){
            lock_t l(registerLock());
            nt_creator_map_t::const_iterator i = nodeRegister().find(n);
            if(i != nodeRegister().end()){
                return i->second->create(s, pl, n);
            }else{
                throw node_type_error("create: Invalid node type");
            }
        }

        static nt_list_t list(){
            nt_list_t r;
            lock_t l(registerLock());
            nt_creator_map_t::const_iterator i;
            for(i = nodeRegister().begin(); i != nodeRegister().end(); i++)
                r.push_back(i->first);
            return r;
        }
        
    private:
        /* avoid the static initialisation fiasco: construct on first use
         */
        static mutex_t& registerLock(){
            static mutex_t s_register_lock;
            return s_register_lock;
        }
        static nt_creator_map_t& nodeRegister(){
            static nt_creator_map_t s_register;
            return s_register;
        }
};


#endif // ndef __NODE_FACTORY_H__
