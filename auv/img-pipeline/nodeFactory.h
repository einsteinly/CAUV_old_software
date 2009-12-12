/**
 * nodeFactory.h
 *
 * Hopefully this file hides away all the messy details of registering and
 * creating nodes of different types:
 *
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

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "pipelineTypes.h"


/* These make registering your new node type a little easier */
#define DECLARE_NFR \
const static NodeFactoryRegister s_nfr

#define DEFINE_NFR(NodeName, nt_ident) \
const NodeFactoryRegister NodeName::s_nfr = NodeFactoryRegister(nt_ident, creator_ptr_t(new Creator<NodeName>()))


/** add new node types here! **/
enum NodeType{
    nt_invalid,
    nt_copy,
    nt_resize,
    nt_file_input,
    nt_file_output,
    nt_local_display,
    nt_max_num_node_types = 0xff
};

/* need to know that nodes exist, but want to #include this file in node.h */
class Node;

struct CreatorBase{
    virtual boost::shared_ptr<Node> create(Scheduler&) const = 0;
};
typedef boost::shared_ptr<CreatorBase> creator_ptr_t;


template<typename T>
struct Creator: public CreatorBase{
    virtual boost::shared_ptr<Node> create(Scheduler& s) const{
        return boost::shared_ptr<T>(new T(s));
    }
};


/* static magic */
class NodeFactoryRegister{
    typedef std::map<NodeType, creator_ptr_t> nt_creator_map_t;
    
    public:
        NodeFactoryRegister(NodeType const& n, creator_ptr_t f){
            boost::lock_guard<boost::recursive_mutex> l(s_register_lock);
            s_register[n] = f;
        }
        
        static boost::shared_ptr<Node> create(NodeType const& n, Scheduler& s){
            boost::lock_guard<boost::recursive_mutex>  l(s_register_lock);
            nt_creator_map_t::const_iterator i = s_register.find(n);
            if(i != s_register.end()){
                return i->second->create(s);
            }else{
                throw node_type_error("create: Invalid node type");
            }
        }
        
    private:
        
        static boost::recursive_mutex s_register_lock;
        static nt_creator_map_t s_register;
};


#endif // ndef __NODE_FACTORY_H__
