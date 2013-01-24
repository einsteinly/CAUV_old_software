/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_NODE_TYPE_H__
#define __CAUV_GUI_NODE_TYPE_H__


#include <typeinfo>
#include <map>
#include <stdexcept>

namespace cauv {
namespace gui {

typedef int node_type;

struct node_types {
    static node_type count;
    static std::map<std::string, node_type> typeMap;
};

template<class T> node_type nodeType(){
    node_types types;
    try {
        return types.typeMap.at(typeid(T).name());
    } catch (std::out_of_range) {
        return types.typeMap[typeid(T).name()] = types.count++;
    }
}

} // namespace gui
} // namespace cauv
#endif // __CAUV_GUI_NODE_TYPE_H__
