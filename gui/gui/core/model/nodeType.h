/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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
