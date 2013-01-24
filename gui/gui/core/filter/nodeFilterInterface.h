/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_NODE_FILTER_INTERFACE_H__
#define __CAUV_GUI_NODE_FILTER_INTERFACE_H__

#include <boost/shared_ptr.hpp>

namespace cauv {
namespace gui {

class Node;

struct NodeFilterInterface {
    // return false to filter out an item (and its children) from the list
    virtual bool filter(boost::shared_ptr<Node> const& node) = 0;

    // should be implmented as a signal by subclasses
    virtual void filterChanged() = 0;
};


} // namespace gui
} // namespace cauv
#endif // __CAUV_GUI_NODE_FILTER_INTERFACE_H__
