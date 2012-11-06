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