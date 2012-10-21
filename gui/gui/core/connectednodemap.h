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

#ifndef __CAUV_GUI_CONNECTED_NODE_MAP_H__
#define __CAUV_GUI_CONNECTED_NODE_MAP_H__

#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace cauv{
namespace gui{

class ConnectedNode;
class Node;

struct ConnectedNodeMap : public std::map<boost::shared_ptr<Node>, ConnectedNode*> {
    boost::mutex lock;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_CONNECTED_NODE_MAP_H__

