/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

