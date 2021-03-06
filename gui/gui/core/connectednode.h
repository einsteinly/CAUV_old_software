/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_CONNECTED_NODE_H__
#define __CAUV_GUI_CONNECTED_NODE_H__

#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <liquid/node.h>
#include <liquid/style.h>
#include <liquid/arcSource.h>

#include <connectednodemap.h>

namespace cauv{
namespace gui{

class Node;

class ConnectedNode: public liquid::LiquidNode {
protected:
    ConnectedNode(boost::shared_ptr<Node> const& node,
                  liquid::NodeStyle const& style,
                  QGraphicsItem *parent=0);
   virtual ~ConnectedNode();

public:
    static ConnectedNode * nodeFor(boost::shared_ptr<Node> const& node);

    virtual liquid::ArcSource * getSourceFor(boost::shared_ptr<Node> const&) const = 0;

    static void setMap(ConnectedNodeMap * map);
    static ConnectedNodeMap * getMap();

private:
    void unregister(boost::shared_ptr<Node> const& node);
    void unregister(ConnectedNode* ln);

    typedef boost::unique_lock<boost::mutex> lock_t;
    static ConnectedNodeMap * m_mapping;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_CONNECTED_NODE_H__

