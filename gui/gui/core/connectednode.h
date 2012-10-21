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

#ifndef __CAUV_GUI_CONNECTED_NODE_H__
#define __CAUV_GUI_CONNECTED_NODE_H__

#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <liquid/node.h>
#include <liquid/style.h>
#include <liquid/arcSource.h>

#include <model/variants.h>

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

