#ifndef __CAUV_GUI_GROUPDROPHANDLER_H__
#define __CAUV_GUI_GROUPDROPHANDLER_H__

#include <boost/shared_ptr.hpp>

#include <QtGui>

#include <liquid/water/graph.h>

#include <debug/cauv_debug.h>

#include <framework/drag/nodeDragging.h>

namespace cauv {
namespace gui {

class Node;
class NodeItemModel;

class GroupDropHandler : public DropHandlerInterface<QGraphicsItem * > {
public:
    GroupDropHandler(boost::shared_ptr<NodeItemModel> model);

    virtual bool accepts(boost::shared_ptr<Node> const& node);
    virtual QGraphicsItem * handle(boost::shared_ptr<Node> const& node);

protected:
    boost::shared_ptr<NodeItemModel> m_model;
};

} // namespace gui
} // namespace cauv

#endif // GRAPHDROPHANDLER_H
