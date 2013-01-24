/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_FLUIDITY_NODE_H__
#define __CAUV_FLUIDITY_NODE_H__

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <liquid/node.h>
#include <liquid/arcSource.h>
#include <liquid/proxyWidget.h>

#include <model/nodeItemModel.h>
#include <model/node.h>

#include <mainwindow.h>
#include <connectednode.h>


namespace cauv {
class CauvNode;

namespace gui {

class GroupingNode;
class NodeScene;

namespace f {
class FView;
} // namespace f


class FluidityNode : public Node {
    public:
    FluidityNode(const nid_t id);
    virtual ~FluidityNode();
    std::string fullPipelineName();
};

class FluidtySourceDelegate : public liquid::ArcSourceDelegate {
    public:
    FluidtySourceDelegate(boost::shared_ptr<FluidityNode> const& node) :
        liquid::ArcSourceDelegate(QVariant("FluiditySourceDelegate")),
        m_node(node){}
    virtual boost::shared_ptr<FluidityNode> node(){ return m_node; }
    boost::shared_ptr<FluidityNode> m_node;
};

class LiquidFluidityNode: public ConnectedNode
{
    Q_OBJECT
public:
    LiquidFluidityNode(boost::shared_ptr<FluidityNode> node,
                       boost::weak_ptr<CauvMainWindow> in_window,
                       QGraphicsItem *parent = 0);

    virtual liquid::ArcSource * getSourceFor(boost::shared_ptr<Node> const&) const;

    boost::shared_ptr<FluidityNode> fluidityNode();

public Q_SLOTS:
    void beginMaximise();
    void zoomIn(int percent);
    void maximise();

    void zoomOut(int percent);    
    void unMaximise();

protected:
    boost::shared_ptr<FluidityNode> m_node;
    liquid::ProxyWidget* m_contents;
    f::FView* m_view;
    liquid::ArcSource * m_source;
    boost::weak_ptr<CauvMainWindow> m_in_window;
    QRectF m_orginal_view_rect;
    QRectF m_zoomed_view_rect;
    bool m_maximising;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_FLUIDITY_NODE_H__
