/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#ifndef __CAUV_FLUIDITY_NODE_H__
#define __CAUV_FLUIDITY_NODE_H__

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <liquid/node.h>
#include <liquid/arcSource.h>
#include <liquid/proxyWidget.h>

#include <gui/core/model/model.h>
#include <gui/core/model/node.h>
#include <gui/core/framework/mainwindow.h>
#include <gui/core/framework/connectednode.h>


namespace cauv {
class CauvNode;

namespace gui {

namespace f {
class FView;
} // namespace f

GENERATE_SIMPLE_NODE(FluidityNode)

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
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_FLUIDITY_NODE_H__