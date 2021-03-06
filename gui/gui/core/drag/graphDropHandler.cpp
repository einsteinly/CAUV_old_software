/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "graphDropHandler.h"

#include <liquid/node.h>
#include <liquid/nodeHeader.h>
#include <liquid/water/dataSeries.h>
#include <liquid/connectionSink.h>
#include <liquid/arcSink.h>

#include "model/node.h"
#include "model/nodes/numericnode.h"

#include "connectednode.h"

#include "elements/style.h"
#include "nodepicker.h"

using namespace cauv;
using namespace cauv::gui;

GraphLayoutItem::GraphLayoutItem(liquid::water::GraphConfig const& config)
    : QGraphicsLayoutItem(),
      liquid::water::Graph(config){
    setMinimumSize(16, 16);
    setMaximumSize(1200, 800);
    setPreferredSize(1200, 800);
}

void GraphLayoutItem::setGeometry(const QRectF& rect){
    setPos(rect.topLeft());
    QGraphicsLayoutItem::setGeometry(QRectF(QPointF(0,0), rect.size()));
    setRect(geometry());
}

void GraphLayoutItem::updateGeometry(){
    liquid::water::Graph::setRect(geometry());
    QGraphicsLayoutItem::updateGeometry();
}

QSizeF GraphLayoutItem::sizeHint(Qt::SizeHint which, const QSizeF &constraint) const{
    switch(which){
    case Qt::PreferredSize: return QSizeF(300, 200);
    case Qt::MinimumSize:   return QSizeF(60, 40);
    default:                return constraint;
    }
}

class LiquidGraphNode : public ConnectedNode,
                        public liquid::RejectingConnectionSink {
public:
    LiquidGraphNode(boost::shared_ptr<NumericNodeBase> node, QGraphicsItem *parent = nullptr) :
        ConnectedNode(node, Graph_Node_Style(), parent),
        m_arc_sink(new liquid::ArcSink(Param_Arc_Style(), Required_Param_Input(), this)){

        addItem(m_arc_sink);
    }

    liquid::ArcSource * getSourceFor(boost::shared_ptr<Node> const&) const{
        return nullptr; //no sources in this node
    }

    liquid::ArcSink * m_arc_sink;
};


GraphingDropHandler::GraphingDropHandler(boost::shared_ptr<NodeItemModel> model) :
    m_model(model){
}

bool GraphingDropHandler::accepts(boost::shared_ptr<Node> const& node){
    CAUV_LOG_DEBUG(0, "GraphingDropHandler drop enter from" << node->nodePath());
    return node->type == nodeType<NumericNodeBase>();
}

QGraphicsItem* GraphingDropHandler::handle(boost::shared_ptr<Node> const& node) {
    CAUV_LOG_DEBUG(0, "GraphingDropHandler drop from" << node->nodePath());

    LiquidGraphNode * ln = new LiquidGraphNode(node->to<NumericNodeBase>());
    ln->setResizable(true);
    ln->setTitle(QString::fromStdString(node->nodeName()));
    ln->setInfo(QString::fromStdString(node->nodePath()));
    auto  graph = new GraphLayoutItem(liquid::water::One_Minute);
    // !!! FIXME need some sort of traits to know what is an angle
    boost::shared_ptr<liquid::water::DataSeries> series(
                new liquid::water::DataSeries(liquid::water::Unlimited_Graph, QString::fromStdString(node->nodeName()))
                );
    graph->addDataSeries(series);
    ln->addItem(graph);

    QObject::connect(node.get(), SIGNAL(onUpdate(QVariant)), series.get(), SLOT(postData(QVariant)));

    return ln;
}
