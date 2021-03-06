/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "plugin.h"

#include <QGraphicsProxyWidget>

#include <liquid/node.h>
#include <debug/cauv_debug.h>

#include <gui/core/framework/elements/style.h>
#include <gui/core/framework/nodescene.h>

#include "graph.h"

using namespace cauv;
using namespace cauv::gui;

/*
bool QwtGraphDropHandler::accepts(boost::shared_ptr<Node> const& node){
    return node->type == nodeType<NumericNodeBase>();
}

QGraphicsItem * QwtGraphDropHandler::handle(boost::shared_ptr<Node> const& node) {

    GraphWidget * graph = new GraphWidget(boost::static_pointer_cast<NumericNodeBase>(node));

    liquid::LiquidNode * ln = new liquid::LiquidNode(AI_Node_Style());
    ln->setResizable(true);

    QGraphicsProxyWidget * proxy = new QGraphicsProxyWidget();
    proxy->setWidget(graph);
    proxy->setFlag(QGraphicsItem::ItemIsMovable);
    proxy->setFlag(QGraphicsItem::ItemIsSelectable);

    ln->addItem(proxy);
    return ln;
}
*/

const QString QwtGraphsPlugin::name() const{
    return QString("QwtGraphs");
}

void QwtGraphsPlugin::initialise(){
    //m_handler = boost::make_shared<QwtGraphDropHandler>();
    //m_actions->scene->registerDropHandler(m_handler);
}

void QwtGraphsPlugin::shutdown(){
    //m_actions->scene->removeDropHandler(m_handler);
}

Q_EXPORT_PLUGIN2(cauv_qwtgraphsplugin, QwtGraphsPlugin)
