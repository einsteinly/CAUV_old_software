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

#include "plugin.h"

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <QTreeView>

#include <common/cauv_node.h>

#include <debug/cauv_debug.h>

#include <gui/core/framework/nodescene.h>
#include <gui/core/framework/nodepicker.h>
#include <gui/core/model/registry.h>

#include <generated/types/GuiaiGroup.h>

#include "messageobserver.h"
#include "aiNode.h"

using namespace cauv;
using namespace cauv::gui;

const QString AiPlugin::name() const{
    return QString("AI");
}

void AiPlugin::initialise(){
    boost::shared_ptr<CauvNode> node = m_actions->node.lock();
    if(node) {
        node->joinGroup("guiai");
        node->addMessageObserver(boost::make_shared<AiMessageObserver>(m_auv));

        //!!! todo: this should be sent more often than just at startup
        //!!! maybe on membership change?

        node->send(boost::make_shared<RequestAIStateMessage>());
    }
    else error() << "AiPlugin failed to lock cauv node";

    m_actions->scene->registerDropHandler(boost::make_shared<AiDropHandler>());



    /*
        AiNode *node = new AiNode();
        node->addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, new liquid::RejectingConnectionSink()));
        node->addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, new liquid::RejectingConnectionSink()));
        node->addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, new liquid::RejectingConnectionSink()));
        QGraphicsProxyWidget * proxy = new QGraphicsProxyWidget();
        proxy->setWidget(new NodePicker(m_actions->auv));
        node->addItem(proxy);
        node->setResizable(true);
        m_actions->scene->addItem(node);
    */

        AiNode *node1 = new AiNode();
        QGraphicsProxyWidget * proxy = new QGraphicsProxyWidget();
        QTreeView * view = new NodeTreeView();
        QAbstractItemModel * model = new NodeItemModel(VehicleRegistry::instance());
        view->setModel(model);
        view->setDragEnabled(true);
        proxy->setWidget(view);
        node1->addItem(proxy);
        node1->setResizable(true);
        m_actions->scene->addItem(node1);

        AiNode *node2 = new AiNode();
        QGraphicsProxyWidget * proxy2 = new QGraphicsProxyWidget();
        QTreeView * view2 = new NodeTreeView();
        view2->setModel(model);
        view2->setDragEnabled(true);
        proxy2->setWidget(view2);
        node2->addItem(proxy2);
        node2->setResizable(true);
        m_actions->scene->addItem(node2);


        AiNode *node3 = new AiNode();
        QGraphicsProxyWidget * proxy3 = new QGraphicsProxyWidget();
        QTreeView * view3 = new NodeTreeView();
        QAbstractItemModel * model3 = new NodeItemModel(VehicleRegistry::instance()->find<Node>("redherring"));
        view3->setModel(model3);
        view3->setDragEnabled(true);
        proxy3->setWidget(view3);
        node3->addItem(proxy3);
        node3->setResizable(true);
        m_actions->scene->addItem(node3);

        node->send(boost::make_shared<RemoveTaskMessage>(1));

}

Q_EXPORT_PLUGIN2(cauv_aiplugin, AiPlugin)
