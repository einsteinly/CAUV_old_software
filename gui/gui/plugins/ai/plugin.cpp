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
#include "messagegenerators.h"
#include "ainode.h"

#include <stdexcept>

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
        NodeTreeView * view = new NodeTreeView();
        view->setModel(m_actions->root.get());
        view->setDragEnabled(true);
        view->addNumericDelegateToColumn(1);
        proxy->setWidget(view);
        node1->addItem(proxy);
        node1->setResizable(true);
        m_actions->scene->addItem(node1);


        AiNode *node3 = new AiNode();
        QGraphicsProxyWidget * proxy3 = new QGraphicsProxyWidget();
        NodeTreeView * view3 = new NodeTreeView();
        view3->setModel(m_actions->root.get());
        view3->setDragEnabled(true);
        view3->setRootIndex(m_actions->root->indexFromNode(VehicleRegistry::instance()->find<Node>("redherring")));
        view3->addNumericDelegateToColumn(1);
        proxy3->setWidget(view3);
        node3->addItem(proxy3);
        node3->setResizable(true);
        m_actions->scene->addItem(node3);


        boost::shared_ptr<GroupingNode> ai = m_auv->findOrCreate<GroupingNode>("ai");
        boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
        connect(tasks.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupTask(boost::shared_ptr<Node>)));
        boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
        connect(conditions.get(), SIGNAL(nodeAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupCondition(boost::shared_ptr<Node>)));
}

void AiPlugin::setupTask(boost::shared_ptr<Node> node){
    try {
        m_auv->attachGenerator(
                    boost::make_shared<MessageHandler<AiTaskNode,
                    SetTaskStateMessage> >(node->to<AiTaskNode>())
                    );
    } catch(std::runtime_error e) {
        error() << "AiPlugin::setupTask: Expecting AiTaskNode" << e.what();

    }
}

void AiPlugin::setupCondition(boost::shared_ptr<Node> node){
    try {
        m_auv->attachGenerator(
                    boost::make_shared<MessageHandler<AiConditionNode,
                    SetConditionStateMessage> >(node->to<AiConditionNode>())
                    );
    } catch(std::runtime_error e) {
        error() << "AiPlugin::setupCondition: Expecting AiTaskNode" << e.what();

    }
}

Q_EXPORT_PLUGIN2(cauv_aiplugin, AiPlugin)
