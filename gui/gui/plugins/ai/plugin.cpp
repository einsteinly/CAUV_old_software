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

#include <QtGui>
#include <liquid/view.h>

#include <common/cauv_node.h>

#include <debug/cauv_debug.h>

#include <gui/core/framework/nodescene.h>
#include <gui/core/framework/nodepicker.h>
#include <gui/core/framework/delegates.h>
#include <gui/core/model/registry.h>

#include <generated/types/GuiaiGroup.h>

#include "aimessaging.h"
#include "ainode.h"

#include <gui/core/framework/nodepicker.h>
#include <stdexcept>

using namespace cauv;
using namespace cauv::gui;


AiPlugin::AiPlugin() : m_filter(boost::make_shared<NodeChildrenExclusionFilter>()){
}

const QString AiPlugin::name() const{
    return QString("AI");
}

void AiPlugin::initialise(){

    foreach(boost::shared_ptr<Vehicle> vehicle, VehicleRegistry::instance()->getVehicles()){
        debug() << "setup AI plugin for" << vehicle;
        setupVehicle(vehicle);
    }
    connect(VehicleRegistry::instance().get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
           this, SLOT(setupVehicle(boost::shared_ptr<Node>)));

    boost::shared_ptr<CauvNode> node = m_actions->node.lock();
    if(node) {
        node->joinGroup("guiai");
        // !!!! FIXME: is there a race condition here between joining the
        // group, and sending the state request message? (if it gets received
        // before we've subscribed to the responses, then the responses won't
        // be sent)

        //!!! todo: this should be sent more often than just at startup
        //!!! maybe on membership change?
        node->send(boost::make_shared<RequestAIStateMessage>());
    } else error() << "AiPlugin failed to lock cauv node";

    ReloadAiFilter * reload = new ReloadAiFilter();
    m_actions->view->installEventFilter(reload);
    connect(reload, SIGNAL(reloadAiRequest()), this, SLOT(reloadAi()));

    m_actions->scene->registerDropHandler(boost::make_shared<AiDropHandler>(m_actions->root, m_actions->node));

    m_actions->nodes->registerDelegate(nodeType<AiTaskNode>(), boost::make_shared<NumericDelegate>());

    m_actions->nodes->registerListFilter(m_filter);

        /*


        AiNode *node = new AiNode();
        node->addItem(new liquid::ArcSink(Image_Arc_Style(), Required_Image_Input(), new liquid::RejectingConnectionSink()));
        node->addItem(new liquid::ArcSink(Image_Arc_Style(), Required_Image_Input(), new liquid::RejectingConnectionSink()));
        node->addItem(new liquid::ArcSink(Image_Arc_Style(), Required_Image_Input(), new liquid::RejectingConnectionSink()));
        QGraphicsProxyWidget * proxy = new QGraphicsProxyWidget();
        proxy->setWidget(new NodePicker(m_actions->auv));
        node->addItem(proxy);
        node->setResizable(true);
        m_actions->scene->addItem(node);


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


*/

}

void AiPlugin::setupTask(boost::shared_ptr<Node> node){
    try {
        node->getClosestParentOfType<Vehicle>()->attachGenerator(
                    boost::make_shared<MessageGenerator<AiTaskNode,
                    SetTaskStateMessage> >(node->to<AiTaskNode>())
                    );
        //m_filter->addNode(node);
        m_actions->scene->addItem(new LiquidTaskNode(node->to<AiTaskNode>()));
        //m_actions->scene->onNodeDroppedAt(node, m_actions->view->mapToScene(
        //                                        m_actions->view->mapFromGlobal(
        //                                          QCursor::pos()
        //                                          )));
    } catch(std::runtime_error& e) {
        error() << "AiPlugin::setupTask: Expecting AiTaskNode" << e.what();

    }
}

void AiPlugin::setupCondition(boost::shared_ptr<Node> node){
    try {
        node->getClosestParentOfType<Vehicle>()->attachGenerator(
                    boost::make_shared<MessageGenerator<AiConditionNode,
                    SetConditionStateMessage> >(node->to<AiConditionNode>())
                    );
        m_actions->scene->addItem(new LiquidConditionNode(node->to<AiConditionNode>()));
        //m_actions->scene->onNodeDroppedAt(node, m_actions->view->mapToScene(
        //                                        m_actions->view->mapFromGlobal(
        //                                          QCursor::pos()
        //                                          )));
        //m_filter->addNode(node);

    } catch(std::runtime_error& e) {
        error() << "AiPlugin::setupCondition: Expecting AiTaskNode" << e.what();

    }
}

void AiPlugin::setupVehicle(boost::shared_ptr<Node> vnode){
    try {
        boost::shared_ptr<Vehicle> vehicle = vnode->to<Vehicle>();
        boost::shared_ptr<GroupingNode> ai = vehicle->findOrCreate<GroupingNode>("ai");
        boost::shared_ptr<GroupingNode> tasks = ai->findOrCreate<GroupingNode>("tasks");
        boost::shared_ptr<GroupingNode> conditions = ai->findOrCreate<GroupingNode>("conditions");
        boost::shared_ptr<GroupingNode> pipelines = ai->findOrCreate<GroupingNode>("pipelines");

        connect(tasks.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupTask(boost::shared_ptr<Node>)));
        connect(conditions.get(), SIGNAL(childAdded(boost::shared_ptr<Node>)),
                this, SLOT(setupCondition(boost::shared_ptr<Node>)));

        boost::shared_ptr<CauvNode> node = m_actions->node.lock();
        if(node) {
            node->addMessageObserver(boost::make_shared<AiMessageObserver>(vehicle));
        } else error() << "Failed to lock CauvNode while setting up vehicle ai";


    } catch(std::runtime_error& e) {
        error() << "AiPlugin::setupVehicle: Expecting Vehicle Node" << e.what();
    }
}

void AiPlugin::reloadAi(){

    foreach(boost::shared_ptr<Vehicle> const& vehicle, VehicleRegistry::instance()->getVehicles()){
        try {
            boost::shared_ptr<GroupingNode> ai = vehicle->find<GroupingNode>("ai");
            boost::shared_ptr<GroupingNode> conditions = ai->find<GroupingNode>("conditions");
            foreach(boost::shared_ptr<AiConditionNode> const& node,
                    conditions->getChildrenOfType<AiConditionNode>()){
                conditions->removeChild(node->nodeId());
            }
            boost::shared_ptr<GroupingNode> tasks = ai->find<GroupingNode>("tasks");
            foreach(boost::shared_ptr<AiTaskNode> const& node,
                    tasks->getChildrenOfType<AiTaskNode>()){
                tasks->removeChild(node->nodeId());
            }
        } catch (std::out_of_range){
            info() << "no ai node on vehicle";
        }
    }

    if(boost::shared_ptr<CauvNode> cauvNode = m_actions->node.lock()) {
        cauvNode->send(boost::make_shared<RequestAIStateMessage>());
    }
}

Q_EXPORT_PLUGIN2(cauv_aiplugin, AiPlugin)
