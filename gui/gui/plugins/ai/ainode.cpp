/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "ainode.h"

#include <QtGui>

#include <debug/cauv_debug.h>

#include <elements/style.h>

#include <generated/types/AiGroup.h>

#include <liquid/nodeHeader.h>

#include <common/cauv_node.h>

#include "ai/tasknode.h"
#include "ai/conditionnode.h"

using namespace cauv;
using namespace cauv::gui;


AiNode::AiNode(boost::shared_ptr<Node> node, QGraphicsItem *parent) :
    ConnectedNode(node, AI_Node_Style(), parent)
{
    node->connect(node.get(), SIGNAL(detachedFrom(boost::shared_ptr<Node>)), this, SLOT(deleteLater()));

    setTitle(QString::fromStdString(node->nodeName()));
}

AiNode::~AiNode(){
    debug(2) << "~AINode()";
}

void AiNode::close(){
    Q_EMIT LiquidNode::closed(this);
    QPropertyAnimation *fade = new QPropertyAnimation(this, "opacity");
    fade->setEndValue(0.25);
    fade->setDuration(200);
    fade->start();
}


AiDropHandler::AiDropHandler(boost::shared_ptr<NodeItemModel> model, boost::weak_ptr<CauvNode> node)
    : m_model(model), m_node(node){
}

bool AiDropHandler::accepts(boost::shared_ptr<Node> const& node){
    return (node->type == nodeType<AiMissionNode>() ||
            node->type == nodeType<AiTaskTypeNode>() ||
            node->type == nodeType<AiTaskNode>() ||
            node->type == nodeType<AiConditionTypeNode>() ||
            node->type == nodeType<AiConditionNode>());
}

QGraphicsItem * AiDropHandler::handle(boost::shared_ptr<Node> const& node) {

    if (node->type == nodeType<AiTaskTypeNode>()) {
        if(boost::shared_ptr<CauvNode> cauvNode = m_node.lock()){
            cauvNode->send(boost::make_shared<AddTaskMessage>(boost::get<std::string>(node->nodeId())));
        }
        return NULL;
    }

    if (node->type == nodeType<AiConditionTypeNode>()) {
        if(boost::shared_ptr<CauvNode> cauvNode = m_node.lock()){
            cauvNode->send(boost::make_shared<AddConditionMessage>(boost::get<std::string>(node->nodeId())));
        }
        return NULL;
    }

    if (node->type == nodeType<AiTaskNode>()) {
        return NULL;//LiquidTaskNode::liquidNode(boost::static_pointer_cast<AiTaskNode>(node));
    }

    if (node->type == nodeType<AiConditionNode>()) {
        return NULL;//LiquidConditionNode::liquidNode(boost::static_pointer_cast<AiConditionNode>(node));
    }

    return NULL;
}
