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

#include "ainode.h"

#include <debug/cauv_debug.h>

#include <gui/core/framework/elements/style.h>

#include <gui/plugins/ai/tasknode.h>

#include <generated/types/GuiaiGroup.h>

#include <common/cauv_node.h>

#include <QtGui>

using namespace cauv;
using namespace cauv::gui;


AiNode::AiNode(QGraphicsItem *parent) :
        liquid::LiquidNode(AI_Node_Style(), parent)
{
}

AiNode::~AiNode(){
    debug(2) << "~AINode()";
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
        return LiquidTaskNode::liquidNode(boost::static_pointer_cast<AiTaskNode>(node));
    }

    if (node->type == nodeType<AiConditionNode>()) {
        return LiquidConditionNode::liquidNode(boost::static_pointer_cast<AiConditionNode>(node));
    }

    return NULL;
}
