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

using namespace cauv;
using namespace cauv::gui;


AiNode::AiNode(QGraphicsItem *parent) :
        liquid::LiquidNode(AI_Node_Style, parent)
{
}

AiNode::~AiNode(){
    debug(2) << "~AINode()";
}



bool AiDropHandler::accepts(boost::shared_ptr<Node> const& node){
    return (node->type == nodeType<AiMissionNode>() ||
            node->type == nodeType<AiTaskNode>() ||
            node->type == nodeType<AiConditionNode>());
}

QGraphicsItem * AiDropHandler::handle(boost::shared_ptr<Node> const& node) {

    if (node->type == nodeType<AiTaskNode>()) {
        LiquidTaskNode * n = new LiquidTaskNode(boost::static_pointer_cast<AiTaskNode>(node));
        n->setSize(QSizeF(300,300));
        return n;
    }

    return new AiNode();
}
