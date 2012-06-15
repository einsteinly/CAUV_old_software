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
        liquid::LiquidNode(AI_Node_Style(), parent)
{
}

AiNode::~AiNode(){
    debug(2) << "~AINode()";
}


AiDropHandler::AiDropHandler(boost::shared_ptr<NodeItemModel> model, boost::weak_ptr<CauvNode> cauv_node)
    : m_model(model), m_cauv_node(cauv_node){
}

bool AiDropHandler::accepts(boost::shared_ptr<Node> const& node){
    debug() << "::accepts" << node->nodePath();
    return (node->type == nodeType<AiMissionNode>() ||
            node->type == nodeType<AiTaskNode>() ||
            node->type == nodeType<PipelineNode>() ||
            node->type == nodeType<AiConditionNode>());
}

QGraphicsItem * AiDropHandler::handle(boost::shared_ptr<Node> const& node) {
    debug() << "AiDropHandler::handle" << node->type;

    if (node->type == nodeType<AiTaskNode>()) {
        LiquidTaskNode * n = ManagedNode::getLiquidNodeFor<LiquidTaskNode>(
                    boost::static_pointer_cast<AiTaskNode>(node));
        n->setSize(QSizeF(300,300));
        return n;
    }

    if (node->type == nodeType<PipelineNode>()) {
        debug() << "type == PipelineNode";
        LiquidPipelineNode * n = ManagedNode::getLiquidNodeFor<LiquidPipelineNode>(
                    boost::static_pointer_cast<PipelineNode>(node));
        debug() << n;
        // OK this is really ugly, but the only reasonable way I could find to get the
        // CauvNode to where it is needed
        n->ensureInited(m_cauv_node);
        n->setSize(QSizeF(300,300));
        return n; 
    }

    if (node->type == nodeType<AiConditionNode>()) {
        LiquidConditionNode * n = ManagedNode::getLiquidNodeFor<LiquidConditionNode>(
                    boost::static_pointer_cast<AiConditionNode>(node));
        n->setSize(QSizeF(300,300));
        return n;
    }

    return new AiNode();
}
