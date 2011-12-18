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

#include "aiNode.h"

#include <debug/cauv_debug.h>

#include <gui/core/framework/elements/style.h>

#include <generated/types/GuiaiGroup.h>

using namespace cauv;
using namespace cauv::gui;


AiNode::AiNode(QGraphicsItem *parent) :
        liquid::LiquidNode(AI_Node_Style, parent)
{
    setSize(QSizeF(200, 200));
}

AiNode::~AiNode(){
    debug(2) << "~AINode()";
}
