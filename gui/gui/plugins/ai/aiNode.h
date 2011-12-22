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

#ifndef __CAUV_ELEMENT_AI_NODE_H__
#define __CAUV_ELEMENT_AI_NODE_H__

#include <QGraphicsObject>

#include <boost/shared_ptr.hpp>

#include <gui/core/model/nodes/numericnode.h>

#include <liquid/node.h>

#include <gui/core/nodedragging.h>

namespace cauv {
    namespace gui {

        GENERATE_SIMPLE_NODE(AiMissionNode)
        GENERATE_SIMPLE_NODE(AiTaskNode)
        GENERATE_SIMPLE_NODE(AiConditionNode)
        GENERATE_SIMPLE_NODE(PipelineNode) //!!! todo: this shouldn't be deifned here

        class AiNode : public liquid::LiquidNode
        {
            Q_OBJECT
            typedef liquid::LiquidNode base_t;
        public:
            AiNode(QGraphicsItem *parent = 0);
            virtual ~AiNode();
        };


        class AiDropHandler : public DropHandlerInterface<QGraphicsItem * > {

            virtual bool accepts(boost::shared_ptr<Node> const& node){
                return (node->type == nodeType<AiMissionNode>() ||
                        node->type == nodeType<AiTaskNode>() ||
                        node->type == nodeType<AiConditionNode>());
            }

            virtual QGraphicsItem * handle(boost::shared_ptr<Node> const&) {

                info() << "added AiNode";

                return new AiNode();
            }
        };

    } // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_AI_NODE_H__
