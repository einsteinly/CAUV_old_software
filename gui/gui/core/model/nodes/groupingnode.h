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

#ifndef GUI_GROUPINGNODE_H
#define GUI_GROUPINGNODE_H

#include "../node.h"

namespace cauv {
    namespace gui {

        class GroupingNode : public Node {
            Q_OBJECT

        public:
            GroupingNode(const nid_t id) : Node(GuiNodeType::GroupingNode, id){
            }

        public Q_SLOTS:

            virtual void update(QString const& value){
                Node::update(value);
                Q_EMIT onUpdate(value);
            }

            virtual bool set(QString const& value){
                return Node::set(value);
            }

        Q_SIGNALS:
            void onUpdate(QString const& value);
            void onSet(QString const& value);
        };


    } //namespace gui
} // namespace cauv

#endif // GUI_GROUPINGNODE_H
