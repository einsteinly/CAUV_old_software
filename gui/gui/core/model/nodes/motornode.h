/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef GUI_MOTORNODE_H
#define GUI_MOTORNODE_H

#include <gui/core/model/node.h>
#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/nodes/groupingnode.h>

namespace cauv {
    namespace gui {

        class MotorNode : public NumericNode<int> {
        public:
            MotorNode(nid_t const& id) : NumericNode<int>(id) {
                typedSetMax(127);
                typedSetMin(-127);
            }
        };
        
        class MotorsNode : public GroupingNode {
        public:
            MotorsNode(nid_t const& id) : GroupingNode(id) {}
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_GROUPINGNODE_H
