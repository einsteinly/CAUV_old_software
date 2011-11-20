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

#ifndef GUI_STRINGNODE_H
#define GUI_STRINGNODE_H

#include "../node.h"

namespace cauv {
    namespace gui {

        class StringNode : public Node {
            Q_OBJECT

        public:
            StringNode(nid_t const& id) : Node(GuiNodeType::StringNode, id){
            }

        public Q_SLOTS:

            virtual void update(QString const& value){
                Node::update(value);
            }

            virtual void update(std::string const& value){
                Node::update(QString::fromStdString(value));
            }

            virtual void set(QString const& value){
                Node::set(value);
            }

            virtual void set(std::string const& value){
                Node::set(QString::fromStdString(value));
            }
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_STRINGNODE_H
