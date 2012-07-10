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

#ifndef __CAUV_GUI_COLOURNODE_H_
#define __CAUV_GUI_COLOURNODE_H_

#include <gui/core/model/node.h>

#include <common/msg_classes/colour.h>

namespace cauv {
namespace gui {

class ColourNode : public Node {
    Q_OBJECT

public:
    ColourNode(nid_t const& id) : Node(id, nodeType<ColourNode>()){
        qRegisterMetaType<Colour>("Colour");
        m_value = QVariant::fromValue<Colour>(Colour());
    }

public Q_SLOTS:
    virtual void typedUpdate(Colour const& value){
        Node::update(QVariant::fromValue<Colour>(value));
    }
};
} //namespace gui
} // namespace cauv


#endif // __CAUV_GUI_COLOURNODE_H_
