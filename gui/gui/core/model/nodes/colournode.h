/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
