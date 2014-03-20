/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_COLOURNODE_H_
#define __CAUV_GUI_COLOURNODE_H_

#include <gui/core/model/node.h>

namespace cauv {
namespace gui {

#warning TODO

class ColourNode : public Node {
    Q_OBJECT

public:
    ColourNode(std::string const& id) : Node(id, nodeType<ColourNode>()){
#if 0
        qRegisterMetaType<Colour>("Colour");
        m_value = QVariant::fromValue<Colour>(Colour());
#endif
    }

public Q_SLOTS:
#if 0
    virtual void typedUpdate(Colour const& value){
        Node::update(QVariant::fromValue<Colour>(value));
    }
#endif
};
} //namespace gui
} // namespace cauv


#endif // __CAUV_GUI_COLOURNODE_H_
