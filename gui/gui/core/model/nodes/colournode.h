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
/*
    static Colour qColorToColour(TypedQColor colour) {
        switch (colour.colorType()) {
        case ColourType::RGB:
            //return Colour::fromRGB(colour.redF(), colour.greenF(), colour.blueF());
        case ColourType::ARGB:
            //return Colour::fromRGBA(colour.redF(), colour.greenF(), colour.blueF(), colour.alphaF());
        case ColourType::BGR:
            //return Colour::fromBGR(colour.redF(), colour.greenF(), colour.blueF());
        case ColourType::BGRA:
            //return Colour::fromBGRA(colour.redF(), colour.greenF(), colour.blueF(), colour.alphaF());
        case ColourType::Grey:
            break;
        default:
            break;
            //return Colour::fromGrey(colour.value());
        }
        return Colour();
    }

    static QColor colorToQColour(Colour const& colour) {
        QColor c;

                switch(colour.type){
                    case ColourType::Grey:
                        c = TypedQColor::fromHsv(0, 0, colour.grey());

                    case ColourType::RGB:
                    case ColourType::ARGB:
                    case ColourType::BGR:
                    case ColourType::BGRA: {
                        c = TypedQColor::fromHsv(0, 0, colour.grey());

                    default:
                        warning() << "colorToQColour() - Unknown colour format";
                        c = TypedQColor::fromHsv(0, 0, colour.grey());

        return c;
    }

*/

public Q_SLOTS:
    virtual void update(Colour const& value){
        Node::update(QVariant::fromValue<Colour>(value));
    }

    virtual void update(QColor const& value){
        //update(qColorToColour(value));
    }
};
} //namespace gui
} // namespace cauv


#endif // __CAUV_GUI_COLOURNODE_H_
