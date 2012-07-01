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

//#include <generated/types/Colour.h>

namespace cauv {
    namespace gui {

        class ColourNode : public Node {
            Q_OBJECT

        public:
            ColourNode(nid_t const& id) : Node(id, nodeType<ColourNode>()){
                m_value = QVariant::fromValue<QColor>(Qt::black);
            }

        protected:
            QColor colourToQColour(Colour const& colour) const{
                /*switch(colour.colourType()){
                    case RGBA:
                    case RGB:
                    return QColor::fromRgbF(colour.red(), colour.green(). colour.blue(), colour.alpha());

                    case Grayscale:
                    return QColor(qGray(qRgba(colour.red(), colour.green(). colour.blue(), colour.alpha())));
                default:
                    warning() << "ColourNode::colourToQColour() - Unknown colour format";*/
                    return Qt::black;
                //}
            }

        public Q_SLOTS:
            virtual void update(QColor const& value){
                Node::update(QVariant::fromValue<QColor>(value));
            }

            virtual void update(Colour const& value){
                Node::update(QVariant::fromValue<QColor>(colourToQColour(value)));
            }
        };

    } //namespace gui
} // namespace cauv


#endif // __CAUV_GUI_COLOURNODE_H_
