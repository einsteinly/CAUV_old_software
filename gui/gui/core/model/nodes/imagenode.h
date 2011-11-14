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

#ifndef GUI_IMAGENODE_H
#define GUI_IMAGENODE_H

#include "../node.h"

namespace cauv {
    namespace gui {


        class ImageNode : public Node<image_variant_t> {
            Q_OBJECT

        public:
            ImageNode(id_variant_t const& id) : Node<image_variant_t>(GuiNodeType::ImageNode, id){
            }

        public Q_SLOTS:

            virtual void update(image_variant_t const& value){
                Node<image_variant_t>::update(value);
            }

        Q_SIGNALS:
            void onUpdate(image_variant_t const& value);
            void onSet(image_variant_t const& value);
        };


    } //namespace gui
} // namespace cauv

#endif // GUI_IMAGENODE_H
