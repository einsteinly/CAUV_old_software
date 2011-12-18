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

#include <QMetaType>

#include <gui/core/model/node.h>

#include <common/image.h>

namespace cauv {
    namespace gui {

        typedef boost::shared_ptr<const Image> image_t;


        class ImageNode : public Node {
            Q_OBJECT

        public:
            ImageNode(nid_t const& id) : Node(id, nodeType<ImageNode>()){
                qRegisterMetaType<image_t>("image_t");
                m_value = QVariant::fromValue<image_t>(image_t());
            }

        public Q_SLOTS:
            virtual void update(image_t const& value){
                Node::update(QVariant::fromValue<image_t>(value));
            }
        };

    } //namespace gui
} // namespace cauv


Q_DECLARE_METATYPE(cauv::gui::image_t)

#endif // GUI_IMAGENODE_H
