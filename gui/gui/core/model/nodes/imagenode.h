/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef GUI_IMAGENODE_H
#define GUI_IMAGENODE_H

#include <QMetaType>

#include <gui/core/model/node.h>

#warning TODO

namespace cauv {
    namespace gui {

//        typedef boost::shared_ptr<const BaseImage> image_t;

        class ImageNode : public Node {
            Q_OBJECT

        public:
            ImageNode(std::string const& id) : Node(id, nodeType<ImageNode>()){
//                qRegisterMetaType<image_t>("image_t");
//                m_value = QVariant::fromValue<image_t>(image_t());
            }

        public Q_SLOTS:
//            virtual void typedUpdate(image_t const& value){
//                Node::update(QVariant::fromValue<image_t>(value));
//            }
        };

    } //namespace gui
} // namespace cauv

//Q_DECLARE_METATYPE(cauv::gui::image_t)

#endif // GUI_IMAGENODE_H
