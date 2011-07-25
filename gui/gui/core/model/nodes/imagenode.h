#ifndef GUI_IMAGENODE_H
#define GUI_IMAGENODE_H

#include "../node.h"

namespace cauv {
    namespace gui {


        class ImageNode : public Node<image_variant_t> {
            Q_OBJECT

        public:
            ImageNode(const id_variant_t id) : Node<image_variant_t>(GuiNodeType::ImageNode, id){
            }

        public Q_SLOTS:

            virtual void update(const image_variant_t & value){
                Node<image_variant_t>::update(value);
                Q_EMIT onUpdate(value);
            }

        Q_SIGNALS:
            void onUpdate(const image_variant_t value);
        };



    } //namespace gui
} // namespace cauv

#endif // GUI_IMAGENODE_H
