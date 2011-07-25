#ifndef GUI_STRINGNODE_H
#define GUI_STRINGNODE_H

#include "../node.h"

namespace cauv {
    namespace gui {

        class StringNode : public Node<std::string> {
            Q_OBJECT

        public:
            StringNode(const id_variant_t id) : Node<std::string>(GuiNodeType::StringNode, id){
            }

        public Q_SLOTS:

            virtual void update(const std::string & value){
                Node<std::string>::update(value);
                Q_EMIT onUpdate(value);
            }

            virtual void set(const std::string & value){
                Node<std::string>::set(value);
                Q_EMIT onSet(value);
            }

        Q_SIGNALS:
            void onUpdate(const std::string value);
            void onSet(const std::string value);
        };


    } //namespace gui
} // namespace cauv

#endif // GUI_STRINGNODE_H
