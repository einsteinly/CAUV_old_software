#ifndef GUI_STRINGNODE_H
#define GUI_STRINGNODE_H

#include "../node.h"

namespace cauv {
    namespace gui {

        class StringNode : public Node<std::string> {
            Q_OBJECT

        public:
            StringNode(id_variant_t const& id) : Node<std::string>(GuiNodeType::StringNode, id){
            }

        public Q_SLOTS:

            virtual void update(std::string  const& value){
                Node<std::string>::update(value);
            }

            virtual void set(std::string  const& value){
                Node<std::string>::set(value);
            }

        Q_SIGNALS:
            void onUpdate(std::string const& value);
            void onSet(std::string const& value);
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_STRINGNODE_H
