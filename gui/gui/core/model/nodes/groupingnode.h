#ifndef GUI_GROUPINGNODE_H
#define GUI_GROUPINGNODE_H

#include "../node.h"

namespace cauv {
    namespace gui {

        class GroupingNode : public Node<std::string> {
            Q_OBJECT

        public:
            GroupingNode(const id_variant_t id) : Node<std::string>(GuiNodeType::GroupingNode, id){
            }

        public Q_SLOTS:

            virtual void update(std::string const& value){
                Node<std::string>::update(value);
            }

            virtual void set(std::string const& value){
                Node<std::string>::set(value);
            }

        Q_SIGNALS:
            void onUpdate(std::string const& value);
            void onSet(std::string const& value);
        };


    } //namespace gui
} // namespace cauv

#endif // GUI_GROUPINGNODE_H
