/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef GUI_STRINGNODE_H
#define GUI_STRINGNODE_H

#include <gui/core/model/node.h>

namespace cauv {
    namespace gui {

        class StringNode : public Node {
            Q_OBJECT

        public:
            StringNode(std::string const& id) : Node(id, nodeType<StringNode>()){
                m_value = QVariant(QString());
            }

        public Q_SLOTS:

            virtual void update(QString const& value){
                update(QVariant(value));
            }

            virtual void update(const std::string& value){
                update(QString::fromStdString(value));
            }

            virtual void update(QVariant const& value){
                Node::update(QVariant(value.toString()));
            }

            virtual bool set(QString const& value){
                return set(QVariant(value));
            }

            virtual bool set(const std::string& value){
                return set(QString::fromStdString(value));
            }

            virtual bool set(QVariant const& value){
                return Node::set(QVariant(value.toString()));
            }
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_STRINGNODE_H
