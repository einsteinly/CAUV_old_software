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

#ifndef GUI_NODES_H
#define GUI_NODES_H

#include <QObject>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>

#include <vector>
#include <stdexcept>

#include <debug/cauv_debug.h>

#include "variants.h"

namespace cauv {
    namespace gui {

        namespace GuiNodeType {
            enum e {
                NumericNode,
                StringNode,
                ImageNode,
                FloatYPRNode,
                FloatXYZNode,
                GroupingNode
            };
        };



        class NodeBase : public QObject, public boost::enable_shared_from_this<NodeBase> {
            Q_OBJECT
        public:

            typedef boost::unordered_map<id_variant_t, boost::shared_ptr<NodeBase> > id_map_t;
            typedef std::vector<boost::shared_ptr<NodeBase> > children_list_t;

            GuiNodeType::e type;

            NodeBase(GuiNodeType::e t, id_variant_t const& id);

            virtual ~NodeBase();

            virtual id_variant_t nodeId() const;
            virtual std::string nodeName() const;
            virtual std::string nodePath() const;
            virtual void addChild(boost::shared_ptr<NodeBase> const& child);
            virtual const children_list_t getChildren() const;
            virtual bool isMutable() const;
            virtual void setMutable(bool mut);
            virtual boost::shared_ptr<NodeBase> getParent();

            boost::shared_ptr<NodeBase> getRoot();

            template<class T> boost::shared_ptr<T> getClosestParentOfType(){
                boost::shared_ptr<NodeBase> node = shared_from_this();

                while(node->m_parent.lock()) {
                    try {
                        return node->to<T>();
                    } catch (std::runtime_error ex) {
                    }
                    node = node->m_parent.lock();
                }
                throw std::out_of_range("Parent of correct not found");
            }

            template <class T> boost::shared_ptr<T> to() {
                if (dynamic_cast<T *>(this)) {
                    return boost::static_pointer_cast<T>(shared_from_this());
                } else {
                    throw std::runtime_error("Invalid node conversion");
                }
            }


            template <class T> const std::vector<boost::shared_ptr<T> > getChildrenOfType() const {
                std::vector<boost::shared_ptr<T> > output;

                BOOST_FOREACH (boost::shared_ptr<NodeBase> const& child, getChildren()) {
                    if (dynamic_cast<T *>(child.get())) {
                        boost::shared_ptr<T> ptr = boost::static_pointer_cast<T>(child);
                        output.push_back(ptr);
                    }
                }

                return output;
            }


            template <class T> boost::shared_ptr<T> find(id_variant_t const& id) const {
                // throws std::out_of_range_exception if its not found
                boost::shared_ptr<NodeBase> node = m_id_map.at(id);
                try {
                    return node->to<T>();
                } catch (std::runtime_error){
                    std::stringstream str;
                    str << "Node not found: " << boost::apply_visitor(id_to_name(), id);
                    throw std::out_of_range(str.str());
                }
            }

            template <class T> boost::shared_ptr<T> findOrCreate(id_variant_t const& id){
                lock_t l(m_creationLock);

                try {
                    return find<T>(id);
                } catch (std::out_of_range){
                    boost::shared_ptr<T> newNode = boost::make_shared<T>(id);
                    this->addChild(newNode);
                    info() << "New node added" << newNode->nodePath();
                    return newNode;
                }
            }

        Q_SIGNALS:
            void nodeAdded(boost::shared_ptr<NodeBase> node);
            void treeChanged();
            void changed();


        protected:
            boost::weak_ptr<NodeBase> m_parent;
            children_list_t m_children;
            id_map_t m_id_map;

            const id_variant_t m_id;
            bool m_mutable;

            typedef boost::mutex mutex_t;
            typedef boost::unique_lock<mutex_t> lock_t;
            mutex_t m_creationLock;
        };



        template<typename char_T, typename traits>
        std::basic_ostream<char_T, traits>& operator<<(
                std::basic_ostream<char_T, traits>& os, NodeBase const& node)
        {
            os << node.nodePath() << "\n";

            BOOST_FOREACH(boost::shared_ptr<NodeBase> child, node.getChildren()){
                os << child;
            }

            return os;
        }



        template <class T> class Node : public NodeBase {

        public:

            Node<T>(GuiNodeType::e type, id_variant_t const& id) :
                    NodeBase(type, id), m_value() {
            }

            virtual void update(T const& value){
                m_value = value;
                Q_EMIT onUpdate(value);
            }

            virtual void set(T const& value){
                debug(0) << nodePath() << "set to" << value;
                update(value);
                Q_EMIT changed();
                Q_EMIT onSet(value);
            }

            virtual const T get() const{
                return m_value;
            }

            boost::shared_ptr<T> makeMutable(){
                this->setMutable(true);
                return shared_from_this();
            }

            // to be implemeted as signals by subclasses
            virtual void onUpdate(T const& value) = 0;
            virtual void onSet(T const& value) = 0;

        protected:
            T m_value;
        };



        struct NodeFilterInterface {
            // return false to filter out an item (and its children) from the list
            virtual bool filter(boost::shared_ptr<NodeBase> const& node) = 0;

            // should be implmented as a signal by subclasses
            virtual void filterChanged() = 0;
        };

    } // namespace gui
} // namespace cauv
#endif // GUI_NODES_H
