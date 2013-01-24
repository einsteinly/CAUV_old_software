/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_NODES_H__
#define __CAUV_GUI_NODES_H__

#include <QtGui>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <boost/unordered_map.hpp>

#include <vector>

#include <debug/cauv_debug.h>

#include <model/variants.h>
#include <model/nodeType.h>


#define GENERATE_SIMPLE_NODE(X) \
    class X : public Node { \
        public: \
            X(const nid_t id) : Node(id, nodeType<X>()){ \
        } \
    };

namespace cauv {
namespace gui {

class Node : public QObject, public boost::enable_shared_from_this<Node> {
    Q_OBJECT
    public:
        typedef boost::unordered_map<nid_t, boost::shared_ptr<Node> > id_map_t;
        typedef std::vector<boost::shared_ptr<Node> > children_list_t;

        node_type type;

        Node(nid_t const& id, node_type type);
        virtual ~Node();

        virtual nid_t nodeId() const;
        virtual std::string nodeName() const;
        virtual std::string nodePath() const;
        virtual void addChild(boost::shared_ptr<Node> const& child);
        virtual bool removeChild(boost::shared_ptr<Node> const& child);
        virtual bool removeChild(nid_t const& childId);
        virtual const children_list_t getChildren() const;
        virtual bool isMutable() const;
        virtual void setMutable(bool mut);

        // QAbstractItemModel helpers
        virtual int row() const;
        virtual int columnCount() const;

        boost::shared_ptr<Node> getParent();
        boost::shared_ptr<Node> getRoot();

        template<class T> boost::shared_ptr<T> getClosestParentOfType(){
            boost::shared_ptr<Node> node = shared_from_this();

            while(node->m_parent.lock()) {
                // TODO: get rid of this exception handling, do it a neater way
                try {
                    return node->to<T>();
                } catch (std::runtime_error ex) {
                }
                node = node->m_parent.lock();
            }
            throw std::out_of_range("Parent of correct type not found");
        }

        template <class T> boost::shared_ptr<T> to() {
            if (dynamic_cast<T *>(this)) {
                return boost::static_pointer_cast<T>(shared_from_this());
            } else {
                throw std::runtime_error("Invalid node conversion");
            }
        }

        template <class T> size_t countChildrenOfType() const {
            size_t n = 0;
            foreach (boost::shared_ptr<Node> const& child, getChildren()) {
                if (dynamic_cast<T *>(child.get())) {
                    ++n;
                }
            }
            return n;
        }

        template <class T> const std::vector<boost::shared_ptr<T> > getChildrenOfType() const {
            std::vector<boost::shared_ptr<T> > output;

            foreach (boost::shared_ptr<Node> const& child, getChildren()) {
                if (dynamic_cast<T *>(child.get())) {
                    output.push_back(boost::static_pointer_cast<T>(child));
                }
            }

            return output;
        }

        template <class T> boost::shared_ptr<T> find(nid_t const& id) const {
            // throws std::out_of_range_exception if its not found
            if(exists<T>(id)) {
                boost::shared_ptr<Node> node = m_id_map.at(id);
                return node->to<T>();
            } else {
                std::stringstream str;
                str << "Node not found: " << boost::apply_visitor(id_to_name(), id);
                throw std::out_of_range(str.str());
            }
        }


        template <class T> boost::shared_ptr<T> findFromPath(QString path) const {
            QStringList pathparts = path.split('/');
            if(pathparts.length() > 1){
                QString prefixNode = path;
                prefixNode.truncate(path.indexOf('/'));
                boost::shared_ptr<Node> node = find<Node>(nid_t(
                                prefixNode.toStdString()));
                return node->findFromPath<T>(path.remove(0, path.indexOf('/')+1));
            }
            else return find<T>(nid_t(path.toStdString()));
        }

        template <class T> bool exists(nid_t const& id) const {
            return m_id_map.find(id) != m_id_map.end();
        }

        template <class T> boost::shared_ptr<T> findOrCreate(nid_t const& id){
            lock_t l(m_creationLock);

            if(exists<T>(id)) {
                return find<T>(id);
            } else {
                boost::shared_ptr<T> newNode = boost::make_shared<T>(id);
                this->addChild(newNode);
                info() << "New node added" << newNode->nodePath();
                return newNode;
            }
        }

    public Q_SLOTS:

        virtual void update(QVariant const& value);
        virtual bool set(QVariant const& value);
        virtual const QVariant get() const;

    Q_SIGNALS:
        // strctural signals
        void childAdded(boost::shared_ptr<Node>);
        void childRemoved(boost::shared_ptr<Node>);
        void detachedFrom(boost::shared_ptr<Node>);
        void attachedTo(boost::shared_ptr<Node>);
        void structureChanged();
        // data change signals
        void onBranchChanged();
        void onUpdate(QVariant const& value);
        void onUpdate();
        void onSet(QVariant const& value);
        void onSet();


    protected:
        QVariant m_value;

        boost::weak_ptr<Node> m_parent;
        children_list_t m_children;
        id_map_t m_id_map;

        const nid_t m_id;
        bool m_mutable;

        typedef boost::mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;
        mutex_t m_creationLock;
        mutable mutex_t m_childrenLock;
};



template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
        std::basic_ostream<char_T, traits>& os, Node const& node)
{
    os << node.nodePath() << "\n";

    foreach(boost::shared_ptr<Node> child, node.getChildren()){
        os << child;
    }

    return os;
}

template<class X>
struct TypedNodeStore {
    TypedNodeStore<X>(boost::shared_ptr<X> node) : m_node(node){}
protected:
    boost::shared_ptr<X> m_node;
};

} // namespace gui
} // namespace cauv
#endif // GUI_NODES_H
