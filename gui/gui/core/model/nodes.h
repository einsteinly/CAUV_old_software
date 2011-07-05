
#ifndef GUI_NODES_H
#define GUI_NODES_H

#include <QObject>
#include <QMetaType>

#include <boost/enable_shared_from_this.hpp>
#include <boost/variant/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>

#include <stdexcept>

#include <generated/messages.h>

#include <debug/cauv_debug.h>

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

        class NodeBase : virtual public QObject, public boost::enable_shared_from_this<NodeBase> {
            Q_OBJECT
        public:

            GuiNodeType::e type;

            NodeBase(GuiNodeType::e t, const std::string name) :
                    type(t), m_parent(), m_name(name), m_mutable(false) {

                qRegisterMetaType<boost::shared_ptr<NodeBase> >("boost::shared_ptr<NodeBase>");


            }

            virtual ~NodeBase(){
                debug(0) << "~NodeBase" << nodeName();
            }

            virtual const std::string nodeName(const bool full=true) const {
                std::stringstream stream;

                if(full && !m_parent.expired()) {
                    boost::shared_ptr<NodeBase> parent = m_parent.lock();
                    if(parent)
                        stream << parent->nodeName() << "/" << m_name;
                }
                else stream << m_name;

                return stream.str();
            }

            virtual void addChild(boost::shared_ptr<NodeBase> child){
                if(boost::shared_ptr<NodeBase> parent = child->m_parent.lock())
                {
                    std::stringstream str;
                    str << "Node already has a parent: " << parent->nodeName();
                    throw std::runtime_error(str.str());
                }

                child->m_parent = boost::weak_ptr<NodeBase>(shared_from_this());
                m_children.push_back(child);

                Q_EMIT nodeAdded(shared_from_this(), child);
            }

            const std::vector<boost::shared_ptr<NodeBase> > getChildren() const {
                return m_children;
            }

            template <class T> const std::vector<boost::shared_ptr<T> > getChildrenOfType() const {

                std::vector<boost::shared_ptr<T> > output;
                foreach (boost::shared_ptr<NodeBase> child, getChildren()) {
                    if (dynamic_cast<T *>(child.get())) {
                        boost::shared_ptr<T> ptr = boost::static_pointer_cast<T>(child);
                        output.push_back(ptr);
                    }
                }

                return output;
            }

            template <class T> boost::shared_ptr<T> find(std::string name){
                debug(0) << "Looking for" << name << "in" << nodeName();
                foreach (boost::shared_ptr<T> child, getChildrenOfType<T>()) {
                    std::string childName = child->nodeName(false);
                    boost::to_lower(childName);
                    boost::to_lower(name);

                    if(childName == name) {
                        debug(0) << "Node matched" << child->nodeName();
                        return child;
                    }
                }
                debug() << name << "is not a child of" << nodeName();
                std::stringstream str;
                str << "Node not found: " << name;
                throw std::out_of_range(str.str());
            }

            template <class T> boost::shared_ptr<T> findOrCreate(std::string name){
                try {
                    return find<T>(name);
                } catch (std::out_of_range){
                    boost::shared_ptr<T> newNode = boost::make_shared<T>(name);
                    this->addChild(newNode);
                    info() << "New node added" << newNode->nodeName();
                    return newNode;
                }
            }


            template <class T> boost::shared_ptr<T> findOrCreateMutable(std::string name){
                boost::shared_ptr<T> node = findOrCreate<T>(name);
                node->setMutable(true);
                return node;
            }

            virtual bool isMutable(){
                return m_mutable;
            }

            virtual void setMutable(bool mut){
                m_mutable = mut;
            }

        Q_SIGNALS:
            void nodeAdded(boost::shared_ptr<NodeBase> parent, boost::shared_ptr<NodeBase> node);

        protected:
            boost::weak_ptr<NodeBase> m_parent;
            std::vector<boost::shared_ptr<NodeBase> > m_children;
            const std::string m_name;
            bool m_mutable;
        };






        template <class T>

                class Node : public NodeBase {

                public:

            Node<T>(GuiNodeType::e type, const std::string name) :
                    NodeBase(type, name), m_value() {
            }

            virtual void update(T value){
                m_value = value;
            }

            virtual void set(T value){
                debug(0) << nodeName() << "set to" << value;
                update(value);
            }

            virtual T get(){
                return m_value;
            }

        protected:
            T m_value;

        };




        class GroupingNode : public Node<std::string> {
            Q_OBJECT

        public:
            GroupingNode(std::string name) : Node<std::string>(GuiNodeType::GroupingNode, name){
            }

        public Q_SLOTS:

            virtual void update(std::string value){
                Node<std::string>::update(value);
                Q_EMIT onUpdate(value);
            }

            virtual void set(std::string value){
                Node<std::string>::set(value);
                Q_EMIT onSet(value);
            }

        Q_SIGNALS:
            void onUpdate(std::string value);
            void onSet(std::string value);
        };



        typedef boost::variant<bool, unsigned int, int, float> numeric_variant_t;

        class NumericNode : public Node<numeric_variant_t> {
            Q_OBJECT

        public:
            NumericNode(std::string name) : Node<numeric_variant_t>(GuiNodeType::NumericNode, name){
                qRegisterMetaType<numeric_variant_t>("numeric_variant_t");
            }

            virtual std::string getUnits(){
                return m_units;
            }

            virtual void setUnits(std::string units) {
                m_units = units;
            }

            numeric_variant_t numericVariantFromString(std::string value){
                int type = get().which();

                switch(type){
                case 0:
                    return numeric_variant_t(boost::lexical_cast<bool>(value));
                case 1:
                    return numeric_variant_t(boost::lexical_cast<unsigned int>(value));
                case 2:
                    return numeric_variant_t(boost::lexical_cast<int>(value));
                case 3:
                    return numeric_variant_t(boost::lexical_cast<float>(value));
                default:
                    throw std::exception();
                }
            }


        public Q_SLOTS:

            virtual void update(numeric_variant_t value){
                debug() << nodeName() << " value set to " << value;
                Node<numeric_variant_t>::update(value);
                Q_EMIT onUpdate(value);
            }

            virtual void set(numeric_variant_t value){
                Node<numeric_variant_t>::set(value);
                Q_EMIT onSet(value);
            }

        Q_SIGNALS:
            void onUpdate(numeric_variant_t value);
            void onSet(numeric_variant_t value);

        protected:
            std::string m_units;
        };




        class StringNode : public Node<std::string> {
            Q_OBJECT

        public:
            StringNode(std::string name) : Node<std::string>(GuiNodeType::StringNode, name){
            }

        public Q_SLOTS:

            virtual void update(std::string value){
                Node<std::string>::update(value);
                Q_EMIT onUpdate(value);
            }

            virtual void set(std::string value){
                Node<std::string>::set(value);
                Q_EMIT onSet(value);
            }

        Q_SIGNALS:
            void onUpdate(std::string value);
            void onSet(std::string value);
        };




        typedef boost::shared_ptr<const Image> image_variant_t;

        class ImageNode : public Node<image_variant_t> {
            Q_OBJECT

        public:
            ImageNode(std::string name) : Node<image_variant_t>(GuiNodeType::ImageNode, name){
            }

        public Q_SLOTS:

            virtual void update(image_variant_t value){
                Node<image_variant_t>::update(value);
                Q_EMIT onUpdate(value);
            }

        Q_SIGNALS:
            void onUpdate(image_variant_t value);
        };






        template<class T>
        class NumericCompoundNode : public Node<T> {

        public:

            enum compound_type {
                FLOAT_YPR,
                FLOAT_XYZ
            } type;

            NumericCompoundNode(GuiNodeType::e GuiNodeType, compound_type t, std::string name) :
                    Node<T>(GuiNodeType, name), type(t) {
            }

            virtual void forceSet() = 0;
        };



        class FloatYPRNode : public NumericCompoundNode <floatYPR> {
            Q_OBJECT

        public:
            FloatYPRNode(std::string name) : NumericCompoundNode<floatYPR>(GuiNodeType::FloatYPRNode, FLOAT_YPR, name),
            m_y(boost::make_shared<NumericNode>("y")),
            m_p(boost::make_shared<NumericNode>("p")),
            m_r(boost::make_shared<NumericNode>("r"))
            {
                this->addChild(m_y);
                this->addChild(m_p);
                this->addChild(m_r);

                this->connect(m_y.get(), SIGNAL(onSet(numeric_variant_t)), this, SLOT(forceSet()));
                this->connect(m_p.get(), SIGNAL(onSet(numeric_variant_t)), this, SLOT(forceSet()));
                this->connect(m_r.get(), SIGNAL(onSet(numeric_variant_t)), this, SLOT(forceSet()));
            }

        public Q_SLOTS:

            virtual void update(floatYPR value){
                NumericCompoundNode<floatYPR>::update(value);
                Q_EMIT onUpdate(value);
                m_y->update(value.yaw);
                m_p->update(value.pitch);
                m_r->update(value.roll);
            }

            virtual void set(floatYPR value){
                NumericCompoundNode<floatYPR>::set(value);
                Q_EMIT onSet(value);
            }

            virtual void forceSet() {
                float y = boost::get<float>(m_y->get());
                float p = boost::get<float>(m_p->get());
                float r = boost::get<float>(m_r->get());
                set(floatYPR(y, p, r));
            }

        Q_SIGNALS:
            void onUpdate(floatYPR value);
            void onSet(floatYPR value);

        protected:
            boost::shared_ptr<NumericNode> m_y, m_p, m_r;

        };




        class FloatXYZNode : public NumericCompoundNode <floatXYZ> {
            Q_OBJECT

        public:
            FloatXYZNode(std::string name) : NumericCompoundNode<floatXYZ>(GuiNodeType::FloatXYZNode, FLOAT_XYZ, name),
            m_x(boost::make_shared<NumericNode>("x")),
            m_y(boost::make_shared<NumericNode>("y")),
            m_z(boost::make_shared<NumericNode>("z"))
            {
                this->addChild(m_x);
                this->addChild(m_y);
                this->addChild(m_z);

                this->connect(m_x.get(), SIGNAL(onSet(numeric_variant_t)), this, SLOT(forceSet()));
                this->connect(m_y.get(), SIGNAL(onSet(numeric_variant_t)), this, SLOT(forceSet()));
                this->connect(m_z.get(), SIGNAL(onSet(numeric_variant_t)), this, SLOT(forceSet()));
            }

        public Q_SLOTS:

            virtual void update(floatXYZ value){
                NumericCompoundNode<floatXYZ>::update(value);
                Q_EMIT onUpdate(value);
                m_x->update(value.x);
                m_y->update(value.y);
                m_z->update(value.z);
            }

            virtual void set(floatXYZ value){
                NumericCompoundNode<floatXYZ>::set(value);
                Q_EMIT onSet(value);
            }

            virtual void forceSet(){
                float x = boost::get<float>(m_x->get());
                float y = boost::get<float>(m_y->get());
                float z = boost::get<float>(m_z->get());
                set(floatXYZ(x, y, z));
            }

        Q_SIGNALS:
            void onUpdate(floatXYZ value);
            void onSet(floatXYZ value);

        protected:
            boost::shared_ptr<NumericNode> m_x, m_y, m_z;

        };


    } // namespace gui
} // namespace cauv
#endif // GUI_NODES_H
