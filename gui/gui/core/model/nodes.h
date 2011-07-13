#ifndef GUI_NODES_H
#define GUI_NODES_H

#include <QObject>
#include <QMetaType>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <stdexcept>

#include <generated/messages.h>
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

            GuiNodeType::e type;

            NodeBase(GuiNodeType::e t, const std::string name);

            virtual ~NodeBase();

            virtual std::string nodeName() const;
            virtual std::string nodePath() const;
            virtual void addChild(boost::shared_ptr<NodeBase> child);
            const std::vector<boost::shared_ptr<NodeBase> > getChildren() const;
            virtual bool isMutable() const;
            virtual void setMutable(bool mut);
            boost::shared_ptr<NodeBase> getRoot();

            template <class T> boost::shared_ptr<T> to() {
                if (dynamic_cast<T *>(this)) {
                    return boost::static_pointer_cast<T>(shared_from_this());
                } else {
                    throw std::runtime_error("Invalid node conversion");
                }
            }


            template <class T> const std::vector<boost::shared_ptr<T> > getChildrenOfType() const{

                std::vector<boost::shared_ptr<T> > output;
                foreach (boost::shared_ptr<NodeBase> child, getChildren()) {
                    if (dynamic_cast<T *>(child.get())) {
                        boost::shared_ptr<T> ptr = boost::static_pointer_cast<T>(child);
                        output.push_back(ptr);
                    }
                }

                return output;
            }

            template <class T> boost::shared_ptr<T> find(std::string name) const {
                debug() << "Looking for" << name << "in" << nodePath();
                foreach (boost::shared_ptr<T> child, getChildrenOfType<T>()) {
                    std::string childName = child->nodePath();
                    boost::to_lower(childName);
                    boost::to_lower(name);

                    if(childName == name) {
                        debug() << "Node matched" << child->nodePath();
                        return child;
                    }
                }
                std::stringstream str;
                str << "Node not found: " << name;
                throw std::out_of_range(str.str());
            }

            template <class T> boost::shared_ptr<T> findOrCreate(std::string name){
                lock_t l(m_updateLock);

                try {
                    return find<T>(name);
                } catch (std::out_of_range){
                    boost::shared_ptr<T> newNode = boost::make_shared<T>(name);
                    this->addChild(newNode);
                    info() << "New node added" << newNode->nodePath();
                    return newNode;
                }
            }

            template <class T> boost::shared_ptr<T> findOrCreateMutable(std::string name){
                boost::shared_ptr<T> node = findOrCreate<T>(name);
                node->setMutable(true);
                return node;
            }

        Q_SIGNALS:
            void nodeAdded(boost::shared_ptr<NodeBase> node);
            void changed();


        protected:
            boost::weak_ptr<NodeBase> m_parent;
            std::vector<boost::shared_ptr<NodeBase> > m_children;
            const std::string m_name;
            bool m_mutable;

            typedef boost::mutex mutex_t;
            typedef boost::unique_lock<mutex_t> lock_t;
            mutex_t m_updateLock;
        };



        template<typename char_T, typename traits>
        std::basic_ostream<char_T, traits>& operator<<(
            std::basic_ostream<char_T, traits>& os, NodeBase const& node)
        {
            os << node.nodePath() << "\n";

            foreach(boost::shared_ptr<NodeBase> child, node.getChildren()){
                os << child;
            }

            return os;
        }



        template <class T> class Node : public NodeBase {

        public:

            Node<T>(GuiNodeType::e type, const std::string name) :
                    NodeBase(type, name), m_value() {
            }

            virtual void update(const T & value){
                m_value = value;
            }

            virtual void set(const T & value){
                debug(0) << nodePath() << "set to" << value;
                update(value);
                info() << "change being emitted";
                Q_EMIT changed();
                info() << "change finished being emitted";
            }

            virtual const T get() const{
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


        class NumericNode : public Node<numeric_variant_t> {
            Q_OBJECT

        public:
            NumericNode(std::string name) : Node<numeric_variant_t>(GuiNodeType::NumericNode, name),
            m_maxSet(false), m_minSet(false), m_wraps(false), m_precision(3)
            {
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

            virtual numeric_variant_t getMax() {
                return m_max;
            }

            virtual numeric_variant_t getMin() {
                return m_min;
            }

            virtual bool getWraps() {
                return m_wraps;
            }

            virtual void setMax(numeric_variant_t max){
                m_max = max;
                m_maxSet = true;

                Q_EMIT paramsUpdated();
            }

            virtual bool isMaxSet(){
                return m_maxSet;
            }

            virtual bool isMinSet(){
                return m_minSet;
            }

            virtual void setMin(numeric_variant_t min){
                m_min = min;
                m_minSet = true;

                Q_EMIT paramsUpdated();
            }

            virtual void setWraps(bool wraps){
                m_wraps = wraps;

                Q_EMIT paramsUpdated();
            }

            virtual void setPrecision(unsigned int precision){
                m_precision = precision;

                Q_EMIT paramsUpdated();
            }

            virtual unsigned int getPrecision(){
                return m_precision;
            }

        public Q_SLOTS:

            virtual void update(const numeric_variant_t & value){
                Node<numeric_variant_t>::update(value);
                Q_EMIT onUpdate(value);

                switch(value.which()){
                case 0:
                    Q_EMIT onUpdate(boost::get<bool>(value));
                    break;
                case 1:
                    Q_EMIT onUpdate(boost::get<unsigned int>(value));
                    break;
                case 2:
                    Q_EMIT onUpdate(boost::get<int>(value));
                    break;
                case 3:
                    Q_EMIT onUpdate(boost::get<float>(value));
                    Q_EMIT onUpdate((double) boost::get<float>(value));
                    break;
                default:
                    throw std::exception();
                }
            }

            virtual void set(bool value){
                set(numeric_variant_t(value));
            }

            virtual void set(int value){
                set(numeric_variant_t(value));
            }

            virtual void set(unsigned int value){
                set(numeric_variant_t(value));
            }

            virtual void set(float value){
                set(numeric_variant_t(value));
            }

            virtual void set(double value){
                set(numeric_variant_t((float)value));
            }

            virtual void set(const numeric_variant_t & value){
                numeric_variant_t output;
                if(m_maxSet) {
                    output = boost::apply_visitor(limit_max(m_max), value);
                }
                if(m_minSet) {
                    output = boost::apply_visitor(limit_min(m_min), value);
                }

                Node<numeric_variant_t>::set(output);

                Q_EMIT onSet(output);

                switch(value.which()){
                case 0:
                    info() << "bool";
                    Q_EMIT onSet(boost::get<bool>(output));
                    break;
                case 1:
                    info() << "unsigned int";
                    Q_EMIT onSet(boost::get<unsigned int>(output));
                    break;
                case 2:
                    info() << "int";
                    Q_EMIT onSet(boost::get<int>(output));
                    break;
                case 3:
                    info() << "float";
                    Q_EMIT onSet(boost::get<float>(output));
                    Q_EMIT onSet((double) boost::get<float>(output));
                    break;
                default:
                    throw std::exception();
                }
            }

        Q_SIGNALS:
            void onUpdate(const numeric_variant_t value);
            void onUpdate(int value);
            void onUpdate(unsigned int value);
            void onUpdate(float value);
            void onUpdate(double value);
            void onUpdate(bool value);

            void paramsUpdated();

            void onSet(const numeric_variant_t value);
            void onSet(int value);
            void onSet(unsigned int value);
            void onSet(float value);
            void onSet(double value);
            void onSet(bool value);

        protected:
            std::string m_units;

            numeric_variant_t m_max;
            numeric_variant_t m_min;
            bool m_maxSet, m_minSet, m_wraps;
            unsigned int m_precision;
        };




        class StringNode : public Node<std::string> {
            Q_OBJECT

        public:
            StringNode(std::string name) : Node<std::string>(GuiNodeType::StringNode, name){
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



        class ImageNode : public Node<image_variant_t> {
            Q_OBJECT

        public:
            ImageNode(std::string name) : Node<image_variant_t>(GuiNodeType::ImageNode, name){
            }

        public Q_SLOTS:

            virtual void update(const image_variant_t & value){
                Node<image_variant_t>::update(value);
                Q_EMIT onUpdate(value);
            }

        Q_SIGNALS:
            void onUpdate(const image_variant_t value);
        };






        template<class T>
        class NumericCompoundNode : public Node<T> {

        public:

            enum compound_type {
                FLOAT_YPR,
                FLOAT_XYZ
            } type;

            NumericCompoundNode(GuiNodeType::e GuiNodeType, compound_type t, std::string name) :
                    Node<T>(GuiNodeType, name), type(t), m_chidrenAdded(false) {
            }

            virtual void forceSet() = 0;

            virtual void addChildren() = 0;

            void update(const T & value){
                Node<T>::update(value);
                if(!m_chidrenAdded)
                    addChildren();
                m_chidrenAdded = true;
            }

        protected:
            bool m_chidrenAdded;
        };



        class FloatYPRNode : public NumericCompoundNode <floatYPR> {
            Q_OBJECT

        public:
            FloatYPRNode(std::string name) : NumericCompoundNode<floatYPR>(GuiNodeType::FloatYPRNode, FLOAT_YPR, name),
            m_y(boost::make_shared<NumericNode>("yaw")),
            m_p(boost::make_shared<NumericNode>("pitch")),
            m_r(boost::make_shared<NumericNode>("roll"))
            {
                this->connect(this, SIGNAL(changed()), this, SLOT(forceSet()));
            }

        public Q_SLOTS:

            void addChildren(){
                this->addChild(m_y);
                this->addChild(m_p);
                this->addChild(m_r);
            }

            virtual void update(const floatYPR & value){
                NumericCompoundNode<floatYPR>::update(value);
                Q_EMIT onUpdate(value);
                m_y->update(value.yaw);
                m_p->update(value.pitch);
                m_r->update(value.roll);
            }

            virtual void set(const floatYPR & value){
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
            void onUpdate(const floatYPR value);
            void onSet(const floatYPR value);

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
                this->connect(this, SIGNAL(changed()), this, SLOT(forceSet()));
            }

        public Q_SLOTS:

            void addChildren(){
                this->addChild(m_x);
                this->addChild(m_y);
                this->addChild(m_z);
            }

            virtual void update(const floatXYZ & value){
                NumericCompoundNode<floatXYZ>::update(value);
                Q_EMIT onUpdate(value);
                m_x->update(value.x);
                m_y->update(value.y);
                m_z->update(value.z);
            }

            virtual void set(const floatXYZ & value){
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
            void onUpdate(const floatXYZ value);
            void onSet(const floatXYZ value);

        protected:
            boost::shared_ptr<NumericNode> m_x, m_y, m_z;

        };

    } // namespace gui
} // namespace cauv
#endif // GUI_NODES_H
