#ifndef GUI_NODES_H
#define GUI_NODES_H

#include <QObject>

#include <boost/enable_shared_from_this.hpp>
#include <boost/variant/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>

#include <vector>

#include <stdexcept>

#include <generated/messages.h>

#include <debug/cauv_debug.h>

namespace cauv {
    namespace gui {

        namespace GuiNodeType {
            enum e {
                NumericNode,
                ImageNode,
                FloatYPRNode,
                FloatXYZNode,
                GroupingNode

            };
        };

        class NodeBase : virtual public QObject, public boost::enable_shared_from_this<NodeBase> {

        public:

            GuiNodeType::e type;

            NodeBase(GuiNodeType::e t, const std::string name) :
                    type(t), m_name(name), m_mutable(false) {
            }

            virtual const std::string nodeName(const bool full=true) const {
                std::stringstream stream;

                boost::shared_ptr<NodeBase> parent = m_parent.lock();
                if (full && parent)
                    stream << parent->nodeName() << " " << m_name;
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
            }

            const std::vector<boost::shared_ptr<NodeBase> > getChildren() const {
                return m_children;
            }

            template <class T>
            boost::shared_ptr<const std::vector<boost::shared_ptr< const T> > > getChildrenOfType() const {

                boost::shared_ptr<const std::vector<boost::shared_ptr< const T> > > output =
                        boost::make_shared<const std::vector<boost::shared_ptr< const T> > >();

                foreach (boost::shared_ptr<T> child, m_children) {
                    if (dynamic_cast<T *>(child.get())) {
                        output->push_back(child);
                    }
                }

                return output;
            }

            virtual bool isMutable(){
                return m_mutable;
            }

            virtual void setMutable(bool mut){
                m_mutable = mut;
            }

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

            virtual void addChild(boost::shared_ptr<NodeBase>child){
                Node<std::string>::addChild(child);
                Q_EMIT nodeAdded(child);
            }

        Q_SIGNALS:
            void onUpdate(std::string value);
            void onSet(std::string value);
            void nodeAdded(boost::shared_ptr<NodeBase> node);
        };



        typedef boost::variant<int, float, int8_t, floatYPR, floatXYZ> numeric_variant_t;

        class NumericNode : public Node<numeric_variant_t> {
            Q_OBJECT

        public:
            NumericNode(std::string name) : Node<numeric_variant_t>(GuiNodeType::NumericNode, name){
            }

        public Q_SLOTS:

            virtual void update(numeric_variant_t value){
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
        };





        typedef const Image * image_variant_t;

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




/*



        class MessageGenerator : public QObject {
            Q_OBJECT

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<Message>);

        };


        template <class T, class controllertype, class streamtype>
                class SetHandler : public AbstractChangeHandler<T> {
                public:
            SetHandler<T, controllertype, streamtype>(boost::shared_ptr<controllertype> controller, boost::shared_ptr<streamtype> stream) :
                    m_controller(controller), m_stream(stream) {
            }

            void set(T){
                m_controller->sendUpdate(m_stream);
            }

            void update(T){
                // do nothing
            }

            bool isMutable(){
                return true;
            }

            boost::shared_ptr<controllertype> m_controller;
            boost::shared_ptr<streamtype> m_stream;
        };


        class Controller : public std::vector<boost::shared_ptr<DataStreamBase> >, public MessageGenerator, public MessageObserver, public boost::enable_shared_from_this<Controller> {

        public:

            void add(boost::shared_ptr<DataStreamBase> stream){
                push_back(stream);
            }

            template <class T, class controllertype, class streamtype>
                    void addWithHandler(boost::shared_ptr<streamtype> stream){
                add(stream);
                stream->setChangeHandler(boost::make_shared<SetHandler<T, controllertype, streamtype> >(stream, shared_from_this()));
            }

            virtual void sendUpdate(boost::shared_ptr<DataStreamBase> source);
        };


        class MotorController : public Controller {

        public:

            boost::shared_ptr<IntStream> speed;

            MotorController(MotorID::e id):
                    speed(boost::make_shared<IntStream>("speed")),
                    m_id(id) {
                addWithHandler<int, MotorController, IntStream>(speed);
            }


            void onMotorStateMessage(MotorMessage_ptr m){
                if(m->motorId() == m_id)
                    speed->update(m->speed());
            }

            void set(IntStream, int t){
                Q_EMIT messageGenerated(boost::make_shared<MotorMessage>(m_id, t));
            }

        protected:
            MotorID::e m_id;

        };




        class AUV
        {
            std::vector<boost::shared_ptr<Controller> > m_controllers;

        public:

            boost::shared_ptr<MotorController> prop;

            AUV() : prop(boost::make_shared<MotorController>(MotorID::Prop))
            {
                m_controllers.push_back(prop);

            }

            std::vector<boost::shared_ptr<Controller> > getControllers() {
                return m_controllers;
            }

        };*/

    } // namespace gui
} // namespace cauv
#endif // GUI_NODES_H
