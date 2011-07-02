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


        class NodeBase : virtual public QObject, public boost::enable_shared_from_this<NodeBase> {

        public:
            NodeBase(const std::string name) :
                    m_name(name), m_mutable(false) {
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

            Node<T>(const std::string name) :
                    NodeBase(name), m_value() {
            }

            virtual void update(T value){
                m_value = value;
            }

            virtual void set(T value){
                update(value);
            }

        protected:
            T m_value;

        };



        typedef boost::variant<int, float, int8_t, floatYPR, floatXYZ> numeric_variant_t;

        class NumericNode : public Node<numeric_variant_t> {
            Q_OBJECT

        public:
            NumericNode(std::string name) : Node<numeric_variant_t>(name){
            }

            virtual void update(numeric_variant_t value){
                Q_EMIT onUpdate(value);
            }

        Q_SIGNALS:
            void onUpdate(numeric_variant_t value);
            void onSet(numeric_variant_t value);
        };





        typedef const Image * image_variant_t;

        class ImageNode : public Node<image_variant_t> {
            Q_OBJECT

        public:
            ImageNode(std::string name) : Node<image_variant_t>(name){
            }

            virtual void update(image_variant_t value){
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

            NumericCompoundNode(compound_type t, std::string name) : Node<T>(name), type(t) {
            }
        };



        class FloatYPRNode : public NumericCompoundNode <floatYPR> {
            Q_OBJECT

        public:
            FloatYPRNode(std::string name) : NumericCompoundNode<floatYPR>(FLOAT_YPR, name),
                    m_y(boost::make_shared<NumericNode>("y")),
                    m_p(boost::make_shared<NumericNode>("p")),
                    m_r(boost::make_shared<NumericNode>("r"))
            {
                this->addChild(m_y);
                this->addChild(m_p);
                this->addChild(m_r);
            }

            virtual void update(floatYPR value){
                Q_EMIT onUpdate(value);
                m_y->update(value.yaw);
                m_p->update(value.pitch);
                m_r->update(value.roll);
            }

        Q_SIGNALS:
            void onUpdate(floatYPR value);

        protected:
            boost::shared_ptr<NumericNode> m_y, m_p, m_r;

        };




        class FloatXYZNode : public NumericCompoundNode <floatXYZ> {
            Q_OBJECT

        public:
            FloatXYZNode(std::string name) : NumericCompoundNode<floatXYZ>(FLOAT_XYZ, name),
                    m_x(boost::make_shared<NumericNode>("x")),
                    m_y(boost::make_shared<NumericNode>("y")),
                    m_z(boost::make_shared<NumericNode>("z"))
            {
                this->addChild(m_x);
                this->addChild(m_y);
                this->addChild(m_z);
            }

            virtual void update(floatXYZ value){
                Q_EMIT onUpdate(value);
                m_x->update(value.x);
                m_y->update(value.y);
                m_z->update(value.z);
            }

        Q_SIGNALS:
            void onUpdate(floatXYZ value);

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
