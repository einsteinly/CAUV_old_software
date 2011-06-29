#ifndef MODEL_H
#define MODEL_H

#include <gui/core/streams/streams.h>

#include <boost/enable_shared_from_this.hpp>

#include <debug/cauv_debug.h>

namespace cauv {
    namespace gui {

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

        };

    } // namespace gui
} // namespace cauv
#endif // MODEL_H
