#ifndef MODEL_H
#define MODEL_H

#include <gui/core/streams/streams.h>

#include <debug/cauv_debug.h>

namespace cauv {
    namespace gui {

        class MessageGenerator : public QObject {
            Q_OBJECT

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<Message>);

        };

/*
        template<class contenttype, class streamtype>
        class SetHandler : public DataStreamBase::DefaultChangeHandler<contenttype, streamtype> {
        public:
            SetHandler(Controller * controller, streamtype * stream) :
                    DataStreamBase::DefaultChangeHandler<contenttype, streamtype>(stream),
                    m_controller(controller), m_stream(stream) {
            }

            void set(contenttype value){
                m_controller->set<contenttype>(m_stream, value);
            }

            Controller * m_controller;
            streamtype * m_stream;
        };*/


        class Controller : public std::vector<boost::shared_ptr<DataStreamBase> >, public MessageGenerator, public MessageObserver {

        public:

            void add(boost::shared_ptr<DataStreamBase> stream){
                push_back(stream);
            }

            void addWithHandler(boost::shared_ptr<DataStreamBase> stream){
                add(stream);
                //stream->setChangeHandler(boost::make_shared<SetHandler>(stream.get(), this));
            }

            template<class contenttype>
            virtual void set(DataStreamBase, contenttype t) = 0;
        };


        class MotorController : public Controller {

        public:

            boost::shared_ptr<IntStream> speed;

            MotorsController(MotorID::e id):
                    speed(boost::make_shared<IntStream>("speed")){
                addWithHandler(speed);
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

            boost::shared_ptr<MotorController> motors;

            AUV() : motors(boost::make_shared<MotorController>())
            {
                m_controllers.push_back(motors);

                motors->addWithHandler(MotorID::Prop, "Prop");
                motors->addWithHandler(MotorID::HBow, "H Bow");
                motors->addWithHandler(MotorID::VBow, "V Bow");
                motors->addWithHandler(MotorID::HStern, "H Stern");
                motors->addWithHandler(MotorID::VStern, "V Stern");
            }

            std::vector<boost::shared_ptr<Controller> > getControllers() {
                return m_controllers;
            }

        };

    } // namespace gui
} // namespace cauv
#endif // MODEL_H
