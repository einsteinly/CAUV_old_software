#ifndef MODEL_H
#define MODEL_H

#include <gui/core/streams/streams.h>

namespace cauv {
    namespace gui {

        class MessageGenerator : public QObject {
            Q_OBJECT

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<Message>);

        };

        template<class idtype, class streamtype, class contenttype>
        class Controller : public MessageGenerator, public MessageObserver {

            boost::unordered_map<idtype, boost::shared_ptr<streamtype> > m_map;
        public:

            class SetHandler : public DataStreamBase::DefaultChangeHandler<contenttype, streamtype> {
            public:
                SetHandler(streamtype * stream, Controller<idtype, streamtype, contenttype > * controller, idtype id) :
                        DataStreamBase::DefaultChangeHandler<contenttype, streamtype>(stream),
                        m_controller(controller), m_id(id) {
                }

                void set(contenttype value){
                    m_controller->set(m_id, value);
                }

                Controller<idtype, streamtype, contenttype> * m_controller;
                idtype m_id;
            };

        public:

            boost::shared_ptr<streamtype> get(idtype id){
                return m_map.at(id);
            }

            boost::unordered_map<idtype, boost::shared_ptr<streamtype> > getAll(){
                return m_map;
            }

            boost::shared_ptr<streamtype> add(idtype id, std::string name, std::string units = ""){
                m_map[id] = boost::make_shared<streamtype>(name, units);
                return m_map[id];
            }

            virtual void set(idtype id, contenttype t) = 0;
        };


        class MotorController : public Controller<MotorID::e, IntStream, IntStream::type> {

        public:
            MotorController(){}

            void onMotorStateMessage(MotorMessage_ptr m){
                get(m->motorId())->update(m->speed());
            }

            void set(MotorID::e id, int t){
                Q_EMIT messageGenerated(boost::make_shared<MotorMessage>(id, t));
                get(id)->update(t);
            }

            boost::shared_ptr<MotorController::SetHandler> makeChangeHandler(MotorID::e id){
                return boost::make_shared<MotorController::SetHandler>(get(id).get(), this, id);
            }

        };



        class AUV
        {
            MotorController motors;

        public:
            AUV(){
                motors.add(MotorID::Prop, "Prop")->setChangeHandler(motors.makeChangeHandler(MotorID::Prop));
                motors.add(MotorID::HBow, "H Bow")->setChangeHandler(motors.makeChangeHandler(MotorID::HBow));
                motors.add(MotorID::VBow, "V Bow")->setChangeHandler(motors.makeChangeHandler(MotorID::VBow));
                motors.add(MotorID::HStern, "H Stern")->setChangeHandler(motors.makeChangeHandler(MotorID::HStern));
                motors.add(MotorID::VStern, "v Stern")->setChangeHandler(motors.makeChangeHandler(MotorID::VStern));
            }
        };

    } // namespace gui
} // namespace cauv
#endif // MODEL_H
