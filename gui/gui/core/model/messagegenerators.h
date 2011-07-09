#ifndef MESSAGEGENERATORS_H
#define MESSAGEGENERATORS_H

#include <QObject>

#include <generated/messages_fwd.h>

#include <gui/core/model/nodes.h>

#include <boost/shared_ptr.hpp>


namespace cauv {
    namespace gui {

        class AUV;

        class MessageGenerator : public QObject
        {
            Q_OBJECT
        public:
            MessageGenerator(boost::shared_ptr<AUV> auv);

        protected:
            boost::shared_ptr<AUV> m_auv;

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<Message> message);

        };


        class MotorMessageGenerator : public MessageGenerator {
            Q_OBJECT
        public:
            MotorMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<NumericNode> motor, MotorID::e id):
                MessageGenerator(auv), m_id(id)
            {
                connect(motor.get(), SIGNAL(onSet(unsigned int)), this, SLOT(send(unsigned int)), Qt::DirectConnection);
                motor->set(10);
            }

        protected Q_SLOTS:
            void send(unsigned int value){
                Q_EMIT messageGenerated(boost::make_shared<MotorMessage>(m_id, (int8_t) value));
            }

        protected:
            MotorID::e m_id;

        };
    } // namespace gui
} // namesapce cauv

#endif // MESSAGEGENERATORS_H
