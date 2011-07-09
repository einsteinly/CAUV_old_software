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
            MotorMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<NumericNode> motor, MotorID::e id);

        protected Q_SLOTS:
            void send(int value);

        protected:
            MotorID::e m_id;
        };


        class AutopilotMessageGenerator : public MessageGenerator {
            Q_OBJECT
        public:
            AutopilotMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<GroupingNode> autopilot);

        protected Q_SLOTS:
            void send();

        protected:
            boost::shared_ptr<GroupingNode> m_autopilot;
        };
    } // namespace gui
} // namesapce cauv

#endif // MESSAGEGENERATORS_H
