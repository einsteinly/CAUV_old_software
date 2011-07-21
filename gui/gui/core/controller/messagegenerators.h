#ifndef MESSAGEGENERATORS_H
#define MESSAGEGENERATORS_H

#include <QObject>

#include <gui/core/model/model.h>

#include <generated/types/message.h>
#include <generated/types/MotorID.h>

#include <boost/shared_ptr.hpp>


namespace cauv {
    namespace gui {

        class NumericNode;
        class NodeBase;

        class MessageGenerator : public QObject
        {
            Q_OBJECT
        public:
            MessageGenerator(boost::shared_ptr<AUV> auv);

        protected:
            boost::shared_ptr<AUV> m_auv;

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<const Message> message);

        };


        class MotorMessageGenerator : public MessageGenerator {
            Q_OBJECT
        public:
            MotorMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<NumericNode> motor);

        protected Q_SLOTS:
            void send(int value);

        protected:
            MotorID::e m_id;
        };


        class AutopilotMessageGenerator : public MessageGenerator {
            Q_OBJECT
        public:
            AutopilotMessageGenerator(boost::shared_ptr<AUV> auv, boost::shared_ptr<NodeBase> autopilot);

        protected Q_SLOTS:
            void send();

        protected:
            boost::shared_ptr<NodeBase> m_autopilot;
            AutopilotID::e m_id;
        };
    } // namespace gui
} // namesapce cauv

#endif // MESSAGEGENERATORS_H
