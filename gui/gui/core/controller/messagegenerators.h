/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef MESSAGEGENERATORS_H
#define MESSAGEGENERATORS_H

#include <QObject>

#include <generated/types/message.h>
#include <generated/types/MotorID.h>
#include <generated/types/Controller.h>

#include <boost/shared_ptr.hpp>

#include "../model/node.h"

namespace cauv {
    namespace gui {

        template<class T> class NumericNode;
        class Node;

        class MessageGenerator : public QObject
        {
            Q_OBJECT
        public:
            MessageGenerator(boost::shared_ptr<Node> auv);

        protected:
            // weak_ptr breaks the auv -> generator list -> auv cycle
            boost::weak_ptr<Node> m_auv;

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<const Message> message);

        };


        class MotorMessageGenerator : public MessageGenerator {
            Q_OBJECT
        public:
            MotorMessageGenerator(boost::shared_ptr<Node> auv, boost::shared_ptr<NumericNode<int> > motor);

        protected Q_SLOTS:
            void send(QVariant value);

        protected:
            MotorID::e m_id;
        };


        class AutopilotMessageGenerator : public MessageGenerator {
            Q_OBJECT
        public:
            AutopilotMessageGenerator(boost::shared_ptr<Node> auv, boost::shared_ptr<Node> autopilot);

        protected Q_SLOTS:
            void send();

        protected:
            boost::shared_ptr<Node> m_autopilot;
            Controller::e m_id;
        };
    } // namespace gui
} // namesapce cauv

#endif // MESSAGEGENERATORS_H
