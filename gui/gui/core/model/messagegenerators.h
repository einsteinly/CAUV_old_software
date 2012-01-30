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

#include <gui/core/model/node.h>

namespace cauv {
    namespace gui {

        class Node;

        struct MessageGenerator : public QObject
        {
            Q_OBJECT

        public:
            void attach(boost::shared_ptr<Node> to) {
                if(m_attachedTo)
                    disconnect(m_attachedTo.get(), SIGNAL(onBranchChanged()), this, SLOT(generate()));
                m_attachedTo = to;
                connect(m_attachedTo.get(), SIGNAL(onBranchChanged()), this, SLOT(generate()));
            }

        private Q_SLOTS:
            virtual void generate(){
                if(m_attachedTo)
                    Q_EMIT messageGenerated(generate(m_attachedTo));
            }

        public Q_SLOTS:
            virtual boost::shared_ptr<const Message> generate(boost::shared_ptr<Node> attachedTo) = 0;

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<const Message> message);

        protected:
            boost::shared_ptr<Node> m_attachedTo;

        };


        struct MotorMessageGenerator : public MessageGenerator {
            Q_OBJECT
        public Q_SLOTS:
            boost::shared_ptr<const Message> generate(boost::shared_ptr<Node> attachedTo);
        };

        struct AutopilotMessageGenerator : public MessageGenerator {
            Q_OBJECT
        public Q_SLOTS:
            boost::shared_ptr<const Message> generate(boost::shared_ptr<Node> attachedTo);
        };

    } // namespace gui
} // namesapce cauv

#endif // MESSAGEGENERATORS_H
