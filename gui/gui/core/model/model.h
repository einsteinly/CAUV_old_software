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

#ifndef GUI_MODEL_H
#define GUI_MODEL_H

#include "node.h"

#include <generated/types/message.h>

#include "../model/nodes/groupingnode.h"



namespace cauv {
    namespace gui {

        class MessageGenerator;

        class Vehicle : public GroupingNode
        {
            Q_OBJECT

        public:
            friend class VehicleRegistry;

        protected:
            Vehicle(std::string name) : GroupingNode(name) {
            }

            virtual void initialise() = 0;

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<const Message>);

        protected:
            void addGenerator(boost::shared_ptr<MessageGenerator> generator);

            std::vector<boost::shared_ptr<MessageGenerator> > m_generators;
        };


        class RedHerring : public Vehicle
        {
            Q_OBJECT

        public:
            friend class VehicleRegistry;

        protected:
            RedHerring(std::string name);
            virtual void initialise();

        protected Q_SLOTS:
            void setupMotor(boost::shared_ptr<NodeBase>);
            void setupAutopilot(boost::shared_ptr<NodeBase> node);
        };

    } // namespace gui
} // namespace cauv

#endif // GUI_MODEL_H
