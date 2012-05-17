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

#ifndef GUI_VEHICLENODE_H
#define GUI_VEHICLENODE_H

#include <gui/core/model/messaging.h>

#include <generated/types/message.h>

namespace cauv {
    namespace gui {

        class BaseMessageGenerator;

        class Vehicle : public Node
        {
            Q_OBJECT

        public:
            friend class VehicleRegistry;

        protected:
            Vehicle(std::string name) : Node(name, nodeType<Vehicle>()) {
            }

            virtual void initialise() = 0;

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<const Message>);
            void observerGenerated(boost::shared_ptr<MessageObserver>);

        public:

            void attachGenerator(boost::shared_ptr<BaseMessageGenerator> generator)
            {
                generator->connect(generator.get(), SIGNAL(messageGenerated(boost::shared_ptr<const Message>)),
                                   this, SIGNAL(messageGenerated(boost::shared_ptr<const Message>)));
                if(dynamic_cast<MessageObserver*>(generator.get()))
                    attachObserver(boost::dynamic_pointer_cast<MessageObserver>(generator));
                else
                    m_generators.insert(generator);
            }

            void attachObserver(boost::shared_ptr<MessageObserver> handler)
            {
                Q_EMIT observerGenerated(handler);
            }

            std::set<boost::shared_ptr<BaseMessageGenerator> > m_generators;
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_VEHICLENODE_H
