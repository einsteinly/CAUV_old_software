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

#include <gui/core/model/nodes/groupingnode.h>
#include <gui/core/model/messagegenerators.h>

#include <generated/types/message.h>

namespace cauv {
    namespace gui {

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
            void addGenerator(boost::shared_ptr<MessageGenerator> generator)
            {
                m_generators.push_back(generator);
                generator->connect(generator.get(), SIGNAL(messageGenerated(boost::shared_ptr<const Message>)),
                                   this, SIGNAL(messageGenerated(boost::shared_ptr<const Message>)));
            }

            std::vector<boost::shared_ptr<MessageGenerator> > m_generators;
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_VEHICLENODE_H
