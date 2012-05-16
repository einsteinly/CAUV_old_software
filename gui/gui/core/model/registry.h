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

#ifndef REGISTRY_H
#define REGISTRY_H

#include <QObject>
#include <QUrl>

#include <boost/shared_ptr.hpp>

#include "nodes/groupingnode.h"

#include "model.h"


namespace cauv {
    namespace gui {

        class VehicleRegistry : public GroupingNode
        {
            Q_OBJECT
            public:
                static boost::shared_ptr<VehicleRegistry> instance()
                {
                    static boost::shared_ptr<VehicleRegistry> m_instance(new VehicleRegistry());
                    return m_instance;
                }

                template <class T> boost::shared_ptr<T> registerVehicle(std::string name){
                    boost::shared_ptr<Vehicle> vehicle(new T(name));
                    connect(vehicle.get(), SIGNAL(observerGenerated(boost::shared_ptr<MessageObserver>)),
                            this, SIGNAL(observerGenerated(boost::shared_ptr<MessageObserver>)));
                    connect(vehicle.get(), SIGNAL(messageGenerated(boost::shared_ptr<const Message>)),
                            this, SIGNAL(messageGenerated(boost::shared_ptr<const Message>)));
                    vehicle->initialise();
                    addChild(vehicle);
                    return boost::static_pointer_cast<T>(vehicle);
                }

                const std::vector<boost::shared_ptr<Vehicle> > getVehicles() const;
                const boost::shared_ptr<Node> getNode(QUrl url);

            Q_SIGNALS:
                void messageGenerated(boost::shared_ptr<const Message>);
                void observerGenerated(boost::shared_ptr<MessageObserver>);

            private:
                VehicleRegistry();
                VehicleRegistry(VehicleRegistry const&);
                void operator=(VehicleRegistry const&);
        };

    } // namespace gui
} // namespace cauv


#endif // REGISTRY_H
