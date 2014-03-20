/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef REGISTRY_H
#define REGISTRY_H

#include <QtGui>

#include <boost/shared_ptr.hpp>

#include <model/nodes/groupingnode.h>
#include <model/nodes/vehiclenode.h>

namespace cauv {

    class BaseMessageObserver;

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

                template <class T> boost::shared_ptr<T> registerVehicle(const std::string& name){
                    boost::shared_ptr<Vehicle> vehicle(new T(name));
//                     connect(vehicle.get(), SIGNAL(observerAttached(boost::shared_ptr<BaseMessageObserver>)),
//                             this, SIGNAL(observerAttached(boost::shared_ptr<BaseMessageObserver>)));
//                     connect(vehicle.get(), SIGNAL(observerDetached(boost::shared_ptr<BaseMessageObserver>)),
//                             this, SIGNAL(observerDetached(boost::shared_ptr<BaseMessageObserver>)));
                    vehicle->initialise();
                    addChild(vehicle);
                    Q_EMIT vehicleAdded(vehicle);
                    return boost::static_pointer_cast<T>(vehicle);
                }

                const std::vector<boost::shared_ptr<Vehicle> > getVehicles() const;
                const boost::shared_ptr<Node> getNode(QUrl url);

            Q_SIGNALS:
                //void observerAttached(boost::shared_ptr<BaseMessageObserver>);
                //void observerDetached(boost::shared_ptr<BaseMessageObserver>);
                void vehicleAdded(boost::shared_ptr<Vehicle>);

            private:
                VehicleRegistry();
                VehicleRegistry(VehicleRegistry const&);
                void operator=(VehicleRegistry const&);
        };

    } // namespace gui
} // namespace cauv


#endif // REGISTRY_H
