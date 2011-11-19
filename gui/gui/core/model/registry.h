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

namespace cauv {
    namespace gui {

        class Vehicle;

        class VehicleRegistry : public GroupingNode
        {
            Q_OBJECT

            public:
                static boost::shared_ptr<VehicleRegistry> instance()
                {
                    static boost::shared_ptr<VehicleRegistry> m_instance(new VehicleRegistry());
                    return m_instance;
                }

                void registerVehicle(boost::shared_ptr<Vehicle> vehicle);
                const std::vector<boost::shared_ptr<Vehicle> > getVehicles() const;
                const boost::shared_ptr<NodeBase> getNode(QUrl url);

            private:
                VehicleRegistry();
                VehicleRegistry(VehicleRegistry const&);
                void operator=(VehicleRegistry const&);
        };

    } // namespace gui
} // namespace cauv


#endif // REGISTRY_H
