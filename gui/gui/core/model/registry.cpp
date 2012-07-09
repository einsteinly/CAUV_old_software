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

#include "registry.h"

#include <QStringList>

using namespace cauv;
using namespace cauv::gui;

VehicleRegistry::VehicleRegistry() : GroupingNode("")
{
}

const std::vector<boost::shared_ptr<Vehicle> > VehicleRegistry::getVehicles() const {
    return getChildrenOfType<Vehicle>();
}

const boost::shared_ptr<Node> VehicleRegistry::getNode(QUrl url) {
    if(url.scheme() == "varstream"){
        QString path = url.path().prepend(url.host()).prepend("/");
        QStringList list = path.split("/", QString::SkipEmptyParts);

        boost::shared_ptr<Node> parent = shared_from_this();
        foreach(QString part, list){
            std::string str = part.toStdString();
            parent = parent->find<Node>(str);
        }

        return parent;
    } else throw std::runtime_error("scheme must be 'varstream' for stream URLs");
}
