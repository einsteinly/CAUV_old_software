/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
