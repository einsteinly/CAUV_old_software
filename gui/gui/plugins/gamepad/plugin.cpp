/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "plugin.h"

#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <QString>

#include <debug/cauv_debug.h>

#include <gui/core/model/registry.h>

#include "gamepad/playstationinput.h"
#include "gamepad/xboxinput.h"
#include "cauvgamepad.h"

using namespace cauv;
using namespace cauv::gui;

const QString GamepadPlugin::name() const{
    return QString("Gamepad");
}

void GamepadPlugin::initialise(){
    foreach(boost::shared_ptr<Vehicle> vehicle, VehicleRegistry::instance()->getVehicles()){
        setVehicle(vehicle);
    }

    connect(VehicleRegistry::instance().get(), SIGNAL(vehicleAdded(boost::shared_ptr<Vehicle>)),
            this, SLOT(setVehicle(boost::shared_ptr<Vehicle>)));
}


void GamepadPlugin::setVehicle(boost::shared_ptr<Vehicle> vehicle){

    try {
        info() << "found" << GamepadInput::getNumDevices() << "gamepads";

        OIS::DeviceList list = GamepadInput::listDevices();
        for( OIS::DeviceList::iterator i = list.begin(); i != list.end(); ++i ) {
            if(i->first == OIS::OISJoyStick){
                info() << "Device: " << "Gamepad" << " Vendor: " << i->second;
                std::string vendor = i->second;
                boost::to_lower(vendor);
                info() << "Connecting to" << vendor <<  "gamepad";

                CauvGamepad* gi;
                if(vendor.find("xbox") != vendor.npos){
                    info() << "detected as an xbox controller";
                    boost::shared_ptr<XBoxInput> controller(new XBoxInput(i->second));
                    gi = new CauvGamepad(controller, vehicle);
                } else {
                    // assume its a playstation controller
                    info() << "assuming playstation controller";
                    boost::shared_ptr<PlaystationInput> controller(new PlaystationInput(i->second));
                    gi = new CauvGamepad(controller, vehicle);
                }

                gi->setParent(this);
            }
        }

    } catch (char const* ex){
        error() << ex;
    } catch (std::runtime_error ex){
        error() << ex.what();
    }
}


Q_EXPORT_PLUGIN2(cauv_gamepadplugin, GamepadPlugin)
