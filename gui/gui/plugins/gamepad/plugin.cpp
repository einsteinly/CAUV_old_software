#include "plugin.h"

#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <QDockWidget>
#include <QPushButton>
#include <QString>

#include <debug/cauv_debug.h>

#include "gamepad/playstationinput.h"
#include "gamepad/xboxinput.h"
#include "cauvgamepad.h"

using namespace cauv;
using namespace cauv::gui;

const QString GamepadPlugin::name() const{
    return QString("Gamepad");
}

void GamepadPlugin::initialise(){

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
                    gi = new CauvGamepad(controller, m_auv);
                } else {
                    // assume its a playstation controller
                    info() << "assuming playstation controller";
                    boost::shared_ptr<PlaystationInput> controller(new PlaystationInput(i->second));
                    gi = new CauvGamepad(controller, m_auv);
                }

                gi->setParent(this);
            }
        }

    } catch (char const* ex){
        error() << ex;
    }
}

Q_EXPORT_PLUGIN2(cauv_gamepadplugin, GamepadPlugin)
