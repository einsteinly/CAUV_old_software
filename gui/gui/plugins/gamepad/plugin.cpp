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

GamepadPlugin::GamepadPlugin()
{
}

const QString GamepadPlugin::name() const{
    return QString("Gamepad");
}

const QList<QString> GamepadPlugin::getGroups() const{
    QList<QString> groups;
    groups.push_back(QString("gui"));
    return groups;
}

void GamepadPlugin::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node){
    CauvBasicPlugin::initialise(auv, node);

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
                    gi = new CauvGamepad(boost::make_shared<XBoxInput>(i->second), m_auv);
                } else {
                    // assume its a playstation controller
                    info() << "assuming playstation controller";
                    gi = new CauvGamepad(boost::make_shared<PlaystationInput>(i->second), m_auv);
                }

                gi->setParent(this);
            }
        }

    } catch (char const* ex){
        error() << ex;
    }
}

Q_EXPORT_PLUGIN2(cauv_gamepadplugin, GamepadPlugin)
