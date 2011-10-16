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
