/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef GAMEPADPLUGIN_H
#define GAMEPADPLUGIN_H

#include <gui/core/cauvbasicplugin.h>
#include <gui/core/model/node.h>

#include <QObject>

namespace cauv {
    namespace gui {

        class GamepadPlugin : public QObject, public CauvBasicPlugin
        {
            Q_OBJECT
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

        public:
            virtual const QString name() const;
            virtual void initialise();

        public Q_SLOTS:
            virtual void setVehicle(boost::shared_ptr<Vehicle>);

        };
    } // namespace gui
} // namespace cauv

#endif // GAMEPADPLUGIN_H
