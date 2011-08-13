#ifndef GAMEPADPLUGIN_H
#define GAMEPADPLUGIN_H

#include <gui/core/cauvbasicplugin.h>

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

        };
    } // namespace gui
} // namespace cauv

#endif // GAMEPADPLUGIN_H
