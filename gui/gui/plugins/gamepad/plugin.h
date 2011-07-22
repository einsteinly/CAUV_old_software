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
            virtual const QList<QString> getGroups() const;
            virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);

        };
    } // namespace gui
} // namespace cauv

#endif // GAMEPADPLUGIN_H
