#ifndef CAUVINTERFACEPLUGIN_H
#define CAUVINTERFACEPLUGIN_H

#include <boost/shared_ptr.hpp>

#include <Qt>
#include <QtPlugin>

#include "framework/guiactions.h"

class QString;

namespace cauv {

    class CauvNode;

    namespace gui {

        class AUV;

        class CauvInterfacePlugin {

        public:
            virtual ~CauvInterfacePlugin() {}

            virtual void initialise(boost::shared_ptr<GuiActions> const&) = 0;

            virtual const QString name() const = 0;

            virtual void initialise() = 0;

            virtual void shutdown() = 0;

        };

    } // namespace gui
} // namespace cauv

Q_DECLARE_INTERFACE(cauv::gui::CauvInterfacePlugin, "CauvInterfacePlugin/1.1")

#endif // CAUVINTERFACEPLUGIN_H
