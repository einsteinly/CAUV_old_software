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

            virtual void initialise(boost::shared_ptr<GuiActions>){};

            virtual const QString name() const = 0;

            virtual void initialise(){}

        };

    } // namespace gui
} // namespace cauv

Q_DECLARE_INTERFACE(cauv::gui::CauvInterfacePlugin, "CauvInterfacePlugin/1.1")

#endif // CAUVINTERFACEPLUGIN_H
