/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef CAUVINTERFACEPLUGIN_H
#define CAUVINTERFACEPLUGIN_H

#include <boost/shared_ptr.hpp>

#include <Qt>
#include <QtPlugin>

#include <guiactions.h>
#include <connectednodemap.h>


class QString;

namespace cauv {

    class CauvNode;

    namespace gui {

        class Vehicle;

        class CauvInterfacePlugin {

        public:
            virtual ~CauvInterfacePlugin() {}

            virtual void initialise(boost::shared_ptr<GuiActions> const&, ConnectedNodeMap*) = 0;

            virtual const QString name() const = 0;

            virtual void initialise() = 0;

            virtual void shutdown() = 0;

        };

    } // namespace gui
} // namespace cauv

Q_DECLARE_INTERFACE(cauv::gui::CauvInterfacePlugin, "CauvInterfacePlugin/1.3")

#endif // CAUVINTERFACEPLUGIN_H
