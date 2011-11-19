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

        class Vehicle;

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
