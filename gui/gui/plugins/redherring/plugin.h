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

#ifndef CAUV_REDHERRINGPLUGIN_H
#define CAUV_REDHERRINGPLUGIN_H

#include <gui/core/cauvbasicplugin.h>
#include <gui/core/model/node.h>
#include <gui/core/model/nodes/vehiclenode.h>

#include <QObject>

namespace cauv {
    namespace gui {

        class RedHerringPlugin : public QObject, public CauvBasicPlugin
        {
            Q_OBJECT
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

        public:
            virtual const QString name() const;
            virtual void initialise();
        };

        class RedHerring : public Vehicle
        {
            Q_OBJECT
            public:
                friend class VehicleRegistry;

            protected:
                RedHerring(std::string name);
                virtual void initialise();

            protected Q_SLOTS:
                void setupMotor(boost::shared_ptr<MotorNode>);
                void setupAutopilot(boost::shared_ptr<AutopilotNode>);
        };

    } // namespace gui
} // namespace cauv

#endif // CAUV_REDHERRINGPLUGIN_H
