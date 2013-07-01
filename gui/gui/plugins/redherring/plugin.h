/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef CAUV_REDHERRINGPLUGIN_H
#define CAUV_REDHERRINGPLUGIN_H

#include <cauvbasicplugin.h>
#include <model/node.h>
#include <model/nodes/vehiclenode.h>

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
                RedHerring(const std::string& name);
                virtual void initialise();

            protected Q_SLOTS:
                void setupMotor(boost::shared_ptr<MotorNode>);
                void setupAutopilot(boost::shared_ptr<AutopilotNode>);
                void setupImager(boost::shared_ptr<Node>);
        };

    } // namespace gui
} // namespace cauv

#endif // CAUV_REDHERRINGPLUGIN_H
