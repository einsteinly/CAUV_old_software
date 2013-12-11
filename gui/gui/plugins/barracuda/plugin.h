/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef CAUV_BARRACUDAPLUGIN_H
#define CAUV_BARRACUDAPLUGIN_H

//has to be included early, there's some issue if it's included after the model/node stage
#include <ros/subscriber.h>

#include <cauvbasicplugin.h>
#include <model/node.h>
#include <model/nodes/vehiclenode.h>

#include <QObject>

namespace cauv {
    namespace gui {

        class BarracudaPlugin : public QObject, public CauvBasicPlugin
        {
            Q_OBJECT
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

        public:
            virtual const QString name() const;
            virtual void initialise();
        private:
            //ros::Subscriber m_telemetry_motor_sub;
            ros::Subscriber m_telemetry_attitude_sub;
            ros::Subscriber m_telemetry_depth_sub;
        };

        class Barracuda : public Vehicle
        {
            Q_OBJECT
            public:
                friend class VehicleRegistry;

            protected:
                Barracuda(const std::string& name);
                virtual void initialise();

            protected Q_SLOTS:
                void setupMotor(boost::shared_ptr<MotorNode>);
                void setupAutopilot(boost::shared_ptr<AutopilotNode>);
                void setupImager(boost::shared_ptr<Node>);
        };

    } // namespace gui
} // namespace cauv

#endif // CAUV_BARRACUDAPLUGIN_H
