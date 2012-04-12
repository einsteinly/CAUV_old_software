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

#ifndef MOTORCONTROLS_H
#define MOTORCONTROLS_H

#include <QDockWidget>

#include <boost/function.hpp>

#include <utility/data_stream.h>
#include <model/auv_model.h> 

#include <gui/core/cauvbasicplugin.h>

namespace Ui {
    class MotorControls;
}

class QDoubleSpinBox;
class QCheckBox;
class QPushButton;
class QLabel;

namespace cauv {

    class MotorBurstController : public QObject {
        Q_OBJECT
    public:
        MotorBurstController(QPushButton *b, boost::shared_ptr<AUV::Motor> motor, int8_t speed);

    public Q_SLOTS:
        void burst();
        void stop();

    protected:
        int8_t m_speed;
        boost::shared_ptr<AUV::Motor> m_motor;
    };


    class AutopilotController : public QObject {
        Q_OBJECT
    public:

        AutopilotController(QCheckBox *enabled, QDoubleSpinBox *target, QLabel * actual, boost::shared_ptr<AUV::Autopilot<float> > autopilot);

    public Q_SLOTS:
        void updateState(bool value);
        void updateTarget(double value);

        void onEnabledUpdate(bool enabled);
        void onTargetUpdate(float target);
        void onActualUpdate(float actual);
        void targetEditingFinished();

    Q_SIGNALS:
        void enabledUpdated(bool enabled);
        void targetUpdated(float target);
        void actualUpdated(float actual);

    protected:
        boost::shared_ptr<AUV::Autopilot<float> > m_autopilot;

        QCheckBox * m_enabled;
        QDoubleSpinBox * m_target;
        QLabel * m_actual;

    };


    class MotorControls : public QDockWidget, public CauvBasicPlugin {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)
    public:
        MotorControls();
        virtual ~MotorControls();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);

    protected:
        void setValue(QDoubleSpinBox *spin, double value);

    private:
        Ui::MotorControls * ui;
        std::vector<boost::shared_ptr<MotorBurstController> > m_burst_controllers;
        std::vector<boost::shared_ptr<AutopilotController> > m_autopilot_controllers;

    };
} // namespace cauv


#endif // MOTORCONTROLS_H
