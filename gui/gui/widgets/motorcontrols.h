#ifndef MOTORCONTROLS_H
#define MOTORCONTROLS_H

#include <QDockWidget>

#include <boost/function.hpp>

#include <common/data_stream.h>
#include <model/auv_model.h> 

#include "cauvinterfaceelement.h"

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


    class MotorControls : public QDockWidget, public CauvInterfaceElement {

    public:
        MotorControls(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual ~MotorControls();
        virtual void initialise();

    protected:
        void setValue(QDoubleSpinBox *spin, double value);

    private:
        Ui::MotorControls * ui;
        std::vector<boost::shared_ptr<MotorBurstController> > m_burst_controllers;
        std::vector<boost::shared_ptr<AutopilotController> > m_autopilot_controllers;

    };
} // namespace cauv


#endif // MOTORCONTROLS_H
