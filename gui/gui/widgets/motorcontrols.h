#ifndef MOTORCONTROLS_H
#define MOTORCONTROLS_H

#include <QDockWidget>

#include <common/data_stream.h>
#include <model/auv_model.h> 

#include "cauvinterfaceelement.h"

namespace Ui {
    class MotorControls;
}

class QDoubleSpinBox;
class QPushButton;

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


    class MotorControls : public QDockWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        MotorControls(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual ~MotorControls();
        virtual void initialise();

    protected:
        void setValue(QDoubleSpinBox *spin, double value);

    protected Q_SLOTS:
        void bearingAutopilotTargetUpdated();
        void bearingAutopilotStateUpdated();

        void pitchAutopilotTargetUpdated();
        void pitchAutopilotStateUpdated();

        void depthAutopilotTargetUpdated();
        void depthAutopilotStateUpdated();

    private:
        Ui::MotorControls * ui;
        std::vector<boost::shared_ptr<MotorBurstController> > m_burst_controllers;

    };
} // namespace cauv


#endif // MOTORCONTROLS_H
