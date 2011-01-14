#ifndef MOTORCONTROLS_H
#define MOTORCONTROLS_H

#include <QDockWidget>

#include "cauvinterfaceelement.h"

namespace Ui {
    class MotorControls;
}

class QDoubleSpinBox;

namespace cauv {

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

    };


} // namespace cauv


#endif // MOTORCONTROLS_H
