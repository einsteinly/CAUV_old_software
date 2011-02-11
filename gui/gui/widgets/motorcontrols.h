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

/*
    class DataStreamQtFwdBase : public QObject {
        Q_OBJECT

    public:
        DataStreamQtFwdBase(){}

    Q_SIGNALS:
        void onChange(const int value);
        void onChange(const bool value);
        void onChange(const int8_t value);
        void onChange(const float value);
        void onChange(const Image value);
        void onChange(const floatYPR value);
        void onChange(const floatXYZ value);
    };


    template<class T>
    class DataStreamQtFwd : public DataStreamQtFwdBase {
    public:

        DataStreamQtFwd(boost::shared_ptr<DataStream<T> > stream){
            stream->onUpdate.connect(boost::bind(&DataStreamQtFwd::change, this, _1));
        }

        void change(const T value){
            Q_EMIT this->onChange(value);
        }
    };
*/

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

    Q_SIGNALS:
        void enabledUpdated(bool enabled);
        void targetUpdated(float target);
        void actualUpdated(float actual);

    protected:

        template<class S>
        void emitQtSignal(boost::function<void(S)> func, S arg){
            Q_EMIT func(arg);
        }

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
