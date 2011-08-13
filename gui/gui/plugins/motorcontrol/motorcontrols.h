#ifndef MOTORCONTROLS_H
#define MOTORCONTROLS_H

#include <QDockWidget>

#include <boost/function.hpp>

#include <gui/core/model/model.h>

#include <gui/core/cauvbasicplugin.h>

namespace Ui {
    class MotorControls;
}

class QDoubleSpinBox;
class QCheckBox;
class QPushButton;
class QLabel;

namespace cauv {
    namespace gui {

        template<class T> class TypedNumericNode;
        class NodeBase;

        class MotorBurstController : public QObject {
            Q_OBJECT
        public:
            MotorBurstController(boost::shared_ptr<TypedNumericNode<int8_t> > motor, int8_t speed);

        public Q_SLOTS:
            void burst();
            void stop();

        protected:
            int8_t m_speed;
            boost::shared_ptr<TypedNumericNode<int8_t> > m_motor;
        };


    class AutopilotController : public QObject {
        Q_OBJECT
    public:

        AutopilotController(QCheckBox *enabled, QDoubleSpinBox *target, QLabel * actual, boost::shared_ptr<NodeBase> autopilot);

    public Q_SLOTS:
        void updateTarget(double target);
        void targetEditingFinished();
        void configureTarget();

    protected:
        boost::shared_ptr<NodeBase> m_autopilot;

        QCheckBox * m_enabled;
        QDoubleSpinBox * m_target;
        QLabel * m_actual;

    };


        class MotorControls : public QDockWidget, public CauvBasicPlugin {
            Q_OBJECT
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)
        public:
                    MotorControls();
            virtual ~MotorControls();

            virtual void initialise();

            const QString name() const;

        protected:
            void setValue(QDoubleSpinBox *spin, double value);
            int m_motorsCount;
            int m_autopilotsCount;

        protected Q_SLOTS:
            void addMotor(boost::shared_ptr<NodeBase> motor);
            void addAutopilot(boost::shared_ptr<NodeBase> ap);

        private:
            Ui::MotorControls * ui;
            std::vector<boost::shared_ptr<MotorBurstController> > m_burst_controllers;
            std::vector<boost::shared_ptr<AutopilotController> > m_autopilot_controllers;

        };
    } // namespace gui
} // namespace cauv


#endif // MOTORCONTROLS_H
