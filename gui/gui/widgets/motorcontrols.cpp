#include "motorcontrols.h"
#include "ui_motorcontrols.h"

#include <QLabel>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>

#include <model/auv_model.h>

using namespace cauv;

MotorBurstController::MotorBurstController(QPushButton * b, boost::shared_ptr<AUV::Motor> motor, int8_t speed): m_speed(speed), m_motor(motor){
    b->connect(b, SIGNAL(pressed()), this, SLOT(burst()));
    b->connect(b, SIGNAL(released()), this, SLOT(stop()));
}

void MotorBurstController::burst(){
    m_motor->set(m_speed);
}

void MotorBurstController::stop() {
    m_motor->set(0);
}

AutopilotController::AutopilotController(QCheckBox *enabled, QDoubleSpinBox *target, boost::shared_ptr<AUV::Autopilot<float> > autopilot): m_autopilot(autopilot){
    enabled->connect(enabled, SIGNAL(clicked(bool)), this, SLOT(updateState(bool)));
    target->connect(target, SIGNAL(valueChanged(double)), this, SLOT(updateTarget(double)));
}

void AutopilotController::updateTarget(double value) {
    m_autopilot->set(value);
}

void AutopilotController::updateState(bool value) {
    m_autopilot->enabled->set(value);
}


MotorControls::MotorControls(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QDockWidget(parent),
        CauvInterfaceElement(name, auv, node),
        ui(new Ui::MotorControls())
{
    ui->setupUi(this);

    // autopilot controls screen
    int count = 0;
    foreach(AUV::autopilot_map::value_type i, auv->autopilots){
        // set up ui
        QLabel * label = new QLabel(QString::fromStdString(i.second->getName()));
        ui->autopilotControlsLayout->addWidget(label, count, 0, 1, 1, Qt::AlignCenter);

        QDoubleSpinBox * target = new QDoubleSpinBox();
        target->setWrapping(true);
        target->setButtonSymbols(QAbstractSpinBox::PlusMinus);
        target->setMaximum(i.second->getMax());
        target->setMinimum(i.second->getMin());
        target->setSuffix(QString::fromUtf8(i.second->getUnits().c_str()));
        ui->autopilotControlsLayout->addWidget(target, count, 1, 1, 1, Qt::AlignCenter);

        QCheckBox * enabled = new QCheckBox("State");
        ui->autopilotControlsLayout->addWidget(enabled, count, 2, 1, 1, Qt::AlignCenter);

        QLabel * actual = new QLabel("Actual");
        actual->setMinimumSize(QSize(60, 0));
        actual->setMaximumSize(QSize(60, 16777215));
        ui->autopilotControlsLayout->addWidget(actual, count, 3, 1, 1, Qt::AlignCenter);

        // set up signals
        i.second->onUpdate.connect(boost::bind(&MotorControls::setValue, this, target, _1));
        i.second->actual->onUpdate.connect(boost::bind(static_cast<void (QLabel::*)(double)>(&QLabel::setNum), actual, _1));
        i.second->enabled->onUpdate.connect(boost::bind(&QCheckBox::setChecked, enabled, _1));
        m_autopilot_controllers.push_back(boost::make_shared<AutopilotController>(enabled, target, i.second));

        count++;
    }


    // motor controls screen
    count = 0;
    foreach(AUV::motor_map::value_type i, auv->motors){
        std::string forward = "Forward";
        std::string backward = "Back";

        switch(i.first){
        case MotorID::HBow:
        case MotorID::HStern:
            forward = "Right";
            backward = "Left";
            break;
        case MotorID::VBow:
        case MotorID::VStern:
            forward = "Up";
            backward = "Down";
            break;
        default: break;
        }

        QPushButton * backButton = new QPushButton(QString::fromStdString(backward));
        m_burst_controllers.push_back(boost::make_shared<MotorBurstController>(backButton, i.second, -127));
        ui->motorControlsLayout->addWidget(backButton, ++count, 0, 1, 1, Qt::AlignCenter);

        QLabel * label = new QLabel(QString::fromStdString(i.second->getName()));
        label->setAlignment(Qt::AlignHCenter);
        ui->motorControlsLayout->addWidget(label, count, 1, 1, 1, Qt::AlignCenter);

        QPushButton * forwardButton = new QPushButton(QString::fromStdString(forward));
        m_burst_controllers.push_back(boost::make_shared<MotorBurstController>(forwardButton, i.second, 127));
        ui->motorControlsLayout->addWidget(forwardButton, count, 2, 1, 1, Qt::AlignCenter);
    }
}

MotorControls::~MotorControls(){
    delete ui;
}

void MotorControls::initialise(){
    m_actions->registerDockView(this, Qt::LeftDockWidgetArea);
}

void MotorControls::setValue(QDoubleSpinBox * spin, double value){
    spin->blockSignals(true);
    spin->setValue(value);
    spin->blockSignals(false);
}
