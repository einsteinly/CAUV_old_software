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



AutopilotController::AutopilotController(QCheckBox *enabled, QDoubleSpinBox *target, QLabel * actual, boost::shared_ptr<AUV::Autopilot<float> > autopilot):
        m_autopilot(autopilot),
        m_enabled(enabled),
        m_target(target),
        m_actual(actual) {

    //incoming events
    // forward the boost signals to Qt signals 
    autopilot->onUpdate.connect(boost::bind(&AutopilotController::targetUpdated, this, _1));
    autopilot->actual->onUpdate.connect(boost::bind(&AutopilotController::actualUpdated, this, _1));
    autopilot->enabled->onUpdate.connect(boost::bind(&AutopilotController::enabledUpdated, this, _1));

    // internal events, makes GUI updates happen in GUI thread
    connect(this, SIGNAL(targetUpdated(float)), this, SLOT(onTargetUpdate(float)));
    connect(this, SIGNAL(actualUpdated(float)), this, SLOT(onActualUpdate(float)));
    connect(this, SIGNAL(enabledUpdated(bool)), this, SLOT(onEnabledUpdate(bool)));

    //outgoing events
    enabled->connect(enabled, SIGNAL(clicked(bool)), this, SLOT(updateState(bool)));
    target->connect(target, SIGNAL(valueChanged(double)), this, SLOT(updateTarget(double)));
}

void AutopilotController::onEnabledUpdate(bool enabled){
    m_enabled->blockSignals(true);
    m_enabled->setChecked(enabled);
    m_enabled->blockSignals(false);
}

void AutopilotController::onTargetUpdate(float target){
    m_target->blockSignals(true);
    m_target->setValue(target);
    m_target->blockSignals(false);
}

void AutopilotController::onActualUpdate(float actual){
    m_actual->blockSignals(true);
    m_actual->setNum(actual);
    m_actual->blockSignals(false);
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
        target->setSuffix(QString::fromStdString(i.second->getUnits()));
        ui->autopilotControlsLayout->addWidget(target, count, 1, 1, 1, Qt::AlignCenter);

        QCheckBox * enabled = new QCheckBox("State");
        ui->autopilotControlsLayout->addWidget(enabled, count, 2, 1, 1, Qt::AlignCenter);

        QLabel * actual = new QLabel("Actual");
        actual->setMinimumSize(QSize(60, 0));
        actual->setMaximumSize(QSize(60, 16777215));
        ui->autopilotControlsLayout->addWidget(actual, count, 3, 1, 1, Qt::AlignCenter);

        // controller for the GUI view
        m_autopilot_controllers.push_back(boost::make_shared<AutopilotController>(enabled, target, actual, i.second));

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
