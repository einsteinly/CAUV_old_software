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


MotorControls::MotorControls(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QDockWidget(parent),
        CauvInterfaceElement(name, auv, node),
        ui(new Ui::MotorControls())
{
    ui->setupUi(this);

    // autopilots screen

    auv->sensors.orientation_split->yaw->onUpdate.connect(boost::bind(static_cast<void (QLabel::*)(double)>(&QLabel::setNum), ui->actualBearing, _1));
    auv->sensors.orientation_split->pitch->onUpdate.connect(boost::bind(static_cast<void (QLabel::*)(double)>(&QLabel::setNum), ui->actualPitch, _1));
    auv->sensors.depth->onUpdate.connect(boost::bind(static_cast<void (QLabel::*)(double)>(&QLabel::setNum), ui->actualDepth, _1));

    auv->autopilots["bearing"]->onUpdate.connect(boost::bind(&MotorControls::setValue, this, ui->bearingTarget, _1));
    auv->autopilots["pitch"]->onUpdate.connect(boost::bind(&MotorControls::setValue, this, ui->pitchTarget, _1));
    auv->autopilots["depth"]->onUpdate.connect(boost::bind(&MotorControls::setValue, this, ui->depthTarget, _1));

    auv->autopilots["bearing"]->enabled->onUpdate.connect(boost::bind(&QCheckBox::setChecked, ui->bearingEnabled, _1));
    auv->autopilots["pitch"]->enabled->onUpdate.connect(boost::bind(&QCheckBox::setChecked, ui->pitchEnabled, _1));
    auv->autopilots["depth"]->enabled->onUpdate.connect(boost::bind(&QCheckBox::setChecked, ui->depthEnabled, _1));

    connect(ui->bearingTarget, SIGNAL(valueChanged(double)), this, SLOT(bearingAutopilotTargetUpdated()));
    connect(ui->bearingEnabled, SIGNAL(clicked()), this, SLOT(bearingAutopilotStateUpdated()));

    connect(ui->pitchTarget, SIGNAL(valueChanged(double)), this, SLOT(pitchAutopilotTargetUpdated()));
    connect(ui->pitchEnabled, SIGNAL(clicked()), this, SLOT(pitchAutopilotStateUpdated()));

    connect(ui->depthTarget, SIGNAL(valueChanged(double)), this, SLOT(depthAutopilotTargetUpdated()));
    connect(ui->depthEnabled, SIGNAL(clicked()), this, SLOT(depthAutopilotStateUpdated()));

    // motor controls screen
    int count = 0;
    foreach(AUV::motor_map::value_type i, auv->motors){
        std::string forward = "Forward";
        std::string backward = "Backward";

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

void MotorControls::bearingAutopilotTargetUpdated(){
    m_auv->autopilots["bearing"]->set(ui->bearingTarget->value());
}
void MotorControls::bearingAutopilotStateUpdated(){
    m_auv->autopilots["bearing"]->enabled->set(ui->bearingEnabled->checkState());
}

void MotorControls::pitchAutopilotTargetUpdated(){
    m_auv->autopilots["pitch"]->set(ui->pitchTarget->value());
}
void MotorControls::pitchAutopilotStateUpdated(){
    m_auv->autopilots["pitch"]->enabled->set(ui->pitchEnabled->checkState());
}

void MotorControls::depthAutopilotTargetUpdated(){
    m_auv->autopilots["depth"]->set(ui->depthTarget->value());
}
void MotorControls::depthAutopilotStateUpdated(){
    m_auv->autopilots["depth"]->enabled->set(ui->depthEnabled->checkState());
}
