#include "motorcontrols.h"

#include <QLabel>
#include <QDoubleSpinBox>
#include <QCheckBox>

using namespace cauv;

MotorControls::MotorControls(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
    QDockWidget(parent),
    CauvInterfaceElement(name, auv, node)
{
    setupUi(this);

    auv->sensors.orientation_split->yaw->onUpdate.connect(boost::bind(static_cast<void (QLabel::*)(double)>(&QLabel::setNum), actualBearing, _1));
    auv->sensors.orientation_split->pitch->onUpdate.connect(boost::bind(static_cast<void (QLabel::*)(double)>(&QLabel::setNum), actualPitch, _1));
    auv->sensors.depth->onUpdate.connect(boost::bind(static_cast<void (QLabel::*)(double)>(&QLabel::setNum), actualDepth, _1));

    auv->autopilots.bearing->onUpdate.connect(boost::bind(&QDoubleSpinBox::setValue, bearingTarget, _1));
    auv->autopilots.pitch->onUpdate.connect(boost::bind(&QDoubleSpinBox::setValue, pitchTarget, _1));
    auv->autopilots.depth->onUpdate.connect(boost::bind(&QDoubleSpinBox::setValue, depthTarget, _1));

    auv->autopilots.bearing->enabled->onUpdate.connect(boost::bind(&QCheckBox::setChecked, bearingEnabled, _1));
    auv->autopilots.pitch->enabled->onUpdate.connect(boost::bind(&QCheckBox::setChecked, pitchEnabled, _1));
    auv->autopilots.depth->enabled->onUpdate.connect(boost::bind(&QCheckBox::setChecked, depthEnabled, _1));

    connect(this->bearingTarget, SIGNAL(valueChanged(double)), this, SLOT(bearingAutopilotUpdated()));
    connect(this->bearingEnabled, SIGNAL(clicked()), this, SLOT(bearingAutopilotUpdated()));

    connect(this->pitchTarget, SIGNAL(valueChanged(double)), this, SLOT(pitchAutopilotUpdated()));
    connect(this->pitchEnabled, SIGNAL(clicked()), this, SLOT(pitchAutopilotUpdated()));

    connect(this->depthTarget, SIGNAL(valueChanged(double)), this, SLOT(depthAutopilotUpdated()));
    connect(this->depthEnabled, SIGNAL(clicked()), this, SLOT(depthAutopilotUpdated()));

}

void MotorControls::initialise(){
    m_actions->registerDockView(this, Qt::LeftDockWidgetArea);
}

void MotorControls::bearingAutopilotUpdated(){
    m_auv->autopilots.bearing->set(bearingTarget->value());
    m_auv->autopilots.bearing->enabled->set(bearingEnabled->checkState());
}

void MotorControls::pitchAutopilotUpdated(){
    m_auv->autopilots.pitch->set(pitchTarget->value());
    m_auv->autopilots.pitch->enabled->set(pitchEnabled->checkState());
}

void MotorControls::depthAutopilotUpdated(){
    m_auv->autopilots.depth->set(depthTarget->value());
    m_auv->autopilots.depth->enabled->set(depthEnabled->checkState());
}

