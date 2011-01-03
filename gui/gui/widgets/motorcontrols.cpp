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

}

void MotorControls::initialise(){
    m_actions->registerDockView(this, Qt::LeftDockWidgetArea);
}

