#include "motorcontrols.h"
#include "motorcontrol/ui_motorcontrols.h"

#include <QLabel>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>

#include <gui/core/model/model.h>

using namespace cauv;
using namespace cauv::gui;

MotorBurstController::MotorBurstController(boost::shared_ptr<NumericNode> motor, int8_t speed):
        m_speed(speed), m_motor(motor){}

void MotorBurstController::burst(){
    m_motor->set(m_speed);
}

void MotorBurstController::stop() {
    m_motor->set(0);
}

/*

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
    target->connect(target, SIGNAL(editingFinished()), this, SLOT(targetEditingFinished()));
}

void AutopilotController::onEnabledUpdate(bool enabled){
    m_enabled->blockSignals(true);
    m_enabled->setChecked(enabled);
    m_enabled->blockSignals(false);
}

void AutopilotController::onTargetUpdate(float target){
    m_target->blockSignals(true);
    if(!m_target->hasFocus())
        m_target->setValue(target);
    m_target->blockSignals(false);
}

void AutopilotController::onActualUpdate(float actual){
    m_actual->blockSignals(true);
    m_actual->setNum(actual);
    m_actual->blockSignals(false);
}

void AutopilotController::targetEditingFinished(){
    m_target->clearFocus();
}

void AutopilotController::updateTarget(double value) {
    m_autopilot->set(value);
}

void AutopilotController::updateState(bool value) {
    m_autopilot->enabled->set(value);
}

*/


MotorControls::MotorControls() :
        m_motorsCount(0), ui(new Ui::MotorControls())
{
    ui->setupUi(this);

    m_docks[this] = Qt::LeftDockWidgetArea;
}

void MotorControls::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node){
    CauvBasicPlugin::initialise(auv, node);

    boost::shared_ptr<GroupingNode> group = auv->findOrCreate<GroupingNode>("motors");

    // for new motors
    group->connect(group.get(), SIGNAL(nodeAdded(boost::shared_ptr<NodeBase>)), this, SLOT(addMotor(boost::shared_ptr<NodeBase>)));


    /*
    // autopilot controls screen
    int count = 0;
    foreach(AUV->autopilots i, auv->autopilots){
        // set up ui
        QLabel * label = new QLabel(QString::fromStdString(i.second->getName()));
        ui->autopilotControlsLayout->addWidget(label, count, 0, 1, 1, Qt::AlignCenter);

        QDoubleSpinBox * target = new QDoubleSpinBox();
        target->setWrapping(true);
        target->setButtonSymbols(QAbstractSpinBox::PlusMinus);
        target->setMaximum(i.second->getMax());
        target->setMinimum(i.second->getMin());
        target->setSuffix(QString::fromStdString(i.second->getUnits()));
        target->setSingleStep((i.second->getMax() - i.second->getMin())/360.0); // 360 is a arbitary value
                                                                             // just chosen to because its
                                                                             // nice for degrees
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
*/

    // motor controls screen
    int count = 0;
    foreach(boost::shared_ptr<NumericNode> motor, auv->findOrCreate<GroupingNode>("motors")->getChildrenOfType<NumericNode>()){
        std::string forward = "Forward";
        std::string backward = "Back";

/*
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
*/


    }
}

MotorControls::~MotorControls(){
    delete ui;
}


void MotorControls::addMotor(boost::shared_ptr<NodeBase> node) {
    boost::shared_ptr<NumericNode> motor = node->to<NumericNode>();

    std::string forward = "Forward";
    std::string backward = "Back";

    QPushButton * backButton = new QPushButton(QString::fromStdString(backward));
    boost::shared_ptr<MotorBurstController> back = boost::make_shared<MotorBurstController>(motor, -127);
    m_burst_controllers.push_back(back);
    backButton->connect(backButton, SIGNAL(pressed()), back.get(), SLOT(burst()));
    backButton->connect(backButton, SIGNAL(released()), back.get(), SLOT(stop()));
    ui->motorControlsLayout->addWidget(backButton, ++m_motorsCount, 0, 1, 1, Qt::AlignCenter);

    QLabel * label = new QLabel(QString::fromStdString(motor->nodeName(false)));
    label->setAlignment(Qt::AlignHCenter);
    ui->motorControlsLayout->addWidget(label, m_motorsCount, 1, 1, 1, Qt::AlignCenter);

    QPushButton * forwardButton = new QPushButton(QString::fromStdString(forward));
    boost::shared_ptr<MotorBurstController> fwd = boost::make_shared<MotorBurstController>(motor, 127);
    m_burst_controllers.push_back(fwd);
    forwardButton->connect(forwardButton, SIGNAL(pressed()), fwd.get(), SLOT(burst()));
    forwardButton->connect(forwardButton, SIGNAL(released()), fwd.get(), SLOT(stop()));
    ui->motorControlsLayout->addWidget(forwardButton, m_motorsCount, 2, 1, 1, Qt::AlignCenter);
}

void MotorControls::addAutopilot(boost::shared_ptr<NodeBase> node){
    boost::shared_ptr<NumericNode> ap = node->to<NumericNode>();

    // set up ui
    QLabel * label = new QLabel(QString::fromStdString(ap->nodeName(false)));
    ui->autopilotControlsLayout->addWidget(label, m_autopilotsCount, 0, 1, 1, Qt::AlignCenter);

    QDoubleSpinBox * target = new QDoubleSpinBox();
    target->setWrapping(true);
    target->setButtonSymbols(QAbstractSpinBox::PlusMinus);
    target->setMaximum(ap->getMax());
    target->setMinimum(i.second->getMin());
    target->setSuffix(QString::fromStdString(i.second->getUnits()));
    target->setSingleStep((i.second->getMax() - i.second->getMin())/360.0); // 360 is a arbitary value
                                                                         // just chosen to because its
                                                                         // nice for degrees
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

const QString MotorControls::name() const{
    return QString("Navigation");
}

const QList<QString> MotorControls::getGroups() const{
    QList<QString> groups;
    groups.push_back(QString("gui"));
    groups.push_back(QString("control"));
    return groups;
}

Q_EXPORT_PLUGIN2(cauv_motorsplugin, MotorControls)
