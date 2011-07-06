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

    // for new motors    
    boost::shared_ptr<GroupingNode> motors = auv->findOrCreate<GroupingNode>("motors");
    motors->connect(motors.get(), SIGNAL(nodeAdded(boost::shared_ptr<NodeBase>)), this, SLOT(addMotor(boost::shared_ptr<NodeBase>)));

    // new autopilots
    boost::shared_ptr<GroupingNode> autopilots = auv->findOrCreate<GroupingNode>("autopilots");
    autopilots->connect(autopilots.get(), SIGNAL(nodeAdded(boost::shared_ptr<NodeBase>)), this, SLOT(addAutopilot(boost::shared_ptr<NodeBase>)));
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
    boost::shared_ptr<NumericNode> targetNode = node->findOrCreate<NumericNode>("target");
    boost::shared_ptr<NumericNode> enabledNode = node->findOrCreate<NumericNode>("enabled");
    //boost::shared_ptr<NumericNode> actualNode = node->findOrCreate<NumericNode>("actual");

    // set up ui
    QLabel * label = new QLabel(QString::fromStdString(node->nodeName(false)));
    ui->autopilotControlsLayout->addWidget(label, m_autopilotsCount, 0, 1, 1, Qt::AlignCenter);

    QDoubleSpinBox * target = new QDoubleSpinBox();
    target->setWrapping(targetNode->getWraps());
    target->setButtonSymbols(QAbstractSpinBox::PlusMinus);
    //if(targetNode->isMaxSet())
    //    target->setMaximum(boost::apply_visitor(to_float(), targetNode->getMax()));
    //if(targetNode->isMinSet())
    //    target->setMinimum(boost::apply_visitor(to_float(), targetNode->getMin()));
    target->setSuffix(QString::fromStdString(targetNode->getUnits()));

    /*if(targetNode->isMaxSet() && targetNode->isMinSet())
        target->setSingleStep((boost::apply_visitor(to_float(), targetNode->getMax()) - boost::apply_visitor(to_float(), targetNode->getMin()))/360.0); // 360 is a arbitary value
                                                                         // just chosen to because its
                                                                         // nice for degrees
    */
    ui->autopilotControlsLayout->addWidget(target, m_autopilotsCount, 1, 1, 1, Qt::AlignCenter);

    QCheckBox * enabled = new QCheckBox("State");
    ui->autopilotControlsLayout->addWidget(enabled, m_autopilotsCount, 2, 1, 1, Qt::AlignCenter);

    //QLabel * actual = new QLabel("Actual");
    //actual->setMinimumSize(QSize(60, 0));
    //actual->setMaximumSize(QSize(60, 16777215));
    //ui->autopilotControlsLayout->addWidget(actual, m_autopilotsCount, 3, 1, 1, Qt::AlignCenter);

    // controller for the GUI view
    //m_autopilot_controllers.push_back(boost::make_shared<AutopilotController>(enabled, target, actual, i.second));

    // sets
    // these have to be Qt::DirectConnections but I'm not sure why...
    connect(enabled, SIGNAL(toggled(bool)), enabledNode.get(), SLOT(setBool(bool)), Qt::DirectConnection);
    connect(target, SIGNAL(valueChanged(double)), this, SLOT(updateTarget(double)), Qt::DirectConnection);
    connect(target, SIGNAL(editingFinished()), this, SLOT(targetEditingFinished()), Qt::DirectConnection);

    // updates
    connect(enabledNode.get(), SIGNAL(onUpdate(bool)), enabled, SLOT(setChecked(bool)));
    connect(targetNode.get(), SIGNAL(onUpdate(double)), target, SLOT(setValue(double)));
    //connect(actualNode.get(), SIGNAL(onUpdate(double)), actual, SLOT(setNum(double)));

    enabledNode->set(false);

    m_autopilotsCount++;
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
