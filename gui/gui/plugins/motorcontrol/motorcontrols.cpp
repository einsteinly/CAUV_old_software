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




AutopilotController::AutopilotController(QCheckBox *enabled, QDoubleSpinBox *target, QLabel * actual, boost::shared_ptr<NodeBase> node):
        m_autopilot(node),
        m_enabled(enabled),
        m_target(target),
        m_actual(actual) {


    boost::shared_ptr<NumericNode> targetNode = node->findOrCreate<NumericNode>("target");
    boost::shared_ptr<NumericNode> enabledNode = node->findOrCreate<NumericNode>("enabled");
    boost::shared_ptr<NumericNode> actualNode = node->findOrCreate<NumericNode>("actual");

    // sets
    connect(target, SIGNAL(editingFinished()), this, SLOT(targetEditingFinished()));
    // these have to be Qt::DirectConnections but I'm not sure why...
    connect(target, SIGNAL(valueChanged(double)), targetNode.get(), SLOT(set(double)), Qt::DirectConnection);
    connect(enabled, SIGNAL(toggled(bool)), enabledNode.get(), SLOT(set(bool)), Qt::DirectConnection);

    // updates
    connect(enabledNode.get(), SIGNAL(onUpdate(bool)), enabled, SLOT(setChecked(bool)));
    connect(targetNode.get(), SIGNAL(onUpdate(double)), this, SLOT(updateTarget(double)));
    connect(actualNode.get(), SIGNAL(onUpdate(double)), actual, SLOT(setNum(double)));

    // target params (max/ min / units etc...)
    targetNode->connect(targetNode.get(), SIGNAL(paramsUpdated()), this, SLOT(configureTarget()));

    configureTarget();
}

void AutopilotController::updateTarget(double target){
    m_target->blockSignals(true);
    if(!m_target->hasFocus())
        m_target->setValue(target);
    m_target->blockSignals(false);
}

void AutopilotController::targetEditingFinished(){
    m_target->clearFocus();
}


void AutopilotController::configureTarget(){
    boost::shared_ptr<NumericNode> targetNode = m_autopilot->findOrCreate<NumericNode>("target");

    // set wrapping
    m_target->setWrapping(targetNode->getWraps());

    // max / min values
    if(targetNode->isMaxSet()){
        numeric_variant_t max = targetNode->getMax();
        m_target->setMaximum(boost::apply_visitor(to_float(), max));
    }
    if(targetNode->isMinSet()) {
        numeric_variant_t min = targetNode->getMin();
        m_target->setMinimum(boost::apply_visitor(to_float(), min));
    }

    // units
    m_target->setSuffix(QString::fromStdString(targetNode->getUnits()));

    // and a sensible step size
    if(targetNode->isMaxSet() && targetNode->isMinSet()){
        numeric_variant_t min = targetNode->getMin();
        numeric_variant_t max = targetNode->getMax();
        m_target->setSingleStep((boost::apply_visitor(to_float(), max) - boost::apply_visitor(to_float(), min))/360.0); // 360 is a arbitary value
                                                                         // just chosen to because its
                                                                         // nice for degrees
    }
}




MotorControls::MotorControls() :
        m_motorsCount(0), m_autopilotsCount(0), ui(new Ui::MotorControls()), m_burst_controllers()
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

    // set up ui
    QLabel * label = new QLabel(QString::fromStdString(node->nodeName(false)));
    ui->autopilotControlsLayout->addWidget(label, m_autopilotsCount, 0, 1, 1, Qt::AlignCenter);

    QDoubleSpinBox * target = new QDoubleSpinBox();

    ui->autopilotControlsLayout->addWidget(target, m_autopilotsCount, 1, 1, 1, Qt::AlignCenter);

    QCheckBox * enabled = new QCheckBox("State");
    ui->autopilotControlsLayout->addWidget(enabled, m_autopilotsCount, 2, 1, 1, Qt::AlignCenter);

    QLabel * actual = new QLabel("Actual");
    actual->setMinimumSize(QSize(60, 0));
    actual->setMaximumSize(QSize(60, 16777215));
    ui->autopilotControlsLayout->addWidget(actual, m_autopilotsCount, 3, 1, 1, Qt::AlignCenter);

    boost::shared_ptr<AutopilotController> controller = boost::make_shared<AutopilotController>(enabled, target, actual, node);
    m_autopilot_controllers.push_back(controller);

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
