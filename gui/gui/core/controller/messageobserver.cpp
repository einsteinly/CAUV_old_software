#include "messageobserver.h"

#include <vector>

#include <debug/cauv_debug.h>

#include <generated/types/ControlGroup.h>
#include <generated/types/GuiGroup.h>
#include <generated/types/DebugGroup.h>
#include <generated/types/SonarctlGroup.h>
#include <generated/types/SonaroutGroup.h>
#include <generated/types/TelemetryGroup.h>
#include <generated/types/ImageGroup.h>
#include <generated/types/PressureGroup.h>

#include <gui/core/model/model.h>

using namespace cauv;
using namespace cauv::gui;

GuiMessageObserver::GuiMessageObserver(boost::shared_ptr<AUV> auv):
        m_auv(auv){
    qRegisterMetaType<boost::shared_ptr<const Message> >("boost::shared_ptr<const Message>");
}

GuiMessageObserver::~GuiMessageObserver() {
    debug() << "~GuiMessageObserver()";
}

void GuiMessageObserver::onMotorStateMessage(MotorStateMessage_ptr message) {
    m_auv->findOrCreate<GroupingNode>("motors")->findOrCreate<NumericNode>(message->motorId())->update(message->speed());
}

void GuiMessageObserver::onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> autopilot = autopilots->findOrCreate<GroupingNode>(AutopilotID::Bearing);
    autopilot->findOrCreate<NumericNode>("target")->update(message->target());
    autopilot->findOrCreate<NumericNode>("enabled")->update(message->enabled());
}

void GuiMessageObserver::onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(AutopilotID::Bearing);

    ap->findOrCreate<NumericNode>("Kp")->update(message->Kp());
    ap->findOrCreate<NumericNode>("Ki")->update(message->Ki());
    ap->findOrCreate<NumericNode>("Kd")->update(message->Kd());
    ap->findOrCreate<NumericNode>("scale")->update(message->scale());
    ap->findOrCreate<NumericNode>("aP")->update(message->Ad());
    ap->findOrCreate<NumericNode>("aI")->update(message->Ai());
    ap->findOrCreate<NumericNode>("aD")->update(message->Ad());
    ap->findOrCreate<NumericNode>("thr")->update(message->thr());
    ap->findOrCreate<NumericNode>("maxError")->update(message->maxError());
}

void GuiMessageObserver::onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> autopilot = autopilots->findOrCreate<GroupingNode>(AutopilotID::Depth);
    autopilot->findOrCreate<NumericNode>("target")->update(message->target());
    autopilot->findOrCreate<NumericNode>("enabled")->update(message->enabled());
}

void GuiMessageObserver::onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(AutopilotID::Depth);

    ap->findOrCreate<NumericNode>("Kp")->update(message->Kp());
    ap->findOrCreate<NumericNode>("Ki")->update(message->Ki());
    ap->findOrCreate<NumericNode>("Kd")->update(message->Kd());
    ap->findOrCreate<NumericNode>("scale")->update(message->scale());
    ap->findOrCreate<NumericNode>("aP")->update(message->Ad());
    ap->findOrCreate<NumericNode>("aI")->update(message->Ai());
    ap->findOrCreate<NumericNode>("aD")->update(message->Ad());
    ap->findOrCreate<NumericNode>("thr")->update(message->thr());
    ap->findOrCreate<NumericNode>("maxError")->update(message->maxError());
}

void GuiMessageObserver::onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> autopilot = autopilots->findOrCreate<GroupingNode>(AutopilotID::Pitch);
    autopilot->findOrCreate<NumericNode>("target")->update(message->target());
    autopilot->findOrCreate<NumericNode>("enabled")->update(message->enabled());
}

void GuiMessageObserver::onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(AutopilotID::Pitch);

    ap->findOrCreate<NumericNode>("Kp")->update(message->Kp());
    ap->findOrCreate<NumericNode>("Ki")->update(message->Ki());
    ap->findOrCreate<NumericNode>("Kd")->update(message->Kd());
    ap->findOrCreate<NumericNode>("scale")->update(message->scale());
    ap->findOrCreate<NumericNode>("aP")->update(message->Ad());
    ap->findOrCreate<NumericNode>("aI")->update(message->Ai());
    ap->findOrCreate<NumericNode>("aD")->update(message->Ad());
    ap->findOrCreate<NumericNode>("thr")->update(message->thr());
    ap->findOrCreate<NumericNode>("maxError")->update(message->maxError());
}


void GuiMessageObserver::onDepthCalibrationMessage(DepthCalibrationMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("sensors")->findOrCreate<GroupingNode>("calibration");
    group->findOrCreate<NumericNode>("aftMultiplier")->update(message->aftMultiplier());
    group->findOrCreate<NumericNode>("aftOffset")->update(message->aftOffset());
    group->findOrCreate<NumericNode>("foreMultiplier")->update(message->foreMultiplier());
    group->findOrCreate<NumericNode>("foreOffset")->update(message->foreOffset());
}

void GuiMessageObserver::onDebugLevelMessage(DebugLevelMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("debug");
    group->findOrCreate<NumericNode>("level")->update(message->level());
}

void GuiMessageObserver::onImageMessage(ImageMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("image");
    boost::shared_ptr<Image> shared_image= boost::make_shared<Image>(message->image());
    group->findOrCreate<ImageNode>(message->source())->update(shared_image);
}

void GuiMessageObserver::onSonarControlMessage(SonarControlMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("image");
    boost::shared_ptr<ImageNode> sonar = group->findOrCreate<ImageNode>(CameraID::Sonar);
    sonar->findOrCreate<NumericNode>("direction")->update(message->direction());
    sonar->findOrCreate<NumericNode>("width")->update(message->width());
    sonar->findOrCreate<NumericNode>("gain")->update(message->gain());
    sonar->findOrCreate<NumericNode>("range")->update(message->range());
    sonar->findOrCreate<NumericNode>("rangeRes")->update(message->rangeRes());
    sonar->findOrCreate<NumericNode>("angularRes")->update(message->angularRes());
}

void GuiMessageObserver::onTelemetryMessage(TelemetryMessage_ptr message){
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("telemtry");
    group->findOrCreate<NumericNode>("depth")->update(message->depth());
    group->findOrCreate<FloatYPRNode>("orientation")->update(message->orientation());

    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    autopilots->findOrCreate<GroupingNode>("bearing")->findOrCreate<NumericNode>("actual")->update(message->orientation().yaw);
    autopilots->findOrCreate<GroupingNode>("pitch")->findOrCreate<NumericNode>("actual")->update(message->orientation().pitch);
    autopilots->findOrCreate<GroupingNode>("depth")->findOrCreate<NumericNode>("actual")->update(message->depth());
}

void GuiMessageObserver::onLocationMessage(LocationMessage_ptr){
    //boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("telemetry")->findOrCreate<GroupingNode>("location");

    //group->findOrCreate<NumericNode>("speed")->update(message->aft());
    //group->findOrCreate<NumericNode>("")->update(message->aft());
    //m_auv->sensors.speed->update(m->speed());
    //m_auv->sensors.location->update(*(m.get()));
}

void GuiMessageObserver::onPressureMessage(PressureMessage_ptr message){
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("sensors")->findOrCreate<GroupingNode>("pressure");
    group->findOrCreate<NumericNode>("fore")->update(message->fore());
    group->findOrCreate<NumericNode>("aft")->update(message->aft());
}

void GuiMessageObserver::onScriptResponseMessage(ScriptResponseMessage_ptr){
//    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("console");
//    group->findOrCreate<StringNode>("response")->update(message->response());
}

void GuiMessageObserver::onBatteryUseMessage(BatteryUseMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("telemetry")->findOrCreate<GroupingNode>("battery");
    group->findOrCreate<NumericNode>("current")->update(message->estimate_current());
    group->findOrCreate<NumericNode>("total")->update(message->estimate_total());
    group->findOrCreate<NumericNode>("remaining")->update(message->fraction_remaining() * 100.f);
}

void GuiMessageObserver::onProcessStatusMessage(ProcessStatusMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("processes");
    boost::shared_ptr<GroupingNode> process = group->findOrCreate<GroupingNode>(message->process());

    process->findOrCreate<NumericNode>("cpu")->update(message->cpu());
    process->findOrCreate<NumericNode>("mem")->update(message->mem());
    process->findOrCreate<NumericNode>("threads")->update(message->threads());
    process->findOrCreate<StringNode>("status")->update(message->status());
}

void GuiMessageObserver::onControllerStateMessage(ControllerStateMessage_ptr message){

    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(message->contoller());
    boost::shared_ptr<GroupingNode> state = ap->findOrCreate<GroupingNode>("state");

    state->findOrCreate<NumericNode>("Kp")->update(message->kp());
    state->findOrCreate<NumericNode>("Ki")->update(message->ki());
    state->findOrCreate<NumericNode>("Kd")->update(message->kd());
    boost::shared_ptr<GroupingNode> demands = state->findOrCreate<GroupingNode>("demands");
    demands->findOrCreate<NumericNode>(MotorID::Prop)->update(message->demand().prop);
    demands->findOrCreate<NumericNode>(MotorID::HBow)->update(message->demand().hbow);
    demands->findOrCreate<NumericNode>(MotorID::HStern)->update(message->demand().hstern);
    demands->findOrCreate<NumericNode>(MotorID::VBow)->update(message->demand().vbow);
    demands->findOrCreate<NumericNode>(MotorID::VStern)->update(message->demand().vstern);
    state->findOrCreate<NumericNode>("error")->update(message->error());
    state->findOrCreate<NumericNode>("derror")->update(message->derror());
    state->findOrCreate<NumericNode>("ierror")->update(message->ierror());
    state->findOrCreate<NumericNode>("mv")->update(message->mv());
}

