/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

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

#include "../model/model.h"
#include "../model/nodes/numericnode.h"
#include "../model/nodes/compoundnodes.h"
#include "../model/nodes/groupingnode.h"
#include "../model/nodes/stringnode.h"
#include "../model/nodes/imagenode.h"

using namespace cauv;
using namespace cauv::gui;

GuiMessageObserver::GuiMessageObserver(boost::shared_ptr<Vehicle> auv):
        m_auv(auv){
    qRegisterMetaType<boost::shared_ptr<const Message> >("boost::shared_ptr<const Message>");
}

GuiMessageObserver::~GuiMessageObserver() {
    debug(2) << "~GuiMessageObserver()";
}

void GuiMessageObserver::onMotorStateMessage(MotorStateMessage_ptr message) {
    m_auv->findOrCreate<GroupingNode>("motors")->findOrCreate<TypedNumericNode<int8_t> >(message->motorId())->update(message->speed());
}

void GuiMessageObserver::onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> autopilot = autopilots->findOrCreate<GroupingNode>(Controller::Bearing);
    autopilot->findOrCreate<TypedNumericNode<float> >("target")->update(message->target());
    autopilot->findOrCreate<TypedNumericNode<bool> >("enabled")->update(message->enabled());
}

void GuiMessageObserver::onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(Controller::Bearing);

    ap->findOrCreate<TypedNumericNode<float> >("Kp")->update(message->Kp());
    ap->findOrCreate<TypedNumericNode<float> >("Ki")->update(message->Ki());
    ap->findOrCreate<TypedNumericNode<float> >("Kd")->update(message->Kd());
    ap->findOrCreate<TypedNumericNode<float> >("scale")->update(message->scale());
    ap->findOrCreate<TypedNumericNode<float> >("aP")->update(message->Ad());
    ap->findOrCreate<TypedNumericNode<float> >("aI")->update(message->Ai());
    ap->findOrCreate<TypedNumericNode<float> >("aD")->update(message->Ad());
    ap->findOrCreate<TypedNumericNode<float> >("thr")->update(message->thr());
    ap->findOrCreate<TypedNumericNode<float> >("maxError")->update(message->maxError());
}

void GuiMessageObserver::onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> autopilot = autopilots->findOrCreate<GroupingNode>(Controller::Depth);
    autopilot->findOrCreate<TypedNumericNode<float> >("target")->update(message->target());
    autopilot->findOrCreate<TypedNumericNode<bool> >("enabled")->update(message->enabled());
}

void GuiMessageObserver::onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(Controller::Depth);

    ap->findOrCreate<TypedNumericNode<float> >("Kp")->update(message->Kp());
    ap->findOrCreate<TypedNumericNode<float> >("Ki")->update(message->Ki());
    ap->findOrCreate<TypedNumericNode<float> >("Kd")->update(message->Kd());
    ap->findOrCreate<TypedNumericNode<float> >("scale")->update(message->scale());
    ap->findOrCreate<TypedNumericNode<float> >("aP")->update(message->Ad());
    ap->findOrCreate<TypedNumericNode<float> >("aI")->update(message->Ai());
    ap->findOrCreate<TypedNumericNode<float> >("aD")->update(message->Ad());
    ap->findOrCreate<TypedNumericNode<float> >("thr")->update(message->thr());
    ap->findOrCreate<TypedNumericNode<float> >("maxError")->update(message->maxError());
}

void GuiMessageObserver::onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> autopilot = autopilots->findOrCreate<GroupingNode>(Controller::Pitch);
    autopilot->findOrCreate<TypedNumericNode<float> >("target")->update(message->target());
    autopilot->findOrCreate<TypedNumericNode<bool> >("enabled")->update(message->enabled());
}

void GuiMessageObserver::onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(Controller::Pitch);

    ap->findOrCreate<TypedNumericNode<float> >("Kp")->update(message->Kp());
    ap->findOrCreate<TypedNumericNode<float> >("Ki")->update(message->Ki());
    ap->findOrCreate<TypedNumericNode<float> >("Kd")->update(message->Kd());
    ap->findOrCreate<TypedNumericNode<float> >("scale")->update(message->scale());
    ap->findOrCreate<TypedNumericNode<float> >("aP")->update(message->Ad());
    ap->findOrCreate<TypedNumericNode<float> >("aI")->update(message->Ai());
    ap->findOrCreate<TypedNumericNode<float> >("aD")->update(message->Ad());
    ap->findOrCreate<TypedNumericNode<float> >("thr")->update(message->thr());
    ap->findOrCreate<TypedNumericNode<float> >("maxError")->update(message->maxError());
}


void GuiMessageObserver::onDepthCalibrationMessage(DepthCalibrationMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("sensors")->findOrCreate<GroupingNode>("calibration");
    group->findOrCreate<TypedNumericNode<float> >("aftMultiplier")->update(message->aftMultiplier());
    group->findOrCreate<TypedNumericNode<float> >("aftOffset")->update(message->aftOffset());
    group->findOrCreate<TypedNumericNode<float> >("foreMultiplier")->update(message->foreMultiplier());
    group->findOrCreate<TypedNumericNode<float> >("foreOffset")->update(message->foreOffset());
}

void GuiMessageObserver::onDebugLevelMessage(DebugLevelMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("debug");
    group->findOrCreate<TypedNumericNode<int> >("level")->update(message->level());
}

void GuiMessageObserver::onImageMessage(ImageMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("image");
    boost::shared_ptr<Image> shared_image= boost::make_shared<Image>(message->image());
    group->findOrCreate<ImageNode>(message->source())->update(shared_image);
}

void GuiMessageObserver::onSonarControlMessage(SonarControlMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("image");
    boost::shared_ptr<ImageNode> sonar = group->findOrCreate<ImageNode>(CameraID::Sonar);
    sonar->findOrCreate<TypedNumericNode<uint16_t> >("direction")->update(message->direction());
    sonar->findOrCreate<TypedNumericNode<uint16_t> >("width")->update(message->width());
    sonar->findOrCreate<TypedNumericNode<uint8_t> >("gain")->update(message->gain());
    sonar->findOrCreate<TypedNumericNode<uint32_t> >("range")->update(message->range());
    sonar->findOrCreate<TypedNumericNode<uint32_t> >("rangeRes")->update(message->rangeRes());
    sonar->findOrCreate<TypedNumericNode<uint32_t> >("angularRes")->update(message->angularRes());
}

void GuiMessageObserver::onTelemetryMessage(TelemetryMessage_ptr message){
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("telemtry");
    group->findOrCreate<TypedNumericNode<float> >("depth")->update(message->depth());
    group->findOrCreate<FloatYPRNode>("orientation")->update(message->orientation());

    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    autopilots->findOrCreate<GroupingNode>("bearing")->findOrCreate<TypedNumericNode<float> >("actual")->update(message->orientation().yaw);
    autopilots->findOrCreate<GroupingNode>("pitch")->findOrCreate<TypedNumericNode<float> >("actual")->update(message->orientation().pitch);
    autopilots->findOrCreate<GroupingNode>("depth")->findOrCreate<TypedNumericNode<float> >("actual")->update(message->depth());
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
    group->findOrCreate<TypedNumericNode<uint16_t> >("fore")->update(message->fore());
    group->findOrCreate<TypedNumericNode<uint16_t> >("aft")->update(message->aft());
}

void GuiMessageObserver::onScriptResponseMessage(ScriptResponseMessage_ptr){
//    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("console");
//    group->findOrCreate<StringNode>("response")->update(message->response());
}

void GuiMessageObserver::onBatteryUseMessage(BatteryUseMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("telemetry")->findOrCreate<GroupingNode>("battery");
    group->findOrCreate<TypedNumericNode<float> >("current")->update(message->estimate_current());
    group->findOrCreate<TypedNumericNode<float> >("total")->update(message->estimate_total());
    group->findOrCreate<TypedNumericNode<float> >("remaining")->update(message->fraction_remaining() * 100.f);
}

void GuiMessageObserver::onProcessStatusMessage(ProcessStatusMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("processes");
    boost::shared_ptr<GroupingNode> process = group->findOrCreate<GroupingNode>(message->process());

    process->findOrCreate<TypedNumericNode<float> >("cpu")->update(message->cpu());
    process->findOrCreate<TypedNumericNode<float> >("mem")->update(message->mem());
    process->findOrCreate<TypedNumericNode<uint32_t> >("threads")->update(message->threads());
    process->findOrCreate<StringNode>("status")->update(message->status());
}

void GuiMessageObserver::onControllerStateMessage(ControllerStateMessage_ptr message){

    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(message->contoller());
    boost::shared_ptr<GroupingNode> state = ap->findOrCreate<GroupingNode>("state");


    boost::shared_ptr<GroupingNode> demands = state->findOrCreate<GroupingNode>("demands");
    demands->findOrCreate<TypedNumericNode<float> >(MotorID::Prop)->update(message->demand().prop);
    demands->findOrCreate<TypedNumericNode<float> >(MotorID::HBow)->update(message->demand().hbow);
    demands->findOrCreate<TypedNumericNode<float> >(MotorID::HStern)->update(message->demand().hstern);
    demands->findOrCreate<TypedNumericNode<float> >(MotorID::VBow)->update(message->demand().vbow);
    demands->findOrCreate<TypedNumericNode<float> >(MotorID::VStern)->update(message->demand().vstern);
    state->findOrCreate<TypedNumericNode<float> >("Kp")->update(message->kp());
    state->findOrCreate<TypedNumericNode<float> >("Ki")->update(message->ki());
    state->findOrCreate<TypedNumericNode<float> >("Kd")->update(message->kd());
    state->findOrCreate<TypedNumericNode<float> >("error")->update(message->error());
    state->findOrCreate<TypedNumericNode<float> >("derror")->update(message->derror());
    state->findOrCreate<TypedNumericNode<float> >("ierror")->update(message->ierror());
    state->findOrCreate<TypedNumericNode<float> >("mv")->update(message->mv());
}

