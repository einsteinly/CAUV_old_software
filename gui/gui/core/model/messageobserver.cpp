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

#include <QMetaType>

#include <debug/cauv_debug.h>

#include <generated/types/ControlGroup.h>
#include <generated/types/GuiGroup.h>
#include <generated/types/DebugGroup.h>
#include <generated/types/SonarctlGroup.h>
#include <generated/types/SonaroutGroup.h>
#include <generated/types/TelemetryGroup.h>
#include <generated/types/ImageGroup.h>
#include <generated/types/PressureGroup.h>

#include "model/model.h"
#include "model/nodes/numericnode.h"
#include "model/nodes/groupingnode.h"
#include "model/nodes/stringnode.h"
#include "model/nodes/imagenode.h"

using namespace cauv;
using namespace cauv::gui;

GuiMessageObserver::GuiMessageObserver(boost::shared_ptr<Vehicle> auv):
        m_auv(auv){
    qRegisterMetaType<boost::shared_ptr<const Message> >("boost::shared_ptr<const Message>");
    qRegisterMetaType<BoundedFloat>("BoundedFloat");
}

GuiMessageObserver::~GuiMessageObserver() {
    debug(2) << "~GuiMessageObserver()";
}



DefaultGuiMessageObserver::DefaultGuiMessageObserver(boost::shared_ptr<Vehicle> auv):
        GuiMessageObserver(auv){
}

void DefaultGuiMessageObserver::onMotorStateMessage(MotorStateMessage_ptr message) {
    m_auv->findOrCreate<GroupingNode>("motors")->findOrCreate<NumericNode<int> >(message->motorId())->update(message->speed());
}

void DefaultGuiMessageObserver::onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<NumericNode<bool> > autopilot = autopilots->findOrCreate<NumericNode<bool> >(Controller::Bearing);
    autopilot->update(message->enabled());
    autopilot->findOrCreate<NumericNode<float> >("target")->update(message->target());
}

void DefaultGuiMessageObserver::onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(Controller::Bearing);

    ap->findOrCreate<NumericNode<float> >("Kp")->update(message->Kp());
    ap->findOrCreate<NumericNode<float> >("Ki")->update(message->Ki());
    ap->findOrCreate<NumericNode<float> >("Kd")->update(message->Kd());
    ap->findOrCreate<NumericNode<float> >("scale")->update(message->scale());
    ap->findOrCreate<NumericNode<float> >("aP")->update(message->Ad());
    ap->findOrCreate<NumericNode<float> >("aI")->update(message->Ai());
    ap->findOrCreate<NumericNode<float> >("aD")->update(message->Ad());
    ap->findOrCreate<NumericNode<float> >("thr")->update(message->thr());
    ap->findOrCreate<NumericNode<float> >("maxError")->update(message->maxError());
}

void DefaultGuiMessageObserver::onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<NumericNode<bool> > autopilot = autopilots->findOrCreate<NumericNode<bool> >(Controller::Depth);
    autopilot->update(message->enabled());
    autopilot->findOrCreate<NumericNode<float> >("target")->update(message->target());
}

void DefaultGuiMessageObserver::onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(Controller::Depth);

    ap->findOrCreate<NumericNode<float> >("Kp")->update(message->Kp());
    ap->findOrCreate<NumericNode<float> >("Ki")->update(message->Ki());
    ap->findOrCreate<NumericNode<float> >("Kd")->update(message->Kd());
    ap->findOrCreate<NumericNode<float> >("scale")->update(message->scale());
    ap->findOrCreate<NumericNode<float> >("aP")->update(message->Ad());
    ap->findOrCreate<NumericNode<float> >("aI")->update(message->Ai());
    ap->findOrCreate<NumericNode<float> >("aD")->update(message->Ad());
    ap->findOrCreate<NumericNode<float> >("thr")->update(message->thr());
    ap->findOrCreate<NumericNode<float> >("maxError")->update(message->maxError());
}

void DefaultGuiMessageObserver::onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<NumericNode<bool> > autopilot = autopilots->findOrCreate<NumericNode<bool> >(Controller::Pitch);
    autopilot->update(message->enabled());
    autopilot->findOrCreate<NumericNode<float> >("target")->update(message->target());
}

void DefaultGuiMessageObserver::onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr message) {
    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(Controller::Pitch);

    ap->findOrCreate<NumericNode<float> >("Kp")->update(message->Kp());
    ap->findOrCreate<NumericNode<float> >("Ki")->update(message->Ki());
    ap->findOrCreate<NumericNode<float> >("Kd")->update(message->Kd());
    ap->findOrCreate<NumericNode<float> >("scale")->update(message->scale());
    ap->findOrCreate<NumericNode<float> >("aP")->update(message->Ad());
    ap->findOrCreate<NumericNode<float> >("aI")->update(message->Ai());
    ap->findOrCreate<NumericNode<float> >("aD")->update(message->Ad());
    ap->findOrCreate<NumericNode<float> >("thr")->update(message->thr());
    ap->findOrCreate<NumericNode<float> >("maxError")->update(message->maxError());
}


void DefaultGuiMessageObserver::onDepthCalibrationMessage(DepthCalibrationMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("sensors")->findOrCreate<GroupingNode>("calibration");
    group->findOrCreate<NumericNode<float> >("aftMultiplier")->update(message->aftMultiplier());
    group->findOrCreate<NumericNode<float> >("aftOffset")->update(message->aftOffset());
    group->findOrCreate<NumericNode<float> >("foreMultiplier")->update(message->foreMultiplier());
    group->findOrCreate<NumericNode<float> >("foreOffset")->update(message->foreOffset());
}

void DefaultGuiMessageObserver::onDebugLevelMessage(DebugLevelMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("debug");
    group->findOrCreate<NumericNode<int> >("level")->update(message->level());
}

void DefaultGuiMessageObserver::onImageMessage(ImageMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("image");
    boost::shared_ptr<Image> shared_image= boost::make_shared<Image>(message->image());
    group->findOrCreate<ImageNode>(message->source())->update(shared_image);
}

void DefaultGuiMessageObserver::onSonarControlMessage(SonarControlMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("image");
    boost::shared_ptr<ImageNode> sonar = group->findOrCreate<ImageNode>(CameraID::Sonar);
    sonar->findOrCreate<NumericNode<int> >("direction")->update(message->direction());
    sonar->findOrCreate<NumericNode<int> >("width")->update(message->width());
    sonar->findOrCreate<NumericNode<int> >("gain")->update(message->gain());
    sonar->findOrCreate<NumericNode<unsigned int> >("range")->update(message->range());
    sonar->findOrCreate<NumericNode<unsigned int> >("rangeRes")->update(message->rangeRes());
    sonar->findOrCreate<NumericNode<unsigned int> >("angularRes")->update(message->angularRes());
}

void DefaultGuiMessageObserver::onTelemetryMessage(TelemetryMessage_ptr message){
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("telemtry");
    group->findOrCreate<NumericNode<float> >("depth")->update(message->depth());
    //!!! group->findOrCreate<FloatYPRNode>("orientation")->update(message->orientation());

    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    autopilots->findOrCreate<GroupingNode>("bearing")->findOrCreate<NumericNode<float> >("actual")->update(message->orientation().yaw);
    autopilots->findOrCreate<GroupingNode>("pitch")->findOrCreate<NumericNode<float> >("actual")->update(message->orientation().pitch);
    autopilots->findOrCreate<GroupingNode>("depth")->findOrCreate<NumericNode<float> >("actual")->update(message->depth());
}

void DefaultGuiMessageObserver::onLocationMessage(LocationMessage_ptr){
    //boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("telemetry")->findOrCreate<GroupingNode>("location");

    //group->findOrCreate<NumericNode>("speed")->update(message->aft());
    //group->findOrCreate<NumericNode>("")->update(message->aft());
    //m_auv->sensors.speed->update(m->speed());
    //m_auv->sensors.location->update(*(m.get()));
}

void DefaultGuiMessageObserver::onPressureMessage(PressureMessage_ptr message){
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("sensors")->findOrCreate<GroupingNode>("pressure");
    group->findOrCreate<NumericNode<unsigned int> >("fore")->update(message->fore());
    group->findOrCreate<NumericNode<unsigned int> >("aft")->update(message->aft());
}

void DefaultGuiMessageObserver::onBatteryUseMessage(BatteryUseMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("telemetry")->findOrCreate<GroupingNode>("battery");
    group->findOrCreate<NumericNode<float> >("current")->update(message->estimate_current());
    group->findOrCreate<NumericNode<float> >("total")->update(message->estimate_total());
    group->findOrCreate<NumericNode<float> >("remaining")->update(message->fraction_remaining() * 100.f);
}

void DefaultGuiMessageObserver::onProcessStatusMessage(ProcessStatusMessage_ptr message) {
    boost::shared_ptr<GroupingNode> group = m_auv->findOrCreate<GroupingNode>("processes");
    boost::shared_ptr<GroupingNode> process = group->findOrCreate<GroupingNode>(message->process());

    process->findOrCreate<NumericNode<float> >("cpu")->update(message->cpu());
    process->findOrCreate<NumericNode<float> >("mem")->update(message->mem());
    process->findOrCreate<NumericNode<unsigned int> >("threads")->update(message->threads());
    process->findOrCreate<StringNode>("status")->update(message->status());
}

void DefaultGuiMessageObserver::onControllerStateMessage(ControllerStateMessage_ptr message){

    boost::shared_ptr<GroupingNode> autopilots = m_auv->findOrCreate<GroupingNode>("autopilots");
    boost::shared_ptr<GroupingNode> ap = autopilots->findOrCreate<GroupingNode>(message->contoller());
    boost::shared_ptr<GroupingNode> state = ap->findOrCreate<GroupingNode>("state");


    boost::shared_ptr<GroupingNode> demands = state->findOrCreate<GroupingNode>("demands");
    demands->findOrCreate<NumericNode<float> >(MotorID::Prop)->update(message->demand().prop);
    demands->findOrCreate<NumericNode<float> >(MotorID::HBow)->update(message->demand().hbow);
    demands->findOrCreate<NumericNode<float> >(MotorID::HStern)->update(message->demand().hstern);
    demands->findOrCreate<NumericNode<float> >(MotorID::VBow)->update(message->demand().vbow);
    demands->findOrCreate<NumericNode<float> >(MotorID::VStern)->update(message->demand().vstern);
    state->findOrCreate<NumericNode<float> >("Kp")->update(message->kp());
    state->findOrCreate<NumericNode<float> >("Ki")->update(message->ki());
    state->findOrCreate<NumericNode<float> >("Kd")->update(message->kd());
    state->findOrCreate<NumericNode<float> >("error")->update(message->error());
    state->findOrCreate<NumericNode<float> >("derror")->update(message->derror());
    state->findOrCreate<NumericNode<float> >("ierror")->update(message->ierror());
    state->findOrCreate<NumericNode<float> >("mv")->update(message->mv());
}
