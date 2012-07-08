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

#include "auv_model.h"

using namespace cauv;


AUV::AUV()
{
    motors[MotorID::Prop] = boost::make_shared<Motor > (MotorID::Prop, "Prop");
    motors[MotorID::HBow] = boost::make_shared<Motor > (MotorID::HBow, "H Bow");
    motors[MotorID::VBow] = boost::make_shared<Motor > (MotorID::VBow, "V Bow");
    motors[MotorID::HStern] = boost::make_shared<Motor > (MotorID::HStern, "H Stern");
    motors[MotorID::VStern] = boost::make_shared<Motor > (MotorID::VStern, "V Stern");

    autopilots["bearing"] = boost::make_shared< Autopilot<float> >("Bearing AP", this->sensors.orientation->yaw, 0, 0, 360, true, "°");
    autopilots["depth"] = boost::make_shared< Autopilot<float> >("Depth AP", this->sensors.depth, 0, 0, 10, false, "m");
    autopilots["pitch"] = boost::make_shared< Autopilot<float> >("Pitch AP", this->sensors.orientation->pitch, 0, 0, 360, true, "°");

    logs[DebugType::Info] = boost::make_shared< DataStream<std::string> >("Info");
    logs[DebugType::Debug] = boost::make_shared< DataStream<std::string> >("Debug");
    logs[DebugType::Warning] = boost::make_shared< DataStream<std::string> >("Warning");
    logs[DebugType::Error] = boost::make_shared< DataStream<std::string> >("Error");

    debug_level = boost::make_shared<MutableDataStream<int32_t> >("Debug Level");

    cameras[CameraID::Forward] = boost::make_shared< Camera > ("Forward Camera");
    cameras[CameraID::Down] = boost::make_shared< Camera > ("Downward Camera");
    cameras[CameraID::Up] = boost::make_shared< Camera > ("Upward Camera");
    cameras[CameraID::Sonar] = boost::make_shared< Sonar > ("Sonar");
}

const MotorID::e& AUV::Motor::getID() {
    return m_id;
}







