#include "auv_model.h"

using namespace cauv;


AUV::AUV(){
    motors[MotorID::Prop] = boost::make_shared<Motor > (MotorID::Prop, "Prop");
    motors[MotorID::HBow] = boost::make_shared<Motor > (MotorID::HBow, "H Bow");
    motors[MotorID::VBow] = boost::make_shared<Motor > (MotorID::VBow, "V Bow");
    motors[MotorID::HStern] = boost::make_shared<Motor > (MotorID::HStern, "H Stern");
    motors[MotorID::VStern] = boost::make_shared<Motor > (MotorID::VStern, "V Stern");

    autopilots["bearing"] = boost::make_shared< Autopilot<float> >("Bearing", 0);
    autopilots["depth"] = boost::make_shared< Autopilot<float> >("Depth", 0);
    autopilots["pitch"] = boost::make_shared< Autopilot<float> >("Pitch", 0);

    logs[DebugType::Debug] = boost::make_shared< DataStream<std::string> >("Debug");
    logs[DebugType::Trace] = boost::make_shared< DataStream<std::string> >("Trace");
    logs[DebugType::Error] = boost::make_shared< DataStream<std::string> >("Error");

    debug_level = boost::make_shared<MutableDataStream<int32_t> >("Debug Level");

    cameras[CameraID::Forward] = boost::make_shared< Camera > ("Forward Camera");
    cameras[CameraID::Down] = boost::make_shared< Camera > ("Downward Camera");
    cameras[CameraID::Sonar] = boost::make_shared< Sonar > ("Sonar");
}

const MotorID::e& AUV::Motor::getID() {
    return m_id;
}







