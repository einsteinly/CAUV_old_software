#include "auv_model.h"

using namespace cauv;


AUV::AUV(){
    motors[MotorID::Prop] = boost::make_shared<Motor > (MotorID::Prop, "Prop");
    motors[MotorID::HBow] = boost::make_shared<Motor > (MotorID::HBow, "H Bow");
    motors[MotorID::VBow] = boost::make_shared<Motor > (MotorID::VBow, "V Bow");
    motors[MotorID::HStern] = boost::make_shared<Motor > (MotorID::HStern, "H Stern");
    motors[MotorID::VStern] = boost::make_shared<Motor > (MotorID::VStern, "V Stern");

    autopilots["bearing"] = boost::make_shared< Autopilot<float> >("Bearing", this->sensors.orientation_split->yaw, 0, 0, 360, "°");
    autopilots["depth"] = boost::make_shared< Autopilot<float> >("Depth", this->sensors.depth, 0, 0, 10, "m");
    autopilots["pitch"] = boost::make_shared< Autopilot<float> >("Pitch", this->sensors.orientation_split->pitch, 0, 0, 360, "°");

    logs[DebugType::Info] = boost::make_shared< DataStream<std::string> >("Info");
    logs[DebugType::Debug] = boost::make_shared< DataStream<std::string> >("Debug");
    logs[DebugType::Warning] = boost::make_shared< DataStream<std::string> >("Warning");
    logs[DebugType::Error] = boost::make_shared< DataStream<std::string> >("Error");

    debug_level = boost::make_shared<MutableDataStream<int32_t> >("Debug Level");

    cameras[CameraID::Forward] = boost::make_shared< Camera > ("Forward Camera");
    cameras[CameraID::Down] = boost::make_shared< Camera > ("Downward Camera");
    cameras[CameraID::Sonar] = boost::make_shared< Sonar > ("Sonar");
}

const MotorID::e& AUV::Motor::getID() {
    return m_id;
}







