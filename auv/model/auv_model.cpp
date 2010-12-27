#include "auv_model.h"

using namespace cauv;

std::ostream& operator <<(std::ostream &os,const depth_calibration_t &value)
{
      os << "fore (" << value.foreOffset << ", x" << value.foreMultiplier << ") aft (" << value.aftOffset << ", x" << value.afteMultiplier << ")";
      return os;
}


std::ostream& operator <<(std::ostream &os,const sonar_params_t &value)
{
      os << "( ar=" << value.angularRes << ", rr=" << value.radialRes << ", range=" << value.range<< ", dir=" << value.direction<< ", gain=" << value.gain << ", width=" << value.width << ")";
      return os;
}


std::ostream& operator <<(std::ostream &os,const autopilot_params_t &value)
{
      os << "(" << value.kP << ", " << value.kI << ", " << value.kD << ", " << value.scale << ")";
      return os;
}


std::ostream& operator <<(std::ostream &os,const int8_t &value)
{
    os << (int)value;
    return os;
}


const MotorID::e& AUV::Motor::getID() {
    return m_id;
}






