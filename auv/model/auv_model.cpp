#include "auv_model.h"


void AUV::Motor::set(int8_t data) {
    if (data >= 127)
        data = 127;

    if (data < -127)
        data = -127;

    DataStream<int8_t>::set(data);
}

const MotorID::e& AUV::Motor::getID() {
    return m_id;
}






