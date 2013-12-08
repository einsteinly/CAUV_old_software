#pragma once

#include <string>
#include <linux/can.h>

#include <boost/thread/thread.hpp>
#include <boost/asio/basic_datagram_socket.hpp>

#include <cauv_control/MotorDemand.h>
#include <utility/observable.h>

namespace cauv {

typedef cauv_control::MotorDemand MotorDemand;

MotorDemand& operator+=(MotorDemand& l, MotorDemand const& r);


struct CANObserver {
    virtual void onPressure(float fore, float aft) = 0;
};

class CANGate : public Observable<CANObserver> {
public:
    CANGate(const std::string& ifname);
    ~CANGate();

    void start();
    void setMotorState(const MotorDemand& state);

private:
    int socket_fd;
    float fore_pressure;
    float aft_pressure;
    boost::thread m_readThread;

    void notifyPressure(float fore, float aft);
    void read_loop();
};

}//namespace cauv
