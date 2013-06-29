#ifndef _CAUV_CAN_GATE_H_
#define _CAUV_CAN_GATE_H_

#include <string>
#include <linux/can.h>

#include <boost/thread/thread.hpp>
#include <boost/asio/basic_datagram_socket.hpp>

#include <utility/observable.h>
#include <generated/message_observers.h>
#include <generated/types/MotorDemand.h>

namespace cauv {

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

#endif//_CAUV_CAN_GATE_H_
