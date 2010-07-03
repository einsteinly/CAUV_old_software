#include "control.h"

#include <iostream>
#include <sstream>
#include <deque>
#include <stdint.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <common/messages.h>
#include <debug/cauv_debug.h>

#include <module/module.h>

#include "xsens_imu.h"

using namespace std;

void sendAlive(boost::shared_ptr<MCBModule> mcb)
{
    debug() << "Starting alive message thread";
    while(true)
    {
        boost::shared_ptr<AliveMessage> m = boost::make_shared<AliveMessage>();
        mcb->send(m);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
}


class DebugXsensObserver : public XsensObserver
{
    public:
        DebugXsensObserver(unsigned int level = 1) : m_level(level)
        {
        }
        virtual void onTelemetry(const floatYPR& attitude)
        {
            debug(m_level) << (std::string)(MakeString() << fixed << setprecision(1) << attitude ); 
        }

    protected:
        unsigned int m_level;
};

int operator-(TimeStamp const& l, TimeStamp const& r)
{
    int secs_delta = l.secs - r.secs;
    int msecs_delta = (l.musecs - r.musecs) / 1000;
    return 1000 * secs_delta + msecs_delta;
}

struct PIDControl
{
    Controller::e controlee;
    double target;
    double Kp,Ki,Kd,scale;
    double integral, previous_derror, previous_mv;
    std::deque< std::pair<TimeStamp, double> > previous_errors;
    TimeStamp previous_time;
    bool is_angle;
    int retain_samples_msecs;

    PIDControl(Controller::e controlee=Controller::NumValues)
        : controlee(controlee), target(0), Kp(1), Ki(1), Kd(1), scale(1),
          integral(0), is_angle(false), retain_samples_msecs(200)
    {
        previous_time.secs = 0;
    }
    
    void reset()
    {
        integral = 0;
        previous_errors.clear();
        previous_derror = 0;
        previous_mv = 0;
        previous_time.secs = 0;
    }

    static double mod(double const& d, double const& base)
    {
        if(d > 0) {
            return d - base * std::floor(d / base);
        }else{
            return d + base * std::floor(-d / base);
        }
    }

    virtual double getErrorAngle(double const& target, double const& current)
    {
        double diff = mod(target - current, 360);
        if(diff >  180) diff -= 360;
        if(diff <= -180) diff += 360;
        return diff;
    }

    virtual double getError(double const& target, double const& current)
    {
        return target - current;
    }

    double smoothedDerivative()
    {
        int n_derivatives = 0;
        double derivative_sum = 0;

        if(!previous_errors.size()){
            warning() << "no derivative samples available";        
            return 0.0;
        }

        for(int i = 0;i < int(previous_errors.size())-1; i++){
            int dt_msecs = (previous_errors[i+1].first - previous_errors[i].first);
            if(dt_msecs != 0){
                // TODO: multiply this by 
                derivative_sum += (previous_errors[i+1].second - previous_errors[i].second) / dt_msecs;
                n_derivatives++;
            }else{
                warning() << "controller update frequency < 1ms";
            }
        }
        if(!n_derivatives){
            warning() << "no derivative samples used";
            return 0.0;
        }
        return derivative_sum / n_derivatives;
    }

    double getMV(double current)
    {
        double error;
        if(is_angle)
            error = getErrorAngle(target, current);
        else
            error = getError(target, current);

        if (previous_time.secs == 0) {
            previous_time = now();
            previous_errors.push_back(std::make_pair(previous_time, error));
            return 0;
        }

        TimeStamp tnow = now();
        previous_errors.push_back(std::make_pair(tnow, error));
        if(tnow - previous_errors.front().first > retain_samples_msecs)
            previous_errors.pop_front(); 

        double dt = tnow - previous_time; // dt is milliseconds
        previous_time = tnow;

        integral += error*dt;
        double de = smoothedDerivative();
        previous_derror = de;
        previous_mv =  scale * (Kp * error + Ki * integral + Kd * de);

        return previous_mv;
    }

    boost::shared_ptr<ControllerStateMessage> stateMsg()
    {
        if(previous_errors.size())
            return boost::make_shared<ControllerStateMessage>(
                controlee, previous_mv, previous_errors.back().second, previous_derror, integral, MotorDemand()
            );
        else
            return boost::make_shared<ControllerStateMessage>(
                controlee, previous_mv, 0, previous_derror, integral, MotorDemand()
            ); 
    }
};


MotorDemand& operator+=(MotorDemand& l, MotorDemand const& r){
    l.prop += r.prop;
    l.hbow += r.hbow;
    l.hstern += r.hstern;
    l.vbow += r.vbow;
    l.vstern += r.vstern;
    return l;
}

class StateObserver : public MessageObserver, public XsensObserver
{
    public:
        StateObserver(boost::shared_ptr<ReconnectingSpreadMailbox> mb)
            : m_mb(mb)
        {
        }
        
        virtual void onTelemetry(const floatYPR& attitude)
        {
            m_orientation = attitude;
        }
        virtual void onStateRequestMessage(StateRequestMessage_ptr)
        {
            m_mb->sendMessage(boost::make_shared<StateMessage>(m_orientation), SAFE_MESS);
        }

    protected:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mb;
        
        floatYPR m_orientation;
};

using namespace Controller;

class ControlLoops : public MessageObserver, public XsensObserver
{
    public:
        ControlLoops(boost::shared_ptr<ReconnectingSpreadMailbox> mb)
            : prop_value(0), hbow_value(0), vbow_value(0),
              hstern_value(0), vstern_value(0), m_max_motor_delta(255/*12*/),
              m_motor_updates_per_second(5), m_mb(mb)
        {
            const MotorMap def = {5, -5, 127, -127};
            prop_map = def;
            hbow_map = def;
            vbow_map = def;
            hstern_map = def;
            vstern_map = def;
            //. tmp test stuff:
            //MotorRampRateMessage_ptr mrrm = boost::make_shared<MotorRampRateMessage>(255, 5);
            //onMotorRampRateMessage(mrrm);
            //SetMotorMapMessage_ptr smmm = boost::make_shared<SetMotorMapMessage>(MotorID::Prop, def);
            //onSetMotorMapMessage(smmm);
        }
        ~ControlLoops()
        {
            stop();
        }

        void set_mcb(boost::shared_ptr<MCBModule> mcb)
        {
            m_mcb = mcb;
        }

        void start()
        {
            stop();
            m_motorControlLoopThread = boost::thread(&ControlLoops::motorControlLoop, this);
        }
        void stop()
        {
            if (m_motorControlLoopThread.get_id() != boost::thread::id()) {
                m_motorControlLoopThread.interrupt();    
                m_motorControlLoopThread.join();    
            }
        }

        virtual void onTelemetry(const floatYPR& attitude)
        {
            if (m_controlenabled[Bearing]) {
                float mv = m_controllers[Bearing].getMV(attitude.yaw);
                debug(2) << "Bearing Control: MV = " << mv;
                m_demand[Bearing].hbow = mv;
                m_demand[Bearing].hstern = -mv;
               
                boost::shared_ptr<ControllerStateMessage> msg = m_controllers[Bearing].stateMsg();
                msg->demand(m_demand[Bearing]);
                m_mb->sendMessage(msg, SAFE_MESS);
            }
            
            if (m_controlenabled[Pitch]) {
                float mv = m_controllers[Pitch].getMV(attitude.pitch);
                debug(2) << "Pitch Control: MV = " << mv;
                m_demand[Pitch].vbow = -mv;
                m_demand[Pitch].vstern = mv;
                
                boost::shared_ptr<ControllerStateMessage> msg = m_controllers[Pitch].stateMsg();
                msg->demand(m_demand[Pitch]);
                m_mb->sendMessage(msg, SAFE_MESS);
            }
        }

        virtual void onPressureMessage(PressureMessage_ptr m)
        {
            if (m_controlenabled[Depth] && m_depthCalibration){
                float fore_depth_calibrated = m_depthCalibration->foreOffset() +
                                              m_depthCalibration->foreMultiplier()
                                              * m->fore();
                float aft_depth_calibrated = m_depthCalibration->aftOffset() +
                                             m_depthCalibration->aftMultiplier() * m->aft();
                float depth = 0.5 * (fore_depth_calibrated + aft_depth_calibrated);

                float mv = m_controllers[Depth].getMV(depth);
                
                m_demand[Depth].vbow = mv;
                m_demand[Depth].vstern = mv;

                debug(2) << "depth: fwd=" << fore_depth_calibrated
                         << "aft=" << aft_depth_calibrated
                         << "mean=" << depth << ", mv =" << mv;
                
                boost::shared_ptr<ControllerStateMessage> msg = m_controllers[Depth].stateMsg();
                msg->demand(m_demand[Depth]);
                m_mb->sendMessage(msg, SAFE_MESS);
            }
        }
        virtual void onDepthCalibrationMessage(DepthCalibrationMessage_ptr m)
        {
            m_depthCalibration = m;
        }
        
    
    protected:
        boost::shared_ptr<MCBModule> m_mcb;
        DepthCalibrationMessage_ptr m_depthCalibration;
       
        
        virtual void onResetMCBMessage(ResetMCBMessage_ptr m)
        {
            debug() << "Resetting MCB";
            m_mcb->send(m);
        }


        virtual void onMotorMessage(MotorMessage_ptr m)
        {
            debug(2) << "Set manual motor demand based on motor message:" << *m;
            
            switch(m->motorId())
            {
                case MotorID::Prop: m_demand[ManualOverride].prop = m->speed(); break;
                case MotorID::HBow: m_demand[ManualOverride].hbow = m->speed(); break;
                case MotorID::HStern: m_demand[ManualOverride].hstern = m->speed(); break;
                case MotorID::VBow: m_demand[ManualOverride].vbow = m->speed(); break;
                case MotorID::VStern: m_demand[ManualOverride].vstern = m->speed(); break;
                case MotorID::NumValues: break;
            }
        }

        
        bool m_controlenabled[Controller::NumValues];
        PIDControl m_controllers[Controller::NumValues];
        MotorDemand m_demand[Controller::NumValues];
        
        virtual void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                m_controlenabled[Bearing] = true;
                m_controllers[Bearing].reset();
                m_controllers[Bearing].target = m->target();
            }
            else {
                m_controlenabled[Bearing] = false;
            }
        }
        virtual void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr m)
        {
            m_controllers[Bearing].Kp = m->Kp();
            m_controllers[Bearing].Ki = m->Ki();
            m_controllers[Bearing].Kd = m->Kd();
            m_controllers[Bearing].scale = m->scale();
            m_controllers[Bearing].reset();
        }

        virtual void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                m_controlenabled[Pitch] = true;
                m_controllers[Pitch].reset();
                m_controllers[Pitch].target = m->target();
            }
            else {
                m_controlenabled[Pitch] = false;
            }
        }
        virtual void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr m)
        {
            m_controllers[Pitch].Kp = m->Kp();
            m_controllers[Pitch].Ki = m->Ki();
            m_controllers[Pitch].Kd = m->Kd();
            m_controllers[Pitch].scale = m->scale();
            m_controllers[Pitch].reset();
        }

        virtual void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr m)
        {
            if (m->enabled()) {
                if (!m_depthCalibration) {
                    warning() << "depth control will not be effective until calibration factors are set";
                }
                m_controlenabled[Depth] = true;
                m_controllers[Depth].reset();
                m_controllers[Depth].target = m->target();
            }
            else {
                m_controlenabled[Depth] = false;
            }
        }
        virtual void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr m)
        {
            m_controllers[Depth].Kp = m->Kp();
            m_controllers[Depth].Ki = m->Ki();
            m_controllers[Depth].Kd = m->Kd();
            m_controllers[Depth].scale = m->scale();
            m_controllers[Depth].reset();
        }

        virtual void onMotorRampRateMessage(MotorRampRateMessage_ptr m)
        {
            // MCB currently handles ramping: don't limit it
            //if(m->maxDelta() >= 127)
            //    warning() << "maximum motor delta set exceptionally high:" << m->maxDelta();
            m_max_motor_delta = m->maxDelta();
            m_motor_updates_per_second = m->updatesPerSecond();
            debug() << "Set motor ramp limit:" << m_max_motor_delta
                    << ", update rate:" << m_motor_updates_per_second;
        }

        virtual void onSetMotorMapMessage(SetMotorMapMessage_ptr m)
        {
            if(m->mapping().zeroPlus >= m->mapping().maxPlus ||
               m->mapping().zeroMinus <= m->mapping().maxMinus)
            {
                error() << "Invalid motor mapping for" << m->motor() << ":" << m->mapping();
                return;
            }
                
            if(m->mapping().zeroPlus < 0 ||
               m->mapping().zeroMinus > 0 ||
               m->mapping().maxPlus < 100 ||
               m->mapping().maxPlus > 150 ||
               m->mapping().maxMinus > -100 ||
               m->mapping().maxMinus < -150)
            {
                warning() << "Unusual motor mapping for" << m->motor() << ":" << m->mapping();
            }

            switch(m->motor())
            {
                default:
                case MotorID::Prop: prop_map = m->mapping(); break;
                case MotorID::HBow: hbow_map = m->mapping(); break;
                case MotorID::VBow: vbow_map = m->mapping(); break;
                case MotorID::HStern: hstern_map = m->mapping(); break;
                case MotorID::VStern: vstern_map = m->mapping(); break;
            }
            debug() << "Set motor mapping:" << m->motor() << ":" << m->mapping();
            
            double test_values[] = {-200, -150, -100, -50, -4, 0, 3, 50, 100, 150, 200};
            for(int i = 0; i < 11; i++)
                debug() << "new map example: " << test_values[i] << "->" << motorMap(test_values[i], m->motor());
        }

    private:
        boost::thread m_motorControlLoopThread;
        
        void motorControlLoop()
        {
            debug() << "Control loop thread started";
            try {
                const MotorDemand no_demand = {0,0,0,0,0};
                for(int i = 0; i < Controller::NumValues; i++)
                {
                    m_controlenabled[i] = false;
                    m_controllers[i].reset();
                    m_controllers[i].controlee = (Controller::e)i;
                    m_demand[i] = no_demand;
                    
                    boost::shared_ptr<ControllerStateMessage> msg = m_controllers[i].stateMsg();
                    msg->demand(m_demand[i]);
                    m_mb->sendMessage(msg, SAFE_MESS);
                }
                m_controllers[Controller::Bearing].is_angle = true;
                m_controlenabled[Controller::ManualOverride] = true;
                
                while(true)
                {
                    boost::this_thread::interruption_point();
                    updateMotorControl();
                    if(m_motor_updates_per_second)
                        msleep(1000/m_motor_updates_per_second);
                }

            } catch (boost::thread_interrupted&) {
                debug() << "Control loop thread interrupted";
            }
            debug() << "Control loop thread exiting";
        }

        int motorMap(float const& demand_value, MotorID::e mid)
        {
            MotorMap m = {0, 0, 127, -127};
            switch(mid)
            {
                default:
                case MotorID::Prop: m = prop_map; break;
                case MotorID::HBow: m = hbow_map; break;
                case MotorID::VBow: m = vbow_map; break;
                case MotorID::HStern: m = hstern_map; break;
                case MotorID::VStern: m = vstern_map; break;
            }
            /*
             *        -127                           0                            127
             * demand:  |---------------------------|0|----------------------------| 
             * output:     |---------------|         0            |----------------|
             *             ^               ^                      ^                ^
             *             maxMinus     zeroMinus               zeroPlus        maxPlus
             */
            if(demand_value < -127)
                return m.maxMinus;
            else if(demand_value < 0)
                //eg:      -30     + (   -30      -   -120    ) *     (-50) / 127 = -65
                return m.zeroMinus + (m.zeroMinus - m.maxMinus) * demand_value / 127;
            else if(demand_value == 0)
                return 0;
            else if(demand_value <= 127)
                //eg:     50      + (  127     -     50    ) *    80        / 127 = 86
                return m.zeroPlus + (m.maxPlus - m.zeroPlus) * demand_value / 127;
            else
                return m.maxPlus;
        }
        
        void updateMotorControl()
        {
            // TODO: This may need a lock. Maybe not, reading/writing ints is atomic. 
            MotorDemand total_demand = {0,0,0,0,0};
            for(int i = 0; i < Controller::NumValues; i++)
                if(m_controlenabled[i])
                    total_demand += m_demand[i];
            
            int new_prop_value = clamp(-127, motorMap(total_demand.prop, MotorID::Prop), 127);
            int new_hbow_value = clamp(-127, motorMap(total_demand.hbow, MotorID::HBow), 127);
            int new_vbow_value = clamp(-127, motorMap(total_demand.vbow, MotorID::VBow), 127);
            int new_hstern_value = clamp(-127, motorMap(total_demand.hstern, MotorID::HStern), 127);
            int new_vstern_value = clamp(-127, motorMap(total_demand.vstern, MotorID::VStern), 127);
            
            if(m_mcb) {
                sendWithMaxDelta(MotorID::Prop, prop_value, new_prop_value, m_max_motor_delta);
                sendWithMaxDelta(MotorID::HBow, hbow_value, new_hbow_value, m_max_motor_delta);
                sendWithMaxDelta(MotorID::VBow, vbow_value, new_vbow_value, m_max_motor_delta);
                sendWithMaxDelta(MotorID::HStern, hstern_value, new_hstern_value, m_max_motor_delta);
                sendWithMaxDelta(MotorID::VStern, vstern_value, new_vstern_value, m_max_motor_delta);
            }
            
            m_mb->sendMessage(boost::make_shared<MotorStateMessage>(total_demand), SAFE_MESS);
        }

        void sendWithMaxDelta(MotorID::e mid, int& oldvalue, int newvalue, unsigned maxDelta)
        {
            if(unsigned(abs(newvalue - oldvalue)) <= maxDelta)
                sendIfNew(mid, oldvalue, newvalue);
            else if(newvalue < oldvalue)
                sendIfNew(mid, oldvalue, oldvalue - maxDelta);
            else
                sendIfNew(mid, oldvalue, oldvalue + maxDelta);
        }

        void sendIfNew(MotorID::e mid, int& oldvalue, int newvalue)
        {
            if(newvalue != oldvalue) {
                oldvalue = newvalue;
                m_mcb->send(boost::make_shared<MotorMessage>(mid, newvalue));
            }
        }

        int prop_value;
        int hbow_value;
        int vbow_value;
        int hstern_value;
        int vstern_value;

        MotorMap prop_map;
        MotorMap hbow_map;
        MotorMap vbow_map;
        MotorMap hstern_map;
        MotorMap vstern_map;

        unsigned m_max_motor_delta;
        unsigned m_motor_updates_per_second;

        boost::shared_ptr<ReconnectingSpreadMailbox> m_mb;
};

class TelemetryBroadcaster : public MessageObserver, public XsensObserver
{
    public:
        TelemetryBroadcaster(boost::shared_ptr<ReconnectingSpreadMailbox> mb) : m_mb(mb)
        {
        }

        ~TelemetryBroadcaster()
        {
            stop();
        }

        void start()
        {
            stop();
            m_telemetryThread = boost::thread(&TelemetryBroadcaster::sendTelemetry, this);
        }
        void stop()
        {
            if (m_telemetryThread.get_id() != boost::thread::id()) {
                m_telemetryThread.interrupt();    
                m_telemetryThread.join();    
            }
        }


        virtual void onTelemetry(const floatYPR& attitude)
        {
            m_orientation = attitude;
        }

        virtual void onPressureMessage(PressureMessage_ptr m)
        {
            if (m_depthCalibration){
                float fore_depth_calibrated = m_depthCalibration->foreOffset() +
                                              m_depthCalibration->foreMultiplier()
                                              * m->fore();
                float aft_depth_calibrated = m_depthCalibration->aftOffset() +
                                             m_depthCalibration->aftMultiplier() * m->aft();
                m_depth = 0.5 * (fore_depth_calibrated + aft_depth_calibrated);
            }
        }
        virtual void onDepthCalibrationMessage(DepthCalibrationMessage_ptr m)
        {
            m_depthCalibration = m;
        }
    protected:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mb;

        DepthCalibrationMessage_ptr m_depthCalibration;
    
        floatYPR m_orientation;
        float m_depth;
        
        boost::thread m_telemetryThread;
        void sendTelemetry()
        {
            try {
                debug() << "Send telemetry thread started";
                while(true)
                {
                    m_mb->sendMessage(boost::make_shared<TelemetryMessage>(m_orientation, m_depth), SAFE_MESS);
                    msleep(100);
                }
            } catch (boost::thread_interrupted&) {
                debug() << "Send telemetry thread interrupted";
            }
            debug() << "Send telemetry thread ending";
        }
};

class MCBForwardingObserver : public MessageObserver
{
    public:
        MCBForwardingObserver(boost::shared_ptr<ReconnectingSpreadMailbox> mb) : m_mb(mb)
        {
        }

        virtual void onPressureMessage(PressureMessage_ptr m)
        {
            debug(5) << "MCBForwardingObserver: Forwarding pressure message:" << *m;
            m_mb->sendMessage(m, UNRELIABLE_MESS);
        }
        virtual void onDebugMessage(DebugMessage_ptr m)
        {
            debug(5) << "MCBForwardingObserver: Forwarding debug message:" << *m;
            m_mb->sendMessage(m, SAFE_MESS);
        }
    protected:
        boost::shared_ptr<ReconnectingSpreadMailbox> m_mb;
};

class NotRootException : public std::exception
{
    public:
        virtual const char* what() const throw() {
            return "Need to be running as root";
        }
};

ControlNode::ControlNode() : CauvNode("Control")
{
    joinGroup("control");
    addMessageObserver(boost::make_shared<DebugMessageObserver>(1));

    m_controlLoops = boost::make_shared<ControlLoops>(mailbox());
    addMessageObserver(m_controlLoops);
    
    m_stateObserver = boost::make_shared<StateObserver>(mailbox());
    addMessageObserver(m_stateObserver);
    
    m_telemetryBroadcaster = boost::make_shared<TelemetryBroadcaster>(mailbox());
    addMessageObserver(m_telemetryBroadcaster);
}
ControlNode::~ControlNode()
{
    if (m_aliveThread.get_id() != boost::thread::id()) {
        m_aliveThread.interrupt();
        m_aliveThread.join();
    }
}

void ControlNode::setMCB(int id)
{
    m_mcb.reset();
    // start up the MCB module
    try {
        m_mcb = boost::make_shared<MCBModule>(id);
        info() << "MCB Connected";
    }
    catch (FTDIException& e)
    {
        error() << "Cannot connect to MCB: " << e.what();
        m_mcb.reset();
        if (e.errCode() == -8) {
            throw NotRootException();
        }
    }
}

void ControlNode::setXsens(int id)
{
    // start up the Xsens IMU
    try {
        m_xsens = boost::make_shared<XsensIMU>(id);
        info() << "Xsens Connected";
        
        CmtOutputMode om = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
        CmtOutputSettings os = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT | CMT_OUTPUTSETTINGS_CALIBMODE_ACCMAG;

        CmtMatrix m;
        m.m_data[0][0] =  1.0; m.m_data[0][1] =  0.0; m.m_data[0][2] =  0.0; 
        m.m_data[1][0] =  0.0; m.m_data[1][1] =  1.0; m.m_data[1][2] =  0.0; 
        m.m_data[2][0] =  0.0; m.m_data[2][1] =  0.0; m.m_data[2][2] =  1.0; 

        m_xsens->setObjectAlignmentMatrix(m);
        m_xsens->configure(om, os);
        
        info() << "XSens Configured";
    } catch (XsensException& e) {
        error() << "Cannot connect to Xsens: " << e.what();
        m_xsens.reset();
    }
}

void ControlNode::addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos)
{
    namespace po = boost::program_options;
    CauvNode::addOptions(desc, pos);
    
    desc.add_options()
        ("xsens,x", po::value<int>()->default_value(0), "USB device id of the Xsens")
        ("mcb,m", po::value<int>()->default_value(0), "FTDI device id of the MCB")
        ("depth-offset,o", po::value<float>()->default_value(0), "Depth calibration offset")
        ("depth-scale,s", po::value<float>()->default_value(0), "Depth calibration scale");
}
int ControlNode::useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc)
{
    namespace po = boost::program_options;
    int ret = CauvNode::useOptionsMap(vm, desc);
    if (ret != 0) return ret;

    if (vm.count("xsens")) {
        setXsens(vm["xsens"].as<int>());
    }
    if (vm.count("mcb")) {
        setMCB(vm["mcb"].as<int>());
    }
    if (vm.count("depth-offset") && vm.count("depth-scale")) {
        float offset = vm["depth-offset"].as<float>();
        float scale = vm["depth-scale"].as<float>();
        m_controlLoops->onDepthCalibrationMessage(boost::make_shared<DepthCalibrationMessage>(offset,scale,offset,scale));
    }
    else if (vm.count("depth-offset") || vm.count("depth-scale")) {
        warning() << "Need both offset and depth for calibration; ignoring calibration input";   
    }
    
    return 0;
}


void ControlNode::onRun()
{
    CauvNode::onRun();
   
    if (m_mcb) {
        m_controlLoops->set_mcb(m_mcb);
        
        m_aliveThread = boost::thread(sendAlive, m_mcb);
        
        m_mcb->addObserver(boost::make_shared<DebugMessageObserver>(2));
        m_mcb->addObserver(boost::make_shared<MCBForwardingObserver>(mailbox()));
        m_mcb->addObserver(m_telemetryBroadcaster);
        m_mcb->addObserver(m_controlLoops);
        
        m_mcb->start();
    }
    else {
        warning() << "MCB not connected. No MCB comms available, so no motor control.";
    }

    if (m_xsens) {
        m_xsens->addObserver(boost::make_shared<DebugXsensObserver>(5));
        m_xsens->addObserver(m_telemetryBroadcaster);
        m_xsens->addObserver(m_controlLoops);
        m_xsens->addObserver(m_stateObserver);

        m_xsens->start();
    }
    else {
        warning() << "Xsens not connected. Telemetry not available.";
    }

    m_controlLoops->start();
}

static ControlNode* node;

void cleanup()
{
    info() << "Cleaning up..." << endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done." << endl;
}

void interrupt(int sig)
{
    cout << endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    signal(SIGINT, interrupt);
    
    try {
        node = new ControlNode();
    
        int ret = node->parseOptions(argc, argv);
        if(ret != 0) return ret;
        
        node->run();
    }
    catch (NotRootException& e) {
        error() << e.what();
    }
    
    cleanup();
    return 0;
}
