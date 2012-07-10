#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <utility/string.h>
#include <utility/rounding.h>
#include <utility/foreach.h>
#include <common/mailbox.h>
#include <debug/cauv_debug.h>

#include <generated/types/TimeStamp.h>
#include <generated/types/MotorDemand.h>
#include <generated/types/ControlGroup.h>
#include <generated/types/Can_ControlGroup.h>
#include <generated/types/Can_StatusGroup.h>
#include <generated/types/StateMessage.h>
#include <generated/types/ControllerStateMessage.h>
#include <generated/types/MotorStateMessage.h>
#include <generated/types/GraphableMessage.h>
#include <generated/types/TelemetryMessage.h>
#include <generated/types/DebugMessage.h>
#include <generated/types/RedHerringBatteryStatusMessage.h>
#include <generated/types/Controller.h>

#include "xsens_imu.h"
#include "sbg_imu.h"
#include "sim_imu.h"
#include "mcb.h"
#include "arm_mcb.h"
#include "pid.h"

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <common/cauv_node.h>

using namespace cauv;

class DebugIMUObserver : public IMUObserver
{
    public:
        DebugIMUObserver(unsigned int level = 1) : m_level(level)
        {
        }
        virtual void onTelemetry(const floatYPR& attitude)
        {
            debug(m_level) << (std::string)(MakeString() << std::fixed << std::setprecision(1) << attitude ); 
        }

    protected:
        unsigned int m_level;
};

MotorDemand& operator+=(MotorDemand& l, MotorDemand const& r){
    l.prop += r.prop;
    l.hbow += r.hbow;
    l.hstern += r.hstern;
    l.vbow += r.vbow;
    l.vstern += r.vstern;
    return l;
}

class ControlLoops : public MessageObserver, public IMUObserver, public MCBObserver
{
    public:
        ControlLoops(boost::shared_ptr<Mailbox> mb);
        ~ControlLoops();

        void set_mcb(boost::shared_ptr<MCB> mcb);

        void start();
        void stop();

        //from IMU
        virtual void onTelemetry(const floatYPR& attitude);

        //from MCB
        virtual void onDepth(float stern, float bow);

    protected:
        boost::shared_ptr<MCB> m_mcb;
        bool m_control_enabled[Controller::NumValues];
        PIDControl m_controllers[Controller::NumValues];
        MotorDemand m_demand[Controller::NumValues];
        
        //From messages
        virtual void onMotorMessage(MotorMessage_ptr m);
        virtual void onMotorRampRateMessage(MotorRampRateMessage_ptr m);
        virtual void onSetMotorMapMessage(SetMotorMapMessage_ptr m);

        virtual void onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr m);
        virtual void onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr m);
        virtual void onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr m);

        virtual void onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr m);
        virtual void onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr m);
        virtual void onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr m);

    private:
        boost::thread m_motorControlLoopThread;
        
        void motorControlLoop();
        int motorMap(float const& demand_value, MotorID mid);
        void updateMotorControl();


        int motor_values[MotorID::NumValues];

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

        boost::shared_ptr<Mailbox> m_mb;
};

ControlLoops::ControlLoops(boost::shared_ptr<Mailbox> mb)
: prop_value(0), hbow_value(0), vbow_value(0),
    hstern_value(0), vstern_value(0), m_max_motor_delta(255),
    m_motor_updates_per_second(5), m_mb(mb)
{
    const MotorMap def(5, -5, 127, -127);
    prop_map = def;
    hbow_map = def;
    vbow_map = def;
    hstern_map = def;
    vstern_map = def;
}

ControlLoops::~ControlLoops()
{
    stop();
}

void ControlLoops::set_mcb(boost::shared_ptr<MCB> mcb)
{
    m_mcb = mcb;
}

void ControlLoops::start()
{
    stop();
    m_motorControlLoopThread = boost::thread(&ControlLoops::motorControlLoop, this);
}

void ControlLoops::stop()
{
    if (m_motorControlLoopThread.get_id() != boost::thread::id()) {
        m_motorControlLoopThread.interrupt();    
        m_motorControlLoopThread.join();    
    }
}

void ControlLoops::onTelemetry(const floatYPR& attitude)
{
    if (m_control_enabled[Controller::Bearing]) {
        float mv = m_controllers[Controller::Bearing].getMV(attitude.yaw);
        debug(2) << "Bearing Control: MV = " << mv;
        m_demand[Controller::Bearing].hbow = mv;
        m_demand[Controller::Bearing].hstern = -mv;

        boost::shared_ptr<ControllerStateMessage> msg = m_controllers[Controller::Bearing].stateMsg();
        msg->demand(m_demand[Controller::Bearing]);
        m_mb->sendMessage(msg, RELIABLE_MSG);

        foreach(boost::shared_ptr<GraphableMessage> m, m_controllers[Controller::Bearing].extraStateMessages()){
            m->name("Bearing-" + m->name());
            m_mb->sendMessage(m, RELIABLE_MSG);
        }
    }

    if (m_control_enabled[Controller::Pitch]) {
        float mv = m_controllers[Controller::Pitch].getMV(attitude.pitch);
        debug(2) << "Pitch Control: MV = " << mv;
        m_demand[Controller::Pitch].vbow = -mv;
        m_demand[Controller::Pitch].vstern = mv;

        boost::shared_ptr<ControllerStateMessage> msg = m_controllers[Controller::Pitch].stateMsg();
        msg->demand(m_demand[Controller::Pitch]);
        m_mb->sendMessage(msg, RELIABLE_MSG);

        foreach(boost::shared_ptr<GraphableMessage> m, m_controllers[Controller::Bearing].extraStateMessages()){
            m->name("Pitch-" + m->name());
            m_mb->sendMessage(m, RELIABLE_MSG);
        }
    }
}

void ControlLoops::onDepth(float fore, float aft)
{
    if (m_control_enabled[Controller::Depth]) {
        float depth = (fore + aft) / 2;
        float mv = m_controllers[Controller::Depth].getMV(depth);

        m_demand[Controller::Depth].vbow = mv;
        m_demand[Controller::Depth].vstern = mv;

        debug(2) << "depth: fwd=" << fore
            << "aft=" << aft
            << "mean=" << depth << ", mv =" << mv;

        boost::shared_ptr<ControllerStateMessage> msg = m_controllers[Controller::Depth].stateMsg();
        msg->demand(m_demand[Controller::Depth]);
        m_mb->sendMessage(msg, RELIABLE_MSG);

        foreach(boost::shared_ptr<GraphableMessage> m, m_controllers[Controller::Bearing].extraStateMessages()){
            m->name("Depth-" + m->name());
            m_mb->sendMessage(m, RELIABLE_MSG);
        }
    }
}

void ControlLoops::onMotorMessage(MotorMessage_ptr m)
{
    debug(2) << "Set manual motor demand based on motor message:" << *m;

    switch(m->motorId())
    {
        case MotorID::Prop:   m_demand[Controller::ManualOverride].prop   = m->speed(); break;
        case MotorID::HBow:   m_demand[Controller::ManualOverride].hbow   = m->speed(); break;
        case MotorID::HStern: m_demand[Controller::ManualOverride].hstern = m->speed(); break;
        case MotorID::VBow:   m_demand[Controller::ManualOverride].vbow   = m->speed(); break;
        case MotorID::VStern: m_demand[Controller::ManualOverride].vstern = m->speed(); break;
        case MotorID::NumValues: break;
    }
}

template <typename T>
void update_controller_params(PIDControl &p, T &m) {
    p.Kp = m.Kp();
    p.Ki = m.Ki();
    p.Kd = m.Kd();
    p.Ap = m.Ap();
    p.Ai = m.Ai();
    p.Ad = m.Ad();
    p.thr = m.thr();
    p.scale = m.scale();
    p.errorMAX = m.maxError();
    p.reset();
}

void ControlLoops::onBearingAutopilotParamsMessage(BearingAutopilotParamsMessage_ptr m)
{
    update_controller_params(m_controllers[Controller::Bearing], *m);
}
void ControlLoops::onPitchAutopilotParamsMessage(PitchAutopilotParamsMessage_ptr m)
{
    update_controller_params(m_controllers[Controller::Pitch], *m);
}
void ControlLoops::onDepthAutopilotParamsMessage(DepthAutopilotParamsMessage_ptr m)
{
    update_controller_params(m_controllers[Controller::Depth], *m);
}

template <typename T>
void update_controller_status(PIDControl &p, bool &enabled, T &m) {
    if (m->enabled()) {
        if (!enabled) {
            enabled = true;
            p.reset();
        }
        p.target = m->target();
    }
    else {
        enabled = false;
    }
}

void ControlLoops::onBearingAutopilotEnabledMessage(BearingAutopilotEnabledMessage_ptr m)
{
    update_controller_status(m_controllers    [Controller::Bearing],
                             m_control_enabled[Controller::Bearing], m);
}

void ControlLoops::onPitchAutopilotEnabledMessage(PitchAutopilotEnabledMessage_ptr m)
{
    update_controller_status(m_controllers    [Controller::Pitch],
                             m_control_enabled[Controller::Pitch], m);
}

void ControlLoops::onDepthAutopilotEnabledMessage(DepthAutopilotEnabledMessage_ptr m)
{
    update_controller_status(m_controllers    [Controller::Depth],
                             m_control_enabled[Controller::Depth], m);
}

void ControlLoops::onMotorRampRateMessage(MotorRampRateMessage_ptr m)
{
    // MCB currently handles ramping: don't limit it
    //if(m->maxDelta() >= 127)
    //    warning() << "maximum motor delta set exceptionally high:" << m->maxDelta();
    m_max_motor_delta = m->maxDelta();
    m_motor_updates_per_second = m->updatesPerSecond();
    debug() << "Set motor ramp limit:" << m_max_motor_delta
        << ", update rate:" << m_motor_updates_per_second;
}

void ControlLoops::onSetMotorMapMessage(SetMotorMapMessage_ptr m)
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
        debug(5) << "new map example: " << test_values[i] << "->" << motorMap(test_values[i], m->motor());
}

void ControlLoops::motorControlLoop()
{
    debug() << "Control loop thread started";
    const MotorDemand no_demand(0,0,0,0,0);
    for(int i = 0; i < Controller::NumValues; i++)
    {
        m_control_enabled[i] = false;
        m_controllers[i].reset();
        m_controllers[i].controlee = (Controller)i;
        m_demand[i] = no_demand;

        boost::shared_ptr<ControllerStateMessage> msg = m_controllers[i].stateMsg();
        msg->demand(m_demand[i]);
        m_mb->sendMessage(msg, RELIABLE_MSG);
    }
    m_controllers[Controller::Pitch].is_angle = true;
    m_controllers[Controller::Bearing].is_angle = true;
    m_control_enabled[Controller::ManualOverride] = true;

    try {
        while(true)
        {
            boost::this_thread::interruption_point();
            updateMotorControl();
            if(m_motor_updates_per_second) {
                msleep(1000/m_motor_updates_per_second);
            }
        }
    } catch (boost::thread_interrupted&) {
        debug() << "Control loop thread interrupted";
    }

    debug() << "Control loop thread exiting";
}

int ControlLoops::motorMap(float const& demand_value, MotorID mid)
{
    MotorMap m(0, 0, 127, -127);
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
    int ret = 0;
    if(demand_value < -127) {
        ret =  m.maxMinus;
    } else if(demand_value < 0) {
        //eg:      -30     + (   -30      -   -120    ) *     (-50) / 127 = -65
        ret =  m.zeroMinus + (m.zeroMinus - m.maxMinus) * demand_value / 127;
    } else if(demand_value == 0) {
        ret = 0;
    } else if(demand_value <= 127) {
        //eg:     50      + (  127     -     50    ) *    80        / 127 = 86
        ret = m.zeroPlus + (m.maxPlus - m.zeroPlus) * demand_value / 127;
    } else {
        ret =  m.maxPlus;
    }
    return clamp(-127, ret, 127);
}

int applyDelta(int &oldValue, int newValue, unsigned int maxDelta) {
    int ret = 0;
    if(unsigned(abs(newValue - oldValue)) <= maxDelta) {
        ret = newValue;
    } else if(newValue < oldValue) {
        ret = oldValue - maxDelta;
    } else {
        ret = oldValue + maxDelta;
    }
    oldValue = newValue;
    return ret;
}

void ControlLoops::updateMotorControl()
{
    // TODO: This may need a lock (demands updating from messaging thread
    // while this one reads them)
    //
    // Maybe not, reading/writing ints is atomic. 
    MotorDemand total_demand(0,0,0,0,0);
    for(int i = 0; i < Controller::NumValues; i++) {
        if(m_control_enabled[i]) {
            total_demand += m_demand[i];
        }
    }
    
    MotorDemand mapped_demand;

    //bleh, duplication all over the place. Needs a refactor
    mapped_demand.prop = motorMap(total_demand.prop, MotorID::Prop);
    mapped_demand.hbow = motorMap(total_demand.hbow, MotorID::HBow);
    mapped_demand.vbow = motorMap(total_demand.vbow, MotorID::VBow);
    mapped_demand.hstern = motorMap(total_demand.hstern, MotorID::HStern);
    mapped_demand.vstern = motorMap(total_demand.vstern, MotorID::VStern);

    mapped_demand.prop = applyDelta(prop_value, mapped_demand.prop, m_max_motor_delta);
    mapped_demand.hbow = applyDelta(hbow_value, mapped_demand.hbow, m_max_motor_delta);
    mapped_demand.vbow = applyDelta(vbow_value, mapped_demand.vbow, m_max_motor_delta);
    mapped_demand.hstern = applyDelta(hstern_value, mapped_demand.hstern, m_max_motor_delta);
    mapped_demand.vstern = applyDelta(vstern_value, mapped_demand.vstern, m_max_motor_delta);

    m_mcb->setMotorState(mapped_demand);
    m_mb->sendMessage(boost::make_shared<MotorStateMessage>(MotorID::Prop, prop_value), RELIABLE_MSG);
    m_mb->sendMessage(boost::make_shared<MotorStateMessage>(MotorID::HBow, hbow_value), RELIABLE_MSG);
    m_mb->sendMessage(boost::make_shared<MotorStateMessage>(MotorID::VBow, vbow_value), RELIABLE_MSG);
    m_mb->sendMessage(boost::make_shared<MotorStateMessage>(MotorID::HStern, hstern_value), RELIABLE_MSG);
    m_mb->sendMessage(boost::make_shared<MotorStateMessage>(MotorID::VStern, vstern_value), RELIABLE_MSG);
}

class TelemetryBroadcaster : public IMUObserver, public MCBObserver
{
    public:
        TelemetryBroadcaster(boost::shared_ptr<Mailbox> mb)
            : m_mb(mb)
        {
        }

        ~TelemetryBroadcaster(){
            stop();
        }

        void start() {
            stop();
            m_telemetryThread = boost::thread(&TelemetryBroadcaster::sendTelemetry, this);
        }
        void stop() {
            if (m_telemetryThread.get_id() != boost::thread::id()) {
                m_telemetryThread.interrupt();    
                m_telemetryThread.join();    
            }
        }

        //from MCB
        virtual void onDepth(float fore, float aft) {
            m_depth = (fore + aft) / 2;
        }

        //from IMU
        virtual void onTelemetry(const floatYPR& attitude) {
            m_orientation = attitude;
        }

    protected:
        boost::shared_ptr<Mailbox> m_mb;

        floatYPR m_orientation;
        float m_depth;
        
        boost::thread m_telemetryThread;
        void sendTelemetry()
        {
            try {
                debug() << "Send telemetry thread started";
                while(true)
                {
                    m_mb->sendMessage(boost::make_shared<TelemetryMessage>(m_orientation, m_depth), RELIABLE_MSG);
                    msleep(100);
                }
            } catch (boost::thread_interrupted&) {
                debug() << "Send telemetry thread interrupted";
            }
            debug() << "Send telemetry thread ending";
        }
};

class NotRootException : public std::exception
{
    public:
        virtual const char* what() const throw() {
            return "Need to be running as root";
        }
};

//Mess of initialisation code starts here...

class ControlNode : public CauvNode
{
    public:
        ControlNode();
        virtual ~ControlNode();
    
#ifdef CAUV_MCB_IS_FTDI
        void setPICMcb(int id);
#else 
        void setPICMcb(std::string filename);
#endif
        void setArmMcb(std::string port);

        void setXsens(int id);
        void setSBG (std::string port, int baud_rate, int pause_time);
        void setSim ();
    
    protected:
        boost::shared_ptr<MCB> m_mcb;
        boost::shared_ptr<IMU> m_imu;
        boost::shared_ptr<ControlLoops> m_controlLoops;
        boost::shared_ptr<TelemetryBroadcaster> m_telemetryBroadcaster;
    
        virtual void addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
        virtual void onRun();
};

ControlNode::ControlNode() : CauvNode("Control")
{
    subMessage(MotorControlMessage());
    subMessage(LightControlMessage());
    subMessage(PowerControlMessage());
    subMessage(CuttingDeviceMessage());
    subMessage(StateMessage());
    subMessage(MotorMessage());
    subMessage(BearingAutopilotEnabledMessage());
    subMessage(DepthAutopilotEnabledMessage());
    subMessage(PitchAutopilotEnabledMessage());
    subMessage(BearingAutopilotParamsMessage());
    subMessage(DepthAutopilotParamsMessage());
    subMessage(PitchAutopilotParamsMessage());
    subMessage(MotorRampRateMessage());
    subMessage(SetMotorMapMessage());
    subMessage(CalibrateNoRotationMessage());

    m_controlLoops = boost::make_shared<ControlLoops>(mailbox());
    m_telemetryBroadcaster = boost::make_shared<TelemetryBroadcaster>(mailbox());
    addMessageObserver(m_controlLoops);
}

ControlNode::~ControlNode()
{
}

void ControlNode::setXsens(int id)
{
    // start up the Xsens IMU
    
    boost::shared_ptr<XsensIMU> xsens;

    try {
        xsens = boost::make_shared<XsensIMU>(id);
        info() << "Xsens Connected";
        
        CmtOutputMode om = CMT_OUTPUTMODE_CALIB | CMT_OUTPUTMODE_ORIENT;
        CmtOutputSettings os = CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_DATAFORMAT_FLOAT | CMT_OUTPUTSETTINGS_CALIBMODE_ACCMAG;

        CmtMatrix m;
        m.m_data[0][0] =  1.0; m.m_data[0][1] =  0.0; m.m_data[0][2] =  0.0; 
        m.m_data[1][0] =  0.0; m.m_data[1][1] =  1.0; m.m_data[1][2] =  0.0; 
        m.m_data[2][0] =  0.0; m.m_data[2][1] =  0.0; m.m_data[2][2] =  1.0; 

        xsens->setObjectAlignmentMatrix(m);
        xsens->configure(om, os);
        
        m_imu = xsens;
                
        info() << "XSens Configured";
    } catch (XsensException& e) {
        error() << "Cannot connect to Xsens: " << e.what();
        xsens.reset();
    }
}

void ControlNode::setSBG (std::string port, int baud_rate, int pause_time)
{
    // start up the SBG IMU

    boost::shared_ptr<sbgIMU> sbg;
    try {
        sbg = boost::make_shared<sbgIMU>(port.c_str(), baud_rate, pause_time);
        sbg->initialise();
        info() << "sbg Connected";
        m_imu = sbg;
        //info() << "sbg configured";

    } catch (sbgException& e) {
        error() << "Cannot connect to sbg: " << e.what ();
        sbg.reset();
    }
}

#ifdef CAUV_MCB_IS_FTDI
void ControlNode::setPICMcb (int conn) {

}
#else
void ControlNode::setPICMcb (std::string port) {

}
#endif

void ControlNode::setSim (void) {
    boost::shared_ptr<SimIMU> sim = boost::make_shared<SimIMU>();
    addMessageObserver(sim);
    m_imu = sim;
    m_mcb = sim;
    joinGroup("pressure");
    joinGroup("state");
}

void ControlNode::setArmMcb(std::string port) {
    boost::shared_ptr<ArmMcb> arm = boost::make_shared<ArmMcb>(port);
    m_mcb = arm;
}

void ControlNode::addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos)
{
    namespace po = boost::program_options;
    CauvNode::addOptions(desc, pos);
    
    desc.add_options()
        ("xsens,x", po::value<int>()->default_value(0), "USB device id of the Xsens")
        ("sbg,b", po::value<std::string>()->default_value("/dev/ttyUSB1"), "TTY device for SBG IG500A")
        ("imu,i", po::value<std::string>()->default_value("xsens"), "default Xsens USB device or TTY device for SBG IG500A, or both")
        ("mcb,m", po::value<std::string>()->default_value("PIC"), "default mcb to use: PIC or ARM")

#ifdef CAUV_MCB_IS_FTDI
        ("port,p", po::value<int>()->default_value(0), "FTDI device id of the MCB")
#else 
        ("port,p", po::value<std::string>()->default_value("/dev/ttyUSB0"), "TTY file for MCB serial comms")
#endif
        ("depth-offset,o", po::value<float>()->default_value(0), "Depth calibration offset")
        ("depth-scale,s", po::value<float>()->default_value(0), "Depth calibration scale")

        ("simulation,N", "Run in simulation mode");
}

int ControlNode::useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc)
{
    namespace po = boost::program_options;
    int ret = CauvNode::useOptionsMap(vm, desc);
    if (ret != 0) return ret;

    bool use_xsens = false, use_sbg = false, use_sim = false;
    if (vm["imu"].as<std::string>() == ("xsens")) use_xsens = true;
    if (vm["imu"].as<std::string>() == ("sbg")) use_sbg = true;
    if (vm["imu"].as<std::string>() == ("sim") || vm.count("simulation")) {
        use_sim = true;
        use_xsens = false;
        use_sbg = false;
    }
    if (use_sim) {
        setSim();
    }
    if (vm.count("xsens") && use_xsens) {
        setXsens(vm["xsens"].as<int>());
    }
    if(vm.count("sbg") && use_sbg){
        setSBG(vm["sbg"].as<std::string>(), 115200, 10);
    }
#ifdef CAUV_MCB_IS_FTDI
    if (vm["mcb"].as<std::string>() == ("PIC")) {
        setPICMcb(vm["port"].as<int>());
    }
#else
    if (vm["mcb"].as<std::string>() == ("PIC")) {
        setPICMcb(vm["port"].as<std::string>());
    } 
#endif
    if (vm["mcb"].as<std::string>() == ("ARM")) {
        setArmMcb(vm["port"].as<std::string>());
    }
//    if (vm.count("depth-offset") && vm.count("depth-scale")) {
//        float offset = vm["depth-offset"].as<float>();
//        float scale = vm["depth-scale"].as<float>();
//        m_controlLoops->onDepthCalibrationMessage(boost::make_shared<DepthCalibrationMessage>(offset,scale,offset,scale));
//    }
//    else if (vm.count("depth-offset") || vm.count("depth-scale")) {
//        warning() << "Need both offset and depth for calibration; ignoring calibration input";   
//    }

    return 0;
}

void ControlNode::onRun()
{
    CauvNode::onRun();
   
    if (m_mcb) {
        m_controlLoops->set_mcb(m_mcb);
        
        m_mcb->addObserver(m_telemetryBroadcaster);
        m_mcb->addObserver(m_controlLoops);
        
        m_mcb->start();
    }
    else {
        warning() << "MCB not connected. No MCB comms available, so no motor control.";
    }

    if (m_imu) {
        m_imu->addObserver(boost::make_shared<DebugIMUObserver>(5));
        m_imu->addObserver(m_telemetryBroadcaster);
        m_imu->addObserver(m_controlLoops);

        m_imu->start();
    }
    else {
        warning() << "IMU not connected. Telemetry not available.";
    }

    m_controlLoops->start();
    m_telemetryBroadcaster->start();
}

void interrupt(int sig)
{
    std::cout << std::endl;
    info() << BashColour::Red << "Interrupt caught!";
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    signal(SIGINT, interrupt);
    
    try {
        ControlNode node;
    
        int ret = node.parseOptions(argc, argv);
        if(ret != 0) return ret;
        
        node.run();
    }
    catch (NotRootException& e) {
        error() << e.what();
    }
    
    return 0;
}
