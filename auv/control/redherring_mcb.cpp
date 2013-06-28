/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

//Ye gods, this needs to be put back together in some kind of sane manner
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
DepthCalibrationMessage_ptr m_depthCalibration;

virtual void onDepthCalibrationMessage(DepthCalibrationMessage_ptr m);
       
virtual void onPressureMessage(PressureMessage_ptr m);
void ControlLoops::onPressureMessage(PressureMessage_ptr m)
{
    if (m_control_enabled[Controller::Depth] && m_depthCalibration){
        float fore_depth_calibrated = m_depthCalibration->foreOffset() +
            m_depthCalibration->foreMultiplier()
            * m->fore();
        float aft_depth_calibrated = m_depthCalibration->aftOffset() +
            m_depthCalibration->aftMultiplier() * m->aft();
        float depth = 0.5 * (fore_depth_calibrated + aft_depth_calibrated);

        //FIXME: MCB sends pairs of pressure messages very close
        //together in time, this screws up derivative calculation
        static TimeStamp last_pressure_message_time = now();
        if(now() - last_pressure_message_time < 10)
            return;
        else
            last_pressure_message_time = now();

void ControlLoops::sendIfNew(MotorID mid, int& oldvalue, int newvalue)
{
    if(newvalue != oldvalue) {
        oldvalue = newvalue;
        if(m_mcb){
            if(mid == MotorID::Prop)
                m_mcb->send(boost::make_shared<MotorMessage>(mid, -newvalue));
            // VBow is the wrong way around, this set-up is for the
            // ducts to be on the bottom, as set on Red Herring on 22/3/2012
            else if(mid != MotorID::VBow)
                m_mcb->send(boost::make_shared<MotorMessage>(mid, newvalue));
            else
                m_mcb->send(boost::make_shared<MotorMessage>(mid, -newvalue));
        }
        m_mb->sendMessage(boost::make_shared<MotorStateMessage>(mid, newvalue), RELIABLE_MSG);
    }
}

class DeviceControlObserver : public MessageObserver
{
    public:
        void set_mcb(boost::shared_ptr<MCBModule> mcb)
        {
            m_mcb = mcb;
        }
        void set_imu(boost::shared_ptr<IMU> imu)
        {
            m_imu = imu;
        }
        
        virtual void onLightMessage(LightMessage_ptr m)
        {
            if (m_mcb) {
                debug(2) << "Forwarding Light Message:" << *m;
                m_mcb->send(m);
            }
            else
                warning() << "Tried to set lights, but there's no MCB";
        }

        virtual void onCuttingDeviceMessage(CuttingDeviceMessage_ptr m)
        {
            if (m_mcb) {
                debug(2) << "Forwarding Cutting Device Control Message:" << *m;
                m_mcb->send(m);
            }
            else
                warning() << "Tried to cut some shit up, but there's no MCB";
        }

    protected:
        boost::shared_ptr<MCBModule> m_mcb;
        boost::shared_ptr<IMU> m_imu;
};

{
    public:
        void set_mcb(boost::shared_ptr<MCBModule> mcb)
        {
            m_mcb = mcb;
        }
        void set_imu(boost::shared_ptr<IMU> imu)
        {
            m_imu = imu;
        }
        
        virtual void onLightMessage(LightMessage_ptr m)
        {
            if (m_mcb) {
                debug(2) << "Forwarding Light Message:" << *m;
                m_mcb->send(m);
            }
            else
                warning() << "Tried to set lights, but there's no MCB";
        }

        virtual void onCuttingDeviceMessage(CuttingDeviceMessage_ptr m)
        {
            if (m_mcb) {
                debug(2) << "Forwarding Cutting Device Control Message:" << *m;
                m_mcb->send(m);
            }
            else
                warning() << "Tried to cut some shit up, but there's no MCB";
        }

        virtual void onCalibrateNoRotationMessage(CalibrateNoRotationMessage_ptr m)
        {
            boost::shared_ptr<XsensIMU> xsens = boost::dynamic_pointer_cast<XsensIMU>(m_imu);
            if (xsens) {
                xsens->calibrateNoRotation(m->duration());
            }
            else
                warning() << "Tried to perform no rotation calibration, but there's no XSens";
        }
    
    protected:
        boost::shared_ptr<MCBModule> m_mcb;
        boost::shared_ptr<IMU> m_imu;
};


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

class MCBForwardingObserver : public BufferedMessageObserver
{
    public:
        MCBForwardingObserver(boost::shared_ptr<Mailbox> mb)
            : m_pressure_min_msecs(50),
              m_battery_min_msecs(5000),
              m_last_pressure_sent(now()),
              m_last_battery_sent(now()),
              m_mb(mb)
        {
            setDoubleBuffered(MessageType::Pressure, true);
            setDoubleBuffered(MessageType::BatteryStatus, true);
        }

        virtual void onPressureMessage(PressureMessage_ptr m)
        {
            if(now() - m_last_pressure_sent > m_pressure_min_msecs){        
                debug(5) << "MCBForwardingObserver: Forwarding pressure message:" << *m;
                m_mb->sendMessage(m, UNRELIABLE_MSG);
                m_last_pressure_sent = now();
            }
        }
        virtual void onBatteryStatusMessage(BatteryStatusMessage_ptr m)
        {
            if(now() - m_last_battery_sent > m_battery_min_msecs){
                debug(5) << "MCBForwardingObserver: Forwarding battery status message:" << *m;
                m_mb->sendMessage(m, UNRELIABLE_MSG);
                m_last_battery_sent = now();
            }
        }
        virtual void onDebugMessage(DebugMessage_ptr m)
        {
            debug(5) << "MCBForwardingObserver: Forwarding debug message:" << *m;
            m_mb->sendMessage(m, RELIABLE_MSG);
        }
    protected:
        float m_pressure_min_msecs;
        float m_battery_min_msecs;
        TimeStamp m_last_pressure_sent;
        TimeStamp m_last_battery_sent;
        boost::shared_ptr<Mailbox> m_mb;
};


void ControlNode::setMCB(const std::string& filename)
{
    m_mcb.reset();
    // start up the MCB module
    try {
        m_mcb = boost::make_shared<MCBModule>(filename);
        info() << "MCB Connected";
    }
    catch (std::exception& e)
    {
        error() << "Cannot connect to MCB on " << filename << ":" << e.what();
        m_mcb.reset();
    }
}

        m_mcb->addObserver(boost::make_shared<DebugMessageObserver>(2));
        m_mcb->addObserver(boost::make_shared<MCBForwardingObserver>(mailbox()));


void ControlLoops::onPressureMessage(PressureMessage_ptr m)
{
    if (m_control_enabled[Controller::Depth] && m_depthCalibration){
        float fore_depth_calibrated = m_depthCalibration->foreOffset() +
            m_depthCalibration->foreMultiplier()
            * m->fore();
        float aft_depth_calibrated = m_depthCalibration->aftOffset() +
            m_depthCalibration->aftMultiplier() * m->aft();
        float depth = 0.5 * (fore_depth_calibrated + aft_depth_calibrated);

        //FIXME: MCB sends pairs of pressure messages very close
        //together in time, this screws up derivative calculation
        static TimeStamp last_pressure_message_time = now();
        if(now() - last_pressure_message_time < 10)
            return;
        else
            last_pressure_message_time = now();

        float mv = m_controllers[Controller::Depth].getMV(depth);

        m_demand[Controller::Depth].vbow = mv;
        m_demand[Controller::Depth].vstern = mv;

        debug(2) << "depth: fwd=" << fore_depth_calibrated
            << "aft=" << aft_depth_calibrated
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
void ControlLoops::onDepthCalibrationMessage(DepthCalibrationMessage_ptr m)
{
    m_depthCalibration = m;
}

if (!m_depthCalibration) {
    warning() << "depth control will not be effective until calibration factors are set";
}
