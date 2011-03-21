#ifndef __CAUV_CONTROL_H__
#define __CAUV_CONTROL_H__

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <common/cauv_node.h>

class ControlLoops;
class StateObserver;
class TelemetryBroadcaster;

namespace cauv{

class MCBModule;
class XsensIMU;

class ControlNode : public CauvNode
{
    public:
        ControlNode();
        virtual ~ControlNode();
    
#ifdef CAUV_MCB_IS_FTDI
        void setMCB(int id);
#else 
        void setMCB(const std::string& filename);
#endif
        void setXsens(int id);
    
    protected:
        boost::shared_ptr<MCBModule> m_mcb;
        boost::shared_ptr<XsensIMU> m_xsens;
        boost::shared_ptr<ControlLoops> m_controlLoops;
        boost::shared_ptr<StateObserver> m_stateObserver;
        boost::shared_ptr<TelemetryBroadcaster> m_telemetryBroadcaster;
    
        boost::thread m_aliveThread;

        virtual void addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
        virtual void onRun();
};

} // namespace cauv

#endif // ndef __CAUV_CONTROL_H__