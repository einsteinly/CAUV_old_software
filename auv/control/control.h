#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <common/cauv_node.h>

class ControlLoops;
class MCBModule;
class XsensIMU;
class ControlLoops;

class ControlNode : public CauvNode
{
    public:
        ControlNode();
        virtual ~ControlNode();
    
        void setMCB(int id);
        void setXsens(int id);
    
    protected:
        boost::shared_ptr<MCBModule> m_mcb;
        boost::shared_ptr<XsensIMU> m_xsens;
        boost::shared_ptr<ControlLoops> m_controlLoops;
    
        boost::thread m_aliveThread;

        virtual void addOptions(boost::program_options::options_description& desc);
        virtual int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
        virtual void onRun();
};

#endif//__CONTROL_H__
