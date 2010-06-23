#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>
#include <module/module.h>

#include "xsens_imu.h"

class ControlLoops;
class ControlNode : public CauvNode
{
    public:
        ControlNode();
        virtual ~ControlNode();
    
    protected:
        boost::shared_ptr<MCBModule> m_mcb;
        boost::shared_ptr<XsensIMU> m_xsens;
        boost::shared_ptr<ControlLoops> m_controlLoops;
    
        boost::thread m_aliveThread;

        virtual void onRun();
};

#endif//__CONTROL_H__
