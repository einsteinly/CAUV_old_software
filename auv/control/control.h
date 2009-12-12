#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <common/cauv_node.h>

#include "xsens_imu.h"

class ControlNode : public CauvNode
{
    public:
        ControlNode(const std::string& group);
        virtual ~ControlNode();
    
    protected:
        XsensIMU* m_xsens;

        virtual void onRun();
};

#endif//__CONTROL_H__
