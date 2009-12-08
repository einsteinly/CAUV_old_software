#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <common/cauv_node.h>

class control_node : public cauv_node
{
    public:
        control_node(const string& group);
        ~control_node();
};

#endif//__CONTROL_H__
