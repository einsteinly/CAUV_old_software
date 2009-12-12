#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <signal.h>
#include <iostream>

class CauvNode
{
	public:
		virtual ~CauvNode();
		
        void run();

	protected:
		std::string m_name;
		std::string m_group;

		virtual void onConnect();
		virtual void onDisconnect();
		virtual void onRun();

        CauvNode(const std::string& name, const std::string& group);
};

#endif//__CAUV_NODE_H__

