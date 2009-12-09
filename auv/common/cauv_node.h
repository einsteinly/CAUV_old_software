#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <signal.h>
#include <iostream>

using namespace std;

class CauvNode
{
	public:
		virtual ~CauvNode();
		
        void run();

	protected:
		string m_name;
		string m_group;

		virtual void onConnect();
		virtual void onDisconnect();
		virtual void onRun();

        CauvNode(const string& name, const string& group);
};

#endif//__CAUV_NODE_H__

