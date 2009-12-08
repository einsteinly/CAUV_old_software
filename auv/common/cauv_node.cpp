#include <iostream>
#include <limits>
#include <string>

#include <common/cauv_global.h>
#include <common/colors.h>

#include "cauv_node.h"


using namespace std;

void CauvNode::onConnect()
{
	//cauv_global::set_socket(m_socket);
}
void CauvNode::onDisconnect()
{
	//cauv_global::set_socket(0);
}
void CauvNode::onRun()
{
}


CauvNode::CauvNode(const string& name, const string& group) : m_name(name), m_group(group)
{
}

CauvNode::~CauvNode()
{
    
	cout << "Shutting down node" << endl;
}

void CauvNode::run()
{
	cauv_global::print_module_header(m_name);

	onRun();

    // SPREAD connect to group named "m_group"

	//cin.clear();
	
    // This won't do anything until you press enter
    //cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cout << "Exiting" << endl;
}

