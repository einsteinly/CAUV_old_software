#include <iostream>
#include <limits>
#include <string>

#include <common/cauv_global.h>
#include <common/colors.h>

#include "cauv_node.h"


using namespace std;

void cauv_node::on_connect()
{
	//cauv_global::set_socket(m_socket);
}
void cauv_node::on_disconnect()
{
	//cauv_global::set_socket(0);
}
void cauv_node::on_run()
{
}


cauv_node::cauv_node(const string& name, const string& group) : m_name(name), m_group(group)
{
}

cauv_node::~cauv_node()
{
    
	cout << "Shutting down node" << endl;
}

void cauv_node::run()
{
	cauv_global::print_module_header(m_name);

	on_run();

    // SPREAD connect to group named "m_group"

	cin.clear();
	
    // This won't do anything until you press enter
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cout << "Exiting" << endl;
}

