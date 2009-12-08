#include "control.h"

#include <iostream>

using namespace std;

LOAD_NODE(control_node)

control_node::control_node(const string& group) : cauv_node("Control", group)
{
}

control_node::~control_node()
{
}
