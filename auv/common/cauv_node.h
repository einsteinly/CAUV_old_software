#ifndef __CAUV_NODE_H__
#define __CAUV_NODE_H__

#include <string>
#include <signal.h>
#include <iostream>

using namespace std;

class cauv_node
{
	public:
		virtual ~cauv_node();

		void run();

	protected:
		string m_name;
		string m_group;

		virtual void on_connect();
		virtual void on_disconnect();
		virtual void on_run();

        cauv_node(const string& name, const string& group);
};

#define LOAD_NODE(NODE)                  \
                                         \
NODE* node;                              \
                                         \
void cleanup()                           \
{                                        \
    cout << "Cleaning up..." << endl;    \
    cauv_node* oldnode = node;           \
    node = 0;                            \
    delete oldnode;                      \
    cout << "Clean up done." << endl;    \
}                                        \
                                         \
void interrupt(int sig)                  \
{                                        \
    cout << "Interrupt caught!" << endl; \
    cleanup();                           \
    signal(SIGINT, SIG_DFL);             \
    raise(sig);                          \
}                                        \
                                         \
int main(int argc, char **argv)          \
{                                        \
    signal(SIGINT, interrupt);           \
    node = new NODE("cauv");             \
    node->run();                         \
    cleanup();                           \
}                                        \

#endif//__CAUV_NODE_H__

