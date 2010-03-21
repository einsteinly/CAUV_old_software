#include <iostream>
#include <sstream>
#include <fstream>

#include <common/bash_cout.h>

#include "cauv_global.h"

using namespace std;

cauv_global* cauv_global::m_current = 0;

cauv_global::cauv_global() //: m_socket(0)
{
}
cauv_global& cauv_global::current()
{
	if (!m_current)
		m_current = new cauv_global();
	return *m_current;
}

/*
void cauv_global::set_socket(CommunicatingSocket* socket)
{
	current().m_socket = socket;
}
*/

void cauv_global::send_trace(const string& msg) const
{
/*	TraceMessage m;
 *  m.msg(msg);
	try
    {
        if (m_mailbox)
            m_mailbox->sendMessage(m, LOW_MESS);
    }
    catch (SpreadException& e)
    {
        cout << "Error sending debug data: " << e.what() << endl;
    }
*/
}
void cauv_global::send_error(const string& msg) const
{
/*	ErrorMessage m(msg);
	try
    {
        if (m_socket)
            m_socket->sendMessage(m);
		else
			cout << "\E[1;30mError not sent: no socket.\E[m" << endl;
    }
    catch (SocketException& e)
    {
        cout << "Error sending debug data: " << e.what() << endl;
    }
*/
}

void cauv_global::print_logo(char* start, char* end, const string& module_name)
{
	char* cur = start;
    stringstream ss;
	ss << endl;
    while (cur != end)
	{
		if (*cur == '\n')
			ss << BashControl::Reset << endl;
		else if (*cur == *(cur-1))
			ss << ' ';
		else
		{
			switch(*cur)
			{
				case 'K':
					ss << BashBackground::Black << ' ';
                    break;
				case 'R':
					ss << BashBackground::Red << ' ';
					break;
				case 'G':
					ss << BashBackground::Green << ' ';
					break;
				case 'Y':
					ss << BashBackground::Yellow << ' ';
					break;
				case 'B':
					ss << BashBackground::Blue << ' ';
					break;
				case 'M':
					ss << BashBackground::Magenta << ' ';
					break;
				case 'C':
					ss << BashBackground::Cyan << ' ';
					break;
				case 'W':
					ss << BashBackground::White << ' ';
					break;
				case ' ':
					ss << BashControl::Reset << ' ';
					break;
				case 'c':
				{
					ss << BashColour::Blue << BashIntensity::Bold;
					int len = 0;
					while (cur != end && *cur == 'c')
                    {
						len++;
					    cur++;
                    }
                    if (len >= (int)module_name.size())
					{
						int start = (len - module_name.size())/2;
						string pad(start, ' ');
						ss << pad << module_name << pad;
					}
					else
					{
						ss << module_name.substr(0, len);
					}
				}
					break;
				default:
					break;
			}
		}
        cur++;
	}
    cout << ss.str();
}


void cauv_global::print_module_header(const string& module_name)
{
    static char cauv_logo_large[] = {
        #include <common/cauv_logo_large.h>
    };
	cauv_global::print_logo(cauv_logo_large,
                            cauv_logo_large + sizeof(cauv_logo_large),
                            module_name);
}



