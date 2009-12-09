#include <iostream>
#include <sstream>
#include <fstream>

#include <common/bash_cout.h>

#include "cauv_global.h"

extern char _binary_common_cauv_logo_large_txt_end;
extern char _binary_common_cauv_logo_large_txt_start;

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
	cout << setcolour("black") << setbold << "TRACE: " << resetcolour << msg << endl;
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
	cout << setcolour("red") << setbold << "ERROR: " << resetcolour << msg << endl;
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

void cauv_global::trace(const string& msg)
{
	cauv_global::current().send_trace(msg);
}
void cauv_global::error(const string& msg)
{
	cauv_global::current().send_error(msg);
}
void cauv_global::error(const string& head, exception& e)
{
	stringstream ss;
	ss << head << ": " << e.what();
	cauv_global::current().send_error(ss.str());
}

void cauv_global::print_logo(char* start, char* end, const string& module_name)
{
	char* cur = start;
    stringstream ss;
	while (cur != end)
	{
		if (*cur == '\n')
			ss << resetcolour << endl;
		else if (*cur == *(cur-1))
			ss << ' ';
		else
		{
			switch(*cur)
			{
				case 'K':
					ss << setbg("black") << ' ';
                    break;
				case 'R':
					ss << setbg("red") << ' ';
					break;
				case 'G':
					ss << setbg("green") << ' ';
					break;
				case 'Y':
					ss << setbg("yellow") << ' ';
					break;
				case 'B':
					ss << setbg("blue") << ' ';
					break;
				case 'M':
					ss << setbg("magenta") << ' ';
					break;
				case 'C':
					ss << setbg("cyan") << ' ';
					break;
				case 'W':
					ss << setbg("white") << ' ';
					break;
				case ' ':
					ss << resetcolour << ' ';
					break;
				case 'c':
				{
					ss << setcolour("blue") << setbold;
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
	cauv_global::print_logo(&_binary_common_cauv_logo_large_txt_start, &_binary_common_cauv_logo_large_txt_end, module_name);
}



