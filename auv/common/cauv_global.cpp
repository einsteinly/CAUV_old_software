#include <iostream>
#include <sstream>
#include <fstream>

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
	cout << "\E[1mTRACE:\E[m " << msg << endl;
/*	TraceMessage m(msg);
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
void cauv_global::send_error(const string& msg) const
{
	cout << "\E[1;31mERROR: \E[m" << msg << endl;
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
	while (cur != end)
	{
		if (*cur == '\n')
			cout << "\E[m" << endl;
		else if (*cur == *(cur-1))
			cout << ' ';
		else
		{
			switch(*cur)
			{
				case 'K':
					cout << "\E[40m ";
					break;
				case 'R':
					cout << "\E[41m ";
					break;
				case 'G':
					cout << "\E[42m ";
					break;
				case 'Y':
					cout << "\E[43m ";
					break;
				case 'B':
					cout << "\E[44m ";
					break;
				case 'M':
					cout << "\E[45m ";
					break;
				case 'C':
					cout << "\E[46m ";
					break;
				case 'W':
					cout << "\E[47m ";
					break;
				case ' ':
					cout << "\E[m ";
					break;
				case 'c':
				{
					cout << "\E[m\E[1;34m";
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
						cout << pad << module_name << pad;
					}
					else
					{
						cout << module_name.substr(0, len);
					}
				}
					break;
				default:
					break;
			}
		}
        cur++;
	}
}

void cauv_global::print_cauv_logo()
{
	cout << "\E[m \E[47m   \E[m  \E[47m  \E[m  \E[47m \E[m  \E[47m \E[m \E[47m \E[m  \E[47m \E[m" << endl;
	cout << "\E[47m \E[m    \E[47m \E[m  \E[47m \E[m \E[47m \E[m  \E[47m \E[m \E[47m \E[m  \E[47m \E[m" << endl;
	cout << "\E[44m \E[m    \E[44m "/*\E[m\E[34m\u2501\u2501\E[m\E[44m*/"   \E[m \E[44m \E[m  \E[44m \E[m \E[44m \E[m  \E[44m \E[m" << endl;
	cout << "\E[m \E[44m   \E[m \E[44m \E[m  \E[44m \E[m \E[44m    \E[m  \E[44m  \E[m" << endl;
}
void cauv_global::print_module_header(const string& module_name)
{
	cauv_global::print_logo(&_binary_common_cauv_logo_large_txt_start, &_binary_common_cauv_logo_large_txt_end, module_name);
}



