#ifndef __CAUV_GLOBAL_H__
#define __CAUV_GLOBAL_H__

#include <string>

using namespace std;

class cauv_global
{
	public:
		static cauv_global& current();
		
		//static void set_mailbox(SpreadMailbox* socket);
		
		void send_trace(const string& msg) const;
		void send_error(const string& msg) const;

		static void trace(const string& msg);
		static void error(const string& msg);
		static void error(const string& exception_type, exception& e);
		
        static void print_logo(char* start, char* end, const string& module_name);
		static void print_module_header(const string& module_name);
	
    private:
		static cauv_global* m_current;
		cauv_global();

	protected:
		
};

#endif//__CAUV_GLOBAL_H__
