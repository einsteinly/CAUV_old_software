#ifndef __CAUV_GLOBAL_H__
#define __CAUV_GLOBAL_H__

#include <string>

namespace cauv{

class cauv_global
{
	public:
		static cauv_global& current();
		
		//static void set_mailbox(SpreadMailbox* socket);
		
		void send_trace(const std::string& msg) const;
		void send_error(const std::string& msg) const;
		
        static void print_logo(char* start, char* end, const std::string& module_name);
		static void print_module_header(const std::string& module_name);
	
    private:
		static cauv_global* m_current;
		cauv_global();

	protected:
		
};

} // namespace cauv

#endif//__CAUV_GLOBAL_H__
