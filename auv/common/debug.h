#ifndef __CAUV_DEBUG_H__
#define __CAUV_DEBUG_H__

#include <iostream>
#include <ostream>
#include <fstream>
#include <list>
#include <sstream>
#include <cctype>
#include <ctime>

#define DEBUG_MUTEX_OUTPUT
#define DEBUG_PRINT_THREAD

#if defined(DEBUG_MUTEX_OUTPUT) || defined(DEBUG_PRINT_THREAD)
#include <boost/thread.hpp>
#endif


/* usage:
 *   debug(1) << stuff; // logs "[HH:MM:SS] stuff", if (DEBUG >= 1)
 *   debug() << stuff << more_stuff << "thing"; // same as debug(1) << ...
 *   debug(3) << stuff; // only print when DEBUG >= 3
 *
 *   if DEBUG is not defined, debug() << ... statements do not print OR LOG
 *   anything
 *
 *   info() << stuff; // prints (cout) and logs [HH:MM:SS] stuff
 *
 *   error() << stuff; // prints (cerr) and logs [HH:MM:SS] ERROR: stuff
 *
 *
 * A newline is added automatically, spaces are added automatically between
 * successive stuffs which don't already have spaces at the ends
 *
 *   info() << "1" << 3 << "test " << 4; // prints "[HH:MM:SS] 1 3 test 4\n"
 *
 */

/* bash colours */
enum bash_colour_e{
    no_colour = 0,
    black = 30,
    red   = 31,
    green = 32,
    brown = 33,
    blue  = 34,
    purple = 35,
    cyan  = 36,
    light_grey = 37,
    dark_grey   = 130,
    light_red   = 131,
    light_green = 132,
    yellow      = 133,
    light_blue  = 134,
    light_purple = 135,
    light_cyan  = 136,
    white       = 137
 };
 enum bash_control_e{
    reset_colour = 0,
    set_bold = 1
 };


template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, bash_control_e const& c){
    switch(c){
        case reset_colour: os << "\E[m"; break;
        case set_bold: os << "\E[1m"; break;
    }
    return os;
}

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, bash_colour_e const& c){
    if(c == no_colour)
        os << reset_colour;
    else if(int(c) < 100)
        os << "\E[0;" << int(c) << "m";
    else 
        os << "\E[1;" << int(c)-100 << "m";
    return os;
}


class NonCopyable
{
    public:
        NonCopyable(){}
    private:
        NonCopyable(NonCopyable const&){}
        NonCopyable& operator=(NonCopyable const&){return *this;}
};

class SmartStreamBase: NonCopyable
{
    public:
        SmartStreamBase(std::ostream& stream,
                        const char* prefix = "",
                        bash_colour_e col = no_colour,
                        bool print=true)
            : m_stream(stream), m_prefix(prefix), m_col(col), m_print(print)
        {
        }

        ~SmartStreamBase()
        {
            // print the contents of this temporary object when it's destroyed,
            if(m_print)
                printToStream(m_stream);
            // always log
            printToStream(logFile());
        }

    protected:
        // stuff to print
        std::list<std::string> m_stuffs;

    private:
        void printToStream(std::ostream& os)
        {
            #ifdef DEBUG_MUTEX_OUTPUT
                boost::lock_guard<boost::recursive_mutex> l(_getMutex(os));
            #endif

            // add timestamp at start of each line:
            time_t raw_time;
            struct tm* gm_time;
            char buffer[80] = "";

            if(m_stuffs.size())
            {
                os << m_col;

                time(&raw_time);
                gm_time = gmtime(&raw_time);
                strftime(buffer, 80, "[%H:%M:%S", gm_time);

                #ifdef DEBUG_PRINT_THREAD
                    os << buffer << " T=" << boost::this_thread::get_id() << "] ";
                #else
                    os << buffer << "] ";
                #endif

                // add defined prefix to each line
                os <<  m_prefix;
            }
            
            // add spaces between consecutive items that do not have spaces
            std::list<std::string>::const_iterator i = m_stuffs.begin();
            bool add_space = false;
            while(i != m_stuffs.end())
            {
                if(i->size())
                {
                    if(add_space && !isspace(*i->begin()))
                        os << " ";
                    os << *i;
                    // maybe add a space next time
                    if(!isspace(*i->rbegin()) && i->rfind("[m") != (i->size()-2))
                    {
                        add_space = true; 
                    }
                }
                i++;
            }
            // if anything was printed, add a newline, reset colour
            if(i != m_stuffs.begin())
                os << reset_colour << std::endl; 
        }
        
        // initialise on first use
        static std::ofstream& logFile()
        {
            static std::ofstream lf;
            if(!lf.is_open())
            {
                lf.open("log.out", std::ios::out | std::ios::app);
                lf << "\n\n----------\nProgram Started" << std::endl;
            }
            return lf;
        }

        // protect cout & cerr to make sure output doesn't become garbled
        static boost::recursive_mutex& _getMutex(std::ostream& s){
            typedef boost::shared_ptr<boost::recursive_mutex> mutex_ptr;
            typedef std::map<void*, mutex_ptr> map_t;
            static map_t mutex_map;
            map_t::iterator i = mutex_map.find(&s);
            if(i != mutex_map.end())
                return *i->second;
            else
                mutex_map[&s] = mutex_ptr(new boost::recursive_mutex);
            return *mutex_map[&s];
        }
        
        std::ostream& m_stream;
        const char* m_prefix;
        bash_colour_e m_col;
        bool m_print;
};

#if defined(DEBUG)
struct debug: SmartStreamBase
{
    debug(int level=1)
        : SmartStreamBase(std::cout, "", brown, true), m_level(level)
    {
    }

    template<typename T>
    debug& operator<<(T const& a)
    {
        if(int(DEBUG) <= m_level)
        {
            // convert to a string
            std::stringstream s;
            s << a; 

            // push this onto the list of things to print 
            m_stuffs.push_back(s.str());
        }
        return *this;
    }

    /* must handle manipulators (e.g. endl) separately:
     */
    debug& operator<<(std::ostream& (*manip)(std::ostream&))
    {
        if(int(DEBUG) <= m_level)
        {
            // apply to a string
            std::stringstream s;
            s << manip;
            
            // and push it onto the list of things to print
            m_stuffs.push_back(s.str());
        }
        return *this;
    }

    private:
        int m_level;
};
#else
struct debug: NonCopyable
{
    debug(int level=1)
    {
        level = level; // suppress warning about unused parameter
    }

    template<typename T>
    debug const& operator<<(T const&) const
    {
        return *this;
    }

    /* must handle manipulators (e.g. endl) separately:
     */
    debug const& operator<<(std::ostream& (*manip)(std::ostream&)) const
    {
        manip = manip; // suppress warning about unused parameter 
        // do nothing with the manipulator
        return *this;
    }
};
#endif

struct error: SmartStreamBase
{
    error()
        : SmartStreamBase(std::cerr, "ERROR: ", red)
    {
    }

    template<typename T>
    error& operator<<(T const& a)
    {
        // convert to a string
        std::stringstream s;
        s << a; 

        // push this onto the list of things to print 
        m_stuffs.push_back(s.str());
        return *this;
    }

    /* must handle manipulators (e.g. endl) separately:
     */
    error& operator<<(std::ostream& (*manip)(std::ostream&))
    {
        // apply to a string
        std::stringstream s;
        s << manip;
        
        // and push it onto the list of things to print
        m_stuffs.push_back(s.str());
        return *this;
    }
};

struct info: SmartStreamBase
{
    info()
        : SmartStreamBase(std::cout, "")
    {
    }

    template<typename T>
    info& operator<<(T const& a)
    {
        // convert to a string
        std::stringstream s;
        s << a; 

        // push this onto the list of things to print 
        m_stuffs.push_back(s.str());
        return *this;
    }

    /* must handle manipulators (e.g. endl) separately:
     */
    info& operator<<(std::ostream& (*manip)(std::ostream&))
    {
        // apply to a string
        std::stringstream s;
        s << manip;
        
        // and push it onto the list of things to print
        m_stuffs.push_back(s.str());
        return *this;
    }
};

#endif // ndef __CAUV_DEBUG_H__

