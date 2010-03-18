#ifndef __CAUV_DEBUG_H__
#define __CAUV_DEBUG_H__

#include <iostream>
#include <ostream>
#include <fstream>
#include <list>
#include <sstream>
#include <cctype>
#include <ctime>

#include <boost/date_time.hpp>  
#include <boost/utility.hpp>  

#define DEBUG_MUTEX_OUTPUT
#define DEBUG_PRINT_THREAD

#if defined(DEBUG_MUTEX_OUTPUT) || defined(DEBUG_PRINT_THREAD)
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#endif

#include <common/cauv_utils.h>
#include <common/bash_cout.h>


/* usage:
 *   debug(1) << stuff; // prints & logs "[HH:MM:SS.fffffff] stuff", if (DEBUG <= 1)
 *   debug() << stuff << more_stuff << "thing"; // same as debug(1) << ...
 *   debug(-7) << stuff; // only print when DEBUG <= -7
 *
 *   if DEBUG is not defined, debug() << ... statements do not print OR LOG
 *   anything
 *
 *   info() << stuff; // prints (cout) and logs [HH:MM:SS.fffffff] stuff
 *
 *   error() << stuff; // prints (cerr) and logs [HH:MM:SS.fffffff] ERROR: stuff
 *
 *   warning() << stuff; // prints (cerr) and logs [HH:MM:SS.fffffff] WARNING: stuff
 *
 * if DEBUG_PRINT_THREAD is defined, the timestamp has the thread id appended:
 *   [HH:MM:SS.fffffff T=0x123456] 
 *
 * A newline is added automatically, spaces are added automatically between
 * successive stuffs which don't already have spaces at the ends
 *
 *   info() << "1" << 3 << "test " << 4; // prints "[HH:MM:SS.fffffff] 1 3 test 4\n"
 *
 * Timestamps are GMT
 *
 */




class SmartStreamBase : boost::noncopyable
{
    public:
        SmartStreamBase(std::ostream& stream,
                        std::string prefix = "",
                        BashColour::e col = BashColour::None,
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

        virtual void printPrefix(std::ostream& os)
        {
        }

    private:
        void printToStream(std::ostream& os)
        {
            #ifdef DEBUG_MUTEX_OUTPUT
                boost::lock_guard<boost::recursive_mutex> l(getMutex(os));
            #endif
            boost::posix_time::time_facet* facet = new boost::posix_time::time_facet("%H:%M:%s");
            os.imbue(std::locale(os.getloc(), facet));

            // add timestamp at start of each line:
            if(m_stuffs.size())
            {
                boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();
                os << m_col << "[" << t;

                #ifdef DEBUG_PRINT_THREAD
                    os << " T=" << boost::this_thread::get_id();
                #endif

                // add defined prefix to each line
                os << "] " << m_prefix;
            }
            
            // add spaces between consecutive items that do not have spaces
            std::list<std::string>::const_iterator i = m_stuffs.begin();
            bool add_space = false;
            while(i != m_stuffs.end())
            {
                if(i->size())
                {
                    if(add_space && mayAddSpaceNow(*i))
                        os << " ";
                    os << *i;
                    // maybe add a space next time
                    add_space = mayAddSpaceNext(*i);
                }
                i++;
            }
            // if anything was printed, add a newline, reset colour
            if(i != m_stuffs.begin())
                os << BashControl::Reset << std::endl; 
        }
            
        // space is added between srings s1 s2 if:
        //   mayAddSpaceNext(s1) == true && mayAddSpaceNow(s2) == true
        static bool mayAddSpaceNext(std::string const& s){
            if(isspace(*s.rbegin()))
                return false;
            switch(isspace(*s.rbegin())){
                case ';':
                case '(':
                case '[':
                case '{':
                    return false;
                default:
                    return true;
            }
        }

        static bool mayAddSpaceNow(std::string const& s){
            if(isspace(*s.begin()))
                return false;
            if(s.find("\E[") == 0)
                return false;
            switch(isspace(*s.begin())){
                case '=':
                case ')':
                case ']':
                case '}':
                    return false;
                default:
                    return true;
            }
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

#if defined(DEBUG_MUTEX_OUTPUT)
        // protect cout & cerr to make sure output doesn't become garbled
        static boost::recursive_mutex& _getMutex(std::ostream& s){
            typedef boost::shared_ptr<boost::recursive_mutex> mutex_ptr;
            typedef std::map<void*, mutex_ptr> map_t;
            static map_t mutex_map;
            map_t::iterator i = mutex_map.find(&s);
            if(i != mutex_map.end())
                return *i->second;
            else
                mutex_map[&s] = boost::make_shared<boost::recursive_mutex>();
            return *mutex_map[&s];
        }
        static boost::recursive_mutex& getMutex(std::ostream& s){
            if(s == std::cout)
                return _getMutex(std::cerr);
            else
                return _getMutex(s);
        }
#endif
        
        std::ostream& m_stream;
        std::string m_prefix;
        BashColour::e m_col;
        bool m_print;
};

#if defined(DEBUG)
struct debug : public SmartStreamBase
{
    debug(int level=1) : SmartStreamBase(std::cout, "", BashColour::Cyan), m_level(level)
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
struct debug : boost::noncopyable
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

struct error : public SmartStreamBase
{
    error() : SmartStreamBase(std::cerr,
                              MakeString() << BashIntensity::Bold << "ERROR: " << BashIntensity::Normal,
                              BashColour::Red)
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

struct warning : public SmartStreamBase
{
    warning() : SmartStreamBase(std::cerr,
                                MakeString() << BashIntensity::Bold << "WARNING: " << BashIntensity::Normal,
                                BashColour::Brown)
    {
    }

    template<typename T>
    warning& operator<<(T const& a)
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
    warning& operator<<(std::ostream& (*manip)(std::ostream&))
    {
        // apply to a string
        std::stringstream s;
        s << manip;
        
        // and push it onto the list of things to print
        m_stuffs.push_back(s.str());
        return *this;
    }
};

struct info : public SmartStreamBase
{
    info() : SmartStreamBase(std::cout)
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

