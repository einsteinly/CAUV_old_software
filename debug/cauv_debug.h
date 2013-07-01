/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_DEBUG_H__
#define __CAUV_DEBUG_H__

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/noncopyable.hpp>

#include <utility/bash_cout.h>

#define CAUV_DEBUG_MUTEXES
#define CAUV_DEBUG_PRINT_THREAD

#ifndef CAUV_DEBUG_LEVEL
#define CAUV_DEBUG_LEVEL 1
#endif

// Forward Declarations
#if defined(CAUV_DEBUG_MUTEXES) || defined(CAUV_DEBUG_PRINT_THREAD)
namespace boost{
    class mutex;
    template<typename T>
    class unique_lock;
} // namespace boost
#endif
namespace boost { namespace program_options { class options_description; } }
namespace boost { namespace program_options { class positional_options_description; } }


/* usage:
 *   info() << stuff;    // prints (cout) and logs [HH:MM:SS.fffffff] stuff 
 *   error() << stuff;   // prints (cerr) and logs [HH:MM:SS.fffffff] ERROR: stuff
 *   warning() << stuff; // prints (cerr) and logs [HH:MM:SS.fffffff] WARNING: stuff
 *
 *   debug() << "stuff";   // prints (cout) and logs [HH:MM:SS.fffffff] stuff
 *   debug(0) << "stuff";  //
 *
 * The level at which debug statements will be printed can be controlled,
 * statements are printed when the current debug level is greater than (or equal
 * to) the level set in the debug statement:
 *   debug(1)  // prints when debug level >= 1 (default)
 *   debug()   // same as debug(1)
 *   debug(-3) // prints when debug level >= -3
 *
 * The debug level can be determined based on command line options by using the
 * debug::parseOptions static member function, or the debug::setLevel static
 * member function
 *   debug::setLevel(4) // prevent printing of debug messages with level > 4
 *
 * if CAUV_NO_DEBUG is defined, debug() statements do not print OR LOG
 * anything, in this case debug::parseOptions is a no-op
 *
 * if CAUV_DEBUG_PRINT_THREAD is defined, the timestamp has the thread id appended:
 *   [HH:MM:SS.fffffff T=0x123456]
 *
 * A newline is added automatically, spaces are added automatically between
 * successive stuffs which don't already have spaces at the ends:
 *   info() << "1" << 3 << "test " << 4; // prints "[HH:MM:SS.fffffff] 1 3 test 4\n"
 *
 * Some punctuation is also treated magically when considering whether to add
 * spaces:
 *   error() << "foo=" << 3 << ", but ("<< 7 << ">" << 4 << ")";
 *   -> "[HH:MM:SS.fffffff] ERROR: foo=3, but (7 > 4)\n"
 *
 * Timestamps are local time
 *
 */

class SmartStreamBase : public boost::noncopyable
{
    public:
        typedef std::ostream stream_t;
        typedef stream_t& (*manip_t)(stream_t&);
        
        SmartStreamBase(std::ostream& stream,
                        BashColour::e col = BashColour::None,
                        bool print=true);

        virtual ~SmartStreamBase();

        static void setLevel(int debug_level);
        static void setProgramName(const std::string&);
        static void setLogfileName(const std::string&);
        static void setLogDirName(const std::string&);
        static void addOptions(boost::program_options::options_description& desc,
                               boost::program_options::positional_options_description& /*pos*/);

        static int parseOptions(int, char**);


    protected:
        struct Settings{
            int debug_level;
            std::string program_name;
            std::string logfile_name;
            std::string logdir_name;
            bool changed;
        };

        // helper functions for derived classes
        
        template<typename T>
        inline void _appendToStuffs(T const& a)
        {
            // convert to a string
            std::stringstream s;

            // apply any manipulators first
            for(std::vector<manip_t>::const_iterator i = m_manipulators.begin(), iend = m_manipulators.end(); i != iend; ++i)
                s << **i;
            m_manipulators.clear();

            s << a;

            // push this onto the vector of things to print
            m_stuffs.push_back(s.str());
        }

        inline void _appendToManips(manip_t a)
        {
            m_manipulators.push_back(a);
        }

        // stuff to print
        std::vector< std::string > m_stuffs;
        std::vector< manip_t > m_manipulators;

        static Settings defaultSettings();
        // initialise on first use
        static Settings& settings();

        std::string m_prefix;
    private:
        void printToStream(std::ostream& os);

        // space is added between strings s1 s2 if:
        //   mayAddSpaceNext(s1) == true && mayAddSpaceNow(s2) == true
        static bool mayAddSpaceNext(const std::string& s);
        static bool mayAddSpaceNow(const std::string& s);
        
        // per-thread:
        static bool& recursive();

        // initialise on first use
        static std::ofstream& logFile();
        
        // return the locale to use for printing
        static std::locale const& getTheLocale();

#if defined(CAUV_DEBUG_MUTEXES)
        typedef boost::mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;

        // protect each stream to make sure output doesn't become garbled
        static mutex_t& getMutex(std::ostream& s);
#endif
        std::ostream& m_stream;
        BashColour::e m_col;
        bool m_print;
};

#if !defined(CAUV_NO_DEBUG)
struct debug : public SmartStreamBase
{
    debug(int level=1);
    virtual ~debug();

    template<typename T>
    debug& operator<<(T const& a)
    {
        if(settings().debug_level >= m_level)
        {
            _appendToStuffs<T>(a);
        }
        return *this;
    }

    /* must handle manipulators (e.g. endl) separately:
     */
    debug& operator<<(manip_t manip);
    
    private:
        int m_level;
};
#else
struct debug : boost::noncopyable
{
    debug(int level=1){ }
    virtual ~debug(){ }

    static void setLevel(int){ }
    static void setProgramName(const std::string&){ }
    static void setLogfileName(const std::string&){ }

    template<typename T>
    debug const& operator<<(T const&) const {
        return *this;
    }

    typedef std::ostream stream_t;
    typedef stream_t& (*manip_t)(stream_t&);
    debug const& operator<<(manip_t manip) const {
        return *this;
    }

    static int parseOptions(int, char**){ return 0; }
};
#endif

struct error : public SmartStreamBase
{
    error();
    virtual ~error();

    template<typename T>
    error& operator<<(T const& a)
    {
        _appendToStuffs<T>(a);
        return *this;
    }

    /* must handle manipulators (e.g. endl) separately:
     */
    error& operator<<(manip_t manip);
};

struct warning : public SmartStreamBase
{
    warning();
    virtual ~warning();

    template<typename T>
    warning& operator<<(T const& a)
    {
        _appendToStuffs<T>(a);
        return *this;
    }

    /* must handle manipulators (e.g. endl) separately:
     */
    warning& operator<<(manip_t manip);
};

struct info : public SmartStreamBase
{
    info();
    virtual ~info();

    template<typename T>
    info& operator<<(T const& a)
    {
        _appendToStuffs<T>(a);
        return *this;
    }

    /* must handle manipulators (e.g. endl) separately:
     */
    info& operator<< (manip_t manip)
    {
        _appendToManips(manip);
        return *this;
    }
};

#endif // ndef __CAUV_DEBUG_H__

