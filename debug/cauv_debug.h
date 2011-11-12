/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_DEBUG_H__
#define __CAUV_DEBUG_H__

#include <iostream>
#include <ostream>
#include <sstream>
#include <list>

#include <boost/foreach.hpp>
#include <boost/utility.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/program_options.hpp>

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

namespace cauv{
class CauvNode;
} // namespace cauv

#include <utility/bash_cout.h>


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
        static void setCauvNode(cauv::CauvNode*);
        static void setProgramName(std::string const&);
        static void setLogfileName(std::string const&);
        static void setLogDirName(std::string const&);
        static void addOptions(boost::program_options::options_description& desc,
                               boost::program_options::positional_options_description& /*pos*/);

        static int parseOptions(int, char**);


    protected:
        struct Settings{
            int debug_level;
            cauv::CauvNode* cauv_node;
            std::string program_name;
            std::string logfile_name;
            std::string logdir_name;
        };

        // helper functions for derived classes
        
        template<typename T>
        inline void _appendToStuffs(T const& a)
        {
            // convert to a string
            std::stringstream s;

            // apply any manipulators first
            BOOST_FOREACH (manip_t m, m_manipulators)
                s << *m;
            m_manipulators.clear();

            s << a;

            // push this onto the list of things to print
            m_stuffs.push_back(s.str());
        }

        inline void _appendToManips(manip_t a)
        {
            m_manipulators.push_back(a);
        }

        // stuff to print
        std::list< std::string > m_stuffs;
        std::list< manip_t > m_manipulators;

        virtual void printPrefix(std::ostream&);
        // can't forward declare enums...
        virtual int debugType() const;

        // initialise on first use
        static Settings& settings();

    private:
        void printToStream(std::ostream& os);

        // space is added between strings s1 s2 if:
        //   mayAddSpaceNext(s1) == true && mayAddSpaceNow(s2) == true
        static bool mayAddSpaceNext(std::string const& s);
        static bool mayAddSpaceNow(std::string const& s);
        
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

        static boost::scoped_ptr<mutex_t> m_mutex;
        boost::scoped_ptr<lock_t> m_lock;
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
    
    virtual void printPrefix(std::ostream&);
    virtual int debugType() const;

    private:
        int m_level;
};
#else
struct debug : boost::noncopyable
{
    debug(int level=1){ }
    virtual ~debug(){ }

    static void setLevel(int){ }
    static void setCauvNode(cauv::CauvNode*){ }
    static void setProgramName(std::string const&){ }
    static void setLogfileName(std::string const&){ }

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
    
    virtual void printPrefix(std::ostream&);
    virtual int debugType() const;
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
    
    virtual void printPrefix(std::ostream&);
    virtual int debugType() const;
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
    
    virtual void printPrefix(std::ostream&);
    virtual int debugType() const;
};

#endif // ndef __CAUV_DEBUG_H__

