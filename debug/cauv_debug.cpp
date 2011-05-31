#include "cauv_debug.h"

#include <iostream>
#include <ostream>
#include <fstream>
#include <list>
#include <sstream>
#include <cctype>
#include <ctime>

#include <boost/date_time.hpp>
#include <boost/utility.hpp>
#include <boost/variant.hpp>
#include <boost/program_options.hpp>

#if defined(CAUV_DEBUG_MUTEXES) || defined(CAUV_DEBUG_PRINT_THREAD)
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#endif
#include <boost/thread/locks.hpp>
#include <boost/thread/tss.hpp>

#include <utility/bash_cout.h>
#include <utility/string.h>
#include <generated/messages_fwd.h>
#include <generated/messages_messages.h>
#include <common/cauv_utils.h>
#include <common/cauv_node.h>

using namespace cauv;

// static data member definition:
#if defined(CAUV_DEBUG_MUTEXES)
boost::scoped_ptr<boost::mutex> SmartStreamBase::m_mutex(new boost::mutex);
#endif


SmartStreamBase::SmartStreamBase(std::ostream& stream, BashColour::e col, bool print)
    : m_stuffs(), m_manipulators(),
#if defined(CAUV_DEBUG_MUTEXES)
      // TODO: this seems to be redundant locking? we already hold a per-stream
      // lock whilst printing, why do we need to stop multiple instances of
      // SmartStreamBase existing at the same time? (which is effectively what
      // this next line does)
      //m_lock(new lock_t(*m_mutex)),
#endif
      m_stream(stream),
      m_col(col),
      m_print(print)
{
}

SmartStreamBase::~SmartStreamBase()
{
    // print the contents of this temporary object when it's destroyed,
    if(m_print)
        printToStream(m_stream);
    // always log
    printToStream(logFile());
    // TODO: possible nastiness when setCauvNode is called from a different
    // thread while this is going on
    // if there's a cauv_node set, and this isn't a recursive call, send debug messages
    /*if(m_print && (debugType() != int(DebugType::Debug)) && settings().cauv_node && !recursive()){
        recursive() = true;
        std::ostringstream oss;
        oss << settings().program_name << ":";
        printToStream(oss);
        std::cout << "debug() sending debug message:" << oss.str() << std::endl;
        settings().cauv_node->send(
            boost::make_shared<DebugMessage>((DebugType::e)debugType(), oss.str()),
            Spread::service(SAFE_MESS)
        );
        recursive() = false;
    }*/
}

void SmartStreamBase::setLevel(int debug_level)
{
    settings().debug_level = debug_level;
}

void SmartStreamBase::setCauvNode(CauvNode* n)
{
    settings().cauv_node = n;
}

void SmartStreamBase::setProgramName(std::string const& n)
{
    settings().program_name = n;
}

void SmartStreamBase::setLogfileName(std::string const& n)
{
    settings().logfile_name = n;
}

void SmartStreamBase::printPrefix(std::ostream&)
{
}
int SmartStreamBase::debugType() const
{
    return DebugType::Info;
}

// initialise on first use
SmartStreamBase::Settings& SmartStreamBase::settings(){
    static Settings s = {
        CAUV_DEBUG_LEVEL,
        NULL,
        "unknown",
        ""
    };
    return s;
}

void SmartStreamBase::printToStream(std::ostream& os)
{
    #ifdef CAUV_DEBUG_MUTEXES
        lock_t l(getMutex(os));
    #endif
    // setting locales on streams seems to cause all sorts of extremely
    // nasty nastiness (crashing memcheck...), so, use a temporary
    // stream that nothing else is going to interfere with to get at
    // the time in the format that we want:
    std::ostringstream oss;
    oss.imbue(getTheLocale());

    // add timestamp at start of each line:
    if(m_stuffs.size())
    {
        boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();
        oss << m_col << "[" << t;

        #ifdef CAUV_DEBUG_PRINT_THREAD
            oss << " T=" << boost::this_thread::get_id();
        #endif

        // add defined prefix to each line
        oss << "] ";
    }

    printPrefix(oss);

    // make sure oss is stringised so that locale nastiness is all over
    // and done with
    os << oss.str();

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
bool SmartStreamBase::mayAddSpaceNext(std::string const& s){
    if(isspace(*s.rbegin()))
        return false;
    if(s.rfind("\033[") != std::string::npos &&
       (s.size() < 8 || s.rfind("\033[") > s.size() - 8) &&
       *s.rbegin() == 'm')
        return false;
    switch(*s.rbegin()){
        case '=':
        case '(': case '[': case '{':
        case '"': case '\'':
            return false;
        default:
            return true;
    }
}

bool SmartStreamBase::mayAddSpaceNow(std::string const& s){
    if(isspace(s[0]))
        return false;
    if(s.find("\033[") == 0)
        return false;
    switch(s[0]){
        case ',': case '.':
        case '=':
        case ')': case ']': case '}':
        case '"': case '\'':
            return false;
        default:
            return true;
    }
}

bool& SmartStreamBase::recursive(){
    static boost::thread_specific_ptr<bool> r;
    if(!r.get())
        r.reset(new bool(false));
    return *r.get();
}

std::string const& logFilePrefix(){
    using boost::posix_time::microsec_clock;
    // abuse thread-safety of static initialisation (compile with it enabled!)
    static std::string the_prefix = MakeString(
        std::locale(
            std::cout.getloc(),
            new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%s")
        )
    )   << microsec_clock::local_time()
        << "-";
    return the_prefix;
}

// initialise on first use
std::ofstream& SmartStreamBase::logFile()
{
    static std::ofstream lf;
    static std::string lfn = settings().logfile_name;
    bool name_did_become_set = false;

    if(settings().logfile_name.size() == 0){
        if(settings().program_name.size())
            settings().logfile_name = logFilePrefix() + settings().program_name + ".log";
        else
            settings().logfile_name = logFilePrefix() + "unknown.log";
    }else if(settings().logfile_name ==  logFilePrefix() + "unknown.log"){
        // program name is now set, now we have a name to use (note that
        // subsequent changes to the program name will not be picked up)
        name_did_become_set = true;
        settings().logfile_name = logFilePrefix() + settings().program_name + ".log";
    }
    if(lfn != settings().logfile_name)
    {
        if(lf.is_open())
            lf.close();
        lfn = settings().logfile_name;
    }
    if(!lf.is_open())
    {
        lf.open(lfn.c_str(), std::ios::out | std::ios::app);
        if(name_did_become_set)
            lf << "\n\n----------\n" << settings().program_name
               << " (continued from " << logFilePrefix() + "unknown.log)" << std::endl; 
        else
            lf << "\n\n----------\n" << settings().program_name << " Started" << std::endl;
    }
    return lf;
}

std::locale const& SmartStreamBase::getTheLocale(){
    static std::locale the_locale = std::locale(
        std::cout.getloc(),
        new boost::posix_time::time_facet("%H:%M:%s")
    );
    return the_locale;
}

#if defined(CAUV_DEBUG_MUTEXES)
// protect cout & cerr to make sure output doesn't become garbled
boost::mutex& _getMutex(std::ostream& s){
    typedef boost::shared_ptr<boost::mutex> mutex_ptr;
    typedef std::map<void*, mutex_ptr> map_t;
    static map_t mutex_map;
    static boost::mutex map_mutex;
        
    mutex_ptr pm;
    CAUV_LOCK(map_mutex)
    {
        map_t::iterator i = mutex_map.find(&s);
        if (i != mutex_map.end()) {
            pm = i->second;
        }
        else {
            pm = mutex_map[&s] = boost::make_shared<boost::mutex>();
        }
    }
    return *pm;
}
boost::mutex& SmartStreamBase::getMutex(std::ostream& s){
    if(s == std::cout || s == std::clog)
        return _getMutex(std::cerr);
    else
        return _getMutex(s);
}
#endif // defined(CAUV_DEBUG_MUTEXES)


#if !defined(CAUV_NO_DEBUG)
debug::debug(int level)
    : SmartStreamBase(std::cout, BashColour::Cyan),
      m_level(level)
{
}

debug::~debug()
{
}

/* must handle manipulators (e.g. endl) separately:
 */
debug& debug::operator<<(manip_t manip)
{
    if(settings().debug_level >= m_level)
    {
        _appendToManips(manip);
    }
    return *this;
}

void debug::printPrefix(std::ostream&)
{
}
int debug::debugType() const
{
    return DebugType::Debug;
}

#endif // !defined(CAUV_NO_DEBUG)



/*** error() << ***/ 
error::error()
    : SmartStreamBase(std::cerr,
                      BashColour::Red)
{
}

error::~error()
{
}

error& error::operator<<(manip_t manip)
{
    _appendToManips(manip);
    return *this;
}

void error::printPrefix(std::ostream& os)
{
    os << BashIntensity::Bold << "ERROR: " << BashIntensity::Normal;
}
int error::debugType() const
{
    return DebugType::Error;
}


/*** warning() << ***/ 
warning::warning()
    : SmartStreamBase(std::cerr, BashColour::Brown)
{
}

warning::~warning()
{
}

warning& warning::operator<<(manip_t manip)
{
    _appendToManips(manip);
    return *this;
}

void warning::printPrefix(std::ostream& os)
{
    os << BashIntensity::Bold << "WARNING: " << BashIntensity::Normal;
}
int warning::debugType() const
{
    return DebugType::Warning;
}


/*** info() << ***/
info::info() : SmartStreamBase(std::cout)
{
}

info::~info()
{
}

void info::printPrefix(std::ostream&)
{
}
int info::debugType() const
{
    return DebugType::Info;
}

