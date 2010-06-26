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

#define CAUV_DEBUG_MUTEX_OUTPUT
#define CAUV_DEBUG_PRINT_THREAD

#ifndef CAUV_DEBUG_LEVEL
#define CAUV_DEBUG_LEVEL 1
#endif

#if defined(CAUV_DEBUG_MUTEX_OUTPUT) || defined(CAUV_DEBUG_PRINT_THREAD)
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#endif

#include <common/cauv_utils.h>
#include <common/bash_cout.h>



SmartStreamBase::SmartStreamBase(std::ostream& stream, std::string const& prefix,
                                 BashColour::e col, bool print)
    : m_stuffs(), m_stream(stream), m_prefix(prefix), m_col(col), m_print(print)
{
}

SmartStreamBase::~SmartStreamBase()
{
    // print the contents of this temporary object when it's destroyed,
    if(m_print)
        printToStream(m_stream);
    // always log
    printToStream(logFile());
}

void SmartStreamBase::setLevel(int debug_level)
{
    settings().debug_level = debug_level;
}

int SmartStreamBase::parseOptions(int argc, char** argv)
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    // isn't overloading wonderful...
    desc.add_options()
        ("help", "produce help message")
        ("verbose,v", po::value<int>(), "set the verbosity of debug messages")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
        std::cout << desc;
        return 1;
    }
    if(vm.count("verbose"))
    {
        setLevel(vm["verbose"].as<unsigned int>());
    }
    return 0;
}

void SmartStreamBase::printPrefix(std::ostream&)
{
}

// initialise on first use
SmartStreamBase::Settings& SmartStreamBase::settings(){
    static Settings s = { CAUV_DEBUG_LEVEL };
    return s;
}

void SmartStreamBase::printToStream(std::ostream& os)
{
    #ifdef CAUV_DEBUG_MUTEX_OUTPUT
        boost::lock_guard<boost::recursive_mutex> l(getMutex(os));
    #endif
    // setting locales on streams seems to cause all sorts of extremely
    // nasty nastiness (crashing memcheck...), so, use a temporary
    // stream that nothing else is going to interfere with to get at
    // the time in the format that we want:
    std::ostringstream oss;
    boost::posix_time::time_facet* facet = new boost::posix_time::time_facet("%H:%M:%s");
    oss.imbue(std::locale(oss.getloc(), facet));

    // add timestamp at start of each line:
    if(m_stuffs.size())
    {
        boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();
        oss << m_col << "[" << t;

        #ifdef CAUV_DEBUG_PRINT_THREAD
            oss << " T=" << boost::this_thread::get_id();
        #endif

        // add defined prefix to each line
        oss << "] " << m_prefix;
    }
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
            return false;
        default:
            return true;
    }
}

// initialise on first use
std::ofstream& SmartStreamBase::logFile()
{
    static std::ofstream lf;
    if(!lf.is_open())
    {
        lf.open("log.out", std::ios::out | std::ios::app);
        lf << "\n\n----------\nProgram Started" << std::endl;
    }
    return lf;
}

#if defined(CAUV_DEBUG_MUTEX_OUTPUT)
// protect cout & cerr to make sure output doesn't become garbled
boost::recursive_mutex& SmartStreamBase::_getMutex(std::ostream& s){
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
boost::recursive_mutex& SmartStreamBase::getMutex(std::ostream& s){
    if(s == std::cout)
        return _getMutex(std::cerr);
    else
        return _getMutex(s);
}
#endif


#if !defined(CAUV_NO_DEBUG)
debug::debug(int level)
    : SmartStreamBase(std::cout, "", BashColour::Cyan),
      m_level(level)
{
}

debug::~debug()
{
}

/* must handle manipulators (e.g. endl) separately:
 */
debug& debug::operator<<(std::ostream& (*manip)(std::ostream&))
{
    if(settings().debug_level >= m_level)
    {
        // apply to a string
        std::stringstream s;
        s << manip;

        // and push it onto the list of things to print
        m_stuffs.push_back(s.str());
    }
    return *this;
}
#endif



/*** error() << ***/ 
error::error()
    : SmartStreamBase(std::cerr,
                      MakeString() << BashIntensity::Bold << "ERROR: " << BashIntensity::Normal,
                      BashColour::Red)
{
}

error::~error()
{
}

error& error::operator<<(std::ostream& (*manip)(std::ostream&))
{
    // apply to a string
    std::stringstream s;
    s << manip;

    // and push it onto the list of things to print
    m_stuffs.push_back(s.str());
    return *this;
}



/*** warning() << ***/ 
warning::warning()
    : SmartStreamBase(std::cerr,
                      MakeString() << BashIntensity::Bold << "WARNING: " << BashIntensity::Normal,
                      BashColour::Brown)
{
}

warning::~warning()
{
}

warning& warning::operator<<(std::ostream& (*manip)(std::ostream&))
{
    // apply to a string
    std::stringstream s;
    s << manip;

    // and push it onto the list of things to print
    m_stuffs.push_back(s.str());
    return *this;
}



/*** info() << ***/
info::info() : SmartStreamBase(std::cout)
{
}

info::~info()
{
}

info& info::operator<<(std::ostream& (*manip)(std::ostream&))
{
    // apply to a string
    std::stringstream s;
    s << manip;

    // and push it onto the list of things to print
    m_stuffs.push_back(s.str());
    return *this;
}

