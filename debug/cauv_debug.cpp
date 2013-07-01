/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "cauv_debug.h"

#include <iostream>
#include <ostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cctype>
#include <ctime>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/noncopyable.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#if defined(CAUV_DEBUG_MUTEXES) || defined(CAUV_DEBUG_PRINT_THREAD)
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>
#endif
#include <boost/thread/tss.hpp>

#include <utility/bash_cout.h>
#include <utility/string.h>
#include <utility/lock.h>

using namespace cauv;

SmartStreamBase::SmartStreamBase(std::ostream& stream, BashColour::e col, bool print)
    : m_stuffs(), m_manipulators(),
      m_stream(stream),
      m_col(col),
      m_print(print)
{
}

SmartStreamBase::~SmartStreamBase()
{
    // print the contents of this temporary object when it's destroyed,
    if(m_stuffs.size()) {
        if(m_print)
            printToStream(m_stream);
        // always log
        printToStream(logFile());
    }
}

void SmartStreamBase::setLevel(int debug_level)
{
    settings().debug_level = debug_level;
    settings().changed = true;
}

void SmartStreamBase::setProgramName(const std::string& n)
{
    //this can get set from multiple places. just go with the first one
    static bool set = false;
    if (!set) {
        settings().program_name = n;
        settings().changed = true;
        set = true;
    }
}

void SmartStreamBase::setLogfileName(const std::string& n)
{
    settings().logfile_name = n;
    settings().changed = true;
}

void SmartStreamBase::setLogDirName(const std::string& n)
{
    std::string new_logdir = n;
    if(new_logdir.size()){
        if(*new_logdir.rbegin() != '/')
            new_logdir += "/";
    }else{
        new_logdir = "./";
    }
    settings().logdir_name = new_logdir;
    settings().changed = true;
}

SmartStreamBase::Settings SmartStreamBase::defaultSettings() {
    //performance is better without stdio sync
    //This is just here because it'll be called once
    std::ios_base::sync_with_stdio(false);

    Settings s;
    char *debugLevelStr = getenv("CAUV_DEBUG_LEVEL");
    if (debugLevelStr) {
        s.debug_level = atoi(debugLevelStr);
    } else {
        s.debug_level = CAUV_DEBUG_LEVEL;
    }
    char *logdir = getenv("CAUV_LOG_DIR");
    if (logdir) {
        s.logdir_name = logdir;
    } else {
        // /var/tmp is persistent across reboots and world-writable.
        // /var/log, alas, is not
        s.logdir_name = "/var/tmp/cauv_log";
    }
    if (*s.logdir_name.rbegin() != '/') {
        s.logdir_name += "/";
    }
    s.program_name = "unknown";
    s.changed = true;
    return s;
}

// initialise on first use
SmartStreamBase::Settings& SmartStreamBase::settings(){
    static Settings s = defaultSettings();
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

    oss << m_prefix;

    // make sure oss is stringised so that locale nastiness is all over
    // and done with
    os << oss.str();

    // add spaces between consecutive items that do not have spaces
    std::vector<std::string>::const_iterator i = m_stuffs.begin();
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
bool SmartStreamBase::mayAddSpaceNext(const std::string& s){
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

bool SmartStreamBase::mayAddSpaceNow(const std::string& s){
    if(isspace(s[0]))
        return false;
    if(s.find("\033[") == 0)
        return false;
    switch(s[0]){
        case ',': case '.': case '%': case ':': case ';':
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

const std::string& logFilePrefix(){
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

    if (settings().changed) {
        std::string new_lfn;
        settings().changed = false;
        new_lfn = settings().logdir_name + logFilePrefix() + settings().program_name + ".log";
        if(settings().logfile_name != new_lfn) {
            bool new_file = true;
            try {
                boost::filesystem::create_directories(settings().logdir_name);
                if (lf.is_open()) {
                    try {
                        boost::filesystem::rename(settings().logfile_name, new_lfn);
                        lf << "\n\n---------\n renamed from " << settings().logfile_name << "\n";
                    } catch (boost::filesystem::filesystem_error &e) {
                        std::cerr << BashColour::Brown << "Log: Could not rename File: " << e.what() << BashControl::Reset << "\n";
                        lf.close();
                        new_file = false;
                    }
                }
                if (!lf.is_open()) {
                    lf.open(new_lfn.c_str(), std::ios::out | std::ios::app);
                    if (new_file) {
                        lf << "\n\n----------\n" << settings().program_name << " Started" << std::endl;
                    } else {
                        lf << "\n\n----------\n continued from " << settings().logfile_name << "\n";
                    }
                }
                try {
                    std::string symlink(settings().logdir_name + settings().program_name + ".log");
                    try {
                        boost::filesystem::remove(symlink);
                    } catch (boost::filesystem::filesystem_error &e) {/*doesn't matter*/};
                    boost::filesystem::create_symlink(new_lfn, symlink);
                } catch (boost::filesystem::filesystem_error &e) {
                    std::cerr << "Log: Could not symlink logfile: " << e.what() << "\n";
                }
                settings().logfile_name = new_lfn;
            } catch (boost::filesystem::filesystem_error &e) {
                std::cerr << BashColour::Brown << "Log: Could not create log directory: " << e.what() << BashControl::Reset << "\n";
            }
        }
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

void SmartStreamBase::addOptions(boost::program_options::options_description& desc,
                                 boost::program_options::positional_options_description&/*pos*/)
{
    namespace po = boost::program_options;
    desc.add_options ()
        ("verbose,v", po::value<int>()->implicit_value(1)->notifier(SmartStreamBase::setLevel), "Set the verbosity of debug messages")
        ("logdir,L", po::value<std::string>()->notifier(SmartStreamBase::setLogDirName), "The directory to log debug messages to")
    ;
};

int SmartStreamBase::parseOptions(int argc, char** argv) {
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    po::positional_options_description pos;
    if(argv && argc) {
        setProgramName(boost::filesystem::path(argv[0]).leaf().native());
    }
    
    addOptions(desc, pos);
    
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).allow_unregistered().options(desc).run(), vm);
    po::notify(vm);
    return 0;
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
        auto i = mutex_map.find(&s);
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
    std::ostringstream oss;
    oss << "D(" << level << "): ";
    m_prefix = oss.str();
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

#endif // !defined(CAUV_NO_DEBUG)



/*** error() << ***/ 
error::error()
    : SmartStreamBase(std::cerr,
                      BashColour::Red)
{
    std::ostringstream oss;
    oss << BashIntensity::Bold << "E: " << BashIntensity::Normal;
    m_prefix = oss.str();
}

error::~error()
{
}

error& error::operator<<(manip_t manip)
{
    _appendToManips(manip);
    return *this;
}

/*** warning() << ***/ 
warning::warning()
    : SmartStreamBase(std::cerr, BashColour::Brown)
{
    std::ostringstream oss;
    oss << BashIntensity::Bold << "W: " << BashIntensity::Normal;
    m_prefix = oss.str();
}

warning::~warning()
{
}

warning& warning::operator<<(manip_t manip)
{
    _appendToManips(manip);
    return *this;
}

/*** info() << ***/
info::info() : SmartStreamBase(std::cout)
{
    m_prefix = "I: ";
}

info::~info()
{
}
