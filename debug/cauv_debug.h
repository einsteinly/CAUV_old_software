/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#pragma once
#include <iostream>
#include <sstream>
#include <string>

#include <boost/noncopyable.hpp>

#ifndef CAUV_SILENCE_DEBUG_LEVEL
#define CAUV_SILENCE_DEBUG_LEVEL 4
#endif

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
 * CAUV_SILENCE_DEBUG_LEVEL is the maximum level of debug for which any output
 * is possible - above that all log statements should be completely optimised
 * away by the compiler.
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

namespace cauv {

class Log : public boost::noncopyable {
    public:
    Log(const char *filename,
        const int line_number,
        const char *func_name,
        const int log_level = 0);
    template <typename T>
    Log& operator<<(T t) {
        if (can_add_space) {
            stream << " ";
        } 
        stream << t;
        can_add_space = true;
        return *this;
    };
    Log& operator<<(const char* t); 
    Log& operator<<(std::string t);
    ~Log();
    private:
    static bool can_add_space_after(std::string s);
    static bool can_add_space_before(std::string s);
    bool can_add_space;
    std::stringstream stream;
    const char * const filename;
    const int line_number;
    const char * const func_name;
    const int level;
};

#define CAUV_LOG_DEBUG(level, values) if (level < CAUV_SILENCE_DEBUG_LEVEL) cauv::Log(__FILE__, __LINE__, __PRETTY_FUNCTION__, level) << values
#define CAUV_LOG_INFO(values)    CAUV_LOG_DEBUG(-1, values)
#define CAUV_LOG_WARNING(values) CAUV_LOG_DEBUG(-2, values)
#define CAUV_LOG_ERROR(values)   CAUV_LOG_DEBUG(-3, values)
#define CAUV_LOG_FATAL(values)   CAUV_LOG_DEBUG(-4, values)

//Needs this since now the loggers are macros they will conflict all over the
//place
#ifdef CAUV_DEBUG_COMPAT
    #define debug(...) if (__VA_ARGS__ -0 < CAUV_SILENCE_DEBUG_LEVEL) cauv::Log(__FILE__, __LINE__, __PRETTY_FUNCTION__, (__VA_ARGS__ - 0))
    #define info() debug(-1)
    #define warning() debug(-2)
    #define error() debug(-3)
#endif

}
