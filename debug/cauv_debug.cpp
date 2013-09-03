/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include "cauv_debug.h"

#include <iostream>

#include <boost/noncopyable.hpp>

#include <ros/console.h>

using namespace cauv;

class LogInitializer {
    public:
    LogInitializer() {
        ros::console::initialize();
        ros::console::initializeLogLocation(&location, "ros.cauv_debug", ros::console::Level::Info);
    }
    ros::console::LogLocation location;
};

Log::Log(const char *filename_,
         const int line_number_,
         const char *func_name_,
         int level_) :
    can_add_space(false),
    filename(filename_),
    line_number(line_number_),
    func_name(func_name_),
    level(level_) {

};

Log& Log::operator<<(const char *t) {
    if (can_add_space && can_add_space_before(t)) {
        stream << " ";
    }
    stream << t;
    can_add_space = can_add_space_after(t);
    return *this;
};

Log& Log::operator<<(std::string t) {
    if (can_add_space && can_add_space_before(t)) {
        stream << " ";
    }
    stream << t;
    can_add_space = can_add_space_after(t);
    return *this;
};

bool Log::can_add_space_before(std::string t) {
    if (t.empty()) {
        return false;
    }
    switch(t[0]) {
        case ' ': case '.': case '%': case ':': case ';':
        case '=': case ')': case ']': case '}': case '\'':
        case '"':
            return false;
        default:
            return true;
    }
}

bool Log::can_add_space_after(std::string t) {
    if (t.empty()) {
        return false;
    }
    switch(*t.rbegin()) {
        case ' ':
        case '=': case '(': case '[': case '{': case '\'':
        case '"':
            return false;
        default:
            return true;
    }
}

Log::~Log() {
    static LogInitializer log;
    ros::console::Level l;
    if (level >= 0) {
        l = ros::console::Level::Debug;
    } else if (level == -1) {
        l = ros::console::Level::Info;
    } else if (level == -2) {
        l = ros::console::Level::Warn;
    } else if (level == -3) {
        l = ros::console::Level::Error;
    } else {
        l = ros::console::Level::Fatal;
    }
    ros::console::print(NULL, log.location.logger_, l, stream, filename, line_number, func_name);
};
