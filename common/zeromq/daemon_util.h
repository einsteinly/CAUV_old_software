#ifndef DAEMON_UTIL_H_
#define DAEMON_UTIL_H_

#include <string>
#include <stdexcept>

class file_error : public std::runtime_error {
    public:
    explicit file_error(const std::string &what_arg) :
        std::runtime_error(what_arg) {};
};

void daemonize(void);

//create a series of directories, like mkdir -p
void create_path(std::string dir_path);

bool get_lock_file(std::string lockfile);

#endif
