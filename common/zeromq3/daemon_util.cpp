#include "daemon_util.h"

#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>


void daemonize(void) {
    int pid = fork();
    if(pid == 0) {
        if(fork() > 0) {
            exit(0);
        }
    } else if (pid < 0) {
        printf("fork unsuccessful!\n");
        exit(1);
    } else {
        exit(0);
    }
    chdir("/");
    umask(0000);
    freopen("/dev/null","r",stdin);
    freopen("/dev/null","w",stdout);
    freopen("/dev/null","w",stderr);
}

void create_working_directory(std::string dir_path) {
    for(size_t ii = 1; ii < dir_path.size(); ii++) {
        if (dir_path[ii] == '/') {
            dir_path[ii] = '\0';
            int err;
            err = mkdir(dir_path.c_str(), 0775);
            if (!(err == 0 || (err == -1 && errno == EEXIST))) {
                fprintf(stderr,"error creating dir %s: %d %s\n",
                        dir_path.c_str(),errno,strerror(errno));
                throw file_error("Could not create directory!");
            }
            dir_path[ii] = '/';
        }
    }
}

bool get_lock_file(std::string lockfile) {
    int lockfile_fd;
    lockfile_fd = open(lockfile.c_str(), O_RDWR | O_CREAT, 0600);

    if (lockfile_fd < 0) {
        fprintf(stderr,"Could not open lock file %s: %d %s\n",lockfile.c_str(),errno,strerror(errno));
        throw file_error("Could not open lock file!");
    }

    int ret = lockf(lockfile_fd, F_TLOCK, 0);
    if (ret < 0) {
        if (errno == EAGAIN) {
            return false;
        } else {
            fprintf(stderr,"Could not lock file %s: %d %s\n",lockfile.c_str(),errno,strerror(errno));
            throw file_error("error getting lock on lock file (not EAGAIN)");
        }
    } else {
        return true;
    }
}
