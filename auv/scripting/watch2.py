#!/usr/bin/env python2.7

import sys
import argparse
import imp
import errno
import subprocess
import utils.multitasking
import os
import shlex
import time
import resource
import pwd
from cauv.debug import debug, info, warning, error
from cauv.messaging import debugParseOptions

def load_session(filename):
    for suffix, mode, type in imp.get_suffixes():
        if args.session.endswith(suffix):
            with open(args.session, mode) as session_file:
                return imp.load_module("session", session_file, "", (suffix, mode, type))
    raise SessionNotFound()

def currentUser():
    return pwd.getpwuid(os.getuid())[0]

parser = argparse.ArgumentParser(description = "Start up and monitor CAUV nodes and other programs", add_help = False)

parser.add_argument("--session", "-s", help="session file to use")

args, unknown = parser.parse_known_args()

class SessionNotFound(Exception):
    pass

if args.session:
    session = load_session(args.session)
try:
    arg_group = parser.add_argument_group(title=args.session)
    session.get_arguments(arg_group)
except AttributeError:
    pass
except NameError:
    pass

parser.add_argument("--help",        "-h",  help="print this usage message",                 action='store_true')
parser.add_argument("--daemonize",   "-d",  help="run as a daemon",                          action='store_true')
parser.add_argument("--core-dumps",  "-c",  help="enable core dumps for started processes",  action = 'store_true')
parser.add_argument("--bin-dir",     "-b",  help="binary files directory",                   default = '.')
parser.add_argument("--script-dir",  "-p",  help="script files directory",                   default = '.')
parser.add_argument("--log-dir",     "-l",  help="log directory for files")
parser.add_argument("--kill",        "-k",  help="Kill all processes in session",            nargs='?', type=int, const=15)
parser.add_argument("--user",        "-u",  help="Default user to run processes as",         default=currentUser())

args = parser.parse_args()

if args.help or not args.session:
    parser.print_help()
    sys.exit(1)

class WatchProcess:
    def __init__(self, process):
        self.p = process
        self.pid = None
        self.restart = True

    def exec_proc(self):
        format_dict = {'SDIR': os.path.abspath(args.script_dir), "BDIR": os.path.abspath(args.bin_dir), "D": ''}
        command = self.p.cmds[0].format(**format_dict)
        work_dir = self.p.work_dir.format(**format_dict)
        os.chdir(work_dir)

        if self.p.user is None:
            username = args.user
        else:
            username = self.p.user
        debug('running \'{}\' in directory {} as {}'.format(command, work_dir, username))
        try:
            uid, gid = pwd.getpwnam(username)[2:4]
            if sys.platform == 'darwin':
                # no such thing as a saved id...
                os.setgid(gid)
                os.setuid(uid)
            else:
                os.setresgid(gid,gid,gid)
                os.setresuid(uid,uid,uid)
        except OSError as e:
            if e.errno == errno.EPERM:
                warning("Can't change user. Running as current user instead")
            else:
                raise

        null_fd = os.open("/dev/null", os.O_RDWR)
        os.dup2(null_fd, sys.stdin.fileno())
        os.dup2(null_fd, sys.stdout.fileno())
        os.dup2(null_fd, sys.stderr.fileno())

        argv = shlex.split(command)
        os.execlp(argv[0], *argv)

    def tick(self):
        if self.pid is not None:
            try:
                os.kill(self.pid, 0)
            except OSError as e:
                if e.errno == errno.ESRCH:
                    self.pid = None
            if self.pid is None:
                warning("Process {} died!".format(self.p.name))
                self.restart = self.p.death_callback(self.p)
        if self.pid is None:
            self.pid = self.p.pid_func()
            if self.pid is not None:
                info("found PID {} for process {}".format(self.pid, self.p.name))
        if self.pid is None and self.restart:
            self.pid = utils.multitasking.spawnDaemon(self.exec_proc)

    def kill(self, sig):
        if self.pid is None:
            self.pid = self.p.pid_func()
        if self.pid is not None:
            info("Killing PID {} ({})".format(self.pid, self.p.name))
            try:
                os.kill(self.pid, sig)
            except OSError as e:
                error("Could not kill PID: {}".format(e))
        else:
            warning("Could not find PID for {}".format(self.p.name))


processes = [WatchProcess(p) for p in session.get_processes(args)]

def monitor():
    if args.daemonize:
        os.chdir("/")
        null_fd = os.open("/dev/null", os.O_RDWR)
        os.dup2(null_fd, sys.stdin.fileno())
        os.dup2(null_fd, sys.stdout.fileno())
        os.dup2(null_fd, sys.stderr.fileno())

    while True:
        for p in processes:
            p.tick()
        time.sleep(1)

if args.user is None:
    try:
        args.user = os.environ["SUDO_USER"]
    except KeyError:
        pass

if args.log_dir:
    os.environ["CAUV_LOG_DIR"] = args.log_dir

if args.core_dumps:
    core_pattern = "/tmp/corefiles/%e.%d.%t.%p"
    try:
        try:
            os.mkdir("/tmp/corefiles", 0777)
        except OSError as e:
            if e.errno != errno.EEXIST:
                error("Could not create corefile directory")
                raise
        with open("/proc/sys/kernel/core_pattern", "r") as pattern_file:
            if not pattern_file.read().startswith(core_pattern):
                with open("/proc/sys/kernel/core_pattern", "w") as pattern_w_file:
                    pattern_w_file.write(core_pattern)
        soft, hard = resource.getrlimit(resource.RLIMIT_CORE)
        resource.setrlimit(resource.RLIMIT_CORE, (min(hard, 10*1024*1024), hard))
    except IOError as e:
        if e.errno == errno.EACCES:
            error("Could not change core pattern: not running as root?")
        elif e.errno == errno.ENOENT:
            error("Could not find core pattern file. Probably using a Mac...")
        else:
            raise

if args.kill is not None:
    for p in processes:
        p.kill(args.kill)
elif args.daemonize:
    utils.multitasking.spawnDaemon(monitor)
else:
    monitor()
