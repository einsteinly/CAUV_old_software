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
from cauv.debug import debug, info, warning, error

def load_session(filename):
    for suffix, mode, type in imp.get_suffixes():
        if args.session.endswith(suffix):
            with open(args.session, mode) as session_file:
                return imp.load_module("session", session_file, "", (suffix, mode, type))
    raise SessionNotFound()

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

parser.add_argument("--help", "-h", help="print this usage message", action='store_true')
parser.add_argument("--daemonize", "-d", help="run as a daemon", action='store_true')
parser.add_argument("--core-dumps", "-c", help="enable core dumps for started processes", action = 'store_true')
parser.add_argument("--bin-dir", "-b", help="binary files directory")
parser.add_argument("--script-dir", "-p", help="script files directory")
parser.add_argument("--log-dir", "-l", help="log directory for files")

args = parser.parse_args()

if args.help or not args.session:
    parser.print_help()
    sys.exit(1)

class WatchProcess:
    def __init__(self, process):
        self.p = process
        self.pid = None
        self.restarts = 0

    def exec_proc(self):
        format_dict = {'SDIR': args.script_dir, "BDIR": args.bin_dir, "D": ''}
        command = self.p.cmds[0].format(**format_dict)
        work_dir = self.p.work_dir.format(**format_dict)
        debug('running \'{}\' in directory {}'.format(command, work_dir))
        os.chdir(work_dir)

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
                self.restarts += 1
        if self.pid is None:
            self.pid = self.p.pid_func()
            if self.pid is not None:
                info("found PID {} for process {}".format(self.pid, self.p.name))
        if self.pid is None:
            self.pid = utils.multitasking.spawnDaemon(self.exec_proc)


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

if args.daemonize:
    utils.multitasking.spawnDaemon(monitor)
else:
    monitor()
