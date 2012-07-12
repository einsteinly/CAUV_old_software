#!/usr/bin/env python2.7

import sys
import argparse
import imp
import utils.multitasking
import os
import pwd
import utils.watch as watch
from cauv.debug import debug, info, warning, error
from cauv.messaging import debugParseOptions

parser = argparse.ArgumentParser(description = "Start up and monitor CAUV nodes and other programs", add_help = False)

parser.add_argument("--session", "-s", help="session file to use")

args, unknown = parser.parse_known_args()

if args.session:
    session = watch.load_session(args.session)
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
parser.add_argument("--kill-after",  "-K",  help="Kill all processes once finished",         nargs='?', type=int, const=3)
parser.add_argument("--user",        "-u",  help="Default user to run processes as",         default=watch.currentUser())
parser.add_argument("--tick",        "-t",  help="Time between process checks (seconds)",    default=1.0, type=float)

args = parser.parse_args()

if args.help or not args.session:
    parser.print_help()
    sys.exit(1)

if args.user is None:
    try:
        args.user = os.environ["SUDO_USER"]
    except KeyError:
        pass

watcher = watch.Watcher(session.get_processes(args), args.core_dumps, args.log_dir,
                        args.user, args.script_dir, args.bin_dir, detach=False)

def monitor():
    if args.daemonize:
        os.chdir("/")
        null_fd = os.open("/dev/null", os.O_RDWR)
        os.dup2(null_fd, sys.stdin.fileno())
        os.dup2(null_fd, sys.stdout.fileno())
        os.dup2(null_fd, sys.stderr.fileno())

    watcher.monitor(args.tick)
    
if args.kill is not None:
    watcher.kill(args.kill)
elif args.daemonize:
    utils.multitasking.spawnDaemon(monitor)
else:
    try:
        monitor()
    finally:
        if args.kill_after is not None:
            watcher.kill(args.kill_after)
