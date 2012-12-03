#!/usr/bin/env python2.7

import os
import sys
import imp
import pwd
import time
import socket
import os.path
import argparse

import utils.daemon
import utils.watch as watch
from cauv.debug import debug, info, warning, error
import cauv.messaging as messaging
import cauv.node
import utils.dirs
import utils.watchfuncs as watchfuncs

try:
    import psi.process
except ImportError:
    psi = None
    warning("Couldn't load psi, no process state reporting!")

parser = argparse.ArgumentParser(description = "Start up and monitor CAUV nodes and other programs", add_help = False)

parser.add_argument("--session", "-s", help="session file to use")

args, unknown = parser.parse_known_args()

if args.session:
    if not os.path.exists(args.session):
        args.session = utils.dirs.config_dir(args.session)
    session = watch.load_session(args.session)
    try:
        arg_group = parser.add_argument_group(title=args.session)
        session.get_arguments(arg_group)
    except AttributeError:
        pass
    except NameError:
        pass

parser.add_argument("--help",        "-h",  help="print this usage message, and help for any session file", action='store_true')
parser.add_argument("--daemonize",   "-d",  help="run as a daemon",                          action='store_true')
parser.add_argument("--core-dumps",  "-c",  help="enable core dumps for started processes",  action='store_true')
parser.add_argument("--log-dir",     "-l",  help="log directory for files")
parser.add_argument("--kill",        "-k",  help="Kill all processes in session",            nargs='?', type=int, const=15)
parser.add_argument("--kill-after",  "-K",  help="Kill all processes once finished",         nargs='?', type=int, const=3)
parser.add_argument("--tick",        "-t",  help="Time between process checks (seconds)",    default=1.0, type=float)
#parser.add_argument("--start",       "-r",  help="Processes to start",                       nargs="+", action='append')
parser.add_argument("--no-node",     "-n",  help="Don't run a CAUV node to start and stop processes", action='store_true')

args = parser.parse_args()

if args.help or not args.session:
    parser.print_help()
    sys.exit(1)

watcher = watch.Watcher(session.get_processes(args), args.core_dumps, args.log_dir, detach = True)

class WatchObserver(messaging.MessageObserver):
    def __init__(self, watcher):
        messaging.MessageObserver.__init__(self)
        self.node = cauv.node.Node('watch')
        self.node.addObserver(self)
        self.watcher = watcher
        self.node.subMessage(messaging.ProcessControlMessage())

    def onProcessControlMessage(self, msg):
        if not (msg.host == '*' or msg.host == socket.gethostname()):
            return
        if msg.command and msg.process not in watcher.processes:
            proc = watchfuncs.Process(msg.process, msg.command)
            watcher.add_process(proc)
        try:
            if msg.action == messaging.ProcessCommand.Start:
                watcher.start(msg.process)
            elif msg.action == messaging.ProcessCommand.Stop:
                watcher.stop(msg.process)
            elif msg.action == messaging.ProcessCommand.Restart:
                watcher.restart(msg.process)
        except KeyError:
            error("Process {} does not exist!".format(msg.process))

    def report(self):
        if psi is None:
            return
        for process in watcher.processes.values():
            if process.state == utils.watch.Running:
                try:
                    stats = psi.process.Process(process.pid)
                except psi.process.NoSuchProcessError:
                    warning("Process {} dissapeared when looking for stats".format(process.p.name))
                    continue
                curr_time, cputime = time.time(), stats.cputime.float()
                try:
                    last_time, last_cputime = process.__last_times
                    cpu_percent = (cputime - last_cputime) / (curr_time - last_time)
                except AttributeError:
                    cpu_percent = 0
                    pass
                process.__last_times = (curr_time, cputime)
                statmsg = messaging.ProcessStatusMessage(process.p.name, 'Running', cpu_percent, stats.rss, stats.nthreads)
                self.node.send(statmsg)
            else:
                pass

def monitor():
    if args.daemonize:
        os.chdir("/")
        null_fd = os.open("/dev/null", os.O_RDWR)
        os.dup2(null_fd, sys.stdin.fileno())
        os.dup2(null_fd, sys.stdout.fileno())
        os.dup2(null_fd, sys.stderr.fileno())
    if args.no_node:
        watcher.monitor(args.tick)
    else:
        observer = WatchObserver(watcher)
        watcher.monitor(args.tick, observer.report)
    
if args.kill is not None:
    watcher.killall(args.kill)
elif args.daemonize:
    utils.daemon.spawnDaemon(monitor)
else:
    try:
        monitor()
    finally:
        if args.kill_after is not None:
            watcher.killall(args.kill_after)
