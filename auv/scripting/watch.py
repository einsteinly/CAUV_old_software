#!/usr/bin/env python2.7

import os
import sys
import imp
import pwd
import time
import socket
import os.path
import argparse
import threading
import cmd
import readline

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
parser.add_argument("--start",       "-r",  help="Processes to start",                       nargs="+", default=[])
parser.add_argument("--no-node",     "-n",  help="Don't run a CAUV node to start and stop processes", action='store_true')
parser.add_argument("--no-prompt",   "-P",  help="Don't start a prompt for watch control commands", action='store_true')

args = parser.parse_args()

if args.help or not args.session:
    parser.print_help()
    sys.exit(1)

watcher = watch.Watcher(session.get_processes(args), args.core_dumps, args.log_dir, detach = True)

class WatchObserver(messaging.MessageObserver):
    def __init__(self, watcher):
        messaging.MessageObserver.__init__(self)
        self.node = cauv.node.Node('watch')
        #give watcher access to node
        watcher._node = self.node
        self.node.addObserver(self)
        self.watcher = watcher
        self.node.subMessage(messaging.ProcessControlMessage())
        self.node.subMessage(messaging.EditProcessMessage())
        self.node.subMessage(messaging.RequestProcessStatusMessage())
        self.report_all = True #report all on first loop
        
    def onRequestProcessStatusMessage(self, msg):
        #Rebroadcast all process states
        self.report_all = True

    def onProcessControlMessage(self, msg):
        #Check we are the host that should be acting on this message
        if not (msg.host == '*' or msg.host == socket.gethostname()):
            return
        #Check that a valid name was given
        if msg.process not in watcher.processes:
            return
        #now try acting on the specified process
        try:
            if msg.action == messaging.ProcessCommand.Start:
                watcher.start(msg.process)
            elif msg.action == messaging.ProcessCommand.Stop:
                watcher.stop(msg.process)
            elif msg.action == messaging.ProcessCommand.Restart:
                watcher.restart(msg.process)
        except KeyError:
            error(str(watcher.processes))
            error("Process {} does not exist!".format(msg.process))
            
    def onEditProcessMessage(self, msg):
        if not (msg.host == '*' or msg.host == socket.gethostname()):
            return
        #info("Setting process {} to command {} with autostart {}, node id {}, prerequisites {} and restart {}".format(msg.process,
        #                        msg.command,msg.autostart,msg.node_id,msg.prereq,msg.restart))
        if msg.process not in watcher.processes:
            proc = watchfuncs.Process(msg.process, msg.command)
            watcher.add_process(proc)
        else:
            proc = watcher.processes[msg.process].p
            proc.cmd = msg.command
        proc.autostart = msg.autostart
        proc.get_pid = watchfuncs.node_pid(msg.node_id)
        proc.prereq = watchfuncs.depends_on(*msg.prereq)
        if msg.restart == -101: #magic number ahoy!
            proc.death_callback = watchfuncs.report_death
        elif msg.restart < 0:
            proc.death_callback = watchfuncs.restart()
        elif msg.restart == 0:
            proc.death_callback = watchfuncs.ignore
        elif msg.restart > 0:
            proc.death_callback = watchfuncs.restart(msg.restart)

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
                statmsg = messaging.ProcessStatusMessage(socket.gethostname(), process.p.name, True, 'Running', cpu_percent, stats.rss, stats.nthreads)
                self.node.send(statmsg)
            elif process.cause_of_death:
                statmsg = messaging.ProcessStatusMessage(socket.gethostname(), process.p.name, False, process.cause_of_death, 0, 0, 0)
                #reset cause of death once reported
                process.cause_of_death = ""
                self.node.send(statmsg)
            elif self.report_all:
                statmsg = messaging.ProcessStatusMessage(socket.gethostname(), process.p.name, False, 'Not started', 0, 0, 0)
                self.node.send(statmsg)
        self.report_all = False #dont report all until requested to again

def monitor():
    if args.daemonize:
        #redirect output appropriately
        os.chdir("/")
        null_fd = os.open("/dev/null", os.O_RDWR)
        os.dup2(null_fd, sys.stdin.fileno())
        os.dup2(null_fd, sys.stdout.fileno())
        os.dup2(null_fd, sys.stderr.fileno())
    if args.no_node:
        #no access to the messaging system, so don't carry out reporting
        watcher.monitor(args.tick)
    else:
        observer = WatchObserver(watcher)
        watcher.monitor(args.tick, observer.report)
    
if args.kill is not None:
    watcher.killall(args.kill)
elif args.daemonize:
    utils.daemon.spawnDaemon(monitor)
elif args.no_prompt:
    try:
        monitor()
    except KeyboardInterrupt:
        debug("Exiting")
    finally:
        if args.kill_after is not None:
            watcher.killall(args.kill_after)
else:
    try:
        thread = threading.Thread(target = monitor)
        thread.daemon = True
        thread.start() 
        for proc in args.start:
            watcher.start(proc)
        try:
            
            class MyCmd(cmd.Cmd):
                prompt = "> "

                def do_start(self, line):
                    watcher.start(line)
                def do_stop(self, line):
                    watcher.stop(line)
                def do_restart(self, line):
                    watcher.restart(line)
                def do_exit(self, line):
                    return True
                def do_EOF(self, line):
                    return self.do_exit(line)

                def completedefault(self, text, line, start_index, end_index):
                    if text:
                        return [ p for p in watcher.processes.keys() if p.startswith(text) ]
                    else:
                        return watcher.processes.keys()

                def cmdloop(self):
                    while True:
                        try:
                            cmd.Cmd.cmdloop(self)
                            break
                        except KeyboardInterrupt as e:
                            print "^C"

                def emptyline(self):
                    pass

            cmdline = MyCmd()
            cmdline.cmdloop()

        except KeyboardInterrupt:
            debug("Exiting")

    finally:
        if args.kill_after is not None:
            watcher.killall(args.kill_after)
