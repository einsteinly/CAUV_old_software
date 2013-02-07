#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import os
import pwd
import imp
import sys
import time
import errno
import shlex
import os.path
import resource
import traceback
import subprocess
import collections


import utils.daemon
from cauv.debug import debug, info, warning, error

class SessionNotFound(Exception):
    pass

def load_session(filename):
    sys.path.append(os.path.dirname(filename))
    for suffix, mode, type in imp.get_suffixes():
        if filename.endswith(suffix):
            with open(filename, mode) as session_file:
                return imp.load_module("session", session_file, "/tmp/" + session_file.name, (suffix, mode, type))
    raise SessionNotFound("Could not find session {}".format(filename))

class StateType(object):
    def __hash__(self):
        return hash(id(self))

Stopped    = StateType()
Starting   = StateType()
Running    = StateType()
Restarting = StateType()
Stopping   = StateType()
Zombie     = StateType()

class WatchProcess:
    def __init__(self, process, watcher, detach):
        self.p = process
        if self.p.autostart:
            self.state = Starting
        else:
            self.state = Stopped
        self.pid = None
        self.proc = None
        self.watcher = watcher
        self.sigs = []

    def stopped(self):
        return

    def starting(self):
        pid = self.p.get_pid()
        if pid is not None:
            self.pid = pid
            info("found PID {} for process {}".format(self.pid, self.p.name))
            self.state = Running
            return
        if self.p.prereq(self):
            try:
                with open(os.devnull, "rw") as nullf:
                    self.proc = subprocess.Popen(self.p.cmd, stdout = nullf,
                                                             stdin = nullf,
                                                             stderr = nullf)
            except Exception:
                error("Error starting up process {}".format(self.p.name))
                error(traceback.format_exc().encode('ascii', 'replace'))
                self.restart = False
                return

            self.pid = self.proc.pid
            info("Started {} with pid {}".format(self.p.name, self.pid))
            self.state = Running

    def running(self):
        if self.pid_running():
            return
        self.pid = None
        self.state = Stopped
        warning("Process {} died!".format(self.p.name))
        try:
            self.p.death_callback(self)
        except Exception:
            error("Error in process {} death_callback:".format(self.p.name))
            error(traceback.format_exc().encode('ascii', 'replace'))
            error("Not restarting {}".format(self.p.name))
            self.state = Stopped

    def pid_running(self):
        if self.pid is None:
            return False
        try:
            os.kill(self.pid, 0)
            return True
        except OSError as e:
            if e.errno == errno.ESRCH:
                return False
            else:
                raise

    def stopping(self):
        if not self.pid_running():
            if self.state == Restarting:
                self.p.restart_callback(self)
                self.state = Starting
            else:
                self.state = Stopped
            return
        try:
            sig = self.sigs.pop(0)
        except IndexError:
            error("No more signals to try for killing {}. Giving up!".format(self.p.name))
            self.state = Zombie
            return
        try:
            info("Killing process {} with signal {}".format(self.p.name, sig))
            os.kill(self.pid, sig)
        except OSError:
            error("Error killing process {}: {}".format(self.p.name,
                    traceback.format_exc().encode('ascii', 'replace')))

    def tick(self):
        {
            Stopped : self.stopped,
            Starting : self.starting,
            Running : self.running,
            Restarting : self.stopping,
            Stopping : self.stopping,
            Zombie : self.stopped,
        }[self.state]()
        if self.proc:
            self.proc.poll()

    def start(self):
        #if already running, dont run
        if self.state in (Starting, Running):
            warning("{} is already running".format(self.p.name))
            return
        self.state = Starting

    def stop(self, sigs):
        if self.state not in (Starting, Running):
            warning("Asked to stop {} which is not running".format(self.p.name))
            return
        info("Stopping {}".format(self.p.name))
        self.sigs = sigs
        if self.state == Starting:
            self.state = Stopped
            return
        self.state = Stopping

    def restart(self, sigs = []):
        if self.state == Zombie:
            warning("Not restarting zombie process {}".format(self.p.name))
            return
        self.sigs = sigs
        self.state = Restarting
        
    def kill(self, signal):
        #if we call this we also want to kill any leftover linked processes
        self.stop(signal)
        last_pid = -1
        while True:
            pid = self.p.get_pid()
            if last_pid == pid:
                warning("Unkillable process {}, pid {}, ignoring".format(self.p.name, pid))
                break
            if pid is not None:
                info("Found {}, pid {}, stopping".format(self.p.name, pid))
                os.kill(pid, signal)
            else:
                break
            last_pid = pid

def setup_core_dumps():
    core_pattern = "/var/tmp/cauv_corefiles/%e.%d.%t.%p"
    try:
        try:
            os.mkdir("/var/tmp/cauv_corefiles", 0777)
        except OSError as e:
            if e.errno != errno.EEXIST:
                error("Could not create corefile directory")
                raise
        with open("/proc/sys/kernel/core_pattern", "r") as pattern_file:
            if not pattern_file.read().startswith(core_pattern):
                with open("/proc/sys/kernel/core_pattern", "w") as pattern_w_file:
                    pattern_w_file.write(core_pattern)
        soft, hard = resource.getrlimit(resource.RLIMIT_CORE)
        resource.setrlimit(resource.RLIMIT_CORE, (min(hard, 10*1024*1024), hard)) #10MB max
    except IOError as e:
        if e.errno == errno.EACCES:
            error("Could not change core pattern: not running as root?")
        elif e.errno == errno.ENOENT:
            error("Could not find core pattern file. Probably using a Mac...")
        else:
            raise
    
class Watcher:
    def __init__(self, processes = None, core_dumps = False, log_dir = None, detach = True):
        self.detach = detach
        if core_dumps:
            setup_core_dumps()
        if processes is None:
            processes = []
        if log_dir:
            os.environ["CAUV_LOG_DIR"] = log_dir

        self.processes = {p.name : WatchProcess(p, self, detach) for p in processes}

    def monitor(self, tick_time = 1, tick_cb = None):
        while True:
            for p in self.processes.values():
                p.tick()
            if tick_cb is not None:
                tick_cb()
            time.sleep(tick_time)

    def add_process(self, process):
        self.processes[process.name] = WatchProcess(process, self, self.detach)

    def stop(self, process, signals=None):
        if signals is None:
            signals = [15,0,0,0,9]
        self.processes[process].stop(signals)

    def start(self, process):
        self.processes[process].start()

    def restart(self, process, signals=None):
        if signals is None:
            signals = [15,0,0,0,9]
        self.processes[process].restart(signals)

    def is_running(self, process):
        try:
            return self.processes[process].state == Running
        except KeyError:
            return False

    def killall(self, signal=15):
        for p in self.processes.values():
            p.kill(signal)

